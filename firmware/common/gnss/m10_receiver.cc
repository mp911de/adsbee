#include "m10_receiver.hh"

#include "comms.hh"  // For CONSOLE_* logging.
#include "hal.hh"

namespace {
constexpr uint64_t kProbeTimeoutUs = 500000;
constexpr uint64_t kReplyTimeoutUs = 1500000;
constexpr uint64_t kBaudAckTimeoutUs = 1000000;
constexpr uint64_t kRetryDelayUs = 1000000;
constexpr uint64_t kBaudSwitchSettleUs = 100000;  // Integration manual: about 100 ms before input at the new rate.
constexpr uint64_t kBackupReplyUs = 200000;
constexpr uint64_t kFirstRfUs = 15000000;  // Time a configured receiver gets to send its first UBX-MON-RF.
constexpr uint64_t kChangeBaudrateTimeoutUs = 6000000;
constexpr uint64_t kPollTimeoutUs = 1500000;
}  // namespace

void M10Receiver::Reset() {
    phase_ = Phase::probe;
    next_action_us_ = 0;
    baud_confirmed_ = false;
    baud_rejected_ = false;
    probe_index_ = 0;
    batch_first_ = batch_total_ = single_item_until_ = 0;
    batch_count_ = 0;
    batch_written_ = false;
    parser_.Resync();
    version_ = {};
    rf_ = {};
    sbas_ = {};
    satellites_ = {};
    comms_.valid = hw3_.valid = sbas_in_bbr_ = false;
}

void M10Receiver::Configure(const M10Options& options, uint32_t baud) {
    if (options == options_ && target_baud_ == baud) return;
    options_ = options;
    target_baud_ = baud;
    Reset();
}

bool M10Receiver::SendBaud(uint8_t layers) {
    uint8_t payload[12];
    const auto len = AppendCfgKeyValue(payload, BeginCfgValset(payload, layers), kCfgUart1Baudrate, target_baud_, 4);
    return port_.SendUbx(kUbxClassCfg, kUbxIdCfgValset, payload, len);
}

bool M10Receiver::RequestBackupValue(uint32_t key) {
    uint8_t request[8];
    AppendCfgKeyValue(request, BeginCfgValget(request, kCfgGetLayerBbr), key, 0, 0);
    return port_.SendUbx(kUbxClassCfg, kUbxIdCfgValget, request, sizeof(request));
}

bool M10Receiver::ChangeBaudrate(uint32_t baud) {
    if (baud == target_baud_ && baud_confirmed_) return true;
    const uint32_t previous = target_baud_;
    target_baud_ = baud;
    Reset();
    const uint64_t deadline = get_time_since_boot_us() + kChangeBaudrateTimeoutUs;
    while (get_time_since_boot_us() < deadline && port_.Service()) {
        Update(get_time_since_boot_us());
        if (baud_confirmed_) return true;
        if (baud_rejected_) break;
    }
    target_baud_ = previous;
    Reset();  // Find the receiver again even if it changed its RAM rate right before the wait ended.
    return false;
}

void M10Receiver::Retry() {
    Reset();
    next_action_us_ = get_time_since_boot_us() + kRetryDelayUs;
}

void M10Receiver::RejectBaud() {
    Retry();
    baud_rejected_ = true;
}

M10ConfigBatch::Window M10Receiver::BatchWindow() const {
    const uint8_t max_items = batch_first_ < single_item_until_ ? 1 : M10ConfigBatch::kMaxItems;
    return {batch_first_, max_items, max_payload_};
}

bool M10Receiver::SendBatch() {
    uint8_t payload[kMaxPayload];
    const bool set = phase_ == Phase::batch_set;
    M10ConfigBatch batch = set ? M10ConfigBatch::Values(BatchWindow(), payload, profile_.layers)
                               : M10ConfigBatch::Keys(BatchWindow(), payload);
    profile_.write(options_, batch);
    batch_count_ = batch.count();
    batch_total_ = batch.total();
    if (batch_count_ == 0) {
        BecomeReady();
        return false;
    }
    return port_.SendUbx(kUbxClassCfg, set ? kUbxIdCfgValset : kUbxIdCfgValget, payload, batch.length());
}

void M10Receiver::OnBatchValues(const UbxMessage& msg) {
    M10ConfigBatch batch = M10ConfigBatch::Compare(BatchWindow(), msg.payload, msg.length);
    profile_.write(options_, batch);
    switch (batch.readback()) {
        case M10ConfigBatch::Readback::kUnrelated:
            break;
        case M10ConfigBatch::Readback::kMatch:
            NextBatch();
            break;
        case M10ConfigBatch::Readback::kMismatch:
            if (batch_written_) {
                Retry();
            } else {
                phase_ = Phase::batch_set;
                next_action_us_ = 0;
            }
            break;
    }
}

void M10Receiver::NextBatch() {
    batch_first_ += batch_count_;
    batch_written_ = false;
    next_action_us_ = 0;
    if (batch_first_ >= batch_total_) {
        BecomeReady();
    } else {
        phase_ = Phase::batch_get;
    }
}

void M10Receiver::RejectBatch() {
    if (batch_count_ > 1) {
        single_item_until_ = batch_first_ + batch_count_;
        batch_written_ = false;
        phase_ = Phase::batch_get;
        next_action_us_ = 0;
        return;
    }
    M10ConfigBatch item = M10ConfigBatch::Count(BatchWindow());
    profile_.write(options_, item);
    CONSOLE_WARNING("M10Receiver", "GNSS rejected configuration key 0x%08lx; skipping it.",
                    static_cast<unsigned long>(item.first_key()));
    NextBatch();
}

void M10Receiver::BecomeReady() {
    phase_ = Phase::ready;
    next_action_us_ = get_time_since_boot_us() + kFirstRfUs;
    CONSOLE_INFO("M10Receiver", "GNSS module configured at %lu baud.", static_cast<unsigned long>(target_baud_));
}

void M10Receiver::Update(uint64_t now_us) {
    if (now_us < next_action_us_) return;
    if (phase_ == Phase::ready) {
        if (rf_.IsFresh(now_us / 1000)) return;
        Reset();  // A receiver that power cycled has lost its RAM settings.
    }
    switch (phase_) {
        case Phase::probe:
            if (port_.SendUbx(M10Diagnostics::kClassMon, M10Diagnostics::kIdMonVer, nullptr, 0)) {
                phase_ = Phase::await_probe;
                next_action_us_ = now_us + kProbeTimeoutUs;
            }
            break;
        case Phase::await_probe:
            phase_ = Phase::next_rate;
            break;
        case Phase::next_rate: {
            constexpr auto& rates = GNSSSettingsOptions::kGNSSBaudRates;
            constexpr uint8_t count = sizeof(rates) / sizeof(rates[0]);
            if (port_.SwitchBaudrate(rates[probe_index_])) {
                probe_index_ = (probe_index_ + 1) % count;
                phase_ = Phase::probe;
            }
            break;
        }
        case Phase::set_baud:
            if (SendBaud(kCfgLayerRam)) phase_ = Phase::drain_baud;
            break;
        case Phase::drain_baud:
            if (!port_.Transmitting()) {
                phase_ = Phase::switch_baud;
                next_action_us_ = now_us + kBaudSwitchSettleUs;
            }
            break;
        case Phase::switch_baud:
            if (port_.SwitchBaudrate(target_baud_)) phase_ = Phase::probe;
            break;
        case Phase::check_baud:
        case Phase::verify_baud:
            if (RequestBackupValue(kCfgUart1Baudrate)) {
                phase_ = phase_ == Phase::check_baud ? Phase::await_saved_rate : Phase::await_verified_rate;
                next_action_us_ = now_us + kProbeTimeoutUs;
            }
            break;
        case Phase::await_saved_rate:
            phase_ = Phase::save_baud;
            break;
        case Phase::save_baud:
            if (SendBaud(kCfgLayerBbr)) {
                phase_ = Phase::await_baud_ack;
                next_action_us_ = now_us + kBaudAckTimeoutUs;
            }
            break;
        case Phase::await_baud_ack:
        case Phase::await_verified_rate:
            RejectBaud();
            break;
        case Phase::read_backup:
            if (RequestBackupValue(kCfgSignalSbas)) {
                phase_ = Phase::batch_get;
                next_action_us_ = now_us + kBackupReplyUs;
            }
            break;
        case Phase::batch_get:
        case Phase::batch_set: {
            const Phase await = phase_ == Phase::batch_get ? Phase::await_batch_values : Phase::await_batch_ack;
            if (SendBatch()) {
                phase_ = await;
                next_action_us_ = now_us + kReplyTimeoutUs;
            }
            break;
        }
        case Phase::await_batch_values:
        case Phase::await_batch_ack:
            Retry();
            break;
        case Phase::ready:
            break;
    }
}

bool M10Receiver::Ingest(uint8_t byte, uint32_t now_ms) {
    const bool nmea = !parser_.InFrame() && byte != UbxParser::kSync1;
    receive_ms_ = now_ms;
    parser_.IngestByte(byte, now_ms);
    return nmea;
}

void M10Receiver::OnUbxMessage(const UbxMessage& msg) {
    if (M10Diagnostics::Decode(msg, *this)) return;
    const bool valget = msg.is_message_type(kUbxClassCfg, kUbxIdCfgValget) && msg.stored == msg.length;
    const bool set_ack = IsAckFor(msg, kUbxClassCfg, kUbxIdCfgValset);
    const bool get_nak = IsAckFor(msg, kUbxClassCfg, kUbxIdCfgValget) && msg.id == kUbxIdAckNak;
    uint64_t value = 0;
    if (valget && ReadCfgValue(msg.payload, msg.length, kCfgGetLayerBbr, kCfgSignalSbas, 1, value)) {
        sbas_in_bbr_ = value != 0;
    }
    switch (phase_) {
        case Phase::await_saved_rate:
        case Phase::await_verified_rate:
            if (valget && ReadCfgValue(msg.payload, msg.length, kCfgGetLayerBbr, kCfgUart1Baudrate, 4, value)) {
                if (value == target_baud_) {
                    baud_confirmed_ = true;
                    phase_ = Phase::read_backup;
                } else if (phase_ == Phase::await_saved_rate) {
                    phase_ = Phase::save_baud;
                } else {
                    RejectBaud();
                    return;
                }
                next_action_us_ = 0;
            } else if (get_nak && phase_ == Phase::await_saved_rate) {
                phase_ = Phase::save_baud;
                next_action_us_ = 0;
            }
            break;
        case Phase::await_baud_ack:
            if (!set_ack) break;
            if (msg.id == kUbxIdAckAck) {
                phase_ = Phase::verify_baud;
                next_action_us_ = 0;
            } else {
                RejectBaud();
            }
            break;
        case Phase::await_batch_values:
            if (get_nak) {
                RejectBatch();
            } else if (valget) {
                OnBatchValues(msg);
            }
            break;
        case Phase::await_batch_ack:
            if (!set_ack) break;
            if (msg.id == kUbxIdAckAck) {
                batch_written_ = true;
                phase_ = Phase::batch_get;
                next_action_us_ = 0;
            } else {
                RejectBatch();
            }
            break;
        default:
            break;
    }
}

void M10Receiver::OnVersion(const M10Diagnostics::Version& version) {
    version_ = version;
    if (phase_ == Phase::await_probe) {
        phase_ = port_.baudrate() == target_baud_ ? Phase::check_baud : Phase::set_baud;
        next_action_us_ = 0;
    }
}

void M10Receiver::OnInfo(bool error, const char* text, uint16_t length, bool truncated) {
    if (observer_) observer_->OnInfo(error, text, length, truncated);
    if (!options_.diagnostics) return;
    if (error) {
        CONSOLE_ERROR("M10Receiver", "%.*s", length, text);
    } else {
        CONSOLE_WARNING("M10Receiver", "%.*s", length, text);
    }
}

void M10Receiver::OnRf(const M10Diagnostics::RfStatus& rf) {
    rf_ = {true, receive_ms_, rf};
    if (observer_) observer_->OnRf(rf);
    if (options_.diagnostics) {
        CONSOLE_INFO("M10Receiver", "jamming=%s antenna=%s power=%s post=0x%08lX noise=%u agc=%u cw=%u I=%d/%u Q=%d/%u",
                     M10Strings::Jamming(rf.jamming), M10Strings::AntennaStatus(rf.antenna_status),
                     M10Strings::AntennaPower(rf.antenna_power), static_cast<unsigned long>(rf.post_status),
                     rf.noise, rf.agc, rf.cw_suppression, rf.offset_i, rf.magnitude_i, rf.offset_q, rf.magnitude_q);
    }
}

bool M10Receiver::Poll(uint8_t id, bool& received) {
    if (!IsHealthy()) return false;
    const uint64_t deadline_us = get_time_since_boot_us() + kPollTimeoutUs;
    bool sent = false;
    while (get_time_since_boot_us() < deadline_us && port_.Service()) {
        if (!sent) {
            received = false;
            sent = port_.SendUbx(M10Diagnostics::kClassMon, id, nullptr, 0);
        } else if (received) {
            return true;
        }
    }
    return false;
}

bool M10Receiver::PollComms(M10Diagnostics::CommsStatus& out) {
    if (!Poll(M10Diagnostics::kIdMonComms, comms_.valid)) return false;
    out = comms_.value;
    return true;
}

bool M10Receiver::PollHw3(M10Diagnostics::Hw3Status& out) {
    if (!Poll(M10Diagnostics::kIdMonHw3, hw3_.valid)) return false;
    out = hw3_.value;
    return true;
}
