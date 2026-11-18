#include "m10_receiver.hh"

#include "comms.hh"
#include "gnss_interface.hh"
#include "hal.hh"

void M10Receiver::Reset() {
    phase_ = Phase::probe;
    next_action_us_ = 0;
    baud_confirmed_ = false;
    baud_rejected_ = false;
    probe_index_ = 0;
#ifdef AIRGREP
    profile_index_ = batch_count_ = single_key_until_ = 0;
#endif
    parser_.Resync();
    version_ = {};
    rf_ = {};
    sbas_ = {};
    satellites_ = {};
    comms_.valid = hw3_.valid = sbas_in_bbr_ = false;
}

void M10Receiver::Configure(SettingsManager::GNSSAntennaMode antenna, bool diagnostics, uint32_t baud) {
    if (antenna == antenna_ && diagnostics == diagnostics_ && target_baud_ == baud) return;
    antenna_ = antenna;
    diagnostics_ = diagnostics;
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
    const uint64_t deadline = get_time_since_boot_us() + 6000000;
    while (get_time_since_boot_us() < deadline && port_.service_during_poll_() && port_.uart_) {
        port_.Read();
        Update(get_time_since_boot_us());
        if (baud_confirmed_) return true;
        if (baud_rejected_) break;
    }
    target_baud_ = previous;
    Reset();  // Rediscover the module even if it changed its RAM rate immediately before timeout or SYNC.
    return false;
}

void M10Receiver::Retry() {
    Reset();
    next_action_us_ = get_time_since_boot_us() + 1000000;
}

void M10Receiver::RejectBaud() {
    Retry();
    baud_rejected_ = true;
}

bool M10Receiver::SendConfiguration() {
#ifdef AIRGREP
    uint8_t payload[GNSSReceiver::kMaxTxFrameSize - 8];
    uint16_t length = BeginCfgValset(payload, kCfgLayerRam | kCfgLayerBbr);
    airgrep::WriteReceiverSettings(antenna_, [&](uint32_t key, uint16_t value, uint8_t size) {
        length = AppendCfgKeyValue(payload, length, key, value, size);
    });
#else
    const bool supervised = antenna_ == SettingsManager::kGNSSAntennaActiveSupervised;
    struct Entry { uint32_t key; uint16_t value; uint8_t size = 1; };
    const Entry entries[] = {
        {kCfgUart1OutNmea, 1}, {kCfgUart1OutUbx, 1}, {kCfgRateMeas, 1000, 2}, {kCfgRateNav, 1, 2},
        {kCfgMsgoutUbxNavSatUart1, kDiagnosticsEpochDivider},
        {kCfgSignalSbas, 1}, {kCfgItfmAntSetting, AntennaSetting(antenna_)},
        {kCfgHwAntCfgVoltCtrl, supervised}, {kCfgHwAntCfgShortDet, supervised}, {kCfgHwAntCfgOpenDet, supervised},
        {kCfgHwAntCfgPowerDown, supervised}, {kCfgHwAntCfgRecover, supervised},
    };
    const Entry supervisor_pins[] = {{kCfgHwAntSupShortPin, 3}, {kCfgHwAntSupOpenPin, 2}};
    constexpr size_t kSelectedMessages = [] {
        size_t count = 0;
        WriteMessageSelection(false, [&](uint32_t, uint8_t) { ++count; });
        return count;
    }();
    uint8_t payload[4 + 2 + 5 * (kSelectedMessages + sizeof(entries) / sizeof(Entry) +
                                 sizeof(supervisor_pins) / sizeof(Entry))];
    static_assert(sizeof(payload) + 8 <= GNSSReceiver::kMaxTxFrameSize);
    uint16_t length = BeginCfgValset(payload, kCfgLayerRam);
    auto append = [&](Entry entry) {
        length = AppendCfgKeyValue(payload, length, entry.key, entry.value, entry.size);
    };
    WriteMessageSelection(diagnostics_, [&](uint32_t key, uint8_t rate) { append({key, rate}); });
    for (const auto& entry : entries) append(entry);
    if (supervised) for (const auto& entry : supervisor_pins) append(entry);
#endif
    return port_.SendUbx(kUbxClassCfg, kUbxIdCfgValset, payload, length);
}

#ifdef AIRGREP
uint8_t M10Receiver::LoadBatch(airgrep::GNSSMessageRate (&entries)[kMessageBatch]) const {
    const uint8_t limit = profile_index_ < single_key_until_ ? 1 :
                          profile_index_ == 0 ? airgrep::kBootstrapMessageCount : kMessageBatch;
    return airgrep::LoadMessages(profile_index_, limit, diagnostics_, entries);
}

bool M10Receiver::SendProfile(bool readback) {
    airgrep::GNSSMessageRate entries[kMessageBatch];
    batch_count_ = LoadBatch(entries);
    uint8_t payload[4 + 5 * kMessageBatch];
    if (readback) {
        return port_.SendUbx(kUbxClassCfg, kUbxIdCfgValget, payload,
                             airgrep::WriteMessageKeys(payload, entries, batch_count_));
    }
    return port_.SendUbx(kUbxClassCfg, kUbxIdCfgValset, payload,
                         airgrep::WriteMessageRates(payload, kCfgLayerRam | kCfgLayerBbr, entries, batch_count_));
}

void M10Receiver::NextBatch() {
    profile_index_ += batch_count_;
    phase_ = profile_index_ == airgrep::kGNSSMessageCount ? Phase::configure : Phase::profile_set;
    next_action_us_ = 0;
}

void M10Receiver::RejectBatch() {
    if (batch_count_ > 1) {
        single_key_until_ = profile_index_ + batch_count_;
        phase_ = Phase::profile_set;
        next_action_us_ = 0;
    } else {
        airgrep::GNSSMessageRate entry[kMessageBatch];
        LoadBatch(entry);
        if (entry[0].rate == 0) NextBatch();
        else Retry();
    }
}
#endif

void M10Receiver::Update(uint64_t now_us) {
    if (now_us < next_action_us_) return;
    if (phase_ == Phase::ready) {
        if (rf_.IsFresh(now_us / 1000)) return;
        Reset();  // A receiver that power cycled has lost its RAM settings.
    }
    switch (phase_) {
        case Phase::probe:
            if (port_.SendUbx(M10Diagnostics::kClassMon, M10Diagnostics::kIdMonVer)) {
                phase_ = Phase::await_probe;
                next_action_us_ = now_us + 500000;
            }
            break;
        case Phase::await_probe:
            phase_ = Phase::next_rate;
            break;
        case Phase::next_rate: {
            constexpr auto& rates = SettingsManager::kGNSSBaudRates;
            constexpr size_t count = sizeof(rates) / sizeof(rates[0]);
            const uint32_t rate = rates[probe_index_];
            if (port_.SwitchBaudrate(rate)) {
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
                next_action_us_ = now_us + 100000;
            }
            break;
        case Phase::switch_baud:
            if (port_.SwitchBaudrate(target_baud_)) phase_ = Phase::probe;
            break;
        case Phase::check_baud:
        case Phase::verify_baud: {
            if (RequestBackupValue(kCfgUart1Baudrate)) {
                phase_ = phase_ == Phase::check_baud ? Phase::await_saved_rate : Phase::await_verified_rate;
                next_action_us_ = now_us + 500000;
            }
            break;
        }
        case Phase::await_saved_rate:
            phase_ = Phase::save_baud;
            break;
        case Phase::save_baud:
            if (SendBaud(kCfgLayerBbr)) {
                phase_ = Phase::await_baud_ack;
                next_action_us_ = now_us + 1000000;
            }
            break;
        case Phase::read_backup:
            if (RequestBackupValue(kCfgSignalSbas)) {
#ifdef AIRGREP
                phase_ = Phase::profile_set;
#else
                phase_ = Phase::configure;
#endif
                next_action_us_ = now_us + 200000;
            }
            break;
#ifdef AIRGREP
        case Phase::profile_set:
        case Phase::profile_get: {
            const bool readback = phase_ == Phase::profile_get;
            if (SendProfile(readback)) {
                phase_ = readback ? Phase::await_profile_values : Phase::await_profile_ack;
                next_action_us_ = now_us + 1500000;
            }
            break;
        }
        case Phase::await_profile_ack:
        case Phase::await_profile_values:
#endif
        case Phase::await_ack:
            Retry();
            break;
        case Phase::await_baud_ack:
        case Phase::await_verified_rate:
            RejectBaud();
            break;
        case Phase::configure:
            if (SendConfiguration()) {
                phase_ = Phase::await_ack;
                next_action_us_ = now_us + 1000000;
            }
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

void M10Receiver::OnVersion(const M10Diagnostics::Version& version) {
    version_ = version;
    if (phase_ == Phase::await_probe) {
        phase_ = port_.baud_ == target_baud_ ? Phase::check_baud : Phase::set_baud;
        next_action_us_ = 0;
    }
}

void M10Receiver::OnUbxMessage(const UbxMessage& msg) {
    if (M10Diagnostics::Decode(msg, *this)) return;
    const bool valget = msg.is_message_type(kUbxClassCfg, kUbxIdCfgValget);
    const bool set_ack = IsAckFor(msg, kUbxClassCfg, kUbxIdCfgValset);
    const bool get_nak = IsAckFor(msg, kUbxClassCfg, kUbxIdCfgValget) && msg.id == kUbxIdAckNak;
    uint64_t value = 0;
    if (valget && ReadCfgValue(msg.payload, msg.length, kCfgGetLayerBbr, kCfgSignalSbas, 1, value)) {
        sbas_in_bbr_ = value != 0;
    }
    if (phase_ == Phase::await_saved_rate || phase_ == Phase::await_verified_rate) {
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
    } else if (phase_ == Phase::await_baud_ack && set_ack) {
        if (msg.id == kUbxIdAckAck) {
            phase_ = Phase::verify_baud;
            next_action_us_ = 0;
        } else {
            RejectBaud();
        }
    }
#ifdef AIRGREP
    else if (phase_ == Phase::await_profile_ack && set_ack) {
        if (msg.id == kUbxIdAckNak) RejectBatch();
        else { phase_ = Phase::profile_get; next_action_us_ = 0; }
    } else if (phase_ == Phase::await_profile_values) {
        if (get_nak) RejectBatch();
        if (valget && msg.stored == msg.length) {
            airgrep::GNSSMessageRate entries[kMessageBatch];
            const auto count = LoadBatch(entries);
            const auto result = airgrep::CheckMessageReadback(entries, count, msg.payload, msg.length);
            if (result == airgrep::Readback::verified) NextBatch();
            else if (result == airgrep::Readback::mismatch) Retry();
        }
    }
#endif
    else if (phase_ == Phase::await_ack && set_ack) {
        if (msg.id == kUbxIdAckAck) {
            phase_ = Phase::ready;
            next_action_us_ = get_time_since_boot_us() + 15000000;
        } else {
            CONSOLE_WARNING("M10Receiver", "GNSS rejected its runtime configuration.");
            Retry();
        }
    }
}

void M10Receiver::OnInfo(bool error, const char* text, uint16_t length, bool) {
    if (!diagnostics_) return;
    if (error) {
        CONSOLE_ERROR("M10Receiver", "%.*s", length, text);
    } else {
        CONSOLE_WARNING("M10Receiver", "%.*s", length, text);
    }
}

void M10Receiver::OnRf(const M10Diagnostics::RfStatus& rf) {
    rf_ = {true, receive_ms_, rf};
    if (diagnostics_) {
        CONSOLE_INFO("M10Receiver", "jamming=%s antenna=%s power=%s post=0x%08lX noise=%u agc=%u cw=%u I=%d/%u Q=%d/%u",
                     M10Strings::Jamming(rf.jamming), M10Strings::AntennaStatus(rf.antenna_status),
                     M10Strings::AntennaPower(rf.antenna_power), static_cast<unsigned long>(rf.post_status),
                     rf.noise, rf.agc, rf.cw_suppression, rf.offset_i, rf.magnitude_i, rf.offset_q, rf.magnitude_q);
    }
}

bool M10Receiver::Poll(uint8_t id, bool& received) {
    if (!IsHealthy()) return false;
    const uint64_t deadline_us = get_time_since_boot_us() + 1500000;
    bool sent = false;
    while (get_time_since_boot_us() < deadline_us && port_.service_during_poll_()) {
        port_.Read();
        if (!sent) {
            received = false;
            sent = port_.SendUbx(M10Diagnostics::kClassMon, id);
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
