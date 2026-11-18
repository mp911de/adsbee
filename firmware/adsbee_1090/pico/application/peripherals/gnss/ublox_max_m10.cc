#include "ublox_max_m10.hh"

#include <cstring>

#ifdef AIRGREP
#include "airgrep_m10_readback.hh"
#endif
#include "comms.hh"  // For comms_manager (UART I/O) and CONSOLE_* logging.
#include "hal.hh"    // For get_time_since_boot_ms().
#include "hardware/uart.h"
#include "hardware/watchdog.h"  // For watchdog_update() during the blocking init/config pass.
#include "pico/time.h"

void UbloxMAXM10::SendUbxFrame(uint8_t msg_class, uint8_t msg_id, const uint8_t* payload, uint16_t payload_len) {
    WriteUbxFrame(msg_class, msg_id, payload, payload_len, [](uint8_t byte) {
        comms_manager.iface_putc(SettingsManager::kGNSSUART, static_cast<char>(byte));
    });
}

bool UbloxMAXM10::ScanForUbxMessage(uint8_t want_class, uint8_t want_id, uint32_t timeout_ms, uint8_t* out_payload,
                                    uint16_t out_cap, uint16_t* out_len, bool* is_ack, bool* config_nak) {
    if (config_nak != nullptr) *config_nak = false;
    scan_ = {true, false, want_class, want_id, out_payload, out_cap, out_len, is_ack, config_nak};
    uint32_t start_ms = get_time_since_boot_ms();
    char c;
    while (!scan_.done && get_time_since_boot_ms() - start_ms < timeout_ms) {
#ifdef AIRGREP
        watchdog_update();
#endif
        if (!ReadBufferedByte(c)) {
            PollUartIntoRxBuffer();
            continue;  // No byte available yet; keep polling until timeout.
        }
        bool nmea = !ubx_parser_.InFrame() && static_cast<uint8_t>(c) != UbxParser::kSync1;
        ubx_parser_.IngestByte(static_cast<uint8_t>(c), get_time_since_boot_ms());
        if (nmea && parser_.IngestByte(c) == NMEAParser::kSentenceRMC) LabelPPSEdge();
    }
    scan_.active = false;
    return scan_.done;
}

void UbloxMAXM10::OnUbxMessage(const UbxMessage& message) {
    M10Diagnostics::Decode(message, *this);
    if (scan_.active && !scan_.done) scan_.done = MatchScan(message);
}

bool UbloxMAXM10::MatchScan(const UbxMessage& message) {
    const uint8_t* p = message.payload;
    // An ACK/NAK for the awaited class/id answers a config request when the caller asked for it.
    if (scan_.config_nak != nullptr && IsAckFor(message, scan_.cls, scan_.id)) {
        *scan_.config_nak = message.id == kUbxIdAckNak;
        if (scan_.out_len != nullptr) *scan_.out_len = 0;
        return true;
    }
    // want_id == kUbxIdWildcard matches any id within the class.
    if (message.cls != scan_.cls || (scan_.id != kUbxIdWildcard && message.id != scan_.id)) return false;
    if (scan_.out_payload != nullptr) {
        uint16_t copy = message.stored < scan_.out_cap ? message.stored : scan_.out_cap;
        memcpy(scan_.out_payload, p, copy);
    }
    if (scan_.is_ack != nullptr) *scan_.is_ack = message.id == kUbxIdAckAck;
    if (scan_.out_len != nullptr) *scan_.out_len = message.length;
    return true;
}

bool UbloxMAXM10::WaitForUbxMessage(uint8_t want_class, uint8_t want_id, uint32_t timeout_ms, bool* is_ack) {
    return ScanForUbxMessage(want_class, want_id, timeout_ms, nullptr, 0, nullptr, is_ack);
}

bool UbloxMAXM10::WaitForAck(uint8_t acked_class, uint8_t acked_id) {
    (void)acked_class;
    (void)acked_id;
    bool is_ack = false;
    if (!WaitForUbxMessage(kUbxClassAck, kUbxIdWildcard, kAckTimeoutMs, &is_ack)) {
        return false;  // No ACK/NAK before timeout.
    }
    return is_ack;
}

bool UbloxMAXM10::Probe() {
    watchdog_update();
    SendUbxFrame(M10Diagnostics::kClassMon, M10Diagnostics::kIdMonVer, nullptr, 0);
    uint8_t payload[640];
    uint16_t length = 0;
    if (!ScanForUbxMessage(M10Diagnostics::kClassMon, M10Diagnostics::kIdMonVer, kProbeTimeoutMs, payload, sizeof(payload), &length, nullptr))
        return false;
    if (length >= 40 && length <= sizeof(payload) && (length - 40) % 30 == 0) {
        CONSOLE_INFO("UbloxMAXM10::Probe", "GNSS version: software=%.*s hardware=%.*s", 30,
                     reinterpret_cast<const char*>(payload), 10, reinterpret_cast<const char*>(payload + 30));
        for (uint16_t offset = 40; offset < length; offset += 30) {
            CONSOLE_INFO("UbloxMAXM10::Probe", "GNSS version extension: %.*s", 30,
                         reinterpret_cast<const char*>(payload + offset));
        }
    }
    return true;
}

void UbloxMAXM10::SetHostBaudrate(uint32_t baudrate) {
    uart_tx_wait_blocking(config_.uart_handle);
    uart_set_baudrate(config_.uart_handle, baudrate);
    char c;
    while (ReadBufferedByte(c)) {
    }
}

void UbloxMAXM10::SendBaudrate(uint32_t baudrate, uint8_t layers) {
    uint8_t payload[4 + 4 + 4];
    SendUbxFrame(kUbxClassCfg, kUbxIdCfgValset, payload,
                 AppendCfgKeyValue(payload, BeginCfgValset(payload, layers), kCfgUart1Baudrate, baudrate, 4));
    uart_tx_wait_blocking(config_.uart_handle);
}

bool UbloxMAXM10::ConnectAtConfiguredBaudrate() {
    uint32_t configured = GetDefaultBaudrate();
    SetHostBaudrate(configured);
    if (Probe()) return true;
    if (configured == kSafeBaudrate) return false;
    SetHostBaudrate(kSafeBaudrate);
    if (!Probe()) {
        SetHostBaudrate(configured);
        return false;
    }

    // Switch RAM only and skip the acknowledgment, which may arrive at either rate. BBR is written
    // only once the module answers at the new rate, so a failed switch is undone by a power cycle.
    SendBaudrate(configured, kCfgLayerRam);
    sleep_ms(kBaudrateSwitchSettleMs);
    SetHostBaudrate(configured);
    if (Probe() && CfgValSetU4(kCfgUart1Baudrate, configured, kCfgLayerBbr)) {
        CONSOLE_INFO("UbloxMAXM10::ConnectAtConfiguredBaudrate", "GNSS module switched from %lu to %lu baud.",
                     static_cast<unsigned long>(kSafeBaudrate), static_cast<unsigned long>(configured));
        return true;
    }
    CONSOLE_WARNING("UbloxMAXM10::ConnectAtConfiguredBaudrate",
                    "GNSS module did not answer at %lu baud; staying at %lu baud.",
                    static_cast<unsigned long>(configured), static_cast<unsigned long>(kSafeBaudrate));
    SetHostBaudrate(kSafeBaudrate);
    return Probe();
}

bool UbloxMAXM10::CfgValGet(uint32_t key_id, uint8_t value_size_bytes, uint64_t& value_out, uint8_t layer) {
    uint8_t req[4 + 4];
    SendUbxFrame(kUbxClassCfg, kUbxIdCfgValget, req, AppendCfgKeyValue(req, BeginCfgValget(req, layer), key_id, 0, 0));

    uint8_t resp[4 + 4 + 8];  // header + one key + up to an 8-byte value.
    uint16_t resp_len = 0;
    if (!ScanForUbxMessage(kUbxClassCfg, kUbxIdCfgValget, kAckTimeoutMs, resp, sizeof(resp), &resp_len, nullptr)) {
        return false;  // No VALGET response (key unknown to the receiver, or timed out).
    }
    return ReadCfgValue(resp, resp_len, layer, key_id, value_size_bytes, value_out);
}

bool UbloxMAXM10::CfgValSet(uint32_t key_id, uint64_t value, uint8_t value_size_bytes, uint8_t layers) {
    uint8_t payload[4 + 4 + 8];
    uint16_t len = AppendCfgKeyValue(payload, BeginCfgValset(payload, layers), key_id, value, value_size_bytes);

    SendUbxFrame(kUbxClassCfg, kUbxIdCfgValset, payload, len);
    return WaitForAck(kUbxClassCfg, kUbxIdCfgValset);
}

int32_t UbloxMAXM10::SyncConfigDefaults(uint32_t deadline_ms) {
    uint8_t layers = kCfgLayerRam | kCfgLayerBbr;
    int32_t num_written = 0;
    uint16_t num_read_fail = 0;
    for (uint32_t i = 0; i < kNumUbloxConfigDefaults; i++) {
        watchdog_update();
        // Signed difference is overflow-safe for wrapping millisecond timestamps.
        if (static_cast<int32_t>(get_time_since_boot_ms() - deadline_ms) >= 0) {
            CONSOLE_WARNING("UbloxMAXM10::SyncConfigDefaults",
                            "Config pass exceeded %lu ms budget after %lu/%u keys (%u read-backs failed so far); "
                            "module unresponsive, aborting.",
                            static_cast<unsigned long>(kInitConfigBudgetMs), static_cast<unsigned long>(i),
                            static_cast<unsigned>(kNumUbloxConfigDefaults), num_read_fail);
            return -1;
        }
        const UbloxCfgDefault& d = kUbloxConfigDefaults[i];
        uint64_t current = 0;
        if (!CfgValGet(d.key_id, d.size_bytes, current, kCfgGetLayerRam)) {
            // Couldn't read this key back; write it to be safe (fresh module / unknown state).
            num_read_fail++;
            if (CfgValSet(d.key_id, d.value, d.size_bytes, layers)) num_written++;
            continue;
        }
        if (current != d.value) {
            if (CfgValSet(d.key_id, d.value, d.size_bytes, layers)) num_written++;
        }
    }
    CONSOLE_INFO("UbloxMAXM10::SyncConfigDefaults", "Config sync: wrote %ld/%u keys (%u read-backs failed).",
                 static_cast<long>(num_written), static_cast<unsigned>(kNumUbloxConfigDefaults), num_read_fail);
    return num_written;
}

#ifndef AIRGREP
void UbloxMAXM10::ApplyRuntimeMessageConfig(uint8_t layers) {
    WriteMessageSelection(settings_manager.settings.gnss_diagnostics,
                          [&](uint32_t key, uint8_t rate) { CfgValSetU1(key, rate, layers); });
    CfgValSetE1(kCfgItfmAntSetting, AntennaSetting(settings_manager.settings.gnss_antenna), layers);
    // TEMPORARY: NAV-SAT feeds DebugDumpModuleStatus (remove with the rest of the debug instrumentation).
    CfgValSetU1(kCfgMsgoutUbxNavSatUart1, 1, layers);
}
#endif

#ifdef AIRGREP
bool UbloxMAXM10::ConfigureAirgrep(uint8_t layers) {
    bool ok = ConfigureAirgrepMessages(layers);
    uint64_t sbas_configured = 0;
    CfgValGet(kCfgSignalSbas, 1, sbas_configured, kCfgGetLayerBbr);

    sbas_known_configured_ = sbas_configured != 0;
    airgrep::WriteReceiverSettings(settings_manager.settings.gnss_antenna, [&](uint32_t key, uint16_t value, uint8_t size) {
        ok &= CfgValSet(key, value, size, layers);
    });
    return ok;
}

bool UbloxMAXM10::ConfigureAirgrepMessages(uint8_t layers) {
    if (!ApplyMessageBatch(airgrep::kGNSSMessages, airgrep::kBootstrapMessageCount, layers, kBootstrapReplyTimeoutMs))
        return false;

    for (uint16_t offset = airgrep::kBootstrapMessageCount; offset < airgrep::kGNSSMessageCount;
         offset += kMessageBatchSize) {
        airgrep::GNSSMessageRate messages[kMessageBatchSize];
        uint8_t count =
            airgrep::LoadMessages(offset, kMessageBatchSize, settings_manager.settings.gnss_diagnostics, messages);
        if (!ApplyMessageBatch(messages, count, layers)) return false;
    }
    return true;
}

bool UbloxMAXM10::ApplyMessageBatch(const airgrep::GNSSMessageRate* messages, uint8_t count, uint8_t layers,
                                    uint32_t timeout_ms) {
    watchdog_update();
    uint8_t payload[4 + kMessageBatchSize * 5];
    SendUbxFrame(kUbxClassCfg, kUbxIdCfgValset, payload, airgrep::WriteMessageRates(payload, layers, messages, count));
    bool nak = false;
    if (!ScanForUbxMessage(kUbxClassCfg, kUbxIdCfgValset, timeout_ms, nullptr, 0, nullptr, nullptr, &nak)) return false;
    if (!nak) return VerifyMessageBatch(messages, count, timeout_ms);
    if (count == 1) {
        CONSOLE_WARNING("UbloxMAXM10::ApplyMessageBatch", "GNSS message key 0x%08lx rejected%s.",
                        static_cast<unsigned long>(messages[0].key),
                        messages[0].rate ? "" : "; skipping disabled output");
        return messages[0].rate == 0;
    }
    uint8_t half = count / 2;
    return ApplyMessageBatch(messages, half, layers, timeout_ms) &&
           ApplyMessageBatch(messages + half, count - half, layers, timeout_ms);
}

bool UbloxMAXM10::VerifyMessageBatch(const airgrep::GNSSMessageRate* messages, uint8_t count, uint32_t timeout_ms) {
    uint8_t request[4 + kMessageBatchSize * 4];
    SendUbxFrame(kUbxClassCfg, kUbxIdCfgValget, request, airgrep::WriteMessageKeys(request, messages, count));
    uint32_t start_ms = get_time_since_boot_ms();
    for (uint32_t elapsed; (elapsed = get_time_since_boot_ms() - start_ms) < timeout_ms;) {
        uint8_t response[4 + kMessageBatchSize * 5];
        uint16_t response_len = 0;
        bool nak = false;
        if (!ScanForUbxMessage(kUbxClassCfg, kUbxIdCfgValget, timeout_ms - elapsed, response, sizeof(response),
                               &response_len, nullptr, &nak))
            return false;
        if (nak) {
            if (count == 1) {
                CONSOLE_WARNING("UbloxMAXM10::VerifyMessageBatch", "GNSS message key 0x%08lx read-back rejected.",
                                static_cast<unsigned long>(messages[0].key));
                return messages[0].rate == 0;
            }
            uint8_t half = count / 2;
            return VerifyMessageBatch(messages, half, timeout_ms) &&
                   VerifyMessageBatch(messages + half, count - half, timeout_ms);
        }
        uint32_t missing_keys = 0;
        const auto result = airgrep::CheckMessageReadback(messages, count, response, response_len, &missing_keys);
        if (result == airgrep::Readback::unrelated) continue;
        for (uint8_t i = 0; i < count; i++) {
            if (!(missing_keys & (1u << i))) continue;
            CONSOLE_WARNING("UbloxMAXM10::VerifyMessageBatch", "GNSS message key 0x%08lx missing from read-back.",
                            static_cast<unsigned long>(messages[i].key));
        }
        return result == airgrep::Readback::verified;
    }
    return false;
}
#endif

bool UbloxMAXM10::SendInitCommands() {
    if (!ConnectAtConfiguredBaudrate()) {
        return false;
    }

#ifndef AIRGREP
    uint32_t config_deadline_ms = get_time_since_boot_ms() + kInitConfigBudgetMs;
    if (SyncConfigDefaults(config_deadline_ms) < 0) {
        return false;
    }
#endif
    uint8_t layers = kCfgLayerRam | kCfgLayerBbr;
    // NMEA sentence selection.
#ifdef AIRGREP
    return ConfigureAirgrep(layers);
#else

    ApplyRuntimeMessageConfig(layers);

    // Enable AssistNow Autonomous so the module predicts its own orbits and shortens time-to-fix
    // even after a genuine cold start (factory default is disabled).
    CfgValSetL(kCfgAnaUseAna, true, layers);
    return true;
#endif
}

void UbloxMAXM10::ResendRuntimeConfig() {
#ifdef AIRGREP
    if (!ConnectAtConfiguredBaudrate()) return;
    healthy_ = ConfigureAirgrep(kCfgLayerRam);
#else
    // After a UART handover (ESP32 flash) the module kept running; just re-assert the NMEA output
    // selection to RAM so sentences resume. Cheap and non-destructive -- no full config pass, so
    // the live fix is undisturbed.
    if (!ConnectAtConfiguredBaudrate()) return;
    ApplyRuntimeMessageConfig(kCfgLayerRam);
#endif
}

void UbloxMAXM10::DiscardModuleStream() {
    ubx_parser_.Resync();
    telemetry_.Reset();
}

void UbloxMAXM10::OnRf(const M10Diagnostics::RfStatus& rf) {
    Store(rf_, rf);
    telemetry_.OnRf(rf);
    CONSOLE_INFO("UbloxMAXM10::OnRf", "jamming=%s antenna=%s power=%s post=0x%08lX noise=%u agc=%u cw=%u I=%d/%u Q=%d/%u",
                 M10Strings::Jamming(rf.jamming), M10Strings::AntennaStatus(rf.antenna_status),
                 M10Strings::AntennaPower(rf.antenna_power), static_cast<unsigned long>(rf.post_status), rf.noise,
                 rf.agc, rf.cw_suppression, rf.offset_i, rf.magnitude_i, rf.offset_q, rf.magnitude_q);
}

void UbloxMAXM10::OnSbas(const M10Diagnostics::SbasStatus& sbas) {
    Store(sbas_, sbas);
    CONSOLE_INFO("UbloxMAXM10::OnSbas", "mode=%s system=%s geo=%u service=0x%02X satellites=%u flags=0x%02X",
                 M10Strings::SbasMode(sbas.mode), M10Strings::SbasSystem(sbas.system), sbas.geo, sbas.service,
                 sbas.count, sbas.flags);
    if (const M10Diagnostics::SbasSatellite* sv = sbas.FirstCorrected()) {
        CONSOLE_INFO("UbloxMAXM10::OnSbas", "sv=%u prc=%dcm ic=%dcm", sv->id, sv->range_correction_cm,
                     sv->ionosphere_correction_cm);
    }
}

void UbloxMAXM10::OnComms(const M10Diagnostics::CommsStatus& comms) {
    Store(comms_, comms);
    for (uint8_t i = 0; i < comms.count && i < M10Diagnostics::kStoredCommsPorts; i++) {
        const M10Diagnostics::CommsPort& port = comms.ports[i];
        CONSOLE_INFO("UbloxMAXM10::OnComms",
                     "txErrors=0x%02X port=%s tx=%luB/%u%%/peak %u%% pending=%u rx=%luB/%u%%/peak %u%% pending=%u "
                     "overrun=%u skipped=%lu",
                     comms.tx_errors, M10Strings::Port(port.id), static_cast<unsigned long>(port.tx_bytes),
                     port.tx_usage, port.tx_peak_usage, port.tx_pending, static_cast<unsigned long>(port.rx_bytes),
                     port.rx_usage, port.rx_peak_usage, port.rx_pending, port.overrun_errors,
                     static_cast<unsigned long>(port.skipped_bytes));
    }
}

void UbloxMAXM10::OnHw3(const M10Diagnostics::Hw3Status& hw3) {
    Store(hw3_, hw3);
    CONSOLE_INFO("UbloxMAXM10::OnHw3", "flags=0x%02X hw=%s pins=%u", hw3.flags, hw3.hardware_version, hw3.count);
}

void UbloxMAXM10::OnSatellites(const M10Diagnostics::SatelliteSummary& satellites) {
    Store(satellites_, satellites);
    CONSOLE_INFO("UbloxMAXM10::OnSatellites",
                 "count=%u noSignal=%u searching=%u acquired=%u unusable=%u locked=%u ephemeris=%u used=%u healthy=%u "
                 "unhealthy=%u maxCno=%udBHz",
                 satellites.count, satellites.no_signal, satellites.searching, satellites.acquired,
                 satellites.unusable, satellites.locked, satellites.ephemeris, satellites.used, satellites.healthy,
                 satellites.unhealthy, satellites.max_cno_dbhz);
}

bool UbloxMAXM10::PollComms(M10Diagnostics::CommsStatus& comms) {
    comms_.valid = false;
    SendUbxFrame(M10Diagnostics::kClassMon, M10Diagnostics::kIdMonComms, nullptr, 0);
    if (!WaitForUbxMessage(M10Diagnostics::kClassMon, M10Diagnostics::kIdMonComms, kDiagnosticsTimeoutMs) || !comms_.valid) return false;
    comms = comms_.value;
    return true;
}

bool UbloxMAXM10::PollHw3(M10Diagnostics::Hw3Status& hw3) {
    hw3_.valid = false;
    SendUbxFrame(M10Diagnostics::kClassMon, M10Diagnostics::kIdMonHw3, nullptr, 0);
    if (!WaitForUbxMessage(M10Diagnostics::kClassMon, M10Diagnostics::kIdMonHw3, kDiagnosticsTimeoutMs) || !hw3_.valid) return false;
    hw3 = hw3_.value;
    return true;
}

void UbloxMAXM10::DebugDumpModuleStatus() {
    // TEMPORARY: print the latest MON-RF + NAV-SAT snapshot (remove with the rest of the
    // GNSS debug instrumentation).
    if (rf_.valid) {
        const M10Diagnostics::RfStatus& rf = rf_.value;
        CONSOLE_INFO("UbloxMAXM10::DebugDumpModuleStatus",
                     "GNSS RF: jam=%u antStatus=%u(%s) antPower=%u(%s) noise=%u agc=%u(/8191) magI=%u magQ=%u\r\n",
                     rf.jamming, rf.antenna_status, M10Strings::AntennaStatus(rf.antenna_status), rf.antenna_power,
                     M10Strings::AntennaPower(rf.antenna_power), rf.noise, rf.agc, rf.magnitude_i, rf.magnitude_q);
    } else {
        CONSOLE_INFO("UbloxMAXM10::DebugDumpModuleStatus", "GNSS RF: waiting for UBX-MON-RF frame...\r\n");
    }
    if (satellites_.valid) {
        CONSOLE_INFO("UbloxMAXM10::DebugDumpModuleStatus", "GNSS SAT: numSvs=%u maxCno=%u dBHz\r\n",
                     satellites_.value.count, satellites_.value.max_cno_dbhz);
    } else {
        CONSOLE_INFO("UbloxMAXM10::DebugDumpModuleStatus", "GNSS SAT: waiting for UBX-NAV-SAT frame...\r\n");
    }
}
