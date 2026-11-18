#include "comms.hh"
#include "gnss_interface.hh"
#include "hal.hh"
#include "settings.hh"
#include "utc_time.hh"

CPP_AT_CALLBACK(CommsManager::ATGNSSCallback) {
    auto& settings = settings_manager.settings;
    switch (op) {
        case '?':
            CPP_AT_CMD_PRINTF("=%d,%s,%d,%d,%s", settings.gnss_enabled,
                              SettingsManager::GNSSReceiverTypeToStr(settings.gnss_receiver_type),
                              settings.gnss_notify, settings.gnss_diagnostics,
                              SettingsManager::GNSSAntennaModeToStr(settings.gnss_antenna));
            CPP_AT_SILENT_SUCCESS();
            break;
        case '=': {
            if (!CPP_AT_HAS_ARG(0)) {
                CPP_AT_ERROR(
                    "Requires AT+GNSS=<0|1>[,<NONE|GENERIC|UBX_MIA>[,<notify>[,<diagnostics>[,<antenna>]]]].");
            }

            bool enabled;
            if (args[0].compare("0") == 0) {
                enabled = false;
            } else if (args[0].compare("1") == 0) {
                enabled = true;
            } else {
                CPP_AT_ERROR("GNSS enable must be 0 or 1.");
            }

            // With no explicit type, preserve the configured receiver selection. Enabling is not
            // allowed when the saved type is NONE because there is no receiver implementation to use.
            auto type = settings.gnss_receiver_type;
            if (CPP_AT_HAS_ARG(1)) {
                if (args[1].compare("NONE") == 0) {
                    type = SettingsManager::kGNSSReceiverNone;
                } else if (args[1].compare("GENERIC") == 0) {
                    type = SettingsManager::kGNSSReceiverGeneric;
                } else if (args[1].compare("UBX_MIA") == 0) {
                    type = SettingsManager::kGNSSReceiverUBXMIA;
                } else {
                    CPP_AT_ERROR("GNSS type must be NONE, GENERIC, or UBX_MIA.");
                }
            } else if (enabled && type == SettingsManager::kGNSSReceiverNone) {
                CPP_AT_ERROR("Cannot enable GNSS while the saved type is NONE; specify GENERIC or UBX_MIA.");
            }

            bool notify = false;
            if (CPP_AT_HAS_ARG(2)) {
                if (args[2].compare("0") == 0) {
                    notify = false;
                } else if (args[2].compare("1") == 0) {
                    notify = true;
                } else {
                    CPP_AT_ERROR("GNSS notify must be 0 or 1.");
                }
            }

            bool diagnostics = settings.gnss_diagnostics;
            if (CPP_AT_HAS_ARG(3)) {
                if (args[3] != "0" && args[3] != "1") CPP_AT_ERROR("GNSS diagnostics must be 0 or 1.");
                diagnostics = args[3] == "1";
            }
            if (CPP_AT_HAS_ARG(4)) {
                uint8_t mode = 0;
                while (mode < SettingsManager::kNumGNSSAntennaModes &&
                       args[4].compare(SettingsManager::kGNSSAntennaModeStrs[mode]) != 0)
                    mode++;
                if (mode == SettingsManager::kNumGNSSAntennaModes)
                    CPP_AT_ERROR("GNSS antenna must be UNKNOWN, PASSIVE, ACTIVE, or ACTIVE_SUPERVISED.");
                settings.gnss_antenna = static_cast<SettingsManager::GNSSAntennaMode>(mode);
            }
            settings.gnss_diagnostics = diagnostics;
            settings.gnss_enabled = enabled && type != SettingsManager::kGNSSReceiverNone;
            settings.gnss_receiver_type = type;
            settings.gnss_notify = notify;
#ifdef ON_TI
            if (!gnss_receiver.Configure(settings.gnss_enabled, type)) {
#else
            if (!ConfigureGNSSReceiver(enabled, SettingsToGNSSModuleType(type))) {
#endif
                CPP_AT_ERROR("GNSS receiver did not initialize successfully.");
            }
            CPP_AT_SUCCESS();
            break;
        }
    }
    CPP_AT_ERROR("Operator '%c' not supported.", op);
}

CPP_AT_CALLBACK(CommsManager::ATGNSSFixCallback) {
    if (op != '?') {
        CPP_AT_ERROR("Only AT+GNSS_FIX? is supported.");
    }

    static constexpr NMEAParser::GNSSFix kNoFix{};
    const NMEAParser::GNSSFix& fix = gnss ? gnss->fix() : kNoFix;
    char utc_time[9] = "--:--:--";
    uint64_t utc_unix_ms;
    char utc_epoch[24] = "";
    if (gnss && gnss->GetFixUTCUnixMs(utc_unix_ms)) {
        FormatUTC(utc_unix_ms, "%H:%M:%S", utc_time, sizeof(utc_time));
        snprintf(utc_epoch, sizeof(utc_epoch), "%llu", static_cast<unsigned long long>(utc_unix_ms));
    }
    CPP_AT_CMD_PRINTF("=%d,%.6f,%.6f,%ld,%.1f,%ld,%u,%s,%lu,%s", gnss && gnss->HasValidFix(), fix.latitude_deg,
                      fix.longitude_deg, static_cast<long>(fix.altitude_ft), fix.heading_deg,
                      static_cast<long>(fix.speed_kts), static_cast<unsigned int>(fix.num_satellites), utc_time,
                      static_cast<unsigned long>(gnss ? gnss->pps_count() : 0), utc_epoch);
    CPP_AT_SILENT_SUCCESS();
}

CPP_AT_CALLBACK(CommsManager::ATGNSSStatusCallback) {
    if (op != '?') CPP_AT_ERROR("Only AT+GNSS_STATUS? is supported.");

    constexpr char kUnknown[] = "UNKNOWN";
    const char *module = kUnknown, *software = kUnknown, *hardware = kUnknown, *antenna = kUnknown, *bbr = kUnknown;
    const char *antenna_power = kUnknown, *sbas_mode = kUnknown, *sbas_system = kUnknown;
    int max_cno = -1, seen = -1, no_signal = -1, searching = -1, acquired = -1, unusable = -1, locked = -1;
    int ephemeris = -1, used = -1, healthy = -1, unhealthy = -1, sbas_satellites = -1, prc_cm = -1, ic_cm = -1;
    if (auto* m10 = gnss ? gnss->M10() : nullptr) {
        uint32_t now_ms = get_time_since_boot_ms();
        const auto& version = m10->version();
        if (version.module[0]) module = version.module;
        if (version.software[0]) software = version.software;
        if (version.hardware[0]) hardware = version.hardware;
        if (m10->rf().IsFresh(now_ms)) {
            antenna = M10Strings::AntennaStatus(m10->rf().value.antenna_status);
            antenna_power = M10Strings::AntennaPower(m10->rf().value.antenna_power);
        }
        if (m10->satellites().IsFresh(now_ms)) {
            const M10Diagnostics::SatelliteSummary& s = m10->satellites().value;
            max_cno = s.max_cno_dbhz, seen = s.count, no_signal = s.no_signal, searching = s.searching;
            acquired = s.acquired, unusable = s.unusable, locked = s.locked, ephemeris = s.ephemeris;
            used = s.used, healthy = s.healthy, unhealthy = s.unhealthy;
        }
        if (m10->sbas().IsFresh(now_ms)) {
            const M10Diagnostics::SbasStatus& sbas = m10->sbas().value;
            sbas_mode = M10Strings::SbasMode(sbas.mode);
            sbas_system = M10Strings::SbasSystem(sbas.system);
            sbas_satellites = sbas.count;
            if (const M10Diagnostics::SbasSatellite* sv = sbas.FirstCorrected()) {
                prc_cm = sv->range_correction_cm;
                ic_cm = sv->ionosphere_correction_cm;
            }
        }
        if (m10->sbas_known_configured()) {
            bbr = "OK";
        }
    }
    CPP_AT_CMD_PRINTF("=%s,%s,%s,%s,%.2f,%s,%s,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%s,%s,%d,%d,%d", module, software,
                      hardware, bbr, gnss ? gnss->fix().hdop : -1.0f, antenna, antenna_power, max_cno, seen, no_signal, searching, acquired, unusable, locked,
                      ephemeris, used, healthy, unhealthy, sbas_mode, sbas_system, sbas_satellites, prc_cm, ic_cm);
    CPP_AT_SILENT_SUCCESS();
}

CPP_AT_CALLBACK(CommsManager::ATGNSSDiagCallback) {
    if (op != '?') CPP_AT_ERROR("Only AT+GNSS_DIAG? is supported.");
    auto* m10 = gnss ? gnss->M10() : nullptr;
    if (!m10 || !gnss->IsHealthy()) CPP_AT_ERROR("No M10 module is running.");
    // Reporting can use the console while polls wait. Emit the complete reply after both polls.
    M10Diagnostics::CommsStatus comms;
    M10Diagnostics::Hw3Status hw3;
    const bool got_comms = m10->PollComms(comms);
    const bool got_hw3 = got_comms && m10->PollHw3(hw3);
    uint32_t now_ms = get_time_since_boot_ms();
    if (m10->rf().valid) {
        const M10Diagnostics::RfStatus& rf = m10->rf().value;
        CPP_AT_PRINTF("RF: age=%lums jamming=%s antenna=%s power=%s noise=%u agc=%u cw=%u I=%d/%u Q=%d/%u\r\n",
                          static_cast<unsigned long>(now_ms - m10->rf().received_ms), M10Strings::Jamming(rf.jamming),
                          M10Strings::AntennaStatus(rf.antenna_status), M10Strings::AntennaPower(rf.antenna_power),
                          rf.noise, rf.agc, rf.cw_suppression, rf.offset_i, rf.magnitude_i, rf.offset_q,
                          rf.magnitude_q);
    }
    if (m10->sbas().valid) {
        const M10Diagnostics::SbasStatus& sbas = m10->sbas().value;
        CPP_AT_PRINTF("SBAS: age=%lums mode=%s system=%s geo=%u service=0x%02X satellites=%u flags=0x%02X\r\n",
                          static_cast<unsigned long>(now_ms - m10->sbas().received_ms), M10Strings::SbasMode(sbas.mode),
                          M10Strings::SbasSystem(sbas.system), sbas.geo, sbas.service, sbas.count, sbas.flags);
        for (uint8_t i = 0; i < sbas.count && i < M10Diagnostics::kStoredSbasSatellites; i++) {
            const M10Diagnostics::SbasSatellite& sv = sbas.satellites[i];
            CPP_AT_PRINTF("SBAS:  sv=%u udre=%u sys=%u service=0x%02X prc=%dcm ic=%dcm\r\n", sv.id, sv.udre,
                              sv.system, sv.service, sv.range_correction_cm, sv.ionosphere_correction_cm);
        }
    }
    if (!got_comms) CPP_AT_ERROR("MON-COMMS poll failed.");
    for (uint8_t i = 0; i < comms.count && i < M10Diagnostics::kStoredCommsPorts; i++) {
        const M10Diagnostics::CommsPort& port = comms.ports[i];
        CPP_AT_PRINTF("COMMS: txErrors=0x%02X port=%s tx=%luB/%u%%/peak %u%% pending=%u rx=%luB/%u%%/peak %u%% "
                          "pending=%u overrun=%u ubx=%u nmea=%u rtcm2=%u rtcm3=%u skipped=%lu\r\n",
                          comms.tx_errors, M10Strings::Port(port.id), static_cast<unsigned long>(port.tx_bytes),
                          port.tx_usage, port.tx_peak_usage, port.tx_pending, static_cast<unsigned long>(port.rx_bytes),
                          port.rx_usage, port.rx_peak_usage, port.rx_pending, port.overrun_errors, port.messages[0],
                          port.messages[1], port.messages[2], port.messages[3],
                          static_cast<unsigned long>(port.skipped_bytes));
    }
    if (!got_hw3) CPP_AT_ERROR("MON-HW3 poll failed.");
    CPP_AT_PRINTF("HW3: flags=0x%02X hw=%s pins=%u\r\n", hw3.flags, hw3.hardware_version, hw3.count);
    for (uint8_t i = 0; i < hw3.count && i < M10Diagnostics::kStoredHw3Pins; i++) {
        CPP_AT_PRINTF("HW3:  pin=%u mask=0x%04X virtual=%u\r\n", hw3.pins[i].id, hw3.pins[i].mask,
                          hw3.pins[i].virtual_pin);
    }
    CPP_AT_SILENT_SUCCESS();
}
