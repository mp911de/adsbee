#pragma once

#include <cstddef>
#include <cstdint>
#include <string_view>

#include "nmea_parser.hh"
#include "ublox_max_m10_diagnostics.hh"

// One airgrep #GNSS frame, the push replacement for AT+GNSS_FIX? and AT+GNSS_STATUS?.
// Payload fields in order, plain decimal unless noted, empty when unknown or stale:
//   fix (0/1, valid and fresh), quality (GGA fix quality), sats (used in fix), lat (deg, 6 decimals), lon (deg),
//   alt (ft MSL), speed (kt over ground), track (deg true, 1 decimal), hdop (2 decimals), utc (HHMMSS),
//   epoch (UTC unix ms), antenna, antenna_power, jamming (UBX-MON-RF codes, as in #M10 MON frames),
//   max_cno (dBHz), sats seen, no_signal, searching, acquired, unusable, locked, ephemeris, used, healthy, unhealthy,
//   sbas_mode, sbas_system (strings as AT+GNSS_STATUS? prints them), sbas_satellites, sbas_prc_cm, sbas_ic_cm,
//   module, software, hardware (UBX-MON-VER strings, commas replaced by spaces), bbr (OK when BBR kept SBAS config).
// Framing and CRC as in #M10: trailing comma, CRC16 over everything before it, uppercase hex without zero padding.
struct AirgrepGnssFrame {
    static constexpr size_t kFrameMaxLen = 320;

    const NMEAParser::GNSSFix& fix;
    bool fix_valid = false;
    bool epoch_valid = false;
    uint64_t epoch_ms = 0;
    const M10Diagnostics::RfStatus* rf = nullptr;
    const M10Diagnostics::SatelliteSummary* satellites = nullptr;
    const M10Diagnostics::SbasStatus* sbas = nullptr;
    // Empty strings print as empty fields.
    const char* module = "";
    const char* software = "";
    const char* hardware = "";
    bool bbr_ok = false;

    // @retval Bytes written including CRLF, 0 when the frame does not fit.
    size_t Format(char* frame, size_t capacity) const;
};

// Reply line of AT+TIME_PING=<echo>: TIME_PING=<echo>,<epoch_ms>,<pps_count>\r\n
// echo is the host's opaque decimal token, 1 to 20 digits, returned unchanged. epoch_ms is the PPS disciplined UTC
// unix time at reply time, empty when unavailable. pps_count is the PPS edge count as AT+GNSS_FIX? reports it.
struct AirgrepTimePing {
    static constexpr size_t kEchoMaxLen = 20;
    static constexpr size_t kReplyMaxLen = 64;

    std::string_view echo;
    bool epoch_valid = false;
    uint64_t epoch_ms = 0;
    uint32_t pps_count = 0;

    // @retval Bytes written including CRLF, 0 when echo is not 1 to 20 decimal digits or the reply does not fit.
    size_t Format(char* reply, size_t capacity) const;
};
