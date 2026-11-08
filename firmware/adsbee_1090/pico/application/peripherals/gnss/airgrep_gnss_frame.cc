#include "airgrep_gnss_frame.hh"

#include <cstdio>

#include "buffer_utils.hh"

namespace {

// Appends text with commas turned into spaces, so a free-form module string cannot split the payload.
size_t AppendField(char* frame, size_t capacity, size_t length, const char* text) {
    for (; *text && length + 1 < capacity; ++text) frame[length++] = *text == ',' ? ' ' : *text;
    frame[length] = '\0';
    return length;
}

}  // namespace

size_t AirgrepGnssFrame::Format(char* frame, size_t capacity) const {
    if (!frame || capacity < kFrameMaxLen + 1) return 0;
    char position[64] = ",,,,";
    if (fix_valid) {
        snprintf(position, sizeof(position), "%.6f,%.6f,%ld,%ld,%.1f", fix.latitude_deg, fix.longitude_deg,
                 static_cast<long>(fix.altitude_ft), static_cast<long>(fix.speed_kts), fix.heading_deg);
    }
    char hdop[8] = "";
    if (fix.hdop >= 0.0f) snprintf(hdop, sizeof(hdop), "%.2f", fix.hdop);
    char utc[8] = "";
    if (fix.utc_time_valid) snprintf(utc, sizeof(utc), "%02u%02u%02u", fix.utc_hour, fix.utc_minute, fix.utc_second);
    char epoch[24] = "";
    if (epoch_valid) snprintf(epoch, sizeof(epoch), "%llu", static_cast<unsigned long long>(epoch_ms));
    char radio[16] = ",,";
    if (rf) snprintf(radio, sizeof(radio), "%u,%u,%u", rf->antenna_status, rf->antenna_power, rf->jamming);
    char sky[48] = ",,,,,,,,,,";
    if (satellites) {
        const M10Diagnostics::SatelliteSummary& s = *satellites;
        snprintf(sky, sizeof(sky), "%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u", s.max_cno_dbhz, s.count, s.no_signal,
                 s.searching, s.acquired, s.unusable, s.locked, s.ephemeris, s.used, s.healthy, s.unhealthy);
    }
    char augmentation[48] = ",,,,";
    if (sbas) {
        char corrections[16] = ",";
        if (const M10Diagnostics::SbasSatellite* sv = sbas->FirstCorrected()) {
            snprintf(corrections, sizeof(corrections), "%d,%d", sv->range_correction_cm, sv->ionosphere_correction_cm);
        }
        snprintf(augmentation, sizeof(augmentation), "%s,%s,%u,%s", M10Strings::SbasMode(sbas->mode),
                 M10Strings::SbasSystem(sbas->system), sbas->count, corrections);
    }

    int header = snprintf(frame, capacity, "#GNSS:%u,%u,%u,%s,%s,%s,%s,%s,%s,%s,", fix_valid ? 1 : 0,
                          fix.fix_quality, fix.num_satellites, position, hdop, utc, epoch, radio, sky, augmentation);
    if (header <= 0 || static_cast<size_t>(header) >= capacity) return 0;
    size_t length = header;
    const char* texts[] = {module, software, hardware, bbr_ok ? "OK" : ""};
    for (const char* text : texts) {
        length = AppendField(frame, capacity, length, text);
        if (length + 1 >= capacity) return 0;
        frame[length++] = ',';
    }
    if (length + 7 > capacity) return 0;
    uint16_t crc = CalculateCRC16(reinterpret_cast<const uint8_t*>(frame), length);
    int tail = snprintf(frame + length, capacity - length, "%X\r\n", crc);
    return length + tail;
}

size_t AirgrepTimePing::Format(char* reply, size_t capacity) const {
    if (!reply || capacity < kReplyMaxLen + 1 || echo.empty() || echo.size() > kEchoMaxLen) return 0;
    for (char c : echo) {
        if (c < '0' || c > '9') return 0;
    }
    char epoch[24] = "";
    if (epoch_valid) snprintf(epoch, sizeof(epoch), "%llu", static_cast<unsigned long long>(epoch_ms));
    int length = snprintf(reply, capacity, "TIME_PING=%.*s,%s,%lu\r\n", static_cast<int>(echo.size()), echo.data(),
                          epoch, static_cast<unsigned long>(pps_count));
    return length > 0 && static_cast<size_t>(length) < capacity ? length : 0;
}
