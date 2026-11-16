#pragma once

#include <cstddef>
#include <cstdint>
#include <ctime>

static constexpr uint32_t kSecondsPerDay = 86400;

/**
 * Seconds since 0000Z of a UTC timestamp.
 * @param[in] utc_unix_ms Milliseconds since the Unix epoch.
 * @retval Seconds in [0, 86400).
 */
inline uint32_t UTCSecondsOfDay(uint64_t utc_unix_ms) {
    time_t utc_s = static_cast<time_t>(utc_unix_ms / 1000);
    struct tm utc;
    gmtime_r(&utc_s, &utc);
    return (utc.tm_hour * 60 + utc.tm_min) * 60 + utc.tm_sec;
}

/**
 * Formats a UTC timestamp with strftime.
 * @param[in] utc_unix_ms Milliseconds since the Unix epoch.
 * @param[in] format strftime format, e.g. "%H:%M:%S".
 * @param[out] buf Output buffer.
 * @param[in] buf_len Size of buf, in bytes.
 * @retval Number of characters written, excluding the terminator. 0 if buf is too small.
 */
inline size_t FormatUTC(uint64_t utc_unix_ms, const char* format, char* buf, size_t buf_len) {
    time_t utc_s = static_cast<time_t>(utc_unix_ms / 1000);
    struct tm utc;
    gmtime_r(&utc_s, &utc);
    return strftime(buf, buf_len, format, &utc);
}
