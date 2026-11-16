#pragma once

#include <cstddef>
#include <cstdint>
#include <ctime>

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
