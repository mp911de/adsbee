#pragma once

#include "airgrep_m10_profile.hh"

namespace airgrep {

enum class Readback { unrelated, mismatch, verified };

// A RAM VALGET reply may omit unsupported outputs whose requested rate is zero.
inline Readback CheckMessageReadback(const GNSSMessageRate* messages, uint8_t count, const uint8_t* payload,
                                     uint16_t length, uint32_t* missing_keys = nullptr) {
    constexpr uint8_t header_size = 4, entry_size = 5;
    if (count == 0 || count > 32 || length < header_size || (length - header_size) % entry_size != 0 ||
        length > header_size + count * entry_size || payload[0] != 1 || payload[1] || payload[2] || payload[3])
        return Readback::unrelated;

    uint32_t seen = 0;
    bool rates_match = true;
    for (uint16_t pos = header_size; pos < length; pos += entry_size) {
        const uint32_t key = uint32_t(payload[pos]) | (uint32_t(payload[pos + 1]) << 8) |
                             (uint32_t(payload[pos + 2]) << 16) | (uint32_t(payload[pos + 3]) << 24);
        uint8_t i = 0;
        while (i < count && messages[i].key != key) ++i;
        if (i == count || (seen & (uint32_t{1} << i))) return Readback::unrelated;
        seen |= uint32_t{1} << i;
        if (messages[i].rate != payload[pos + 4]) rates_match = false;
    }
    uint32_t missing = 0;
    for (uint8_t i = 0; i < count; ++i) {
        const uint32_t bit = uint32_t{1} << i;
        if (seen & bit) continue;
        missing |= bit;
        if (messages[i].rate != 0) rates_match = false;
    }
    if (missing_keys) *missing_keys = missing;
    return rates_match ? Readback::verified : Readback::mismatch;
}

}  // namespace airgrep
