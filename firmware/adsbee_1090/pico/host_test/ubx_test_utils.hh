#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "ubx_parser.hh"

// Builds a complete UBX frame with a valid checksum.
inline std::vector<uint8_t> UBX(uint8_t cls, uint8_t id, const std::string& payload) {
    std::vector<uint8_t> frame{0xB5, 0x62, cls, id, static_cast<uint8_t>(payload.size()),
                               static_cast<uint8_t>(payload.size() >> 8)};
    frame.insert(frame.end(), payload.begin(), payload.end());
    uint8_t a = 0, b = 0;
    for (size_t i = 2; i < frame.size(); ++i) {
        a += frame[i];
        b += a;
    }
    frame.push_back(a);
    frame.push_back(b);
    return frame;
}

inline void Feed(UbxParser& parser, const std::vector<uint8_t>& bytes, uint32_t now = 1) {
    for (uint8_t b : bytes) parser.IngestByte(b, now);
}
