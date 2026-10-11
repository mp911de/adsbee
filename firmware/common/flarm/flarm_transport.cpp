#include "flarm_transport.hh"
#include <cstring>
extern "C" {
#include "flarm_famp.h"
}

bool DecodeFlarmRadioFrame(const uint8_t* encoded, unsigned length, uint8_t payload[24]) {
    if (!encoded || !payload || length != 54) return false;
    bool inverted = encoded[0] == 0x9a && encoded[1] == 0x69;
    if (!inverted && !(encoded[0] == 0x65 && encoded[1] == 0x96)) return false;
    uint8_t frame[26] = {};
    for (unsigned bit = 0; bit < 208; ++bit) {
        unsigned pair = (encoded[2 + bit / 4] >> (6 - 2 * (bit % 4))) & 3;
        if (inverted) pair ^= 3;
        if (pair != 1 && pair != 2) return false;
        frame[bit / 8] |= (pair == 1 ? 1 : 0) << (7 - bit % 8);
    }
    uint16_t crc = famp_crc16(famp_crc16(0xffff, famp_sync, 3), frame, 24);
    if (crc != (frame[24] | (uint16_t(frame[25]) << 8))) return false;
    memcpy(payload, frame, 24);
    return true;
}

void FlarmPhaseEstimator::Record(uint32_t rx_end_ms, uint8_t channel) {
    uint32_t t = rx_end_ms - kPacketMs - (channel == 2 ? kFrame2Start - kFrame1Start : 0);
    if (count_ == kSamples) memmove(times_, times_ + 1, sizeof(times_) - sizeof(times_[0]));
    else ++count_;
    times_[count_ - 1] = t;
}

uint8_t FlarmPhaseEstimator::Channel(uint32_t now_ms) {
    unsigned keep = 0;
    for (unsigned i = 0; i < count_; ++i)
        if (now_ms - times_[i] < kExpiryMs) times_[keep++] = times_[i];
    count_ = keep;
    if (!count_) return 1;
    uint16_t phase[kSamples];
    for (unsigned i = 0; i < count_; ++i) {
        uint16_t v = times_[i] % 1000, j = i;
        for (; j && phase[j - 1] > v; --j) phase[j] = phase[j - 1];
        phase[j] = v;
    }
    unsigned gap = 1000 - phase[count_ - 1] + phase[0], start = phase[0];
    for (unsigned i = 1; i < count_; ++i) {
        unsigned g = phase[i] - phase[i - 1];
        if (g > gap) gap = g, start = phase[i];
    }
    unsigned span = 1000 - gap;
    if (span < kMinSpanMs || span > kFrameWindowMs + 2 * kPacketMs) return 1;
    unsigned now = (kFrame1Start + (kFrameWindowMs - span) / 2 + now_ms % 1000 + 1000 - start) % 1000;
    return now >= kHop1 && now < kHop2 ? 1 : 2;
}
