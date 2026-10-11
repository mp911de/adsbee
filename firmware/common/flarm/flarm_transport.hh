#pragma once
#include <cstdint>
#include "receiver_config.hh"

struct FlarmRadioPacket {
    uint32_t sequence = 0;
    uint32_t age_ms = 0;  // Reception timestamp on CC1312, converted to age when read over SPI.
    uint8_t payload[24] = {};
    int8_t rssi_dbm = 0;
    uint8_t reserved[3] = {};
};
struct FlarmBatch {
    static constexpr uint8_t kCapacity = 8;
    uint8_t version = 1;
    uint8_t count = 0;
    uint16_t dropped = 0;  // Cumulative CC1312 queue overflows, wraps.
    FlarmRadioPacket packets[kCapacity] = {};
};
static_assert(sizeof(FlarmRadioPacket) == 36);
static_assert(sizeof(FlarmBatch) == 292);

// Bytes following the first 32 Manchester sync symbols: remaining sync + 26 encoded bytes.
bool DecodeFlarmRadioFrame(const uint8_t* encoded, unsigned length, uint8_t payload[24]);

// Infers the FLARM 1 s cycle from reception times. FTD-116 2.2: Frame 1 on 868.2 MHz (channel 1)
// spans 450-820 ms, Frame 2 on 868.4 MHz (channel 2) spans 830-1200 ms. The radio listens on
// channel 1 from kHop1 to kHop2 and hops only once the observed transmissions spread over most of a
// frame, otherwise it stays on channel 1.
class FlarmPhaseEstimator {
public:
    static constexpr uint32_t kExpiryMs = 30000;
    static constexpr unsigned kSamples = 32;
    static constexpr uint16_t kFrame1Start = 450, kFrame2Start = 830, kFrameWindowMs = 370;
    static constexpr uint16_t kHop1 = 325, kHop2 = 825;  // Mid quiet gap, mid 10 ms guard.
    static constexpr uint16_t kMinSpanMs = 300, kPacketMs = 5;
    void Record(uint32_t rx_end_ms, uint8_t channel);
    uint8_t Channel(uint32_t now_ms);
    void Reset() { count_ = 0; }
private:
    uint32_t times_[kSamples] = {};
    unsigned count_ = 0;
};
