#pragma once
#include <cstddef>
#include <cstdint>
#include "flarm_transport.hh"
extern "C" {
#include "flarm_famp.h"
}

struct FlarmTarget {
    famp_data_t data = {};
    famp_position_t position = {};
    uint32_t received_ms = 0;
    uint16_t frames = 0;
    int8_t rssi_dbm = 0;
    bool occupied = false;
    bool position_valid = false;
};

int WriteCSBeeFlarm(char* buf, size_t capacity, const FlarmTarget& target, uint32_t now_ms);

// Local safety targets only. Never inserted into the network-exported aircraft dictionary.
class FlarmTraffic {
public:
    static constexpr size_t kCapacity = 64;
    static constexpr uint32_t kExpiryMs = 5000;
    bool Ingest(const FlarmRadioPacket& packet, uint32_t now_ms, uint64_t utc_ms,
                bool reference_valid, int32_t ref_lat, int32_t ref_lon);
    void BeginReport() { report_index_ = 0; }
    // Next record of the round, 0 when the round is complete, negative when a record did not fit.
    int NextReport(char* buf, size_t capacity, uint32_t now_ms);
    void Clear();
private:
    FlarmTarget targets_[kCapacity] = {};
    uint32_t last_sequence_ = 0;
    bool have_sequence_ = false;
    size_t report_index_ = kCapacity;
};
