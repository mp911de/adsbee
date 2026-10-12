#pragma once
#include <bitset>
#include <cstddef>
#include <cstdint>
#include "flarm_transport.hh"
#include "flarm_snapshot.hh"
extern "C" {
#include "flarm_famp.h"
}

class AircraftDictionary;

struct FlarmTarget {
    uint32_t id = 0;
    uint8_t id_type = 0;
    famp_data_t data = {};
    famp_position_t position = {};
    uint32_t received_ms = 0;
    uint32_t position_ms = 0;
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
    bool Ingest(const FlarmRadioPacket& packet, uint32_t now_ms, uint64_t utc_ms,
                bool reference_valid, int32_t ref_lat, int32_t ref_lon);
    const char* reject_reason() const { return reject_reason_; }  // Why the last Ingest returned false.
    void BeginReport(uint32_t now_ms) { Expire(now_ms); report_index_ = 0; }
    // Next record of the round, 0 when the round is complete, negative when a record did not fit.
    int NextReport(char* buf, size_t capacity, uint32_t now_ms);
    void Clear(uint32_t now_ms);
    void ResetSequence() { have_sequence_ = false; last_sequence_ = 0; }
    void SetAddressReservations(const AircraftDictionary& aircraft, uint32_t now_ms);
    void MakeSnapshot(FlarmSnapshot& snapshot, uint32_t now_ms);
private:
    void Expire(uint32_t now_ms);
    void Release(const FlarmTarget& target, uint32_t now_ms);
    uint32_t Allocate(uint32_t now_ms);
    std::bitset<kFlarmSyntheticIdCount> held_, reserved_;
    uint32_t released_ms_[kFlarmSyntheticIdCount] = {};
    FlarmTarget targets_[kCapacity] = {};
    uint16_t next_id_ = 0;
    uint32_t generation_ = 0;
    uint32_t last_sequence_ = 0;
    bool have_sequence_ = false;
    size_t report_index_ = kCapacity;
    const char* reject_reason_ = "";
};
