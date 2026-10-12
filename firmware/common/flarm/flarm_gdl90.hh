#pragma once
#include "flarm_snapshot.hh"
#include "gdl90_utils.hh"

// Emits each newer FLARM fix once. A report counts as sent when written, a lost UDP send drops that fix only.
class FlarmGDL90 {
public:
    void Update(const FlarmSnapshot& snapshot, uint32_t received_ms);
    uint16_t WriteReport(size_t index, uint8_t* buf, size_t capacity, uint32_t now_ms,
                         const AircraftDictionary& aircraft, uint32_t ownship_icao = UINT32_MAX);
    bool SuppressesAircraft(uint32_t uid, const AircraftDictionary& aircraft, uint32_t now_ms) const;
    size_t Count() const { return snapshot_.count; }
private:
    uint32_t Age(uint32_t age_at_snapshot_ms, uint32_t now_ms) const;
    bool Eligible(const FlarmReport& target, uint32_t now_ms) const;
    bool PrefersFlarm(const FlarmReport& target, const AircraftDictionary::AircraftEntry& entry, uint32_t now_ms) const;
    FlarmSnapshot snapshot_;
    uint32_t received_ms_ = 0;
    uint32_t reported_next_[FlarmSnapshot::kCapacity] = {};  // Oldest fix_secs still unreported, 0 = none sent.
    GDL90Reporter encoder_;
};
