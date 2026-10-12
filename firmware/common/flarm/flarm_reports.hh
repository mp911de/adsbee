#pragma once
#include "flarm_snapshot.hh"
#include "gdl90_utils.hh"

// Shared snapshot and source selection for local GDL90 and map output.
class FlarmReports {
public:
    bool Update(FlarmSnapshotReceiver& receiver);  // Copies straight into the working snapshot.
    void Update(const FlarmSnapshot& snapshot, uint32_t received_ms);
    uint16_t WriteGDL90(size_t index, uint8_t* buf, size_t capacity, uint32_t now_ms,
                         const AircraftDictionary& aircraft, uint32_t ownship_icao = UINT32_MAX);
    int WriteJSON(size_t index, char* buf, size_t capacity, uint32_t now_ms,
                  const AircraftDictionary& aircraft) const;
    // max_fix_age_ms: oldest FLARM fix that still competes (GDL90 kFlarmFreshMs, map keeps retained fixes).
    bool SuppressesAircraft(uint32_t uid, const AircraftDictionary& aircraft, uint32_t now_ms,
                            uint32_t max_fix_age_ms = kFlarmFreshMs) const;
    size_t Count() const { return snapshot_.count; }
private:
    struct Identity { uint32_t id; uint8_t id_type; };
    size_t SaveIdentities(Identity* ids, uint32_t* next) const;
    void Remap(const Identity* ids, const uint32_t* next, size_t count);
    uint32_t Age(uint32_t age_at_snapshot_ms, uint32_t now_ms) const;
    bool Eligible(const FlarmReport& target, uint32_t now_ms, uint32_t max_fix_age_ms) const;
    bool Preferred(const FlarmReport& target, const AircraftDictionary& aircraft, uint32_t now_ms) const;
    bool PrefersFlarm(const FlarmReport& target, const AircraftDictionary::AircraftEntry& entry, uint32_t now_ms) const;
    FlarmSnapshot snapshot_;
    uint32_t received_ms_ = 0;
    uint32_t reported_next_[FlarmSnapshot::kCapacity] = {};  // Oldest fix_secs still unreported, 0 = none sent.
    GDL90Reporter encoder_;
};
