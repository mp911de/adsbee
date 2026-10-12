#include "flarm_gdl90.hh"
#include <cstring>

namespace {
bool SelfAssignedAddressInUse(uint32_t address, const AircraftDictionary& aircraft) {
    for (const auto& [uid, entry] : aircraft.dict)
        if (AircraftDictionary::GetSelfAssignedAddress(entry) == address) return true;
    return false;
}
}

void FlarmGDL90::Update(const FlarmSnapshot& snapshot, uint32_t received_ms) {
    if (snapshot.generation == snapshot_.generation || !snapshot.IsValid()) return;
    uint32_t next[FlarmSnapshot::kCapacity] = {};
    for (size_t i = 0; i < snapshot.count; ++i) {
        for (size_t j = 0; j < snapshot_.count; ++j) {
            if (snapshot.targets[i].id == snapshot_.targets[j].id &&
                snapshot.targets[i].id_type == snapshot_.targets[j].id_type) {
                next[i] = reported_next_[j];
                break;
            }
        }
    }
    memcpy(reported_next_, next, sizeof(next));
    snapshot_ = snapshot;
    received_ms_ = received_ms;
}

namespace {
uint32_t PositionAge(const AircraftDictionary::AircraftEntry& entry, uint32_t now_ms) {
    if (const auto* ac = std::get_if<ModeSAircraft>(&entry);
        ac && ac->HasBitFlag(ModeSAircraft::kBitFlagPositionValid)) return now_ms - ac->last_position_update_ms;
    if (const auto* ac = std::get_if<UATAircraft>(&entry);
        ac && ac->HasBitFlag(UATAircraft::kBitFlagPositionValid)) return now_ms - ac->last_position_update_ms;
    return UINT32_MAX;
}
}

uint32_t FlarmGDL90::Age(uint32_t age_at_snapshot_ms, uint32_t now_ms) const {
    uint64_t age = uint64_t(age_at_snapshot_ms) + uint32_t(now_ms - received_ms_);
    return age > UINT32_MAX ? UINT32_MAX : uint32_t(age);
}

bool FlarmGDL90::Eligible(const FlarmReport& r, uint32_t now_ms) const {
    return r.position_valid && !r.flags && Age(r.rx_age_ms, now_ms) < kFlarmExpiryMs &&
        Age(r.position_age_ms, now_ms) <= kFlarmFreshMs;
}

bool FlarmGDL90::PrefersFlarm(const FlarmReport& r, const AircraftDictionary::AircraftEntry& entry, uint32_t now_ms) const {
    uint32_t age = PositionAge(entry, now_ms);
    uint32_t flarm_age = Age(r.position_age_ms, now_ms);
    bool fresh = age <= AircraftDictionary::kPreferredReportPositionFreshMs;
    bool flarm_fresh = flarm_age <= AircraftDictionary::kPreferredReportPositionFreshMs;
    if (fresh != flarm_fresh) return flarm_fresh;
    return !fresh && flarm_age < age;
}

bool FlarmGDL90::SuppressesAircraft(uint32_t uid, const AircraftDictionary& aircraft, uint32_t now_ms) const {
    auto it = aircraft.dict.find(uid);
    if (it == aircraft.dict.end()) return false;
    uint32_t id;
    if (AircraftDictionary::GetAddressSourceRank(it->second, id) == AircraftDictionary::kAddressSourceRankUnique)
        return false;
    for (size_t i = 0; i < snapshot_.count; ++i) {
        const auto& r = snapshot_.targets[i];
        if (r.id_type == 0 && r.id == id && Eligible(r, now_ms)) return PrefersFlarm(r, it->second, now_ms);
    }
    return false;
}

uint16_t FlarmGDL90::WriteReport(size_t index, uint8_t* buf, size_t capacity, uint32_t now_ms,
                                const AircraftDictionary& aircraft, uint32_t ownship_icao) {
    if (index >= snapshot_.count || !buf || capacity < 64) return 0;
    const auto& r = snapshot_.targets[index];
    if (r.id_type == 0 && r.id == ownship_icao) return 0;
    if (!Eligible(r, now_ms) || r.fix_secs < reported_next_[index]) return 0;
    if (r.id_type == 1 && SelfAssignedAddressInUse(r.id, aircraft)) return 0;
    if (r.id_type == 0) {
        for (const auto& [uid, entry] : aircraft.dict) {
            uint32_t id;
            if (AircraftDictionary::GetAddressSourceRank(entry, id) != AircraftDictionary::kAddressSourceRankUnique &&
                id == r.id && aircraft.IsPreferredReportForAddress(uid, now_ms) && !PrefersFlarm(r, entry, now_ms))
                return 0;
        }
    }
    GDL90Reporter::GDL90TargetReportData data;
    data.participant_address = r.id;
    data.address_type = static_cast<GDL90Reporter::GDL90TargetReportData::AddressType>(r.id_type);
    data.latitude_deg = r.lat * 1e-7; data.longitude_deg = r.lon * 1e-7;
    data.direction_deg = r.track * 0.5f;
    data.speed_kts = r.speed * 0.194384449f;
    data.vertical_rate_fpm = r.climb * 19.6850394f;
    data.SetMiscIndicator(GDL90Reporter::GDL90TargetReportData::kMiscIndicatorTTIsTrueTrackAngle,
                          false, r.movement != 1);
    data.navigation_integrity_category = r.nic <= 11 ? MIN(r.nic, 8) : 0;
    if (r.hacc < 30 && r.vacc < 16) data.navigation_accuracy_category_position = 11;
    else if (r.hacc < 100 && r.vacc < 60) data.navigation_accuracy_category_position = 10;
    else if (r.hacc < 300 && r.vacc < 180) data.navigation_accuracy_category_position = 9;
    else if (r.hacc < 926) data.navigation_accuracy_category_position = 8;
    else if (r.hacc < 1852) data.navigation_accuracy_category_position = 7;
    else data.navigation_accuracy_category_position = 6;
    static constexpr uint8_t categories[] = {0,9,0,7,11,0,12,12,0,0,7,10,10,14,0,19,0,14,14,14};
    data.emitter_category = r.aircraft_type < sizeof(categories) ? categories[r.aircraft_type] : 0;
    if (r.urgency >= 2) data.emergency_priority_code =
        GDL90Reporter::GDL90TargetReportData::kEmergencyPriorityCodeGeneralEmergency;
    uint16_t len = encoder_.WriteGDL90TargetReportMessage(buf, 64, data, false);
    if (len) reported_next_[index] = r.fix_secs + 1;
    return len;
}
