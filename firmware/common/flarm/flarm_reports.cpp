#include "flarm_reports.hh"
#include <cstring>
#include <cstdio>

namespace {
bool SelfAssignedAddressInUse(uint32_t address, const AircraftDictionary& aircraft) {
    for (const auto& [uid, entry] : aircraft.dict)
        if (AircraftDictionary::GetSelfAssignedAddress(entry) == address) return true;
    return false;
}
}

size_t FlarmReports::SaveIdentities(Identity* ids, uint32_t* next) const {
    for (size_t i = 0; i < snapshot_.count; ++i) ids[i] = {snapshot_.targets[i].id, snapshot_.targets[i].id_type};
    memcpy(next, reported_next_, sizeof(reported_next_));
    return snapshot_.count;
}

// Carry the per-target emission state over to the matching entries of the new snapshot.
void FlarmReports::Remap(const Identity* ids, const uint32_t* next, size_t count) {
    memset(reported_next_, 0, sizeof(reported_next_));
    for (size_t i = 0; i < snapshot_.count; ++i) {
        for (size_t j = 0; j < count; ++j) {
            if (snapshot_.targets[i].id == ids[j].id && snapshot_.targets[i].id_type == ids[j].id_type) {
                reported_next_[i] = next[j];
                break;
            }
        }
    }
}

bool FlarmReports::Update(FlarmSnapshotReceiver& receiver) {
    Identity ids[FlarmSnapshot::kCapacity];
    uint32_t next[FlarmSnapshot::kCapacity];
    size_t count = SaveIdentities(ids, next);
    if (!receiver.CopySnapshot(snapshot_, received_ms_)) return false;
    Remap(ids, next, count);
    return true;
}

void FlarmReports::Update(const FlarmSnapshot& snapshot, uint32_t received_ms) {
    if (snapshot.generation == snapshot_.generation || !snapshot.IsValid()) return;
    Identity ids[FlarmSnapshot::kCapacity];
    uint32_t next[FlarmSnapshot::kCapacity];
    size_t count = SaveIdentities(ids, next);
    snapshot_ = snapshot;
    received_ms_ = received_ms;
    Remap(ids, next, count);
}

namespace {
const ModeSAircraft* ModeS(uint32_t icao, const AircraftDictionary& aircraft) {
    auto it = aircraft.dict.find(Aircraft::ICAOToUID(icao, Aircraft::kAircraftTypeModeS));
    return it == aircraft.dict.end() ? nullptr : std::get_if<ModeSAircraft>(&it->second);
}

uint32_t PositionAge(const AircraftDictionary::AircraftEntry& entry, uint32_t now_ms) {
    if (const auto* ac = std::get_if<ModeSAircraft>(&entry);
        ac && ac->HasBitFlag(ModeSAircraft::kBitFlagPositionValid)) return now_ms - ac->last_position_update_ms;
    if (const auto* ac = std::get_if<UATAircraft>(&entry);
        ac && ac->HasBitFlag(UATAircraft::kBitFlagPositionValid)) return now_ms - ac->last_position_update_ms;
    return UINT32_MAX;
}
}

uint32_t FlarmReports::Age(uint32_t age_at_snapshot_ms, uint32_t now_ms) const {
    uint64_t age = uint64_t(age_at_snapshot_ms) + uint32_t(now_ms - received_ms_);
    return age > UINT32_MAX ? UINT32_MAX : uint32_t(age);
}

bool FlarmReports::Eligible(const FlarmReport& r, uint32_t now_ms, uint32_t max_fix_age_ms) const {
    return r.position_valid && !r.flags && Age(r.rx_age_ms, now_ms) < kFlarmExpiryMs &&
        Age(r.position_age_ms, now_ms) <= max_fix_age_ms;
}

bool FlarmReports::PrefersFlarm(const FlarmReport& r, const AircraftDictionary::AircraftEntry& entry, uint32_t now_ms) const {
    uint32_t age = PositionAge(entry, now_ms);
    uint32_t flarm_age = Age(r.position_age_ms, now_ms);
    bool fresh = age <= AircraftDictionary::kPreferredReportPositionFreshMs;
    bool flarm_fresh = flarm_age <= AircraftDictionary::kPreferredReportPositionFreshMs;
    if (fresh != flarm_fresh) return flarm_fresh;
    return !fresh && flarm_age < age;
}

bool FlarmReports::SuppressesAircraft(uint32_t uid, const AircraftDictionary& aircraft, uint32_t now_ms, uint32_t max_fix_age_ms) const {
    auto it = aircraft.dict.find(uid);
    if (it == aircraft.dict.end()) return false;
    uint32_t id;
    if (AircraftDictionary::GetAddressSourceRank(it->second, id) == AircraftDictionary::kAddressSourceRankUnique)
        return false;
    for (size_t i = 0; i < snapshot_.count; ++i) {
        const auto& r = snapshot_.targets[i];
        if (r.id_type == 0 && r.id == id && Eligible(r, now_ms, max_fix_age_ms)) return PrefersFlarm(r, it->second, now_ms);
    }
    return false;
}

uint16_t FlarmReports::WriteGDL90(size_t index, uint8_t* buf, size_t capacity, uint32_t now_ms,
                                const AircraftDictionary& aircraft, uint32_t ownship_icao) {
    if (index >= snapshot_.count || !buf || capacity < 64) return 0;
    const auto& r = snapshot_.targets[index];
    if (r.id_type == 0 && r.id == ownship_icao) return 0;
    if (!Eligible(r, now_ms, kFlarmFreshMs) || r.fix_secs < reported_next_[index]) return 0;
    if (!Preferred(r, aircraft, now_ms)) return 0;
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
    // FAMP has no call sign: a type code plus the address tail tells a pilot what to look for and keeps targets apart.
    static constexpr char kTypeCodes[][5] = {"FLRM", "GLDR", "TOWP", "HELI", "PARA", "DROP", "HANG", "PARG", "PSTN",
                                             "HEVY", "GYRO", "BALN", "SHIP", "DRON", "AFLD", "OBST", "EVTL", "DRON",
                                             "DRON", "DRON"};
    snprintf(data.callsign, sizeof(data.callsign), "%s%04lX",
             kTypeCodes[r.aircraft_type < sizeof(kTypeCodes) / sizeof(kTypeCodes[0]) ? r.aircraft_type : 0],
             static_cast<unsigned long>(r.id & 0xFFFF));
    if (r.urgency >= 2) data.emergency_priority_code =
        GDL90Reporter::GDL90TargetReportData::kEmergencyPriorityCodeGeneralEmergency;
    uint16_t len = encoder_.WriteGDL90TargetReportMessage(buf, 64, data, false);
    if (len) reported_next_[index] = r.fix_secs + 1;
    return len;
}

bool FlarmReports::Preferred(const FlarmReport& r, const AircraftDictionary& aircraft, uint32_t now_ms) const {
    if (r.id_type == 1 && SelfAssignedAddressInUse(r.id, aircraft)) return false;
    if (r.id_type == 0) {
        for (const auto& [uid, entry] : aircraft.dict) {
            uint32_t id;
            if (AircraftDictionary::GetAddressSourceRank(entry, id) != AircraftDictionary::kAddressSourceRankUnique &&
                id == r.id && aircraft.IsPreferredReportForAddress(uid, now_ms) && !PrefersFlarm(r, entry, now_ms))
                return false;
        }
    }
    return true;
}

int FlarmReports::WriteJSON(size_t index, char* buf, size_t capacity, uint32_t now_ms,
                             const AircraftDictionary& aircraft) const {
    if (index >= snapshot_.count || !buf || !capacity) return 0;
    const auto& r = snapshot_.targets[index];
    if (!Eligible(r, now_ms, UINT32_MAX) || !Preferred(r, aircraft, now_ms)) return 0;
    int n = snprintf(buf, capacity,
        "{\"hex\":\"%s%06lx\",\"type\":\"flarm\","
        "\"lat\":%.7f,\"lon\":%.7f,\"alt_geom\":%.1f,\"gs\":%.2f,\"track\":%.1f,"
        "\"geom_rate\":%.1f%s,\"seen\":%.3f,\"seen_pos\":%.3f,\"fix_utc\":%lu,"
        "\"rssi\":%d,\"flarm_type\":%u,\"flarm_movement\":%u,\"flarm_urgency\":%u,"
        "\"flarm_turn\":%.2f,\"flarm_version\":%u,\"nic\":%u,\"sil\":%u,\"sda\":%u",
        r.id_type ? "~" : "", static_cast<unsigned long>(r.id),
        r.lat * 1e-7, r.lon * 1e-7, r.hae * 3.280839895, r.speed * 0.194384449, r.track * 0.5,
        r.climb * 19.6850394, r.movement == 1 ? ",\"on_ground\":1" : "", Age(r.rx_age_ms, now_ms) * 0.001,
        Age(r.position_age_ms, now_ms) * 0.001, static_cast<unsigned long>(r.fix_secs),
        int(r.rssi), r.aircraft_type, r.movement, r.urgency, r.turn * 0.05, r.version, r.nic, r.sil, r.sda);
    // A suppressed Mode S track with the same ICAO address keeps contributing what FLARM lacks.
    auto put = [&](const char* fmt, auto... args) {
        if (n > 0 && size_t(n) < capacity) n += snprintf(buf + n, capacity - n, fmt, args...);
    };
    if (const ModeSAircraft* ac = r.id_type == 0 ? ModeS(r.id, aircraft) : nullptr; ac) {
        if (ac->callsign[0] != '?' && ac->callsign[0] != '\0') put(",\"flight\":\"%s\"", ac->callsign);
        if (ac->squawk != ADSBTypes::kSquawkCodeNotYetReceived) put(",\"squawk\":\"%04u\"", unsigned(ac->squawk % 10000));
        if (ac->HasBitFlag(ModeSAircraft::kBitFlagBaroAltitudeValid)) put(",\"alt_baro\":%d", int(ac->baro_altitude_ft));
        if (ac->emitter_category >= 0)
            put(",\"category\":\"%c%d\"", char('A' + ac->emitter_category / 8), int(ac->emitter_category % 8));
    }
    put("}");
    if (n < 0 || size_t(n) >= capacity) { buf[0] = 0; return -1; }
    return n;
}
