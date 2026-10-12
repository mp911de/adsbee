#include "flarm_traffic.hh"
#include <cstdio>
#include <cstring>
#include "aircraft_dictionary.hh"
#include "buffer_utils.hh"

int WriteCSBeeFlarm(char* buf, size_t capacity, const FlarmTarget& t, uint32_t now_ms) {
    if (!buf || capacity < 1) return -1;
    const auto& d = t.data;
    char id[7], lat[16] = "", lon[16] = "";
    snprintf(id, sizeof(id), "%06lX", static_cast<unsigned long>(t.id));
    if (t.position_valid) {
        snprintf(lat, sizeof(lat), "%.7f", t.position.lat * 1e-7);
        snprintf(lon, sizeof(lon), "%.7f", t.position.lon * 1e-7);
    }
    int n = snprintf(buf, capacity,
        "#F:1,%s,%u,%X,%u,%s,%s,%d,%.1f,%.1f,%.1f,%.2f,%u,%u,%.1f,%.2f,%.1f,%u,%u,%u,%lu,%lu,%u,%u,%d,%u,",
        id, t.id_type, unsigned(d.stealth | (d.no_track << 1)), d.acft_type, lat, lon,
        int(d.alt), d.track * 0.5, d.speed * 0.1, d.climb * 0.1, d.turnrate * 0.05,
        d.mov_mode, d.urgency, d.acc_pos_hor * 0.1, d.acc_pos_ver * 0.25, d.acc_vel * 0.1,
        d.sil, d.sda, d.nic, static_cast<unsigned long>(d.fix_secs),
        static_cast<unsigned long>(now_ms - t.received_ms), d.ver, d.ver_max, int(t.rssi_dbm),
        t.frames);
    if (n < 0 || size_t(n) + 7 > capacity) { buf[0] = 0; return -1; }
    return n + snprintf(buf + n, capacity - n, "%X\r\n", CalculateCRC16(reinterpret_cast<uint8_t*>(buf), n));
}

void FlarmTraffic::Release(const FlarmTarget& target, uint32_t now_ms) {
    if (target.id_type != 1 || !IsFlarmSyntheticId(target.id)) return;
    unsigned offset = target.id - kFlarmSyntheticIdBase;
    held_.set(offset);
    released_ms_[offset] = now_ms;
}

void FlarmTraffic::Expire(uint32_t now_ms) {
    for (auto& t : targets_) {
        if (t.occupied && now_ms - t.received_ms >= kFlarmExpiryMs) {
            Release(t, now_ms);
            t = {};
        }
    }
}

uint32_t FlarmTraffic::Allocate(uint32_t now_ms) {
    for (unsigned i = 0; i < kFlarmSyntheticIdCount; ++i) {
        unsigned offset = next_id_;
        next_id_ = (next_id_ + 1) % kFlarmSyntheticIdCount;
        if (reserved_[offset]) continue;
        if (held_[offset] && now_ms - released_ms_[offset] < kFlarmExpiryMs) continue;
        held_.reset(offset);
        uint32_t id = kFlarmSyntheticIdBase + offset;
        bool active = false;
        for (const auto& t : targets_) if (t.occupied && t.id_type == 1 && t.id == id) active = true;
        if (!active) return id;
    }
    return 0;
}

bool FlarmTraffic::Ingest(const FlarmRadioPacket& p, uint32_t now_ms, uint64_t utc_ms,
                          bool ref_valid, int32_t ref_lat, int32_t ref_lon) {
    if (have_sequence_ && uint32_t(p.sequence - last_sequence_) >= 0x80000000u) return false;
    if (have_sequence_ && p.sequence == last_sequence_) return false;
    last_sequence_ = p.sequence;
    have_sequence_ = true;
    if (p.age_ms >= 5000 || utc_ms < p.age_ms || (utc_ms - p.age_ms) / 1000 > UINT32_MAX) return false;
    famp_data_t d;
    if (famp_decode(p.payload, (utc_ms - p.age_ms) / 1000, &d) != FAMP_OK || d.msg_type_ext != 0) return false;
    Expire(now_ms);
    FlarmTarget* slot = nullptr;
    FlarmTarget* oldest = &targets_[0];
    bool same = false;
    for (auto& t : targets_) {
        bool expired = now_ms - t.received_ms >= kFlarmExpiryMs;
        if (t.occupied && t.data.addr_type == d.addr_type && t.data.addr == d.addr && t.data.addr_ext == d.addr_ext) {
            slot = &t;
            same = !expired;
            break;
        }
        if (!t.occupied || expired) slot = &t;
        if (now_ms - t.received_ms > now_ms - oldest->received_ms) oldest = &t;
    }
    if (!same && d.addr_type == FAMP_ADDR_EPRA) {
        FlarmTarget* next = nullptr;
        unsigned matches = 0;
        for (auto& t : targets_) {
            if (t.occupied && now_ms - t.received_ms < kFlarmExpiryMs &&
                t.data.addr_type == FAMP_ADDR_EPRA && t.data.addr_next == d.addr) {
                next = &t;
                ++matches;
            }
        }
        if (matches == 1) { slot = next; same = true; }
    }
    if (!slot) slot = oldest;
    if (same && d.fix_secs < slot->data.fix_secs) return false;
    uint16_t frames = same ? (slot->frames == UINT16_MAX ? UINT16_MAX : slot->frames + 1) : 1;
    if (!same && slot->occupied) Release(*slot, now_ms);
    uint32_t id = same ? slot->id : d.addr_type == FAMP_ADDR_ICAO ? d.addr : 0;
    if (d.addr_type != FAMP_ADDR_ICAO && id == 0) id = Allocate(now_ms);
    *slot = {};
    slot->id = id;
    slot->id_type = d.addr_type == FAMP_ADDR_ICAO ? 0 : 1;
    slot->occupied = true;
    slot->data = d;
    slot->received_ms = now_ms - p.age_ms;
    uint64_t fix_ms = uint64_t(d.fix_secs) * 1000;
    slot->position_ms = now_ms - static_cast<uint32_t>(utc_ms > fix_ms ? utc_ms - fix_ms : 0);
    slot->rssi_dbm = p.rssi_dbm;
    slot->frames = frames;
    slot->position_valid = ref_valid &&
        famp_resolve_position(&d, ref_lat, ref_lon, &slot->position) == FAMP_OK;
    return true;
}

int FlarmTraffic::NextReport(char* buf, size_t capacity, uint32_t now_ms) {
    while (report_index_ < kCapacity) {
        auto& t = targets_[report_index_++];
        if (!t.occupied) continue;
        if (t.id_type == 1 && t.id == 0) continue;
        int n = WriteCSBeeFlarm(buf, capacity, t, now_ms);
        if (n > 0) t.frames = 0;
        return n;
    }
    return 0;
}

void FlarmTraffic::Clear(uint32_t now_ms) {
    for (auto& target : targets_) {
        if (target.occupied) Release(target, now_ms);
        target = {};
    }
    ResetSequence();
    report_index_ = kCapacity;
}

void FlarmTraffic::MakeSnapshot(FlarmSnapshot& snapshot, uint32_t now_ms) {
    Expire(now_ms);
    snapshot = {};
    snapshot.generation = ++generation_;
    for (const auto& t : targets_) {
        if (!t.occupied || (t.id_type == 1 && t.id == 0)) continue;
        const auto& d = t.data;
        auto& r = snapshot.targets[snapshot.count++];
        r.id = t.id; r.id_type = t.id_type; r.fix_secs = d.fix_secs;
        r.rx_age_ms = now_ms - t.received_ms; r.position_age_ms = now_ms - t.position_ms;
        r.lat = t.position.lat; r.lon = t.position.lon; r.position_valid = t.position_valid;
        r.hae = d.alt; r.turn = d.turnrate; r.climb = d.climb; r.speed = d.speed; r.track = d.track;
        r.hacc = d.acc_pos_hor; r.vacc = d.acc_pos_ver; r.sacc = d.acc_vel;
        r.flags = d.stealth | (d.no_track << 1); r.aircraft_type = d.acft_type;
        r.movement = d.mov_mode; r.urgency = d.urgency; r.nic = d.nic; r.sil = d.sil; r.sda = d.sda;
        r.version = d.ver; r.max_version = d.ver_max; r.rssi = t.rssi_dbm; r.frames = t.frames;
    }
}

void FlarmTraffic::SetAddressReservations(const AircraftDictionary& aircraft, uint32_t now_ms) {
    reserved_.reset();
    for (const auto& [uid, entry] : aircraft.dict) {
        uint32_t address = AircraftDictionary::GetSelfAssignedAddress(entry);
        if (IsFlarmSyntheticId(address)) reserved_.set(address - kFlarmSyntheticIdBase);
    }
    for (auto& t : targets_) {
        if (!t.occupied || t.id_type != 1 || !t.id || !reserved_[t.id - kFlarmSyntheticIdBase]) continue;
        Release(t, now_ms);
        t.id = Allocate(now_ms);
    }
}
