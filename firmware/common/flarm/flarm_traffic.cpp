#include "flarm_traffic.hh"
#include <cstdio>
#include <cstring>
#include "buffer_utils.hh"

int WriteCSBeeFlarm(char* buf, size_t capacity, const FlarmTarget& t, uint32_t now_ms) {
    if (!buf || capacity < 1) return -1;
    const auto& d = t.data;
    char id[11], next[9] = "", lat[16] = "", lon[16] = "";
    if (d.addr_type == FAMP_ADDR_FLARM_EXT)
        snprintf(id, sizeof(id), "%04X%06lX", d.addr_ext, static_cast<unsigned long>(d.addr));
    else
        snprintf(id, sizeof(id), d.addr_type == FAMP_ADDR_EPRA ? "%08lX" : "%06lX",
                 static_cast<unsigned long>(d.addr));
    if (d.addr_type == FAMP_ADDR_EPRA)
        snprintf(next, sizeof(next), "%08lX", static_cast<unsigned long>(d.addr_next));
    if (t.position_valid) {
        snprintf(lat, sizeof(lat), "%.7f", t.position.lat * 1e-7);
        snprintf(lon, sizeof(lon), "%.7f", t.position.lon * 1e-7);
    }
    int n = snprintf(buf, capacity,
        "#F:1,%s,%u,%X,%u,%s,%s,%d,%.1f,%.1f,%.1f,%.2f,%u,%u,%.1f,%.2f,%.1f,%u,%u,%u,%lu,%lu,%u,%u,%d,%u,%s,",
        id, d.addr_type, unsigned(d.stealth | (d.no_track << 1)), d.acft_type, lat, lon,
        int(d.alt), d.track * 0.5, d.speed * 0.1, d.climb * 0.1, d.turnrate * 0.05,
        d.mov_mode, d.urgency, d.acc_pos_hor * 0.1, d.acc_pos_ver * 0.25, d.acc_vel * 0.1,
        d.sil, d.sda, d.nic, static_cast<unsigned long>(d.fix_secs),
        static_cast<unsigned long>(now_ms - t.received_ms), d.ver, d.ver_max, int(t.rssi_dbm),
        t.frames, next);
    if (n < 0 || size_t(n) + 7 > capacity) { buf[0] = 0; return -1; }
    return n + snprintf(buf + n, capacity - n, "%X\r\n", CalculateCRC16(reinterpret_cast<uint8_t*>(buf), n));
}

bool FlarmTraffic::Ingest(const FlarmRadioPacket& p, uint32_t now_ms, uint64_t utc_ms,
                          bool ref_valid, int32_t ref_lat, int32_t ref_lon) {
    if (have_sequence_ && uint32_t(p.sequence - last_sequence_) >= 0x80000000u) return false;
    if (have_sequence_ && p.sequence == last_sequence_) return false;
    last_sequence_ = p.sequence;
    have_sequence_ = true;
    if (p.age_ms >= kExpiryMs || utc_ms < p.age_ms || (utc_ms - p.age_ms) / 1000 > UINT32_MAX) return false;
    famp_data_t d;
    if (famp_decode(p.payload, (utc_ms - p.age_ms) / 1000, &d) != FAMP_OK || d.msg_type_ext != 0) return false;
    FlarmTarget* slot = nullptr;
    FlarmTarget* oldest = &targets_[0];
    bool same = false;
    for (auto& t : targets_) {
        bool expired = now_ms - t.received_ms >= kExpiryMs;
        if (t.occupied && t.data.addr_type == d.addr_type && t.data.addr == d.addr && t.data.addr_ext == d.addr_ext) {
            slot = &t;
            same = !expired;
            break;
        }
        if (!t.occupied || expired) slot = &t;
        if (now_ms - t.received_ms > now_ms - oldest->received_ms) oldest = &t;
    }
    if (!slot) slot = oldest;
    if (same && d.fix_secs < slot->data.fix_secs) return false;
    uint16_t frames = same ? (slot->frames == UINT16_MAX ? UINT16_MAX : slot->frames + 1) : 1;
    *slot = {};
    slot->occupied = true;
    slot->data = d;
    slot->received_ms = now_ms - p.age_ms;
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
        if (now_ms - t.received_ms >= kExpiryMs) { t = {}; continue; }
        int n = WriteCSBeeFlarm(buf, capacity, t, now_ms);
        if (n > 0) t.frames = 0;
        return n;
    }
    return 0;
}

void FlarmTraffic::Clear() {
    for (auto& target : targets_) target = {};
    have_sequence_ = false;
    last_sequence_ = 0;
    report_index_ = kCapacity;
}
