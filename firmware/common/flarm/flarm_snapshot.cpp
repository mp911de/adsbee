#include "flarm_snapshot.hh"
#include <cstring>

bool FlarmSnapshot::IsValid() const {
    if (version != 1 || count > kCapacity) return false;
    for (size_t i = 0; i < count; ++i) {
        const auto& r = targets[i];
        if (r.id_type > 1 || r.id > 0xFFFFFF ||
            (r.id_type == 1 && !IsFlarmSyntheticId(r.id)) ||
            r.lat < -900000000 || r.lat > 900000000 || r.lon < -1800000000 || r.lon > 1800000000 ||
            r.flags > 3 || r.position_valid > 1 || r.aircraft_type > 31 || r.movement > 3 || r.urgency > 3 ||
            r.track > 719 || r.speed > 3832 || r.climb < -952 || r.climb > 952 ||
            r.turn < -952 || r.turn > 952 || r.hae < -1000 || r.hae > 11286 ||
            r.hacc > 1912 || r.vacc > 892 || r.sacc > 112 || r.nic > 15 || r.sil > 3 || r.sda > 3 ||
            r.version != 3 || r.max_version > 15 || r.rx_age_ms >= kFlarmExpiryMs || r.position_age_ms < r.rx_age_ms ||
            r.reserved[0] || r.reserved[1] || r.reserved[2]) return false;
        for (size_t j = 0; j < i; ++j)
            if (r.id == targets[j].id && r.id_type == targets[j].id_type) return false;
    }
    return true;
}

bool FlarmSnapshotReceiver::Write(const uint8_t* bytes, size_t length, size_t offset, uint32_t now_ms) {
    if (!bytes || !length || offset > sizeof(staging_) || length > sizeof(staging_) - offset) {
        next_offset_ = 0;
        return false;
    }
    if (offset < next_offset_ && length <= next_offset_ - offset &&
        memcmp(reinterpret_cast<const uint8_t*>(&staging_) + offset, bytes, length) == 0) return true;
    if (offset == 0) { next_offset_ = 0; started_ms_ = now_ms; }
    if (uint32_t(now_ms - started_ms_) >= 1000) { next_offset_ = 0; return false; }
    if (offset != next_offset_) { next_offset_ = 0; return false; }
    memcpy(reinterpret_cast<uint8_t*>(&staging_) + offset, bytes, length);
    next_offset_ += length;
    if (next_offset_ == sizeof(staging_)) {
        if (!staging_.IsValid()) { next_offset_ = 0; return false; }
        published_ = staging_;
        received_ms_ = started_ms_;
    }
    return true;
}
