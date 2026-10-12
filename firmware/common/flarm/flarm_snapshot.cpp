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
    if (!bytes || !length || offset > sizeof(snapshot_) || length > sizeof(snapshot_) - offset) {
        next_offset_ = 0;
        return false;
    }
    if (offset < next_offset_ && length <= next_offset_ - offset &&
        memcmp(reinterpret_cast<const uint8_t*>(&snapshot_) + offset, bytes, length) == 0) return true;
    if (offset == 0) { next_offset_ = 0; started_ms_ = now_ms; }
    if (uint32_t(now_ms - started_ms_) >= 1000) { next_offset_ = 0; return false; }
    if (offset != next_offset_) { next_offset_ = 0; return false; }
    PFB_MUTEX_LOCK(mutex_);
    complete_ = false;
    memcpy(reinterpret_cast<uint8_t*>(&snapshot_) + offset, bytes, length);
    next_offset_ += length;
    if (next_offset_ == sizeof(snapshot_)) {
        complete_ = snapshot_.IsValid();
        if (complete_) received_ms_ = started_ms_;
        else next_offset_ = 0;
    }
    PFB_MUTEX_UNLOCK(mutex_);
    return next_offset_ != 0;
}

bool FlarmSnapshotReceiver::CopySnapshot(FlarmSnapshot& snapshot, uint32_t& received_ms) {
    PFB_MUTEX_LOCK(mutex_);
    bool changed = complete_ && snapshot.generation != snapshot_.generation;
    if (changed) {
        snapshot = snapshot_;
        received_ms = received_ms_;
    }
    PFB_MUTEX_UNLOCK(mutex_);
    return changed;
}
