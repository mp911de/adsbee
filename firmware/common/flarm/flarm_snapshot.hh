#pragma once
#include <cstddef>
#include <cstdint>

// Shared limits for the tracker (RP2040) and the GDL90 adapter (ESP32).
constexpr uint32_t kFlarmSyntheticIdBase = 0xF09000, kFlarmSyntheticIdCount = 1024;
constexpr uint32_t kFlarmExpiryMs = 60000;  // Target retention after the last reception.
constexpr uint32_t kFlarmFreshMs = 6000;    // Oldest fix that GDL90 still emits.
constexpr bool IsFlarmSyntheticId(uint32_t id) {
    return id >= kFlarmSyntheticIdBase && id < kFlarmSyntheticIdBase + kFlarmSyntheticIdCount;
}

// Fixed-width SPI representation. Native addresses remain on the RP2040.
struct FlarmReport {
    uint32_t id = 0, fix_secs = 0, rx_age_ms = 0, position_age_ms = 0;
    int32_t lat = 0, lon = 0;
    int16_t hae = 0, turn = 0, climb = 0;
    uint16_t speed = 0, track = 0, hacc = 0, vacc = 0, frames = 0;
    uint8_t id_type = 0, flags = 0, aircraft_type = 0, movement = 0, urgency = 0, nic = 0;
    uint8_t sacc = 0, sil = 0, sda = 0, version = 0, max_version = 0, position_valid = 0;
    int8_t rssi = 0;
    uint8_t reserved[3] = {};
};
static_assert(sizeof(FlarmReport) == 56);

struct FlarmSnapshot {
    static constexpr size_t kCapacity = 64;
    uint16_t version = 1, count = 0;
    uint32_t generation = 0;
    FlarmReport targets[kCapacity] = {};
    bool IsValid() const;
};
static_assert(sizeof(FlarmSnapshot) == 3592);

class FlarmSnapshotReceiver {
public:
    bool Write(const uint8_t* bytes, size_t length, size_t offset, uint32_t now_ms);
    const FlarmSnapshot& Snapshot() const { return published_; }
    uint32_t ReceivedMs() const { return received_ms_; }
private:
    FlarmSnapshot staging_, published_;
    size_t next_offset_ = 0;
    uint32_t started_ms_ = 0, received_ms_ = 0;
};
