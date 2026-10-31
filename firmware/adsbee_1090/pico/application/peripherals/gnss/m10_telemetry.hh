#pragma once

#include <cstddef>
#include <cstdint>

#include "m10_diagnostics.hh"

// Turns MAX-M10 error, warning and RF status events into airgrep #M10 frames.
// Each event kind keeps only its latest occurrence until a frame sequence picks it up.
class M10Telemetry : public M10Diagnostics::Listener {
   public:
    static constexpr size_t kPayloadMaxLen = 256;
    static constexpr size_t kFragmentPayloadMaxLen = 160;
    static constexpr size_t kFrameMaxLen = 190;
    static constexpr uint8_t kMaxFragments = 63;

    void SetEnabled(bool enabled);
    void Reset();
    void DropActive() { active_kind_ = nullptr; }
    void OnInfo(bool error, const char* text, uint16_t length, bool truncated) override;
    void OnRf(const M10Diagnostics::RfStatus& rf) override;
    // Emits one complete frame. Pending events can change while the active message stays frozen.
    size_t NextFrame(char* frame, size_t capacity);

   private:
    struct Text {
        char bytes[kPayloadMaxLen];
        uint16_t length = 0;
        bool pending = false;
    };
    bool enabled_ = false;
    Text error_, warning_;
    char mon_[128] = {};
    uint16_t mon_length_ = 0;
    bool mon_pending_ = false;
    char active_[kPayloadMaxLen] = {};
    uint16_t active_length_ = 0, active_offset_ = 0;
    uint8_t escape_offset_ = 0;
    const char* active_kind_ = nullptr;
    uint8_t next_id_ = 0, active_id_ = 0, part_ = 0, count_ = 0;

    void Start(const char* kind, const char* bytes, size_t length);
    static bool NeedsEscape(uint8_t byte);
};
