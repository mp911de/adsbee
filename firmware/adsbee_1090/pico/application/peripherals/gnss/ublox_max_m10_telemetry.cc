#include "ublox_max_m10_telemetry.hh"

#include <cstdio>
#include <cstring>

#include "buffer_utils.hh"

void M10Telemetry::Reset() {
    error_.pending = warning_.pending = mon_pending_ = false;
    active_kind_ = nullptr;
}

void M10Telemetry::SetEnabled(bool enabled) {
    if (enabled_ != enabled) Reset();
    enabled_ = enabled;
}

void M10Telemetry::OnInfo(bool error, const char* text, uint16_t length, bool truncated) {
    if (!enabled_) return;
    Text& target = error ? error_ : warning_;
    target.length = length > kPayloadMaxLen ? kPayloadMaxLen : length;
    memcpy(target.bytes, text, target.length);
    if (truncated || length > kPayloadMaxLen) {
        constexpr char marker[] = "[truncated]";
        target.length = kPayloadMaxLen;
        memcpy(target.bytes + kPayloadMaxLen - (sizeof(marker) - 1), marker, sizeof(marker) - 1);
    }
    target.pending = true;
}

void M10Telemetry::OnRf(const M10Diagnostics::RfStatus& rf) {
    if (!enabled_) return;
    // The leading 0 is the UBX-MON-RF message version the decoder accepts.
    int n = snprintf(mon_, sizeof(mon_), "0,%u,%u,%u,%u,%lu,%u,%u,%u,%d,%u,%d,%u", rf.block_id, rf.jamming,
                     rf.antenna_status, rf.antenna_power, static_cast<unsigned long>(rf.post_status), rf.noise, rf.agc,
                     rf.cw_suppression, rf.offset_i, rf.magnitude_i, rf.offset_q, rf.magnitude_q);
    if (n <= 0 || static_cast<size_t>(n) >= sizeof(mon_)) return;
    mon_length_ = n;
    mon_pending_ = true;
}

bool M10Telemetry::NeedsEscape(uint8_t byte) { return byte == '\r' || byte == '\n' || byte == '*' || byte == '%'; }

void M10Telemetry::Start(const char* kind, const char* bytes, size_t length) {
    memcpy(active_, bytes, length);
    active_length_ = length;
    active_offset_ = escape_offset_ = 0;
    active_kind_ = kind;
    active_id_ = next_id_;
    next_id_ = (next_id_ + 1) & 63;
    size_t escaped_length = 0;
    for (size_t i = 0; i < length; ++i) escaped_length += NeedsEscape(active_[i]) ? 3 : 1;
    count_ = (escaped_length + kFragmentPayloadMaxLen - 1) / kFragmentPayloadMaxLen;
    if (count_ == 0) count_ = 1;
    part_ = 1;
    if (count_ > kMaxFragments) active_kind_ = nullptr;
}

size_t M10Telemetry::NextFrame(char* frame, size_t capacity) {
    if (!enabled_ || !frame || capacity < kFrameMaxLen + 1) return 0;
    if (!active_kind_) {
        if (error_.pending) {
            Start("ERR", error_.bytes, error_.length);
            error_.pending = false;
        } else if (warning_.pending) {
            Start("WARN", warning_.bytes, warning_.length);
            warning_.pending = false;
        } else if (mon_pending_) {
            Start("MON", mon_, mon_length_);
            mon_pending_ = false;
        }
    }
    if (!active_kind_) return 0;

    char payload[kFragmentPayloadMaxLen];
    size_t used = 0;
    constexpr char hex[] = "0123456789ABCDEF";
    while (used < sizeof(payload) && active_offset_ < active_length_) {
        uint8_t byte = active_[active_offset_];
        if (NeedsEscape(byte)) {
            char escape[] = {'%', hex[byte >> 4], hex[byte & 15]};
            payload[used++] = escape[escape_offset_++];
            if (escape_offset_ < 3) continue;
            escape_offset_ = 0;
        } else {
            payload[used++] = byte;
        }
        ++active_offset_;
    }
    int header = snprintf(frame, capacity, "#M10:%u:%s:%02u:%02u/%02u:", static_cast<unsigned>(used),
                          active_kind_, active_id_, part_, count_);
    memcpy(frame + header, payload, used);
    size_t length = header + used;
    frame[length++] = ',';
    uint16_t crc = CalculateCRC16(reinterpret_cast<const uint8_t*>(frame), length);
    int tail = snprintf(frame + length, capacity - length, "%X\r\n", crc);
    if (part_++ == count_) active_kind_ = nullptr;
    return length + tail;
}
