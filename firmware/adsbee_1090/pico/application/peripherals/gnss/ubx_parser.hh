#pragma once

#include <cstddef>
#include <cstdint>

// 8-bit Fletcher checksum over a UBX frame's class, id, length and payload bytes.
struct UbxChecksum {
    uint8_t a = 0, b = 0;
    void Update(uint8_t byte) {
        a = static_cast<uint8_t>(a + byte);
        b = static_cast<uint8_t>(b + a);
    }
};

// One checksum-verified UBX frame. payload holds the first stored bytes of a length byte payload.
struct UbxMessage {
    uint8_t cls = 0, id = 0;
    uint16_t length = 0, stored = 0;
    const uint8_t* payload = nullptr;
    bool is_message_type(uint8_t want_cls, uint8_t want_id) const { return cls == want_cls && id == want_id; }
};

// Byte-at-a-time UBX framer for a stream that also carries NMEA. Delivers each frame with a valid
// checksum to the listener.
class UbxParser {
   public:
    static constexpr uint8_t kSync1 = 0xB5;
    static constexpr uint8_t kSync2 = 0x62;
    static constexpr size_t kPayloadMaxLen = 640;
    // Longer length fields are corrupt. Swallowing them would hide the stream until the idle timeout.
    static constexpr uint16_t kLengthMax = 1024;
    static constexpr uint32_t kIdleTimeoutMs = 1000;

    class Listener {
       public:
        virtual ~Listener() = default;
        virtual void OnUbxMessage(const UbxMessage& message) = 0;
    };

    explicit UbxParser(Listener& listener) : listener_(listener) {}
    void IngestByte(uint8_t byte, uint32_t now_ms);
    void Resync() { state_ = State::kSync1; }
    bool InFrame() const { return state_ != State::kSync1; }

   private:
    enum class State : uint8_t { kSync1, kSync2, kClass, kID, kLengthLow, kLengthHigh, kPayload, kCkA, kCkB };
    Listener& listener_;
    State state_ = State::kSync1;
    UbxMessage message_;
    uint16_t offset_ = 0;
    UbxChecksum checksum_;
    bool checksum_a_valid_ = false;
    uint32_t last_byte_ms_ = 0;
    uint8_t payload_[kPayloadMaxLen] = {};
};
