#include "ubx_parser.hh"

void UbxParser::IngestByte(uint8_t byte, uint32_t now_ms) {
    if (now_ms - last_byte_ms_ > kIdleTimeoutMs) state_ = State::kSync1;
    last_byte_ms_ = now_ms;
    switch (state_) {
        case State::kSync1:
            if (byte == kSync1) state_ = State::kSync2;
            return;
        case State::kSync2:
            if (byte == kSync2) {
                checksum_ = UbxChecksum();
                state_ = State::kClass;
            } else if (byte != kSync1) {
                state_ = State::kSync1;
            }
            return;
        case State::kClass:
            message_.cls = byte;
            state_ = State::kID;
            break;
        case State::kID:
            message_.id = byte;
            state_ = State::kLengthLow;
            break;
        case State::kLengthLow:
            message_.length = byte;
            state_ = State::kLengthHigh;
            break;
        case State::kLengthHigh:
            message_.length |= static_cast<uint16_t>(byte) << 8;
            offset_ = 0;
            if (message_.length > kLengthMax) {
                state_ = State::kSync1;
                return;
            }
            state_ = message_.length ? State::kPayload : State::kCkA;
            break;
        case State::kPayload:
            if (offset_ < kPayloadMaxLen) payload_[offset_] = byte;
            if (++offset_ == message_.length) state_ = State::kCkA;
            break;
        case State::kCkA:
            checksum_a_valid_ = byte == checksum_.a;
            state_ = State::kCkB;
            return;
        case State::kCkB:
            state_ = State::kSync1;
            if (!checksum_a_valid_ || byte != checksum_.b) return;
            message_.stored = message_.length < kPayloadMaxLen ? message_.length : kPayloadMaxLen;
            message_.payload = payload_;
            listener_.OnUbxMessage(message_);
            return;
    }
    checksum_.Update(byte);
}
