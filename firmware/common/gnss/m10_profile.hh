#pragma once

#include <cstdint>

#include "ublox_max_m10_config.hh"

// Settings a profile depends on.
struct M10Options {
    GNSSSettingsOptions::GNSSAntennaMode antenna = GNSSSettingsOptions::kGNSSAntennaUnknown;
    bool diagnostics = false;  // Enables the receiver's INF messages.

    bool operator==(const M10Options& other) const {
        return antenna == other.antenna && diagnostics == other.diagnostics;
    }
};

// One batch of a profile's configuration items. The profile calls Add() for every item it holds. The batch takes
// the items that start at `first` and fit, counts the rest, and builds one request from them or checks one reply.
class M10ConfigBatch {
   public:
    static constexpr uint8_t kMaxItems = 64;  // Keys per UBX-CFG-VALGET and UBX-CFG-VALSET.

    struct Window {
        uint16_t first;        // Index of the first item.
        uint8_t max_items;     // At most kMaxItems.
        uint16_t max_payload;  // Bytes the UBX-CFG-VALSET payload of the batch may take.
    };
    enum class Readback : uint8_t {
        kUnrelated,  // Not the reply to this batch: it must hold exactly the batch's keys.
        kMismatch,
        kMatch
    };

    // Counts the items without building anything.
    static M10ConfigBatch Count(Window window) { return M10ConfigBatch(Op::kCount, window); }

    // UBX-CFG-VALGET request for the RAM values of the batch. payload needs window.max_payload bytes.
    static M10ConfigBatch Keys(Window window, uint8_t* payload) {
        M10ConfigBatch batch(Op::kKeys, window);
        batch.payload_ = payload;
        batch.length_ = BeginCfgValget(payload, M10Config::kCfgGetLayerRam);
        return batch;
    }

    // UBX-CFG-VALSET that writes the batch to layers. payload needs window.max_payload bytes.
    static M10ConfigBatch Values(Window window, uint8_t* payload, uint8_t layers) {
        M10ConfigBatch batch(Op::kValues, window);
        batch.payload_ = payload;
        batch.length_ = BeginCfgValset(payload, layers);
        return batch;
    }

    // Compares the batch with a UBX-CFG-VALGET reply.
    static M10ConfigBatch Compare(Window window, const uint8_t* reply, uint16_t length) {
        M10ConfigBatch batch(Op::kCompare, window);
        batch.reply_ = reply;
        batch.length_ = length;
        batch.unrelated_ = !batch.CountReplyItems();
        return batch;
    }

    void Add(uint32_t key, uint64_t value) {
        const uint16_t index = total_++;
        if (index < window_.first || closed_) return;
        const uint8_t size = CfgValueSize(key);
        if (count_ == window_.max_items || valset_length_ + 4 + size > window_.max_payload) {
            closed_ = true;
            return;
        }
        if (count_++ == 0) first_key_ = key;
        valset_length_ += 4 + size;
        switch (op_) {
            case Op::kCount:
                break;
            case Op::kKeys:
                length_ = AppendCfgKeyValue(payload_, length_, key, 0, 0);
                break;
            case Op::kValues:
                length_ = AppendCfgKeyValue(payload_, length_, key, value, size);
                break;
            case Op::kCompare:
                CompareItem(key, value, size);
                break;
        }
    }

    uint8_t count() const { return count_; }     // Items in the batch.
    uint16_t total() const { return total_; }    // Items the profile holds.
    uint16_t length() const { return length_; }  // Payload length of the request built.
    uint32_t first_key() const { return first_key_; }

    Readback readback() const {
        if (unrelated_ || matched_ != reply_items_ || matched_ != count_) return Readback::kUnrelated;
        return mismatch_ ? Readback::kMismatch : Readback::kMatch;
    }

   private:
    enum class Op : uint8_t { kCount, kKeys, kValues, kCompare };
    static constexpr uint16_t kHeaderSize = 4;

    M10ConfigBatch(Op op, Window window) : op_(op), window_(window) {}

    static uint32_t ReadKey(const uint8_t* bytes) {
        return uint32_t(bytes[0]) | (uint32_t(bytes[1]) << 8) | (uint32_t(bytes[2]) << 16) | (uint32_t(bytes[3]) << 24);
    }

    // @retval False if the reply is not a well-formed RAM layer reply.
    bool CountReplyItems() {
        if (length_ < kHeaderSize || reply_[0] != 1 || reply_[1] != M10Config::kCfgGetLayerRam || reply_[2] ||
            reply_[3])
            return false;
        for (uint16_t pos = kHeaderSize; pos < length_; ++reply_items_) {
            if (length_ - pos < 4) return false;
            const uint8_t size = CfgValueSize(ReadKey(reply_ + pos));
            if (size == 0 || length_ - pos < 4 + size) return false;
            pos += 4 + size;
        }
        return true;
    }

    void CompareItem(uint32_t key, uint64_t value, uint8_t size) {
        if (unrelated_) return;
        for (uint16_t pos = kHeaderSize; pos < length_;) {
            const uint32_t reply_key = ReadKey(reply_ + pos);
            if (reply_key == key) {
                uint64_t reply_value = 0, wanted = 0;
                for (uint8_t i = 0; i < size; ++i) {
                    reply_value |= uint64_t(reply_[pos + 4 + i]) << (8 * i);
                    wanted |= value & (uint64_t(0xFF) << (8 * i));
                }
                if (reply_value != wanted) mismatch_ = true;
                ++matched_;
                return;
            }
            pos += 4 + CfgValueSize(reply_key);
        }
    }

    Op op_;
    Window window_;
    uint8_t* payload_ = nullptr;
    const uint8_t* reply_ = nullptr;
    uint16_t length_ = 0;
    uint16_t valset_length_ = kHeaderSize;
    uint16_t total_ = 0;
    uint8_t count_ = 0;
    uint8_t reply_items_ = 0, matched_ = 0;
    bool closed_ = false, unrelated_ = false, mismatch_ = false;
    uint32_t first_key_ = 0;
};

// The configuration one product wants in the receiver.
struct M10Profile {
    // Calls batch.Add() for every item, in the order to apply, each key once. Must enable UBX-MON-RF on UART1:
    // M10Receiver takes a missing MON-RF as a receiver that lost its configuration.
    void (*write)(const M10Options& options, M10ConfigBatch& batch);
    uint8_t layers;  // UBX-CFG-VALSET layers that receive changed items.
};
