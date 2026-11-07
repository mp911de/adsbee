#pragma once

#include <cstdint>
#include <cstring>
#include <string_view>

// Limits console logging to lines whose tag starts with one of the AT+LOG_CATEGORY prefixes. Not persisted.
class LogCategoryFilter {
   public:
    static constexpr uint8_t kMaxPrefixes = 8;
    static constexpr uint8_t kPrefixMaxLen = 64;
    static constexpr std::string_view kAll = "*";

    enum SetResult { kOk, kTooMany, kTooLong, kEmpty, kAllNotAlone };

    // Replaces the prefixes. A single "*" passes every line. Leaves the prefixes unchanged unless the result is kOk.
    SetResult Set(const std::string_view* prefixes, uint8_t count) {
        if (count == 0) return kEmpty;
        if (count == 1 && prefixes[0] == kAll) {
            count_ = 0;
            return kOk;
        }
        if (count > kMaxPrefixes) return kTooMany;
        for (uint8_t i = 0; i < count; i++) {
            if (prefixes[i].empty()) return kEmpty;
            if (prefixes[i] == kAll) return kAllNotAlone;
            if (prefixes[i].length() > kPrefixMaxLen) return kTooLong;
        }
        for (uint8_t i = 0; i < count; i++) {
            lengths_[i] = prefixes[i].length();
            memcpy(prefixes_[i], prefixes[i].data(), lengths_[i]);
        }
        count_ = count;
        return kOk;
    }

    // line starts with its tag.
    bool Passes(const char* line) const {
        if (count_ == 0) return true;
        for (uint8_t i = 0; i < count_; i++) {
            if (strncmp(line, prefixes_[i], lengths_[i]) == 0) return true;
        }
        return false;
    }

    // 0 while every line passes.
    uint8_t count() const { return count_; }
    // Not NUL terminated.
    std::string_view prefix(uint8_t index) const { return {prefixes_[index], lengths_[index]}; }

   private:
    char prefixes_[kMaxPrefixes][kPrefixMaxLen] = {};
    uint8_t lengths_[kMaxPrefixes] = {};
    uint8_t count_ = 0;
};
