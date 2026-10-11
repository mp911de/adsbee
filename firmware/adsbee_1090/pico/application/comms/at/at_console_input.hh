#pragma once

#include "comms/at_console_guard.hh"

// A receive error invalidates the whole line, including any command-looking suffix.
template <uint16_t kMaxLen>
class ATConsoleInput {
   public:
    ATConsoleGuard guard;

    bool Push(char c, uint32_t now, bool error = false) {
        if (guard.DiscardChar(now)) return false;
        if (error) {
            dropping_ = true;
            line_.Clear();
            return false;
        }
        if (dropping_) {
            if (c == '\n') dropping_ = false;
            return false;
        }
        auto result = line_.Push(c, now);
        if (result == ATLineAssembler<kMaxLen>::kOverflow) dropping_ = c != '\n';
        return result == ATLineAssembler<kMaxLen>::kLine;
    }

    std::string_view line() const { return line_.line(); }

   private:
    ATLineAssembler<kMaxLen> line_{guard};
    bool dropping_ = false;
};
