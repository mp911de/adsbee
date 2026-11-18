#pragma once

#include "nmea_parser.hh"
#include "pps_clock.hh"

class GNSSNavigation {
   public:
    static constexpr uint32_t kFixTimeoutUs = 3000000;

    void Ingest(char byte, uint64_t now_us, uint64_t earliest_us, uint64_t edge_us) {
        if (byte == '$') sentence_start_us_ = earliest_us;
        const uint64_t now_ms = now_us / 1000;
        parser_.SetTimestampMs(now_ms);
        const auto sentence = parser_.IngestByte(byte);
        if (sentence == NMEAParser::kSentenceGGA) {
            last_gga_us_ = now_us;
            gga_valid_ = parser_.fix().valid && parser_.fix().last_update_timestamp_ms == now_ms;
        } else if (sentence == NMEAParser::kSentenceRMC) {
            last_rmc_us_ = now_us;
            rmc_valid_ = parser_.fix().valid && parser_.fix().last_update_timestamp_ms == now_ms;
            uint64_t utc_ms;
            // A sentence already in flight at the edge belongs to the previous epoch.
            if (edge_us && edge_us <= sentence_start_us_ && now_us - edge_us < PPSClock::kMaxLabelDelayUs &&
                parser_.GetLastUTCUnixMs(utc_ms)) {
                clock_.Label(edge_us, now_us, utc_ms);
                last_anchor_us_ = edge_us;
            } else {
                clock_.Invalidate();
            }
        }
    }

    bool HasFreshSentences(uint64_t now_us) const {
        return last_gga_us_ && now_us - last_gga_us_ < kFixTimeoutUs &&
               last_rmc_us_ && now_us - last_rmc_us_ < kFixTimeoutUs;
    }

    const NMEAParser::GNSSFix& fix() const { return parser_.fix(); }
    bool GetFixUTCUnixMs(uint64_t now_us, uint64_t& utc_ms) const {
        if (!parser_.GetUTCUnixMs(now_us / 1000, utc_ms)) return false;
        GetUTCUnixMs(now_us, utc_ms);
        return true;
    }

    const NMEAParser::GNSSFix* GetFix(uint64_t now_us) const {
        return gga_valid_ && rmc_valid_ && HasFreshSentences(now_us) ? &parser_.fix() : nullptr;
    }

    bool GetUTCUnixMs(uint64_t now_us, uint64_t& utc_ms) const {
        return now_us - last_anchor_us_ < PPSClock::kHoldoverUs && clock_.GetUTCUnixMs(now_us, utc_ms);
    }

   private:
    NMEAParser parser_;
    PPSClock clock_;
    uint64_t sentence_start_us_ = 0;
    uint64_t last_gga_us_ = 0;
    uint64_t last_rmc_us_ = 0;
    uint64_t last_anchor_us_ = 0;
    bool gga_valid_ = false;
    bool rmc_valid_ = false;
};
