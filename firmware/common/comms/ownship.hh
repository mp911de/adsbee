#pragma once

#include <cctype>
#include <cmath>
#include <cstdint>
#include <cstring>

/**
 * Ownship identity and live data set over the console (AT+OWNSHIP, AT+OWNSHIP_BARO, AT+OWNSHIP_AHRS), for example by
 * airgrep. RAM only. Live data expires kDataTimeoutMs after it was set, so a silent source stops being reported.
 */
class Ownship {
   public:
    static constexpr uint32_t kDataTimeoutMs = 5000;
    static constexpr uint16_t kCallsignMaxLen = 8;
    static constexpr uint8_t kEmitterCategoryMax = 39;
    static constexpr uint32_t kAddressMax = 0xFFFFFF;
    static constexpr int32_t kAirspeedMaxKts = 0xFFFE;  // 0xFFFF encodes invalid.

    // Ownship as the GDL90 reporters see it, with expiry applied. Travels to the ESP32 in the RP2040 device status.
    struct __attribute__((packed)) Snapshot {
        uint32_t address = 0;  // 24 bit ICAO address, 0 = unknown.
        char callsign[kCallsignMaxLen + 1] = "ADSBEE";
        uint8_t emitter_category = 0;
        bool baro_valid = false;
        int32_t baro_altitude_ft = 0;  // Pressure altitude referenced to 1013.25 hPa.
        bool vertical_rate_valid = false;
        int32_t vertical_rate_fpm = 0;
        bool ahrs_valid = false;
        float roll_deg = NAN;     // Positive = right wing down. NaN = unknown.
        float pitch_deg = NAN;    // Positive = nose up. NaN = unknown.
        float heading_deg = NAN;  // NaN = unknown.
        bool heading_is_magnetic = false;
        int32_t indicated_airspeed_kts = -1;  // Negative = unknown.
        int32_t true_airspeed_kts = -1;       // Negative = unknown.

        // The identity alone, for when the live data can no longer be trusted.
        Snapshot WithoutLiveData() const {
            Snapshot snapshot = *this;
            snapshot.baro_valid = false;
            snapshot.vertical_rate_valid = false;
            snapshot.ahrs_valid = false;
            return snapshot;
        }
    };

    /**
     * @param[in] address 24 bit ICAO address, 0 = unknown.
     * @param[in] callsign Up to 8 letters or digits, upper cased. Empty restores the default.
     * @param[in] emitter_category GDL90 emitter category, 0 to 39.
     * @retval False if a value is out of range. Nothing changes then.
     */
    bool SetIdentity(uint32_t address, const char* callsign, uint32_t emitter_category) {
        size_t len = strlen(callsign);
        if (address > kAddressMax || len > kCallsignMaxLen || emitter_category > kEmitterCategoryMax) return false;
        for (size_t i = 0; i < len; i++) {
            if (!isalnum(static_cast<unsigned char>(callsign[i]))) return false;
        }
        data_.address = address;
        if (len == 0) {
            strcpy(data_.callsign, Snapshot().callsign);
        } else {
            for (size_t i = 0; i <= len; i++) data_.callsign[i] = toupper(static_cast<unsigned char>(callsign[i]));
        }
        data_.emitter_category = static_cast<uint8_t>(emitter_category);
        return true;
    }

    void SetBaro(int32_t altitude_ft, bool vertical_rate_valid, int32_t vertical_rate_fpm, uint32_t now_ms) {
        data_.baro_valid = true;
        data_.baro_altitude_ft = altitude_ft;
        data_.vertical_rate_valid = vertical_rate_valid;
        data_.vertical_rate_fpm = vertical_rate_fpm;
        baro_timestamp_ms_ = now_ms;
    }

    /**
     * Each value may be unknown: NaN for angles, negative for airspeeds.
     * @retval False if every value is unknown, or a known value is out of range: roll and pitch [-180, 180], heading
     * [0, 360), airspeeds up to kAirspeedMaxKts. Nothing changes then.
     */
    bool SetAHRS(float roll_deg, float pitch_deg, float heading_deg, bool heading_is_magnetic,
                 int32_t indicated_airspeed_kts, int32_t true_airspeed_kts, uint32_t now_ms) {
        auto in_range = [](float value, float min, float max, bool max_inclusive) {
            return std::isnan(value) || (value >= min && (max_inclusive ? value <= max : value < max));
        };
        if (!in_range(roll_deg, -180.0f, 180.0f, true) || !in_range(pitch_deg, -180.0f, 180.0f, true) ||
            !in_range(heading_deg, 0.0f, 360.0f, false) || indicated_airspeed_kts > kAirspeedMaxKts ||
            true_airspeed_kts > kAirspeedMaxKts) {
            return false;
        }
        if (std::isnan(roll_deg) && std::isnan(pitch_deg) && std::isnan(heading_deg) && indicated_airspeed_kts < 0 &&
            true_airspeed_kts < 0) {
            return false;
        }
        data_.ahrs_valid = true;
        data_.roll_deg = roll_deg;
        data_.pitch_deg = pitch_deg;
        data_.heading_deg = heading_deg;
        data_.heading_is_magnetic = heading_is_magnetic;
        data_.indicated_airspeed_kts = indicated_airspeed_kts;
        data_.true_airspeed_kts = true_airspeed_kts;
        ahrs_timestamp_ms_ = now_ms;
        return true;
    }

    Snapshot Get(uint32_t now_ms) const {
        Snapshot snapshot = data_;
        if (!IsFresh(data_.baro_valid, baro_timestamp_ms_, now_ms)) {
            snapshot.baro_valid = false;
            snapshot.vertical_rate_valid = false;
        }
        snapshot.ahrs_valid = IsFresh(data_.ahrs_valid, ahrs_timestamp_ms_, now_ms);
        return snapshot;
    }

    // Time since the data was set. Meaningful only while Get() reports the data valid.
    uint32_t BaroAgeMs(uint32_t now_ms) const { return now_ms - baro_timestamp_ms_; }
    uint32_t AHRSAgeMs(uint32_t now_ms) const { return now_ms - ahrs_timestamp_ms_; }

   private:
    static bool IsFresh(bool set, uint32_t timestamp_ms, uint32_t now_ms) {
        return set && now_ms - timestamp_ms < kDataTimeoutMs;
    }

    Snapshot data_;
    uint32_t baro_timestamp_ms_ = 0;
    uint32_t ahrs_timestamp_ms_ = 0;
};

extern Ownship ownship;
