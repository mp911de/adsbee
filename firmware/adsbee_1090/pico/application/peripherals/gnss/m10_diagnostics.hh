#pragma once

#include <cstddef>
#include <cstdint>

#include "ubx_parser.hh"

// Decodes the MAX-M10 status messages into typed records and hands each one to a Listener.
class M10Diagnostics {
   public:
    static constexpr uint8_t kClassNav = 0x01, kClassInf = 0x04, kClassMon = 0x0A;
    static constexpr uint8_t kIdInfError = 0x00, kIdInfWarning = 0x01;
    static constexpr uint8_t kIdNavSbas = 0x32, kIdNavSat = 0x35;
    static constexpr uint8_t kIdMonComms = 0x36, kIdMonHw3 = 0x37, kIdMonRf = 0x38;
    static constexpr uint8_t kStoredSbasSatellites = 8;
    static constexpr uint8_t kStoredCommsPorts = 4;
    static constexpr uint8_t kStoredHw3Pins = 16;
    // Count is the wire count. Arrays retain the first entries up to their stored limit.

    struct SbasSatellite {
        uint8_t id = 0, udre = 0, system = 0, service = 0;
        int16_t range_correction_cm = 0, ionosphere_correction_cm = 0;
    };
    struct SbasStatus {
        uint32_t time_of_week_ms = 0;
        uint8_t geo = 0, mode = 0, service = 0, count = 0, flags = 0;
        int8_t system = 0;
        SbasSatellite satellites[kStoredSbasSatellites] = {};
    };
    struct CommsPort {
        uint16_t id = 0, tx_pending = 0, rx_pending = 0, overrun_errors = 0;
        uint32_t tx_bytes = 0, rx_bytes = 0, skipped_bytes = 0;
        uint8_t tx_usage = 0, tx_peak_usage = 0, rx_usage = 0, rx_peak_usage = 0;
        uint16_t messages[4] = {};
    };
    struct CommsStatus {
        uint8_t count = 0, tx_errors = 0, protocol_ids[4] = {};
        CommsPort ports[kStoredCommsPorts] = {};
    };
    struct Hw3Pin {
        uint8_t id = 0, virtual_pin = 0;
        uint16_t mask = 0;
    };
    struct Hw3Status {
        uint8_t count = 0, flags = 0;
        char hardware_version[11] = {};
        Hw3Pin pins[kStoredHw3Pins] = {};
    };
    struct RfStatus {
        uint8_t block_id = 0, jamming = 0, antenna_status = 0, antenna_power = 0;
        uint8_t cw_suppression = 0, magnitude_i = 0, magnitude_q = 0;
        int8_t offset_i = 0, offset_q = 0;
        uint16_t noise = 0, agc = 0;
        uint32_t post_status = 0;
    };
    struct SatelliteSummary {
        uint8_t count = 0, max_cno_dbhz = 0;
    };

    class Listener {
       public:
        virtual ~Listener() = default;
        // text is not NUL terminated. truncated means the parser kept only the first length bytes.
        virtual void OnInfo(bool error, const char* text, uint16_t length, bool truncated) {}
        virtual void OnSbas(const SbasStatus& sbas) {}
        virtual void OnComms(const CommsStatus& comms) {}
        virtual void OnHw3(const Hw3Status& hw3) {}
        virtual void OnRf(const RfStatus& rf) {}
        virtual void OnSatellites(const SatelliteSummary& satellites) {}
    };

    // @retval True if the message was a complete status message of a known kind.
    static bool Decode(const UbxMessage& message, Listener& listener);

   private:
    static bool DecodeSbas(const UbxMessage& message, Listener& listener);
    static bool DecodeComms(const UbxMessage& message, Listener& listener);
    static bool DecodeHw3(const UbxMessage& message, Listener& listener);
    static bool DecodeRf(const UbxMessage& message, Listener& listener);
    static bool DecodeSatellites(const UbxMessage& message, Listener& listener);
};

// Names for the enumerated UBX-MON-RF, UBX-NAV-SBAS and UBX-MON-COMMS fields, as the interface description lists them.
namespace M10Strings {
const char* Jamming(uint8_t state);
const char* AntennaStatus(uint8_t status);
const char* AntennaPower(uint8_t power);
const char* SbasMode(uint8_t mode);
const char* SbasSystem(int8_t system);
const char* Port(uint16_t id);
}  // namespace M10Strings
