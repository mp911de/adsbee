#pragma once

#include <cstdint>

// Stub shadowing adsbee_1090/pico/application/peripherals/gnss/gnss_interface.hh for the 1421
// build. The shared common/comms/comms_reporting.cpp includes this header on non-ESP32 targets
// and reads `gnss->GetUTCUnixMs()` when building GDL90 heartbeats. The 1421 has no GNSS
// receiver, so `gnss` is always nullptr and UTC is never known.
class GNSSReceiver {
   public:
    bool GetUTCUnixMs(uint64_t& utc_unix_ms) const { return false; }
};

inline GNSSReceiver* gnss = nullptr;
