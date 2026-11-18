#pragma once

#include <cstdint>

struct GNSSSettingsOptions {
    enum GNSSReceiverType : uint8_t {
        kGNSSReceiverNone = 0,
        kGNSSReceiverGeneric,
        kGNSSReceiverUBXMIA,
    };

    enum GNSSAntennaMode : uint8_t {
        kGNSSAntennaUnknown = 0,
        kGNSSAntennaPassive,
        kGNSSAntennaActive,
        kGNSSAntennaActiveSupervised,
        kNumGNSSAntennaModes
    };

    static constexpr const char* GNSSReceiverTypeToStr(GNSSReceiverType type) {
        switch (type) {
            case kGNSSReceiverGeneric: return "GENERIC";
            case kGNSSReceiverUBXMIA: return "UBX_MIA";
            default: return "NONE";
        }
    }

    static constexpr const char* kGNSSAntennaModeStrs[kNumGNSSAntennaModes] = {
        "UNKNOWN", "PASSIVE", "ACTIVE", "ACTIVE_SUPERVISED"};
    static constexpr const char* GNSSAntennaModeToStr(GNSSAntennaMode mode) {
        return kGNSSAntennaModeStrs[mode < kNumGNSSAntennaModes ? mode : kGNSSAntennaUnknown];
    }
};
