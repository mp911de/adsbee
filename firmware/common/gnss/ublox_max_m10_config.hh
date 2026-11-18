#pragma once

#include <cstdint>

#include "../settings/gnss_settings_options.hh"
#include "ubx_parser.hh"

struct M10Config {
    static constexpr uint8_t kUbxClassAck = 0x05, kUbxClassCfg = 0x06;
    static constexpr uint8_t kUbxIdAckNak = 0x00, kUbxIdAckAck = 0x01;
    static constexpr uint8_t kUbxIdCfgValset = 0x8A, kUbxIdCfgValget = 0x8B;

    // UBX-CFG-VALSET takes a bitfield of layers, UBX-CFG-VALGET the number of one layer.
    enum CfgLayer : uint8_t {
        kCfgLayerRam = 0x01,    // Effective immediately.
        kCfgLayerBbr = 0x02,    // Battery-backed RAM; effective on restart, persists with V_BCKP.
        kCfgLayerFlash = 0x04,  // External flash (if present)

        kCfgGetLayerRam = 0x00,
        kCfgGetLayerBbr = 0x01,
        kCfgGetLayerFlash = 0x02,
        kCfgGetLayerDefault = 0x07,
    };

    // CFG-ITFM-ANTSETTING value for an antenna mode.
    static constexpr uint8_t AntennaSetting(GNSSSettingsOptions::GNSSAntennaMode mode) {
        return mode == GNSSSettingsOptions::kGNSSAntennaUnknown ? 0 : mode == GNSSSettingsOptions::kGNSSAntennaPassive ? 1 : 2;
    }

    // Size of each value is encoded in bits 30..28 of the key id.
    static constexpr uint32_t kCfgUart1Baudrate = 0x40520001;          // U4
    static constexpr uint32_t kCfgMsgoutNmeaGgaUart1 = 0x209100bb;     // U1
    static constexpr uint32_t kCfgMsgoutNmeaRmcUart1 = 0x209100ac;     // U1
    static constexpr uint32_t kCfgMsgoutNmeaGsaUart1 = 0x209100c0;     // U1
    static constexpr uint32_t kCfgMsgoutNmeaGsvUart1 = 0x209100c5;     // U1
    static constexpr uint32_t kCfgMsgoutNmeaGllUart1 = 0x209100ca;     // U1
    static constexpr uint32_t kCfgMsgoutNmeaVtgUart1 = 0x209100b1;     // U1
    static constexpr uint32_t kCfgNmeaProtver = 0x20930001;            // E1
    static constexpr uint32_t kCfgAnaUseAna = 0x10230001;             // L: AssistNow Autonomous enable
    static constexpr uint32_t kCfgInfmsgUbxUart1 = 0x20920002;         // X1: bit 0 error, bit 1 warning
    static constexpr uint8_t kInfmsgErrorAndWarning = 3;
    static constexpr uint32_t kCfgRateMeas = 0x30210001;
    static constexpr uint32_t kCfgRateNav = 0x30210002;
    static constexpr uint32_t kCfgSignalSbas = 0x10310020;
    static constexpr uint32_t kCfgSignalSbasL1Ca = 0x10310005;
    static constexpr uint32_t kCfgMsgoutUbxMonRfUart1 = 0x2091035a;    // U1
    static constexpr uint32_t kCfgMsgoutUbxNavSbasUart1 = 0x2091006b;  // U1
    static constexpr uint32_t kCfgI2cEnabled = 0x10510003;             // L
    static constexpr uint32_t kCfgItfmAntSetting = 0x20410010;         // E1: 0 unknown, 1 passive, 2 active
    static constexpr uint32_t kCfgHwAntCfgVoltCtrl = 0x10a3002e;       // L
    static constexpr uint32_t kCfgHwAntCfgShortDet = 0x10a3002f;       // L
    static constexpr uint32_t kCfgHwAntCfgOpenDet = 0x10a30031;        // L
    static constexpr uint32_t kCfgHwAntSupShortPin = 0x20a30037;       // U1, PIO number
    static constexpr uint32_t kCfgHwAntSupOpenPin = 0x20a30038;        // U1, PIO number
    // Diagnostic streams on UART1, every Nth navigation epoch.
    static constexpr uint8_t kDiagnosticsEpochDivider = 5;
    // TEMPORARY diagnostic (remove with the rest of the GNSS debug instrumentation).
    static constexpr uint32_t kCfgMsgoutUbxNavSatUart1 = 0x20910016;  // U1
    static constexpr uint32_t kCfgUart1OutUbx = 0x10740001;
    static constexpr uint32_t kCfgUart1OutNmea = 0x10740002;
    static constexpr uint32_t kCfgHwAntCfgPowerDown = 0x10a30033;
    static constexpr uint32_t kCfgHwAntCfgRecover = 0x10a30035;
};

// Bytes a configuration value occupies, from the storage size in bits 30..28 of its key. 0 for an invalid key.
constexpr uint8_t CfgValueSize(uint32_t key_id) {
    constexpr uint8_t kSizes[8] = {0, 1, 1, 2, 4, 8, 0, 0};
    return kSizes[(key_id >> 28) & 0x7];
}

// Append a UBX-CFG-VALSET cfgData entry (4-byte key, little-endian, followed by value_size value
// bytes, little-endian) to buf at offset. Returns the new offset.
inline uint16_t AppendCfgKeyValue(uint8_t* buf, uint16_t offset, uint32_t key_id, uint64_t value, uint8_t value_size_bytes) {
    buf[offset++] = key_id & 0xFF;
    buf[offset++] = (key_id >> 8) & 0xFF;
    buf[offset++] = (key_id >> 16) & 0xFF;
    buf[offset++] = (key_id >> 24) & 0xFF;
    for (uint8_t i = 0; i < value_size_bytes; i++) {
        buf[offset++] = (value >> (8 * i)) & 0xFF;
    }
    return offset;
}

// Start a UBX-CFG-VALSET payload for a bitfield of layers. Returns the offset of the first cfgData entry.
inline uint16_t BeginCfgValset(uint8_t* buf, uint8_t layers) {
    buf[0] = 0;  // Version 0: no transaction.
    buf[1] = layers;
    buf[2] = buf[3] = 0;
    return 4;
}

// Start a UBX-CFG-VALGET request for one layer. Returns the offset of the first key.
inline uint16_t BeginCfgValget(uint8_t* buf, uint8_t layer) {
    buf[0] = 0;  // Version 0: request.
    buf[1] = layer;
    buf[2] = buf[3] = 0;  // Position: start at the first matching item.
    return 4;
}

// @retval True for the UBX-ACK-ACK or UBX-ACK-NAK that answers a message of class cls and id. message.id tells which.
inline bool IsAckFor(const UbxMessage& message, uint8_t cls, uint8_t id) {
    return message.cls == M10Config::kUbxClassAck &&
           (message.id == M10Config::kUbxIdAckAck || message.id == M10Config::kUbxIdAckNak) && message.length == 2 &&
           message.payload[0] == cls && message.payload[1] == id;
}

// Read the value from the UBX-CFG-VALGET reply to a request for the single key key_id in layer.
// @retval False if the payload is not that reply.
inline bool ReadCfgValue(const uint8_t* payload, uint16_t length, uint8_t layer, uint32_t key_id,
                         uint8_t value_size_bytes, uint64_t& value) {
    uint8_t expected[8];
    AppendCfgKeyValue(expected, BeginCfgValget(expected, layer), key_id, 0, 0);
    expected[0] = 1;  // Version 1: reply.
    if (length != sizeof(expected) + value_size_bytes) return false;
    for (uint8_t i = 0; i < sizeof(expected); i++) {
        if (payload[i] != expected[i]) return false;
    }
    value = 0;
    for (uint8_t i = 0; i < value_size_bytes; i++) value |= static_cast<uint64_t>(payload[8 + i]) << (8 * i);
    return true;
}

// The NMEA sentences and status messages both products use on UART1. diagnostics adds the receiver's INF messages.
template <typename Write>
void WriteMessageSelection(bool diagnostics, Write write) {
    using Cfg = M10Config;
    write(Cfg::kCfgMsgoutNmeaGgaUart1, 1);
    write(Cfg::kCfgMsgoutNmeaRmcUart1, 1);
    write(Cfg::kCfgMsgoutNmeaGsaUart1, 0);
    write(Cfg::kCfgMsgoutNmeaGsvUart1, 0);
    write(Cfg::kCfgMsgoutNmeaGllUart1, 0);
    write(Cfg::kCfgMsgoutNmeaVtgUart1, 0);
    write(Cfg::kCfgMsgoutUbxMonRfUart1, Cfg::kDiagnosticsEpochDivider);
    write(Cfg::kCfgMsgoutUbxNavSbasUart1, Cfg::kDiagnosticsEpochDivider);
    write(Cfg::kCfgInfmsgUbxUart1, diagnostics ? Cfg::kInfmsgErrorAndWarning : 0);
}
