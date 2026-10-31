#include "m10_diagnostics.hh"

#include <cstring>

namespace {
uint16_t ReadU2(const uint8_t* p) { return static_cast<uint16_t>(p[0] | (p[1] << 8)); }
uint32_t ReadU4(const uint8_t* p) {
    return static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8) |
           (static_cast<uint32_t>(p[2]) << 16) | (static_cast<uint32_t>(p[3]) << 24);
}
// A fully stored payload of a fixed header followed by count blocks of stride bytes.
bool HasBlocks(const UbxMessage& message, size_t header, size_t stride, uint8_t count) {
    return message.length == header + count * stride && message.stored == message.length;
}
}  // namespace

bool M10Diagnostics::Decode(const UbxMessage& message, Listener& listener) {
    if (message.cls == kClassInf && (message.id == kIdInfError || message.id == kIdInfWarning)) {
        listener.OnInfo(message.id == kIdInfError, reinterpret_cast<const char*>(message.payload), message.stored,
                        message.stored < message.length);
        return true;
    }
    if (message.is_message_type(kClassNav, kIdNavSbas)) return DecodeSbas(message, listener);
    if (message.is_message_type(kClassNav, kIdNavSat)) return DecodeSatellites(message, listener);
    if (message.is_message_type(kClassMon, kIdMonComms)) return DecodeComms(message, listener);
    if (message.is_message_type(kClassMon, kIdMonHw3)) return DecodeHw3(message, listener);
    if (message.is_message_type(kClassMon, kIdMonRf)) return DecodeRf(message, listener);
    return false;
}

bool M10Diagnostics::DecodeSbas(const UbxMessage& message, Listener& listener) {
    const uint8_t* p = message.payload;
    if (message.length < 12 || !HasBlocks(message, 12, 12, p[8])) return false;
    SbasStatus sbas;
    sbas.time_of_week_ms = ReadU4(p);
    sbas.geo = p[4];
    sbas.mode = p[5];
    sbas.system = static_cast<int8_t>(p[6]);
    sbas.service = p[7];
    sbas.count = p[8];
    sbas.flags = p[9];
    for (uint8_t i = 0; i < sbas.count && i < kStoredSbasSatellites; ++i) {
        const uint8_t* s = p + 12 + i * 12;
        sbas.satellites[i] = {s[0], s[2], s[3], s[4], static_cast<int16_t>(ReadU2(s + 6)),
                              static_cast<int16_t>(ReadU2(s + 10))};
    }
    listener.OnSbas(sbas);
    return true;
}

bool M10Diagnostics::DecodeComms(const UbxMessage& message, Listener& listener) {
    const uint8_t* p = message.payload;
    if (message.length < 8 || p[0] != 0 || !HasBlocks(message, 8, 40, p[1])) return false;
    CommsStatus comms;
    comms.count = p[1];
    comms.tx_errors = p[2];
    memcpy(comms.protocol_ids, p + 4, sizeof(comms.protocol_ids));
    for (uint8_t i = 0; i < comms.count && i < kStoredCommsPorts; ++i) {
        const uint8_t* b = p + 8 + i * 40;
        CommsPort& port = comms.ports[i];
        port.id = ReadU2(b);
        port.tx_pending = ReadU2(b + 2);
        port.tx_bytes = ReadU4(b + 4);
        port.tx_usage = b[8];
        port.tx_peak_usage = b[9];
        port.rx_pending = ReadU2(b + 10);
        port.rx_bytes = ReadU4(b + 12);
        port.rx_usage = b[16];
        port.rx_peak_usage = b[17];
        port.overrun_errors = ReadU2(b + 18);
        for (uint8_t protocol = 0; protocol < 4; ++protocol) port.messages[protocol] = ReadU2(b + 20 + protocol * 2);
        port.skipped_bytes = ReadU4(b + 36);
    }
    listener.OnComms(comms);
    return true;
}

bool M10Diagnostics::DecodeHw3(const UbxMessage& message, Listener& listener) {
    const uint8_t* p = message.payload;
    if (message.length < 22 || p[0] != 0 || !HasBlocks(message, 22, 6, p[1])) return false;
    Hw3Status hw3;
    hw3.count = p[1];
    hw3.flags = p[2];
    memcpy(hw3.hardware_version, p + 3, 10);
    for (uint8_t i = 0; i < hw3.count && i < kStoredHw3Pins; ++i) {
        const uint8_t* b = p + 22 + i * 6;
        hw3.pins[i] = {b[1], b[4], ReadU2(b + 2)};
    }
    listener.OnHw3(hw3);
    return true;
}

bool M10Diagnostics::DecodeRf(const UbxMessage& message, Listener& listener) {
    const uint8_t* p = message.payload;
    if (message.length < 28 || p[0] != 0 || p[1] == 0 || !HasBlocks(message, 4, 24, p[1])) return false;
    const uint8_t* b = p + 4;  // First RF block.
    RfStatus rf;
    rf.block_id = b[0];
    rf.jamming = b[1] & 3;
    rf.antenna_status = b[2];
    rf.antenna_power = b[3];
    rf.post_status = ReadU4(b + 4);
    rf.noise = ReadU2(b + 12);
    rf.agc = ReadU2(b + 14);
    rf.cw_suppression = b[16];
    rf.offset_i = static_cast<int8_t>(b[17]);
    rf.magnitude_i = b[18];
    rf.offset_q = static_cast<int8_t>(b[19]);
    rf.magnitude_q = b[20];
    listener.OnRf(rf);
    return true;
}

// NAV-SAT can outgrow the parser buffer. The summary covers the stored satellites only.
bool M10Diagnostics::DecodeSatellites(const UbxMessage& message, Listener& listener) {
    const uint8_t* p = message.payload;
    if (message.stored < 8 || p[4] != 1 || message.length != 8 + p[5] * 12) return false;
    SatelliteSummary satellites;
    satellites.count = p[5];
    for (size_t offset = 8; offset + 12 <= message.stored; offset += 12) {
        uint8_t cno = p[offset + 2];
        if (cno > satellites.max_cno_dbhz) satellites.max_cno_dbhz = cno;
    }
    listener.OnSatellites(satellites);
    return true;
}

namespace M10Strings {
namespace {
const char* Pick(const char* const* names, size_t count, size_t index) { return index < count ? names[index] : "?"; }
}  // namespace

const char* Jamming(uint8_t state) {
    static const char* const kNames[] = {"UNKNOWN", "NONE", "WARNING", "CRITICAL"};
    return Pick(kNames, 4, state);
}

const char* AntennaStatus(uint8_t status) {
    static const char* const kNames[] = {"INIT", "DONTKNOW", "OK", "SHORT", "OPEN"};
    return Pick(kNames, 5, status);
}

const char* AntennaPower(uint8_t power) {
    static const char* const kNames[] = {"OFF", "ON", "DONTKNOW"};
    return Pick(kNames, 3, power);
}

const char* SbasMode(uint8_t mode) {
    switch (mode) {
        case 0:
            return "DISABLED";
        case 1:
            return "ENABLED";
        case 3:
            return "TESTMODE";
        default:
            return "?";
    }
}

const char* SbasSystem(int8_t system) {
    switch (system) {
        case 0:
            return "WAAS";
        case 1:
            return "EGNOS";
        case 2:
            return "MSAS";
        case 3:
            return "GAGAN";
        case 16:
            return "GPS";
        default:
            return "UNKNOWN";
    }
}

// Port ids follow the u-blox convention: high byte selects the interface, low byte the instance.
const char* Port(uint16_t id) {
    switch (id >> 8) {
        case 0:
            return "I2C";
        case 1:
            return "UART1";
        case 2:
            return "UART2";
        case 3:
            return "USB";
        case 4:
            return "SPI";
        default:
            return "?";
    }
}
}  // namespace M10Strings
