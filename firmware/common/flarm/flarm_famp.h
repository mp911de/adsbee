/** @file
    FLARM FAMP (FLARM Aircraft Motion Prediction) public protocol, core algorithms.

    Copyright (C) 2026 Mark Paluch

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    Implements the FLARM FAMP Public Protocol (FFPP) as described in
    FLARM Technology AG document FTD-116 v1.02 (2026-09-07).
    Use of the FFPP is subject to the license terms in Annex 1 of that document:
    non-commercial, receive-only, academic/research and amateur/private use.
    The terms are reproduced in FLARM_FFPP_LICENSE.md and must accompany
    any distribution of this file.

    This unit is free of rtl_433 dependencies so it can be unit tested standalone.
*/

#ifndef INCLUDE_FLARM_FAMP_H_
#define INCLUDE_FLARM_FAMP_H_

#include <stdint.h>

/// Radio packet size in bytes: 7 byte header + 17 byte payload.
#define FAMP_PACKET_LEN 24

/// Sync word as transmitted, before Manchester encoding. Part of the CRC.
extern uint8_t const famp_sync[3];

/// FAMP protocol version this unit implements. Other versions are rejected.
#define FAMP_VERSION 3

/// Message type for FAMP payloads.
#define FAMP_MSG_TYPE 2

enum famp_addr_type {
    FAMP_ADDR_EPRA      = 0, ///< Enhanced-Privacy Random Address
    FAMP_ADDR_ICAO      = 1,
    FAMP_ADDR_FLARM     = 2,
    FAMP_ADDR_FLARM_EXT = 3,
};

enum famp_error {
    FAMP_OK             = 0,
    FAMP_ERR_MSG_TYPE   = -1, ///< msg_type is not FAMP
    FAMP_ERR_VERSION    = -2, ///< version is not FAMP_VERSION
    FAMP_ERR_TIME       = -3, ///< no nonce candidate produced a plausible time
    FAMP_ERR_AMBIGUOUS  = -4, ///< two nonce candidates look equally plausible
    FAMP_ERR_POSITION   = -5, ///< reference or reconstructed position outside geographic bounds
};

/// Decoded FAMP packet. Physical units are scaled integers, see comments.
typedef struct famp_data {
    // Header (clear text)
    uint8_t addr_type;    ///< enum famp_addr_type
    uint32_t addr;        ///< ICAO/FLARM: 24 bit. EPRA: 32 bit. FLARM ext: lower 24 bit.
    uint16_t addr_ext;    ///< FLARM ext: upper 16 bit
    uint8_t epra_rnd;     ///< EPRA: random value mixed into the next address
    uint8_t msg_type_ext;
    uint8_t urgency;      ///< 0 normal, 1 high, 2 pan-pan, 3 mayday
    uint8_t stealth;
    uint8_t no_track;
    // Payload
    uint8_t ver;
    uint8_t ver_max;      ///< 0xf: sender cannot receive
    uint8_t time;         ///< low 6 bit of Unix time in quarter seconds
    uint32_t fix_secs;    ///< Unix seconds of the GNSS fix, rebuilt from nonce interval and time
    uint8_t acft_type;
    int16_t alt;          ///< m above WGS-84 ellipsoid
    uint32_t lat_grid;    ///< 20-bit latitude coordinate within the repeating ACC grid
    uint32_t lon_grid;    ///< 20-bit longitude coordinate within the repeating ACC grid
    int16_t turnrate;     ///< 0.05 deg/s
    uint16_t speed;       ///< 0.1 m/s
    int16_t climb;        ///< 0.1 m/s
    uint16_t track;       ///< 0.5 deg
    uint8_t mov_mode;     ///< 0 unknown, 1 ground, 2 flying, 3 circling
    uint16_t acc_pos_hor; ///< 0.1 m
    uint16_t acc_pos_ver; ///< 0.25 m
    uint8_t acc_vel;      ///< 0.1 m/s
    uint8_t sil;
    uint8_t sda;
    uint8_t nic;
    uint32_t addr_next;   ///< EPRA: address after the next update
} famp_data_t;

/// Geographic position reconstructed with a nearby reference, in 1e-7 degrees.
typedef struct famp_position {
    int32_t lat;
    int32_t lon;
} famp_position_t;

/// CRC-16/CCITT, poly 0x1021. Start with crc = 0xFFFF, feed sync word then packet.
uint16_t famp_crc16(uint16_t crc, uint8_t const *data, unsigned len);

/// Decode and decrypt one FAMP packet.
/// The nonce holds the 16 s interval of the fix. Candidates around the receiver clock are
/// tried and the plaintext is checked for consistency with that clock. FAMP has no message
/// authentication, so keep the receiver clock within a second of UTC.
/// Grid coordinates are decoded without a geographic reference. They are unsigned
/// code points in [0, 1048575], not metres or offsets from the receiver.
/// @param packet the 24 raw packet bytes (header + payload) as received
/// @param utc_secs receiver's UTC time as Unix seconds, used for the nonce
/// @param out decoded data, only valid when the return value is FAMP_OK
/// @return FAMP_OK or a negative enum famp_error
int famp_decode(uint8_t const packet[FAMP_PACKET_LEN], uint32_t utc_secs,
        famp_data_t *out);

/// Resolve decoded ACC grid coordinates to the position nearest a geographic hint.
/// Latitude grid steps are 52 * 1e-7 degrees; longitude steps depend on the resolved
/// latitude. Each axis repeats every 2^20 steps. The hint must be within half a grid
/// period of the transmitter on each axis to select the correct position.
/// A distant hint can select a different cell without any decoding error.
/// @param data successfully decoded packet; its grid coordinates remain unchanged
/// @param ref_lat approximate receiver latitude in 1e-7 deg, in [-900000000, 900000000]
/// @param ref_lon approximate receiver longitude in 1e-7 deg, in [-1800000000, 1800000000]
/// @param out reconstructed WGS-84 latitude and longitude, unchanged on failure
/// @return FAMP_OK or FAMP_ERR_POSITION
int famp_resolve_position(famp_data_t const *data, int32_t ref_lat, int32_t ref_lon,
        famp_position_t *out);

/// Next EPRA address after mixing in the random value.
uint32_t famp_epra_next(uint32_t addr, uint8_t rnd);

/// Human readable aircraft type, never NULL.
char const *famp_acft_type_str(unsigned acft_type);

#endif /* INCLUDE_FLARM_FAMP_H_ */
