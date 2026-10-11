/** @file
    FLARM FAMP (FLARM Aircraft Motion Prediction) public protocol, core algorithms.

    Copyright (C) 2026 Mark Paluch

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    Implements the FLARM FAMP Public Protocol (FFPP), FLARM Technology AG
    document FTD-116 v1.02. Section numbers below refer to that document.
    Use is subject to the FFPP license terms in Annex 1 of that document,
    reproduced in FLARM_FFPP_LICENSE.md. Those terms must accompany
    any distribution of this file.
*/

#include "flarm_famp.h"

#include <stdlib.h>
#include <string.h>

// Section 2.5
uint8_t const famp_sync[3] = {0x31, 0xFA, 0xB6};

// Section 2.7: CRC-16/CCITT, poly 0x1021, init 0xFFFF, over sync word then payload.
uint16_t famp_crc16(uint16_t crc, uint8_t const *data, unsigned len)
{
    for (unsigned i = 0; i < len; ++i) {
        crc ^= (uint16_t)(data[i] << 8);
        for (int b = 0; b < 8; ++b) {
            crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
        }
    }
    return crc;
}

// Section 1.1: little endian bit fields, bit 0 is the LSB of byte 0.
static uint32_t famp_bits(uint8_t const *b, unsigned pos, unsigned width)
{
    uint32_t v = 0;
    for (unsigned i = 0; i < width; ++i) {
        unsigned bit = pos + i;
        v |= (uint32_t)((b[bit >> 3] >> (bit & 7)) & 1) << i;
    }
    return v;
}

static uint32_t famp_le32(uint8_t const *b)
{
    return (uint32_t)b[0] | ((uint32_t)b[1] << 8) | ((uint32_t)b[2] << 16) | ((uint32_t)b[3] << 24);
}

static void famp_put_le32(uint8_t *b, uint32_t v)
{
    b[0] = (uint8_t)v;
    b[1] = (uint8_t)(v >> 8);
    b[2] = (uint8_t)(v >> 16);
    b[3] = (uint8_t)(v >> 24);
}

// Section 3.1: Extended Range Coding, unsigned. n_m mantissa bits, remaining bits exponent.
static uint32_t erc_decode(uint32_t code, unsigned n_m)
{
    uint32_t exponent = code >> n_m;
    uint32_t mantissa = code & ((1u << n_m) - 1);
    return (((1u << n_m) + mantissa) << exponent) - (1u << n_m);
}

// Section 3.1.2: MSB of the code point is the sign.
static int32_t erc_decode_signed(uint32_t code, unsigned n, unsigned n_m)
{
    uint32_t sign = 1u << (n - 1);
    if (code & sign) {
        return -(int32_t)erc_decode(code & ~sign, n_m);
    }
    return (int32_t)erc_decode(code, n_m);
}

// Section 3.2: Adaptive Coordinate Compression.
#define ACC_MODULUS (1 << 20)
#define ACC_D_LAT 52

// Section 3.2.3: longitude resolution divider, piecewise linear in whole degrees of latitude.
// The spec does not say how to get whole degrees. Truncation is right: Annex 2 case 5
// (73.79 deg) needs D = 174, rounding would give 190.
static int32_t acc_res_divider_lon(int32_t lat_deg)
{
    static int const thresholds[] = {12, 39, 56, 67, 73, 79, 82};
    int32_t res = ACC_D_LAT;
    for (int i = 0; i < 7; ++i) {
        int p = thresholds[i];
        if (lat_deg >= p) {
            int shift = i - 1; // coefficient 2^(i-1)
            if (shift >= 0) {
                res += (lat_deg - p) << shift;
            }
            else {
                res += (lat_deg - p) >> (-shift);
            }
        }
    }
    return res;
}

// Section 3.2.5: choose the grid cell closest to the reference position.
static int64_t acc_uncompress(int32_t input, int32_t reference, int32_t res_divider, int32_t modulus)
{
    int32_t ref_reduced   = reference / res_divider;
    int32_t ref_grid_base = (ref_reduced / modulus) * modulus;
    int32_t result        = input + ref_grid_base;
    for (int i = 0; i < 2; ++i) {
        if (result >= ref_reduced + modulus / 2) {
            result -= modulus;
        }
    }
    if (result <= ref_reduced - modulus / 2) {
        result += modulus;
    }
    return (int64_t)result * res_divider;
}

// Section 3.3: the next Enhanced-Privacy Random Address. Section 3.3.1 mixes the
// four bytes like one AES MixColumns step: 2*a[i] + 3*a[i+1] + a[i+2] + a[i+3] in GF(2^8).
uint32_t famp_epra_next(uint32_t addr, uint8_t rnd)
{
    uint32_t x = addr ^ rnd;
    uint8_t a[4] = {x, x >> 8, x >> 16, x >> 24};
    uint8_t b[4]; // a[i] times 2 in the field
    for (int i = 0; i < 4; ++i) {
        b[i] = (a[i] << 1) ^ (a[i] & 0x80 ? 0x1B : 0);
    }
    uint8_t r0 = b[0] ^ a[3] ^ a[2] ^ b[1] ^ a[1];
    uint8_t r1 = b[1] ^ a[0] ^ a[3] ^ b[2] ^ a[2];
    uint8_t r2 = b[2] ^ a[1] ^ a[0] ^ b[3] ^ a[3];
    uint8_t r3 = b[3] ^ a[2] ^ a[1] ^ b[0] ^ a[0];
    return r0 | (r1 << 8) | (r2 << 16) | ((uint32_t)r3 << 24);
}

#define TEA_DELTA 0x9e3779b9u

// The XXTEA mixing term, and the keyless variant the nonce pass uses.
static uint32_t xxtea_mx(uint32_t z, uint32_t y, uint32_t sum, uint32_t k)
{
    return (((z >> 5) ^ (y << 2)) + ((y >> 3) ^ (z << 4))) ^ ((sum ^ y) + (k ^ z));
}

static uint32_t nonce_mx(uint32_t z, uint32_t y, uint32_t sum)
{
    return (((z >> 5) ^ (y << 2)) + ((y >> 3) ^ (z << 4))) ^ (sum ^ y);
}

// Section 4.2.1: nonce mixing, an XXTEA-like pass over bytes. The spec fragment takes
// uint8_t* and that is literal: 16 lanes of 8 bit, the arithmetic is 32 bit. Word lanes
// (n = 4) decrypt none of the Annex 2 cases.
static void famp_mix_nonce(uint8_t *v, unsigned n, unsigned rounds)
{
    uint32_t sum = 0;
    while (rounds-- > 0) {
        sum += TEA_DELTA;
        uint32_t z = v[n - 1];
        for (unsigned p = 0; p < n - 1; ++p) {
            v[p] += nonce_mx(z, v[p + 1], sum);
            z = v[p];
        }
        v[n - 1] += nonce_mx(z, v[0], sum);
    }
}

// Section 4.2: XXTEA decryption with the spec's fixed round count instead of 6 + 52 / n.
static void famp_xxtea_decrypt(uint32_t *v, unsigned n, uint32_t const key[4], unsigned rounds)
{
    uint32_t sum = rounds * TEA_DELTA;
    while (rounds-- > 0) {
        uint32_t e = (sum >> 2) & 3;
        uint32_t y = v[0];
        for (unsigned p = n - 1; p > 0; --p) {
            v[p] -= xxtea_mx(v[p - 1], y, sum, key[(p & 3) ^ e]);
            y = v[p];
        }
        v[0] -= xxtea_mx(v[n - 1], y, sum, key[e]);
        sum -= TEA_DELTA;
    }
}

#define FAMP_XXTEA_ROUNDS 6
#define FAMP_NONCE_ROUNDS 2
#define FAMP_NONCE_CONST 0x956F6C77u

static uint32_t const famp_key[4] = {0xA5F9B21C, 0xAB3F9D12, 0xC6F34E34, 0xD72FA378};

// Decrypt payload bytes 1..16 of the packet using the 16 s interval as nonce time.
static void famp_decrypt(uint8_t const packet[FAMP_PACKET_LEN], uint32_t time_utc_16s, uint8_t payload[17])
{
    uint8_t nonce[16];
    memcpy(nonce, packet, 8); // header and version byte
    famp_put_le32(nonce + 8, time_utc_16s);
    famp_put_le32(nonce + 12, FAMP_NONCE_CONST);
    famp_mix_nonce(nonce, 16, FAMP_NONCE_ROUNDS);

    uint32_t v[4];
    for (int i = 0; i < 4; ++i) {
        v[i] = famp_le32(packet + 8 + 4 * i);
    }
    famp_xxtea_decrypt(v, 4, famp_key, FAMP_XXTEA_ROUNDS);

    payload[0] = packet[7];
    for (int i = 0; i < 4; ++i) {
        famp_put_le32(payload + 1 + 4 * i, v[i]);
    }
    for (int i = 0; i < 16; ++i) {
        payload[1 + i] ^= nonce[i];
    }
}

// A fix is taken at the top of a second and sent 0.45 to 1.2 s later. Allow for that,
// for a receiver clock that runs late, and for one that runs slightly ahead.
#define FAMP_MAX_AGE_S 3
#define FAMP_MAX_FUTURE_S 1
#define FAMP_TYPICAL_AGE_S 1

int famp_decode(uint8_t const packet[FAMP_PACKET_LEN], uint32_t utc_secs,
        famp_data_t *out)
{
    if (famp_bits(packet, 24, 4) != FAMP_MSG_TYPE) {
        return FAMP_ERR_MSG_TYPE;
    }
    // Section 3.4: only the version this unit implements. Other layouts are not assumed.
    if ((packet[7] & 0xf) != FAMP_VERSION) {
        return FAMP_ERR_VERSION;
    }

    // Sections 4.2.1 and 4.4.2: the nonce carries the 16 s interval of the fix, the payload
    // the seconds within it. The fix may sit in the previous interval and the receiver clock
    // may be off by a bit, so try the neighbours too. A wrong interval decrypts to noise,
    // which the rebuilt fix time catches nearly always. FAMP has no message authentication,
    // so when two candidates look sane the one with the more typical age wins, and a tie
    // is rejected rather than guessed.
    static int const offsets[] = {0, -1, 1};
    uint8_t payload[17];
    uint32_t fix_secs = 0;
    int best = -1; // distance from the typical age, lower is better
    int tie  = 0;
    for (int i = 0; i < 3; ++i) {
        uint32_t interval = (utc_secs >> 4) + offsets[i];
        uint8_t candidate[17];
        famp_decrypt(packet, interval, candidate);

        unsigned t = famp_bits(candidate, 8, 6);
        if (t & 3) {
            continue; // fixes are at whole seconds
        }
        uint32_t fix = interval * 16 + t / 4;
        int age      = (int32_t)(utc_secs - fix);
        if (age < -FAMP_MAX_FUTURE_S || age > FAMP_MAX_AGE_S) {
            continue;
        }
        if (famp_bits(candidate, 100, 10) > 719) {
            continue; // track is 0..359.5 degrees
        }
        int dist = abs(age - FAMP_TYPICAL_AGE_S);
        if (best < 0 || dist < best) {
            best     = dist;
            tie      = 0;
            fix_secs = fix;
            memcpy(payload, candidate, sizeof(payload));
        }
        else if (dist == best) {
            tie = 1;
        }
    }
    if (best < 0) {
        return FAMP_ERR_TIME;
    }
    if (tie) {
        return FAMP_ERR_AMBIGUOUS;
    }

    memset(out, 0, sizeof(*out));
    out->fix_secs     = fix_secs;
    out->addr_type    = famp_bits(packet, 28, 2);
    out->urgency      = famp_bits(packet, 30, 2);
    out->msg_type_ext = famp_bits(packet, 48, 6);
    out->stealth      = famp_bits(packet, 54, 1);
    out->no_track     = famp_bits(packet, 55, 1);
    out->addr         = famp_bits(packet, 0, 24);
    if (out->addr_type == FAMP_ADDR_EPRA) {
        out->epra_rnd  = famp_bits(packet, 32, 8);
        out->addr     |= famp_bits(packet, 40, 8) << 24;
        out->addr_next = famp_epra_next(out->addr, out->epra_rnd);
    }
    else if (out->addr_type == FAMP_ADDR_FLARM_EXT) {
        out->addr_ext = famp_bits(packet, 32, 16);
    }

    out->ver       = famp_bits(payload, 0, 4);
    out->ver_max   = famp_bits(payload, 4, 4);
    out->time      = famp_bits(payload, 8, 6);
    out->acft_type = famp_bits(payload, 14, 5);
    out->alt       = erc_decode(famp_bits(payload, 19, 13), 12) - 1000;

    out->lat_grid = famp_bits(payload, 32, 20);
    out->lon_grid = famp_bits(payload, 52, 20);

    out->turnrate    = erc_decode_signed(famp_bits(payload, 72, 9), 9, 6);
    out->speed       = erc_decode(famp_bits(payload, 81, 10), 8);
    out->climb       = erc_decode_signed(famp_bits(payload, 91, 9), 9, 6);
    out->track       = famp_bits(payload, 100, 10);
    out->mov_mode    = famp_bits(payload, 110, 2);
    out->acc_pos_hor = erc_decode(famp_bits(payload, 112, 6), 3);
    out->acc_pos_ver = erc_decode(famp_bits(payload, 118, 5), 2);
    out->acc_vel     = erc_decode(famp_bits(payload, 123, 5), 3);
    out->sil         = famp_bits(payload, 128, 2);
    out->sda         = famp_bits(payload, 130, 2);
    out->nic         = famp_bits(payload, 132, 4);

    return FAMP_OK;
}

int famp_resolve_position(famp_data_t const *data, int32_t ref_lat, int32_t ref_lon,
        famp_position_t *out)
{
    if (ref_lat < -900000000 || ref_lat > 900000000 ||
            ref_lon < -1800000000 || ref_lon > 1800000000 ||
            data->lat_grid >= ACC_MODULUS || data->lon_grid >= ACC_MODULUS)
        return FAMP_ERR_POSITION;
    int64_t lat = acc_uncompress(data->lat_grid, ref_lat, ACC_D_LAT, ACC_MODULUS);
    if (lat < -900000000 || lat > 900000000)
        return FAMP_ERR_POSITION;
    int32_t d_lon = acc_res_divider_lon(abs((int32_t)lat) / 10000000);
    int64_t lon = acc_uncompress(data->lon_grid, ref_lon, d_lon, ACC_MODULUS);
    if (lon < -1800000000 || lon > 1800000000)
        return FAMP_ERR_POSITION;
    out->lat = (int32_t)lat;
    out->lon = (int32_t)lon;
    return FAMP_OK;
}

char const *famp_acft_type_str(unsigned acft_type)
{
    static char const *const names[] = {
            "Undefined",
            "Glider",
            "Towplane",
            "Helicopter",
            "Parachute",
            "Dropplane",
            "Hangglider",
            "Paraglider",
            "Single-engine piston",
            "Heavy/complex fixed wing",
            "Gyrocopter",
            "Balloon",
            "Airship",
            "UAV",
            "Airfield",
            "Static obstacle",
            "eVTOL/UAM",
            "UAV Open",
            "UAV Specific",
            "UAV Certified",
    };
    if (acft_type < sizeof(names) / sizeof(names[0])) {
        return names[acft_type];
    }
    return "Reserved";
}

#ifdef _TEST

#include <stdio.h>

// Annex 2 test data of FTD-116 v1.02.
typedef struct {
    uint32_t time;
    int32_t ref_lat;
    int32_t ref_lon;
    uint8_t packet[FAMP_PACKET_LEN];
    famp_position_t position;
    famp_data_t expect;
} famp_test_case_t;

// clang-format off
static famp_test_case_t const test_cases[] = {
    {1996151268, -273941486, -67004173,
     {0xfb, 0x18, 0x0d, 0x22, 0x00, 0x00, 0xc0, 0x03, 0x42, 0x1c, 0x8b, 0x57,
      0xf5, 0x77, 0x38, 0xc2, 0x0f, 0x29, 0xe5, 0x61, 0x15, 0x40, 0xa9, 0x52},
     {-274178008, -67099756},
     {.addr_type = 2, .addr = 0x0d18fb, .urgency = 0, .stealth = 1, .no_track = 1,
      .ver = 3, .ver_max = 0, .time = 16, .acft_type = 8, .alt = 11240,
      .lat_grid = 1018802, .lon_grid = 959868, .turnrate = 304, .speed = 1760, .climb = 560,
      .track = 540, .mov_mode = 2, .acc_pos_hor = 152, .acc_pos_ver = 764, .acc_vel = 14,
      .sil = 0, .sda = 2, .nic = 0}},
    {1448385623, -350742399, -1008353120,
     {0x6f, 0x96, 0xdd, 0x02, 0x0f, 0xcd, 0x40, 0x33, 0xb1, 0x8e, 0x48, 0xec,
      0x30, 0x65, 0xb4, 0x26, 0x83, 0x95, 0x92, 0xe4, 0x5a, 0x34, 0x13, 0x8a},
     {-350558468, -1008705411},
     {.addr_type = 0, .addr = 0xcddd966f, .epra_rnd = 0x0f, .urgency = 0, .stealth = 1, .no_track = 0,
      .ver = 3, .ver_max = 3, .time = 28, .acft_type = 4, .alt = -466,
      .lat_grid = 598523, .lon_grid = 766019, .turnrate = 376, .speed = 1268, .climb = 164,
      .track = 309, .mov_mode = 0, .acc_pos_hor = 40, .acc_pos_ver = 20, .acc_vel = 112,
      .sil = 3, .sda = 0, .nic = 0}},
    {1917945369, 211847422, -999236589,
     {0xb0, 0xf3, 0xf0, 0xa2, 0x00, 0x00, 0x80, 0x03, 0x55, 0x77, 0x49, 0x0c,
      0xc8, 0xfb, 0x15, 0x4c, 0xa6, 0xa4, 0xfa, 0xde, 0x62, 0xc2, 0x72, 0xdc},
     {211932032, -999386640},
     {.addr_type = 2, .addr = 0xf0f3b0, .urgency = 2, .stealth = 0, .no_track = 1,
      .ver = 3, .ver_max = 0, .time = 36, .acft_type = 17, .alt = 1710,
      .lat_grid = 929888, .lon_grid = 1028178, .turnrate = -140, .speed = 123, .climb = 436,
      .track = 240, .mov_mode = 2, .acc_pos_hor = 168, .acc_pos_ver = 10, .acc_vel = 24,
      .sil = 1, .sda = 1, .nic = 3}},
    {2557412086, -521067434, 229778893,
     {0xa2, 0x31, 0x67, 0x72, 0xa5, 0xdf, 0x80, 0x33, 0x81, 0x67, 0xd8, 0x28,
      0xa9, 0x5c, 0xe9, 0x0f, 0x26, 0x18, 0xcf, 0xde, 0x3d, 0x6e, 0xfd, 0x39},
     {-521345084, 230040940},
     {.addr_type = 3, .addr = 0x6731a2, .addr_ext = 0xdfa5, .urgency = 1, .stealth = 0, .no_track = 1,
      .ver = 3, .ver_max = 3, .time = 24, .acft_type = 14, .alt = 3236,
      .lat_grid = 459893, .lon_grid = 609212, .turnrate = 480, .speed = 2184, .climb = -424,
      .track = 396, .mov_mode = 0, .acc_pos_hor = 888, .acc_pos_ver = 124, .acc_vel = 8,
      .sil = 0, .sda = 0, .nic = 4}},
    {3647407757, 39210529, -748523182,
     {0x95, 0xa9, 0x8a, 0xe2, 0x00, 0x00, 0xc0, 0x33, 0x21, 0xd1, 0x1f, 0xbf,
      0x5b, 0x53, 0x88, 0x39, 0x98, 0x7a, 0x67, 0xcf, 0x9b, 0x09, 0x5b, 0x3f},
     {39416364, -748571096},
     {.addr_type = 2, .addr = 0x8aa995, .urgency = 3, .stealth = 1, .no_track = 1,
      .ver = 3, .ver_max = 3, .time = 52, .acft_type = 10, .alt = 143,
      .lat_grid = 758007, .lon_grid = 284466, .turnrate = 656, .speed = 1368, .climb = 304,
      .track = 589, .mov_mode = 0, .acc_pos_hor = 22, .acc_pos_ver = 10, .acc_vel = 3,
      .sil = 3, .sda = 0, .nic = 8}},
    {1657145206, 737698753, -551572165,
     {0x20, 0x83, 0x43, 0xc2, 0x6c, 0x83, 0xc0, 0x33, 0x7b, 0xe7, 0x4b, 0x0e,
      0xd1, 0xa0, 0x16, 0x78, 0xee, 0x42, 0xa7, 0xcd, 0x5e, 0xbc, 0xb7, 0xbb},
     {737944480, -552922410},
     {.addr_type = 0, .addr = 0x83438320, .epra_rnd = 0x6c, .urgency = 3, .stealth = 1, .no_track = 1,
      .ver = 3, .ver_max = 3, .time = 24, .acft_type = 0, .alt = 2585,
      .lat_grid = 559752, .lon_grid = 1016589, .turnrate = 236, .speed = 472, .climb = 248,
      .track = 604, .mov_mode = 3, .acc_pos_hor = 280, .acc_pos_ver = 380, .acc_vel = 72,
      .sil = 2, .sda = 1, .nic = 1}},
    {1008426509, -166961921, 1500956331,
     {0xc0, 0x7f, 0xa4, 0x52, 0x00, 0x00, 0x80, 0x33, 0xac, 0xa2, 0x3c, 0x90,
      0xf1, 0x38, 0xdb, 0x4a, 0x5c, 0x44, 0xf5, 0x0a, 0x08, 0x36, 0x61, 0x95},
     {-167208288, 1500930972},
     {.addr_type = 1, .addr = 0xa47fc0, .urgency = 1, .stealth = 0, .no_track = 1,
      .ver = 3, .ver_max = 3, .time = 52, .acft_type = 4, .alt = 9028,
      .lat_grid = 978760, .lon_grid = 532042, .turnrate = -912, .speed = 3088, .climb = 9,
      .track = 603, .mov_mode = 3, .acc_pos_hor = 40, .acc_pos_ver = 4, .acc_vel = 48,
      .sil = 2, .sda = 2, .nic = 7}},
    {1891960921, 102113980, 605718156,
     {0x2c, 0xa6, 0x5a, 0x92, 0x00, 0x00, 0x40, 0x03, 0xa7, 0x44, 0x1b, 0x2c,
      0xaa, 0xfa, 0xf3, 0xf4, 0x76, 0xce, 0x37, 0x72, 0x50, 0x25, 0x0a, 0x0b},
     {102040224, 606113820},
     {.addr_type = 1, .addr = 0x5aa62c, .urgency = 2, .stealth = 1, .no_track = 0,
      .ver = 3, .ver_max = 0, .time = 36, .acft_type = 16, .alt = 5828,
      .lat_grid = 913736, .lon_grid = 121699, .turnrate = -146, .speed = 614, .climb = 60,
      .track = 536, .mov_mode = 1, .acc_pos_hor = 824, .acc_pos_ver = 0, .acc_vel = 10,
      .sil = 2, .sda = 0, .nic = 6}},
    {3243042182, -181928771, 1093943597,
     {0xa4, 0xe1, 0x2e, 0x02, 0xc4, 0xfe, 0x40, 0x33, 0xb9, 0x8b, 0xc3, 0xb8,
      0xa6, 0x55, 0xba, 0x1c, 0xea, 0xf3, 0x0f, 0x28, 0x64, 0x3c, 0x60, 0xe0},
     {-182132808, 1093637875},
     {.addr_type = 0, .addr = 0xfe2ee1a4, .epra_rnd = 0xc4, .urgency = 0, .stealth = 1, .no_track = 0,
      .ver = 3, .ver_max = 3, .time = 24, .acft_type = 18, .alt = 1950,
      .lat_grid = 691750, .lon_grid = 1009957, .turnrate = 76, .speed = 1388, .climb = 800,
      .track = 311, .mov_mode = 0, .acc_pos_hor = 440, .acc_pos_ver = 76, .acc_vel = 72,
      .sil = 0, .sda = 1, .nic = 6}},
    {2426921248, 189800145, -56924174,
     {0x28, 0xbb, 0x63, 0x22, 0x00, 0x00, 0x40, 0x03, 0x7a, 0x2d, 0x60, 0xf2,
      0x34, 0xf5, 0x0d, 0x2d, 0x30, 0x8f, 0x7a, 0x36, 0x19, 0x6b, 0xda, 0x9f},
     {189727616, -56655885},
     {.addr_type = 2, .addr = 0x63bb28, .urgency = 0, .stealth = 1, .no_track = 0,
      .ver = 3, .ver_max = 0, .time = 0, .acft_type = 19, .alt = 1293,
      .lat_grid = 502880, .lon_grid = 18469, .turnrate = 76, .speed = 256, .climb = -944,
      .track = 390, .mov_mode = 2, .acc_pos_hor = 44, .acc_pos_ver = 2, .acc_vel = 3,
      .sil = 3, .sda = 3, .nic = 10}},
};
// clang-format on

#define CHECK_FIELD(f)                                                                                 \
    do {                                                                                               \
        if (got.f != tc->expect.f) {                                                                   \
            fprintf(stderr, "case %d: %s expected %ld got %ld\n", i, #f, (long)tc->expect.f, (long)got.f); \
            fails++;                                                                                   \
        }                                                                                              \
    } while (0)

int main(void)
{
    int fails = 0;
    int n = (int)(sizeof(test_cases) / sizeof(test_cases[0]));
    for (int i = 0; i < n; ++i) {
        famp_test_case_t const *tc = &test_cases[i];
        famp_data_t got;
        int ret = famp_decode(tc->packet, tc->time, &got);
        if (ret != FAMP_OK) {
            fprintf(stderr, "case %d: famp_decode returned %d\n", i, ret);
            fails++;
            continue;
        }
        CHECK_FIELD(addr_type);
        CHECK_FIELD(addr);
        CHECK_FIELD(addr_ext);
        CHECK_FIELD(epra_rnd);
        CHECK_FIELD(urgency);
        CHECK_FIELD(stealth);
        CHECK_FIELD(no_track);
        CHECK_FIELD(ver);
        CHECK_FIELD(ver_max);
        CHECK_FIELD(time);
        CHECK_FIELD(acft_type);
        CHECK_FIELD(alt);
        CHECK_FIELD(lat_grid);
        CHECK_FIELD(lon_grid);
        famp_position_t position;
        int position_ret = famp_resolve_position(&got, tc->ref_lat, tc->ref_lon, &position);
        if (position_ret != FAMP_OK || position.lat != tc->position.lat || position.lon != tc->position.lon) {
            fprintf(stderr, "case %d: geographic reconstruction mismatch\n", i);
            fails++;
        }
        CHECK_FIELD(turnrate);
        CHECK_FIELD(speed);
        CHECK_FIELD(climb);
        CHECK_FIELD(track);
        CHECK_FIELD(mov_mode);
        CHECK_FIELD(acc_pos_hor);
        CHECK_FIELD(acc_pos_ver);
        CHECK_FIELD(acc_vel);
        CHECK_FIELD(sil);
        CHECK_FIELD(sda);
        CHECK_FIELD(nic);
        if (got.fix_secs != tc->time) {
            fprintf(stderr, "case %d: fix_secs expected %u got %u\n", i, tc->time, got.fix_secs);
            fails++;
        }
    }

    // A wrong nonce interval must never win: sweep the receiver clock over the accepted window.
    for (int i = 0; i < n; ++i) {
        famp_test_case_t const *tc = &test_cases[i];
        for (int off = -FAMP_MAX_FUTURE_S; off <= FAMP_MAX_AGE_S; ++off) {
            famp_data_t got;
            int ret = famp_decode(tc->packet, tc->time + (uint32_t)off, &got);
            if (ret != FAMP_OK || got.alt != tc->expect.alt || got.track != tc->expect.track
                    || got.lat_grid != tc->expect.lat_grid || got.lon_grid != tc->expect.lon_grid || got.time != tc->expect.time
                    || got.fix_secs != tc->time) {
                fprintf(stderr, "case %d: clock offset %+d gives ret %d alt %d track %u\n",
                        i, off, ret, got.alt, got.track);
                fails++;
            }
        }
    }

    // Clocks far off must mostly be rejected. FAMP has no message authentication and
    // adjacent nonces mix weakly, so noise passes the plausibility checks now and then.
    // This measures that residual so a loosened check shows up. Measured 9.2% on this set.
    int wrong    = 0;
    int accepted = 0;
    for (int i = 0; i < n; ++i) {
        famp_test_case_t const *tc = &test_cases[i];
        for (uint32_t k = 2; k < 200; ++k) {
            famp_data_t got;
            wrong++;
            if (famp_decode(tc->packet, tc->time + 16 * k, &got) == FAMP_OK) {
                accepted++;
            }
        }
    }
    fprintf(stderr, "flarm_famp: wrong clock accepted %d of %d\n", accepted, wrong);
    if (accepted * 100 > wrong * 15) {
        fprintf(stderr, "wrong clock false accept rate above 15%%\n");
        fails++;
    }

    // Section 3.3.1: mix32 is one AES MixColumns step, FIPS-197 vector db 13 53 45 -> 8e 4d a1 bc.
    // Byte 0 is the low byte, and the random value is XORed in first.
    if (famp_epra_next(0x455313db, 0x00) != 0xbca14d8e || famp_epra_next(0x455313d4, 0x0f) != 0xbca14d8e) {
        fprintf(stderr, "epra next mismatch: %08x\n", famp_epra_next(0x455313db, 0x00));
        fails++;
    }

    // Wrong packet type and pre-FAMP version must be rejected before decryption.
    // A newer version cannot be tested here: the version byte is part of the nonce.
    {
        uint8_t bad[FAMP_PACKET_LEN];
        famp_data_t got;
        memcpy(bad, test_cases[0].packet, FAMP_PACKET_LEN);
        bad[3] = (uint8_t)((bad[3] & 0xf0) | 0x5);
        if (famp_decode(bad, test_cases[0].time, &got) != FAMP_ERR_MSG_TYPE) {
            fprintf(stderr, "msg_type check failed\n");
            fails++;
        }
        memcpy(bad, test_cases[0].packet, FAMP_PACKET_LEN);
        bad[7] = (uint8_t)((bad[7] & 0xf0) | 0x2);
        if (famp_decode(bad, test_cases[0].time, &got) != FAMP_ERR_VERSION) {
            fprintf(stderr, "version check failed\n");
            fails++;
        }
        bad[7] = (uint8_t)((bad[7] & 0xf0) | 0x4);
        if (famp_decode(bad, test_cases[0].time, &got) != FAMP_ERR_VERSION) {
            fprintf(stderr, "future version check failed\n");
            fails++;
        }
    }

    fprintf(stderr, "flarm_famp: %d test cases, %d failures\n", n, fails);
    return fails ? 1 : 0;
}

#endif /* _TEST */
