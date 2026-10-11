#include "gtest/gtest.h"
#include "flarm_traffic.hh"
#include "buffer_utils.hh"
#include <array>
#include <cstring>
#include <string>
#include <vector>

namespace {
const uint8_t kPayload[] = {0xfb,0x18,0x0d,0x22,0,0,0xc0,0x03,0x42,0x1c,0x8b,0x57,
                          0xf5,0x77,0x38,0xc2,0x0f,0x29,0xe5,0x61,0x15,0x40,0xa9,0x52};
FlarmRadioPacket Packet(uint32_t sequence = 1) {
    FlarmRadioPacket p;
    p.sequence = sequence;
    p.rssi_dbm = -87;
    memcpy(p.payload, kPayload, 24);
    return p;
}
std::array<uint8_t,54> EncodeFrame() {
    std::array<uint8_t,54> encoded{};
    encoded[0] = 0x65; encoded[1] = 0x96;
    uint8_t frame[26];
    memcpy(frame, kPayload, 24);
    uint16_t crc = famp_crc16(famp_crc16(0xffff, famp_sync, 3), frame, 24);
    frame[24] = crc; frame[25] = crc >> 8;
    for (unsigned bit = 0; bit < 208; ++bit) {
        unsigned pair = frame[bit / 8] & (1 << (7 - bit % 8)) ? 1 : 2;
        encoded[2 + bit / 4] |= pair << (6 - 2 * (bit % 4));
    }
    return encoded;
}
std::vector<std::string> Fields(const char* message) {
    std::vector<std::string> fields;
    std::string s(message);
    size_t start = 0;
    for (size_t pos; (pos = s.find(',', start)) != std::string::npos; start = pos + 1)
        fields.push_back(s.substr(start, pos-start));
    fields.push_back(s.substr(start));
    return fields;
}
}

TEST(Flarm, RadioFrameAndInvertedPolarity) {
    auto frame = EncodeFrame(); uint8_t payload[24];
    ASSERT_TRUE(DecodeFlarmRadioFrame(frame.data(), frame.size(), payload));
    EXPECT_EQ(memcmp(payload, kPayload, 24), 0);
    for (auto& b : frame) b ^= 0xff;
    ASSERT_TRUE(DecodeFlarmRadioFrame(frame.data(), frame.size(), payload));
    EXPECT_EQ(memcmp(payload, kPayload, 24), 0);
}
TEST(Flarm, RejectsCorruptShortAndMalformedManchester) {
    auto frame = EncodeFrame(); uint8_t payload[24];
    EXPECT_FALSE(DecodeFlarmRadioFrame(frame.data(), 53, payload));
    frame[2] ^= 0xc0;
    EXPECT_FALSE(DecodeFlarmRadioFrame(frame.data(), frame.size(), payload));
    frame = EncodeFrame(); frame[2] &= 0x3f;
    EXPECT_FALSE(DecodeFlarmRadioFrame(frame.data(), frame.size(), payload));
    frame = EncodeFrame(); frame[0] ^= 1;
    EXPECT_FALSE(DecodeFlarmRadioFrame(frame.data(), frame.size(), payload));
}
TEST(Flarm, PrivacyMarkedAnnexPacketReachesLocalReportWithNativePrecision) {
    FlarmTraffic traffic;
    ASSERT_TRUE(traffic.Ingest(Packet(), 1000, 1996151268000ULL, true, -273941486, -67004173));
    char message[200];
    traffic.BeginReport();
    ASSERT_GT(traffic.NextReport(message, sizeof(message), 1000), 0);
    auto f = Fields(message);
    ASSERT_EQ(f.size(), 28u);
    EXPECT_EQ(f[0], "#F:1"); EXPECT_EQ(f[1], "0D18FB"); EXPECT_EQ(f[2], "2");
    EXPECT_EQ(f[3], "3"); EXPECT_EQ(f[4], "8");
    EXPECT_EQ(f[5], "-27.4178008"); EXPECT_EQ(f[6], "-6.7099756");
    EXPECT_EQ(f[7], "11240"); EXPECT_EQ(f[8], "270.0"); EXPECT_EQ(f[9], "176.0");
    EXPECT_EQ(f[10], "56.0"); EXPECT_EQ(f[11], "15.20");
    EXPECT_EQ(f[14], "15.2"); EXPECT_EQ(f[15], "191.00"); EXPECT_EQ(f[16], "1.4");
    EXPECT_EQ(f[20], "1996151268"); EXPECT_EQ(f[21], "0"); EXPECT_EQ(f[24], "-87");
    EXPECT_EQ(f[25], "1"); EXPECT_TRUE(f[26].empty());
    size_t crc_offset = std::string(message).rfind(',') + 1;
    EXPECT_EQ(std::stoul(f[27], nullptr, 16), CalculateCRC16(reinterpret_cast<uint8_t*>(message), crc_offset));
    EXPECT_EQ(f[27].substr(f[27].size()-2), "\r\n");
}
TEST(Flarm, MissingReferenceLeavesCoordinatesEmpty) {
    FlarmTraffic traffic;
    ASSERT_TRUE(traffic.Ingest(Packet(), 1000, 1996151268000ULL, false, 0, 0));
    char message[200]; traffic.BeginReport();
    ASSERT_GT(traffic.NextReport(message, sizeof(message), 1000), 0);
    auto f = Fields(message); EXPECT_TRUE(f[5].empty()); EXPECT_TRUE(f[6].empty());
}
TEST(Flarm, ReceptionAgeRepeatedReportsAndExpiry) {
    FlarmTraffic traffic; auto p = Packet(); p.age_ms = 400;
    ASSERT_TRUE(traffic.Ingest(p, 1000, 1996151268400ULL, false, 0, 0));
    char message[200]; traffic.BeginReport();
    ASSERT_GT(traffic.NextReport(message, sizeof(message), 1000), 0);
    EXPECT_EQ(Fields(message)[21], "400");
    EXPECT_EQ(traffic.NextReport(message, sizeof(message), 1000), 0);
    traffic.BeginReport();
    ASSERT_GT(traffic.NextReport(message, sizeof(message), 2000), 0);
    EXPECT_EQ(Fields(message)[21], "1400"); EXPECT_EQ(Fields(message)[25], "0");
    EXPECT_EQ(traffic.NextReport(message, sizeof(message), 2000), 0);
    traffic.BeginReport();
    EXPECT_EQ(traffic.NextReport(message, sizeof(message), 5600), 0);
    p.sequence++; p.age_ms = 5000;
    EXPECT_FALSE(traffic.Ingest(p, 6000, 1996151273000ULL, false, 0, 0));
}
TEST(Flarm, SPIRetryDoesNotDuplicateReceptionsAndSequenceWrapWorks) {
    FlarmTraffic traffic;
    ASSERT_TRUE(traffic.Ingest(Packet(UINT32_MAX-1), 1000, 1996151268000ULL, false, 0, 0));
    ASSERT_TRUE(traffic.Ingest(Packet(UINT32_MAX), 1000, 1996151268000ULL, false, 0, 0));
    EXPECT_FALSE(traffic.Ingest(Packet(UINT32_MAX-1), 1000, 1996151268000ULL, false, 0, 0));
    EXPECT_FALSE(traffic.Ingest(Packet(UINT32_MAX), 1000, 1996151268000ULL, false, 0, 0));
    ASSERT_TRUE(traffic.Ingest(Packet(0), 1000, 1996151268000ULL, false, 0, 0));
    char message[200]; traffic.BeginReport();
    ASSERT_GT(traffic.NextReport(message, sizeof(message), 1000), 0);
    EXPECT_EQ(Fields(message)[25], "3");
}
TEST(Flarm, IdentityNamespacesAndBoundedSerializer) {
    FlarmTarget t; t.data.addr = 0xffffff; t.data.addr_ext = 0xffff; t.data.addr_type = FAMP_ADDR_FLARM_EXT;
    char message[200]; ASSERT_GT(WriteCSBeeFlarm(message, sizeof(message), t, 0), 0);
    EXPECT_EQ(Fields(message)[1], "FFFFFFFFFF");
    t.data.addr_type = FAMP_ADDR_EPRA; t.data.addr = 0xffffffff; t.data.addr_next = 0x123456;
    ASSERT_GT(WriteCSBeeFlarm(message, sizeof(message), t, 0), 0);
    EXPECT_EQ(Fields(message)[1], "FFFFFFFF"); EXPECT_EQ(Fields(message)[26], "00123456");
    char small[5] = {'x','x','x','x','!'};
    EXPECT_EQ(WriteCSBeeFlarm(small, 4, t, 0), -1);
    EXPECT_EQ(small[0], 0); EXPECT_EQ(small[4], '!');
}
TEST(Flarm, ClearRequiresFreshPackets) {
    FlarmTraffic traffic; ASSERT_TRUE(traffic.Ingest(Packet(),1000,1996151268000ULL,false,0,0));
    traffic.Clear(); char message[200]; traffic.BeginReport();
    EXPECT_EQ(traffic.NextReport(message,sizeof(message),1000),0);
    ASSERT_TRUE(traffic.Ingest(Packet(),2000,1996151268000ULL,false,0,0));
}

TEST(Flarm, InvalidReconstructedPositionKeepsPrivacyMarkedTrafficVisible) {
    const uint8_t payload[] = {0xfb,0x18,0x0d,0x22,0,0,0xc0,3,0x3c,0x2c,0x93,0xda,
                               0xd4,0xe9,0x47,0xad,0x6f,0x2d,0xc6,0x98,0x48,0x74,0xf1,0x31};
    auto p = Packet(); memcpy(p.payload, payload, sizeof(payload));
    FlarmTraffic traffic;
    ASSERT_TRUE(traffic.Ingest(p, 1000, 1996151268000ULL, true, 900000000, 1800000000));
    char message[200]; traffic.BeginReport();
    ASSERT_GT(traffic.NextReport(message, sizeof(message), 1000), 0);
    auto f = Fields(message);
    EXPECT_EQ(f[1], "0D18FB"); EXPECT_EQ(f[3], "3");
    EXPECT_TRUE(f[5].empty()); EXPECT_TRUE(f[6].empty());
}
TEST(Flarm, CoordinateResolutionRejectsOverflowBeforeNarrowing) {
    famp_data_t data{}; data.lat_grid = 338168; data.lon_grid = 849746;
    famp_position_t position{123,456};
    EXPECT_EQ(famp_resolve_position(&data, 890000000, 1800000000, &position), FAMP_ERR_POSITION);
    EXPECT_EQ(position.lat, 123); EXPECT_EQ(position.lon, 456);
    EXPECT_EQ(famp_resolve_position(&data, INT32_MIN, 0, &position), FAMP_ERR_POSITION);
    EXPECT_EQ(famp_resolve_position(&data, 0, INT32_MAX, &position), FAMP_ERR_POSITION);
}

namespace {
// Reception ending at cycle phase p of a cycle whose phase 0 is utc_offset ms past boot.
uint32_t RxEnd(uint32_t utc_offset, uint32_t cycle, uint16_t p) {
    return utc_offset + cycle * 1000 + p + FlarmPhaseEstimator::kPacketMs;
}
}
TEST(FlarmPhase, StaysOnChannel1WithoutSpread) {
    FlarmPhaseEstimator e;
    EXPECT_EQ(e.Channel(0), 1);
    for (uint32_t c = 0; c < 10; ++c) e.Record(RxEnd(777, c, 460 + c * 25), 1);
    EXPECT_EQ(e.Channel(RxEnd(777, 10, 0)), 1);
    EXPECT_EQ(e.Channel(RxEnd(777, 10, 600)), 1);
}
TEST(FlarmPhase, HopsOnceSpreadCoversFrameAndHealsOnSilence) {
    FlarmPhaseEstimator e;
    uint32_t off = 777;
    for (uint32_t c = 0; c < 6; ++c) e.Record(RxEnd(off, c, 455 + c * 60), 1);  // 300 ms spread, +35 ms bias.
    for (uint16_t p : {0, 100, 280}) EXPECT_EQ(e.Channel(RxEnd(off, 7, p)), 2) << p;
    for (uint16_t p : {340, 500, 780}) EXPECT_EQ(e.Channel(RxEnd(off, 7, p)), 1) << p;
    e.Record(RxEnd(off, 8, 900), 2);  // Frame 2 reception maps into Frame 1.
    EXPECT_EQ(e.Channel(RxEnd(off, 9, 100)), 2);
    EXPECT_EQ(e.Channel(RxEnd(off, 9, 100) + FlarmPhaseEstimator::kExpiryMs), 1);
}
TEST(FlarmPhase, ContradictingTimestampsFallBack) {
    FlarmPhaseEstimator e;
    for (uint16_t p = 0; p < 1000; p += 50) e.Record(1000 + p, 1);
    EXPECT_EQ(e.Channel(3000), 1);
}
TEST(FlarmPhase, WrapAroundSecondBoundary) {
    FlarmPhaseEstimator e;
    uint32_t off = 400;  // Frame 1 straddles the ms%1000 wrap.
    for (uint32_t c = 0; c < 6; ++c) e.Record(RxEnd(off, c, 455 + c * 60), 1);
    EXPECT_EQ(e.Channel(RxEnd(off, 7, 500)), 1);
    EXPECT_EQ(e.Channel(RxEnd(off, 7, 900)), 2);
}
