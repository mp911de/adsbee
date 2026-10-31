#include "m10_diagnostics.hh"

#include <string>
#include <vector>

#include "gtest/gtest.h"
#include "ubx_test_utils.hh"

namespace {

// Records every decoded event. The UBX framing is covered by the UbxParser tests.
struct Capture : public M10Diagnostics::Listener {
    struct Info {
        bool error;
        std::string text;
        bool truncated;
    };
    std::vector<Info> info;
    std::vector<M10Diagnostics::SbasStatus> sbas;
    std::vector<M10Diagnostics::CommsStatus> comms;
    std::vector<M10Diagnostics::Hw3Status> hw3;
    std::vector<M10Diagnostics::RfStatus> rf;
    std::vector<M10Diagnostics::SatelliteSummary> satellites;

    void OnInfo(bool error, const char* text, uint16_t length, bool truncated) override {
        info.push_back({error, std::string(text, length), truncated});
    }
    void OnSbas(const M10Diagnostics::SbasStatus& s) override { sbas.push_back(s); }
    void OnComms(const M10Diagnostics::CommsStatus& c) override { comms.push_back(c); }
    void OnHw3(const M10Diagnostics::Hw3Status& h) override { hw3.push_back(h); }
    void OnRf(const M10Diagnostics::RfStatus& r) override { rf.push_back(r); }
    void OnSatellites(const M10Diagnostics::SatelliteSummary& s) override { satellites.push_back(s); }
};

bool Decode(Capture& capture, uint8_t cls, uint8_t id, const std::string& payload, uint16_t stored = UINT16_MAX) {
    UbxMessage message;
    message.cls = cls;
    message.id = id;
    message.length = payload.size();
    message.stored = stored < payload.size() ? stored : payload.size();
    message.payload = reinterpret_cast<const uint8_t*>(payload.data());
    return M10Diagnostics::Decode(message, capture);
}

}  // namespace

TEST(M10Diagnostics, ErrorAndWarningTextWithTruncationFlag) {
    Capture c;
    EXPECT_TRUE(Decode(c, 4, 0, "error"));
    EXPECT_TRUE(Decode(c, 4, 1, std::string(10, 'w'), 6));
    EXPECT_FALSE(Decode(c, 4, 2, "notice"));
    ASSERT_EQ(c.info.size(), 2u);
    EXPECT_TRUE(c.info[0].error);
    EXPECT_EQ(c.info[0].text, "error");
    EXPECT_FALSE(c.info[0].truncated);
    EXPECT_FALSE(c.info[1].error);
    EXPECT_EQ(c.info[1].text, "wwwwww");
    EXPECT_TRUE(c.info[1].truncated);
}

TEST(M10Diagnostics, ParsesSbasWithSignedCorrectionsAndRejectsWrongLength) {
    Capture c;
    std::string payload(24, '\0');
    payload[0] = 0xE8;
    payload[1] = 0x03;
    payload[4] = 123;
    payload[5] = 1;
    payload[6] = static_cast<char>(-1);
    payload[7] = 2;
    payload[8] = 1;
    payload[12] = 123;
    payload[14] = 3;
    payload[15] = 1;
    payload[16] = 2;
    payload[18] = static_cast<char>(0xFE);
    payload[19] = static_cast<char>(0xFF);
    payload[22] = 7;
    EXPECT_TRUE(Decode(c, 0x01, 0x32, payload));
    ASSERT_EQ(c.sbas.size(), 1u);
    EXPECT_EQ(c.sbas[0].time_of_week_ms, 1000u);
    EXPECT_EQ(c.sbas[0].geo, 123);
    EXPECT_EQ(c.sbas[0].system, -1);
    EXPECT_EQ(c.sbas[0].count, 1);
    EXPECT_EQ(c.sbas[0].satellites[0].id, 123);
    EXPECT_EQ(c.sbas[0].satellites[0].udre, 3);
    EXPECT_EQ(c.sbas[0].satellites[0].range_correction_cm, -2);
    EXPECT_EQ(c.sbas[0].satellites[0].ionosphere_correction_cm, 7);
    payload.pop_back();
    EXPECT_FALSE(Decode(c, 0x01, 0x32, payload));
    EXPECT_EQ(c.sbas.size(), 1u);
}

TEST(M10Diagnostics, ParsesCommsPortAndRejectsWrongCountOrPartialPayload) {
    Capture c;
    std::string payload(48, '\0');
    payload[1] = 1;
    payload[2] = 2;
    payload[5] = 1;
    payload[8] = 1;
    payload[10] = 3;
    payload[12] = 0x34;
    payload[13] = 0x12;
    payload[16] = 40;
    payload[18] = 4;
    payload[20] = 0x78;
    payload[21] = 0x56;
    payload[24] = 50;
    payload[26] = 9;
    payload[28] = 11;
    payload[44] = 12;
    EXPECT_TRUE(Decode(c, 0x0A, 0x36, payload));
    ASSERT_EQ(c.comms.size(), 1u);
    EXPECT_EQ(c.comms[0].count, 1);
    EXPECT_EQ(c.comms[0].tx_errors, 2);
    EXPECT_EQ(c.comms[0].protocol_ids[1], 1);
    EXPECT_EQ(c.comms[0].ports[0].id, 1);
    EXPECT_EQ(c.comms[0].ports[0].tx_pending, 3);
    EXPECT_EQ(c.comms[0].ports[0].tx_bytes, 0x1234u);
    EXPECT_EQ(c.comms[0].ports[0].tx_usage, 40);
    EXPECT_EQ(c.comms[0].ports[0].rx_pending, 4);
    EXPECT_EQ(c.comms[0].ports[0].rx_bytes, 0x5678u);
    EXPECT_EQ(c.comms[0].ports[0].rx_usage, 50);
    EXPECT_EQ(c.comms[0].ports[0].overrun_errors, 9);
    EXPECT_EQ(c.comms[0].ports[0].messages[0], 11);
    EXPECT_EQ(c.comms[0].ports[0].skipped_bytes, 12u);
    EXPECT_FALSE(Decode(c, 0x0A, 0x36, payload, 40));
    payload[1] = 2;
    EXPECT_FALSE(Decode(c, 0x0A, 0x36, payload));
    EXPECT_EQ(c.comms.size(), 1u);
}

TEST(M10Diagnostics, ParsesHw3Pins) {
    Capture c;
    std::string payload(28, '\0');
    payload[1] = 1;
    payload[2] = 3;
    payload.replace(3, 5, "HW123");
    payload[23] = 7;
    payload[24] = 0x31;
    payload[26] = 9;
    EXPECT_TRUE(Decode(c, 0x0A, 0x37, payload));
    ASSERT_EQ(c.hw3.size(), 1u);
    EXPECT_EQ(c.hw3[0].count, 1);
    EXPECT_EQ(c.hw3[0].flags, 3);
    EXPECT_STREQ(c.hw3[0].hardware_version, "HW123");
    EXPECT_EQ(c.hw3[0].pins[0].id, 7);
    EXPECT_EQ(c.hw3[0].pins[0].mask, 0x31);
    EXPECT_EQ(c.hw3[0].pins[0].virtual_pin, 9);
}

TEST(M10Diagnostics, ParsesFirstRfBlockWithSignedOffsets) {
    Capture c;
    std::string payload(28, '\0');
    payload[1] = 1;
    payload[4] = 1;
    payload[5] = 0x12 | 2;
    payload[6] = 2;
    payload[7] = 1;
    payload[8] = 0x78;
    payload[9] = 0x56;
    payload[16] = 50;
    payload[18] = 0xA0;
    payload[19] = 0x0F;
    payload[20] = 4;
    payload[21] = static_cast<char>(-128);
    payload[22] = 100;
    payload[23] = static_cast<char>(-1);
    payload[24] = 101;
    EXPECT_TRUE(Decode(c, 0x0A, 0x38, payload));
    ASSERT_EQ(c.rf.size(), 1u);
    EXPECT_EQ(c.rf[0].block_id, 1);
    EXPECT_EQ(c.rf[0].jamming, 2);
    EXPECT_EQ(c.rf[0].antenna_status, 2);
    EXPECT_EQ(c.rf[0].antenna_power, 1);
    EXPECT_EQ(c.rf[0].post_status, 0x5678u);
    EXPECT_EQ(c.rf[0].noise, 50);
    EXPECT_EQ(c.rf[0].agc, 4000);
    EXPECT_EQ(c.rf[0].cw_suppression, 4);
    EXPECT_EQ(c.rf[0].offset_i, -128);
    EXPECT_EQ(c.rf[0].magnitude_i, 100);
    EXPECT_EQ(c.rf[0].offset_q, -1);
    EXPECT_EQ(c.rf[0].magnitude_q, 101);
    payload[1] = 0;
    EXPECT_FALSE(Decode(c, 0x0A, 0x38, payload));
    payload[0] = 1;
    payload[1] = 1;
    EXPECT_FALSE(Decode(c, 0x0A, 0x38, payload));
    EXPECT_EQ(c.rf.size(), 1u);
}

TEST(M10Diagnostics, SatelliteSummaryCoversStoredRecordsOnly) {
    Capture c;
    std::string payload(8 + 3 * 12, '\0');
    payload[4] = 1;
    payload[5] = 3;
    payload[8 + 2] = 30;
    payload[8 + 12 + 2] = 45;
    payload[8 + 24 + 2] = 50;
    EXPECT_TRUE(Decode(c, 0x01, 0x35, payload));
    EXPECT_TRUE(Decode(c, 0x01, 0x35, payload, 8 + 24));
    ASSERT_EQ(c.satellites.size(), 2u);
    EXPECT_EQ(c.satellites[0].count, 3);
    EXPECT_EQ(c.satellites[0].max_cno_dbhz, 50);
    EXPECT_EQ(c.satellites[1].count, 3);
    EXPECT_EQ(c.satellites[1].max_cno_dbhz, 45);
    payload[4] = 0;
    EXPECT_FALSE(Decode(c, 0x01, 0x35, payload));
}

TEST(M10Diagnostics, UnknownMessageIsIgnored) {
    Capture c;
    EXPECT_FALSE(Decode(c, 0x05, 0x01, "\x06\x8A"));
    EXPECT_TRUE(c.info.empty() && c.rf.empty());
}
