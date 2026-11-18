#include <utility>
#include <vector>

#include "airgrep_m10_profile.hh"
#include "gtest/gtest.h"
#include "ublox_max_m10_config.hh"

namespace {

using Bytes = std::vector<uint8_t>;
using Cfg = M10Config;

UbxMessage Message(uint8_t cls, uint8_t id, const Bytes& payload) {
    return {cls, id, static_cast<uint16_t>(payload.size()), static_cast<uint16_t>(payload.size()), payload.data()};
}

}  // namespace

TEST(M10Config, ValsetCarriesLayersKeyAndLittleEndianValue) {
    uint8_t payload[12];
    uint16_t length = AppendCfgKeyValue(payload, BeginCfgValset(payload, Cfg::kCfgLayerRam | Cfg::kCfgLayerBbr),
                                        Cfg::kCfgUart1Baudrate, 115200, 4);
    EXPECT_EQ(Bytes(payload, payload + length), (Bytes{0, 3, 0, 0, 0x01, 0x00, 0x52, 0x40, 0x00, 0xC2, 0x01, 0x00}));
}

TEST(M10Config, ValgetRequestsOneKeyFromOneLayer) {
    uint8_t request[8];
    uint16_t length =
        AppendCfgKeyValue(request, BeginCfgValget(request, Cfg::kCfgGetLayerBbr), Cfg::kCfgSignalSbas, 0, 0);
    EXPECT_EQ(Bytes(request, request + length), (Bytes{0, 1, 0, 0, 0x20, 0x00, 0x31, 0x10}));
}

TEST(M10Config, ReadsValueFromMatchingValgetReply) {
    const Bytes reply{1, 1, 0, 0, 0x01, 0x00, 0x52, 0x40, 0x00, 0xC2, 0x01, 0x00};
    uint64_t value = 0;
    EXPECT_TRUE(ReadCfgValue(reply.data(), reply.size(), Cfg::kCfgGetLayerBbr, Cfg::kCfgUart1Baudrate, 4, value));
    EXPECT_EQ(value, 115200u);
}

TEST(M10Config, RejectsValgetReplyForAnotherRequest) {
    const Bytes reply{1, 1, 0, 0, 0x01, 0x00, 0x52, 0x40, 0x00, 0xC2, 0x01, 0x00};
    uint64_t value = 7;
    EXPECT_FALSE(ReadCfgValue(reply.data(), reply.size(), Cfg::kCfgGetLayerRam, Cfg::kCfgUart1Baudrate, 4, value));
    EXPECT_FALSE(ReadCfgValue(reply.data(), reply.size(), Cfg::kCfgGetLayerBbr, Cfg::kCfgSignalSbas, 4, value));
    EXPECT_FALSE(ReadCfgValue(reply.data(), reply.size(), Cfg::kCfgGetLayerBbr, Cfg::kCfgUart1Baudrate, 2, value));
    EXPECT_FALSE(ReadCfgValue(reply.data(), 8, Cfg::kCfgGetLayerBbr, Cfg::kCfgUart1Baudrate, 4, value));
    Bytes request = reply;
    request[0] = 0;
    EXPECT_FALSE(ReadCfgValue(request.data(), request.size(), Cfg::kCfgGetLayerBbr, Cfg::kCfgUart1Baudrate, 4, value));
    EXPECT_EQ(value, 7u);
}

TEST(M10Config, MatchesAckAndNakForTheAwaitedMessage) {
    const Bytes valset{Cfg::kUbxClassCfg, Cfg::kUbxIdCfgValset};
    EXPECT_TRUE(IsAckFor(Message(Cfg::kUbxClassAck, Cfg::kUbxIdAckAck, valset), Cfg::kUbxClassCfg, Cfg::kUbxIdCfgValset));
    EXPECT_TRUE(IsAckFor(Message(Cfg::kUbxClassAck, Cfg::kUbxIdAckNak, valset), Cfg::kUbxClassCfg, Cfg::kUbxIdCfgValset));
    EXPECT_FALSE(IsAckFor(Message(Cfg::kUbxClassAck, Cfg::kUbxIdAckAck, valset), Cfg::kUbxClassCfg, Cfg::kUbxIdCfgValget));
    EXPECT_FALSE(IsAckFor(Message(Cfg::kUbxClassCfg, Cfg::kUbxIdAckAck, valset), Cfg::kUbxClassCfg, Cfg::kUbxIdCfgValset));
    EXPECT_FALSE(IsAckFor(Message(Cfg::kUbxClassAck, Cfg::kUbxIdAckAck, {Cfg::kUbxClassCfg}), Cfg::kUbxClassCfg,
                          Cfg::kUbxIdCfgValset));
}

TEST(M10Config, MapsAntennaModeToInterferenceSetting) {
    EXPECT_EQ(Cfg::AntennaSetting(GNSSSettingsOptions::kGNSSAntennaUnknown), 0);
    EXPECT_EQ(Cfg::AntennaSetting(GNSSSettingsOptions::kGNSSAntennaPassive), 1);
    EXPECT_EQ(Cfg::AntennaSetting(GNSSSettingsOptions::kGNSSAntennaActive), 2);
    EXPECT_EQ(Cfg::AntennaSetting(GNSSSettingsOptions::kGNSSAntennaActiveSupervised), 2);
}

TEST(M10Config, SelectsPositionSentencesAndStatusMessages) {
    using Selection = std::vector<std::pair<uint32_t, uint8_t>>;
    Selection quiet, verbose;
    WriteMessageSelection(false, [&](uint32_t key, uint8_t rate) { quiet.emplace_back(key, rate); });
    WriteMessageSelection(true, [&](uint32_t key, uint8_t rate) { verbose.emplace_back(key, rate); });
    EXPECT_EQ(quiet, (Selection{{0x209100bb, 1}, {0x209100ac, 1}, {0x209100c0, 0}, {0x209100c5, 0}, {0x209100ca, 0},
                                {0x209100b1, 0}, {0x2091035a, 5}, {0x2091006b, 5}, {0x20920002, 0}}));
    quiet.back().second = 3;
    EXPECT_EQ(verbose, quiet);
}

TEST(M10Config, ReadsValueSizeFromKey) {
    EXPECT_EQ(CfgValueSize(Cfg::kCfgSignalSbas), 1);         // 0x1...: one bit, stored in a byte.
    EXPECT_EQ(CfgValueSize(Cfg::kCfgMsgoutNmeaGgaUart1), 1);  // 0x2...: one byte.
    EXPECT_EQ(CfgValueSize(Cfg::kCfgRateMeas), 2);            // 0x3...: two bytes.
    EXPECT_EQ(CfgValueSize(Cfg::kCfgUart1Baudrate), 4);       // 0x4...: four bytes.
    EXPECT_EQ(CfgValueSize(0x50000001), 8);                   // 0x5...: eight bytes.
    EXPECT_EQ(CfgValueSize(0x00000001), 0);
    EXPECT_EQ(CfgValueSize(0x60000001), 0);
}

namespace {

using Batch = M10ConfigBatch;

// GGA rate 1, measurement period 1000 ms, GSA rate 0.
void WriteThree(const M10Options&, Batch& batch) {
    batch.Add(Cfg::kCfgMsgoutNmeaGgaUart1, 1);
    batch.Add(Cfg::kCfgRateMeas, 1000);
    batch.Add(Cfg::kCfgMsgoutNmeaGsaUart1, 0);
}

Batch::Readback Compare(const Bytes& reply, Batch::Window window = {0, 64, 256}) {
    Batch batch = Batch::Compare(window, reply.data(), reply.size());
    WriteThree({}, batch);
    return batch.readback();
}

const Bytes kThreeReply{1, 0, 0, 0, 0xbb, 0x00, 0x91, 0x20, 1, 0x01, 0x00, 0x21, 0x30, 0xE8, 0x03,
                        0xc0, 0x00, 0x91, 0x20, 0};

}  // namespace

TEST(M10ConfigBatch, BuildsRequestsForItsWindow) {
    uint8_t payload[64];
    Batch keys = Batch::Keys({0, 64, 64}, payload);
    WriteThree({}, keys);
    EXPECT_EQ(keys.count(), 3);
    EXPECT_EQ(keys.total(), 3);
    EXPECT_EQ(Bytes(payload, payload + keys.length()),
              (Bytes{0, 0, 0, 0, 0xbb, 0x00, 0x91, 0x20, 0x01, 0x00, 0x21, 0x30, 0xc0, 0x00, 0x91, 0x20}));

    Batch values = Batch::Values({0, 64, 64}, payload, Cfg::kCfgLayerRam | Cfg::kCfgLayerBbr);
    WriteThree({}, values);
    Bytes expected = kThreeReply;
    expected[0] = 0;  // Version 0.
    expected[1] = 3;  // RAM and battery-backed RAM.
    EXPECT_EQ(Bytes(payload, payload + values.length()), expected);
}

TEST(M10ConfigBatch, LimitsWindowByItemsAndPayload) {
    uint8_t payload[64];
    Batch second = Batch::Keys({1, 1, 64}, payload);
    WriteThree({}, second);
    EXPECT_EQ(second.count(), 1);
    EXPECT_EQ(second.total(), 3);
    EXPECT_EQ(second.first_key(), Cfg::kCfgRateMeas);
    EXPECT_EQ(Bytes(payload, payload + second.length()), (Bytes{0, 0, 0, 0, 0x01, 0x00, 0x21, 0x30}));

    // Header 4, GGA 5 and the period 6 make 15 bytes as a VALSET; GSA does not fit into 19.
    Batch tight = Batch::Values({0, 64, 19}, payload, Cfg::kCfgLayerRam);
    WriteThree({}, tight);
    EXPECT_EQ(tight.count(), 2);
    EXPECT_EQ(tight.length(), 15);

    Batch past_end = Batch::Count({3, 64, 64});
    WriteThree({}, past_end);
    EXPECT_EQ(past_end.count(), 0);
    EXPECT_EQ(past_end.total(), 3);
}

TEST(M10ConfigBatch, ComparesReplyValues) {
    EXPECT_EQ(Compare(kThreeReply), Batch::Readback::kMatch);
    Bytes other_rate = kThreeReply;
    other_rate[8] = 0;
    EXPECT_EQ(Compare(other_rate), Batch::Readback::kMismatch);
    Bytes other_period = kThreeReply;
    other_period[14] = 0x04;
    EXPECT_EQ(Compare(other_period), Batch::Readback::kMismatch);
}

TEST(M10ConfigBatch, IgnoresRepliesToOtherRequests) {
    Bytes other_key = kThreeReply;
    other_key[4] = 0xac;  // RMC is not in the batch.
    EXPECT_EQ(Compare(other_key), Batch::Readback::kUnrelated);
    Bytes other_layer = kThreeReply;
    other_layer[1] = Cfg::kCfgGetLayerBbr;
    EXPECT_EQ(Compare(other_layer), Batch::Readback::kUnrelated);
    Bytes request = kThreeReply;
    request[0] = 0;
    EXPECT_EQ(Compare(request), Batch::Readback::kUnrelated);
    const Bytes cut(kThreeReply.begin(), kThreeReply.end() - 1);
    EXPECT_EQ(Compare(cut), Batch::Readback::kUnrelated);
    // A reply holds exactly the keys of its request.
    EXPECT_EQ(Compare(kThreeReply, {0, 1, 256}), Batch::Readback::kUnrelated);
    const Bytes without_gsa(kThreeReply.begin(), kThreeReply.begin() + 15);
    EXPECT_EQ(Compare(without_gsa), Batch::Readback::kUnrelated);
}

TEST(AirgrepM10Profile, HoldsMessageSelectionThenReceiverSettings) {
    constexpr uint16_t kMessages = sizeof(airgrep::kGNSSMessages) / sizeof(airgrep::kGNSSMessages[0]);
    Batch passive = Batch::Count({0, 64, 256});
    airgrep::kM10Profile.write({GNSSSettingsOptions::kGNSSAntennaPassive, false}, passive);
    EXPECT_EQ(passive.total(), kMessages + 8);
    EXPECT_EQ(passive.first_key(), airgrep::kGNSSMessages[0].key);

    Batch supervised = Batch::Count({0, 64, 256});
    airgrep::kM10Profile.write({GNSSSettingsOptions::kGNSSAntennaActiveSupervised, false}, supervised);
    EXPECT_EQ(supervised.total(), kMessages + 10);  // Adds the two supervisor pins.

    uint8_t payload[16];
    Batch settings = Batch::Values({kMessages, 1, sizeof(payload)}, payload, Cfg::kCfgLayerRam);
    airgrep::kM10Profile.write({}, settings);
    EXPECT_EQ(Bytes(payload, payload + settings.length()), (Bytes{0, 1, 0, 0, 0x01, 0x00, 0x21, 0x30, 0xE8, 0x03}));
}

TEST(AirgrepM10Profile, DiagnosticsEnableInfoMessages) {
    constexpr uint16_t kMessages = sizeof(airgrep::kGNSSMessages) / sizeof(airgrep::kGNSSMessages[0]);
    ASSERT_EQ(airgrep::kGNSSMessages[kMessages - 2].key, Cfg::kCfgInfmsgUbxUart1);
    for (bool diagnostics : {false, true}) {
        uint8_t payload[16];
        Batch info = Batch::Values({static_cast<uint16_t>(kMessages - 2), 1, sizeof(payload)}, payload, Cfg::kCfgLayerRam);
        airgrep::kM10Profile.write({GNSSSettingsOptions::kGNSSAntennaUnknown, diagnostics}, info);
        EXPECT_EQ(Bytes(payload, payload + info.length()),
                  (Bytes{0, 1, 0, 0, 0x02, 0x00, 0x92, 0x20, static_cast<uint8_t>(diagnostics ? 3 : 0)}));
    }
}
