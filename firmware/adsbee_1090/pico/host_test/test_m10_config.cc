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

TEST(AirgrepM10Profile, LoadsBatchesAndEnablesInfoMessagesForDiagnostics) {
    airgrep::GNSSMessageRate batch[32];
    EXPECT_EQ(airgrep::LoadMessages(0, airgrep::kBootstrapMessageCount, true, batch), airgrep::kBootstrapMessageCount);
    EXPECT_EQ(batch[0].key, airgrep::kGNSSMessages[0].key);
    EXPECT_EQ(batch[7].rate, airgrep::kGNSSMessages[7].rate);

    const uint16_t tail = airgrep::kGNSSMessageCount - 2;  // INFMSG-UBX_UART1, INFMSG-NMEA_UART1
    ASSERT_EQ(airgrep::LoadMessages(tail, 32, false, batch), 2);
    EXPECT_EQ(batch[0].key, Cfg::kCfgInfmsgUbxUart1);
    EXPECT_EQ(batch[0].rate, 0);
    ASSERT_EQ(airgrep::LoadMessages(tail, 32, true, batch), 2);
    EXPECT_EQ(batch[0].rate, 3);
    EXPECT_EQ(batch[1].rate, 0);
    EXPECT_EQ(airgrep::LoadMessages(airgrep::kGNSSMessageCount, 32, true, batch), 0);
}

TEST(AirgrepM10Profile, WritesRatesAndReadbackKeys) {
    constexpr airgrep::GNSSMessageRate rates[] = {{0x209100bb, 1}, {0x209100c0, 0}};  // GGA, GSA
    uint8_t payload[4 + 2 * 5];
    uint16_t length = airgrep::WriteMessageRates(payload, Cfg::kCfgLayerRam | Cfg::kCfgLayerBbr, rates, 2);
    EXPECT_EQ(Bytes(payload, payload + length),
              (Bytes{0, 3, 0, 0, 0xbb, 0x00, 0x91, 0x20, 1, 0xc0, 0x00, 0x91, 0x20, 0}));
    length = airgrep::WriteMessageKeys(payload, rates, 2);
    EXPECT_EQ(Bytes(payload, payload + length), (Bytes{0, 0, 0, 0, 0xbb, 0x00, 0x91, 0x20, 0xc0, 0x00, 0x91, 0x20}));
}
