#include "m10_telemetry.hh"

#include <string>
#include <vector>

#include "buffer_utils.hh"
#include "gtest/gtest.h"
#include "ubx_test_utils.hh"

namespace {

// Parser -> decoder -> telemetry, as wired in UbloxMAXM10.
struct Rig : public UbxParser::Listener {
    M10Telemetry telemetry;
    UbxParser parser{*this};
    void OnUbxMessage(const UbxMessage& message) override { M10Diagnostics::Decode(message, telemetry); }
    void Feed(const std::vector<uint8_t>& frame, uint32_t now = 1) { ::Feed(parser, frame, now); }
    void SetEnabled(bool enabled) { telemetry.SetEnabled(enabled); }
};

std::string Next(Rig& rig) {
    char frame[M10Telemetry::kFrameMaxLen + 1];
    size_t length = rig.telemetry.NextFrame(frame, sizeof(frame));
    EXPECT_LE(length, 190u);
    if (length) {
        EXPECT_EQ(frame[length], '\0');
    }
    return std::string(frame, length);
}

std::string Payload(const std::string& frame) {
    EXPECT_EQ(frame.substr(frame.size() - 2), "\r\n");
    size_t start = 0;
    for (int i = 0; i < 5; ++i) start = frame.find(':', start) + 1;
    size_t star = frame.rfind('*');
    auto payload = frame.substr(start, star - start);
    EXPECT_EQ(std::stoul(frame.substr(5)), payload.size());
    EXPECT_EQ(std::stoul(frame.substr(star + 1), nullptr, 16),
              CalculateCRC16(reinterpret_cast<const uint8_t*>(frame.data()), star));
    return payload;
}

}  // namespace

TEST(M10Telemetry, DisabledAndUnknownMessagesAreSilent) {
    Rig d;
    d.Feed(UBX(4, 0, "error"));
    EXPECT_TRUE(Next(d).empty());
    d.SetEnabled(true);
    Feed(d.parser, UBX(4, 2, "notice"));
    EXPECT_TRUE(Next(d).empty());
}

TEST(M10Telemetry, LatestPerSeverityAndRepeatedOccurrences) {
    Rig d;
    d.SetEnabled(true);
    d.Feed(UBX(4, 1, "old warning"));
    d.Feed(UBX(4, 0, "error"));
    d.Feed(UBX(4, 1, "warning"));
    EXPECT_EQ(Payload(Next(d)), "error");
    EXPECT_EQ(Payload(Next(d)), "warning");
    EXPECT_TRUE(Next(d).empty());
    d.Feed(UBX(4, 1, "warning"));
    EXPECT_EQ(Payload(Next(d)), "warning");
}

TEST(M10Telemetry, EscapesBeforeSplittingAndFreezesActiveMessage) {
    Rig d;
    d.SetEnabled(true);
    std::string text(53, '%');
    text += "\r\n*\xC3\xA4";
    d.Feed(UBX(4, 1, text));
    std::string first = Next(d);
    EXPECT_NE(first.find(":WARN:00:01/02:"), std::string::npos);
    std::string escaped = Payload(first);
    EXPECT_EQ(escaped.size(), 160u);
    EXPECT_EQ(escaped.back(), '%');
    d.Feed(UBX(4, 1, "replacement"));
    std::string second = Next(d);
    EXPECT_NE(second.find(":WARN:00:02/02:"), std::string::npos);
    escaped += Payload(second);
    std::string expected;
    for (int i = 0; i < 53; ++i) expected += "%25";
    expected += "%0D%0A%2A\xC3\xA4";
    EXPECT_EQ(escaped, expected);
    EXPECT_EQ(Payload(Next(d)), "replacement");

    d.Feed(UBX(4, 1, std::string(159, 'x') + "\xC3\xA4"));
    EXPECT_EQ(Payload(Next(d)), std::string(159, 'x') + "\xC3");
    EXPECT_EQ(Payload(Next(d)), "\xA4");
}

TEST(M10Telemetry, OversizedTextIsTruncatedWithMarker) {
    Rig d;
    d.SetEnabled(true);
    d.Feed(UBX(4, 0, std::string(300, 'x')));
    std::string text;
    for (std::string f = Next(d); !f.empty(); f = Next(d)) text += Payload(f);
    EXPECT_EQ(text, std::string(245, 'x') + "[truncated]");
    // Longer than the parser keeps: still marked, never over-read.
    d.Feed(UBX(4, 0, std::string(1000, 'y')));
    text.clear();
    for (std::string f = Next(d); !f.empty(); f = Next(d)) text += Payload(f);
    EXPECT_EQ(text, std::string(245, 'y') + "[truncated]");
}

TEST(M10Telemetry, WorstCaseEscapingNeedsFiveBoundedFragments) {
    Rig d;
    d.SetEnabled(true);
    d.Feed(UBX(4, 1, std::string(256, '\n')));
    std::string text;
    int count = 0;
    for (std::string f = Next(d); !f.empty(); f = Next(d)) {
        EXPECT_NE(f.find("/05:"), std::string::npos);
        text += Payload(f);
        ++count;
    }
    EXPECT_EQ(count, 5);
    EXPECT_EQ(text.size(), 768u);
    EXPECT_LE(sizeof(M10Telemetry), 1024u);
}

TEST(M10Telemetry, MessageIDsWrapAt63) {
    Rig d;
    d.SetEnabled(true);
    for (int i = 0; i < 65; ++i) {
        d.Feed(UBX(4, 0, "x"));
        char header[40];
        snprintf(header, sizeof(header), "#M10:1:ERR:%02u:01/01:", i & 63);
        EXPECT_EQ(Next(d).find(header), 0u);
    }
}

TEST(M10Telemetry, ResetAndDisableDiscardPendingAndActiveMessages) {
    Rig d;
    d.SetEnabled(true);
    d.Feed(UBX(4, 0, std::string(256, 'x')));
    ASSERT_FALSE(Next(d).empty());
    d.Feed(UBX(4, 1, "pending"));
    d.telemetry.Reset();
    EXPECT_TRUE(Next(d).empty());
    d.Feed(UBX(4, 0, "pending"));
    d.SetEnabled(false);
    d.SetEnabled(true);
    EXPECT_TRUE(Next(d).empty());
}

TEST(M10Telemetry, FailedDeliveryDropsActiveWithoutDroppingNewerPendingEvent) {
    Rig d;
    d.SetEnabled(true);
    d.Feed(UBX(4, 0, std::string(256, 'x')));
    char short_buffer[190];
    EXPECT_EQ(d.telemetry.NextFrame(short_buffer, sizeof(short_buffer)), 0u);
    EXPECT_EQ(d.telemetry.NextFrame(nullptr, 191), 0u);
    EXPECT_NE(Next(d).find(":ERR:00:01/02:"), std::string::npos);
    d.Feed(UBX(4, 0, "new occurrence"));
    d.telemetry.DropActive();
    EXPECT_EQ(Payload(Next(d)), "new occurrence");
    EXPECT_TRUE(Next(d).empty());
}

TEST(M10Telemetry, MonFieldOrderSignedValuesAndKnownCRC) {
    Rig d;
    d.SetEnabled(true);
    std::string payload(28, '\0');
    payload[1] = 1;
    payload[5] = 1;
    payload[6] = 2;
    payload[7] = 1;
    payload[16] = 50;
    payload[18] = 0xA0;
    payload[19] = 0x0F;
    payload[22] = 100;
    payload[24] = 100;
    d.Feed(UBX(0x0A, 0x38, payload));
    EXPECT_EQ(Next(d), "#M10:33:MON:00:01/01:0,0,1,2,1,0,50,4000,0,0,100,0,100*BF08\r\n");
    payload[21] = static_cast<char>(-128);
    payload[23] = static_cast<char>(-1);
    d.Feed(UBX(0x0A, 0x38, payload));
    EXPECT_EQ(Payload(Next(d)), "0,0,1,2,1,0,50,4000,0,-128,100,-1,100");
    d.SetEnabled(false);
    d.Feed(UBX(0x0A, 0x38, payload));
    EXPECT_TRUE(Next(d).empty());
}
