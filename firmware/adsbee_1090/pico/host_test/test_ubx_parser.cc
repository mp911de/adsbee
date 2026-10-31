#include "ubx_parser.hh"

#include <string>
#include <vector>

#include "gtest/gtest.h"
#include "ubx_test_utils.hh"

namespace {

struct Capture : public UbxParser::Listener {
    struct Seen {
        uint8_t cls, id;
        uint16_t length, stored;
        std::string payload;
    };
    std::vector<Seen> messages;
    void OnUbxMessage(const UbxMessage& m) override {
        messages.push_back({m.cls, m.id, m.length, m.stored,
                            std::string(reinterpret_cast<const char*>(m.payload), m.stored)});
    }
};

}  // namespace

TEST(UbxParser, DeliversFrameAndIgnoresInterleavedNmea) {
    Capture capture;
    UbxParser parser(capture);
    Feed(parser, std::vector<uint8_t>{'$', 'G', 'N', 'R', 'M', 'C', '\r', '\n'});
    Feed(parser, UBX(4, 0, "error"));
    Feed(parser, std::vector<uint8_t>{'$', 'x'});
    Feed(parser, UBX(0x0A, 0x38, ""));
    ASSERT_EQ(capture.messages.size(), 2u);
    EXPECT_EQ(capture.messages[0].cls, 4);
    EXPECT_EQ(capture.messages[0].payload, "error");
    EXPECT_EQ(capture.messages[1].id, 0x38);
    EXPECT_EQ(capture.messages[1].length, 0);
}

TEST(UbxParser, RejectsEitherCorruptChecksumByteAndRecovers) {
    Capture capture;
    UbxParser parser(capture);
    for (int checksum : {0, 1}) {
        auto frame = UBX(4, 0, "bad");
        frame[frame.size() - 1 - checksum] ^= 1;
        Feed(parser, frame);
    }
    EXPECT_TRUE(capture.messages.empty());
    Feed(parser, UBX(4, 0, "good"));
    ASSERT_EQ(capture.messages.size(), 1u);
    EXPECT_EQ(capture.messages[0].payload, "good");
}

TEST(UbxParser, RepeatedSyncByteStartsFrame) {
    Capture capture;
    UbxParser parser(capture);
    auto frame = UBX(4, 0, "x");
    frame.insert(frame.begin(), 0xB5);
    Feed(parser, frame);
    ASSERT_EQ(capture.messages.size(), 1u);
}

TEST(UbxParser, StoresPrefixOfLongPayloadAfterChecksumValidation) {
    Capture capture;
    UbxParser parser(capture);
    auto frame = UBX(4, 0, std::string(700, 'x'));
    Feed(parser, frame);
    ASSERT_EQ(capture.messages.size(), 1u);
    EXPECT_EQ(capture.messages[0].length, 700);
    EXPECT_EQ(capture.messages[0].stored, UbxParser::kPayloadMaxLen);
    frame.back() ^= 1;
    Feed(parser, frame);
    EXPECT_EQ(capture.messages.size(), 1u);
}

TEST(UbxParser, ImplausibleLengthResyncsImmediately) {
    Capture capture;
    UbxParser parser(capture);
    std::vector<uint8_t> corrupt{0xB5, 0x62, 4, 0, 0xFF, 0xFF};
    Feed(parser, corrupt);
    EXPECT_FALSE(parser.InFrame());
    Feed(parser, UBX(4, 0, "recovered"));
    ASSERT_EQ(capture.messages.size(), 1u);
}

TEST(UbxParser, PartialFrameTimesOutAndResynchronizes) {
    Capture capture;
    UbxParser parser(capture);
    auto broken = UBX(4, 0, "incomplete");
    broken.resize(8);
    Feed(parser, broken);
    EXPECT_TRUE(parser.InFrame());
    Feed(parser, UBX(4, 0, "recovered"), 1002);
    ASSERT_EQ(capture.messages.size(), 1u);
    EXPECT_EQ(capture.messages[0].payload, "recovered");
}

TEST(UbxParser, ResyncDropsPartialFrame) {
    Capture capture;
    UbxParser parser(capture);
    auto broken = UBX(4, 0, "incomplete");
    broken.resize(8);
    Feed(parser, broken);
    parser.Resync();
    EXPECT_FALSE(parser.InFrame());
    Feed(parser, UBX(4, 0, "next"));
    ASSERT_EQ(capture.messages.size(), 1u);
    EXPECT_EQ(capture.messages[0].payload, "next");
}
