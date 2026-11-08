#include "airgrep_gnss_frame.hh"

#include <string>

#include "buffer_utils.hh"
#include "gtest/gtest.h"

namespace {

std::string Frame(const AirgrepGnssFrame& report) {
    char frame[AirgrepGnssFrame::kFrameMaxLen + 1];
    size_t length = report.Format(frame, sizeof(frame));
    EXPECT_GT(length, 0u);
    EXPECT_LE(length, AirgrepGnssFrame::kFrameMaxLen);
    EXPECT_EQ(frame[length], '\0');
    return std::string(frame, length);
}

// Strips and verifies the CRC and CRLF, returns the comma separated payload.
std::string Payload(const std::string& frame) {
    EXPECT_EQ(frame.substr(0, 6), "#GNSS:");
    EXPECT_EQ(frame.substr(frame.size() - 2), "\r\n");
    size_t comma = frame.rfind(',');
    EXPECT_EQ(std::stoul(frame.substr(comma + 1), nullptr, 16),
              CalculateCRC16(reinterpret_cast<const uint8_t*>(frame.data()), comma + 1));
    return frame.substr(6, comma - 6);
}

}  // namespace

TEST(AirgrepGnssFrame, UnknownFieldsAreEmpty) {
    NMEAParser::GNSSFix fix;
    AirgrepGnssFrame report{.fix = fix};
    EXPECT_EQ(Frame(report), "#GNSS:0,0,0," + std::string(31, ',') + "2695\r\n");
    EXPECT_EQ(Payload(Frame(report)), "0,0,0," + std::string(30, ','));
}

TEST(AirgrepGnssFrame, FullFixKeepsFieldOrder) {
    NMEAParser::GNSSFix fix;
    fix.valid = true;
    fix.latitude_deg = 50.5f;
    fix.longitude_deg = -8.25f;
    fix.altitude_ft = 1234;
    fix.heading_deg = 271.5f;
    fix.speed_kts = 87;
    fix.num_satellites = 11;
    fix.fix_quality = 2;
    fix.hdop = 0.9f;
    fix.utc_time_valid = true;
    fix.utc_hour = 7;
    fix.utc_minute = 5;
    fix.utc_second = 9;
    M10Diagnostics::RfStatus rf;
    rf.antenna_status = 2;
    rf.antenna_power = 1;
    rf.jamming = 1;
    M10Diagnostics::SatelliteSummary sky;
    sky.max_cno_dbhz = 47;
    sky.count = 24;
    sky.no_signal = 5;
    sky.searching = 4;
    sky.acquired = 3;
    sky.unusable = 2;
    sky.locked = 10;
    sky.ephemeris = 12;
    sky.used = 11;
    sky.healthy = 13;
    sky.unhealthy = 1;
    M10Diagnostics::SbasStatus sbas;
    sbas.mode = 1;    // ENABLED
    sbas.system = 1;  // EGNOS
    sbas.count = 2;
    sbas.satellites[1].service = M10Diagnostics::kSbasServiceCorrections;
    sbas.satellites[1].range_correction_cm = -312;
    sbas.satellites[1].ionosphere_correction_cm = 45;
    AirgrepGnssFrame report{.fix = fix,
                            .fix_valid = true,
                            .epoch_valid = true,
                            .epoch_ms = 1762588800123ull,
                            .rf = &rf,
                            .satellites = &sky,
                            .sbas = &sbas,
                            .module = "MAX-M10S",
                            .software = "ROM SPG 5.10 (7b202e)",
                            .hardware = "00000A00",
                            .bbr_ok = true};
    EXPECT_EQ(Payload(Frame(report)),
              "1,2,11,50.500000,-8.250000,1234,87,271.5,0.90,070509,1762588800123,2,1,1,"
              "47,24,5,4,3,2,10,12,11,13,1,ENABLED,EGNOS,2,-312,45,MAX-M10S,ROM SPG 5.10 (7b202e),00000A00,OK");
}

TEST(AirgrepGnssFrame, SbasWithoutCorrectionsAndCommasInTextStaySingleFields) {
    NMEAParser::GNSSFix fix;
    M10Diagnostics::SbasStatus sbas;
    sbas.mode = 0;  // DISABLED
    sbas.system = -1;
    AirgrepGnssFrame report{.fix = fix, .sbas = &sbas, .module = "A,B", .software = "C,,D"};
    EXPECT_EQ(Payload(Frame(report)), "0,0,0," + std::string(22, ',') + "DISABLED,UNKNOWN,0,,,A B,C  D,,");
}

TEST(AirgrepTimePing, EchoesTokenWithEpochAndPpsCount) {
    char reply[AirgrepTimePing::kReplyMaxLen + 1];
    AirgrepTimePing ping{.echo = "1762588800123", .epoch_valid = true, .epoch_ms = 1762588800456ull, .pps_count = 42};
    size_t length = ping.Format(reply, sizeof(reply));
    EXPECT_EQ(std::string(reply, length), "TIME_PING=1762588800123,1762588800456,42\r\n");
}

TEST(AirgrepTimePing, EpochIsEmptyWhenUnavailable) {
    char reply[AirgrepTimePing::kReplyMaxLen + 1];
    AirgrepTimePing ping{.echo = "7"};
    size_t length = ping.Format(reply, sizeof(reply));
    EXPECT_EQ(std::string(reply, length), "TIME_PING=7,,0\r\n");
}

TEST(AirgrepTimePing, AcceptsTwentyDigitsAndRejectsOtherTokens) {
    char reply[AirgrepTimePing::kReplyMaxLen + 1];
    std::string twenty(20, '9');
    AirgrepTimePing ping{.echo = twenty, .epoch_valid = true, .epoch_ms = UINT64_MAX, .pps_count = UINT32_MAX};
    size_t length = ping.Format(reply, sizeof(reply));
    EXPECT_EQ(length, AirgrepTimePing::kReplyMaxLen);
    EXPECT_EQ(std::string(reply, length), "TIME_PING=" + twenty + ",18446744073709551615,4294967295\r\n");
    EXPECT_EQ(AirgrepTimePing{.echo = twenty + "0"}.Format(reply, sizeof(reply)), 0u);
    EXPECT_EQ(AirgrepTimePing{.echo = ""}.Format(reply, sizeof(reply)), 0u);
    EXPECT_EQ(AirgrepTimePing{.echo = "12,3"}.Format(reply, sizeof(reply)), 0u);
    EXPECT_EQ(AirgrepTimePing{.echo = "-5"}.Format(reply, sizeof(reply)), 0u);
}

TEST(AirgrepGnssFrame, StaleFixDropsPositionButKeepsTime) {
    NMEAParser::GNSSFix fix;
    fix.valid = true;
    fix.latitude_deg = 1.0f;
    fix.num_satellites = 3;
    fix.fix_quality = 1;
    fix.hdop = 12.0f;
    fix.utc_time_valid = true;
    fix.utc_hour = 23;
    fix.utc_minute = 59;
    fix.utc_second = 58;
    AirgrepGnssFrame report{.fix = fix, .fix_valid = false};
    EXPECT_EQ(Payload(Frame(report)), "0,1,3,,,,,,12.00,235958" + std::string(24, ','));
}

TEST(AirgrepGnssFrame, RejectsSmallBuffer) {
    NMEAParser::GNSSFix fix;
    AirgrepGnssFrame report{.fix = fix};
    char frame[AirgrepGnssFrame::kFrameMaxLen];
    EXPECT_EQ(report.Format(frame, sizeof(frame)), 0u);
}
