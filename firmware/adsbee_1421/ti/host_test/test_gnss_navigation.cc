#include "comms/gnss_navigation.hh"
#include "gtest/gtest.h"

namespace {
constexpr char kGga[] = "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47\r\n";
constexpr char kRmc[] = "$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A\r\n";
constexpr uint64_t kEpochMs = 764426119000;

void Feed(GNSSNavigation& nav, const char* sentence, uint64_t now, uint64_t edge) {
    while (*sentence) nav.Ingest(*sentence++, now, now - 5000, edge);
}
}  // namespace

TEST(GNSSNavigation, ReportsFreshPositionAndPPSTimeIndependently) {
    GNSSNavigation nav;
    uint64_t utc = 0;
    Feed(nav, kGga, 1100000, 1000000);
    EXPECT_EQ(nullptr, nav.GetFix(1100000));
    Feed(nav, kRmc, 1200000, 1000000);
    ASSERT_NE(nullptr, nav.GetFix(1200000));
    EXPECT_NEAR(48.1173, nav.GetFix(1200000)->latitude_deg, 0.0001);
    EXPECT_EQ(22, nav.GetFix(1200000)->speed_kts);
    ASSERT_TRUE(nav.GetUTCUnixMs(1250000, utc));
    EXPECT_EQ(kEpochMs + 250, utc);
    EXPECT_FALSE(nav.GetUTCUnixMs(4000000, utc));
    EXPECT_EQ(nullptr, nav.GetFix(4100000));

    Feed(nav, kRmc, 4200000, 1000000);
    EXPECT_EQ(nullptr, nav.GetFix(4200000));  // Altitude expired.
    Feed(nav, kGga, 4250000, 1000000);
    EXPECT_NE(nullptr, nav.GetFix(4250000));
    EXPECT_FALSE(nav.GetUTCUnixMs(4250000, utc));
}

TEST(GNSSNavigation, RejectsLateOrStraddlingPulseLabels) {
    GNSSNavigation nav;
    uint64_t utc = 0;
    nav.Ingest('$', 1003000, 998000, 1000000);
    Feed(nav, kRmc + 1, 1020000, 1000000);
    EXPECT_FALSE(nav.GetUTCUnixMs(1030000, utc));
    Feed(nav, kRmc, 1600000, 1000000);
    EXPECT_FALSE(nav.GetUTCUnixMs(1610000, utc));
    Feed(nav, kRmc, 2100000, 2000000);
    ASSERT_TRUE(nav.GetUTCUnixMs(2110000, utc));
    EXPECT_FALSE(nav.GetUTCUnixMs((1ULL << 32) + 2110000, utc));
}

TEST(GNSSNavigation, FixQueryFallsBackToDatedRmcUntilItExpires) {
    GNSSNavigation nav;
    uint64_t utc = 0;
    Feed(nav, kRmc, 1200000, 0);
    ASSERT_TRUE(nav.GetFixUTCUnixMs(1300000, utc));
    EXPECT_EQ(kEpochMs + 100, utc);
    EXPECT_FALSE(nav.GetUTCUnixMs(1300000, utc));
    EXPECT_FALSE(nav.GetFixUTCUnixMs(6300000, utc));
    constexpr uint64_t wrap_us = (1ULL << 32) * 1000;
    EXPECT_FALSE(nav.GetFixUTCUnixMs(wrap_us + 1300000, utc));
    EXPECT_FALSE(nav.fix().IsFresh((wrap_us + 1300000) / 1000));
    Feed(nav, kRmc, 7200000, 7000000);
    ASSERT_TRUE(nav.GetFixUTCUnixMs(7300000, utc));
    EXPECT_EQ(kEpochMs + 300, utc);
    Feed(nav, kRmc, wrap_us + 7200000, wrap_us + 7000000);
    ASSERT_TRUE(nav.GetFixUTCUnixMs(wrap_us + 7300000, utc));
    EXPECT_EQ(kEpochMs + 300, utc);
}
