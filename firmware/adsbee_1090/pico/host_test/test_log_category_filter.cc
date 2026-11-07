#include <string>

#include "../application/comms/log_category_filter.hh"
#include "gtest/gtest.h"

namespace {
using Prefixes = std::initializer_list<std::string_view>;

LogCategoryFilter::SetResult Set(LogCategoryFilter& filter, Prefixes prefixes) {
    return filter.Set(prefixes.begin(), prefixes.size());
}
}  // namespace

TEST(LogCategoryFilter, PassesEverythingByDefault) {
    LogCategoryFilter filter;
    EXPECT_EQ(filter.count(), 0);
    EXPECT_TRUE(filter.Passes("UbloxMAXM10::OnRf: noise=1"));
    EXPECT_TRUE(filter.Passes(""));
}

TEST(LogCategoryFilter, PassesLinesStartingWithAPrefix) {
    LogCategoryFilter filter;
    ASSERT_EQ(Set(filter, {"UbloxMAXM10", "ESP32SerialFlasher"}), LogCategoryFilter::kOk);
    EXPECT_TRUE(filter.Passes("UbloxMAXM10::OnRf: noise=1"));
    EXPECT_TRUE(filter.Passes("ESP32SerialFlasher::FlashESP32: done"));
    EXPECT_FALSE(filter.Passes("ESP32::Init: UbloxMAXM10"));
    EXPECT_FALSE(filter.Passes("Ublox"));
    EXPECT_FALSE(filter.Passes(""));
}

TEST(LogCategoryFilter, NarrowsToOneMethod) {
    LogCategoryFilter filter;
    ASSERT_EQ(Set(filter, {"UbloxMAXM10::OnRf"}), LogCategoryFilter::kOk);
    EXPECT_TRUE(filter.Passes("UbloxMAXM10::OnRf: noise=1"));
    EXPECT_FALSE(filter.Passes("UbloxMAXM10::OnSbas: mode=1"));
}

TEST(LogCategoryFilter, AsteriskRestoresEverything) {
    LogCategoryFilter filter;
    ASSERT_EQ(Set(filter, {"UbloxMAXM10"}), LogCategoryFilter::kOk);
    ASSERT_EQ(Set(filter, {"*"}), LogCategoryFilter::kOk);
    EXPECT_EQ(filter.count(), 0);
    EXPECT_TRUE(filter.Passes("ADSBee::Init: ok"));
}

TEST(LogCategoryFilter, ReportsPrefixesInOrder) {
    LogCategoryFilter filter;
    ASSERT_EQ(Set(filter, {"B", "A"}), LogCategoryFilter::kOk);
    ASSERT_EQ(filter.count(), 2);
    EXPECT_EQ(filter.prefix(0), "B");
    EXPECT_EQ(filter.prefix(1), "A");
}

TEST(LogCategoryFilter, AcceptsEightPrefixesAndRejectsNine) {
    LogCategoryFilter filter;
    EXPECT_EQ(Set(filter, {"1", "2", "3", "4", "5", "6", "7", "8"}), LogCategoryFilter::kOk);
    EXPECT_EQ(Set(filter, {"1", "2", "3", "4", "5", "6", "7", "8", "9"}), LogCategoryFilter::kTooMany);
}

TEST(LogCategoryFilter, RejectedSetKeepsPreviousPrefixes) {
    LogCategoryFilter filter;
    ASSERT_EQ(Set(filter, {"UbloxMAXM10"}), LogCategoryFilter::kOk);

    EXPECT_EQ(Set(filter, {}), LogCategoryFilter::kEmpty);
    EXPECT_EQ(Set(filter, {"ADSBee", ""}), LogCategoryFilter::kEmpty);
    EXPECT_EQ(Set(filter, {"ADSBee", "*"}), LogCategoryFilter::kAllNotAlone);
    std::string too_long(LogCategoryFilter::kPrefixMaxLen + 1, 'x');
    EXPECT_EQ(Set(filter, {"ADSBee", too_long}), LogCategoryFilter::kTooLong);

    ASSERT_EQ(filter.count(), 1);
    EXPECT_EQ(filter.prefix(0), "UbloxMAXM10");
    EXPECT_FALSE(filter.Passes("ADSBee::Init: ok"));
}

TEST(LogCategoryFilter, AcceptsPrefixOfMaximumLength) {
    LogCategoryFilter filter;
    std::string longest(LogCategoryFilter::kPrefixMaxLen, 'x');
    ASSERT_EQ(Set(filter, {longest}), LogCategoryFilter::kOk);
    EXPECT_EQ(filter.prefix(0), longest);
    EXPECT_TRUE(filter.Passes((longest + ": line").c_str()));
    EXPECT_FALSE(filter.Passes(longest.substr(1).c_str()));
}
