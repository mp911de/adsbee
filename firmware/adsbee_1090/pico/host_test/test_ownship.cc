#include "comms/ownship.hh"

#include <cmath>

#include "gtest/gtest.h"

TEST(Ownship, DefaultsReportNothing) {
    Ownship ownship;
    Ownship::Snapshot s = ownship.Get(0);
    EXPECT_EQ(s.address, 0u);
    EXPECT_STREQ(s.callsign, "ADSBEE");
    EXPECT_EQ(s.emitter_category, 0);
    EXPECT_FALSE(s.baro_valid);
    EXPECT_FALSE(s.vertical_rate_valid);
    EXPECT_FALSE(s.ahrs_valid);
}

TEST(Ownship, SetIdentity) {
    Ownship ownship;
    EXPECT_TRUE(ownship.SetIdentity(0xABC123, "deabc", 1));
    Ownship::Snapshot s = ownship.Get(0);
    EXPECT_EQ(s.address, 0xABC123u);
    EXPECT_STREQ(s.callsign, "DEABC");
    EXPECT_EQ(s.emitter_category, 1);

    EXPECT_TRUE(ownship.SetIdentity(0, "", 0));  // Empty callsign restores the default.
    EXPECT_STREQ(ownship.Get(0).callsign, "ADSBEE");

    // Rejected values leave the identity unchanged.
    ownship.SetIdentity(0x123456, "N123", 9);
    EXPECT_FALSE(ownship.SetIdentity(0x1000000, "N123", 9));
    EXPECT_FALSE(ownship.SetIdentity(0x123456, "N1234567X", 9));  // Longer than 8 characters.
    EXPECT_FALSE(ownship.SetIdentity(0x123456, "N12 3", 9));
    EXPECT_FALSE(ownship.SetIdentity(0x123456, "D-EABC", 9));
    EXPECT_FALSE(ownship.SetIdentity(0x123456, "N123", 40));
    EXPECT_EQ(ownship.Get(0).address, 0x123456u);
    EXPECT_STREQ(ownship.Get(0).callsign, "N123");
    EXPECT_EQ(ownship.Get(0).emitter_category, 9);
}

TEST(Ownship, BaroExpires) {
    Ownship ownship;
    ownship.SetBaro(4500, true, -300, 1000);
    Ownship::Snapshot s = ownship.Get(5999);
    EXPECT_TRUE(s.baro_valid);
    EXPECT_EQ(s.baro_altitude_ft, 4500);
    EXPECT_TRUE(s.vertical_rate_valid);
    EXPECT_EQ(s.vertical_rate_fpm, -300);
    s = ownship.Get(6000);
    EXPECT_FALSE(s.baro_valid);
    EXPECT_FALSE(s.vertical_rate_valid);

    ownship.SetBaro(4600, false, 0, 7000);
    EXPECT_TRUE(ownship.Get(7000).baro_valid);
    EXPECT_FALSE(ownship.Get(7000).vertical_rate_valid);
}

TEST(Ownship, AHRSFieldsAreOptionalAndExpire) {
    Ownship ownship;
    ASSERT_TRUE(ownship.SetAHRS(2.5f, -1.0f, NAN, false, -1, -1, 100));
    Ownship::Snapshot s = ownship.Get(5099);
    EXPECT_TRUE(s.ahrs_valid);
    EXPECT_FLOAT_EQ(s.roll_deg, 2.5f);
    EXPECT_FLOAT_EQ(s.pitch_deg, -1.0f);
    EXPECT_TRUE(std::isnan(s.heading_deg));
    EXPECT_EQ(s.indicated_airspeed_kts, -1);
    EXPECT_EQ(s.true_airspeed_kts, -1);
    EXPECT_FALSE(ownship.Get(5100).ahrs_valid);

    ASSERT_TRUE(ownship.SetAHRS(NAN, NAN, 271.0f, true, 95, 102, 200));
    s = ownship.Get(200);
    EXPECT_FLOAT_EQ(s.heading_deg, 271.0f);
    EXPECT_TRUE(s.heading_is_magnetic);
    EXPECT_EQ(s.indicated_airspeed_kts, 95);
    EXPECT_EQ(s.true_airspeed_kts, 102);
}

TEST(Ownship, AHRSRejectsOutOfRange) {
    Ownship ownship;
    EXPECT_FALSE(ownship.SetAHRS(180.1f, 0, NAN, false, -1, -1, 0));
    EXPECT_FALSE(ownship.SetAHRS(0, -180.1f, NAN, false, -1, -1, 0));
    EXPECT_FALSE(ownship.SetAHRS(0, 0, 360.0f, false, -1, -1, 0));
    EXPECT_FALSE(ownship.SetAHRS(0, 0, -0.1f, false, -1, -1, 0));
    EXPECT_FALSE(ownship.SetAHRS(0, 0, NAN, false, 0xFFFF, -1, 0));  // 0xFFFF encodes invalid.
    EXPECT_FALSE(ownship.SetAHRS(NAN, NAN, NAN, false, -1, -1, 0));  // Nothing known.
    EXPECT_FALSE(ownship.Get(0).ahrs_valid);
}

TEST(Ownship, DataAge) {
    Ownship ownship;
    ownship.SetBaro(0, false, 0, 1000);
    EXPECT_EQ(ownship.BaroAgeMs(1250), 250u);
    ASSERT_TRUE(ownship.SetAHRS(0, 0, NAN, false, -1, -1, 2000));
    EXPECT_EQ(ownship.AHRSAgeMs(2100), 100u);
}
