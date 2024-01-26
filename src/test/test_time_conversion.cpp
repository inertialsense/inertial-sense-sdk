#include <gtest/gtest.h>
#include "../time_conversion.h"
#include "../protocol_nmea.h"

TEST(time_conversion, GPS_to_UTC)
{
    uint32_t gpsWeek = 2294; 
    uint32_t gpsTowMs = 421338800;
    uint32_t gpsLeapS = 18;
    uint32_t utc_hours, utc_minutes, utc_seconds, utc_ms;
    gpsTowMsToUtcTime(gpsTowMs, gpsLeapS, &utc_hours, &utc_minutes, &utc_seconds, &utc_ms);

    ASSERT_EQ(utc_hours, 21);
    ASSERT_EQ(utc_minutes, 02);
    ASSERT_EQ(utc_seconds, 0);
    ASSERT_EQ(utc_ms, 800);
}

#if 0
TEST(time_conversion, GPS_to_julian)
{
    uint32_t gpsWeek = 2294; 
    uint32_t gpsTowMs = 425870; 
    uint32_t leapSeconds = 18;

	double julian = gpsToJulian(gpsWeek, gpsTowMs, leapSeconds);

    ASSERT_EQ(julian, 2460307.429053);
}

TEST(time_conversion, GPS_to_UTC)
{
    unsigned short     gps_week;
    double             gps_towMs;
    double             gps_tow;

    unsigned short    utc_year;
    unsigned char     utc_month;
    unsigned char     utc_day;
    unsigned char     utc_hour;
    unsigned char     utc_minute;
    float             utc_seconds;

    gps_week = 2294;
    gps_towMs = 421338800;
    gps_tow = gps_towMs * 0.001;

    int gps_time = 2294 * 604800 + gps_tow;   // seconds since Jan 6, 1980

    TIMECONV_UTCTimeFromGPSTime(
        gps_week,
        gps_tow,
        &utc_year,
        &utc_month,
        &utc_day,
        &utc_hour,
        &utc_minute,
        &utc_seconds);

    TIMECONV_UTCTimeFromGPSTime2(
        gps_week,
        gps_towMs,
        &utc_year,
        &utc_month,
        &utc_day,
        &utc_hour,
        &utc_minute,
        &utc_seconds);

    ASSERT_EQ(gps_time, 1387832538);
    ASSERT_EQ(utc_year, 2023);
    ASSERT_EQ(utc_month, 12);
    ASSERT_EQ(utc_day, 28);
    ASSERT_EQ(utc_hour, 21);
    ASSERT_EQ(utc_minute, 02);
    ASSERT_EQ(utc_seconds, 0.8);


}
#endif
