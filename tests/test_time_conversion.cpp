#include <gtest/gtest.h>
#include "ISUtilities.h"
#include "time_conversion.h"
#include "test_data_utils.h"
#include "protocol_nmea.h"


TEST(time_conversion, UTC_to_GPS_to_UTC_time)
{
    SetUtcTimeZone();

    int gpsTowMs = 111072800;
    int gpsWeek = 2309;
    int leapS = 18;

    // Cycle through entire range of time of week
    for (gpsTowMs = 0; gpsTowMs < C_MILLISECONDS_PER_WEEK; gpsTowMs += 200)
    {
        std::tm utcTime = GpsTimeToUtcDateTime(gpsTowMs/1000, gpsWeek);
        uint32_t msec = gpsTowMs%1000;
#if 0
        printf("tow: %d ms %d week   ", gpsTowMs, gpsWeek);
        PrintUtcTime(utcTime, msec);
#endif
        uint32_t gpsTowMs2, gpsWeek2;
        UtcDateTimeToGpsTime(utcTime, gpsTowMs2, gpsWeek2);
        gpsTowMs2 = 1000*gpsTowMs2 + msec;
        ASSERT_EQ(gpsTowMs, gpsTowMs2);
        ASSERT_EQ(gpsWeek,  gpsWeek2);

    	utc_time_t t;
        gpsTowMsToUtcTime(gpsTowMs, leapS, &t);
        ASSERT_EQ(utcTime.tm_hour, t.hour);
        ASSERT_EQ(utcTime.tm_min, t.minute);
        ASSERT_EQ(utcTime.tm_sec, t.second);
        ASSERT_EQ(msec, t.millisecond);

        uint32_t gpsTowMs3;
        uint32_t weekday = gpsTowMs / C_MILLISECONDS_PER_DAY;
        utcTimeToGpsTowMs(&t, weekday, &gpsTowMs3, leapS);
        ASSERT_EQ(gpsTowMs, gpsTowMs3);
    }
}

TEST(time_conversion, GPS_to_UTC)
{
    uint32_t gpsWeek = 2294; 
    uint32_t gpsTowMs = 421338800;
    uint32_t gpsLeapS = 18;
    utc_time_t t;
    gpsTowMsToUtcTime(gpsTowMs, gpsLeapS, &t);

    ASSERT_EQ(t.hour, 21);
    ASSERT_EQ(t.minute, 02);
    ASSERT_EQ(t.second, 0);
    ASSERT_EQ(t.millisecond, 800);
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
