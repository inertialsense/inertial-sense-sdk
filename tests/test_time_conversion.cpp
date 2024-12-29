#include <gtest/gtest.h>
#include "ISUtilities.h"
#include "time_conversion.h"
#include "test_data_utils.h"
#include "protocol_nmea.h"


TEST(time_conversion, UTC_to_GPS_to_UTC_time)
{
    // GTEST_SKIP();
    
    int gpsTowMs = 111072800;
    int gpsWeek = 2309;
    int leapS = C_GPS_LEAP_SECONDS;

    SetUtcTimeZone();

    // Cycle through entire range of time of week
    for (gpsTowMs = 0; gpsTowMs < C_MILLISECONDS_PER_WEEK; gpsTowMs += 200)
    {
        std::tm utcTime = stdGpsTimeToUtcDateTime(gpsTowMs/1000, gpsWeek, leapS);
        uint32_t msec = gpsTowMs%1000;

#if 0   // Enable print for debugging
        printf("tow: %d ms %d week   ", gpsTowMs, gpsWeek);
        PrintUtcStdTm(utcTime, msec);
#endif
        uint32_t gpsTowMs2, gpsWeek2;
        stdUtcDateTimeToGpsTime(utcTime, leapS, gpsTowMs2, gpsWeek2);
        gpsTowMs2 = 1000*gpsTowMs2 + msec;
        ASSERT_EQ(gpsTowMs, gpsTowMs2);
        ASSERT_EQ(gpsWeek,  gpsWeek2);

	    int datetime[7] = { 
            utcTime.tm_year + 1900, 
            utcTime.tm_mon + 1, 
            utcTime.tm_mday, 
            utcTime.tm_hour, 
            utcTime.tm_min, 
            utcTime.tm_sec,
            (int)msec };
        uint32_t gpsTowMs4, gpsWeek4;
        UtcDateTimeToGpsTime(datetime, leapS, gpsTowMs4, gpsWeek4);

    	utc_time_t t;
        gpsTowMsToUtcTime(gpsTowMs, leapS, &t);
        ASSERT_EQ(utcTime.tm_hour, t.hour);
        ASSERT_EQ(utcTime.tm_min, t.minute);
        ASSERT_EQ(utcTime.tm_sec, t.second);
        ASSERT_EQ(msec, t.millisecond);

        uint32_t gpsTowMs3;
        utcTimeToGpsTowMs(&t, utcTime.tm_wday, &gpsTowMs3, leapS);
        ASSERT_EQ(gpsTowMs, gpsTowMs3);
    }

    RevertUtcTimeZone();
}

TEST(time_conversion, GPS_to_UTC_to_GPS_time)
{
    // GTEST_SKIP();

    int leapS = C_GPS_LEAP_SECONDS;

    // Cycle through entire range of time of week
    uint32_t gpsWeek = 2345;
    // for (uint32_t gpsWeek = 2345; gpsWeek < 2404; gpsWeek++)
    {
        for (uint32_t gpsTowMs = 0; gpsTowMs < C_MILLISECONDS_PER_WEEK; gpsTowMs += 200)
        {
#if 0       // Enable print for debugging
            printf("tow: %d ms %d week   ", gpsTowMs, gpsWeek);
            // PrintUtcStdTm(utcTime, msec);
            printf("\n");
#endif
            // Convert GPS time and week to UTC date and time
            utc_date_t d;
            utc_time_t t;
            uint32_t milliseconds;
            gpsWeekTowMsToUtcDateTime(gpsWeek, gpsTowMs, leapS, &d, &t, &milliseconds);
            gpsTowMsToUtcTime(gpsTowMs, leapS, &t);

            // Convert UTC date and time to GPS time and week 		
            int datetime[7] = { d.year, d.month, d.day, t.hour, t.minute, t.second, (int)milliseconds };	// year,month,day,hour,min,sec,msec
            uint32_t gpsTowMs2, gpsWeek2;
            UtcDateTimeToGpsTime(datetime, leapS, gpsTowMs2, gpsWeek2);

            ASSERT_EQ(gpsTowMs, gpsTowMs2);
            ASSERT_EQ(gpsWeek, gpsWeek2);
        }
    }
}

TEST(time_conversion, UTC_to_GPS_time_edge_cases)
{
    int leapS = 18;
    struct test_time_conversion {
        int dateTime[7];
        uint32_t gpsWeek;
        uint32_t gpsTowMs;
    };
    uint32_t resultWeek;
    uint32_t resultTowMs;
    
    const test_time_conversion testDates[] = {
        // {Year, month, day, hour, minute, sec, msec}, gpsWeek, gpsTowMs}
        {{1980,  1,  6,  0,  0,  0,   0},      0,      18000},    // GPS epoch start (GPS TOW is leap seconds)
        {{1980,  1,  6, 12,  0,  0,   0},      0,   43218000},    // Midday of GPS epoch day
        {{1980,  1,  6, 23, 59, 59, 900},      0,   86417900},    // End of GPS epoch day
        {{1980,  1,  7,  0,  0,  0,   0},      0,   86418000},    // Start of the second GPS day
        {{1981,  6, 30, 23, 59, 59, 900},     77,  259217900},    // GPS time after 1 year and leap second minus 0.1 s
        {{1981,  7,  1,  0,  0,  0,   0},     77,  259218000},    // GPS time after 1 year and leap second
        {{1999,  8, 22, 14, 45,  0, 123},   1024,   53118123},    // Random date in GPS time, milliseconds round down
        {{2000,  1,  1,  0,  0,  0,   0},   1042,  518418000},    // Y2K start
        {{2016,  2, 29,  0,  0,  0,   0},   1886,   86418000},    // Leap year day
        {{2016, 12, 31, 23, 59, 59, 999},   1930,      17999},    // Leap second (end of 2016)
        {{2018,  9, 23, 15, 46,  0, 123},   2020,   56778123},    // Random date in GPS time, milliseconds round down
        {{2018,  9, 23, 15, 46,  0, 124},   2020,   56778124},    // Random date in GPS time, milliseconds round up
        {{2020,  8, 16, 12,  0,  0, 456},   2119,   43218456},    // Random timestamp
        {{2020,  7, 19, 12, 30, 45, 678},   2115,   45063678},    // Random timestamp
        {{2024,  2,  6, 17, 36, 22,   0},   2300,  236200000},    // Current edge case example
        {{2030, 12, 31, 23, 59, 59, 900},   2660,  259217900},    // Future
        {{2099, 12, 31,  0,  0,  0,   0},   6260,  345618000},    // Start of last GPS day of 2099
        {{2099, 12, 31, 12,  0,  0,   0},   6260,  388818000},    // Midday of last GPS day of 2099
        {{2099, 12, 31, 23, 59, 59,   0},   6260,  432017000},    // Near upper limit
        {{2099, 12, 31, 23, 59, 59, 999},   6260,  432017999},    // Fractional second at the last moment
    };

    // Run tests
    for (const auto& testDate : testDates) {
        UtcDateTimeToGpsTime(testDate.dateTime, leapS, resultTowMs, resultWeek);
#if 0
        printf("{{");
        for (int i=0; i<6; i++)
            printf("%3d, ", testDate.dateTime[i]);
        printf("%4d}", testDate.dateTime[6]);
        printf(", %5d, %10d},\n", resultWeek, resultTowMs);
#endif
        ASSERT_EQ(testDate.gpsWeek, resultWeek);
        ASSERT_EQ(testDate.gpsTowMs, resultTowMs);

        utc_time_t t;
        gpsTowMsToUtcTime(resultTowMs, leapS, &t);
        ASSERT_EQ(testDate.gpsWeek, resultWeek);
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
