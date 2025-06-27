#include <gtest/gtest.h>
#include <vector>
#include "ISEarth.h"
#include "protocol_nmea.h"
#include "test_data_utils.h"
#include "time_conversion.h"
#include "gtest_helpers.h"

using namespace std;

#define PRINT_TEST_DESCRIPTION(description)   { TEST_COUT << "TEST DESCRIPTION: " << description << "\n"; }

#define ASCII_BUF_LEN   200
#define POS_LAT_DEG     40.330578
#define POS_LON_DEG     -111.725816
#define POS_ALT_M       1406.39
#define LEAP_SEC        18

#if 0
#define DEBUG_PRINTF	printf
#else
#define DEBUG_PRINTF	
#endif

void init_sat_and_sig(gps_sat_t* gpsSat, gps_sig_t* gpsSig);

void compareGpsPos(gps_pos_t &g1, gps_pos_t &g2)
{
    EXPECT_NEAR(g1.timeOfWeekMs, g2.timeOfWeekMs, 1);
    EXPECT_EQ(g1.week, g2.week);
    EXPECT_EQ(g1.status, g2.status);
    for (int i=0; i<3; i++)
    {
        EXPECT_NEAR(g1.ecef[i], g2.ecef[i], 0.02);      // 20 cm
    }
    for (int i=0; i<2; i++)
    {
        EXPECT_NEAR(g1.lla[i], g2.lla[i], 1.0e-6);      // (deg)
    }
    EXPECT_NEAR(g1.lla[2], g2.lla[2], 0.01);            // 10 cm
    EXPECT_NEAR(g1.hMSL, g2.hMSL, 0.01);                // 10 cm
    EXPECT_NEAR(g1.hAcc, g2.hAcc, 0.01);                // 10 cm
    EXPECT_NEAR(g1.vAcc, g2.vAcc, 0.01);                // 10 cm
    EXPECT_NEAR(g1.pDop, g2.pDop, 0.01);
    EXPECT_NEAR(g1.cnoMean, g2.cnoMean, 0.01);
    EXPECT_NEAR(g1.towOffset, g2.towOffset, 0.01);
    EXPECT_EQ(g1.leapS, g2.leapS);
    EXPECT_EQ(g1.satsUsed, g2.satsUsed);
    EXPECT_EQ(g1.cnoMeanSigma, g2.cnoMeanSigma);
    EXPECT_EQ(g1.status2, g2.status2);
}

void compareGpsVel(gps_vel_t &g1, gps_vel_t &g2)
{
    EXPECT_NEAR(g1.timeOfWeekMs, g2.timeOfWeekMs, 1);
    for (int i=0; i<3; i++)
    {
        EXPECT_NEAR(g1.vel[i], g2.vel[i], 0.01);        // 10 cm
    }
    EXPECT_NEAR(g1.sAcc, g2.sAcc, 0.01);                // 10 cm/s
    EXPECT_EQ(g1.status, g2.status);
}

uint32_t g_cpu_msec;
sys_params_t g_sysParams;
debug_array_t g_debug;

void initGlobals()
{
    memset(&g_sysParams, 0, sizeof(g_sysParams));
    g_cpu_msec = 0;
    memset(&g_debug, 0, sizeof(g_debug));
}

bool timeWithin(uint32_t timeSec, uint32_t startSec, uint32_t durationSec)
{
    return (timeSec >= startSec) && (timeSec < (startSec + durationSec));
}

TEST(protocol_nmea, zda_gps_time_skip)
{
    GTEST_SKIP();   // This test must be run manually as the statically SDK build does not include the ZDA TOD work around code

#ifdef _WIN32
    GTEST_SKIP() << "Skipping test on Windows.";
#endif
    printf("DESCRIPTION: Test that ZDA time skip detect code works correctly for 1-2 second jumps in the ZDA UTC time due to jumps in GPS time of week.\n");
    initGlobals();
    char buf[1024]; 
    gps_pos_t pos = {};
    pos.leapS = 18;
    pos.week = 2345;
    bool faultLast = false;
    int simulatedOffsetMs = 0;

    for (int timeSec=0; timeSec<C_SECONDS_PER_WEEK-1; timeSec++)
    {
        pos.timeOfWeekMs = timeSec*1000;
        g_cpu_msec = timeSec*1000;
        bool fault = 
            timeWithin(timeSec, C_SECONDS_PER_WEEK/4, 200) || 
            timeWithin(timeSec, C_SECONDS_PER_WEEK/2, 10);

        bool toggle = fault != faultLast;
        faultLast = fault;
        if (toggle)
        {
            if (fault)
            {
                simulatedOffsetMs += 1000;
                if (simulatedOffsetMs>2000) { simulatedOffsetMs = 0; }
            }
        }

        if (fault)
        {   // Simulate offset in GPS tow
            pos.timeOfWeekMs += simulatedOffsetMs;
        }

        int n = nmea_zda(buf, sizeof(buf), pos);

        ASSERT_EQ((g_sysParams.genFaultCode&GFC_GNSS_RECEIVER_TIME) != 0, toggle) << "genFaultCode failed at timeSec: " << timeSec;
        ASSERT_EQ(g_debug.f[8] != 0, toggle) << "Correction offset failed at timeSec: " << timeSec;
        ASSERT_EQ(g_debug.f[7], (fault ? simulatedOffsetMs/1000 : 0)) << "Correction offset failed at timeSec: " << timeSec;

        uint32_t gpsTowMs;
        uint32_t gpsWeek;
        utc_date_t utcDate;
        utc_time_t utcTime;
        nmea_parse_zda(buf, n, gpsTowMs, gpsWeek, utcDate, utcTime, pos.leapS);
#if 0
        // if (toggle)
        {
            printf("timeSec: %d  ", timeSec);
            printf("gpsTowMs: %d  ", pos.timeOfWeekMs);
            if (fault)  printf("(fault on)  ");
            else        printf("(fault off) ");
            PrintUtcDateTime(utcDate, utcTime);
        }
#endif
        // ASSERT_EQ(gpsTowMs, timeSec*1000) << "Continuous at timeSec: " << timeSec;
        // ASSERT_EQ(gpsTowMs, pos.timeOfWeekMs - (fault?simulatedOffsetMs:0)) << "GPS tow failed at timeSec: " << timeSec;

        g_debug.i[6] = 0;
        g_debug.f[8] = 0;
        g_sysParams.genFaultCode = 0;
    }
}

TEST(protocol_nmea, zda_cpu_time_skip)
{
#ifdef _WIN32
    GTEST_SKIP() << "Skipping test on Windows.";
#endif
    printf("DESCRIPTION: Test that ZDA work around will pregress linearly and not apply incorrectly apply offset when GPS update is missing.\n");
    initGlobals();
    char buf[1024]; 
    gps_pos_t pos = {};
    pos.leapS = 18;
    pos.week = 2345;
    bool faultLast = false;    

    for (int timeSec=0; timeSec<C_SECONDS_PER_WEEK-1; timeSec++)
    {
        pos.timeOfWeekMs = timeSec*1000;
        g_cpu_msec = timeSec*1000;
        bool fault = 
            timeWithin(timeSec, C_SECONDS_PER_WEEK/4, 200) || 
            timeWithin(timeSec, C_SECONDS_PER_WEEK/2, 10);

        bool toggle = fault != faultLast;
        faultLast = fault;
        if (toggle)
        {   // Simulate absent message
            // printf("timeSec: %d  (absent)\n", timeSec);
            continue;
        }

        int n = nmea_zda(buf, sizeof(buf), pos);

        ASSERT_EQ(g_debug.i[5], 0) << "Correction offset non-zero at timeSec: " << timeSec;
        ASSERT_EQ(g_debug.i[6], 0) << "Correction offset non-zero at timeSec: " << timeSec;

        uint32_t gpsTowMs;
        uint32_t gpsWeek;
        utc_date_t utcDate;
        utc_time_t utcTime;
        nmea_parse_zda(buf, n, gpsTowMs, gpsWeek, utcDate, utcTime, pos.leapS);
#if 0
        if (toggle)
        {
            printf("timeSec: %d  ", timeSec);
            printf("gpsTowMs: %d  ", pos.timeOfWeekMs);
            PrintUtcDateTime(utcDate, utcTime);
        }
#endif
        ASSERT_EQ(gpsTowMs, timeSec*1000) << "Linear time at timeSec: " << timeSec;
        ASSERT_EQ(gpsTowMs, pos.timeOfWeekMs) << "GPS tow failed at timeSec: " << timeSec;

        g_debug.i[6] = 0;
        g_sysParams.genFaultCode = 0;
    }
}

TEST(protocol_nmea, nmea_parse_asce)
{
	PRINT_TEST_DESCRIPTION("Tests the $ASCE parser function nmea_parse_asce().");

    rmci_t rmci[NUM_COM_PORTS] = {};
    int port = 1;
    rmci_t &r = rmci[port];
    r.rmcNmea.nmeaPeriod[NMEA_MSG_ID_PINS1] = 2;
    r.rmcNmea.nmeaPeriod[NMEA_MSG_ID_PPIMU] = 1;
    r.rmcNmea.nmeaPeriod[NMEA_MSG_ID_GNGGA] = 1;
    r.rmcNmea.nmeaPeriod[NMEA_MSG_ID_PINS2] = 10;
    r.rmcNmea.nmeaPeriod[NMEA_MSG_ID_GNGSV] = 7;
    r.rmcNmea.nmeaBits = 
        NMEA_RMC_BITS_PINS1 |
        NMEA_RMC_BITS_PPIMU |
        NMEA_RMC_BITS_GNGGA |
        NMEA_RMC_BITS_PINS2 |
        NMEA_RMC_BITS_GNGSV;
    uint32_t options = RMC_OPTIONS_PRESERVE_CTRL | RMC_OPTIONS_PERSISTENT;

    char a[ASCII_BUF_LEN] = {};
    int n=0;
	nmea_sprint(a, ASCII_BUF_LEN, n, "$ASCE,%u", options);
    nmea_sprint(a, ASCII_BUF_LEN, n, ",PINS1,%u", r.rmcNmea.nmeaPeriod[NMEA_MSG_ID_PINS1]);
    nmea_sprint(a, ASCII_BUF_LEN, n, ",PPIMU,%u", r.rmcNmea.nmeaPeriod[NMEA_MSG_ID_PPIMU]);
    nmea_sprint(a, ASCII_BUF_LEN, n, ",GNGGA,%u", r.rmcNmea.nmeaPeriod[NMEA_MSG_ID_GNGGA]);
    nmea_sprint(a, ASCII_BUF_LEN, n, ",%u,%u", NMEA_MSG_ID_PINS2, r.rmcNmea.nmeaPeriod[NMEA_MSG_ID_PINS2]);
    nmea_sprint(a, ASCII_BUF_LEN, n, ",%u,%u", NMEA_MSG_ID_GNGSV, r.rmcNmea.nmeaPeriod[NMEA_MSG_ID_GNGSV]);
	nmea_sprint_footer(a, ASCII_BUF_LEN, n);
    
    rmci_t outRmci[NUM_COM_PORTS] = {};
    uint32_t outOptions = nmea_parse_asce(port, a, n, outRmci);

    //cout << a << endl;

    ASSERT_EQ( options, outOptions );
    for (int i=0; i<NUM_COM_PORTS; i++)
    {
        rmci_t &a = rmci[i];
        rmci_t &b = outRmci[i];
        ASSERT_EQ( a.rmc.bits, b.rmc.bits );
         
        // cout << "I: " << i << " a: " << a.rmcNmea.nmeaBits << " b: " <<  b.rmcNmea.nmeaBits << "\n"; 
        ASSERT_EQ( a.rmcNmea.nmeaBits, b.rmcNmea.nmeaBits );
        for (int j=1; j < NMEA_MSG_ID_COUNT; j++)
        {
            // cout << "J: " << j << " a: " << a.rmcNmea.nmeaPeriod[j] << " b: " <<  b.rmcNmea.nmeaPeriod[j] << "\n";  
            ASSERT_EQ( a.rmcNmea.nmeaPeriod[j], b.rmcNmea.nmeaPeriod[j] );
        }   
    }
}

TEST(protocol_nmea, INFO)
{
    dev_info_t info = {};
    info.serialNumber = 1234;
    for (int i=0; i<4; i++)
    {
        info.hardwareVer[i] = 10+i;
        info.firmwareVer[i] = 20+i;
    }
    info.buildNumber = 5678;
    for (int i=0; i<4; i++)
    {
        info.protocolVer[i] = 30+i;
    }
    info.repoRevision = 789;
    snprintf(info.manufacturer, DEVINFO_MANUFACTURER_STRLEN, "manufacturer string 123");
    info.buildType = 'r';
    info.buildYear = 23;
    info.buildMonth = 6;
    info.buildDay = 9;
    info.buildHour = 12;
    info.buildMinute = 8;
    info.buildSecond = 20;
    info.buildMillisecond = 23;

    snprintf(info.addInfo, DEVINFO_ADDINFO_STRLEN, "additional string   123");

    char abuf[ASCII_BUF_LEN] = { 0 };
    nmea_dev_info(abuf, ASCII_BUF_LEN, info);

    dev_info_t result = {};
    nmea_parse_info(result, abuf, ASCII_BUF_LEN);

    int compVal = memcmp(&info, &result, sizeof(result));
    if (compVal != 0)    printf("%s\n", abuf);
    ASSERT_EQ(compVal, 0);
}

TEST(protocol_nmea, PIMU)
{
    imu_t imu = {};
    imu.time = 123.456;
    for (int i=0; i<3; i++)
    {
        imu.I.pqr[i] = 10.0f+i;
        imu.I.acc[i] = 20.0f+i;
    }

    char abuf[ASCII_BUF_LEN] = { 0 };
    nmea_pimu(abuf, ASCII_BUF_LEN, imu, "$PIMU");
    // printf("%s\n", abuf);
    imu_t result = {};
    nmea_parse_pimu(result, abuf, ASCII_BUF_LEN);
    ASSERT_EQ(memcmp(&imu, &result, sizeof(result)), 0);
}

TEST(protocol_nmea, PRIMU)
{
    imu_t imu = {};
    imu.time = 123.456;
    for (int i=0; i<3; i++)
    {
        imu.I.pqr[i] = 10.0f+i;
        imu.I.acc[i] = 20.0f+i;
    }

    char abuf[ASCII_BUF_LEN] = { 0 };
    nmea_pimu(abuf, ASCII_BUF_LEN, imu, "$PRIMU");
    // printf("%s\n", abuf);
    imu_t result = {};
    nmea_parse_pimu_to_rimu(result, abuf, ASCII_BUF_LEN);
    ASSERT_EQ(memcmp(&imu, &result, sizeof(result)), 0);
}

TEST(protocol_nmea, PPIMU)
{
    pimu_t pimu = {};
    pimu.time = 123.456;
    pimu.dt = 7.89f;
    for (int i=0; i<3; i++)
    {
        pimu.theta[i] = 10.0f+i;
        pimu.vel[i] = 20.0f+i;
    }

    char abuf[ASCII_BUF_LEN] = { 0 };
    nmea_ppimu(abuf, ASCII_BUF_LEN, pimu);
    // printf("%s\n", abuf);
    pimu_t result = {};
    nmea_parse_ppimu(result, abuf, ASCII_BUF_LEN);
    ASSERT_EQ(memcmp(&pimu, &result, sizeof(result)), 0);
}

TEST(protocol_nmea, PINS1)
{
    ins_1_t ins = {};
    ins.week = 12;
    ins.timeOfWeek = 3.456;
    ins.insStatus = 789;
    ins.hdwStatus = 123;
    for (int i=0; i<3; i++)
    {
        ins.theta[i] = 10.0f+i;
        ins.uvw[i] = 20.0f+i;
        ins.ned[i] = 40.0f+i;
    }
    ins.lla[0] = POS_LAT_DEG;
    ins.lla[1] = POS_LON_DEG;
    ins.lla[2] = POS_ALT_M;

    char abuf[ASCII_BUF_LEN] = { 0 };
    nmea_pins1(abuf, ASCII_BUF_LEN, ins);
    // printf("%s\n", abuf);
    ins_1_t result = {};
    nmea_parse_pins1(result, abuf, ASCII_BUF_LEN);
    ASSERT_EQ(memcmp(&ins, &result, sizeof(result)), 0);
}

TEST(protocol_nmea, PINS2)
{
    ins_2_t ins = {};
    ins.week = 12;
    ins.timeOfWeek = 3.456;
    ins.insStatus = 789;
    ins.hdwStatus = 123;
    for (int i=0; i<3; i++)
    {
        ins.qn2b[i] = 10.0f+i;
    }
    for (int i=0; i<3; i++)
    {
        ins.uvw[i] = 20.0f+i;
    }
    ins.lla[0] = POS_LAT_DEG;
    ins.lla[1] = POS_LON_DEG;
    ins.lla[2] = POS_ALT_M;

    char abuf[ASCII_BUF_LEN] = { 0 };
    nmea_pins2(abuf, ASCII_BUF_LEN, ins);
    // printf("%s\n", abuf);
    ins_2_t result = {};
    nmea_parse_pins2(result, abuf, ASCII_BUF_LEN);
    ASSERT_EQ(memcmp(&ins, &result, sizeof(result)), 0);
}

TEST(protocol_nmea, PGPSP)
{
    gps_pos_t pos = {};
    gps_vel_t vel = {};
    pos.week = 2309;
    pos.timeOfWeekMs = vel.timeOfWeekMs = 370659600;
    pos.satsUsed = 45;
    pos.status = GPS_STATUS_NUM_SATS_USED_MASK & pos.satsUsed;
    for (int i=0; i<3; i++)
    {
        // pos.ecef[i] = 20.0f+i;   // Not in full conversion
        vel.vel[i] = 40.0f+i;
    }
    pos.lla[0] = POS_LAT_DEG;
    pos.lla[1] = POS_LON_DEG;
    pos.lla[2] = POS_ALT_M;
    pos.hMSL = 89;
    pos.hAcc = 123;
    pos.vAcc = 45;
    pos.pDop = 6;
    pos.cnoMean = 78;
    pos.towOffset = 9;
    pos.leapS = 12;
    vel.sAcc = 345;

    char abuf[ASCII_BUF_LEN] = { 0 };
    nmea_pgpsp(abuf, ASCII_BUF_LEN, pos, vel);
    // printf("%s\n", abuf);
    gps_pos_t resultPos = {};
    gps_vel_t resultVel = {};
    nmea_parse_pgpsp(resultPos, resultVel, abuf, ASCII_BUF_LEN);

    compareGpsPos(pos, resultPos);
    compareGpsVel(vel, resultVel);
}

#define OPERATING_LIMIT_MPS     500     // (m/s)

TEST(protocol_nmea, PGPSP_sweep_operating_range)
{
    double invTowMsMax = 1.0/(double)C_MILLISECONDS_PER_WEEK;

    // Cycle through entire range of time of week in milliseconds
    for (int towMs = 0; towMs < C_MILLISECONDS_PER_WEEK; towMs += 500)
    {   // Scale will transition from 0.0 to 1.0
        double scale = ((double)towMs) * invTowMsMax;

        gps_pos_t pos = {};
        gps_vel_t vel = {};
        pos.week = 2309;
        pos.timeOfWeekMs = vel.timeOfWeekMs = towMs;
        pos.satsUsed = 45;
        pos.status = GPS_STATUS_NUM_SATS_USED_MASK & pos.satsUsed;
        // for (int i=0; i<3; i++)
        // {
        //     // pos.ecef[i] = 20.0f+i;   // Not in full conversion
        //     vel.vel[i] = -OPERATING_LIMIT_MPS + 2*OPERATING_LIMIT_MPS*scale;
        // }
        pos.hMSL = -100 + 50000 * scale;
        pos.lla[0] =  -90.0 + 180.0 * scale;
        pos.lla[1] = -180.0 + 230.0 * scale;
        pos.lla[2] = pos.hMSL - 18.8;
        pos.hAcc = 123;
        pos.vAcc = 45;
        pos.pDop = 6;
        pos.cnoMean = 78;
        pos.towOffset = 9;
        pos.leapS = 12;
        vel.sAcc = 345;

        char abuf[ASCII_BUF_LEN] = { 0 };
        nmea_pgpsp(abuf, ASCII_BUF_LEN, pos, vel);
        // printf("%s\n", abuf);
        gps_pos_t resultPos = {};
        gps_vel_t resultVel = {};
        nmea_parse_pgpsp(resultPos, resultVel, abuf, ASCII_BUF_LEN);

        compareGpsPos(pos, resultPos);
        compareGpsVel(vel, resultVel);
    }
}

TEST(protocol_nmea, GGA)
{
    char gga[ASCII_BUF_LEN] = {0};
    snprintf(gga, ASCII_BUF_LEN, "$GNGGA,231100.200,4003.34247,N,11139.51850,W,2,12,0.47,1438.20,M,-18.80,M,,*73\r\n");
    gps_pos_t pos = {};
    pos.week = 2260;
    pos.timeOfWeekMs = 342678200;
    pos.satsUsed = 12;
    pos.status = 
        GPS_STATUS_NUM_SATS_USED_MASK & pos.satsUsed |
        GPS_STATUS_FLAGS_FIX_OK |
        GPS_STATUS_FLAGS_DGPS_USED |
        GPS_STATUS_FIX_DGPS |
        GPS_STATUS_FLAGS_GPS_NMEA_DATA;        
    pos.hMSL = 1438.2f;
    pos.lla[0] =  ( 40.0 +  3.34247/60.0);
    pos.lla[1] = -(111.0 + 39.51850/60.0);
    pos.lla[2] = pos.hMSL - 18.8;
    pos.pDop = 0.47f;
    pos.leapS = LEAP_SEC;
	// Convert LLA to ECEF.  Ensure LLA uses ellipsoid altitude
	ixVector3d lla;
	lla[0] = DEG2RAD(pos.lla[0]);
	lla[1] = DEG2RAD(pos.lla[1]);
	lla[2] = pos.lla[2];		// Use ellipsoid altitude
	lla2ecef(lla, pos.ecef);

    char abuf[ASCII_BUF_LEN] = { 0 };
    int n = nmea_gga(abuf, ASCII_BUF_LEN, pos);
    // printf("%s\n", gga);
    // printf("%s\n", abuf);
    ASSERT_EQ(memcmp(&gga, &abuf, n), 0);

    gps_pos_t pos2 = {};
    pos2.week = pos.week;
    pos2.leapS = pos.leapS;
    utc_time_t t;
    int utcWeekday = gpsTowMsToUtcWeekday(pos.timeOfWeekMs, pos.leapS);
    nmea_parse_gga(abuf, ASCII_BUF_LEN, pos2, t, utcWeekday);
    pos.hAcc = pos2.hAcc;

    compareGpsPos(pos, pos2);
}

TEST(protocol_nmea, GGA_sweep_operating_range)
{
    double invTowMsMax = 1.0/(double)C_MILLISECONDS_PER_WEEK;

    // Cycle through entire range of time of week in milliseconds
    for (int towMs = 0; towMs < C_MILLISECONDS_PER_WEEK; towMs += 500)
    {   // Scale will transition from 0.0 to 1.0
        double scale = ((double)towMs) * invTowMsMax;

        gps_pos_t pos = {};
        pos.week = 2309;
        pos.timeOfWeekMs = towMs;
        pos.satsUsed = 12;
        pos.status =
            GPS_STATUS_NUM_SATS_USED_MASK & pos.satsUsed |
            GPS_STATUS_FLAGS_FIX_OK |
            GPS_STATUS_FLAGS_DGPS_USED |
            GPS_STATUS_FIX_DGPS |
            GPS_STATUS_FLAGS_GPS_NMEA_DATA;
        pos.hMSL = -100 + 50000 * scale;
        pos.lla[0] =  -90.0 + 180.0 * scale;
        pos.lla[1] = -180.0 + 230.0 * scale;
        pos.lla[2] = pos.hMSL - 18.8;
        pos.pDop = 0.47f;
        pos.leapS = LEAP_SEC;
        // Convert LLA to ECEF.  Ensure LLA uses ellipsoid altitude
        ixVector3d lla;
        lla[0] = DEG2RAD(pos.lla[0]);
        lla[1] = DEG2RAD(pos.lla[1]);
        lla[2] = pos.lla[2];		// Use ellipsoid altitude
        lla2ecef(lla, pos.ecef);

        char abuf[ASCII_BUF_LEN] = { 0 };
        int n = nmea_gga(abuf, ASCII_BUF_LEN, pos);
        // printf("%d ms, %d week, %s\n", towMs, pos.week, abuf);

        gps_pos_t pos2 = {};
        pos2.week = pos.week;
        pos2.leapS = pos.leapS;
        utc_time_t t;
        int utcWeekday = gpsTowMsToUtcWeekday(pos.timeOfWeekMs, pos.leapS);
        nmea_parse_gga(abuf, ASCII_BUF_LEN, pos2, t, utcWeekday);
        pos.hAcc = pos2.hAcc;

        compareGpsPos(pos, pos2);

        char abuf2[ASCII_BUF_LEN] = {0};
        n = nmea_gga(abuf2, ASCII_BUF_LEN, pos2);
        // printf("%d ms, %d week, %s\n", pos2.timeOfWeekMs, pos2.week, abuf2);
        ASSERT_EQ(memcmp(&abuf, &abuf2, n), 0) << "towMs " << towMs << "  abuf: " << abuf << "abuf2: " << abuf2;
    }
}

TEST(protocol_nmea, GGA2)
{
    char gga[ASCII_BUF_LEN] = { 0 };
    snprintf(gga, ASCII_BUF_LEN, "$GNGGA,181457.400,3903.80427,N,07709.29556,W,2,12,0.47,1438.20,M,-18.80,M,,*7D\r\n");
    gps_pos_t pos = {};
    pos.week = 2260;
    pos.timeOfWeekMs = 152115400;
    pos.satsUsed = 12;
    pos.status =
        GPS_STATUS_NUM_SATS_USED_MASK & pos.satsUsed |
        GPS_STATUS_FLAGS_FIX_OK |
        GPS_STATUS_FLAGS_DGPS_USED |
        GPS_STATUS_FIX_DGPS |
        GPS_STATUS_FLAGS_GPS_NMEA_DATA;
    pos.hMSL = 1438.2f;
    pos.lla[0] = (39.0 + 3.80427 / 60.0);
    pos.lla[1] = -(77.0 + 9.29556 / 60.0);
    pos.lla[2] = pos.hMSL - 18.8;
    pos.pDop = 0.47f;
    pos.leapS = LEAP_SEC;
    // Convert LLA to ECEF.  Ensure LLA uses ellipsoid altitude
    ixVector3d lla;
    lla[0] = DEG2RAD(pos.lla[0]);
    lla[1] = DEG2RAD(pos.lla[1]);
    lla[2] = pos.lla[2];		// Use ellipsoid altitude
    lla2ecef(lla, pos.ecef);

    char abuf[ASCII_BUF_LEN] = { 0 };
    int n = nmea_gga(abuf, ASCII_BUF_LEN, pos);
    //printf("%s\n", gga);
    //printf("%s\n", abuf);
    ASSERT_EQ(memcmp(&gga, &abuf, n), 0);

    gps_pos_t result = {};
    result.week = pos.week;
    result.leapS = pos.leapS;
    utc_time_t t;
    int utcWeekday = gpsTowMsToUtcWeekday(pos.timeOfWeekMs, pos.leapS);
    nmea_parse_gga(abuf, ASCII_BUF_LEN, result, t, utcWeekday);
    pos.hAcc = result.hAcc;
    ASSERT_EQ(memcmp(&pos, &result, sizeof(result)), 0);
}

TEST(protocol_nmea, GGA3)
{
    char gga[ASCII_BUF_LEN] = { 0 };
    snprintf(gga, ASCII_BUF_LEN, "$GNGGA,162148.000,5150.60402,N,00058.30337,W,2,10,0.47,1438.20,M,-18.80,M,,*71\r\n");
    gps_pos_t pos = {};
    pos.week = 2260;
    pos.timeOfWeekMs = 231726000;
    pos.satsUsed = 10;
    pos.status =
            GPS_STATUS_NUM_SATS_USED_MASK & pos.satsUsed |
            GPS_STATUS_FLAGS_FIX_OK |
            GPS_STATUS_FLAGS_DGPS_USED |
            GPS_STATUS_FIX_DGPS |
            GPS_STATUS_FLAGS_GPS_NMEA_DATA;
    pos.hMSL = 1438.2f;
    pos.lla[0] = (51.0 + 50.60402 / 60.0);
    pos.lla[1] = -(0.0 + 58.30337 / 60.0);
    pos.lla[2] = pos.hMSL - 18.8;
    pos.pDop = 0.47f;
    pos.leapS = LEAP_SEC;
    // Convert LLA to ECEF.  Ensure LLA uses ellipsoid altitude
    ixVector3d lla;
    lla[0] = DEG2RAD(pos.lla[0]);
    lla[1] = DEG2RAD(pos.lla[1]);
    lla[2] = pos.lla[2];		// Use ellipsoid altitude
    lla2ecef(lla, pos.ecef);

    char abuf[ASCII_BUF_LEN] = { 0 };
    int n = nmea_gga(abuf, ASCII_BUF_LEN, pos);
    //printf("%s\n", gga);
    //printf("%s\n", abuf);
    EXPECT_EQ(memcmp(&gga, &abuf, n), 0) << "Expected: " << gga << "Actual:" << abuf;

    gps_pos_t result = {};
    result.week = pos.week;
    result.leapS = pos.leapS;
    utc_time_t t;
    int utcWeekday = gpsTowMsToUtcWeekday(pos.timeOfWeekMs, pos.leapS);
    nmea_parse_gga(abuf, ASCII_BUF_LEN, result, t, utcWeekday);
    pos.hAcc = result.hAcc;
    EXPECT_EQ(memcmp(&pos, &result, sizeof(result)), 0);
}

TEST(protocol_nmea, GGA4)
{
    char gga[ASCII_BUF_LEN] = { 0 };
    snprintf(gga, ASCII_BUF_LEN, "$GNGGA,162148.000,0050.60402,S,00035.30337,E,2,10,0.47,1438.20,M,-18.80,M,,*71\r\n");
    gps_pos_t pos = {};
    pos.week = 2260;
    pos.timeOfWeekMs = 231726000;
    pos.satsUsed = 10;
    pos.status =
            GPS_STATUS_NUM_SATS_USED_MASK & pos.satsUsed |
            GPS_STATUS_FLAGS_FIX_OK |
            GPS_STATUS_FLAGS_DGPS_USED |
            GPS_STATUS_FIX_DGPS |
            GPS_STATUS_FLAGS_GPS_NMEA_DATA;
    pos.hMSL = 1438.2f;
    pos.lla[0] = -(0.0 + 50.60402 / 60.0);
    pos.lla[1] = (0.0 + 35.30337 / 60.0);
    pos.lla[2] = pos.hMSL - 18.8;
    pos.pDop = 0.47f;
    pos.leapS = LEAP_SEC;
    // Convert LLA to ECEF.  Ensure LLA uses ellipsoid altitude
    ixVector3d lla;
    lla[0] = DEG2RAD(pos.lla[0]);
    lla[1] = DEG2RAD(pos.lla[1]);
    lla[2] = pos.lla[2];		// Use ellipsoid altitude
    lla2ecef(lla, pos.ecef);

    char abuf[ASCII_BUF_LEN] = { 0 };
    int n = nmea_gga(abuf, ASCII_BUF_LEN, pos);
    //printf("%s\n", gga);
    //printf("%s\n", abuf);
    EXPECT_EQ(memcmp(&gga, &abuf, n), 0) << "Expected: " << gga << "Actual:" << abuf;

    gps_pos_t result = {};
    result.week = pos.week;
    result.leapS = pos.leapS;
    utc_time_t t;
    int utcWeekday = gpsTowMsToUtcWeekday(pos.timeOfWeekMs, pos.leapS);
    nmea_parse_gga(abuf, ASCII_BUF_LEN, result, t, utcWeekday);
    pos.hAcc = result.hAcc;
    EXPECT_EQ(memcmp(&pos, &result, sizeof(result)), 0);
}

TEST(protocol_nmea, GLL)
{
    gps_pos_t pos = {};

    pos.week = 2270;
    pos.timeOfWeekMs = 370659600;
    pos.status = (GPS_STATUS_NUM_SATS_USED_MASK & pos.satsUsed) | GPS_STATUS_FIX_2D;
    pos.lla[0] = POS_LAT_DEG;
    pos.lla[1] = POS_LON_DEG;
    pos.leapS = LEAP_SEC;

    char abuf[ASCII_BUF_LEN] = { 0 };
    nmea_gll(abuf, ASCII_BUF_LEN, pos);
    // printf("%s\n", abuf);
    gps_pos_t result = {};
    result.leapS = pos.leapS;
    result.week = pos.week;
    uint32_t weekday = pos.timeOfWeekMs / C_MILLISECONDS_PER_DAY;
    utc_time_t t;
    nmea_parse_gll(abuf, ASCII_BUF_LEN, result, t, weekday);

    int comValue = memcmp(&pos, &result, sizeof(result));

    if (comValue != 0)
    {
        printf("%s", abuf);
        printf("lat in: %f\r\nlat out:%f\r\n", pos.lla[0], result.lla[0]);
        printf("lon in: %f\r\nlon out:%f\r\n", pos.lla[1], result.lla[1]);
        printf("time in: %d\r\ntime out:%d\r\n", pos.timeOfWeekMs, result.timeOfWeekMs);
        printf("stat in: %d\r\nstat out:%d\r\n", pos.status, result.status);
    }

    ASSERT_EQ(comValue, 0);
}

TEST(protocol_nmea, GLL_noFixStat)
{
    gps_pos_t pos = {};

    pos.week = 2270;
    pos.timeOfWeekMs = 370659600;
    pos.status = GPS_STATUS_NUM_SATS_USED_MASK & pos.satsUsed;
    pos.lla[0] = POS_LAT_DEG;
    pos.lla[1] = POS_LON_DEG;
    pos.leapS = LEAP_SEC;

    char abuf[ASCII_BUF_LEN] = { 0 };
    nmea_gll(abuf, ASCII_BUF_LEN, pos);
    // printf("%s\n", abuf);
    gps_pos_t result = {};
    result.leapS = pos.leapS;
    result.week = pos.week;
    uint32_t weekday = pos.timeOfWeekMs / C_MILLISECONDS_PER_DAY;
    utc_time_t t;
    nmea_parse_gll(abuf, ASCII_BUF_LEN, result, t, weekday);

    // alter for test results 
    pos.lla[0] = 0;
    pos.lla[1] = 0;
    pos.status &= ~(GPS_STATUS_FIX_MASK);

    int comValue = memcmp(&pos, &result, sizeof(result));

    if (comValue != 0)
    {
        printf("%s", abuf);
        printf("lat in: %f\r\nlat out:%f\r\n", pos.lla[0], result.lla[0]);
        printf("lon in: %f\r\nlon out:%f\r\n", pos.lla[1], result.lla[1]);
        printf("time in: %d\r\ntime out:%d\r\n", pos.timeOfWeekMs, result.timeOfWeekMs);
        printf("stat in: %d\r\nstat out:%d\r\n", pos.status, result.status);
    }

    ASSERT_EQ(comValue, 0);
}

TEST(protocol_nmea, GLL_noLat)
{
    gps_pos_t pos = {};

    pos.week = 2270;
    pos.timeOfWeekMs = 370659600;
    pos.status = (GPS_STATUS_NUM_SATS_USED_MASK & pos.satsUsed) | GPS_STATUS_FIX_2D;
    pos.lla[0] = POS_LAT_DEG;
    pos.lla[1] = 0;
    pos.leapS = LEAP_SEC;

    char abuf[ASCII_BUF_LEN] = { 0 };
    nmea_gll(abuf, ASCII_BUF_LEN, pos);
    // printf("%s\n", abuf);
    gps_pos_t result = {};
    result.leapS = pos.leapS;
    result.week = pos.week;
    uint32_t weekday = pos.timeOfWeekMs / C_MILLISECONDS_PER_DAY;
    utc_time_t t;
    nmea_parse_gll(abuf, ASCII_BUF_LEN, result, t, weekday);

    // alter for test results 
    // pos.lla[0] = 0;
    // pos.lla[1] = 0;
    // pos.status &= ~(GPS_STATUS_FIX_MASK);

    int comValue = memcmp(&pos, &result, sizeof(result));

    if (comValue != 0)
    {
        printf("%s", abuf);
        printf("lat in: %f\r\nlat out:%f\r\n", pos.lla[0], result.lla[0]);
        printf("lon in: %f\r\nlon out:%f\r\n", pos.lla[1], result.lla[1]);
        printf("time in: %d\r\ntime out:%d\r\n", pos.timeOfWeekMs, result.timeOfWeekMs);
        printf("stat in: %d\r\nstat out:%d\r\n", pos.status, result.status);
    }

    ASSERT_EQ(comValue, 0);
}

TEST(protocol_nmea, GLL_noLon)
{
    gps_pos_t pos = {};

    pos.week = 2270;
    pos.timeOfWeekMs = 370659600;
    pos.status = (GPS_STATUS_NUM_SATS_USED_MASK & pos.satsUsed) | GPS_STATUS_FIX_2D;
    pos.lla[0] = 0;
    pos.lla[1] = POS_LON_DEG;
    pos.leapS = LEAP_SEC;

    char abuf[ASCII_BUF_LEN] = { 0 };
    nmea_gll(abuf, ASCII_BUF_LEN, pos);
    // printf("%s\n", abuf);
    gps_pos_t result = {};
    result.leapS = pos.leapS;
    result.week = pos.week;
    uint32_t weekday = pos.timeOfWeekMs / C_MILLISECONDS_PER_DAY;
    utc_time_t t;
    nmea_parse_gll(abuf, ASCII_BUF_LEN, result, t, weekday);

    // alter for test results 
    // pos.lla[0] = 0;
    // pos.lla[1] = 0;
    // pos.status &= ~(GPS_STATUS_FIX_MASK);

    int comValue = memcmp(&pos, &result, sizeof(result));

    if (comValue != 0)
    {
        printf("%s", abuf);
        printf("lat in: %f\r\nlat out:%f\r\n", pos.lla[0], result.lla[0]);
        printf("lon in: %f\r\nlon out:%f\r\n", pos.lla[1], result.lla[1]);
        printf("time in: %d\r\ntime out:%d\r\n", pos.timeOfWeekMs, result.timeOfWeekMs);
        printf("stat in: %d\r\nstat out:%d\r\n", pos.status, result.status);
    }

    ASSERT_EQ(comValue, 0);
}

TEST(protocol_nmea, GLL_void)
{
    gps_pos_t pos = {};

    pos.week = 0;
    pos.timeOfWeekMs = 370659600;
    pos.status = (GPS_STATUS_NUM_SATS_USED_MASK & pos.satsUsed) | GPS_STATUS_FIX_2D;
    pos.lla[0] = POS_LAT_DEG;
    pos.lla[1] = POS_LON_DEG;
    pos.leapS = LEAP_SEC;

    char abuf[ASCII_BUF_LEN] = { 0 };
    nmea_gll(abuf, ASCII_BUF_LEN, pos);
    // printf("%s\n", abuf);
    gps_pos_t result = {};
    result.leapS = pos.leapS;
    result.week = pos.week;
    uint32_t weekday = pos.timeOfWeekMs / C_MILLISECONDS_PER_DAY;
    utc_time_t t;
    nmea_parse_gll(abuf, ASCII_BUF_LEN, result, t, weekday);

    // alter for test results 
    pos.lla[0] = 0;
    pos.lla[1] = 0;
    pos.timeOfWeekMs = 0;
    pos.status &= ~(GPS_STATUS_FIX_MASK);

    int comValue = memcmp(&pos, &result, sizeof(result));

    if (comValue != 0)
    {
        printf("%s", abuf);
        printf("lat in: %f\r\nlat out:%f\r\n", pos.lla[0], result.lla[0]);
        printf("lon in: %f\r\nlon out:%f\r\n", pos.lla[1], result.lla[1]);
        printf("time in: %d\r\ntime out:%d\r\n", pos.timeOfWeekMs, result.timeOfWeekMs);
        printf("stat in: %d\r\nstat out:%d\r\n", pos.status, result.status);
    }

    ASSERT_EQ(comValue, 0);
}

TEST(protocol_nmea, GSA)
{
    gps_pos_t pos = {};
    pos.pDop = 6;
    pos.hAcc = 7;
    pos.vAcc = 8;

    gps_sat_t sat = {};
	for (uint32_t i = 0; i < 10; i++)
    {
        sat.sat[i].svId = i+1;
    }

    char abuf[ASCII_BUF_LEN] = { 0 };
    int n = nmea_gsa(abuf, ASCII_BUF_LEN, pos, sat);
    // printf("%s\n", abuf);
    gps_pos_t resultPos = {};
    gps_sat_t resultSat = {};
    nmea_parse_gsa(abuf, n, resultPos, &resultSat);
    ASSERT_EQ(memcmp(&pos, &resultPos, sizeof(resultPos)), 0);
    ASSERT_EQ(memcmp(&sat, &resultSat, sizeof(resultSat)), 0);
}

TEST(protocol_nmea, RMC)
{
    char rmc[ASCII_BUF_LEN] = { 0 };
    snprintf(rmc, ASCII_BUF_LEN, "$GNRMC,181457,A,4003.34252,N,11139.51903,W,000.0,000.0,010523,000.0,E*71\r\n");
    gps_pos_t pos = {};
    gps_vel_t vel = {};
    float magDeclination = 0.0f;
    pos.week = 2260;
    pos.timeOfWeekMs = 152115000;
    pos.satsUsed = 12;
    pos.status =
        GPS_STATUS_NUM_SATS_USED_MASK & pos.satsUsed |
        GPS_STATUS_FLAGS_FIX_OK |
        GPS_STATUS_FLAGS_DGPS_USED |
        GPS_STATUS_FIX_DGPS |
        GPS_STATUS_FLAGS_GPS_NMEA_DATA;
    pos.hMSL = 1438.2f;
    pos.lla[0] = (40.0 + 3.34252 / 60.0);
    pos.lla[1] = -(111.0 + 39.51903 / 60.0);
    pos.lla[2] = pos.hMSL - 18.8;
    pos.pDop = 0.47f;
    pos.leapS = LEAP_SEC;
    // Convert LLA to ECEF.  Ensure LLA uses ellipsoid altitude
    ixVector3d lla;
    lla[0] = DEG2RAD(pos.lla[0]);
    lla[1] = DEG2RAD(pos.lla[1]);
    lla[2] = pos.lla[2];		// Use ellipsoid altitude
    lla2ecef(lla, pos.ecef);

    // float courseMadeTrue  = 0.0f * C_DEG2RAD_F;
    // float speed2dKnots = 0.0f;

    char abuf[ASCII_BUF_LEN] = { 0 };
    int n = nmea_rmc(abuf, ASCII_BUF_LEN, pos, vel, magDeclination);

    int compVal = memcmp(&rmc, &abuf, n);

    if (compVal != 0)
    {
        printf("%s\n", rmc);
        printf("%s\n", abuf);
        ASSERT_EQ(compVal, 0);
    }
}

TEST(protocol_nmea, ZDA)
{
    gps_pos_t pos = {};
    pos.timeOfWeekMs = 423199200;
    pos.week = 2277;
    pos.leapS = C_GPS_LEAP_SECONDS;

    char abuf[ASCII_BUF_LEN] = { 0 };
    int n = nmea_zda(abuf, ASCII_BUF_LEN, pos);
    // printf("%s\n", abuf);
    uint32_t gpsTowMs;
    uint32_t gpsWeek;
    utc_date_t utcDate;
    utc_time_t utcTime;
    nmea_parse_zda(abuf, n, gpsTowMs, gpsWeek, utcDate, utcTime, pos.leapS);

    ASSERT_EQ(pos.timeOfWeekMs, gpsTowMs);
    ASSERT_EQ(pos.week, gpsWeek);
}

TEST(protocol_nmea, VTG)
{
    gps_pos_t pos = {};    
    pos.timeOfWeekMs = 423199200;
    pos.lla[0] = 40.19759002;
    pos.lla[1] = -111.62147172;
    pos.lla[2] = 1408.565264;

    ixQuat qe2n;
    float velNed[3] = { 0.0f, 4.0f, 0.0f };
    quat_ecef2ned(C_DEG2RAD_F*(float)pos.lla[0], C_DEG2RAD_F*(float)pos.lla[1], qe2n);
    gps_vel_t vel = {};
    // Velocity in ECEF
    quatRot(vel.vel, qe2n, velNed);
    
    float magVarCorrectionRad = 11.1f;

    char abuf[ASCII_BUF_LEN] = { 0 };
    int n = nmea_vtg(abuf, ASCII_BUF_LEN, pos, vel, magVarCorrectionRad);
    // printf("%s\n", abuf);
    gps_vel_t resultVel = {};

    nmea_parse_vtg(abuf, n, resultVel, pos.lla);

    for (int i=0; i<3; i++)
    {
        ASSERT_NEAR(vel.vel[i], resultVel.vel[i], 0.02f);
    }
}

TEST(protocol_nmea, INTEL)
{
    dev_info_t info = {};
    info.firmwareVer[0] = 1;
    info.firmwareVer[1] = 2;
    info.firmwareVer[2] = 3;
    info.firmwareVer[3] = 4;

    gps_pos_t pos = {};    
    pos.timeOfWeekMs = 423199200;
    pos.lla[0] = 40.19759002;
    pos.lla[1] = -111.62147172;
    pos.lla[2] = 1408.565264;

    ixQuat qe2n;
    float velNed[3] = { 0.0f, 4.0f, 0.0f };
    quat_ecef2ned(C_DEG2RAD_F*(float)pos.lla[0], C_DEG2RAD_F*(float)pos.lla[1], qe2n);
    gps_vel_t vel = {};
    // Velocity in ECEF
    quatRot(vel.vel, qe2n, velNed);
    
    char abuf[ASCII_BUF_LEN] = { 0 };
    int n = nmea_intel(abuf, ASCII_BUF_LEN, info, pos, vel);
    // printf("%s\n", abuf);
    dev_info_t resultInfo = {};
    gps_pos_t resultPos = {};
    gps_vel_t resultVel = {};
    float resultPpsPhase[2];
    uint32_t resultPpsNoiseNs[1];

    nmea_parse_intel(abuf, n, resultInfo, resultPos, resultVel, resultPpsPhase, resultPpsNoiseNs);

    for (int i=0; i<3; i++)
    {
        ASSERT_NEAR(vel.vel[i], resultVel.vel[i], 0.02f);
    }
    for (int i=0; i<4; i++)
    {
        ASSERT_EQ(info.firmwareVer[i], resultInfo.firmwareVer[i]);
    }
}

/**
 * @brief Test creation and parsing of the POWTLV message.
 */
TEST(protocol_nmea, POWTLV)
{
    gps_pos_t pos = {};    
    pos.timeOfWeekMs = 423199200;
    pos.week = 2361;
    pos.leapS = 18;
    pos.lla[0] = 40.19759002;
    pos.lla[1] = -111.62147172;
    pos.lla[2] = 1408.565264;
    pos.hMSL = 1438.2f;

    gps_vel_t vel = {};
    vel.vel[0] = 1.0;
    vel.vel[1] = 2.0;
    vel.vel[2] = 3.0;
    
    char abuf[ASCII_BUF_LEN] = { 0 };
    int n = nmea_powtlv(abuf, ASCII_BUF_LEN, pos, vel);

    // printf("%s\n", abuf);

    gps_pos_t resultPos = {};
    gps_vel_t resultVel = {};

    nmea_parse_powtlv(abuf, n, resultPos, resultVel);

    // Checks time valid bit set field 1
    ASSERT_EQ(abuf[8], '1');

    for (int i=0; i<3; i++)
    {
        // test field 13,14,15
        ASSERT_NEAR(vel.vel[i], resultVel.vel[i], 0.02f);
        // test field 7,8,9,10,11
        ASSERT_NEAR(pos.lla[i], resultPos.lla[i], 0.02f);
    }
    
    // test field 1,2
    ASSERT_EQ(pos.week, resultPos.week);

    // test field 1,3
    ASSERT_EQ(pos.timeOfWeekMs, resultPos.timeOfWeekMs);

    // test field 4,5
    ASSERT_EQ(pos.leapS, resultPos.leapS);

    // tests field 12
    ASSERT_EQ(pos.hMSL, resultPos.hMSL);
}

/**
 * @brief Test creation and parsing of the POWGPS message.
 */
TEST(protocol_nmea, POWGPS_valid)
{
    gps_pos_t pos = {};    
    pos.timeOfWeekMs = 423199200;
    pos.week = 2361;
    pos.leapS = 18;
    pos.lla[0] = 40.19759002;
    pos.lla[1] = -111.62147172;
    pos.lla[2] = 1408.565264;
    pos.hMSL = 1438.2f;
    
    char abuf[ASCII_BUF_LEN] = { 0 };
    int n = nmea_powgps(abuf, ASCII_BUF_LEN, pos);
    
    // printf("%s\n", abuf);

    gps_pos_t resultPos = {};

    nmea_parse_powgps(abuf, n, resultPos);

    // test field 1
    ASSERT_EQ(abuf[8], '1');

    // test field 1,2
    ASSERT_EQ(pos.week, resultPos.week);
    // test field 1,3
    ASSERT_EQ(pos.timeOfWeekMs, resultPos.timeOfWeekMs);

    // test field 4,5
    ASSERT_EQ(pos.leapS, resultPos.leapS);
}

/**
 * @brief Test creation and parsing of the POWGPS message with invalid time * 
 */
TEST(protocol_nmea, POWGPS_gps_time_invalid)
{
    gps_pos_t pos = {};    
    pos.timeOfWeekMs = 423199200;
    pos.week = 2270;
    pos.leapS = 18;
    pos.lla[0] = 40.19759002;
    pos.lla[1] = -111.62147172;
    pos.lla[2] = 1408.565264;
    pos.hMSL = 1438.2f;
    
    char abuf[ASCII_BUF_LEN] = { 0 };
    int n = nmea_powgps(abuf, ASCII_BUF_LEN, pos);

    // printf("%s\n", abuf);

    gps_pos_t resultPos = {};

    nmea_parse_powgps(abuf, n, resultPos);

    // test field 1
    ASSERT_EQ(abuf[8], '0');

    // test field 1,2
    ASSERT_EQ(0, resultPos.week);
    // test field 1,3
    ASSERT_EQ(0, resultPos.timeOfWeekMs);

    // test field 4,5
    ASSERT_EQ(pos.leapS, resultPos.leapS);
}

/**
 * @brief Test creation and parsing of the POWGPS message with invalid leap second
 */
TEST(protocol_nmea, POWGPS_leap_invalid)
{
    gps_pos_t pos = {};    
    pos.timeOfWeekMs = 423199200;
    pos.week = 2361;
    pos.leapS = 9;
    pos.lla[0] = 40.19759002;
    pos.lla[1] = -111.62147172;
    pos.lla[2] = 1408.565264;
    pos.hMSL = 1438.2f;
    
    char abuf[ASCII_BUF_LEN] = { 0 };
    int n = nmea_powgps(abuf, ASCII_BUF_LEN, pos);

    // printf("%s\n", abuf);

    gps_pos_t resultPos = {};

    nmea_parse_powgps(abuf, n, resultPos);

    // test field 1
    ASSERT_EQ(abuf[8], '1');

    // test field 1,2
    ASSERT_EQ(pos.week, resultPos.week);
    // test field 1,3
    ASSERT_EQ(pos.timeOfWeekMs, resultPos.timeOfWeekMs);
    // test field 4,5
    ASSERT_EQ(0, resultPos.leapS);
}

#define ASCII_BUF2  2048

void clear_GSV_values()
{
    extern void gsv_clear_const_mask();
    gsv_clear_const_mask();
}

TEST(protocol_nmea, GSV_binary_GSV)
{
    // set inital conditions
    clear_GSV_values();
    nmea_set_protocol_version(NMEA_PROTOCOL_2P3);

    string buf;

   buf = "$ASCE,0,GxGSV,1*44\r\n";

    rmci_t outRmci[NUM_COM_PORTS] = {};
    nmea_parse_asce(0, buf.c_str(), buf.size(), outRmci);

    // GPS & SBAS        #msgs,msg#,numSV,  svid,elv,azm,cno, ..., signalId*checksum
    buf =  "$GPGSV,6,1,23" ",02,40,310,43" ",08,07,324,31" ",10,48,267,45" ",15,37,053,45"      "*7C\r\n";
    buf += "$GPGSV,6,2,23" ",16,12,268,35" ",18,69,078,41" ",23,74,336,40" ",24,15,111,37"      "*79\r\n";
    buf += "$GPGSV,6,3,23" ",26,02,239,31" ",27,35,307,38" ",29,12,162,37" ",32,14,199,39"      "*7B\r\n";
    buf += "$GPGSV,6,4,23" ",44,43,188,43" ",46,40,206,43" ",522,48,267,45" ",527,37,053,26"    "*73\r\n";
    buf += "$GPGSV,6,5,23" ",530,69,078,34" ",535,74,336,34" ",536,15,111,25" ",538,02,239,18"  "*74\r\n";
    buf += "$GPGSV,6,6,23" ",539,35,307,27" ",541,12,162,21" ",544,14,199,25"                   "*73\r\n";
    // Galileo
    buf += "$GAGSV,2,1,08" ",05,65,144,41" ",09,39,052,43" ",34,71,341,42" ",36,46,105,39"      "*6A\r\n";
    buf += "$GAGSV,2,2,08" ",517,65,144,30" ",521,39,052,30" ",546,71,341,27" ",548,46,105,30"  "*64\r\n";
    // Beidou
    buf += "$GBGSV,3,1,10" ",11,09,141,34" ",14,52,047,44" ",27,32,313,43" ",28,80,263,44"      "*64\r\n";
    buf += "$GBGSV,3,2,10" ",33,81,039,43" ",41,43,230,42" ",43,33,148,42" ",58,,,44"           "*5B\r\n";
    buf += "$GBGSV,3,3,10" ",11,09,141,16" ",14,52,047,32"                                      "*60\r\n";
    // // QZSS
    buf += "$GQGSV,1,1,01" ",02,45,101,30"                                                      "*49\r\n";
    // // GLONASS
    buf += "$GLGSV,2,1,07" ",65,85,260,33" ",66,28,217,30" ",72,36,034,35" ",81,20,324,33"      "*69\r\n";
    buf += "$GLGSV,2,2,07" ",87,47,127,35" ",88,73,350,34" ",87,47,127,20"                      "*53\r\n";

    uint32_t cnoSum = 0, cnoCount = 0;
    gps_sat_t gpsSat = {};
    gps_sig_t gpsSig = {};
    for (char *ptr = (char*)(buf.c_str()); ptr < (buf.c_str() + buf.size()); )
    {
        ptr = nmea_parse_gsv(ptr, (int)buf.size(), &gpsSat, &gpsSig, &cnoSum, &cnoCount);
    }
    char abuf[ASCII_BUF2] = { 0 };
    int abuf_n = nmea_gsv(abuf, ASCII_BUF2, gpsSat, gpsSig);

    // cout << "Before (" << buf.size() << "):\n" << buf;
    // cout << "After (" << abuf_n << "):\n" << abuf;

    ASSERT_TRUE( abuf_n == buf.size() );
    ASSERT_TRUE( memcmp(abuf, buf.c_str(), abuf_n) == 0 );
}

TEST(protocol_nmea_4p11, GSV_binary_GSV)
{
    clear_GSV_values();

    nmea_set_protocol_version(NMEA_PROTOCOL_4P10);

    string buf = "$ASCE,0,GXGSV,1*64\r\n";

    rmci_t outRmci[NUM_COM_PORTS] = {};
    nmea_parse_asce(0, buf.c_str(), buf.size(), outRmci);

    // GPS & SBAS        #msgs,msg#,numSV,  svid,elv,azm,cno, ..., signalId*checksum
    buf = "$GPGSV,4,1,14" ",02,40,310,43" ",08,07,324,31" ",10,48,267,45" ",15,37,053,45"  ",1" "*67\r\n";
    buf += "$GPGSV,4,2,14" ",16,12,268,35" ",18,69,078,41" ",23,74,336,40" ",24,15,111,37"  ",1" "*62\r\n";
    buf += "$GPGSV,4,3,14" ",26,02,239,31" ",27,35,307,38" ",29,12,162,37" ",32,14,199,39"  ",1" "*60\r\n";
    buf += "$GPGSV,4,4,14" ",44,43,188,43" ",46,40,206,43"                                  ",1" "*65\r\n";
    buf += "$GPGSV,3,1,09" ",10,48,267,45" ",15,37,053,26" ",18,69,078,34" ",23,74,336,34"  ",6" "*68\r\n";
    buf += "$GPGSV,3,2,09" ",24,15,111,25" ",26,02,239,18" ",27,35,307,27" ",29,12,162,21"  ",6" "*64\r\n";
    buf += "$GPGSV,3,3,09" ",32,14,199,25"                                                  ",6" "*58\r\n";
    // Galileo
    buf += "$GAGSV,1,1,04" ",05,65,144,41" ",09,39,052,43" ",34,71,341,42" ",36,46,105,39"  ",7" "*7E\r\n";
    buf += "$GAGSV,1,1,04" ",05,65,144,30" ",09,39,052,30" ",34,71,341,27" ",36,46,105,30"  ",2" "*73\r\n";
    // Beidou
    buf += "$GBGSV,2,1,08" ",11,09,141,34" ",14,52,047,44" ",27,32,313,43" ",28,80,263,44"  ",1" "*71\r\n";
    buf += "$GBGSV,2,2,08" ",33,81,039,43" ",41,43,230,42" ",43,33,148,42" ",58,,,44"       ",1" "*4E\r\n";
    buf += "$GBGSV,1,1,02" ",11,09,141,16" ",14,52,047,32"                                  ",B" "*0D\r\n";
    // QZSS
    buf += "$GQGSV,1,1,01" ",02,45,101,30"                                                  ",1" "*54\r\n";
    // GLONASS
    buf += "$GLGSV,2,1,06" ",65,85,260,33" ",66,28,217,30" ",72,36,034,35" ",81,20,324,33"  ",1" "*75\r\n";
    buf += "$GLGSV,2,2,06" ",87,47,127,35" ",88,73,350,34"                                  ",1" "*75\r\n";
    buf += "$GLGSV,1,1,01" ",87,47,127,20"                                                  ",3" "*41\r\n";

    uint32_t cnoSum = 0, cnoCount = 0;
    gps_sat_t gpsSat = {};
    gps_sig_t gpsSig = {};
    char abuf[ASCII_BUF2] = { 0 };

    for (char *ptr = (char*)(buf.c_str()); ptr < (buf.c_str() + buf.size()); )
    {
        ptr = nmea_parse_gsv(ptr, (int)buf.size(), &gpsSat, &gpsSig, &cnoSum, &cnoCount);
    }
    int abuf_n = nmea_gsv(abuf, ASCII_BUF2, gpsSat, gpsSig);

    // cout << "Before (" << buf.size() << "):\n" << buf;
    // cout << "After (" << abuf_n << "):\n" << abuf;

    ASSERT_TRUE( abuf_n == buf.size() );
    ASSERT_TRUE( memcmp(abuf, buf.c_str(), abuf_n) == 0 );
}

TEST(protocol_nmea, binary_GSV_binary)
{
    gps_sat_t gpsSat = {};
    gps_sig_t gpsSig = {};

    init_sat_and_sig(&gpsSat, &gpsSig);
    clear_GSV_values();

    string buf = "$ASCE,0,15,1*3D\r\n";

    rmci_t outRmci[NUM_COM_PORTS] = {};
    nmea_parse_asce(0, buf.c_str(), buf.size(), outRmci);

    {   // Test NMEA protocol 2.3
        nmea_set_protocol_version(NMEA_PROTOCOL_2P3);
        char abuf[ASCII_BUF2] = { 0 };
        int abuf_n = nmea_gsv(abuf, ASCII_BUF2, gpsSat, gpsSig);

        gps_sat_t outSat = {};
        gps_sig_t outSig = {};
        uint32_t cnoSum = 0, cnoCount = 0;

        for (char *ptr = abuf; ptr < (abuf + abuf_n); )
        {
            ptr = nmea_parse_gsv(ptr, abuf_n, &outSat, &outSig, &cnoSum, &cnoCount);
        }

        // cout << "NMEA (" << abuf_n << "):\n" << abuf;

        ASSERT_TRUE( outSat.numSats == gpsSat.numSats );
        for (uint32_t i=0; i<outSat.numSats; i++)
        {
            gps_sat_sv_t &src = gpsSat.sat[i];
            gps_sat_sv_t &dst = outSat.sat[i];

            // printf("%d   gnss: %d %d,  svid: %d %d,  cno: %d %d,  ele: %d %d,  azm: %d %d\n", 
            //     i,
            //     src.gnssId, dst.gnssId, 
            //     src.svId, dst.svId, 
            //     src.cno, dst.cno, 
            //     src.elev, dst.elev, 
            //     src.azim, dst.azim);

            ASSERT_TRUE( dst.gnssId == src.gnssId );
            ASSERT_TRUE( dst.svId == src.svId );
            ASSERT_TRUE( dst.elev == src.elev );
            ASSERT_TRUE( dst.azim == src.azim );
            ASSERT_TRUE( dst.cno == src.cno );
        }

        ASSERT_TRUE( outSig.numSigs == gpsSig.numSigs );
        for (uint32_t i=0; i<outSig.numSigs; i++)
        {
            gps_sig_sv_t &src = gpsSig.sig[i];
            gps_sig_sv_t &dst = outSig.sig[i];

            // printf("%d   gnss: %d %d,  svid: %d %d,  sigId: %d %d,  quality: %d %d,  cno: %d %d\n", 
            //     i,
            //     src.gnssId, dst.gnssId, 
            //     src.svId, dst.svId, 
            //     src.sigId, dst.sigId, 
            //     src.quality, dst.quality,
            //     src.cno, dst.cno);

            ASSERT_TRUE( dst.gnssId == src.gnssId );
            ASSERT_TRUE( dst.svId == src.svId );
            ASSERT_TRUE( dst.sigId == src.sigId );
            ASSERT_TRUE( dst.quality == src.quality );
            ASSERT_TRUE( dst.cno == src.cno );
        }
    }

    {   // Test NMEA protocol 4.10
        nmea_set_protocol_version(NMEA_PROTOCOL_4P10);
        char abuf[ASCII_BUF2] = { 0 };
        int abuf_n = nmea_gsv(abuf, ASCII_BUF2, gpsSat, gpsSig);

        gps_sat_t outSat = {};
        gps_sig_t outSig = {};
        uint32_t cnoSum = 0, cnoCount = 0;

        for (char *ptr = abuf; ptr < (abuf + abuf_n); )
        {
            ptr = nmea_parse_gsv(ptr, abuf_n, &outSat, &outSig, &cnoSum, &cnoCount);
        }

        // cout << "NMEA (" << abuf_n << "):\n" << abuf;

        ASSERT_TRUE( outSat.numSats == gpsSat.numSats );
        for (uint32_t i=0; i<outSat.numSats; i++)
        {
            gps_sat_sv_t &src = gpsSat.sat[i];
            gps_sat_sv_t &dst = outSat.sat[i];
            ASSERT_TRUE( dst.gnssId == src.gnssId );
            ASSERT_TRUE( dst.svId == src.svId );
            ASSERT_TRUE( dst.elev == src.elev );
            ASSERT_TRUE( dst.azim == src.azim );
            ASSERT_TRUE( dst.cno == src.cno );
        }

        ASSERT_TRUE( outSig.numSigs == gpsSig.numSigs );
        for (uint32_t i=0; i<outSig.numSigs; i++)
        {
            gps_sig_sv_t &src = gpsSig.sig[i];
            gps_sig_sv_t &dst = outSig.sig[i];
            ASSERT_TRUE( dst.gnssId == src.gnssId );
            ASSERT_TRUE( dst.svId == src.svId );
            ASSERT_TRUE( dst.sigId == src.sigId );
            ASSERT_TRUE( dst.quality == src.quality );
            ASSERT_TRUE( dst.cno == src.cno );
        }
    }
}

TEST(protocol_nmea, GNGSV)
{
    gps_sat_t gpsSat = {};
    gps_sig_t gpsSig = {};

    init_sat_and_sig(&gpsSat, &gpsSig);
    clear_GSV_values();

    string buf = "$ASCE,0,GNGSV,1*72\r\n";

    rmci_t outRmci[NUM_COM_PORTS] = {};
    nmea_parse_asce(0, buf.c_str(), buf.size(), outRmci);

    {   // Test NMEA protocol 2.3
        nmea_set_protocol_version(NMEA_PROTOCOL_2P3);
        char abuf[ASCII_BUF2] = { 0 };
        int abuf_n = nmea_gsv(abuf, ASCII_BUF2, gpsSat, gpsSig);

        gps_sat_t outSat = {};
        gps_sig_t outSig = {};
        uint32_t cnoSum = 0, cnoCount = 0;

        for (char *ptr = abuf; ptr < (abuf + abuf_n); )
        {
            ptr = nmea_parse_gsv(ptr, abuf_n, &outSat, &outSig, &cnoSum, &cnoCount);
        }

        // cout << "NMEA (" << abuf_n << "):\n" << abuf;

        ASSERT_TRUE( outSat.numSats == gpsSat.numSats );
        for (uint32_t i=0; i<outSat.numSats; i++)
        {
            gps_sat_sv_t &src = gpsSat.sat[i];
            gps_sat_sv_t &dst = outSat.sat[i];
            // printf("%d   gnss: %d %d,  svid: %d %d,  cno: %d %d,  ele: %d %d,  azm: %d %d\n", 
            //     i,
            //     src.gnssId, dst.gnssId, 
            //     src.svId, dst.svId, 
            //     src.cno, dst.cno, 
            //     src.elev, dst.elev, 
            //     src.azim, dst.azim);
            ASSERT_TRUE( dst.gnssId == src.gnssId );
            ASSERT_TRUE( dst.svId == src.svId );
            ASSERT_TRUE( dst.elev == src.elev );
            ASSERT_TRUE( dst.azim == src.azim );
            ASSERT_TRUE( dst.cno == src.cno );
        }

        ASSERT_TRUE( outSig.numSigs == gpsSig.numSigs );
        for (uint32_t i=0; i<outSig.numSigs; i++)
        {
            gps_sig_sv_t &src = gpsSig.sig[i];
            gps_sig_sv_t &dst = outSig.sig[i];
            // printf("%d   gnss: %d %d,  svid: %d %d,  sigId: %d %d,  quality: %d %d,  cno: %d %d\n", 
            //     i,
            //     src.gnssId, dst.gnssId, 
            //     src.svId, dst.svId, 
            //     src.sigId, dst.sigId, 
            //     src.quality, dst.quality,
            //     src.cno, dst.cno
            // );
            ASSERT_TRUE( dst.gnssId == src.gnssId );
            ASSERT_TRUE( dst.svId == src.svId );
            ASSERT_TRUE( dst.sigId == src.sigId );
            ASSERT_TRUE( dst.quality == src.quality );
            ASSERT_TRUE( dst.cno == src.cno );
        }
    }

    {   // Test NMEA protocol 4.10
        nmea_set_protocol_version(NMEA_PROTOCOL_4P10);
        char abuf[ASCII_BUF2] = { 0 };
        int abuf_n = nmea_gsv(abuf, ASCII_BUF2, gpsSat, gpsSig);

        gps_sat_t outSat = {};
        gps_sig_t outSig = {};
        uint32_t cnoSum = 0, cnoCount = 0;

        for (char *ptr = abuf; ptr < (abuf + abuf_n); )
        {
            ptr = nmea_parse_gsv(ptr, abuf_n, &outSat, &outSig, &cnoSum, &cnoCount);
        }

        // cout << "NMEA (" << abuf_n << "):\n" << abuf;

        ASSERT_TRUE( outSat.numSats == gpsSat.numSats );
        for (uint32_t i=0; i<outSat.numSats; i++)
        {
            gps_sat_sv_t &src = gpsSat.sat[i];
            gps_sat_sv_t &dst = outSat.sat[i];
            ASSERT_TRUE( dst.gnssId == src.gnssId );
            ASSERT_TRUE( dst.svId == src.svId );
            ASSERT_TRUE( dst.elev == src.elev );
            ASSERT_TRUE( dst.azim == src.azim );
            ASSERT_TRUE( dst.cno == src.cno );
        }

        ASSERT_TRUE( outSig.numSigs == gpsSig.numSigs );
        for (uint32_t i=0; i<outSig.numSigs; i++)
        {
            gps_sig_sv_t &src = gpsSig.sig[i];
            gps_sig_sv_t &dst = outSig.sig[i];
            ASSERT_TRUE( dst.gnssId == src.gnssId );
            ASSERT_TRUE( dst.svId == src.svId );
            ASSERT_TRUE( dst.sigId == src.sigId );
            ASSERT_TRUE( dst.quality == src.quality );
            ASSERT_TRUE( dst.cno == src.cno );
        }
    }
}

TEST(protocol_nmea, GPGSV)
{
    gps_sat_t gpsSat = {};
    gps_sig_t gpsSig = {};

    init_sat_and_sig(&gpsSat, &gpsSig);
    clear_GSV_values();

    string buf = "$ASCE,0,GPGSV,1*6C\r\n";

    rmci_t outRmci[NUM_COM_PORTS] = {};
    nmea_parse_asce(0, buf.c_str(), buf.size(), outRmci);

    {   // Test NMEA protocol 2.3
        nmea_set_protocol_version(NMEA_PROTOCOL_2P3);
        char abuf[ASCII_BUF2] = { 0 };
        int abuf_n = nmea_gsv(abuf, ASCII_BUF2, gpsSat, gpsSig);

        gps_sat_t outSat = {};
        gps_sig_t outSig = {};
        uint32_t cnoSum = 0, cnoCount = 0;

        for (char* ptr = abuf; ptr < (abuf + abuf_n); )
        {
            ptr = nmea_parse_gsv(ptr, abuf_n, &outSat, &outSig, &cnoSum, &cnoCount);
        }

        // cout << "NMEA (" << abuf_n << "):\n" << abuf;

        ASSERT_TRUE(outSat.numSats == 12);

        for (uint32_t i = 0; i < outSat.numSats; i++)
        {
            ASSERT_TRUE(outSat.sat[i].gnssId == SAT_SV_GNSS_ID_GPS);
        }

        ASSERT_TRUE(outSig.numSigs == 19);

        for (uint32_t i = 0; i < outSig.numSigs; i++)
        {
            ASSERT_TRUE(outSig.sig[i].gnssId == SAT_SV_GNSS_ID_GPS);
        }
    }

    {   // Test NMEA protocol 4.10
        nmea_set_protocol_version(NMEA_PROTOCOL_4P10);
        char abuf[ASCII_BUF2] = { 0 };
        int abuf_n = nmea_gsv(abuf, ASCII_BUF2, gpsSat, gpsSig);

        gps_sat_t outSat = {};
        gps_sig_t outSig = {};
        uint32_t cnoSum = 0, cnoCount = 0;

        for (char* ptr = abuf; ptr < (abuf + abuf_n); )
        {
            ptr = nmea_parse_gsv(ptr, abuf_n, &outSat, &outSig, &cnoSum, &cnoCount);
        }

        ASSERT_TRUE(outSat.numSats == 12);

        for (uint32_t i = 0; i < outSat.numSats; i++)
        {
            ASSERT_TRUE(outSat.sat[i].gnssId == SAT_SV_GNSS_ID_GPS);
        }

        ASSERT_TRUE(outSig.numSigs == 19);

        for (uint32_t i = 0; i < outSig.numSigs; i++)
        {
            ASSERT_TRUE(outSig.sig[i].gnssId == SAT_SV_GNSS_ID_GPS);
        }
    }
}

TEST(protocol_nmea, GAGSV)
{
    gps_sat_t gpsSat = {};
    gps_sig_t gpsSig = {};

    init_sat_and_sig(&gpsSat, &gpsSig);
    clear_GSV_values();

    string buf = "$ASCE,0,GAGSV,1*7D\r\n";

    rmci_t outRmci[NUM_COM_PORTS] = {};
    nmea_parse_asce(0, buf.c_str(), buf.size(), outRmci);

    {   // Test NMEA protocol 2.3
        nmea_set_protocol_version(NMEA_PROTOCOL_2P3);
        char abuf[ASCII_BUF2] = { 0 };
        int abuf_n = nmea_gsv(abuf, ASCII_BUF2, gpsSat, gpsSig);

        gps_sat_t outSat = {};
        gps_sig_t outSig = {};
        uint32_t cnoSum = 0, cnoCount = 0;

        for (char* ptr = abuf; ptr < (abuf + abuf_n); )
        {
            ptr = nmea_parse_gsv(ptr, abuf_n, &outSat, &outSig, &cnoSum, &cnoCount);
        }

        // cout << "NMEA (" << abuf_n << "):\n" << abuf;

        ASSERT_TRUE(outSat.numSats == 4);

        for (uint32_t i = 0; i < outSat.numSats; i++)
        {
            ASSERT_TRUE(outSat.sat[i].gnssId == SAT_SV_GNSS_ID_GAL);
        }

        ASSERT_TRUE(outSig.numSigs == 8);

        for (uint32_t i = 0; i < outSig.numSigs; i++)
        {
            ASSERT_TRUE(outSig.sig[i].gnssId == SAT_SV_GNSS_ID_GAL);
        }
    }

    {   // Test NMEA protocol 4.10
        nmea_set_protocol_version(NMEA_PROTOCOL_4P10);
        char abuf[ASCII_BUF2] = { 0 };
        int abuf_n = nmea_gsv(abuf, ASCII_BUF2, gpsSat, gpsSig);

        gps_sat_t outSat = {};
        gps_sig_t outSig = {};
        uint32_t cnoSum = 0, cnoCount = 0;

        for (char* ptr = abuf; ptr < (abuf + abuf_n); )
        {
            ptr = nmea_parse_gsv(ptr, abuf_n, &outSat, &outSig, &cnoSum, &cnoCount);
        }

        ASSERT_TRUE(outSat.numSats == 4);

        for (uint32_t i = 0; i < outSat.numSats; i++)
        {
            ASSERT_TRUE(outSat.sat[i].gnssId == SAT_SV_GNSS_ID_GAL);
        }

        ASSERT_TRUE(outSig.numSigs == 8);

        for (uint32_t i = 0; i < outSig.numSigs; i++)
        {
            ASSERT_TRUE(outSig.sig[i].gnssId == SAT_SV_GNSS_ID_GAL);
        }
    }
}

TEST(protocol_nmea, GBGSV)
{
    gps_sat_t gpsSat = {};
    gps_sig_t gpsSig = {};

    init_sat_and_sig(&gpsSat, &gpsSig);
    clear_GSV_values();

    string buf = "$ASCE,0,GBGSV,1*7E\r\n";

    rmci_t outRmci[NUM_COM_PORTS] = {};
    nmea_parse_asce(0, buf.c_str(), buf.size(), outRmci);

    {   // Test NMEA protocol 2.3
        nmea_set_protocol_version(NMEA_PROTOCOL_2P3);
        char abuf[ASCII_BUF2] = { 0 };
        int abuf_n = nmea_gsv(abuf, ASCII_BUF2, gpsSat, gpsSig);

        gps_sat_t outSat = {};
        gps_sig_t outSig = {};
        uint32_t cnoSum = 0, cnoCount = 0;

        for (char* ptr = abuf; ptr < (abuf + abuf_n); )
        {
            ptr = nmea_parse_gsv(ptr, abuf_n, &outSat, &outSig, &cnoSum, &cnoCount);
        }

        // cout << "NMEA (" << abuf_n << "):\n" << abuf;

        ASSERT_TRUE(outSat.numSats == 8);

        for (uint32_t i = 0; i < outSat.numSats; i++)
        {
            ASSERT_TRUE(outSat.sat[i].gnssId == SAT_SV_GNSS_ID_BEI);
        }

        ASSERT_TRUE(outSig.numSigs == 10);

        for (uint32_t i = 0; i < outSig.numSigs; i++)
        {
            ASSERT_TRUE(outSig.sig[i].gnssId == SAT_SV_GNSS_ID_BEI);
        }
    }

    {   // Test NMEA protocol 4.10
        nmea_set_protocol_version(NMEA_PROTOCOL_4P10);
        char abuf[ASCII_BUF2] = { 0 };
        int abuf_n = nmea_gsv(abuf, ASCII_BUF2, gpsSat, gpsSig);

        gps_sat_t outSat = {};
        gps_sig_t outSig = {};
        uint32_t cnoSum = 0, cnoCount = 0;

        for (char* ptr = abuf; ptr < (abuf + abuf_n); )
        {
            ptr = nmea_parse_gsv(ptr, abuf_n, &outSat, &outSig, &cnoSum, &cnoCount);
        }

        ASSERT_TRUE(outSat.numSats == 8);

        for (uint32_t i = 0; i < outSat.numSats; i++)
        {
            ASSERT_TRUE(outSat.sat[i].gnssId == SAT_SV_GNSS_ID_BEI);
        }

        ASSERT_TRUE(outSig.numSigs == 10);

        for (uint32_t i = 0; i < outSig.numSigs; i++)
        {
            ASSERT_TRUE(outSig.sig[i].gnssId == SAT_SV_GNSS_ID_BEI);
        }
    }
}

TEST(protocol_nmea, GLGSV)
{
    gps_sat_t gpsSat = {};
    gps_sig_t gpsSig = {};

    init_sat_and_sig(&gpsSat, &gpsSig);
    clear_GSV_values();

    string buf = "$ASCE,0,GLGSV,1*70\r\n";

    rmci_t outRmci[NUM_COM_PORTS] = {};
    nmea_parse_asce(0, buf.c_str(), buf.size(), outRmci);

    {   // Test NMEA protocol 2.3
        nmea_set_protocol_version(NMEA_PROTOCOL_2P3);
        char abuf[ASCII_BUF2] = { 0 };
        int abuf_n = nmea_gsv(abuf, ASCII_BUF2, gpsSat, gpsSig);

        gps_sat_t outSat = {};
        gps_sig_t outSig = {};
        uint32_t cnoSum = 0, cnoCount = 0;

        for (char* ptr = abuf; ptr < (abuf + abuf_n); )
        {
            ptr = nmea_parse_gsv(ptr, abuf_n, &outSat, &outSig, &cnoSum, &cnoCount);
        }

        // cout << "NMEA (" << abuf_n << "):\n" << abuf;

        ASSERT_TRUE(outSat.numSats == 6);

        for (uint32_t i = 0; i < outSat.numSats; i++)
        {
            ASSERT_TRUE(outSat.sat[i].gnssId == SAT_SV_GNSS_ID_GLO);
        }

        ASSERT_TRUE(outSig.numSigs == 6);

        for (uint32_t i = 0; i < outSig.numSigs; i++)
        {
            ASSERT_TRUE(outSig.sig[i].gnssId == SAT_SV_GNSS_ID_GLO);
        }
    }

    {   // Test NMEA protocol 4.10
        nmea_set_protocol_version(NMEA_PROTOCOL_4P10);
        char abuf[ASCII_BUF2] = { 0 };
        int abuf_n = nmea_gsv(abuf, ASCII_BUF2, gpsSat, gpsSig);

        gps_sat_t outSat = {};
        gps_sig_t outSig = {};
        uint32_t cnoSum = 0, cnoCount = 0;

        for (char* ptr = abuf; ptr < (abuf + abuf_n); )
        {
            ptr = nmea_parse_gsv(ptr, abuf_n, &outSat, &outSig, &cnoSum, &cnoCount);
        }

        ASSERT_TRUE(outSat.numSats == 6);

        for (uint32_t i = 0; i < outSat.numSats; i++)
        {
            ASSERT_TRUE(outSat.sat[i].gnssId == SAT_SV_GNSS_ID_GLO);
        }

        ASSERT_TRUE(outSig.numSigs == 6);

        for (uint32_t i = 0; i < outSig.numSigs; i++)
        {
            ASSERT_TRUE(outSig.sig[i].gnssId == SAT_SV_GNSS_ID_GLO);
        }
    }
}

#if 0   // Uncomment to generate example NMEA strings for user manual documentation. 
TEST(protocol_nmea, generate_example_nmea_for_user_manual)
{
    uint32_t options = RMC_OPTIONS_PRESERVE_CTRL | RMC_OPTIONS_PERSISTENT;

    for (int id=0; id<=NMEA_MSG_ID_GNGSV; id++)
    {
        char a[ASCII_BUF_LEN] = {};
        int n=0;
        nmea_sprint(a, ASCII_BUF_LEN, n, "$ASCE,%u", 0);
        nmea_sprint(a, ASCII_BUF_LEN, n, ",%u,%u", id, 1);
        nmea_sprint_footer(a, ASCII_BUF_LEN, n);
        a[n] = 0;
        printf("%s", a);
    }
}
#endif


#if 0   // Uncomment to generate example NMEA strings for select customer. 
TEST(protocol_nmea, generate_example_nmea_for_customer)
{
    uint32_t options = RMC_OPTIONS_PRESERVE_CTRL | RMC_OPTIONS_PERSISTENT;

    char a[ASCII_BUF_LEN] = {};
    int n=0;
    nmea_sprint(a, ASCII_BUF_LEN, n, "$ASCE,%u", 0);
    nmea_sprint(a, ASCII_BUF_LEN, n, ",%u,%u", NMEA_MSG_ID_GNGGA, 1);
    nmea_sprint(a, ASCII_BUF_LEN, n, ",%u,%u", NMEA_MSG_ID_GNGLL, 1);
    nmea_sprint(a, ASCII_BUF_LEN, n, ",%u,%u", NMEA_MSG_ID_GNGSA, 1);
    nmea_sprint(a, ASCII_BUF_LEN, n, ",%u,%u", NMEA_MSG_ID_GNZDA, 1);
    nmea_sprint(a, ASCII_BUF_LEN, n, ",%u,%u", NMEA_MSG_ID_GNGSV, 1);
    nmea_sprint_footer(a, ASCII_BUF_LEN, n);
    a[n] = 0;
    printf("%s", a);
}
#endif


#if 1
void testChecksum(const char* str)
{
    char a[200] = {};
    char b[200] = {};
    int m = snprintf(a, sizeof(a), "%s", str);
    int n = m-5;
    memcpy(b, a, n);
    nmea_sprint_footer(b, sizeof(b), n);
    DEBUG_PRINTF("%s", a);
    DEBUG_PRINTF("%s", b);
    ASSERT_EQ(m, n);     // Check that 5 characters (*xx\r\n) were added
    ASSERT_TRUE(memcmp(a, b, m) == 0);
}

TEST(protocol_nmea, checksum)
{
    testChecksum("$GPGSV,3,3,12,522,10,279,46,530,67,310,54,535,42,295,50,521,047,46,04,21,047,40,261,49,224,47,05,49,224,42*54\r\n");
    testChecksum("$GAGSV,3,2,12,267715.76121,W,1,11,1.10,0.00,M,169.55,M,,*2D\r\n");  // NEMA Wrong Checksum 2d lenREc=59
    testChecksum("$GNVTG,0.00,T,,M,0.0.000*57\r\n");  // NEMA Wrong Checksum 57 lenREc=27
    testChecksum("$GPGSV,3,3,12,522,10,279,46,530,67,310,54,535,42,295,50,52,11,32,140,37*61\r\n");   // NEMA Wrong Checksum 61 lenREc=74
    testChecksum("$GAGSV,3,3,12,290,39,316,47,34,39,316,42,292,84,16305,10,13,15,18,23,24,29,04,05,09,11,1.1,1.0,1.0*5E\r\n");    // NEMA Wrong Checksum 5e lenREc=101
    testChecksum("$GNRMC,160350,A,3911.2480.26,+2.50,+0.00,0.009,0.009,2.676,1,0*38\r\n");    // NEMA Wrong Checksum 38 lenREc=65
    testChecksum("$GPGSV,3,3,12,522,10,279,46,530,67,310,54,535,42,295,50,5,12,260,21,047,46,04,21,047,40,261,49,224,47,05,49,224,42*4F\r\n");    // NEMA Wrong Checksum 4f lenREc=117
    testChecksum("$GAGSV,34843,N,07715.76121,W,1,11,1.10,0.00,M,169.55,M,,*41\r\n");  // NEMA Wrong Checksum 41 lenREc=59
    testChecksum("$GNGSA,A,03,05,00.0,E*6C\r\n"); // NEMA Wrong Checksum 6c lenREc=24
    testChecksum("$GNVTG,0.00,0.000,0.000*7E\r\n");   // NEMA Wrong Checksum 7e lenREc=26
    testChecksum("$GAGSV,3,1,12,260,21,047,46,04,21,047,40,261,49,224,43,48,36,84,163,44*49\r\n");    // NEMA Wrong Checksum 49 lenREc=73
    testChecksum("$GNGGA,160351.000,3911.24843,N,07715.76121,W,1,11,1.3911.24843,N,07715.76121,W,000.0,000.0,180923,000.0,E*12\r\n"); // NEMA Wrong Checksum 12 lenREc=108
    testChecksum("$GNZDA,160351.00000,2280,0,0.000,0.000,0,0.000,0.000,0.000,0.000,0.000,0.000*40\r\n");  // NEMA Wrong Checksum 40 lenREc=79
    testChecksum("$GPGSV,3,3,12,522,10,279,46,530,67,310,54,535,42,295,50,5,12,260,21,047,46,04,21,047,40,261,49,224,47,05,49,224,42*4F\r\n");    // NEMA Wrong Checksum 4f lenREc=117
    testChecksum("$GAGSV,34843,N,07715.76121,W,1,11,1.10,0.00,M,169.55,M,,*41\r\n");  // NEMA Wrong Checksum 41 lenREc=59
    testChecksum("$GNGSA,A,03,05,00.0,E*6C\r\n"); // NEMA Wrong Checksum 6c lenREc=24
    testChecksum("$GNVTG,0.00,0.000,0.000*7E\r\n");   // NEMA Wrong Checksum 7e lenREc=26
    testChecksum("$GAGSV,3,1,12,260,21,047,46,04,21,047,40,261,49,224,43,48,36,84,163,44*49\r\n");    // NEMA Wrong Checksum 49 lenREc=73
    testChecksum("$GNGGA,160352.000,3911.24843,N,07715.76121,W,1,11,1.911.24843,N,07715.76121,W,000.0,000.0,180923,000.0,E*22\r\n");  // NEMA Wrong Checksum 22 lenREc=107
    testChecksum("$GNZDA,160352.000,00,2280,0,0.000,0.000,0,0.000,0.000,0.000,0.000,0.000,0.000*6F\r\n"); // NEMA Wrong Checksum 6f lenREc=80
}
#endif

void init_sat_and_sig(gps_sat_t* gpsSat, gps_sig_t* gpsSig)
{
    // Satellite data array
    static const gps_sat_sv_t sat_data[] = 
    {
        {SAT_SV_GNSS_ID_GPS, 2, 40, 310, 43, 95},
        {SAT_SV_GNSS_ID_GPS, 8, 7, 324, 31, 95},
        {SAT_SV_GNSS_ID_GPS, 10, 48, 267, 45, 31},
        {SAT_SV_GNSS_ID_GPS, 15, 37, 53, 45, 31},
        {SAT_SV_GNSS_ID_GPS, 16, 12, 268, 37, 95},
        {SAT_SV_GNSS_ID_GPS, 18, 69, 78, 41, 31},
        {SAT_SV_GNSS_ID_GPS, 23, 74, 336, 41, 31},
        {SAT_SV_GNSS_ID_GPS, 24, 15, 111, 37, 31},
        {SAT_SV_GNSS_ID_GPS, 26, 2, 239, 31, 31},
        {SAT_SV_GNSS_ID_GPS, 27, 35, 307, 41, 95},
        {SAT_SV_GNSS_ID_GPS, 29, 12, 162, 36, 31},
        {SAT_SV_GNSS_ID_GPS, 32, 14, 199, 40, 95},
        {SAT_SV_GNSS_ID_SBS, 131, 43, 188, 43, 95},
        {SAT_SV_GNSS_ID_SBS, 133, 40, 206, 43, 95},
        {SAT_SV_GNSS_ID_SBS, 138, 43, 173, 35, 95},
        {SAT_SV_GNSS_ID_GAL, 5, 65, 145, 41, 31},
        {SAT_SV_GNSS_ID_GAL, 9, 39, 53, 43, 31},
        {SAT_SV_GNSS_ID_GAL, 34, 71, 340, 42, 31},
        {SAT_SV_GNSS_ID_GAL, 36, 47, 104, 40, 31},
        {SAT_SV_GNSS_ID_BEI, 11, 9, 141, 36, 23},
        {SAT_SV_GNSS_ID_BEI, 14, 52, 47, 44, 31},
        {SAT_SV_GNSS_ID_BEI, 27, 31, 313, 38, 31},
        {SAT_SV_GNSS_ID_BEI, 28, 79, 267, 44, 31},
        {SAT_SV_GNSS_ID_BEI, 33, 82, 40, 42, 31},
        {SAT_SV_GNSS_ID_BEI, 41, 43, 230, 43, 31},
        {SAT_SV_GNSS_ID_BEI, 43, 34, 148, 43, 31},
        {SAT_SV_GNSS_ID_BEI, 58, 0, 0, 43, 39},
        {SAT_SV_GNSS_ID_GLO, 1, 85, 252, 31, 23},
        {SAT_SV_GNSS_ID_GLO, 2, 28, 217, 29, 31},
        {SAT_SV_GNSS_ID_GLO, 8, 37, 34, 35, 31},
        {SAT_SV_GNSS_ID_GLO, 17, 19, 324, 19, 19},
        {SAT_SV_GNSS_ID_GLO, 23, 48, 126, 36, 31},
        {SAT_SV_GNSS_ID_GLO, 24, 72, 349, 32, 28}
    };

    // Signal data array
    static const gps_sig_sv_t sig_data[] = 
    {
        {SAT_SV_GNSS_ID_GPS, 2, SAT_SV_SIG_ID_GPS_L1CA, 43, 7, 361},
        {SAT_SV_GNSS_ID_GPS, 8, SAT_SV_SIG_ID_GPS_L1CA, 31, 7, 41},
        {SAT_SV_GNSS_ID_GPS, 10, SAT_SV_SIG_ID_GPS_L1CA, 45, 7, 41},
        {SAT_SV_GNSS_ID_GPS, 15, SAT_SV_SIG_ID_GPS_L1CA, 45, 7, 41},
        {SAT_SV_GNSS_ID_GPS, 16, SAT_SV_SIG_ID_GPS_L1CA, 37, 7, 361},
        {SAT_SV_GNSS_ID_GPS, 18, SAT_SV_SIG_ID_GPS_L1CA, 41, 7, 41},
        {SAT_SV_GNSS_ID_GPS, 23, SAT_SV_SIG_ID_GPS_L1CA, 41, 7, 41},
        {SAT_SV_GNSS_ID_GPS, 24, SAT_SV_SIG_ID_GPS_L1CA, 37, 7, 41},
        {SAT_SV_GNSS_ID_GPS, 26, SAT_SV_SIG_ID_GPS_L1CA, 31, 7, 41},
        {SAT_SV_GNSS_ID_GPS, 27, SAT_SV_SIG_ID_GPS_L1CA, 41, 7, 361},
        {SAT_SV_GNSS_ID_GPS, 29, SAT_SV_SIG_ID_GPS_L1CA, 36, 7, 41},
        {SAT_SV_GNSS_ID_GPS, 32, SAT_SV_SIG_ID_GPS_L1CA, 40, 7, 361},
        {2, 131, 0, 43, 7, 361}, 
        {2, 133, 0, 43, 7, 361}, 
        {2, 138, 0, 35, 7, 361},
        {SAT_SV_GNSS_ID_GPS, 10, SAT_SV_SIG_ID_GPS_L2CL, 45, 7, 41},
        {SAT_SV_GNSS_ID_GPS, 15, SAT_SV_SIG_ID_GPS_L2CL, 27, 7, 41},
        {SAT_SV_GNSS_ID_GPS, 18, SAT_SV_SIG_ID_GPS_L2CL, 33, 7, 41},
        {SAT_SV_GNSS_ID_GPS, 23, SAT_SV_SIG_ID_GPS_L2CL, 34, 7, 41},
        {SAT_SV_GNSS_ID_GPS, 27, SAT_SV_SIG_ID_GPS_L2CL, 23, 7, 9},
        {SAT_SV_GNSS_ID_GPS, 29, SAT_SV_SIG_ID_GPS_L2CL, 25, 7, 41},
        {SAT_SV_GNSS_ID_GPS, 32, SAT_SV_SIG_ID_GPS_L5, 28, 7, 41},
        {SAT_SV_GNSS_ID_GAL, 5, SAT_SV_SIG_ID_Galileo_E1C2, 41, 7, 41},
        {SAT_SV_GNSS_ID_GAL, 9, SAT_SV_SIG_ID_Galileo_E1C2, 43, 7, 41},
        {SAT_SV_GNSS_ID_GAL, 34, SAT_SV_SIG_ID_Galileo_E1C2, 42, 7, 41},
        {SAT_SV_GNSS_ID_GAL, 36, SAT_SV_SIG_ID_Galileo_E1C2, 40, 7, 41},
        {SAT_SV_GNSS_ID_GAL, 5, 6, 30, 7, 41}, 
        {SAT_SV_GNSS_ID_GAL, 9, 6, 30, 7, 41},
        {SAT_SV_GNSS_ID_GAL, 34, 6, 26, 7, 41}, 
        {SAT_SV_GNSS_ID_GAL, 36, 6, 31, 7, 41},
        {SAT_SV_GNSS_ID_BEI, 11, SAT_SV_SIG_ID_BeiDou_B1D1, 36, 7, 1},
        {SAT_SV_GNSS_ID_BEI, 14, SAT_SV_SIG_ID_BeiDou_B1D1, 44, 7, 41},
        {SAT_SV_GNSS_ID_BEI, 27, SAT_SV_SIG_ID_BeiDou_B1D1, 38, 7, 41},
        {SAT_SV_GNSS_ID_BEI, 28, SAT_SV_SIG_ID_BeiDou_B1D1, 44, 7, 41},
        {SAT_SV_GNSS_ID_BEI, 33, SAT_SV_SIG_ID_BeiDou_B1D1, 42, 7, 41},
        {SAT_SV_GNSS_ID_BEI, 41, SAT_SV_SIG_ID_BeiDou_B1D1, 43, 7, 41},
        {SAT_SV_GNSS_ID_BEI, 43, SAT_SV_SIG_ID_BeiDou_B1D1, 43, 7, 41},
        {SAT_SV_GNSS_ID_BEI, 58, SAT_SV_SIG_ID_BeiDou_B1D1, 43, 7, 2},
        {SAT_SV_GNSS_ID_BEI, 11, 2, 21, 7, 1}, 
        {SAT_SV_GNSS_ID_BEI, 14, 2, 30, 7, 41},
        {SAT_SV_GNSS_ID_GLO, 1, SAT_SV_SIG_ID_GLONASS_L1OF, 31, 7, 1},
        {SAT_SV_GNSS_ID_GLO, 2, SAT_SV_SIG_ID_GLONASS_L1OF, 29, 7, 41},
        {SAT_SV_GNSS_ID_GLO, 8, SAT_SV_SIG_ID_GLONASS_L1OF, 35, 7, 41},
        {SAT_SV_GNSS_ID_GLO, 17, SAT_SV_SIG_ID_GLONASS_L1OF, 19, 7, 1},
        {SAT_SV_GNSS_ID_GLO, 23, SAT_SV_SIG_ID_GLONASS_L1OF, 36, 7, 41},
        {SAT_SV_GNSS_ID_GLO, 24, SAT_SV_SIG_ID_GLONASS_L1OF, 32, 7, 41}
    };

    // Initialize gpsSat
    gpsSat->timeOfWeekMs = 436693200;
    gpsSat->numSats = sizeof(sat_data)/sizeof(gps_sat_sv_t);
    for (size_t i = 0; i < gpsSat->numSats; i++) 
    {
        gpsSat->sat[i] = sat_data[i];
    }

    // Initialize gpsSig
    gpsSig->timeOfWeekMs = 436693200;
    gpsSig->numSigs = sizeof(sig_data)/sizeof(gps_sig_sv_t);
    for (size_t i = 0; i < gpsSig->numSigs; i++) 
    {
        gpsSig->sig[i] = sig_data[i];
    }

    // Array bounds checks
    ASSERT_TRUE(gpsSat->numSats <= MAX_NUM_SATELLITES);
    ASSERT_TRUE(gpsSig->numSigs <= MAX_NUM_SAT_SIGNALS);
}


