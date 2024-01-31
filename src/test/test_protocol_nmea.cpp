#include <gtest/gtest.h>
#include <vector>
#include "../protocol_nmea.h"
#include "../ISEarth.h"
using namespace std;

#define PRINT_TEST_DESCRIPTION(description)   { cout << "TEST DESCRIPTION: " << description << "\n"; }

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

TEST(protocol_nmea, nmea_parse_ascb)
{
	PRINT_TEST_DESCRIPTION("Tests the $ASCB parser function nmea_parse_ascb().");

    rmci_t rmci[NUM_COM_PORTS] = {};
    int port = 1;
    rmci_t &r = rmci[port];
    r.rmcNmea.nmeaPeriod[NMEA_MSG_ID_PINS2] = '1';
    r.rmcNmea.nmeaPeriod[NMEA_MSG_ID_PIMU] = '2';
    r.rmcNmea.nmeaPeriod[NMEA_MSG_ID_GxGGA] = '3';
    r.rmcNmea.nmeaBits = 
        NMEA_RMC_BITS_PINS2 |
        NMEA_RMC_BITS_PIMU |
        NMEA_RMC_BITS_GxGGA;
    uint32_t options = RMC_OPTIONS_PRESERVE_CTRL | RMC_OPTIONS_PERSISTENT;

    char a[ASCII_BUF_LEN] = {};
    int n=0;
    nmea_sprint(a, ASCII_BUF_LEN, n, "$ASCB,%u,%u,,,%u,,,%u,", 
        options, 
        r.rmcNmea.nmeaPeriod[NMEA_MSG_ID_PIMU],
        r.rmcNmea.nmeaPeriod[NMEA_MSG_ID_PINS2],
        r.rmcNmea.nmeaPeriod[NMEA_MSG_ID_GxGGA]
        );
	nmea_sprint_footer(a, ASCII_BUF_LEN, n);

    rmci_t outRmci[NUM_COM_PORTS] = {};
    uint32_t outOptions = nmea_parse_ascb(port, a, n, outRmci);

    ASSERT_EQ( options, outOptions );
    for (int i=0; i<NUM_COM_PORTS; i++)
    {
        rmci_t &a = rmci[i];
        rmci_t &b = outRmci[i];
        ASSERT_EQ( a.rmc.bits, b.rmc.bits );
        ASSERT_EQ( a.rmcNmea.nmeaBits, b.rmcNmea.nmeaBits );
        //cout << "I: " << i << " a: " << a.rmcNmea.nmeaBits << " b: " <<  b.rmcNmea.nmeaBits << "\n"; 
        for (int j=0; j<NMEA_MSG_ID_COUNT; j++)
        {
            // cout << "J: " << j << " a: " << a.rmcNmea.nmeaPeriod[j] << " b: " <<  b.rmcNmea.nmeaPeriod[j] << "\n"; 
            ASSERT_EQ( a.rmcNmea.nmeaPeriod[j], b.rmcNmea.nmeaPeriod[j] );
        }    
    }
}

TEST(protocol_nmea, nmea_parse_asce)
{
	PRINT_TEST_DESCRIPTION("Tests the $ASCE parser function nmea_parse_asce().");

    rmci_t rmci[NUM_COM_PORTS] = {};
    int port = 1;
    rmci_t &r = rmci[port];
    r.rmcNmea.nmeaPeriod[NMEA_MSG_ID_PINS2] = '2';
    r.rmcNmea.nmeaPeriod[NMEA_MSG_ID_PIMU] = '1';
    r.rmcNmea.nmeaPeriod[NMEA_MSG_ID_GxGGA] = '0';
    r.rmcNmea.nmeaBits = 
        NMEA_RMC_BITS_PINS2 |
        NMEA_RMC_BITS_PIMU |
        NMEA_RMC_BITS_GxGGA;
    uint32_t options = RMC_OPTIONS_PRESERVE_CTRL | RMC_OPTIONS_PERSISTENT;

    char a[ASCII_BUF_LEN] = {};
    int n=0;
	nmea_sprint(a, ASCII_BUF_LEN, n, "$ASCE,%u", options);
    nmea_sprint(a, ASCII_BUF_LEN, n, ",%u,%u", NMEA_MSG_ID_PINS2, r.rmcNmea.nmeaPeriod[NMEA_MSG_ID_PINS2]);
    nmea_sprint(a, ASCII_BUF_LEN, n, ",%u,%u", NMEA_MSG_ID_PIMU,  r.rmcNmea.nmeaPeriod[NMEA_MSG_ID_PIMU]);
    nmea_sprint(a, ASCII_BUF_LEN, n, ",%u,%u", NMEA_MSG_ID_GxGGA, r.rmcNmea.nmeaPeriod[NMEA_MSG_ID_GxGGA]);
	nmea_sprint_footer(a, ASCII_BUF_LEN, n);

    rmci_t outRmci[NUM_COM_PORTS] = {};
    uint32_t outOptions = nmea_parse_asce(port, a, n, outRmci);

    ASSERT_EQ( options, outOptions );
    for (int i=0; i<NUM_COM_PORTS; i++)
    {
        rmci_t &a = rmci[i];
        rmci_t &b = outRmci[i];
        ASSERT_EQ( a.rmc.bits, b.rmc.bits );
         
        // cout << "I: " << i << " a: " << a.rmcNmea.nmeaBits << " b: " <<  b.rmcNmea.nmeaBits << "\n"; 
        ASSERT_EQ( a.rmcNmea.nmeaBits, b.rmcNmea.nmeaBits );
        for (int j=0; j < NMEA_MSG_ID_COUNT; j++)
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
    for (int i=0; i<4; i++)
    {
        info.buildDate[i] = i;
        info.buildTime[i] = i+4;
    }
    info.buildDate[0] = 'r';    // Build type
    snprintf(info.addInfo, DEVINFO_ADDINFO_STRLEN, "additional string   123");

    char abuf[ASCII_BUF_LEN] = { 0 };
    nmea_dev_info(abuf, ASCII_BUF_LEN, info);
    printf("%s\n", abuf);
    dev_info_t result = {};
    nmea_parse_info(result, abuf, ASCII_BUF_LEN);
    ASSERT_EQ(memcmp(&info, &result, sizeof(result)), 0);
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
    pos.week = 1;
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
    ASSERT_EQ(memcmp(&pos, &resultPos, sizeof(resultPos)), 0);
    ASSERT_EQ(memcmp(&vel, &resultVel, sizeof(resultVel)), 0);
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

    gps_pos_t result = {};
    result.week = pos.week;
    result.leapS = pos.leapS;
    uint32_t weekday = pos.timeOfWeekMs / 86400000;
    nmea_parse_gga_to_did_gps(result, abuf, ASCII_BUF_LEN, weekday);
    pos.hAcc = result.hAcc;
    ASSERT_EQ(memcmp(&pos, &result, sizeof(result)), 0);
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
    uint32_t weekday = pos.timeOfWeekMs / 86400000;
    nmea_parse_gga_to_did_gps(result, abuf, ASCII_BUF_LEN, weekday);
    pos.hAcc = result.hAcc;
    ASSERT_EQ(memcmp(&pos, &result, sizeof(result)), 0);
}

TEST(protocol_nmea, GLL)
{
    gps_pos_t pos = {};
    // pos.week = 12;
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
    uint32_t weekday = pos.timeOfWeekMs / 86400000;
    nmea_parse_gll_to_did_gps(result, abuf, ASCII_BUF_LEN, weekday);
    ASSERT_EQ(memcmp(&pos, &result, sizeof(result)), 0);
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
    nmea_parse_gsa_to_did_gps(resultPos, resultSat, abuf, n);
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
    printf("%s\n", rmc);
    printf("%s\n", abuf);
    ASSERT_EQ(memcmp(&rmc, &abuf, n), 0);
}

TEST(protocol_nmea, ZDA)
{
    gps_pos_t pos = {};
    pos.timeOfWeekMs = 423199200;
    pos.week = 2277;
    pos.leapS = 18;

    char abuf[ASCII_BUF_LEN] = { 0 };
    int n = nmea_zda(abuf, ASCII_BUF_LEN, pos);
    // printf("%s\n", abuf);
    gps_pos_t resultPos = {};
    nmea_parse_zda_to_did_gps(resultPos, abuf, n, pos.leapS);

    ASSERT_EQ(memcmp(&pos, &resultPos, sizeof(resultPos)), 0);
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

    nmea_parse_vtg_to_did_gps(resultVel, abuf, n, pos.lla);

    for (int i=0; i<3; i++)
    {
        ASSERT_NEAR(vel.vel[i], resultVel.vel[i], 0.02f);
    }
}

#define ASCII_BUF2  2048

TEST(protocol_nmea, GSV_binary_GSV)
{
    nmea_set_protocol_version(NMEA_PROTOCOL_2P3);

    string buf;

    // GPS & SBAS        #msgs,msg#,numSV,  svid,elv,azm,cno, ..., signalId*checksum
    buf += "$GPGSV,6,1,23" ",02,40,310,43" ",08,07,324,31" ",10,48,267,45" ",15,37,053,45"      "*7C\r\n";
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
    nmea_set_protocol_version(NMEA_PROTOCOL_4P10);

    string buf;

    // GPS & SBAS        #msgs,msg#,numSV,  svid,elv,azm,cno, ..., signalId*checksum
    buf += "$GPGSV,4,1,14" ",02,40,310,43" ",08,07,324,31" ",10,48,267,45" ",15,37,053,45"  ",1" "*67\r\n";
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
    gpsSat.numSats = 33;
    gpsSat.timeOfWeekMs = 436693200;
    gps_sat_sv_t *sat = &(gpsSat.sat[0]);
    sat->azim = 310;
    sat->cno = 43;
    sat->elev = 40;
    sat->gnssId = 1;
    sat->status = 95;
    sat->svId = 2;
    sat++;
    sat->azim = 324;
    sat->cno = 31;
    sat->elev = 07;
    sat->gnssId = 1;
    sat->status = 95;
    sat->svId = 8;
    sat++;
    sat->azim = 267;
    sat->cno = 45;
    sat->elev = 48;
    sat->gnssId = 1;
    sat->status = 31;
    sat->svId = 10;
    sat++;
    sat->azim = 53;
    sat->cno = 45;
    sat->elev = 37;
    sat->gnssId = 1;
    sat->status = 31;
    sat->svId = 15;
    sat++;
    sat->azim = 268;
    sat->cno = 37;
    sat->elev = 12;
    sat->gnssId = 1;
    sat->status = 95;
    sat->svId = 16;
    sat++;
    sat->azim = 78;
    sat->cno = 41;
    sat->elev = 69;
    sat->gnssId = 1;
    sat->status = 31;
    sat->svId = 18;
    sat++;
    sat->azim = 336;
    sat->cno = 41;
    sat->elev = 74;
    sat->gnssId = 1;
    sat->status = 31;
    sat->svId = 23;
    sat++;
    sat->azim = 111;
    sat->cno = 37;
    sat->elev = 15;
    sat->gnssId = 1;
    sat->status = 31;
    sat->svId = 24;
    sat++;
    sat->azim = 239;
    sat->cno = 31;
    sat->elev = 02;
    sat->gnssId = 1;
    sat->status = 31;
    sat->svId = 26;
    sat++;
    sat->azim = 307;
    sat->cno = 41;
    sat->elev = 35;
    sat->gnssId = 1;
    sat->status = 95;
    sat->svId = 27;
    sat++;
    sat->azim = 162;
    sat->cno = 36;
    sat->elev = 12;
    sat->gnssId = 1;
    sat->status = 31;
    sat->svId = 29;
    sat++;
    sat->azim = 199;
    sat->cno = 40;
    sat->elev = 14;
    sat->gnssId = 1;
    sat->status = 95;
    sat->svId = 32;
    sat++;
    sat->azim = 188;
    sat->cno = 43;
    sat->elev = 43;
    sat->gnssId = 2;
    sat->status = 95;
    sat->svId = 131;
    sat++;
    sat->azim = 206;
    sat->cno = 43;
    sat->elev = 40;
    sat->gnssId = 2;
    sat->status = 95;
    sat->svId = 133;
    sat++;
    sat->azim = 173;
    sat->cno = 35;
    sat->elev = 43;
    sat->gnssId = 2;
    sat->status = 95;
    sat->svId = 138;
    sat++;
    sat->azim = 145;
    sat->cno = 41;
    sat->elev = 65;
    sat->gnssId = 3;
    sat->status = 31;
    sat->svId = 5;
    sat++;
    sat->azim = 53;
    sat->cno = 43;
    sat->elev = 39;
    sat->gnssId = 3;
    sat->status = 31;
    sat->svId = 9;
    sat++;
    sat->azim = 340;
    sat->cno = 42;
    sat->elev = 71;
    sat->gnssId = 3;
    sat->status = 31;
    sat->svId = 34;
    sat++;
    sat->azim = 104;
    sat->cno = 40;
    sat->elev = 47;
    sat->gnssId = 3;
    sat->status = 31;
    sat->svId = 36;
    sat++;
    sat->azim = 141;
    sat->cno = 36;
    sat->elev = 9;
    sat->gnssId = 4;
    sat->status = 23;
    sat->svId = 11;
    sat++;
    sat->azim = 47;
    sat->cno = 44;
    sat->elev = 52;
    sat->gnssId = 4;
    sat->status = 31;
    sat->svId = 14;
    sat++;
    sat->azim = 313;
    sat->cno = 38;
    sat->elev = 31;
    sat->gnssId = 4;
    sat->status = 31;
    sat->svId = 27;
    sat++;
    sat->azim = 267;
    sat->cno = 44;
    sat->elev = 79;
    sat->gnssId = 4;
    sat->status = 31;
    sat->svId = 28;
    sat++;
    sat->azim = 40;
    sat->cno = 42;
    sat->elev = 82;
    sat->gnssId = 4;
    sat->status = 31;
    sat->svId = 33;
    sat++;
    sat->azim = 230;
    sat->cno = 43;
    sat->elev = 43;
    sat->gnssId = 4;
    sat->status = 31;
    sat->svId = 41;
    sat++;
    sat->azim = 148;
    sat->cno = 43;
    sat->elev = 34;
    sat->gnssId = 4;
    sat->status = 31;
    sat->svId = 43;
    sat++;
    sat->azim = 0;
    sat->cno = 43;
    sat->elev = 0;
    sat->gnssId = 4;
    sat->status = 39;
    sat->svId = 58;
    sat++;
    sat->azim = 252;
    sat->cno = 31;
    sat->elev = 85;
    sat->gnssId = 6;
    sat->status = 23;
    sat->svId = 1;
    sat++;
    sat->azim = 217;
    sat->cno = 29;
    sat->elev = 28;
    sat->gnssId = 6;
    sat->status = 31;
    sat->svId = 2;
    sat++;
    sat->azim = 34;
    sat->cno = 35;
    sat->elev = 37;
    sat->gnssId = 6;
    sat->status = 31;
    sat->svId = 8;
    sat++;
    sat->azim = 324;
    sat->cno = 19;
    sat->elev = 19;
    sat->gnssId = 6;
    sat->status = 19;
    sat->svId = 17;
    sat++;
    sat->azim = 126;
    sat->cno = 36;
    sat->elev = 48;
    sat->gnssId = 6;
    sat->status = 31;
    sat->svId = 23;
    sat++;
    sat->azim = 349;
    sat->cno = 32;
    sat->elev = 72;
    sat->gnssId = 6;
    sat->status = 28;
    sat->svId = 24;
    sat++;
    sat->azim = 0;
    sat->cno = 1;
    sat->elev = 1;
    sat->gnssId = 4;
    sat->status = 0;
    sat->svId = 0;
    sat++;
    sat->azim = 0;
    sat->cno = 2;
    sat->elev = 0;
    sat->gnssId = 0;
    sat->status = 0;
    sat->svId = 0;

    // Check array out of bounds
    ASSERT_TRUE( sat < &(gpsSat.sat[MAX_NUM_SATELLITES]) );



    gps_sig_t gpsSig = {};
    gpsSig.numSigs = 46;
    gpsSig.timeOfWeekMs = 436693200;
    gps_sig_sv_t *sig = &(gpsSig.sig[0]);
    sig->cno = 43;
    sig->gnssId = 1;
    sig->quality = 7;
    sig->sigId = 0;
    sig->status = 361;
    sig->svId = 2;   
    sig++;
    sig->cno = 31;
    sig->gnssId = 1;
    sig->quality = 7;
    sig->sigId = 0;
    sig->status = 41;
    sig->svId = 8;
    sig++;
    sig->cno = 45;
    sig->gnssId = 1;
    sig->quality = 7;
    sig->sigId = 0;
    sig->status = 41;
    sig->svId = 10;
    sig++;
    sig->cno = 45;
    sig->gnssId = 1;
    sig->quality = 7;
    sig->sigId = 0;
    sig->status = 41;
    sig->svId = 15;
    sig++;
    sig->cno = 37;
    sig->gnssId = 1;
    sig->quality = 7;
    sig->sigId = 0;
    sig->status = 361;
    sig->svId = 16;
    sig++;
    sig->cno = 41;
    sig->gnssId = 1;
    sig->quality = 7;
    sig->sigId = 0;
    sig->status = 41;
    sig->svId = 18;
    sig++;
    sig->cno = 41;
    sig->gnssId = 1;
    sig->quality = 7;
    sig->sigId = 0;
    sig->status = 41;
    sig->svId = 23;
    sig++;
    sig->cno = 37;
    sig->gnssId = 1;
    sig->quality = 7;
    sig->sigId = 0;
    sig->status = 41;
    sig->svId = 24;
    sig++;
    sig->cno = 31;
    sig->gnssId = 1;
    sig->quality = 7;
    sig->sigId = 0;
    sig->status = 41;
    sig->svId = 26;
    sig++;
    sig->cno = 41;
    sig->gnssId = 1;
    sig->quality = 7;
    sig->sigId = 0;
    sig->status = 361;
    sig->svId = 27;
    sig++;
    sig->cno = 36;
    sig->gnssId = 1;
    sig->quality = 7;
    sig->sigId = 0;
    sig->status = 41;
    sig->svId = 29;
    sig++;
    sig->cno = 40;
    sig->gnssId = 1;
    sig->quality = 7;
    sig->sigId = 0;
    sig->status = 361;
    sig->svId = 32;
    sig++;
    sig->cno = 43;
    sig->gnssId = 2;
    sig->quality = 7;
    sig->sigId = 0;
    sig->status = 361;
    sig->svId = 131;
    sig++;
    sig->cno = 43;
    sig->gnssId = 2;
    sig->quality = 7;
    sig->sigId = 0;
    sig->status = 361;
    sig->svId = 133;
    sig++;
    sig->cno = 35;
    sig->gnssId = 2;
    sig->quality = 7;
    sig->sigId = 0;
    sig->status = 361;
    sig->svId = 138;
    sig++;
    sig->cno = 45;
    sig->gnssId = 1;
    sig->quality = 7;
    sig->sigId = 3;
    sig->status = 41;
    sig->svId = 10;
    sig++;
    sig->cno = 27;
    sig->gnssId = 1;
    sig->quality = 7;
    sig->sigId = 3;
    sig->status = 41;
    sig->svId = 15;
    sig++;
    sig->cno = 33;
    sig->gnssId = 1;
    sig->quality = 7;
    sig->sigId = 3;
    sig->status = 41;
    sig->svId = 18;
    sig++;
    sig->cno = 34;
    sig->gnssId = 1;
    sig->quality = 7;
    sig->sigId = 3;
    sig->status = 41;
    sig->svId = 23;
    sig++;
    sig->cno = 23;
    sig->gnssId = 1;
    sig->quality = 7;
    sig->sigId = 3;
    sig->status = 9;
    sig->svId = 27;
    sig++;
    sig->cno = 25;
    sig->gnssId = 1;
    sig->quality = 7;
    sig->sigId = 3;
    sig->status = 41;
    sig->svId = 29;
    sig++;
    sig->cno = 28;
    sig->gnssId = 1;
    sig->quality = 7;
    sig->sigId = 3;
    sig->status = 41;
    sig->svId = 32;
    sig++;
    sig->cno = 41;
    sig->gnssId = 3;
    sig->quality = 7;
    sig->sigId = 0;
    sig->status = 41;
    sig->svId = 5;
    sig++;
    sig->cno = 43;
    sig->gnssId = 3;
    sig->quality = 7;
    sig->sigId = 0;
    sig->status = 41;
    sig->svId = 9;
    sig++;
    sig->cno = 42;
    sig->gnssId = 3;
    sig->quality = 7;
    sig->sigId = 0;
    sig->status = 41;
    sig->svId = 34;
    sig++;
    sig->cno = 40;
    sig->gnssId = 3;
    sig->quality = 7;
    sig->sigId = 0;
    sig->status = 41;
    sig->svId = 36;
    sig++;
    sig->cno = 30;
    sig->gnssId = 3;
    sig->quality = 7;
    sig->sigId = 6;
    sig->status = 41;
    sig->svId = 5;
    sig++;
    sig->cno = 30;
    sig->gnssId = 3;
    sig->quality = 7;
    sig->sigId = 6;
    sig->status = 41;
    sig->svId = 9;
    sig++;
    sig->cno = 26;
    sig->gnssId = 3;
    sig->quality = 7;
    sig->sigId = 6;
    sig->status = 41;
    sig->svId = 34;
    sig++;
    sig->cno = 31;
    sig->gnssId = 3;
    sig->quality = 7;
    sig->sigId = 6;
    sig->status = 41;
    sig->svId = 36;
    sig++;
    sig->cno = 36;
    sig->gnssId = 4;
    sig->quality = 7;
    sig->sigId = 0;
    sig->status = 1;
    sig->svId = 11;
    sig++;
    sig->cno = 44;
    sig->gnssId = 4;
    sig->quality = 7;
    sig->sigId = 0;
    sig->status = 41;
    sig->svId = 14;
    sig++;
    sig->cno = 38;
    sig->gnssId = 4;
    sig->quality = 7;
    sig->sigId = 0;
    sig->status = 41;
    sig->svId = 27;
    sig++;
    sig->cno = 44;
    sig->gnssId = 4;
    sig->quality = 7;
    sig->sigId = 0;
    sig->status = 41;
    sig->svId = 28;
    sig++;
    sig->cno = 42;
    sig->gnssId = 4;
    sig->quality = 7;
    sig->sigId = 0;
    sig->status = 41;
    sig->svId = 33;
    sig++;
    sig->cno = 43;
    sig->gnssId = 4;
    sig->quality = 7;
    sig->sigId = 0;
    sig->status = 41;
    sig->svId = 41;
    sig++;
    sig->cno = 43;
    sig->gnssId = 4;
    sig->quality = 7;
    sig->sigId = 0;
    sig->status = 41;
    sig->svId = 43;
    sig++;
    sig->cno = 43;
    sig->gnssId = 4;
    sig->quality = 7;
    sig->sigId = 0;
    sig->status = 2;
    sig->svId = 58;
    sig++;
    sig->cno = 21;
    sig->gnssId = 4;
    sig->quality = 7;
    sig->sigId = 2;
    sig->status = 1;
    sig->svId = 11;
    sig++;
    sig->cno = 30;
    sig->gnssId = 4;
    sig->quality = 7;
    sig->sigId = 2;
    sig->status = 41;
    sig->svId = 14;
    sig++;
    sig->cno = 31;
    sig->gnssId = 6;
    sig->quality = 7;
    sig->sigId = 0;
    sig->status = 1;
    sig->svId = 1;
    sig++;
    sig->cno = 29;
    sig->gnssId = 6;
    sig->quality = 7;
    sig->sigId = 0;
    sig->status = 41;
    sig->svId = 2;
    sig++;
    sig->cno = 35;
    sig->gnssId = 6;
    sig->quality = 7;
    sig->sigId = 0;
    sig->status = 41;
    sig->svId = 8;
    sig++;
    sig->cno = 19;
    sig->gnssId = 6;
    sig->quality = 7;
    sig->sigId = 0;
    sig->status = 1;
    sig->svId = 17;
    sig++;
    sig->cno = 36;
    sig->gnssId = 6;
    sig->quality = 7;
    sig->sigId = 0;
    sig->status = 41;
    sig->svId = 23;
    sig++;
    sig->cno = 32;
    sig->gnssId = 6;
    sig->quality = 7;
    sig->sigId = 0;
    sig->status = 41;
    sig->svId = 24;
    sig++;
    sig->cno = 208;
    sig->gnssId = 0;
    sig->quality = 233;
    sig->sigId = 0;
    sig->status = 8978;
    sig->svId = 0;
    sig++;
    sig->cno = 1;
    sig->gnssId = 107;
    sig->quality = 0;
    sig->sigId = 133;
    sig->status = 256;
    sig->svId = 33;
    sig++;
    sig->cno = 15;
    sig->gnssId = 0;
    sig->quality = 35;
    sig->sigId = 0;
    sig->status = 389;
    sig->svId = 0;
    sig++;
    sig->cno = 236;
    sig->gnssId = 0;
    sig->quality = 18;
    sig->sigId = 0;
    sig->status = 34083;
    sig->svId = 0;
    sig++;
    sig->cno = 112;
    sig->gnssId = 1;
    sig->quality = 0;
    sig->sigId = 0;
    sig->status = 0;
    sig->svId = 0;
    sig++;
    sig->cno = 35;
    sig->gnssId = 160;
    sig->quality = 133;
    sig->sigId = 18;
    sig->status = 1;
    sig->svId = 234;
    sig++;
    sig->cno = 235;
    sig->gnssId = 0;
    sig->quality = 18;
    sig->sigId = 48;
    sig->status = 34083;
    sig->svId = 0;
    sig++;
    sig->cno = 208;
    sig->gnssId = 1;
    sig->quality = 235;
    sig->sigId = 0;
    sig->status = 8978;
    sig->svId = 0;
    sig++;
    sig->cno = 1;
    sig->gnssId = 107;
    sig->quality = 0;
    sig->sigId = 133;
    sig->status = 10240;
    sig->svId = 33;
    sig++;
    sig->cno = 1;
    sig->gnssId = 243;
    sig->quality = 0;
    sig->sigId = 118;
    sig->status = 0;
    sig->svId = 186;
    sig++;
    sig->cno = 33;
    sig->gnssId = 100;
    sig->quality = 133;
    sig->sigId = 107;
    sig->status = 1;
    sig->svId = 7;
    sig++;
    sig->cno = 15;
    sig->gnssId = 0;
    sig->quality = 35;
    sig->sigId = 0;
    sig->status = 389;
    sig->svId = 0;
    sig++;
    sig->cno = 235;
    sig->gnssId = 0;
    sig->quality = 18;
    sig->sigId = 64;
    sig->status = 34083;
    sig->svId = 0;
    sig++;
    sig->cno = 52;
    sig->gnssId = 1;
    sig->quality = 0;
    sig->sigId = 0;
    sig->status = 0;
    sig->svId = 0;
    sig++;
    sig->cno = 107;
    sig->gnssId = 0;
    sig->quality = 33;
    sig->sigId = 0;
    sig->status = 389;
    sig->svId = 0;
    sig++;
    sig->cno = 234;
    sig->gnssId = 0;
    sig->quality = 18;
    sig->sigId = 176;
    sig->status = 34083;
    sig->svId = 0;
    sig++;
    sig->cno = 74;
    sig->gnssId = 1;
    sig->quality = 0;
    sig->sigId = 0;
    sig->status = 19972;
    sig->svId = 0;
    sig++;
    sig->cno = 33;
    sig->gnssId = 90;
    sig->quality = 133;
    sig->sigId = 107;
    sig->status = 1;
    sig->svId = 7;
    sig++;
    sig->cno = 186;
    sig->gnssId = 0;
    sig->quality = 118;
    sig->sigId = 242;
    sig->status = 1;
    sig->svId = 48;

    // Check array out of bounds
    ASSERT_TRUE( sig < &(gpsSig.sig[MAX_NUM_SAT_SIGNALS]) );


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


#if 0   // Uncomment to generate example NMEA strings for user manual documentation. 
TEST(protocol_nmea, generate_example_nmea_for_user_manual)
{
    uint32_t options = RMC_OPTIONS_PRESERVE_CTRL | RMC_OPTIONS_PERSISTENT;

    for (int id=0; id<=NMEA_MSG_ID_GxGSV; id++)
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
    nmea_sprint(a, ASCII_BUF_LEN, n, ",%u,%u", NMEA_MSG_ID_GxGGA, 1);
    nmea_sprint(a, ASCII_BUF_LEN, n, ",%u,%u", NMEA_MSG_ID_GxGLL, 1);
    nmea_sprint(a, ASCII_BUF_LEN, n, ",%u,%u", NMEA_MSG_ID_GxGSA, 1);
    nmea_sprint(a, ASCII_BUF_LEN, n, ",%u,%u", NMEA_MSG_ID_GxZDA, 1);
    nmea_sprint(a, ASCII_BUF_LEN, n, ",%u,%u", NMEA_MSG_ID_GxGSV, 1);
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
