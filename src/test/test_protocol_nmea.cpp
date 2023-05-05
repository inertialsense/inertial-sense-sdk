#include <gtest/gtest.h>
#include <vector>
#include "../protocol_nmea.h"
#include "../ISEarth.h"
using namespace std;


#define ASCII_BUF_LEN   200
#define POS_LAT_DEG     40.330578
#define POS_LON_DEG     -111.725816
#define POS_ALT_M       1406.39
#define LEAP_SEC        18

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
    info.buildDate[0] = 0;
    snprintf(info.addInfo, DEVINFO_ADDINFO_STRLEN, "additional string   123");

    char ascii_buf[ASCII_BUF_LEN] = { 0 };
    did_dev_info_to_nmea_info(ascii_buf, ASCII_BUF_LEN, info);
    // printf("%s\n", ascii_buf);
    dev_info_t result = {};
    nmea_info_to_did_dev_info(result, ascii_buf, ASCII_BUF_LEN);
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

    char ascii_buf[ASCII_BUF_LEN] = { 0 };
    did_imu_to_nmea_pimu(ascii_buf, ASCII_BUF_LEN, imu, "$PIMU");
    // printf("%s\n", ascii_buf);
    imu_t result = {};
    nmea_pimu_to_did_imu(result, ascii_buf, ASCII_BUF_LEN);
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

    char ascii_buf[ASCII_BUF_LEN] = { 0 };
    did_imu_to_nmea_pimu(ascii_buf, ASCII_BUF_LEN, imu, "$PRIMU");
    // printf("%s\n", ascii_buf);
    imu_t result = {};
    nmea_pimu_to_did_rimu(result, ascii_buf, ASCII_BUF_LEN);
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

    char ascii_buf[ASCII_BUF_LEN] = { 0 };
    did_pimu_to_nmea_ppimu(ascii_buf, ASCII_BUF_LEN, pimu);
    // printf("%s\n", ascii_buf);
    pimu_t result = {};
    nmea_ppimu_to_did_pimu(result, ascii_buf, ASCII_BUF_LEN);
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

    char ascii_buf[ASCII_BUF_LEN] = { 0 };
    did_ins1_to_nmea_pins1(ascii_buf, ASCII_BUF_LEN, ins);
    // printf("%s\n", ascii_buf);
    ins_1_t result = {};
    nmea_pins1_to_did_ins1(result, ascii_buf, ASCII_BUF_LEN);
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

    char ascii_buf[ASCII_BUF_LEN] = { 0 };
    did_ins2_to_nmea_pins2(ascii_buf, ASCII_BUF_LEN, ins);
    // printf("%s\n", ascii_buf);
    ins_2_t result = {};
    nmea_pins2_to_did_ins2(result, ascii_buf, ASCII_BUF_LEN);
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

    char ascii_buf[ASCII_BUF_LEN] = { 0 };
    did_gps_to_nmea_pgpsp(ascii_buf, ASCII_BUF_LEN, pos, vel);
    // printf("%s\n", ascii_buf);
    gps_pos_t resultPos = {};
    gps_vel_t resultVel = {};
    nmea_pgpsp_to_did_gps(resultPos, resultVel, ascii_buf, ASCII_BUF_LEN);
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
    pos.hMSL = 1438.2;
    pos.lla[0] =  ( 40.0 +  3.34247/60.0);
    pos.lla[1] = -(111.0 + 39.51850/60.0);
    pos.lla[2] = pos.hMSL - 18.8;
    pos.pDop = 0.47;
    pos.leapS = LEAP_SEC;
	// Convert LLA to ECEF.  Ensure LLA uses ellipsoid altitude
	ixVector3d lla;
	lla[0] = DEG2RAD(pos.lla[0]);
	lla[1] = DEG2RAD(pos.lla[1]);
	lla[2] = pos.lla[2];		// Use ellipsoid altitude
	lla2ecef(lla, pos.ecef);

    char ascii_buf[ASCII_BUF_LEN] = { 0 };
    int n = did_gps_to_nmea_gga(ascii_buf, ASCII_BUF_LEN, pos);
    // printf("%s\n", gga);
    // printf("%s\n", ascii_buf);
    ASSERT_EQ(memcmp(&gga, &ascii_buf, n), 0);

    gps_pos_t result = {};
    result.week = pos.week;
    result.leapS = pos.leapS;
    uint32_t weekday = pos.timeOfWeekMs / 86400000;
    nmea_gga_to_did_gps(result, ascii_buf, ASCII_BUF_LEN, weekday);
    pos.hAcc = result.hAcc;
    ASSERT_EQ(memcmp(&pos, &result, sizeof(result)), 0);
}

TEST(protocol_nmea, GGL)
{
    gps_pos_t pos = {};
    // pos.week = 12;
    pos.timeOfWeekMs = 370659600;
    pos.status = GPS_STATUS_NUM_SATS_USED_MASK & pos.satsUsed;
    pos.lla[0] = POS_LAT_DEG;
    pos.lla[1] = POS_LON_DEG;
    pos.leapS = LEAP_SEC;

    char ascii_buf[ASCII_BUF_LEN] = { 0 };
    did_gps_to_nmea_gll(ascii_buf, ASCII_BUF_LEN, pos);
    // printf("%s\n", ascii_buf);
    gps_pos_t result = {};
    result.leapS = pos.leapS;
    uint32_t weekday = pos.timeOfWeekMs / 86400000;
    nmea_gll_to_did_gps(result, ascii_buf, ASCII_BUF_LEN, weekday);
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

    char ascii_buf[ASCII_BUF_LEN] = { 0 };
    did_gps_to_nmea_gsa(ascii_buf, ASCII_BUF_LEN, pos, sat);
    // printf("%s\n", ascii_buf);
    gps_pos_t resultPos = {};
    gps_sat_t resultSat = {};
    nmea_gsa_to_did_gps(resultPos, resultSat, ascii_buf, ASCII_BUF_LEN);
    ASSERT_EQ(memcmp(&pos, &resultPos, sizeof(resultPos)), 0);
    ASSERT_EQ(memcmp(&sat, &resultSat, sizeof(resultSat)), 0);
}

TEST(protocol_nmea, GSV)
{
    vector <string> buf;

    // $GPGSV,4,1,16, 02,80,085,53, 11,76,091,,   12,61,180,51, 25,58,284,52*79
    // $GPGSV,4,2,16, 20,42,142,50, 06,33,056,47, 29,28,280,47, 05,19,166,47*7F
    // $GPGSV,4,3,16, 31,15,321,46, 19,08,093,46, 04,04,025,,   09,03,055,*78
    // $GPGSV,4,4,16, 44,32,184,48, 51,31,171,48, 48,31,194,47, 46,30,199,48*7E

    // GPS & SBAS         #msgs,msg#,numSV, svid,elv,azm,cno, ..., signalId*checksum
    // buf.push_back("$GPGSV,3,1,12,    05,11,296,40,   07,71,058,48,   08,42,063,48,   09,37,171,45,   1*65\r\n");
    // buf.push_back("$GPGSV,3,2,12,    13,13,319,43,   14,46,250,46,   17,07,192,38,   20,07,272,35,   1*6C\r\n");
    // buf.push_back("$GPGSV,3,3,12,    27,15,040,43,   30,61,316,48,   44,43,188,45,   46,40,206,44,   1*6C\r\n");
    // buf.push_back("$GPGSV,1,1,02,    04,08,155,,     21,08,107,,                                     0*67\r\n");
    // GLONASS
    // buf.push_back("$GLGSV,3,1,09,    69,30,200,43,   70,47,285,35,   71,15,332,44,   73,11,271,38,   1*7C\r\n");
    // buf.push_back("$GLGSV,3,2,09,    79,42,044,49,   80,53,314,48,   81,30,083,47,   82,16,134,40,   1*7C\r\n");
    // buf.push_back("$GLGSV,3,3,09,    88,14,030,42,                                                   1*41\r\n");
    // buf.push_back("$GLGSV,1,1,01,    78,00,072,,                                                     0*42\r\n");
    // Galileo
    // buf.push_back("$GAGSV,3,1,09,    02,68,274,47,   03,53,189,45,   07,10,322,44,   08,55,289,46,   7*7E\r\n");
    // buf.push_back("$GAGSV,3,2,09,    10,10,063,43,   12,05,040,36,   14,07,197,38,   25,53,046,47,   7*78\r\n");
    // buf.push_back("$GAGSV,3,3,09,    30,17,248,36,                                                   7*44\r\n");
    // buf.push_back("$GAGSV,1,1,03,    05,08,158,,     24,02,060,,     36,01,134,,                     0*46\r\n");
    // QZSS & IRNSS
    // buf.push_back("$GQGSV,1,1,00,                                                                    0*64\r\n");

    // GPS & SBAS         #msgs,msg#,numSV, svid,elv,azm,cno, ..., signalId*checksum
    buf.push_back("$GPGSV,3,1,12,05,11,296,40,07,71,058,48,08,42,063,48,09,37,171,45,1*65\r\n");
    buf.push_back("$GPGSV,3,2,12,13,13,319,43,14,46,250,46,17,07,192,38,20,07,272,35,1*6C\r\n");
    buf.push_back("$GPGSV,3,3,12,27,15,040,43,30,61,316,48,44,43,188,45,46,40,206,44,1*6C\r\n");
    buf.push_back("$GPGSV,1,1,02,04,08,155,,21,08,107,,0*67\r\n");
    // GLONASS
    buf.push_back("$GLGSV,3,1,09,69,30,200,43,70,47,285,35,71,15,332,44,73,11,271,38,1*7C\r\n");
    buf.push_back("$GLGSV,3,2,09,79,42,044,49,80,53,314,48,81,30,083,47,82,16,134,40,1*7C\r\n");
    buf.push_back("$GLGSV,3,3,09,88,14,030,42,1*41\r\n");
    buf.push_back("$GLGSV,1,1,01,78,00,072,,0*42\r\n");
    // Galileo
    buf.push_back("$GAGSV,3,1,09,02,68,274,47,03,53,189,45,07,10,322,44,08,55,289,46,7*7E\r\n");
    buf.push_back("$GAGSV,3,2,09,10,10,063,43,12,05,040,36,14,07,197,38,25,53,046,47,7*78\r\n");
    buf.push_back("$GAGSV,3,3,09,30,17,248,36,7*44\r\n");
    buf.push_back("$GAGSV,1,1,03,05,08,158,,24,02,060,,36,01,134,,0*46\r\n");
    // QZSS & IRNSS
    buf.push_back("$GQGSV,1,1,00,0*64\r\n");


    gps_sat_t sat = {};
    sat.numSats = 33;
    sat.timeOfWeekMs = 509591200;
    sat.sat[0].azim = 145;
    sat.sat[0].cno = 44;
    sat.sat[0].elev = 61;
    sat.sat[0].flags = 5314911;
    sat.sat[0].gnssId = 0;
    sat.sat[0].prRes = 1;
    sat.sat[0].svId = 5;
    sat.sat[1].azim = 61;
    sat.sat[1].cno = 0;
    sat.sat[1].elev = 3;
    sat.sat[1].flags = 4625;
    sat.sat[1].gnssId = 0;
    sat.sat[1].prRes = 0;
    sat.sat[1].svId = 6;
    sat.sat[2].azim = 34;
    sat.sat[2].cno = 0;
    sat.sat[2].elev = 1;
    sat.sat[2].flags = 4625;
    sat.sat[2].gnssId = 0;
    sat.sat[2].prRes = 0;
    sat.sat[2].svId = 9;
    sat.sat[3].azim = 51;
    sat.sat[3].cno = 40;
    sat.sat[3].elev = 42;
    sat.sat[3].flags = 5314911;
    sat.sat[3].gnssId = 0;
    sat.sat[3].prRes = 4;
    sat.sat[3].svId = 11;
    sat.sat[4].azim = 181;
    sat.sat[4].cno = 43;
    sat.sat[4].elev = 47;
    sat.sat[4].flags = 5310815;
    sat.sat[4].gnssId = 0;
    sat.sat[4].prRes = 1;
    sat.sat[4].svId = 12;
    sat.sat[5].azim = 249;
    sat.sat[5].cno = 35;
    sat.sat[5].elev = 11;
    sat.sat[5].flags = 5310815;
    sat.sat[5].gnssId = 0;
    sat.sat[5].prRes = 4;
    sat.sat[5].svId = 18;
    sat.sat[6].azim = 80;
    sat.sat[6].cno = 42;
    sat.sat[6].elev = 62;
    sat.sat[6].flags = 5314911;
    sat.sat[6].gnssId = 0;
    sat.sat[6].prRes = 3;
    sat.sat[6].svId = 20;
    sat.sat[7].azim = 243;
    sat.sat[7].cno = 39;
    sat.sat[7].elev = 61;
    sat.sat[7].flags = 5314911;
    sat.sat[7].gnssId = 0;
    sat.sat[7].prRes = 22;
    sat.sat[7].svId = 25;
    sat.sat[8].azim = 289;
    sat.sat[8].cno = 37;
    sat.sat[8].elev = 8;
    sat.sat[8].flags = 6423;
    sat.sat[8].gnssId = 0;
    sat.sat[8].prRes = 17;
    sat.sat[8].svId = 28;
    sat.sat[9].azim = 312;
    sat.sat[9].cno = 42;
    sat.sat[9].elev = 50;
    sat.sat[9].flags = 5314911;
    sat.sat[9].gnssId = 0;
    sat.sat[9].prRes = 11;
    sat.sat[9].svId = 29;
    sat.sat[10].azim = 313;
    sat.sat[10].cno = 30;
    sat.sat[10].elev = 6;
    sat.sat[10].flags = 6423;
    sat.sat[10].gnssId = 0;
    sat.sat[10].prRes = 35;
    sat.sat[10].svId = 31;
    sat.sat[11].azim = 188;
    sat.sat[11].cno = 42;
    sat.sat[11].elev = 43;
    sat.sat[11].flags = 5314911;
    sat.sat[11].gnssId = 1;
    sat.sat[11].prRes = 4;
    sat.sat[11].svId = 131;
    sat.sat[12].azim = 206;
    sat.sat[12].cno = 42;
    sat.sat[12].elev = 40;
    sat.sat[12].flags = 5314911;
    sat.sat[12].gnssId = 1;
    sat.sat[12].prRes = 9;
    sat.sat[12].svId = 133;
    sat.sat[13].azim = 173;
    sat.sat[13].cno = 0;
    sat.sat[13].elev = 43;
    sat.sat[13].flags = 1793;
    sat.sat[13].gnssId = 1;
    sat.sat[13].prRes = 0;
    sat.sat[13].svId = 138;
    sat.sat[14].azim = 32;
    sat.sat[14].cno = 0;
    sat.sat[14].elev = 2;
    sat.sat[14].flags = 4625;
    sat.sat[14].gnssId = 2;
    sat.sat[14].prRes = 0;
    sat.sat[14].svId = 7;
    sat.sat[15].azim = 267;
    sat.sat[15].cno = 34;
    sat.sat[15].elev = 50;
    sat.sat[15].flags = 6431;
    sat.sat[15].gnssId = 2;
    sat.sat[15].prRes = 33;
    sat.sat[15].svId = 10;
    sat.sat[16].azim = 255;
    sat.sat[16].cno = 33;
    sat.sat[16].elev = 23;
    sat.sat[16].flags = 2335;
    sat.sat[16].gnssId = 2;
    sat.sat[16].prRes = 47;
    sat.sat[16].svId = 11;
    sat.sat[17].azim = 304;
    sat.sat[17].cno = 37;
    sat.sat[17].elev = 71;
    sat.sat[17].flags = 2335;
    sat.sat[17].gnssId = 2;
    sat.sat[17].prRes = 13;
    sat.sat[17].svId = 12;
    sat.sat[18].azim = 0;
    sat.sat[18].cno = 32;
    sat.sat[18].elev = 91;
    sat.sat[18].flags = 3;
    sat.sat[18].gnssId = 2;
    sat.sat[18].prRes = 0;
    sat.sat[18].svId = 14;
    sat.sat[19].azim = 266;
    sat.sat[19].cno = 39;
    sat.sat[19].elev = 59;
    sat.sat[19].flags = 6431;
    sat.sat[19].gnssId = 2;
    sat.sat[19].prRes = 22;
    sat.sat[19].svId = 24;
    sat.sat[20].azim = 319;
    sat.sat[20].cno = 22;
    sat.sat[20].elev = 19;
    sat.sat[20].flags = 6428;
    sat.sat[20].gnssId = 2;
    sat.sat[20].prRes = 2;
    sat.sat[20].svId = 25;
    sat.sat[21].azim = 183;
    sat.sat[21].cno = 41;
    sat.sat[21].elev = 41;
    sat.sat[21].flags = 6431;
    sat.sat[21].gnssId = 2;
    sat.sat[21].prRes = 0;
    sat.sat[21].svId = 31;
    sat.sat[22].azim = 51;
    sat.sat[22].cno = 41;
    sat.sat[22].elev = 41;
    sat.sat[22].flags = 6431;
    sat.sat[22].gnssId = 2;
    sat.sat[22].prRes = 8;
    sat.sat[22].svId = 33;
    sat.sat[23].azim = 313;
    sat.sat[23].cno = 32;
    sat.sat[23].elev = 1;
    sat.sat[23].flags = 6423;
    sat.sat[23].gnssId = 5;
    sat.sat[23].prRes = 24;
    sat.sat[23].svId = 2;
    sat.sat[24].azim = 132;
    sat.sat[24].cno = 29;
    sat.sat[24].elev = 26;
    sat.sat[24].flags = 6431;
    sat.sat[24].gnssId = 6;
    sat.sat[24].prRes = 22;
    sat.sat[24].svId = 1;
    sat.sat[25].azim = 24;
    sat.sat[25].cno = 0;
    sat.sat[25].elev = 11;
    sat.sat[25].flags = 4625;
    sat.sat[25].gnssId = 6;
    sat.sat[25].prRes = 0;
    sat.sat[25].svId = 7;
    sat.sat[26].azim = 70;
    sat.sat[26].cno = 31;
    sat.sat[26].elev = 33;
    sat.sat[26].flags = 6431;
    sat.sat[26].gnssId = 6;
    sat.sat[26].prRes = 77;
    sat.sat[26].svId = 8;
    sat.sat[27].azim = 197;
    sat.sat[27].cno = 30;
    sat.sat[27].elev = 16;
    sat.sat[27].flags = 6431;
    sat.sat[27].gnssId = 6;
    sat.sat[27].prRes = 37;
    sat.sat[27].svId = 13;
    sat.sat[28].azim = 258;
    sat.sat[28].cno = 47;
    sat.sat[28].elev = 46;
    sat.sat[28].flags = 6431;
    sat.sat[28].gnssId = 6;
    sat.sat[28].prRes = 56;
    sat.sat[28].svId = 14;
    sat.sat[29].azim = 321;
    sat.sat[29].cno = 38;
    sat.sat[29].elev = 26;
    sat.sat[29].flags = 6431;
    sat.sat[29].gnssId = 6;
    sat.sat[29].prRes = 4;
    sat.sat[29].svId = 15;
    sat.sat[30].azim = 280;
    sat.sat[30].cno = 30;
    sat.sat[30].elev = 18;
    sat.sat[30].flags = 6428;
    sat.sat[30].gnssId = 6;
    sat.sat[30].prRes = 73;
    sat.sat[30].svId = 17;
    sat.sat[31].azim = 50;
    sat.sat[31].cno = 34;
    sat.sat[31].elev = 37;
    sat.sat[31].flags = 6431;
    sat.sat[31].gnssId = 6;
    sat.sat[31].prRes = 51;
    sat.sat[31].svId = 23;
    sat.sat[32].azim = 332;
    sat.sat[32].cno = 37;
    sat.sat[32].elev = 56;
    sat.sat[32].flags = 6431;
    sat.sat[32].gnssId = 6;
    sat.sat[32].prRes = 120;
    sat.sat[32].svId = 24;

    char ascii_buf[ASCII_BUF_LEN] = { 0 };

    did_gps_sat_to_nmea_gsv(ascii_buf, ASCII_BUF_LEN, sat);


    // for(int i=0; i<buf.size(); i++)
    // {
    //     printf("%s", buf[i].c_str());

    // }

}



