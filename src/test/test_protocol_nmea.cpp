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

    // GPS & SBAS        #msgs,msg#,numSV,  svid,elv,azm,cno, ..., signalId*checksum
    buf.push_back("$GPGSV,3,1,12,"  "01,28,088,45," "07,32,130,46," "08,02,043,39," "13,45,279,46," "1*63\r\n");
    buf.push_back("$GPGSV,3,2,12,"  "14,72,000,47," "15,21,303,46," "17,60,196,45," "19,31,206,44," "1*61\r\n");
    buf.push_back("$GPGSV,3,3,12,"  "21,23,058,46," "30,67,129,49," "44,43,188,45," "46,40,206,44," "1*6A\r\n");
    // GLONASS
    buf.push_back("$GLGSV,3,1,10,"  "65,08,221,39," "66,27,272,44," "67,15,325,43," "75,48,065,51," "1*76\r\n");
    buf.push_back("$GLGSV,3,2,10,"  "76,47,326,48," "77,08,296,30," "84,12,024,33," "85,54,079,49," "1*71\r\n");
    buf.push_back("$GLGSV,3,3,10,"  "86,39,151,46," ",,,50,"                                        "1*4F\r\n");
    buf.push_back("$GLGSV,1,1,01,"  "74,05,098,,"                                                   "0*4F\r\n");
    // Galileo
    buf.push_back("$GAGSV,3,1,09,"  "02,30,071,44," "07,41,242,44," "08,16,189,37," "18,47,201,44," "7*7B\r\n");
    buf.push_back("$GAGSV,3,2,09,"  "20,,,36,"      "27,33,280,44," "30,71,002,47," "34,32,117,44," "7*46\r\n");
    buf.push_back("$GAGSV,3,3,09,"  "36,25,058,44,"                                                 "7*45\r\n");
    buf.push_back("$GAGSV,1,1,01,"  "15,09,164,,"                                                   "0*4B\r\n");
    // QZSS & IRNSS
    buf.push_back("$GQGSV,1,1,00,"                                                                  "0*64\r\n");

    gps_sat_t sat = {};
    sat.numSats = 44;
    sat.timeOfWeekMs = 572440400;
    sat.sat[0].azim	88;
    sat.sat[0].cno = 45;
    sat.sat[0].elev = 28;
    sat.sat[0].flags = 5314911;
    sat.sat[0].gnssId = 0;
    sat.sat[0].prRes = 0;
    sat.sat[0].svId = 1;
    sat.sat[1].azim = 130;
    sat.sat[1].cno = 46;
    sat.sat[1].elev = 32;
    sat.sat[1].flags = 5310815;
    sat.sat[1].gnssId = 0;
    sat.sat[1].prRes = 7;
    sat.sat[1].svId = 7;
    sat.sat[2].azim = 43;
    sat.sat[2].cno = 35;
    sat.sat[2].elev = 2;
    sat.sat[2].flags = 2327;
    sat.sat[2].gnssId = 0;
    sat.sat[2].prRes = 0;
    sat.sat[2].svId = 8;
    sat.sat[3].azim = 279;
    sat.sat[3].cno = 46;
    sat.sat[3].elev = 45;
    sat.sat[3].flags = 5314911;
    sat.sat[3].gnssId = 0;
    sat.sat[3].prRes = 1;
    sat.sat[3].svId = 13;
    sat.sat[4].azim = 0;
    sat.sat[4].cno = 47;
    sat.sat[4].elev = 72;
    sat.sat[4].flags = 5314911;
    sat.sat[4].gnssId = 0;
    sat.sat[4].prRes = 0;
    sat.sat[4].svId = 14;
    sat.sat[5].azim = 303;
    sat.sat[5].cno = 45;
    sat.sat[5].elev = 21;
    sat.sat[5].flags = 5314911;
    sat.sat[5].gnssId = 0;
    sat.sat[5].prRes = 6;
    sat.sat[5].svId = 15;
    sat.sat[6].azim = 196;
    sat.sat[6].cno = 45;
    sat.sat[6].elev = 60;
    sat.sat[6].flags = 5314911;
    sat.sat[6].gnssId = 0;
    sat.sat[6].prRes = 0;
    sat.sat[6].svId = 17;
    sat.sat[7].azim = 206;
    sat.sat[7].cno = 42;
    sat.sat[7].elev = 31;
    sat.sat[7].flags = 5314911;
    sat.sat[7].gnssId = 0;
    sat.sat[7].prRes = 5;
    sat.sat[7].svId = 19;
    sat.sat[8].azim = 58;
    sat.sat[8].cno = 46;
    sat.sat[8].elev = 23;
    sat.sat[8].flags = 5314911;
    sat.sat[8].gnssId = 0;
    sat.sat[8].prRes = 8;
    sat.sat[8].svId = 21;
    sat.sat[9].azim = 0;
    sat.sat[9].cno = 0;
    sat.sat[9].elev = 4294967205;
    sat.sat[9].flags = 17;
    sat.sat[9].gnssId = 0;
    sat.sat[9].prRes = 0;
    sat.sat[9].svId = 29;
    sat.sat[10].azim = 129;
    sat.sat[10].cno = 49;
    sat.sat[10].elev = 67;
    sat.sat[10].flags = 5310815;
    sat.sat[10].gnssId = 0;
    sat.sat[10].prRes = 3;
    sat.sat[10].svId = 30;
    sat.sat[11].azim = 0;
    sat.sat[11].cno = 0;
    sat.sat[11].elev = 4294967205;
    sat.sat[11].flags = 17;
    sat.sat[11].gnssId = 0;
    sat.sat[11].prRes = 0;
    sat.sat[11].svId = 31;
    sat.sat[12].azim = 188;
    sat.sat[12].cno = 45;
    sat.sat[12].elev = 43;
    sat.sat[12].flags = 5314911;
    sat.sat[12].gnssId = 1;
    sat.sat[12].prRes = 3;
    sat.sat[12].svId = 131;
    sat.sat[13].azim = 206;
    sat.sat[13].cno = 43;
    sat.sat[13].elev = 40;
    sat.sat[13].flags = 5314911;
    sat.sat[13].gnssId = 1;
    sat.sat[13].prRes = 2;
    sat.sat[13].svId = 133;
    sat.sat[14].azim = 173;
    sat.sat[14].cno = 0;
    sat.sat[14].elev = 43;
    sat.sat[14].flags = 1793;
    sat.sat[14].gnssId = 1;
    sat.sat[14].prRes = 0;
    sat.sat[14].svId = 138;
    sat.sat[15].azim = 72;
    sat.sat[15].cno = 44;
    sat.sat[15].elev = 30;
    sat.sat[15].flags = 6431;
    sat.sat[15].gnssId = 2;
    sat.sat[15].prRes = 15;
    sat.sat[15].svId = 2;
    sat.sat[16].azim = 241;
    sat.sat[16].cno = 44;
    sat.sat[16].elev = 40;
    sat.sat[16].flags = 6431;
    sat.sat[16].gnssId = 2;
    sat.sat[16].prRes = 5;
    sat.sat[16].svId = 7;
    sat.sat[17].azim = 189;
    sat.sat[17].cno = 36;
    sat.sat[17].elev = 15;
    sat.sat[17].flags = 6431;
    sat.sat[17].gnssId = 2;
    sat.sat[17].prRes = 33;
    sat.sat[17].svId = 8;
    sat.sat[18].azim = 164;
    sat.sat[18].cno = 0;
    sat.sat[18].elev = 	9;
    sat.sat[18].flags = 4625;
    sat.sat[18].gnssId = 2;
    sat.sat[18].prRes = 0;
    sat.sat[18].svId = 15;
    sat.sat[19].azim = 201;
    sat.sat[19].cno = 44;
    sat.sat[19].elev = 47;
    sat.sat[19].flags = 2343;
    sat.sat[19].gnssId = 2;
    sat.sat[19].prRes = 0;
    sat.sat[19].svId = 18;
    sat.sat[20].azim = 0;
    sat.sat[20].cno = 36;
    sat.sat[20].elev = 4294967205;
    sat.sat[20].flags = 7;
    sat.sat[20].gnssId = 2;
    sat.sat[20].prRes = 0;
    sat.sat[20].svId = 20;
    sat.sat[21].azim = 280;
    sat.sat[21].cno = 44;
    sat.sat[21].elev = 33;
    sat.sat[21].flags = 6431;
    sat.sat[21].gnssId = 2;
    sat.sat[21].prRes = 7;
    sat.sat[21].svId = 27;
    sat.sat[22].azim = 2;
    sat.sat[22].cno = 47;
    sat.sat[22].elev = 71;
    sat.sat[22].flags = 6431;
    sat.sat[22].gnssId = 2;
    sat.sat[22].prRes = 2;
    sat.sat[22].svId = 30;
    sat.sat[23].azim = 117;
    sat.sat[23].cno = 44;
    sat.sat[23].elev = 32;
    sat.sat[23].flags = 6431;
    sat.sat[23].gnssId = 2;
    sat.sat[23].prRes = 4;
    sat.sat[23].svId = 34;
    sat.sat[24].azim = 58;
    sat.sat[24].cno = 44;
    sat.sat[24].elev = 25;
    sat.sat[24].flags = 6431;
    sat.sat[24].gnssId = 2;
    sat.sat[24].prRes = 15;
    sat.sat[24].svId = 36;
    sat.sat[25].azim = 0;
    sat.sat[25].cno = 0;
    sat.sat[25].elev = 4294967205;
    sat.sat[25].flags = 1;
    sat.sat[25].gnssId = 5;
    sat.sat[25].prRes = 0;
    sat.sat[25].svId = 2;
    sat.sat[26].azim = 0;
    sat.sat[26].cno = 0;
    sat.sat[26].elev = 4294967205;
    sat.sat[26].flags = 1;
    sat.sat[26].gnssId = 5;
    sat.sat[26].prRes = 0;
    sat.sat[26].svId = 3;
    sat.sat[27].azim = 0;
    sat.sat[27].cno = 0;
    sat.sat[27].elev = 4294967205;
    sat.sat[27].flags = 1;
    sat.sat[27].gnssId = 5;
    sat.sat[27].prRes = 0;
    sat.sat[27].svId = 4;
    sat.sat[28].azim = 0;
    sat.sat[28].cno = 0;
    sat.sat[28].elev = 4294967205;
    sat.sat[28].flags = 1;
    sat.sat[28].gnssId = 5;
    sat.sat[28].prRes = 0;
    sat.sat[28].svId = 5;
    sat.sat[29].azim = 0;
    sat.sat[29].cno = 0;
    sat.sat[29].elev = 4294967205;
    sat.sat[29].flags = 1;
    sat.sat[29].gnssId = 5;
    sat.sat[29].prRes = 0;
    sat.sat[29].svId = 6;
    sat.sat[30].azim = 0;
    sat.sat[30].cno = 0;
    sat.sat[30].elev = 4294967205;
    sat.sat[30].flags = 1;
    sat.sat[30].gnssId = 5;
    sat.sat[30].prRes = 0;
    sat.sat[30].svId = 8;
    sat.sat[31].azim = 0;
    sat.sat[31].cno = 0;
    sat.sat[31].elev = 4294967205;
    sat.sat[31].flags = 1;
    sat.sat[31].gnssId = 5;
    sat.sat[31].prRes = 0;
    sat.sat[31].svId = 9;
    sat.sat[32].azim = 0;
    sat.sat[32].cno = 0;
    sat.sat[32].elev = 4294967205;
    sat.sat[32].flags = 1;
    sat.sat[32].gnssId = 5;
    sat.sat[32].prRes = 0;
    sat.sat[32].svId = 10;
    sat.sat[33].azim = 221;
    sat.sat[33].cno = 39;
    sat.sat[33].elev = 8;
    sat.sat[33].flags = 6423;
    sat.sat[33].gnssId = 6;
    sat.sat[33].prRes = 18;
    sat.sat[33].svId = 1;
    sat.sat[34].azim = 272;
    sat.sat[34].cno = 46;
    sat.sat[34].elev = 27;
    sat.sat[34].flags = 6431;
    sat.sat[34].gnssId = 6;
    sat.sat[34].prRes = 37;
    sat.sat[34].svId = 2;
    sat.sat[35].azim = 325;
    sat.sat[35].cno = 43;
    sat.sat[35].elev = 15;
    sat.sat[35].flags = 6431;
    sat.sat[35].gnssId = 6;
    sat.sat[35].prRes = 43;
    sat.sat[35].svId = 3;
    sat.sat[36].azim = 98;
    sat.sat[36].cno = 0;
    sat.sat[36].elev = 5;
    sat.sat[36].flags = 4624;
    sat.sat[36].gnssId = 6;
    sat.sat[36].prRes = 0;
    sat.sat[36].svId = 10;
    sat.sat[37].azim = 65;
    sat.sat[37].cno = 51;
    sat.sat[37].elev = 48;
    sat.sat[37].flags = 6431;
    sat.sat[37].gnssId = 6;
    sat.sat[37].prRes = 10;
    sat.sat[37].svId = 11;
    sat.sat[38].azim = 326;
    sat.sat[38].cno = 48;
    sat.sat[38].elev = 47;
    sat.sat[38].flags = 6431;
    sat.sat[38].gnssId = 6;
    sat.sat[38].prRes = 14;
    sat.sat[38].svId = 12;
    sat.sat[39].azim = 296;
    sat.sat[39].cno = 32;
    sat.sat[39].elev = 8;
    sat.sat[39].flags = 6423;
    sat.sat[39].gnssId = 6;
    sat.sat[39].prRes = 4;
    sat.sat[39].svId = 13;
    sat.sat[40].azim = 24;
    sat.sat[40].cno = 34;
    sat.sat[40].elev = 12;
    sat.sat[40].flags = 6431;
    sat.sat[40].gnssId = 6;
    sat.sat[40].prRes = 36;
    sat.sat[40].svId = 20;
    sat.sat[41].azim = 79;
    sat.sat[41].cno = 49;
    sat.sat[41].elev = 54;
    sat.sat[41].flags = 6431;
    sat.sat[41].gnssId = 6;
    sat.sat[41].prRes = 30;
    sat.sat[41].svId = 21;
    sat.sat[42].azim = 151;
    sat.sat[42].cno = 46;
    sat.sat[42].elev = 39;
    sat.sat[42].flags = 6431;
    sat.sat[42].gnssId = 6;
    sat.sat[42].prRes = 6;
    sat.sat[42].svId = 22;
    sat.sat[43].azim = 0;
    sat.sat[43].cno = 49;
    sat.sat[43].elev = 4294967205;
    sat.sat[43].flags = 7;
    sat.sat[43].gnssId = 6;
    sat.sat[43].prRes = 0;
    sat.sat[43].svId = 255;





#define ASCII_BUF2  1024
    char ascii_buf[ASCII_BUF2] = { 0 };

    did_gps_sat_to_nmea_gsv(ascii_buf, ASCII_BUF2, sat);
    printf("%s\n", ascii_buf);

    // for(int i=0; i<buf.size(); i++)
    // {
    //     printf("%s", buf[i].c_str());

    // }

}



