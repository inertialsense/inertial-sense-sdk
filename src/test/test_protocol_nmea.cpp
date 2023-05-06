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
    buf.push_back("$GPGSV,3,1,11,"  "05,43,161,46," "06,17,052,43," "11,60,043,47," "12,68,189,46," "1*6D\r\n");
    buf.push_back("$GPGSV,3,2,11,"  "20,61,121,47," "25,61,285,46," "28,11,305,43," "29,35,299,46," "1*6A\r\n");
    buf.push_back("$GPGSV,3,3,11,"  "44,43,188,45," "46,40,206,44," "51,43,173,28,"                 "1*5D\r\n");
    buf.push_back("$GPGSV,2,1,08,"  "05,43,161,43," "06,17,052,40," "11,60,043,46," "12,68,189,43," "6*61\r\n");
    buf.push_back("$GPGSV,2,2,08,"  "25,61,285,44," "28,11,305,43," "29,35,299,42," "31,02,328,13," "6*6D\r\n");
    buf.push_back("$GPGSV,1,1,03,"  "09,01,049,,"   "19,04,097,,"   "24,00,204,,"                   "0*51\r\n");
    // GLONASS
    buf.push_back("$GLGSV,3,1,09,"  "65,26,093,46," "72,17,038,48," "78,41,208,48," "79,50,291,47," "1*76\r\n");
    buf.push_back("$GLGSV,3,2,09,"  "80,12,334,46," "81,48,297,48," "82,09,264,32," "87,07,062,34," "1*71\r\n");
    buf.push_back("$GLGSV,3,3,09,"  "88,49,030,49,"                                                 "1*42\r\n");
    buf.push_back("$GLGSV,2,1,08,"  "65,26,093,40," "72,17,038,40," "78,41,208,45," "79,50,291,44," "3*74\r\n");
    buf.push_back("$GLGSV,2,2,08,"  "80,12,334,39," "81,48,297,45," "82,09,264,32," "88,49,030,42," "3*75\r\n");
    buf.push_back("$GLGSV,1,1,01,"  "66,06,143,,"                                                   "0*48\r\n");
    // Galileo
    buf.push_back("$GAGSV,2,1,05,"  "13,73,014,47," "15,23,052,44," "21,69,230,45," "26,43,250,47," "2*79\r\n");
    buf.push_back("$GAGSV,2,2,05,"  "27,30,162,46,"                                                 "2*42\r\n");
    buf.push_back("$GAGSV,2,1,05,"  "13,73,014,45," "15,23,052,44," "21,69,230,46," "26,43,250,43," "7*79\r\n");
    buf.push_back("$GAGSV,2,2,05,"  "27,30,162,43,"                                                 "7*42\r\n");
    buf.push_back("$GAGSV,1,1,02,"  "03,05,063,,"   "08,04,109,,"                                   "0*71\r\n");
    // Beidou
    buf.push_back("$GBGSV,2,1,08,"  "19,27,131,44," "20,28,075,45," "24,22,309,44," "26,40,257,45," "1*75\r\n");
    buf.push_back("$GBGSV,2,2,08,"  "29,44,065,47," "35,65,315,48," "44,20,276,44," "57,,,45,"      "1*49\r\n");
    // QZSS & IRNSS
    buf.push_back("$GQGSV,1,1,00,"                                                                  "0*64\r\n");


    gps_sat_t sat = {};
    sat.numSats = 40;
    sat.timeOfWeekMs = 593477000;

    sat.sat[0].azim = 161;
    sat.sat[0].cno = ".\000\000";
    sat.sat[0].elev = 44;
    sat.sat[0].flags = 3;
    sat.sat[0].gnssId = 1;
    sat.sat[0].status = "\001\000\000";
    sat.sat[0].svId = 5;
    sat.sat[1].azim = 52;
    sat.sat[1].cno = ")\000\000";
    sat.sat[1].elev = 16;
    sat.sat[1].flags = 3;
    sat.sat[1].gnssId = 1;
    sat.sat[1].status = "\001\000\000";
    sat.sat[1].svId = 6;
    sat.sat[2].azim = 48;
    sat.sat[2].cno = "\000\000\000";
    sat.sat[2].elev = 1;
    sat.sat[2].flags = 3;
    sat.sat[2].gnssId = 1;
    sat.sat[2].status = "\000\000\000";
    sat.sat[2].svId = 9;
    sat.sat[3].azim = 43;
    sat.sat[3].cno = "/\000\000";
    sat.sat[3].elev = 59;
    sat.sat[3].flags = 3;
    sat.sat[3].gnssId = 1;
    sat.sat[3].status = "\001\000\000";
    sat.sat[3].svId = 11;
    sat.sat[4].azim = 188;
    sat.sat[4].cno = "/\000\000";
    sat.sat[4].elev = 67;
    sat.sat[4].flags = 3;
    sat.sat[4].gnssId = 1;
    sat.sat[4].status = "\001\000\000";
    sat.sat[4].svId = 12;
    sat.sat[5].azim = 97;
    sat.sat[5].cno = "\000\000\000";
    sat.sat[5].elev = 4;
    sat.sat[5].flags = 3;
    sat.sat[5].gnssId = 1;
    sat.sat[5].status = "\000\000\000";
    sat.sat[5].svId = 19;
    sat.sat[6].azim = 120;
    sat.sat[6].cno = ".\000\000";
    sat.sat[6].elev = 61;
    sat.sat[6].flags = 3;
    sat.sat[6].gnssId = 1;
    sat.sat[6].status = "\001\000\000";
    sat.sat[6].svId = 20;
    sat.sat[7].azim = 204;
    sat.sat[7].cno = "\000\000\000";
    sat.sat[7].elev = 0;
    sat.sat[7].flags = 3;
    sat.sat[7].gnssId = 1;
    sat.sat[7].status = "\000\000\000";
    sat.sat[7].svId = 24;
    sat.sat[8].azim = 283;
    sat.sat[8].cno = "/\000\000";
    sat.sat[8].elev = 61;
    sat.sat[8].flags = 3;
    sat.sat[8].gnssId = 1;
    sat.sat[8].status = "\001\000\000";
    sat.sat[8].svId = 25;
    sat.sat[9].azim = 305;
    sat.sat[9].cno = ")\000\000";
    sat.sat[9].elev = 11;
    sat.sat[9].flags = 3;
    sat.sat[9].gnssId = 1;
    sat.sat[9].status = "\001\000\000";
    sat.sat[9].svId = 28;
    sat.sat[10].azim = 300;
    sat.sat[10].cno = "/\000\000";
    sat.sat[10].elev = 36;
    sat.sat[10].flags = 3;
    sat.sat[10].gnssId = 1;
    sat.sat[10].status = "\001\000\000";
    sat.sat[10].svId = 29;
    sat.sat[11].azim = 328;
    sat.sat[11].cno = "\030\000\000";
    sat.sat[11].elev = 3;
    sat.sat[11].flags = 3;
    sat.sat[11].gnssId = 1;
    sat.sat[11].status = "\000\000\000";
    sat.sat[11].svId = 31;
    sat.sat[12].azim = 188;
    sat.sat[12].cno = "-\000\000";
    sat.sat[12].elev = 43;
    sat.sat[12].flags = 3;
    sat.sat[12].gnssId = 2;
    sat.sat[12].status = "\001\000\000";
    sat.sat[12].svId = 131;
    sat.sat[13].azim = 206;
    sat.sat[13].cno = ",\000\000";
    sat.sat[13].elev = 40;
    sat.sat[13].flags = 3;
    sat.sat[13].gnssId = 2;
    sat.sat[13].status = "\001\000\000";
    sat.sat[13].svId = 133;
    sat.sat[14].azim = 173;
    sat.sat[14].cno = "\000\000\000";
    sat.sat[14].elev = 43;
    sat.sat[14].flags = 3;
    sat.sat[14].gnssId = 2;
    sat.sat[14].status = "\000\000\000";
    sat.sat[14].svId = 138;
    sat.sat[15].azim = 63;
    sat.sat[15].cno = "\000\000\000";
    sat.sat[15].elev = 5;
    sat.sat[15].flags = 3;
    sat.sat[15].gnssId = 3;
    sat.sat[15].status = "\000\000\000";
    sat.sat[15].svId = 3;
    sat.sat[16].azim = 108;
    sat.sat[16].cno = "\000\000\000";
    sat.sat[16].elev = 5;
    sat.sat[16].flags = 3;
    sat.sat[16].gnssId = 3;
    sat.sat[16].status = "\001\000\000";
    sat.sat[16].svId = 8;
    sat.sat[17].azim = 16;
    sat.sat[17].cno = "-\000\000";
    sat.sat[17].elev = 72;
    sat.sat[17].flags = 3;
    sat.sat[17].gnssId = 3;
    sat.sat[17].status = "\001\000\000";
    sat.sat[17].svId = 13;
    sat.sat[18].azim = 53;
    sat.sat[18].cno = ",\000\000";
    sat.sat[18].elev = 22;
    sat.sat[18].flags = 3;
    sat.sat[18].gnssId = 3;
    sat.sat[18].status = "\001\000\000";
    sat.sat[18].svId = 15;
    sat.sat[19].azim = 228;
    sat.sat[19].cno = ".\000\000";
    sat.sat[19].elev = 69;
    sat.sat[19].flags = 3;
    sat.sat[19].gnssId = 3;
    sat.sat[19].status = "\001\000\000";
    sat.sat[19].svId = 21;
    sat.sat[20].azim = 252;
    sat.sat[20].cno = ",\000\000";
    sat.sat[20].elev = 44;
    sat.sat[20].flags = 3;
    sat.sat[20].gnssId = 3;
    sat.sat[20].status = "\001\000\000";
    sat.sat[20].svId = 26;
    sat.sat[21].azim = 163;
    sat.sat[21].cno = ",\000\000";
    sat.sat[21].elev = 28;
    sat.sat[21].flags = 3;
    sat.sat[21].gnssId = 3;
    sat.sat[21].status = "\001\000\000";
    sat.sat[21].svId = 27;
    sat.sat[22].azim = 130;
    sat.sat[22].cno = ",\000\000";
    sat.sat[22].elev = 28;
    sat.sat[22].flags = 3;
    sat.sat[22].gnssId = 4;
    sat.sat[22].status = "\001\000\000";
    sat.sat[22].svId = 19;
    sat.sat[23].azim = 73;
    sat.sat[23].cno = "/\000\000";
    sat.sat[23].elev = 28;
    sat.sat[23].flags = 3;
    sat.sat[23].gnssId = 4;
    sat.sat[23].status = "\001\000\000";
    sat.sat[23].svId = 20;
    sat.sat[24].azim = 308;
    sat.sat[24].cno = "-\000\000";
    sat.sat[24].elev = 22;
    sat.sat[24].flags = 3;
    sat.sat[24].gnssId = 4;
    sat.sat[24].status = "\001\000\000";
    sat.sat[24].svId = 24;
    sat.sat[25].azim = 255;
    sat.sat[25].cno = ".\000\000";
    sat.sat[25].elev = 40;
    sat.sat[25].flags = 3;
    sat.sat[25].gnssId = 4;
    sat.sat[25].status = "\001\000\000";
    sat.sat[25].svId = 26;
    sat.sat[26].azim = 66;
    sat.sat[26].cno = "/\000\000";
    sat.sat[26].elev = 43;
    sat.sat[26].flags = 3;
    sat.sat[26].gnssId = 4;
    sat.sat[26].status = "\001\000\000";
    sat.sat[26].svId = 29;
    sat.sat[27].azim = 318;
    sat.sat[27].cno = "0\000\000";
    sat.sat[27].elev = 66;
    sat.sat[27].flags = 3;
    sat.sat[27].gnssId = 4;
    sat.sat[27].status = "\001\000\000";
    sat.sat[27].svId = '#' 	35	0x23	unsigned char
    sat.sat[28].azim = 277;
    sat.sat[28].cno = "-\000\000";
    sat.sat[28].elev = 21;
    sat.sat[28].flags = 3;
    sat.sat[28].gnssId = 4;
    sat.sat[28].status = "\001\000\000";
    sat.sat[28].svId = 44;
    sat.sat[29].azim = 0;
    sat.sat[29].cno = ".\000\000";
    sat.sat[29].elev = 0xa5;
    sat.sat[29].flags = 3;
    sat.sat[29].gnssId = 4;
    sat.sat[29].status = "\001\000\000";
    sat.sat[29].svId = 57;
    sat.sat[30].azim = 92;
    sat.sat[30].cno = "0\000\000";
    sat.sat[30].elev = 27    	0x1b	char
    sat.sat[30].flags = 3;
    sat.sat[30].gnssId = 6;
    sat.sat[30].status = "\001\000\000";
    sat.sat[30].svId = 1;
    sat.sat[31].azim = 142;
    sat.sat[31].cno = "\000\000\000";
    sat.sat[31].elev = 7;
    sat.sat[31].flags = 3;
    sat.sat[31].gnssId = 6;
    sat.sat[31].status = "\001\000\000";
    sat.sat[31].svId = 2;
    sat.sat[32].azim = 38;
    sat.sat[32].cno = "2\000\000";
    sat.sat[32].elev = 17;
    sat.sat[32].flags = 3;
    sat.sat[32].gnssId = 6;
    sat.sat[32].status = "\001\000\000";
    sat.sat[32].svId = 8;
    sat.sat[33].azim = 207;
    sat.sat[33].cno = "1\000\000";
    sat.sat[33].elev = 40;
    sat.sat[33].flags = 3;
    sat.sat[33].gnssId = 6;
    sat.sat[33].status = "\001\000\000";
    sat.sat[33].svId = 14;
    sat.sat[34].azim = 290;
    sat.sat[34].cno = "0\000\000";
    sat.sat[34].elev = 50;
    sat.sat[34].flags = 3;
    sat.sat[34].gnssId = 6;
    sat.sat[34].status = "\001\000\000";
    sat.sat[34].svId = 15;
    sat.sat[35].azim = 334;
    sat.sat[35].cno = "-\000\000";
    sat.sat[35].elev = 12;
    sat.sat[35].flags = 3;
    sat.sat[35].gnssId = 6;
    sat.sat[35].status = "\001\000\000";
    sat.sat[35].svId = 16;
    sat.sat[36].azim = 298;
    sat.sat[36].cno = "0\000\000";
    sat.sat[36].elev = 49;
    sat.sat[36].flags = 3;
    sat.sat[36].gnssId = 6;
    sat.sat[36].status = "\001\000\000";
    sat.sat[36].svId = 17;
    sat.sat[37].azim = 264;
    sat.sat[37].cno = "!\000\000";
    sat.sat[37].elev = 10;
    sat.sat[37].flags = 3;
    sat.sat[37].gnssId = 6;
    sat.sat[37].status = "\001\000\000";
    sat.sat[37].svId = 18;
    sat.sat[38].azim = 63;
    sat.sat[38].cno = "\026\000\000";
    sat.sat[38].elev = 7;
    sat.sat[38].flags = 3;
    sat.sat[38].gnssId = 6;
    sat.sat[38].status = "\001\000\000";
    sat.sat[38].svId = 23;
    sat.sat[39].azim = 31;
    sat.sat[39].cno = "3\000\000";
    sat.sat[39].elev = 48;
    sat.sat[39].flags = 3;
    sat.sat[39].gnssId = 6;
    sat.sat[39].status = "\001\000\000";
    sat.sat[39].svId = 24;


#define ASCII_BUF2  1024
    char ascii_buf[ASCII_BUF2] = { 0 };

    did_gps_sat_to_nmea_gsv(ascii_buf, ASCII_BUF2, sat);
    printf("%s\n", ascii_buf);

    // for(int i=0; i<buf.size(); i++)
    // {
    //     printf("%s", buf[i].c_str());

    // }

}



