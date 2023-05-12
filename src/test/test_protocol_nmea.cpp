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

#define ASCII_BUF2  2048
char g_ascii_buf[ASCII_BUF2] = { 0 };

TEST(protocol_nmea, GSV)
{
    string buf;

    // GPS & SBAS        #msgs,msg#,numSV,  svid,elv,azm,cno, ..., signalId*checksum
    buf += "$GPGSV,3,1,12," "02,27,302,31," "10,36,250,43," "13,24,047,42," "15,49,069,43," "1*65\r\n";
    buf += "$GPGSV,3,2,12," "16,19,283,34," "18,73,022,45," "23,70,277,41," "26,11,251,35," "1*6E\r\n";
    buf += "$GPGSV,3,3,12," "27,22,316,42," "29,28,159,39," "44,43,188,43," "46,40,206,43," "1*62\r\n";
    buf += "$GPGSV,2,1,07," "10,36,250,30," "15,49,069,29," "18,73,022,32," "23,70,277,28," "6*68\r\n";
    buf += "$GPGSV,2,2,07," "26,11,251,15," "27,22,316,27," "29,28,159,24,"                 "6*5C\r\n";
    buf += "$GPGSV,1,1,02," "05,05,059,,"   "24,05,123,,"                                   "0*68\r\n";
    // Galileo
    buf += "$GAGSV,2,1,08," "02,04,215,24," "05,52,157,28," "09,47,066,27," "15,12,299,24," "2*7C\r\n";
    buf += "$GAGSV,2,2,08," "27,09,314,17," "30,15,263,25," "34,60,324,29," "36,55,087,28," "2*7A\r\n";
    buf += "$GAGSV,2,1,08," "02,04,215,34," "05,52,157,41," "09,47,066,44," "15,12,299,35," "7*72\r\n";
    buf += "$GAGSV,2,2,08," "27,09,314,30," "30,15,263,25," "34,60,324,39," "36,55,087,44," "7*71\r\n";
    buf += "$GAGSV,1,1,02," "04,05,035,,"   "11,09,107,,"                                   "0*7E\r\n";
    // Beidou
    buf += "$GBGSV,2,1,08," "11,21,134,37," "14,66,054,44," "28,69,308,43," "33,82,187,44," "1*75\r\n";
    buf += "$GBGSV,2,2,08," "41,30,220,39," "42,38,048,43," "43,47,140,40," "58,,,42,"      "1*4A\r\n";
    buf += "$GBGSV,1,1,02," "11,21,134,28," "14,66,054,32,"                                 "B*0D\r\n";
    buf += "$GBGSV,1,1,02," "27,19,313,,"   "34,02,136,,"                                   "0*78\r\n";
    // QZSS
    buf += "$GQGSV,1,1,01," "02,04,308,33,"                                                 "1*59\r\n";
    buf += "$GQGSV,1,1,01," "02,04,308,23,"                                                 "6*5F\r\n";
    // GLONASS
    buf += "$GLGSV,2,1,08," "65,65,201,33," "66,11,208,27," "72,55,038,29," "79,03,312,21," "1*74\r\n";
    buf += "$GLGSV,2,2,08," "81,05,317,31," "86,09,125,26," "87,61,098,38," "88,54,335,37," "1*7C\r\n";
    buf += "$GLGSV,1,1,01," "88,54,335,37,"                                                 "3*4B\r\n";
    buf += "$GLGSV,1,1,02," "71,07,032,,"   "80,00,359,,"                                   "0*7C\r\n";



    // GPS & SBAS        #msgs,msg#,numSV,  svid,elv,azm,cno, ..., signalId*checksum
    buf += "$GPGSV,4,1,14,02,40,310,43,08,07,324,31,10,48,267,45,15,37,053,45,1*67\r\n"
    buf += "$GPGSV,4,2,14,16,12,268,35,18,69,078,41,23,74,336,40,24,15,111,37,1*62\r\n"
    buf += "$GPGSV,4,3,14,26,02,239,31,27,35,307,38,29,12,162,37,32,14,199,39,1*60\r\n"
    buf += "$GPGSV,4,4,14,44,43,188,43,46,40,206,43,1*65\r\n"
    buf += "$GPGSV,3,1,09,10,48,267,27,15,37,053,26,18,69,078,34,23,74,336,34,6*6C\r\n"
    buf += "$GPGSV,3,2,09,24,15,111,25,26,02,239,18,27,35,307,27,29,12,162,21,6*64\r\n"
    buf += "$GPGSV,3,3,09,32,14,199,25,6*58\r\n"
    buf += "$GLGSV,2,1,06,65,85,260,33,66,28,217,30,72,36,034,35,81,20,324,33,1*75\r\n"
    buf += "$GLGSV,2,2,06,87,47,127,35,88,73,350,34,1*75\r\n"
    buf += "$GLGSV,1,1,01,87,47,127,20,3*41\r\n"
    buf += "$GLGSV,1,1,01,80,01,343,,0*45\r\n"
    buf += "$GAGSV,1,1,04,05,65,144,30,09,39,052,30,34,71,341,27,36,46,105,30,2*73\r\n"
    buf += "$GAGSV,1,1,04,05,65,144,41,09,39,052,43,34,71,341,42,36,46,105,39,7*7E\r\n"
    buf += "$GBGSV,2,1,08,11,09,141,34,14,52,047,44,27,32,313,43,28,80,263,44,1*71\r\n"
    buf += "$GBGSV,2,2,08,33,81,039,43,41,43,230,42,43,33,148,42,58,,,44,1*4E\r\n"
    buf += "$GBGSV,1,1,02,11,09,141,16,14,52,047,32,B*0D\r\n"
    buf += "$GQGSV,1,1,01,02,,,30,1*65\r\n"


    gps_sat_t sat = {};
    sat.numSats = 31;
    sat.timeOfWeekMs = 436693200;


    sat.sat[0].azim = 310;
    sat.sat[0].cno = 35;
    sat.sat[0].elev = 40;
    sat.sat[0].gnssId = 1;
    sat.sat[0].status = 95;
    sat.sat[0].svId = 2;
    sat.sat[1].azim = 266;
    sat.sat[1].cno = 45;
    sat.sat[1].elev = 48;
    sat.sat[1].gnssId = 1;
    sat.sat[1].status = 31;
    sat.sat[1].svId = 10;
    sat.sat[2].azim = 54;
    sat.sat[2].cno = 45;
    sat.sat[2].elev = 38;
    sat.sat[2].gnssId = 1;
    sat.sat[2].status = 31;
    sat.sat[2].svId = 15;
    sat.sat[3].azim = 269;
    sat.sat[3].cno = 37;
    sat.sat[3].elev = 13;
    sat.sat[3].gnssId = 1;
    sat.sat[3].status = 95;
    sat.sat[3].svId = 16;
    sat.sat[4].azim = 76;
    sat.sat[4].cno = 41;
    sat.sat[4].elev = 69;
    sat.sat[4].gnssId = 1;
    sat.sat[4].status = 31;
    sat.sat[4].svId = 18;
    sat.sat[5].azim = 333;
    sat.sat[5].cno = 41;
    sat.sat[5].elev = 74;
    sat.sat[5].gnssId = 1;
    sat.sat[5].status = 31;
    sat.sat[5].svId = 23;
    sat.sat[6].azim = 307;
    sat.sat[6].cno = 41;
    sat.sat[6].elev = 35;
    sat.sat[6].gnssId = 1;
    sat.sat[6].status = 95;
    sat.sat[6].svId = 27;
    sat.sat[7].azim = 162;
    sat.sat[7].cno = 36;
    sat.sat[7].elev = 13;
    sat.sat[7].gnssId = 1;
    sat.sat[7].status = 31;
    sat.sat[7].svId = 29;
    sat.sat[8].azim = 199;
    sat.sat[8].cno = 40;
    sat.sat[8].elev = 13;
    sat.sat[8].gnssId = 1;
    sat.sat[8].status = 95;
    sat.sat[8].svId = 32;
    sat.sat[9].azim = 188;
    sat.sat[9].cno = 43;
    sat.sat[9].elev = 43;
    sat.sat[9].gnssId = 2;
    sat.sat[9].status = 95;
    sat.sat[9].svId = 131;
    sat.sat[10].azim = 206;
    sat.sat[10].cno = 43;
    sat.sat[10].elev = 40;
    sat.sat[10].gnssId = 2;
    sat.sat[10].status = 95;
    sat.sat[10].svId = 133;
    sat.sat[11].azim = 173;
    sat.sat[11].cno = 0;
    sat.sat[11].elev = 43;
    sat.sat[11].gnssId = 2;
    sat.sat[11].status = 1;
    sat.sat[11].svId = 138;
    sat.sat[12].azim = 145;
    sat.sat[12].cno = 41;
    sat.sat[12].elev = 65;
    sat.sat[12].gnssId = 3;
    sat.sat[12].status = 31;
    sat.sat[12].svId = 5;
    sat.sat[13].azim = 53;
    sat.sat[13].cno = 43;
    sat.sat[13].elev = 39;
    sat.sat[13].gnssId = 3;
    sat.sat[13].status = 31;
    sat.sat[13].svId = 9;
    sat.sat[14].azim = 340;
    sat.sat[14].cno = 42;
    sat.sat[14].elev = 71;
    sat.sat[14].gnssId = 3;
    sat.sat[14].status = 31;
    sat.sat[14].svId = 34;
    sat.sat[15].azim = 104;
    sat.sat[15].cno = 40;
    sat.sat[15].elev = 47;
    sat.sat[15].gnssId = 3;
    sat.sat[15].status = 31;
    sat.sat[15].svId = 36;
    sat.sat[16].azim = 141;
    sat.sat[16].cno = 36;
    sat.sat[16].elev = 9;
    sat.sat[16].gnssId = 4;
    sat.sat[16].status = 23;
    sat.sat[16].svId = 11;
    sat.sat[17].azim = 47;
    sat.sat[17].cno = 44;
    sat.sat[17].elev = 52;
    sat.sat[17].gnssId = 4;
    sat.sat[17].status = 31;
    sat.sat[17].svId = 14;
    sat.sat[18].azim = 313;
    sat.sat[18].cno = 38;
    sat.sat[18].elev = 31;
    sat.sat[18].gnssId = 4;
    sat.sat[18].status = 31;
    sat.sat[18].svId = 27;
    sat.sat[19].azim = 267;
    sat.sat[19].cno = 44;
    sat.sat[19].elev = 79;
    sat.sat[19].gnssId = 4;
    sat.sat[19].status = 31;
    sat.sat[19].svId = 28;
    sat.sat[20].azim = 40;
    sat.sat[20].cno = 42;
    sat.sat[20].elev = 82;
    sat.sat[20].gnssId = 4;
    sat.sat[20].status = 31;
    sat.sat[20].svId = 33;
    sat.sat[21].azim = 230;
    sat.sat[21].cno = 43;
    sat.sat[21].elev = 43;
    sat.sat[21].gnssId = 4;
    sat.sat[21].status = 31;
    sat.sat[21].svId = 41;
    sat.sat[22].azim = 148;
    sat.sat[22].cno = 43;
    sat.sat[22].elev = 34;
    sat.sat[22].gnssId = 4;
    sat.sat[22].status = 31;
    sat.sat[22].svId = 43;
    sat.sat[23].azim = 0;
    sat.sat[23].cno = 43;
    sat.sat[23].elev = 0;
    sat.sat[23].gnssId = 4;
    sat.sat[23].status = 39;
    sat.sat[23].svId = 58;
    sat.sat[24].azim = 252;
    sat.sat[24].cno = 31;
    sat.sat[24].elev = 85;
    sat.sat[24].gnssId = 6;
    sat.sat[24].status = 23;
    sat.sat[24].svId = 1;
    sat.sat[25].azim = 217;
    sat.sat[25].cno = 29;
    sat.sat[25].elev = 28;
    sat.sat[25].gnssId = 6;
    sat.sat[25].status = 31;
    sat.sat[25].svId = 2;
    sat.sat[26].azim = 34;
    sat.sat[26].cno = 35;
    sat.sat[26].elev = 37;
    sat.sat[26].gnssId = 6;
    sat.sat[26].status = 31;
    sat.sat[26].svId = 8;
    sat.sat[27].azim = 344;
    sat.sat[27].cno = 0;
    sat.sat[27].elev = 1;
    sat.sat[27].gnssId = 6;
    sat.sat[27].status = 17;
    sat.sat[27].svId = 16;
    sat.sat[28].azim = 324;
    sat.sat[28].cno = 19;
    sat.sat[28].elev = 19;
    sat.sat[28].gnssId = 6;
    sat.sat[28].status = 19;
    sat.sat[28].svId = 17;
    sat.sat[29].azim = 126;
    sat.sat[29].cno = 36;
    sat.sat[29].elev = 48;
    sat.sat[29].gnssId = 6;
    sat.sat[29].status = 31;
    sat.sat[29].svId = 23;
    sat.sat[30].azim = 349;
    sat.sat[30].cno = 32;
    sat.sat[30].elev = 72;
    sat.sat[30].gnssId = 6;
    sat.sat[30].status = 28;
    sat.sat[30].svId = 24;
    sat.sat[31].azim = 374;
    sat.sat[31].cno = 0;
    sat.sat[31].elev = 0;
    sat.sat[31].gnssId = 40;
    sat.sat[31].status = 0;
    sat.sat[31].svId = 243;
    sat.sat[32].azim = 374;
    sat.sat[32].cno = 0;
    sat.sat[32].elev = 0;
    sat.sat[32].gnssId = 0;
    sat.sat[32].status = 0;
    sat.sat[32].svId = 239;
    sat.sat[33].azim = 0;
    sat.sat[33].cno = 0;
    sat.sat[33].elev = 0;
    sat.sat[33].gnssId = 0;
    sat.sat[33].status = 0;
    sat.sat[33].svId = 0;
    sat.sat[34].azim = 0;
    sat.sat[34].cno = 1;
    sat.sat[34].elev = 1;
    sat.sat[34].gnssId = 4;
    sat.sat[34].status = 0;
    sat.sat[34].svId = 0;
    sat.sat[35].azim = 0;
    sat.sat[35].cno = 2;
    sat.sat[35].elev = 0;
    sat.sat[35].gnssId = 0;
    sat.sat[35].status = 0;
    sat.sat[35].svId = 0;
    sat.sat[36].azim = 3;
    sat.sat[36].cno = 0;
    sat.sat[36].elev = 4;
    sat.sat[36].gnssId = 7;
    sat.sat[36].status = 0;
    sat.sat[36].svId = 0;
    sat.sat[37].azim = 0;
    sat.sat[37].cno = 0;
    sat.sat[37].elev = 0;
    sat.sat[37].gnssId = 72;
    sat.sat[37].status = 0;
    sat.sat[37].svId = 0;
    sat.sat[38].azim = 0;
    sat.sat[38].cno = 1;
    sat.sat[38].elev = 0;
    sat.sat[38].gnssId = 1;
    sat.sat[38].status = 0;
    sat.sat[38].svId = 0;
    sat.sat[39].azim = 0;
    sat.sat[39].cno = 0;
    sat.sat[39].elev = 0;
    sat.sat[39].gnssId = 0;
    sat.sat[39].status = 0;
    sat.sat[39].svId = 0;
    sat.sat[40].azim = 0;
    sat.sat[40].cno = 0;
    sat.sat[40].elev = 0;
    sat.sat[40].gnssId = 0;
    sat.sat[40].status = 0;
    sat.sat[40].svId = 0;
    sat.sat[41].azim = 0;
    sat.sat[41].cno = 0;
    sat.sat[41].elev = 0;
    sat.sat[41].gnssId = 82;
    sat.sat[41].status = 0;
    sat.sat[41].svId = 0;
    sat.sat[42].azim = 0;
    sat.sat[42].cno = 1;
    sat.sat[42].elev = 18;
    sat.sat[42].gnssId = 160;
    sat.sat[42].status = 0;
    sat.sat[42].svId = 234;
    sat.sat[43].azim = 0;
    sat.sat[43].cno = 0;
    sat.sat[43].elev = 0;
    sat.sat[43].gnssId = 4;
    sat.sat[43].status = 0;
    sat.sat[43].svId = 0;
    sat.sat[44].azim = 0;
    sat.sat[44].cno = 0;
    sat.sat[44].elev = 0;
    sat.sat[44].gnssId = 1;
    sat.sat[44].status = 0;
    sat.sat[44].svId = 0;
    sat.sat[45].azim = 0;
    sat.sat[45].cno = 0;
    sat.sat[45].elev = 0;
    sat.sat[45].gnssId = 82;
    sat.sat[45].status = 0;
    sat.sat[45].svId = 0;
    sat.sat[46].azim = 0;
    sat.sat[46].cno = 127;
    sat.sat[46].elev = 0;
    sat.sat[46].gnssId = 42;
    sat.sat[46].status = 0;
    sat.sat[46].svId = 66;
    sat.sat[47].azim = 0;
    sat.sat[47].cno = 1;
    sat.sat[47].elev = 107;
    sat.sat[47].gnssId = 232;
    sat.sat[47].status = 0;
    sat.sat[47].svId = 2;
    sat.sat[48].azim = 374;
    sat.sat[48].cno = 0;
    sat.sat[48].elev = 0;
    sat.sat[48].gnssId = 153;
    sat.sat[48].status = 0;
    sat.sat[48].svId = 242;
    sat.sat[49].azim = 0;
    sat.sat[49].cno = 1;
    sat.sat[49].elev = 107;
    sat.sat[49].gnssId = 0;
    sat.sat[49].status = 0;
    sat.sat[49].svId = 0;



    gps_sig_t sig = {};
    sig.numSigs = 58;
    sig.timeOfWeekMs = 436693200;

    sat.sat[0].cno = 35;
    sat.sat[0].gnssId = 1;
    sat.sat[0].quality = 7;
    sat.sat[0].sigId = 0;
    sat.sat[0].status = 361;
    sat.sat[0].svId = 2;
    sat.sat[1].cno = 0;
    sat.sat[1].gnssId = 1;
    sat.sat[1].quality = 1;
    sat.sat[1].sigId = 4;
    sat.sat[1].status = 1;
    sat.sat[1].svId = 2;
    sat.sat[2].cno = 45;
    sat.sat[2].gnssId = 1;
    sat.sat[2].quality = 7;
    sat.sat[2].sigId = 0;
    sat.sat[2].status = 41;
    sat.sat[2].svId = 10;
    sat.sat[3].cno = 25;
    sat.sat[3].gnssId = 1;
    sat.sat[3].quality = 4;
    sat.sat[3].sigId = 3;
    sat.sat[3].status = 41;
    sat.sat[3].svId = 10;
    sat.sat[4].cno = 45;
    sat.sat[4].gnssId = 1;
    sat.sat[4].quality = 7;
    sat.sat[4].sigId = 0;
    sat.sat[4].status = 41;
    sat.sat[4].svId = 15;
    sat.sat[5].cno = 27;
    sat.sat[5].gnssId = 1;
    sat.sat[5].quality = 7;
    sat.sat[5].sigId = 3;
    sat.sat[5].status = 41;
    sat.sat[5].svId = 15;
    sat.sat[6].cno = 37;
    sat.sat[6].gnssId = 1;
    sat.sat[6].quality = 7;
    sat.sat[6].sigId = 0;
    sat.sat[6].status = 361;
    sat.sat[6].svId = 16;
    sat.sat[7].cno = 0;
    sat.sat[7].gnssId = 1;
    sat.sat[7].quality = 1;
    sat.sat[7].sigId = 4;
    sat.sat[7].status = 1;
    sat.sat[7].svId = 16;
    sat.sat[8].cno = 41;
    sat.sat[8].gnssId = 1;
    sat.sat[8].quality = 7;
    sat.sat[8].sigId = 0;
    sat.sat[8].status = 41;
    sat.sat[8].svId = 18;
    sat.sat[9].cno = 33;
    sat.sat[9].gnssId = 1;
    sat.sat[9].quality = 7;
    sat.sat[9].sigId = 3;
    sat.sat[9].status = 41;
    sat.sat[9].svId = 18;
    sat.sat[10].cno = 41;
    sat.sat[10].gnssId = 1;
    sat.sat[10].quality = 7;
    sat.sat[10].sigId = 0;
    sat.sat[10].status = 41;
    sat.sat[10].svId = 23;
    sat.sat[11].cno = 34;
    sat.sat[11].gnssId = 1;
    sat.sat[11].quality = 7;
    sat.sat[11].sigId = 3;
    sat.sat[11].status = 41;
    sat.sat[11].svId = 23;
    sat.sat[12].cno = 41;
    sat.sat[12].gnssId = 1;
    sat.sat[12].quality = 7;
    sat.sat[12].sigId = 0;
    sat.sat[12].status = 361;
    sat.sat[12].svId = 27;
    sat.sat[13].cno = 23;
    sat.sat[13].gnssId = 1;
    sat.sat[13].quality = 4;
    sat.sat[13].sigId = 3;
    sat.sat[13].status = 9;
    sat.sat[13].svId = 27;
    sat.sat[14].cno = 36;
    sat.sat[14].gnssId = 1;
    sat.sat[14].quality = 7;
    sat.sat[14].sigId = 0;
    sat.sat[14].status = 41;
    sat.sat[14].svId = 29;
    sat.sat[15].cno = 25;
    sat.sat[15].gnssId = 1;
    sat.sat[15].quality = 4;
    sat.sat[15].sigId = 3;
    sat.sat[15].status = 41;
    sat.sat[15].svId = 29;
    sat.sat[16].cno = 40;
    sat.sat[16].gnssId = 1;
    sat.sat[16].quality = 7;
    sat.sat[16].sigId = 0;
    sat.sat[16].status = 361;
    sat.sat[16].svId = 32;
    sat.sat[17].cno = 28;
    sat.sat[17].gnssId = 1;
    sat.sat[17].quality = 7;
    sat.sat[17].sigId = 3;
    sat.sat[17].status = 41;
    sat.sat[17].svId = 32;
    sat.sat[18].cno = 43;
    sat.sat[18].gnssId = 2;
    sat.sat[18].quality = 7;
    sat.sat[18].sigId = 0;
    sat.sat[18].status = 361;
    sat.sat[18].svId = 131;
    sat.sat[19].cno = 43;
    sat.sat[19].gnssId = 2;
    sat.sat[19].quality = 7;
    sat.sat[19].sigId = 0;
    sat.sat[19].status = 361;
    sat.sat[19].svId = 133;
    sat.sat[20].cno = 0;
    sat.sat[20].gnssId = 2;
    sat.sat[20].quality = 1;
    sat.sat[20].sigId = 0;
    sat.sat[20].status = 0;
    sat.sat[20].svId = 138;
    sat.sat[21].cno = 41;
    sat.sat[21].gnssId = 3;
    sat.sat[21].quality = 7;
    sat.sat[21].sigId = 0;
    sat.sat[21].status = 41;
    sat.sat[21].svId = 5;
    sat.sat[22].cno = 30;
    sat.sat[22].gnssId = 3;
    sat.sat[22].quality = 7;
    sat.sat[22].sigId = 6;
    sat.sat[22].status = 41;
    sat.sat[22].svId = 5;
    sat.sat[23].cno = 43;
    sat.sat[23].gnssId = 3;
    sat.sat[23].quality = 7;
    sat.sat[23].sigId = 0;
    sat.sat[23].status = 41;
    sat.sat[23].svId = 9;
    sat.sat[24].cno = 30;
    sat.sat[24].gnssId = 3;
    sat.sat[24].quality = 7;
    sat.sat[24].sigId = 6;
    sat.sat[24].status = 41;
    sat.sat[24].svId = 9;
    sat.sat[25].cno = 42;
    sat.sat[25].gnssId = 3;
    sat.sat[25].quality = 7;
    sat.sat[25].sigId = 0;
    sat.sat[25].status = 41;
    sat.sat[25].svId = 34;
    sat.sat[26].cno = 26;
    sat.sat[26].gnssId = 3;
    sat.sat[26].quality = 7;
    sat.sat[26].sigId = 6;
    sat.sat[26].status = 41;
    sat.sat[26].svId = 34;
    sat.sat[27].cno = 40;
    sat.sat[27].gnssId = 3;
    sat.sat[27].quality = 7;
    sat.sat[27].sigId = 0;
    sat.sat[27].status = 41;
    sat.sat[27].svId = 36;
    sat.sat[28].cno = 31;
    sat.sat[28].gnssId = 3;
    sat.sat[28].quality = 7;
    sat.sat[28].sigId = 6;
    sat.sat[28].status = 41;
    sat.sat[28].svId = 36;
    sat.sat[29].cno = 36;
    sat.sat[29].gnssId = 4;
    sat.sat[29].quality = 7;
    sat.sat[29].sigId = 0;
    sat.sat[29].status = 1;
    sat.sat[29].svId = 11;
    sat.sat[30].cno = 21;
    sat.sat[30].gnssId = 4;
    sat.sat[30].quality = 4;
    sat.sat[30].sigId = 2;
    sat.sat[30].status = 1;
    sat.sat[30].svId = 11;
    sat.sat[31].cno = 44;
    sat.sat[31].gnssId = 4;
    sat.sat[31].quality = 7;
    sat.sat[31].sigId = 0;
    sat.sat[31].status = 41;
    sat.sat[31].svId = 14;
    sat.sat[32].cno = 30;
    sat.sat[32].gnssId = 4;
    sat.sat[32].quality = 7;
    sat.sat[32].sigId = 2;
    sat.sat[32].status = 41;
    sat.sat[32].svId = 14;
    sat.sat[33].cno = 38;
    sat.sat[33].gnssId = 4;
    sat.sat[33].quality = 7;
    sat.sat[33].sigId = 0;
    sat.sat[33].status = 41;
    sat.sat[33].svId = 27;
    sat.sat[34].cno = 0;
    sat.sat[34].gnssId = 4;
    sat.sat[34].quality = 1;
    sat.sat[34].sigId = 2;
    sat.sat[34].status = 1;
    sat.sat[34].svId = 27;
    sat.sat[35].cno = 44;
    sat.sat[35].gnssId = 4;
    sat.sat[35].quality = 7;
    sat.sat[35].sigId = 0;
    sat.sat[35].status = 41;
    sat.sat[35].svId = 28;
    sat.sat[36].cno = 0;
    sat.sat[36].gnssId = 4;
    sat.sat[36].quality = 1;
    sat.sat[36].sigId = 2;
    sat.sat[36].status = 1;
    sat.sat[36].svId = 28;
    sat.sat[37].cno = 42;
    sat.sat[37].gnssId = 4;
    sat.sat[37].quality = 7;
    sat.sat[37].sigId = 0;
    sat.sat[37].status = 41;
    sat.sat[37].svId = 33;
    sat.sat[38].cno = 0;
    sat.sat[38].gnssId = 4;
    sat.sat[38].quality = 1;
    sat.sat[38].sigId = 2;
    sat.sat[38].status = 1;
    sat.sat[38].svId = 33;
    sat.sat[39].cno = 43;
    sat.sat[39].gnssId = 4;
    sat.sat[39].quality = 7;
    sat.sat[39].sigId = 0;
    sat.sat[39].status = 41;
    sat.sat[39].svId = 41;
    sat.sat[40].cno = 0;
    sat.sat[40].gnssId = 4;
    sat.sat[40].quality = 1;
    sat.sat[40].sigId = 2;
    sat.sat[40].status = 1;
    sat.sat[40].svId = 41;
    sat.sat[41].cno = 43;
    sat.sat[41].gnssId = 4;
    sat.sat[41].quality = 7;
    sat.sat[41].sigId = 0;
    sat.sat[41].status = 41;
    sat.sat[41].svId = 43;
    sat.sat[42].cno = 0;
    sat.sat[42].gnssId = 4;
    sat.sat[42].quality = 1;
    sat.sat[42].sigId = 2;
    sat.sat[42].status = 1;
    sat.sat[42].svId = 43;
    sat.sat[43].cno = 43;
    sat.sat[43].gnssId = 4;
    sat.sat[43].quality = 7;
    sat.sat[43].sigId = 0;
    sat.sat[43].status = 2;
    sat.sat[43].svId = 58;
    sat.sat[44].cno = 31;
    sat.sat[44].gnssId = 6;
    sat.sat[44].quality = 7;
    sat.sat[44].sigId = 0;
    sat.sat[44].status = 1;
    sat.sat[44].svId = 1;
    sat.sat[45].cno = 0;
    sat.sat[45].gnssId = 6;
    sat.sat[45].quality = 1;
    sat.sat[45].sigId = 2;
    sat.sat[45].status = 1;
    sat.sat[45].svId = 1;
    sat.sat[46].cno = 29;
    sat.sat[46].gnssId = 6;
    sat.sat[46].quality = 7;
    sat.sat[46].sigId = 0;
    sat.sat[46].status = 41;
    sat.sat[46].svId = 2;
    sat.sat[47].cno = 0;
    sat.sat[47].gnssId = 6;
    sat.sat[47].quality = 1;
    sat.sat[47].sigId = 2;
    sat.sat[47].status = 1;
    sat.sat[47].svId = 2;
    sat.sat[48].cno = 35;
    sat.sat[48].gnssId = 6;
    sat.sat[48].quality = 7;
    sat.sat[48].sigId = 0;
    sat.sat[48].status = 41;
    sat.sat[48].svId = 8;
    sat.sat[49].cno = 0;
    sat.sat[49].gnssId = 6;
    sat.sat[49].quality = 1;
    sat.sat[49].sigId = 2;
    sat.sat[49].status = 1;
    sat.sat[49].svId = 8;
    sat.sat[50].cno = 0;
    sat.sat[50].gnssId = 6;
    sat.sat[50].quality = 1;
    sat.sat[50].sigId = 0;
    sat.sat[50].status = 1;
    sat.sat[50].svId = 16;
    sat.sat[51].cno = 0;
    sat.sat[51].gnssId = 6;
    sat.sat[51].quality = 1;
    sat.sat[51].sigId = 2;
    sat.sat[51].status = 1;
    sat.sat[51].svId = 16;
    sat.sat[52].cno = 19;
    sat.sat[52].gnssId = 6;
    sat.sat[52].quality = 3;
    sat.sat[52].sigId = 0;
    sat.sat[52].status = 1;
    sat.sat[52].svId = 17;
    sat.sat[53].cno = 0;
    sat.sat[53].gnssId = 6;
    sat.sat[53].quality = 1;
    sat.sat[53].sigId = 2;
    sat.sat[53].status = 1;
    sat.sat[53].svId = 17;
    sat.sat[54].cno = 36;
    sat.sat[54].gnssId = 6;
    sat.sat[54].quality = 7;
    sat.sat[54].sigId = 0;
    sat.sat[54].status = 41;
    sat.sat[54].svId = 23;
    sat.sat[55].cno = 0;
    sat.sat[55].gnssId = 6;
    sat.sat[55].quality = 1;
    sat.sat[55].sigId = 2;
    sat.sat[55].status = 1;
    sat.sat[55].svId = 23;
    sat.sat[56].cno = 32;
    sat.sat[56].gnssId = 6;
    sat.sat[56].quality = 4;
    sat.sat[56].sigId = 0;
    sat.sat[56].status = 41;
    sat.sat[56].svId = 24;
    sat.sat[57].cno = 0;
    sat.sat[57].gnssId = 6;
    sat.sat[57].quality = 1;
    sat.sat[57].sigId = 2;
    sat.sat[57].status = 1;
    sat.sat[57].svId = 24;
    sat.sat[58].cno = 0;
    sat.sat[58].gnssId = 0;
    sat.sat[58].quality = 0;
    sat.sat[58].sigId = 240;
    sat.sat[58].status = 0;
    sat.sat[58].svId = 0;
    sat.sat[59].cno = 208;
    sat.sat[59].gnssId = 0;
    sat.sat[59].quality = 233;
    sat.sat[59].sigId = 0;
    sat.sat[59].status = 8978;
    sat.sat[59].svId = 0;
    sat.sat[60].cno = 0;
    sat.sat[60].gnssId = 133;
    sat.sat[60].quality = 40;
    sat.sat[60].sigId = 0;
    sat.sat[60].status = 27395;
    sat.sat[60].svId = 1;
    sat.sat[61].cno = 0;
    sat.sat[61].gnssId = 33;
    sat.sat[61].quality = 0;
    sat.sat[61].sigId = 1;
    sat.sat[61].status = 336;
    sat.sat[61].svId = 133;
    sat.sat[62].cno = 1;
    sat.sat[62].gnssId = 107;
    sat.sat[62].quality = 0;
    sat.sat[62].sigId = 133;
    sat.sat[62].status = 256;
    sat.sat[62].svId = 33;
    sat.sat[63].cno = 0;
    sat.sat[63].gnssId = 0;
    sat.sat[63].quality = 0;
    sat.sat[63].sigId = 0;
    sat.sat[63].status = 0;
    sat.sat[63].svId = 0;
    sat.sat[64].cno = 0;
    sat.sat[64].gnssId = 128;
    sat.sat[64].quality = 0;
    sat.sat[64].sigId = 0;
    sat.sat[64].status = 0;
    sat.sat[64].svId = 4;
    sat.sat[65].cno = 15;
    sat.sat[65].gnssId = 0;
    sat.sat[65].quality = 35;
    sat.sat[65].sigId = 0;
    sat.sat[65].status = 389;
    sat.sat[65].svId = 0;
    sat.sat[66].cno = 236;
    sat.sat[66].gnssId = 0;
    sat.sat[66].quality = 18;
    sat.sat[66].sigId = 0;
    sat.sat[66].status = 34083;
    sat.sat[66].svId = 0;
    sat.sat[67].cno = 112;
    sat.sat[67].gnssId = 1;
    sat.sat[67].quality = 0;
    sat.sat[67].sigId = 0;
    sat.sat[67].status = 0;
    sat.sat[67].svId = 0;
    sat.sat[68].cno = 0;
    sat.sat[68].gnssId = 0;
    sat.sat[68].quality = 2;
    sat.sat[68].sigId = 0;
    sat.sat[68].status = 1024;
    sat.sat[68].svId = 0;
    sat.sat[69].cno = 0;
    sat.sat[69].gnssId = 6;
    sat.sat[69].quality = 0;
    sat.sat[69].sigId = 1;
    sat.sat[69].status = 62;
    sat.sat[69].svId = 133;
    sat.sat[70].cno = 0;
    sat.sat[70].gnssId = 0;
    sat.sat[70].quality = 0;
    sat.sat[70].sigId = 0;
    sat.sat[70].status = 4352;
    sat.sat[70].svId = 0;
    sat.sat[71].cno = 0;
    sat.sat[71].gnssId = 0;
    sat.sat[71].quality = 0;
    sat.sat[71].sigId = 0;
    sat.sat[71].status = 0;
    sat.sat[71].svId = 0;
    sat.sat[72].cno = 35;
    sat.sat[72].gnssId = 160;
    sat.sat[72].quality = 133;
    sat.sat[72].sigId = 18;
    sat.sat[72].status = 1;
    sat.sat[72].svId = 234;
    sat.sat[73].cno = 0;
    sat.sat[73].gnssId = 0;
    sat.sat[73].quality = 0;
    sat.sat[73].sigId = 0;
    sat.sat[73].status = 0;
    sat.sat[73].svId = 8;
    sat.sat[74].cno = 235;
    sat.sat[74].gnssId = 0;
    sat.sat[74].quality = 18;
    sat.sat[74].sigId = 48;
    sat.sat[74].status = 34083;
    sat.sat[74].svId = 0;
    sat.sat[75].cno = 208;
    sat.sat[75].gnssId = 1;
    sat.sat[75].quality = 235;
    sat.sat[75].sigId = 0;
    sat.sat[75].status = 8978;
    sat.sat[75].svId = 0;
    sat.sat[76].cno = 0;
    sat.sat[76].gnssId = 133;
    sat.sat[76].quality = 40;
    sat.sat[76].sigId = 0;
    sat.sat[76].status = 27395;
    sat.sat[76].svId = 1;
    sat.sat[77].cno = 0;
    sat.sat[77].gnssId = 33;
    sat.sat[77].quality = 0;
    sat.sat[77].sigId = 1;
    sat.sat[77].status = 336;
    sat.sat[77].svId = 133;
    sat.sat[78].cno = 1;
    sat.sat[78].gnssId = 107;
    sat.sat[78].quality = 0;
    sat.sat[78].sigId = 133;
    sat.sat[78].status = 10240;
    sat.sat[78].svId = 33;
    sat.sat[79].cno = 1;
    sat.sat[79].gnssId = 243;
    sat.sat[79].quality = 0;
    sat.sat[79].sigId = 118;
    sat.sat[79].status = 0;
    sat.sat[79].svId = 186;
    sat.sat[80].cno = 33;
    sat.sat[80].gnssId = 100;
    sat.sat[80].quality = 133;
    sat.sat[80].sigId = 107;
    sat.sat[80].status = 1;
    sat.sat[80].svId = 7;
    sat.sat[81].cno = 15;
    sat.sat[81].gnssId = 0;
    sat.sat[81].quality = 35;
    sat.sat[81].sigId = 0;
    sat.sat[81].status = 389;
    sat.sat[81].svId = 0;
    sat.sat[82].cno = 235;
    sat.sat[82].gnssId = 0;
    sat.sat[82].quality = 18;
    sat.sat[82].sigId = 64;
    sat.sat[82].status = 34083;
    sat.sat[82].svId = 0;
    sat.sat[83].cno = 52;
    sat.sat[83].gnssId = 1;
    sat.sat[83].quality = 0;
    sat.sat[83].sigId = 0;
    sat.sat[83].status = 0;
    sat.sat[83].svId = 0;
    sat.sat[84].cno = 0;
    sat.sat[84].gnssId = 0;
    sat.sat[84].quality = 208;
    sat.sat[84].sigId = 0;
    sat.sat[84].status = 0;
    sat.sat[84].svId = 0;
    sat.sat[85].cno = 0;
    sat.sat[85].gnssId = 0;
    sat.sat[85].quality = 0;
    sat.sat[85].sigId = 0;
    sat.sat[85].status = 4;
    sat.sat[85].svId = 0;
    sat.sat[86].cno = 0;
    sat.sat[86].gnssId = 0;
    sat.sat[86].quality = 0;
    sat.sat[86].sigId = 0;
    sat.sat[86].status = 3328;
    sat.sat[86].svId = 0;
    sat.sat[87].cno = 0;
    sat.sat[87].gnssId = 0;
    sat.sat[87].quality = 0;
    sat.sat[87].sigId = 0;
    sat.sat[87].status = 0;
    sat.sat[87].svId = 0;
    sat.sat[88].cno = 0;
    sat.sat[88].gnssId = 0;
    sat.sat[88].quality = 0;
    sat.sat[88].sigId = 0;
    sat.sat[88].status = 0;
    sat.sat[88].svId = 0;
    sat.sat[89].cno = 107;
    sat.sat[89].gnssId = 0;
    sat.sat[89].quality = 33;
    sat.sat[89].sigId = 0;
    sat.sat[89].status = 389;
    sat.sat[89].svId = 0;
    sat.sat[90].cno = 234;
    sat.sat[90].gnssId = 0;
    sat.sat[90].quality = 18;
    sat.sat[90].sigId = 176;
    sat.sat[90].status = 34083;
    sat.sat[90].svId = 0;
    sat.sat[91].cno = 74;
    sat.sat[91].gnssId = 1;
    sat.sat[91].quality = 0;
    sat.sat[91].sigId = 0;
    sat.sat[91].status = 19972;
    sat.sat[91].svId = 0;
    sat.sat[92].cno = 0;
    sat.sat[92].gnssId = 133;
    sat.sat[92].quality = 107;
    sat.sat[92].sigId = 0;
    sat.sat[92].status = 5121;
    sat.sat[92].svId = 1;
    sat.sat[93].cno = 0;
    sat.sat[93].gnssId = 80;
    sat.sat[93].quality = 0;
    sat.sat[93].sigId = 127;
    sat.sat[93].status = 2;
    sat.sat[93].svId = 250;
    sat.sat[94].cno = 0;
    sat.sat[94].gnssId = 4;
    sat.sat[94].quality = 0;
    sat.sat[94].sigId = 0;
    sat.sat[94].status = 61440;
    sat.sat[94].svId = 6;
    sat.sat[95].cno = 0;
    sat.sat[95].gnssId = 0;
    sat.sat[95].quality = 0;
    sat.sat[95].sigId = 0;
    sat.sat[95].status = 0;
    sat.sat[95].svId = 0;
    sat.sat[96].cno = 33;
    sat.sat[96].gnssId = 90;
    sat.sat[96].quality = 133;
    sat.sat[96].sigId = 107;
    sat.sat[96].status = 1;
    sat.sat[96].svId = 7;
    sat.sat[97].cno = 186;
    sat.sat[97].gnssId = 0;
    sat.sat[97].quality = 118;
    sat.sat[97].sigId = 242;
    sat.sat[97].status = 1;
    sat.sat[97].svId = 48;
    sat.sat[98].cno = 0;
    sat.sat[98].gnssId = 0;
    sat.sat[98].quality = 0;
    sat.sat[98].sigId = 4;
    sat.sat[98].status = 34048;
    sat.sat[98].svId = 0;
    sat.sat[99].cno = 60;
    sat.sat[99].gnssId = 1;
    sat.sat[99].quality = 0;
    sat.sat[99].sigId = 0;
    sat.sat[99].status = 0;
    sat.sat[99].svId = 0;









#if 0
    // GPS & SBAS        #msgs,msg#,numSV,  svid,elv,azm,cno, ..., signalId*checksum
    buf += "$GPGSV,3,1,11," "05,43,161,46," "06,17,052,43," "11,60,043,47," "12,68,189,46," "1*6D\r\n";
    buf += "$GPGSV,3,2,11," "20,61,121,47," "25,61,285,46," "28,11,305,43," "29,35,299,46," "1*6A\r\n";
    buf += "$GPGSV,3,3,11," "44,43,188,45," "46,40,206,44," "51,43,173,28,"                 "1*5D\r\n";
    buf += "$GPGSV,2,1,08," "05,43,161,43," "06,17,052,40," "11,60,043,46," "12,68,189,43," "6*61\r\n";
    buf += "$GPGSV,2,2,08," "25,61,285,44," "28,11,305,43," "29,35,299,42," "31,02,328,13," "6*6D\r\n";
    buf += "$GPGSV,1,1,03," "09,01,049,,"   "19,04,097,,"   "24,00,204,,"                   "0*51\r\n";
    // Galileo
    buf += "$GAGSV,2,1,05," "13,73,014,47," "15,23,052,44," "21,69,230,45," "26,43,250,47," "2*79\r\n";
    buf += "$GAGSV,2,2,05," "27,30,162,46,"                                                 "2*42\r\n";
    buf += "$GAGSV,2,1,05," "13,73,014,45," "15,23,052,44," "21,69,230,46," "26,43,250,43," "7*79\r\n";
    buf += "$GAGSV,2,2,05," "27,30,162,43,"                                                 "7*42\r\n";
    buf += "$GAGSV,1,1,02," "03,05,063,,"   "08,04,109,,"                                   "0*71\r\n";
    // Beidou
    buf += "$GBGSV,2,1,08," "19,27,131,44," "20,28,075,45," "24,22,309,44," "26,40,257,45," "1*75\r\n";
    buf += "$GBGSV,2,2,08," "29,44,065,47," "35,65,315,48," "44,20,276,44," "57,,,45,"      "1*49\r\n";
    // QZSS
    buf += "$GQGSV,1,1,00,"                                                                 "0*64\r\n";
    // GLONASS
    buf += "$GLGSV,3,1,09," "65,26,093,46," "72,17,038,48," "78,41,208,48," "79,50,291,47," "1*76\r\n";
    buf += "$GLGSV,3,2,09," "80,12,334,46," "81,48,297,48," "82,09,264,32," "87,07,062,34," "1*71\r\n";
    buf += "$GLGSV,3,3,09," "88,49,030,49,"                                                 "1*42\r\n";
    buf += "$GLGSV,2,1,08," "65,26,093,40," "72,17,038,40," "78,41,208,45," "79,50,291,44," "3*74\r\n";
    buf += "$GLGSV,2,2,08," "80,12,334,39," "81,48,297,45," "82,09,264,32," "88,49,030,42," "3*75\r\n";
    buf += "$GLGSV,1,1,01," "66,06,143,,"                                                   "0*48\r\n";

    gps_sat_t sat = {};
    sat.numSats = 40;
    sat.timeOfWeekMs = 593477000;

    sat.sat[0].azim = 161;
    sat.sat[0].cno = 46;
    sat.sat[0].elev = 43;
    sat.sat[0].gnssId = 1;
    sat.sat[0].status = 1;
    sat.sat[0].svId = 5;
    sat.sat[1].azim = 52;
    sat.sat[1].cno = 43;
    sat.sat[1].elev = 17;
    sat.sat[1].gnssId = 1;
    sat.sat[1].status = 1;
    sat.sat[1].svId = 6;
    sat.sat[2].azim = 48;
    sat.sat[2].cno = 0;
    sat.sat[2].elev = 1;
    sat.sat[2].gnssId = 1;
    sat.sat[2].status = 0;
    sat.sat[2].svId = 9;
    sat.sat[3].azim = 43;
    sat.sat[3].cno = 47;
    sat.sat[3].elev = 59;
    sat.sat[3].gnssId = 1;
    sat.sat[3].status = 1;
    sat.sat[3].svId = 11;
    sat.sat[4].azim = 188;
    sat.sat[4].cno = 46;
    sat.sat[4].elev = 67;
    sat.sat[4].gnssId = 1;
    sat.sat[4].status = 1;
    sat.sat[4].svId = 12;
    sat.sat[5].azim = 97;
    sat.sat[5].cno = 0;
    sat.sat[5].elev = 4;
    sat.sat[5].gnssId = 1;
    sat.sat[5].status = 0;
    sat.sat[5].svId = 19;
    sat.sat[6].azim = 120;
    sat.sat[6].cno = 46;
    sat.sat[6].elev = 61;
    sat.sat[6].gnssId = 1;
    sat.sat[6].status = 1;
    sat.sat[6].svId = 20;
    sat.sat[7].azim = 204;
    sat.sat[7].cno = 0;
    sat.sat[7].elev = 0;
    sat.sat[7].gnssId = 1;
    sat.sat[7].status = 0;
    sat.sat[7].svId = 24;
    sat.sat[8].azim = 283;
    sat.sat[8].cno = 47;
    sat.sat[8].elev = 61;
    sat.sat[8].gnssId = 1;
    sat.sat[8].status = 1;
    sat.sat[8].svId = 25;
    sat.sat[9].azim = 305;
    sat.sat[9].cno = 43;
    sat.sat[9].elev = 11;
    sat.sat[9].gnssId = 1;
    sat.sat[9].status = 1;
    sat.sat[9].svId = 28;
    sat.sat[10].azim = 300;
    sat.sat[10].cno = 47;
    sat.sat[10].elev = 36;
    sat.sat[10].gnssId = 1;
    sat.sat[10].status = 1;
    sat.sat[10].svId = 29;
    sat.sat[11].azim = 328;
    sat.sat[11].cno = 13;
    sat.sat[11].elev = 2;
    sat.sat[11].gnssId = 1;
    sat.sat[11].status = 0;
    sat.sat[11].svId = 31;
    sat.sat[12].azim = 188;
    sat.sat[12].cno = 45;
    sat.sat[12].elev = 43;
    sat.sat[12].gnssId = 2;
    sat.sat[12].status = 1;
    sat.sat[12].svId = 131;
    sat.sat[13].azim = 206;
    sat.sat[13].cno = 44;
    sat.sat[13].elev = 40;
    sat.sat[13].gnssId = 2;
    sat.sat[13].status = 1;
    sat.sat[13].svId = 133;
    sat.sat[14].azim = 173;
    sat.sat[14].cno = 0;
    sat.sat[14].elev = 43;
    sat.sat[14].gnssId = 2;
    sat.sat[14].status = 0;
    sat.sat[14].svId = 138;
    sat.sat[15].azim = 63;
    sat.sat[15].cno = 0;
    sat.sat[15].elev = 5;
    sat.sat[15].gnssId = 3;
    sat.sat[15].status = 0;
    sat.sat[15].svId = 3;
    sat.sat[16].azim = 108;
    sat.sat[16].cno = 0;
    sat.sat[16].elev = 5;
    sat.sat[16].gnssId = 3;
    sat.sat[16].status = 1;
    sat.sat[16].svId = 8;
    sat.sat[17].azim = 16;
    sat.sat[17].cno = 45;
    sat.sat[17].elev = 72;
    sat.sat[17].gnssId = 3;
    sat.sat[17].status = 1;
    sat.sat[17].svId = 13;
    sat.sat[18].azim = 53;
    sat.sat[18].cno = 44;
    sat.sat[18].elev = 22;
    sat.sat[18].gnssId = 3;
    sat.sat[18].status = 1;
    sat.sat[18].svId = 15;
    sat.sat[19].azim = 228;
    sat.sat[19].cno = 46;
    sat.sat[19].elev = 69;
    sat.sat[19].gnssId = 3;
    sat.sat[19].status = 1;
    sat.sat[19].svId = 21;
    sat.sat[20].azim = 252;
    sat.sat[20].cno = 44;
    sat.sat[20].elev = 44;
    sat.sat[20].gnssId = 3;
    sat.sat[20].status = 1;
    sat.sat[20].svId = 26;
    sat.sat[21].azim = 163;
    sat.sat[21].cno = 44;
    sat.sat[21].elev = 28;
    sat.sat[21].gnssId = 3;
    sat.sat[21].status = 1;
    sat.sat[21].svId = 27;
    sat.sat[22].azim = 130;
    sat.sat[22].cno = 44;
    sat.sat[22].elev = 28;
    sat.sat[22].gnssId = 4;
    sat.sat[22].status = 1;
    sat.sat[22].svId = 19;
    sat.sat[23].azim = 73;
    sat.sat[23].cno = 47;
    sat.sat[23].elev = 28;
    sat.sat[23].gnssId = 4;
    sat.sat[23].status = 1;
    sat.sat[23].svId = 20;
    sat.sat[24].azim = 308;
    sat.sat[24].cno = 45;
    sat.sat[24].elev = 22;
    sat.sat[24].gnssId = 4;
    sat.sat[24].status = 1;
    sat.sat[24].svId = 24;
    sat.sat[25].azim = 255;
    sat.sat[25].cno = 46;
    sat.sat[25].elev = 40;
    sat.sat[25].gnssId = 4;
    sat.sat[25].status = 1;
    sat.sat[25].svId = 26;
    sat.sat[26].azim = 66;
    sat.sat[26].cno = 47;
    sat.sat[26].elev = 43;
    sat.sat[26].gnssId = 4;
    sat.sat[26].status = 1;
    sat.sat[26].svId = 29;
    sat.sat[27].azim = 318;
    sat.sat[27].cno = 0;
    sat.sat[27].elev = 66;
    sat.sat[27].gnssId = 4;
    sat.sat[27].status = 1;
    sat.sat[27].svId = 35;
    sat.sat[28].azim = 277;
    sat.sat[28].cno = 45;
    sat.sat[28].elev = 21;
    sat.sat[28].gnssId = 4;
    sat.sat[28].status = 1;
    sat.sat[28].svId = 44;
    sat.sat[29].azim = 0;
    sat.sat[29].cno = 46;
    sat.sat[29].elev = 0xa5;
    sat.sat[29].gnssId = 4;
    sat.sat[29].status = 1;
    sat.sat[29].svId = 57;
    sat.sat[30].azim = 92;
    sat.sat[30].cno = 0;
    sat.sat[30].elev = 27;
    sat.sat[30].gnssId = 6;
    sat.sat[30].status = 1;
    sat.sat[30].svId = 1;
    sat.sat[31].azim = 142;
    sat.sat[31].cno = 0;
    sat.sat[31].elev = 7;
    sat.sat[31].gnssId = 6;
    sat.sat[31].status = 1;
    sat.sat[31].svId = 2;
    sat.sat[32].azim = 38;
    sat.sat[32].cno = 2;
    sat.sat[32].elev = 17;
    sat.sat[32].gnssId = 6;
    sat.sat[32].status = 1;
    sat.sat[32].svId = 8;
    sat.sat[33].azim = 207;
    sat.sat[33].cno = 1;
    sat.sat[33].elev = 40;
    sat.sat[33].gnssId = 6;
    sat.sat[33].status = 1;
    sat.sat[33].svId = 14;
    sat.sat[34].azim = 290;
    sat.sat[34].cno = 0;
    sat.sat[34].elev = 50;
    sat.sat[34].gnssId = 6;
    sat.sat[34].status = 1;
    sat.sat[34].svId = 15;
    sat.sat[35].azim = 334;
    sat.sat[35].cno = 45;
    sat.sat[35].elev = 12;
    sat.sat[35].gnssId = 6;
    sat.sat[35].status = 1;
    sat.sat[35].svId = 16;
    sat.sat[36].azim = 298;
    sat.sat[36].cno = 0;
    sat.sat[36].elev = 49;
    sat.sat[36].gnssId = 6;
    sat.sat[36].status = 1;
    sat.sat[36].svId = 17;
    sat.sat[37].azim = 264;
    sat.sat[37].cno = 33;
    sat.sat[37].elev = 10;
    sat.sat[37].gnssId = 6;
    sat.sat[37].status = 1;
    sat.sat[37].svId = 18;
    sat.sat[38].azim = 63;
    sat.sat[38].cno = 26;
    sat.sat[38].elev = 7;
    sat.sat[38].gnssId = 6;
    sat.sat[38].status = 1;
    sat.sat[38].svId = 23;
    sat.sat[39].azim = 31;
    sat.sat[39].cno = 3;
    sat.sat[39].elev = 48;
    sat.sat[39].gnssId = 6;
    sat.sat[39].status = 1;
    sat.sat[39].svId = 24;
#endif


    nmea_gsv(g_ascii_buf, ASCII_BUF2, sat);
    // printf("%s\n", g_ascii_buf);

    // cout << "buf size: " << buf.size() << "\n";
    // cout << buf;

    // gps_sat_t result = {};
    // int lastGSVmsg[2] = {};
    // int satCount = 0;
    // uint32_t cnoSum = 0;
    // uint32_t cnoCount = 0;
    // nmea_parse_gsv(buf.c_str(), buf.size(), &result, lastGSVmsg, &satCount, &cnoSum, &cnoCount);

}



