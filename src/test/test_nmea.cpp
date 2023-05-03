#include <gtest/gtest.h>
#include "../../../SDK/src/protocol_nmea.h"


#define ASCII_BUF_LEN   200
#define POS_LAT_DEG     40.330578
#define POS_LON_DEG     -111.725816
#define POS_ALT_M       1406.39
#define LEAP_SEC        18

TEST(nmea, INFO)
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

TEST(nmea, PIMU)
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

TEST(nmea, PRIMU)
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

TEST(nmea, PPIMU)
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

TEST(nmea, PINS1)
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

TEST(nmea, PINS2)
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

TEST(nmea, PGPSP)
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

TEST(nmea, GGA)
{
    gps_pos_t pos = {};
    // pos.week = 12;
    pos.timeOfWeekMs = 370659600;
    pos.satsUsed = 34;
    pos.status = GPS_STATUS_NUM_SATS_USED_MASK & pos.satsUsed;
    pos.lla[0] = POS_LAT_DEG;
    pos.lla[1] = POS_LON_DEG;
    pos.lla[2] = POS_ALT_M;
    pos.hMSL = 89;
    pos.pDop = 6;
    pos.leapS = LEAP_SEC;

    char ascii_buf[ASCII_BUF_LEN] = { 0 };
    did_gps_to_nmea_gga(ascii_buf, ASCII_BUF_LEN, pos);
    // printf("%s\n", ascii_buf);
    gps_pos_t result = {};
    result.leapS = pos.leapS;
     uint32_t weekday = pos.timeOfWeekMs / 86400000;
    nmea_gga_to_did_gps(result, ascii_buf, ASCII_BUF_LEN, weekday);
    ASSERT_EQ(memcmp(&pos, &result, sizeof(result)), 0);
}

TEST(nmea, GGL)
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

TEST(nmea, GSA)
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
    printf("%s\n", ascii_buf);
    gps_pos_t resultPos = {};
    gps_sat_t resultSat = {};
    nmea_gsa_to_did_gps(resultPos, resultSat, ascii_buf, ASCII_BUF_LEN);
    ASSERT_EQ(memcmp(&pos, &resultPos, sizeof(resultPos)), 0);
    ASSERT_EQ(memcmp(&sat, &resultSat, sizeof(resultSat)), 0);
}



