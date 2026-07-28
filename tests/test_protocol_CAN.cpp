#include <gtest/gtest.h>
#include "protocol_CAN.h"
#include "ISPose.h"

// Stub firmware globals required by CAN_ISB_INS_to_CAN_altitude and
// CAN_ISB_IMU_to_CAN_roll_rollRate. In firmware builds these live in globals.c;
// in the SDK test build we own the definitions here.
typedef struct { gnss_pos_t pos; gnss_vel_t vel; } gnss_posvel_t;
gnss_posvel_t g_gnssNav[2] = {};
ins_output_t  g_insOut     = {};

// ============================================================================
// ISB -> CAN: individual encoding functions
// ============================================================================

TEST(protocol_CAN, INS_to_time)
{
    ins_1_t src = {};
    src.week       = 2345;
    src.timeOfWeek = 123456.789;

    is_can_payload p = {};
    int sz = CAN_ISB_INS_to_CAN_time(&src, &p);

    EXPECT_EQ(sz, (int)sizeof(is_can_time));
    EXPECT_EQ(p.time.week, src.week);
    EXPECT_NEAR(p.time.timeOfWeekMs, (float)src.timeOfWeek, 1e-3f);
}

TEST(protocol_CAN, INS_to_ins_status)
{
    ins_1_t src = {};
    src.insStatus = 0xDEADBEEF;
    src.hdwStatus = 0x12345678;

    is_can_payload p = {};
    int sz = CAN_ISB_INS_to_CAN_ins_status(&src, &p);

    EXPECT_EQ(sz, (int)sizeof(is_can_ins_status));
    EXPECT_EQ(p.insstatus.insStatus, src.insStatus);
    EXPECT_EQ(p.insstatus.hdwStatus, src.hdwStatus);
}

TEST(protocol_CAN, INS_to_ins_euler)
{
    ins_1_t src = {};
    src.theta[0] = 0.1234f;
    src.theta[1] = -0.5678f;
    src.theta[2] = 1.2345f;

    is_can_payload p = {};
    int sz = CAN_ISB_INS_to_CAN_ins_euler(&src, &p);

    EXPECT_EQ(sz, (int)sizeof(is_can_ins_euler));
    EXPECT_NEAR(p.euler.theta1 / 10000.0f, src.theta[0], 1e-4f);
    EXPECT_NEAR(p.euler.theta2 / 10000.0f, src.theta[1], 1e-4f);
    EXPECT_NEAR(p.euler.theta3 / 10000.0f, src.theta[2], 1e-4f);
}

TEST(protocol_CAN, INS_to_quatn2b)
{
    ins_2_t src = {};
    src.qn2b[0] = 0.7071f;
    src.qn2b[1] = 0.7071f;
    src.qn2b[2] = 0.0f;
    src.qn2b[3] = 0.0f;

    is_can_payload p = {};
    int sz = CAN_ISB_INS_to_CAN_quatn2b(&src, &p);

    EXPECT_EQ(sz, (int)sizeof(is_can_ins_quatn2b));
    EXPECT_NEAR(p.quatn2b.qn2b1 / 10000.0f, src.qn2b[0], 1e-4f);
    EXPECT_NEAR(p.quatn2b.qn2b2 / 10000.0f, src.qn2b[1], 1e-4f);
    EXPECT_NEAR(p.quatn2b.qn2b3 / 10000.0f, src.qn2b[2], 1e-4f);
    EXPECT_NEAR(p.quatn2b.qn2b4 / 10000.0f, src.qn2b[3], 1e-4f);
}

TEST(protocol_CAN, INS_to_quate2b)
{
    ins_4_t src = {};
    src.qe2b[0] = 0.5f;
    src.qe2b[1] = 0.5f;
    src.qe2b[2] = 0.5f;
    src.qe2b[3] = 0.5f;

    is_can_payload p = {};
    int sz = CAN_ISB_INS_to_CAN_quate2b(&src, &p);

    EXPECT_EQ(sz, (int)sizeof(is_can_ins_quate2b));
    EXPECT_NEAR(p.quate2b.qe2b1 / 10000.0f, src.qe2b[0], 1e-4f);
    EXPECT_NEAR(p.quate2b.qe2b2 / 10000.0f, src.qe2b[1], 1e-4f);
    EXPECT_NEAR(p.quate2b.qe2b3 / 10000.0f, src.qe2b[2], 1e-4f);
    EXPECT_NEAR(p.quate2b.qe2b4 / 10000.0f, src.qe2b[3], 1e-4f);
}

TEST(protocol_CAN, INS_to_uvw)
{
    ins_1_t src = {};
    src.uvw[0] = 12.34f;
    src.uvw[1] = -5.67f;
    src.uvw[2] = 89.10f;

    is_can_payload p = {};
    int sz = CAN_ISB_INS_to_CAN_uvw(&src, &p);

    EXPECT_EQ(sz, (int)sizeof(is_can_uvw));
    EXPECT_NEAR(p.uvw.uvw1 / 100.0f, src.uvw[0], 0.01f);
    EXPECT_NEAR(p.uvw.uvw2 / 100.0f, src.uvw[1], 0.01f);
    EXPECT_NEAR(p.uvw.uvw3 / 100.0f, src.uvw[2], 0.01f);
}

TEST(protocol_CAN, INS_to_ve)
{
    ins_4_t src = {};
    src.ve[0] = 3.21f;
    src.ve[1] = -7.65f;
    src.ve[2] = 1.00f;

    is_can_payload p = {};
    int sz = CAN_ISB_INS_to_CAN_ve(&src, &p);

    EXPECT_EQ(sz, (int)sizeof(is_can_ve));
    EXPECT_NEAR(p.ve.ve1 / 100.0f, src.ve[0], 0.01f);
    EXPECT_NEAR(p.ve.ve2 / 100.0f, src.ve[1], 0.01f);
    EXPECT_NEAR(p.ve.ve3 / 100.0f, src.ve[2], 0.01f);
}

TEST(protocol_CAN, INS_to_latitude)
{
    ins_1_t src = {};
    src.lla[0] = 40.330578;

    is_can_payload p = {};
    int sz = CAN_ISB_INS_to_CAN_latitude(&src, &p);

    EXPECT_EQ(sz, (int)sizeof(is_can_ins_lat));
    EXPECT_DOUBLE_EQ(p.lat.lat, src.lla[0]);
}

TEST(protocol_CAN, INS_to_longitude)
{
    ins_1_t src = {};
    src.lla[1] = -111.725816;

    is_can_payload p = {};
    int sz = CAN_ISB_INS_to_CAN_longitude(&src, &p);

    EXPECT_EQ(sz, (int)sizeof(is_can_ins_lon));
    EXPECT_DOUBLE_EQ(p.lon.lon, src.lla[1]);
}

TEST(protocol_CAN, INS_to_altitude)
{
    // Set the global GNSS status stub used by CAN_ISB_INS_to_CAN_altitude.
    g_gnssNav[0].pos.status = 0xABCD1234u;

    ins_1_t src = {};
    src.lla[2] = 1406.39;

    is_can_payload p = {};
    int sz = CAN_ISB_INS_to_CAN_altitude(&src, &p);

    EXPECT_EQ(sz, (int)sizeof(is_can_ins_alt));
    EXPECT_NEAR(p.alt.alt, (float)src.lla[2], 1e-3f);
    EXPECT_EQ(p.alt.status, g_gnssNav[0].pos.status);
}

TEST(protocol_CAN, INS_to_ned_north_east)
{
    ins_1_t src = {};
    src.ned[0] = 10.5f;
    src.ned[1] = -3.2f;

    is_can_payload p = {};
    int sz = CAN_ISB_INS_to_CAN_ned_north_east(&src, &p);

    EXPECT_EQ(sz, (int)sizeof(is_can_north_east));
    EXPECT_FLOAT_EQ(p.ne.ned1, src.ned[0]);
    EXPECT_FLOAT_EQ(p.ne.ned2, src.ned[1]);
}

TEST(protocol_CAN, INS_to_ned_down)
{
    ins_1_t src = {};
    src.ned[2]    = -7.8f;
    src.insStatus = 0x00FF0000u;

    is_can_payload p = {};
    int sz = CAN_ISB_INS_to_CAN_ned_down(&src, &p);

    EXPECT_EQ(sz, (int)sizeof(is_can_down));
    EXPECT_FLOAT_EQ(p.down.ned3, src.ned[2]);
    EXPECT_EQ(p.down.insStatus, src.insStatus);
}

TEST(protocol_CAN, INS_to_ecef_x)
{
    ins_4_t src = {};
    src.ecef[0] = -2703779.123;

    is_can_payload p = {};
    int sz = CAN_ISB_INS_to_CAN_ecef_x(&src, &p);

    EXPECT_EQ(sz, (int)sizeof(is_can_ecef_x));
    EXPECT_DOUBLE_EQ(p.ecefx.ecef1, src.ecef[0]);
}

TEST(protocol_CAN, INS_to_ecef_y)
{
    ins_4_t src = {};
    src.ecef[1] = -4354914.456;

    is_can_payload p = {};
    int sz = CAN_ISB_INS_to_CAN_ecef_y(&src, &p);

    EXPECT_EQ(sz, (int)sizeof(is_can_ecef_y));
    EXPECT_DOUBLE_EQ(p.ecefy.ecef2, src.ecef[1]);
}

TEST(protocol_CAN, INS_to_ecef_z)
{
    ins_4_t src = {};
    src.ecef[2] = 4452679.789;

    is_can_payload p = {};
    int sz = CAN_ISB_INS_to_CAN_ecef_z(&src, &p);

    EXPECT_EQ(sz, (int)sizeof(is_can_ecef_z));
    EXPECT_DOUBLE_EQ(p.ecefz.ecef3, src.ecef[2]);
}

TEST(protocol_CAN, INS_to_msl)
{
    ins_3_t src = {};
    src.msl = 1388.25f;

    is_can_payload p = {};
    int sz = CAN_ISB_INS_to_CAN_msl(&src, &p);

    EXPECT_EQ(sz, (int)sizeof(is_can_msl));
    EXPECT_FLOAT_EQ(p.msl.msl, src.msl);
}

TEST(protocol_CAN, PIMU_px)
{
    pimu_t src = {};
    src.theta[0] = 0.123f;
    src.vel[0]   = 4.56f;
    src.dt       = 0.005f;

    is_can_payload p = {};
    int sz = CAN_ISB_PIMU_to_CAN_pimu_px(&src, &p);

    EXPECT_EQ(sz, (int)sizeof(is_can_preint_imu_px));
    EXPECT_NEAR(p.pimupx.theta0 / 1000.0f, src.theta[0], 1e-3f);
    EXPECT_NEAR(p.pimupx.vel0   / 100.0f,  src.vel[0],   0.01f);
    EXPECT_NEAR(p.pimupx.dt     / 1000.0f, src.dt,        1e-3f);
}

TEST(protocol_CAN, PIMU_qy)
{
    pimu_t src = {};
    src.theta[1] = -0.234f;
    src.vel[1]   = -7.89f;
    src.dt       = 0.01f;

    is_can_payload p = {};
    int sz = CAN_ISB_PIMU_to_CAN_pimu_qy(&src, &p);

    EXPECT_EQ(sz, (int)sizeof(is_can_preint_imu_qy));
    EXPECT_NEAR(p.pimuqy.theta1 / 1000.0f, src.theta[1], 1e-3f);
    EXPECT_NEAR(p.pimuqy.vel1   / 100.0f,  src.vel[1],   0.01f);
    EXPECT_NEAR(p.pimuqy.dt     / 1000.0f, src.dt,        1e-3f);
}

TEST(protocol_CAN, PIMU_rz)
{
    pimu_t src = {};
    src.theta[2] = 0.456f;
    src.vel[2]   = 1.23f;
    src.dt       = 0.02f;

    is_can_payload p = {};
    int sz = CAN_ISB_PIMU_to_CAN_pimu_rz(&src, &p);

    EXPECT_EQ(sz, (int)sizeof(is_can_preint_imu_rz));
    EXPECT_NEAR(p.pimurz.theta2 / 1000.0f, src.theta[2], 1e-3f);
    EXPECT_NEAR(p.pimurz.vel2   / 100.0f,  src.vel[2],   0.01f);
    EXPECT_NEAR(p.pimurz.dt     / 1000.0f, src.dt,        1e-3f);
}

TEST(protocol_CAN, IMU_dual_px)
{
    imu_t src = {};
    src.I.pqr[0] = 0.567f;
    src.I.acc[0] = 9.81f;
    src.status   = 0x00000003u;

    is_can_payload p = {};
    int sz = CAN_ISB_IMU_to_CAN_dual_imu_px(&src, &p);

    EXPECT_EQ(sz, (int)sizeof(is_can_dual_imu_px));
    EXPECT_NEAR(p.dimupx.theta0 / 1000.0f, src.I.pqr[0], 1e-3f);
    EXPECT_NEAR(p.dimupx.vel0   / 100.0f,  src.I.acc[0],  0.01f);
    EXPECT_EQ(p.dimupx.status, src.status);
}

TEST(protocol_CAN, IMU_dual_qy)
{
    imu_t src = {};
    src.I.pqr[1] = -0.234f;
    src.I.acc[1] = -1.23f;
    src.status   = 0x00000001u;

    is_can_payload p = {};
    int sz = CAN_ISB_IMU_to_CAN_dual_imu_qy(&src, &p);

    EXPECT_EQ(sz, (int)sizeof(is_can_dual_imu_qy));
    EXPECT_NEAR(p.dimuqy.theta1 / 1000.0f, src.I.pqr[1], 1e-3f);
    EXPECT_NEAR(p.dimuqy.vel1   / 100.0f,  src.I.acc[1],  0.01f);
    EXPECT_EQ(p.dimuqy.status, src.status);
}

TEST(protocol_CAN, IMU_dual_rz)
{
    imu_t src = {};
    src.I.pqr[2] = 0.789f;
    src.I.acc[2] = 4.56f;
    src.status   = 0x00000002u;

    is_can_payload p = {};
    int sz = CAN_ISB_IMU_to_CAN_dual_imu_rz(&src, &p);

    EXPECT_EQ(sz, (int)sizeof(is_can_dual_imu_rz));
    EXPECT_NEAR(p.dimurz.theta2 / 1000.0f, src.I.pqr[2], 1e-3f);
    EXPECT_NEAR(p.dimurz.vel2   / 100.0f,  src.I.acc[2],  0.01f);
    EXPECT_EQ(p.dimurz.status, src.status);
}

TEST(protocol_CAN, GNSS_gnss1_pos)
{
    gnss_pos_t src = {};
    src.status  = 0x00000C12u;
    src.cnoMean = 38.5f;

    is_can_payload p = {};
    int sz = CAN_ISB_GNSS_to_CAN_gnss1_pos(&src, &p);

    EXPECT_EQ(sz, (int)sizeof(is_can_gnss_pos_status));
    EXPECT_EQ(p.gnsspos.status, src.status);
    EXPECT_EQ(p.gnsspos.cnoMean, (uint32_t)src.cnoMean);
}

TEST(protocol_CAN, GNSS_gnss2_pos)
{
    gnss_pos_t src = {};
    src.status  = 0x00001008u;
    src.cnoMean = 42.0f;

    is_can_payload p = {};
    int sz = CAN_ISB_GNSS_to_CAN_gnss2_pos(&src, &p);

    EXPECT_EQ(sz, (int)sizeof(is_can_gnss_pos_status));
    EXPECT_EQ(p.gnsspos.status, src.status);
    EXPECT_EQ(p.gnsspos.cnoMean, (uint32_t)src.cnoMean);
}

TEST(protocol_CAN, GNSS_rtk_pos_rel)
{
    gnss_rtk_rel_t src = {};
    src.arRatio             = 3.5f;
    src.differentialAge     = 1.0f;
    src.baseToRoverDistance = 245.6f;
    src.rtkHeading  = 1.234f;

    is_can_payload p = {};
    int sz = CAN_ISB_GNSS_to_CAN_gnss1_rtk_pos_rel(&src, &p);

    EXPECT_EQ(sz, (int)sizeof(is_can_gnss_rtk_rel));
    EXPECT_EQ(p.rtkrel.arRatio,         (uint8_t)src.arRatio);
    EXPECT_EQ(p.rtkrel.differentialAge, (uint8_t)src.differentialAge);
    EXPECT_FLOAT_EQ(p.rtkrel.distanceToBase, src.baseToRoverDistance);
    EXPECT_NEAR(p.rtkrel.rtkHeading / 1000.0f, src.rtkHeading, 1e-3f);
}

TEST(protocol_CAN, GNSS_rtk_cmp_rel)
{
    gnss_rtk_rel_t src = {};
    src.arRatio             = 5.0f;
    src.differentialAge     = 2.0f;
    src.baseToRoverDistance = 123.4f;
    src.rtkHeading          = -0.567f;

    is_can_payload p = {};
    int sz = CAN_ISB_GNSS_to_CAN_gnss2_rtk_cmp_rel(&src, &p);

    EXPECT_EQ(sz, (int)sizeof(is_can_gnss_rtk_rel));
    EXPECT_EQ(p.rtkrel.arRatio,         (uint8_t)src.arRatio);
    EXPECT_EQ(p.rtkrel.differentialAge, (uint8_t)src.differentialAge);
    EXPECT_FLOAT_EQ(p.rtkrel.distanceToBase, src.baseToRoverDistance);
    EXPECT_NEAR(p.rtkrel.rtkHeading / 1000.0f, src.rtkHeading, 1e-3f);
}

TEST(protocol_CAN, IMU_roll_rollRate)
{
    // Set global INS output quaternion (identity → roll=0).
    g_insOut = {};
    g_insOut.qn2b[0] = 1.0f;   // w=1, x=y=z=0 → identity → theta = {0,0,0}

    imu_t src = {};
    src.I.pqr[0] = 0.314f;

    is_can_payload p = {};
    int sz = CAN_ISB_IMU_to_CAN_roll_rollRate(&src, &p);

    EXPECT_EQ(sz, (int)sizeof(is_can_roll_rollRate));
    // identity quaternion → roll ≈ 0
    EXPECT_NEAR(p.rollrollrate.insRoll / 10000.0f, 0.0f, 1e-4f);
    EXPECT_NEAR(p.rollrollrate.pImu1   / 1000.0f,  src.I.pqr[0], 1e-3f);
    EXPECT_NEAR(p.rollrollrate.pImu2   / 1000.0f,  src.I.pqr[0], 1e-3f);
}

// ============================================================================
// CAN -> ISB: round-trip tests (encode then decode, check within quantization)
// ============================================================================

TEST(protocol_CAN, roundtrip_time)
{
    ins_1_t src = {}, out = {};
    src.week       = 2345;
    src.timeOfWeek = 302400.0;

    is_can_payload p = {};
    CAN_ISB_INS_to_CAN_time(&src, &p);
    CAN_CAN_time_to_ISB_INS_1(&p, &out);

    EXPECT_EQ(out.week, src.week);
    EXPECT_NEAR(out.timeOfWeek, src.timeOfWeek, 1e-3);
}

TEST(protocol_CAN, roundtrip_ins_status)
{
    ins_1_t src = {}, out = {};
    src.insStatus = 0xCAFEBABEu;
    src.hdwStatus = 0x0000FFFFu;

    is_can_payload p = {};
    CAN_ISB_INS_to_CAN_ins_status(&src, &p);
    CAN_CAN_ins_status_to_ISB_INS_1(&p, &out);

    EXPECT_EQ(out.insStatus, src.insStatus);
    EXPECT_EQ(out.hdwStatus, src.hdwStatus);
}

TEST(protocol_CAN, roundtrip_euler)
{
    ins_1_t src = {}, out = {};
    src.theta[0] = 0.1f;
    src.theta[1] = -0.2f;
    src.theta[2] = 0.3f;

    is_can_payload p = {};
    CAN_ISB_INS_to_CAN_ins_euler(&src, &p);
    CAN_CAN_ins_euler_to_ISB_INS_1(&p, &out);

    for (int i = 0; i < 3; i++)
        EXPECT_NEAR(out.theta[i], src.theta[i], 1e-4f) << "theta[" << i << "]";
}

TEST(protocol_CAN, roundtrip_quatn2b)
{
    ins_2_t src = {}, out = {};
    src.qn2b[0] = 0.7071f;
    src.qn2b[1] = 0.7071f;
    src.qn2b[2] = 0.0f;
    src.qn2b[3] = 0.0f;

    is_can_payload p = {};
    CAN_ISB_INS_to_CAN_quatn2b(&src, &p);
    CAN_CAN_quatn2b_to_ISB_INS_2(&p, &out);

    for (int i = 0; i < 4; i++)
        EXPECT_NEAR(out.qn2b[i], src.qn2b[i], 1e-4f) << "qn2b[" << i << "]";
}

TEST(protocol_CAN, roundtrip_quate2b)
{
    ins_4_t src = {}, out = {};
    src.qe2b[0] = 0.5f;
    src.qe2b[1] = 0.5f;
    src.qe2b[2] = 0.5f;
    src.qe2b[3] = 0.5f;

    is_can_payload p = {};
    CAN_ISB_INS_to_CAN_quate2b(&src, &p);
    CAN_CAN_quate2b_to_ISB_INS_4(&p, &out);

    for (int i = 0; i < 4; i++)
        EXPECT_NEAR(out.qe2b[i], src.qe2b[i], 1e-4f) << "qe2b[" << i << "]";
}

TEST(protocol_CAN, roundtrip_uvw)
{
    ins_1_t src = {}, out = {};
    src.uvw[0] = 12.34f;
    src.uvw[1] = -56.78f;
    src.uvw[2] = 100.00f;

    is_can_payload p = {};
    CAN_ISB_INS_to_CAN_uvw(&src, &p);
    CAN_CAN_uvw_to_ISB_INS_1(&p, &out);

    for (int i = 0; i < 3; i++)
        EXPECT_NEAR(out.uvw[i], src.uvw[i], 0.01f) << "uvw[" << i << "]";
}

TEST(protocol_CAN, roundtrip_ve)
{
    ins_4_t src = {}, out = {};
    src.ve[0] = 3.21f;
    src.ve[1] = -7.65f;
    src.ve[2] = 0.50f;

    is_can_payload p = {};
    CAN_ISB_INS_to_CAN_ve(&src, &p);
    CAN_CAN_ve_to_ISB_INS_4(&p, &out);

    for (int i = 0; i < 3; i++)
        EXPECT_NEAR(out.ve[i], src.ve[i], 0.01f) << "ve[" << i << "]";
}

TEST(protocol_CAN, roundtrip_latitude)
{
    ins_1_t src = {}, out = {};
    src.lla[0] = 40.330578;

    is_can_payload p = {};
    CAN_ISB_INS_to_CAN_latitude(&src, &p);
    CAN_CAN_latitude_to_ISB_INS_1(&p, &out);

    EXPECT_DOUBLE_EQ(out.lla[0], src.lla[0]);
}

TEST(protocol_CAN, roundtrip_longitude)
{
    ins_1_t src = {}, out = {};
    src.lla[1] = -111.725816;

    is_can_payload p = {};
    CAN_ISB_INS_to_CAN_longitude(&src, &p);
    CAN_CAN_longitude_to_ISB_INS_1(&p, &out);

    EXPECT_DOUBLE_EQ(out.lla[1], src.lla[1]);
}

TEST(protocol_CAN, roundtrip_altitude)
{
    g_gnssNav[0].pos.status = 0x12345678u;

    ins_1_t src = {}, out = {};
    src.lla[2] = 1406.39;

    is_can_payload p = {};
    CAN_ISB_INS_to_CAN_altitude(&src, &p);
    CAN_CAN_altitude_to_ISB_INS_1(&p, &out);

    EXPECT_NEAR(out.lla[2], src.lla[2], 1e-3);
}

TEST(protocol_CAN, roundtrip_ned_north_east)
{
    ins_1_t src = {}, out = {};
    src.ned[0] = 100.5f;
    src.ned[1] = -200.3f;

    is_can_payload p = {};
    CAN_ISB_INS_to_CAN_ned_north_east(&src, &p);
    CAN_CAN_ned_north_east_to_ISB_INS_1(&p, &out);

    EXPECT_FLOAT_EQ(out.ned[0], src.ned[0]);
    EXPECT_FLOAT_EQ(out.ned[1], src.ned[1]);
}

TEST(protocol_CAN, roundtrip_ned_down)
{
    ins_1_t src = {}, out = {};
    src.ned[2]    = -50.5f;
    src.insStatus = 0x00F00000u;

    is_can_payload p = {};
    CAN_ISB_INS_to_CAN_ned_down(&src, &p);
    CAN_CAN_ned_down_to_ISB_INS_1(&p, &out);

    EXPECT_FLOAT_EQ(out.ned[2], src.ned[2]);
    EXPECT_EQ(out.insStatus, src.insStatus);
}

TEST(protocol_CAN, roundtrip_ecef_x)
{
    ins_4_t src = {}, out = {};
    src.ecef[0] = -2703779.123;

    is_can_payload p = {};
    CAN_ISB_INS_to_CAN_ecef_x(&src, &p);
    CAN_CAN_ecef_x_to_ISB_INS_4(&p, &out);

    EXPECT_DOUBLE_EQ(out.ecef[0], src.ecef[0]);
}

TEST(protocol_CAN, roundtrip_ecef_y)
{
    ins_4_t src = {}, out = {};
    src.ecef[1] = -4354914.456;

    is_can_payload p = {};
    CAN_ISB_INS_to_CAN_ecef_y(&src, &p);
    CAN_CAN_ecef_y_to_ISB_INS_4(&p, &out);

    EXPECT_DOUBLE_EQ(out.ecef[1], src.ecef[1]);
}

TEST(protocol_CAN, roundtrip_ecef_z)
{
    ins_4_t src = {}, out = {};
    src.ecef[2] = 4452679.789;

    is_can_payload p = {};
    CAN_ISB_INS_to_CAN_ecef_z(&src, &p);
    CAN_CAN_ecef_z_to_ISB_INS_4(&p, &out);

    EXPECT_DOUBLE_EQ(out.ecef[2], src.ecef[2]);
}

TEST(protocol_CAN, roundtrip_msl)
{
    ins_3_t src = {}, out = {};
    src.msl = 1388.25f;

    is_can_payload p = {};
    CAN_ISB_INS_to_CAN_msl(&src, &p);
    CAN_CAN_msl_to_ISB_INS_3(&p, &out);

    EXPECT_FLOAT_EQ(out.msl, src.msl);
}

TEST(protocol_CAN, roundtrip_gnss1_pos)
{
    gnss_pos_t src = {}, out = {};
    src.status  = 0x00000C12u;
    src.cnoMean = 38.0f;

    is_can_payload p = {};
    CAN_ISB_GNSS_to_CAN_gnss1_pos(&src, &p);
    CAN_CAN_gnss1_pos_to_ISB_GNSS_POS(&p, &out);

    EXPECT_EQ(out.status, src.status);
    EXPECT_FLOAT_EQ(out.cnoMean, src.cnoMean);
}

TEST(protocol_CAN, roundtrip_gnss2_pos)
{
    gnss_pos_t src = {}, out = {};
    src.status  = 0x00001008u;
    src.cnoMean = 42.0f;

    is_can_payload p = {};
    CAN_ISB_GNSS_to_CAN_gnss2_pos(&src, &p);
    CAN_CAN_gnss2_pos_to_ISB_GNSS_POS(&p, &out);

    EXPECT_EQ(out.status, src.status);
    EXPECT_FLOAT_EQ(out.cnoMean, src.cnoMean);
}

TEST(protocol_CAN, roundtrip_rtk_pos_rel)
{
    gnss_rtk_rel_t src = {}, out = {};
    src.arRatio             = 3.0f;     // uint8_t on wire: truncates to 3
    src.differentialAge     = 1.0f;
    src.baseToRoverDistance = 245.6f;
    src.rtkHeading  = 1.234f;

    is_can_payload p = {};
    CAN_ISB_GNSS_to_CAN_gnss1_rtk_pos_rel(&src, &p);
    CAN_CAN_gnss1_rtk_pos_rel_to_ISB_GNSS_RTK_REL(&p, &out);

    EXPECT_FLOAT_EQ(out.arRatio,         (float)(uint8_t)src.arRatio);
    EXPECT_FLOAT_EQ(out.differentialAge, (float)(uint8_t)src.differentialAge);
    EXPECT_FLOAT_EQ(out.baseToRoverDistance, src.baseToRoverDistance);
    EXPECT_NEAR(out.rtkHeading, src.rtkHeading, 1e-3f);
}

TEST(protocol_CAN, roundtrip_rtk_cmp_rel)
{
    gnss_rtk_rel_t src = {}, out = {};
    src.arRatio             = 5.0f;
    src.differentialAge     = 2.0f;
    src.baseToRoverDistance = 123.4f;
    src.rtkHeading          = -0.567f;

    is_can_payload p = {};
    CAN_ISB_GNSS_to_CAN_gnss2_rtk_cmp_rel(&src, &p);
    CAN_CAN_gnss2_rtk_cmp_rel_to_ISB_GNSS_RTK_REL(&p, &out);

    EXPECT_FLOAT_EQ(out.arRatio,         (float)(uint8_t)src.arRatio);
    EXPECT_FLOAT_EQ(out.differentialAge, (float)(uint8_t)src.differentialAge);
    EXPECT_FLOAT_EQ(out.baseToRoverDistance, src.baseToRoverDistance);
    EXPECT_NEAR(out.rtkHeading, src.rtkHeading, 1e-3f);
}

TEST(protocol_CAN, roundtrip_pimu_px)
{
    pimu_t src = {}, out = {};
    src.theta[0] = 0.123f;
    src.vel[0]   = 4.56f;
    src.dt       = 0.005f;

    is_can_payload p = {};
    CAN_ISB_PIMU_to_CAN_pimu_px(&src, &p);
    CAN_CAN_pimu_px_to_ISB_PIMU(&p, &out);

    EXPECT_NEAR(out.theta[0], src.theta[0], 1e-3f);
    EXPECT_NEAR(out.vel[0],   src.vel[0],   0.01f);
    EXPECT_NEAR(out.dt,       src.dt,        1e-3f);
}

TEST(protocol_CAN, roundtrip_pimu_qy)
{
    pimu_t src = {}, out = {};
    src.theta[1] = -0.234f;
    src.vel[1]   = -7.89f;
    src.dt       = 0.01f;

    is_can_payload p = {};
    CAN_ISB_PIMU_to_CAN_pimu_qy(&src, &p);
    CAN_CAN_pimu_qy_to_ISB_PIMU(&p, &out);

    EXPECT_NEAR(out.theta[1], src.theta[1], 1e-3f);
    EXPECT_NEAR(out.vel[1],   src.vel[1],   0.01f);
    EXPECT_NEAR(out.dt,       src.dt,        1e-3f);
}

TEST(protocol_CAN, roundtrip_pimu_rz)
{
    pimu_t src = {}, out = {};
    src.theta[2] = 0.456f;
    src.vel[2]   = 1.23f;
    src.dt       = 0.02f;

    is_can_payload p = {};
    CAN_ISB_PIMU_to_CAN_pimu_rz(&src, &p);
    CAN_CAN_pimu_rz_to_ISB_PIMU(&p, &out);

    EXPECT_NEAR(out.theta[2], src.theta[2], 1e-3f);
    EXPECT_NEAR(out.vel[2],   src.vel[2],   0.01f);
    EXPECT_NEAR(out.dt,       src.dt,        1e-3f);
}

TEST(protocol_CAN, roundtrip_dual_imu_px)
{
    imu_t src = {}, out = {};
    src.I.pqr[0] = 0.567f;
    src.I.acc[0] = 9.81f;
    src.status   = 0x00000003u;

    is_can_payload p = {};
    CAN_ISB_IMU_to_CAN_dual_imu_px(&src, &p);
    CAN_CAN_dual_imu_px_to_ISB_IMU(&p, &out);

    EXPECT_NEAR(out.I.pqr[0], src.I.pqr[0], 1e-3f);
    EXPECT_NEAR(out.I.acc[0], src.I.acc[0],  0.01f);
    EXPECT_EQ(out.status, src.status);
}

TEST(protocol_CAN, roundtrip_dual_imu_qy)
{
    imu_t src = {}, out = {};
    src.I.pqr[1] = -0.234f;
    src.I.acc[1] = -1.23f;
    src.status   = 0x00000001u;

    is_can_payload p = {};
    CAN_ISB_IMU_to_CAN_dual_imu_qy(&src, &p);
    CAN_CAN_dual_imu_qy_to_ISB_IMU(&p, &out);

    EXPECT_NEAR(out.I.pqr[1], src.I.pqr[1], 1e-3f);
    EXPECT_NEAR(out.I.acc[1], src.I.acc[1],  0.01f);
    EXPECT_EQ(out.status, src.status);
}

TEST(protocol_CAN, roundtrip_dual_imu_rz)
{
    imu_t src = {}, out = {};
    src.I.pqr[2] = 0.789f;
    src.I.acc[2] = 4.56f;
    src.status   = 0x00000002u;

    is_can_payload p = {};
    CAN_ISB_IMU_to_CAN_dual_imu_rz(&src, &p);
    CAN_CAN_dual_imu_rz_to_ISB_IMU(&p, &out);

    EXPECT_NEAR(out.I.pqr[2], src.I.pqr[2], 1e-3f);
    EXPECT_NEAR(out.I.acc[2], src.I.acc[2],  0.01f);
    EXPECT_EQ(out.status, src.status);
}

TEST(protocol_CAN, roundtrip_roll_rollRate)
{
    // Set a non-trivial roll angle via the global quaternion.
    // Roll of 0.3 rad around the body X axis → qn2b = {cos(0.15), sin(0.15), 0, 0}
    float halfAngle = 0.15f;
    g_insOut = {};
    g_insOut.qn2b[0] = cosf(halfAngle);
    g_insOut.qn2b[1] = sinf(halfAngle);
    g_insOut.qn2b[2] = 0.0f;
    g_insOut.qn2b[3] = 0.0f;

    imu_t src = {}, out = {};
    src.I.pqr[0] = 0.314f;

    is_can_payload p = {};
    CAN_ISB_IMU_to_CAN_roll_rollRate(&src, &p);
    // Only pqr[0] round-trips; insRoll comes from global g_insOut, not imu_t.
    CAN_CAN_roll_rollRate_to_ISB_IMU(&p, &out);

    EXPECT_NEAR(out.I.pqr[0], src.I.pqr[0], 1e-3f);
    // Verify the roll packed from g_insOut is approximately 0.3 rad.
    EXPECT_NEAR(p.rollrollrate.insRoll / 10000.0f, 2.0f * halfAngle, 5e-4f);
}

// ============================================================================
// Dispatch
// ============================================================================

TEST(protocol_CAN, dispatch_unknown_cid_returns_minus1)
{
    is_can_payload p = {};
    int sz = CAN_ISB_dispatch(nullptr, NUM_CIDS, &p);
    EXPECT_EQ(sz, -1);
}

TEST(protocol_CAN, dispatch_routes_correctly)
{
    ins_1_t ins1 = {};
    ins1.week = 1234;
    ins1.timeOfWeek = 56789.0;

    is_can_payload p = {};
    int sz = CAN_ISB_dispatch(&ins1, CID_INS_TIME, &p);

    EXPECT_EQ(sz, (int)sizeof(is_can_time));
    EXPECT_EQ(p.time.week, ins1.week);
    EXPECT_NEAR(p.time.timeOfWeekMs, (float)ins1.timeOfWeek, 1e-3f);
}

TEST(protocol_CAN, dispatch_covers_all_cids)
{
    // Verify dispatch returns > 0 (not unrecognised) for every valid CID.
    // Data pointers are zeroed structs — output values are not checked here,
    // only that each CID is handled without returning -1.
    ins_1_t   ins1  = {};
    ins_2_t   ins2  = {};
    ins_3_t   ins3  = {};
    ins_4_t   ins4  = {};
    imu_t     imu   = {};
    pimu_t    pimu  = {};
    gnss_pos_t gnssPos = {};
    gnss_rtk_rel_t rtkRel = {};

    g_gnssNav[0].pos.status = 0;
    g_insOut = {};
    g_insOut.qn2b[0] = 1.0f;   // identity quaternion

    struct { can_cid_t cid; void* ptr; } cases[] = {
        { CID_INS_TIME,          &ins1    },
        { CID_INS_STATUS,        &ins1    },
        { CID_INS_EULER,         &ins1    },
        { CID_INS_QUATN2B,       &ins2    },
        { CID_INS_QUATE2B,       &ins4    },
        { CID_INS_UVW,           &ins1    },
        { CID_INS_VE,            &ins4    },
        { CID_INS_LAT,           &ins1    },
        { CID_INS_LON,           &ins1    },
        { CID_INS_ALT,           &ins1    },
        { CID_INS_NORTH_EAST,    &ins1    },
        { CID_INS_DOWN,          &ins1    },
        { CID_INS_ECEF_X,        &ins4    },
        { CID_INS_ECEF_Y,        &ins4    },
        { CID_INS_ECEF_Z,        &ins4    },
        { CID_INS_MSL,           &ins3    },
        { CID_PREINT_PX,         &pimu   },
        { CID_PREINT_QY,         &pimu   },
        { CID_PREINT_RZ,         &pimu   },
        { CID_DUAL_PX,           &imu    },
        { CID_DUAL_QY,           &imu    },
        { CID_DUAL_RZ,           &imu    },
        { CID_GNSS1_POS,         &gnssPos },
        { CID_GNSS2_POS,         &gnssPos },
        { CID_GNSS1_RTK_POS_REL, &rtkRel  },
        { CID_GNSS2_RTK_CMP_REL, &rtkRel  },
        { CID_ROLL_ROLLRATE,     &imu    },
    };
    static_assert(sizeof(cases)/sizeof(cases[0]) == NUM_CIDS,
        "dispatch_covers_all_cids: case count must match NUM_CIDS");

    for (auto& c : cases)
    {
        is_can_payload p = {};
        int sz = CAN_ISB_dispatch(c.ptr, c.cid, &p);
        EXPECT_GT(sz, 0) << "CID " << (int)c.cid << " returned " << sz;
    }
}
