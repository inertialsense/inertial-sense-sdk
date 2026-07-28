#include "protocol_CAN.h"
#include "data_sets_canbus.h"
#include "ISPose.h"

// Forward declarations for firmware globals used by two functions below.
// Defined in families/imx/globals.cpp in firmware builds;
// stub definitions are provided in test_protocol_CAN.cpp for SDK unit-test builds.
typedef struct { gnss_pos_t pos; gnss_vel_t vel; } gnss_posvel_t;

#ifdef IS_IMX
// IMX firmware: defined in families/imx/globals.cpp
extern gnss_posvel_t g_gnssNav[2];
extern ins_output_t g_insOut;
#elif PLATFORM_IS_EMBEDDED
// GPX / other non-IMX embedded: no globals.cpp and no test file; own the storage here.
gnss_posvel_t g_gnssNav[2];
ins_output_t g_insOut;
#else
// SDK host (lib + unit-test) builds: forward-declare only.
// Definitions are provided by test_protocol_CAN.cpp when linked into unit tests.
extern gnss_posvel_t g_gnssNav[2];
extern ins_output_t g_insOut;
#endif

// ============================================================================
// ISB -> CAN
// ============================================================================

/** @brief Pack GPS week and time-of-week into a CAN time payload. */
int CAN_ISB_INS_to_CAN_time(ins_1_t* ins1, is_can_payload* out)
{
    *out = {};
    out->time.week         = ins1->week;
    out->time.timeOfWeekMs = static_cast<float>(ins1->timeOfWeek);
    return (int)sizeof(is_can_time);
}

/** @brief Pack INS and hardware status flags into a CAN status payload. */
int CAN_ISB_INS_to_CAN_ins_status(ins_1_t* ins1, is_can_payload* out)
{
    *out = {};
    out->insstatus.insStatus = ins1->insStatus;
    out->insstatus.hdwStatus = ins1->hdwStatus;
    return (int)sizeof(is_can_ins_status);
}

/** @brief Pack Euler angles (roll, pitch, yaw, scaled x10000) into a CAN Euler payload. */
int CAN_ISB_INS_to_CAN_ins_euler(ins_1_t* ins1, is_can_payload* out)
{
    *out = {};
    out->euler.theta1 = static_cast<int16_t>(ins1->theta[0]*10000);
    out->euler.theta2 = static_cast<int16_t>(ins1->theta[1]*10000);
    out->euler.theta3 = static_cast<int16_t>(ins1->theta[2]*10000);
    return (int)sizeof(is_can_ins_euler);
}

/** @brief Pack the NED quaternion (qn2b, scaled x10000) into a CAN quaternion payload. */
int CAN_ISB_INS_to_CAN_quatn2b(ins_2_t* ins, is_can_payload* out)
{
    *out = {};
    out->quatn2b.qn2b1 = static_cast<int16_t>(ins->qn2b[0]*10000);
    out->quatn2b.qn2b2 = static_cast<int16_t>(ins->qn2b[1]*10000);
    out->quatn2b.qn2b3 = static_cast<int16_t>(ins->qn2b[2]*10000);
    out->quatn2b.qn2b4 = static_cast<int16_t>(ins->qn2b[3]*10000);
    return (int)sizeof(is_can_ins_quatn2b);
}

/** @brief Pack the ECEF quaternion (qe2b, scaled x10000) into a CAN quaternion payload. */
int CAN_ISB_INS_to_CAN_quate2b(ins_4_t* ins, is_can_payload* out)
{
    *out = {};
    out->quate2b.qe2b1 = static_cast<int16_t>(ins->qe2b[0]*10000);
    out->quate2b.qe2b2 = static_cast<int16_t>(ins->qe2b[1]*10000);
    out->quate2b.qe2b3 = static_cast<int16_t>(ins->qe2b[2]*10000);
    out->quate2b.qe2b4 = static_cast<int16_t>(ins->qe2b[3]*10000);
    return (int)sizeof(is_can_ins_quate2b);
}

/** @brief Pack body-frame velocity (U, V, W, scaled x100) into a CAN UVW payload. */
int CAN_ISB_INS_to_CAN_uvw(ins_1_t* ins1, is_can_payload* out)
{
    *out = {};
    out->uvw.uvw1 = static_cast<int16_t>(ins1->uvw[0]*100);
    out->uvw.uvw2 = static_cast<int16_t>(ins1->uvw[1]*100);
    out->uvw.uvw3 = static_cast<int16_t>(ins1->uvw[2]*100);
    return (int)sizeof(is_can_uvw);
}

/** @brief Pack ECEF velocity (scaled x100) into a CAN velocity payload. */
int CAN_ISB_INS_to_CAN_ve(ins_4_t* ins, is_can_payload* out)
{
    *out = {};
    out->ve.ve1 = static_cast<int16_t>(ins->ve[0]*100);
    out->ve.ve2 = static_cast<int16_t>(ins->ve[1]*100);
    out->ve.ve3 = static_cast<int16_t>(ins->ve[2]*100);
    return (int)sizeof(is_can_ve);
}

/** @brief Pack WGS-84 latitude (degrees) into a CAN latitude payload. */
int CAN_ISB_INS_to_CAN_latitude(ins_1_t* ins1, is_can_payload* out)
{
    *out = {};
    out->lat.lat = ins1->lla[0];
    return (int)sizeof(is_can_ins_lat);
}

/** @brief Pack WGS-84 longitude (degrees) into a CAN longitude payload. */
int CAN_ISB_INS_to_CAN_longitude(ins_1_t* ins1, is_can_payload* out)
{
    *out = {};
    out->lon.lon = ins1->lla[1];
    return (int)sizeof(is_can_ins_lon);
}

/** @brief Pack WGS-84 altitude and GNSS status (from g_gnssNav[0]) into a CAN altitude payload. */
int CAN_ISB_INS_to_CAN_altitude(ins_1_t* ins1, is_can_payload* out)
{
    *out = {};
    out->alt.alt    = static_cast<float>(ins1->lla[2]);
    out->alt.status = g_gnssNav[0].pos.status;
    return (int)sizeof(is_can_ins_alt);
}

/** @brief Pack north and east NED offsets into a CAN NED north/east payload. */
int CAN_ISB_INS_to_CAN_ned_north_east(ins_1_t* ins1, is_can_payload* out)
{
    *out = {};
    out->ne.ned1 = ins1->ned[0];
    out->ne.ned2 = ins1->ned[1];
    return (int)sizeof(is_can_north_east);
}

/** @brief Pack the down NED offset and INS status into a CAN NED-down payload. */
int CAN_ISB_INS_to_CAN_ned_down(ins_1_t* ins1, is_can_payload* out)
{
    *out = {};
    out->down.ned3      = ins1->ned[2];
    out->down.insStatus = ins1->insStatus;
    return (int)sizeof(is_can_down);
}

/** @brief Pack ECEF X position into a CAN ECEF-X payload. */
int CAN_ISB_INS_to_CAN_ecef_x(ins_4_t* ins, is_can_payload* out)
{
    *out = {};
    out->ecefx.ecef1 = ins->ecef[0];
    return (int)sizeof(is_can_ecef_x);
}

/** @brief Pack ECEF Y position into a CAN ECEF-Y payload. */
int CAN_ISB_INS_to_CAN_ecef_y(ins_4_t* ins, is_can_payload* out)
{
    *out = {};
    out->ecefy.ecef2 = ins->ecef[1];
    return (int)sizeof(is_can_ecef_y);
}

/** @brief Pack ECEF Z position into a CAN ECEF-Z payload. */
int CAN_ISB_INS_to_CAN_ecef_z(ins_4_t* ins, is_can_payload* out)
{
    *out = {};
    out->ecefz.ecef3 = ins->ecef[2];
    return (int)sizeof(is_can_ecef_z);
}

/** @brief Pack mean sea level altitude into a CAN MSL payload. */
int CAN_ISB_INS_to_CAN_msl(ins_3_t* ins, is_can_payload* out)
{
    *out = {};
    out->msl.msl = ins->msl;
    return (int)sizeof(is_can_msl);
}

/** @brief Pack GNSS-1 fix status and mean CNO into a CAN GNSS position status payload. */
int CAN_ISB_GNSS_to_CAN_gnss1_pos(gnss_pos_t* gnss, is_can_payload* out)
{
    *out = {};
    out->gnsspos.status  = gnss->status;
    out->gnsspos.cnoMean = static_cast<uint32_t>(gnss->cnoMean);
    return (int)sizeof(is_can_gnss_pos_status);
}

/** @brief Pack GNSS-2 fix status and mean CNO into a CAN GNSS position status payload. */
int CAN_ISB_GNSS_to_CAN_gnss2_pos(gnss_pos_t* gnss, is_can_payload* out)
{
    *out = {};
    out->gnsspos.status  = gnss->status;
    out->gnsspos.cnoMean = static_cast<uint32_t>(gnss->cnoMean);
    return (int)sizeof(is_can_gnss_pos_status);
}

/** @brief Pack GNSS-1 RTK position-relative data (heading scaled x1000) into a CAN RTK payload. */
int CAN_ISB_GNSS_to_CAN_gnss1_rtk_pos_rel(gnss_rtk_rel_t* gnss, is_can_payload* out)
{
    *out = {};
    out->rtkrel.arRatio         = static_cast<uint8_t>(gnss->arRatio);
    out->rtkrel.differentialAge = static_cast<uint8_t>(gnss->differentialAge);
    out->rtkrel.distanceToBase  = gnss->baseToRoverDistance;
    out->rtkrel.rtkHeading      = static_cast<int16_t>(gnss->rtkHeading*1000);
    return (int)sizeof(is_can_gnss_rtk_rel);
}

/** @brief Pack GNSS-2 RTK compassing-relative data (heading scaled x1000) into a CAN RTK payload. */
int CAN_ISB_GNSS_to_CAN_gnss2_rtk_cmp_rel(gnss_rtk_rel_t* gnss, is_can_payload* out)
{
    *out = {};
    out->rtkrel.arRatio         = static_cast<uint8_t>(gnss->arRatio);
    out->rtkrel.differentialAge = static_cast<uint8_t>(gnss->differentialAge);
    out->rtkrel.distanceToBase  = gnss->baseToRoverDistance;
    out->rtkrel.rtkHeading      = static_cast<int16_t>(gnss->rtkHeading*1000);
    return (int)sizeof(is_can_gnss_rtk_rel);
}

/** @brief Pack X-axis preintegrated IMU data (theta x1000, vel x100, dt x1000) into a CAN PIMU-PX payload. */
int CAN_ISB_PIMU_to_CAN_pimu_px(pimu_t* pimu, is_can_payload* out)
{
    *out = {};
    out->pimupx.theta0 = static_cast<int16_t>(pimu->theta[0]*1000);
    out->pimupx.vel0   = static_cast<int16_t>(pimu->vel[0]*100);
    out->pimupx.dt     = static_cast<uint16_t>(pimu->dt*1000);
    return (int)sizeof(is_can_preint_imu_px);
}

/** @brief Pack Y-axis preintegrated IMU data (theta x1000, vel x100, dt x1000) into a CAN PIMU-QY payload. */
int CAN_ISB_PIMU_to_CAN_pimu_qy(pimu_t* pimu, is_can_payload* out)
{
    *out = {};
    out->pimuqy.theta1 = static_cast<int16_t>(pimu->theta[1]*1000);
    out->pimuqy.vel1   = static_cast<int16_t>(pimu->vel[1]*100);
    out->pimuqy.dt     = static_cast<uint16_t>(pimu->dt*1000);
    return (int)sizeof(is_can_preint_imu_qy);
}

/** @brief Pack Z-axis preintegrated IMU data (theta x1000, vel x100, dt x1000) into a CAN PIMU-RZ payload. */
int CAN_ISB_PIMU_to_CAN_pimu_rz(pimu_t* pimu, is_can_payload* out)
{
    *out = {};
    out->pimurz.theta2 = static_cast<int16_t>(pimu->theta[2]*1000);
    out->pimurz.vel2   = static_cast<int16_t>(pimu->vel[2]*100);
    out->pimurz.dt     = static_cast<uint16_t>(pimu->dt*1000);
    return (int)sizeof(is_can_preint_imu_rz);
}

/** @brief Pack X-axis dual-IMU angular rate (pqr x1000) and acceleration (acc x100) into a CAN dual-IMU-PX payload. */
int CAN_ISB_IMU_to_CAN_dual_imu_px(imu_t* imu, is_can_payload* out)
{
    *out = {};
    out->dimupx.theta0 = static_cast<int16_t>(imu->I.pqr[0]*1000);
    out->dimupx.vel0   = static_cast<int16_t>(imu->I.acc[0]*100);
    out->dimupx.status = imu->status;
    return (int)sizeof(is_can_dual_imu_px);
}

/** @brief Pack Y-axis dual-IMU angular rate (pqr x1000) and acceleration (acc x100) into a CAN dual-IMU-QY payload. */
int CAN_ISB_IMU_to_CAN_dual_imu_qy(imu_t* imu, is_can_payload* out)
{
    *out = {};
    out->dimuqy.theta1 = static_cast<int16_t>(imu->I.pqr[1]*1000);
    out->dimuqy.vel1   = static_cast<int16_t>(imu->I.acc[1]*100);
    out->dimuqy.status = imu->status;
    return (int)sizeof(is_can_dual_imu_qy);
}

/** @brief Pack Z-axis dual-IMU angular rate (pqr x1000) and acceleration (acc x100) into a CAN dual-IMU-RZ payload. */
int CAN_ISB_IMU_to_CAN_dual_imu_rz(imu_t* imu, is_can_payload* out)
{
    *out = {};
    out->dimurz.theta2 = static_cast<int16_t>(imu->I.pqr[2]*1000);
    out->dimurz.vel2   = static_cast<int16_t>(imu->I.acc[2]*100);
    out->dimurz.status = imu->status;
    return (int)sizeof(is_can_dual_imu_rz);
}

/** @brief Pack roll angle (from g_insOut quaternion, x10000) and roll rate (pqr[0] x1000) into a CAN roll/roll-rate payload. */
int CAN_ISB_IMU_to_CAN_roll_rollRate(imu_t* imu, is_can_payload* out)
{
    *out = {};
    float theta[3];
    quat2euler(g_insOut.qn2b, theta);
    out->rollrollrate.insRoll = static_cast<int16_t>(theta[0]*10000);
    out->rollrollrate.pImu1   = static_cast<int16_t>(imu->I.pqr[0]*1000);
    out->rollrollrate.pImu2   = static_cast<int16_t>(imu->I.pqr[0]*1000);
    return (int)sizeof(is_can_roll_rollRate);
}

// ============================================================================
// CAN -> ISB
// ============================================================================

/** @brief Unpack GPS week and time-of-week from a CAN time payload into an INS-1 struct. */
int CAN_CAN_time_to_ISB_INS_1(is_can_payload* in, ins_1_t* out)
{
    out->week       = in->time.week;
    out->timeOfWeek = static_cast<double>(in->time.timeOfWeekMs);
    return (int)sizeof(is_can_time);
}

/** @brief Unpack INS and hardware status flags from a CAN status payload into an INS-1 struct. */
int CAN_CAN_ins_status_to_ISB_INS_1(is_can_payload* in, ins_1_t* out)
{
    out->insStatus = in->insstatus.insStatus;
    out->hdwStatus = in->insstatus.hdwStatus;
    return (int)sizeof(is_can_ins_status);
}

/** @brief Unpack Euler angles (inverse scale 1/10000) from a CAN Euler payload into an INS-1 struct. */
int CAN_CAN_ins_euler_to_ISB_INS_1(is_can_payload* in, ins_1_t* out)
{
    out->theta[0] = in->euler.theta1 / 10000.0f;
    out->theta[1] = in->euler.theta2 / 10000.0f;
    out->theta[2] = in->euler.theta3 / 10000.0f;
    return (int)sizeof(is_can_ins_euler);
}

/** @brief Unpack the NED quaternion (inverse scale 1/10000) from a CAN quaternion payload into an INS-2 struct. */
int CAN_CAN_quatn2b_to_ISB_INS_2(is_can_payload* in, ins_2_t* out)
{
    out->qn2b[0] = in->quatn2b.qn2b1 / 10000.0f;
    out->qn2b[1] = in->quatn2b.qn2b2 / 10000.0f;
    out->qn2b[2] = in->quatn2b.qn2b3 / 10000.0f;
    out->qn2b[3] = in->quatn2b.qn2b4 / 10000.0f;
    return (int)sizeof(is_can_ins_quatn2b);
}

/** @brief Unpack the ECEF quaternion (inverse scale 1/10000) from a CAN quaternion payload into an INS-4 struct. */
int CAN_CAN_quate2b_to_ISB_INS_4(is_can_payload* in, ins_4_t* out)
{
    out->qe2b[0] = in->quate2b.qe2b1 / 10000.0f;
    out->qe2b[1] = in->quate2b.qe2b2 / 10000.0f;
    out->qe2b[2] = in->quate2b.qe2b3 / 10000.0f;
    out->qe2b[3] = in->quate2b.qe2b4 / 10000.0f;
    return (int)sizeof(is_can_ins_quate2b);
}

/** @brief Unpack body-frame velocity (inverse scale 1/100) from a CAN UVW payload into an INS-1 struct. */
int CAN_CAN_uvw_to_ISB_INS_1(is_can_payload* in, ins_1_t* out)
{
    out->uvw[0] = in->uvw.uvw1 / 100.0f;
    out->uvw[1] = in->uvw.uvw2 / 100.0f;
    out->uvw[2] = in->uvw.uvw3 / 100.0f;
    return (int)sizeof(is_can_uvw);
}

/** @brief Unpack ECEF velocity (inverse scale 1/100) from a CAN velocity payload into an INS-4 struct. */
int CAN_CAN_ve_to_ISB_INS_4(is_can_payload* in, ins_4_t* out)
{
    out->ve[0] = in->ve.ve1 / 100.0f;
    out->ve[1] = in->ve.ve2 / 100.0f;
    out->ve[2] = in->ve.ve3 / 100.0f;
    return (int)sizeof(is_can_ve);
}

/** @brief Unpack WGS-84 latitude from a CAN latitude payload into an INS-1 struct. */
int CAN_CAN_latitude_to_ISB_INS_1(is_can_payload* in, ins_1_t* out)
{
    out->lla[0] = in->lat.lat;
    return (int)sizeof(is_can_ins_lat);
}

/** @brief Unpack WGS-84 longitude from a CAN longitude payload into an INS-1 struct. */
int CAN_CAN_longitude_to_ISB_INS_1(is_can_payload* in, ins_1_t* out)
{
    out->lla[1] = in->lon.lon;
    return (int)sizeof(is_can_ins_lon);
}

/** @brief Unpack WGS-84 altitude from a CAN altitude payload into an INS-1 struct.
 *         GNSS status in the payload is not in ins_1_t; use CAN_CAN_gnss1_pos_to_ISB_GNSS_POS to recover it. */
int CAN_CAN_altitude_to_ISB_INS_1(is_can_payload* in, ins_1_t* out)
{
    out->lla[2] = static_cast<double>(in->alt.alt);
    return (int)sizeof(is_can_ins_alt);
}

/** @brief Unpack north and east NED offsets from a CAN NED north/east payload into an INS-1 struct. */
int CAN_CAN_ned_north_east_to_ISB_INS_1(is_can_payload* in, ins_1_t* out)
{
    out->ned[0] = in->ne.ned1;
    out->ned[1] = in->ne.ned2;
    return (int)sizeof(is_can_north_east);
}

/** @brief Unpack the down NED offset and INS status from a CAN NED-down payload into an INS-1 struct. */
int CAN_CAN_ned_down_to_ISB_INS_1(is_can_payload* in, ins_1_t* out)
{
    out->ned[2]    = in->down.ned3;
    out->insStatus = in->down.insStatus;
    return (int)sizeof(is_can_down);
}

/** @brief Unpack ECEF X position from a CAN ECEF-X payload into an INS-4 struct. */
int CAN_CAN_ecef_x_to_ISB_INS_4(is_can_payload* in, ins_4_t* out)
{
    out->ecef[0] = in->ecefx.ecef1;
    return (int)sizeof(is_can_ecef_x);
}

/** @brief Unpack ECEF Y position from a CAN ECEF-Y payload into an INS-4 struct. */
int CAN_CAN_ecef_y_to_ISB_INS_4(is_can_payload* in, ins_4_t* out)
{
    out->ecef[1] = in->ecefy.ecef2;
    return (int)sizeof(is_can_ecef_y);
}

/** @brief Unpack ECEF Z position from a CAN ECEF-Z payload into an INS-4 struct. */
int CAN_CAN_ecef_z_to_ISB_INS_4(is_can_payload* in, ins_4_t* out)
{
    out->ecef[2] = in->ecefz.ecef3;
    return (int)sizeof(is_can_ecef_z);
}

/** @brief Unpack mean sea level altitude from a CAN MSL payload into an INS-3 struct. */
int CAN_CAN_msl_to_ISB_INS_3(is_can_payload* in, ins_3_t* out)
{
    out->msl = in->msl.msl;
    return (int)sizeof(is_can_msl);
}

/** @brief Unpack GNSS-1 fix status and mean CNO from a CAN GNSS position status payload. */
int CAN_CAN_gnss1_pos_to_ISB_GNSS_POS(is_can_payload* in, gnss_pos_t* out)
{
    out->status  = in->gnsspos.status;
    out->cnoMean = static_cast<float>(in->gnsspos.cnoMean);
    return (int)sizeof(is_can_gnss_pos_status);
}

/** @brief Unpack GNSS-2 fix status and mean CNO from a CAN GNSS position status payload. */
int CAN_CAN_gnss2_pos_to_ISB_GNSS_POS(is_can_payload* in, gnss_pos_t* out)
{
    out->status  = in->gnsspos.status;
    out->cnoMean = static_cast<float>(in->gnsspos.cnoMean);
    return (int)sizeof(is_can_gnss_pos_status);
}

/** @brief Unpack GNSS-1 RTK position-relative data (heading inverse scale 1/1000) from a CAN RTK payload. */
int CAN_CAN_gnss1_rtk_pos_rel_to_ISB_GNSS_RTK_REL(is_can_payload* in, gnss_rtk_rel_t* out)
{
    out->arRatio                = static_cast<float>(in->rtkrel.arRatio);
    out->differentialAge        = static_cast<float>(in->rtkrel.differentialAge);
    out->baseToRoverDistance    = in->rtkrel.distanceToBase;
    out->rtkHeading             = in->rtkrel.rtkHeading / 1000.0f;
    return (int)sizeof(is_can_gnss_rtk_rel);
}

/** @brief Unpack GNSS-2 RTK compassing-relative data (heading inverse scale 1/1000) from a CAN RTK payload. */
int CAN_CAN_gnss2_rtk_cmp_rel_to_ISB_GNSS_RTK_REL(is_can_payload* in, gnss_rtk_rel_t* out)
{
    out->arRatio                = static_cast<float>(in->rtkrel.arRatio);
    out->differentialAge        = static_cast<float>(in->rtkrel.differentialAge);
    out->baseToRoverDistance    = in->rtkrel.distanceToBase;
    out->rtkHeading             = in->rtkrel.rtkHeading / 1000.0f;
    return (int)sizeof(is_can_gnss_rtk_rel);
}

/** @brief Unpack X-axis preintegrated IMU data (theta 1/1000, vel 1/100, dt 1/1000) from a CAN PIMU-PX payload. */
int CAN_CAN_pimu_px_to_ISB_PIMU(is_can_payload* in, pimu_t* out)
{
    out->theta[0] = in->pimupx.theta0 / 1000.0f;
    out->vel[0]   = in->pimupx.vel0   / 100.0f;
    out->dt       = in->pimupx.dt     / 1000.0f;
    return (int)sizeof(is_can_preint_imu_px);
}

/** @brief Unpack Y-axis preintegrated IMU data (theta 1/1000, vel 1/100, dt 1/1000) from a CAN PIMU-QY payload. */
int CAN_CAN_pimu_qy_to_ISB_PIMU(is_can_payload* in, pimu_t* out)
{
    out->theta[1] = in->pimuqy.theta1 / 1000.0f;
    out->vel[1]   = in->pimuqy.vel1   / 100.0f;
    out->dt       = in->pimuqy.dt     / 1000.0f;
    return (int)sizeof(is_can_preint_imu_qy);
}

/** @brief Unpack Z-axis preintegrated IMU data (theta 1/1000, vel 1/100, dt 1/1000) from a CAN PIMU-RZ payload. */
int CAN_CAN_pimu_rz_to_ISB_PIMU(is_can_payload* in, pimu_t* out)
{
    out->theta[2] = in->pimurz.theta2 / 1000.0f;
    out->vel[2]   = in->pimurz.vel2   / 100.0f;
    out->dt       = in->pimurz.dt     / 1000.0f;
    return (int)sizeof(is_can_preint_imu_rz);
}

/** @brief Unpack X-axis dual-IMU data (pqr 1/1000, acc 1/100) from a CAN dual-IMU-PX payload. */
int CAN_CAN_dual_imu_px_to_ISB_IMU(is_can_payload* in, imu_t* out)
{
    out->I.pqr[0] = in->dimupx.theta0 / 1000.0f;
    out->I.acc[0] = in->dimupx.vel0   / 100.0f;
    out->status   = in->dimupx.status;
    return (int)sizeof(is_can_dual_imu_px);
}

/** @brief Unpack Y-axis dual-IMU data (pqr 1/1000, acc 1/100) from a CAN dual-IMU-QY payload. */
int CAN_CAN_dual_imu_qy_to_ISB_IMU(is_can_payload* in, imu_t* out)
{
    out->I.pqr[1] = in->dimuqy.theta1 / 1000.0f;
    out->I.acc[1] = in->dimuqy.vel1   / 100.0f;
    out->status   = in->dimuqy.status;
    return (int)sizeof(is_can_dual_imu_qy);
}

/** @brief Unpack Z-axis dual-IMU data (pqr 1/1000, acc 1/100) from a CAN dual-IMU-RZ payload. */
int CAN_CAN_dual_imu_rz_to_ISB_IMU(is_can_payload* in, imu_t* out)
{
    out->I.pqr[2] = in->dimurz.theta2 / 1000.0f;
    out->I.acc[2] = in->dimurz.vel2   / 100.0f;
    out->status   = in->dimurz.status;
    return (int)sizeof(is_can_dual_imu_rz);
}

/** @brief Unpack roll rate (pImu1 1/1000) from a CAN roll/roll-rate payload.
 *         insRoll is an INS navigation output not in imu_t; only pqr[0] is recovered. */
int CAN_CAN_roll_rollRate_to_ISB_IMU(is_can_payload* in, imu_t* out)
{
    out->I.pqr[0] = in->rollrollrate.pImu1 / 1000.0f;
    return (int)sizeof(is_can_roll_rollRate);
}

// ============================================================================
// CAN FD encode (ISB → wire)
// ============================================================================

/** @brief Encode DID_INS_1 into a 64-byte CAN FD INS-1 payload. See protocol_CAN.h for full doc. */
int CANFD_ISB_INS1_to_CANFD(ins_1_t* ins, is_canfd_payload* out)
{
    out->ins1.week       = ins->week;
    out->ins1.timeOfWeek = ins->timeOfWeek;
    out->ins1.insStatus  = ins->insStatus;
    out->ins1.hdwStatus  = ins->hdwStatus;
    out->ins1.theta[0]   = ins->theta[0];
    out->ins1.theta[1]   = ins->theta[1];
    out->ins1.theta[2]   = ins->theta[2];
    out->ins1.uvw[0]     = ins->uvw[0];
    out->ins1.uvw[1]     = ins->uvw[1];
    out->ins1.uvw[2]     = ins->uvw[2];
    out->ins1.lat        = ins->lla[0];
    out->ins1.lon        = ins->lla[1];
    out->ins1.alt        = (float)ins->lla[2];
    return (int)sizeof(is_canfd_ins1);
}

/** @brief Encode DID_INS_2 quaternion into a 16-byte CAN FD INS-2 payload. See protocol_CAN.h for full doc. */
int CANFD_ISB_INS2_to_CANFD(ins_2_t* ins, is_canfd_payload* out)
{
    out->ins2.qn2b[0] = ins->qn2b[0];
    out->ins2.qn2b[1] = ins->qn2b[1];
    out->ins2.qn2b[2] = ins->qn2b[2];
    out->ins2.qn2b[3] = ins->qn2b[3];
    return (int)sizeof(is_canfd_ins2);
}

/** @brief Encode DID_INS_3 MSL altitude into a 4-byte CAN FD INS-3 payload. See protocol_CAN.h for full doc. */
int CANFD_ISB_INS3_to_CANFD(ins_3_t* ins, is_canfd_payload* out)
{
    out->ins3.msl = ins->msl;
    return (int)sizeof(is_canfd_ins3);
}

/** @brief Encode DID_INS_4 into a 52-byte CAN FD INS-4 payload (64-byte frame, zero-padded). See protocol_CAN.h for full doc. */
int CANFD_ISB_INS4_to_CANFD(ins_4_t* ins, is_canfd_payload* out)
{
    out->ins4.qe2b[0] = ins->qe2b[0];
    out->ins4.qe2b[1] = ins->qe2b[1];
    out->ins4.qe2b[2] = ins->qe2b[2];
    out->ins4.qe2b[3] = ins->qe2b[3];
    out->ins4.ve[0]   = ins->ve[0];
    out->ins4.ve[1]   = ins->ve[1];
    out->ins4.ve[2]   = ins->ve[2];
    out->ins4.ecef[0] = ins->ecef[0];
    out->ins4.ecef[1] = ins->ecef[1];
    out->ins4.ecef[2] = ins->ecef[2];
    return (int)sizeof(is_canfd_ins4);
}

/** @brief Encode DID_PIMU into a 32-byte CAN FD PIMU payload. See protocol_CAN.h for full doc. */
int CANFD_ISB_PIMU_to_CANFD(pimu_t* pimu, is_canfd_payload* out)
{
    out->pimu.theta[0] = pimu->theta[0];
    out->pimu.theta[1] = pimu->theta[1];
    out->pimu.theta[2] = pimu->theta[2];
    out->pimu.vel[0]   = pimu->vel[0];
    out->pimu.vel[1]   = pimu->vel[1];
    out->pimu.vel[2]   = pimu->vel[2];
    out->pimu.dt       = pimu->dt;
    out->pimu.status   = pimu->status;
    return (int)sizeof(is_canfd_pimu);
}

/** @brief Encode DID_IMU into a 28-byte CAN FD IMU payload (32-byte frame, zero-padded). See protocol_CAN.h for full doc. */
int CANFD_ISB_IMU_to_CANFD(imu_t* imu, is_canfd_payload* out)
{
    out->imu.pqr[0]  = imu->I.pqr[0];
    out->imu.pqr[1]  = imu->I.pqr[1];
    out->imu.pqr[2]  = imu->I.pqr[2];
    out->imu.acc[0]  = imu->I.acc[0];
    out->imu.acc[1]  = imu->I.acc[1];
    out->imu.acc[2]  = imu->I.acc[2];
    out->imu.status  = imu->status;
    return (int)sizeof(is_canfd_imu);
}

/** @brief Encode DID_GNSS1_POS status and CNO into an 8-byte CAN FD GNSS-1 position payload. See protocol_CAN.h for full doc. */
int CANFD_ISB_GNSS1_POS_to_CANFD(gnss_pos_t* gnss, is_canfd_payload* out)
{
    out->gnsspos.status  = gnss->status;
    out->gnsspos.cnoMean = gnss->cnoMean;
    return (int)sizeof(is_canfd_gnss_pos);
}

/** @brief Encode DID_GNSS2_POS status and CNO into an 8-byte CAN FD GNSS-2 position payload. See protocol_CAN.h for full doc. */
int CANFD_ISB_GNSS2_POS_to_CANFD(gnss_pos_t* gnss, is_canfd_payload* out)
{
    out->gnsspos.status  = gnss->status;
    out->gnsspos.cnoMean = gnss->cnoMean;
    return (int)sizeof(is_canfd_gnss_pos);
}

/** @brief Encode DID_GNSS1_RTK_POS_REL into a 16-byte CAN FD RTK relative payload. See protocol_CAN.h for full doc. */
int CANFD_ISB_GNSS1_RTK_REL_to_CANFD(gnss_rtk_rel_t* gnss, is_canfd_payload* out)
{
    out->rtkrel.arRatio         = gnss->arRatio;
    out->rtkrel.differentialAge = gnss->differentialAge;
    out->rtkrel.distanceToBase  = gnss->baseToRoverDistance;
    out->rtkrel.rtkHeading      = gnss->rtkHeading;
    return (int)sizeof(is_canfd_gnss_rtk_rel);
}

/** @brief Encode DID_GNSS2_RTK_CMP_REL into a 16-byte CAN FD RTK relative payload. See protocol_CAN.h for full doc. */
int CANFD_ISB_GNSS2_RTK_REL_to_CANFD(gnss_rtk_rel_t* gnss, is_canfd_payload* out)
{
    out->rtkrel.arRatio         = gnss->arRatio;
    out->rtkrel.differentialAge = gnss->differentialAge;
    out->rtkrel.distanceToBase  = gnss->baseToRoverDistance;
    out->rtkrel.rtkHeading      = gnss->rtkHeading;
    return (int)sizeof(is_canfd_gnss_rtk_rel);
}

// ============================================================================
// CAN FD decode (wire → ISB)
// ============================================================================

/** @brief Decode a CAN FD INS-1 payload into a DID_INS_1 structure. See protocol_CAN.h for full doc. */
int CANFD_CANFD_to_ISB_INS1(is_canfd_payload* in, ins_1_t* out)
{
    out->week        = in->ins1.week;
    out->timeOfWeek  = in->ins1.timeOfWeek;
    out->insStatus   = in->ins1.insStatus;
    out->hdwStatus   = in->ins1.hdwStatus;
    out->theta[0]    = in->ins1.theta[0];
    out->theta[1]    = in->ins1.theta[1];
    out->theta[2]    = in->ins1.theta[2];
    out->uvw[0]      = in->ins1.uvw[0];
    out->uvw[1]      = in->ins1.uvw[1];
    out->uvw[2]      = in->ins1.uvw[2];
    out->lla[0]      = in->ins1.lat;
    out->lla[1]      = in->ins1.lon;
    out->lla[2]      = in->ins1.alt;
    return (int)sizeof(is_canfd_ins1);
}

/** @brief Decode a CAN FD INS-2 payload into a DID_INS_2 structure. See protocol_CAN.h for full doc. */
int CANFD_CANFD_to_ISB_INS2(is_canfd_payload* in, ins_2_t* out)
{
    out->qn2b[0] = in->ins2.qn2b[0];
    out->qn2b[1] = in->ins2.qn2b[1];
    out->qn2b[2] = in->ins2.qn2b[2];
    out->qn2b[3] = in->ins2.qn2b[3];
    return (int)sizeof(is_canfd_ins2);
}

/** @brief Decode a CAN FD INS-3 payload into a DID_INS_3 structure. See protocol_CAN.h for full doc. */
int CANFD_CANFD_to_ISB_INS3(is_canfd_payload* in, ins_3_t* out)
{
    out->msl = in->ins3.msl;
    return (int)sizeof(is_canfd_ins3);
}

/** @brief Decode a CAN FD INS-4 payload into a DID_INS_4 structure. See protocol_CAN.h for full doc. */
int CANFD_CANFD_to_ISB_INS4(is_canfd_payload* in, ins_4_t* out)
{
    out->qe2b[0] = in->ins4.qe2b[0];
    out->qe2b[1] = in->ins4.qe2b[1];
    out->qe2b[2] = in->ins4.qe2b[2];
    out->qe2b[3] = in->ins4.qe2b[3];
    out->ve[0]   = in->ins4.ve[0];
    out->ve[1]   = in->ins4.ve[1];
    out->ve[2]   = in->ins4.ve[2];
    out->ecef[0] = in->ins4.ecef[0];
    out->ecef[1] = in->ins4.ecef[1];
    out->ecef[2] = in->ins4.ecef[2];
    return (int)sizeof(is_canfd_ins4);
}

/** @brief Decode a CAN FD PIMU payload into a DID_PIMU structure. See protocol_CAN.h for full doc. */
int CANFD_CANFD_to_ISB_PIMU(is_canfd_payload* in, pimu_t* out)
{
    out->theta[0] = in->pimu.theta[0];
    out->theta[1] = in->pimu.theta[1];
    out->theta[2] = in->pimu.theta[2];
    out->vel[0]   = in->pimu.vel[0];
    out->vel[1]   = in->pimu.vel[1];
    out->vel[2]   = in->pimu.vel[2];
    out->dt       = in->pimu.dt;
    out->status   = in->pimu.status;
    return (int)sizeof(is_canfd_pimu);
}

/** @brief Decode a CAN FD IMU payload into a DID_IMU structure. See protocol_CAN.h for full doc. */
int CANFD_CANFD_to_ISB_IMU(is_canfd_payload* in, imu_t* out)
{
    out->I.pqr[0] = in->imu.pqr[0];
    out->I.pqr[1] = in->imu.pqr[1];
    out->I.pqr[2] = in->imu.pqr[2];
    out->I.acc[0] = in->imu.acc[0];
    out->I.acc[1] = in->imu.acc[1];
    out->I.acc[2] = in->imu.acc[2];
    out->status   = in->imu.status;
    return (int)sizeof(is_canfd_imu);
}

/** @brief Decode a CAN FD GNSS-1 position payload into a DID_GNSS1_POS structure. See protocol_CAN.h for full doc. */
int CANFD_CANFD_to_ISB_GNSS1_POS(is_canfd_payload* in, gnss_pos_t* out)
{
    out->status  = in->gnsspos.status;
    out->cnoMean = in->gnsspos.cnoMean;
    return (int)sizeof(is_canfd_gnss_pos);
}

/** @brief Decode a CAN FD GNSS-2 position payload into a DID_GNSS2_POS structure. See protocol_CAN.h for full doc. */
int CANFD_CANFD_to_ISB_GNSS2_POS(is_canfd_payload* in, gnss_pos_t* out)
{
    out->status  = in->gnsspos.status;
    out->cnoMean = in->gnsspos.cnoMean;
    return (int)sizeof(is_canfd_gnss_pos);
}

/** @brief Decode a CAN FD RTK relative payload into a DID_GNSS1_RTK_POS_REL structure. See protocol_CAN.h for full doc. */
int CANFD_CANFD_to_ISB_GNSS1_RTK_REL(is_canfd_payload* in, gnss_rtk_rel_t* out)
{
    out->arRatio                = in->rtkrel.arRatio;
    out->differentialAge        = in->rtkrel.differentialAge;
    out->baseToRoverDistance    = in->rtkrel.distanceToBase;
    out->rtkHeading             = in->rtkrel.rtkHeading;
    return (int)sizeof(is_canfd_gnss_rtk_rel);
}

/** @brief Decode a CAN FD RTK relative payload into a DID_GNSS2_RTK_CMP_REL structure. See protocol_CAN.h for full doc. */
int CANFD_CANFD_to_ISB_GNSS2_RTK_REL(is_canfd_payload* in, gnss_rtk_rel_t* out)
{
    out->arRatio                = in->rtkrel.arRatio;
    out->differentialAge        = in->rtkrel.differentialAge;
    out->baseToRoverDistance    = in->rtkrel.distanceToBase;
    out->rtkHeading             = in->rtkrel.rtkHeading;
    return (int)sizeof(is_canfd_gnss_rtk_rel);
}

// ============================================================================
// Dispatch
// ============================================================================

/** @brief Dispatch an ISB-to-CAN conversion by CAN message ID; calls the
 *         appropriate CAN_ISB_*_to_CAN_* function for the given cid. */
int CAN_ISB_dispatch(void* data, can_cid_t cid, is_can_payload* out)
{
    switch (cid)
    {
    case CID_INS_TIME:          return CAN_ISB_INS_to_CAN_time(static_cast<ins_1_t*>(data), out);
    case CID_INS_STATUS:        return CAN_ISB_INS_to_CAN_ins_status(static_cast<ins_1_t*>(data), out);
    case CID_INS_EULER:         return CAN_ISB_INS_to_CAN_ins_euler(static_cast<ins_1_t*>(data), out);
    case CID_INS_QUATN2B:       return CAN_ISB_INS_to_CAN_quatn2b(static_cast<ins_2_t*>(data), out);
    case CID_INS_QUATE2B:       return CAN_ISB_INS_to_CAN_quate2b(static_cast<ins_4_t*>(data), out);
    case CID_INS_UVW:           return CAN_ISB_INS_to_CAN_uvw(static_cast<ins_1_t*>(data), out);
    case CID_INS_VE:            return CAN_ISB_INS_to_CAN_ve(static_cast<ins_4_t*>(data), out);
    case CID_INS_LAT:           return CAN_ISB_INS_to_CAN_latitude(static_cast<ins_1_t*>(data), out);
    case CID_INS_LON:           return CAN_ISB_INS_to_CAN_longitude(static_cast<ins_1_t*>(data), out);
    case CID_INS_ALT:           return CAN_ISB_INS_to_CAN_altitude(static_cast<ins_1_t*>(data), out);
    case CID_INS_NORTH_EAST:    return CAN_ISB_INS_to_CAN_ned_north_east(static_cast<ins_1_t*>(data), out);
    case CID_INS_DOWN:          return CAN_ISB_INS_to_CAN_ned_down(static_cast<ins_1_t*>(data), out);
    case CID_INS_ECEF_X:        return CAN_ISB_INS_to_CAN_ecef_x(static_cast<ins_4_t*>(data), out);
    case CID_INS_ECEF_Y:        return CAN_ISB_INS_to_CAN_ecef_y(static_cast<ins_4_t*>(data), out);
    case CID_INS_ECEF_Z:        return CAN_ISB_INS_to_CAN_ecef_z(static_cast<ins_4_t*>(data), out);
    case CID_INS_MSL:           return CAN_ISB_INS_to_CAN_msl(static_cast<ins_3_t*>(data), out);
    case CID_PREINT_PX:         return CAN_ISB_PIMU_to_CAN_pimu_px(static_cast<pimu_t*>(data), out);
    case CID_PREINT_QY:         return CAN_ISB_PIMU_to_CAN_pimu_qy(static_cast<pimu_t*>(data), out);
    case CID_PREINT_RZ:         return CAN_ISB_PIMU_to_CAN_pimu_rz(static_cast<pimu_t*>(data), out);
    case CID_DUAL_PX:           return CAN_ISB_IMU_to_CAN_dual_imu_px(static_cast<imu_t*>(data), out);
    case CID_DUAL_QY:           return CAN_ISB_IMU_to_CAN_dual_imu_qy(static_cast<imu_t*>(data), out);
    case CID_DUAL_RZ:           return CAN_ISB_IMU_to_CAN_dual_imu_rz(static_cast<imu_t*>(data), out);
    case CID_GNSS1_POS:         return CAN_ISB_GNSS_to_CAN_gnss1_pos(static_cast<gnss_pos_t*>(data), out);
    case CID_GNSS2_POS:         return CAN_ISB_GNSS_to_CAN_gnss2_pos(static_cast<gnss_pos_t*>(data), out);
    case CID_GNSS1_RTK_POS_REL: return CAN_ISB_GNSS_to_CAN_gnss1_rtk_pos_rel(static_cast<gnss_rtk_rel_t*>(data), out);
    case CID_GNSS2_RTK_CMP_REL: return CAN_ISB_GNSS_to_CAN_gnss2_rtk_cmp_rel(static_cast<gnss_rtk_rel_t*>(data), out);
    case CID_ROLL_ROLLRATE:     return CAN_ISB_IMU_to_CAN_roll_rollRate(static_cast<imu_t*>(data), out);
    default:                    return -1;
    }
}

/** @brief Encode an ISB data struct into a CAN FD payload by FDCID. See protocol_CAN.h for full doc. */
int CANFD_ISB_dispatch(void* data, canfd_cid_t fdcid, is_canfd_payload* out)
{
    switch (fdcid)
    {
    case FDCID_INS_1:           return CANFD_ISB_INS1_to_CANFD((ins_1_t*)data, out);
    case FDCID_INS_2:           return CANFD_ISB_INS2_to_CANFD((ins_2_t*)data, out);
    case FDCID_INS_3:           return CANFD_ISB_INS3_to_CANFD((ins_3_t*)data, out);
    case FDCID_INS_4:           return CANFD_ISB_INS4_to_CANFD((ins_4_t*)data, out);
    case FDCID_PIMU:            return CANFD_ISB_PIMU_to_CANFD((pimu_t*)data, out);
    case FDCID_IMU:             return CANFD_ISB_IMU_to_CANFD((imu_t*)data, out);
    case FDCID_GNSS1_POS:       return CANFD_ISB_GNSS1_POS_to_CANFD((gnss_pos_t*)data, out);
    case FDCID_GNSS2_POS:       return CANFD_ISB_GNSS2_POS_to_CANFD((gnss_pos_t*)data, out);
    case FDCID_GNSS1_RTK_POS_REL: return CANFD_ISB_GNSS1_RTK_REL_to_CANFD((gnss_rtk_rel_t*)data, out);
    case FDCID_GNSS2_RTK_CMP_REL: return CANFD_ISB_GNSS2_RTK_REL_to_CANFD((gnss_rtk_rel_t*)data, out);
    default:                    return -1;
    }
}
