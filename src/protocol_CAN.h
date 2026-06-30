#ifndef PROTOCOL_CAN_H_
#define PROTOCOL_CAN_H_

#include "data_sets.h"
#include "data_sets_canbus.h"

#include <stdint.h>

// ============================================================================
// ISB -> CAN
// Each function packs the relevant fields of an IS data structure into the
// corresponding member of a CAN payload union and returns the number of bytes
// the caller should transmit (sizeof of the specific is_can_xxx struct).
// ============================================================================

/**
 * @brief Pack GPS week and time-of-week from an INS-1 solution into a CAN time payload.
 * @param ins1  Source INS-1 data structure.
 * @param out   CAN payload union to write; populates the @c time member.
 * @return      Bytes to transmit: sizeof(is_can_time).
 */
int CAN_ISB_INS_to_CAN_time(ins_1_t* ins1, is_can_payload* out);

/**
 * @brief Pack INS and hardware status flags from an INS-1 solution into a CAN status payload.
 * @param ins1  Source INS-1 data structure.
 * @param out   CAN payload union to write; populates the @c insstatus member.
 * @return      Bytes to transmit: sizeof(is_can_ins_status).
 */
int CAN_ISB_INS_to_CAN_ins_status(ins_1_t* ins1, is_can_payload* out);

/**
 * @brief Pack Euler angles (roll, pitch, yaw) from an INS-1 solution into a CAN Euler payload.
 *        Values are scaled by 10000 and stored as int16_t.
 * @param ins1  Source INS-1 data structure.
 * @param out   CAN payload union to write; populates the @c euler member.
 * @return      Bytes to transmit: sizeof(is_can_ins_euler).
 */
int CAN_ISB_INS_to_CAN_ins_euler(ins_1_t* ins1, is_can_payload* out);

/**
 * @brief Pack the NED quaternion (qn2b) from an INS-2 solution into a CAN quaternion payload.
 *        Components are scaled by 10000 and stored as int16_t.
 * @param ins   Source INS-2 data structure.
 * @param out   CAN payload union to write; populates the @c quatn2b member.
 * @return      Bytes to transmit: sizeof(is_can_ins_quatn2b).
 */
int CAN_ISB_INS_to_CAN_quatn2b(ins_2_t* ins, is_can_payload* out);

/**
 * @brief Pack the ECEF quaternion (qe2b) from an INS-4 solution into a CAN quaternion payload.
 *        Components are scaled by 10000 and stored as int16_t.
 * @param ins   Source INS-4 data structure.
 * @param out   CAN payload union to write; populates the @c quate2b member.
 * @return      Bytes to transmit: sizeof(is_can_ins_quate2b).
 */
int CAN_ISB_INS_to_CAN_quate2b(ins_4_t* ins, is_can_payload* out);

/**
 * @brief Pack body-frame velocity (U, V, W) from an INS-1 solution into a CAN UVW payload.
 *        Values are scaled by 100 and stored as int16_t.
 * @param ins1  Source INS-1 data structure.
 * @param out   CAN payload union to write; populates the @c uvw member.
 * @return      Bytes to transmit: sizeof(is_can_uvw).
 */
int CAN_ISB_INS_to_CAN_uvw(ins_1_t* ins1, is_can_payload* out);

/**
 * @brief Pack ECEF velocity from an INS-4 solution into a CAN velocity payload.
 *        Values are scaled by 100 and stored as int16_t.
 * @param ins   Source INS-4 data structure.
 * @param out   CAN payload union to write; populates the @c ve member.
 * @return      Bytes to transmit: sizeof(is_can_ve).
 */
int CAN_ISB_INS_to_CAN_ve(ins_4_t* ins, is_can_payload* out);

/**
 * @brief Pack WGS-84 latitude (degrees) from an INS-1 solution into a CAN latitude payload.
 * @param ins1  Source INS-1 data structure.
 * @param out   CAN payload union to write; populates the @c lat member.
 * @return      Bytes to transmit: sizeof(is_can_ins_lat).
 */
int CAN_ISB_INS_to_CAN_latitude(ins_1_t* ins1, is_can_payload* out);

/**
 * @brief Pack WGS-84 longitude (degrees) from an INS-1 solution into a CAN longitude payload.
 * @param ins1  Source INS-1 data structure.
 * @param out   CAN payload union to write; populates the @c lon member.
 * @return      Bytes to transmit: sizeof(is_can_ins_lon).
 */
int CAN_ISB_INS_to_CAN_longitude(ins_1_t* ins1, is_can_payload* out);

/**
 * @brief Pack WGS-84 altitude and GNSS status into a CAN altitude payload.
 *        GNSS status is sourced from the global @c g_gnssNav[0].pos.status.
 * @param ins1  Source INS-1 data structure.
 * @param out   CAN payload union to write; populates the @c alt member.
 * @return      Bytes to transmit: sizeof(is_can_ins_alt).
 */
int CAN_ISB_INS_to_CAN_altitude(ins_1_t* ins1, is_can_payload* out);

/**
 * @brief Pack north and east NED offsets from an INS-1 solution into a CAN NED north/east payload.
 * @param ins1  Source INS-1 data structure.
 * @param out   CAN payload union to write; populates the @c ne member.
 * @return      Bytes to transmit: sizeof(is_can_north_east).
 */
int CAN_ISB_INS_to_CAN_ned_north_east(ins_1_t* ins1, is_can_payload* out);

/**
 * @brief Pack the down NED offset and INS status from an INS-1 solution into a CAN NED-down payload.
 * @param ins1  Source INS-1 data structure.
 * @param out   CAN payload union to write; populates the @c down member.
 * @return      Bytes to transmit: sizeof(is_can_down).
 */
int CAN_ISB_INS_to_CAN_ned_down(ins_1_t* ins1, is_can_payload* out);

/**
 * @brief Pack the ECEF X position from an INS-4 solution into a CAN ECEF-X payload.
 * @param ins   Source INS-4 data structure.
 * @param out   CAN payload union to write; populates the @c ecefx member.
 * @return      Bytes to transmit: sizeof(is_can_ecef_x).
 */
int CAN_ISB_INS_to_CAN_ecef_x(ins_4_t* ins, is_can_payload* out);

/**
 * @brief Pack the ECEF Y position from an INS-4 solution into a CAN ECEF-Y payload.
 * @param ins   Source INS-4 data structure.
 * @param out   CAN payload union to write; populates the @c ecefy member.
 * @return      Bytes to transmit: sizeof(is_can_ecef_y).
 */
int CAN_ISB_INS_to_CAN_ecef_y(ins_4_t* ins, is_can_payload* out);

/**
 * @brief Pack the ECEF Z position from an INS-4 solution into a CAN ECEF-Z payload.
 * @param ins   Source INS-4 data structure.
 * @param out   CAN payload union to write; populates the @c ecefz member.
 * @return      Bytes to transmit: sizeof(is_can_ecef_z).
 */
int CAN_ISB_INS_to_CAN_ecef_z(ins_4_t* ins, is_can_payload* out);

/**
 * @brief Pack mean sea level altitude from an INS-3 solution into a CAN MSL payload.
 * @param ins   Source INS-3 data structure.
 * @param out   CAN payload union to write; populates the @c msl member.
 * @return      Bytes to transmit: sizeof(is_can_msl).
 */
int CAN_ISB_INS_to_CAN_msl(ins_3_t* ins, is_can_payload* out);

/**
 * @brief Pack GNSS-1 fix status and mean CNO into a CAN GNSS position status payload.
 * @param gnss  Source GNSS position data structure.
 * @param out   CAN payload union to write; populates the @c gnsspos member.
 * @return      Bytes to transmit: sizeof(is_can_gnss_pos_status).
 */
int CAN_ISB_GNSS_to_CAN_gnss1_pos(gnss_pos_t* gnss, is_can_payload* out);

/**
 * @brief Pack GNSS-2 fix status and mean CNO into a CAN GNSS position status payload.
 * @param gnss  Source GNSS position data structure.
 * @param out   CAN payload union to write; populates the @c gnsspos member.
 * @return      Bytes to transmit: sizeof(is_can_gnss_pos_status).
 */
int CAN_ISB_GNSS_to_CAN_gnss2_pos(gnss_pos_t* gnss, is_can_payload* out);

/**
 * @brief Pack GNSS-1 RTK position-relative data into a CAN RTK-relative payload.
 *        Heading is scaled by 1000 and stored as int16_t.
 * @param gnss  Source RTK relative data structure.
 * @param out   CAN payload union to write; populates the @c rtkrel member.
 * @return      Bytes to transmit: sizeof(is_can_gnss_rtk_rel).
 */
int CAN_ISB_GNSS_to_CAN_gnss1_rtk_pos_rel(gnss_rtk_rel_t* gnss, is_can_payload* out);

/**
 * @brief Pack GNSS-2 RTK compassing-relative data into a CAN RTK-relative payload.
 *        Heading is scaled by 1000 and stored as int16_t.
 * @param gnss  Source RTK relative data structure.
 * @param out   CAN payload union to write; populates the @c rtkrel member.
 * @return      Bytes to transmit: sizeof(is_can_gnss_rtk_rel).
 */
int CAN_ISB_GNSS_to_CAN_gnss2_rtk_cmp_rel(gnss_rtk_rel_t* gnss, is_can_payload* out);

/**
 * @brief Pack the X-axis preintegrated IMU data (theta, velocity, dt) into a CAN PIMU-PX payload.
 *        theta scaled by 1000, velocity by 100, dt by 1000, all as int16_t/uint16_t.
 * @param pimu  Source preintegrated IMU data structure.
 * @param out   CAN payload union to write; populates the @c pimupx member.
 * @return      Bytes to transmit: sizeof(is_can_preint_imu_px).
 */
int CAN_ISB_PIMU_to_CAN_pimu_px(pimu_t* pimu, is_can_payload* out);

/**
 * @brief Pack the Y-axis preintegrated IMU data (theta, velocity, dt) into a CAN PIMU-QY payload.
 *        theta scaled by 1000, velocity by 100, dt by 1000, all as int16_t/uint16_t.
 * @param pimu  Source preintegrated IMU data structure.
 * @param out   CAN payload union to write; populates the @c pimuqy member.
 * @return      Bytes to transmit: sizeof(is_can_preint_imu_qy).
 */
int CAN_ISB_PIMU_to_CAN_pimu_qy(pimu_t* pimu, is_can_payload* out);

/**
 * @brief Pack the Z-axis preintegrated IMU data (theta, velocity, dt) into a CAN PIMU-RZ payload.
 *        theta scaled by 1000, velocity by 100, dt by 1000, all as int16_t/uint16_t.
 * @param pimu  Source preintegrated IMU data structure.
 * @param out   CAN payload union to write; populates the @c pimurz member.
 * @return      Bytes to transmit: sizeof(is_can_preint_imu_rz).
 */
int CAN_ISB_PIMU_to_CAN_pimu_rz(pimu_t* pimu, is_can_payload* out);

/**
 * @brief Pack X-axis dual-IMU angular rate and acceleration into a CAN dual-IMU-PX payload.
 *        pqr scaled by 1000, acc by 100, both as int16_t.
 * @param imu   Source IMU data structure.
 * @param out   CAN payload union to write; populates the @c dimupx member.
 * @return      Bytes to transmit: sizeof(is_can_dual_imu_px).
 */
int CAN_ISB_IMU_to_CAN_dual_imu_px(imu_t* imu, is_can_payload* out);

/**
 * @brief Pack Y-axis dual-IMU angular rate and acceleration into a CAN dual-IMU-QY payload.
 *        pqr scaled by 1000, acc by 100, both as int16_t.
 * @param imu   Source IMU data structure.
 * @param out   CAN payload union to write; populates the @c dimuqy member.
 * @return      Bytes to transmit: sizeof(is_can_dual_imu_qy).
 */
int CAN_ISB_IMU_to_CAN_dual_imu_qy(imu_t* imu, is_can_payload* out);

/**
 * @brief Pack Z-axis dual-IMU angular rate and acceleration into a CAN dual-IMU-RZ payload.
 *        pqr scaled by 1000, acc by 100, both as int16_t.
 * @param imu   Source IMU data structure.
 * @param out   CAN payload union to write; populates the @c dimurz member.
 * @return      Bytes to transmit: sizeof(is_can_dual_imu_rz).
 */
int CAN_ISB_IMU_to_CAN_dual_imu_rz(imu_t* imu, is_can_payload* out);

/**
 * @brief Pack roll angle (from global g_insOut quaternion) and roll rates into a CAN roll/roll-rate payload.
 *        insRoll scaled by 10000; pqr[0] scaled by 1000, stored twice as pImu1 and pImu2.
 * @param imu   Source IMU data structure.
 * @param out   CAN payload union to write; populates the @c rollrollrate member.
 * @return      Bytes to transmit: sizeof(is_can_roll_rollRate).
 */
int CAN_ISB_IMU_to_CAN_roll_rollRate(imu_t* imu, is_can_payload* out);

// ============================================================================
// CAN -> ISB
// Each function unpacks the relevant member of a CAN payload union back into
// the corresponding IS data structure.  Only the fields carried by that CAN
// message are written; all other fields in *out are left unchanged.
// Returns the number of bytes consumed (sizeof of the specific is_can_xxx struct).
// ============================================================================

/**
 * @brief Unpack GPS week and time-of-week from a CAN time payload into an INS-1 struct.
 * @param in   CAN payload union to read; consumes the @c time member.
 * @param out  Target INS-1 structure; writes @c week and @c timeOfWeek.
 * @return     Bytes consumed: sizeof(is_can_time).
 */
int CAN_CAN_time_to_ISB_INS_1(is_can_payload* in, ins_1_t* out);

/**
 * @brief Unpack INS and hardware status flags from a CAN status payload into an INS-1 struct.
 * @param in   CAN payload union to read; consumes the @c insstatus member.
 * @param out  Target INS-1 structure; writes @c insStatus and @c hdwStatus.
 * @return     Bytes consumed: sizeof(is_can_ins_status).
 */
int CAN_CAN_ins_status_to_ISB_INS_1(is_can_payload* in, ins_1_t* out);

/**
 * @brief Unpack Euler angles from a CAN Euler payload into an INS-1 struct.
 *        Applies inverse scale 1/10000.
 * @param in   CAN payload union to read; consumes the @c euler member.
 * @param out  Target INS-1 structure; writes @c theta[0..2].
 * @return     Bytes consumed: sizeof(is_can_ins_euler).
 */
int CAN_CAN_ins_euler_to_ISB_INS_1(is_can_payload* in, ins_1_t* out);

/**
 * @brief Unpack the NED quaternion from a CAN quaternion payload into an INS-2 struct.
 *        Applies inverse scale 1/10000.
 * @param in   CAN payload union to read; consumes the @c quatn2b member.
 * @param out  Target INS-2 structure; writes @c qn2b[0..3].
 * @return     Bytes consumed: sizeof(is_can_ins_quatn2b).
 */
int CAN_CAN_quatn2b_to_ISB_INS_2(is_can_payload* in, ins_2_t* out);

/**
 * @brief Unpack the ECEF quaternion from a CAN quaternion payload into an INS-4 struct.
 *        Applies inverse scale 1/10000.
 * @param in   CAN payload union to read; consumes the @c quate2b member.
 * @param out  Target INS-4 structure; writes @c qe2b[0..3].
 * @return     Bytes consumed: sizeof(is_can_ins_quate2b).
 */
int CAN_CAN_quate2b_to_ISB_INS_4(is_can_payload* in, ins_4_t* out);

/**
 * @brief Unpack body-frame velocity from a CAN UVW payload into an INS-1 struct.
 *        Applies inverse scale 1/100.
 * @param in   CAN payload union to read; consumes the @c uvw member.
 * @param out  Target INS-1 structure; writes @c uvw[0..2].
 * @return     Bytes consumed: sizeof(is_can_uvw).
 */
int CAN_CAN_uvw_to_ISB_INS_1(is_can_payload* in, ins_1_t* out);

/**
 * @brief Unpack ECEF velocity from a CAN velocity payload into an INS-4 struct.
 *        Applies inverse scale 1/100.
 * @param in   CAN payload union to read; consumes the @c ve member.
 * @param out  Target INS-4 structure; writes @c ve[0..2].
 * @return     Bytes consumed: sizeof(is_can_ve).
 */
int CAN_CAN_ve_to_ISB_INS_4(is_can_payload* in, ins_4_t* out);

/**
 * @brief Unpack WGS-84 latitude from a CAN latitude payload into an INS-1 struct.
 * @param in   CAN payload union to read; consumes the @c lat member.
 * @param out  Target INS-1 structure; writes @c lla[0].
 * @return     Bytes consumed: sizeof(is_can_ins_lat).
 */
int CAN_CAN_latitude_to_ISB_INS_1(is_can_payload* in, ins_1_t* out);

/**
 * @brief Unpack WGS-84 longitude from a CAN longitude payload into an INS-1 struct.
 * @param in   CAN payload union to read; consumes the @c lon member.
 * @param out  Target INS-1 structure; writes @c lla[1].
 * @return     Bytes consumed: sizeof(is_can_ins_lon).
 */
int CAN_CAN_longitude_to_ISB_INS_1(is_can_payload* in, ins_1_t* out);

/**
 * @brief Unpack WGS-84 altitude from a CAN altitude payload into an INS-1 struct.
 *        Note: the GNSS status also present in the payload is not in ins_1_t;
 *        use CAN_CAN_gnss1_pos_to_ISB_GNSS_POS to recover it separately.
 * @param in   CAN payload union to read; consumes the @c alt member.
 * @param out  Target INS-1 structure; writes @c lla[2].
 * @return     Bytes consumed: sizeof(is_can_ins_alt).
 */
int CAN_CAN_altitude_to_ISB_INS_1(is_can_payload* in, ins_1_t* out);

/**
 * @brief Unpack north and east NED offsets from a CAN NED north/east payload into an INS-1 struct.
 * @param in   CAN payload union to read; consumes the @c ne member.
 * @param out  Target INS-1 structure; writes @c ned[0] and @c ned[1].
 * @return     Bytes consumed: sizeof(is_can_north_east).
 */
int CAN_CAN_ned_north_east_to_ISB_INS_1(is_can_payload* in, ins_1_t* out);

/**
 * @brief Unpack the down NED offset and INS status from a CAN NED-down payload into an INS-1 struct.
 * @param in   CAN payload union to read; consumes the @c down member.
 * @param out  Target INS-1 structure; writes @c ned[2] and @c insStatus.
 * @return     Bytes consumed: sizeof(is_can_down).
 */
int CAN_CAN_ned_down_to_ISB_INS_1(is_can_payload* in, ins_1_t* out);

/**
 * @brief Unpack the ECEF X position from a CAN ECEF-X payload into an INS-4 struct.
 * @param in   CAN payload union to read; consumes the @c ecefx member.
 * @param out  Target INS-4 structure; writes @c ecef[0].
 * @return     Bytes consumed: sizeof(is_can_ecef_x).
 */
int CAN_CAN_ecef_x_to_ISB_INS_4(is_can_payload* in, ins_4_t* out);

/**
 * @brief Unpack the ECEF Y position from a CAN ECEF-Y payload into an INS-4 struct.
 * @param in   CAN payload union to read; consumes the @c ecefy member.
 * @param out  Target INS-4 structure; writes @c ecef[1].
 * @return     Bytes consumed: sizeof(is_can_ecef_y).
 */
int CAN_CAN_ecef_y_to_ISB_INS_4(is_can_payload* in, ins_4_t* out);

/**
 * @brief Unpack the ECEF Z position from a CAN ECEF-Z payload into an INS-4 struct.
 * @param in   CAN payload union to read; consumes the @c ecefz member.
 * @param out  Target INS-4 structure; writes @c ecef[2].
 * @return     Bytes consumed: sizeof(is_can_ecef_z).
 */
int CAN_CAN_ecef_z_to_ISB_INS_4(is_can_payload* in, ins_4_t* out);

/**
 * @brief Unpack mean sea level altitude from a CAN MSL payload into an INS-3 struct.
 * @param in   CAN payload union to read; consumes the @c msl member.
 * @param out  Target INS-3 structure; writes @c msl.
 * @return     Bytes consumed: sizeof(is_can_msl).
 */
int CAN_CAN_msl_to_ISB_INS_3(is_can_payload* in, ins_3_t* out);

/**
 * @brief Unpack GNSS-1 fix status and mean CNO from a CAN GNSS position status payload.
 * @param in   CAN payload union to read; consumes the @c gnsspos member.
 * @param out  Target GNSS position structure; writes @c status and @c cnoMean.
 * @return     Bytes consumed: sizeof(is_can_gnss_pos_status).
 */
int CAN_CAN_gnss1_pos_to_ISB_GNSS_POS(is_can_payload* in, gnss_pos_t* out);

/**
 * @brief Unpack GNSS-2 fix status and mean CNO from a CAN GNSS position status payload.
 * @param in   CAN payload union to read; consumes the @c gnsspos member.
 * @param out  Target GNSS position structure; writes @c status and @c cnoMean.
 * @return     Bytes consumed: sizeof(is_can_gnss_pos_status).
 */
int CAN_CAN_gnss2_pos_to_ISB_GNSS_POS(is_can_payload* in, gnss_pos_t* out);

/**
 * @brief Unpack GNSS-1 RTK position-relative data from a CAN RTK payload.
 *        Applies inverse scale 1/1000 to heading.
 * @param in   CAN payload union to read; consumes the @c rtkrel member.
 * @param out  Target RTK relative structure; writes @c arRatio, @c differentialAge,
 *             @c baseToRoverDistance, and @c baseToRoverHeading.
 * @return     Bytes consumed: sizeof(is_can_gnss_rtk_rel).
 */
int CAN_CAN_gnss1_rtk_pos_rel_to_ISB_GNSS_RTK_REL(is_can_payload* in, gnss_rtk_rel_t* out);

/**
 * @brief Unpack GNSS-2 RTK compassing-relative data from a CAN RTK payload.
 *        Applies inverse scale 1/1000 to heading.
 * @param in   CAN payload union to read; consumes the @c rtkrel member.
 * @param out  Target RTK relative structure; writes @c arRatio, @c differentialAge,
 *             @c baseToRoverDistance, and @c baseToRoverHeading.
 * @return     Bytes consumed: sizeof(is_can_gnss_rtk_rel).
 */
int CAN_CAN_gnss2_rtk_cmp_rel_to_ISB_GNSS_RTK_REL(is_can_payload* in, gnss_rtk_rel_t* out);

/**
 * @brief Unpack X-axis preintegrated IMU data from a CAN PIMU-PX payload.
 *        Applies inverse scales: theta 1/1000, velocity 1/100, dt 1/1000.
 * @param in   CAN payload union to read; consumes the @c pimupx member.
 * @param out  Target PIMU structure; writes @c theta[0], @c vel[0], and @c dt.
 * @return     Bytes consumed: sizeof(is_can_preint_imu_px).
 */
int CAN_CAN_pimu_px_to_ISB_PIMU(is_can_payload* in, pimu_t* out);

/**
 * @brief Unpack Y-axis preintegrated IMU data from a CAN PIMU-QY payload.
 *        Applies inverse scales: theta 1/1000, velocity 1/100, dt 1/1000.
 * @param in   CAN payload union to read; consumes the @c pimuqy member.
 * @param out  Target PIMU structure; writes @c theta[1], @c vel[1], and @c dt.
 * @return     Bytes consumed: sizeof(is_can_preint_imu_qy).
 */
int CAN_CAN_pimu_qy_to_ISB_PIMU(is_can_payload* in, pimu_t* out);

/**
 * @brief Unpack Z-axis preintegrated IMU data from a CAN PIMU-RZ payload.
 *        Applies inverse scales: theta 1/1000, velocity 1/100, dt 1/1000.
 * @param in   CAN payload union to read; consumes the @c pimurz member.
 * @param out  Target PIMU structure; writes @c theta[2], @c vel[2], and @c dt.
 * @return     Bytes consumed: sizeof(is_can_preint_imu_rz).
 */
int CAN_CAN_pimu_rz_to_ISB_PIMU(is_can_payload* in, pimu_t* out);

/**
 * @brief Unpack X-axis dual-IMU data from a CAN dual-IMU-PX payload.
 *        Applies inverse scales: pqr 1/1000, acc 1/100.
 * @param in   CAN payload union to read; consumes the @c dimupx member.
 * @param out  Target IMU structure; writes @c I.pqr[0], @c I.acc[0], and @c status.
 * @return     Bytes consumed: sizeof(is_can_dual_imu_px).
 */
int CAN_CAN_dual_imu_px_to_ISB_IMU(is_can_payload* in, imu_t* out);

/**
 * @brief Unpack Y-axis dual-IMU data from a CAN dual-IMU-QY payload.
 *        Applies inverse scales: pqr 1/1000, acc 1/100.
 * @param in   CAN payload union to read; consumes the @c dimuqy member.
 * @param out  Target IMU structure; writes @c I.pqr[1], @c I.acc[1], and @c status.
 * @return     Bytes consumed: sizeof(is_can_dual_imu_qy).
 */
int CAN_CAN_dual_imu_qy_to_ISB_IMU(is_can_payload* in, imu_t* out);

/**
 * @brief Unpack Z-axis dual-IMU data from a CAN dual-IMU-RZ payload.
 *        Applies inverse scales: pqr 1/1000, acc 1/100.
 * @param in   CAN payload union to read; consumes the @c dimurz member.
 * @param out  Target IMU structure; writes @c I.pqr[2], @c I.acc[2], and @c status.
 * @return     Bytes consumed: sizeof(is_can_dual_imu_rz).
 */
int CAN_CAN_dual_imu_rz_to_ISB_IMU(is_can_payload* in, imu_t* out);

/**
 * @brief Unpack roll rate from a CAN roll/roll-rate payload into an IMU struct.
 *        Only @c I.pqr[0] is recoverable; the roll angle (insRoll) is an INS
 *        navigation output not present in imu_t.
 *        Applies inverse scale 1/1000 to pImu1.
 * @param in   CAN payload union to read; consumes the @c rollrollrate member.
 * @param out  Target IMU structure; writes @c I.pqr[0].
 * @return     Bytes consumed: sizeof(is_can_roll_rollRate).
 */
int CAN_CAN_roll_rollRate_to_ISB_IMU(is_can_payload* in, imu_t* out);

#endif // PROTOCOL_CAN_H_
