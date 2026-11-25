/**
 * @file config_can.c
 * @brief CONFIG_CAN: schemas, recorded defaults and field names.
 *
 * Generated from the CBOR key registry by is-common scripts/cbor_registry.py -- do not edit.
 *
 * @copyright Copyright (c) 2026 Inertial Sense, Inc.
 */

#include <stddef.h>
#include "cbor/cbor_registry.h"

static const cbor_field_t s_fields_config_can_message_ins_time[] = {
    { 0, CBOR_T_UINT, 2, 0, offsetof(cbor_keyset_can_message_t, periodMultiple), NULL },
    { 1, CBOR_T_UINT, 4, 0, offsetof(cbor_keyset_can_message_t, transmitAddress), NULL },
};
static const cbor_schema_t s_schema_config_can_message_ins_time = { s_fields_config_can_message_ins_time, sizeof(s_fields_config_can_message_ins_time) / sizeof(s_fields_config_can_message_ins_time[0]) };

static const cbor_field_t s_fields_config_can_message_ins_status[] = {
    { 0, CBOR_T_UINT, 2, 0, offsetof(cbor_keyset_can_message_t, periodMultiple), NULL },
    { 1, CBOR_T_UINT, 4, 0, offsetof(cbor_keyset_can_message_t, transmitAddress), NULL },
};
static const cbor_schema_t s_schema_config_can_message_ins_status = { s_fields_config_can_message_ins_status, sizeof(s_fields_config_can_message_ins_status) / sizeof(s_fields_config_can_message_ins_status[0]) };

static const cbor_field_t s_fields_config_can_message_ins_euler[] = {
    { 0, CBOR_T_UINT, 2, 0, offsetof(cbor_keyset_can_message_t, periodMultiple), NULL },
    { 1, CBOR_T_UINT, 4, 0, offsetof(cbor_keyset_can_message_t, transmitAddress), NULL },
};
static const cbor_schema_t s_schema_config_can_message_ins_euler = { s_fields_config_can_message_ins_euler, sizeof(s_fields_config_can_message_ins_euler) / sizeof(s_fields_config_can_message_ins_euler[0]) };

static const cbor_field_t s_fields_config_can_message_ins_quatn2b[] = {
    { 0, CBOR_T_UINT, 2, 0, offsetof(cbor_keyset_can_message_t, periodMultiple), NULL },
    { 1, CBOR_T_UINT, 4, 0, offsetof(cbor_keyset_can_message_t, transmitAddress), NULL },
};
static const cbor_schema_t s_schema_config_can_message_ins_quatn2b = { s_fields_config_can_message_ins_quatn2b, sizeof(s_fields_config_can_message_ins_quatn2b) / sizeof(s_fields_config_can_message_ins_quatn2b[0]) };

static const cbor_field_t s_fields_config_can_message_ins_quate2b[] = {
    { 0, CBOR_T_UINT, 2, 0, offsetof(cbor_keyset_can_message_t, periodMultiple), NULL },
    { 1, CBOR_T_UINT, 4, 0, offsetof(cbor_keyset_can_message_t, transmitAddress), NULL },
};
static const cbor_schema_t s_schema_config_can_message_ins_quate2b = { s_fields_config_can_message_ins_quate2b, sizeof(s_fields_config_can_message_ins_quate2b) / sizeof(s_fields_config_can_message_ins_quate2b[0]) };

static const cbor_field_t s_fields_config_can_message_ins_uvw[] = {
    { 0, CBOR_T_UINT, 2, 0, offsetof(cbor_keyset_can_message_t, periodMultiple), NULL },
    { 1, CBOR_T_UINT, 4, 0, offsetof(cbor_keyset_can_message_t, transmitAddress), NULL },
};
static const cbor_schema_t s_schema_config_can_message_ins_uvw = { s_fields_config_can_message_ins_uvw, sizeof(s_fields_config_can_message_ins_uvw) / sizeof(s_fields_config_can_message_ins_uvw[0]) };

static const cbor_field_t s_fields_config_can_message_ins_ve[] = {
    { 0, CBOR_T_UINT, 2, 0, offsetof(cbor_keyset_can_message_t, periodMultiple), NULL },
    { 1, CBOR_T_UINT, 4, 0, offsetof(cbor_keyset_can_message_t, transmitAddress), NULL },
};
static const cbor_schema_t s_schema_config_can_message_ins_ve = { s_fields_config_can_message_ins_ve, sizeof(s_fields_config_can_message_ins_ve) / sizeof(s_fields_config_can_message_ins_ve[0]) };

static const cbor_field_t s_fields_config_can_message_ins_lat[] = {
    { 0, CBOR_T_UINT, 2, 0, offsetof(cbor_keyset_can_message_t, periodMultiple), NULL },
    { 1, CBOR_T_UINT, 4, 0, offsetof(cbor_keyset_can_message_t, transmitAddress), NULL },
};
static const cbor_schema_t s_schema_config_can_message_ins_lat = { s_fields_config_can_message_ins_lat, sizeof(s_fields_config_can_message_ins_lat) / sizeof(s_fields_config_can_message_ins_lat[0]) };

static const cbor_field_t s_fields_config_can_message_ins_lon[] = {
    { 0, CBOR_T_UINT, 2, 0, offsetof(cbor_keyset_can_message_t, periodMultiple), NULL },
    { 1, CBOR_T_UINT, 4, 0, offsetof(cbor_keyset_can_message_t, transmitAddress), NULL },
};
static const cbor_schema_t s_schema_config_can_message_ins_lon = { s_fields_config_can_message_ins_lon, sizeof(s_fields_config_can_message_ins_lon) / sizeof(s_fields_config_can_message_ins_lon[0]) };

static const cbor_field_t s_fields_config_can_message_ins_alt[] = {
    { 0, CBOR_T_UINT, 2, 0, offsetof(cbor_keyset_can_message_t, periodMultiple), NULL },
    { 1, CBOR_T_UINT, 4, 0, offsetof(cbor_keyset_can_message_t, transmitAddress), NULL },
};
static const cbor_schema_t s_schema_config_can_message_ins_alt = { s_fields_config_can_message_ins_alt, sizeof(s_fields_config_can_message_ins_alt) / sizeof(s_fields_config_can_message_ins_alt[0]) };

static const cbor_field_t s_fields_config_can_message_ins_north_east[] = {
    { 0, CBOR_T_UINT, 2, 0, offsetof(cbor_keyset_can_message_t, periodMultiple), NULL },
    { 1, CBOR_T_UINT, 4, 0, offsetof(cbor_keyset_can_message_t, transmitAddress), NULL },
};
static const cbor_schema_t s_schema_config_can_message_ins_north_east = { s_fields_config_can_message_ins_north_east, sizeof(s_fields_config_can_message_ins_north_east) / sizeof(s_fields_config_can_message_ins_north_east[0]) };

static const cbor_field_t s_fields_config_can_message_ins_down[] = {
    { 0, CBOR_T_UINT, 2, 0, offsetof(cbor_keyset_can_message_t, periodMultiple), NULL },
    { 1, CBOR_T_UINT, 4, 0, offsetof(cbor_keyset_can_message_t, transmitAddress), NULL },
};
static const cbor_schema_t s_schema_config_can_message_ins_down = { s_fields_config_can_message_ins_down, sizeof(s_fields_config_can_message_ins_down) / sizeof(s_fields_config_can_message_ins_down[0]) };

static const cbor_field_t s_fields_config_can_message_ins_ecef_x[] = {
    { 0, CBOR_T_UINT, 2, 0, offsetof(cbor_keyset_can_message_t, periodMultiple), NULL },
    { 1, CBOR_T_UINT, 4, 0, offsetof(cbor_keyset_can_message_t, transmitAddress), NULL },
};
static const cbor_schema_t s_schema_config_can_message_ins_ecef_x = { s_fields_config_can_message_ins_ecef_x, sizeof(s_fields_config_can_message_ins_ecef_x) / sizeof(s_fields_config_can_message_ins_ecef_x[0]) };

static const cbor_field_t s_fields_config_can_message_ins_ecef_y[] = {
    { 0, CBOR_T_UINT, 2, 0, offsetof(cbor_keyset_can_message_t, periodMultiple), NULL },
    { 1, CBOR_T_UINT, 4, 0, offsetof(cbor_keyset_can_message_t, transmitAddress), NULL },
};
static const cbor_schema_t s_schema_config_can_message_ins_ecef_y = { s_fields_config_can_message_ins_ecef_y, sizeof(s_fields_config_can_message_ins_ecef_y) / sizeof(s_fields_config_can_message_ins_ecef_y[0]) };

static const cbor_field_t s_fields_config_can_message_ins_ecef_z[] = {
    { 0, CBOR_T_UINT, 2, 0, offsetof(cbor_keyset_can_message_t, periodMultiple), NULL },
    { 1, CBOR_T_UINT, 4, 0, offsetof(cbor_keyset_can_message_t, transmitAddress), NULL },
};
static const cbor_schema_t s_schema_config_can_message_ins_ecef_z = { s_fields_config_can_message_ins_ecef_z, sizeof(s_fields_config_can_message_ins_ecef_z) / sizeof(s_fields_config_can_message_ins_ecef_z[0]) };

static const cbor_field_t s_fields_config_can_message_ins_msl[] = {
    { 0, CBOR_T_UINT, 2, 0, offsetof(cbor_keyset_can_message_t, periodMultiple), NULL },
    { 1, CBOR_T_UINT, 4, 0, offsetof(cbor_keyset_can_message_t, transmitAddress), NULL },
};
static const cbor_schema_t s_schema_config_can_message_ins_msl = { s_fields_config_can_message_ins_msl, sizeof(s_fields_config_can_message_ins_msl) / sizeof(s_fields_config_can_message_ins_msl[0]) };

static const cbor_field_t s_fields_config_can_message_preint_px[] = {
    { 0, CBOR_T_UINT, 2, 0, offsetof(cbor_keyset_can_message_t, periodMultiple), NULL },
    { 1, CBOR_T_UINT, 4, 0, offsetof(cbor_keyset_can_message_t, transmitAddress), NULL },
};
static const cbor_schema_t s_schema_config_can_message_preint_px = { s_fields_config_can_message_preint_px, sizeof(s_fields_config_can_message_preint_px) / sizeof(s_fields_config_can_message_preint_px[0]) };

static const cbor_field_t s_fields_config_can_message_preint_qy[] = {
    { 0, CBOR_T_UINT, 2, 0, offsetof(cbor_keyset_can_message_t, periodMultiple), NULL },
    { 1, CBOR_T_UINT, 4, 0, offsetof(cbor_keyset_can_message_t, transmitAddress), NULL },
};
static const cbor_schema_t s_schema_config_can_message_preint_qy = { s_fields_config_can_message_preint_qy, sizeof(s_fields_config_can_message_preint_qy) / sizeof(s_fields_config_can_message_preint_qy[0]) };

static const cbor_field_t s_fields_config_can_message_preint_rz[] = {
    { 0, CBOR_T_UINT, 2, 0, offsetof(cbor_keyset_can_message_t, periodMultiple), NULL },
    { 1, CBOR_T_UINT, 4, 0, offsetof(cbor_keyset_can_message_t, transmitAddress), NULL },
};
static const cbor_schema_t s_schema_config_can_message_preint_rz = { s_fields_config_can_message_preint_rz, sizeof(s_fields_config_can_message_preint_rz) / sizeof(s_fields_config_can_message_preint_rz[0]) };

static const cbor_field_t s_fields_config_can_message_dual_px[] = {
    { 0, CBOR_T_UINT, 2, 0, offsetof(cbor_keyset_can_message_t, periodMultiple), NULL },
    { 1, CBOR_T_UINT, 4, 0, offsetof(cbor_keyset_can_message_t, transmitAddress), NULL },
};
static const cbor_schema_t s_schema_config_can_message_dual_px = { s_fields_config_can_message_dual_px, sizeof(s_fields_config_can_message_dual_px) / sizeof(s_fields_config_can_message_dual_px[0]) };

static const cbor_field_t s_fields_config_can_message_dual_qy[] = {
    { 0, CBOR_T_UINT, 2, 0, offsetof(cbor_keyset_can_message_t, periodMultiple), NULL },
    { 1, CBOR_T_UINT, 4, 0, offsetof(cbor_keyset_can_message_t, transmitAddress), NULL },
};
static const cbor_schema_t s_schema_config_can_message_dual_qy = { s_fields_config_can_message_dual_qy, sizeof(s_fields_config_can_message_dual_qy) / sizeof(s_fields_config_can_message_dual_qy[0]) };

static const cbor_field_t s_fields_config_can_message_dual_rz[] = {
    { 0, CBOR_T_UINT, 2, 0, offsetof(cbor_keyset_can_message_t, periodMultiple), NULL },
    { 1, CBOR_T_UINT, 4, 0, offsetof(cbor_keyset_can_message_t, transmitAddress), NULL },
};
static const cbor_schema_t s_schema_config_can_message_dual_rz = { s_fields_config_can_message_dual_rz, sizeof(s_fields_config_can_message_dual_rz) / sizeof(s_fields_config_can_message_dual_rz[0]) };

static const cbor_field_t s_fields_config_can_message_gnss1_pos[] = {
    { 0, CBOR_T_UINT, 2, 0, offsetof(cbor_keyset_can_message_t, periodMultiple), NULL },
    { 1, CBOR_T_UINT, 4, 0, offsetof(cbor_keyset_can_message_t, transmitAddress), NULL },
};
static const cbor_schema_t s_schema_config_can_message_gnss1_pos = { s_fields_config_can_message_gnss1_pos, sizeof(s_fields_config_can_message_gnss1_pos) / sizeof(s_fields_config_can_message_gnss1_pos[0]) };

static const cbor_field_t s_fields_config_can_message_gnss2_pos[] = {
    { 0, CBOR_T_UINT, 2, 0, offsetof(cbor_keyset_can_message_t, periodMultiple), NULL },
    { 1, CBOR_T_UINT, 4, 0, offsetof(cbor_keyset_can_message_t, transmitAddress), NULL },
};
static const cbor_schema_t s_schema_config_can_message_gnss2_pos = { s_fields_config_can_message_gnss2_pos, sizeof(s_fields_config_can_message_gnss2_pos) / sizeof(s_fields_config_can_message_gnss2_pos[0]) };

static const cbor_field_t s_fields_config_can_message_gnss1_rtk_pos_rel[] = {
    { 0, CBOR_T_UINT, 2, 0, offsetof(cbor_keyset_can_message_t, periodMultiple), NULL },
    { 1, CBOR_T_UINT, 4, 0, offsetof(cbor_keyset_can_message_t, transmitAddress), NULL },
};
static const cbor_schema_t s_schema_config_can_message_gnss1_rtk_pos_rel = { s_fields_config_can_message_gnss1_rtk_pos_rel, sizeof(s_fields_config_can_message_gnss1_rtk_pos_rel) / sizeof(s_fields_config_can_message_gnss1_rtk_pos_rel[0]) };

static const cbor_field_t s_fields_config_can_message_gnss2_rtk_cmp_rel[] = {
    { 0, CBOR_T_UINT, 2, 0, offsetof(cbor_keyset_can_message_t, periodMultiple), NULL },
    { 1, CBOR_T_UINT, 4, 0, offsetof(cbor_keyset_can_message_t, transmitAddress), NULL },
};
static const cbor_schema_t s_schema_config_can_message_gnss2_rtk_cmp_rel = { s_fields_config_can_message_gnss2_rtk_cmp_rel, sizeof(s_fields_config_can_message_gnss2_rtk_cmp_rel) / sizeof(s_fields_config_can_message_gnss2_rtk_cmp_rel[0]) };

static const cbor_field_t s_fields_config_can_message_roll_rollrate[] = {
    { 0, CBOR_T_UINT, 2, 0, offsetof(cbor_keyset_can_message_t, periodMultiple), NULL },
    { 1, CBOR_T_UINT, 4, 0, offsetof(cbor_keyset_can_message_t, transmitAddress), NULL },
};
static const cbor_schema_t s_schema_config_can_message_roll_rollrate = { s_fields_config_can_message_roll_rollrate, sizeof(s_fields_config_can_message_roll_rollrate) / sizeof(s_fields_config_can_message_roll_rollrate[0]) };

static const cbor_field_t s_fields_config_can_message[] = {
    { 0, CBOR_T_MAP, 0, 0, CBOR_CAN_MESSAGE_INS_TIME * sizeof(cbor_keyset_can_message_t), &s_schema_config_can_message_ins_time },
    { 1, CBOR_T_MAP, 0, 0, CBOR_CAN_MESSAGE_INS_STATUS * sizeof(cbor_keyset_can_message_t), &s_schema_config_can_message_ins_status },
    { 2, CBOR_T_MAP, 0, 0, CBOR_CAN_MESSAGE_INS_EULER * sizeof(cbor_keyset_can_message_t), &s_schema_config_can_message_ins_euler },
    { 3, CBOR_T_MAP, 0, 0, CBOR_CAN_MESSAGE_INS_QUATN2B * sizeof(cbor_keyset_can_message_t), &s_schema_config_can_message_ins_quatn2b },
    { 4, CBOR_T_MAP, 0, 0, CBOR_CAN_MESSAGE_INS_QUATE2B * sizeof(cbor_keyset_can_message_t), &s_schema_config_can_message_ins_quate2b },
    { 5, CBOR_T_MAP, 0, 0, CBOR_CAN_MESSAGE_INS_UVW * sizeof(cbor_keyset_can_message_t), &s_schema_config_can_message_ins_uvw },
    { 6, CBOR_T_MAP, 0, 0, CBOR_CAN_MESSAGE_INS_VE * sizeof(cbor_keyset_can_message_t), &s_schema_config_can_message_ins_ve },
    { 7, CBOR_T_MAP, 0, 0, CBOR_CAN_MESSAGE_INS_LAT * sizeof(cbor_keyset_can_message_t), &s_schema_config_can_message_ins_lat },
    { 8, CBOR_T_MAP, 0, 0, CBOR_CAN_MESSAGE_INS_LON * sizeof(cbor_keyset_can_message_t), &s_schema_config_can_message_ins_lon },
    { 9, CBOR_T_MAP, 0, 0, CBOR_CAN_MESSAGE_INS_ALT * sizeof(cbor_keyset_can_message_t), &s_schema_config_can_message_ins_alt },
    { 10, CBOR_T_MAP, 0, 0, CBOR_CAN_MESSAGE_INS_NORTH_EAST * sizeof(cbor_keyset_can_message_t), &s_schema_config_can_message_ins_north_east },
    { 11, CBOR_T_MAP, 0, 0, CBOR_CAN_MESSAGE_INS_DOWN * sizeof(cbor_keyset_can_message_t), &s_schema_config_can_message_ins_down },
    { 12, CBOR_T_MAP, 0, 0, CBOR_CAN_MESSAGE_INS_ECEF_X * sizeof(cbor_keyset_can_message_t), &s_schema_config_can_message_ins_ecef_x },
    { 13, CBOR_T_MAP, 0, 0, CBOR_CAN_MESSAGE_INS_ECEF_Y * sizeof(cbor_keyset_can_message_t), &s_schema_config_can_message_ins_ecef_y },
    { 14, CBOR_T_MAP, 0, 0, CBOR_CAN_MESSAGE_INS_ECEF_Z * sizeof(cbor_keyset_can_message_t), &s_schema_config_can_message_ins_ecef_z },
    { 15, CBOR_T_MAP, 0, 0, CBOR_CAN_MESSAGE_INS_MSL * sizeof(cbor_keyset_can_message_t), &s_schema_config_can_message_ins_msl },
    { 16, CBOR_T_MAP, 0, 0, CBOR_CAN_MESSAGE_PREINT_PX * sizeof(cbor_keyset_can_message_t), &s_schema_config_can_message_preint_px },
    { 17, CBOR_T_MAP, 0, 0, CBOR_CAN_MESSAGE_PREINT_QY * sizeof(cbor_keyset_can_message_t), &s_schema_config_can_message_preint_qy },
    { 18, CBOR_T_MAP, 0, 0, CBOR_CAN_MESSAGE_PREINT_RZ * sizeof(cbor_keyset_can_message_t), &s_schema_config_can_message_preint_rz },
    { 19, CBOR_T_MAP, 0, 0, CBOR_CAN_MESSAGE_DUAL_PX * sizeof(cbor_keyset_can_message_t), &s_schema_config_can_message_dual_px },
    { 20, CBOR_T_MAP, 0, 0, CBOR_CAN_MESSAGE_DUAL_QY * sizeof(cbor_keyset_can_message_t), &s_schema_config_can_message_dual_qy },
    { 21, CBOR_T_MAP, 0, 0, CBOR_CAN_MESSAGE_DUAL_RZ * sizeof(cbor_keyset_can_message_t), &s_schema_config_can_message_dual_rz },
    { 22, CBOR_T_MAP, 0, 0, CBOR_CAN_MESSAGE_GNSS1_POS * sizeof(cbor_keyset_can_message_t), &s_schema_config_can_message_gnss1_pos },
    { 23, CBOR_T_MAP, 0, 0, CBOR_CAN_MESSAGE_GNSS2_POS * sizeof(cbor_keyset_can_message_t), &s_schema_config_can_message_gnss2_pos },
    { 24, CBOR_T_MAP, 0, 0, CBOR_CAN_MESSAGE_GNSS1_RTK_POS_REL * sizeof(cbor_keyset_can_message_t), &s_schema_config_can_message_gnss1_rtk_pos_rel },
    { 25, CBOR_T_MAP, 0, 0, CBOR_CAN_MESSAGE_GNSS2_RTK_CMP_REL * sizeof(cbor_keyset_can_message_t), &s_schema_config_can_message_gnss2_rtk_cmp_rel },
    { 26, CBOR_T_MAP, 0, 0, CBOR_CAN_MESSAGE_ROLL_ROLLRATE * sizeof(cbor_keyset_can_message_t), &s_schema_config_can_message_roll_rollrate },
};
static const cbor_schema_t s_schema_config_can_message = { s_fields_config_can_message, sizeof(s_fields_config_can_message) / sizeof(s_fields_config_can_message[0]) };

static const cbor_field_t s_fields_config_can[] = {
    { 0, CBOR_T_UINT, 2, 0, offsetof(cbor_config_can_t, bitRateKbps), NULL },
    { 1, CBOR_T_BOOL, 1, 0, offsetof(cbor_config_can_t, fdEnabled), NULL },
    { 2, CBOR_T_UINT, 4, 0, offsetof(cbor_config_can_t, receiveAddress), NULL },
    { 3, CBOR_T_MAP, 0, 0, offsetof(cbor_config_can_t, Message), &s_schema_config_can_message },
};
const cbor_schema_t g_cborSchema_CONFIG_CAN = { s_fields_config_can, sizeof(s_fields_config_can) / sizeof(s_fields_config_can[0]) };

static const cbor_field_t s_fields_config_can_imx_message_ins_time[] = {
    { 0, CBOR_T_UINT, 2, 0, offsetof(cbor_keyset_can_message_t, periodMultiple), NULL },
    { 1, CBOR_T_UINT, 4, 0, offsetof(cbor_keyset_can_message_t, transmitAddress), NULL },
};
static const cbor_schema_t s_schema_config_can_imx_message_ins_time = { s_fields_config_can_imx_message_ins_time, sizeof(s_fields_config_can_imx_message_ins_time) / sizeof(s_fields_config_can_imx_message_ins_time[0]) };

static const cbor_field_t s_fields_config_can_imx_message_ins_status[] = {
    { 0, CBOR_T_UINT, 2, 0, offsetof(cbor_keyset_can_message_t, periodMultiple), NULL },
    { 1, CBOR_T_UINT, 4, 0, offsetof(cbor_keyset_can_message_t, transmitAddress), NULL },
};
static const cbor_schema_t s_schema_config_can_imx_message_ins_status = { s_fields_config_can_imx_message_ins_status, sizeof(s_fields_config_can_imx_message_ins_status) / sizeof(s_fields_config_can_imx_message_ins_status[0]) };

static const cbor_field_t s_fields_config_can_imx_message_ins_euler[] = {
    { 0, CBOR_T_UINT, 2, 0, offsetof(cbor_keyset_can_message_t, periodMultiple), NULL },
    { 1, CBOR_T_UINT, 4, 0, offsetof(cbor_keyset_can_message_t, transmitAddress), NULL },
};
static const cbor_schema_t s_schema_config_can_imx_message_ins_euler = { s_fields_config_can_imx_message_ins_euler, sizeof(s_fields_config_can_imx_message_ins_euler) / sizeof(s_fields_config_can_imx_message_ins_euler[0]) };

static const cbor_field_t s_fields_config_can_imx_message_ins_quatn2b[] = {
    { 0, CBOR_T_UINT, 2, 0, offsetof(cbor_keyset_can_message_t, periodMultiple), NULL },
    { 1, CBOR_T_UINT, 4, 0, offsetof(cbor_keyset_can_message_t, transmitAddress), NULL },
};
static const cbor_schema_t s_schema_config_can_imx_message_ins_quatn2b = { s_fields_config_can_imx_message_ins_quatn2b, sizeof(s_fields_config_can_imx_message_ins_quatn2b) / sizeof(s_fields_config_can_imx_message_ins_quatn2b[0]) };

static const cbor_field_t s_fields_config_can_imx_message_ins_quate2b[] = {
    { 0, CBOR_T_UINT, 2, 0, offsetof(cbor_keyset_can_message_t, periodMultiple), NULL },
    { 1, CBOR_T_UINT, 4, 0, offsetof(cbor_keyset_can_message_t, transmitAddress), NULL },
};
static const cbor_schema_t s_schema_config_can_imx_message_ins_quate2b = { s_fields_config_can_imx_message_ins_quate2b, sizeof(s_fields_config_can_imx_message_ins_quate2b) / sizeof(s_fields_config_can_imx_message_ins_quate2b[0]) };

static const cbor_field_t s_fields_config_can_imx_message_ins_uvw[] = {
    { 0, CBOR_T_UINT, 2, 0, offsetof(cbor_keyset_can_message_t, periodMultiple), NULL },
    { 1, CBOR_T_UINT, 4, 0, offsetof(cbor_keyset_can_message_t, transmitAddress), NULL },
};
static const cbor_schema_t s_schema_config_can_imx_message_ins_uvw = { s_fields_config_can_imx_message_ins_uvw, sizeof(s_fields_config_can_imx_message_ins_uvw) / sizeof(s_fields_config_can_imx_message_ins_uvw[0]) };

static const cbor_field_t s_fields_config_can_imx_message_ins_ve[] = {
    { 0, CBOR_T_UINT, 2, 0, offsetof(cbor_keyset_can_message_t, periodMultiple), NULL },
    { 1, CBOR_T_UINT, 4, 0, offsetof(cbor_keyset_can_message_t, transmitAddress), NULL },
};
static const cbor_schema_t s_schema_config_can_imx_message_ins_ve = { s_fields_config_can_imx_message_ins_ve, sizeof(s_fields_config_can_imx_message_ins_ve) / sizeof(s_fields_config_can_imx_message_ins_ve[0]) };

static const cbor_field_t s_fields_config_can_imx_message_ins_lat[] = {
    { 0, CBOR_T_UINT, 2, 0, offsetof(cbor_keyset_can_message_t, periodMultiple), NULL },
    { 1, CBOR_T_UINT, 4, 0, offsetof(cbor_keyset_can_message_t, transmitAddress), NULL },
};
static const cbor_schema_t s_schema_config_can_imx_message_ins_lat = { s_fields_config_can_imx_message_ins_lat, sizeof(s_fields_config_can_imx_message_ins_lat) / sizeof(s_fields_config_can_imx_message_ins_lat[0]) };

static const cbor_field_t s_fields_config_can_imx_message_ins_lon[] = {
    { 0, CBOR_T_UINT, 2, 0, offsetof(cbor_keyset_can_message_t, periodMultiple), NULL },
    { 1, CBOR_T_UINT, 4, 0, offsetof(cbor_keyset_can_message_t, transmitAddress), NULL },
};
static const cbor_schema_t s_schema_config_can_imx_message_ins_lon = { s_fields_config_can_imx_message_ins_lon, sizeof(s_fields_config_can_imx_message_ins_lon) / sizeof(s_fields_config_can_imx_message_ins_lon[0]) };

static const cbor_field_t s_fields_config_can_imx_message_ins_alt[] = {
    { 0, CBOR_T_UINT, 2, 0, offsetof(cbor_keyset_can_message_t, periodMultiple), NULL },
    { 1, CBOR_T_UINT, 4, 0, offsetof(cbor_keyset_can_message_t, transmitAddress), NULL },
};
static const cbor_schema_t s_schema_config_can_imx_message_ins_alt = { s_fields_config_can_imx_message_ins_alt, sizeof(s_fields_config_can_imx_message_ins_alt) / sizeof(s_fields_config_can_imx_message_ins_alt[0]) };

static const cbor_field_t s_fields_config_can_imx_message_ins_north_east[] = {
    { 0, CBOR_T_UINT, 2, 0, offsetof(cbor_keyset_can_message_t, periodMultiple), NULL },
    { 1, CBOR_T_UINT, 4, 0, offsetof(cbor_keyset_can_message_t, transmitAddress), NULL },
};
static const cbor_schema_t s_schema_config_can_imx_message_ins_north_east = { s_fields_config_can_imx_message_ins_north_east, sizeof(s_fields_config_can_imx_message_ins_north_east) / sizeof(s_fields_config_can_imx_message_ins_north_east[0]) };

static const cbor_field_t s_fields_config_can_imx_message_ins_down[] = {
    { 0, CBOR_T_UINT, 2, 0, offsetof(cbor_keyset_can_message_t, periodMultiple), NULL },
    { 1, CBOR_T_UINT, 4, 0, offsetof(cbor_keyset_can_message_t, transmitAddress), NULL },
};
static const cbor_schema_t s_schema_config_can_imx_message_ins_down = { s_fields_config_can_imx_message_ins_down, sizeof(s_fields_config_can_imx_message_ins_down) / sizeof(s_fields_config_can_imx_message_ins_down[0]) };

static const cbor_field_t s_fields_config_can_imx_message_ins_ecef_x[] = {
    { 0, CBOR_T_UINT, 2, 0, offsetof(cbor_keyset_can_message_t, periodMultiple), NULL },
    { 1, CBOR_T_UINT, 4, 0, offsetof(cbor_keyset_can_message_t, transmitAddress), NULL },
};
static const cbor_schema_t s_schema_config_can_imx_message_ins_ecef_x = { s_fields_config_can_imx_message_ins_ecef_x, sizeof(s_fields_config_can_imx_message_ins_ecef_x) / sizeof(s_fields_config_can_imx_message_ins_ecef_x[0]) };

static const cbor_field_t s_fields_config_can_imx_message_ins_ecef_y[] = {
    { 0, CBOR_T_UINT, 2, 0, offsetof(cbor_keyset_can_message_t, periodMultiple), NULL },
    { 1, CBOR_T_UINT, 4, 0, offsetof(cbor_keyset_can_message_t, transmitAddress), NULL },
};
static const cbor_schema_t s_schema_config_can_imx_message_ins_ecef_y = { s_fields_config_can_imx_message_ins_ecef_y, sizeof(s_fields_config_can_imx_message_ins_ecef_y) / sizeof(s_fields_config_can_imx_message_ins_ecef_y[0]) };

static const cbor_field_t s_fields_config_can_imx_message_ins_ecef_z[] = {
    { 0, CBOR_T_UINT, 2, 0, offsetof(cbor_keyset_can_message_t, periodMultiple), NULL },
    { 1, CBOR_T_UINT, 4, 0, offsetof(cbor_keyset_can_message_t, transmitAddress), NULL },
};
static const cbor_schema_t s_schema_config_can_imx_message_ins_ecef_z = { s_fields_config_can_imx_message_ins_ecef_z, sizeof(s_fields_config_can_imx_message_ins_ecef_z) / sizeof(s_fields_config_can_imx_message_ins_ecef_z[0]) };

static const cbor_field_t s_fields_config_can_imx_message_ins_msl[] = {
    { 0, CBOR_T_UINT, 2, 0, offsetof(cbor_keyset_can_message_t, periodMultiple), NULL },
    { 1, CBOR_T_UINT, 4, 0, offsetof(cbor_keyset_can_message_t, transmitAddress), NULL },
};
static const cbor_schema_t s_schema_config_can_imx_message_ins_msl = { s_fields_config_can_imx_message_ins_msl, sizeof(s_fields_config_can_imx_message_ins_msl) / sizeof(s_fields_config_can_imx_message_ins_msl[0]) };

static const cbor_field_t s_fields_config_can_imx_message_preint_px[] = {
    { 0, CBOR_T_UINT, 2, 0, offsetof(cbor_keyset_can_message_t, periodMultiple), NULL },
    { 1, CBOR_T_UINT, 4, 0, offsetof(cbor_keyset_can_message_t, transmitAddress), NULL },
};
static const cbor_schema_t s_schema_config_can_imx_message_preint_px = { s_fields_config_can_imx_message_preint_px, sizeof(s_fields_config_can_imx_message_preint_px) / sizeof(s_fields_config_can_imx_message_preint_px[0]) };

static const cbor_field_t s_fields_config_can_imx_message_preint_qy[] = {
    { 0, CBOR_T_UINT, 2, 0, offsetof(cbor_keyset_can_message_t, periodMultiple), NULL },
    { 1, CBOR_T_UINT, 4, 0, offsetof(cbor_keyset_can_message_t, transmitAddress), NULL },
};
static const cbor_schema_t s_schema_config_can_imx_message_preint_qy = { s_fields_config_can_imx_message_preint_qy, sizeof(s_fields_config_can_imx_message_preint_qy) / sizeof(s_fields_config_can_imx_message_preint_qy[0]) };

static const cbor_field_t s_fields_config_can_imx_message_preint_rz[] = {
    { 0, CBOR_T_UINT, 2, 0, offsetof(cbor_keyset_can_message_t, periodMultiple), NULL },
    { 1, CBOR_T_UINT, 4, 0, offsetof(cbor_keyset_can_message_t, transmitAddress), NULL },
};
static const cbor_schema_t s_schema_config_can_imx_message_preint_rz = { s_fields_config_can_imx_message_preint_rz, sizeof(s_fields_config_can_imx_message_preint_rz) / sizeof(s_fields_config_can_imx_message_preint_rz[0]) };

static const cbor_field_t s_fields_config_can_imx_message_dual_px[] = {
    { 0, CBOR_T_UINT, 2, 0, offsetof(cbor_keyset_can_message_t, periodMultiple), NULL },
    { 1, CBOR_T_UINT, 4, 0, offsetof(cbor_keyset_can_message_t, transmitAddress), NULL },
};
static const cbor_schema_t s_schema_config_can_imx_message_dual_px = { s_fields_config_can_imx_message_dual_px, sizeof(s_fields_config_can_imx_message_dual_px) / sizeof(s_fields_config_can_imx_message_dual_px[0]) };

static const cbor_field_t s_fields_config_can_imx_message_dual_qy[] = {
    { 0, CBOR_T_UINT, 2, 0, offsetof(cbor_keyset_can_message_t, periodMultiple), NULL },
    { 1, CBOR_T_UINT, 4, 0, offsetof(cbor_keyset_can_message_t, transmitAddress), NULL },
};
static const cbor_schema_t s_schema_config_can_imx_message_dual_qy = { s_fields_config_can_imx_message_dual_qy, sizeof(s_fields_config_can_imx_message_dual_qy) / sizeof(s_fields_config_can_imx_message_dual_qy[0]) };

static const cbor_field_t s_fields_config_can_imx_message_dual_rz[] = {
    { 0, CBOR_T_UINT, 2, 0, offsetof(cbor_keyset_can_message_t, periodMultiple), NULL },
    { 1, CBOR_T_UINT, 4, 0, offsetof(cbor_keyset_can_message_t, transmitAddress), NULL },
};
static const cbor_schema_t s_schema_config_can_imx_message_dual_rz = { s_fields_config_can_imx_message_dual_rz, sizeof(s_fields_config_can_imx_message_dual_rz) / sizeof(s_fields_config_can_imx_message_dual_rz[0]) };

static const cbor_field_t s_fields_config_can_imx_message_gnss1_pos[] = {
    { 0, CBOR_T_UINT, 2, 0, offsetof(cbor_keyset_can_message_t, periodMultiple), NULL },
    { 1, CBOR_T_UINT, 4, 0, offsetof(cbor_keyset_can_message_t, transmitAddress), NULL },
};
static const cbor_schema_t s_schema_config_can_imx_message_gnss1_pos = { s_fields_config_can_imx_message_gnss1_pos, sizeof(s_fields_config_can_imx_message_gnss1_pos) / sizeof(s_fields_config_can_imx_message_gnss1_pos[0]) };

static const cbor_field_t s_fields_config_can_imx_message_gnss2_pos[] = {
    { 0, CBOR_T_UINT, 2, 0, offsetof(cbor_keyset_can_message_t, periodMultiple), NULL },
    { 1, CBOR_T_UINT, 4, 0, offsetof(cbor_keyset_can_message_t, transmitAddress), NULL },
};
static const cbor_schema_t s_schema_config_can_imx_message_gnss2_pos = { s_fields_config_can_imx_message_gnss2_pos, sizeof(s_fields_config_can_imx_message_gnss2_pos) / sizeof(s_fields_config_can_imx_message_gnss2_pos[0]) };

static const cbor_field_t s_fields_config_can_imx_message_gnss1_rtk_pos_rel[] = {
    { 0, CBOR_T_UINT, 2, 0, offsetof(cbor_keyset_can_message_t, periodMultiple), NULL },
    { 1, CBOR_T_UINT, 4, 0, offsetof(cbor_keyset_can_message_t, transmitAddress), NULL },
};
static const cbor_schema_t s_schema_config_can_imx_message_gnss1_rtk_pos_rel = { s_fields_config_can_imx_message_gnss1_rtk_pos_rel, sizeof(s_fields_config_can_imx_message_gnss1_rtk_pos_rel) / sizeof(s_fields_config_can_imx_message_gnss1_rtk_pos_rel[0]) };

static const cbor_field_t s_fields_config_can_imx_message_gnss2_rtk_cmp_rel[] = {
    { 0, CBOR_T_UINT, 2, 0, offsetof(cbor_keyset_can_message_t, periodMultiple), NULL },
    { 1, CBOR_T_UINT, 4, 0, offsetof(cbor_keyset_can_message_t, transmitAddress), NULL },
};
static const cbor_schema_t s_schema_config_can_imx_message_gnss2_rtk_cmp_rel = { s_fields_config_can_imx_message_gnss2_rtk_cmp_rel, sizeof(s_fields_config_can_imx_message_gnss2_rtk_cmp_rel) / sizeof(s_fields_config_can_imx_message_gnss2_rtk_cmp_rel[0]) };

static const cbor_field_t s_fields_config_can_imx_message_roll_rollrate[] = {
    { 0, CBOR_T_UINT, 2, 0, offsetof(cbor_keyset_can_message_t, periodMultiple), NULL },
    { 1, CBOR_T_UINT, 4, 0, offsetof(cbor_keyset_can_message_t, transmitAddress), NULL },
};
static const cbor_schema_t s_schema_config_can_imx_message_roll_rollrate = { s_fields_config_can_imx_message_roll_rollrate, sizeof(s_fields_config_can_imx_message_roll_rollrate) / sizeof(s_fields_config_can_imx_message_roll_rollrate[0]) };

static const cbor_field_t s_fields_config_can_imx_message[] = {
    { 0, CBOR_T_MAP, 0, 0, CBOR_CAN_MESSAGE_INS_TIME * sizeof(cbor_keyset_can_message_t), &s_schema_config_can_imx_message_ins_time },
    { 1, CBOR_T_MAP, 0, 0, CBOR_CAN_MESSAGE_INS_STATUS * sizeof(cbor_keyset_can_message_t), &s_schema_config_can_imx_message_ins_status },
    { 2, CBOR_T_MAP, 0, 0, CBOR_CAN_MESSAGE_INS_EULER * sizeof(cbor_keyset_can_message_t), &s_schema_config_can_imx_message_ins_euler },
    { 3, CBOR_T_MAP, 0, 0, CBOR_CAN_MESSAGE_INS_QUATN2B * sizeof(cbor_keyset_can_message_t), &s_schema_config_can_imx_message_ins_quatn2b },
    { 4, CBOR_T_MAP, 0, 0, CBOR_CAN_MESSAGE_INS_QUATE2B * sizeof(cbor_keyset_can_message_t), &s_schema_config_can_imx_message_ins_quate2b },
    { 5, CBOR_T_MAP, 0, 0, CBOR_CAN_MESSAGE_INS_UVW * sizeof(cbor_keyset_can_message_t), &s_schema_config_can_imx_message_ins_uvw },
    { 6, CBOR_T_MAP, 0, 0, CBOR_CAN_MESSAGE_INS_VE * sizeof(cbor_keyset_can_message_t), &s_schema_config_can_imx_message_ins_ve },
    { 7, CBOR_T_MAP, 0, 0, CBOR_CAN_MESSAGE_INS_LAT * sizeof(cbor_keyset_can_message_t), &s_schema_config_can_imx_message_ins_lat },
    { 8, CBOR_T_MAP, 0, 0, CBOR_CAN_MESSAGE_INS_LON * sizeof(cbor_keyset_can_message_t), &s_schema_config_can_imx_message_ins_lon },
    { 9, CBOR_T_MAP, 0, 0, CBOR_CAN_MESSAGE_INS_ALT * sizeof(cbor_keyset_can_message_t), &s_schema_config_can_imx_message_ins_alt },
    { 10, CBOR_T_MAP, 0, 0, CBOR_CAN_MESSAGE_INS_NORTH_EAST * sizeof(cbor_keyset_can_message_t), &s_schema_config_can_imx_message_ins_north_east },
    { 11, CBOR_T_MAP, 0, 0, CBOR_CAN_MESSAGE_INS_DOWN * sizeof(cbor_keyset_can_message_t), &s_schema_config_can_imx_message_ins_down },
    { 12, CBOR_T_MAP, 0, 0, CBOR_CAN_MESSAGE_INS_ECEF_X * sizeof(cbor_keyset_can_message_t), &s_schema_config_can_imx_message_ins_ecef_x },
    { 13, CBOR_T_MAP, 0, 0, CBOR_CAN_MESSAGE_INS_ECEF_Y * sizeof(cbor_keyset_can_message_t), &s_schema_config_can_imx_message_ins_ecef_y },
    { 14, CBOR_T_MAP, 0, 0, CBOR_CAN_MESSAGE_INS_ECEF_Z * sizeof(cbor_keyset_can_message_t), &s_schema_config_can_imx_message_ins_ecef_z },
    { 15, CBOR_T_MAP, 0, 0, CBOR_CAN_MESSAGE_INS_MSL * sizeof(cbor_keyset_can_message_t), &s_schema_config_can_imx_message_ins_msl },
    { 16, CBOR_T_MAP, 0, 0, CBOR_CAN_MESSAGE_PREINT_PX * sizeof(cbor_keyset_can_message_t), &s_schema_config_can_imx_message_preint_px },
    { 17, CBOR_T_MAP, 0, 0, CBOR_CAN_MESSAGE_PREINT_QY * sizeof(cbor_keyset_can_message_t), &s_schema_config_can_imx_message_preint_qy },
    { 18, CBOR_T_MAP, 0, 0, CBOR_CAN_MESSAGE_PREINT_RZ * sizeof(cbor_keyset_can_message_t), &s_schema_config_can_imx_message_preint_rz },
    { 19, CBOR_T_MAP, 0, 0, CBOR_CAN_MESSAGE_DUAL_PX * sizeof(cbor_keyset_can_message_t), &s_schema_config_can_imx_message_dual_px },
    { 20, CBOR_T_MAP, 0, 0, CBOR_CAN_MESSAGE_DUAL_QY * sizeof(cbor_keyset_can_message_t), &s_schema_config_can_imx_message_dual_qy },
    { 21, CBOR_T_MAP, 0, 0, CBOR_CAN_MESSAGE_DUAL_RZ * sizeof(cbor_keyset_can_message_t), &s_schema_config_can_imx_message_dual_rz },
    { 22, CBOR_T_MAP, 0, 0, CBOR_CAN_MESSAGE_GNSS1_POS * sizeof(cbor_keyset_can_message_t), &s_schema_config_can_imx_message_gnss1_pos },
    { 23, CBOR_T_MAP, 0, 0, CBOR_CAN_MESSAGE_GNSS2_POS * sizeof(cbor_keyset_can_message_t), &s_schema_config_can_imx_message_gnss2_pos },
    { 24, CBOR_T_MAP, 0, 0, CBOR_CAN_MESSAGE_GNSS1_RTK_POS_REL * sizeof(cbor_keyset_can_message_t), &s_schema_config_can_imx_message_gnss1_rtk_pos_rel },
    { 25, CBOR_T_MAP, 0, 0, CBOR_CAN_MESSAGE_GNSS2_RTK_CMP_REL * sizeof(cbor_keyset_can_message_t), &s_schema_config_can_imx_message_gnss2_rtk_cmp_rel },
    { 26, CBOR_T_MAP, 0, 0, CBOR_CAN_MESSAGE_ROLL_ROLLRATE * sizeof(cbor_keyset_can_message_t), &s_schema_config_can_imx_message_roll_rollrate },
};
static const cbor_schema_t s_schema_config_can_imx_message = { s_fields_config_can_imx_message, sizeof(s_fields_config_can_imx_message) / sizeof(s_fields_config_can_imx_message[0]) };

static const cbor_field_t s_fields_config_can_imx[] = {
    { 0, CBOR_T_UINT, 2, 0, offsetof(cbor_config_can_t, bitRateKbps), NULL },
    { 1, CBOR_T_BOOL, 1, 0, offsetof(cbor_config_can_t, fdEnabled), NULL },
    { 2, CBOR_T_UINT, 4, 0, offsetof(cbor_config_can_t, receiveAddress), NULL },
    { 3, CBOR_T_MAP, 0, 0, offsetof(cbor_config_can_t, Message), &s_schema_config_can_imx_message },
};
const cbor_schema_t g_cborSchema_CONFIG_CAN_imx = { s_fields_config_can_imx, sizeof(s_fields_config_can_imx) / sizeof(s_fields_config_can_imx[0]) };

const cbor_config_can_t g_cborDefaults_CONFIG_CAN = {
    .bitRateKbps = 100,
    .fdEnabled = 0,
    .receiveAddress = 0,
    .Message[CBOR_CAN_MESSAGE_INS_TIME].periodMultiple = 0,
    .Message[CBOR_CAN_MESSAGE_INS_TIME].transmitAddress = 0,
    .Message[CBOR_CAN_MESSAGE_INS_STATUS].periodMultiple = 0,
    .Message[CBOR_CAN_MESSAGE_INS_STATUS].transmitAddress = 0,
    .Message[CBOR_CAN_MESSAGE_INS_EULER].periodMultiple = 0,
    .Message[CBOR_CAN_MESSAGE_INS_EULER].transmitAddress = 0,
    .Message[CBOR_CAN_MESSAGE_INS_QUATN2B].periodMultiple = 0,
    .Message[CBOR_CAN_MESSAGE_INS_QUATN2B].transmitAddress = 0,
    .Message[CBOR_CAN_MESSAGE_INS_QUATE2B].periodMultiple = 0,
    .Message[CBOR_CAN_MESSAGE_INS_QUATE2B].transmitAddress = 0,
    .Message[CBOR_CAN_MESSAGE_INS_UVW].periodMultiple = 0,
    .Message[CBOR_CAN_MESSAGE_INS_UVW].transmitAddress = 0,
    .Message[CBOR_CAN_MESSAGE_INS_VE].periodMultiple = 0,
    .Message[CBOR_CAN_MESSAGE_INS_VE].transmitAddress = 0,
    .Message[CBOR_CAN_MESSAGE_INS_LAT].periodMultiple = 0,
    .Message[CBOR_CAN_MESSAGE_INS_LAT].transmitAddress = 0,
    .Message[CBOR_CAN_MESSAGE_INS_LON].periodMultiple = 0,
    .Message[CBOR_CAN_MESSAGE_INS_LON].transmitAddress = 0,
    .Message[CBOR_CAN_MESSAGE_INS_ALT].periodMultiple = 0,
    .Message[CBOR_CAN_MESSAGE_INS_ALT].transmitAddress = 0,
    .Message[CBOR_CAN_MESSAGE_INS_NORTH_EAST].periodMultiple = 0,
    .Message[CBOR_CAN_MESSAGE_INS_NORTH_EAST].transmitAddress = 0,
    .Message[CBOR_CAN_MESSAGE_INS_DOWN].periodMultiple = 0,
    .Message[CBOR_CAN_MESSAGE_INS_DOWN].transmitAddress = 0,
    .Message[CBOR_CAN_MESSAGE_INS_ECEF_X].periodMultiple = 0,
    .Message[CBOR_CAN_MESSAGE_INS_ECEF_X].transmitAddress = 0,
    .Message[CBOR_CAN_MESSAGE_INS_ECEF_Y].periodMultiple = 0,
    .Message[CBOR_CAN_MESSAGE_INS_ECEF_Y].transmitAddress = 0,
    .Message[CBOR_CAN_MESSAGE_INS_ECEF_Z].periodMultiple = 0,
    .Message[CBOR_CAN_MESSAGE_INS_ECEF_Z].transmitAddress = 0,
    .Message[CBOR_CAN_MESSAGE_INS_MSL].periodMultiple = 0,
    .Message[CBOR_CAN_MESSAGE_INS_MSL].transmitAddress = 0,
    .Message[CBOR_CAN_MESSAGE_PREINT_PX].periodMultiple = 0,
    .Message[CBOR_CAN_MESSAGE_PREINT_PX].transmitAddress = 0,
    .Message[CBOR_CAN_MESSAGE_PREINT_QY].periodMultiple = 0,
    .Message[CBOR_CAN_MESSAGE_PREINT_QY].transmitAddress = 0,
    .Message[CBOR_CAN_MESSAGE_PREINT_RZ].periodMultiple = 0,
    .Message[CBOR_CAN_MESSAGE_PREINT_RZ].transmitAddress = 0,
    .Message[CBOR_CAN_MESSAGE_DUAL_PX].periodMultiple = 0,
    .Message[CBOR_CAN_MESSAGE_DUAL_PX].transmitAddress = 0,
    .Message[CBOR_CAN_MESSAGE_DUAL_QY].periodMultiple = 0,
    .Message[CBOR_CAN_MESSAGE_DUAL_QY].transmitAddress = 0,
    .Message[CBOR_CAN_MESSAGE_DUAL_RZ].periodMultiple = 0,
    .Message[CBOR_CAN_MESSAGE_DUAL_RZ].transmitAddress = 0,
    .Message[CBOR_CAN_MESSAGE_GNSS1_POS].periodMultiple = 0,
    .Message[CBOR_CAN_MESSAGE_GNSS1_POS].transmitAddress = 0,
    .Message[CBOR_CAN_MESSAGE_GNSS2_POS].periodMultiple = 0,
    .Message[CBOR_CAN_MESSAGE_GNSS2_POS].transmitAddress = 0,
    .Message[CBOR_CAN_MESSAGE_GNSS1_RTK_POS_REL].periodMultiple = 0,
    .Message[CBOR_CAN_MESSAGE_GNSS1_RTK_POS_REL].transmitAddress = 0,
    .Message[CBOR_CAN_MESSAGE_GNSS2_RTK_CMP_REL].periodMultiple = 0,
    .Message[CBOR_CAN_MESSAGE_GNSS2_RTK_CMP_REL].transmitAddress = 0,
    .Message[CBOR_CAN_MESSAGE_ROLL_ROLLRATE].periodMultiple = 0,
    .Message[CBOR_CAN_MESSAGE_ROLL_ROLLRATE].transmitAddress = 0,
};

const uint8_t g_cborDefaultFixed_CONFIG_CAN[57] = {
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
};


const cbor_registry_name_t g_cborNames_CONFIG_CAN[CBOR_CONFIG_CAN_NAME_COUNT] = {
    { 0x0000, "bitRateKbps" },
    { 0x0001, "fdEnabled" },
    { 0x0002, "receiveAddress" },
    { 0x0003, "Message" },
    { 0x0000, "INS_TIME" },
    { 0x0000, "periodMultiple" },
    { 0x0001, "transmitAddress" },
    { 0x0001, "INS_STATUS" },
    { 0x0000, "periodMultiple" },
    { 0x0001, "transmitAddress" },
    { 0x0002, "INS_EULER" },
    { 0x0000, "periodMultiple" },
    { 0x0001, "transmitAddress" },
    { 0x0003, "INS_QUATN2B" },
    { 0x0000, "periodMultiple" },
    { 0x0001, "transmitAddress" },
    { 0x0004, "INS_QUATE2B" },
    { 0x0000, "periodMultiple" },
    { 0x0001, "transmitAddress" },
    { 0x0005, "INS_UVW" },
    { 0x0000, "periodMultiple" },
    { 0x0001, "transmitAddress" },
    { 0x0006, "INS_VE" },
    { 0x0000, "periodMultiple" },
    { 0x0001, "transmitAddress" },
    { 0x0007, "INS_LAT" },
    { 0x0000, "periodMultiple" },
    { 0x0001, "transmitAddress" },
    { 0x0008, "INS_LON" },
    { 0x0000, "periodMultiple" },
    { 0x0001, "transmitAddress" },
    { 0x0009, "INS_ALT" },
    { 0x0000, "periodMultiple" },
    { 0x0001, "transmitAddress" },
    { 0x000A, "INS_NORTH_EAST" },
    { 0x0000, "periodMultiple" },
    { 0x0001, "transmitAddress" },
    { 0x000B, "INS_DOWN" },
    { 0x0000, "periodMultiple" },
    { 0x0001, "transmitAddress" },
    { 0x000C, "INS_ECEF_X" },
    { 0x0000, "periodMultiple" },
    { 0x0001, "transmitAddress" },
    { 0x000D, "INS_ECEF_Y" },
    { 0x0000, "periodMultiple" },
    { 0x0001, "transmitAddress" },
    { 0x000E, "INS_ECEF_Z" },
    { 0x0000, "periodMultiple" },
    { 0x0001, "transmitAddress" },
    { 0x000F, "INS_MSL" },
    { 0x0000, "periodMultiple" },
    { 0x0001, "transmitAddress" },
    { 0x0010, "PREINT_PX" },
    { 0x0000, "periodMultiple" },
    { 0x0001, "transmitAddress" },
    { 0x0011, "PREINT_QY" },
    { 0x0000, "periodMultiple" },
    { 0x0001, "transmitAddress" },
    { 0x0012, "PREINT_RZ" },
    { 0x0000, "periodMultiple" },
    { 0x0001, "transmitAddress" },
    { 0x0013, "DUAL_PX" },
    { 0x0000, "periodMultiple" },
    { 0x0001, "transmitAddress" },
    { 0x0014, "DUAL_QY" },
    { 0x0000, "periodMultiple" },
    { 0x0001, "transmitAddress" },
    { 0x0015, "DUAL_RZ" },
    { 0x0000, "periodMultiple" },
    { 0x0001, "transmitAddress" },
    { 0x0016, "GNSS1_POS" },
    { 0x0000, "periodMultiple" },
    { 0x0001, "transmitAddress" },
    { 0x0017, "GNSS2_POS" },
    { 0x0000, "periodMultiple" },
    { 0x0001, "transmitAddress" },
    { 0x0018, "GNSS1_RTK_POS_REL" },
    { 0x0000, "periodMultiple" },
    { 0x0001, "transmitAddress" },
    { 0x0019, "GNSS2_RTK_CMP_REL" },
    { 0x0000, "periodMultiple" },
    { 0x0001, "transmitAddress" },
    { 0x001A, "ROLL_ROLLRATE" },
    { 0x0000, "periodMultiple" },
    { 0x0001, "transmitAddress" },
};

const uint16_t g_cborNameParents_CONFIG_CAN[CBOR_CONFIG_CAN_NAME_COUNT] = {
    0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 3, 4, 4, 3, 7, 7, 3, 10, 10, 3, 13, 13,
    3, 16, 16, 3, 19, 19, 3, 22, 22, 3, 25, 25, 3, 28, 28, 3,
    31, 31, 3, 34, 34, 3, 37, 37, 3, 40, 40, 3, 43, 43, 3, 46,
    46, 3, 49, 49, 3, 52, 52, 3, 55, 55, 3, 58, 58, 3, 61, 61,
    3, 64, 64, 3, 67, 67, 3, 70, 70, 3, 73, 73, 3, 76, 76, 3,
    79, 79, 3, 82, 82,
};
