/**
 * @file cbor_did_config_sensors.c
 * @brief CONFIG_SENSORS: field names.
 *
 * Generated from the CBOR key registry by is-common scripts/cbor_registry.py -- do not edit.
 *
 * @copyright Copyright (c) 2026 Inertial Sense, Inc.
 */

#include <stddef.h>
#include "cbor/cbor_registry.h"

static const cbor_field_t s_fields_config_sensors_imus_gyr_primary[] = {
    { 0, CBOR_T_UINT, 1, 0, offsetof(cbor_keyset_channel_t, fullScale), NULL },
    { 1, CBOR_T_UINT, 1, 0, offsetof(cbor_keyset_channel_t, dlpf), NULL },
};
static const cbor_schema_t s_schema_config_sensors_imus_gyr_primary = { s_fields_config_sensors_imus_gyr_primary, sizeof(s_fields_config_sensors_imus_gyr_primary) / sizeof(s_fields_config_sensors_imus_gyr_primary[0]) };

static const cbor_field_t s_fields_config_sensors_imus_gyr_high_g[] = {
    { 0, CBOR_T_UINT, 1, 0, offsetof(cbor_keyset_channel_t, fullScale), NULL },
    { 1, CBOR_T_UINT, 1, 0, offsetof(cbor_keyset_channel_t, dlpf), NULL },
};
static const cbor_schema_t s_schema_config_sensors_imus_gyr_high_g = { s_fields_config_sensors_imus_gyr_high_g, sizeof(s_fields_config_sensors_imus_gyr_high_g) / sizeof(s_fields_config_sensors_imus_gyr_high_g[0]) };

static const cbor_field_t s_fields_config_sensors_imus_gyr[] = {
    { 0, CBOR_T_MAP, 0, 0, CBOR_IMU_CHANNEL_PRIMARY * sizeof(cbor_keyset_channel_t), &s_schema_config_sensors_imus_gyr_primary },
    { 1, CBOR_T_MAP, 0, 0, CBOR_IMU_CHANNEL_HIGH_G * sizeof(cbor_keyset_channel_t), &s_schema_config_sensors_imus_gyr_high_g },
};
static const cbor_schema_t s_schema_config_sensors_imus_gyr = { s_fields_config_sensors_imus_gyr, sizeof(s_fields_config_sensors_imus_gyr) / sizeof(s_fields_config_sensors_imus_gyr[0]) };

static const cbor_field_t s_fields_config_sensors_imus_acc_primary[] = {
    { 0, CBOR_T_UINT, 1, 0, offsetof(cbor_keyset_channel_t, fullScale), NULL },
    { 1, CBOR_T_UINT, 1, 0, offsetof(cbor_keyset_channel_t, dlpf), NULL },
};
static const cbor_schema_t s_schema_config_sensors_imus_acc_primary = { s_fields_config_sensors_imus_acc_primary, sizeof(s_fields_config_sensors_imus_acc_primary) / sizeof(s_fields_config_sensors_imus_acc_primary[0]) };

static const cbor_field_t s_fields_config_sensors_imus_acc_high_g[] = {
    { 0, CBOR_T_UINT, 1, 0, offsetof(cbor_keyset_channel_t, fullScale), NULL },
    { 1, CBOR_T_UINT, 1, 0, offsetof(cbor_keyset_channel_t, dlpf), NULL },
};
static const cbor_schema_t s_schema_config_sensors_imus_acc_high_g = { s_fields_config_sensors_imus_acc_high_g, sizeof(s_fields_config_sensors_imus_acc_high_g) / sizeof(s_fields_config_sensors_imus_acc_high_g[0]) };

static const cbor_field_t s_fields_config_sensors_imus_acc[] = {
    { 0, CBOR_T_MAP, 0, 0, CBOR_IMU_CHANNEL_PRIMARY * sizeof(cbor_keyset_channel_t), &s_schema_config_sensors_imus_acc_primary },
    { 1, CBOR_T_MAP, 0, 0, CBOR_IMU_CHANNEL_HIGH_G * sizeof(cbor_keyset_channel_t), &s_schema_config_sensors_imus_acc_high_g },
};
static const cbor_schema_t s_schema_config_sensors_imus_acc = { s_fields_config_sensors_imus_acc, sizeof(s_fields_config_sensors_imus_acc) / sizeof(s_fields_config_sensors_imus_acc[0]) };

static const cbor_field_t s_fields_config_sensors_imus[] = {
    { 0, CBOR_T_BOOL, 1, 0, offsetof(cbor_keyset_imu_t, enabled), NULL },
    { 1, CBOR_T_MAP, 0, 0, offsetof(cbor_keyset_imu_t, Gyr), &s_schema_config_sensors_imus_gyr },
    { 2, CBOR_T_MAP, 0, 0, offsetof(cbor_keyset_imu_t, Acc), &s_schema_config_sensors_imus_acc },
};
static const cbor_schema_t s_schema_config_sensors_imus = { s_fields_config_sensors_imus, sizeof(s_fields_config_sensors_imus) / sizeof(s_fields_config_sensors_imus[0]) };

static const cbor_field_t s_fields_config_sensors_imu_imu1_gyr_primary[] = {
    { 0, CBOR_T_UINT, 1, 0, offsetof(cbor_keyset_channel_t, fullScale), NULL },
    { 1, CBOR_T_UINT, 1, 0, offsetof(cbor_keyset_channel_t, dlpf), NULL },
};
static const cbor_schema_t s_schema_config_sensors_imu_imu1_gyr_primary = { s_fields_config_sensors_imu_imu1_gyr_primary, sizeof(s_fields_config_sensors_imu_imu1_gyr_primary) / sizeof(s_fields_config_sensors_imu_imu1_gyr_primary[0]) };

static const cbor_field_t s_fields_config_sensors_imu_imu1_gyr_high_g[] = {
    { 0, CBOR_T_UINT, 1, 0, offsetof(cbor_keyset_channel_t, fullScale), NULL },
    { 1, CBOR_T_UINT, 1, 0, offsetof(cbor_keyset_channel_t, dlpf), NULL },
};
static const cbor_schema_t s_schema_config_sensors_imu_imu1_gyr_high_g = { s_fields_config_sensors_imu_imu1_gyr_high_g, sizeof(s_fields_config_sensors_imu_imu1_gyr_high_g) / sizeof(s_fields_config_sensors_imu_imu1_gyr_high_g[0]) };

static const cbor_field_t s_fields_config_sensors_imu_imu1_gyr[] = {
    { 0, CBOR_T_MAP, 0, 0, CBOR_IMU_CHANNEL_PRIMARY * sizeof(cbor_keyset_channel_t), &s_schema_config_sensors_imu_imu1_gyr_primary },
    { 1, CBOR_T_MAP, 0, 0, CBOR_IMU_CHANNEL_HIGH_G * sizeof(cbor_keyset_channel_t), &s_schema_config_sensors_imu_imu1_gyr_high_g },
};
static const cbor_schema_t s_schema_config_sensors_imu_imu1_gyr = { s_fields_config_sensors_imu_imu1_gyr, sizeof(s_fields_config_sensors_imu_imu1_gyr) / sizeof(s_fields_config_sensors_imu_imu1_gyr[0]) };

static const cbor_field_t s_fields_config_sensors_imu_imu1_acc_primary[] = {
    { 0, CBOR_T_UINT, 1, 0, offsetof(cbor_keyset_channel_t, fullScale), NULL },
    { 1, CBOR_T_UINT, 1, 0, offsetof(cbor_keyset_channel_t, dlpf), NULL },
};
static const cbor_schema_t s_schema_config_sensors_imu_imu1_acc_primary = { s_fields_config_sensors_imu_imu1_acc_primary, sizeof(s_fields_config_sensors_imu_imu1_acc_primary) / sizeof(s_fields_config_sensors_imu_imu1_acc_primary[0]) };

static const cbor_field_t s_fields_config_sensors_imu_imu1_acc_high_g[] = {
    { 0, CBOR_T_UINT, 1, 0, offsetof(cbor_keyset_channel_t, fullScale), NULL },
    { 1, CBOR_T_UINT, 1, 0, offsetof(cbor_keyset_channel_t, dlpf), NULL },
};
static const cbor_schema_t s_schema_config_sensors_imu_imu1_acc_high_g = { s_fields_config_sensors_imu_imu1_acc_high_g, sizeof(s_fields_config_sensors_imu_imu1_acc_high_g) / sizeof(s_fields_config_sensors_imu_imu1_acc_high_g[0]) };

static const cbor_field_t s_fields_config_sensors_imu_imu1_acc[] = {
    { 0, CBOR_T_MAP, 0, 0, CBOR_IMU_CHANNEL_PRIMARY * sizeof(cbor_keyset_channel_t), &s_schema_config_sensors_imu_imu1_acc_primary },
    { 1, CBOR_T_MAP, 0, 0, CBOR_IMU_CHANNEL_HIGH_G * sizeof(cbor_keyset_channel_t), &s_schema_config_sensors_imu_imu1_acc_high_g },
};
static const cbor_schema_t s_schema_config_sensors_imu_imu1_acc = { s_fields_config_sensors_imu_imu1_acc, sizeof(s_fields_config_sensors_imu_imu1_acc) / sizeof(s_fields_config_sensors_imu_imu1_acc[0]) };

static const cbor_field_t s_fields_config_sensors_imu_imu1[] = {
    { 0, CBOR_T_BOOL, 1, 0, offsetof(cbor_keyset_imu_t, enabled), NULL },
    { 1, CBOR_T_MAP, 0, 0, offsetof(cbor_keyset_imu_t, Gyr), &s_schema_config_sensors_imu_imu1_gyr },
    { 2, CBOR_T_MAP, 0, 0, offsetof(cbor_keyset_imu_t, Acc), &s_schema_config_sensors_imu_imu1_acc },
};
static const cbor_schema_t s_schema_config_sensors_imu_imu1 = { s_fields_config_sensors_imu_imu1, sizeof(s_fields_config_sensors_imu_imu1) / sizeof(s_fields_config_sensors_imu_imu1[0]) };

static const cbor_field_t s_fields_config_sensors_imu_imu2_gyr_primary[] = {
    { 0, CBOR_T_UINT, 1, 0, offsetof(cbor_keyset_channel_t, fullScale), NULL },
    { 1, CBOR_T_UINT, 1, 0, offsetof(cbor_keyset_channel_t, dlpf), NULL },
};
static const cbor_schema_t s_schema_config_sensors_imu_imu2_gyr_primary = { s_fields_config_sensors_imu_imu2_gyr_primary, sizeof(s_fields_config_sensors_imu_imu2_gyr_primary) / sizeof(s_fields_config_sensors_imu_imu2_gyr_primary[0]) };

static const cbor_field_t s_fields_config_sensors_imu_imu2_gyr_high_g[] = {
    { 0, CBOR_T_UINT, 1, 0, offsetof(cbor_keyset_channel_t, fullScale), NULL },
    { 1, CBOR_T_UINT, 1, 0, offsetof(cbor_keyset_channel_t, dlpf), NULL },
};
static const cbor_schema_t s_schema_config_sensors_imu_imu2_gyr_high_g = { s_fields_config_sensors_imu_imu2_gyr_high_g, sizeof(s_fields_config_sensors_imu_imu2_gyr_high_g) / sizeof(s_fields_config_sensors_imu_imu2_gyr_high_g[0]) };

static const cbor_field_t s_fields_config_sensors_imu_imu2_gyr[] = {
    { 0, CBOR_T_MAP, 0, 0, CBOR_IMU_CHANNEL_PRIMARY * sizeof(cbor_keyset_channel_t), &s_schema_config_sensors_imu_imu2_gyr_primary },
    { 1, CBOR_T_MAP, 0, 0, CBOR_IMU_CHANNEL_HIGH_G * sizeof(cbor_keyset_channel_t), &s_schema_config_sensors_imu_imu2_gyr_high_g },
};
static const cbor_schema_t s_schema_config_sensors_imu_imu2_gyr = { s_fields_config_sensors_imu_imu2_gyr, sizeof(s_fields_config_sensors_imu_imu2_gyr) / sizeof(s_fields_config_sensors_imu_imu2_gyr[0]) };

static const cbor_field_t s_fields_config_sensors_imu_imu2_acc_primary[] = {
    { 0, CBOR_T_UINT, 1, 0, offsetof(cbor_keyset_channel_t, fullScale), NULL },
    { 1, CBOR_T_UINT, 1, 0, offsetof(cbor_keyset_channel_t, dlpf), NULL },
};
static const cbor_schema_t s_schema_config_sensors_imu_imu2_acc_primary = { s_fields_config_sensors_imu_imu2_acc_primary, sizeof(s_fields_config_sensors_imu_imu2_acc_primary) / sizeof(s_fields_config_sensors_imu_imu2_acc_primary[0]) };

static const cbor_field_t s_fields_config_sensors_imu_imu2_acc_high_g[] = {
    { 0, CBOR_T_UINT, 1, 0, offsetof(cbor_keyset_channel_t, fullScale), NULL },
    { 1, CBOR_T_UINT, 1, 0, offsetof(cbor_keyset_channel_t, dlpf), NULL },
};
static const cbor_schema_t s_schema_config_sensors_imu_imu2_acc_high_g = { s_fields_config_sensors_imu_imu2_acc_high_g, sizeof(s_fields_config_sensors_imu_imu2_acc_high_g) / sizeof(s_fields_config_sensors_imu_imu2_acc_high_g[0]) };

static const cbor_field_t s_fields_config_sensors_imu_imu2_acc[] = {
    { 0, CBOR_T_MAP, 0, 0, CBOR_IMU_CHANNEL_PRIMARY * sizeof(cbor_keyset_channel_t), &s_schema_config_sensors_imu_imu2_acc_primary },
    { 1, CBOR_T_MAP, 0, 0, CBOR_IMU_CHANNEL_HIGH_G * sizeof(cbor_keyset_channel_t), &s_schema_config_sensors_imu_imu2_acc_high_g },
};
static const cbor_schema_t s_schema_config_sensors_imu_imu2_acc = { s_fields_config_sensors_imu_imu2_acc, sizeof(s_fields_config_sensors_imu_imu2_acc) / sizeof(s_fields_config_sensors_imu_imu2_acc[0]) };

static const cbor_field_t s_fields_config_sensors_imu_imu2[] = {
    { 0, CBOR_T_BOOL, 1, 0, offsetof(cbor_keyset_imu_t, enabled), NULL },
    { 1, CBOR_T_MAP, 0, 0, offsetof(cbor_keyset_imu_t, Gyr), &s_schema_config_sensors_imu_imu2_gyr },
    { 2, CBOR_T_MAP, 0, 0, offsetof(cbor_keyset_imu_t, Acc), &s_schema_config_sensors_imu_imu2_acc },
};
static const cbor_schema_t s_schema_config_sensors_imu_imu2 = { s_fields_config_sensors_imu_imu2, sizeof(s_fields_config_sensors_imu_imu2) / sizeof(s_fields_config_sensors_imu_imu2[0]) };

static const cbor_field_t s_fields_config_sensors_imu_imu3_gyr_primary[] = {
    { 0, CBOR_T_UINT, 1, 0, offsetof(cbor_keyset_channel_t, fullScale), NULL },
    { 1, CBOR_T_UINT, 1, 0, offsetof(cbor_keyset_channel_t, dlpf), NULL },
};
static const cbor_schema_t s_schema_config_sensors_imu_imu3_gyr_primary = { s_fields_config_sensors_imu_imu3_gyr_primary, sizeof(s_fields_config_sensors_imu_imu3_gyr_primary) / sizeof(s_fields_config_sensors_imu_imu3_gyr_primary[0]) };

static const cbor_field_t s_fields_config_sensors_imu_imu3_gyr_high_g[] = {
    { 0, CBOR_T_UINT, 1, 0, offsetof(cbor_keyset_channel_t, fullScale), NULL },
    { 1, CBOR_T_UINT, 1, 0, offsetof(cbor_keyset_channel_t, dlpf), NULL },
};
static const cbor_schema_t s_schema_config_sensors_imu_imu3_gyr_high_g = { s_fields_config_sensors_imu_imu3_gyr_high_g, sizeof(s_fields_config_sensors_imu_imu3_gyr_high_g) / sizeof(s_fields_config_sensors_imu_imu3_gyr_high_g[0]) };

static const cbor_field_t s_fields_config_sensors_imu_imu3_gyr[] = {
    { 0, CBOR_T_MAP, 0, 0, CBOR_IMU_CHANNEL_PRIMARY * sizeof(cbor_keyset_channel_t), &s_schema_config_sensors_imu_imu3_gyr_primary },
    { 1, CBOR_T_MAP, 0, 0, CBOR_IMU_CHANNEL_HIGH_G * sizeof(cbor_keyset_channel_t), &s_schema_config_sensors_imu_imu3_gyr_high_g },
};
static const cbor_schema_t s_schema_config_sensors_imu_imu3_gyr = { s_fields_config_sensors_imu_imu3_gyr, sizeof(s_fields_config_sensors_imu_imu3_gyr) / sizeof(s_fields_config_sensors_imu_imu3_gyr[0]) };

static const cbor_field_t s_fields_config_sensors_imu_imu3_acc_primary[] = {
    { 0, CBOR_T_UINT, 1, 0, offsetof(cbor_keyset_channel_t, fullScale), NULL },
    { 1, CBOR_T_UINT, 1, 0, offsetof(cbor_keyset_channel_t, dlpf), NULL },
};
static const cbor_schema_t s_schema_config_sensors_imu_imu3_acc_primary = { s_fields_config_sensors_imu_imu3_acc_primary, sizeof(s_fields_config_sensors_imu_imu3_acc_primary) / sizeof(s_fields_config_sensors_imu_imu3_acc_primary[0]) };

static const cbor_field_t s_fields_config_sensors_imu_imu3_acc_high_g[] = {
    { 0, CBOR_T_UINT, 1, 0, offsetof(cbor_keyset_channel_t, fullScale), NULL },
    { 1, CBOR_T_UINT, 1, 0, offsetof(cbor_keyset_channel_t, dlpf), NULL },
};
static const cbor_schema_t s_schema_config_sensors_imu_imu3_acc_high_g = { s_fields_config_sensors_imu_imu3_acc_high_g, sizeof(s_fields_config_sensors_imu_imu3_acc_high_g) / sizeof(s_fields_config_sensors_imu_imu3_acc_high_g[0]) };

static const cbor_field_t s_fields_config_sensors_imu_imu3_acc[] = {
    { 0, CBOR_T_MAP, 0, 0, CBOR_IMU_CHANNEL_PRIMARY * sizeof(cbor_keyset_channel_t), &s_schema_config_sensors_imu_imu3_acc_primary },
    { 1, CBOR_T_MAP, 0, 0, CBOR_IMU_CHANNEL_HIGH_G * sizeof(cbor_keyset_channel_t), &s_schema_config_sensors_imu_imu3_acc_high_g },
};
static const cbor_schema_t s_schema_config_sensors_imu_imu3_acc = { s_fields_config_sensors_imu_imu3_acc, sizeof(s_fields_config_sensors_imu_imu3_acc) / sizeof(s_fields_config_sensors_imu_imu3_acc[0]) };

static const cbor_field_t s_fields_config_sensors_imu_imu3[] = {
    { 0, CBOR_T_BOOL, 1, 0, offsetof(cbor_keyset_imu_t, enabled), NULL },
    { 1, CBOR_T_MAP, 0, 0, offsetof(cbor_keyset_imu_t, Gyr), &s_schema_config_sensors_imu_imu3_gyr },
    { 2, CBOR_T_MAP, 0, 0, offsetof(cbor_keyset_imu_t, Acc), &s_schema_config_sensors_imu_imu3_acc },
};
static const cbor_schema_t s_schema_config_sensors_imu_imu3 = { s_fields_config_sensors_imu_imu3, sizeof(s_fields_config_sensors_imu_imu3) / sizeof(s_fields_config_sensors_imu_imu3[0]) };

static const cbor_field_t s_fields_config_sensors_imu_imu4_gyr_primary[] = {
    { 0, CBOR_T_UINT, 1, 0, offsetof(cbor_keyset_channel_t, fullScale), NULL },
    { 1, CBOR_T_UINT, 1, 0, offsetof(cbor_keyset_channel_t, dlpf), NULL },
};
static const cbor_schema_t s_schema_config_sensors_imu_imu4_gyr_primary = { s_fields_config_sensors_imu_imu4_gyr_primary, sizeof(s_fields_config_sensors_imu_imu4_gyr_primary) / sizeof(s_fields_config_sensors_imu_imu4_gyr_primary[0]) };

static const cbor_field_t s_fields_config_sensors_imu_imu4_gyr_high_g[] = {
    { 0, CBOR_T_UINT, 1, 0, offsetof(cbor_keyset_channel_t, fullScale), NULL },
    { 1, CBOR_T_UINT, 1, 0, offsetof(cbor_keyset_channel_t, dlpf), NULL },
};
static const cbor_schema_t s_schema_config_sensors_imu_imu4_gyr_high_g = { s_fields_config_sensors_imu_imu4_gyr_high_g, sizeof(s_fields_config_sensors_imu_imu4_gyr_high_g) / sizeof(s_fields_config_sensors_imu_imu4_gyr_high_g[0]) };

static const cbor_field_t s_fields_config_sensors_imu_imu4_gyr[] = {
    { 0, CBOR_T_MAP, 0, 0, CBOR_IMU_CHANNEL_PRIMARY * sizeof(cbor_keyset_channel_t), &s_schema_config_sensors_imu_imu4_gyr_primary },
    { 1, CBOR_T_MAP, 0, 0, CBOR_IMU_CHANNEL_HIGH_G * sizeof(cbor_keyset_channel_t), &s_schema_config_sensors_imu_imu4_gyr_high_g },
};
static const cbor_schema_t s_schema_config_sensors_imu_imu4_gyr = { s_fields_config_sensors_imu_imu4_gyr, sizeof(s_fields_config_sensors_imu_imu4_gyr) / sizeof(s_fields_config_sensors_imu_imu4_gyr[0]) };

static const cbor_field_t s_fields_config_sensors_imu_imu4_acc_primary[] = {
    { 0, CBOR_T_UINT, 1, 0, offsetof(cbor_keyset_channel_t, fullScale), NULL },
    { 1, CBOR_T_UINT, 1, 0, offsetof(cbor_keyset_channel_t, dlpf), NULL },
};
static const cbor_schema_t s_schema_config_sensors_imu_imu4_acc_primary = { s_fields_config_sensors_imu_imu4_acc_primary, sizeof(s_fields_config_sensors_imu_imu4_acc_primary) / sizeof(s_fields_config_sensors_imu_imu4_acc_primary[0]) };

static const cbor_field_t s_fields_config_sensors_imu_imu4_acc_high_g[] = {
    { 0, CBOR_T_UINT, 1, 0, offsetof(cbor_keyset_channel_t, fullScale), NULL },
    { 1, CBOR_T_UINT, 1, 0, offsetof(cbor_keyset_channel_t, dlpf), NULL },
};
static const cbor_schema_t s_schema_config_sensors_imu_imu4_acc_high_g = { s_fields_config_sensors_imu_imu4_acc_high_g, sizeof(s_fields_config_sensors_imu_imu4_acc_high_g) / sizeof(s_fields_config_sensors_imu_imu4_acc_high_g[0]) };

static const cbor_field_t s_fields_config_sensors_imu_imu4_acc[] = {
    { 0, CBOR_T_MAP, 0, 0, CBOR_IMU_CHANNEL_PRIMARY * sizeof(cbor_keyset_channel_t), &s_schema_config_sensors_imu_imu4_acc_primary },
    { 1, CBOR_T_MAP, 0, 0, CBOR_IMU_CHANNEL_HIGH_G * sizeof(cbor_keyset_channel_t), &s_schema_config_sensors_imu_imu4_acc_high_g },
};
static const cbor_schema_t s_schema_config_sensors_imu_imu4_acc = { s_fields_config_sensors_imu_imu4_acc, sizeof(s_fields_config_sensors_imu_imu4_acc) / sizeof(s_fields_config_sensors_imu_imu4_acc[0]) };

static const cbor_field_t s_fields_config_sensors_imu_imu4[] = {
    { 0, CBOR_T_BOOL, 1, 0, offsetof(cbor_keyset_imu_t, enabled), NULL },
    { 1, CBOR_T_MAP, 0, 0, offsetof(cbor_keyset_imu_t, Gyr), &s_schema_config_sensors_imu_imu4_gyr },
    { 2, CBOR_T_MAP, 0, 0, offsetof(cbor_keyset_imu_t, Acc), &s_schema_config_sensors_imu_imu4_acc },
};
static const cbor_schema_t s_schema_config_sensors_imu_imu4 = { s_fields_config_sensors_imu_imu4, sizeof(s_fields_config_sensors_imu_imu4) / sizeof(s_fields_config_sensors_imu_imu4[0]) };

static const cbor_field_t s_fields_config_sensors_imu_imu5_gyr_primary[] = {
    { 0, CBOR_T_UINT, 1, 0, offsetof(cbor_keyset_channel_t, fullScale), NULL },
    { 1, CBOR_T_UINT, 1, 0, offsetof(cbor_keyset_channel_t, dlpf), NULL },
};
static const cbor_schema_t s_schema_config_sensors_imu_imu5_gyr_primary = { s_fields_config_sensors_imu_imu5_gyr_primary, sizeof(s_fields_config_sensors_imu_imu5_gyr_primary) / sizeof(s_fields_config_sensors_imu_imu5_gyr_primary[0]) };

static const cbor_field_t s_fields_config_sensors_imu_imu5_gyr_high_g[] = {
    { 0, CBOR_T_UINT, 1, 0, offsetof(cbor_keyset_channel_t, fullScale), NULL },
    { 1, CBOR_T_UINT, 1, 0, offsetof(cbor_keyset_channel_t, dlpf), NULL },
};
static const cbor_schema_t s_schema_config_sensors_imu_imu5_gyr_high_g = { s_fields_config_sensors_imu_imu5_gyr_high_g, sizeof(s_fields_config_sensors_imu_imu5_gyr_high_g) / sizeof(s_fields_config_sensors_imu_imu5_gyr_high_g[0]) };

static const cbor_field_t s_fields_config_sensors_imu_imu5_gyr[] = {
    { 0, CBOR_T_MAP, 0, 0, CBOR_IMU_CHANNEL_PRIMARY * sizeof(cbor_keyset_channel_t), &s_schema_config_sensors_imu_imu5_gyr_primary },
    { 1, CBOR_T_MAP, 0, 0, CBOR_IMU_CHANNEL_HIGH_G * sizeof(cbor_keyset_channel_t), &s_schema_config_sensors_imu_imu5_gyr_high_g },
};
static const cbor_schema_t s_schema_config_sensors_imu_imu5_gyr = { s_fields_config_sensors_imu_imu5_gyr, sizeof(s_fields_config_sensors_imu_imu5_gyr) / sizeof(s_fields_config_sensors_imu_imu5_gyr[0]) };

static const cbor_field_t s_fields_config_sensors_imu_imu5_acc_primary[] = {
    { 0, CBOR_T_UINT, 1, 0, offsetof(cbor_keyset_channel_t, fullScale), NULL },
    { 1, CBOR_T_UINT, 1, 0, offsetof(cbor_keyset_channel_t, dlpf), NULL },
};
static const cbor_schema_t s_schema_config_sensors_imu_imu5_acc_primary = { s_fields_config_sensors_imu_imu5_acc_primary, sizeof(s_fields_config_sensors_imu_imu5_acc_primary) / sizeof(s_fields_config_sensors_imu_imu5_acc_primary[0]) };

static const cbor_field_t s_fields_config_sensors_imu_imu5_acc_high_g[] = {
    { 0, CBOR_T_UINT, 1, 0, offsetof(cbor_keyset_channel_t, fullScale), NULL },
    { 1, CBOR_T_UINT, 1, 0, offsetof(cbor_keyset_channel_t, dlpf), NULL },
};
static const cbor_schema_t s_schema_config_sensors_imu_imu5_acc_high_g = { s_fields_config_sensors_imu_imu5_acc_high_g, sizeof(s_fields_config_sensors_imu_imu5_acc_high_g) / sizeof(s_fields_config_sensors_imu_imu5_acc_high_g[0]) };

static const cbor_field_t s_fields_config_sensors_imu_imu5_acc[] = {
    { 0, CBOR_T_MAP, 0, 0, CBOR_IMU_CHANNEL_PRIMARY * sizeof(cbor_keyset_channel_t), &s_schema_config_sensors_imu_imu5_acc_primary },
    { 1, CBOR_T_MAP, 0, 0, CBOR_IMU_CHANNEL_HIGH_G * sizeof(cbor_keyset_channel_t), &s_schema_config_sensors_imu_imu5_acc_high_g },
};
static const cbor_schema_t s_schema_config_sensors_imu_imu5_acc = { s_fields_config_sensors_imu_imu5_acc, sizeof(s_fields_config_sensors_imu_imu5_acc) / sizeof(s_fields_config_sensors_imu_imu5_acc[0]) };

static const cbor_field_t s_fields_config_sensors_imu_imu5[] = {
    { 0, CBOR_T_BOOL, 1, 0, offsetof(cbor_keyset_imu_t, enabled), NULL },
    { 1, CBOR_T_MAP, 0, 0, offsetof(cbor_keyset_imu_t, Gyr), &s_schema_config_sensors_imu_imu5_gyr },
    { 2, CBOR_T_MAP, 0, 0, offsetof(cbor_keyset_imu_t, Acc), &s_schema_config_sensors_imu_imu5_acc },
};
static const cbor_schema_t s_schema_config_sensors_imu_imu5 = { s_fields_config_sensors_imu_imu5, sizeof(s_fields_config_sensors_imu_imu5) / sizeof(s_fields_config_sensors_imu_imu5[0]) };

static const cbor_field_t s_fields_config_sensors_imu[] = {
    { 0, CBOR_T_MAP, 0, 0, CBOR_IMU_SLOT_IMU1 * sizeof(cbor_keyset_imu_t), &s_schema_config_sensors_imu_imu1 },
    { 1, CBOR_T_MAP, 0, 0, CBOR_IMU_SLOT_IMU2 * sizeof(cbor_keyset_imu_t), &s_schema_config_sensors_imu_imu2 },
    { 2, CBOR_T_MAP, 0, 0, CBOR_IMU_SLOT_IMU3 * sizeof(cbor_keyset_imu_t), &s_schema_config_sensors_imu_imu3 },
    { 3, CBOR_T_MAP, 0, 0, CBOR_IMU_SLOT_IMU4 * sizeof(cbor_keyset_imu_t), &s_schema_config_sensors_imu_imu4 },
    { 4, CBOR_T_MAP, 0, 0, CBOR_IMU_SLOT_IMU5 * sizeof(cbor_keyset_imu_t), &s_schema_config_sensors_imu_imu5 },
};
static const cbor_schema_t s_schema_config_sensors_imu = { s_fields_config_sensors_imu, sizeof(s_fields_config_sensors_imu) / sizeof(s_fields_config_sensors_imu[0]) };

static const cbor_field_t s_fields_config_sensors_mags[] = {
    { 0, CBOR_T_BOOL, 1, 0, offsetof(cbor_keyset_aux_t, enabled), NULL },
};
static const cbor_schema_t s_schema_config_sensors_mags = { s_fields_config_sensors_mags, sizeof(s_fields_config_sensors_mags) / sizeof(s_fields_config_sensors_mags[0]) };

static const cbor_field_t s_fields_config_sensors_mag_mag1[] = {
    { 0, CBOR_T_BOOL, 1, 0, offsetof(cbor_keyset_aux_t, enabled), NULL },
};
static const cbor_schema_t s_schema_config_sensors_mag_mag1 = { s_fields_config_sensors_mag_mag1, sizeof(s_fields_config_sensors_mag_mag1) / sizeof(s_fields_config_sensors_mag_mag1[0]) };

static const cbor_field_t s_fields_config_sensors_mag[] = {
    { 0, CBOR_T_MAP, 0, 0, CBOR_MAG_ID_MAG1 * sizeof(cbor_keyset_aux_t), &s_schema_config_sensors_mag_mag1 },
};
static const cbor_schema_t s_schema_config_sensors_mag = { s_fields_config_sensors_mag, sizeof(s_fields_config_sensors_mag) / sizeof(s_fields_config_sensors_mag[0]) };

static const cbor_field_t s_fields_config_sensors_baros[] = {
    { 0, CBOR_T_BOOL, 1, 0, offsetof(cbor_keyset_aux_t, enabled), NULL },
};
static const cbor_schema_t s_schema_config_sensors_baros = { s_fields_config_sensors_baros, sizeof(s_fields_config_sensors_baros) / sizeof(s_fields_config_sensors_baros[0]) };

static const cbor_field_t s_fields_config_sensors_baro_baro1[] = {
    { 0, CBOR_T_BOOL, 1, 0, offsetof(cbor_keyset_aux_t, enabled), NULL },
};
static const cbor_schema_t s_schema_config_sensors_baro_baro1 = { s_fields_config_sensors_baro_baro1, sizeof(s_fields_config_sensors_baro_baro1) / sizeof(s_fields_config_sensors_baro_baro1[0]) };

static const cbor_field_t s_fields_config_sensors_baro[] = {
    { 0, CBOR_T_MAP, 0, 0, CBOR_BARO_ID_BARO1 * sizeof(cbor_keyset_aux_t), &s_schema_config_sensors_baro_baro1 },
};
static const cbor_schema_t s_schema_config_sensors_baro = { s_fields_config_sensors_baro, sizeof(s_fields_config_sensors_baro) / sizeof(s_fields_config_sensors_baro[0]) };

static const cbor_field_t s_fields_config_sensors[] = {
    { 0, CBOR_T_UINT, 1, 0, offsetof(cbor_config_sensors_t, rotation), NULL },
    { 1, CBOR_T_MAP, 0, 0, offsetof(cbor_config_sensors_t, Imus), &s_schema_config_sensors_imus },
    { 2, CBOR_T_MAP, 0, 0, offsetof(cbor_config_sensors_t, Imu), &s_schema_config_sensors_imu },
    { 3, CBOR_T_MAP, 0, 0, offsetof(cbor_config_sensors_t, Mags), &s_schema_config_sensors_mags },
    { 4, CBOR_T_MAP, 0, 0, offsetof(cbor_config_sensors_t, Mag), &s_schema_config_sensors_mag },
    { 5, CBOR_T_MAP, 0, 0, offsetof(cbor_config_sensors_t, Baros), &s_schema_config_sensors_baros },
    { 6, CBOR_T_MAP, 0, 0, offsetof(cbor_config_sensors_t, Baro), &s_schema_config_sensors_baro },
};
const cbor_schema_t g_cborSchema_CONFIG_SENSORS = { s_fields_config_sensors, sizeof(s_fields_config_sensors) / sizeof(s_fields_config_sensors[0]) };

const cbor_config_sensors_t g_cborDefaults_CONFIG_SENSORS = {
    .rotation = 0,
    .Imus.enabled = 1,
    .Imus.Gyr[CBOR_IMU_CHANNEL_PRIMARY].dlpf = 0,
    .Imus.Gyr[CBOR_IMU_CHANNEL_HIGH_G].dlpf = 0,
    .Imus.Acc[CBOR_IMU_CHANNEL_PRIMARY].dlpf = 0,
    .Imus.Acc[CBOR_IMU_CHANNEL_HIGH_G].dlpf = 0,
    .Imu[CBOR_IMU_SLOT_IMU1].enabled = 1,
    .Imu[CBOR_IMU_SLOT_IMU1].Gyr[CBOR_IMU_CHANNEL_PRIMARY].dlpf = 0,
    .Imu[CBOR_IMU_SLOT_IMU1].Gyr[CBOR_IMU_CHANNEL_HIGH_G].dlpf = 0,
    .Imu[CBOR_IMU_SLOT_IMU1].Acc[CBOR_IMU_CHANNEL_PRIMARY].dlpf = 0,
    .Imu[CBOR_IMU_SLOT_IMU1].Acc[CBOR_IMU_CHANNEL_HIGH_G].dlpf = 0,
    .Imu[CBOR_IMU_SLOT_IMU2].enabled = 1,
    .Imu[CBOR_IMU_SLOT_IMU2].Gyr[CBOR_IMU_CHANNEL_PRIMARY].dlpf = 0,
    .Imu[CBOR_IMU_SLOT_IMU2].Gyr[CBOR_IMU_CHANNEL_HIGH_G].dlpf = 0,
    .Imu[CBOR_IMU_SLOT_IMU2].Acc[CBOR_IMU_CHANNEL_PRIMARY].dlpf = 0,
    .Imu[CBOR_IMU_SLOT_IMU2].Acc[CBOR_IMU_CHANNEL_HIGH_G].dlpf = 0,
    .Imu[CBOR_IMU_SLOT_IMU3].enabled = 1,
    .Imu[CBOR_IMU_SLOT_IMU3].Gyr[CBOR_IMU_CHANNEL_PRIMARY].dlpf = 0,
    .Imu[CBOR_IMU_SLOT_IMU3].Gyr[CBOR_IMU_CHANNEL_HIGH_G].dlpf = 0,
    .Imu[CBOR_IMU_SLOT_IMU3].Acc[CBOR_IMU_CHANNEL_PRIMARY].dlpf = 0,
    .Imu[CBOR_IMU_SLOT_IMU3].Acc[CBOR_IMU_CHANNEL_HIGH_G].dlpf = 0,
    .Imu[CBOR_IMU_SLOT_IMU4].enabled = 1,
    .Imu[CBOR_IMU_SLOT_IMU4].Gyr[CBOR_IMU_CHANNEL_PRIMARY].dlpf = 0,
    .Imu[CBOR_IMU_SLOT_IMU4].Gyr[CBOR_IMU_CHANNEL_HIGH_G].dlpf = 0,
    .Imu[CBOR_IMU_SLOT_IMU4].Acc[CBOR_IMU_CHANNEL_PRIMARY].dlpf = 0,
    .Imu[CBOR_IMU_SLOT_IMU4].Acc[CBOR_IMU_CHANNEL_HIGH_G].dlpf = 0,
    .Imu[CBOR_IMU_SLOT_IMU5].enabled = 1,
    .Imu[CBOR_IMU_SLOT_IMU5].Gyr[CBOR_IMU_CHANNEL_PRIMARY].dlpf = 0,
    .Imu[CBOR_IMU_SLOT_IMU5].Gyr[CBOR_IMU_CHANNEL_HIGH_G].dlpf = 0,
    .Imu[CBOR_IMU_SLOT_IMU5].Acc[CBOR_IMU_CHANNEL_PRIMARY].dlpf = 0,
    .Imu[CBOR_IMU_SLOT_IMU5].Acc[CBOR_IMU_CHANNEL_HIGH_G].dlpf = 0,
    .Mags.enabled = 1,
    .Mag[CBOR_MAG_ID_MAG1].enabled = 1,
    .Baros.enabled = 1,
    .Baro[CBOR_BARO_ID_BARO1].enabled = 1,
};

const uint8_t g_cborDefaultFixed_CONFIG_SENSORS[59] = {
    1, 1, 0, 1, 0, 1, 0, 1, 0, 1, 1, 0, 1, 0, 1, 0, 1, 0, 1, 1, 0, 1, 0, 1, 0, 1, 0, 1, 1, 0, 1, 0, 1, 0, 1, 0, 1, 1, 0, 1, 0, 1, 0, 1, 0, 1, 1, 0, 1, 0, 1, 0, 1, 0, 1, 1, 1, 1, 1,
};


const cbor_registry_name_t g_cborNames_CONFIG_SENSORS[CBOR_CONFIG_SENSORS_NAME_COUNT] = {
    { 0x0000, "rotation" },
    { 0x0001, "Imus" },
    { 0x0000, "enabled" },
    { 0x0001, "Gyr" },
    { 0x0000, "PRIMARY" },
    { 0x0000, "fullScale" },
    { 0x0001, "dlpf" },
    { 0x0001, "HIGH_G" },
    { 0x0000, "fullScale" },
    { 0x0001, "dlpf" },
    { 0x0002, "Acc" },
    { 0x0000, "PRIMARY" },
    { 0x0000, "fullScale" },
    { 0x0001, "dlpf" },
    { 0x0001, "HIGH_G" },
    { 0x0000, "fullScale" },
    { 0x0001, "dlpf" },
    { 0x0002, "Imu" },
    { 0x0000, "IMU1" },
    { 0x0000, "enabled" },
    { 0x0001, "Gyr" },
    { 0x0000, "PRIMARY" },
    { 0x0000, "fullScale" },
    { 0x0001, "dlpf" },
    { 0x0001, "HIGH_G" },
    { 0x0000, "fullScale" },
    { 0x0001, "dlpf" },
    { 0x0002, "Acc" },
    { 0x0000, "PRIMARY" },
    { 0x0000, "fullScale" },
    { 0x0001, "dlpf" },
    { 0x0001, "HIGH_G" },
    { 0x0000, "fullScale" },
    { 0x0001, "dlpf" },
    { 0x0001, "IMU2" },
    { 0x0000, "enabled" },
    { 0x0001, "Gyr" },
    { 0x0000, "PRIMARY" },
    { 0x0000, "fullScale" },
    { 0x0001, "dlpf" },
    { 0x0001, "HIGH_G" },
    { 0x0000, "fullScale" },
    { 0x0001, "dlpf" },
    { 0x0002, "Acc" },
    { 0x0000, "PRIMARY" },
    { 0x0000, "fullScale" },
    { 0x0001, "dlpf" },
    { 0x0001, "HIGH_G" },
    { 0x0000, "fullScale" },
    { 0x0001, "dlpf" },
    { 0x0002, "IMU3" },
    { 0x0000, "enabled" },
    { 0x0001, "Gyr" },
    { 0x0000, "PRIMARY" },
    { 0x0000, "fullScale" },
    { 0x0001, "dlpf" },
    { 0x0001, "HIGH_G" },
    { 0x0000, "fullScale" },
    { 0x0001, "dlpf" },
    { 0x0002, "Acc" },
    { 0x0000, "PRIMARY" },
    { 0x0000, "fullScale" },
    { 0x0001, "dlpf" },
    { 0x0001, "HIGH_G" },
    { 0x0000, "fullScale" },
    { 0x0001, "dlpf" },
    { 0x0003, "IMU4" },
    { 0x0000, "enabled" },
    { 0x0001, "Gyr" },
    { 0x0000, "PRIMARY" },
    { 0x0000, "fullScale" },
    { 0x0001, "dlpf" },
    { 0x0001, "HIGH_G" },
    { 0x0000, "fullScale" },
    { 0x0001, "dlpf" },
    { 0x0002, "Acc" },
    { 0x0000, "PRIMARY" },
    { 0x0000, "fullScale" },
    { 0x0001, "dlpf" },
    { 0x0001, "HIGH_G" },
    { 0x0000, "fullScale" },
    { 0x0001, "dlpf" },
    { 0x0004, "IMU5" },
    { 0x0000, "enabled" },
    { 0x0001, "Gyr" },
    { 0x0000, "PRIMARY" },
    { 0x0000, "fullScale" },
    { 0x0001, "dlpf" },
    { 0x0001, "HIGH_G" },
    { 0x0000, "fullScale" },
    { 0x0001, "dlpf" },
    { 0x0002, "Acc" },
    { 0x0000, "PRIMARY" },
    { 0x0000, "fullScale" },
    { 0x0001, "dlpf" },
    { 0x0001, "HIGH_G" },
    { 0x0000, "fullScale" },
    { 0x0001, "dlpf" },
    { 0x0003, "Mags" },
    { 0x0000, "enabled" },
    { 0x0004, "Mag" },
    { 0x0000, "MAG1" },
    { 0x0000, "enabled" },
    { 0x0005, "Baros" },
    { 0x0000, "enabled" },
    { 0x0006, "Baro" },
    { 0x0000, "BARO1" },
    { 0x0000, "enabled" },
};
