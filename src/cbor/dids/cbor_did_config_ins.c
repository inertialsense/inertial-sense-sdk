/**
 * @file cbor_did_config_ins.c
 * @brief CONFIG_INS: field names.
 *
 * Generated from the CBOR key registry by is-common scripts/cbor_registry.py -- do not edit.
 *
 * @copyright Copyright (c) 2026 Inertial Sense, Inc.
 */

#include <stddef.h>
#include "cbor/cbor_registry.h"

static const cbor_field_t s_fields_config_ins_fusion[] = {
    { 0, CBOR_T_BOOL, 1, 0, offsetof(cbor_config_ins_fusion_t, mag), NULL },
    { 1, CBOR_T_BOOL, 1, 0, offsetof(cbor_config_ins_fusion_t, baro), NULL },
    { 2, CBOR_T_BOOL, 1, 0, offsetof(cbor_config_ins_fusion_t, wheel), NULL },
};
static const cbor_schema_t s_schema_config_ins_fusion = { s_fields_config_ins_fusion, sizeof(s_fields_config_ins_fusion) / sizeof(s_fields_config_ins_fusion[0]) };

static const cbor_field_t s_fields_config_ins_zerovelocity[] = {
    { 0, CBOR_T_FLOAT, 4, 3, offsetof(cbor_config_ins_zerovelocity_t, rotation), NULL },
    { 1, CBOR_T_FLOAT, 4, 3, offsetof(cbor_config_ins_zerovelocity_t, offset), NULL },
    { 2, CBOR_T_BOOL, 1, 0, offsetof(cbor_config_ins_zerovelocity_t, autoZupt), NULL },
    { 3, CBOR_T_BOOL, 1, 0, offsetof(cbor_config_ins_zerovelocity_t, autoZaru), NULL },
};
static const cbor_schema_t s_schema_config_ins_zerovelocity = { s_fields_config_ins_zerovelocity, sizeof(s_fields_config_ins_zerovelocity) / sizeof(s_fields_config_ins_zerovelocity[0]) };

static const cbor_field_t s_fields_config_ins_shock[] = {
    { 0, CBOR_T_BOOL, 1, 0, offsetof(cbor_config_ins_shock_t, enabled), NULL },
    { 1, CBOR_T_BOOL, 1, 0, offsetof(cbor_config_ins_shock_t, fastRecovery), NULL },
    { 2, CBOR_T_UINT, 1, 0, offsetof(cbor_config_ins_shock_t, detectLatencyMs), NULL },
    { 3, CBOR_T_UINT, 1, 0, offsetof(cbor_config_ins_shock_t, rejectLatchMs), NULL },
    { 4, CBOR_T_UINT, 1, 0, offsetof(cbor_config_ins_shock_t, deltaAccHigh), NULL },
    { 5, CBOR_T_UINT, 1, 0, offsetof(cbor_config_ins_shock_t, deltaAccLow), NULL },
    { 6, CBOR_T_UINT, 1, 0, offsetof(cbor_config_ins_shock_t, deltaGyroHigh), NULL },
    { 7, CBOR_T_UINT, 1, 0, offsetof(cbor_config_ins_shock_t, deltaGyroLow), NULL },
};
static const cbor_schema_t s_schema_config_ins_shock = { s_fields_config_ins_shock, sizeof(s_fields_config_ins_shock) / sizeof(s_fields_config_ins_shock[0]) };

static const cbor_field_t s_fields_config_ins_faultdetect[] = {
    { 0, CBOR_T_BOOL, 1, 0, offsetof(cbor_config_ins_faultdetect_t, gyr), NULL },
    { 1, CBOR_T_BOOL, 1, 0, offsetof(cbor_config_ins_faultdetect_t, acc), NULL },
    { 2, CBOR_T_UINT, 1, 0, offsetof(cbor_config_ins_faultdetect_t, gyroRejectThreshLow), NULL },
    { 3, CBOR_T_UINT, 1, 0, offsetof(cbor_config_ins_faultdetect_t, gyroRejectThreshHigh), NULL },
};
static const cbor_schema_t s_schema_config_ins_faultdetect = { s_fields_config_ins_faultdetect, sizeof(s_fields_config_ins_faultdetect) / sizeof(s_fields_config_ins_faultdetect[0]) };

static const cbor_field_t s_fields_config_ins_complementaryfilter[] = {
    { 0, CBOR_T_FLOAT, 4, 0, offsetof(cbor_config_ins_complementaryfilter_t, bKpqr), NULL },
    { 1, CBOR_T_FLOAT, 4, 0, offsetof(cbor_config_ins_complementaryfilter_t, bKuvw), NULL },
    { 2, CBOR_T_FLOAT, 4, 0, offsetof(cbor_config_ins_complementaryfilter_t, oKat1), NULL },
    { 3, CBOR_T_FLOAT, 4, 0, offsetof(cbor_config_ins_complementaryfilter_t, oKat2), NULL },
    { 4, CBOR_T_FLOAT, 4, 0, offsetof(cbor_config_ins_complementaryfilter_t, oKuvw), NULL },
    { 5, CBOR_T_FLOAT, 4, 0, offsetof(cbor_config_ins_complementaryfilter_t, oKlla), NULL },
};
static const cbor_schema_t s_schema_config_ins_complementaryfilter = { s_fields_config_ins_complementaryfilter, sizeof(s_fields_config_ins_complementaryfilter) / sizeof(s_fields_config_ins_complementaryfilter[0]) };

static const cbor_field_t s_fields_config_ins[] = {
    { 0, CBOR_T_FLOAT, 4, 3, offsetof(cbor_config_ins_t, rotation), NULL },
    { 1, CBOR_T_FLOAT, 4, 3, offsetof(cbor_config_ins_t, offset), NULL },
    { 2, CBOR_T_FLOAT, 8, 3, offsetof(cbor_config_ins_t, nedOrigin), NULL },
    { 3, CBOR_T_UINT, 1, 0, offsetof(cbor_config_ins_t, dynamicModel), NULL },
    { 4, CBOR_T_BOOL, 1, 0, offsetof(cbor_config_ins_t, ekfEnabled), NULL },
    { 5, CBOR_T_UINT, 1, 0, offsetof(cbor_config_ins_t, deadReckoningTimeoutS), NULL },
    { 6, CBOR_T_FLOAT, 4, 0, offsetof(cbor_config_ins_t, lastPositionUpdateDistanceM), NULL },
    { 7, CBOR_T_BOOL, 1, 0, offsetof(cbor_config_ins_t, referenceImu), NULL },
    { 8, CBOR_T_BOOL, 1, 0, offsetof(cbor_config_ins_t, stationaryOnStrobe), NULL },
    { 9, CBOR_T_MAP, 0, 0, offsetof(cbor_config_ins_t, fusion), &s_schema_config_ins_fusion },
    { 10, CBOR_T_MAP, 0, 0, offsetof(cbor_config_ins_t, zeroVelocity), &s_schema_config_ins_zerovelocity },
    { 11, CBOR_T_MAP, 0, 0, offsetof(cbor_config_ins_t, shock), &s_schema_config_ins_shock },
    { 12, CBOR_T_MAP, 0, 0, offsetof(cbor_config_ins_t, faultDetect), &s_schema_config_ins_faultdetect },
    { 13, CBOR_T_MAP, 0, 0, offsetof(cbor_config_ins_t, complementaryFilter), &s_schema_config_ins_complementaryfilter },
};
const cbor_schema_t g_cborSchema_CONFIG_INS = { s_fields_config_ins, sizeof(s_fields_config_ins) / sizeof(s_fields_config_ins[0]) };

const cbor_config_ins_t g_cborDefaults_CONFIG_INS = {
    .rotation = { 0, 0, 0 },
    .offset = { 0, 0, 0 },
    .dynamicModel = 8,
    .ekfEnabled = 1,
    .deadReckoningTimeoutS = 30,
    .lastPositionUpdateDistanceM = 1000,
    .referenceImu = 0,
    .stationaryOnStrobe = 0,
    .fusion.mag = 1,
    .fusion.baro = 1,
    .fusion.wheel = 1,
    .zeroVelocity.rotation = { 0, 0, 0 },
    .zeroVelocity.offset = { 0, 0, 0 },
    .zeroVelocity.autoZupt = 1,
    .zeroVelocity.autoZaru = 1,
    .shock.enabled = 0,
    .shock.fastRecovery = 1,
    .shock.detectLatencyMs = 10,
    .shock.rejectLatchMs = 5,
    .shock.deltaAccHigh = 20,
    .shock.deltaAccLow = 3,
    .shock.deltaGyroHigh = 120,
    .shock.deltaGyroLow = 12,
    .faultDetect.gyr = 0,
    .faultDetect.acc = 0,
    .faultDetect.gyroRejectThreshLow = 2,
    .faultDetect.gyroRejectThreshHigh = 3,
};

const uint8_t g_cborDefaultFixed_CONFIG_INS[34] = {
    1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0,
};


const cbor_registry_name_t g_cborNames_CONFIG_INS[CBOR_CONFIG_INS_NAME_COUNT] = {
    { 0x0000, "rotation" },
    { 0x0001, "offset" },
    { 0x0002, "nedOrigin" },
    { 0x0003, "dynamicModel" },
    { 0x0004, "ekfEnabled" },
    { 0x0005, "deadReckoningTimeoutS" },
    { 0x0006, "lastPositionUpdateDistanceM" },
    { 0x0007, "referenceImu" },
    { 0x0008, "stationaryOnStrobe" },
    { 0x0009, "fusion" },
    { 0x0000, "mag" },
    { 0x0001, "baro" },
    { 0x0002, "wheel" },
    { 0x000A, "zeroVelocity" },
    { 0x0000, "rotation" },
    { 0x0001, "offset" },
    { 0x0002, "autoZupt" },
    { 0x0003, "autoZaru" },
    { 0x000B, "shock" },
    { 0x0000, "enabled" },
    { 0x0001, "fastRecovery" },
    { 0x0002, "detectLatencyMs" },
    { 0x0003, "rejectLatchMs" },
    { 0x0004, "deltaAccHigh" },
    { 0x0005, "deltaAccLow" },
    { 0x0006, "deltaGyroHigh" },
    { 0x0007, "deltaGyroLow" },
    { 0x000C, "faultDetect" },
    { 0x0000, "gyr" },
    { 0x0001, "acc" },
    { 0x0002, "gyroRejectThreshLow" },
    { 0x0003, "gyroRejectThreshHigh" },
    { 0x000D, "complementaryFilter" },
    { 0x0000, "bKpqr" },
    { 0x0001, "bKuvw" },
    { 0x0002, "oKat1" },
    { 0x0003, "oKat2" },
    { 0x0004, "oKuvw" },
    { 0x0005, "oKlla" },
};
