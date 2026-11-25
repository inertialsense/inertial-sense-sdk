/**
 * @file cbor_did_config_mag.c
 * @brief CONFIG_MAG: field names.
 *
 * Generated from the CBOR key registry by is-common scripts/cbor_registry.py -- do not edit.
 *
 * @copyright Copyright (c) 2026 Inertial Sense, Inc.
 */

#include <stddef.h>
#include "cbor/cbor_registry.h"

static const cbor_field_t s_fields_config_mag_declination[] = {
    { 0, CBOR_T_FLOAT, 4, 0, offsetof(cbor_config_mag_declination_t, valueRad), NULL },
    { 1, CBOR_T_BOOL, 1, 0, offsetof(cbor_config_mag_declination_t, estimate), NULL },
    { 2, CBOR_T_BOOL, 1, 0, offsetof(cbor_config_mag_declination_t, useWmm), NULL },
};
static const cbor_schema_t s_schema_config_mag_declination = { s_fields_config_mag_declination, sizeof(s_fields_config_mag_declination) / sizeof(s_fields_config_mag_declination[0]) };

static const cbor_field_t s_fields_config_mag[] = {
    { 0, CBOR_T_BOOL, 1, 0, offsetof(cbor_config_mag_t, continuousCal), NULL },
    { 1, CBOR_T_BOOL, 1, 0, offsetof(cbor_config_mag_t, autoRecal), NULL },
    { 2, CBOR_T_UINT, 1, 0, offsetof(cbor_config_mag_t, recalMode), NULL },
    { 3, CBOR_T_FLOAT, 4, 0, offsetof(cbor_config_mag_t, interferenceThreshold), NULL },
    { 4, CBOR_T_FLOAT, 4, 0, offsetof(cbor_config_mag_t, calQualityThreshold), NULL },
    { 5, CBOR_T_MAP, 0, 0, offsetof(cbor_config_mag_t, declination), &s_schema_config_mag_declination },
};
const cbor_schema_t g_cborSchema_CONFIG_MAG = { s_fields_config_mag, sizeof(s_fields_config_mag) / sizeof(s_fields_config_mag[0]) };

const cbor_config_mag_t g_cborDefaults_CONFIG_MAG = {
    .continuousCal = 0,
    .autoRecal = 0,
    .recalMode = 1,
    .interferenceThreshold = 3,
    .calQualityThreshold = 10,
    .declination.valueRad = 0,
    .declination.estimate = 1,
    .declination.useWmm = 0,
};

const uint8_t g_cborDefaultFixed_CONFIG_MAG[8] = {
    1, 1, 1, 1, 1, 1, 1, 1,
};


const cbor_registry_name_t g_cborNames_CONFIG_MAG[CBOR_CONFIG_MAG_NAME_COUNT] = {
    { 0x0000, "continuousCal" },
    { 0x0001, "autoRecal" },
    { 0x0002, "recalMode" },
    { 0x0003, "interferenceThreshold" },
    { 0x0004, "calQualityThreshold" },
    { 0x0005, "declination" },
    { 0x0000, "valueRad" },
    { 0x0001, "estimate" },
    { 0x0002, "useWmm" },
};
