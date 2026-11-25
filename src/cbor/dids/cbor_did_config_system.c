/**
 * @file cbor_did_config_system.c
 * @brief CONFIG_SYSTEM: field names.
 *
 * Generated from the CBOR key registry by is-common scripts/cbor_registry.py -- do not edit.
 *
 * @copyright Copyright (c) 2026 Inertial Sense, Inc.
 */

#include <stddef.h>
#include "cbor/cbor_registry.h"

static const cbor_field_t s_fields_config_system_platform[] = {
    { 0, CBOR_T_UINT, 1, 0, offsetof(cbor_config_system_platform_t, type), NULL },
    { 1, CBOR_T_UINT, 1, 0, offsetof(cbor_config_system_platform_t, preset), NULL },
    { 2, CBOR_T_UINT, 1, 0, offsetof(cbor_config_system_platform_t, ioExpanderBits), NULL },
};
static const cbor_schema_t s_schema_config_system_platform = { s_fields_config_system_platform, sizeof(s_fields_config_system_platform) / sizeof(s_fields_config_system_platform[0]) };

static const cbor_field_t s_fields_config_system[] = {
    { 0, CBOR_T_UINT, 4, 0, offsetof(cbor_config_system_t, navPeriodMs), NULL },
    { 1, CBOR_T_UINT, 4, 0, offsetof(cbor_config_system_t, imuOdr), NULL },
    { 2, CBOR_T_BOOL, 1, 0, offsetof(cbor_config_system_t, ledsEnabled), NULL },
    { 3, CBOR_T_UINT, 1, 0, offsetof(cbor_config_system_t, brownOutLevel), NULL },
    { 4, CBOR_T_BOOL, 1, 0, offsetof(cbor_config_system_t, bitOnStartup), NULL },
    { 5, CBOR_T_MAP, 0, 0, offsetof(cbor_config_system_t, platform), &s_schema_config_system_platform },
    { 6, CBOR_T_UINT, 1, 0, offsetof(cbor_config_system_t, debug), NULL },
};
const cbor_schema_t g_cborSchema_CONFIG_SYSTEM = { s_fields_config_system, sizeof(s_fields_config_system) / sizeof(s_fields_config_system[0]) };

const cbor_config_system_t g_cborDefaults_CONFIG_SYSTEM = {
    .imuOdr = 1,
    .ledsEnabled = 1,
    .brownOutLevel = 0,
    .bitOnStartup = 1,
    .platform.preset = 0,
    .platform.ioExpanderBits = 0,
};

const uint8_t g_cborDefaultFixed_CONFIG_SYSTEM[9] = {
    0, 1, 1, 1, 1, 0, 1, 1, 0,
};


const cbor_registry_name_t g_cborNames_CONFIG_SYSTEM[CBOR_CONFIG_SYSTEM_NAME_COUNT] = {
    { 0x0000, "navPeriodMs" },
    { 0x0001, "imuOdr" },
    { 0x0002, "ledsEnabled" },
    { 0x0003, "brownOutLevel" },
    { 0x0004, "bitOnStartup" },
    { 0x0005, "platform" },
    { 0x0000, "type" },
    { 0x0001, "preset" },
    { 0x0002, "ioExpanderBits" },
    { 0x0006, "debug" },
};
