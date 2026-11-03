/**
 * @file cbor_did_gpx_flash_cfg.c
 * @brief GPX_FLASH_CFG: schema, recorded defaults and field names.
 *
 * Generated from the CBOR key registry by is-common scripts/cbor_registry.py -- do not edit.
 *
 * @copyright Copyright (c) 2026 Inertial Sense, Inc.
 */

#include <stddef.h>
#include "cbor/cbor_registry.h"

static const cbor_field_t s_fields[] = {
    { 0, CBOR_T_UINT, 4, 0, offsetof(gpx_flash_cfg_t, ser0BaudRate), NULL },
    { 1, CBOR_T_UINT, 4, 0, offsetof(gpx_flash_cfg_t, ser1BaudRate), NULL },
    { 2, CBOR_T_UINT, 4, 0, offsetof(gpx_flash_cfg_t, ser2BaudRate), NULL },
    { 3, CBOR_T_UINT, 4, 0, offsetof(gpx_flash_cfg_t, startupGnssDtMs), NULL },
    { 4, CBOR_T_FLOAT, 4, 3, offsetof(gpx_flash_cfg_t, gnss1AntOffset), NULL },
    { 5, CBOR_T_FLOAT, 4, 3, offsetof(gpx_flash_cfg_t, gnss2AntOffset), NULL },
    { 6, CBOR_T_UINT, 2, 0, offsetof(gpx_flash_cfg_t, gnssSatSigConst), NULL },
    { 7, CBOR_T_UINT, 1, 0, offsetof(gpx_flash_cfg_t, dynamicModel), NULL },
    { 8, CBOR_T_UINT, 1, 0, offsetof(gpx_flash_cfg_t, debug), NULL },
    { 9, CBOR_T_UINT, 4, 0, offsetof(gpx_flash_cfg_t, gnssTimeSyncPeriodMs), NULL },
    { 10, CBOR_T_FLOAT, 4, 0, offsetof(gpx_flash_cfg_t, gnssTimeUserDelay), NULL },
    { 11, CBOR_T_FLOAT, 4, 0, offsetof(gpx_flash_cfg_t, gnssMinimumElevation), NULL },
    { 12, CBOR_T_UINT, 4, 0, offsetof(gpx_flash_cfg_t, RTKCfgBits), NULL },
    { 13, CBOR_T_UINT, 1, 0, offsetof(gpx_flash_cfg_t, gnssCn0Minimum), NULL },
    { 14, CBOR_T_UINT, 1, 0, offsetof(gpx_flash_cfg_t, gnssCn0DynMinOffset), NULL },
    { 15, CBOR_T_UINT, 4, 0, offsetof(gpx_flash_cfg_t, sysCfgBits), NULL },
    { 16, CBOR_T_FLOAT, 8, 3, offsetof(gpx_flash_cfg_t, refLla), NULL },
};

const cbor_schema_t g_cborSchema_GPX_FLASH_CFG = { s_fields, sizeof(s_fields) / sizeof(s_fields[0]) };

/* Each entry matches its gpx_flash_cfg_t member in type and size */
_Static_assert(_Generic(((gpx_flash_cfg_t *)0)->ser0BaudRate, uint32_t: 1, default: 0), "GPX_FLASH_CFG.ser0BaudRate type");
_Static_assert(_Generic(((gpx_flash_cfg_t *)0)->ser1BaudRate, uint32_t: 1, default: 0), "GPX_FLASH_CFG.ser1BaudRate type");
_Static_assert(_Generic(((gpx_flash_cfg_t *)0)->ser2BaudRate, uint32_t: 1, default: 0), "GPX_FLASH_CFG.ser2BaudRate type");
_Static_assert(_Generic(((gpx_flash_cfg_t *)0)->startupGnssDtMs, uint32_t: 1, default: 0), "GPX_FLASH_CFG.startupGnssDtMs type");
_Static_assert(_Generic(((gpx_flash_cfg_t *)0)->gnss1AntOffset[0], float: 1, default: 0), "GPX_FLASH_CFG.gnss1AntOffset type");
_Static_assert(sizeof(((gpx_flash_cfg_t *)0)->gnss1AntOffset) == sizeof(float) * (3), "GPX_FLASH_CFG.gnss1AntOffset count");
_Static_assert(_Generic(((gpx_flash_cfg_t *)0)->gnss2AntOffset[0], float: 1, default: 0), "GPX_FLASH_CFG.gnss2AntOffset type");
_Static_assert(sizeof(((gpx_flash_cfg_t *)0)->gnss2AntOffset) == sizeof(float) * (3), "GPX_FLASH_CFG.gnss2AntOffset count");
_Static_assert(_Generic(((gpx_flash_cfg_t *)0)->gnssSatSigConst, uint16_t: 1, default: 0), "GPX_FLASH_CFG.gnssSatSigConst type");
_Static_assert(_Generic(((gpx_flash_cfg_t *)0)->dynamicModel, uint8_t: 1, default: 0), "GPX_FLASH_CFG.dynamicModel type");
_Static_assert(_Generic(((gpx_flash_cfg_t *)0)->debug, uint8_t: 1, default: 0), "GPX_FLASH_CFG.debug type");
_Static_assert(_Generic(((gpx_flash_cfg_t *)0)->gnssTimeSyncPeriodMs, uint32_t: 1, default: 0), "GPX_FLASH_CFG.gnssTimeSyncPeriodMs type");
_Static_assert(_Generic(((gpx_flash_cfg_t *)0)->gnssTimeUserDelay, float: 1, default: 0), "GPX_FLASH_CFG.gnssTimeUserDelay type");
_Static_assert(_Generic(((gpx_flash_cfg_t *)0)->gnssMinimumElevation, float: 1, default: 0), "GPX_FLASH_CFG.gnssMinimumElevation type");
_Static_assert(_Generic(((gpx_flash_cfg_t *)0)->RTKCfgBits, uint32_t: 1, default: 0), "GPX_FLASH_CFG.RTKCfgBits type");
_Static_assert(_Generic(((gpx_flash_cfg_t *)0)->gnssCn0Minimum, uint8_t: 1, default: 0), "GPX_FLASH_CFG.gnssCn0Minimum type");
_Static_assert(_Generic(((gpx_flash_cfg_t *)0)->gnssCn0DynMinOffset, uint8_t: 1, default: 0), "GPX_FLASH_CFG.gnssCn0DynMinOffset type");
_Static_assert(_Generic(((gpx_flash_cfg_t *)0)->sysCfgBits, uint32_t: 1, default: 0), "GPX_FLASH_CFG.sysCfgBits type");
_Static_assert(_Generic(((gpx_flash_cfg_t *)0)->refLla[0], double: 1, default: 0), "GPX_FLASH_CFG.refLla type");
_Static_assert(sizeof(((gpx_flash_cfg_t *)0)->refLla) == sizeof(double) * (3), "GPX_FLASH_CFG.refLla count");

const gpx_flash_cfg_t g_cborDefaults_GPX_FLASH_CFG = {
    .ser0BaudRate = 921600,
    .ser1BaudRate = 921600,
    .ser2BaudRate = 921600,
    .startupGnssDtMs = 200,
    .gnss1AntOffset = { 0, 0, 0 },
    .gnss2AntOffset = { 0, 0, 0 },
    .gnssSatSigConst = 5119,
    .dynamicModel = 8,
    .debug = 32,
    .gnssTimeSyncPeriodMs = 1000,
    .gnssTimeUserDelay = 0,
    .gnssMinimumElevation = 0.17453292f,
    .RTKCfgBits = 0,
    .gnssCn0Minimum = 30,
    .gnssCn0DynMinOffset = 20,
    .sysCfgBits = 0,
    .refLla = { 0, 0, 0 },
};

const uint8_t g_cborDefaultFixed_GPX_FLASH_CFG[] = {
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
};

const cbor_registry_name_t g_cborNames_GPX_FLASH_CFG[CBOR_GPX_FLASH_CFG_NAME_COUNT] = {
    { 0x0000, "ser0BaudRate" },
    { 0x0001, "ser1BaudRate" },
    { 0x0002, "ser2BaudRate" },
    { 0x0003, "startupGnssDtMs" },
    { 0x0004, "gnss1AntOffset" },
    { 0x0005, "gnss2AntOffset" },
    { 0x0006, "gnssSatSigConst" },
    { 0x0007, "dynamicModel" },
    { 0x0008, "debug" },
    { 0x0009, "gnssTimeSyncPeriodMs" },
    { 0x000A, "gnssTimeUserDelay" },
    { 0x000B, "gnssMinimumElevation" },
    { 0x000C, "RTKCfgBits" },
    { 0x000D, "gnssCn0Minimum" },
    { 0x000E, "gnssCn0DynMinOffset" },
    { 0x000F, "sysCfgBits" },
    { 0x0010, "refLla" },
};
