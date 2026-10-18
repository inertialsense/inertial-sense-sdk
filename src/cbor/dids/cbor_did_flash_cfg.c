/**
 * @file cbor_did_flash_cfg.c
 * @brief FLASH_CFG: schema, recorded defaults and field names.
 *
 * Generated from the CBOR key registry by is-common scripts/cbor_registry.py -- do not edit.
 *
 * @copyright Copyright (c) 2026 Inertial Sense, Inc.
 */

#include <stddef.h>
#include "cbor/cbor_registry.h"

static const cbor_field_t s_fields[] = {
    { 0, CBOR_T_UINT, 4, 0, offsetof(nvm_flash_cfg_t, startupImuDtMs), NULL },
    { 1, CBOR_T_UINT, 4, 0, offsetof(nvm_flash_cfg_t, startupNavDtMs), NULL },
    { 2, CBOR_T_UINT, 4, 0, offsetof(nvm_flash_cfg_t, ser0BaudRate), NULL },
    { 3, CBOR_T_UINT, 4, 0, offsetof(nvm_flash_cfg_t, ser1BaudRate), NULL },
    { 4, CBOR_T_FLOAT, 4, 3, offsetof(nvm_flash_cfg_t, insRotation), NULL },
    { 5, CBOR_T_FLOAT, 4, 3, offsetof(nvm_flash_cfg_t, insOffset), NULL },
    { 6, CBOR_T_FLOAT, 4, 3, offsetof(nvm_flash_cfg_t, gnss1AntOffset), NULL },
    { 7, CBOR_T_UINT, 1, 0, offsetof(nvm_flash_cfg_t, dynamicModel), NULL },
    { 8, CBOR_T_UINT, 1, 0, offsetof(nvm_flash_cfg_t, debug), NULL },
    { 9, CBOR_T_UINT, 2, 0, offsetof(nvm_flash_cfg_t, gnssSatSigConst), NULL },
    { 10, CBOR_T_UINT, 4, 0, offsetof(nvm_flash_cfg_t, sysCfgBits), NULL },
    { 11, CBOR_T_FLOAT, 8, 3, offsetof(nvm_flash_cfg_t, refLla), NULL },
    { 12, CBOR_T_FLOAT, 8, 3, offsetof(nvm_flash_cfg_t, lastLla), NULL },
    { 13, CBOR_T_UINT, 4, 0, offsetof(nvm_flash_cfg_t, lastLlaTimeOfWeekMs), NULL },
    { 14, CBOR_T_UINT, 4, 0, offsetof(nvm_flash_cfg_t, lastLlaWeek), NULL },
    { 15, CBOR_T_FLOAT, 4, 0, offsetof(nvm_flash_cfg_t, lastLlaUpdateDistance), NULL },
    { 16, CBOR_T_UINT, 4, 0, offsetof(nvm_flash_cfg_t, ioConfig), NULL },
    { 17, CBOR_T_UINT, 4, 0, offsetof(nvm_flash_cfg_t, platformConfig), NULL },
    { 18, CBOR_T_FLOAT, 4, 3, offsetof(nvm_flash_cfg_t, gnss2AntOffset), NULL },
    { 19, CBOR_T_FLOAT, 4, 3, offsetof(nvm_flash_cfg_t, zeroVelRotation), NULL },
    { 20, CBOR_T_FLOAT, 4, 3, offsetof(nvm_flash_cfg_t, zeroVelOffset), NULL },
    { 21, CBOR_T_FLOAT, 4, 0, offsetof(nvm_flash_cfg_t, gnssTimeUserDelay), NULL },
    { 22, CBOR_T_FLOAT, 4, 0, offsetof(nvm_flash_cfg_t, magDeclination), NULL },
    { 23, CBOR_T_UINT, 4, 0, offsetof(nvm_flash_cfg_t, gnssTimeSyncPeriodMs), NULL },
    { 24, CBOR_T_UINT, 4, 0, offsetof(nvm_flash_cfg_t, startupGnssDtMs), NULL },
    { 25, CBOR_T_UINT, 4, 0, offsetof(nvm_flash_cfg_t, RTKCfgBits), NULL },
    { 26, CBOR_T_UINT, 4, 0, offsetof(nvm_flash_cfg_t, sensorConfig), NULL },
    { 27, CBOR_T_FLOAT, 4, 0, offsetof(nvm_flash_cfg_t, gnssMinimumElevation), NULL },
    { 28, CBOR_T_UINT, 4, 0, offsetof(nvm_flash_cfg_t, ser2BaudRate), NULL },
    { 29, CBOR_T_UINT, 4, 0, offsetof(nvm_flash_cfg_t, wheelConfig.bits), NULL },
    { 30, CBOR_T_FLOAT, 4, 3, offsetof(nvm_flash_cfg_t, wheelConfig.transform.e_b2w), NULL },
    { 31, CBOR_T_FLOAT, 4, 3, offsetof(nvm_flash_cfg_t, wheelConfig.transform.e_b2w_sigma), NULL },
    { 32, CBOR_T_FLOAT, 4, 3, offsetof(nvm_flash_cfg_t, wheelConfig.transform.t_b2w), NULL },
    { 33, CBOR_T_FLOAT, 4, 3, offsetof(nvm_flash_cfg_t, wheelConfig.transform.t_b2w_sigma), NULL },
    { 34, CBOR_T_FLOAT, 4, 0, offsetof(nvm_flash_cfg_t, wheelConfig.track_width), NULL },
    { 35, CBOR_T_FLOAT, 4, 0, offsetof(nvm_flash_cfg_t, wheelConfig.radius), NULL },
    { 36, CBOR_T_FLOAT, 4, 0, offsetof(nvm_flash_cfg_t, magInterferenceThreshold), NULL },
    { 37, CBOR_T_FLOAT, 4, 0, offsetof(nvm_flash_cfg_t, magCalibrationQualityThreshold), NULL },
    { 38, CBOR_T_UINT, 1, 0, offsetof(nvm_flash_cfg_t, gnssCn0Minimum), NULL },
    { 39, CBOR_T_UINT, 1, 0, offsetof(nvm_flash_cfg_t, gnssCn0DynMinOffset), NULL },
    { 40, CBOR_T_UINT, 1, 0, offsetof(nvm_flash_cfg_t, imuRejectThreshGyroLow), NULL },
    { 41, CBOR_T_UINT, 1, 0, offsetof(nvm_flash_cfg_t, imuRejectThreshGyroHigh), NULL },
    { 42, CBOR_T_UINT, 1, 0, offsetof(nvm_flash_cfg_t, imuShockDetectLatencyMs), NULL },
    { 43, CBOR_T_UINT, 1, 0, offsetof(nvm_flash_cfg_t, imuShockRejectLatchMs), NULL },
    { 44, CBOR_T_UINT, 1, 0, offsetof(nvm_flash_cfg_t, imuShockOptions), NULL },
    { 45, CBOR_T_UINT, 1, 0, offsetof(nvm_flash_cfg_t, imuShockDeltaAccHighThreshold), NULL },
    { 46, CBOR_T_UINT, 1, 0, offsetof(nvm_flash_cfg_t, imuShockDeltaAccLowThreshold), NULL },
    { 47, CBOR_T_UINT, 1, 0, offsetof(nvm_flash_cfg_t, imuShockDeltaGyroHighThreshold), NULL },
    { 48, CBOR_T_UINT, 1, 0, offsetof(nvm_flash_cfg_t, imuShockDeltaGyroLowThreshold), NULL },
    { 49, CBOR_T_UINT, 1, 0, offsetof(nvm_flash_cfg_t, ioConfig2), NULL },
};

const cbor_schema_t g_cborSchema_FLASH_CFG = { s_fields, sizeof(s_fields) / sizeof(s_fields[0]) };

/* Each entry matches its nvm_flash_cfg_t member in type and size */
_Static_assert(_Generic(((nvm_flash_cfg_t *)0)->startupImuDtMs, uint32_t: 1, default: 0), "FLASH_CFG.startupImuDtMs type");
_Static_assert(_Generic(((nvm_flash_cfg_t *)0)->startupNavDtMs, uint32_t: 1, default: 0), "FLASH_CFG.startupNavDtMs type");
_Static_assert(_Generic(((nvm_flash_cfg_t *)0)->ser0BaudRate, uint32_t: 1, default: 0), "FLASH_CFG.ser0BaudRate type");
_Static_assert(_Generic(((nvm_flash_cfg_t *)0)->ser1BaudRate, uint32_t: 1, default: 0), "FLASH_CFG.ser1BaudRate type");
_Static_assert(_Generic(((nvm_flash_cfg_t *)0)->insRotation[0], float: 1, default: 0), "FLASH_CFG.insRotation type");
_Static_assert(sizeof(((nvm_flash_cfg_t *)0)->insRotation) == sizeof(float) * (3), "FLASH_CFG.insRotation count");
_Static_assert(_Generic(((nvm_flash_cfg_t *)0)->insOffset[0], float: 1, default: 0), "FLASH_CFG.insOffset type");
_Static_assert(sizeof(((nvm_flash_cfg_t *)0)->insOffset) == sizeof(float) * (3), "FLASH_CFG.insOffset count");
_Static_assert(_Generic(((nvm_flash_cfg_t *)0)->gnss1AntOffset[0], float: 1, default: 0), "FLASH_CFG.gnss1AntOffset type");
_Static_assert(sizeof(((nvm_flash_cfg_t *)0)->gnss1AntOffset) == sizeof(float) * (3), "FLASH_CFG.gnss1AntOffset count");
_Static_assert(_Generic(((nvm_flash_cfg_t *)0)->dynamicModel, uint8_t: 1, default: 0), "FLASH_CFG.dynamicModel type");
_Static_assert(_Generic(((nvm_flash_cfg_t *)0)->debug, uint8_t: 1, default: 0), "FLASH_CFG.debug type");
_Static_assert(_Generic(((nvm_flash_cfg_t *)0)->gnssSatSigConst, uint16_t: 1, default: 0), "FLASH_CFG.gnssSatSigConst type");
_Static_assert(_Generic(((nvm_flash_cfg_t *)0)->sysCfgBits, uint32_t: 1, default: 0), "FLASH_CFG.sysCfgBits type");
_Static_assert(_Generic(((nvm_flash_cfg_t *)0)->refLla[0], double: 1, default: 0), "FLASH_CFG.refLla type");
_Static_assert(sizeof(((nvm_flash_cfg_t *)0)->refLla) == sizeof(double) * (3), "FLASH_CFG.refLla count");
_Static_assert(_Generic(((nvm_flash_cfg_t *)0)->lastLla[0], double: 1, default: 0), "FLASH_CFG.lastLla type");
_Static_assert(sizeof(((nvm_flash_cfg_t *)0)->lastLla) == sizeof(double) * (3), "FLASH_CFG.lastLla count");
_Static_assert(_Generic(((nvm_flash_cfg_t *)0)->lastLlaTimeOfWeekMs, uint32_t: 1, default: 0), "FLASH_CFG.lastLlaTimeOfWeekMs type");
_Static_assert(_Generic(((nvm_flash_cfg_t *)0)->lastLlaWeek, uint32_t: 1, default: 0), "FLASH_CFG.lastLlaWeek type");
_Static_assert(_Generic(((nvm_flash_cfg_t *)0)->lastLlaUpdateDistance, float: 1, default: 0), "FLASH_CFG.lastLlaUpdateDistance type");
_Static_assert(_Generic(((nvm_flash_cfg_t *)0)->ioConfig, uint32_t: 1, default: 0), "FLASH_CFG.ioConfig type");
_Static_assert(_Generic(((nvm_flash_cfg_t *)0)->platformConfig, uint32_t: 1, default: 0), "FLASH_CFG.platformConfig type");
_Static_assert(_Generic(((nvm_flash_cfg_t *)0)->gnss2AntOffset[0], float: 1, default: 0), "FLASH_CFG.gnss2AntOffset type");
_Static_assert(sizeof(((nvm_flash_cfg_t *)0)->gnss2AntOffset) == sizeof(float) * (3), "FLASH_CFG.gnss2AntOffset count");
_Static_assert(_Generic(((nvm_flash_cfg_t *)0)->zeroVelRotation[0], float: 1, default: 0), "FLASH_CFG.zeroVelRotation type");
_Static_assert(sizeof(((nvm_flash_cfg_t *)0)->zeroVelRotation) == sizeof(float) * (3), "FLASH_CFG.zeroVelRotation count");
_Static_assert(_Generic(((nvm_flash_cfg_t *)0)->zeroVelOffset[0], float: 1, default: 0), "FLASH_CFG.zeroVelOffset type");
_Static_assert(sizeof(((nvm_flash_cfg_t *)0)->zeroVelOffset) == sizeof(float) * (3), "FLASH_CFG.zeroVelOffset count");
_Static_assert(_Generic(((nvm_flash_cfg_t *)0)->gnssTimeUserDelay, float: 1, default: 0), "FLASH_CFG.gnssTimeUserDelay type");
_Static_assert(_Generic(((nvm_flash_cfg_t *)0)->magDeclination, float: 1, default: 0), "FLASH_CFG.magDeclination type");
_Static_assert(_Generic(((nvm_flash_cfg_t *)0)->gnssTimeSyncPeriodMs, uint32_t: 1, default: 0), "FLASH_CFG.gnssTimeSyncPeriodMs type");
_Static_assert(_Generic(((nvm_flash_cfg_t *)0)->startupGnssDtMs, uint32_t: 1, default: 0), "FLASH_CFG.startupGnssDtMs type");
_Static_assert(_Generic(((nvm_flash_cfg_t *)0)->RTKCfgBits, uint32_t: 1, default: 0), "FLASH_CFG.RTKCfgBits type");
_Static_assert(_Generic(((nvm_flash_cfg_t *)0)->sensorConfig, uint32_t: 1, default: 0), "FLASH_CFG.sensorConfig type");
_Static_assert(_Generic(((nvm_flash_cfg_t *)0)->gnssMinimumElevation, float: 1, default: 0), "FLASH_CFG.gnssMinimumElevation type");
_Static_assert(_Generic(((nvm_flash_cfg_t *)0)->ser2BaudRate, uint32_t: 1, default: 0), "FLASH_CFG.ser2BaudRate type");
_Static_assert(_Generic(((nvm_flash_cfg_t *)0)->wheelConfig.bits, uint32_t: 1, default: 0), "FLASH_CFG.wheelConfig.bits type");
_Static_assert(_Generic(((nvm_flash_cfg_t *)0)->wheelConfig.transform.e_b2w[0], float: 1, default: 0), "FLASH_CFG.wheelConfig.transform.e_b2w type");
_Static_assert(sizeof(((nvm_flash_cfg_t *)0)->wheelConfig.transform.e_b2w) == sizeof(float) * (3), "FLASH_CFG.wheelConfig.transform.e_b2w count");
_Static_assert(_Generic(((nvm_flash_cfg_t *)0)->wheelConfig.transform.e_b2w_sigma[0], float: 1, default: 0), "FLASH_CFG.wheelConfig.transform.e_b2w_sigma type");
_Static_assert(sizeof(((nvm_flash_cfg_t *)0)->wheelConfig.transform.e_b2w_sigma) == sizeof(float) * (3), "FLASH_CFG.wheelConfig.transform.e_b2w_sigma count");
_Static_assert(_Generic(((nvm_flash_cfg_t *)0)->wheelConfig.transform.t_b2w[0], float: 1, default: 0), "FLASH_CFG.wheelConfig.transform.t_b2w type");
_Static_assert(sizeof(((nvm_flash_cfg_t *)0)->wheelConfig.transform.t_b2w) == sizeof(float) * (3), "FLASH_CFG.wheelConfig.transform.t_b2w count");
_Static_assert(_Generic(((nvm_flash_cfg_t *)0)->wheelConfig.transform.t_b2w_sigma[0], float: 1, default: 0), "FLASH_CFG.wheelConfig.transform.t_b2w_sigma type");
_Static_assert(sizeof(((nvm_flash_cfg_t *)0)->wheelConfig.transform.t_b2w_sigma) == sizeof(float) * (3), "FLASH_CFG.wheelConfig.transform.t_b2w_sigma count");
_Static_assert(_Generic(((nvm_flash_cfg_t *)0)->wheelConfig.track_width, float: 1, default: 0), "FLASH_CFG.wheelConfig.track_width type");
_Static_assert(_Generic(((nvm_flash_cfg_t *)0)->wheelConfig.radius, float: 1, default: 0), "FLASH_CFG.wheelConfig.radius type");
_Static_assert(_Generic(((nvm_flash_cfg_t *)0)->magInterferenceThreshold, float: 1, default: 0), "FLASH_CFG.magInterferenceThreshold type");
_Static_assert(_Generic(((nvm_flash_cfg_t *)0)->magCalibrationQualityThreshold, float: 1, default: 0), "FLASH_CFG.magCalibrationQualityThreshold type");
_Static_assert(_Generic(((nvm_flash_cfg_t *)0)->gnssCn0Minimum, uint8_t: 1, default: 0), "FLASH_CFG.gnssCn0Minimum type");
_Static_assert(_Generic(((nvm_flash_cfg_t *)0)->gnssCn0DynMinOffset, uint8_t: 1, default: 0), "FLASH_CFG.gnssCn0DynMinOffset type");
_Static_assert(_Generic(((nvm_flash_cfg_t *)0)->imuRejectThreshGyroLow, uint8_t: 1, default: 0), "FLASH_CFG.imuRejectThreshGyroLow type");
_Static_assert(_Generic(((nvm_flash_cfg_t *)0)->imuRejectThreshGyroHigh, uint8_t: 1, default: 0), "FLASH_CFG.imuRejectThreshGyroHigh type");
_Static_assert(_Generic(((nvm_flash_cfg_t *)0)->imuShockDetectLatencyMs, uint8_t: 1, default: 0), "FLASH_CFG.imuShockDetectLatencyMs type");
_Static_assert(_Generic(((nvm_flash_cfg_t *)0)->imuShockRejectLatchMs, uint8_t: 1, default: 0), "FLASH_CFG.imuShockRejectLatchMs type");
_Static_assert(_Generic(((nvm_flash_cfg_t *)0)->imuShockOptions, uint8_t: 1, default: 0), "FLASH_CFG.imuShockOptions type");
_Static_assert(_Generic(((nvm_flash_cfg_t *)0)->imuShockDeltaAccHighThreshold, uint8_t: 1, default: 0), "FLASH_CFG.imuShockDeltaAccHighThreshold type");
_Static_assert(_Generic(((nvm_flash_cfg_t *)0)->imuShockDeltaAccLowThreshold, uint8_t: 1, default: 0), "FLASH_CFG.imuShockDeltaAccLowThreshold type");
_Static_assert(_Generic(((nvm_flash_cfg_t *)0)->imuShockDeltaGyroHighThreshold, uint8_t: 1, default: 0), "FLASH_CFG.imuShockDeltaGyroHighThreshold type");
_Static_assert(_Generic(((nvm_flash_cfg_t *)0)->imuShockDeltaGyroLowThreshold, uint8_t: 1, default: 0), "FLASH_CFG.imuShockDeltaGyroLowThreshold type");
_Static_assert(_Generic(((nvm_flash_cfg_t *)0)->ioConfig2, uint8_t: 1, default: 0), "FLASH_CFG.ioConfig2 type");

const nvm_flash_cfg_t g_cborDefaults_FLASH_CFG = {
    .startupImuDtMs = 1,
    .ser0BaudRate = 921600,
    .ser1BaudRate = 921600,
    .insRotation = { 0, 0, 0 },
    .insOffset = { 0, 0, 0 },
    .gnss1AntOffset = { 0, 0, 0 },
    .dynamicModel = 8,
    .debug = 0,
    .gnssSatSigConst = 5119,
    .sysCfgBits = 256,
    .lastLlaUpdateDistance = 1000,
    .gnss2AntOffset = { 0, 0, 0 },
    .zeroVelRotation = { 0, 0, 0 },
    .zeroVelOffset = { 0, 0, 0 },
    .gnssTimeUserDelay = 0,
    .magDeclination = 0,
    .gnssTimeSyncPeriodMs = 1000,
    .startupGnssDtMs = 200,
    .RTKCfgBits = 0,
    .sensorConfig = 119,
    .gnssMinimumElevation = 0.17453292f,
    .ser2BaudRate = 921600,
    .wheelConfig.bits = 0,
    .wheelConfig.transform.e_b2w = { 0, 0, 0 },
    .wheelConfig.transform.e_b2w_sigma = { 0.0174532924f, 0.0174532924f, 0.0174532924f },
    .wheelConfig.transform.t_b2w = { 0, 0, 0 },
    .wheelConfig.transform.t_b2w_sigma = { 0.300000012f, 0.300000012f, 0.300000012f },
    .wheelConfig.track_width = 0,
    .wheelConfig.radius = 0,
    .magInterferenceThreshold = 3,
    .magCalibrationQualityThreshold = 10,
    .gnssCn0Minimum = 30,
    .gnssCn0DynMinOffset = 20,
    .imuRejectThreshGyroLow = 2,
    .imuRejectThreshGyroHigh = 3,
    .imuShockDetectLatencyMs = 10,
    .imuShockRejectLatchMs = 5,
    .imuShockOptions = 2,
    .imuShockDeltaAccHighThreshold = 20,
    .imuShockDeltaAccLowThreshold = 3,
    .imuShockDeltaGyroHighThreshold = 120,
    .imuShockDeltaGyroLowThreshold = 12,
};

const uint8_t g_cborDefaultFixed_FLASH_CFG[] = {
    1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 1, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0,
};

const cbor_registry_name_t g_cborNames_FLASH_CFG[CBOR_FLASH_CFG_NAME_COUNT] = {
    { 0x0000, "startupImuDtMs" },
    { 0x0001, "startupNavDtMs" },
    { 0x0002, "ser0BaudRate" },
    { 0x0003, "ser1BaudRate" },
    { 0x0004, "insRotation" },
    { 0x0005, "insOffset" },
    { 0x0006, "gnss1AntOffset" },
    { 0x0007, "dynamicModel" },
    { 0x0008, "debug" },
    { 0x0009, "gnssSatSigConst" },
    { 0x000A, "sysCfgBits" },
    { 0x000B, "refLla" },
    { 0x000C, "lastLla" },
    { 0x000D, "lastLlaTimeOfWeekMs" },
    { 0x000E, "lastLlaWeek" },
    { 0x000F, "lastLlaUpdateDistance" },
    { 0x0010, "ioConfig" },
    { 0x0011, "platformConfig" },
    { 0x0012, "gnss2AntOffset" },
    { 0x0013, "zeroVelRotation" },
    { 0x0014, "zeroVelOffset" },
    { 0x0015, "gnssTimeUserDelay" },
    { 0x0016, "magDeclination" },
    { 0x0017, "gnssTimeSyncPeriodMs" },
    { 0x0018, "startupGnssDtMs" },
    { 0x0019, "RTKCfgBits" },
    { 0x001A, "sensorConfig" },
    { 0x001B, "gnssMinimumElevation" },
    { 0x001C, "ser2BaudRate" },
    { 0x001D, "wheelConfig.bits" },
    { 0x001E, "wheelConfig.transform.e_b2w" },
    { 0x001F, "wheelConfig.transform.e_b2w_sigma" },
    { 0x0020, "wheelConfig.transform.t_b2w" },
    { 0x0021, "wheelConfig.transform.t_b2w_sigma" },
    { 0x0022, "wheelConfig.track_width" },
    { 0x0023, "wheelConfig.radius" },
    { 0x0024, "magInterferenceThreshold" },
    { 0x0025, "magCalibrationQualityThreshold" },
    { 0x0026, "gnssCn0Minimum" },
    { 0x0027, "gnssCn0DynMinOffset" },
    { 0x0028, "imuRejectThreshGyroLow" },
    { 0x0029, "imuRejectThreshGyroHigh" },
    { 0x002A, "imuShockDetectLatencyMs" },
    { 0x002B, "imuShockRejectLatchMs" },
    { 0x002C, "imuShockOptions" },
    { 0x002D, "imuShockDeltaAccHighThreshold" },
    { 0x002E, "imuShockDeltaAccLowThreshold" },
    { 0x002F, "imuShockDeltaGyroHighThreshold" },
    { 0x0030, "imuShockDeltaGyroLowThreshold" },
    { 0x0031, "ioConfig2" },
};
