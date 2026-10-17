/* Generated from src/cbor/registry.yaml by scripts/cbor_registry.py -- do not edit. */

#include "data_sets.h"
#include "cbor/cbor_registry.h"

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

const can_config_t g_cborDefaults_CAN_CONFIG = {
    .can_setting = 100,
    .can_receive_address = 0,
};
const uint8_t g_cborDefaultFixed_CAN_CONFIG[] = {
    1, 1, 1, 1,
};

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
