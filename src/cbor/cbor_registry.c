/* Generated from src/cbor/registry.yaml by scripts/cbor_registry.py -- do not edit. */

#include <stddef.h>
#include "data_sets.h"
#include "ISComm.h"
#include "cbor/cbor_registry.h"

_Static_assert(DID_CBOR == 129, "registry.yaml transport.isb_did does not match DID_CBOR");
_Static_assert((NUM_CIDS) == 27, "registry.yaml constant NUM_CIDS does not match the C definition");
_Static_assert((NMEA_MSG_ID_COUNT) == 20, "registry.yaml constant NMEA_MSG_ID_COUNT does not match the C definition");
_Static_assert((RMC_CFG_MAX_PORTS) == 6, "registry.yaml constant RMC_CFG_MAX_PORTS does not match the C definition");
_Static_assert((ISB_MAX_DATASET_SIZE) == 2040, "registry.yaml constant ISB_MAX_DATASET_SIZE does not match the C definition");

const flash_cfg_field_desc_t g_cborFields_FLASH_CFG[] = {
    { 0, FCTYPE_UINT32, 1, offsetof(nvm_flash_cfg_t, startupImuDtMs) },
    { 1, FCTYPE_UINT32, 1, offsetof(nvm_flash_cfg_t, startupNavDtMs) },
    { 2, FCTYPE_UINT32, 1, offsetof(nvm_flash_cfg_t, ser0BaudRate) },
    { 3, FCTYPE_UINT32, 1, offsetof(nvm_flash_cfg_t, ser1BaudRate) },
    { 4, FCTYPE_FLOAT_ARRAY, 3, offsetof(nvm_flash_cfg_t, insRotation) },
    { 5, FCTYPE_FLOAT_ARRAY, 3, offsetof(nvm_flash_cfg_t, insOffset) },
    { 6, FCTYPE_FLOAT_ARRAY, 3, offsetof(nvm_flash_cfg_t, gnss1AntOffset) },
    { 7, FCTYPE_UINT8, 1, offsetof(nvm_flash_cfg_t, dynamicModel) },
    { 8, FCTYPE_UINT8, 1, offsetof(nvm_flash_cfg_t, debug) },
    { 9, FCTYPE_UINT16, 1, offsetof(nvm_flash_cfg_t, gnssSatSigConst) },
    { 10, FCTYPE_UINT32, 1, offsetof(nvm_flash_cfg_t, sysCfgBits) },
    { 11, FCTYPE_DOUBLE_ARRAY, 3, offsetof(nvm_flash_cfg_t, refLla) },
    { 12, FCTYPE_DOUBLE_ARRAY, 3, offsetof(nvm_flash_cfg_t, lastLla) },
    { 13, FCTYPE_UINT32, 1, offsetof(nvm_flash_cfg_t, lastLlaTimeOfWeekMs) },
    { 14, FCTYPE_UINT32, 1, offsetof(nvm_flash_cfg_t, lastLlaWeek) },
    { 15, FCTYPE_FLOAT, 1, offsetof(nvm_flash_cfg_t, lastLlaUpdateDistance) },
    { 16, FCTYPE_UINT32, 1, offsetof(nvm_flash_cfg_t, ioConfig) },
    { 17, FCTYPE_UINT32, 1, offsetof(nvm_flash_cfg_t, platformConfig) },
    { 18, FCTYPE_FLOAT_ARRAY, 3, offsetof(nvm_flash_cfg_t, gnss2AntOffset) },
    { 19, FCTYPE_FLOAT_ARRAY, 3, offsetof(nvm_flash_cfg_t, zeroVelRotation) },
    { 20, FCTYPE_FLOAT_ARRAY, 3, offsetof(nvm_flash_cfg_t, zeroVelOffset) },
    { 21, FCTYPE_FLOAT, 1, offsetof(nvm_flash_cfg_t, gnssTimeUserDelay) },
    { 22, FCTYPE_FLOAT, 1, offsetof(nvm_flash_cfg_t, magDeclination) },
    { 23, FCTYPE_UINT32, 1, offsetof(nvm_flash_cfg_t, gnssTimeSyncPeriodMs) },
    { 24, FCTYPE_UINT32, 1, offsetof(nvm_flash_cfg_t, startupGnssDtMs) },
    { 25, FCTYPE_UINT32, 1, offsetof(nvm_flash_cfg_t, RTKCfgBits) },
    { 26, FCTYPE_UINT32, 1, offsetof(nvm_flash_cfg_t, sensorConfig) },
    { 27, FCTYPE_FLOAT, 1, offsetof(nvm_flash_cfg_t, gnssMinimumElevation) },
    { 28, FCTYPE_UINT32, 1, offsetof(nvm_flash_cfg_t, ser2BaudRate) },
    { 29, FCTYPE_UINT32, 1, offsetof(nvm_flash_cfg_t, wheelConfig.bits) },
    { 30, FCTYPE_FLOAT_ARRAY, 3, offsetof(nvm_flash_cfg_t, wheelConfig.transform.e_b2w) },
    { 31, FCTYPE_FLOAT_ARRAY, 3, offsetof(nvm_flash_cfg_t, wheelConfig.transform.e_b2w_sigma) },
    { 32, FCTYPE_FLOAT_ARRAY, 3, offsetof(nvm_flash_cfg_t, wheelConfig.transform.t_b2w) },
    { 33, FCTYPE_FLOAT_ARRAY, 3, offsetof(nvm_flash_cfg_t, wheelConfig.transform.t_b2w_sigma) },
    { 34, FCTYPE_FLOAT, 1, offsetof(nvm_flash_cfg_t, wheelConfig.track_width) },
    { 35, FCTYPE_FLOAT, 1, offsetof(nvm_flash_cfg_t, wheelConfig.radius) },
    { 36, FCTYPE_FLOAT, 1, offsetof(nvm_flash_cfg_t, magInterferenceThreshold) },
    { 37, FCTYPE_FLOAT, 1, offsetof(nvm_flash_cfg_t, magCalibrationQualityThreshold) },
    { 38, FCTYPE_UINT8, 1, offsetof(nvm_flash_cfg_t, gnssCn0Minimum) },
    { 39, FCTYPE_UINT8, 1, offsetof(nvm_flash_cfg_t, gnssCn0DynMinOffset) },
    { 40, FCTYPE_UINT8, 1, offsetof(nvm_flash_cfg_t, imuRejectThreshGyroLow) },
    { 41, FCTYPE_UINT8, 1, offsetof(nvm_flash_cfg_t, imuRejectThreshGyroHigh) },
    { 42, FCTYPE_UINT8, 1, offsetof(nvm_flash_cfg_t, imuShockDetectLatencyMs) },
    { 43, FCTYPE_UINT8, 1, offsetof(nvm_flash_cfg_t, imuShockRejectLatchMs) },
    { 44, FCTYPE_UINT8, 1, offsetof(nvm_flash_cfg_t, imuShockOptions) },
    { 45, FCTYPE_UINT8, 1, offsetof(nvm_flash_cfg_t, imuShockDeltaAccHighThreshold) },
    { 46, FCTYPE_UINT8, 1, offsetof(nvm_flash_cfg_t, imuShockDeltaAccLowThreshold) },
    { 47, FCTYPE_UINT8, 1, offsetof(nvm_flash_cfg_t, imuShockDeltaGyroHighThreshold) },
    { 48, FCTYPE_UINT8, 1, offsetof(nvm_flash_cfg_t, imuShockDeltaGyroLowThreshold) },
    { 49, FCTYPE_UINT8, 1, offsetof(nvm_flash_cfg_t, ioConfig2) },
};
const uint16_t g_cborFieldsCount_FLASH_CFG = sizeof(g_cborFields_FLASH_CFG) / sizeof(g_cborFields_FLASH_CFG[0]);

/* FLASH_CFG: each entry matches its nvm_flash_cfg_t member in type and size */
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

const flash_cfg_field_desc_t g_cborFields_CAN_CONFIG[] = {
    { 0, FCTYPE_UINT16_ARRAY, NUM_CIDS, offsetof(can_config_t, can_period_mult) },
    { 1, FCTYPE_UINT32_ARRAY, NUM_CIDS, offsetof(can_config_t, can_transmit_address) },
    { 2, FCTYPE_UINT16, 1, offsetof(can_config_t, can_setting) },
    { 3, FCTYPE_UINT32, 1, offsetof(can_config_t, can_receive_address) },
};
const uint16_t g_cborFieldsCount_CAN_CONFIG = sizeof(g_cborFields_CAN_CONFIG) / sizeof(g_cborFields_CAN_CONFIG[0]);

/* CAN_CONFIG: each entry matches its can_config_t member in type and size */
_Static_assert(_Generic(((can_config_t *)0)->can_period_mult[0], uint16_t: 1, default: 0), "CAN_CONFIG.can_period_mult type");
_Static_assert(sizeof(((can_config_t *)0)->can_period_mult) == sizeof(uint16_t) * (NUM_CIDS), "CAN_CONFIG.can_period_mult count");
_Static_assert(_Generic(((can_config_t *)0)->can_transmit_address[0], uint32_t: 1, default: 0), "CAN_CONFIG.can_transmit_address type");
_Static_assert(sizeof(((can_config_t *)0)->can_transmit_address) == sizeof(uint32_t) * (NUM_CIDS), "CAN_CONFIG.can_transmit_address count");
_Static_assert(_Generic(((can_config_t *)0)->can_setting, uint16_t: 1, default: 0), "CAN_CONFIG.can_setting type");
_Static_assert(_Generic(((can_config_t *)0)->can_receive_address, uint32_t: 1, default: 0), "CAN_CONFIG.can_receive_address type");

const flash_cfg_field_desc_t g_cborFields_GPX_FLASH_CFG[] = {
    { 0, FCTYPE_UINT32, 1, offsetof(gpx_flash_cfg_t, ser0BaudRate) },
    { 1, FCTYPE_UINT32, 1, offsetof(gpx_flash_cfg_t, ser1BaudRate) },
    { 2, FCTYPE_UINT32, 1, offsetof(gpx_flash_cfg_t, ser2BaudRate) },
    { 3, FCTYPE_UINT32, 1, offsetof(gpx_flash_cfg_t, startupGnssDtMs) },
    { 4, FCTYPE_FLOAT_ARRAY, 3, offsetof(gpx_flash_cfg_t, gnss1AntOffset) },
    { 5, FCTYPE_FLOAT_ARRAY, 3, offsetof(gpx_flash_cfg_t, gnss2AntOffset) },
    { 6, FCTYPE_UINT16, 1, offsetof(gpx_flash_cfg_t, gnssSatSigConst) },
    { 7, FCTYPE_UINT8, 1, offsetof(gpx_flash_cfg_t, dynamicModel) },
    { 8, FCTYPE_UINT8, 1, offsetof(gpx_flash_cfg_t, debug) },
    { 9, FCTYPE_UINT32, 1, offsetof(gpx_flash_cfg_t, gnssTimeSyncPeriodMs) },
    { 10, FCTYPE_FLOAT, 1, offsetof(gpx_flash_cfg_t, gnssTimeUserDelay) },
    { 11, FCTYPE_FLOAT, 1, offsetof(gpx_flash_cfg_t, gnssMinimumElevation) },
    { 12, FCTYPE_UINT32, 1, offsetof(gpx_flash_cfg_t, RTKCfgBits) },
    { 13, FCTYPE_UINT8, 1, offsetof(gpx_flash_cfg_t, gnssCn0Minimum) },
    { 14, FCTYPE_UINT8, 1, offsetof(gpx_flash_cfg_t, gnssCn0DynMinOffset) },
    { 15, FCTYPE_UINT32, 1, offsetof(gpx_flash_cfg_t, sysCfgBits) },
    { 16, FCTYPE_DOUBLE_ARRAY, 3, offsetof(gpx_flash_cfg_t, refLla) },
};
const uint16_t g_cborFieldsCount_GPX_FLASH_CFG = sizeof(g_cborFields_GPX_FLASH_CFG) / sizeof(g_cborFields_GPX_FLASH_CFG[0]);

/* GPX_FLASH_CFG: each entry matches its gpx_flash_cfg_t member in type and size */
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
