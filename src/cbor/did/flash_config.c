/**
 * @file flash_config.c
 * @brief Converters between nvm_flash_cfg_t and the domains that hold its values, and compile-time checks of every
 *        member, mask and index the registry names in it.
 *
 * Generated from the CBOR key registry by is-common scripts/cbor_registry.py -- do not edit.
 *
 * @copyright Copyright (c) 2026 Inertial Sense, Inc.
 */

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "cbor/did/flash_config.h"

/** Lowest set bit of a mask: the unit of the value it holds */
#define CBOR_BIND_LSB(m)    ((uint32_t)(m) & (0u - (uint32_t)(m)))

/**
 * Fills the CONFIG_SYSTEM keys held in nvm_flash_cfg_t into the view.
 *
 * Keys of CONFIG_SYSTEM that live elsewhere are left as they are, so the view of a domain whose keys live in
 * several structs is filled by each of their mappings in turn.
 *
 * @param view  [out] the view
 * @param src   the nvm_flash_cfg_t the keys are read from
 */
void cbor_flash_config_to_config_system(cbor_config_system_t *view, const nvm_flash_cfg_t *src)
{
    view->navPeriodMs = src->startupNavDtMs;
    view->imuOdr = src->startupImuOdr;
    view->ledsEnabled = ((src->sysCfgBits & ((uint32_t)(SYS_CFG_BITS_DISABLE_LEDS))) == 0u);
    view->brownOutLevel = (uint8_t)(((uint32_t)src->sysCfgBits & ((uint32_t)(SYS_CFG_BITS_BOR_THRESHOLD_MASK))) / CBOR_BIND_LSB(((uint32_t)(SYS_CFG_BITS_BOR_THRESHOLD_MASK))));
    view->bitOnStartup = ((src->sysCfgBits & ((uint32_t)(SYS_CFG_BITS_DISABLE_AUTO_BIT_ON_STARTUP))) == 0u);
    view->platform.type = (uint8_t)(((uint32_t)src->platformConfig & ((uint32_t)(PLATFORM_CFG_TYPE_MASK))) / CBOR_BIND_LSB(((uint32_t)(PLATFORM_CFG_TYPE_MASK))));
    view->platform.preset = (uint8_t)(((uint32_t)src->platformConfig & ((uint32_t)(PLATFORM_CFG_PRESET_MASK))) / CBOR_BIND_LSB(((uint32_t)(PLATFORM_CFG_PRESET_MASK))));
    view->platform.ioExpanderBits = (uint8_t)(((uint32_t)src->platformConfig & ((uint32_t)(PLATFORM_CFG_RUG_IOEXP_BIT_MASK))) / CBOR_BIND_LSB(((uint32_t)(PLATFORM_CFG_RUG_IOEXP_BIT_MASK))));
    view->debug = src->debug;
}

/**
 * Checks that every CONFIG_SYSTEM value held in nvm_flash_cfg_t fits its home (a packed field or a bool).
 *
 * A domain whose keys live in several structs is written all or nothing by checking each of their mappings
 * before writing any of them.
 *
 * @param view  the view
 * @return true if cbor_flash_config_from_config_system() would write the view
 */
bool cbor_flash_config_fits_config_system(const cbor_config_system_t *view)
{
    if (view->ledsEnabled > 1u) return false;
    if (((uint64_t)view->brownOutLevel * CBOR_BIND_LSB(((uint32_t)(SYS_CFG_BITS_BOR_THRESHOLD_MASK)))) & ~(uint64_t)((uint32_t)(SYS_CFG_BITS_BOR_THRESHOLD_MASK))) return false;
    if (view->bitOnStartup > 1u) return false;
    if (((uint64_t)view->platform.type * CBOR_BIND_LSB(((uint32_t)(PLATFORM_CFG_TYPE_MASK)))) & ~(uint64_t)((uint32_t)(PLATFORM_CFG_TYPE_MASK))) return false;
    if (((uint64_t)view->platform.preset * CBOR_BIND_LSB(((uint32_t)(PLATFORM_CFG_PRESET_MASK)))) & ~(uint64_t)((uint32_t)(PLATFORM_CFG_PRESET_MASK))) return false;
    if (((uint64_t)view->platform.ioExpanderBits * CBOR_BIND_LSB(((uint32_t)(PLATFORM_CFG_RUG_IOEXP_BIT_MASK)))) & ~(uint64_t)((uint32_t)(PLATFORM_CFG_RUG_IOEXP_BIT_MASK))) return false;
    return true;
}

/**
 * Writes the CONFIG_SYSTEM keys held in nvm_flash_cfg_t from the view, all or nothing.
 *
 * Bits and members of nvm_flash_cfg_t that no key covers are kept. Nothing is written unless every value fits its
 * home (a packed field or a bool).
 *
 * @param dst   the nvm_flash_cfg_t the keys are written to
 * @param view  the view
 * @return false, writing nothing, if a value does not fit its home
 */
bool cbor_flash_config_from_config_system(nvm_flash_cfg_t *dst, const cbor_config_system_t *view)
{
    if (view->ledsEnabled > 1u) return false;
    if (((uint64_t)view->brownOutLevel * CBOR_BIND_LSB(((uint32_t)(SYS_CFG_BITS_BOR_THRESHOLD_MASK)))) & ~(uint64_t)((uint32_t)(SYS_CFG_BITS_BOR_THRESHOLD_MASK))) return false;
    if (view->bitOnStartup > 1u) return false;
    if (((uint64_t)view->platform.type * CBOR_BIND_LSB(((uint32_t)(PLATFORM_CFG_TYPE_MASK)))) & ~(uint64_t)((uint32_t)(PLATFORM_CFG_TYPE_MASK))) return false;
    if (((uint64_t)view->platform.preset * CBOR_BIND_LSB(((uint32_t)(PLATFORM_CFG_PRESET_MASK)))) & ~(uint64_t)((uint32_t)(PLATFORM_CFG_PRESET_MASK))) return false;
    if (((uint64_t)view->platform.ioExpanderBits * CBOR_BIND_LSB(((uint32_t)(PLATFORM_CFG_RUG_IOEXP_BIT_MASK)))) & ~(uint64_t)((uint32_t)(PLATFORM_CFG_RUG_IOEXP_BIT_MASK))) return false;
    dst->startupNavDtMs = view->navPeriodMs;
    dst->startupImuOdr = view->imuOdr;
    if (view->ledsEnabled) dst->sysCfgBits &= ~((uint32_t)(SYS_CFG_BITS_DISABLE_LEDS)); else dst->sysCfgBits |= ((uint32_t)(SYS_CFG_BITS_DISABLE_LEDS));
    dst->sysCfgBits = (dst->sysCfgBits & ~((uint32_t)(SYS_CFG_BITS_BOR_THRESHOLD_MASK))) | (((uint32_t)view->brownOutLevel * CBOR_BIND_LSB(((uint32_t)(SYS_CFG_BITS_BOR_THRESHOLD_MASK)))) & ((uint32_t)(SYS_CFG_BITS_BOR_THRESHOLD_MASK)));
    if (view->bitOnStartup) dst->sysCfgBits &= ~((uint32_t)(SYS_CFG_BITS_DISABLE_AUTO_BIT_ON_STARTUP)); else dst->sysCfgBits |= ((uint32_t)(SYS_CFG_BITS_DISABLE_AUTO_BIT_ON_STARTUP));
    dst->platformConfig = (dst->platformConfig & ~((uint32_t)(PLATFORM_CFG_TYPE_MASK))) | (((uint32_t)view->platform.type * CBOR_BIND_LSB(((uint32_t)(PLATFORM_CFG_TYPE_MASK)))) & ((uint32_t)(PLATFORM_CFG_TYPE_MASK)));
    dst->platformConfig = (dst->platformConfig & ~((uint32_t)(PLATFORM_CFG_PRESET_MASK))) | (((uint32_t)view->platform.preset * CBOR_BIND_LSB(((uint32_t)(PLATFORM_CFG_PRESET_MASK)))) & ((uint32_t)(PLATFORM_CFG_PRESET_MASK)));
    dst->platformConfig = (dst->platformConfig & ~((uint32_t)(PLATFORM_CFG_RUG_IOEXP_BIT_MASK))) | (((uint32_t)view->platform.ioExpanderBits * CBOR_BIND_LSB(((uint32_t)(PLATFORM_CFG_RUG_IOEXP_BIT_MASK)))) & ((uint32_t)(PLATFORM_CFG_RUG_IOEXP_BIT_MASK)));
    dst->debug = view->debug;
    return true;
}

/**
 * Fills the CONFIG_INS keys held in nvm_flash_cfg_t into the view.
 *
 * Keys of CONFIG_INS that live elsewhere are left as they are, so the view of a domain whose keys live in
 * several structs is filled by each of their mappings in turn.
 *
 * @param view  [out] the view
 * @param src   the nvm_flash_cfg_t the keys are read from
 */
void cbor_flash_config_to_config_ins(cbor_config_ins_t *view, const nvm_flash_cfg_t *src)
{
    memcpy(view->rotation, src->insRotation, sizeof(view->rotation));
    memcpy(view->offset, src->insOffset, sizeof(view->offset));
    memcpy(view->nedOrigin, src->refLla, sizeof(view->nedOrigin));
    view->dynamicModel = src->dynamicModel;
    view->ekfEnabled = ((src->sysCfgBits & ((uint32_t)(SYS_CFG_BITS_DISABLE_INS_EKF))) == 0u);
    view->deadReckoningTimeoutS = src->deadReckoningTimeout;
    view->lastPositionUpdateDistanceM = src->lastLlaUpdateDistance;
    view->referenceImu = ((src->sysCfgBits & ((uint32_t)(SYS_CFG_USE_REFERENCE_IMU_IN_EKF))) != 0u);
    view->stationaryOnStrobe = ((src->sysCfgBits & ((uint32_t)(SYS_CFG_EKF_REF_POINT_STATIONARY_ON_STROBE_INPUT))) != 0u);
    view->fusion.mag = ((src->sysCfgBits & ((uint32_t)(SYS_CFG_BITS_DISABLE_MAGNETOMETER_FUSION))) == 0u);
    view->fusion.baro = ((src->sysCfgBits & ((uint32_t)(SYS_CFG_BITS_DISABLE_BAROMETER_FUSION))) == 0u);
    view->fusion.wheel = ((src->sysCfgBits & ((uint32_t)(SYS_CFG_BITS_DISABLE_WHEEL_ENCODER_FUSION))) == 0u);
    memcpy(view->zeroVelocity.rotation, src->zeroVelRotation, sizeof(view->zeroVelocity.rotation));
    memcpy(view->zeroVelocity.offset, src->zeroVelOffset, sizeof(view->zeroVelocity.offset));
    view->zeroVelocity.autoZupt = ((src->sysCfgBits & ((uint32_t)(SYS_CFG_BITS_DISABLE_AUTO_ZERO_VELOCITY_UPDATES))) == 0u);
    view->zeroVelocity.autoZaru = ((src->sysCfgBits & ((uint32_t)(SYS_CFG_BITS_DISABLE_AUTO_ZERO_ANGULAR_RATE_UPDATES))) == 0u);
    view->shock.enabled = ((src->imuShockOptions & ((uint32_t)(IMU_SHOCK_OPTIONS_ENABLE))) != 0u);
    view->shock.fastRecovery = ((src->imuShockOptions & ((uint32_t)(IMU_SHOCK_OPTIONS_FAST_RECOVERY))) != 0u);
    view->shock.detectLatencyMs = src->imuShockDetectLatencyMs;
    view->shock.rejectLatchMs = src->imuShockRejectLatchMs;
    view->shock.deltaAccHigh = src->imuShockDeltaAccHighThreshold;
    view->shock.deltaAccLow = src->imuShockDeltaAccLowThreshold;
    view->shock.deltaGyroHigh = src->imuShockDeltaGyroHighThreshold;
    view->shock.deltaGyroLow = src->imuShockDeltaGyroLowThreshold;
    view->faultDetect.gyr = ((src->sensorConfig & ((uint32_t)(SENSOR_CFG_IMU_FAULT_DETECT_GYR))) != 0u);
    view->faultDetect.acc = ((src->sensorConfig & ((uint32_t)(SENSOR_CFG_IMU_FAULT_DETECT_ACC))) != 0u);
    view->faultDetect.gyroRejectThreshLow = src->imuRejectThreshGyroLow;
    view->faultDetect.gyroRejectThreshHigh = src->imuRejectThreshGyroHigh;
}

/**
 * Checks that every CONFIG_INS value held in nvm_flash_cfg_t fits its home (a packed field or a bool).
 *
 * A domain whose keys live in several structs is written all or nothing by checking each of their mappings
 * before writing any of them.
 *
 * @param view  the view
 * @return true if cbor_flash_config_from_config_ins() would write the view
 */
bool cbor_flash_config_fits_config_ins(const cbor_config_ins_t *view)
{
    if (view->ekfEnabled > 1u) return false;
    if (view->referenceImu > 1u) return false;
    if (view->stationaryOnStrobe > 1u) return false;
    if (view->fusion.mag > 1u) return false;
    if (view->fusion.baro > 1u) return false;
    if (view->fusion.wheel > 1u) return false;
    if (view->zeroVelocity.autoZupt > 1u) return false;
    if (view->zeroVelocity.autoZaru > 1u) return false;
    if (view->shock.enabled > 1u) return false;
    if (view->shock.fastRecovery > 1u) return false;
    if (view->faultDetect.gyr > 1u) return false;
    if (view->faultDetect.acc > 1u) return false;
    return true;
}

/**
 * Writes the CONFIG_INS keys held in nvm_flash_cfg_t from the view, all or nothing.
 *
 * Bits and members of nvm_flash_cfg_t that no key covers are kept. Nothing is written unless every value fits its
 * home (a packed field or a bool).
 *
 * @param dst   the nvm_flash_cfg_t the keys are written to
 * @param view  the view
 * @return false, writing nothing, if a value does not fit its home
 */
bool cbor_flash_config_from_config_ins(nvm_flash_cfg_t *dst, const cbor_config_ins_t *view)
{
    if (view->ekfEnabled > 1u) return false;
    if (view->referenceImu > 1u) return false;
    if (view->stationaryOnStrobe > 1u) return false;
    if (view->fusion.mag > 1u) return false;
    if (view->fusion.baro > 1u) return false;
    if (view->fusion.wheel > 1u) return false;
    if (view->zeroVelocity.autoZupt > 1u) return false;
    if (view->zeroVelocity.autoZaru > 1u) return false;
    if (view->shock.enabled > 1u) return false;
    if (view->shock.fastRecovery > 1u) return false;
    if (view->faultDetect.gyr > 1u) return false;
    if (view->faultDetect.acc > 1u) return false;
    memcpy(dst->insRotation, view->rotation, sizeof(view->rotation));
    memcpy(dst->insOffset, view->offset, sizeof(view->offset));
    memcpy(dst->refLla, view->nedOrigin, sizeof(view->nedOrigin));
    dst->dynamicModel = view->dynamicModel;
    if (view->ekfEnabled) dst->sysCfgBits &= ~((uint32_t)(SYS_CFG_BITS_DISABLE_INS_EKF)); else dst->sysCfgBits |= ((uint32_t)(SYS_CFG_BITS_DISABLE_INS_EKF));
    dst->deadReckoningTimeout = view->deadReckoningTimeoutS;
    dst->lastLlaUpdateDistance = view->lastPositionUpdateDistanceM;
    if (view->referenceImu) dst->sysCfgBits |= ((uint32_t)(SYS_CFG_USE_REFERENCE_IMU_IN_EKF)); else dst->sysCfgBits &= ~((uint32_t)(SYS_CFG_USE_REFERENCE_IMU_IN_EKF));
    if (view->stationaryOnStrobe) dst->sysCfgBits |= ((uint32_t)(SYS_CFG_EKF_REF_POINT_STATIONARY_ON_STROBE_INPUT)); else dst->sysCfgBits &= ~((uint32_t)(SYS_CFG_EKF_REF_POINT_STATIONARY_ON_STROBE_INPUT));
    if (view->fusion.mag) dst->sysCfgBits &= ~((uint32_t)(SYS_CFG_BITS_DISABLE_MAGNETOMETER_FUSION)); else dst->sysCfgBits |= ((uint32_t)(SYS_CFG_BITS_DISABLE_MAGNETOMETER_FUSION));
    if (view->fusion.baro) dst->sysCfgBits &= ~((uint32_t)(SYS_CFG_BITS_DISABLE_BAROMETER_FUSION)); else dst->sysCfgBits |= ((uint32_t)(SYS_CFG_BITS_DISABLE_BAROMETER_FUSION));
    if (view->fusion.wheel) dst->sysCfgBits &= ~((uint32_t)(SYS_CFG_BITS_DISABLE_WHEEL_ENCODER_FUSION)); else dst->sysCfgBits |= ((uint32_t)(SYS_CFG_BITS_DISABLE_WHEEL_ENCODER_FUSION));
    memcpy(dst->zeroVelRotation, view->zeroVelocity.rotation, sizeof(view->zeroVelocity.rotation));
    memcpy(dst->zeroVelOffset, view->zeroVelocity.offset, sizeof(view->zeroVelocity.offset));
    if (view->zeroVelocity.autoZupt) dst->sysCfgBits &= ~((uint32_t)(SYS_CFG_BITS_DISABLE_AUTO_ZERO_VELOCITY_UPDATES)); else dst->sysCfgBits |= ((uint32_t)(SYS_CFG_BITS_DISABLE_AUTO_ZERO_VELOCITY_UPDATES));
    if (view->zeroVelocity.autoZaru) dst->sysCfgBits &= ~((uint32_t)(SYS_CFG_BITS_DISABLE_AUTO_ZERO_ANGULAR_RATE_UPDATES)); else dst->sysCfgBits |= ((uint32_t)(SYS_CFG_BITS_DISABLE_AUTO_ZERO_ANGULAR_RATE_UPDATES));
    if (view->shock.enabled) dst->imuShockOptions |= ((uint32_t)(IMU_SHOCK_OPTIONS_ENABLE)); else dst->imuShockOptions &= ~((uint32_t)(IMU_SHOCK_OPTIONS_ENABLE));
    if (view->shock.fastRecovery) dst->imuShockOptions |= ((uint32_t)(IMU_SHOCK_OPTIONS_FAST_RECOVERY)); else dst->imuShockOptions &= ~((uint32_t)(IMU_SHOCK_OPTIONS_FAST_RECOVERY));
    dst->imuShockDetectLatencyMs = view->shock.detectLatencyMs;
    dst->imuShockRejectLatchMs = view->shock.rejectLatchMs;
    dst->imuShockDeltaAccHighThreshold = view->shock.deltaAccHigh;
    dst->imuShockDeltaAccLowThreshold = view->shock.deltaAccLow;
    dst->imuShockDeltaGyroHighThreshold = view->shock.deltaGyroHigh;
    dst->imuShockDeltaGyroLowThreshold = view->shock.deltaGyroLow;
    if (view->faultDetect.gyr) dst->sensorConfig |= ((uint32_t)(SENSOR_CFG_IMU_FAULT_DETECT_GYR)); else dst->sensorConfig &= ~((uint32_t)(SENSOR_CFG_IMU_FAULT_DETECT_GYR));
    if (view->faultDetect.acc) dst->sensorConfig |= ((uint32_t)(SENSOR_CFG_IMU_FAULT_DETECT_ACC)); else dst->sensorConfig &= ~((uint32_t)(SENSOR_CFG_IMU_FAULT_DETECT_ACC));
    dst->imuRejectThreshGyroLow = view->faultDetect.gyroRejectThreshLow;
    dst->imuRejectThreshGyroHigh = view->faultDetect.gyroRejectThreshHigh;
    return true;
}

/**
 * Fills the CONFIG_MAG keys held in nvm_flash_cfg_t into the view.
 *
 * Keys of CONFIG_MAG that live elsewhere are left as they are, so the view of a domain whose keys live in
 * several structs is filled by each of their mappings in turn.
 *
 * @param view  [out] the view
 * @param src   the nvm_flash_cfg_t the keys are read from
 */
void cbor_flash_config_to_config_mag(cbor_config_mag_t *view, const nvm_flash_cfg_t *src)
{
    view->continuousCal = ((src->sysCfgBits & ((uint32_t)(SYS_CFG_BITS_ENABLE_MAG_CONTINUOUS_CAL))) != 0u);
    view->autoRecal = ((src->sysCfgBits & ((uint32_t)(SYS_CFG_BITS_AUTO_MAG_RECAL))) != 0u);
    view->recalMode = (uint8_t)(((uint32_t)src->sysCfgBits & ((uint32_t)(SYS_CFG_BITS_MAG_RECAL_MODE_MASK))) / CBOR_BIND_LSB(((uint32_t)(SYS_CFG_BITS_MAG_RECAL_MODE_MASK))));
    view->interferenceThreshold = src->magInterferenceThreshold;
    view->calQualityThreshold = src->magCalibrationQualityThreshold;
    view->declination.valueRad = src->magDeclination;
    view->declination.estimate = ((src->sysCfgBits & ((uint32_t)(SYS_CFG_BITS_DISABLE_MAG_DECL_ESTIMATION))) == 0u);
    view->declination.useWmm = ((src->sysCfgBits & ((uint32_t)(SYS_CFG_BITS_MAG_ENABLE_WMM_DECLINATION))) != 0u);
}

/**
 * Checks that every CONFIG_MAG value held in nvm_flash_cfg_t fits its home (a packed field or a bool).
 *
 * A domain whose keys live in several structs is written all or nothing by checking each of their mappings
 * before writing any of them.
 *
 * @param view  the view
 * @return true if cbor_flash_config_from_config_mag() would write the view
 */
bool cbor_flash_config_fits_config_mag(const cbor_config_mag_t *view)
{
    if (view->continuousCal > 1u) return false;
    if (view->autoRecal > 1u) return false;
    if (((uint64_t)view->recalMode * CBOR_BIND_LSB(((uint32_t)(SYS_CFG_BITS_MAG_RECAL_MODE_MASK)))) & ~(uint64_t)((uint32_t)(SYS_CFG_BITS_MAG_RECAL_MODE_MASK))) return false;
    if (view->declination.estimate > 1u) return false;
    if (view->declination.useWmm > 1u) return false;
    return true;
}

/**
 * Writes the CONFIG_MAG keys held in nvm_flash_cfg_t from the view, all or nothing.
 *
 * Bits and members of nvm_flash_cfg_t that no key covers are kept. Nothing is written unless every value fits its
 * home (a packed field or a bool).
 *
 * @param dst   the nvm_flash_cfg_t the keys are written to
 * @param view  the view
 * @return false, writing nothing, if a value does not fit its home
 */
bool cbor_flash_config_from_config_mag(nvm_flash_cfg_t *dst, const cbor_config_mag_t *view)
{
    if (view->continuousCal > 1u) return false;
    if (view->autoRecal > 1u) return false;
    if (((uint64_t)view->recalMode * CBOR_BIND_LSB(((uint32_t)(SYS_CFG_BITS_MAG_RECAL_MODE_MASK)))) & ~(uint64_t)((uint32_t)(SYS_CFG_BITS_MAG_RECAL_MODE_MASK))) return false;
    if (view->declination.estimate > 1u) return false;
    if (view->declination.useWmm > 1u) return false;
    if (view->continuousCal) dst->sysCfgBits |= ((uint32_t)(SYS_CFG_BITS_ENABLE_MAG_CONTINUOUS_CAL)); else dst->sysCfgBits &= ~((uint32_t)(SYS_CFG_BITS_ENABLE_MAG_CONTINUOUS_CAL));
    if (view->autoRecal) dst->sysCfgBits |= ((uint32_t)(SYS_CFG_BITS_AUTO_MAG_RECAL)); else dst->sysCfgBits &= ~((uint32_t)(SYS_CFG_BITS_AUTO_MAG_RECAL));
    dst->sysCfgBits = (dst->sysCfgBits & ~((uint32_t)(SYS_CFG_BITS_MAG_RECAL_MODE_MASK))) | (((uint32_t)view->recalMode * CBOR_BIND_LSB(((uint32_t)(SYS_CFG_BITS_MAG_RECAL_MODE_MASK)))) & ((uint32_t)(SYS_CFG_BITS_MAG_RECAL_MODE_MASK)));
    dst->magInterferenceThreshold = view->interferenceThreshold;
    dst->magCalibrationQualityThreshold = view->calQualityThreshold;
    dst->magDeclination = view->declination.valueRad;
    if (view->declination.estimate) dst->sysCfgBits &= ~((uint32_t)(SYS_CFG_BITS_DISABLE_MAG_DECL_ESTIMATION)); else dst->sysCfgBits |= ((uint32_t)(SYS_CFG_BITS_DISABLE_MAG_DECL_ESTIMATION));
    if (view->declination.useWmm) dst->sysCfgBits |= ((uint32_t)(SYS_CFG_BITS_MAG_ENABLE_WMM_DECLINATION)); else dst->sysCfgBits &= ~((uint32_t)(SYS_CFG_BITS_MAG_ENABLE_WMM_DECLINATION));
    return true;
}

/**
 * Fills the CONFIG_GROUND_VEHICLE keys held in nvm_flash_cfg_t into the view.
 *
 * Keys of CONFIG_GROUND_VEHICLE that live elsewhere are left as they are, so the view of a domain whose keys live in
 * several structs is filled by each of their mappings in turn.
 *
 * @param view  [out] the view
 * @param src   the nvm_flash_cfg_t the keys are read from
 */
void cbor_flash_config_to_config_ground_vehicle(cbor_config_ground_vehicle_t *view, const nvm_flash_cfg_t *src)
{
    memcpy(view->transform.rotation, src->wheelConfig.transform.e_b2w, sizeof(view->transform.rotation));
    memcpy(view->transform.rotationSigma, src->wheelConfig.transform.e_b2w_sigma, sizeof(view->transform.rotationSigma));
    memcpy(view->transform.translation, src->wheelConfig.transform.t_b2w, sizeof(view->transform.translation));
    memcpy(view->transform.translationSigma, src->wheelConfig.transform.t_b2w_sigma, sizeof(view->transform.translationSigma));
    view->trackWidthM = src->wheelConfig.track_width;
    view->radiusM = src->wheelConfig.radius;
    view->controlEnabled = ((src->wheelConfig.bits & ((uint32_t)(WHEEL_CFG_BITS_ENABLE_CONTROL))) != 0u);
    view->encoder.enabled = ((src->wheelConfig.bits & ((uint32_t)(WHEEL_CFG_BITS_ENABLE_ENCODER))) != 0u);
    view->encoder.source = (uint8_t)(((uint32_t)src->wheelConfig.bits & ((uint32_t)(WHEEL_CFG_BITS_ENCODER_SOURCE))) / CBOR_BIND_LSB(((uint32_t)(WHEEL_CFG_BITS_ENCODER_SOURCE))));
    view->encoder.reverseLeft = ((src->wheelConfig.bits & ((uint32_t)(WHEEL_CFG_BITS_DIRECTION_REVERSE_LEFT))) != 0u);
    view->encoder.reverseRight = ((src->wheelConfig.bits & ((uint32_t)(WHEEL_CFG_BITS_DIRECTION_REVERSE_RIGHT))) != 0u);
}

/**
 * Checks that every CONFIG_GROUND_VEHICLE value held in nvm_flash_cfg_t fits its home (a packed field or a bool).
 *
 * A domain whose keys live in several structs is written all or nothing by checking each of their mappings
 * before writing any of them.
 *
 * @param view  the view
 * @return true if cbor_flash_config_from_config_ground_vehicle() would write the view
 */
bool cbor_flash_config_fits_config_ground_vehicle(const cbor_config_ground_vehicle_t *view)
{
    if (view->controlEnabled > 1u) return false;
    if (view->encoder.enabled > 1u) return false;
    if (((uint64_t)view->encoder.source * CBOR_BIND_LSB(((uint32_t)(WHEEL_CFG_BITS_ENCODER_SOURCE)))) & ~(uint64_t)((uint32_t)(WHEEL_CFG_BITS_ENCODER_SOURCE))) return false;
    if (view->encoder.reverseLeft > 1u) return false;
    if (view->encoder.reverseRight > 1u) return false;
    return true;
}

/**
 * Writes the CONFIG_GROUND_VEHICLE keys held in nvm_flash_cfg_t from the view, all or nothing.
 *
 * Bits and members of nvm_flash_cfg_t that no key covers are kept. Nothing is written unless every value fits its
 * home (a packed field or a bool).
 *
 * @param dst   the nvm_flash_cfg_t the keys are written to
 * @param view  the view
 * @return false, writing nothing, if a value does not fit its home
 */
bool cbor_flash_config_from_config_ground_vehicle(nvm_flash_cfg_t *dst, const cbor_config_ground_vehicle_t *view)
{
    if (view->controlEnabled > 1u) return false;
    if (view->encoder.enabled > 1u) return false;
    if (((uint64_t)view->encoder.source * CBOR_BIND_LSB(((uint32_t)(WHEEL_CFG_BITS_ENCODER_SOURCE)))) & ~(uint64_t)((uint32_t)(WHEEL_CFG_BITS_ENCODER_SOURCE))) return false;
    if (view->encoder.reverseLeft > 1u) return false;
    if (view->encoder.reverseRight > 1u) return false;
    memcpy(dst->wheelConfig.transform.e_b2w, view->transform.rotation, sizeof(view->transform.rotation));
    memcpy(dst->wheelConfig.transform.e_b2w_sigma, view->transform.rotationSigma, sizeof(view->transform.rotationSigma));
    memcpy(dst->wheelConfig.transform.t_b2w, view->transform.translation, sizeof(view->transform.translation));
    memcpy(dst->wheelConfig.transform.t_b2w_sigma, view->transform.translationSigma, sizeof(view->transform.translationSigma));
    dst->wheelConfig.track_width = view->trackWidthM;
    dst->wheelConfig.radius = view->radiusM;
    if (view->controlEnabled) dst->wheelConfig.bits |= ((uint32_t)(WHEEL_CFG_BITS_ENABLE_CONTROL)); else dst->wheelConfig.bits &= ~((uint32_t)(WHEEL_CFG_BITS_ENABLE_CONTROL));
    if (view->encoder.enabled) dst->wheelConfig.bits |= ((uint32_t)(WHEEL_CFG_BITS_ENABLE_ENCODER)); else dst->wheelConfig.bits &= ~((uint32_t)(WHEEL_CFG_BITS_ENABLE_ENCODER));
    dst->wheelConfig.bits = (dst->wheelConfig.bits & ~((uint32_t)(WHEEL_CFG_BITS_ENCODER_SOURCE))) | (((uint32_t)view->encoder.source * CBOR_BIND_LSB(((uint32_t)(WHEEL_CFG_BITS_ENCODER_SOURCE)))) & ((uint32_t)(WHEEL_CFG_BITS_ENCODER_SOURCE)));
    if (view->encoder.reverseLeft) dst->wheelConfig.bits |= ((uint32_t)(WHEEL_CFG_BITS_DIRECTION_REVERSE_LEFT)); else dst->wheelConfig.bits &= ~((uint32_t)(WHEEL_CFG_BITS_DIRECTION_REVERSE_LEFT));
    if (view->encoder.reverseRight) dst->wheelConfig.bits |= ((uint32_t)(WHEEL_CFG_BITS_DIRECTION_REVERSE_RIGHT)); else dst->wheelConfig.bits &= ~((uint32_t)(WHEEL_CFG_BITS_DIRECTION_REVERSE_RIGHT));
    return true;
}

/**
 * Fills the CONFIG_PORTS keys held in nvm_flash_cfg_t into the view.
 *
 * Keys of CONFIG_PORTS that live elsewhere are left as they are, so the view of a domain whose keys live in
 * several structs is filled by each of their mappings in turn.
 *
 * @param view  [out] the view
 * @param src   the nvm_flash_cfg_t the keys are read from
 */
void cbor_flash_config_to_config_ports(cbor_config_ports_t *view, const nvm_flash_cfg_t *src)
{
    view->Port[CBOR_PORT_ID_SER0].baudRate = src->ser0BaudRate;
    view->Port[CBOR_PORT_ID_SER1].baudRate = src->ser1BaudRate;
    view->Port[CBOR_PORT_ID_SER2].baudRate = src->ser2BaudRate;
}

/**
 * Checks that every CONFIG_PORTS value held in nvm_flash_cfg_t fits its home (a packed field or a bool).
 *
 * A domain whose keys live in several structs is written all or nothing by checking each of their mappings
 * before writing any of them.
 *
 * @param view  the view
 * @return true if cbor_flash_config_from_config_ports() would write the view
 */
bool cbor_flash_config_fits_config_ports(const cbor_config_ports_t *view)
{
    (void)view;
    return true;
}

/**
 * Writes the CONFIG_PORTS keys held in nvm_flash_cfg_t from the view, all or nothing.
 *
 * Bits and members of nvm_flash_cfg_t that no key covers are kept. Nothing is written unless every value fits its
 * home (a packed field or a bool).
 *
 * @param dst   the nvm_flash_cfg_t the keys are written to
 * @param view  the view
 * @return false, writing nothing, if a value does not fit its home
 */
bool cbor_flash_config_from_config_ports(nvm_flash_cfg_t *dst, const cbor_config_ports_t *view)
{
    dst->ser0BaudRate = view->Port[CBOR_PORT_ID_SER0].baudRate;
    dst->ser1BaudRate = view->Port[CBOR_PORT_ID_SER1].baudRate;
    dst->ser2BaudRate = view->Port[CBOR_PORT_ID_SER2].baudRate;
    return true;
}

/**
 * Fills the CONFIG_SENSORS keys held in nvm_flash_cfg_t into the view.
 *
 * Keys of CONFIG_SENSORS that live elsewhere are left as they are, so the view of a domain whose keys live in
 * several structs is filled by each of their mappings in turn.
 *
 * @param view  [out] the view
 * @param src   the nvm_flash_cfg_t the keys are read from
 */
void cbor_flash_config_to_config_sensors(cbor_config_sensors_t *view, const nvm_flash_cfg_t *src)
{
    view->rotation = (uint8_t)(((uint32_t)src->sensorConfig & ((uint32_t)(SENSOR_CFG_SENSOR_ROTATION_MASK))) / CBOR_BIND_LSB(((uint32_t)(SENSOR_CFG_SENSOR_ROTATION_MASK))));
    view->Imus.Gyr[CBOR_IMU_CHANNEL_PRIMARY].fullScale = (uint8_t)(((uint32_t)src->sensorConfig & ((uint32_t)(SENSOR_CFG_GYR_FS_MASK))) / CBOR_BIND_LSB(((uint32_t)(SENSOR_CFG_GYR_FS_MASK))));
    view->Imus.Gyr[CBOR_IMU_CHANNEL_PRIMARY].dlpf = (uint8_t)(((uint32_t)src->sensorConfig & ((uint32_t)(SENSOR_CFG_GYR_DLPF_MASK))) / CBOR_BIND_LSB(((uint32_t)(SENSOR_CFG_GYR_DLPF_MASK))));
    view->Imus.Acc[CBOR_IMU_CHANNEL_PRIMARY].fullScale = (uint8_t)(((uint32_t)src->sensorConfig & ((uint32_t)(SENSOR_CFG_ACC_FS_MASK))) / CBOR_BIND_LSB(((uint32_t)(SENSOR_CFG_ACC_FS_MASK))));
    view->Imus.Acc[CBOR_IMU_CHANNEL_PRIMARY].dlpf = (uint8_t)(((uint32_t)src->sensorConfig & ((uint32_t)(SENSOR_CFG_ACC_DLPF_MASK))) / CBOR_BIND_LSB(((uint32_t)(SENSOR_CFG_ACC_DLPF_MASK))));
    view->Imus.Acc[CBOR_IMU_CHANNEL_HIGH_G].fullScale = (uint8_t)(((uint32_t)src->sensorConfig2 & ((uint32_t)(SENSOR_CFG2_ACC_HG_FS_MASK))) / CBOR_BIND_LSB(((uint32_t)(SENSOR_CFG2_ACC_HG_FS_MASK))));
    view->Imu[CBOR_IMU_SLOT_IMU1].enabled = ((src->sensorConfig2 & ((uint32_t)(SENSOR_CFG2_IMU_1_DISABLE))) == 0u);
    view->Imu[CBOR_IMU_SLOT_IMU2].enabled = ((src->sensorConfig2 & ((uint32_t)(SENSOR_CFG2_IMU_2_DISABLE))) == 0u);
    view->Imu[CBOR_IMU_SLOT_IMU3].enabled = ((src->sensorConfig2 & ((uint32_t)(SENSOR_CFG2_IMU_3_DISABLE))) == 0u);
    view->Imu[CBOR_IMU_SLOT_IMU4].enabled = ((src->sensorConfig2 & ((uint32_t)(SENSOR_CFG2_IMU_4_DISABLE))) == 0u);
    view->Imu[CBOR_IMU_SLOT_IMU5].enabled = ((src->sensorConfig2 & ((uint32_t)(SENSOR_CFG2_IMU_5_DISABLE))) == 0u);
    view->Mags.enabled = ((src->sensorConfig & ((uint32_t)(SENSOR_CFG_DISABLE_MAGNETOMETER))) == 0u);
    view->Baros.enabled = ((src->sensorConfig & ((uint32_t)(SENSOR_CFG_DISABLE_BAROMETER))) == 0u);
}

/**
 * Checks that every CONFIG_SENSORS value held in nvm_flash_cfg_t fits its home (a packed field or a bool).
 *
 * A domain whose keys live in several structs is written all or nothing by checking each of their mappings
 * before writing any of them.
 *
 * @param view  the view
 * @return true if cbor_flash_config_from_config_sensors() would write the view
 */
bool cbor_flash_config_fits_config_sensors(const cbor_config_sensors_t *view)
{
    if (((uint64_t)view->rotation * CBOR_BIND_LSB(((uint32_t)(SENSOR_CFG_SENSOR_ROTATION_MASK)))) & ~(uint64_t)((uint32_t)(SENSOR_CFG_SENSOR_ROTATION_MASK))) return false;
    if (((uint64_t)view->Imus.Gyr[CBOR_IMU_CHANNEL_PRIMARY].fullScale * CBOR_BIND_LSB(((uint32_t)(SENSOR_CFG_GYR_FS_MASK)))) & ~(uint64_t)((uint32_t)(SENSOR_CFG_GYR_FS_MASK))) return false;
    if (((uint64_t)view->Imus.Gyr[CBOR_IMU_CHANNEL_PRIMARY].dlpf * CBOR_BIND_LSB(((uint32_t)(SENSOR_CFG_GYR_DLPF_MASK)))) & ~(uint64_t)((uint32_t)(SENSOR_CFG_GYR_DLPF_MASK))) return false;
    if (((uint64_t)view->Imus.Acc[CBOR_IMU_CHANNEL_PRIMARY].fullScale * CBOR_BIND_LSB(((uint32_t)(SENSOR_CFG_ACC_FS_MASK)))) & ~(uint64_t)((uint32_t)(SENSOR_CFG_ACC_FS_MASK))) return false;
    if (((uint64_t)view->Imus.Acc[CBOR_IMU_CHANNEL_PRIMARY].dlpf * CBOR_BIND_LSB(((uint32_t)(SENSOR_CFG_ACC_DLPF_MASK)))) & ~(uint64_t)((uint32_t)(SENSOR_CFG_ACC_DLPF_MASK))) return false;
    if (((uint64_t)view->Imus.Acc[CBOR_IMU_CHANNEL_HIGH_G].fullScale * CBOR_BIND_LSB(((uint32_t)(SENSOR_CFG2_ACC_HG_FS_MASK)))) & ~(uint64_t)((uint32_t)(SENSOR_CFG2_ACC_HG_FS_MASK))) return false;
    if (view->Imu[CBOR_IMU_SLOT_IMU1].enabled > 1u) return false;
    if (view->Imu[CBOR_IMU_SLOT_IMU2].enabled > 1u) return false;
    if (view->Imu[CBOR_IMU_SLOT_IMU3].enabled > 1u) return false;
    if (view->Imu[CBOR_IMU_SLOT_IMU4].enabled > 1u) return false;
    if (view->Imu[CBOR_IMU_SLOT_IMU5].enabled > 1u) return false;
    if (view->Mags.enabled > 1u) return false;
    if (view->Baros.enabled > 1u) return false;
    return true;
}

/**
 * Writes the CONFIG_SENSORS keys held in nvm_flash_cfg_t from the view, all or nothing.
 *
 * Bits and members of nvm_flash_cfg_t that no key covers are kept. Nothing is written unless every value fits its
 * home (a packed field or a bool).
 *
 * @param dst   the nvm_flash_cfg_t the keys are written to
 * @param view  the view
 * @return false, writing nothing, if a value does not fit its home
 */
bool cbor_flash_config_from_config_sensors(nvm_flash_cfg_t *dst, const cbor_config_sensors_t *view)
{
    if (((uint64_t)view->rotation * CBOR_BIND_LSB(((uint32_t)(SENSOR_CFG_SENSOR_ROTATION_MASK)))) & ~(uint64_t)((uint32_t)(SENSOR_CFG_SENSOR_ROTATION_MASK))) return false;
    if (((uint64_t)view->Imus.Gyr[CBOR_IMU_CHANNEL_PRIMARY].fullScale * CBOR_BIND_LSB(((uint32_t)(SENSOR_CFG_GYR_FS_MASK)))) & ~(uint64_t)((uint32_t)(SENSOR_CFG_GYR_FS_MASK))) return false;
    if (((uint64_t)view->Imus.Gyr[CBOR_IMU_CHANNEL_PRIMARY].dlpf * CBOR_BIND_LSB(((uint32_t)(SENSOR_CFG_GYR_DLPF_MASK)))) & ~(uint64_t)((uint32_t)(SENSOR_CFG_GYR_DLPF_MASK))) return false;
    if (((uint64_t)view->Imus.Acc[CBOR_IMU_CHANNEL_PRIMARY].fullScale * CBOR_BIND_LSB(((uint32_t)(SENSOR_CFG_ACC_FS_MASK)))) & ~(uint64_t)((uint32_t)(SENSOR_CFG_ACC_FS_MASK))) return false;
    if (((uint64_t)view->Imus.Acc[CBOR_IMU_CHANNEL_PRIMARY].dlpf * CBOR_BIND_LSB(((uint32_t)(SENSOR_CFG_ACC_DLPF_MASK)))) & ~(uint64_t)((uint32_t)(SENSOR_CFG_ACC_DLPF_MASK))) return false;
    if (((uint64_t)view->Imus.Acc[CBOR_IMU_CHANNEL_HIGH_G].fullScale * CBOR_BIND_LSB(((uint32_t)(SENSOR_CFG2_ACC_HG_FS_MASK)))) & ~(uint64_t)((uint32_t)(SENSOR_CFG2_ACC_HG_FS_MASK))) return false;
    if (view->Imu[CBOR_IMU_SLOT_IMU1].enabled > 1u) return false;
    if (view->Imu[CBOR_IMU_SLOT_IMU2].enabled > 1u) return false;
    if (view->Imu[CBOR_IMU_SLOT_IMU3].enabled > 1u) return false;
    if (view->Imu[CBOR_IMU_SLOT_IMU4].enabled > 1u) return false;
    if (view->Imu[CBOR_IMU_SLOT_IMU5].enabled > 1u) return false;
    if (view->Mags.enabled > 1u) return false;
    if (view->Baros.enabled > 1u) return false;
    dst->sensorConfig = (dst->sensorConfig & ~((uint32_t)(SENSOR_CFG_SENSOR_ROTATION_MASK))) | (((uint32_t)view->rotation * CBOR_BIND_LSB(((uint32_t)(SENSOR_CFG_SENSOR_ROTATION_MASK)))) & ((uint32_t)(SENSOR_CFG_SENSOR_ROTATION_MASK)));
    dst->sensorConfig = (dst->sensorConfig & ~((uint32_t)(SENSOR_CFG_GYR_FS_MASK))) | (((uint32_t)view->Imus.Gyr[CBOR_IMU_CHANNEL_PRIMARY].fullScale * CBOR_BIND_LSB(((uint32_t)(SENSOR_CFG_GYR_FS_MASK)))) & ((uint32_t)(SENSOR_CFG_GYR_FS_MASK)));
    dst->sensorConfig = (dst->sensorConfig & ~((uint32_t)(SENSOR_CFG_GYR_DLPF_MASK))) | (((uint32_t)view->Imus.Gyr[CBOR_IMU_CHANNEL_PRIMARY].dlpf * CBOR_BIND_LSB(((uint32_t)(SENSOR_CFG_GYR_DLPF_MASK)))) & ((uint32_t)(SENSOR_CFG_GYR_DLPF_MASK)));
    dst->sensorConfig = (dst->sensorConfig & ~((uint32_t)(SENSOR_CFG_ACC_FS_MASK))) | (((uint32_t)view->Imus.Acc[CBOR_IMU_CHANNEL_PRIMARY].fullScale * CBOR_BIND_LSB(((uint32_t)(SENSOR_CFG_ACC_FS_MASK)))) & ((uint32_t)(SENSOR_CFG_ACC_FS_MASK)));
    dst->sensorConfig = (dst->sensorConfig & ~((uint32_t)(SENSOR_CFG_ACC_DLPF_MASK))) | (((uint32_t)view->Imus.Acc[CBOR_IMU_CHANNEL_PRIMARY].dlpf * CBOR_BIND_LSB(((uint32_t)(SENSOR_CFG_ACC_DLPF_MASK)))) & ((uint32_t)(SENSOR_CFG_ACC_DLPF_MASK)));
    dst->sensorConfig2 = (dst->sensorConfig2 & ~((uint32_t)(SENSOR_CFG2_ACC_HG_FS_MASK))) | (((uint32_t)view->Imus.Acc[CBOR_IMU_CHANNEL_HIGH_G].fullScale * CBOR_BIND_LSB(((uint32_t)(SENSOR_CFG2_ACC_HG_FS_MASK)))) & ((uint32_t)(SENSOR_CFG2_ACC_HG_FS_MASK)));
    if (view->Imu[CBOR_IMU_SLOT_IMU1].enabled) dst->sensorConfig2 &= ~((uint32_t)(SENSOR_CFG2_IMU_1_DISABLE)); else dst->sensorConfig2 |= ((uint32_t)(SENSOR_CFG2_IMU_1_DISABLE));
    if (view->Imu[CBOR_IMU_SLOT_IMU2].enabled) dst->sensorConfig2 &= ~((uint32_t)(SENSOR_CFG2_IMU_2_DISABLE)); else dst->sensorConfig2 |= ((uint32_t)(SENSOR_CFG2_IMU_2_DISABLE));
    if (view->Imu[CBOR_IMU_SLOT_IMU3].enabled) dst->sensorConfig2 &= ~((uint32_t)(SENSOR_CFG2_IMU_3_DISABLE)); else dst->sensorConfig2 |= ((uint32_t)(SENSOR_CFG2_IMU_3_DISABLE));
    if (view->Imu[CBOR_IMU_SLOT_IMU4].enabled) dst->sensorConfig2 &= ~((uint32_t)(SENSOR_CFG2_IMU_4_DISABLE)); else dst->sensorConfig2 |= ((uint32_t)(SENSOR_CFG2_IMU_4_DISABLE));
    if (view->Imu[CBOR_IMU_SLOT_IMU5].enabled) dst->sensorConfig2 &= ~((uint32_t)(SENSOR_CFG2_IMU_5_DISABLE)); else dst->sensorConfig2 |= ((uint32_t)(SENSOR_CFG2_IMU_5_DISABLE));
    if (view->Mags.enabled) dst->sensorConfig &= ~((uint32_t)(SENSOR_CFG_DISABLE_MAGNETOMETER)); else dst->sensorConfig |= ((uint32_t)(SENSOR_CFG_DISABLE_MAGNETOMETER));
    if (view->Baros.enabled) dst->sensorConfig &= ~((uint32_t)(SENSOR_CFG_DISABLE_BAROMETER)); else dst->sensorConfig |= ((uint32_t)(SENSOR_CFG_DISABLE_BAROMETER));
    return true;
}

/**
 * Fills the CONFIG_IO keys held in nvm_flash_cfg_t into the view.
 *
 * Keys of CONFIG_IO that live elsewhere are left as they are, so the view of a domain whose keys live in
 * several structs is filled by each of their mappings in turn.
 *
 * @param view  [out] the view
 * @param src   the nvm_flash_cfg_t the keys are read from
 */
void cbor_flash_config_to_config_io(cbor_config_io_t *view, const nvm_flash_cfg_t *src)
{
    view->strobeActiveHigh = ((src->ioConfig & ((uint32_t)(IO_CONFIG_STROBE_TRIGGER_HIGH))) != 0u);
    view->Pin[CBOR_PIN_ID_G1G2].function = (uint8_t)(((uint32_t)src->ioConfig & ((uint32_t)(IO_CONFIG_G1G2_MASK))) / CBOR_BIND_LSB(((uint32_t)(IO_CONFIG_G1G2_MASK))));
    view->Pin[CBOR_PIN_ID_G5G8].function = (uint8_t)(((uint32_t)src->ioConfig & ((uint32_t)(IO_CONFIG_G5G8_MASK))) / CBOR_BIND_LSB(((uint32_t)(IO_CONFIG_G5G8_MASK))));
    view->Pin[CBOR_PIN_ID_G6G7].function = (uint8_t)(((uint32_t)src->ioConfig & ((uint32_t)(IO_CONFIG_G6G7_MASK))) / CBOR_BIND_LSB(((uint32_t)(IO_CONFIG_G6G7_MASK))));
    view->Pin[CBOR_PIN_ID_G9].function = (uint8_t)(((uint32_t)src->ioConfig & ((uint32_t)(IO_CONFIG_G9_MASK))) / CBOR_BIND_LSB(((uint32_t)(IO_CONFIG_G9_MASK))));
    view->Pin[CBOR_PIN_ID_G11].function = (uint8_t)(((uint32_t)src->ioConfig2 & ((uint32_t)(IO_CFG2_G11_BITMASK))) / CBOR_BIND_LSB(((uint32_t)(IO_CFG2_G11_BITMASK))));
    view->Pin[CBOR_PIN_ID_G12].function = (uint8_t)(((uint32_t)src->ioConfig2 & ((uint32_t)(IO_CFG2_G12_BITMASK))) / CBOR_BIND_LSB(((uint32_t)(IO_CFG2_G12_BITMASK))));
    view->Pin[CBOR_PIN_ID_G13].function = (uint8_t)(((uint32_t)src->ioConfig2 & ((uint32_t)(IO_CFG2_G13_BITMASK))) / CBOR_BIND_LSB(((uint32_t)(IO_CFG2_G13_BITMASK))));
    view->Pin[CBOR_PIN_ID_G15].function = (uint8_t)(((uint32_t)src->ioConfig & ((uint32_t)(IO_CONFIG_G15_STROBE_INPUT))) / CBOR_BIND_LSB(((uint32_t)(IO_CONFIG_G15_STROBE_INPUT))));
}

/**
 * Checks that every CONFIG_IO value held in nvm_flash_cfg_t fits its home (a packed field or a bool).
 *
 * A domain whose keys live in several structs is written all or nothing by checking each of their mappings
 * before writing any of them.
 *
 * @param view  the view
 * @return true if cbor_flash_config_from_config_io() would write the view
 */
bool cbor_flash_config_fits_config_io(const cbor_config_io_t *view)
{
    if (view->strobeActiveHigh > 1u) return false;
    if (((uint64_t)view->Pin[CBOR_PIN_ID_G1G2].function * CBOR_BIND_LSB(((uint32_t)(IO_CONFIG_G1G2_MASK)))) & ~(uint64_t)((uint32_t)(IO_CONFIG_G1G2_MASK))) return false;
    if (((uint64_t)view->Pin[CBOR_PIN_ID_G5G8].function * CBOR_BIND_LSB(((uint32_t)(IO_CONFIG_G5G8_MASK)))) & ~(uint64_t)((uint32_t)(IO_CONFIG_G5G8_MASK))) return false;
    if (((uint64_t)view->Pin[CBOR_PIN_ID_G6G7].function * CBOR_BIND_LSB(((uint32_t)(IO_CONFIG_G6G7_MASK)))) & ~(uint64_t)((uint32_t)(IO_CONFIG_G6G7_MASK))) return false;
    if (((uint64_t)view->Pin[CBOR_PIN_ID_G9].function * CBOR_BIND_LSB(((uint32_t)(IO_CONFIG_G9_MASK)))) & ~(uint64_t)((uint32_t)(IO_CONFIG_G9_MASK))) return false;
    if (((uint64_t)view->Pin[CBOR_PIN_ID_G11].function * CBOR_BIND_LSB(((uint32_t)(IO_CFG2_G11_BITMASK)))) & ~(uint64_t)((uint32_t)(IO_CFG2_G11_BITMASK))) return false;
    if (((uint64_t)view->Pin[CBOR_PIN_ID_G12].function * CBOR_BIND_LSB(((uint32_t)(IO_CFG2_G12_BITMASK)))) & ~(uint64_t)((uint32_t)(IO_CFG2_G12_BITMASK))) return false;
    if (((uint64_t)view->Pin[CBOR_PIN_ID_G13].function * CBOR_BIND_LSB(((uint32_t)(IO_CFG2_G13_BITMASK)))) & ~(uint64_t)((uint32_t)(IO_CFG2_G13_BITMASK))) return false;
    if (((uint64_t)view->Pin[CBOR_PIN_ID_G15].function * CBOR_BIND_LSB(((uint32_t)(IO_CONFIG_G15_STROBE_INPUT)))) & ~(uint64_t)((uint32_t)(IO_CONFIG_G15_STROBE_INPUT))) return false;
    return true;
}

/**
 * Writes the CONFIG_IO keys held in nvm_flash_cfg_t from the view, all or nothing.
 *
 * Bits and members of nvm_flash_cfg_t that no key covers are kept. Nothing is written unless every value fits its
 * home (a packed field or a bool).
 *
 * @param dst   the nvm_flash_cfg_t the keys are written to
 * @param view  the view
 * @return false, writing nothing, if a value does not fit its home
 */
bool cbor_flash_config_from_config_io(nvm_flash_cfg_t *dst, const cbor_config_io_t *view)
{
    if (view->strobeActiveHigh > 1u) return false;
    if (((uint64_t)view->Pin[CBOR_PIN_ID_G1G2].function * CBOR_BIND_LSB(((uint32_t)(IO_CONFIG_G1G2_MASK)))) & ~(uint64_t)((uint32_t)(IO_CONFIG_G1G2_MASK))) return false;
    if (((uint64_t)view->Pin[CBOR_PIN_ID_G5G8].function * CBOR_BIND_LSB(((uint32_t)(IO_CONFIG_G5G8_MASK)))) & ~(uint64_t)((uint32_t)(IO_CONFIG_G5G8_MASK))) return false;
    if (((uint64_t)view->Pin[CBOR_PIN_ID_G6G7].function * CBOR_BIND_LSB(((uint32_t)(IO_CONFIG_G6G7_MASK)))) & ~(uint64_t)((uint32_t)(IO_CONFIG_G6G7_MASK))) return false;
    if (((uint64_t)view->Pin[CBOR_PIN_ID_G9].function * CBOR_BIND_LSB(((uint32_t)(IO_CONFIG_G9_MASK)))) & ~(uint64_t)((uint32_t)(IO_CONFIG_G9_MASK))) return false;
    if (((uint64_t)view->Pin[CBOR_PIN_ID_G11].function * CBOR_BIND_LSB(((uint32_t)(IO_CFG2_G11_BITMASK)))) & ~(uint64_t)((uint32_t)(IO_CFG2_G11_BITMASK))) return false;
    if (((uint64_t)view->Pin[CBOR_PIN_ID_G12].function * CBOR_BIND_LSB(((uint32_t)(IO_CFG2_G12_BITMASK)))) & ~(uint64_t)((uint32_t)(IO_CFG2_G12_BITMASK))) return false;
    if (((uint64_t)view->Pin[CBOR_PIN_ID_G13].function * CBOR_BIND_LSB(((uint32_t)(IO_CFG2_G13_BITMASK)))) & ~(uint64_t)((uint32_t)(IO_CFG2_G13_BITMASK))) return false;
    if (((uint64_t)view->Pin[CBOR_PIN_ID_G15].function * CBOR_BIND_LSB(((uint32_t)(IO_CONFIG_G15_STROBE_INPUT)))) & ~(uint64_t)((uint32_t)(IO_CONFIG_G15_STROBE_INPUT))) return false;
    if (view->strobeActiveHigh) dst->ioConfig |= ((uint32_t)(IO_CONFIG_STROBE_TRIGGER_HIGH)); else dst->ioConfig &= ~((uint32_t)(IO_CONFIG_STROBE_TRIGGER_HIGH));
    dst->ioConfig = (dst->ioConfig & ~((uint32_t)(IO_CONFIG_G1G2_MASK))) | (((uint32_t)view->Pin[CBOR_PIN_ID_G1G2].function * CBOR_BIND_LSB(((uint32_t)(IO_CONFIG_G1G2_MASK)))) & ((uint32_t)(IO_CONFIG_G1G2_MASK)));
    dst->ioConfig = (dst->ioConfig & ~((uint32_t)(IO_CONFIG_G5G8_MASK))) | (((uint32_t)view->Pin[CBOR_PIN_ID_G5G8].function * CBOR_BIND_LSB(((uint32_t)(IO_CONFIG_G5G8_MASK)))) & ((uint32_t)(IO_CONFIG_G5G8_MASK)));
    dst->ioConfig = (dst->ioConfig & ~((uint32_t)(IO_CONFIG_G6G7_MASK))) | (((uint32_t)view->Pin[CBOR_PIN_ID_G6G7].function * CBOR_BIND_LSB(((uint32_t)(IO_CONFIG_G6G7_MASK)))) & ((uint32_t)(IO_CONFIG_G6G7_MASK)));
    dst->ioConfig = (dst->ioConfig & ~((uint32_t)(IO_CONFIG_G9_MASK))) | (((uint32_t)view->Pin[CBOR_PIN_ID_G9].function * CBOR_BIND_LSB(((uint32_t)(IO_CONFIG_G9_MASK)))) & ((uint32_t)(IO_CONFIG_G9_MASK)));
    dst->ioConfig2 = (dst->ioConfig2 & ~((uint32_t)(IO_CFG2_G11_BITMASK))) | (((uint32_t)view->Pin[CBOR_PIN_ID_G11].function * CBOR_BIND_LSB(((uint32_t)(IO_CFG2_G11_BITMASK)))) & ((uint32_t)(IO_CFG2_G11_BITMASK)));
    dst->ioConfig2 = (dst->ioConfig2 & ~((uint32_t)(IO_CFG2_G12_BITMASK))) | (((uint32_t)view->Pin[CBOR_PIN_ID_G12].function * CBOR_BIND_LSB(((uint32_t)(IO_CFG2_G12_BITMASK)))) & ((uint32_t)(IO_CFG2_G12_BITMASK)));
    dst->ioConfig2 = (dst->ioConfig2 & ~((uint32_t)(IO_CFG2_G13_BITMASK))) | (((uint32_t)view->Pin[CBOR_PIN_ID_G13].function * CBOR_BIND_LSB(((uint32_t)(IO_CFG2_G13_BITMASK)))) & ((uint32_t)(IO_CFG2_G13_BITMASK)));
    dst->ioConfig = (dst->ioConfig & ~((uint32_t)(IO_CONFIG_G15_STROBE_INPUT))) | (((uint32_t)view->Pin[CBOR_PIN_ID_G15].function * CBOR_BIND_LSB(((uint32_t)(IO_CONFIG_G15_STROBE_INPUT)))) & ((uint32_t)(IO_CONFIG_G15_STROBE_INPUT)));
    return true;
}

/**
 * Fills the CONFIG_GNSS keys held in nvm_flash_cfg_t into the view.
 *
 * Keys of CONFIG_GNSS that live elsewhere are left as they are, so the view of a domain whose keys live in
 * several structs is filled by each of their mappings in turn.
 *
 * @param view  [out] the view
 * @param src   the nvm_flash_cfg_t the keys are read from
 */
void cbor_flash_config_to_config_gnss(cbor_config_gnss_t *view, const nvm_flash_cfg_t *src)
{
    view->periodMs = src->startupGnssDtMs;
    view->primaryReceiver = (uint8_t)(((uint32_t)src->ioConfig2 & ((uint32_t)(IO_CFG2_USE_GNSS2_AS_SOURCE))) / CBOR_BIND_LSB(((uint32_t)(IO_CFG2_USE_GNSS2_AS_SOURCE))));
    view->timeSync.periodMs = src->gnssTimeSyncPeriodMs;
    view->timeSync.userDelayS = src->gnssTimeUserDelay;
    view->Receivers.satSignals = src->gnssSatSigConst;
    view->Receivers.minElevation = src->gnssMinimumElevation;
    view->Receivers.cn0Min = src->gnssCn0Minimum;
    view->Receivers.cn0DynOffset = src->gnssCn0DynMinOffset;
    view->Receiver[CBOR_RECEIVER_ID_GNSS1].type = (uint8_t)(((uint32_t)src->ioConfig & ((uint32_t)(IO_CONFIG_GNSS_TYPE_MASK << IO_CONFIG_GNSS1_TYPE_OFFSET))) / CBOR_BIND_LSB(((uint32_t)(IO_CONFIG_GNSS_TYPE_MASK << IO_CONFIG_GNSS1_TYPE_OFFSET))));
    view->Receiver[CBOR_RECEIVER_ID_GNSS1].port = (uint8_t)(((uint32_t)src->ioConfig & ((uint32_t)(IO_CONFIG_GNSS_SOURCE_MASK << IO_CONFIG_GNSS1_SOURCE_OFFSET))) / CBOR_BIND_LSB(((uint32_t)(IO_CONFIG_GNSS_SOURCE_MASK << IO_CONFIG_GNSS1_SOURCE_OFFSET))));
    view->Receiver[CBOR_RECEIVER_ID_GNSS1].ppsPin = (uint8_t)(((uint32_t)src->ioConfig & ((uint32_t)(IO_CFG_GNSS1_PPS_SOURCE_BITMASK))) / CBOR_BIND_LSB(((uint32_t)(IO_CFG_GNSS1_PPS_SOURCE_BITMASK))));
    view->Receiver[CBOR_RECEIVER_ID_GNSS1].configure = ((src->ioConfig & ((uint32_t)(IO_CONFIG_GNSS1_NO_INIT))) == 0u);
    view->Receiver[CBOR_RECEIVER_ID_GNSS1].fusionEnabled = ((src->sysCfgBits & ((uint32_t)(SYS_CFG_BITS_DISABLE_GNSS1_FUSION))) == 0u);
    view->Receiver[CBOR_RECEIVER_ID_GNSS2].type = (uint8_t)(((uint32_t)src->ioConfig & ((uint32_t)(IO_CONFIG_GNSS_TYPE_MASK << IO_CONFIG_GNSS2_TYPE_OFFSET))) / CBOR_BIND_LSB(((uint32_t)(IO_CONFIG_GNSS_TYPE_MASK << IO_CONFIG_GNSS2_TYPE_OFFSET))));
    view->Receiver[CBOR_RECEIVER_ID_GNSS2].port = (uint8_t)(((uint32_t)src->ioConfig & ((uint32_t)(IO_CONFIG_GNSS_SOURCE_MASK << IO_CONFIG_GNSS2_SOURCE_OFFSET))) / CBOR_BIND_LSB(((uint32_t)(IO_CONFIG_GNSS_SOURCE_MASK << IO_CONFIG_GNSS2_SOURCE_OFFSET))));
    view->Receiver[CBOR_RECEIVER_ID_GNSS2].ppsPin = (uint8_t)(((uint32_t)src->ioConfig2 & ((uint32_t)(IO_CFG2_GNSS2_PPS_SOURCE_BITMASK))) / CBOR_BIND_LSB(((uint32_t)(IO_CFG2_GNSS2_PPS_SOURCE_BITMASK))));
    view->Receiver[CBOR_RECEIVER_ID_GNSS2].configure = ((src->ioConfig & ((uint32_t)(IO_CONFIG_GNSS2_NO_INIT))) == 0u);
    view->Receiver[CBOR_RECEIVER_ID_GNSS2].fusionEnabled = ((src->sysCfgBits & ((uint32_t)(SYS_CFG_BITS_DISABLE_GNSS2_FUSION))) == 0u);
    view->Antennas.offsetEstimation = ((src->sysCfgBits & ((uint32_t)(SYS_CFG_BITS_ENABLE_GNSS_ANTENNA_OFFSET_ESTIMATION))) != 0u);
    memcpy(view->Antenna[CBOR_ANTENNA_ID_ANT1].offset, src->gnss1AntOffset, sizeof(view->Antenna[CBOR_ANTENNA_ID_ANT1].offset));
    memcpy(view->Antenna[CBOR_ANTENNA_ID_ANT2].offset, src->gnss2AntOffset, sizeof(view->Antenna[CBOR_ANTENNA_ID_ANT2].offset));
    view->Rtk.mode = (uint8_t)(((uint32_t)src->RTKCfgBits & ((uint32_t)(RTK_CFG_BITS_ROVER_MODE_MASK))) / CBOR_BIND_LSB(((uint32_t)(RTK_CFG_BITS_ROVER_MODE_MASK))));
    view->Rtk.basePositionMoving = ((src->RTKCfgBits & ((uint32_t)(RTK_CFG_BITS_BASE_POS_MOVING))) != 0u);
    view->Rtk.baseIdenticalToRover = ((src->RTKCfgBits & ((uint32_t)(RTK_CFG_BITS_RTK_BASE_IS_IDENTICAL_TO_ROVER))) != 0u);
    view->Rtk.passThrough = ((src->RTKCfgBits & ((uint32_t)(RTK_CFG_BITS_GNSS_PORT_PASS_THROUGH))) != 0u);
    view->Rtk.multipathMitigation = ((src->RTKCfgBits & ((uint32_t)(RTK_CFG_BITS_MULTIPATH_MITIGATION))) != 0u);
    view->Rtk.BaseOutput[CBOR_BASE_STREAM_GNSS1_UBLOX].ser0 = ((src->RTKCfgBits & ((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS1_UBLOX_SER0))) != 0u);
    view->Rtk.BaseOutput[CBOR_BASE_STREAM_GNSS1_UBLOX].ser1 = ((src->RTKCfgBits & ((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS1_UBLOX_SER1))) != 0u);
    view->Rtk.BaseOutput[CBOR_BASE_STREAM_GNSS1_UBLOX].ser2 = ((src->RTKCfgBits & ((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS1_UBLOX_SER2))) != 0u);
    view->Rtk.BaseOutput[CBOR_BASE_STREAM_GNSS1_UBLOX].usb = ((src->RTKCfgBits & ((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS1_UBLOX_USB))) != 0u);
    view->Rtk.BaseOutput[CBOR_BASE_STREAM_GNSS1_RTCM3].ser0 = ((src->RTKCfgBits & ((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS1_RTCM3_SER0))) != 0u);
    view->Rtk.BaseOutput[CBOR_BASE_STREAM_GNSS1_RTCM3].ser1 = ((src->RTKCfgBits & ((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS1_RTCM3_SER1))) != 0u);
    view->Rtk.BaseOutput[CBOR_BASE_STREAM_GNSS1_RTCM3].ser2 = ((src->RTKCfgBits & ((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS1_RTCM3_SER2))) != 0u);
    view->Rtk.BaseOutput[CBOR_BASE_STREAM_GNSS1_RTCM3].usb = ((src->RTKCfgBits & ((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS1_RTCM3_USB))) != 0u);
    view->Rtk.BaseOutput[CBOR_BASE_STREAM_GNSS2_UBLOX].ser0 = ((src->RTKCfgBits & ((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS2_UBLOX_SER0))) != 0u);
    view->Rtk.BaseOutput[CBOR_BASE_STREAM_GNSS2_UBLOX].ser1 = ((src->RTKCfgBits & ((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS2_UBLOX_SER1))) != 0u);
    view->Rtk.BaseOutput[CBOR_BASE_STREAM_GNSS2_UBLOX].ser2 = ((src->RTKCfgBits & ((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS2_UBLOX_SER2))) != 0u);
    view->Rtk.BaseOutput[CBOR_BASE_STREAM_GNSS2_UBLOX].usb = ((src->RTKCfgBits & ((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS2_UBLOX_USB))) != 0u);
    view->Rtk.BaseOutput[CBOR_BASE_STREAM_GNSS2_RTCM3].ser0 = ((src->RTKCfgBits & ((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS2_RTCM3_SER0))) != 0u);
    view->Rtk.BaseOutput[CBOR_BASE_STREAM_GNSS2_RTCM3].ser1 = ((src->RTKCfgBits & ((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS2_RTCM3_SER1))) != 0u);
    view->Rtk.BaseOutput[CBOR_BASE_STREAM_GNSS2_RTCM3].ser2 = ((src->RTKCfgBits & ((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS2_RTCM3_SER2))) != 0u);
    view->Rtk.BaseOutput[CBOR_BASE_STREAM_GNSS2_RTCM3].usb = ((src->RTKCfgBits & ((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS2_RTCM3_USB))) != 0u);
}

/**
 * Checks that every CONFIG_GNSS value held in nvm_flash_cfg_t fits its home (a packed field or a bool).
 *
 * A domain whose keys live in several structs is written all or nothing by checking each of their mappings
 * before writing any of them.
 *
 * @param view  the view
 * @return true if cbor_flash_config_from_config_gnss() would write the view
 */
bool cbor_flash_config_fits_config_gnss(const cbor_config_gnss_t *view)
{
    if (((uint64_t)view->primaryReceiver * CBOR_BIND_LSB(((uint32_t)(IO_CFG2_USE_GNSS2_AS_SOURCE)))) & ~(uint64_t)((uint32_t)(IO_CFG2_USE_GNSS2_AS_SOURCE))) return false;
    if (((uint64_t)view->Receiver[CBOR_RECEIVER_ID_GNSS1].type * CBOR_BIND_LSB(((uint32_t)(IO_CONFIG_GNSS_TYPE_MASK << IO_CONFIG_GNSS1_TYPE_OFFSET)))) & ~(uint64_t)((uint32_t)(IO_CONFIG_GNSS_TYPE_MASK << IO_CONFIG_GNSS1_TYPE_OFFSET))) return false;
    if (((uint64_t)view->Receiver[CBOR_RECEIVER_ID_GNSS1].port * CBOR_BIND_LSB(((uint32_t)(IO_CONFIG_GNSS_SOURCE_MASK << IO_CONFIG_GNSS1_SOURCE_OFFSET)))) & ~(uint64_t)((uint32_t)(IO_CONFIG_GNSS_SOURCE_MASK << IO_CONFIG_GNSS1_SOURCE_OFFSET))) return false;
    if (((uint64_t)view->Receiver[CBOR_RECEIVER_ID_GNSS1].ppsPin * CBOR_BIND_LSB(((uint32_t)(IO_CFG_GNSS1_PPS_SOURCE_BITMASK)))) & ~(uint64_t)((uint32_t)(IO_CFG_GNSS1_PPS_SOURCE_BITMASK))) return false;
    if (view->Receiver[CBOR_RECEIVER_ID_GNSS1].configure > 1u) return false;
    if (view->Receiver[CBOR_RECEIVER_ID_GNSS1].fusionEnabled > 1u) return false;
    if (((uint64_t)view->Receiver[CBOR_RECEIVER_ID_GNSS2].type * CBOR_BIND_LSB(((uint32_t)(IO_CONFIG_GNSS_TYPE_MASK << IO_CONFIG_GNSS2_TYPE_OFFSET)))) & ~(uint64_t)((uint32_t)(IO_CONFIG_GNSS_TYPE_MASK << IO_CONFIG_GNSS2_TYPE_OFFSET))) return false;
    if (((uint64_t)view->Receiver[CBOR_RECEIVER_ID_GNSS2].port * CBOR_BIND_LSB(((uint32_t)(IO_CONFIG_GNSS_SOURCE_MASK << IO_CONFIG_GNSS2_SOURCE_OFFSET)))) & ~(uint64_t)((uint32_t)(IO_CONFIG_GNSS_SOURCE_MASK << IO_CONFIG_GNSS2_SOURCE_OFFSET))) return false;
    if (((uint64_t)view->Receiver[CBOR_RECEIVER_ID_GNSS2].ppsPin * CBOR_BIND_LSB(((uint32_t)(IO_CFG2_GNSS2_PPS_SOURCE_BITMASK)))) & ~(uint64_t)((uint32_t)(IO_CFG2_GNSS2_PPS_SOURCE_BITMASK))) return false;
    if (view->Receiver[CBOR_RECEIVER_ID_GNSS2].configure > 1u) return false;
    if (view->Receiver[CBOR_RECEIVER_ID_GNSS2].fusionEnabled > 1u) return false;
    if (view->Antennas.offsetEstimation > 1u) return false;
    if (((uint64_t)view->Rtk.mode * CBOR_BIND_LSB(((uint32_t)(RTK_CFG_BITS_ROVER_MODE_MASK)))) & ~(uint64_t)((uint32_t)(RTK_CFG_BITS_ROVER_MODE_MASK))) return false;
    if (view->Rtk.basePositionMoving > 1u) return false;
    if (view->Rtk.baseIdenticalToRover > 1u) return false;
    if (view->Rtk.passThrough > 1u) return false;
    if (view->Rtk.multipathMitigation > 1u) return false;
    if (view->Rtk.BaseOutput[CBOR_BASE_STREAM_GNSS1_UBLOX].ser0 > 1u) return false;
    if (view->Rtk.BaseOutput[CBOR_BASE_STREAM_GNSS1_UBLOX].ser1 > 1u) return false;
    if (view->Rtk.BaseOutput[CBOR_BASE_STREAM_GNSS1_UBLOX].ser2 > 1u) return false;
    if (view->Rtk.BaseOutput[CBOR_BASE_STREAM_GNSS1_UBLOX].usb > 1u) return false;
    if (view->Rtk.BaseOutput[CBOR_BASE_STREAM_GNSS1_RTCM3].ser0 > 1u) return false;
    if (view->Rtk.BaseOutput[CBOR_BASE_STREAM_GNSS1_RTCM3].ser1 > 1u) return false;
    if (view->Rtk.BaseOutput[CBOR_BASE_STREAM_GNSS1_RTCM3].ser2 > 1u) return false;
    if (view->Rtk.BaseOutput[CBOR_BASE_STREAM_GNSS1_RTCM3].usb > 1u) return false;
    if (view->Rtk.BaseOutput[CBOR_BASE_STREAM_GNSS2_UBLOX].ser0 > 1u) return false;
    if (view->Rtk.BaseOutput[CBOR_BASE_STREAM_GNSS2_UBLOX].ser1 > 1u) return false;
    if (view->Rtk.BaseOutput[CBOR_BASE_STREAM_GNSS2_UBLOX].ser2 > 1u) return false;
    if (view->Rtk.BaseOutput[CBOR_BASE_STREAM_GNSS2_UBLOX].usb > 1u) return false;
    if (view->Rtk.BaseOutput[CBOR_BASE_STREAM_GNSS2_RTCM3].ser0 > 1u) return false;
    if (view->Rtk.BaseOutput[CBOR_BASE_STREAM_GNSS2_RTCM3].ser1 > 1u) return false;
    if (view->Rtk.BaseOutput[CBOR_BASE_STREAM_GNSS2_RTCM3].ser2 > 1u) return false;
    if (view->Rtk.BaseOutput[CBOR_BASE_STREAM_GNSS2_RTCM3].usb > 1u) return false;
    return true;
}

/**
 * Writes the CONFIG_GNSS keys held in nvm_flash_cfg_t from the view, all or nothing.
 *
 * Bits and members of nvm_flash_cfg_t that no key covers are kept. Nothing is written unless every value fits its
 * home (a packed field or a bool).
 *
 * @param dst   the nvm_flash_cfg_t the keys are written to
 * @param view  the view
 * @return false, writing nothing, if a value does not fit its home
 */
bool cbor_flash_config_from_config_gnss(nvm_flash_cfg_t *dst, const cbor_config_gnss_t *view)
{
    if (((uint64_t)view->primaryReceiver * CBOR_BIND_LSB(((uint32_t)(IO_CFG2_USE_GNSS2_AS_SOURCE)))) & ~(uint64_t)((uint32_t)(IO_CFG2_USE_GNSS2_AS_SOURCE))) return false;
    if (((uint64_t)view->Receiver[CBOR_RECEIVER_ID_GNSS1].type * CBOR_BIND_LSB(((uint32_t)(IO_CONFIG_GNSS_TYPE_MASK << IO_CONFIG_GNSS1_TYPE_OFFSET)))) & ~(uint64_t)((uint32_t)(IO_CONFIG_GNSS_TYPE_MASK << IO_CONFIG_GNSS1_TYPE_OFFSET))) return false;
    if (((uint64_t)view->Receiver[CBOR_RECEIVER_ID_GNSS1].port * CBOR_BIND_LSB(((uint32_t)(IO_CONFIG_GNSS_SOURCE_MASK << IO_CONFIG_GNSS1_SOURCE_OFFSET)))) & ~(uint64_t)((uint32_t)(IO_CONFIG_GNSS_SOURCE_MASK << IO_CONFIG_GNSS1_SOURCE_OFFSET))) return false;
    if (((uint64_t)view->Receiver[CBOR_RECEIVER_ID_GNSS1].ppsPin * CBOR_BIND_LSB(((uint32_t)(IO_CFG_GNSS1_PPS_SOURCE_BITMASK)))) & ~(uint64_t)((uint32_t)(IO_CFG_GNSS1_PPS_SOURCE_BITMASK))) return false;
    if (view->Receiver[CBOR_RECEIVER_ID_GNSS1].configure > 1u) return false;
    if (view->Receiver[CBOR_RECEIVER_ID_GNSS1].fusionEnabled > 1u) return false;
    if (((uint64_t)view->Receiver[CBOR_RECEIVER_ID_GNSS2].type * CBOR_BIND_LSB(((uint32_t)(IO_CONFIG_GNSS_TYPE_MASK << IO_CONFIG_GNSS2_TYPE_OFFSET)))) & ~(uint64_t)((uint32_t)(IO_CONFIG_GNSS_TYPE_MASK << IO_CONFIG_GNSS2_TYPE_OFFSET))) return false;
    if (((uint64_t)view->Receiver[CBOR_RECEIVER_ID_GNSS2].port * CBOR_BIND_LSB(((uint32_t)(IO_CONFIG_GNSS_SOURCE_MASK << IO_CONFIG_GNSS2_SOURCE_OFFSET)))) & ~(uint64_t)((uint32_t)(IO_CONFIG_GNSS_SOURCE_MASK << IO_CONFIG_GNSS2_SOURCE_OFFSET))) return false;
    if (((uint64_t)view->Receiver[CBOR_RECEIVER_ID_GNSS2].ppsPin * CBOR_BIND_LSB(((uint32_t)(IO_CFG2_GNSS2_PPS_SOURCE_BITMASK)))) & ~(uint64_t)((uint32_t)(IO_CFG2_GNSS2_PPS_SOURCE_BITMASK))) return false;
    if (view->Receiver[CBOR_RECEIVER_ID_GNSS2].configure > 1u) return false;
    if (view->Receiver[CBOR_RECEIVER_ID_GNSS2].fusionEnabled > 1u) return false;
    if (view->Antennas.offsetEstimation > 1u) return false;
    if (((uint64_t)view->Rtk.mode * CBOR_BIND_LSB(((uint32_t)(RTK_CFG_BITS_ROVER_MODE_MASK)))) & ~(uint64_t)((uint32_t)(RTK_CFG_BITS_ROVER_MODE_MASK))) return false;
    if (view->Rtk.basePositionMoving > 1u) return false;
    if (view->Rtk.baseIdenticalToRover > 1u) return false;
    if (view->Rtk.passThrough > 1u) return false;
    if (view->Rtk.multipathMitigation > 1u) return false;
    if (view->Rtk.BaseOutput[CBOR_BASE_STREAM_GNSS1_UBLOX].ser0 > 1u) return false;
    if (view->Rtk.BaseOutput[CBOR_BASE_STREAM_GNSS1_UBLOX].ser1 > 1u) return false;
    if (view->Rtk.BaseOutput[CBOR_BASE_STREAM_GNSS1_UBLOX].ser2 > 1u) return false;
    if (view->Rtk.BaseOutput[CBOR_BASE_STREAM_GNSS1_UBLOX].usb > 1u) return false;
    if (view->Rtk.BaseOutput[CBOR_BASE_STREAM_GNSS1_RTCM3].ser0 > 1u) return false;
    if (view->Rtk.BaseOutput[CBOR_BASE_STREAM_GNSS1_RTCM3].ser1 > 1u) return false;
    if (view->Rtk.BaseOutput[CBOR_BASE_STREAM_GNSS1_RTCM3].ser2 > 1u) return false;
    if (view->Rtk.BaseOutput[CBOR_BASE_STREAM_GNSS1_RTCM3].usb > 1u) return false;
    if (view->Rtk.BaseOutput[CBOR_BASE_STREAM_GNSS2_UBLOX].ser0 > 1u) return false;
    if (view->Rtk.BaseOutput[CBOR_BASE_STREAM_GNSS2_UBLOX].ser1 > 1u) return false;
    if (view->Rtk.BaseOutput[CBOR_BASE_STREAM_GNSS2_UBLOX].ser2 > 1u) return false;
    if (view->Rtk.BaseOutput[CBOR_BASE_STREAM_GNSS2_UBLOX].usb > 1u) return false;
    if (view->Rtk.BaseOutput[CBOR_BASE_STREAM_GNSS2_RTCM3].ser0 > 1u) return false;
    if (view->Rtk.BaseOutput[CBOR_BASE_STREAM_GNSS2_RTCM3].ser1 > 1u) return false;
    if (view->Rtk.BaseOutput[CBOR_BASE_STREAM_GNSS2_RTCM3].ser2 > 1u) return false;
    if (view->Rtk.BaseOutput[CBOR_BASE_STREAM_GNSS2_RTCM3].usb > 1u) return false;
    dst->startupGnssDtMs = view->periodMs;
    dst->ioConfig2 = (dst->ioConfig2 & ~((uint32_t)(IO_CFG2_USE_GNSS2_AS_SOURCE))) | (((uint32_t)view->primaryReceiver * CBOR_BIND_LSB(((uint32_t)(IO_CFG2_USE_GNSS2_AS_SOURCE)))) & ((uint32_t)(IO_CFG2_USE_GNSS2_AS_SOURCE)));
    dst->gnssTimeSyncPeriodMs = view->timeSync.periodMs;
    dst->gnssTimeUserDelay = view->timeSync.userDelayS;
    dst->gnssSatSigConst = view->Receivers.satSignals;
    dst->gnssMinimumElevation = view->Receivers.minElevation;
    dst->gnssCn0Minimum = view->Receivers.cn0Min;
    dst->gnssCn0DynMinOffset = view->Receivers.cn0DynOffset;
    dst->ioConfig = (dst->ioConfig & ~((uint32_t)(IO_CONFIG_GNSS_TYPE_MASK << IO_CONFIG_GNSS1_TYPE_OFFSET))) | (((uint32_t)view->Receiver[CBOR_RECEIVER_ID_GNSS1].type * CBOR_BIND_LSB(((uint32_t)(IO_CONFIG_GNSS_TYPE_MASK << IO_CONFIG_GNSS1_TYPE_OFFSET)))) & ((uint32_t)(IO_CONFIG_GNSS_TYPE_MASK << IO_CONFIG_GNSS1_TYPE_OFFSET)));
    dst->ioConfig = (dst->ioConfig & ~((uint32_t)(IO_CONFIG_GNSS_SOURCE_MASK << IO_CONFIG_GNSS1_SOURCE_OFFSET))) | (((uint32_t)view->Receiver[CBOR_RECEIVER_ID_GNSS1].port * CBOR_BIND_LSB(((uint32_t)(IO_CONFIG_GNSS_SOURCE_MASK << IO_CONFIG_GNSS1_SOURCE_OFFSET)))) & ((uint32_t)(IO_CONFIG_GNSS_SOURCE_MASK << IO_CONFIG_GNSS1_SOURCE_OFFSET)));
    dst->ioConfig = (dst->ioConfig & ~((uint32_t)(IO_CFG_GNSS1_PPS_SOURCE_BITMASK))) | (((uint32_t)view->Receiver[CBOR_RECEIVER_ID_GNSS1].ppsPin * CBOR_BIND_LSB(((uint32_t)(IO_CFG_GNSS1_PPS_SOURCE_BITMASK)))) & ((uint32_t)(IO_CFG_GNSS1_PPS_SOURCE_BITMASK)));
    if (view->Receiver[CBOR_RECEIVER_ID_GNSS1].configure) dst->ioConfig &= ~((uint32_t)(IO_CONFIG_GNSS1_NO_INIT)); else dst->ioConfig |= ((uint32_t)(IO_CONFIG_GNSS1_NO_INIT));
    if (view->Receiver[CBOR_RECEIVER_ID_GNSS1].fusionEnabled) dst->sysCfgBits &= ~((uint32_t)(SYS_CFG_BITS_DISABLE_GNSS1_FUSION)); else dst->sysCfgBits |= ((uint32_t)(SYS_CFG_BITS_DISABLE_GNSS1_FUSION));
    dst->ioConfig = (dst->ioConfig & ~((uint32_t)(IO_CONFIG_GNSS_TYPE_MASK << IO_CONFIG_GNSS2_TYPE_OFFSET))) | (((uint32_t)view->Receiver[CBOR_RECEIVER_ID_GNSS2].type * CBOR_BIND_LSB(((uint32_t)(IO_CONFIG_GNSS_TYPE_MASK << IO_CONFIG_GNSS2_TYPE_OFFSET)))) & ((uint32_t)(IO_CONFIG_GNSS_TYPE_MASK << IO_CONFIG_GNSS2_TYPE_OFFSET)));
    dst->ioConfig = (dst->ioConfig & ~((uint32_t)(IO_CONFIG_GNSS_SOURCE_MASK << IO_CONFIG_GNSS2_SOURCE_OFFSET))) | (((uint32_t)view->Receiver[CBOR_RECEIVER_ID_GNSS2].port * CBOR_BIND_LSB(((uint32_t)(IO_CONFIG_GNSS_SOURCE_MASK << IO_CONFIG_GNSS2_SOURCE_OFFSET)))) & ((uint32_t)(IO_CONFIG_GNSS_SOURCE_MASK << IO_CONFIG_GNSS2_SOURCE_OFFSET)));
    dst->ioConfig2 = (dst->ioConfig2 & ~((uint32_t)(IO_CFG2_GNSS2_PPS_SOURCE_BITMASK))) | (((uint32_t)view->Receiver[CBOR_RECEIVER_ID_GNSS2].ppsPin * CBOR_BIND_LSB(((uint32_t)(IO_CFG2_GNSS2_PPS_SOURCE_BITMASK)))) & ((uint32_t)(IO_CFG2_GNSS2_PPS_SOURCE_BITMASK)));
    if (view->Receiver[CBOR_RECEIVER_ID_GNSS2].configure) dst->ioConfig &= ~((uint32_t)(IO_CONFIG_GNSS2_NO_INIT)); else dst->ioConfig |= ((uint32_t)(IO_CONFIG_GNSS2_NO_INIT));
    if (view->Receiver[CBOR_RECEIVER_ID_GNSS2].fusionEnabled) dst->sysCfgBits &= ~((uint32_t)(SYS_CFG_BITS_DISABLE_GNSS2_FUSION)); else dst->sysCfgBits |= ((uint32_t)(SYS_CFG_BITS_DISABLE_GNSS2_FUSION));
    if (view->Antennas.offsetEstimation) dst->sysCfgBits |= ((uint32_t)(SYS_CFG_BITS_ENABLE_GNSS_ANTENNA_OFFSET_ESTIMATION)); else dst->sysCfgBits &= ~((uint32_t)(SYS_CFG_BITS_ENABLE_GNSS_ANTENNA_OFFSET_ESTIMATION));
    memcpy(dst->gnss1AntOffset, view->Antenna[CBOR_ANTENNA_ID_ANT1].offset, sizeof(view->Antenna[CBOR_ANTENNA_ID_ANT1].offset));
    memcpy(dst->gnss2AntOffset, view->Antenna[CBOR_ANTENNA_ID_ANT2].offset, sizeof(view->Antenna[CBOR_ANTENNA_ID_ANT2].offset));
    dst->RTKCfgBits = (dst->RTKCfgBits & ~((uint32_t)(RTK_CFG_BITS_ROVER_MODE_MASK))) | (((uint32_t)view->Rtk.mode * CBOR_BIND_LSB(((uint32_t)(RTK_CFG_BITS_ROVER_MODE_MASK)))) & ((uint32_t)(RTK_CFG_BITS_ROVER_MODE_MASK)));
    if (view->Rtk.basePositionMoving) dst->RTKCfgBits |= ((uint32_t)(RTK_CFG_BITS_BASE_POS_MOVING)); else dst->RTKCfgBits &= ~((uint32_t)(RTK_CFG_BITS_BASE_POS_MOVING));
    if (view->Rtk.baseIdenticalToRover) dst->RTKCfgBits |= ((uint32_t)(RTK_CFG_BITS_RTK_BASE_IS_IDENTICAL_TO_ROVER)); else dst->RTKCfgBits &= ~((uint32_t)(RTK_CFG_BITS_RTK_BASE_IS_IDENTICAL_TO_ROVER));
    if (view->Rtk.passThrough) dst->RTKCfgBits |= ((uint32_t)(RTK_CFG_BITS_GNSS_PORT_PASS_THROUGH)); else dst->RTKCfgBits &= ~((uint32_t)(RTK_CFG_BITS_GNSS_PORT_PASS_THROUGH));
    if (view->Rtk.multipathMitigation) dst->RTKCfgBits |= ((uint32_t)(RTK_CFG_BITS_MULTIPATH_MITIGATION)); else dst->RTKCfgBits &= ~((uint32_t)(RTK_CFG_BITS_MULTIPATH_MITIGATION));
    if (view->Rtk.BaseOutput[CBOR_BASE_STREAM_GNSS1_UBLOX].ser0) dst->RTKCfgBits |= ((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS1_UBLOX_SER0)); else dst->RTKCfgBits &= ~((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS1_UBLOX_SER0));
    if (view->Rtk.BaseOutput[CBOR_BASE_STREAM_GNSS1_UBLOX].ser1) dst->RTKCfgBits |= ((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS1_UBLOX_SER1)); else dst->RTKCfgBits &= ~((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS1_UBLOX_SER1));
    if (view->Rtk.BaseOutput[CBOR_BASE_STREAM_GNSS1_UBLOX].ser2) dst->RTKCfgBits |= ((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS1_UBLOX_SER2)); else dst->RTKCfgBits &= ~((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS1_UBLOX_SER2));
    if (view->Rtk.BaseOutput[CBOR_BASE_STREAM_GNSS1_UBLOX].usb) dst->RTKCfgBits |= ((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS1_UBLOX_USB)); else dst->RTKCfgBits &= ~((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS1_UBLOX_USB));
    if (view->Rtk.BaseOutput[CBOR_BASE_STREAM_GNSS1_RTCM3].ser0) dst->RTKCfgBits |= ((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS1_RTCM3_SER0)); else dst->RTKCfgBits &= ~((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS1_RTCM3_SER0));
    if (view->Rtk.BaseOutput[CBOR_BASE_STREAM_GNSS1_RTCM3].ser1) dst->RTKCfgBits |= ((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS1_RTCM3_SER1)); else dst->RTKCfgBits &= ~((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS1_RTCM3_SER1));
    if (view->Rtk.BaseOutput[CBOR_BASE_STREAM_GNSS1_RTCM3].ser2) dst->RTKCfgBits |= ((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS1_RTCM3_SER2)); else dst->RTKCfgBits &= ~((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS1_RTCM3_SER2));
    if (view->Rtk.BaseOutput[CBOR_BASE_STREAM_GNSS1_RTCM3].usb) dst->RTKCfgBits |= ((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS1_RTCM3_USB)); else dst->RTKCfgBits &= ~((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS1_RTCM3_USB));
    if (view->Rtk.BaseOutput[CBOR_BASE_STREAM_GNSS2_UBLOX].ser0) dst->RTKCfgBits |= ((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS2_UBLOX_SER0)); else dst->RTKCfgBits &= ~((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS2_UBLOX_SER0));
    if (view->Rtk.BaseOutput[CBOR_BASE_STREAM_GNSS2_UBLOX].ser1) dst->RTKCfgBits |= ((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS2_UBLOX_SER1)); else dst->RTKCfgBits &= ~((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS2_UBLOX_SER1));
    if (view->Rtk.BaseOutput[CBOR_BASE_STREAM_GNSS2_UBLOX].ser2) dst->RTKCfgBits |= ((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS2_UBLOX_SER2)); else dst->RTKCfgBits &= ~((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS2_UBLOX_SER2));
    if (view->Rtk.BaseOutput[CBOR_BASE_STREAM_GNSS2_UBLOX].usb) dst->RTKCfgBits |= ((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS2_UBLOX_USB)); else dst->RTKCfgBits &= ~((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS2_UBLOX_USB));
    if (view->Rtk.BaseOutput[CBOR_BASE_STREAM_GNSS2_RTCM3].ser0) dst->RTKCfgBits |= ((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS2_RTCM3_SER0)); else dst->RTKCfgBits &= ~((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS2_RTCM3_SER0));
    if (view->Rtk.BaseOutput[CBOR_BASE_STREAM_GNSS2_RTCM3].ser1) dst->RTKCfgBits |= ((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS2_RTCM3_SER1)); else dst->RTKCfgBits &= ~((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS2_RTCM3_SER1));
    if (view->Rtk.BaseOutput[CBOR_BASE_STREAM_GNSS2_RTCM3].ser2) dst->RTKCfgBits |= ((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS2_RTCM3_SER2)); else dst->RTKCfgBits &= ~((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS2_RTCM3_SER2));
    if (view->Rtk.BaseOutput[CBOR_BASE_STREAM_GNSS2_RTCM3].usb) dst->RTKCfgBits |= ((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS2_RTCM3_USB)); else dst->RTKCfgBits &= ~((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS2_RTCM3_USB));
    return true;
}

/**
 * Fills the STATE keys held in nvm_flash_cfg_t into the view.
 *
 * Keys of STATE that live elsewhere are left as they are, so the view of a domain whose keys live in
 * several structs is filled by each of their mappings in turn.
 *
 * @param view  [out] the view
 * @param src   the nvm_flash_cfg_t the keys are read from
 */
void cbor_flash_config_to_state(cbor_state_t *view, const nvm_flash_cfg_t *src)
{
    memcpy(view->lastPosition.lla, src->lastLla, sizeof(view->lastPosition.lla));
    view->lastPosition.timeOfWeekMs = src->lastLlaTimeOfWeekMs;
    view->lastPosition.week = src->lastLlaWeek;
}

/**
 * Checks that every STATE value held in nvm_flash_cfg_t fits its home (a packed field or a bool).
 *
 * A domain whose keys live in several structs is written all or nothing by checking each of their mappings
 * before writing any of them.
 *
 * @param view  the view
 * @return true if cbor_flash_config_from_state() would write the view
 */
bool cbor_flash_config_fits_state(const cbor_state_t *view)
{
    (void)view;
    return true;
}

/**
 * Writes the STATE keys held in nvm_flash_cfg_t from the view, all or nothing.
 *
 * Bits and members of nvm_flash_cfg_t that no key covers are kept. Nothing is written unless every value fits its
 * home (a packed field or a bool).
 *
 * @param dst   the nvm_flash_cfg_t the keys are written to
 * @param view  the view
 * @return false, writing nothing, if a value does not fit its home
 */
bool cbor_flash_config_from_state(nvm_flash_cfg_t *dst, const cbor_state_t *view)
{
    memcpy(dst->lastLla, view->lastPosition.lla, sizeof(view->lastPosition.lla));
    dst->lastLlaTimeOfWeekMs = view->lastPosition.timeOfWeekMs;
    dst->lastLlaWeek = view->lastPosition.week;
    return true;
}

/* Compile-time checks of every source the registry names */
_Static_assert(_Generic(((nvm_flash_cfg_t *)0)->startupNavDtMs, uint32_t: 1, default: 0), "CONFIG_SYSTEM.navPeriodMs: type differs from its member");
_Static_assert(_Generic(((nvm_flash_cfg_t *)0)->startupImuOdr, uint32_t: 1, default: 0), "CONFIG_SYSTEM.imuOdr: type differs from its member");
_Static_assert(((uint32_t)(SYS_CFG_BITS_DISABLE_LEDS)) != 0u, "CONFIG_SYSTEM.ledsEnabled: mask is zero");
_Static_assert(sizeof(((nvm_flash_cfg_t *)0)->sysCfgBits) <= 4, "CONFIG_SYSTEM.ledsEnabled: a mask needs an integer member of at most 32 bits");
_Static_assert((((uint32_t)(SYS_CFG_BITS_DISABLE_LEDS)) >> 1 >> (8 * sizeof(((nvm_flash_cfg_t *)0)->sysCfgBits) - 1)) == 0u, "CONFIG_SYSTEM.ledsEnabled: mask is wider than its member");
_Static_assert((((uint32_t)(SYS_CFG_BITS_DISABLE_LEDS)) & (((uint32_t)(SYS_CFG_BITS_DISABLE_LEDS)) - 1u)) == 0u, "CONFIG_SYSTEM.ledsEnabled: a bool needs a single-bit mask");
_Static_assert(((uint32_t)(SYS_CFG_BITS_BOR_THRESHOLD_MASK)) != 0u, "CONFIG_SYSTEM.brownOutLevel: mask is zero");
_Static_assert(sizeof(((nvm_flash_cfg_t *)0)->sysCfgBits) <= 4, "CONFIG_SYSTEM.brownOutLevel: a mask needs an integer member of at most 32 bits");
_Static_assert((((uint32_t)(SYS_CFG_BITS_BOR_THRESHOLD_MASK)) >> 1 >> (8 * sizeof(((nvm_flash_cfg_t *)0)->sysCfgBits) - 1)) == 0u, "CONFIG_SYSTEM.brownOutLevel: mask is wider than its member");
_Static_assert(((uint32_t)(SYS_CFG_BITS_BOR_THRESHOLD_MASK)) / CBOR_BIND_LSB(((uint32_t)(SYS_CFG_BITS_BOR_THRESHOLD_MASK))) <= UINT8_MAX, "CONFIG_SYSTEM.brownOutLevel: mask holds more than the key's type");
_Static_assert(((uint32_t)(SYS_CFG_BITS_DISABLE_AUTO_BIT_ON_STARTUP)) != 0u, "CONFIG_SYSTEM.bitOnStartup: mask is zero");
_Static_assert(sizeof(((nvm_flash_cfg_t *)0)->sysCfgBits) <= 4, "CONFIG_SYSTEM.bitOnStartup: a mask needs an integer member of at most 32 bits");
_Static_assert((((uint32_t)(SYS_CFG_BITS_DISABLE_AUTO_BIT_ON_STARTUP)) >> 1 >> (8 * sizeof(((nvm_flash_cfg_t *)0)->sysCfgBits) - 1)) == 0u, "CONFIG_SYSTEM.bitOnStartup: mask is wider than its member");
_Static_assert((((uint32_t)(SYS_CFG_BITS_DISABLE_AUTO_BIT_ON_STARTUP)) & (((uint32_t)(SYS_CFG_BITS_DISABLE_AUTO_BIT_ON_STARTUP)) - 1u)) == 0u, "CONFIG_SYSTEM.bitOnStartup: a bool needs a single-bit mask");
_Static_assert(((uint32_t)(PLATFORM_CFG_TYPE_MASK)) != 0u, "CONFIG_SYSTEM.platform.type: mask is zero");
_Static_assert(sizeof(((nvm_flash_cfg_t *)0)->platformConfig) <= 4, "CONFIG_SYSTEM.platform.type: a mask needs an integer member of at most 32 bits");
_Static_assert((((uint32_t)(PLATFORM_CFG_TYPE_MASK)) >> 1 >> (8 * sizeof(((nvm_flash_cfg_t *)0)->platformConfig) - 1)) == 0u, "CONFIG_SYSTEM.platform.type: mask is wider than its member");
_Static_assert(((uint32_t)(PLATFORM_CFG_TYPE_MASK)) / CBOR_BIND_LSB(((uint32_t)(PLATFORM_CFG_TYPE_MASK))) <= UINT8_MAX, "CONFIG_SYSTEM.platform.type: mask holds more than the key's type");
_Static_assert(((uint32_t)(PLATFORM_CFG_PRESET_MASK)) != 0u, "CONFIG_SYSTEM.platform.preset: mask is zero");
_Static_assert(sizeof(((nvm_flash_cfg_t *)0)->platformConfig) <= 4, "CONFIG_SYSTEM.platform.preset: a mask needs an integer member of at most 32 bits");
_Static_assert((((uint32_t)(PLATFORM_CFG_PRESET_MASK)) >> 1 >> (8 * sizeof(((nvm_flash_cfg_t *)0)->platformConfig) - 1)) == 0u, "CONFIG_SYSTEM.platform.preset: mask is wider than its member");
_Static_assert(((uint32_t)(PLATFORM_CFG_PRESET_MASK)) / CBOR_BIND_LSB(((uint32_t)(PLATFORM_CFG_PRESET_MASK))) <= UINT8_MAX, "CONFIG_SYSTEM.platform.preset: mask holds more than the key's type");
_Static_assert(((uint32_t)(PLATFORM_CFG_RUG_IOEXP_BIT_MASK)) != 0u, "CONFIG_SYSTEM.platform.ioExpanderBits: mask is zero");
_Static_assert(sizeof(((nvm_flash_cfg_t *)0)->platformConfig) <= 4, "CONFIG_SYSTEM.platform.ioExpanderBits: a mask needs an integer member of at most 32 bits");
_Static_assert((((uint32_t)(PLATFORM_CFG_RUG_IOEXP_BIT_MASK)) >> 1 >> (8 * sizeof(((nvm_flash_cfg_t *)0)->platformConfig) - 1)) == 0u, "CONFIG_SYSTEM.platform.ioExpanderBits: mask is wider than its member");
_Static_assert(((uint32_t)(PLATFORM_CFG_RUG_IOEXP_BIT_MASK)) / CBOR_BIND_LSB(((uint32_t)(PLATFORM_CFG_RUG_IOEXP_BIT_MASK))) <= UINT8_MAX, "CONFIG_SYSTEM.platform.ioExpanderBits: mask holds more than the key's type");
_Static_assert(_Generic(((nvm_flash_cfg_t *)0)->debug, uint8_t: 1, default: 0), "CONFIG_SYSTEM.debug: type differs from its member");
_Static_assert(sizeof(((nvm_flash_cfg_t *)0)->insRotation) == sizeof(((cbor_config_ins_t *)0)->rotation), "CONFIG_INS.rotation: array size differs from its member");
_Static_assert(_Generic(((nvm_flash_cfg_t *)0)->insRotation[0], float: 1, default: 0), "CONFIG_INS.rotation: element type differs from its member");
_Static_assert(sizeof(((nvm_flash_cfg_t *)0)->insOffset) == sizeof(((cbor_config_ins_t *)0)->offset), "CONFIG_INS.offset: array size differs from its member");
_Static_assert(_Generic(((nvm_flash_cfg_t *)0)->insOffset[0], float: 1, default: 0), "CONFIG_INS.offset: element type differs from its member");
_Static_assert(sizeof(((nvm_flash_cfg_t *)0)->refLla) == sizeof(((cbor_config_ins_t *)0)->nedOrigin), "CONFIG_INS.nedOrigin: array size differs from its member");
_Static_assert(_Generic(((nvm_flash_cfg_t *)0)->refLla[0], double: 1, default: 0), "CONFIG_INS.nedOrigin: element type differs from its member");
_Static_assert(_Generic(((nvm_flash_cfg_t *)0)->dynamicModel, uint8_t: 1, default: 0), "CONFIG_INS.dynamicModel: type differs from its member");
_Static_assert(((uint32_t)(SYS_CFG_BITS_DISABLE_INS_EKF)) != 0u, "CONFIG_INS.ekfEnabled: mask is zero");
_Static_assert(sizeof(((nvm_flash_cfg_t *)0)->sysCfgBits) <= 4, "CONFIG_INS.ekfEnabled: a mask needs an integer member of at most 32 bits");
_Static_assert((((uint32_t)(SYS_CFG_BITS_DISABLE_INS_EKF)) >> 1 >> (8 * sizeof(((nvm_flash_cfg_t *)0)->sysCfgBits) - 1)) == 0u, "CONFIG_INS.ekfEnabled: mask is wider than its member");
_Static_assert((((uint32_t)(SYS_CFG_BITS_DISABLE_INS_EKF)) & (((uint32_t)(SYS_CFG_BITS_DISABLE_INS_EKF)) - 1u)) == 0u, "CONFIG_INS.ekfEnabled: a bool needs a single-bit mask");
_Static_assert(_Generic(((nvm_flash_cfg_t *)0)->deadReckoningTimeout, uint8_t: 1, default: 0), "CONFIG_INS.deadReckoningTimeoutS: type differs from its member");
_Static_assert(_Generic(((nvm_flash_cfg_t *)0)->lastLlaUpdateDistance, float: 1, default: 0), "CONFIG_INS.lastPositionUpdateDistanceM: type differs from its member");
_Static_assert(((uint32_t)(SYS_CFG_USE_REFERENCE_IMU_IN_EKF)) != 0u, "CONFIG_INS.referenceImu: mask is zero");
_Static_assert(sizeof(((nvm_flash_cfg_t *)0)->sysCfgBits) <= 4, "CONFIG_INS.referenceImu: a mask needs an integer member of at most 32 bits");
_Static_assert((((uint32_t)(SYS_CFG_USE_REFERENCE_IMU_IN_EKF)) >> 1 >> (8 * sizeof(((nvm_flash_cfg_t *)0)->sysCfgBits) - 1)) == 0u, "CONFIG_INS.referenceImu: mask is wider than its member");
_Static_assert((((uint32_t)(SYS_CFG_USE_REFERENCE_IMU_IN_EKF)) & (((uint32_t)(SYS_CFG_USE_REFERENCE_IMU_IN_EKF)) - 1u)) == 0u, "CONFIG_INS.referenceImu: a bool needs a single-bit mask");
_Static_assert(((uint32_t)(SYS_CFG_EKF_REF_POINT_STATIONARY_ON_STROBE_INPUT)) != 0u, "CONFIG_INS.stationaryOnStrobe: mask is zero");
_Static_assert(sizeof(((nvm_flash_cfg_t *)0)->sysCfgBits) <= 4, "CONFIG_INS.stationaryOnStrobe: a mask needs an integer member of at most 32 bits");
_Static_assert((((uint32_t)(SYS_CFG_EKF_REF_POINT_STATIONARY_ON_STROBE_INPUT)) >> 1 >> (8 * sizeof(((nvm_flash_cfg_t *)0)->sysCfgBits) - 1)) == 0u, "CONFIG_INS.stationaryOnStrobe: mask is wider than its member");
_Static_assert((((uint32_t)(SYS_CFG_EKF_REF_POINT_STATIONARY_ON_STROBE_INPUT)) & (((uint32_t)(SYS_CFG_EKF_REF_POINT_STATIONARY_ON_STROBE_INPUT)) - 1u)) == 0u, "CONFIG_INS.stationaryOnStrobe: a bool needs a single-bit mask");
_Static_assert(((uint32_t)(SYS_CFG_BITS_DISABLE_MAGNETOMETER_FUSION)) != 0u, "CONFIG_INS.fusion.mag: mask is zero");
_Static_assert(sizeof(((nvm_flash_cfg_t *)0)->sysCfgBits) <= 4, "CONFIG_INS.fusion.mag: a mask needs an integer member of at most 32 bits");
_Static_assert((((uint32_t)(SYS_CFG_BITS_DISABLE_MAGNETOMETER_FUSION)) >> 1 >> (8 * sizeof(((nvm_flash_cfg_t *)0)->sysCfgBits) - 1)) == 0u, "CONFIG_INS.fusion.mag: mask is wider than its member");
_Static_assert((((uint32_t)(SYS_CFG_BITS_DISABLE_MAGNETOMETER_FUSION)) & (((uint32_t)(SYS_CFG_BITS_DISABLE_MAGNETOMETER_FUSION)) - 1u)) == 0u, "CONFIG_INS.fusion.mag: a bool needs a single-bit mask");
_Static_assert(((uint32_t)(SYS_CFG_BITS_DISABLE_BAROMETER_FUSION)) != 0u, "CONFIG_INS.fusion.baro: mask is zero");
_Static_assert(sizeof(((nvm_flash_cfg_t *)0)->sysCfgBits) <= 4, "CONFIG_INS.fusion.baro: a mask needs an integer member of at most 32 bits");
_Static_assert((((uint32_t)(SYS_CFG_BITS_DISABLE_BAROMETER_FUSION)) >> 1 >> (8 * sizeof(((nvm_flash_cfg_t *)0)->sysCfgBits) - 1)) == 0u, "CONFIG_INS.fusion.baro: mask is wider than its member");
_Static_assert((((uint32_t)(SYS_CFG_BITS_DISABLE_BAROMETER_FUSION)) & (((uint32_t)(SYS_CFG_BITS_DISABLE_BAROMETER_FUSION)) - 1u)) == 0u, "CONFIG_INS.fusion.baro: a bool needs a single-bit mask");
_Static_assert(((uint32_t)(SYS_CFG_BITS_DISABLE_WHEEL_ENCODER_FUSION)) != 0u, "CONFIG_INS.fusion.wheel: mask is zero");
_Static_assert(sizeof(((nvm_flash_cfg_t *)0)->sysCfgBits) <= 4, "CONFIG_INS.fusion.wheel: a mask needs an integer member of at most 32 bits");
_Static_assert((((uint32_t)(SYS_CFG_BITS_DISABLE_WHEEL_ENCODER_FUSION)) >> 1 >> (8 * sizeof(((nvm_flash_cfg_t *)0)->sysCfgBits) - 1)) == 0u, "CONFIG_INS.fusion.wheel: mask is wider than its member");
_Static_assert((((uint32_t)(SYS_CFG_BITS_DISABLE_WHEEL_ENCODER_FUSION)) & (((uint32_t)(SYS_CFG_BITS_DISABLE_WHEEL_ENCODER_FUSION)) - 1u)) == 0u, "CONFIG_INS.fusion.wheel: a bool needs a single-bit mask");
_Static_assert(sizeof(((nvm_flash_cfg_t *)0)->zeroVelRotation) == sizeof(((cbor_config_ins_t *)0)->zeroVelocity.rotation), "CONFIG_INS.zeroVelocity.rotation: array size differs from its member");
_Static_assert(_Generic(((nvm_flash_cfg_t *)0)->zeroVelRotation[0], float: 1, default: 0), "CONFIG_INS.zeroVelocity.rotation: element type differs from its member");
_Static_assert(sizeof(((nvm_flash_cfg_t *)0)->zeroVelOffset) == sizeof(((cbor_config_ins_t *)0)->zeroVelocity.offset), "CONFIG_INS.zeroVelocity.offset: array size differs from its member");
_Static_assert(_Generic(((nvm_flash_cfg_t *)0)->zeroVelOffset[0], float: 1, default: 0), "CONFIG_INS.zeroVelocity.offset: element type differs from its member");
_Static_assert(((uint32_t)(SYS_CFG_BITS_DISABLE_AUTO_ZERO_VELOCITY_UPDATES)) != 0u, "CONFIG_INS.zeroVelocity.autoZupt: mask is zero");
_Static_assert(sizeof(((nvm_flash_cfg_t *)0)->sysCfgBits) <= 4, "CONFIG_INS.zeroVelocity.autoZupt: a mask needs an integer member of at most 32 bits");
_Static_assert((((uint32_t)(SYS_CFG_BITS_DISABLE_AUTO_ZERO_VELOCITY_UPDATES)) >> 1 >> (8 * sizeof(((nvm_flash_cfg_t *)0)->sysCfgBits) - 1)) == 0u, "CONFIG_INS.zeroVelocity.autoZupt: mask is wider than its member");
_Static_assert((((uint32_t)(SYS_CFG_BITS_DISABLE_AUTO_ZERO_VELOCITY_UPDATES)) & (((uint32_t)(SYS_CFG_BITS_DISABLE_AUTO_ZERO_VELOCITY_UPDATES)) - 1u)) == 0u, "CONFIG_INS.zeroVelocity.autoZupt: a bool needs a single-bit mask");
_Static_assert(((uint32_t)(SYS_CFG_BITS_DISABLE_AUTO_ZERO_ANGULAR_RATE_UPDATES)) != 0u, "CONFIG_INS.zeroVelocity.autoZaru: mask is zero");
_Static_assert(sizeof(((nvm_flash_cfg_t *)0)->sysCfgBits) <= 4, "CONFIG_INS.zeroVelocity.autoZaru: a mask needs an integer member of at most 32 bits");
_Static_assert((((uint32_t)(SYS_CFG_BITS_DISABLE_AUTO_ZERO_ANGULAR_RATE_UPDATES)) >> 1 >> (8 * sizeof(((nvm_flash_cfg_t *)0)->sysCfgBits) - 1)) == 0u, "CONFIG_INS.zeroVelocity.autoZaru: mask is wider than its member");
_Static_assert((((uint32_t)(SYS_CFG_BITS_DISABLE_AUTO_ZERO_ANGULAR_RATE_UPDATES)) & (((uint32_t)(SYS_CFG_BITS_DISABLE_AUTO_ZERO_ANGULAR_RATE_UPDATES)) - 1u)) == 0u, "CONFIG_INS.zeroVelocity.autoZaru: a bool needs a single-bit mask");
_Static_assert(((uint32_t)(IMU_SHOCK_OPTIONS_ENABLE)) != 0u, "CONFIG_INS.shock.enabled: mask is zero");
_Static_assert(sizeof(((nvm_flash_cfg_t *)0)->imuShockOptions) <= 4, "CONFIG_INS.shock.enabled: a mask needs an integer member of at most 32 bits");
_Static_assert((((uint32_t)(IMU_SHOCK_OPTIONS_ENABLE)) >> 1 >> (8 * sizeof(((nvm_flash_cfg_t *)0)->imuShockOptions) - 1)) == 0u, "CONFIG_INS.shock.enabled: mask is wider than its member");
_Static_assert((((uint32_t)(IMU_SHOCK_OPTIONS_ENABLE)) & (((uint32_t)(IMU_SHOCK_OPTIONS_ENABLE)) - 1u)) == 0u, "CONFIG_INS.shock.enabled: a bool needs a single-bit mask");
_Static_assert(((uint32_t)(IMU_SHOCK_OPTIONS_FAST_RECOVERY)) != 0u, "CONFIG_INS.shock.fastRecovery: mask is zero");
_Static_assert(sizeof(((nvm_flash_cfg_t *)0)->imuShockOptions) <= 4, "CONFIG_INS.shock.fastRecovery: a mask needs an integer member of at most 32 bits");
_Static_assert((((uint32_t)(IMU_SHOCK_OPTIONS_FAST_RECOVERY)) >> 1 >> (8 * sizeof(((nvm_flash_cfg_t *)0)->imuShockOptions) - 1)) == 0u, "CONFIG_INS.shock.fastRecovery: mask is wider than its member");
_Static_assert((((uint32_t)(IMU_SHOCK_OPTIONS_FAST_RECOVERY)) & (((uint32_t)(IMU_SHOCK_OPTIONS_FAST_RECOVERY)) - 1u)) == 0u, "CONFIG_INS.shock.fastRecovery: a bool needs a single-bit mask");
_Static_assert(_Generic(((nvm_flash_cfg_t *)0)->imuShockDetectLatencyMs, uint8_t: 1, default: 0), "CONFIG_INS.shock.detectLatencyMs: type differs from its member");
_Static_assert(_Generic(((nvm_flash_cfg_t *)0)->imuShockRejectLatchMs, uint8_t: 1, default: 0), "CONFIG_INS.shock.rejectLatchMs: type differs from its member");
_Static_assert(_Generic(((nvm_flash_cfg_t *)0)->imuShockDeltaAccHighThreshold, uint8_t: 1, default: 0), "CONFIG_INS.shock.deltaAccHigh: type differs from its member");
_Static_assert(_Generic(((nvm_flash_cfg_t *)0)->imuShockDeltaAccLowThreshold, uint8_t: 1, default: 0), "CONFIG_INS.shock.deltaAccLow: type differs from its member");
_Static_assert(_Generic(((nvm_flash_cfg_t *)0)->imuShockDeltaGyroHighThreshold, uint8_t: 1, default: 0), "CONFIG_INS.shock.deltaGyroHigh: type differs from its member");
_Static_assert(_Generic(((nvm_flash_cfg_t *)0)->imuShockDeltaGyroLowThreshold, uint8_t: 1, default: 0), "CONFIG_INS.shock.deltaGyroLow: type differs from its member");
_Static_assert(((uint32_t)(SENSOR_CFG_IMU_FAULT_DETECT_GYR)) != 0u, "CONFIG_INS.faultDetect.gyr: mask is zero");
_Static_assert(sizeof(((nvm_flash_cfg_t *)0)->sensorConfig) <= 4, "CONFIG_INS.faultDetect.gyr: a mask needs an integer member of at most 32 bits");
_Static_assert((((uint32_t)(SENSOR_CFG_IMU_FAULT_DETECT_GYR)) >> 1 >> (8 * sizeof(((nvm_flash_cfg_t *)0)->sensorConfig) - 1)) == 0u, "CONFIG_INS.faultDetect.gyr: mask is wider than its member");
_Static_assert((((uint32_t)(SENSOR_CFG_IMU_FAULT_DETECT_GYR)) & (((uint32_t)(SENSOR_CFG_IMU_FAULT_DETECT_GYR)) - 1u)) == 0u, "CONFIG_INS.faultDetect.gyr: a bool needs a single-bit mask");
_Static_assert(((uint32_t)(SENSOR_CFG_IMU_FAULT_DETECT_ACC)) != 0u, "CONFIG_INS.faultDetect.acc: mask is zero");
_Static_assert(sizeof(((nvm_flash_cfg_t *)0)->sensorConfig) <= 4, "CONFIG_INS.faultDetect.acc: a mask needs an integer member of at most 32 bits");
_Static_assert((((uint32_t)(SENSOR_CFG_IMU_FAULT_DETECT_ACC)) >> 1 >> (8 * sizeof(((nvm_flash_cfg_t *)0)->sensorConfig) - 1)) == 0u, "CONFIG_INS.faultDetect.acc: mask is wider than its member");
_Static_assert((((uint32_t)(SENSOR_CFG_IMU_FAULT_DETECT_ACC)) & (((uint32_t)(SENSOR_CFG_IMU_FAULT_DETECT_ACC)) - 1u)) == 0u, "CONFIG_INS.faultDetect.acc: a bool needs a single-bit mask");
_Static_assert(_Generic(((nvm_flash_cfg_t *)0)->imuRejectThreshGyroLow, uint8_t: 1, default: 0), "CONFIG_INS.faultDetect.gyroRejectThreshLow: type differs from its member");
_Static_assert(_Generic(((nvm_flash_cfg_t *)0)->imuRejectThreshGyroHigh, uint8_t: 1, default: 0), "CONFIG_INS.faultDetect.gyroRejectThreshHigh: type differs from its member");
_Static_assert(((uint32_t)(SYS_CFG_BITS_ENABLE_MAG_CONTINUOUS_CAL)) != 0u, "CONFIG_MAG.continuousCal: mask is zero");
_Static_assert(sizeof(((nvm_flash_cfg_t *)0)->sysCfgBits) <= 4, "CONFIG_MAG.continuousCal: a mask needs an integer member of at most 32 bits");
_Static_assert((((uint32_t)(SYS_CFG_BITS_ENABLE_MAG_CONTINUOUS_CAL)) >> 1 >> (8 * sizeof(((nvm_flash_cfg_t *)0)->sysCfgBits) - 1)) == 0u, "CONFIG_MAG.continuousCal: mask is wider than its member");
_Static_assert((((uint32_t)(SYS_CFG_BITS_ENABLE_MAG_CONTINUOUS_CAL)) & (((uint32_t)(SYS_CFG_BITS_ENABLE_MAG_CONTINUOUS_CAL)) - 1u)) == 0u, "CONFIG_MAG.continuousCal: a bool needs a single-bit mask");
_Static_assert(((uint32_t)(SYS_CFG_BITS_AUTO_MAG_RECAL)) != 0u, "CONFIG_MAG.autoRecal: mask is zero");
_Static_assert(sizeof(((nvm_flash_cfg_t *)0)->sysCfgBits) <= 4, "CONFIG_MAG.autoRecal: a mask needs an integer member of at most 32 bits");
_Static_assert((((uint32_t)(SYS_CFG_BITS_AUTO_MAG_RECAL)) >> 1 >> (8 * sizeof(((nvm_flash_cfg_t *)0)->sysCfgBits) - 1)) == 0u, "CONFIG_MAG.autoRecal: mask is wider than its member");
_Static_assert((((uint32_t)(SYS_CFG_BITS_AUTO_MAG_RECAL)) & (((uint32_t)(SYS_CFG_BITS_AUTO_MAG_RECAL)) - 1u)) == 0u, "CONFIG_MAG.autoRecal: a bool needs a single-bit mask");
_Static_assert(((uint32_t)(SYS_CFG_BITS_MAG_RECAL_MODE_MASK)) != 0u, "CONFIG_MAG.recalMode: mask is zero");
_Static_assert(sizeof(((nvm_flash_cfg_t *)0)->sysCfgBits) <= 4, "CONFIG_MAG.recalMode: a mask needs an integer member of at most 32 bits");
_Static_assert((((uint32_t)(SYS_CFG_BITS_MAG_RECAL_MODE_MASK)) >> 1 >> (8 * sizeof(((nvm_flash_cfg_t *)0)->sysCfgBits) - 1)) == 0u, "CONFIG_MAG.recalMode: mask is wider than its member");
_Static_assert(((uint32_t)(SYS_CFG_BITS_MAG_RECAL_MODE_MASK)) / CBOR_BIND_LSB(((uint32_t)(SYS_CFG_BITS_MAG_RECAL_MODE_MASK))) <= UINT8_MAX, "CONFIG_MAG.recalMode: mask holds more than the key's type");
_Static_assert(_Generic(((nvm_flash_cfg_t *)0)->magInterferenceThreshold, float: 1, default: 0), "CONFIG_MAG.interferenceThreshold: type differs from its member");
_Static_assert(_Generic(((nvm_flash_cfg_t *)0)->magCalibrationQualityThreshold, float: 1, default: 0), "CONFIG_MAG.calQualityThreshold: type differs from its member");
_Static_assert(_Generic(((nvm_flash_cfg_t *)0)->magDeclination, float: 1, default: 0), "CONFIG_MAG.declination.valueRad: type differs from its member");
_Static_assert(((uint32_t)(SYS_CFG_BITS_DISABLE_MAG_DECL_ESTIMATION)) != 0u, "CONFIG_MAG.declination.estimate: mask is zero");
_Static_assert(sizeof(((nvm_flash_cfg_t *)0)->sysCfgBits) <= 4, "CONFIG_MAG.declination.estimate: a mask needs an integer member of at most 32 bits");
_Static_assert((((uint32_t)(SYS_CFG_BITS_DISABLE_MAG_DECL_ESTIMATION)) >> 1 >> (8 * sizeof(((nvm_flash_cfg_t *)0)->sysCfgBits) - 1)) == 0u, "CONFIG_MAG.declination.estimate: mask is wider than its member");
_Static_assert((((uint32_t)(SYS_CFG_BITS_DISABLE_MAG_DECL_ESTIMATION)) & (((uint32_t)(SYS_CFG_BITS_DISABLE_MAG_DECL_ESTIMATION)) - 1u)) == 0u, "CONFIG_MAG.declination.estimate: a bool needs a single-bit mask");
_Static_assert(((uint32_t)(SYS_CFG_BITS_MAG_ENABLE_WMM_DECLINATION)) != 0u, "CONFIG_MAG.declination.useWmm: mask is zero");
_Static_assert(sizeof(((nvm_flash_cfg_t *)0)->sysCfgBits) <= 4, "CONFIG_MAG.declination.useWmm: a mask needs an integer member of at most 32 bits");
_Static_assert((((uint32_t)(SYS_CFG_BITS_MAG_ENABLE_WMM_DECLINATION)) >> 1 >> (8 * sizeof(((nvm_flash_cfg_t *)0)->sysCfgBits) - 1)) == 0u, "CONFIG_MAG.declination.useWmm: mask is wider than its member");
_Static_assert((((uint32_t)(SYS_CFG_BITS_MAG_ENABLE_WMM_DECLINATION)) & (((uint32_t)(SYS_CFG_BITS_MAG_ENABLE_WMM_DECLINATION)) - 1u)) == 0u, "CONFIG_MAG.declination.useWmm: a bool needs a single-bit mask");
_Static_assert(sizeof(((nvm_flash_cfg_t *)0)->wheelConfig.transform.e_b2w) == sizeof(((cbor_config_ground_vehicle_t *)0)->transform.rotation), "CONFIG_GROUND_VEHICLE.transform.rotation: array size differs from its member");
_Static_assert(_Generic(((nvm_flash_cfg_t *)0)->wheelConfig.transform.e_b2w[0], float: 1, default: 0), "CONFIG_GROUND_VEHICLE.transform.rotation: element type differs from its member");
_Static_assert(sizeof(((nvm_flash_cfg_t *)0)->wheelConfig.transform.e_b2w_sigma) == sizeof(((cbor_config_ground_vehicle_t *)0)->transform.rotationSigma), "CONFIG_GROUND_VEHICLE.transform.rotationSigma: array size differs from its member");
_Static_assert(_Generic(((nvm_flash_cfg_t *)0)->wheelConfig.transform.e_b2w_sigma[0], float: 1, default: 0), "CONFIG_GROUND_VEHICLE.transform.rotationSigma: element type differs from its member");
_Static_assert(sizeof(((nvm_flash_cfg_t *)0)->wheelConfig.transform.t_b2w) == sizeof(((cbor_config_ground_vehicle_t *)0)->transform.translation), "CONFIG_GROUND_VEHICLE.transform.translation: array size differs from its member");
_Static_assert(_Generic(((nvm_flash_cfg_t *)0)->wheelConfig.transform.t_b2w[0], float: 1, default: 0), "CONFIG_GROUND_VEHICLE.transform.translation: element type differs from its member");
_Static_assert(sizeof(((nvm_flash_cfg_t *)0)->wheelConfig.transform.t_b2w_sigma) == sizeof(((cbor_config_ground_vehicle_t *)0)->transform.translationSigma), "CONFIG_GROUND_VEHICLE.transform.translationSigma: array size differs from its member");
_Static_assert(_Generic(((nvm_flash_cfg_t *)0)->wheelConfig.transform.t_b2w_sigma[0], float: 1, default: 0), "CONFIG_GROUND_VEHICLE.transform.translationSigma: element type differs from its member");
_Static_assert(_Generic(((nvm_flash_cfg_t *)0)->wheelConfig.track_width, float: 1, default: 0), "CONFIG_GROUND_VEHICLE.trackWidthM: type differs from its member");
_Static_assert(_Generic(((nvm_flash_cfg_t *)0)->wheelConfig.radius, float: 1, default: 0), "CONFIG_GROUND_VEHICLE.radiusM: type differs from its member");
_Static_assert(((uint32_t)(WHEEL_CFG_BITS_ENABLE_CONTROL)) != 0u, "CONFIG_GROUND_VEHICLE.controlEnabled: mask is zero");
_Static_assert(sizeof(((nvm_flash_cfg_t *)0)->wheelConfig.bits) <= 4, "CONFIG_GROUND_VEHICLE.controlEnabled: a mask needs an integer member of at most 32 bits");
_Static_assert((((uint32_t)(WHEEL_CFG_BITS_ENABLE_CONTROL)) >> 1 >> (8 * sizeof(((nvm_flash_cfg_t *)0)->wheelConfig.bits) - 1)) == 0u, "CONFIG_GROUND_VEHICLE.controlEnabled: mask is wider than its member");
_Static_assert((((uint32_t)(WHEEL_CFG_BITS_ENABLE_CONTROL)) & (((uint32_t)(WHEEL_CFG_BITS_ENABLE_CONTROL)) - 1u)) == 0u, "CONFIG_GROUND_VEHICLE.controlEnabled: a bool needs a single-bit mask");
_Static_assert(((uint32_t)(WHEEL_CFG_BITS_ENABLE_ENCODER)) != 0u, "CONFIG_GROUND_VEHICLE.encoder.enabled: mask is zero");
_Static_assert(sizeof(((nvm_flash_cfg_t *)0)->wheelConfig.bits) <= 4, "CONFIG_GROUND_VEHICLE.encoder.enabled: a mask needs an integer member of at most 32 bits");
_Static_assert((((uint32_t)(WHEEL_CFG_BITS_ENABLE_ENCODER)) >> 1 >> (8 * sizeof(((nvm_flash_cfg_t *)0)->wheelConfig.bits) - 1)) == 0u, "CONFIG_GROUND_VEHICLE.encoder.enabled: mask is wider than its member");
_Static_assert((((uint32_t)(WHEEL_CFG_BITS_ENABLE_ENCODER)) & (((uint32_t)(WHEEL_CFG_BITS_ENABLE_ENCODER)) - 1u)) == 0u, "CONFIG_GROUND_VEHICLE.encoder.enabled: a bool needs a single-bit mask");
_Static_assert(((uint32_t)(WHEEL_CFG_BITS_ENCODER_SOURCE)) != 0u, "CONFIG_GROUND_VEHICLE.encoder.source: mask is zero");
_Static_assert(sizeof(((nvm_flash_cfg_t *)0)->wheelConfig.bits) <= 4, "CONFIG_GROUND_VEHICLE.encoder.source: a mask needs an integer member of at most 32 bits");
_Static_assert((((uint32_t)(WHEEL_CFG_BITS_ENCODER_SOURCE)) >> 1 >> (8 * sizeof(((nvm_flash_cfg_t *)0)->wheelConfig.bits) - 1)) == 0u, "CONFIG_GROUND_VEHICLE.encoder.source: mask is wider than its member");
_Static_assert(((uint32_t)(WHEEL_CFG_BITS_ENCODER_SOURCE)) / CBOR_BIND_LSB(((uint32_t)(WHEEL_CFG_BITS_ENCODER_SOURCE))) <= UINT8_MAX, "CONFIG_GROUND_VEHICLE.encoder.source: mask holds more than the key's type");
_Static_assert(((uint32_t)(WHEEL_CFG_BITS_DIRECTION_REVERSE_LEFT)) != 0u, "CONFIG_GROUND_VEHICLE.encoder.reverseLeft: mask is zero");
_Static_assert(sizeof(((nvm_flash_cfg_t *)0)->wheelConfig.bits) <= 4, "CONFIG_GROUND_VEHICLE.encoder.reverseLeft: a mask needs an integer member of at most 32 bits");
_Static_assert((((uint32_t)(WHEEL_CFG_BITS_DIRECTION_REVERSE_LEFT)) >> 1 >> (8 * sizeof(((nvm_flash_cfg_t *)0)->wheelConfig.bits) - 1)) == 0u, "CONFIG_GROUND_VEHICLE.encoder.reverseLeft: mask is wider than its member");
_Static_assert((((uint32_t)(WHEEL_CFG_BITS_DIRECTION_REVERSE_LEFT)) & (((uint32_t)(WHEEL_CFG_BITS_DIRECTION_REVERSE_LEFT)) - 1u)) == 0u, "CONFIG_GROUND_VEHICLE.encoder.reverseLeft: a bool needs a single-bit mask");
_Static_assert(((uint32_t)(WHEEL_CFG_BITS_DIRECTION_REVERSE_RIGHT)) != 0u, "CONFIG_GROUND_VEHICLE.encoder.reverseRight: mask is zero");
_Static_assert(sizeof(((nvm_flash_cfg_t *)0)->wheelConfig.bits) <= 4, "CONFIG_GROUND_VEHICLE.encoder.reverseRight: a mask needs an integer member of at most 32 bits");
_Static_assert((((uint32_t)(WHEEL_CFG_BITS_DIRECTION_REVERSE_RIGHT)) >> 1 >> (8 * sizeof(((nvm_flash_cfg_t *)0)->wheelConfig.bits) - 1)) == 0u, "CONFIG_GROUND_VEHICLE.encoder.reverseRight: mask is wider than its member");
_Static_assert((((uint32_t)(WHEEL_CFG_BITS_DIRECTION_REVERSE_RIGHT)) & (((uint32_t)(WHEEL_CFG_BITS_DIRECTION_REVERSE_RIGHT)) - 1u)) == 0u, "CONFIG_GROUND_VEHICLE.encoder.reverseRight: a bool needs a single-bit mask");
_Static_assert(_Generic(((nvm_flash_cfg_t *)0)->ser0BaudRate, uint32_t: 1, default: 0), "CONFIG_PORTS.Port.SER0.baudRate: type differs from its member");
_Static_assert(_Generic(((nvm_flash_cfg_t *)0)->ser1BaudRate, uint32_t: 1, default: 0), "CONFIG_PORTS.Port.SER1.baudRate: type differs from its member");
_Static_assert(_Generic(((nvm_flash_cfg_t *)0)->ser2BaudRate, uint32_t: 1, default: 0), "CONFIG_PORTS.Port.SER2.baudRate: type differs from its member");
_Static_assert(((uint32_t)(SENSOR_CFG_SENSOR_ROTATION_MASK)) != 0u, "CONFIG_SENSORS.rotation: mask is zero");
_Static_assert(sizeof(((nvm_flash_cfg_t *)0)->sensorConfig) <= 4, "CONFIG_SENSORS.rotation: a mask needs an integer member of at most 32 bits");
_Static_assert((((uint32_t)(SENSOR_CFG_SENSOR_ROTATION_MASK)) >> 1 >> (8 * sizeof(((nvm_flash_cfg_t *)0)->sensorConfig) - 1)) == 0u, "CONFIG_SENSORS.rotation: mask is wider than its member");
_Static_assert(((uint32_t)(SENSOR_CFG_SENSOR_ROTATION_MASK)) / CBOR_BIND_LSB(((uint32_t)(SENSOR_CFG_SENSOR_ROTATION_MASK))) <= UINT8_MAX, "CONFIG_SENSORS.rotation: mask holds more than the key's type");
_Static_assert(((uint32_t)(SENSOR_CFG_GYR_FS_MASK)) != 0u, "CONFIG_SENSORS.Imus.Gyr.PRIMARY.fullScale: mask is zero");
_Static_assert(sizeof(((nvm_flash_cfg_t *)0)->sensorConfig) <= 4, "CONFIG_SENSORS.Imus.Gyr.PRIMARY.fullScale: a mask needs an integer member of at most 32 bits");
_Static_assert((((uint32_t)(SENSOR_CFG_GYR_FS_MASK)) >> 1 >> (8 * sizeof(((nvm_flash_cfg_t *)0)->sensorConfig) - 1)) == 0u, "CONFIG_SENSORS.Imus.Gyr.PRIMARY.fullScale: mask is wider than its member");
_Static_assert(((uint32_t)(SENSOR_CFG_GYR_FS_MASK)) / CBOR_BIND_LSB(((uint32_t)(SENSOR_CFG_GYR_FS_MASK))) <= UINT8_MAX, "CONFIG_SENSORS.Imus.Gyr.PRIMARY.fullScale: mask holds more than the key's type");
_Static_assert(((uint32_t)(SENSOR_CFG_GYR_DLPF_MASK)) != 0u, "CONFIG_SENSORS.Imus.Gyr.PRIMARY.dlpf: mask is zero");
_Static_assert(sizeof(((nvm_flash_cfg_t *)0)->sensorConfig) <= 4, "CONFIG_SENSORS.Imus.Gyr.PRIMARY.dlpf: a mask needs an integer member of at most 32 bits");
_Static_assert((((uint32_t)(SENSOR_CFG_GYR_DLPF_MASK)) >> 1 >> (8 * sizeof(((nvm_flash_cfg_t *)0)->sensorConfig) - 1)) == 0u, "CONFIG_SENSORS.Imus.Gyr.PRIMARY.dlpf: mask is wider than its member");
_Static_assert(((uint32_t)(SENSOR_CFG_GYR_DLPF_MASK)) / CBOR_BIND_LSB(((uint32_t)(SENSOR_CFG_GYR_DLPF_MASK))) <= UINT8_MAX, "CONFIG_SENSORS.Imus.Gyr.PRIMARY.dlpf: mask holds more than the key's type");
_Static_assert(((uint32_t)(SENSOR_CFG_ACC_FS_MASK)) != 0u, "CONFIG_SENSORS.Imus.Acc.PRIMARY.fullScale: mask is zero");
_Static_assert(sizeof(((nvm_flash_cfg_t *)0)->sensorConfig) <= 4, "CONFIG_SENSORS.Imus.Acc.PRIMARY.fullScale: a mask needs an integer member of at most 32 bits");
_Static_assert((((uint32_t)(SENSOR_CFG_ACC_FS_MASK)) >> 1 >> (8 * sizeof(((nvm_flash_cfg_t *)0)->sensorConfig) - 1)) == 0u, "CONFIG_SENSORS.Imus.Acc.PRIMARY.fullScale: mask is wider than its member");
_Static_assert(((uint32_t)(SENSOR_CFG_ACC_FS_MASK)) / CBOR_BIND_LSB(((uint32_t)(SENSOR_CFG_ACC_FS_MASK))) <= UINT8_MAX, "CONFIG_SENSORS.Imus.Acc.PRIMARY.fullScale: mask holds more than the key's type");
_Static_assert(((uint32_t)(SENSOR_CFG_ACC_DLPF_MASK)) != 0u, "CONFIG_SENSORS.Imus.Acc.PRIMARY.dlpf: mask is zero");
_Static_assert(sizeof(((nvm_flash_cfg_t *)0)->sensorConfig) <= 4, "CONFIG_SENSORS.Imus.Acc.PRIMARY.dlpf: a mask needs an integer member of at most 32 bits");
_Static_assert((((uint32_t)(SENSOR_CFG_ACC_DLPF_MASK)) >> 1 >> (8 * sizeof(((nvm_flash_cfg_t *)0)->sensorConfig) - 1)) == 0u, "CONFIG_SENSORS.Imus.Acc.PRIMARY.dlpf: mask is wider than its member");
_Static_assert(((uint32_t)(SENSOR_CFG_ACC_DLPF_MASK)) / CBOR_BIND_LSB(((uint32_t)(SENSOR_CFG_ACC_DLPF_MASK))) <= UINT8_MAX, "CONFIG_SENSORS.Imus.Acc.PRIMARY.dlpf: mask holds more than the key's type");
_Static_assert(((uint32_t)(SENSOR_CFG2_ACC_HG_FS_MASK)) != 0u, "CONFIG_SENSORS.Imus.Acc.HIGH_G.fullScale: mask is zero");
_Static_assert(sizeof(((nvm_flash_cfg_t *)0)->sensorConfig2) <= 4, "CONFIG_SENSORS.Imus.Acc.HIGH_G.fullScale: a mask needs an integer member of at most 32 bits");
_Static_assert((((uint32_t)(SENSOR_CFG2_ACC_HG_FS_MASK)) >> 1 >> (8 * sizeof(((nvm_flash_cfg_t *)0)->sensorConfig2) - 1)) == 0u, "CONFIG_SENSORS.Imus.Acc.HIGH_G.fullScale: mask is wider than its member");
_Static_assert(((uint32_t)(SENSOR_CFG2_ACC_HG_FS_MASK)) / CBOR_BIND_LSB(((uint32_t)(SENSOR_CFG2_ACC_HG_FS_MASK))) <= UINT8_MAX, "CONFIG_SENSORS.Imus.Acc.HIGH_G.fullScale: mask holds more than the key's type");
_Static_assert(((uint32_t)(SENSOR_CFG2_IMU_1_DISABLE)) != 0u, "CONFIG_SENSORS.Imu.IMU1.enabled: mask is zero");
_Static_assert(sizeof(((nvm_flash_cfg_t *)0)->sensorConfig2) <= 4, "CONFIG_SENSORS.Imu.IMU1.enabled: a mask needs an integer member of at most 32 bits");
_Static_assert((((uint32_t)(SENSOR_CFG2_IMU_1_DISABLE)) >> 1 >> (8 * sizeof(((nvm_flash_cfg_t *)0)->sensorConfig2) - 1)) == 0u, "CONFIG_SENSORS.Imu.IMU1.enabled: mask is wider than its member");
_Static_assert((((uint32_t)(SENSOR_CFG2_IMU_1_DISABLE)) & (((uint32_t)(SENSOR_CFG2_IMU_1_DISABLE)) - 1u)) == 0u, "CONFIG_SENSORS.Imu.IMU1.enabled: a bool needs a single-bit mask");
_Static_assert(((uint32_t)(SENSOR_CFG2_IMU_2_DISABLE)) != 0u, "CONFIG_SENSORS.Imu.IMU2.enabled: mask is zero");
_Static_assert(sizeof(((nvm_flash_cfg_t *)0)->sensorConfig2) <= 4, "CONFIG_SENSORS.Imu.IMU2.enabled: a mask needs an integer member of at most 32 bits");
_Static_assert((((uint32_t)(SENSOR_CFG2_IMU_2_DISABLE)) >> 1 >> (8 * sizeof(((nvm_flash_cfg_t *)0)->sensorConfig2) - 1)) == 0u, "CONFIG_SENSORS.Imu.IMU2.enabled: mask is wider than its member");
_Static_assert((((uint32_t)(SENSOR_CFG2_IMU_2_DISABLE)) & (((uint32_t)(SENSOR_CFG2_IMU_2_DISABLE)) - 1u)) == 0u, "CONFIG_SENSORS.Imu.IMU2.enabled: a bool needs a single-bit mask");
_Static_assert(((uint32_t)(SENSOR_CFG2_IMU_3_DISABLE)) != 0u, "CONFIG_SENSORS.Imu.IMU3.enabled: mask is zero");
_Static_assert(sizeof(((nvm_flash_cfg_t *)0)->sensorConfig2) <= 4, "CONFIG_SENSORS.Imu.IMU3.enabled: a mask needs an integer member of at most 32 bits");
_Static_assert((((uint32_t)(SENSOR_CFG2_IMU_3_DISABLE)) >> 1 >> (8 * sizeof(((nvm_flash_cfg_t *)0)->sensorConfig2) - 1)) == 0u, "CONFIG_SENSORS.Imu.IMU3.enabled: mask is wider than its member");
_Static_assert((((uint32_t)(SENSOR_CFG2_IMU_3_DISABLE)) & (((uint32_t)(SENSOR_CFG2_IMU_3_DISABLE)) - 1u)) == 0u, "CONFIG_SENSORS.Imu.IMU3.enabled: a bool needs a single-bit mask");
_Static_assert(((uint32_t)(SENSOR_CFG2_IMU_4_DISABLE)) != 0u, "CONFIG_SENSORS.Imu.IMU4.enabled: mask is zero");
_Static_assert(sizeof(((nvm_flash_cfg_t *)0)->sensorConfig2) <= 4, "CONFIG_SENSORS.Imu.IMU4.enabled: a mask needs an integer member of at most 32 bits");
_Static_assert((((uint32_t)(SENSOR_CFG2_IMU_4_DISABLE)) >> 1 >> (8 * sizeof(((nvm_flash_cfg_t *)0)->sensorConfig2) - 1)) == 0u, "CONFIG_SENSORS.Imu.IMU4.enabled: mask is wider than its member");
_Static_assert((((uint32_t)(SENSOR_CFG2_IMU_4_DISABLE)) & (((uint32_t)(SENSOR_CFG2_IMU_4_DISABLE)) - 1u)) == 0u, "CONFIG_SENSORS.Imu.IMU4.enabled: a bool needs a single-bit mask");
_Static_assert(((uint32_t)(SENSOR_CFG2_IMU_5_DISABLE)) != 0u, "CONFIG_SENSORS.Imu.IMU5.enabled: mask is zero");
_Static_assert(sizeof(((nvm_flash_cfg_t *)0)->sensorConfig2) <= 4, "CONFIG_SENSORS.Imu.IMU5.enabled: a mask needs an integer member of at most 32 bits");
_Static_assert((((uint32_t)(SENSOR_CFG2_IMU_5_DISABLE)) >> 1 >> (8 * sizeof(((nvm_flash_cfg_t *)0)->sensorConfig2) - 1)) == 0u, "CONFIG_SENSORS.Imu.IMU5.enabled: mask is wider than its member");
_Static_assert((((uint32_t)(SENSOR_CFG2_IMU_5_DISABLE)) & (((uint32_t)(SENSOR_CFG2_IMU_5_DISABLE)) - 1u)) == 0u, "CONFIG_SENSORS.Imu.IMU5.enabled: a bool needs a single-bit mask");
_Static_assert(((uint32_t)(SENSOR_CFG_DISABLE_MAGNETOMETER)) != 0u, "CONFIG_SENSORS.Mags.enabled: mask is zero");
_Static_assert(sizeof(((nvm_flash_cfg_t *)0)->sensorConfig) <= 4, "CONFIG_SENSORS.Mags.enabled: a mask needs an integer member of at most 32 bits");
_Static_assert((((uint32_t)(SENSOR_CFG_DISABLE_MAGNETOMETER)) >> 1 >> (8 * sizeof(((nvm_flash_cfg_t *)0)->sensorConfig) - 1)) == 0u, "CONFIG_SENSORS.Mags.enabled: mask is wider than its member");
_Static_assert((((uint32_t)(SENSOR_CFG_DISABLE_MAGNETOMETER)) & (((uint32_t)(SENSOR_CFG_DISABLE_MAGNETOMETER)) - 1u)) == 0u, "CONFIG_SENSORS.Mags.enabled: a bool needs a single-bit mask");
_Static_assert(((uint32_t)(SENSOR_CFG_DISABLE_BAROMETER)) != 0u, "CONFIG_SENSORS.Baros.enabled: mask is zero");
_Static_assert(sizeof(((nvm_flash_cfg_t *)0)->sensorConfig) <= 4, "CONFIG_SENSORS.Baros.enabled: a mask needs an integer member of at most 32 bits");
_Static_assert((((uint32_t)(SENSOR_CFG_DISABLE_BAROMETER)) >> 1 >> (8 * sizeof(((nvm_flash_cfg_t *)0)->sensorConfig) - 1)) == 0u, "CONFIG_SENSORS.Baros.enabled: mask is wider than its member");
_Static_assert((((uint32_t)(SENSOR_CFG_DISABLE_BAROMETER)) & (((uint32_t)(SENSOR_CFG_DISABLE_BAROMETER)) - 1u)) == 0u, "CONFIG_SENSORS.Baros.enabled: a bool needs a single-bit mask");
_Static_assert(((uint32_t)(IO_CONFIG_STROBE_TRIGGER_HIGH)) != 0u, "CONFIG_IO.strobeActiveHigh: mask is zero");
_Static_assert(sizeof(((nvm_flash_cfg_t *)0)->ioConfig) <= 4, "CONFIG_IO.strobeActiveHigh: a mask needs an integer member of at most 32 bits");
_Static_assert((((uint32_t)(IO_CONFIG_STROBE_TRIGGER_HIGH)) >> 1 >> (8 * sizeof(((nvm_flash_cfg_t *)0)->ioConfig) - 1)) == 0u, "CONFIG_IO.strobeActiveHigh: mask is wider than its member");
_Static_assert((((uint32_t)(IO_CONFIG_STROBE_TRIGGER_HIGH)) & (((uint32_t)(IO_CONFIG_STROBE_TRIGGER_HIGH)) - 1u)) == 0u, "CONFIG_IO.strobeActiveHigh: a bool needs a single-bit mask");
_Static_assert(((uint32_t)(IO_CONFIG_G1G2_MASK)) != 0u, "CONFIG_IO.Pin.G1G2.function: mask is zero");
_Static_assert(sizeof(((nvm_flash_cfg_t *)0)->ioConfig) <= 4, "CONFIG_IO.Pin.G1G2.function: a mask needs an integer member of at most 32 bits");
_Static_assert((((uint32_t)(IO_CONFIG_G1G2_MASK)) >> 1 >> (8 * sizeof(((nvm_flash_cfg_t *)0)->ioConfig) - 1)) == 0u, "CONFIG_IO.Pin.G1G2.function: mask is wider than its member");
_Static_assert(((uint32_t)(IO_CONFIG_G1G2_MASK)) / CBOR_BIND_LSB(((uint32_t)(IO_CONFIG_G1G2_MASK))) <= UINT8_MAX, "CONFIG_IO.Pin.G1G2.function: mask holds more than the key's type");
_Static_assert(((uint32_t)(IO_CONFIG_G5G8_MASK)) != 0u, "CONFIG_IO.Pin.G5G8.function: mask is zero");
_Static_assert(sizeof(((nvm_flash_cfg_t *)0)->ioConfig) <= 4, "CONFIG_IO.Pin.G5G8.function: a mask needs an integer member of at most 32 bits");
_Static_assert((((uint32_t)(IO_CONFIG_G5G8_MASK)) >> 1 >> (8 * sizeof(((nvm_flash_cfg_t *)0)->ioConfig) - 1)) == 0u, "CONFIG_IO.Pin.G5G8.function: mask is wider than its member");
_Static_assert(((uint32_t)(IO_CONFIG_G5G8_MASK)) / CBOR_BIND_LSB(((uint32_t)(IO_CONFIG_G5G8_MASK))) <= UINT8_MAX, "CONFIG_IO.Pin.G5G8.function: mask holds more than the key's type");
_Static_assert(((uint32_t)(IO_CONFIG_G6G7_MASK)) != 0u, "CONFIG_IO.Pin.G6G7.function: mask is zero");
_Static_assert(sizeof(((nvm_flash_cfg_t *)0)->ioConfig) <= 4, "CONFIG_IO.Pin.G6G7.function: a mask needs an integer member of at most 32 bits");
_Static_assert((((uint32_t)(IO_CONFIG_G6G7_MASK)) >> 1 >> (8 * sizeof(((nvm_flash_cfg_t *)0)->ioConfig) - 1)) == 0u, "CONFIG_IO.Pin.G6G7.function: mask is wider than its member");
_Static_assert(((uint32_t)(IO_CONFIG_G6G7_MASK)) / CBOR_BIND_LSB(((uint32_t)(IO_CONFIG_G6G7_MASK))) <= UINT8_MAX, "CONFIG_IO.Pin.G6G7.function: mask holds more than the key's type");
_Static_assert(((uint32_t)(IO_CONFIG_G9_MASK)) != 0u, "CONFIG_IO.Pin.G9.function: mask is zero");
_Static_assert(sizeof(((nvm_flash_cfg_t *)0)->ioConfig) <= 4, "CONFIG_IO.Pin.G9.function: a mask needs an integer member of at most 32 bits");
_Static_assert((((uint32_t)(IO_CONFIG_G9_MASK)) >> 1 >> (8 * sizeof(((nvm_flash_cfg_t *)0)->ioConfig) - 1)) == 0u, "CONFIG_IO.Pin.G9.function: mask is wider than its member");
_Static_assert(((uint32_t)(IO_CONFIG_G9_MASK)) / CBOR_BIND_LSB(((uint32_t)(IO_CONFIG_G9_MASK))) <= UINT8_MAX, "CONFIG_IO.Pin.G9.function: mask holds more than the key's type");
_Static_assert(((uint32_t)(IO_CFG2_G11_BITMASK)) != 0u, "CONFIG_IO.Pin.G11.function: mask is zero");
_Static_assert(sizeof(((nvm_flash_cfg_t *)0)->ioConfig2) <= 4, "CONFIG_IO.Pin.G11.function: a mask needs an integer member of at most 32 bits");
_Static_assert((((uint32_t)(IO_CFG2_G11_BITMASK)) >> 1 >> (8 * sizeof(((nvm_flash_cfg_t *)0)->ioConfig2) - 1)) == 0u, "CONFIG_IO.Pin.G11.function: mask is wider than its member");
_Static_assert(((uint32_t)(IO_CFG2_G11_BITMASK)) / CBOR_BIND_LSB(((uint32_t)(IO_CFG2_G11_BITMASK))) <= UINT8_MAX, "CONFIG_IO.Pin.G11.function: mask holds more than the key's type");
_Static_assert(((uint32_t)(IO_CFG2_G12_BITMASK)) != 0u, "CONFIG_IO.Pin.G12.function: mask is zero");
_Static_assert(sizeof(((nvm_flash_cfg_t *)0)->ioConfig2) <= 4, "CONFIG_IO.Pin.G12.function: a mask needs an integer member of at most 32 bits");
_Static_assert((((uint32_t)(IO_CFG2_G12_BITMASK)) >> 1 >> (8 * sizeof(((nvm_flash_cfg_t *)0)->ioConfig2) - 1)) == 0u, "CONFIG_IO.Pin.G12.function: mask is wider than its member");
_Static_assert(((uint32_t)(IO_CFG2_G12_BITMASK)) / CBOR_BIND_LSB(((uint32_t)(IO_CFG2_G12_BITMASK))) <= UINT8_MAX, "CONFIG_IO.Pin.G12.function: mask holds more than the key's type");
_Static_assert(((uint32_t)(IO_CFG2_G13_BITMASK)) != 0u, "CONFIG_IO.Pin.G13.function: mask is zero");
_Static_assert(sizeof(((nvm_flash_cfg_t *)0)->ioConfig2) <= 4, "CONFIG_IO.Pin.G13.function: a mask needs an integer member of at most 32 bits");
_Static_assert((((uint32_t)(IO_CFG2_G13_BITMASK)) >> 1 >> (8 * sizeof(((nvm_flash_cfg_t *)0)->ioConfig2) - 1)) == 0u, "CONFIG_IO.Pin.G13.function: mask is wider than its member");
_Static_assert(((uint32_t)(IO_CFG2_G13_BITMASK)) / CBOR_BIND_LSB(((uint32_t)(IO_CFG2_G13_BITMASK))) <= UINT8_MAX, "CONFIG_IO.Pin.G13.function: mask holds more than the key's type");
_Static_assert(((uint32_t)(IO_CONFIG_G15_STROBE_INPUT)) != 0u, "CONFIG_IO.Pin.G15.function: mask is zero");
_Static_assert(sizeof(((nvm_flash_cfg_t *)0)->ioConfig) <= 4, "CONFIG_IO.Pin.G15.function: a mask needs an integer member of at most 32 bits");
_Static_assert((((uint32_t)(IO_CONFIG_G15_STROBE_INPUT)) >> 1 >> (8 * sizeof(((nvm_flash_cfg_t *)0)->ioConfig) - 1)) == 0u, "CONFIG_IO.Pin.G15.function: mask is wider than its member");
_Static_assert(((uint32_t)(IO_CONFIG_G15_STROBE_INPUT)) / CBOR_BIND_LSB(((uint32_t)(IO_CONFIG_G15_STROBE_INPUT))) <= UINT8_MAX, "CONFIG_IO.Pin.G15.function: mask holds more than the key's type");
_Static_assert(_Generic(((nvm_flash_cfg_t *)0)->startupGnssDtMs, uint32_t: 1, default: 0), "CONFIG_GNSS.periodMs: type differs from its member");
_Static_assert(((uint32_t)(IO_CFG2_USE_GNSS2_AS_SOURCE)) != 0u, "CONFIG_GNSS.primaryReceiver: mask is zero");
_Static_assert(sizeof(((nvm_flash_cfg_t *)0)->ioConfig2) <= 4, "CONFIG_GNSS.primaryReceiver: a mask needs an integer member of at most 32 bits");
_Static_assert((((uint32_t)(IO_CFG2_USE_GNSS2_AS_SOURCE)) >> 1 >> (8 * sizeof(((nvm_flash_cfg_t *)0)->ioConfig2) - 1)) == 0u, "CONFIG_GNSS.primaryReceiver: mask is wider than its member");
_Static_assert(((uint32_t)(IO_CFG2_USE_GNSS2_AS_SOURCE)) / CBOR_BIND_LSB(((uint32_t)(IO_CFG2_USE_GNSS2_AS_SOURCE))) <= UINT8_MAX, "CONFIG_GNSS.primaryReceiver: mask holds more than the key's type");
_Static_assert(_Generic(((nvm_flash_cfg_t *)0)->gnssTimeSyncPeriodMs, uint32_t: 1, default: 0), "CONFIG_GNSS.timeSync.periodMs: type differs from its member");
_Static_assert(_Generic(((nvm_flash_cfg_t *)0)->gnssTimeUserDelay, float: 1, default: 0), "CONFIG_GNSS.timeSync.userDelayS: type differs from its member");
_Static_assert(_Generic(((nvm_flash_cfg_t *)0)->gnssSatSigConst, uint16_t: 1, default: 0), "CONFIG_GNSS.Receivers.satSignals: type differs from its member");
_Static_assert(_Generic(((nvm_flash_cfg_t *)0)->gnssMinimumElevation, float: 1, default: 0), "CONFIG_GNSS.Receivers.minElevation: type differs from its member");
_Static_assert(_Generic(((nvm_flash_cfg_t *)0)->gnssCn0Minimum, uint8_t: 1, default: 0), "CONFIG_GNSS.Receivers.cn0Min: type differs from its member");
_Static_assert(_Generic(((nvm_flash_cfg_t *)0)->gnssCn0DynMinOffset, uint8_t: 1, default: 0), "CONFIG_GNSS.Receivers.cn0DynOffset: type differs from its member");
_Static_assert(((uint32_t)(IO_CONFIG_GNSS_TYPE_MASK << IO_CONFIG_GNSS1_TYPE_OFFSET)) != 0u, "CONFIG_GNSS.Receiver.GNSS1.type: mask is zero");
_Static_assert(sizeof(((nvm_flash_cfg_t *)0)->ioConfig) <= 4, "CONFIG_GNSS.Receiver.GNSS1.type: a mask needs an integer member of at most 32 bits");
_Static_assert((((uint32_t)(IO_CONFIG_GNSS_TYPE_MASK << IO_CONFIG_GNSS1_TYPE_OFFSET)) >> 1 >> (8 * sizeof(((nvm_flash_cfg_t *)0)->ioConfig) - 1)) == 0u, "CONFIG_GNSS.Receiver.GNSS1.type: mask is wider than its member");
_Static_assert(((uint32_t)(IO_CONFIG_GNSS_TYPE_MASK << IO_CONFIG_GNSS1_TYPE_OFFSET)) / CBOR_BIND_LSB(((uint32_t)(IO_CONFIG_GNSS_TYPE_MASK << IO_CONFIG_GNSS1_TYPE_OFFSET))) <= UINT8_MAX, "CONFIG_GNSS.Receiver.GNSS1.type: mask holds more than the key's type");
_Static_assert(((uint32_t)(IO_CONFIG_GNSS_SOURCE_MASK << IO_CONFIG_GNSS1_SOURCE_OFFSET)) != 0u, "CONFIG_GNSS.Receiver.GNSS1.port: mask is zero");
_Static_assert(sizeof(((nvm_flash_cfg_t *)0)->ioConfig) <= 4, "CONFIG_GNSS.Receiver.GNSS1.port: a mask needs an integer member of at most 32 bits");
_Static_assert((((uint32_t)(IO_CONFIG_GNSS_SOURCE_MASK << IO_CONFIG_GNSS1_SOURCE_OFFSET)) >> 1 >> (8 * sizeof(((nvm_flash_cfg_t *)0)->ioConfig) - 1)) == 0u, "CONFIG_GNSS.Receiver.GNSS1.port: mask is wider than its member");
_Static_assert(((uint32_t)(IO_CONFIG_GNSS_SOURCE_MASK << IO_CONFIG_GNSS1_SOURCE_OFFSET)) / CBOR_BIND_LSB(((uint32_t)(IO_CONFIG_GNSS_SOURCE_MASK << IO_CONFIG_GNSS1_SOURCE_OFFSET))) <= UINT8_MAX, "CONFIG_GNSS.Receiver.GNSS1.port: mask holds more than the key's type");
_Static_assert(((uint32_t)(IO_CFG_GNSS1_PPS_SOURCE_BITMASK)) != 0u, "CONFIG_GNSS.Receiver.GNSS1.ppsPin: mask is zero");
_Static_assert(sizeof(((nvm_flash_cfg_t *)0)->ioConfig) <= 4, "CONFIG_GNSS.Receiver.GNSS1.ppsPin: a mask needs an integer member of at most 32 bits");
_Static_assert((((uint32_t)(IO_CFG_GNSS1_PPS_SOURCE_BITMASK)) >> 1 >> (8 * sizeof(((nvm_flash_cfg_t *)0)->ioConfig) - 1)) == 0u, "CONFIG_GNSS.Receiver.GNSS1.ppsPin: mask is wider than its member");
_Static_assert(((uint32_t)(IO_CFG_GNSS1_PPS_SOURCE_BITMASK)) / CBOR_BIND_LSB(((uint32_t)(IO_CFG_GNSS1_PPS_SOURCE_BITMASK))) <= UINT8_MAX, "CONFIG_GNSS.Receiver.GNSS1.ppsPin: mask holds more than the key's type");
_Static_assert(((uint32_t)(IO_CONFIG_GNSS1_NO_INIT)) != 0u, "CONFIG_GNSS.Receiver.GNSS1.configure: mask is zero");
_Static_assert(sizeof(((nvm_flash_cfg_t *)0)->ioConfig) <= 4, "CONFIG_GNSS.Receiver.GNSS1.configure: a mask needs an integer member of at most 32 bits");
_Static_assert((((uint32_t)(IO_CONFIG_GNSS1_NO_INIT)) >> 1 >> (8 * sizeof(((nvm_flash_cfg_t *)0)->ioConfig) - 1)) == 0u, "CONFIG_GNSS.Receiver.GNSS1.configure: mask is wider than its member");
_Static_assert((((uint32_t)(IO_CONFIG_GNSS1_NO_INIT)) & (((uint32_t)(IO_CONFIG_GNSS1_NO_INIT)) - 1u)) == 0u, "CONFIG_GNSS.Receiver.GNSS1.configure: a bool needs a single-bit mask");
_Static_assert(((uint32_t)(SYS_CFG_BITS_DISABLE_GNSS1_FUSION)) != 0u, "CONFIG_GNSS.Receiver.GNSS1.fusionEnabled: mask is zero");
_Static_assert(sizeof(((nvm_flash_cfg_t *)0)->sysCfgBits) <= 4, "CONFIG_GNSS.Receiver.GNSS1.fusionEnabled: a mask needs an integer member of at most 32 bits");
_Static_assert((((uint32_t)(SYS_CFG_BITS_DISABLE_GNSS1_FUSION)) >> 1 >> (8 * sizeof(((nvm_flash_cfg_t *)0)->sysCfgBits) - 1)) == 0u, "CONFIG_GNSS.Receiver.GNSS1.fusionEnabled: mask is wider than its member");
_Static_assert((((uint32_t)(SYS_CFG_BITS_DISABLE_GNSS1_FUSION)) & (((uint32_t)(SYS_CFG_BITS_DISABLE_GNSS1_FUSION)) - 1u)) == 0u, "CONFIG_GNSS.Receiver.GNSS1.fusionEnabled: a bool needs a single-bit mask");
_Static_assert(((uint32_t)(IO_CONFIG_GNSS_TYPE_MASK << IO_CONFIG_GNSS2_TYPE_OFFSET)) != 0u, "CONFIG_GNSS.Receiver.GNSS2.type: mask is zero");
_Static_assert(sizeof(((nvm_flash_cfg_t *)0)->ioConfig) <= 4, "CONFIG_GNSS.Receiver.GNSS2.type: a mask needs an integer member of at most 32 bits");
_Static_assert((((uint32_t)(IO_CONFIG_GNSS_TYPE_MASK << IO_CONFIG_GNSS2_TYPE_OFFSET)) >> 1 >> (8 * sizeof(((nvm_flash_cfg_t *)0)->ioConfig) - 1)) == 0u, "CONFIG_GNSS.Receiver.GNSS2.type: mask is wider than its member");
_Static_assert(((uint32_t)(IO_CONFIG_GNSS_TYPE_MASK << IO_CONFIG_GNSS2_TYPE_OFFSET)) / CBOR_BIND_LSB(((uint32_t)(IO_CONFIG_GNSS_TYPE_MASK << IO_CONFIG_GNSS2_TYPE_OFFSET))) <= UINT8_MAX, "CONFIG_GNSS.Receiver.GNSS2.type: mask holds more than the key's type");
_Static_assert(((uint32_t)(IO_CONFIG_GNSS_SOURCE_MASK << IO_CONFIG_GNSS2_SOURCE_OFFSET)) != 0u, "CONFIG_GNSS.Receiver.GNSS2.port: mask is zero");
_Static_assert(sizeof(((nvm_flash_cfg_t *)0)->ioConfig) <= 4, "CONFIG_GNSS.Receiver.GNSS2.port: a mask needs an integer member of at most 32 bits");
_Static_assert((((uint32_t)(IO_CONFIG_GNSS_SOURCE_MASK << IO_CONFIG_GNSS2_SOURCE_OFFSET)) >> 1 >> (8 * sizeof(((nvm_flash_cfg_t *)0)->ioConfig) - 1)) == 0u, "CONFIG_GNSS.Receiver.GNSS2.port: mask is wider than its member");
_Static_assert(((uint32_t)(IO_CONFIG_GNSS_SOURCE_MASK << IO_CONFIG_GNSS2_SOURCE_OFFSET)) / CBOR_BIND_LSB(((uint32_t)(IO_CONFIG_GNSS_SOURCE_MASK << IO_CONFIG_GNSS2_SOURCE_OFFSET))) <= UINT8_MAX, "CONFIG_GNSS.Receiver.GNSS2.port: mask holds more than the key's type");
_Static_assert(((uint32_t)(IO_CFG2_GNSS2_PPS_SOURCE_BITMASK)) != 0u, "CONFIG_GNSS.Receiver.GNSS2.ppsPin: mask is zero");
_Static_assert(sizeof(((nvm_flash_cfg_t *)0)->ioConfig2) <= 4, "CONFIG_GNSS.Receiver.GNSS2.ppsPin: a mask needs an integer member of at most 32 bits");
_Static_assert((((uint32_t)(IO_CFG2_GNSS2_PPS_SOURCE_BITMASK)) >> 1 >> (8 * sizeof(((nvm_flash_cfg_t *)0)->ioConfig2) - 1)) == 0u, "CONFIG_GNSS.Receiver.GNSS2.ppsPin: mask is wider than its member");
_Static_assert(((uint32_t)(IO_CFG2_GNSS2_PPS_SOURCE_BITMASK)) / CBOR_BIND_LSB(((uint32_t)(IO_CFG2_GNSS2_PPS_SOURCE_BITMASK))) <= UINT8_MAX, "CONFIG_GNSS.Receiver.GNSS2.ppsPin: mask holds more than the key's type");
_Static_assert(((uint32_t)(IO_CONFIG_GNSS2_NO_INIT)) != 0u, "CONFIG_GNSS.Receiver.GNSS2.configure: mask is zero");
_Static_assert(sizeof(((nvm_flash_cfg_t *)0)->ioConfig) <= 4, "CONFIG_GNSS.Receiver.GNSS2.configure: a mask needs an integer member of at most 32 bits");
_Static_assert((((uint32_t)(IO_CONFIG_GNSS2_NO_INIT)) >> 1 >> (8 * sizeof(((nvm_flash_cfg_t *)0)->ioConfig) - 1)) == 0u, "CONFIG_GNSS.Receiver.GNSS2.configure: mask is wider than its member");
_Static_assert((((uint32_t)(IO_CONFIG_GNSS2_NO_INIT)) & (((uint32_t)(IO_CONFIG_GNSS2_NO_INIT)) - 1u)) == 0u, "CONFIG_GNSS.Receiver.GNSS2.configure: a bool needs a single-bit mask");
_Static_assert(((uint32_t)(SYS_CFG_BITS_DISABLE_GNSS2_FUSION)) != 0u, "CONFIG_GNSS.Receiver.GNSS2.fusionEnabled: mask is zero");
_Static_assert(sizeof(((nvm_flash_cfg_t *)0)->sysCfgBits) <= 4, "CONFIG_GNSS.Receiver.GNSS2.fusionEnabled: a mask needs an integer member of at most 32 bits");
_Static_assert((((uint32_t)(SYS_CFG_BITS_DISABLE_GNSS2_FUSION)) >> 1 >> (8 * sizeof(((nvm_flash_cfg_t *)0)->sysCfgBits) - 1)) == 0u, "CONFIG_GNSS.Receiver.GNSS2.fusionEnabled: mask is wider than its member");
_Static_assert((((uint32_t)(SYS_CFG_BITS_DISABLE_GNSS2_FUSION)) & (((uint32_t)(SYS_CFG_BITS_DISABLE_GNSS2_FUSION)) - 1u)) == 0u, "CONFIG_GNSS.Receiver.GNSS2.fusionEnabled: a bool needs a single-bit mask");
_Static_assert(((uint32_t)(SYS_CFG_BITS_ENABLE_GNSS_ANTENNA_OFFSET_ESTIMATION)) != 0u, "CONFIG_GNSS.Antennas.offsetEstimation: mask is zero");
_Static_assert(sizeof(((nvm_flash_cfg_t *)0)->sysCfgBits) <= 4, "CONFIG_GNSS.Antennas.offsetEstimation: a mask needs an integer member of at most 32 bits");
_Static_assert((((uint32_t)(SYS_CFG_BITS_ENABLE_GNSS_ANTENNA_OFFSET_ESTIMATION)) >> 1 >> (8 * sizeof(((nvm_flash_cfg_t *)0)->sysCfgBits) - 1)) == 0u, "CONFIG_GNSS.Antennas.offsetEstimation: mask is wider than its member");
_Static_assert((((uint32_t)(SYS_CFG_BITS_ENABLE_GNSS_ANTENNA_OFFSET_ESTIMATION)) & (((uint32_t)(SYS_CFG_BITS_ENABLE_GNSS_ANTENNA_OFFSET_ESTIMATION)) - 1u)) == 0u, "CONFIG_GNSS.Antennas.offsetEstimation: a bool needs a single-bit mask");
_Static_assert(sizeof(((nvm_flash_cfg_t *)0)->gnss1AntOffset) == sizeof(((cbor_config_gnss_t *)0)->Antenna[CBOR_ANTENNA_ID_ANT1].offset), "CONFIG_GNSS.Antenna.ANT1.offset: array size differs from its member");
_Static_assert(_Generic(((nvm_flash_cfg_t *)0)->gnss1AntOffset[0], float: 1, default: 0), "CONFIG_GNSS.Antenna.ANT1.offset: element type differs from its member");
_Static_assert(sizeof(((nvm_flash_cfg_t *)0)->gnss2AntOffset) == sizeof(((cbor_config_gnss_t *)0)->Antenna[CBOR_ANTENNA_ID_ANT2].offset), "CONFIG_GNSS.Antenna.ANT2.offset: array size differs from its member");
_Static_assert(_Generic(((nvm_flash_cfg_t *)0)->gnss2AntOffset[0], float: 1, default: 0), "CONFIG_GNSS.Antenna.ANT2.offset: element type differs from its member");
_Static_assert(((uint32_t)(RTK_CFG_BITS_ROVER_MODE_MASK)) != 0u, "CONFIG_GNSS.Rtk.mode: mask is zero");
_Static_assert(sizeof(((nvm_flash_cfg_t *)0)->RTKCfgBits) <= 4, "CONFIG_GNSS.Rtk.mode: a mask needs an integer member of at most 32 bits");
_Static_assert((((uint32_t)(RTK_CFG_BITS_ROVER_MODE_MASK)) >> 1 >> (8 * sizeof(((nvm_flash_cfg_t *)0)->RTKCfgBits) - 1)) == 0u, "CONFIG_GNSS.Rtk.mode: mask is wider than its member");
_Static_assert(((uint32_t)(RTK_CFG_BITS_ROVER_MODE_MASK)) / CBOR_BIND_LSB(((uint32_t)(RTK_CFG_BITS_ROVER_MODE_MASK))) <= UINT8_MAX, "CONFIG_GNSS.Rtk.mode: mask holds more than the key's type");
_Static_assert(((uint32_t)(RTK_CFG_BITS_BASE_POS_MOVING)) != 0u, "CONFIG_GNSS.Rtk.basePositionMoving: mask is zero");
_Static_assert(sizeof(((nvm_flash_cfg_t *)0)->RTKCfgBits) <= 4, "CONFIG_GNSS.Rtk.basePositionMoving: a mask needs an integer member of at most 32 bits");
_Static_assert((((uint32_t)(RTK_CFG_BITS_BASE_POS_MOVING)) >> 1 >> (8 * sizeof(((nvm_flash_cfg_t *)0)->RTKCfgBits) - 1)) == 0u, "CONFIG_GNSS.Rtk.basePositionMoving: mask is wider than its member");
_Static_assert((((uint32_t)(RTK_CFG_BITS_BASE_POS_MOVING)) & (((uint32_t)(RTK_CFG_BITS_BASE_POS_MOVING)) - 1u)) == 0u, "CONFIG_GNSS.Rtk.basePositionMoving: a bool needs a single-bit mask");
_Static_assert(((uint32_t)(RTK_CFG_BITS_RTK_BASE_IS_IDENTICAL_TO_ROVER)) != 0u, "CONFIG_GNSS.Rtk.baseIdenticalToRover: mask is zero");
_Static_assert(sizeof(((nvm_flash_cfg_t *)0)->RTKCfgBits) <= 4, "CONFIG_GNSS.Rtk.baseIdenticalToRover: a mask needs an integer member of at most 32 bits");
_Static_assert((((uint32_t)(RTK_CFG_BITS_RTK_BASE_IS_IDENTICAL_TO_ROVER)) >> 1 >> (8 * sizeof(((nvm_flash_cfg_t *)0)->RTKCfgBits) - 1)) == 0u, "CONFIG_GNSS.Rtk.baseIdenticalToRover: mask is wider than its member");
_Static_assert((((uint32_t)(RTK_CFG_BITS_RTK_BASE_IS_IDENTICAL_TO_ROVER)) & (((uint32_t)(RTK_CFG_BITS_RTK_BASE_IS_IDENTICAL_TO_ROVER)) - 1u)) == 0u, "CONFIG_GNSS.Rtk.baseIdenticalToRover: a bool needs a single-bit mask");
_Static_assert(((uint32_t)(RTK_CFG_BITS_GNSS_PORT_PASS_THROUGH)) != 0u, "CONFIG_GNSS.Rtk.passThrough: mask is zero");
_Static_assert(sizeof(((nvm_flash_cfg_t *)0)->RTKCfgBits) <= 4, "CONFIG_GNSS.Rtk.passThrough: a mask needs an integer member of at most 32 bits");
_Static_assert((((uint32_t)(RTK_CFG_BITS_GNSS_PORT_PASS_THROUGH)) >> 1 >> (8 * sizeof(((nvm_flash_cfg_t *)0)->RTKCfgBits) - 1)) == 0u, "CONFIG_GNSS.Rtk.passThrough: mask is wider than its member");
_Static_assert((((uint32_t)(RTK_CFG_BITS_GNSS_PORT_PASS_THROUGH)) & (((uint32_t)(RTK_CFG_BITS_GNSS_PORT_PASS_THROUGH)) - 1u)) == 0u, "CONFIG_GNSS.Rtk.passThrough: a bool needs a single-bit mask");
_Static_assert(((uint32_t)(RTK_CFG_BITS_MULTIPATH_MITIGATION)) != 0u, "CONFIG_GNSS.Rtk.multipathMitigation: mask is zero");
_Static_assert(sizeof(((nvm_flash_cfg_t *)0)->RTKCfgBits) <= 4, "CONFIG_GNSS.Rtk.multipathMitigation: a mask needs an integer member of at most 32 bits");
_Static_assert((((uint32_t)(RTK_CFG_BITS_MULTIPATH_MITIGATION)) >> 1 >> (8 * sizeof(((nvm_flash_cfg_t *)0)->RTKCfgBits) - 1)) == 0u, "CONFIG_GNSS.Rtk.multipathMitigation: mask is wider than its member");
_Static_assert((((uint32_t)(RTK_CFG_BITS_MULTIPATH_MITIGATION)) & (((uint32_t)(RTK_CFG_BITS_MULTIPATH_MITIGATION)) - 1u)) == 0u, "CONFIG_GNSS.Rtk.multipathMitigation: a bool needs a single-bit mask");
_Static_assert(((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS1_UBLOX_SER0)) != 0u, "CONFIG_GNSS.Rtk.BaseOutput.GNSS1_UBLOX.ser0: mask is zero");
_Static_assert(sizeof(((nvm_flash_cfg_t *)0)->RTKCfgBits) <= 4, "CONFIG_GNSS.Rtk.BaseOutput.GNSS1_UBLOX.ser0: a mask needs an integer member of at most 32 bits");
_Static_assert((((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS1_UBLOX_SER0)) >> 1 >> (8 * sizeof(((nvm_flash_cfg_t *)0)->RTKCfgBits) - 1)) == 0u, "CONFIG_GNSS.Rtk.BaseOutput.GNSS1_UBLOX.ser0: mask is wider than its member");
_Static_assert((((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS1_UBLOX_SER0)) & (((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS1_UBLOX_SER0)) - 1u)) == 0u, "CONFIG_GNSS.Rtk.BaseOutput.GNSS1_UBLOX.ser0: a bool needs a single-bit mask");
_Static_assert(((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS1_UBLOX_SER1)) != 0u, "CONFIG_GNSS.Rtk.BaseOutput.GNSS1_UBLOX.ser1: mask is zero");
_Static_assert(sizeof(((nvm_flash_cfg_t *)0)->RTKCfgBits) <= 4, "CONFIG_GNSS.Rtk.BaseOutput.GNSS1_UBLOX.ser1: a mask needs an integer member of at most 32 bits");
_Static_assert((((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS1_UBLOX_SER1)) >> 1 >> (8 * sizeof(((nvm_flash_cfg_t *)0)->RTKCfgBits) - 1)) == 0u, "CONFIG_GNSS.Rtk.BaseOutput.GNSS1_UBLOX.ser1: mask is wider than its member");
_Static_assert((((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS1_UBLOX_SER1)) & (((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS1_UBLOX_SER1)) - 1u)) == 0u, "CONFIG_GNSS.Rtk.BaseOutput.GNSS1_UBLOX.ser1: a bool needs a single-bit mask");
_Static_assert(((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS1_UBLOX_SER2)) != 0u, "CONFIG_GNSS.Rtk.BaseOutput.GNSS1_UBLOX.ser2: mask is zero");
_Static_assert(sizeof(((nvm_flash_cfg_t *)0)->RTKCfgBits) <= 4, "CONFIG_GNSS.Rtk.BaseOutput.GNSS1_UBLOX.ser2: a mask needs an integer member of at most 32 bits");
_Static_assert((((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS1_UBLOX_SER2)) >> 1 >> (8 * sizeof(((nvm_flash_cfg_t *)0)->RTKCfgBits) - 1)) == 0u, "CONFIG_GNSS.Rtk.BaseOutput.GNSS1_UBLOX.ser2: mask is wider than its member");
_Static_assert((((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS1_UBLOX_SER2)) & (((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS1_UBLOX_SER2)) - 1u)) == 0u, "CONFIG_GNSS.Rtk.BaseOutput.GNSS1_UBLOX.ser2: a bool needs a single-bit mask");
_Static_assert(((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS1_UBLOX_USB)) != 0u, "CONFIG_GNSS.Rtk.BaseOutput.GNSS1_UBLOX.usb: mask is zero");
_Static_assert(sizeof(((nvm_flash_cfg_t *)0)->RTKCfgBits) <= 4, "CONFIG_GNSS.Rtk.BaseOutput.GNSS1_UBLOX.usb: a mask needs an integer member of at most 32 bits");
_Static_assert((((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS1_UBLOX_USB)) >> 1 >> (8 * sizeof(((nvm_flash_cfg_t *)0)->RTKCfgBits) - 1)) == 0u, "CONFIG_GNSS.Rtk.BaseOutput.GNSS1_UBLOX.usb: mask is wider than its member");
_Static_assert((((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS1_UBLOX_USB)) & (((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS1_UBLOX_USB)) - 1u)) == 0u, "CONFIG_GNSS.Rtk.BaseOutput.GNSS1_UBLOX.usb: a bool needs a single-bit mask");
_Static_assert(((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS1_RTCM3_SER0)) != 0u, "CONFIG_GNSS.Rtk.BaseOutput.GNSS1_RTCM3.ser0: mask is zero");
_Static_assert(sizeof(((nvm_flash_cfg_t *)0)->RTKCfgBits) <= 4, "CONFIG_GNSS.Rtk.BaseOutput.GNSS1_RTCM3.ser0: a mask needs an integer member of at most 32 bits");
_Static_assert((((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS1_RTCM3_SER0)) >> 1 >> (8 * sizeof(((nvm_flash_cfg_t *)0)->RTKCfgBits) - 1)) == 0u, "CONFIG_GNSS.Rtk.BaseOutput.GNSS1_RTCM3.ser0: mask is wider than its member");
_Static_assert((((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS1_RTCM3_SER0)) & (((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS1_RTCM3_SER0)) - 1u)) == 0u, "CONFIG_GNSS.Rtk.BaseOutput.GNSS1_RTCM3.ser0: a bool needs a single-bit mask");
_Static_assert(((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS1_RTCM3_SER1)) != 0u, "CONFIG_GNSS.Rtk.BaseOutput.GNSS1_RTCM3.ser1: mask is zero");
_Static_assert(sizeof(((nvm_flash_cfg_t *)0)->RTKCfgBits) <= 4, "CONFIG_GNSS.Rtk.BaseOutput.GNSS1_RTCM3.ser1: a mask needs an integer member of at most 32 bits");
_Static_assert((((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS1_RTCM3_SER1)) >> 1 >> (8 * sizeof(((nvm_flash_cfg_t *)0)->RTKCfgBits) - 1)) == 0u, "CONFIG_GNSS.Rtk.BaseOutput.GNSS1_RTCM3.ser1: mask is wider than its member");
_Static_assert((((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS1_RTCM3_SER1)) & (((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS1_RTCM3_SER1)) - 1u)) == 0u, "CONFIG_GNSS.Rtk.BaseOutput.GNSS1_RTCM3.ser1: a bool needs a single-bit mask");
_Static_assert(((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS1_RTCM3_SER2)) != 0u, "CONFIG_GNSS.Rtk.BaseOutput.GNSS1_RTCM3.ser2: mask is zero");
_Static_assert(sizeof(((nvm_flash_cfg_t *)0)->RTKCfgBits) <= 4, "CONFIG_GNSS.Rtk.BaseOutput.GNSS1_RTCM3.ser2: a mask needs an integer member of at most 32 bits");
_Static_assert((((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS1_RTCM3_SER2)) >> 1 >> (8 * sizeof(((nvm_flash_cfg_t *)0)->RTKCfgBits) - 1)) == 0u, "CONFIG_GNSS.Rtk.BaseOutput.GNSS1_RTCM3.ser2: mask is wider than its member");
_Static_assert((((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS1_RTCM3_SER2)) & (((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS1_RTCM3_SER2)) - 1u)) == 0u, "CONFIG_GNSS.Rtk.BaseOutput.GNSS1_RTCM3.ser2: a bool needs a single-bit mask");
_Static_assert(((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS1_RTCM3_USB)) != 0u, "CONFIG_GNSS.Rtk.BaseOutput.GNSS1_RTCM3.usb: mask is zero");
_Static_assert(sizeof(((nvm_flash_cfg_t *)0)->RTKCfgBits) <= 4, "CONFIG_GNSS.Rtk.BaseOutput.GNSS1_RTCM3.usb: a mask needs an integer member of at most 32 bits");
_Static_assert((((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS1_RTCM3_USB)) >> 1 >> (8 * sizeof(((nvm_flash_cfg_t *)0)->RTKCfgBits) - 1)) == 0u, "CONFIG_GNSS.Rtk.BaseOutput.GNSS1_RTCM3.usb: mask is wider than its member");
_Static_assert((((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS1_RTCM3_USB)) & (((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS1_RTCM3_USB)) - 1u)) == 0u, "CONFIG_GNSS.Rtk.BaseOutput.GNSS1_RTCM3.usb: a bool needs a single-bit mask");
_Static_assert(((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS2_UBLOX_SER0)) != 0u, "CONFIG_GNSS.Rtk.BaseOutput.GNSS2_UBLOX.ser0: mask is zero");
_Static_assert(sizeof(((nvm_flash_cfg_t *)0)->RTKCfgBits) <= 4, "CONFIG_GNSS.Rtk.BaseOutput.GNSS2_UBLOX.ser0: a mask needs an integer member of at most 32 bits");
_Static_assert((((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS2_UBLOX_SER0)) >> 1 >> (8 * sizeof(((nvm_flash_cfg_t *)0)->RTKCfgBits) - 1)) == 0u, "CONFIG_GNSS.Rtk.BaseOutput.GNSS2_UBLOX.ser0: mask is wider than its member");
_Static_assert((((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS2_UBLOX_SER0)) & (((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS2_UBLOX_SER0)) - 1u)) == 0u, "CONFIG_GNSS.Rtk.BaseOutput.GNSS2_UBLOX.ser0: a bool needs a single-bit mask");
_Static_assert(((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS2_UBLOX_SER1)) != 0u, "CONFIG_GNSS.Rtk.BaseOutput.GNSS2_UBLOX.ser1: mask is zero");
_Static_assert(sizeof(((nvm_flash_cfg_t *)0)->RTKCfgBits) <= 4, "CONFIG_GNSS.Rtk.BaseOutput.GNSS2_UBLOX.ser1: a mask needs an integer member of at most 32 bits");
_Static_assert((((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS2_UBLOX_SER1)) >> 1 >> (8 * sizeof(((nvm_flash_cfg_t *)0)->RTKCfgBits) - 1)) == 0u, "CONFIG_GNSS.Rtk.BaseOutput.GNSS2_UBLOX.ser1: mask is wider than its member");
_Static_assert((((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS2_UBLOX_SER1)) & (((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS2_UBLOX_SER1)) - 1u)) == 0u, "CONFIG_GNSS.Rtk.BaseOutput.GNSS2_UBLOX.ser1: a bool needs a single-bit mask");
_Static_assert(((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS2_UBLOX_SER2)) != 0u, "CONFIG_GNSS.Rtk.BaseOutput.GNSS2_UBLOX.ser2: mask is zero");
_Static_assert(sizeof(((nvm_flash_cfg_t *)0)->RTKCfgBits) <= 4, "CONFIG_GNSS.Rtk.BaseOutput.GNSS2_UBLOX.ser2: a mask needs an integer member of at most 32 bits");
_Static_assert((((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS2_UBLOX_SER2)) >> 1 >> (8 * sizeof(((nvm_flash_cfg_t *)0)->RTKCfgBits) - 1)) == 0u, "CONFIG_GNSS.Rtk.BaseOutput.GNSS2_UBLOX.ser2: mask is wider than its member");
_Static_assert((((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS2_UBLOX_SER2)) & (((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS2_UBLOX_SER2)) - 1u)) == 0u, "CONFIG_GNSS.Rtk.BaseOutput.GNSS2_UBLOX.ser2: a bool needs a single-bit mask");
_Static_assert(((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS2_UBLOX_USB)) != 0u, "CONFIG_GNSS.Rtk.BaseOutput.GNSS2_UBLOX.usb: mask is zero");
_Static_assert(sizeof(((nvm_flash_cfg_t *)0)->RTKCfgBits) <= 4, "CONFIG_GNSS.Rtk.BaseOutput.GNSS2_UBLOX.usb: a mask needs an integer member of at most 32 bits");
_Static_assert((((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS2_UBLOX_USB)) >> 1 >> (8 * sizeof(((nvm_flash_cfg_t *)0)->RTKCfgBits) - 1)) == 0u, "CONFIG_GNSS.Rtk.BaseOutput.GNSS2_UBLOX.usb: mask is wider than its member");
_Static_assert((((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS2_UBLOX_USB)) & (((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS2_UBLOX_USB)) - 1u)) == 0u, "CONFIG_GNSS.Rtk.BaseOutput.GNSS2_UBLOX.usb: a bool needs a single-bit mask");
_Static_assert(((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS2_RTCM3_SER0)) != 0u, "CONFIG_GNSS.Rtk.BaseOutput.GNSS2_RTCM3.ser0: mask is zero");
_Static_assert(sizeof(((nvm_flash_cfg_t *)0)->RTKCfgBits) <= 4, "CONFIG_GNSS.Rtk.BaseOutput.GNSS2_RTCM3.ser0: a mask needs an integer member of at most 32 bits");
_Static_assert((((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS2_RTCM3_SER0)) >> 1 >> (8 * sizeof(((nvm_flash_cfg_t *)0)->RTKCfgBits) - 1)) == 0u, "CONFIG_GNSS.Rtk.BaseOutput.GNSS2_RTCM3.ser0: mask is wider than its member");
_Static_assert((((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS2_RTCM3_SER0)) & (((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS2_RTCM3_SER0)) - 1u)) == 0u, "CONFIG_GNSS.Rtk.BaseOutput.GNSS2_RTCM3.ser0: a bool needs a single-bit mask");
_Static_assert(((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS2_RTCM3_SER1)) != 0u, "CONFIG_GNSS.Rtk.BaseOutput.GNSS2_RTCM3.ser1: mask is zero");
_Static_assert(sizeof(((nvm_flash_cfg_t *)0)->RTKCfgBits) <= 4, "CONFIG_GNSS.Rtk.BaseOutput.GNSS2_RTCM3.ser1: a mask needs an integer member of at most 32 bits");
_Static_assert((((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS2_RTCM3_SER1)) >> 1 >> (8 * sizeof(((nvm_flash_cfg_t *)0)->RTKCfgBits) - 1)) == 0u, "CONFIG_GNSS.Rtk.BaseOutput.GNSS2_RTCM3.ser1: mask is wider than its member");
_Static_assert((((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS2_RTCM3_SER1)) & (((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS2_RTCM3_SER1)) - 1u)) == 0u, "CONFIG_GNSS.Rtk.BaseOutput.GNSS2_RTCM3.ser1: a bool needs a single-bit mask");
_Static_assert(((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS2_RTCM3_SER2)) != 0u, "CONFIG_GNSS.Rtk.BaseOutput.GNSS2_RTCM3.ser2: mask is zero");
_Static_assert(sizeof(((nvm_flash_cfg_t *)0)->RTKCfgBits) <= 4, "CONFIG_GNSS.Rtk.BaseOutput.GNSS2_RTCM3.ser2: a mask needs an integer member of at most 32 bits");
_Static_assert((((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS2_RTCM3_SER2)) >> 1 >> (8 * sizeof(((nvm_flash_cfg_t *)0)->RTKCfgBits) - 1)) == 0u, "CONFIG_GNSS.Rtk.BaseOutput.GNSS2_RTCM3.ser2: mask is wider than its member");
_Static_assert((((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS2_RTCM3_SER2)) & (((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS2_RTCM3_SER2)) - 1u)) == 0u, "CONFIG_GNSS.Rtk.BaseOutput.GNSS2_RTCM3.ser2: a bool needs a single-bit mask");
_Static_assert(((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS2_RTCM3_USB)) != 0u, "CONFIG_GNSS.Rtk.BaseOutput.GNSS2_RTCM3.usb: mask is zero");
_Static_assert(sizeof(((nvm_flash_cfg_t *)0)->RTKCfgBits) <= 4, "CONFIG_GNSS.Rtk.BaseOutput.GNSS2_RTCM3.usb: a mask needs an integer member of at most 32 bits");
_Static_assert((((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS2_RTCM3_USB)) >> 1 >> (8 * sizeof(((nvm_flash_cfg_t *)0)->RTKCfgBits) - 1)) == 0u, "CONFIG_GNSS.Rtk.BaseOutput.GNSS2_RTCM3.usb: mask is wider than its member");
_Static_assert((((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS2_RTCM3_USB)) & (((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS2_RTCM3_USB)) - 1u)) == 0u, "CONFIG_GNSS.Rtk.BaseOutput.GNSS2_RTCM3.usb: a bool needs a single-bit mask");
_Static_assert(sizeof(((nvm_flash_cfg_t *)0)->lastLla) == sizeof(((cbor_state_t *)0)->lastPosition.lla), "STATE.lastPosition.lla: array size differs from its member");
_Static_assert(_Generic(((nvm_flash_cfg_t *)0)->lastLla[0], double: 1, default: 0), "STATE.lastPosition.lla: element type differs from its member");
_Static_assert(_Generic(((nvm_flash_cfg_t *)0)->lastLlaTimeOfWeekMs, uint32_t: 1, default: 0), "STATE.lastPosition.timeOfWeekMs: type differs from its member");
_Static_assert(_Generic(((nvm_flash_cfg_t *)0)->lastLlaWeek, uint32_t: 1, default: 0), "STATE.lastPosition.week: type differs from its member");
_Static_assert(((uint64_t)((uint32_t)(SYS_CFG_BITS_DISABLE_LEDS)) + (uint64_t)((uint32_t)(SYS_CFG_BITS_BOR_THRESHOLD_MASK)) + (uint64_t)((uint32_t)(SYS_CFG_BITS_DISABLE_AUTO_BIT_ON_STARTUP)) + (uint64_t)((uint32_t)(SYS_CFG_BITS_DISABLE_INS_EKF)) + (uint64_t)((uint32_t)(SYS_CFG_USE_REFERENCE_IMU_IN_EKF)) + (uint64_t)((uint32_t)(SYS_CFG_EKF_REF_POINT_STATIONARY_ON_STROBE_INPUT)) + (uint64_t)((uint32_t)(SYS_CFG_BITS_DISABLE_MAGNETOMETER_FUSION)) + (uint64_t)((uint32_t)(SYS_CFG_BITS_DISABLE_BAROMETER_FUSION)) + (uint64_t)((uint32_t)(SYS_CFG_BITS_DISABLE_WHEEL_ENCODER_FUSION)) + (uint64_t)((uint32_t)(SYS_CFG_BITS_DISABLE_AUTO_ZERO_VELOCITY_UPDATES)) + (uint64_t)((uint32_t)(SYS_CFG_BITS_DISABLE_AUTO_ZERO_ANGULAR_RATE_UPDATES)) + (uint64_t)((uint32_t)(SYS_CFG_BITS_ENABLE_MAG_CONTINUOUS_CAL)) + (uint64_t)((uint32_t)(SYS_CFG_BITS_AUTO_MAG_RECAL)) + (uint64_t)((uint32_t)(SYS_CFG_BITS_MAG_RECAL_MODE_MASK)) + (uint64_t)((uint32_t)(SYS_CFG_BITS_DISABLE_MAG_DECL_ESTIMATION)) + (uint64_t)((uint32_t)(SYS_CFG_BITS_MAG_ENABLE_WMM_DECLINATION)) + (uint64_t)((uint32_t)(SYS_CFG_BITS_DISABLE_GNSS1_FUSION)) + (uint64_t)((uint32_t)(SYS_CFG_BITS_DISABLE_GNSS2_FUSION)) + (uint64_t)((uint32_t)(SYS_CFG_BITS_ENABLE_GNSS_ANTENNA_OFFSET_ESTIMATION)) + (uint64_t)((uint32_t)(0xFC0000E1))) == (uint64_t)(((uint32_t)(SYS_CFG_BITS_DISABLE_LEDS)) | ((uint32_t)(SYS_CFG_BITS_BOR_THRESHOLD_MASK)) | ((uint32_t)(SYS_CFG_BITS_DISABLE_AUTO_BIT_ON_STARTUP)) | ((uint32_t)(SYS_CFG_BITS_DISABLE_INS_EKF)) | ((uint32_t)(SYS_CFG_USE_REFERENCE_IMU_IN_EKF)) | ((uint32_t)(SYS_CFG_EKF_REF_POINT_STATIONARY_ON_STROBE_INPUT)) | ((uint32_t)(SYS_CFG_BITS_DISABLE_MAGNETOMETER_FUSION)) | ((uint32_t)(SYS_CFG_BITS_DISABLE_BAROMETER_FUSION)) | ((uint32_t)(SYS_CFG_BITS_DISABLE_WHEEL_ENCODER_FUSION)) | ((uint32_t)(SYS_CFG_BITS_DISABLE_AUTO_ZERO_VELOCITY_UPDATES)) | ((uint32_t)(SYS_CFG_BITS_DISABLE_AUTO_ZERO_ANGULAR_RATE_UPDATES)) | ((uint32_t)(SYS_CFG_BITS_ENABLE_MAG_CONTINUOUS_CAL)) | ((uint32_t)(SYS_CFG_BITS_AUTO_MAG_RECAL)) | ((uint32_t)(SYS_CFG_BITS_MAG_RECAL_MODE_MASK)) | ((uint32_t)(SYS_CFG_BITS_DISABLE_MAG_DECL_ESTIMATION)) | ((uint32_t)(SYS_CFG_BITS_MAG_ENABLE_WMM_DECLINATION)) | ((uint32_t)(SYS_CFG_BITS_DISABLE_GNSS1_FUSION)) | ((uint32_t)(SYS_CFG_BITS_DISABLE_GNSS2_FUSION)) | ((uint32_t)(SYS_CFG_BITS_ENABLE_GNSS_ANTENNA_OFFSET_ESTIMATION)) | ((uint32_t)(0xFC0000E1))), "imx_flash_cfg.sysCfgBits: two of CONFIG_SYSTEM.ledsEnabled, CONFIG_SYSTEM.brownOutLevel, CONFIG_SYSTEM.bitOnStartup, CONFIG_INS.ekfEnabled, CONFIG_INS.referenceImu, CONFIG_INS.stationaryOnStrobe, CONFIG_INS.fusion.mag, CONFIG_INS.fusion.baro, CONFIG_INS.fusion.wheel, CONFIG_INS.zeroVelocity.autoZupt, CONFIG_INS.zeroVelocity.autoZaru, CONFIG_MAG.continuousCal, CONFIG_MAG.autoRecal, CONFIG_MAG.recalMode, CONFIG_MAG.declination.estimate, CONFIG_MAG.declination.useWmm, CONFIG_GNSS.Receiver.GNSS1.fusionEnabled, CONFIG_GNSS.Receiver.GNSS2.fusionEnabled, CONFIG_GNSS.Antennas.offsetEstimation, imx_flash_cfg.sysCfgBits not stored/pending overlap");
_Static_assert((((uint32_t)(SYS_CFG_BITS_DISABLE_LEDS)) | ((uint32_t)(SYS_CFG_BITS_BOR_THRESHOLD_MASK)) | ((uint32_t)(SYS_CFG_BITS_DISABLE_AUTO_BIT_ON_STARTUP)) | ((uint32_t)(SYS_CFG_BITS_DISABLE_INS_EKF)) | ((uint32_t)(SYS_CFG_USE_REFERENCE_IMU_IN_EKF)) | ((uint32_t)(SYS_CFG_EKF_REF_POINT_STATIONARY_ON_STROBE_INPUT)) | ((uint32_t)(SYS_CFG_BITS_DISABLE_MAGNETOMETER_FUSION)) | ((uint32_t)(SYS_CFG_BITS_DISABLE_BAROMETER_FUSION)) | ((uint32_t)(SYS_CFG_BITS_DISABLE_WHEEL_ENCODER_FUSION)) | ((uint32_t)(SYS_CFG_BITS_DISABLE_AUTO_ZERO_VELOCITY_UPDATES)) | ((uint32_t)(SYS_CFG_BITS_DISABLE_AUTO_ZERO_ANGULAR_RATE_UPDATES)) | ((uint32_t)(SYS_CFG_BITS_ENABLE_MAG_CONTINUOUS_CAL)) | ((uint32_t)(SYS_CFG_BITS_AUTO_MAG_RECAL)) | ((uint32_t)(SYS_CFG_BITS_MAG_RECAL_MODE_MASK)) | ((uint32_t)(SYS_CFG_BITS_DISABLE_MAG_DECL_ESTIMATION)) | ((uint32_t)(SYS_CFG_BITS_MAG_ENABLE_WMM_DECLINATION)) | ((uint32_t)(SYS_CFG_BITS_DISABLE_GNSS1_FUSION)) | ((uint32_t)(SYS_CFG_BITS_DISABLE_GNSS2_FUSION)) | ((uint32_t)(SYS_CFG_BITS_ENABLE_GNSS_ANTENNA_OFFSET_ESTIMATION)) | ((uint32_t)(0xFC0000E1))) == (0xFFFFFFFFu >> (32 - 8 * sizeof(((nvm_flash_cfg_t *)0)->sysCfgBits))), "imx_flash_cfg.sysCfgBits: a bit has no key and is not listed as not_stored or pending");
_Static_assert(((uint64_t)((uint32_t)(PLATFORM_CFG_TYPE_MASK)) + (uint64_t)((uint32_t)(PLATFORM_CFG_PRESET_MASK)) + (uint64_t)((uint32_t)(PLATFORM_CFG_RUG_IOEXP_BIT_MASK)) + (uint64_t)((uint32_t)(PLATFORM_CFG_TYPE_FROM_MANF_OTP)) + (uint64_t)((uint32_t)(PLATFORM_CFG_UPDATE_IO_CONFIG)) + (uint64_t)((uint32_t)(0xFE000040))) == (uint64_t)(((uint32_t)(PLATFORM_CFG_TYPE_MASK)) | ((uint32_t)(PLATFORM_CFG_PRESET_MASK)) | ((uint32_t)(PLATFORM_CFG_RUG_IOEXP_BIT_MASK)) | ((uint32_t)(PLATFORM_CFG_TYPE_FROM_MANF_OTP)) | ((uint32_t)(PLATFORM_CFG_UPDATE_IO_CONFIG)) | ((uint32_t)(0xFE000040))), "imx_flash_cfg.platformConfig: two of CONFIG_SYSTEM.platform.type, CONFIG_SYSTEM.platform.preset, CONFIG_SYSTEM.platform.ioExpanderBits, imx_flash_cfg.platformConfig not stored/pending, imx_flash_cfg.platformConfig not stored/pending, imx_flash_cfg.platformConfig not stored/pending overlap");
_Static_assert((((uint32_t)(PLATFORM_CFG_TYPE_MASK)) | ((uint32_t)(PLATFORM_CFG_PRESET_MASK)) | ((uint32_t)(PLATFORM_CFG_RUG_IOEXP_BIT_MASK)) | ((uint32_t)(PLATFORM_CFG_TYPE_FROM_MANF_OTP)) | ((uint32_t)(PLATFORM_CFG_UPDATE_IO_CONFIG)) | ((uint32_t)(0xFE000040))) == (0xFFFFFFFFu >> (32 - 8 * sizeof(((nvm_flash_cfg_t *)0)->platformConfig))), "imx_flash_cfg.platformConfig: a bit has no key and is not listed as not_stored or pending");
_Static_assert(((uint64_t)((uint32_t)(SENSOR_CFG_IMU_FAULT_DETECT_GYR)) + (uint64_t)((uint32_t)(SENSOR_CFG_IMU_FAULT_DETECT_ACC)) + (uint64_t)((uint32_t)(SENSOR_CFG_SENSOR_ROTATION_MASK)) + (uint64_t)((uint32_t)(SENSOR_CFG_GYR_FS_MASK)) + (uint64_t)((uint32_t)(SENSOR_CFG_GYR_DLPF_MASK)) + (uint64_t)((uint32_t)(SENSOR_CFG_ACC_FS_MASK)) + (uint64_t)((uint32_t)(SENSOR_CFG_ACC_DLPF_MASK)) + (uint64_t)((uint32_t)(SENSOR_CFG_DISABLE_MAGNETOMETER)) + (uint64_t)((uint32_t)(SENSOR_CFG_DISABLE_BAROMETER)) + (uint64_t)((uint32_t)(0xFC200088))) == (uint64_t)(((uint32_t)(SENSOR_CFG_IMU_FAULT_DETECT_GYR)) | ((uint32_t)(SENSOR_CFG_IMU_FAULT_DETECT_ACC)) | ((uint32_t)(SENSOR_CFG_SENSOR_ROTATION_MASK)) | ((uint32_t)(SENSOR_CFG_GYR_FS_MASK)) | ((uint32_t)(SENSOR_CFG_GYR_DLPF_MASK)) | ((uint32_t)(SENSOR_CFG_ACC_FS_MASK)) | ((uint32_t)(SENSOR_CFG_ACC_DLPF_MASK)) | ((uint32_t)(SENSOR_CFG_DISABLE_MAGNETOMETER)) | ((uint32_t)(SENSOR_CFG_DISABLE_BAROMETER)) | ((uint32_t)(0xFC200088))), "imx_flash_cfg.sensorConfig: two of CONFIG_INS.faultDetect.gyr, CONFIG_INS.faultDetect.acc, CONFIG_SENSORS.rotation, CONFIG_SENSORS.Imus.Gyr.PRIMARY.fullScale, CONFIG_SENSORS.Imus.Gyr.PRIMARY.dlpf, CONFIG_SENSORS.Imus.Acc.PRIMARY.fullScale, CONFIG_SENSORS.Imus.Acc.PRIMARY.dlpf, CONFIG_SENSORS.Mags.enabled, CONFIG_SENSORS.Baros.enabled, imx_flash_cfg.sensorConfig not stored/pending overlap");
_Static_assert((((uint32_t)(SENSOR_CFG_IMU_FAULT_DETECT_GYR)) | ((uint32_t)(SENSOR_CFG_IMU_FAULT_DETECT_ACC)) | ((uint32_t)(SENSOR_CFG_SENSOR_ROTATION_MASK)) | ((uint32_t)(SENSOR_CFG_GYR_FS_MASK)) | ((uint32_t)(SENSOR_CFG_GYR_DLPF_MASK)) | ((uint32_t)(SENSOR_CFG_ACC_FS_MASK)) | ((uint32_t)(SENSOR_CFG_ACC_DLPF_MASK)) | ((uint32_t)(SENSOR_CFG_DISABLE_MAGNETOMETER)) | ((uint32_t)(SENSOR_CFG_DISABLE_BAROMETER)) | ((uint32_t)(0xFC200088))) == (0xFFFFFFFFu >> (32 - 8 * sizeof(((nvm_flash_cfg_t *)0)->sensorConfig))), "imx_flash_cfg.sensorConfig: a bit has no key and is not listed as not_stored or pending");
_Static_assert(((uint64_t)((uint32_t)(SENSOR_CFG2_ACC_HG_FS_MASK)) + (uint64_t)((uint32_t)(SENSOR_CFG2_IMU_1_DISABLE)) + (uint64_t)((uint32_t)(SENSOR_CFG2_IMU_2_DISABLE)) + (uint64_t)((uint32_t)(SENSOR_CFG2_IMU_3_DISABLE)) + (uint64_t)((uint32_t)(SENSOR_CFG2_IMU_4_DISABLE)) + (uint64_t)((uint32_t)(SENSOR_CFG2_IMU_5_DISABLE)) + (uint64_t)((uint32_t)(0xFFFFE0F8))) == (uint64_t)(((uint32_t)(SENSOR_CFG2_ACC_HG_FS_MASK)) | ((uint32_t)(SENSOR_CFG2_IMU_1_DISABLE)) | ((uint32_t)(SENSOR_CFG2_IMU_2_DISABLE)) | ((uint32_t)(SENSOR_CFG2_IMU_3_DISABLE)) | ((uint32_t)(SENSOR_CFG2_IMU_4_DISABLE)) | ((uint32_t)(SENSOR_CFG2_IMU_5_DISABLE)) | ((uint32_t)(0xFFFFE0F8))), "imx_flash_cfg.sensorConfig2: two of CONFIG_SENSORS.Imus.Acc.HIGH_G.fullScale, CONFIG_SENSORS.Imu.IMU1.enabled, CONFIG_SENSORS.Imu.IMU2.enabled, CONFIG_SENSORS.Imu.IMU3.enabled, CONFIG_SENSORS.Imu.IMU4.enabled, CONFIG_SENSORS.Imu.IMU5.enabled, imx_flash_cfg.sensorConfig2 not stored/pending overlap");
_Static_assert((((uint32_t)(SENSOR_CFG2_ACC_HG_FS_MASK)) | ((uint32_t)(SENSOR_CFG2_IMU_1_DISABLE)) | ((uint32_t)(SENSOR_CFG2_IMU_2_DISABLE)) | ((uint32_t)(SENSOR_CFG2_IMU_3_DISABLE)) | ((uint32_t)(SENSOR_CFG2_IMU_4_DISABLE)) | ((uint32_t)(SENSOR_CFG2_IMU_5_DISABLE)) | ((uint32_t)(0xFFFFE0F8))) == (0xFFFFFFFFu >> (32 - 8 * sizeof(((nvm_flash_cfg_t *)0)->sensorConfig2))), "imx_flash_cfg.sensorConfig2: a bit has no key and is not listed as not_stored or pending");
_Static_assert(((uint64_t)((uint32_t)(IMU_SHOCK_OPTIONS_ENABLE)) + (uint64_t)((uint32_t)(IMU_SHOCK_OPTIONS_FAST_RECOVERY)) + (uint64_t)((uint32_t)(0x000000FC))) == (uint64_t)(((uint32_t)(IMU_SHOCK_OPTIONS_ENABLE)) | ((uint32_t)(IMU_SHOCK_OPTIONS_FAST_RECOVERY)) | ((uint32_t)(0x000000FC))), "imx_flash_cfg.imuShockOptions: two of CONFIG_INS.shock.enabled, CONFIG_INS.shock.fastRecovery, imx_flash_cfg.imuShockOptions not stored/pending overlap");
_Static_assert((((uint32_t)(IMU_SHOCK_OPTIONS_ENABLE)) | ((uint32_t)(IMU_SHOCK_OPTIONS_FAST_RECOVERY)) | ((uint32_t)(0x000000FC))) == (0xFFFFFFFFu >> (32 - 8 * sizeof(((nvm_flash_cfg_t *)0)->imuShockOptions))), "imx_flash_cfg.imuShockOptions: a bit has no key and is not listed as not_stored or pending");
_Static_assert(((uint64_t)((uint32_t)(WHEEL_CFG_BITS_ENABLE_CONTROL)) + (uint64_t)((uint32_t)(WHEEL_CFG_BITS_ENABLE_ENCODER)) + (uint64_t)((uint32_t)(WHEEL_CFG_BITS_ENCODER_SOURCE)) + (uint64_t)((uint32_t)(WHEEL_CFG_BITS_DIRECTION_REVERSE_LEFT)) + (uint64_t)((uint32_t)(WHEEL_CFG_BITS_DIRECTION_REVERSE_RIGHT)) + (uint64_t)((uint32_t)(0xFFFFF8F9))) == (uint64_t)(((uint32_t)(WHEEL_CFG_BITS_ENABLE_CONTROL)) | ((uint32_t)(WHEEL_CFG_BITS_ENABLE_ENCODER)) | ((uint32_t)(WHEEL_CFG_BITS_ENCODER_SOURCE)) | ((uint32_t)(WHEEL_CFG_BITS_DIRECTION_REVERSE_LEFT)) | ((uint32_t)(WHEEL_CFG_BITS_DIRECTION_REVERSE_RIGHT)) | ((uint32_t)(0xFFFFF8F9))), "imx_flash_cfg.wheelConfig.bits: two of CONFIG_GROUND_VEHICLE.controlEnabled, CONFIG_GROUND_VEHICLE.encoder.enabled, CONFIG_GROUND_VEHICLE.encoder.source, CONFIG_GROUND_VEHICLE.encoder.reverseLeft, CONFIG_GROUND_VEHICLE.encoder.reverseRight, imx_flash_cfg.wheelConfig.bits not stored/pending overlap");
_Static_assert((((uint32_t)(WHEEL_CFG_BITS_ENABLE_CONTROL)) | ((uint32_t)(WHEEL_CFG_BITS_ENABLE_ENCODER)) | ((uint32_t)(WHEEL_CFG_BITS_ENCODER_SOURCE)) | ((uint32_t)(WHEEL_CFG_BITS_DIRECTION_REVERSE_LEFT)) | ((uint32_t)(WHEEL_CFG_BITS_DIRECTION_REVERSE_RIGHT)) | ((uint32_t)(0xFFFFF8F9))) == (0xFFFFFFFFu >> (32 - 8 * sizeof(((nvm_flash_cfg_t *)0)->wheelConfig.bits))), "imx_flash_cfg.wheelConfig.bits: a bit has no key and is not listed as not_stored or pending");
_Static_assert(((uint64_t)((uint32_t)(IO_CONFIG_STROBE_TRIGGER_HIGH)) + (uint64_t)((uint32_t)(IO_CONFIG_G1G2_MASK)) + (uint64_t)((uint32_t)(IO_CONFIG_G5G8_MASK)) + (uint64_t)((uint32_t)(IO_CONFIG_G6G7_MASK)) + (uint64_t)((uint32_t)(IO_CONFIG_G9_MASK)) + (uint64_t)((uint32_t)(IO_CONFIG_G15_STROBE_INPUT)) + (uint64_t)((uint32_t)(IO_CONFIG_GNSS_TYPE_MASK << IO_CONFIG_GNSS1_TYPE_OFFSET)) + (uint64_t)((uint32_t)(IO_CONFIG_GNSS_SOURCE_MASK << IO_CONFIG_GNSS1_SOURCE_OFFSET)) + (uint64_t)((uint32_t)(IO_CFG_GNSS1_PPS_SOURCE_BITMASK)) + (uint64_t)((uint32_t)(IO_CONFIG_GNSS1_NO_INIT)) + (uint64_t)((uint32_t)(IO_CONFIG_GNSS_TYPE_MASK << IO_CONFIG_GNSS2_TYPE_OFFSET)) + (uint64_t)((uint32_t)(IO_CONFIG_GNSS_SOURCE_MASK << IO_CONFIG_GNSS2_SOURCE_OFFSET)) + (uint64_t)((uint32_t)(IO_CONFIG_GNSS2_NO_INIT)) + (uint64_t)((uint32_t)(0xE0000000))) == (uint64_t)(((uint32_t)(IO_CONFIG_STROBE_TRIGGER_HIGH)) | ((uint32_t)(IO_CONFIG_G1G2_MASK)) | ((uint32_t)(IO_CONFIG_G5G8_MASK)) | ((uint32_t)(IO_CONFIG_G6G7_MASK)) | ((uint32_t)(IO_CONFIG_G9_MASK)) | ((uint32_t)(IO_CONFIG_G15_STROBE_INPUT)) | ((uint32_t)(IO_CONFIG_GNSS_TYPE_MASK << IO_CONFIG_GNSS1_TYPE_OFFSET)) | ((uint32_t)(IO_CONFIG_GNSS_SOURCE_MASK << IO_CONFIG_GNSS1_SOURCE_OFFSET)) | ((uint32_t)(IO_CFG_GNSS1_PPS_SOURCE_BITMASK)) | ((uint32_t)(IO_CONFIG_GNSS1_NO_INIT)) | ((uint32_t)(IO_CONFIG_GNSS_TYPE_MASK << IO_CONFIG_GNSS2_TYPE_OFFSET)) | ((uint32_t)(IO_CONFIG_GNSS_SOURCE_MASK << IO_CONFIG_GNSS2_SOURCE_OFFSET)) | ((uint32_t)(IO_CONFIG_GNSS2_NO_INIT)) | ((uint32_t)(0xE0000000))), "imx_flash_cfg.ioConfig: two of CONFIG_IO.strobeActiveHigh, CONFIG_IO.Pin.G1G2.function, CONFIG_IO.Pin.G5G8.function, CONFIG_IO.Pin.G6G7.function, CONFIG_IO.Pin.G9.function, CONFIG_IO.Pin.G15.function, CONFIG_GNSS.Receiver.GNSS1.type, CONFIG_GNSS.Receiver.GNSS1.port, CONFIG_GNSS.Receiver.GNSS1.ppsPin, CONFIG_GNSS.Receiver.GNSS1.configure, CONFIG_GNSS.Receiver.GNSS2.type, CONFIG_GNSS.Receiver.GNSS2.port, CONFIG_GNSS.Receiver.GNSS2.configure, imx_flash_cfg.ioConfig not stored/pending overlap");
_Static_assert((((uint32_t)(IO_CONFIG_STROBE_TRIGGER_HIGH)) | ((uint32_t)(IO_CONFIG_G1G2_MASK)) | ((uint32_t)(IO_CONFIG_G5G8_MASK)) | ((uint32_t)(IO_CONFIG_G6G7_MASK)) | ((uint32_t)(IO_CONFIG_G9_MASK)) | ((uint32_t)(IO_CONFIG_G15_STROBE_INPUT)) | ((uint32_t)(IO_CONFIG_GNSS_TYPE_MASK << IO_CONFIG_GNSS1_TYPE_OFFSET)) | ((uint32_t)(IO_CONFIG_GNSS_SOURCE_MASK << IO_CONFIG_GNSS1_SOURCE_OFFSET)) | ((uint32_t)(IO_CFG_GNSS1_PPS_SOURCE_BITMASK)) | ((uint32_t)(IO_CONFIG_GNSS1_NO_INIT)) | ((uint32_t)(IO_CONFIG_GNSS_TYPE_MASK << IO_CONFIG_GNSS2_TYPE_OFFSET)) | ((uint32_t)(IO_CONFIG_GNSS_SOURCE_MASK << IO_CONFIG_GNSS2_SOURCE_OFFSET)) | ((uint32_t)(IO_CONFIG_GNSS2_NO_INIT)) | ((uint32_t)(0xE0000000))) == (0xFFFFFFFFu >> (32 - 8 * sizeof(((nvm_flash_cfg_t *)0)->ioConfig))), "imx_flash_cfg.ioConfig: a bit has no key and is not listed as not_stored or pending");
_Static_assert(((uint64_t)((uint32_t)(IO_CFG2_G11_BITMASK)) + (uint64_t)((uint32_t)(IO_CFG2_G12_BITMASK)) + (uint64_t)((uint32_t)(IO_CFG2_G13_BITMASK)) + (uint64_t)((uint32_t)(IO_CFG2_USE_GNSS2_AS_SOURCE)) + (uint64_t)((uint32_t)(IO_CFG2_GNSS2_PPS_SOURCE_BITMASK))) == (uint64_t)(((uint32_t)(IO_CFG2_G11_BITMASK)) | ((uint32_t)(IO_CFG2_G12_BITMASK)) | ((uint32_t)(IO_CFG2_G13_BITMASK)) | ((uint32_t)(IO_CFG2_USE_GNSS2_AS_SOURCE)) | ((uint32_t)(IO_CFG2_GNSS2_PPS_SOURCE_BITMASK))), "imx_flash_cfg.ioConfig2: two of CONFIG_IO.Pin.G11.function, CONFIG_IO.Pin.G12.function, CONFIG_IO.Pin.G13.function, CONFIG_GNSS.primaryReceiver, CONFIG_GNSS.Receiver.GNSS2.ppsPin overlap");
_Static_assert((((uint32_t)(IO_CFG2_G11_BITMASK)) | ((uint32_t)(IO_CFG2_G12_BITMASK)) | ((uint32_t)(IO_CFG2_G13_BITMASK)) | ((uint32_t)(IO_CFG2_USE_GNSS2_AS_SOURCE)) | ((uint32_t)(IO_CFG2_GNSS2_PPS_SOURCE_BITMASK))) == (0xFFFFFFFFu >> (32 - 8 * sizeof(((nvm_flash_cfg_t *)0)->ioConfig2))), "imx_flash_cfg.ioConfig2: a bit has no key and is not listed as not_stored or pending");
_Static_assert(((uint64_t)((uint32_t)(RTK_CFG_BITS_ROVER_MODE_MASK)) + (uint64_t)((uint32_t)(RTK_CFG_BITS_BASE_POS_MOVING)) + (uint64_t)((uint32_t)(RTK_CFG_BITS_RTK_BASE_IS_IDENTICAL_TO_ROVER)) + (uint64_t)((uint32_t)(RTK_CFG_BITS_GNSS_PORT_PASS_THROUGH)) + (uint64_t)((uint32_t)(RTK_CFG_BITS_MULTIPATH_MITIGATION)) + (uint64_t)((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS1_UBLOX_SER0)) + (uint64_t)((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS1_UBLOX_SER1)) + (uint64_t)((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS1_UBLOX_SER2)) + (uint64_t)((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS1_UBLOX_USB)) + (uint64_t)((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS1_RTCM3_SER0)) + (uint64_t)((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS1_RTCM3_SER1)) + (uint64_t)((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS1_RTCM3_SER2)) + (uint64_t)((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS1_RTCM3_USB)) + (uint64_t)((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS2_UBLOX_SER0)) + (uint64_t)((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS2_UBLOX_SER1)) + (uint64_t)((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS2_UBLOX_SER2)) + (uint64_t)((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS2_UBLOX_USB)) + (uint64_t)((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS2_RTCM3_SER0)) + (uint64_t)((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS2_RTCM3_SER1)) + (uint64_t)((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS2_RTCM3_SER2)) + (uint64_t)((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS2_RTCM3_USB)) + (uint64_t)((uint32_t)(RTK_CFG_BITS_RESERVED1)) + (uint64_t)((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_RTCM3_CUR_PORT_MASK)) + (uint64_t)((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_RTCM3_CLEAR_CUR_PORT)) + (uint64_t)((uint32_t)(0xF0000000))) == (uint64_t)(((uint32_t)(RTK_CFG_BITS_ROVER_MODE_MASK)) | ((uint32_t)(RTK_CFG_BITS_BASE_POS_MOVING)) | ((uint32_t)(RTK_CFG_BITS_RTK_BASE_IS_IDENTICAL_TO_ROVER)) | ((uint32_t)(RTK_CFG_BITS_GNSS_PORT_PASS_THROUGH)) | ((uint32_t)(RTK_CFG_BITS_MULTIPATH_MITIGATION)) | ((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS1_UBLOX_SER0)) | ((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS1_UBLOX_SER1)) | ((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS1_UBLOX_SER2)) | ((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS1_UBLOX_USB)) | ((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS1_RTCM3_SER0)) | ((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS1_RTCM3_SER1)) | ((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS1_RTCM3_SER2)) | ((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS1_RTCM3_USB)) | ((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS2_UBLOX_SER0)) | ((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS2_UBLOX_SER1)) | ((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS2_UBLOX_SER2)) | ((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS2_UBLOX_USB)) | ((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS2_RTCM3_SER0)) | ((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS2_RTCM3_SER1)) | ((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS2_RTCM3_SER2)) | ((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS2_RTCM3_USB)) | ((uint32_t)(RTK_CFG_BITS_RESERVED1)) | ((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_RTCM3_CUR_PORT_MASK)) | ((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_RTCM3_CLEAR_CUR_PORT)) | ((uint32_t)(0xF0000000))), "imx_flash_cfg.RTKCfgBits: two of CONFIG_GNSS.Rtk.mode, CONFIG_GNSS.Rtk.basePositionMoving, CONFIG_GNSS.Rtk.baseIdenticalToRover, CONFIG_GNSS.Rtk.passThrough, CONFIG_GNSS.Rtk.multipathMitigation, CONFIG_GNSS.Rtk.BaseOutput.GNSS1_UBLOX.ser0, CONFIG_GNSS.Rtk.BaseOutput.GNSS1_UBLOX.ser1, CONFIG_GNSS.Rtk.BaseOutput.GNSS1_UBLOX.ser2, CONFIG_GNSS.Rtk.BaseOutput.GNSS1_UBLOX.usb, CONFIG_GNSS.Rtk.BaseOutput.GNSS1_RTCM3.ser0, CONFIG_GNSS.Rtk.BaseOutput.GNSS1_RTCM3.ser1, CONFIG_GNSS.Rtk.BaseOutput.GNSS1_RTCM3.ser2, CONFIG_GNSS.Rtk.BaseOutput.GNSS1_RTCM3.usb, CONFIG_GNSS.Rtk.BaseOutput.GNSS2_UBLOX.ser0, CONFIG_GNSS.Rtk.BaseOutput.GNSS2_UBLOX.ser1, CONFIG_GNSS.Rtk.BaseOutput.GNSS2_UBLOX.ser2, CONFIG_GNSS.Rtk.BaseOutput.GNSS2_UBLOX.usb, CONFIG_GNSS.Rtk.BaseOutput.GNSS2_RTCM3.ser0, CONFIG_GNSS.Rtk.BaseOutput.GNSS2_RTCM3.ser1, CONFIG_GNSS.Rtk.BaseOutput.GNSS2_RTCM3.ser2, CONFIG_GNSS.Rtk.BaseOutput.GNSS2_RTCM3.usb, imx_flash_cfg.RTKCfgBits not stored/pending, imx_flash_cfg.RTKCfgBits not stored/pending, imx_flash_cfg.RTKCfgBits not stored/pending, imx_flash_cfg.RTKCfgBits not stored/pending overlap");
_Static_assert((((uint32_t)(RTK_CFG_BITS_ROVER_MODE_MASK)) | ((uint32_t)(RTK_CFG_BITS_BASE_POS_MOVING)) | ((uint32_t)(RTK_CFG_BITS_RTK_BASE_IS_IDENTICAL_TO_ROVER)) | ((uint32_t)(RTK_CFG_BITS_GNSS_PORT_PASS_THROUGH)) | ((uint32_t)(RTK_CFG_BITS_MULTIPATH_MITIGATION)) | ((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS1_UBLOX_SER0)) | ((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS1_UBLOX_SER1)) | ((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS1_UBLOX_SER2)) | ((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS1_UBLOX_USB)) | ((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS1_RTCM3_SER0)) | ((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS1_RTCM3_SER1)) | ((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS1_RTCM3_SER2)) | ((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS1_RTCM3_USB)) | ((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS2_UBLOX_SER0)) | ((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS2_UBLOX_SER1)) | ((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS2_UBLOX_SER2)) | ((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS2_UBLOX_USB)) | ((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS2_RTCM3_SER0)) | ((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS2_RTCM3_SER1)) | ((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS2_RTCM3_SER2)) | ((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS2_RTCM3_USB)) | ((uint32_t)(RTK_CFG_BITS_RESERVED1)) | ((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_RTCM3_CUR_PORT_MASK)) | ((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_RTCM3_CLEAR_CUR_PORT)) | ((uint32_t)(0xF0000000))) == (0xFFFFFFFFu >> (32 - 8 * sizeof(((nvm_flash_cfg_t *)0)->RTKCfgBits))), "imx_flash_cfg.RTKCfgBits: a bit has no key and is not listed as not_stored or pending");
