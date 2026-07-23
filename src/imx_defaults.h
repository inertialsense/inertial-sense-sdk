/**
 * @file imx_defaults.h
 * @brief IMX-5/IMX-6 default navigation output periods and platform-config helper functions.
 *
 * Defines the minimum (fastest) navigation-output periods for each IMX hardware variant and
 * operating mode (NAV/AHRS/VRS), selects the active set via the IMX_5 build flag, and declares
 * the platformConfig <-> nvm_flash_cfg_t IO-config conversion helpers used at startup and when
 * applying a platform-type preset.
 *
 * @author Inertial Sense, Inc.
 * @copyright Copyright (c) 2014-2026 Inertial Sense, Inc. - http://inertialsense.com
 */

#ifndef __IMX_DEFAULTS_H_
#define __IMX_DEFAULTS_H_

#ifndef NAPP

#ifdef __cplusplus
extern "C" {
#endif

#include "data_sets.h"

/** IMX-5 minimum (fastest) navigation-output periods (ms) per operating mode. */
#define tNAV_MIN_OUTPUT_PERIOD_IMX5_MS_NAV_MODE    7       //!< W/ GPS
#define tNAV_MIN_OUTPUT_PERIOD_IMX5_MS_AHRS_MODE   5       //!< No GPS
#define tNAV_MIN_OUTPUT_PERIOD_IMX5_MS_VRS_MODE    4       //!< No GPS or magnetometer

/** IMX-6 minimum (fastest) navigation-output periods (ms) per operating mode. */
#define tNAV_MIN_OUTPUT_PERIOD_IMX6_MS_NAV_MODE    2       //!< W/ GPS
#define tNAV_MIN_OUTPUT_PERIOD_IMX6_MS_AHRS_MODE   2       //!< No GPS
#define tNAV_MIN_OUTPUT_PERIOD_IMX6_MS_VRS_MODE    2       //!< No GPS or magnetometer

/** Active navigation-output period set (ms), selected at build time by the IMX_5 flag. */
#if defined(IMX_5)
#define tNAV_MIN_OUTPUT_PERIOD_MS_NAV_MODE      tNAV_MIN_OUTPUT_PERIOD_IMX5_MS_NAV_MODE     //!< W/ GPS
#define tNAV_MIN_OUTPUT_PERIOD_MS_AHRS_MODE     tNAV_MIN_OUTPUT_PERIOD_IMX5_MS_AHRS_MODE    //!< No GPS
#define tNAV_MIN_OUTPUT_PERIOD_MS_VRS_MODE      tNAV_MIN_OUTPUT_PERIOD_IMX5_MS_VRS_MODE     //!< No GPS or magnetometer
#define tNAV_DEFAULT_PERIOD_MS                  tNAV_MIN_OUTPUT_PERIOD_MS_NAV_MODE          //!< Reliable / safe period for operation
#else   // IMX_6
#define tNAV_MIN_OUTPUT_PERIOD_MS_NAV_MODE      tNAV_MIN_OUTPUT_PERIOD_IMX6_MS_NAV_MODE     //!< W/ GPS
#define tNAV_MIN_OUTPUT_PERIOD_MS_AHRS_MODE     tNAV_MIN_OUTPUT_PERIOD_IMX6_MS_AHRS_MODE    //!< No GPS
#define tNAV_MIN_OUTPUT_PERIOD_MS_VRS_MODE      tNAV_MIN_OUTPUT_PERIOD_IMX6_MS_VRS_MODE     //!< No GPS or magnetometer
#define tNAV_DEFAULT_PERIOD_MS                  5                                           //!< Reliable / safe period for operation
#endif

#define tMAINT_MAX_RUN_TIME_US                  100000                                      //!< Maximum maintenance-task run time (us); used to increment gap count and indicate error


/**
 * @brief Check whether a platformConfig value's platform-type field names a valid, known platform.
 * @param platformConfig Raw platformConfig value (see @ref ePlatformConfig / PLATFORM_CFG_TYPE_MASK).
 * @return Non-zero if the platform type is valid, 0 otherwise.
 */
int imxPlatformConfigTypeValid(uint32_t platformConfig);

/**
 * @brief Validate a platformConfig value in place, resetting it to a known-good default if invalid.
 * @param platformConfig Pointer to the platformConfig value to check and, if necessary, correct.
 */
void imxPlatformConfigErrorCheck(uint32_t *platformConfig);

/**
 * @brief Derive the nvm_flash_cfg_t IO-config fields implied by a full platformConfig value.
 * @param ioConfig     Output: primary IO configuration bitmask (see nvm_flash_cfg_t::ioConfig).
 * @param ioConfig2    Output: secondary IO configuration bitmask (see nvm_flash_cfg_t::ioConfig2).
 * @param platformConfig Input platformConfig value to derive the IO configuration from.
 */
void imxPlatformConfigToFlashCfgIoConfig(uint32_t *ioConfig, uint8_t *ioConfig2, uint32_t platformConfig);

/**
 * @brief Derive the nvm_flash_cfg_t IO-config fields implied by just a platform-type value.
 * @param ioConfig    Output: primary IO configuration bitmask (see nvm_flash_cfg_t::ioConfig).
 * @param ioConfig2   Output: secondary IO configuration bitmask (see nvm_flash_cfg_t::ioConfig2).
 * @param platformType Platform-type value (see ePlatformConfig's PLATFORM_CFG_TYPE_MASK field).
 */
void imxPlatformConfigTypeToFlashCfgIoConfig(uint32_t *ioConfig, uint8_t* ioConfig2, uint32_t platformType);

/**
 * @brief Get the default platformConfig value for a given platform type.
 * @param platformType Platform-type value (see ePlatformConfig's PLATFORM_CFG_TYPE_MASK field).
 * @return Default platformConfig value for that platform type.
 */
uint32_t imxPlatformConfigTypeToDefaultPlatformConfig(uint32_t platformType);

/**
 * @brief Get the default platformConfig preset (hardware wiring/pinout variant) for a given platform type.
 * @param platformType Platform-type value (see ePlatformConfig's PLATFORM_CFG_TYPE_MASK field).
 * @return Default preset value for that platform type (0 for types without multiple presets, e.g. non-RUG3).
 */
uint32_t imxPlatformConfigTypeToDefaultPlatformPreset(uint32_t platformType);

/**
 * @brief Compute the minimum allowed navigation-output period (ms) for the device's current configuration.
 * @param cfg Pointer to the device's current flash configuration.
 * @return Minimum navigation-output period in milliseconds, given cfg's platform type and enabled sensors.
 */
uint32_t imxMinNavOutputMs(nvm_flash_cfg_t *cfg);


#ifdef __cplusplus
}
#endif

#endif // NAPP

#endif // __IMX_DEFAULTS_H_

