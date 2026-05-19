#ifndef __IMX_DEFAULTS_H_
#define __IMX_DEFAULTS_H_

#ifndef NAPP

#ifdef __cplusplus
extern "C" {
#endif

#include "data_sets.h"

#define tNAV_MIN_OUTPUT_PERIOD_IMX5_MS_NAV_MODE    7       // W/ GPS
#define tNAV_MIN_OUTPUT_PERIOD_IMX5_MS_AHRS_MODE   5       // No GPS
#define tNAV_MIN_OUTPUT_PERIOD_IMX5_MS_VRS_MODE    4       // No GPS or magnetometer

#define tNAV_MIN_OUTPUT_PERIOD_IMX6_MS_NAV_MODE    2       // W/ GPS
#define tNAV_MIN_OUTPUT_PERIOD_IMX6_MS_AHRS_MODE   2       // No GPS
#define tNAV_MIN_OUTPUT_PERIOD_IMX6_MS_VRS_MODE    2       // No GPS or magnetometer

#if defined(IMX_5)
#define tNAV_MIN_OUTPUT_PERIOD_MS_NAV_MODE      tNAV_MIN_OUTPUT_PERIOD_IMX5_MS_NAV_MODE     // W/ GPS
#define tNAV_MIN_OUTPUT_PERIOD_MS_AHRS_MODE     tNAV_MIN_OUTPUT_PERIOD_IMX5_MS_AHRS_MODE    // No GPS
#define tNAV_MIN_OUTPUT_PERIOD_MS_VRS_MODE      tNAV_MIN_OUTPUT_PERIOD_IMX5_MS_VRS_MODE     // No GPS or magnetometer
#else   // IMX_6
#define tNAV_MIN_OUTPUT_PERIOD_MS_NAV_MODE      tNAV_MIN_OUTPUT_PERIOD_IMX6_MS_NAV_MODE     // W/ GPS
#define tNAV_MIN_OUTPUT_PERIOD_MS_AHRS_MODE     tNAV_MIN_OUTPUT_PERIOD_IMX6_MS_AHRS_MODE    // No GPS
#define tNAV_MIN_OUTPUT_PERIOD_MS_VRS_MODE      tNAV_MIN_OUTPUT_PERIOD_IMX6_MS_VRS_MODE     // No GPS or magnetometer
#endif

#define tNAV_DEFAULT_PERIOD_MS                  tNAV_MIN_OUTPUT_PERIOD_MS_NAV_MODE          // Reliable / safe period for operation
#define tMAINT_MAX_RUN_TIME_US                  100000                                      // Used to increment gap count and indicate error


int imxPlatformConfigTypeValid(uint32_t platformConfig);
void imxPlatformConfigErrorCheck(uint32_t *platformConfig);
void imxPlatformConfigToFlashCfgIoConfig(uint32_t *ioConfig, uint8_t *ioConfig2, uint32_t platformConfig);
void imxPlatformConfigTypeToFlashCfgIoConfig(uint32_t *ioConfig, uint8_t* ioConfig2, uint32_t platformType);
uint32_t imxPlatformConfigTypeToDefaultPlatformConfig(uint32_t platformType);
uint32_t imxPlatformConfigTypeToDefaultPlatformPreset(uint32_t platformType);

uint32_t imxMinNavOutputMs(nvm_flash_cfg_t *cfg);


#ifdef __cplusplus
}
#endif

#endif // NAPP

#endif // __IMX_DEFAULTS_H_

