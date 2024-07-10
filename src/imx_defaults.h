#ifndef __IMX_DEFAULTS_H_
#define __IMX_DEFAULTS_H_

#ifndef NAPP

#ifdef __cplusplus
extern "C" {
#endif

#include "data_sets.h"

#define tNAV_MIN_PERIOD_IMX5_MS_NAV_MODE    7       // W/ GPS
#define tNAV_MIN_PERIOD_IMX5_MS_AHRS_MODE   5       // No GPS
#define tNAV_MIN_PERIOD_IMX5_MS_VRS_MODE    4       // No GPS or magnetometer
#define tNAV_MIN_PERIOD_INS3_MS_NAV_MODE    2       // W/ GPS
#define tNAV_MIN_PERIOD_INS3_MS_AHRS_MODE   2       // No GPS
#define tNAV_MIN_PERIOD_INS3_MS_VRS_MODE    2       // No GPS or magnetometer
#ifdef IMX_5
#define tNAV_MIN_PERIOD_MS_NAV_MODE         tNAV_MIN_PERIOD_IMX5_MS_NAV_MODE        // W/ GPS
#define tNAV_MIN_PERIOD_MS_AHRS_MODE        tNAV_MIN_PERIOD_IMX5_MS_AHRS_MODE       // No GPS
#define tNAV_MIN_PERIOD_MS_VRS_MODE         tNAV_MIN_PERIOD_IMX5_MS_VRS_MODE        // No GPS or magnetometer
#define tMAINT_MAX_RUN_TIME_US              100000  // Used to increment gap count and indicate error
#define tNAV_DEFAULT_PERIOD_MS              tNAV_MIN_PERIOD_MS_NAV_MODE      // Reliable / safe period for operation
#else // uINS-3
#define tNAV_MIN_PERIOD_MS_NAV_MODE         tNAV_MIN_PERIOD_INS3_MS_NAV_MODE        // W/ GPS
#define tNAV_MIN_PERIOD_MS_AHRS_MODE        tNAV_MIN_PERIOD_INS3_MS_AHRS_MODE       // No GPS
#define tNAV_MIN_PERIOD_MS_VRS_MODE         tNAV_MIN_PERIOD_INS3_MS_VRS_MODE        // No GPS or magnetometer
#define tMAINT_MAX_RUN_TIME_US              40000   // Used to increment gap count and indicate error. uINS-3 onboard RTK takes ~20ms max maint task.
#define tNAV_DEFAULT_PERIOD_MS              4      // Reliable / safe period for operation
#endif

int platformConfigTypeValid(uint32_t platformConfig);
void platformConfigErrorCheck(uint32_t *platformConfig);
void platformConfigToFlashCfgIoConfig(uint32_t *ioConfig, uint32_t platformConfig);
void platformConfigTypeToFlashCfgIoConfig(uint32_t *ioConfig, uint32_t platformType);
uint32_t platformConfigTypeToDefaultPlatformConfig(uint32_t platformType);
uint32_t platformConfigTypeToDefaultPlatformPreset(uint32_t platformType);

uint32_t minNavOutputMs(nvm_flash_cfg_t *cfg);
void setNavOutputRateMs(nvm_flash_cfg_t *cfg, sys_params_t *sysParams, uint32_t dtMs);


#ifdef __cplusplus
}
#endif

#endif // NAPP

#endif // __IMX_DEFAULTS_H_

