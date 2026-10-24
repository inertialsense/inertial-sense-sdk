/**
 * @file cbor_did_flash_cfg.h
 * @brief FLASH_CFG: legacy DID_FLASH_CONFIG (nvm_flash_cfg_t), served by IMX-5, IMX-6. Field keys, schema and recorded
 *        defaults.
 *
 * Generated from the CBOR key registry by is-common scripts/cbor_registry.py -- do not edit.
 *
 * @copyright Copyright (c) 2026 Inertial Sense, Inc.
 */

#ifndef CBOR_DID_FLASH_CFG_H
#define CBOR_DID_FLASH_CFG_H

#include "cbor/cbor_registry.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @name FLASH_CFG field keys */
/** @{ */
#define CBOR_FLASH_CFG__STARTUPIMUODR                0        //!< startupImuOdr: uint32, default 1
#define CBOR_FLASH_CFG__STARTUPNAVDTMS               1        //!< startupNavDtMs: uint32, default set by the firmware
#define CBOR_FLASH_CFG__SER0BAUDRATE                 2        //!< ser0BaudRate: uint32, default 921600
#define CBOR_FLASH_CFG__SER1BAUDRATE                 3        //!< ser1BaudRate: uint32, default 921600
#define CBOR_FLASH_CFG__INSROTATION                  4        //!< insRotation: float32[3], default [0, 0, 0]
#define CBOR_FLASH_CFG__INSOFFSET                    5        //!< insOffset: float32[3], default [0, 0, 0]
#define CBOR_FLASH_CFG__GNSS1ANTOFFSET               6        //!< gnss1AntOffset: float32[3], default [0, 0, 0]
#define CBOR_FLASH_CFG__DYNAMICMODEL                 7        //!< dynamicModel: uint8, default 8
#define CBOR_FLASH_CFG__DEBUG                        8        //!< debug: uint8, default 0
#define CBOR_FLASH_CFG__GNSSSATSIGCONST              9        //!< gnssSatSigConst: uint16, default 5119
#define CBOR_FLASH_CFG__SYSCFGBITS                   10       //!< sysCfgBits: uint32, default 256
#define CBOR_FLASH_CFG__REFLLA                       11       //!< refLla: float64[3], default set by the firmware
#define CBOR_FLASH_CFG__LASTLLA                      12       //!< lastLla: float64[3], default set by the firmware
#define CBOR_FLASH_CFG__LASTLLATIMEOFWEEKMS          13       //!< lastLlaTimeOfWeekMs: uint32, default set by the firmware
#define CBOR_FLASH_CFG__LASTLLAWEEK                  14       //!< lastLlaWeek: uint32, default set by the firmware
#define CBOR_FLASH_CFG__LASTLLAUPDATEDISTANCE        15       //!< lastLlaUpdateDistance: float32, default 1000
#define CBOR_FLASH_CFG__IOCONFIG                     16       //!< ioConfig: uint32, default set by the firmware
#define CBOR_FLASH_CFG__PLATFORMCONFIG               17       //!< platformConfig: uint32, default set by the firmware
#define CBOR_FLASH_CFG__GNSS2ANTOFFSET               18       //!< gnss2AntOffset: float32[3], default [0, 0, 0]
#define CBOR_FLASH_CFG__ZEROVELROTATION              19       //!< zeroVelRotation: float32[3], default [0, 0, 0]
#define CBOR_FLASH_CFG__ZEROVELOFFSET                20       //!< zeroVelOffset: float32[3], default [0, 0, 0]
#define CBOR_FLASH_CFG__GNSSTIMEUSERDELAY            21       //!< gnssTimeUserDelay: float32, default 0
#define CBOR_FLASH_CFG__MAGDECLINATION               22       //!< magDeclination: float32, default 0
#define CBOR_FLASH_CFG__GNSSTIMESYNCPERIODMS         23       //!< gnssTimeSyncPeriodMs: uint32, default 1000
#define CBOR_FLASH_CFG__STARTUPGNSSDTMS              24       //!< startupGnssDtMs: uint32, default 200
#define CBOR_FLASH_CFG__RTKCFGBITS                   25       //!< RTKCfgBits: uint32, default 0
#define CBOR_FLASH_CFG__SENSORCONFIG                 26       //!< sensorConfig: uint32, default 119
#define CBOR_FLASH_CFG__GNSSMINIMUMELEVATION         27       //!< gnssMinimumElevation: float32, default 0.17453292
#define CBOR_FLASH_CFG__SER2BAUDRATE                 28       //!< ser2BaudRate: uint32, default 921600
#define CBOR_FLASH_CFG__WHEELCONFIG_BITS             29       //!< wheelConfig.bits: uint32, default 0
#define CBOR_FLASH_CFG__WHEELCONFIG_TRANSFORM_E_B2W  30       //!< wheelConfig.transform.e_b2w: float32[3], default [0, 0, 0]
#define CBOR_FLASH_CFG__WHEELCONFIG_TRANSFORM_E_B2W_SIGMA 31       //!< wheelConfig.transform.e_b2w_sigma: float32[3], default [0.0174532924, 0.0174532924, 0.0174532924]
#define CBOR_FLASH_CFG__WHEELCONFIG_TRANSFORM_T_B2W  32       //!< wheelConfig.transform.t_b2w: float32[3], default [0, 0, 0]
#define CBOR_FLASH_CFG__WHEELCONFIG_TRANSFORM_T_B2W_SIGMA 33       //!< wheelConfig.transform.t_b2w_sigma: float32[3], default [0.300000012, 0.300000012, 0.300000012]
#define CBOR_FLASH_CFG__WHEELCONFIG_TRACK_WIDTH      34       //!< wheelConfig.track_width: float32, default 0
#define CBOR_FLASH_CFG__WHEELCONFIG_RADIUS           35       //!< wheelConfig.radius: float32, default 0
#define CBOR_FLASH_CFG__MAGINTERFERENCETHRESHOLD     36       //!< magInterferenceThreshold: float32, default 3
#define CBOR_FLASH_CFG__MAGCALIBRATIONQUALITYTHRESHOLD 37       //!< magCalibrationQualityThreshold: float32, default 10
#define CBOR_FLASH_CFG__GNSSCN0MINIMUM               38       //!< gnssCn0Minimum: uint8, default 30
#define CBOR_FLASH_CFG__GNSSCN0DYNMINOFFSET          39       //!< gnssCn0DynMinOffset: uint8, default 20
#define CBOR_FLASH_CFG__IMUREJECTTHRESHGYROLOW       40       //!< imuRejectThreshGyroLow: uint8, default 2
#define CBOR_FLASH_CFG__IMUREJECTTHRESHGYROHIGH      41       //!< imuRejectThreshGyroHigh: uint8, default 3
#define CBOR_FLASH_CFG__IMUSHOCKDETECTLATENCYMS      42       //!< imuShockDetectLatencyMs: uint8, default 10
#define CBOR_FLASH_CFG__IMUSHOCKREJECTLATCHMS        43       //!< imuShockRejectLatchMs: uint8, default 5
#define CBOR_FLASH_CFG__IMUSHOCKOPTIONS              44       //!< imuShockOptions: uint8, default 2
#define CBOR_FLASH_CFG__IMUSHOCKDELTAACCHIGHTHRESHOLD 45       //!< imuShockDeltaAccHighThreshold: uint8, default 20
#define CBOR_FLASH_CFG__IMUSHOCKDELTAACCLOWTHRESHOLD 46       //!< imuShockDeltaAccLowThreshold: uint8, default 3
#define CBOR_FLASH_CFG__IMUSHOCKDELTAGYROHIGHTHRESHOLD 47       //!< imuShockDeltaGyroHighThreshold: uint8, default 120
#define CBOR_FLASH_CFG__IMUSHOCKDELTAGYROLOWTHRESHOLD 48       //!< imuShockDeltaGyroLowThreshold: uint8, default 12
#define CBOR_FLASH_CFG__IOCONFIG2                    49       //!< ioConfig2: uint8, default set by the firmware
/** @} */

#define CBOR_FLASH_CFG_NAME_COUNT                    50       //!< entries in g_cborNames_FLASH_CFG

/** FLASH_CFG field names, in registry order */
extern const cbor_registry_name_t g_cborNames_FLASH_CFG[CBOR_FLASH_CFG_NAME_COUNT];

/** Schema binding FLASH_CFG's fields to nvm_flash_cfg_t */
extern const cbor_schema_t g_cborSchema_FLASH_CFG;

/** The registry's recorded defaults; fields set by the firmware are zero */
extern const nvm_flash_cfg_t g_cborDefaults_FLASH_CFG;

/** One per g_cborSchema_FLASH_CFG field: 1 if the registry records a fixed default, 0 if the firmware sets it */
extern const uint8_t g_cborDefaultFixed_FLASH_CFG[];

#ifdef __cplusplus
}
#endif

#endif /* CBOR_DID_FLASH_CFG_H */
