/**
 * @file cbor_did_gpx_flash_cfg.h
 * @brief GPX_FLASH_CFG: legacy DID_GPX_FLASH_CFG (gpx_flash_cfg_t), served by GPX-1. Field keys, schema and recorded
 *        defaults.
 *
 * Generated from the CBOR key registry by is-common scripts/cbor_registry.py -- do not edit.
 *
 * @copyright Copyright (c) 2026 Inertial Sense, Inc.
 */

#ifndef CBOR_DID_GPX_FLASH_CFG_H
#define CBOR_DID_GPX_FLASH_CFG_H

#include "cbor/cbor_registry.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @name GPX_FLASH_CFG field keys */
/** @{ */
#define CBOR_GPX_FLASH_CFG__SER0BAUDRATE             0        //!< ser0BaudRate: uint32, default 921600
#define CBOR_GPX_FLASH_CFG__SER1BAUDRATE             1        //!< ser1BaudRate: uint32, default 921600
#define CBOR_GPX_FLASH_CFG__SER2BAUDRATE             2        //!< ser2BaudRate: uint32, default 921600
#define CBOR_GPX_FLASH_CFG__STARTUPGNSSDTMS          3        //!< startupGnssDtMs: uint32, default 200
#define CBOR_GPX_FLASH_CFG__GNSS1ANTOFFSET           4        //!< gnss1AntOffset: float32[3], default [0, 0, 0]
#define CBOR_GPX_FLASH_CFG__GNSS2ANTOFFSET           5        //!< gnss2AntOffset: float32[3], default [0, 0, 0]
#define CBOR_GPX_FLASH_CFG__GNSSSATSIGCONST          6        //!< gnssSatSigConst: uint16, default 5119
#define CBOR_GPX_FLASH_CFG__DYNAMICMODEL             7        //!< dynamicModel: uint8, default 8
#define CBOR_GPX_FLASH_CFG__DEBUG                    8        //!< debug: uint8, default 32
#define CBOR_GPX_FLASH_CFG__GNSSTIMESYNCPERIODMS     9        //!< gnssTimeSyncPeriodMs: uint32, default 1000
#define CBOR_GPX_FLASH_CFG__GNSSTIMEUSERDELAY        10       //!< gnssTimeUserDelay: float32, default 0
#define CBOR_GPX_FLASH_CFG__GNSSMINIMUMELEVATION     11       //!< gnssMinimumElevation: float32, default 0.17453292
#define CBOR_GPX_FLASH_CFG__RTKCFGBITS               12       //!< RTKCfgBits: uint32, default 0
#define CBOR_GPX_FLASH_CFG__GNSSCN0MINIMUM           13       //!< gnssCn0Minimum: uint8, default 30
#define CBOR_GPX_FLASH_CFG__GNSSCN0DYNMINOFFSET      14       //!< gnssCn0DynMinOffset: uint8, default 20
#define CBOR_GPX_FLASH_CFG__SYSCFGBITS               15       //!< sysCfgBits: uint32, default 0
#define CBOR_GPX_FLASH_CFG__REFLLA                   16       //!< refLla: float64[3], default [0, 0, 0]
/** @} */

#define CBOR_GPX_FLASH_CFG_NAME_COUNT                17       //!< entries in g_cborNames_GPX_FLASH_CFG

/** GPX_FLASH_CFG field names, in registry order */
extern const cbor_registry_name_t g_cborNames_GPX_FLASH_CFG[CBOR_GPX_FLASH_CFG_NAME_COUNT];

/** Schema binding GPX_FLASH_CFG's fields to gpx_flash_cfg_t */
extern const cbor_schema_t g_cborSchema_GPX_FLASH_CFG;

/** The registry's recorded defaults; fields set by the firmware are zero */
extern const gpx_flash_cfg_t g_cborDefaults_GPX_FLASH_CFG;

/** One per g_cborSchema_GPX_FLASH_CFG field: 1 if the registry records a fixed default, 0 if the firmware sets it */
extern const uint8_t g_cborDefaultFixed_GPX_FLASH_CFG[];

#ifdef __cplusplus
}
#endif

#endif /* CBOR_DID_GPX_FLASH_CFG_H */
