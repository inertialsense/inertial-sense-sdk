/**
 * @file cbor_did_config_mag.h
 * @brief CONFIG_MAG: CBOR-only DID, served by IMX-5, IMX-6. Field keys.
 *
 * Generated from the CBOR key registry by is-common scripts/cbor_registry.py -- do not edit.
 *
 * @copyright Copyright (c) 2026 Inertial Sense, Inc.
 */

#ifndef CBOR_DID_CONFIG_MAG_H
#define CBOR_DID_CONFIG_MAG_H

#include "cbor/cbor_registry.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @name CONFIG_MAG field keys */
/** @{ */
#define CBOR_CONFIG_MAG__CONTINUOUSCAL               0        //!< continuousCal: bool, default false
#define CBOR_CONFIG_MAG__AUTORECAL                   1        //!< autoRecal: bool, default false
#define CBOR_CONFIG_MAG__RECALMODE                   2        //!< recalMode: uint8, default 1
#define CBOR_CONFIG_MAG__INTERFERENCETHRESHOLD       3        //!< interferenceThreshold: float32, default 3
#define CBOR_CONFIG_MAG__CALQUALITYTHRESHOLD         4        //!< calQualityThreshold: float32, default 10
#define CBOR_CONFIG_MAG__DECLINATION                 5        //!< declination: map
#define CBOR_CONFIG_MAG__DECLINATION_VALUERAD        0        //!< valueRad: float32, default 0
#define CBOR_CONFIG_MAG__DECLINATION_ESTIMATE        1        //!< estimate: bool, default true
#define CBOR_CONFIG_MAG__DECLINATION_USEWMM          2        //!< useWmm: bool, default false
/** @} */

#define CBOR_CONFIG_MAG_NAME_COUNT                   9        //!< entries in g_cborNames_CONFIG_MAG

/** CONFIG_MAG field names, in registry order */
extern const cbor_registry_name_t g_cborNames_CONFIG_MAG[CBOR_CONFIG_MAG_NAME_COUNT];

/** CONFIG_MAG.declination: one member per key, in registry types */
typedef struct
{
    float           valueRad;               //!< key 0: float32, default 0
    uint8_t         estimate;               //!< key 1: bool, default true
    uint8_t         useWmm;                 //!< key 2: bool, default false
} cbor_config_mag_declination_t;

/** CONFIG_MAG: one member per key, in registry types */
typedef struct
{
    uint8_t         continuousCal;          //!< key 0: bool, default false
    uint8_t         autoRecal;              //!< key 1: bool, default false
    uint8_t         recalMode;              //!< key 2: uint8, default 1
    float           interferenceThreshold;  //!< key 3: float32, default 3
    float           calQualityThreshold;    //!< key 4: float32, default 10
    cbor_config_mag_declination_t declination;  //!< key 5: map
} cbor_config_mag_t;

/** Schema of every CONFIG_MAG key, over cbor_config_mag_t; a device stores and accepts a subset of them */
extern const cbor_schema_t g_cborSchema_CONFIG_MAG;

/** The registry's recorded defaults; keys whose default the firmware sets are zero */
extern const cbor_config_mag_t g_cborDefaults_CONFIG_MAG;

/** One per leaf key, depth first in key order: 1 if the registry records a fixed default, 0 if the firmware sets it */
extern const uint8_t g_cborDefaultFixed_CONFIG_MAG[8];

#ifdef __cplusplus
}
#endif

#endif /* CBOR_DID_CONFIG_MAG_H */
