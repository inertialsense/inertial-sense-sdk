/**
 * @file cbor_did_state.h
 * @brief STATE: CBOR-only DID, served by IMX-5, IMX-6. Field keys.
 *
 * Generated from the CBOR key registry by is-common scripts/cbor_registry.py -- do not edit.
 *
 * @copyright Copyright (c) 2026 Inertial Sense, Inc.
 */

#ifndef CBOR_DID_STATE_H
#define CBOR_DID_STATE_H

#include "cbor/cbor_registry.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @name STATE field keys */
/** @{ */
#define CBOR_STATE__LASTPOSITION                     0        //!< lastPosition: map
#define CBOR_STATE__LASTPOSITION_LLA                 0        //!< lla: float64[3], default [0, 0, 0]
#define CBOR_STATE__LASTPOSITION_TIMEOFWEEKMS        1        //!< timeOfWeekMs: uint32, default 0
#define CBOR_STATE__LASTPOSITION_WEEK                2        //!< week: uint32, default 0
#define CBOR_STATE__MAG                              1        //!< Mag: map
#define CBOR_STATE__MAG_MAG1                         0        //!< MAG1: map
#define CBOR_STATE__MAG_MAG1_COVERAGE                0        //!< coverage: map
#define CBOR_STATE__MAG_MAG1_COVERAGE_GRID           0        //!< grid: uint8, default 0
#define CBOR_STATE__MAG_MAG1_COVERAGE_COUNTS         1        //!< counts: uint8[64], default 0
#define CBOR_STATE__MAG_MAG1_COVERAGE_SAMPLECOUNT    2        //!< sampleCount: uint32, default 0
#define CBOR_STATE__MAG_MAG1_SOLVER                  1        //!< solver: map
#define CBOR_STATE__MAG_MAG1_SOLVER_DTD              0        //!< dtd: float32[100], default 0
/** @} */

#define CBOR_STATE_NAME_COUNT                        12       //!< entries in g_cborNames_STATE

/** STATE field names, in registry order */
extern const cbor_registry_name_t g_cborNames_STATE[CBOR_STATE_NAME_COUNT];

/** The parent of each g_cborNames_STATE entry: the index of its enclosing map's entry, or 0xFFFF */
extern const uint16_t g_cborNameParents_STATE[CBOR_STATE_NAME_COUNT];

/** STATE.lastPosition: one member per key, in registry types */
typedef struct
{
    double          lla[3];                 //!< key 0: float64[3], default [0, 0, 0]
    uint32_t        timeOfWeekMs;           //!< key 1: uint32, default 0
    uint32_t        week;                   //!< key 2: uint32, default 0
} cbor_state_lastposition_t;

/** STATE: one member per key, in registry types */
typedef struct
{
    cbor_state_lastposition_t lastPosition;  //!< key 0: map
    cbor_keyset_mag_state_t Mag[1];  //!< key 1: map of mag_state by mag_id
} cbor_state_t;

/** Schema of every STATE key, over cbor_state_t; a device stores and accepts a subset of them */
extern const cbor_schema_t g_cborSchema_STATE;

/** The registry's recorded defaults; keys whose default the firmware sets are zero */
extern const cbor_state_t g_cborDefaults_STATE;

/** One per leaf key, depth first in key order: 1 if the registry records a fixed default, 0 if the firmware sets it */
extern const uint8_t g_cborDefaultFixed_STATE[7];

#ifdef __cplusplus
}
#endif

#endif /* CBOR_DID_STATE_H */
