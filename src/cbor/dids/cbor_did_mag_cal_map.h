/**
 * @file cbor_did_mag_cal_map.h
 * @brief MAG_CAL_MAP: CBOR-only DID. Field keys.
 *
 * Generated from the CBOR key registry by is-common scripts/cbor_registry.py -- do not edit.
 *
 * @copyright Copyright (c) 2026 Inertial Sense, Inc.
 */

#ifndef CBOR_DID_MAG_CAL_MAP_H
#define CBOR_DID_MAG_CAL_MAP_H

#include "cbor/cbor_registry.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @name MAG_CAL_MAP field keys */
/** @{ */
#define CBOR_MAG_CAL_MAP__CELLS                      0        //!< cells: bytes, default 0
#define CBOR_MAG_CAL_MAP__SAMPLECOUNT                1        //!< sampleCount: uint32, default 0
/** @} */

#define CBOR_MAG_CAL_MAP_NAME_COUNT                  2        //!< entries in g_cborNames_MAG_CAL_MAP

/** MAG_CAL_MAP field names, in registry order */
extern const cbor_registry_name_t g_cborNames_MAG_CAL_MAP[CBOR_MAG_CAL_MAP_NAME_COUNT];

#ifdef __cplusplus
}
#endif

#endif /* CBOR_DID_MAG_CAL_MAP_H */
