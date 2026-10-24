/**
 * @file cbor_did_rmc.h
 * @brief RMC: CBOR-only DID, served by IMX-5, IMX-6, GPX-1. Field keys.
 *
 * Generated from the CBOR key registry by is-common scripts/cbor_registry.py -- do not edit.
 *
 * @copyright Copyright (c) 2026 Inertial Sense, Inc.
 */

#ifndef CBOR_DID_RMC_H
#define CBOR_DID_RMC_H

#include "cbor/cbor_registry.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @name RMC field keys */
/** @{ */
#define CBOR_RMC__RMC_PRESET                         0x8001   //!< RMC_PRESET: uint8 (rmc_preset), default NONE
#define CBOR_RMC__NMEA_SPEED_FILTER                  0x8002   //!< NMEA_SPEED_FILTER: uint8 (nmea_speed_filter), default DEFAULT
/** @} */

#define CBOR_RMC_NAME_COUNT                          2        //!< entries in g_cborNames_RMC

/** RMC field names, in registry order */
extern const cbor_registry_name_t g_cborNames_RMC[CBOR_RMC_NAME_COUNT];

#ifdef __cplusplus
}
#endif

#endif /* CBOR_DID_RMC_H */
