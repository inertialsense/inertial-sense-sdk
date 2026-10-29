/**
 * @file cbor_did_rmc_stored.h
 * @brief RMC_STORED: CBOR-only DID, served by IMX-5, IMX-6. Field keys.
 *
 * Generated from the CBOR key registry by is-common scripts/cbor_registry.py -- do not edit.
 *
 * @copyright Copyright (c) 2026 Inertial Sense, Inc.
 */

#ifndef CBOR_DID_RMC_STORED_H
#define CBOR_DID_RMC_STORED_H

#include "cbor/cbor_registry.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @name RMC_STORED field keys */
/** @{ */
#define CBOR_RMC_STORED__RMC_SAVE_PORTS              0x8000   //!< RMC_SAVE_PORTS: array of uint8, up to RMC_CFG_MAX_PORTS, default set by the protocol
#define CBOR_RMC_STORED__RMC_PRESET                  0x8001   //!< RMC_PRESET: uint8 (rmc_preset), default NONE
#define CBOR_RMC_STORED__NMEA_SPEED_FILTER           0x8002   //!< NMEA_SPEED_FILTER: uint8 (nmea_speed_filter), default DEFAULT
/** @} */

#define CBOR_RMC_STORED_NAME_COUNT                   3        //!< entries in g_cborNames_RMC_STORED

/** RMC_STORED field names, in registry order */
extern const cbor_registry_name_t g_cborNames_RMC_STORED[CBOR_RMC_STORED_NAME_COUNT];

#ifdef __cplusplus
}
#endif

#endif /* CBOR_DID_RMC_STORED_H */
