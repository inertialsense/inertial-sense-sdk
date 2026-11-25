/**
 * @file cbor_did_config_io.h
 * @brief CONFIG_IO: CBOR-only DID, served by IMX-5, IMX-6. Field keys.
 *
 * Generated from the CBOR key registry by is-common scripts/cbor_registry.py -- do not edit.
 *
 * @copyright Copyright (c) 2026 Inertial Sense, Inc.
 */

#ifndef CBOR_DID_CONFIG_IO_H
#define CBOR_DID_CONFIG_IO_H

#include "cbor/cbor_registry.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @name CONFIG_IO field keys */
/** @{ */
#define CBOR_CONFIG_IO__STROBEACTIVEHIGH             0        //!< strobeActiveHigh: bool, default set by the firmware
#define CBOR_CONFIG_IO__PIN                          1        //!< Pin: map
#define CBOR_CONFIG_IO__PIN_G1G2                     0        //!< G1G2: map
#define CBOR_CONFIG_IO__PIN_G1G2_FUNCTION            0        //!< function: uint8, default set by the firmware
#define CBOR_CONFIG_IO__PIN_G5G8                     1        //!< G5G8: map
#define CBOR_CONFIG_IO__PIN_G5G8_FUNCTION            0        //!< function: uint8, default set by the firmware
#define CBOR_CONFIG_IO__PIN_G6G7                     2        //!< G6G7: map
#define CBOR_CONFIG_IO__PIN_G6G7_FUNCTION            0        //!< function: uint8, default set by the firmware
#define CBOR_CONFIG_IO__PIN_G9                       3        //!< G9: map
#define CBOR_CONFIG_IO__PIN_G9_FUNCTION              0        //!< function: uint8, default set by the firmware
#define CBOR_CONFIG_IO__PIN_G11                      4        //!< G11: map
#define CBOR_CONFIG_IO__PIN_G11_FUNCTION             0        //!< function: uint8, default set by the firmware
#define CBOR_CONFIG_IO__PIN_G12                      5        //!< G12: map
#define CBOR_CONFIG_IO__PIN_G12_FUNCTION             0        //!< function: uint8, default set by the firmware
#define CBOR_CONFIG_IO__PIN_G13                      6        //!< G13: map
#define CBOR_CONFIG_IO__PIN_G13_FUNCTION             0        //!< function: uint8, default set by the firmware
#define CBOR_CONFIG_IO__PIN_G15                      7        //!< G15: map
#define CBOR_CONFIG_IO__PIN_G15_FUNCTION             0        //!< function: uint8, default set by the firmware
/** @} */

#define CBOR_CONFIG_IO_NAME_COUNT                    18       //!< entries in g_cborNames_CONFIG_IO

/** CONFIG_IO field names, in registry order */
extern const cbor_registry_name_t g_cborNames_CONFIG_IO[CBOR_CONFIG_IO_NAME_COUNT];

/** The parent of each g_cborNames_CONFIG_IO entry: the index of its enclosing map's entry, or 0xFFFF */
extern const uint16_t g_cborNameParents_CONFIG_IO[CBOR_CONFIG_IO_NAME_COUNT];

/** CONFIG_IO: one member per key, in registry types */
typedef struct
{
    uint8_t         strobeActiveHigh;       //!< key 0: bool, default set by the firmware
    cbor_keyset_pin_t Pin[8];  //!< key 1: map of pin by pin_id
} cbor_config_io_t;

/** Schema of every CONFIG_IO key, over cbor_config_io_t; a device stores and accepts a subset of them */
extern const cbor_schema_t g_cborSchema_CONFIG_IO;

/** The registry's recorded defaults; keys whose default the firmware sets are zero */
extern const cbor_config_io_t g_cborDefaults_CONFIG_IO;

/** One per leaf key, depth first in key order: 1 if the registry records a fixed default, 0 if the firmware sets it */
extern const uint8_t g_cborDefaultFixed_CONFIG_IO[9];

#ifdef __cplusplus
}
#endif

#endif /* CBOR_DID_CONFIG_IO_H */
