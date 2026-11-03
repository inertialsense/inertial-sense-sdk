/**
 * @file cbor_did_can_config.h
 * @brief CAN_CONFIG: legacy DID_CAN_CONFIG (can_config_t), served by IMX-5, IMX-6. Field keys, schema and recorded
 *        defaults.
 *
 * Generated from the CBOR key registry by is-common scripts/cbor_registry.py -- do not edit.
 *
 * @copyright Copyright (c) 2026 Inertial Sense, Inc.
 */

#ifndef CBOR_DID_CAN_CONFIG_H
#define CBOR_DID_CAN_CONFIG_H

#include "cbor/cbor_registry.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @name CAN_CONFIG field keys */
/** @{ */
#define CBOR_CAN_CONFIG__CAN_PERIOD_MULT             0        //!< can_period_mult: uint16[NUM_CIDS], default 0
#define CBOR_CAN_CONFIG__CAN_TRANSMIT_ADDRESS        1        //!< can_transmit_address: uint32[NUM_CIDS], default 0
#define CBOR_CAN_CONFIG__CAN_SETTING                 2        //!< can_setting: uint16, default 100
#define CBOR_CAN_CONFIG__CAN_RECEIVE_ADDRESS         3        //!< can_receive_address: uint32, default 0
/** @} */

#define CBOR_CAN_CONFIG_NAME_COUNT                   4        //!< entries in g_cborNames_CAN_CONFIG

/** CAN_CONFIG field names, in registry order */
extern const cbor_registry_name_t g_cborNames_CAN_CONFIG[CBOR_CAN_CONFIG_NAME_COUNT];

/** Schema binding CAN_CONFIG's fields to can_config_t */
extern const cbor_schema_t g_cborSchema_CAN_CONFIG;

/** The registry's recorded defaults; fields set by the firmware are zero */
extern const can_config_t g_cborDefaults_CAN_CONFIG;

/** One per g_cborSchema_CAN_CONFIG field: 1 if the registry records a fixed default, 0 if the firmware sets it */
extern const uint8_t g_cborDefaultFixed_CAN_CONFIG[];

#ifdef __cplusplus
}
#endif

#endif /* CBOR_DID_CAN_CONFIG_H */
