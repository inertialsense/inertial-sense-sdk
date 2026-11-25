/**
 * @file can_config.h
 * @brief Mapping of DID_CAN_CONFIG (can_config_t) to the domains that hold its values: CONFIG_CAN. A domain whose keys
 *        live in several structs is read and written through each of their mappings.
 *
 * Generated from the CBOR key registry by is-common scripts/cbor_registry.py -- do not edit.
 *
 * @copyright Copyright (c) 2026 Inertial Sense, Inc.
 */

#ifndef CBOR_DID_CAN_CONFIG_H
#define CBOR_DID_CAN_CONFIG_H

#include <stdbool.h>
#include "cbor/cbor_registry.h"

#ifdef __cplusplus
extern "C" {
#endif

void cbor_can_config_to_config_can(cbor_config_can_t *view, const can_config_t *src);
bool cbor_can_config_fits_config_can(const cbor_config_can_t *view);
bool cbor_can_config_from_config_can(can_config_t *dst, const cbor_config_can_t *view);

#ifdef __cplusplus
}
#endif

#endif /* CBOR_DID_CAN_CONFIG_H */
