/**
 * @file cbor_did_can_config.c
 * @brief CAN_CONFIG: schema, recorded defaults and field names.
 *
 * Generated from the CBOR key registry by is-common scripts/cbor_registry.py -- do not edit.
 *
 * @copyright Copyright (c) 2026 Inertial Sense, Inc.
 */

#include <stddef.h>
#include "cbor/cbor_registry.h"

static const cbor_field_t s_fields[] = {
    { 0, CBOR_T_UINT, 2, NUM_CIDS, offsetof(can_config_t, can_period_mult), NULL },
    { 1, CBOR_T_UINT, 4, NUM_CIDS, offsetof(can_config_t, can_transmit_address), NULL },
    { 2, CBOR_T_UINT, 2, 0, offsetof(can_config_t, can_setting), NULL },
    { 3, CBOR_T_UINT, 4, 0, offsetof(can_config_t, can_receive_address), NULL },
};

const cbor_schema_t g_cborSchema_CAN_CONFIG = { s_fields, sizeof(s_fields) / sizeof(s_fields[0]) };

/* Each entry matches its can_config_t member in type and size */
_Static_assert(_Generic(((can_config_t *)0)->can_period_mult[0], uint16_t: 1, default: 0), "CAN_CONFIG.can_period_mult type");
_Static_assert(sizeof(((can_config_t *)0)->can_period_mult) == sizeof(uint16_t) * (NUM_CIDS), "CAN_CONFIG.can_period_mult count");
_Static_assert(_Generic(((can_config_t *)0)->can_transmit_address[0], uint32_t: 1, default: 0), "CAN_CONFIG.can_transmit_address type");
_Static_assert(sizeof(((can_config_t *)0)->can_transmit_address) == sizeof(uint32_t) * (NUM_CIDS), "CAN_CONFIG.can_transmit_address count");
_Static_assert(_Generic(((can_config_t *)0)->can_setting, uint16_t: 1, default: 0), "CAN_CONFIG.can_setting type");
_Static_assert(_Generic(((can_config_t *)0)->can_receive_address, uint32_t: 1, default: 0), "CAN_CONFIG.can_receive_address type");

const can_config_t g_cborDefaults_CAN_CONFIG = {
    .can_setting = 100,
    .can_receive_address = 0,
};

const uint8_t g_cborDefaultFixed_CAN_CONFIG[] = {
    1, 1, 1, 1,
};

const cbor_registry_name_t g_cborNames_CAN_CONFIG[CBOR_CAN_CONFIG_NAME_COUNT] = {
    { 0x0000, "can_period_mult" },
    { 0x0001, "can_transmit_address" },
    { 0x0002, "can_setting" },
    { 0x0003, "can_receive_address" },
};
