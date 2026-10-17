/**
 * @file cbor_registry_names.c
 * @brief Field name lookup across every DID in the registry. SDK only: firmware does not carry the names.
 *
 * Generated from the CBOR key registry by is-common scripts/cbor_registry.py -- do not edit.
 *
 * @copyright Copyright (c) 2026 Inertial Sense, Inc.
 */

#include <string.h>
#include "cbor/cbor_registry.h"

/** One DID's name table */
typedef struct
{
    uint16_t                    did;    //!< root DID key
    const cbor_registry_name_t *names;  //!< the DID's field names
    uint16_t                    count;  //!< entries in names
} cbor_registry_did_names_t;

static const cbor_registry_did_names_t s_dids[] = {
    { CBOR_DID_FLASH_CFG, g_cborNames_FLASH_CFG, CBOR_FLASH_CFG_NAME_COUNT },
    { CBOR_DID_CAN_CONFIG, g_cborNames_CAN_CONFIG, CBOR_CAN_CONFIG_NAME_COUNT },
    { CBOR_DID_GPX_FLASH_CFG, g_cborNames_GPX_FLASH_CFG, CBOR_GPX_FLASH_CFG_NAME_COUNT },
    { CBOR_DID_RMC_STORED, g_cborNames_RMC_STORED, CBOR_RMC_STORED_NAME_COUNT },
    { CBOR_DID_RMC, g_cborNames_RMC, CBOR_RMC_NAME_COUNT },
    { CBOR_DID_ROUTE_FILTER, g_cborNames_ROUTE_FILTER, CBOR_ROUTE_FILTER_NAME_COUNT },
};

/**
 * @brief Finds a DID's name table.
 * @param did  root DID key
 * @return the table, or NULL if the DID has no field keys
 */
static const cbor_registry_did_names_t *did_names(uint16_t did)
{
    for (size_t i = 0; i < sizeof(s_dids) / sizeof(s_dids[0]); i++)
        if (s_dids[i].did == did)
            return &s_dids[i];
    return NULL;
}

const char *cbor_registry_field_name(uint16_t did, uint16_t key)
{
    const cbor_registry_did_names_t *t = did_names(did);
    for (uint16_t i = 0; t && i < t->count; i++)
        if (t->names[i].key == key)
            return t->names[i].name;
    return NULL;
}

int cbor_registry_field_key(uint16_t did, const char *name)
{
    const cbor_registry_did_names_t *t = did_names(did);
    for (uint16_t i = 0; t && i < t->count; i++)
        if (strcmp(t->names[i].name, name) == 0)
            return t->names[i].key;
    return -1;
}
