/**
 * @file cbor_registry.c
 * @brief Compile-time checks that the registry's transport DID and constants match their C definitions.
 *
 * Generated from the CBOR key registry by is-common scripts/cbor_registry.py -- do not edit.
 *
 * @copyright Copyright (c) 2026 Inertial Sense, Inc.
 */

#include "ISComm.h"
#include "cbor/cbor_registry.h"
#include "cbor/dids/cbor_rmc_stored.h"

_Static_assert(DID_CBOR == 129, "registry transport.isb_did does not match DID_CBOR");
_Static_assert((NUM_CIDS) == 27, "registry constant NUM_CIDS does not match the C definition");
_Static_assert((NMEA_MSG_ID_COUNT) == 20, "registry constant NMEA_MSG_ID_COUNT does not match the C definition");
_Static_assert((RMC_CFG_MAX_PORTS) == 6, "registry constant RMC_CFG_MAX_PORTS does not match the C definition");
_Static_assert((ISB_MAX_DATASET_SIZE) == 2040, "registry constant ISB_MAX_DATASET_SIZE does not match the C definition");
