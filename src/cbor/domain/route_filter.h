/**
 * @file route_filter.h
 * @brief ROUTE_FILTER: domain, served by IMX-5, IMX-6. Field keys.
 *
 * Generated from the CBOR key registry by is-common scripts/cbor_registry.py -- do not edit.
 *
 * @copyright Copyright (c) 2026 Inertial Sense, Inc.
 */

#ifndef CBOR_DOMAIN_ROUTE_FILTER_H
#define CBOR_DOMAIN_ROUTE_FILTER_H

#include "cbor/cbor_registry.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @name ROUTE_FILTER field keys */
/** @{ */
#define CBOR_ROUTE_FILTER__MATCH_DEV                 0        //!< MATCH_DEV: uint64, default set by the protocol
#define CBOR_ROUTE_FILTER__MATCH_MSG                 1        //!< MATCH_MSG: array of uint8, up to 3, default set by the protocol
#define CBOR_ROUTE_FILTER__MATCH_DIDS                2        //!< MATCH_DIDS: array of uint16, up to 16, default set by the protocol
#define CBOR_ROUTE_FILTER__ACTION                    3        //!< ACTION: uint8 (route_action), default DENY
/** @} */

#define CBOR_ROUTE_FILTER_NAME_COUNT                 4        //!< entries in g_cborNames_ROUTE_FILTER

/** ROUTE_FILTER field names, in registry order */
extern const cbor_registry_name_t g_cborNames_ROUTE_FILTER[CBOR_ROUTE_FILTER_NAME_COUNT];

/** The parent of each g_cborNames_ROUTE_FILTER entry: the index of its enclosing map's entry, or 0xFFFF */
extern const uint16_t g_cborNameParents_ROUTE_FILTER[CBOR_ROUTE_FILTER_NAME_COUNT];

#ifdef __cplusplus
}
#endif

#endif /* CBOR_DOMAIN_ROUTE_FILTER_H */
