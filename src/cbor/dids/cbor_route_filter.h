/**
 * @file cbor_route_filter.h
 * @brief ROUTE_FILTER: the ordered rules a forwarding device applies before passing a message to a downstream device.
 *
 * Each rule matches a downstream device (device filter), optionally a set of message operations and a set of DIDs, and
 * names an action. Rules are checked in order and the first match wins; when none matches, the message is forwarded.
 *
 * Design: is-common/docs_internal/cbor/CBOR_PROTOCOL_DESIGN.md (section 8).
 *
 * @copyright Copyright (c) 2026 Inertial Sense, Inc.
 */

#ifndef CBOR_ROUTE_FILTER_H
#define CBOR_ROUTE_FILTER_H

#include <stddef.h>
#include <stdint.h>

#include "cbor/cbor_codec.h"

#ifdef __cplusplus
extern "C" {
#endif

#define CBOR_ROUTE_RULE_MSGS_MAX    3   //!< Operations one rule can name (registry: MATCH_MSG max_count)
#define CBOR_ROUTE_RULE_DIDS_MAX    16  //!< DIDs one rule can name (registry: MATCH_DIDS max_count)

/** One ROUTE_FILTER rule. A count of 0 matches every operation / every DID. */
typedef struct
{
    uint64_t    matchDev;                           //!< Device filter the downstream device must match
    uint8_t     msgCount;
    uint8_t     msgs[CBOR_ROUTE_RULE_MSGS_MAX];     //!< Operations (eCborMsgType) the rule applies to
    uint8_t     didCount;
    uint16_t    dids[CBOR_ROUTE_RULE_DIDS_MAX];     //!< Root DID keys the rule applies to
    uint8_t     action;                             //!< eCborRouteAction
} cbor_route_rule_t;

/**
 * @brief The action for forwarding one DID of a message to a downstream device.
 * @param devId  the downstream device's unique ID
 * @param op     the message's operation (eCborMsgType, without flags)
 * @param did    the root DID key
 * @return CBOR_ROUTE_ACTION_ALLOW or CBOR_ROUTE_ACTION_DENY
 */
uint8_t cbor_route_action(const cbor_route_rule_t *rules, size_t ruleCount, uint64_t devId, uint8_t op, uint16_t did);

/**
 * @brief Encodes rules as the ROUTE_FILTER map (rule n at key n), within size bytes. A selection picks rules (one key)
 *        or fields of a rule (two keys); a rule that does not fit is marked RESP_TOO_LARGE.
 * @return bytes written, or CBOR_CODEC_ERR_NO_SPACE if not even the map header fits
 */
int cbor_route_encode(const cbor_route_rule_t *rules, size_t ruleCount, const cbor_path_t *paths, size_t pathCount,
                      uint8_t *buf, size_t size);

/** @brief Writes the ROUTE_FILTER field keys (the rule indices) as an array. @return >= 0, or NANOCBOR_ERR_END */
int cbor_route_encode_keys(size_t ruleCount, nanocbor_encoder_t *enc);

#ifdef __cplusplus
}
#endif

#endif /* CBOR_ROUTE_FILTER_H */
