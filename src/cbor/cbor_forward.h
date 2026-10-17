/**
 * @file cbor_forward.h
 * @brief Forwarding CBOR messages between devices: the copy a forwarder sends downstream, with its arrival port pushed
 *        onto MSG_ROUTE and denied DIDs left out, and popping MSG_ROUTE to send a reply back upstream.
 *
 * Design: is-common/docs_internal/cbor/CBOR_PROTOCOL_DESIGN.md
 *
 * @copyright Copyright (c) 2026 Inertial Sense, Inc.
 */

#ifndef CBOR_FORWARD_H
#define CBOR_FORWARD_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "cbor_envelope.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Maximum DIDs a forwarder reports as denied for one message */
#define CBOR_FORWARD_DENIED_MAX     8

/** Outcome of cbor_msg_forward_build() */
typedef struct
{
    uint16_t    denied[CBOR_FORWARD_DENIED_MAX];    //!< DIDs left out of the forwarded copy
    uint8_t     deniedCount;
    uint16_t    forwardedDids;                      //!< DIDs kept in the forwarded copy
} cbor_forward_result_t;

/** cbor_msg_forward_build(): MSG_ROUTE already holds CBOR_ROUTE_MAX entries */
#define CBOR_MSG_ROUTE_FULL         (-17)

/**
 * @brief Builds the copy of a message a forwarder sends downstream: the envelope unchanged except MSG_ROUTE, which gets
 *        arrivalPort appended, and the DIDs for which deny() returns false. Envelope keys stay ahead of the DIDs.
 * @param deny   returns true to leave a DID out (NULL = forward every DID)
 * @return length written to out; 0 if every DID was denied (nothing to forward); CBOR_MSG_ROUTE_FULL; or another
 *         negative eCborCodecResult (malformed input, or out too small)
 */
int cbor_msg_forward_build(const uint8_t *msg, size_t len, uint8_t arrivalPort, bool (*deny)(void *ctx, uint16_t did),
                           void *ctx, uint8_t *out, size_t outSize, cbor_forward_result_t *res);

/**
 * @brief Removes the last MSG_ROUTE entry of a message in place (a forwarder sending a reply back upstream). MSG_ROUTE
 *        is dropped when it becomes empty.
 * @param len   [in/out] message length
 * @param port  [out] the removed entry: the port to send the message out of
 * @return CBOR_CODEC_OK, CBOR_CODEC_ERR_VALUE if the message has no MSG_ROUTE, or CBOR_CODEC_ERR_MALFORMED
 */
int cbor_msg_route_pop(uint8_t *msg, size_t *len, uint8_t *port);

#ifdef __cplusplus
}
#endif

#endif /* CBOR_FORWARD_H */
