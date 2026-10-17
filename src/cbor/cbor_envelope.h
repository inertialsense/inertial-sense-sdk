/**
 * @file cbor_envelope.h
 * @brief The envelope of a CBOR message carried by DID_CBOR, and device-filter matching.
 *
 * A message is one CBOR map. Keys at 0x8000 and above form the envelope (MSG_*, options); keys below 0x8000 are DIDs.
 * MSG_TYPE is an operation (DATA, GET, SUBSCRIBE) plus flags; a reply carries the operation it answers with the REPLY
 * flag, and is never applied, acknowledged or answered. Absent envelope keys take the defaults of design doc section 4.
 *
 * Design: is-common/docs_internal/cbor/CBOR_PROTOCOL_DESIGN.md
 *
 * @copyright Copyright (c) 2026 Inertial Sense, Inc.
 */

#ifndef CBOR_ENVELOPE_H
#define CBOR_ENVELOPE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "cbor_codec.h"
#include "cbor_registry.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Maximum MSG_ROUTE depth (forwarding hops) */
#define CBOR_ROUTE_MAX              4

/** Message envelope. Absent keys take their defaults (design doc section 4). */
typedef struct
{
    uint8_t     type;                       //!< eCborMsgType operation | eCborMsgTypeFlag flags; on parse, inferred when
                                            //!< absent (values: DATA, keys only: GET)
    uint8_t     ackType;                    //!< eCborAckType; default ACK_MINIMAL
    uint8_t     status;                     //!< eCborMsgStatus; envelope-level outcome
    uint8_t     routeDepth;                 //!< entries in route (0 = not forwarded)
    uint8_t     route[CBOR_ROUTE_MAX];      //!< MSG_ROUTE stack, one port per forwarder
    bool        hasSrc;                     //!< MSG_SRC_DEV_ID present (absent = the host)
    bool        hasDst;                     //!< MSG_DST_DEV_FILTER present (absent = the link peer only)
    bool        hasReqId;                   //!< MSG_REQ_ID present (absent = unsolicited)
    bool        hasTimeGps;                 //!< MSG_TIME_GPS_NS present
    bool        hasUptime;                  //!< MSG_UPTIME_NS present
    bool        noTime;                     //!< MSG_OPTION_NO_TIME: the sender asks for no time on the reply
    uint16_t    reqId;                      //!< MSG_REQ_ID, echoed by replies
    uint64_t    srcDevId;                   //!< MSG_SRC_DEV_ID: the sender's unique ID
    uint64_t    dstFilter;                  //!< MSG_DST_DEV_FILTER: devices the message is for (0 = the host)
    uint64_t    timeGpsNs;                  //!< MSG_TIME_GPS_NS: ns since the GPS epoch
    uint64_t    uptimeNs;                   //!< MSG_UPTIME_NS: the sender's uptime in ns
    uint16_t    didCount;                   //!< on parse: DID keys in the message
} cbor_envelope_t;

/**
 * @param type  a MSG_TYPE value
 * @return its operation (eCborMsgType)
 */
static inline uint8_t cbor_msg_op(uint8_t type)         { return (uint8_t)(type & CBOR_MSG_TYPE_OP_MASK); }

/**
 * @param type  MSG_TYPE of a request
 * @return the MSG_TYPE of a reply to it
 */
static inline uint8_t cbor_msg_reply_type(uint8_t type) { return (uint8_t)((unsigned)CBOR_MSG_TYPE_FLAG_REPLY | type); }

/**
 * @param type  a MSG_TYPE value
 * @return true if it marks a reply
 */
static inline bool cbor_msg_is_reply(uint8_t type)      { return (type & CBOR_MSG_TYPE_FLAG_REPLY) != 0; }

/**
 * @brief Parses the envelope of a message.
 * @param msg  the message
 * @param len  its length
 * @param env  [out] the envelope, with defaults for absent keys
 * @return CBOR_CODEC_OK; CBOR_CODEC_ERR_VALUE if an envelope key has an invalid value (the rest of env is still
 *         filled); CBOR_CODEC_ERR_MALFORMED if msg is not exactly one well-formed CBOR map
 */
int cbor_msg_parse_envelope(const uint8_t *msg, size_t len, cbor_envelope_t *env);

/**
 * @brief Writes the envelope's present keys as map entries (no map header), in protocol order.
 * @param enc  encoder to write to
 * @param env  the envelope
 * @return number of entries written, or a negative value if the encoder ran out of space
 */
int cbor_msg_encode_envelope(nanocbor_encoder_t *enc, const cbor_envelope_t *env);

/**
 * @brief Matches a device filter against a unique ID, sub-field by sub-field (hardware type, major, minor, serial);
 *        an all-ones sub-field matches any value.
 * @param filter  MSG_DST_DEV_FILTER value
 * @param devId   a device's unique ID
 * @return true if the device matches
 */
bool cbor_dev_filter_match(uint64_t filter, uint64_t devId);

/**
 * @param env    the message's envelope
 * @param devId  the device's unique ID
 * @return true if the message is for the device: it has no filter (the link peer), or a matching one
 */
bool cbor_msg_addressed_to(const cbor_envelope_t *env, uint64_t devId);

#ifdef __cplusplus
}
#endif

#endif /* CBOR_ENVELOPE_H */
