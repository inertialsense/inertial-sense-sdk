/**
 * @file cbor_msg.h
 * @brief CBOR messages carried by DID_CBOR: the envelope, device-filter matching, and the device-side handler that
 *        answers DATA and GET for a table of DIDs.
 *
 * A message is one CBOR map. Keys at 0x8000 and above form the envelope (MSG_*, options); keys below 0x8000 are DIDs.
 * MSG_TYPE is an operation (DATA, GET, SUBSCRIBE) plus flags; a reply carries the operation it answers with the REPLY
 * flag, and is never applied, acknowledged or answered.
 * The device handler applies DATA per DID (all-or-nothing within a DID), answers GET with the selected fields, builds
 * replies within a fixed buffer (marking what does not fit with RESP_TOO_LARGE), acknowledges per MSG_ACK_TYPE and
 * answers CAPABILITIES from its DID table.
 *
 * Design: is-common/docs_internal/cbor/CBOR_PROTOCOL_DESIGN.md (sections 1, 4-6).
 *
 * @copyright Copyright (c) 2026 Inertial Sense, Inc.
 */

#ifndef CBOR_MSG_H
#define CBOR_MSG_H

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

/** cbor_device_handle(): the message is not addressed to this device */
#define CBOR_MSG_NOT_ADDRESSED      (-16)

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
    uint16_t    reqId;
    uint64_t    srcDevId;
    uint64_t    dstFilter;
    uint64_t    timeGpsNs;                  //!< ns since the GPS epoch
    uint64_t    uptimeNs;
    uint16_t    didCount;                   //!< on parse: DID keys in the message
} cbor_envelope_t;

/** @return the operation of a MSG_TYPE value (eCborMsgType) */
static inline uint8_t cbor_msg_op(uint8_t type)         { return (uint8_t)(type & CBOR_MSG_TYPE_OP_MASK); }

/** @return the MSG_TYPE of a reply to a message of the given MSG_TYPE */
static inline uint8_t cbor_msg_reply_type(uint8_t type) { return (uint8_t)((unsigned)CBOR_MSG_TYPE_FLAG_REPLY | type); }

/** @return true if a MSG_TYPE value marks a reply */
static inline bool cbor_msg_is_reply(uint8_t type)      { return (type & CBOR_MSG_TYPE_FLAG_REPLY) != 0; }

/**
 * @brief Parses the envelope of a message.
 * @return CBOR_CODEC_OK; CBOR_CODEC_ERR_VALUE if an envelope key has an invalid value (the rest of env is still
 *         filled); CBOR_CODEC_ERR_MALFORMED if msg is not exactly one well-formed CBOR map
 */
int cbor_msg_parse_envelope(const uint8_t *msg, size_t len, cbor_envelope_t *env);

/**
 * @brief Writes the envelope's present keys as map entries (no map header), in protocol order.
 * @return number of entries written, or a negative value if the encoder ran out of space
 */
int cbor_msg_encode_envelope(nanocbor_encoder_t *enc, const cbor_envelope_t *env);

/**
 * @brief Matches a device filter against a unique ID, sub-field by sub-field (hardware type, major, minor, serial);
 *        an all-ones sub-field matches any value.
 */
bool cbor_dev_filter_match(uint64_t filter, uint64_t devId);

/** @return true if a message with this envelope is for the device: no filter (the link peer), or a matching one */
bool cbor_msg_addressed_to(const cbor_envelope_t *env, uint64_t devId);

/*---------------------------------------------------------------------------- device side */

typedef struct cbor_did_s cbor_did_t;

/**
 * A DID the device owns. Most are backed by a C struct described by schema (read/write). A DID that is not a fixed
 * struct (rule lists, per-port tables) leaves schema NULL and provides encode/decode/keys instead.
 */
struct cbor_did_s
{
    uint16_t                did;        //!< Root key (CBOR_DID_*)
    const cbor_schema_t    *schema;
    uint16_t                size;       //!< sizeof the struct
    /** @return the current values */
    const void *(*read)(const cbor_did_t *d, void *ctx);
    /**
     * Validates, applies and persists new values (a full struct: the current values with the message's changes).
     * The device may adjust values (clamp, round); the reply reports what read() returns afterwards.
     * @return eCborMsgStatus (CBOR_MSG_STATUS_OK on success). NULL = the DID is read-only.
     */
    uint8_t (*write)(const cbor_did_t *d, void *ctx, const void *values);
    /**
     * GET for a DID without a schema: writes the selected part (pathCount 0 = all) as one map into buf, marking what
     * does not fit with RESP_TOO_LARGE. @return bytes written, or a negative value if not even the map header fits
     */
    int (*encode)(const cbor_did_t *d, void *ctx, const cbor_path_t *paths, size_t pathCount, uint8_t *buf,
                  size_t size);
    /**
     * DATA for a DID without a schema: validates and applies the map at it, all or nothing, advancing past it; the
     * key paths of invalid values go in res. @return eCborMsgStatus. NULL (with write NULL) = read-only.
     */
    uint8_t (*decode)(const cbor_did_t *d, void *ctx, nanocbor_value_t *it, cbor_decode_result_t *res);
    /** CAPABILITIES for a DID without a schema: writes the array of field keys served. NULL = an empty array. */
    int (*keys)(const cbor_did_t *d, void *ctx, nanocbor_encoder_t *enc);
};

/** A device answering CBOR messages */
typedef struct
{
    uint64_t            devId;          //!< This device's unique ID (MSG_SRC_DEV_ID)
    const cbor_did_t   *dids;
    uint16_t            didCount;
    uint8_t            *reply;          //!< Reply buffer
    size_t              replySize;
    void               *scratch;        //!< Work area, aligned for any type; at least the largest writable DID's size
    size_t              scratchSize;
    /** Current time: *gpsNs = ns since the GPS epoch, or 0 when GNSS time is not synced; *uptimeNs = system uptime */
    void              (*time)(void *ctx, uint64_t *gpsNs, uint64_t *uptimeNs);
    void               *ctx;            //!< Passed to the callbacks
} cbor_device_t;

/**
 * @brief Handles one message addressed to the device and builds the reply in dev->reply. Replies are not handled
 *        here (they belong to whoever sent the request): a reply returns 0 and changes nothing.
 * @return reply length (0 = no reply), or CBOR_MSG_NOT_ADDRESSED if the message is not for this device
 */
int cbor_device_handle(const cbor_device_t *dev, const uint8_t *msg, size_t len);

/**
 * @brief Writes into dev->reply a reply to req that marks each of dids with status (e.g. FORWARD_DENIED).
 * @return reply length, 0 if req asked for no acknowledgement, or a negative value if it does not fit
 */
int cbor_device_reply_status(const cbor_device_t *dev, const cbor_envelope_t *req, const uint16_t *dids,
                             size_t didCount, uint8_t status);

/*---------------------------------------------------------------------------- forwarding */

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

#endif /* CBOR_MSG_H */
