/**
 * @file cbor_msg.h
 * @brief CBOR messages carried by DID_CBOR: the envelope, device-filter matching, and the device-side handler that
 *        answers DATA and GET for a table of DIDs.
 *
 * A message is one CBOR map. Keys at 0x8000 and above form the envelope (MSG_*, options); keys below 0x8000 are DIDs.
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
    uint8_t     type;                       //!< eCborMsgType; on parse, inferred when absent (values: DATA, keys only: GET)
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

/** A DID the device owns, backed by a C struct described by schema */
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
 * @brief Handles one message addressed to the device and builds the reply in dev->reply.
 * @return reply length (0 = no reply), or CBOR_MSG_NOT_ADDRESSED if the message is not for this device
 */
int cbor_device_handle(const cbor_device_t *dev, const uint8_t *msg, size_t len);

#ifdef __cplusplus
}
#endif

#endif /* CBOR_MSG_H */
