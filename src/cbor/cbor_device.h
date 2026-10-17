/**
 * @file cbor_device.h
 * @brief The device side of CBOR messaging: a table of the DIDs a device owns, and the handler that answers DATA and
 *        GET for them.
 *
 * The handler applies DATA per DID (all or nothing within a DID), answers GET with the selected fields, builds the
 * reply within a fixed buffer (marking what does not fit with RESP_TOO_LARGE), acknowledges per MSG_ACK_TYPE and
 * answers CAPABILITIES from the DID table. It allocates nothing: the reply buffer and work area are the device's.
 *
 * Design: is-common/docs_internal/cbor/CBOR_PROTOCOL_DESIGN.md
 *
 * @copyright Copyright (c) 2026 Inertial Sense, Inc.
 */

#ifndef CBOR_DEVICE_H
#define CBOR_DEVICE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "cbor_envelope.h"

#ifdef __cplusplus
extern "C" {
#endif

/** cbor_device_handle(): the message is not addressed to this device */
#define CBOR_MSG_NOT_ADDRESSED      (-16)

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

#ifdef __cplusplus
}
#endif

#endif /* CBOR_DEVICE_H */
