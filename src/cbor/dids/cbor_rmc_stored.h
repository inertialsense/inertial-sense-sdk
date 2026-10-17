/**
 * @file cbor_rmc_stored.h
 * @brief RMC_STORED on a device that keeps its startup subscriptions as per-port rmci_t (the IMX's startupRmc[]).
 *
 * RMC_STORED is a map of ports (key n = port n). Each port holds RMC_PRESET and NMEA_SPEED_FILTER options and an entry
 * per subscribed DID or NMEA sentence pseudo-DID (0x3F00 + NMEA_MSG_ID_*): {SUB_RATE_MULTI: m}. The DID-level option
 * RMC_SAVE_PORTS copies the listed ports' runtime subscriptions into storage. In phase 1 a DID is stored only if it
 * has an RMC bit, and rates are period multiples: SUB_RATE_PERIOD and field selections (MSG_FIELDS) are rejected.
 *
 * Writes are all or nothing: the whole map is validated before any port changes. Within a port, RMC_PRESET is applied
 * first and the DID entries after it; a multiple of 0 removes the DID.
 *
 * Design: is-common/docs_internal/cbor/CBOR_PROTOCOL_DESIGN.md (section 9).
 *
 * @copyright Copyright (c) 2026 Inertial Sense, Inc.
 */

#ifndef CBOR_RMC_STORED_H
#define CBOR_RMC_STORED_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "cbor/cbor_codec.h"
#include "data_sets.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Number of ports RMC_STORED holds */
#define RMC_CFG_MAX_PORTS   6

/** A device's stored per-port subscriptions */
typedef struct
{
    rmci_t     *ports;          //!< Stored state, one per port (port n = ports[n])
    uint8_t     portCount;      //!< Entries in ports
    /** Sets the period multiples a preset implies, after its bits have been applied to r */
    void      (*presetDefaults)(rmci_t *r, uint64_t presetBits, void *ctx);
    /** Copies port's runtime subscriptions into ports[port] (RMC_SAVE_PORTS) */
    void      (*savePort)(uint8_t port, void *ctx);
    void       *ctx;            //!< Passed to the callbacks
} cbor_rmc_store_t;

/**
 * @param preset  an eCborRmcPreset value
 * @return the RMC_PRESET_* bits it stands for (0 for NONE or an unknown value)
 */
uint64_t cbor_rmc_preset_bits(uint8_t preset);

/**
 * @brief Encodes RMC_STORED within size bytes. A selection picks ports (one key) or entries of a port (two keys); a
 *        port that does not fit is marked RESP_TOO_LARGE.
 * @param s          the stored subscriptions
 * @param paths      selection (NULL or pathCount 0 = every port)
 * @param pathCount  entries in paths
 * @param buf        output buffer
 * @param size       bytes available in buf
 * @return bytes written, or CBOR_CODEC_ERR_NO_SPACE if not even the map header fits
 */
int cbor_rmc_encode(const cbor_rmc_store_t *s, const cbor_path_t *paths, size_t pathCount, uint8_t *buf, size_t size);

/**
 * @brief Validates the RMC_STORED map at it and, if every value is valid, applies it to s->ports. Advances past it.
 * @param s    the stored subscriptions
 * @param it   decoder positioned at the map
 * @param res  [out] key paths of invalid values
 * @return CBOR_MSG_STATUS_OK, CBOR_MSG_STATUS_BAD_VALUE (nothing applied) or CBOR_MSG_STATUS_BAD_REQUEST (malformed)
 */
uint8_t cbor_rmc_decode(const cbor_rmc_store_t *s, nanocbor_value_t *it, cbor_decode_result_t *res);

/**
 * @brief Applies a stored RMC_STORED map (flash storage) on top of s->ports, skipping any entry it cannot apply rather
 *        than rejecting the map, so storage written by other firmware loads as far as it is understood.
 * @param s   the stored subscriptions
 * @param it  decoder positioned at the stored map; advanced past it
 */
void cbor_rmc_load(const cbor_rmc_store_t *s, nanocbor_value_t *it);

/**
 * @brief Writes the RMC_STORED field keys (the port numbers) as an array, for CAPABILITIES.
 * @param s    the stored subscriptions
 * @param enc  encoder to write to
 * @return >= 0, or NANOCBOR_ERR_END
 */
int cbor_rmc_encode_keys(const cbor_rmc_store_t *s, nanocbor_encoder_t *enc);

#ifdef __cplusplus
}
#endif

#endif /* CBOR_RMC_STORED_H */
