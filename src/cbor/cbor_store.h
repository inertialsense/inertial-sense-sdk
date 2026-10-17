/**
 * @file cbor_store.h
 * @brief Stored configuration in the wire encoding: a document is one CBOR map keyed by DID, each value holding only
 *        what differs from the firmware's current defaults.
 *
 * Loading starts every DID from its defaults and applies the stored delta, so a changed default reaches every field the
 * user never set. A DID whose stored values are invalid for this firmware is loaded as its defaults; unknown DIDs and
 * fields are ignored.
 *
 * Design: is-common/docs_internal/cbor/CBOR_PROTOCOL_DESIGN.md (section 10).
 *
 * @copyright Copyright (c) 2026 Inertial Sense, Inc.
 */

#ifndef CBOR_STORE_H
#define CBOR_STORE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "cbor_codec.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @brief Starts a document in buf. @return false if buf cannot hold even an empty map */
bool cbor_store_begin(cbor_map_writer_t *w, uint8_t *buf, size_t size);

/**
 * @brief Adds did: {fields of data that differ from defaults}. Nothing is added when no field differs.
 * @return 0, or CBOR_CODEC_ERR_NO_SPACE (the document is then incomplete and must not be written)
 */
int cbor_store_add_delta(cbor_map_writer_t *w, uint16_t did, const cbor_schema_t *schema, const void *data,
                         const void *defaults);

/**
 * @brief Adds did: <a map written by encode into [buf, buf + size)> (for DIDs that are not one struct).
 * @return 0, or CBOR_CODEC_ERR_NO_SPACE
 */
int cbor_store_add(cbor_map_writer_t *w, uint16_t did, int (*encode)(void *ctx, uint8_t *buf, size_t size), void *ctx);

/** @brief Closes the document. @return its length */
size_t cbor_store_end(cbor_map_writer_t *w);

/** @brief Finds did in a document. @return true with *it positioned at its value */
bool cbor_store_find(const uint8_t *doc, size_t len, uint16_t did, nanocbor_value_t *it);

/**
 * @brief Loads a struct-backed DID: data = defaults, then the stored delta applied (all or nothing).
 * @return CBOR_CODEC_OK (also when the document does not hold the DID), or a negative eCborCodecResult when the stored
 *         values are invalid (data is then equal to defaults)
 */
int cbor_store_load_delta(const uint8_t *doc, size_t len, uint16_t did, const cbor_schema_t *schema, void *data,
                          const void *defaults, size_t size);

#ifdef __cplusplus
}
#endif

#endif /* CBOR_STORE_H */
