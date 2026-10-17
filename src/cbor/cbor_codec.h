/**
 * @file cbor_codec.h
 * @brief Encoding and decoding of CBOR message bodies against field schemas generated from the CBOR key registry.
 *
 * A schema describes one map: each field's key, value type, element size, count and offset into a C struct, and for
 * nested maps a sub-schema. The codec provides:
 *  - value encode/decode with the protocol's decoding rules (any exact encoding accepted, anything that changes the
 *    meaning rejected; arrays apply the elements that overlap);
 *  - key-path selections (MSG_FIELDS / MSG_ERR_KEYS entries: a key, or an array of keys into nested maps);
 *  - map encoding into a byte budget, marking each field that does not fit with {MSG_STATUS: RESP_TOO_LARGE} and
 *    never splitting a value;
 *  - two-pass map decoding: the whole map is validated (collecting the key paths of invalid values) before anything is
 *    written, so a map is applied entirely or not at all.
 *
 * Design: is-common/docs_internal/cbor/CBOR_PROTOCOL_DESIGN.md (sections 5-7).
 *
 * @copyright Copyright (c) 2026 Inertial Sense, Inc.
 */

#ifndef CBOR_CODEC_H
#define CBOR_CODEC_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "nanocbor/nanocbor.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Value type of a schema field */
typedef enum
{
    CBOR_T_UINT  = 1,   //!< Unsigned integer; size 1, 2, 4 or 8 bytes
    CBOR_T_INT   = 2,   //!< Signed integer; size 1, 2, 4 or 8 bytes
    CBOR_T_FLOAT = 3,   //!< Floating point; size 4 (float) or 8 (double)
    CBOR_T_BOOL  = 4,   //!< Boolean stored in one byte (0 or 1)
    CBOR_T_TEXT  = 5,   //!< Text in a fixed char[count]; at most count-1 bytes, NUL-terminated in memory
    CBOR_T_BYTES = 6,   //!< Opaque bytes in a fixed uint8_t[count]; exactly count bytes on the wire
    CBOR_T_MAP   = 7,   //!< Nested map described by sub; offset addresses the nested struct
} eCborFieldType;

typedef struct cbor_schema_s cbor_schema_t;

/** One field of a schema */
typedef struct
{
    uint16_t                key;        //!< Map key (scoped to the enclosing map)
    uint8_t                 type;       //!< eCborFieldType
    uint8_t                 size;       //!< Bytes per element (numbers); 1 for TEXT/BYTES; 0 for MAP
    uint16_t                count;      //!< Array elements (0 = scalar); buffer size for TEXT/BYTES
    uint16_t                offset;     //!< offsetof() into the struct the schema describes
    const cbor_schema_t    *sub;        //!< CBOR_T_MAP: schema of the nested map; otherwise NULL
} cbor_field_t;

/** A map: its fields in key order */
struct cbor_schema_s
{
    const cbor_field_t     *fields;
    uint16_t                count;
};

/** Codec results. Values >= 0 are byte counts or item counts. */
typedef enum
{
    CBOR_CODEC_OK            =  0,
    CBOR_CODEC_ERR_MALFORMED = -1,  //!< Input is not well-formed CBOR, or not the expected container
    CBOR_CODEC_ERR_VALUE     = -2,  //!< One or more values are invalid (see cbor_decode_result_t)
    CBOR_CODEC_ERR_NO_SPACE  = -3,  //!< Not even the map header (or a too-large marker) fits the budget
    CBOR_CODEC_ERR_TOO_MANY  = -4,  //!< More entries than the caller's array holds
    CBOR_CODEC_ERR_ARG       = -5,  //!< Invalid argument
} eCborCodecResult;

/** Maximum depth of a key path */
#define CBOR_PATH_MAX_DEPTH     4

/** A key path: one key for a top-level field, or a key per level for a nested field */
typedef struct
{
    uint8_t                 depth;
    uint16_t                key[CBOR_PATH_MAX_DEPTH];
} cbor_path_t;

/** Maximum invalid-value paths a decode reports */
#define CBOR_DECODE_ERR_MAX     8

/** Outcome of cbor_codec_decode_map() */
typedef struct
{
    uint8_t                 errCount;                       //!< Invalid values found (may exceed CBOR_DECODE_ERR_MAX)
    cbor_path_t             err[CBOR_DECODE_ERR_MAX];       //!< Paths of the first invalid values
    uint16_t                appliedCount;                   //!< Known fields present in the map (applied when valid)
} cbor_decode_result_t;

/*---------------------------------------------------------------------------- values */

/**
 * @brief Encodes one field's value from src (the field's location in memory).
 * @return >= 0 on success, NANOCBOR_ERR_END if the encoder is out of space (the caller rolls back)
 */
int cbor_codec_encode_value(nanocbor_encoder_t *enc, const cbor_field_t *f, const void *src);

/**
 * @brief Decodes one field's value, applying the protocol's decoding rules.
 * @param it      decoder positioned at the value; advanced past it unless the result is CBOR_CODEC_ERR_MALFORMED
 * @param f       field describing the value
 * @param dst     the field's location in memory, or NULL to validate only
 * @param res     [out, optional] for CBOR_T_MAP fields, receives the key paths of invalid nested values
 * @param prefix  key path of the map holding f (NULL at the top level); nested error paths are prefix + key
 * @return CBOR_CODEC_OK; 1 if a nested map recorded its own errors in res; CBOR_CODEC_ERR_VALUE if the value is
 *         invalid; CBOR_CODEC_ERR_MALFORMED if it is not well-formed CBOR
 */
int cbor_codec_decode_value(nanocbor_value_t *it, const cbor_field_t *f, void *dst, cbor_decode_result_t *res,
                            const cbor_path_t *prefix);

/*---------------------------------------------------------------------------- key paths */

/**
 * @brief Parses a selection array (MSG_FIELDS): each entry is a key or an array of keys.
 * @param it    decoder positioned at the array
 * @param paths [out] parsed paths
 * @return number of paths (0 = the array was empty, meaning every field), or a negative eCborCodecResult
 */
int cbor_codec_decode_paths(nanocbor_value_t *it, cbor_path_t *paths, size_t pathsMax);

/** @brief Encodes a path as a key (depth 1) or an array of keys. @return >= 0, or NANOCBOR_ERR_END */
int cbor_codec_encode_path(nanocbor_encoder_t *enc, const cbor_path_t *path);

/*---------------------------------------------------------------------------- maps */

/**
 * @brief Encodes a struct as a map into buf, within bufSize bytes.
 *
 * Fields are written in schema order. With a selection, only the selected fields are written (a path into a nested
 * map writes that part of it). A field whose value does not fit the remaining budget is written as
 * {MSG_STATUS: RESP_TOO_LARGE} instead; a nested map that does not fit whole is written with its own fields marked
 * the same way. A value is never split.
 *
 * @param paths      selection (NULL or pathCount 0 = every field)
 * @param tooLarge   [out, optional] number of fields marked too large
 * @return bytes written, or CBOR_CODEC_ERR_NO_SPACE if not even the map header fits
 */
int cbor_codec_encode_map(const cbor_schema_t *schema, const void *data, const cbor_path_t *paths, size_t pathCount,
                          uint8_t *buf, size_t bufSize, uint16_t *tooLarge);

/**
 * @brief Decodes a map onto a struct: validates every known field first and writes only if all are valid.
 *
 * Unknown keys (and keys that are not unsigned integers) are skipped. Fields absent from the map are left
 * unchanged. Arrays apply the elements that overlap the field and leave the rest unchanged.
 *
 * @param it    decoder positioned at the map; advanced past it on success or CBOR_CODEC_ERR_VALUE
 * @param data  struct to apply onto, or NULL to validate only
 * @param res   [out] invalid-value paths and the number of known fields present
 * @return CBOR_CODEC_OK, CBOR_CODEC_ERR_VALUE (nothing written), or CBOR_CODEC_ERR_MALFORMED
 */
int cbor_codec_decode_map(nanocbor_value_t *it, const cbor_schema_t *schema, void *data, cbor_decode_result_t *res);

/** @return the schema field with the given key, or NULL */
const cbor_field_t *cbor_codec_find_field(const cbor_schema_t *schema, uint16_t key);

#ifdef __cplusplus
}
#endif

#endif /* CBOR_CODEC_H */
