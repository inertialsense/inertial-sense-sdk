/**
 * @file cbor_msg_internal.h
 * @brief Helpers shared by the message-layer sources (cbor_envelope.c, cbor_device.c, cbor_forward.c). Not part of
 *        the API.
 *
 * @copyright Copyright (c) 2026 Inertial Sense, Inc.
 */

#ifndef CBOR_MSG_INTERNAL_H
#define CBOR_MSG_INTERNAL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "cbor_msg.h"

/** Largest {MSG_STATUS: x} marker for one DID: DID key (3) + map header (1) + MSG_STATUS key (3) + status (1) */
#define DID_MARKER_MAX      8

/** Bytes of the field mask: bit i selects schema field i */
#define FIELD_MASK_BYTES    32

static const cbor_field_t s_u8   = { 0, CBOR_T_UINT, 1, 0, 0, NULL };
static const cbor_field_t s_u16  = { 0, CBOR_T_UINT, 2, 0, 0, NULL };
static const cbor_field_t s_u64  = { 0, CBOR_T_UINT, 8, 0, 0, NULL };
static const cbor_field_t s_bool = { 0, CBOR_T_BOOL, 1, 0, 0, NULL };

/*============================================================================================== helpers */

/** True when the item at it uses an indefinite length, which the protocol does not allow */
static inline bool is_indefinite(const nanocbor_value_t *it)
{
    if (it->cur >= it->end)
        return false;
    uint8_t b = *it->cur;
    uint8_t major = b >> NANOCBOR_TYPE_OFFSET;
    return (major >= NANOCBOR_TYPE_BSTR && major <= NANOCBOR_TYPE_MAP) && (b & 0x1F) == 31;
}

/**
 * Reads the next map key. Returns CBOR_CODEC_OK with *key set; 1 when the key is not a uint16 (the entry, key and
 * value, has been skipped); CBOR_CODEC_ERR_MALFORMED on bad input.
 */
static inline int next_key(nanocbor_value_t *map, uint16_t *key)
{
    uint32_t k;
    if (nanocbor_get_type(map) == NANOCBOR_TYPE_UINT && nanocbor_get_uint32(map, &k) >= 0 && k <= 0xFFFF)
    {
        *key = (uint16_t)k;
        return CBOR_CODEC_OK;
    }
    if (nanocbor_skip(map) < 0 || nanocbor_skip(map) < 0)
        return CBOR_CODEC_ERR_MALFORMED;
    return 1;
}

/** Bit i of a field mask (schema field i) */
static inline bool mask_test(const uint8_t *mask, uint16_t i)  { return (mask[i / 8] & (1u << (i % 8))) != 0; }
static inline void mask_set(uint8_t *mask, uint16_t i)         { mask[i / 8] |= (uint8_t)(1u << (i % 8)); }

/** Advances enc past n bytes written directly into its buffer */
static inline void enc_advance(nanocbor_encoder_t *enc, size_t n)
{
    enc->cur += n;
    enc->len += n;
}

/** Decodes a scalar envelope value; CBOR_CODEC_ERR_VALUE (value skipped) if it is not valid for f */
static inline int decode_scalar(nanocbor_value_t *map, const cbor_field_t *f, void *dst)
{
    int rc = cbor_codec_decode_value(map, f, dst, NULL, NULL);
    return (rc == CBOR_CODEC_ERR_MALFORMED || rc == CBOR_CODEC_ERR_VALUE) ? rc : CBOR_CODEC_OK;
}

/**
 * Decodes MSG_ROUTE into env. An invalid entry, or more than CBOR_ROUTE_MAX, rejects the whole route (routeDepth 0).
 * @return CBOR_CODEC_OK, CBOR_CODEC_ERR_VALUE, or CBOR_CODEC_ERR_MALFORMED
 */
static inline int decode_route(nanocbor_value_t *map, cbor_envelope_t *env)
{
    nanocbor_value_t arr;
    if (nanocbor_get_type(map) != NANOCBOR_TYPE_ARR || is_indefinite(map) || nanocbor_enter_array(map, &arr) < 0)
        return nanocbor_skip(map) < 0 ? CBOR_CODEC_ERR_MALFORMED : CBOR_CODEC_ERR_VALUE;
    int rc = CBOR_CODEC_OK;
    uint8_t depth = 0;
    while (!nanocbor_at_end(&arr))
    {
        uint8_t port;
        if (depth >= CBOR_ROUTE_MAX || decode_scalar(&arr, &s_u8, &port) != CBOR_CODEC_OK)
        {
            rc = CBOR_CODEC_ERR_VALUE;
            if (nanocbor_skip(&arr) < 0)
                return CBOR_CODEC_ERR_MALFORMED;
            continue;
        }
        env->route[depth++] = port;
    }
    if (nanocbor_leave_container(map, &arr) < 0)
        return CBOR_CODEC_ERR_MALFORMED;
    env->routeDepth = (rc == CBOR_CODEC_OK) ? depth : 0;
    return rc;
}

#endif /* CBOR_MSG_INTERNAL_H */
