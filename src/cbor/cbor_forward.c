/**
 * @file cbor_forward.c
 * @brief Forwarding: the downstream copy of a message and the route back upstream. See cbor_forward.h.
 *
 * @copyright Copyright (c) 2026 Inertial Sense, Inc.
 */

#include <string.h>

#include "cbor_msg_internal.h"

/*============================================================================================== forwarding */

/** Appends raw CBOR bytes to an encoder */
static bool put_raw(nanocbor_encoder_t *enc, const uint8_t *p, size_t n)
{
    if ((size_t)(enc->end - enc->cur) < n)
        return false;
    memcpy(enc->cur, p, n);
    enc_advance(enc, n);
    return true;
}

int cbor_msg_forward_build(const uint8_t *msg, size_t len, uint8_t arrivalPort, bool (*deny)(void *ctx, uint16_t did),
                           void *ctx, uint8_t *out, size_t outSize, cbor_forward_result_t *res)
{
    cbor_forward_result_t local;
    if (!res)
        res = &local;
    memset(res, 0, sizeof(*res));

    cbor_envelope_t env;
    if (cbor_msg_parse_envelope(msg, len, &env) == CBOR_CODEC_ERR_MALFORMED)
        return CBOR_CODEC_ERR_MALFORMED;
    if (env.routeDepth >= CBOR_ROUTE_MAX)
        return CBOR_MSG_ROUTE_FULL;

    cbor_map_writer_t w;
    if (!cbor_map_begin(&w, out, out + outSize))
        return CBOR_CODEC_ERR_NO_SPACE;

    uint16_t didsSeen = 0;
    for (int pass = 0; pass < 2; pass++)
    {   // pass 0: envelope keys, then the new MSG_ROUTE; pass 1: DIDs
        nanocbor_value_t it, map;
        nanocbor_decoder_init(&it, msg, len);
        nanocbor_enter_map(&it, &map);
        while (!nanocbor_at_end(&map))
        {
            const uint8_t *start = map.cur;
            uint16_t key;
            int rc = next_key(&map, &key);
            if (rc < 0)
                return CBOR_CODEC_ERR_MALFORMED;
            if (rc > 0)
                continue;   // non-uint16 keys are not forwarded
            if (nanocbor_skip(&map) < 0)
                return CBOR_CODEC_ERR_MALFORMED;
            bool isDid = key < 0x8000;
            if ((pass == 0) == isDid || key == CBOR_MSG_ROUTE)
                continue;
            if (isDid)
            {
                didsSeen++;
                if (deny && deny(ctx, key))
                {
                    if (res->deniedCount < CBOR_FORWARD_DENIED_MAX)
                        res->denied[res->deniedCount++] = key;
                    continue;
                }
                res->forwardedDids++;
            }
            if (!put_raw(&w.enc, start, (size_t)(map.cur - start)))
                return CBOR_CODEC_ERR_NO_SPACE;
            w.entries++;
        }
        if (pass == 0)
        {
            if (nanocbor_fmt_uint(&w.enc, CBOR_MSG_ROUTE) < 0 || nanocbor_fmt_array(&w.enc, (size_t)env.routeDepth + 1) < 0)
                return CBOR_CODEC_ERR_NO_SPACE;
            for (uint8_t i = 0; i < env.routeDepth; i++)
                if (nanocbor_fmt_uint(&w.enc, env.route[i]) < 0)
                    return CBOR_CODEC_ERR_NO_SPACE;
            if (nanocbor_fmt_uint(&w.enc, arrivalPort) < 0)
                return CBOR_CODEC_ERR_NO_SPACE;
            w.entries++;
        }
    }
    if (didsSeen && !res->forwardedDids)
        return 0;
    return (int)cbor_map_end(&w);
}

int cbor_msg_route_pop(uint8_t *msg, size_t *len, uint8_t *port)
{
    nanocbor_value_t it, map;
    nanocbor_decoder_init(&it, msg, *len);
    if (nanocbor_get_type(&it) != NANOCBOR_TYPE_MAP || is_indefinite(&it) || nanocbor_enter_map(&it, &map) < 0)
        return CBOR_CODEC_ERR_MALFORMED;
    size_t entries = nanocbor_map_items_remaining(&map);
    while (!nanocbor_at_end(&map))
    {
        uint8_t *keyStart = (uint8_t *)map.cur;
        uint16_t key;
        int rc = next_key(&map, &key);
        if (rc < 0)
            return CBOR_CODEC_ERR_MALFORMED;
        if (rc > 0)
            continue;
        if (key != CBOR_MSG_ROUTE)
        {
            if (nanocbor_skip(&map) < 0)
                return CBOR_CODEC_ERR_MALFORMED;
            continue;
        }
        cbor_envelope_t env;
        memset(&env, 0, sizeof(env));
        uint8_t *arrStart = (uint8_t *)map.cur;
        if (decode_route(&map, &env) != CBOR_CODEC_OK || env.routeDepth == 0)
            return CBOR_CODEC_ERR_VALUE;
        uint8_t *arrEnd = (uint8_t *)map.cur;
        *port = env.route[env.routeDepth - 1];

        // Rewrite the entry: fewer elements (all < 24 entries, so the array header stays one byte), or none at all
        uint8_t repl[1 + CBOR_ROUTE_MAX * 2];
        nanocbor_encoder_t enc;
        nanocbor_encoder_init(&enc, repl, sizeof(repl));
        uint8_t *from = arrStart;
        size_t newRoot = entries;
        if (env.routeDepth > 1)
        {
            nanocbor_fmt_array(&enc, (size_t)env.routeDepth - 1);
            for (uint8_t i = 0; i + 1 < env.routeDepth; i++)
                nanocbor_fmt_uint(&enc, env.route[i]);
        }
        else
        {
            from = keyStart;    // drop the key too
            newRoot = entries - 1;
        }
        size_t replLen = (size_t)(enc.cur - repl);
        size_t tail = (size_t)((msg + *len) - arrEnd);
        memmove(from + replLen, arrEnd, tail);
        memcpy(from, repl, replLen);
        *len = (size_t)(from - msg) + replLen + tail;

        if (newRoot != entries)
        {   // the root map header may shrink by a byte when its count drops below 24 or 256
            size_t oldHdr = (entries < 24) ? 1 : (entries < 256) ? 2 : 3;
            size_t newHdr = (newRoot < 24) ? 1 : (newRoot < 256) ? 2 : 3;
            nanocbor_encoder_t h;
            uint8_t hdr[3];
            nanocbor_encoder_init(&h, hdr, sizeof(hdr));
            nanocbor_fmt_map(&h, newRoot);
            memmove(msg + newHdr, msg + oldHdr, *len - oldHdr);
            memcpy(msg, hdr, newHdr);
            *len -= oldHdr - newHdr;
        }
        return CBOR_CODEC_OK;
    }
    return CBOR_CODEC_ERR_VALUE;
}
