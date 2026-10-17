/**
 * @file cbor_store.c
 * @brief Stored configuration documents. See cbor_store.h.
 *
 * @copyright Copyright (c) 2026 Inertial Sense, Inc.
 */

#include <string.h>

#include "cbor_store.h"

/** Bytes of the field mask: bit i selects schema field i */
#define FIELD_MASK_BYTES    32

bool cbor_store_begin(cbor_map_writer_t *w, uint8_t *buf, size_t size)
{
    return cbor_map_begin(w, buf, buf + size);
}

size_t cbor_store_end(cbor_map_writer_t *w)
{
    return cbor_map_end(w);
}

int cbor_store_add_delta(cbor_map_writer_t *w, uint16_t did, const cbor_schema_t *schema, const void *data,
                         const void *defaults)
{
    uint8_t mask[FIELD_MASK_BYTES] = { 0 };
    bool any = false;
    for (uint16_t i = 0; i < schema->count; i++)
    {
        const cbor_field_t *f = &schema->fields[i];
        if (i >= FIELD_MASK_BYTES * 8)
            return CBOR_CODEC_ERR_ARG;
        if (memcmp((const uint8_t *)data + f->offset, (const uint8_t *)defaults + f->offset, cbor_codec_field_size(f)))
        {
            mask[i / 8] |= (uint8_t)(1u << (i % 8));
            any = true;
        }
    }
    if (!any)
        return 0;

    nanocbor_encoder_t save = w->enc;
    uint16_t tooLarge = 0;
    if (nanocbor_fmt_uint(&w->enc, did) >= 0)
    {
        int n = cbor_codec_encode_fields(schema, data, mask, w->enc.cur, (size_t)(w->enc.end - w->enc.cur), &tooLarge);
        if (n >= 0 && tooLarge == 0)
        {
            w->enc.cur += n;
            w->enc.len += (size_t)n;
            w->entries++;
            return 0;
        }
    }
    w->enc = save;
    return CBOR_CODEC_ERR_NO_SPACE;
}

int cbor_store_add(cbor_map_writer_t *w, uint16_t did, int (*encode)(void *ctx, uint8_t *buf, size_t size), void *ctx)
{
    nanocbor_encoder_t save = w->enc;
    if (nanocbor_fmt_uint(&w->enc, did) >= 0)
    {
        int n = encode(ctx, w->enc.cur, (size_t)(w->enc.end - w->enc.cur));
        if (n >= 0)
        {
            w->enc.cur += n;
            w->enc.len += (size_t)n;
            w->entries++;
            return 0;
        }
    }
    w->enc = save;
    return CBOR_CODEC_ERR_NO_SPACE;
}

bool cbor_store_find(const uint8_t *doc, size_t len, uint16_t did, nanocbor_value_t *it)
{
    nanocbor_value_t root, map;
    nanocbor_decoder_init(&root, doc, len);
    if (nanocbor_get_type(&root) != NANOCBOR_TYPE_MAP || nanocbor_enter_map(&root, &map) < 0)
        return false;
    while (!nanocbor_at_end(&map))
    {
        uint32_t key;
        bool isKey = nanocbor_get_type(&map) == NANOCBOR_TYPE_UINT && nanocbor_get_uint32(&map, &key) >= 0;
        if (!isKey && nanocbor_skip(&map) < 0)
            return false;
        if (isKey && key == did)
        {
            *it = map;
            return true;
        }
        if (nanocbor_skip(&map) < 0)
            return false;
    }
    return false;
}

int cbor_store_load_delta(const uint8_t *doc, size_t len, uint16_t did, const cbor_schema_t *schema, void *data,
                          const void *defaults, size_t size)
{
    memcpy(data, defaults, size);
    nanocbor_value_t it;
    if (!doc || !cbor_store_find(doc, len, did, &it))
        return CBOR_CODEC_OK;
    int rc = cbor_codec_decode_map(&it, schema, data, NULL);
    if (rc != CBOR_CODEC_OK)
        memcpy(data, defaults, size);
    return rc;
}
