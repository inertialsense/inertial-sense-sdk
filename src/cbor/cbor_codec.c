/**
 * @file cbor_codec.c
 * @brief Schema-driven CBOR value, key-path and map encoding/decoding. See cbor_codec.h.
 *
 * @copyright Copyright (c) 2026 Inertial Sense, Inc.
 */

#include <math.h>
#include <string.h>

#include "cbor_codec.h"
#include "cbor_registry.h"

/** Bytes reserved for a map header whose entry count is not yet known (major type byte + uint16 count) */
#define MAP_HDR_RESERVE     3

/** Sub-selections tracked per nested field; more select the whole field */
#define SUB_SELECT_MAX      8

/*============================================================================================== helpers */

static bool is_array_field(const cbor_field_t *f)
{
    return f->count > 0 && f->type != CBOR_T_TEXT && f->type != CBOR_T_BYTES && f->type != CBOR_T_MAP;
}

/** True when the item at it uses an indefinite length, which the protocol does not allow */
static bool is_indefinite(const nanocbor_value_t *it)
{
    if (it->cur >= it->end)
        return false;
    uint8_t b = *it->cur;
    uint8_t major = b >> NANOCBOR_TYPE_OFFSET;
    return (major >= NANOCBOR_TYPE_BSTR && major <= NANOCBOR_TYPE_MAP) && (b & 0x1F) == 31;
}

static void record_error(cbor_decode_result_t *res, const cbor_path_t *prefix, uint16_t key)
{
    if (!res)
        return;
    if (res->errCount < CBOR_DECODE_ERR_MAX)
    {
        cbor_path_t *p = &res->err[res->errCount];
        *p = prefix ? *prefix : (cbor_path_t){ 0 };
        if (p->depth < CBOR_PATH_MAX_DEPTH)
            p->key[p->depth++] = key;
    }
    if (res->errCount < 0xFF)
        res->errCount++;
}

static uint64_t uint_max(uint8_t size)
{
    return (size >= 8) ? UINT64_MAX : ((1ULL << (size * 8)) - 1);
}

static int64_t int_max(uint8_t size)
{
    return (size >= 8) ? INT64_MAX : (int64_t)((1ULL << (size * 8 - 1)) - 1);
}

static void store_uint(void *dst, uint8_t size, uint64_t v)
{
    switch (size)
    {
    case 1: { uint8_t  x = (uint8_t)v;  memcpy(dst, &x, 1); break; }
    case 2: { uint16_t x = (uint16_t)v; memcpy(dst, &x, 2); break; }
    case 4: { uint32_t x = (uint32_t)v; memcpy(dst, &x, 4); break; }
    default: memcpy(dst, &v, 8); break;
    }
}

static uint64_t load_uint(const void *src, uint8_t size)
{
    switch (size)
    {
    case 1: { uint8_t  x; memcpy(&x, src, 1); return x; }
    case 2: { uint16_t x; memcpy(&x, src, 2); return x; }
    case 4: { uint32_t x; memcpy(&x, src, 4); return x; }
    default: { uint64_t x; memcpy(&x, src, 8); return x; }
    }
}

static int64_t load_int(const void *src, uint8_t size)
{
    switch (size)
    {
    case 1: { int8_t  x; memcpy(&x, src, 1); return x; }
    case 2: { int16_t x; memcpy(&x, src, 2); return x; }
    case 4: { int32_t x; memcpy(&x, src, 4); return x; }
    default: { int64_t x; memcpy(&x, src, 8); return x; }
    }
}

/** Reads any CBOR integer as a sign flag plus magnitude, so uint64 values above INT64_MAX are kept exact */
static int read_integer(nanocbor_value_t *it, bool *negative, uint64_t *magnitude)
{
    int type = nanocbor_get_type(it);
    if (type == NANOCBOR_TYPE_UINT)
    {
        *negative = false;
        return nanocbor_get_uint64(it, magnitude) < 0 ? CBOR_CODEC_ERR_MALFORMED : CBOR_CODEC_OK;
    }
    if (type == NANOCBOR_TYPE_NINT)
    {
        int64_t v;
        if (nanocbor_get_int64(it, &v) < 0)
            return CBOR_CODEC_ERR_VALUE;     // below INT64_MIN: out of range for every field
        *negative = true;
        *magnitude = (uint64_t)(-(v + 1)) + 1;
        return CBOR_CODEC_OK;
    }
    return CBOR_CODEC_ERR_VALUE;
}

/*============================================================================================== value decode */

static int decode_element_at(nanocbor_value_t *it, const cbor_field_t *f, void *dst);

/**
 * Decodes one number or bool element of f into dst (NULL = validate only). On CBOR_CODEC_ERR_VALUE the decoder is
 * left at the start of the element, so the caller skips it.
 */
static int decode_element(nanocbor_value_t *it, const cbor_field_t *f, void *dst)
{
    nanocbor_value_t start = *it;
    int rc = decode_element_at(it, f, dst);
    if (rc == CBOR_CODEC_ERR_VALUE)
        *it = start;
    return rc;
}

static int decode_element_at(nanocbor_value_t *it, const cbor_field_t *f, void *dst)
{
    int type = nanocbor_get_type(it);
    if (type < 0)
        return CBOR_CODEC_ERR_MALFORMED;

    switch (f->type)
    {
    case CBOR_T_UINT:
    {
        bool neg; uint64_t mag;
        int rc = read_integer(it, &neg, &mag);
        if (rc < 0) return rc;
        if (neg || mag > uint_max(f->size)) return CBOR_CODEC_ERR_VALUE;
        if (dst) store_uint(dst, f->size, mag);
        return CBOR_CODEC_OK;
    }
    case CBOR_T_INT:
    {
        bool neg; uint64_t mag;
        int rc = read_integer(it, &neg, &mag);
        if (rc < 0) return rc;
        int64_t v;
        if (neg)
        {
            if (mag > (uint64_t)int_max(f->size) + 1) return CBOR_CODEC_ERR_VALUE;
            v = (mag == (uint64_t)INT64_MAX + 1) ? INT64_MIN : -(int64_t)mag;
        }
        else
        {
            if (mag > (uint64_t)int_max(f->size)) return CBOR_CODEC_ERR_VALUE;
            v = (int64_t)mag;
        }
        if (dst) store_uint(dst, f->size, (uint64_t)v);
        return CBOR_CODEC_OK;
    }
    case CBOR_T_FLOAT:
    {
        double d;
        if (type == NANOCBOR_TYPE_FLOAT)
        {
            if (f->size == 4)
            {   // half and single decode bit-exactly; a double is narrowed if it stays finite
                float v;
                if (nanocbor_get_float(it, &v) >= 0)
                {
                    if (dst) memcpy(dst, &v, sizeof v);
                    return CBOR_CODEC_OK;
                }
            }
            if (nanocbor_get_double(it, &d) < 0)
                return CBOR_CODEC_ERR_VALUE;    // simple values (null, undefined, true/false) are not numbers
        }
        else
        {   // an integer stands for a float only when the float holds it exactly
            bool neg; uint64_t mag;
            int rc = read_integer(it, &neg, &mag);
            if (rc < 0) return rc;
            d = (double)mag;
            if (d >= 18446744073709551616.0 || (uint64_t)d != mag) return CBOR_CODEC_ERR_VALUE;
            if (neg) d = -d;
            if (f->size == 4 && (double)(float)d != d) return CBOR_CODEC_ERR_VALUE;
        }
        if (f->size == 4)
        {
            float v = (float)d;
            if (isinf(v) && !isinf(d)) return CBOR_CODEC_ERR_VALUE;    // finite value outside float range
            if (dst) memcpy(dst, &v, sizeof v);
        }
        else if (dst)
        {
            memcpy(dst, &d, sizeof d);
        }
        return CBOR_CODEC_OK;
    }
    case CBOR_T_BOOL:
    {
        uint8_t v;
        if (type == NANOCBOR_TYPE_FLOAT)
        {
            bool b;
            if (nanocbor_get_bool(it, &b) < 0) return CBOR_CODEC_ERR_VALUE;
            v = b ? 1 : 0;
        }
        else
        {
            bool neg; uint64_t mag;
            int rc = read_integer(it, &neg, &mag);
            if (rc < 0) return rc;
            if (neg || mag > 1) return CBOR_CODEC_ERR_VALUE;
            v = (uint8_t)mag;
        }
        if (dst) *(uint8_t *)dst = v;
        return CBOR_CODEC_OK;
    }
    default:
        return CBOR_CODEC_ERR_VALUE;
    }
}

static int decode_map_fields(nanocbor_value_t *map, const cbor_schema_t *schema, void *data,
                             cbor_decode_result_t *res, const cbor_path_t *prefix);

int cbor_codec_decode_value(nanocbor_value_t *it, const cbor_field_t *f, void *dst, cbor_decode_result_t *res,
                            const cbor_path_t *prefix)
{
    int type = nanocbor_get_type(it);
    if (type < 0)
        return CBOR_CODEC_ERR_MALFORMED;
    if (type == NANOCBOR_TYPE_TAG || is_indefinite(it))
        return nanocbor_skip(it) < 0 ? CBOR_CODEC_ERR_MALFORMED : CBOR_CODEC_ERR_VALUE;

    switch (f->type)
    {
    case CBOR_T_TEXT:
    case CBOR_T_BYTES:
    {
        const uint8_t *s; size_t len;
        int want = (f->type == CBOR_T_TEXT) ? NANOCBOR_TYPE_TSTR : NANOCBOR_TYPE_BSTR;
        if (type != want)
            return nanocbor_skip(it) < 0 ? CBOR_CODEC_ERR_MALFORMED : CBOR_CODEC_ERR_VALUE;
        int rc = (f->type == CBOR_T_TEXT) ? nanocbor_get_tstr(it, &s, &len) : nanocbor_get_bstr(it, &s, &len);
        if (rc < 0)
            return CBOR_CODEC_ERR_MALFORMED;
        size_t cap = (f->type == CBOR_T_TEXT) ? (size_t)f->count - 1 : (size_t)f->count;
        if (len > cap)
            return CBOR_CODEC_ERR_VALUE;
        if (dst)
        {
            memset(dst, 0, f->count);
            memcpy(dst, s, len);
        }
        return CBOR_CODEC_OK;
    }

    case CBOR_T_MAP:
    {
        if (type != NANOCBOR_TYPE_MAP || !f->sub)
            return nanocbor_skip(it) < 0 ? CBOR_CODEC_ERR_MALFORMED : CBOR_CODEC_ERR_VALUE;
        nanocbor_value_t map;
        if (nanocbor_enter_map(it, &map) < 0)
            return CBOR_CODEC_ERR_MALFORMED;
        cbor_path_t path = prefix ? *prefix : (cbor_path_t){ 0 };
        if (path.depth < CBOR_PATH_MAX_DEPTH)
            path.key[path.depth++] = f->key;
        uint8_t before = res ? res->errCount : 0;
        int rc = decode_map_fields(&map, f->sub, dst, res, &path);
        if (rc == CBOR_CODEC_ERR_MALFORMED || nanocbor_leave_container(it, &map) < 0)
            return CBOR_CODEC_ERR_MALFORMED;
        // nested errors were recorded with their full paths; tell the caller not to record this field again
        return (res && res->errCount != before) ? 1 : CBOR_CODEC_OK;
    }

    default:
        break;
    }

    if (!is_array_field(f))
    {
        if (type == NANOCBOR_TYPE_ARR || type == NANOCBOR_TYPE_MAP)
            return nanocbor_skip(it) < 0 ? CBOR_CODEC_ERR_MALFORMED : CBOR_CODEC_ERR_VALUE;
        int rc = decode_element(it, f, dst);
        if (rc == CBOR_CODEC_ERR_VALUE && nanocbor_skip(it) < 0)
            return CBOR_CODEC_ERR_MALFORMED;
        return rc;
    }

    // Array field: apply the elements that overlap; ignore extra elements; leave missing ones unchanged
    if (type != NANOCBOR_TYPE_ARR)
        return nanocbor_skip(it) < 0 ? CBOR_CODEC_ERR_MALFORMED : CBOR_CODEC_ERR_VALUE;
    nanocbor_value_t arr;
    if (nanocbor_enter_array(it, &arr) < 0)
        return CBOR_CODEC_ERR_MALFORMED;
    int result = CBOR_CODEC_OK;
    for (size_t i = 0; !nanocbor_at_end(&arr); i++)
    {
        int rc;
        if (i < f->count && result == CBOR_CODEC_OK)
        {
            rc = decode_element(&arr, f, dst ? (uint8_t *)dst + i * f->size : NULL);
            if (rc == CBOR_CODEC_ERR_VALUE)
            {
                result = CBOR_CODEC_ERR_VALUE;
                rc = nanocbor_skip(&arr);
            }
        }
        else
        {
            rc = nanocbor_skip(&arr);
        }
        if (rc < 0)
            return CBOR_CODEC_ERR_MALFORMED;
    }
    if (nanocbor_leave_container(it, &arr) < 0)
        return CBOR_CODEC_ERR_MALFORMED;
    return result;
}

/*============================================================================================== value encode */

static int encode_map(const cbor_schema_t *schema, const void *data, const cbor_path_t *paths, size_t pathCount,
                      const uint8_t *mask, uint8_t *buf, size_t bufSize, uint16_t *tooLarge, bool strict);

static int encode_element(nanocbor_encoder_t *enc, const cbor_field_t *f, const void *src)
{
    switch (f->type)
    {
    case CBOR_T_UINT:  return nanocbor_fmt_uint(enc, load_uint(src, f->size));
    case CBOR_T_INT:   return nanocbor_fmt_int(enc, load_int(src, f->size));
    case CBOR_T_BOOL:  return nanocbor_fmt_bool(enc, *(const uint8_t *)src != 0);
    case CBOR_T_FLOAT:
        if (f->size == 4) { float v;  memcpy(&v, src, sizeof v); return nanocbor_fmt_float(enc, v); }
        else              { double v; memcpy(&v, src, sizeof v); return nanocbor_fmt_double(enc, v); }
    default:           return CBOR_CODEC_ERR_ARG;
    }
}

int cbor_codec_encode_value(nanocbor_encoder_t *enc, const cbor_field_t *f, const void *src)
{
    switch (f->type)
    {
    case CBOR_T_TEXT:
        return nanocbor_put_tstrn(enc, (const char *)src, strnlen((const char *)src, f->count ? f->count - 1 : 0));
    case CBOR_T_BYTES:
        return nanocbor_put_bstr(enc, (const uint8_t *)src, f->count);
    case CBOR_T_MAP:
    {
        size_t room = (size_t)(enc->end - enc->cur);
        int n = encode_map(f->sub, src, NULL, 0, NULL, enc->cur, room, NULL, true);
        if (n < 0)
            return NANOCBOR_ERR_END;
        enc->cur += n;
        enc->len += (size_t)n;
        return n;
    }
    default:
        break;
    }
    if (!is_array_field(f))
        return encode_element(enc, f, src);
    int rc = nanocbor_fmt_array(enc, f->count);
    for (uint16_t i = 0; rc >= 0 && i < f->count; i++)
        rc = encode_element(enc, f, (const uint8_t *)src + i * f->size);
    return rc;
}

/*============================================================================================== key paths */

int cbor_codec_decode_paths(nanocbor_value_t *it, cbor_path_t *paths, size_t pathsMax)
{
    if (!paths)
        return CBOR_CODEC_ERR_ARG;
    if (nanocbor_get_type(it) != NANOCBOR_TYPE_ARR || is_indefinite(it))
        return CBOR_CODEC_ERR_MALFORMED;
    nanocbor_value_t arr;
    if (nanocbor_enter_array(it, &arr) < 0)
        return CBOR_CODEC_ERR_MALFORMED;
    size_t n = 0;
    while (!nanocbor_at_end(&arr))
    {
        if (n >= pathsMax)
            return CBOR_CODEC_ERR_TOO_MANY;
        cbor_path_t *p = &paths[n];
        p->depth = 0;
        int type = nanocbor_get_type(&arr);
        if (type == NANOCBOR_TYPE_UINT)
        {
            uint32_t k;
            if (nanocbor_get_uint32(&arr, &k) < 0 || k > 0xFFFF)
                return CBOR_CODEC_ERR_VALUE;
            p->key[p->depth++] = (uint16_t)k;
        }
        else if (type == NANOCBOR_TYPE_ARR && !is_indefinite(&arr))
        {
            nanocbor_value_t sub;
            if (nanocbor_enter_array(&arr, &sub) < 0)
                return CBOR_CODEC_ERR_MALFORMED;
            while (!nanocbor_at_end(&sub))
            {
                uint32_t k;
                if (p->depth >= CBOR_PATH_MAX_DEPTH || nanocbor_get_uint32(&sub, &k) < 0 || k > 0xFFFF)
                    return CBOR_CODEC_ERR_VALUE;
                p->key[p->depth++] = (uint16_t)k;
            }
            if (p->depth == 0 || nanocbor_leave_container(&arr, &sub) < 0)
                return CBOR_CODEC_ERR_VALUE;
        }
        else
        {
            return CBOR_CODEC_ERR_VALUE;
        }
        n++;
    }
    if (nanocbor_leave_container(it, &arr) < 0)
        return CBOR_CODEC_ERR_MALFORMED;
    return (int)n;
}

int cbor_codec_encode_path(nanocbor_encoder_t *enc, const cbor_path_t *path)
{
    if (path->depth == 1)
        return nanocbor_fmt_uint(enc, path->key[0]);
    int rc = nanocbor_fmt_array(enc, path->depth);
    for (uint8_t i = 0; rc >= 0 && i < path->depth; i++)
        rc = nanocbor_fmt_uint(enc, path->key[i]);
    return rc;
}

/*============================================================================================== map encode */

const cbor_field_t *cbor_codec_find_field(const cbor_schema_t *schema, uint16_t key)
{
    for (uint16_t i = 0; schema && i < schema->count; i++)
        if (schema->fields[i].key == key)
            return &schema->fields[i];
    return NULL;
}

/** Bytes of a CBOR map header for n entries */
static size_t map_hdr_len(size_t n)
{
    return (n < 24) ? 1 : (n < 256) ? 2 : 3;
}

/**
 * Selection for one field. Returns false if the field is not selected; otherwise sets whole (the full field) or
 * fills sub with the remaining paths into it.
 */
static bool select_field(const cbor_field_t *f, const cbor_path_t *paths, size_t pathCount, bool *whole,
                         cbor_path_t *sub, size_t subMax, size_t *subCount)
{
    *whole = false;
    *subCount = 0;
    if (!paths || pathCount == 0)
    {
        *whole = true;
        return true;
    }
    bool selected = false;
    for (size_t i = 0; i < pathCount; i++)
    {
        if (paths[i].depth == 0 || paths[i].key[0] != f->key)
            continue;
        selected = true;
        if (paths[i].depth == 1 || f->type != CBOR_T_MAP)
        {
            *whole = true;
            continue;
        }
        if (*subCount >= subMax)
        {
            *whole = true;      // more sub-selections than tracked: write the whole field
            continue;
        }
        cbor_path_t *s = &sub[(*subCount)++];
        s->depth = (uint8_t)(paths[i].depth - 1);
        memcpy(s->key, &paths[i].key[1], s->depth * sizeof(uint16_t));
    }
    return selected;
}

static int encode_too_large_marker(nanocbor_encoder_t *enc, uint16_t key)
{
    if (nanocbor_fmt_uint(enc, key) < 0 || nanocbor_fmt_map(enc, 1) < 0 ||
        nanocbor_fmt_uint(enc, CBOR_MSG_STATUS) < 0 || nanocbor_fmt_uint(enc, CBOR_MSG_STATUS_RESP_TOO_LARGE) < 0)
        return NANOCBOR_ERR_END;
    return 0;
}

/**
 * mask, when given, selects fields by schema index instead of paths. Strict: fail with CBOR_CODEC_ERR_NO_SPACE if any
 * selected field does not fit, instead of marking it.
 */
static int encode_map(const cbor_schema_t *schema, const void *data, const cbor_path_t *paths, size_t pathCount,
                      const uint8_t *mask, uint8_t *buf, size_t bufSize, uint16_t *tooLarge, bool strict)
{
    if (!schema || !data || !buf)
        return CBOR_CODEC_ERR_ARG;
    if (bufSize < MAP_HDR_RESERVE)
        return CBOR_CODEC_ERR_NO_SPACE;

    nanocbor_encoder_t enc;
    nanocbor_encoder_init(&enc, buf + MAP_HDR_RESERVE, bufSize - MAP_HDR_RESERVE);
    size_t entries = 0;

    for (uint16_t i = 0; i < schema->count; i++)
    {
        const cbor_field_t *f = &schema->fields[i];
        bool whole;
        cbor_path_t sub[SUB_SELECT_MAX];
        size_t subCount = 0;
        if (mask)
        {
            if (!(mask[i / 8] & (1u << (i % 8))))
                continue;
            whole = true;
        }
        else if (!select_field(f, paths, pathCount, &whole, sub, SUB_SELECT_MAX, &subCount))
            continue;

        const uint8_t *src = (const uint8_t *)data + f->offset;
        nanocbor_encoder_t save = enc;
        if (nanocbor_fmt_uint(&enc, f->key) >= 0)
        {
            int n;
            if (f->type == CBOR_T_MAP)
            {   // a nested map that does not fit whole is written with its own fields marked
                n = encode_map(f->sub, src, whole ? NULL : sub, whole ? 0 : subCount, NULL,
                               enc.cur, (size_t)(enc.end - enc.cur), tooLarge, strict);
                if (n >= 0)
                {
                    enc.cur += n;
                    enc.len += (size_t)n;
                }
            }
            else
            {
                n = cbor_codec_encode_value(&enc, f, src);
            }
            if (n >= 0)
            {
                entries++;
                continue;
            }
        }
        enc = save;
        if (strict || encode_too_large_marker(&enc, f->key) < 0)
        {
            enc = save;
            return CBOR_CODEC_ERR_NO_SPACE;     // not even the marker fits: the caller marks this map instead
        }
        entries++;
        if (tooLarge)
            (*tooLarge)++;
    }

    // Write the exact header and close the reserved gap
    size_t body = (size_t)(enc.cur - (buf + MAP_HDR_RESERVE));
    size_t hdr = map_hdr_len(entries);
    nanocbor_encoder_t h;
    nanocbor_encoder_init(&h, buf, hdr);
    if (nanocbor_fmt_map(&h, entries) < 0)
        return CBOR_CODEC_ERR_NO_SPACE;
    memmove(buf + hdr, buf + MAP_HDR_RESERVE, body);
    return (int)(hdr + body);
}

int cbor_codec_encode_map(const cbor_schema_t *schema, const void *data, const cbor_path_t *paths, size_t pathCount,
                          uint8_t *buf, size_t bufSize, uint16_t *tooLarge)
{
    return encode_map(schema, data, paths, pathCount, NULL, buf, bufSize, tooLarge, false);
}

int cbor_codec_encode_fields(const cbor_schema_t *schema, const void *data, const uint8_t *mask, uint8_t *buf,
                             size_t bufSize, uint16_t *tooLarge)
{
    if (!mask)
        return CBOR_CODEC_ERR_ARG;
    return encode_map(schema, data, NULL, 0, mask, buf, bufSize, tooLarge, false);
}

size_t cbor_codec_field_size(const cbor_field_t *f)
{
    if (f->type == CBOR_T_MAP)
    {   // extent of the nested struct's members (trailing padding excluded)
        size_t end = 0;
        for (uint16_t i = 0; f->sub && i < f->sub->count; i++)
        {
            size_t e = f->sub->fields[i].offset + cbor_codec_field_size(&f->sub->fields[i]);
            if (e > end)
                end = e;
        }
        return end;
    }
    if (f->type == CBOR_T_TEXT || f->type == CBOR_T_BYTES)
        return f->count;
    return (size_t)f->size * (f->count ? f->count : 1);
}

/*============================================================================================== map decode */

static int decode_map_fields(nanocbor_value_t *map, const cbor_schema_t *schema, void *data,
                             cbor_decode_result_t *res, const cbor_path_t *prefix)
{
    int result = CBOR_CODEC_OK;
    while (!nanocbor_at_end(map))
    {
        uint32_t key;
        if (nanocbor_get_type(map) != NANOCBOR_TYPE_UINT || nanocbor_get_uint32(map, &key) < 0 || key > 0xFFFF)
        {   // keys that are not uint16: skip the entry
            if (nanocbor_skip(map) < 0 || nanocbor_skip(map) < 0)
                return CBOR_CODEC_ERR_MALFORMED;
            continue;
        }
        const cbor_field_t *f = cbor_codec_find_field(schema, (uint16_t)key);
        if (!f)
        {   // unknown key: ignored
            if (nanocbor_skip(map) < 0)
                return CBOR_CODEC_ERR_MALFORMED;
            continue;
        }
        if (res && !prefix)
            res->appliedCount++;
        int rc = cbor_codec_decode_value(map, f, data ? (uint8_t *)data + f->offset : NULL, res, prefix);
        if (rc == CBOR_CODEC_ERR_MALFORMED)
            return rc;
        if (rc == CBOR_CODEC_ERR_VALUE)
        {
            record_error(res, prefix, f->key);
            result = CBOR_CODEC_ERR_VALUE;
        }
        else if (rc > 0)
        {
            result = CBOR_CODEC_ERR_VALUE;      // a nested map recorded its own errors
        }
    }
    return result;
}

int cbor_codec_decode_map(nanocbor_value_t *it, const cbor_schema_t *schema, void *data, cbor_decode_result_t *res)
{
    if (!it || !schema)
        return CBOR_CODEC_ERR_ARG;
    cbor_decode_result_t local;
    if (!res)
        res = &local;
    memset(res, 0, sizeof(*res));

    if (nanocbor_get_type(it) != NANOCBOR_TYPE_MAP || is_indefinite(it))
        return CBOR_CODEC_ERR_MALFORMED;

    // Pass 1: validate everything without writing
    nanocbor_value_t check = *it, map;
    if (nanocbor_enter_map(&check, &map) < 0)
        return CBOR_CODEC_ERR_MALFORMED;
    int rc = decode_map_fields(&map, schema, NULL, res, NULL);
    if (rc == CBOR_CODEC_ERR_MALFORMED || nanocbor_leave_container(&check, &map) < 0)
        return CBOR_CODEC_ERR_MALFORMED;
    if (rc == CBOR_CODEC_ERR_VALUE || !data)
    {
        *it = check;
        return rc;
    }

    // Pass 2: apply
    uint16_t applied = res->appliedCount;
    if (nanocbor_enter_map(it, &map) < 0)
        return CBOR_CODEC_ERR_MALFORMED;
    rc = decode_map_fields(&map, schema, data, NULL, NULL);
    if (rc != CBOR_CODEC_OK || nanocbor_leave_container(it, &map) < 0)
        return CBOR_CODEC_ERR_MALFORMED;
    res->appliedCount = applied;
    return CBOR_CODEC_OK;
}
