/**
 * @file cbor_msg.c
 * @brief CBOR message envelope, device-filter matching and the device-side DATA/GET handler. See cbor_msg.h.
 *
 * @copyright Copyright (c) 2026 Inertial Sense, Inc.
 */

#include <string.h>

#include "cbor_msg.h"
#include "cbor_registry.h"

/** Bytes reserved for a map header whose entry count is not yet known */
#define MAP_HDR_RESERVE     3

/** Largest {MSG_STATUS: x} marker for one DID: DID key (3) + map header (1) + MSG_STATUS key (3) + status (1) */
#define DID_MARKER_MAX      8

/** Bytes of the field mask: bit i selects schema field i */
#define FIELD_MASK_BYTES    32

static const cbor_field_t s_u8   = { 0, CBOR_T_UINT, 1, 0, 0, NULL };
static const cbor_field_t s_u16  = { 0, CBOR_T_UINT, 2, 0, 0, NULL };
static const cbor_field_t s_u64  = { 0, CBOR_T_UINT, 8, 0, 0, NULL };
static const cbor_field_t s_bool = { 0, CBOR_T_BOOL, 1, 0, 0, NULL };

/*============================================================================================== helpers */

static bool is_indefinite(const nanocbor_value_t *it)
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
static int next_key(nanocbor_value_t *map, uint16_t *key)
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

static bool mask_test(const uint8_t *mask, uint16_t i)  { return (mask[i / 8] & (1u << (i % 8))) != 0; }
static void mask_set(uint8_t *mask, uint16_t i)         { mask[i / 8] |= (uint8_t)(1u << (i % 8)); }

/** Writes a map whose entry count is known only at the end: reserve the header, then close the gap */
typedef struct
{
    uint8_t            *start;
    nanocbor_encoder_t  enc;
    size_t              entries;
} map_writer_t;

static bool mw_begin(map_writer_t *w, uint8_t *buf, const uint8_t *end)
{
    if (end - buf < MAP_HDR_RESERVE)
        return false;
    w->start = buf;
    w->entries = 0;
    nanocbor_encoder_init(&w->enc, buf + MAP_HDR_RESERVE, (size_t)(end - buf) - MAP_HDR_RESERVE);
    return true;
}

static size_t mw_end(map_writer_t *w)
{
    size_t body = (size_t)(w->enc.cur - (w->start + MAP_HDR_RESERVE));
    size_t hdr = (w->entries < 24) ? 1 : (w->entries < 256) ? 2 : 3;
    nanocbor_encoder_t h;
    nanocbor_encoder_init(&h, w->start, hdr);
    nanocbor_fmt_map(&h, w->entries);
    memmove(w->start + hdr, w->start + MAP_HDR_RESERVE, body);
    return hdr + body;
}

/** Advances enc past n bytes written directly into its buffer */
static void enc_advance(nanocbor_encoder_t *enc, size_t n)
{
    enc->cur += n;
    enc->len += n;
}

/*============================================================================================== envelope */

bool cbor_dev_filter_match(uint64_t filter, uint64_t devId)
{
    static const struct { uint8_t shift, bits; } subfields[] = { { 58, 6 }, { 54, 4 }, { 48, 6 }, { 0, 48 } };
    for (size_t i = 0; i < sizeof(subfields) / sizeof(subfields[0]); i++)
    {
        uint64_t mask = (1ULL << subfields[i].bits) - 1;
        uint64_t f = (filter >> subfields[i].shift) & mask;
        if (f != mask && f != ((devId >> subfields[i].shift) & mask))
            return false;
    }
    return true;
}

bool cbor_msg_addressed_to(const cbor_envelope_t *env, uint64_t devId)
{
    return !env->hasDst || cbor_dev_filter_match(env->dstFilter, devId);
}

/** Decodes a scalar envelope value; CBOR_CODEC_ERR_VALUE (value skipped) if it is not valid for f */
static int decode_scalar(nanocbor_value_t *map, const cbor_field_t *f, void *dst)
{
    int rc = cbor_codec_decode_value(map, f, dst, NULL, NULL);
    return (rc == CBOR_CODEC_ERR_MALFORMED || rc == CBOR_CODEC_ERR_VALUE) ? rc : CBOR_CODEC_OK;
}

static int decode_route(nanocbor_value_t *map, cbor_envelope_t *env)
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

static int decode_envelope_key(nanocbor_value_t *map, uint16_t key, cbor_envelope_t *env)
{
    int rc;
    switch (key)
    {
    case CBOR_MSG_TYPE:
        rc = decode_scalar(map, &s_u8, &env->type);
        return (rc == CBOR_CODEC_OK && env->type == 0) ? CBOR_CODEC_ERR_VALUE : rc;
    case CBOR_MSG_SRC_DEV_ID:
        rc = decode_scalar(map, &s_u64, &env->srcDevId);
        env->hasSrc = (rc == CBOR_CODEC_OK);
        return rc;
    case CBOR_MSG_DST_DEV_FILTER:
        rc = decode_scalar(map, &s_u64, &env->dstFilter);
        env->hasDst = (rc == CBOR_CODEC_OK);
        return rc;
    case CBOR_MSG_REQ_ID:
        rc = decode_scalar(map, &s_u16, &env->reqId);
        env->hasReqId = (rc == CBOR_CODEC_OK);
        return rc;
    case CBOR_MSG_ACK_TYPE:
    {
        uint8_t v;
        rc = decode_scalar(map, &s_u8, &v);
        if (rc != CBOR_CODEC_OK)
            return rc;
        if (v > CBOR_ACK_TYPE_ACK_DETAILED)
            return CBOR_CODEC_ERR_VALUE;
        env->ackType = v;
        return CBOR_CODEC_OK;
    }
    case CBOR_MSG_ROUTE:
        return decode_route(map, env);
    case CBOR_MSG_TIME_GPS_NS:
        rc = decode_scalar(map, &s_u64, &env->timeGpsNs);
        env->hasTimeGps = (rc == CBOR_CODEC_OK);
        return rc;
    case CBOR_MSG_UPTIME_NS:
        rc = decode_scalar(map, &s_u64, &env->uptimeNs);
        env->hasUptime = (rc == CBOR_CODEC_OK);
        return rc;
    case CBOR_MSG_STATUS:
        return decode_scalar(map, &s_u8, &env->status);
    case CBOR_MSG_OPTION_NO_TIME:
        return decode_scalar(map, &s_bool, &env->noTime);
    default:    // unknown envelope keys are ignored
        return nanocbor_skip(map) < 0 ? CBOR_CODEC_ERR_MALFORMED : CBOR_CODEC_OK;
    }
}

/** True if a DID's map holds at least one field key (a value), as opposed to options only */
static bool map_has_values(const nanocbor_value_t *it)
{
    nanocbor_value_t copy = *it, map;
    if (nanocbor_enter_map(&copy, &map) < 0)
        return false;
    while (!nanocbor_at_end(&map))
    {
        uint16_t key;
        int rc = next_key(&map, &key);
        if (rc < 0)
            return false;
        if (rc > 0)
            continue;
        if (key < 0x8000)
            return true;
        if (nanocbor_skip(&map) < 0)
            return false;
    }
    return false;
}

int cbor_msg_parse_envelope(const uint8_t *msg, size_t len, cbor_envelope_t *env)
{
    memset(env, 0, sizeof(*env));
    env->ackType = CBOR_ACK_TYPE_ACK_MINIMAL;
    if (!msg)
        return CBOR_CODEC_ERR_MALFORMED;

    nanocbor_value_t it, map;
    nanocbor_decoder_init(&it, msg, len);
    if (nanocbor_get_type(&it) != NANOCBOR_TYPE_MAP || is_indefinite(&it) || nanocbor_enter_map(&it, &map) < 0)
        return CBOR_CODEC_ERR_MALFORMED;

    int result = CBOR_CODEC_OK;
    bool values = false;
    while (!nanocbor_at_end(&map))
    {
        uint16_t key;
        int rc = next_key(&map, &key);
        if (rc < 0)
            return CBOR_CODEC_ERR_MALFORMED;
        if (rc > 0)
            continue;
        if (key < 0x8000)
        {
            env->didCount++;
            if (!values && nanocbor_get_type(&map) == NANOCBOR_TYPE_MAP)
                values = map_has_values(&map);
            rc = (nanocbor_skip(&map) < 0) ? CBOR_CODEC_ERR_MALFORMED : CBOR_CODEC_OK;
        }
        else
        {
            rc = decode_envelope_key(&map, key, env);
        }
        if (rc == CBOR_CODEC_ERR_MALFORMED)
            return rc;
        if (rc != CBOR_CODEC_OK)
            result = CBOR_CODEC_ERR_VALUE;
    }
    if (nanocbor_leave_container(&it, &map) < 0 || it.cur != msg + len)
        return CBOR_CODEC_ERR_MALFORMED;    // not exactly one map

    if (env->type == 0)
        env->type = values ? CBOR_MSG_TYPE_DATA : CBOR_MSG_TYPE_GET;
    return result;
}

int cbor_msg_encode_envelope(nanocbor_encoder_t *enc, const cbor_envelope_t *env)
{
    int n = 0;
#define PUT(cond, key, fmt)  do { if (cond) { if (nanocbor_fmt_uint(enc, key) < 0 || (fmt) < 0) return NANOCBOR_ERR_END; n++; } } while (0)
    PUT(env->type != 0,                             CBOR_MSG_TYPE,           nanocbor_fmt_uint(enc, env->type));
    PUT(env->hasSrc,                                CBOR_MSG_SRC_DEV_ID,     nanocbor_fmt_uint(enc, env->srcDevId));
    PUT(env->hasDst,                                CBOR_MSG_DST_DEV_FILTER, nanocbor_fmt_uint(enc, env->dstFilter));
    PUT(env->hasReqId,                              CBOR_MSG_REQ_ID,         nanocbor_fmt_uint(enc, env->reqId));
    PUT(env->ackType != CBOR_ACK_TYPE_ACK_MINIMAL,  CBOR_MSG_ACK_TYPE,       nanocbor_fmt_uint(enc, env->ackType));
    if (env->routeDepth)
    {
        if (nanocbor_fmt_uint(enc, CBOR_MSG_ROUTE) < 0 || nanocbor_fmt_array(enc, env->routeDepth) < 0)
            return NANOCBOR_ERR_END;
        for (uint8_t i = 0; i < env->routeDepth; i++)
            if (nanocbor_fmt_uint(enc, env->route[i]) < 0)
                return NANOCBOR_ERR_END;
        n++;
    }
    PUT(env->hasTimeGps,                            CBOR_MSG_TIME_GPS_NS,    nanocbor_fmt_uint(enc, env->timeGpsNs));
    PUT(env->hasUptime,                             CBOR_MSG_UPTIME_NS,      nanocbor_fmt_uint(enc, env->uptimeNs));
    PUT(env->status != CBOR_MSG_STATUS_OK,          CBOR_MSG_STATUS,         nanocbor_fmt_uint(enc, env->status));
    PUT(env->noTime,                                CBOR_MSG_OPTION_NO_TIME, nanocbor_fmt_bool(enc, true));
#undef PUT
    return n;
}

/*============================================================================================== device: replies */

/** The reply under construction: its root map, and the bytes held back so every remaining DID can still be marked */
typedef struct
{
    map_writer_t        root;
    const uint8_t      *end;            //!< true end of the reply buffer
    size_t              reserve;        //!< bytes held back for the markers of DIDs not yet answered
    bool                content;        //!< a DID entry has been written
} reply_t;

/** Limits the root encoder to what one DID may use: everything except the remaining DIDs' markers */
static void reply_limit(reply_t *r)
{
    size_t room = (size_t)(r->end - r->root.enc.cur);
    r->root.enc.end = r->root.enc.cur + (room > r->reserve ? room - r->reserve : 0);
}

static void reply_unlimit(reply_t *r)
{
    r->root.enc.end = (uint8_t *)r->end;
}

static void reply_did_done(reply_t *r)
{
    if (r->reserve >= DID_MARKER_MAX)
        r->reserve -= DID_MARKER_MAX;
}

/**
 * Writes did: {MSG_STATUS: status [, MSG_ERR_KEYS: errs]}. MSG_ERR_KEYS must fit the DID's budget and is dropped if
 * it does not; the status alone always fits, in the room held back for this DID.
 */
static void reply_did_status(reply_t *r, uint16_t did, uint8_t status, const cbor_decode_result_t *errs)
{
    nanocbor_encoder_t *enc = &r->root.enc;
    reply_unlimit(r);
    nanocbor_encoder_t save = *enc;
    size_t errCount = errs ? (errs->errCount < CBOR_DECODE_ERR_MAX ? errs->errCount : CBOR_DECODE_ERR_MAX) : 0;
    for (int withErrs = (errCount > 0); withErrs >= 0; withErrs--)
    {
        *enc = save;
        if (withErrs)
            reply_limit(r);
        else
            reply_unlimit(r);
        bool ok = nanocbor_fmt_uint(enc, did) >= 0 && nanocbor_fmt_map(enc, withErrs ? 2 : 1) >= 0 &&
                  nanocbor_fmt_uint(enc, CBOR_MSG_STATUS) >= 0 && nanocbor_fmt_uint(enc, status) >= 0;
        if (ok && withErrs)
        {
            ok = nanocbor_fmt_uint(enc, CBOR_MSG_ERR_KEYS) >= 0 && nanocbor_fmt_array(enc, errCount) >= 0;
            for (size_t i = 0; ok && i < errCount; i++)
                ok = cbor_codec_encode_path(enc, &errs->err[i]) >= 0;
        }
        if (ok)
        {
            reply_unlimit(r);
            r->root.entries++;
            r->content = true;
            return;
        }
    }
    *enc = save;    // only when the buffer cannot hold the envelope plus a marker per DID
    reply_unlimit(r);
}

/**
 * Writes did: <map> with encodeFn filling the map within the DID's budget. A DID that does not fit at all is
 * marked RESP_TOO_LARGE.
 */
typedef int (*did_encode_fn)(const void *arg, uint8_t *buf, size_t size);

static void reply_did_map(reply_t *r, uint16_t did, did_encode_fn fn, const void *arg)
{
    nanocbor_encoder_t *enc = &r->root.enc;
    nanocbor_encoder_t save = *enc;
    reply_limit(r);
    if (nanocbor_fmt_uint(enc, did) >= 0)
    {
        int n = fn(arg, enc->cur, (size_t)(enc->end - enc->cur));
        if (n >= 0)
        {
            enc_advance(enc, (size_t)n);
            reply_unlimit(r);
            r->root.entries++;
            r->content = true;
            return;
        }
    }
    *enc = save;
    reply_did_status(r, did, CBOR_MSG_STATUS_RESP_TOO_LARGE, NULL);
}

/*============================================================================================== device: DIDs */

typedef struct
{
    const cbor_schema_t    *schema;
    const void             *data;
    const cbor_path_t      *paths;
    size_t                  pathCount;
    const uint8_t          *mask;
} fields_arg_t;

static int encode_fields_fn(const void *arg, uint8_t *buf, size_t size)
{
    const fields_arg_t *a = (const fields_arg_t *)arg;
    if (a->mask)
        return cbor_codec_encode_fields(a->schema, a->data, a->mask, buf, size, NULL);
    return cbor_codec_encode_map(a->schema, a->data, a->paths, a->pathCount, buf, size, NULL);
}

typedef struct
{
    const cbor_device_t    *dev;
    const cbor_path_t      *paths;
    size_t                  pathCount;
} caps_arg_t;

static bool caps_selected(const caps_arg_t *a, uint16_t did)
{
    if (a->pathCount == 0)
        return true;
    for (size_t i = 0; i < a->pathCount; i++)
        if (a->paths[i].depth >= 1 && a->paths[i].key[0] == did)
            return true;
    return false;
}

/** One CAPABILITIES entry: did: [field keys] (for CAPABILITIES itself, the DID keys) */
static int encode_caps_entry(nanocbor_encoder_t *enc, const cbor_device_t *dev, uint16_t did,
                             const cbor_schema_t *schema)
{
    if (nanocbor_fmt_uint(enc, did) < 0)
        return NANOCBOR_ERR_END;
    if (schema)
    {
        if (nanocbor_fmt_array(enc, schema->count) < 0)
            return NANOCBOR_ERR_END;
        for (uint16_t i = 0; i < schema->count; i++)
            if (nanocbor_fmt_uint(enc, schema->fields[i].key) < 0)
                return NANOCBOR_ERR_END;
        return 0;
    }
    if (nanocbor_fmt_array(enc, (size_t)dev->didCount + 1) < 0)
        return NANOCBOR_ERR_END;
    for (uint16_t i = 0; i < dev->didCount; i++)
        if (nanocbor_fmt_uint(enc, dev->dids[i].did) < 0)
            return NANOCBOR_ERR_END;
    return nanocbor_fmt_uint(enc, CBOR_DID_CAPABILITIES) < 0 ? NANOCBOR_ERR_END : 0;
}

static int encode_caps_fn(const void *arg, uint8_t *buf, size_t size)
{
    const caps_arg_t *a = (const caps_arg_t *)arg;
    map_writer_t w;
    if (!mw_begin(&w, buf, buf + size))
        return CBOR_CODEC_ERR_NO_SPACE;
    for (uint16_t i = 0; i <= a->dev->didCount; i++)
    {
        bool self = (i == a->dev->didCount);
        uint16_t did = self ? CBOR_DID_CAPABILITIES : a->dev->dids[i].did;
        if (!caps_selected(a, did))
            continue;
        nanocbor_encoder_t save = w.enc;
        if (encode_caps_entry(&w.enc, a->dev, did, self ? NULL : a->dev->dids[i].schema) < 0)
        {
            w.enc = save;
            if (nanocbor_fmt_uint(&w.enc, did) < 0 || nanocbor_fmt_map(&w.enc, 1) < 0 ||
                nanocbor_fmt_uint(&w.enc, CBOR_MSG_STATUS) < 0 ||
                nanocbor_fmt_uint(&w.enc, CBOR_MSG_STATUS_RESP_TOO_LARGE) < 0)
                return CBOR_CODEC_ERR_NO_SPACE;
        }
        w.entries++;
    }
    return (int)mw_end(&w);
}

static const cbor_did_t *find_did(const cbor_device_t *dev, uint16_t did)
{
    for (uint16_t i = 0; i < dev->didCount; i++)
        if (dev->dids[i].did == did)
            return &dev->dids[i];
    return NULL;
}

/**
 * Reads a GET's selection for one DID: an array of key paths, or an options map with an optional MSG_FIELDS.
 * @return number of paths (0 = every field), or a negative value for a bad request (the value has been skipped)
 */
static int read_selection(nanocbor_value_t *it, cbor_path_t *paths, size_t pathsMax)
{
    int type = nanocbor_get_type(it);
    if (type == NANOCBOR_TYPE_ARR)
    {
        nanocbor_value_t start = *it;
        int n = cbor_codec_decode_paths(it, paths, pathsMax);
        if (n < 0)
        {
            *it = start;
            nanocbor_skip(it);
        }
        return n;
    }
    if (type != NANOCBOR_TYPE_MAP)
    {
        nanocbor_skip(it);
        return CBOR_CODEC_ERR_VALUE;
    }
    nanocbor_value_t map;
    if (nanocbor_enter_map(it, &map) < 0)
        return CBOR_CODEC_ERR_MALFORMED;
    int n = 0;
    while (!nanocbor_at_end(&map))
    {
        uint16_t key;
        int rc = next_key(&map, &key);
        if (rc < 0)
            return CBOR_CODEC_ERR_MALFORMED;
        if (rc > 0)
            continue;
        if (key == CBOR_MSG_FIELDS && n >= 0)
        {
            nanocbor_value_t start = map;
            n = cbor_codec_decode_paths(&map, paths, pathsMax);
            if (n < 0)
            {
                map = start;
                nanocbor_skip(&map);
            }
            continue;
        }
        if (key < 0x8000)
            n = CBOR_CODEC_ERR_VALUE;   // values in a GET
        if (nanocbor_skip(&map) < 0)
            return CBOR_CODEC_ERR_MALFORMED;
    }
    if (nanocbor_leave_container(it, &map) < 0)
        return CBOR_CODEC_ERR_MALFORMED;
    return n;
}

static void handle_get(const cbor_device_t *dev, reply_t *r, uint16_t didKey, const cbor_did_t *d,
                       nanocbor_value_t *it)
{
    cbor_path_t *paths = (cbor_path_t *)dev->scratch;
    size_t pathsMax = dev->scratch ? dev->scratchSize / sizeof(cbor_path_t) : 0;
    int n = read_selection(it, paths, pathsMax);
    if (n < 0)
    {
        reply_did_status(r, didKey, CBOR_MSG_STATUS_BAD_REQUEST, NULL);
        return;
    }
    if (!d)
    {
        caps_arg_t a = { dev, paths, (size_t)n };
        reply_did_map(r, didKey, encode_caps_fn, &a);
        return;
    }
    fields_arg_t a = { d->schema, d->read(d, dev->ctx), paths, (size_t)n, NULL };
    reply_did_map(r, didKey, encode_fields_fn, &a);
}

/**
 * Scans a DATA map without decoding values: marks the schema fields it sets, and takes a DID-level MSG_ACK_TYPE.
 * @return false if a DID-level option is invalid
 */
static bool scan_data_map(nanocbor_value_t it, const cbor_schema_t *schema, uint8_t *mask, uint8_t *ackType)
{
    nanocbor_value_t map;
    if (nanocbor_enter_map(&it, &map) < 0)
        return false;
    bool ok = true;
    while (!nanocbor_at_end(&map))
    {
        uint16_t key;
        int rc = next_key(&map, &key);
        if (rc < 0)
            return false;
        if (rc > 0)
            continue;
        if (key == CBOR_MSG_ACK_TYPE)
        {
            uint8_t v;
            if (decode_scalar(&map, &s_u8, &v) != CBOR_CODEC_OK || v > CBOR_ACK_TYPE_ACK_DETAILED)
                ok = false;
            else
                *ackType = v;
            continue;
        }
        const cbor_field_t *f = (key < 0x8000) ? cbor_codec_find_field(schema, key) : NULL;
        if (f)
        {
            uint16_t i = (uint16_t)(f - schema->fields);
            if (i < FIELD_MASK_BYTES * 8)
                mask_set(mask, i);
        }
        if (nanocbor_skip(&map) < 0)
            return false;
    }
    return ok;
}

static void handle_data(const cbor_device_t *dev, reply_t *r, uint16_t didKey, const cbor_did_t *d,
                        nanocbor_value_t *it, uint8_t ackType)
{
    if (nanocbor_get_type(it) != NANOCBOR_TYPE_MAP)
    {
        nanocbor_skip(it);
        if (ackType != CBOR_ACK_TYPE_ACK_NONE)
            reply_did_status(r, didKey, CBOR_MSG_STATUS_BAD_REQUEST, NULL);
        return;
    }
    uint8_t mask[FIELD_MASK_BYTES] = { 0 };
    bool optionsOk = scan_data_map(*it, d ? d->schema : NULL, mask, &ackType);

    uint8_t status = CBOR_MSG_STATUS_OK;
    cbor_decode_result_t res;
    res.errCount = 0;
    if (!optionsOk)
        status = CBOR_MSG_STATUS_BAD_REQUEST;
    else if (!d || !d->write)
        status = CBOR_MSG_STATUS_READ_ONLY;
    else if (!dev->scratch || dev->scratchSize < d->size)
        status = CBOR_MSG_STATUS_ERROR;

    if (status != CBOR_MSG_STATUS_OK)
    {
        nanocbor_skip(it);
    }
    else
    {
        memcpy(dev->scratch, d->read(d, dev->ctx), d->size);
        nanocbor_value_t start = *it;
        int rc = cbor_codec_decode_map(it, d->schema, dev->scratch, &res);
        if (rc == CBOR_CODEC_ERR_VALUE)
            status = CBOR_MSG_STATUS_BAD_VALUE;
        else if (rc != CBOR_CODEC_OK)
        {
            status = CBOR_MSG_STATUS_BAD_REQUEST;
            *it = start;
            nanocbor_skip(it);
        }
        else
            status = d->write(d, dev->ctx, dev->scratch);
    }

    if (ackType == CBOR_ACK_TYPE_ACK_NONE)
        return;
    if (status != CBOR_MSG_STATUS_OK)
    {
        reply_did_status(r, didKey, status, status == CBOR_MSG_STATUS_BAD_VALUE ? &res : NULL);
        return;
    }

    // Acknowledge: ACK_DETAILED returns the kept values of every field sent; ACK_MINIMAL only those the device
    // kept differently from what was sent
    const uint8_t *kept = (const uint8_t *)d->read(d, dev->ctx);
    if (ackType == CBOR_ACK_TYPE_ACK_MINIMAL)
    {
        bool any = false;
        for (uint16_t i = 0; i < d->schema->count && i < FIELD_MASK_BYTES * 8; i++)
        {
            if (!mask_test(mask, i))
                continue;
            const cbor_field_t *f = &d->schema->fields[i];
            if (memcmp(kept + f->offset, (const uint8_t *)dev->scratch + f->offset, cbor_codec_field_size(f)) == 0)
                mask[i / 8] &= (uint8_t)~(1u << (i % 8));
            else
                any = true;
        }
        if (!any)
            return;
    }
    fields_arg_t a = { d->schema, kept, NULL, 0, mask };
    reply_did_map(r, didKey, encode_fields_fn, &a);
}

/*============================================================================================== device: entry */

static void reply_envelope(const cbor_device_t *dev, const cbor_envelope_t *req, cbor_envelope_t *rep)
{
    memset(rep, 0, sizeof(*rep));
    rep->type = cbor_msg_reply_type(req->type);
    rep->ackType = CBOR_ACK_TYPE_ACK_MINIMAL;
    rep->hasSrc = true;
    rep->srcDevId = dev->devId;
    if (req->hasSrc)
    {   // a device asked: address it
        rep->hasDst = true;
        rep->dstFilter = req->srcDevId;
    }
    else if (req->routeDepth)
    {   // a host behind a forwarder asked
        rep->hasDst = true;
        rep->dstFilter = 0;
    }
    rep->hasReqId = req->hasReqId;
    rep->reqId = req->reqId;
    rep->routeDepth = req->routeDepth;
    memcpy(rep->route, req->route, sizeof(rep->route));
    if (!req->noTime && dev->time)
    {
        uint64_t gps = 0, up = 0;
        dev->time(dev->ctx, &gps, &up);
        if (gps)
        {
            rep->hasTimeGps = true;
            rep->timeGpsNs = gps;
        }
        else
        {
            rep->hasUptime = true;
            rep->uptimeNs = up;
        }
    }
}

static int envelope_only(const cbor_device_t *dev, const cbor_envelope_t *rep)
{
    map_writer_t w;
    if (!mw_begin(&w, dev->reply, dev->reply + dev->replySize))
        return CBOR_CODEC_ERR_NO_SPACE;
    int n = cbor_msg_encode_envelope(&w.enc, rep);
    if (n < 0)
        return CBOR_CODEC_ERR_NO_SPACE;
    w.entries = (size_t)n;
    return (int)mw_end(&w);
}

static bool owns(const cbor_device_t *dev, uint16_t did)
{
    return did == CBOR_DID_CAPABILITIES || find_did(dev, did) != NULL;
}

int cbor_device_handle(const cbor_device_t *dev, const uint8_t *msg, size_t len)
{
    if (!dev || !dev->reply)
        return CBOR_CODEC_ERR_ARG;

    cbor_envelope_t req, rep;
    int prc = cbor_msg_parse_envelope(msg, len, &req);
    if (prc != CBOR_CODEC_ERR_MALFORMED && !cbor_msg_addressed_to(&req, dev->devId))
        return CBOR_MSG_NOT_ADDRESSED;
    if (prc != CBOR_CODEC_ERR_MALFORMED && cbor_msg_is_reply(req.type))
        return 0;   // a reply is never answered, not even with an error

    reply_envelope(dev, &req, &rep);
    uint8_t op = cbor_msg_op(req.type);
    if (prc != CBOR_CODEC_OK)
        rep.status = CBOR_MSG_STATUS_BAD_REQUEST;
    else if (req.type != op || (op != CBOR_MSG_TYPE_DATA && op != CBOR_MSG_TYPE_GET))
        rep.status = CBOR_MSG_STATUS_UNSUPPORTED;   // unknown flags or an operation this device does not handle
    if (rep.status != CBOR_MSG_STATUS_OK)
        return (prc == CBOR_CODEC_OK && req.ackType == CBOR_ACK_TYPE_ACK_NONE) ? 0 : envelope_only(dev, &rep);

    reply_t r;
    memset(&r, 0, sizeof(r));
    r.end = dev->reply + dev->replySize;
    if (!mw_begin(&r.root, dev->reply, r.end))
        return CBOR_CODEC_ERR_NO_SPACE;
    int n = cbor_msg_encode_envelope(&r.root.enc, &rep);
    if (n < 0)
        return CBOR_CODEC_ERR_NO_SPACE;
    r.root.entries = (size_t)n;

    // Walk the DIDs twice: count the owned ones to hold back room for their markers, then answer each
    nanocbor_value_t it, map;
    for (int pass = 0; pass < 2; pass++)
    {
        nanocbor_decoder_init(&it, msg, len);
        nanocbor_enter_map(&it, &map);
        while (!nanocbor_at_end(&map))
        {
            uint16_t key;
            int rc = next_key(&map, &key);
            if (rc < 0)
                return CBOR_CODEC_ERR_MALFORMED;
            if (rc > 0)
                continue;
            if (key >= 0x8000 || !owns(dev, key))
            {   // envelope keys were parsed already; DIDs the device does not own are left out of the reply
                nanocbor_skip(&map);
                continue;
            }
            if (pass == 0)
            {
                r.reserve += DID_MARKER_MAX;
                nanocbor_skip(&map);
                continue;
            }
            reply_did_done(&r);
            const cbor_did_t *d = find_did(dev, key);
            if (req.type == CBOR_MSG_TYPE_GET)
                handle_get(dev, &r, key, d, &map);
            else
                handle_data(dev, &r, key, d, &map, req.ackType);
        }
    }

    if (req.type == CBOR_MSG_TYPE_DATA && req.ackType == CBOR_ACK_TYPE_ACK_NONE && !r.content)
        return 0;
    reply_unlimit(&r);
    return (int)mw_end(&r.root);
}
