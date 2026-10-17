/**
 * @file cbor_device.c
 * @brief The device-side handler: DATA and GET per DID, acknowledgements and CAPABILITIES. See cbor_device.h.
 *
 * @copyright Copyright (c) 2026 Inertial Sense, Inc.
 */

#include <string.h>

#include "cbor_msg_internal.h"

/*============================================================================================== device: replies */

/** The reply under construction: its root map, and the bytes held back so every remaining DID can still be marked */
typedef struct
{
    cbor_map_writer_t        root;
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
    const cbor_did_t       *d;
    void                   *ctx;
    const cbor_path_t      *paths;
    size_t                  pathCount;
} custom_arg_t;

static int encode_custom_fn(const void *arg, uint8_t *buf, size_t size)
{
    const custom_arg_t *a = (const custom_arg_t *)arg;
    if (!a->d->encode)
        return CBOR_CODEC_ERR_ARG;
    return a->d->encode(a->d, a->ctx, a->paths, a->pathCount, buf, size);
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

/** One CAPABILITIES entry: did: [field keys] (for CAPABILITIES itself, d is NULL and the entry lists the DIDs) */
static int encode_caps_entry(nanocbor_encoder_t *enc, const cbor_device_t *dev, uint16_t did, const cbor_did_t *d)
{
    if (nanocbor_fmt_uint(enc, did) < 0)
        return NANOCBOR_ERR_END;
    if (d && !d->schema)
        return d->keys ? (d->keys(d, dev->ctx, enc) < 0 ? NANOCBOR_ERR_END : 0)
                       : (nanocbor_fmt_array(enc, 0) < 0 ? NANOCBOR_ERR_END : 0);
    const cbor_schema_t *schema = d ? d->schema : NULL;
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
    cbor_map_writer_t w;
    if (!cbor_map_begin(&w, buf, buf + size))
        return CBOR_CODEC_ERR_NO_SPACE;
    for (uint16_t i = 0; i <= a->dev->didCount; i++)
    {
        bool self = (i == a->dev->didCount);
        uint16_t did = self ? CBOR_DID_CAPABILITIES : a->dev->dids[i].did;
        if (!caps_selected(a, did))
            continue;
        nanocbor_encoder_t save = w.enc;
        if (encode_caps_entry(&w.enc, a->dev, did, self ? NULL : &a->dev->dids[i]) < 0)
        {
            w.enc = save;
            if (nanocbor_fmt_uint(&w.enc, did) < 0 || nanocbor_fmt_map(&w.enc, 1) < 0 ||
                nanocbor_fmt_uint(&w.enc, CBOR_MSG_STATUS) < 0 ||
                nanocbor_fmt_uint(&w.enc, CBOR_MSG_STATUS_RESP_TOO_LARGE) < 0)
                return CBOR_CODEC_ERR_NO_SPACE;
        }
        w.entries++;
    }
    return (int)cbor_map_end(&w);
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
    if (!d->schema)
    {
        custom_arg_t a = { d, dev->ctx, paths, (size_t)n };
        reply_did_map(r, didKey, encode_custom_fn, &a);
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

/** DATA for a DID without a schema: its decode hook applies it; ACK_DETAILED returns the whole DID as kept */
static void handle_data_custom(const cbor_device_t *dev, reply_t *r, uint16_t didKey, const cbor_did_t *d,
                               nanocbor_value_t *it, uint8_t ackType)
{
    cbor_decode_result_t res;
    memset(&res, 0, sizeof(res));
    uint8_t status;
    if (!d->decode)
    {
        nanocbor_skip(it);
        status = CBOR_MSG_STATUS_READ_ONLY;
    }
    else
    {
        nanocbor_value_t start = *it;
        status = d->decode(d, dev->ctx, it, &res);
        if (it->cur == start.cur)
            nanocbor_skip(it);
    }
    if (ackType == CBOR_ACK_TYPE_ACK_NONE)
        return;
    if (status != CBOR_MSG_STATUS_OK)
    {
        reply_did_status(r, didKey, status, res.errCount ? &res : NULL);
        return;
    }
    if (ackType == CBOR_ACK_TYPE_ACK_DETAILED)
    {
        custom_arg_t a = { d, dev->ctx, NULL, 0 };
        reply_did_map(r, didKey, encode_custom_fn, &a);
    }
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
    else if (d && !d->schema)
    {
        handle_data_custom(dev, r, didKey, d, it, ackType);
        return;
    }
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
    cbor_map_writer_t w;
    if (!cbor_map_begin(&w, dev->reply, dev->reply + dev->replySize))
        return CBOR_CODEC_ERR_NO_SPACE;
    int n = cbor_msg_encode_envelope(&w.enc, rep);
    if (n < 0)
        return CBOR_CODEC_ERR_NO_SPACE;
    w.entries = (size_t)n;
    return (int)cbor_map_end(&w);
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
    if (!cbor_map_begin(&r.root, dev->reply, r.end))
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
    return (int)cbor_map_end(&r.root);
}

int cbor_device_reply_status(const cbor_device_t *dev, const cbor_envelope_t *req, const uint16_t *dids,
                             size_t didCount, uint8_t status)
{
    if (req->ackType == CBOR_ACK_TYPE_ACK_NONE)
        return 0;
    cbor_envelope_t rep;
    reply_envelope(dev, req, &rep);
    cbor_map_writer_t w;
    if (!cbor_map_begin(&w, dev->reply, dev->reply + dev->replySize))
        return CBOR_CODEC_ERR_NO_SPACE;
    int n = cbor_msg_encode_envelope(&w.enc, &rep);
    if (n < 0)
        return CBOR_CODEC_ERR_NO_SPACE;
    w.entries = (size_t)n;
    for (size_t i = 0; i < didCount; i++)
    {
        if (nanocbor_fmt_uint(&w.enc, dids[i]) < 0 || nanocbor_fmt_map(&w.enc, 1) < 0 ||
            nanocbor_fmt_uint(&w.enc, CBOR_MSG_STATUS) < 0 || nanocbor_fmt_uint(&w.enc, status) < 0)
            return CBOR_CODEC_ERR_NO_SPACE;
        w.entries++;
    }
    return (int)cbor_map_end(&w);
}
