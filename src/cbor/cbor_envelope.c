/**
 * @file cbor_envelope.c
 * @brief The message envelope and device-filter matching. See cbor_envelope.h.
 *
 * @copyright Copyright (c) 2026 Inertial Sense, Inc.
 */

#include <string.h>

#include "cbor_envelope.h"

static const cbor_field_t s_u8   = { 0, CBOR_T_UINT, 1, 0, 0, NULL };
static const cbor_field_t s_u16  = { 0, CBOR_T_UINT, 2, 0, 0, NULL };
static const cbor_field_t s_u64  = { 0, CBOR_T_UINT, 8, 0, 0, NULL };
static const cbor_field_t s_bool = { 0, CBOR_T_BOOL, 1, 0, 0, NULL };

/** True when the item at it uses an indefinite length, which the protocol does not allow */
static bool is_indefinite(const nanocbor_value_t *it)
{
    if (it->cur >= it->end)
        return false;
    uint8_t b = *it->cur;
    uint8_t major = b >> NANOCBOR_TYPE_OFFSET;
    return (major >= NANOCBOR_TYPE_BSTR && major <= NANOCBOR_TYPE_MAP) && (b & 0x1F) == 31;
}

/** Decodes a scalar envelope value; CBOR_CODEC_ERR_VALUE (value skipped) if it is not valid for f */
static int decode_scalar(nanocbor_value_t *map, const cbor_field_t *f, void *dst)
{
    int rc = cbor_codec_decode_value(map, f, dst, NULL, NULL);
    return (rc == CBOR_CODEC_ERR_MALFORMED || rc == CBOR_CODEC_ERR_VALUE) ? rc : CBOR_CODEC_OK;
}

/*============================================================================================== keys and routes */

int cbor_msg_next_key(nanocbor_value_t *map, uint16_t *key)
{
    uint64_t k;
    if (nanocbor_get_type(map) == NANOCBOR_TYPE_UINT && nanocbor_get_uint64(map, &k) >= 0)
    {   // the key has been read: a value too large for a key leaves only its value to skip
        if (k <= 0xFFFF)
        {
            *key = (uint16_t)k;
            return CBOR_CODEC_OK;
        }
        return nanocbor_skip(map) < 0 ? CBOR_CODEC_ERR_MALFORMED : 1;
    }
    if (nanocbor_skip(map) < 0 || nanocbor_skip(map) < 0)
        return CBOR_CODEC_ERR_MALFORMED;
    return 1;
}

int cbor_msg_decode_route(nanocbor_value_t *it, cbor_envelope_t *env)
{
    nanocbor_value_t arr;
    if (nanocbor_get_type(it) != NANOCBOR_TYPE_ARR || is_indefinite(it) || nanocbor_enter_array(it, &arr) < 0)
        return nanocbor_skip(it) < 0 ? CBOR_CODEC_ERR_MALFORMED : CBOR_CODEC_ERR_VALUE;
    int rc = CBOR_CODEC_OK;
    uint8_t depth = 0;
    while (!nanocbor_at_end(&arr))
    {
        uint8_t port;
        if (depth >= CBOR_ROUTE_MAX)
        {
            rc = CBOR_CODEC_ERR_VALUE;
            if (nanocbor_skip(&arr) < 0)
                return CBOR_CODEC_ERR_MALFORMED;
            continue;
        }
        int r = decode_scalar(&arr, &s_u8, &port);     // advances past the entry, valid or not
        if (r == CBOR_CODEC_ERR_MALFORMED)
            return r;
        if (r != CBOR_CODEC_OK)
        {
            rc = CBOR_CODEC_ERR_VALUE;
            continue;
        }
        env->route[depth++] = port;
    }
    if (nanocbor_leave_container(it, &arr) < 0)
        return CBOR_CODEC_ERR_MALFORMED;
    env->routeDepth = (rc == CBOR_CODEC_OK) ? depth : 0;
    return rc;
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

/**
 * Decodes the value of one envelope key into env; the value of a key the envelope does not know is skipped.
 * @return CBOR_CODEC_OK, CBOR_CODEC_ERR_VALUE (value skipped), or CBOR_CODEC_ERR_MALFORMED
 */
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
        return cbor_msg_decode_route(map, env);
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
        int rc = cbor_msg_next_key(&map, &key);
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
        int rc = cbor_msg_next_key(&map, &key);
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
