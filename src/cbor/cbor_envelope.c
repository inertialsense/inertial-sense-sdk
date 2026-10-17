/**
 * @file cbor_envelope.c
 * @brief The message envelope and device-filter matching. See cbor_envelope.h.
 *
 * @copyright Copyright (c) 2026 Inertial Sense, Inc.
 */

#include <string.h>

#include "cbor_msg_internal.h"

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
