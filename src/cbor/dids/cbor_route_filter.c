/**
 * @file cbor_route_filter.c
 * @brief ROUTE_FILTER evaluation and encoding. See cbor_route_filter.h.
 *
 * @copyright Copyright (c) 2026 Inertial Sense, Inc.
 */

#include "cbor/dids/cbor_route_filter.h"
#include "cbor/cbor_msg.h"
#include "cbor/cbor_registry.h"

uint8_t cbor_route_action(const cbor_route_rule_t *rules, size_t ruleCount, uint64_t devId, uint8_t op, uint16_t did)
{
    for (size_t i = 0; rules && i < ruleCount; i++)
    {
        const cbor_route_rule_t *r = &rules[i];
        if (!cbor_dev_filter_match(r->matchDev, devId))
            continue;
        bool msgMatch = (r->msgCount == 0);
        for (uint8_t j = 0; !msgMatch && j < r->msgCount && j < CBOR_ROUTE_RULE_MSGS_MAX; j++)
            msgMatch = (r->msgs[j] == op);
        bool didMatch = (r->didCount == 0);
        for (uint8_t j = 0; !didMatch && j < r->didCount && j < CBOR_ROUTE_RULE_DIDS_MAX; j++)
            didMatch = (r->dids[j] == did);
        if (msgMatch && didMatch)
            return r->action;
    }
    return CBOR_ROUTE_ACTION_ALLOW;
}

/** @return true if the selection includes rule index (field < 0) or that field of it; an empty selection includes all */
static bool selected(const cbor_path_t *paths, size_t pathCount, uint16_t index, int field)
{
    if (pathCount == 0)
        return true;
    for (size_t i = 0; i < pathCount; i++)
    {
        if (paths[i].depth == 0 || paths[i].key[0] != index)
            continue;
        if (field < 0 || paths[i].depth == 1 || paths[i].key[1] == (uint16_t)field)
            return true;
    }
    return false;
}

/** Writes rule index's map, limited to the selection; fields with no entries (match every value) are left out */
static int encode_rule(nanocbor_encoder_t *enc, const cbor_route_rule_t *r, const cbor_path_t *paths, size_t pathCount,
                       uint16_t index)
{
    bool fMsg = r->msgCount && selected(paths, pathCount, index, CBOR_ROUTE_FILTER__MATCH_MSG);
    bool fDid = r->didCount && selected(paths, pathCount, index, CBOR_ROUTE_FILTER__MATCH_DIDS);
    bool fDev = selected(paths, pathCount, index, CBOR_ROUTE_FILTER__MATCH_DEV);
    bool fAct = selected(paths, pathCount, index, CBOR_ROUTE_FILTER__ACTION);
    if (nanocbor_fmt_map(enc, (size_t)fDev + fMsg + fDid + fAct) < 0)
        return NANOCBOR_ERR_END;
    if (fDev && (nanocbor_fmt_uint(enc, CBOR_ROUTE_FILTER__MATCH_DEV) < 0 || nanocbor_fmt_uint(enc, r->matchDev) < 0))
        return NANOCBOR_ERR_END;
    if (fMsg)
    {
        if (nanocbor_fmt_uint(enc, CBOR_ROUTE_FILTER__MATCH_MSG) < 0 || nanocbor_fmt_array(enc, r->msgCount) < 0)
            return NANOCBOR_ERR_END;
        for (uint8_t j = 0; j < r->msgCount; j++)
            if (nanocbor_fmt_uint(enc, r->msgs[j]) < 0)
                return NANOCBOR_ERR_END;
    }
    if (fDid)
    {
        if (nanocbor_fmt_uint(enc, CBOR_ROUTE_FILTER__MATCH_DIDS) < 0 || nanocbor_fmt_array(enc, r->didCount) < 0)
            return NANOCBOR_ERR_END;
        for (uint8_t j = 0; j < r->didCount; j++)
            if (nanocbor_fmt_uint(enc, r->dids[j]) < 0)
                return NANOCBOR_ERR_END;
    }
    if (fAct && (nanocbor_fmt_uint(enc, CBOR_ROUTE_FILTER__ACTION) < 0 || nanocbor_fmt_uint(enc, r->action) < 0))
        return NANOCBOR_ERR_END;
    return 0;
}

int cbor_route_encode(const cbor_route_rule_t *rules, size_t ruleCount, const cbor_path_t *paths, size_t pathCount,
                      uint8_t *buf, size_t size)
{
    cbor_map_writer_t w;
    if (!cbor_map_begin(&w, buf, buf + size))
        return CBOR_CODEC_ERR_NO_SPACE;
    for (size_t i = 0; rules && i < ruleCount && i <= 0x7FFF; i++)
    {
        uint16_t index = (uint16_t)i;
        if (!selected(paths, pathCount, index, -1))
            continue;
        nanocbor_encoder_t save = w.enc;
        if (nanocbor_fmt_uint(&w.enc, index) < 0 || encode_rule(&w.enc, &rules[i], paths, pathCount, index) < 0)
        {   // a rule is written whole or marked
            w.enc = save;
            if (cbor_codec_encode_too_large(&w.enc, index) < 0)
                return CBOR_CODEC_ERR_NO_SPACE;
        }
        w.entries++;
    }
    return (int)cbor_map_end(&w);
}

int cbor_route_encode_keys(size_t ruleCount, nanocbor_encoder_t *enc)
{
    if (nanocbor_fmt_array(enc, ruleCount) < 0)
        return NANOCBOR_ERR_END;
    for (size_t i = 0; i < ruleCount; i++)
        if (nanocbor_fmt_uint(enc, i) < 0)
            return NANOCBOR_ERR_END;
    return 0;
}
