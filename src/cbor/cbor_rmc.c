/**
 * @file cbor_rmc.c
 * @brief RMC_STORED encoding and decoding over per-port rmci_t. See cbor_rmc.h.
 *
 * @copyright Copyright (c) 2026 Inertial Sense, Inc.
 */

#include <string.h>

#include "cbor_rmc.h"
#include "cbor_registry.h"

static const cbor_field_t s_u8   = { 0, CBOR_T_UINT, 1, 0, 0, NULL };

uint64_t cbor_rmc_preset_bits(uint8_t preset)
{
    switch (preset)
    {
    case CBOR_RMC_PRESET_IMX_PPD_NO_IMU:          return RMC_PRESET_IMX_PPD_NO_IMU;
    case CBOR_RMC_PRESET_IMX_PPD:                 return RMC_PRESET_IMX_PPD;
    case CBOR_RMC_PRESET_IMX_PPD_IMUS_RAW:        return RMC_PRESET_IMX_PPD_IMUS_RAW;
    case CBOR_RMC_PRESET_IMX_PPD_IMUS_UNCAL:      return RMC_PRESET_IMX_PPD_IMUS_UNCAL;
    case CBOR_RMC_PRESET_INS:                     return RMC_PRESET_INS;
    case CBOR_RMC_PRESET_IMX_PPD_RTK_DBG:         return RMC_PRESET_IMX_PPD_RTK_DBG;
    case CBOR_RMC_PRESET_IMX_PPD_GROUND_VEHICLE:  return RMC_PRESET_IMX_PPD_GROUND_VEHICLE;
    case CBOR_RMC_PRESET_ALLAN_VARIANCE:          return RMC_PRESET_ALLAN_VARIANCE;
    case CBOR_RMC_PRESET_ALLAN_VARIANCE_IMUS:     return RMC_PRESET_ALLAN_VARIANCE_IMUS;
    case CBOR_RMC_PRESET_GNSS:                    return RMC_PRESET_GNSS;
    case CBOR_RMC_PRESET_GPX_PPD:                 return RMC_PRESET_GPX_PPD;
    default:                                      return 0;
    }
}

/** First NMEA sentence pseudo-DID (0x3F00 + NMEA_MSG_ID_*) */
#define NMEA_PSEUDO_DID_BASE    0x3F00

static bool is_nmea_did(uint16_t key)
{
    return key > NMEA_PSEUDO_DID_BASE && key < NMEA_PSEUDO_DID_BASE + NMEA_MSG_ID_COUNT;
}

/** True if key is a DID (with an RMC bit) or an NMEA pseudo-DID that an rmci_t can hold */
static bool storable(uint16_t key)
{
    return (key < DID_COUNT && g_didToRmcBit[key] != 0) || is_nmea_did(key);
}

static bool did_stored(const rmci_t *r, uint16_t key)
{
    if (is_nmea_did(key))
        return (r->rmcNmea.nmeaBits & (1UL << (key - NMEA_PSEUDO_DID_BASE))) != 0;
    if (key >= DID_COUNT)
        return false;
    uint64_t bit = g_didToRmcBit[key];
    return bit && (r->rmc.bits & bit);
}

static uint8_t stored_multiple(const rmci_t *r, uint16_t key)
{
    return is_nmea_did(key) ? r->rmcNmea.nmeaPeriod[key - NMEA_PSEUDO_DID_BASE] : r->periodMultiple[key];
}

/** Adds (mult > 0) or removes (mult 0) a DID or NMEA sentence */
static void set_stored(rmci_t *r, uint16_t key, uint8_t mult)
{
    if (is_nmea_did(key))
    {
        uint32_t bit = 1UL << (key - NMEA_PSEUDO_DID_BASE);
        r->rmcNmea.nmeaBits = mult ? (r->rmcNmea.nmeaBits | bit) : (r->rmcNmea.nmeaBits & ~bit);
        r->rmcNmea.nmeaPeriod[key - NMEA_PSEUDO_DID_BASE] = mult;
        return;
    }
    uint64_t bit = g_didToRmcBit[key];
    r->rmc.bits = mult ? (r->rmc.bits | bit) : (r->rmc.bits & ~bit);
    r->periodMultiple[key] = mult;
}

/** Visits the DIDs, then the NMEA pseudo-DIDs, an rmci_t can hold, in key order */
#define FOR_EACH_STORABLE(key) \
    for (uint16_t key = 0; key < NMEA_PSEUDO_DID_BASE + NMEA_MSG_ID_COUNT; key = (key + 1 == DID_COUNT) ? NMEA_PSEUDO_DID_BASE + 1 : key + 1)

static uint8_t speed_filter(const rmci_t *r)
{
    return (uint8_t)((r->rmc.options & RMC_OPTIONS_NMEA_SPEED_FILTER_BITMASK) >> RMC_OPTIONS_NMEA_SPEED_FILTER_OFFSET);
}

/*============================================================================================== encode */

static bool selected(const cbor_path_t *paths, size_t pathCount, uint16_t port, int32_t key)
{
    if (pathCount == 0)
        return true;
    for (size_t i = 0; i < pathCount; i++)
    {
        if (paths[i].depth == 0 || paths[i].key[0] != port)
            continue;
        if (key < 0 || paths[i].depth == 1 || paths[i].key[1] == (uint16_t)key)
            return true;
    }
    return false;
}

static int encode_port(nanocbor_encoder_t *enc, const rmci_t *r, const cbor_path_t *paths, size_t pathCount,
                       uint16_t port)
{
    size_t n = 0;
    bool filter = speed_filter(r) != CBOR_NMEA_SPEED_FILTER_DEFAULT &&
                  selected(paths, pathCount, port, CBOR_RMC_STORED__NMEA_SPEED_FILTER);
    n += filter;
    FOR_EACH_STORABLE(did)
        n += did_stored(r, did) && selected(paths, pathCount, port, did);

    if (nanocbor_fmt_map(enc, n) < 0)
        return NANOCBOR_ERR_END;
    FOR_EACH_STORABLE(did)
    {
        if (!did_stored(r, did) || !selected(paths, pathCount, port, did))
            continue;
        if (nanocbor_fmt_uint(enc, did) < 0 || nanocbor_fmt_map(enc, 1) < 0 ||
            nanocbor_fmt_uint(enc, CBOR_SUB_RATE_MULTI) < 0 || nanocbor_fmt_uint(enc, stored_multiple(r, did)) < 0)
            return NANOCBOR_ERR_END;
    }
    if (filter && (nanocbor_fmt_uint(enc, CBOR_RMC_STORED__NMEA_SPEED_FILTER) < 0 ||
                   nanocbor_fmt_uint(enc, speed_filter(r)) < 0))
        return NANOCBOR_ERR_END;
    return 0;
}

int cbor_rmc_encode(const cbor_rmc_store_t *s, const cbor_path_t *paths, size_t pathCount, uint8_t *buf, size_t size)
{
    cbor_map_writer_t w;
    if (!cbor_map_begin(&w, buf, buf + size))
        return CBOR_CODEC_ERR_NO_SPACE;
    for (uint8_t port = 0; port < s->portCount; port++)
    {
        if (!selected(paths, pathCount, port, -1))
            continue;
        nanocbor_encoder_t save = w.enc;
        if (nanocbor_fmt_uint(&w.enc, port) < 0 || encode_port(&w.enc, &s->ports[port], paths, pathCount, port) < 0)
        {   // a port is written whole or marked
            w.enc = save;
            if (cbor_codec_encode_too_large(&w.enc, port) < 0)
                return CBOR_CODEC_ERR_NO_SPACE;
        }
        w.entries++;
    }
    return (int)cbor_map_end(&w);
}

int cbor_rmc_encode_keys(const cbor_rmc_store_t *s, nanocbor_encoder_t *enc)
{
    if (nanocbor_fmt_array(enc, s->portCount) < 0)
        return NANOCBOR_ERR_END;
    for (uint8_t port = 0; port < s->portCount; port++)
        if (nanocbor_fmt_uint(enc, port) < 0)
            return NANOCBOR_ERR_END;
    return 0;
}

/*============================================================================================== decode */

typedef struct
{
    const cbor_rmc_store_t *s;
    cbor_decode_result_t   *res;
    bool                    apply;
    bool                    malformed;
} walk_t;

static void bad(walk_t *w, uint8_t depth, uint16_t k0, uint16_t k1, uint16_t k2)
{
    if (!w->res)
        return;
    if (w->res->errCount < CBOR_DECODE_ERR_MAX)
    {
        cbor_path_t *p = &w->res->err[w->res->errCount];
        p->depth = depth;
        p->key[0] = k0;
        p->key[1] = k1;
        p->key[2] = k2;
    }
    if (w->res->errCount < 0xFF)
        w->res->errCount++;
}

static int read_key(walk_t *w, nanocbor_value_t *map, uint16_t *key)
{
    uint32_t k;
    if (nanocbor_get_type(map) == NANOCBOR_TYPE_UINT && nanocbor_get_uint32(map, &k) >= 0 && k <= 0xFFFF)
    {
        *key = (uint16_t)k;
        return 0;
    }
    if (nanocbor_skip(map) < 0 || nanocbor_skip(map) < 0)
        w->malformed = true;
    return 1;   // not a uint16 key: entry skipped
}

static void skip(walk_t *w, nanocbor_value_t *it)
{
    if (nanocbor_skip(it) < 0)
        w->malformed = true;
}

/** One DID entry {SUB_RATE_MULTI: m}: validates, and with w->apply sets or removes the DID */
static void walk_did(walk_t *w, nanocbor_value_t *it, uint8_t port, uint16_t did)
{
    if (nanocbor_get_type(it) != NANOCBOR_TYPE_MAP || !storable(did))
    {
        bad(w, 2, port, did, 0);
        skip(w, it);
        return;
    }
    nanocbor_value_t map;
    if (nanocbor_enter_map(it, &map) < 0)
    {
        w->malformed = true;
        return;
    }
    rmci_t *r = &w->s->ports[port];
    int mult = -1;
    bool invalid = false;
    while (!nanocbor_at_end(&map) && !w->malformed)
    {
        uint16_t key;
        if (read_key(w, &map, &key))
            continue;
        if (key == CBOR_SUB_RATE_MULTI)
        {
            uint8_t v;
            if (cbor_codec_decode_value(&map, &s_u8, &v, NULL, NULL) != CBOR_CODEC_OK)
            {
                bad(w, 3, port, did, key);
                invalid = true;
            }
            else
                mult = v;
        }
        else if (key == CBOR_SUB_RATE_PERIOD || key == CBOR_MSG_FIELDS)
        {   // not representable in rmci_t
            bad(w, 3, port, did, key);
            invalid = true;
            skip(w, &map);
        }
        else
            skip(w, &map);  // unknown keys are ignored
    }
    if (nanocbor_leave_container(it, &map) < 0)
        w->malformed = true;
    if (!w->apply || invalid)
        return;
    if (mult < 0)
        mult = did_stored(r, did) ? stored_multiple(r, did) : 1;   // no rate given: keep it, or the source rate
    set_stored(r, did, (uint8_t)mult);
}

/** One port map. With w->apply: RMC_PRESET first, then the DID entries and NMEA_SPEED_FILTER. */
static void walk_port(walk_t *w, nanocbor_value_t *it, uint16_t portKey)
{
    if (portKey >= w->s->portCount || nanocbor_get_type(it) != NANOCBOR_TYPE_MAP)
    {
        bad(w, 1, portKey, 0, 0);
        skip(w, it);
        return;
    }
    uint8_t port = (uint8_t)portKey;
    rmci_t *r = &w->s->ports[port];

    for (int pass = 0; pass < 2 && !w->malformed; pass++)
    {   // pass 0: options (the preset applies before any entry); pass 1: DID entries
        nanocbor_value_t copy = *it, map;
        if (nanocbor_enter_map(&copy, &map) < 0)
        {
            w->malformed = true;
            return;
        }
        while (!nanocbor_at_end(&map) && !w->malformed)
        {
            uint16_t key;
            if (read_key(w, &map, &key))
                continue;
            if (key < 0x8000)
            {
                if (pass == 1)
                    walk_did(w, &map, port, key);
                else
                    skip(w, &map);
                continue;
            }
            if (pass == 1 || (key != CBOR_RMC_STORED__RMC_PRESET && key != CBOR_RMC_STORED__NMEA_SPEED_FILTER))
            {
                skip(w, &map);
                continue;
            }
            uint8_t v;
            if (cbor_codec_decode_value(&map, &s_u8, &v, NULL, NULL) != CBOR_CODEC_OK ||
                (key == CBOR_RMC_STORED__RMC_PRESET && v != CBOR_RMC_PRESET_NONE && !cbor_rmc_preset_bits(v)) ||
                (key == CBOR_RMC_STORED__NMEA_SPEED_FILTER && v > CBOR_NMEA_SPEED_FILTER_DISABLE))
            {
                bad(w, 2, port, key, 0);
                continue;
            }
            if (!w->apply)
                continue;
            if (key == CBOR_RMC_STORED__RMC_PRESET && v != CBOR_RMC_PRESET_NONE)
            {
                uint64_t bits = cbor_rmc_preset_bits(v);
                r->rmc.bits = bits;
                if (w->s->presetDefaults)
                    w->s->presetDefaults(r, bits, w->s->ctx);
            }
            else if (key == CBOR_RMC_STORED__NMEA_SPEED_FILTER)
            {
                r->rmc.options = (r->rmc.options & ~RMC_OPTIONS_NMEA_SPEED_FILTER_BITMASK) |
                                 ((uint32_t)v << RMC_OPTIONS_NMEA_SPEED_FILTER_OFFSET);
            }
        }
        if (nanocbor_leave_container(&copy, &map) < 0)
            w->malformed = true;
    }
    skip(w, it);
}

/** RMC_SAVE_PORTS: validates the port list, and with w->apply copies each port's runtime state */
static void walk_save_ports(walk_t *w, nanocbor_value_t *it)
{
    nanocbor_value_t arr;
    if (nanocbor_get_type(it) != NANOCBOR_TYPE_ARR || nanocbor_enter_array(it, &arr) < 0)
    {
        bad(w, 1, CBOR_RMC_STORED__RMC_SAVE_PORTS, 0, 0);
        skip(w, it);
        return;
    }
    while (!nanocbor_at_end(&arr) && !w->malformed)
    {
        uint8_t port;
        if (cbor_codec_decode_value(&arr, &s_u8, &port, NULL, NULL) != CBOR_CODEC_OK || port >= w->s->portCount)
        {
            bad(w, 1, CBOR_RMC_STORED__RMC_SAVE_PORTS, 0, 0);
            continue;
        }
        if (w->apply && w->s->savePort)
            w->s->savePort(port, w->s->ctx);
    }
    if (nanocbor_leave_container(it, &arr) < 0)
        w->malformed = true;
}

/** Walks the whole map; RMC_SAVE_PORTS is applied before the port entries so they can adjust what was copied */
static void walk(walk_t *w, nanocbor_value_t it)
{
    for (int pass = 0; pass < 2 && !w->malformed; pass++)
    {
        nanocbor_value_t copy = it, map;
        if (nanocbor_enter_map(&copy, &map) < 0)
        {
            w->malformed = true;
            return;
        }
        while (!nanocbor_at_end(&map) && !w->malformed)
        {
            uint16_t key;
            if (read_key(w, &map, &key))
                continue;
            if (pass == 0 && key == CBOR_RMC_STORED__RMC_SAVE_PORTS)
                walk_save_ports(w, &map);
            else if (pass == 1 && key < 0x8000)
                walk_port(w, &map, key);
            else
                skip(w, &map);  // DID-level options other than RMC_SAVE_PORTS (e.g. MSG_ACK_TYPE) are not ours
        }
        if (nanocbor_leave_container(&copy, &map) < 0)
            w->malformed = true;
    }
}

uint8_t cbor_rmc_decode(const cbor_rmc_store_t *s, nanocbor_value_t *it, cbor_decode_result_t *res)
{
    if (nanocbor_get_type(it) != NANOCBOR_TYPE_MAP)
    {
        nanocbor_skip(it);
        return CBOR_MSG_STATUS_BAD_REQUEST;
    }
    walk_t w = { s, res, false, false };
    walk(&w, *it);
    uint8_t status = w.malformed ? CBOR_MSG_STATUS_BAD_REQUEST
                   : (res && res->errCount) ? CBOR_MSG_STATUS_BAD_VALUE : CBOR_MSG_STATUS_OK;
    if (status == CBOR_MSG_STATUS_OK)
    {
        walk_t a = { s, NULL, true, false };
        walk(&a, *it);
    }
    nanocbor_skip(it);
    return status;
}

void cbor_rmc_load(const cbor_rmc_store_t *s, nanocbor_value_t *it)
{
    if (nanocbor_get_type(it) == NANOCBOR_TYPE_MAP)
    {
        walk_t a = { s, NULL, true, false };
        walk(&a, *it);
    }
    nanocbor_skip(it);
}
