/**
 * @file ISCbor.cpp
 * @brief Host-side CBOR messaging helpers. See ISCbor.h.
 *
 * @copyright Copyright (c) 2026 Inertial Sense, Inc. All rights reserved.
 */

#include <cstdlib>
#include <cstring>
#include <cctype>

#include "ISCbor.h"
#include "ISComm.h"
#include "ISConstants.h"
#include "time_conversion.h"

namespace {

/** Encodes into a growing buffer through nanocbor */
class Writer
{
public:
    explicit Writer(size_t size = 256) : buf(size) { nanocbor_encoder_init(&enc, buf.data(), buf.size()); }
    nanocbor_encoder_t enc;
    std::vector<uint8_t> bytes() const { return std::vector<uint8_t>(buf.data(), (const uint8_t*)enc.cur); }
private:
    std::vector<uint8_t> buf;
};

bool isMarker(nanocbor_value_t it, uint8_t* status)
{
    nanocbor_value_t map;
    if (nanocbor_get_type(&it) != NANOCBOR_TYPE_MAP || nanocbor_enter_map(&it, &map) < 0)
        return false;
    while (!nanocbor_at_end(&map))
    {
        uint32_t k;
        if (nanocbor_get_uint32(&map, &k) < 0)
            return false;
        if (k == CBOR_MSG_STATUS)
        {
            uint32_t s;
            if (nanocbor_get_uint32(&map, &s) < 0)
                return false;
            *status = (uint8_t)s;
            return true;
        }
        if (nanocbor_skip(&map) < 0)
            return false;
    }
    return false;
}

}   // namespace

/*============================================================================================== CborRequest */

CborRequest& CborRequest::get(uint16_t did, const std::vector<cbor_path_t>& fields)
{
    Writer w(16 + fields.size() * (2 + CBOR_PATH_MAX_DEPTH * 3));
    nanocbor_fmt_array(&w.enc, fields.size());
    for (const cbor_path_t& p : fields)
        cbor_codec_encode_path(&w.enc, &p);
    entries.emplace_back(did, w.bytes());
    return *this;
}

CborRequest& CborRequest::getKeys(uint16_t did, const std::vector<uint16_t>& keys)
{
    std::vector<cbor_path_t> paths;
    for (uint16_t k : keys)
    {
        cbor_path_t p = {};
        p.depth = 1;
        p.key[0] = k;
        paths.push_back(p);
    }
    return get(did, paths);
}

CborRequest& CborRequest::set(uint16_t did, const cbor_schema_t& schema, const void* values,
                              const std::vector<uint16_t>& keys)
{
    std::vector<uint8_t> mask(32, 0);
    for (uint16_t i = 0; i < schema.count && i < 256; i++)
    {
        bool sel = keys.empty();
        for (uint16_t k : keys)
            sel |= (schema.fields[i].key == k);
        if (sel)
            mask[i / 8] |= (uint8_t)(1u << (i % 8));
    }
    std::vector<uint8_t> buf(ISB_MAX_DATASET_SIZE);
    int n = cbor_codec_encode_fields(&schema, values, mask.data(), buf.data(), buf.size(), nullptr);
    buf.resize(n > 0 ? (size_t)n : 0);
    entries.emplace_back(did, buf);
    return *this;
}

CborRequest& CborRequest::value(uint16_t did, std::vector<uint8_t> cbor)
{
    entries.emplace_back(did, std::move(cbor));
    return *this;
}

std::vector<uint8_t> CborRequest::encode(uint16_t reqId) const
{
    size_t size = 64;
    for (auto& e : entries)
        size += 3 + e.second.size();
    std::vector<uint8_t> buf(size);

    cbor_envelope_t env = {};
    env.type = type;
    env.ackType = ackType;
    env.hasDst = dst.has_value();
    env.dstFilter = dst.value_or(0);
    env.hasReqId = true;
    env.reqId = reqId;
    env.noTime = omitTime;

    cbor_map_writer_t w;
    if (!cbor_map_begin(&w, buf.data(), buf.data() + buf.size()))
        return {};
    int n = cbor_msg_encode_envelope(&w.enc, &env);
    if (n < 0)
        return {};
    w.entries = (size_t)n;
    for (auto& e : entries)
    {
        if (e.second.empty() || nanocbor_fmt_uint(&w.enc, e.first) < 0 ||
            (size_t)(w.enc.end - w.enc.cur) < e.second.size())
            return {};
        memcpy(w.enc.cur, e.second.data(), e.second.size());
        w.enc.cur += e.second.size();
        w.entries++;
    }
    buf.resize(cbor_map_end(&w));
    return buf;
}

/*============================================================================================== CborReply */

bool CborReply::find(uint16_t did, nanocbor_value_t& it) const
{
    nanocbor_value_t root, map;
    nanocbor_decoder_init(&root, msg.data(), msg.size());
    if (nanocbor_enter_map(&root, &map) < 0)
        return false;
    while (!nanocbor_at_end(&map))
    {
        uint32_t k;
        if (nanocbor_get_type(&map) != NANOCBOR_TYPE_UINT || nanocbor_get_uint32(&map, &k) < 0)
        {
            if (nanocbor_skip(&map) < 0 || nanocbor_skip(&map) < 0)
                return false;
            continue;
        }
        if (k == did)
        {
            it = map;
            return true;
        }
        if (nanocbor_skip(&map) < 0)
            return false;
    }
    return false;
}

bool CborReply::has(uint16_t did) const
{
    nanocbor_value_t it;
    return find(did, it);
}

uint8_t CborReply::status(uint16_t did) const
{
    nanocbor_value_t it;
    uint8_t s;
    if (find(did, it) && isMarker(it, &s))
        return s;
    return CBOR_MSG_STATUS_OK;
}

std::vector<cbor_path_t> CborReply::errKeys(uint16_t did) const
{
    std::vector<cbor_path_t> out;
    nanocbor_value_t it, map;
    if (!find(did, it) || nanocbor_enter_map(&it, &map) < 0)
        return out;
    while (!nanocbor_at_end(&map))
    {
        uint32_t k;
        if (nanocbor_get_uint32(&map, &k) < 0)
            return out;
        if (k == CBOR_MSG_ERR_KEYS)
        {
            cbor_path_t paths[CBOR_DECODE_ERR_MAX];
            int n = cbor_codec_decode_paths(&map, paths, CBOR_DECODE_ERR_MAX);
            for (int i = 0; i < n; i++)
                out.push_back(paths[i]);
            return out;
        }
        if (nanocbor_skip(&map) < 0)
            return out;
    }
    return out;
}

int CborReply::decode(uint16_t did, const cbor_schema_t& schema, void* data, std::vector<uint16_t>* tooLarge) const
{
    nanocbor_value_t it, map;
    if (!find(did, it) || nanocbor_get_type(&it) != NANOCBOR_TYPE_MAP || nanocbor_enter_map(&it, &map) < 0)
        return -1;
    int applied = 0;
    while (!nanocbor_at_end(&map))
    {
        uint32_t k;
        if (nanocbor_get_type(&map) != NANOCBOR_TYPE_UINT || nanocbor_get_uint32(&map, &k) < 0 || k > 0xFFFF)
        {
            if (nanocbor_skip(&map) < 0 || nanocbor_skip(&map) < 0)
                return applied;
            continue;
        }
        const cbor_field_t* f = cbor_codec_find_field(&schema, (uint16_t)k);
        uint8_t s;
        if (f && f->type != CBOR_T_MAP && isMarker(map, &s))
        {
            if (tooLarge && s == CBOR_MSG_STATUS_RESP_TOO_LARGE)
                tooLarge->push_back((uint16_t)k);
            f = nullptr;
        }
        if (!f)
        {
            if (nanocbor_skip(&map) < 0)
                return applied;
            continue;
        }
        int rc = cbor_codec_decode_value(&map, f, (uint8_t*)data + f->offset, nullptr, nullptr);
        if (rc == CBOR_CODEC_ERR_MALFORMED)
            return applied;
        if (rc == CBOR_CODEC_OK)
            applied++;
    }
    return applied;
}

/*============================================================================================== notation and time */

namespace {

struct SubField { uint8_t shift, bits; };
const SubField kSubFields[] = { { 58, 6 }, { 54, 4 }, { 48, 6 }, { 0, 48 } };

bool iequals(const std::string& a, const char* b)
{
    size_t n = strlen(b);
    if (a.size() != n)
        return false;
    for (size_t i = 0; i < n; i++)
        if (tolower((unsigned char)a[i]) != tolower((unsigned char)b[i]))
            return false;
    return true;
}

uint64_t subField(uint64_t v, int i)    { return (v >> kSubFields[i].shift) & ((1ULL << kSubFields[i].bits) - 1); }
bool isAll(uint64_t v, int i)           { return subField(v, i) == ((1ULL << kSubFields[i].bits) - 1); }

const char* typeName(uint64_t t)
{
    switch (t)
    {
    case IS_HARDWARE_TYPE_UINS: return "uINS";
    case IS_HARDWARE_TYPE_EVB:  return "EVB";
    case IS_HARDWARE_TYPE_IMX:  return "IMX";
    case IS_HARDWARE_TYPE_GPX:  return "GPX";
    default:                    return nullptr;
    }
}

/** Parses one sub-field: "-1", "*" or empty = all ones; else a number */
bool parsePart(const std::string& s, int i, uint64_t& out)
{
    uint64_t all = (1ULL << kSubFields[i].bits) - 1;
    if (s.empty() || s == "-1" || s == "*")
    {
        out = all;
        return true;
    }
    char* end = nullptr;
    unsigned long long v = strtoull(s.c_str(), &end, 10);
    if (*end || v > all)
        return false;
    out = v;
    return true;
}

}   // namespace

std::string cborDevFilterToString(uint64_t filter)
{
    if (filter == 0)
        return "host";
    std::string s;
    if (isAll(filter, 0))
        s = "-1";
    else
    {
        const char* n = typeName(subField(filter, 0));
        s = n ? n : std::to_string(subField(filter, 0));
    }
    if (!isAll(filter, 0) || !isAll(filter, 1) || !isAll(filter, 2))
    {
        s += "-" + (isAll(filter, 1) ? std::string("-1") : std::to_string(subField(filter, 1)));
        s += "." + (isAll(filter, 2) ? std::string("-1") : std::to_string(subField(filter, 2)));
    }
    s += "::" + (isAll(filter, 3) ? std::string("-1") : std::to_string(subField(filter, 3)));
    return s;
}

bool cborDevFilterIsWildcard(uint64_t filter)
{
    return filter != 0 && (isAll(filter, 0) || isAll(filter, 1) || isAll(filter, 2) || isAll(filter, 3));
}

bool cborDevFilterFromString(const std::string& str, uint64_t& filter)
{
    if (str == "all" || str == "*")
    {
        filter = UINT64_MAX;
        return true;
    }
    if (str == "host" || str == "0")
    {
        filter = 0;
        return true;
    }
    if (str.size() > 2 && str[0] == '0' && (str[1] == 'x' || str[1] == 'X'))
    {
        char* end = nullptr;
        filter = strtoull(str.c_str(), &end, 16);
        return *end == 0;
    }

    size_t sep = str.find("::");
    std::string hw = (sep == std::string::npos) ? str : str.substr(0, sep);
    std::string sn = (sep == std::string::npos) ? "" : str.substr(sep + 2);
    if (sn.size() > 2 && toupper((unsigned char)sn[0]) == 'S' && toupper((unsigned char)sn[1]) == 'N')
        sn = sn.substr(2);

    // hw = TYPE[-MAJOR[.MINOR]]; TYPE may itself be "-1" or "*"
    std::string type = hw, major, minor;
    size_t dash = hw.find('-', (hw.rfind("-1", 0) == 0) ? 2 : 0);
    if (dash != std::string::npos)
    {
        type = hw.substr(0, dash);
        std::string ver = hw.substr(dash + 1);
        size_t dot = ver.find('.');
        major = (dot == std::string::npos) ? ver : ver.substr(0, dot);
        minor = (dot == std::string::npos) ? "" : ver.substr(dot + 1);
    }

    uint64_t parts[4];
    if (type == "-1" || type == "*" || type.empty())
        parts[0] = (1ULL << kSubFields[0].bits) - 1;
    else if (iequals(type, "IMX"))  parts[0] = IS_HARDWARE_TYPE_IMX;
    else if (iequals(type, "GPX"))  parts[0] = IS_HARDWARE_TYPE_GPX;
    else if (iequals(type, "uINS")) parts[0] = IS_HARDWARE_TYPE_UINS;
    else if (iequals(type, "EVB"))  parts[0] = IS_HARDWARE_TYPE_EVB;
    else if (!parsePart(type, 0, parts[0]))
        return false;
    if (!parsePart(major, 1, parts[1]) || !parsePart(minor, 2, parts[2]) || !parsePart(sn, 3, parts[3]))
        return false;
    filter = 0;
    for (int i = 0; i < 4; i++)
        filter |= parts[i] << kSubFields[i].shift;
    return true;
}

void cborGpsNsToWeekTow(uint64_t gpsNs, uint32_t& week, double& tow)
{
    const uint64_t nsPerWeek = (uint64_t)C_SECONDS_PER_WEEK * 1000000000ULL;
    week = (uint32_t)(gpsNs / nsPerWeek);
    tow = (double)(gpsNs % nsPerWeek) * 1e-9;
}

double cborGpsNsToUnixUtc(uint64_t gpsNs, int leapSeconds)
{
    return (double)gpsNs * 1e-9 + C_GPS_TO_UNIX_OFFSET_S - leapSeconds;
}
