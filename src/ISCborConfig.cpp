/**
 * @file ISCborConfig.cpp
 * @brief Host-side helpers for DID_CBOR_CONFIG
 *
 * @copyright Copyright (c) 2026 Inertial Sense, Inc.
 */

#include "ISCborConfig.h"

#include <cctype>
#include <cstdlib>
#include <cstring>
#include <map>
#include <sstream>

#include "ISDataMappings.h"
#include "nanocbor/nanocbor.h"

namespace {

bool equalsIgnoreCase(const std::string& a, const std::string& b)
{
    if (a.size() != b.size())
        return false;
    for (size_t i = 0; i < a.size(); i++)
        if (std::tolower((unsigned char)a[i]) != std::tolower((unsigned char)b[i]))
            return false;
    return true;
}

const flash_cfg_field_desc_t* canFields()  { uint16_t n; return flash_cfg_cbor_can_fields(&n); }
const flash_cfg_field_desc_t* rmcFields()  { uint16_t n; return flash_cfg_cbor_rmc_port_fields(&n); }

/** A field group as seen by name lookups: the table, its key base, and how its fields are named */
struct GroupNames
{
    const flash_cfg_field_desc_t* fields;
    uint16_t count;
    uint16_t idBase;
    uint32_t did;               //!< DID whose field mapping supplies names by offset, or DID_NULL
    std::string prefix;         //!< prepended to each name
};

std::vector<GroupNames> groupNames(uint16_t hdwId)
{
    std::vector<GroupNames> out;
    uint16_t count;
    const flash_cfg_field_desc_t* f;
    switch (DECODE_HDW_TYPE(hdwId))
    {
    case IS_HARDWARE_TYPE_IMX:
        f = flash_cfg_cbor_imx_fields(&count);
        out.push_back({ f, count, 0, DID_FLASH_CONFIG, "" });
        f = flash_cfg_cbor_can_fields(&count);
        out.push_back({ f, count, 0, DID_NULL, "" });
        for (unsigned port = 0; port < RMC_CFG_MAX_PORTS; port++)
        {
            f = flash_cfg_cbor_rmc_port_fields(&count);
            out.push_back({ f, count, flash_cfg_cbor_rmc_port_id_base(port), DID_NULL, "rmc[" + std::to_string(port) + "]." });
        }
        break;
    case IS_HARDWARE_TYPE_GPX:
        f = flash_cfg_cbor_gpx_fields(&count);
        out.push_back({ f, count, 0, DID_GPX_FLASH_CFG, "" });
        break;
    default:
        break;
    }
    return out;
}

/** Struct member names for fields that have no DID mapping of their own */
const char* memberName(const GroupNames& g, const flash_cfg_field_desc_t& f)
{
    if (g.fields == canFields())
    {
        switch (f.id)
        {
        case CAN_CFG_FIELD_PERIOD_MULT:     return "can_period_mult";
        case CAN_CFG_FIELD_TRANSMIT_ADDR:   return "can_transmit_address";
        case CAN_CFG_FIELD_SETTING:         return "can_setting";
        case CAN_CFG_FIELD_RECEIVE_ADDR:    return "can_receive_address";
        }
    }
    else if (g.fields == rmcFields())
    {
        switch (f.id)
        {
        case RMC_CFG_FIELD_BITS:            return "bits";
        case RMC_CFG_FIELD_OPTIONS:         return "options";
        case RMC_CFG_FIELD_PERIOD_MULT:     return "periodMultiple";
        case RMC_CFG_FIELD_NMEA_BITS:       return "nmeaBits";
        case RMC_CFG_FIELD_NMEA_PERIOD:     return "nmeaPeriod";
        }
    }
    return nullptr;
}

std::string fieldName(const GroupNames& g, const flash_cfg_field_desc_t& f)
{
    if (g.did != DID_NULL)
    {
        const data_info_t* info = cISDataMappings::FieldInfoByOffset(g.did, f.offset);
        if (info && info->offset == f.offset)
            return g.prefix + info->name;
    }
    else if (const char* name = memberName(g, f))
    {
        return g.prefix + name;
    }
    return "key" + std::to_string(g.idBase + f.id);
}

/** key -> name for one device type, built on first use */
const std::map<uint16_t, std::string>& nameTable(uint16_t hdwId)
{
    static std::map<uint16_t, std::map<uint16_t, std::string>> tables;
    uint16_t type = (uint16_t)DECODE_HDW_TYPE(hdwId);
    auto it = tables.find(type);
    if (it == tables.end())
    {
        std::map<uint16_t, std::string> table;
        for (const GroupNames& g : groupNames(hdwId))
            for (uint16_t i = 0; i < g.count; i++)
                table[(uint16_t)(g.idBase + g.fields[i].id)] = fieldName(g, g.fields[i]);
        it = tables.emplace(type, std::move(table)).first;
    }
    return it->second;
}

size_t mapHeaderSize(size_t count)
{
    return (count < 24) ? 1 : (count < 256) ? 2 : (count < 65536) ? 3 : 5;
}

void appendUint(std::vector<uint8_t>& out, uint64_t value)
{
    uint8_t buf[9];
    nanocbor_encoder_t enc;
    nanocbor_encoder_init(&enc, buf, sizeof(buf));
    nanocbor_fmt_uint(&enc, value);
    out.insert(out.end(), buf, buf + nanocbor_encoded_len(&enc));
}

void appendMapHeader(std::vector<uint8_t>& out, size_t count)
{
    uint8_t buf[9];
    nanocbor_encoder_t enc;
    nanocbor_encoder_init(&enc, buf, sizeof(buf));
    nanocbor_fmt_map(&enc, count);
    out.insert(out.end(), buf, buf + nanocbor_encoded_len(&enc));
}

size_t entrySize(const CborConfigField& f)
{
    std::vector<uint8_t> key;
    appendUint(key, f.key);
    return key.size() + f.value.size();
}

} // namespace

namespace cborcfg
{

bool parseMap(const uint8_t* buf, size_t size, CborConfigFields& out)
{
    nanocbor_value_t dec, map;
    nanocbor_decoder_init(&dec, buf, size);
    if (nanocbor_enter_map(&dec, &map) < 0)
        return false;

    while (!nanocbor_at_end(&map))
    {
        uint32_t key = 0;
        bool keyOk = (nanocbor_get_type(&map) == NANOCBOR_TYPE_UINT) && (nanocbor_get_uint32(&map, &key) >= 0) && key <= UINT16_MAX;
        if (!keyOk && nanocbor_skip(&map) < 0)
            return false;

        const uint8_t* start = map.cur;
        if (nanocbor_skip(&map) < 0)
            return false;
        if (keyOk)
            out.push_back({ (uint16_t)key, std::vector<uint8_t>(start, map.cur) });
    }

    bool truncated = nanocbor_container_indefinite(&map) ? (map.cur >= map.end) : (nanocbor_container_remaining(&map) != 0);
    return !truncated;
}

std::vector<uint8_t> encodeMap(const CborConfigFields& fields)
{
    std::vector<uint8_t> out;
    appendMapHeader(out, fields.size());
    for (const CborConfigField& f : fields)
    {
        appendUint(out, f.key);
        out.insert(out.end(), f.value.begin(), f.value.end());
    }
    return out;
}

std::vector<CborConfigFields> splitFields(const CborConfigFields& fields, size_t maxMapSize)
{
    std::vector<CborConfigFields> groups;
    size_t entriesSize = 0;
    for (const CborConfigField& f : fields)
    {
        size_t n = entrySize(f);
        if (groups.empty() || mapHeaderSize(groups.back().size() + 1) + entriesSize + n > maxMapSize)
        {
            groups.emplace_back();
            entriesSize = 0;
        }
        groups.back().push_back(f);
        entriesSize += n;
    }
    return groups;
}

std::string keyName(uint16_t hdwId, uint16_t key)
{
    const auto& table = nameTable(hdwId);
    auto it = table.find(key);
    return (it != table.end()) ? it->second : "key" + std::to_string(key);
}

bool keyFromName(uint16_t hdwId, const std::string& name, uint16_t& key)
{
    for (const auto& [k, n] : nameTable(hdwId))
    {
        if (equalsIgnoreCase(n, name))
        {
            key = k;
            return true;
        }
    }
    if (name.size() > 3 && equalsIgnoreCase(name.substr(0, 3), "key"))
    {
        char* end = nullptr;
        unsigned long v = strtoul(name.c_str() + 3, &end, 10);
        if (end && *end == '\0' && v <= UINT16_MAX)
        {
            key = (uint16_t)v;
            return true;
        }
    }
    return false;
}

std::vector<uint16_t> keys(uint16_t hdwId)
{
    std::vector<uint16_t> out;
    for (const GroupNames& g : groupNames(hdwId))
        for (uint16_t i = 0; i < g.count; i++)
            out.push_back((uint16_t)(g.idBase + g.fields[i].id));
    return out;
}

const char* statusName(int status)
{
    switch (status)
    {
    case CBOR_CFG_STATUS_OK:            return "OK";
    case CBOR_CFG_STATUS_PARTIAL:       return "PARTIAL";
    case CBOR_CFG_STATUS_TOO_LARGE:     return "TOO_LARGE";
    case CBOR_CFG_STATUS_BAD_REQUEST:   return "BAD_REQUEST";
    case CBOR_CFG_STATUS_UNSUPPORTED:   return "UNSUPPORTED";
    case CBOR_CFG_STATUS_ERROR:         return "ERROR";
    case CBOR_CFG_RESULT_TIMEOUT:       return "TIMEOUT";
    case CBOR_CFG_RESULT_SEND_FAILED:   return "SEND_FAILED";
    case CBOR_CFG_RESULT_MALFORMED:     return "MALFORMED";
    default:                            return "UNKNOWN";
    }
}

} // namespace cborcfg

std::vector<flash_cfg_cbor_group_t> CborConfigImage::groups()
{
    std::vector<flash_cfg_cbor_group_t> out;
    flash_cfg_cbor_group_t g = {};
    switch (DECODE_HDW_TYPE(hdwId))
    {
    case IS_HARDWARE_TYPE_IMX:
        g.fields = flash_cfg_cbor_imx_fields(&g.count);
        g.data = &imx;
        out.push_back(g);
        g.fields = flash_cfg_cbor_can_fields(&g.count);
        g.data = &can;
        out.push_back(g);
        for (unsigned port = 0; port < RMC_CFG_MAX_PORTS; port++)
        {
            g.fields = flash_cfg_cbor_rmc_port_fields(&g.count);
            g.idBase = flash_cfg_cbor_rmc_port_id_base(port);
            g.data = &rmc[port];
            out.push_back(g);
        }
        break;
    case IS_HARDWARE_TYPE_GPX:
        g.fields = flash_cfg_cbor_gpx_fields(&g.count);
        g.data = &gpx;
        out.push_back(g);
        break;
    default:
        break;
    }
    return out;
}

bool CborConfigImage::apply(const CborConfigFields& fields)
{
    auto g = groups();
    std::vector<uint8_t> map = cborcfg::encodeMap(fields);
    return flash_cfg_cbor_decode_groups(g.data(), g.size(), map.data(), map.size(), nullptr, 0, nullptr) == FLASH_CFG_CBOR_OK;
}

CborConfigFields CborConfigImage::encode(const std::vector<uint16_t>& keys)
{
    auto g = groups();
    CborConfigFields out;
    std::vector<uint8_t> buf(4096);
    uint16_t cursor = 0;
    bool complete = false;
    while (!complete)
    {
        int n = flash_cfg_cbor_encode_groups(g.data(), g.size(), keys.empty() ? nullptr : keys.data(), keys.size(),
                                             &cursor, &complete, buf.data(), buf.size());
        if (n < 0 || !cborcfg::parseMap(buf.data(), (size_t)n, out))
            break;
    }
    return out;
}

namespace {

/** The group and field descriptor that hold key in image, and the group's DID for name mapping */
struct FieldRef
{
    flash_cfg_cbor_group_t group;
    const flash_cfg_field_desc_t* field = nullptr;
    uint32_t did = DID_NULL;
};

bool findField(CborConfigImage& image, uint16_t key, FieldRef& ref)
{
    auto names = groupNames(image.hdwId);
    auto groups = image.groups();
    for (size_t g = 0; g < groups.size() && g < names.size(); g++)
    {
        for (uint16_t i = 0; i < groups[g].count; i++)
        {
            if (groups[g].idBase + groups[g].fields[i].id == key)
            {
                ref.group = groups[g];
                ref.field = &groups[g].fields[i];
                ref.did = names[g].did;
                return true;
            }
        }
    }
    return false;
}

size_t elementSize(const flash_cfg_field_desc_t& f)
{
    switch (f.type)
    {
    case FCTYPE_UINT8:  case FCTYPE_UINT8_ARRAY:    return 1;
    case FCTYPE_UINT16: case FCTYPE_UINT16_ARRAY:   return 2;
    case FCTYPE_UINT32: case FCTYPE_UINT32_ARRAY:   return 4;
    case FCTYPE_UINT64:                             return 8;
    case FCTYPE_FLOAT:  case FCTYPE_FLOAT_ARRAY:    return 4;
    case FCTYPE_DOUBLE: case FCTYPE_DOUBLE_ARRAY:   return 8;
    default:                                        return 0;
    }
}

bool isFloat(const flash_cfg_field_desc_t& f)
{
    return f.type == FCTYPE_FLOAT || f.type == FCTYPE_FLOAT_ARRAY || f.type == FCTYPE_DOUBLE || f.type == FCTYPE_DOUBLE_ARRAY;
}

/** Bitmask-like fields of the unmapped groups are shown in hex */
bool showHex(const std::string& name)
{
    return name.find("bits") != std::string::npos || name.find("Bits") != std::string::npos || name.find("options") != std::string::npos;
}

std::string formatElement(const FieldRef& ref, unsigned index, bool hex)
{
    const uint8_t* p = (const uint8_t*)ref.group.data + ref.field->offset + index * elementSize(*ref.field);
    std::ostringstream oss;
    switch (elementSize(*ref.field))
    {
    case 1: { uint8_t v; memcpy(&v, p, 1); hex ? oss << "0x" << std::hex << (unsigned)v : oss << (unsigned)v; break; }
    case 2: { uint16_t v; memcpy(&v, p, 2); hex ? oss << "0x" << std::hex << v : oss << v; break; }
    case 4:
        if (isFloat(*ref.field)) { float v; memcpy(&v, p, 4); oss << v; }
        else { uint32_t v; memcpy(&v, p, 4); hex ? oss << "0x" << std::hex << v : oss << v; }
        break;
    case 8:
        if (isFloat(*ref.field)) { double v; memcpy(&v, p, 8); oss.precision(12); oss << v; }
        else { uint64_t v; memcpy(&v, p, 8); hex ? oss << "0x" << std::hex << v : oss << v; }
        break;
    }
    return oss.str();
}

bool parseElement(const FieldRef& ref, unsigned index, const std::string& value)
{
    uint8_t* p = (uint8_t*)ref.group.data + ref.field->offset + index * elementSize(*ref.field);
    char* end = nullptr;
    if (isFloat(*ref.field))
    {
        double v = strtod(value.c_str(), &end);
        if (!end || *end != '\0' || value.empty())
            return false;
        if (elementSize(*ref.field) == 4) { float f = (float)v; memcpy(p, &f, 4); }
        else memcpy(p, &v, 8);
        return true;
    }
    unsigned long long v = strtoull(value.c_str(), &end, 0);
    if (!end || *end != '\0' || value.empty() || value[0] == '-')
        return false;
    size_t n = elementSize(*ref.field);
    if (n < 8 && v >> (8 * n))
        return false;
    memcpy(p, &v, n);       // little-endian host
    return true;
}

std::string trim(const std::string& s)
{
    size_t b = s.find_first_not_of(" \t\r\n");
    size_t e = s.find_last_not_of(" \t\r\n");
    return (b == std::string::npos) ? std::string() : s.substr(b, e - b + 1);
}

} // namespace

namespace cborcfg
{

bool parseFieldRef(uint16_t hdwId, const std::string& refIn, uint16_t& key, int& index)
{
    std::string ref = trim(refIn);
    index = -1;
    if (keyFromName(hdwId, ref, key))
        return true;
    // trailing [i] selects an array element; names may themselves contain brackets (rmc[2].bits)
    if (ref.size() > 3 && ref.back() == ']')
    {
        size_t open = ref.rfind('[');
        if (open != std::string::npos && open > 0)
        {
            char* end = nullptr;
            long i = strtol(ref.c_str() + open + 1, &end, 10);
            if (end && *end == ']' && i >= 0 && keyFromName(hdwId, ref.substr(0, open), key))
            {
                index = (int)i;
                return true;
            }
        }
    }
    return false;
}

std::string formatField(CborConfigImage& image, uint16_t key)
{
    FieldRef ref;
    if (!findField(image, key, ref))
        return "";
    std::string name = keyName(image.hdwId, key);

    if (ref.did != DID_NULL)
    {   // as the legacy listing renders it, without its header line
        std::string out;
        if (cISDataMappings::DidBufferToString(ref.did, (const uint8_t*)ref.group.data, out, name))
        {
            size_t nl = out.find('\n');
            return (nl == std::string::npos) ? out : out.substr(nl + 1);
        }
        return "";
    }

    std::ostringstream oss;
    bool hex = showHex(name);
    if (ref.field->count <= 1)
        oss << name << " = " << formatElement(ref, 0, hex) << std::endl;
    else
        for (unsigned i = 0; i < ref.field->count; i++)
            oss << name << "[" << i << "] = " << formatElement(ref, i, hex) << std::endl;
    return oss.str();
}

bool parseAssignment(CborConfigImage& image, const std::string& assignment, uint16_t& key)
{
    size_t eq = assignment.find('=');
    if (eq == std::string::npos)
        return false;
    std::string lhs = trim(assignment.substr(0, eq));
    std::string value = trim(assignment.substr(eq + 1));
    int index;
    FieldRef ref;
    if (!parseFieldRef(image.hdwId, lhs, key, index) || !findField(image, key, ref))
        return false;
    if (index >= (int)_MAX(1u, (unsigned)ref.field->count))
        return false;

    if (ref.did != DID_NULL)
        return cISDataMappings::StringToDidBuffer(ref.did, lhs + "=" + value, (uint8_t*)ref.group.data);

    if (index < 0 && ref.field->count > 1)
        return false;       // an array element must be named, e.g. rmc[0].periodMultiple[5]=2
    return parseElement(ref, (unsigned)_MAX(0, index), value);
}

} // namespace cborcfg
