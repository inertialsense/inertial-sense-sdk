/**
 * @file ISCborFormat.cpp
 * @brief CBOR messages as text: YAML requests and replies, and diagnostic notation. See ISCborFormat.h.
 *
 * @copyright Copyright (c) 2026 Inertial Sense, Inc. All rights reserved.
 */

#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <map>
#include <vector>

#include "ISCborFormat.h"
#include "ISComm.h"
#include "ISDataMappings.h"

namespace {

/** Where a key sits: at the root, or inside a DID's entry at some map depth */
struct Scope
{
    int     did = -1;       //!< root DID key of the entry, or -1 at the root
    int     depth = 0;      //!< map depth inside the entry (1 = the DID's own map)

    Scope child(int key) const
    {
        Scope s = *this;
        if (did < 0)
        {
            if (key >= 0 && key < 0x8000)
            {
                s.did = key;
                s.depth = 1;
            }
        }
        else
            s.depth++;
        return s;
    }
};

/**
 * Depth at which a DID's keys are themselves DIDs: CAPABILITIES lists each DID the device owns, and RMC / RMC_STORED
 * hold each port's subscribed DIDs under the port key.
 */
int didKeyedDepth(int did)
{
    switch (did)
    {
    case CBOR_DID_CAPABILITIES: return 1;
    case CBOR_DID_RMC:
    case CBOR_DID_RMC_STORED:   return 2;
    default:                    return 0;
    }
}

/** A root DID's name: the registry's, else the legacy DID's, else NULL */
const char* didName(int did)
{
    const char* n = cbor_registry_did_name((uint16_t)did);
    if (!n && did > DID_NULL && did < DID_COUNT)
        n = cISDataMappings::DataName((uint32_t)did);
    return n;
}

/** The registry name of key in scope, or NULL */
const char* keyName(const Scope& sc, int key)
{
    if (sc.did < 0)
        return key >= 0x8000 ? cbor_registry_msg_key_name((uint16_t)key) : didName(key);
    if (key >= 0x9000)
        return cbor_registry_msg_key_name((uint16_t)key);
    const char* n = cbor_registry_field_name((uint16_t)sc.did, (uint16_t)key);
    if (!n && key < 0x8000 && sc.depth == didKeyedDepth(sc.did))
        n = didName(key);
    return n;
}

/** Parses a whole string as an unsigned or signed integer (decimal, or 0x hex) */
bool parseInt(const std::string& s, int64_t& v, bool& isUnsigned, uint64_t& u)
{
    if (s.empty())
        return false;
    const char* p = s.c_str();
    char* end = nullptr;
    errno = 0;
    if (*p == '-')
    {
        v = strtoll(p, &end, 0);
        isUnsigned = false;
    }
    else
    {
        u = strtoull(p, &end, 0);
        v = (int64_t)u;
        isUnsigned = true;
    }
    return errno == 0 && end && *end == '\0';
}

/** Parses a whole string as a number */
bool parseDouble(const std::string& s, double& d)
{
    if (s.empty())
        return false;
    char* end = nullptr;
    d = strtod(s.c_str(), &end);
    return end && *end == '\0';
}

/** Resolves a key written as text in scope: a number, a message key, a field name, or a DID where DIDs are keys */
int keyFromString(const Scope& sc, const std::string& s)
{
    int64_t v;
    uint64_t u;
    bool isUnsigned;
    if (parseInt(s, v, isUnsigned, u))
        return (isUnsigned && u <= 0xFFFF) ? (int)u : -1;
    int k = cbor_registry_msg_key(s.c_str());
    if (k >= 0)
        return k;
    if (sc.did < 0)
        return cborDidFromString(s);
    k = cbor_registry_field_key((uint16_t)sc.did, s.c_str());
    if (k < 0 && sc.depth == didKeyedDepth(sc.did))
        k = cborDidFromString(s);
    return k;
}

/** Splits "name[i]" into name and index; index is -1 without one */
std::string splitIndex(const std::string& s, int& index)
{
    index = -1;
    size_t open = s.find('[');
    if (open == std::string::npos || s.back() != ']')
        return s;
    index = atoi(s.c_str() + open + 1);
    return s.substr(0, open);
}

/** True for keys whose value is a device ID or device filter; sc is the scope of the value */
bool isDevIdKey(const Scope& sc, int key)
{
    if (key == CBOR_MSG_SRC_DEV_ID || key == CBOR_MSG_DST_DEV_FILTER)
        return true;
    return sc.did == CBOR_DID_ROUTE_FILTER && sc.depth == 3 && key == CBOR_ROUTE_FILTER__MATCH_DEV;   // in a rule's map
}

/** The enum key's value takes in scope, or NULL */
const cbor_registry_enum_t* keyEnum(const Scope& sc, int key)
{
    if (key < 0)
        return nullptr;
    return cbor_registry_key_enum(sc.did < 0 ? 0 : (uint16_t)sc.did, (uint16_t)key);
}

/** The name of an enum value, or NULL */
const char* enumName(const cbor_registry_enum_t* e, uint64_t v)
{
    for (uint16_t i = 0; e && i < e->count; i++)
        if (e->values[i].key == v)
            return e->values[i].name;
    return nullptr;
}

/** MSG_TYPE as flags and operation, e.g. "REPLY|GET" */
std::string msgTypeName(uint64_t v)
{
    std::string s;
    if (v & CBOR_MSG_TYPE_FLAG_REPLY)
        s = "REPLY|";
    const char* op = enumName(cbor_registry_key_enum(0, CBOR_MSG_TYPE), v & CBOR_MSG_TYPE_OP_MASK);
    return s + (op ? op : std::to_string(v & CBOR_MSG_TYPE_OP_MASK));
}

/** A readable form of an unsigned value of key, or empty if the number is the best form */
std::string uintText(const Scope& sc, int key, uint64_t v)
{
    if (key == CBOR_MSG_TYPE && (sc.did < 0 || key >= 0x9000))
        return msgTypeName(v);
    if (isDevIdKey(sc, key))
        return cborDevFilterToString(v);
    const char* n = enumName(keyEnum(sc, key), v);
    return n ? n : "";
}

/** Shortest text that reads back as the same double */
std::string doubleText(double d, bool diag)
{
    if (std::isnan(d))
        return diag ? "NaN" : ".nan";
    if (std::isinf(d))
        return diag ? (d > 0 ? "Infinity" : "-Infinity") : (d > 0 ? ".inf" : "-.inf");
    char buf[40];
    for (int prec = 6; prec <= 17; prec++)
    {
        snprintf(buf, sizeof(buf), "%.*g", prec, d);
        if (strtod(buf, nullptr) == d)
            break;
    }
    std::string s = buf;
    if (diag && s.find_first_of(".en") == std::string::npos)
        s += ".0";
    return s;
}

/*============================================================================================== CBOR -> text */

/** True if every item of container c was read: a definite-length container cut short also reads as at its end */
bool emptied(const nanocbor_value_t& c)
{
    return nanocbor_container_indefinite(&c) || nanocbor_container_remaining(&c) == 0;
}

/** Appends one data item in diagnostic notation; key is the map key the item is the value of (-1 if none) */
bool diagItem(nanocbor_value_t* it, std::string& out, const Scope& sc, int key, bool names, int listDid);

/** Appends a map key, named when it has a name */
void diagKey(std::string& out, const Scope& sc, int key, bool names)
{
    const char* n = names ? keyName(sc, key) : nullptr;
    char num[16];
    snprintf(num, sizeof(num), (n && key >= 0x8000) ? "0x%04X" : "%d", key);
    out += n ? std::string(n) + "(" + num + ")" : std::string(num);
}

bool diagContainer(nanocbor_value_t* it, std::string& out, const Scope& sc, int key, bool names, bool isMap,
                   int listDid)
{
    nanocbor_value_t c;
    if ((isMap ? nanocbor_enter_map(it, &c) : nanocbor_enter_array(it, &c)) < 0)
        return false;
    out += isMap ? "{" : "[";
    bool first = true;
    while (!nanocbor_at_end(&c))
    {
        if (!first)
            out += ", ";
        first = false;
        if (isMap)
        {
            int k = -1;
            if (nanocbor_get_type(&c) == NANOCBOR_TYPE_UINT)
            {
                uint64_t u;
                if (nanocbor_get_uint64(&c, &u) < 0)
                    return false;
                if (u <= 0xFFFF)
                {
                    k = (int)u;
                    diagKey(out, sc, k, names);
                }
                else
                    out += std::to_string(u);
            }
            else if (!diagItem(&c, out, sc, -1, false, -1))
                return false;
            out += ": ";
            Scope cs = (k >= 0) ? sc.child(k) : sc;
            int childList = -1;
            if (k == CBOR_MSG_ERR_KEYS || k == CBOR_MSG_FIELDS)
                childList = sc.did;
            else if (sc.did == CBOR_DID_CAPABILITIES && sc.depth == 1)
                childList = k;     // a DID's served field keys
            if (!diagItem(&c, out, cs, k, names, childList))
                return false;
        }
        else if (!diagItem(&c, out, sc, key, names, listDid))
            return false;
    }
    if (!emptied(c))
        return false;
    out += isMap ? "}" : "]";
    return nanocbor_leave_container(it, &c) >= 0;
}

bool diagItem(nanocbor_value_t* it, std::string& out, const Scope& sc, int key, bool names, int listDid)
{
    char buf[64];
    switch (nanocbor_get_type(it))
    {
    case NANOCBOR_TYPE_UINT:
    {
        uint64_t u;
        if (nanocbor_get_uint64(it, &u) < 0)
            return false;
        const char* fieldName = (names && listDid >= 0 && u <= 0x7FFF) ?
            cbor_registry_field_name((uint16_t)listDid, (uint16_t)u) : nullptr;
        if (fieldName)
        {
            out += std::string(fieldName) + "(" + std::to_string(u) + ")";
            return true;
        }
        std::string t = names ? uintText(sc, key, u) : "";
        if (names && sc.did < 0 && key == CBOR_MSG_TIME_GPS_NS)
        {
            uint32_t week;
            double tow;
            cborGpsNsToWeekTow(u, week, tow);
            snprintf(buf, sizeof(buf), "week %u tow %.6f", week, tow);
            t = buf;
        }
        else if (names && sc.did < 0 && key == CBOR_MSG_UPTIME_NS)
        {
            snprintf(buf, sizeof(buf), "uptime %.6f s", (double)u * 1e-9);
            t = buf;
        }
        out += t.empty() ? std::to_string(u) : t;
        return true;
    }
    case NANOCBOR_TYPE_NINT:
    {
        int64_t v;
        if (nanocbor_get_int64(it, &v) < 0)
            return false;
        out += std::to_string(v);
        return true;
    }
    case NANOCBOR_TYPE_BSTR:
    {
        const uint8_t* p;
        size_t n;
        if (nanocbor_get_bstr(it, &p, &n) < 0)
            return false;
        out += "h'";
        for (size_t i = 0; i < n; i++)
        {
            snprintf(buf, sizeof(buf), "%02x", p[i]);
            out += buf;
        }
        out += "'";
        return true;
    }
    case NANOCBOR_TYPE_TSTR:
    {
        const uint8_t* p;
        size_t n;
        if (nanocbor_get_tstr(it, &p, &n) < 0)
            return false;
        out += '"';
        for (size_t i = 0; i < n; i++)
        {
            char ch = (char)p[i];
            if (ch == '"' || ch == '\\')
                out += '\\';
            out += ch;
        }
        out += '"';
        return true;
    }
    case NANOCBOR_TYPE_ARR:
        return diagContainer(it, out, sc, key, names, false, listDid);
    case NANOCBOR_TYPE_MAP:
        return diagContainer(it, out, sc, key, names, true, listDid);
    case NANOCBOR_TYPE_TAG:
    {
        uint64_t tag;
        if (nanocbor_get_tag64(it, &tag) < 0)
            return false;
        out += std::to_string(tag) + "(";
        if (!diagItem(it, out, sc, key, names, listDid))
            return false;
        out += ")";
        return true;
    }
    case NANOCBOR_TYPE_FLOAT:
    {
        bool b;
        uint8_t simple;
        double d;
        if (nanocbor_get_bool(it, &b) >= 0)
            out += b ? "true" : "false";
        else if (nanocbor_get_null(it) >= 0)
            out += "null";
        else if (nanocbor_get_undefined(it) >= 0)
            out += "undefined";
        else if (nanocbor_get_double(it, &d) >= 0)
            out += doubleText(d, true);
        else if (nanocbor_get_simple(it, &simple) >= 0)
            out += "simple(" + std::to_string(simple) + ")";
        else
            return false;
        return true;
    }
    default:
        return false;
    }
}

/** Converts one data item to YAML; key is the map key the item is the value of (-1 if none) */
bool yamlItem(nanocbor_value_t* it, YAML::Node& node, const Scope& sc, int key)
{
    switch (nanocbor_get_type(it))
    {
    case NANOCBOR_TYPE_UINT:
    {
        uint64_t u;
        if (nanocbor_get_uint64(it, &u) < 0)
            return false;
        std::string t = uintText(sc, key, u);
        node = t.empty() ? YAML::Node(u) : YAML::Node(t);
        return true;
    }
    case NANOCBOR_TYPE_NINT:
    {
        int64_t v;
        if (nanocbor_get_int64(it, &v) < 0)
            return false;
        node = v;
        return true;
    }
    case NANOCBOR_TYPE_BSTR:
    case NANOCBOR_TYPE_TAG:
    {
        std::string d;
        if (!diagItem(it, d, sc, key, false, -1))
            return false;
        node = d;
        return true;
    }
    case NANOCBOR_TYPE_TSTR:
    {
        const uint8_t* p;
        size_t n;
        if (nanocbor_get_tstr(it, &p, &n) < 0)
            return false;
        node = std::string((const char*)p, n);
        return true;
    }
    case NANOCBOR_TYPE_ARR:
    {
        nanocbor_value_t c;
        if (nanocbor_enter_array(it, &c) < 0)
            return false;
        node = YAML::Node(YAML::NodeType::Sequence);
        while (!nanocbor_at_end(&c))
        {
            YAML::Node e;
            if (!yamlItem(&c, e, sc, key))
                return false;
            node.push_back(e);
        }
        return emptied(c) && nanocbor_leave_container(it, &c) >= 0;
    }
    case NANOCBOR_TYPE_MAP:
    {
        nanocbor_value_t c;
        if (nanocbor_enter_map(it, &c) < 0)
            return false;
        node = YAML::Node(YAML::NodeType::Map);
        while (!nanocbor_at_end(&c))
        {
            uint64_t u;
            if (nanocbor_get_type(&c) != NANOCBOR_TYPE_UINT || nanocbor_get_uint64(&c, &u) < 0 || u > 0xFFFF)
                return false;
            int k = (int)u;
            YAML::Node v;
            if (!yamlItem(&c, v, sc.child(k), k))
                return false;
            const char* n = keyName(sc, k);
            node[n ? std::string(n) : std::to_string(k)] = v;
        }
        return emptied(c) && nanocbor_leave_container(it, &c) >= 0;
    }
    case NANOCBOR_TYPE_FLOAT:
    {
        bool b;
        double d;
        if (nanocbor_get_bool(it, &b) >= 0)
            node = b;
        else if (nanocbor_get_null(it) >= 0 || nanocbor_get_undefined(it) >= 0)
            node = YAML::Node(YAML::NodeType::Null);
        else if (nanocbor_get_double(it, &d) >= 0)
            node = d;
        else
            return false;
        return true;
    }
    default:
        return false;
    }
}

/*============================================================================================== YAML -> CBOR */

/** Encodes CBOR into a growing buffer */
class Writer
{
public:
    Writer() : buf_(ISB_MAX_DATASET_SIZE) { nanocbor_encoder_init(&enc, buf_.data(), buf_.size()); }
    nanocbor_encoder_t enc;
    bool ok() const { return enc.cur <= enc.end; }
    std::vector<uint8_t> bytes() const { return std::vector<uint8_t>(buf_.data(), (const uint8_t*)enc.cur); }
private:
    std::vector<uint8_t> buf_;
};

/** The text of a YAML scalar */
std::string scalar(const YAML::Node& n)
{
    return n.IsScalar() ? n.Scalar() : std::string();
}

/** Encodes a scalar given without a field type: enum name, device filter, bool, integer, number or text */
bool encodeGenericScalar(nanocbor_encoder_t* enc, const Scope& sc, int key, const std::string& s, std::string& error)
{
    const cbor_registry_enum_t* e = keyEnum(sc, key);
    for (uint16_t i = 0; e && i < e->count; i++)
        if (s == e->values[i].name)
            return nanocbor_fmt_uint(enc, e->values[i].key) >= 0;
    int64_t v;
    uint64_t u;
    bool isUnsigned;
    double d;
    if (parseInt(s, v, isUnsigned, u))
        return (isUnsigned ? nanocbor_fmt_uint(enc, u) : nanocbor_fmt_int(enc, v)) >= 0;
    if (isDevIdKey(sc, key))
    {
        uint64_t filter;
        if (!cborDevFilterFromString(s, filter))
        {
            error = "not a device filter: " + s;
            return false;
        }
        return nanocbor_fmt_uint(enc, filter) >= 0;
    }
    if (s == "true" || s == "false")
        return nanocbor_fmt_bool(enc, s == "true") >= 0;
    if (s == "null" || s == "~")
        return nanocbor_fmt_null(enc) >= 0;
    if (parseDouble(s, d))
        return ((double)(float)d == d ? nanocbor_fmt_float(enc, (float)d) : nanocbor_fmt_double(enc, d)) >= 0;
    if (e)
    {
        error = "unknown " + std::string(e->name) + " value: " + s;
        return false;
    }
    return nanocbor_put_tstr(enc, s.c_str()) >= 0;
}

bool encodeGeneric(nanocbor_encoder_t* enc, const Scope& sc, int key, const YAML::Node& n, std::string& error);

/** Encodes a YAML map without a schema: keys resolved in scope */
bool encodeGenericMap(nanocbor_encoder_t* enc, const Scope& sc, const YAML::Node& n, std::string& error)
{
    if (nanocbor_fmt_map(enc, n.size()) < 0)
        return false;
    for (auto kv : n)
    {
        std::string ks = kv.first.as<std::string>();
        int k = keyFromString(sc, ks);
        if (k < 0)
        {
            error = "unknown key: " + ks;
            return false;
        }
        if (nanocbor_fmt_uint(enc, (uint64_t)k) < 0 || !encodeGeneric(enc, sc.child(k), k, kv.second, error))
            return false;
    }
    return true;
}

bool encodeGeneric(nanocbor_encoder_t* enc, const Scope& sc, int key, const YAML::Node& n, std::string& error)
{
    if (!n || n.IsNull())
        return nanocbor_fmt_null(enc) >= 0;
    if (n.IsScalar())
        return encodeGenericScalar(enc, sc, key, n.Scalar(), error);
    if (n.IsSequence())
    {
        if (nanocbor_fmt_array(enc, n.size()) < 0)
            return false;
        for (auto e : n)
            if (!encodeGeneric(enc, sc, key, e, error))
                return false;
        return true;
    }
    return encodeGenericMap(enc, sc, n, error);
}

/** Encodes one scalar of a schema field from text */
bool encodeFieldScalar(nanocbor_encoder_t* enc, const cbor_field_t* f, const Scope& sc, const std::string& s,
                       std::string& error)
{
    const cbor_registry_enum_t* e = keyEnum(sc, f->key);
    for (uint16_t i = 0; e && i < e->count; i++)
        if (s == e->values[i].name)
            return nanocbor_fmt_uint(enc, e->values[i].key) >= 0;

    int64_t v;
    uint64_t u;
    bool isUnsigned;
    double d;
    switch (f->type)
    {
    case CBOR_T_UINT:
        if (!parseInt(s, v, isUnsigned, u) || !isUnsigned || (f->size < 8 && u >> (f->size * 8)))
            break;
        return nanocbor_fmt_uint(enc, u) >= 0;
    case CBOR_T_INT:
    {
        if (!parseInt(s, v, isUnsigned, u) || (isUnsigned && u > (uint64_t)INT64_MAX))
            break;
        int64_t lim = (f->size < 8) ? ((int64_t)1 << (f->size * 8 - 1)) : 0;
        if (f->size < 8 && (v >= lim || v < -lim))
            break;
        return nanocbor_fmt_int(enc, v) >= 0;
    }
    case CBOR_T_FLOAT:
        if (!parseDouble(s, d))
            break;
        return (f->size == 4 ? nanocbor_fmt_float(enc, (float)d) : nanocbor_fmt_double(enc, d)) >= 0;
    case CBOR_T_BOOL:
        if (s == "true" || s == "1")
            return nanocbor_fmt_bool(enc, true) >= 0;
        if (s == "false" || s == "0")
            return nanocbor_fmt_bool(enc, false) >= 0;
        break;
    case CBOR_T_TEXT:
        if (s.size() >= f->count)
            break;
        return nanocbor_put_tstr(enc, s.c_str()) >= 0;
    case CBOR_T_BYTES:
    {
        if (s.size() != (size_t)f->count * 2)
            break;
        std::vector<uint8_t> b(f->count);
        for (size_t i = 0; i < b.size(); i++)
            b[i] = (uint8_t)strtoul(s.substr(i * 2, 2).c_str(), nullptr, 16);
        return nanocbor_put_bstr(enc, b.data(), b.size()) >= 0;
    }
    default:
        break;
    }
    const char* n = keyName(sc, f->key);
    error = "invalid value for " + std::string(n ? n : std::to_string(f->key)) + ": " + s;
    return false;
}

/** True if f is an array of numbers or bools (not text or bytes, which are one value) */
bool isArrayField(const cbor_field_t* f)
{
    return f->count > 0 && f->type != CBOR_T_TEXT && f->type != CBOR_T_BYTES && f->type != CBOR_T_MAP;
}

bool encodeSchemaMap(nanocbor_encoder_t* enc, const cbor_schema_t* schema, const Scope& sc, const YAML::Node& n,
                     const uint8_t* current, std::string& error);

/** Encodes a schema field's value. elements: values set by index (name[i]), which need current. */
bool encodeField(nanocbor_encoder_t* enc, const cbor_field_t* f, const Scope& sc, const YAML::Node* whole,
                 const std::map<int, YAML::Node>& elements, const uint8_t* current, std::string& error)
{
    if (f->type == CBOR_T_MAP)
    {
        if (!whole || !whole->IsMap())
        {
            error = "a map is needed for " + std::string(keyName(sc, f->key) ? keyName(sc, f->key) : "field");
            return false;
        }
        return encodeSchemaMap(enc, f->sub, sc.child(f->key), *whole, current ? current + f->offset : nullptr, error);
    }
    if (!isArrayField(f))
    {
        if (!whole || !whole->IsScalar())
        {
            error = "a single value is needed for " + std::string(keyName(sc, f->key) ? keyName(sc, f->key) : "field");
            return false;
        }
        return encodeFieldScalar(enc, f, sc, whole->Scalar(), error);
    }

    cbor_field_t elem = *f;
    elem.count = 0;
    elem.offset = 0;
    if (whole)
    {
        std::vector<std::string> vals;
        if (whole->IsSequence())
            for (auto e : *whole)
                vals.push_back(scalar(e));
        else
            vals.push_back(scalar(*whole));
        if (vals.size() > f->count)
        {
            error = std::string(keyName(sc, f->key) ? keyName(sc, f->key) : "field") + " has only " +
                    std::to_string(f->count) + " elements";
            return false;
        }
        if (nanocbor_fmt_array(enc, vals.size()) < 0)
            return false;
        for (const std::string& v : vals)
            if (!encodeFieldScalar(enc, &elem, sc, v, error))
                return false;
        return true;
    }

    // Elements set by index: the array is sent whole, the other elements taken from the current values
    if (!current)
    {
        error = "current values are needed to set one element of " +
                std::string(keyName(sc, f->key) ? keyName(sc, f->key) : "field");
        return false;
    }
    if (nanocbor_fmt_array(enc, f->count) < 0)
        return false;
    for (int i = 0; i < f->count; i++)
    {
        auto it = elements.find(i);
        if (it != elements.end())
        {
            if (!encodeFieldScalar(enc, &elem, sc, scalar(it->second), error))
                return false;
        }
        else if (cbor_codec_encode_value(enc, &elem, current + f->offset + (size_t)i * f->size) < 0)
            return false;
    }
    for (auto& e : elements)
        if (e.first < 0 || e.first >= f->count)
        {
            error = "index " + std::to_string(e.first) + " is outside " +
                    std::string(keyName(sc, f->key) ? keyName(sc, f->key) : "field");
            return false;
        }
    return true;
}

/** Encodes a YAML map onto a schema: names resolved to fields, name[i] grouped into one array per field */
bool encodeSchemaMap(nanocbor_encoder_t* enc, const cbor_schema_t* schema, const Scope& sc, const YAML::Node& n,
                     const uint8_t* current, std::string& error)
{
    struct Entry
    {
        const cbor_field_t*         field = nullptr;
        const YAML::Node*           whole = nullptr;
        std::map<int, YAML::Node>   elements;
        YAML::Node                  raw;        //!< value of a key the schema does not know, sent as given
        bool                        isRaw = false;
    };
    std::vector<std::pair<int, Entry>> entries;
    std::vector<YAML::Node> values;     // keeps each value alive for Entry::whole
    values.reserve(n.size());
    for (auto kv : n)
    {
        std::string ks = kv.first.as<std::string>();
        int index;
        std::string name = splitIndex(ks, index);
        int k = keyFromString(sc, name);
        if (k < 0)
        {
            error = "unknown field: " + ks;
            return false;
        }
        Entry* e = nullptr;
        for (auto& p : entries)
            if (p.first == k)
                e = &p.second;
        if (!e)
        {
            entries.push_back({ k, Entry() });
            e = &entries.back().second;
        }
        e->field = cbor_codec_find_field(schema, (uint16_t)k);
        if (!e->field)
        {
            e->isRaw = true;
            e->raw = kv.second;
        }
        else if (index >= 0)
            e->elements[index] = kv.second;
        else
        {
            values.push_back(kv.second);
            e->whole = &values.back();
        }
    }
    if (nanocbor_fmt_map(enc, entries.size()) < 0)
        return false;
    for (auto& p : entries)
    {
        if (nanocbor_fmt_uint(enc, (uint64_t)p.first) < 0)
            return false;
        bool ok = p.second.isRaw ? encodeGeneric(enc, sc.child(p.first), p.first, p.second.raw, error) :
                  encodeField(enc, p.second.field, sc, p.second.whole, p.second.elements, current, error);
        if (!ok)
            return false;
    }
    return true;
}

/** Collects GET key paths from a YAML value under prefix */
bool collectPaths(const YAML::Node& n, const Scope& sc, cbor_path_t prefix, std::vector<cbor_path_t>& paths,
                  std::string& error)
{
    auto add = [&](const std::string& ks, const YAML::Node* sub) -> bool
    {
        int index;
        std::string name = splitIndex(ks, index);
        int k = keyFromString(sc, name);
        if (k < 0)
        {
            error = "unknown field: " + ks;
            return false;
        }
        if (prefix.depth >= CBOR_PATH_MAX_DEPTH)
        {
            error = "key path deeper than " + std::to_string(CBOR_PATH_MAX_DEPTH);
            return false;
        }
        cbor_path_t p = prefix;
        p.key[p.depth++] = (uint16_t)k;
        if (sub && !sub->IsNull())
            return collectPaths(*sub, sc.child(k), p, paths, error);
        paths.push_back(p);
        return true;
    };
    if (!n || n.IsNull())
        return true;
    if (n.IsScalar())
        return add(n.Scalar(), nullptr);
    if (n.IsSequence())
    {
        for (auto e : n)
            if (!(e.IsScalar() ? add(e.Scalar(), nullptr) : collectPaths(e, sc, prefix, paths, error)))
                return false;
        return true;
    }
    for (auto kv : n)
    {
        YAML::Node v = kv.second;
        if (!add(kv.first.as<std::string>(), &v))
            return false;
    }
    return true;
}

/** Applies a root message key to the request's options, or encodes it to send as given */
bool applyEnvelopeKey(CborRequest& req, int key, const YAML::Node& v, std::vector<std::pair<uint16_t,
                      std::vector<uint8_t>>>& raw, std::string& error)
{
    std::string s = scalar(v);
    Scope root;
    switch (key)
    {
    case CBOR_MSG_TYPE:
    {
        const cbor_registry_enum_t* e = cbor_registry_key_enum(0, CBOR_MSG_TYPE);
        for (uint16_t i = 0; e && i < e->count; i++)
            if (s == e->values[i].name)
            {
                req.type = (uint8_t)e->values[i].key;
                return true;
            }
        req.type = (uint8_t)strtoul(s.c_str(), nullptr, 0);
        return true;
    }
    case CBOR_MSG_DST_DEV_FILTER:
    {
        uint64_t filter;
        if (!cborDevFilterFromString(s, filter))
        {
            error = "not a device filter: " + s;
            return false;
        }
        req.to(filter);
        return true;
    }
    case CBOR_MSG_ACK_TYPE:
    {
        const cbor_registry_enum_t* e = cbor_registry_key_enum(0, CBOR_MSG_ACK_TYPE);
        for (uint16_t i = 0; e && i < e->count; i++)
            if (s == e->values[i].name)
            {
                req.ack((uint8_t)e->values[i].key);
                return true;
            }
        req.ack((uint8_t)strtoul(s.c_str(), nullptr, 0));
        return true;
    }
    case CBOR_MSG_OPTION_NO_TIME:
        req.omitTime = (s == "true" || s == "1");
        return true;
    case CBOR_MSG_REQ_ID:
        error = "MSG_REQ_ID is assigned when the request is sent";
        return false;
    default:
    {
        Writer w;
        if (!encodeGeneric(&w.enc, root, key, v, error))
            return false;
        raw.emplace_back((uint16_t)key, w.bytes());
        return true;
    }
    }
}

}   // namespace

int cborDidFromString(const std::string& s)
{
    int64_t v;
    uint64_t u;
    bool isUnsigned;
    if (parseInt(s, v, isUnsigned, u))
        return (isUnsigned && u < 0x8000) ? (int)u : -1;
    int k = cbor_registry_did_key(s.c_str());
    if (k >= 0)
        return k;
    uint32_t did = cISDataMappings::NameToDid(s);
    return (did > DID_NULL && did < DID_COUNT) ? (int)did : -1;
}

bool cborRequestFromYaml(const YAML::Node& node, uint8_t op, CborRequest& req, std::string& error,
                         const CborCurrentValues& current)
{
    if (!node.IsMap())
    {
        error = "expected a map, e.g. {FLASH_CFG: {ser0BaudRate}}";
        return false;
    }
    Scope root;
    std::vector<std::pair<uint16_t, std::vector<uint8_t>>> raw;
    for (auto kv : node)
    {   // envelope keys first: MSG_TYPE may change how the DIDs are read
        std::string ks = kv.first.as<std::string>();
        int k = keyFromString(root, ks);
        if (k < 0)
        {
            error = "unknown DID or message key: " + ks;
            return false;
        }
        if (k >= 0x8000 && !applyEnvelopeKey(req, k, kv.second, raw, error))
            return false;
    }
    if (req.type == 0)
        req.type = op;
    uint8_t reqOp = cbor_msg_op(req.type);

    std::vector<std::pair<uint16_t, std::vector<uint8_t>>> dids;
    for (auto kv : node)
    {
        int did = keyFromString(root, kv.first.as<std::string>());
        if (did >= 0x8000)
            continue;
        Scope sc = root.child(did);
        if (reqOp == CBOR_MSG_TYPE_GET)
        {
            std::vector<cbor_path_t> paths;
            cbor_path_t none = {};
            if (!collectPaths(kv.second, sc, none, paths, error))
                return false;
            CborRequest tmp;
            tmp.get((uint16_t)did, paths);
            dids.push_back(tmp.entries.front());
            continue;
        }
        Writer w;
        const cbor_schema_t* schema = cbor_registry_did_schema((uint16_t)did);
        bool ok;
        if (!kv.second.IsMap())
        {
            error = "a map of field values is needed for " + kv.first.as<std::string>();
            return false;
        }
        if (schema)
            ok = encodeSchemaMap(&w.enc, schema, sc, kv.second,
                                 current ? (const uint8_t*)current((uint16_t)did) : nullptr, error);
        else
            ok = encodeGenericMap(&w.enc, sc, kv.second, error);
        if (!ok || !w.ok())
        {
            if (error.empty())
                error = "values do not fit one message";
            return false;
        }
        dids.emplace_back((uint16_t)did, w.bytes());
    }
    req.entries.insert(req.entries.end(), raw.begin(), raw.end());
    req.entries.insert(req.entries.end(), dids.begin(), dids.end());
    return true;
}

YAML::Node cborToYaml(const uint8_t* msg, size_t len, bool includeEnvelope)
{
    nanocbor_value_t it, map;
    nanocbor_decoder_init(&it, msg, len);
    if (nanocbor_enter_map(&it, &map) < 0)
        return YAML::Node();
    YAML::Node out(YAML::NodeType::Map);
    Scope root;
    while (!nanocbor_at_end(&map))
    {
        uint64_t u;
        if (nanocbor_get_type(&map) != NANOCBOR_TYPE_UINT || nanocbor_get_uint64(&map, &u) < 0 || u > 0xFFFF)
            return YAML::Node();
        int k = (int)u;
        if (k >= 0x8000 && !includeEnvelope)
        {
            if (nanocbor_skip(&map) < 0)
                return YAML::Node();
            continue;
        }
        YAML::Node v;
        if (!yamlItem(&map, v, root.child(k), k))
            return YAML::Node();
        const char* n = keyName(root, k);
        out[n ? std::string(n) : std::to_string(k)] = v;
    }
    return emptied(map) ? out : YAML::Node();
}

std::string cborToDiag(const uint8_t* cbor, size_t len, bool names)
{
    nanocbor_value_t it;
    nanocbor_decoder_init(&it, cbor, len);
    std::string out;
    Scope root;
    if (!diagItem(&it, out, root, -1, names, -1))
        out += "<malformed>";
    return out;
}
