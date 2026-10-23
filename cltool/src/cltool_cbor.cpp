/**
 * @file cltool_cbor.cpp
 * @brief cltool over CBOR messaging. See cltool_cbor.h.
 *
 * @copyright Copyright (c) 2026 Inertial Sense, Inc. All rights reserved.
 */

#include <cstring>
#include <iostream>
#include <optional>
#include <set>

#include "cltool.h"
#include "cltool_cbor.h"
#include "ISCborFormat.h"
#include "ISDataMappings.h"

using namespace std;

namespace {

/** Time allowed for the CAPABILITIES probe: firmware without CBOR does not answer */
constexpr uint32_t CAPABILITIES_TIMEOUT_MS = 500;

/** Prints a YAML node on its own lines */
void printYaml(const YAML::Node& node)
{
    YAML::Emitter out;
    out << node;
    if (out.good())
        cout << out.c_str() << endl;
    else
        cerr << "YAML emitter error: " << out.GetLastError() << endl;
}

/** The DIDs a device serves over CBOR, from its CAPABILITIES; empty if it does not answer CBOR */
set<uint16_t> servedDids(ISDevice& device, optional<uint64_t> dst)
{
    set<uint16_t> dids;
    CborRequest req(CBOR_MSG_TYPE_GET);
    if (dst)
        req.to(*dst);
    req.get(CBOR_DID_CAPABILITIES);
    for (const CborReply& rep : device.cborRequest(req, CAPABILITIES_TIMEOUT_MS))
    {
        nanocbor_value_t it, map;
        if (!rep.find(CBOR_DID_CAPABILITIES, it) || nanocbor_enter_map(&it, &map) < 0)
            continue;
        while (!nanocbor_at_end(&map))
        {
            uint16_t did;
            if (nanocbor_get_uint16(&map, &did) < 0 || nanocbor_skip(&map) < 0)
                break;
            dids.insert(did);
        }
    }
    return dids;
}

/** Field keys whose bytes differ between two copies of a struct described by schema */
vector<uint16_t> changedKeys(const cbor_schema_t& schema, const void* before, const void* after)
{
    vector<uint16_t> keys;
    for (uint16_t i = 0; i < schema.count; i++)
    {
        const cbor_field_t& f = schema.fields[i];
        if (memcmp((const uint8_t*)before + f.offset, (const uint8_t*)after + f.offset, cbor_codec_field_size(&f)))
            keys.push_back(f.key);
    }
    return keys;
}

/** Names the key paths in a DID's MSG_ERR_KEYS, e.g. "ser1BaudRate" */
string errKeyNames(const CborReply& rep, uint16_t did)
{
    string s;
    for (const cbor_path_t& p : rep.errKeys(did))
    {
        if (!s.empty())
            s += ", ";
        for (uint8_t d = 0; d < p.depth; d++)
        {
            const char* n = cbor_registry_field_name(did, p.key[d]);
            s += (d ? "." : "") + (n ? string(n) : to_string(p.key[d]));
        }
    }
    return s;
}

/** The name of a MSG_STATUS value */
string statusName(uint8_t status)
{
    const cbor_registry_enum_t* e = cbor_registry_key_enum(0, CBOR_MSG_STATUS);
    for (uint16_t i = 0; e && i < e->count; i++)
        if (e->values[i].key == status)
            return e->values[i].name;
    return to_string(status);
}

/**
 * Reports the outcome of a DATA write of did: the device's refusal with its reason, or the values it kept differently
 * (an ACK_MINIMAL reply carries only those).
 * @return true if the write was accepted
 */
bool reportWrite(const vector<CborReply>& reps, uint16_t did, const char* what)
{
    if (reps.empty())
    {
        cout << "Failed to set " << what << " flash config: no reply." << endl;
        return false;
    }
    const CborReply& rep = reps.front();
    uint8_t status = rep.status(did);
    if (status != CBOR_MSG_STATUS_OK)
    {
        string keys = errKeyNames(rep, did);
        cout << "Failed to set " << what << " flash config: " << statusName(status) << (keys.empty() ? "" : " (" + keys + ")")
             << "." << endl;
        return false;
    }
    if (rep.has(did))
    {   // values the device kept differently from what was sent
        YAML::Node y = cborToYaml(rep.msg.data(), rep.msg.size());
        cout << "The device adjusted:" << endl;
        printYaml(y);
    }
    return true;
}

/** True if a device filter can match more than one device (one of its sub-fields is a wildcard) */
bool isWildcard(uint64_t filter)
{
    return cborDevFilterToString(filter).find("-1") != string::npos;
}

}   // namespace

void cltool_cborOut(const char* direction, const uint8_t* msg, size_t len)
{
    cout << "CBOR " << direction << ": " << cborToDiag(msg, len, g_commandLineOptions.cborOutNames) << endl;
}

void cltool_cborWatch(ISDevice& device)
{
    if (g_commandLineOptions.cborOut)
        device.cborOnSent = [](const uint8_t* msg, size_t len) { cltool_cborOut("tx", msg, len); };
}

bool cltool_cborGetSet(InertialSense& inertialSenseInterface)
{
    if (inertialSenseInterface.getDevices().empty())
        return false;
    device_handle_t device = inertialSenseInterface.getDevices().front();
    cltool_cborWatch(*device);

    bool isSet = g_commandLineOptions.cborSetNode && g_commandLineOptions.cborSetNode.size() > 0;
    const YAML::Node& node = isSet ? g_commandLineOptions.cborSetNode : g_commandLineOptions.cborGetNode;

    CborRequest req;
    if (!g_commandLineOptions.cborDst.empty())
    {
        uint64_t filter;
        cborDevFilterFromString(g_commandLineOptions.cborDst, filter);  // checked when the option was parsed
        req.to(filter);
    }
    if (g_commandLineOptions.cborAck >= 0)
        req.ack((uint8_t)g_commandLineOptions.cborAck);

    // name[i] in a -cbor-set needs the current values of its DID: read them first
    std::map<uint16_t, std::vector<uint8_t>> current;
    CborCurrentValues currentFn = [&](uint16_t did) -> const void*
    {
        auto it = current.find(did);
        if (it != current.end())
            return it->second.data();
        const cbor_schema_t* schema = cbor_registry_did_schema(did);
        if (!schema)
            return nullptr;
        std::vector<uint8_t>& buf = current[did];
        buf.assign(ISB_MAX_DATASET_SIZE, 0);
        if (device->cborGetStruct(did, *schema, buf.data(), req.dst, g_commandLineOptions.cborTimeoutMs) < 0)
            return nullptr;
        return buf.data();
    };

    string error;
    if (!cborRequestFromYaml(node, isSet ? CBOR_MSG_TYPE_DATA : CBOR_MSG_TYPE_GET, req, error, currentFn))
    {
        cout << "Invalid " << (isSet ? "-cbor-set" : "-cbor-get") << " argument: " << error << endl;
        return false;
    }

    bool collectAll = req.dst && isWildcard(*req.dst);
    vector<CborReply> reps = device->cborRequest(req, g_commandLineOptions.cborTimeoutMs, collectAll);
    if (reps.empty())
    {
        if (req.ackType == CBOR_ACK_TYPE_ACK_NONE)
            return true;    // nothing is answered
        cout << "No CBOR reply." << endl;
        return false;
    }

    bool ok = true;
    for (const CborReply& rep : reps)
    {
        YAML::Node y = cborToYaml(rep.msg.data(), rep.msg.size());
        for (auto kv : y)
            if (kv.second.IsMap() && kv.second["MSG_STATUS"] && kv.second["MSG_STATUS"].as<string>() != "OK" &&
                kv.second["MSG_STATUS"].as<string>() != "RESP_TOO_LARGE")
                ok = false;
        if (reps.size() > 1 || collectAll)
        {   // several devices may answer: say which one each reply is from
            YAML::Node byDevice;
            byDevice[cborDevFilterToString(rep.env.srcDevId)] = (y.size() ? y : YAML::Node("OK"));
            printYaml(byDevice);
        }
        else if (y.size())
            printYaml(y);
        else
            cout << "OK" << endl;
    }
    return ok;
}

bool cltool_cborImxFlashCfg(ISDevice& device, const string& str, bool& handled)
{
    cltool_cborWatch(device);
    handled = servedDids(device, nullopt).count(CBOR_DID_FLASH_CFG) > 0;
    if (!handled)
        return true;

    // Start from the host's copy for the members CBOR does not carry (key); size and checksum follow from the values
    nvm_flash_cfg_t cfg = {};
    device.ImxFlashConfig(cfg, 0);
    if (device.cborGetStruct(CBOR_DID_FLASH_CFG, g_cborSchema_FLASH_CFG, &cfg) < 0)
    {
        cout << "Failed to sync IMX flash config." << endl;
        return false;
    }
    cfg.size = sizeof(cfg);
    cfg.checksum = flashChecksum32(&cfg, sizeof(cfg));

    if (str.find('=') == string::npos)
    {   // read
        string output;
        if (cISDataMappings::DidBufferToString(DID_FLASH_CONFIG, (uint8_t*)&cfg, output, str))
            cout << output;
        else
            cout << "Failed to read flash config." << endl;
        return true;
    }

    nvm_flash_cfg_t want = cfg;
    if (!cISDataMappings::StringToDidBuffer(DID_FLASH_CONFIG, str, (uint8_t*)&want))
    {
        cout << "Failed to parse flash config string." << endl;
        return false;
    }
    vector<uint16_t> keys = changedKeys(g_cborSchema_FLASH_CFG, &cfg, &want);
    if (keys.empty())
        return true;
    CborRequest req(CBOR_MSG_TYPE_DATA);
    req.set(CBOR_DID_FLASH_CFG, g_cborSchema_FLASH_CFG, &want, keys);
    return reportWrite(device.cborRequest(req), CBOR_DID_FLASH_CFG, "IMX");
}

bool cltool_cborGpxFlashCfg(ISDevice& device, const string& str, bool& handled)
{
    cltool_cborWatch(device);
    bool direct = device.devInfo.hardwareType == IS_HARDWARE_TYPE_GPX;
    bool write = str.find('=') != string::npos;
    handled = false;
    if (!direct && write)
        return true;    // DID_GPX_FLASH_CFG to the IMX, which proxies it to its GPX through its own copy

    optional<uint64_t> dst;
    if (!direct)
    {   // the GPX behind this IMX
        uint64_t filter;
        cborDevFilterFromString("GPX", filter);
        dst = filter;
    }
    handled = servedDids(device, dst).count(CBOR_DID_GPX_FLASH_CFG) > 0;
    if (!handled)
        return true;

    // Start from the host's copy for the members CBOR does not carry (key, reserved); size and checksum follow
    gpx_flash_cfg_t cfg = {};
    device.GpxFlashConfig(cfg, 0);
    if (device.cborGetStruct(CBOR_DID_GPX_FLASH_CFG, g_cborSchema_GPX_FLASH_CFG, &cfg, dst) < 0)
    {
        cout << "Failed to sync GPX flash config." << endl;
        return false;
    }
    cfg.size = sizeof(cfg);
    cfg.checksum = flashChecksum32(&cfg, sizeof(cfg));

    if (!write)
    {
        string output;
        if (cISDataMappings::DidBufferToString(DID_GPX_FLASH_CFG, (uint8_t*)&cfg, output, str))
            cout << output;
        else
            cout << "Failed to read flash config." << endl;
        return true;
    }

    gpx_flash_cfg_t want = cfg;
    if (!cISDataMappings::StringToDidBuffer(DID_GPX_FLASH_CFG, str, (uint8_t*)&want))
    {
        cout << "Failed to parse flash config string." << endl;
        return false;
    }
    vector<uint16_t> keys = changedKeys(g_cborSchema_GPX_FLASH_CFG, &cfg, &want);
    if (keys.empty())
        return true;
    CborRequest req(CBOR_MSG_TYPE_DATA);
    req.set(CBOR_DID_GPX_FLASH_CFG, g_cborSchema_GPX_FLASH_CFG, &want, keys);
    return reportWrite(device.cborRequest(req), CBOR_DID_GPX_FLASH_CFG, "GPX");
}
