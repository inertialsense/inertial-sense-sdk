/**
 * @file cltool_cbor.cpp
 * @brief cltool over CBOR messaging. See cltool_cbor.h.
 *
 * @author Kyle Mallory on 2026-10-23
 * @copyright Copyright (c) 2026 Inertial Sense, Inc. All rights reserved.
 */

#include <iostream>

#include "cltool.h"
#include "cltool_cbor.h"
#include "ISCborFormat.h"
#include "ISDataMappings.h"

using namespace std;

namespace {

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

    bool collectAll = req.dst && cborDevFilterIsWildcard(*req.dst);
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
