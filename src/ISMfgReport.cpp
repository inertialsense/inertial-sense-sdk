/**
 * @file ISMfgReport.cpp
 * @brief Manufacturing test results: record builder and background reporter. See ISMfgReport.h.
 *
 * @copyright Copyright (c) 2025 Inertial Sense, Inc. All rights reserved.
 */

#include "ISMfgReport.h"

#include <algorithm>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <sstream>

#include "ISDeviceCal.h"
#include "ISLogger.h"
#include "util/util.h"

namespace fs = std::filesystem;
using json = nlohmann::json;

static const char* statusToString(eMfgStatus s)
{
    switch (s)
    {
    case MFG_STATUS_PASS:   return "pass";
    case MFG_STATUS_FAIL:   return "fail";
    case MFG_STATUS_SKIP:   return "skip";
    default:                return "info";
    }
}

sMfgTestRecord ISMfgReport::newRecord(const dev_info_t& devInfo, const manufacturing_info_t& manfInfo, const std::string& testType)
{
    sMfgTestRecord r;
    r.hwType = utils::getHardwareAsString(devInfo);
    r.serialNumber = devInfo.serialNumber;
    r.manfUid = utils::manfUidToString(manfInfo.uid);
    r.timestampMs = (int64_t)std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
    r.uuid = utils::generateUUIDv7((uint64_t)r.timestampMs);
    r.testType = testType;
    return r;
}

std::string ISMfgReport::resultString(const std::vector<sMfgCheck>& checks)
{
    int pass = 0, fail = 0, skip = 0, warn = 0;
    for (const sMfgCheck& c : checks)
    {
        switch (c.status)
        {
        case MFG_STATUS_PASS:   pass++; if (c.warn) warn++; break;
        case MFG_STATUS_FAIL:   fail++; break;
        case MFG_STATUS_SKIP:   skip++; break;
        default: break;
        }
    }

    std::string s = utils::string_format("%d pass, %d fail", pass, fail);
    if (skip)
        s += utils::string_format(", %d skip", skip);
    if (warn)
        s += utils::string_format(", %d warn", warn);
    return s;
}

json ISMfgReport::buildCheckJson(const sMfgCheck& check)
{
    json j = json::object();
    j["status"] = statusToString(check.status);
    if (check.warn && check.status == MFG_STATUS_PASS)
        j["warn"] = true;
    if (!check.unit.empty())
        j["unit"] = check.unit;

    const sMfgLimits& l = check.limits;
    if (l.any())
    {
        json jl = json::object();
        if (l.hasMin)     jl["min"] = l.min;
        if (l.hasMax)     jl["max"] = l.max;
        if (l.hasWarnMin) jl["warnMin"] = l.warnMin;
        if (l.hasWarnMax) jl["warnMax"] = l.warnMax;
        j["limits"] = jl;
    }

    if (!check.result.empty())
        j["result"] = check.result;
    if (!check.note.empty())
        j["note"] = check.note;
    return j;
}

json ISMfgReport::buildTestJson(const sMfgTestRecord& r)
{
    json checks = json::object();
    for (const sMfgCheck& c : r.checks)
        checks[c.name] = buildCheckJson(c);

    json payload = r.extras.is_object() ? r.extras : json::object();
    payload["checks"] = checks;

    json j = json::object();
    j["hwType"] = r.hwType;
    j["devSerialNum"] = r.serialNumber;
    j["manfUid"] = r.manfUid;
    j["uuid"] = r.uuid;
    j["timestamp"] = utils::unixMsToRfc3339(r.timestampMs);
    j["testType"] = r.testType;
    j["result"] = resultString(r.checks);
    if (!r.station.empty())      j["station"] = r.station;
    if (!r.operatorName.empty()) j["operator"] = r.operatorName;
    if (!r.fwVersion.empty())    j["fwVersion"] = r.fwVersion;
    j["payload"] = payload;
    return j;
}

json ISMfgReport::buildRegisterJson(const dev_info_t& devInfo, const manufacturing_info_t& manfInfo)
{
    sCalDeviceIdentity identity = ISDeviceCal::makeIdentity(devInfo, manfInfo, 0, 0);
    if (manufacturing_info_isWritten(&manfInfo))
        return ISDeviceCal::buildDeviceJson(identity);

    json j = json::object();
    j["hwType"] = identity.hwType;
    j["devSerialNum"] = identity.serialNumber;
    j["manfUid"] = utils::manfUidToString(manfInfo.uid);
    return j;
}

ISHttpRequest::Response ISMfgReport::post(const std::string& restBaseUrl, const json& testJson)
{
    std::vector<ISHttpRequest::MultipartField> fields;
    fields.push_back({ "data", "application/json", testJson.dump() });
    return ISHttpRequest::putMultipart(restBaseUrl + "/api/test", fields);
}


//  ISMfgReporter

ISMfgReporter::ISMfgReporter(const std::string& spoolDir) : m_spoolDir(spoolDir)
{
    std::error_code ec;
    fs::create_directories(fs::path(m_spoolDir) / "failed", ec);
}

ISMfgReporter::~ISMfgReporter()
{
    stop();
}

void ISMfgReporter::setServer(const std::string& restBaseUrl)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    m_baseUrl = restBaseUrl;
    m_kick = true;
    m_wake.notify_all();
}

void ISMfgReporter::setCallback(Callback cb)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    m_callback = cb;
}

void ISMfgReporter::setRetryDelays(double firstSeconds, double maxSeconds)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    m_retryFirst = firstSeconds;
    m_retryMax = maxSeconds;
}

void ISMfgReporter::start()
{
    if (m_thread.joinable())
        return;
    m_stop = false;
    m_kick = true;      // send whatever an earlier run left in the spool
    m_thread = std::thread(&ISMfgReporter::run, this);
}

void ISMfgReporter::stop()
{
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_stop = true;
        m_wake.notify_all();
    }
    if (m_thread.joinable())
        m_thread.join();
}

std::string ISMfgReporter::spoolPath(const std::string& name) const
{
    return (fs::path(m_spoolDir) / name).string();
}

bool ISMfgReporter::enqueue(const json& testJson, const json& registerJson)
{
    if (!testJson.contains("uuid") || !testJson["uuid"].is_string())
        return false;

    json wrapper = { {"test", testJson}, {"register", registerJson} };
    std::string name = testJson["uuid"].get<std::string>() + ".json";
    std::string tmp = spoolPath(name + ".tmp");
    {
        std::ofstream f(tmp, std::ios::binary | std::ios::trunc);
        if (!f)
            return false;
        f << wrapper.dump();
        f.flush();
        if (!f)
            return false;
    }
    std::error_code ec;
    fs::rename(tmp, spoolPath(name), ec);       // atomic: a reader never sees half a record
    if (ec)
        return false;

    std::lock_guard<std::mutex> lock(m_mutex);
    m_kick = true;
    m_wake.notify_all();
    return true;
}

std::vector<std::string> ISMfgReporter::pendingFiles() const
{
    std::vector<std::string> files;
    std::error_code ec;
    for (const auto& e : fs::directory_iterator(m_spoolDir, ec))
    {
        if (e.is_regular_file() && e.path().extension() == ".json")
            files.push_back(e.path().filename().string());
    }
    std::sort(files.begin(), files.end());      // UUIDv7 names sort by creation time: oldest first
    return files;
}

size_t ISMfgReporter::pending() const
{
    return pendingFiles().size();
}

void ISMfgReporter::report(const Outcome& outcome)
{
    Callback cb;
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        cb = m_callback;
    }
    if (cb)
        cb(outcome);
}

void ISMfgReporter::finish(const std::string& file, const Outcome& outcome, bool toFailed)
{
    std::error_code ec;
    if (toFailed)
        fs::rename(spoolPath(file), (fs::path(m_spoolDir) / "failed" / file).string(), ec);
    else
        fs::remove(spoolPath(file), ec);
    m_attempts.erase(outcome.uuid);
    report(outcome);
}

ISMfgReporter::eProcessResult ISMfgReporter::process(const std::string& file)
{
    std::string baseUrl;
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        baseUrl = m_baseUrl;
    }

    Outcome out;
    json wrapper;
    {
        std::ifstream f(spoolPath(file), std::ios::binary);
        std::stringstream ss;
        ss << f.rdbuf();
        wrapper = json::parse(ss.str(), nullptr, false);
    }
    if (!wrapper.is_object() || !wrapper.contains("test") || !wrapper["test"].is_object())
    {
        out.uuid = fs::path(file).stem().string();
        out.statusCode = -1;
        out.message = "unreadable spool file";
        out.permanent = true;
        finish(file, out, true);
        return PROCESS_DONE;
    }

    const json& test = wrapper["test"];
    out.uuid = test.value("uuid", fs::path(file).stem().string());
    out.testType = test.value("testType", "");
    out.attempts = ++m_attempts[out.uuid];

    ISHttpRequest::Response resp = ISMfgReport::post(baseUrl, test);
    if (resp.statusCode == 404 && wrapper.contains("register") && wrapper["register"].is_object())
    {   // The database does not know the device: record its identity, then send the test once more
        ISHttpRequest::Response reg = ISHttpRequest::put(baseUrl + "/api/device", wrapper["register"].dump());
        log_info(IS_LOG_ISDEVICE, "Mfg report %s: device unknown, registered by identity: HTTP %d", out.uuid.c_str(), reg.statusCode);
        if (reg.statusCode == 200 || reg.statusCode == 201)
            resp = ISMfgReport::post(baseUrl, test);
        else
            resp = reg;
    }

    out.statusCode = resp.statusCode;
    out.message = resp.statusMessage;
    if (resp.statusCode == 200 || resp.statusCode == 201 || resp.statusCode == 409)
    {
        out.stored = true;
        out.duplicate = (resp.statusCode == 409);
        finish(file, out, false);
        return PROCESS_DONE;
    }

    bool clientError = resp.statusCode >= 400 && resp.statusCode < 500 && resp.statusCode != 408 && resp.statusCode != 429;
    if (clientError)
    {
        out.permanent = true;
        finish(file, out, true);
        return PROCESS_DONE;
    }

    // Server trouble or no connection: keep the file and try again later. Only the first failure of a record is reported.
    log_warn(IS_LOG_ISDEVICE, "Mfg report %s: HTTP %d, will retry", out.uuid.c_str(), resp.statusCode);
    if (out.attempts == 1)
        report(out);
    return PROCESS_TRANSIENT;
}

void ISMfgReporter::run()
{
    double delay = 0;
    while (!m_stop)
    {
        bool anyTransient = false;
        std::string baseUrl;
        {
            std::lock_guard<std::mutex> lock(m_mutex);
            baseUrl = m_baseUrl;
        }

        if (!baseUrl.empty())
        {
            for (const std::string& file : pendingFiles())
            {
                if (m_stop)
                    return;
                if (process(file) == PROCESS_TRANSIENT)
                {   // keep order: do not send newer records past one that is stuck
                    anyTransient = true;
                    break;
                }
            }
        }

        std::unique_lock<std::mutex> lock(m_mutex);
        if (anyTransient)
        {
            delay = (delay == 0) ? m_retryFirst : std::min(delay * 2, m_retryMax);
            m_wake.wait_for(lock, std::chrono::duration<double>(delay), [this] { return m_stop.load(); });
        }
        else
        {
            delay = 0;
            m_wake.wait(lock, [this] { return m_stop.load() || m_kick; });
            m_kick = false;
        }
    }
}
