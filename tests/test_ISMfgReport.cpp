/**
 * @file test_ISMfgReport.cpp
 * @brief Tests for the manufacturing test-result builder and background reporter (ISMfgReport.h).
 *
 * @copyright Copyright (c) 2025 Inertial Sense, Inc. All rights reserved.
 */

#include <chrono>
#include <deque>
#include <filesystem>
#include <map>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <gtest/gtest.h>

#include "ISMfgReport.h"
#include "data_sets.h"
#include "json.hpp"
#include "httplib.h"
#include "util/util.h"

using json = nlohmann::json;
namespace fs = std::filesystem;

namespace {

// Loopback server (cpp-httplib) that answers each path from a queue of scripted statuses; the last one repeats.
// A path with no script answers 404. Every request is recorded.
class ScriptedServer {
public:
    struct Req { std::string method, path, body; };

    void script(const std::string& path, std::vector<int> statuses) { std::lock_guard<std::mutex> l(m_mutex); m_script[path] = std::deque<int>(statuses.begin(), statuses.end()); }

    bool start()
    {
        m_port = m_server.bind_to_any_port("127.0.0.1");
        if (m_port <= 0) return false;
        auto handler = [this](const httplib::Request& req, httplib::Response& res) {
            std::lock_guard<std::mutex> l(m_mutex);
            std::string body = req.body;
            if (req.is_multipart_form_data() && req.has_file("data"))
                body = req.get_file_value("data").content;
            m_reqs.push_back({ req.method, req.path, body });
            auto it = m_script.find(req.path);
            int st = 404;
            if (it != m_script.end() && !it->second.empty()) {
                st = it->second.front();
                if (it->second.size() > 1) it->second.pop_front();
            }
            res.status = st;
            res.set_content("{}", "application/json");
        };
        m_server.Put(".*", handler);
        m_thread = std::thread([this] { m_server.listen_after_bind(); });
        while (!m_server.is_running()) std::this_thread::sleep_for(std::chrono::milliseconds(5));
        return true;
    }
    ~ScriptedServer() { m_server.stop(); if (m_thread.joinable()) m_thread.join(); }

    std::string url() const { return "http://127.0.0.1:" + std::to_string(m_port); }
    std::vector<Req> requests() { std::lock_guard<std::mutex> l(m_mutex); return m_reqs; }
    int count(const std::string& path) { int n = 0; for (auto& r : requests()) if (r.path == path) n++; return n; }

private:
    httplib::Server m_server;
    std::map<std::string, std::deque<int>> m_script;
    std::vector<Req> m_reqs;
    std::mutex m_mutex;
    std::thread m_thread;
    int m_port = 0;
};

dev_info_t makeDev()
{
    dev_info_t d = {};
    d.hardwareType = IS_HARDWARE_TYPE_IMX;
    d.hardwareVer[0] = 6;
    d.serialNumber = 3054153446u;
    return d;
}

manufacturing_info_t makeManf()
{
    manufacturing_info_t m = {};
    m.uid[0] = 0x001D0028u; m.uid[1] = 0x41465006u; m.uid[2] = 0x20363841u;
    return m;
}

sMfgTestRecord makeRecord(const std::string& type = "imx.comprehensive")
{
    sMfgTestRecord r = ISMfgReport::newRecord(makeDev(), makeManf(), type);
    r.station = "bench-1";
    r.operatorName = "kyle";
    r.fwVersion = "3.1.1";
    r.checks.push_back(sMfgCheck("flashConfig", MFG_STATUS_PASS));
    return r;
}

struct test_ISMfgReporter : ::testing::Test {
    fs::path dir;
    void SetUp() override {
        dir = fs::temp_directory_path() / ("mfgreport_" + utils::generateUUIDv4());
        fs::create_directories(dir);
    }
    void TearDown() override { std::error_code ec; fs::remove_all(dir, ec); }

    static bool waitFor(const std::function<bool()>& cond, int ms = 5000)
    {
        for (int i = 0; i < ms / 10; i++) { if (cond()) return true; std::this_thread::sleep_for(std::chrono::milliseconds(10)); }
        return cond();
    }
    size_t failedCount() const
    {
        size_t n = 0;
        std::error_code ec;
        for (auto& e : fs::directory_iterator(dir / "failed", ec)) if (e.is_regular_file()) n++;
        return n;
    }
};

} // namespace

//  Builder

TEST(test_ISMfgReport_builder, recordHasIdentityUuidAndTimestamp)
{
    sMfgTestRecord r = makeRecord();
    json j = ISMfgReport::buildTestJson(r);
    EXPECT_EQ(j["hwType"], "IMX-6.0");
    EXPECT_EQ(j["devSerialNum"], 3054153446u);
    EXPECT_EQ(j["manfUid"], "28001d00065046414138362000000000");
    EXPECT_EQ(j["testType"], "imx.comprehensive");
    EXPECT_EQ(j["station"], "bench-1");
    EXPECT_EQ(j["operator"], "kyle");
    EXPECT_EQ(j["fwVersion"], "3.1.1");
    EXPECT_EQ(j["uuid"].get<std::string>().size(), 36u);
    EXPECT_EQ(j["uuid"].get<std::string>()[14], '7');       // UUIDv7
    EXPECT_EQ(j["timestamp"], utils::unixMsToRfc3339(r.timestampMs));
    EXPECT_EQ(j["payload"]["checks"]["flashConfig"]["status"], "pass");
}

TEST(test_ISMfgReport_builder, checkRepresentationsLimitsAndNote)
{
    sMfgLimits lim;
    lim.setMin(0.0).setMax(0.055).setWarnMax(0.04);
    sMfgCheck c = sMfgCheck("gyroSigma", MFG_STATUS_PASS).setUnit("deg/s").setLimits(lim).setWarn().setNote("near limit").setAxes(0.01, 0.02, 0.03).setExtra("samples", 100);
    json j = ISMfgReport::buildCheckJson(c);
    EXPECT_EQ(j["status"], "pass");
    EXPECT_TRUE(j["warn"].get<bool>());
    EXPECT_EQ(j["unit"], "deg/s");
    EXPECT_EQ(j["note"], "near limit");
    EXPECT_DOUBLE_EQ(j["limits"]["max"].get<double>(), 0.055);
    EXPECT_DOUBLE_EQ(j["limits"]["warnMax"].get<double>(), 0.04);
    EXPECT_FALSE(j["limits"].contains("warnMin"));
    EXPECT_DOUBLE_EQ(j["result"]["axes"]["z"].get<double>(), 0.03);
    EXPECT_EQ(j["result"]["x.samples"], 100);
}

TEST(test_ISMfgReport_builder, warnIsOnlySentWithPass)
{
    EXPECT_FALSE(ISMfgReport::buildCheckJson(sMfgCheck("a", MFG_STATUS_FAIL).setWarn()).contains("warn"));
    EXPECT_FALSE(ISMfgReport::buildCheckJson(sMfgCheck("a", MFG_STATUS_SKIP).setWarn()).contains("warn"));
    EXPECT_FALSE(ISMfgReport::buildCheckJson(sMfgCheck("a", MFG_STATUS_PASS)).contains("warn"));
}

TEST(test_ISMfgReport_builder, statusMapping)
{
    EXPECT_EQ(ISMfgReport::buildCheckJson(sMfgCheck("a", MFG_STATUS_PASS))["status"], "pass");
    EXPECT_EQ(ISMfgReport::buildCheckJson(sMfgCheck("a", MFG_STATUS_FAIL))["status"], "fail");
    EXPECT_EQ(ISMfgReport::buildCheckJson(sMfgCheck("a", MFG_STATUS_SKIP))["status"], "skip");
    EXPECT_EQ(ISMfgReport::buildCheckJson(sMfgCheck("a", MFG_STATUS_INFO))["status"], "info");
}

TEST(test_ISMfgReport_builder, resultString)
{
    std::vector<sMfgCheck> c;
    EXPECT_EQ(ISMfgReport::resultString(c), "0 pass, 0 fail");
    c.push_back(sMfgCheck("a", MFG_STATUS_PASS));
    c.push_back(sMfgCheck("b", MFG_STATUS_PASS).setWarn());
    c.push_back(sMfgCheck("c", MFG_STATUS_FAIL));
    EXPECT_EQ(ISMfgReport::resultString(c), "2 pass, 1 fail, 1 warn");
    c.push_back(sMfgCheck("d", MFG_STATUS_SKIP));
    c.push_back(sMfgCheck("e", MFG_STATUS_INFO));
    EXPECT_EQ(ISMfgReport::resultString(c), "2 pass, 1 fail, 1 skip, 1 warn");
}

TEST(test_ISMfgReport_builder, registerJsonForUnwrittenOtpIsIdentityOnly)
{
    manufacturing_info_t m = makeManf();       // key == 0: OTP never written
    json j = ISMfgReport::buildRegisterJson(makeDev(), m);
    EXPECT_EQ(j.size(), 3u);
    EXPECT_EQ(j["hwType"], "IMX-6.0");
    EXPECT_EQ(j["devSerialNum"], 3054153446u);
    EXPECT_EQ(j["manfUid"], "28001d00065046414138362000000000");
}

//  Reporter

TEST_F(test_ISMfgReporter, storedRecordIsPostedAndSpoolEmptied)
{
    ScriptedServer srv;
    srv.script("/api/test", { 200 });
    ASSERT_TRUE(srv.start());

    ISMfgReporter rep(dir.string());
    std::mutex m; std::vector<ISMfgReporter::Outcome> outs;
    rep.setCallback([&](const ISMfgReporter::Outcome& o) { std::lock_guard<std::mutex> l(m); outs.push_back(o); });
    rep.setServer(srv.url());
    rep.start();

    sMfgTestRecord r = makeRecord();
    ASSERT_TRUE(rep.enqueue(ISMfgReport::buildTestJson(r), ISMfgReport::buildRegisterJson(makeDev(), makeManf())));
    ASSERT_TRUE(waitFor([&] { std::lock_guard<std::mutex> l(m); return !outs.empty(); }));
    rep.stop();

    EXPECT_TRUE(outs[0].stored);
    EXPECT_EQ(outs[0].uuid, r.uuid);
    EXPECT_EQ(rep.pending(), 0u);
    auto reqs = srv.requests();
    ASSERT_EQ(reqs.size(), 1u);
    EXPECT_EQ(json::parse(reqs[0].body)["uuid"], r.uuid);
}

TEST_F(test_ISMfgReporter, duplicateIsTreatedAsStored)
{
    ScriptedServer srv;
    srv.script("/api/test", { 409 });
    ASSERT_TRUE(srv.start());

    ISMfgReporter rep(dir.string());
    std::mutex m; std::vector<ISMfgReporter::Outcome> outs;
    rep.setCallback([&](const ISMfgReporter::Outcome& o) { std::lock_guard<std::mutex> l(m); outs.push_back(o); });
    rep.setServer(srv.url());
    rep.start();
    ASSERT_TRUE(rep.enqueue(ISMfgReport::buildTestJson(makeRecord()), json::object()));
    ASSERT_TRUE(waitFor([&] { std::lock_guard<std::mutex> l(m); return !outs.empty(); }));
    rep.stop();

    EXPECT_TRUE(outs[0].stored);
    EXPECT_TRUE(outs[0].duplicate);
    EXPECT_EQ(rep.pending(), 0u);
    EXPECT_EQ(failedCount(), 0u);
}

TEST_F(test_ISMfgReporter, unknownDeviceIsRegisteredByIdentityThenRetriedOnce)
{
    ScriptedServer srv;
    srv.script("/api/test", { 404, 200 });
    srv.script("/api/device", { 201 });
    ASSERT_TRUE(srv.start());

    ISMfgReporter rep(dir.string());
    std::mutex m; std::vector<ISMfgReporter::Outcome> outs;
    rep.setCallback([&](const ISMfgReporter::Outcome& o) { std::lock_guard<std::mutex> l(m); outs.push_back(o); });
    rep.setServer(srv.url());
    rep.start();
    ASSERT_TRUE(rep.enqueue(ISMfgReport::buildTestJson(makeRecord()), ISMfgReport::buildRegisterJson(makeDev(), makeManf())));
    ASSERT_TRUE(waitFor([&] { std::lock_guard<std::mutex> l(m); return !outs.empty(); }));
    rep.stop();

    EXPECT_TRUE(outs[0].stored);
    EXPECT_EQ(srv.count("/api/test"), 2);
    EXPECT_EQ(srv.count("/api/device"), 1);
    // the registration carried the identity of the unit
    for (auto& r : srv.requests())
        if (r.path == "/api/device")
            EXPECT_EQ(json::parse(r.body)["manfUid"], "28001d00065046414138362000000000");
}

TEST_F(test_ISMfgReporter, badRequestAndTooLargeMoveToFailedAndAreNotRetried)
{
    for (int status : { 400, 413 })
    {
        fs::remove_all(dir / "failed");
        ScriptedServer srv;
        srv.script("/api/test", { status });
        ASSERT_TRUE(srv.start());

        ISMfgReporter rep(dir.string());
        std::mutex m; std::vector<ISMfgReporter::Outcome> outs;
        rep.setCallback([&](const ISMfgReporter::Outcome& o) { std::lock_guard<std::mutex> l(m); outs.push_back(o); });
        rep.setServer(srv.url());
        rep.start();
        ASSERT_TRUE(rep.enqueue(ISMfgReport::buildTestJson(makeRecord()), json::object()));
        ASSERT_TRUE(waitFor([&] { std::lock_guard<std::mutex> l(m); return !outs.empty(); })) << status;
        rep.stop();

        EXPECT_TRUE(outs[0].permanent) << status;
        EXPECT_FALSE(outs[0].stored) << status;
        EXPECT_EQ(outs[0].statusCode, status);
        EXPECT_EQ(rep.pending(), 0u) << status;
        EXPECT_EQ(failedCount(), 1u) << status;
        EXPECT_EQ(srv.count("/api/test"), 1) << status;
    }
}

TEST_F(test_ISMfgReporter, serverErrorIsRetriedUntilStoredAndFirstFailureReportedOnce)
{
    ScriptedServer srv;
    srv.script("/api/test", { 503, 503, 200 });
    ASSERT_TRUE(srv.start());

    ISMfgReporter rep(dir.string());
    rep.setRetryDelays(0.02, 0.05);
    std::mutex m; std::vector<ISMfgReporter::Outcome> outs;
    rep.setCallback([&](const ISMfgReporter::Outcome& o) { std::lock_guard<std::mutex> l(m); outs.push_back(o); });
    rep.setServer(srv.url());
    rep.start();
    ASSERT_TRUE(rep.enqueue(ISMfgReport::buildTestJson(makeRecord()), json::object()));
    ASSERT_TRUE(waitFor([&] { std::lock_guard<std::mutex> l(m); return !outs.empty() && outs.back().stored; }));
    rep.stop();

    ASSERT_EQ(outs.size(), 2u);                 // one transient report, then the stored one
    EXPECT_FALSE(outs[0].stored);
    EXPECT_FALSE(outs[0].permanent);
    EXPECT_EQ(outs[0].statusCode, 503);
    EXPECT_TRUE(outs[1].stored);
    EXPECT_EQ(outs[1].attempts, 3);
    EXPECT_EQ(srv.count("/api/test"), 3);
}

TEST_F(test_ISMfgReporter, unreachableServerKeepsRecordAndSpoolSurvivesRestart)
{
    std::string uuid;
    {   // nothing is listening on this port
        ISMfgReporter rep(dir.string());
        rep.setRetryDelays(0.02, 0.05);
        std::mutex m; std::vector<ISMfgReporter::Outcome> outs;
        rep.setCallback([&](const ISMfgReporter::Outcome& o) { std::lock_guard<std::mutex> l(m); outs.push_back(o); });
        rep.setServer("http://127.0.0.1:1");
        rep.start();
        sMfgTestRecord r = makeRecord();
        uuid = r.uuid;
        ASSERT_TRUE(rep.enqueue(ISMfgReport::buildTestJson(r), json::object()));
        ASSERT_TRUE(waitFor([&] { std::lock_guard<std::mutex> l(m); return !outs.empty(); }));
        rep.stop();
        EXPECT_EQ(outs[0].statusCode, -1);
        EXPECT_EQ(rep.pending(), 1u);
    }

    ScriptedServer srv;
    srv.script("/api/test", { 200 });
    ASSERT_TRUE(srv.start());
    ISMfgReporter rep2(dir.string());            // a new run finds the record in the spool
    std::mutex m; std::vector<ISMfgReporter::Outcome> outs;
    rep2.setCallback([&](const ISMfgReporter::Outcome& o) { std::lock_guard<std::mutex> l(m); outs.push_back(o); });
    rep2.setServer(srv.url());
    rep2.start();
    ASSERT_TRUE(waitFor([&] { std::lock_guard<std::mutex> l(m); return !outs.empty(); }));
    rep2.stop();
    EXPECT_TRUE(outs[0].stored);
    EXPECT_EQ(outs[0].uuid, uuid);
    EXPECT_EQ(rep2.pending(), 0u);
}

TEST_F(test_ISMfgReporter, recordsWaitInSpoolWhileNoServerIsSet)
{
    ISMfgReporter rep(dir.string());
    rep.start();
    ASSERT_TRUE(rep.enqueue(ISMfgReport::buildTestJson(makeRecord()), json::object()));
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    EXPECT_EQ(rep.pending(), 1u);

    ScriptedServer srv;
    srv.script("/api/test", { 200 });
    ASSERT_TRUE(srv.start());
    rep.setServer(srv.url());
    ASSERT_TRUE(waitFor([&] { return rep.pending() == 0; }));
    rep.stop();
}

TEST_F(test_ISMfgReporter, recordsAreSentOldestFirst)
{
    ScriptedServer srv;
    srv.script("/api/test", { 200 });
    ASSERT_TRUE(srv.start());

    ISMfgReporter rep(dir.string());
    std::vector<std::string> uuids;
    for (int i = 0; i < 3; i++)
    {
        sMfgTestRecord r = makeRecord();
        r.timestampMs += i * 10;                        // distinct, increasing ms in the UUIDv7
        r.uuid = utils::generateUUIDv7((uint64_t)r.timestampMs);
        uuids.push_back(r.uuid);
        ASSERT_TRUE(rep.enqueue(ISMfgReport::buildTestJson(r), json::object()));
    }
    rep.setServer(srv.url());
    rep.start();
    ASSERT_TRUE(waitFor([&] { return rep.pending() == 0; }));
    rep.stop();

    auto reqs = srv.requests();
    ASSERT_EQ(reqs.size(), 3u);
    for (int i = 0; i < 3; i++)
        EXPECT_EQ(json::parse(reqs[i].body)["uuid"], uuids[i]);
}

TEST_F(test_ISMfgReporter, unreadableSpoolFileMovesToFailed)
{
    { std::ofstream f(dir / "garbage.json"); f << "not json"; }
    ScriptedServer srv;
    ASSERT_TRUE(srv.start());
    ISMfgReporter rep(dir.string());
    rep.setServer(srv.url());
    rep.start();
    ASSERT_TRUE(waitFor([&] { return rep.pending() == 0; }));
    rep.stop();
    EXPECT_EQ(failedCount(), 1u);
}
