/**
 * @file ISMfgReport.h
 * @brief Manufacturing test results: the check and record structures, the builder for the Manufacturing Database's
 * `PUT /api/test` body, and a background reporter that spools records to disk and makes sure they reach the database.
 *
 * Tests describe what they measured as sMfgCheck entries while they run; none of this touches a user interface. The
 * reporter is independent of the code that produced a record: a producer hands it a finished record and moves on, and the
 * reporter retries until the record is in the database, reporting each outcome through an optional callback.
 *
 * @copyright Copyright (c) 2025 Inertial Sense, Inc. All rights reserved.
 */

#ifndef IS_SDK_ISMFGREPORT_H
#define IS_SDK_ISMFGREPORT_H

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <map>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "json.hpp"
#include "data_sets.h"
#include "ISHttpRequest.h"

/** The limits a check was judged against. `min` and `max` are the fail limits; `warnMin` and `warnMax` bound a pass that is flagged as a warning. */
struct sMfgLimits
{
    bool    hasMin = false, hasMax = false, hasWarnMin = false, hasWarnMax = false;
    double  min = 0, max = 0, warnMin = 0, warnMax = 0;

    sMfgLimits& setMin(double v)     { min = v; hasMin = true; return *this; }
    sMfgLimits& setMax(double v)     { max = v; hasMax = true; return *this; }
    sMfgLimits& setWarnMin(double v) { warnMin = v; hasWarnMin = true; return *this; }
    sMfgLimits& setWarnMax(double v) { warnMax = v; hasWarnMax = true; return *this; }
    bool any() const { return hasMin || hasMax || hasWarnMin || hasWarnMax; }
};

/**
 * A check's verdict. FAIL covers any check that was attempted and either failed or could not be measured. SKIP is only for
 * a check that a rule excluded before it was attempted (for example the GNSS checks when no GNSS is configured).
 */
enum eMfgStatus
{
    MFG_STATUS_PASS = 0,
    MFG_STATUS_FAIL,
    MFG_STATUS_SKIP,
    MFG_STATUS_INFO,    //!< measured, no verdict
};

/** One measurement of a test run. Names are global across suites: one name means one quantity in one unit. */
struct sMfgCheck
{
    std::string     name;                       //!< letters, digits, '_' and '.', starting with a letter (e.g. "gnss1.cno")
    eMfgStatus      status = MFG_STATUS_INFO;
    bool            warn = false;               //!< a marginal pass; only sent with MFG_STATUS_PASS
    std::string     unit;
    sMfgLimits      limits;
    std::string     note;                       //!< why it failed, was flagged, or was skipped
    nlohmann::json  result = nlohmann::json::object();      //!< representations: "value", "stats", "axes", or "x." keys

    sMfgCheck() {}
    sMfgCheck(const std::string& n, eMfgStatus s) : name(n), status(s) {}

    /** Convenience setters, chainable */
    sMfgCheck& setValue(const nlohmann::json& v)                { result["value"] = v; return *this; }
    sMfgCheck& setUnit(const std::string& u)                    { unit = u; return *this; }
    sMfgCheck& setNote(const std::string& n)                    { note = n; return *this; }
    sMfgCheck& setWarn(bool w = true)                           { warn = w; return *this; }
    sMfgCheck& setLimits(const sMfgLimits& l)                   { limits = l; return *this; }
    sMfgCheck& setStats(double mean, uint32_t n)                { result["stats"] = { {"mean", mean}, {"n", n} }; return *this; }
    sMfgCheck& setAxes(double x, double y, double z)            { result["axes"] = { {"x", x}, {"y", y}, {"z", z} }; return *this; }
    sMfgCheck& setExtra(const std::string& key, const nlohmann::json& v) { result["x." + key] = v; return *this; }
};

/** One run of a test suite on one device. */
struct sMfgTestRecord
{
    std::string             hwType;             //!< e.g. "IMX-6.0"
    uint32_t                serialNumber = 0;
    std::string             manfUid;            //!< 32 lowercase hex digits
    std::string             uuid;               //!< UUIDv7, assigned when the record is created and kept through any retry
    int64_t                 timestampMs = 0;    //!< when the run happened, ms since the Unix epoch (UTC)
    std::string             testType;           //!< "family.name", e.g. "imx.comprehensive"
    std::string             station;            //!< where the run happened
    std::string             operatorName;       //!< who ran it
    std::string             fwVersion;          //!< firmware the device was running
    std::vector<sMfgCheck>  checks;
    nlohmann::json          extras = nlohmann::json::object();      //!< other free-form payload keys
};

class ISMfgReport
{
public:
    /**
     * Starts a record for a device: its identity, a new UUIDv7 and the current time.
     * @param devInfo the device's DID_DEV_INFO
     * @param manfInfo the device's DID_MANUFACTURING_INFO (only the UID is used)
     * @param testType the suite name, "family.name"
     */
    static sMfgTestRecord newRecord(const dev_info_t& devInfo, const manufacturing_info_t& manfInfo, const std::string& testType);

    /** The run's outcome text, e.g. "5 pass, 1 fail, 1 skip, 2 warn" (a warning is also a pass). */
    static std::string resultString(const std::vector<sMfgCheck>& checks);

    /** The body of `PUT /api/test`'s data part (the draft Client API, section 4). */
    static nlohmann::json buildTestJson(const sMfgTestRecord& record);

    /** The JSON of one check. A `warn` flag on anything other than a pass is dropped, as the server rejects it. */
    static nlohmann::json buildCheckJson(const sMfgCheck& check);

    /**
     * The body of `PUT /api/device` that records a device's identity (hwType, devSerialNum, manfUid), plus the manufacturing
     * fields when the OTP has been written. Unlike ISDevice::RegisterManufacturingInfoToDb() it works for a unit whose OTP
     * is not written, so a failed unit is still recorded.
     */
    static nlohmann::json buildRegisterJson(const dev_info_t& devInfo, const manufacturing_info_t& manfInfo);

    /** `PUT /api/test` for one record. Returns the server's response (200 stored, 404 unknown device, 409 already stored, ...). */
    static ISHttpRequest::Response post(const std::string& restBaseUrl, const nlohmann::json& testJson);
};

/**
 * Posts test records to the Manufacturing Database in the background. A producer calls enqueue() with a finished record
 * and carries on; the reporter does the rest on its own thread:
 *  - every record is written to the spool directory first, so a crash or an outage loses nothing, and any records left from
 *    an earlier run are sent when the reporter starts;
 *  - 200 and 409 (already stored) finish a record;
 *  - 404 (the device is unknown) registers the device from the record's identity and tries once more;
 *  - 400, 413 and other client errors are permanent: the record moves to the "failed" subdirectory and is not retried;
 *  - 5xx and connection errors are retried later, with a growing delay.
 * Records go out oldest first. Nothing here blocks or changes the producer.
 */
class ISMfgReporter
{
public:
    /** What happened to one record. */
    struct Outcome
    {
        std::string uuid;
        std::string testType;
        int         statusCode = 0;         //!< the server's status, or -1 for no response
        std::string message;
        bool        stored = false;         //!< the record is in the database (including 409, already there)
        bool        duplicate = false;      //!< stored because the server already had it (409)
        bool        permanent = false;      //!< failed for good: moved to the "failed" directory
        int         attempts = 0;
    };
    using Callback = std::function<void(const Outcome&)>;

    /** @param spoolDir directory for pending records; created if needed */
    explicit ISMfgReporter(const std::string& spoolDir);
    ~ISMfgReporter();

    /** The database to post to. While it is empty nothing is posted, and records wait in the spool. */
    void setServer(const std::string& restBaseUrl);

    /**
     * Receives every outcome that ends a record (stored or permanent failure) and the first transient failure of each
     * record. Called on the reporter's thread.
     */
    void setCallback(Callback cb);

    /** Starts the thread; records already in the spool are sent. */
    void start();

    /** Stops the thread after the record in flight; records still pending stay in the spool. */
    void stop();

    /**
     * Spools a record and wakes the reporter.
     * @param testJson the record, from ISMfgReport::buildTestJson() (its `uuid` names the spool file)
     * @param registerJson the body of `PUT /api/device` used if the database does not know the device (identity at least)
     * @return false if the record could not be written to the spool
     */
    bool enqueue(const nlohmann::json& testJson, const nlohmann::json& registerJson);

    /** Number of records waiting in the spool. */
    size_t pending() const;

    /** Seconds before the first retry, doubling to maxRetrySeconds. Tests lower these. */
    void setRetryDelays(double firstSeconds, double maxSeconds);

private:
    enum eProcessResult { PROCESS_DONE, PROCESS_TRANSIENT };

    std::string spoolPath(const std::string& name) const;
    std::vector<std::string> pendingFiles() const;
    eProcessResult process(const std::string& file);
    void finish(const std::string& file, const Outcome& outcome, bool toFailed);
    void report(const Outcome& outcome);
    void run();

    std::string                     m_spoolDir;
    mutable std::mutex              m_mutex;
    std::condition_variable         m_wake;
    std::string                     m_baseUrl;
    Callback                        m_callback;
    std::thread                     m_thread;
    std::atomic<bool>               m_stop{false};
    bool                            m_kick = false;
    std::map<std::string, int>      m_attempts;         //!< by record uuid
    double                          m_retryFirst = 5.0, m_retryMax = 300.0;
};

#endif // IS_SDK_ISMFGREPORT_H
