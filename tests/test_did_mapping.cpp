/**
 * @file test_did_mapping.cpp
 * @brief SN-8328 category H — DID data-mapping / field extraction.
 *
 * The thinnest existing coverage and the likeliest home of "strange results":
 * these pin down that (H1) a DID's field map matches its C struct
 * (offset/size/type), and (H2) values written into a payload extract back
 * correctly through the same map-driven path a consumer (Logalyzer's
 * RawSeriesBuilder) uses — reading the payload bytes from a record read back
 * via fromSegments off the writer's own .idx.
 */
#include <gtest/gtest.h>

#include "com_manager.h"  // first — see test_log_reader.cpp comment

#include "DeviceLog.h"
#include "ISDataMappings.h"
#include "ISDeviceLog.h"
#include "ISFileManager.h"
#include "ISLogger.h"
#include "data_sets.h"

#include "SimulatedDevice.h"  // testsim::frame()

#include <cstddef>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <string>
#include <vector>

using namespace inertial_sense;
namespace fs = std::filesystem;

namespace {

constexpr uint16_t kHwId   = ENCODE_HDW_ID(IS_HARDWARE_TYPE_IMX, 5, 0);
constexpr uint32_t kSerial = 777001u;

//! Portable unique temp directory (POSIX + Windows CI).
fs::path makeTempDir(const std::string& prefix) {
    static unsigned counter = 0;
    return fs::temp_directory_path() / (prefix + "_" + std::to_string(counter++));
}

//! Extract a scalar (or array element) as a double via the DID field map —
//! the consumer path. Array fields are stored under the base name with a
//! per-element stride (elementSize); element i lives at offset + i*stride.
double extractScalar(const map_name_to_info_t& m, const char* field,
                     const uint8_t* payload, bool& ok, unsigned arrayIndex = 0) {
    ok = false;
    auto it = m.find(field);
    if (it == m.end()) return 0.0;
    const data_info_t& info = it->second;
    const uint32_t stride = info.elementSize ? info.elementSize : info.size;
    const uint8_t* p = payload + info.offset + arrayIndex * stride;
    double v = 0.0;
    switch (info.type) {
        case DATA_TYPE_F64:    { double d;   std::memcpy(&d, p, 8); v = d; ok = true; break; }
        case DATA_TYPE_F32:    { float f;    std::memcpy(&f, p, 4); v = f; ok = true; break; }
        case DATA_TYPE_UINT32: { uint32_t u; std::memcpy(&u, p, 4); v = u; ok = true; break; }
        case DATA_TYPE_INT32:  { int32_t s;  std::memcpy(&s, p, 4); v = s; ok = true; break; }
        default: break;
    }
    return v;
}

//! Write a single INS_2 record through cISLogger; return its .raw path.
fs::path writeOneIns2(const ins_2_t& s) {
    const fs::path dir = makeTempDir("test_didmap");
    ISFileManager::DeleteDirectory(dir.string());

    cISLogger logger;
    cISLogger::sSaveOptions opts;
    opts.logType               = cISLogger::LOGTYPE_RAW;
    opts.useSubFolderTimestamp = false;
    if (!logger.InitSave(dir.string(), opts)) return {};
    auto dl = logger.registerDevice(kHwId, kSerial);
    if (!dl) return {};
    logger.EnableLogging(true);
    const testsim::SimPacket p = testsim::frame(DID_INS_2, &s, sizeof(s));
    logger.LogData(dl, static_cast<int>(p.framed.size()), p.framed.data());
    logger.CloseAllFiles();

    std::vector<ISFileManager::file_info_t> raws;
    ISFileManager::GetAllFilesInDirectory(dir.string(), true, "\\.raw$", raws);
    return raws.empty() ? fs::path{} : fs::path(raws.front().name);
}

// H1: the DID_INS_2 field map must agree with the C struct — offset and type
// for a couple of well-known fields. A drift here is exactly what produces
// wrong extracted values downstream.
TEST(DidMappingTest, Ins2FieldMapMatchesStruct) {
    const map_name_to_info_t* m = cISDataMappings::NameToInfoMap(DID_INS_2);
    ASSERT_NE(m, nullptr) << "DID_INS_2 must have a field map";
    ASSERT_FALSE(m->empty());

    ASSERT_TRUE(m->count("timeOfWeek")) << "expected field 'timeOfWeek'";
    const data_info_t& tow = m->at("timeOfWeek");
    EXPECT_EQ(tow.offset, static_cast<uint32_t>(offsetof(ins_2_t, timeOfWeek)));
    EXPECT_EQ(tow.type, DATA_TYPE_F64);

    ASSERT_TRUE(m->count("week")) << "expected field 'week'";
    const data_info_t& wk = m->at("week");
    EXPECT_EQ(wk.offset, static_cast<uint32_t>(offsetof(ins_2_t, week)));
    EXPECT_EQ(wk.type, DATA_TYPE_UINT32);
}

// H2: values written into a payload must extract back correctly via the map,
// reading the record's payload bytes off the writer's .idx through fromSegments
// (the first test that dereferences payload bytes from a writer-idx record).
TEST(DidMappingTest, ExtractedValuesMatchWritten) {
    ins_2_t s{};
    s.week       = 2300;
    s.timeOfWeek = 123456.789;
    s.qn2b[0]    = 1.0f;
    s.lla[0]     = 40.25;
    s.lla[1]     = -111.75;
    s.lla[2]     = 1500.0;

    const fs::path raw = writeOneIns2(s);
    ASSERT_FALSE(raw.empty());

    auto log = ISDeviceLog::fromSegments({ raw });
    ASSERT_TRUE(log.has_value());
    const map_name_to_info_t* m = cISDataMappings::NameToInfoMap(DID_INS_2);
    ASSERT_NE(m, nullptr);

    std::size_t seen = 0;
    for (auto rv : log.value().records(DID_INS_2)) {
        auto [ptr, size] = rv.bytes();
        ASSERT_NE(ptr, nullptr) << "record bytes must be addressable";
        // ISRecordView::bytes() is the full framed on-disk record (framing +
        // header + payload + checksum), NOT the payload start — mirror the
        // consumer (RawSeriesBuilder) and re-parse to find the payload before
        // applying map offsets.
        EXPECT_GT(size, sizeof(ins_2_t)) << "framed record is larger than the payload";

        is_comm_instance_t comm{};
        uint8_t commBuf[1024];
        is_comm_init(&comm, commBuf, sizeof(commBuf), nullptr);
        const uint8_t* payload = nullptr;
        uint32_t parsedDid = 0;
        for (std::size_t i = 0; i < size; ++i) {
            const protocol_type_t pt = is_comm_parse_byte(&comm, ptr[i]);
            if (pt == _PTYPE_INERTIAL_SENSE_DATA || pt == _PTYPE_INERTIAL_SENSE_CMD) {
                payload   = comm.rxPkt.data.ptr;
                parsedDid = comm.rxPkt.dataHdr.id;
                break;
            }
        }
        ASSERT_NE(payload, nullptr) << "record must parse back to an ISB payload";
        EXPECT_EQ(parsedDid, static_cast<uint32_t>(DID_INS_2));

        bool ok = false;
        const double tow = extractScalar(*m, "timeOfWeek", payload, ok);
        EXPECT_TRUE(ok);
        EXPECT_DOUBLE_EQ(tow, 123456.789) << "F64 field extracted via the DID map";

        const double wk = extractScalar(*m, "week", payload, ok);
        EXPECT_TRUE(ok);
        EXPECT_DOUBLE_EQ(wk, 2300.0) << "UINT32 field extracted via the DID map";

        // Array / vec element addressing: base name + per-element stride.
        const double lat = extractScalar(*m, "lla", payload, ok, 0);
        EXPECT_TRUE(ok); EXPECT_DOUBLE_EQ(lat, 40.25)   << "lla[0] (F64 array element)";
        const double lon = extractScalar(*m, "lla", payload, ok, 1);
        EXPECT_TRUE(ok); EXPECT_DOUBLE_EQ(lon, -111.75) << "lla[1]";
        const double alt = extractScalar(*m, "lla", payload, ok, 2);
        EXPECT_TRUE(ok); EXPECT_DOUBLE_EQ(alt, 1500.0)  << "lla[2]";
        const double q0 = extractScalar(*m, "qn2b", payload, ok, 0);
        EXPECT_TRUE(ok); EXPECT_DOUBLE_EQ(q0, 1.0)      << "qn2b[0] (F32 array element)";
        ++seen;
    }
    EXPECT_EQ(seen, 1u) << "the one written INS_2 record must be found";

    ISFileManager::DeleteDirectory(raw.parent_path().string());
}

}  // namespace
