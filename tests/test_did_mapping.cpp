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
#include <cstdlib>
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

//! Parse a framed on-disk record back to a copy of its ISB payload bytes.
//! ({} if it doesn't parse.) Mirrors the consumer: bytes() is the full framed
//! record, so re-parse to reach the payload.
std::vector<uint8_t> parsePayload(const uint8_t* framed, std::size_t n) {
    is_comm_instance_t comm{};
    uint8_t commBuf[1024];
    is_comm_init(&comm, commBuf, sizeof(commBuf), nullptr);
    for (std::size_t i = 0; i < n; ++i) {
        const protocol_type_t pt = is_comm_parse_byte(&comm, framed[i]);
        if (pt == _PTYPE_INERTIAL_SENSE_DATA || pt == _PTYPE_INERTIAL_SENSE_CMD) {
            const uint8_t* p = comm.rxPkt.data.ptr;
            return std::vector<uint8_t>(p, p + comm.rxPkt.dataHdr.size);
        }
    }
    return {};
}

//! Write a single record of any DID through cISLogger; return its .raw path.
fs::path writeOneRecord(uint32_t did, const void* payload, std::size_t size) {
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
    const testsim::SimPacket p = testsim::frame(did, payload, size);
    logger.LogData(dl, static_cast<int>(p.framed.size()), p.framed.data());
    logger.CloseAllFiles();

    std::vector<ISFileManager::file_info_t> raws;
    ISFileManager::GetAllFilesInDirectory(dir.string(), true, "\\.raw$", raws);
    return raws.empty() ? fs::path{} : fs::path(raws.front().name);
}

fs::path writeOneIns2(const ins_2_t& s) { return writeOneRecord(DID_INS_2, &s, sizeof(s)); }

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

// H (status decode): a status/bitmask field must map as UINT32, extract its raw
// value, and render to a non-empty decoded string via DataToString (the decode
// path EvalTool/Logalyzer use for status columns/tooltips).
TEST(DidMappingTest, StatusFieldExtractsAndDecodes) {
    ins_2_t s{};
    s.week       = 2300;
    s.timeOfWeek = 100.0;
    s.qn2b[0]    = 1.0f;
    s.insStatus  = 0x00012345u;  // arbitrary recognizable bits

    const fs::path raw = writeOneIns2(s);
    ASSERT_FALSE(raw.empty());
    auto log = ISDeviceLog::fromSegments({ raw });
    ASSERT_TRUE(log.has_value());

    const map_name_to_info_t* m = cISDataMappings::NameToInfoMap(DID_INS_2);
    ASSERT_NE(m, nullptr);
    ASSERT_TRUE(m->count("insStatus")) << "expected status field 'insStatus'";
    const data_info_t& info = m->at("insStatus");
    EXPECT_EQ(info.type, DATA_TYPE_UINT32);

    std::size_t seen = 0;
    for (auto rv : log.value().records(DID_INS_2)) {
        auto [ptr, size] = rv.bytes();
        const std::vector<uint8_t> payload = parsePayload(ptr, size);
        ASSERT_EQ(payload.size(), sizeof(ins_2_t));

        bool ok = false;
        const double rawVal = extractScalar(*m, "insStatus", payload.data(), ok);
        EXPECT_TRUE(ok);
        EXPECT_EQ(static_cast<uint32_t>(rawVal), 0x00012345u) << "raw status value round-trips";

        // Decode via the mapping's string renderer (status columns / tooltips).
        p_data_hdr_t hdr{};
        hdr.id     = DID_INS_2;
        hdr.size   = sizeof(ins_2_t);
        hdr.offset = 0;
        data_mapping_string_t str{};
        const bool okStr = cISDataMappings::DataToString(info, &hdr, payload.data(), str);
        EXPECT_TRUE(okStr) << "DataToString must decode the status field";
        EXPECT_GT(std::strlen(str), 0u) << "decoded status string must be non-empty";
        ++seen;
    }
    EXPECT_EQ(seen, 1u);

    ISFileManager::DeleteDirectory(raw.parent_path().string());
}

// SN-8113 regression guard: DID_IMUS_RAW per-IMU fields must be registered in
// ISDataMappings so Logalyzer can build series for IMUS_RAW.I*.acc/pqr. The
// original bug was that the DID had no field map at all (PopulateMapImus not
// wired up). Fields are stored as base-name 3-vectors ("I0.acc", "I0.pqr", ...);
// the consumer (RawSeriesBuilder::splitArrayField) resolves a subscripted name
// like "I2.acc[1]" to base "I2.acc" + element index 1. This locks the mapping
// in — a regression (e.g. the call getting commented out, as happened to
// DID_DIAGNOSTIC_MESSAGE / SN-8127) would fail here.
TEST(DidMappingTest, ImusRawFieldsRegistered_SN8113) {
    EXPECT_EQ(cISDataMappings::NameToDid("DID_IMUS_RAW"),
              static_cast<uint32_t>(DID_IMUS_RAW)) << "NameToDid must resolve DID_IMUS_RAW";

    const map_name_to_info_t* m = cISDataMappings::NameToInfoMap(DID_IMUS_RAW);
    ASSERT_NE(m, nullptr) << "DID_IMUS_RAW must have a field map (SN-8113)";

    for (const char* base : { "I0.acc", "I0.pqr", "I1.acc", "I1.pqr", "I2.acc", "I2.pqr" }) {
        ASSERT_TRUE(m->count(base)) << "missing IMUS_RAW field '" << base << "'";
        const data_info_t& info = m->at(base);
        EXPECT_EQ(info.type, DATA_TYPE_F32) << base;
        EXPECT_EQ(info.arraySize, 3u) << base << " must be a 3-vector";
    }
    // Offsets agree with the imus_t struct (per-IMU stride correct).
    EXPECT_EQ(m->at("I0.acc").offset, static_cast<uint32_t>(offsetof(imus_t, I[0].acc)));
    EXPECT_EQ(m->at("I2.acc").offset, static_cast<uint32_t>(offsetof(imus_t, I[2].acc)));
    EXPECT_EQ(m->at("I2.pqr").offset, static_cast<uint32_t>(offsetof(imus_t, I[2].pqr)));
}

// SN-8113 FUNCTIONAL round-trip: not just "is it registered" but "does the
// parser actually work" — write a DID_IMUS_RAW (imus_t) record with known
// per-IMU/axis values through cISLogger, read it back via fromSegments, and
// extract element values through the field map exactly as RawSeriesBuilder
// would. Asserting the values survive proves the end-to-end parse, not just
// the mapping's presence.
TEST(DidMappingTest, ImusRawParsesAndExtractsValues_SN8113) {
    imus_t s{};
    s.time       = 12.5;
    s.status     = 0x7u;
    s.I[0].acc[0] = 1.0f;  s.I[0].acc[1] = 2.0f;  s.I[0].acc[2] = 3.0f;
    s.I[1].pqr[2] = -0.5f;
    s.I[2].acc[1] = 9.81f;

    const fs::path raw = writeOneRecord(DID_IMUS_RAW, &s, sizeof(s));
    ASSERT_FALSE(raw.empty());
    auto log = ISDeviceLog::fromSegments({ raw });
    ASSERT_TRUE(log.has_value());
    const map_name_to_info_t* m = cISDataMappings::NameToInfoMap(DID_IMUS_RAW);
    ASSERT_NE(m, nullptr);

    std::size_t seen = 0;
    for (auto rv : log.value().records(DID_IMUS_RAW)) {
        auto [ptr, size] = rv.bytes();
        const std::vector<uint8_t> payload = parsePayload(ptr, size);
        ASSERT_EQ(payload.size(), sizeof(imus_t)) << "IMUS_RAW payload round-trips whole";

        bool ok = false;
        EXPECT_FLOAT_EQ(static_cast<float>(extractScalar(*m, "I0.acc", payload.data(), ok, 0)), 1.0f);  EXPECT_TRUE(ok);
        EXPECT_FLOAT_EQ(static_cast<float>(extractScalar(*m, "I0.acc", payload.data(), ok, 1)), 2.0f);  EXPECT_TRUE(ok);
        EXPECT_FLOAT_EQ(static_cast<float>(extractScalar(*m, "I0.acc", payload.data(), ok, 2)), 3.0f);  EXPECT_TRUE(ok);
        EXPECT_FLOAT_EQ(static_cast<float>(extractScalar(*m, "I1.pqr", payload.data(), ok, 2)), -0.5f); EXPECT_TRUE(ok);
        // The field the original bug report cited: DID_IMUS_RAW / I2.acc[1].
        EXPECT_FLOAT_EQ(static_cast<float>(extractScalar(*m, "I2.acc", payload.data(), ok, 1)), 9.81f); EXPECT_TRUE(ok);
        ++seen;
    }
    EXPECT_EQ(seen, 1u) << "the written IMUS_RAW record must be found and parsed";

    ISFileManager::DeleteDirectory(raw.parent_path().string());
}

// H (conversion factor): a field with a unit conversion (DID_IMUS_RAW pqr is
// stored in rad/s, conversion = C_RAD2DEG) must render the converted value when
// useConversion=true and the raw value when false. Guards that the conversion
// factor is both registered and actually applied by DataToString.
TEST(DidMappingTest, ConversionFactorAppliedOnRender) {
    imus_t s{};
    s.I[0].pqr[0] = 1.0f;  // 1 rad/s -> ~57.2958 deg/s after conversion
    const fs::path raw = writeOneRecord(DID_IMUS_RAW, &s, sizeof(s));
    ASSERT_FALSE(raw.empty());
    auto log = ISDeviceLog::fromSegments({ raw });
    ASSERT_TRUE(log.has_value());

    const map_name_to_info_t* m = cISDataMappings::NameToInfoMap(DID_IMUS_RAW);
    ASSERT_NE(m, nullptr);
    ASSERT_TRUE(m->count("I0.pqr"));
    const data_info_t& info = m->at("I0.pqr");
    ASSERT_GT(info.conversion, 1.0) << "pqr must carry a rad->deg conversion (~57.3)";

    std::size_t seen = 0;
    for (auto rv : log.value().records(DID_IMUS_RAW)) {
        auto [ptr, size] = rv.bytes();
        const std::vector<uint8_t> payload = parsePayload(ptr, size);
        ASSERT_EQ(payload.size(), sizeof(imus_t));

        p_data_hdr_t hdr{};
        hdr.id = DID_IMUS_RAW; hdr.size = sizeof(imus_t); hdr.offset = 0;
        data_mapping_string_t withConv{}, noConv{};
        ASSERT_TRUE(cISDataMappings::DataToString(info, &hdr, payload.data(), withConv,
                                                  /*arrayIndex*/ 0, /*json*/ false, /*useConversion*/ true));
        ASSERT_TRUE(cISDataMappings::DataToString(info, &hdr, payload.data(), noConv,
                                                  /*arrayIndex*/ 0, /*json*/ false, /*useConversion*/ false));
        EXPECT_STRNE(withConv, noConv) << "conversion must change the rendered value";
        // raw 1.0 rad/s -> unconverted ~1.0, converted ~= the conversion factor.
        EXPECT_NEAR(std::strtod(noConv, nullptr), 1.0, 0.01);
        EXPECT_NEAR(std::strtod(withConv, nullptr), info.conversion, info.conversion * 0.01);
        ++seen;
    }
    EXPECT_EQ(seen, 1u);

    ISFileManager::DeleteDirectory(raw.parent_path().string());
}

}  // namespace
