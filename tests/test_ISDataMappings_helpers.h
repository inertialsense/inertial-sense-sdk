/**
 * @file test_ISDataMappings_helpers.h
 * @brief Small reusable harness for testing `data_info_t::renderExtended` wiring in
 *        ISDataMappings.cpp -- i.e. that a given (DID, field name) actually decodes to a
 *        non-trivial tooltip string in EvalTool, not just that a decode table exists somewhere
 *        (see test_ISStatusDecode.cpp for table-level tests).
 *
 * Before SN-8491 there was no test coverage at all for whether a field's `renderExtended` was
 * actually assigned to a specialized decoder. Two traps make this harder than it looks:
 *  1. `data_info_t::renderExtended` is default-initialized to a no-op lambda, but `AddMember()`
 *     unconditionally overwrites that default with `renderVariableAndStatsToString` (a generic
 *     "format the raw value, honoring DATA_FLAGS_DISPLAY_HEX" renderer) BEFORE any specialized
 *     `.renderExtended = renderXxx` assignment runs. So an UNWIRED field's `renderExtended` is
 *     never the trivial no-op -- it's this generic formatter.
 *  2. That generic formatter itself produces non-empty output for almost any input (e.g. a plain
 *     hex string like "0xFFFFFFFF"), so a pointer-nullity check AND a plain "is the output
 *     non-empty" check both fail to distinguish "wired to a specialized decoder" from "still
 *     falling through to the generic formatter." (SN-8491 discovered this the hard way: an
 *     earlier version of this harness asserted non-emptiness alone and could not have caught a
 *     field that was never actually wired.)
 * The only reliable signal is checking for text that ONLY the specialized decoder would produce
 * (a decoded label, not a hex digit) -- hence `expectedSubstring` below, not just non-emptiness.
 *
 * @copyright Copyright (c) 2026 Inertial Sense, Inc. All rights reserved.
 */

#ifndef IS_SDK_UNIT_TESTS_TEST_ISDATAMAPPINGS_HELPERS_H
#define IS_SDK_UNIT_TESTS_TEST_ISDATAMAPPINGS_HELPERS_H

#include <gtest/gtest.h>

#include <any>
#include <cstdio>
#include <string>

#include "ISDataMappings.h"

/**
 * @brief Look up a field's `data_info_t` by (DID, name), failing the current test with a clear
 *        message (rather than throwing/crashing via map::at) if the DID or field isn't registered.
 */
inline const data_info_t* FindMappedField(uint32_t did, const std::string& fieldName)
{
    const map_name_to_info_t* map = cISDataMappings::NameToInfoMap(did);
    if (map == nullptr)
    {
        ADD_FAILURE() << "DID " << did << " has no registered field map";
        return nullptr;
    }
    auto it = map->find(fieldName);
    if (it == map->end())
    {
        ADD_FAILURE() << "DID " << did << " has no field named \"" << fieldName << "\"";
        return nullptr;
    }
    return &it->second;
}

/**
 * @brief Invoke `info.renderExtended` directly and return its output. Does not itself assert
 *        anything -- callers decide what output is expected.
 */
inline std::string CallRenderExtended(const data_info_t& info, std::any value, int arrayIdx = 0, int flags = 0)
{
    return info.renderExtended(info, value, arrayIdx, flags);
}

/**
 * @brief One (DID, field name, representative raw value, expected decoded substring) case used to
 *        assert that a field's `renderExtended` is wired to a real specialized decoder -- i.e. its
 *        output contains text that only that decoder could have produced -- as opposed to silently
 *        falling through to the generic `renderVariableAndStatsToString` formatter every field gets
 *        by default (see the file-level comment above for why non-emptiness alone can't tell these
 *        apart).
 */
struct RenderExtendedWiringCase
{
    uint32_t did;
    const char* fieldName;
    std::any representativeValue;    //!< a value expected to decode to `expectedSubstring`
    const char* expectedSubstring;   //!< text ONLY the specialized decoder would emit (a label, not a hex digit)
    const char* label;               //!< human label for gtest failure output (e.g. "DID_IMU.status")
};

/**
 * @brief Assert that `c.fieldName` on `c.did` is registered and that calling its `renderExtended`
 *        with `c.representativeValue` produces output containing `c.expectedSubstring`.
 */
inline void ExpectRenderExtendedWired(const RenderExtendedWiringCase& c)
{
    const data_info_t* info = FindMappedField(c.did, c.fieldName);
    ASSERT_NE(info, nullptr) << c.label;
    const std::string rendered = CallRenderExtended(*info, c.representativeValue);
    EXPECT_NE(rendered.find(c.expectedSubstring), std::string::npos)
        << c.label << " renderExtended output \"" << rendered << "\" did not contain expected \"" << c.expectedSubstring << "\"";
}

/**
 * @brief Assert that `rendered` contains a "0xHEX - name" line for `mask`, where HEX is the
 *        mask value itself formatted to `hexDigits` digits. This is the single-bit-flag
 *        convention: a lone set bit's hex is always unambiguous regardless of nibble alignment,
 *        so `renderBitNameList()`'s callers (RTKCfgBits, rtkMode, grmcBits, grmcNMEABits, or
 *        rmc_t.bits -- all flat single-bit-flag lists) and single-bit Bit-kind decode-table lines
 *        keep this format. For multi-bit Enum-kind decode-table subfields, see
 *        `ExpectEnumSubfieldLine()` below instead (Kyle, 2026-08-13: bit-range annotation over
 *        hex there, since those fields are rarely nibble-aligned). The expected prefix is
 *        computed from `mask` with the same format string the production code uses, rather than
 *        hand-typed as a literal, so this can't silently drift from the real bit value.
 */
inline void ExpectHexPrefixedLine(const std::string& rendered, unsigned long long mask, const std::string& name, int hexDigits = 16)
{
    char hexbuf[24];
    std::snprintf(hexbuf, sizeof(hexbuf), (hexDigits <= 8) ? "0x%08llX" : "0x%016llX", mask);
    const std::string expectedLine = std::string(hexbuf) + " - " + name;
    EXPECT_NE(rendered.find(expectedLine), std::string::npos)
        << "expected line \"" << expectedLine << "\" not found in \"" << rendered << "\"";
}

/**
 * @brief Assert that `rendered` contains the line `RenderStatusFromDecode()`'s Enum case produces
 *        for a subfield with the given `mask`/`shift`, given its raw (pre-shift) enum index
 *        `rawIndexValue` -- "bits[hi:lo]=0xN - name" for a multi-bit mask, or "0xHEX - name" for a
 *        single-bit mask (mirroring production's own single-bit-vs-multi-bit branch). `hi`/`lo`
 *        are derived from `mask` via the same ctz/clz approach production uses, never hand-typed,
 *        so a future re-bit-packing of a field can't silently desync the test from reality.
 */
inline void ExpectEnumSubfieldLine(const std::string& rendered, uint32_t mask, uint32_t shift, unsigned long long rawIndexValue, const std::string& name)
{
    const int lo = mask ? __builtin_ctz(mask) : 0;
    const int hi = mask ? (31 - __builtin_clz(mask)) : 0;
    char buf[64];
    std::string expectedLine;
    if (hi == lo)
    {
        std::snprintf(buf, sizeof(buf), "0x%08llX", rawIndexValue << shift);
        expectedLine = std::string(buf) + " - " + name;
    }
    else
    {
        std::snprintf(buf, sizeof(buf), "bits[%d:%d]=0x%llX", hi, lo, rawIndexValue);
        expectedLine = std::string(buf) + " - " + name;
    }
    EXPECT_NE(rendered.find(expectedLine), std::string::npos)
        << "expected line \"" << expectedLine << "\" not found in \"" << rendered << "\"";
}

#endif // IS_SDK_UNIT_TESTS_TEST_ISDATAMAPPINGS_HELPERS_H
