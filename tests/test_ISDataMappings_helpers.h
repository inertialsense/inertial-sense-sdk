/**
 * @file test_ISDataMappings_helpers.h
 * @brief Small reusable harness for testing `data_info_t::renderExtended` wiring in
 *        ISDataMappings.cpp -- i.e. that a given (DID, field name) actually decodes to a
 *        non-trivial tooltip string in EvalTool, not just that a decode table exists somewhere
 *        (see test_ISStatusDecode.cpp for table-level tests).
 *
 * Before SN-8491 there was no test coverage at all for whether a field's `renderExtended` was
 * actually assigned and reachable through `cISDataMappings::NameToInfoMap()` -- only the decode
 * tables themselves were tested. `data_info_t::renderExtended` is default-initialized to a no-op
 * lambda (not nullptr), so a pointer-nullity check can't tell "wired" apart from "unwired"; these
 * helpers call through with a representative value and assert on the actual rendered string.
 *
 * @copyright Copyright (c) 2026 Inertial Sense, Inc. All rights reserved.
 */

#ifndef IS_SDK_UNIT_TESTS_TEST_ISDATAMAPPINGS_HELPERS_H
#define IS_SDK_UNIT_TESTS_TEST_ISDATAMAPPINGS_HELPERS_H

#include <gtest/gtest.h>

#include <any>
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
 *        anything -- callers decide what output (empty vs. containing specific text) is expected.
 */
inline std::string CallRenderExtended(const data_info_t& info, std::any value, int arrayIdx = 0, int flags = 0)
{
    return info.renderExtended(info, value, arrayIdx, flags);
}

/**
 * @brief One (DID, field name, representative non-zero raw value) case used to assert that a
 *        field's `renderExtended` is actually wired to a real decoder -- i.e. produces non-empty
 *        output for a value expected to decode to at least one line -- as opposed to silently
 *        falling through to the default no-op renderer (which also returns a non-throwing empty
 *        string, so a pointer-nullity check on `renderExtended` cannot distinguish the two).
 */
struct RenderExtendedWiringCase
{
    uint32_t did;
    const char* fieldName;
    std::any representativeValue;   //!< a value expected to render at least one decoded line
    const char* label;              //!< human label for gtest failure output (e.g. "DID_IMU.status")
};

/**
 * @brief Assert that `c.fieldName` on `c.did` is registered and that calling its `renderExtended`
 *        with `c.representativeValue` produces non-empty output.
 */
inline void ExpectRenderExtendedWired(const RenderExtendedWiringCase& c)
{
    const data_info_t* info = FindMappedField(c.did, c.fieldName);
    ASSERT_NE(info, nullptr) << c.label;
    const std::string rendered = CallRenderExtended(*info, c.representativeValue);
    EXPECT_FALSE(rendered.empty()) << c.label << " renderExtended produced empty output for a value expected to decode";
}

#endif // IS_SDK_UNIT_TESTS_TEST_ISDATAMAPPINGS_HELPERS_H
