/**
 * @file test_cbor_registry.cpp
 * @brief The SDK side of the CBOR key registry: DID_CBOR is registered, and the generated key constants and name
 *        tables agree with each other.
 *
 * @author Kyle Mallory on 2026-10-23
 * @copyright Copyright (c) 2026 Inertial Sense, Inc. All rights reserved.
 */

#include <gtest/gtest.h>
#include <cstring>

#include "ISDataMappings.h"
#include "cbor/cbor_registry.h"

TEST(CborRegistry, CborDidIsRegistered)
{
    EXPECT_EQ(DID_CBOR, CBOR_ISB_DID);
    EXPECT_STREQ(cISDataMappings::DataName(DID_CBOR), "DID_CBOR");
}

TEST(CborRegistry, FieldNamesAndKeysRoundTrip)
{
    EXPECT_STREQ(cbor_registry_field_name(CBOR_DOMAIN_CONFIG_INS, CBOR_CONFIG_INS__DYNAMICMODEL), "dynamicModel");
    EXPECT_EQ(cbor_registry_field_key(CBOR_DOMAIN_CONFIG_INS, "dynamicModel"), CBOR_CONFIG_INS__DYNAMICMODEL);
    EXPECT_EQ(cbor_registry_field_key(CBOR_DOMAIN_CONFIG_SYSTEM, "navPeriodMs"), CBOR_CONFIG_SYSTEM__NAVPERIODMS);
    EXPECT_STREQ(cbor_registry_field_name(CBOR_DOMAIN_RMC_STORED, CBOR_RMC_STORED__RMC_PRESET), "RMC_PRESET");
}

TEST(CborRegistry, UnknownFieldsAreNotFound)
{
    EXPECT_EQ(cbor_registry_field_name(CBOR_DOMAIN_CONFIG_INS, 0x7FFF), nullptr);
    EXPECT_EQ(cbor_registry_field_key(CBOR_DOMAIN_CONFIG_INS, "noSuchField"), -1);
    EXPECT_EQ(cbor_registry_did_key("FLASH_CFG"), -1) << "the flash block's schemas are not domains";
}

TEST(CborRegistry, DomainSchemasAreInKeyOrder)
{
    for (const cbor_schema_t* s : { &g_cborSchema_CONFIG_SYSTEM, &g_cborSchema_CONFIG_INS, &g_cborSchema_CONFIG_GNSS,
                                    &g_cborSchema_CONFIG_PORTS, &g_cborSchema_CONFIG_CAN, &g_cborSchema_STATE })
        for (uint16_t i = 1; i < s->count; i++)
            EXPECT_LT(s->fields[i - 1].key, s->fields[i].key);
}
