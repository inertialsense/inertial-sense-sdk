/**
 * @file test_cbor_registry.cpp
 * @brief The SDK side of the CBOR key registry: DID_CBOR is registered, and the generated key constants and name
 *        tables agree with each other.
 *
 * @copyright Copyright (c) 2026 Inertial Sense, Inc.
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
    EXPECT_STREQ(cbor_registry_field_name(CBOR_DID_FLASH_CFG, CBOR_FLASH_CFG__SER0BAUDRATE), "ser0BaudRate");
    EXPECT_EQ(cbor_registry_field_key(CBOR_DID_FLASH_CFG, "ser0BaudRate"), CBOR_FLASH_CFG__SER0BAUDRATE);
    EXPECT_EQ(cbor_registry_field_key(CBOR_DID_GPX_FLASH_CFG, "ser0BaudRate"), CBOR_GPX_FLASH_CFG__SER0BAUDRATE);
    EXPECT_STREQ(cbor_registry_field_name(CBOR_DID_RMC_STORED, CBOR_RMC_STORED__RMC_PRESET), "RMC_PRESET");
}

TEST(CborRegistry, UnknownFieldsAreNotFound)
{
    EXPECT_EQ(cbor_registry_field_name(CBOR_DID_FLASH_CFG, 0x7FFF), nullptr);
    EXPECT_EQ(cbor_registry_field_key(CBOR_DID_FLASH_CFG, "noSuchField"), -1);
}

TEST(CborRegistry, LegacySchemasAreInKeyOrder)
{
    for (const cbor_schema_t* s : { &g_cborSchema_FLASH_CFG, &g_cborSchema_CAN_CONFIG, &g_cborSchema_GPX_FLASH_CFG })
        for (uint16_t i = 1; i < s->count; i++)
            EXPECT_LT(s->fields[i - 1].key, s->fields[i].key);
}
