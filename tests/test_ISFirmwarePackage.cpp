/**
 * @file test_ISFirmwarePackage.cpp 
 * @brief ${BRIEF_DESC}
 *
 * @author Kyle Mallory on 1/15/24.
 * @copyright Copyright (c) 2024 Inertial Sense, Inc. All rights reserved.
 */

#include <gtest/gtest.h>
#include "gtest_helpers.h"

#include "ISFirmwareUpdater.h"

TEST(ISFirmwarePackage, parse_package) {
    dev_info_t devInfo;
    ISFirmwareUpdater* updater = new ISFirmwareUpdater(0, "/dev/ttyACM0", &devInfo);
    // updater.openFirmwarePackage("test.fpk");
    delete updater;
}