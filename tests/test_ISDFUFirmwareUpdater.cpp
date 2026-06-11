/**
 * Unit tests for the DFU updater's read-protection (RDP) decision logic and error-name lookup.
 *
 * SN-8043: the USB DFU transfer itself requires hardware and is covered by a hardware-in-the-loop
 * test (Set C). These tests exercise the pure, hardware-independent decision logic that gates the
 * pre-flight RDP check and the post-write verification error reporting.
 */

#include <gtest/gtest.h>
#include "ISDFUFirmwareUpdater.h"

// --- rdpVerdict(): raw FLASH_OPTR RDP byte -> dfu_error verdict ---

TEST(ISDFUFirmwareUpdater_RDP, Level0_Unprotected_Proceeds) {
    // RDP == 0xAA is the only value that permits flash programming.
    EXPECT_EQ(DFU_ERROR_NONE, DFUDevice::rdpVerdict(0xAA));
}

TEST(ISDFUFirmwareUpdater_RDP, Level2_Permanent_Locked) {
    EXPECT_EQ(DFU_ERROR_RDP_PERMANENT_LOCKED, DFUDevice::rdpVerdict(0xCC));
}

TEST(ISDFUFirmwareUpdater_RDP, Level1_Locked) {
    // 0xBB is the production state called out in the bug report; any non-0xAA/0xCC value is Level 1.
    EXPECT_EQ(DFU_ERROR_RDP_LOCKED, DFUDevice::rdpVerdict(0xBB));
    EXPECT_EQ(DFU_ERROR_RDP_LOCKED, DFUDevice::rdpVerdict(0x00));
    EXPECT_EQ(DFU_ERROR_RDP_LOCKED, DFUDevice::rdpVerdict(0xFF));
    EXPECT_EQ(DFU_ERROR_RDP_LOCKED, DFUDevice::rdpVerdict(0x55));
    EXPECT_EQ(DFU_ERROR_RDP_LOCKED, DFUDevice::rdpVerdict(0xAB)); // one off from unprotected
}

// --- getErrorName(): index convention is -dfu_error; must be bounds-safe ---

TEST(ISDFUFirmwareUpdater_RDP, ErrorNames_KnownCodes) {
    EXPECT_STREQ("SUCCESS",              DFUDevice::getErrorName(-DFU_ERROR_NONE));
    EXPECT_STREQ("INVALID_IMAGE",        DFUDevice::getErrorName(-DFU_ERROR_FILE_INVALID));
    EXPECT_STREQ("RDP_LOCKED",           DFUDevice::getErrorName(-DFU_ERROR_RDP_LOCKED));
    EXPECT_STREQ("RDP_PERMANENT_LOCKED", DFUDevice::getErrorName(-DFU_ERROR_RDP_PERMANENT_LOCKED));
    EXPECT_STREQ("WRITE_VERIFY_FAILED",  DFUDevice::getErrorName(-DFU_ERROR_WRITE_VERIFY_FAILED));
}

TEST(ISDFUFirmwareUpdater_RDP, ErrorNames_OutOfRangeIsSafe) {
    // Negative indices (e.g. from the legacy "-(err & 0xF)" caller convention on packed libusb
    // errors) and overly-large indices must not read out of bounds.
    EXPECT_STREQ("UNKNOWN", DFUDevice::getErrorName(-1));
    EXPECT_STREQ("UNKNOWN", DFUDevice::getErrorName(9999));
    EXPECT_STREQ("UNKNOWN", DFUDevice::getErrorName(-100));
}
