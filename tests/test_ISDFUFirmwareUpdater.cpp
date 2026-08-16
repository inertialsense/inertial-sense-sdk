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

// --- isExpectedOptionByteResetError(): disconnect-class libusb error after the Option-Bytes
//     write is the expected, successful end of finalize; everything else is a real failure (SN-8193) ---

TEST(ISDFUFirmwareUpdater_Finalize, ExpectedResetDisconnects_AreSuccess) {
    // Writing the Option Bytes triggers a mandatory immediate reset; the device drops off the bus
    // mid-transfer. These are the disconnect-class codes libusb reports for that, and they must NOT
    // be treated as finalize failures (the false "Complete (finalize warning: LIBUSB_ERROR)" bug).
    EXPECT_TRUE(DFUDevice::isExpectedOptionByteResetError(LIBUSB_ERROR_NO_DEVICE));
    EXPECT_TRUE(DFUDevice::isExpectedOptionByteResetError(LIBUSB_ERROR_IO));
    EXPECT_TRUE(DFUDevice::isExpectedOptionByteResetError(LIBUSB_ERROR_PIPE));
}

TEST(ISDFUFirmwareUpdater_Finalize, SuccessAndGenuineErrors_AreNotResetDisconnects) {
    // A successful transfer (>= 0) is not an error at all, and genuine fault codes must still fail
    // finalize rather than being silently swallowed as an "expected reset".
    EXPECT_FALSE(DFUDevice::isExpectedOptionByteResetError(LIBUSB_SUCCESS));
    EXPECT_FALSE(DFUDevice::isExpectedOptionByteResetError(40)); // positive = bytes transferred
    EXPECT_FALSE(DFUDevice::isExpectedOptionByteResetError(LIBUSB_ERROR_TIMEOUT));
    EXPECT_FALSE(DFUDevice::isExpectedOptionByteResetError(LIBUSB_ERROR_ACCESS));
    EXPECT_FALSE(DFUDevice::isExpectedOptionByteResetError(LIBUSB_ERROR_INVALID_PARAM));
    EXPECT_FALSE(DFUDevice::isExpectedOptionByteResetError(LIBUSB_ERROR_BUSY));
    EXPECT_FALSE(DFUDevice::isExpectedOptionByteResetError(LIBUSB_ERROR_NO_MEM));
    EXPECT_FALSE(DFUDevice::isExpectedOptionByteResetError(LIBUSB_ERROR_NOT_SUPPORTED));
}

// --- libusbErrorName(): the specific libusb sub-code is now preserved and nameable for
//     diagnostics. Previously `DFU_ERROR_LIBUSB | (code << 16)` discarded it (the -4 tag is
//     sign-extended to all-ones, so OR-ing the shifted code was a no-op and the result was
//     always -4, regardless of the underlying libusb error). SN-8193. ---

TEST(ISDFUFirmwareUpdater_Finalize, LibusbErrorName_NamesDistinctCodes) {
    // The distinct disconnect-class codes that the old packing collapsed into an indistinguishable
    // "-4" must now map to distinct, recognizable names.
    EXPECT_STREQ("LIBUSB_ERROR_NO_DEVICE", DFUDevice::libusbErrorName(LIBUSB_ERROR_NO_DEVICE));
    EXPECT_STREQ("LIBUSB_ERROR_IO",        DFUDevice::libusbErrorName(LIBUSB_ERROR_IO));
    EXPECT_STREQ("LIBUSB_ERROR_PIPE",      DFUDevice::libusbErrorName(LIBUSB_ERROR_PIPE));
    EXPECT_STREQ("LIBUSB_ERROR_TIMEOUT",   DFUDevice::libusbErrorName(LIBUSB_ERROR_TIMEOUT));

    // They are genuinely distinct (regression guard against any future collapse to one code).
    EXPECT_STRNE(DFUDevice::libusbErrorName(LIBUSB_ERROR_NO_DEVICE),
                 DFUDevice::libusbErrorName(LIBUSB_ERROR_IO));
}

// --- getNumDevices() before initLibUSB(): SN-8492 regression -----------------------------------
//
// getNumDevices() previously called libusb_get_device_list(NULL, ...) unconditionally, and
// initLibUSB() discarded libusb_init()'s return value -- so on any environment where libusb_init()
// fails (no USB subsystem, restricted permissions, sandboxing), the default context stayed
// uninitialized and getNumDevices() segfaulted inside pthread_mutex_lock on first call. This test
// exercises the exact failure precondition directly: getNumDevices() called before initLibUSB()
// has EVER succeeded in this process, relying on ISDFUFirmwareUpdater::libUsbAvailable's true
// default-initialized value (false). MUST run before any test/call in this binary invokes
// initLibUSB() successfully, since that flag is process-wide, static, and has no reset hook.

TEST(ISDFUFirmwareUpdater_LibUsbInit, GetNumDevices_BeforeInit_ReturnsZeroInsteadOfCrashing) {
    EXPECT_EQ(0, ISDFUFirmwareUpdater::getNumDevices());
    EXPECT_EQ(0, ISDFUFirmwareUpdater::getNumDevices(0x1234, 0x5678));
}
