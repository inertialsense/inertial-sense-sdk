/**
 * Unit tests for the SDK serial-layer baud-rate policy (SN-8239).
 *
 * These exercise the pure, hardware-independent decision logic that gates baud-rate selection:
 *  - serialPortBaudRateSupported(): is a rate accepted at all (0 < rate <= 10 Mbaud)?
 *  - serialPortStandardBaudRate(): does a rate have a standard termios Bxxx constant, or must it use
 *    the platform custom-rate path (return 0)?
 *
 * The actual USB DFU/serial transfer requires hardware and is covered by bench validation (Set C).
 * Both functions are POSIX-only (they live in the non-Windows branch of serialPortPlatform.c and
 * serialPortStandardBaudRate returns termios Bxxx constants), so the suite is compiled only off-Windows.
 */

#include <gtest/gtest.h>

#if !defined(_WIN32)

extern "C" {
#include "serialPortPlatform.h"
}

// --- serialPortBaudRateSupported(): accept any positive rate up to SERIAL_PORT_BAUDRATE_MAX ---

TEST(SerialPortBaudRate, Supported_AcceptsInRange) {
    EXPECT_EQ(1, serialPortBaudRateSupported(9600));
    EXPECT_EQ(1, serialPortBaudRateSupported(921600));
    EXPECT_EQ(1, serialPortBaudRateSupported(4000000));      // the SN-8239 bug rate
    EXPECT_EQ(1, serialPortBaudRateSupported(5000000));      // arbitrary custom
    EXPECT_EQ(1, serialPortBaudRateSupported(SERIAL_PORT_BAUDRATE_MAX)); // 10 Mbaud ceiling
}

TEST(SerialPortBaudRate, Supported_RejectsOutOfRange) {
    EXPECT_EQ(0, serialPortBaudRateSupported(0));
    EXPECT_EQ(0, serialPortBaudRateSupported(-1));
    EXPECT_EQ(0, serialPortBaudRateSupported(SERIAL_PORT_BAUDRATE_MAX + 1)); // above 10 Mbaud
}

// --- serialPortStandardBaudRate(): non-zero Bxxx for standard rates, 0 for custom-path rates ---

TEST(SerialPortBaudRate, Standard_KnownRatesMapToConstant) {
    EXPECT_NE(0, serialPortStandardBaudRate(9600));
    EXPECT_NE(0, serialPortStandardBaudRate(921600));
    EXPECT_NE(0, serialPortStandardBaudRate(1000000));       // previously missing from the switch
    EXPECT_NE(0, serialPortStandardBaudRate(3000000));
    EXPECT_NE(0, serialPortStandardBaudRate(4000000));       // the SN-8239 fix: now has B4000000
}

TEST(SerialPortBaudRate, Standard_CustomAndInvalidRatesReturnZero) {
    // 1220000 / 1440000 are advertised BAUDRATE_* values but have NO standard Linux Bxxx constant,
    // so they must route through the custom (BOTHER) path -> 0 here (0 != "unsupported").
    EXPECT_EQ(0, serialPortStandardBaudRate(1220000));
    EXPECT_EQ(0, serialPortStandardBaudRate(1440000));
    EXPECT_EQ(0, serialPortStandardBaudRate(5000000));       // arbitrary custom rate
    EXPECT_EQ(0, serialPortStandardBaudRate(1234));          // non-standard nonsense
    EXPECT_EQ(0, serialPortStandardBaudRate(0));
}

// A custom rate that has no standard constant must still be reported as supported (it will use the
// termios2/BOTHER path) - this is the pairing that the SN-8239 custom-rate feature relies on.
TEST(SerialPortBaudRate, CustomRate_SupportedButNotStandard) {
    EXPECT_EQ(1, serialPortBaudRateSupported(1440000));
    EXPECT_EQ(0, serialPortStandardBaudRate(1440000));
    EXPECT_EQ(1, serialPortBaudRateSupported(6000000));
    EXPECT_EQ(0, serialPortStandardBaudRate(6000000));
}

#endif // !_WIN32
