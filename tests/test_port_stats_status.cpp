/**
 * @file test_port_stats_status.cpp
 * @brief SN-8862 -- portStatsStatus() encodes the state a port already holds (valid / opened / last error) into
 *        port_stats_t::status, so a closed, never-opened or invalidated port reads differently from a healthy one.
 */

#include <gtest/gtest.h>

#include "core/base_port.h"

namespace {

base_port_t makePort(uint16_t flags) {
    base_port_t p = {};
    p.pnum = 2;
    p.ptype = PORT_TYPE__UART | PORT_TYPE__COMM;
    p.pflags = flags;
    portRecalcChksum(&p);
    return p;
}

} // namespace

TEST(PortStatsStatus, HealthyOpenPortIsValidOpenAndErrorFree) {
    base_port_t p = makePort(PORT_FLAG__VALID | PORT_FLAG__OPENED);
    const uint32_t s = portStatsStatus(&p);
    EXPECT_EQ(s, (uint32_t)(PORT_STATUS__REPORTED | PORT_STATUS__VALID | PORT_STATUS__OPENED));
    EXPECT_EQ(s & PORT_STATUS__ERROR, 0u);
    EXPECT_EQ(s & PORT_STATUS__ERROR_MASK, 0u);
}

TEST(PortStatsStatus, ValidButNeverOpenedPortReadsClosed) {
    base_port_t p = makePort(PORT_FLAG__VALID);
    const uint32_t s = portStatsStatus(&p);
    EXPECT_EQ(s, (uint32_t)(PORT_STATUS__REPORTED | PORT_STATUS__VALID));
    EXPECT_NE(s, portStatsStatus(nullptr)) << "a valid closed port is not the same as no port";
}

TEST(PortStatsStatus, ClosedAfterBeingOpenedClearsTheOpenBit) {
    base_port_t p = makePort(PORT_FLAG__VALID | PORT_FLAG__OPENED);
    portFlagsClear(&p, PORT_FLAG__OPENED);
    EXPECT_EQ(portStatsStatus(&p), (uint32_t)(PORT_STATUS__REPORTED | PORT_STATUS__VALID));
}

TEST(PortStatsStatus, InvalidatedPortDropsTheValidBitAndIsNeverZero) {
    base_port_t p = makePort(PORT_FLAG__VALID | PORT_FLAG__OPENED);
    portInvalidate(&p);
    const uint32_t s = portStatsStatus(&p);
    EXPECT_EQ(s & PORT_STATUS__VALID, 0u);
    EXPECT_NE(s, 0u) << "an invalidated port must not read as 'not reported' (0)";
    EXPECT_NE(s, (uint32_t)(PORT_STATUS__REPORTED | PORT_STATUS__VALID | PORT_STATUS__OPENED));
}

TEST(PortStatsStatus, LastErrorCodeIsCarriedInTheUpperBits) {
    const int16_t codes[] = { PORT_ERROR__NOT_CONNECTED, PORT_ERROR__WRITE_FAILURE, PORT_ERROR__READ_FAILURE, PORT_ERROR__TIMEOUT, PORT_ERROR__OPEN_FAILURE };
    for (int16_t code : codes) {
        base_port_t p = makePort(PORT_FLAG__VALID | PORT_FLAG__OPENED);
        p.perror = (uint16_t)code;
        const uint32_t s = portStatsStatus(&p);
        EXPECT_NE(s & PORT_STATUS__ERROR, 0u) << "code " << code;
        EXPECT_EQ((int16_t)((s & PORT_STATUS__ERROR_MASK) >> PORT_STATUS__ERROR_SHIFT), code);
    }
}

TEST(PortStatsStatus, NullPortReadsAsInvalidClosedAndReported) {
    EXPECT_EQ(portStatsStatus(nullptr), (uint32_t)PORT_STATUS__REPORTED);
}

TEST(PortStatsStatus, EveryStateHasADistinctWord) {
    base_port_t healthy = makePort(PORT_FLAG__VALID | PORT_FLAG__OPENED);
    base_port_t closed = makePort(PORT_FLAG__VALID);
    base_port_t invalidated = makePort(PORT_FLAG__VALID | PORT_FLAG__OPENED); portInvalidate(&invalidated);
    base_port_t errored = makePort(PORT_FLAG__VALID | PORT_FLAG__OPENED); errored.perror = (uint16_t)PORT_ERROR__WRITE_FAILURE;
    const uint32_t w[] = { portStatsStatus(&healthy), portStatsStatus(&closed), portStatsStatus(&invalidated), portStatsStatus(&errored) };
    for (int i = 0; i < 4; i++) for (int j = i + 1; j < 4; j++) EXPECT_NE(w[i], w[j]) << i << " vs " << j;
}

namespace {
base_port_t makeUsbPort(uint16_t flags) { base_port_t p = makePort(flags); p.ptype = PORT_TYPE__USB | PORT_TYPE__COMM; portRecalcChksum(&p); return p; }
}

TEST(PortStatsStatusUsb, EnumeratedIsItsOwnBit) {
    const uint32_t others = PORT_STATUS__VALID | PORT_STATUS__OPENED | PORT_STATUS__ERROR | PORT_STATUS__REPORTED | PORT_STATUS__ERROR_MASK;
    EXPECT_EQ(PORT_STATUS__ENUMERATED & others, 0u);
}

TEST(PortStatsStatusUsb, ThreeLevelsReadDistinctlyAndWithTheDocumentedWords) {
    base_port_t usb = makeUsbPort(PORT_FLAG__VALID | PORT_FLAG__OPENED);   // firmware flags the port OPENED from init; that must not leak into the status
    EXPECT_EQ(portStatsStatusUsb(&usb, false, false), (uint32_t)(PORT_STATUS__REPORTED | PORT_STATUS__VALID));
    EXPECT_EQ(portStatsStatusUsb(&usb, true,  false), (uint32_t)(PORT_STATUS__REPORTED | PORT_STATUS__VALID | PORT_STATUS__ENUMERATED));
    EXPECT_EQ(portStatsStatusUsb(&usb, true,  true),  (uint32_t)(PORT_STATUS__REPORTED | PORT_STATUS__VALID | PORT_STATUS__ENUMERATED | PORT_STATUS__OPENED));
    EXPECT_EQ(portStatsStatusUsb(&usb, true,  true),  0x1Bu);
}

TEST(PortStatsStatusUsb, StaleDtrWithoutEnumerationDoesNotReadOpen) {
    // IMX-5 only clears its DTR flag on a SET_CONTROL_LINE_STATE, so it can be left set by an unplugged cable
    base_port_t usb = makeUsbPort(PORT_FLAG__VALID | PORT_FLAG__OPENED);
    EXPECT_EQ(portStatsStatusUsb(&usb, false, true) & (PORT_STATUS__OPENED | PORT_STATUS__ENUMERATED), 0u);
}

TEST(PortStatsStatusUsb, InvalidatedPortReportsNeitherEnumeratedNorOpen) {
    base_port_t usb = makeUsbPort(PORT_FLAG__VALID | PORT_FLAG__OPENED); portInvalidate(&usb);
    const uint32_t st = portStatsStatusUsb(&usb, true, true);
    EXPECT_EQ(st & (PORT_STATUS__VALID | PORT_STATUS__ENUMERATED | PORT_STATUS__OPENED), 0u);
    EXPECT_NE(st & PORT_STATUS__REPORTED, 0u);
    EXPECT_EQ(portStatsStatusUsb(nullptr, true, true), (uint32_t)PORT_STATUS__REPORTED);
}

TEST(PortStatsStatusUsb, OpenedImpliesEnumeratedImpliesValidForEveryInput) {
    for (int flags = 0; flags < 2; flags++) for (int en = 0; en < 2; en++) for (int open = 0; open < 2; open++) {
        base_port_t usb = makeUsbPort(flags ? (PORT_FLAG__VALID | PORT_FLAG__OPENED) : 0);
        const uint32_t st = portStatsStatusUsb(&usb, en, open);
        if (st & PORT_STATUS__OPENED)     EXPECT_NE(st & PORT_STATUS__ENUMERATED, 0u) << flags << en << open;
        if (st & PORT_STATUS__ENUMERATED) EXPECT_NE(st & PORT_STATUS__VALID, 0u) << flags << en << open;
    }
}

TEST(PortStatsStatusUsb, LastErrorCodeSurvives) {
    base_port_t usb = makeUsbPort(PORT_FLAG__VALID | PORT_FLAG__OPENED); usb.perror = (uint16_t)PORT_ERROR__WRITE_FAILURE;
    const uint32_t st = portStatsStatusUsb(&usb, true, true);
    EXPECT_NE(st & PORT_STATUS__ERROR, 0u);
    EXPECT_EQ((st & PORT_STATUS__ERROR_MASK) >> PORT_STATUS__ERROR_SHIFT, (uint32_t)(uint16_t)PORT_ERROR__WRITE_FAILURE);
}
