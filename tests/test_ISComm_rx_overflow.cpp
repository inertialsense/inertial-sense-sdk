/**
 * @file test_ISComm_rx_overflow.cpp
 * @brief is_comm_parse_byte() must never write past the end of the comm RX buffer.
 */

#include <cstring>

#include <gtest/gtest.h>

#include "ISComm.h"

namespace {
constexpr int RX_BUF_SIZE = 64;
constexpr int GUARD_SIZE = 16;
constexpr uint8_t GUARD = 0xA5;
}

// Fill the RX buffer to capacity the documented "fast" way (write at rxBuf.tail, advance tail) without
// parsing it, then feed one more byte through is_comm_parse_byte(). is_comm_free() reports 0 free bytes
// but does not reset the buffer (no packet in progress, unscanned data remains), so the byte used to be
// stored at rxBuf.end -- one past the buffer.
TEST(ISComm, ParseByteOnFullBufferDoesNotWritePastEnd)
{
    uint8_t mem[RX_BUF_SIZE + GUARD_SIZE];
    is_comm_instance_t comm;
    is_comm_init(&comm, mem, RX_BUF_SIZE, NULL);
    memset(mem + RX_BUF_SIZE, GUARD, GUARD_SIZE);

    ASSERT_EQ(is_comm_free(&comm), RX_BUF_SIZE);
    memset(comm.rxBuf.tail, 0x00, RX_BUF_SIZE);     // payload that matches no protocol preamble
    comm.rxBuf.tail += RX_BUF_SIZE;
    ASSERT_EQ(comm.rxBuf.tail, comm.rxBuf.end);

    is_comm_parse_byte(&comm, 0x42);

    for (int i = 0; i < GUARD_SIZE; i++)
        EXPECT_EQ(mem[RX_BUF_SIZE + i], GUARD) << "byte written " << i << " past the RX buffer end";
    EXPECT_LE(comm.rxBuf.tail, comm.rxBuf.end);

    // The stale data was flushed and the new byte stored at the start of the buffer.
    EXPECT_EQ(comm.rxBuf.tail, comm.rxBuf.start + 1);
    EXPECT_EQ(mem[0], 0x42);
}
