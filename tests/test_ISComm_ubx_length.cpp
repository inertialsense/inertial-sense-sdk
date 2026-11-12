/**
 * @file test_ISComm_ubx_length.cpp
 * @brief The UBX parser must reject a header whose length can never fit, and resync on the next packet.
 */

#include <cstring>
#include <vector>

#include <gtest/gtest.h>

#include "ISComm.h"

namespace {

std::vector<uint8_t> makeUbx(uint8_t cls, uint8_t id, const std::vector<uint8_t>& payload)
{
    std::vector<uint8_t> pkt = { UBLOX_START_BYTE1, UBLOX_START_BYTE2, cls, id,
                                 (uint8_t)(payload.size() & 0xFF), (uint8_t)(payload.size() >> 8) };
    pkt.insert(pkt.end(), payload.begin(), payload.end());
    uint16_t ck = is_comm_fletcher16(0, pkt.data() + 2, (uint32_t)(pkt.size() - 2));
    uint8_t ckb[2];
    memcpy(ckb, &ck, sizeof(ck));       // same byte order the parser reads it back in
    pkt.push_back(ckb[0]);
    pkt.push_back(ckb[1]);
    return pkt;
}

} // namespace

TEST(ISComm, UbxOversizedLengthIsRejectedAndParserResyncs)
{
    uint8_t buf[PKT_BUF_SIZE];
    is_comm_instance_t comm;
    is_comm_init(&comm, buf, sizeof(buf), NULL);
    comm.rxErrorState = 0;      // count the first parse error too

    // Corrupt header claiming a 16 KB payload (larger than the RX buffer), immediately followed by a valid packet.
    std::vector<uint8_t> stream = { UBLOX_START_BYTE1, UBLOX_START_BYTE2, 0x01, 0x07, 0x00, 0x40 };
    const std::vector<uint8_t> valid = makeUbx(0x01, 0x02, { 1, 2, 3, 4, 5, 6, 7, 8 });
    stream.insert(stream.end(), valid.begin(), valid.end());

    int ubxCount = 0;
    for (uint8_t b : stream)
    {
        if (is_comm_parse_byte(&comm, b) == _PTYPE_UBLOX)
        {
            ubxCount++;
            EXPECT_EQ(comm.rxPkt.id, 0x01);     // the parser reports the UBX class as the id
            EXPECT_EQ(comm.rxPkt.data.size, valid.size());
        }
    }

    EXPECT_EQ(ubxCount, 1) << "valid UBX packet after a corrupt length header was not parsed";
    EXPECT_GE(comm.rxErrorTypeCount[EPARSE_INVALID_SIZE], 1u);
}

TEST(ISComm, UbxMaxLengthDoesNotWrapParserSize)
{
    uint8_t buf[PKT_BUF_SIZE];
    is_comm_instance_t comm;
    is_comm_init(&comm, buf, sizeof(buf), NULL);
    comm.rxErrorState = 0;

    // 6 + 0xFFFF + 2 does not fit the parser's 16-bit size; must be rejected as a size error.
    const uint8_t hdr[] = { UBLOX_START_BYTE1, UBLOX_START_BYTE2, 0x01, 0x07, 0xFF, 0xFF };
    for (uint8_t b : hdr)
        is_comm_parse_byte(&comm, b);
    EXPECT_EQ(comm.rxErrorTypeCount[EPARSE_INVALID_SIZE], 1u);
}

TEST(ISComm, UbxLargeValidPacketStillParses)
{
    uint8_t buf[PKT_BUF_SIZE];
    is_comm_instance_t comm;
    is_comm_init(&comm, buf, sizeof(buf), NULL);

    // Larger than the old (unused) 1024-byte MAX_MSG_LENGTH_UBX, but fits the RX buffer.
    const std::vector<uint8_t> payload(1500, 0x5A);
    const std::vector<uint8_t> pkt = makeUbx(0x02, 0x15, payload);

    int ubxCount = 0;
    for (uint8_t b : pkt)
        ubxCount += (is_comm_parse_byte(&comm, b) == _PTYPE_UBLOX);
    EXPECT_EQ(ubxCount, 1);
}
