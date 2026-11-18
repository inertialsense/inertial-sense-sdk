/**
 * @file test_CorrectionService.cpp
 * @brief SN-9075 (GitHub #1379): RTCM3 message 1029 (text) handling in CorrectionService.
 *
 * Both 1029 paths hand the text after a 12-byte offset to the registered listeners:
 *  - CorrectionService::onRtcm3Handler(), reached when the source port's parser delivers an RTCM3 packet, and
 *  - CorrectionService::finalPacketFilter(), used by services that transform the raw stream.
 * A framed RTCM3 message can be as short as 8 bytes (a 2-byte payload that holds only the message number), so frames
 * shorter than the 12-byte offset are valid input from a network correction stream. The short-frame cases run as death
 * tests that must exit normally, so a crash fails the test instead of taking down the whole test binary.
 */
#if !defined(_WIN32)

#include <gtest/gtest.h>

#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <chrono>
#include <cstdlib>
#include <string>
#include <thread>
#include <vector>

// onRtcm3Handler(), finalPacketFilter() and the packet counters are private; the tests drive them the way step() and the raw-data handler do.
#define private public
#include "CorrectionService.h"
#undef private
#include "TcpPortFactory.h"
#include "ntrip_test_helpers.h"
#include "tcp_test_helpers.h"

namespace {

constexpr uint16_t MSG_TEXT = 1029;
constexpr uint32_t TEXT_OFFSET = 12;    //!< offset of the text within a framed 1029 message, as used by CorrectionService
constexpr uint32_t FRAME_OVERHEAD = 6;  //!< 3-byte RTCM3 header + 3-byte CRC around the payload

/** @return a framed 1029 message whose text field holds @p text (header bytes between the message number and the text are zero). */
std::vector<uint8_t> buildTextFrame(const std::string& text) {
    std::vector<uint8_t> frame = ntrip_test::buildRtcm3Frame(MSG_TEXT, static_cast<uint16_t>(9 + text.size()));
    frame.resize(frame.size() - 3);  // drop the CRC; it is recomputed below
    for (size_t i = 0; i < 9 + text.size(); ++i)
        if (i >= 2) frame[3 + i] = (i < 9) ? 0 : static_cast<uint8_t>(text[i - 9]);
    const unsigned int crc = calculate24BitCRCQ(frame.data(), static_cast<unsigned int>(frame.size()));
    frame.push_back(static_cast<uint8_t>((crc >> 16) & 0xFF));
    frame.push_back(static_cast<uint8_t>((crc >> 8) & 0xFF));
    frame.push_back(static_cast<uint8_t>(crc & 0xFF));
    return frame;
}

/** @return a framed 1029 message that is @p frameSize bytes long in total (8 or more). */
std::vector<uint8_t> buildFrameOfSize(uint32_t frameSize) { return ntrip_test::buildRtcm3Frame(MSG_TEXT, static_cast<uint16_t>(frameSize - FRAME_OVERHEAD)); }

/**
 * Feeds @p bytes through finalPacketFilter() and returns the 1029 notices delivered to a listener.
 * Nothing in the SDK initialises CorrectionService::packetParser (the base packetTransformer() is a stub that returns 0, so the filter never
 * receives data), so the test does what a subclass that implements packetTransformer() would have to do: give the parser a buffer.
 */
std::vector<std::string> runFinalPacketFilter(const std::vector<uint8_t>& bytes, int* packets = nullptr) {
    std::vector<std::string> notices;
    CorrectionService service;
    std::vector<uint8_t> parserBuffer(8192);
    is_comm_init(&service.packetParser, parserBuffer.data(), static_cast<uint32_t>(parserBuffer.size()), nullptr);
    service.addRTCM3Msg1029Listeners([&notices](std::string s) { notices.push_back(std::move(s)); });
    std::vector<uint8_t> out(8192);
    uint32_t processed = 0;
    const int n = service.finalPacketFilter(bytes.data(), static_cast<uint32_t>(bytes.size()), out.data(), static_cast<uint32_t>(out.size()), &processed);
    if (packets) *packets = n;
    return notices;
}

/** Sends @p bytes to a CorrectionService whose source is a loopback TCP port, steps it until @p expectPackets RTCM3 packets were handled, and returns the 1029 notices. */
std::vector<std::string> runSourceHandler(const std::vector<uint8_t>& bytes, unsigned expectPackets) {
    std::vector<std::string> notices;
    tcp_test::Listener listener;
    port_handle_t port = TcpPortFactory::getInstance().bindPort(listener.url(), PORT_TYPE__TCP | PORT_TYPE__COMM);
    EXPECT_NE(nullptr, port);
    int peer = -1;
    {
        CorrectionService service(port);
        service.addRTCM3Msg1029Listeners([&notices](std::string s) { notices.push_back(std::move(s)); });

        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
        while (!portIsOpened(port) && std::chrono::steady_clock::now() < deadline) {
            portOpen(port);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        peer = listener.acceptPeer();
        EXPECT_GE(peer, 0);
        if (peer >= 0) {
            EXPECT_EQ(static_cast<ssize_t>(bytes.size()), ::send(peer, bytes.data(), bytes.size(), 0));
            while (service.rtcm3PacketsProcessed < expectPackets && std::chrono::steady_clock::now() < deadline) {
                service.step();
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
            }
        }
    }
    if (peer >= 0) close(peer);
    TcpPortFactory::getInstance().releasePort(port);
    return notices;
}

}  // namespace

// ---- finalPacketFilter() path ------------------------------------------------------------------------------------------------------------------

TEST(test_CorrectionService, finalPacketFilter_ShortFrame_NoNotice) {
    for (uint32_t size = 8; size < TEXT_OFFSET; ++size) {
        EXPECT_EXIT({ std::exit(runFinalPacketFilter(buildFrameOfSize(size)).empty() ? 0 : 3); }, ::testing::ExitedWithCode(0), "") << "frame size " << size;
    }
}

TEST(test_CorrectionService, finalPacketFilter_ValidFrame_DeliversText) {
    const std::vector<uint8_t> frame = buildTextFrame("HELLO");
    ASSERT_GE(frame.size(), TEXT_OFFSET);
    const std::vector<std::string> notices = runFinalPacketFilter(frame);
    ASSERT_EQ(1u, notices.size());
    EXPECT_EQ(frame.size() - TEXT_OFFSET, notices[0].size());  // the notice is everything after the 12-byte offset
    EXPECT_EQ(0u, notices[0].rfind("HELLO", 0));
}

TEST(test_CorrectionService, finalPacketFilter_FrameAtOffset_DeliversEmptyNotice) {
    const std::vector<std::string> notices = runFinalPacketFilter(buildFrameOfSize(TEXT_OFFSET));
    ASSERT_EQ(1u, notices.size());
    EXPECT_EQ(0u, notices[0].size());
}

TEST(test_CorrectionService, finalPacketFilter_LargestFrame_DeliversText) {
    // The parser rejects RTCM3 frames longer than MAX_MSG_SIZE_RTCM, so the "size < 1024" upper bound in CorrectionService can never be reached.
    const std::vector<std::string> notices = runFinalPacketFilter(buildFrameOfSize(MAX_MSG_SIZE_RTCM));
    ASSERT_EQ(1u, notices.size());
    EXPECT_EQ(MAX_MSG_SIZE_RTCM - TEXT_OFFSET, notices[0].size());
}

// ---- onRtcm3Handler() path (source port parser) ------------------------------------------------------------------------------------------------

TEST(test_CorrectionService, onRtcm3Handler_ShortFrame_NoNotice) {
    for (uint32_t size = 8; size < TEXT_OFFSET; ++size) {
        EXPECT_EXIT({ std::exit(runSourceHandler(buildFrameOfSize(size), 1).empty() ? 0 : 3); }, ::testing::ExitedWithCode(0), "") << "frame size " << size;
    }
}

TEST(test_CorrectionService, onRtcm3Handler_ValidFrame_DeliversText) {
    const std::vector<uint8_t> frame = buildTextFrame("HELLO");
    const std::vector<std::string> notices = runSourceHandler(frame, 1);
    ASSERT_EQ(1u, notices.size());
    EXPECT_EQ(frame.size() - TEXT_OFFSET, notices[0].size());
    EXPECT_EQ(0u, notices[0].rfind("HELLO", 0));
}

TEST(test_CorrectionService, onRtcm3Handler_FrameAtOffset_DeliversEmptyNotice) {
    const std::vector<std::string> notices = runSourceHandler(buildFrameOfSize(TEXT_OFFSET), 1);
    ASSERT_EQ(1u, notices.size());
    EXPECT_EQ(0u, notices[0].size());
}

TEST(test_CorrectionService, onRtcm3Handler_LargestFrame_DeliversText) {
    const std::vector<std::string> notices = runSourceHandler(buildFrameOfSize(MAX_MSG_SIZE_RTCM), 1);
    ASSERT_EQ(1u, notices.size());
    EXPECT_EQ(MAX_MSG_SIZE_RTCM - TEXT_OFFSET, notices[0].size());
}

// ---- mixed stream (functional) -----------------------------------------------------------------------------------------------------------------

TEST(test_CorrectionService, onRtcm3Handler_TruncatedAndValidMix_ServiceSurvives) {
    std::vector<uint8_t> stream;
    auto append = [&stream](const std::vector<uint8_t>& f) { stream.insert(stream.end(), f.begin(), f.end()); };
    append(buildTextFrame("FIRST"));
    append(buildFrameOfSize(8));                    // truncated 1029: must not take the service down
    append(ntrip_test::buildRtcm3Frame(1005, 19));  // unrelated message in between
    append(buildFrameOfSize(9));
    append(buildTextFrame("LAST"));

    std::vector<std::string> notices;
    EXPECT_EXIT(
        {
            notices = runSourceHandler(stream, 5);
            std::exit(notices.size() == 2 ? 0 : 3);
        },
        ::testing::ExitedWithCode(0), "");
}

#endif  // !defined(_WIN32)
