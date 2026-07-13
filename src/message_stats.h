/*
MIT LICENSE

Copyright (c) 2014-2025 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

/**
 * @file message_stats.h
 * @brief Per-message-type throughput and error statistics for all supported protocols.
 *
 * MessageStats accumulates packet counts, byte counts, and timestamps for every
 * protocol type parsed by the SDK (ISB, NMEA, u-blox, RTCM3, Septentrio).
 * It is used by EvalTool and cltool to display real-time throughput and diagnostics.
 *
 * @author Inertial Sense, Inc.
 * @copyright Copyright (c) 2014-2025 Inertial Sense, Inc. - http://inertialsense.com
 */

#ifndef __GPS_STATS_H__
#define __GPS_STATS_H__

#include <string>
#include <map>
#include <vector>

#include "ISComm.h"

/**
 * @brief Accumulates per-message throughput statistics across all supported protocols.
 *
 * Maintains running counts and bandwidth figures for ISB, NMEA, u-blox, RTCM3, and
 * Septentrio messages. The static helpers (@ref append, @ref summary) can be used
 * without an instance when only aggregate reporting is needed.
 */
class MessageStats
{
public:
    /** @brief Per-message-ID statistics snapshot. */
    struct stats_t
    {
        int count = 0;              //!< Total packet count since tracking began
        uint64_t timeMs = 0;        //!< Timestamp of the most recent packet (ms)
        uint64_t prevTimeMs = 0;    //!< Timestamp of the previous packet (ms)
        uint64_t bytes = 0;         //!< Total bytes received for this message ID
        uint64_t startTimeMs = 0;   //!< Timestamp when tracking for this ID began (ms)
        int bytesPerSec = 0;        //!< Computed throughput in bytes per second
        std::string description;    //!< Human-readable label for this message type
    };

    /** @brief Statistics grouped by protocol, covering all observed message IDs. */
    struct mul_stats_t
    {
        std::map<int, stats_t> isb;         //!< ISB DID -> stats
        std::map<int, stats_t> nmea;        //!< NMEA sentence type -> stats
        std::map<int, stats_t> ublox;       //!< u-blox message ID -> stats
        std::map<int, stats_t> rtcm3;       //!< RTCM3 message type -> stats
        std::map<int, stats_t> sept_sbf;    //!< Septentrio SBF block ID -> stats
        std::map<int, stats_t> sept_reply;  //!< Septentrio reply message -> stats
        stats_t ack;                        //!< ISB ACK/NACK aggregate statistics
        stats_t parseError;                 //!< Parse-error aggregate statistics
    };

    /**
     * @brief Append a received message to the aggregate statistics table.
     * @param message   Human-readable label for the message (appended to history).
     * @param msgStats  Statistics table to update.
     * @param ptype     Protocol type (see @ref protocol_type_t).
     * @param id        Protocol-specific message identifier.
     * @param bytes     Size of the message in bytes.
     * @param timeMs    Current time in milliseconds.
     */
    static void append(const std::string& message, mul_stats_t &msgStats, unsigned int ptype, int id, int bytes, int timeMs);

    /**
     * @brief Format a human-readable summary string from a statistics table.
     * @param msgStats Statistics table to summarize.
     * @return Multi-line summary string suitable for display or logging.
     */
    static std::string summary(mul_stats_t &msgStats);

    /**
     * @brief Generic packet-handler callback; dispatches to the per-protocol process methods.
     * @param ctx  User context pointer (unused, exists to match @ref pfnIsCommHandler signature).
     * @param ptype Protocol type of the received packet.
     * @param pkt   Pointer to the parsed packet.
     * @param port  Port on which the packet was received.
     * @return 0 if handled, non-zero to continue to the next handler.
     */
    int processData(void* ctx, protocol_type_t ptype, packet_t *pkt, port_handle_t port);

    /**
     * @brief Update statistics for a received ISB data message.
     * @param data Pointer to the ISB data structure containing the DID and payload size.
     */
    void processISB(p_data_t *data);

    /**
     * @brief Update statistics for a received NMEA sentence.
     * @param msg     Pointer to the raw NMEA sentence bytes.
     * @param msgSize Length of the sentence in bytes.
     */
    void processASCII(const uint8_t *msg, int msgSize);

    /**
     * @brief Update statistics for a received u-blox binary message.
     * @param msg     Pointer to the raw u-blox message bytes.
     * @param msgSize Length of the message in bytes.
     */
    void processUblox(const uint8_t* msg, int msgSize);

    /**
     * @brief Update statistics for a received RTCM3 message.
     * @param msg     Pointer to the raw RTCM3 message bytes.
     * @param msgSize Length of the message in bytes.
     */
    void processRTCM3(const uint8_t* msg, int msgSize);

    /** @brief Reset all statistics and history to initial state. */
    void clear()
    {
        stats.isb.clear();
        stats.nmea.clear();
        stats.ublox.clear();
        stats.rtcm3.clear();
        history.clear();
        historyPaused = false;
        update = false;
    }

    std::vector<std::string>    history;        //!< Circular log of recent message-received events
    bool                        historyPaused;  //!< When true, historyWrite() is suppressed
    mul_stats_t                 stats = {};     //!< Live per-protocol statistics
    bool                        update;         //!< Set to true when stats have changed since last display refresh

private:
    void historyWrite(const std::string& message, int ptype, int id, int bytes, int timeMs);

    static std::string descriptionUblox(uint8_t msgClass, uint8_t msgID);
    static std::string descriptionRtcm3(int id);
    static stats_t createNewMsgStats(int timeMs, const std::string& description = "");
    static void updateTimeMs(stats_t &s, int timeMs, int bytes);
    static std::string getCurrentTimeString();
};

#endif // __GPS_STATS_H__
