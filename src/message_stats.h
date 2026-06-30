/*
MIT LICENSE

Copyright (c) 2014-2025 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#ifndef __GPS_STATS_H__
#define __GPS_STATS_H__

#include <string>
#include <map>
#include <vector>

#include "ISComm.h"

class MessageStats
{
public:
    // Nested types
    struct stats_t
    {
        int count;
        uint64_t timeMs;
        uint64_t prevTimeMs;
        uint64_t bytes;
        uint64_t startTimeMs;
        int bytesPerSec;
        std::string description;
    };

    struct mul_stats_t
    {
        std::map<int, stats_t> isb;
        std::map<int, stats_t> nmea;
        std::map<int, stats_t> ublox;
        std::map<int, stats_t> rtcm3;
        std::map<int, stats_t> sept_sbf;
        std::map<int, stats_t> sept_reply;
        stats_t ack;
        stats_t parseError;
    };
    // Static functions (can be called without an instance)
    static void append(const std::string& message, mul_stats_t &msgStats, unsigned int ptype, int id, int bytes, int timeMs);
    static std::string summary(mul_stats_t &msgStats);
    
    // Instance methods
    int  processData(void* ctx, protocol_type_t ptype, packet_t *pkt, port_handle_t port);
    void processISB(p_data_t *data);
    void processASCII(const uint8_t *msg, int msgSize);
    void processUblox(const uint8_t* msg, int msgSize);
    void processRTCM3(const uint8_t* msg, int msgSize);
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

    std::vector<std::string>    history;
    bool                        historyPaused;
    mul_stats_t                 stats = {};
    bool                        update;

private:
    // Private instance methods
    void historyWrite(const std::string& message, int ptype, int id, int bytes, int timeMs);

    // Private helper functions
    static std::string descriptionUblox(uint8_t msgClass, uint8_t msgID);
    static std::string descriptionRtcm3(int id);
    static stats_t createNewMsgStats(int timeMs, const std::string& description = "");
    static void updateTimeMs(stats_t &s, int timeMs, int bytes);
    static std::string getCurrentTimeString();
};

#endif // __GPS_STATS_H__
