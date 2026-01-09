/*
MIT LICENSE

Copyright (c) 2014-2025 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include <map>
#include <chrono>
#include <ctime>

#include "ISComm.h"
#include "ISDataMappings.h"
#include "ISUtilities.h"
#include "message_stats.h"

using namespace std;


string MessageStats::descriptionUblox(uint8_t msgClass, uint8_t msgID)
{
    string s;
    switch (msgClass)
    {
        case 0x01:    
            s.append("UBX-NAV");
            switch (msgID)
            {
                case 0x01:    s.append("-POSECEF"); break;
                case 0x02:    s.append("-POSLLH");  break;
            }
            break;

        case 0x02:    
            s.append("UBX-RXM");
            switch (msgID)
            {
                case 0x13:    s.append("-SFRBX");   break;
                case 0x15:    s.append("-RAWX");    break;
                case 0x32:    s.append("-RTCM");    break;
            }
            break;
        case 0x06:    s.append("UBX-CFG");          break;
        case 0x0D:    s.append("UBX-TIM");          break;
    }

    return s;
}

string MessageStats::descriptionRtcm3(int id)
{
    // RTCM3 descriptions: https://www.use-snip.com/kb/knowledge-base/rtcm-3-message-list/
    switch (id)
    {
        case 1002:    return string("Extended L1-Only GPS RTK Observables");
        case 1004:    return string("Extended L1&L2 GPS RTK Observables");
        case 1005:    return string("Stationary RTK reference station ARP");
        case 1006:    return string("Reference RTK Station Antenna Position (ECEF)");
        case 1007:    return string("Antenna Descriptor");
        case 1010:    return string("Extended L1-Only GLONASS RTK Observables");
        case 1012:    return string("Extended L1&L2 GLONASS RTK Observables");
        case 1019:    return string("GPS ephemerides");
        case 1020:    return string("GLONASS ephemerides");
        case 1023:    return string("Residuals, Ellipsoidal Grid Representation");
        case 1029:    return string("Text");
        case 1030:    return string("GPS Network RTK Residual");
        case 1031:    return string("GLONASS Network RTK Residual");
        case 1032:    return string("Physical Reference Station Position");
        case 1033:    return string("Receiver and Antenna Descriptors");
        case 1045:    return string("Galileo ephemerides");
        case 1074:    return string("GPS MSM4");
        case 1075:    return string("GPS MSM5");
        case 1077:    return string("GPS MSM7");
        case 1084:    return string("GLONASS MSM4");
        case 1085:    return string("GLONASS MSM5");
        case 1087:    return string("GLONASS MSM7");
        case 1094:    return string("Galileo MSM4");
        case 1095:    return string("Galileo MSM5");
        case 1097:    return string("Galileo MSM7");
        case 1124:    return string("BeiDou MSM4");
        case 1127:    return string("BeiDou MSM7");
        case 1230:    return string("GLONASS code-phase biases");
        case 4094:    return string("Assigned to : Trimble Navigation Ltd.");
    }

    return "";
}

MessageStats::stats_t MessageStats::createNewMsgStats(int timeMs, const string& description)
{
    MessageStats::stats_t s = {};
    s.prevTimeMs = s.timeMs = timeMs;
    s.description = description;
    return s;
}

void MessageStats::updateTimeMs(MessageStats::stats_t &s, int timeMs, int bytes)
{
    s.count++;
    s.prevTimeMs = s.timeMs;
    s.timeMs = timeMs;

    // Compute data rate (Bytes/s)
    s.bytes += bytes;
    if (s.startTimeMs == 0)
    {   // Initialize time
        s.startTimeMs = timeMs;
    }
    else
    {
        uint32_t dtMs = timeMs - s.startTimeMs;
        if (dtMs >= 1000)
        {   // Update ever second
            s.bytesPerSec = (1000 * s.bytes) / dtMs;
            s.bytes = 0;
            s.startTimeMs = timeMs;
        }
    }
}

void MessageStats::append(const string& message, MessageStats::mul_stats_t &msgStats, unsigned int ptype, int id, int bytes, int timeMs)
{
    switch (ptype)
    {
    case _PTYPE_INERTIAL_SENSE_CMD:
    case _PTYPE_INERTIAL_SENSE_DATA:
        if (msgStats.isb.find(id) == msgStats.isb.end())
        {   // Create new 
            msgStats.isb[id] = createNewMsgStats(timeMs, cISDataMappings::DataName(id));
        }

        {   // Update count and timestamps
            updateTimeMs(msgStats.isb[id], timeMs, bytes);
        }
        break;

    case _PTYPE_NMEA:
        if (msgStats.nmea.find(id) == msgStats.nmea.end())
        {   // Create new 
            msgStats.nmea[id] = createNewMsgStats(timeMs);
        }

        {   // Update count and timestamps
            updateTimeMs(msgStats.nmea[id], timeMs, bytes);
        }
        break;

    case _PTYPE_UBLOX:
        if (msgStats.ublox.find(id) == msgStats.ublox.end())
        {   // Create new 
            uint8_t msgClass = (uint8_t)id;
            uint8_t msgID = (uint8_t)(id >> 8);
            msgStats.ublox[id] = createNewMsgStats(timeMs, descriptionUblox(msgClass, msgID));
        }

        {   // Update count and timestamps
            updateTimeMs(msgStats.ublox[id], timeMs, bytes);
        }
        break;

    case _PTYPE_RTCM3:
        if (msgStats.rtcm3.find(id) == msgStats.rtcm3.end())
        {   // Create new 
            msgStats.rtcm3[id] = createNewMsgStats(timeMs, descriptionRtcm3(id));
        }

        {   // Update count and timestamps
            MessageStats::stats_t &s = msgStats.rtcm3[id];
            updateTimeMs(s, timeMs, bytes);

            if (id == 1029)
            {
                s.description = string("Text String: ") + message;
            }
        }
        break;

    case _PTYPE_INERTIAL_SENSE_ACK:
        {
            updateTimeMs(msgStats.ack, timeMs, bytes);
        }
        break;

    default:
    case _PTYPE_PARSE_ERROR:
        {
            updateTimeMs(msgStats.parseError, timeMs, bytes);
        }
        break;
    }
}

void MessageStats::historyWrite(const std::string& message, int ptype, int id, int bytes, int timeMs)
{
    // Limit number of lines in history
    while (history.size() > 300)
    {
        history.erase(history.begin());
    }
    history.push_back(message);

    append(message, stats, ptype, id, bytes, timeMs);
    update = true;
}

string MessageStats::summary(MessageStats::mul_stats_t &msgStats)
{
    string str;
#define BUF_SIZE 512
    char buf[BUF_SIZE];

    if (!msgStats.isb.empty())
    {
        str.append("Inertial Sense Binary: __________________\n");
        str.append(" DID   Count  dtMs   Bps  Description\n");
        for (auto it = msgStats.isb.begin(); it != msgStats.isb.end(); it++)
        {
            int did = it->first;
            MessageStats::stats_t &s = it->second;
            int dtMs = (s.prevTimeMs ? (s.timeMs - s.prevTimeMs) : 0);

            SNPRINTF(buf, BUF_SIZE, "%4d %7d %5d %5d  %s\n", did, s.count, dtMs, s.bytesPerSec, s.description.c_str());
            str.append(string(buf));
        }
    }

    if (!msgStats.nmea.empty())
    {
        str.append("NMEA: __________________________________\n");
        str.append("  ID   Count  dtMs   Bps  Description\n");
        for (auto it = msgStats.nmea.begin(); it != msgStats.nmea.end(); it++)
        {
            MessageStats::stats_t &s = it->second;
            int dtMs = (s.prevTimeMs ? (s.timeMs - s.prevTimeMs) : 0);
            union
            {
                int id;
                char str[8];
            } val = {};
            val.id = it->first;
            SNPRINTF(buf, BUF_SIZE, "%4s %7d %5d %5d  %s\n", val.str, s.count, dtMs, s.bytesPerSec, s.description.c_str());
            str.append(string(buf));
        }
    }

    if (!msgStats.ublox.empty())
    {
        str.append("Ublox: __________________________________\n");
        str.append("(Class  ID)   Count  dtMs   Bps  Description\n");
        for (auto it = msgStats.ublox.begin(); it != msgStats.ublox.end(); it++)
        {
            int id = it->first;
            MessageStats::stats_t &s = it->second;
            uint8_t msgClass = (uint8_t)id;
            uint8_t msgID = (uint8_t)(id >> 8);
            int dtMs = (s.prevTimeMs ? (s.timeMs - s.prevTimeMs) : 0);
            SNPRINTF(buf, BUF_SIZE, "(0x%02x 0x%02x) %7d %5d %5d  %s\n", msgClass, msgID, s.count, dtMs, s.bytesPerSec, s.description.c_str());
            str.append(string(buf));
        }
    }

    if (!msgStats.rtcm3.empty())
    {
        str.append("RTCM3: __________________________________\n");
        str.append("  ID   Count  dtMs   Bps  Description\n");
        for (auto it = msgStats.rtcm3.begin(); it != msgStats.rtcm3.end(); it++)
        {
            int id = it->first;
            MessageStats::stats_t &s = it->second;
            int dtMs = (s.prevTimeMs ? (s.timeMs - s.prevTimeMs) : 0);
            SNPRINTF(buf, BUF_SIZE, "%3d %7d %5d %5d  %s\n", id, s.count, dtMs, s.bytesPerSec, s.description.c_str());
            str.append(string(buf));
        }
    }

    if (msgStats.ack.count>5)
    {
        str.append("Acknowledge: ____________________________\n");
        str.append("   Count  dtMs   Bps\n");
        MessageStats::stats_t &s = msgStats.ack;
        int dtMs = (s.prevTimeMs ? (s.timeMs - s.prevTimeMs) : 0);
        SNPRINTF(buf, BUF_SIZE, "%8d %5d %5d\n", s.count, dtMs, s.bytesPerSec);
        str.append(string(buf));
    }

#ifdef DEBUG
    if (msgStats.parseError.count>5)
    {
        str.append("Parse Error: ____________________________\n");
        str.append("   Count   dtMs   Bps\n");
        MessageStats::stats_t &s = msgStats.parseError;
        int dtMs = (s.prevTimeMs ? (s.timeMs - s.prevTimeMs) : 0);
        SNPRINTF(buf, BUF_SIZE, "%8d %5d\n", s.count, dtMs, s.bytesPerSec);
        str.append(string(buf));
    }
#endif
    return str;
}


int MessageStats::processData(void* ctx, protocol_type_t ptype, packet_t *pkt, port_handle_t port)
{
    (void)ctx; // ctx is currently unused but kept for interface compatibility
    (void)ctx; // ctx is currently unused but kept for interface compatibility
    
    switch( ptype )
    {
        case _PTYPE_INERTIAL_SENSE_DATA:
        case _PTYPE_INERTIAL_SENSE_CMD:
        case _PTYPE_INERTIAL_SENSE_ACK:
        {
            p_data_t data;
            data.hdr = pkt->dataHdr;
            data.ptr = pkt->data.ptr;
            processISB(&data);
            break;
        }

        case _PTYPE_NMEA:   processASCII(pkt->data.ptr, pkt->size); break;
        case _PTYPE_UBLOX:  processUblox(pkt->data.ptr, pkt->size); break;
        case _PTYPE_RTCM3:  processRTCM3(pkt->data.ptr, pkt->size); break;

        default:
            break;
    }

    return 0;
}

std::string MessageStats::getCurrentTimeString()
{
    auto now = std::chrono::system_clock::now();
    auto now_time_t = std::chrono::system_clock::to_time_t(now);
    auto now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;
    
    std::tm tm;
#ifdef _WIN32
    localtime_s(&tm, &now_time_t);
#else
    localtime_r(&now_time_t, &tm);
#endif
    
    char buf[32];
    snprintf(buf, sizeof(buf), "[%02d:%02d:%02d.%03d] ", 
             tm.tm_hour, tm.tm_min, tm.tm_sec, (int)now_ms.count());
    return std::string(buf);
}

void MessageStats::processISB(p_data_t *data)
{
    char buf[256];
    snprintf(buf, sizeof(buf), "IS-DID %2d, size %3d, ", data->hdr.id, data->hdr.size);
    std::string strID = buf;
    
    std::string strDescription;
    if (data->hdr.id == DID_DEBUG_STRING)
    {
        int n = (data->hdr.size < DEBUG_STRING_SIZE - 1) ? data->hdr.size : (DEBUG_STRING_SIZE - 1);    // Limit number of bytes copied
        strDescription = std::string((const char*)data->ptr, static_cast<size_t>(n));
    }
    else if (data->hdr.id == DID_DIAGNOSTIC_MESSAGE)
    {
        strDescription = std::string(((diag_msg_t*)data->ptr)->message);
        // Trim whitespace
        size_t start = strDescription.find_first_not_of(" \t\n\r");
        size_t end = strDescription.find_last_not_of(" \t\n\r");
        if (start != std::string::npos && end != std::string::npos)
            strDescription = strDescription.substr(start, end - start + 1);
        else if (start == std::string::npos)
            strDescription = "";
    }
    else
    {
        strDescription = std::string(cISDataMappings::DataName(data->hdr.id));
    }
    std::string str = getCurrentTimeString() + strID + strDescription + "\n";

    historyWrite(str, _PTYPE_INERTIAL_SENSE_DATA, data->hdr.id, ISB_HDR_TO_PACKET_SIZE(data->hdr), current_timeMs());
}

void MessageStats::processASCII(const uint8_t *msg, int msgSize)
{
    std::string s;
    for (int i = 0; i < msgSize; i++)
    {
        s += msg[i];
    }
    std::string str = getCurrentTimeString() + s;

    // Use a 4-character NMEA sentence identifier: the last four characters before the first comma
    // (e.g. "PGGA" from "$GPGGA,..."). If there are fewer than 4 characters before the comma,
    // fall back to the 4 characters starting after the '$'.
    int id = 0;
    const char* comma_pos = strchr((const char*)msg, ',');
    if (!comma_pos || (comma_pos - (const char*)msg) < 5)
    {   // Not enough characters before the first comma
        if (msgSize >= 5)
        {
            memcpy(&id, msg+1, 4);
        }
    }
    else
    {
        memcpy(&id, comma_pos-4, 4);
    }

    historyWrite(str, _PTYPE_NMEA, id, msgSize, current_timeMs());
}

void MessageStats::processUblox(const uint8_t* msg, int msgSize)
{
    if (msg == nullptr || msgSize < 4)
    {   // Not enough data to safely read UBX message class and ID
        return; 
    }

    uint8_t msgClass = msg[2];
    uint8_t msgID = msg[3];
    char buf[64];
    snprintf(buf, sizeof(buf), "(0x%02x 0x%02x) UBX", msgClass, msgID);
    std::string strIDs = buf;
    std::string str = getCurrentTimeString() + strIDs + descriptionUblox(msgClass, msgID) + "\n";
    int id = *((uint16_t*)(&msg[2]));

    historyWrite(str, _PTYPE_UBLOX, id, msgSize, current_timeMs());
}

void MessageStats::processRTCM3(const uint8_t* msg, int msgSize)
{
    int id = RTCM3_MSG_ID(msg);
    char buf[64];
    snprintf(buf, sizeof(buf), "RTCM3: %d ", id);
    std::string strID = buf;
    std::string str = getCurrentTimeString() + strID + descriptionRtcm3(id) + "\n";

    historyWrite(str, _PTYPE_RTCM3, id, msgSize, current_timeMs());
}
