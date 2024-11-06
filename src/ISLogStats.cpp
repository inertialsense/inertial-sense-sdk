/*
MIT LICENSE

Copyright (c) 2014-2024 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>

#include "ISDataMappings.h"
#include "ISLogFile.h"
#include "ISLogFileFactory.h"
#include "ISLogStats.h"
#include "ISUtilities.h"
#include "protocol_nmea.h"

using namespace std;


cLogStatMsgId::cLogStatMsgId()
{
    count = 0;
    errors = 0;
    averageTimeDelta = 0.0;
    totalTimeDelta = 0.0;
    lastTimestamp = 0.0;
    lastTimestampDelta = 0.0;
    maxTimestampDelta = 0.0;
    minTimestampDelta = 1.0E6;
    timestampDeltaCount = 0;
    timestampDropCount = 0;
}

void cLogStatMsgId::LogTimestamp(double timestamp)
{
    // check for corrupt data
    if (_ISNAN(timestamp) || timestamp < 0.0 || timestamp > 999999999999.0)
    {
        return;
    }
    else if (lastTimestamp > 0.0)
    {
        double delta = fabs(timestamp - lastTimestamp);
        minTimestampDelta = _MIN(delta, minTimestampDelta);
        maxTimestampDelta = _MAX(delta, maxTimestampDelta);
        totalTimeDelta += delta;
        averageTimeDelta = (totalTimeDelta / (double)++timestampDeltaCount);
        if (lastTimestampDelta != 0.0 && (fabs(delta - lastTimestampDelta) > (lastTimestampDelta * 0.5)))
        {
            timestampDropCount++;
        }
        lastTimestampDelta = delta;
    }
    lastTimestamp = timestamp;
}

void cLogStatMsgId::PrintStats()
{

#if !PLATFORM_IS_EMBEDDED

    printf(" Count: %llu,   Errors: %llu\r\n", (unsigned long long)count, (unsigned long long)errors);
    printf(" Time delta: (ave, min, max) %f, %f, %f\r\n", averageTimeDelta, minTimestampDelta, maxTimestampDelta);
    printf(" Time delta drop: %llu\r\n", (unsigned long long)timestampDropCount);

#endif

}

cLogStats::cLogStats()
{
    Clear();
}

void cLogStats::Clear()
{
    for (auto& [ptype, msg] : msgs) 
    {
        msg.count = 0;
        msg.errors = 0;
        for (uint32_t id=0; id < DID_COUNT; id++)
        {
            msg.stats[id] = {};
            msg.stats[id].minTimestampDelta = 1.0E6;
        }
    }
}

void cLogStats::IsbLogError(const p_data_hdr_t* hdr)
{
    sLogStatPType &msg = msgs[_PTYPE_INERTIAL_SENSE_DATA];

    msg.errors++;
    if (hdr != NULL && hdr->id < DID_COUNT)
    {
        cLogStatMsgId& d = msg.stats[hdr->id];
        d.errors++;
    }
}

void cLogStats::LogDataRealtime(protocol_type_t ptype, int id, double timestamp)
{
    if (timestamp == 0.0)
    {
        timestamp = current_timeSecD();
    }
    LogData(ptype, id, timestamp);
}

void cLogStats::LogData(protocol_type_t ptype, int id, double timestamp)
{
    sLogStatPType &msg = msgs[ptype];
    cLogStatMsgId &d = msg.stats[id];
    msg.count++;
    d.count++;

    if (timestamp != 0.0)
    {   // Use system time
        msgs[ptype].stats[id].LogTimestamp(timestamp);
    }
}

void cLogStats::PrintStats()
{

#if 1
    cout << Stats();

#else

#if !PLATFORM_IS_EMBEDDED

    printf("Count: %llu,   Errors: %llu\n", (unsigned long long)count, (unsigned long long)errors);
    for (auto it = isbStats.begin(); it != isbStats.end(); ++it) 
    {
        int id = it->first;
        cLogStatMsgId &d = it->second;
        if (d.count != 0)
        {
            printf(" ISB: %d\n", id);
            d.PrintStats();
            printf("\n");
        }
    }

#endif

#endif

}

string cLogStats::MessageStats(protocol_type_t ptype, sLogStatPType &msg, bool showDeltaTime)
{
    std::stringstream ss;

    string msgName;
    int colWidName = 24;
    switch (ptype)
    {
    default:                                msgName = "UNKNOWN";    break;
    case _PTYPE_INERTIAL_SENSE_CMD:         msgName = "ISB-CMD";    break;
    case _PTYPE_INERTIAL_SENSE_DATA:        msgName = "ISB";        break;
    case _PTYPE_NMEA:                       msgName = "NMEA";       colWidName = 8;      break;
    case _PTYPE_UBLOX:                      msgName = "UBLOX";      colWidName = 8;      break;
    case _PTYPE_RTCM3:                      msgName = "RTCM3";      break;
    case _PTYPE_SPARTN:                     msgName = "SPARTN";     break;
    case _PTYPE_SONY:                       msgName = "SONY";       break;
    }

    ss << std::endl;
    ss << msgName << ": count " << msg.count << ", errors " << msg.errors << std::endl;

    ss << " ID " << std::setw(colWidName) << std::left << "Name" << std::right << " Count Errors  dtMs( ave, min, max)   drops" << std::endl;

    for (auto& [id, stat] : msg.stats)
    {
        if (stat.count == 0 && stat.errors == 0)
        {   // Exclude zero count stats
            continue;
        }

        // Don't print using stringstream as it causes the system to hang on the EVB.
        switch (ptype)
        {
        case _PTYPE_INERTIAL_SENSE_CMD:
        case _PTYPE_INERTIAL_SENSE_DATA:
            ss << std::setw(3) << std::right << id << " " << std::setw(colWidName) << std::left << cISDataMappings::DataName(id) << std::right;
            break;

        case _PTYPE_NMEA: {
                char talker[10] = {0};
                nmeaMsgIdToTalker(id, talker, sizeof(talker));
                ss << std::setw(3) << ::right << id << " " << std::setw(colWidName) << std::left << talker << std::right;
            }
            break;

        case _PTYPE_UBLOX: {
                uint8_t msgClass = 0xFF & (id>>0);
                uint8_t msgID    = 0xFF & (id>>8);
                ss << "Class ID (0x" << std::hex << msgClass << "0x" << std::hex << msgID << ")" << std::hex << std::endl;
            }
            break;

        default:
            ss << std::setw(3) << id;
            break;
        }

        ss << std::setw(6) << std::setfill(' ') << stat.count << std::setw(7) << stat.errors;
        if (showDeltaTime)
        {
#define DT_COL_WIDTH    6
            if (stat.timestampDeltaCount)
            {
                int dtMsAve = int(stat.averageTimeDelta*1000.0);
                int dtMsMin = int(stat.minTimestampDelta*1000.0);
                int dtMsMax = int(stat.maxTimestampDelta*1000.0);
                ss << "  (" 
                   << std::setw(DT_COL_WIDTH) << dtMsAve << "," 
                   << std::setw(DT_COL_WIDTH) << dtMsMin << "," 
                   << std::setw(DT_COL_WIDTH) << dtMsMax << ") " 
                   << std::setw(5) << stat.timestampDropCount;
            }
        }
        ss << std::endl;
    }

    return ss.str();
}

unsigned int cLogStats::Count()
{
    unsigned int count = 0;
    for (auto& [ptype, msg] : msgs) 
    {
        count  += msg.count;
    }
    return count;
}

unsigned int cLogStats::Errors()
{
    unsigned int errors = 0;
    for (auto& [ptype, msg] : msgs) 
    {
        errors += msg.errors;
    }
    return errors;
}

string cLogStats::Stats()
{
    std::stringstream ss;
    unsigned int count = Count();
    unsigned int errors = Errors();
    ss << "Total: count " << count << ", errors " << errors << std::endl;
    for (auto& [ptype, msg] : msgs) 
    {
        ss << MessageStats(ptype, msg);
    }
    return ss.str();
}

void cLogStats::WriteToFile(const string& file_name)
{
    unsigned int count = Count();
    // unsigned int errors = Errors();
    if (count != 0)
    {   // Write log stats to disk
        statsFile = CreateISLogFile(file_name, "wb");
        statsFile->lprintf("%s", Stats().c_str());
        CloseISLogFile(statsFile);
    }
}
