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
    timestampIrregCount = 0;
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
            timestampIrregCount++;
        }
        lastTimestampDelta = delta;
    }
    lastTimestamp = timestamp;
}

cLogStats::cLogStats()
{
    Clear();
}

void cLogStats::Clear()
{
    for (uint32_t id = 0; id < DID_COUNT; id++)
    {
        isbStats[id] = {};
        isbStats[id].minTimestampDelta = 1.0E6;
    }
    errorCount = 0;
    count = 0;
}

void cLogStats::LogError(const p_data_hdr_t* hdr)
{
    errorCount++;
    if (hdr != NULL && hdr->id < DID_COUNT)
    {
        cLogStatMsgId& d = isbStats[hdr->id];
        d.errors++;
    }
}

cLogStatMsgId* cLogStats::MsgStats(protocol_type_t ptype, uint32_t id)
{
    switch (ptype)
    {
    default: return NULL;
    case _PTYPE_INERTIAL_SENSE_DATA:
    case _PTYPE_INERTIAL_SENSE_CMD:     return &isbStats[id];
    case _PTYPE_NMEA:                   return &nmeaStats[id];
    case _PTYPE_UBLOX:                  return &ubloxStats[id];
    case _PTYPE_RTCM3:                  return &rtcm3Stats[id];
    }
}

void cLogStats::LogData(uint32_t id, protocol_type_t ptype)
{
    cLogStatMsgId *d = MsgStats(ptype, id);
    if (d)
    {
        d->count++;
    }
    count++;
}

void cLogStats::LogDataAndTimestamp(uint32_t id, double timestamp, protocol_type_t ptype)
{
    cLogStatMsgId *d = MsgStats(ptype, id);
    if (d)
    {
        d->count++;
    if (timestamp != 0.0)
        {
            d->LogTimestamp(timestamp);
    }
    }
    count++;
}

void cLogStats::WriteMsgStats(std::map<int, cLogStatMsgId> &msgStats, const char* msgName, protocol_type_t ptype)
    {
    for (auto it = msgStats.begin(); it != msgStats.end(); ++it) 
    {
        uint32_t id = it->first;
        cLogStatMsgId& stat = it->second;
        if (stat.count == 0 && stat.errors == 0)
        {   // Exclude zero count stats
            continue;
        }

        // Don't print using stringstream as it causes the system to hang on the EVB.
        switch (ptype)
        {
        case _PTYPE_INERTIAL_SENSE_CMD:
        case _PTYPE_INERTIAL_SENSE_DATA:
            statsFile->lprintf("%s - ID: %d (%s)\r\n", msgName, id, cISDataMappings::DataName(id));
            break;

        case _PTYPE_NMEA: 
            {
                char talker[10] = {0};
                nmeaMsgIdToTalker(id, talker, sizeof(talker));
                statsFile->lprintf("%s - %s\r\n", msgName, talker);
            }
            break;

        case _PTYPE_UBLOX: 
            {
                uint8_t msgClass = 0xFF & (id>>0);
                uint8_t msgID    = 0xFF & (id>>8);
                statsFile->lprintf("%s - Class ID (0x%02x 0x%02x)\r\n", msgName, msgClass, msgID);
            }
            break;

        default:
            statsFile->lprintf("%s - ID: %d\r\n", msgName, id);
            break;
        }
        statsFile->lprintf("Count: %d,   Errors: %d\r\n", stat.count, stat.errors);
        statsFile->lprintf("Timestamp Delta (ave, min, max): %.4f, %.4f, %.4f\r\n", stat.averageTimeDelta, stat.minTimestampDelta, stat.maxTimestampDelta);
        statsFile->lprintf("Timestamp Drops: %d\r\n", stat.timestampIrregCount);
        statsFile->lprintf("\r\n");
    }
}

void cLogStats::WriteToFile(const string& file_name)
{
    if (count != 0)
    {
        // flush log stats to disk

        WriteMsgStats(isbStats,   "ISB",   _PTYPE_INERTIAL_SENSE_DATA);
        WriteMsgStats(nmeaStats,  "NMEA" , _PTYPE_NMEA);
        WriteMsgStats(ubloxStats, "UBLOX", _PTYPE_UBLOX);
        WriteMsgStats(rtcm3Stats, "RTCM3", _PTYPE_RTCM3);

        CloseISLogFile(statsFile);
    }
}
