/*
MIT LICENSE

Copyright (c) 2014-2023 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include <string>

#include "ISLogFile.h"
#include "ISLogFileFactory.h"
#include "ISDataMappings.h"
#include "ISLogStats.h"

using namespace std;


cLogStatDataId::cLogStatDataId()
{
    count = 0;
    errorCount = 0;
    averageTimeDelta = 0.0;
    totalTimeDelta = 0.0;
    lastTimestamp = 0.0;
    lastTimestampDelta = 0.0;
    maxTimestampDelta = 0.0;
    minTimestampDelta = 1.0E6;
    timestampDeltaCount = 0;
    timestampDropCount = 0;
}

void cLogStatDataId::LogTimestamp(double timestamp)
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

void cLogStatDataId::Printf()
{

#if !PLATFORM_IS_EMBEDDED

    printf(" Count: %llu,   Errors: %llu\r\n", (unsigned long long)count, (unsigned long long)errorCount);
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
    for (uint32_t id = 0; id < DID_COUNT; id++)
    {
        dataIdStats[id] = {};
        dataIdStats[id].minTimestampDelta = 1.0E6;
    }
    errorCount = 0;
    count = 0;
}

void cLogStats::LogError(const p_data_hdr_t* hdr)
{
    errorCount++;
    if (hdr != NULL && hdr->id < DID_COUNT)
    {
        cLogStatDataId& d = dataIdStats[hdr->id];
        d.errorCount++;
    }
}

void cLogStats::LogData(uint32_t dataId)
{
    if (dataId < DID_COUNT)
    {
        cLogStatDataId& d = dataIdStats[dataId];
        d.count++;
        count++;
    }
}

void cLogStats::LogDataAndTimestamp(uint32_t dataId, double timestamp)
{
    if (dataId < DID_COUNT)
    {
        cLogStatDataId& d = dataIdStats[dataId];
        d.count++;
        count++;
        if (timestamp != 0.0)
        {
            d.LogTimestamp(timestamp);
        }
    }
}

void cLogStats::Printf()
{

#if !PLATFORM_IS_EMBEDDED

    printf("LOG STATS\r\n");
    printf("----------");
    printf("Count: %llu,   Errors: %llu\r\n", (unsigned long long)count, (unsigned long long)errorCount);
    for (uint32_t id = 0; id < DID_COUNT; id++)
    {
        if (dataIdStats[id].count != 0)
        {
            printf(" DID: %d\r\n", id);
            dataIdStats[id].Printf();
            printf("\r\n");
        }
    }

#endif

}

void cLogStats::WriteToFile(const string& file_name)
{
    if (count != 0)
    {
        // flush log stats to disk
#if 1
        cISLogFileBase* statsFile = CreateISLogFile(file_name, "wb");
        statsFile->lprintf("Total count: %d,   Total errors: \r\n\r\n", count, errorCount);
        for (uint32_t id = 0; id < DID_COUNT; id++)
        {
            cLogStatDataId& stat = dataIdStats[id];
            if (stat.count == 0 && stat.errorCount == 0)
            {   // Exclude zero count stats
                continue;
            }

            statsFile->lprintf("Data Id: %d (%s)\r\n", id, cISDataMappings::GetDataSetName(id));
            statsFile->lprintf("Count: %d,   Errors: %d\r\n", stat.count, stat.errorCount);
            statsFile->lprintf("Timestamp Delta (ave, min, max): %.4f, %.4f, %.4f\r\n", stat.averageTimeDelta, stat.minTimestampDelta, stat.maxTimestampDelta);
            statsFile->lprintf("Timestamp Drops: %d\r\n", stat.timestampDropCount);
            statsFile->lprintf("\r\n");
        }
        CloseISLogFile(statsFile);
#else
        // The following code doesn't work on the EVB-2.  Converting stringstream to a string causes the system to hang.
        CONST_EXPRESSION char CRLF[] = "\r\n";
        stringstream stats;
        stats << fixed << setprecision(6);
        stats << "Total count: " << count << ",   Total errors: " << errorCount << CRLF << CRLF;
        for (uint32_t id = 0; id < DID_COUNT; id++)
        {
            cLogStatDataId& stat = dataIdStats[id];
            if (stat.count == 0 && stat.errorCount == 0)
            {   // Exclude zero count stats
                continue;
            }

            stats << "Data Id: " << id << " (" << cISDataMappings::GetDataSetName(id) << ")" << CRLF;
            stats << "Count: " << stat.count << ",   Errors: " << stat.errorCount << CRLF;
            stats << "Timestamp Delta (ave, min, max): " << stat.averageTimeDelta << ", " << stat.minTimestampDelta << ", " << stat.maxTimestampDelta << CRLF;
            stats << "Timestamp Drops: " << stat.timestampDropCount << CRLF;
            stats << CRLF;
        }
        cISLogFileBase* statsFile = CreateISLogFile(file_name, "wb");
        statsFile->puts(stats.str().c_str());
        CloseISLogFile(statsFile);
#endif
    }
}
