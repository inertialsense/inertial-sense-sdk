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
	return;
}

void cLogStats::LogError(const p_data_hdr_t* hdr, protocol_type_t ptype)
{
	return;
}

void cLogStats::LogData(protocol_type_t ptype, int id, double timestamp)
{
	return;
}

string cLogStats::MessageStats(protocol_type_t ptype, sLogStatPType &msg, bool showDeltaTime, bool showErrors)
{
	return "";
}

unsigned int cLogStats::Count()
{
	return 0;
;
}

unsigned int cLogStats::Errors()
{
	return 0;
}

string cLogStats::Stats()
{
	return "";
}

void cLogStats::WriteToFile(const string& file_name)
{
	return;
}
