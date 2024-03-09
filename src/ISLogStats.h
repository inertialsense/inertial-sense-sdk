/*
MIT LICENSE

Copyright (c) 2014-2023 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#ifndef IS_LOG_STATS_H
#define IS_LOG_STATS_H

#include <string>
#include <cstdint>
#include <map>

#include "data_sets.h"
#include "ISComm.h"


typedef void (*FuncLogDataAndTimestamp)(uint32_t dataId, double timestamp);

class cLogStatDataId
{
public:
	uint64_t count; // count for this data id
	uint64_t errorCount; // error count for this data id
	double averageTimeDelta; // average time delta for the data id
	double totalTimeDelta; // sum of all time deltas
	double lastTimestamp;
	double lastTimestampDelta;
	double minTimestampDelta;
	double maxTimestampDelta;
	uint64_t timestampDeltaCount;
	uint64_t timestampDropCount; // count of delta timestamps > 50% different from previous delta timestamp

	cLogStatDataId();
	void LogTimestamp(double timestamp);
	void Printf();
};

class cLogStats
{
public:
	std::map<int, cLogStatDataId> isbStats;
	std::map<int, cLogStatDataId> nmeaStats;
	std::map<int, cLogStatDataId> rtcm3Stats;
	std::map<int, cLogStatDataId> ubloxStats;
	uint64_t count; // count of all data ids
	uint64_t errorCount; // total error count
	cISLogFileBase* statsFile;

	cLogStats();
	void Clear();
	void LogError(const p_data_hdr_t* hdr);
	cLogStatDataId* MsgStats(protocol_type_t ptype, uint32_t dataId);
	void LogData(uint32_t dataId, protocol_type_t ptype=_PTYPE_INERTIAL_SENSE_DATA);
	void LogDataAndTimestamp(uint32_t dataId, double timestamp, protocol_type_t ptype=_PTYPE_INERTIAL_SENSE_DATA);
	void Printf();
	void WriteMsgStats(std::map<int, cLogStatDataId> &msgStats, const char* msgName, protocol_type_t ptype=_PTYPE_NONE);
	void WriteToFile(const std::string& fileName);
};



#endif // IS_LOG_STATS_H
