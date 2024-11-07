/*
MIT LICENSE

Copyright (c) 2014-2024 Inertial Sense, Inc. - http://inertialsense.com

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

class cLogStatMsgId
{
public:
	unsigned int count;         // count for this data id
	unsigned int errors;        // error count for this data id
	double averageTimeDelta;    // average time delta for the data id
	double totalTimeDelta;      // sum of all time deltas
	double lastTimestamp;
	double lastTimestampDelta;
	double minTimestampDelta;
	double maxTimestampDelta;
	unsigned int timestampDeltaCount;
	unsigned int timestampIrregCount; 	// count of irregularities in delta timestamps (> 50% different from previous delta timestamp)

	cLogStatMsgId();
	void LogTimestamp(double timestamp);
};

struct sLogStatPType
{
	std::map<int, cLogStatMsgId> stats;     // ID, cLogStatMsgId
	unsigned int count;                     // count of all message ids
	unsigned int errors;                    // total error count
};

class cLogStats
{
public:
	std::map<protocol_type_t, sLogStatPType> msgs;
	cISLogFileBase* statsFile;

	cLogStats();
	void Clear();
	void LogError(const p_data_hdr_t* hdr, protocol_type_t ptype=_PTYPE_INERTIAL_SENSE_DATA);
	void LogData(protocol_type_t ptype, int id, double timestamp=0.0);
	unsigned int Count();
	unsigned int Errors();
	std::string MessageStats(protocol_type_t ptype, sLogStatPType &msg, bool showDeltaTime=true, bool showErrors=false);
	std::string Stats();
	void WriteToFile(const std::string& fileName);
};



#endif // IS_LOG_STATS_H
