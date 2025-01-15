/*
MIT LICENSE

Copyright (c) 2014-2025 Inertial Sense, Inc. - http://inertialsense.com

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


typedef void (*FuncLogDataAndTimestamp)(uint32_t dataId, double timeMs);

class cLogStatMsgId
{
public:
	unsigned int count;         // count for this data id
	unsigned int errors;        // error count for this data id
	unsigned int meanDtMs;    	// average time delta for the data id
	unsigned int accumDtMs;     // sum of all time deltas
	unsigned int lastTimeMs;
	unsigned int lastDtMs;
	unsigned int minDtMs;
	unsigned int maxDtMs;
	unsigned int dtMsCount;
	unsigned int timeIrregCount; 	// count of irregularities in delta timestamps (> 50% different from previous delta timestamp)

	unsigned int bpsBytes;
	unsigned int bpsStartTimeMs;
	unsigned int bytesPerSec;

	cLogStatMsgId();
	void LogTimestamp(unsigned int timeMs);
	void LogByteSize(unsigned int timeMs, int bytes);
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
    void LogData(protocol_type_t ptype, int id, int bytes, double timeMs=0.0);
	unsigned int Count();
	unsigned int Errors();
	std::string MessageStats(protocol_type_t ptype, sLogStatPType &msg, bool showDeltaTime=true, bool showErrors=false);
	std::string Stats();
	void WriteToFile(const std::string& fileName);
};



#endif // IS_LOG_STATS_H
