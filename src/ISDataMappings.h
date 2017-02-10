/*
MIT LICENSE

Copyright 2014 Inertial Sense, LLC - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#ifndef __ISDATAMAPPINGS_H_
#define __ISDATAMAPPINGS_H_

#include <string>
#include <map>
#include <inttypes.h>

using namespace std;

#define IS_DATA_MAPPING_MAX_STRING_LENGTH 128

typedef enum
{
	DataTypeInt8,
	DataTypeUInt8,
	DataTypeInt16,
	DataTypeUInt16,
	DataTypeInt32,
	DataTypeUInt32,
	DataTypeInt64,
	DataTypeUInt64,
	DataTypeFloat,
	DataTypeDouble,
	DataTypeString
} eDataType;

typedef struct
{
	uint32_t dataOffset;
	uint32_t dataSize;
	eDataType dataType;
} data_info_t;

// map of field name to data info
typedef map<string, data_info_t> map_name_to_info_t;

// map of data id to map of name and data info
typedef map<uint32_t, map_name_to_info_t> map_lookup_name_t;

class cISDataMappings
{
public:
	/*!
	* Get a data set name from an id, returns NULL if not found
	*/
	static const char* GetDataSetName(uint32_t dataId);

	/*!
	* Get the data id to name/info lookup table
	*/
	static const map_lookup_name_t& GetMap();

	/*!
	* Get the size of a given data id, 0 if not found or unknown
	*/
	static uint32_t GetSize(uint32_t dataId);

	/*!
	* Convert a string to a data field
	* buf must be the size of the entire data structure
	*/
	static bool StringToData(const char* stringBuffer, uint8_t* buf, const data_info_t& info);

	/*!
	* Convert data to a string
	* dataBuffer must be the size of the entire data structure
	*/
	static bool DataToString(const data_info_t& info, const uint8_t* dataBuffer, char stringBuffer[IS_DATA_MAPPING_MAX_STRING_LENGTH]);

private:
	cISDataMappings();

	map_lookup_name_t m_columnMappings;
	map<uint32_t, uint32_t> m_lookupSize;

	static cISDataMappings s_map;
};

#endif // __ISDATAMAPPINGS_H_

