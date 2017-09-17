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
#include "com_manager.h"

using namespace std;

#define IS_DATA_MAPPING_MAX_STRING_LENGTH 2048

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
	DataTypeString,
	DataTypeBinary,

	DataTypeCount
} eDataType;

/*!
* Get the size of an eDataType
* @param dataType the data type to get size for
* @return the size of the data type, or 0 if unknown or variable length (i.e. DataTypeString)
*/
uint32_t GetDataTypeSize(eDataType dataType);

/*
* Metadata about a specific field
*/
typedef struct
{
	uint32_t dataOffset;
	uint32_t dataSize;
	eDataType dataType;
	string name;
} data_info_t;

// map of field name to data info
typedef map<string, data_info_t> map_name_to_info_t;

// map of data id to map of name and data info
typedef map<uint32_t, map_name_to_info_t> map_lookup_name_t;

typedef char data_mapping_string_t[IS_DATA_MAPPING_MAX_STRING_LENGTH];

class cISDataMappings
{
public:
	/*!
	* Get a data set name from an id
	* @param dataId the data id to get a data set name from
	* @return data set name or NULL if not found
	*/
	static const char* GetDataSetName(uint32_t dataId);

	/*!
	* Get the data id to name/info lookup table
	* @return the global map lookup table
	*/
	static const map_lookup_name_t& GetMap();

	/*!
	* Get the size of a given data id
	* @param dataId the data id
	* @return the data id size or 0 if not found or unknown
	*/
	static uint32_t GetSize(uint32_t dataId);

	/*!
	* Convert a string to a data field
	* @param stringBuffer the null terminated string to convert, must not be NULL
	* @param stringLength the number of chars in stringBuffer
	* @param hdr packet header, NULL means dataBuffer is the entire data structure
	* @param dataBuffer packet buffer
	* @param info metadata about the field to convert
	* @return true if success, false if error
	*/
	static bool StringToData(const char* stringBuffer, int stringLength, const p_data_hdr_t* hdr, uint8_t* dataBuffer, const data_info_t& info);

	/*!
	* Convert data to a string
	* @param info metadata about the field to convert
	* @param hdr packet header, NULL means dataBuffer is the entire data structure
	* @param dataBuffer packet buffer
	* @param stringBuffer the buffer to hold the converted string
	* @return true if success, false if error
	*/
	static bool DataToString(const data_info_t& info, const p_data_hdr_t* hdr, const uint8_t* dataBuffer, data_mapping_string_t stringBuffer);

	/*!
	* Get a timestamp from data if available
	* @param hdr data header
	* @param buf data buffer
	* @return timestamp, or 0.0 if no timestamp available
	*/
    static double GetTimestamp(const p_data_hdr_t* hdr, const uint8_t* buf);

	/*!
	* Check whether field data can be retrieved given a data packet
	* @param info metadata for the field to get
	* @param hdr packet header
	* @param buf packet buffer
	* @param ptr receives the offset to get data at if the return value is true
	* @return true if the data can be retrieved, false otherwise
	*/
	static bool CanGetFieldData(const data_info_t& info, const p_data_hdr_t* hdr, const uint8_t* buf, const uint8_t*& ptr);

private:
	cISDataMappings();

	map_lookup_name_t m_columnMappings;
	map<uint32_t, uint32_t> m_lookupSize;

	static cISDataMappings s_map;
};

#endif // __ISDATAMAPPINGS_H_

