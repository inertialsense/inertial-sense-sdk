/*
MIT LICENSE

Copyright (c) 2014-2024 Inertial Sense, Inc. - http://inertialsense.com

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

#include <type_traits>
#include <cstdint>
#include <cstddef>

#ifndef CHAR_BIT
#define CHAR_BIT 8
#endif

#if defined(INCLUDE_LUNA_DATA_SETS)
#include "luna_data_sets.h"
#endif

#define IS_DATA_MAPPING_MAX_STRING_LENGTH 2048

typedef enum
{
	DATA_TYPE_INT8,		// 8-bit  signed 	integer
	DATA_TYPE_UINT8,	// 8-bit  unsigned 	integer
	DATA_TYPE_INT16,	// 16-bit signed 	integer
	DATA_TYPE_UINT16,	// 16-bit unsigned 	integer
	DATA_TYPE_INT32,	// 32-bit signed 	integer
	DATA_TYPE_UINT32,	// 32-bit unsigned 	integer
	DATA_TYPE_INT64,	// 64-bit signed 	integer
	DATA_TYPE_UINT64,	// 64-bit unsigned 	integer
	DATA_TYPE_F32,		// 32-bit float 
	DATA_TYPE_F64,		// 64-bit float
	DATA_TYPE_STRING,
	DATA_TYPE_BINARY,

	DATA_TYPE_COUNT
} eDataType;

typedef enum
{
	DATA_FLAGS_FIXED_DECIMAL_MASK        = 0x0000000F,
	DATA_FLAGS_FIXED_DECIMAL_1           = 0x00000001,
	DATA_FLAGS_FIXED_DECIMAL_2           = 0x00000002,
	DATA_FLAGS_FIXED_DECIMAL_3           = 0x00000003,
	DATA_FLAGS_FIXED_DECIMAL_4           = 0x00000004,
	DATA_FLAGS_FIXED_DECIMAL_5           = 0x00000005,
	DATA_FLAGS_FIXED_DECIMAL_6           = 0x00000006,
	DATA_FLAGS_FIXED_DECIMAL_7           = 0x00000007,
	DATA_FLAGS_FIXED_DECIMAL_8           = 0x00000008,
	DATA_FLAGS_FIXED_DECIMAL_9           = 0x00000009,
	DATA_FLAGS_FIXED_DECIMAL_10          = 0x0000000A,
	DATA_FLAGS_FIXED_DECIMAL_11          = 0x0000000B,
	DATA_FLAGS_FIXED_DECIMAL_12          = 0x0000000C,
	DATA_FLAGS_FIXED_DECIMAL_13          = 0x0000000D,
	DATA_FLAGS_FIXED_DECIMAL_14          = 0x0000000E,
	DATA_FLAGS_FIXED_DECIMAL_15          = 0x0000000F,
	DATA_FLAGS_READ_ONLY                 = 0x00000010,
	DATA_FLAGS_DISPLAY_HEX               = 0x00000100,
	DATA_FLAGS_ANGLE                     = 0x00000200,  // Supports unwrapping angle
    DATA_FLAGS_DECOR_ROLL_MASK           = 0x000F0000,  // Decoration roll
    DATA_FLAGS_INS_STATUS                = 0x00010000,  // "
	DATA_FLAGS_GPS_STATUS                = 0x00020000, 	// "
} eDataFlags;

/*
* Metadata about a specific field
*/
typedef struct
{
	uint32_t    dataOffset;
	uint32_t    dataSize;
	eDataType   dataType;
	uint32_t    elementCount;
	uint32_t    elementSize;
	eDataFlags  dataFlags;
	std::string name;
	std::string units;			// Units (after conversion)
	std::string description;
	double conversion;			// Unit conversion when converting to string
} data_info_t;


CONST_EXPRESSION uint32_t s_eDataTypeSizes[DATA_TYPE_COUNT] =
{
    (uint32_t)sizeof(int8_t),
    (uint32_t)sizeof(uint8_t),
    (uint32_t)sizeof(int16_t),
    (uint32_t)sizeof(uint16_t),
    (uint32_t)sizeof(int32_t),
    (uint32_t)sizeof(uint32_t),
    (uint32_t)sizeof(int64_t),
    (uint32_t)sizeof(uint64_t),
    (uint32_t)sizeof(float),
    (uint32_t)sizeof(double),
    (uint32_t)0, // string, must be set to actual size by caller
    (uint32_t)0  // binary, must be set to actual size by caller
};

#define INIT_MAP(dtype, id) \
    typedef dtype MAP_TYPE; \
    map_name_to_info_t& map = mappings[(id)]; \
    map_index_to_info_t& idx = indices[(id)]; \
    uint32_t totalSize = 0; \
    uint32_t fieldCount = 0; \
	lookupSize[(id)] = sizeof(dtype);

#if CPP11_IS_ENABLED

// dataSize can be 0 for default size, must be set for string type
#define ADD_MAP_NO_VALIDATION(name, member, dataType, fieldType, units, description, flags, conversion)  map[std::string(name)] = { (uint32_t)offsetof(MAP_TYPE, member), (uint32_t)sizeof(fieldType), (dataType), 0, 0, (eDataFlags)(flags), (name), (units), (description), (conversion) }; idx[fieldCount++] = &(map[std::string(name)]); totalSize += sizeof(fieldType);

// note when passing member type for arrays, it must be a reference, i.e. float&
#define ADD_MAP_4(name, member, dataType, fieldType) \
    ADD_MAP_NO_VALIDATION(name, member, dataType, fieldType, "", "", 0, 1.0); \
    static_assert(std::is_same<decltype(MAP_TYPE::member), fieldType>::value, "Field type is an unexpected type"); \
    static_assert((uint32_t)sizeof(fieldType) == sizeof(fieldType), "Field type is an unexpected size, sizeof(fieldType)"); \
    static_assert((uint32_t)sizeof(fieldType) == sizeof(MAP_TYPE::member), "Field type is an unexpected size, sizeof(MAP_TYPE::member)"); \
    static_assert(s_eDataTypeSizes[dataType] == 0 || (uint32_t)sizeof(fieldType) == s_eDataTypeSizes[dataType], "Data type size does not match member size");
#define ADD_MAP_5(name, member, dataType, fieldType, dataFlags) \
    ADD_MAP_NO_VALIDATION(name, member, dataType, fieldType, "", "", dataFlags, 1.0); \
    static_assert(std::is_same<decltype(MAP_TYPE::member), fieldType>::value, "Field type is an unexpected type"); \
    static_assert((uint32_t)sizeof(fieldType) == sizeof(fieldType), "Field type is an unexpected size, sizeof(fieldType)"); \
    static_assert((uint32_t)sizeof(fieldType) == sizeof(MAP_TYPE::member), "Field type is an unexpected size, sizeof(MAP_TYPE::member)"); \
    static_assert(s_eDataTypeSizes[dataType] == 0 || (uint32_t)sizeof(fieldType) == s_eDataTypeSizes[dataType], "Data type size does not match member size");
#define ADD_MAP_6(name, member, dataType, fieldType, units, description) \
    ADD_MAP_NO_VALIDATION(name, member, dataType, fieldType, units, description, 0, 1.0); \
    static_assert(std::is_same<decltype(MAP_TYPE::member), fieldType>::value, "Field type is an unexpected type"); \
    static_assert((uint32_t)sizeof(fieldType) == sizeof(fieldType), "Field type is an unexpected size, sizeof(fieldType)"); \
    static_assert((uint32_t)sizeof(fieldType) == sizeof(MAP_TYPE::member), "Field type is an unexpected size, sizeof(MAP_TYPE::member)"); \
    static_assert(s_eDataTypeSizes[dataType] == 0 || (uint32_t)sizeof(fieldType) == s_eDataTypeSizes[dataType], "Data type size does not match member size");
#define ADD_MAP_7(name, member, dataType, fieldType, units, description, dataFlags) \
    ADD_MAP_NO_VALIDATION(name, member, dataType, fieldType, units, description, dataFlags, 1.0); \
    static_assert(std::is_same<decltype(MAP_TYPE::member), fieldType>::value, "Field type is an unexpected type"); \
    static_assert((uint32_t)sizeof(fieldType) == sizeof(fieldType), "Field type is an unexpected size, sizeof(fieldType)"); \
    static_assert((uint32_t)sizeof(fieldType) == sizeof(MAP_TYPE::member), "Field type is an unexpected size, sizeof(MAP_TYPE::member)"); \
    static_assert(s_eDataTypeSizes[dataType] == 0 || (uint32_t)sizeof(fieldType) == s_eDataTypeSizes[dataType], "Data type size does not match member size");
#define ADD_MAP_8(name, member, dataType, fieldType, units, description, dataFlags, conversion) \
    ADD_MAP_NO_VALIDATION(name, member, dataType, fieldType, units, description, dataFlags, conversion); \
    static_assert(std::is_same<decltype(MAP_TYPE::member), fieldType>::value, "Field type is an unexpected type"); \
    static_assert((uint32_t)sizeof(fieldType) == sizeof(fieldType), "Field type is an unexpected size, sizeof(fieldType)"); \
    static_assert((uint32_t)sizeof(fieldType) == sizeof(MAP_TYPE::member), "Field type is an unexpected size, sizeof(MAP_TYPE::member)"); \
    static_assert(s_eDataTypeSizes[dataType] == 0 || (uint32_t)sizeof(fieldType) == s_eDataTypeSizes[dataType], "Data type size does not match member size");
#define ASSERT_SIZE(s) assert(s == sizeof(MAP_TYPE))

#else

#define ADD_MAP_NO_VALIDATION(name, member, dataType, fieldType, dataFlags) map[std::string(name)] = { (uint32_t)offsetof(MAP_TYPE, member), (uint32_t)sizeof(fieldType), dataType, (eDataFlags)dataFlags, name }; totalSize += sizeof(fieldType);
#define ADD_MAP_4(name, member, dataType, fieldType, dataFlags) ADD_MAP_NO_VALIDATION(name, member, dataType, fieldType, dataFlags)
#define ASSERT_SIZE(s) // not supported on VS < 2015

#endif



#if !PLATFOM_IS_EMBEDDED

extern const unsigned char g_asciiToLowerMap[256];

#endif

/**
* Case-insensitive comparator for std::find and other functions
*/
struct sCaseInsensitiveCompare
{
	struct nocase_compare
	{
		bool operator() (const unsigned char& c1, const unsigned char& c2) const
		{
			return g_asciiToLowerMap[c1] < g_asciiToLowerMap[c2];
		}
	};

	/*
	size_t operator()(const std::string s) const
	{
		size_t hashCode = 5381;
		char c;
		const char* ptr = s.c_str();
		const char* ptrEnd = ptr + s.size();
		for (; ptr < ptrEnd; ptr++)
		{
			c = g_asciiToLowerMap[*ptr];
			hashCode = ((hashCode << 5) + hashCode) + c;
		}
		return hashCode;
	}
	*/

	// less than, not equal
	bool operator() (const std::string& s1, const std::string& s2) const
	{
		// return std::lexicographical_compare(s1.begin(), s1.end(), s2.begin(), s2.end(), nocase_compare());

		// we don't need unicode or fancy language handling here, and we do not want branching
		// so we have hand-coded a highly performant NMEA case insensitive compare here.
		// this custom code is 3x speed of lexicographical_compare
		char c1, c2;
		const char* ptr1 = s1.c_str();
		const char* ptr2 = s2.c_str();
		size_t size1 = s1.size();
		size_t size2 = s2.size();
		int sizeDiff = (int)size1 - (int)size2;

		// branchless min
		// y + ((x - y) & ((x - y) >> (sizeof(int) * CHAR_BIT - 1))); 
		size_t minSize = size2 + (sizeDiff & (sizeDiff >> (sizeof(int) * CHAR_BIT - 1)));

		for (size_t i = 0; i < minSize; i++)
		{
			c1 = g_asciiToLowerMap[(int)ptr1[i]];
			c2 = g_asciiToLowerMap[(int)ptr2[i]];
			if (c1 != c2)
			{
				return (c1 < c2);
			}
		}
		return (s1.size() < s2.size());
	}
};


// map of field name to data info
typedef std::map<std::string, data_info_t, sCaseInsensitiveCompare> map_name_to_info_t;
typedef std::map<uint32_t, data_info_t*> map_index_to_info_t;
typedef char data_mapping_string_t[IS_DATA_MAPPING_MAX_STRING_LENGTH];



template <typename Dtype>
class DataMapper
{
public:
    typedef Dtype MAP_TYPE;

    DataMapper(map_name_to_info_t mappings[DID_COUNT], uint32_t lookupSize[DID_COUNT], map_index_to_info_t indices[DID_COUNT], uint32_t id) : map(mappings[id]), idx(indices[id]), mappedSize(0), fieldCount(0)
    {
        lookupSize[id] = structSize = sizeof(MAP_TYPE);
    }

    ~DataMapper()
	{
        assert((mappedSize == structSize) && "Size of mapped fields does not match struct size");
	}

    template <typename MemberType>
	void AddMember(const std::string& name, 
		MemberType member,
		eDataType dataType,
		const std::string& units = "", 
		const std::string& description = "",
		int dataFlags = 0, 
		double conversion = 1.0) 	
    {
        using FieldType = typename std::remove_cv<typename std::remove_reference<decltype(((MAP_TYPE*)nullptr)->*member)>::type>::type;
        uint32_t offset = (uint32_t)(uintptr_t)&(((MAP_TYPE*)nullptr)->*member);
		uint32_t dataSize = (uint32_t)sizeof(FieldType);
		uint32_t elementCount = 0; 	// Zero for single element 
		uint32_t elementSize = dataSize;

        // Populate the map with the new entry
        map[name] = { 
            offset,
            dataSize,
            dataType,
			elementCount,
            elementSize,
            eDataFlags(dataFlags), 
            name, 
            units, 
            description, 
            conversion 
        };

        // Add the entry to the index
		data_info_t &dinfo = map[name];
        idx[fieldCount++] = &dinfo;
        mappedSize += dataSize;

        // Static assertions for type and size validation
        static_assert(std::is_same<MemberType, FieldType MAP_TYPE::*>::value, "MemberType is not a member pointer");
        static_assert((uint32_t)sizeof(FieldType) == sizeof(FieldType), "Field type is an unexpected size");
		if (dataType != DATA_TYPE_STRING)
		{
			assert((s_eDataTypeSizes[dataType] != 0) && "Data type size invalid");
			assert((s_eDataTypeSizes[dataType] == dinfo.dataSize) && "Data type size mismatch");
		}
    }

    template <typename MemberType>
	void AddArray(const std::string& name, 
		MemberType member,
		eDataType dataType,
		uint32_t elementCount,
		const std::string& units = "", 
		const std::string& description = "",
		int dataFlags = 0, 
		double conversion = 1.0) 	
    {
        using FieldType = typename std::remove_cv<typename std::remove_reference<decltype(((MAP_TYPE*)nullptr)->*member)>::type>::type;
        uint32_t offset = (uint32_t)(uintptr_t)&(((MAP_TYPE*)nullptr)->*member);
		uint32_t dataSize = (uint32_t)sizeof(FieldType);
		uint32_t elementSize = dataSize/elementCount;

        // Populate the map with the new entry
        map[name] = { 
            offset,
            dataSize, 
            dataType,
			elementCount,
			elementSize,
            eDataFlags(dataFlags), 
            name, 
            units, 
            description, 
            conversion 
        };

        // Add the entry to the index
        idx[fieldCount++] = &(map[name]);
        mappedSize += dataSize;

        // Static assertions for type and size validation
        static_assert(std::is_same<MemberType, FieldType MAP_TYPE::*>::value, "MemberType is not a member pointer");
        static_assert((uint32_t)sizeof(FieldType) == sizeof(FieldType), "Field type is an unexpected size");
        assert(((s_eDataTypeSizes[dataType]) != 0) && "Data type size invalid");
        assert(((s_eDataTypeSizes[dataType]*elementCount) == dataSize) && "Data type size mismatch");
    }

private:
	uint32_t did;
    map_name_to_info_t& map;  		// Reference to the specific mapping for this instance
    map_index_to_info_t& idx; 		// Reference to the specific index for this instance
    uint32_t structSize;    		// Size of the data set struct
    uint32_t mappedSize;    		// Size of all mapped fields
    uint32_t fieldCount;   			// Tracks the count of fields
};



/*
template <>
struct equal_to<std::string> : public unary_function<std::string, bool>
{
	bool operator()(const std::string& s1, const std::string& s2) const
	{
		// we don't need unicode or fancy language handling here, and we do not want branching
		// so we have hand-coded a highly performant NMEA case insensitive compare here.
		// this custom code is 3x speed of lexicographical_compare
		if (s1.size() != s2.size())
		{
			return false;
		}

		char c1, c2;
		const char* ptr1 = s1.c_str();
		const char* ptr2 = s2.c_str();

		for (size_t i = 0; i < s1.size(); i++)
		{
			c1 = g_asciiToLowerMap[ptr1[i]];
			c2 = g_asciiToLowerMap[ptr2[i]];
			if (c1 != c2)
			{
				return false;
			}
		}
		return true;
	}
};
*/


class cISDataMappings
{
public:
	/**
	* Destructor
	*/
	virtual ~cISDataMappings();

	/**
	* Get a data set name from an id
	* @param dataId the data id to get a data set name from
	* @return data set name or NULL if not found
	*/
	static const char* GetName(uint32_t dataId);

	/**
	* Get a data set id from name
	* @param dataId the data id to get a data set name from
	* @return data set name or NULL if not found
	*/
	static uint32_t GetId(std::string name);

	/**
	* Get the info for a data id
	* @return the info for the data id, or NULL if none found
	*/
	static const map_name_to_info_t* GetMapInfo(uint32_t dataId);

	/**
	* Get map pointer for a data id
	* @return map pointer for the data id, or NULL if none found
	*/
	static const map_index_to_info_t* GetIndexMapInfo(uint32_t dataId);

	/**
	* Get data info pointer based on data id and field index number
	* @return the data info for data id field, or NULL if none found
	*/
	// static const data_info_t* cISDataMappings::GetFieldDataInfo(uint32_t dataId, uint32_t field);

	/**
	* Get the size of a given data id
	* @param dataId the data id
	* @return the data id size or 0 if not found or unknown
	*/
	static uint32_t GetSize(uint32_t dataId);

	/**
	* Convert a string to a data field inside a data set.
	* @param stringBuffer the null terminated string to convert, must not be NULL
	* @param stringLength the number of chars in stringBuffer
	* @param hdr packet header, NULL means dataBuffer is the entire data structure
	* @param datasetBuffer packet buffer
	* @param info metadata about the field to convert
	* @param radix (base 10, base 16, etc.) to use if the field is a number field, ignored otherwise
	* @param json true if json, false if csv
	* @return true if success, false if error
	*/
	static bool StringToData(const char* stringBuffer, int stringLength, const p_data_hdr_t* hdr, uint8_t* datasetBuffer, const data_info_t& info, int radix = 10, bool json = false);

	/**
	* Convert a string to a variable.
	* @param stringBuffer the null terminated string to convert, must not be NULL
	* @param stringLength the number of chars in stringBuffer
	* @param dataBuffer data buffer pointer
	* @param dataType data type
	* @param radix (base 10, base 16, etc.) to use if the field is a number field, ignored otherwise
	* @param json true if json, false if csv
	* @return true if success, false if error
	*/
	static bool StringToVariable(const char* stringBuffer, int stringLength, const uint8_t* dataBuffer, eDataType dataType, uint32_t dataSize, int radix = 10, bool json = false);

	/**
	* Convert dataset field to a string
	* @param info metadata about the field to convert
	* @param hdr packet header, NULL means dataBuffer is the entire data structure
	* @param datasetBuffer packet buffer
	* @param stringBuffer the buffer to hold the converted string
	* @param elementIndex index into array
	* @param elementSize size of elements in array
	* @param json true if json, false if csv
	* @return true if success, false if error
	*/
	static bool DataToString(const data_info_t& info, const p_data_hdr_t* hdr, const uint8_t* datasetBuffer, data_mapping_string_t stringBuffer, int elementIndex = 0, int elementSize = 0, bool json = false);

	/**
	* Convert a variable to a string
	* @param dataType data type
	* @param dataFlags data flags 
	* @param dataBuffer data buffer pointer
	* @param stringBuffer the buffer to hold the converted string
	* @param elementIndex index into array
	* @param elementSize size of elements in array
	* @param json true if json, false if csv
	* @return true if success, false if error
	*/
	static bool VariableToString(eDataType dataType, eDataFlags dataFlags, const uint8_t* ptr, const uint8_t* dataBuffer, uint32_t dataSize, data_mapping_string_t stringBuffer, int elementIndex = 0, int elementSize = 0, bool json = false);

	/**
	* Get a timestamp from data if available
	* @param hdr data header
	* @param buf data buffer
	* @return timestamp, or 0.0 if no timestamp available
	*/
    static double GetTimestamp(const p_data_hdr_t* hdr, const uint8_t* buf);

	/**
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

	static const char* const m_dataIdNames[];

	uint32_t m_lookupSize[DID_COUNT];
	const data_info_t* m_timestampFields[DID_COUNT];
	map_name_to_info_t m_lookupInfo[DID_COUNT];
	map_index_to_info_t m_indexInfo[DID_COUNT];

    #define PROTECT_UNALIGNED_ASSIGNS
    template<typename T>
    static inline void protectUnalignedAssign(void* out, T in) {
    #ifdef PROTECT_UNALIGNED_ASSIGNS
        memcpy((void*)out, (void*)&in, sizeof(T));
    #else
        *(T*)out = in;
    #endif
    }

    template<typename T>
    static inline T protectUnalignedAssign(void* in) {
    #ifdef PROTECT_UNALIGNED_ASSIGNS
        T out;
        memcpy((void*)&out, in, sizeof(T));
        return out;
    #else
        return *(T*)in;
    #endif
    }


#if PLATFORM_IS_EMBEDDED

	// on embedded we cannot new up C++ runtime until after free rtos has started
	static cISDataMappings* s_map;

#else

	static cISDataMappings s_map;

#endif

};

#endif // __ISDATAMAPPINGS_H_

