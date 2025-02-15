/*
MIT LICENSE

Copyright (c) 2014-2025 Inertial Sense, Inc. - http://inertialsense.com

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
    DATA_FLAGS_FIXED_DECIMAL_0           = 0x00000001,
    DATA_FLAGS_FIXED_DECIMAL_1           = 0x00000002,
	DATA_FLAGS_FIXED_DECIMAL_2           = 0x00000003,
	DATA_FLAGS_FIXED_DECIMAL_3           = 0x00000004,
    DATA_FLAGS_FIXED_DECIMAL_4           = 0x00000005,
	DATA_FLAGS_FIXED_DECIMAL_5           = 0x00000006,
	DATA_FLAGS_FIXED_DECIMAL_6           = 0x00000007,
	DATA_FLAGS_FIXED_DECIMAL_7           = 0x00000008,
	DATA_FLAGS_FIXED_DECIMAL_8           = 0x00000009,
	DATA_FLAGS_FIXED_DECIMAL_9           = 0x0000000A,
	DATA_FLAGS_FIXED_DECIMAL_10          = 0x0000000B,
	DATA_FLAGS_FIXED_DECIMAL_11          = 0x0000000C,
	DATA_FLAGS_FIXED_DECIMAL_12          = 0x0000000D,
	DATA_FLAGS_FIXED_DECIMAL_13          = 0x0000000E,
	DATA_FLAGS_FIXED_DECIMAL_14          = 0x0000000F,
	DATA_FLAGS_READ_ONLY                 = 0x00000010,
	DATA_FLAGS_HIDDEN                    = 0x00000020,	// Do not print to screen
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
	uint32_t    offset;
	uint32_t    size;
	eDataType   type;
	uint32_t    arraySize;		// Number of elements in array.  Zero for single/non-array elements.
	uint32_t    elementSize;	// Element size in bytes
	eDataFlags  flags;
	std::string name;
	std::string units;			// Units (after conversion)
	std::string description;
	double conversion;			// Unit conversion when converting to string
} data_info_t;

CONST_EXPRESSION uint32_t s_eDataTypeSize[DATA_TYPE_COUNT] =
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

CONST_EXPRESSION uint32_t s_eDataTypeHexStringSize[DATA_TYPE_COUNT] =
{	// Number of characters in hexidecimal string including prefix "0x" 
    (uint32_t)4,	// 0x00
    (uint32_t)4,    // 0x00
    (uint32_t)6,    // 0x0000
    (uint32_t)6,    // 0x0000
    (uint32_t)10,   // 0x0000000000
    (uint32_t)10,   // 0x0000000000
    (uint32_t)18,   // 0x00000000000000000000
    (uint32_t)18,   // 0x00000000000000000000
    (uint32_t)0,
    (uint32_t)0,
    (uint32_t)0,
    (uint32_t)0
};

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

typedef std::map<std::string, data_info_t, sCaseInsensitiveCompare>     map_name_to_info_t;             // map of field name to data info
typedef std::map<uint32_t, data_info_t*>                                map_index_to_info_t;            // map of field index to data info pointer
typedef std::map<uint32_t, data_info_t*>                                map_element_to_info_t;          // map of element index to data info pointer
typedef std::map<uint32_t, uint32_t>                                    map_element_to_array_size_t;    // map of element index to array size
typedef char data_mapping_string_t[IS_DATA_MAPPING_MAX_STRING_LENGTH];

typedef struct
{
	uint32_t                    size;
	map_name_to_info_t          nameToInfo;
	map_index_to_info_t         indexToInfo;
	map_element_to_info_t	    elementToInfo;
	map_element_to_array_size_t elementToArraySize;
	uint32_t                    elementCount;
	const data_info_t*          timestampFields;
} data_set_t;

template <typename Dtype>
class DataMapper
{
public:
    typedef Dtype MAP_TYPE;

    DataMapper(data_set_t data_set[DID_COUNT], uint32_t did) : ds(data_set[did]), totalSize(0), memberCount(0)
    {
        data_set[did].size = structSize = sizeof(MAP_TYPE);
    }

    ~DataMapper()
    {
        assert((totalSize == structSize) && "Size of mapped fields does not match struct size");
	}

    template <typename MemberType>
	void AddMember(const std::string& name, 
		MemberType member,
		eDataType type,
		const std::string& units = "", 
		const std::string& description = "",
		int flags = 0, 
		double conversion = 1.0) 	
    {
        using FieldType = typename std::remove_cv<typename std::remove_reference<decltype(((MAP_TYPE*)nullptr)->*member)>::type>::type;
        uint32_t offset = (uint32_t)(uintptr_t)&(((MAP_TYPE*)nullptr)->*member);
		uint32_t size = (uint32_t)sizeof(FieldType);
		uint32_t arraySize = 0; 	// Zero for single element 
		uint32_t elementSize = size;

        // Populate the map with the new entry
        ds.nameToInfo[name] = { 
            offset,
            size,
            type,
			arraySize,
            elementSize,
            eDataFlags(flags), 
            name, 
            units, 
            description, 
            conversion 
        };

        // Add the entry to the index
		data_info_t *dinfo = &ds.nameToInfo[name];
        ds.indexToInfo[memberCount++] = dinfo;
        totalSize += size;
		{
			ds.elementToInfo[ds.elementCount] = dinfo;
			ds.elementToArraySize[ds.elementCount] = arraySize;
			ds.elementCount++;
		}
		
        // Static assertions for type and size validation
        static_assert(std::is_same<MemberType, FieldType MAP_TYPE::*>::value, "MemberType is not a member pointer");
        static_assert((uint32_t)sizeof(FieldType) == sizeof(FieldType), "Field type is an unexpected size");
		if ((type != DATA_TYPE_STRING) && (type != DATA_TYPE_BINARY))
		{
			assert((s_eDataTypeSize[type] != 0) && "Data type size invalid");
			assert((s_eDataTypeSize[type] == dinfo->size) && "Data type size mismatch");
		}
    }

    template <typename MemberType>
	void AddArray(const std::string& name, 
		MemberType member,
		eDataType type,
		uint32_t arraySize,
		const std::string& units = "", 
		const std::string& description = "",
        int flags = 0,
		double conversion = 1.0) 	
    {
        using FieldType = typename std::remove_cv<typename std::remove_reference<decltype(((MAP_TYPE*)nullptr)->*member)>::type>::type;
        uint32_t offset = (uint32_t)(uintptr_t)&(((MAP_TYPE*)nullptr)->*member);
		uint32_t size = (uint32_t)sizeof(FieldType);
		uint32_t elementSize = size/arraySize;

        // Populate the map with the new entry
        ds.nameToInfo[name] = { 
            offset,
            size, 
            type,
			arraySize,
			elementSize,
            eDataFlags(flags),
            name, 
            units, 
            description, 
            conversion 
        };

        // Add the entry to the index
		data_info_t *dinfo = &ds.nameToInfo[name];
        ds.indexToInfo[memberCount++] = dinfo;
        totalSize += size;
		for (uint32_t i=0; i<arraySize; i++)
		{
			ds.elementToInfo[ds.elementCount] = dinfo;
			ds.elementToArraySize[ds.elementCount] = i;
			ds.elementCount++;
		}

        // Static assertions for type and size validation
        static_assert(std::is_same<MemberType, FieldType MAP_TYPE::*>::value, "MemberType is not a member pointer");
        static_assert((uint32_t)sizeof(FieldType) == sizeof(FieldType), "Field type is an unexpected size");
		if ((type != DATA_TYPE_STRING) && (type != DATA_TYPE_BINARY))
        {
            assert((s_eDataTypeSize[type] != 0) && "Data type size invalid");
            assert((s_eDataTypeSize[type]*arraySize == size) && "Data type size mismatch");
        }
    }

	void AddMember2(const std::string& name, 
		uint32_t offset,
		eDataType type,
		const std::string& units = "", 
		const std::string& description = "",
		int flags = 0, 
		double conversion = 1.0,
		uint32_t typeSize = 0)
    {
		uint32_t size = (typeSize ? typeSize : s_eDataTypeSize[type]);
		uint32_t arraySize = 0; 	// Zero for single element 
		uint32_t elementSize = size;

        // Populate the map with the new entry
        ds.nameToInfo[name] = { 
            offset,
            size,
            type,
			arraySize,
            elementSize,
            eDataFlags(flags), 
            name, 
            units, 
            description, 
            conversion 
        };

        // Add the entry to the index
		data_info_t *dinfo = &ds.nameToInfo[name];
        ds.indexToInfo[memberCount++] = dinfo;
        totalSize += size;
		{
			ds.elementToInfo[ds.elementCount] = dinfo;
			ds.elementToArraySize[ds.elementCount] = arraySize;
			ds.elementCount++;
		}
		
        // Static assertions for type and size validation
        // static_assert(std::is_same<MemberType, FieldType MAP_TYPE::*>::value, "MemberType is not a member pointer");
        // static_assert((uint32_t)sizeof(FieldType) == sizeof(FieldType), "Field type is an unexpected size");
		if ((type != DATA_TYPE_STRING) && (type != DATA_TYPE_BINARY))
		{
			assert((s_eDataTypeSize[type] != 0 || (type == DATA_TYPE_STRING)) && "Data type size invalid");
			assert((s_eDataTypeSize[type] == dinfo->size) && "Data type size mismatch");
		}
    }

	void AddArray2(const std::string& name,
		uint32_t offset,
		eDataType type,
		uint32_t arraySize,
		const std::string& units = "", 
		const std::string& description = "",
        int flags = 0,
		double conversion = 1.0,
		uint32_t typeSize = 0)
    {
		uint32_t elementSize = (typeSize ? typeSize : s_eDataTypeSize[type]);
		uint32_t size = elementSize * arraySize;

        // Populate the map with the new entry
        ds.nameToInfo[name] = { 
            offset,
            size, 
            type,
			arraySize,
			elementSize,
            eDataFlags(flags),
            name, 
            units, 
            description, 
            conversion 
        };

        // Add the entry to the index
		data_info_t *dinfo = &ds.nameToInfo[name];
        ds.indexToInfo[memberCount++] = dinfo;
        totalSize += size;
		for (uint32_t i=0; i<arraySize; i++)
		{
			ds.elementToInfo[ds.elementCount] = dinfo;
			ds.elementToArraySize[ds.elementCount] = i;
			ds.elementCount++;
		}

        // Static assertions for type and size validation
        // static_assert(std::is_same<MemberType, FieldType MAP_TYPE::*>::value, "MemberType is not a member pointer");
        // static_assert((uint32_t)sizeof(FieldType) == sizeof(FieldType), "Field type is an unexpected size");
		if ((type != DATA_TYPE_STRING) && (type != DATA_TYPE_BINARY))
		{
			assert((s_eDataTypeSize[type] != 0) && "Data type size invalid");
			assert((s_eDataTypeSize[type]*arraySize == size) && "Data type size mismatch");
		}
    }

	void AddLlaDegM(const std::string& name, 
		uint32_t offset,
		const std::string& description = "",
		const std::string& descriptionAltitude = "",
		int flags = 0)
    {
		eDataType type = DATA_TYPE_F64;
		AddMember2(name + "[0]", offset + 0*s_eDataTypeSize[type], type, "°", description + " latitude", flags | DATA_FLAGS_FIXED_DECIMAL_8);
		AddMember2(name + "[1]", offset + 1*s_eDataTypeSize[type], type, "°", description + " longitude", flags | DATA_FLAGS_FIXED_DECIMAL_8);
		AddMember2(name + "[2]", offset + 2*s_eDataTypeSize[type], type, "m", description + " " + descriptionAltitude, flags | DATA_FLAGS_FIXED_DECIMAL_3);
	}

	void AddVec3(const std::string& name, 
		uint32_t offset,
		eDataType type,
		const std::string& units = "",
		const std::string& description1 = "",
		const std::string& description2 = "",
		const std::string& description3 = "",
		int flags = 0,
		double conversion = 1.0)
    {
		AddMember2(name + "[0]", offset + 0*s_eDataTypeSize[type], type, units, description1, flags, conversion);
		AddMember2(name + "[1]", offset + 1*s_eDataTypeSize[type], type, units, description2, flags, conversion);
		AddMember2(name + "[2]", offset + 2*s_eDataTypeSize[type], type, units, description3, flags, conversion);
	}

	void AddVec4(const std::string& name, 
		uint32_t offset,
		eDataType type,
		const std::string& units = "",
		const std::string& description1 = "",
		const std::string& description2 = "",
		const std::string& description3 = "",
		const std::string& description4 = "",
		int flags = 0,
		double conversion = 1.0)
    {
		AddMember2(name + "[0]", offset + 0*s_eDataTypeSize[type], type, units, description1, flags, conversion);
		AddMember2(name + "[1]", offset + 1*s_eDataTypeSize[type], type, units, description2, flags, conversion);
		AddMember2(name + "[2]", offset + 2*s_eDataTypeSize[type], type, units, description3, flags, conversion);
		AddMember2(name + "[3]", offset + 3*s_eDataTypeSize[type], type, units, description4, flags, conversion);
	}

	void AddVec3Xyz(const std::string& name, 
		uint32_t offset,
		eDataType type,
		const std::string& units = "",
		const std::string& description = "",
		int flags = 0,
		double conversion = 1.0)
	{
		AddVec3(name, offset, type, units, "X "+description, "Y "+description, "Z "+description, flags, conversion);
	}

	void AddVec3Rpy(const std::string& name, 
		uint32_t offset,
		eDataType type,
		const std::string& units = "",
		const std::string& description = "",
		int flags = 0,
		double conversion = 1.0)
	{
		AddVec3(name, offset, type, units, "Roll "+description, "Pitch "+description, "Yaw "+description, flags, conversion);
	}

private:
	data_set_t& ds;						// data set reference
    uint32_t structSize;                // size of data set struct. Used to compare against totalSize to ensure all members were included.
    uint32_t totalSize;                 // size of mapped fields
    uint32_t memberCount;               // number of members in struct
};


class cISDataMappings
{
public:
	cISDataMappings();

	virtual ~cISDataMappings() {}

	/**
	* Get a data set name from an id
	* @param did the data id to get a data set name from
	* @return data set name or NULL if not found
	*/
	static const char* DataName(uint32_t did);

	/**
	* Get a data set id from name
	* @param did the data id to get a data set name from
	* @return data set name or NULL if not found
	*/
	static uint32_t Did(std::string name);

	/**
	* Get the size of a given data id
	* @param did the data id
	* @return the data id size or 0 if not found or unknown
	*/
	static uint32_t DataSize(uint32_t did);

	/**
	* Get the data set for a data id
	* @return the data set for the data id, or NULL if none found
	*/
	static data_set_t* DataSet(uint32_t did);

	/**
	* Get the info for a data id
	* @return the info for the data id, or NULL if none found
	*/
	static const map_name_to_info_t* NameToInfoMap(uint32_t did);

	/**
	* Get map pointer for a data id
	* @return map pointer for the data id, or NULL if none found
	*/
	static const map_index_to_info_t* IndexToInfoMap(uint32_t did);

	/**
	* Get map pointer for a data id
	* @return map pointer for the data id (or NULL if none found) and array index
	*/
	static const data_info_t* ElementToInfo(uint32_t did, uint32_t element, uint32_t &arrayIndex);

	/**
	* Get number of elements of a given data id.  Arrays get counted as multiple elements.
	* @param did the data id
	* @return number of elements or 0 if not found or unknown
	*/
	static uint32_t ElementCount(uint32_t did);

	/**
	* Get the default period multiple for the specified data set.  This is used to prevent non-rmc messages from streaming at 1ms periods (too high).  
	* @param did the data id
	* @return the default period multiple
	*/
	static uint32_t DefaultPeriodMultiple(uint32_t did);

	/**
	* Convert a string to a data field inside a data set.
	* @param stringBuffer the null terminated string to convert, must not be NULL
	* @param stringLength the number of chars in stringBuffer
	* @param hdr packet header, NULL means dataBuffer is the entire data structure
	* @param datasetBuffer packet buffer
	* @param info metadata about the field to convert
	* @param arrayIndex index into array
	* @param elementSize size of elements in array
	* @param json true if json, false if csv
	* @return true if success, false if error
	*/
	static bool StringToData(const char* stringBuffer, int stringLength, const p_data_hdr_t* hdr, uint8_t* datasetBuffer, const data_info_t& info, unsigned int arrayIndex = 0, bool json = false);

	/**
	* Convert a string to a variable.
	* @param stringBuffer the null terminated string to convert, must not be NULL
	* @param stringLength the number of chars in stringBuffer
	* @param dataBuffer data buffer pointer
	* @param dataType data type
	* @param radix (10 = base 10 for decimal, 16 = base 16 for hexidecimal) if the field is a number field, ignored otherwise
	* @param conversion conversion of value (i.e. rad2deg)
	* @param json true if json, false if csv
	* @return true if success, false if error
	*/
	static bool StringToVariable(const char* stringBuffer, int stringLength, const uint8_t* dataBuffer, eDataType dataType, uint32_t dataSize, int radix = 10, double conversion = 1.0, bool json = false);

	/**
	* Convert dataset field to a string
	* @param info metadata about the field to convert
	* @param hdr packet header, NULL means dataBuffer is the entire data structure
	* @param datasetBuffer packet buffer
	* @param stringBuffer the buffer to hold the converted string
	* @param arrayIndex index into array
	* @param json true if json, false if csv
	* @return true if success, false if error
	*/
	static bool DataToString(const data_info_t& info, const p_data_hdr_t* hdr, const uint8_t* datasetBuffer, data_mapping_string_t stringBuffer, unsigned int arrayIndex = 0, bool json = false);

	/**
	* Convert a variable to a string
	* @param dataType data type
	* @param dataFlags data flags 
	* @param dataBuffer data pointer
	* @param dataSize size of data at data pointer
	* @param stringBuffer the buffer to hold the converted string
	* @param conversion conversion of value (i.e. rad2deg)
	* @param json true if json, false if csv
	* @return true if success, false if error
	*/
	static bool VariableToString(eDataType dataType, eDataFlags dataFlags, const uint8_t* dataBuffer, uint32_t dataSize, data_mapping_string_t stringBuffer, double conversion = 1.0, bool json = false);

	/**
	* Get a timestamp from data if available
	* @param hdr data header
	* @param buf data buffer
	* @return timestamp, or 0.0 if no timestamp available
	*/
    static double Timestamp(const p_data_hdr_t* hdr, const uint8_t* buf);

	/**
	* Get a timestamp from data if available.  If not, use the current local time.
	* @param hdr data header
	* @param buf data buffer
	* @return timestamp, or current local time if no timestamp available
	*/
	static double TimestampOrCurrentTime(const p_data_hdr_t* hdr, const uint8_t* buf);

	/**
	* Check whether field data can be retrieved given a data packet
	* @param info metadata for the field to get
	* @param arrayIndex index into array
	* @param hdr packet header
	* @param buf packet buffer
	* @return pointer to get data if valid or NULL if not valid.
	*/
	static const uint8_t* FieldData(const data_info_t& info, uint32_t arrayIndex, const p_data_hdr_t* hdr, const uint8_t* buf);

protected:
	static const char* const m_dataIdNames[];

	data_set_t m_data_set[DID_COUNT];

    #define PROTECT_UNALIGNED_ASSIGNS
    template<typename T>
    static inline void protectUnalignedAssign(void* out, T in) {
		// This gets called from cISDataMappings::StringToVariable
		if (errno == ERANGE) {
        	// std::cerr << "Underflow or overflow error: value is out of range" << std::endl;
        	return;		// abort to prevent use of invalid number: -1 (0xFFFFFFFFFFFFFFFF)
    	}
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

