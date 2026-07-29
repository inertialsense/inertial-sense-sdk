/**
 * @file ISDataMappings.h
 * @brief Runtime field-level reflection for every DID data struct: per-field metadata
 *        (data_info_t: offset/size/type/units/description/render functions), the DataMapper
 *        builder classes used to populate that metadata for a struct, and cISDataMappings'
 *        static lookup/conversion API (name<->DID, string<->binary field conversion, YAML
 *        round-tripping) built on top of it.
 *
 * @author Inertial Sense, Inc.
 * @copyright Copyright (c) 2014-2025 Inertial Sense, Inc. All rights reserved.
 */

/*
MIT LICENSE

Copyright (c) 2014-2025 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#ifndef __ISDATAMAPPINGS_H_
#define __ISDATAMAPPINGS_H_

#include <any>
#include <cinttypes>
#include <cstdio>
#include <map>
#include <string>
#include <typeinfo>
#include <vector>
#include <functional>
#include <type_traits>
#include <cstdint>
#include <cstddef>

#include "com_manager.h"

#if PLATFORM_IS_EMBEDDED == 0
    #include <yaml-cpp/yaml.h>
#endif

#ifndef CHAR_BIT
#define CHAR_BIT 8
#endif

#if defined(INCLUDE_LUNA_DATA_SETS)
#include "luna_data_sets.h"
#endif

#define IS_DATA_MAPPING_MAX_STRING_LENGTH 2048

/** The primitive wire/storage type of a mapped field (see data_info_t::type). */
typedef enum
{
    DATA_TYPE_INT8,     //!< 8-bit  signed    integer
    DATA_TYPE_UINT8,    //!< 8-bit  unsigned  integer
    DATA_TYPE_INT16,    //!< 16-bit signed    integer
    DATA_TYPE_UINT16,   //!< 16-bit unsigned  integer
    DATA_TYPE_INT32,    //!< 32-bit signed    integer
    DATA_TYPE_UINT32,   //!< 32-bit unsigned  integer
    DATA_TYPE_INT64,    //!< 64-bit signed    integer
    DATA_TYPE_UINT64,   //!< 64-bit unsigned  integer
    DATA_TYPE_F32,      //!< 32-bit float
    DATA_TYPE_F64,      //!< 64-bit float
    DATA_TYPE_STRING,   //!< variable-length text; size is caller-supplied (not in s_eDataTypeSize)
    DATA_TYPE_BINARY,   //!< variable-length raw bytes; size is caller-supplied (not in s_eDataTypeSize)

    DATA_TYPE_COUNT     //!< number of defined data types; must be last
} eDataType;

/** Bitmask of rendering/behavior flags for a mapped field (see data_info_t::flags). */
typedef enum
{
    DATA_FLAGS_FIXED_DECIMAL_MASK        = 0x0000000F,  //!< bits 0-3: mask isolating the fixed-decimal-place count below
    DATA_FLAGS_FIXED_DECIMAL_0           = 0x00000001,  //!< render with 0 decimal places
    DATA_FLAGS_FIXED_DECIMAL_1           = 0x00000002,  //!< render with 1 decimal place
    DATA_FLAGS_FIXED_DECIMAL_2           = 0x00000003,  //!< render with 2 decimal places
    DATA_FLAGS_FIXED_DECIMAL_3           = 0x00000004,  //!< render with 3 decimal places
    DATA_FLAGS_FIXED_DECIMAL_4           = 0x00000005,  //!< render with 4 decimal places
    DATA_FLAGS_FIXED_DECIMAL_5           = 0x00000006,  //!< render with 5 decimal places
    DATA_FLAGS_FIXED_DECIMAL_6           = 0x00000007,  //!< render with 6 decimal places
    DATA_FLAGS_FIXED_DECIMAL_7           = 0x00000008,  //!< render with 7 decimal places
    DATA_FLAGS_FIXED_DECIMAL_8           = 0x00000009,  //!< render with 8 decimal places
    DATA_FLAGS_FIXED_DECIMAL_9           = 0x0000000A,  //!< render with 9 decimal places
    DATA_FLAGS_FIXED_DECIMAL_10          = 0x0000000B,  //!< render with 10 decimal places
    DATA_FLAGS_FIXED_DECIMAL_11          = 0x0000000C,  //!< render with 11 decimal places
    DATA_FLAGS_FIXED_DECIMAL_12          = 0x0000000D,  //!< render with 12 decimal places
    DATA_FLAGS_FIXED_DECIMAL_13          = 0x0000000E,  //!< render with 13 decimal places
    DATA_FLAGS_FIXED_DECIMAL_14          = 0x0000000F,  //!< render with 14 decimal places
    DATA_FLAGS_READ_ONLY                 = 0x00000010,  //!< bit 4: field is read-only; StringToData()/StringToVariable() should refuse to write it
    DATA_FLAGS_HIDDEN                    = 0x00000020,  //!< bit 5: do not print to screen
    DATA_FLAGS_DISPLAY_HEX               = 0x00000100,  //!< bit 8: render the value in hexadecimal
    DATA_FLAGS_ANGLE                     = 0x00000200,  //!< bit 9: field is an angle; supports unwrapping
    DATA_FLAGS_DECOR_ROLL_MASK           = 0x000F0000,  //!< bits 16-19: mask isolating the "decoration roll" flags below
    DATA_FLAGS_INS_STATUS                = 0x00010000,  //!< bit 16: field is an insStatus bitmask; render with INS-status decoration
    DATA_FLAGS_GNSS_STATUS                = 0x00020000,  //!< bit 17: field is a gnssStatus bitmask; render with GNSS-status decoration
} eDataFlags;

/** Forward declaration; see the full definition below for field documentation. */
struct data_info_t;

/**
 * This is the template for a Render function, which is a callable used by this class to so custom rendering of data values suitable for individual UI's, etc.
 * @param info is a reference to the specific DID+field which it to be rendered - note that data_info_t DOES NOT contain the data/value to be rendered
 * @param value the value to be rendered - the type is extracted from the info described in data_info_t
 * @param arrayIdx the array (if > 0) into an array of of fields
 * @param flags are user-defined flags which can be used to alter how the data is rendered - to be interpreted by the renderer though some UIs (cltool, evaltool) may have known flags (HTML, etc).
 * @param userData is an opaque data pointer which maybe useful in some rendering contexts - typically called with the ISDevice* instance (if there is one) which holds this value
 *    Note individual renderers are expected to know how to handle userData - it is purely opaque.
 */
using RenderFunction = std::function<std::string(const data_info_t& info, std::any value, int arrayIdx, int flags)>;

/**
 * Metadata about a single field of a DID struct: where it lives in memory, its wire type,
 * display units/description, and the render callbacks used to convert its raw value to text.
 * One instance exists per registered field, built by DataMapper::AddMember()/AddArray() (or the
 * offset-based AddMember2()/AddArray2()) and owned by the enclosing data_set_t.
 */
struct data_info_t
{
    uint32_t    offset;                         //!< Offset in to the data struct where this specific field resides
    uint32_t    size;                           //!< Size of this field, in bytes (the total size, if this is an array)
    eDataType   type;                           //!< Type indicator for this field (Int, Array, Float, etc)
    uint32_t    arraySize;                      //!< Number of elements in array.  Zero for single/non-array elements.
    uint32_t    elementSize;                    //!< The size of each element in an array, in bytes
    eDataFlags  flags;                          //!< Flags about this data type, such as rendering options, etc
    std::string name;                           //!< The name of the field
    std::vector<std::string> units;             //!< The Units that this field should be displayed in (after conversion)
    std::vector<std::string> description;       //!< A description for this field; what it means, how to interpret its values, etc.
    double conversion;                          //!< A scalar that the raw value is divided by prior to converting to a string
    RenderFunction renderBasic =                //!< A function to render / convert a value to a simple string - VariableToString() calls this function - this should not include newlines, etc.
            [](const data_info_t& info, std::any val, int arrayIdx, int flags) -> std::string { (void)info; (void)val; (void)arrayIdx; (void)flags; return ""; };
    RenderFunction renderExtended =             //!< A function to render a value to string using advanced logic and formatting - this may include newlines, html formatting, etc. can be used for tooltips, and useful for bitmasks, etc and other advanced formatting
            [](const data_info_t& info, std::any val, int arrayIdx, int flags) -> std::string { (void)info; (void)val; (void)arrayIdx; (void)flags; return ""; };
};

/**
 * Table schema: one entry per eDataType enumerator (indexed by the enum value), giving the
 * fixed size in bytes of that primitive type. DATA_TYPE_STRING and DATA_TYPE_BINARY are
 * variable-length, so their entries are 0 -- the actual size must be supplied by the caller
 * (e.g. via DataMapper::AddMember2()'s typeSize parameter) rather than looked up here.
 */
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

/**
 * Table schema: one entry per eDataType enumerator (indexed by the enum value), giving the
 * number of characters (including the "0x" prefix) needed to render that type in hexadecimal
 * (see DATA_FLAGS_DISPLAY_HEX). Variable-length types (DATA_TYPE_STRING/DATA_TYPE_BINARY) are
 * not hex-rendered, so their entries are 0.
 */
CONST_EXPRESSION uint32_t s_eDataTypeHexStringSize[DATA_TYPE_COUNT] =
{   // Number of characters in hexidecimal string including prefix "0x"
    (uint32_t)4,    // 0x00
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
extern const unsigned char g_asciiToLowerMap[256];  //!< lookup table mapping each byte value to its ASCII-lowercased equivalent, indexed by the byte itself
#endif

/**
 * Case-insensitive comparator for std::find and other functions
 */
struct sCaseInsensitiveCompare
{
    /** Case-insensitive less-than comparator for a single character, using g_asciiToLowerMap. */
    struct nocase_compare
    {
        /** @param c1 the first character @param c2 the second character @return true if the lowercased c1 is less than the lowercased c2 */
        bool operator() (const unsigned char& c1, const unsigned char& c2) const
        {
            return g_asciiToLowerMap[c1] < g_asciiToLowerMap[c2];
        }
    };

    /**
     * Case-insensitive less-than comparison, hand-coded for NMEA-style ASCII strings (no
     * unicode/locale handling) for roughly 3x the speed of std::lexicographical_compare.
     * @param s1 the first string
     * @param s2 the second string
     * @return true if s1 is lexicographically less than s2, ignoring case
     */
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

typedef std::map<std::string, data_info_t, sCaseInsensitiveCompare>     map_name_to_info_t;             //!< map of field name to data info
typedef std::map<uint32_t, data_info_t*>                                map_index_to_info_t;            //!< map of field index to data info pointer
typedef std::map<uint32_t, data_info_t*>                                map_element_to_info_t;          //!< map of element index to data info pointer
typedef std::map<uint32_t, uint32_t>                                    map_element_to_array_size_t;    //!< map of element index to array size
typedef char data_mapping_string_t[IS_DATA_MAPPING_MAX_STRING_LENGTH];  //!< fixed-capacity buffer used by DataToString()/VariableToString() for a single rendered field value

/**
 * The complete field-mapping table for one DID: every registered field, indexed by name,
 * sequential field index, and flattened array-element index. Populated by a DataMapper<Dtype>
 * bound to this entry in cISDataMappings::m_data_set[did], and queried via cISDataMappings'
 * static NameToInfoMap()/IndexToInfoMap()/ElementToInfo() etc.
 */
typedef struct
{
    uint32_t                    size;                  //!< sizeof the mapped DID struct (set by DataMapper's constructor)
    map_name_to_info_t          nameToInfo;             //!< every registered field, keyed by field name (case-insensitive)
    map_index_to_info_t         indexToInfo;            //!< every registered field, keyed by its sequential registration index
    map_element_to_info_t       elementToInfo;          //!< every registered field, keyed by flattened element index (arrays contribute one entry per element)
    map_element_to_array_size_t elementToArraySize;      //!< for each flattened element index, the array index (0 for scalar fields) within its field
    uint32_t                    elementCount;           //!< total number of flattened elements across all registered fields
    const data_info_t*          timestampFields;        //!< the field(s), if any, used to derive a record's timestamp (see cISDataMappings::Timestamp())
} data_set_t;

/**
 * Default data_info_t::renderBasic implementation: converts value to a simple string with no
 * newlines/formatting (per data_info_t::renderBasic's contract). Used by VariableToString().
 * @param info metadata for the field being rendered
 * @param value the raw value to render, as extracted by dataToStdAny()
 * @param arrayIdx the array index (0 for scalar fields) of the specific value being rendered
 * @param flags renderer-specific flags, interpreted by this function
 * @return the rendered string
 */
std::string renderVariableToString(const data_info_t& info, std::any value, int arrayIdx, int flags);

/**
 * Default data_info_t::renderExtended implementation: renders value with additional
 * statistics/formatting (may include newlines/HTML), for tooltips and advanced display contexts.
 * @param info metadata for the field being rendered
 * @param value the raw value to render, as extracted by dataToStdAny()
 * @param arrayIdx the array index (0 for scalar fields) of the specific value being rendered
 * @param flags renderer-specific flags, interpreted by this function
 * @return the rendered string
 */
std::string renderVariableAndStatsToString(const data_info_t& info, std::any value, int arrayIdx, int flags);

/**
 * Specialized renderExtended implementation for RTK configuration bitmask fields, decoding
 * individual bits into a human-readable description.
 * @param info metadata for the field being rendered
 * @param value the raw value to render, as extracted by dataToStdAny()
 * @param arrayIdx the array index (0 for scalar fields) of the specific value being rendered
 * @param flags renderer-specific flags, interpreted by this function
 * @return the rendered string
 */
std::string renderRTKCfgBits(const data_info_t& info, std::any value, int arrayIdx, int flags);

/**
 * Builder used to populate one data_set_t entry with per-field data_info_t metadata for a
 * concrete DID struct (Dtype). Typically constructed on the stack for the duration of a single
 * DID's registration; each AddMember()/AddArray()/etc. call appends one field, and the
 * destructor asserts that the total size of all registered fields equals sizeof(Dtype), catching
 * a struct/mapping mismatch immediately rather than at first use.
 */
template <typename Dtype>
class DataMapper
{
public:
    typedef Dtype MAP_TYPE;

    /**
     * @param data_set the SDK-wide array of data sets; data_set[did] is the entry this mapper populates
     * @param did the DID being registered
     */
    DataMapper(data_set_t data_set[DID_COUNT], uint32_t did) : ds(data_set[did]), totalSize(0), memberCount(0)
    {
        data_set[did].size = structSize = sizeof(MAP_TYPE);
    }

    /** Asserts (and logs to stderr) if the total size of all registered fields doesn't equal sizeof(MAP_TYPE). */
    ~DataMapper()
    {
        if (totalSize != structSize)
        {
            fprintf(stderr, "DataMapper ERROR [%s]: mapped size %u != struct size %u (diff %d bytes)\n",
                typeid(MAP_TYPE).name(), totalSize, structSize, (int)structSize - (int)totalSize);
            assert(false && "Size of mapped fields does not match struct size");
        }
    }

    /**
     * Registers a single (non-array) scalar field, located via a pointer-to-member so the offset
     * is computed by the compiler rather than passed by hand.
     * @param name the field's name, used for name-based lookup
     * @param member pointer-to-member (e.g. &MAP_TYPE::fieldName) identifying the field's location
     * @param type the field's wire/storage type
     * @param units the display units for this field, if any
     * @param description a human-readable description of the field
     * @param flags an eDataFlags bitmask controlling rendering/behavior
     * @param conversion a scalar divisor applied to the raw value before display
     * @return a reference to the newly-registered data_info_t, for further customization (e.g. overriding renderBasic)
     */
    template <typename MemberType>
    data_info_t& AddMember(const std::string& name,
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
        uint32_t arraySize = 0;     // Zero for single element
        uint32_t elementSize = size;

        std::vector<std::string> unitsCopy = { units };
        if (unitsCopy.size() && unitsCopy.size() < arraySize)
        {    // Extend the units vector to match the array size
            std::string lastUnit = unitsCopy.empty() ? "" : unitsCopy.back();
            unitsCopy.resize(arraySize, lastUnit);
        }
        std::vector<std::string> descriptionCopy = { description };
        if (descriptionCopy.size() && descriptionCopy.size() < arraySize)
        {    // Extend the description vector to match the array size
            std::string lastDesc = descriptionCopy.empty() ? "" : descriptionCopy.back();
            descriptionCopy.resize(arraySize, lastDesc);
        }

        // Populate the map with the new entry
        ds.nameToInfo[name] = {
            offset,
            size,
            type,
            arraySize,
            elementSize,
            eDataFlags(flags), 
            name, 
            std::vector<std::string>{units}, 
            std::vector<std::string>{description}, 
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

        dinfo->renderBasic = renderVariableToString;
        dinfo->renderExtended = renderVariableAndStatsToString;
        return *dinfo;
    }

    /**
     * Registers an array field, located via a pointer-to-member so the offset is computed by the
     * compiler rather than passed by hand.
     * @param name the field's name, used for name-based lookup
     * @param member pointer-to-member (e.g. &MAP_TYPE::fieldName) identifying the array's location
     * @param type the wire/storage type of each element
     * @param arraySize the number of elements in the array
     * @param units per-element display units (extended to arraySize entries if shorter)
     * @param description per-element descriptions (extended to arraySize entries if shorter)
     * @param flags an eDataFlags bitmask controlling rendering/behavior
     * @param conversion a scalar divisor applied to each raw value before display
     * @return a reference to the newly-registered data_info_t, for further customization (e.g. overriding renderBasic)
     */
    template <typename MemberType>
    data_info_t& AddArray(const std::string& name,
        MemberType member,
        eDataType type,
        uint32_t arraySize,
        const std::vector<std::string>& units = {},
        const std::vector<std::string>& description = {},
        int flags = 0,
        double conversion = 1.0)
    {
        using FieldType = typename std::remove_cv<typename std::remove_reference<decltype(((MAP_TYPE*)nullptr)->*member)>::type>::type;
        uint32_t offset = (uint32_t)(uintptr_t)&(((MAP_TYPE*)nullptr)->*member);
        uint32_t size = (uint32_t)sizeof(FieldType);
        uint32_t elementSize = size/arraySize;

        std::vector<std::string> unitsCopy = { units };
        if (unitsCopy.size() && unitsCopy.size() < arraySize)
        {    // Extend the units vector to match the array size
            std::string lastUnit = unitsCopy.empty() ? "" : unitsCopy.back();
            unitsCopy.resize(arraySize, lastUnit);
        }
        std::vector<std::string> descriptionCopy = { description };
        if (descriptionCopy.size() && descriptionCopy.size() < arraySize)
        {    // Extend the description vector to match the array size
            std::string lastDesc = descriptionCopy.empty() ? "" : descriptionCopy.back();
            descriptionCopy.resize(arraySize, lastDesc);
        }

        // Populate the map with the new entry
        ds.nameToInfo[name] = {
            offset,
            size, 
            type,
            arraySize,
            elementSize,
            eDataFlags(flags),
            name, 
            unitsCopy, 
            descriptionCopy, 
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

        dinfo->renderBasic = renderVariableToString;
        dinfo->renderExtended = renderVariableAndStatsToString;
        return *dinfo;
    }

    /**
     * Registers a single (non-array) scalar field by explicit byte offset, for cases where a
     * pointer-to-member isn't available or convenient (e.g. offsets computed programmatically).
     * @param name the field's name, used for name-based lookup
     * @param offset the field's byte offset within the mapped struct
     * @param type the field's wire/storage type
     * @param units the display units for this field, if any
     * @param description a human-readable description of the field
     * @param flags an eDataFlags bitmask controlling rendering/behavior
     * @param conversion a scalar divisor applied to the raw value before display
     * @param typeSize explicit field size in bytes; required for DATA_TYPE_STRING/DATA_TYPE_BINARY (0 = look up s_eDataTypeSize[type])
     * @return a reference to the newly-registered data_info_t, for further customization (e.g. overriding renderBasic)
     */
    data_info_t& AddMember2(const std::string& name,
        uint32_t offset,
        eDataType type,
        const std::string& units = "",
        const std::string& description = "",
        int flags = 0,
        double conversion = 1.0,
        uint32_t typeSize = 0)
    {
        uint32_t size = (typeSize ? typeSize : s_eDataTypeSize[type]);
        uint32_t arraySize = 0;     // Zero for single element
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
            std::vector<std::string>{units}, 
            std::vector<std::string>{description}, 
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

        dinfo->renderBasic = renderVariableToString;
        dinfo->renderExtended = renderVariableAndStatsToString;
        return *dinfo;
    }

    /**
     * Registers an array field by explicit byte offset, for cases where a pointer-to-member
     * isn't available or convenient (e.g. offsets computed programmatically).
     * @param name the field's name, used for name-based lookup
     * @param offset the array's starting byte offset within the mapped struct
     * @param type the wire/storage type of each element
     * @param arraySize the number of elements in the array
     * @param units per-element display units (extended to arraySize entries if shorter)
     * @param description per-element descriptions (extended to arraySize entries if shorter)
     * @param flags an eDataFlags bitmask controlling rendering/behavior
     * @param conversion a scalar divisor applied to each raw value before display
     * @param typeSize explicit per-element size in bytes; required for DATA_TYPE_STRING/DATA_TYPE_BINARY (0 = look up s_eDataTypeSize[type])
     * @return a reference to the newly-registered data_info_t, for further customization (e.g. overriding renderBasic)
     */
    data_info_t& AddArray2(const std::string& name,
        uint32_t offset,
        eDataType type,
        uint32_t arraySize,
        const std::vector<std::string>& units = {},
        const std::vector<std::string>& description = {},
        int flags = 0,
        double conversion = 1.0,
        uint32_t typeSize = 0)
    {
        uint32_t elementSize = (typeSize ? typeSize : s_eDataTypeSize[type]);
        uint32_t size = elementSize * arraySize;

        std::vector<std::string> unitsCopy = units;
        if (unitsCopy.size() && unitsCopy.size() < arraySize)
        {    // Extend the units vector to match the array size
            std::string lastUnit = unitsCopy.empty() ? "" : unitsCopy.back();
            unitsCopy.resize(arraySize, lastUnit);
        }
        std::vector<std::string> descriptionCopy = description;
        if (descriptionCopy.size() && descriptionCopy.size() < arraySize)
        {    // Extend the description vector to match the array size
            std::string lastDesc = descriptionCopy.empty() ? "" : descriptionCopy.back();
            descriptionCopy.resize(arraySize, lastDesc);
        }
        
        // Populate the map with the new entry
        ds.nameToInfo[name] = {
            offset,
            size, 
            type,
            arraySize,
            elementSize,
            eDataFlags(flags),
            name,
            unitsCopy,
            descriptionCopy,
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
        dinfo->renderBasic = renderVariableToString;
        dinfo->renderExtended = renderVariableAndStatsToString;
        return *dinfo;
    }

    /**
     * Registers a 3-element [lat, lon, alt] array field (degrees, degrees, meters) with
     * per-element units/descriptions and fixed 8-decimal precision pre-applied.
     * @param name the field's name, used for name-based lookup
     * @param offset the array's starting byte offset within the mapped struct
     * @param description base description, prefixed onto "latitude"/"longitude"/altitude
     * @param descriptionAltitude description suffix for the altitude element
     * @param flags an eDataFlags bitmask controlling rendering/behavior (fixed-decimal bits are overridden)
     * @return a reference to the newly-registered data_info_t, for further customization (e.g. overriding renderBasic)
     */
    data_info_t& AddLlaDegM(const std::string& name,
        uint32_t offset,
        const std::string& description = "",
        const std::string& descriptionAltitude = "",
        int flags = 0)
    {
        eDataType type = DATA_TYPE_F64;
        flags &= ~DATA_FLAGS_FIXED_DECIMAL_MASK;
        return AddArray2(name, offset, type, 3, {"°", "°", "m"}, {description + " latitude", description + " longitude", description + " " + descriptionAltitude}, flags | DATA_FLAGS_FIXED_DECIMAL_8);
    }

    /**
     * Registers a 3-element [X, Y, Z] vector array field, with per-element descriptions derived
     * from description.
     * @param name the field's name, used for name-based lookup
     * @param offset the array's starting byte offset within the mapped struct
     * @param type the wire/storage type of each element
     * @param units the display units, applied to all three elements
     * @param description base description, prefixed with "X "/"Y "/"Z " per element
     * @param flags an eDataFlags bitmask controlling rendering/behavior
     * @param conversion a scalar divisor applied to each raw value before display
     * @return a reference to the newly-registered data_info_t, for further customization (e.g. overriding renderBasic)
     */
    data_info_t& AddVec3Xyz(const std::string& name,
        uint32_t offset,
        eDataType type,
        const std::string& units = "",
        const std::string& description = "",
        int flags = 0,
        double conversion = 1.0)
    {
        return AddArray2(name, offset, type, 3, {units}, {"X "+description, "Y "+description, "Z "+description}, flags, conversion);
    }

    /**
     * Registers a 3-element [Roll, Pitch, Yaw] vector array field, with per-element descriptions
     * derived from description.
     * @param name the field's name, used for name-based lookup
     * @param offset the array's starting byte offset within the mapped struct
     * @param type the wire/storage type of each element
     * @param units the display units, applied to all three elements
     * @param description base description, prefixed with "Roll "/"Pitch "/"Yaw " per element
     * @param flags an eDataFlags bitmask controlling rendering/behavior
     * @param conversion a scalar divisor applied to each raw value before display
     * @return a reference to the newly-registered data_info_t, for further customization (e.g. overriding renderBasic)
     */
    data_info_t& AddVec3Rpy(const std::string& name,
        uint32_t offset,
        eDataType type,
        const std::string& units = "",
        const std::string& description = "",
        int flags = 0,
        double conversion = 1.0)
    {
        return AddArray2(name, offset, type, 3, {units}, {"Roll "+description, "Pitch "+description, "Yaw "+description}, flags, conversion);
    }

private:
    data_set_t& ds;             // data set reference
    uint32_t    structSize;     // size of data set struct. Used to compare against totalSize to ensure all members were included.
    uint32_t    totalSize;      // size of mapped fields
    uint32_t    memberCount;    // number of members in struct
};


/**
 * SN-8068: Array-of-struct element model + generic per-DID element identity.
 *
 * Some DIDs carry a fixed-capacity array of an inner struct whose populated count
 * varies message-to-message (e.g. DID_GNSS1_SAT's gnss_sat_sv_t sat[MAX_NUM_SATELLITES],
 * DID_PORT_MONITOR's port_stats_t port[NUM_SERIAL_PORTS]). Rather than registering one
 * flat field per (slot, member) pair, the inner fields are exposed ONCE as
 * "<struct>.<field>" and consumers fan a single field out across every populated element.
 *
 * Crucially the fan-out is keyed by a STABLE element IDENTITY (a satellite's PRN, a port's
 * id) rather than its array slot, because a given satellite migrates between sat[] slots
 * across messages. This is additive: existing mappings, names, and iteration order are
 * untouched.
 */
struct element_identity_t
{
    uint64_t    keyId = 0;          //!< Stable identity of this element across messages (e.g. gnssId<<8|svId, or port index). The array slot is NOT stable; this is.
    std::string label;              //!< Human label for legends / tables (e.g. "G14", "ser0").
    bool        valid = false;      //!< False when the slot is not populated or its identity cannot be decoded.
};

/** Given the raw record buffer and an element slot index, return that element's identity. */
using ElementIdentityFn = std::function<element_identity_t(const uint8_t* recordBuf, uint32_t slot)>;

/** Describes one array-of-struct member of a DID and how to address / identify its elements. */
struct array_struct_info_t
{
    std::string         structName;                     //!< Inner struct member name, e.g. "sat", "sig", "port".
    uint32_t            arrayBaseOffset = 0;            //!< Byte offset of element[0] within the DID struct.
    uint32_t            elementStride = 0;              //!< sizeof(inner element struct).
    uint32_t            maxElements = 0;                //!< Array capacity (fixed).
    uint32_t            countOffset = 0;                //!< Byte offset of the live-count field (numSats / numSigs / activePorts).
    eDataType           countType = DATA_TYPE_UINT32;   //!< Type of the live-count field.
    ElementIdentityFn   identity;                       //!< Per-element identity decoder (never null; default decoder keys on the slot index).
};


/**
 * Owns the field-mapping table (data_set_t m_data_set[DID_COUNT]) for every DID and exposes it
 * through a static lookup/conversion API: name<->DID, field lookup by name/index/offset/element,
 * and string<->binary field conversion (including YAML round-tripping). A process-wide singleton
 * (see s_map) built once and never destroyed, so other statics can safely query it during their
 * own static destruction.
 */
class cISDataMappings
{
public:
    cISDataMappings();

    virtual ~cISDataMappings() {}

    /** A single contiguous heap allocation tracked for later validation/leak-checking (see AppendMemoryUsage()). */
    struct MemoryUsage
    {
        uint8_t* ptr;    //!< pointer to the start of the allocation
        size_t size;     //!< size of the allocation, in bytes

        /** @return a pointer one-past-the-end of the allocation */
        uint8_t* end() const { return ptr + size; }
    };

    /**
    * Get a data set name from an id
    * @param did the data id to get a data set name from
    * @return data set name or NULL if not found
    */
    static const char* DataName(uint32_t did);

    /**
    * Get a data set id from a numeric or alphabetic string (i.e. "DID_INS_1" or "4")
    * @param string the string to convert to a data id
    * @return data set ID or NULL if not found
    */
    static uint32_t Did(std::string string);

    /**
     * @brief Convert a name to a data set id
     * 
     * @param name the name to convert
    * @return data set ID or NULL if not found
     */
    static uint32_t NameToDid(std::string name);

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
    * Get number of elements of a given data id.  Arrays get counted as multiple elements.
    * @param did the data id
    * @return number of elements or 0 if not found or unknown
    */
    static uint32_t ElementCount(uint32_t did);

    /**
    * SN-8068: Array-of-struct members of a DID (additive). These expose inner struct
    * fields once as "<struct>.<field>" and carry the metadata needed to fan a field out
    * across every populated element, keyed by identity.
    * @return pointer to the DID's array-struct descriptors, or NULL if the DID has none.
    */
    static const std::vector<array_struct_info_t>* ArrayStructFields(uint32_t did);

    /**
    * SN-8068: Number of populated elements in a specific record (reads the live-count
    * field named by info.countOffset/countType, clamped to info.maxElements).
    * @param info    an array-struct descriptor (from ArrayStructFields)
    * @param recordBuf pointer to the start of the decoded DID struct for one record
    * @return live element count, clamped to [0, maxElements]
    */
    static uint32_t ElementCountForRecord(const array_struct_info_t& info, const uint8_t* recordBuf);

    /**
    * SN-8068: Identity (stable keyId + human label) of one element slot in a record.
    * @param info    an array-struct descriptor (from ArrayStructFields)
    * @param recordBuf pointer to the start of the decoded DID struct for one record
    * @param slot    element index in [0, maxElements)
    * @return the element's identity; .valid is false if the slot is unpopulated/undecodable
    */
    static element_identity_t ElementIdentity(const array_struct_info_t& info, const uint8_t* recordBuf, uint32_t slot);

    /**
    * Get the default period multiple for the specified data set.  This is used to prevent non-rmc messages from streaming at 1ms periods (too high).  
    * @param did the data id
    * @return the default period multiple
    */
    static uint32_t DefaultPeriodMultiple(uint32_t did);

    /**
     * Extracts a single field value (or one array element) out of a raw record buffer into a
     * type-erased std::any, per info's type/offset/elementSize. Only numeric types are supported;
     * DATA_TYPE_STRING/DATA_TYPE_BINARY and out-of-range array indices yield an empty std::any.
     * @param dataBuff pointer to the start of the decoded DID struct for one record
     * @param info metadata for the field to extract
     * @param arrayIndex the array index to extract (0 for scalar fields)
     * @return the extracted value, or an empty std::any if info.type/arrayIndex is not supported
     */
    static std::any dataToStdAny(const uint8_t* dataBuff, const data_info_t& info, unsigned int arrayIndex = 0) {
        std::any value;

        if ((!info.arraySize && !arrayIndex) || (info.arraySize && (arrayIndex < info.arraySize))) {
            switch (info.type) {
                case DATA_TYPE_INT8:       value = std::make_any<int8_t>(*(int8_t*)(dataBuff + info.offset + (info.elementSize * arrayIndex)));       break;
                case DATA_TYPE_UINT8:      value = std::make_any<uint8_t>(*(uint8_t*)(dataBuff + info.offset + (info.elementSize * arrayIndex)));     break;
                case DATA_TYPE_INT16:      value = std::make_any<int16_t>(*(int16_t*)(dataBuff + info.offset + (info.elementSize * arrayIndex)));     break;
                case DATA_TYPE_UINT16:     value = std::make_any<uint16_t>(*(uint16_t*)(dataBuff + info.offset + (info.elementSize * arrayIndex)));   break;
                case DATA_TYPE_INT32:      value = std::make_any<int32_t>(*(int32_t*)(dataBuff + info.offset + (info.elementSize * arrayIndex)));     break;
                case DATA_TYPE_UINT32:     value = std::make_any<uint32_t>(*(uint32_t*)(dataBuff + info.offset + (info.elementSize * arrayIndex)));   break;
                case DATA_TYPE_INT64:      value = std::make_any<int64_t>(*(int64_t*)(dataBuff + info.offset + (info.elementSize * arrayIndex)));     break;
                case DATA_TYPE_UINT64:     value = std::make_any<uint64_t>(*(uint64_t*)(dataBuff + info.offset + (info.elementSize * arrayIndex)));   break;
                case DATA_TYPE_F32:        value = std::make_any<float>(*(float*)(dataBuff + info.offset + (info.elementSize * arrayIndex)));         break;
                case DATA_TYPE_F64:        value = std::make_any<double>(*(double*)(dataBuff + info.offset + (info.elementSize * arrayIndex)));       break;
                default:
                    break;
            }
        }
        return value;
    }

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
    static bool StringToData(const char* stringBuffer, int stringLength, const p_data_hdr_t* hdr, uint8_t* datasetBuffer, const data_info_t& info, unsigned int arrayIndex = 0, bool json = false, bool useConversion = true);

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
    static bool DataToString(const data_info_t& info, const p_data_hdr_t* hdr, const uint8_t* datasetBuffer, data_mapping_string_t stringBuffer, unsigned int arrayIndex = 0, bool json = false, bool useConversion = true);

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

#if defined(YAML_CPP_API)
    /*** Convert a did data set buffer to a YAML node
    * @param did the data ID
    * @param dataPtr pointer to the did buffer
    * @param output the YAML node representation of the data set
    * @param filter optional filter to apply to the output
    * @return true if successful, false if error
    */
    static bool DataToYaml(int did, const uint8_t* dataPtr, YAML::Node& output);
    static bool DataToYaml(int did, const uint8_t* dataPtr, YAML::Node& output, const YAML::Node& filter);

    /*** Convert a YAML node to a did data set buffer
    * @param did the data ID
    * @param yaml the YAML node to convert
    * @param dataPtr pointer to the did buffer
    * @param usageVec optional vector to hold memory usage information
    * @return true if successful, false if error
    */
    static bool YamlToData(int did, const YAML::Node& yaml, uint8_t* dataPtr, std::vector<MemoryUsage>* usageVec = nullptr);
#endif
    /**
     * @brief Convert a did data set buffer to a string representation
     *
     * @param did the data ID
     * @param dataPtr pointer to the did buffer
     * @param output the string representation of the data set
     * @param fields optional fields to include in the output
     * @return true if successful, false if error
     */
    static bool DidBufferToString(int did, const uint8_t* dataPtr, std::string &output, std::string fields="");

    /**
    * Convert a string representation to a did data set buffer
    * @param did the data ID
    * @param fields optional fields to include in the output
    * @param dataPtr pointer to the did buffer
    * @return true if successful, false if error
    */
    static bool StringToDidBuffer(int did, const std::string& fields, uint8_t* dataPtr);

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

    /**
     * returns the data_info_t of a field into DataSets by its offset into the data buffer,
     * @param did the Data ID the field belongs to
     * @param offset the offset from the start of the struct where the field belongs
     * @return a pointer to the nearest data_info_t which addresses this offset, without exceeding the offset,
     *   or NULL if there is no field info to covers the specified offset
     */
    static const data_info_t* FieldInfoByOffset(uint32_t did, uint16_t offset);

    /**
     * Records a heap allocation in usageVec, for later validation/leak-checking of YamlToData()'s
     * dynamically-allocated fields (e.g. variable-length strings/arrays).
     * @param usageVec the vector to append the new allocation record to
     * @param newPtr pointer to the start of the allocation
     * @param newSize size of the allocation, in bytes
     */
    static void AppendMemoryUsage(std::vector<MemoryUsage>& usageVec, void* newPtr, size_t newSize);

protected:

    /**
     * Parses and removes a trailing "[N]" array-index suffix from str, if present.
     * @param str the field-name string to parse; the "[N]" suffix, if found, is erased from it
     * @return the parsed array index N, or -1 if str had no array-index suffix
     */
    static int ExtractArrayIndex(std::string &str);

    /** Table schema: human-readable name for each DID, indexed by the DID value (see DataName()/Did()). */
    static const char* const m_dataIdNames[];

    data_set_t m_data_set[DID_COUNT];    //!< the field-mapping table for every DID, populated by this instance's constructor

#if PLATFORM_IS_EMBEDDED
    // on embedded we cannot new up C++ runtime until after free rtos has started
    static cISDataMappings* s_map;    //!< the process-wide singleton instance, lazily constructed on first use
#else
    // Heap-allocated pointer (never deleted) to avoid static destruction order fiasco:
    // other static objects (e.g. cDeviceLog containers) may call NameToInfoMap() during
    // their own destructors, after a value-typed s_map would have already been destroyed.
    static cISDataMappings* s_map;    //!< the process-wide singleton instance, lazily constructed on first use and never deleted
#endif

private:
    #define PROTECT_UNALIGNED_ASSIGNS    //!< when defined, protectUnalignedAssign() copies via memcpy instead of a direct pointer cast, avoiding unaligned-access faults on strict-alignment platforms
    template<typename T>
    static inline void protectUnalignedAssign(void* out, T in) {
        // This gets called from cISDataMappings::StringToVariable
        if (errno == ERANGE) {
            // std::cerr << "Underflow or overflow error: value is out of range" << std::endl;
            return;        // abort to prevent use of invalid number: -1 (0xFFFFFFFFFFFFFFFF)
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

};

#endif // __ISDATAMAPPINGS_H_