/**
 * @file util.h 
 * @brief General Utility functions. Most of these functions should be static, as we don't want/need a "Util" instance running around.
 *
 * @author Kyle Mallory on 3/14/24.
 * @copyright Copyright (c) 2024 Inertial Sense, Inc. All rights reserved.
 */

#ifndef IS_SDK__UTIL_H
#define IS_SDK__UTIL_H

#include <string>
#include <vector>
#include <memory>
#include <stdexcept>
#include <sstream>
#include <functional>

#include "ISComm.h"

namespace utils {
    /**
     * @brief Trims left-side/leading characters (whitespace by default) from the passed string; this modifies the string, in place.
     * Locates the first characters which are not within the set of ws, and removes all characters from the start upto that first
     * non-matching character.
     *
     * @param s the string to be trimmed
     * @param t a set of characters, which will be removed if they exists
     * @return a reference to the input string.
     */
    std::string& ltrim(std::string& s, const char* t = " \t\n\r\f\v");

    /**
     * @brief Trims right-side/trailing characters (whitespace by default) from the passed string; this modifies the string, in place.
     * Locates the last characters of the string which are not within the set of ws, and removes all characters from that position
     * until the end of the string non-matching character.
     *
     * @param s the string to be trimmed
     * @param t a set of characters, which will be removed if they exists
     * @return a reference to the input string.
     */
    std::string& rtrim(std::string& s, const char* t = " \t\n\r\f\v");

    /**
     * @brief Trims left-size/leading and right-side/trailing characters (whitespace by default) from the passed string; this modifies
     * the string, in place. Call both rtrim() and ltrim() in a single call.
     *
     * @param s the string to be trimmed
     * @param t a set of characters, which will be removed if they exists
     * @return a reference to the input string.
     */
    std::string& trim(std::string& s, const char* t = " \t\n\r\f\v");


    /**
     * ltrim() equivalent which makes a new copy, and does not modify the original
     * @param s the string to be trimmed
     * @param t a set of characters, which will be removed if they exists
     * @return the modified/trimmed copy of the original input string.
     */
    std::string ltrim_copy(std::string s, const char* t = " \t\n\r\f\v");

    /**
     * rtrim() equivalent which makes a new copy, and does not modify the original
     * @param s the string to be trimmed
     * @param t a set of characters, which will be removed if they exists
     * @return the modified/trimmed copy of the original input string.
     */
    std::string rtrim_copy(std::string s, const char* t = " \t\n\r\f\v");

    /**
     * trim() equivalent which makes a new copy, and does not modify the original
     * @param s the string to be trimmed
     * @param t a set of characters, which will be removed if they exists
     * @return the modified/trimmed copy of the original input string.
     */
    std::string trim_copy(std::string s, const char* t = " \t\n\r\f\v");

    /**
     * Performs sprintf-type formatting, using a std:string for the format string, and outputting a std::string
     * @tparam Args
     * @param format
     * @param args
     * @return
     */
    template<typename ... Args>
    std::string string_format(const std::string& format, Args ... args) {
        int size_s = std::snprintf(nullptr, 0, format.c_str(), args ...) + 1; // Extra space for '\0'
        if (size_s <= 0) {
            throw std::runtime_error("Error during formatting.");
        }
        auto size = static_cast<size_t>(size_s);
        std::unique_ptr<char[]> buf(new char[ size ]);
        std::snprintf(buf.get(), size, format.c_str(), args ...);
        return std::string(buf.get(), buf.get() + size - 1); // We don't want the '\0' inside
    }

    /**
     * Combine all elements of a container denoted by the start and ending iterators, to join into a
     *   single string, using the specified delimiter
     * @tparam T the type of container/iterator
     * @param v a copy of the iterator denoting the first element to join
     * @param end reference to the iterator denoting the last element to join
     * @param delimiter a delimiter string to be placed between each element in the output string
     * @return a string of all joined elements
     */
    template <typename T>
    std::string join_to_string(T begin, const T& end, const std::string& delimiter) {
        std::ostringstream s;
        for (T cur = begin; cur != end; cur++) {
            if (cur != begin) s << delimiter.c_str();
            s << *cur;
        }
        return s.str();
    }

    /**
     * Combine all elements of a container into a single string, using the specified delimiter
     * @tparam T the type of container
     * @param v reference to the container of elements to join
     * @param delimiter a delimiter string to be placed between each element in the output string
     * @return a string of all joined elements
     */
    template <typename T>
    std::string join_to_string(const T& v, const std::string& delimiter) {
        std::ostringstream s;
        for (const auto& i : v) {
            if (&i != &v[0]) {
                s << delimiter;
            }
            s << i;
        }
        return s.str();
    }

    /**
     * Parses a string of delimited values (ie, x.x.x.x) and populate the values into a passed
     * std::array of type T and size N. If the string contains fewer than N numbers elements,
     * the remaining elements are not assigned (you should initialize vOut before calling this
     * function). At most N elements will be parsed. The lamba is used to convert the parsed
     * substring into the value of type T.
     *
     * This is primarily used to parse versions and ip address, and the template provides
     *  default values that support this usage (T = uint8_t, N = 4). If you wish to use it for
     *  other types, remember to set the template parameters.
     *
     * @param s the string to split
     * @param vOut a std::array<T,n> each element containing a parsed value.
     * @param d the delimiters to use when splitting (if multiple, will be separated on ANY)
     *  (defaults to '.')
     * @param lambda a lambda used to convert the parsed substring to a value of type T (defaults
     *  to stoi(), returning a decimal
     * @return returns the number of elements parsed.
     */
    template <typename T=uint8_t, int N=4>
    int split_from_string(const std::string& s, T vOut[N], const char* d = ".", std::function<T(const std::string&)> lambda = [](const std::string& ss) -> T { return stoi(ss); } ) {
        long unsigned int start = 0, n = 0, end = 0;
        while ( (end = s.find_first_of(d, start)) != std::string::npos ) {
            vOut[n++] = lambda(s.substr(start, end - start));
            start = end + 1;
        }
        vOut[n++] = lambda(s.substr(start));
        return n;
    }


    /**
     * Splits the passed string into a vector of strings, delimited by delimiter.
     * @param str the string to be split
     * @param delimiter the substring to use as a delimiter
     * @return a vector of strings
     */
    std::vector<std::string> split_string(const std::string& str, const std::string& delimiter);

    std::string raw_hexdump(const char* raw_data, int bytesLen, int bytesPerLine);
    std::string did_hexdump(const char *raw_data, const p_data_hdr_t& hdr, int bytesPerLine);

    enum dev_info_fmt_e : uint16_t {
        DV_BIT_SERIALNO         = 0x001,        //!< serial number
        DV_BIT_FIRMWARE_VER     = 0x002,        //!< firmware version w/ optional release type
        DV_BIT_HARDWARE_INFO    = 0x004,        //!< hdw type & version
        DV_BIT_BUILD_KEY        = 0x008,        //!< build key and build number
        DV_BIT_BUILD_DATE       = 0x010,        //!< build date
        DV_BIT_BUILD_TIME       = 0x020,        //!< build time
        DV_BIT_BUILD_COMMIT     = 0x040,        //!< repo hash & build status (dirty)
        DV_BIT_ADDITIONAL_INFO  = 0x100,        //!< additional info
    };

    std::string getHardwareAsString(const dev_info_t& devInfo);
    std::string getFirmwareAsString(const dev_info_t& devInfo);
    std::string getBuildAsString(const dev_info_t& devInfo, uint16_t flags = -1);

    std::string getCurrentTimestamp();
    std::string devInfoToString(const dev_info_t& devInfo, uint16_t flags = -1);
    uint16_t devInfoFromString(const std::string& str, dev_info_t& devInfo);
    int parseStringVersion(const std::string& vIn, uint8_t vOut[4]);
    uint64_t intDateTimeFromDevInfo(const dev_info_t& a, bool useMillis = false);
    bool devInfoFromFirmwareImage(std::string imgFilename, dev_info_t& devInfo);
    bool isDevInfoCompatible(const dev_info_t& a, const dev_info_t& b);
    bool compareFirmwareVersions(const dev_info_t& a, const dev_info_t& b);
};


#endif //IS_SDK__UTIL_H
