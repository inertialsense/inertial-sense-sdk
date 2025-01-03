/**
 * @file util.h 
 * @brief General Utility functions. Most of these functions should be static, as we don't want/need a "Util" instance running around.
 *
 * @author Kyle Mallory on 3/14/24.
 * @copyright Copyright (c) 2024 Inertial Sense, Inc. All rights reserved.
 */

#ifndef INERTIALSENSE_SDK__UTIL_H
#define INERTIALSENSE_SDK__UTIL_H

#include <string>
#include <vector>
#include <memory>
#include <stdexcept>
#include <sstream>

#include "ISComm.h"

namespace utils {
    std::string getCurrentTimestamp();
    std::string raw_hexdump(const char* raw_data, int bytesLen, int bytesPerLine);
    std::string did_hexdump(const char *raw_data, const p_data_hdr_t& hdr, int bytesPerLine);

    // trim from left
    std::string& ltrim(std::string& s, const char* t = " \t\n\r\f\v");
    // trim from right
    std::string& rtrim(std::string& s, const char* t = " \t\n\r\f\v");
    // trim from left & right
    std::string& trim(std::string& s, const char* t = " \t\n\r\f\v");

    // copying versions
    std::string ltrim_copy(std::string s, const char* t = " \t\n\r\f\v");
    std::string rtrim_copy(std::string s, const char* t = " \t\n\r\f\v");
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
     * returns a vector of sub-strings derived from str, seperated by delimiter
     * @param str
     * @param delimiter
     * @return
     */
    std::vector<std::string> split_string(const std::string& str, const std::string& delimiter);

    std::string devInfoToString(const dev_info_t& devInfo);
    bool devInfoFromString(const std::string& str, dev_info_t& devInfo);
    int parseStringVersion(const std::string& vIn, uint8_t vOut[4]);
    uint64_t intDateTimeFromDevInfo(const dev_info_t& a, bool useMillis = false);
    bool fillDevInfoFromFirmwareImage(std::string imgFilename, dev_info_t& devInfo);
    bool isDevInfoCompatible(const dev_info_t& a, const dev_info_t& b);
    bool compareFirmwareVersions(const dev_info_t& a, const dev_info_t& b);
};


#endif //INERTIALSENSE_SDK__UTIL_H
