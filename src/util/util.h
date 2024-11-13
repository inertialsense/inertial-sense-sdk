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
#include <memory>
#include <stdexcept>

#include "ISComm.h"

namespace utils {
    std::string getCurrentTimestamp();
    std::string raw_hexdump(const char* raw_data, int bytesLen, int bytesPerLine);
    std::string did_hexdump(const char *raw_data, const p_data_hdr_t& hdr, int bytesPerLine);

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

    std::string devInfoToString(const dev_info_t& devInfo);
    bool devInfoFromString(const std::string& str, dev_info_t& devInfo);
    int parseStringVersion(const std::string& vIn, uint8_t vOut[4]);
    uint64_t intDateTimeFromDevInfo(const dev_info_t& a, bool useMillis = false);
    bool fillDevInfoFromFirmwareImage(std::string imgFilename, dev_info_t& devInfo);
    bool isDevInfoCompatible(const dev_info_t& a, const dev_info_t& b);
    bool compareFirmwareVersions(const dev_info_t& a, const dev_info_t& b);
};


#endif //INERTIALSENSE_SDK__UTIL_H
