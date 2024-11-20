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
#include <iostream>
#include <vector>
#include <streambuf>
#include <istream>
#include <ostream>
#include <cstring>
#include <algorithm>


#include "ISComm.h"

namespace utils {
    std::string getCurrentTimestamp();
    std::string raw_hexdump(const char* raw_data, int bytesLen, int bytesPerLine);
    std::string did_hexdump(const char *raw_data, const p_data_hdr_t& hdr, int bytesPerLine);

    template<typename ... Args>
    std::string string_format(const std::string& format, Args ... args) {
#pragma GCC diagnostic ignored "-Wformat-security"
        int size_s = std::snprintf( NULL, 0, format.c_str(), args ... ) + 1; // Extra space for '\0'
        if (size_s <= 0) {
            throw std::runtime_error("Error during formatting.");
        }
        auto size = static_cast<size_t>(size_s);
        std::unique_ptr<char[]> buf(new char[ size ]);
#pragma GCC diagnostic ignored "-Wformat-security"
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

class ByteBuffer : public std::streambuf {
public:
    ByteBuffer(std::size_t size) : size_(size) {
        buffer_.resize(size_, 0); // Initialize buffer with zeros
        setg(buffer_.data(), buffer_.data(), buffer_.data() + buffer_.size());
        setp(buffer_.data(), buffer_.data() + buffer_.size());
    }

    void insert(std::size_t pos, const uint8_t* data, std::size_t len) {
        if (pos + len > buffer_.size()) {
            throw std::out_of_range("Insert position out of range");
        }
        std::memcpy(buffer_.data() + pos, data, len);
        initialized_ranges_.emplace_back(pos, pos + len);
        merge_initialized_ranges();
    }

    std::size_t tellg() const {
        return gptr() - eback();
    }

    std::size_t tellp() const {
        return pptr() - pbase();
    }

    std::size_t data_size() const {
        return current_write_pos_;
    }

    void seekg(std::size_t pos) {
        setg(eback(), eback() + pos, egptr());
    }

    bool is_initialized(std::size_t pos, std::size_t len) const {
        auto end_pos = pos + len;
        for (const auto& range : initialized_ranges_) {
            if (pos >= range.first && end_pos <= range.second) {
                return true;
            }
        }
        return false;
    }

private:
    std::vector<char> buffer_;
    std::size_t size_;
    std::size_t current_write_pos_ = 0;
    std::vector<std::pair<std::size_t, std::size_t>> initialized_ranges_;

    void merge_initialized_ranges() {
        if (initialized_ranges_.empty()) return;
        std::sort(initialized_ranges_.begin(), initialized_ranges_.end());
        std::vector<std::pair<std::size_t, std::size_t>> merged;
        merged.push_back(initialized_ranges_[0]);

        for (const auto& range : initialized_ranges_) {
            if (merged.back().second >= range.first) {
                merged.back().second = (merged.back().second > range.second ? merged.back().second : range.second);
            } else {
                merged.push_back(range);
            }
        }
        initialized_ranges_ = std::move(merged);
    }
};

class ByteBufferStream : public std::iostream {
public:
    ByteBufferStream(ByteBuffer& buffer)
            : std::iostream(&buffer), buffer_(buffer) {}

    std::size_t tellg() const {
        return buffer_.tellg();
    }

    std::size_t tellp() const {
        return buffer_.tellp();
    }

    std::size_t data_size() const {
        return buffer_.data_size();
    }

    void seekg(std::size_t pos) {
        buffer_.seekg(pos);
    }

    bool is_initialized(std::size_t pos, std::size_t len) const {
        return buffer_.is_initialized(pos, len);
    }

private:
    ByteBuffer& buffer_;
};


#endif //INERTIALSENSE_SDK__UTIL_H
