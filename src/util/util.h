/**
 * @file util.h 
 * @brief General Utility functions. Most of these functions should be static, as we don't want/need a "Util" instance running around.
 *
 * @author Kyle Mallory on 3/14/24.
 * @copyright Copyright (c) 2024 Inertial Sense, Inc. All rights reserved.
 */

#ifndef IS_GPX_UTIL_H
#define IS_GPX_UTIL_H

#include <string>
#include <memory>
#include <stdexcept>

namespace utils {
    std::string getCurrentTimestamp();

    template<typename ... Args>
    std::string string_format(const std::string& format, Args ... args) {
        int size_s = std::snprintf( nullptr, 0, format.c_str(), args ... ) + 1; // Extra space for '\0'
        if( size_s <= 0 ) {
            throw std::runtime_error( "Error during formatting." );
        }
        auto size = static_cast<size_t>( size_s );
        std::unique_ptr<char[]> buf( new char[ size ] );
        std::snprintf( buf.get(), size, format.c_str(), args ... );
        return std::string( buf.get(), buf.get() + size - 1 ); // We don't want the '\0' inside
    }
};


#endif //IS_GPX_UTIL_H
