/**
 * @file util.cpp 
 * @brief ${BRIEF_DESC}
 *
 * @author Kyle Mallory on 3/14/24.
 * @copyright Copyright (c) 2024 Inertial Sense, Inc. All rights reserved.
 */

#include "util.h"

#include <chrono>
#include <cstring>

#include <memory>
#include <string>
#include <stdexcept>


namespace utils {

    std::string getCurrentTimestamp() {
        using std::chrono::system_clock;
        auto currentTime = std::chrono::system_clock::now();
        char buffer[80];

        auto transformed = currentTime.time_since_epoch().count() / 1000000;

        auto millis = transformed % 1000;

        std::time_t tt;
        tt = system_clock::to_time_t(currentTime);
        auto timeinfo = localtime(&tt);
        strftime(buffer, 80, "%F %H:%M:%S", timeinfo);
        sprintf(buffer + strlen(buffer), ".%03d", (int) millis);

        return std::string(buffer);
    }

}