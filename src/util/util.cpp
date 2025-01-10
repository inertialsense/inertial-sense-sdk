/**
 * @file util.cpp 
 * @brief ${BRIEF_DESC}
 *
 * @author Kyle Mallory on 3/14/24.
 * @copyright Copyright (c) 2025 Inertial Sense, Inc. All rights reserved.
 */

#include "util.h"

#include <chrono>
#include <cstring>

#include <memory>
#include <string>
#include <stdexcept>

#include "ISDataMappings.h"


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
        sprintf(buffer + strlen(buffer), "%03d", (int) millis);

        return std::string(buffer);
    }

/**
 * Formats the specified DID's raw data as a "hexadecimal view". This can be used with any DID that is not
 * otherwise supported.
 * @param raw_data a pointer to the raw DID byte stream
 * @param hdr the DID header
 * @param bytesPerLine the number of hexadecimal bytes to print per line.
 * @return returns a fully formatted string
 */
    std::string did_hexdump(const char *raw_data, const p_data_hdr_t& hdr, int bytesPerLine)
    {
        (void)hdr;
        char buf[2048];
        char* ptrEnd = buf + 2048;
        char* ptr = buf;

        ptr += SNPRINTF(ptr, ptrEnd - ptr, "(%d) %s (RAW):", hdr.id, cISDataMappings::DataName(hdr.id));

#if DISPLAY_DELTA_TIME==1
        static double lastTime[2] = { 0 };
        double dtMs = 1000.0*(wheel.timeOfWeek - lastTime[i]);
        lastTime[i] = wheel.timeOfWeek;
        ptr += SNPRINTF(ptr, ptrEnd - ptr, " %4.1lfms", dtMs);
#else
#endif
        ptr += SNPRINTF(ptr, ptrEnd - ptr, "\n");
        int lines = hdr.size / bytesPerLine;
        for (int j = 0; j < lines; j++) {
            int linelen = (j == lines-1) ? hdr.size % bytesPerLine : bytesPerLine;
            ptr += SNPRINTF(ptr, ptrEnd - ptr, "\t");
            for (int i = 0; i < linelen; i++) {
                ptr += SNPRINTF(ptr, ptrEnd - ptr, "%02x ", (uint8_t)raw_data[(j * bytesPerLine) + i]);
            }
            ptr += SNPRINTF(ptr, ptrEnd - ptr, "\n");
        }

        return std::string(buf);
    }
}