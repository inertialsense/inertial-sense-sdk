/**
 * @file msg_logger.h 
 * @brief ${BRIEF_DESC}
 *
 * @author Kyle Mallory on 4/7/25.
 * @copyright Copyright (c) 2025 Inertial Sense, Inc. All rights reserved.
 */

#ifndef IS_CORE__MSG_LOGGER_H
#define IS_CORE__MSG_LOGGER_H

#define DEBUG_LOGGING
#define LOG_LEVEL           LOG_LEVEL_DEBUG
#define ENABLED_FACILITIES  (LOG_FWUPDATE)

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>

#include <stdio.h>
#include <stdarg.h>
#include <string.h>

#ifndef DEBUG_LOGGING
    #define log_debug(...)
    #define log_info(...)
    #define log_warn(...)
    #define log_error(...)
    #define debug_message(...)
#else
    #define LOG_LEVEL_NONE          0       // use this to disable all log messages
    #define LOG_LEVEL_ERROR         1       // errors - that that prevents the application from proceeding as normal
    #define LOG_LEVEL_WARN          2       // warnings - important to now, but we can move on anyway
    #define LOG_LEVEL_INFO          3       // informative for the customer regarding normal operations
    #define LOG_LEVEL_INFO_MORE     4       // additional information for the customer regarding normal operations - might be annoying
    #define LOG_LEVEL_DEBUG         5       // informative for support & basic troubleshooting
    #define LOG_LEVEL_DEBUG_MORE    6       // informative for advanced troubleshooting - maybe annoying
    #define LOG_LEVEL_BOMBASTIC     7       // excessive on all but the most extreme cases - will definitely be annoying


    // Define facility bitmasks. Use powers of 2 for unique bits.
    #define LOG_FACILITY_NONE       0x0000                          // This facility is ALWAYS enabled (but still followed LOG_LEVEL)
    #define LOG_PORT                0x0001
    #define LOG_FWUPDATE            ((LOG_PORT << 1))
    #define LOG_ISDEVICE            ((LOG_FWUPDATE << 1))
    #define LOG_PORT_FACTORY        ((LOG_ISDEVICE << 1))
    #define LOG_PORT_MANAGER        ((LOG_PORT_FACTORY << 1))
    #define LOG_DEVICE_FACTORY      ((LOG_PORT_MANAGER << 1))
    #define LOG_DEVICE_MANAGER      ((LOG_DEVICE_FACTORY << 1))
    #define LOG_CHRONO_STATS        ((LOG_DEVICE_MANAGER << 1))
    #define LOG_FACILITY_MDNS       ((LOG_CHRONO_STATS << 1))
    #define LOG_FACILITY_ALL        0xFFFF

    // --- Compile-time configuration ---
    // Define the desired log level to show messages at or above this level.
    #ifndef LOG_LEVEL
    #define LOG_LEVEL  LOG_LEVEL_WARN // Default to WARNING
    #endif

    // Define the facilities you want to enable.
    // Combine multiple facilities using the bitwise OR operator.
    #ifndef ENABLED_FACILITIES
    #define ENABLED_FACILITIES (LOG_ISDEVICE)
    #endif

    // Macro to check if a facility is enabled
    #define IS_FACILITY_ENABLED(facility) ((ENABLED_FACILITIES & (facility)) == (facility))

    // --- Macros for different log levels and facilities ---
    #define LOG_MSG(facility, facility_name, level_code, level_name, ...) { do { \
        if (IS_FACILITY_ENABLED(facility) && (LOG_LEVEL >= level_code)) { \
            static_log_msg(facility, level_code, level_name, facility_name, __VA_ARGS__); \
        } \
    } while (0); }

    #define log_error(facility, ...)        LOG_MSG(facility, #facility, LOG_LEVEL_ERROR,      "[ERROR]", __VA_ARGS__)
    #define log_warn(facility, ...)         LOG_MSG(facility, #facility, LOG_LEVEL_WARN,       "[WARN]",  __VA_ARGS__)
    #define log_info(facility, ...)         LOG_MSG(facility, #facility, LOG_LEVEL_INFO,       "[INFO]",  __VA_ARGS__)
    #define log_info_more(facility, ...)    LOG_MSG(facility, #facility, LOG_LEVEL_INFO_MORE,  "[INFO]",  __VA_ARGS__)
    #define log_debug(facility, ...)        LOG_MSG(facility, #facility, LOG_LEVEL_DEBUG,      "[DEBUG]", __VA_ARGS__)
    #define log_debug_more(facility, ...)   LOG_MSG(facility, #facility, LOG_LEVEL_DEBUG_MORE, "[DEBUG]", __VA_ARGS__)
    #define log_bombastic(facility, ...)    LOG_MSG(facility, #facility, LOG_LEVEL_BOMBASTIC,  "[CRAZY]", __VA_ARGS__)

    // #define debug_message(facility, ...)    LOG_MSG(facility, "[DEBUG]", 4, __VA_ARGS__)


    // --- Internal static inline functions ---
    static inline void static_log_msg(int facility_code, int log_level, const char *level_name, const char *facility_name, const char *format, ...) {
        static char logMsg[512];
        memset(logMsg, 0, 512);
        int endPos, msgLen = sizeof(logMsg) - 1;

        if (facility_code)
            endPos = snprintf(logMsg, msgLen, "%s %s : ", facility_name, level_name);
        else
            endPos = snprintf(logMsg, msgLen, "%s : ", level_name);

        va_list args;
        va_start(args, format);
        vsnprintf(&logMsg[endPos], (size_t)logMsg - endPos, format, args);
        va_end(args);

#ifdef __ZEPHYR__
        LOG_DBG(log_msg);
#else
        printf("%s\n", logMsg);
        fflush(stdout);
#endif
    }
#endif

#ifdef __cplusplus
} // END extern 'C'
#endif

#endif //IS_CORE__MSG_LOGGER_H
