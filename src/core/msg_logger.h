/**
 * @file msg_logger.h 
 * @brief ${BRIEF_DESC}
 *
 * @author Kyle Mallory on 4/7/25.
 * @copyright Copyright (c) 2025 Inertial Sense, Inc. All rights reserved.
 */

#ifndef IS_CORE__MSG_LOGGER_H
#define IS_CORE__MSG_LOGGER_H


//#ifdef __ZEPHYR__
//    #include <zephyr/logging/log.h>
//#endif

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include <stdarg.h>
#include <string.h>

#include "core/types.h"


#define DEBUG_LOGGING
#define IS_LOG_LEVEL           IS_LOG_LEVEL_INFO
#define IS_ENABLED_FACILITIES  (IS_LOG_FWUPDATE)

#ifndef DEBUG_LOGGING
    #define log_debug(...)
    #define log_info(...)
    #define log_warn(...)
    #define log_error(...)
    #define debug_message(...)
#else
    // #define IS_LOG_LEVEL_NONE          0       // use this to disable all log messages
    // #define IS_LOG_LEVEL_ERROR         1       // errors - that that prevents the application from proceeding as normal
    // #define IS_LOG_LEVEL_WARN          2       // warnings - important to now, but we can move on anyway
    // #define IS_LOG_LEVEL_INFO          3       // informative for the customer regarding normal operations
    // #define IS_LOG_LEVEL_INFO_MORE     4       // additional information for the customer regarding normal operations - might be annoying
    // #define IS_LOG_LEVEL_DEBUG         5       // informative for support & basic troubleshooting
    // #define IS_LOG_LEVEL_DEBUG_MORE    6       // informative for advanced troubleshooting - maybe annoying
    // #define IS_LOG_LEVEL_BOMBASTIC     7       // excessive on all but the most extreme cases - will definitely be annoying


    // Define facility bitmasks. Use powers of 2 for unique bits.
    #define IS_LOG_FACILITY_NONE       0x0000                          // This facility is ALWAYS enabled (but still followed LOG_LEVEL)
    #define IS_LOG_PORT                0x0001
    #define IS_LOG_FWUPDATE            ((IS_LOG_PORT << 1))
    #define IS_LOG_ISDEVICE            ((IS_LOG_FWUPDATE << 1))
    #define IS_LOG_PORT_FACTORY        ((IS_LOG_ISDEVICE << 1))
    #define IS_LOG_PORT_MANAGER        ((IS_LOG_PORT_FACTORY << 1))
    #define IS_LOG_DEVICE_FACTORY      ((IS_LOG_PORT_MANAGER << 1))
    #define IS_LOG_DEVICE_MANAGER      ((IS_LOG_DEVICE_FACTORY << 1))
    #define IS_LOG_CHRONO_STATS        ((IS_LOG_DEVICE_MANAGER << 1))
    #define IS_LOG_FACILITY_MDNS       ((IS_LOG_CHRONO_STATS << 1))
    #define IS_LOG_FACILITY_ALL        0xFFFF

    // --- Compile-time configuration ---
    // Define the desired log level to show messages at or above this level.
    #ifndef IS_LOG_LEVEL
    #define IS_LOG_LEVEL  IS_LOG_LEVEL_WARN // Default to WARNING
    #endif

    // Define the facilities you want to enable.
    // Combine multiple facilities using the bitwise OR operator.
    #ifndef IS_ENABLED_FACILITIES
    #define IS_ENABLED_FACILITIES (IS_LOG_ISDEVICE)
    #endif

    // Macro to check if a facility is enabled
    #define IS_FACILITY_ENABLED(facility) ((IS_ENABLED_FACILITIES & (facility)) == (facility))

    // --- Macros for different log levels and facilities ---
    #define IS_LOG_MSG(facility, facility_name, level_code, level_name, ...) { do { \
        if (IS_FACILITY_ENABLED(facility) && (IS_LOG_LEVEL >= level_code)) { \
            static_log_msg(facility, level_code, level_name, facility_name, __VA_ARGS__); \
        } \
    } while (0); }

    #define log_error(facility, ...)        ( IS_LOG_MSG(facility, #facility, IS_LOG_LEVEL_ERROR,      "[ERROR]", __VA_ARGS__) )
    #define log_warn(facility, ...)         ( IS_LOG_MSG(facility, #facility, IS_LOG_LEVEL_WARN,       "[WARN]",  __VA_ARGS__) )
    #define log_info(facility, ...)         ( IS_LOG_MSG(facility, #facility, IS_LOG_LEVEL_INFO,       "[INFO]",  __VA_ARGS__) )
    #define log_info_more(facility, ...)    ( IS_LOG_MSG(facility, #facility, IS_LOG_LEVEL_INFO_MORE,  "[INFO]",  __VA_ARGS__) )
    #define log_debug(facility, ...)        ( IS_LOG_MSG(facility, #facility, IS_LOG_LEVEL_DEBUG,      "[DEBUG]", __VA_ARGS__) )
    #define log_debug_more(facility, ...)   ( IS_LOG_MSG(facility, #facility, IS_LOG_LEVEL_DEBUG_MORE, "[DEBUG]", __VA_ARGS__) )
    #define log_bombastic(facility, ...)    ( IS_LOG_MSG(facility, #facility, IS_LOG_LEVEL_BOMBASTIC,  "[CRAZY]", __VA_ARGS__) )

    // #define debug_message(facility, ...)    IS_LOG_MSG(facility, "[DEBUG]", 4, __VA_ARGS__)


    // --- Internal static inline functions ---
    static inline void static_log_msg(int facility_code, int log_level, const char *level_name, const char *facility_name, const char *format, ...) {
        (void)log_level; // Suppress unused parameter warning
    #if defined(__ZEPHYR__) // defined(PLATFORM_IS_EMBEDDED) ||
        static char logMsg[512];
        va_list args;
        va_start(args, format);
        memset(logMsg, 0, 512);
        vsnprintf(logMsg, sizeof(logMsg) - 1, format, args);
        va_end(args);

        // LOG_DBG(logMsg);
    #else
        static char logMsg[512];
        va_list args;
        va_start(args, format);
        memset(logMsg, 0, 512);
        vsnprintf(logMsg, sizeof(logMsg) - 1, format, args);
        va_end(args);

        if (facility_code)
            printf("%s ", facility_name);
        printf("%s : %s\n", level_name, logMsg);
        fflush(stdout);
    #endif
    }
#endif

#ifdef __cplusplus
} // END extern 'C'
#endif

#endif //IS_CORE__MSG_LOGGER_H
