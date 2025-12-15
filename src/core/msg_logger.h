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
#include <time.h>

#include "types.h"


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

    #define log_error(facility, ...)        IS_LOG_MSG(facility, #facility, IS_LOG_LEVEL_ERROR,      "[ERROR]", __VA_ARGS__)
    #define log_warn(facility, ...)         IS_LOG_MSG(facility, #facility, IS_LOG_LEVEL_WARN,       "[WARN]",  __VA_ARGS__)
    #define log_info(facility, ...)         IS_LOG_MSG(facility, #facility, IS_LOG_LEVEL_INFO,       "[INFO]",  __VA_ARGS__)
    #define log_more_info(facility, ...)    IS_LOG_MSG(facility, #facility, IS_LOG_LEVEL_MORE_INFO,  "[INFO]",  __VA_ARGS__)
    #define log_debug(facility, ...)        IS_LOG_MSG(facility, #facility, IS_LOG_LEVEL_DEBUG,      "[DEBUG]", __VA_ARGS__)
    #define log_more_debug(facility, ...)   IS_LOG_MSG(facility, #facility, IS_LOG_LEVEL_MORE_DEBUG, "[DEBUG]", __VA_ARGS__)
    #define log_bombastic(facility, ...)    IS_LOG_MSG(facility, #facility, IS_LOG_LEVEL_BOMBASTIC,  "[CRAZY]", __VA_ARGS__)

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
    #elif defined(PLATFORM_IS_WINDOWS) || defined(PLATFORM_IS_LINUX)
        static char logMsg[512];
        va_list args;
        va_start(args, format);
        memset(logMsg, 0, 512);
        vsnprintf(logMsg, sizeof(logMsg) - 1, format, args);
        va_end(args);

        struct timespec ts;
        timespec_get(&ts, TIME_UTC);
        printf("%ld.%06ld: ", ts.tv_sec, ts.tv_nsec / 1000);

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
