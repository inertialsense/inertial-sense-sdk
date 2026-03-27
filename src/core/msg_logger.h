/**
 * @file msg_logger.h
 * @brief ${BRIEF_DESC}
 *
 * @author Kyle Mallory on 4/7/25.
 * @copyright Copyright (c) 2025 Inertial Sense, Inc. All rights reserved.
 */

#ifndef IS_CORE__MSG_LOGGER_H
#define IS_CORE__MSG_LOGGER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <time.h>

#include "ISConstants.h"
#include "types.h"


// --- Compile-time configuration ---
// Define the default log level to show messages at or above this level.
#ifndef IS_LOG_LEVEL
#define IS_LOG_LEVEL  IS_LOG_LEVEL_INFO // Default to INFO
#endif

// Define the highest (most verbose) level that will be compiled into the binary
// IE, if this is set to IS_LOG_LEVEL_WARN, you will never see IS_LOG_LEVEL_INFO
// without changing this and recompiling the SDK/binaries.  This is useful to
// reduce executable size, if necessary, but generally should be left alone.
#ifndef IS_LOG_LEVEL_COMPILER
#define IS_LOG_LEVEL_COMPILER  IS_LOG_LEVEL_MORE_DEBUG // Default to MORE_DEBUG
#endif

// Define the facilities you want to enable.
// Combine multiple facilities using the bitwise OR operator.
#ifndef IS_ENABLED_FACILITIES
#define IS_ENABLED_FACILITIES (IS_LOG_PORT | IS_LOG_PORT_FACTORY | IS_LOG_PORT_MANAGER | IS_LOG_DEVICE_FACTORY | IS_LOG_DEVICE_MANAGER | IS_LOG_ISDEVICE | IS_LOG_FN_PROFILER | IS_LOG_FWUPDATE)
#endif

// Macro to check if a facility is enabled
#define IS_FACILITY_ENABLED(facility) ((IS_ENABLED_FACILITIES & (facility)) == (facility))

// --- Macros for different log levels and facilities ---
#define IS_LOG_MSG(facility, facility_name, level_code, ...) { do { \
    if (IS_FACILITY_ENABLED(facility) && (IS_LOG_LEVEL_COMPILER >= level_code)) {        \
        static_log_msg(facility, level_code, facility_name, __VA_ARGS__); \
    } \
} while (0); }

#define log_msg(facility, level, ...)       IS_LOG_MSG(facility, #facility, level, __VA_ARGS__)
#define log_error(facility, ...)            IS_LOG_MSG(facility, #facility, IS_LOG_LEVEL_ERROR, __VA_ARGS__)
#define log_warn(facility, ...)             IS_LOG_MSG(facility, #facility, IS_LOG_LEVEL_WARN, __VA_ARGS__)
#define log_info(facility, ...)             IS_LOG_MSG(facility, #facility, IS_LOG_LEVEL_INFO, __VA_ARGS__)
#define log_more_info(facility, ...)        IS_LOG_MSG(facility, #facility, IS_LOG_LEVEL_MORE_INFO, __VA_ARGS__)
#define log_debug(facility, ...)            IS_LOG_MSG(facility, #facility, IS_LOG_LEVEL_DEBUG, __VA_ARGS__)
#define log_more_debug(facility, ...)       IS_LOG_MSG(facility, #facility, IS_LOG_LEVEL_MORE_DEBUG, __VA_ARGS__)
#define log_bombastic(facility, ...)        IS_LOG_MSG(facility, #facility, IS_LOG_LEVEL_BOMBASTIC, __VA_ARGS__)

#define IS_SET_LOG_LEVEL(new_level)         static_set_log_level(new_level)
#define IS_GET_LOG_LEVEL()                  static_get_log_level()

// --- Internal static inline functions ---
extern eLogLevel log_level;
static inline eLogLevel static_get_log_level() { return log_level; }
static inline void static_set_log_level(eLogLevel new_level) { log_level = new_level; }

#if defined(PLATFORM_IS_WINDOWS) || defined(PLATFORM_IS_LINUX)
    extern FILE* log_file;
    #define IS_LOG_OUTPUT(out)                  static_log_output(out)
    static inline void static_log_output(FILE* out) { log_file = out; }
#endif

void static_log_msg(int facility_code, int msg_log_level, const char *facility_name, const char *format, ...);
void static_log_buffer(const char* prefix, const unsigned char* buffer, int len);

#define IS_PRINTABLE(n) (((n >= 0x20) && (n <= 0x7E)) ) //  || ((n >= 0xA1) && (n <= 0xDF)))

#ifdef __cplusplus
} // END extern 'C'
#endif

#endif //IS_CORE__MSG_LOGGER_H
