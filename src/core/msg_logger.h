/**
 * @file msg_logger.h
 * @brief Facility- and level-gated logging macros (log_error()/log_warn()/log_info()/etc.), the
 *        eLogLevel run-time level control (IS_SET_LOG_LEVEL()/IS_GET_LOG_LEVEL()), and the
 *        static_log_msg()/static_log_buffer() backends they expand to.
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
/** Default run-time log level; messages below this eLogLevel are suppressed unless IS_SET_LOG_LEVEL() raises it. */
#ifndef IS_LOG_LEVEL
#define IS_LOG_LEVEL  IS_LOG_LEVEL_INFO // Default to INFO
#endif

/**
 * The highest (most verbose) eLogLevel that will be compiled into the binary. IE, if this is set
 * to IS_LOG_LEVEL_WARN, you will never see IS_LOG_LEVEL_INFO messages without changing this and
 * recompiling the SDK/binaries. This is useful to reduce executable size, if necessary, but
 * generally should be left alone.
 */
#ifndef IS_LOG_LEVEL_COMPILER
#define IS_LOG_LEVEL_COMPILER  IS_LOG_LEVEL_MORE_DEBUG // Default to MORE_DEBUG
#endif

/**
 * Bitmask of the IS_LOG_ facilities (see types.h) enabled at compile time. Combine multiple
 * facilities using the bitwise OR operator; only enabled facilities pass IS_FACILITY_ENABLED().
 */
#ifndef IS_ENABLED_FACILITIES
#define IS_ENABLED_FACILITIES (IS_LOG_PORT | IS_LOG_PORT_FACTORY | IS_LOG_PORT_MANAGER | IS_LOG_DEVICE_FACTORY | IS_LOG_DEVICE_MANAGER | IS_LOG_ISDEVICE | IS_LOG_FN_PROFILER | IS_LOG_FWUPDATE | IS_LOG_ISLOG | IS_LOG_APP_LOGALYZER)
#endif

/**
 * Checks whether the given facility bit(s) are enabled in IS_ENABLED_FACILITIES.
 * @param facility a facility bitmask (or combination) to test, e.g. IS_LOG_PORT
 * @return non-zero if every bit in facility is enabled, otherwise zero
 */
#define IS_FACILITY_ENABLED(facility) ((IS_ENABLED_FACILITIES & (facility)) == (facility))

// --- Macros for different log levels and facilities ---
/**
 * Core expansion used by all log_*() convenience macros below: expands to a no-op unless
 * facility is enabled and IS_LOG_LEVEL_COMPILER allows level_code, in which case it forwards to
 * static_log_msg().
 * @param facility      bitmask identifying the log facility (see IS_LOG_ constants in types.h)
 * @param facility_name string form of facility, normally supplied via the # stringizing operator
 * @param level_code    the eLogLevel of this message
 * @param ...           printf-style format string and arguments
 */
/* NOTE the shape: a bare do{...}while(0) with NO enclosing braces and NO trailing semicolon, so that one
 * invocation plus the caller's own `;` is exactly ONE statement and a braceless `if (x) log_x(); else ...`
 * still compiles. Do not add enclosing braces. */
#define IS_LOG_MSG(facility, facility_name, level_code, ...) do { \
    if (IS_FACILITY_ENABLED(facility) && (IS_LOG_LEVEL_COMPILER >= level_code)) {        \
        static_log_msg(facility, level_code, facility_name, __VA_ARGS__); \
    } \
} while (0)

/**
 * Logs a message at an arbitrary level.
 * @param facility bitmask identifying the log facility (see IS_LOG_ constants in types.h)
 * @param level    the eLogLevel of this message
 * @param ...      printf-style format string and arguments
 */
#define log_msg(facility, level, ...)       IS_LOG_MSG(facility, #facility, level, __VA_ARGS__)

/**
 * Logs a message at IS_LOG_LEVEL_ERROR.
 * @param facility bitmask identifying the log facility (see IS_LOG_ constants in types.h)
 * @param ...      printf-style format string and arguments
 */
#define log_error(facility, ...)            IS_LOG_MSG(facility, #facility, IS_LOG_LEVEL_ERROR, __VA_ARGS__)

/**
 * Logs a message at IS_LOG_LEVEL_WARN.
 * @param facility bitmask identifying the log facility (see IS_LOG_ constants in types.h)
 * @param ...      printf-style format string and arguments
 */
#define log_warn(facility, ...)             IS_LOG_MSG(facility, #facility, IS_LOG_LEVEL_WARN, __VA_ARGS__)

/**
 * Logs a message at IS_LOG_LEVEL_INFO.
 * @param facility bitmask identifying the log facility (see IS_LOG_ constants in types.h)
 * @param ...      printf-style format string and arguments
 */
#define log_info(facility, ...)             IS_LOG_MSG(facility, #facility, IS_LOG_LEVEL_INFO, __VA_ARGS__)

/**
 * Logs a message at IS_LOG_LEVEL_MORE_INFO.
 * @param facility bitmask identifying the log facility (see IS_LOG_ constants in types.h)
 * @param ...      printf-style format string and arguments
 */
#define log_more_info(facility, ...)        IS_LOG_MSG(facility, #facility, IS_LOG_LEVEL_MORE_INFO, __VA_ARGS__)

/**
 * Logs a message at IS_LOG_LEVEL_DEBUG.
 * @param facility bitmask identifying the log facility (see IS_LOG_ constants in types.h)
 * @param ...      printf-style format string and arguments
 */
#define log_debug(facility, ...)            IS_LOG_MSG(facility, #facility, IS_LOG_LEVEL_DEBUG, __VA_ARGS__)

/**
 * Logs a message at IS_LOG_LEVEL_MORE_DEBUG.
 * @param facility bitmask identifying the log facility (see IS_LOG_ constants in types.h)
 * @param ...      printf-style format string and arguments
 */
#define log_more_debug(facility, ...)       IS_LOG_MSG(facility, #facility, IS_LOG_LEVEL_MORE_DEBUG, __VA_ARGS__)

/**
 * Logs a message at IS_LOG_LEVEL_BOMBASTIC.
 * @param facility bitmask identifying the log facility (see IS_LOG_ constants in types.h)
 * @param ...      printf-style format string and arguments
 */
#define log_bombastic(facility, ...)        IS_LOG_MSG(facility, #facility, IS_LOG_LEVEL_BOMBASTIC, __VA_ARGS__)

/**
 * Sets the current run-time log level.
 * @param new_level the eLogLevel to switch to
 */
#define IS_SET_LOG_LEVEL(new_level)         static_set_log_level(new_level)

/** Returns the current run-time log level. */
#define IS_GET_LOG_LEVEL()                  static_get_log_level()

// --- Internal static inline functions ---
extern eLogLevel log_level;    //!< current run-time log level; messages below this are suppressed

/** @return the current run-time log level. */
static inline eLogLevel static_get_log_level() { return log_level; }

/**
 * Sets the current run-time log level.
 * @param new_level the eLogLevel to switch to
 */
static inline void static_set_log_level(eLogLevel new_level) { log_level = new_level; }

#if defined(PLATFORM_IS_WINDOWS) || defined(PLATFORM_IS_LINUX)
    extern FILE* log_file;    //!< destination stream for log output; set via IS_LOG_OUTPUT()

    /**
     * Redirects log output to the given stream.
     * @param out destination stream that log messages will be written to
     */
    #define IS_LOG_OUTPUT(out)                  static_log_output(out)
    static inline void static_log_output(FILE* out) { log_file = out; }
#endif

/**
 * Backend that formats and emits a single log message; invoked by the log_*() macros above.
 * Prefer the log_*() macros over calling this directly.
 * @param facility_code bitmask identifying the log facility (see IS_LOG_ constants in types.h)
 * @param msg_log_level the eLogLevel of this message
 * @param facility_name string form of facility_code, included in the rendered output
 * @param format        printf-style format string
 * @param ...            format arguments
 */
void static_log_msg(int facility_code, int msg_log_level, const char *facility_name, const char *format, ...);

/**
 * Emits a hex/ASCII dump of buffer, prefixed by prefix, to the log output.
 * @param prefix short label printed before the dump
 * @param buffer the bytes to dump
 * @param len    the number of bytes in buffer
 */
void static_log_buffer(const char* prefix, const unsigned char* buffer, int len);

/**
 * Checks whether n is within the printable ASCII range (0x20..0x7E).
 * @param n the character/byte value to test
 * @return non-zero if n is printable ASCII, otherwise zero
 */
#define IS_PRINTABLE(n) (((n >= 0x20) && (n <= 0x7E)) ) //  || ((n >= 0xA1) && (n <= 0xDF)))

#ifdef __cplusplus
} // END extern 'C'
#endif

#endif //IS_CORE__MSG_LOGGER_H
