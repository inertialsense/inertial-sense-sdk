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

#include "ISConstants.h"
#include "types.h"


// --- Compile-time configuration ---
// Define the default log level to show messages at or above this level.
#ifndef IS_LOG_LEVEL
#define IS_LOG_LEVEL  IS_LOG_LEVEL_INFO // Default to WARNING
#endif

// Define the highest (most verbose) level that will be compiled into the binary
// IE, if this is set to IS_LOG_LEVEL_WARN, you will never see IS_LOG_LEVEL_INFO
// without changing this and recompiling the SDK/binaries.  This is useful to
// reduce executable size, if necessary, but generally should be left alone.
#ifndef IS_LOG_LEVEL_COMPILER
#define IS_LOG_LEVEL_COMPILER  IS_LOG_LEVEL_DEBUG // Default to MORE_DEBUG
#endif

// Define the facilities you want to enable.
// Combine multiple facilities using the bitwise OR operator.
#ifndef IS_ENABLED_FACILITIES
#define IS_ENABLED_FACILITIES (IS_LOG_FACILITY_ALL)
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

static inline void static_log_msg(int facility_code, int msg_log_level, const char *facility_name, const char *format, ...) {
#if defined(__ZEPHYR__) // defined(PLATFORM_IS_EMBEDDED) ||
    (void) msg_log_level;
    static char logMsg[512];
    va_list args;
    va_start(args, format);
    memset(logMsg, 0, 512);
    vsnprintf(logMsg, sizeof(logMsg) - 1, format, args);
    va_end(args);

    // LOG_DBG(logMsg);
#elif defined(PLATFORM_IS_WINDOWS) || defined(PLATFORM_IS_LINUX)
    static const char* log_level_names[] = { "[NONE]", "[ERROR]", "[WARN]", "[INFO]", "[INFO+]", "[DEBUG]", "[DEBUG+]", "[CRAZY]" };
    if (msg_log_level > (int)log_level) return;

    static char logMsg[512];
    va_list args;
    va_start(args, format);
    memset(logMsg, 0, 512);
    vsnprintf(logMsg, sizeof(logMsg) - 1, format, args);
    va_end(args);

    if (log_file == NULL)
        log_file = fopen("inertial_sense.log", "a+"); // stdout;
    if (log_file == NULL)
        log_file = stdout;

    struct timespec ts;
    timespec_get(&ts, TIME_UTC);
    fprintf(log_file, "%ld.%06ld: ", (long)ts.tv_sec, (long)ts.tv_nsec / 1000);

    if (facility_code)
        fprintf(log_file, "%s ", facility_name);
    fprintf(log_file, "%s : %s\n", log_level_names[msg_log_level], logMsg);
    fflush(log_file);
#endif
}

#define IS_PRINTABLE(n) (((n >= 0x20) && (n <= 0x7E)) ) //  || ((n >= 0xA1) && (n <= 0xDF)))

static inline void static_log_buffer(const char* prefix, const unsigned char* buffer, int len) {
#if defined(__ZEPHYR__) // defined(PLATFORM_IS_EMBEDDED) ||
#elif defined(PLATFORM_IS_WINDOWS) || defined(PLATFORM_IS_LINUX)
    if (len > 0) {
        const int bytes_per_line = 32;

        if (log_file == NULL)
            log_file = fopen("inertial_sense.log", "a+"); // stdout;
        if (log_file == NULL)
            log_file = stdout;

        struct timespec ts;
        timespec_get(&ts, TIME_UTC);
        fprintf(log_file, "%ld.%06ld: %s", (long)ts.tv_sec, (long)ts.tv_nsec / 1000, prefix);

        const unsigned char* buff_ofs = buffer;
        int remaining = len, i = 0;
        do {
            for (i = 0; (i < remaining) && (i < bytes_per_line); i++)
                fprintf(log_file, " %02x", buff_ofs[i]);

            int pad = ((int)strlen(prefix) + (bytes_per_line * 3) + 3) - (i * 3);    // note that 'i' is carried from the above for-loop
            fprintf(log_file, "%*c", pad, ' ');

            for (i = 0; (i < remaining) && (i < bytes_per_line); i++)
                fprintf(log_file, "%c", IS_PRINTABLE(buff_ofs[i]) ? buff_ofs[i] : 0xB7);

            buff_ofs += i;
            remaining -= i;

            fprintf(log_file, "\n");
            if (remaining > 0)
                fprintf(log_file, "                      ");
        } while (remaining > 0);
        // fprintf(log_file, "\n");
    }
#endif
}

#ifdef __cplusplus
} // END extern 'C'
#endif

#endif //IS_CORE__MSG_LOGGER_H
