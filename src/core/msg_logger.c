#include "msg_logger.h"
#include <stdlib.h>

#if defined(PLATFORM_IS_WINDOWS)
#include <windows.h>
#else
#include <pthread.h>
#endif

eLogLevel log_level = IS_LOG_LEVEL;

#if !defined(PLATFORM_IS_WINDOWS) && !defined(PLATFORM_IS_LINUX)

// For other platforms like Zephyr, we provide stub implementations
/**
 * @brief Stub implementation of the message logger used on platforms (e.g. Zephyr) that do not support file/console logging; discards all arguments and does nothing.
 *
 * @param facility_code - identifier for the subsystem/facility generating the message (unused)
 * @param msg_log_level - severity level of the message (unused)
 * @param facility_name - human-readable name of the facility (unused)
 * @param format - printf-style format string (unused)
 * @param ... - format arguments (unused)
 */
void static_log_msg(int facility_code, int msg_log_level, const char *facility_name, const char *format, ...) {
    (void)facility_code;
    (void)msg_log_level;
    (void)facility_name;
    (void)format;
}

/**
 * @brief Stub implementation of the buffer logger used on platforms (e.g. Zephyr) that do not support file/console logging; discards all arguments and does nothing.
 *
 * @param prefix - text to prepend to the logged buffer dump (unused)
 * @param buffer - bytes to be logged (unused)
 * @param len - number of bytes in buffer (unused)
 */
void static_log_buffer(const char* prefix, const unsigned char* buffer, int len) {
    (void)prefix;
    (void)buffer;
    (void)len;
}

#else // defined(PLATFORM_IS_WINDOWS) || defined(PLATFORM_IS_LINUX)

FILE* log_file = NULL;

#if defined(PLATFORM_IS_WINDOWS)
static CRITICAL_SECTION log_mutex;
static INIT_ONCE log_init_once = INIT_ONCE_STATIC_INIT;
#else
static pthread_mutex_t log_mutex;
static pthread_once_t log_init_once = PTHREAD_ONCE_INIT;
#endif

/**
 * @brief Cleans up logging resources at process exit: destroys the log mutex and closes the log file if it was opened (leaves stdout/stderr untouched).
 * @note Registered via atexit() from the mutex init routines, so it runs automatically on normal program termination.
 */
static void static_log_end(void) {
#if defined(PLATFORM_IS_WINDOWS)
    DeleteCriticalSection(&log_mutex);
#else
    pthread_mutex_destroy(&log_mutex);
#endif
    if (log_file && log_file != stdout && log_file != stderr) {
        fclose(log_file);
        log_file = NULL;
    }
}

#if defined(PLATFORM_IS_WINDOWS)
/**
 * @brief Windows InitOnceExecuteOnce callback that initializes the log critical section and registers static_log_end() to run at exit.
 *
 * @param InitOnce - the one-time initialization structure being executed (unused)
 * @param Parameter - user parameter passed to InitOnceExecuteOnce (unused)
 * @param Context - output context pointer for InitOnceExecuteOnce (unused)
 *
 * @return TRUE always, indicating successful one-time initialization
 */
static BOOL CALLBACK InitMutex(PINIT_ONCE InitOnce, PVOID Parameter, PVOID *Context)
{
    InitializeCriticalSection(&log_mutex);
    atexit(static_log_end);
    return TRUE;
}
#else
/**
 * @brief Non-Windows one-time initializer that constructs the log mutex and registers static_log_end() to run at exit.
 */
static void init_mutex() {
    pthread_mutex_init(&log_mutex, NULL);
    atexit(static_log_end);
}
#endif

/**
 * @brief Guarantees the log mutex and exit handler are initialized exactly once, regardless of how many threads call it.
 */
static void ensure_initialized() {
#if defined(PLATFORM_IS_WINDOWS)
    PVOID lpContext = NULL;
    InitOnceExecuteOnce(&log_init_once, InitMutex, NULL, &lpContext);
#else
    pthread_once(&log_init_once, init_mutex);
#endif
}

/**
 * @brief Ensures the log mutex is initialized and then acquires it, blocking until available.
 */
static void lock_mutex() {
    ensure_initialized();
#if defined(PLATFORM_IS_WINDOWS)
    EnterCriticalSection(&log_mutex);
#else
    pthread_mutex_lock(&log_mutex);
#endif
}

/**
 * @brief Releases the log mutex previously acquired by lock_mutex().
 */
static void unlock_mutex() {
#if defined(PLATFORM_IS_WINDOWS)
    LeaveCriticalSection(&log_mutex);
#else
    pthread_mutex_unlock(&log_mutex);
#endif
}

/**
 * @brief Writes a "[HH:MM:SS.microseconds] prefix" timestamp header to the given log file using local time.
 *
 * @param log_file - open file stream to write the timestamp to
 * @param prefix - optional text appended after the timestamp (may be NULL, treated as empty)
 */
static inline void static_log_timestamp(FILE* log_file, const char* prefix) {
    struct timespec ts;
    timespec_get(&ts, TIME_UTC);

    struct tm tm_buf;
    #ifdef _WIN32
    localtime_s(&tm_buf, &ts.tv_sec);
    #else
    localtime_r(&ts.tv_sec, &tm_buf);
    #endif
    fprintf(log_file, "[%02d:%02d:%02d.%06ld] %s", tm_buf.tm_hour, tm_buf.tm_min, tm_buf.tm_sec, (long)(ts.tv_nsec / 1000), (prefix ? prefix : ""));
}

/**
 * @brief Formats and writes a single log message line (timestamp, level, optional facility name, then the formatted message) to the log file, opening "inertial_sense.log" on first use.
 * @note If msg_log_level exceeds the current global log_level the call is a no-op. If the log file cannot be opened, output falls back to stdout. Access is serialized with the log mutex.
 *
 * @param facility_code - nonzero to include facility_name in the output, zero to omit it
 * @param msg_log_level - severity level of this message; used both to filter against log_level and to select the printed level name
 * @param facility_name - human-readable name of the subsystem generating the message (only used when facility_code is nonzero)
 * @param format - printf-style format string for the message body
 * @param ... - format arguments
 */
void static_log_msg(int facility_code, int msg_log_level, const char *facility_name, const char *format, ...) {
    if (msg_log_level > (int)log_level) return;

    lock_mutex();

    if (log_file == NULL) {
        log_file = fopen("inertial_sense.log", "a+");
        if (log_file == NULL) {
            log_file = stdout;
        }
    }

    static const char* log_level_names[] = { "NONE", "ERROR", "WARN", "INFO", "INFO+", "DEBUG", "DEBUG+", "CRAZY" };
    static_log_timestamp(log_file, NULL);

    if (facility_code)
        fprintf(log_file, "%-6s (%s) :: ", log_level_names[msg_log_level],  facility_name);
    else
        fprintf(log_file, "%-6s :: ", log_level_names[msg_log_level]);

    char logMsg[512];
    va_list args;
    va_start(args, format);
    vsnprintf(logMsg, sizeof(logMsg), format, args);
    va_end(args);

    fprintf(log_file, "%s\n", logMsg);
    fflush(log_file);

    unlock_mutex();
}

/**
 * @brief Writes a timestamped hex-and-ASCII dump of a byte buffer to the log file, formatted in rows of up to 32 bytes with the hex bytes on the left and printable-character representation on the right.
 * @note Non-printable bytes are rendered as the character code 0xB7 in the ASCII column. If the log file cannot be opened, output falls back to stdout. Access is serialized with the log mutex.
 *
 * @param prefix - text prepended to the timestamp header on the first line
 * @param buffer - bytes to be logged
 * @param len - number of bytes in buffer; if <= 0 the function returns without logging anything
 */
void static_log_buffer(const char* prefix, const unsigned char* buffer, int len) {
    const int BYTES_PER_LINE = 32;
    if (len <= 0) return;

    lock_mutex();

    if (log_file == NULL) {
        log_file = fopen("inertial_sense.log", "a+");
        if (log_file == NULL) {
            log_file = stdout;
        }
    }

    static_log_timestamp(log_file, prefix);

    const unsigned char* buff_ofs = buffer;
    int remaining = len;
    do {
        int i;
        for (i = 0; (i < remaining) && (i < BYTES_PER_LINE); i++) {
            fprintf(log_file, " %02x", buff_ofs[i]);
        }

        int pad = ((int)strlen(prefix) + (BYTES_PER_LINE * 3) + 3) - (i * 3);
        fprintf(log_file, "%*c", pad, ' ');

        for (i = 0; (i < remaining) && (i < BYTES_PER_LINE); i++) {
            fprintf(log_file, "%c", IS_PRINTABLE(buff_ofs[i]) ? buff_ofs[i] : 0xB7);
        }

        buff_ofs += i;
        remaining -= i;

        fprintf(log_file, "\n");
        if (remaining > 0) {
            fprintf(log_file, "                      ");
        }
    } while (remaining > 0);

    fflush(log_file);

    unlock_mutex();
}

#endif
