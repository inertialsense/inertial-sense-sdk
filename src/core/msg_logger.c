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
void static_log_msg(int facility_code, int msg_log_level, const char *facility_name, const char *format, ...) {
    (void)facility_code;
    (void)msg_log_level;
    (void)facility_name;
    (void)format;
}

void static_log_buffer(const char* prefix, const unsigned char* buffer, int len) {
    (void)prefix;
    (void)buffer;
    (void)len;
}

#else // defined(PLATFORM_IS_WINDOWS) || defined(PLATFORM_IS_LINUX)

FILE* log_file = NULL;
static int log_file_owned = 0;                      //!< non-zero when log_file was opened by the logger and must be closed by it
static char log_file_path[1024] = IS_LOG_DEFAULT_OUTPUT_PATH;  //!< path of log_file, "STDOUT"/"STDERR", or "" for a caller-supplied stream

#if defined(PLATFORM_IS_WINDOWS)
static CRITICAL_SECTION log_mutex;
static INIT_ONCE log_init_once = INIT_ONCE_STATIC_INIT;
#else
static pthread_mutex_t log_mutex;
static pthread_once_t log_init_once = PTHREAD_ONCE_INIT;
#endif

static void lock_mutex();
static void unlock_mutex();

/**
 * Flushes the log at process exit. The output and mutex are deliberately left open: static destructors that run
 * after this handler still log, and every message is already flushed, so the OS closing the file loses nothing.
 */
static void static_log_end(void) {
    lock_mutex();
    if (log_file)
        fflush(log_file);
    unlock_mutex();
}

#if defined(PLATFORM_IS_WINDOWS)
static BOOL CALLBACK InitMutex(PINIT_ONCE InitOnce, PVOID Parameter, PVOID *Context)
{
    InitializeCriticalSection(&log_mutex);
    atexit(static_log_end);
    return TRUE;
}
#else
static void init_mutex() {
    pthread_mutex_init(&log_mutex, NULL);
    atexit(static_log_end);
}
#endif

static void ensure_initialized() {
#if defined(PLATFORM_IS_WINDOWS)
    PVOID lpContext = NULL;
    InitOnceExecuteOnce(&log_init_once, InitMutex, NULL, &lpContext);
#else
    pthread_once(&log_init_once, init_mutex);
#endif
}

static void lock_mutex() {
    ensure_initialized();
#if defined(PLATFORM_IS_WINDOWS)
    EnterCriticalSection(&log_mutex);
#else
    pthread_mutex_lock(&log_mutex);
#endif
}

static void unlock_mutex() {
#if defined(PLATFORM_IS_WINDOWS)
    LeaveCriticalSection(&log_mutex);
#else
    pthread_mutex_unlock(&log_mutex);
#endif
}

/** Closes log_file if the logger opened it. Caller must hold log_mutex. */
static void close_owned_output_locked(void) {
    if (log_file_owned && log_file && log_file != stdout && log_file != stderr) {
        fclose(log_file);
    }
    log_file_owned = 0;
}

/** Opens the default log file if no output is configured yet, falling back to stdout. Caller must hold log_mutex. */
static void ensure_output_locked(void) {
    if (log_file != NULL)
        return;

    log_file = fopen(IS_LOG_DEFAULT_OUTPUT_PATH, "a+");
    if (log_file != NULL) {
        log_file_owned = 1;
        snprintf(log_file_path, sizeof(log_file_path), "%s", IS_LOG_DEFAULT_OUTPUT_PATH);
    } else {
        log_file = stdout;
        log_file_owned = 0;
        snprintf(log_file_path, sizeof(log_file_path), "STDOUT");
    }
}

/** Case-insensitive ASCII string equality. */
static int str_equals_nocase(const char* a, const char* b) {
    for (; *a && *b; a++, b++) {
        char ca = (*a >= 'a' && *a <= 'z') ? (char)(*a - 32) : *a;
        char cb = (*b >= 'a' && *b <= 'z') ? (char)(*b - 32) : *b;
        if (ca != cb)
            return 0;
    }
    return (*a == *b);
}

void static_log_output(FILE* out) {
    lock_mutex();
    if (out != log_file)
        close_owned_output_locked();
    log_file = out;
    log_file_owned = 0;
    if (out == stdout)          snprintf(log_file_path, sizeof(log_file_path), "STDOUT");
    else if (out == stderr)     snprintf(log_file_path, sizeof(log_file_path), "STDERR");
    else if (out == NULL)       snprintf(log_file_path, sizeof(log_file_path), "%s", IS_LOG_DEFAULT_OUTPUT_PATH);
    else                        log_file_path[0] = '\0';
    unlock_mutex();
}

int static_log_set_output_path(const char* path, int append) {
    if (path == NULL || path[0] == '\0')
        path = IS_LOG_DEFAULT_OUTPUT_PATH;

    FILE* out = NULL;
    int owned = 0;
    if (str_equals_nocase(path, "STDOUT")) {
        out = stdout;
        path = "STDOUT";
    } else if (str_equals_nocase(path, "STDERR")) {
        out = stderr;
        path = "STDERR";
    } else {
        // Open before taking the lock so a slow or failing open never stalls logging threads.
        out = fopen(path, append ? "a+" : "w");
        if (out == NULL)
            return -1;
        owned = 1;
    }

    lock_mutex();
    if (out != log_file)
        close_owned_output_locked();
    log_file = out;
    log_file_owned = owned;
    snprintf(log_file_path, sizeof(log_file_path), "%s", path);
    unlock_mutex();
    return 0;
}

const char* static_log_get_output_path(void) {
    return log_file_path;
}

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

void static_log_msg(int facility_code, int msg_log_level, const char *facility_name, const char *format, ...) {
    if (msg_log_level > (int)log_level) return;

    lock_mutex();

    ensure_output_locked();

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

void static_log_buffer(const char* prefix, const unsigned char* buffer, int len) {
    const int BYTES_PER_LINE = 32;
    if (len <= 0) return;

    lock_mutex();

    ensure_output_locked();

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
