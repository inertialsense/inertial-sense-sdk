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

#if defined(PLATFORM_IS_WINDOWS)
static CRITICAL_SECTION log_mutex;
static INIT_ONCE log_init_once = INIT_ONCE_STATIC_INIT;
#else
static pthread_mutex_t log_mutex;
static pthread_once_t log_init_once = PTHREAD_ONCE_INIT;
#endif

static void static_log_end() {
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

static inline void static_log_timestamp(FILE* log_file, const char* prefix) {
    struct timespec ts;
    timespec_get(&ts, TIME_UTC);

    struct tm tm_buf;
    #ifdef _WIN32
    localtime_s(&tm_buf, &ts.tv_set);
    #else
    localtime_r(&ts.tv_sec, &tm_buf);
    #endif
    fprintf(log_file, "[%02d:%02d:%02d.%03ld] %s", tm_buf.tm_hour, tm_buf.tm_min, tm_buf.tm_sec, (long)(ts.tv_nsec / 1000), (prefix ? prefix : ""));
}

void static_log_msg(int facility_code, int msg_log_level, const char *facility_name, const char *format, ...) {
    if (msg_log_level > (int)log_level) return;

    lock_mutex();

    if (log_file == NULL) {
        log_file = fopen("inertial_sense.log", "a+");
        if (log_file == NULL) {
            log_file = stdout;
        }
    }

    static_log_timestamp(log_file, ":: ");

    if (facility_code) {
        fprintf(log_file, "%s ", facility_name);
    }

    static const char* log_level_names[] = { "[NONE]", "[ERROR]", "[WARN]", "[INFO]", "[INFO+]", "[DEBUG]", "[DEBUG+]", "[CRAZY]" };
    fprintf(log_file, "%s : ", log_level_names[msg_log_level]);

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
