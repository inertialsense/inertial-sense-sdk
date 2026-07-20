/*
MIT LICENSE

Copyright (c) 2014-2025 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include <cstdio>
#include <sstream>
#include <iostream>
#include <cmath>
#include <string>
#include <ctime>

#include "ISUtilities.h"
#include "ISPose.h"
#include "ISEarth.h"

#if CPP11_IS_ENABLED
    #include <thread>
    #include <mutex>
#endif

#if PLATFORM_IS_EMBEDDED
    #if __cplusplus >= 201703L && __has_include("drivers/d_time.h")
        #include "drivers/d_time.h"
    #else
        #error "Unable to compile with out a valid millisecond-precision timer implementation."
        #define time_msec()  (( -1 ))
        #define time_ticks_u64() (( UINT64_MAX ))
    #endif
#elif PLATFORM_IS_WINDOWS
    #include <windows.h>
    #include <process.h>
#elif PLATFORM_IS_LINUX
    // Nothing to do
#elif PLATFORM_IS_APPLE
    // Nothing to do
#else
    #error "Unsupported platform"
#endif

using namespace std;

static const string s_base64_chars = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

/**
 * @brief Checks whether a character is a valid Base64 alphabet character (alphanumeric, '+', or '/').
 *
 * @param c - character to test
 *
 * @return true if c is a Base64 alphabet character, false otherwise
 */
static inline bool is_base64(unsigned char c)
{
    return (isalnum(c) || (c == '+') || (c == '/'));
}

/**
 * @brief Encodes a buffer of raw bytes into a Base64-encoded string, padding with '=' as needed.
 *
 * @param bytes_to_encode - pointer to the raw byte buffer to encode
 * @param in_len - number of bytes in bytes_to_encode
 *
 * @return the Base64-encoded string
 */
string base64Encode(const unsigned char* bytes_to_encode, unsigned int in_len)
{
    string ret;
    int i = 0;
    int j = 0;
    unsigned char char_array_3[3];
    unsigned char char_array_4[4];

    while (in_len--)
    {
        char_array_3[i++] = *(bytes_to_encode++);
        if (i == 3)
        {
            char_array_4[0] = (char_array_3[0] & 0xfc) >> 2;
            char_array_4[1] = ((char_array_3[0] & 0x03) << 4) + ((char_array_3[1] & 0xf0) >> 4);
            char_array_4[2] = ((char_array_3[1] & 0x0f) << 2) + ((char_array_3[2] & 0xc0) >> 6);
            char_array_4[3] = char_array_3[2] & 0x3f;
            for (i = 0; (i < 4); i++)
            {
                ret += s_base64_chars[char_array_4[i]];
            }
            i = 0;
        }
    }

    if (i)
    {
        for (j = i; j < 3; j++)
        {
            char_array_3[j] = '\0';
        }

        char_array_4[0] = (char_array_3[0] & 0xfc) >> 2;
        char_array_4[1] = ((char_array_3[0] & 0x03) << 4) + ((char_array_3[1] & 0xf0) >> 4);
        char_array_4[2] = ((char_array_3[1] & 0x0f) << 2) + ((char_array_3[2] & 0xc0) >> 6);
        char_array_4[3] = char_array_3[2] & 0x3f;

        for (j = 0; (j < i + 1); j++)
        {
            ret += s_base64_chars[char_array_4[j]];
        }

        while ((i++ < 3))
        {
            ret += '=';
        }
    }

    return ret;

}

/**
 * @brief Decodes a Base64-encoded string back into its original byte sequence (returned as a string of raw bytes).
 *
 * @param encoded_string - Base64-encoded input string
 *
 * @return the decoded raw bytes, packed into a std::string
 */
string base64Decode(const string& encoded_string)
{
    int in_len = (int)encoded_string.size();
    int i = 0;
    int j = 0;
    int in_ = 0;
    unsigned char char_array_4[4], char_array_3[3];
    string ret;

    while (in_len-- && (encoded_string[in_] != '=') && is_base64(encoded_string[in_]))
    {
        char_array_4[i++] = encoded_string[in_];
        in_++;
        if (i == 4)
        {
            for (i = 0; i < 4; i++)
            {
                char_array_4[i] = (unsigned char)s_base64_chars.find(char_array_4[i]);
            }
            char_array_3[0] = (char_array_4[0] << 2) + ((char_array_4[1] & 0x30) >> 4);
            char_array_3[1] = ((char_array_4[1] & 0xf) << 4) + ((char_array_4[2] & 0x3c) >> 2);
            char_array_3[2] = ((char_array_4[2] & 0x3) << 6) + char_array_4[3];
            for (i = 0; (i < 3); i++)
            {
                ret += char_array_3[i];
            }
            i = 0;
        }
    }

    if (i)
    {
        for (j = i; j < 4; j++)
        {
            char_array_4[j] = 0;
        }
        for (j = 0; j < 4; j++)
        {
            char_array_4[j] = (unsigned char)s_base64_chars.find(char_array_4[j]);
        }
        char_array_3[0] = (char_array_4[0] << 2) + ((char_array_4[1] & 0x30) >> 4);
        char_array_3[1] = ((char_array_4[1] & 0xf) << 4) + ((char_array_4[2] & 0x3c) >> 2);
        char_array_3[2] = ((char_array_4[2] & 0x3) << 6) + char_array_4[3];
        for (j = 0; (j < i - 1); j++)
        {
            ret += char_array_3[j];
        }
    }

    return ret;
}

/**
 * @brief Splits a string into substrings on a delimiter character and appends the pieces to a result vector.
 *
 * @param str - string to split
 * @param delimiter - character used to separate fields
 * @param result - [out] vector cleared and filled with the split substrings, in order
 *
 * @return number of substrings placed in result
 */
size_t splitString(const string str, const char delimiter, vector<string>& result)
{
    result.clear();
    istringstream f(str);
    string s;
    while (getline(f, s, delimiter))
    {
        result.push_back(s);
    }
    return result.size();
}

/**
 * @brief Joins a vector of strings into a single string, inserting a separator character between elements.
 *
 * @param v - strings to join, in order
 * @param c - separator character inserted between consecutive elements
 * @param result - [out] cleared and set to the joined string
 */
void joinStrings(const vector<string>& v, const char c, string& result)
{
    result.clear();

    for (vector<string>::const_iterator p = v.begin(); p != v.end(); ++p) 
    {
        result += *p;
        if (p != v.end() - 1)
        {
            result += c;
        }
    }
}

#ifdef __cplusplus
extern "C" {
#endif

#if PLATFORM_IS_WINDOWS

    /**
     * @brief Windows implementation of POSIX usleep(): suspends the calling thread for the given number of microseconds using a waitable timer.
     *
     * @param usec - number of microseconds to sleep
     */
    void usleep(__int64 usec)
    {
        HANDLE timer;
        LARGE_INTEGER ft;

        ft.QuadPart = -(10 * usec); // Convert to 100 nanosecond interval, negative value indicates relative time

        timer = CreateWaitableTimer(NULL, true, NULL);
        SetWaitableTimer(timer, &ft, 0, NULL, NULL, 0);
        WaitForSingleObject(timer, INFINITE);
        CloseHandle(timer);
    }

#else

#include <unistd.h>
#include <sys/time.h>

#endif

/** System time in seconds */
double current_timeSecD() {
#if PLATFORM_IS_WINDOWS
    // Time since week start (Sunday morning) in seconds, GMT
    LARGE_INTEGER StartingTime;
    LARGE_INTEGER Frequency;

    QueryPerformanceCounter(&StartingTime);
    QueryPerformanceFrequency(&Frequency);

    return static_cast<double>(StartingTime.QuadPart) / Frequency.QuadPart;
#else
    // Time since epoch, January 1, 1970 (midnight UTC/GMT) in seconds
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return tv.tv_sec + tv.tv_usec / 1000000.0;
#endif
}

/**
 * @brief Returns the current system time, in seconds.
 * @note Platform behavior differs: on Windows this returns just the local wall-clock seconds field (0-59); on other platforms it returns whole seconds since the Unix epoch.
 *
 * @return current time in seconds (interpretation is platform-dependent, see note)
 */
unsigned int current_timeSec() {
#if PLATFORM_IS_WINDOWS
    SYSTEMTIME st;
    GetLocalTime(&st);
    return st.wSecond;
#else
    struct timeval  tv;
    gettimeofday(&tv, NULL);
    return tv.tv_sec;
#endif
}

/** System time in milliseconds */
unsigned int current_timeMs() {
#if PLATFORM_IS_WINDOWS
    // Time since week start (Sunday morning) in milliseconds, GMT
    SYSTEMTIME st;
    GetSystemTime(&st);
    return    st.wMilliseconds + 1000 * (st.wSecond + 60 * (st.wMinute + 60 * (st.wHour + 24 * st.wDayOfWeek)));
#elif PLATFORM_IS_EMBEDDED
    return time_msec();
#else
    // Time since epoch, January 1, 1970 (midnight UTC/GMT)
    struct timeval  tv;
    gettimeofday(&tv, NULL);
    return tv.tv_usec / 1000 + 1000 * tv.tv_sec;
#endif
}

/** System time in milliseconds */
uint64_t current_timeUs() {
#if PLATFORM_IS_WINDOWS
    // Time since week start (Sunday morning) in milliseconds, GMT
    LARGE_INTEGER StartingTime;
    LARGE_INTEGER Frequency;

    QueryPerformanceCounter(&StartingTime);
    QueryPerformanceFrequency(&Frequency);

    StartingTime.QuadPart *= 1000000;
    StartingTime.QuadPart /= Frequency.QuadPart;

    return StartingTime.QuadPart;
#else
    // Time since epoch, January 1, 1970 (midnight UTC/GMT)
    struct timeval  tv;
    gettimeofday(&tv, NULL);
    return tv.tv_usec + 1000000 * tv.tv_sec;
#endif
}

/**
 * Returns the number of milliseconds since the host processor was started (most commonly).
 * This is uses the monotonic clock, which is guaranteed NOT to go back in time (such as if setting the system clock after startup)
 * @return number of milliseconds
 */
uint32_t current_uptimeMs() {
    uint32_t upTimeMs = UINT32_MAX;
#if (defined(IS_IMX) || defined(GPX_1))
    struct timeval tv;
    gettimeofday(&tv, NULL);
    upTimeMs = tv.tv_usec / 1000 + 1000 * tv.tv_sec;
#elif defined(PLATFORM_IS_WINDOWS)
    #define MS_PER_SEC      1000ULL     // MS = milliseconds
    #define US_PER_MS       1000ULL     // US = microseconds
    #define HNS_PER_US      10ULL       // HNS = hundred-nanoseconds (e.g., 1 hns = 100 ns)
    #define NS_PER_US       1000ULL

    #define HNS_PER_SEC     (MS_PER_SEC * US_PER_MS * HNS_PER_US)
    #define NS_PER_HNS      (100ULL)    // NS = nanoseconds
    #define NS_PER_SEC      (MS_PER_SEC * US_PER_MS * NS_PER_US)

    long tv_sec; long tv_nsec;
    static LARGE_INTEGER ticksPerSec;
    LARGE_INTEGER ticks;

    if (!ticksPerSec.QuadPart) {
        QueryPerformanceFrequency(&ticksPerSec);
        if (!ticksPerSec.QuadPart) {
            errno = ENOTSUP;
            return -1;
        }
    }

    QueryPerformanceCounter(&ticks);

    tv_sec = (long)(ticks.QuadPart / ticksPerSec.QuadPart);
    tv_nsec = (long)(((ticks.QuadPart % ticksPerSec.QuadPart) * NS_PER_SEC) / ticksPerSec.QuadPart);

    upTimeMs = (uint32_t)(tv_nsec / 1000000 + 1000 * tv_sec);
#else
    // Posix systems?
    struct timespec tv;
    clock_gettime(CLOCK_MONOTONIC, &tv);
    upTimeMs = (uint32_t)(tv.tv_nsec / 1000000 + 1000 * tv.tv_sec);
#endif
    return upTimeMs;
}

/**
 * @brief Captures a timestamp (in microseconds) suitable for pairing with timerUsEnd() to measure elapsed time.
 *
 * @return platform-specific timestamp in microseconds
 */
uint64_t timerUsStart() {
#if PLATFORM_IS_WINDOWS
    LARGE_INTEGER StartingTime;
    QueryPerformanceCounter(&StartingTime);
    return StartingTime.QuadPart;
#else
    // Time since epoch, January 1, 1970 (midnight UTC/GMT)
    struct timeval  tv;
    gettimeofday(&tv, NULL);
    return tv.tv_usec + 1000000 * tv.tv_sec;
#endif
}

/**
 * @brief Computes the elapsed time in microseconds since a timestamp previously captured by timerUsStart().
 *
 * @param start - timestamp returned by a prior call to timerUsStart()
 *
 * @return elapsed time in microseconds
 */
uint64_t timerUsEnd(uint64_t start)
{

#if PLATFORM_IS_WINDOWS

    LARGE_INTEGER EndingTime, ElapsedTimeUs;
    LARGE_INTEGER Frequency;

    QueryPerformanceCounter(&EndingTime);
    QueryPerformanceFrequency(&Frequency);

    ElapsedTimeUs.QuadPart = EndingTime.QuadPart - start;

    ElapsedTimeUs.QuadPart *= 1000000;
    ElapsedTimeUs.QuadPart /= Frequency.QuadPart;

    return ElapsedTimeUs.QuadPart;

#else

    struct timeval  tv;
    gettimeofday(&tv, NULL);
    uint64_t stopTimeUs = tv.tv_usec + 1000000 * tv.tv_sec;
    return stopTimeUs - start;

#endif

}

/**
 * @brief Captures a raw platform timer count suitable for pairing with timerRawEnd() to measure elapsed ticks.
 * @note Unlike timerUsStart(), the returned value is not normalized to microseconds on Windows (it is raw QueryPerformanceCounter ticks); on non-Windows platforms it is microseconds since the epoch.
 *
 * @return platform-specific raw timer value
 */
uint64_t timerRawStart()
{

#if PLATFORM_IS_WINDOWS

    LARGE_INTEGER StartingTime;
    QueryPerformanceCounter(&StartingTime);
    return StartingTime.QuadPart;

#else

    struct timeval  tv;
    gettimeofday(&tv, NULL);
    return tv.tv_usec + 1000000 * tv.tv_sec;

#endif

}

/**
 * @brief Computes the elapsed raw timer value since a timestamp previously captured by timerRawStart().
 *
 * @param start - timestamp returned by a prior call to timerRawStart()
 *
 * @return elapsed value in the same (platform-specific) units returned by timerRawStart()
 */
uint64_t timerRawEnd(uint64_t start)
{

#if PLATFORM_IS_WINDOWS

    LARGE_INTEGER EndingTime, ElapsedTimeUs;
    QueryPerformanceCounter(&EndingTime);
    ElapsedTimeUs.QuadPart = EndingTime.QuadPart - start;
    return ElapsedTimeUs.QuadPart;

#else

    struct timeval  tv;
    gettimeofday(&tv, NULL);
    uint64_t stopTimeUs = tv.tv_usec + 1000000 * tv.tv_sec;
    return stopTimeUs - start;

#endif

}

/**
 * @brief Returns a monotonically increasing system tick count in milliseconds, used for measuring elapsed time.
 *
 * @return tick count in milliseconds, or 0 if the underlying clock call fails (non-Windows, non-embedded platforms)
 */
uint64_t getTickCount(void)
{

#if PLATFORM_IS_WINDOWS
    return GetTickCount64();
#elif PLATFORM_IS_EVB_2
    return time_ticks_u64();
#elif PLATFORM_IS_EMBEDDED
    return time_ticks_u64();
#else
    struct timespec now;
    if (clock_gettime(CLOCK_MONOTONIC, &now))
    {
        return 0;
    }
    return (uint64_t)(now.tv_sec * 1000.0 + now.tv_nsec / 1000000.0);
#endif

}

/**
 * @brief Advances a persistent phase accumulator by one time step and returns the resulting sine wave sample.
 * @note The phase angle is unwrapped (wrapped back into [-pi, pi]) each call to prevent unbounded growth of *sig_gen.
 *
 * @param sig_gen - [in/out] pointer to the persistent phase angle (radians) accumulated across calls
 * @param freqHz - signal frequency in Hz
 * @param amplitude - output amplitude
 * @param periodSec - time step duration in seconds since the last call
 *
 * @return the sine wave sample for this step, amplitude * sin(*sig_gen)
 */
float step_sinwave(float *sig_gen, float freqHz, float amplitude, float periodSec)
{
    *sig_gen += freqHz * periodSec * C_TWOPI_F;

    // Unwrap Angle
    if (*sig_gen > C_PI_F)
    {
        *sig_gen -= C_TWOPI_F;
    }

    return amplitude * sinf(*sig_gen);
}

/**
 * @brief Opens a file, using the secure fopen_s() on MSVC builds and fopen() elsewhere.
 *
 * @param path - path to the file to open
 * @param mode - fopen-style mode string (e.g. "rb", "w")
 *
 * @return handle to the opened file, or NULL/0 on failure
 */
FILE* openFile(const char* path, const char* mode)
{
    FILE* file = 0;

#ifdef _MSC_VER

    fopen_s(&file, path, mode);

#else

    file = fopen(path, mode);

#endif

    return file;

}

/**
 * @brief Returns the platform's temporary directory path.
 * @note On Windows the result is cached in a static buffer after the first call.
 *
 * @return pointer to a null-terminated string containing the temp directory path (including trailing separator)
 */
const char* tempPath()
{

#if PLATFORM_IS_WINDOWS

    static char _tempPath[MAX_PATH];
    if (_tempPath[0] == 0)
    {
        GetTempPathA(MAX_PATH, _tempPath);
    }
    return _tempPath;

#else

    return "/tmp/";

#endif

}

/**
 * @brief Returns a pointer to a static lookup table mapping a 4-bit nibble value (0-15) to its uppercase hex character.
 *
 * @return pointer to a 16-entry static array of hex digit characters ('0'-'9', 'A'-'F')
 */
const unsigned char* getHexLookupTable()
{
    static const unsigned char s_hexLookupTable[16] = { '0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', 'B', 'C', 'D', 'E', 'F' };
    return s_hexLookupTable;
}

/**
 * @brief Converts a single ASCII hex character ('0'-'9', 'A'-'F', or 'a'-'f') to its 4-bit numeric value.
 * @note Uses bit arithmetic on the ASCII code rather than a table lookup; behavior is undefined if hex is not a valid hex digit.
 *
 * @param hex - ASCII character representing a hex digit
 *
 * @return numeric value (0-15) represented by hex
 */
uint8_t getHexValue(unsigned char hex)
{
    return 9 * (hex >> 6) + (hex & 017);
}

/**
 * @brief Creates and starts a new thread running the given function, using the platform's native threading API.
 * @note On embedded platforms this is a no-op that returns NULLPTR (threading unsupported). On Linux with C++11 threads and glibc >= 2.12, threadName is applied to the OS thread via pthread_setname_np().
 *
 * @param function - thread entry point function, called with info as its argument
 * @param info - opaque pointer passed to function when the thread starts
 * @param threadName - optional human-readable name assigned to the thread (Linux/glibc only); may be ignored on other platforms
 *
 * @return opaque handle to the created thread, or NULLPTR on embedded platforms
 */
void* threadCreateAndStart(void(*function)(void*), void* info, const char* threadName)
{
#if PLATFORM_IS_EMBEDDED

    return NULLPTR;

#elif CPP11_IS_ENABLED
    auto new_thread = new thread(function, info);
    #if defined(PLATFORM_IS_LINUX) && (__GLIBC__ > 2 || ((__GLIBC__ == 2) && (__GLIBC_MINOR__ >= 12)))
    if (threadName) {
        pthread_setname_np(new_thread->native_handle(), threadName);
    }
    #endif
    return new_thread;
#elif PLATFORM_IS_WINDOWS
    return CreateThread(NULL, 0, (LPTHREAD_START_ROUTINE)function, info, 0, NULL);
#elif PPTHREAD_ONCE_INIT
    pthread_t* t = (pthread_t*)MALLOC(sizeof(pthread_t));
    pthread_create(t, NULL, function, info);
    return t;
#endif
}

/**
 * @brief Waits for a thread to finish and releases the resources associated with its handle.
 *
 * @param handle - thread handle previously returned by threadCreateAndStart(); if NULL, the function does nothing
 */
void threadJoinAndFree(void* handle)
{
    if (handle == NULL)
    {
        return;
    }

#if PLATFORM_IS_EMBEDDED

    return;

#elif CPP11_IS_ENABLED

    if (((thread*)handle)->joinable()) 
    {
        ((thread*)handle)->join();
        delete (thread*)handle;
    }

#elif PLATFORM_IS_WINDOWS

    WaitForSingleObject(handle, 0);
    CloseHandle(handle);

#else

    pthread_join((pthread_t*)handle);
    FREE(handle);

#endif

}

/**
 * @brief Allocates and initializes a new mutex using the platform's native synchronization primitive.
 *
 * @return opaque handle to the created mutex, or NULLPTR on embedded platforms (mutexes unsupported)
 */
void* mutexCreate(void)
{

#if PLATFORM_IS_EMBEDDED

    return NULLPTR;

#elif CPP11_IS_ENABLED

    return new mutex();

#elif PLATFORM_IS_WINDOWS

    CRITICAL_SECTION* c = (CRITICAL_SECTION*)MALLOC(sizeof(CRITICAL_SECTION));
    InitializeCriticalSection(c);
    return c;    

#else

    pthread_mutex_t* m = (pthread_mutex_t*)MALLOC(sizeof(pthread_mutex_t));
    pthread_mutex_init(m, NULL);
    return m;

#endif

}

/**
 * @brief Locks (acquires) a mutex previously created by mutexCreate(), blocking until it is available.
 *
 * @param handle - mutex handle returned by mutexCreate()
 */
void mutexLock(void* handle)
{

#if PLATFORM_IS_EMBEDDED

    return;

#elif CPP11_IS_ENABLED

    ((mutex*)handle)->lock();

#elif PLATFORM_IS_WINDOWS

    EnterCriticalSection((CRITICAL_SECTION*)handle);

#else

    pthread_mutex_lock((pthread_mutex_t*)handle);

#endif

}

/**
 * @brief Unlocks (releases) a mutex previously locked via mutexLock().
 *
 * @param handle - mutex handle returned by mutexCreate()
 */
void mutexUnlock(void* handle)
{

#if PLATFORM_IS_EMBEDDED

    return;

#elif CPP11_IS_ENABLED

    ((mutex*)handle)->unlock();

#elif PLATFORM_IS_WINDOWS

    LeaveCriticalSection((CRITICAL_SECTION*)handle);

#else

    pthread_mutex_unlock((pthread_mutex_t*)handle);

#endif

}

/**
 * @brief Destroys a mutex and releases the resources associated with its handle.
 *
 * @param handle - mutex handle previously returned by mutexCreate(); if NULL, the function does nothing
 */
void mutexFree(void* handle)
{
    if (handle == NULL)
    {
        return;
    }

#if PLATFORM_IS_EMBEDDED

    return;

#elif CPP11_IS_ENABLED

    delete (mutex*)handle;

#elif PLATFORM_IS_WINDOWS

    DeleteCriticalSection((CRITICAL_SECTION*)handle);
    FREE(handle);

#else

    pthread_mutex_destroy((pthread_mutex_t*)handle);
    FREE(handle);

#endif

}

/**
 * @brief Converts a calendar date (year/month/day) to a Modified Julian Date (MJD).
 *
 * @param year - calendar year (e.g. 2024)
 * @param month - calendar month (1-12)
 * @param day - day of month (1-31)
 *
 * @return the Modified Julian Date corresponding to the given calendar date
 */
int32_t convertDateToMjd(int32_t year, int32_t month, int32_t day)
{
    return
        367 * year
        - 7 * (year + (month + 9) / 12) / 4
        - 3 * ((year + (month - 9) / 7) / 100 + 1) / 4
        + 275 * month / 9
        + day
        + 1721028
        - 2400000;
}

/**
 * @brief Converts a GPS week number and time-of-week (in seconds) to a Modified Julian Date (MJD).
 *
 * @param gpsWeek - GPS week number (weeks since the GPS epoch, January 6, 1980)
 * @param gpsSeconds - time of week in seconds
 *
 * @return the Modified Julian Date corresponding to the given GPS time
 */
int32_t convertGpsToMjd(int32_t gpsWeek, int32_t gpsSeconds)
{
    uint32_t gpsDays = gpsWeek * 7 + (gpsSeconds / 86400);
    return convertDateToMjd(1980, 1, 6) + gpsDays;
}

/**
 * @brief Converts a Modified Julian Date (MJD) to a calendar year, month, and day.
 *
 * @param mjd - Modified Julian Date to convert
 * @param year - [out] calendar year
 * @param month - [out] calendar month (1-12)
 * @param day - [out] day of month (1-31)
 */
void convertMjdToDate(int32_t mjd, int32_t* year, int32_t* month, int32_t* day)
{
    int32_t j, c, y, m;

    j = mjd + 2400001 + 68569;
    c = 4 * j / 146097;
    j = j - (146097 * c + 3) / 4;
    y = 4000 * (j + 1) / 1461001;
    j = j - 1461 * y / 4 + 31;
    m = 80 * j / 2447;
    *day = j - 2447 * m / 80;
    j = m / 11;
    *month = m + 2 - (12 * j);
    *year = 100 * (c - 49) + y + j;
}

/**
 * @brief Converts a GPS time-of-week value (in seconds) into hours, minutes, and seconds of the current day.
 *
 * @param gpsSeconds - time of week in seconds (values beyond a single day are reduced modulo 86400)
 * @param hour - [out] hour of day (0-23)
 * @param minutes - [out] minute of hour (0-59)
 * @param seconds - [out] second of minute (0-59)
 */
void convertGpsToHMS(int32_t gpsSeconds, int32_t* hour, int32_t* minutes, int32_t* seconds)
{
    // shave off days
    gpsSeconds = gpsSeconds % 86400;

    // compute hours, minutes, seconds
    *hour = gpsSeconds / 3600;
    *minutes = (gpsSeconds / 60) % 60;
    *seconds = gpsSeconds % 60;
}

/**
 * @brief Computes the day of the week for a given calendar date using a Zeller's-congruence-style calculation.
 *
 * @param ul_year - calendar year
 * @param ul_month - calendar month (1-12)
 * @param ul_day - day of month (1-31)
 *
 * @return day of week as a 1-based index (numbering determined by the algorithm's internal constants)
 */
uint32_t dateToWeekDay(uint32_t ul_year, uint32_t ul_month, uint32_t ul_day)
{
    uint32_t ul_week;

    if (ul_month == 1 || ul_month == 2) {
        ul_month += 12;
        --ul_year;
    }

    ul_week = (ul_day + 2 * ul_month + 3 * (ul_month + 1) / 5 + ul_year +
    ul_year / 4 - ul_year / 100 + ul_year / 400) % 7;

    ++ul_week;

    return ul_week;
}

#ifdef __cplusplus
} // extern C
#endif

/**
 * @brief Constructs a cMutex, creating the underlying platform mutex handle.
 */
cMutex::cMutex()
{
    m_handle = mutexCreate();
}

/**
 * @brief Destroys the cMutex, releasing the underlying platform mutex handle.
 */
cMutex::~cMutex()
{
    mutexFree(m_handle);
}

/**
 * @brief Locks the underlying platform mutex, blocking until it is available.
 */
void cMutex::Lock()
{
    mutexLock(m_handle);
}

/**
 * @brief Unlocks the underlying platform mutex.
 */
void cMutex::Unlock()
{
    mutexUnlock(m_handle);
}

/**
 * @brief Constructs a cMutexLocker that immediately locks the given mutex (RAII-style scoped lock).
 *
 * @param mutex - mutex to lock for the lifetime of this cMutexLocker; must not be NULLPTR
 */
cMutexLocker::cMutexLocker(cMutex* mutex)
{
    assert(mutex != NULLPTR);
    m_mutex = mutex;
    m_mutex->Lock();
}

/**
 * @brief Destroys the cMutexLocker, unlocking the mutex that was locked in the constructor.
 */
cMutexLocker::~cMutexLocker()
{
    m_mutex->Unlock();
}

/**
 * @brief Prints a rotating ASCII "spinner" character (/, -, \, |) to stdout to indicate progress, advancing at most once every 50ms.
 * @note Uses static state (last-update time and spinner position) shared across all calls, so it is not reentrant/thread-safe.
 */
void advance_cursor(void)
{
    static unsigned int timeLast = current_timeMs();
    if (current_timeMs() - timeLast < 50U) return; 
    timeLast = current_timeMs();
    static int pos=0;
    char cursor[4]={'/','-','\\','|'};
    printf("%c\b", cursor[pos]);
    fflush(stdout);
    pos = (pos+1) % 4;
}
