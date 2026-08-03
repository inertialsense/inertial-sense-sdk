/*
MIT LICENSE

Copyright (c) 2014-2025 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

/**
 * @file ISUtilities.h
 * @brief General-purpose SDK utility grab-bag: base64/string helpers, a C++ mutex wrapper,
 *        cross-platform time/timer/uptime functions, a C-ABI thread/mutex abstraction, and
 *        GPS/MJD date-conversion helpers.
 *
 * @author Inertial Sense, Inc.
 * @copyright Copyright (c) 2025 Inertial Sense, Inc. All rights reserved.
 */

#ifndef IS_UTILITIES_H
#define IS_UTILITIES_H

#include <stdlib.h>
#include <inttypes.h>
#include <stdio.h>
#include "data_sets.h"

// C++ API
#ifdef __cplusplus

#include <string>
#include <vector>


/**
* Encode data as base64
* @param bytes_to_encode the data to encode
* @param in_len the number of bytes to encode
* @return the base64 encoded data
*/
std::string base64Encode(const unsigned char* bytes_to_encode, unsigned int in_len);

/**
* Decode base64 data
* @param encoded_string the base64 encoded data
* @return the base64 decoded data - if error, this may be an incomplete set of data
*/
std::string base64Decode(const std::string& encoded_string);

/**
* Split string by delimiter
* @param str the string to split
* @param delimiter the delimiter to split on
* @param result cleared and then filled with split strings
* @return the number of items in result
*/
size_t splitString(const std::string str, const char delimiter, std::vector<std::string>& result);

/**
* Join vector of strings with character
* @param v the vector of strings to join
* @param c the delimiter to insert between joined strings
* @param result cleared and then filled with the joined string
*/
void joinStrings(const std::vector<std::string>& v, const char c, std::string& result);

/**
* Wraps the mutex functions below
*/
class cMutex
{
public:
    /**
    * Constructor - creates the mutex
    */
    cMutex();

    /**
    * Destructor - frees the mutex
    */
    virtual ~cMutex();

    /**
    * Lock the mutex
    */
    void Lock();

    /**
    * Unlock the mutex
    */
    void Unlock();

private:
    void* m_handle;
};

/**
* Locks / unlocks a mutex via constructor and destructor
*/
class cMutexLocker
{
public:
    /**
    * Constructor
    * @param mutex the mutex to lock
    */
    cMutexLocker(cMutex* mutex);

    /**
    * Destructor - unlocks the mutex
    */
    virtual ~cMutexLocker();

private:
    cMutex* m_mutex;
};

#endif

// C API...
#ifdef __cplusplus
extern "C" {
#endif

#if PLATFORM_IS_WINDOWS
    void usleep(__int64 usec);
    #define DEFAULT_COM_PORT "COM4"   //!< default serial port name to try when none is specified (Windows)

    #ifndef SLEEP_MS
        /** @param milliseconds duration to sleep, in milliseconds */
        #define SLEEP_MS(milliseconds) Sleep(milliseconds);
    #endif

    #ifndef SLEEP_US
        /** @param timeUs duration to sleep, in microseconds */
        #define SLEEP_US(timeUs) usleep(timeUs);
    #endif
#else // LINUX
    #include <unistd.h>
    #include <sys/time.h>
    #include <stdarg.h>

    #define DEFAULT_COM_PORT "/dev/ttyUSB0"   //!< default serial port name to try when none is specified (Linux)

    #ifndef SLEEP_MS
        /** @param timeMs duration to sleep, in milliseconds */
        #define SLEEP_MS(timeMs) usleep(timeMs * 1000);
    #endif

    #ifndef SLEEP_US
        /** @param timeUs duration to sleep, in microseconds */
        #define SLEEP_US(timeUs) usleep(timeUs);
    #endif

    // 0 for no output, 1 for verbose, 2 for spinning cursor
    #if PLATFORM_IS_EMBEDDED
        #define LOG_DEBUG_GEN            0   //!< debug-output verbosity: 0 = no output (embedded default)
    #elif !defined(LOG_DEBUG_GEN)
        #define LOG_DEBUG_GEN            2   //!< debug-output verbosity: 0 = no output, 1 = verbose, 2 = spinning cursor (desktop default)
    #else
        // Nothing to do here...
    #endif
#endif

/** @return the current system time, in seconds, as a double (platform-specific epoch: week-start on Windows, Unix epoch elsewhere) */
double current_timeSecD();

/** @return the current system time, in whole seconds (platform-specific epoch) */
unsigned int current_timeSec();

/** System time in milliseconds. @return the current system time, in milliseconds (platform-specific epoch) */
unsigned int current_timeMs();

/** System time in microseconds. @return the current system time, in microseconds (platform-specific epoch) */
uint64_t current_timeUs();

/**
 * Returns the number of milliseconds since the host processor was started (most commonly).
 * This is uses the monotonic clock, which is guaranteed NOT to go back in time (such as if setting the system clock after startup)
 * @return number of milliseconds
 */
uint32_t current_uptimeMs();

/** @return an opaque starting timestamp (microsecond resolution) to be passed to timerUsEnd() */
uint64_t timerUsStart();

/**
 * @param start a timestamp previously returned by timerUsStart()
 * @return the elapsed time, in microseconds, since start
 */
uint64_t timerUsEnd(uint64_t start);

/** @return an opaque starting timestamp (raw, platform-native resolution) to be passed to timerRawEnd() */
uint64_t timerRawStart();

/**
 * @param start a timestamp previously returned by timerRawStart()
 * @return the elapsed time since start, in the same units used by timerRawStart()
 */
uint64_t timerRawEnd(uint64_t start);

/** Prints and advances a simple spinning-cursor character ('/','-','\\','|') to stdout, throttled to update no more than once every 50ms. */
void advance_cursor(void);

/** @return a monotonically increasing tick count, in milliseconds, suitable for measuring elapsed time */
uint64_t getTickCount(void);

/**
 * Advances a sine-wave signal generator by one time step.
 * @param[in,out] sig_gen current phase angle (radians) of the generator; advanced and unwrapped to (-PI, PI] in place
 * @param freqHz the signal frequency, in Hz
 * @param amplitude the desired output amplitude
 * @param periodSec the elapsed time, in seconds, to advance the phase by
 * @return the generated signal value, amplitude * sin(sig_gen)
 */
float step_sinwave(float *sig_gen, float freqHz, float amplitude, float periodSec);

/**
 * Opens a file, using the secure fopen_s() on MSVC and fopen() elsewhere.
 * @param path the path of the file to open
 * @param mode the fopen()-style mode string (e.g. "rb", "wb")
 * @return the opened file handle, or nullptr on failure
 */
FILE* openFile(const char* path, const char* mode);

/** @return the platform's temporary-directory path, ending with a directory separator */
const char* tempPath(); // ends with dir separator

/**
* Return a pointer to 16 elements to map nibbles to for converting to hex
* @return pointer to 16 bytes for mapping nibbles to chars
*/
const unsigned char* getHexLookupTable();

/**
* Get numberic value of a hex code - no bounds check is done, so non-hex chars will return undefined values
* @param hex the hex digit, i.e. 'A'
* @return the numberic value
*/
uint8_t getHexValue(unsigned char hex);

/**
* Create a thread and execute a function
* @param function the function to execute in a background thread
* @param info the parameter to pass to the thread function
* @param threadName an optional name to assign to the thread, for debugging (may be ignored on some platforms)
* @return the thread handle
*/
void* threadCreateAndStart(void(*function)(void*), void* info, const char* threadName);

/**
* Join a thread with this thread, waiting for it to finish, then free the thread
* @param handle the thread handle to join, wait for finish and then to free
*/
void threadJoinAndFree(void* handle);

/**
* Create a mutex which allows exclusive access to a shared resource
* @return the mutex handle
*/
void* mutexCreate(void);

/**
* Lock a mutex - mutex cannot be locked until mutexUnlock is called
* @param handle the mutex handle to lock
*/
void mutexLock(void* handle);

/**
* Unlock a mutex
* @param handle the mutex handle to unlock
*/
void mutexUnlock(void* handle);

/**
* Free a mutex
* @param handle the mutex handle to free
*/
void mutexFree(void* handle);

// taken from http://www.leapsecond.com/tools/gpsdate.c, uses UTC time

/**
 * Converts a calendar date (UTC) to a Modified Julian Date.
 * @param year the calendar year (e.g. 2025)
 * @param month the calendar month (1-12)
 * @param day the calendar day of month (1-31)
 * @return the Modified Julian Date
 */
int32_t convertDateToMjd(int32_t year, int32_t month, int32_t day);

/**
 * Converts a GPS week/seconds-of-week pair to a Modified Julian Date.
 * @param gpsWeek the GPS week number
 * @param gpsSeconds the number of seconds into gpsWeek
 * @return the Modified Julian Date
 */
int32_t convertGpsToMjd(int32_t gpsWeek, int32_t gpsSeconds);

/**
 * Converts a Modified Julian Date to a calendar date (UTC).
 * @param mjd the Modified Julian Date to convert
 * @param[out] year the calendar year
 * @param[out] month the calendar month (1-12)
 * @param[out] day the calendar day of month (1-31)
 */
void convertMjdToDate(int32_t mjd, int32_t* year, int32_t* month, int32_t* day);

/**
 * Converts GPS seconds-of-week to hours/minutes/seconds of the current day.
 * @param gpsSeconds the number of seconds into the current GPS week
 * @param[out] hour the hour of the day (0-23)
 * @param[out] minutes the minute of the hour (0-59)
 * @param[out] seconds the second of the minute (0-59)
 */
void convertGpsToHMS(int32_t gpsSeconds, int32_t* hour, int32_t* minutes, int32_t* seconds);

/**
 * Computes the day of the week for a given calendar date (UTC).
 * @param ul_year the calendar year (e.g. 2025)
 * @param ul_month the calendar month (1-12)
 * @param ul_day the calendar day of month (1-31)
 * @return the day of the week as an index from 1 to 7
 */
uint32_t dateToWeekDay(uint32_t ul_year, uint32_t ul_month, uint32_t ul_day);

#ifdef __cplusplus
} // extern C
#endif

#endif // IS_UTILITIES_H
