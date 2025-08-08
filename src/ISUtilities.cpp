/*
MIT LICENSE

Copyright (c) 2014-2025 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include <stdio.h>
#include <sstream>
#include <iostream>
#include <math.h>
#include <string>

#include "ISUtilities.h"
#include "ISPose.h"
#include "ISEarth.h"

#if CPP11_IS_ENABLED
    #include <thread>
    #include <mutex>
#endif

#if PLATFORM_IS_EMBEDDED
    #include "drivers/d_time.h"
#elif PLATFORM_IS_WINDOWS
    #include <windows.h>
    #include <process.h>
#elif PLATFORM_IS_LINUX
    // Nothing to do
#else
    #error "Unsupported platform"
#endif

using namespace std;

static const string s_base64_chars = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
static inline bool is_base64(unsigned char c)
{
	return (isalnum(c) || (c == '+') || (c == '/'));
}

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
	return	st.wMilliseconds + 1000 * (st.wSecond + 60 * (st.wMinute + 60 * (st.wHour + 24 * st.wDayOfWeek)));
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

const unsigned char* getHexLookupTable()
{
	static const unsigned char s_hexLookupTable[16] = { '0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', 'B', 'C', 'D', 'E', 'F' };
	return s_hexLookupTable;
}

uint8_t getHexValue(unsigned char hex)
{
	return 9 * (hex >> 6) + (hex & 017);
}

void* threadCreateAndStart(void(*function)(void*), void* info)
{
#if PLATFORM_IS_EMBEDDED

    return NULLPTR;

#elif CPP11_IS_ENABLED

	return new thread(function, info);

#elif PLATFORM_IS_WINDOWS

	return CreateThread(NULL, 0, (LPTHREAD_START_ROUTINE)function, info, 0, NULL);

#else

	pthread_t* t = (pthread_t*)MALLOC(sizeof(pthread_t));
	pthread_create(t, NULL, function, info);
	return t;

#endif

}

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

int32_t convertGpsToMjd(int32_t gpsWeek, int32_t gpsSeconds)
{
    uint32_t gpsDays = gpsWeek * 7 + (gpsSeconds / 86400);
    return convertDateToMjd(1980, 1, 6) + gpsDays;
}

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

void convertGpsToHMS(int32_t gpsSeconds, int32_t* hour, int32_t* minutes, int32_t* seconds)
{
    // shave off days
    gpsSeconds = gpsSeconds % 86400;

    // compute hours, minutes, seconds
    *hour = gpsSeconds / 3600;
    *minutes = (gpsSeconds / 60) % 60;
    *seconds = gpsSeconds % 60;
}

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

cMutex::cMutex()
{
	m_handle = mutexCreate();
}

cMutex::~cMutex()
{
	mutexFree(m_handle);
}

void cMutex::Lock()
{
	mutexLock(m_handle);
}

void cMutex::Unlock()
{
	mutexUnlock(m_handle);
}

cMutexLocker::cMutexLocker(cMutex* mutex)
{
	assert(mutex != NULLPTR);
	m_mutex = mutex;
	m_mutex->Lock();
}

cMutexLocker::~cMutexLocker()
{
	m_mutex->Unlock();
}

void advance_cursor(void) 
{
	static unsigned int timeLast = current_timeMs();
	if(current_timeMs() - timeLast < 50U) return; 
	timeLast = current_timeMs();
	static int pos=0;
	char cursor[4]={'/','-','\\','|'};
	printf("%c\b", cursor[pos]);
	fflush(stdout);
	pos = (pos+1) % 4;
}
