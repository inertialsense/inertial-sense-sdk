/*
MIT LICENSE

Copyright 2014 Inertial Sense, LLC - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include <stdio.h>
#include <iostream>
#include <math.h>
#include <string>

#include "ISUtilities.h"
#include "ISConstants.h"

#if C11_IS_ENABLED

#include <thread>
#include <mutex>

#elif PLATFORM_IS_WINDOWS

#include <windows.h>
#include <process.h>

#elif PLATFORM_IS_LINUX

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


#ifdef __cplusplus
extern "C" {
#endif

#if PLATFORM_IS_WINDOWS

	void usleep(__int64 usec)
	{
		HANDLE timer;
		LARGE_INTEGER ft;

		ft.QuadPart = -(10 * usec); // Convert to 100 nanosecond interval, negative value indicates relative time

		timer = CreateWaitableTimer(NULL, TRUE, NULL);
		SetWaitableTimer(timer, &ft, 0, NULL, NULL, 0);
		WaitForSingleObject(timer, INFINITE);
		CloseHandle(timer);
	}

#else

#include <unistd.h>
#include <sys/time.h>

#endif

int current_timeSec()
{

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

int current_timeMs()
{

#if PLATFORM_IS_WINDOWS

	SYSTEMTIME st;
	GetSystemTime(&st);
	return st.wMilliseconds;

#else

	struct timeval  tv;
	gettimeofday(&tv, NULL);
	return tv.tv_usec / 1000;

#endif

}

/*! Time since week start (Sunday morning) in milliseconds, GMT */
int current_weekMs()
{

#if PLATFORM_IS_WINDOWS

	SYSTEMTIME st;
	GetSystemTime(&st);
	return	st.wMilliseconds + 1000 * (st.wSecond + 60 * (st.wMinute + 60 * (st.wHour + 24 * st.wDayOfWeek)));

#else

	struct timeval  tv;
	gettimeofday(&tv, NULL);
	return tv.tv_usec / 1000 + 1000 * tv.tv_sec;

#endif

}

uint64_t current_weekUs()
{

#if PLATFORM_IS_WINDOWS

	LARGE_INTEGER StartingTime;
	LARGE_INTEGER Frequency;

	QueryPerformanceCounter(&StartingTime);
	QueryPerformanceFrequency(&Frequency);

	StartingTime.QuadPart *= 1000000;
	StartingTime.QuadPart /= Frequency.QuadPart;

	return StartingTime.QuadPart;

#else

	struct timeval  tv;
	gettimeofday(&tv, NULL);
	return tv.tv_usec + 1000000 * tv.tv_sec;

#endif

}

uint64_t timerUsStart()
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

int bootloadUploadProgress(const void* port, float percent)
{
	// Suppress compiler warnings
	(void)port;

	printf("\rBootloader upload: %d%%     \r", (int)(percent * 100.0f));
	if (percent == 1.0f)
	{
		printf("\r\n");
	}
	fflush(stdout);	// stdout stream is buffered (in Linux) so output is only seen after a newline '\n' or fflush().  

	return 1; // could return 0 to abort
}

int bootloadVerifyProgress(const void* port, float percent)
{
	// Suppress compiler warnings
	(void)port;

	printf("\rBootloader verify: %d%%     \r", (int)(percent * 100.0f));
	if (percent == 1.0f)
	{
		printf("\r\n");
	}
	fflush(stdout);	// stdout stream is buffered (in Linux) so output is only seen after a newline '\n' or fflush().  

	return 1; // could return 0 to abort
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

const unsigned char* getHexLookupTable()
{
	static const unsigned char s_hexLookupTable[16] = { '0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', 'B', 'C', 'D', 'E', 'F' };
	return s_hexLookupTable;
}

uint8_t getHexValue(unsigned char hex)
{
	return 9 * (hex >> 6) + (hex & 017);
}

void* threadCreateAndStart(void(*function)(void* info), void* info)
{

#if C11_IS_ENABLED

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

#if C11_IS_ENABLED

	((thread*)handle)->join();
	delete (thread*)handle;

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

#if C11_IS_ENABLED

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

#if C11_IS_ENABLED

	((mutex*)handle)->lock();

#elif PLATFORM_IS_WINDOWS

	EnterCriticalSection((CRITICAL_SECTION*)handle);

#else

	pthread_mutex_lock((pthread_mutex_t*)handle);

#endif

}

void mutexUnlock(void* handle)
{

#if C11_IS_ENABLED

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

#if C11_IS_ENABLED

	delete (mutex*)handle;

#elif PLATFORM_IS_WINDOWS

	DeleteCriticalSection((CRITICAL_SECTION*)handle);
	FREE(handle);

#else

	pthread_mutex_destroy((pthread_mutex_t*)handle);
	FREE(handle);

#endif

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

#if defined(ENABLE_IS_PYTHON_WRAPPER)

test_initializer isutilities([](py::module &m) {
	py::module m2 = m.def_submodule("isutilities");

	m2.def("bootloadUploadProgress", bootloadUploadProgress);
	m2.def("bootloadVerifyProgress", bootloadVerifyProgress);

});

#endif
