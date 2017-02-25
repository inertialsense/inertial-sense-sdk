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

using namespace std;

#if defined(_WIN32)		//   =========  Windows  =========

#include <windows.h>

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

#else					//   ==========  Linux  ==========

#include <unistd.h>
#include <sys/time.h>

#endif

int current_timeSec()
{
#if defined(_WIN32)
	SYSTEMTIME st;
	GetLocalTime( &st );
	return st.wSecond;
#else
	struct timeval  tv;
	gettimeofday( &tv, NULL );
	return tv.tv_sec;
#endif
}

int current_timeMs()
{
#if defined(_WIN32)
	SYSTEMTIME st;
// 	GetLocalTime( &st );
	GetSystemTime( &st );
	return st.wMilliseconds;
#else
	struct timeval  tv;
	gettimeofday( &tv, NULL );
	return tv.tv_usec / 1000;
#endif
}

/*! Time since week start (Sunday morning) in milliseconds, GMT */
int current_weekMs()
{
#if defined(_WIN32)
	SYSTEMTIME st;
// 	GetLocalTime( &st );
	GetSystemTime( &st );
	return	st.wMilliseconds + 1000 * (st.wSecond + 60 * (st.wMinute + 60 * (st.wHour + 24 * st.wDayOfWeek)));
#else
	struct timeval  tv;
	gettimeofday( &tv, NULL );
	return tv.tv_usec / 1000 + 1000 * tv.tv_sec;
#endif
}

uint64_t current_weekUs()
{
#if defined(_WIN32)
	LARGE_INTEGER StartingTime;
	LARGE_INTEGER Frequency;

	QueryPerformanceCounter( &StartingTime );
	QueryPerformanceFrequency( &Frequency );

	StartingTime.QuadPart *= 1000000;
	StartingTime.QuadPart /= Frequency.QuadPart;

	return StartingTime.QuadPart;
#else
	struct timeval  tv;
	gettimeofday( &tv, NULL );
	return tv.tv_usec + 1000000 * tv.tv_sec;
#endif
}

uint64_t timerUsStart()
{
#if defined(_WIN32)
	LARGE_INTEGER StartingTime;

	QueryPerformanceCounter( &StartingTime );

	return StartingTime.QuadPart;
#else
	struct timeval  tv;
	gettimeofday( &tv, NULL );
	return tv.tv_usec + 1000000 * tv.tv_sec;
#endif
}

uint64_t timerUsEnd(uint64_t start)
{
#if defined(_WIN32)
	LARGE_INTEGER EndingTime, ElapsedTimeUs;
	LARGE_INTEGER Frequency;

	QueryPerformanceCounter( &EndingTime );
	QueryPerformanceFrequency( &Frequency );

	ElapsedTimeUs.QuadPart = EndingTime.QuadPart - start;

	ElapsedTimeUs.QuadPart *= 1000000;
	ElapsedTimeUs.QuadPart /= Frequency.QuadPart;

	return ElapsedTimeUs.QuadPart;
#else
	struct timeval  tv;
	gettimeofday( &tv, NULL );
	uint64_t stopTimeUs = tv.tv_usec + 1000000 * tv.tv_sec;
	return stopTimeUs - start;
#endif
}

uint64_t timerRawStart()
{
#if defined(_WIN32)
	LARGE_INTEGER StartingTime;

	QueryPerformanceCounter( &StartingTime );

	return StartingTime.QuadPart;
#else
	struct timeval  tv;
	gettimeofday( &tv, NULL );
	return tv.tv_usec + 1000000 * tv.tv_sec;
#endif
}

uint64_t timerRawEnd(uint64_t start)
{
#if defined(_WIN32)
	LARGE_INTEGER EndingTime, ElapsedTimeUs;

	QueryPerformanceCounter( &EndingTime );

	ElapsedTimeUs.QuadPart = EndingTime.QuadPart - start;
// 	ElapsedTimeUs.QuadPart *= 1000000;
	return ElapsedTimeUs.QuadPart;
#else
	struct timeval  tv;
	gettimeofday( &tv, NULL );
	uint64_t stopTimeUs = tv.tv_usec + 1000000 * tv.tv_sec;
	return stopTimeUs - start;
#endif
}


int bootloadUploadProgress(const void* port, float percent)
{
	// Suppress compiler warnings
	(void)port;

	printf( "\rBootloader upload: %d%%     \r", (int)(percent * 100.0f) );
	if( percent == 1.0f )
	{
		printf( "\r\n" );
	}
	fflush( stdout );	// stdout stream is buffered (in Linux) so output is only seen after a newline '\n' or fflush().  

	return 1; // could return 0 to abort
}

int bootloadVerifyProgress(const void* port, float percent)
{
	// Suppress compiler warnings
	(void)port;

	printf( "\rBootloader verify: %d%%     \r", (int)(percent * 100.0f) );
	if( percent == 1.0f )
	{
		printf( "\r\n" );
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

	return amplitude * sinf( *sig_gen );
}
