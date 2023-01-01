/*
MIT LICENSE

Copyright 2014-2019 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT, IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include "ISBoards.h"
#include "d_time.h"

inline uint32_t time_ticks_to_usec(uint32_t ticks)
{
#if TIME_TICKS_PER_US == 0	// Granularity isn't better than 1us (SAMx70)
	return (ticks / TIME_TICKS_PER_MS) * 1000;
#else
	return (ticks / TIME_TICKS_PER_US);
#endif
}

inline uint32_t time_usec_to_ticks(uint32_t usec)
{
	return (usec * TIME_TICKS_PER_US);
}

inline float time_ticksf_to_usecf(float ticks)
{
	return (ticks * TIME_US_PER_TICK_F);
}

inline float time_usecf_to_ticksf(float usec)
{
	return (usec / TIME_US_PER_TICK_F);
}

void time_delay(uint32_t ms)
{
	if (ms != 0)
	{
		uint32_t start = time_ticks_u32();	// Rollover is automatically handled even without 64 bit rollover value
		uint32_t ticktarget = ms * TIME_TICKS_PER_MS;
		while (time_ticks_u32() - start < ticktarget);
	}
}

inline uint32_t time_msec(void)
{
	return (uint32_t)(time_ticks_u64() / TIME_TICKS_PER_MS);			
}

inline uint32_t time_usec(void)
{
#if TIME_TICKS_PER_US == 0	// Granularity isn't better than 1us (SAMx70)
	return (uint32_t)(time_ticks_u64() / TIME_TICKS_PER_MS) * 1000;
#else
	return (uint32_t)(time_ticks_u64() / TIME_TICKS_PER_US);			
#endif
}

inline float time_secf(void)
{
	uint64_t ticks = time_ticks_u64();
	return TIME_SECS_PER_TICK_F * (float)ticks;
}

inline float time_msecf(void)
{
	uint64_t ticks = time_ticks_u64();
	return TIME_MS_PER_TICK_F * (float)ticks;
}

inline float time_usecf(void)
{
	uint64_t ticks = time_ticks_u64();
	return TIME_US_PER_TICK_F * (float)ticks;
}

#if !defined(BOOTLOADER) || !defined(IMX_5)	// Don't use doubles in IMX-5 bootloader

double time_seclf(void)
{
	uint64_t ticks = time_ticks_u64();
	return TIME_SECS_PER_TICK_LF * (double)ticks;
}

inline double time_mseclf(void)
{
	uint64_t ticks = time_ticks_u64();
	return TIME_MS_PER_TICK_LF * (double)ticks;
}

inline double time_useclf(void)
{
	uint64_t ticks = time_ticks_u64();
	return TIME_US_PER_TICK_LF * (double)ticks;
}

#endif
