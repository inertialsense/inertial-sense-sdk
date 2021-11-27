/*
MIT LICENSE

Copyright 2014-2019 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT, IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include <asf.h>
#include "d_time.h"
#include "d_timer.h"
#include "misc/rtos.h"

#include "conf_interrupts.h"
static volatile uint32_t g_rollover = 0;
static volatile uint32_t g_timer = 0;

void RTT_Handler(void)
{
	uint32_t status = rtt_get_status(RTT);

	// alarm
	if (status & RTT_SR_ALMS)
	{
		rtosResetTaskCounters();
		g_rollover++;
	}
}

void time_init(void)
{
	static int initialized = 0;	if (initialized) { return; } initialized = 1;

#ifndef __INERTIAL_SENSE_EVB_2__
	timer_time_init();
#endif

	// configure RTT to increment as frequently as possible
	rtt_sel_source(RTT, false);

	rtt_init(RTT, RTPRES);
	
	NVIC_DisableIRQ(RTT_IRQn);
	NVIC_ClearPendingIRQ(RTT_IRQn);
	NVIC_SetPriority(RTT_IRQn, INT_PRIORITY_RTT);
	NVIC_EnableIRQ(RTT_IRQn);
	
	// notify us when we rollover
	rtt_write_alarm_time(RTT, 0);
	
	g_rollover = 0;	// Reset the rollover now that the timer is fully configured.
}

inline volatile uint64_t time_ticks(void)
{
	volatile uint32_t timer;

	// Time must be read TWICE in ASF code and compared for corruptness.
	timer = rtt_read_timer_value(RTT);
	
	
#if 0	// Add an offset to the counter to simulate what happens on wrap.
	static bool reset = true;
	
	if(timer > 0xFFFF && reset)
	{
		reset = false;
		rtosResetTaskCounters();
		g_rollover++;
	}
	
	timer += 0xFFFF0000;
	
	g_debug.i[0] = timer;
#endif
	
	// this assumes little endian
	volatile ticks_t ticks;
	ticks.u32[1] = g_rollover;
	ticks.u32[0] = timer;

	return ticks.u64;
}


void time_delay(uint32_t ms)
{
	if (ms != 0)
	{
		volatile uint64_t start = time_ticks();
		volatile uint64_t ms64 = (uint64_t)ms * TIME_TICKS_PER_MS;
		while (time_ticks() - start < ms64);
	}
}


inline uint32_t time_msec(void)
{	
	return (uint32_t)((uint64_t)((double)time_ticks() * TIME_MS_PER_TICK_LF) & 0x00000000FFFFFFFF);
}


inline uint32_t time_usec(void)
{
	return (uint32_t)((uint64_t)((double)time_ticks() * TIME_US_PER_TICK_LF) & 0x00000000FFFFFFFF);
}

inline float time_secf(void)
{
	uint64_t ticks = time_ticks();
	return TIME_SECS_PER_TICK_F * (float)ticks;
}


inline float time_msecf(void)
{
	uint64_t ticks = time_ticks();
	return TIME_MS_PER_TICK_F * (float)ticks;
}


inline float time_usecf(void)
{
	uint64_t ticks = time_ticks();
	return TIME_US_PER_TICK_F * (float)ticks;
}


double time_seclf(void)
{
	uint64_t ticks = time_ticks();
	return TIME_SECS_PER_TICK_LF * (double)ticks;
}


inline double time_mseclf(void)
{
	uint64_t ticks = time_ticks();
	return TIME_MS_PER_TICK_LF * (double)ticks;
}


inline double time_useclf(void)
{
	uint64_t ticks = time_ticks();
	return TIME_US_PER_TICK_LF * (double)ticks;
}

