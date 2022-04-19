/*
MIT LICENSE

Copyright 2014-2019 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT, IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include <asf.h>
#include "d_time.h"
#include "misc/rtos.h"
#include "globals.h"

#ifndef TESTBED
#include "d_timer.h"
#endif

#include "conf_interrupts.h"
static volatile uint32_t g_rollover = 0;
static volatile uint32_t g_timer = 0;

void RTT_Handler(void)
{
	// alarm
	if (RTT->RTT_SR & RTT_SR_ALMS)
	{
		rtosResetTaskCounters();
		g_rollover++;
	}
}

void time_init(void)
{
	static int initialized = 0;	if (initialized) { return; } initialized = 1;

#if !defined(__INERTIAL_SENSE_EVB_2__) && !defined(TESTBED) 
	timer_time_init();
#endif

	// configure RTT to increment as frequently as possible
	rtt_sel_source(RTT, false);

	rtt_init(RTT, RTPRES);
	
	// notify us when we rollover
#if 0	// Bit shift 15 for faster rollover, works once.
	rtt_write_alarm_time(RTT, 0x1FFFF);
#else	// Standard operation
	rtt_write_alarm_time(RTT, 0);
#endif
	
	rtt_enable_interrupt(RTT, RTT_MR_ALMIEN);
	
	NVIC_DisableIRQ(RTT_IRQn);
	NVIC_ClearPendingIRQ(RTT_IRQn);
	NVIC_SetPriority(RTT_IRQn, INT_PRIORITY_RTT);
	NVIC_EnableIRQ(RTT_IRQn);
	
	g_rollover = 0;	// Reset the rollover now that the timer is fully configured.
}

inline volatile uint64_t time_ticks(void)
{
	// Time must be read TWICE in ASF code and compared for corruptness.
	
	volatile uint32_t timer = RTT->RTT_VR;
//	volatile uint64_t timer = RTT->RTT_VR;

	while (timer != RTT->RTT_VR) 
	{
		timer = RTT->RTT_VR;
	}

#if 0	// Bit shift the timer to make overflow happen quicker (testing)
	timer = timer << 15;
#endif
	
	// this assumes little endian
	volatile ticks_t ticks;
	ticks.u32[1] = g_rollover;
	ticks.u32[0] = (uint32_t)timer;
	
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

