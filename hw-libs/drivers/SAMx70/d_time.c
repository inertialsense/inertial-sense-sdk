/*
MIT LICENSE

Copyright 2014-2019 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT, IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include "rtt.h"
#include "d_time.h"
#include "conf_interrupts.h"

#if USE_TIMER_DRIVER
#include "d_timer.h"
#endif

#if USE_FREERTOS
#include "rtos.h"
#endif

static union {
	struct {
		uint32_t tim;
		uint32_t rollcnt;
	};
	uint64_t u64;
} g_ticks = {0};

void RTT_Handler(void)
{
	// alarm
	if (RTT->RTT_SR & RTT_SR_ALMS)
	{
#if USE_FREERTOS
		rtosResetTaskCounters();
#endif
		g_ticks.rollcnt++;
	}
}

void time_init(void)
{
	// Only allow init once every reset
	static bool inited = false;	
	if (inited) return;
	inited = true;

#if USE_TIMER_DRIVER
	timer_time_init();
#endif

	// Increment as frequently as possible
	rtt_sel_source(RTT, false);

	rtt_init(RTT, TIM_PRESC);
	
	// Interrupt on 32bit rollover
	rtt_write_alarm_time(RTT, 0);
	rtt_enable_interrupt(RTT, RTT_MR_ALMIEN);
	
	NVIC_DisableIRQ(RTT_IRQn);
	NVIC_ClearPendingIRQ(RTT_IRQn);
	NVIC_SetPriority(RTT_IRQn, INT_PRIORITY_RTT);
	NVIC_EnableIRQ(RTT_IRQn);
}

inline volatile uint64_t time_ticks_u64(void)
{
	// Time must be read until two consecutive reads match so we don't get a partially updated value
	do {
		g_ticks.tim = RTT->RTT_VR;
	} while (g_ticks.tim != RTT->RTT_VR);
	
	return g_ticks.u64;	// Return uint64_t value; g_ticks.rollcnt is updated in interrupt
}

inline volatile uint32_t time_ticks_u32(void)
{
	// Time must be read until two consecutive reads match so we don't get a partially updated value
	do {
		g_ticks.tim = RTT->RTT_VR;
	} while (g_ticks.tim != RTT->RTT_VR);
	
	return g_ticks.tim;
}

inline uint32_t time_ticks_to_usec(uint32_t ticks)
{
	return ((ticks / TIME_TICKS_PER_MS) * 1000U);	// Tick rate is slower on this processor than 1us
}

inline uint32_t time_usec_to_ticks(uint32_t usec)
{
	return ((usec * TIME_TICKS_PER_MS) / 1000U);	// Tick rate is slower on this processor than 1us
}

inline float time_ticksf_to_usecf(float ticks)
{
	return (ticks * TIME_US_PER_TICK_F);
}

inline float time_usecf_to_ticksf(float usec)
{
	return (usec / TIME_US_PER_TICK_F);
}

void time_delay_msec(uint32_t ms)
{
	if (ms != 0)
	{
		uint32_t start = time_ticks_u32();	// Rollover is automatically handled even without 64 bit rollover value
		uint32_t ticktarget = ms * TIME_TICKS_PER_MS;
		while (time_ticks_u32() - start < ticktarget);
	}
}

void time_delay_usec(uint32_t us)
{
	if (us != 0)
	{
		uint32_t start = time_ticks_u32();	// Rollover is automatically handled even without 64 bit rollover value
		uint32_t ticktarget = us * TIME_TICKS_PER_MS / 1000U;	// Tick rate is slower on this processor than 1us
		while (time_ticks_u32() - start < ticktarget);
	}
}

inline uint32_t time_msec(void)
{	
	return (uint32_t)((uint64_t)(time_ticks_u64() * TIME_MS_PER_TICK_LF) & 0x00000000FFFFFFFF);
}

inline uint32_t time_usec(void)
{
	return (uint32_t)((uint64_t)(time_ticks_u64() * TIME_US_PER_TICK_LF) & 0x00000000FFFFFFFF);
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
