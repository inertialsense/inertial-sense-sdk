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

#define TC0_CHANNEL0_ID		23
#define TC0_CHANNEL1_ID		24
	
static void time_init_tc(void)
{
	// Configure the PMC to enable the TC module.
	sysclk_enable_peripheral_clock(TC0_CHANNEL0_ID);
	sysclk_enable_peripheral_clock(TC0_CHANNEL1_ID);
		
	tc_init(TC0, 0,
			TC_CMR_TCCLKS_TIMER_CLOCK2					// Waveform Clock Selection (doesn't matter as we are setting NODIVCLK below)
			| TC_CMR_WAVE								// Waveform mode is enabled
			| TC_CMR_ACPA_CLEAR							// RA Compare Effect
			| TC_CMR_ACPC_SET							// RC Compare Effect
			| TC_CMR_WAVSEL_UP							// UP mode
			| TC_CMR_CPCTRG								// UP mode with automatic trigger on RC Compare
	);
	
	tc_write_ra(TC0, 0, 0x7FFF);
	tc_write_rc(TC0, 0, 0xFFFF);
	
	TC0->TC_CHANNEL[0].TC_EMR |= TC_EMR_NODIVCLK;		// Clock the peripheral directly with no divider
	
	tc_init(TC0, 1,
			TC_CMR_TCCLKS_XC1							// Waveform Clock Selection
			| TC_CMR_WAVE								// Waveform mode is enabled
			| TC_CMR_WAVSEL_UP							// UP mode
	);
		
	TC0->TC_BMR |= TC_BMR_TC1XC1S_TIOA0;				// Clock the second channel with the first
	
	NVIC_DisableIRQ(TC1_IRQn);							// TC0 Channel 1 is on TC1 handler
	NVIC_ClearPendingIRQ(TC1_IRQn);
	NVIC_SetPriority(TC1_IRQn, 0);
	NVIC_EnableIRQ(TC1_IRQn);
	
	tc_enable_interrupt(TC0, 1, TC_IER_COVFS);			// Trigger interrupt on overflow of 32-bit counter

	tc_start(TC0, 1);
	tc_start(TC0, 0);
	
	uint32_t previous_time = tc_read_cv(TC0, 0);		
	while (previous_time == tc_read_cv(TC0, 0));		// Wait for timebase to startup
}

#ifndef USE_TC_FOR_FAST_DEBUG
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

#else

void TC1_Handler(void)		// TC0 Channel 1 is on TC1 handler
{
	uint32_t status = TC0->TC_CHANNEL[1].TC_SR;
	
	if(status & TC_SR_COVFS)
	{
		rtosResetTaskCounters();
		g_rollover++;
	}
}
#endif

void time_init(void)
{
	static int initialized = 0;	if (initialized) { return; } initialized = 1;

#ifndef __INERTIAL_SENSE_EVB_2__
	// timer_time_init();
#endif

#ifndef USE_TC_FOR_FAST_DEBUG

	// configure RTT to increment as frequently as possible
#if SAM4N || SAM4S || SAM4E || SAM4C || SAM4CP || SAM4CM || SAMV71 || SAMV70 || SAME70 || SAMS70

	rtt_sel_source(RTT, false);
	
#endif

	rtt_init(RTT, RTPRES);
	
	NVIC_DisableIRQ(RTT_IRQn);
	NVIC_ClearPendingIRQ(RTT_IRQn);
	NVIC_SetPriority(RTT_IRQn, INT_PRIORITY_RTT);
	NVIC_EnableIRQ(RTT_IRQn);
	
	// notify us when we rollover
	rtt_write_alarm_time(RTT, 0);
	
#else	// USE_TC_FOR_FAST_DEBUG

	time_init_tc();
	
#endif
	
	g_rollover = 0;	// Reset the rollover now that the timer is fully configured.
}

inline volatile uint32_t get_tc_chain_time(void)
{
	return (uint32_t)((TC0->TC_CHANNEL[1].TC_CV & 0xFFFF) << 16) | (uint16_t)(TC0->TC_CHANNEL[0].TC_CV & 0xFFFF);
}

inline volatile uint64_t time_ticks(void)
{
	volatile uint32_t timer;

#ifndef USE_TC_FOR_FAST_DEBUG
	// Time must be read TWICE in ASF code and compared for corruptness.
	timer = rtt_read_timer_value(RTT);
#else
	timer = get_tc_chain_time();
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

