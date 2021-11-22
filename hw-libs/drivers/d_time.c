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

#define USE_TC_FOR_FAST_DEBUG	// Define to user TC instead of RTT so faults in RTOS happen faster.

#define TC0_CHANNEL0_ID		23
#define TC0_CHANNEL1_ID		24

void TC_Handler(void)
{
	
}

	#define TC_WAVEFORM_TIMER_SELECTION		TC_CMR_TCCLKS_TIMER_CLOCK2
	#define TC_WAVEFORM_DIVISOR				8
	#define TC_WAVEFORM_FREQUENCY			178
	#define TC_WAVEFORM_DUTY_CYCLE			30
	
static void tc_waveform_initialize(void)
{
	// Configure the PMC to enable the TC module.
	sysclk_enable_peripheral_clock(TC0_CHANNEL0_ID);
	sysclk_enable_peripheral_clock(TC0_CHANNEL1_ID);
		
	tc_init(TC0, 0,
			TC_CMR_TCCLKS_TIMER_CLOCK2 // Waveform Clock Selection (doesn't matter as we are setting NODIVCLK below)
			| TC_CMR_WAVE       // Waveform mode is enabled
			| TC_CMR_ACPA_TOGGLE // RC Compare Effect: clear
			| (0x2 << TC_CMR_WAVSEL_Pos)     // UP mode
	);
	
	tc_init(TC0, 1,
			TC_CMR_TCCLKS_XC1 // Waveform Clock Selection
			| TC_CMR_WAVE       // Waveform mode is enabled
			| TC_CMR_ACPA_SET   // RA Compare Effect: set
			| TC_CMR_ACPC_CLEAR // RC Compare Effect: clear
			| (0x2 << TC_CMR_WAVSEL_Pos)     // UP mode
	);
		
	TC0.TC_CHANNEL[0].TC_EMR |= TC_EMR_NODIVCLK;	// Clock the peripheral directly with no divider
	TC0.TC_CHANNEL[1].TC_BMR |= 0x2 << TC_BMR_TC1XC1S_Pos;	// Clock the second channel with the first
	
	tc_write_ra(TC0, 0, 0x7FFF);
	tc_write_rc(TC0, 0, 0xFFFF);
	tc_write_ra(TC0, 1, 0x7FFF);
	tc_write_rc(TC0, 1, 0xFFFF);

	tc_start(TC0, 1);
	tc_start(TC0, 0);
}

void RTT_Handler(void)
{
	uint32_t status = rtt_get_status(RTT);

	// time has changed
	//	if (status & RTT_SR_RTTINC)
	//	{
	//		g_timer = rtt_read_timer_value(RTT);
	//	}

	// alarm
	if (status & RTT_SR_ALMS)
	{
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
#if SAM4N || SAM4S || SAM4E || SAM4C || SAM4CP || SAM4CM || SAMV71 || SAMV70 || SAME70 || SAMS70

	rtt_sel_source(RTT, false);
	
#endif

	rtt_init(RTT, RTPRES);
	NVIC_DisableIRQ(RTT_IRQn);
	NVIC_ClearPendingIRQ(RTT_IRQn);
	NVIC_SetPriority(RTT_IRQn, INT_PRIORITY_RTT);
	NVIC_EnableIRQ(RTT_IRQn);

#ifdef NDEBUG

	// interrupt for each tick - release mode only, makes debugging impossible
//	rtt_enable_interrupt(RTT, RTT_MR_RTTINCIEN);
	
#endif
	
	// notify us when we rollover
	rtt_write_alarm_time(RTT, 0);
	
	g_rollover = 0;	// Reset the rollover now that the timer is fully configured.
}


inline volatile uint64_t time_ticks(void)
{
	volatile uint32_t timer;
	
	// Time must be read TWICE in ASF code and compared for corruptness.
	timer = rtt_read_timer_value(RTT);
	
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

