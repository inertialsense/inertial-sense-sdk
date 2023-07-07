/*
MIT LICENSE

Copyright 2014-2019 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT, IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/ 

#include <asf.h>
#include "d_quadEnc.h"
#include "conf_d_quadEnc.h"
#include "globals.h"

//Use of index signal is not support at this time. Needs setup for pin and reading values
#define QDEC_USE_INDEX	0

static float s_tc2sec = 0.0f;
static bool s_direction_reverse_0 = 0;
static bool s_direction_reverse_1 = 0;

static void quadEncSetModePosition(Tc *const timercounter, uint32_t ID_timercounter)
{
	//Enable the QDEC channel clocks
	sysclk_enable_peripheral_clock(ID_timercounter);		//Channel 0
#if QDEC_USE_INDEX
	sysclk_enable_peripheral_clock(ID_timercounter + 1);	//Channel 1
#endif
		
	//Init TC channel 0 to QDEC Position mode
	tc_init(timercounter, 0,
		TC_CMR_TCCLKS_XC0
		| TC_CMR_ETRGEDG_RISING     /* To clear the counter on a rising edge of the TIOA signal*/
		| TC_CMR_ABETRG             /* To select TIOA as a trigger for this channel 0 */
	);

#if QDEC_USE_INDEX
	//Init TC channel 1 to QDEC Rotation mode
	tc_init(timercounter, 1,
		/* QDEC Clock Selection */
		TC_CMR_TCCLKS_XC0
	);
#endif

	//Enable TC QDEC channel 0 in QDEC Position mode
	tc_set_block_mode(timercounter, TC_BMR_QDEN /* QDEC mode enabled */
		| TC_BMR_POSEN              /* Position measure is enabled */
		| TC_BMR_EDGPHA             /* Detect quadrature on both PHA and PHB (4X decoding)*/
		| (0<< TC_BMR_MAXFILT_Pos)  /* enable filter on input signal*/
	);	
	
	tc_start(timercounter, 0);	//For position measurement
#if QDEC_USE_INDEX
	tc_start(timercounter, 1);	//For rotation measurement
#endif
}

typedef struct 
{
	uint32_t 		count;
	uint32_t 		overflow;
	uint32_t		directionLast;
	uint32_t		running;
	uint32_t 		timeMs;
	uint32_t		pulseCount;		// Number of pulses accumulated in count
	float			period;
} capture_t;

static capture_t capture[2] = {};

static void quadEncPeriodSetStopped(capture_t *c)
{	// Wheels are stationary or spinning very slowly
	c->running = false;
	c->count = 0;
	c->pulseCount = 0;
	c->period = 0.0f;
}

// This wheel encoder works by 
static inline void tc_encoder_handler(capture_t *c, Tc *p_tc, uint32_t ul_channel, uint32_t direction)
{
	uint32_t status = tc_get_status(p_tc, ul_channel);

	if (c->directionLast != direction)
	{	// Switched direction.  Reset velocity measurements and indicate system is not running.
		c->directionLast = direction;
		quadEncPeriodSetStopped(c);
	}

	if (status & TC_SR_COVFS)
	{	// Counter Overflow
		c->overflow += 0x00010000;
	}
	
	if (status & TC_SR_LDRAS)
	{	// RA Loading
		uint32_t ra_count = tc_read_ra(p_tc, ul_channel);
		if(c->running != 0)
		{
			c->count += c->overflow + ra_count;
			c->pulseCount++;
		}
		else
		{	// First read after stopping is invalid
			c->running = true;
			c->count = 0;
			c->pulseCount = 0;
		}

		c->timeMs = time_msec();
		c->overflow = 0;
	}
}

void TCCAP0_SPD_Handler(void)
{
	tc_encoder_handler(&capture[0], TCCAP0_SPD, TCCAP0_SPD_CHANNEL, QE0_POS->TC_QISR & TC_QISR_DIR);	
}

void TCCAP1_SPD_Handler(void)
{
	tc_encoder_handler(&capture[1], TCCAP1_SPD, TCCAP1_SPD_CHANNEL, QE1_POS->TC_QISR & TC_QISR_DIR);	
}

static void quadEncSetModeSpeed(Tc *const timercounter, int timerchannel, int timerirq, uint32_t ID_timercounter, uint32_t pck6_pres)
{
	//Enable the TC channel clock
	sysclk_enable_peripheral_clock(ID_timercounter);

	if(!pmc_is_pck_enabled(PMC_PCK_6))
	{
		pmc_disable_pck(PMC_PCK_6);
		pmc_switch_pck_to_mainck(PMC_PCK_6, PMC_PCK_PRES(pck6_pres));
		pmc_enable_pck(PMC_PCK_6);
	}

	// Init TC to capture mode.
	tc_init(timercounter, timerchannel,
		TC_CMR_TCCLKS_TIMER_CLOCK1  /* Clock Selection */
		| TC_CMR_LDRA_RISING        /* RA Loading: rising edge of TIOA */
		| TC_CMR_ABETRG             /* External Trigger Source: TIOA */
		| TC_CMR_ETRGEDG_RISING	    /* External Trigger Edge: rising */
	);

	// Setup capture and overflow interrupt		
	NVIC_DisableIRQ(timerirq);
	NVIC_ClearPendingIRQ(timerirq);
	NVIC_SetPriority(timerirq, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY-1);
	NVIC_EnableIRQ(timerirq);
	tc_enable_interrupt(timercounter, timerchannel, TC_IER_LDRAS | TC_IER_COVFS);
	
	tc_start(timercounter, timerchannel);
}

void quadEncInit(uint32_t pck6_pres)
{
	/**** Configure pins ****/
	// Left Encoder - Position
	ioport_set_pin_mode(PIN_QE0_POS_PHA, PIN_QE0_POS_PHA_MUX);
	ioport_disable_pin(PIN_QE0_POS_PHA);
	ioport_set_pin_mode(PIN_QE0_POS_PHB, PIN_QE0_POS_PHB_MUX);
	ioport_disable_pin(PIN_QE0_POS_PHB);

	// Left Encoder - Speed
	ioport_set_pin_mode(PIN_TCCAP0_SPD, PIN_TCCAP0_SPD_MUX);
	ioport_disable_pin(PIN_TCCAP0_SPD);
	
	// Right Encoder - Position
	ioport_set_pin_mode(PIN_QE1_POS_PHA, PIN_QE1_POS_PHA_MUX);
	ioport_disable_pin(PIN_QE1_POS_PHA);
	ioport_set_pin_mode(PIN_QE1_POS_PHB, PIN_QE1_POS_PHB_MUX);
	ioport_disable_pin(PIN_QE1_POS_PHB);

	// Right Encoder - Speed
	ioport_set_pin_mode(PIN_TCCAP1_SPD, PIN_TCCAP1_SPD_MUX);
	ioport_disable_pin(PIN_TCCAP1_SPD);
	
	/**** Setup hardware ****/
	quadEncSetModePosition(QE0_POS, ID_QE0_POS);
	quadEncSetModeSpeed(TCCAP0_SPD, TCCAP0_SPD_CHANNEL, TCCAP0_SPD_IRQn, ID_TCCAP0_SPD, pck6_pres);
	
	quadEncSetModePosition(QE1_POS, ID_QE1_POS);
	quadEncSetModeSpeed(TCCAP1_SPD, TCCAP1_SPD_CHANNEL, TCCAP1_SPD_IRQn, ID_TCCAP1_SPD, pck6_pres);
	
	s_tc2sec = ((float)(pck6_pres+1)) / 12.0e6;
}

void quadEncSetDirectionReverse( bool reverse_0, bool reverse_1 )
{
	s_direction_reverse_0 = reverse_0;
	s_direction_reverse_1 = reverse_1;
}

void quadEncReadPositionAll(int *pos0, bool *dir0, int *pos1, bool *dir1)
{
	int16_t cv;

	taskENTER_CRITICAL();
	
	cv = QE0_POS->TC_CHANNEL[0].TC_CV;
	*pos0 = cv;
	*dir0 = 0x1 && (QE0_POS->TC_QISR & TC_QISR_DIR);
	
	cv = QE1_POS->TC_CHANNEL[0].TC_CV;
	*pos1 = cv;
	*dir1 = 0x1 && (QE1_POS->TC_QISR & TC_QISR_DIR);

	// Reverse direction
	if (s_direction_reverse_0) { *pos0 = -(*pos0); *dir0 = !(*dir0); }
	if (s_direction_reverse_1) { *pos1 = -(*pos1); *dir1 = !(*dir1); }

	taskEXIT_CRITICAL();
}

static float quadEncReadPeriod(capture_t *c)
{
	if (c->running)
	{
		uint32_t dtMs = time_msec() - c->timeMs;
		if (dtMs > 500)		// 100ms timeout
		// We might try scaling this timeout based on the minimum velocity and number of ticks per radian.  We need to wait longer for lower resolution encoders. 
		// threshold = rad_per_tick * min_velocity * (125)     Hoverbot ~500ms, ZT ~2ms
		{	 
			quadEncPeriodSetStopped(c);
		}
	}

	if (c->pulseCount)
	{
		c->period = s_tc2sec * (float)(c->count) / (float)(c->pulseCount);
		c->count = 0;
		c->pulseCount = 0;
	}
// 	else
// 	{
// 		c->period = 0.0f;
// 	}

	return c->period;
}

void quadEncReadPeriodAll(float *period0, float *period1)
{
	// g_debug.i[4] = capture[0].pulseCount;
	// g_debug.i[5] = capture[1].pulseCount;

	taskENTER_CRITICAL();

	*period0 = quadEncReadPeriod(&capture[0]);
	*period1 = quadEncReadPeriod(&capture[1]);

	taskEXIT_CRITICAL();

	// g_debug.i[0] = capture[0].count;
	// g_debug.i[1] = capture[1].count;
	// g_debug.i[2] = capture[0].overflow;
	// g_debug.i[3] = capture[1].overflow;	
	// g_debug.f[0] = *period0 * 1000.0;
	// g_debug.f[1] = *period1 * 1000.0;
}

void test_quad_encoders(void)
{
#if	0	//quadEnc testing

	udi_cdc_write_buf("Running\r\n", 9);
	quadEncInit(239);

	while(1)
	{
		#define BUF_SIZE 100
		char str[BUF_SIZE];
		int chL, chR;
		bool dirL, dirR;
		float periodL, periodR;
		
		quadEncReadPositionAll(&chL, &dirL, &chR, &dirR);
		quadEncReadPeriodAll(&periodL, &periodR);
		
		// Set velocity direction
		if(dirL)
		{
			periodL = -periodL;
		}
		if(dirR)
		{
			periodR = -periodR;
		}
		
		int len = SNPRINTF(str, BUF_SIZE, "ch0 %c %7d %7.4f ch1 %c %7d %7.4f\r\n",
				dirL ? 'R':'F', chL, periodL,
				dirR ? 'R':'F', chR, periodR);
		udi_cdc_write_buf(str, len);
		
		vTaskDelay(200);
	}

#endif	//END quadEnc testing
}
