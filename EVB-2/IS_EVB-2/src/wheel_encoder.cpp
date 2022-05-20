/*
MIT LICENSE

Copyright 2014-2019 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT, IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include <asf.h>
#include "globals.h"
#include "arm_math.h"
#include "drivers/d_quadEnc.h"
#include "wheel_encoder.h"


void init_wheel_encoder(void)
{
#ifdef CONF_BOARD_QUAD_ENCODER
    if(g_flashCfg->wheelCfgBits&WHEEL_CFG_BITS_ENABLE_ENCODER)
    {   
		quadEncInit(4);		// Sensing range: ~27ms
		quadEncSetDirectionReverse(g_flashCfg->wheelCfgBits&WHEEL_CFG_BITS_DIRECTION_REVERSE_LEFT, g_flashCfg->wheelCfgBits&WHEEL_CFG_BITS_DIRECTION_REVERSE_RIGHT);
    }
#endif
}


void step_wheel_encoder(is_comm_instance_t &comm)
{
#ifdef CONF_BOARD_QUAD_ENCODER           // Encoder Rx   =======================================================

    if(!(g_flashCfg->wheelCfgBits&WHEEL_CFG_BITS_ENABLE_ENCODER))
    {   // Wheel encoders disabled
        return;
    }

	if(g_flashCfg->wheelCfgBits&WHEEL_CFG_BITS_ENCODER_SOURCE)
	{
		return;	// EVB is providing wheel encoders
	}

	int chL, chR;
	bool dirL, dirR;
	int n=0;
	float periodL, periodR;
	static int encoderSendTimeMs=0;
	static wheel_encoder_t wheelEncoderLast = {0};

	++encoderSendTimeMs;
	if(encoderSendTimeMs >= (int)g_flashCfg->velocityControlPeriodMs)
	{  
		encoderSendTimeMs = 0;
		
		// Call read encoders
#if WHEEL_ENCODER_TIME_FROM_TOW == 1
		g_wheelEncoder.timeOfWeek = g_comm_time + g_towOffset;
#else
		g_wheelEncoder.timeOfWeek = g_comm_time;
#endif

		g_wheelEncoderTimeMs = (uint32_t)round(g_wheelEncoder.timeOfWeek*1000.0);
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
				
		// Convert encoder ticks to radians.
		g_wheelEncoder.theta_l = (chL * g_flashCfg->encoderTickToWheelRad) * 0.25; 	/* Division by 4 to account for 4x encoding */
		g_wheelEncoder.theta_r = (chR * g_flashCfg->encoderTickToWheelRad) * 0.25;

        // Convert TC pulse period to rad/sec.
        if(periodL!=0.0f)
        {			
            g_wheelEncoder.omega_l = g_flashCfg->encoderTickToWheelRad / periodL;
        }
        else
        {
            g_wheelEncoder.omega_l = 0.0f;
        }
        if(periodR!=0.0f)
		{
            g_wheelEncoder.omega_r = g_flashCfg->encoderTickToWheelRad / periodR;
        }
        else
        {
            g_wheelEncoder.omega_r = 0.0f;
        }
		
		// Encoder Wrap count (currently counting revolutions) 
//		g_wheelEncoder.wrap_count_l = g_wheelEncoder.theta_l / (2*PI);
//		g_wheelEncoder.wrap_count_r = g_wheelEncoder.theta_r / (2*PI);	

		if (g_ermc.bits & ERMC_BITS_WHEEL_ENCODER)
		{
			n = is_comm_data(&comm, DID_WHEEL_ENCODER, 0, sizeof(wheel_encoder_t), (void*)&(g_wheelEncoder));

#if 1		// Send to uINS
			comWrite(EVB2_PORT_UINS0, comm.buf.start, n, LED_INS_TXD_PIN);
#else		// Send to Luna
			comWrite(EVB2_PORT_USB, comm.buf.start, n, 0);
#endif
		}

#if 0
		n = is_comm_data(&comm, DID_EVB_DEBUG_ARRAY, 0, sizeof(debug_array_t), (void*)&(g_debug));
		comWrite(EVB2_PORT_USB, comm.buf.start, n, 0);
#endif	

#if 0
		n = is_comm_data(&comm, DID_EVB_DEBUG_ARRAY, 0, sizeof(debug_array_t), (void*)&(g_debug));
		comWrite(EVB2_PORT_USB, comm.buf.start, n, 0);
#endif

		// Update history
		wheelEncoderLast = g_wheelEncoder;
	}
	
#endif
}