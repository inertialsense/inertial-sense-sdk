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

static void quadEncSetMode(Tc *const timercounter)
{
	//Init TC channel 0 to QDEC Position mode
	tc_init(timercounter, 0,
		TC_CMR_TCCLKS_XC0
		| TC_CMR_ETRGEDG_RISING /*To clear the counter on a rising edge of the TIOA signal*/
		| TC_CMR_ABETRG /*To select TIOA as a trigger for this channel 0 */
	);

#if 0	//Use with index signal
	//Init TC channel 1 to QDEC Rotation mode
	tc_init(timercounter, 1,
		/* QDEC Clock Selection */
		TC_CMR_TCCLKS_XC0
	);
#endif

	//Enable TC QDEC channel 0 in QDEC Position mode
	tc_set_block_mode(timercounter, TC_BMR_QDEN /*  QDEC mode enabled */
		| TC_BMR_POSEN /* Position measure is enabled */
		| TC_BMR_EDGPHA /* Detect quadrature on both PHA and PHB (4X decoding)*/
		| (0<< TC_BMR_MAXFILT_Pos) /* enable filter on input signal*/
	);	
	
	tc_start(timercounter, 0);
#if 0	//Use with index signal
	tc_start(timercounter, 1);
#endif
}

void quadEncInit(void)
{
	// Configure pins - Left Encoder
	ioport_set_pin_mode(PIN_QE0_PHA, PIN_QE0_PHA_MUX);
	ioport_disable_pin(PIN_QE0_PHA);
	ioport_set_pin_mode(PIN_QE0_PHB, PIN_QE0_PHB_MUX);
	ioport_disable_pin(PIN_QE0_PHB);

	// Configure pins - Right Encoder
	ioport_set_pin_mode(PIN_QE1_PHA, PIN_QE1_PHA_MUX);
	ioport_disable_pin(PIN_QE1_PHA);
	ioport_set_pin_mode(PIN_QE1_PHB, PIN_QE1_PHB_MUX);
	ioport_disable_pin(PIN_QE1_PHB);

	//Enable the QDEC channel clocks
	sysclk_enable_peripheral_clock(ID_QE0_CH0);
	sysclk_enable_peripheral_clock(ID_QE0_CH1);

	sysclk_enable_peripheral_clock(ID_QE1_CH0);
	sysclk_enable_peripheral_clock(ID_QE1_CH1);

	//Set mode
	quadEncSetMode(QE0);
	quadEncSetMode(QE1);
}

void quadEncReadAll(int *pos0, bool *dir0, int *pos1, bool *dir1)
{
	int16_t cv;

	taskENTER_CRITICAL();
	
	cv = QE0->TC_CHANNEL[0].TC_CV;
	*pos0 = cv;
	*dir0 = (QE0->TC_QISR & TC_QISR_DIR) / TC_QISR_DIR;
	
	cv = QE1->TC_CHANNEL[0].TC_CV;
	*pos1 = cv;
	*dir1 = (QE1->TC_QISR & TC_QISR_DIR) / TC_QISR_DIR;

	taskEXIT_CRITICAL();
}

void test_quad_encoders(void)
{
	#if	1	//quadEnc testing

	udi_cdc_write_buf("Running\r\n", 9);
	quadEncInit();

	while(1)
	{
		#define BUF_SIZE 100
		char str[BUF_SIZE];
		int chL, chR;
		bool dirL, dirR;
		
		quadEncReadAll(&chL, &dirL, &chR, &dirR);
		
		int len = sprintf(str, "ch0 %d %d ch1 %d %d\r\n", chL, dirL, chR, dirR);
		udi_cdc_write_buf(str, len);
		
		vTaskDelay(500);
	}

	#endif	//END quadEnc testing
}

void Read_quad_encoders(void)
{
	#if	1	//quadEnc reading

	udi_cdc_write_buf("Running\r\n", 9);
	quadEncInit();

	while(1)
	{
		#define BUF_SIZE 100
		char str[BUF_SIZE];
		int chL, chR;
		bool dirL, dirR;
		
		quadEncReadAll(&chL, &dirL, &chR, &dirR);
		
		int len = sprintf(str, "ch0 %d %d ch1 %d %d\r\n", chL, dirL, chR, dirR);
		udi_cdc_write_buf(str, len);
		
		vTaskDelay(500);
	}

	#endif	//END quadEnc read
}