/*
MIT LICENSE

Copyright 2014-2019 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT, IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/


#ifndef CONF_D_QUADENC_H_
#define CONF_D_QUADENC_H_

#if 0

// Left Wheel Encoder
// EVB H8-7: GPIO3 - QDEC0-A - TIOA0 - PA0
// EVB H8-8: GPIO4 - QDEC0-B - TIOB0 - PA1
#define QE0				TC0
#define ID_QE0_CH0		ID_TC0
#define ID_QE0_CH1		ID_TC1

#define PIN_QE0_PHA		(PIO_PA0_IDX)		//TC0 TIOA0
#define PIN_QE0_PHA_MUX	(IOPORT_MODE_MUX_B)

#define PIN_QE0_PHB		(PIO_PA1_IDX)		//TC0 TIOB0
#define PIN_QE0_PHB_MUX	(IOPORT_MODE_MUX_B)

#else

// Left Wheel Encoder
// EVB H8-9:  GPIO5 - QDEC2-A - TIOA6 - PC5
// EVB H8-10: GPIO6 - QDEC2-B - TIOB6 - PC6
#define QE0				TC2
#define ID_QE0_CH0		ID_TC6
#define ID_QE0_CH1		ID_TC7

#define PIN_QE0_PHA		(PIO_PC5_IDX)		//TC2 TIOA6
#define PIN_QE0_PHA_MUX	(IOPORT_MODE_MUX_B)

#define PIN_QE0_PHB		(PIO_PC6_IDX)		//TC2 TIOB6
#define PIN_QE0_PHB_MUX	(IOPORT_MODE_MUX_B)

#endif

// Right Wheel Encoder
// EVB H8-13: GPIO9  - QDEC3-A - TIOA9 - PE0
// EVB H8-14: GPIO10 - QDEC3-B - TIOB9 - PE1
#define QE1				TC3
#define ID_QE1_CH0		ID_TC9
#define ID_QE1_CH1		ID_TC10

#define PIN_QE1_PHA		(PIO_PE0_IDX)		//TC3 TIOA9
#define PIN_QE1_PHA_MUX	(IOPORT_MODE_MUX_B)

#define PIN_QE1_PHB		(PIO_PE1_IDX)		//TC3 TIOB9
#define PIN_QE1_PHB_MUX	(IOPORT_MODE_MUX_B)


#endif /* CONF_D_QUADENC_H_ */