/*
MIT LICENSE

Copyright 2014-2019 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT, IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#ifndef __COMMUNICATIONS_H_
#define __COMMUNICATIONS_H_

#include <asf.h>
#include "globals.h"
#include "spiTouINS.h"
#include "../../../../SDK/src/ISLogger.h"
#include "../../SDK/hw-libs/communications/CAN_comm.h"


extern bool                     g_usb_cdc_open;


int comWrite(int serialNum, const unsigned char *buf, int size, uint32_t ledPin );
int comRead(int serialNum, unsigned char *buf, int size, uint32_t ledPin);
int comRxUsed(int serialNum);

void callback_cdc_set_config(uint8_t port, usb_cdc_line_coding_t * cfg);
void callback_cdc_disable(void);
void callback_cdc_set_dtr(uint8_t port, bool b_enable);

void uINS0_stream_stop_all(is_comm_instance_t &comm);
void uINS0_stream_enable_std(is_comm_instance_t &comm);
void uINS0_stream_enable_PPD(is_comm_instance_t &comm);

void parse_uINS_data(cISLogger &logger, is_comm_instance_t &comm);
void parse_EVB_data(int len, is_comm_instance_t *comm, uint8_t *buf);

void com_bridge_forward(uint32_t srcPort, uint8_t *buf, int len);
void step_com_bridge(is_comm_instance_t &comm);
void communications_init(void);

/*CAN Message*/
typedef struct PACKED
{
	uint8_t startByte;
	uint32_t address;
	uint8_t dlc; //Data Length
	is_can_payload payload;
	uint8_t endByte;
		
} is_can_message;


#endif // __COMMUNICATIONS_H_
