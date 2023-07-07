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
#include "../../../src/ISLogger.h"
#include "../hw-libs/communications/CAN_comm.h"


extern is_comm_instance_t 		g_commTx;
extern bool                     g_usb_cdc_open;

typedef void(*pfnHandleUinsData)(p_data_hdr_t &dataHdr, uDatasets &data);
typedef void(*pfnHandleHostData)(is_comm_instance_t *comm, protocol_type_t ptype, uint32_t srcPort);
typedef void(*pfnHandleBroadcst)(is_comm_instance_t *comm, uint32_t didSendNow);
typedef uint64_t(*pfnHandleDid2Ermc)(uint32_t dataId);

int comWrite(int serialNum, const unsigned char *buf, int size, uint32_t ledPin );
int comRead(int serialNum, unsigned char *buf, int size, uint32_t ledPin);
int comRxUsed(int serialNum);

void callback_cdc_set_config(uint8_t port, usb_cdc_line_coding_t * cfg);
void callback_cdc_disable(void);
void callback_cdc_set_dtr(uint8_t port, bool b_enable);

void uINS_stream_stop_all(void);
void uINS_stream_enable_std(void);
void uINS_stream_enable_PPD(void);

void broadcastRmcMessage(is_comm_instance_t *comm, uint32_t did, uint32_t size, void* data, uint32_t &time_ms, uint32_t didSendNow);
void step_broadcast_data(is_comm_instance_t *comm, uint32_t didSendNow=0);
void log_uINS_data(cISLogger &logger, is_comm_instance_t &comm);
void setErmcBroadcastBits(is_comm_instance_t *comm, uint32_t srcPort, uint64_t bits);

void com_bridge_smart_forward(uint32_t srcPort, uint32_t ledPin);
void com_bridge_forward(uint32_t srcPort, uint8_t *buf, int len);
void step_com_bridge(is_comm_instance_t &comm);
void comunications_set_uins_data_callback( pfnHandleUinsData pfn );
void comunications_set_host_data_callback( pfnHandleHostData pfn );
void communications_init( pfnHandleBroadcst pfnHandleBroadcst = NULLPTR, pfnHandleDid2Ermc pfnDid2Ermc = NULLPTR );

/*CAN Message*/
typedef struct PACKED
{
	uint8_t startByte;
	uint32_t address;
	uint8_t dlc; //Data Length
	is_can_payload payload;
	uint8_t endByte;
		
} is_can_message;

typedef struct PACKED
{
	uint32_t			marker;
	protocol_type_t		ptype;
	uint32_t			size;
} is_evb_log_stream;


#endif // __COMMUNICATIONS_H_
