/*
MIT LICENSE

Copyright (c) 2014-2023 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include "com_manager.h"
#include <string.h>
#include <stdlib.h>


// enable filtering of duplicate packets
#define ENABLE_FILTER_DUPLICATE_PACKETS 1

// whether the first character or all characters are checked in duplicate packets
#define ENABLE_FILTER_DUPLICATE_PACKETS_MATCH_ALL_CHARACTERS 0

#define PARSE_DOUBLE(str) strtod(str, 0)
#define PARSE_FLOAT(str) strtof(str, 0)

#define MIN_REQUEST_PERIOD_MS       1               // (ms) 1 KHz
#define MAX_REQUEST_PERIOD_MS       65000           // (ms)
#define MSG_PERIOD_SEND_ONCE		-1
#define MSG_PERIOD_DISABLED			0

static com_manager_t g_cm;

int initComManagerInstanceInternal
(
	com_manager_t* cmInstance,
	int numPorts,
	int stepPeriodMilliseconds,
	pfnComManagerRead portReadFnc,
	pfnIsCommPortWrite portWriteFnc,
	pfnComManagerSendBufferAvailableBytes txFreeFnc,
	pfnComManagerPostRead pstRxFnc,
	pfnComManagerPostAck pstAckFnc,
	pfnComManagerDisableBroadcasts disableBcastFnc,
	com_manager_init_t *buffers,
	com_manager_port_t *cmPorts,
	uint32_t *timeMs
);
// int processAsciiRxPacket(com_manager_t* cmInstance, int pHandle, unsigned char* start, int count);
// void parseAsciiPacket(com_manager_t* cmInstance, int pHandle, unsigned char* buf, int count);
int processBinaryRxPacket(com_manager_t* cmInstance, int pHandle, packet_t *pkt);
void enableBroadcastMsg(com_manager_t* cmInstance, broadcast_msg_t *msg, int periodMultiple);
void disableBroadcastMsg(com_manager_t* cmInstance, broadcast_msg_t *msg);
void disableDidBroadcast(com_manager_t* cmInstance, int pHandle, p_data_disable_t *disable);
int sendDataPacket(com_manager_t* cmInstance, int pHandle, packet_t *pkt);
void sendAck(com_manager_t* cmInstance, int pHandle, packet_t *pkt, uint8_t pTypeFlags);
int findAsciiMessage(const void * a, const void * b);
int asciiMessageCompare(const void* elem1, const void* elem2);
void stepComManagerSendMessages(void);
void stepComManagerSendMessagesInstance(CMHANDLE cmInstance);

static int comManagerStepRxInstanceHandler(com_manager_t* cmInstance, com_manager_port_t* port, is_comm_instance_t* comm, int32_t pHandle, protocol_type_t ptype);

CMHANDLE comManagerGetGlobal(void) { return &g_cm; }

int comManagerInit
(
	int numPorts,
	int stepPeriodMilliseconds,
	pfnComManagerRead portReadFnc,
	pfnIsCommPortWrite portWriteFnc,
	pfnComManagerSendBufferAvailableBytes txFreeFnc,
	pfnComManagerPostRead pstRxFnc,
	pfnComManagerPostAck pstAckFnc,
	pfnComManagerDisableBroadcasts disableBcastFnc,
	com_manager_init_t *buffers,
	com_manager_port_t *cmPorts,
	uint32_t *timeMs
)
{
	return initComManagerInstanceInternal(
		&g_cm, 
		numPorts, 
		stepPeriodMilliseconds, 
		portReadFnc, 
		portWriteFnc, 
		txFreeFnc, 
		pstRxFnc, 
		pstAckFnc, 
		disableBcastFnc, 
		buffers,
		cmPorts,
		timeMs);
}

int comManagerInitInstance
(
	CMHANDLE cmHandle,
	int numPorts,
	int stepPeriodMilliseconds,
	pfnComManagerRead portReadFnc,
	pfnIsCommPortWrite portWriteFnc,
	pfnComManagerSendBufferAvailableBytes txFreeFnc,
	pfnComManagerPostRead pstRxFnc,
	pfnComManagerPostAck pstAckFnc,
	pfnComManagerDisableBroadcasts disableBcastFnc,
	com_manager_init_t *buffers,
	com_manager_port_t *cmPorts,
	uint32_t *timeMs
)
{
	int result = 0;

	com_manager_t* cmInstance = (com_manager_t*)cmHandle;
	if (cmInstance != 0)
	{
		memset(cmInstance, 0, sizeof(com_manager_t));
		result = initComManagerInstanceInternal(
			cmInstance, 
			numPorts, 
			stepPeriodMilliseconds, 
			portReadFnc, 
			portWriteFnc, 
			txFreeFnc, 
			pstRxFnc, 
			pstAckFnc, 
			disableBcastFnc,
			buffers, 
			cmPorts, 
			timeMs);
	}
	return result;
}

int initComManagerInstanceInternal
(
	com_manager_t* cmInstance,
	int numPorts,
	int stepPeriodMilliseconds,
	pfnComManagerRead portReadFnc,
	pfnIsCommPortWrite portWriteFnc,
	pfnComManagerSendBufferAvailableBytes txFreeFnc,
	pfnComManagerPostRead pstRxFnc,
	pfnComManagerPostAck pstAckFnc,
	pfnComManagerDisableBroadcasts disableBcastFnc,
	com_manager_init_t *buffers,
	com_manager_port_t *cmPorts, 
	uint32_t *timeMs
)
{
	int32_t i;

	if (numPorts <= 0)
	{
		return -1;
	}
	numPorts = _CLAMP(numPorts, 1, 1024);

	// assign new variables
	cmInstance->portRead = portReadFnc;
	cmInstance->portWrite = portWriteFnc;
	cmInstance->txFree = txFreeFnc;
	cmInstance->pstRxFnc = pstRxFnc;
	cmInstance->pstAckFnc = pstAckFnc;
	cmInstance->disableBcastFnc = disableBcastFnc;
	cmInstance->numPorts = numPorts;
	cmInstance->stepPeriodMilliseconds = stepPeriodMilliseconds;
	cmInstance->cmMsgHandlerAscii = NULL;
	cmInstance->cmMsgHandlerUblox = NULL;
	cmInstance->cmMsgHandlerRtcm3 = NULL;

	if (buffers == NULL)
	{
		return -1;
	}
	
	// Buffer: message broadcasts
	if (buffers->broadcastMsg == NULL || buffers->broadcastMsgSize < COM_MANAGER_BUF_SIZE_BCAST_MSG(MAX_NUM_BCAST_MSGS))
	{
		return -1;
	}
	cmInstance->broadcastMessages = (broadcast_msg_t*)buffers->broadcastMsg;
	memset(cmInstance->broadcastMessages, 0, buffers->broadcastMsgSize);
		
	// Port specific info
	cmInstance->ports = cmPorts;
	for (i = 0; i < numPorts; i++)
	{	// Initialize IScomm instance, for serial reads / writes
		com_manager_port_t *port = &(cmInstance->ports[i]);
		is_comm_init(&(port->comm), port->comm_buffer, MEMBERSIZE(com_manager_port_t, comm_buffer), timeMs);
					
#if ENABLE_PACKET_CONTINUATION			
		// Packet data continuation
		memset(&(port->con), 0, MEMBERSIZE(com_manager_port_t,con));
#endif
	}

	return 0;
}

int asciiMessageCompare(const void* elem1, const void* elem2)
{
	asciiMessageMap_t* e1 = (asciiMessageMap_t*)elem1;
	asciiMessageMap_t* e2 = (asciiMessageMap_t*)elem2;

	return memcmp(e1->messageId, e2->messageId, 4);
}

void comManagerRegister(uint16_t did, pfnComManagerPreSend txFnc, pfnComManagerPostRead pstRxFnc, const void* txDataPtr, void* rxDataPtr, uint16_t size, uint8_t pktFlags)
{
	comManagerRegisterInstance(&g_cm, did, txFnc, pstRxFnc, txDataPtr, rxDataPtr, size, pktFlags);
}

void comManagerRegisterInstance(CMHANDLE cmInstance_, uint16_t did, pfnComManagerPreSend txFnc, pfnComManagerPostRead pstRxFnc, const void* txDataPtr, void* rxDataPtr, uint16_t size, uint8_t pktFlags)
{
	com_manager_t* cmInstance = (com_manager_t*)cmInstance_;

	// Validate ID and data pointer
	if (did >= DID_COUNT_UINS)
	{
		return;
	}

	// Function called to update struct before data is sent
	cmInstance->regData[did].preTxFnc = txFnc;

	// Function called after data is received and struct is updated
	cmInstance->regData[did].pstRxFnc = pstRxFnc;

	// Pointer to data struct for Tx
	cmInstance->regData[did].dataSet.txPtr = (unsigned char*)txDataPtr;

	// Pointer to data struct for Rx
	cmInstance->regData[did].dataSet.rxPtr = (unsigned char*)rxDataPtr;

	// Size of data struct
	cmInstance->regData[did].dataSet.size = size;
	
	// Packet flags
	cmInstance->regData[did].pktFlags = pktFlags;
}

void comManagerStep(void)
{
	comManagerStepRxInstance(&g_cm);
	comManagerStepTxInstance(&g_cm);
}

void comManagerStepInstance(CMHANDLE cmInstance_)
{
	com_manager_t* cmInstance = (com_manager_t*)cmInstance_;
	comManagerStepRxInstance(cmInstance);
	comManagerStepTxInstance(cmInstance);
}

void comManagerStepRxInstance(CMHANDLE cmInstance_)
{
	com_manager_t* cmInstance = (com_manager_t*)cmInstance_;
	int32_t port;
	
	if (!cmInstance->portRead)
	{
		return;
	}
		
	for (port = 0; port < cmInstance->numPorts; port++)
	{
		com_manager_port_t *cmPort = &(cmInstance->ports[port]);
		is_comm_instance_t *comm = &(cmPort->comm);
		protocol_type_t ptype;

		// Get available size of comm buffer
		int n = is_comm_free(comm);

		// Read data directly into comm buffer
		if ((n = cmInstance->portRead(port, comm->rxBuf.tail, n)))
		{
			// Update comm buffer tail pointer
			comm->rxBuf.tail += n;

			// Search comm buffer for valid packets
			while ((ptype = is_comm_parse(comm)) != _PTYPE_NONE)
			{	
				int error = comManagerStepRxInstanceHandler(cmInstance, cmPort, comm, port, ptype);		
				if(error == CM_ERROR_FORWARD_OVERRUN) 
				{
					break;	// Stop parsing and continue in outer loop
				}
			}
		}			
	}
}

static int comManagerStepRxInstanceHandler(com_manager_t* cmInstance, com_manager_port_t* cmPort, is_comm_instance_t* comm, int32_t port, protocol_type_t ptype)
{
	int error = 0;
	uint8_t *data = comm->rxPkt.data.ptr + comm->rxPkt.offset;
	uint16_t size = comm->rxPkt.data.size;

	switch (ptype)
	{
	case _PTYPE_PARSE_ERROR:
		error = 1;
		break;

	case _PTYPE_INERTIAL_SENSE_DATA:
	case _PTYPE_INERTIAL_SENSE_CMD:
		error = processBinaryRxPacket(cmInstance, port, &(comm->rxPkt));
		break;

	case _PTYPE_UBLOX:
		if (cmInstance->cmMsgHandlerUblox)
		{
			error = cmInstance->cmMsgHandlerUblox(port, data, size);
		}
		break;

	case _PTYPE_RTCM3:
		if (cmInstance->cmMsgHandlerRtcm3)
		{
			error = cmInstance->cmMsgHandlerRtcm3(port, data, size);
		}
		break;

	case _PTYPE_NMEA:
		if (cmInstance->cmMsgHandlerAscii)
		{
			error = cmInstance->cmMsgHandlerAscii(port, data, size);
		}
		break;
	case _PTYPE_SPARTN:
		if (cmInstance->cmMsgHandlerSpartn)
		{
			error = cmInstance->cmMsgHandlerSpartn(port, data, size);
		}
		break;
	default:
		break;
	}

	return error;
}

void comManagerStepTxInstance(CMHANDLE cmInstance_)
{
	com_manager_t* cmInstance = (com_manager_t*)cmInstance_;
	stepComManagerSendMessagesInstance(cmInstance);
}

void stepComManagerSendMessages(void)
{
	stepComManagerSendMessagesInstance(&g_cm);
}

void stepComManagerSendMessagesInstance(CMHANDLE cmInstance_)
{
	com_manager_t* cmInstance = cmInstance_;
	
	// Send data (if necessary)
	for (broadcast_msg_t* bcPtr = cmInstance->broadcastMessages, *ptrEnd = (cmInstance->broadcastMessages + MAX_NUM_BCAST_MSGS); bcPtr < ptrEnd; bcPtr++)
	{
		// If send buffer does not have space, exit out
		if (cmInstance->txFree && (bcPtr->pkt.size > (uint32_t)cmInstance->txFree(bcPtr->pHandle)))
		{
			break;
		}
		// Send once and remove from message queue
		else if (bcPtr->period == MSG_PERIOD_SEND_ONCE)
		{
			sendDataPacket(cmInstance, bcPtr->pHandle, &(bcPtr->pkt));
			disableBroadcastMsg(cmInstance, bcPtr);
		}
		// Broadcast messages
		else if (bcPtr->period > 0)
		{
			// Check if counter has expired
			if (++bcPtr->counter >= bcPtr->period)
			{
				bcPtr->counter = 0;    // reset counter

				// Prep data if callback exists
				unsigned int id = bcPtr->pkt.hdr.id;
				int sendData = 1;
				if (id<DID_COUNT_UINS && cmInstance->regData[id].preTxFnc)
				{					
					sendData = cmInstance->regData[id].preTxFnc(bcPtr->pHandle, &bcPtr->pkt.dataHdr);
				}
				if (sendData)
				{
					sendDataPacket(cmInstance, bcPtr->pHandle, &(bcPtr->pkt));
				}
			}
		}
	}
}

void comManagerSetCallbacks(
	pfnComManagerAsapMsg handlerRmc,
	pfnComManagerGenMsgHandler handlerAscii,
	pfnComManagerGenMsgHandler handlerUblox, 
	pfnComManagerGenMsgHandler handlerRtcm3,
	pfnComManagerGenMsgHandler handlerSpartn)
{
	comManagerSetCallbacksInstance(&g_cm, handlerRmc, handlerAscii, handlerUblox, handlerRtcm3, handlerSpartn);
}

void comManagerSetCallbacksInstance(CMHANDLE cmInstance, 
	pfnComManagerAsapMsg handlerRmc,
	pfnComManagerGenMsgHandler handlerAscii,
	pfnComManagerGenMsgHandler handlerUblox,
	pfnComManagerGenMsgHandler handlerRtcm3,
	pfnComManagerGenMsgHandler handlerSpartn)
{
	if (cmInstance != 0)
	{
		((com_manager_t*)cmInstance)->cmMsgHandlerRmc = handlerRmc;
		((com_manager_t*)cmInstance)->cmMsgHandlerAscii = handlerAscii;
		((com_manager_t*)cmInstance)->cmMsgHandlerUblox = handlerUblox;
		((com_manager_t*)cmInstance)->cmMsgHandlerRtcm3 = handlerRtcm3;
		((com_manager_t*)cmInstance)->cmMsgHandlerSpartn = handlerSpartn;
	}
}

void comManagerAssignUserPointer(CMHANDLE cmInstance, void* userPointer)
{
	((com_manager_t*)cmInstance)->userPointer = userPointer;
}

void* comManagerGetUserPointer(CMHANDLE cmInstance)
{
	return ((com_manager_t*)cmInstance)->userPointer;
}

is_comm_instance_t* comManagerGetIsComm(int pHandle)
{
	return comManagerGetIsCommInstance(&g_cm, pHandle);
}

is_comm_instance_t* comManagerGetIsCommInstance(CMHANDLE cmInstance, int pHandle)
{
	com_manager_t *cm = (com_manager_t*)cmInstance;
	
	if(cm->numPorts <= 0 || pHandle < 0 || pHandle >= cm->numPorts)
	{
		return NULL;
	}
	
	return &(cm->ports[pHandle].comm);
}

/**
*   @brief Request data
*   This function requests the specified data w/ offset and length
*   for partial reads.
*
*	@param[in] dataId       Data structure ID
*	@param[in] offset   Byte offset into data structure.  0 = data start.
*	@param[in] length   Byte length of data.  0 = entire structure.
*	@param[in] periodMultiple Broadcast period of requested data.  0 = single request.
*
*	@return 0 on successful request.  -1 on failure.
*/
void comManagerGetData(int pHandle, uint16_t did, uint16_t size, uint16_t offset, uint16_t period)
{
	comManagerGetDataInstance(&g_cm, pHandle, did, size, offset, period);
}

void comManagerGetDataInstance(CMHANDLE cmInstance, int pHandle, uint16_t did, uint16_t size, uint16_t offset, uint16_t period)
{
	// Create and Send request packet
    p_data_get_t get;
    get.id = did;
    get.offset = offset;
    get.size = size;
    get.period = period;

    comManagerSendInstance(cmInstance, pHandle, PKT_TYPE_GET_DATA, &get, 0, sizeof(get), 0);
}

void comManagerGetDataRmc(int pHandle, uint64_t rmcBits, uint32_t rmcOptions)
{
	comManagerGetDataRmcInstance(&g_cm, pHandle, rmcBits, rmcOptions);
}

void comManagerGetDataRmcInstance(CMHANDLE cmInstance, int pHandle, uint64_t rmcBits, uint32_t rmcOptions)
{
	rmc_t rmc;
	rmc.bits = rmcBits;
	rmc.options = rmcOptions;

	comManagerSendDataInstance(cmInstance, pHandle, &rmc, DID_RMC, sizeof(rmc_t), 0);
}

int comManagerSendData(int pHandle, void *data, uint16_t did, uint16_t size, uint16_t offset)
{	
	return comManagerSendDataInstance(&g_cm, pHandle, data, did, size, offset);
}

int comManagerSendDataInstance(CMHANDLE cmInstance, int pHandle, void* data, uint16_t did, uint16_t size, uint16_t offset)
{
	return comManagerSendInstance(cmInstance, pHandle, PKT_TYPE_SET_DATA, data, did, size, offset);
}

int comManagerSendDataNoAck(int pHandle, void *data, uint16_t did, uint16_t size, uint16_t offset)
{
	return comManagerSendDataNoAckInstance(&g_cm, pHandle, data, did, size, offset);
}

int comManagerSendDataNoAckInstance(CMHANDLE cmInstance, int pHandle, void *data, uint16_t did, uint16_t size, uint16_t offset)
{
	return comManagerSendInstance((com_manager_t*)cmInstance, pHandle, PKT_TYPE_DATA, data, did, size, offset);
}

int comManagerSendRawData(int pHandle, void *data, uint16_t did, uint16_t size, uint16_t offset)
{
	return comManagerSendRawDataInstance(&g_cm, pHandle, data, did, size, offset);
}

int comManagerSendRawDataInstance(CMHANDLE cmInstance, int pHandle, void* data, uint16_t did, uint16_t size, uint16_t offset)
{
	return comManagerSendInstance((com_manager_t*)cmInstance, pHandle, PKT_TYPE_SET_DATA, data, did, size, offset);
}

int comManagerDisableData(int pHandle, uint16_t did)
{
	return comManagerDisableDataInstance(&g_cm, pHandle, did);
}

int comManagerDisableDataInstance(CMHANDLE cmInstance, int pHandle, uint16_t did)
{
	return comManagerSendInstance(cmInstance, pHandle, PKT_TYPE_STOP_DID_BROADCAST, &did, 0, 0, 0);
}

int comManagerSend(int pHandle, uint8_t pFlags, void* data, uint16_t size, uint16_t did, uint16_t offset)
{
	return comManagerSendInstance(&g_cm, pHandle, pFlags, data, did, size, offset);
}

int comManagerSendInstance(CMHANDLE cmInstance, int port, uint8_t pFlags, void *data, uint16_t did, uint16_t size, uint16_t offset)
{
	com_manager_t *cm = (com_manager_t*)cmInstance;
	return is_comm_write(cm->portWrite, port, &(cm->ports[port].comm), pFlags, did, size, offset, data);
}

int findAsciiMessage(const void * a, const void * b)
{
	unsigned char* a1 = (unsigned char*)a;
	asciiMessageMap_t* a2 = (asciiMessageMap_t*)b;

	return memcmp(a1, a2->messageId, 4);
}

/**
*   @brief Process binary packet content:
*
*	@return 0 on success.  -1 on failure.
*/
int processBinaryRxPacket(com_manager_t* cmInstance, int pHandle, packet_t *pkt)
{
	packet_hdr_t        *hdr = &(pkt->hdr);
	registered_data_t   *regd = NULL;
	uint8_t             ptype = (uint8_t)(pkt->hdr.flags&PKT_TYPE_MASK);

	switch (ptype)
	{
	default:    // Data ID Unknown
		return -1;

	case PKT_TYPE_SET_DATA:
	case PKT_TYPE_DATA:
	{		
		// Validate Data
		if (hdr->id < DID_COUNT_UINS)
		{
			regd = &(cmInstance->regData[hdr->id]);

			// Validate and constrain Rx data size to fit within local data struct
			if (regd->dataSet.size && (pkt->offset + hdr->payloadSize) > regd->dataSet.size)
			{
				// trim the size down so it fits
				uint16_t size = (int)(regd->dataSet.size - pkt->offset);
				if (size < 4)
				{
					// we are completely out of bounds, we cannot process this message at all
					// the minimum data struct size is 4 bytes
					return -1;
				}

				// Update Rx data size
				hdr->payloadSize = _MIN(hdr->payloadSize, (uint8_t)size);
			}
		}

		p_data_t *data = (p_data_t*)&(pkt->dataHdr);

#if ENABLE_PACKET_CONTINUATION

		// Consolidate datasets that were broken-up across multiple packets
		p_data_t* con = &cmInstance->ports[pHandle].con;
		if (additionalDataAvailable || (con->hdr.size != 0 && con->hdr.id == dataHdr->id))
		{
			// New dataset
			if (con->hdr.id == 0 || con->hdr.size == 0 || con->hdr.id != dataHdr->id || con->hdr.size > dataHdr->offset)
			{
				// Reset data consolidation
				con->hdr.id = dataHdr->id;
				con->hdr.offset = dataHdr->offset;
				con->hdr.size = 0;
			}

			// Ensure data will fit in buffer
			if ((con->hdr.size + dataHdr->size) < sizeof(con->buf))
			{
				// Add data to buffer
				memcpy(con->buf + con->hdr.size, data->buf, dataHdr->size);
				con->hdr.size += dataHdr->size;
			}
			else
			{
				// buffer overflow
			}

			// Wait for end of data
			if (additionalDataAvailable)
			{
				return 0;
			}

			// Use consolidated data
			data = con;
		}
		
#else
	
// 		unsigned char additionalDataAvailable // function parameter removed 
// 		(void)additionalDataAvailable;

#endif

		if (regd)
		{
			// Write to data structure if it was registered
			if (regd->dataSet.rxPtr)
			{
				copyDataPToStructP(regd->dataSet.rxPtr, data, regd->dataSet.size);
			}

			// Call data specific callback after data has been received
			if (regd->pstRxFnc)
			{
				regd->pstRxFnc(pHandle, data);
			}
		}

		// Call general/global callback
		if (cmInstance->pstRxFnc)
		{
			cmInstance->pstRxFnc(pHandle, data);
		}

#if ENABLE_PACKET_CONTINUATION

		// Clear dataset consolidation
		con->hdr.id = con->hdr.size = con->hdr.offset = 0;

#endif

		// Reply w/ ACK for PKT_TYPE_SET_DATA
		if (ptype == PKT_TYPE_SET_DATA)
		{
			sendAck(cmInstance, pHandle, pkt, PKT_TYPE_ACK);
		}
	}
		break;

	case PKT_TYPE_GET_DATA:
		if (comManagerGetDataRequestInstance(cmInstance, pHandle, (p_data_get_t*)(pkt->data.ptr)))
		{
			sendAck(cmInstance, pHandle, pkt, PKT_TYPE_NACK);
		}
		break;

	case PKT_TYPE_STOP_BROADCASTS_ALL_PORTS:
		comManagerDisableBroadcastsInstance(cmInstance, -1);

		// Call disable broadcasts callback if exists
		if (cmInstance->disableBcastFnc)
		{
			cmInstance->disableBcastFnc(-1);
		}
		sendAck(cmInstance, pHandle, pkt, PKT_TYPE_ACK);
		break;

	case PKT_TYPE_STOP_BROADCASTS_CURRENT_PORT:
		comManagerDisableBroadcastsInstance(cmInstance, pHandle);

		// Call disable broadcasts callback if exists
		if (cmInstance->disableBcastFnc)
		{
			cmInstance->disableBcastFnc(pHandle);
		}
		sendAck(cmInstance, pHandle, pkt, PKT_TYPE_ACK);
		break;

	case PKT_TYPE_STOP_DID_BROADCAST:
		disableDidBroadcast(cmInstance, pHandle, (p_data_disable_t*)(pkt->data.ptr));
		break;

	case PKT_TYPE_NACK:
	case PKT_TYPE_ACK:
		// Call general ack callback
		if (cmInstance->pstAckFnc)
		{
			cmInstance->pstAckFnc(pHandle, (p_ack_t*)(pkt->data.ptr), ptype);
		}
		break;
	}

	return 0;
}

bufTxRxPtr_t* comManagerGetRegisteredDataInfo(uint16_t did)
{
	return comManagerGetRegisteredDataInfoInstance(&g_cm, did);
}

bufTxRxPtr_t* comManagerGetRegisteredDataInfoInstance(CMHANDLE _cmInstance, uint16_t did)
{
	if (did < DID_COUNT_UINS)
	{
		com_manager_t* cmInstance = (com_manager_t*)_cmInstance;
		return &cmInstance->regData[did].dataSet;
	}

	return 0;
}

// 0 on success. -1 on failure.
int comManagerGetDataRequest(int port, p_data_get_t* req)
{
	return comManagerGetDataRequestInstance(&g_cm, port, req);
}

int comManagerGetDataRequestInstance(CMHANDLE _cmInstance, int pHandle, p_data_get_t* req)
{
	com_manager_t* cmInstance = (com_manager_t*)_cmInstance;
	broadcast_msg_t* msg = 0;

	// Validate the request
	if (req->id >= DID_COUNT)
	{
		// invalid data id
		return -1;
	}
	// Call RealtimeMessageController (RMC) handler
	else if (cmInstance->cmMsgHandlerRmc && (cmInstance->cmMsgHandlerRmc(pHandle, req) == 0))
	{
		// Don't allow comManager broadcasts for messages handled by RealtimeMessageController. 
		return 0;
	}
	// if size is 0 and offset is 0, set size to full data struct size
	else if (req->size == 0 && req->offset == 0 && req->id < DID_COUNT)
	{
		req->size = cmInstance->regData[req->id].dataSet.size;
	}
	
	// Copy reference to source data
	bufTxRxPtr_t* dataSetPtr = &cmInstance->regData[req->id].dataSet;

	if (req->offset + req->size > dataSetPtr->size)
	{
		req->offset = 0;
		req->size = dataSetPtr->size;
	}

	// Search for matching message (i.e. matches pHandle, id, size, and offset)...
	for (broadcast_msg_t* bcPtr = cmInstance->broadcastMessages, *ptrEnd = (cmInstance->broadcastMessages + MAX_NUM_BCAST_MSGS); bcPtr < ptrEnd; bcPtr++)
	{
		if (bcPtr->pHandle == pHandle && bcPtr->pkt.hdr.id == req->id && bcPtr->pkt.hdr.payloadSize == req->size && bcPtr->pkt.offset == req->offset)
		{
			msg = bcPtr;
			break;
		}
	}

	// otherwise use the first available (period=0) message.
	if (msg == 0)
	{
		for (broadcast_msg_t* bcPtr = cmInstance->broadcastMessages, *ptrEnd = (cmInstance->broadcastMessages + MAX_NUM_BCAST_MSGS); bcPtr < ptrEnd; bcPtr++)
		{
			if (bcPtr->period <= MSG_PERIOD_DISABLED)
			{
				msg = bcPtr;
				break;
			}
		}

		if (msg == 0)
		{
			// use last slot, force overwrite
			msg = cmInstance->broadcastMessages + MAX_NUM_BCAST_MSGS - 1;
		}
	}

	msg->pHandle = pHandle;
	packet_t *pkt = &(msg->pkt);
	uint8_t *dataPtr;
	if (dataSetPtr->txPtr)
	{
		dataPtr = cmInstance->regData[req->id].dataSet.txPtr + req->offset;
	}
	else
	{
		dataPtr = NULL;
	}
	is_comm_encode_hdr(pkt, PKT_TYPE_DATA, req->id, req->size, req->offset, dataPtr);

	// Prep data if callback exists
	int sendData = 1;
	if (cmInstance->regData[req->id].preTxFnc)
	{
		sendData = cmInstance->regData[req->id].preTxFnc(pHandle, &(pkt->dataHdr));
	}

	if (req->id == DID_REFERENCE_IMU)
	{
		return -1;
	}
	
	// Constrain request broadcast period if necessary
	if (req->period != 0)
	{
		_LIMIT2(req->period, MIN_REQUEST_PERIOD_MS, MAX_REQUEST_PERIOD_MS);
	}

	// Send data
	if (req->period > 0)
	{
		// ***  Request Broadcast  ***
		// Send data immediately if possible
		if (cmInstance->txFree == 0 || pkt->size <= (uint32_t)cmInstance->txFree(pHandle))
		{
			if (sendData)
			{
				sendDataPacket(cmInstance, pHandle, &(msg->pkt));
			}
		}

		// Enable broadcast message
		enableBroadcastMsg(cmInstance, msg, req->period);
	}
	else
	{
		// ***  Request Single  ***
		// Send data immediately if possible
		if (cmInstance->txFree == 0 || pkt->size <= (uint32_t)cmInstance->txFree(pHandle))
		{
			if (sendData)
			{
				sendDataPacket(cmInstance, pHandle, &(msg->pkt));
			}
			disableBroadcastMsg(cmInstance, msg);
		}
		else
		{
			// Won't fit in queue, so send it later
			enableBroadcastMsg(cmInstance, msg, req->period);
		}
	}

	return 0;
}

void enableBroadcastMsg(com_manager_t* cmInstance, broadcast_msg_t* msg, int periodMultiple)
{
	// Update broadcast period
	if (periodMultiple > 0)
	{
		msg->period = periodMultiple / cmInstance->stepPeriodMilliseconds;
	}
	else
	{
		msg->period = MSG_PERIOD_SEND_ONCE;
	}
	msg->counter = -1;   // Keeps broadcast from sending for at least one period
}

void disableBroadcastMsg(com_manager_t* cmInstance, broadcast_msg_t *msg)
{
	(void)cmInstance;

	// Remove item from linked list
	msg->period = MSG_PERIOD_DISABLED;
}

void comManagerDisableBroadcasts(int pHandle)
{
	comManagerDisableBroadcastsInstance(&g_cm, pHandle);
}

void comManagerDisableBroadcastsInstance(CMHANDLE cmInstance_, int pHandle)
{
	com_manager_t* cmInstance = (com_manager_t*)cmInstance_;
	for (broadcast_msg_t* bcPtr = cmInstance->broadcastMessages, *ptrEnd = (cmInstance->broadcastMessages + MAX_NUM_BCAST_MSGS); bcPtr < ptrEnd; bcPtr++)
	{
		if (pHandle < 0 || bcPtr->pHandle == pHandle)
		{
			bcPtr->period = MSG_PERIOD_DISABLED;
		}
	}
}

void disableDidBroadcast(com_manager_t* cmInstance, int pHandle, p_data_disable_t* disable)
{
	for (broadcast_msg_t* bcPtr = cmInstance->broadcastMessages, *ptrEnd = (cmInstance->broadcastMessages + MAX_NUM_BCAST_MSGS); bcPtr < ptrEnd; bcPtr++)
	{
		if ((pHandle < 0 || pHandle == bcPtr->pHandle) && bcPtr->pkt.hdr.id == disable->id)
		{
			bcPtr->period = MSG_PERIOD_DISABLED;
		}
	}
	
	// Call global broadcast handler to disable message control
	if (cmInstance->cmMsgHandlerRmc)
	{
		p_data_get_t req;
		req.id = disable->id;
		req.size = 0;
		req.offset = 0;
		req.period = 0;
		cmInstance->cmMsgHandlerRmc(pHandle, &req);
	}
}

// Consolidate this with sendPacket() so that we break up packets into multiples that fit our buffer size.
int sendDataPacket(com_manager_t* cm, int port, packet_t* pkt)
{
	return is_comm_write_isb_precomp_to_port(cm->portWrite, port, &(cm->ports[port].comm), pkt);
}

void sendAck(com_manager_t* cmInstance, int pHandle, packet_t *pkt, uint8_t pTypeFlags)
{
	int ackSize;

	// Create and Send request packet
	p_ack_t ack = { 0 };
	ack.hdr.pktInfo = pkt->hdr.flags;
	ackSize = sizeof(p_ack_hdr_t);

	// Set ack body
	switch (pkt->hdr.flags & PKT_TYPE_MASK)
	{
	case PKT_TYPE_SET_DATA:
		ack.body.dataHdr = *((p_data_hdr_t*)(pkt->data.ptr));
		ackSize += sizeof(p_data_hdr_t);
		break;
	}

	comManagerSendInstance(cmInstance, pHandle, pTypeFlags, &ack, 0, sizeof(ack), 0);
}

int comManagerValidateBaudRate(unsigned int baudRate)
{
	// Valid baudrates for InertialSense hardware
	return validateBaudRate(baudRate);
}

