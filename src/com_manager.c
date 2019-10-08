/*
MIT LICENSE

Copyright (c) 2014-2019 Inertial Sense, Inc. - http://inertialsense.com

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
#define MAX_REQUEST_PERIOD_MS       100000          // (ms)
#define MSG_PERIOD_SEND_ONCE		-1
#define MSG_PERIOD_DISABLED			0

static com_manager_t g_cm;

int initComManagerInstanceInternal
(
	com_manager_t* cmInstance,
	int numHandles,
	int maxEnsuredPackets,
	int stepPeriodMilliseconds,
	int retryCount,
	pfnComManagerRead readFnc,
	pfnComManagerSend sendFnc,
	pfnComManagerSendBufferAvailableBytes txFreeFnc,
	pfnComManagerPostRead pstRxFnc,
	pfnComManagerPostAck pstAckFnc,
	pfnComManagerDisableBroadcasts disableBcastFnc,
	com_manager_buffers_t *buffers
);
int processAsciiRxPacket(com_manager_t* cmInstance, int pHandle, unsigned char* start, int count);
void parseAsciiPacket(com_manager_t* cmInstance, int pHandle, unsigned char* buf, int count);
int processBinaryRxPacket(com_manager_t* cmInstance, int pHandle, packet_t *pkt, unsigned char additionalDataAvailable);
void enableBroadcastMsg(com_manager_t* cmInstance, broadcast_msg_t *msg, int periodMultiple);
void disableBroadcastMsg(com_manager_t* cmInstance, broadcast_msg_t *msg);
void disableDidBroadcast(com_manager_t* cmInstance, int pHandle, p_data_disable_t *disable);
int sendPacket(com_manager_t* cmInstance, int pHandle, packet_t *dPkt, uint8_t additionalPktFlags);
int sendDataPacket(com_manager_t* cmInstance, int pHandle, pkt_info_t *msg);
void sendAck(com_manager_t* cmInstance, int pHandle, packet_t *pkt, unsigned char pid_ack);
int findAsciiMessage(const void * a, const void * b);

//  Packet processing
// com manager only...
int encodeBinaryPacket(com_manager_t* cmInstance, int pHandle, buffer_t *pkt, packet_t *dPkt, uint8_t additionalPktFlags);
int validateAsciiChecksum(com_manager_t* cmInstance, unsigned char* buf, int count);
int processAsciiPacket(com_manager_t* cmInstance, int pHandle, unsigned char* data, int dataLength);
int processBinaryPacket(com_manager_t* cmInstance, int pHandle, unsigned char* dataStart, unsigned char* data, int dataLength, unsigned char* additionalDataAvailable);
int processPassThroughBytes(com_manager_t* cmInstance, int pHandle, ring_buffer_t* ringBuffer);
int beginUbloxPacket(com_manager_t* cmInstance, int pHandle, ring_buffer_t* ringBuffer);
int beginRtcm3Packet(com_manager_t* cmInstance, int pHandle, ring_buffer_t* ringBuffer);
int asciiMessageCompare(const void* elem1, const void* elem2);

//  Packet Retry
void stepPacketRetry(com_manager_t* cmInstance);
packet_t* registerPacketRetry(com_manager_t* cmInstance, int pHandle, uint8_t pid, unsigned char data[], unsigned int dataSize);
void updatePacketRetryData(com_manager_t* cmInstance, packet_t *pkt);
void updatePacketRetryAck(com_manager_t* cmInstance, packet_t *pkt);

void stepComManagerSendMessages(void);
void stepComManagerSendMessagesInstance(CMHANDLE cmInstance);

CMHANDLE comManagerGetGlobal(void) { return &g_cm; }

int comManagerInit
(
	int numHandles,
	int maxEnsuredPackets,
	int stepPeriodMilliseconds,
	int retryCount,
	pfnComManagerRead readFnc,
	pfnComManagerSend sendFnc,
	pfnComManagerSendBufferAvailableBytes txFreeFnc,
	pfnComManagerPostRead pstRxFnc,
	pfnComManagerPostAck pstAckFnc,
	pfnComManagerDisableBroadcasts disableBcastFnc,
	com_manager_buffers_t *buffers
)
{
	return initComManagerInstanceInternal(
		&g_cm, 
		numHandles, 
		maxEnsuredPackets, 
		stepPeriodMilliseconds, 
		retryCount, 
		readFnc, 
		sendFnc, 
		txFreeFnc, 
		pstRxFnc, 
		pstAckFnc, 
		disableBcastFnc, 
		buffers);
}

int comManagerInitInstance
(
	CMHANDLE cmHandle,
	int numHandles,
	int maxEnsuredPackets,
	int stepPeriodMilliseconds,
	int retryCount,
	pfnComManagerRead readFnc,
	pfnComManagerSend sendFnc,
	pfnComManagerSendBufferAvailableBytes txFreeFnc,
	pfnComManagerPostRead pstRxFnc,
	pfnComManagerPostAck pstAckFnc,
	pfnComManagerDisableBroadcasts disableBcastFnc,
	com_manager_buffers_t *buffers
)
{
	int result = 0;

	com_manager_t* cmInstance = (com_manager_t*)cmHandle;
	if (cmInstance != 0)
	{
		memset(cmInstance, 0, sizeof(com_manager_t));
		result = initComManagerInstanceInternal(
			cmInstance, 
			numHandles, 
			maxEnsuredPackets, 
			stepPeriodMilliseconds, 
			retryCount, 
			readFnc, 
			sendFnc, 
			txFreeFnc, 
			pstRxFnc, 
			pstAckFnc, 
			disableBcastFnc,
			buffers);
	}
	return result;
}

int initComManagerInstanceInternal
(
	com_manager_t* cmInstance,
	int numHandles,
	int maxEnsuredPackets,
	int stepPeriodMilliseconds,
	int retryCount,
	pfnComManagerRead readFnc,
	pfnComManagerSend sendFnc,
	pfnComManagerSendBufferAvailableBytes txFreeFnc,
	pfnComManagerPostRead pstRxFnc,
	pfnComManagerPostAck pstAckFnc,
	pfnComManagerDisableBroadcasts disableBcastFnc,
	com_manager_buffers_t *buffers
)
{
	int32_t i;

	numHandles = _CLAMP(numHandles, 1, 1024);

	// assign new variables
	cmInstance->maxEnsuredPackets = maxEnsuredPackets;
	cmInstance->readCallback = readFnc;
	cmInstance->sendPacketCallback = sendFnc;
	cmInstance->txFreeCallback = txFreeFnc;
	cmInstance->pstRxFnc = pstRxFnc;
	cmInstance->pstAckFnc = pstAckFnc;
	cmInstance->disableBcastFnc = disableBcastFnc;
	cmInstance->numHandes = numHandles;
	cmInstance->stepPeriodMilliseconds = stepPeriodMilliseconds;
	cmInstance->ensureRetryCount = retryCount;

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
	
	// Buffer: ring buffers for serial reads / writes
	if (buffers->ringBuffer == NULL || buffers->ringBufferSize < COM_MANAGER_BUF_SIZE_RING_BUFFER(numHandles))
	{
		return -1;
	}
	cmInstance->ringBuffers = (ring_buffer_t*)buffers->ringBuffer;
	memset(cmInstance->ringBuffers, 0, buffers->ringBufferSize);

	// Buffer: ensured packets
	if (cmInstance->maxEnsuredPackets > 0)
	{
		if (buffers->ensuredPackets == NULL || buffers->ensuredPacketsSize < COM_MANAGER_BUF_SIZE_ENSURED_PKTS(cmInstance->maxEnsuredPackets))
		{
			return -1;
		}
		cmInstance->ensuredPackets = (ensured_pkt_t*)buffers->ensuredPackets;
		memset(cmInstance->ensuredPackets, 0, buffers->ensuredPacketsSize);
		for (i = 0; i < cmInstance->maxEnsuredPackets; i++)
		{
			cmInstance->ensuredPackets[i].counter = -2; // indicates no retries are enabled
			cmInstance->ensuredPackets[i].pkt.body.ptr = cmInstance->ensuredPackets[i].pktBody;
		}
	}

	// Buffer: status
	if (buffers->status == NULL || buffers->statusSize < COM_MANAGER_BUF_SIZE_STATUS(numHandles))
	{
		return -1;
	}
	cmInstance->status = (com_manager_status_t*)buffers->status;
	memset(cmInstance->status, 0, buffers->statusSize);

#if ENABLE_PACKET_CONTINUATION
		
	// Buffer: status
	if (buffers->packetContinuation == NULL || buffers->packetContinuationSize < COM_MANAGER_BUF_SIZE_PKT_CONTINUATION(numHandles))
	{
		return -1;
	}
	cmInstance->con = (p_data_t*)buffers->packetContinuation;
	memset(cmInstance->con, 0, buffers->packetContinuationSize);
	
#endif

	return 0;
}

int asciiMessageCompare(const void* elem1, const void* elem2)
{
	asciiMessageMap_t* e1 = (asciiMessageMap_t*)elem1;
	asciiMessageMap_t* e2 = (asciiMessageMap_t*)elem2;

	return memcmp(e1->messageId, e2->messageId, 4);
}

void comManagerRegisterASCII(asciiMessageMap_t* asciiMessages, int asciiMessagesCount, pfnComManagerAsciiMessageHandler msgFnc, pfnComManagerAsciiMessageHandler msgFncPost)
{
	comManagerRegisterASCIIInstance(&g_cm, asciiMessages, asciiMessagesCount, msgFnc, msgFncPost);
}

void comManagerRegisterASCIIInstance(CMHANDLE cmInstance_, asciiMessageMap_t* asciiMessages, int asciiMessagesCount, pfnComManagerAsciiMessageHandler msgFnc, pfnComManagerAsciiMessageHandler msgFncPost)
{
	com_manager_t* cmInstance = (com_manager_t*)cmInstance_;

	cmInstance->asciiMessages = asciiMessages;
	cmInstance->asciiMessagesCount = asciiMessagesCount;
	cmInstance->asciiMessageHandler = msgFnc;
	cmInstance->asciiMessageHandlerPost = msgFncPost;
	
	if (asciiMessagesCount > 1)
	{
		qsort(asciiMessages, sizeof(asciiMessages[0]), asciiMessagesCount, asciiMessageCompare);
	}
}

void comManagerRegister(uint32_t dataId, pfnComManagerPreSend txFnc, pfnComManagerPostRead pstRxFnc, const void* txDataPtr, void* rxDataPtr, int dataSize, uint8_t pktFlags)
{
	comManagerRegisterInstance(&g_cm, dataId, txFnc, pstRxFnc, txDataPtr, rxDataPtr, dataSize, pktFlags);
}

void comManagerRegisterInstance(CMHANDLE cmInstance_, uint32_t dataId, pfnComManagerPreSend txFnc, pfnComManagerPostRead pstRxFnc, const void* txDataPtr, void* rxDataPtr, int dataSize, uint8_t pktFlags)
{
	com_manager_t* cmInstance = (com_manager_t*)cmInstance_;

	// Validate ID and data pointer
	if (dataId >= DID_COUNT)
	{
		return;
	}

	// Function called to update struct before data is sent
	cmInstance->regData[dataId].preTxFnc = txFnc;

	// Function called after data is received and struct is updated
	cmInstance->regData[dataId].pstRxFnc = pstRxFnc;

	// Pointer to data struct for Tx
	cmInstance->regData[dataId].dataSet.txPtr = (unsigned char*)txDataPtr;

	// Pointer to data struct for Rx
	cmInstance->regData[dataId].dataSet.rxPtr = (unsigned char*)rxDataPtr;

	// Size of data struct
	cmInstance->regData[dataId].dataSet.size = dataSize;
	
	// Packet flags
	cmInstance->regData[dataId].pktFlags = pktFlags;
}

void comManagerStep(void)
{
	comManagerStepInstance(&g_cm);
}

void comManagerStepInstance(CMHANDLE cmInstance_)
{
	com_manager_t* cmInstance = (com_manager_t*)cmInstance_;
	uint8_t c, additionalDataAvailable, canReadAgain;
	int32_t n, pHandle, freeByteCount;
	ring_buffer_t* ringBuffer;

	if (cmInstance->readCallback)
	{
		for (pHandle = 0; pHandle < cmInstance->numHandes; pHandle++)
		{
			ringBuffer = cmInstance->ringBuffers + pHandle;
			canReadAgain = 1;

			readAgain:

			freeByteCount = (RING_BUFFER_SIZE - ringBuffer->endIndex);

			// if we are out of free space, we need to either move bytes over or start over
			if (freeByteCount == 0)
			{
				if (ringBuffer->startIndex < RING_BUFFER_FLUSH_THRESHOLD)
				{
					// we will be hung unless we flush the ring buffer, we have to drop bytes in this case and the caller
					//  will need to resend the data
					ringBuffer->startIndex = 0;
					ringBuffer->endIndex = 0;
					ringBuffer->scanIndex = 0;
				}
				else
				{
					// shift over the remaining data in the hopes that we will get a valid packet by appending the next read call
					ringBuffer->endIndex -= ringBuffer->startIndex;
					ringBuffer->scanIndex -= ringBuffer->startIndex;
					memmove(ringBuffer->buf, ringBuffer->buf + ringBuffer->startIndex, ringBuffer->endIndex);
					ringBuffer->startIndex = 0;
				}

				// re-calculate free byte count
				freeByteCount = (RING_BUFFER_SIZE - ringBuffer->endIndex);
			}

			if ((n = cmInstance->readCallback(cmInstance, pHandle, ringBuffer->buf + ringBuffer->endIndex, freeByteCount)) > 0)
			{
				additionalDataAvailable = 0;
				cmInstance->status[pHandle].readCounter += n;
				ringBuffer->endIndex += n;

				while (ringBuffer->scanIndex < ringBuffer->endIndex)
				{
					// check for pass through bytes
					if (cmInstance->status[pHandle].passThroughBytes > 0)
					{
						processPassThroughBytes(cmInstance, pHandle, ringBuffer);
						continue;
					}

					c = ringBuffer->buf[ringBuffer->scanIndex++];

					switch (c)
					{
						case PSC_ASCII_START_BYTE:
						{
							cmInstance->status[pHandle].startByte = PSC_ASCII_START_BYTE;
							ringBuffer->startIndex = ringBuffer->scanIndex - 1;
						} break;

						case PSC_START_BYTE:
						{
							cmInstance->status[pHandle].startByte = PSC_START_BYTE;
							ringBuffer->startIndex = ringBuffer->scanIndex - 1;
						} break;

						case PSC_ASCII_END_BYTE:
						{
							if (cmInstance->status[pHandle].startByte == PSC_ASCII_START_BYTE)
							{
								processAsciiPacket(cmInstance, pHandle, ringBuffer->buf + ringBuffer->startIndex, ringBuffer->scanIndex - ringBuffer->startIndex);
							}
							ringBuffer->startIndex = ringBuffer->scanIndex;
							cmInstance->status[pHandle].startByte = 0;
						} break;

						case PSC_END_BYTE:
						{
							if (cmInstance->status[pHandle].startByte == PSC_START_BYTE)
							{
								processBinaryPacket(cmInstance, pHandle, ringBuffer->buf, ringBuffer->buf + ringBuffer->startIndex, ringBuffer->scanIndex - ringBuffer->startIndex, &additionalDataAvailable);
							}
							ringBuffer->startIndex = ringBuffer->scanIndex;
							cmInstance->status[pHandle].startByte = 0;
						} break;

						case UBLOX_START_BYTE1:
						{
							if (beginUbloxPacket(cmInstance, pHandle, ringBuffer))
							{
								return;
							}
						} break;

						case RTCM3_START_BYTE:
						{
							if (beginRtcm3Packet(cmInstance, pHandle, ringBuffer))
							{
								return;
							}
						} break;
					}
				}

				if (additionalDataAvailable && canReadAgain)
				{
					canReadAgain = 0;
					goto readAgain;
				}
			}
			
			if ((cmInstance->status[pHandle].flags & CM_PKT_FLAGS_RX_VALID_DATA) && cmInstance->status[pHandle].readCounter > 128)
			{
				// communication problem, clear communication received bit
				cmInstance->status[pHandle].flags &= (~CM_PKT_FLAGS_RX_VALID_DATA);
			}
		}
	}

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
		if (cmInstance->txFreeCallback && (bcPtr->pkt.txData.size > (uint32_t)cmInstance->txFreeCallback(cmInstance, bcPtr->pHandle)))
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
				if (cmInstance->regData[bcPtr->dataHdr.id].preTxFnc)
				{
					cmInstance->regData[bcPtr->dataHdr.id].preTxFnc(cmInstance, bcPtr->pHandle);
				}
				sendDataPacket(cmInstance, bcPtr->pHandle, &(bcPtr->pkt));
			}
		}
	}

	// Resend data (if necessary)
	stepPacketRetry(cmInstance);
}

void comManagerSetPassThrough(pfnComManagerPassThrough handler)
{
	comManagerSetPassThroughInstance(&g_cm, handler);
}

void comManagerSetPassThroughInstance(CMHANDLE cmInstance, pfnComManagerPassThrough handler)
{
	if (cmInstance != 0)
	{
		((com_manager_t*)cmInstance)->passThroughHandler = handler;
	}
}

void comManagerSetRmcCallback(pfnComManagerAsapMsg handler)
{
	comManagerSetRmcCallbackInstance(&g_cm, handler);
}

void comManagerSetRmcCallbackInstance(CMHANDLE cmInstance, pfnComManagerAsapMsg handler)
{
	if (cmInstance != 0)
	{
		((com_manager_t*)cmInstance)->rmcHandler = handler;
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

com_manager_status_t* comManagerGetStatus(int pHandle)
{
	return comManagerGetStatusInstance(&g_cm, pHandle);
}

com_manager_status_t* comManagerGetStatusInstance(CMHANDLE cmInstance, int pHandle)
{
	return &(((com_manager_t*)cmInstance)->status[pHandle]);
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
void comManagerGetData(int pHandle, uint32_t dataId, int offset, int size, int periodMultiple)
{
	comManagerGetDataInstance(&g_cm, pHandle, dataId, offset, size, periodMultiple);
}

void comManagerGetDataInstance(CMHANDLE cmInstance, int pHandle, uint32_t dataId, int offset, int size, int periodMultiple)
{
	p_data_get_t request;
	bufPtr_t data;

	// Create and Send request packet
	request.id = dataId;
	request.offset = offset;
	request.size = size;
	request.bc_period_multiple = periodMultiple;

	data.ptr = (uint8_t*)&request;
	data.size = sizeof(request);
	comManagerSendInstance(cmInstance, pHandle, PID_GET_DATA, 0, &data, 0);

	// comManagerSendEnsured(pHandle, PID_GET_DATA, (unsigned char*)&request, sizeof(request));
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

	comManagerSendDataInstance(cmInstance, pHandle, DID_RMC, &rmc, sizeof(rmc_t), 0);
}

int comManagerSendData(int pHandle, uint32_t dataId, void *dataPtr, int dataSize, int dataOffset)
{
	return comManagerSendDataInstance(&g_cm, pHandle, dataId, dataPtr, dataSize, dataOffset);
}

int comManagerSendDataInstance(CMHANDLE cmInstance, int pHandle, uint32_t dataId, void* dataPtr, int dataSize, int dataOffset)
{
	p_data_hdr_t hdr;
	bufPtr_t bodyHdr, data;

	// Data Header
	hdr.id = dataId;
	hdr.size = dataSize;
	hdr.offset = dataOffset;

	// Packet Body
	bodyHdr.ptr = (uint8_t*)&hdr;
	bodyHdr.size = sizeof(hdr);
	data.ptr = (uint8_t*)dataPtr;
	data.size = dataSize;

	return comManagerSendInstance(cmInstance, pHandle, PID_SET_DATA, &bodyHdr, &data, 0);
}

int comManagerSendDataNoAck(int pHandle, uint32_t dataId, void *dataPtr, int dataSize, int dataOffset)
{
	return comManagerSendDataNoAckInstance(&g_cm, pHandle, dataId, dataPtr, dataSize, dataOffset);
}

int comManagerSendDataNoAckInstance(CMHANDLE cmInstance, int pHandle, uint32_t dataId, void* dataPtr, int dataSize, int dataOffset)
{
	p_data_hdr_t hdr;
	bufPtr_t bodyHdr, data;

	// Data Header
	hdr.id = dataId;
	hdr.size = dataSize;
	hdr.offset = dataOffset;

	// Packet Body
	bodyHdr.ptr = (uint8_t*)&hdr;
	bodyHdr.size = sizeof(hdr);
	data.ptr = (uint8_t*)dataPtr;
	data.size = dataSize;

	return comManagerSendInstance((com_manager_t*)cmInstance, pHandle, PID_DATA, &bodyHdr, &data, 0);
}

int comManagerSendRawData(int pHandle, uint32_t dataId, void *dataPtr, int dataSize, int dataOffset)
{
	return comManagerSendRawDataInstance(&g_cm, pHandle, dataId, dataPtr, dataSize, dataOffset);
}

int comManagerSendRawDataInstance(CMHANDLE cmInstance, int pHandle, uint32_t dataId, void* dataPtr, int dataSize, int dataOffset)
{
	p_data_hdr_t hdr;
	bufPtr_t bodyHdr, data;

	// Data Header
	hdr.id = dataId;
	hdr.size = dataSize;
	hdr.offset = dataOffset;

	// Packet Body
	bodyHdr.ptr = (uint8_t*)&hdr;
	bodyHdr.size = sizeof(hdr);
	data.ptr = (uint8_t*)dataPtr;
	data.size = dataSize;

	return comManagerSendInstance((com_manager_t*)cmInstance, pHandle, PID_SET_DATA, &bodyHdr, &data, CM_PKT_FLAGS_RAW_DATA_NO_SWAP);
}

int comManagerDisableData(int pHandle, uint32_t dataId)
{
	return comManagerDisableDataInstance(&g_cm, pHandle, dataId);
}

int comManagerDisableDataInstance(CMHANDLE cmInstance, int pHandle, uint32_t dataId)
{
	bufPtr_t data;
	data.ptr  = (uint8_t*)&dataId;
	data.size = 4;

	return comManagerSendInstance(cmInstance, pHandle, PID_STOP_DID_BROADCAST, 0, &data, 0);
}

int comManagerSend(int pHandle, uint8_t pktInfo, bufPtr_t *bodyHdr, bufPtr_t *txData, uint8_t pFlags)
{
	return comManagerSendInstance(&g_cm, pHandle, pktInfo, bodyHdr, txData, pFlags);
}

int comManagerSendInstance(CMHANDLE cmInstance, int pHandle, uint8_t pktInfo, bufPtr_t* bodyHdr, bufPtr_t* txData, uint8_t pktFlags)
{
	pkt_info_t pkt;
	memset(&pkt, 0, sizeof(pkt_info_t));

	// Create Packet String (start to end byte)
	pkt.hdr.startByte = PSC_START_BYTE;
	pkt.hdr.pid = pktInfo;
	pkt.hdr.flags = pktFlags;

	if (bodyHdr)
	{
		pkt.bodyHdr = *bodyHdr;
	}
	if (txData)
	{
		pkt.txData = *txData;
	}

	return sendDataPacket(cmInstance, pHandle, &pkt);
}

int comManagerSendEnsured(int pHandle, uint8_t pktInfo, unsigned char* data, unsigned int dataSize)
{
	return comManagerSendEnsuredInstance(&g_cm, pHandle, pktInfo, data, dataSize);
}

int comManagerSendEnsuredInstance(CMHANDLE cmInstance, int pHandle, uint8_t pktInfo, unsigned char *data, unsigned int dataSize)
{
	packet_t *pkt;

	// Change retry "Ensured" packets to so that we encode packets first (including pkt counter)
	// and then ensure they are delivered.  Include packet checksum in ACK/NACK to validate delivery.
	// Then, if all the ensured slots are occupied because of bad comm, either allow
	// to clear ensured packets or just block until they are delivered.  We must
	// ensure NACKs are used to clear blocking ensured packets.

	// Create Packet String (start to end byte)
	if ((pkt = registerPacketRetry((com_manager_t*)cmInstance, pHandle, pktInfo, data, dataSize)) == 0)
	{
		return -1;
	}

	return sendPacket((com_manager_t*)cmInstance, pHandle, pkt, 0);
}

int findAsciiMessage(const void * a, const void * b)
{
	unsigned char* a1 = (unsigned char*)a;
	asciiMessageMap_t* a2 = (asciiMessageMap_t*)b;

	return memcmp(a1, a2->messageId, 4);
}


int validateAsciiChecksum(com_manager_t* cmInstance, unsigned char* buf, int count)
{
	int checkSum = 0;
	unsigned char c;
	unsigned char* end = buf + count;

	// Suppress compiler warnings
	(void)cmInstance;

	// calculate and validate the checksum, skipping the initial $ char
	while (buf != end)
	{
		c = *(++buf);
		if (c == '*')
		{
			// there must be at least four more chars - the checksum and \r\n to proceed
			if (buf + 4 >= end)
			{
				return 0;
			}
			char tmp[3];
			tmp[0] = *(++buf);
			tmp[1] = *(++buf);
			tmp[2] = '\0';
			int actualCheckSum = strtol(tmp, 0, 16);
			return (checkSum == actualCheckSum);
		}
		checkSum ^= c;
	}

	return 0;
}

void parseAsciiPacket(com_manager_t* cmInstance, int pHandle, unsigned char* buf, int count)
{
	unsigned char c;
	unsigned char* messageId;
	unsigned char* messageData = 0;
	unsigned char* end = buf + count;
	unsigned char* ptr = buf;
	int fieldIndex = 0;
	int length;
	asciiMessageMap_t* foundMap = 0;
	cmInstance->status[pHandle].flags |= CM_PKT_FLAGS_RX_VALID_DATA; // communication received
	cmInstance->status[pHandle].readCounter = 0;

	// Packet read success
	cmInstance->status[pHandle].rxPktCount++;

	// message id is right after the $ char
	messageId = ++ptr;

	// in order to parse the fields from the ascii message there must be:
	//  Global handler didn't parse any data AND
	//  have at least one ascii message defined AND
	//  have a map for this message id
	if 
	(
		(cmInstance->asciiMessageHandler == 0 || cmInstance->asciiMessageHandler(cmInstance, pHandle, messageId, buf, count) == 0) &&
		(cmInstance->asciiMessages != 0 && cmInstance->asciiMessagesCount > 0) &&
		((foundMap = bsearch(messageId, cmInstance->asciiMessages, cmInstance->asciiMessagesCount, sizeof(cmInstance->asciiMessages[0]), findAsciiMessage)) != 0)
	)
	{
		while (ptr < end)
		{
			c = *ptr;
			if (c == ',' || c == '*')
			{
				// check if we have the first piece of data yet and have not run out of fields
				if (fieldIndex < foundMap->fieldCount && messageData != 0)
				{
					// if we have a non-empty message, parse it out
					if ((length = (int)(ptr - messageData)) > 0)
					{
						*ptr = '\0';
						uint16_t fieldAndOffset = foundMap->fieldsAndOffsets[fieldIndex];
						asciiDataType type = (fieldAndOffset & 0x00FF);
						uint8_t offset = (fieldAndOffset & 0xFF00) >> 8;
						switch (type)
						{
							case asciiTypeInt:
							{
								const int dataValue = strtol((char*)messageData, 0, 10);
								memcpy(foundMap->ptr + offset, &dataValue, sizeof(dataValue));
							} break;
							case asciiTypeUInt:
							{
								const unsigned int dataValue = strtoul((char*)messageData, 0, 10);
								memcpy(foundMap->ptr + offset, &dataValue, sizeof(dataValue));
							} break;
							case asciiTypeFloat:
							{
								const float dataValue = PARSE_FLOAT((char*)messageData);
								memcpy(foundMap->ptr + offset, &dataValue, sizeof(dataValue));
							} break;
							case asciiTypeDouble:
							{
								const double dataValue = PARSE_DOUBLE((char*)messageData);
								memcpy(foundMap->ptr + offset, &dataValue, sizeof(dataValue));
							} break;
						}
						*ptr = c;
					}
					fieldIndex++;
				}

				// message data is at next char
				messageData = ++ptr;
			}
			else
			{
				ptr++;
			}
		}
	}
				
	// send post ascii parse message if set
	if (cmInstance->asciiMessageHandlerPost != 0)
	{
		cmInstance->asciiMessageHandlerPost(cmInstance, pHandle, messageId, buf, count);
	}
}


// Return value: 0 = success, -1 = error.
int processAsciiRxPacket(com_manager_t* cmInstance, int pHandle, unsigned char* buf, int count)
{
	if (count < 10 || count > 512 || validateAsciiChecksum(cmInstance, buf, count) == 0)
	{
		return -1;
	}
	else if ((cmInstance->asciiMessages != 0 && cmInstance->asciiMessagesCount != 0) || (cmInstance->asciiMessageHandler != 0))
	{
		// parse the packet if we have any registered handlers
		parseAsciiPacket(cmInstance, pHandle, buf, count);
	}
	return 0;
}

/**
*   @brief Process binary packet content:
*
*	@return 0 on success.  -1 on failure.
*/
int processBinaryRxPacket(com_manager_t* cmInstance, int pHandle, packet_t *pkt, unsigned char additionalDataAvailable)
{
	p_data_t			*data;
	p_data_hdr_t		*dataHdr;
// 	uint8_t				*dataBuf;
	registered_data_t	*regd;
	uint8_t		pid = (uint8_t)(pkt->hdr.pid);

	cmInstance->status[pHandle].flags |= CM_PKT_FLAGS_RX_VALID_DATA; // communication received
	cmInstance->status[pHandle].readCounter = 0;

	// Packet read success
	cmInstance->status[pHandle].rxPktCount++;

	switch (pid)
	{
	default:    // Data ID Unknown
		return -1;

	case PID_SET_DATA:
	case PID_DATA:
		data = (p_data_t*)(pkt->body.ptr);
		dataHdr = &(data->hdr);
// 		dataBuf = data->buf;

		// Validate Data
		if (dataHdr->id >= DID_COUNT)
		{
			return -1;
		}

		regd = &(cmInstance->regData[dataHdr->id]);

		// Validate and constrain Rx data size to fit within local data struct
		if (regd->dataSet.size && (dataHdr->offset + dataHdr->size) > regd->dataSet.size)
		{
			// trim the size down so it fits
			int size = (int)(regd->dataSet.size - dataHdr->offset);
			if (size < 4)
			{
				// we are completely out of bounds, we cannot process this message at all
				// the minimum data struct size is 4 bytes
				return -1;
			}

			// Update Rx data size
			dataHdr->size = _MIN(dataHdr->size, (uint8_t)size);
		}

#if ENABLE_PACKET_CONTINUATION

		// Consolidate datasets that were broken-up across multiple packets
		p_data_t* con = &cmInstance->con[pHandle];
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
		
		(void)additionalDataAvailable;

#endif

		// Write to data structure if it was registered
		if (regd->dataSet.rxPtr)
		{
			copyDataPToStructP(regd->dataSet.rxPtr, data, regd->dataSet.size);
		}

		// Call data specific callback after data has been written to
		if (regd->pstRxFnc)
		{
			regd->pstRxFnc(cmInstance, pHandle, data);
		}

		// Call general/global callback
		if (cmInstance->pstRxFnc)
		{
			cmInstance->pstRxFnc(cmInstance, pHandle, data);
		}

		// Remove retry from linked list if necessary
		updatePacketRetryData(cmInstance, pkt);

#if ENABLE_PACKET_CONTINUATION

		// Clear dataset consolidation
		con->hdr.id = con->hdr.size = con->hdr.offset = 0;

#endif

		// Replay w/ ACK for PID_SET_DATA
		if (pid == PID_SET_DATA)
		{
			sendAck(cmInstance, pHandle, pkt, PID_ACK);
		}
		break;

	case PID_GET_DATA:
		if (comManagerGetDataRequestInstance(cmInstance, pHandle, (p_data_get_t *)(pkt->body.ptr)))
		{
			sendAck(cmInstance, pHandle, pkt, PID_NACK);
		}
		break;

	case PID_STOP_BROADCASTS_ALL_PORTS:
		comManagerDisableBroadcastsInstance(cmInstance, -1);

		// Call disable broadcasts callback if exists
		if (cmInstance->disableBcastFnc)
		{
			cmInstance->disableBcastFnc(cmInstance, -1);
		}
		sendAck(cmInstance, pHandle, pkt, PID_ACK);
		break;

	case PID_STOP_BROADCASTS_CURRENT_PORT:
		comManagerDisableBroadcastsInstance(cmInstance, pHandle);

		// Call disable broadcasts callback if exists
		if (cmInstance->disableBcastFnc)
		{
			cmInstance->disableBcastFnc(cmInstance, pHandle);
		}
		sendAck(cmInstance, pHandle, pkt, PID_ACK);
		break;

	case PID_STOP_DID_BROADCAST:
		disableDidBroadcast(cmInstance, pHandle, (p_data_disable_t *)(pkt->body.ptr));
		break;

	case PID_NACK:
	case PID_ACK:
		// Remove retry from linked list if necessary
		updatePacketRetryAck(cmInstance, pkt);

		// Call general ack callback
		if (cmInstance->pstAckFnc)
		{
			cmInstance->pstAckFnc(cmInstance, pHandle, (p_ack_t*)(pkt->body.ptr), pid);
		}
		break;
	}

	return 0;
}

bufTxRxPtr_t* comManagerGetRegisteredDataInfo(uint32_t dataId)
{
	return comManagerGetRegisteredDataInfoInstance(&g_cm, dataId);
}

bufTxRxPtr_t* comManagerGetRegisteredDataInfoInstance(CMHANDLE _cmInstance, uint32_t dataId)
{
	if (dataId >= DID_COUNT)
	{
		return 0;
	}

	com_manager_t* cmInstance = (com_manager_t*)_cmInstance;
	return &cmInstance->regData[dataId].dataSet;
}

// 0 on success. -1 on failure.
int comManagerGetDataRequest(int pHandle, p_data_get_t* req)
{
	return comManagerGetDataRequestInstance(&g_cm, pHandle, req);
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
	else if (cmInstance->rmcHandler && (cmInstance->rmcHandler(cmInstance, pHandle, req) == 0))
	{
		// Don't allow comManager broadcasts for messages handled by RealtimeMessageController. 
		return 0;
	}
	// if size is 0 and offset is 0, set size to full data struct size
	else if (req->size == 0 && req->offset == 0 && req->id < _ARRAY_ELEMENT_COUNT(cmInstance->regData))
	{
		req->size = cmInstance->regData[req->id].dataSet.size;
	}
	
	// Copy reference to source data
	bufTxRxPtr_t* dataSetPtr = &cmInstance->regData[req->id].dataSet;

	// Abort if no data pointer is registered or offset + size is out of bounds
	if (dataSetPtr->txPtr == 0 || dataSetPtr->size == 0)
	{
		return -1;
	}
	else if (req->offset + req->size > dataSetPtr->size)
	{
		req->offset = 0;
		req->size = dataSetPtr->size;
	}

	// Search for matching message (i.e. matches pHandle, id, size, and offset)...
	for (broadcast_msg_t* bcPtr = cmInstance->broadcastMessages, *ptrEnd = (cmInstance->broadcastMessages + MAX_NUM_BCAST_MSGS); bcPtr < ptrEnd; bcPtr++)
	{
		if (bcPtr->pHandle == pHandle && bcPtr->dataHdr.id == req->id && bcPtr->dataHdr.size == req->size && bcPtr->dataHdr.offset == req->offset)
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

	// Port handle
	msg->pHandle = pHandle;

	// Packet parameters
	msg->pkt.hdr.startByte = PSC_START_BYTE;
	msg->pkt.hdr.pid = PID_DATA;

	// Data Header
	msg->dataHdr.id = req->id;
	msg->dataHdr.size = req->size;
	msg->dataHdr.offset = req->offset;
	msg->pkt.hdr.flags = cmInstance->regData[msg->dataHdr.id].pktFlags;
	msg->pkt.bodyHdr.ptr = (uint8_t *)&msg->dataHdr;
	msg->pkt.bodyHdr.size = sizeof(msg->dataHdr);
	msg->pkt.txData.size = req->size;
	msg->pkt.txData.ptr = cmInstance->regData[req->id].dataSet.txPtr + req->offset;

	// Prep data if callback exists
	if (cmInstance->regData[msg->dataHdr.id].preTxFnc)
	{
		cmInstance->regData[msg->dataHdr.id].preTxFnc(cmInstance, pHandle);
	}
	
	// Constrain request broadcast period if necessary
	if (req->bc_period_multiple != 0)
	{
		_LIMIT2(req->bc_period_multiple, MIN_REQUEST_PERIOD_MS, MAX_REQUEST_PERIOD_MS);
	}

	// Send data
	if (req->bc_period_multiple > 0)
	{
		// ***  Request Broadcast  ***
		// Send data immediately if possible
		if (cmInstance->txFreeCallback == 0 || msg->pkt.txData.size <= (uint32_t)cmInstance->txFreeCallback(cmInstance, pHandle))
		{
			sendDataPacket(cmInstance, pHandle, &(msg->pkt));
		}

		// Enable broadcast message
		enableBroadcastMsg(cmInstance, msg, req->bc_period_multiple);
	}
	else
	{
		// ***  Request Single  ***
		// Send data immediately if possible
		if (cmInstance->txFreeCallback == 0 || msg->pkt.txData.size <= (uint32_t)cmInstance->txFreeCallback(cmInstance, pHandle))
		{
			sendDataPacket(cmInstance, pHandle, &(msg->pkt));
			disableBroadcastMsg(cmInstance, msg);
		}
		else
		{
			// Won't fit in queue, so send it later
			enableBroadcastMsg(cmInstance, msg, req->bc_period_multiple);
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
		if ((pHandle < 0 || pHandle == bcPtr->pHandle) && bcPtr->dataHdr.id == disable->id)
		{
			bcPtr->period = MSG_PERIOD_DISABLED;
		}
	}
	
	// Call global broadcast handler to disable message control
	if (cmInstance->rmcHandler)
	{
		p_data_get_t req;
		req.id = disable->id;
		req.size = 0;
		req.offset = 0;
		req.bc_period_multiple = 0;
		cmInstance->rmcHandler(cmInstance, pHandle, &req);
	}
}

/**
*   @brief Encode and send out serial port the referenced packet structure.
*
*	@param[in/out] dPkt Packet structure containing packet info.
*
*	@return 0 on success.  -1 on failure.
*/
int sendPacket(com_manager_t* cmInstance, int pHandle, packet_t *dPkt, uint8_t additionalPktFlags)
{
	buffer_t buffer;

	if (encodeBinaryPacket(cmInstance, pHandle, &buffer, dPkt, additionalPktFlags))
	{
		return -1;
	}

	// Send Packet
	else if (cmInstance->sendPacketCallback)
	{
		cmInstance->sendPacketCallback(cmInstance, pHandle, &buffer);
	}
	cmInstance->status[pHandle].txPktCount++;

	return 0;
}

// Consolidate this with sendPacket() so that we break up packets into multiples that fit our buffer size.
int sendDataPacket(com_manager_t* cmInstance, int pHandle, pkt_info_t* msg)
{
	pfnComManagerSend sendCallback = cmInstance->sendPacketCallback;
	if (sendCallback == 0)
	{
		return -1;
	}

	buffer_t bufToSend;
	packet_t pkt;
	pkt.hdr = msg->hdr;

	switch (pkt.hdr.pid)
	{
		// Large data support - breaks data up into separate packets for Tx
		case PID_DATA:
		case PID_SET_DATA:
		{
			// Setup packet and encoding state
			buffer_t bufToEncode;
			p_data_hdr_t hdr = *(p_data_hdr_t*)msg->bodyHdr.ptr;
			p_data_hdr_t* hdrToSend = (p_data_hdr_t*)bufToEncode.buf;
			uint32_t size = hdr.size;
			uint32_t offset = 0;
			uint32_t id = hdr.id;
			pkt.body.ptr = bufToEncode.buf;

#if ENABLE_PACKET_CONTINUATION

			while (size > 0)
			{
				
#endif
				
				// Assign data header values
				hdrToSend->size = _MIN(size, MAX_P_DATA_BODY_SIZE);
				hdrToSend->offset = hdr.offset + offset;
				hdrToSend->id = id;

				// copy the data to send to bufToEncode, skipping the data header - since we had to create that data header, we now have to append the actual data
				memcpy(bufToEncode.buf + sizeof(p_data_hdr_t), msg->txData.ptr + offset, hdrToSend->size);
				
				// reduce size by the amount sent - if packet continuation is off, this must become 0 otherwise we fail
				size -= hdrToSend->size;
				
#if ENABLE_PACKET_CONTINUATION

				// increment offset for the next packet
				offset += hdrToSend->size;
				
#else

				if (size > 0)
				{
					// data was too large to fit in one packet, fail
					return -1;
				}
				
#endif

				// Set data body size
				pkt.body.size = sizeof(p_data_hdr_t) + hdrToSend->size;

				// Encode the packet, handling special characters, etc.
				if (encodeBinaryPacket(cmInstance, pHandle, &bufToSend, &pkt, CM_PKT_FLAGS_MORE_DATA_AVAILABLE * (size != 0)))
				{
					return -1;
				}

				// Send the packet using the specified callback
				sendCallback(cmInstance, pHandle, &bufToSend);
				cmInstance->status[pHandle].txPktCount++;
				
#if ENABLE_PACKET_CONTINUATION

			}
			
#endif

		} break;

		// Single packet commands/data sets. No data header, just body.
		default:
		{
			// Assign packet pointer and encode data as is
			pkt.body = msg->txData;
			if (encodeBinaryPacket(cmInstance, pHandle, &bufToSend, &pkt, 0))
			{
				return -1;
			}

			// Send the packet using the specified callback
			sendCallback(cmInstance, pHandle, &bufToSend);
			cmInstance->status[pHandle].txPktCount++;
		} break;
	}

	return 0;
}

void sendAck(com_manager_t* cmInstance, int pHandle, packet_t *pkt, unsigned char pid_ack)
{
	int ackSize;
	bufPtr_t data;

	// Create and Send request packet
	p_ack_t ack;
	ack.hdr.pktInfo = pkt->hdr.pid;
	ack.hdr.pktCounter = pkt->hdr.counter;
	ackSize = sizeof(p_ack_hdr_t);

	// Set ack body
	switch (pkt->hdr.pid)
	{
		case PID_SET_DATA:
		memcpy(&(ack.buf), (p_data_hdr_t*)(pkt->body.ptr), sizeof(p_data_hdr_t));
		ackSize += sizeof(p_data_hdr_t);
		break;
	}

	data.ptr = (unsigned char*)&ack;
	data.size = ackSize;

	comManagerSendInstance(cmInstance, pHandle, (uint8_t)pid_ack, 0, &data, 0);
}

//////////////////////////////////////////////////////////////////////////
//  Packet Composition
//////////////////////////////////////////////////////////////////////////
/**
*  @brief Adds data to a packet: adds start, info, data length, data, checksum, and stop bytes.
*  All data is communicated in the endianess of the sender, each packet has a bit that determines big or little endian.
*  Process for Creating Tx Packet:
*  1.) Add to packet
*      - pkt start byte
*      - pkt ID
*      - pkt counter
*      - pkt flags
*      - data length
*      - data ID
*      - data start...
*      - ...data end
*      - pkt reserved
*      - computed cksum (2 bytes)
*      - pkt end byte
*  2.) Tx encode extraneous special characters to remove them from packet
*
*	@return 0 on success, -1 on failure.
*/
int encodeBinaryPacket(com_manager_t* cmInstance, int pHandle, buffer_t *pkt, packet_t *dPkt, uint8_t additionalPktFlags)
{
	void* srcBuffer = dPkt->body.ptr;
	int srcBufferLength = dPkt->body.size;
	void* encodedPacket = pkt->buf;
	int encodedPacketLength = PKT_BUF_SIZE - 1;
	packet_hdr_t* hdr = &dPkt->hdr;
	hdr->counter = cmInstance->pktCounter++;
	pkt->size = is_encode_binary_packet(srcBuffer, srcBufferLength, hdr, additionalPktFlags | cmInstance->status[pHandle].flags, encodedPacket, encodedPacketLength);
	return (-1 * ((int)pkt->size < 8));
}

int processAsciiPacket(com_manager_t* cmInstance, int pHandle, unsigned char* data, int dataLength)
{
	if (processAsciiRxPacket(cmInstance, pHandle, data, dataLength))
	{
		// Error parsing packet
		cmInstance->status[pHandle].readCounter += 32;
		cmInstance->status[pHandle].rxError = (uint32_t)-1;
		cmInstance->status[pHandle].communicationErrorCount++;
		return 1;
	}
	return 0;
}

int processBinaryPacket(com_manager_t* cmInstance, int pHandle, unsigned char* dataStart, unsigned char* data, int dataLength, unsigned char* additionalDataAvailable)
{
	packet_t pkt;
	pkt.body.ptr = dataStart;

	// Found Packet -> Now process packet
	if (!is_decode_binary_packet(&pkt, data, dataLength))
	{
		// bit index 2 is whether another packet is available that is related to this packet
		*additionalDataAvailable = pkt.hdr.flags & CM_PKT_FLAGS_MORE_DATA_AVAILABLE;

		if (!processBinaryRxPacket(cmInstance, pHandle, &pkt, *additionalDataAvailable))
		{
			return 0;
		}
	}

	// Error parsing packet
	cmInstance->status[pHandle].readCounter += 32;
	cmInstance->status[pHandle].rxError = (uint32_t)-1;
	cmInstance->status[pHandle].communicationErrorCount++;
	cmInstance->status[pHandle].startByte = 0;

	return 1;
}

int processPassThroughBytes(com_manager_t* cmInstance, int pHandle, ring_buffer_t* ringBuffer)
{
	(void)pHandle;

	uint8_t startByte = cmInstance->status[pHandle].startByte;
	uint32_t passThroughIndex = ringBuffer->startIndex;
	uint32_t byteCount = _MIN(cmInstance->status[pHandle].passThroughBytes, ringBuffer->endIndex - ringBuffer->startIndex);
	cmInstance->status[pHandle].passThroughBytes -= byteCount;
	ringBuffer->startIndex += byteCount;
	ringBuffer->scanIndex = ringBuffer->startIndex;

	// clear the start byte once all pass through bytes are sent
	if (cmInstance->status[pHandle].passThroughBytes <= 0)
	{
		cmInstance->status[pHandle].startByte = 0;
	}

	switch (startByte)
	{
		case UBLOX_START_BYTE1:
		case RTCM3_START_BYTE:
		{
			if (cmInstance->passThroughHandler != 0)
			{
				cmInstance->passThroughHandler(cmInstance, (com_manager_pass_through_t)startByte, pHandle, ringBuffer->buf + passThroughIndex, byteCount);
			}
			return 0;
		} break;
	}

	return 1;
}

int beginUbloxPacket(com_manager_t* cmInstance, int pHandle, ring_buffer_t* ringBuffer)
{
	uint32_t count = ringBuffer->endIndex - (--ringBuffer->scanIndex);
	if (count < UBLOX_HEADER_SIZE)
	{
		// not enough data to read a ublox packet header, wait for more data to arrive
		return 1; // 1 means more data needed
	}

	// get the header starting at the ublox start byte
	const unsigned char* header = (ringBuffer->buf + ringBuffer->scanIndex);
	int headerLength = ((int)header[4] | (int)header[5] << 8);

	// if the second preamble byte is wrong or the length is greater than 2048, probably not a ublox packet
	if (header[1] != UBLOX_START_BYTE2 || headerLength > 2048)
	{
		// skip past the ublox start byte to avoid an infinite loop
		ringBuffer->scanIndex++;
		cmInstance->status[pHandle].startByte = 0;
		return 0; // 0 means keep reading as normal
	}

	// probably a valid ublox packet
	cmInstance->status[pHandle].startByte = UBLOX_START_BYTE1;
	ringBuffer->startIndex = ringBuffer->scanIndex;
	cmInstance->status[pHandle].passThroughBytes = UBLOX_HEADER_SIZE + headerLength + 2; // + 2 for checksum
	return 0; // 0 means keep reading as normal
}

int beginRtcm3Packet(com_manager_t* cmInstance, int pHandle, ring_buffer_t* ringBuffer)
{
	uint32_t count = ringBuffer->endIndex - (--ringBuffer->scanIndex);
	if (count < RTCM3_HEADER_SIZE)
	{
		// not enough data to read a rtcm3 packet header, wait for more data to arrive
		return 1; // 1 means more data needed
	}

	// if message length valid, probably an rtcm3 packet, message length starts at bit 14 and goes for 10 bits
	uint32_t msgLength = 0;
	for (uint32_t i = 14; i < 14 + 10; i++)
	{
		msgLength = (msgLength << 1) + ((ringBuffer->buf[ringBuffer->scanIndex + (i / 8)] >> (7 - i % 8)) & 1u);
	}
	if (msgLength > 1023)
	{
		ringBuffer->scanIndex++;
		cmInstance->status[pHandle].startByte = 0;
		return 0;
	}

	// probably a valid rtcm3 packet
	cmInstance->status[pHandle].startByte = RTCM3_START_BYTE;
	ringBuffer->startIndex = ringBuffer->scanIndex;
	cmInstance->status[pHandle].passThroughBytes = RTCM3_HEADER_SIZE + msgLength + 3; // + 3 for crc24
	return 0; // 0 means keep reading as normal
}

//////////////////////////////////////////////////////////////////////////
//  Packet Retry
//////////////////////////////////////////////////////////////////////////

/**
*   @brief stepPacketRetry - Resend the ensured packets after the ENSURE_RETRY_COUNT
*   period if the expected response was not received.
*/
void stepPacketRetry(com_manager_t* cmInstance)
{
	int32_t i;
	ensured_pkt_t* ePkt;

	for (i = 0; i < cmInstance->maxEnsuredPackets; i++)
	{
		ePkt = &(cmInstance->ensuredPackets[i]);

		// No more retries in list
		if (ePkt->counter == -2)
		{
			return;
		}

		// Check that retry is enabled
		if (ePkt->counter >= 0)
		{
			// Check if counter has expired
			if (--(ePkt->counter) == 0)
			{
				// Reset counter
				ePkt->counter = cmInstance->ensureRetryCount;

				// Reset packet
				sendPacket(cmInstance, ePkt->pHandle, &(ePkt->pkt), 0);
			}
		}
	}
}

/**
*   @brief registerPacketRetry - Saves data and packet header info
*   to a retry list that will be resent if the corresponding response
*   is not received (data or ack) within the given period.  The packet
*   header info must be populated following a call to this function.
*
*	@param[in] data[]   Pointer to data buffer.
*	@param[in] dataSize Size of the data buffer.
*
*	@return Pointer to retry packet.  The header info must be populated.
*/
packet_t* registerPacketRetry(com_manager_t* cmInstance, int pHandle, uint8_t pid, unsigned char data[], unsigned int dataSize)
{
	int32_t i;
	ensured_pkt_t *ePkt = 0;
	unsigned char searching = 1;

	#if ENABLE_FILTER_DUPLICATE_PACKETS

	#if ENABLE_FILTER_DUPLICATE_PACKETS_MATCH_ALL_CHARACTERS

	int32_t j;

	#endif

	// Filter out redundant retries (replace same type packets and pHandle with latest)
	p_data_get_t *getData1, *getData2;

	// Validate Data Size
	if (dataSize > MAX_P_DATA_BODY_SIZE)
	{
		return 0;
	}

	// Check for existing retry
	for (i = 0; searching && i < cmInstance->maxEnsuredPackets; i++)
	{
		ePkt = &(cmInstance->ensuredPackets[i]);

		// No more retries to search over.  Abort and look for first disabled slot.
		if (ePkt->counter == -2)
		{
			break;
		}

		// Found enabled retry w/ matching packet ID and data size
		if (ePkt->counter >= 0 &&
		ePkt->pkt.hdr.pid == pid		&&
		ePkt->pkt.body.size == dataSize &&
		ePkt->pHandle == pHandle)
		{
			switch (pid)
			{
			case PID_GET_DATA:
				getData1 = (p_data_get_t*)data;
				getData2 = (p_data_get_t*)ePkt->pktBody;

				// Match: all Get Data parameters
				if (getData1->id == getData2->id     &&
				getData1->size == getData2->size   &&
				getData1->offset == getData2->offset)
				searching = 0;
				break;

			case PID_STOP_BROADCASTS_ALL_PORTS:
				searching = 0;
				break;

			default:

#if !ENABLE_FILTER_DUPLICATE_PACKETS_MATCH_ALL_CHARACTERS

				// Match: first character
				if (ePkt->pkt.body.ptr[0] == data[0])
				{
					searching = 0;
				}

#else

				// Match: All character
				for (j = 0; j < dataSize; j++)
				{
					if (ePkt->pkt.body.ptr[j] == data[j])
					{
						searching = 0;
						break;
					}
				}

#endif

				break;
			}
		}
	}

	#endif

	// Find Empty Slot - either first available or tail if all used.
	for (i = 0; searching && i < cmInstance->maxEnsuredPackets; i++)
	{
		ePkt = &(cmInstance->ensuredPackets[i]);

		// Found empty slot
		if (ePkt->counter < 0)
		{
			searching = 0;
			break;
		}
	}

	// All slots enabled, so take the oldest (one after last used)
	if (searching && cmInstance->ensuredPackets != 0)
	{
		if (++cmInstance->lastEnsuredPacketIndex >= cmInstance->maxEnsuredPackets)
		{
			cmInstance->lastEnsuredPacketIndex = 0;
		}
		ePkt = &(cmInstance->ensuredPackets[cmInstance->lastEnsuredPacketIndex]);
	}
	else
	{
		cmInstance->lastEnsuredPacketIndex = i;
	}
	if (ePkt == 0)
	{
		return 0;
	}

	// Backup packet contents for retry if not already registered
	ePkt->counter = cmInstance->ensureRetryCount;
	memcpy(ePkt->pktBody, data, dataSize);

	// Update ePkt pkt header and body info
	ePkt->pkt.hdr.startByte = PSC_START_BYTE;
	ePkt->pkt.hdr.pid = pid;
	ePkt->pkt.body.ptr = ePkt->pktBody; // point to ePkt buffer "pktBody"
	ePkt->pkt.body.size = dataSize;
	ePkt->pHandle = pHandle;

	return &(ePkt->pkt);
}

/**
*   @brief Update packet retry.  If the specific data requested or acknowledge
*   is received, the retry list is updated as to no continue to resend the
*   corresponding message.
*
*	@param[in] *pkt        Pointer to pkt buffer.
*/
void updatePacketRetryData(com_manager_t* cmInstance, packet_t *pkt)
{
	int32_t i;
	ensured_pkt_t *ePkt;

	// Search for retries that match packet received.  If found, removed it from the retry list.
	for (i = 0; i < cmInstance->maxEnsuredPackets; i++)
	{
		ePkt = &(cmInstance->ensuredPackets[i]);

		if (ePkt->counter == -2)
		{
			// No more retries to search for
			return;
		}

		if (ePkt->counter < 0)
		{
			// This retry is disabled.  Skip it.
			continue;
		}

		// Found packet response expected.  Remove from retry list.
		if (ePkt->pktBody[0] == pkt->body.ptr[0])
		{
			// Indicate disabled retry
			ePkt->counter = -1;
		}
	}

	// Update last retry indicator
	for (i = cmInstance->maxEnsuredPackets - 1; i >= 0; i--)
	{
		// Current is enabled so stop
		if (cmInstance->ensuredPackets[i].counter >= 0)
		{
			break;
		}

		// Indicate no more retries in list
		cmInstance->ensuredPackets[i].counter = -2;
	}
}

void updatePacketRetryAck(com_manager_t* cmInstance, packet_t *pkt)
{
	int32_t i;
	ensured_pkt_t *ePkt;
	p_ack_t *ack;
	uint8_t ackInfo;

	ack = (p_ack_t*)(pkt->body.ptr);
	ackInfo = (uint8_t)(ack->hdr.pktInfo);

	// Search for retries that match packet received.  If found, removed it from the retry list.
	for (i = 0; i < cmInstance->maxEnsuredPackets; i++)
	{
		ePkt = &(cmInstance->ensuredPackets[i]);

		if (ePkt->counter == -2)
		{
			// No more retries to search for
			return;
		}

		if (ePkt->counter == -1)
		{
			// This retry is disabled.  Skip it.
			continue;
		}

		// Check packet info matches
		if (ack->hdr.pktInfo == ePkt->pkt.hdr.pid)
		{
			p_data_hdr_t *dHdr, *eHdr;

			switch (ackInfo)
			{
				default:
				// Custom/Specific Packets
				case PID_STOP_BROADCASTS_ALL_PORTS: // No body ID available
				ePkt->counter = -1;                 // indicate disabled retry
				break;

				case PID_SET_DATA:
				dHdr = (p_data_hdr_t*)ack->buf;
				eHdr = (p_data_hdr_t*)(ePkt->pktBody);

				if (dHdr->id == eHdr->id &&
				dHdr->size == eHdr->size &&
				dHdr->offset == eHdr->offset)
				{
					ePkt->counter = -1;             // indicate disabled retry
				}
				break;
			}
		}
	}

	// Update last retry indicator
	for (i = cmInstance->maxEnsuredPackets - 1; i >= 0; i--)
	{
		// Current is enabled so stop
		if (cmInstance->ensuredPackets[i].counter >= 0)
		{
			break;
		}
		cmInstance->ensuredPackets[i].counter = -2;         // Indicate no more retries in list
	}
}

int comManagerValidateBaudRate(unsigned int baudRate)
{
	// Valid baudrates for InertialSense hardware
	for (size_t i = 0; i < _ARRAY_ELEMENT_COUNT(g_validBaudRates); i++)
	{
		if (g_validBaudRates[i] == baudRate)
		{
			return 0;
		}
	}
	return -1;
}

