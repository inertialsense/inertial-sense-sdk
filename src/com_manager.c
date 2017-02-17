/*
MIT LICENSE

Copyright 2014 Inertial Sense, LLC - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include "com_manager.h"
#include <string.h>
#include <stdlib.h>

// allow packet continuation or not. If enabled, an extra 1K buffer is allocated globally for each com_manager_t instance.
#define ENABLE_PACKET_CONTINUATION 1

// enable filtering of duplicate packets
#define ENABLE_FILTER_DUPLICATE_PACKETS 1

// whether the first character or all characters are checked in duplicate packets
#define ENABLE_FILTER_DUPLICATE_PACKETS_MATCH_ALL_CHARACTERS 0

// enable the new parser in preparation for pass through
#define ENABLE_NEW_COM_MANAGER_PARSER 1

#define UBLOX_HEADER_SIZE 6
#define UBLOX_START_BYTE1 0xB5
#define UBLOX_START_BYTE2 0x62
#define RTCM3_START_BYTE 0xD3

#define PARSE_DOUBLE(str) strtod(str, 0)
#define PARSE_FLOAT(str) strtof(str, 0)

#if !defined(MALLOC) && !defined(FREE)

#if defined(AVR) || defined(ARM)

extern void* pvPortMalloc(size_t xWantedSize);
extern void vPortFree(void* pv);
#define MALLOC(m) pvPortMalloc(m)
#define FREE(m) vPortFree(m)

#else

#define MALLOC(m) malloc(m)
#define FREE(m) free(m)

#endif

#endif

#define MIN_REQUEST_PERIOD_MS       1               // (ms) 1 KHz
#define MAX_REQUEST_PERIOD_MS       100000          // (ms)
#define MSG_PERIOD_SEND_ONCE		-1
#define MSG_PERIOD_DISABLED			0

/* Contains callback information for a before and after send for a data structure */
typedef struct
{
	/* Pointer and size of entire data struct (not sub portion that is communicated) */
	bufTxRxPtr_t dataSet;

	/* Callback function pointer, used to prepare data before send */
	pfnComManagerPreSend preTxFnc;

	/* Callback function pointer, used to prepare data after received */
	pfnComManagerPostRead pstRxFnc;
	
	/* Packet type to use */
	uint8_t pktFlags;
} registered_data_t;

/* Contains data that determines what messages are being broadcast */
typedef struct
{
	/* Packet  */
	pkt_info_t              pkt;

	/* Broadcast specific data header (i.e. data size and offset) */
	p_data_hdr_t            dataHdr;

	/* Pointer and size of entire data set (not sub portion that is communicated) */
	bufTxRxPtr_t            dataSet;

	/* Broadcast period counter */
	int                     counter;

	/* Millisecond broadcast period intervals.  -1 = send once.  0 = disabled/unused/don't send. */
	int                     period;

	/* Linked list pointer.  0 indicates this is the head. */
	void*                   llPrv;

	/* Linked list pointer.  0 indicates this is the tail. */
	void*                   llNxt;

	/* Port to broadcast on. */
	int                     pHandle;
} broadcast_msg_t;

/* Contains data to implement ensured packet delivery */
typedef struct
{
	/* Packet struct */
	packet_t                pkt;

	/* Packet contents/body */
	uint8_t                 pktBody[PKT_BUF_SIZE];

	/* Count down counter between retries.  < 0 means disabled. -2 means no more enabled beyond this. */
	int                     counter;

	/* Port packet was sent on */
	int                     pHandle;
} ensured_pkt_t;

// continuation of packet data
typedef struct
{
	p_data_hdr_t	hdr;
	unsigned char	buf[MAX_DATASET_SIZE];
} com_manager_con_t;

// ring buffer storage size
#define RING_BUFFER_SIZE PKT_BUF_SIZE

// if ring buffer start index is less than this and no space is left, clear the entire ring buffer
#define RING_BUFFER_FLUSH_THRESHOLD (RING_BUFFER_SIZE / 3)

// ring buffer struct for each pHandle in com manager
typedef struct
{
	unsigned char buf[RING_BUFFER_SIZE];
	uint32_t startIndex;
	uint32_t endIndex;
	uint32_t scanIndex;
} ring_buffer_t;

typedef struct
{
	unsigned char pktCounter;

	// reads n bytes into buffer from the source (usually a serial port)
	pfnComManagerRead readCallback;

	// write data to the destination (usually a serial port)
	pfnComManagerSend sendPacketCallback;

	// bytes free in Tx buffer (used to check if packet, keeps us from overflowing the Tx buffer)
	pfnComManagerSendBufferAvailableBytes txFreeCallback;

	// Callback function pointer, used to respond to data input
	pfnComManagerPostRead pstRxFnc;

	// Callback function pointer, used to respond to ack
	pfnComManagerPostAck pstAckFnc;

	// Callback function pointer, used when disabling all broadcast messages
	pfnComManagerDisableBroadcasts disableBcastFnc;

	// Pointer to local data and data specific callback functions
	registered_data_t regData[DID_COUNT];

	// numHandles elements
	ring_buffer_t* ringBuffers;								

	// ensured packets
	ensured_pkt_t* ensuredPackets;		

	// handle ASCII messages not handled elsewhere
	pfnComManagerAsciiMessageHandler asciiMessageHandler;

	asciiMessageMap_t* asciiMessages;
	int	asciiMessagesCount;
	broadcast_msg_t msgs[MAX_NUM_BCAST_MSGS];
	broadcast_msg_t* msgsHead;
	broadcast_msg_t* msgsTail;

	// Number of communication ports
	int numHandes;											

	// default is 2, max number of packets to insured delivery at one time.  Adjust based on available memory.
	int maxEnsuredPackets;									

	// default is 2 - processing interval
	int stepPeriodMilliseconds;								

	// default is 3 - Ensure retry count
	int ensureRetryCount;				

	// current status, malloc for each possible port
	com_manager_status_t* status;

	// user defined pointer
	void* userPointer;

	// pass through handler
	pfnComManagerPassThrough passThroughHandler;

#if ENABLE_PACKET_CONTINUATION

	// continuation data for packets
	com_manager_con_t con;

#endif

} com_manager_t;

static com_manager_t g_cm = { 0 };

const unsigned int g_validBaudRates[IS_BAUDRATE_COUNT] = {IS_BAUDRATE_3000000, IS_BAUDRATE_921600, IS_BAUDRATE_460800, IS_BAUDRATE_230400, IS_BAUDRATE_115200};
	
void initComManagerInstanceInternal
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
	pfnComManagerDisableBroadcasts disableBcastFnc
);
int processAsciiRxPacket(com_manager_t* cmInstance, int pHandle, unsigned char* start, int count);
void parseAsciiPacket(com_manager_t* cmInstance, int pHandle, unsigned char* buf, int count);
int processBinaryRxPacket(com_manager_t* cmInstance, int pHandle, packet_t *pkt, unsigned char additionalDataAvailable);
int processDataRequest(com_manager_t* cmInstance, int pHandle, p_data_get_t *req);
void enableBroadcastMsg(com_manager_t* cmInstance, broadcast_msg_t *msg, int period_ms);
void disableBroadcastMsg(com_manager_t* cmInstance, broadcast_msg_t *msg);
void disableDidBroadcast(com_manager_t* cmInstance, int pHandle, p_data_disable_t *disable);
int sendPacket(com_manager_t* cmInstance, int pHandle, packet_t *dPkt, uint8_t additionalPktFlags);
int sendDataPacket(com_manager_t* cmInstance, int pHandle, pkt_info_t *msg);
void sendAck(com_manager_t* cmInstance, int pHandle, packet_t *pkt, unsigned char pid_ack);
int findAsciiMessage(const void * a, const void * b);

//  Packet processing
int encodeBinaryPacket(com_manager_t* cmInstance, int pHandle, buffer_t *pkt, packet_t *dPkt, uint8_t additionalPktFlags);
int decodeBinaryPacket(com_manager_t* cmInstance, int pHandle, packet_t *pkt, unsigned char* pbuf, int pbufSize);
int validateAsciiChecksum(com_manager_t* cmInstance, unsigned char* buf, int count);
int processAsciiPacket(com_manager_t* cmInstance, int pHandle, unsigned char* data, int dataLength);
int processBinaryPacket(com_manager_t* cmInstance, int pHandle, unsigned char* dataStart, unsigned char* data, int dataLength, unsigned char* additionalDataAvailable);
int processPassThroughBytes(com_manager_t* cmInstance, int pHandle, ring_buffer_t* ringBuffer);
int beginUbloxPacket(com_manager_t* cmInstance, int pHandle, ring_buffer_t* ringBuffer);
int decodeBinaryPacketByte(com_manager_t* cmInstance, int pHandle, uint8_t** _ptrSrc, uint8_t** _ptrDest, uint32_t* checksum, uint32_t shift);

//  Packet Retry
void stepPacketRetry(com_manager_t* cmInstance);
packet_t* registerPacketRetry(com_manager_t* cmInstance, int pHandle, pkt_info_byte_t pid, unsigned char data[], unsigned int dataSize);
void updatePacketRetryData(com_manager_t* cmInstance, packet_t *pkt);
void updatePacketRetryAck(com_manager_t* cmInstance, packet_t *pkt);

void stepComManagerSendMessages(void);
void stepComManagerSendMessagesInstance(CMHANDLE cmInstance);

CMHANDLE getGlobalComManager(void) { return &g_cm; }

void initComManager
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
	pfnComManagerDisableBroadcasts disableBcastFnc
)
{
	initComManagerInstanceInternal(&g_cm, numHandles, maxEnsuredPackets, stepPeriodMilliseconds, retryCount, readFnc, sendFnc, txFreeFnc, pstRxFnc, pstAckFnc, disableBcastFnc);
}

CMHANDLE initComManagerInstance
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
	pfnComManagerDisableBroadcasts disableBcastFnc
)
{
	com_manager_t* cmInstance = (com_manager_t*)MALLOC(sizeof(com_manager_t));
	while( cmInstance==NULL ) { /* Error check malloc */ }
	memset(cmInstance, 0, sizeof(com_manager_t));
	initComManagerInstanceInternal(cmInstance, numHandles, maxEnsuredPackets, stepPeriodMilliseconds, retryCount, readFnc, sendFnc, txFreeFnc, pstRxFnc, pstAckFnc, disableBcastFnc);

	return cmInstance;
}

void initComManagerInstanceInternal
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
	pfnComManagerDisableBroadcasts disableBcastFnc
)
{
	int i;
	
	initDataSets();
	
	// free ring buffers and ensured packet memory
	if (cmInstance->ringBuffers != 0)
	{
		FREE(cmInstance->ringBuffers);
		cmInstance->ringBuffers = 0;
	}
	if (cmInstance->ensuredPackets != 0)
	{
		FREE(cmInstance->ensuredPackets);
		cmInstance->ensuredPackets = 0;
	}
	if (cmInstance->status != 0)
	{
		FREE(cmInstance->status);
		cmInstance->status = 0;
	}
	
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

	// Allocate ring buffers for serial reads / writes
	cmInstance->ringBuffers = (ring_buffer_t*)MALLOC(sizeof(ring_buffer_t) * numHandles);
    memset(cmInstance->ringBuffers, 0, sizeof(ring_buffer_t) * numHandles);

	// Allocate memory for ensured packets
	if (cmInstance->maxEnsuredPackets > 0)
	{
		cmInstance->ensuredPackets = MALLOC(sizeof(ensured_pkt_t) * cmInstance->maxEnsuredPackets);
        memset(cmInstance->ensuredPackets, 0, sizeof(ensured_pkt_t) * cmInstance->maxEnsuredPackets);
        for (i = 0; i < cmInstance->maxEnsuredPackets; i++)
        {
            cmInstance->ensuredPackets[i].counter = -2; // indicates no retries are enabled
            cmInstance->ensuredPackets[i].pkt.body.ptr = cmInstance->ensuredPackets[i].pktBody;
        }
	}

	cmInstance->status = MALLOC(sizeof(com_manager_status_t) * numHandles);
    memset(cmInstance->status, 0, sizeof(com_manager_status_t) * numHandles);
	for (int i = 0; i < numHandles; i++)
	{
		cmInstance->status[i].flags = IS_LITTLE_ENDIAN;
	}
}

static int asciiMessageCompare(const void* elem1, const void* elem2)
{
	asciiMessageMap_t* e1 = (asciiMessageMap_t*)elem1;
	asciiMessageMap_t* e2 = (asciiMessageMap_t*)elem2;

	return memcmp(e1->messageId, e2->messageId, 4);
}

void registerComManagerASCII(asciiMessageMap_t* asciiMessages, int asciiMessagesCount, pfnComManagerAsciiMessageHandler msgFnc)
{
	registerComManagerASCIIInstance(&g_cm, asciiMessages, asciiMessagesCount, msgFnc);
}

void registerComManagerASCIIInstance(CMHANDLE cmInstance_, asciiMessageMap_t* asciiMessages, int asciiMessagesCount, pfnComManagerAsciiMessageHandler msgFnc)
{
	com_manager_t* cmInstance = (com_manager_t*)cmInstance_;

	cmInstance->asciiMessages = asciiMessages;
	cmInstance->asciiMessagesCount = asciiMessagesCount;
	cmInstance->asciiMessageHandler = msgFnc;
	
	if (asciiMessagesCount > 1)
	{
		qsort(asciiMessages, sizeof(asciiMessages[0]), asciiMessagesCount, asciiMessageCompare);
	}
}

void registerComManager(uint32_t dataId, pfnComManagerPreSend txFnc, pfnComManagerPostRead pstRxFnc, void* txDataPtr, void* rxDataPtr, int dataSize, uint8_t pktFlags)
{
	registerComManagerInstance(&g_cm, dataId, txFnc, pstRxFnc, txDataPtr, rxDataPtr, dataSize, pktFlags);
}

void registerComManagerInstance(CMHANDLE cmInstance_, uint32_t dataId, pfnComManagerPreSend txFnc, pfnComManagerPostRead pstRxFnc, void* txDataPtr, void* rxDataPtr, int dataSize, uint8_t pktFlags)
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

void stepComManager(void)
{
	stepComManagerInstance(&g_cm);
}

void stepComManagerInstance(CMHANDLE cmInstance_)
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
						//case RTCM3_START_BYTE:
						//{
						// TODO: Implement RTCM3 forwarder
						//} break;
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
	broadcast_msg_t* bcPtr = cmInstance->msgsHead;

	while (bcPtr)
	{
		// Wait until buffer is empty enough.
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
		else
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

			// Iterate to next message in message list
			bcPtr = (broadcast_msg_t*)bcPtr->llNxt;
		}		
	}

	// Resend data (if necessary)
	stepPacketRetry(cmInstance);
}

void setComManagerSendEndianess(int pHandle, int littleEndian)
{
	setComManagerSendEndianessInstance(&g_cm, pHandle, littleEndian);
}

void setComManagerSendEndianessInstance(CMHANDLE cmInstance, int pHandle, int littleEndian)
{
	com_manager_t* _cmInstance = (com_manager_t*)cmInstance;
	
	if (littleEndian)
	{
		_cmInstance->status[pHandle].flags |= (CM_PKT_FLAGS_LITTLE_ENDIAN);
	}
	else
	{
		_cmInstance->status[pHandle].flags &= ~(CM_PKT_FLAGS_LITTLE_ENDIAN);
	}
}

void freeComManagerInstance(CMHANDLE cmInstance_)
{
	if (cmInstance_ != 0 && cmInstance_ != &g_cm)
	{
		com_manager_t* cmInstance = (com_manager_t*)cmInstance_;
		if (cmInstance->ringBuffers != 0)
		{
			FREE(cmInstance->ringBuffers);
		}
		if (cmInstance->ensuredPackets != 0)
		{
			FREE(cmInstance->ensuredPackets);
		}
		if (cmInstance->status != 0)
		{
			FREE(cmInstance->status);
		}
		FREE(cmInstance);
	}
}

void setComManagerPassThrough(pfnComManagerPassThrough handler)
{
	setComManagerPassThroughInstance(&g_cm, handler);
}

void setComManagerPassThroughInstance(CMHANDLE cmInstance, pfnComManagerPassThrough handler)
{
	if (cmInstance != 0)
	{
		((com_manager_t*)cmInstance)->passThroughHandler = handler;
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

com_manager_status_t* getStatusComManager(int pHandle)
{
	return getStatusComManagerInstance(&g_cm, pHandle);
}

com_manager_status_t* getStatusComManagerInstance(CMHANDLE cmInstance, int pHandle)
{
	return &(((com_manager_t*)cmInstance)->status[pHandle]);
}

/*!
*   @brief Request data
*   This function requests the specified data w/ offset and length 
*   for partial reads.  
*
*	@param[in] dataId       Data structure ID
*	@param[in] offset   Byte offset into data structure.  0 = data start.
*	@param[in] length   Byte length of data.  0 = entire structure.
*	@param[in] period_ms Broadcast period of requested data.  0 = single request.
*
*	@return 0 on successful request.  -1 on failure.
*/
void getDataComManager(int pHandle, uint32_t dataId, int offset, int size, int period_ms)
{
	getDataComManagerInstance(&g_cm, pHandle, dataId, offset, size, period_ms);
}

void getDataComManagerInstance(CMHANDLE cmInstance, int pHandle, uint32_t dataId, int offset, int size, int period_ms)
{
	p_data_get_t request;
	bufPtr_t data;

	// Create and Send request packet
	request.id = dataId;
	request.offset = offset;
	request.size = size;
	request.bc_period_ms = period_ms;

	data.ptr = (uint8_t*)&request;
	data.size = sizeof(request);
	sendComManagerInstance(cmInstance, pHandle, PID_GET_DATA, 0, &data, 0);

	// sendEnsuredComManager(pHandle, PID_GET_DATA, (unsigned char*)&request, sizeof(request));
}


int sendDataComManager(int pHandle, uint32_t dataId, void *dataPtr, int dataSize, int dataOffset)
{
	return sendDataComManagerInstance(&g_cm, pHandle, dataId, dataPtr, dataSize, dataOffset);
}

int sendDataComManagerInstance(CMHANDLE cmInstance, int pHandle, uint32_t dataId, void* dataPtr, int dataSize, int dataOffset)
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

	return sendComManagerInstance(cmInstance, pHandle, PID_SET_DATA, &bodyHdr, &data, 0);

	// return sendEnsuredComManager(pHandle, PID_SET_DATA, &bodyHdr, &data);
}

int sendDataComManagerNoAck(int pHandle, uint32_t dataId, void *dataPtr, int dataSize, int dataOffset)
{
	return sendDataComManagerNoAckInstance(&g_cm, pHandle, dataId, dataPtr, dataSize, dataOffset);
}

int sendDataComManagerNoAckInstance(CMHANDLE cmInstance, int pHandle, uint32_t dataId, void* dataPtr, int dataSize, int dataOffset)
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

	return sendComManagerInstance((com_manager_t*)cmInstance, pHandle, PID_DATA, &bodyHdr, &data, 0);
}

int sendRawDataComManager(int pHandle, uint32_t dataId, void *dataPtr, int dataSize, int dataOffset)
{
	return sendRawDataComManagerInstance(&g_cm, pHandle, dataId, dataPtr, dataSize, dataOffset);
}

int sendRawDataComManagerInstance(CMHANDLE cmInstance, int pHandle, uint32_t dataId, void* dataPtr, int dataSize, int dataOffset)
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

	return sendComManagerInstance((com_manager_t*)cmInstance, pHandle, PID_SET_DATA, &bodyHdr, &data, CM_PKT_FLAGS_RAW_DATA_NO_SWAP);
}

int disableDataComManager(int pHandle, uint32_t dataId)
{
	return disableDataComManagerInstance(&g_cm, pHandle, dataId);
}

int disableDataComManagerInstance(CMHANDLE cmInstance, int pHandle, uint32_t dataId)
{
    bufPtr_t data;
    data.ptr  = (uint8_t*)&dataId;
    data.size = 4;

    return sendComManagerInstance(cmInstance, pHandle, PID_STOP_DID_BROADCAST, 0, &data, 0);
}

int sendComManager(int pHandle, pkt_info_byte_t pktInfo, bufPtr_t *bodyHdr, bufPtr_t *txData, uint8_t pFlags)
{
	return sendComManagerInstance(&g_cm, pHandle, pktInfo, bodyHdr, txData, pFlags);
}

int sendComManagerInstance(CMHANDLE cmInstance, int pHandle, pkt_info_byte_t pktInfo, bufPtr_t* bodyHdr, bufPtr_t* txData, uint8_t pktFlags)
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

int sendEnsuredComManager(int pHandle, pkt_info_byte_t pktInfo, unsigned char* data, unsigned int dataSize)
{
	return sendEnsuredComManagerInstance(&g_cm, pHandle, pktInfo, data, dataSize);
}

int sendEnsuredComManagerInstance(CMHANDLE cmInstance, int pHandle, pkt_info_byte_t pktInfo, unsigned char *data, unsigned int dataSize)
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
	unsigned char* start = buf + 1; // skip $ char
	unsigned char* end = buf + count;
	unsigned char* ptr = buf;
	int fieldIndex = -1;
	asciiMessageMap_t* foundMap = 0;
	cmInstance->status[pHandle].flags |= CM_PKT_FLAGS_RX_VALID_DATA; // communication received
	cmInstance->status[pHandle].readCounter = 0;

	// Packet read success
	cmInstance->status[pHandle].rxPktCount++;

	// parse the ASCII line, skipping the initial $ char
	while (ptr <= end)
	{
		c = *(++ptr);
		if (c == ',' || c == '*')
		{
			if (fieldIndex == -1)
			{
				if (ptr - start < 4)
				{
					// message ids must be exactly 4 chars, they can be suffixed with a -[ANYTHING] though - in reality the suffix can be any sequence of non comma and non asterisk chars, and the hyphen
					// is not required, although it is recommended for clarity. The suffix is useful when the message is echoed back so the caller can know exactly which message was received
					// in the event that multiple messages with the same message id are being transmitted in quick succession.
					return;
				}

				// see if there's a global handler that processes the message, if so we are done
				// if the global handler did not process the message and we don't have any data mappings, we are also done
				if ((cmInstance->asciiMessageHandler != 0 && cmInstance->asciiMessageHandler(cmInstance, start, buf, count)) ||
					cmInstance->asciiMessages == 0 || cmInstance->asciiMessagesCount == 0)
				{
					return;
				}

				foundMap = bsearch(start, cmInstance->asciiMessages, cmInstance->asciiMessagesCount, sizeof(cmInstance->asciiMessages[0]), findAsciiMessage);
				if (foundMap == 0)
				{
					// no global handle and no data mapping for this message id, we are done
					return;
				}
			}
			else if (fieldIndex >= foundMap->fieldCount)
			{
				// field count exceeded, we are done
				return;
			}
			else
			{
				// empty fields do not perform any modification
				if (start + 1 != ptr)
				{
					unsigned char tmp = *ptr;
					*ptr = '\0';
					uint16_t fieldAndOffset = foundMap->fieldsAndOffsets[fieldIndex];
					asciiDataType type = (fieldAndOffset & 0x00FF);
					uint8_t offset = (fieldAndOffset & 0xFF00) >> 8;

					switch (type)
					{
						case asciiTypeInt:
						{
							const int dataValue = strtol((char*)start, 0, 10);
							memcpy(foundMap->ptr + offset, &dataValue, sizeof(dataValue));
						} break;
						case asciiTypeUInt:
						{
							const unsigned int dataValue = strtoul((char*)start, 0, 10);
							memcpy(foundMap->ptr + offset, &dataValue, sizeof(dataValue));
						} break;
						case asciiTypeFloat:
						{
                            const float dataValue = PARSE_FLOAT((char*)start);
							memcpy(foundMap->ptr + offset, &dataValue, sizeof(dataValue));
						} break;
						case asciiTypeDouble:
						{
                            const double dataValue = PARSE_DOUBLE((char*)start);
							memcpy(foundMap->ptr + offset, &dataValue, sizeof(dataValue));
						} break;
					}
					
					*ptr = tmp;
				}
			}
			start = ++ptr; // skip comma char with ++
			fieldIndex++;
		}
	}
}


// Return value: 0 = success, -1 = error.
int processAsciiRxPacket(com_manager_t* cmInstance, int pHandle, unsigned char* buf, int count)
{
	if (count < 10 || ((cmInstance->asciiMessages == 0 || cmInstance->asciiMessagesCount == 0) && (cmInstance->asciiMessageHandler == 0)))
	{
		return -1;
	}

	// checksum check first, we don't want to start messing up memory or running commands if the checksum doesn't match
	if (validateAsciiChecksum(cmInstance, buf, count) == 0)
	{
		cmInstance->status[pHandle].readCounter += 32;
		return -1;
	}

	// parse the packet, it is probably good to go
	parseAsciiPacket(cmInstance, pHandle, buf, count);
	return 0;
}

/*!
*   @brief Process binary packet content:
*
*	@return 0 on success.  -1 on failure.
*/
int processBinaryRxPacket(com_manager_t* cmInstance, int pHandle, packet_t *pkt, unsigned char additionalDataAvailable)
{
	p_data_t			*data;
	p_data_hdr_t		*dataHdr;
	uint8_t				*dataBuf;
	registered_data_t	*regd;
	pkt_info_byte_t		pid = (pkt_info_byte_t)(pkt->hdr.pid);

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
		dataBuf = data->buf;

		// Validate Data
		if (dataHdr->id >= DID_COUNT)
		{
			return -1;
		}

		regd = &(cmInstance->regData[dataHdr->id]);

		if ((pkt->hdr.flags & CM_PKT_FLAGS_ENDIANNESS_MASK) != IS_LITTLE_ENDIAN)
		{
			uint16_t* offsets;
			uint16_t offsetsLength;

			// flip doubles back if needed
			if ((offsets = getDoubleOffsets(dataHdr->id, &offsetsLength)))
			{
				flipDoubles(dataBuf, dataHdr->size, dataHdr->offset, offsets, offsetsLength);
			}

			// flip strings back if needed
			if ((offsets = getStringOffsetsLengths(dataHdr->id, &offsetsLength)))
			{
				flipStrings(dataBuf, dataHdr->size, dataHdr->offset, offsets, offsetsLength);
			}
		}

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
		if (additionalDataAvailable || cmInstance->con.hdr.size)
		{
			// New dataset
			if (cmInstance->con.hdr.id == 0 ||
				cmInstance->con.hdr.size == 0 ||
				cmInstance->con.hdr.id != dataHdr->id)
			{
				// Reset data consolidation
				memset(&cmInstance->con, 0, sizeof(cmInstance->con));
				cmInstance->con.hdr.id = dataHdr->id;
				cmInstance->con.hdr.offset = dataHdr->offset;
			}

			// Ensure data will fit in buffer
			if ((cmInstance->con.hdr.size + dataHdr->size) < sizeof(cmInstance->con.buf))
			{
				// Add data to buffer
				memcpy(&(cmInstance->con.buf[cmInstance->con.hdr.size]), dataBuf, dataHdr->size);
				cmInstance->con.hdr.size += dataHdr->size;
			}

			// Wait for end of data
			if (additionalDataAvailable)
			{
				return 0;
			}

			// Use consolidated data
			data = (p_data_t*)&cmInstance->con;
		}
            
#else
            
        (void)additionalDataAvailable;

#endif

		// Write to data structure if it was registered
		if (regd->dataSet.rxPtr)
		{
			copyDataPToStructP(regd->dataSet.rxPtr, data, regd->dataSet.size);
		}

		// Call data specific callback
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

		// Clear dataset consolidation if necessary
		if (cmInstance->con.hdr.size)
		{
			cmInstance->con.hdr.size = 0;
		}

#endif

		// Replay w/ ACK for PID_SET_DATA
		if (pid == PID_SET_DATA)
		{
			sendAck(cmInstance, pHandle, pkt, PID_ACK);
		}
		break;

	case PID_GET_DATA:
		if (processDataRequest(cmInstance, pHandle, (p_data_get_t *)(pkt->body.ptr)))
		{
			sendAck(cmInstance, pHandle, pkt, PID_NACK);
		}
		break;

	case PID_STOP_ALL_BROADCASTS:
		disableAllBroadcastsInstance(cmInstance);

		// Call disable all broadcast callback if exists
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

// 0 on success. -1 on failure.
int processDataRequest(com_manager_t* cmInstance, int pHandle, p_data_get_t *req)
{
	broadcast_msg_t* msg = 0;
	int i;

	// Validate the request
	if (req->id >= DID_COUNT)
	{
		return -1;
	}

	// Constrain request broadcast period if necessary
	if (req->bc_period_ms != 0)
	{
		_LIMIT2(req->bc_period_ms, MIN_REQUEST_PERIOD_MS, MAX_REQUEST_PERIOD_MS);
	}

	// if size is 0 and offset is 0, set size to full data struct size
	if (req->size == 0 && req->offset == 0 && req->id < sizeof(cmInstance->regData) / sizeof(cmInstance->regData[0]))
	{
		req->size = cmInstance->regData[req->id].dataSet.size;
	}

	// Search for matching message (i.e. matches pHandle, id, size, and offset)...
	for (i = 0; i < MAX_NUM_BCAST_MSGS; i++)
	{
		if (cmInstance->msgs[i].pHandle == pHandle &&
			cmInstance->msgs[i].dataHdr.id == req->id &&
			cmInstance->msgs[i].dataHdr.size == req->size &&
			cmInstance->msgs[i].dataHdr.offset == req->offset
			)
		{
			msg = &cmInstance->msgs[i];
			break;
		}
	}

	// otherwise use the first available (period=0) message.
	if (msg == 0)
	{
		for (i = 0; i < MAX_NUM_BCAST_MSGS; i++)
		{
			if (cmInstance->msgs[i].period == MSG_PERIOD_DISABLED)
			{
				msg = &cmInstance->msgs[i];
				break;
			}
		}

		// Abort if we ran out of broadcast message slots
		if (msg == 0)
		{
			return -1;
		}
	}

	// Copy reference to source data
	msg->dataSet = cmInstance->regData[req->id].dataSet;

	// Abort if no data pointer is registered or offset + size is out of bounds
	if (msg->dataSet.txPtr == 0 || msg->dataSet.size == 0 || req->offset + req->size > msg->dataSet.size)
	{
		return -1;
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
	msg->pkt.txData.ptr = msg->dataSet.txPtr + req->offset;

	// Prep data if callback exists
	if (cmInstance->regData[msg->dataHdr.id].preTxFnc)
	{
		cmInstance->regData[msg->dataHdr.id].preTxFnc(cmInstance, pHandle);
	}

	// Send data
	if (req->bc_period_ms > 0)
	{	
		// ***  Request Broadcast  ***
		// Send data immediately if possible
		if (cmInstance->txFreeCallback == 0 || msg->pkt.txData.size <= (uint32_t)cmInstance->txFreeCallback(cmInstance, pHandle))
		{
			sendDataPacket(cmInstance, pHandle, &(msg->pkt));
		}

		// Enable broadcast message
		enableBroadcastMsg(cmInstance, msg, req->bc_period_ms);
	}
	else
	{	// ***  Request Single  ***

		// Send data immediately if possible
		if (cmInstance->txFreeCallback == 0 || msg->pkt.txData.size <= (uint32_t)cmInstance->txFreeCallback(cmInstance, pHandle))
		{
			sendDataPacket(cmInstance, pHandle, &(msg->pkt));
			disableBroadcastMsg(cmInstance, msg);
		}
		else
		{
			// Won't fit in queue, so send it later
			enableBroadcastMsg(cmInstance, msg, req->bc_period_ms);
		}
	}

	return 0;
}

void enableBroadcastMsg(com_manager_t* cmInstance, broadcast_msg_t *msg, int period_ms)
{
	// Add to linked list if not already running
	if (msg->period == MSG_PERIOD_DISABLED)
	{
		if (cmInstance->msgsHead)
		{
			// Non-empty linked list.  Add to head of linked list
			cmInstance->msgsHead->llPrv = msg;
			msg->llNxt = cmInstance->msgsHead;
			cmInstance->msgsHead = msg;
		}
		else
		{   // Empty linked list.  Add to head.
			cmInstance->msgsHead = msg;
			cmInstance->msgsTail = msg;
		}
	}

	// Update broadcast period
	if (period_ms > 0)
	{
		msg->period = period_ms / cmInstance->stepPeriodMilliseconds;
	}
	else
	{
		msg->period = MSG_PERIOD_SEND_ONCE;
	}
	msg->counter = -1;   // Keeps broadcast from sending for at least one period
}

void disableBroadcastMsg(com_manager_t* cmInstance, broadcast_msg_t *msg)
{
	// Remove item from linked list
	broadcast_msg_t* llPrv = (broadcast_msg_t*)msg->llPrv;
	broadcast_msg_t* llNxt = (broadcast_msg_t*)msg->llNxt;
	if (llPrv)   llPrv->llNxt = llNxt;
	if (llNxt)   llNxt->llPrv = llPrv;
	msg->llPrv = 0;
	msg->llNxt = 0;

	// Update Head pointer if needed
	if (cmInstance->msgsHead == msg)
	{
		cmInstance->msgsHead = llNxt;
	}

	// Update Tail pointer if needed
	if (cmInstance->msgsTail == msg)
	{
		cmInstance->msgsTail = llPrv;
	}

	// Disabled indicator
	msg->period = MSG_PERIOD_DISABLED;
}

void disableAllBroadcasts(void)
{
	disableAllBroadcastsInstance(&g_cm);
}

void disableAllBroadcastsInstance(CMHANDLE cmInstance_)
{
	com_manager_t* cmInstance = (com_manager_t*)cmInstance_;
	broadcast_msg_t *msg = cmInstance->msgsHead;
	broadcast_msg_t *llNxt;

	// Zero out the head and tail pointers.
	cmInstance->msgsHead = 0;
	cmInstance->msgsTail = 0;

	// Ensure period and the previous/next linked list pointers are 0.
	while (msg)
	{
		llNxt = (broadcast_msg_t*)msg->llNxt;
		msg->llPrv = msg->llNxt = 0;
		msg->period = MSG_PERIOD_DISABLED;
		msg = llNxt;
	}
}

void disableDidBroadcast(com_manager_t* cmInstance, int pHandle, p_data_disable_t *disable)
{
	int i;

	// Search for matching message by ID
	for (i = 0; i < MAX_NUM_BCAST_MSGS; i++)
	{
		if (cmInstance->msgs[i].dataHdr.id == disable->id && cmInstance->msgs[i].pHandle == pHandle)
		{
			disableBroadcastMsg(cmInstance, &cmInstance->msgs[i]);
			break;
		}
	}
}

/*!
*   @brief Encode and send out serial port the referenced packet structure.
* 
*	@param[in/out] dPkt Packet structure containing packet info.
*   
*	@return 0 on success.  -1 on failure.
*/
int sendPacket(com_manager_t* cmInstance, int pHandle, packet_t *dPkt, uint8_t additionalPktFlags)
{
	buffer_t buffer;

	// check if we need to change the send data endianess
	if ((cmInstance->status[pHandle].flags & CM_PKT_FLAGS_ENDIANNESS_MASK) != IS_LITTLE_ENDIAN)
	{
		flipEndianess32(dPkt->body.ptr, dPkt->body.size);
	}

	if (encodeBinaryPacket(cmInstance, pHandle, &buffer, dPkt, additionalPktFlags))
	{
		return -1;
	}

	// Send Packet
	if (cmInstance->sendPacketCallback)
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
			// Breakup data into MAX_P_DATA_BODY_SIZE chunks of data if necessary
			int forceEndianessSwap = ((cmInstance->status[pHandle].flags & CM_PKT_FLAGS_ENDIANNESS_MASK) != IS_LITTLE_ENDIAN);

			// Setup packet and encoding state
			buffer_t bufToEncode;
			p_data_hdr_t hdr = *(p_data_hdr_t*)msg->bodyHdr.ptr;
			p_data_hdr_t* hdrToSend = (p_data_hdr_t*)bufToEncode.buf;
			uint32_t size = hdr.size;
			uint32_t offset = 0;
			uint32_t id = hdr.id;
			pkt.body.ptr = bufToEncode.buf;
			pkt.body.size = hdrToSend->size;

			while (size > 0)
			{
				// Assign data header values
				hdrToSend->size = _MIN(size, MAX_P_DATA_BODY_SIZE);
				hdrToSend->offset = hdr.offset + offset;
				hdrToSend->id = id;

				// copy the data to send to bufToEncode, skipping the data header - since we had to create that data header, we now have to append the actual data
				memcpy(bufToEncode.buf + sizeof(p_data_hdr_t), msg->txData.ptr + offset, hdrToSend->size);

				// adjust size and offset values in case we have another piece of this packet to send
				size -= hdrToSend->size;
				offset += hdrToSend->size;

				// Set data body size
				pkt.body.size = sizeof(p_data_hdr_t) + hdrToSend->size;

				// check if we need to change the send data endianess
				if (forceEndianessSwap)
				{
					// send the packet in the endianess that was set
					uint16_t* offsets;
					uint16_t offsetsLength;

					// flip doubles back if needed
					if ((offsets = getDoubleOffsets(id, &offsetsLength)))
					{
						flipDoubles(pkt.body.ptr + sizeof(p_data_hdr_t), hdrToSend->size, hdrToSend->offset, offsets, offsetsLength);
					}

					// flip strings back if needed
					if ((offsets = getStringOffsetsLengths(id, &offsetsLength)))
					{
						flipStrings(pkt.body.ptr + sizeof(p_data_hdr_t), hdrToSend->size, hdrToSend->offset, offsets, offsetsLength);
					}

					// flip all 4 byte pairs - id, offset and size in the header are now invalid after this point
					flipEndianess32(pkt.body.ptr, hdrToSend->size + sizeof(p_data_hdr_t));
				}

				// Encode the packet, handling special characters, etc.
				if (encodeBinaryPacket(cmInstance, pHandle, &bufToSend, &pkt, CM_PKT_FLAGS_MORE_DATA_AVAILABLE * (size & 1)))
				{
					return -1;
				}

				// Send the packet using the specified callback
				sendCallback(cmInstance, pHandle, &bufToSend);
				cmInstance->status[pHandle].txPktCount++;
			}
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

	sendComManagerInstance(cmInstance, pHandle, (pkt_info_byte_t)pid_ack, 0, &data, 0);
}


// Replace special character with encoded equivalent and add to buffer
static __inline uint8_t* encodeByteAddToBuffer(unsigned val, uint8_t* ptrDest)
{
	switch (val)
	{
		case PSC_ASCII_START_BYTE: 
		case PSC_ASCII_END_BYTE: 
		case PSC_START_BYTE: 
		case PSC_END_BYTE: 
		case PSC_RESERVED_KEY:
		case UBLOX_START_BYTE1:
		case RTCM3_START_BYTE:
			*ptrDest++ = PSC_RESERVED_KEY;
			*ptrDest++ = ~val;
			break;
		default:
			*ptrDest++ = val;
			break;
	}
	
	return ptrDest;
}


//////////////////////////////////////////////////////////////////////////
//  Packet Composition
//////////////////////////////////////////////////////////////////////////
/*!
*  @brief Adds data to a packet: adds start, info, data length, data, checksum, and stop bytes.
*  All data is communicated in Big Endian (AVR/ARM) order.
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
    // Ensure data size is small enough, assuming packet size could double after encoding.
    if (dPkt->body.size > MAX_PKT_BODY_SIZE)
	{
        return -1;
	}

    // Update Packet Counter		
	dPkt->hdr.counter = cmInstance->pktCounter++;
    uint8_t* ptrSrc;
	uint8_t* ptrSrcEnd;
	uint8_t* ptrDest = pkt->buf;
	unsigned shifter = 8;
	unsigned checkSumValue = 0x0000FF00; // the packet start byte begins the checksum	
	unsigned val;

	// Packet header -------------------------------------------------------------------------------------------
	*ptrDest++ = PSC_START_BYTE;

	// PID
	val = dPkt->hdr.pid;
	ptrDest = encodeByteAddToBuffer(val, ptrDest);
	checkSumValue ^= val; // no shift
	
	// Counter
	val = dPkt->hdr.counter;
	ptrDest = encodeByteAddToBuffer(val, ptrDest);
	checkSumValue ^= (val << 8); // shift 8
	
	// Flags
	val = cmInstance->status[pHandle].flags | dPkt->hdr.flags | additionalPktFlags;
	ptrDest = encodeByteAddToBuffer(val, ptrDest);
	checkSumValue ^= val; // no shift

	// Packet body ----------------------------------------------------------------------------------------------
	ptrSrc = (uint8_t*)dPkt->body.ptr;
	ptrSrcEnd = ptrSrc + dPkt->body.size;

	// copy body bytes, doing encoding and checksum
	while (ptrSrc != ptrSrcEnd)
	{
		val = *ptrSrc++;
		checkSumValue ^= (val << shifter);
		shifter ^= 8;
		ptrDest = encodeByteAddToBuffer(val, ptrDest);
	}
	
	// footer, not part of checksum ----------------------------------------------------------------------------------------------------
	*ptrDest++ = 0; // reserved
	
	// we need to encode the checksum values if they end up being a special character value
	val = (uint8_t)(checkSumValue >> 8) & 0xFF;
	ptrDest = encodeByteAddToBuffer(val, ptrDest);
	
	val = (uint8_t)(checkSumValue & 0xFF);
	ptrDest = encodeByteAddToBuffer(val, ptrDest);
		
	// finish the packet
	*ptrDest++ = PSC_END_BYTE;
	pkt->size = (uint32_t)(ptrDest - pkt->buf);

    return 0;
}


static __inline void decodeBinaryPacketFooter(packet_ftr_t* ftr, uint8_t** _ptrSrcEnd, uint32_t* checksum)
{
	uint8_t* ptrSrcEnd = *_ptrSrcEnd;

	// decode the footer first, accounting for special bytes
	ftr->stopByte = *(--ptrSrcEnd);
	ftr->cksum2 = *(--ptrSrcEnd);
	ftr->cksum1 = *(--ptrSrcEnd);
	if (ftr->cksum1 == PSC_RESERVED_KEY)
	{
		ftr->cksum2 = ~ftr->cksum2;
		ftr->cksum1 = *(--ptrSrcEnd);
		ftr->reserved = *(--ptrSrcEnd);
		if (ftr->reserved == PSC_RESERVED_KEY)
		{
			ftr->cksum1 = ~ftr->cksum1;
			ftr->reserved = *(--ptrSrcEnd);
		}
	}
	else
	{
		ftr->reserved = *(--ptrSrcEnd);
		if (ftr->reserved == PSC_RESERVED_KEY)
		{
			ftr->cksum1 = ~ftr->cksum1;
			ftr->reserved = *(--ptrSrcEnd);
		}
	}
	*checksum = ((unsigned)ftr->cksum1 << 8) | (unsigned)ftr->cksum2;

	*_ptrSrcEnd = ptrSrcEnd;
}

__inline int decodeBinaryPacketByte(com_manager_t* cmInstance, int pHandle, uint8_t** _ptrSrc, uint8_t** _ptrDest, uint32_t* checksum, uint32_t shift)
{
	uint8_t* ptrSrc = *_ptrSrc;
// 	uint8_t* ptrDest = *_ptrDest;
	(void)_ptrDest;

	// packet id byte
	uint32_t val = *ptrSrc++;
	switch (val)
	{
	case PSC_ASCII_START_BYTE:
	case PSC_ASCII_END_BYTE:
	case PSC_START_BYTE:
	case PSC_END_BYTE:
		// corrupt data
		cmInstance->status[pHandle].readCounter += 32;
		return 1;

	case PSC_RESERVED_KEY:
		// skip special byte
		val = (~(*ptrSrc++) & 0x000000FF);
		// fall through intentional

	default:
		*checksum ^= (val << shift);
		*((*_ptrDest)++) = val;
	}
	*_ptrSrc = ptrSrc;

	return 0;
}

/*!
*   @brief Unpackages data in following order: 
*   1.) removes special characters 
*   2.) validates checksum 
*   3.) extracts data structure.
* 
*	@return 0 on success.  -1 on failure.
*/
int decodeBinaryPacket(com_manager_t* cmInstance, int pHandle, packet_t* pkt, unsigned char* pbuf, int pbufSize)
{
	// before we even get in this method, we can be assured that pbuf starts with a packet start byte and ends with a packet end byte
	// all other data can potentially be garbage
	if (pbufSize < 8)
	{
		// corrupt data
		cmInstance->status[pHandle].readCounter += 32;
		return -1;
	}

	// decode the body and calculate checksum
	uint8_t* ptrSrc = pbuf;
	uint8_t* ptrDest = (uint8_t*)&pkt->hdr;
	uint8_t* ptrSrcEnd = pbuf + pbufSize;
	packet_ftr_t ftr;
	uint32_t checkSumValue = 0x0000FF00; // the packet start byte begins the checksum
	uint32_t actualCheckSumValue;
	uint32_t shifter = 8;
	decodeBinaryPacketFooter(&ftr, &ptrSrcEnd, &actualCheckSumValue);

	// start packet byte
	*ptrDest++ = *ptrSrc++;

	// packet id - no shift
	if (decodeBinaryPacketByte(cmInstance, pHandle, &ptrSrc, &ptrDest, &checkSumValue, 0))
	{
		return -1;
	}

	// packet counter - shift 8
	if (decodeBinaryPacketByte(cmInstance, pHandle, &ptrSrc, &ptrDest, &checkSumValue, 8))
	{
		return -1;
	}

	// packet flags - no shift
	if (decodeBinaryPacketByte(cmInstance, pHandle, &ptrSrc, &ptrDest, &checkSumValue, 0))
	{
		return -1;
	}

	// decode the body - start shift 8
	ptrDest = pkt->body.ptr;
	while (ptrSrc < ptrSrcEnd)
	{
		if (decodeBinaryPacketByte(cmInstance, pHandle, &ptrSrc, &ptrDest, &checkSumValue, shifter))
		{
			return -1;
		}
		shifter ^= 8;
	}

	if (actualCheckSumValue != checkSumValue)
	{
		// corrupt data
		cmInstance->status[pHandle].readCounter += 32;
		cmInstance->status[pHandle].startByte = 0;
		return -1;
	}

	pkt->body.size = (uint32_t)(ptrDest - pkt->body.ptr);
	if (pkt->body.size > MAX_PKT_BODY_SIZE)
	{
		// corrupt data
		cmInstance->status[pHandle].readCounter += 32;
		cmInstance->status[pHandle].startByte = 0;
		return -1;
	}

	// if the endianness of the packet doesn't match our CPU, we need to flip the data so it will be correct for our CPU architecture
	if (pkt->body.size != 0 && (pkt->hdr.flags & CM_PKT_FLAGS_ENDIANNESS_MASK) != IS_LITTLE_ENDIAN)
	{
		if (pkt->hdr.flags & CM_PKT_FLAGS_RAW_DATA_NO_SWAP)
		{
			// Just swap the data header
			flipEndianess32(pkt->body.ptr, sizeof(p_data_hdr_t));
		}
		else
		{
			// Swap entire packet body (data header and data)
			flipEndianess32(pkt->body.ptr, pkt->body.size);			
		}
	}

	return 0;
}

int processAsciiPacket(com_manager_t* cmInstance, int pHandle, unsigned char* data, int dataLength)
{
	if (processAsciiRxPacket(cmInstance, pHandle, data, dataLength))
	{
		// Error parsing packet
		cmInstance->status[pHandle].rxError = -1;
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
	if (!decodeBinaryPacket(cmInstance, pHandle, &pkt, data, dataLength))
	{
		// bit index 2 is whether another packet is available that is related to this packet
		*additionalDataAvailable = pkt.hdr.flags & CM_PKT_FLAGS_MORE_DATA_AVAILABLE;

		if (!processBinaryRxPacket(cmInstance, pHandle, &pkt, *additionalDataAvailable))
		{
			return 0;
		}
	}

	// Error parsing packet
	cmInstance->status[pHandle].rxError = -1;
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
		{
			if (cmInstance->passThroughHandler != 0)
			{
				cmInstance->passThroughHandler(cmInstance, COM_MANAGER_PASS_THROUGH_UBLOX, pHandle, ringBuffer->buf + passThroughIndex, byteCount);
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

//////////////////////////////////////////////////////////////////////////
//  Packet Retry
//////////////////////////////////////////////////////////////////////////

/*!
*   @brief stepPacketRetry - Resend the ensured packets after the ENSURE_RETRY_COUNT
*   period if the expected response was not received.
*/
void stepPacketRetry(com_manager_t* cmInstance)
{
	int i;
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

/*!
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
packet_t* registerPacketRetry(com_manager_t* cmInstance, int pHandle, pkt_info_byte_t pid, unsigned char data[], unsigned int dataSize)
{
	int i;
	ensured_pkt_t *ePkt = 0;
	static int iLast;
	unsigned char searching = 1;

#if ENABLE_FILTER_DUPLICATE_PACKETS

#if ENABLE_FILTER_DUPLICATE_PACKETS_MATCH_ALL_CHARACTERS

	int j;

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

			case PID_STOP_ALL_BROADCASTS:
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
	if (searching)
	{
		if (++iLast >= cmInstance->maxEnsuredPackets)
		{
			iLast = 0;
		}
		ePkt = &(cmInstance->ensuredPackets[iLast]);
	}
	else
	{
		iLast = i;
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

/*!
*   @brief Update packet retry.  If the specific data requested or acknowledge 
*   is received, the retry list is updated as to no continue to resend the 
*   corresponding message.
*
*	@param[in] *pkt        Pointer to pkt buffer.
*/
void updatePacketRetryData(com_manager_t* cmInstance, packet_t *pkt)
{
	int i;
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
	int i;
	ensured_pkt_t *ePkt;
	p_ack_t *ack;
	pkt_info_byte_t ackInfo;

	ack = (p_ack_t*)(pkt->body.ptr);
	ackInfo = (pkt_info_byte_t)(ack->hdr.pktInfo);

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
			case PID_STOP_ALL_BROADCASTS: // No body ID available
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

char copyDataPToStructP(void *sptr, const p_data_t *data, const unsigned int maxsize)
{
    if ((data->hdr.size + data->hdr.offset) <= maxsize)
    {
        memcpy((uint8_t*)sptr + data->hdr.offset, data->buf, data->hdr.size);
        return 0;
    }
    else
    {
        return -1;
    }
}

/*! Copies packet data into a data structure.  Returns 0 on success, -1 on failure. */
char copyDataPToStructP2(void *sptr, const p_data_hdr_t *dataHdr, const uint8_t *dataBuf, const unsigned int maxsize)
{
    if ((dataHdr->size + dataHdr->offset) <= maxsize)
    {
        memcpy((uint8_t*)sptr + dataHdr->offset, dataBuf, dataHdr->size);
        return 0;
    }
    else
    {
        return -1;
    }
}


// 0 on success, -1 on failure
int validateBaudRate( int baudrate )
{
	// Valid baudrates for InertialSense hardware
	switch (baudrate)
	{
		case IS_BAUDRATE_115200:
		case IS_BAUDRATE_230400:
		case IS_BAUDRATE_460800:
		case IS_BAUDRATE_921600:
		case IS_BAUDRATE_3000000:
		return 0;	// success
	}

	return -1;	// failure
}
