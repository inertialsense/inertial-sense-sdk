/*
MIT LICENSE

Copyright (c) 2014-2024 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include <string.h>
#include <stdlib.h>
#include <vector>

#include "com_manager.h"
#include "serialPort.h"

#ifdef IMX_5
#include "globals.h"
#endif


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

static com_manager_t s_cm = {0};

int initComManagerInstanceInternal
(
    com_manager_t* cmInstance,
    port_handle_t port,
    int stepPeriodMilliseconds,
    pfnComManagerRead portReadFnc,
    pfnIsCommPortWrite portWriteFnc,
    pfnComManagerSendBufferAvailableBytes txFreeFnc,
    pfnComManagerPostRead pstRxFnc,
    pfnComManagerPostAck pstAckFnc,
    pfnComManagerDisableBroadcasts disableBcastFnc,
    std::array<broadcast_msg_t, MAX_NUM_BCAST_MSGS>* buffers   //! was: com_manager_init_t *buffers,
);

// int processAsciiRxPacket(com_manager_t* cmInstance, port_handle_t port, unsigned char* start, int count);
// void parseAsciiPacket(com_manager_t* cmInstance, port_handle_t port, unsigned char* buf, int count);
int processBinaryRxPacket(com_manager_t* cmInstance, port_handle_t port, packet_t *pkt);
void enableBroadcastMsg(com_manager_t* cmInstance, broadcast_msg_t *msg, int periodMultiple);
void disableBroadcastMsg(com_manager_t* cmInstance, broadcast_msg_t *msg);
void disableDidBroadcast(com_manager_t* cmInstance, port_handle_t port, uint16_t did);
int sendDataPacket(com_manager_t* cmInstance, port_handle_t port, packet_t *pkt);
void sendAck(com_manager_t* cmInstance, port_handle_t port, packet_t *pkt, uint8_t pTypeFlags);
int findAsciiMessage(const void * a, const void * b);
int asciiMessageCompare(const void* elem1, const void* elem2);
void stepComManagerSendMessages(void);
void stepComManagerSendMessagesInstance(CMHANDLE cmInstance);

static int comManagerStepRxInstanceHandler(com_manager_t* cmInstance, comm_port_t* port, protocol_type_t ptype);

CMHANDLE comManagerGetGlobal(void) { return &s_cm; }

int comManagerInit(
	port_handle_t port,
    int stepPeriodMilliseconds,
    pfnComManagerRead portReadFnc,
    pfnIsCommPortWrite portWriteFnc,
    pfnComManagerSendBufferAvailableBytes txFreeFnc,
    pfnComManagerPostRead pstRxFnc,
    pfnComManagerPostAck pstAckFnc,
    pfnComManagerDisableBroadcasts disableBcastFnc,
    std::array<broadcast_msg_t, MAX_NUM_BCAST_MSGS>* buffers)   //! was: com_manager_init_t *buffers,
{
    return initComManagerInstanceInternal(
        &s_cm,
        port,
        stepPeriodMilliseconds,
        portReadFnc, 
        portWriteFnc, 
        txFreeFnc, 
        pstRxFnc, 
        pstAckFnc, 
        disableBcastFnc, 
        buffers);
}

int comManagerInitInstance
(	CMHANDLE cmHandle,
    port_handle_t port,
    int stepPeriodMilliseconds,
    pfnComManagerRead portReadFnc,
    pfnIsCommPortWrite portWriteFnc,
    pfnComManagerSendBufferAvailableBytes txFreeFnc,
    pfnComManagerPostRead pstRxFnc,
    pfnComManagerPostAck pstAckFnc,
    pfnComManagerDisableBroadcasts disableBcastFnc,
    std::array<broadcast_msg_t, MAX_NUM_BCAST_MSGS>* buffers)   //! was: com_manager_init_t *buffers,
{
    int result = 0;

    com_manager_t* cmInstance = (com_manager_t*)cmHandle;
    if (cmInstance != 0)
    {
        memset((void *)cmInstance, 0, sizeof(com_manager_t));
        result = initComManagerInstanceInternal(
            cmInstance,
            port,
            stepPeriodMilliseconds,
            portReadFnc, 
            portWriteFnc, 
            txFreeFnc, 
            pstRxFnc, 
            pstAckFnc, 
            disableBcastFnc,
            buffers);
    }
    return result;
}

int initComManagerInstanceInternal
(	com_manager_t* cmInstance,
    port_handle_t port,
    int stepPeriodMilliseconds,
    pfnComManagerRead portReadFnc,
    pfnIsCommPortWrite portWriteFnc,
    pfnComManagerSendBufferAvailableBytes txFreeFnc,
    pfnComManagerPostRead pstRxFnc,
    pfnComManagerPostAck pstAckFnc,
    pfnComManagerDisableBroadcasts disableBcastFnc,
    std::array<broadcast_msg_t, MAX_NUM_BCAST_MSGS>* buffers)   //! was: com_manager_init_t *buffers,
{
    if ((port == NULL) || (buffers == NULL))
    {
        return -1;
    }

    // assign new variables
    cmInstance->portRead = portReadFnc;
    cmInstance->portWrite = portWriteFnc;
    cmInstance->txFree = txFreeFnc;
    cmInstance->pstRxFnc = pstRxFnc;
    cmInstance->pstAckFnc = pstAckFnc;
    cmInstance->disableBcastFnc = disableBcastFnc;
    // cmInstance->numPorts = numPorts;
    cmInstance->stepPeriodMilliseconds = stepPeriodMilliseconds;
    cmInstance->cmMsgHandlerNmea = NULL;
    cmInstance->cmMsgHandlerUblox = NULL;
    cmInstance->cmMsgHandlerRtcm3 = NULL;

    // Buffer: message broadcasts
    cmInstance->broadcastMessages = buffers;

    // Initialize IScomm instance, for serial reads / writes
    comm_port_t* comm = (comm_port_t*)port;
    is_comm_init(&(comm->comm), comm->buffer, sizeof(comm->buffer));

#if ENABLE_PACKET_CONTINUATION
    // Packet data continuation
    memset(&(port->con), 0, MEMBERSIZE(com_manager_port_t,con));
#endif

    cmInstance->ports.push_back(port);

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
    comManagerRegisterInstance(&s_cm, did, txFnc, pstRxFnc, txDataPtr, rxDataPtr, size, pktFlags);
}

void comManagerRegisterInstance(CMHANDLE cmInstance_, uint16_t did, pfnComManagerPreSend txFnc, pfnComManagerPostRead pstRxFnc, const void* txDataPtr, void* rxDataPtr, uint16_t size, uint8_t pktFlags)
{
    com_manager_t* cmInstance = (com_manager_t*)cmInstance_;

    // Validate ID and data pointer
    if (did >= DID_COUNT)
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

void comManagerStep()
{
    comManagerStepRxInstance(&s_cm, 0);
    comManagerStepTxInstance(&s_cm);
}

void comManagerStepTimeout(uint32_t timeMs)
{
    comManagerStepRxInstance(&s_cm, timeMs);
    comManagerStepTxInstance(&s_cm);
}

void comManagerStepInstance(CMHANDLE cmInstance_)
{
    com_manager_t* cmInstance = (com_manager_t*)cmInstance_;
    comManagerStepRxInstance(cmInstance, 0);
    comManagerStepTxInstance(cmInstance);
}

void comManagerStepRxInstance(CMHANDLE cmInstance_, uint32_t timeMs)
{
    com_manager_t* cmInstance = (com_manager_t*)cmInstance_;
    if (!cmInstance->portRead)
    {
        return;
    }
        
    for (auto port : cmInstance->ports)
    {
        // com_manager_port_t *cmPort = &(cmInstance->ports[port]);
        is_comm_instance_t *comm = &((comm_port_t *)port)->comm;
        protocol_type_t ptype = _PTYPE_NONE;

        // Read data directly into comm buffer
        // Here there lie dragons - is_comm_free() modifies comm->rxBuf pointers, so make sure you call here first!!
        int free_size = is_comm_free(comm);
        int n = cmInstance->portRead(port, comm->rxBuf.tail, free_size);
        if (n > 0)
        {
            // Update comm buffer tail pointer
            comm->rxBuf.tail += n;

            // Search comm buffer for valid packets
            while ((ptype = is_comm_parse_timeout(comm, timeMs)) != _PTYPE_NONE)
            {
                int error = comManagerStepRxInstanceHandler(cmInstance, (comm_port_t*)port, ptype);
                if(error == CM_ERROR_FORWARD_OVERRUN) 
                {
                    break;	// Stop parsing and continue in outer loop
                }
            }
        }
    }
}

static int comManagerStepRxInstanceHandler(com_manager_t* cmInstance, comm_port_t* port, protocol_type_t ptype)
{
    int error = 0;
    uint8_t *data = port->comm.rxPkt.data.ptr + port->comm.rxPkt.offset;
    uint16_t size = port->comm.rxPkt.data.size;

    switch (ptype)
    {
    case _PTYPE_PARSE_ERROR:
        if (cmInstance->cmMsgHandlerError)
        {
            cmInstance->cmMsgHandlerError((port_handle_t)port, &port->comm);
        }
        error = 1;
        break;

    case _PTYPE_INERTIAL_SENSE_DATA:
    case _PTYPE_INERTIAL_SENSE_CMD:
        error = processBinaryRxPacket(cmInstance, (port_handle_t)port, &(port->comm.rxPkt));
        break;

    case _PTYPE_UBLOX:
        if (cmInstance->cmMsgHandlerUblox)
        {
            error = cmInstance->cmMsgHandlerUblox((port_handle_t)port, data, size);
        }
        break;

    case _PTYPE_RTCM3:
        if (cmInstance->cmMsgHandlerRtcm3)
        {
            error = cmInstance->cmMsgHandlerRtcm3((port_handle_t)port, data, size);
        }
        break;

    case _PTYPE_NMEA:
        if (cmInstance->cmMsgHandlerNmea)
        {
            error = cmInstance->cmMsgHandlerNmea((port_handle_t)port, data, size);
        }
        break;

    case _PTYPE_SPARTN:
        if (cmInstance->cmMsgHandlerSpartn)
        {
            error = cmInstance->cmMsgHandlerSpartn((port_handle_t)port, data, size);
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
    stepComManagerSendMessagesInstance(&s_cm);
}

__attribute__((optimize("O0")))
void stepComManagerSendMessagesInstance(CMHANDLE cmInstance_)
{
    com_manager_t* cmInstance = static_cast<com_manager_t *>(cmInstance_);
    
    // Send data (if necessary)
    // for (broadcast_msg_t* bcPtr = cmInstance->broadcastMessages, *ptrEnd = (cmInstance->broadcastMessages + MAX_NUM_BCAST_MSGS); bcPtr < ptrEnd; bcPtr++)
    for (auto& bc : *(cmInstance->broadcastMessages))
    {
        // If send buffer does not have space, exit out
        if (cmInstance->txFree && (bc.pkt.size > (uint32_t)cmInstance->txFree(bc.port)))
        {
            break;
        }
        // Send once and remove from message queue
        else if (bc.period == MSG_PERIOD_SEND_ONCE)
        {
            sendDataPacket(cmInstance, bc.port, &(bc.pkt));
            disableBroadcastMsg(cmInstance, &bc);
        }
        // Broadcast messages
        else if (bc.period > 0)
        {
            // Check if counter has expired
            if (++bc.counter >= bc.period)
            {
                bc.counter = 0;    // reset counter

                // Prep data if callback exists
                unsigned int id = bc.pkt.hdr.id;
                int sendData = 1;
                if (id<DID_COUNT && cmInstance->regData[id].preTxFnc)
                {					
                    sendData = cmInstance->regData[id].preTxFnc(bc.port, &bc.pkt.dataHdr);
                }
                if (sendData)
                {
                    sendDataPacket(cmInstance, bc.port, &(bc.pkt));
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
    pfnComManagerGenMsgHandler handlerSpartn,
    pfnComManagerParseErrorHandler handlerError)
{
    comManagerSetCallbacksInstance(&s_cm, handlerRmc, handlerAscii, handlerUblox, handlerRtcm3, handlerSpartn, handlerError);
}

void comManagerSetCallbacksInstance(CMHANDLE cmInstance, 
    pfnComManagerAsapMsg handlerRmc,
    pfnComManagerGenMsgHandler handlerAscii,
    pfnComManagerGenMsgHandler handlerUblox,
    pfnComManagerGenMsgHandler handlerRtcm3,
    pfnComManagerGenMsgHandler handlerSpartn,
    pfnComManagerParseErrorHandler handlerError)
{
    if (cmInstance != 0)
    {
        ((com_manager_t*)cmInstance)->cmMsgHandlerRmc = handlerRmc;
        ((com_manager_t*)cmInstance)->cmMsgHandlerNmea = handlerAscii;
        ((com_manager_t*)cmInstance)->cmMsgHandlerUblox = handlerUblox;
        ((com_manager_t*)cmInstance)->cmMsgHandlerRtcm3 = handlerRtcm3;
        ((com_manager_t*)cmInstance)->cmMsgHandlerSpartn = handlerSpartn;
        ((com_manager_t*)cmInstance)->cmMsgHandlerError = handlerError;        
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

is_comm_instance_t* comManagerGetIsComm(port_handle_t port)
{
    return comManagerGetIsCommInstance(&s_cm, port);
}

is_comm_instance_t* comManagerGetIsCommInstance(CMHANDLE cmInstance, port_handle_t port)
{
    if (port == NULL)
        return NULL;

    return &((comm_port_t*)port)->comm;
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
void comManagerGetData(port_handle_t port, uint16_t did, uint16_t size, uint16_t offset, uint16_t period)
{
    comManagerGetDataInstance(&s_cm, port, did, size, offset, period);
}

void comManagerGetDataInstance(CMHANDLE cmInstance, port_handle_t port, uint16_t did, uint16_t size, uint16_t offset, uint16_t period)
{
    // Create and Send request packet
    p_data_get_t get;
    get.id = did;
    get.offset = offset;
    get.size = size;
    get.period = period;

    comManagerSendInstance(cmInstance, port, PKT_TYPE_GET_DATA, &get, 0, sizeof(get), 0);
}

void comManagerGetDataRmc(port_handle_t port, uint64_t rmcBits, uint32_t rmcOptions)
{
    comManagerGetDataRmcInstance(&s_cm, port, rmcBits, rmcOptions);
}

void comManagerGetDataRmcInstance(CMHANDLE cmInstance, port_handle_t port, uint64_t rmcBits, uint32_t rmcOptions)
{
    rmc_t rmc;
    rmc.bits = rmcBits;
    rmc.options = rmcOptions;

    comManagerSendDataInstance(cmInstance, port, &rmc, DID_RMC, sizeof(rmc_t), 0);
}

int comManagerSendData(port_handle_t port, void *data, uint16_t did, uint16_t size, uint16_t offset)
{	
    return comManagerSendDataInstance(&s_cm, port, data, did, size, offset);
}

int comManagerSendDataInstance(CMHANDLE cmInstance, port_handle_t port, void* data, uint16_t did, uint16_t size, uint16_t offset)
{
    return comManagerSendInstance(cmInstance, port, PKT_TYPE_SET_DATA, data, did, size, offset);
}

int comManagerSendDataNoAck(port_handle_t port, void *data, uint16_t did, uint16_t size, uint16_t offset)
{
    return comManagerSendDataNoAckInstance(&s_cm, port, data, did, size, offset);
}

int comManagerSendDataNoAckInstance(CMHANDLE cmInstance, port_handle_t port, void *data, uint16_t did, uint16_t size, uint16_t offset)
{
    return comManagerSendInstance((com_manager_t*)cmInstance, port, PKT_TYPE_DATA, data, did, size, offset);
}

int comManagerSendRawData(port_handle_t port, void *data, uint16_t did, uint16_t size, uint16_t offset)
{
    return comManagerSendRawDataInstance(&s_cm, port, data, did, size, offset);
}

int comManagerSendRawDataInstance(CMHANDLE cmInstance, port_handle_t port, void* data, uint16_t did, uint16_t size, uint16_t offset)
{
    return comManagerSendInstance((com_manager_t*)cmInstance, port, PKT_TYPE_SET_DATA, data, did, size, offset);
}

int comManagerSendRaw(port_handle_t port, void *dataPtr, int dataSize)
{
    return comManagerSendRawInstance(&s_cm, port, dataPtr, dataSize);
}

// Returns 0 on success, -1 on failure.
int comManagerSendRawInstance(CMHANDLE cmInstance, port_handle_t port, void* dataPtr, int dataSize)
{
    pfnIsCommPortWrite writeCallback = ((com_manager_t*)cmInstance)->portWrite;
    if (writeCallback == 0){ return 0; }
    return (writeCallback(port, static_cast<const uint8_t *>(dataPtr), dataSize) ? 0 : -1);
}

int comManagerDisableData(port_handle_t port, uint16_t did)
{
    return comManagerDisableDataInstance(&s_cm, port, did);
}

int comManagerDisableDataInstance(CMHANDLE cmInstance, port_handle_t port, uint16_t did)
{
    return comManagerSendInstance(cmInstance, port, PKT_TYPE_STOP_DID_BROADCAST, NULL, did, 0, 0);
}

int comManagerSend(port_handle_t pHandle, uint8_t pFlags, void* data, uint16_t did, uint16_t size, uint16_t offset)
{
    return comManagerSendInstance(&s_cm, pHandle, pFlags, data, did, size, offset);
}

int comManagerSendInstance(CMHANDLE cmInstance, port_handle_t port, uint8_t pFlags, void *data, uint16_t did, uint16_t size, uint16_t offset)
{
    com_manager_t *cm = (com_manager_t*)cmInstance;
    int bytes = is_comm_write(cm->portWrite, port, pFlags, did, size, offset, data);
    return (bytes < 0) ? -1 : 0;    // Return 0 on success, -1 on failure
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
int processBinaryRxPacket(com_manager_t* cmInstance, port_handle_t port, packet_t *pkt)
{
    packet_hdr_t        *hdr = &(pkt->hdr);
    registered_data_t   *regData = NULL;
    uint8_t             ptype = (uint8_t)(pkt->hdr.flags&PKT_TYPE_MASK);

    switch (ptype)
    {
    default:    // Data ID Unknown
        return -1;

    case PKT_TYPE_SET_DATA:
    case PKT_TYPE_DATA:
    {		
        // Validate Data
        if (hdr->id >= DID_COUNT || hdr->payloadSize == 0)
        {
            return -1;
        }

        regData = &(cmInstance->regData[hdr->id]);

        p_data_t data;
        data.hdr.id = pkt->dataHdr.id;
        data.hdr.offset = pkt->offset;
        data.hdr.size = pkt->data.size;
        data.ptr = pkt->data.ptr;

        // Validate and constrain Rx data size to fit within local data struct
        if (regData->dataSet.size && (uint32_t)(data.hdr.offset + data.hdr.size) > regData->dataSet.size)
        {
            // trim the size down so it fits
            uint16_t size = (int)(regData->dataSet.size - data.hdr.offset);
            if (size < 4)
            {
                // we are completely out of bounds, we cannot process this message at all
                // the minimum data struct size is 4 bytes
                return -1;
            }

            // Update Rx data size
            data.hdr.size = _MIN(data.hdr.size, (uint8_t)size);
        }

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

        if (regData)
        {
            // Write to data structure if it was registered
            if (regData->dataSet.rxPtr)
            {
                copyDataPToStructP(regData->dataSet.rxPtr, &data, regData->dataSet.size);
            }

            // Call data specific callback after data has been received
            if (regData->pstRxFnc)
            {
                regData->pstRxFnc(port, &data);
            }
        }

        // Call general/global callback
        if (cmInstance->pstRxFnc)
        {
            cmInstance->pstRxFnc(port, &data);
        }

#if ENABLE_PACKET_CONTINUATION

        // Clear dataset consolidation
        con->hdr.id = con->hdr.size = con->hdr.offset = 0;

#endif

        // Reply w/ ACK for PKT_TYPE_SET_DATA
        if (ptype == PKT_TYPE_SET_DATA)
        {
            sendAck(cmInstance, port, pkt, PKT_TYPE_ACK);
        }
    }
        break;

    case PKT_TYPE_GET_DATA:
        #ifdef IMX_5
        // Forward to gpx
        if(IO_CONFIG_GPS1_TYPE(g_nvmFlashCfg->ioConfig) == IO_CONFIG_GPS_TYPE_GPX && 
            (((p_data_get_t*)(pkt->data.ptr))->id == DID_RTK_DEBUG || (
            (((p_data_get_t*)(pkt->data.ptr))->id >= DID_GPX_FIRST) && 
            (((p_data_get_t*)(pkt->data.ptr))->id <= DID_GPX_LAST))))
        {
            comManagerGetDataInstance(comManagerGetGlobal(), COM0_PORT_NUM, ((p_data_get_t*)(pkt->data.ptr))->id, ((p_data_get_t*)(pkt->data.ptr))->size, ((p_data_get_t*)(pkt->data.ptr))->offset, ((p_data_get_t*)(pkt->data.ptr))->period);
        }
        
        #endif
        if (comManagerGetDataRequestInstance(cmInstance, port, (p_data_get_t*)(pkt->data.ptr)))
        {
            sendAck(cmInstance, port, pkt, PKT_TYPE_NACK);
        }
        break;

    case PKT_TYPE_STOP_BROADCASTS_ALL_PORTS:
        comManagerDisableBroadcastsInstance(cmInstance, NULL);  // all ports

        // Call disable broadcasts callback if exists
        if (cmInstance->disableBcastFnc)
        {
            cmInstance->disableBcastFnc(NULL);  // all ports
        }
        sendAck(cmInstance, port, pkt, PKT_TYPE_ACK);
        break;

    case PKT_TYPE_STOP_BROADCASTS_CURRENT_PORT:
        comManagerDisableBroadcastsInstance(cmInstance, port);

        // Call disable broadcasts callback if exists
        if (cmInstance->disableBcastFnc)
        {
            cmInstance->disableBcastFnc(port);
        }
        sendAck(cmInstance, port, pkt, PKT_TYPE_ACK);
        break;

    case PKT_TYPE_STOP_DID_BROADCAST:
        disableDidBroadcast(cmInstance, port, pkt->hdr.id);
        break;

    case PKT_TYPE_NACK:
    case PKT_TYPE_ACK:
        // Call general ack callback
        if (cmInstance->pstAckFnc)
        {
            cmInstance->pstAckFnc(port, (p_ack_t*)(pkt->data.ptr), ptype);
        }
        break;
    }

    // Success
    return 0;
}

bufTxRxPtr_t* comManagerGetRegisteredDataInfo(uint16_t did)
{
    return comManagerGetRegisteredDataInfoInstance(&s_cm, did);
}

bufTxRxPtr_t* comManagerGetRegisteredDataInfoInstance(CMHANDLE _cmInstance, uint16_t did)
{
    if (did < DID_COUNT)
    {
        com_manager_t* cmInstance = (com_manager_t*)_cmInstance;
        return &cmInstance->regData[did].dataSet;
    }

    return 0;
}

// 0 on success. -1 on failure.
int comManagerGetDataRequest(port_handle_t port, p_data_get_t* req)
{
    return comManagerGetDataRequestInstance(&s_cm, port, req);
}

__attribute__((optimize("O0")))
int comManagerGetDataRequestInstance(CMHANDLE _cmInstance, port_handle_t port, p_data_get_t* req)
{
    com_manager_t* cmInstance = (com_manager_t*)_cmInstance;
    broadcast_msg_t* msg = NULL;

    // Validate the request
    if (req->id >= DID_COUNT)
    {
        // invalid data id
        return -1;
    }
    // Call RealtimeMessageController (RMC) handler
    else if (cmInstance->cmMsgHandlerRmc && (cmInstance->cmMsgHandlerRmc(port, req) == 0))
    {
        // Don't allow comManager broadcasts for messages handled by RealtimeMessageController. 
        return 0;
    }
    // if size is 0 and offset is 0, set size to full data struct size
    else if (req->size == 0 && req->offset == 0 && req->id < DID_COUNT)
    {
        req->size = cmInstance->regData[req->id].dataSet.size;
    }
    
    if (req->size == 0)
    {	// Don't respond if data size is zero. Return zero to prevent sending NACK.
        return 0;
    }

    // Copy reference to source data
    bufTxRxPtr_t* dataSetPtr = &cmInstance->regData[req->id].dataSet;

    if ((uint32_t)(req->offset + req->size) > dataSetPtr->size)
    {
        req->offset = 0;
        req->size = dataSetPtr->size;
    }

    // Search for matching message (i.e. matches pHandle, id, size, and offset)...
    // for (broadcast_msg_t* bcPtr = cmInstance->broadcastMessages, *ptrEnd = (cmInstance->broadcastMessages + MAX_NUM_BCAST_MSGS); bcPtr < ptrEnd; bcPtr++)
    for (auto& bc : *(cmInstance->broadcastMessages))
    {
        if (bc.port == port && bc.pkt.hdr.id == req->id && bc.pkt.hdr.payloadSize == req->size && bc.pkt.offset == req->offset)
        {
            msg = &bc;
            break;
        }
    }

    // otherwise use the first available (period=0) message.
    if (msg == 0)
    {
        // for (broadcast_msg_t* bcPtr = cmInstance->broadcastMessages, *ptrEnd = (cmInstance->broadcastMessages + MAX_NUM_BCAST_MSGS); bcPtr < ptrEnd; bcPtr++)
        for (auto& bc : *(cmInstance->broadcastMessages))
        {
            if (bc.period <= MSG_PERIOD_DISABLED)
            {
                msg = &bc;
                break;
            }
        }

        if (msg == 0)
        {
            // use last slot, force overwrite
            // FIXME: I think we want to allocate a new broadcast slot, unless we are reached our maximum allowed,
            //    in which case, we'll either overwrite the last, or pop() the first
            msg = &(*cmInstance->broadcastMessages)[0]; //  + MAX_NUM_BCAST_MSGS - 1;
        }
    }

    msg->port = port;
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
    if (req->id < DID_COUNT)
    {
        if (cmInstance->regData[req->id].preTxFnc)
        {
            sendData = cmInstance->regData[req->id].preTxFnc(port, &(pkt->dataHdr));
        }
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
        if (cmInstance->txFree == 0 || pkt->size <= (uint32_t)cmInstance->txFree(port))
        {
            if (sendData)
            {
                sendDataPacket(cmInstance, port, &(msg->pkt));
            }
        }

        // Enable broadcast message
        enableBroadcastMsg(cmInstance, msg, req->period);
    }
    else
    {
        // ***  Request Single  ***
        // Send data immediately if possible
        if (cmInstance->txFree == 0 || pkt->size <= (uint32_t)cmInstance->txFree(port))
        {
            if (sendData)
            {
                sendDataPacket(cmInstance, port, &(msg->pkt));
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

void comManagerDisableBroadcasts(port_handle_t port)
{
    comManagerDisableBroadcastsInstance(&s_cm, port);
}

void comManagerDisableBroadcastsInstance(CMHANDLE cmInstance_, port_handle_t port)
{
    com_manager_t* cmInstance = (com_manager_t*)cmInstance_;
    // for (broadcast_msg_t* bcPtr = cmInstance->broadcastMessages, *ptrEnd = (cmInstance->broadcastMessages + MAX_NUM_BCAST_MSGS); bcPtr < ptrEnd; bcPtr++)
    for (auto& bc : *(cmInstance->broadcastMessages))
    {
        if ((port == NULL) || bc.port == port)
        {
            bc.period = MSG_PERIOD_DISABLED;
        }
    }
}

void disableDidBroadcast(com_manager_t* cmInstance, port_handle_t port, uint16_t did)
{
    // for (broadcast_msg_t* bcPtr = cmInstance->broadcastMessages, *ptrEnd = (cmInstance->broadcastMessages + MAX_NUM_BCAST_MSGS); bcPtr < ptrEnd; bcPtr++)
    for (auto& bc : *(cmInstance->broadcastMessages))
    {
        if (((port == NULL) || (port == bc.port)) && bc.pkt.hdr.id == did)
        {
            bc.period = MSG_PERIOD_DISABLED;
        }
    }
    
    // Call global broadcast handler to disable message control
    if (cmInstance->cmMsgHandlerRmc)
    {
        p_data_get_t req;
        req.id = did;
        req.size = 0;
        req.offset = 0;
        req.period = 0;
        cmInstance->cmMsgHandlerRmc(port, &req);
    }
}

// Consolidate this with sendPacket() so that we break up packets into multiples that fit our buffer size.  Returns 0 on success, -1 on failure.
int sendDataPacket(com_manager_t* cm, port_handle_t port, packet_t* pkt)
{
    int bytes = is_comm_write_isb_precomp_to_port(cm->portWrite, port, pkt);

    // Return 0 on success, -1 on failure
    return (bytes < 0) ? -1 : 0;
}

void sendAck(com_manager_t* cmInstance, port_handle_t port, packet_t *pkt, uint8_t pTypeFlags)
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

    comManagerSendInstance(cmInstance, port, pTypeFlags, &ack, 0, sizeof(ack), 0);
}

int comManagerValidateBaudRate(unsigned int baudRate)
{
    // Valid baudrate for InertialSense hardware
    return validateBaudRate(baudRate);
}

