/*
MIT LICENSE

Copyright (c) 2014-2024 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include <string.h>
#include <stdlib.h>

#include <algorithm>
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
#define MSG_PERIOD_SEND_ONCE        -1
#define MSG_PERIOD_DISABLED         0

ISComManager s_cm;

CMHANDLE comManagerGetGlobal(void) { return &s_cm; }

int ISComManager::init(
        std::unordered_set<port_handle_t>* portSet,
        int stepPeriodMillis,
        pfnComManagerPostRead pstRxFncCb,
        pfnComManagerPostAck pstAckFncCb,
        pfnComManagerRmcHandler rmcHandler,
        pfnComManagerDisableBroadcasts disableBcastFncCb,
        broadcast_msg_array_t* bcastBuffers)
{
    pstRxFnc = pstRxFncCb;
    pstAckFnc = pstAckFncCb;
    disableBcastFnc = disableBcastFncCb;
    stepPeriodMilliseconds = stepPeriodMillis;

    cmMsgHandlerRmc = rmcHandler;

    // Buffer: message broadcasts
    broadcastMessages = bcastBuffers;

    defaultCbs = {};
    defaultCbs.all = comManagerProcessBinaryRxPacket;

    if (!portSet) portSet = new std::unordered_set<port_handle_t>();
    ports = portSet;

    return 0;
}

int comManagerInit(
        std::unordered_set<port_handle_t>* portSet,
        int stepPeriodMilliseconds,
        pfnComManagerPostRead pstRxFnc,
        pfnComManagerPostAck pstAckFnc,
        pfnComManagerRmcHandler rmcHandler,
        pfnComManagerDisableBroadcasts disableBcastFnc,
        std::array<broadcast_msg_t, MAX_NUM_BCAST_MSGS>* buffers) {
    return s_cm.init(
            portSet,
            stepPeriodMilliseconds,
            pstRxFnc,
            pstAckFnc,
            rmcHandler,
            disableBcastFnc,
            buffers);
}

pfnIsCommGenMsgHandler comManagerRegisterProtocolHandler(int ptype, pfnIsCommGenMsgHandler cbHandler, port_handle_t port) {
    return s_cm.registerProtocolHandler(ptype, cbHandler, port);
}

/**
 *
 * @param port
 * @return
 */
bool ISComManager::registerPort(port_handle_t port, is_comm_callbacks_t* cbs) {
    if (!port)
        return false;

    is_comm_callbacks_t portCbs = defaultCbs;
    if (cbs) portCbs = *cbs; // override defaults

    // Initialize IScomm instance, for serial reads / writes
    if ((portType(port) & PORT_TYPE__COMM)) {
        comm_port_t* comm = COMM_PORT(port);

        is_comm_init(&(comm->comm), comm->buffer, sizeof(comm->buffer), portCbs.all);
        is_comm_register_port_callbacks(port, &portCbs);

#if ENABLE_PACKET_CONTINUATION
        // Packet data continuation
        memset(&(port->con), 0, MEMBERSIZE(com_manager_port_t,con));
#endif
    }

    if (ports)
        ports->insert(port);

    return true;
}

bool comManagerRegisterPort(port_handle_t port) {
    return s_cm.registerPort(port, NULL);
}

bool comManagerRegisterPort(port_handle_t port, is_comm_callbacks_t* cbs) {
    return s_cm.registerPort(port, cbs);
}

std::unordered_set<port_handle_t>& ISComManager::getPorts() {
    return *ports;
}

std::unordered_set<port_handle_t>& comManagerGetPorts() {
    return s_cm.getPorts();
}

bool ISComManager::removePort(port_handle_t port) {
    auto found = std::find(ports->begin(), ports->end(), port);
    if (found == ports->end())
        return false;

    if (port) {
        serialPortClose(port);
        // ports->erase(found);
    }

    return true;
}

bool comManagerRemovePort(port_handle_t port) {
    return s_cm.removePort(port);
}

bool ISComManager::removeAllPorts() {
//    for (auto port : ports) {
//        removePort(port);
//        // TODO delete (serial_port_s*)port;
//    }
    ports->clear();
    return true;
}

bool comManagerReleaseAllPorts() {
    return s_cm.removeAllPorts();
}


int asciiMessageCompare(const void* elem1, const void* elem2)
{
    asciiMessageMap_t* e1 = (asciiMessageMap_t*)elem1;
    asciiMessageMap_t* e2 = (asciiMessageMap_t*)elem2;

    return memcmp(e1->messageId, e2->messageId, 4);
}

void comManagerRegister(uint16_t did, pfnComManagerPreSend txFnc, pfnComManagerPostRead pstRxFnc, const void* txDataPtr, void* rxDataPtr, uint16_t size, uint8_t pktFlags)
{
    s_cm.registerDid(did, txFnc, pstRxFnc, txDataPtr, rxDataPtr, size, pktFlags);
}

void ISComManager::registerDid(uint16_t did, pfnComManagerPreSend txFnc, pfnComManagerPostRead pstRxFnc, const void* txDataPtr, void* rxDataPtr, uint16_t size, uint8_t pktFlags)
{
    // Validate ID and data pointer
    if (did >= DID_COUNT) {
        return;
    }

    auto& entry = didRegistrationMap[did];
    entry.dataSet.txPtr = (unsigned char*)txDataPtr;     // Pointer to data struct for Tx
    entry.dataSet.rxPtr = (unsigned char*)rxDataPtr;     // Pointer to data struct for Rx
    entry.dataSet.size = size;                           // Size of data struct
    entry.preTxFnc = txFnc;                              // Function called to update struct before data is sent
    entry.pstRxFnc = pstRxFnc;                           // Function called after data is received and struct is updated
    entry.pktFlags = pktFlags;                           // Packet flags
}


void comManagerStepTimeout(uint32_t timeMs) {
    s_cm.stepRx();
    s_cm.stepTx();
}

void ISComManager::stepTimeout(uint32_t timeMs)
{
    stepRx();
    stepTx();
}

void comManagerStep()
{
    s_cm.stepRx();
    s_cm.stepTx();
}

void ISComManager::step()
{
    stepRx();
    stepTx();
}

void comManagerStep(port_handle_t port)
{
    s_cm.stepRx(port);
    s_cm.stepTx();
}

void ISComManager::stepRx()
{
    if (!ports) return;  // nothing to do...

    for (port_handle_t port : *ports)
    {
        // Read data directly into comm buffer and call callback functions
        is_comm_port_parse_messages(port);
    }
}

void ISComManager::stepRx(port_handle_t port)
{
    if (!port) return;  // nothing to do...
    // Read data directly into comm buffer and call callback functions
    is_comm_port_parse_messages(port);
}

void ISComManager::stepTx()
{
    stepSendMessages();
}

void stepSendMessages(void)
{
    s_cm.stepSendMessages();
}

// __attribute__((optimize("O0")))
void ISComManager::stepSendMessages()
{
    if (broadcastMessages == NULL)
        return;

    // Send data (if necessary)
    for (auto& bc : *(broadcastMessages))
    {
        // If send buffer does not have space, exit out
        if (txFree && (bc.pkt.size > (uint32_t)txFree(bc.port)))
        {
            break;
        }
        // Send once and remove from message queue
        else if (bc.period == MSG_PERIOD_SEND_ONCE)
        {
            sendDataPacket(bc.port, &(bc.pkt));
            disableBroadcastMsg(&bc);
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
                if (id<DID_COUNT && didRegistrationMap[id].preTxFnc)
                {                    
                    sendData = didRegistrationMap[id].preTxFnc(bc.port, &bc.pkt.dataHdr);
                }
                if (sendData)
                {
                    sendDataPacket(bc.port, &(bc.pkt));
                }
            }
        }
    }
}

void ISComManager::assignUserPointer(void* userPointer)
{
    userPointer = userPointer;
}

void* ISComManager::getUserPointer()
{
    return userPointer;
}

is_comm_instance_t* comManagerGetIsComm(port_handle_t port)
{
    return s_cm.getIsComm(port);
}

is_comm_instance_t* ISComManager::getIsComm(port_handle_t port)
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
*    @param[in] dataId       Data structure ID
*    @param[in] offset   Byte offset into data structure.  0 = data start.
*    @param[in] length   Byte length of data.  0 = entire structure.
*    @param[in] periodMultiple Broadcast period of requested data.  0 = single request.
*
*    @return 0 on successful request.  -1 on failure.
*/
void comManagerGetData(port_handle_t port, uint16_t did, uint16_t size, uint16_t offset, uint16_t period)
{
    s_cm.getData(port, did, size, offset, period);
}

void ISComManager::getData(port_handle_t port, uint16_t did, uint16_t size, uint16_t offset, uint16_t period)
{
    // Create and Send request packet
    p_data_get_t get;
    get.id = did;
    get.offset = offset;
    get.size = size;
    get.period = period;

    if (send(port, PKT_TYPE_GET_DATA, &get, 0, sizeof(get), 0)) {
        // if send() is true, then an error occurred...
        // depending on the nature of the error, we may want to close the port.
        // FIXME: we really should be more selective with which errors we actually close the port for.
        if (SERIAL_PORT(port)->errorCode > 0) {
    #ifndef IMX_5
            serialPortClose(port);
            // removePort(port);
            // memset(SERIAL_PORT(port), 0, sizeof(serial_port_s));
            // TODO: we still haven't deleted all references to the port, and this will likely cause problems (ie, InertialSense class, etc).
    #endif
        }
    }
}

void comManagerGetDataRmc(port_handle_t port, uint64_t rmcBits, uint32_t rmcOptions)
{
    s_cm.getDataRmc(port, rmcBits, rmcOptions);
}

void ISComManager::getDataRmc(port_handle_t port, uint64_t rmcBits, uint32_t rmcOptions)
{
    rmc_t rmc;
    rmc.bits = rmcBits;
    rmc.options = rmcOptions;

    sendData(port, &rmc, DID_RMC, sizeof(rmc_t), 0);
}

int comManagerSendData(port_handle_t port, const void *data, uint16_t did, uint16_t size, uint16_t offset)
{
    return s_cm.sendData(port, data, did, size, offset);
}

int ISComManager::sendData(port_handle_t port, const void* data, uint16_t did, uint16_t size, uint16_t offset)
{
    return send(port, PKT_TYPE_SET_DATA, data, did, size, offset);
}

int comManagerSendDataNoAck(port_handle_t port, const void *data, uint16_t did, uint16_t size, uint16_t offset)
{
    return s_cm.sendDataNoAck(port, data, did, size, offset);
}

int ISComManager::sendDataNoAck(port_handle_t port, const void *data, uint16_t did, uint16_t size, uint16_t offset)
{
    return send(port, PKT_TYPE_DATA, data, did, size, offset);
}

int comManagerSendRaw(port_handle_t port, const void *dataPtr, int dataSize)
{
    return s_cm.sendRaw(port, dataPtr, dataSize);
}

int ISComManager::sendRaw(port_handle_t port, const void* dataPtr, int dataSize)
{
    // Return 0 on success, -1 on failure.
    return (portWrite(port, static_cast<const uint8_t *>(dataPtr), dataSize) ? 0 : -1);
}

int comManagerDisableData(port_handle_t port, uint16_t did)
{
    return s_cm.disableData(port, did);
}

int ISComManager::disableData(port_handle_t port, uint16_t did)
{
    return send(port, PKT_TYPE_STOP_DID_BROADCAST, NULL, did, 0, 0);
}

int comManagerSend(port_handle_t port, uint8_t pFlags, const void* data, uint16_t did, uint16_t size, uint16_t offset)
{
    return s_cm.send(port, pFlags, data, did, size, offset);
}

/**
 * Packages and sends an ISb packet to the specified port
 * @param port the port to send the data to
 * @param pFlags the flags to be attributed to the packet
 * @param data the payload data; this pointer should point to the start of the actual data to be sent,
 *   which if sending only a partial payload, may not be the start of the payload buffer (see/use offset)
 * @param did the data id associated with the payload
 * @param size the size of the payload
 * @param offset (optional) the offset into the receivers payload buffer, where this payload should be applied.
 * @return 0 on success, and -1 on failure
 */
int ISComManager::send(port_handle_t port, uint8_t pFlags, const void *data, uint16_t did, uint16_t size, uint16_t offset)
{
    // Return 0 on success, -1 on failure
    return (is_comm_write(port, pFlags, did, size, offset, data) < 0 ? -1 : 0);
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
*    @return 0 on success.  -1 on failure.
*/
int ISComManager::processBinaryRxPacket(protocol_type_t ptype, packet_t *pkt, port_handle_t port)
{
    if ((ptype != _PTYPE_INERTIAL_SENSE_DATA) && (ptype != _PTYPE_INERTIAL_SENSE_CMD))
        return -1;

    packet_hdr_t        *hdr = &(pkt->hdr);
    registered_data_t   *regData = NULL;
    uint8_t             isbPktType = (uint8_t)(pkt->hdr.flags&PKT_TYPE_MASK);

    switch (isbPktType)
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

        p_data_t data;
        data.hdr.id = pkt->dataHdr.id;
        data.hdr.offset = pkt->offset;
        data.hdr.size = pkt->data.size;
        data.ptr = pkt->data.ptr;

        // TODO: std::map.contains() isn't available in older c++ standards.  We may need to perform a local equivalent
        //  if (!didRegistrationMap.contains(hdr->id)) {
        if (didRegistrationMap.find(hdr->id) != didRegistrationMap.end()) {
            // NOTE we do the find() above to see if its exists, because making the following call will insert an empty regData into
            // the map, if ones not already there..
            regData = &didRegistrationMap[hdr->id];
        }

        // Validate and constrain Rx data size to fit within local data struct
        if (regData && regData->dataSet.size && ((uint32_t)(data.hdr.offset + data.hdr.size) > regData->dataSet.size))
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
    
//         unsigned char additionalDataAvailable // function parameter removed 
//         (void)additionalDataAvailable;

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
                regData->pstRxFnc(&data, port);
            }
        }

        // Call general/global callback
        if (pstRxFnc)
            pstRxFnc(&data, port);

#if ENABLE_PACKET_CONTINUATION

        // Clear dataset consolidation
        con->hdr.id = con->hdr.size = con->hdr.offset = 0;

#endif

        // Reply w/ ACK for PKT_TYPE_SET_DATA
        if (isbPktType == PKT_TYPE_SET_DATA)
        {
            sendAck(port, pkt, PKT_TYPE_ACK);
        }
    }
        break;

    case PKT_TYPE_GET_DATA:
#ifdef IMX_5
        {
            p_data_get_t *gdata = ((p_data_get_t *) (pkt->data.ptr));
            // Forward to gpx
            if (IO_CONFIG_GPS1_TYPE(g_nvmFlashCfg->ioConfig) == IO_CONFIG_GPS_TYPE_GPX &&
                (((gdata->id >= DID_GPX_FIRST) && (gdata->id <= DID_GPX_LAST)) || (gdata->id == DID_RTK_DEBUG))) {
                comManagerGetData(COM0_PORT, gdata->id, gdata->size, gdata->offset, gdata->period);

                if (gdata->id == DID_RTK_DEBUG) {
                    if (gdata->period != 0)
                        g_GpxRtkDebugReq |= 0x01 << portId(port);
                    else
                        g_GpxRtkDebugReq |= 0x01 << (portId(port) + 4);
                }
            }
        }
#endif
        if (getDataRequest(port, (p_data_get_t*)(pkt->data.ptr)))
        {
            sendAck(port, pkt, PKT_TYPE_NACK);
        }
        break;

    case PKT_TYPE_STOP_BROADCASTS_ALL_PORTS:
        disableBroadcasts(NULL);  // all ports

        // Call disable broadcasts callback if exists
        if (disableBcastFnc)
            disableBcastFnc(NULL);  // all ports

        sendAck(port, pkt, PKT_TYPE_ACK);
#ifdef IMX_5
        g_GpxRtkDebugReq = 0;
#endif
        break;

    case PKT_TYPE_STOP_BROADCASTS_CURRENT_PORT:
        disableBroadcasts(port);

        // Call disable broadcasts callback if exists
        if (disableBcastFnc)
            disableBcastFnc(port);

        sendAck(port, pkt, PKT_TYPE_ACK);

#ifdef IMX_5
        g_GpxRtkDebugReq &= ~(0x01 << portId(port));
#endif
        break;

    case PKT_TYPE_STOP_DID_BROADCAST:
        disableDidBroadcast(port, pkt->hdr.id);
#ifdef IMX_5
        if (DID_RTK_DEBUG)
            g_GpxRtkDebugReq &= ~(0x01 << portId(port));
#endif
        break;

    case PKT_TYPE_NACK:
    case PKT_TYPE_ACK:
        // Call general ack callback
        if (pstAckFnc)
            pstAckFnc(port, (p_ack_t*)(pkt->data.ptr), ptype);
        break;
    }

    // Success
    return 0;
}

int comManagerProcessBinaryRxPacket(protocol_type_t ptype, packet_t *pkt, port_handle_t port)
{
    return s_cm.processBinaryRxPacket(ptype, pkt, port);
}

bufTxRxPtr_t* comManagerGetRegisteredDataInfo(uint16_t did)
{
    return s_cm.getRegisteredDataInfo(did);
}

bufTxRxPtr_t* ISComManager::getRegisteredDataInfo(uint16_t did)
{
    // TODO: contains() isn't available in older c++ standards.  We may need to perform a local equivalent
    //  if (!didRegistrationMap.contains(did)) {
    if (didRegistrationMap.find(did) == didRegistrationMap.end()) {
        return 0;
    }
    return &didRegistrationMap[did].dataSet;
}

// 0 on success. -1 on failure.
int comManagerGetDataRequest(port_handle_t port, p_data_get_t* req)
{
    return s_cm.getDataRequest(port, req);
}

//__attribute__((optimize("O0")))
int ISComManager::getDataRequest(port_handle_t port, p_data_get_t* req)
{
    broadcast_msg_t* msg = NULL;

    // Validate the request
    if (req->id >= DID_COUNT)
    {
        // invalid data id
        return -1;
    }
    // Call RealtimeMessageController (RMC) handler
    else if (cmMsgHandlerRmc && (cmMsgHandlerRmc(req, port) == 0))
    {
        // Don't allow comManager broadcasts for messages handled by RealtimeMessageController. 
        return 0;
    }
    // if size is 0 and offset is 0, set size to full data struct size
    else if (req->size == 0 && req->offset == 0 && req->id < DID_COUNT)
    {
        req->size = didRegistrationMap[req->id].dataSet.size;
    }
    
    if (req->size == 0)
    {   // Don't respond if data size is zero. Return zero to prevent sending NACK.
        return 0;
    }

    // Copy reference to source data
    bufTxRxPtr_t* dataSetPtr = &didRegistrationMap[req->id].dataSet;

    if ((uint32_t)(req->offset + req->size) > dataSetPtr->size)
    {
        req->offset = 0;
        req->size = dataSetPtr->size;
    }

    // Search for matching message (i.e. matches port, id, size, and offset)...
    for (auto& bc : *(broadcastMessages))
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
        for (auto& bc : *(broadcastMessages))
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
            msg = reinterpret_cast<broadcast_msg_t *>(&(broadcastMessages)[0]); //  + MAX_NUM_BCAST_MSGS - 1;
        }
    }

    msg->port = port;
    packet_t *pkt = &(msg->pkt);
    uint8_t *dataPtr;
    if (dataSetPtr->txPtr)
    {
        dataPtr = didRegistrationMap[req->id].dataSet.txPtr + req->offset;
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
        if (didRegistrationMap[req->id].preTxFnc)
        {
            sendData = didRegistrationMap[req->id].preTxFnc(port, &(pkt->dataHdr));
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
        if (txFree == 0 || pkt->size <= (uint32_t)txFree(port))
        {
            if (sendData)
            {
                sendDataPacket(port, &(msg->pkt));
            }
        }

        // Enable broadcast message
        enableBroadcastMsg(msg, req->period);
    }
    else
    {
        // ***  Request Single  ***
        // Send data immediately if possible
        if (txFree == 0 || pkt->size <= (uint32_t)txFree(port))
        {
            if (sendData)
            {
                sendDataPacket(port, &(msg->pkt));
            }
            disableBroadcastMsg(msg);
        }
        else
        {
            // Won't fit in queue, so send it later
            enableBroadcastMsg(msg, req->period);
        }
    }

    return 0;
}

void ISComManager::enableBroadcastMsg(broadcast_msg_t* msg, int periodMultiple)
{
    // Update broadcast period
    if (periodMultiple > 0)
    {
        msg->period = periodMultiple / stepPeriodMilliseconds;
    }
    else
    {
        msg->period = MSG_PERIOD_SEND_ONCE;
    }
    msg->counter = -1;   // Keeps broadcast from sending for at least one period
}

void ISComManager::disableBroadcastMsg(broadcast_msg_t *msg)
{
    // Remove item from linked list
    msg->period = MSG_PERIOD_DISABLED;
}

void comManagerDisableBroadcasts(port_handle_t port)
{
    s_cm.disableBroadcasts(port);
}

void ISComManager::disableBroadcasts(port_handle_t port)
{
    // for (broadcast_msg_t* bcPtr = cmInstance->broadcastMessages, *ptrEnd = (cmInstance->broadcastMessages + MAX_NUM_BCAST_MSGS); bcPtr < ptrEnd; bcPtr++)
    for (auto& bc : *broadcastMessages)
    {
        if ((port == NULL) || (bc.port == port))
        {
            bc.period = MSG_PERIOD_DISABLED;
        }
    }
}

void ISComManager::disableDidBroadcast(port_handle_t port, uint16_t did)
{
    // for (broadcast_msg_t* bcPtr = cmInstance->broadcastMessages, *ptrEnd = (cmInstance->broadcastMessages + MAX_NUM_BCAST_MSGS); bcPtr < ptrEnd; bcPtr++)
    for (auto& bc : *broadcastMessages)
    {
        if (((port == NULL) || (port == bc.port)) && (bc.pkt.hdr.id == did))
        {
            bc.period = MSG_PERIOD_DISABLED;
        }
    }
    
    // Call global broadcast handler to disable message control
    if (cmMsgHandlerRmc)
    {
        p_data_get_t req;
        req.id = did;
        req.size = 0;
        req.offset = 0;
        req.period = 0;
        cmMsgHandlerRmc(&req, port);
    }
}

// Consolidate this with sendPacket() so that we break up packets into multiples that fit our buffer size.  Returns 0 on success, -1 on failure.
int ISComManager::sendDataPacket(port_handle_t port, packet_t* pkt)
{
    int bytes = is_comm_write_isb_precomp_to_port(port, pkt);

    // Return 0 on success, -1 on failure
    return (bytes < 0) ? -1 : 0;
}

void ISComManager::sendAck(port_handle_t port, packet_t *pkt, uint8_t pTypeFlags)
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

    send(port, pTypeFlags, &ack, 0, sizeof(ack), 0);
}

int comManagerValidateBaudRate(unsigned int baudRate)
{
    // Valid baudrate for InertialSense hardware
    return validateBaudRate(baudRate);
}

void comManagerSetErrorHandler(pfnComManagerParseErrorHandler errorCb) {
    s_cm.setErrorHandler(errorCb);
}

pfnIsCommIsbDataHandler ISComManager::registerIsbDataHandler(pfnIsCommIsbDataHandler cbHandler, port_handle_t port) {
    if (port && portType(port) & PORT_TYPE__COMM) {
        return is_comm_register_isb_handler(&COMM_PORT(port)->comm, cbHandler);
    }

    if (ports && !port) {
        // if port is null, set this for all available ports
        defaultCbs.isbData = cbHandler;
        for (auto port : *ports) {
            if (port && portType(port) & PORT_TYPE__COMM) {
                is_comm_register_isb_handler(&COMM_PORT(port)->comm, cbHandler);
            }
        }
    }
    return NULL;
}

pfnIsCommGenMsgHandler ISComManager::registerProtocolHandler(int ptype, pfnIsCommGenMsgHandler cbHandler, port_handle_t port) {
    if (port && portType(port) & PORT_TYPE__COMM) {
        return is_comm_register_msg_handler(&COMM_PORT(port)->comm, ptype, cbHandler);
    }

    if (ports && !port) {
        // if port is null, set this for all available ports
        defaultCbs.generic[ptype] = cbHandler; // TODO: range check this
        for (auto port : *ports) {
            if (port && portType(port) & PORT_TYPE__COMM) {
                is_comm_register_msg_handler(&COMM_PORT(port)->comm, ptype, cbHandler);
            }
        }
    }
    return NULL;
}