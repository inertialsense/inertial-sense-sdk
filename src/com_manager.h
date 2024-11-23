/*
MIT LICENSE

Copyright (c) 2014-2024 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#ifndef COM_MANAGER_H
#define COM_MANAGER_H

#include <array>
#include <list>
#include <unordered_set>
#include <vector>
#include <map>

#include <stdint.h>
#include "ISComm.h"
#include "linked_list.h"

/** Maximum number of messages that may be broadcast simultaneously, per port.
Since most messages use the RMC (real-time message controller) now, this can be fairly low */
#define MAX_NUM_BCAST_MSGS 12

// Convenience macros for creating Com Manager buffers
#define COM_MANAGER_BUF_SIZE_BCAST_MSG(max_num_bcast_msgs)      ((max_num_bcast_msgs)*sizeof(broadcast_msg_t))

/* Contains data that determines what messages are being broadcast */
typedef struct
{
    packet_t                pkt;

    /* Broadcast period counter */
    int32_t                 counter;

    /* Millisecond broadcast period intervals.  -1 = send once.  0 = disabled/unused/don't send. */
    int32_t                 period;

    /* Port to broadcast on. */
    port_handle_t           port;
} broadcast_msg_t;

typedef std::array<broadcast_msg_t, MAX_NUM_BCAST_MSGS> broadcast_msg_array_t;

/** Contains status for the com manager */
typedef struct  
{
    /** 0 if no errors encountered, otherwise non-zero.  Used in conjunction with communicationErrorCount. */
    uint32_t rxError;

    /** number of communication errors - can be reset to 0 if desired */
    uint32_t communicationErrorCount;

    /**
    flags to send on each request - do not modify
    valid data : flags & CM_PKT_FLAGS_RX_VALID_DATA
    */
    // uint8_t flags;
} com_manager_status_t;

enum eComManagerErrorType
{
    CM_ERROR_FORWARD_OVERRUN = -1,
    CM_ERROR_RX_PARSE = -2,
};

// com manager callback prototypes
// readFnc read data from the serial port. Returns number of bytes read.
// typedef int(*pfnComManagerRead)(port_handle_t port, unsigned char* buf, int len);

// txFreeFnc optional, return the number of free bytes in the send buffer for the serial port represented by port
typedef int(*pfnComManagerSendBufferAvailableBytes)(port_handle_t port);

// pstRxFnc optional, called after data is sent to the serial port represented by port
typedef int(*pfnComManagerPostRead)(p_data_t* dataRead, port_handle_t port);

// pstAckFnc optional, called after an ACK is received by the serial port represented by port
typedef int(*pfnComManagerPostAck)(port_handle_t port, p_ack_t* ack, unsigned char packetIdentifier);

// disableBcastFnc optional, mostly for internal use, this can be left as 0 or NULL.  Set port to -1 for all ports.
typedef int(*pfnComManagerDisableBroadcasts)(port_handle_t port);

// Called right before data is to be sent.  Data is not sent if this callback returns 0.
typedef int(*pfnComManagerPreSend)(port_handle_t port, p_data_hdr_t *dataHdr);

// broadcast message handler
typedef int(*pfnComManagerRmcHandler)(p_data_get_t* req, port_handle_t port);

// Parse error handler function, return 1 if message handled
typedef int(*pfnComManagerParseErrorHandler)(port_handle_t port);


/**
* Initializes the default global com manager. This is generally only called once on program start.
* The global com manager is used by the functions that do not have the Instance suffix and first parameter of void* cmInstance.
* The instance functions can be ignored, unless you have a reason to have two com managers in the same process.
*
* @param a port to initialize with (maybe NULL if no ports are available yet)
* @param stepPeriodMilliseconds how many milliseconds you are waiting in between calls to comManagerStep
* @param readFnc read data from the serial port represented by port
* @param sendFnc send data to the serial port represented by port
* @param txFreeFnc optional, return the number of free bytes in the send buffer for the serial port represented by port
* @param pstRxFnc optional, called after new data is available (successfully parsed and checksum passed) from the serial port represented by port
* @param pstAckFnc optional, called after an ACK is received by the serial port represented by port
* @param disableBcastFnc mostly for internal use, this can be left as 0 or NULL
* @param timeMs pointer to current time in milliseconds, used for parser state timeout.  Leave NULL if timeout feature is not used.
* @return 0 on success, -1 on failure
*
* Example:
* @code
* comManagerInit(20, 20, 10, 25, staticReadData, staticSendData, NULL, staticProcessRxData, staticProcessAck, 0);
* @endcode
*/
int comManagerInit(
        std::unordered_set<port_handle_t>* portSet,
        int stepPeriodMilliseconds,
        pfnComManagerPostRead pstRxFnc,
        pfnComManagerPostAck pstAckFnc,
        pfnComManagerRmcHandler rmcHandler,
        pfnComManagerDisableBroadcasts disableBcastFnc,
        std::array<broadcast_msg_t, MAX_NUM_BCAST_MSGS>* buffers);

/** Alternate without having to specify a port...
 * TODO: Remove this and move port to the last, and make it optional.
 * @param port
 * @param stepPeriodMilliseconds
 * @param pstRxFnc
 * @param pstAckFnc
 * @param disableBcastFnc
 * @param buffers
 * @return
 */
int comManagerInit(
        int stepPeriodMilliseconds,
        pfnComManagerPostRead pstRxFnc,
        pfnComManagerPostAck pstAckFnc,
        pfnComManagerRmcHandler rmcHandler,
        pfnComManagerDisableBroadcasts disableBcastFnc,
        std::array<broadcast_msg_t, MAX_NUM_BCAST_MSGS>* buffers);

pfnIsCommGenMsgHandler comManagerRegisterProtocolHandler(int ptype, pfnIsCommGenMsgHandler cbHandler, port_handle_t port = NULL);


port_handle_t comManagerAllocatePort(int ptype);


/**
 * registered a port with the comm manager (allowing the port to be managed by ISComManager.
 * @param port
 * @return true if this port was registered, otherwise false
 */
bool comManagerRegisterPort(port_handle_t port);

/**
 * registered a port with the comm manager (allowing the port to be managed by ISComManager.
 * @param port
 * @param callbacks custom set of callbacks to use for this port (if null, use defaults)
 * @return true if this port was registered, otherwise false
 */
bool comManagerRegisterPort(port_handle_t port, is_comm_callbacks_t* callbacks);


/**
 * @return a vector of all registered ports
 */
std::unordered_set<port_handle_t>& comManagerGetPorts();


/**
 * Removes the requested port from the comManager, preventing it from being considered in broadcasts or received data.
 * NOTE that this call does not close the port of flush the port.
 * @return true if the port was found and removed, otherwise false.
 */
bool comManagerRemovePort(port_handle_t port);


/**
 *
 * @return
 */
bool comManagerReleaseAllPorts();

/**
* Performs one round of sending and receiving message. Call as frequently as needed to send and receive data.
* @param timeMs current time in milliseconds used for paser timeout.  Used to invalidate packet parsing if PKT_PARSER_TIMEOUT_MS time has lapsed since any data has been received.
*/
void comManagerStepTimeout(uint32_t timeMs);

void comManagerStep();

void comManagerStep(port_handle_t port);

void stepSendMessages(void);

/**
* Make a request to a port to broadcast a piece of data at a set interval.
*
* @param port the port to request broadcast data from
* @param dataId the data id to broadcast
* @param size number of bytes in the data structure from offset to broadcast - pass size and offset of 0 to receive the entire data set
* @param offset offset into the structure for the data id to broadcast - pass size and offset of 0 to receive the entire data set
* @param periodMultiple the data broadcast period in multiples of the base update period
*
* Example that makes a request to receive the device info just once:
* @code
* comManagerGetData(0, DID_DEV_INFO, 0, 0, 0);
* @endcode
*
* Example that broadcasts INS data every 50 milliseconds:
* @code
* comManagerGetData(0, DID_INS_1, 0, 0, 50);
* @endcode
*/
void comManagerGetData(port_handle_t port, uint16_t did, uint16_t size, uint16_t offset, uint16_t period);

/**
* Make a request to a port handle to broadcast a piece of data at a set interval.
*
* @param port the port handle to request broadcast data from
* @param RMC bits specifying data messages to stream.  See presets: RMC_PRESET_IMX_PPD = post processing data, RMC_PRESET_INS = INS2 and GPS data at full rate
* @param RMC options to enable data streaming on ports other than the current port.
* @param offset offset into the structure for the data id to broadcast - pass offset and size of 0 to receive the entire data set
* @param size number of bytes in the data structure from offset to broadcast - pass offset and size of 0 to receive the entire data set
* @param periodMultiple the data broadcast period in multiples of the base update period
*
* Example that enables streaming of all data messages necessary for post processing:
* @code
* comManagerGetDataRmc(port, RMC_PRESET_IMX_PPD, 0);
* @endcode
*
* Example that broadcasts INS and GPS data at full rate:
* @code
* comManagerGetDataRmc(port, RMC_PRESET_INS, 0);
* @endcode
*/
void comManagerGetDataRmc(port_handle_t port, uint64_t rmcBits, uint32_t rmcOptions);

/**
* Disable a broadcast for a specified port handle and data identifier
*
* @param port the port handle to disable a broadcast for
* @param dataId the data id to disable boradcast for
* @return 0 if success, anything else if failure
*
* Example:
* @code
* comManagerDisableData(0, DID_INS_1);
* @endcode
*/
int comManagerDisableData(port_handle_t port, uint16_t did);

/**
* Send a packet to a port handle
*
* @param port the port handle to send the packet to
* @param pktInfo the type of packet (PID)
* @param bodyHdr optional, can contain information about the actual data of the body (txData), usually the data id, offset and size
* @param txData optional, the actual body of the packet
* @return 0 if success, anything else if failure
*
* Example:
* @code
* p_data_get_t request;
* bufPtr_t data;
* request.id = DID_INS_1;
* request.offset = 0;
* request.size = sizeof(ins_1_t);
* request.bc_period_ms = 50;
* data.ptr = (uint8_t*)&request;
* data.size = sizeof(request);
* comManagerSend(port, PKT_TYPE_GET_DATA, 0, &data)
* @endcode
*/
int comManagerSend(port_handle_t port, uint8_t pFlags, const void *data, uint16_t did, uint16_t size, uint16_t offset = 0);

/**
* Convenience function that wraps comManagerSend for sending data structures.  Must be multiple of 4 bytes in size.
*
* @param port the port handle to send data to
* @param dataId the data id of the data to send
* @param dataPtr pointer to the data structure to send
* @param dataSize number of bytes to send
* @param dataOffset offset into dataPtr to send at
* @return 0 if success, anything else if failure
*
* Example:
* @code
* comManagerSendData(0, DID_DEV_INFO, &g_devInfo, sizeof(dev_info_t), 0);
* @endcode
*/
int comManagerSendData(port_handle_t port, const void* data, uint16_t did, uint16_t size, uint16_t offset = 0);

// INTERNAL FUNCTIONS...
/**
* Same as comManagerSend, except that no retry is attempted
*
* @param port the port handle to send the packet to
* @param dataId Data structure ID number.
* @param dataPtr Pointer to actual data.
* @param dataSize Size of data to send in number of bytes.
* @param dataOffset Offset into data structure where copied data starts.
* @param pFlags Additional packet flags if needed.
* @return 0 if success, anything else if failure
*/
int comManagerSendDataNoAck(port_handle_t port, const void *data, uint16_t did, uint16_t size, uint16_t offset = 0);

/**
* Write bare data directly to the serial port.
*
* @param port the port handle to send data to
* @param dataPtr pointer to the data structure to send
* @param dataSize number of bytes to send
* @return 0 if success, anything else if failure
*
* Example:
* @code
* comManagerSendRaw(0, &g_devInfo, sizeof(dev_info_t));
* @endcode
*/
int comManagerSendRaw(port_handle_t port, const void* dataPtr, int dataSize);


/**
* Disables broadcasts of all messages on specified port, or all ports if phandle == -1.
* @param port the port to disable broadcasts on, -1 for all
*/
void comManagerDisableBroadcasts(port_handle_t port);


/**
* Get the ISComm structure.
*
* @return com manager ISComm structure, this pointer is owned by the com manager
*/
is_comm_instance_t* comManagerGetIsComm(port_handle_t port);


/**
* Internal use mostly, get data info for a the specified pre-registered dataId
*
* @return 0 on failure, pointer on success
*/
bufTxRxPtr_t* comManagerGetRegisteredDataInfo(uint16_t did);


/**
* Internal use mostly, process a get data request for a message that needs to be broadcasted
*
* @return 0 on success, anything else is failure
*/
int comManagerGetDataRequest(port_handle_t port, p_data_get_t* req);

/**
 * Register a callback function when a parse error occurs.
 * @param errorCb
 */
void comManagerSetErrorHandler(pfnComManagerParseErrorHandler errorCb);

/**
* Register message handling function for a received data id (binary). This is mostly an internal use function,
* but can be used if you are implementing your own receiver on a device.
*
* @param dataId the data id to register the handler for
* @param txFnc called right before the data is sent
* @param pstRxFnc called after data is received for the data id
* @param txDataPtr a pointer to the structure in memory of the data to send
* @param rxDataPtr a pointer to the structure in memory to copy received data to
* @param dataSize size of the data structure in txDataPtr and rxDataPtr
* @param pktFlags packet flags, usually 0
*
* Example:
* @code
* registerComManager(DID_INS_1, prepMsgINS, writeMsgINS, &g_insData, &g_insData, sizeof(ins_1_t));
* @endcode
*/
void comManagerRegister(uint16_t did, pfnComManagerPreSend txFnc, pfnComManagerPostRead pstRxFnc, const void* txDataPtr, void* rxDataPtr, uint16_t size, uint8_t pktFlags);

int comManagerProcessBinaryRxPacket(protocol_type_t ptype, packet_t *pkt, port_handle_t port);

class ISComManager {
public:
    void step();
    void stepTx();
    void stepRx(port_handle_t port);
    void stepRx();

    /**
    * Initializes the default global com manager. This is generally only called once on program start.
    * The global com manager is used by the functions that do not have the Instance suffix and first parameter of void* cmInstance.
    * The instance functions can be ignored, unless you have a reason to have two com managers in the same process.
    *
    * @param portSet a pointer to a container which manages all available ports
    * @param stepPeriodMilliseconds how many milliseconds you are waiting in between calls to comManagerStep
    * @param readFnc read data from the serial port represented by port
    * @param sendFnc send data to the serial port represented by port
    * @param txFreeFnc optional, return the number of free bytes in the send buffer for the serial port represented by port
    * @param pstRxFnc optional, called after new data is available (successfully parsed and checksum passed) from the serial port represented by port
    * @param pstAckFnc optional, called after an ACK is received by the serial port represented by port
    * @param disableBcastFnc mostly for internal use, this can be left as 0 or NULL
    * @param timeMs pointer to current time in milliseconds, used for parser state timeout.  Leave NULL if timeout feature is not used.
    * @return 0 on success, -1 on failure
    *
    * Example:
    * @code
    * comManagerInit(20, 20, 10, 25, staticReadData, staticSendData, NULL, staticProcessRxData, staticProcessAck, 0);
    * @endcode
    */
    int init(
            std::unordered_set<port_handle_t>* portSet,
            int stepPeriodMilliseconds,
            pfnComManagerPostRead pstRxFnc,
            pfnComManagerPostAck pstAckFnc,
            pfnComManagerRmcHandler rmcHandler,
            pfnComManagerDisableBroadcasts disableBcastFnc,
            std::array<broadcast_msg_t, MAX_NUM_BCAST_MSGS>* buffers);

    pfnIsCommIsbDataHandler registerIsbDataHandler(pfnIsCommIsbDataHandler cbHandler, port_handle_t port = NULL);

    pfnIsCommGenMsgHandler registerProtocolHandler(int ptype, pfnIsCommGenMsgHandler cbHandler, port_handle_t port = NULL);

    void setErrorHandler(pfnComManagerParseErrorHandler errorCb) { errorHandlerFnc = errorCb; }

    /**
     * Allocates and registers a new port.
     * @param ptype
     * @return
     */
    port_handle_t allocatePort(int ptype);


    /**
     * registered a port with the comm manager (allowing the port to be managed by ISComManager.
     * @param port
     * @return true if this port was registered, otherwise false
     */
    bool registerPort(port_handle_t port, is_comm_callbacks_t* callbacks = NULL);


    /**
     * @return a vector of all registered ports
     */
    std::unordered_set<port_handle_t>& getPorts();


    /**
     * Assigns the passed vector of port_handle_t as the set of registered ports used by the comManager
     * @param newPorts
     * @return returns the difference in ports between the old set and the new
     */
    int setPorts(std::list<port_handle_t> newPorts);


    /**
     * Removes the requested port from the comManager, preventing it from being considered in broadcasts or received data.
     * NOTE that this call does not close the port of flush the port.
     * @return true if the port was found and removed, otherwise false.
     */
    bool removePort(port_handle_t port);


    /**
     * Close and release(free/delete) all registered/allocated ports
     * @return
     */
    bool removeAllPorts();

    /**
    * Performs one round of sending and receiving message. Call as frequently as needed to send and receive data.
    * @param timeMs current time in milliseconds used for paser timeout.  Used to invalidate packet parsing if PKT_PARSER_TIMEOUT_MS time has lapsed since any data has been received.
    */
    void stepTimeout(uint32_t timeMs);


    /**
    * Make a request to a port handle to broadcast a piece of data at a set interval.
    *
    * @param port the port handle to request broadcast data from
    * @param dataId the data id to broadcast
    * @param size number of bytes in the data structure from offset to broadcast - pass size and offset of 0 to receive the entire data set
    * @param offset offset into the structure for the data id to broadcast - pass size and offset of 0 to receive the entire data set
    * @param periodMultiple the data broadcast period in multiples of the base update period
    *
    * Example that makes a request to receive the device info just once:
    * @code
    * comManagerGetData(0, DID_DEV_INFO, 0, 0, 0);
    * @endcode
    *
    * Example that broadcasts INS data every 50 milliseconds:
    * @code
    * comManagerGetData(0, DID_INS_1, 0, 0, 50);
    * @endcode
    */
    void getData(port_handle_t port, uint16_t did, uint16_t size, uint16_t offset, uint16_t period);

    /**
    * Make a request to a port handle to broadcast a piece of data at a set interval.
    *
    * @param port the port handle to request broadcast data from
    * @param RMC bits specifying data messages to stream.  See presets: RMC_PRESET_PPD_BITS = post processing data, RMC_PRESET_INS_BITS = INS2 and GPS data at full rate
    * @param RMC options to enable data streaming on ports other than the current port.
    * @param offset offset into the structure for the data id to broadcast - pass offset and size of 0 to receive the entire data set
    * @param size number of bytes in the data structure from offset to broadcast - pass offset and size of 0 to receive the entire data set
    * @param periodMultiple the data broadcast period in multiples of the base update period
    *
    * Example that enables streaming of all data messages necessary for post processing:
    * @code
    * comManagerGetDataRmc(port, RMC_PRESET_PPD_BITS, 0);
    * @endcode
    *
    * Example that broadcasts INS and GPS data at full rate:
    * @code
    * comManagerGetDataRmc(port, RMC_PRESET_INS_BITS, 0);
    * @endcode
    */
    void getDataRmc(port_handle_t port, uint64_t rmcBits, uint32_t rmcOptions);

    /**
    * Disable a broadcast for a specified port handle and data identifier
    *
    * @param port the port handle to disable a broadcast for
    * @param dataId the data id to disable boradcast for
    * @return 0 if success, anything else if failure
    *
    * Example:
    * @code
    * comManagerDisableData(0, DID_INS_1);
    * @endcode
    */
    int disableData(port_handle_t port, uint16_t did);

    /**
    * Send a packet to a port handle
    *
    * @param port the port handle to send the packet to
    * @param pktInfo the type of packet (PID)
    * @param bodyHdr optional, can contain information about the actual data of the body (txData), usually the data id, offset and size
    * @param txData optional, the actual body of the packet
    * @return 0 if success, anything else if failure
    *
    * Example:
    * @code
    * p_data_get_t request;
    * bufPtr_t data;
    * request.id = DID_INS_1;
    * request.offset = 0;
    * request.size = sizeof(ins_1_t);
    * request.bc_period_ms = 50;
    * data.ptr = (uint8_t*)&request;
    * data.size = sizeof(request);
    * comManagerSend(port, PKT_TYPE_GET_DATA, 0, &data)
    * @endcode
    */
    int send(port_handle_t port, uint8_t pFlags, const void *data, uint16_t did, uint16_t size, uint16_t offset = 0);

    /**
    * Convenience function that wraps comManagerSend for sending data structures.  Must be multiple of 4 bytes in size.
    *
    * @param port the port handle to send data to
    * @param dataId the data id of the data to send
    * @param dataPtr pointer to the data structure to send
    * @param dataSize number of bytes to send
    * @param dataOffset offset into dataPtr to send at
    * @return 0 if success, anything else if failure
    *
    * Example:
    * @code
    * comManagerSendData(0, DID_DEV_INFO, &g_devInfo, sizeof(dev_info_t), 0);
    * @endcode
    */
    int sendData(port_handle_t port, const void* data, uint16_t did, uint16_t size, uint16_t offset = 0);

    // INTERNAL FUNCTIONS...
    /**
    * Same as comManagerSend, except that no retry is attempted
    *
    * @param port the port handle to send the packet to
    * @param dataId Data structure ID number.
    * @param dataPtr Pointer to actual data.
    * @param dataSize Size of data to send in number of bytes.
    * @param dataOffset Offset into data structure where copied data starts.
    * @param pFlags Additional packet flags if needed.
    * @return 0 if success, anything else if failure
    */
    int sendDataNoAck(port_handle_t port, const void *data, uint16_t did, uint16_t size, uint16_t offset = 0);

    /**
    * Write bare data directly to the serial port.
    *
    * @param port the port handle to send data to
    * @param dataPtr pointer to the data structure to send
    * @param dataSize number of bytes to send
    * @return 0 if success, anything else if failure
    *
    * Example:
    * @code
    * comManagerSendRaw(0, &g_devInfo, sizeof(dev_info_t));
    * @endcode
    */
    int sendRaw(port_handle_t port, const void* dataPtr, int dataSize);


    /**
    * Disables broadcasts of all messages on specified port, or all ports if phandle == -1.
    * @param port the port to disable broadcasts on, -1 for all
    */
    void disableBroadcasts(port_handle_t port);


    /**
    * Get the ISComm structure.
    *
    * @return com manager ISComm structure, this pointer is owned by the com manager
    */
    is_comm_instance_t* getIsComm(port_handle_t port);


    /**
    * Internal use mostly, get data info for a the specified pre-registered dataId
    *
    * @return 0 on failure, pointer on success
    */
    bufTxRxPtr_t* getRegisteredDataInfo(uint16_t did);


    /**
    * Internal use mostly, process a get data request for a message that needs to be broadcasted
    *
    * @return 0 on success, anything else is failure
    */
    int getDataRequest(port_handle_t port, p_data_get_t* req);


    /**
    * Register message handling function for a received data id (binary). This is mostly an internal use function,
    * but can be used if you are implementing your own receiver on a device.
    *
    * @param dataId the data id to register the handler for
    * @param txFnc called right before the data is sent
    * @param pstRxFnc called after data is received for the data id
    * @param txDataPtr a pointer to the structure in memory of the data to send
    * @param rxDataPtr a pointer to the structure in memory to copy received data to
    * @param dataSize size of the data structure in txDataPtr and rxDataPtr
    * @param pktFlags packet flags, usually 0
    *
    * Example:
    * @code
    * registerComManager(DID_INS_1, prepMsgINS, writeMsgINS, &g_insData, &g_insData, sizeof(ins_1_t));
    * @endcode
    */
    void registerDid(uint16_t did, pfnComManagerPreSend txFnc, pfnComManagerPostRead pstRxFnc, const void* txDataPtr, void* rxDataPtr, uint16_t size, uint8_t pktFlags);

    /**
    * Attach user defined data to a com manager instance
    */
    void assignUserPointer(void* userPointer);


    /**
    * Get user defined data to from a com manager instance
    */
    void* getUserPointer();



    void stepSendMessages(void);


    /**
    * Ensure baudrate is valid for InertialSense hardware
    * @param baudRate the baud rate to check
    * @return 0 if baud rate is valid, -1 if not
    */
    int validateBaudRate(unsigned int baudRate);

    int processBinaryRxPacket(protocol_type_t ptype, packet_t *pkt, port_handle_t port);

private:
// int processAsciiRxPacket(com_manager_t* cmInstance, port_handle_t port, unsigned char* start, int count);
// void parseAsciiPacket(com_manager_t* cmInstance, port_handle_t port, unsigned char* buf, int count);

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

    void enableBroadcastMsg(broadcast_msg_t *msg, int periodMultiple);
    void disableBroadcastMsg(broadcast_msg_t *msg);
    void disableDidBroadcast(port_handle_t port, uint16_t did);
    int sendDataPacket(port_handle_t port, packet_t *pkt);
    void sendAck(port_handle_t port, packet_t *pkt, uint8_t pTypeFlags);

    int findAsciiMessage(const void * a, const void * b);
    int asciiMessageCompare(const void* elem1, const void* elem2);

    is_comm_callbacks_t defaultCbs; // local copy of any callbacks passed at init

    // Array of port
    std::unordered_set<port_handle_t>* ports = NULLPTR;  // this is not a vector of ports, its a pointer to an EXTERNAL set of ports.  This MUST be initialized!!!

    // reads n bytes into buffer from the source (usually a serial port)
    // pfnIsCommPortRead portRead;

    // write data to the destination (usually a serial port)
    // pfnIsCommPortWrite portWrite;

    // bytes free in Tx buffer (used to check if packet, keeps us from overflowing the Tx buffer)
    pfnComManagerSendBufferAvailableBytes txFree;

    // Callback function pointer, used to respond to data input
    pfnComManagerPostRead pstRxFnc;

    // Callback function pointer, used to respond to ack
    pfnComManagerPostAck pstAckFnc;

    // Callback function pointer to disable broadcasts on specified port, or all ports if port is -1
    pfnComManagerDisableBroadcasts disableBcastFnc;

    // Callback function pointer for parse errors
    pfnComManagerParseErrorHandler errorHandlerFnc;

    // Pointer to local data and data specific callback functions  ::  NOTE: https://howardhinnant.github.io/stack_alloc.html  if using this in embedded environments and dynamic allocation is a concern
    std::map<int, registered_data_t> didRegistrationMap;

    broadcast_msg_array_t* broadcastMessages; // MAX_NUM_BCAST_MSGS slots

    // processing interval
    int32_t stepPeriodMilliseconds;

    // user defined pointer
    void* userPointer;

    // Broadcast message handler.  Called whenever we get a message broadcast request or message disable command.
    pfnComManagerRmcHandler cmMsgHandlerRmc;

    // Error handler
    pfnComManagerParseErrorHandler cmMsgHandlerError;

};

extern ISComManager s_cm;

// com manager instance / handle is a void*
typedef ISComManager* CMHANDLE;

// get the global instance of the com manager - this is only needed if you are working with multiple com managers and need to compare instances
CMHANDLE comManagerGetGlobal(void);

#endif // COM_MANAGER_H
