/*
MIT LICENSE

Copyright (c) 2014-2024 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#ifndef COM_MANAGER_H
#define COM_MANAGER_H


#ifdef __cplusplus
#include <array>
#include <vector>
#include <map>

extern "C" {
#endif
    
#include <stdint.h>
#include "ISComm.h"
#include "linked_list.h"


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

/** Buffers used in com manager */
/*
typedef struct
{
	broadcast_msg_t* broadcastMsg;
	uint32_t broadcastMsgSize;			// MAX_NUM_BCAST_MSGS * sizeof(broadcast_msg_t)
} com_manager_init_t;
*/

enum eComManagerErrorType
{
	CM_ERROR_FORWARD_OVERRUN = -1, 
	CM_ERROR_RX_PARSE = -2,
};

/** Maximum number of messages that may be broadcast simultaneously, per port.
Since most messages use the RMC (real-time message controller) now, this can be fairly low */
#define MAX_NUM_BCAST_MSGS 12

// Convenience macros for creating Com Manager buffers
#define COM_MANAGER_BUF_SIZE_BCAST_MSG(max_num_bcast_msgs)		((max_num_bcast_msgs)*sizeof(broadcast_msg_t))

// com manager callback prototypes
// readFnc read data from the serial port. Returns number of bytes read.
typedef int(*pfnComManagerRead)(port_handle_t port, unsigned char* buf, int len);

// txFreeFnc optional, return the number of free bytes in the send buffer for the serial port represented by pHandle
typedef int(*pfnComManagerSendBufferAvailableBytes)(port_handle_t port);

// pstRxFnc optional, called after data is sent to the serial port represented by pHandle
typedef void(*pfnComManagerPostRead)(port_handle_t port, p_data_t* dataRead);

// pstAckFnc optional, called after an ACK is received by the serial port represented by pHandle
typedef void(*pfnComManagerPostAck)(port_handle_t port, p_ack_t* ack, unsigned char packetIdentifier);

// disableBcastFnc optional, mostly for internal use, this can be left as 0 or NULL.  Set port to -1 for all ports.
typedef void(*pfnComManagerDisableBroadcasts)(port_handle_t port);

// Called right before data is to be sent.  Data is not sent if this callback returns 0.
typedef int(*pfnComManagerPreSend)(port_handle_t port, p_data_hdr_t *dataHdr);

// Generic message handler function, return 1 if message handled
typedef int(*pfnComManagerGenMsgHandler)(port_handle_t port, const unsigned char* msg, int msgSize);

// Parse error handler function, return 1 if message handled
typedef int(*pfnComManagerParseErrorHandler)(port_handle_t port, is_comm_instance_t* comm);

// "Global" handler for any Binary Data which does not have an explicit handler (registerDid)
typedef int(*pfnComManagerBinaryDataHandler)(port_handle_t port, p_data_t* data);

// broadcast message handler
typedef int(*pfnComManagerAsapMsg)(port_handle_t port, p_data_get_t* req);

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

/*
typedef struct
{
	// Comm instances 
	is_comm_instance_t comm;

	// Comm instance data buffer
	uint8_t comm_buffer[PKT_BUF_SIZE];

#if ENABLE_PACKET_CONTINUATION

	// Continuation data for packets
	p_data_t con;

#endif
	
} com_manager_port_t;
*/

typedef std::array<broadcast_msg_t, MAX_NUM_BCAST_MSGS> broadcast_msg_array_t;
typedef struct
{
	// reads n bytes into buffer from the source (usually a serial port)
	pfnComManagerRead portRead;

	// write data to the destination (usually a serial port)
	pfnIsCommPortWrite portWrite;

	// bytes free in Tx buffer (used to check if packet, keeps us from overflowing the Tx buffer)
	pfnComManagerSendBufferAvailableBytes txFree;

	// Callback function pointer, used to respond to data input
	pfnComManagerPostRead pstRxFnc;

	// Callback function pointer, used to respond to ack
	pfnComManagerPostAck pstAckFnc;

	// Callback function pointer to disable broadcasts on specified port, or all ports if port is -1
	pfnComManagerDisableBroadcasts disableBcastFnc;

	// Pointer to local data and data specific callback functions  ::  NOTE: https://howardhinnant.github.io/stack_alloc.html  if using this in embedded environments and dynamic allocation is a concern
	std::map<int, registered_data_t> didRegistrationMap;
	
	// Array of port
    std::vector<port_handle_t> ports;
	//com_manager_port_t *ports;

	// Number of communication ports
	// int32_t numPorts;

    broadcast_msg_array_t* broadcastMessages; // MAX_NUM_BCAST_MSGS slots

	// processing interval
	int32_t stepPeriodMilliseconds;

	// user defined pointer
	void* userPointer;

    // "Global" handler for any Binary Data which does not have an explicit handler (registerDid)
    pfnComManagerBinaryDataHandler cmMsgHandleDID;

	// Broadcast message handler.  Called whenever we get a message broadcast request or message disable command.
	pfnComManagerAsapMsg cmMsgHandlerRmc;

	// Message handler - NMEA
	pfnComManagerGenMsgHandler cmMsgHandlerNmea;

	// Message handler - Ublox
	pfnComManagerGenMsgHandler cmMsgHandlerUblox;

	// Message handler - RTCM3
	pfnComManagerGenMsgHandler cmMsgHandlerRtcm3;
	
	// Message handler - SPARTN
	pfnComManagerGenMsgHandler cmMsgHandlerSpartn;

    // Error handler
	pfnComManagerParseErrorHandler cmMsgHandlerError;

} com_manager_t;


/**
* Initializes the default global com manager. This is generally only called once on program start.
* The global com manager is used by the functions that do not have the Instance suffix and first parameter of void* cmInstance.
* The instance functions can be ignored, unless you have a reason to have two com managers in the same process.
*
* @param numPorts the max number of serial ports possible
* @param stepPeriodMilliseconds how many milliseconds you are waiting in between calls to comManagerStep
* @param readFnc read data from the serial port represented by pHandle
* @param sendFnc send data to the serial port represented by pHandle
* @param txFreeFnc optional, return the number of free bytes in the send buffer for the serial port represented by pHandle
* @param pstRxFnc optional, called after new data is available (successfully parsed and checksum passed) from the serial port represented by pHandle
* @param pstAckFnc optional, called after an ACK is received by the serial port represented by pHandle
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
        port_handle_t port,
        int stepPeriodMilliseconds,
        pfnComManagerRead readFnc,
        pfnIsCommPortWrite sendFnc,
        pfnComManagerSendBufferAvailableBytes txFreeFnc,
        pfnComManagerPostRead pstRxFnc,
        pfnComManagerPostAck pstAckFnc,
        pfnComManagerDisableBroadcasts disableBcastFnc,
        std::array<broadcast_msg_t, MAX_NUM_BCAST_MSGS>* buffers);   //! was: com_manager_init_t *buffers,

/**
* Performs one round of sending and receiving message. Call as frequently as needed to send and receive data.
* @param timeMs current time in milliseconds used for paser timeout.  Used to invalidate packet parsing if PKT_PARSER_TIMEOUT_MS time has lapsed since any data has been received.
*/
void comManagerStepTimeout(uint32_t timeMs);

void comManagerStep();

void stepSendMessages(void);



/**
* Make a request to a port handle to broadcast a piece of data at a set interval.
*
* @param pHandle the port handle to request broadcast data from
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
* @param pHandle the port handle to request broadcast data from
* @param RMC bits specifying data messages to stream.  See presets: RMC_PRESET_PPD_BITS = post processing data, RMC_PRESET_INS_BITS = INS2 and GPS data at full rate
* @param RMC options to enable data streaming on ports other than the current port.
* @param offset offset into the structure for the data id to broadcast - pass offset and size of 0 to receive the entire data set
* @param size number of bytes in the data structure from offset to broadcast - pass offset and size of 0 to receive the entire data set
* @param periodMultiple the data broadcast period in multiples of the base update period
*
* Example that enables streaming of all data messages necessary for post processing:
* @code
* comManagerGetDataRmc(pHandle, RMC_PRESET_PPD_BITS, 0);
* @endcode
*
* Example that broadcasts INS and GPS data at full rate:
* @code
* comManagerGetDataRmc(pHandle, RMC_PRESET_INS_BITS, 0);
* @endcode
*/
void comManagerGetDataRmc(port_handle_t port, uint64_t rmcBits, uint32_t rmcOptions);

/**
* Disable a broadcast for a specified port handle and data identifier
*
* @param pHandle the port handle to disable a broadcast for
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
* @param pHandle the port handle to send the packet to
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
* comManagerSend(pHandle, PKT_TYPE_GET_DATA, 0, &data)
* @endcode
*/
int comManagerSend(port_handle_t port, uint8_t pFlags, void *data, uint16_t did, uint16_t size, uint16_t offset);

/**
* Convenience function that wraps comManagerSend for sending data structures.  Must be multiple of 4 bytes in size.
*
* @param pHandle the port handle to send data to
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
int comManagerSendData(port_handle_t port, void* data, uint16_t did, uint16_t size, uint16_t offset);

// INTERNAL FUNCTIONS...
/**
* Same as comManagerSend, except that no retry is attempted
*
* @param pHandle the port handle to send the packet to
* @param dataId Data structure ID number.
* @param dataPtr Pointer to actual data.
* @param dataSize Size of data to send in number of bytes.
* @param dataOffset Offset into data structure where copied data starts.
* @param pFlags Additional packet flags if needed.
* @return 0 if success, anything else if failure
*/
int comManagerSendDataNoAck(port_handle_t port, void *data, uint16_t did, uint16_t size, uint16_t offset);

/**
* Convenience function that wraps comManagerSend for sending data structures.  Allows arbitrary bytes size, 4 byte multiple not required.
* No byte swapping occurs.
*
* @param pHandle the port handle to send data to
* @param dataId the data id of the data to send
* @param dataPtr pointer to the data structure to send
* @param dataSize number of bytes to send
* @param dataOffset offset into dataPtr to send at
* @return 0 if success, anything else if failure
*
* Example:
* @code
* comManagerSendRawData(0, DID_DEV_INFO, &g_devInfo, sizeof(dev_info_t), 0);
* @endcode
*/
int comManagerSendRawData(port_handle_t port, void* data, uint16_t did, uint16_t size, uint16_t offset);

/**
* Write bare data directly to the serial port.
*
* @param pHandle the port handle to send data to
* @param dataPtr pointer to the data structure to send
* @param dataSize number of bytes to send
* @return 0 if success, anything else if failure
*
* Example:
* @code
* comManagerSendRaw(0, &g_devInfo, sizeof(dev_info_t));
* @endcode
*/
int comManagerSendRaw(port_handle_t port, void* dataPtr, int dataSize);


/**
* Disables broadcasts of all messages on specified port, or all ports if phandle == -1.
* @param pHandle the pHandle to disable broadcasts on, -1 for all
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

/**
* Register message handler callback functions.  Pass in NULL to disable any of these callbacks.
*
* @param rmcHandler handler for Realtime Message Controller (RMC) called whenever we get a message broadcast request or message disable command.
* @param asciiHandler handler for NMEA messages.
* @param ubloxHandler handler for ublox messages.
* @param rtcm3Handler handler for RTCM3 messages.
* @param spartnHandler handler for SPARTN messages.
* @param handlerError handler for parse errors.
*/
void comManagerSetCallbacks(
        pfnComManagerAsapMsg rmcHandler,
        pfnComManagerGenMsgHandler asciiHandler,
        pfnComManagerGenMsgHandler ubloxHandler,
        pfnComManagerGenMsgHandler rtcm3Handler,
        pfnComManagerGenMsgHandler spartnHandler,
        pfnComManagerParseErrorHandler handlerError);

void comManagerSetBinaryDataCallback(
        pfnComManagerBinaryDataHandler binaryDataHandler);

#ifdef __cplusplus
}
#endif


class ISComManager : com_manager_t {
public:
    void step();
    void stepRx(uint32_t timeMs);
    int stepRxHandler(comm_port_t* port, protocol_type_t ptype);

    void stepTx();


    /**
    * Initializes the default global com manager. This is generally only called once on program start.
    * The global com manager is used by the functions that do not have the Instance suffix and first parameter of void* cmInstance.
    * The instance functions can be ignored, unless you have a reason to have two com managers in the same process.
    *
    * @param numPorts the max number of serial ports possible
    * @param stepPeriodMilliseconds how many milliseconds you are waiting in between calls to comManagerStep
    * @param readFnc read data from the serial port represented by pHandle
    * @param sendFnc send data to the serial port represented by pHandle
    * @param txFreeFnc optional, return the number of free bytes in the send buffer for the serial port represented by pHandle
    * @param pstRxFnc optional, called after new data is available (successfully parsed and checksum passed) from the serial port represented by pHandle
    * @param pstAckFnc optional, called after an ACK is received by the serial port represented by pHandle
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
            port_handle_t port,
            int stepPeriodMilliseconds,
            pfnComManagerRead readFnc,
            pfnIsCommPortWrite sendFnc,
            pfnComManagerSendBufferAvailableBytes txFreeFnc,
            pfnComManagerPostRead pstRxFnc,
            pfnComManagerPostAck pstAckFnc,
            pfnComManagerDisableBroadcasts disableBcastFnc,
            std::array<broadcast_msg_t, MAX_NUM_BCAST_MSGS>* buffers);   //! was: com_manager_init_t *buffers,

    /**
    * Performs one round of sending and receiving message. Call as frequently as needed to send and receive data.
    * @param timeMs current time in milliseconds used for paser timeout.  Used to invalidate packet parsing if PKT_PARSER_TIMEOUT_MS time has lapsed since any data has been received.
    */
    void stepTimeout(uint32_t timeMs);


    /**
    * Make a request to a port handle to broadcast a piece of data at a set interval.
    *
    * @param pHandle the port handle to request broadcast data from
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
    * @param pHandle the port handle to request broadcast data from
    * @param RMC bits specifying data messages to stream.  See presets: RMC_PRESET_PPD_BITS = post processing data, RMC_PRESET_INS_BITS = INS2 and GPS data at full rate
    * @param RMC options to enable data streaming on ports other than the current port.
    * @param offset offset into the structure for the data id to broadcast - pass offset and size of 0 to receive the entire data set
    * @param size number of bytes in the data structure from offset to broadcast - pass offset and size of 0 to receive the entire data set
    * @param periodMultiple the data broadcast period in multiples of the base update period
    *
    * Example that enables streaming of all data messages necessary for post processing:
    * @code
    * comManagerGetDataRmc(pHandle, RMC_PRESET_PPD_BITS, 0);
    * @endcode
    *
    * Example that broadcasts INS and GPS data at full rate:
    * @code
    * comManagerGetDataRmc(pHandle, RMC_PRESET_INS_BITS, 0);
    * @endcode
    */
    void getDataRmc(port_handle_t port, uint64_t rmcBits, uint32_t rmcOptions);

    /**
    * Disable a broadcast for a specified port handle and data identifier
    *
    * @param pHandle the port handle to disable a broadcast for
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
    * @param pHandle the port handle to send the packet to
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
    * comManagerSend(pHandle, PKT_TYPE_GET_DATA, 0, &data)
    * @endcode
    */
    int send(port_handle_t port, uint8_t pFlags, void *data, uint16_t did, uint16_t size, uint16_t offset);

    /**
    * Convenience function that wraps comManagerSend for sending data structures.  Must be multiple of 4 bytes in size.
    *
    * @param pHandle the port handle to send data to
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
    int sendData(port_handle_t port, void* data, uint16_t did, uint16_t size, uint16_t offset);

    // INTERNAL FUNCTIONS...
    /**
    * Same as comManagerSend, except that no retry is attempted
    *
    * @param pHandle the port handle to send the packet to
    * @param dataId Data structure ID number.
    * @param dataPtr Pointer to actual data.
    * @param dataSize Size of data to send in number of bytes.
    * @param dataOffset Offset into data structure where copied data starts.
    * @param pFlags Additional packet flags if needed.
    * @return 0 if success, anything else if failure
    */
    int sendDataNoAck(port_handle_t port, void *data, uint16_t did, uint16_t size, uint16_t offset);

    /**
    * Convenience function that wraps comManagerSend for sending data structures.  Allows arbitrary bytes size, 4 byte multiple not required.
    * No byte swapping occurs.
    *
    * @param pHandle the port handle to send data to
    * @param dataId the data id of the data to send
    * @param dataPtr pointer to the data structure to send
    * @param dataSize number of bytes to send
    * @param dataOffset offset into dataPtr to send at
    * @return 0 if success, anything else if failure
    *
    * Example:
    * @code
    * comManagerSendRawData(0, DID_DEV_INFO, &g_devInfo, sizeof(dev_info_t), 0);
    * @endcode
    */
    int sendRawData(port_handle_t port, void* data, uint16_t did, uint16_t size, uint16_t offset);

    /**
    * Write bare data directly to the serial port.
    *
    * @param pHandle the port handle to send data to
    * @param dataPtr pointer to the data structure to send
    * @param dataSize number of bytes to send
    * @return 0 if success, anything else if failure
    *
    * Example:
    * @code
    * comManagerSendRaw(0, &g_devInfo, sizeof(dev_info_t));
    * @endcode
    */
    int sendRaw(port_handle_t port, void* dataPtr, int dataSize);


    /**
    * Disables broadcasts of all messages on specified port, or all ports if phandle == -1.
    * @param pHandle the pHandle to disable broadcasts on, -1 for all
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
    * Register message handler callback functions.  Pass in NULL to disable any of these callbacks.
    *
    * @param rmcHandler handler for Realtime Message Controller (RMC) called whenever we get a message broadcast request or message disable command.
    * @param asciiHandler handler for NMEA messages.
    * @param ubloxHandler handler for ublox messages.
    * @param rtcm3Handler handler for RTCM3 messages.
    * @param spartnHandler handler for SPARTN messages.
    * @param handlerError handler for parse errors.
    */
    void setCallbacks(
            pfnComManagerAsapMsg rmcHandler,
            pfnComManagerGenMsgHandler asciiHandler,
            pfnComManagerGenMsgHandler ubloxHandler,
            pfnComManagerGenMsgHandler rtcm3Handler,
            pfnComManagerGenMsgHandler spartnHandler,
            pfnComManagerParseErrorHandler handlerError);

    void setBinaryDataCallback(
            pfnComManagerBinaryDataHandler binaryDataHandler);

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

private:
// int processAsciiRxPacket(com_manager_t* cmInstance, port_handle_t port, unsigned char* start, int count);
// void parseAsciiPacket(com_manager_t* cmInstance, port_handle_t port, unsigned char* buf, int count);
    int processBinaryRxPacket(port_handle_t port, packet_t *pkt);
    void enableBroadcastMsg(broadcast_msg_t *msg, int periodMultiple);
    void disableBroadcastMsg(broadcast_msg_t *msg);
    void disableDidBroadcast(port_handle_t port, uint16_t did);
    int sendDataPacket(port_handle_t port, packet_t *pkt);
    void sendAck(port_handle_t port, packet_t *pkt, uint8_t pTypeFlags);

    int findAsciiMessage(const void * a, const void * b);
    int asciiMessageCompare(const void* elem1, const void* elem2);
};

extern ISComManager s_cm;

// com manager instance / handle is a void*
typedef ISComManager* CMHANDLE;

// get the global instance of the com manager - this is only needed if you are working with multiple com managers and need to compare instances
CMHANDLE comManagerGetGlobal(void);

#endif // COM_MANAGER_H
