/*
MIT LICENSE

Copyright (c) 2014-2025 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#ifndef COM_MANAGER_H
#define COM_MANAGER_H

#ifdef __cplusplus
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
	int32_t                 pHandle;
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
typedef struct
{
	broadcast_msg_t* broadcastMsg;
	uint32_t broadcastMsgSize;			// MAX_NUM_BCAST_MSGS * sizeof(broadcast_msg_t)
} com_manager_init_t;

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

// com manager instance / handle is a void*
typedef void* CMHANDLE;

// com manager callback prototypes
// txFreeFnc optional, return the number of free bytes in the send buffer for the serial port represented by pHandle
typedef int(*pfnComManagerSendBufferAvailableBytes)(unsigned int port);

// pstRxFnc optional, called after data is sent to the serial port represented by pHandle
typedef int(*pfnComManagerPostRead)(unsigned int port, p_data_t* dataRead);

// pstAckFnc optional, called after an ACK is received by the serial port represented by pHandle
typedef int(*pfnComManagerPostAck)(unsigned int port, p_ack_t* ack, unsigned char packetIdentifier);

// disableBcastFnc optional, mostly for internal use, this can be left as 0 or NULL.  Set port to -1 for all ports.
typedef int(*pfnComManagerDisableBroadcasts)(int port);

// Called right before data is to be sent.  Data is not sent if this callback returns 0.
typedef int(*pfnComManagerPreSend)(unsigned int port, p_data_hdr_t *dataHdr);

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
	
typedef struct
{
	// reads n bytes into buffer from the source (usually a serial port)
	pfnIsCommPortRead portRead;

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

	// Pointer to local data and data specific callback functions
	registered_data_t regData[DID_COUNT];
	
	// Array of port
	com_manager_port_t *ports;

	// Number of communication ports
	int32_t numPorts;

	broadcast_msg_t* broadcastMessages; // MAX_NUM_BCAST_MSGS slots

	// processing interval
	int32_t stepPeriodMilliseconds;

	// user defined pointer
	void* userPointer;

	// Message handlers
	is_comm_callbacks_t callbacks;

} com_manager_t;


// get the global instance of the com manager - this is only needed if you are working with multiple com managers and need to compare instances
CMHANDLE comManagerGetGlobal(void);

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
int comManagerInit
(	int numPorts,
	int stepPeriodMilliseconds,
	pfnIsCommPortRead readFnc,
	pfnIsCommPortWrite sendFnc,
	pfnComManagerSendBufferAvailableBytes txFreeFnc,
	pfnComManagerPostRead pstRxFnc,
	pfnComManagerPostAck pstAckFnc,
	pfnComManagerDisableBroadcasts disableBcastFnc,
	com_manager_init_t *buffers,
	com_manager_port_t *cmPorts,
	is_comm_callbacks_t *callbacks);

// Initialize an instance to a com manager that can be passed to instance functions and can later be freed with freeComManagerInstance
// this function may be called multiple times.  Return 0 on success, -1 on failure.
int comManagerInitInstance
(	CMHANDLE cmHandle,
	int numPorts,
	int stepPeriodMilliseconds,
	pfnIsCommPortRead readFnc,
	pfnIsCommPortWrite sendFnc,
	pfnComManagerSendBufferAvailableBytes txFreeFnc,
	pfnComManagerPostRead pstRxFnc,
	pfnComManagerPostAck pstAckFnc,
	pfnComManagerDisableBroadcasts disableBcastFnc,
	com_manager_init_t *buffers,
	com_manager_port_t *cmPorts,
	is_comm_callbacks_t *callbacks);

/**
* Performs one round of sending and receiving message. Call as frequently as needed to send and receive data.
* 
* @param timeMs current time in milliseconds used for paser timeout.  Used to invalidate packet parsing if PKT_PARSER_TIMEOUT_MS time has lapsed since any data has been received.  
*/
void comManagerStep(void);
void comManagerStepTimeout(uint32_t timeMs);
void comManagerStepInstance(CMHANDLE cmInstance_);
void comManagerStepRxInstance(CMHANDLE cmInstance, uint32_t timeMs);
void comManagerStepTxInstance(CMHANDLE cmInstance);

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
void comManagerGetData(int port, uint16_t did, uint16_t size, uint16_t offset, uint16_t period);
void comManagerGetDataInstance(CMHANDLE cmInstance, int port, uint16_t did, uint16_t size, uint16_t offset, uint16_t period);

/**
* Make a request to a port handle to broadcast a piece of data at a set interval.
* 
* @param pHandle the port handle to request broadcast data from
* @param RMC bits specifying data messages to stream.  See presets: RMC_PRESET_IMX_PPD = post processing data, RMC_PRESET_INS = INS2 and GPS data at full rate
* @param RMC options to enable data streaming on ports other than the current port. 
* @param offset offset into the structure for the data id to broadcast - pass offset and size of 0 to receive the entire data set
* @param size number of bytes in the data structure from offset to broadcast - pass offset and size of 0 to receive the entire data set
* @param periodMultiple the data broadcast period in multiples of the base update period
* 
* Example that enables streaming of all data messages necessary for post processing:
* @code
* comManagerGetDataRmc(pHandle, RMC_PRESET_IMX_PPD, 0);
* @endcode
* 
* Example that broadcasts INS and GPS data at full rate:
* @code
* comManagerGetDataRmc(pHandle, RMC_PRESET_INS, 0);
* @endcode
*/
void comManagerGetDataRmc(int port, uint64_t rmcBits, uint32_t rmcOptions);
void comManagerGetDataRmcInstance(CMHANDLE cmInstance, int port, uint64_t rmcBits, uint32_t rmcOptions);

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
int comManagerDisableData(int port, uint16_t did);
int comManagerDisableDataInstance(CMHANDLE cmInstance, int port, uint16_t did);

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
int comManagerSend(int port, uint8_t pFlags, void *data, uint16_t did, uint16_t size, uint16_t offset);
int comManagerSendInstance(CMHANDLE cmInstance, int port, uint8_t pFlags, void *data, uint16_t did, uint16_t size, uint16_t offset);

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
int comManagerSendData(int port, void* data, uint16_t did, uint16_t size, uint16_t offset);
int comManagerSendDataInstance(CMHANDLE cmInstance, int port, void* data, uint16_t did, uint16_t size, uint16_t offset);

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
int comManagerSendDataNoAck(int port, void *data, uint16_t did, uint16_t size, uint16_t offset);
int comManagerSendDataNoAckInstance(CMHANDLE cmInstance, int port, void *data, uint16_t did, uint16_t size, uint16_t offset);

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
int comManagerSendRawData(int port, void* data, uint16_t did, uint16_t size, uint16_t offset);
int comManagerSendRawDataInstance(CMHANDLE cmInstance, int port, void* data, uint16_t did, uint16_t size, uint16_t offset);

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
int comManagerSendRaw(int pHandle, void* dataPtr, int dataSize);
int comManagerSendRawInstance(CMHANDLE cmInstance, int pHandle, void* dataPtr, int dataSize);

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
int comManagerSendRaw(int pHandle, void* dataPtr, int dataSize);
int comManagerSendRawInstance(CMHANDLE cmInstance, int pHandle, void* dataPtr, int dataSize);

/**
* Disables broadcasts of all messages on specified port, or all ports if phandle == -1.
* @param pHandle the pHandle to disable broadcasts on, -1 for all
*/
void comManagerDisableBroadcasts(int port);
void comManagerDisableBroadcastsInstance(CMHANDLE cmInstance, int port);

/**
* Get the ISComm structure. 
* 
* @return com manager ISComm structure, this pointer is owned by the com manager
*/
is_comm_instance_t* comManagerGetIsComm(int port);
is_comm_instance_t* comManagerGetIsCommInstance(CMHANDLE cmInstance, int port);

/**
* Internal use mostly, get data info for a the specified pre-registered dataId
* 
* @return 0 on failure, pointer on success
*/
bufTxRxPtr_t* comManagerGetRegisteredDataInfo(uint16_t did);
bufTxRxPtr_t* comManagerGetRegisteredDataInfoInstance(CMHANDLE cmInstance, uint16_t did);

/**
* Internal use mostly, process a get data request for a message that needs to be broadcasted
* 
* @return 0 on success, anything else is failure
*/
int comManagerGetDataRequest(int port, p_data_get_t* req);
int comManagerGetDataRequestInstance(CMHANDLE cmInstance, int port, p_data_get_t* req);

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
void comManagerRegisterInstance(CMHANDLE cmInstance, uint16_t did, pfnComManagerPreSend txFnc, pfnComManagerPostRead pstRxFnc, const void* txDataPtr, void* rxDataPtr, uint16_t size, uint8_t pktFlags);

/**
* Attach user defined data to a com manager instance
*/
void comManagerAssignUserPointer(CMHANDLE cmInstance, void* userPointer);

/**
* Get user defined data to from a com manager instance
*/
void* comManagerGetUserPointer(CMHANDLE cmInstance);

/**
* Ensure baudrate is valid for InertialSense hardware
* @param baudRate the baud rate to check
* @return 0 if baud rate is valid, -1 if not
*/
int comManagerValidateBaudRate(unsigned int baudRate);

#ifdef __cplusplus
}
#endif

#endif // COM_MANAGER_H
