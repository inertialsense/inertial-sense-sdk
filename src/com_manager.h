/*
MIT LICENSE

Copyright (c) 2014-2019 Inertial Sense, Inc. - http://inertialsense.com

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

/**
Types of pass through data where the com manager will simply forward the data on to a pass through handler
*/
typedef enum
{
	/** No pass through */
	COM_MANAGER_PASS_THROUGH_NONE = 0,

	/** UBLOX pass through */
	COM_MANAGER_PASS_THROUGH_UBLOX = UBLOX_START_BYTE1,

	/** RTCM3 pass through */
	COM_MANAGER_PASS_THROUGH_RTCM3 = RTCM3_START_BYTE
} com_manager_pass_through_t;

/** Contains status for the com manager */
typedef struct  
{
	/** bytes read without a valid message */
	uint32_t readCounter;

	/** 0 if no errors encountered, otherwise non-zero */
	uint32_t rxError;

	/** Number of packets read successfully */
	uint32_t rxPktCount;

	/** Number of packets written */
	uint32_t txPktCount;

	/** number of communication errors - can be reset to 0 if desired */
	uint32_t communicationErrorCount;

	/**
	flags to send on each request - do not modify
	valid data : flags & CM_PKT_FLAGS_RX_VALID_DATA
	*/
	uint8_t flags;

	/** keep track of the start byte for binary parsing - do not modify */
	uint8_t startByte;

	/** keep track of the number of pass through bytes remaining - do not modify */
	uint32_t passThroughBytes;
} com_manager_status_t;

// com manager instance / handle is a void*
typedef void* CMHANDLE;

// com manager callback prototypes
// readFnc read data from the serial port represented by pHandle - return number of bytes read
typedef int  (*pfnComManagerRead)(CMHANDLE cmHandle, int pHandle, unsigned char* readIntoBytes, int numberOfBytes);

// sendFnc send data to the serial port represented by pHandle - return number of bytes written
typedef int  (*pfnComManagerSend)(CMHANDLE cmHandle, int pHandle, buffer_t* bufferToSend);

// txFreeFnc optional, return the number of free bytes in the send buffer for the serial port represented by pHandle
typedef int  (*pfnComManagerSendBufferAvailableBytes)(CMHANDLE cmHandle, int pHandle);

// pstRxFnc optional, called after data is sent to the serial port represented by pHandle
typedef void (*pfnComManagerPostRead)(CMHANDLE cmHandle, int pHandle, p_data_t* dataRead);

// pstAckFnc optional, called after an ACK is received by the serial port represented by pHandle
typedef void (*pfnComManagerPostAck)(CMHANDLE cmHandle, int pHandle, p_ack_t* ack, unsigned char packetIdentifier);

// disableBcastFnc optional, mostly for internal use, this can be left as 0 or NULL
typedef void (*pfnComManagerDisableBroadcasts)(CMHANDLE cmHandle, int pHandle);

// called right before data is sent
typedef void (*pfnComManagerPreSend)(CMHANDLE cmHandle, int pHandle);

// ASCII message handler function, return 1 if message handled
typedef int  (*pfnComManagerAsciiMessageHandler)(CMHANDLE cmHandle, int pHandle, unsigned char* messageId, unsigned char* line, int lineLength);

// pass through handler
typedef void (*pfnComManagerPassThrough)(CMHANDLE cmHandle, com_manager_pass_through_t passThroughType, int pHandle, const unsigned char* data, int dataLength);

// broadcast message handler
typedef int (*pfnComManagerAsapMsg)(CMHANDLE cmHandle, int pHandle, p_data_get_t* req);

// get the global instance of the com manager - this is only needed if you are working with multiple com managers and need to compare instances
CMHANDLE comManagerGetGlobal(void);

/**
Initializes the default global com manager. This is generally only called once on program start.
The global com manager is used by the functions that do not have the Instance suffix and first parameter of void* cmInstance.
The instance functions can be ignored, unless you have a reason to have two com managers in the same process.

@param numHandles the max number of serial ports possible
@param maxEnsuredPackets the max number of ensured packets
@param stepPeriodMilliseconds how many milliseconds you are waiting in between calls to comManagerStep
@param retryCount the number of times to retry failed packets
@param readFnc read data from the serial port represented by pHandle
@param sendFnc send data to the serial port represented by pHandle
@param txFreeFnc optional, return the number of free bytes in the send buffer for the serial port represented by pHandle
@param pstRxFnc optional, called after new data is available (successfully parsed and checksum passed) from the serial port represented by pHandle
@param pstAckFnc optional, called after an ACK is received by the serial port represented by pHandle
@param disableBcastFnc mostly for internal use, this can be left as 0 or NULL

Example:
@code
comManagerInit(20, 20, 10, 25, staticReadPacket, staticSendPacket, NULL, staticProcessRxData, staticProcessAck, 0);
@endcode
*/
void comManagerInit
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
);

// Initialize and return an instance to a com manager that can be passed to instance functions and can later be freed with freeComManagerInstance
// this function may be called multiple times
CMHANDLE comManagerInitInstance
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
);

/**
Performs one round of sending and receiving message. This should be called as often as you want to send and receive data.
*/
void comManagerStep(void);
void comManagerStepInstance(CMHANDLE cmInstance);

/**
Make a request to a port handle to broadcast a piece of data at a set interval.

@param pHandle the port handle to request broadcast data from
@param dataId the data id to broadcast
@param offset offset into the structure for the data id to broadcast - pass offset and size of 0 to receive the entire data set
@param size number of bytes in the data structure from offset to broadcast - pass offset and size of 0 to receive the entire data set
@param periodMultiple the data broadcast period in multiples of the base update period

Example that makes a request to receive the device info just once:
@code
comManagerGetData(0, DID_DEV_INFO, 0, sizeof(dev_info_t), 0);
@endcode

Example that broadcasts INS data every 50 milliseconds:
@code
comManagerGetData(0, DID_INS_1, 0, sizeof(ins_1_t), 50);
@endcode
*/
void comManagerGetData(int pHandle, uint32_t dataId, int offset, int size, int periodMultiple);
void comManagerGetDataInstance(CMHANDLE cmInstance, int pHandle, uint32_t dataId, int offset, int size, int periodMultiple);

/**
Make a request to a port handle to broadcast a piece of data at a set interval.

@param pHandle the port handle to request broadcast data from
@param RMC bits specifying data messages to stream.  See presets: RMC_PRESET_PPD_BITS = post processing data, RMC_PRESET_INS_BITS = INS2 and GPS data at full rate
@param RMC options to enable data streaming on ports other than the current port. 
@param offset offset into the structure for the data id to broadcast - pass offset and size of 0 to receive the entire data set
@param size number of bytes in the data structure from offset to broadcast - pass offset and size of 0 to receive the entire data set
@param periodMultiple the data broadcast period in multiples of the base update period

Example that enables streaming of all data messages necessary for post processing:
@code
comManagerGetDataRmc(pHandle, RMC_PRESET_PPD_BITS, 0);
@endcode

Example that broadcasts INS and GPS data at full rate:
@code
comManagerGetDataRmc(pHandle, RMC_PRESET_INS_BITS, 0);
@endcode
*/
void comManagerGetDataRmc(int pHandle, uint64_t rmcBits, uint32_t rmcOptions);
void comManagerGetDataRmcInstance(CMHANDLE cmInstance, int pHandle, uint64_t rmcBits, uint32_t rmcOptions);

/**
Disable a broadcast for a specified port handle and data identifier

@param pHandle the port handle to disable a broadcast for
@param dataId the data id to disable boradcast for
@return 0 if success, anything else if failure

Example:
@code
comManagerDisableData(0, DID_INS_1);
@endcode
*/
int comManagerDisableData(int pHandle, uint32_t dataId);
int comManagerDisableDataInstance(CMHANDLE cmInstance, int pHandle, uint32_t dataId);

/**
Send a packet to a port handle

@param pHandle the port handle to send the packet to
@param pktInfo the type of packet (PID)
@param bodyHdr optional, can contain information about the actual data of the body (txData), usually the data id, offset and size
@param txData optional, the actual body of the packet
@return 0 if success, anything else if failure

Example:
@code
p_data_get_t request;
bufPtr_t data;
request.id = DID_INS_1;
request.offset = 0;
request.size = sizeof(ins_1_t);
request.bc_period_ms = 50;
data.ptr = (uint8_t*)&request;
data.size = sizeof(request);
comManagerSend(pHandle, PID_GET_DATA, 0, &data)
@endcode
*/
int comManagerSend(int pHandle, uint8_t pktInfo, bufPtr_t* bodyHdr, bufPtr_t* txData, uint8_t pktFlags);
int comManagerSendInstance(CMHANDLE cmInstance, int pHandle, uint8_t pktInfo, bufPtr_t* bodyHdr, bufPtr_t* txData, uint8_t pktFlags);

/**
Convenience function that wraps comManagerSend for sending data structures.  Must be multiple of 4 bytes in size.

@param pHandle the port handle to send data to
@param dataId the data id of the data to send
@param dataPtr pointer to the data structure to send
@param dataSize number of bytes to send
@param dataOffset offset into dataPtr to send at
@return 0 if success, anything else if failure

Example:
@code
comManagerSendData(0, DID_DEV_INFO, &g_devInfo, sizeof(dev_info_t), 0);
@endcode
*/
int comManagerSendData(int pHandle, uint32_t dataId, void* dataPtr, int dataSize, int dataOffset);
int comManagerSendDataInstance(CMHANDLE cmInstance, int pHandle, uint32_t dataId, void* dataPtr, int dataSize, int dataOffset);

/**
Same as comManagerSend, except that the com manager may retry the send if an ACK is not received

@param pHandle the port handle to send the packet to
@param pktInfo the type of packet (PID)
@param bodyHdr optional, can contain information about the actual data of the body (txData), usually the data id, offset and size
@param txData optional, the actual body of the packet
@return 0 if success, anything else if failure
*/
int comManagerSendEnsured(int pHandle, uint8_t pktInfo, unsigned char *data, unsigned int dataSize);
int comManagerSendEnsuredInstance(CMHANDLE cmInstance, int pHandle, uint8_t pktInfo, unsigned char *data, unsigned int dataSize);

// INTERNAL FUNCTIONS...
/**
Same as comManagerSend, except that no retry is attempted

@param pHandle the port handle to send the packet to
@param dataId Data structure ID number.
@param dataPtr Pointer to actual data.
@param dataSize Size of data to send in number of bytes.
@param dataOffset Offset into data structure where copied data starts.
@param pFlags Additional packet flags if needed.
@return 0 if success, anything else if failure
*/
int comManagerSendDataNoAck(int pHandle, uint32_t dataId, void* dataPtr, int dataSize, int dataOffset);
int comManagerSendDataNoAckInstance(CMHANDLE cmInstance, int pHandle, uint32_t dataId, void* dataPtr, int dataSize, int dataOffset);

/**
Convenience function that wraps comManagerSend for sending data structures.  Allows arbitrary bytes size, 4 byte multiple not required. 
No byte swapping occurs.

@param pHandle the port handle to send data to
@param dataId the data id of the data to send
@param dataPtr pointer to the data structure to send
@param dataSize number of bytes to send
@param dataOffset offset into dataPtr to send at
@return 0 if success, anything else if failure

Example:
@code
comManagerSendRawData(0, DID_DEV_INFO, &g_devInfo, sizeof(dev_info_t), 0);
@endcode
*/
int comManagerSendRawData(int pHandle, uint32_t dataId, void* dataPtr, int dataSize, int dataOffset);
int comManagerSendRawDataInstance(CMHANDLE cmInstance, int pHandle, uint32_t dataId, void* dataPtr, int dataSize, int dataOffset);

/**
Disables broadcasts of all messages on specified port, or all ports if phandle == -1.
@param pHandle the pHandle to disable broadcasts on, -1 for all
*/
void comManagerDisableBroadcasts(int pHandle);
void comManagerDisableBroadcastsInstance(CMHANDLE cmInstance, int pHandle);

/**
Get the most recent status of the com manager

@return com manager status, this pointer is owned by the com manager
*/
com_manager_status_t* comManagerGetStatus(int pHandle);
com_manager_status_t* comManagerGetStatusInstance(CMHANDLE cmInstance, int pHandle);

/**
Internal use mostly, get data info for a the specified pre-registered dataId

@return 0 on failure, pointer on success
*/
bufTxRxPtr_t* comManagerGetRegisteredDataInfo(uint32_t dataId);
bufTxRxPtr_t* comManagerGetRegisteredDataInfoInstance(CMHANDLE cmInstance, uint32_t dataId);

/**
Internal use mostly, process a get data request for a message that needs to be broadcasted

@return 0 on success, anything else is failure
*/
int comManagerGetDataRequest(int pHandle, p_data_get_t* req);
int comManagerGetDataRequestInstance(CMHANDLE cmInstance, int pHandle, p_data_get_t* req);

/**
Register message handling function for a received data id (binary). This is mostly an internal use function,
but can be used if you are implementing your own receiver on a device.

@param dataId the data id to register the handler for
@param txFnc called right before the data is sent
@param pstRxFnc called after data is received for the data id
@param txDataPtr a pointer to the structure in memory of the data to send
@param rxDataPtr a pointer to the structure in memory to copy received data to
@param dataSize size of the data structure in txDataPtr and rxDataPtr
@param pktFlags packet flags, usually 0

Example:
@code
registerComManager(DID_INS_1, prepMsgINS, writeMsgINS, &g_insData, &g_insData, sizeof(ins_1_t));
@endcode
*/
void comManagerRegister(uint32_t dataId, pfnComManagerPreSend txFnc, pfnComManagerPostRead pstRxFnc, const void* txDataPtr, void* rxDataPtr, int dataSize, uint8_t pktFlags);
void comManagerRegisterInstance(CMHANDLE cmInstance, uint32_t dataId, pfnComManagerPreSend txFnc, pfnComManagerPostRead pstRxFnc, const void* txDataPtr, void* rxDataPtr, int dataSize, uint8_t pktFlags);

/**
Register handlers for when ASCII messages are received. This is also an internal function,
but you can implement it if you are writing your own receiver on a device.

@param asciiMessages the ASCII messages to handle, ownership is assumed to be elsewhere (usually this will be a static variable)
@param asciiMessagesCount the number of items in asciiMessages
@param msgFunc handler for messages that were not in the asciiMessages array
@param msgFuncPost handle for messages that were not in the asciiMessages array after they are parsed out
*/
void comManagerRegisterASCII(asciiMessageMap_t* asciiMessages, int asciiMessagesCount, pfnComManagerAsciiMessageHandler msgFnc, pfnComManagerAsciiMessageHandler msgFncPost);
void comManagerRegisterASCIIInstance(CMHANDLE cmInstance, asciiMessageMap_t* asciiMessages, int asciiMessagesCount, pfnComManagerAsciiMessageHandler msgFnc, pfnComManagerAsciiMessageHandler msgFncPost);

/**
Set a com manager pass through handler, handler can be 0 or null to remove the current handler

	// setup pass-through handler for u-blox, rtcm3, etc.  This should be called following comManagerInit().
	comManagerSetPassThrough(passThroughHandler);

    // Example how to handle incoming ublox, rtcm, etc. data
    void passThroughHandler(CMHANDLE handle, com_manager_pass_through_t type, int pHandle, const unsigned char* data, int dataLength)
    {
        if (type == COM_MANAGER_PASS_THROUGH_UBLOX)
        {
            if (g_nvmFlashCfg->RTKCfgBits & RTK_CFG_BITS_RTK_ROVER)
            {
                processUbloxRtkPacket(data, dataLength, pHandle);
            }
        }
        else if (type == COM_MANAGER_PASS_THROUGH_RTCM3 && (g_nvmFlashCfg->RTKCfgBits & RTK_CFG_BITS_RTK_ROVER) != 0)
        {
            processRtcm3RtkPacket(data, dataLength, pHandle);
        }
    }
*/
void comManagerSetPassThrough(pfnComManagerPassThrough handler);
void comManagerSetPassThroughInstance(CMHANDLE cmInstance, pfnComManagerPassThrough handler);

/**
Set a com manager broadcast message handler, handler can be 0 or null to remove the current handler
*/
void comManagerSetRmcCallback(pfnComManagerAsapMsg handler);
void comManagerSetRmcCallbackInstance(CMHANDLE cmInstance, pfnComManagerAsapMsg handler);

/**
Free a com manager instance. This instance should never be used again. The default global com manager can never be freed.
*/
void comManagerFreeInstance(CMHANDLE cmInstance);

/**
Attach user defined data to a com manager instance
*/
void comManagerAssignUserPointer(CMHANDLE cmInstance, void* userPointer);

/**
Get user defined data to from a com manager instance
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