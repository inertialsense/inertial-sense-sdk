/*
MIT LICENSE

Copyright (c) 2014-2025 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

/**
 * @file com_manager.h
 * @brief ISComManager - port management, broadcast scheduling, and packet dispatch.
 *
 * The com manager owns a set of port handles and drives the IS SDK communication loop.
 * Its responsibilities include:
 *  - Periodic broadcast scheduling (sending data at configured intervals via @ref comManagerGetData).
 *  - Dispatching received ISB and non-ISB packets to registered handler callbacks.
 *  - Managing the per-DID registration table (pre-send and post-receive hooks).
 *
 * The global @ref ISComManager instance is @ref s_cm.  The C-style free functions
 * (comManagerInit, comManagerStep, comManagerGetData, etc.) all delegate to @ref s_cm
 * and are provided for C and legacy C++ callers.
 *
 * @author Inertial Sense, Inc.
 * @copyright Copyright (c) 2014-2026 Inertial Sense, Inc. - http://inertialsense.com
 */

#ifndef COM_MANAGER_H
#define COM_MANAGER_H

#include <array>
#include <cstdint>
#include <list>
#include <map>
#include <set>
#include <vector>

#include "ISComm.h"
#include "linked_list.h"

/** Maximum number of messages that may be broadcast simultaneously, per port.
Since most messages use the RMC (real-time message controller) now, this can be fairly low */
#define MAX_NUM_BCAST_MSGS 12

/**
 * @brief Compute the byte size required for a broadcast message buffer array.
 * @param max_num_bcast_msgs Maximum number of concurrent broadcast slots needed.
 */
#define COM_MANAGER_BUF_SIZE_BCAST_MSG(max_num_bcast_msgs)      ((max_num_bcast_msgs)*sizeof(broadcast_msg_t))

/** Tracks a single scheduled broadcast message, including its packet payload and timing. */
typedef struct
{
    packet_t                pkt;        //!< Pre-built packet to transmit on each broadcast tick
    int32_t                 counter;    //!< Countdown counter; decremented each step; fires when it reaches 0
    int32_t                 period;     //!< Broadcast period in step intervals. -1 = send once; 0 = disabled
    port_handle_t           port;       //!< Port handle on which this broadcast is sent
} broadcast_msg_t;

/** Fixed-size array of @ref broadcast_msg_t slots; one array is needed per com manager instance. */
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

/** Error codes returned by com manager error callbacks. */
enum eComManagerErrorType
{
    CM_ERROR_FORWARD_OVERRUN = -1,  //!< Forwarding buffer overran; some data was dropped
    CM_ERROR_RX_PARSE        = -2,  //!< A parse error occurred on a received packet
};

/**
 * @brief Callback: return the number of free bytes in the port's transmit buffer.
 * @param ctx  User context pointer passed through from the com manager.
 * @param port Port handle to query.
 * @return Number of free bytes available in the transmit buffer.
 */
typedef int(*pfnComManagerSendBufferAvailableBytes)(void* ctx, port_handle_t port);

/**
 * @brief Callback: invoked after a complete ISB data packet has been received and parsed.
 * @param ctx      User context pointer.
 * @param dataRead Pointer to the parsed ISB data (DID, size, offset, and payload pointer).
 * @param port     Port on which the packet arrived.
 * @return 0 to continue processing; non-zero to suppress further dispatch.
 */
typedef int(*pfnComManagerPostRead)(void* ctx, p_data_t* dataRead, port_handle_t port);

/**
 * @brief Callback: invoked after an ISB ACK or NACK packet is received.
 * @param ctx               User context pointer.
 * @param port              Port on which the ACK/NACK arrived.
 * @param ack               Pointer to the parsed ACK/NACK body.
 * @param packetIdentifier  Packet type flags of the acknowledged packet.
 * @return 0 to continue processing; non-zero to suppress further dispatch.
 */
typedef int(*pfnComManagerPostAck)(void* ctx, port_handle_t port, p_ack_t* ack, unsigned char packetIdentifier);

/**
 * @brief Callback: invoked to disable all broadcasts on a given port (or all ports).
 *
 * Mostly for internal use; can be left NULL.  Pass port == -1 to disable on all ports.
 *
 * @param ctx  User context pointer.
 * @param port Port handle to disable broadcasts on, or -1 for all ports.
 * @return 0 on success.
 */
typedef int(*pfnComManagerDisableBroadcasts)(void* ctx, port_handle_t port);

/**
 * @brief Callback: invoked immediately before a data packet is sent.
 *
 * Return 0 to suppress transmission of the packet; any other value allows it to proceed.
 *
 * @param ctx     User context pointer.
 * @param port    Port the packet will be sent on.
 * @param dataHdr Header of the packet about to be transmitted.
 * @return 0 to suppress the send; non-zero to allow it.
 */
typedef int(*pfnComManagerPreSend)(void* ctx, port_handle_t port, p_data_hdr_t *dataHdr);

/**
 * @brief Callback: invoked when a GET_DATA / broadcast-enable request is received.
 * @param ctx  User context pointer.
 * @param req  The data-get request specifying DID, size, offset, and period.
 * @param port Port that sent the request.
 * @return 0 if the request was handled; non-zero to pass it to the default handler.
 */
typedef int(*pfnComManagerRmcHandler)(void* ctx, p_data_get_t* req, port_handle_t port);

/**
 * @brief Callback: invoked when a packet parse error is detected.
 * @param ctx  User context pointer.
 * @param port Port on which the parse error occurred.
 * @return 1 if the error was handled; 0 to let the com manager apply default error handling.
 */
typedef int(*pfnComManagerParseErrorHandler)(void* ctx, port_handle_t port);
// typedef std::function<int(port_handle_t)> pfnComManagerParseErrorHandler;


/**
 * @brief Initialize the global com manager. Call once at program start.
 *
 * @param portSet                Pointer to the set of port handles the com manager will own.
 *                               May be NULL if no ports are available at initialization.
 * @param stepPeriodMilliseconds Nominal milliseconds between comManagerStep() calls; used for
 *                               broadcast period accounting.
 * @param pstRxFnc               Optional: called after each successfully parsed ISB data packet.
 * @param pstAckFnc              Optional: called after each ISB ACK/NACK is received.
 * @param rmcHandler             Optional: called when a broadcast-enable (GET_DATA) request arrives.
 * @param disableBcastFnc        Optional: called to disable broadcasts; can be NULL.
 * @param buffers                Pointer to the broadcast message buffer array (MAX_NUM_BCAST_MSGS entries).
 * @return 0 on success, -1 on failure.
 */
int comManagerInit(
        std::set<port_handle_t>* portSet,
        int stepPeriodMilliseconds,
        pfnComManagerPostRead pstRxFnc,
        pfnComManagerPostAck pstAckFnc,
        pfnComManagerRmcHandler rmcHandler,
        pfnComManagerDisableBroadcasts disableBcastFnc,
        std::array<broadcast_msg_t, MAX_NUM_BCAST_MSGS>* buffers);

/**
 * @brief Initialize the global com manager without specifying an initial port set.
 *
 * Equivalent to the portSet overload with portSet == NULL.
 *
 * @param stepPeriodMilliseconds Nominal milliseconds between comManagerStep() calls.
 * @param pstRxFnc               Optional: called after each successfully parsed ISB data packet.
 * @param pstAckFnc              Optional: called after each ISB ACK/NACK is received.
 * @param rmcHandler             Optional: called when a broadcast-enable request arrives.
 * @param disableBcastFnc        Optional: called to disable broadcasts; can be NULL.
 * @param buffers                Pointer to the broadcast message buffer array.
 * @return 0 on success, -1 on failure.
 */
int comManagerInit(
        int stepPeriodMilliseconds,
        pfnComManagerPostRead pstRxFnc,
        pfnComManagerPostAck pstAckFnc,
        pfnComManagerRmcHandler rmcHandler,
        pfnComManagerDisableBroadcasts disableBcastFnc,
        std::array<broadcast_msg_t, MAX_NUM_BCAST_MSGS>* buffers);

/**
 * @brief Register a protocol-specific message handler on the global com manager.
 * @param ptype     Protocol type to handle (see @ref protocol_type_t).
 * @param cbHandler Handler function to register; replaces any existing handler for @p ptype.
 * @param port      Optional: port to scope the handler to; NULL registers globally.
 * @return Previously registered handler for @p ptype, or NULL if none was set.
 */
pfnIsCommGenMsgHandler comManagerRegisterProtocolHandler(int ptype, pfnIsCommGenMsgHandler cbHandler, port_handle_t port = NULL);

/**
 * @brief Allocate and register a new port of the specified type.
 * @param ptype Protocol type mask for the new port (see @ref eProtocolMask).
 * @return Handle to the newly allocated port, or NULL on failure.
 */
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
std::set<port_handle_t>& comManagerGetPorts();


/**
 * @brief Remove a port from the com manager's managed set.
 *
 * The port is not closed or flushed; only its association with the com manager is removed.
 *
 * @param port Port handle to remove.
 * @return true if the port was found and removed, false if it was not registered.
 */
bool comManagerRemovePort(port_handle_t port);

/**
 * @brief Remove all ports from the com manager's managed set.
 * @return true on success.
 */
bool comManagerReleaseAllPorts();

/**
* Performs one round of sending and receiving message. Call as frequently as needed to send and receive data.
* @param timeMs current time in milliseconds used for paser timeout.  Used to invalidate packet parsing if PKT_PARSER_TIMEOUT_MS time has lapsed since any data has been received.
*/
void comManagerStepTimeout(uint32_t timeMs);

/** @brief Process one communication cycle: read all ports, dispatch received packets, send pending broadcasts. */
void comManagerStep();

/**
 * @brief Process one communication cycle for a single port only.
 * @param port Port handle to read and dispatch; broadcast scheduling is not performed.
 */
void comManagerStep(port_handle_t port);

/** @brief Transmit all pending broadcast messages whose period counters have expired. */
void stepSendMessages(void);

/**
 * @brief Request a device to broadcast a specific data set at a periodic interval.
 * @param port   Port handle to send the request to.
 * @param did    Data ID to request (see DID_* constants in data_sets.h).
 * @param size   Number of bytes to request from @p offset; 0 = full structure.
 * @param offset Byte offset into the data structure; 0 = start.
 * @param period Broadcast period in step-interval multiples; 0 = one-shot request.
 */
void comManagerGetData(port_handle_t port, uint16_t did, uint16_t size, uint16_t offset, uint16_t period);

/**
 * @brief Same as comManagerGetData(), with an additional ISB packet flags parameter (see
 *        eISBPacketFlags) OR'd into the request -- e.g. ISB_FLAGS_GET_DATA_PRESERVE_STREAM,
 *        which asks a device that supports it not to stop an existing broadcast for this DID on
 *        this port when period is 0 (SN-8471). comManagerGetData() is equivalent to calling this
 *        with flags=0.
 */
void comManagerGetDataFlags(port_handle_t port, uint16_t did, uint16_t size, uint16_t offset, uint16_t period, uint8_t flags);

/**
 * @brief Request a device to broadcast a preset collection of messages via RMC bits.
 *
 * @param port       Port handle to send the request to.
 * @param rmcBits    RMC bitmask specifying which data IDs to stream.
 *                   Common presets: RMC_PRESET_IMX_PPD (post-processing), RMC_PRESET_INS.
 * @param rmcOptions RMC option flags; use 0 for default behavior (current port only).
 *
 * @code
 * comManagerGetDataRmc(port, RMC_PRESET_IMX_PPD, 0);   // enable post-processing stream
 * comManagerGetDataRmc(port, RMC_PRESET_INS, 0);       // INS + GPS at full rate
 * @endcode
 */
void comManagerGetDataRmc(port_handle_t port, uint64_t rmcBits, uint32_t rmcOptions);

/**
 * @brief Stop broadcasting a specific data ID on the given port.
 *
 * @param port Port handle to disable the broadcast on.
 * @param did  Data ID whose broadcast should be stopped.
 * @return 0 on success, non-zero on failure.
 *
 * @code
 * comManagerDisableData(port, DID_INS_1);
 * @endcode
 */
int comManagerDisableData(port_handle_t port, uint16_t did);

/**
 * @brief Send an ISB packet to the specified port.
 *
 * @param port   Port handle to send to.
 * @param pFlags ISB packet flags including the packet type (see @ref eISBPacketFlags).
 * @param data   Pointer to the payload data; may be NULL for command packets with no body.
 * @param did    Data ID for the payload.
 * @param size   Size in bytes of the payload.
 * @param offset Byte offset into the data structure; 0 for the start of the structure.
 * @return 0 on success, non-zero on failure.
 */
int comManagerSend(port_handle_t port, uint8_t pFlags, const void *data, uint16_t did, uint16_t size, uint16_t offset = 0);

/**
 * @brief Send an ISB SET_DATA packet; wraps comManagerSend() with PKT_TYPE_SET_DATA flags.
 *
 * The device responds with an ACK.  The payload size must be a multiple of 4 bytes.
 *
 * @param port   Port handle to send to.
 * @param data   Pointer to the data structure to send.
 * @param did    Data ID of the structure.
 * @param size   Number of bytes to send from @p data.
 * @param offset Byte offset into the data structure; 0 for the start.
 * @return 0 on success, non-zero on failure.
 *
 * @code
 * comManagerSendData(port, &g_devInfo, DID_DEV_INFO, sizeof(dev_info_t), 0);
 * @endcode
 */
int comManagerSendData(port_handle_t port, const void* data, uint16_t did, uint16_t size, uint16_t offset = 0);

/**
 * @brief Send an ISB DATA packet (no ACK expected); same as comManagerSendData() but uses PKT_TYPE_DATA.
 *
 * Unlike comManagerSendData(), the device does not send an ACK in response.
 *
 * @param port   Port handle to send to.
 * @param data   Pointer to the data structure to send.
 * @param did    Data ID of the structure.
 * @param size   Number of bytes to send from @p data.
 * @param offset Byte offset into the data structure; 0 for the start.
 * @return 0 on success, non-zero on failure.
 */
extern "C" int comManagerSendDataNoAck(port_handle_t port, const void *data, uint16_t did, uint16_t size, uint16_t offset = 0);

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
 * @brief Get the ISComm instance associated with the given port.
 * @param port Port handle to look up.
 * @return Pointer to the is_comm_instance_t owned by the com manager for @p port; NULL if not found.
 */
is_comm_instance_t* comManagerGetIsComm(port_handle_t port);

/**
 * @brief Look up the transmit/receive buffer pointers registered for a data ID.
 * @param did Data ID to look up.
 * @return Pointer to the registered bufTxRxPtr_t, or NULL if @p did is not registered.
 */
bufTxRxPtr_t* comManagerGetRegisteredDataInfo(uint16_t did);

/**
 * @brief Process a GET_DATA request and schedule the requested broadcast.
 * @param port Port handle that sent the request.
 * @param req  Parsed GET_DATA request specifying the DID, offset, size, and period.
 * @return 0 on success, non-zero on failure.
 */
int comManagerGetDataRequest(port_handle_t port, p_data_get_t* req);

/**
 * @brief Register a callback invoked whenever a packet parse error is detected.
 * @param errorCb Error handler callback to register; replaces any previously set handler.
 */
void comManagerSetErrorHandler(pfnComManagerParseErrorHandler errorCb);

/**
 * @brief Register pre-send and post-receive callbacks for a specific data ID.
 *
 * @param did       Data ID to register handlers for.
 * @param txFnc     Optional: called immediately before a packet with @p did is sent; NULL to skip.
 * @param pstRxFnc  Optional: called after a packet with @p did is received; NULL to skip.
 * @param txDataPtr Pointer to the structure to transmit (owned by the caller).
 * @param rxDataPtr Pointer to the structure to copy received data into (owned by the caller).
 * @param size      Size in bytes of the data structure at @p txDataPtr / @p rxDataPtr.
 * @param pktFlags  ISB packet flags for outbound packets; use 0 for the default.
 *
 * @code
 * comManagerRegister(DID_INS_1, prepMsgINS, writeMsgINS, &g_insData, &g_insData, sizeof(ins_1_t), 0);
 * @endcode
 */
void comManagerRegister(uint16_t did, pfnComManagerPreSend txFnc, pfnComManagerPostRead pstRxFnc, const void* txDataPtr, void* rxDataPtr, uint16_t size, uint8_t pktFlags);

/**
 * @brief Process a single received packet; dispatches to registered DID and protocol handlers.
 *
 * This function matches the @ref pfnIsCommHandler signature and is registered as the default
 * packet handler during comManagerInit().  It is exposed for unit-testing purposes.
 *
 * @param ctx   User context pointer (the ISComManager instance).
 * @param ptype Protocol type of the received packet.
 * @param pkt   Pointer to the fully parsed packet.
 * @param port  Port on which the packet was received.
 * @return 0 if the packet was handled; non-zero otherwise.
 */
int comManagerProcessRxPacket(void* ctx, protocol_type_t ptype, packet_t *pkt, port_handle_t port);

/**
 * @brief Core com manager class — manages ports, drives the communication loop, and dispatches packets.
 *
 * All C-style comManager* free functions delegate to the global @ref s_cm instance of this class.
 * Use the class API directly when working with multiple independent com manager instances.
 */
class ISComManager {
public:
    /** @brief Run one full communication cycle: receive from all ports, dispatch, send broadcasts. */
    void step();
    /** @brief Transmit pending outbound broadcasts; does not read from ports. */
    void stepTx();
    /**
     * @brief Receive and parse packets from a single port; does not send broadcasts.
     * @param port Port handle to read and dispatch.
     */
    void stepRx(port_handle_t port);
    /** @brief Receive and parse packets from all registered ports; does not send broadcasts. */
    void stepRx();

    /**
     * @brief Initialize this com manager instance. Call once before any other methods.
     *
     * @param portSet                Pointer to the set of port handles to manage; may be NULL.
     * @param stepPeriodMilliseconds Nominal milliseconds between step() calls (for broadcast timing).
     * @param pstRxFnc               Optional: called after each successfully parsed ISB data packet.
     * @param pstAckFnc              Optional: called after each ISB ACK/NACK is received.
     * @param rmcHandler             Optional: called when a broadcast-enable request arrives.
     * @param disableBcastFnc        Optional: called to disable broadcasts; can be NULL.
     * @param buffers                Broadcast message buffer array (MAX_NUM_BCAST_MSGS entries).
     * @return 0 on success, -1 on failure.
     */
    int init(
            std::set<port_handle_t>* portSet,
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
     * @brief Register a port with the com manager so it is included in broadcast and receive cycles.
     * @param port      Port handle to register.
     * @param callbacks Optional custom callback set for this port; NULL uses the manager defaults.
     * @return true if the port was successfully registered, false if it was already registered.
     */
    bool registerPort(port_handle_t port, is_comm_callbacks_t* callbacks = NULL);


    /**
     * @return a vector of all registered ports
     */
    std::set<port_handle_t>& getPorts();


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
     * @brief Request broadcast of a specific data set at a periodic interval.
     * @param port   Port handle to send the request to.
     * @param did    Data ID to request.
     * @param size   Bytes to request from @p offset; 0 = full structure.
     * @param offset Byte offset into the data structure; 0 = start.
     * @param period Broadcast period in step multiples; 0 = one-shot request.
     */
    void getData(port_handle_t port, uint16_t did, uint16_t size, uint16_t offset, uint16_t period, uint8_t flags = 0);

    /**
     * @brief Request broadcast of a preset collection of messages via RMC bits.
     * @param port       Port handle to send the request to.
     * @param rmcBits    RMC bitmask specifying which data IDs to stream.
     * @param rmcOptions RMC option flags; 0 for default (current port only).
     */
    void getDataRmc(port_handle_t port, uint64_t rmcBits, uint32_t rmcOptions);

    /**
     * @brief Stop broadcasting a specific data ID on the given port.
     * @param port Port handle to disable the broadcast on.
     * @param did  Data ID whose broadcast should be stopped.
     * @return 0 on success, non-zero on failure.
     */
    int disableData(port_handle_t port, uint16_t did);

    /**
     * @brief Send an ISB packet to the specified port.
     * @param port   Port handle to send to.
     * @param pFlags ISB packet flags including the packet type (see @ref eISBPacketFlags).
     * @param data   Pointer to the payload; may be NULL for command packets.
     * @param did    Data ID for the payload.
     * @param size   Payload size in bytes.
     * @param offset Byte offset into the data structure; 0 = start.
     * @return 0 on success, non-zero on failure.
     */
    int send(port_handle_t port, uint8_t pFlags, const void *data, uint16_t did, uint16_t size, uint16_t offset = 0);

    /**
     * @brief Send an ISB SET_DATA packet. Device responds with an ACK.
     * @param port   Port handle to send to.
     * @param data   Pointer to the data structure.
     * @param did    Data ID.
     * @param size   Number of bytes to send.
     * @param offset Byte offset into the data structure.
     * @return 0 on success, non-zero on failure.
     */
    int sendData(port_handle_t port, const void* data, uint16_t did, uint16_t size, uint16_t offset = 0);

    /**
     * @brief Send an ISB DATA packet (no ACK sent by device).
     * @param port   Port handle to send to.
     * @param data   Pointer to the data structure.
     * @param did    Data ID.
     * @param size   Number of bytes to send.
     * @param offset Byte offset into the data structure.
     * @return 0 on success, non-zero on failure.
     */
    int sendDataNoAck(port_handle_t port, const void *data, uint16_t did, uint16_t size, uint16_t offset = 0);

    /**
     * @brief Write raw bytes directly to a port without ISB framing.
     * @param port     Port handle to send to.
     * @param dataPtr  Pointer to the raw bytes to send.
     * @param dataSize Number of bytes to send.
     * @return 0 on success, non-zero on failure.
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
     * @brief Register pre-send and post-receive callbacks for a specific data ID.
     * @param did       Data ID to register handlers for.
     * @param txFnc     Optional: called immediately before a packet with @p did is sent; NULL to skip.
     * @param pstRxFnc  Optional: called after a packet with @p did is received; NULL to skip.
     * @param txDataPtr Pointer to the structure to transmit (owned by the caller).
     * @param rxDataPtr Pointer to the structure to copy received data into (owned by the caller).
     * @param size      Size in bytes of the data structure at @p txDataPtr / @p rxDataPtr.
     * @param pktFlags  ISB packet flags for outbound packets; use 0 for the default.
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

    // Array of ports - that will be managed, but not owned, by the ISComManager instance
    std::set<port_handle_t>* ports = NULLPTR;  // this is not a vector of ports, its a pointer to an EXTERNAL set of ports.  This MUST be initialized!!!

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

/** Global ISComManager singleton used by all C-style comManager* free functions. */
extern ISComManager s_cm;

/** Handle type for a com manager instance (pointer to ISComManager). */
typedef ISComManager* CMHANDLE;

/**
 * @brief Return a handle to the global ISComManager singleton (@ref s_cm).
 *
 * Only needed when comparing or passing com manager instances across module boundaries.
 *
 * @return Pointer to the global ISComManager instance.
 */
CMHANDLE comManagerGetGlobal(void);

#endif // COM_MANAGER_H
