/**
 * @file CorrectionService.h
 * @brief Class used to distribute an incoming RTCM stream of corrections to "Rover" devices
 * The term service is a little ambiguous by choice. A CorrectionService can be used on both
 * the Rover-side, receiving RTCM3 corrections from a single RTCM3 service, and forwarding to
 * many rovers. Additionally, it can be used on the Base-side, by receiving RTCM3 output from
 * a single RTK base, and forwarding it to multiple ports (most commonly tcpPorts, but not
 * always), where another CorrectionService is receiving those corrections and forwarding them
 * to one or more physical device (or even another CorrectionService).
 *
 * In this way, the CorrectionService is purely a service which parses RTCM3 from a single port
 * and forwards it to multiple other ports.
 *
 * @author FiriusFoxx on 9/4/25.
 * @copyright Copyright (c) 2025 Inertial Sense, Inc. Licensed under the MIT license
 */

#ifndef IS_SDK_CORRECTION_SERVICE_H
#define IS_SDK_CORRECTION_SERVICE_H

#include <vector>

#include "core/types.h"
#include "core/base_port.h"
#include "ISConstants.h"

#include "PortFactory.h"
#include "ISDevice.h"
#include "message_stats.h"

/**
 * The CorrectionService is a "Rover"-side manager responsible for establishing a connection to
 * a correction service, such as an NTRIP caster, or raw RTCM3 provider, reading correction data
 * from the source port, and forwarding it to one or more associated ISDevices.
 */
class CorrectionService {
public:
    /** Callback signature for addRTCM3Msg1029Listeners(): receives the RTCM3 Msg 1029 text payload. */
    typedef std::function<void(std::string)> tRTCM3Msg1029ListenerCallback;
    /** Callback signature for addRTCM3PacketListeners(): receives (RTCM3 message type, packet data, packet size). */
    typedef std::function<void(uint16_t, const void*, uint32_t)> tRTCM3PacketListenerCallback;

    /** @brief Default constructor; no source port is bound until setSourcePort() is called. */
    CorrectionService() { init(nullptr); }

    /**
     * The base constructor, binds the specified port as the source for corrections data that
     * will be forwarded to all associated ISDevices
     * @param srcPort The port on which incoming RCTM3 data to recieved from
     */
    explicit CorrectionService(port_handle_t srcPort) { init(srcPort); };

    /**
     * A convenience constructor which creates/binds the necessary port as described by the portName, by
     * attempting to allocate the named port from each of the provided factories.  This is particularly
     * useful when the port is not directly discoverable, such as TCP, where the portName is a URL to
     * a remote TCP server.
     * @param portName The name of the port to be allocated
     * @param factories An optional list of factories to ask to create the given port name
     */
    explicit CorrectionService(const std::string& portName, const std::vector<PortFactory*>& factories = nullFactories);

    virtual ~CorrectionService() {
        // Don't actually close down any ports for the base CorrectionService - extended services can choose to, if they need but don't do it here
        // for (auto a : ports)  portClose(a);

        if (source && localSrcPort) {
            portClose(source);
            if (srcPortFactory)
                srcPortFactory->releasePort(source);
            else
                PortManager::getInstance().releasePort(source);
        }
    }

    /**
     * Sets the port to read corrections from
     * @param srcPort the port to bind as the correction source, replacing any previously-bound source port
     */
    void setSourcePort(port_handle_t srcPort);

    /**
     * @returns the port which provides the source corrections to all other ports
     */
    port_handle_t getSourcePort() { return source; }

    /**
     * Adds a port to the recieve corrections from this service
     * @param port A reference to the port to send corrections to
     */
    void addPort(port_handle_t port);

    /**
     * Adds multiple ports to recieve corrections from this service
     * @param ports_ A vector of references to devices to send corrections to
     */
    void addPorts(const std::vector<port_handle_t>& ports_) { for (auto& p : ports_) { addPort(p); } }

    /**
     * Adds a device to the recieve corrections from this service
     * @param device A reference to the device to send corrections to
     */
    void addDevice(device_handle_t device);

    /**
     * Adds multiple devices to recieve corrections from this service
     * @param devices A vector of references to devices to send corrections to
     */
    void addDevices(const std::vector<device_handle_t>& devices) { for (auto& d : devices) { addDevice(d); } }

    /**
     * Cease sending corrections from this service a port
     * @param port A reference to the port to cease sending corrections to
     */
    void removePort(port_handle_t port);

    /**
     * Cease sending corrections from this service to multiple ports
     * @param ports_ A vector of references to devices to cease corrections to
     */
    void removePorts(const std::vector<port_handle_t>& ports_) { for (auto& p : ports_) { removePort(p); } }

    /**
     * Cease sending corrections from this service a device
     * @param device A reference to the device to cease sending corrections to
     */
    void removeDevice(device_handle_t device);

    /**
     * Cease sending corrections from this service to multiple devices
     * @param devices A vector of references to devices to cease corrections to
     */
    void removeDevices(const std::vector<device_handle_t>& devices) { for (auto& d : devices) { removeDevice(d); } }

    /**
     * Check if this CorrectionService is sending correction data to a given port
     * @param port A reference of a port to check for
     * @return true if port is among this service's downstream ports, false otherwise.
     */
    bool hasPort(port_handle_t port);

    /**
     * Check if this CorrectionService is sending correction data to a given device
     * @param device A reference of a device to check for
     * @return true if device's port is among this service's downstream ports, false otherwise.
     */
    bool hasDevice(device_handle_t device);

    /**
     * Gets the stats for the source port for this CorrectionService
     * @return the source port's accumulated read/write statistics.
     */
    [[nodiscard]] port_stats_t* getSourceStats() const {return BASE_PORT(source)->stats;}

    /**
     * Adds a callback to be called when a Msg 1029 is received from RTCM3
     * @param callback Callback to call when receiving a Msg 1029 over RTCM3
     * @return id That can be used to remove this callback later
     */
    uint32_t addRTCM3Msg1029Listeners(const tRTCM3Msg1029ListenerCallback &callback);

    /**
     * Remove a callback from being called on Msg 1029
     * @param id The id returned from addRTCM3Msg1029Listeners
     */
    void removeRTCM3Msg1029Listeners(uint32_t id);

    /**
     * Adds a callback to be called when any RTCM3 packet is received
     * @param callback Callback to call when receiving an RTCM3 Packet
     * @return id That can be used to remove this callback later
     */
    uint32_t addRTCM3PacketListeners(const std::function<void(uint16_t, const void*, uint32_t)>& callback);

    /**
     * Remove a callback from being called on an RTCM3 Packet
     * @param id The id returned from addRTCM3PacketListeners
     */
    void removeRTCM3PacketListeners(uint32_t id);

    /**
     * Checks the source port for data and forwards it to all devices.
     * Also (re)opens the source port if it is not currently open. Virtual so that protocol-specific
     * subclasses (e.g. NtripCorrectionService) can extend the (re)connection handling -- NTRIP, for
     * example, must re-negotiate its mount-point request after every (re)connect, not just reopen the socket.
     * @return 0 if no packets processed, positive number representing number of packets processed, negative number representing errno
     */
    virtual int step();

    /**
     * @return the age, in milliseconds, of the last received RTCM3 message.
     */
    uint32_t getLastRtcm3PacketAge() { return current_timeMs() - rtcm3PacketLastMs; }

    /**
     * @return the number of opened "downstream" ports to which RTCM3 messages will be forwarded
     */
    int getActiveConnections() { return (int)ports.size(); }

    /**
     * If a pointer to a MessageStats object is provided that object will be updated as correction messages are parsed.
     * @param stats a pointer to a MessageStats object, or nullptr if no stats should be collected
     */
    void setMessageStats(MessageStats::mul_stats_t* stats) { msgStats = stats; }

    /**
     * @return the associated MessageStats instance, if any.
     */
    MessageStats::mul_stats_t* getMessageStats() { return msgStats; }


protected:
    port_handle_t source {};                  //!< The bound source port from which correction data is read
    std::vector<port_handle_t> ports;          //!< Downstream ports to which correction data is forwarded
    uint32_t lastConnAttemptTs = 0;            //!< timestamp (ms) after which the next source (re)connection attempt is allowed (reconnect backoff)

private:
    inline static const std::vector<PortFactory*>& nullFactories = {};    //!< Empty factory list, used as the default argument for the portName-based constructor
    std::vector<tRTCM3Msg1029ListenerCallback> rtcm3Msg1029Listeners;      //!< Registered callbacks notified on every RTCM3 Msg 1029
    std::vector<tRTCM3PacketListenerCallback> rtcm3PacketListeners;       //!< Registered callbacks notified on every RTCM3 packet
    is_comm_instance_t packetParser = {};                                 //!< SDK comm-protocol parser instance bound to the source port
    uint32_t rtcm3PacketsProcessed = 0;                                 //!< total number of RTCM3 packets that have been processed
    uint32_t rtcm3PacketLastMs = 0;                                     //!< timestamp in ms, since the last RTCM3 packet was seen
    bool localSrcPort = false;                                          //!< true if the source port was locally instantiated rather than passed in the constructor
    PortFactory* srcPortFactory = nullptr;                               //!< the factory used to create the source port (if localSrcPort is true)

    MessageStats::mul_stats_t* msgStats = nullptr;                      //!< if not-null, call into the msgStats when parsing the source port

    pfnIsCommGenMsgHandler previousRtcm3Handler = nullptr;              //!< RTCM3 handler previously registered on the source port, restored when the source port changes/closes
    pfnIsCommGenMsgHandler previousErrorHandler = nullptr;              //!< Error/raw-data handler previously registered on the source port, restored when the source port changes/closes

    /**
     * Transforms one format (like NTRIP) to RTCM3 to be processed by the device.
     *
     * This is the primary extension point for subclasses implementing another corrections
     * protocol in a host-side application. It's only invoked from onRawDataHandler(), for data the
     * base comm parser could not already identify as RTCM3 (data recognized as RTCM3 bypasses this
     * entirely via onRtcm3Handler() and is forwarded as-is). The base implementation is a no-op
     * stub that reads/writes nothing and returns 0 -- i.e. unrecognized raw data is dropped unless
     * a subclass overrides this to translate its protocol's framing into RTCM3.
     * @param inputBuffer Data to be transformed
     * @param inputLength Length of input buffer
     * @param finalBuffer To be transmitted to the device
     * @param finalBufferSize The size of the final buffer
     * @return Number of bytes written into finalBuffer
     */
    virtual uint32_t packetTransformer(const uint8_t *inputBuffer, uint32_t inputLength, uint8_t *finalBuffer, uint32_t finalBufferSize);

    /**
     * Final packet filter ensures only RTCM3 packets are sent to the device
     * Also processes callbacks for RTCM3 1029 messages
     * @param inputBuffer Data to be filter on
     * @param inputLength Length of input buffer
     * @param finalBuffer To be transmitted to the device
     * @param finalBufferSize The size of the final buffer
     * @param bytesProcessed A pointer to a uint32_t to store the number of bytes to be sent to the device
     * @return Number of packets processed
     */
    int finalPacketFilter(const uint8_t *inputBuffer, uint32_t inputLength, uint8_t *finalBuffer,
                          uint32_t finalBufferSize, uint32_t *bytesProcessed);

    /**
     * Send data to all devices
     * @param inputBuffer Input buffer of bytes to send
     * @param inputLength Number of bytes to send
     */
    void sendData(const uint8_t *inputBuffer, uint32_t inputLength);

    /**
     * Private initializer function
     * @param srcPort Port to use for Corrections
     */
    void init(port_handle_t srcPort);

    /**
     * Handles incoming RTCM3 packet
     * @param msg Contents of RTCM3 packet
     * @param msgSize Size of RTCM3 packet
     * @param port port which recieved RTCM3 packet
     * @return 0 on success, -1 on error, always succeeds
     */
    int onRtcm3Handler(const unsigned char* msg, int msgSize, port_handle_t port);

    /**
     * Handles incoming Unknown packet format
     * @param msg Contents of Raw data packet
     * @param msgSize Size of Raw data packet
     * @param port port which recieved Raw data packet
     * @return 0 on success, -1 on error, always succeeds
     */
    int onRawDataHandler(const unsigned char* msg, int msgSize, port_handle_t port);
};


#endif //IS_SDK_CORRECTION_SERVICE_H
