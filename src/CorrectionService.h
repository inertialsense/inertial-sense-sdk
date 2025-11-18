/**
 * @file CorrectionService.h
 * @brief Class used to distribute an incoming RTCM stream of corrections to "Rover" devices
 * The term service is a little ambiguous by choice. A CorrectionService can be used on both
 * the Rover-side, receiving RTCM3 corrections from a single RTCM3 service, and forwarding to
 * many rovers. Additionally, it can be used on the Base-side, by receiving RTCM3 outfrom from
 * a single RTK base, and forwarding it to multiple ports (most commonly tcpPorts, but not
 * always), where another CorrectionService is receiving those corrections and forwarding them
 * to one or more physical device (or even another CorrectionService).
 *
 * In this way, the CorrectionService is purely a service which parses RTCM3 from a single port
 * and forwards it to multiple other ports. Additionally, it handles
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

/**
 * The CorrectionService is a "Rover"-side manager responsible for establishing a connection to
 * a correction service, such as an NTRIP caster, or raw RTCM3 provider, reading correction data
 * from the source port, and forwarding it to one or more associated ISDevices.
 */
class CorrectionService {
public:
    virtual ~CorrectionService() = default;
    typedef std::function<void(std::string)> tRTCM3Msg1029ListenerCallback;
    typedef std::function<void(uint16_t, const void*, uint32_t)> tRTCM3PacketListenerCallback;

    CorrectionService() { init(nullptr); }

    /**
     * The base constructor, binds the specified port as the source for corrections data that
     * will be forwarded to all associated ISDevices
     * @param srcPort The port on which incoming RCTM3 data to recieved from
     */
    explicit CorrectionService(port_handle_t srcPort);

    /**
     * A convenience constructor which creates/binds the necessary port as described by the portName, by
     * attempting to allocate the named port from each of the provided factories.  This is particularly
     * useful when the port is not directly discoverable, such as TCP, where the portName is a URL to
     * a remote TCP server.
     * @param portName The name of the port to be allocated
     * @param factories An optional list of factories to ask to create the given port name
     */
    explicit CorrectionService(const std::string& portName, const std::vector<PortFactory*>& factories = nullFactories);

    /**
     * Sets the port to read corrections from
     * @param port
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
     */
    bool hasPort(port_handle_t port);

    /**
     * Check if this CorrectionService is sending correction data to a given device
     * @param device A reference of a device to check for
     */
    bool hasDevice(device_handle_t device);

    /**
     * Gets the stats for the source port for this CorrectionService
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
     * Checks the source port for data and forwards it to all devices
     * @return 0 if no packets processed, positive number representing number of packets processed, negative number representing errno
     */
    [[nodiscard]] int step() const;

protected:
    port_handle_t source {};
    std::vector<port_handle_t> ports;

private:
    inline static const std::vector<PortFactory*>& nullFactories = {};
    std::vector<tRTCM3Msg1029ListenerCallback> rtcm3Msg1029Listeners;
    std::vector<tRTCM3PacketListenerCallback> rtcm3PacketListeners;
    is_comm_instance_t packetParser = {};
    uint32_t rtcm3PacketsProcessed = 0;

    pfnIsCommGenMsgHandler previousRtcm3Handler = nullptr;
    pfnIsCommGenMsgHandler previousErrorHandler = nullptr;

    /**
     * Transforms one format (like NTRIP) to RTCM3 to be processed by the device
     * You should overload this if your implementing another corrections protocol in a host side application
     * @param inputBuffer Data to be transformed
     * @param inputLength Length of input buffer
     * @param finalBuffer To be transmitted to the device
     * @param finalBufferSize The size of the final buffer
     * @return Number of bytes processed
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
     * @param port Port to use for Corrections
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
