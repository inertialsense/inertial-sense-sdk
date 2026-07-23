/**
 * @file Rtcm3CorrectionServer.h 
 * @brief Provides a TCP server for streaming RTCM3 GNSS correction data.
 *
 * @author Kyle Mallory on 11/10/25.
 * @copyright Copyright (c) 2025 Inertial Sense, Inc. All rights reserved.
 */

#ifndef IS_SDK_RTCM3_CORRECTION_SERVER_H
#define IS_SDK_RTCM3_CORRECTION_SERVER_H

#include "DeviceManager.h"
#include "CorrectionService.h"
#include "TcpServerPortFactory.h"
#include "util/util.h"

/**
 * @brief CorrectionService specialization that acts as a TCP server (base station side): it reads
 * RTCM3 corrections from a source device, listens for incoming TCP client connections, and
 * forwards the correction stream to every connected client as they connect.
 *
 * Combines CorrectionService (source-to-many-ports forwarding) with TcpServerPortFactory
 * (protected base, providing the listening socket / accept loop) so newly-accepted client sockets
 * can be registered directly as CorrectionService downstream ports.
 */
class Rtcm3CorrectionServer : public CorrectionService, protected TcpServerPortFactory {
public:
    /**
     * @brief Construct and start listening on the address/port parsed from a URI.
     * @param portUri listen URI, e.g. "tcp://[IP]:port"; any omitted component falls back to the
     *   defaults (127.0.0.1:7777).
     * @param max_connections maximum number of simultaneous client connections to accept.
     */
    explicit Rtcm3CorrectionServer(const std::string& portUri, int max_connections = 10) : TcpServerPortFactory() {
        // Parse the listen URI (e.g. "tcp://[IP]:port"); any omitted component falls back to the defaults.
        const utils::UriParts uri = utils::parseUri(portUri, "tcp://127.0.0.1:7777");
        configure(uri.port, uri.host, max_connections);     // configure() already starts the listener; do not call startListening() again
    }

    /**
     * @brief Construct and start listening on an explicit address/port.
     * @param port TCP port to listen on.
     * @param listenAddr local IP address to bind the listening socket to.
     * @param max_connections maximum number of simultaneous client connections to accept.
     */
    explicit Rtcm3CorrectionServer(int port = 7777, std::string listenAddr = "127.0.0.1", int max_connections = 10) : TcpServerPortFactory(port, listenAddr, max_connections) {
        startListening();
    }

    /**
     * @brief Construct, start listening on the given URI, and bind srcDevice as the correction source.
     * @param srcDevice device whose port will be used as the correction source (see setSourceDevice()).
     * @param portUri listen URI, e.g. "tcp://[IP]:port"; any omitted component falls back to the defaults.
     * @param max_connections maximum number of simultaneous client connections to accept.
     */
    explicit Rtcm3CorrectionServer(const device_handle_t srcDevice, const std::string& portUri, int max_connections = 10) : Rtcm3CorrectionServer(portUri, max_connections) {
        setSourceDevice(srcDevice);
    }

    /**
     * @brief Construct, start listening on an explicit address/port, and bind srcDevice as the correction source.
     * @param srcDevice device whose port will be used as the correction source (see setSourceDevice()).
     * @param port TCP port to listen on.
     * @param listenAddr local IP address to bind the listening socket to.
     * @param max_connections maximum number of simultaneous client connections to accept.
     */
    explicit Rtcm3CorrectionServer(const device_handle_t srcDevice, int port = 7777, std::string listenAddr = "127.0.0.1", int max_connections = 10) : Rtcm3CorrectionServer(port, listenAddr, max_connections) {
        setSourceDevice(srcDevice);
    }

    /**
     * @brief Construct, start listening on an explicit address/port, and bind the device identified
     * by srcDeviceId as the correction source (see setSourceDevice(uint64_t)).
     * @param srcDeviceId unique device Id to look up via DeviceManager once devices are discovered.
     * @param port TCP port to listen on.
     * @param listenAddr local IP address to bind the listening socket to.
     * @param max_connections maximum number of simultaneous client connections to accept.
     */
    explicit Rtcm3CorrectionServer(const uint64_t srcDeviceId, int port = 7777, std::string listenAddr = "127.0.0.1", int max_connections = 10) : Rtcm3CorrectionServer(port, listenAddr, max_connections) {
        setSourceDevice(srcDeviceId);
    }

    ~Rtcm3CorrectionServer() override = default;

    /**
     * @brief Stop listening, reconfigure the listen address/port/backlog, and resume listening.
     * @param port TCP port to listen on.
     * @param listenAddr local IP address to bind the listening socket to.
     * @param max_connections maximum number of simultaneous client connections to accept.
     */
    void configure(int port = 7777, std::string listenAddr = "127.0.0.1", int max_connections = 10) {
        stopListening();
        TcpServerPortFactory::configure(port, listenAddr, max_connections);
        startListening();
    }

    /** @brief Stop accepting new connections, close all client ports, and terminate any still-open client connections. */
    void shutdown() {
        stopListening();        // don't accept new connections
        for (auto p : ports) portClose(p);  // disconnect all clients when we shutdown
        shutdownAllClients();   // terminate existing connections
    }

    /**
     * @brief Extends CorrectionService::step(): first accepts any pending client connections
     * (registering each as a new downstream port via CorrectionService::addPort()), then delegates
     * to CorrectionService::step() to read the source and forward corrections to all downstream
     * ports, including the newly-accepted ones.
     * @return same as CorrectionService::step() -- 0 if no packets processed, positive number of
     *   packets processed, negative errno on error.
     */
    int step() {
        processPendingConnections([&](const socket_entry_t& e) {
            // The base TCP Port Factory doesn't provide a discovery service, but we must still "locate" any ports we determine are valid
            if (validatePort(e.portName, PORT_TYPE__TCP | PORT_TYPE__COMM)) {
                port_handle_t port = TcpServerPortFactory::bindPort(e.portName, PORT_TYPE__TCP | PORT_TYPE__COMM);
                addPort(port);
            }
        });

        return CorrectionService::step();
    }

    /**
     * @brief Sets the correction source to the given device's port, or clears the source if device is null.
     * @param device the device to read corrections from; pass nullptr to clear the source.
     */
    void setSourceDevice(const device_handle_t device) {
        sourceDevice = device;
        if (sourceDevice) {
            srcDeviceId = sourceDevice->getUniqueId();
            setSourcePort(sourceDevice->port);
        } else {
            srcDeviceId = 0;
            setSourcePort(nullptr);
        }
    }

    /**
     * @brief Sets the correction source by device Id, looking it up via DeviceManager. If the
     * device isn't currently known to DeviceManager, the source is cleared (deviceId is still
     * remembered so a later call can be retried once the device is discovered).
     * @param deviceId unique device Id (see ISDevice::getUniqueId()) to look up.
     */
    void setSourceDevice(uint64_t deviceId) {
        srcDeviceId = deviceId;
        sourceDevice = DeviceManager::getInstance().getDevice(srcDeviceId);
        if (sourceDevice) {
            setSourcePort(sourceDevice->port);
        } else {
            setSourcePort(nullptr);
        }
    }

    /** @return the device currently bound as the correction source, or nullptr if none is bound. */
    device_handle_t& getSourceDevice() { return sourceDevice; }

    /** @return the unique Id of the device currently bound as the correction source, or 0 if none is bound. */
    uint64_t getSourceDeviceID() { return srcDeviceId; }

    /** @return the TCP port this server is currently listening on. */
    int getListenIpPort() { return factoryOptions.listenerPort; }

    /** @return the local IP address this server is currently listening on, in dotted-decimal notation. */
    std::string getListenIpAddress() { return std::string( inet_ntoa(factoryOptions.listeningAddr.sin_addr) ); }

    /** @return the number of currently connected TCP clients (downstream ports). */
    int getActiveClients() { return (int)ports.size(); }

    // re-expose the listener-error forensic accessor (TcpServerPortFactory is inherited as protected)
    using TcpServerPortFactory::getLastListenError;

private:
    uint64_t        srcDeviceId = 0;             //!< selected device UID (derived from hdwId + SN)
    device_handle_t sourceDevice = nullptr;      //!< device currently bound as the correction source, or nullptr if none

};


#endif //IS_SDK_RTCM3_CORRECTION_SERVER_H
