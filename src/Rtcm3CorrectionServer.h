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

class Rtcm3CorrectionServer : protected CorrectionService, protected TcpServerPortFactory {
public:
    explicit Rtcm3CorrectionServer(int port = 7777, std::string listenAddr = "127.0.0.1", int max_connections = 10) : TcpServerPortFactory(port, listenAddr, max_connections) {
        startListening();
    };

    explicit Rtcm3CorrectionServer(const device_handle_t srcDevice, int port = 7777, std::string listenAddr = "127.0.0.1", int max_connections = 10) : Rtcm3CorrectionServer(port, listenAddr, max_connections) {
        setSourceDevice(srcDevice);
    };

    explicit Rtcm3CorrectionServer(const uint64_t srcDeviceId, int port = 7777, std::string listenAddr = "127.0.0.1", int max_connections = 10) : Rtcm3CorrectionServer(port, listenAddr, max_connections) {
        setSourceDevice(srcDeviceId);
    };

    ~Rtcm3CorrectionServer() {
        stopListening();        // don't accept new connections
        shutdownAllClients();   // terminate existing connections
    };

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

    void setSourceDevice(uint64_t deviceId) {
        srcDeviceId = deviceId;
        sourceDevice = DeviceManager::getInstance().getDevice(srcDeviceId);
        if (sourceDevice) {
            setSourcePort(sourceDevice->port);
        } else {
            setSourcePort(nullptr);
        }
    }

    device_handle_t& getSourceDevice() { return sourceDevice; }

    uint64_t getSourceDeviceID() { return srcDeviceId; }

    port_handle_t getSourcePort() { return (sourceDevice ? sourceDevice->port : nullptr); }

private:
    uint64_t        srcDeviceId = 0;             // selected device UID (derived from hdwId + SN)
    device_handle_t sourceDevice = nullptr;

};


#endif //IS_SDK_RTCM3_CORRECTION_SERVER_H
