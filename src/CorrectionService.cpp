/**
 * @file CorrectionService.cpp
 * @brief Class used to distribute an incoming RTCM stream of corrections to "Rover" devices
 *
 * @author FiriusFoxx on 9/4/25.
 * @copyright Copyright (c) 2025 Inertial Sense, Inc. Licensed under the MIT license
 */

#include "CorrectionService.h"

#include <utility>
#include <algorithm>
#include <string>

#include "core/msg_logger.h"
#include "PortManager.h"
#include "message_stats.h"

CorrectionService::CorrectionService(const std::string& portName, const std::vector<PortFactory*>& factories) {
    PortManager* portManager = &PortManager::getInstance();
    std::vector<PortFactory*> localFactories = factories.empty() ? portManager->getPortFactories() : factories;
    PortFactory* portFactory = nullptr;
    uint16_t type = PORT_TYPE__UNKNOWN;
    std::string name; // Final portName
    for (auto& factory : localFactories) {
        factory->locatePorts([&portFactory, &type, &name](PortFactory* pfactory, uint16_t ptype, const std::string& pname) {
            portFactory = pfactory;
            type = ptype;
            name = pname;
        }, portName, PORT_TYPE__COMM);
        if (portFactory && !name.empty()) {
            break;
        }
    }
    if (portFactory && !name.empty()) {
        port_handle_t port = portFactory->bindPort(name, type);
        if (port) {
            localSrcPort = true;
            srcPortFactory = portFactory;
            init(port);
            return;
        }
    }
    throw std::invalid_argument("Couldn't find port of given name to use as RCTM3 corrections source");
}

bool CorrectionService::hasPort(port_handle_t port) {
    return std::find(this->ports.begin(), this->ports.end(), port) != this->ports.end();
}

void CorrectionService::addPort(port_handle_t port) {
    if (!hasPort(port))
        this->ports.push_back(port);
}

void CorrectionService::removePort(port_handle_t port) {
    ports.erase(std::remove(ports.begin(), ports.end(), port), ports.end());
}

void CorrectionService::addDevice(device_handle_t device) {
    addPort(device->port);
}

void CorrectionService::removeDevice(device_handle_t device) {
    removePort(device->port);
}

bool CorrectionService::hasDevice(device_handle_t device) {
    return hasPort(device->port);
}

uint32_t CorrectionService::addRTCM3Msg1029Listeners(const std::function<void(std::string)>& callback) {
    rtcm3Msg1029Listeners.push_back(callback);
    return (uint32_t)rtcm3Msg1029Listeners.size()-1;
}

void CorrectionService::removeRTCM3Msg1029Listeners(const uint32_t id) {
    if (id < rtcm3Msg1029Listeners.size()) {
        rtcm3Msg1029Listeners.erase(rtcm3Msg1029Listeners.begin() + id);
    }
}

uint32_t CorrectionService::addRTCM3PacketListeners(const std::function<void(uint16_t, const void*, uint32_t)>& callback) {
    rtcm3PacketListeners.push_back(callback);
    return (uint32_t)rtcm3PacketListeners.size()-1;
}

void CorrectionService::removeRTCM3PacketListeners(const uint32_t id) {
    if (id < rtcm3PacketListeners.size()) {
        rtcm3PacketListeners.erase(rtcm3PacketListeners.begin() + id);
    }
}

int CorrectionService::step() {
    if (!portIsOpened(source)) {
        if ((lastConnAttemptTs < current_timeMs()) && (portOpen(source) != PORT_ERROR__NONE)) {
            lastConnAttemptTs = current_timeMs() + 2500;    // retry again in 2.5 seconds
        }
        if (!portIsOpened(source))
            return -1;
    }

    unsigned int rtcm3PacketsProcessedPrevCount = rtcm3PacketsProcessed;
    is_comm_port_parse_messages(source);
    return (int)(rtcm3PacketsProcessed - rtcm3PacketsProcessedPrevCount);
}

/**
 * This is a stub packet transformer which just copies the data from the input buffer into the destination buffer
 * This implementation currently only makes sense for RTCM3 packets as they are processed on device
 * @param inputBuffer Data to be transformed
 * @param inputLength Length of input buffer
 * @param finalBuffer To be transmitted to the device
 * @param finalBufferSize The size of the final buffer
 * @return Number of bytes processed
 */
uint32_t CorrectionService::packetTransformer(const uint8_t *inputBuffer, const uint32_t inputLength, uint8_t *finalBuffer, const uint32_t finalBufferSize) {
    (void)inputBuffer;
    (void)inputLength;
    (void)finalBuffer;
    (void)finalBufferSize;
    return 0;
}

int CorrectionService::finalPacketFilter(const uint8_t *inputBuffer, const uint32_t inputLength, uint8_t *finalBuffer, const uint32_t finalBufferSize, uint32_t *bytesProcessed) {
    is_comm_instance_t *comm = &(packetParser);
    protocol_type_t ptype = _PTYPE_NONE;
    static int error = 0;

    int packetsProcessed = 0;
    *bytesProcessed = 0;

    // Get available size of comm buffer.  is_comm_free() modifies comm->rxBuf pointers, call it before using comm->rxBuf.tail.
    uint32_t bufferSpace = is_comm_free(comm);

    // Copy data to parsing buffer
    for (uint32_t i = 0; (i < bufferSpace) && (i < inputLength); i++) {
        *(comm->rxBuf.tail) = inputBuffer[i];
        comm->rxBuf.tail++;
    }

    if (inputLength > 0) {
        // Run the parser
        while ((ptype = is_comm_parse(comm)) != _PTYPE_NONE) {
            switch (ptype)
            {
                case _PTYPE_RTCM3:
                    if (comm->rxPkt.data.size + (*bytesProcessed) > finalBufferSize) {
                        printf("CorrectionService::finalPacketFilter() Destination buffer out of space, dropping packet");
                        return packetsProcessed; // There is no more room in the final buffer
                    }
                    for (uint32_t i = 0; i < comm->rxPkt.data.size; i++) {
                        finalBuffer[*bytesProcessed] = comm->rxPkt.data.ptr[i];
                        (*bytesProcessed)++;
                    }
                    packetsProcessed++;

                    if ((comm->rxPkt.id == 1029) && (comm->rxPkt.data.size < 1024))
                    {
                        std::string msg = std::string().assign(reinterpret_cast<char*>(comm->rxPkt.data.ptr + 12), comm->rxPkt.data.size - 12);
                        for (const tRTCM3Msg1029ListenerCallback& callback: rtcm3Msg1029Listeners) {
                            callback(msg);
                        }
                    }
                    break;

                case _PTYPE_PARSE_ERROR:
                    if (error)
                    {   // Don't print first error.  Likely due to port having been closed.
                        printf("CorrectionService::finalPacketFilter() PARSE ERROR count: %d\n", error);
                    }
                    error++;
                    break;

                default:
                    break;
            }
        }
    }
    return packetsProcessed;
}

void CorrectionService::sendData(const uint8_t *inputBuffer, const uint32_t inputLength) {
    std::vector<port_handle_t> deadPorts;

    for (const auto port: ports) {
        switch (portWrite(port, inputBuffer, inputLength)) {
            case PORT_ERROR__INVALID:
                deadPorts.push_back(port);  // invalid port - did the device reboot?
                break;
        }
    }

    // cleanup dead ports...
    for (const auto port: deadPorts) {
        removePort(port);
    }
}

void CorrectionService::setSourcePort(port_handle_t srcPort) {
    if (source != nullptr) {
        if (portIsValid(source)) {
            is_comm_register_port_msg_handler(source, _PTYPE_RTCM3, previousRtcm3Handler);
            is_comm_register_port_msg_handler(source, _PTYPE_PARSE_ERROR, previousErrorHandler);
        }
    }

    if (!srcPort) {     // if null, then we're intentionally clearing the source port
        source = nullptr;
        return;
    }

    if (!portIsValid(srcPort))  // if not null, but neither valid, ignore it
        return;

    source = srcPort;
    COMM_PORT(source)->comm.cb.context = this;
    previousRtcm3Handler = is_comm_register_port_msg_handler(source, _PTYPE_RTCM3, [](void* ctx, const unsigned char* msg, int msgSize, port_handle_t port) {
        auto* cs = static_cast<CorrectionService*>(ctx);
        return (cs && cs->source == port) ? cs->onRtcm3Handler(msg, msgSize, port) : -1;
    });
    previousErrorHandler = is_comm_register_port_msg_handler(source, _PTYPE_PARSE_ERROR, [](void* ctx, const unsigned char* msg, int msgSize, port_handle_t port) {
        auto* cs = static_cast<CorrectionService*>(ctx);
        return (cs && cs->source == port) ? cs->onRawDataHandler(msg, msgSize, port) : -1;
    });
}

void CorrectionService::init(port_handle_t srcPort) {
    ports.clear();
    rtcm3Msg1029Listeners.clear();
    rtcm3PacketListeners.clear();
    setSourcePort(srcPort);
}

int CorrectionService::onRtcm3Handler(const unsigned char* msg, int msgSize, port_handle_t port) {
    (void)port;
    rtcm3PacketsProcessed++;
    rtcm3PacketLastMs = current_timeMs();

    if (msgStats) MessageStats::append("", *msgStats, _PTYPE_RTCM3, COMM_PORT(source)->comm.rxPkt.id, COMM_PORT(source)->comm.rxPkt.data.size, rtcm3PacketLastMs);

    if ((COMM_PORT(source)->comm.rxPkt.id == 1029) && (COMM_PORT(source)->comm.rxPkt.data.size < 1024))
    {
        const std::string notice = std::string().assign(reinterpret_cast<char*>(COMM_PORT(source)->comm.rxPkt.data.ptr + 12), COMM_PORT(source)->comm.rxPkt.data.size - 12);
        for (const tRTCM3Msg1029ListenerCallback& callback: rtcm3Msg1029Listeners) {
            callback(notice);
        }
    }

    for (const tRTCM3PacketListenerCallback& callback: rtcm3PacketListeners) {
        void* tmp = malloc(COMM_PORT(source)->comm.rxPkt.data.size);
        memcpy(tmp, COMM_PORT(source)->comm.rxPkt.data.ptr, COMM_PORT(source)->comm.rxPkt.data.size);
        callback(COMM_PORT(source)->comm.rxPkt.id, tmp, COMM_PORT(source)->comm.rxPkt.data.size);
        free(tmp);
    }

    sendData(msg, msgSize);
    return 0;
}

int CorrectionService::onRawDataHandler(const unsigned char* msg, int msgSize, port_handle_t port) {
    (void)port;
    uint8_t bufferA[PKT_BUF_SIZE] = {0};
    uint8_t bufferB[PKT_BUF_SIZE] = {0};

    uint32_t bytesRead = packetTransformer(msg, msgSize, bufferA, PKT_BUF_SIZE);
    rtcm3PacketsProcessed += finalPacketFilter(bufferA, bytesRead, bufferB, PKT_BUF_SIZE, &bytesRead);
    sendData(bufferB, bytesRead);
    return 0;
}
