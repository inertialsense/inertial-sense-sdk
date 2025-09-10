/**
 * @file CorrectionService.cpp
 * @brief Class used to distribute an incoming RTCM stream of corrections to "Rover" devices
 *
 * @author FiriusFoxx on 9/4/25.
 * @copyright Copyright (c) 2025 Inertial Sense, Inc. Licensed under the MIT license
 */

#include "CorrectionService.h"

#include <utility>

#include "PortManager.h"

CorrectionService::CorrectionService(port_handle_t srcPort) {
    this->source = srcPort;
    this->devices = std::vector<ISDevice*>();
    this->rtcm3Msg1029Listeners = std::vector<tRTCM3Msg1029ListenerCallback>();
    is_comm_init(&this->packetParser, this->packetBuffer, sizeof(this->packetBuffer), NULL);
}

CorrectionService::CorrectionService(const std::string& portName, const std::vector<PortFactory*>& factories) {
    PortFactory* portFactory = nullptr;
    uint16_t type = PORT_TYPE__UNKNOWN;
    std::string name; // Final portName
    for (auto& factory : factories) {
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
        *this = CorrectionService(portFactory->bindPort(name, type));
        return;
    }
    throw std::invalid_argument("Couldn't find port of given name to use as RCTM3 corrections source");
}

CorrectionService::CorrectionService(const std::string& portName) {
    PortManager* portManager = &PortManager::getInstance();
    const std::vector<PortFactory*> factories = portManager->getPortFactories();
    *this = CorrectionService(portName, factories);
}

void CorrectionService::addDevice(ISDevice* device) {
    this->devices.push_back(device);
}

void CorrectionService::removeDevice(ISDevice* device) {
    std::erase(this->devices, device);
}

bool CorrectionService::hasDevice(ISDevice* device) {
    return std::ranges::find(this->devices, device) != this->devices.end();
}

uint32_t CorrectionService::addRTCM3Msg1029Listeners(const std::function<void(std::string)>& callback) {
    this->rtcm3Msg1029Listeners.push_back(callback);
    return this->rtcm3Msg1029Listeners.size()-1;
}

void CorrectionService::removeRTCM3Msg1029Listeners(const uint32_t id) {
    this->rtcm3Msg1029Listeners.erase(this->rtcm3Msg1029Listeners.begin() + id);
}

int CorrectionService::step() {
    uint8_t bufferA[PKT_BUF_SIZE];
    uint8_t bufferB[PKT_BUF_SIZE];

    uint32_t bytesRead = portRead(source,bufferA,PKT_BUF_SIZE);
    bytesRead = packetTransformer(bufferA, bytesRead, bufferB, PKT_BUF_SIZE);
    int rtcm3Pkts = finalPacketFilter(bufferB, bytesRead, bufferA, PKT_BUF_SIZE, &bytesRead);
    sendData(bufferA, bytesRead);
    return rtcm3Pkts;
}

/**
 * This is a stub packet transformer which just copies the data from the input buffer into the destination buffer
 * This currently only makes sense for RTCM3 packets as they are processed on device
 * @param inputBuffer Data to be transformed
 * @param inputLength Length of input buffer
 * @param finalBuffer To be transmitted to the device
 * @param finalBufferSize The size of the final buffer
 * @return Number of bytes processed
 */
int CorrectionService::packetTransformer(const uint8_t *inputBuffer, const uint32_t inputLength, uint8_t *finalBuffer, const uint32_t finalBufferSize) {
    int totalBytes = 0;
    for (uint32_t i = 0; (i < inputLength) || (i < finalBufferSize); i++) {
        finalBuffer[i] = inputBuffer[i];
        totalBytes++;
    }
    return totalBytes;
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
    for (uint32_t i = 0; (i < bufferSpace) || (i < inputLength); i++) {
        comm->rxBuf.tail[i] = inputBuffer[i];
    }

    // Update comm buffer tail pointer
    comm->rxBuf.tail += inputLength;

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
    for (auto device: devices) {
        device->SendRaw(inputBuffer, inputLength);
    }
}
