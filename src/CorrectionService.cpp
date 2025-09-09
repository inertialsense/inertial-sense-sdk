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

int CorrectionService::step() {
    return -ENOSYS;
}