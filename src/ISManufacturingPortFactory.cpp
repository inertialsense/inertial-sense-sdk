//
// Created by firiusfoxx on 7/3/25.
//

#include <util.h>
#include <chrono>
#include "ISManufacturingPortFactory.h"

#include <sys/sysmacros.h>

#include <regex>

#include "PortManager.h"
#include "mdns.h"

static std::chrono::time_point<std::chrono::steady_clock> lastQueryTime;

/**
 * This function parses and creates a new port_handle_t repersenting a TCP Port
 * when passed a URL in the format is-manu://hostname/port to pName and a pType of PORT_TYPE__TCP | PORT_TYPE__COMM
 * @param pName The URL and name of the new port to bind a port_handle_to
 * @param pType The port type requested to be generated
 * @return A port_handle_t bound to the newly created TCP port for the connection pName represents
 */
port_handle_t ISManufacturingPortFactory::bindPort(const std::string& pName, uint16_t pType) {
    if (!validatePort(pName, pType)) {
        return nullptr;
    }

    // Parse pName for address
    const utils::URL url = utils::parseURL(pName);
    if (url.protocol != "is-manu") {
        return nullptr;
    }

    tick(); // Tick everything to ensure we have the latest data

    // MDNS / DNS-SD Resolve goes here

    return nullptr;

    auto* tcpPort = new tcp_port_t;
    auto port = (port_handle_t)tcpPort;
    *tcpPort = {};
    auto id = static_cast<uint16_t>(PortManager::getInstance().getPortCount());
    //tcpPortInit(port, id, this->portOptions.defaultBlocking, pName.c_str(), &addr);

    return port;
}

/**
 * Releases and frees the memory used by this port
 * @param port The TCP Port handle to deinitialize
 * @return True if successful, false otherwise
 */
bool ISManufacturingPortFactory::releasePort(port_handle_t port) {
    if (!port) {
        return false;
    }

    debug_message("[DBG] Releasing network port '%s'\n", ((tcp_port_t*)port)->portName);
    tcpPortDelete(port);
    delete static_cast<tcp_port_t*>(port);

    return true;
}

/**
 * Validate that a provided pName can create a TCP Port
 * @param pName The URL to validate starting with is-manu://
 * @param pType Must be PORT_TYPE__TCP | PORT_TYPE__COMM
 * @return True if port can be created, false otherwise
 */
bool ISManufacturingPortFactory::validatePort(const std::string& pName, uint16_t pType) {
    const utils::URL url = utils::parseURL(pName);
    if (url.protocol != "is-manu") {
        return false;
    }

    if (pType != (PORT_TYPE__TCP | PORT_TYPE__COMM)) {
        return false;
    }

    tick(); // Tick everything to ensure we have the latest data

    return false;
    // Validate existence of MDNS / DNS-SD records
}

/**
 * TCP Port Factory implements a stub function that only creates 1 port if the provided pattern is a valid TCP port url
 * @param portCallback The function to callback into to indicate that this port has been "found"
 * @param pattern The URL to validate and "discover"
 * @param pType Ignored
 */
void ISManufacturingPortFactory::locatePorts(std::function<void(PortFactory*, uint16_t, std::string)> portCallback, const std::string& pattern, uint16_t pType) {
    tick(); // Tick everything to ensure we have the latest data

    std::unordered_map<std::string, std::vector<port_t>> portsAndHosts = getPorts();
    for (const std::pair<std::string, std::vector<port_t>> ph : portsAndHosts) {
        std::string hostname = ph.first;
        std::vector<port_t> ports = ph.second;
        for (const port_t port : ports) {
            std::string portString;
        }
    }
}

/**
 * Tick function used to call the mdns::tick() function
 */
void ISManufacturingPortFactory::tick() {
    // Send a lookup for all devices advertising inertialsense-manufacturing every 150 ms
    if (std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - lastQueryTime).count() > IS_MANUFACTURING_PORT_FACTORY_TIME_BETWEEN_QUERIES_MS) {
        mdns::sendQuery(MDNS_RECORDTYPE_PTR, "_inertialsense-manufacturing._tcp.local");
        lastQueryTime = std::chrono::steady_clock::now();
    }

    // Allow the MDNS responder to do all of it's processing
    mdns::tick();
}

/**
 * Parse a port's name to get info about the hostname, devnum and port doesn't confirm that the port exists
 * @param pName Name to parse
 * @return pair with first value as server hostname and second value as struct containing devnum and port
 */
std::pair<std::string, ISManufacturingPortFactory::port_t> ISManufacturingPortFactory::parsePortName(const std::string& pName) {
    const utils::URL url = utils::parseURL(pName);
    if (url.protocol != "is-manu") throw std::invalid_argument("Invalid URL Protocol");

    if (!utils::validDomainName(url.address)) throw std::invalid_argument("Address of URL is not a valid DNS Domain Name");
    if (!(url.address.ends_with(".local") || url.address.ends_with(".local."))) throw std::invalid_argument("Address of URL doesn't end in .local");

    std::string hostname = url.address;
    if (!hostname.ends_with(".")) {
        hostname.append("");
    }

    uint32_t devid = 0;
    uint16_t port = 0;

    if (url.port.empty() && url.path.empty()) throw std::invalid_argument("port or path must be specified in URL");
    if (!url.port.empty()) {
        if (std::find_if(url.port.begin(), url.port.end(), [](unsigned char c) { return !std::isdigit(c); }) != url.port.end()) {
            throw std::invalid_argument("port in URL is not a number");
        }
        int largePort = 0;
        try {
            largePort = std::stoi(url.port);
        } catch (const std::invalid_argument& e) {
            throw std::invalid_argument("Unable to parse port number");
        }
        if (largePort < 1 || largePort > 65535) throw std::invalid_argument("Port is out of bounds");
        port = static_cast<uint16_t>(largePort);
    }

    if (!url.path.empty()) {
        int major = 0; // 12 bits
        int minor = 0; // 20 bits
        std::regex regexp1(R"(^([0-9]+):([0-9]+)$)");
        std::regex regexp2(R"(^dev\/(.*)$)");
        std::smatch match;

        if (std::regex_match(url.path, match, regexp1)) {
            try {
                major = stoi(match[1].str());
            } catch (const std::invalid_argument& e) {
                throw std::invalid_argument("Unable to parse major number");
            }
            try {
                minor = stoi(match[2].str());
            } catch (const std::invalid_argument& e) {
                throw std::invalid_argument("Unable to parse minor number");
            }

            if (major > 4095) throw std::invalid_argument("Major number is out of bounds");
            if (minor > 1048575) throw std::invalid_argument("Minor number is out of bounds");
            devid = makedev(major, minor);
        } else if (std::regex_match(url.path, match, regexp2)) {
            for (std::pair<uint16_t, std::string> majorPair: majorAtlas) {
                if (match[1].str().starts_with( majorPair.second)) {
                    major = majorPair.first;
                    try {
                        minor = std::stoi(match[1].str().substr(majorPair.second.size()));
                    } catch (const std::invalid_argument& e) {
                        throw std::invalid_argument("Unable to parse device number");
                    }
                    break;
                }
            }
            if (major == 0) throw std::invalid_argument("Unknown device path to major");
            if (major > 4095) throw std::invalid_argument("Major number atlas is invalid (Major number out of bounds)");
            if (minor > 1048575) throw std::invalid_argument("Device number is out of bounds");
            devid = makedev(major, minor);
        } else {
            throw std::invalid_argument("Unknow format for URL path");
        }
    }

    port_t portData = {devid, port};
    return {hostname, portData};
}

/**
 * Validate's just the port's name not it's existence
 * @param pName Name to validate
 * @return True if port name is valid
 */
bool ISManufacturingPortFactory::validatePortName(const std::string& pName) {
    const utils::URL url = utils::parseURL(pName);
    if (url.protocol != "is-manu") return false;

    if (!utils::validDomainName(url.address)) return false;
    if (!(url.address.ends_with(".local") || url.address.ends_with(".local."))) return false;

    if (url.port.empty() && url.path.empty()) return false;
    if (!url.port.empty() && std::find_if(url.port.begin(), url.port.end(), [](unsigned char c) { return !std::isdigit(c); }) != url.port.end()) return false;

    std::regex regexp(R"(^[0-9]+:[0-9]+$)");
    std::smatch match;
    if (!url.path.empty() && !std::regex_match(url.path, match, regexp)) {
        std::regex regexp2(R"(^dev\/(.*)$)");
        std::smatch match2;
        if (std::regex_match(url.path, match2, regexp2)) {
            for (std::pair<uint16_t, std::string> majorPair: majorAtlas) {
                if (match2[1].str().starts_with( majorPair.second)) {
                    return true;
                }
            }
        }
        return false;
    }
    return true;
}

std::unordered_map<std::string, std::vector<ISManufacturingPortFactory::port_t>> ISManufacturingPortFactory::getPorts() {
    // Locate ports matching pattern using MDNS / DNS-SD records
    std::vector<mdns::mdns_record_cpp_t> PTRrecords = mdns::getRecords([](const mdns::mdns_record_cpp_t& record) -> bool {
        return record.type == MDNS_RECORDTYPE_PTR && record.name == "_inertialsense-manufacturing._tcp.local.";
    });

    std::unordered_map<std::string, std::vector<port_t>> returnValue;

    // For every service located get the SRV record (for the hostname) and for each port in it's TXT records create a port with hostname
    for (const mdns::mdns_record_cpp_t& PTRrecord : PTRrecords) {
        std::vector<mdns::mdns_record_cpp_t> SRVrecords = mdns::getRecords([PTRrecord](const mdns::mdns_record_cpp_t& record) -> bool {
            return record.type == MDNS_RECORDTYPE_SRV && record.name == PTRrecord.data.ptr.name;
        });
        if (SRVrecords.empty()) {continue;}
        std::string hostname = SRVrecords[0].data.srv.name;
        std::vector<mdns::mdns_record_cpp_t> TXTrecords = mdns::getRecords([PTRrecord](const mdns::mdns_record_cpp_t& record) -> bool {
            if (record.type == MDNS_RECORDTYPE_TXT && record.name == PTRrecord.data.ptr.name && record.data.txt.key.length() > 5 && record.data.txt.key.starts_with("ports")) {
                std::string txtPortsIndex = record.data.txt.key.substr(5);
                if (std::find_if(txtPortsIndex.begin(), txtPortsIndex.end(), [](unsigned char c) { return !std::isdigit(c); }) == txtPortsIndex.end()) {
                    return record.data.txt.value.length() % 6 == 0 && record.data.txt.value.length() > 0; // Length of value must be divide by 6 with no remainder and be greater then 0
                }
            }
            return false;
        });

        std::vector<std::vector<port_t>> portListNonAssembled;
        portListNonAssembled.resize(TXTrecords.size());
        for (const mdns::mdns_record_cpp_t& TXTrecord : TXTrecords) {
            std::string txtPortsIndex = TXTrecord.data.txt.key.substr(5);
            int portsIndex = std::stoi(txtPortsIndex);
            const char *portTxt = TXTrecord.data.txt.value.c_str();
            for (int j = 0; j < TXTrecord.data.txt.value.length(); j = j + 6) {
                port_t port;
                port.devid = ntohl(*(uint32_t*)&portTxt[j]);
                port.port = ntohs(*(uint16_t*)&portTxt[j+4]);
                portListNonAssembled[portsIndex].push_back(port);
            }
        }

        std::vector<port_t> portList;
        for (std::vector<port_t> segement : portListNonAssembled) {
            portList.insert(portList.end(), segement.begin(), segement.end());
        }
        returnValue.insert_or_assign(hostname, portList);
    }
    return returnValue;
}