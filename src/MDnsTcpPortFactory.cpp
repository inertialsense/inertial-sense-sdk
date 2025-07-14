//
// Created by firiusfoxx on 7/3/25.
//

#include <util.h>
#include <ifaddrs.h>
#include <functional>
#include <net/if.h>
#include "mdns/mdns.h"
#include "MDnsTcpPortFactory.h"
#include "PortManager.h"

/**
 * This function parses and creates a new port_handle_t repersenting a TCP Port
 * when passed a URL in the format is-manu://hostname/port to pName and a pType of PORT_TYPE__TCP | PORT_TYPE__COMM
 * @param pName The URL and name of the new port to bind a port_handle_to
 * @param pType The port type requested to be generated
 * @return A port_handle_t bound to the newly created TCP port for the connection pName represents
 */
port_handle_t MDnsTcpPortFactory::bindPort(const std::string& pName, uint16_t pType) {
    if (!validatePort(pName, pType)) {
        return nullptr;
    }

    // Parse pName for address
    const utils::URL url = utils::parseURL(pName);
    if (url.protocol != "is-manu") {
        return nullptr;
    }

    if (create_mdns_sockets() <= 0) {
        debug_message("[WRN] Failed to open any sockets to listen for MDNS responses on");
        return nullptr;
    }

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
bool MDnsTcpPortFactory::releasePort(port_handle_t port) {
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
bool MDnsTcpPortFactory::validatePort(const std::string& pName, uint16_t pType) {
    const utils::URL url = utils::parseURL(pName);
    if (url.protocol != "is-manu") {
        return false;
    }

    if (pType != (PORT_TYPE__TCP | PORT_TYPE__COMM)) {
        return false;
    }

    if (create_mdns_sockets() <= 0) {
        debug_message("[WRN] Failed to open any sockets to listen for MDNS responses on");
        return false;
    }

    return false;
    // Validate existence of MDNS / DNS-SD records
}

/**
 * TCP Port Factory implements a stub function that only creates 1 port if the provided pattern is a valid TCP port url
 * @param portCallback The function to callback into to indicate that this port has been "found"
 * @param pattern The URL to validate and "discover"
 * @param pType Ignored
 */
void MDnsTcpPortFactory::locatePorts(std::function<void(PortFactory*, uint16_t, std::string)> portCallback, const std::string& pattern, uint16_t pType) {
    if (create_mdns_sockets() <= 0) {
        debug_message("[WRN] Failed to open any sockets to listen for MDNS responses on");
        return;
    }

    send_mdns_query(MDNS_RECORDTYPE_PTR, "_inertialsense-manufacturing._tcp.local");
    handle_mdns_query_responses();

    // Locate ports matching pattern using MDNS / DNS-SD records
}

/**
 * This function creates a number of sockets for the MDNS Port Factory to listen on to discover services
 * It is copied and pasted from mdns.c with a few modifications
 */
int MDnsTcpPortFactory::create_mdns_sockets() {
    // We should only open the sockets once if they are already open just return the number opened
    if (socketsOpened > 0) {
        return socketsOpened;
    }

    // When sending, each socket can only send to one network interface
    // Thus we need to open one socket for each interface and address family
    int num_sockets = 0;
    int max_sockets = sizeof(mdnsSockets) / sizeof(mdnsSockets[0]);
    int port = 0;

#ifdef _WIN32

    IP_ADAPTER_ADDRESSES* adapter_address = 0;
    ULONG address_size = 8000;
    unsigned int ret;
    unsigned int num_retries = 4;
    do {
        adapter_address = (IP_ADAPTER_ADDRESSES*)malloc(address_size);
        ret = GetAdaptersAddresses(AF_UNSPEC, GAA_FLAG_SKIP_MULTICAST | GAA_FLAG_SKIP_ANYCAST, 0,
                                   adapter_address, &address_size);
        if (ret == ERROR_BUFFER_OVERFLOW) {
            free(adapter_address);
            adapter_address = 0;
            address_size *= 2;
        } else {
            break;
        }
    } while (num_retries-- > 0);

    if (!adapter_address || (ret != NO_ERROR)) {
        free(adapter_address);
        printf("Failed to get network adapter addresses\n");
        return num_sockets;
    }

    for (PIP_ADAPTER_ADDRESSES adapter = adapter_address; adapter; adapter = adapter->Next) {
        if (adapter->TunnelType == TUNNEL_TYPE_TEREDO)
            continue;
        if (adapter->OperStatus != IfOperStatusUp)
            continue;

        for (IP_ADAPTER_UNICAST_ADDRESS* unicast = adapter->FirstUnicastAddress; unicast;
             unicast = unicast->Next) {
            if (unicast->Address.lpSockaddr->sa_family == AF_INET) {
                struct sockaddr_in* saddr = (struct sockaddr_in*)unicast->Address.lpSockaddr;
                if ((saddr->sin_addr.S_un.S_un_b.s_b1 != 127) ||
                    (saddr->sin_addr.S_un.S_un_b.s_b2 != 0) ||
                    (saddr->sin_addr.S_un.S_un_b.s_b3 != 0) ||
                    (saddr->sin_addr.S_un.S_un_b.s_b4 != 1)) {
                    if (num_sockets < max_sockets) {
                        saddr->sin_port = htons((unsigned short)port);
                        int sock = mdns_socket_open_ipv4(saddr);
                        if (sock >= 0) {
                            sockets[num_sockets++] = sock;
                        }
                    }
                }
            } else if (unicast->Address.lpSockaddr->sa_family == AF_INET6) {
                struct sockaddr_in6* saddr = (struct sockaddr_in6*)unicast->Address.lpSockaddr;
                //// Ignore link-local addresses
                //if (saddr->sin6_scope_id)
                //    continue;
                static const unsigned char localhost[] = {0, 0, 0, 0, 0, 0, 0, 0,
                                                          0, 0, 0, 0, 0, 0, 0, 1};
                static const unsigned char localhost_mapped[] = {0, 0, 0,    0,    0,    0, 0, 0,
                                                                 0, 0, 0xff, 0xff, 0x7f, 0, 0, 1};
                if ((unicast->DadState == NldsPreferred) &&
                    memcmp(saddr->sin6_addr.s6_addr, localhost, 16) &&
                    memcmp(saddr->sin6_addr.s6_addr, localhost_mapped, 16)) {
                    if (num_sockets < max_sockets) {
                        saddr->sin6_port = htons((unsigned short)port);
                        int sock = mdns_socket_open_ipv6(saddr);
                        if (sock >= 0) {
                            mdnsSockets[num_sockets++] = sock;
                        }
                    }
                }
            }
        }
    }

    free(adapter_address);

#else

    struct ifaddrs* ifaddr = 0;
    struct ifaddrs* ifa = 0;

    if (getifaddrs(&ifaddr) < 0)
        printf("Unable to get interface addresses\n");

    for (ifa = ifaddr; ifa; ifa = ifa->ifa_next) {
        if (!ifa->ifa_addr)
            continue;
        if (!(ifa->ifa_flags & IFF_UP) || !(ifa->ifa_flags & IFF_MULTICAST))
            continue;
        if ((ifa->ifa_flags & IFF_LOOPBACK) || (ifa->ifa_flags & IFF_POINTOPOINT))
            continue;

        if (ifa->ifa_addr->sa_family == AF_INET) {
            struct sockaddr_in* saddr = (struct sockaddr_in*)ifa->ifa_addr;
            if (saddr->sin_addr.s_addr != htonl(INADDR_LOOPBACK)) {
                if (num_sockets < max_sockets) {
                    saddr->sin_port = htons(port);
                    int sock = mdns_socket_open_ipv4(saddr);
                    if (sock >= 0) {
                        mdnsSockets[num_sockets++] = sock;
                    }
                }
            }
        } else if (ifa->ifa_addr->sa_family == AF_INET6) {
            struct sockaddr_in6* saddr = (struct sockaddr_in6*)ifa->ifa_addr;
            //// Ignore link-local addresses
            //if (saddr->sin6_scope_id)
            //    continue;
            static const unsigned char localhost[] = {0, 0, 0, 0, 0, 0, 0, 0,
                                                      0, 0, 0, 0, 0, 0, 0, 1};
            static const unsigned char localhost_mapped[] = {0, 0, 0,    0,    0,    0, 0, 0,
                                                             0, 0, 0xff, 0xff, 0x7f, 0, 0, 1};
            if (memcmp(saddr->sin6_addr.s6_addr, localhost, 16) &&
                memcmp(saddr->sin6_addr.s6_addr, localhost_mapped, 16)) {
                if (num_sockets < max_sockets) {
                    saddr->sin6_port = htons(port);
                    int sock = mdns_socket_open_ipv6(saddr);
                    if (sock >= 0) {
                        mdnsSockets[num_sockets++] = sock;
                    }
                }
            }
        }
    }

    freeifaddrs(ifaddr);

#endif
    socketsOpened = num_sockets;
    return num_sockets;
}

/**
 * Sends out a MDNS Query on all interfaces/sockets
 */
void MDnsTcpPortFactory::send_mdns_query(mdns_record_type_t type, std::string query) {
    size_t capacity = 2048;
    void* buffer = malloc(capacity);
    if (buffer == nullptr) {
        printf("[ERR] Failed to allocate memory for MDNS query are you out of memory?\n");
        return;
    }
    for (int isock = 0; isock < socketsOpened; ++isock) {
        if (mdns_query_send(mdnsSockets[isock], type, query.c_str(), strlen(query.c_str()), buffer, capacity, 0)) {
            debug_message("[WRN] Failed to send DNS-DS discovery: %s\n", strerror(errno));
        }
    }
    free(buffer);
}

/**
 * Callback called from mdns.h for each record
 * @param sock Socket that returned this record
 * @param from Address that sent this record
 * @param addrlen Length of address
 * @param entry MDNS Entry Type
 * @param query_id Query ID that caused this response
 * @param rtype Record Type
 * @param rclass Record Class
 * @param ttl Record TTL
 * @param data Raw Data
 * @param size Size of data
 * @param name_offset Offset to look for Name in data
 * @param name_length Length of name to look for in data
 * @param record_offset Offset of record to look for in data
 * @param record_length Length of record in data
 * @param user_data User data stored in record
 * @return Non-zero indicates error, Zero indicates Success
 */
int MDnsTcpPortFactory::query_callback(int sock, const struct sockaddr* from, size_t addrlen, mdns_entry_type_t entry,
                                       uint16_t query_id, uint16_t rtype, uint16_t rclass, uint32_t ttl, const void* data,
                                       size_t size, size_t name_offset, size_t name_length, size_t record_offset,
                                       size_t record_length, void* user_data) {
    static char namebuffer[256];
    static char entrybuffer[256];
    static mdns_record_txt_t txtbuffer[128];

    mdns_string_t entrystr = mdns_string_extract(data, size, &name_offset, entrybuffer, sizeof(entrybuffer));
    std::string serviceNameFull = std::string(MDNS_STRING_ARGS(entrystr));

    if (rtype == MDNS_RECORDTYPE_PTR) {
        mdns_string_t namestr = mdns_record_parse_ptr(data, size, record_offset, record_length, namebuffer, sizeof(namebuffer));
        send_mdns_query(MDNS_RECORDTYPE_TXT, std::string(MDNS_STRING_ARGS(namestr)));
        return 0;
    }
    if (rtype == MDNS_RECORDTYPE_TXT) {
        size_t parsed = mdns_record_parse_txt(data, size, record_offset, record_length, txtbuffer, sizeof(txtbuffer) / sizeof(mdns_record_txt_t));
        const std::string hostname = utils::split_string(serviceNameFull, ".").front().append(".local");
        send_mdns_query(MDNS_RECORDTYPE_AAAA, hostname);
        send_mdns_query(MDNS_RECORDTYPE_A, hostname);

        for (size_t itxt = 0; itxt < parsed; ++itxt) {
            if (txtbuffer[itxt].value.length) {
                const std::string key = std::string(MDNS_STRING_ARGS(txtbuffer[itxt].key));
                if (key.substr(0, 5) != "ports") {
                    return 0;
                }
                int portIndex = stoi(key.substr(5));
                const std::string value = std::string(MDNS_STRING_ARGS(txtbuffer[itxt].value));

                std::vector<std::vector<port_t>> ports = port_records[hostname];
                if ((portIndex + 1) > ports.size()) {
                    ports.resize(portIndex + 1);
                }
                if (value.length() % 6 != 0) {
                    return 0;
                }

                const char *portTxt = value.c_str();
                for (int i = 0; i < value.length(); i = i + 6) {
                    port_t port;
                    port.devid = ntohl(*(uint32_t*)&portTxt[i]);
                    port.port = ntohs(*(uint16_t*)&portTxt[i+4]);
                    ports[portIndex].push_back(port);
                }

                port_records[hostname] = ports;
            }
        }

        //port_records.insert_or_assign(hostname, );
        return 0;
    }

    debug_message("[WRN] Unknown record type: %d\n", rtype);
    return 1;
}

/**
 * Receives and handles MDNS Query Responses and stores them into the records vectors
 * @return Number of records processed into records vector
 */
int MDnsTcpPortFactory::handle_mdns_query_responses() {
    size_t capacity = 2048;
    void* buffer = malloc(capacity);
    void* user_data = 0;
    int records;

    int res;
    do {
        struct timeval timeout;
        timeout.tv_sec = 0;
        timeout.tv_usec = 0;

        int nfds = 0;
        fd_set readfs;
        FD_ZERO(&readfs);
        for (int isock = 0; isock < socketsOpened; ++isock) {
            if (mdnsSockets[isock] >= nfds)
                nfds = mdnsSockets[isock] + 1;
            FD_SET(mdnsSockets[isock], &readfs);
        }

        records = 0;
        res = select(nfds, &readfs, nullptr, nullptr, &timeout);
        if (res > 0) {
            for (int isock = 0; isock < socketsOpened; ++isock) {
                if (FD_ISSET(mdnsSockets[isock], &readfs)) {
                    records += mdns_query_recv(mdnsSockets[isock], buffer, capacity, query_callback, user_data, 0);
                }
            }
        }
    } while (res > 0);

    free(buffer);

    return records;
}
