//
// Created by firiusfoxx on 7/14/25.
//

#include "mdns.h"
#include <algorithm>
#include <ifaddrs.h>
#include <net/if.h>
#include <chrono>
#include <random>
#include <mutex>
#include "libmdns/mdns.h"

#include <functional>
#include <system_error>

#include "core/msg_logger.h"

/**
 * Handles incoming mdns responses and removes timed out old values
 */
void mdns::tick() {
    std::unique_lock lock(mutex, std::defer_lock);
    if(!lock.try_lock()) return; // If the mutex is locked return

    if (createMdnsSockets() <= 0) {
        debug_message("[WRN] Failed to open any sockets to listen for MDNS responses on");
        return;
    }

    handleMdnsQueryResponses();
    cleanupExpiredResponses();
}

/**
 * Sends out a MDNS Query on all interfaces/sockets
 */
void mdns::sendQuery(mdns_record_type_t type, const std::string& query) {
    size_t capacity = 2048;
    void* buffer = malloc(capacity);
    if (buffer == nullptr) {
        printf("[ERR] Failed to allocate memory for MDNS query are you out of memory?\n");
        return;
    }

    for (int isock = 0; isock < socketsOpened; ++isock) {
        // This code is slightly obtuse, but it simply generates random queryIds until it generates one that isn't already used in usedQueryIds
        uint16_t queryId = 0;
        do {
            queryId = hwRandom();
        } while (std::find_if(usedQueryIds.begin(), usedQueryIds.end(), [&queryId](used_query_id_t x) {
            return x.queryId == queryId;
        }) != usedQueryIds.end());

        if (mdns_query_send(mdnsSockets[isock], type, query.c_str(), strlen(query.c_str()), buffer, capacity, queryId)) {
            debug_message("[WRN] Failed to send DNS-DS discovery: %s\n", strerror(errno));
        }

        used_query_id_t newPair = {queryId, std::chrono::steady_clock::now(), false};
        usedQueryIds.push_back(newPair);
    }

    free(buffer);
}

/**
 * Get discovered MDNS records (newest record in first) where filter returns true
 * @param filter Function describing filter for search
 * @return A vector containing all discovered MDNS records
 */
std::vector<mdns::mdns_record_cpp_t> mdns::getRecords(const std::function<bool (mdns_record_cpp_t)>& filter) {
    std::shared_lock lock(mutex, std::defer_lock);
    bool locked = false;
    try {
        lock.lock();
        locked = true;
    } catch (std::system_error &e) {
        if (e.code() != std::errc::resource_deadlock_would_occur) { // If this thread already has a lock it likely being called from tick() and that is Ok.
            throw e;
        }
    }

    // Copy responses map to vector
    std::vector<std::pair<mdns_record_cpp_t, std::chrono::time_point<std::chrono::steady_clock>>> sortedMap;
    sortedMap.reserve(responses.size());
    for (std::pair<mdns_record_cpp_t, std::chrono::time_point<std::chrono::steady_clock>> pair: responses) {
        if (filter(pair.first)) {
            sortedMap.push_back(pair);
        }
    }
    if (locked) lock.unlock();

    // Sort responses by time
    std::sort(sortedMap.begin(), sortedMap.end(),
              [](const std::pair<mdns_record_cpp_t, std::chrono::time_point<std::chrono::steady_clock>>& a,
                       const std::pair<mdns_record_cpp_t, std::chrono::time_point<std::chrono::steady_clock>>& b) -> bool {
        return a.second > b.second; // Greater than causes the newest record to be first
    });

    // Copy sorted responses into final vector
    std::vector<mdns_record_cpp_t> returnList;
    returnList.reserve(sortedMap.size());
    for (const std::pair<mdns_record_cpp_t, std::chrono::time_point<std::chrono::steady_clock>>& pair: sortedMap) {
        returnList.push_back(pair.first);
    }

    return returnList;
}

/**
 * Get all discovered MDNS records (newest record in first)
 * @return A vector containing all discovered MDNS records
 */
std::vector<mdns::mdns_record_cpp_t> mdns::getRecords() {
    return getRecords([](mdns_record_cpp_t x) -> bool {return true;});
}

/**
 * Resolve hostname using MDNS
 * @return A sockaddr_storage struct representing either a IPv6 or IPv4 Address, or type of AF_UNSPEC if you need to try again or the address doesn't exist
 */
sockaddr_storage mdns::resolveName(const std::string& name) {
    std::vector<mdns_record_cpp_t> records = getRecords();
    sockaddr_storage result = {.ss_family = AF_UNSPEC};
    for (mdns_record_cpp_t& record : records) {
        if (record.name != name) {
            continue;
        }
        switch (record.type) {
            case MDNS_RECORDTYPE_A: {
                memcpy(&result, &record.data.a.addr, sizeof(record.data.a.addr));
                break;
            }
            case MDNS_RECORDTYPE_AAAA: {
                memcpy(&result, &record.data.aaaa.addr, sizeof(record.data.aaaa.addr));
                break;
            }
            default: continue;
        }
        if (result.ss_family != AF_UNSPEC) {
            break;
        }
    }
    if (result.ss_family == AF_UNSPEC) {
        sendQuery(MDNS_RECORDTYPE_AAAA, name);
        sendQuery(MDNS_RECORDTYPE_A, name);
    }

    return result;
}

// Private functions below here

/**
 * This function creates a number of sockets for the MDNS Port Factory to listen on to discover services
 * It is copied and pasted from libmdns/mdns.c with a few modifications
 */
int mdns::createMdnsSockets() {
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
int mdns::queryCallback(int sock, const struct sockaddr* from, size_t addrlen, mdns_entry_type_t entry,
                        uint16_t query_id, uint16_t rtype, uint16_t rclass, uint32_t ttl, const void* data,
                        size_t size, size_t name_offset, size_t name_length, size_t record_offset,
                        size_t record_length, void* user_data) {

    std::list<used_query_id_t>::iterator foundQuery =
        std::find_if(usedQueryIds.begin(), usedQueryIds.end(),
                     [&query_id](used_query_id_t x) -> bool {
            return x.queryId == query_id;
        });

    if (foundQuery == usedQueryIds.end()) {
        debug_message("[ERR] Unable to find received query ID is local list")
        return -EBADMSG;
    } else {
        foundQuery->queryRecieved = true;
    }

    if (entry != MDNS_ENTRYTYPE_ANSWER) {
        debug_message("[WRN] Unable to process non ANSWER responses: Not Supported")
        return -ENOTSUP;
    }

    char entrybuffer[256];
    char namebuffer[256];

    mdns_string_t entryName = mdns_string_extract(data, size, &name_offset, entrybuffer, sizeof(entrybuffer));
    if (rtype != MDNS_RECORDTYPE_TXT) {
        mdns_record_cpp_t newRecord;
        newRecord.name = std::string(MDNS_STRING_ARGS(entryName));
        newRecord.rclass = rclass;
        newRecord.ttl = ttl;

        if (rtype == MDNS_RECORDTYPE_A) {
            struct sockaddr_in addr;
            mdns_record_parse_a(data, size, record_offset, record_length, &addr);
            newRecord.data.a = mdns_record_a_cpp_t(addr);
        } else if (rtype == MDNS_RECORDTYPE_PTR) {
            mdns_string_t ptrTxt = mdns_record_parse_ptr(data, size, record_offset, record_length,
                                                      namebuffer, sizeof(namebuffer));
            mdns_record_ptr_cpp_t ptrRecord = mdns_record_ptr_cpp_t(std::string(MDNS_STRING_ARGS(ptrTxt)));
            newRecord.data.ptr = ptrRecord;
        } else if (rtype == MDNS_RECORDTYPE_AAAA) {
            struct sockaddr_in6 addr;
            mdns_record_parse_aaaa(data, size, record_offset, record_length, &addr);
            mdns_record_aaaa_cpp_t aaaaRecord = mdns_record_aaaa_cpp_t(addr);
            newRecord.data.aaaa = aaaaRecord;
        } else if (rtype == MDNS_RECORDTYPE_SRV) {
            mdns_record_srv_t srv = mdns_record_parse_srv(data, size, record_offset, record_length,
                                                          namebuffer, sizeof(namebuffer));
            mdns_record_srv_cpp_t srvRecord = mdns_record_srv_cpp_t(srv.priority, srv.weight, srv.port, std::string(MDNS_STRING_ARGS(srv.name)));
            newRecord.data.srv = srvRecord;
        } else {
            debug_message("[WRN] Unable to process unknown MDNS record type: Not Supported")
            return -ENOTSUP;
        }
        newRecord.type = static_cast<mdns_record_type>(rtype);
        responses.insert_or_assign(newRecord,std::chrono::steady_clock::now()-std::chrono::microseconds(100*backtick));
        backtick++;
    } else { // TXT records
        static mdns_record_txt_t txtbuffer[128];
        size_t parsed = mdns_record_parse_txt(data, size, record_offset, record_length, txtbuffer,
                                              sizeof(txtbuffer) / sizeof(mdns_record_txt_t));
        for (size_t itxt = 0; itxt < parsed; ++itxt) {
            mdns_record_cpp_t newRecord;
            newRecord.name = std::string(MDNS_STRING_ARGS(entryName));
            newRecord.rclass = rclass;
            newRecord.ttl = ttl;
            mdns_record_txt_cpp_t txtRecord = mdns_record_txt_cpp_t(std::string(MDNS_STRING_ARGS(txtbuffer[itxt].key)), std::string(MDNS_STRING_ARGS(txtbuffer[itxt].value)));
            newRecord.data.txt = txtRecord;
            newRecord.type = (mdns_record_type)rtype;
            responses.insert_or_assign(newRecord, std::chrono::steady_clock::now()-std::chrono::microseconds(100*backtick));
            backtick++;
        }
    }

    return 0;
}

/**
 * Receives and handles MDNS Query Responses and stores them into the records vectors
 * @return Number of records processed into records vector
 */
int mdns::handleMdnsQueryResponses() {
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
                    backtick = 0;
                    records += mdns_query_recv(mdnsSockets[isock], buffer, capacity, queryCallback, user_data, 0);
                }
            }
        }
    } while (res > 0);

    free(buffer);

    // Cleanup used query ids
    std::list<used_query_id_t>::iterator i = usedQueryIds.begin();
    while (i != usedQueryIds.end()) {
        if (i->queryRecieved || (std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - i->querySent).count() > MDNS_REQUEST_TIMEOUT_MS)) {
            i = usedQueryIds.erase(i);
        } else {
            i++;
        }
    }

    return records;
}

/**
 * Cleanup expired responses that have either had their TTLs expire or are older than the MDNS_RECORD_TIMEOUT_MS
 */
void mdns::cleanupExpiredResponses() {
    std::vector<mdns_record_cpp_t> deadRecords;
    for (std::pair<mdns_record_cpp_t, std::chrono::time_point<std::chrono::steady_clock>> response: responses) {
        // Delete records with expired TTLs
        if (response.second < std::chrono::steady_clock::now() - std::chrono::seconds(response.first.ttl)) {
            deadRecords.push_back(response.first);
        }

        // Delete records that exceed our timeout
        if (response.second < std::chrono::steady_clock::now() - std::chrono::milliseconds(MDNS_RECORD_TIMEOUT_MS)) {
            deadRecords.push_back(response.first);
        }
    }
    for (mdns_record_cpp_t deadRecord: deadRecords) {
        responses.erase(deadRecord);
    }
}