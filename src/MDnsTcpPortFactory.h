//
// Created by firiusfoxx on 7/3/25.
//

#ifndef IS_SDK__MDNS_TCP_PORT_FACTORY_H
#define IS_SDK__MDNS_TCP_PORT_FACTORY_H

#include <map>
#include "PortFactory.h"
#include "mdns/mdns.h"
#include "core/tcpPort.h"

class MDnsTcpPortFactory : public PortFactory {
public:
    struct {
        bool defaultBlocking = false;
    } portOptions = {};

    static MDnsTcpPortFactory& getInstance() {
        static MDnsTcpPortFactory instance;
        return instance;
    }

    MDnsTcpPortFactory(MDnsTcpPortFactory const &) = delete;
    MDnsTcpPortFactory& operator=(MDnsTcpPortFactory const&) = delete;

    void locatePorts(std::function<void(PortFactory*, uint16_t, std::string)> portCallback, const std::string& pattern, uint16_t pType) override;

    bool validatePort(const std::string& pName, uint16_t pType = 0) override;

    port_handle_t bindPort(const std::string& pName, uint16_t pType = 0) override;

    bool releasePort(port_handle_t port) override;

private:
    MDnsTcpPortFactory() = default;
    ~MDnsTcpPortFactory() = default;

    typedef struct {
        uint32_t devid;
        uint16_t port;
    } port_t;

    inline static int mdnsSockets[32];
    inline static uint32_t lastQueryTime;
    inline static int socketsOpened;
    inline static std::map<std::string, std::vector<std::vector<port_t>>> port_records;
    inline static std::map<std::string, sockaddr_storage> host_records;

    static int create_mdns_sockets();
    static void send_mdns_query(mdns_record_type_t type, std::string query);
    static int query_callback(int sock, const struct sockaddr* from, size_t addrlen, mdns_entry_type_t entry, uint16_t query_id,
                       uint16_t rtype, uint16_t rclass, uint32_t ttl, const void* data, size_t size, size_t name_offset,
                       size_t name_length, size_t record_offset, size_t record_length, void* user_data);
    static int handle_mdns_query_responses();
};

#endif //IS_SDK__MDNS_TCP_PORT_FACTORY_H
