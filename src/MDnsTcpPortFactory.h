//
// Created by firiusfoxx on 7/3/25.
//

#ifndef IS_SDK__MDNS_TCP_PORT_FACTORY_H
#define IS_SDK__MDNS_TCP_PORT_FACTORY_H

#include "PortFactory.h"
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
    static void tick();

private:
    MDnsTcpPortFactory() = default;
    ~MDnsTcpPortFactory() = default;

    typedef struct {
        uint32_t devid;
        uint16_t port;
    } port_t;

    inline static std::chrono::time_point<std::chrono::steady_clock> lastQueryTime;
};

#endif //IS_SDK__MDNS_TCP_PORT_FACTORY_H
