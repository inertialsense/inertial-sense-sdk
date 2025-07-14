//
// Created by firiusfoxx on 6/12/25.
//

#ifndef IS_SDK__TCP_PORT_FACTORY_H
#define IS_SDK__TCP_PORT_FACTORY_H
#include "PortFactory.h"
#include "core/tcpPort.h"

class TcpPortFactory : public PortFactory {
public:
    struct {
        bool defaultBlocking = false;
    } portOptions = {};

    static TcpPortFactory& getInstance() {
        static TcpPortFactory instance;
        return instance;
    }

    TcpPortFactory(TcpPortFactory const &) = delete;
    TcpPortFactory& operator=(TcpPortFactory const&) = delete;

    void locatePorts(std::function<void(PortFactory*, uint16_t, std::string)> portCallback, const std::string& pattern, uint16_t pType) override;

    bool validatePort(const std::string& pName, uint16_t pType = 0) override;

    port_handle_t bindPort(const std::string& pName, uint16_t pType = 0) override;

    bool releasePort(port_handle_t port) override;

private:
    TcpPortFactory() = default;
    ~TcpPortFactory() = default;
};

#endif //IS_SDK__TCP_PORT_FACTORY_H
