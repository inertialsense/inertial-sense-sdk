//
// Created by firiusfoxx on 6/12/25.
//

#ifndef TCPPORTFACTORY_H
#define TCPPORTFACTORY_H
#include "PortFactory.h"
#include "tcpPort.h"

class TCPPortFactory : public PortFactory {
public:
    struct {
        bool defaultBlocking = false;
    } portOptions = {};

    static TCPPortFactory& getInstance() {
        static TCPPortFactory instance;
        return instance;
    }

    TCPPortFactory(TCPPortFactory const &) = delete;
    TCPPortFactory& operator=(TCPPortFactory const&) = delete;

    void locatePorts(std::function<void(PortFactory*, uint16_t, std::string)> portCallback, const std::string& pattern, uint16_t pType) override;

    bool validatePort(const std::string& pName, uint16_t pType = 0) override;

    port_handle_t bindPort(const std::string& pName, uint16_t pType = 0) override;

    bool releasePort(port_handle_t port) override;

private:
    TCPPortFactory() = default;
    ~TCPPortFactory() = default;

    struct URL {
        std::string fullurl;
        std::string protocol;
        std::string address;
        std::string port;
        std::string path;
        std::string params;
        std::string tags;
    };

    static URL parseURL(const std::string& pName);
};

#endif //TCPPORTFACTORY_H
