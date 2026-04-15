/**
 * @file ISmDnsPortFactory.h
 * @brief This is a port factory used to connect a large number of devices over TCP/IP
 *
 * @author FiriusFoxx on 2025-07-03.
 * @copyright Copyright (c) 2025 Inertial Sense, Inc. Licensed under the MIT license
 */

#ifndef IS_SDK__IS_MDNS_PORT_FACTORY_H
#define IS_SDK__IS_MDNS_PORT_FACTORY_H

#define IS_MDNS_PORT_FACTORY_TIME_BETWEEN_QUERIES_MS 200 // How long to wait between sending MDNS queries

#include <chrono>
#include "core/tcpPort.h"
#include "PortFactory.h"

enum MdnsResolveFlags : uint8_t {
    MDNS_RESOLVE_IPV4     = 0x01,   //!< Prefer resolved IPv4 address (e.g. tcp://192.168.1.5:port)
    MDNS_RESOLVE_IPV6     = 0x02,   //!< Prefer resolved IPv6 address (e.g. tcp://[fdc2::1]:port)
    MDNS_RESOLVE_HOSTNAME = 0x04,   //!< Use mDNS hostname (e.g. tcp://hostname.local:port)
    MDNS_RESOLVE_DEFAULT  = MDNS_RESOLVE_IPV4 | MDNS_RESOLVE_IPV6 | MDNS_RESOLVE_HOSTNAME,
};

/**
 * Singleton class passed to PortManager to autodiscover and connect to remote serial ports over the network
 *
 * @code{.cpp} portManager.addPortFactory((PortFactory*)&(ISmDnsPortFactory::getInstance())); @endcode
 * Call to a PortManager adding a ISmDnsPortFactory as an available PortFactory
 */
class ISmDnsPortFactory : public PortFactory {
public:
    struct {
        bool defaultBlocking = false;
        uint8_t resolvePreference = MDNS_RESOLVE_DEFAULT;   //!< Bitmask of MdnsResolveFlags; precedence: IPv4 > IPv6 > hostname
    } portOptions = {};

    static ISmDnsPortFactory& getInstance() {
        static ISmDnsPortFactory instance;
        return instance;
    }

    ISmDnsPortFactory(ISmDnsPortFactory const &) = delete;
    ISmDnsPortFactory& operator=(ISmDnsPortFactory const&) = delete;

    void locatePorts(std::function<void(PortFactory*, uint16_t, std::string)> portCallback, const std::string& pattern, uint16_t pType) override;
    bool validatePort(const std::string& pName, uint16_t pType = 0) override;
    port_handle_t bindPort(const std::string& pName, uint16_t pType = 0) override;
    bool releasePort(port_handle_t port) override;
    static void tick();

    /**
     * Issues a fresh mDNS PTR query and blocks for up to timeoutMs while pumping responses
     * into the shared mDNS cache. After returning, call locatePorts() or discoverPorts()
     * to read the refreshed results. Use this for "refresh" actions in EvalTool/cltool
     * instead of waiting for the next 200ms tick cycle.
     * @param timeoutMs maximum time to wait for responses (default 1000ms)
     */
    static void queryNow(uint32_t timeoutMs = 1000);

    static inline const std::unordered_map<uint16_t, std::string> majorAtlas = {
        {166, "ttyACM"}
    };

private:
    ISmDnsPortFactory() = default;
    ~ISmDnsPortFactory() = default;

    typedef struct {
        uint32_t devid;
        uint16_t port;
    } port_t;

    inline static std::chrono::time_point<std::chrono::steady_clock> lastQueryTime;

    static std::unordered_map<std::string, std::vector<port_t>> getPorts();
    static std::pair<std::string, ISmDnsPortFactory::port_t> parsePortName(const std::string& pName);
    static bool validatePortName(const std::string& pName);
    static std::pair<std::string, ISmDnsPortFactory::port_t> getCanonicalPortData(const std::pair<std::string, ISmDnsPortFactory::port_t>& partialPortPair);
    static std::string getPortURL(const std::pair<std::string, ISmDnsPortFactory::port_t> &port);
};

#endif //IS_SDK__IS_MDNS_PORT_FACTORY_H
