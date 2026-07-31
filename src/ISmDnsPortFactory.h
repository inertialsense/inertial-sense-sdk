/**
 * @file ISmDnsPortFactory.h
 * @brief This is a port factory used to connect a large number of devices over TCP/IP
 *
 * @author FiriusFoxx on 2025-07-03.
 * @copyright Copyright (c) 2025 Inertial Sense, Inc. Licensed under the MIT license
 */

#ifndef IS_SDK__IS_MDNS_PORT_FACTORY_H
#define IS_SDK__IS_MDNS_PORT_FACTORY_H

#define IS_MDNS_PORT_FACTORY_TIME_BETWEEN_QUERIES_MS 200   //!< How long to wait between sending MDNS queries, in milliseconds

#include <chrono>
#include "core/tcpPort.h"
#include "PortFactory.h"

/** Bitmask flags controlling which resolved address form(s) locatePorts() will emit for a discovered device. */
enum MdnsResolveFlags : uint8_t {
    MDNS_RESOLVE_IPV4     = 0x01,   //!< Prefer resolved IPv4 address (e.g. tcp://192.168.1.5:port)
    MDNS_RESOLVE_IPV6     = 0x02,   //!< Prefer resolved IPv6 address (e.g. tcp://[fdc2::1]:port)
    MDNS_RESOLVE_HOSTNAME = 0x04,   //!< Use mDNS hostname (e.g. tcp://hostname.local:port)
    MDNS_RESOLVE_DEFAULT  = MDNS_RESOLVE_IPV4 | MDNS_RESOLVE_IPV6 | MDNS_RESOLVE_HOSTNAME,  //!< try every resolved form, in IPv4 > IPv6 > hostname precedence
};

/**
 * Singleton class passed to PortManager to autodiscover and connect to remote serial ports over the network
 *
 * @code{.cpp} portManager.addPortFactory((PortFactory*)&(ISmDnsPortFactory::getInstance())); @endcode
 * Call to a PortManager adding a ISmDnsPortFactory as an available PortFactory
 */
class ISmDnsPortFactory : public PortFactory {
public:
    /** Default options applied to ports created by this factory. */
    struct {
        bool defaultBlocking = false;                        //!< default blocking mode applied to ports bound by this factory
        uint8_t resolvePreference = MDNS_RESOLVE_DEFAULT;   //!< Bitmask of MdnsResolveFlags; precedence: IPv4 > IPv6 > hostname
    } portOptions = {};   //!< default options applied to ports created by this factory

    /** @return the process-wide singleton ISmDnsPortFactory instance. */
    static ISmDnsPortFactory& getInstance() {
        static ISmDnsPortFactory instance;
        return instance;
    }

    ISmDnsPortFactory(ISmDnsPortFactory const &) = delete;
    ISmDnsPortFactory& operator=(ISmDnsPortFactory const&) = delete;

    /**
     * Refreshes the mDNS cache (via tick()) and emits a "tcp://host:port" URL, via @p portCallback,
     * for every advertised device port whose mDNS URL or resolved TCP URL matches the @p pattern regex.
     * Deduplicates devices advertised on multiple interfaces before emitting.
     * @param portCallback function to call for each matching port (TcpPortFactory instance, port-type, tcp:// URL)
     * @param pattern regex matched against each candidate's mDNS URL and resolved tcp:// URL
     * @param pType OR'd into the port-type flags passed to portCallback
     */
    void locatePorts(std::function<void(PortFactory*, uint16_t, std::string)> portCallback, const std::string& pattern, uint16_t pType) override;

    /**
     * Validates that @p pName is a well-formed "is-mdns://" port name that resolves to a currently
     * known device/port in the mDNS cache (refreshed via tick() first).
     * @param pName the URL to validate, starting with is-mdns://
     * @param pType must include PORT_TYPE__TCP (PORT_TYPE__COMM is also expected but not enforced)
     * @return true if a port can be created from pName, otherwise false
     */
    bool validatePort(const std::string& pName, uint16_t pType = 0) override;

    /**
     * Parses and creates a new port_handle_t representing a TCP port for a URL in the format
     * is-mdns://hostname/port, resolved to a concrete address via the mDNS cache.
     * @param pName the URL and name of the new port to bind a port_handle_t to
     * @param pType the port type requested to be generated
     * @return a port_handle_t bound to the newly created TCP port for the connection pName represents, or nullptr on failure
     */
    port_handle_t bindPort(const std::string& pName, uint16_t pType = 0) override;

    /**
     * Releases and frees the memory used by this port.
     * @param port the TCP port handle to deinitialize
     * @return true if successful, false otherwise
     */
    bool releasePort(port_handle_t port) override;

    /** Sends a periodic mDNS PTR query (rate-limited to IS_MDNS_PORT_FACTORY_TIME_BETWEEN_QUERIES_MS) and pumps mdns::tick() to process responses. */
    static void tick();

    /**
     * Issues a fresh mDNS PTR query and blocks for up to timeoutMs while pumping responses
     * into the shared mDNS cache. After returning, call locatePorts() or discoverPorts()
     * to read the refreshed results. Use this for "refresh" actions in EvalTool/cltool
     * instead of waiting for the next 200ms tick cycle.
     * @param timeoutMs maximum time to wait for responses (default 1000ms)
     */
    static void queryNow(uint32_t timeoutMs = 1000);

    /** Maps a Linux tty major device number to its driver name prefix, for devices whose advertised hostname encodes major:minor rather than a friendly name. */
    static inline const std::unordered_map<uint16_t, std::string> majorAtlas = {
        {166, "ttyACM"}
    };

private:
    ISmDnsPortFactory() = default;
    ~ISmDnsPortFactory() = default;

    typedef struct {
        uint32_t devid;   //!< numeric device id encoded in the advertised mDNS hostname
        uint16_t port;    //!< TCP port advertised for this device
    } port_t;

    inline static std::chrono::time_point<std::chrono::steady_clock> lastQueryTime;

    static std::unordered_map<std::string, std::vector<port_t>> getPorts();
    static std::pair<std::string, ISmDnsPortFactory::port_t> parsePortName(const std::string& pName);
    static bool validatePortName(const std::string& pName);
    static std::pair<std::string, ISmDnsPortFactory::port_t> getCanonicalPortData(const std::pair<std::string, ISmDnsPortFactory::port_t>& partialPortPair);
    static std::string getPortURL(const std::pair<std::string, ISmDnsPortFactory::port_t> &port);
};

#endif //IS_SDK__IS_MDNS_PORT_FACTORY_H
