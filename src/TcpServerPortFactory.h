/**
 * @file TcpServerPortFactory.h
 * @brief This is a TcpPortFactory which binds a socket listener to a specified server port, and creates new tcpPorts for each
 *  incoming connection request on that port.
 *
 * @author Kyle Mallory on 10/17/25.
 * @copyright Copyright (c) 2025 Inertial Sense, Inc. All rights reserved.
 */

#ifndef IS_SDK_TCP_SERVER_PORT_FACTORY_H
#define IS_SDK_TCP_SERVER_PORT_FACTORY_H

#include <csignal>
#include <cerrno>
#include <iostream>
#include <set>
#include <utility>

#include "ISConstants.h"

#if PLATFORM_IS_WINDOWS
    #include <winsock2.h>
    #include <ws2tcpip.h>
    #pragma comment(lib, "ws2_32.lib") // Link with ws2_32.lib
#elif !PLATFORM_IS_EMBEDDED
    #include <unistd.h>
    #include <fcntl.h>
    #include <netdb.h>

    #include <arpa/inet.h>
    #include <sys/socket.h> // For AF_INET
#endif

#include "core/msg_logger.h"
#include "core/tcpPort.h"
#include "PortFactory.h"

/**
 * Context codes identifying which step of TcpServerPortFactory::startListening() failed; returned as the
 * first element of getLastListenError().  This is intentionally separate from the PORT_ERROR__* / base_port_t
 * framework, which applies to accepted client connections rather than the listening socket itself.
 */
enum eTcpListenErrorContext {
    TCP_LISTEN_CTX__NONE      = 0,      //!< no listener-setup failure recorded (listener is up)
    TCP_LISTEN_CTX__SOCKET    = 1,      //!< socket() failed
    TCP_LISTEN_CTX__GETFLAGS  = 2,      //!< fcntl(F_GETFL) failed
    TCP_LISTEN_CTX__NONBLOCK  = 3,      //!< fcntl(F_SETFL, O_NONBLOCK) / ioctlsocket(FIONBIO) failed
    TCP_LISTEN_CTX__BIND      = 4,      //!< bind() failed
    TCP_LISTEN_CTX__LISTEN    = 5,      //!< listen() failed
};

//!< Human-readable names for eTcpListenErrorContext, indexed by enum value
[[maybe_unused]] inline static const char* tcp_listen_error_context_names[] = { "none", "socket()", "fcntl(F_GETFL)", "fcntl/ioctlsocket(non-blocking)", "bind()", "listen()" };

/**
 * Unlike other PortFactories, TcpServerPortFactory is NOT a singleton - since there may be multiple instances which listen an unique ports, etc.
 * By this same logic, it may make sense that no PortFactory should be a singleton; but this is definitely the first case that warrants it
 */
class TcpServerPortFactory : public PortFactory {
public:
    /** Configuration for this factory's listener, set by the constructor. */
    struct {
        uint16_t listenerPort = 4321;       //!< the tcp port to listen for incoming connections on
        struct sockaddr_in listeningAddr;   //!< listening address (this binds to a specific interface, defaults to 127.0.0.1)
        bool backgroundListener = false;    //!< if true, we'll setup a thread to process incoming connections -- note that this doesn't service those connections, just the listener
        int maxConnections = 10;            //!< the maximum number of connections that can be kept open - additional connection requests will be rejected
        bool portDefaultBlocking = false;   //!< if true, created tcpPorts will be configured for blocking by default (usually, we don't want that).
    } factoryOptions = {};   //!< configuration for this factory's listener, set by the constructor

    /**
     * Constructs and configures a listener; does not start listening (call startListening() or
     * drive it via locatePorts()). Performs one-time platform setup (ignores SIGPIPE on Linux,
     * calls WSAStartup on Windows).
     * @param listenPort the tcp port to listen for incoming connections on
     * @param listenAddr the local interface address to bind to; accepts "<address>" or "<address> (<name>)", defaults to 127.0.0.1
     * @param maxConnections the maximum number of connections that can be kept open
     * @param portDefaultBlocking if true, created tcpPorts will be configured for blocking by default
     * @param backgroundListener if true, a thread will be set up to process incoming connections (the listener only, not the connections themselves)
     */
    explicit TcpServerPortFactory(uint16_t listenPort = 4321, const std::string& listenAddr = "127.0.0.1", int maxConnections = 10, bool portDefaultBlocking = false, bool backgroundListener = false) {
#ifdef PLATFORM_IS_LINUX
        signal(SIGPIPE, SIG_IGN); // ignore broken pipes
#endif
#ifdef PLATFORM_IS_WINDOWS
        WSADATA wsa_data;
        int wsa_result = WSAStartup(MAKEWORD(2, 2), &wsa_data);
        if (wsa_result != 0) {
            log_error(IS_LOG_PORT_FACTORY, "TcpServerPortFactory: WSAStartup failed with error code %d", wsa_result);
            // Optionally, you could throw or set a flag here to prevent further use
        }
#endif

        configure(listenPort, listenAddr, maxConnections, portDefaultBlocking, backgroundListener);
    };

    /**
     * NOTE that TcpServerPortFactory is an outlier in PortFactory, because it retains knowledge of AT LEAST its own listening port
     * But generally it also knows about all client sockets which are connected to it.  If the factory is destroyed, to be good stewards
     * of the heap, we should clean up and destroy all the associated ports.
     */
    ~TcpServerPortFactory() {
        stopListening();
        shutdownAllClients();
        for (auto& se : knownSockets) {
            releasePort(se.port);
        }
        knownSockets.clear();
#ifdef PLATFORM_IS_WINDOWS
        WSACleanup();
#endif
    };

    // TcpServerPortFactory(TcpServerPortFactory const &) = delete;
    // TcpServerPortFactory& operator=(TcpServerPortFactory const&) = delete;

    /**
     * Services pending incoming connections on the listening socket (starting it if not already
     * started) and invokes @p portCallback for each accepted connection whose auto-generated
     * "tcp://ip:port" name validates.
     * @param portCallback the function to call back into to indicate that this port has been "found"
     * @param pattern the URL to validate and "discover"
     * @param pType ignored
     */
    void locatePorts(std::function<void(PortFactory*, uint16_t, std::string)> portCallback, const std::string& pattern, uint16_t pType) override;

    /**
     * Validates that @p pName is a well-formed "tcp://host:port" URL whose host resolves.
     * @param pName the URL to validate, starting with tcp://
     * @param pType must be PORT_TYPE__TCP
     * @return true if a port can be created from pName, otherwise false
     */
    bool validatePort(const std::string& pName, uint16_t pType) override;

    /**
     * Parses and creates a new port_handle_t representing a TCP port for a URL in the format
     * tcp://ipAddr:port.
     * @param pName the URL and name of the new port to bind a port_handle_t to
     * @param pType the port type requested to be generated
     * @return a port_handle_t bound to the newly created TCP port for the connection pName represents, or nullptr on failure
     */
    port_handle_t bindPort(const std::string& pName, uint16_t pType) override;

    /**
     * Releases and frees the memory used by this port.
     * @param port the TCP port handle to deinitialize
     * @return true if successful, false otherwise
     */
    bool releasePort(port_handle_t port) override;

    /** Shuts down (SHUT_RDWR) and closes every currently-accepted client socket, without removing them from knownSockets. */
    void shutdownAllClients();

    /**
     * Forensic accessor for listener-setup failures.  Intentionally separate from the PORT_ERROR__* /
     * base_port_t framework (which tracks accepted client connections); this reports only on the
     * listening socket itself.
     * @return a {context, error} pair for the most recent startListening() failure: .first is the
     *  eTcpListenErrorContext step that failed, .second is the platform socket error (errno on POSIX,
     *  WSAGetLastError() on Windows).  Returns {TCP_LISTEN_CTX__NONE, 0} when the listener was last
     *  set up successfully.
     */
    std::pair<eTcpListenErrorContext, int> getLastListenError() const { return lastListenError; }

protected:
    /** A single accepted client connection: its raw socket, auto-generated "tcp://ip:port" name, and (once bound) its port handle. */
    struct socket_entry_t {
        int socket = 0;                        //!< the accepted client's raw socket descriptor
        std::string portName;                  //!< auto-generated "tcp://ip:port" name for this connection
        mutable port_handle_t port = nullptr;   //!< the bound port handle for this connection, once created

        /**
         * @param _s the accepted client's raw socket descriptor
         * @param _n auto-generated "tcp://ip:port" name for this connection
         * @param _p the bound port handle for this connection, if already created
         */
        socket_entry_t(int _s, std::string _n, port_handle_t _p = nullptr) : socket(_s), portName(std::move(_n)), port(_p) {};

        /**
         * Strict weak ordering by socket, falling back to portName so entries with the same
         * (reused) socket value on different names remain distinguishable in the containing std::set.
         * @param other the socket_entry_t to compare against
         * @return true if this entry sorts before @p other
         */
        bool operator<(const socket_entry_t& other) const {
            return (socket != other.socket) ? socket < other.socket : portName < other.portName; // Secondary sorting criterion
        }

    };

    /**
     * Applies the listener configuration used by startListening(); does not itself open a socket.
     * @param listenPort the tcp port to listen for incoming connections on
     * @param listenAddr the local interface address to bind to; accepts "<address>" or "<address> (<name>)", defaults to 127.0.0.1
     * @param maxConnections the maximum number of connections that can be kept open
     * @param portDefaultBlocking if true, created tcpPorts will be configured for blocking by default
     * @param backgroundListener if true, a thread will be set up to process incoming connections (the listener only, not the connections themselves)
     */
    void configure(uint16_t listenPort = 4321, const std::string& listenAddr = "127.0.0.1", int maxConnections = 10, bool portDefaultBlocking = false, bool backgroundListener = false) {
        factoryOptions.listenerPort = listenPort;
        factoryOptions.maxConnections = maxConnections;
        factoryOptions.portDefaultBlocking = portDefaultBlocking;
        factoryOptions.backgroundListener = backgroundListener;
        factoryOptions.listeningAddr.sin_family = AF_INET;
        factoryOptions.listeningAddr.sin_port = htons(listenPort);

        // we expect either a string "<address>" or "<address> (<name>)" - in either case, we just want the <address> part (upto the space)
        std::string ipAddr = listenAddr.substr(0, listenAddr.find_first_of(' '));

        struct in_addr addr = {};
        if (inet_pton(AF_INET, ipAddr.c_str(), &addr) <= 0) {
            // Handle error: invalid address or address not supported
            factoryOptions.listeningAddr.sin_addr.s_addr = INADDR_NONE; // A common error indicator for in_addr_t
        }
        factoryOptions.listeningAddr.sin_addr = addr;
    }

    /**
     * Creates, configures (non-blocking, SO_REUSEADDR), binds, and listens on the socket described
     * by factoryOptions. Idempotent — if a listener is already open, returns true immediately without
     * creating a second socket. On failure, records the failing step and platform error via
     * recordListenError() and tears the half-open socket back down (see getLastListenError()).
     * @return true if a listener is open (already was, or was just created), false on failure
     */
    bool startListening();

    /** Closes the listening socket (if open) and resets it to the not-listening state. */
    void stopListening();

    /** @return the number of currently-accepted client connections. */
    int getClientConnectionCount() {
        return (int)knownSockets.size();
    }

    /** @return a snapshot copy of all currently-accepted client connections. */
    std::vector<socket_entry_t> getClientSockets() {
        std::vector<socket_entry_t> out;
        for (const auto& ks : knownSockets)
            out.emplace_back(ks);
        return out;
    }

    /**
     * The primary service routine - this should be called periodically (and frequently) to service incoming connections.
     * If this is not called, no ports will ever be discovered/created
     * @param cb callback invoked once per newly-accepted connection, with the socket_entry_t describing it
     * @return true if at least one connection was accepted during this call; false if none were (including when no listener is active)
     */
    bool processPendingConnections(std::function<void(const socket_entry_t&)> cb);

private:
    /**
     * Captures the platform socket error (errno / WSAGetLastError()) together with the supplied
     * failure context into lastListenError, and logs it.  Called by startListening() before it tears
     * the half-open socket back down, so the forensic detail survives the cleanup.
     * @param context the eTcpListenErrorContext step that failed
     */
    void recordListenError(eTcpListenErrorContext context);

    std::set<socket_entry_t> knownSockets;
    std::set<socket_entry_t> prevSockets;

    int listen_fd = 0; /* listener socket */
    std::pair<eTcpListenErrorContext, int> lastListenError { TCP_LISTEN_CTX__NONE, 0 };   //!< {context, error} of the last startListening() failure; {NONE, 0} when the listener is up
};


#endif //IS_SDK_TCP_SERVER_PORT_FACTORY_H
