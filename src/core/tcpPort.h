/**
 * @file tcpPort.h
 * @brief TCP-socket implementation of the base_port_t/comm_port_t interface (tcp_port_t), plus
 *        the init/delete/blocking-mode entry points used to create and manage it.
 *
 * @author firiusfoxx on 6/5/25.
 * @copyright Copyright (c) 2025 Inertial Sense, Inc. All rights reserved.
 */

#ifndef __IS_TCPPORT_H
#define __IS_TCPPORT_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>

#include "ISComm.h"
#include "ISConstants.h"

#if PLATFORM_IS_WINDOWS
    #include <winsock2.h>
#elif !PLATFORM_IS_EMBEDDED
    #include <netinet/in.h>
    #ifdef __unix__ // Unix sockets can do TCP connections via files on Unix systems
    #include <sys/un.h>
    #endif
#endif

#define MAX_TCP_PORT_NAME_LENGTH 63     //!< maximum length (excluding the null terminator) of a tcp_port_t name

/**
 * Concrete base_port_t/comm_port_t implementation backed by a TCP socket. Embeds base_port_t (or
 * comm_port_t, when the PORT_TYPE__COMM bit is set) as its first member so a tcp_port_t* can be
 * reinterpreted and driven through the generic port helpers in base_port.h.
 */
struct tcp_port_s
{
    // base "implementation"
    union {
        base_port_t base;      //!< base_port_t view of this port
        comm_port_t comm;      //!< comm_port_t view of this port, valid when PORT_TYPE__COMM is set
    };

    port_monitor_set_t stats;  //!< traffic/error counters for this port, referenced by base.stats

    // the port name (do not modify directly)
    char name[MAX_TCP_PORT_NAME_LENGTH + 1];   //!< display name for this port, set via tcpPortInit()/tcpPortInitWithSocket()

    // Actual socket
    int socket;                 //!< native socket descriptor, or a negative -errno value if not open/invalid

    // Store an Address type that can connect via TCP
    union {
#if PLATFORM_IS_WINDOWS
        ADDRESS_FAMILY domain;          //!< address family of the socket (Windows)
#elif !PLATFORM_IS_EMBEDDED
        sa_family_t domain;             //!< address family of the socket (Linux/POSIX), e.g. AF_INET/AF_INET6/AF_UNIX
#endif
        struct sockaddr generic;        //!< generic view of the peer/bind address
        struct sockaddr_in ipv4;        //!< IPv4 view of the peer/bind address
        struct sockaddr_in6 ipv6;       //!< IPv6 view of the peer/bind address
#ifdef __unix__
        struct sockaddr_un UNIX;        //!< Unix-domain-socket-file view of the peer/bind address
#endif
        struct sockaddr_storage storage; //!< generic storage, large enough for any supported address family
    } addr;                              //!< remote (or, for a Unix socket file, local) address this port connects to

    bool blocking;           //!< caller-requested blocking mode for read/write operations
    bool blocking_internal;  //!< actual current blocking mode of the underlying socket (may transiently differ from blocking)
};

typedef struct tcp_port_s tcp_port_t;   //!< see struct tcp_port_s
#define TCP_PORT(n)  ((tcp_port_t*)n)    //!< reinterprets a port_handle_t (or any compatible pointer) as a pointer to tcp_port_t

/**
 * Initializes a new tcp port bound to the given remote address, without opening it. Call
 * portOpen() (or portOpenRetry()) afterward to actually connect.
 * @param port  the port handle to initialize
 * @param id    the id of the new port handle (a unique id)
 * @param name  the name to be associated with the new port
 * @param ip    the remote address (and port) to connect to over TCP
 * @param flags port-specific bit-flags to associate with this port
 */
void tcpPortInit(port_handle_t port, int id, const char* name, const struct sockaddr_storage* ip, int flags);

/**
 * Initializes a tcp port with the specified name and an existing, already-connected socket.
 * @param port The port handle to initialize
 * @param id The id of the new port handle (a unique id)
 * @param type The type modifier (OR'd with PORT_TYPE__TCP | PORT_FLAG__VALID)
 * @param name The name to be associated with the new port
 * @param socket a socket handle describing an existing and valid tcp socket
 * @param flags port-specific bit-flags to associate with this port
 *
 * Note that initializing a TCP Port with a valid socket implies that the underlying socket is
 * already connected. As a result, the PORT_FLAG__OPENED will also be set, and there is no need
 * to call portOpen()
 */
void tcpPortInitWithSocket(port_handle_t port, int id, int type, const char* name, const int socket, int flags);

/**
 * Deinitializes a tcp port, closing it if still open, and zeros its underlying memory.
 * @param port The port handle to deinitialize
 */
void tcpPortDelete(port_handle_t port);

/**
 * Sets the blocking mode used for socket I/O on this port.
 * @param port     the tcp port to configure
 * @param blocking true to use blocking I/O, false for non-blocking
 * @return 0 on success, or a negative ioctl() error code on failure
 */
int tcpPortSetBlocking(port_handle_t port, bool blocking);

#ifdef __cplusplus
}
#endif

#endif //__IS_TCPPORT_H
