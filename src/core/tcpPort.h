//
// Created by firiusfoxx on 6/5/25.
//

#ifndef __IS_TCPPORT_H
#define __IS_TCPPORT_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>

#include "ISComm.h"
#include "ISConstants.h"

#ifdef PLATFORM_IS_WINDOWS
#include <winsock2.h>
#else
#include <netinet/in.h>
#ifdef __unix__ // Unix sockets can do TCP connections via files on Unix systems
#include <sys/un.h>
#endif
#endif

#define MAX_TCP_PORT_NAME_LENGTH 63

// Allows communicating over a TCP Socket
struct tcp_port_s
{
    // base "implementation"
    union {
        base_port_t base;
        comm_port_t comm;
    };

    port_monitor_set_t stats;

    rmci_t rmci;
    uint8_t rmciUPMcnt[DID_COUNT];
    uint8_t rmciNMEAcnt[NMEA_MSG_ID_COUNT];

    // the port name (do not modify directly)
    char* name;

    // Actual socket
    int socket;

    // Store an Address type that can connect via TCP
    union {
#ifdef PLATFORM_IS_WINDOWS
        ADDRESS_FAMILY domain;
#else
        sa_family_t domain; // Type of socket to use
#endif
        struct sockaddr generic;
        struct sockaddr_in ipv4;
        struct sockaddr_in6 ipv6;
#ifdef __unix__
        struct sockaddr_un UNIX;
#endif
        struct sockaddr_storage storage;
    } addr;

    bool blocking;
    bool blocking_internal;
};

typedef struct tcp_port_s tcp_port_t;
#define TCP_PORT(n)  ((tcp_port_t*)n)

void tcpPortInit(port_handle_t port, int id, const char* name, const struct sockaddr_storage* ip, int flags);

/**
 * Initializes a tcp port with the specified name and socket
 * @param port The port handle to initialize
 * @param id The id of the new port handle (a unique id)
 * @param type The type modifier (OR'd with PORT_TYPE__TCP | PORT_FLAG__VALID)
 * @param name The name to be associated with the new port
 * @param socket a socket handle describing an existing and valid tcp socket
 * @param flags port-specific bit-flags to associated with this port (defaults to PORT_FLAG__COMM)
 */
void tcpPortInitWithSocket(port_handle_t port, int id, int type, const char* name, const int socket, int flags);

void tcpPortDelete(port_handle_t port);
int tcpPortSetBlocking(port_handle_t port, bool blocking);

#ifdef __cplusplus
}
#endif

#endif //__IS_TCPPORT_H
