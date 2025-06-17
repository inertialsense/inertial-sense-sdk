//
// Created by firiusfoxx on 6/5/25.
//

#ifndef __IS_TCPPORT_H
#define __IS_TCPPORT_H
#include <netinet/in.h>

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>

#include "ISComm.h"

#ifdef __unix__ // Unix sockets can do TCP connections via files on Unix systems
#include <sys/un.h>
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
        sa_family_t domain; // Type of socket to use
        struct sockaddr generic;
        struct sockaddr_in ipv4;
        struct sockaddr_in6 ipv6;
#ifdef __unix__
        struct sockaddr_un UNIX;
#endif
    } addr;

    bool blocking;
};

typedef struct tcp_port_s tcp_port_t;
#define TCP_PORT(n)  ((tcp_port_t*)n)

void tcpPortInit(port_handle_t port, int id, int type, bool blocking, const char* name, const struct sockaddr* ip);
void tcpPortDelete(port_handle_t port);

#ifdef __cplusplus
}
#endif

#endif //__IS_TCPPORT_H
