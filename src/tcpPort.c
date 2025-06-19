//
// Created by firiusfoxx on 6/5/25.
//

#include "tcpPort.h"
#include <errno.h>
#include <stdbool.h>

#ifdef PLATFORM_IS_WINDOWS
#include <winsock2.h>
#define errno WSAGetLastError()
#define close closesocket
#define ioctl ioctlsocket
#else
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/ioctl.h>
#include <sys/time.h>
#include <sys/socket.h>
#endif

const char* tcpPortGetName(port_handle_t port) {
    tcp_port_t* tcpPort = TCP_PORT(port);
    return tcpPort->name;
}

int tcpPortValidate(port_handle_t port) {
    tcp_port_t* tcpPort = TCP_PORT(port);
    if (tcpPort->socket < 0) { // Socket is currently closed, so let's create a new socket and see if it errors
        int sock = socket(tcpPort->addr.domain, SOCK_STREAM, IPPROTO_TCP);
        if (sock < 0) { // Can't create socket FD port can't be valid
            tcpPort->base.perror = errno; // Store errno somewhere where clients can read it
            return -errno; // Return error code to calling function
        }
        close(sock); // Close socket if we didn't error
    }
    return 0; // Creating the stream socket is likely the most validation we are getting without connecting
    // The socket is allocated, and we can attempt to connect to the server
    // although validating the server's existence is impossible without connecting or opening the socket
}

int tcpPortOpen(port_handle_t port) {
    tcp_port_t* tcpPort = TCP_PORT(port);
    if (tcpPort->socket >= 0) { // The socket is already open, we don't need to do anything
        return -EISCONN;
    }
    // Create new socket
    tcpPort->socket = socket(tcpPort->addr.domain, SOCK_STREAM, IPPROTO_TCP);
    if (tcpPort->socket < 0) {
        tcpPort->socket = -errno; // File descriptors should always be positive so we can store the errno here
        tcpPort->base.perror = errno; // Store errno somewhere where clients can read it
        return tcpPort->socket; // Return error code to calling function
    }
    // Connect socket to remote
    int retval = connect(tcpPort->socket, &tcpPort->addr.generic, sizeof(tcpPort->addr.generic));
    if (retval != 0) {
        tcpPort->base.perror = errno;
        return -errno;
    }
    tcpPort->base.ptype |= PORT_FLAG__OPENED;
    return 0;
}

int tcpPortClose(port_handle_t port) {
    tcp_port_t* tcpPort = TCP_PORT(port);
    if (tcpPort->socket < 0) { // The file descriptor is invalid, creating it errored, or we already closed it.
        tcpPort->base.perror = -(tcpPort->socket);
        return tcpPort->socket;
    }
    int retval = close(tcpPort->socket);
    if (retval != 0) {
        tcpPort->base.perror = errno;
        // close will permit the kernel to reuse the file descriptor immediately even if there is an error.
    }
    tcpPort->socket = -EBADF; // Mark the File Descriptor as closed.
    tcpPort->base.ptype &= ~PORT_FLAG__OPENED;
    return 0;
}

int tcpPortFree(port_handle_t port) {
    tcp_port_t* tcpPort = TCP_PORT(port);
    if (tcpPort->socket < 0) { // The file descriptor is invalid, creating it errored, or we already closed it.
        tcpPort->base.perror = -(tcpPort->socket);
        return tcpPort->socket;
    }

    int bufferSize = 0;
    socklen_t bufferSizeLen = sizeof(bufferSize); // in/out parameter
    if (getsockopt(tcpPort->socket, SOL_SOCKET, SO_SNDBUF, &bufferSize, &bufferSizeLen) < 0) {
        tcpPort->base.perror = errno;
        return -errno;
    }

    int bytesUsed;
    if (ioctl(tcpPort->socket,TIOCOUTQ, &bytesUsed) < 0) {
        tcpPort->base.perror = errno;
        return -errno;
    }
    return bufferSize - bytesUsed;
}

int tcpPortAvailable(port_handle_t port) {
    tcp_port_t* tcpPort = TCP_PORT(port);
    if (tcpPort->socket < 0) { // The file descriptor is invalid, creating it errored, or we already closed it.
        tcpPort->base.perror = -(tcpPort->socket);
        return tcpPort->socket;
    }

    int bytesAvailable;
    if (ioctl(tcpPort->socket, FIONREAD, &bytesAvailable) < 0) {
        tcpPort->base.perror = errno;
        return -errno;
    }
    return bytesAvailable;
}

int tcpPortFlush(port_handle_t port) {
    tcp_port_t* tcpPort = TCP_PORT(port);
    if (tcpPort->socket < 0) { // The file descriptor is invalid, creating it errored, or we already closed it.
        tcpPort->base.perror = -(tcpPort->socket);
        return tcpPort->socket;
    }

    while (true) {
        int bufferSize = tcpPortAvailable(port);
        if (bufferSize > 0) {
            void *bigBuffer = malloc(bufferSize);
            if (bigBuffer == NULL) {
                tcpPort->base.perror = errno;
                return -errno;
            }
            memset(bigBuffer, 0, bufferSize);
            ssize_t retval = recv(tcpPort->socket, bigBuffer, bufferSize, MSG_DONTWAIT);
            free(bigBuffer); // Free doesn't modify errno
            bigBuffer = NULL;
            if (retval < 0) {
                tcpPort->base.perror = errno;
                return -errno;
            } else if (retval == 0) {
                return 0;
            }
        } else if (bufferSize == 0) {
            return 0;
        } else {
            tcpPort->base.perror = -bufferSize;
            return bufferSize;
        }
    }
}

int tcpPortDrain(port_handle_t port, __attribute__((unused)) uint32_t timeout) {
    tcp_port_t* tcpPort = TCP_PORT(port);
    if (tcpPort->socket < 0) { // The file descriptor is invalid, creating it errored, or we already closed it.
        tcpPort->base.perror = -(tcpPort->socket);
        return tcpPort->socket;
    }

    // Tell the kernel to send everything
    int flag = 1;
    int retval = setsockopt(tcpPort->socket, IPPROTO_TCP, TCP_NODELAY, &flag, sizeof(int));
    if (retval != 0) {
        tcpPort->base.perror = errno;
        return -errno;
    }

    // Makes sures the kernel actually tries to send something
    retval = send(tcpPort->socket, NULL, 0, MSG_DONTWAIT);
    if (retval < 0) {
        tcpPort->base.perror = errno;
        return -errno;
    }

    // Tell the kernel to buffer messages again cause TCP_NODELAY kills network performance
    flag = 0;
    retval = setsockopt(tcpPort->socket, IPPROTO_TCP, TCP_NODELAY, &flag, sizeof(int));
    if (retval != 0) {
        tcpPort->base.perror = errno;
        return -errno;
    }
    return 0;
}

int tcpPortRead(port_handle_t port, uint8_t* buf, unsigned int len) {
    tcp_port_t* tcpPort = TCP_PORT(port);
    if (tcpPort->socket < 0) { // The file descriptor is invalid, creating it errored, or we already closed it.
        tcpPort->base.perror = -(tcpPort->socket);
        return tcpPort->socket;
    }

    // Determine flags
    int flags = 0;
    if (!tcpPort->blocking) {
        flags |= MSG_DONTWAIT;
    }

    // Receive data from the socket
    ssize_t retval = recv(tcpPort->socket, buf, len, flags);
    if (retval < 0) {
        tcpPort->base.perror = errno;
        return -errno;
    }
    return retval;
}

int tcpPortReadTimeout(port_handle_t port, uint8_t* buf, unsigned int len, uint32_t timeout) {
    tcp_port_t* tcpPort = TCP_PORT(port);
    if (tcpPort->socket < 0) { // The file descriptor is invalid, creating it errored, or we already closed it.
        tcpPort->base.perror = -(tcpPort->socket);
        return tcpPort->socket;
    }

    // Set a timeout on the socket
#ifdef PLATFORM_IS_WINDOWS
    DWORD tv = timeout;
#else
    struct timeval tv;
    tv.tv_sec = timeout/1000;
    tv.tv_usec = (timeout % 1000) * 1000;
#endif
    if (setsockopt(tcpPort->socket, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv)) != 0) {
        tcpPort->base.perror = errno;
        return -errno;
    }

    // Receive data from the socket
    ssize_t retval = recv(tcpPort->socket, buf, len, 0);

    // Reset socket timeout
#ifdef PLATFORM_IS_WINDOWS
    tv = 0;
#else
    tv.tv_sec = 0;
    tv.tv_usec = 0;
#endif
    if (setsockopt(tcpPort->socket, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv)) != 0) {
        tcpPort->base.perror = errno;
        return -errno;
    }

    // Return
    if (retval < 0) {
        tcpPort->base.perror = errno;
        return -errno;
    }
    return retval;
}

int tcpPortWrite(port_handle_t port, const uint8_t* buf, unsigned int len) {
    tcp_port_t* tcpPort = TCP_PORT(port);
    if (tcpPort->socket < 0) { // The file descriptor is invalid, creating it errored, or we already closed it.
        tcpPort->base.perror = -(tcpPort->socket);
        return tcpPort->socket;
    }

    // Reset socket timeout
#ifdef PLATFORM_IS_WINDOWS
    DWORD tv = 0;
#else
    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 0;
#endif
    if (setsockopt(tcpPort->socket, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv)) != 0) {
        tcpPort->base.perror = errno;
        return -errno;
    }

    // Determine flags
    int flags = 0;
    if (!tcpPort->blocking) {
        flags |= MSG_DONTWAIT;
    }

    // Receive data from the socket
    ssize_t retval = send(tcpPort->socket, buf, len, flags);
    if (retval < 0) {
        tcpPort->base.perror = errno;
        return -errno;
    }
    return retval;
}

void tcpPortInit(port_handle_t port, int id, int type, bool blocking, const char* name, const struct sockaddr* ip) {
    tcp_port_t* tcpPort = TCP_PORT(port);
    tcpPort->base.pnum = id;
    tcpPort->base.ptype = type | PORT_TYPE__TCP | PORT_TYPE__COMM | PORT_FLAG__VALID;

    tcpPort->base.stats = (port_stats_t*)&(tcpPort->stats);

    tcpPort->base.portName = tcpPortGetName;
    tcpPort->base.portValidate = tcpPortValidate;
    tcpPort->base.portOpen = tcpPortOpen;
    tcpPort->base.portClose = tcpPortClose;
    tcpPort->base.portFree = tcpPortFree;
    tcpPort->base.portAvailable = tcpPortAvailable;
    tcpPort->base.portFlush = tcpPortFlush;
    tcpPort->base.portDrain = tcpPortDrain;
    tcpPort->base.portRead = tcpPortRead;
    tcpPort->base.portReadTimeout = tcpPortReadTimeout;
    tcpPort->base.portWrite = tcpPortWrite;

    tcpPort->socket = -EBADF;
    tcpPort->name = strdup(name);
    tcpPort->addr.generic = *ip;
    tcpPort->blocking = blocking;
}

void tcpPortDelete(port_handle_t port) {
    if (!port)
        return;
    tcp_port_t* tcpPort = TCP_PORT(port);
    free(tcpPort->name);
    memset(port, 0, sizeof(tcp_port_t));
}