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
#define ssize_t SSIZE_T
#else
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/ioctl.h>
#include <sys/time.h>
#include <sys/socket.h>
#endif

/**
 * Returns the name, if any, associated with this port
 * @param port
 * @return
 */
const char* tcpPortGetName(port_handle_t port) {
    tcp_port_t* tcpPort = TCP_PORT(port);
    return tcpPort->name;
}

/**
 * Determines viability of the specified port. Does not connect or otherwise interface with the port
 * directly, but generally indicates whether the underlying OS devices and resources exist in order to
 * successfully open and operate on the port. If the port is successfully validated, returns 0
 * @param port the port to validate
 * @return 0 if the port is valid, or <0 if an error occurred and port is invalid
 */
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

/**
 * Opens or establishes a connection to port.
 * @param port the port to open
 * @return 0 on success, -errno on error
 */
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

/**
 * Closes or disconnects a connection to port.
 * @param port the port to close
 * @return 0 on success, -errno on error
 */
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

/**
 * returns the number of free bytes in the underlying TX buffer, for new data to be queued for
 * sending. Attempting to send more than portFree() bytes will result in a TX_OVERFLOW being raised
 * on the port. It is the callers responsibility to detect this, and retain unsent data until all
 * data can be transmitted, or to discard the excess information.
 * This function isn't supported on Windows
 * @param port the port to query
 * @return number of free bytes in TX Buffer or -errno on Linux, not supported on Windows
 */
int tcpPortFree(port_handle_t port) {
#ifdef PLATFORM_IS_WINDOWS
    return PORT_ERROR__NOT_SUPPORTED;
#else
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
#endif
}

/**
 * returns the number of bytes available to be read from the underlying RX buffer, if any.
 * @param port the port to query
 * @return Number of bytes available to read from RX Buffer
 */
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

/**
 * flushes/removes all bytes currently waiting to be read from the port
 * @param port the port to query
 * @return if >= 0, the number of bytes that were flushed, otherwise -errno
 */
int tcpPortFlush(port_handle_t port) {
    tcp_port_t* tcpPort = TCP_PORT(port);
    if (tcpPort->socket < 0) { // The file descriptor is invalid, creating it errored, or we already closed it.
        tcpPort->base.perror = -(tcpPort->socket);
        return tcpPort->socket;
    }

    ssize_t clearedData = 0;
    while (true) {
        int bufferSize = tcpPortAvailable(port);
        int flag = 1;
        ioctl(tcpPort->socket,FIONBIO, &flag);
        if (bufferSize > 0) {
            void *bigBuffer = malloc(bufferSize);
            if (bigBuffer == NULL) {
                tcpPort->base.perror = errno;
                return -errno;
            }
            memset(bigBuffer, 0, bufferSize);
            ssize_t retval = recv(tcpPort->socket, bigBuffer, bufferSize, 0);
            free(bigBuffer); // Free doesn't modify errno
            bigBuffer = NULL;
            if (retval < 0) {
                tcpPort->base.perror = errno;
                return -errno;
            } else if (retval == 0) {
                flag = 0;
                ioctl(tcpPort->socket,FIONBIO, &flag);
                return clearedData;
            }
            clearedData += retval;
        } else if (bufferSize == 0) {
            flag = 0;
            ioctl(tcpPort->socket,FIONBIO, &flag);
            return clearedData;
        } else {
            flag = 0;
            ioctl(tcpPort->socket,FIONBIO, &flag);
            tcpPort->base.perror = -bufferSize;
            return bufferSize;
        }
    }
}

/**
 * Tells the kernel to transmit all queued TX data to be sent to the device.
 * This effectively flushes the TX buffer. This will return instantly
 * @param port the port to query
 * @param timeout is ignored for this function
 * @return 0 on success, otherwise an -errno
 */
int tcpPortDrain(port_handle_t port, uint32_t timeout) {
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

    // Set nonblocking
    flag = 0;
    if (!tcpPort->blocking) {
        flag = 1;
    }
    ioctl(tcpPort->socket,FIONBIO, &flag);

    // Makes sures the kernel actually tries to send something
    retval = send(tcpPort->socket, NULL, 0, 0);
    if (retval < 0) {
        tcpPort->base.perror = errno;
        return -errno;
    }

    // Tell the kernel to buffer messages again cause TCP_NODELAY kills network performance
    ioctl(tcpPort->socket,FIONBIO, &flag);
    retval = setsockopt(tcpPort->socket, IPPROTO_TCP, TCP_NODELAY, &flag, sizeof(int));
    if (retval != 0) {
        tcpPort->base.perror = errno;
        return -errno;
    }
    return 0;
}

/**
 * Reads upto/at most 'len' number of bytes, copying those bytes into the buffer pointed to by 'buf'. Data copied into 'buf' is
 * removed from the internal RX buffer of the port, releasing 'len' bytes from the underlying buffer. It cannot be read again.
 * @param port the port from which to read data
 * @param buf the buffer to place a copy of the data into
 * @param len the maximum number of bytes to read; if fewer than 'len' bytes are available, only those bytes available will be returned.
 * @return the number of actual bytes read from the internal RX buffer or -errno on error
 */
int tcpPortRead(port_handle_t port, uint8_t* buf, unsigned int len) {
    tcp_port_t* tcpPort = TCP_PORT(port);
    if (tcpPort->socket < 0) { // The file descriptor is invalid, creating it errored, or we already closed it.
        tcpPort->base.perror = -(tcpPort->socket);
        return tcpPort->socket;
    }

    // Determine flags
    int flag = 0;
    if (!tcpPort->blocking) {
        flag = 1;
    }
    ioctl(tcpPort->socket,FIONBIO, &flag);

    // Receive data from the socket
    ssize_t retval = recv(tcpPort->socket, buf, len, 0);
    if (retval < 0) {
        tcpPort->base.perror = errno;
        return -errno;
    }
    flag = 0;
    ioctl(tcpPort->socket,FIONBIO, &flag);
    return retval;
}

/**
 * Reads upto/at most 'len' number of bytes, copying those bytes into the buffer pointed to by 'buf'. Data copied into 'buf' is
 * removed from the internal RX buffer of the port, releasing 'len' bytes from the underlying buffer. It cannot be read again.
 * @param port the port from which to read data
 * @param buf the buffer to place a copy of the data into
 * @param len the maximum number of bytes to read; if fewer than 'len' bytes are available, only those bytes available will be returned.
 * @param timeout the maximum time, in milliseconds, to wait for 'len' bytes to be received before returning.
 * @return the number of actual bytes read from the internal RX buffer or -errno on error
 */
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

/**
 * Write the specified len bytes pointed to by buf to the provided port
 * @param port the port to send the data to
 * @param buf the buffer to send data from
 * @param len the number of bytes to send
 * @return the number of bytes sent (0 is valid), or -errno in event on an error
 */
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
    int flag = 0;
    if (!tcpPort->blocking) {
        flag = 1;
    }
    ioctl(tcpPort->socket,FIONBIO, &flag);

    // Receive data from the socket
    ssize_t retval = send(tcpPort->socket, buf, len, 0);
    if (retval < 0) {
        tcpPort->base.perror = errno;
        return -errno;
    }
    flag = 0;
    ioctl(tcpPort->socket,FIONBIO, &flag);
    return retval;
}

/**
 * Initializes a new tcp port with the following parameters
 * @param port The port handle to initialize
 * @param id The id of the new port handle
 * @param blocking To configure if this port is blocking
 * @param name The name of the new port
 * @param ip The address and port to connect to over TCP
 */
void tcpPortInit(port_handle_t port, int id, bool blocking, const char* name, const struct sockaddr* ip) {
    tcp_port_t* tcpPort = TCP_PORT(port);
    tcpPort->base.pnum = id;
    tcpPort->base.ptype = PORT_TYPE__TCP | PORT_TYPE__COMM | PORT_FLAG__VALID;

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

/**
 * Deinitializes a tcp port and clears and zeros it's allocated memory
 * @param port The port handle to deinitialize
 */
void tcpPortDelete(port_handle_t port) {
    if (!port)
        return;
    tcp_port_t* tcpPort = TCP_PORT(port);
    free(tcpPort->name);
    memset(port, 0, sizeof(tcp_port_t));
}