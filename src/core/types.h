/**
 * @file types.h 
 * @brief ${BRIEF_DESC}
 *
 * @author Kyle Mallory on 7/3/24.
 * @copyright Copyright (c) 2024 Inertial Sense, Inc. All rights reserved.
 */

#ifndef IS_CORE_TYPES_H
#define IS_CORE_TYPES_H

#include <inttypes.h>

typedef enum {
    IS_LOG_LEVEL_NONE  = 0,
    IS_LOG_LEVEL_ERROR = 1,
    IS_LOG_LEVEL_WARN  = 2,
    IS_LOG_LEVEL_INFO  = 3,
    IS_LOG_LEVEL_MORE_INFO = 4,
    IS_LOG_LEVEL_DEBUG = 5,
    IS_LOG_LEVEL_MORE_DEBUG = 6,
    IS_LOG_LEVEL_SILLY = 7
} eLogLevel;

typedef enum {
    IS_BL_TYPE_NONE = 0,
    IS_BL_TYPE_SAMBA,
    IS_BL_TYPE_ISB,
    IS_BL_TYPE_APP,
    IS_BL_TYPE_DFU,
} eBootLoaderType;

/**
 * Port definitions used across the entire product line & SDK.
 */
#define PORT_TYPE__UNKNOWN          0xFFFF  //! Invalid or unknown port type
#define PORT_TYPE__UART             0x0001
#define PORT_TYPE__USB              0x0002
#define PORT_TYPE__SPI              0x0003
#define PORT_TYPE__CAN              0x0004
#define PORT_TYPE__LOOPBACK         0x000F

#define PORT_TYPE__GNSS             0x0020    //! bit indicates that this port is a GNSS receiver port
#define PORT_TYPE__COMM             0x0040    //! bit indicates that this port has an ISComm associated with it
#define PORT_TYPE__HDW              0x0080    //! bit indicates that this port is static/hardware-defined

#define PORT_ERROR__NONE                 0
#define PORT_ERROR__NOT_SUPPORTED       -1
#define PORT_ERROR__INVALID_PORT        -2
#define PORT_ERROR__WRITE_FAILURE       -3

#define PORT_OP__READ               0x00
#define PORT_OP__WRITE              0x01
#define PORT_OP__OPEN               0x02
#define PORT_OP__CLOSE              0x03
#define PORT_OP__FLUSH              0x04

typedef void* port_handle_t;

typedef int(*pfnPortFree)(port_handle_t port);
typedef int(*pfnPortAvailable)(port_handle_t port);
typedef int(*pfnPortRead)(port_handle_t port, uint8_t* buf, unsigned int len);
typedef int(*pfnPortWrite)(port_handle_t port, const uint8_t* buf, unsigned int len);
typedef int(*pfnPortLogger)(port_handle_t port, uint8_t op, const uint8_t* buf, unsigned int len, void* userData);
typedef const char*(*pfnPortName)(port_handle_t port);

typedef struct base_port_s {
    uint16_t pnum;                  //! an identifier for a specific port that belongs to this device
    uint16_t ptype;                 //! an indicator of the type of port

    pfnPortName portName;           //! a function which returns an optional name to (ideally) uniquely identify this port
    pfnPortFree portFree;           //! a function which returns the number of bytes which can safely be written
    pfnPortAvailable portAvailable; //! a function which returns the number of bytes available to be read
    pfnPortRead portRead;           //! a function to return copy some number of bytes available for reading into a local buffer (and removed from the ports read buffer)
    pfnPortWrite portWrite;         //! a function to copy some number of bytes from a local buffer into the ports write buffer (when and how this data is actually "sent" is implementation specific)
    pfnPortLogger portLogger;       //! a function, if set, to be called anytime a portRead or portWrite call is made; used to monitor/copy all data that goes through the port
    void *portLoggerData;           //! an opaque pointer of "user data" associated with the portLogger that is passed whenever the portLogger() callback function is called
} base_port_t;

/**
 * returns the Port ID for the specified port, or 0xFFFF (-1) if the port is invalid/null
 * @param port the port handle
 * @return the port ID
 */
static inline uint16_t portId(port_handle_t port) {
    return (port) ? ((base_port_t*)port)->pnum : 0xFFFF;
}

/**
 * returns the Port Type for the specified port, or 0xFFFF (-1) if the port is invalid/null.
 * The port type is a combination of ID + flags which defines the capability of the port.
 * Bits 0-3 determine the hardware implementation of the port (UART, SPI, etc).
 * Bits 4-7 are flags indicating the functional type of the port.
 * Bits 8-16 are reserved for future use
 * @param port the port handle
 * @return the port type
 */
static inline uint16_t portType(port_handle_t port) {
    return (port) ? ((base_port_t*)port)->ptype : 0xFFFF;
}

/**
 * Returns the name, if any, associated with this port
 * @param port
 * @return
 */
static inline const char *portName(port_handle_t port) {
    return (port && ((base_port_t*)port)->portName) ? ((base_port_t*)port)->portName(port) : (const char *)0;
}

/**
 * returns the number of free bytes in the underlying TX buffer, for new data to be queued for
 * sending. Attempting to send more than portFree() bytes will result in a TX_OVERFLOW being raised
 * on the port. It is the callers responsibility to detect this, and retain unsent data until all
 * data can be transmitted, or to discard the excess information.
 * @param port the port to query
 * @return the number of bytes which be be safely written to the port without data drop
 */
static inline int portFree(port_handle_t port) {
    if (port && ( (portType(port) <= 0) || (portType(port) >= 0xFF))) return PORT_ERROR__INVALID;
    return (port && ((base_port_t*)port)->portFree) ? ((base_port_t*)port)->portFree(port) : PORT_ERROR__NOT_SUPPORTED;
}

/**
 * returns the number of bytes available to be read from the underlying RX buffer, if any.
 * @param port the port to query
 * @return
 */
static inline int portAvailable(port_handle_t port) {
    if (port && ( (portType(port) <= 0) || (portType(port) >= 0xFF))) return PORT_ERROR__INVALID;
    return (port && ((base_port_t*)port)->portAvailable) ? ((base_port_t*)port)->portAvailable(port) : PORT_ERROR__NOT_SUPPORTED;
}

/**
 * a callback function called internally when a particular action (usually read or write) are
 * performed on the port, which is useful in logging or monitoring the data which goes through
 * the port
 * @param port the port which the operation was performed on
 * @param op an identifier of the operation that was performed
 * @param buf a pointer to the underlying data buffer used in the operation (you should avoid modifying this data unless you know what you are doing)
 * @param len the number of bytes of data associated with the operation
 * @param userData an opaque "user data" pointer which is provided by the port implementation
 * @return an implementation specific number
 */
static inline int portLog(port_handle_t port, uint8_t op, const uint8_t* buf, unsigned int len, void *userData) {
    if (port && ( (portType(port) <= 0) || (portType(port) >= 0xFF))) return PORT_ERROR__INVALID;
    return (port && ((base_port_t*)port)->portLogger) ? ((base_port_t*)port)->portLogger(port, op, buf, len, userData) : PORT_ERROR__NOT_SUPPORTED;
}

/**
 * Reads upto/at most 'len' number of bytes, copying those bytes into the buffer pointed to by 'buf'. Data copied into 'buf' is
 * removed from the internal RX buffer of the port, releasing 'len' bytes from the underlying buffer. It cannot be read again.
 * @param port the port from which to read data
 * @param buf the buffer to place a copy of the data into
 * @param len the maximum number of bytes to read; if fewer than 'len' bytes are available, only those bytes available will be returned.
 * @return the number of actual bytes read from the internal RX buffer.
 */
static inline int portRead(port_handle_t port, uint8_t* buf, unsigned int len) {
    if (port && ( (portType(port) <= 0) || (portType(port) >= 0xFF))) return PORT_ERROR__INVALID;
    if (!port && !((base_port_t*)port)->portRead) return PORT_ERROR__NOT_SUPPORTED;
    int bytesRead =  ((base_port_t*)port)->portRead(port, buf, len);
    if (port && ((base_port_t*)port)->portLogger) portLog(port, PORT_OP__READ, buf, bytesRead, ((base_port_t*)port)->portLoggerData);
    return bytesRead;
}

/**
 * Write the specified len bytes pointed to by buf to the provided port
 * @param port the port to send the data to
 * @param buf the buffer to send data from
 * @param len the number of bytes to send
 * @return the number of bytes sent (0 is valid), or <0 in the event of an error
 */
static inline int portWrite(port_handle_t port, const uint8_t* buf, unsigned int len) {
    if (port && ( (portType(port) <= 0) || (portType(port) >= 0xFF))) return PORT_ERROR__INVALID;
    if (port && ((base_port_t*)port)->portLogger) portLog(port, PORT_OP__WRITE, buf, len, ((base_port_t*)port)->portLoggerData);
    return (port && ((base_port_t*)port)->portWrite) ? ((base_port_t*)port)->portWrite(port, buf, len) : PORT_ERROR__NOT_SUPPORTED;
}

/**
 * Returns the portLogger function associated with this port, if any
 * @param port the port to query
 * @return the pointer to the pfnPortLogger function, or NULL if none.
 */
static inline pfnPortLogger portLogger(port_handle_t port) {
    return (port && ((base_port_t*)port)->portLogger) ? ((base_port_t*)port)->portLogger : 0;
}

/**
 * Sets the portLogger function to be associated with this port, or to clear it if NULL.
 * @param port the port to associate the logger with
 * @param portLogger a pointer to the pfnRportLogger function
 * @param loggerData an opaque "user data" pointer that will be passed to future calls to the portLogger function
 */
static inline void setPortLogger(port_handle_t port, pfnPortLogger portLogger, void* loggerData) {
    if (port) {
        ((base_port_t *) port)->portLogger = portLogger;
        ((base_port_t *) port)->portLoggerData = loggerData;
    }
}
// static inline void setPortLogger(port_handle_t port, pfnPortLogger portLogger) { setPortLogger(port, portLogger, NULL); }

static inline const char *portName(port_handle_t port) {
    if (port && ( (portType(port) <= 0) || (portType(port) >= 0xFF))) return (const char *)0;
    return (port && ((base_port_t*)port)->portName) ? ((base_port_t*)port)->portName(port) : (const char *)0;
}

#endif //IS_CORE_TYPES_H
