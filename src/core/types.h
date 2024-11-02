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
    uint16_t pnum;              //! an identifier for a specific port that belongs to this device
    uint16_t ptype;             //! an indicator of the type of port

    pfnPortName portName;
    pfnPortFree portFree;
    pfnPortAvailable portAvailable;
    pfnPortRead portRead;
    pfnPortWrite portWrite;
    pfnPortLogger portLogger;
    void *portLoggerData;

} base_port_t;

static inline uint16_t portId(port_handle_t port) {
    return (port) ? ((base_port_t*)port)->pnum : 0xFFFF;
}

static inline uint16_t portType(port_handle_t port) {
    return (port) ? ((base_port_t*)port)->ptype : 0xFFFF;
}

static inline int portFree(port_handle_t port) {
    return (port && ((base_port_t*)port)->portFree) ? ((base_port_t*)port)->portFree(port) : PORT_ERROR__NOT_SUPPORTED;
}

static inline int portAvailable(port_handle_t port) {
    return (port && ((base_port_t*)port)->portAvailable) ? ((base_port_t*)port)->portAvailable(port) : PORT_ERROR__NOT_SUPPORTED;
}

static inline int portLog(port_handle_t port, uint8_t op, const uint8_t* buf, unsigned int len, void *userData) {
    return (port && ((base_port_t*)port)->portLogger) ? ((base_port_t*)port)->portLogger(port, op, buf, len, userData) : PORT_ERROR__NOT_SUPPORTED;
}

static inline int portRead(port_handle_t port, uint8_t* buf, unsigned int len) {
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
    if (port && ((base_port_t*)port)->portLogger) portLog(port, PORT_OP__WRITE, buf, len, ((base_port_t*)port)->portLoggerData);
    return (port && ((base_port_t*)port)->portWrite) ? ((base_port_t*)port)->portWrite(port, buf, len) : PORT_ERROR__NOT_SUPPORTED;
}

static inline pfnPortLogger portLogger(port_handle_t port) { return (port && ((base_port_t*)port)->portLogger) ? ((base_port_t*)port)->portLogger : 0; }

static inline void setPortLogger(port_handle_t port, pfnPortLogger portLogger, void* loggerData) {
    if (port) {
        ((base_port_t *) port)->portLogger = portLogger;
        ((base_port_t *) port)->portLoggerData = loggerData;
    }
}
// static inline void setPortLogger(port_handle_t port, pfnPortLogger portLogger) { setPortLogger(port, portLogger, NULL); }

static inline const char *portName(port_handle_t port) {
    return (port && ((base_port_t*)port)->portName) ? ((base_port_t*)port)->portName(port) : (const char *)0;
}

#endif //IS_CORE_TYPES_H
