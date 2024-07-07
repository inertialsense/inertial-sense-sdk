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
#define PORT_TYPE__UART             0x01
#define PORT_TYPE__USB              0x02
#define PORT_TYPE__SPI              0x03
#define PORT_TYPE__CAN              0x04
#define PORT_TYPE__LOOPBACK         0x0F

#define PORT_TYPE__GNSS             0x20    //! bit indicates that this port is a GNSS receiver port
#define PORT_TYPE__COMM             0x40    //! bit indicates that this port has an ISComm associated with it
#define PORT_TYPE__HDW              0x80    //! bit indicates that this port is static/hardware-defined

#define PORT_ERROR__NONE            0x00
#define PORT_ERROR__NOT_SUPPORTED   0x01

typedef void* port_handle_t;

typedef int(*pfnPortRead)(port_handle_t port, uint8_t* buf, int len);
typedef int(*pfnPortWrite)(port_handle_t port, const uint8_t* buf, int len);
typedef int(*pfnPortFree)(port_handle_t port);
typedef int(*pfnPortAvailable)(port_handle_t port);

typedef struct base_port_s {
    unsigned int pnum;              //! an identifier for a specific port that belongs to this device
    unsigned int ptype;             //! an indicator of the type of port

    pfnPortRead portRead;
    pfnPortWrite portWrite;
    pfnPortFree portFree;
    pfnPortFree portAvailable;
} base_port_t;
// typedef base_port_t* port_handle_t;

static int portId(port_handle_t port) { return ((base_port_t*)port)->pnum; }
static int portType(port_handle_t port) { return ((base_port_t*)port)->ptype; }
static int portRead(port_handle_t port, uint8_t* buf, int len) { return ((base_port_t*)port)->portRead ? ((base_port_t*)port)->portRead(port, buf, len) : PORT_ERROR__NOT_SUPPORTED; }
static int portWrite(port_handle_t port, const uint8_t* buf, int len) { return ((base_port_t*)port)->portWrite ? ((base_port_t*)port)->portWrite(port, buf, len) : PORT_ERROR__NOT_SUPPORTED; }
static int portFree(port_handle_t port) { return ((base_port_t*)port)->portFree ? ((base_port_t*)port)->portFree(port) : PORT_ERROR__NOT_SUPPORTED; }
static int portAvailable(port_handle_t port) { return ((base_port_t*)port)->portAvailable ? ((base_port_t*)port)->portAvailable(port) : PORT_ERROR__NOT_SUPPORTED; }

#endif //IS_CORE_TYPES_H
