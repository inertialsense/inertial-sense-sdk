/**
 * @file CustomVirtualPort.h
 * @brief From a collection of functions and classes that might be useful when writing/running unit tests,
 * ported to use as customer example of building a custom port implementation to extend base_port
 *
 * @author TylerS
 * @remark Originated as Walt Johonson's tests/test_serial_utils.h
 * @copyright Copyright (c) 2026 Inertial Sense, Inc. All rights reserved.
 */

#ifndef CUSTOM_VIRTUAL_PORT_H
#define CUSTOM_VIRTUAL_PORT_H

/** STEP 1: Include IS core, other needed SDK header files here, and your own channel implementation files
 * as required
 */
#include "core/base_port.h"
#include "ring_buffer.h"  //optional, depends upon your implementation
#include "com_manager.h" //optional, depends upon your implementation

/**
 * PORT IMPLEMENTATION used for unit and functional tests
 * This is a generic port implementation that provides both bridging and loopback capability
 * There are 6 ports defined:
 *      LOOPBACK (TEST0_PORT, TEST1_PORT):
 *         All data that is written to the port is placed into a ringbuffer which feeds subsequent reads
 *         If no data has been written, there is not data to be read.  If you want to test synchronous
 *         functionality, use a loopback and perform all your writes, then read from the same port to
 *         ensure the data was written (or is read/parsed) correctly.
 *
 *      BRIDGE (TEST2_PORT <-> TEST3_PORT, TEST4_PORT <-> TEST5_PORT):
 *         All data that is written to the port of written into the ringbuffer of the paired port.
 */

#define COM_BUFFER_SIZE     4096
#define PORT_BUFFER_SIZE    8192

/** STEP 2: Create your custom port declaration, extending base_port_t
 */
typedef struct custom_port_s {
    union {
        base_port_t base;
        comm_port_t comm;  //optional depending upon your application
    };

    // Used to simulate serial ports
    ring_buf_t      portRingBuf;
    uint8_t         portBuffer[PORT_BUFFER_SIZE];
    uint8_t         name[6];
} custom_port_t;

/** These are defined in the .cpp file
 */
extern custom_port_t g_customPorts[NUM_COM_PORTS];
extern std::array<broadcast_msg_t, MAX_NUM_BCAST_MSGS> g_cmBufBcastMsg;

/** Macros that give us an easy way to reference the various custom test ports
 */
#define TEST_PORT(n)     ((custom_port_t *)&g_customPorts[n])
#define TEST0_PORT       TEST_PORT(0)
#define TEST1_PORT       TEST_PORT(1)
#define TEST2_PORT       TEST_PORT(2)
#define TEST3_PORT       TEST_PORT(3)
#define TEST4_PORT       TEST_PORT(4)
#define TEST5_PORT       TEST_PORT(5)

#define TEST_ENABLE_MANUAL_TX   0       // Set to 0 for normal loopback testing
#define TEST_ENABLE_MANUAL_RX   0       // Set to 0 for normal loopback testing

/** Declarations for the core port functions which will provide the underlying implementation for the base_port
 */
static int customPortRead(port_handle_t port, unsigned char* buf, unsigned int len);
static int customPortWrite(port_handle_t port, const unsigned char* buf, unsigned int len);
static int customPortFree(port_handle_t port);
static int customPortAvailable(port_handle_t port);
static const char* customPortName(port_handle_t port);

/** Other internal support functions, not hooked to the base_port API
 */
void initCustomPorts();


#endif // CUSTOM_VIRTUAL_PORT_H
