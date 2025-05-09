#include <cstdint>

#ifndef TEST_SERIAL_UTILS_H
#define TEST_SERIAL_UTILS_H

#include "core/base_port.h"
#include "ring_buffer.h"
#include "com_manager.h"

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

typedef struct test_port_s {
    union {
        base_port_t base;
        comm_port_t comm;
    };

    rmci_t          rmci;
    uint8_t         rmciUPMcnt[DID_COUNT];
    uint8_t         rmciNMEAcnt[NMEA_MSG_ID_COUNT];

    // Used to simulate serial ports
    ring_buf_t      portRingBuf;
    uint8_t         portBuffer[PORT_BUFFER_SIZE];
    uint8_t         name[6];
} test_port_t;

extern test_port_t g_testPorts[NUM_COM_PORTS];
extern std::array<broadcast_msg_t, MAX_NUM_BCAST_MSGS> g_cmBufBcastMsg;

#define TEST_PORT(n)     ((test_port_t *)&g_testPorts[n])
#define TEST0_PORT       TEST_PORT(0)
#define TEST1_PORT       TEST_PORT(1)
#define TEST2_PORT       TEST_PORT(2)
#define TEST3_PORT       TEST_PORT(3)
#define TEST4_PORT       TEST_PORT(4)
#define TEST5_PORT       TEST_PORT(5)

#define TEST_ENABLE_MANUAL_TX   0       // Set to 0 for normal loopback testing
#define TEST_ENABLE_MANUAL_RX   0       // Set to 0 for normal loopback testing

#if PLATFORM_IS_EMBEDDED
void serial_port_bridge_forward_unidirectional(is_comm_instance_t &comm, uint8_t &serialPortBridge, port_handle_t srcPort, port_handle_t dstPort, uint32_t led=0, int testMode=1);
#endif

static int testPortRead(port_handle_t port, unsigned char* buf, unsigned int len);
static int testPortWrite(port_handle_t port, const unsigned char* buf, unsigned int len);
static int testPortFree(port_handle_t port);
static int testPortAvailable(port_handle_t port);
static const char* testPortName(port_handle_t port);

void initTestPorts();

int64_t test_serial_rx_receive(uint8_t rxBuf[], int len, bool waitForStartSequence=true);
int test_serial_generate_ordered_data(uint8_t buf[], int bufSize);
void test_serial_delay_for_tx(int bufSize, int baudrate = 921600);

#endif // TEST_SERIAL_UTILS_H