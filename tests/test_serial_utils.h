#include <cstdint>

#ifndef TEST_SERIAL_UTILS_H
#define TEST_SERIAL_UTILS_H

#include "core/types.h"
#include "ring_buffer.h"
#include "com_manager.h"

#define COM_BUFFER_SIZE     4096
#define PORT_BUFFER_SIZE    8192

typedef struct test_port_s {
    union {
        base_port_t base;
        comm_port_t comm;
    };

    rmci_t rmci;
    uint8_t rmciUPMcnt[DID_COUNT];
    uint8_t rmciNMEAcnt[NMEA_MSG_ID_COUNT];

    // Used to simulate serial ports
    ring_buf_t				loopbackPortBuf;
    uint8_t					loopbackportBuffer[PORT_BUFFER_SIZE];
} test_port_t;

extern test_port_t g_testPorts[NUM_COM_PORTS];
extern std::array<broadcast_msg_t, MAX_NUM_BCAST_MSGS> g_cmBufBcastMsg;

//#define TEST_PORT ((port_handle_t)(&g_testPort))
#define TEST_PORT(n)     ((test_port_t *)&g_testPorts[n])
#define TEST0_PORT       TEST_PORT(0)
#define TEST1_PORT       TEST_PORT(1)
#define TEST2_PORT       TEST_PORT(2)
#define TEST3_PORT       TEST_PORT(3)
#define TEST4_PORT       TEST_PORT(4)
#define TEST5_PORT       TEST_PORT(5)

// static is_comm_instance_t& g_comm = g_testPort.comm;
// static uint8_t* g_comm_buffer = g_testPort.buffer;

#define TEST_ENABLE_MANUAL_TX   0       // Set to 0 for normal loopback testing
#define TEST_ENABLE_MANUAL_RX   0       // Set to 0 for normal loopback testing

#if PLATFORM_IS_EMBEDDED
void serial_port_bridge_forward_unidirectional(is_comm_instance_t &comm, uint8_t &serialPortBridge, port_handle_t srcPort, port_handle_t dstPort, uint32_t led=0, int testMode=1);
#endif

static int loopbackPortRead(port_handle_t port, unsigned char* buf, int len);
static int loopbackPortWrite(port_handle_t port, const unsigned char* buf, int len);
static int loopbackPortFree(port_handle_t port);
static int loopbackPortAvailable(port_handle_t port);

void initTestPorts();

int64_t test_serial_rx_receive(uint8_t rxBuf[], int len, bool waitForStartSequence=true);
int test_serial_generate_ordered_data(uint8_t buf[], int bufSize);
void test_serial_delay_for_tx(int bufSize, int baudrate = 921600);

#endif // TEST_SERIAL_UTILS_H