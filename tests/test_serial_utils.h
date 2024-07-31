#include <cstdint>

#define TEST_ENABLE_MANUAL_TX   0       // Set to 0 for normal loopback testing
#define TEST_ENABLE_MANUAL_RX   0       // Set to 0 for normal loopback testing


#if PLATFORM_IS_EMBEDDED
void serial_port_bridge_forward_unidirectional(is_comm_instance_t &comm, uint8_t &serialPortBridge, unsigned int srcPort, unsigned int dstPort, uint32_t led=0, int testMode=1);
#endif
int64_t test_serial_rx_receive(uint8_t rxBuf[], int len, bool waitForStartSequence=true);
int test_serial_generate_ordered_data(uint8_t buf[], int bufSize);
void test_serial_delay_for_tx(int bufSize, int baudrate = 921600);

