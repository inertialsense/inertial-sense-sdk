

void test_serial_rx_profile_frequency(void);
void test_serial_rx_receive(uint8_t rxBuf[], int n);
int test_serial_generate_ordered_data(uint8_t buf[], int bufSize);
void test_serial_delay_for_tx(int bufSize, int baudrate = 921600);