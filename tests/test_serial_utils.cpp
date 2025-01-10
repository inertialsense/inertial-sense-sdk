/**
 * @file test_serial_utils.cpp
 * @brief a collection of functions and classes that might be useful when writing/running unit tests
 *
 * @author Walt Johnson on 5/16/24
 * @copyright Copyright (c) 2025 Inertial Sense, Inc. All rights reserved.
 */

#include "../src/ISComm.h"
#include "test_serial_utils.h"

#if PLATFORM_IS_EMBEDDED
#include "drivers/d_time.h"
#include "drivers/d_serial.h"
#if defined(IMX_5)
#include "drivers/d_watchdog.h"
#endif
#define TIME_USEC()             time_usec()
#define TIME_DELAY_USEC(us)     time_delay_usec(us)
#else
#include "../src/ISUtilities.h"
#define TIME_USEC()             current_timeUs()
#define TIME_DELAY_USEC(us)     SLEEP_US(us)
#endif


#if PLATFORM_IS_EMBEDDED
void serWriteInPieces(int serPort, const unsigned char *buf, int length)
{
    int left = length; 
    for (int send=1; left>0; send*=2)
    {
        send = _MIN(left, send);
        serWrite(serPort, &(buf[length - left]), send);
        left -= send; 
    }
}

/**
 * @brief Forward data read from one serial port to another.  Includes testmode used to verify integrity of serial port driver.  
 * 
 * @param comm is_comm_instance_t used for parsing data (to disable serial port bridge)
 * @param srcPort Serial port to read from.
 * @param dstPort Serial port to write to.
 * @param testMode Enable test mode to perform sequential serWrite() calls back to back to test capability of the serial driver.
 */
void serial_port_bridge_forward_unidirectional(is_comm_instance_t &comm, uint8_t &serialPortBridge, unsigned int srcPort, unsigned int dstPort, uint32_t led, int testMode)
{
#if TEST_ENABLE_MANUAL_TX   // Manual Tx Test - Uncomment and run device_tx_manual_test in run test_serial_driver.cpp 
    while(1)
    {
        uint8_t txBuf[200];
        int n = test_serial_generate_ordered_data(txBuf, sizeof(txBuf));
#if 1   // Send data once
        serWrite(dstPort, (unsigned char*)&(txBuf), n);
#else   // Send data in pieces
        serWriteInPieces(dstPort, (unsigned char*)&(txBuf), n);
#endif
        test_serial_delay_for_tx(n+5);

#if PLATFORM_IS_EMBEDDED && defined(IMX_5)
        // Prevent watchdog reset
        watchdog_preemptive();  watchdog_maintenance();
#endif
    }
#endif

    // gpio_toggle_level(G19_QDEC1B_PIN);   // GPX debug

    int n = is_comm_free(&comm);    // Call before adding data to comm->rxBuf.tail with serRead().
    if ((n = serRead(srcPort, comm.rxBuf.tail, n)) <= 0)
    {   // No data to forward
        return;
    }

#if TEST_ENABLE_MANUAL_RX   // Manual Rx Test - Uncomment and run device_onboard_rx_manual_test in run test_serial_driver.cpp 
    if (test_serial_rx_receive(comm.rxBuf.tail, n) < 0)
    {	// Catch error here
    	while(1);
    }
    return;  // Return to prevent Tx
#endif

    // Forward data
    if (testMode)
    {   // Test mode enabled for driver testing.  Forward data in pieces.
        serWriteInPieces(dstPort, comm.rxBuf.tail, n);
    }
    else
    {   // All at once
        serWrite(dstPort, comm.rxBuf.tail, n);
    }

    // Update comm buffer tail pointer
    comm.rxBuf.tail += n;

#if PLATFORM_IS_EMBEDDED && !defined(IMX_5) && !defined(GPX_1)
    if (led){ LED_TOGGLE(led); }
#endif

    //////////////////////////////////////////////////
    // Data parser follows
    static uint32_t enabledMaskBackup=0;
    if (enabledMaskBackup==0)
    {   
        enabledMaskBackup = comm.config.enabledMask;
        comm.config.enabledMask = ENABLE_PROTOCOL_ISB;      // Disable all protocols except ISB to prevent delays in parsing that could cause data drop
    }
    protocol_type_t ptype;
    while((ptype = is_comm_parse(&comm)) != _PTYPE_NONE)
    {
        switch (ptype)
        {	
        default:	break;	// Do nothing

        case _PTYPE_INERTIAL_SENSE_DATA:
            if (comm.rxPkt.dataHdr.id == DID_SYS_CMD)
            {
                system_command_t *cmd = (system_command_t*)(comm.rxPkt.data.ptr);
                if (cmd->command == ~cmd->invCommand)
                {	// Valid command
                    switch (cmd->command)
                    {
                    case SYS_CMD_DISABLE_SERIAL_PORT_BRIDGE:
                        // Look for disable serial port bridge command
                        // Require users to first disable serial bridge before enabling other bridge
                        // We want to keep all serial bridge enable code in writeSysCmd().  WHJ
                        serialPortBridge = 0;

                        // Restore enabled protocol mask
                        comm.config.enabledMask = enabledMaskBackup;
                        enabledMaskBackup = 0;
                        testMode = 0;
                        break;
                    }
                }
            }
            break;
        }
    }
}
#endif  // PLATFORM_IS_EMBEDDED

/**
 * @brief Manual test used to verify that a repeating consecutive series of uint8 data from 
 * 0 to 255 is received. The test is reset when start sequence is received is received.
 * 
 * @param rxBuf Data received
 * @param len number of bytes received
 * @param waitForStartSequence If test should wait for start sequence before running the test.
 * @return int64_t Number of bytes received with test passing.  -1 if test fails.
 */
int64_t test_serial_rx_receive(uint8_t rxBuf[], int len, bool waitForStartSequence)
{
    static bool waitForStart = waitForStartSequence;

#if 0   // Rx Test - 8-bit
    static uint8_t testVal = 0;
    static uint8_t testValLast = 0;
    static uint8_t rxBufLast = 0;
    static uint64_t count = 0;
    for (int i=0; i<len; i++)
    {
        if (rxBuf[i] == 0)
        {   // Reset Rx testVal
            testVal = 0;
            g_debugDtUsMax = 0;
        }
        
        if (rxBuf[i] != testVal)
        {   // Uncomment and put breakpoint here
            while(1);
        }
        rxBufLast = rxBuf[i];
        testValLast = testVal;

        testVal++;
        count++;
    }
#else   // Rx Test - 16-bit
    static union
    {
        struct
        {
            uint8_t lb;         // Lower byte
            uint8_t ub;         // Upper byte
        };
        struct
        {
            uint16_t u16;
        };
    } rx;

    static uint16_t testVal = 0;
    static bool rxUpperByte = false;
    static uint8_t rxByteLast[3] = {0};
    static int64_t count = 0;
    for (int i=0; i<len; i++)
    {
        uint8_t rxByte = rxBuf[i];

        if (rxByteLast[2] == 0 && rxByteLast[1] == 0 &&
            rxByteLast[0] == 1 && rxByte == 0)
        {   // Received 0x00 0x00 0x01 0x00.  Reset Rx testVal.
            // count = 3;
            rx.u16 = testVal = 1;
            rxUpperByte = true;
            waitForStart = false;
        }

        rxByteLast[2] = rxByteLast[1];
        rxByteLast[1] = rxByteLast[0];
        rxByteLast[0] = rxByte;

        if (waitForStart)
        {
            continue;
        }

        if (rxUpperByte)
        {   // Upper byte comes second
            rx.ub = rxByte;
            rxUpperByte = false;

            // Run the test (exclude zero because it is used to reset test)
            if (rx.u16 != 0 && rx.u16 != testVal)
            {   // Uncomment and put breakpoint here
//                while(1);
                return -1;
            }
            testVal++;
        }
        else
        {   // Lower byte comes first
            rx.lb = rxByte;
            rxUpperByte = true;
        }
        
        count++;
    }
#endif

    return count;
}

/**
 * @brief Generate Tx data for manual serial test. 
 * 
 * @param buf Buffer where data is to be written.  Must be a multiple of two bytes.
 * @param bufSize Size of available buffer.
 * @return int 
 */
int test_serial_generate_ordered_data(uint8_t buf[], int bufSize)
{
    static uint16_t testVal = 0;

#if 0   // 8-bit count
    for (int i=1; i<bufSize; i++)
    {
        buf[i] = (uint8_t)testVal;
        testVal++;
    }
#else   // 16-bit count
    uint16_t *ptr = (uint16_t*)buf;

    // Populate buffer
    for (int i=0; i<bufSize/2; i++)
    {
        ptr[i] = testVal;
        testVal++;
    }
#endif

    return bufSize;
}

/**
 * @brief Creates a delay sufficient for the specified data at baudrate to be sent, preventing buffer overflow.
 * 
 * @param bufSize Buffer for data to be written to.
 * @param baudrate Size of available buffer.
 */
void test_serial_delay_for_tx(int bufSize, int baudrate)
{
    int bytes_per_sec = (baudrate/10);  // ~10 (bits/byte)

    // Delay for enough time to allow data
    int delayUs = (1000000 * bufSize / bytes_per_sec) + 10;     // + 10us additional for buffer
    TIME_DELAY_USEC(delayUs);
}


