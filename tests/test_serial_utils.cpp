/**
 * @file test_serial_utils.cpp
 * @brief a collection of functions and classes that might be useful when writing/running unit tests
 *
 * @author Walt Johnson on 5/16/24
 * @copyright Copyright (c) 2024 Inertial Sense, Inc. All rights reserved.
 */

#include "ISConstants.h"
#include "test_serial_utils.h"

#if PLATFORM_IS_EMBEDDED
#include "drivers/d_time.h"
#include "drivers/d_serial.h"
#define TIME_USEC()             time_usec()
#define TIME_DELAY_USEC(us)     time_delay_usec(us)
#else
#include "ISUtilities.h"
#define TIME_USEC()             current_timeUs()
#define TIME_DELAY_USEC(us)     SLEEP_US(us)
#endif

#if 1
uint32_t g_debugDtUsMax = 0;
uint32_t g_debugTimeUsLast = 0;
void test_serial_rx_profile_frequency(void)
{
    // Profile read frequency
	uint32_t timeUs = TIME_USEC(); 
    if (g_debugTimeUsLast==0) { g_debugTimeUsLast = timeUs; }
    uint32_t dtUs = timeUs - g_debugTimeUsLast;

    g_debugTimeUsLast = timeUs;
    if (g_debugDtUsMax < dtUs)
    {
        g_debugDtUsMax = dtUs;
    }
}


// Manual test used to verify that a repeating consecutive series of uint8 data from 0 to 255 is received.  The test is reset when zero is received.
void test_serial_rx_receive(uint8_t rxBuf[], int n)
{
#if 0   // Rx Test - 8-bit
    static uint8_t testVal = 0;
    static uint8_t testValLast = 0;
    static uint8_t rxBufLast = 0;
    static uint64_t count = 0;
    for (int i=0; i<n; i++)
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

    static uint16_t rxLast = 0;        
    static uint16_t testVal = 0;
    static uint16_t testValLast = 0;
    static bool rxUpperByte = false;
    static uint8_t rxByteLast[3] = {0};
    static uint64_t count = 0;
    for (int i=0; i<n; i++)
    {
        uint8_t rxByte = rxBuf[i];

        if (rxByteLast[2] == 0 && rxByteLast[1] == 0 &&
            rxByteLast[0] == 1 && rxByte == 0)
        {   // Received 0x00 0x00 0x01 0x00.  Reset Rx testVal.
            count = 3;
            rx.u16 = testVal = 1;
            g_debugDtUsMax = 0;
            rxUpperByte = true;
        }

        if (rxUpperByte)
        {   // Upper byte comes second
            rx.ub = rxByte;
            rxUpperByte = false;

            // Run the test (exclude zero because it is used to reset test)
            if (rx.u16 != 0 && rx.u16 != testVal)
            {   // Uncomment and put breakpoint here
                while(1);
            }
            testValLast = testVal;
            rxLast = rx.u16;
            
            testVal++;
        }
        else
        {   // Lower byte comes first
            rx.lb = rxByte;
            rxUpperByte = true;
        }
        
        count++;
        rxByteLast[2] = rxByteLast[1];
        rxByteLast[1] = rxByteLast[0];
        rxByteLast[0] = rxByte;
    }
#endif
}
#endif


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


void test_serial_delay_for_tx(int bufSize, int baudrate)
{
    int bytes_per_sec = (baudrate/10);  // ~10 (bits/byte)

    // Delay for enough time to allow data
    int delayUs = (1000000 * bufSize / bytes_per_sec);
    TIME_DELAY_USEC(delayUs);
}
