/**
 * @file CustomVirtualPort.cpp
 * @brief From a collection of functions and classes that might be useful when writing/running unit tests,
 * ported to use as customer example of building a custom port implementation to extend base_port
 *
 * @author TylerS
 * @remark Originated as Walt Johonson's tests/test_serial_utils.cpp
 * @copyright Copyright (c) 2026 Inertial Sense, Inc. All rights reserved.
 */

/** STEP 1: Include IS core and other needed SDK header files here
 */
#include <array>
#include <string>
#include <stdexcept>

#include "ISConstants.h"
#include "ISComm.h"
#include "com_manager.h"
#include "CustomVirtualPort.h"

#include "util/util.h"
#include "../src/ISUtilities.h"
#define TIME_USEC()             current_timeUs()
#define TIME_DELAY_USEC(us)     SLEEP_US(us)

/** For this example, we use a fixed stack allocation of our ports
 */
custom_port_t g_customPorts[NUM_COM_PORTS] = {};

std::array<broadcast_msg_t, MAX_NUM_BCAST_MSGS> g_cmBufBcastMsg; // [MAX_NUM_BCAST_MSGS];


/** This indexes pointers to the custom ports
 */
static custom_port_t* boundPorts[NUM_COM_PORTS] {
        #if (NUM_COM_PORTS > 0)
            TEST0_PORT, // loopback
        #endif
        #if (NUM_COM_PORTS > 1)
            TEST1_PORT, // loopback
        #endif
        #if (NUM_COM_PORTS > 3)
            TEST3_PORT, TEST2_PORT, // PORT2 <-> PORT3
        #endif
        #if (NUM_COM_PORTS > 5)
            TEST5_PORT, TEST4_PORT, // PORT4 <-> PORT5
        #endif
};

/** Implementations of our core functions for this custom virtual port
 */
static int customPortRead(port_handle_t port, unsigned char* buf, unsigned int len)
{
    return ringBufRead(&((custom_port_t*)port)->portRingBuf, buf, len);
}

static int customPortWrite(port_handle_t port, const unsigned char* buf, unsigned int len)
{
    custom_port_t* destPort = boundPorts[portId(port)];

    if (ringBufWrite(&destPort->portRingBuf, (unsigned char*)buf, len))
    {   
        // Buffer overflow
        throw new std::out_of_range(utils::string_format("customPortWrite ring buffer overflow on %s: %d !!!\n",
                                                         portName(destPort), ringBufUsed(&destPort->portRingBuf) + len));

        return PORT_ERROR__WRITE_FAILURE;
    }
    return len;
}

static int customPortFree(port_handle_t port) {
    return ringBufFree(&((custom_port_t*)port)->portRingBuf);
}

static int customPortAvailable(port_handle_t port) {
    return ringBufUsed(&((custom_port_t*)port)->portRingBuf);
}

static const char* customPortName(port_handle_t port) {
    return (const char*)((custom_port_t*)port)->name;
}

/** Implementations of our support functions for this custom virtual port
 */
void initCustomPorts() {
    int portNum = 0;
    for (custom_port_t& port : g_customPorts) {
        port.base.pnum = portNum;
        port.base.ptype = PORT_TYPE__COMM;
        if (portNum <= 1)
            port.base.ptype |= PORT_TYPE__LOOPBACK;  // only PORT0 and PORT1 are Loopbacks

        port.base.portRead = customPortRead;
        port.base.portWrite = customPortWrite;
        port.base.portFree = customPortFree;
        port.base.portAvailable = customPortAvailable;
        port.base.portName = customPortName;
        portFlagsSet(&port, PORT_FLAG__VALID);
        portFlagsSet(&port, PORT_FLAG__OPENED);

        ringBufInit(&port.portRingBuf, port.portBuffer, PORT_BUFFER_SIZE, 1);
        SNPRINTF((char *)port.name, 6, "TEST%1d", portNum);

        portNum++;
    }
} //initCustomPorts

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

    // Rx Test - 16-bit
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
//                while (1);
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

    return count;
} //test_serial_rx_receive

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

    uint16_t *ptr = (uint16_t*)buf;

    // Populate buffer
    for (int i=0; i<bufSize/2; i++)
    {
        ptr[i] = testVal;
        testVal++;
    }

    return bufSize;
}

/**
 * @brief Calculate a delay sufficient for the specified data at baudrate to be sent,
 *  preventing buffer overflow and optionally sleep for that duration.
 * @param bytes number of bytes that has/will be sent
 * @param baud Size of available buffer.
 * @param sleep if true (default) will sleep for the calculated time, otherwise will return immediately
 * @return the calculated wait in microseconds
 */
int test_serial_delay_for_tx(int bytes, int baud, bool sleep)
{
    int bytes_per_sec = (baud / 10);  // ~10 (bits/byte)

    // Delay for enough time to allow data
    int delayUs = (1000000 * bytes / bytes_per_sec) + 10;     // + 10us additional for buffer
    if (!sleep) TIME_DELAY_USEC(delayUs);
    return delayUs;
}


