/*
MIT LICENSE

Copyright 2014-2018 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include <stdio.h>

// STEP 1: Add Includes
// Change these include paths to the correct paths for your project
#include "ISComm.h"
#include "ISPose.h"
#include "ISUtilities.h"
#include "PortFactory.h"
#include "protocol_nmea.h"

/**
 * Simple custom handler for the DID_INS_1 message
 * @param ins the ins_1_t message structure
 */
static void handleIns1Message(ins_1_t* ins)
{
    printf("INS TimeOfWeek: %.3fs, LLA: %3.7f,%3.7f,%5.2f, Euler: %5.1f,%5.1f,%5.1f\r\n",
        ins->timeOfWeek,
        ins->lla[0], ins->lla[1], ins->lla[2],
        ins->theta[0] * C_RAD2DEG_F, ins->theta[1] * C_RAD2DEG_F, ins->theta[2] * C_RAD2DEG_F);
}

/**
 * Simple custom handler for the DID_INS_2 message
 * @param ins the ins_2_t message structure
 */
static void handleIns2Message(ins_2_t* ins)
{
    ixVector3 theta;
    quat2euler(ins->qn2b, theta);

    printf("INS TimeOfWeek: %.3fs, LLA: %3.7f,%3.7f,%5.2f, Euler: %5.1f,%5.1f,%5.1f\r\n",
        ins->timeOfWeek,
        ins->lla[0], ins->lla[1], ins->lla[2],
        theta[0] * C_RAD2DEG_F, theta[1] * C_RAD2DEG_F, theta[2] * C_RAD2DEG_F);
}

/**
 * Simple custom handler for the DID_GPS1_POS message
 * @param ins the gps_pos_t message structure
 */
static void handleGpsMessage(gps_pos_t* pos)
{
    printf("GPS TimeOfWeek: %dms, LLA: %3.7f,%3.7f,%5.2f\r\n", pos->timeOfWeekMs, pos->lla[0], pos->lla[1], pos->lla[2]);
}

/**
 * Simple custom handler for the DID_IMU message
 * @param ins the imu_t message structure
 */
static void handleImuMessage(imu_t* imu)
{
    printf("IMU Time: %.3fs, PQR: %5.1f,%5.1f,%5.1f, ACC: %5.1f,%5.1f,%5.1f,\r\n",
        imu->time,
        imu->I.pqr[0], imu->I.pqr[1], imu->I.pqr[2],
        imu->I.acc[0], imu->I.acc[1], imu->I.acc[2]);
}

/**
 * This is the callback handler from the ISComm parser; this will be called for each
 * InertialSense binary message that is successfully parsed.
 * @param ctx a context pointer that can be associated with the port/ISCOMM instance.
 *   This is usually used for instance pointers in C++ implementations.
 * @param data a pointer to the packet/message structure of the parsed message
 * @param port the port the packet/message was received by
 * @return
 */
int isbDataHandler(void* ctx, p_data_t* data, port_handle_t port) {
     switch (data->hdr.id)
     {
     case DID_INS_1:
         handleIns1Message((ins_1_t*)data->ptr);
         break;

     case DID_INS_2:
         handleIns2Message((ins_2_t*)data->ptr);
         break;

     case DID_GPS1_POS:
         handleGpsMessage((gps_pos_t*)data->ptr);
         break;

     case DID_IMU:
         handleImuMessage((imu_t*)data->ptr);
         break;

         // TODO: add other cases for other data ids that you care about
     }

     return 0;
}

int main(int argc, char* argv[])
{
    if (argc < 2)
    {
        printf("Please pass the com port as the only argument (i.e. /dev/ttyACM0 or COM5)\r\n");
        // In Visual Studio IDE, this can be done through "Project Properties -> Debugging -> Command Arguments: COM3" 
        return -1;
    }

    // STEP 2: Initialize and open serial port
    // Initialize the serial port (Windows, MAC or Linux) - if using an embedded system like Arduino,
    //  you will need to handle the serial port creation, open and reads yourself.

    SerialPortFactory& spf = SerialPortFactory::getInstance();
    spf.setBaudRate(921600);
    port_handle_t port = spf.bindPort(argv[1]);

    if (port == nullptr) {
        printf("Failed to allocate port\r\n");
        return -2;
    }

    // Binding a port does not open a port.. so let's open it
    if (!portIsOpened(port) && (portOpen(port) != PORT_ERROR__NONE)) {
        printf("Failed to open port\r\n");
        return -3;
    }

    // STEP 3: Stop any message broadcasting
    is_comm_stop_broadcasts_all_ports(port);

    // STEP 4: Bind callbacks to the port
    // Any ISB protocol messages will call into this handler (defined above).
    is_comm_register_port_isb_handler(port, isbDataHandler);

    // STEP 5: Enable message broadcasting
    // Request INS1_1 message at 100x startupNavDtd (this should be about 100 x 7ms = 700ms)
    is_comm_get_data(port, DID_INS_1, 0, 0, 100);

    // STEP 6: In a loop, process and parse messages from the port.
    // This should run a fairly fast rate, (1ms is typical) to avoid data from filling
    // the COMM buffer, which could lead to data drop.
    while (portIsOpened(port)) {
        is_comm_port_parse_messages(port);
        SLEEP_MS(1);
    }

}

