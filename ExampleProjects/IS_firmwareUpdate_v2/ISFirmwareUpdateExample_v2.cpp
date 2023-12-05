/*
MIT LICENSE

Copyright (c) 2014-2023 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include <stdio.h>
#include <vector>
#include <string>
#include <algorithm>

// STEP 1: Add Includes
// Change these include paths to the correct paths for your project
#include "../../src/ISComm.h"
#include "../../src/serialPortPlatform.h"
#include "../../src/ISBootloaderThread.h"
#include "../../src/ISBootloaderBase.h"
#include "../../src/ISSerialPort.h"

#include "../../src/InertialSense.h"
#include "../../src/protocol/FirmwareUpdate.h"
#include "../../src/ISLogger.h"

using namespace ISBootloader;
using namespace std;

#define MAX_FILE_SIZE_DISK_PERCENT_50	.5
#define MAX_FILE_SIZE_100k				100000

bool setupCommunicationsDIDs(InertialSense& inertialSenseInterface)
{
    inertialSenseInterface.StopBroadcasts();	// Stop streaming any prior messages

    // ask for device info every 2 seconds
    inertialSenseInterface.BroadcastBinaryData(DID_DEV_INFO, 2000);

    return true;
}

// print out upload progress
static is_operation_result uploadProgress(void* obj, float pct)
{
    if (obj == NULL) return IS_OP_OK;

    cISBootloaderBase* ctx = (cISBootloaderBase*)obj;
    int percent = (int)(pct * 100.0f);
    printf("\rUpload Progress: %d%%\r", percent);
    ctx->m_update_progress = percent;

    return IS_OP_OK;
}

// print out verify progress
static is_operation_result verifyProgress(void* obj, float pct)
{
    if (obj == NULL) return IS_OP_OK;

    cISBootloaderBase* ctx = (cISBootloaderBase*)obj;
    int percent = (int)(pct * 100.0f);
    printf("\rVerify Progress: %d%%\r", percent);
    ctx->m_verify_progress = percent;

    return IS_OP_OK;
}

static void statusText(void* obj, eLogLevel level, const char* info, ...)
{
    if (obj == NULL) return;

    cISBootloaderBase* ctx = (cISBootloaderBase*)obj;

    if (ctx->m_sn != 0 && ctx->m_port_name.size() != 0)
    {
        printf("%s (SN%d):\r", ctx->m_port_name.c_str(), ctx->m_sn);
    }
    else if(ctx->m_sn != 0)
    {
        printf("(SN%d):\r", ctx->m_sn);
    }
    else if (ctx->m_port_name.size() != 0)
    {
        printf("%s:\r", ctx->m_port_name.c_str());
    }
    else
    {
        printf("SN?:\r");
    }

    printf("\t\t\t%s\r\n", info);
}

// [C++ COMM INSTRUCTION] Handle received data 
static void example_dataCallback(InertialSense* i, p_data_t* data, int pHandle)
{

    (void)i;
    (void)pHandle;

    // Print data to terminal
    printf("HDR_ID: %d\r\n", data->hdr.id);

}

static void fwUpdateProgress(void* obj, const std::string stepName, int step, int steps, float pct) {
    printf(" [%s] Step %d of %d: %5.1f %%\n", stepName.c_str(), step, steps, pct * 100.0f);
}

static void fwUpdateStatus(void* obj, int level, const char* info, ...) {
    static char buffer[256];

    va_list ap;
    va_start(ap, info);
    vsnprintf(buffer, sizeof(buffer) - 1, info, ap);
    va_end(ap);

    printf("%s\n", buffer);
}

/**
 * TODO: This is a temporary method demonstrating direct DFU Firmare Updates.  Eventually this should be integrated into the normal FirmwareUpdate_V2 example, by using a DFU-based target.
 * This is primarily here as a lightweight testing/development tool while the DFUFirmwareUpdater is a WORK-IN-PROGRESS.
 * @param argc
 * @param argv
 * @return
 */
static int doDFUFirmwareUpdate(int argc, char* argv[]) {
    std::vector<dfu::DFUDevice*> devices;
    libusb_init(NULL);
    int result = dfu::DFU_ERROR_NONE;

    int count = dfu::ISDFUFirmwareUpdater::getAvailableDevices(devices);
    for (auto device : devices) {
        device->setStatusCb(fwUpdateStatus);
        device->setProgressCb(fwUpdateProgress);
        if (md5_matches(device->getFingerprint(), dfu::DFU_FINGERPRINT_STM32L4)) {
            if (result == dfu::DFU_ERROR_NONE)
                result = device->updateFirmware("/home/kylemallory/is-zephyr/is-gpx/is-common/cpp/hdw-src/IMX-5/Release/IS_IMX-5.bin", 0x08000000 + 24576);
            if (result == dfu::DFU_ERROR_NONE)
                result = device->updateFirmware("/home/kylemallory/is-zephyr/is-gpx/is-common/cpp/hdw-src/bootloader/bootloader-STM32L4/bootloader/Release/IS_bootloader-STM32L4.hex");
            if (result == dfu::DFU_ERROR_NONE)
                device->finalizeFirmware();

            fwUpdateStatus(nullptr, IS_LOG_LEVEL_INFO, "Firmware update finished with status: %d", result);
        }
        if (md5_matches(device->getFingerprint(), dfu::DFU_FINGERPRINT_STM32U5)) {
            // NOTE THAT GPX BOOTLOADER/FIRMWARE .hex files are ALWAYS configured to write to the correct memory location
            result = dfu::DFU_ERROR_NONE;
            if (result == dfu::DFU_ERROR_NONE)
                result = device->updateFirmware("/home/kylemallory/is-zephyr/is-gpx/scripts/IS_GPX-1_zephyr_v2.0.0.10_b376_2023-12-01_104805.signed.bin", 0x08084000);
            if (result == dfu::DFU_ERROR_NONE)
                result = device->updateFirmware("/home/kylemallory/is-zephyr/is-gpx/scripts/IS_GPX-1_mcuboot_v2.0.0.10_b376_2023-12-01_104805.bin");
            if (result == dfu::DFU_ERROR_NONE)
                result = device->finalizeFirmware();

            fwUpdateStatus(nullptr, IS_LOG_LEVEL_INFO, "Firmware update finished with status: %d", result);
        }
    }
    return 0;
}

int main(int argc, char* argv[])
{
    fwUpdate::update_status_e status;
    string COMNum = "/dev/ttyACM1";
    string fileName = "../../../Firmware/IS_GPX-1_zephyr_v2.0.0.6_b197_2023-09-18_120501.encrypted.bin";
    uint32_t baudRate = IS_BAUDRATE_921600;
    int deviceIndex = -1;

    if ((argc == 2) && !strcmp(argv[1], "--dfu"))
    {
        return doDFUFirmwareUpdate(argc, argv);
    }

    // check if we are using a static COM port
    if (argc == 2)
    {
        COMNum = argv[1];
    }

    // print COM port to console
    printf("COM port: %s\r\n", COMNum.c_str());

    // [C++ COMM INSTRUCTION] STEP 1: Instantiate InertialSense Class
    // Create InertialSense object, passing in data callback function pointer.
    InertialSense inertialSenseInterface(NULL);

    // Disable device response requirement to validate open port
    inertialSenseInterface.EnableDeviceValidation(false);

    // [C++ COMM INSTRUCTION] STEP 2: Open serial port
    if (!inertialSenseInterface.Open(COMNum.c_str(), baudRate, true))
    {
        cout << "Failed to open serial port at " << COMNum.c_str() << endl;
        return -1;	// Failed to open serial port
    }
    else
        cout << "COM port open!\r\n";

    // [C++ COMM INSTRUCTION] STEP 3: Enable data broadcasting
    if (setupCommunicationsDIDs(inertialSenseInterface))
    {
        // [LOGGER INSTRUCTION] Setup and start data logger
        if (!inertialSenseInterface.SetLoggerEnabled(
                true,
                "",
                cISLogger::LOGTYPE_CSV,
                0,
                0,
                MAX_FILE_SIZE_DISK_PERCENT_50,
                MAX_FILE_SIZE_100k,
                ""))
        {
            cout << "Failed to setup logger!" << endl;
            inertialSenseInterface.Close();
            inertialSenseInterface.CloseServerConnection();
            return -1;
        }
        else
            cout << "Logger set!\r\n";

        try
        {
            if (!fileName.empty())
            {
                std::vector<std::string> commands;
                commands.push_back("slot=0"); // target slot-0 (only valid slot for GPX)
                commands.push_back("timeout=10000"); // set the timeout to 10 seconds (default is 15 sec)
                commands.push_back("upload=" + fileName); // upload image to device
                commands.push_back("reset"); // reset device

                if (inertialSenseInterface.updateFirmware(
                        COMNum, // COM port
                        baudRate, // baud rate
                        fwUpdate::TARGET_DFU_GPX1, // Target GPX (using DFU)
                        commands, // vector of strings, commands to run when performing the update
                        uploadProgress,
                        verifyProgress,
                        statusText,
                        NULL) != IS_OP_OK)
                {
                    inertialSenseInterface.Close();
                    inertialSenseInterface.CloseServerConnection();
                    return -1;
                }
                else
                    cout << "Logger set!\r\n";


                // get device index assignment for our com number
                deviceIndex = inertialSenseInterface.getUpdateDeviceIndex(COMNum.c_str());

                // check if we got a valid index
                if (deviceIndex >= 0)
                {
                    // Main loop. Could be in separate thread if desired.
                    do
                    {
                        // [C++ COMM INSTRUCTION] STEP 4: Read data
                        if (!inertialSenseInterface.Update())
                        {	// device disconnected, exit
                            cout << "Device disconnected!\r\n";
                            break;
                        }

                        status = inertialSenseInterface.getUpdateStatus(deviceIndex);

                    } while (status >= fwUpdate::NOT_STARTED && status < fwUpdate::FINISHED);

                    cout << "Finished!\r\n";
                }
                else
                    cout << "Bad device index!\r\n";
            }
            else
                cout << "No file provided!!";
        }
        catch (...)
        {
            cout << "Exception occured!!";
        }
    }
    else
        cout << "Failed to set broadcast DIDs!\r\n";

    // [C++ COMM INSTRUCTION] STEP 5: Close interface
    // Close cleanly to ensure serial port and logging are shutdown properly.  (optional)
    inertialSenseInterface.Close();
    inertialSenseInterface.CloseServerConnection();

    return 0;
}