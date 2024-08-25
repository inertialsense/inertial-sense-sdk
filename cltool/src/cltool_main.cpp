/*
MIT LICENSE

Copyright (c) 2014-2024 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

/*
*  main.cpp
*
*  (c) 2014 Inertial Sense, LLC
*
*  The Inertial Sense Command Line Tool (cltool) shows how easy it is to communicate with the uINS, log data, update firmware and more.
*
*  The following keywords are found within this file to identify instructions
*  for SDK implementation.  
*
*    KEYWORD:                  SDK Implementation
*    [C++ COMM INSTRUCTION]    - C++ binding API - InertialSense class with binary communication 
*                                protocol and serial port support for Linux and Windows.
*    [C COMM INSTRUCTION]      - C binding API - Com Manager with binary communication protocol.
*    [LOGGER INSTRUCTION]      - Data logger.
*    [BOOTLOADER INSTRUCTION]  - Firmware update feature.
*
*  This app is designed to be compiled in Linux and Windows.  When using MS
*  Visual Studio IDE, command line arguments can be supplied by right clicking 
*  the project in solution explorer and then selecting properties -> debugging -> command line arguments
*/

#include <signal.h>

// Contains command line parsing and utility functions.  Include this in your project to use these utility functions.
#include "cltool.h"
#include "protocol_nmea.h"
#include "util/natsort.h"

using namespace std;

#define XMIT_CLOSE_DELAY_MS    1000     // (ms) delay prior to cltool close to ensure data transmission

static bool g_killThreadsNow = false;
int g_devicesUpdating = 0;

static void display_server_client_status(InertialSense* i, bool server=false, bool showMessageSummary=false, bool refreshDisplay=false)
{
    if (g_inertialSenseDisplay.GetDisplayMode() == cInertialSenseDisplay::DMODE_QUIET ||
        g_inertialSenseDisplay.GetDisplayMode() == cInertialSenseDisplay::DMODE_SCROLL)
    {
        return;
    }

    static float serverKBps = 0;
    static uint64_t serverByteCount = 0;
    static uint64_t serverByteRateTimeMsLast = 0;
    static uint64_t serverByteCountLast = 0;
    static stringstream outstream;

    uint64_t newServerByteCount = i->ClientServerByteCount();
    if (serverByteCount != newServerByteCount)
    {
        serverByteCount = newServerByteCount;

        // Data rate of server bytes
        uint64_t timeMs = getTickCount();
        uint64_t dtMs = timeMs - serverByteRateTimeMsLast;
        if (dtMs >= 1000)
        {
            uint64_t serverBytesDelta = serverByteCount - serverByteCountLast;
            serverKBps = ((float)serverBytesDelta / (float)dtMs);

            // Update history
            serverByteCountLast = serverByteCount;
            serverByteRateTimeMsLast = timeMs;
        }

        outstream.str("");    // clear
        outstream << "\n";
        if (server)
        {
            outstream << "Server: " << i->TcpServerIpAddressPort()   << "     Tx: ";
        }
        else
        {
            outstream << "Client: " << i->ClientConnectionInfo()     << "     Rx: ";
        }
        outstream << fixed << setw(3) << setprecision(1) << serverKBps << " KB/s, " << (long long)i->ClientServerByteCount() << " bytes    \n";

        if (server)
        {   // Server
            outstream << "Connections: " << i->ClientConnectionCurrent() << " current, " << i->ClientConnectionTotal() << " total    \n";
            if (showMessageSummary)
            {
                outstream << i->ServerMessageStatsSummary();
            }
            refreshDisplay = true;
        }
        else
        {   // Client
            is_comm_instance_t* comm = comManagerGetIsComm(0);
            if (comm != NULLPTR && comm->rxErrorCount>2)
            {
                outstream << "Com errors: " << comm->rxErrorCount << "     \n";
            }
            if (showMessageSummary)
            {
                outstream << i->ClientMessageStatsSummary();
            }
        }
    }

    if (refreshDisplay)
    {
        cout << outstream.str();
    }
}

static void display_logger_status(InertialSense* i, bool refreshDisplay=false)
{
	if (!i || !refreshDisplay)
	{
		return;
	}

	cISLogger &logger = *(i->Logger());

	if (!logger.Enabled())
	{
		return;
	}

    float logSize = logger.LogSizeAll() * 0.001;
    if (logSize < 5000.0)
        printf("\nLogging %.1f MB to: %s\n", logSize, logger.LogDirectory().c_str());
    else
        printf("\nLogging %.2f KB to: %s\n", logSize * 0.001f, logger.LogDirectory().c_str());
}

static int cltool_errorCallback(unsigned int port, is_comm_instance_t* comm)
{
    #define BUF_SIZE    8192
    #define BLACK   "\u001b[30m"
    #define RED     "\u001b[31m"
    #define GREEN   "\u001b[32m"
    #define YELLOW  "\u001b[33m"
    #define BLUE    "\u001b[34m"
    #define MAGENTA "\u001b[35m"
    #define CYAN    "\u001b[36m"
    #define WHITE   "\u001b[37m"
    #define RESET   "\u001b[0m"

    if (g_commandLineOptions.displayMode != cInertialSenseDisplay::DMODE_RAW_PARSE)
        return 0;

    static const char* errorNames[] = {
            "INVALID_PREAMBLE",
            "INVALID_SIZE",
            "INVALID_CHKSUM",
            "INVALID_DATATYPE",
            "MISSING_EOS_MARKER",
            "INCOMPLETE_PACKET",
            "INVALID_HEADER",
            "INVALID_PAYLOAD",
            "RXBUFFER_FLUSHED",
            "STREAM_UNPARSEABLE",
    };

    typedef union
    {
        uint16_t ck;
        struct
        {
            uint8_t a;	// Lower 8 bits
            uint8_t b;	// Upper 8 bits
        };
    } checksum16_u;

    char buf[BUF_SIZE];
    char* ptr = buf;
    char* ptrEnd = buf + BUF_SIZE;
    int bytesPerLine = 32;

    const packet_t* rxPkt = &(comm->rxPkt);
    const uint8_t* raw_data = rxPkt->data.ptr;

    ptr += SNPRINTF(ptr, ptrEnd - ptr, BLUE "ERROR:   " RED "%s\n", errorNames[comm->rxErrorType]);

    ptr += SNPRINTF(ptr, ptrEnd - ptr, BLUE "[PREAMB] " RED "%02x %02x ", ((rxPkt->preamble >> 8) & 0xFF), (rxPkt->preamble & 0xFF));
    ptr += SNPRINTF(ptr, ptrEnd - ptr, BLUE "[Flags: " YELLOW "%u" BLUE "] " RED "%02x ", rxPkt->flags, rxPkt->flags);
    ptr += SNPRINTF(ptr, ptrEnd - ptr, BLUE "[Id: " YELLOW "%s" BLUE "] " RED "%02x ", cISDataMappings::GetDataSetName(rxPkt->dataHdr.id), rxPkt->dataHdr.id);
    ptr += SNPRINTF(ptr, ptrEnd - ptr, BLUE "[Size: " YELLOW "%u" BLUE "] " RED "%02x %02x ", rxPkt->dataHdr.size, ((rxPkt->dataHdr.size >> 8) & 0xFF), (rxPkt->dataHdr.size & 0xFF));
    ptr += SNPRINTF(ptr, ptrEnd - ptr, BLUE "[Offset: " YELLOW "%u" BLUE "] " RED "%02x %02x", rxPkt->dataHdr.offset, ((rxPkt->dataHdr.offset >> 8) & 0xFF), (rxPkt->dataHdr.offset & 0xFF));

#if DISPLAY_DELTA_TIME==1
    static double lastTime[2] = { 0 };
	double dtMs = 1000.0*(wheel.timeOfWeek - lastTime[i]);
	lastTime[i] = wheel.timeOfWeek;
	ptr += SNPRINTF(ptr, ptrEnd - ptr, " %4.1lfms", dtMs);
#else
#endif
    ptr += SNPRINTF(ptr, ptrEnd - ptr, "\n");
    int lines = (rxPkt->dataHdr.size / bytesPerLine) + 1;
    for (int j = 0; j < lines; j++) {
        int linelen = (j == lines-1) ? rxPkt->dataHdr.size % bytesPerLine : bytesPerLine;
        if (linelen > 0) {
            if (j == 0)
                ptr += SNPRINTF(ptr, ptrEnd - ptr, BLUE "  [DATA] " RED);
            else
                ptr += SNPRINTF(ptr, ptrEnd - ptr, "         ");

            for (int i = 0; i < linelen; i++) {
                ptr += SNPRINTF(ptr, ptrEnd - ptr, "%02x ", (uint8_t) raw_data[(j * bytesPerLine) + i]);
            }
            ptr += SNPRINTF(ptr, ptrEnd - ptr, "\n");
        }
    }

    // recalculated actual checksum
    packet_buf_t *isbPkt = (packet_buf_t*)(comm->rxBuf.head);
    uint16_t payloadSize = isbPkt->hdr.payloadSize;
    uint8_t *payload = comm->rxBuf.head + sizeof(packet_hdr_t);
    checksum16_u *cksum = (checksum16_u*)(payload + payloadSize);
    int bytes_cksum = rxPkt->size - 2;
    uint16_t calcCksum = is_comm_isb_checksum16(0, comm->rxBuf.head, bytes_cksum);

    ptr += SNPRINTF(ptr, ptrEnd - ptr, BLUE "[CHKSUM] " RED "%02x %02x ", ((calcCksum >> 8) & 0xFF), (calcCksum & 0xFF));
    ptr += SNPRINTF(ptr, ptrEnd - ptr, BLUE "  Expected: " RED "%02x %02x ", ((rxPkt->checksum >> 8) & 0xFF), (rxPkt->checksum & 0xFF));

    cout << buf << RESET << endl;
    return 0;
}

// [C++ COMM INSTRUCTION] STEP 5: Handle received data 
static void cltool_dataCallback(InertialSense* i, p_data_t* data, int pHandle)
{
    if (g_commandLineOptions.outputOnceDid) {
        if (data->hdr.id != g_commandLineOptions.outputOnceDid)
            return;
    }

    (void)i;
    (void)pHandle;

    // track which DIDs we've received and when, and how frequently
    for (stream_did_t& did : g_commandLineOptions.datasets) {
        if (did.did == data->hdr.id) {
            did.rxStats.lastRxTime = current_timeMs();
            did.rxStats.rxCount++;
        }
    }

    // Print data to terminal - but only if we aren't doing a firmware update...
    if (g_devicesUpdating)
        cout.flush();
    else
        g_inertialSenseDisplay.ProcessData(data);
}


/**
 * requests any data which is not being actively received
 * @param inertialSenseInterface
 * @param datasets a vector of stream_did_t indicating the set of DIDs which should be requested and the rate to be requested at.
 * @return the number of did which were actually requested. This maybe less than the number of items in the specified dataset
 *  if those DIDs have already been recently received. If 0 is returned, it indicates that all requested DIDs are already
 *  streaming.
 */
int cltool_requestDataSets(InertialSense& inertialSenseInterface, std::vector<stream_did_t>& datasets) {
    int didRequested = 0;
    unsigned int currentTime = current_timeMs();

    for (stream_did_t& dataItem : datasets)
    {   // Datasets to stream
        if ((currentTime - dataItem.rxStats.lastRxTime) < (dataItem.periodMultiple * 500)) {
            // FIXME: the "500" above should probably be the specific base period for the DID * 5, or something
            //  basically, we want to know how often we should expect that DID, based on its base rate * the periodMultiple,
            //  and then we should re-request if we've exceeded that by, say, 3x? 5x? 10x?
            // we've already received this message recently, so its probably already been requested...
            continue; // so, let's skip it.
        }

        didRequested++;
        g_inertialSenseDisplay.SelectEditDataset(dataItem.did, true);     // Select DID for generic display

        inertialSenseInterface.BroadcastBinaryData(dataItem.did, dataItem.periodMultiple);
        switch (dataItem.did)
        {
            case DID_RTOS_INFO:
                system_command_t cfg;
                cfg.command = SYS_CMD_ENABLE_RTOS_STATS;
                cfg.invCommand = ~cfg.command;
                inertialSenseInterface.SendRawData(DID_SYS_CMD, (uint8_t*)&cfg, sizeof(system_command_t), 0);
                break;
        }
    }

    return didRequested;
}

// Where we tell the IMX what data to send and at what rate.  
// "cltool_dataCallback()" is registered as the callback functions for all received data.
// All DID messages are found in data_sets.h
static bool cltool_setupCommunications(InertialSense& inertialSenseInterface)
{
    inertialSenseInterface.StopBroadcasts();    // Stop streaming any prior messages

    if (g_commandLineOptions.asciiMessages.size() != 0)
    {
        serialPortWriteAscii(inertialSenseInterface.SerialPort(), g_commandLineOptions.asciiMessages.c_str(), (int)g_commandLineOptions.asciiMessages.size());
        return true;
    }

    if (!g_commandLineOptions.disableDeviceValidation)
    {   // check for any compatible (protocol version 2) devices
        for (int i = inertialSenseInterface.DeviceCount() - 1; i >= 0; i--) {
            if (inertialSenseInterface.DeviceInfo(i).protocolVer[0] != PROTOCOL_VERSION_CHAR0) {
                printf("ERROR: One or more connected devices are using an incompatible protocol version (requires %d.x.x.x).\n", PROTOCOL_VERSION_CHAR0);
                // let's print the dev info for all connected devices (so the user can identify the errant device)
                for (int i = inertialSenseInterface.DeviceCount() - 1; i >= 0; i--) {
                    std::string devInfo = g_inertialSenseDisplay.DataToStringDevInfo(inertialSenseInterface.DeviceInfo(i), true);
                    printf("%s\n", devInfo.c_str());
                }
                return false;
            }
        }
    }

    // ask for device info every 2 seconds
    inertialSenseInterface.BroadcastBinaryData(DID_DEV_INFO, 2000);

    // depending on command line options. stream various data sets
    if (g_commandLineOptions.datasetEdit.did)
    {   // Dataset to edit
        g_inertialSenseDisplay.SelectEditDataset(g_commandLineOptions.datasetEdit.did);
        inertialSenseInterface.BroadcastBinaryData(g_commandLineOptions.datasetEdit.did, g_commandLineOptions.datasetEdit.periodMultiple);
    }
    else
    {
        cltool_requestDataSets(inertialSenseInterface, g_commandLineOptions.datasets);
    }
    if (g_commandLineOptions.timeoutFlushLoggerSeconds > 0)
    {
        inertialSenseInterface.SetTimeoutFlushLoggerSeconds(g_commandLineOptions.timeoutFlushLoggerSeconds);
    }
    if (g_commandLineOptions.magRecal)
    {
        // Enable broadcase of DID_MAG_CAL so we can observe progress and tell when the calibration is done (i.e. DID_MAG_CAL.state: 200=in progress, 201=done).
        inertialSenseInterface.BroadcastBinaryData(DID_MAG_CAL, 100);
        // Enable mag recal
        inertialSenseInterface.SendRawData(DID_MAG_CAL, (uint8_t*)&g_commandLineOptions.magRecalMode, sizeof(g_commandLineOptions.magRecalMode), offsetof(mag_cal_t, state));
    }
    if (g_commandLineOptions.surveyIn.state)
    {   // Enable mult-axis 
        inertialSenseInterface.SendRawData(DID_SURVEY_IN, (uint8_t*)&g_commandLineOptions.surveyIn, sizeof(survey_in_t), 0);
    }
    if (g_commandLineOptions.rmcPreset)
    {
        inertialSenseInterface.BroadcastBinaryDataRmcPreset(g_commandLineOptions.rmcPreset, RMC_OPTIONS_PRESERVE_CTRL);
    }
    if (g_commandLineOptions.persistentMessages)
    {   // Save persistent messages to flash
        cout << "Sending save persistent messages." << endl;
        inertialSenseInterface.SendRaw((uint8_t*)NMEA_CMD_SAVE_PERSISTENT_MESSAGES_TO_FLASH, NMEA_CMD_SIZE);
    }
    if (g_commandLineOptions.softwareReset)
    {   // Issue software reset
        cout << "Sending software reset." << endl;
        inertialSenseInterface.SendRaw((uint8_t*)NMEA_CMD_SOFTWARE_RESET, NMEA_CMD_SIZE);
        SLEEP_MS(XMIT_CLOSE_DELAY_MS);      // Delay to allow transmit time before port closes
        return false;
    }
    if (g_commandLineOptions.sysCommand != 0)
    {   // Send system command to IMX
        cout << "Sending system command: " << g_commandLineOptions.sysCommand;
        bool manfUnlock = false;
        switch(g_commandLineOptions.sysCommand)
        {
            case SYS_CMD_ENABLE_SERIAL_PORT_BRIDGE_USB_TO_GPS1:
            case SYS_CMD_ENABLE_SERIAL_PORT_BRIDGE_USB_TO_GPS2:
            case SYS_CMD_ENABLE_SERIAL_PORT_BRIDGE_USB_TO_SER0:
            case SYS_CMD_ENABLE_SERIAL_PORT_BRIDGE_USB_TO_SER1:
            case SYS_CMD_ENABLE_SERIAL_PORT_BRIDGE_USB_TO_SER2:
            case SYS_CMD_ENABLE_SERIAL_PORT_BRIDGE_SER0_TO_GPS1:
            case SYS_CMD_ENABLE_SERIAL_PORT_BRIDGE_CUR_PORT_TO_GPS1:
            case SYS_CMD_ENABLE_SERIAL_PORT_BRIDGE_CUR_PORT_TO_GPS2:
            case SYS_CMD_ENABLE_SERIAL_PORT_BRIDGE_CUR_PORT_TO_USB:
            case SYS_CMD_ENABLE_SERIAL_PORT_BRIDGE_CUR_PORT_TO_SER0:
            case SYS_CMD_ENABLE_SERIAL_PORT_BRIDGE_CUR_PORT_TO_SER1:
            case SYS_CMD_ENABLE_SERIAL_PORT_BRIDGE_CUR_PORT_TO_SER2:
            case SYS_CMD_ENABLE_SERIAL_PORT_BRIDGE_USB_LOOPBACK:
            case SYS_CMD_ENABLE_SERIAL_PORT_BRIDGE_SER0_LOOPBACK:
            case SYS_CMD_ENABLE_SERIAL_PORT_BRIDGE_SER1_LOOPBACK:
            case SYS_CMD_ENABLE_SERIAL_PORT_BRIDGE_SER2_LOOPBACK:
            case SYS_CMD_ENABLE_SERIAL_PORT_BRIDGE_CUR_PORT_LOOPBACK:
            case SYS_CMD_ENABLE_SERIAL_PORT_BRIDGE_CUR_PORT_LOOPBACK_TESTMODE:
                cout << " Enable serial bridge"; break;
            case SYS_CMD_DISABLE_SERIAL_PORT_BRIDGE:
                cout << " Disable serial bridge"; break;
            case SYS_CMD_MANF_FACTORY_RESET:            manfUnlock = true;  cout << " Factory Reset";           break;
            case SYS_CMD_MANF_CHIP_ERASE:               manfUnlock = true;  cout << " Chip Erase";              break;
            case SYS_CMD_MANF_DOWNGRADE_CALIBRATION:    manfUnlock = true;  cout << " Downgrade Calibration";   break;
            case SYS_CMD_MANF_ENABLE_ROM_BOOTLOADER:    manfUnlock = true;  cout << " Enable ROM Bootloader";   break;
        }
        cout << endl;
        system_command_t cfg;

        if (manfUnlock)
        {
            cfg.command = SYS_CMD_MANF_UNLOCK;
            cfg.invCommand = ~cfg.command;
            inertialSenseInterface.SendRawData(DID_SYS_CMD, (uint8_t*)&cfg, sizeof(system_command_t), 0);
        }

        cfg.command = g_commandLineOptions.sysCommand;
        cfg.invCommand = ~cfg.command;
        inertialSenseInterface.SendRawData(DID_SYS_CMD, (uint8_t*)&cfg, sizeof(system_command_t), 0);
        SLEEP_MS(XMIT_CLOSE_DELAY_MS);      // Delay to allow transmit time before port closes
        return false;
    }
    if (g_commandLineOptions.platformType >= 0 && g_commandLineOptions.platformType < PLATFORM_CFG_TYPE_COUNT)
    {   // Confirm
        cout << "CAUTION!!!\n\nSetting the device(s) platform type in OTP memory.  This can only be done a limited number of times.\n\nPlatform: " << g_commandLineOptions.platformType << "\n\n";

        // Set platform type in OTP memory
        manufacturing_info_t manfInfo = {};
        manfInfo.key = 72720;
        manfInfo.platformType = g_commandLineOptions.platformType;
        // Write key (uint32_t) and platformType (int32_t), 8 bytes
        inertialSenseInterface.SendRawData(DID_MANUFACTURING_INFO, (uint8_t*)&manfInfo.key, sizeof(uint32_t)*2, offsetof(manufacturing_info_t, key));
        SLEEP_MS(XMIT_CLOSE_DELAY_MS);      // Delay to allow transmit time before port closes
        return false;
    }

    if (g_commandLineOptions.roverConnection.length() != 0)
    {
        vector<string> pieces;
        splitString(g_commandLineOptions.roverConnection, ':', pieces);
        if (pieces[0] != "TCP" &&
            pieces[0] != "SERIAL")
        {
            cout << "Invalid base connection, 1st field must be: TCP or SERIAL\n  -rover=" << g_commandLineOptions.roverConnection << endl;
            return false;
        }
        if (pieces[1] != "RTCM3" &&
            pieces[1] != "IS" &&
            pieces[1] != "UBLOX")
        {
            cout << "Invalid base connection, 2nd field must be: RTCM3, UBLOX, or IS\n  -rover=" << g_commandLineOptions.roverConnection << endl;
            return false;
        }

        if (!inertialSenseInterface.OpenConnectionToServer(g_commandLineOptions.roverConnection))
        {
            cout << "Failed to connect to server (base)." << endl;
        }
    }
    if (g_commandLineOptions.flashCfg.length() != 0)
    {
        return cltool_updateFlashCfg(inertialSenseInterface, g_commandLineOptions.flashCfg);
    }
    return true;
}

std::vector<ISBootloader::cISBootloaderBase*> firmwareProgressContexts;

is_operation_result bootloadUpdateCallback(void* obj, float percent);
is_operation_result bootloadVerifyCallback(void* obj, float percent);

static int cltool_updateFirmware()
{
    // [BOOTLOADER INSTRUCTION] Update firmware
    if (g_commandLineOptions.updateBootloaderFilename.size() > 0)
    {
        cout << "Checking bootloader firmware:  " << g_commandLineOptions.updateBootloaderFilename << endl;
    }
    cout << "Updating application firmware: " << g_commandLineOptions.updateAppFirmwareFilename << endl;

    firmwareProgressContexts.clear();
    if(InertialSense::BootloadFile(
            g_commandLineOptions.comPort,
            0,
            g_commandLineOptions.updateAppFirmwareFilename,
            g_commandLineOptions.updateBootloaderFilename,
            g_commandLineOptions.forceBootloaderUpdate,
            g_commandLineOptions.baudRate,
            bootloadUpdateCallback,
            (g_commandLineOptions.bootloaderVerify ? bootloadVerifyCallback : 0),
            cltool_bootloadUpdateInfo,
            cltool_firmwareUpdateWaiter
    ) == IS_OP_OK) return 0;
    return -1;
}

std::mutex print_mutex;

void printProgress()
{
    print_mutex.lock();

    int divisor = 0;
    float total = 0.0f;

    cISBootloaderThread::m_ctx_mutex.lock();

    for (size_t i = 0; i < cISBootloaderThread::ctx.size(); i++)
    {
        if (cISBootloaderThread::ctx[i] && cISBootloaderThread::ctx[i]->m_use_progress)
        {
            divisor++;

            if (!cISBootloaderThread::ctx[i]->m_verify)
            {
                total += cISBootloaderThread::ctx[i]->m_update_progress;
            }
            else
            {
                total += cISBootloaderThread::ctx[i]->m_update_progress * 0.5f;
                total += cISBootloaderThread::ctx[i]->m_verify_progress * 0.5f;
            }
        }
    }

    cISBootloaderThread::m_ctx_mutex.unlock();

    if (divisor)
    {
        total /= divisor;
        int display = (int)(total * 100);

        // Print progress in condensed format.
        static int displayLast = 0;
#define DISPLAY_RES    5
        if (display == displayLast && display!=0)
        {
            printf("%d%% ", display);
        }
        fflush(stdout);

        while (display < displayLast)
        {   // Decrement
            displayLast -= DISPLAY_RES;
        }
        while (display >= displayLast)
        {   // Increment
            displayLast += DISPLAY_RES;
        }
    }

    print_mutex.unlock();
}

is_operation_result bootloadUpdateCallback(void* obj, float percent)
{
    if(obj)
    {
        ISBootloader::cISBootloaderBase* ctx = (ISBootloader::cISBootloaderBase*)obj;
        ctx->m_update_progress = percent;
    }
    return g_killThreadsNow ? IS_OP_CANCELLED : IS_OP_OK;
}

is_operation_result bootloadVerifyCallback(void* obj, float percent)
{
    if(obj)
    {
        ISBootloader::cISBootloaderBase* ctx = (ISBootloader::cISBootloaderBase*)obj;
        ctx->m_verify_progress = percent;
    }

    return g_killThreadsNow ? IS_OP_CANCELLED : IS_OP_OK;
}

void cltool_bootloadUpdateInfo(void* obj, ISBootloader::eLogLevel level, const char* str, ...)
{
    print_mutex.lock();
    static char buffer[256];

    va_list ap;
    va_start(ap, str);
    vsnprintf(buffer, sizeof(buffer) - 1, str, ap);
    va_end(ap);

    if(obj == NULL)
    {
        cout << buffer << endl;
        print_mutex.unlock();
        return;
    }

    ISBootloader::cISBootloaderBase* ctx = (ISBootloader::cISBootloaderBase *)obj;

    if (ctx->m_sn != 0 && ctx->m_port_name.size() != 0)
    {
        printf("    | %s (SN%d):", ctx->m_port_name.c_str(), ctx->m_sn);
    }
    else if(ctx->m_sn != 0)
    {
        printf("    | (SN%d):", ctx->m_sn);
    }
    else if (ctx->m_port_name.size() != 0)
    {
        printf("    | %s:", ctx->m_port_name.c_str());
    }
    else
    {
        printf("    | SN?:");
    }

    if (buffer[0])
        printf(" %s", buffer);

    printf("\r\n");

    print_mutex.unlock();
}

void cltool_firmwareUpdateInfo(void* obj, ISBootloader::eLogLevel level, const char* str, ...)
{
    print_mutex.lock();
    static char buffer[256];

    memset(buffer, 0, sizeof(buffer));
    if (str) {
        va_list ap;
        va_start(ap, str);
        vsnprintf(buffer, sizeof(buffer) - 1, str, ap);
        va_end(ap);
    }

    if(obj == NULL) {
        cout << buffer << endl;
    } else {
        ISFirmwareUpdater *fwCtx = (ISFirmwareUpdater *) obj;
        if (buffer[0] || (((g_commandLineOptions.displayMode != cInertialSenseDisplay::DMODE_QUIET) && (fwCtx->fwUpdate_getSessionStatus() == fwUpdate::IN_PROGRESS)))) {
            printf("[%5.2f] [%s:SN%07d > %s]", current_timeMs() / 1000.0f, fwCtx->portName, fwCtx->devInfo->serialNumber, fwCtx->fwUpdate_getSessionTargetName());
            if (fwCtx->fwUpdate_getSessionStatus() == fwUpdate::IN_PROGRESS) {
                int tot = fwCtx->fwUpdate_getTotalChunks();
                int num = fwCtx->fwUpdate_getNextChunkID();
                float percent = num / (float) (tot) * 100.f;
                printf(" :: Progress %d/%d (%0.1f%%)", num, tot, percent);
            }
            if (buffer[0])
                printf(" :: %s", buffer);
            printf("\n");
        }
    }

    print_mutex.unlock();
}

void cltool_firmwareUpdateWaiter()
{
    printProgress();
}

static int cltool_createHost()
{
    InertialSense inertialSenseInterface;
    if (!inertialSenseInterface.Open(g_commandLineOptions.comPort.c_str(), g_commandLineOptions.baudRate))
    {
        cout << "Failed to open serial port at " << g_commandLineOptions.comPort.c_str() << endl;
        return -1;
    }
    else if (g_commandLineOptions.flashCfg.length() != 0 && !cltool_updateFlashCfg(inertialSenseInterface, g_commandLineOptions.flashCfg))
    {
        cout << "Failed to update flash config" << endl;
        return -1;
    }
    else if (!inertialSenseInterface.CreateHost(g_commandLineOptions.baseConnection))
    {
        cout << "Failed to create host at " << g_commandLineOptions.baseConnection << endl;
        return -1;
    }

    inertialSenseInterface.StopBroadcasts();

    unsigned int timeSinceClearMs = 0, curTimeMs;
    while (!g_inertialSenseDisplay.ExitProgram())
    {
        inertialSenseInterface.Update();
        curTimeMs = current_timeMs();
        bool refresh = false;
        if (curTimeMs - timeSinceClearMs > 2000 || curTimeMs < timeSinceClearMs)
        {   // Clear terminal
            g_inertialSenseDisplay.Clear();
            timeSinceClearMs = curTimeMs;
            refresh = true;
        }
        g_inertialSenseDisplay.Home();
        cout << g_inertialSenseDisplay.Hello();
		display_logger_status(&inertialSenseInterface, refresh);
        display_server_client_status(&inertialSenseInterface, true, true, refresh);
    }
    cout << "Shutting down..." << endl;

    // close the interface cleanly, this ensures serial port and any logging are shutdown properly
    // Note the following two lines are called by the InertialSense destructor, which will be called when this function exits
    // inertialSenseInterface.Close();
    // inertialSenseInterface.CloseServerConnection();

    return 0;
}

static int cltool_dataStreaming()
{
    // [C++ COMM INSTRUCTION] STEP 1: Instantiate InertialSense Class
    // Create InertialSense object, passing in data callback function pointer.
    InertialSense inertialSenseInterface(cltool_dataCallback);
    inertialSenseInterface.setErrorHandler(cltool_errorCallback);
    inertialSenseInterface.EnableDeviceValidation(!g_commandLineOptions.disableDeviceValidation);

    // [C++ COMM INSTRUCTION] STEP 2: Open serial port
    if (!inertialSenseInterface.Open(g_commandLineOptions.comPort.c_str(), g_commandLineOptions.baudRate, g_commandLineOptions.disableBroadcastsOnClose))
    {
        cout << "Failed to open serial port at " << g_commandLineOptions.comPort.c_str() << endl;
        return -1;    // Failed to open serial port
    }

    if (g_commandLineOptions.list_devices) {
        struct nat_cmp {
            bool operator()(const std::string& s1, const std::string& s2) const {
                return (utils::natcmp(s1, s2) <= 0);
            }
        };
        std::map<std::string, std::string, nat_cmp> portDevices;
        int maxPortLen = 0;
        for (auto d : inertialSenseInterface.getDevices()) {
            if (ENCODE_DEV_INFO_TO_HDW_ID(d.devInfo) != 0) {
                std::string port(d.serialPort.port);
                if (d.devInfo.firmwareVer[3] == 0) {
                    portDevices[port] = utils::string_format("SN%u, %s-%d.%d (fw%d.%d.%d %d%c)",
                                                                          d.devInfo.serialNumber,
                                                                          g_isHardwareTypeNames[d.devInfo.hardwareType], d.devInfo.hardwareVer[0], d.devInfo.hardwareVer[1],
                                                                          d.devInfo.firmwareVer[0], d.devInfo.firmwareVer[1], d.devInfo.firmwareVer[2],
                                                                          d.devInfo.buildNumber, d.devInfo.buildType);
                } else {
                    portDevices[port] = utils::string_format("SN%u, %s-%d.%d (fw%d.%d.%d.%d %d%c)",
                                                                          d.devInfo.serialNumber,
                                                                          g_isHardwareTypeNames[d.devInfo.hardwareType], d.devInfo.hardwareVer[0], d.devInfo.hardwareVer[1],
                                                                          d.devInfo.firmwareVer[0], d.devInfo.firmwareVer[1], d.devInfo.firmwareVer[2], d.devInfo.firmwareVer[3],
                                                                          d.devInfo.buildNumber, d.devInfo.buildType);
                }
                maxPortLen = std::max<int>(maxPortLen, (int)strlen(d.serialPort.port));
            }
        }
        for (auto i : portDevices) {
            printf("%s --> %s\n", i.first.c_str(), i.second.c_str());
        }
        return 0;
    }

    int exitCode = 0;

    // [C++ COMM INSTRUCTION] STEP 3: Enable data broadcasting
    if (cltool_setupCommunications(inertialSenseInterface))
    {
        if (g_commandLineOptions.displayMode == cInertialSenseDisplay::DMODE_RAW_PARSE) {
            g_inertialSenseDisplay.showRawData(true);
        }

        // [LOGGER INSTRUCTION] Setup and start data logger
        if (g_commandLineOptions.asciiMessages.size() == 0 && !cltool_setupLogger(inertialSenseInterface))
        {
            cout << "Failed to setup logger!" << endl;
            // Note the following two lines are called by the InertialSense destructor, which will be called when this function exits
            // inertialSenseInterface.Close();
            // inertialSenseInterface.CloseServerConnection();
            return -1;
        }
        try
        {
            if ((g_commandLineOptions.updateFirmwareTarget != fwUpdate::TARGET_HOST) && !g_commandLineOptions.fwUpdateCmds.empty()) {
                if(inertialSenseInterface.updateFirmware(
                        g_commandLineOptions.comPort,
                        g_commandLineOptions.baudRate,
                        g_commandLineOptions.updateFirmwareTarget,
                        g_commandLineOptions.fwUpdateCmds,
                        bootloadUpdateCallback,
                        (g_commandLineOptions.bootloaderVerify ? bootloadVerifyCallback : 0),
                        cltool_firmwareUpdateInfo,
                        cltool_firmwareUpdateWaiter
                ) != IS_OP_OK) {
                    // Note the following two lines are called by the InertialSense destructor, which will be called when this function exits
                    // inertialSenseInterface.Close();
                    // inertialSenseInterface.CloseServerConnection();
                    return -1;
                };
            }

            if (g_commandLineOptions.evFCont.sendEVF)
                inertialSenseInterface.SetEventFilter(g_commandLineOptions.evFCont.dest, 
                    g_commandLineOptions.evFCont.evFilter.eventMask.msgTypeIdMask,  
                    g_commandLineOptions.evFCont.evFilter.portMask,
                    g_commandLineOptions.evFCont.evFilter.eventMask.priorityLevel);

            // Main loop. Could be in separate thread if desired.
            uint32_t startTime = current_timeMs();
            uint32_t nextDataSetCheck = current_timeMs() + 250;
            while (!g_inertialSenseDisplay.ExitProgram())
            {
                g_inertialSenseDisplay.GetKeyboardInput();

                if (g_inertialSenseDisplay.UploadNeeded())
                {
                    cInertialSenseDisplay::edit_data_t *edata = g_inertialSenseDisplay.EditData();
                    inertialSenseInterface.SendData(edata->did, edata->data, edata->info.dataSize, edata->info.dataOffset);
                }

                if (current_timeMs() > nextDataSetCheck) {
                    if (cltool_requestDataSets(inertialSenseInterface, g_commandLineOptions.datasets))
                        nextDataSetCheck = current_timeMs() + 1000; // still waiting on some, so check more frequently
                    else
                        nextDataSetCheck = current_timeMs() + 5000; // we've receive everythign we're waiting for, so we can back off the re-requests
                }

                // [C++ COMM INSTRUCTION] STEP 4: Read data
                if (!inertialSenseInterface.Update())
                {   // device disconnected, exit
                    exitCode = -2;
                    break;
                }

                // If updating firmware, and all devices have finished, Exit
                if (g_commandLineOptions.updateFirmwareTarget != fwUpdate::TARGET_HOST) {
                    if (inertialSenseInterface.isFirmwareUpdateFinished()) {
                        exitCode = 0;
                        break;
                    }
                } else {  // Only print the usual output if we AREN'T updating firmware...
                    bool refreshDisplay = g_inertialSenseDisplay.PrintData();

                    // Collect and print summary list of client messages received
                    display_logger_status(&inertialSenseInterface, refreshDisplay);
                    display_server_client_status(&inertialSenseInterface, false, false, refreshDisplay);
                }

                // If the user specified a runDuration, then exit after that time has elapsed
                if (g_commandLineOptions.runDuration && ((current_timeMs() - startTime) > g_commandLineOptions.runDuration)) {
                    break;
                }

                // Prevent processor overload
                SLEEP_MS(1);
            }
        }
        catch (...)
        {
            cout << "Unknown exception...";
        }
    }

    //If Firmware Update is specified return an error code based on the Status of the Firmware Update
    if ((g_commandLineOptions.updateFirmwareTarget != fwUpdate::TARGET_HOST) && !g_commandLineOptions.updateAppFirmwareFilename.empty()) {
        for (auto& device : inertialSenseInterface.getDevices()) {
            if (device.fwUpdate.hasError) {
                exitCode = -3;
                break;
            }
        }
    }

    // [C++ COMM INSTRUCTION] STEP 6: Close interface
    // Close cleanly to ensure serial port and logging are shutdown properly.  (optional)
    // Note the following two lines are called by the InertialSense destructor, which will be called when this function exits
    // inertialSenseInterface.Close();
    // inertialSenseInterface.CloseServerConnection();

    return exitCode;
}

static void sigint_cb(int sig)
{
    g_killThreadsNow = true;
    cltool_bootloadUpdateInfo(NULL, ISBootloader::eLogLevel::IS_LOG_LEVEL_ERROR, "Update cancelled, killing threads and exiting...");
    signal(SIGINT, SIG_DFL);
}

static int inertialSenseMain()
{
    // clear display
    g_inertialSenseDisplay.SetDisplayMode((cInertialSenseDisplay::eDisplayMode)g_commandLineOptions.displayMode);
    g_inertialSenseDisplay.SetKeyboardNonBlocking();

    // if replay data log specified on command line, do that now and return
    if (g_commandLineOptions.replayDataLog)
    {
        // [REPLAY INSTRUCTION] 1.) Replay data log
        return cltool_replayDataLog();
    }
        // if app firmware was specified on the command line, do that now and return
    else if ((g_commandLineOptions.updateFirmwareTarget == fwUpdate::TARGET_HOST) && (g_commandLineOptions.updateAppFirmwareFilename.length() != 0))
    {
        // FIXME: {{ DEPRECATED }} -- This is the legacy update method (still required by the uINS3 and IMX-5, but will go away with the IMX-5.1)
        signal(SIGINT, sigint_cb);
        return cltool_updateFirmware();
    }
    else if (g_commandLineOptions.updateBootloaderFilename.length() != 0)
    {
        cout << "option -uf [FILENAME] must be used with option -ub [FILENAME] " << endl;
        return -1;
    }
        // if host was specified on the command line, create a tcp server
    else if (g_commandLineOptions.baseConnection.length() != 0)
    {
        return cltool_createHost();
    }
    else if (g_commandLineOptions.asciiMessages.size() != 0)
    {
        serial_port_t serialForAscii;
        serialPortPlatformInit(&serialForAscii);
        serialPortOpen(&serialForAscii, g_commandLineOptions.comPort.c_str(), g_commandLineOptions.baudRate, 0);
        serialPortWriteAscii(&serialForAscii, "STPB", 4);
        serialPortWriteAscii(&serialForAscii, ("ASCB," + g_commandLineOptions.asciiMessages).c_str(), (int)(5 + g_commandLineOptions.asciiMessages.size()));
        unsigned char line[512];
        unsigned char* asciiData;
        while (!g_inertialSenseDisplay.ExitProgram())
        {
            int count = serialPortReadAsciiTimeout(&serialForAscii, line, sizeof(line), 100, &asciiData);
            if (count > 0)
            {
                printf("%s", (char*)asciiData);
                printf("\r\n");
            }
        }
    }
    else
    {   // open the device, start streaming data and logging if needed
        return cltool_dataStreaming();
    }

    return 0;
}


int main(int argc, char* argv[])
{
    // Parse command line options
    if (!cltool_parseCommandLine(argc, argv))
    {
        // parsing failed
        return -1;
    }

    g_inertialSenseDisplay.setOutputOnceDid(g_commandLineOptions.outputOnceDid);

    // InertialSense class example using command line options
    int result = inertialSenseMain();
    if (result == -1)
    {
        cltool_outputHelp();

        // Pause so user can read error
        SLEEP_MS(2000);
    }

    g_inertialSenseDisplay.ShutDown();

    return result;
}
