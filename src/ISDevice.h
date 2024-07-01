/**
 * @file ISDevice.h 
 * @brief ${BRIEF_DESC}
 *
 * @author Kyle Mallory on 2/24/24.
 * @copyright Copyright (c) 2024 Inertial Sense, Inc. All rights reserved.
 */

#ifndef INERTIALSENSESDK_ISDEVICE_H
#define INERTIALSENSESDK_ISDEVICE_H

#include <memory>

#include "DeviceLog.h"
#include "protocol/FirmwareUpdate.h"
#include "ISFirmwareUpdater.h"

#include <functional>

extern "C"
{
    // [C COMM INSTRUCTION]  Include data_sets.h and com_manager.h
    #include "data_sets.h"
    #include "com_manager.h"
    #include "serialPortPlatform.h"
}


#define PRINT_DEBUG 0
#if PRINT_DEBUG
#define DEBUG_PRINT(...)    printf("L%d: ", __LINE__); printf(__VA_ARGS__)
#else
#define DEBUG_PRINT(...)
#endif

class ISFirmwareUpdater;

class ISDevice {
public:
    enum eHdwRunStates:uint8_t {
        HDW_STATE_UNKNOWN,
        HDW_STATE_BOOTLOADER,
        HDW_STATE_APP,
    };

    int portHandle = 0;
    serial_port_t serialPort = { };
    CMHANDLE cmInstance = { 0 };
    // libusb_device* usbDevice = nullptr; // reference to the USB device (if using a USB connection), otherwise should be nullptr.

    uint16_t hdwId;                         //! hardware type and version (ie, IMX-5.0)
    eHdwRunStates hdwRunState;                   //! state of hardware (running, bootloader, etc).

    dev_info_t devInfo = { };
    sys_params_t sysParams = { };
    nvm_flash_cfg_t flashCfg = { };
    unsigned int flashCfgUploadTimeMs = 0;		// (ms) non-zero time indicates an upload is in progress and local flashCfg should not be overwritten
    uint32_t flashCfgUploadChecksum = 0;
    evb_flash_cfg_t evbFlashCfg = { };
    system_command_t sysCmd = { };

    std::shared_ptr<cDeviceLog> devLogger;
    fwUpdate::update_status_e closeStatus = { };

    struct {
        float percent = 0.f;
        bool hasError = false;
        uint16_t lastSlot = 0;
        fwUpdate::target_t lastTarget = fwUpdate::TARGET_UNKNOWN;
        fwUpdate::update_status_e lastStatus = fwUpdate::NOT_STARTED;
        std::string lastMessage;

        std::vector<std::string> target_idents;
        std::vector<std::string> target_messages;
    } fwState = {};
    ISFirmwareUpdater *fwUpdater = nullptr;

    bool fwUpdateInProgress();
    bool fwUpdate();

    static ISDevice invalidRef;

    ISDevice() {
        hdwId = 0;
        hdwRunState = HDW_STATE_UNKNOWN;
        portHandle = -1;
        serialPort = {};
        sysParams.flashCfgChecksum = 0xFFFFFFFF;		// Invalidate flash config checksum to trigger sync event
    };

    ISDevice(int ph, const serial_port_t & sp) {
        hdwId = 0;
        hdwRunState = HDW_STATE_UNKNOWN;
        portHandle = ph;
        serialPort = sp;
        sysParams.flashCfgChecksum = 0xFFFFFFFF;		// Invalidate flash config checksum to trigger sync event
    }

    /**
     * Associates this IS device with the specified serial port instance. This call is used when a port is
     * known to have a device, but the device itself is not known. This will associate which ever device is
     * connected to the specified port, even if that device is different from the current instance's device.
     * @param port
     * @return true if a valid device was discovered and bound, otherwise false
     */
    bool bindDevice(serial_port_t& port);

    /**
     * Attempts to locate the port which hosts this device. This call is used to find a specific device from
     * a set of ports, by querying each available port and checking whether the device on that port matches
     * the current device information. If the device is found on one of the provided port, it is bound to
     * that port.
     * @return true if the device was found and successfully bound to the port, otherwise false.
     */
    bool findDevice(std::vector<serial_port_t&> ports);

    /**
     * Opens the bound port, and enables communication with the device.  NOTE: THIS MAYBE UNNECESSARY as the
     * port is likely already opened when initially bound. Ultimately, we want the connection opened/close
     * and the device independent of that connection, ie, device.connection.open(...)
     * @param baudRate the baud rate to open the connection at
     * @param validate performs device comms validation after opening the port; ensures that the device is alive and well
     * @return true if successfully opened, otherwise false
     */
    bool open(int baudRate, bool validate);

    /**
     * @return true if this device has an option port/connection, otherwise false
     */
    bool isOpen();

    int writeTo(const unsigned char* buf, int len);

    int readFrom(unsigned char* buf, int len);

    /**
     * Closes the current connection/port.
     */
    void close();

    /**
     * Queries the device info from the bound port, and populates devInfo.
     * @return true if the device was successfully queried, and devInfo populated, otherwise false
     */
    bool queryDeviceInfo();

    bool stopBroadcasts(bool allPorts = false);
    bool enableBroadcast(int did, int period);
    bool sendReset();

protected:
    bool handshakeISB();
    bool queryDeviceInfoISB();
    bool queryDeviceInfoDFU();
    bool validateDevice(uint32_t timeout);

    void processRxData(p_data_t* data);
    void processRxNmea(const uint8_t* msg, int msgSize);

    void getData(uint16_t did, uint16_t size, uint16_t offset, uint16_t period);

    void stepComms();

private:
    typedef std::function<void(ISDevice* device, p_data_t* data)> pfnHandleBinaryData;
    typedef void(*pfnStepLogFunction)(ISDevice* device, const p_data_t* data);
    typedef int(*pfnDeviceRmcHandler)(port_handle_t port, p_data_get_t* req);
    typedef int(*pfnDeviceGenMsgHandler)(ISDevice* device, const unsigned char* msg, int msgSize);
    typedef int(*pfnDeviceParseErrorHandler)(ISDevice* device, is_comm_instance_t* comm);

    enum queryTypes {
        QUERYTYPE_NMEA = 0,
        QUERYTYPE_ISB,
        QUERYTYPE_IBbootloader,
        QUERYTYPE_mcuBoot,
        QUERYTYPE_MAX = QUERYTYPE_mcuBoot,
    };

    std::vector<broadcast_msg_t> broadcastMsgs;

    typedef struct {
        uint8_t updated;
        uint32_t lastRxTime;
        pfnHandleBinaryData callback;
    } did_info_t;

    pfnHandleBinaryData binaryCallbackGlobal;
    std::map<int, did_info_t> binaryCallbacks;

    uint32_t m_timeMs;
    pfnDeviceRmcHandler         m_handlerRmc = NULLPTR;
    pfnDeviceGenMsgHandler      m_handlerNmea = NULLPTR;
    pfnDeviceGenMsgHandler      m_handlerUblox = NULLPTR;
    pfnDeviceGenMsgHandler      m_handlerRtcm3 = NULLPTR;
    pfnDeviceGenMsgHandler      m_handlerSpartn = NULLPTR;
    pfnDeviceParseErrorHandler  m_handlerError = NULLPTR;

    pfnStepLogFunction stepLogFunction;

    struct {
        is_comm_instance_t comm;            //! Comm instances
        uint8_t comm_buffer[PKT_BUF_SIZE];  //! Comm instance data buffer

        #if ENABLE_PACKET_CONTINUATION
        p_data_t con;       //! Continuation data for packets
        #endif
    } cmPort;
};



#endif //INERTIALSENSESDK_ISDEVICE_H
