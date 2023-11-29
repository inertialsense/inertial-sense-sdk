/**
 * @file ISDFUFirmwareUpdater.h 
 * @brief ${BRIEF_DESC}
 *
 * @author Kyle Mallory on 11/28/23.
 * @copyright Copyright (c) 2023 Inertial Sense, Inc. All rights reserved.
 */

#ifndef IS_DFU_FIRMWAREUPDATER_H
#define IS_DFU_FIRMWAREUPDATER_H

#include "protocol/FirmwareUpdate.h"

#include "libusb.h"

class ISDFUFirmwareUpdater : public fwUpdate::FirmwareUpdateDevice {

public:

    class DFUDevice {

    };

    /**
     * Constructor to establish a connected to the specified device, and optionally validating against hdwId and serialNo
     * @param device the libusb device which identifies the connected device to update. This is NOT a libusb_device_handle! If null, this function will use to first detected DFU device which matches the hdwId and/or serialNo
     * @param hdwId the hardware id (from manufacturing info) used to identify which specific hdwType + hdwVer we should be targeting (used in validation)
     * @param serialNo the device-specific unique Id (or serial number) that is used to uniquely identify a particular device (used in validation)
     */
    ISDFUFirmwareUpdater(libusb_device* device, uint32_t hdwId, uint32_t serialNo);

    /**
     * @return Returns true if there is an active USB DFU device connection which matches the filter/validation conditions specified in the constructor.
     */
    bool isConnected();

protected:
    int findDFUDevices()


};


#endif //IS_DFU_FIRMWAREUPDATER_H
