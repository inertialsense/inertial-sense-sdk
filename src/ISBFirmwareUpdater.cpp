/**
 * @file ISBFirmwareUpdater.cpp 
 * @brief ${BRIEF_DESC}
 *
 * @author Kyle Mallory on 5/29/24.
 * @copyright Copyright (c) 2024 Inertial Sense, Inc. All rights reserved.
 */

#include "ISBFirmwareUpdater.h"
#include "ISBootloaderBase.h"
#include "ISBootloaderISB.h"
#include "InertialSense.h"


/**
 * This is an internal method used to send an update message to the host system regarding the status of the update process
 * This variation allows for printf-based string formatting
 * @param level the criticality/severity of this message (0 = Critical, 1 = Error, 2 = Warning, 3 = Info, 4 = Debug, etc)
 * @param message the actual message to be sent to the host
 * @
 * @return true if the message was sent, false if there was an error
 */
bool ISBFirmwareUpdater::fwUpdate_sendProgressFormatted(int level, const char* message, ...) {
    static char buffer[256];
    size_t msg_len = 0;

    if ((session_id == 0) || (session_status == fwUpdate::NOT_STARTED))
        return false;

    memset(buffer, 0, sizeof(buffer));
    if (message) {
        va_list ap;
        va_start(ap, message);
        msg_len = vsnprintf(buffer, sizeof(buffer) - 1, message, ap) + 1;
        va_end(ap);
    }

    if (msg_len >= sizeof(buffer)-1)
        return false;

    fwUpdate::payload_t msg;
    msg.hdr.target_device = fwUpdate::TARGET_HOST; // progress messages always go back to the host.
    msg.hdr.msg_type = fwUpdate::MSG_UPDATE_PROGRESS;
    msg.data.progress.session_id = session_id;
    msg.data.progress.status = session_status;

    // We have a few different steps that we need to monitor; transfer, erase, write, and possible verify
    // We really should update the protocol to address its shortcoming, and handle multiple states, but that would be A LOT of work
    // instead, rather than reporting actual chunks, we'll report an arbitrary number of chunks that is derived from the total steps

    msg.data.progress.totl_chunks = 3000;
    uint16_t numChunks = _MAX(0, _MIN(session_total_chunks, last_chunk_id+1));
    transferProgress = (float)numChunks / (float)session_total_chunks;
    msg.data.progress.num_chunks = (transferProgress + eraseProgress + writeProgress) * 1000;

    msg.data.progress.msg_level = level;
    msg.data.progress.msg_len = msg_len;
    msg.data.progress.message = 0;

    msg_len = fwUpdate_packPayload(build_buffer, sizeof(build_buffer), msg, buffer);
    return fwUpdate_writeToWire((fwUpdate::target_t) msg.hdr.target_device, build_buffer, msg_len);
}


/**
 * Same as fwUpdate_sendProgressFormatted() but allows overriding the progress total/num chunks for this progress message.
 * Normally, you shouldn't use this, but some update mechanisms need a way to send additional progress, such as during
 * finalization, etc.
 * @param level
 * @param total_chunks
 * @param num_chunks
 * @param message
 * @param ...
 * @return
 */
bool ISBFirmwareUpdater::fwUpdate_sendProgressFormatted(int level, int total_chunks, int num_chunks, const char* message, ...) {
    static char buffer[256];
    size_t msg_len = 0;

    if ((session_id == 0) || (session_status == fwUpdate::NOT_STARTED))
        return false;

    if (message) {
        va_list ap;
        va_start(ap, message);
        msg_len = vsnprintf(buffer, sizeof(buffer) - 1, message, ap);
        va_end(ap);
    } else
        memset(buffer, 0, sizeof(buffer));

    if (msg_len >= sizeof(buffer)-1)
        return false;

    fwUpdate::payload_t msg;
    msg.hdr.target_device = fwUpdate::TARGET_HOST; // progress messages always go back to the host.
    msg.hdr.msg_type = fwUpdate::MSG_UPDATE_PROGRESS;
    msg.data.progress.session_id = session_id;
    msg.data.progress.status = session_status;
    msg.data.progress.totl_chunks = total_chunks;
    msg.data.progress.num_chunks = _MAX(0, _MIN(total_chunks, num_chunks));
    msg.data.progress.msg_level = level;
    msg.data.progress.msg_len = msg_len;
    msg.data.progress.message = 0;

    msg_len = fwUpdate_packPayload(build_buffer, sizeof(build_buffer), msg, buffer);
    return fwUpdate_writeToWire((fwUpdate::target_t) msg.hdr.target_device, build_buffer, msg_len);
}



// this is called internally by processMessage() to do the things to do, it should also be called periodically to send status updated, etc.
bool ISBFirmwareUpdater::fwUpdate_step(fwUpdate::msg_types_e msg_type, bool processed) {
    static int nextStep = 0;

    if (session_status == fwUpdate::NOT_STARTED)
        return false;

    // printf("fwUpdate_step(): %s\n", fwUpdate_getStatusName(session_status)); fflush(stdout);

    if (session_status == fwUpdate::INITIALIZING) {
        // if we are INITIALIZING, we've successfully issued a RESET_INTO_BOOTLOADER, and we're waiting
        // for this device to become available again, but in ISbootloader mode.  Its possible, if we are
        // connected via a USB port, that we will get a new port id, so we need to scan ports to see if
        // that device becomes available through a new port. Once we have the expected SN# and the device
        // is reporting as running in HDW_STATE_BOOTLOADER, then we can advance to ready. The InertialSense
        // class should handle most of this for us, we just need to tell it to look for new ISDevices.


        SLEEP_MS(500);
        InertialSense* is = InertialSense::getLastInstance();
        if (!is)
            is = new InertialSense();

        // Wait upto 15 seconds this device to reboot into bootloader mode.
        bool foundIt = false;
        // FIXME: We're basically waiting around for the device to reboot into ISbootloader state.
        //   We want to keep looking for new devices, querying them, and hoping that one of them matches our original device
        uint32_t timeout = current_timeMs() + 15000 * 10; // For debugging
        while ( (current_timeMs() < timeout) && !foundIt ) {
            for (auto dev : is->getDevices()) {
                if (dev->devInfo.hdwRunState == HDW_STATE_BOOTLOADER) {
                    if ((dev == device) || (ENCODE_DEV_INFO_TO_UNIQUE_ID(dev->devInfo) == ENCODE_DEV_INFO_TO_UNIQUE_ID(device->devInfo))) {
                        this->device = dev; // update the device reference to use the new device (though these should actually be the same pointer)
                        if (fwUpdate_handleInitialize(lastPayload) && (session_status >= fwUpdate::INITIALIZING)) {
                            foundIt = true;
                            session_status = fwUpdate::READY;
                            return true;
                        }

                        fwUpdate_sendProgressFormatted(IS_LOG_LEVEL_ERROR, "Rediscovered %s after rebooting into bootloader; but device failed compatibility check.", device->getIdAsString().c_str());
                        session_status = fwUpdate::ERR_COMMS;
                        return true;
                    }
                }
            }
            SLEEP_MS(500);
            bool portsChanged = is->portManager.discoverPorts();
            SLEEP_MS(100);
            if (portsChanged || (is->portManager.getPortCount() > 0))
                is->deviceManager.discoverDevices(device->hdwId, 0, DeviceManager::DISCOVERY__CLOSE_PORT_ON_FAILURE | DeviceManager::DISCOVERY__FORCE_REVALIDATION);
        }

        if (!foundIt) {
            fwUpdate_sendProgressFormatted(IS_LOG_LEVEL_ERROR, "Unable to locate %s, after rebooting into bootloader.", device->getIdAsString().c_str());
            session_status = fwUpdate::ERR_COMMS;
            return true;
        }
    }

    if ((session_status == fwUpdate::READY) || (session_status == fwUpdate::IN_PROGRESS) || (session_status == fwUpdate::FINALIZING)) {
        switch (updateState) {
            case UPLOADING: // transfer
                if (transferProgress >= 1.0f)
                    updateState = ERASING;
                session_status = fwUpdate::IN_PROGRESS;
                break;
            case ERASING: // prepare for erase
                eraseState = eraseFlash_step();
                if (eraseState >= ERASE_DONE)
                    updateState = WRITING;
                break;
            case WRITING: // prepare for write
                writeState = writeFlash_step(writeTimeout);
                if (writeState >= WRITE_DONE)
                    updateState = VERIFYING;
                break;
            case VERIFYING: // waiting for write to finish
                if (doVerify) {
                    // do some verify step??
                } else {
                    updateState = UPDATE_DONE;
                    session_status = fwUpdate::FINALIZING;
                }
                break;
            case UPDATE_DONE:
                if (imgStream) { delete imgStream; imgStream = nullptr; }
                if (imgBuffer) { delete imgBuffer; imgBuffer = nullptr; }
                rebootToAPP(true);  // we should be able to keep the port open
                session_status = fwUpdate::FINALIZING;
                break;
        }
    }

#ifdef DEBUG_INFO
    char prog_msg[256];
        memset(prog_msg, 0, sizeof(prog_msg)); // clear any messages...
        if (msg->hdr.msg_type == fwUpdate::MSG_UPDATE_CHUNK)
            snprintf(prog_msg, sizeof(prog_msg), "DEV :: Received MSG %s (Chunk %d)...\n", MSG_TYPES[msg->hdr.msg_type], msg->data.chunk.chunk_id);
        else
            snprintf(prog_msg, sizeof(prog_msg), "DEV :: Received MSG %s...\n", MSG_TYPES[msg->hdr.msg_type]);
        PRINTF("%s", prog_msg);
    if (result)ISFirmware
        sendProgress(3, (const char *)prog_msg);
#endif // DEBUG_INFO

    if (++nextStep > 1000 ) {
        nextStep = 0;
    }

    if (session_status < fwUpdate::NOT_STARTED)
        fwUpdate_sendProgress();

    // printf(" %s\n", fwUpdate_getStatusName(session_status)); fflush(stdout);
    return true;
}

// called internally to perform a system reset of various severity per reset_flags (HARD, SOFT, etc)
int ISBFirmwareUpdater::fwUpdate_performReset(fwUpdate::target_t target_id, fwUpdate::reset_flags_e reset_flags) {
/*
    RESET_SOFT = 0,             // typically, a software reset (start the program over, but don't remove power or clear RAM)
    RESET_HARD = 1,             // a hard reset, in which the device is power-cycled; this may not always be possible since generally software on a device can't remove its own power
    RESET_INTO_BOOTLOADER = 2,  // indicates that the device should reset into the bootloader (this may not always be possible)
    RESET_CONFIG = 4,           // indicates that the device should clear its configuration before performing the reset (Ie, factory restart?)
    RESET_UPSTREAM = 8,         // indicates that this device should reset all of its upstream devices, in addition to itself
*/

    if (reset_flags == fwUpdate::RESET_SOFT) {
        rebootToISB();
        return 0;
    }

    if (reset_flags == fwUpdate::RESET_INTO_BOOTLOADER) {
        rebootToISB();
        return 0;
    }

    return fwUpdate::ERR_NOT_SUPPORTED;
}

// called internally (by the receiving device) to populate the dev_info_t struct for the requested device
bool ISBFirmwareUpdater::fwUpdate_queryVersionInfo(fwUpdate::target_t target_id, dev_info_t& dev_info) {
    if (get_device_info() == IS_OP_OK) {
        device->devInfo.serialNumber = m_sn;
        device->devInfo.hdwRunState = HDW_STATE_BOOTLOADER;
    }
    if ((device->port != nullptr) && (device->hdwId != 0)) {
        dev_info = device->DeviceInfo();
        return true;
    }
    return false;
}

// this initializes the system to begin receiving firmware image chunks for the target device, image slot and image size
fwUpdate::update_status_e ISBFirmwareUpdater::fwUpdate_startUpdate(const fwUpdate::payload_t& msg) {
    lastPayload = msg;
    session_image_size = msg.data.req_update.file_size;

    if (device->devInfo.hdwRunState == HDW_STATE_BOOTLOADER) {
        if (check_is_compatible()) {
            if (!imgBuffer) // don't leak memory
                imgBuffer = new ByteBuffer(session_image_size);
            if (!imgStream) // don't leak memory
                imgStream = new ByteBufferStream(*imgBuffer);
            return fwUpdate::READY;
        }
    } else if (device->devInfo.hdwRunState == HDW_STATE_APP) {
        fwUpdate_sendProgressFormatted(IS_LOG_LEVEL_INFO, "Resetting %s into Bootloader.", device->getIdAsString().c_str());
        rebootToISB();
    }
    return fwUpdate::INITIALIZING;
}

// writes the indicated block of data (of len bytes) to the target and device-specific image slot, and with the specified offset
fwUpdate::update_status_e  ISBFirmwareUpdater::fwUpdate_writeImageChunk(fwUpdate::target_t target_id, int slot_id, int offset, int len, uint8_t *data) {
    imgBuffer->insert(offset, data, len);
    return fwUpdate::IN_PROGRESS;
}

// this marks the finish of the upgrade, that all image bytes have been received, the md5 sum passed, the device can complete the requested upgrade, and perform any device-specific finalization
fwUpdate::update_status_e  ISBFirmwareUpdater::fwUpdate_finishUpdate(fwUpdate::target_t target_id, int slot_id, int flags) {

    // First: confirm our image has the correct checksum

    // Second: confirm our image is a valid ISbootloader signature/image

    // Sixth: cleanup, and return FINISHED
    if (updateState <= 4) {
        return fwUpdate::FINALIZING;
    }

    return fwUpdate::FINISHED;
    // return fwUpdate::ERR_INVALID_SESSION;
}


// called internally to transmit data to back to the host
bool ISBFirmwareUpdater::fwUpdate_writeToWire(fwUpdate::target_t target, uint8_t* buffer, int buff_len) {
    while (buff_len--)
        toHost.push_back(*buffer++);

    return true;
}


uint32_t ISBFirmwareUpdater::get_device_info()
{
    sync();
    portFlush(device->port);

    // Send command
    portWrite(device->port, (uint8_t*)":020000041000EA", 15);
    SLEEP_MS(10);

    uint8_t buf[14] = { 0 };

    // Read Version, SAM-BA Available, serial number (in version 6+) and ok (.\r\n) response
    int count = portReadTimeout(device->port, buf, 14, 1000);

    if (count < 8 || buf[0] != 0xAA || buf[1] != 0x55)
    {   // Bad read
        m_isb_major = 0;
        m_isb_minor = 0;
        m_isb_props.rom_available = 1;
        m_isb_props.processor = IS_PROCESSOR_SAMx70;
        m_isb_props.is_evb = false;
        m_sn = 0;

        fwUpdate_sendProgress(IS_LOG_LEVEL_ERROR, "(ISB) get_device_info bad read.");
        return IS_OP_ERROR;
    }

    m_isb_major = buf[2];
    m_isb_minor = (char)buf[3];
    m_isb_props.rom_available = buf[4];

    if (buf[11] == '.' && buf[12] == '\r' && buf[13] == '\n')
    {
        m_isb_props.processor = (eProcessorType)buf[5];
        m_isb_props.is_evb = buf[6];
        memcpy(&m_sn, &buf[7], sizeof(uint32_t));
    }
    else
    {
        m_sn = 0;
    }

    if (m_isb_major == 1)
    {   // version 1
        m_isb_props.app_offset = 8192;
    }
    else if (m_isb_major >= 2 && m_isb_major <= 5)
    {   // version 2, 3 (which sent v2), 4, 5
        m_isb_props.app_offset = 16384;
    }
    else if (m_isb_major >= 6)
    {   // version 6
        m_isb_props.app_offset = 24576;
    }
    else
    {
        fwUpdate_sendProgressFormatted(IS_LOG_LEVEL_ERROR, "(ISB) get_device_info invalid m_isb_major: %d", m_isb_major);
        return IS_OP_ERROR;
    }

#if PLATFORM_IS_WINDOWS
    // EvalTool and multiple bootloads under Windows 10 have issues with dropped data if verify runs too fast
    m_isb_props.verify_size = 125;
#else
    m_isb_props.verify_size = MAX_VERIFY_CHUNK_SIZE;
#endif

    return IS_OP_OK;
}


ISBFirmwareUpdater::eImageSignature ISBFirmwareUpdater::check_is_compatible()
{
    uint8_t buf[14] = { 0 };
    int count = 0;

    if (portType(device->port) & PORT_TYPE__COMM) {
        COMM_PORT(device->port)->flags |= COMM_PORT_FLAG__EXPLICIT_READ;
    }
    portFlush(device->port);
    portRead(device->port, buf, sizeof(buf));    // empty Rx buffer
    sync();

    SLEEP_MS(100);

    for (int retry=0;; retry++)
    {
        // Send command
        portFlush(device->port);
        portRead(device->port, buf, sizeof(buf));    // empty Rx buffer
        portWrite(device->port, (uint8_t*)":020000041000EA", 15);
        SLEEP_MS(10);

        // Read Version, SAM-BA Available, serial number (in version 6+) and ok (.\r\n) response
#define READ_DELAY_MS   500
        count = portReadTimeout(device->port, buf, 14, READ_DELAY_MS);

        if (count >= 8 && buf[0] == 0xAA && buf[1] == 0x55)
        {
            break;
        }

        if (retry*READ_DELAY_MS > 4000)
        {   // No response
            // FIXME: m_info_callback(NULL, IS_LOG_LEVEL_ERROR, "    | (ISB Error) (%s) check_is_compatible response missing.", m_port->port);
            fwUpdate_sendProgress(IS_LOG_LEVEL_INFO, "(ISB) check_is_compatible response missing.");
            return IS_IMAGE_SIGN_NONE;
        }
    }

    uint32_t valid_signatures = IS_IMAGE_SIGN_IMX_5p0;  // Assume IMX-5

    m_isb_major = buf[2];
    m_isb_minor = (char)buf[3];
    bool rom_available = buf[4];
    uint8_t processor = 0xFF;
    m_isb_props.is_evb = false;
    m_sn = 0;

    if (buf[11] == '.' && buf[12] == '\r' && buf[13] == '\n')
    {   // Valid packet found
        processor = (eProcessorType)buf[5];
        m_isb_props.is_evb = buf[6];
        memcpy(&m_sn, &buf[7], sizeof(uint32_t));
    }
    else
    {   // Error parsing
        char msg[200] = { 0 };
        int n = SNPRINTF(msg, sizeof(msg), "(ISB) check_is_compatible parse error:\n 0x ");
        for (int i=0; i<count; i++)
        {
            if (i%2 == 0)
            {   // Add space every other
                n += SNPRINTF(&msg[n], sizeof(msg)-n, " ");
            }
            n += SNPRINTF(&msg[n], sizeof(msg)-n, "%02x", buf[i]);
        }
        // FIXME: m_info_callback(NULL, IS_LOG_LEVEL_ERROR, msg);
        fwUpdate_sendProgress(IS_LOG_LEVEL_INFO, msg);
        return (eImageSignature)valid_signatures;
    }

    if (m_isb_major >= 6)
    {   // v6 and up has EVB detection built-in
        if (processor == IS_PROCESSOR_SAMx70)
        {
            valid_signatures |= m_isb_props.is_evb ? IS_IMAGE_SIGN_EVB_2_24K : IS_IMAGE_SIGN_UINS_3_24K;
            if (rom_available) valid_signatures |= IS_IMAGE_SIGN_ISB_SAMx70_16K | IS_IMAGE_SIGN_ISB_SAMx70_24K;
        }
        else if (processor == IS_PROCESSOR_STM32L4)
        {
            valid_signatures |= IS_IMAGE_SIGN_IMX_5p0;
            if (rom_available) valid_signatures |= IS_IMAGE_SIGN_ISB_STM32L4;
        }
    }
    else
    {
        valid_signatures |= IS_IMAGE_SIGN_EVB_2_16K | IS_IMAGE_SIGN_UINS_3_16K;
        if (rom_available) valid_signatures |= IS_IMAGE_SIGN_ISB_SAMx70_16K | IS_IMAGE_SIGN_ISB_SAMx70_24K;
    }

    if (valid_signatures == 0)
    {
        // FIXME: m_info_callback(NULL, IS_LOG_LEVEL_ERROR, "    | (ISB Error) (%s) check_is_compatible no valid signature.", m_port->port);
        fwUpdate_sendProgress(IS_LOG_LEVEL_INFO,"(ISB) check_is_compatible no valid signature.");
    }

    return (eImageSignature)valid_signatures;
}

is_operation_result ISBFirmwareUpdater::sync()
{
    static const uint8_t handshakerChar = 'U';

    // Bootloader sync requires at least 6 'U' characters to be sent every 10ms.
    // write a 'U' to handshake with the boot loader - once we get a 'U' back we are ready to go
    for (int i = 0; i < BOOTLOADER_RETRIES; i++)
    {
        if (portWrite(device->port, &handshakerChar, 1) != 1)
        {
            return IS_OP_ERROR;
        }

        if (portWaitForTimeout(device->port, &handshakerChar, 1, BOOTLOADER_RESPONSE_DELAY))
        {   // Success
            return IS_OP_OK;
        }
    }

#if defined(SUPPORT_BOOTLOADER_V5A)     // ONLY NEEDED TO SUPPORT BOOTLOADER v5a.  Delete this and associated code in Q4 2022 after bootloader v5a is out of circulation. WHJ
    static const unsigned char handshaker[] = "INERTIAL_SENSE_SYNC_DFU";

    // Attempt handshake using extended string for bootloader v5a
    for (int i = 0; i < BOOTLOADER_RETRIES; i++)
    {
        if (portWriteAndWaitForTimeout(device->port, (const unsigned char*)&handshaker, (int)sizeof(handshaker), &handshakerChar, 1, BOOTLOADER_RESPONSE_DELAY))
        {   // Success
            return IS_OP_OK;
        }
    }
#endif

    return IS_OP_ERROR;
}

/**
 * Instructs the Firmware to reset into the ROM DFU bootloader. This will generally
 * reinitialize the device, forcing a drop of the serial connection. If this device
 * is connected via USB, the device will likely enumerate as a USB DFU device, and
 * will not appear as a regular CDC/Uart serial port.
 * @return true on success, otherwise false
 */
bool ISBFirmwareUpdater::rebootToRomDfu()
{
    fwUpdate_sendProgress(IS_LOG_LEVEL_INFO, "(ISB) Rebooting to Bootloader (ROM-DFU)...");

    if (device->devInfo.hdwRunState == HDW_STATE_APP) {
        // In case we are in program mode, try and send the commands to go into bootloader mode
        for (size_t loop = 0; loop < 10; loop++) {
            if (!portWriteAscii(device->port, "STPB", 4)) break;     // If the write fails, assume the device is now in bootloader mode.
            if (!portWriteAscii(device->port, "BLEN", 4)) break;
            uint8_t c = 0;
            if (portReadCharTimeout(device->port, &c, 13) == 1) {
                if (c == '$') {
                    // done, we got into bootloader mode
                    return true;
                }
            }
            else portFlush(device->port);
        }
    } else if (device->devInfo.hdwRunState == HDW_STATE_BOOTLOADER) {
        if (portWrite(device->port, (unsigned char*)":020000040700F3", 15) == 15)
            return true;
        if (portWrite(device->port, (unsigned char*)":020000040500F5", 15) == 15)
            return true;
    }

    // we never received a valid response.  We are in an unknown state.
    return false;
}

/**
 * Instructs the Firmware to reset into the ISbootloader. The device will reinitialize
 * the serial port, forcing a drop of the serial connection. If this device is connected
 * via USB, the USB port may get re-enumerated by the oS and assigned a new port.
 * @return true on success, otherwise false
 */
bool ISBFirmwareUpdater::rebootToISB()
{
    fwUpdate_sendProgress(IS_LOG_LEVEL_INFO, "(ISB) Rebooting to Bootloader (ISbl)...");

    if (device->devInfo.hdwRunState == HDW_STATE_APP) {
        // In case we are in program mode, try and send the commands to go into bootloader mode
        for (size_t loop = 0; loop < 10; loop++) {
            if (device->SendNmea("STPB") || device->SendNmea("BLEN"))
                break;        // If the write fails, assume the device is now in bootloader mode.
            uint8_t c = 0;
            if (portReadCharTimeout(device->port, &c, 13) == 1) {
                if (c == '$') {
                    // done, we got into bootloader mode
                    return true;
                }
            }
            else portFlush(device->port);
        }
        device->disconnect();   // I think the intent here, is that we rebooted the device, and possibly got a new port - so disconnect the old one.
        device->devInfo.hdwRunState = HDW_STATE_UNKNOWN;    // clear this so we don't get confused about the state of the device.
    } else if (device->devInfo.hdwRunState == HDW_STATE_BOOTLOADER) {
        if (device->SendRaw(":020000040500F5", 15) == 15)
            return true;
    }

    // we never received a valid response.  We are in an unknown state.
    return false;
}

/**
 * Instructs the ISBootloader to boot back into APP mode. This effectively
 * sets the ISB boot flag, and then causes the bootloader to load and jump
 * to the application space. By default, this call will close the serial
 * port connection to the device once completed.
 * @param keepPortOpen if true, the port will remain open after the command is issued.
 * @return true if successful, otherwise false
 */
bool ISBFirmwareUpdater::rebootToAPP(bool keepPortOpen) {
    if (!device || !portIsOpened(device->port))
        return false;

    fwUpdate_sendProgress(IS_LOG_LEVEL_INFO, "(ISB) Rebooting to APP mode...");

    // send the "reboot to program mode" command and the device should start in program mode
    portWrite(device->port, (unsigned char*)":020000040300F7", 15);
    portFlush(device->port);
    SLEEP_MS(20);

    device->devInfo.hdwRunState = HDW_STATE_UNKNOWN;    // invalidated, because we don't know until we rediscover the device
    if (!keepPortOpen) {
        device->disconnect();
    }
    return true;
}


/**
 * Helper function which generates/appends a Intel HEX format checksum to the end of an buffer
 * This is actually a really strange function and is probably ripe with issues; or maybe I just
 * don't understand its brilliance.  In any case, take what you see here with a dash of salt.
 *
 * @param checkSum a starting checksum "seed" (this would also be the result of a previous call which was not the final call).
 * @param ptr a pointer to the data buffer
 * @param start an offset into the data buffer from which to start calculating the checksum
 * @param end an offset into the data buffer, specifying when to stop calculating the checkum
 * @param checkSumPosition an offset into the data buffer (usually the very end), where the resulting checksum will be set
 * @param finalCheckSum a bool indicating if this is the final checksum() call to be made on this buffer
 * @return
 */
int ISBFirmwareUpdater::checksum(int checkSum, uint8_t* ptr, int start, int end, int checkSumPosition, int finalCheckSum)
{
    uint8_t c1, c2;
    uint8_t* currentPtr = (uint8_t*)(ptr + start);
    uint8_t* endPtr = (uint8_t*)(ptr + end - 1);
    uint8_t b;

    while (currentPtr < endPtr)
    {
        c1 = *(currentPtr++) | 0x20;
        c1 = (c1 <= '9' ? c1 + 0xD0 : c1 + 0xA9);
        c2 = *(currentPtr++) | 0x20;
        c2 = (c2 <= '9' ? c2 + 0xD0 : c2 + 0xA9);
        b = (c1 << 4) | c2;
        checkSum += b;
    }

    if (finalCheckSum) checkSum = (uint8_t)(~checkSum + 1);
    if (checkSumPosition != 0) SNPRINTF((char*)(ptr + checkSumPosition), 3, "%.2X", checkSum);

    return checkSum;
}

bool ISBFirmwareUpdater::sendCmd(const std::string& cmd, int chksumPos) {
    if (!device || !device->port)
        return false;

    unsigned char cmdBuffer[36];
    memset(cmdBuffer, 0, sizeof(cmdBuffer));

    if (chksumPos == -1) chksumPos = cmd.length() - 2;
    memcpy(cmdBuffer, cmd.c_str(), cmd.length());
    checksum(0, cmdBuffer, 1, chksumPos, chksumPos, 1);
    return (portWrite(device->port, cmdBuffer, cmd.length()) >= 0);
}

/**
 * Helper functions which waits for a particular acknowledgement response from the remote device, and generates periodic status
 * reports while it waits.  This function calculates (and returns via 'progress') a percentage progress as an indication of
 * time elapsed between elasped and maxTimeout. The implementation for progress is non-linear (exponential).
 * @param ackStr the string to look for, which indicates an acknowledgement
 * @param progressMsg a message to report when sending periodic status updates
 * @param maxTimeout the maximum number of milliseconds to wait for the acknowledgement
 * @param elapsed the number of milliseconds that have elasped since submitting the operation that is waiting for the acknowledgement
 * @param progress  a calculated percentage (0-1.0) indicating a "progress" toward a timeout occurring
 * @return false if an acknowledgement has been received, otherwise false
 */
bool ISBFirmwareUpdater::waitForAck(const std::string& ackStr, const std::string& progressMsg, uint32_t maxTimeout, uint32_t& elapsedTime, float& progress) {
    if (!device || !device->port)
        return false;

    int count = portRead(device->port, rxWorkBufPtr, ackStr.length());
    rxWorkBufPtr += count;

    // we want to have a calculated progress which seems reasonable.
    // Since erasing flash is a non-deterministic operation (from our standpoint)
    // let's use a log algorithm that will elapse approx 75% of the progress in
    // the average time that a device takes to complete this operation (about 10 seconds)
    // the remaining 25% will slowly elapse as we get closer to the timeout period.
    progress = (float)(1.0 - std::pow((double)(maxTimeout - elapsedTime) / (double)maxTimeout, 4));
    if ((progress_interval > 0) && (nextProgressReport < current_timeMs())) {
        nextProgressReport = current_timeMs() + progress_interval;
        fwUpdate_sendProgressFormatted(IS_LOG_LEVEL_INFO, progressMsg.c_str());
    }

    while ((size_t)(rxWorkBufPtr - rxWorkBuf) >= ackStr.length()) {
        if (memcmp(rxWorkBuf, ackStr.c_str(), ackStr.length()) == 0) {
            rxWorkBufPtr = rxWorkBuf;
            progress = 1.0f;
            elapsedTime = 0;
            return true;
        }
        rxWorkBufPtr--;
        memmove(rxWorkBuf, rxWorkBuf+1, sizeof(rxWorkBufPtr)-1);
        rxWorkBuf[sizeof(rxWorkBuf)-1] = 0;
    }
    return false;
}

/**
 * Performs an incremental ERASE operation in a non-blocking fashion.  This function is intended to be
 * called multiple times, until it indicates that the operation has been completed.
 * @returns a fwUpdate::update_status_e type; a IN_PROGRESS or FINALIZING indicates that the
 *   operation is still in progress.
 */
ISBFirmwareUpdater::eraseState_t ISBFirmwareUpdater::eraseFlash_step(uint32_t timeout) {
    static const std::string SET_LOCATION = ":03000006030000F4CC";
    static const std::string ERASE_FLASH = ":0200000400FFFBCC";

    float setLocProgress = 0.0f;  // use this instead of eraseProgress in ERASE_INITIALIZE, so it doesn't 'glitch' our progress
    switch (eraseState) {
        default:
            eraseElapsed = 0;
            eraseStartedMs = 0;
            eraseState = ERASE_INITIALIZE;
            // fallthrough - any other state should reset back to the init state
        case ERASE_INITIALIZE:
            // load erase location
            if (eraseStartedMs == 0) {
                fwUpdate_sendProgress(IS_LOG_LEVEL_DEBUG, "Initializing flash erase.");
                if (sendCmd(SET_LOCATION))
                    eraseStartedMs = current_timeMs();
                SLEEP_MS(30); // give a moment for the device to respond to the command (but not too long).
            }

            eraseElapsed = current_timeMs() - eraseStartedMs;
            if (!waitForAck(".\r\n", "Erasing flash", timeout, eraseElapsed, setLocProgress)) {
                if (eraseElapsed > timeout) {
                    fwUpdate_sendProgress(IS_LOG_LEVEL_ERROR, "Error while erasing flash (timeout after set_location)");
                    session_status = fwUpdate::ERR_FLASH_WRITE_FAILURE;
                    eraseState = ERASE_TIMEOUT;
                    return eraseState;
                }
            } else {
                eraseStartedMs = 0;
                eraseState = ERASE;
            }
            break;
        case ERASE:
            // load erase location
            if (eraseStartedMs == 0) {
                fwUpdate_sendProgress(IS_LOG_LEVEL_DEBUG, "Initiating flash erase.");
                if (sendCmd(ERASE_FLASH))
                    eraseStartedMs = current_timeMs();
                SLEEP_MS(30); // give a moment for the device to respond to the command (but not too long).
            }

            eraseElapsed = current_timeMs() - eraseStartedMs;
            if (!waitForAck(".\r\n", "Erasing flash", timeout, eraseElapsed, eraseProgress)) {
                if (eraseElapsed > timeout) {
                    fwUpdate_sendProgress(IS_LOG_LEVEL_ERROR, "Error while erasing flash (timeout after erase_flash)");
                    session_status = fwUpdate::ERR_FLASH_WRITE_FAILURE;
                    eraseState = ERASE_TIMEOUT;
                }
            } else {
                eraseState = ERASE_FINALIZE;
                rxWorkBufPtr = rxWorkBuf;
            }
            break;
        case ERASE_FINALIZE:
            eraseState = ERASE_DONE;
            break;
        case ERASE_DONE:
            fwUpdate_sendProgress(IS_LOG_LEVEL_DEBUG, "Flash erase complete.");
            eraseProgress = 1.0f;
            break;
    }
    return eraseState;
}

is_operation_result ISBFirmwareUpdater::select_page(int page)
{
    // Atmel select page command (0x06) is 4 bytes and the data is always 0301xxxx where xxxx is a 16 bit page number in hex
    unsigned char changePage[24];

    // Change page
    fwUpdate_sendProgressFormatted(IS_LOG_LEVEL_DEBUG, "Selecting flash page %d.", page);
    SNPRINTF((char*)changePage, 24, ":040000060301%04XCC", page);
    checksum(0, changePage, 1, 17, 17, 1);
    if (portWriteAndWaitForTimeout(device->port, changePage, 19, (unsigned char*)".\r\n", 3, BOOTLOADER_TIMEOUT_DEFAULT) == 0)
    {
        fwUpdate_sendProgress(IS_LOG_LEVEL_ERROR, "Failed to select page");
        return IS_OP_ERROR;
    }

    return IS_OP_OK;
}

is_operation_result ISBFirmwareUpdater::begin_program_for_current_page(int startOffset, int endOffset)
{
    // Atmel begin program command is 0x01, different from standard intel hex where command 0x01 is end of file
    // After the 0x01 is a 00 which means begin writing program
    // The begin program command uses the current page and specifies two 16 bit addresses that specify where in the current page
    // the program code will be written
    unsigned char programPage[24];

    // Select offset
    fwUpdate_sendProgressFormatted(IS_LOG_LEVEL_DEBUG, "Setting \"Begin Program\" for current flash page (%d, %04X >= %04X).", currentPage, startOffset, endOffset);
    SNPRINTF((char*)programPage, 24, ":0500000100%04X%04XCC", startOffset, endOffset);
    checksum(0, programPage, 1, 19, 19, 1);
    if (portWriteAndWaitForTimeout(device->port, programPage, 21, (unsigned char*)".\r\n", 3, BOOTLOADER_TIMEOUT_DEFAULT) == 0)
    {
        fwUpdate_sendProgress(IS_LOG_LEVEL_ERROR, "(ISB) Failed to start programming page");
        return IS_OP_ERROR;
    }

    return IS_OP_OK;
}

int ISBFirmwareUpdater::is_isb_read_line(ByteBufferStream& byteStream, char line[HEX_BUFFER_SIZE])
{
    char c;
    char* currentPtr = line;
    char* endPtr = currentPtr + 1023;

    while (currentPtr != endPtr)
    {
        // read one char
        c = (char)byteStream.get();
        if (c == '\r') continue;                        // eat '\r' chars
        else if (c == '\n' || c == (char)EOF) break;    // newline char, we have a line
        *currentPtr++ = c;
    }

    *currentPtr = '\0';

    // TODO: Figure out why ARM64 bootloader hits this...
    if (currentPtr - line == 1023)
    {
        return 0;
    }
    return (int)(currentPtr - line);
}

/**
 * Formats and sends a single record (upto 255 bytes, or 510 characters) of HEX data to the
 * receiving device on the current page, and waits for confirmation back that the data was
 * successfully received.
 * @param hexData
 * @param byteCount
 * @return is_operation_result IS_OP_OK if no error, otherwise IS_OP_ERROR
 */
is_operation_result ISBFirmwareUpdater::upload_hex_page(unsigned char* hexData, uint8_t byteCount)
{
    if (byteCount == 0)
    {
        return IS_OP_OK;
    }

    // create a program request with just the hex characters that will fit on this page
    unsigned char programLine[12];
    SNPRINTF((char*)programLine, 12, ":%02X%04X00", byteCount, currentOffset);
    size_t count = strlen((char*)programLine);
    if (portWrite(device->port, programLine, count) != (int)count)
    {
        fwUpdate_sendProgress(IS_LOG_LEVEL_ERROR, "(ISB) Failed to write start page");
        return IS_OP_ERROR;
    }

    // add the previously written chars to the checksum
    int checkSum = checksum(0, programLine, 1, 9, 0, 0);

    // write all of the hex chars
    int charsForThisPage = byteCount * 2;
    if (portWrite(device->port, hexData, charsForThisPage) != charsForThisPage)
    {
        fwUpdate_sendProgress(IS_LOG_LEVEL_ERROR, "(ISB) Failed to write data to device");
        return IS_OP_ERROR;
    }

    int newVerifyChecksum = verifyCheckSum;

    // calculate verification checksum for this data
    for (int i = 0; i < charsForThisPage; i++)
    {
        newVerifyChecksum = ((newVerifyChecksum << 5) + newVerifyChecksum) + hexData[i];
    }

    checkSum = checksum(checkSum, hexData, 0, charsForThisPage, 0, 1);
    unsigned char checkSumHex[3];
    SNPRINTF((char*)checkSumHex, 3, "%02X", checkSum);

    // For some reason, the checksum doesn't always make it through to the IMX-5. Re-send until we get a response or timeout.
    // Update 8/25/22: Increasing the portReadTimeout from 10 to 100 seems to have fixed this. Still needs to be proven.
    for (int i = 0; i < 10; i++)
    {
        if (portWrite(device->port, checkSumHex, 2) != 2)
        {
            fwUpdate_sendProgress(IS_LOG_LEVEL_ERROR, "(ISB) Failed to write checksum to device");
            return IS_OP_ERROR;
        }

        SLEEP_MS(5);
        unsigned char buf[5] = { 0 };
        int count = portReadTimeout(device->port, buf, 3, 100);
        if (count == 3 && memcmp(buf, ".\r\n", 3) == 0)
        {
            break;
        }

        if (i == 9)
        {
            return IS_OP_ERROR;
        }
    }

    totalBytes += byteCount;
    currentOffset += byteCount;
    verifyCheckSum = newVerifyChecksum;

    return IS_OP_OK;
}

is_operation_result ISBFirmwareUpdater::upload_hex(unsigned char* hexData, uint16_t charCount)
{
    (void)currentPage;

    if (charCount > MAX_SEND_COUNT)
    {
        fwUpdate_sendProgress(IS_LOG_LEVEL_ERROR, "(ISB) Unexpected char count");
        return IS_OP_ERROR;
    }
    else if (charCount == 0)
    {
        return IS_OP_OK;
    }

    int byteCount = charCount / 2;

    // check if we will overrun the current page
    if (currentOffset + byteCount > FLASH_PAGE_SIZE)
    {
        int pageByteCount = FLASH_PAGE_SIZE - currentOffset;
        if ((pageByteCount < 0) || (pageByteCount > 255) || upload_hex_page(hexData, pageByteCount) != IS_OP_OK)
        {
            fwUpdate_sendProgress(IS_LOG_LEVEL_ERROR, "(ISB) Upload hex page error");
            return IS_OP_ERROR;
        }

        hexData += (pageByteCount * 2);
        charCount -= (pageByteCount * 2);
    }

    if (charCount != 0 && upload_hex_page(hexData, charCount / 2) != IS_OP_OK)
    {
        return IS_OP_ERROR;
    }

    return IS_OP_OK;
}

is_operation_result ISBFirmwareUpdater::fill_current_page()
{
    if (currentOffset < FLASH_PAGE_SIZE)
    {
        unsigned char hexData[256];
        memset(hexData, 'F', 256);

        fwUpdate_sendProgressFormatted(IS_LOG_LEVEL_DEBUG, "Filling remainder of page %d with 0xFF (%d bytes).", currentPage, FLASH_PAGE_SIZE - currentOffset);
        while (currentOffset < FLASH_PAGE_SIZE)
        {
            int byteCount = (FLASH_PAGE_SIZE - currentOffset) * 2;
            if (byteCount > 256)
            {
                byteCount = 256;
            }
            memset(hexData, 'F', byteCount);

            if (upload_hex_page(hexData, byteCount / 2) != IS_OP_OK)
            {
                fwUpdate_sendProgress(IS_LOG_LEVEL_DEBUG, "(ISB) Failed to fill page with bytes");
                return IS_OP_OK; // FIXME - this should actually be an error  return IS_OP_ERROR;

            }
        }
    }

    return IS_OP_OK;
}

/**
 * Processes an Intel hex file, one chunk/line at a time. This function is meant to be called
 * repeatedly (in a step/thread, minimal-blocking fashion) until the entire file is processed.
 * @return true if the entire file has been processed, otherwise false (repeat until true)
 */
ISBFirmwareUpdater::writeState_t ISBFirmwareUpdater::writeFlash_step(uint32_t timeout) {
    switch (writeState) {
        case WRITE_INITIALIZE:
            fwUpdate_sendProgress(IS_LOG_LEVEL_DEBUG, "Initializing flash write.");
            currentPage = 0;
            currentOffset = m_isb_props.app_offset;
            totalBytes = 0; // FIXME??  Why was this: m_isb_props.app_offset;
            verifyCheckSum = 5381;
            writeState = WRITE;
            select_page(currentPage);
            begin_program_for_current_page(m_isb_props.app_offset, FLASH_PAGE_SIZE - 1);
            break;
        case WRITE:
        {
            // if (writeStartedMs == 0) {
            //    fwUpdate_sendProgress(IS_LOG_LEVEL_DEBUG, "Initiating flash write.");
            //    if (sendCmd(WRITE_FLASH))
            //       writeStartedMs = current_timeMs();
            //       SLEEP_MS(100); // give a moment for the device to respond to the command (but not too long).
            //    }
            // }

            switch (process_hex_stream(*imgStream)) {
                case IS_OP_OK:  // normal operation, and still more data to process
                    writeState = WRITE;
                    break;
                case IS_OP_CLOSED:  // normal operation, all data sent w/ no errors
                    writeState = WRITE_FINALIZE;
                    session_status = fwUpdate::FINALIZING;
                    break;
                case IS_OP_CANCELLED:   // user cancelled the update??  (currently, no mechanism for this)
                    writeState = WRITE_DONE;
                    session_status = fwUpdate::ERR_UPDATER_CLOSED;
                    return writeState;
                case IS_OP_RETRY:   // something requested that we start over again??
                    SLEEP_MS(200);
                    writeState = WRITE_INITIALIZE;
                    break;
                case IS_OP_INCOMPATIBLE:    // data is valid, but not for this device (should not happen at this point)
                    session_status = fwUpdate::ERR_INVALID_TARGET;
                    return writeState;
                case IS_OP_ERROR:   // an error; invalid HEX data, etc
                    writeState = WRITE_ERROR;
                    session_status = fwUpdate::ERR_FLASH_WRITE_FAILURE;
                    return writeState;
            }
            break;
        }
        case WRITE_TIMEOUT:
        case WRITE_ERROR:
            fwUpdate_sendProgress(IS_LOG_LEVEL_ERROR, "Error while writing to flash.");
            // whoops...
            break;
        case WRITE_FINALIZE:
            fwUpdate_sendProgress(IS_LOG_LEVEL_DEBUG, "Finalizing flash write.");
            writeState = WRITE_DONE;
            break;
        case WRITE_DONE:
            fwUpdate_sendProgress(IS_LOG_LEVEL_DEBUG, "Flash write completed.");
            break;
    }

    writeProgress = _MIN((float)imgStream->tellg() / (float)session_image_size, 1.0f);
    if ((progress_interval > 0) && (nextProgressReport < current_timeMs())) {
        session_status = fwUpdate::IN_PROGRESS;
        fwUpdate_sendProgress(IS_LOG_LEVEL_INFO, "Writing Flash");
        nextProgressReport = current_timeMs() + progress_interval;
    }

    return writeState;
}

is_operation_result ISBFirmwareUpdater::process_hex_stream(ByteBufferStream& byteStream)
{

    int lineLength;
    // m_update_progress = 0.0f;
    char line[HEX_BUFFER_SIZE];
    int outputSize = 0;
    int pad;
    unsigned char tmp[5];
    int subOffset = 0, i = 0;

    if (lastSubOffset < 0) {
        lastSubOffset = currentOffset;
        outputPtr = output;
    }


    if ((lineLength = is_isb_read_line(byteStream, line)) == 0)
        return IS_OP_CLOSED;

    if (lineLength > 12 && line[7] == '0' && line[8] == '0') {
        if (lineLength > HEX_BUFFER_SIZE * 4) {
            fwUpdate_sendProgress(IS_LOG_LEVEL_ERROR, "(ISB) hex file line length too long");
            return IS_OP_ERROR;
        }

        // we need to know the offset that this line was supposed to be stored at so we can check if offsets are skipped
        memcpy(tmp, line + 3, 4);
        tmp[4] = '\0';
        subOffset = strtol((char*)tmp, 0, 16);

        // check if we skipped an offset, the intel hex file format can do this, in which case we need to make sure
        // that the bytes that were skipped get set to something
        if (subOffset > lastSubOffset) {
            // pad with FF bytes, this is an internal implementation detail to how the device stores unused memory
            pad = (subOffset - lastSubOffset);
            if (outputPtr + pad >= outputPtrEnd) {
                fwUpdate_sendProgress(IS_LOG_LEVEL_ERROR, "(ISB) FF padding overflowed buffer");
                return IS_OP_ERROR;
            }

            while (pad-- != 0) {
                *outputPtr++ = 'F';
                *outputPtr++ = 'F';
            }
        }

        // skip the first 9 chars which are not data, then take everything else minus the last two chars which are a checksum
        // check for overflow
        pad = lineLength - 11;
        if (outputPtr + pad >= outputPtrEnd) {
            fwUpdate_sendProgress(IS_LOG_LEVEL_ERROR, "(ISB) Line data overflowed output buffer");
            return IS_OP_ERROR;
        }

        for (i = 9; i < lineLength - 2; i++) {
            *outputPtr++ = line[i];
        }

        // set the end offset so we can check later for skipped offsets
        lastSubOffset = subOffset + ((lineLength - 11) / 2);
        outputSize = (int)(outputPtr - output);

        // we try to send the most allowed by this hex file format
        if (outputSize < MAX_SEND_COUNT) {
            // keep buffering
            return IS_OP_OK;
        }
        // upload this chunk
        if (upload_hex(output, _MIN(MAX_SEND_COUNT, outputSize)) != IS_OP_OK) {
            fwUpdate_sendProgress(IS_LOG_LEVEL_ERROR, "(ISB) Error in upload chunk");
            return IS_OP_ERROR;
        }

        outputSize -= MAX_SEND_COUNT;

        if (outputSize < 0 || outputSize > HEX_BUFFER_SIZE) {
            fwUpdate_sendProgress(IS_LOG_LEVEL_ERROR, "(ISB) Output size was too large (1)");
            return IS_OP_ERROR;
        }

        if (outputSize > 0) {
            // move the left-over data to the beginning
            memmove(output, output + MAX_SEND_COUNT, outputSize);
        }

        // reset output ptr back to the next chunk of data
        outputPtr = output + outputSize;
    } else if (strncmp(line, ":020000040", 10) == 0 && strlen(line) >= 13) {
        memcpy(tmp, line + 12, 3);      // Only support up to 10 pages currently
        tmp[1] = '\0';
        int newPage = strtol((char*)tmp, 0, 16);
        fwUpdate_sendProgressFormatted(IS_LOG_LEVEL_DEBUG, "(ISB) flash page changed (%d) in stream.", newPage);

        if (newPage == 0) {
            lastSubOffset = currentOffset = m_isb_props.app_offset;
            return IS_OP_OK;
        }

        lastSubOffset = 0;
        outputSize = (int)(outputPtr - output);

        if (outputSize < 0 || outputSize > HEX_BUFFER_SIZE) {
            fwUpdate_sendProgress(IS_LOG_LEVEL_ERROR, "(ISB) Output size was too large (2)");
            return IS_OP_ERROR;
        }

        // flush the remainder of data to the current page
        if (upload_hex(output, outputSize) != IS_OP_OK) {
            fwUpdate_sendProgress(IS_LOG_LEVEL_ERROR, "(ISB) Error in upload hex");
            return IS_OP_ERROR;
        }

        // fill remainder of the current page, the next time that bytes try to be written the page will be automatically incremented
        if (fill_current_page() != IS_OP_OK) {
            fwUpdate_sendProgress(IS_LOG_LEVEL_ERROR, "(ISB) Error in fill page");
            return IS_OP_ERROR;
        }

        // change to the next page
        currentPage = newPage;
        currentOffset = 0;
        if (select_page(currentPage) != IS_OP_OK || begin_program_for_current_page(0, FLASH_PAGE_SIZE - 1) != IS_OP_OK) {
            fwUpdate_sendProgress(IS_LOG_LEVEL_ERROR, "(ISB) Failed to issue select page or to start programming");
            return IS_OP_ERROR;
        }

        // set the output ptr back to the beginning, no more data is in the queue
        outputPtr = output;
    }
    else if (lineLength > 10 && line[7] == '0' && line[8] == '1')
    {   // End of last page (end of file marker)
        fwUpdate_sendProgress(IS_LOG_LEVEL_DEBUG, "(ISB) End of last page/file.");
        outputSize = (int)(outputPtr - output);

        // flush the remainder of data to the page
        if (upload_hex(output, outputSize) != IS_OP_OK) {
            fwUpdate_sendProgress(IS_LOG_LEVEL_ERROR, "(ISB) Error in upload hex (last)");
            return IS_OP_ERROR;
        }
        if (currentOffset != 0 && fill_current_page() != IS_OP_OK) {
            fwUpdate_sendProgress(IS_LOG_LEVEL_ERROR, "(ISB) Error in fill page (last)");
            return IS_OP_ERROR;
        }

        outputPtr = output;
    }

    return IS_OP_OK;
}


is_operation_result ISBFirmwareUpdater::download_data(int startOffset, int endOffset)
{
    // Atmel download data command is 0x03, different from standard intel hex where command 0x03 is start segment address
    unsigned char programLine[25];
    int n;
    n = SNPRINTF((char*)programLine, 24, ":0500000300%.4X%.4XCC", startOffset, endOffset);
    programLine[n] = 0;
    checksum(0, programLine, 1, 19, 19, 1);
    if (portWrite(device->port, programLine, 21) != 21)
    {
        fwUpdate_sendProgress(IS_LOG_LEVEL_ERROR, "Failed to download");
        return IS_OP_ERROR;
    }

    return IS_OP_OK;
}
/**
 * Returns a fwUpdate-compatible target type (fwUpdate::target_t) appropriate for this DFU device,
 * given the parsed hardware id, where available.
 * @return determined fwUpdate::target_t is detectable, otherwise TARGET_UNKNOWN
 */
fwUpdate::target_t ISBFirmwareUpdater::getTargetType() {
    switch (DECODE_HDW_TYPE(hardwareId)) {
        case IS_HARDWARE_TYPE_IMX:
            if ((DECODE_HDW_MAJOR(hardwareId) == 5) && (DECODE_HDW_MINOR(hardwareId) == 0)) return fwUpdate::TARGET_IMX5;
            // else if ((DECODE_HDW_MAJOR(hardwareId) == 5) && (DECODE_HDW_MINOR(hardwareId) == 1)) return fwUpdate::TARGET_IMX51;
            break;
        case IS_HARDWARE_TYPE_GPX:
            if ((DECODE_HDW_MAJOR(hardwareId) == 1) && (DECODE_HDW_MINOR(hardwareId) == 0)) return fwUpdate::TARGET_GPX1;
            break;
    }

    return fwUpdate::TARGET_UNKNOWN;
}