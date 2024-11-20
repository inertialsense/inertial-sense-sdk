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


// this is called internally by processMessage() to do the things to do, it should also be called periodically to send status updated, etc.
bool ISBFirmwareUpdater::fwUpdate_step(fwUpdate::msg_types_e msg_type, bool processed) {
    static int nextStep = 0;

    if (fwUpdate_getSessionStatus() == fwUpdate::NOT_STARTED)
        return false;

    if (fwUpdate_getSessionStatus() == fwUpdate::INITIALIZING) {
        // if we are initializing, we've successfully issued a RESET_INTO_BOOTLOADER, and we're waiting
        // for this device to become available again, in ISbootloader mode.
        // its possible, if we are connected via a USB port, that we will get a new port id, so we need
        // to scan ports to see if that device becomes available through a new port.
        // once we have the expected SN# and the device is reporting as running in HDW_STATE_BOOTLOADER,
        // then we can advance to ready.

        // one strategy is to setup a second InertialSense instance, and have it query devices;
        // our challenge is that we have to inform our primary InertialSense instance of the newly discovered port

        SLEEP_MS(2000);
        InertialSense is;
        // Wait upto 15 seconds this device to reboot into bootloader mode.
        bool foundIt = false;
        // FIXME: We're basically waiting around for the device to reboot into ISbooloader state.
        //   We want to keep looking for new devices, querying them, and hoping that one of them matches out original device
        for (uint32_t timeout = current_timeMs() + 15000; (current_timeMs() < timeout) && !foundIt; is.Open("*") ) {
            if (is.DeviceCount() > 0) {
                for (auto dev : is.getDevices()) {
                    if ((dev->devInfo.serialNumber == device->devInfo.serialNumber) && (dev->hdwId == device->hdwId) && (dev->hdwRunState == ISDevice::HDW_STATE_BOOTLOADER)) {
                        foundIt = true;
                        device->port = dev->port;
                        device->hdwRunState = dev->hdwRunState;
                        serialPortClose(device->port);
                        serialPortClose(&dev->port);   // cleanup/close the newly opened port, so we can can reopen it again...
                        serialPortOpen(device->port, portName(dev->port), SERIAL_PORT(dev->port)->baudRate, 0);
                        break;
                    }
                }
            }
            SLEEP_MS(1000);
        }

        if (!foundIt) {
            fwUpdate_sendProgressFormatted(IS_LOG_LEVEL_ERROR, "Unable to locate SN%d, after rebooting into bootloader.", device->devInfo.serialNumber);
            session_status = fwUpdate::ERR_COMMS;
            return true;
        }
    }

    if ((fwUpdate_getSessionStatus() == fwUpdate::IN_PROGRESS) || (fwUpdate_getSessionStatus() == fwUpdate::FINALIZING)) {
        // fwUpdate_sendProgressFormatted(IS_LOG_LEVEL_INFO, "Finalizing Update: %d", updateStage);
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
                if (writeState < WRITE_DONE) {
                    writeState = writeFlash_step(writeTimeout);
                    if (writeState >= WRITE_DONE)
                        updateState = VERIFYING;
                } else {
                    updateState = (doVerify ? VERIFYING : UPDATE_DONE);
                    session_status = fwUpdate::FINALIZING;
                }
                break;
            case VERIFYING: // waiting for write to finish
                session_status = fwUpdate::FINALIZING;
                break;
            case UPDATE_DONE:
                delete imgStream;
                delete imgBuffer;
                rebootToAPP();
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

    nextStep++;

    if (nextStep > 1000 ) {
        nextStep = 0;
    }

    if (session_status < fwUpdate::NOT_STARTED)
        fwUpdate_sendProgress();

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
    get_device_info();
    if ((device->port != nullptr) && (device->hdwId != 0)) {
        dev_info = device->DeviceInfo();
        return true;
    }
    return false;
}

// this initializes the system to begin receiving firmware image chunks for the target device, image slot and image size
fwUpdate::update_status_e ISBFirmwareUpdater::fwUpdate_startUpdate(const fwUpdate::payload_t& msg) {
    session_image_size = msg.data.req_update.file_size;

    if (device->hdwRunState == ISDevice::HDW_STATE_BOOTLOADER) {
        if (check_is_compatible()) {
            imgBuffer = new ByteBuffer(session_image_size);
            imgStream = new ByteBufferStream(*imgBuffer);
            return fwUpdate::READY;
        }
    } else if (device->hdwRunState == ISDevice::HDW_STATE_APP) {
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
    serialPortFlush(device->port);

    // Send command
    portWrite(device->port, (uint8_t*)":020000041000EA", 15);

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
        return 0;
    }

    m_isb_major = buf[2];
    m_isb_minor = (char)buf[3];
    m_isb_props.rom_available = buf[4];

    if(buf[11] == '.' && buf[12] == '\r' && buf[13] == '\n')
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
        return 0;
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
    serialPortFlush(device->port);
    portRead(device->port, buf, sizeof(buf));    // empty Rx buffer
    sync();

    SLEEP_MS(100);

    for (int retry=0;; retry++)
    {
        // Send command
        serialPortFlush(device->port);
        portRead(device->port, buf, sizeof(buf));    // empty Rx buffer
        portWrite(device->port, (uint8_t*)":020000041000EA", 15);

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

    if(buf[11] == '.' && buf[12] == '\r' && buf[13] == '\n')
    {   // Valid packet found
        processor = (eProcessorType)buf[5];
        m_isb_props.is_evb = buf[6];
        memcpy(&m_sn, &buf[7], sizeof(uint32_t));
    }
    else
    {   // Error parsing
        char msg[200] = { 0 };
        int n = SNPRINTF(msg, sizeof(msg), "(ISB) check_is_compatible parse error:\n 0x ");
        for(int i=0; i<count; i++)
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

    if(m_isb_major >= 6)
    {   // v6 and up has EVB detection built-in
        if(processor == IS_PROCESSOR_SAMx70)
        {
            valid_signatures |= m_isb_props.is_evb ? IS_IMAGE_SIGN_EVB_2_24K : IS_IMAGE_SIGN_UINS_3_24K;
            if (rom_available) valid_signatures |= IS_IMAGE_SIGN_ISB_SAMx70_16K | IS_IMAGE_SIGN_ISB_SAMx70_24K;
        }
        else if(processor == IS_PROCESSOR_STM32L4)
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
        if(portWrite(device->port, &handshakerChar, 1) != 1)
        {
            return IS_OP_ERROR;
        }

        if (serialPortWaitForTimeout(device->port, &handshakerChar, 1, BOOTLOADER_RESPONSE_DELAY))
        {	// Success
            return IS_OP_OK;
        }
    }

#if defined(SUPPORT_BOOTLOADER_V5A)     // ONLY NEEDED TO SUPPORT BOOTLOADER v5a.  Delete this and associated code in Q4 2022 after bootloader v5a is out of circulation. WHJ
    static const unsigned char handshaker[] = "INERTIAL_SENSE_SYNC_DFU";

    // Attempt handshake using extended string for bootloader v5a
    for (int i = 0; i < BOOTLOADER_RETRIES; i++)
    {
        if (serialPortWriteAndWaitForTimeout(device->port, (const unsigned char*)&handshaker, (int)sizeof(handshaker), &handshakerChar, 1, BOOTLOADER_RESPONSE_DELAY))
        {	// Success
            return IS_OP_OK;
        }
    }
#endif

    return IS_OP_ERROR;
}

/**
 * Instructs the Firmware to reset into the ISbootloader. The device will reinitialize
 * the serial port, forcing a drop of the serial connection.
 * @return true on success, otherwise false
 */
bool ISBFirmwareUpdater::rebootToISB()
{
    fwUpdate_sendProgress(IS_LOG_LEVEL_INFO, "Resetting to ISBootloader...");

    // In case we are in program mode, try and send the commands to go into bootloader mode
    for (size_t loop = 0; loop < 10; loop++) {
        if (!serialPortWriteAscii(device->port, "STPB", 4)) break;     // If the write fails, assume the device is now in bootloader mode.
        if (!serialPortWriteAscii(device->port, "BLEN", 4)) break;
        uint8_t c = 0;
        if (serialPortReadCharTimeout(device->port, &c, 13) == 1) {
            if (c == '$') {
                // done, we got into bootloader mode
                return true;
            }
        }
        else serialPortFlush(device->port);
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
    if (!serialPortIsOpen(device->port))
        return false;

    fwUpdate_sendProgress(IS_LOG_LEVEL_INFO, "(ISB) Rebooting to APP mode...");

    // send the "reboot to program mode" command and the device should start in program mode
    portWrite(device->port, (unsigned char*)":020000040300F7", 15);
    serialPortFlush(device->port);
    if (!keepPortOpen) {
        SLEEP_MS(100);
        serialPortClose(device->port);
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

bool ISBFirmwareUpdater::waitForAck(const std::string& ackStr, const std::string& progressMsg, uint32_t maxTimeout, uint32_t& timeout, float& progress) {
    if (!device || !device->port)
        return false;

    int count = portRead(device->port, rxWorkBufPtr, 3);
    rxWorkBufPtr += count;

    // we want to have a calculated progress which seems reasonable.
    // Since erasing flash is a non-deterministic operation (from our standpoint)
    // let's use a log algorithm that will elapse approx 75% of the progress in
    // in the average time that a device takes to complete this operation (about 10 seconds)
    // the remaining 25% will slowly elapse as we get closer to the timeout period.
    progress = 1.0f - ((timeout - current_timeMs()) / (float)maxTimeout);
    if ((progress_interval > 0) && (nextProgressReport < current_timeMs())) {
        nextProgressReport = current_timeMs() + progress_interval;
        fwUpdate_sendProgress(IS_LOG_LEVEL_INFO, progressMsg);
    }

    while ((rxWorkBufPtr - rxWorkBuf) >= ackStr.length()) {
        if (memcmp(rxWorkBuf, ackStr.c_str(), ackStr.length()) == 0) {
            rxWorkBufPtr = rxWorkBuf;
            progress = 1.0f;
            timeout = 0;
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
 * @return returns a fwUpdate::update_status_e type; a IN_PROGRESS or FINALIZING indicates that the
 *   operation is still in progress.
 */
ISBFirmwareUpdater::eraseState_t ISBFirmwareUpdater::eraseFlash_step(uint32_t timeout) {
    static const std::string SET_LOCATION = ":03000006030000F4CC";
    static const std::string ERASE_FLASH = ":0200000400FFFBCC";

    switch (eraseState) {
        default:
            eraseTimeout = 0;
            eraseState = ERASE_INITIALIZE;
            // fallthrough - any other state should reset back to the init state
        case ERASE_INITIALIZE:
            // load erase location
            if (eraseTimeout == 0) {
                if (sendCmd(SET_LOCATION))    // size should be 19, and chksumPos = 17
                    eraseTimeout = current_timeMs() + BOOTLOADER_TIMEOUT_DEFAULT;
                SLEEP_MS(100); // give a moment for the device to respond to the command (but not too long).
            }

            if (!waitForAck(".\r\n", "Erasing Flash", BOOTLOADER_TIMEOUT_DEFAULT, eraseTimeout, eraseProgress)) {
                if (current_timeMs() > eraseTimeout) {
                    fwUpdate_sendProgress(IS_LOG_LEVEL_ERROR, "Error while erasing flash (timeout after set_location)");
                    session_status = fwUpdate::ERR_FLASH_WRITE_FAILURE;
                    eraseState = ERASE_TIMEOUT;
                    return eraseState;
                }
            } else {
                eraseState = ERASE;
            }
            break;
        case ERASE:
            // load erase location
            if (eraseTimeout == 0) {
                if (sendCmd(ERASE_FLASH))
                    eraseTimeout = current_timeMs() + timeout;
                SLEEP_MS(100); // give a moment for the device to respond to the command (but not too long).
            }

            if (!waitForAck(".\r\n", "Erasing Flash", timeout, eraseTimeout, eraseProgress)) {
                if (current_timeMs() > eraseTimeout) {
                    fwUpdate_sendProgress(IS_LOG_LEVEL_ERROR, "Error while erasing flash (timeout after erase_flash)");
                    session_status = fwUpdate::ERR_FLASH_WRITE_FAILURE;
                    eraseState = ERASE_TIMEOUT;
                    return eraseState;
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
    SNPRINTF((char*)changePage, 24, ":040000060301%.4XCC", page);
    checksum(0, changePage, 1, 17, 17, 1);
    if (serialPortWriteAndWaitForTimeout(device->port, changePage, 19, (unsigned char*)".\r\n", 3, BOOTLOADER_TIMEOUT_DEFAULT) == 0)
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
    //  the program code will be written
    unsigned char programPage[24];

    // Select offset
    SNPRINTF((char*)programPage, 24, ":0500000100%.4X%.4XCC", startOffset, endOffset);
    checksum(0, programPage, 1, 19, 19, 1);
    if (serialPortWriteAndWaitForTimeout(device->port, programPage, 21, (unsigned char*)".\r\n", 3, BOOTLOADER_TIMEOUT_DEFAULT) == 0)
    {
        fwUpdate_sendProgress(IS_LOG_LEVEL_ERROR, "(ISB) Failed to start programming page");
        return IS_OP_ERROR;
    }

    return IS_OP_OK;
}

int ISBFirmwareUpdater::is_isb_read_line(ByteBufferStream& byteStream, char line[1024])
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

is_operation_result ISBFirmwareUpdater::upload_hex_page(unsigned char* hexData, int byteCount)
{
    if (byteCount == 0)
    {
        return IS_OP_OK;
    }

    // create a program request with just the hex characters that will fit on this page
    unsigned char programLine[12];
    SNPRINTF((char*)programLine, 12, ":%02X%04X00", byteCount, currentOffset);
    if (portWrite(device->port, programLine, 9) != 9)
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
    SNPRINTF((char*)checkSumHex, 3, "%.2X", checkSum);

    // For some reason, the checksum doesn't always make it through to the IMX-5. Re-send until we get a response or timeout.
    // Update 8/25/22: Increasing the serialPortReadTimeout from 10 to 100 seems to have fixed this. Still needs to be proven.
    for(int i = 0; i < 10; i++)
    {
        if (portWrite(device->port, checkSumHex, 2) != 2)
        {
            fwUpdate_sendProgress(IS_LOG_LEVEL_ERROR, "(ISB) Failed to write checksum to device");
            return IS_OP_ERROR;
        }

        unsigned char buf[5] = { 0 };
        int count = portReadTimeout(device->port, buf, 3, 1000);
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

is_operation_result ISBFirmwareUpdater::upload_hex(unsigned char* hexData, int charCount)
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
    if (currentOffset + byteCount < FLASH_PAGE_SIZE)
    {
        int pageByteCount = FLASH_PAGE_SIZE - currentOffset;
        if (upload_hex_page(hexData, pageByteCount) != IS_OP_OK)
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
                fwUpdate_sendProgress(IS_LOG_LEVEL_ERROR, "(ISB) Failed to fill page with bytes");
                return IS_OP_ERROR;
            }
        }
    }

    return IS_OP_OK;
}

#define HEX_BUFFER_SIZE 1024
/*
fwUpdate::update_status_e ISBFirmwareUpdater::doHexData(unsigned char* line, int lineLength, unsigned char* output, unsigned char* outputPtr) {
    if (lineLength > HEX_BUFFER_SIZE * 4) {
        fwUpdate_sendProgress(IS_LOG_LEVEL_ERROR, "(ISB) hex file line length too long");
        writeState = WRITE_DONE;
        return fwUpdate::ERR_INVALID_IMAGE;
    }

    // we need to know the offset that this line was supposed to be stored at so we can check if offsets are skipped
    unsigned char tmp[5];
    int pad = 0;

    memcpy(tmp, line + 3, 4);
    tmp[4] = '\0';
    int subOffset = strtol((char*)tmp, 0, 16);

    // check if we skipped an offset, the intel hex file format can do this, in which case we need to make sure
    // that the bytes that were skipped get set to something
    if (subOffset > lastSubOffset) {
        // pad with FF bytes, this is an internal implementation detail to how the device stores unused memory
        pad = (subOffset - lastSubOffset);
        if (outputPtr + pad >= outputPtrEnd) {
            writeState = WRITE_DONE;
            fwUpdate_sendProgress(IS_LOG_LEVEL_ERROR, "(ISB) FF padding overflowed buffer");
            return fwUpdate::ERR_INVALID_IMAGE;
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
        writeState = WRITE_DONE;
        fwUpdate_sendProgress(IS_LOG_LEVEL_ERROR, "(ISB) Line data overflowed output buffer");
        return fwUpdate::ERR_INVALID_IMAGE;
    }

    for (int i = 9; i < lineLength - 2; i++) {
        *outputPtr++ = line[i];
    }

    // set the end offset so we can check later for skipped offsets
    lastSubOffset = subOffset + ((lineLength - 11) / 2);
    int outputSize = (int)(outputPtr - output);

    // we try to send the most allowed by this hex file format
    if (outputSize < MAX_SEND_COUNT) {
        // keep buffering
        writeState = WRITE;
        return fwUpdate::IN_PROGRESS;   // we've done what we can, for now...
    }
    // upload this chunk
    if (upload_hex(output, _MIN(MAX_SEND_COUNT, outputSize)) != IS_OP_OK) {
        writeState = WRITE_DONE;
        fwUpdate_sendProgress(IS_LOG_LEVEL_ERROR, "(ISB) Error in upload chunk");
        return fwUpdate::ERR_INVALID_IMAGE;
    }

    outputSize -= MAX_SEND_COUNT;

    if (outputSize < 0 || outputSize > HEX_BUFFER_SIZE) {
        writeState = WRITE_DONE;
        fwUpdate_sendProgress(IS_LOG_LEVEL_ERROR, "(ISB) Output size was too large (1)");
        return fwUpdate::ERR_INVALID_IMAGE;
    }

    if (outputSize > 0) {
        // move the left-over data to the beginning
        memmove(output, output + MAX_SEND_COUNT, outputSize);
    }

    // reset output ptr back to the next chunk of data
    outputPtr = output + outputSize;
}

fwUpdate::update_status_e ISBFirmwareUpdater::writeNewFlashPage(unsigned char* line, unsigned char* output, unsigned char* outputPtr) {
    unsigned char tmp[5];

    memcpy(tmp, line + 12, 3);      // Only support up to 10 pages currently
    tmp[1] = '\0';
    currentPage = strtol((char*)tmp, 0, 16);

    if(currentPage == 0) {
        writeState = WRITE;
        lastSubOffset = currentOffset;
        return fwUpdate::IN_PROGRESS;   // we've done what we can, for now...
    } else {
        lastSubOffset = 0;
    }

    int outputSize = (int)(outputPtr - output);

    if (outputSize < 0 || outputSize > HEX_BUFFER_SIZE) {
        writeState = WRITE_DONE;
        fwUpdate_sendProgress(IS_LOG_LEVEL_ERROR, "(ISB) Output size was too large (2)");
        return fwUpdate::ERR_INVALID_IMAGE;
    }

    // flush the remainder of data to the page
    if (upload_hex(output, outputSize) != IS_OP_OK) {
        writeState = WRITE_DONE;
        fwUpdate_sendProgress(IS_LOG_LEVEL_ERROR, "(ISB) Error in upload hex");
        return fwUpdate::ERR_INVALID_IMAGE;
    }

    // fill remainder of the current page, the next time that bytes try to be written the page will be automatically incremented
    if (fill_current_page() != IS_OP_OK) {
        writeState = WRITE_DONE;
        fwUpdate_sendProgress(IS_LOG_LEVEL_ERROR, "(ISB) Error in fill page");
        return fwUpdate::ERR_INVALID_IMAGE;
    }

    // change to the next page
    currentOffset = 0;
    if (select_page(currentPage) != IS_OP_OK || begin_program_for_current_page(0, FLASH_PAGE_SIZE - 1) != IS_OP_OK) {
        writeState = WRITE_DONE;
        fwUpdate_sendProgress(IS_LOG_LEVEL_ERROR, "(ISB) Failed to issue select page or to start programming");
        return fwUpdate::ERR_INVALID_IMAGE;
    }

    // set the output ptr back to the beginning, no more data is in the queue
    outputPtr = output;
}

fwUpdate::update_status_e ISBFirmwareUpdater::doEndOfFile(unsigned char* output, unsigned char* outputPtr) {
    int outputSize = (int)(outputPtr - output);

    // flush the remainder of data to the page
    if (upload_hex(output, outputSize) != IS_OP_OK) {
        writeState = WRITE_DONE;
        fwUpdate_sendProgress(IS_LOG_LEVEL_ERROR, "(ISB) Error in upload hex (last)");
        return fwUpdate::ERR_INVALID_IMAGE;
    }
    if (currentOffset != 0 && fill_current_page() != IS_OP_OK) {
        writeState = WRITE_DONE;
        fwUpdate_sendProgress(IS_LOG_LEVEL_ERROR, "(ISB) Error in fill page (last)");
        return fwUpdate::ERR_INVALID_IMAGE;
    }

    outputPtr = output;
}
*/

/**
 * Processes an Intel hex file, one chunk/line at a time. This function is meant to be called
 * repeatedly (in a step/thread, minimal-blocking fashion) until the entire file is processed.
 * @return true if the entire file has been processed, otherwise false (repeat until true)
 */
ISBFirmwareUpdater::writeState_t ISBFirmwareUpdater::writeFlash_step(uint32_t timeout) {
    // local variables that can be volatile/non-static
    static unsigned char output[HEX_BUFFER_SIZE * 2]; // big enough to store an entire extra line of buffer if needed
    unsigned char* outputPtr = output;
    const unsigned char* outputPtrEnd = output + (HEX_BUFFER_SIZE * 2);
    unsigned char tmp[5];
    static IhexTransformer hexTransformer;

    auto portEmitter = [&](const std::string& record) {
        printf(":: %s\n", record.c_str());
        switch (IhexTransformer::sendRecord(device->port, record)) {
            case IhexTransformer::IHEX_DATA_READY_TO_SEND:
            case IhexTransformer::IHEX_OP_OK:
                return;
            default:
            // Error occurred.
            case IhexTransformer::IHEX_ERROR__FAILED_TO_ACK:
            case IhexTransformer::IHEX_ERROR__FAILED_TO_SEND:
            case IhexTransformer::IHEX_ERROR__BUFLEN_EXCEEDED:
            case IhexTransformer::IHEX_ERROR__LINE_LEN_EXCEEDED:
            case IhexTransformer::IHEX_ERROR__LINE_LEN_MOD:
            case IhexTransformer::IHEX_ERROR__INVALID_CHECKSUM:
            case IhexTransformer::IHEX_ERROR__INVALID_START:
                printf("Unable to send record.\n");
                break;
        }
    };


    switch (writeState) {
        case WRITE_INITIALIZE:
            select_page(0);
            begin_program_for_current_page(m_isb_props.app_offset, FLASH_PAGE_SIZE - 1);
            currentPage = -1;
            currentOffset = m_isb_props.app_offset;
            totalBytes = m_isb_props.app_offset;
            verifyCheckSum = 5381;
            writeState = WRITE;
            break;
        case WRITE:
        {
            process_hex_file(*imgStream);
/*
            char line[HEX_BUFFER_SIZE];
            int lineLen = is_isb_read_line(*imgStream, line);
            if (lineLen == 0) {
                writeState = WRITE_FINALIZE;
                return writeState;
            }
            //std::string record(line);
            //hexTransformer.processRecord(record, portEmitter);
            upload_hex(reinterpret_cast<unsigned char *>(line), (lineLen-1) / 2);
*/
            break;
        }
        case WRITE_FINALIZE:
            break;
        case WRITE_DONE:
            break;
    }

    int lastSubOffset = currentOffset;
    int subOffset;

    writeProgress = ((float)imgStream->tellg() / (float)session_image_size);
    if ((progress_interval > 0) && (nextProgressReport < current_timeMs())) {
        writeState = WRITE_DONE;
        nextProgressReport = current_timeMs() + progress_interval;
        fwUpdate_sendProgress(IS_LOG_LEVEL_INFO, "Writing Flash");
    }

    session_status = fwUpdate::IN_PROGRESS;
    writeState = WRITE;
    return writeState;
}

is_operation_result ISBFirmwareUpdater::process_hex_file(ByteBufferStream& byteStream)
{
    int currentPage = -1;
    int currentOffset = m_isb_props.app_offset;
    int lastSubOffset = currentOffset;
    int subOffset;
    int totalBytes = m_isb_props.app_offset;

    int verifyCheckSum = 5381;
    int lineLength;
    // m_update_progress = 0.0f;
    char line[HEX_BUFFER_SIZE];
    unsigned char output[HEX_BUFFER_SIZE * 2]; // big enough to store an entire extra line of buffer if needed
    unsigned char* outputPtr = output;
    const unsigned char* outputPtrEnd = output + (HEX_BUFFER_SIZE * 2);
    int outputSize = 0;
    int pad;
    unsigned char tmp[5];
    int i;

    while ((lineLength = is_isb_read_line(byteStream, line)) != 0) {
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
                continue;
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
            currentPage = strtol((char*)tmp, 0, 16);

            if(currentPage == 0) {
                lastSubOffset = currentOffset;
                continue;
            } else {
                lastSubOffset = 0;
            }

            outputSize = (int)(outputPtr - output);

            if (outputSize < 0 || outputSize > HEX_BUFFER_SIZE) {
                fwUpdate_sendProgress(IS_LOG_LEVEL_ERROR, "(ISB) Output size was too large (2)");
                return IS_OP_ERROR;
            }

            // flush the remainder of data to the page
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

        writeProgress = ((float)byteStream.tellg() / (float)session_image_size);
        if ((progress_interval > 0) && (nextProgressReport < current_timeMs())) {
            nextProgressReport = current_timeMs() + progress_interval;
            fwUpdate_sendProgress(IS_LOG_LEVEL_INFO, "Writing Flash");
        }
    }

    if (writeProgress != 1.0f) {
        writeProgress = 1.0f;
        fwUpdate_sendProgress(IS_LOG_LEVEL_INFO, "Writing Flash");
    }

    // Set the verify function up
    // m_currentPage = currentPage;
    // m_verifyCheckSum = verifyCheckSum;

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

/**
 * Calculates a IHEX checksum from the provided "line". If you are calculating an checksum for outgoing message
 * set 'append' to true, which will appends the resulting checksum, in ones-complement, to the end of the line.
 * If you are calculating the checksum for a line which includes the checksum, the returned checksum should be 0
 * indicating a pass, any other value indicates a failure to match.
 * @param line
 * @param append, if true causes the calculated checksum to be appended to the current line
 * @return if >= 0, indicates the calculated checksum, else (<0) indicates a parse error
 */
int IhexTransformer::checksum(std::string& line, bool append, uint8_t initChksum)
{
    std::string lineCopy(line);
    if (lineCopy[0] == ':')
        lineCopy = line.substr(1);
    if ((lineCopy.length() % 2) != 0)
        return -1; // provided record has invalid length

    uint8_t checksum = initChksum;
    for (size_t i = 0; i < lineCopy.size(); i += 2) {
        std::string byteChunk = lineCopy.substr(i, 2);
        checksum += std::stoul(byteChunk, NULL, 16);
    }

    if (append) {
        char chkStr[3] = {};
        SNPRINTF(chkStr, 3, "%02X", checksum);
        line.append(chkStr);
    } else {
        checksum = (uint8_t)(~checksum + 1);
    }
    return checksum;
}

/**
 * Send the assemble IHEX record/packet to the remote device, including populated the relevant header fields,
 * and calculating and appending the final checksum. This also waits for a response from the remove device
 * confirming receipt of the provided data. This function does not do any data encoding; the hexData to be
 * sent must already be in the ASCII HEX (2 characters per byte) necessary for the Intel HEX format
 * @param port the port to send the record/data do
 * @param recNum the Intel HEX record number associated with this data
 * @param hexData a byte array of hexadecimal encoded pairs of nibbles the represent the data to be sent.
 * @param hexDataLen the number of actual characters (not represented bytes) in the hexData payload
 * @return
 */
std::string IhexTransformer::buildRecord(uint16_t recordOffset, uint8_t recordType, const char* hexData, int hexDataLen, IhexTransformer::ihex_op_t& error) {
    // create a record request with just the hex characters that will fit on this page
    int byteCount = hexDataLen / 2;
    if (byteCount > 255) {
        error = IHEX_ERROR__LINE_LEN_EXCEEDED;
        return "";
    }

    std::string recordLine = utils::string_format(":%02X%04X%02X", byteCount, recordOffset, recordType);
    recordLine.append((const char *)hexData, hexDataLen);
    int newChecksum = checksum(recordLine, false);
    std::string chksumHex = utils::string_format("%02X", newChecksum);
    recordLine.append(chksumHex);
    error = IHEX_OP_OK;
    return recordLine;
}


IhexTransformer::ihex_op_t IhexTransformer::emitRecord(std::function<void(const std::string& record)> emitterCb, uint16_t recordOffset, uint8_t recordType, const char* hexData, int hexDataLen) {
    ihex_op_t buildResult = IHEX_OP_OK;

    std::string record = buildRecord(recordOffset, recordType, hexData, hexDataLen, buildResult);
    if (buildResult == IHEX_OP_OK) {
        if (recordType == 0)
            initializeBuffer(); // safe to do, because 'record' is an encoded copy of the entire buffer
        emitterCb(record);
    }

    return buildResult;
}

uint8_t IhexTransformer::waitForAnswer(port_handle_t port, uint32_t timeoutMs) {
    if (port == 0)
        return 0;

    const uint16_t TERMINATOR = ('\r' << 8) | '\n';
    static uint32_t answer = 0;

    uint32_t timeout = current_timeMs() + timeoutMs;
    while (current_timeMs() < timeout) {
        uint8_t byte = 0;
        while (portRead(port, &byte, 1)) {
             answer = (answer << 8) | (byte & 0xFF);
             if ((answer & 0xFFFF) == TERMINATOR)
                 return ((answer >> 16) & 0xFF);
        }
    }
    return 0;
}

/**
 * Builds and sends a complete packet out to the specified port, and waits for a positive acknowledgement from the remote.
 * NOTE that this function will attempt to resend the final CHKSUM periodically if an acknowledgement isn't received immediately.
 * @param port the port to send the record on
 * @param record the complete HEX record, previously encoded (see buildRecord) including checksum
 * @param bytesSent a pointer to an integer which, if not null, will be set to indicates the total number of bytes transfered
 * @return
 */
IhexTransformer::ihex_op_t IhexTransformer::sendRecord(port_handle_t port, const std::string& record, int* bytesSent)
{
    // create a record request with just the hex characters that will fit on this page
    std::string chksumHex = record.substr(record.length() - 2, 2);
    int bytesWritten = portWrite(port, (const uint8_t*)record.c_str(), record.length());
    if (bytesWritten != (int)record.length())
        return IHEX_ERROR__FAILED_TO_SEND;

    SLEEP_MS(10);
    if (bytesSent)
        *bytesSent = bytesWritten;

    // For some reason, the checksum doesn't always make it through to the IMX-5. Re-send until we get a response or timeout.
    // Update 8/25/22: Increasing the serialPortReadTimeout from 10 to 100 seems to have fixed this. Still needs to be proven.
    uint32_t timeout = current_timeMs() + 1000;
    while (current_timeMs() < timeout) {
        uint8_t reply = waitForAnswer(port, 200);
        if (!reply) {
            printf("Timeout waiting for reply: %c\n", reply);
        } else if (reply != '.') {
            printf("Unexpected reply: %c\n", reply);
            return IHEX_ERROR__FAILED_TO_ACK;
        } else {
            return IHEX_OP_OK;
        }

        if (portWrite(port, (const uint8_t*)chksumHex.c_str(), chksumHex.length()) != 2)
            return IHEX_ERROR__FAILED_TO_SEND;

        if (bytesSent)
            *bytesSent += 2;
    }
    return IHEX_ERROR__NO_RESPONSE;
}


void IhexTransformer::initializeBuffer() {
    memset(hexBuffer, 0, sizeof(hexBuffer));
    baseAddress = -1;
    currentPage = -1;
    currentOffset = -1;
    bufferOffset = -1;
}

IhexTransformer::ihex_op_t IhexTransformer::processRecord(const std::string &line, std::function<void(const std::string& record)> emitterCb, uint8_t fillByte) {
    // parse out the header first.
    std::string finalRecord;
    ihex_op_t buildResult = IHEX_OP_OK;

    if (line[0] != ':')
        return IHEX_ERROR__INVALID_START;

    uint8_t recordByteLen = std::stoul(line.substr(IHEX_DATA_LEN_POS, IHEX_DATA_LEN_LEN), 0, BASE_HEXADECIMAL);
    uint16_t recordOffset = std::stoul(line.substr(IHEX_WRITE_ADDR_POS, IHEX_WRITE_ADDR_LEN), 0, BASE_HEXADECIMAL);
    uint8_t recordType = std::stoul(line.substr(IHEX_RECORD_TYPE_POS, IHEX_RECORD_TYPE_LEN), 0, BASE_HEXADECIMAL);

    if (checksum(const_cast<std::string &>(line)) != 0) // invalid checksum
        return IHEX_ERROR__INVALID_CHECKSUM;

    int lineDataCharLen = recordByteLen * 2;
    if (lineDataCharLen != (line.length() - (IHEX_RECORD_TYPE_POS + IHEX_RECORD_TYPE_LEN) - 2))
        return IHEX_ERROR__LINE_LEN_MOD;

    std::string dataStr = line.substr(IHEX_DATA_POS, lineDataCharLen);

    if (recordType != 0x00) {
        // All none-zero record types are handled here, because they are sent directly to the remote target, and require no transforming.
        // But, we can't handle them if we have a transformed back in the works, if so, send it before we move on.
        if (bufferOffset > 0) {
            auto result = emitRecord(emitterCb, currentOffset, 0x00, reinterpret_cast<const char *>(hexBuffer), bufferOffset);
            if (result != IHEX_OP_OK)
                return result;
        }

        // But, occasionally, we need to do some pre-processing when we receive them, so let's handle that here...
        if ((recordType == 0x04) && (recordByteLen == 2)) {
            // a record type of 0x04 sets the base address and resets all other working parameters
            baseAddress = (std::stoul(dataStr, 0, BASE_HEXADECIMAL) << 16);
            currentOffset = -1;
        }

        return emitRecord(emitterCb, recordOffset, recordType, dataStr.c_str(), dataStr.length());
    }

    if (currentOffset < 0) {
        currentOffset = recordOffset; // only set this if it hasn't been set
        lastWriteOffset = 0;
    }
    bufferOffset = (recordOffset - currentOffset) * 2;  // remember, buffer offset is the number of characters in the hex buffer, not actual byte offset, so x2
    if (bufferOffset < lastWriteOffset)
        lastWriteOffset = bufferOffset; // lastWrite should never be greater than the bufferOffset; it so, then we've moved back to an already written address

    if (bufferOffset + lineDataCharLen >= HEX_BUFFER_SIZE ) {
        return IHEX_ERROR__BUFLEN_EXCEEDED;
    }

    // Now that we've validated everything, push the newly parsed data into the buffer
    char* writePtr = (char*)&hexBuffer[lastWriteOffset];

    // check if we skipped an offset, the intel hex file format can do this, in which case we need to make sure
    // that the bytes that were skipped get set to something
    int pad = (bufferOffset - lastWriteOffset);
    if (pad > 0) {
        if (pad + lastWriteOffset < MAX_SEND_COUNT) {
            // fill the skipped memory with our fillByte (defaults to 0xFF)
            for (; pad > 0; pad -= 2, writePtr += 2) {
                SNPRINTF(writePtr, 3, "%02X", fillByte);
            }
            lastWriteOffset = bufferOffset;
        } else {
            // if there isn't room to pad, then just send this record and start over with a clean one
            // NOTE the use of lastWriteOffset below because bufferOffset is where we WOULD start writing new data, which exceeds the MAX_SEND_COUNT
            auto result = emitRecord(emitterCb, currentOffset, 0x00, reinterpret_cast<const char *>(hexBuffer), lastWriteOffset);
            if (result != IHEX_OP_OK)
                return result;
            // NOTE, if we successfully sent it, sendRecord() will flush the current buffer, and reinitialize all our variables (like currentOffset/bufferOffset)
            //   At this point, its probably easier/safer to call stuffHexData() again
            return processRecord(line, emitterCb, fillByte);
        }
    }

    if (bufferOffset + lineDataCharLen >= MAX_SEND_COUNT) {
        // if we are here, it means we've exceeded our current max SEND limit of 256 bytes (510 characters), so lets send what we have already, and then start over with a fresh buffer
        auto result = emitRecord(emitterCb, currentOffset, 0x00, reinterpret_cast<const char *>(hexBuffer), bufferOffset);
        if (result == IHEX_OP_OK) {
            // reset a new buffer and
            memset(hexBuffer, 0, sizeof(hexBuffer));
            currentOffset = bufferOffset = -1;
            return processRecord(line, emitterCb, fillByte);
        }
    }

    // else, there is still sufficient room to append the record

    writePtr = (char*)&hexBuffer[bufferOffset];
    SNPRINTF(writePtr, lineDataCharLen+1, "%s", dataStr.c_str());
    bufferOffset += lineDataCharLen;
    lastWriteOffset = bufferOffset;

    // we try to send the most allowed by this hex file format
    if (bufferOffset < MAX_SEND_COUNT) {
        // keep buffering
        return IHEX_OP_OK;
    }

    auto result = emitRecord(emitterCb, currentOffset, 0x00, reinterpret_cast<const char *>(hexBuffer), bufferOffset);
    return result;
}
