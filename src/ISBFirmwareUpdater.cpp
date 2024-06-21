/**
 * @file ISBFirmwareUpdater.cpp 
 * @brief ${BRIEF_DESC}
 *
 * @author Kyle Mallory on 5/29/24.
 * @copyright Copyright (c) 2024 Inertial Sense, Inc. All rights reserved.
 */

#include "ISBFirmwareUpdater.h"
#include "ISBootloaderBase.h"
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

        sleep(2);
        InertialSense is;
        // Wait upto 15 seconds this device to reboot into bootloader mode.
        bool foundIt = false;
        for (uint32_t timeout = current_timeMs() + 15000; (current_timeMs() < timeout) && !foundIt; is.Open("*") ) {
            if (is.DeviceCount() > 0) {
                for (auto& dev: is.getDevices()) {
                    if ((dev.devInfo.serialNumber == device.devInfo.serialNumber) && (dev.hdwId == device.hdwId) && (dev.hdwRunState == ISDevice::HDW_STATE_BOOTLOADER)) {
                        foundIt = true;
                        device.portHandle = dev.portHandle;
                        device.hdwRunState = dev.hdwRunState;
                        serialPortClose(&device.serialPort);
                        serialPortClose(&dev.serialPort);   // cleanup/close the newly opened port, so we can can reopen it again...
                        serialPortOpen(&device.serialPort, dev.serialPort.port, dev.serialPort.baudRate, 0);
                        break;
                    }
                }
            }
            sleep(1);
        }

        if (!foundIt) {
            fwUpdate_sendProgressFormatted(IS_LOG_LEVEL_ERROR, "Unable to locate SN%d, after rebooting into bootloader.", device.devInfo.serialNumber);
            session_status = fwUpdate::ERR_COMMS;
            return true;
        }
    }

    if (fwUpdate_getSessionStatus() == fwUpdate::IN_PROGRESS) {
        // do some thing..
        int a = a + 1;
    }
    if (fwUpdate_getSessionStatus() == fwUpdate::FINALIZING) {
        switch (updateStage) {
            case 0: // transfer
                if (transferProgress >= 1.0f)
                    updateStage++;
                return fwUpdate::FINALIZING;
            case 1: // prepare for erase
                get_device_info();
                updateStage++;
                return fwUpdate::FINALIZING;
            case 2: // waiting for erase to finish
                erase_flash();
                updateStage++;
                return fwUpdate::FINALIZING;
            case 3: // prepare for write
                select_page(0);
                begin_program_for_current_page(m_isb_props.app_offset, FLASH_PAGE_SIZE - 1);
                updateStage++;
                return fwUpdate::FINALIZING;
            case 4: // waiting for write to finish
                if (step_loadHex() != fwUpdate::IN_PROGRESS)
                    updateStage++;
                return fwUpdate::FINALIZING;
            case 5:
                delete imgStream;
                delete imgBuffer;
                rebootToAPP();
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
    if ((device.serialPort.handle != nullptr) && (device.hdwId != 0)) {
        dev_info = device.devInfo;
        return true;
    }
    return false;
}

// this initializes the system to begin receiving firmware image chunks for the target device, image slot and image size
fwUpdate::update_status_e ISBFirmwareUpdater::fwUpdate_startUpdate(const fwUpdate::payload_t& msg) {
    session_image_size = msg.data.req_update.file_size;

    if (device.hdwRunState == ISDevice::HDW_STATE_BOOTLOADER) {
        if (check_is_compatible()) {
            imgBuffer = new ByteBuffer(session_image_size);
            imgStream = new ByteBufferStream(*imgBuffer);
            return fwUpdate::READY;
        }
    } else if (device.hdwRunState == ISDevice::HDW_STATE_APP) {
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
    if (updateStage <= 4) {
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
    serialPortFlush(&device.serialPort);

    // Send command
    serialPortWrite(&device.serialPort, (uint8_t*)":020000041000EA", 15);

    uint8_t buf[14] = { 0 };

    // Read Version, SAM-BA Available, serial number (in version 6+) and ok (.\r\n) response
    int count = serialPortReadTimeout(&device.serialPort, buf, 14, 1000);

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

    serialPortFlush(&device.serialPort);
    serialPortRead(&device.serialPort, buf, sizeof(buf));    // empty Rx buffer
    sync();

    SLEEP_MS(100);

    for (int retry=0;; retry++)
    {
        // Send command
        serialPortFlush(&device.serialPort);
        serialPortRead(&device.serialPort, buf, sizeof(buf));    // empty Rx buffer
        serialPortWrite(&device.serialPort, (uint8_t*)":020000041000EA", 15);

        // Read Version, SAM-BA Available, serial number (in version 6+) and ok (.\r\n) response
#define READ_DELAY_MS   500
        count = serialPortReadTimeout(&device.serialPort, buf, 14, READ_DELAY_MS);

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
        if(serialPortWrite(&device.serialPort, &handshakerChar, 1) != 1)
        {
            return IS_OP_ERROR;
        }

        if (serialPortWaitForTimeout(&device.serialPort, &handshakerChar, 1, BOOTLOADER_RESPONSE_DELAY))
        {	// Success
            return IS_OP_OK;
        }
    }

#if defined(SUPPORT_BOOTLOADER_V5A)     // ONLY NEEDED TO SUPPORT BOOTLOADER v5a.  Delete this and associated code in Q4 2022 after bootloader v5a is out of circulation. WHJ
    static const unsigned char handshaker[] = "INERTIAL_SENSE_SYNC_DFU";

    // Attempt handshake using extended string for bootloader v5a
    for (int i = 0; i < BOOTLOADER_RETRIES; i++)
    {
        if (serialPortWriteAndWaitForTimeout(&device.serialPort, (const unsigned char*)&handshaker, (int)sizeof(handshaker), &handshakerChar, 1, BOOTLOADER_RESPONSE_DELAY))
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
        if (!serialPortWriteAscii(&device.serialPort, "STPB", 4)) break;     // If the write fails, assume the device is now in bootloader mode.
        if (!serialPortWriteAscii(&device.serialPort, "BLEN", 4)) break;
        uint8_t c = 0;
        if (serialPortReadCharTimeout(&device.serialPort, &c, 13) == 1) {
            if (c == '$') {
                // done, we got into bootloader mode
                return true;
            }
        }
        else serialPortFlush(&device.serialPort);
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
    if (!serialPortIsOpen(&device.serialPort))
        return false;

    fwUpdate_sendProgress(IS_LOG_LEVEL_INFO, "(ISB) Rebooting to APP mode...");

    // send the "reboot to program mode" command and the device should start in program mode
    serialPortWrite(&device.serialPort, (unsigned char*)":020000040300F7", 15);
    serialPortFlush(&device.serialPort);
    if (!keepPortOpen) {
        SLEEP_MS(100);
        serialPortClose(&device.serialPort);
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

/**
 * Performs an ERASE operation on all available flash memory reserved for the firmware (this does not include the flash reserved for the bootloader)
 * @return
 */
fwUpdate::update_status_e ISBFirmwareUpdater::erase_flash() {
    uint32_t timeout = current_timeMs() + 60000;  // give the device 60 seconds to erase flash before giving up

    // load erase location
    unsigned char cmdBuffer[24];
    memcpy(cmdBuffer, ":03000006030000F4CC\0\0\0\0\0", 24);
    checksum(0, cmdBuffer, 1, 17, 17, 1);
    if (serialPortWriteAndWaitForTimeout(&device.serialPort, cmdBuffer, 19, (unsigned char*)".\r\n", 3, BOOTLOADER_TIMEOUT_DEFAULT) == 0)
        return fwUpdate::ERR_FLASH_WRITE_FAILURE;

    // perform erase
    memcpy(cmdBuffer, ":0200000400FFFBCC\0", 18);
    checksum(0, cmdBuffer, 1, 15, 15, 1);
    serialPortWrite(&device.serialPort, cmdBuffer, 17);

    // wait for response (up to 60 seconds)
    uint8_t buf[128];
    uint8_t *bufPtr = buf;
    int count = 0;
    while (current_timeMs() < timeout) {
        count += serialPortReadTimeout(&device.serialPort, bufPtr, 3, 100);
        bufPtr = buf + count;

        // we want to have a calculated progress which seems reasonable.
        // Since erasing flash is a non-deterministic operation (from our standpoint)
        // let's use a log algorithm that will elapse approx 75% of the progress in
        // in the average time that a device takes to complete this operation (about 10 seconds)
        // the remaining 25% will slowly elapse as we get closer to the timeout period.
        eraseProgress = 1.0f - ((timeout - current_timeMs()) / 60000.f);
        if ((progress_interval > 0) && (nextProgressReport < current_timeMs())) {
            nextProgressReport = current_timeMs() + progress_interval;
            fwUpdate_sendProgress(IS_LOG_LEVEL_INFO, "Erasing Flash");
        }

        if (count == 3 && memcmp(buf, ".\r\n", 3) == 0) {
            eraseProgress = 1.0f;
            return session_status;
        }
    }

    fwUpdate_sendProgress(IS_LOG_LEVEL_ERROR, "Error while erasing flash");
    return fwUpdate::ERR_FLASH_WRITE_FAILURE;
}

is_operation_result ISBFirmwareUpdater::select_page(int page)
{
    // Atmel select page command (0x06) is 4 bytes and the data is always 0301xxxx where xxxx is a 16 bit page number in hex
    unsigned char changePage[24];

    // Change page
    SNPRINTF((char*)changePage, 24, ":040000060301%.4XCC", page);
    checksum(0, changePage, 1, 17, 17, 1);
    if (serialPortWriteAndWaitForTimeout(&device.serialPort, changePage, 19, (unsigned char*)".\r\n", 3, BOOTLOADER_TIMEOUT_DEFAULT) == 0)
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
    if (serialPortWriteAndWaitForTimeout(&device.serialPort, programPage, 21, (unsigned char*)".\r\n", 3, BOOTLOADER_TIMEOUT_DEFAULT) == 0)
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

is_operation_result ISBFirmwareUpdater::upload_hex_page(unsigned char* hexData, int byteCount, int* currentOffset, int* totalBytes, int* verifyCheckSum)
{
    if (byteCount == 0)
    {
        return IS_OP_OK;
    }

    // create a program request with just the hex characters that will fit on this page
    unsigned char programLine[12];
    SNPRINTF((char*)programLine, 12, ":%.2X%.4X00", byteCount, *currentOffset);
    if (serialPortWrite(&device.serialPort, programLine, 9) != 9)
    {
        fwUpdate_sendProgress(IS_LOG_LEVEL_ERROR, "(ISB) Failed to write start page");
        return IS_OP_ERROR;
    }

    // add the previously written chars to the checksum
    int checkSum = checksum(0, programLine, 1, 9, 0, 0);

    // write all of the hex chars
    int charsForThisPage = byteCount * 2;
    if (serialPortWrite(&device.serialPort, hexData, charsForThisPage) != charsForThisPage)
    {
        fwUpdate_sendProgress(IS_LOG_LEVEL_ERROR, "(ISB) Failed to write data to device");
        return IS_OP_ERROR;
    }

    int newVerifyChecksum = *verifyCheckSum;

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
        if (serialPortWrite(&device.serialPort, checkSumHex, 2) != 2)
        {
            fwUpdate_sendProgress(IS_LOG_LEVEL_ERROR, "(ISB) Failed to write checksum to device");
            return IS_OP_ERROR;
        }

        unsigned char buf[5] = { 0 };
        int count = serialPortReadTimeout(&device.serialPort, buf, 3, 1000);

        if (count == 3 && memcmp(buf, ".\r\n", 3) == 0)
        {
            break;
        }

        if (i == 9)
        {
            return IS_OP_ERROR;
        }
    }

    *totalBytes += byteCount;
    *currentOffset += byteCount;
    *verifyCheckSum = newVerifyChecksum;

    return IS_OP_OK;
}

is_operation_result ISBFirmwareUpdater::upload_hex(unsigned char* hexData, int charCount, int* currentOffset, int* currentPage, int* totalBytes, int* verifyCheckSum)
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
    if (*currentOffset + byteCount > FLASH_PAGE_SIZE)
    {
        int pageByteCount = FLASH_PAGE_SIZE - *currentOffset;

        if (upload_hex_page(hexData, pageByteCount, currentOffset, totalBytes, verifyCheckSum) != IS_OP_OK)
        {
            fwUpdate_sendProgress(IS_LOG_LEVEL_ERROR, "(ISB) Upload hex page error");
            return IS_OP_ERROR;
        }

        hexData += (pageByteCount * 2);
        charCount -= (pageByteCount * 2);
    }

    if (charCount != 0 && upload_hex_page(hexData, charCount / 2, currentOffset, totalBytes, verifyCheckSum) != IS_OP_OK)
    {
        return IS_OP_ERROR;
    }

    return IS_OP_OK;
}

is_operation_result ISBFirmwareUpdater::fill_current_page(int* currentPage, int* currentOffset, int* totalBytes, int* verifyCheckSum)
{
    (void)currentPage;

    if (*currentOffset < FLASH_PAGE_SIZE)
    {
        unsigned char hexData[256];
        memset(hexData, 'F', 256);

        while (*currentOffset < FLASH_PAGE_SIZE)
        {
            int byteCount = (FLASH_PAGE_SIZE - *currentOffset) * 2;
            if (byteCount > 256)
            {
                byteCount = 256;
            }
            memset(hexData, 'F', byteCount);

            if (upload_hex_page(hexData, byteCount / 2, currentOffset, totalBytes, verifyCheckSum) != IS_OP_OK)
            {
                fwUpdate_sendProgress(IS_LOG_LEVEL_ERROR, "(ISB) Failed to fill page with bytes");
                return IS_OP_ERROR;
            }
        }
    }

    return IS_OP_OK;
}

#define HEX_BUFFER_SIZE 1024

/**
 * Processes an Intel hex file, one chunk/line at a time. This function is meant to be called
 * repeatedly (in a step/thread, minimal-blocking fashion) until the entire file is processed.
 * @return true if the entire file has been processed, otherwise false (repeat until true)
 */
fwUpdate::update_status_e ISBFirmwareUpdater::step_loadHex() {
    // these are variables which probably need to be moved into member or static variables.
    // they should be initialized, and then will need to be references in subsequent calls.
    int currentPage = -1;
    int currentOffset = m_isb_props.app_offset;
    int totalBytes = m_isb_props.app_offset;
    int lastSubOffset = currentOffset;
    int subOffset;

    // local variables that can be volatile/non-static
    static unsigned char output[HEX_BUFFER_SIZE * 2]; // big enough to store an entire extra line of buffer if needed
    unsigned char* outputPtr = output;
    const unsigned char* outputPtrEnd = output + (HEX_BUFFER_SIZE * 2);
    unsigned char tmp[5];
    char line[HEX_BUFFER_SIZE];
    int verifyCheckSum = 5381;
    int lineLength, i, pad, outputSize = 0;


    if ((lineLength = is_isb_read_line(*imgStream, line)) == 0)
        return fwUpdate::FINALIZING;

    if (lineLength > 12 && line[7] == '0' && line[8] == '0') {
        if (lineLength > HEX_BUFFER_SIZE * 4) {
            fwUpdate_sendProgress(IS_LOG_LEVEL_ERROR, "(ISB) hex file line length too long");
            return fwUpdate::ERR_INVALID_IMAGE;
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
            fwUpdate_sendProgress(IS_LOG_LEVEL_ERROR, "(ISB) Line data overflowed output buffer");
            return fwUpdate::ERR_INVALID_IMAGE;
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
            return fwUpdate::IN_PROGRESS;   // we've done what we can, for now...
        }
        // upload this chunk
        if (upload_hex(output, _MIN(MAX_SEND_COUNT, outputSize), &currentOffset, &currentPage, &totalBytes, &verifyCheckSum) != IS_OP_OK) {
            fwUpdate_sendProgress(IS_LOG_LEVEL_ERROR, "(ISB) Error in upload chunk");
            return fwUpdate::ERR_INVALID_IMAGE;
        }

        outputSize -= MAX_SEND_COUNT;

        if (outputSize < 0 || outputSize > HEX_BUFFER_SIZE) {
            fwUpdate_sendProgress(IS_LOG_LEVEL_ERROR, "(ISB) Output size was too large (1)");
            return fwUpdate::ERR_INVALID_IMAGE;
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
            return fwUpdate::IN_PROGRESS;   // we've done what we can, for now...
        } else {
            lastSubOffset = 0;
        }

        outputSize = (int)(outputPtr - output);

        if (outputSize < 0 || outputSize > HEX_BUFFER_SIZE) {
            fwUpdate_sendProgress(IS_LOG_LEVEL_ERROR, "(ISB) Output size was too large (2)");
            return fwUpdate::ERR_INVALID_IMAGE;
        }

        // flush the remainder of data to the page
        if (upload_hex(output, outputSize, &currentOffset, &currentPage, &totalBytes, &verifyCheckSum) != IS_OP_OK) {
            fwUpdate_sendProgress(IS_LOG_LEVEL_ERROR, "(ISB) Error in upload hex");
            return fwUpdate::ERR_INVALID_IMAGE;
        }

        // fill remainder of the current page, the next time that bytes try to be written the page will be automatically incremented
        if (fill_current_page(&currentPage, &currentOffset, &totalBytes, &verifyCheckSum) != IS_OP_OK) {
            fwUpdate_sendProgress(IS_LOG_LEVEL_ERROR, "(ISB) Error in fill page");
            return fwUpdate::ERR_INVALID_IMAGE;
        }

        // change to the next page
        currentOffset = 0;
        if (select_page(currentPage) != IS_OP_OK || begin_program_for_current_page(0, FLASH_PAGE_SIZE - 1) != IS_OP_OK) {
            fwUpdate_sendProgress(IS_LOG_LEVEL_ERROR, "(ISB) Failed to issue select page or to start programming");
            return fwUpdate::ERR_INVALID_IMAGE;
        }

        // set the output ptr back to the beginning, no more data is in the queue
        outputPtr = output;
    }
    else if (lineLength > 10 && line[7] == '0' && line[8] == '1')
    {   // End of last page (end of file marker)
        outputSize = (int)(outputPtr - output);

        // flush the remainder of data to the page
        if (upload_hex(output, outputSize, &currentOffset, &currentPage, &totalBytes, &verifyCheckSum) != IS_OP_OK) {
            fwUpdate_sendProgress(IS_LOG_LEVEL_ERROR, "(ISB) Error in upload hex (last)");
            return fwUpdate::ERR_INVALID_IMAGE;
        }
        if (currentOffset != 0 && fill_current_page(&currentPage, &currentOffset, &totalBytes, &verifyCheckSum) != IS_OP_OK) {
            fwUpdate_sendProgress(IS_LOG_LEVEL_ERROR, "(ISB) Error in fill page (last)");
            return fwUpdate::ERR_INVALID_IMAGE;
        }

        outputPtr = output;
    }

    writeProgress = ((float)imgStream->tellg() / (float)session_image_size);
    if ((progress_interval > 0) && (nextProgressReport < current_timeMs())) {
        nextProgressReport = current_timeMs() + progress_interval;
        fwUpdate_sendProgress(IS_LOG_LEVEL_INFO, "Writing Flash");
    }

    return  fwUpdate::IN_PROGRESS;
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
            if (upload_hex(output, _MIN(MAX_SEND_COUNT, outputSize), &currentOffset, &currentPage, &totalBytes, &verifyCheckSum) != IS_OP_OK) {
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
            if (upload_hex(output, outputSize, &currentOffset, &currentPage, &totalBytes, &verifyCheckSum) != IS_OP_OK) {
                fwUpdate_sendProgress(IS_LOG_LEVEL_ERROR, "(ISB) Error in upload hex");
                return IS_OP_ERROR;
            }

            // fill remainder of the current page, the next time that bytes try to be written the page will be automatically incremented
            if (fill_current_page(&currentPage, &currentOffset, &totalBytes, &verifyCheckSum) != IS_OP_OK) {
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
            if (upload_hex(output, outputSize, &currentOffset, &currentPage, &totalBytes, &verifyCheckSum) != IS_OP_OK) {
                fwUpdate_sendProgress(IS_LOG_LEVEL_ERROR, "(ISB) Error in upload hex (last)");
                return IS_OP_ERROR;
            }
            if (currentOffset != 0 && fill_current_page(&currentPage, &currentOffset, &totalBytes, &verifyCheckSum) != IS_OP_OK) {
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
    if (serialPortWrite(&device.serialPort, programLine, 21) != 21)
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
