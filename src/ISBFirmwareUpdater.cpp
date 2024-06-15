/**
 * @file ISBFirmwareUpdater.cpp 
 * @brief ${BRIEF_DESC}
 *
 * @author Kyle Mallory on 5/29/24.
 * @copyright Copyright (c) 2024 Inertial Sense, Inc. All rights reserved.
 */

#include "ISBFirmwareUpdater.h"
#include "ISBootloaderBase.h"


// this is called internally by processMessage() to do the things to do, it should also be called periodically to send status updated, etc.
bool ISBFirmwareUpdater::fwUpdate_step(fwUpdate::msg_types_e msg_type, bool processed) {
    static int nextStep = 0;

    if (fwUpdate_getSessionStatus() == fwUpdate::NOT_STARTED)
        return false;

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
        return true;
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

    rebootToISB(5, 0, false);

    return 0;
}

// called internally (by the receiving device) to populate the dev_info_t struct for the requested device
bool ISBFirmwareUpdater::fwUpdate_queryVersionInfo(fwUpdate::target_t target_id, dev_info_t& dev_info) {
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
        rebootToISB(5, 0, false);
    }
    return fwUpdate::INITIALIZING;
}

// writes the indicated block of data (of len bytes) to the target and device-specific image slot, and with the specified offset
fwUpdate::update_status_e  ISBFirmwareUpdater::fwUpdate_writeImageChunk(fwUpdate::target_t target_id, int slot_id, int offset, int len, uint8_t *data) {
    // we want write our data into a iostream that we'll also pass to the DFUDevice; it will help buffer our data
    imgBuffer->insert(offset, data, len);

    // check if we have enough to write the next page...


    return fwUpdate::IN_PROGRESS;
}

// this marks the finish of the upgrade, that all image bytes have been received, the md5 sum passed, the device can complete the requested upgrade, and perform any device-specific finalization
fwUpdate::update_status_e  ISBFirmwareUpdater::fwUpdate_finishUpdate(fwUpdate::target_t target_id, int slot_id, int flags) {
    delete imgStream;
    delete imgBuffer;
    return fwUpdate::FINISHED;
    // return fwUpdate::ERR_INVALID_SESSION;
}


// called internally to transmit data to back to the host
bool ISBFirmwareUpdater::fwUpdate_writeToWire(fwUpdate::target_t target, uint8_t* buffer, int buff_len) {
    while (buff_len--)
        toHost.push_back(*buffer++);

    return true;
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
            if (statusCb) statusCb(this, IS_LOG_LEVEL_INFO, "(ISB) check_is_compatible response missing.");
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
        if (statusCb) statusCb(this, IS_LOG_LEVEL_INFO, msg);
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
        if (statusCb) statusCb(this, IS_LOG_LEVEL_INFO,"(ISB) check_is_compatible no valid signature.");
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

is_operation_result ISBFirmwareUpdater::rebootToISB(uint8_t major, char minor, bool force)
{
    (void)force;
    (void)minor;
    (void)major;

    if (statusCb) statusCb(this, IS_LOG_LEVEL_INFO, "(APP) Rebooting to IS-bootloader mode...");

    // In case we are in program mode, try and send the commands to go into bootloader mode
    uint8_t c = 0;

    for (size_t loop = 0; loop < 10; loop++)
    {
        if (!serialPortWriteAscii(&device.serialPort, "STPB", 4)) break;     // If the write fails, assume the device is now in bootloader mode.
        if (!serialPortWriteAscii(&device.serialPort, "BLEN", 4)) break;
        c = 0;
        if (serialPortReadCharTimeout(&device.serialPort, &c, 13) == 1)
        {
            if (c == '$')
            {
                // done, we got into bootloader mode
                break;
            }
        }
        else serialPortFlush(&device.serialPort);
    }

    return IS_OP_OK;
}

is_operation_result ISBFirmwareUpdater::rebootToAPP() {
    if (statusCb) statusCb(this, IS_LOG_LEVEL_INFO, "(ISB) Rebooting to APP mode...");

    // send the "reboot to program mode" command and the device should start in program mode
    serialPortWrite(&device.serialPort, (unsigned char*)":020000040300F7", 15);
    serialPortFlush(&device.serialPort);
    SLEEP_MS(100);
    serialPortClose(&device.serialPort);
    return IS_OP_OK;
}


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

is_operation_result ISBFirmwareUpdater::erase_flash()
{
    // give the device 60 seconds to erase flash before giving up
    unsigned char selectFlash[24];

    // Write location to erase at
    memcpy(selectFlash, ":03000006030000F4CC\0\0\0\0\0", 24);
    checksum(0, selectFlash, 1, 17, 17, 1);
    if (serialPortWriteAndWaitForTimeout(&device.serialPort, selectFlash, 19, (unsigned char*)".\r\n", 3, BOOTLOADER_TIMEOUT_DEFAULT) == 0) return IS_OP_ERROR;

    // Erase
    memcpy(selectFlash, ":0200000400FFFBCC\0", 18);
    checksum(0, selectFlash, 1, 15, 15, 1);
    serialPortWrite(&device.serialPort, selectFlash, 17);

    // Check for response and allow quit (up to 60 seconds)
    uint8_t buf[128];
    uint8_t *bufPtr = buf;
    int count = 0;
    for(size_t i = 0; i < 600; i++)
    {
        count += serialPortReadTimeout(&device.serialPort, bufPtr, 3, 100);
        bufPtr = buf + count;

        if (progressCb(this, 0.0f, "Erasing Flash", 0, 1) != IS_OP_OK)
        {
            return IS_OP_CANCELLED;
        }
        if (count == 3 && memcmp(buf, ".\r\n", 3) == 0)
        {
            return IS_OP_OK;
        }
    }

    if (statusCb) statusCb((void*)this, IS_LOG_LEVEL_ERROR, "(ISB) Error in erase flash");
    return IS_OP_ERROR;
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
