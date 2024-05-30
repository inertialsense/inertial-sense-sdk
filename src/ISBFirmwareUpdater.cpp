/**
 * @file ISBFirmwareUpdater.cpp 
 * @brief ${BRIEF_DESC}
 *
 * @author Kyle Mallory on 5/29/24.
 * @copyright Copyright (c) 2024 Inertial Sense, Inc. All rights reserved.
 */

#include "ISBFirmwareUpdater.h"
#include "ISBootloaderBase.h"


ISBFirmwareUpdater::eImageSignature ISBFirmwareUpdater::check_is_compatible()
{
    uint8_t buf[14] = { 0 };
    int count = 0;

    serialPortFlush(m_port);
    serialPortRead(m_port, buf, sizeof(buf));    // empty Rx buffer
    sync(m_port);

    SLEEP_MS(100);

    for (int retry=0;; retry++)
    {
        // Send command
        serialPortFlush(m_port);
        serialPortRead(m_port, buf, sizeof(buf));    // empty Rx buffer
        serialPortWrite(m_port, (uint8_t*)":020000041000EA", 15);

        // Read Version, SAM-BA Available, serial number (in version 6+) and ok (.\r\n) response
#define READ_DELAY_MS   500
        count = serialPortReadTimeout(m_port, buf, 14, READ_DELAY_MS);

        if (count >= 8 && buf[0] == 0xAA && buf[1] == 0x55)
        {
            break;
        }

        if (retry*READ_DELAY_MS > 4000)
        {   // No response
            // FIXME: m_info_callback(NULL, IS_LOG_LEVEL_ERROR, "    | (ISB Error) (%s) check_is_compatible response missing.", m_port->port);
            if (statusCb) statusCb(this, IS_LOG_LEVEL_INFO, "(ISB Error) (%s) check_is_compatible response missing.", m_port->port);
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
        int n = SNPRINTF(msg, sizeof(msg), "    | (ISB Error) (%s) check_is_compatible parse error:\n 0x ", m_port->port);
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
        if (statusCb) statusCb(this, IS_LOG_LEVEL_INFO,"(ISB Error) (%s) check_is_compatible no valid signature.", m_port->port);
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
        if(serialPortWrite(m_port, &handshakerChar, 1) != 1)
        {
            return IS_OP_ERROR;
        }

        if (serialPortWaitForTimeout(m_port, &handshakerChar, 1, BOOTLOADER_RESPONSE_DELAY))
        {	// Success
            return IS_OP_OK;
        }
    }

#if defined(SUPPORT_BOOTLOADER_V5A)     // ONLY NEEDED TO SUPPORT BOOTLOADER v5a.  Delete this and associated code in Q4 2022 after bootloader v5a is out of circulation. WHJ
    static const unsigned char handshaker[] = "INERTIAL_SENSE_SYNC_DFU";

    // Attempt handshake using extended string for bootloader v5a
    for (int i = 0; i < BOOTLOADER_RETRIES; i++)
    {
        if (serialPortWriteAndWaitForTimeout(m_port, (const unsigned char*)&handshaker, (int)sizeof(handshaker), &handshakerChar, 1, BOOTLOADER_RESPONSE_DELAY))
        {	// Success
            return IS_OP_OK;
        }
    }
#endif

    return IS_OP_ERROR;
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
    if (serialPortWriteAndWaitForTimeout(m_port, selectFlash, 19, (unsigned char*)".\r\n", 3, BOOTLOADER_TIMEOUT_DEFAULT) == 0) return IS_OP_ERROR;

    // Erase
    memcpy(selectFlash, ":0200000400FFFBCC\0", 18);
    checksum(0, selectFlash, 1, 15, 15, 1);
    serialPortWrite(m_port, selectFlash, 17);

    // Check for response and allow quit (up to 60 seconds)
    uint8_t buf[128];
    uint8_t *bufPtr = buf;
    int count = 0;
    for(size_t i = 0; i < 600; i++)
    {
        count += serialPortReadTimeout(m_port, bufPtr, 3, 100);
        bufPtr = buf + count;

        if (progressCb(this, "ERASE", 0, 1, 0.0f) != IS_OP_OK)
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
