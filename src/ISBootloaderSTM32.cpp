/*
MIT LICENSE

Copyright (c) 2014-2025 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include "ISBootloaderSTM32.h"
#include "ihex.h"
#include "ISUtilities.h"
#include "serialPortPlatform.h"

#include <time.h>
#include <stddef.h>
#include <mutex>

using namespace ISBootloader;

typedef enum
{
    STM32_GET = 0x00,               // Uses stm32_get_t
    STM32_GET_VERSION = 0x01,       // Uses stm32_get_t
    STM32_GET_ID = 0x02,            // Uses stm32_get_t
    STM32_READ_MEMORY = 0x11,       // Uses stm32_data_t
    STM32_GO = 0x21,                // Uses stm32_go_t
    STM32_WRITE_MEMORY = 0x31,      // Uses stm32_data_t    
    STM32_ERASE = 0x43,             // Uses stm32_erase_t
    STM32_EXTENDED_ERASE = 0x44,
    STM32_SPECIAL = 0x50,           
    STM32_EXTENDED_SPECIAL = 0x51,
    STM32_WRITE_PROTECT = 0x63,
    STM32_WRITE_UNPROTECT = 0x73,
    STM32_READOUT_PROTECT = 0x82,
    STM32_READOUT_UNPROTECT = 0x92,
    STM32_GET_CHECKSUM = 0xA1,
} eSTM32RomCommands;

enum
{
    STM32_AUTOBAUD = 0x7F,
    STM32_ACK = 0x79,
    STM32_NACK = 0x1F,
};

is_operation_result cISBootloaderSTM32::match_test(void* param)
{
    const char* serial_name = (const char*)param;

    if(strnlen(serial_name, 100) != 0 && strncmp(serial_name, m_port->port, 100) == 0)
    {
        return IS_OP_OK;
    }

    return IS_OP_ERROR;
}

uint8_t cISBootloaderSTM32::check_is_compatible(uint32_t imgSign)
{
    // Close and reopen the port with even parity and 115200 baud
    serialPortClose(m_port);
    serialPortSetOptions(m_port, OPT_PARITY_EVEN);
    serialPortOpen(m_port, m_port->port, 115200U, 1);

    // Flush the bootloader command buffer
    serialPortFlush(m_port);

    // Set non-interactive mode and wait for response, but don't check so we can continue if it fails
    serialPortWriteAndWaitForTimeout(m_port, (const uint8_t*)STM32_AUTOBAUD, 1, (const uint8_t*)STM32_ACK, 1, 1000U);

    // Read the device ID
    if(get_id() != STM32_ACK) return IS_IMAGE_SIGN_NONE;

    // Check the device ID
    switch(m_pid)
    {
    case 0x62: 
        m_info_callback(this, "Detected STM32L4 device", IS_LOG_LEVEL_DEBUG);
        return IS_IMAGE_SIGN_STM_L4;   // STM32L452 (IMX-5)
    case 0x82: 
        m_info_callback(this, "Detected STM32U5 device", IS_LOG_LEVEL_DEBUG);
        return IS_IMAGE_SIGN_STM_U5;   // STM32U575/STM32U585 (GPX-1/IMX-6)
    default: 
        m_info_callback(this, "No STM32 device detected", IS_LOG_LEVEL_DEBUG);
        return IS_IMAGE_SIGN_NONE;
    }

    return IS_IMAGE_SIGN_NONE;
}

is_operation_result cISBootloaderSTM32::reboot_up()
{
    // Jump to the application in FLASH memory
    if(go(0x08000000) != STM32_ACK)
    {
        m_info_callback(this, "Failed to jump to application in FLASH memory", IS_LOG_LEVEL_ERROR);
        return IS_OP_ERROR;
    }

    // Close the port and reset the options
    serialPortClose(m_port);
    serialPortSetOptions(m_port, OPT_PARITY_NONE);

    return IS_OP_OK;
}

uint32_t cISBootloaderSTM32::get_device_info()
{
    // TODO: Read the device serial number from OTP

    return 0U;
}

is_operation_result cISBootloaderSTM32::download_image(void)
{
    ihex_image_section_t image[MAX_NUM_IHEX_SECTIONS];

    // Load the firmware image from the Intel HEX file
    const size_t numSections = ihex_load_sections(m_filename.c_str(), image, MAX_NUM_IHEX_SECTIONS);
    if(numSections <= 0) return IS_OP_ERROR;

    uint32_t totalLen = 0U;         // Holds the total length of the firmware image
    uint32_t bytesWritten = 0U;     // Holds the number of bytes written to the device
    for(size_t i = 0U; i < numSections; i++)
    {
        totalLen += image[i].len;
    }

    status_update("(STM) Erasing flash...", IS_LOG_LEVEL_INFO);
    
    // Perform the erase operation
    if(mass_erase() != STM32_ACK)
    {
        ihex_unload_sections(image, numSections);
        return IS_OP_ERROR;
    }

    status_update("(STM) Programming flash...", IS_LOG_LEVEL_INFO);

    uint8_t dataBuf[256];
    stm32_data_t payload; payload.data = dataBuf;

    // Write memory
    for(size_t i = 0; i < numSections; i++)
    {
        uint32_t bytesLeft = image[i].len;
        uint32_t offset = 0U;
        uint8_t retries = 0U;

        // Check the address range
        switch(m_pid)
        {
        case 0x62:      // STM32L452 (IMX-5)
            // TODO: Add support for other areas.
            if(image[i].address < 0x08000000 || (image[i].address + image[i].len) > 0x0807FFFF)
            {
                m_info_callback(this, "Invalid address range for STM32L4 device", IS_LOG_LEVEL_WARN);
                continue;
            }
            break;
        case 0x82:      // STM32U575/STM32U585 (GPX-1/IMX-6)
            // TODO: Add support for other areas.
            if(image[i].address < 0x08000000 || (image[i].address + image[i].len) > 0x08200000)
            {
                m_info_callback(this, "Invalid address range for STM32U5 device", IS_LOG_LEVEL_WARN);
                continue;
            }
            break;
        default: 
            m_info_callback(this, "No STM32 device detected", IS_LOG_LEVEL_DEBUG);
            ihex_unload_sections(image, numSections);
            return IS_OP_ERROR;
        }

        while (bytesLeft > 0 && bytesLeft < MAX_IHEX_SECTION_LEN)
        {
            // Payload length is one less than actual length because of STM32 protocol
            payload.len = 0xFF;
            if (bytesLeft < payload.len) payload.len = bytesLeft - 1;

            // Set the address to write at
            payload.addr = image[i].address + offset;

            // Copy image into buffer for transmission
            memcpy(payload.data, &image[i].image[offset], (size_t)payload.len + 1);

            // Write the memory
            if(write_memory(&payload) != STM32_ACK && ++retries > 3) 
            {
                ihex_unload_sections(image, numSections);
                return IS_OP_ERROR;
            }

            retries = 0U;

            offset += (uint32_t)payload.len + 1;
            bytesLeft -= (uint32_t)payload.len + 1;
            bytesWritten += (uint32_t)payload.len + 1;

            m_update_progress = 0.25f + 0.75f * ((float)bytesWritten / (float)totalLen);
            m_update_callback(this, m_update_progress);
        } 
    }

    // Unload the firmware image
    ihex_unload_sections(image, numSections);

    return IS_OP_OK;
}

uint8_t cISBootloaderSTM32::send_command(uint8_t cmd)
{
    if((m_port->options & OPT_PARITY_MASK) != OPT_PARITY_EVEN)
    {
        serialPortClose(m_port);
        serialPortSetOptions(m_port, OPT_PARITY_EVEN);
        serialPortOpen(m_port, m_port->port, 115200U, 1);
    }

    uint8_t cmdbuf[2] = { cmd, cmd ^ 0xFF };
    uint8_t resp;

    // Send command and inverted command bytes
    serialPortFlush(m_port);
    serialPortWrite(m_port, cmdbuf, sizeof(cmdbuf));

    int ackLen = serialPortReadTimeout(m_port, &resp, 1, 1000U);
    if(ackLen != 1 || resp != STM32_ACK) return STM32_NACK;

    return STM32_ACK;

}

uint8_t cISBootloaderSTM32::get(void)
{
    // Send the command and check for ACK
    if(send_command(STM32_GET) != STM32_ACK) return STM32_NACK;

    // Read the first byte of the response to get the length
    uint8_t respBuf[32];
    uint8_t xor = 0;
    int respLen = serialPortReadTimeout(m_port, respBuf, 1, 1000U);
    if(respLen != 1) return STM32_NACK;

    // Compute the checksum and get the length of the response
    xorCompute(&xor, respBuf, 1);
    uint16_t bytesLeft = respBuf[0] + 1;
    
    // Read the rest of the response
    respLen = serialPortReadTimeout(m_port, respBuf, bytesLeft, 1000U);
    if(respLen != bytesLeft) return STM32_NACK;
    xorCompute(&xor, respBuf, respLen);

    // Read and check the checksum
    uint8_t csum;
    respLen = serialPortReadTimeout(m_port, &csum, 1, 100U);
    if(respLen != 1 || csum != xor) return STM32_NACK;

    // Populate the list of valid commands in the class
    if(bytesLeft > sizeof(m_valid_commands)) bytesLeft = sizeof(m_valid_commands);
    memset(m_valid_commands, 0U, sizeof(m_valid_commands));
    memcpy(m_valid_commands, respBuf, bytesLeft);
    
    return STM32_ACK;
}

uint8_t cISBootloaderSTM32::get_version(void)
{
    // Send the command and check for ACK
    if(send_command(STM32_GET_VERSION) != STM32_ACK) return STM32_NACK;

    // Read the first two bytes of the response to get the length
    uint8_t respBuf[32];
    uint8_t xor = 0;
    int respLen = serialPortReadTimeout(m_port, respBuf, 4, 1000U);
    if(respLen != 4 || respBuf[3] != STM32_ACK) return STM32_NACK;

    m_version = respBuf[0];
    
    return STM32_ACK;
}

uint8_t cISBootloaderSTM32::get_id(void)
{
    // Send the command and check for ACK
    if(send_command(STM32_GET_ID) != STM32_ACK) return STM32_NACK;

    // Read the first byte of the response to get the length
    uint8_t respBuf[10];
    uint8_t xor = 0;
    int respLen = serialPortReadTimeout(m_port, respBuf, 1, 1000U);
    if(respLen != 1) return STM32_NACK;

    // Read the id
    uint8_t lenVer = respBuf[0] + 2;    // 2 for checksum, 1 for length
    if(lenVer > sizeof(respBuf)) lenVer = sizeof(respBuf);
    respLen = serialPortReadTimeout(m_port, respBuf, lenVer, 1000U);
    if(respLen != lenVer || respBuf[lenVer] != STM32_ACK) return STM32_NACK;

    // Copy the PID into the class (respBuf[0] is always 0x04 for all STM32 devices)
    m_pid = respBuf[1];
    
    return STM32_ACK;
}

/** Memory manipulation commands */
uint8_t cISBootloaderSTM32::mass_erase(void)
{
    // Send the command and check for ACK
    if(send_command(STM32_ERASE) != STM32_ACK) return STM32_NACK;

    // Send the mass erase command
    uint8_t cmd[2] = { 0xFF, 0x00 };
    serialPortWrite(m_port, cmd, 2);

    if(checkAck() != STM32_ACK) return STM32_NACK;

    return STM32_ACK;
}

uint8_t cISBootloaderSTM32::read_memory(stm32_data_t *data)
{
    if(data == NULL || data->data == NULL) return STM32_NACK;

    // Send the command and check for ACK
    if(send_command(STM32_READ_MEMORY) != STM32_ACK) return STM32_NACK;

    // Send address to jump to
    uint8_t buf[5];
    addrBufCopy(data->addr, buf);
    serialPortWrite(m_port, buf, sizeof(buf));

    // Wait for ACK
    if(checkAck() != STM32_ACK) return STM32_NACK;

    // Send the number of bytes to read with a checksum
    uint8_t len[2] = { data->len - 1, 0x00 };
    xorCompute(&len[1], &len[0], 1);
    serialPortWrite(m_port, len, 2);
    
    // Wait for ACK
    if(checkAck() != STM32_ACK) return STM32_NACK;

    // Read memory into buffer
    int respLen = serialPortReadTimeout(m_port, data->data, data->len, 1000U);
    if(respLen != data->len) return STM32_NACK;
    
    // Make sure the checksum is correct
    uint8_t chksum = 0, lastbyte;
    xorCompute(&chksum, data->data, data->len);
    respLen = serialPortReadTimeout(m_port, &lastbyte, 1, 1000U);
    xorCompute(&chksum, &lastbyte, 1);
    if(respLen != 1 || chksum != 0) return STM32_NACK;

    return STM32_ACK;
}

uint8_t cISBootloaderSTM32::write_memory(stm32_data_t *data)
{
    // Send the command and check for ACK
    if(send_command(STM32_WRITE_MEMORY) != STM32_ACK) return STM32_NACK;

    // Send address to jump to
    uint8_t buf[5];
    addrBufCopy(data->addr, buf);
    serialPortWrite(m_port, buf, sizeof(buf));

    // Wait for ACK
    if(checkAck() != STM32_ACK) return STM32_NACK;

    // Send the number of bytes to write
    uint8_t len = data->len - 1;
    uint8_t chksum = 0;
    serialPortWrite(m_port, &len, 1);
    xorCompute(&chksum, &len, 1);

    // Send the bytes to write
    serialPortWrite(m_port, data->data, data->len);
    xorCompute(&chksum, data->data, data->len);

    // Write the checksum
    serialPortWrite(m_port, &chksum, 1);
    
    // Wait for ACK
    if(checkAck() != STM32_ACK) return STM32_NACK;

    return STM32_ACK;
}

uint8_t cISBootloaderSTM32::go(uint32_t addr)
{
    // Send the command and check for ACK
    if(send_command(STM32_GO) != STM32_ACK) return STM32_NACK;

    // Send address to jump to
    uint8_t buf[5];
    addrBufCopy(addr, buf);
    serialPortWrite(m_port, buf, sizeof(buf));

    if(checkAck() != STM32_ACK) return STM32_NACK;
    
    return STM32_ACK;
}

uint8_t cISBootloaderSTM32::addrBufCopy(uint32_t addr, uint8_t *buf)
{
    buf[0] = (uint8_t) addr >> 24;
    buf[1] = (uint8_t) addr >> 16;
    buf[2] = (uint8_t) addr >> 8;
    buf[3] = (uint8_t) addr;
    xorCompute(&buf[4], buf, 4);
}

uint8_t cISBootloaderSTM32::checkAck(void)
{
    uint8_t respBuf[1];
    int respLen = serialPortReadTimeout(m_port, respBuf, 1, 1000U);
    if(respLen != 1 || respBuf[0] != STM32_ACK) return STM32_NACK;
    
    return STM32_ACK;
}

void cISBootloaderSTM32::xorCompute(uint8_t *chksum, uint8_t *data, uint16_t len)
{
    for(uint16_t i = 0; i < len; i++)
    {
        *chksum ^= data[i];
    }
}
