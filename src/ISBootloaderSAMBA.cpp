/**
 * @file ISBootloaderSAMBA.cpp
 * @author Dave Cutting (davidcutting42@gmail.com)
 * @brief Inertial Sense routines for updating ISB (Inertial Sense Bootloader)
 *  images using the SAM-BA protocol.
 * 
 */

/*
MIT LICENSE

Copyright (c) 2014-2023 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include "ISBootloaderSAMBA.h"
#include "ihex.h"
#include "ISUtilities.h"
#include "serialPortPlatform.h"

#include <time.h>
#include <stddef.h>
#include <mutex>

using namespace ISBootloader;

// Resources for SAM-BA protocol:
//  - Datasheet ROM boot section
//  - https://sourceforge.net/p/lejos/wiki-nxt/SAM-BA%20Protocol/

#define UART_XMODEM_SOH 0x01
#define UART_XMODEM_EOT 0x04
#define UART_XMODEM_ACK 0x06
#define UART_XMODEM_NAK 0x15
#define UART_XMODEM_CAN 0x18
#define UART_XMODEM_CRC_POLY 0x1021
#define UART_XMODEM_PAYLOAD_SIZE 128

#define SAMBA_BAUDRATE 115200
#define SAMBA_TIMEOUT_DEFAULT 1000
#define SAMBA_FLASH_START_ADDRESS 0x00400000
#define SAMBA_BOOTLOADER_SIZE_24K 0x6000

#define SAMBA_STATUS(x, level) m_info_callback(this, x, level)
#define SAMBA_ERROR_CHECK(x, error) if(x != IS_OP_OK) \
    { /* serialPortClose(port); */ \
        SAMBA_STATUS(error, IS_LOG_LEVEL_ERROR); \
        return IS_OP_ERROR; \
    }
#define SAMBA_ERROR_CHECK_SN(x, error) if(x != IS_OP_OK) \
    { /* serialPortClose(port); */ \
        SAMBA_STATUS(error, IS_LOG_LEVEL_ERROR); \
        return 0; \
    }

PUSH_PACK_1
typedef struct
{
    uint8_t start;
    uint8_t block;
    uint8_t block_neg;
    uint8_t payload[UART_XMODEM_PAYLOAD_SIZE];
    uint16_t crc;
} xmodem_chunk_t;
POP_PACK

is_operation_result cISBootloaderSAMBA::match_test(void* param)
{
    const char* serial_name = (const char*)param;

    if(strnlen(serial_name, 100) != 0 && strncmp(serial_name, m_port->port, 100) == 0)
    {
        return IS_OP_OK;
    }

    return IS_OP_ERROR;
}

eImageSignature cISBootloaderSAMBA::check_is_compatible()
{
    int count = 0;

    // Flush the bootloader command buffer
    serialPortWrite(m_port, (uint8_t*)"#", 1);
    serialPortSleep(m_port, 10);
    serialPortFlush(m_port);

    // Set non-interactive mode and wait for response
    count = serialPortWriteAndWaitForTimeout(m_port, (const uint8_t*)"N#", 2, (const uint8_t*)"\n\r", 2, 100);
    
    if (!count)
    {   // Failed to handshake with bootloader
        return IS_IMAGE_SIGN_NONE;
    }

    return IS_IMAGE_SIGN_SAMBA;
}

is_operation_result cISBootloaderSAMBA::reboot()
{
    // RSTC_CR, RSTC_CR_KEY_PASSWD | RSTC_CR_PROCRST
    write_word(0x400e1800, 0xa5000001);
    return IS_OP_OK;
}

is_operation_result cISBootloaderSAMBA::reboot_up()
{
    m_info_callback(this, "(SAM-BA) Rebooting to ISB mode...", IS_LOG_LEVEL_INFO);

    // EEFC.FCR, EEFC_FCR_FKEY_PASSWD | EEFC_FCR_FARG_BOOT | EEFC_FCR_FCMD_SGPB
    if (write_word(0x400e0c04, 0x5a00010b) == IS_OP_OK)
    {
        wait_eefc_ready(true);
        
        reboot();

        serialPortSleep(m_port, 500);

        return IS_OP_OK;
    }
    return IS_OP_ERROR;
}

is_operation_result cISBootloaderSAMBA::download_image(std::string filename)
{
    serial_port_t* port = m_port;

    // https://github.com/atmelcorp/sam-ba/tree/master/src/plugins/connection/serial
    // https://sourceforge.net/p/lejos/wiki-nxt/SAM-BA%20Protocol/
    uint8_t buf[SAMBA_PAGE_SIZE];

    SAMBA_ERROR_CHECK(erase_flash(), "(SAM-BA) Failed to erase flash memory");

    // try non-USB and then USB mode (0 and 1)
    for(int isUSB = 0; isUSB < 2; isUSB++)
    {
        serialPortSleep(port, 250);
        serialPortFlush(port);

        // flush
        serialPortWrite(port, (const uint8_t*)"#", 2);
        serialPortReadTimeout(port, buf, sizeof(buf), 100);

        FILE* file;
#ifdef _MSC_VER
        fopen_s(&file, filename.c_str(), "rb");
#else
        file = fopen(filename.c_str(), "rb");
#endif

        if (file == 0)
        {
            SAMBA_STATUS("(SAM-BA) Unable to load bootloader file", IS_LOG_LEVEL_ERROR);
            // serialPortClose(port);
            return IS_OP_ERROR;
        }

        fseek(file, 0, SEEK_END);
        int size = ftell(file);
        fseek(file, 0, SEEK_SET);
        checksum = 0;

        if (size != SAMBA_BOOTLOADER_SIZE_24K)
        {
            SAMBA_STATUS("(SAM-BA) Invalid or old (v5 or earlier) bootloader file", IS_LOG_LEVEL_ERROR);
            // serialPortClose(port);
            return IS_OP_ERROR;
        }

        if(isUSB == 0) SAMBA_STATUS("(SAM-BA) Writing ISB bootloader...", IS_LOG_LEVEL_INFO);

        uint32_t offset = 0;
        size_t len;
        while ((len = fread(buf, 1, SAMBA_PAGE_SIZE, file)) == SAMBA_PAGE_SIZE)
        {
            if (flash_erase_write_page(offset, buf, isUSB) != IS_OP_OK)
            {
                if (!isUSB) { offset = 0; break; } // try USB mode
                SAMBA_STATUS("Failed to upload page", IS_LOG_LEVEL_ERROR);
                // serialPortClose(port);
                return IS_OP_ERROR;
            }
            for (uint32_t* ptr = (uint32_t*)buf, *ptrEnd = (uint32_t*)(buf + sizeof(buf)); ptr < ptrEnd; ptr++)
            {
                checksum ^= *ptr;
            }
            offset += SAMBA_PAGE_SIZE;
            
            m_update_progress = (float)offset / (float)SAMBA_BOOTLOADER_SIZE_24K;
            if (m_update_callback != 0)
            {
                m_update_callback(this, m_update_progress);
            }
        }
        fclose(file);
        if (offset != 0) break; // success!
    }

    return IS_OP_OK;
}

is_operation_result cISBootloaderSAMBA::erase_flash()
{
    SAMBA_STATUS("(SAM-BA) Erasing flash memory...", IS_LOG_LEVEL_INFO);
    
    // Erase 3 sectors of 16 pares each (8K)
    if (write_word(0x400e0c04, 0x5a000207) == IS_OP_OK)
    {
        SLEEP_MS(200);    // From datasheet, max time it could take
        wait_eefc_ready(true);
    }
    if (write_word(0x400e0c04, 0x5a001207) == IS_OP_OK)
    {
        SLEEP_MS(200);    // From datasheet, max time it could take
        wait_eefc_ready(true);
    }
    if (write_word(0x400e0c04, 0x5a002207) == IS_OP_OK)
    {
        SLEEP_MS(200);    // From datasheet, max time it could take
        return wait_eefc_ready(true);
    }

    return IS_OP_ERROR;
}

uint32_t cISBootloaderSAMBA::get_device_info()
{
    serial_port_t* port = m_port;
    uint8_t buf[SAMBA_PAGE_SIZE];

    // Flush the bootloader command buffer
    serialPortWrite(port, (const uint8_t*)"#", 2);
    int count = serialPortReadTimeout(port, buf, sizeof(buf), 100);

    // Set non-interactive mode and wait for response
    count = serialPortWriteAndWaitFor(port, (const uint8_t*)"N#", 2, (const uint8_t*)"\n\r", 2);
    if (!count)
    {   // Failed to handshake with bootloader
        // serialPortClose(port);
        return 0;
    }

    // Set flash mode register
    SAMBA_ERROR_CHECK_SN(write_word(0x400e0c00, 0x04000600), "Failed to set flash mode register");

    // Set flash command to STUS (start read unique signature)
    SAMBA_ERROR_CHECK_SN(write_word(0x400e0c04, 0x5a000014), "Failed to command signature readout");
    
    // Wait until EEFC.FSR.FRDY is cleared
    SAMBA_ERROR_CHECK_SN(wait_eefc_ready(false), "Failed to clear flash ready bit");

    // Read out the unique identifier
    SAMBA_ERROR_CHECK_SN(read_word(0x00400000 + offsetof(manufacturing_info_t, serialNumber), &m_sn), "Failed to read UID word");
    
    // Set flash command to SPUS (stop read unique identifier)
    SAMBA_ERROR_CHECK_SN(write_word(0x400e0c04, 0x5a000015), "Failed to command stop signature readout");
    
    return m_sn;
}

is_operation_result cISBootloaderSAMBA::read_word(uint32_t address, uint32_t* word)
{
    uint8_t buf[16];
    int count = SNPRINTF((char*)buf, sizeof(buf), "w%08x,#", address);
    if ((serialPortWrite(m_port, buf, count) == count) &&
        (serialPortReadTimeout(m_port, buf, sizeof(uint32_t), SAMBA_TIMEOUT_DEFAULT) == sizeof(uint32_t)))
    {
        *word = (buf[3] << 24) | (buf[2] << 16) | (buf[1] << 8) | buf[0];
        return IS_OP_OK;
    }
    return IS_OP_ERROR;
}

is_operation_result cISBootloaderSAMBA::write_word(uint32_t address, uint32_t word)
{
    unsigned char buf[32];
    int count = SNPRINTF((char*)buf, sizeof(buf), "W%08x,%08x#", address, word);
    if(serialPortWrite(m_port, buf, count) != count) return IS_OP_ERROR;
    return IS_OP_OK;
}

is_operation_result cISBootloaderSAMBA::wait_eefc_ready(bool waitReady)
{
    uint32_t status = 0;
    for (int i = 0; i < 10; i++)
    {
        // EEFC.FSR.FRDY - Flash ready status
        if(read_word(0x400e0c08, &status) == IS_OP_ERROR) continue; 
        if( waitReady &&  (status & 0x01)) return IS_OP_OK;   // status is ready and we are waiting for ready
        if(!waitReady && !(status & 0x01)) return IS_OP_OK;   // status is not ready and we are waiting for not ready
        serialPortSleep(m_port, 20);
    }
    return IS_OP_ERROR;
}

is_operation_result cISBootloaderSAMBA::write_uart_modem(uint8_t* buf, size_t len)
{
    int ret;
    uint8_t eot = UART_XMODEM_EOT;
    uint8_t answer;
    xmodem_chunk_t chunk = {0};
    chunk.block = 1;
    chunk.start = UART_XMODEM_SOH;

    // wait for ping from bootloader
    do
    {
        if (serialPortRead(m_port, &answer, sizeof(uint8_t)) != sizeof(uint8_t))
        {
            return IS_OP_ERROR;
        }
    } while (answer != 'C');

    serialPortFlush(m_port);

    // write up to one sector
    while (len)
    {
        size_t z = 0;
        
        z = _MIN(len, sizeof(chunk.payload));
        memcpy(chunk.payload, buf, z);
        memset(chunk.payload + z, 0xff, sizeof(chunk.payload) - z);

        chunk.crc = SWAP16(crc16(chunk.payload, sizeof(chunk.payload)));
        chunk.block_neg = 0xff - chunk.block;

        ret = serialPortWrite(m_port, (const uint8_t*)&chunk, sizeof(xmodem_chunk_t));
        if (ret != sizeof(xmodem_chunk_t))
        {
            return IS_OP_ERROR;
        }

        ret = serialPortReadTimeout(m_port, &answer, sizeof(uint8_t), SAMBA_TIMEOUT_DEFAULT);
        if (ret != sizeof(uint8_t))
        {
            return IS_OP_ERROR;
        }

        switch (answer)
        {
        case UART_XMODEM_ACK:
            chunk.block++;
            len -= z;
            buf += z;
            break;
        case UART_XMODEM_CAN:
            return IS_OP_ERROR;
        default:
        case UART_XMODEM_NAK:
            break;
        }
    }

    ret = serialPortWrite(m_port, &eot, sizeof(uint8_t));
    if (ret != sizeof(uint8_t))
    {
        return IS_OP_ERROR;
    }
    ret = serialPortReadCharTimeout(m_port, &eot, SAMBA_TIMEOUT_DEFAULT);
    if (ret == 0 || eot != UART_XMODEM_ACK)
    {
        return IS_OP_ERROR;
    }
    
    return IS_OP_OK;
}

is_operation_result cISBootloaderSAMBA::flash_erase_write_page(size_t offset, uint8_t data[SAMBA_PAGE_SIZE], bool isUSB)
{
    uint16_t page = (uint16_t)(offset / SAMBA_PAGE_SIZE);
    uint8_t buf[32];
    int count;

    count = SNPRINTF((char*)buf, sizeof(buf), "S%08x,%08x#",
                     (unsigned int)(SAMBA_FLASH_START_ADDRESS + offset),
                     (unsigned int)SAMBA_PAGE_SIZE);
    serialPortWrite(m_port, buf, count);

    // Copy data into latch buffer prior to write
    if (isUSB)
    {
        serialPortWrite(m_port, data, SAMBA_PAGE_SIZE);
    }
    else
    {
        // send page data
        if (write_uart_modem(data, SAMBA_PAGE_SIZE) != IS_OP_OK)
        {
            return IS_OP_ERROR;
        }
    }

    // EEFC.FCR - WP - copy latch buffers into flash
    count = SNPRINTF((char*)buf, sizeof(buf), "W%08x,5a%04x01#", 0x400e0c04, page);
    serialPortWrite(m_port, buf, count);
    
    return wait_eefc_ready(true);
}

is_operation_result cISBootloaderSAMBA::verify_image(std::string filename)
{
    (void)filename;    // Checksum is used instead of re-reading file

    uint32_t checksum2 = 0;
    uint32_t nextAddress;
    uint8_t buf[SAMBA_PAGE_SIZE] = {0};
    uint8_t cmd[42] = { 0 };
    int count;

    serialPortFlush(m_port);

    SAMBA_STATUS("(SAM-BA) Verifying ISB bootloader (may take some time)...", IS_LOG_LEVEL_INFO);

    while (serialPortRead(m_port, buf, 1));

    for (uint32_t address = SAMBA_FLASH_START_ADDRESS; address < (SAMBA_FLASH_START_ADDRESS + SAMBA_BOOTLOADER_SIZE_24K); )
    {
        int index = 0;
        nextAddress = address + SAMBA_PAGE_SIZE;
        while (address < nextAddress)
        {
            count = SNPRINTF((char*)cmd, sizeof(cmd), "w%08x,#", address);
            serialPortWrite(m_port, (const uint8_t*)"#", 2);
            serialPortWrite(m_port, cmd, count);
            index += serialPortReadTimeout(m_port, buf + index, sizeof(uint32_t), SAMBA_TIMEOUT_DEFAULT);
            address += sizeof(uint32_t);
        }
        /*count = serialPortReadTimeout(m_port, buf, SAMBA_PAGE_SIZE, SAMBA_TIMEOUT_DEFAULT);*/
        if (index == SAMBA_PAGE_SIZE)
        {
            for (uint32_t* ptr = (uint32_t*)buf, *ptrEnd = (uint32_t*)(buf + sizeof(buf)); ptr < ptrEnd; ptr++)
            {
                checksum2 ^= *ptr;
            }
        }
        else return IS_OP_ERROR;

        m_verify_progress = (float)(address - SAMBA_FLASH_START_ADDRESS) / (float)SAMBA_BOOTLOADER_SIZE_24K;
        if (m_verify_callback != 0)
        {
            m_verify_callback(this, m_verify_progress);
        }
    }
    if (checksum != checksum2) return IS_OP_ERROR;
    return IS_OP_OK;
}

uint16_t cISBootloaderSAMBA::crc_update(uint16_t crc_in, int incr)
{
    uint16_t crc = crc_in >> 15;
    uint16_t out = crc_in << 1;

    if (incr) out++;
    if (crc) out ^= UART_XMODEM_CRC_POLY;

    return out;
}


uint16_t cISBootloaderSAMBA::crc16(uint8_t* data, uint16_t size)
{
    uint16_t crc, i;

    for (crc = 0; size > 0; size--, data++)
    {
        for (i = 0x80; i; i >>= 1)
        {
            crc = crc_update(crc, *data & i);
        }
    }

    for (i = 0; i < 16; i++) crc = crc_update(crc, 0);

    return crc;
}

