/**
 * @file ISBootloaderSAMBA.c
 * @author Dave Cutting (davidcutting42@gmail.com)
 * @brief Inertial Sense routines for updating ISB (Inertial Sense Bootloader)
 *  images using the SAM-BA protocol.
 * 
 */

/*
MIT LICENSE

Copyright (c) 2014-2022 Inertial Sense, Inc. - http://inertialsense.com

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

// Resources for SAM-BA protocol:
//  - Datasheet ROM boot section
//  - https://github.com/atmelcorp/sam-ba/tree/master/src/plugins/connection/serial
//  - https://sourceforge.net/p/lejos/wiki-nxt/SAM-BA%20Protocol/

#define UART_XMODEM_SOH 0x01
#define UART_XMODEM_EOT 0x04
#define UART_XMODEM_ACK 0x06
#define UART_XMODEM_NAK 0x15
#define UART_XMODEM_CAN 0x18
#define UART_XMODEM_CRC_POLY 0x1021
#define UART_XMODEM_PAYLOAD_SIZE 128

#define SAMBA_PAGE_SIZE 512
#define SAMBA_BAUDRATE 115200
#define SAMBA_TIMEOUT_DEFAULT 1000
#define SAMBA_FLASH_START_ADDRESS 0x00400000
#define SAMBA_BOOTLOADER_SIZE 16384
#define SAMBA_BOOTLOADER_SIZE_24K 0x6000

#define SAMBA_STATUS(x, level) ctx->info_callback(ctx, x, level)
#define SAMBA_ERROR_CHECK(x, error) if(x != IS_OP_OK) \
    { \
        serialPortClose(port); \
        SAMBA_STATUS(error, IS_LOG_LEVEL_ERROR); \
        return IS_OP_ERROR; \
    }

static is_operation_result is_samba_read_word(is_device_context* ctx, uint32_t address, uint32_t* word);
static is_operation_result is_samba_write_word(is_device_context* ctx, uint32_t address, uint32_t word);
static is_operation_result is_samba_wait_eefc_ready(is_device_context* ctx, bool waitReady);
static is_operation_result is_samba_boot_from_flash(is_device_context* ctx);
static is_operation_result is_samba_write_uart_modem(is_device_context* ctx, uint8_t* buf, size_t len);
static is_operation_result is_samba_flash_erase_write_page(is_device_context* ctx, size_t offset, uint8_t data[SAMBA_PAGE_SIZE], bool isUSB);
static is_operation_result is_samba_verify(is_device_context* ctx, uint32_t checksum);
static is_operation_result is_samba_reset(is_device_context* ctx);

static uint16_t crc_update(uint16_t crc_in, int incr);
static uint16_t crc16(uint8_t* data, uint16_t size);

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

/**
 * @brief Flash the bootloader image onto the chip 
 * 
 * @param ctx a filled device context with bootloader firmware image
 * @return is_operation_result 
 */
is_operation_result is_samba_flash(is_device_context* ctx)
{
    serial_port_t* port = &ctx->handle.port;

    is_samba_reset(ctx);

    SAMBA_ERROR_CHECK(is_samba_init(ctx), "Failed to re-init SAM-BA device");

    // https://github.com/atmelcorp/sam-ba/tree/master/src/plugins/connection/serial
    // https://sourceforge.net/p/lejos/wiki-nxt/SAM-BA%20Protocol/
    uint8_t buf[SAMBA_PAGE_SIZE];
    uint32_t checksum = 0;

    // try non-USB and then USB mode (0 and 1)
    for(int isUSB = 0; isUSB < 2; isUSB++)
    {
        serialPortSleep(port, 250);
        serialPortClose(port);
        if(!serialPortOpenRetry(port, port->port, SAMBA_BAUDRATE, 1))
        {
            serialPortClose(port);
            return IS_OP_ERROR;
        }

        // flush
        serialPortWrite(port, (const uint8_t*)"#", 2);
        serialPortReadTimeout(port, buf, sizeof(buf), 100);

        FILE* file;
#ifdef _MSC_VER
        fopen_s(&file, ctx->firmware_path, "rb");
#else
        file = fopen(ctx->firmware_path, "rb");
#endif

        if (file == 0)
        {
            SAMBA_STATUS("Unable to load bootloader file", IS_LOG_LEVEL_ERROR);
            serialPortClose(port);
            return 0;
        }

        fseek(file, 0, SEEK_END);
        int size = ftell(file);
        fseek(file, 0, SEEK_SET);
        checksum = 0;

        if (size != SAMBA_BOOTLOADER_SIZE_24K)
        {
            SAMBA_STATUS("Invalid or old (v5 or earlier) bootloader file", IS_LOG_LEVEL_ERROR);
            serialPortClose(port);
            return 0;
        }

        SAMBA_STATUS("Writing bootloader...", IS_LOG_LEVEL_INFO);

        uint32_t offset = 0;
        while (fread(buf, 1, SAMBA_PAGE_SIZE, file) == SAMBA_PAGE_SIZE)
        {
            if (is_samba_flash_erase_write_page(ctx, offset, buf, isUSB) != IS_OP_OK)
            {
                if (!isUSB) { offset = 0; break; } // try USB mode
                SAMBA_STATUS("Failed to upload page", IS_LOG_LEVEL_ERROR);
                serialPortClose(port);
                return 0;
            }
            for (uint32_t* ptr = (uint32_t*)buf, *ptrEnd = (uint32_t*)(buf + sizeof(buf)); ptr < ptrEnd; ptr++)
            {
                checksum ^= *ptr;
            }
            offset += SAMBA_PAGE_SIZE;
            
            ctx->update_progress = (float)offset / (float)SAMBA_BOOTLOADER_SIZE_24K;
            if (ctx->update_callback != 0)
            {
                ctx->update_callback(ctx, ctx->update_progress);
            }
        }
        fclose(file);
        if (offset != 0) break; // success!
    }

    if (ctx->verify_callback != 0)
    {
        SAMBA_STATUS("Verifying bootloader...", IS_LOG_LEVEL_INFO);
        SAMBA_ERROR_CHECK(is_samba_verify(ctx, checksum), "Verification error!");
    }
    
    SAMBA_ERROR_CHECK(is_samba_boot_from_flash(ctx), "Failed to set boot from flash GPNVM bit!");
    is_samba_reset(ctx);

    serialPortClose(port);

    SLEEP_MS(1000);

    return IS_OP_OK;
}

/**
 * @brief Initialize the registers of the SAM-BA device and retrieves the serial number from FLASH user pages
 * 
 * @param ctx a filled device context
 * @return is_operation_result 
 */
is_operation_result is_samba_init(is_device_context* ctx)
{
    serial_port_t* port = &ctx->handle.port;
    uint8_t buf[SAMBA_PAGE_SIZE];

    serialPortSleep(port, 250);
    serialPortClose(port);
    if (!serialPortOpenRetry(port, port->port, SAMBA_BAUDRATE, 1))
    {
        serialPortClose(port);
        return IS_OP_ERROR;
    }

    // Flush the bootloader command buffer
    serialPortWrite(port, (const uint8_t*)"#", 2);
    int count = serialPortReadTimeout(port, buf, sizeof(buf), 100);

    // Set non-interactive mode and wait for response
    count = serialPortWriteAndWaitFor(port, (const uint8_t*)"N#", 2, (const uint8_t*)"\n\r", 2);
    if (!count)
    {   // Failed to handshake with bootloader
        serialPortClose(port);
        return IS_OP_ERROR;
    }

    // Set flash mode register
    SAMBA_ERROR_CHECK(is_samba_write_word(ctx, 0x400e0c00, 0x04000600), "Failed to set flash mode register");

    SAMBA_STATUS("SAM-BA ROM bootloader initialized", IS_LOG_LEVEL_INFO);

    serialPortClose(port);

    return IS_OP_OK;
}

/**
 * @brief Fills the 32-bit serial number into `ctx->props.serial`
 * 
 * @param ctx device context with open serial port connected to initialized SAM-BA device
 * @return is_operation_result 
 */
is_operation_result is_samba_get_serial(is_device_context* ctx)
{
    serial_port_t* port = &ctx->handle.port;

    // Set flash command to STUS (start read unique signature)
    SAMBA_ERROR_CHECK(is_samba_write_word(ctx, 0x400e0c04, 0x5a000014), "Failed to command signature readout");
    
    // Wait until EEFC.FSR.FRDY is cleared
    SAMBA_ERROR_CHECK(is_samba_wait_eefc_ready(ctx, false), "Failed to clear flash ready bit");

    // Read out the unique identifier
    SAMBA_ERROR_CHECK(is_samba_read_word(ctx, 0x00400000 + offsetof(manufacturing_info_t, serialNumber), &ctx->props.serial), "Failed to read UID word");
    
    // Set flash command to SPUS (stop read unique identifier)
    SAMBA_ERROR_CHECK(is_samba_write_word(ctx, 0x400e0c04, 0x5a000015), "Failed to command stop signature readout");
    
    return IS_OP_OK;
}

/**
 * @brief Write a single (32-bit) word to the device. Can read from any peripheral or memory
 * 
 * @param ctx device context with open serial port registered under `handler`
 * @param address Address to read at
 * @param word Filled with the read value
 * @return is_operation_result 
 */
static is_operation_result is_samba_read_word(is_device_context* ctx, uint32_t address, uint32_t* word)
{
    uint8_t buf[16];
    int count = SNPRINTF((char*)buf, sizeof(buf), "w%08x,#", address);
    if ((serialPortWrite(&ctx->handle.port, buf, count) == count) &&
        (serialPortReadTimeout(&ctx->handle.port, buf, sizeof(uint32_t), SAMBA_TIMEOUT_DEFAULT) == sizeof(uint32_t)))
    {
        *word = (buf[3] << 24) | (buf[2] << 16) | (buf[1] << 8) | buf[0];
        return IS_OP_OK;
    }
    return IS_OP_ERROR;
}

/**
 * @brief Write a single (32-bit) word to the device. Can write to any peripheral or memory
 * 
 * @param ctx device context with open serial port registered under `handler`
 * @param address Address to write at
 * @param word Value to write
 * @return is_operation_result 
 */
static is_operation_result is_samba_write_word(is_device_context* ctx, uint32_t address, uint32_t word)
{
    unsigned char buf[32];
    int count = SNPRINTF((char*)buf, sizeof(buf), "W%08x,%08x#", address, word);
    if(serialPortWrite(&ctx->handle.port, buf, count) != count) return IS_OP_ERROR;
    return IS_OP_OK;
}

/**
 * @brief Wait for the embedded flash controller to be ready or not ready
 * 
 * @param ctx device context with open serial port registered under `handler`
 * @param waitReady if `true`, wait until controller is ready. `false`, wait until not ready
 * @return is_operation_result 
 */
static is_operation_result is_samba_wait_eefc_ready(is_device_context* ctx, bool waitReady)
{
    uint32_t status = 0;
    for (int i = 0; i < 10; i++)
    {
        // EEFC.FSR.FRDY - Flash ready status
        if(is_samba_read_word(ctx, 0x400e0c08, &status) == IS_OP_ERROR) continue; 
        if( waitReady &&  (status & 0x01)) return IS_OP_OK;   // status is ready and we are waiting for ready
        if(!waitReady && !(status & 0x01)) return IS_OP_OK;   // status is not ready and we are waiting for not ready
        serialPortSleep(&ctx->handle.port, 20);
    }
    return IS_OP_ERROR;
}

/**
 * @brief Boot from flash memory
 * 
 * @param ctx device context with open serial port registered under `handler`
 * @return is_operation_result 
 */
static is_operation_result is_samba_boot_from_flash(is_device_context* ctx)
{
    // EEFC.FCR, EEFC_FCR_FKEY_PASSWD | EEFC_FCR_FARG_BOOT | EEFC_FCR_FCMD_SGPB
    if (is_samba_write_word(ctx, 0x400e0c04, 0x5a00010b) == IS_OP_OK)
    {
        return is_samba_wait_eefc_ready(ctx, true);
    }
    return IS_OP_ERROR;
}

/**
 * @brief Write a buffer to the device if connected via UART, minding rules (not 
 *  sure where the rules are documented) of the UART connection
 * 
 * @param ctx device context with open serial port registered under `handler`
 * @param buf buffer to send to device
 * @param len length of buffer
 * @return is_operation_result 
 */
static is_operation_result is_samba_write_uart_modem(is_device_context* ctx, uint8_t* buf, size_t len)
{
    int ret;
    uint8_t eot = UART_XMODEM_EOT;   // "X" comes from xModem, not sure where that came from
    uint8_t answer;
    xmodem_chunk_t chunk = { .block = 1, .start = UART_XMODEM_SOH };

    // wait for ping from bootloader
    do
    {
        if (serialPortRead(&ctx->handle.port, &answer, sizeof(uint8_t)) != sizeof(uint8_t))
        {
            return IS_OP_ERROR;
        }
    } while (answer != 'C');

    // write up to one sector
    while (len)
    {
        size_t z = 0;
        
        z = _MIN(len, sizeof(chunk.payload));
        memcpy(chunk.payload, buf, z);
        memset(chunk.payload + z, 0xff, sizeof(chunk.payload) - z);

        chunk.crc = SWAP16(crc16(chunk.payload, sizeof(chunk.payload)));
        chunk.block_neg = 0xff - chunk.block;

        ret = serialPortWrite(&ctx->handle.port, (const uint8_t*)&chunk, sizeof(xmodem_chunk_t));
        if (ret != sizeof(xmodem_chunk_t))
        {
            return IS_OP_ERROR;
        }

        ret = serialPortReadTimeout(&ctx->handle.port, &answer, sizeof(uint8_t), SAMBA_TIMEOUT_DEFAULT);
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

    ret = serialPortWrite(&ctx->handle.port, &eot, sizeof(uint8_t));
    if (ret != sizeof(uint8_t))
    {
        return -1;
    }
    // serialPortReadChar(&ctx->handle.port, &eot);
    return IS_OP_OK;
}

/**
 * @brief Erase, then write a page of flash on the device
 * 
 * @param ctx device context with open serial port registered under `handler`
 * @param offset offset from the base address of the flash memory to write at
 * @param data buffer to write. Must be at least `SAMBA_PAGE_SIZE` long
 * @param isUSB if the device is connected over UART, different rules apply for transmission
 * @return is_operation_result 
 */
static is_operation_result is_samba_flash_erase_write_page(is_device_context* ctx, size_t offset, uint8_t data[SAMBA_PAGE_SIZE], bool isUSB)
{
    uint16_t page = (uint16_t)(offset / SAMBA_PAGE_SIZE);
    uint8_t buf[32];
    int count;

    // Copy data into latch buffer prior to write
    if (isUSB)
    {
        count = SNPRINTF((char*)buf, sizeof(buf), "S%08x,%08x#",
                         (unsigned int)(SAMBA_FLASH_START_ADDRESS + offset),
                         (unsigned int)SAMBA_PAGE_SIZE);
        serialPortWrite(&ctx->handle.port, buf, count);
        serialPortWrite(&ctx->handle.port, data, SAMBA_PAGE_SIZE);
    }
    else
    {
        count = SNPRINTF((char*)buf, sizeof(buf), "S%08x,#", (unsigned int)(SAMBA_FLASH_START_ADDRESS + offset));
        serialPortWrite(&ctx->handle.port, buf, count);

        // send page data
        if (is_samba_write_uart_modem(ctx, buf, SAMBA_PAGE_SIZE) != IS_OP_OK)
        {
            return IS_OP_ERROR;
        }
    }

    // EEFC.FCR - EWP - erase page, then copy latch buffers into flash
    count = SNPRINTF((char*)buf, sizeof(buf), "W%08x,5a%04x03#", 0x400e0c04, page);
    serialPortWrite(&ctx->handle.port, buf, count);
    
    return is_samba_wait_eefc_ready(ctx, true);
}

/**
 * @brief Verify the bootloader image
 * 
 * @param ctx device context with open serial port registered under `handler`
 * @param checksum the target checksum
 * @return is_operation_result 
 */
static is_operation_result is_samba_verify(is_device_context* ctx, uint32_t checksum)
{
    uint32_t checksum2 = 0;
    uint32_t nextAddress;
    unsigned char buf[512];
    int count;

    for (uint32_t address = SAMBA_FLASH_START_ADDRESS; address < (SAMBA_FLASH_START_ADDRESS + SAMBA_BOOTLOADER_SIZE_24K); )
    {
        nextAddress = address + SAMBA_PAGE_SIZE;
        serialPortFlush(&ctx->handle.port);
        while (address < nextAddress)
        {
            count = SNPRINTF((char*)buf, sizeof(buf), "w%08x,#", address);
            serialPortWrite(&ctx->handle.port, buf, count);
            address += sizeof(uint32_t);
            serialPortSleep(&ctx->handle.port, 2); // give device time to process command
        }
        count = serialPortReadTimeout(&ctx->handle.port, buf, SAMBA_PAGE_SIZE, SAMBA_TIMEOUT_DEFAULT);
        if (count == SAMBA_PAGE_SIZE)
        {
            for (uint32_t* ptr = (uint32_t*)buf, *ptrEnd = (uint32_t*)(buf + sizeof(buf)); ptr < ptrEnd; ptr++)
            {
                checksum2 ^= *ptr;
            }
        }
        else return IS_OP_ERROR;

        ctx->verify_progress = (float)(address - SAMBA_PAGE_SIZE) / (float)SAMBA_BOOTLOADER_SIZE_24K;
        if (ctx->verify_callback != 0)
        {
            ctx->verify_callback(ctx, ctx->verify_progress);
        }
    }
    if (checksum != ~checksum2) return IS_OP_ERROR;
    return IS_OP_OK;
}

/**
 * @brief Software reset the chip
 * 
 * @param ctx device context with open serial port registered under `handler`
 * @return is_operation_result 
 */
static is_operation_result is_samba_reset(is_device_context* ctx)
{
    // RSTC_CR, RSTC_CR_KEY_PASSWD | RSTC_CR_PROCRST
    uint32_t status;
    if (is_samba_write_word(ctx, 0x400e1800, 0xa5000001) != IS_OP_OK)
    {
        return IS_OP_ERROR;
    }
    for (int i = 0; i < 100; i++)
    {
        if(is_samba_read_word(ctx, 0x400e1804, &status) != IS_OP_OK) return IS_OP_ERROR; // RSTC_SR
        if (!(status & 0x00020000)) // RSTC_SR_SRCMP
        {
            return IS_OP_OK;
        }
        serialPortSleep(&ctx->handle.port, 20);
    }
    return IS_OP_ERROR;
}

/**
 * @brief CRC16 helper function
 * 
 * @param crc_in 
 * @param incr 
 * @return uint16_t 
 */
static uint16_t crc_update(uint16_t crc_in, int incr)
{
    uint16_t crc = crc_in >> 15;
    uint16_t out = crc_in << 1;

    if (incr) out++;
    if (crc) out ^= UART_XMODEM_CRC_POLY;

    return out;
}

/**
 * @brief CRC16 generator
 * 
 * @param data buffer to compute crc16 on
 * @param size length of buffer
 * @return uint16_t crc16 value
 */
static uint16_t crc16(uint8_t* data, uint16_t size)
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

