/**
 * @file ISBootloaderISB.cpp
 * @author Dave Cutting (davidcutting42@gmail.com)
 * @brief Inertial Sense routines for updating application images 
 *  using ISB (Inertial Sense Bootloader) protocol
 *  
 */

/*
MIT LICENSE

Copyright (c) 2014-2023 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include "ISBootloaderISB.h"
#include "ISUtilities.h"

#include <algorithm>

using namespace std;
using namespace ISBootloader;

std::vector<uint32_t> cISBootloaderISB::serial_list;
std::vector<uint32_t> cISBootloaderISB::rst_serial_list;
std::mutex cISBootloaderISB::serial_list_mutex;
std::mutex cISBootloaderISB::rst_serial_list_mutex;

// Delete this and assocated code in Q4 2022 after bootloader v5a is out of circulation. WHJ
#define SUPPORT_BOOTLOADER_V5A

/** uINS bootloader baud rate */
#define IS_BAUD_RATE_BOOTLOADER 921600

#define BOOTLOADER_RETRIES          100
#define BOOTLOADER_RESPONSE_DELAY   10
#define BOOTLOADER_REFRESH_DELAY    500
#define MAX_VERIFY_CHUNK_SIZE       1024
#define BOOTLOADER_TIMEOUT_DEFAULT  1000
#define MAX_SEND_COUNT              510

// logical page size, offsets for pages are 0x0000 to 0xFFFF - flash page size on devices will vary and is not relevant to the bootloader client
#define FLASH_PAGE_SIZE 65536

is_operation_result cISBootloaderISB::match_test(void* param)
{
    const char* serial_name = (const char*)param;

    if(strnlen(serial_name, 100) != 0 && strncmp(serial_name, m_port->port, 100) == 0)
    {
        return IS_OP_OK;
    }

    return IS_OP_ERROR;
}

eImageSignature cISBootloaderISB::check_is_compatible()
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
            status_update(NULL, IS_LOG_LEVEL_ERROR, "    | (ISB Error) (%s) check_is_compatible response missing.", m_port->port);
            return IS_IMAGE_SIGN_NONE;
        }
    }

    uint32_t valid_signatures = 0;
    
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
        status_update(NULL, IS_LOG_LEVEL_ERROR, msg);
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
            valid_signatures |= IS_IMAGE_SIGN_IMX_5;
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
        status_update(NULL, IS_LOG_LEVEL_ERROR, "    | (ISB Error) (%s) check_is_compatible no valid signature.", m_port->port);
    }

    return (eImageSignature)valid_signatures;
}

is_operation_result cISBootloaderISB::reboot_up()
{
    m_info_callback(this, "(ISB) Rebooting to APP mode...", IS_LOG_LEVEL_INFO);

    // send the "reboot to program mode" command and the device should start in program mode
    serialPortWrite(m_port, (unsigned char*)":020000040300F7", 15);
    serialPortFlush(m_port);
    SLEEP_MS(100);
    serialPortClose(m_port);
    return IS_OP_OK;
}

is_operation_result cISBootloaderISB::reboot_down(uint8_t major, char minor, bool force)
{
    char message[100] = {0};
    int n = SNPRINTF(message, 100, "(ISB) Bootloader version: file %c%c, device %c%c. ", major + '0', (minor ? minor : '0'), m_isb_major + '0', m_isb_minor);

    if(!force)
    {   
        if(major == 0 || minor == 0)
        {
            return IS_OP_ERROR;
        }

        if(major < m_isb_major ||
          (major == m_isb_major && minor <= m_isb_minor))
        {
            SNPRINTF(message+n, sizeof(message)-n, "No update.");
            m_info_callback(this, message, IS_LOG_LEVEL_INFO);
            return IS_OP_OK;
        }
    }

    SNPRINTF(message+n, sizeof(message)-n, "Update needed...");
    m_info_callback(this, message, IS_LOG_LEVEL_INFO);
    m_info_callback(this, "(ISB) Rebooting to ROM bootloader mode...", IS_LOG_LEVEL_INFO);

    // USE WITH CAUTION! This will put in bootloader ROM mode allowing a new bootloader to be put on
    // In some cases, the device may become unrecoverable because of interference on its ports.

    // restart bootloader assist command
    serialPortWrite(m_port, (unsigned char*)":020000040700F3", 15);

    serialPortSleep(m_port, 500);

    return IS_OP_OK;
}

is_operation_result cISBootloaderISB::reboot_force()
{
    // restart bootloader command
    if(serialPortWrite(m_port, (unsigned char*)":020000040500F5", 15) != 15)
    {
        status_update("(ISB) Error in reboot force", IS_LOG_LEVEL_ERROR);
        return IS_OP_ERROR;
    }
   
    return IS_OP_OK;
}

is_operation_result cISBootloaderISB::reboot()
{
    rst_serial_list_mutex.lock();
    if(find(rst_serial_list.begin(), rst_serial_list.end(), m_sn) != rst_serial_list.end())
    {
        status_update("(ISB) Could not find serial port", IS_LOG_LEVEL_ERROR);
        rst_serial_list_mutex.unlock();
        return IS_OP_ERROR;
    }

    status_update("(ISB) Resetting before App update...", IS_LOG_LEVEL_INFO);

    // restart bootloader command
    if(reboot_force() == IS_OP_OK)
    {
        rst_serial_list.push_back(m_sn);
        rst_serial_list_mutex.unlock();

        return IS_OP_OK;
    }

    rst_serial_list_mutex.unlock();

    return IS_OP_CLOSED;
}

uint32_t cISBootloaderISB::get_device_info()
{
    sync(m_port);
    serialPortFlush(m_port);

	// Send command
	serialPortWrite(m_port, (uint8_t*)":020000041000EA", 15);

    uint8_t buf[14] = { 0 };

    // Read Version, SAM-BA Available, serial number (in version 6+) and ok (.\r\n) response
	int count = serialPortReadTimeout(m_port, buf, 14, 1000);

    if (count < 8 || buf[0] != 0xAA || buf[1] != 0x55)
    {   // Bad read
        m_isb_major = 0;
        m_isb_minor = 0;
        m_isb_props.rom_available = 1;
        m_isb_props.processor = IS_PROCESSOR_SAMx70;
        m_isb_props.is_evb = false;
        m_sn = 0;

        status_update("(ISB) get_device_info bad read.", IS_LOG_LEVEL_ERROR);
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
        status_update(NULL, IS_LOG_LEVEL_ERROR, "(ISB) (%s) (ISB) get_device_info invalid m_isb_major: %d", m_port->port, m_isb_major);
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

is_operation_result cISBootloaderISB::sync(serial_port_t* s)
{
    static const uint8_t handshakerChar = 'U';

    // Bootloader sync requires at least 6 'U' characters to be sent every 10ms. 
    // write a 'U' to handshake with the boot loader - once we get a 'U' back we are ready to go
    for (int i = 0; i < BOOTLOADER_RETRIES; i++)
    {
        if(serialPortWrite(s, &handshakerChar, 1) != 1)
        {
            return IS_OP_ERROR;
        }

        if (serialPortWaitForTimeout(s, &handshakerChar, 1, BOOTLOADER_RESPONSE_DELAY))
        {	// Success
            return IS_OP_OK;
        }
    }

#if defined(SUPPORT_BOOTLOADER_V5A)     // ONLY NEEDED TO SUPPORT BOOTLOADER v5a.  Delete this and associated code in Q4 2022 after bootloader v5a is out of circulation. WHJ
    static const unsigned char handshaker[] = "INERTIAL_SENSE_SYNC_DFU";

    // Attempt handshake using extended string for bootloader v5a
    for (int i = 0; i < BOOTLOADER_RETRIES; i++)
    {
        if (serialPortWriteAndWaitForTimeout(s, (const unsigned char*)&handshaker, (int)sizeof(handshaker), &handshakerChar, 1, BOOTLOADER_RESPONSE_DELAY))
        {	// Success
            return IS_OP_OK;
        }
    }
#endif

    return IS_OP_ERROR;
}

int cISBootloaderISB::checksum(int checkSum, uint8_t* ptr, int start, int end, int checkSumPosition, int finalCheckSum)
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

is_operation_result cISBootloaderISB::erase_flash()
{
    // give the device 60 seconds to erase flash before giving up
    unsigned char selectFlash[24];

    serial_port_t* s = m_port;

    // Write location to erase at
    memcpy(selectFlash, ":03000006030000F4CC\0\0\0\0\0", 24);
    checksum(0, selectFlash, 1, 17, 17, 1);
    if (serialPortWriteAndWaitForTimeout(s, selectFlash, 19, (unsigned char*)".\r\n", 3, BOOTLOADER_TIMEOUT_DEFAULT) == 0) return IS_OP_ERROR;

    // Erase
    memcpy(selectFlash, ":0200000400FFFBCC\0", 18);
    checksum(0, selectFlash, 1, 15, 15, 1);
    serialPortWrite(s, selectFlash, 17);
    
    // Check for response and allow quit (up to 60 seconds)
    uint8_t buf[128];
    uint8_t *bufPtr = buf;
    int count = 0;
    for(size_t i = 0; i < 600; i++)
    {   
        count += serialPortReadTimeout(s, bufPtr, 3, 100);
        bufPtr = buf + count;

        if (m_update_callback(this, 0.0f) != IS_OP_OK)
        {
            return IS_OP_CANCELLED;
        }
        if (count == 3 && memcmp(buf, ".\r\n", 3) == 0)
        {
            return IS_OP_OK;
        }
    } 

    status_update("(ISB) Error in erase flash", IS_LOG_LEVEL_ERROR);
    return IS_OP_ERROR;
}

is_operation_result cISBootloaderISB::select_page(int page)
{
    serial_port_t* s = m_port;

    // Atmel select page command (0x06) is 4 bytes and the data is always 0301xxxx where xxxx is a 16 bit page number in hex
    unsigned char changePage[24];
    
    // Change page
    SNPRINTF((char*)changePage, 24, ":040000060301%.4XCC", page);
    checksum(0, changePage, 1, 17, 17, 1);
    if (serialPortWriteAndWaitForTimeout(s, changePage, 19, (unsigned char*)".\r\n", 3, BOOTLOADER_TIMEOUT_DEFAULT) == 0) 
    {
        status_update("(ISB) Failed to select page", IS_LOG_LEVEL_ERROR);
        return IS_OP_ERROR;
    }

    return IS_OP_OK;
}

is_operation_result cISBootloaderISB::begin_program_for_current_page(int startOffset, int endOffset)
{
    serial_port_t* s = m_port;

    // Atmel begin program command is 0x01, different from standard intel hex where command 0x01 is end of file
    // After the 0x01 is a 00 which means begin writing program
    // The begin program command uses the current page and specifies two 16 bit addresses that specify where in the current page
    //  the program code will be written
    unsigned char programPage[24];
    
    // Select offset
    SNPRINTF((char*)programPage, 24, ":0500000100%.4X%.4XCC", startOffset, endOffset);
    checksum(0, programPage, 1, 19, 19, 1);
    if (serialPortWriteAndWaitForTimeout(s, programPage, 21, (unsigned char*)".\r\n", 3, BOOTLOADER_TIMEOUT_DEFAULT) == 0)
    {
        status_update("(ISB) Failed to start programming page", IS_LOG_LEVEL_ERROR);
        return IS_OP_ERROR;
    }

    return IS_OP_OK;
}

int cISBootloaderISB::is_isb_read_line(FILE* file, char line[1024])
{
    char c;
    char* currentPtr = line;
    char* endPtr = currentPtr + 1023;

    while (currentPtr != endPtr)
    {
        // read one char
        c = (char)fgetc(file);
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

is_operation_result cISBootloaderISB::upload_hex_page(unsigned char* hexData, int byteCount, int* currentOffset, int* totalBytes, int* verifyCheckSum)
{
    serial_port_t* s = m_port;

    if (byteCount == 0)
    {
        return IS_OP_OK;
    }

    // create a program request with just the hex characters that will fit on this page
    unsigned char programLine[12];
    SNPRINTF((char*)programLine, 12, ":%.2X%.4X00", byteCount, *currentOffset);
    if (serialPortWrite(s, programLine, 9) != 9)
    {
        status_update("(ISB) Failed to write start page", IS_LOG_LEVEL_ERROR);
        return IS_OP_ERROR;
    }

    // add the previously written chars to the checksum
    int checkSum = checksum(0, programLine, 1, 9, 0, 0);

    // write all of the hex chars
    int charsForThisPage = byteCount * 2;
    if (serialPortWrite(s, hexData, charsForThisPage) != charsForThisPage)
    {
        status_update("(ISB) Failed to write data to device", IS_LOG_LEVEL_ERROR);
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
        if (serialPortWrite(s, checkSumHex, 2) != 2)
        {
            status_update("(ISB) Failed to write checksum to device", IS_LOG_LEVEL_ERROR);
            return IS_OP_ERROR;
        }

        unsigned char buf[5] = { 0 };
        int count = serialPortReadTimeout(s, buf, 3, 1000);

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

is_operation_result cISBootloaderISB::upload_hex(unsigned char* hexData, int charCount, int* currentOffset, int* currentPage, int* totalBytes, int* verifyCheckSum)
{
    (void)currentPage;

    if (charCount > MAX_SEND_COUNT)
    {
        status_update("(ISB) Unexpected char count", IS_LOG_LEVEL_ERROR);
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
            status_update("(ISB) Upload hex page error", IS_LOG_LEVEL_ERROR);
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

is_operation_result cISBootloaderISB::fill_current_page(int* currentPage, int* currentOffset, int* totalBytes, int* verifyCheckSum)
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
                status_update("(ISB) Failed to fill page with bytes", IS_LOG_LEVEL_ERROR);
                return IS_OP_ERROR;
            }
        }
    }

    return IS_OP_OK;
}

is_operation_result cISBootloaderISB::download_data(int startOffset, int endOffset)
{
    serial_port_t* s = m_port;

    // Atmel download data command is 0x03, different from standard intel hex where command 0x03 is start segment address
    unsigned char programLine[25];
    int n;
    n = SNPRINTF((char*)programLine, 24, ":0500000300%.4X%.4XCC", startOffset, endOffset);
    programLine[n] = 0;
    checksum(0, programLine, 1, 19, 19, 1);
    if (serialPortWrite(s, programLine, 21) != 21)
    {
        status_update("(ISB) Failed to attempt download", IS_LOG_LEVEL_ERROR);
        return IS_OP_ERROR;
    }

    return IS_OP_OK;
}

is_operation_result cISBootloaderISB::verify_image(std::string filename)
{
    int verifyChunkSize = m_isb_props.verify_size;
    int chunkSize = _MIN(FLASH_PAGE_SIZE, verifyChunkSize);
    int realCheckSum = 5381;
    int totalCharCount = m_isb_props.app_offset * 2;
    int grandTotalCharCount = (m_currentPage + 1) * FLASH_PAGE_SIZE * 2; // char count
    int i, pageOffset, readCount, actualPageOffset, pageChars, chunkIndex, lines;
    int verifyByte = -1;
    unsigned char chunkBuffer[(MAX_VERIFY_CHUNK_SIZE * 2) + 64]; // extra space for overhead
    unsigned char c=0;
    FILE* verifyFile = 0;

    m_verify_progress = 0.0f;

#ifdef _MSC_VER
    fopen_s(&verifyFile, filename.c_str(), "wb");
#else
    verifyFile = fopen(filename.c_str(), "wb");
#endif

    for (i = 0; i <= m_currentPage; i++)
    {
        if (select_page(i) != IS_OP_OK)
        {
            status_update("(ISB) Failure issuing select page command for verify", IS_LOG_LEVEL_ERROR);
            return IS_OP_ERROR;
        }
        pageOffset = (i == 0 ? m_isb_props.app_offset : 0);
        while (pageOffset < FLASH_PAGE_SIZE)
        {
            readCount = _MIN(chunkSize, FLASH_PAGE_SIZE - pageOffset);

            // range is inclusive on the uINS, so subtract one
            if (download_data(pageOffset, pageOffset + readCount - 1) != IS_OP_OK)
            {
                status_update("(ISB) Failure issuing download data command", IS_LOG_LEVEL_ERROR);
                return IS_OP_ERROR;
            }

            // each line has 7 overhead bytes, plus two bytes (hex) for each byte on the page and max 255 bytes per line
            lines = (int)ceilf((float)readCount / 255.0f);
            readCount = serialPortReadTimeout(m_port, chunkBuffer, (7 * lines) + (readCount * 2), BOOTLOADER_TIMEOUT_DEFAULT);
            chunkIndex = 0;

            while (chunkIndex < readCount)
            {
                if (chunkIndex > readCount - 5)
                {
                    status_update("(ISB) Unexpected start line during verify (1)", IS_LOG_LEVEL_ERROR);
                    return IS_OP_ERROR;
                }
                // skip the first 5 chars, they are simply ####=
                if (chunkBuffer[chunkIndex] == 'X')
                {
                    status_update("(ISB) Invalid checksum during verify", IS_LOG_LEVEL_ERROR);
                    return IS_OP_ERROR;
                }
                else if (chunkBuffer[chunkIndex += 4] != '=')
                {
                    status_update("(ISB) Unexpected start line during verify (2)", IS_LOG_LEVEL_ERROR);
                    return IS_OP_ERROR;
                }
                chunkBuffer[chunkIndex] = '\0';
                actualPageOffset = strtol((char*)(chunkBuffer + chunkIndex - 4), 0, 16);
                if (actualPageOffset != pageOffset)
                {
                    status_update("(ISB) Unexpected offset during verify", IS_LOG_LEVEL_ERROR);
                    return IS_OP_ERROR;
                }
                pageChars = 0;
                chunkIndex++;
                while (chunkIndex < readCount)
                {
                    c = chunkBuffer[chunkIndex++];
                    if ((c >= '0' && c <= '9') || (c >= 'A' && c <= 'F'))
                    {
                        pageChars++;
                        totalCharCount++;
                        realCheckSum = ((realCheckSum << 5) + realCheckSum) + c;

                        if (verifyFile != 0)
                        {
                            if (verifyByte == -1)
                            {
                                verifyByte = ((c >= '0' && c <= '9') ? c - '0' : c - 'A' + 10) << 4;
                            }
                            else
                            {
                                verifyByte |= (c >= '0' && c <= '9') ? c - '0' : c - 'A' + 10;
                                fputc(verifyByte, verifyFile);
                                verifyByte = -1;
                            }
                        }
                    }
                    else if (c == '\r')
                    {
                        continue;
                    }
                    else if (c == '\n')
                    {
                        break;
                    }
                    else
                    {
                        status_update("(ISB) Unexpected hex data during verify", IS_LOG_LEVEL_ERROR);
                        return IS_OP_ERROR;
                    }
                }

                if (c != '\n')
                {
                    status_update("(ISB) Unexpected end of line char during verify", IS_LOG_LEVEL_ERROR);
                    return IS_OP_ERROR;
                }

                // increment page offset
                pageOffset += (pageChars / 2);

                if (m_verify_callback != 0)
                {
                    m_verify_progress = (float)totalCharCount / (float)grandTotalCharCount;
                    if (m_verify_callback(this, m_verify_progress) != IS_OP_OK)
                    {
                        status_update("(ISB) Firmware validate cancelled", IS_LOG_LEVEL_ERROR);
                        return IS_OP_CANCELLED;
                    }
                }
            }
        }
    }

    if (verifyFile != 0)
    {
        fclose(verifyFile);
    }

    if (realCheckSum != m_verifyCheckSum)
    {
        status_update("(ISB) Checksum mismatch during verify", IS_LOG_LEVEL_ERROR);
        return IS_OP_ERROR;
    }

    return IS_OP_OK;
}

#define HEX_BUFFER_SIZE 1024

is_operation_result cISBootloaderISB::process_hex_file(FILE* file)
{
    int currentPage = -1;
    int currentOffset = m_isb_props.app_offset;
    int lastSubOffset = currentOffset;
    int subOffset;
    int totalBytes = m_isb_props.app_offset;

    int verifyCheckSum = 5381;
    int lineLength;
    m_update_progress = 0.0f;
    char line[HEX_BUFFER_SIZE];
    unsigned char output[HEX_BUFFER_SIZE * 2]; // big enough to store an entire extra line of buffer if needed
    unsigned char* outputPtr = output;
    const unsigned char* outputPtrEnd = output + (HEX_BUFFER_SIZE * 2);
    int outputSize;
    //int page = 0;
    int pad;
    int fileSize;
    unsigned char tmp[5];
    int i;

    fseek(file, 0, SEEK_END);
    fileSize = ftell(file);
    fseek(file, 0, SEEK_SET);

    while ((lineLength = is_isb_read_line(file, line)) != 0)
    {
        if (lineLength > 12 && line[7] == '0' && line[8] == '0')
        {
            if (lineLength > HEX_BUFFER_SIZE * 4)
            {
                status_update("(ISB) hex file line length too long", IS_LOG_LEVEL_ERROR);
                return IS_OP_ERROR;
            }

            // we need to know the offset that this line was supposed to be stored at so we can check if offsets are skipped
            memcpy(tmp, line + 3, 4);
            tmp[4] = '\0';
            subOffset = strtol((char*)tmp, 0, 16);

            // check if we skipped an offset, the intel hex file format can do this, in which case we need to make sure
            // that the bytes that were skipped get set to something
            if (subOffset > lastSubOffset)
            {
                // pad with FF bytes, this is an internal implementation detail to how the device stores unused memory
                pad = (subOffset - lastSubOffset);
                if (outputPtr + pad >= outputPtrEnd)
                {
                    status_update("(ISB) FF padding overflowed buffer", IS_LOG_LEVEL_ERROR);
                    return IS_OP_ERROR;
                }

                while (pad-- != 0)
                {
                    *outputPtr++ = 'F';
                    *outputPtr++ = 'F';
                }
            }

            // skip the first 9 chars which are not data, then take everything else minus the last two chars which are a checksum
            // check for overflow
            pad = lineLength - 11;
            if (outputPtr + pad >= outputPtrEnd)
            {
                status_update("(ISB) Line data overflowed output buffer", IS_LOG_LEVEL_ERROR);
                return IS_OP_ERROR;
            }

            for (i = 9; i < lineLength - 2; i++)
            {
                *outputPtr++ = line[i];
            }

            // set the end offset so we can check later for skipped offsets
            lastSubOffset = subOffset + ((lineLength - 11) / 2);
            outputSize = (int)(outputPtr - output);

            // we try to send the most allowed by this hex file format
            if (outputSize < MAX_SEND_COUNT)
            {
                // keep buffering
                continue;
            }
            // upload this chunk
            if (upload_hex(output, _MIN(MAX_SEND_COUNT, outputSize), &currentOffset, &currentPage, &totalBytes, &verifyCheckSum) != IS_OP_OK)
            {
                status_update("(ISB) Error in upload chunk", IS_LOG_LEVEL_ERROR);
                return IS_OP_ERROR;
            }

            outputSize -= MAX_SEND_COUNT;

            if (outputSize < 0 || outputSize > HEX_BUFFER_SIZE)
            {
                status_update("(ISB) Output size was too large (1)", IS_LOG_LEVEL_ERROR);
                return IS_OP_ERROR;
            }
            if (outputSize > 0)
            {
                // move the left-over data to the beginning
                memmove(output, output + MAX_SEND_COUNT, outputSize);
            }

            // reset output ptr back to the next chunk of data
            outputPtr = output + outputSize;
        }
        else if (strncmp(line, ":020000040", 10) == 0 && strlen(line) >= 13)
        {
            memcpy(tmp, line + 12, 3);      // Only support up to 10 pages currently
            tmp[1] = '\0';
            currentPage = strtol((char*)tmp, 0, 16);

            if(currentPage == 0) 
            {
                lastSubOffset = currentOffset;
                continue;
            }
            else 
            {
                lastSubOffset = 0;
            }
            
            outputSize = (int)(outputPtr - output);

            if (outputSize < 0 || outputSize > HEX_BUFFER_SIZE)
            {
                status_update("(ISB) Output size was too large (2)", IS_LOG_LEVEL_ERROR);
                return IS_OP_ERROR;
            }
            // flush the remainder of data to the page
            if (upload_hex(output, outputSize, &currentOffset, &currentPage, &totalBytes, &verifyCheckSum) != IS_OP_OK)
            {
                status_update("(ISB) Error in upload hex", IS_LOG_LEVEL_ERROR);
                return IS_OP_ERROR;
            }
            // // fill the remainder of the current page, the next time that bytes try to be written the page will be automatically incremented
            if (fill_current_page(&currentPage, &currentOffset, &totalBytes, &verifyCheckSum) != IS_OP_OK)
            {
                status_update("(ISB) Error in fill page", IS_LOG_LEVEL_ERROR);
                return IS_OP_ERROR;
            }

            // change to the next page
            currentOffset = 0;
            if (select_page(currentPage) != IS_OP_OK || begin_program_for_current_page(0, FLASH_PAGE_SIZE - 1) != IS_OP_OK)
            {
                status_update("(ISB) Failed to issue select page or to start programming", IS_LOG_LEVEL_ERROR);
                return IS_OP_ERROR;
            }

            // set the output ptr back to the beginning, no more data is in the queue
            outputPtr = output;
        }
        else if (lineLength > 10 && line[7] == '0' && line[8] == '1')
        {   // End of last page (end of file marker)
            outputSize = (int)(outputPtr - output);

            // flush the remainder of data to the page
            if (upload_hex(output, outputSize, &currentOffset, &currentPage, &totalBytes, &verifyCheckSum) != IS_OP_OK)
            {
                status_update("(ISB) Error in upload hex (last)", IS_LOG_LEVEL_ERROR);
                return IS_OP_ERROR;
            }
            if (currentOffset != 0 && fill_current_page(&currentPage, &currentOffset, &totalBytes, &verifyCheckSum) != IS_OP_OK)
            {
                status_update("(ISB) Error in fill page (last)", IS_LOG_LEVEL_ERROR);
                return IS_OP_ERROR;
            }

            outputPtr = output;            
        }

        if (m_update_callback != 0)
        {
            m_update_progress = (float)ftell(file) / (float)fileSize;	// Dummy line to call ftell() once
            m_update_progress = (float)ftell(file) / (float)fileSize;

            // Try catch added m_update_callback being correupted
            try
            {
                if (m_update_callback(this, m_update_progress) != IS_OP_OK)
                {
                    status_update("(ISB) Firmware update cancelled", IS_LOG_LEVEL_ERROR);
                    return IS_OP_CANCELLED;
                }
            }
            catch(int e)
            {
                string tmp = "(ISB) Firmware update cancelled. Error number: " + to_string(e);
                status_update(tmp.c_str(), IS_LOG_LEVEL_ERROR);
                return IS_OP_CANCELLED;
            }
        }
    }

    if (m_update_callback != 0 && m_update_progress != 1.0f)
    {
        m_update_progress = 1.0f;
        m_update_callback(this, m_update_progress);
    }

    // Set the verify function up
    m_currentPage = currentPage;
    m_verifyCheckSum = verifyCheckSum;

    return IS_OP_OK;
}

is_operation_result cISBootloaderISB::download_image(std::string filename)
{
    FILE* firmware_file = 0;
    is_operation_result result;

#ifdef _MSC_VER
    fopen_s(&firmware_file, filename.c_str(), "rb");
#else
    firmware_file = fopen(filename.c_str(), "rb");
#endif

    if (!firmware_file)
    {
        status_update("(ISB) Error in opening file", IS_LOG_LEVEL_ERROR);
        return IS_OP_INCOMPATIBLE;
    }

    status_update("(ISB) Erasing flash...", IS_LOG_LEVEL_INFO);

    result = erase_flash();
    if(result != IS_OP_OK) { fclose(firmware_file); return result; }
    result = select_page(0);
    if(result != IS_OP_OK) { fclose(firmware_file); return result; }

    status_update("(ISB) Programming flash...", IS_LOG_LEVEL_INFO);
    
    result = begin_program_for_current_page(m_isb_props.app_offset, FLASH_PAGE_SIZE - 1);
    if(result != IS_OP_OK) { fclose(firmware_file); return result; }
    result = process_hex_file(firmware_file);
    if(result != IS_OP_OK) { fclose(firmware_file); return result; }

    fclose(firmware_file);

    SLEEP_MS(1000); // Allow some time for commands to be sent in UART mode

    return IS_OP_OK;
}

is_operation_result cISBootloaderISB::get_version_from_file(const char* filename, uint8_t* major, char* minor)
{
    FILE* blfile = 0;

#ifdef _MSC_VER
    fopen_s(&blfile, filename, "rb");
#else
    blfile = fopen(filename, "rb");
#endif

    if (blfile == 0)
    {
        return IS_OP_ERROR;
    }

    fseek(blfile, 0x3DFC, SEEK_SET);
    unsigned char ver_info[4];
	size_t n = fread(ver_info, 1, 4, blfile);
	(void)n;
    fclose(blfile);

    //Check for marker for valid version info
    if (ver_info[0] == 0xAA && ver_info[1] == 0x55)
    {
        if (major)
            *major = ver_info[2];
        if (minor)
            *minor = ver_info[3];
        return IS_OP_OK;
    }

    //No version found
    return IS_OP_ERROR;
}





