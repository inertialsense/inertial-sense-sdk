/**
 * @file ISBootloaderISB.c
 * @author Dave Cutting (davidcutting42@gmail.com)
 * @brief Inertial Sense routines for updating application images 
 *  using ISB (Inertial Sense Bootloader) protocol
 *  
 */

/*
MIT LICENSE

Copyright (c) 2014-2022 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include "ISBootloader.h"
#include "ISBootloaderISB.h"
#include "ISUtilities.h"

// Delete this and assocated code in Q4 2022 after bootloader v5a is out of circulation. WHJ
#define SUPPORT_BOOTLOADER_V5A

/** uINS bootloader baud rate */
#define IS_BAUD_RATE_BOOTLOADER 921600

/** uINS rs232 bootloader baud rate */
#define IS_BAUD_RATE_BOOTLOADER_RS232 230400

/** uINS slow bootloader baud rate */
#define IS_BAUD_RATE_BOOTLOADER_SLOW 115200

/** uINS bootloader baud rate - legacy */
#define IS_BAUD_RATE_BOOTLOADER_LEGACY 2000000

#define BOOTLOADER_RETRIES          30
#define BOOTLOADER_RESPONSE_DELAY   10
#define BOOTLOADER_REFRESH_DELAY    500
#define MAX_VERIFY_CHUNK_SIZE       1024
#define BOOTLOADER_TIMEOUT_DEFAULT  1000
#define MAX_SEND_COUNT              510

// logical page size, offsets for pages are 0x0000 to 0xFFFF - flash page size on devices will vary and is not relevant to the bootloader client
#define FLASH_PAGE_SIZE 65536

/**
 * @brief Negotiate the bootloader version, once a 'U' character has been read, 
 *  we read another character, timing out after 500 milliseconds if nothing 
 *  comes back, we are using version 1, otherwise the version is the number sent 
 *  back
 * 
 * @param ctx 
 * @return is_operation_result 
 */
is_operation_result is_isb_negotiate_version(is_device_context* ctx)
{
    unsigned char v = 0;

    do {
        if (serialPortReadCharTimeout(&ctx->handle.port, &v, 500) == 0) v = '1';
    } while (v == 'U');

    if (v == '1')
    {   // version 1
        ctx->props.isb.major = 1;
        ctx->props.isb.app_offset = 8192;
    }
    else if (v >= '2' && v <= '5')
    {   // version 2, 3 (which sent v2), 4, 5
        ctx->props.isb.major = v-'0';
        ctx->props.isb.app_offset = 16384;
    }
    else if (v == '6')
    {   // version 6
        ctx->props.isb.major = v-'6';
        ctx->props.isb.app_offset = 24576;
    }
    else
    {
        return IS_OP_ERROR;
    }

#if PLATFORM_IS_WINDOWS
    // EvalTool and multiple bootloads under Windows 10 have issues with dropped data if verify runs too fast
    ctx->props.isb.verify_size = 125;
#else
    ctx->props.isb.verify_size = MAX_VERIFY_CHUNK_SIZE;
#endif

    return IS_OP_OK;
}

void is_isb_restart_rom(serial_port_t* s)
{
    // USE WITH CAUTION! This will put in bootloader ROM mode allowing a new bootloader to be put on
    // In some cases, the device may become unrecoverable because of interferece on its ports.

    // restart bootloader assist command
    serialPortWrite(s, (unsigned char*)":020000040700F3", 15);
}

void is_isb_restart(serial_port_t* s)
{
    // restart bootloader command
    serialPortWrite(s, (unsigned char*)":020000040500F5", 15);
    serialPortClose(s);
    serialPortSleep(s, BOOTLOADER_REFRESH_DELAY);
    serialPortOpenRetry(s, s->port, IS_BAUD_RATE_BOOTLOADER, 1);
}

// Must be ordered fastest to slowest
static const uint32_t s_baudRateList[] = { IS_BAUD_RATE_BOOTLOADER, IS_BAUD_RATE_BOOTLOADER_LEGACY, IS_BAUD_RATE_BOOTLOADER_RS232, IS_BAUD_RATE_BOOTLOADER_SLOW };

static uint32_t is_isb_cycle_baudrate(uint32_t baudRate)
{
	for (uint32_t i = 0; i < _ARRAY_ELEMENT_COUNT(s_baudRateList); i++)
		if (baudRate == s_baudRateList[i])  // Find current baudrate
			if (i + 1 < _ARRAY_ELEMENT_COUNT(s_baudRateList))   // Get next baudrate
                return s_baudRateList[i + 1];

	return s_baudRateList[0];
}

// Finds the closest bootloader supported baudrate 
static uint32_t is_isb_closest_baudrate(uint32_t baudRate)
{
	for (uint32_t i = 0; i < _ARRAY_ELEMENT_COUNT(s_baudRateList); i++)
		if (baudRate >= s_baudRateList[i]) 
            return s_baudRateList[i];

	return IS_BAUD_RATE_BOOTLOADER_SLOW;
}

static is_operation_result is_isb_sync(serial_port_t* s)
{
    static const uint8_t handshakerChar = 'U';

    // Try to reboot the device in case it is stuck
    is_isb_restart(s);

    // serialPortClose(s);
    // if (serialPortOpenRetry(s, s->port, IS_BAUD_RATE_BOOTLOADER, 1) == 0)
    // {
    //     // can't open the port, fail
    //     return IS_OP_ERROR;
    // }

    // write a 'U' to handshake with the boot loader - once we get a 'U' back we are ready to go
    for (int i = 0; i < BOOTLOADER_RETRIES; i++)
    {
        if (serialPortWriteAndWaitForTimeout(s, &handshakerChar, 1, &handshakerChar, 1, BOOTLOADER_RESPONSE_DELAY))
        {	// Success
            serialPortSleep(s, BOOTLOADER_REFRESH_DELAY);
            return IS_OP_OK;
        }
    }

#if defined(SUPPORT_BOOTLOADER_V5A)     // ONLY NEEDED TO SUPPORT BOOTLOADER v5a.  Delete this and assocated code in Q4 2022 after bootloader v5a is out of circulation. WHJ
    static const unsigned char handshaker[] = "INERTIAL_SENSE_SYNC_DFU";

    // Attempt handshake using extended string for bootloader v5a
    for (int i = 0; i < BOOTLOADER_RETRIES; i++)
    {
        if (serialPortWriteAndWaitForTimeout(s, (const unsigned char*)&handshaker, (int)sizeof(handshaker), &handshakerChar, 1, BOOTLOADER_RESPONSE_DELAY))
        {	// Success
            serialPortSleep(s, BOOTLOADER_REFRESH_DELAY);
            return IS_OP_OK;
        }
    }
#endif

    return IS_OP_ERROR;
}

is_operation_result is_isb_handshake(is_device_context* ctx)
{
    serial_port_t* port = &ctx->handle.port;

	// ensure that we return a valid baud rate
	ctx->handle.baud = IS_BAUD_RATE_BOOTLOADER;

    // try handshaking at each baud rate
    // for (unsigned int i = 0; i < _ARRAY_ELEMENT_COUNT(s_baudRateList) + 1; i++)
    {
        // serialPortClose(port);
        // if (serialPortOpenRetry(port, port->port, IS_BAUD_RATE_BOOTLOADER, 1) == 0)
        // {
        //     // can't open the port, fail
        //     return IS_OP_ERROR;
        // }
        if (is_isb_sync(port) == IS_OP_OK)  //else
        {
            ctx->handle.baud = IS_BAUD_RATE_BOOTLOADER;
            return IS_OP_OK;
        }
    }

//    ctx->info_callback(ctx, "(ISB) Failed to handshake with bootloader", IS_LOG_LEVEL_ERROR);

    // Failed to handshake
    return IS_OP_ERROR;
}

is_operation_result is_isb_enable(is_device_context* ctx, const char* enable_cmd)
{
    serial_port_t* port = &ctx->handle.port;
    uint32_t baudRates[] = { ctx->handle.baud, IS_BAUD_RATE_BOOTLOADER, IS_BAUD_RATE_BOOTLOADER_RS232, IS_BAUD_RATE_BOOTLOADER_SLOW };

    // In case we are in program mode, try and send the commands to go into bootloader mode
    uint8_t c = 0;
    for (size_t i = 0; i < _ARRAY_ELEMENT_COUNT(baudRates); i++)
    {
        if (baudRates[i] == 0)
            continue;

        // serialPortClose(port);
        // if (serialPortOpenRetry(port, port->port, baudRates[i], 1) == 0)
        // {
        //     ctx->info_callback(ctx, "(ISB) Failed to open serial port", IS_LOG_LEVEL_ERROR);
        //     serialPortClose(port);
        //     return IS_OP_ERROR;
        // }
        for (size_t loop = 0; loop < 10; loop++)
        {
            serialPortWriteAscii(port, "STPB", 4);
            serialPortWriteAscii(port, enable_cmd, 4);
            c = 0;
            if (serialPortReadCharTimeout(port, &c, 13) == 1)
            {
                if (c == '$')
                {
                    // done, we got into bootloader mode
                    i = 9999;
                    break;
                }
            }
            else serialPortFlush(port);
        }
    }

    // serialPortClose(port);
    // SLEEP_MS(BOOTLOADER_REFRESH_DELAY);

    // // if we can't handshake at this point, bootloader enable has failed
    // if (is_isb_handshake(ctx) != IS_OP_OK)
    // {
    //     // failure
    //     serialPortClose(port);
    //     return IS_OP_ERROR;
    // }

    // // ensure bootloader restarts in fresh state
//    is_isb_restart(port);

    // // by this point the bootloader should be enabled
    // serialPortClose(port);
    return IS_OP_OK;
}

/**
 * @brief Calculate checksum for Inertial Sense Bootloader
 * 
 * @param checkSum 
 * @param ptr 
 * @param start INCLUSIVE
 * @param end EXCLUSIVE
 * @param checkSumPosition if not 0, the checkSum is written to ptr + checkSumPosition
 * @param finalCheckSum 
 * @return int 
 */
static int is_isb_checksum(int checkSum, uint8_t* ptr, int start, int end, int checkSumPosition, int finalCheckSum)
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

static is_operation_result is_isb_erase_flash(serial_port_t* s)
{
    // give the device 60 seconds to erase flash before giving up
    static const int eraseFlashTimeoutMilliseconds = 60000;
    unsigned char selectFlash[24];

    // Write location to erase at
    memcpy(selectFlash, ":03000006030000F4CC\0\0\0\0\0", 24);
    is_isb_checksum(0, selectFlash, 1, 17, 17, 1);
    if (serialPortWriteAndWaitForTimeout(s, selectFlash, 19, (unsigned char*)".\r\n", 3, BOOTLOADER_TIMEOUT_DEFAULT) == 0) return IS_OP_ERROR;

    // Erase
    memcpy(selectFlash, ":0200000400FFFBCC\0", 18);
    is_isb_checksum(0, selectFlash, 1, 15, 15, 1);
    if (serialPortWriteAndWaitForTimeout(s, selectFlash, 17, (unsigned char*)".\r\n", 3, eraseFlashTimeoutMilliseconds) == 0) return IS_OP_ERROR;

    return IS_OP_OK;
}

static is_operation_result is_isb_select_page(is_device_context* ctx, int page)
{
    serial_port_t* s = &ctx->handle.port;

    // Atmel select page command (0x06) is 4 bytes and the data is always 0301xxxx where xxxx is a 16 bit page number in hex
    unsigned char changePage[24];
    
    // Change page
    SNPRINTF((char*)changePage, 24, ":040000060301%.4XCC", page);
    is_isb_checksum(0, changePage, 1, 17, 17, 1);
    if (serialPortWriteAndWaitForTimeout(s, changePage, 19, (unsigned char*)".\r\n", 3, BOOTLOADER_TIMEOUT_DEFAULT) == 0) 
    {
        ctx->info_callback(ctx, "(ISB) Failed to select page in ISB bootloader", IS_LOG_LEVEL_ERROR);
        return IS_OP_ERROR;
    }

    return IS_OP_OK;
}

static is_operation_result is_isb_begin_program_for_current_page(is_device_context* ctx, int startOffset, int endOffset)
{
    serial_port_t* s = &ctx->handle.port;

    // Atmel begin program command is 0x01, different from standard intel hex where command 0x01 is end of file
    // After the 0x01 is a 00 which means begin writing program
    // The begin program command uses the current page and specifies two 16 bit addresses that specify where in the current page
    //  the program code will be written
    unsigned char programPage[24];
    
    // Select offset
    SNPRINTF((char*)programPage, 24, ":0500000100%.4X%.4XCC", startOffset, endOffset);
    is_isb_checksum(0, programPage, 1, 19, 19, 1);
    if (serialPortWriteAndWaitForTimeout(s, programPage, 21, (unsigned char*)".\r\n", 3, BOOTLOADER_TIMEOUT_DEFAULT) == 0)
    {
        ctx->info_callback(ctx, "(ISB) Failed to start programming page ISB bootloader", IS_LOG_LEVEL_ERROR);
        return IS_OP_ERROR;
    }

    return IS_OP_OK;
}

static int is_isb_read_line(FILE* file, char line[1024])
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

static is_operation_result is_isb_upload_hex_page(is_device_context* ctx, unsigned char* hexData, int byteCount, int* currentOffset, int* totalBytes, int* verifyCheckSum)
{
    serial_port_t* s = &ctx->handle.port;
    int i;

    if (byteCount == 0)
    {
        return IS_OP_OK;
    }

    // create a program request with just the hex characters that will fit on this page
    unsigned char programLine[12];
    SNPRINTF((char*)programLine, 12, ":%.2X%.4X00", byteCount, *currentOffset);
    if (serialPortWrite(s, programLine, 9) != 9)
    {
        ctx->info_callback(ctx, "(ISB) Failed to write start page", IS_LOG_LEVEL_ERROR);
        return IS_OP_ERROR;
    }

    // add the previously written chars to the checksum
    int checkSum = is_isb_checksum(0, programLine, 1, 9, 0, 0);

    // write all of the hex chars
    int charsForThisPage = byteCount * 2;
    if (serialPortWrite(s, hexData, charsForThisPage) != charsForThisPage)
    {
        ctx->info_callback(ctx, "(ISB) Failed to write data to device", IS_LOG_LEVEL_ERROR);
        return IS_OP_ERROR;
    }

    // calculate verification checksum for this data
    for (i = 0; i < charsForThisPage; i++)
    {
        *verifyCheckSum = ((*verifyCheckSum << 5) + *verifyCheckSum) + hexData[i];
    }

    checkSum = is_isb_checksum(checkSum, hexData, 0, charsForThisPage, 0, 1);
    unsigned char checkSumHex[3];
    SNPRINTF((char*)checkSumHex, 3, "%.2X", checkSum);

    if (serialPortWrite(s, checkSumHex, 2) != 2)
    {
        ctx->info_callback(ctx, "(ISB) Failed to write checksum to device", IS_LOG_LEVEL_ERROR);
        return IS_OP_ERROR;
    }

    unsigned char buf[128] = { 0 };
    int count = serialPortReadTimeout(s, buf, 3, BOOTLOADER_TIMEOUT_DEFAULT);

	if (count != 3 || memcmp(buf, ".\r\n", 3) != 0)
	{
        buf[count] = '\0'; 
        ctx->info_callback(ctx, (const char*)buf, IS_LOG_LEVEL_ERROR);
		return IS_OP_ERROR;
	}

    // if (serialPortWriteAndWaitForTimeout(s, checkSumHex, 2, (unsigned char*)".\r\n", 3, BOOTLOADER_TIMEOUT_DEFAULT) == 0)
    // {
    //     ctx->info_callback(ctx, "(ISB) Failed to write checksum to device", IS_LOG_LEVEL_ERROR);
    //     return IS_OP_ERROR;
    // }

    *totalBytes += byteCount;
    *currentOffset += byteCount;

    return IS_OP_OK;
}

static is_operation_result is_isb_upload_hex(is_device_context* ctx, unsigned char* hexData, int charCount, int* currentOffset, int* currentPage, int* totalBytes, int* verifyCheckSum)
{
    if (charCount > MAX_SEND_COUNT)
    {
        ctx->info_callback(ctx, "(ISB) Unexpected char count", IS_LOG_LEVEL_ERROR);
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
        if (is_isb_upload_hex_page(ctx, hexData, pageByteCount, currentOffset, totalBytes, verifyCheckSum) != IS_OP_OK)
        {
            ctx->info_callback(ctx, "(ISB) Failed to upload bytes (1)", IS_LOG_LEVEL_ERROR);
            return IS_OP_ERROR;
        }
        hexData += (pageByteCount * 2);
        charCount -= (pageByteCount * 2);

        // change to the next page
        *currentOffset = 0;
        (*currentPage)++;
        if (is_isb_select_page(ctx, *currentPage) != IS_OP_OK || is_isb_begin_program_for_current_page(ctx, 0, FLASH_PAGE_SIZE - 1) != IS_OP_OK)
        {
            ctx->info_callback(ctx, "(ISB) Failed to issue select page or to start programming", IS_LOG_LEVEL_ERROR);
            return IS_OP_ERROR;
        }
    }

    if (charCount != 0 && is_isb_upload_hex_page(ctx, hexData, charCount / 2, currentOffset, totalBytes, verifyCheckSum) != IS_OP_OK)
    {
        ctx->info_callback(ctx, "(ISB) Failed to upload bytes (2)", IS_LOG_LEVEL_ERROR);
        return IS_OP_ERROR;
    }

    return IS_OP_OK;
}

static is_operation_result is_isb_fill_current_page(is_device_context* ctx, int* currentPage, int* currentOffset, int* totalBytes, int* verifyCheckSum)
{
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

            if (is_isb_upload_hex_page(ctx, hexData, byteCount / 2, currentOffset, totalBytes, verifyCheckSum) != IS_OP_OK)
            {
                ctx->info_callback(ctx, "(ISB) Failed to fill page with bytes", IS_LOG_LEVEL_ERROR);
                return IS_OP_ERROR;
            }
        }
    }

    return IS_OP_OK;
}

static is_operation_result is_isb_download_data(is_device_context* ctx, int startOffset, int endOffset)
{
    serial_port_t* s = &ctx->handle.port;

    // Atmel download data command is 0x03, different from standard intel hex where command 0x03 is start segment address
    unsigned char programLine[24];
    int n;
    n = SNPRINTF((char*)programLine, 24, ":0500000300%.4X%.4XCC", startOffset, endOffset);
    programLine[n] = 0;
    is_isb_checksum(0, programLine, 1, 19, 19, 1);
    if (serialPortWrite(s, programLine, 21) != 21)
    {
        ctx->info_callback(ctx, "(ISB) Failed to attempt download", IS_LOG_LEVEL_ERROR);
        return IS_OP_ERROR;
    }

    return IS_OP_OK;
}

static is_operation_result is_isb_verify(int lastPage, int checkSum, is_device_context* ctx)
{
    int verifyChunkSize = ctx->props.isb.verify_size;
    int chunkSize = _MIN(FLASH_PAGE_SIZE, verifyChunkSize);
    int realCheckSum = 5381;
    int totalCharCount = ctx->props.isb.app_offset * 2;
    int grandTotalCharCount = (lastPage + 1) * FLASH_PAGE_SIZE * 2; // char count
    int i, pageOffset, readCount, actualPageOffset, pageChars, chunkIndex, lines;
    int verifyByte = -1;
    unsigned char chunkBuffer[(MAX_VERIFY_CHUNK_SIZE * 2) + 64]; // extra space for overhead
    ctx->update_progress = 0.0f;
    unsigned char c=0;
    FILE* verifyFile = 0;

    if (ctx->verify_path != 0)
    {
#ifdef _MSC_VER
        fopen_s(&verifyFile, ctx->verify_path, "wb");
#else
        verifyFile = fopen(ctx->verify_path, "wb");
#endif
    }

    for (i = 0; i <= lastPage; i++)
    {
        if (is_isb_select_page(ctx, i) != IS_OP_OK)
        {
            ctx->info_callback(ctx, "(ISB) Failure issuing select page command for verify", IS_LOG_LEVEL_ERROR);
            return IS_OP_ERROR;
        }
        pageOffset = (i == 0 ? ctx->props.isb.app_offset : 0);
        while (pageOffset < FLASH_PAGE_SIZE)
        {
            readCount = _MIN(chunkSize, FLASH_PAGE_SIZE - pageOffset);

            // range is inclusive on the uINS, so subtract one
            if (is_isb_download_data(ctx, pageOffset, pageOffset + readCount - 1) == 0)
            {
                ctx->info_callback(ctx, "(ISB) Failure issuing download data command", IS_LOG_LEVEL_ERROR);
                return IS_OP_ERROR;
            }

            // each line has 7 overhead bytes, plus two bytes (hex) for each byte on the page and max 255 bytes per line
            lines = (int)ceilf((float)readCount / 255.0f);
            readCount = serialPortReadTimeout(&ctx->handle.port, chunkBuffer, (7 * lines) + (readCount * 2), BOOTLOADER_TIMEOUT_DEFAULT);
            chunkIndex = 0;

            while (chunkIndex < readCount)
            {
                if (chunkIndex > readCount - 5)
                {
                    ctx->info_callback(ctx, "(ISB) Unexpected start line during verify (1)", IS_LOG_LEVEL_ERROR);
                    return IS_OP_ERROR;
                }

                // skip the first 5 chars, they are simply ####=
                if (chunkBuffer[chunkIndex] == 'X')
                {
                    ctx->info_callback(ctx, "(ISB) Invalid checksum during verify", IS_LOG_LEVEL_ERROR);
                    return IS_OP_ERROR;
                }
                else if (chunkBuffer[chunkIndex += 4] != '=')
                {
                    ctx->info_callback(ctx, "(ISB) Unexpected start line during verify (2)", IS_LOG_LEVEL_ERROR);
                    return IS_OP_ERROR;
                }
                chunkBuffer[chunkIndex] = '\0';
                actualPageOffset = strtol((char*)(chunkBuffer + chunkIndex - 4), 0, 16);
                if (actualPageOffset != pageOffset)
                {
                    ctx->info_callback(ctx, "(ISB) Unexpected offset during verify", IS_LOG_LEVEL_ERROR);
                    return 0;
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
                        ctx->info_callback(ctx, "(ISB) Unexpected hex data during verify", IS_LOG_LEVEL_ERROR);
                        return IS_OP_ERROR;
                    }
                }

                if (c != '\n')
                {
                    ctx->info_callback(ctx, "(ISB) Unexpected end of lin char during verify", IS_LOG_LEVEL_ERROR);
                    return IS_OP_ERROR;
                }

                // increment page offset
                pageOffset += (pageChars / 2);

                if (ctx->verify_callback != 0)
                {
                    ctx->verify_progress = (float)totalCharCount / (float)grandTotalCharCount;
                    if (ctx->verify_callback(ctx, ctx->verify_progress) != IS_OP_OK)
                    {
                        ctx->info_callback(ctx, "(ISB) Firmware validate cancelled", IS_LOG_LEVEL_ERROR);
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

    if (realCheckSum != checkSum)
    {
        ctx->info_callback(ctx, "(ISB) Checksum mismatch during verify", IS_LOG_LEVEL_ERROR);
        return IS_OP_ERROR;
    }

    return IS_OP_OK;
}

#define HEX_BUFFER_SIZE 1024

static is_operation_result is_isb_process_hex_file(FILE* file, is_device_context* ctx)
{
    int currentPage = 0;
    int currentOffset = ctx->props.isb.app_offset;
    int lastSubOffset = currentOffset;
    int subOffset;
    int totalBytes = ctx->props.isb.app_offset;

    int verifyCheckSum = 5381;
    int lineLength;
    ctx->update_progress = 0.0f;
    char line[HEX_BUFFER_SIZE];
    unsigned char output[HEX_BUFFER_SIZE * 2]; // big enough to store an entire extra line of buffer if needed
    unsigned char* outputPtr = output;
    const unsigned char* outputPtrEnd = output + (HEX_BUFFER_SIZE * 2);
    int outputSize;
    int page;
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
                ctx->info_callback(ctx, "(ISB) hex file line length too long", IS_LOG_LEVEL_ERROR);
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
                    ctx->info_callback(ctx, "(ISB) FF padding overflowed buffer", IS_LOG_LEVEL_ERROR);
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
                ctx->info_callback(ctx, "(ISB) Line data overflowed output buffer", IS_LOG_LEVEL_ERROR);
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
            else if (is_isb_upload_hex(ctx, output, _MIN(MAX_SEND_COUNT, outputSize), &currentOffset, &currentPage, &totalBytes, &verifyCheckSum) != IS_OP_OK)
            {
                return IS_OP_ERROR;
            }

            outputSize -= MAX_SEND_COUNT;

            if (outputSize < 0 || outputSize > HEX_BUFFER_SIZE)
            {
                ctx->info_callback(ctx, "(ISB) Output size was too large (1)", IS_LOG_LEVEL_ERROR);
                return IS_OP_ERROR;
            }
            else if (outputSize > 0)
            {
                // move the left-over data to the beginning
                memmove(output, output + MAX_SEND_COUNT, outputSize);
            }

            // reset output ptr back to the next chunk of data
            outputPtr = output + outputSize;
        }
        else if (strncmp(line, ":020000048", 10) == 0 && strlen(line) >= 13)
        {
            memcpy(tmp, line + 10, 3);
            tmp[3] = '\0';
            page = strtol((char*)tmp, 0, 16);

            if (page != 0)
            {
                // we found a change page command beyond the first, let's finish up this page and fill it to the max
                lastSubOffset = 0;
                outputSize = (int)(outputPtr - output);

                if (outputSize < 0 || outputSize > HEX_BUFFER_SIZE)
                {
                    ctx->info_callback(ctx, "(ISB) Output size was too large (2)", IS_LOG_LEVEL_ERROR);
                    return IS_OP_ERROR;
                }
                // flush the remainder of data to the page
                else if (is_isb_upload_hex(ctx, output, outputSize, &currentOffset, &currentPage, &totalBytes, &verifyCheckSum) != IS_OP_OK)
                {
                    return IS_OP_ERROR;
                }
                // fill the remainder of the current page, the next time that bytes try to be written the page will be automatically incremented
                else if (is_isb_fill_current_page(ctx, &currentPage, &currentOffset, &totalBytes, &verifyCheckSum) != IS_OP_OK)
                {
                    return IS_OP_ERROR;
                }

                // set the output ptr back to the beginning, no more data is in the queue
                outputPtr = output;
            }
        }

        if (ctx->update_callback != 0)
        {
            ctx->update_progress = (float)ftell(file) / (float)fileSize;	// Dummy line to call ftell() once
            ctx->update_progress = (float)ftell(file) / (float)fileSize;
            if (ctx->update_callback(ctx, ctx->update_progress) != IS_OP_OK)
            {
                ctx->info_callback(ctx, "(ISB) Firmware update cancelled", IS_LOG_LEVEL_ERROR);
                return IS_OP_CANCELLED;
            }
        }
    }

    // upload any left over data
    outputSize = (int)(outputPtr - output);
    if (is_isb_upload_hex(ctx, output, outputSize, &currentOffset, &currentPage, &totalBytes, &verifyCheckSum) != IS_OP_OK)
    {
        return IS_OP_ERROR;
    }

    // pad the remainder of the page with fill bytes
    if (currentOffset != 0 && is_isb_fill_current_page(ctx, &currentPage, &currentOffset, &totalBytes, &verifyCheckSum) != IS_OP_OK)
    {
        return IS_OP_ERROR;
    }

    if (ctx->update_callback != 0 && ctx->update_progress != 1.0f)
    {
        ctx->update_progress = 1.0f;
        ctx->update_callback(ctx, ctx->update_progress);
    }

    if (ctx->verify_callback != 0 && ctx->verify_progress != 0)
    {
        if (ctx->info_callback != 0) ctx->info_callback(ctx, "Verifying flash...", IS_LOG_LEVEL_INFO);

        if (is_isb_verify(currentPage, verifyCheckSum, ctx) != IS_OP_OK)
        {
            return IS_OP_ERROR;
        }
    }

    return IS_OP_OK;
}

is_operation_result is_isb_flash(is_device_context* ctx)
{
    FILE* firmware_file = 0;

#ifdef _MSC_VER
    fopen_s(&firmware_file, ctx->firmware_path, "rb");
#else
    firmware_file = fopen(ctx->firmware_path, "rb");
#endif

    // Sync with bootloader
    if(is_isb_handshake(ctx) != IS_OP_OK)
        return IS_OP_ERROR;
    if(is_isb_negotiate_version(ctx) != IS_OP_OK) // negotiate version
        return IS_OP_ERROR;
    if(is_isb_get_version(ctx) != IS_OP_OK)
        return IS_OP_ERROR;

    if(ctx->info_callback != 0)
        ctx->info_callback(ctx, "Erasing flash...", IS_LOG_LEVEL_INFO);
    if(is_isb_erase_flash(&ctx->handle.port) != IS_OP_OK)
        return IS_OP_ERROR;
    if(is_isb_select_page(ctx, 0) != IS_OP_OK)
        return IS_OP_ERROR;

    if(ctx->info_callback != 0)
        ctx->info_callback(ctx, "Programming flash...", IS_LOG_LEVEL_INFO);
    if(is_isb_begin_program_for_current_page(ctx, ctx->props.isb.app_offset, FLASH_PAGE_SIZE - 1) != IS_OP_OK)
        return IS_OP_ERROR;

    is_isb_process_hex_file(firmware_file, ctx);

    fclose(firmware_file);
    // serialPortClose(&ctx->handle.port);

    return IS_OP_OK;
}

/**
 * @brief Gets the version (e.g. 6a) from the bootloader file. Should be used in 
 *  conjunction with the function that gets the signature from the firmware 
 *  image.
 * 
 * @param filename file name of the bootloader
 * @param major filled with major version
 * @param minor filled with minor version
 * @return is_operation_result 
 */
is_operation_result is_isb_get_version_from_file(const char* filename, uint8_t* major, char* minor)
{
    FILE* blfile = 0;

#ifdef _MSC_VER
    fopen_s(&blfile, filename, "rb");
#else
    blfile = fopen(filename, "rb");
#endif

    if (blfile == 0)
        return IS_OP_ERROR;

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

/**
 * @brief Cycles through all valid baud rates and commands the bootloader back
 *  into application mode
 * 
 * @param port 
 * @return is_operation_result 
 */
is_operation_result is_isb_reboot_to_app(serial_port_t* port)
{
    is_operation_result ret = IS_OP_OK;

    for(uint32_t i = 0; i < _ARRAY_ELEMENT_COUNT(s_baudRateList); i++)
    {
        // serialPortClose(port);
        // if (serialPortOpenRetry(port, port->port, s_baudRateList[i], 1) == 0)
        // {
            // ret = IS_OP_ERROR; continue;
        // }
        
        // send the "reboot to program mode" command and the device should start in program mode
        serialPortWrite(port, (unsigned char*)":020000040300F7", 15);
        serialPortSleep(port, 250);
    }

    // serialPortClose(port);

    return ret;
}

is_operation_result is_isb_get_version(is_device_context* ctx)
{
    serialPortClose(&ctx->handle.port);
    serialPortOpenRetry(&ctx->handle.port, ctx->handle.port.port, ctx->handle.baud, 1);
	
    serialPortFlush(&ctx->handle.port);

	// Send command
	serialPortWrite(&ctx->handle.port, (uint8_t*)":020000041000EA", 15);

    uint8_t buf[14] = { 0 };

    // Read Version, SAM-BA Available, serial number (in version 6+) and ok (.\r\n) response
	int count = serialPortReadTimeout(&ctx->handle.port, buf, 14, 1000);

    if (count < 8 || buf[0] != 0xAA || buf[1] != 0x55)
    {   // Bad read
        ctx->props.isb.major = 0;
        ctx->props.isb.minor = 0;
        ctx->props.isb.rom_available = 1;
        ctx->props.isb.processor = IS_PROCESSOR_SAMx70;
        ctx->props.isb.is_evb = false;
        ctx->props.serial = 0;
        return IS_OP_ERROR;
    }

    ctx->props.isb.major = buf[2];
    ctx->props.isb.minor = buf[3];
    ctx->props.isb.rom_available = buf[4];

    if(buf[11] == '.' && buf[12] == '\r' && buf[13] == '\n')
    {
        ctx->props.isb.processor = buf[5];
        ctx->props.isb.is_evb = buf[6];
        memcpy(&ctx->props.serial, &buf[7], sizeof(uint32_t));
    }
    else
    {
        ctx->props.serial = 0;
    }

    return IS_OP_OK;
}

