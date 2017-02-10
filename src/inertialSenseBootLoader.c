/*
MIT LICENSE

Copyright 2014 Inertial Sense, LLC - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include <stdio.h>
#include <math.h>
#include "inertialSenseBootLoader.h"

#define ENABLE_HEX_BOOT_LOADER 1
#define MAX_SEND_COUNT 510
#define MAX_VERIFY_CHUNK_SIZE 1024

// logical page size, offsets for pages are 0x0000 to 0xFFFF - flash page size on devices will vary and is not relevant to the bootloader client
#define PAGE_SIZE 65536

typedef struct
{
	int version;
	int firstPageSkipBytes;
	int verifyChunkSize;
} bootloader_state_t;

#if defined(_WIN32)

#define bootloader_snprintf _snprintf

#else

#define bootloader_snprintf snprintf

#endif

#define bootloader_perror(s, ...) \
	if (s->error) \
	{ \
		bootloader_snprintf(s->error, s->errorLength, __VA_ARGS__); \
	}

#define bootloader_min(a, b) (a < b ? a : b)
#define bootloader_max(a, b) (a > b ? a : b)


char g_enableBootloaderVerify = 1;		// Allow users to skip verify


// negotiate the bootloader version, once a 'U' character has been read, we read another character, timing out after 500 milliseconds
// if nothing comes back, we are using version 1, otherwise the version is the number sent back
static int bootloaderNegotiateVersion(serial_port_t* s, bootloader_state_t* state)
{
	unsigned char v;
	if (serialPortReadCharTimeout(s, &v, 500) == 0 || v == '1')
	{
		// version 1
		state->version = 1;
		state->firstPageSkipBytes = 8192;
}
	else if (v == '2')
	{
		// version 2
		state->version = 2;
		state->firstPageSkipBytes = 16384;
	}
	else
	{
		bootloader_perror(s, "Invalid version sent from bootloader: 0x%02X", (int)v);
		return 0;
	}

#if defined(WIN32) || defined(WIN64)

	// eval tool and multiple bootloads under Windows 10 have issues with dropped data if verify runs too fast
	state->verifyChunkSize = 125;

#else

	state->verifyChunkSize = MAX_VERIFY_CHUNK_SIZE;

#endif

	return 1;
}

static int serialPortOpenInternal(serial_port_t* s, int baudRate, char* error, int errorLength)
{
	if (error != NULL && errorLength > 0)
	{
		*error = '\0';
	}
	serialPortClose(s);
	s->error = error;
	s->errorLength = errorLength;
	if (serialPortOpen(s, s->port, baudRate, 1) == 0)
	{
		bootloader_perror(s, "Unable to open serial port at %s", s->port);
		return 0;
	}

	return 1;
}

// calculate checksum - start is INCLUSIVE and end is EXCLUSIVE, if checkSumPosition is not 0, the checkSum is written to ptr + checkSumPosition
static int bootloaderChecksum(int checkSum, unsigned char* ptr, int start, int end, int checkSumPosition, int finalCheckSum)
{
	unsigned char c1, c2;
	unsigned char* currentPtr = (unsigned char*)(ptr + start);
	unsigned char* endPtr = (unsigned char*)(ptr + end - 1);
	unsigned char b;

	while (currentPtr < endPtr)
	{
		c1 = *(currentPtr++) | 0x20;
		c1 = (c1 <= '9' ? c1 + 0xD0 : c1 + 0xA9);
		c2 = *(currentPtr++) | 0x20;
		c2 = (c2 <= '9' ? c2 + 0xD0 : c2 + 0xA9);
		b = (c1 << 4) | c2;
		checkSum += b;
	}

	if (finalCheckSum)
	{
		checkSum = (unsigned char)(~checkSum + 1);
	}
	if (checkSumPosition != 0)
	{
		bootloader_snprintf((char*)(ptr + checkSumPosition), 3, "%.2X", checkSum);
	}

	return checkSum;
}

static int bootloaderEraseFlash(serial_port_t* s)
{
	// give the device this many seconds to erase flash before giving up
	static const int eraseFlashTimeoutMilliseconds = 60000;

	unsigned char selectFlash[24];
	
	memcpy(selectFlash, ":03000006030000F4CC\0\0\0\0\0", 24);
	bootloaderChecksum(0, selectFlash, 1, 17, 17, 1);
	if (serialPortWriteAndWaitFor(s, selectFlash, 19, (unsigned char*)".\r\n", 3) == 0)
	{
		bootloader_perror(s, "Failed to select flash memory to erase");
		return 0;
	}

	memcpy(selectFlash, ":0200000400FFFBCC\0", 18);
	bootloaderChecksum(0, selectFlash, 1, 15, 15, 1);
	if (serialPortWriteAndWaitForTimeout(s, selectFlash, 17, (unsigned char*)".\r\n", 3, eraseFlashTimeoutMilliseconds) == 0)
	{
		bootloader_perror(s, "Failed to perform erase flash memory operation");
		return 0;
	}

	return 1;
}

static int bootloaderSelectPage(serial_port_t* s, int page)
{
	// Atmel select page command (0x06) is 4 bytes and the data is always 0301xxxx where xxxx is a 16 bit page number in hex
	unsigned char changePage[24];
	bootloader_snprintf((char*)changePage, 24, ":040000060301%.4XCC", page);
	bootloaderChecksum(0, changePage, 1, 17, 17, 1);

	if (serialPortWriteAndWaitFor(s, changePage, 19, (unsigned char*)".\r\n", 3) == 0)
	{
		bootloader_perror(s, "Failed to change to page %d", page);
		return 0;
	}

	return 1;
}

static int bootloaderBeginProgramForCurrentPage(serial_port_t* s, int startOffset, int endOffset)
{
	// Atmel begin program command is 0x01, different from standard intel hex where command 0x01 is end of file
	// After the 0x01 is a 00 which means begin writing program
	// The begin program command uses the current page and specifies two 16 bit addresses that specify where in the current page
	//  the program code will be written
	unsigned char programPage[24];
	bootloader_snprintf((char*)programPage, 24, ":0500000100%.4X%.4XCC", startOffset, endOffset);
	bootloaderChecksum(0, programPage, 1, 19, 19, 1);

	if (serialPortWriteAndWaitFor(s, programPage, 21, (unsigned char*)".\r\n", 3) == 0)
	{
		bootloader_perror(s, "Failed to select offset %X to %X", startOffset, endOffset);
		return 0;
	}

	return 1;
}

static int bootloaderReadLine(FILE* file, char line[1024])
{
	char c;
	char* currentPtr = line;
	char* endPtr = currentPtr + 1023;

	while (currentPtr != endPtr)
	{
		// read one char
		c = fgetc(file);
		if (c == '\r')
		{
			// eat '\r' chars
			continue;
		}
		else if (c == '\n' || c == (char)EOF)
		{
			// newline char, we have a line
			break;
		}
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

static int bootloaderUploadHexDataPage(serial_port_t* s, unsigned char* hexData, int byteCount, int* currentOffset, int* totalBytes, int* verifyCheckSum)
{
	int i;

	if (byteCount == 0)
	{
		return 1;
	}

	// create a program request with just the hex characters that will fit on this page
	unsigned char programLine[12];
	bootloader_snprintf((char*)programLine, 12, ":%.2X%.4X00", byteCount, *currentOffset);
	if (serialPortWrite(s, programLine, 9) != 9)
	{
		bootloader_perror(s, "Failed to write start page at offset %d", *currentOffset);
		return 0;
	}

	// add the previously written chars to the checksum
	int checkSum = bootloaderChecksum(0, programLine, 1, 9, 0, 0);

	// write all of the hex chars
	int charsForThisPage = byteCount * 2;
	if (serialPortWrite(s, hexData, charsForThisPage) != charsForThisPage)
	{
		bootloader_perror(s, "Failed to write page data at offset %d", *currentOffset);
		return 0;
	}

	// calculate verification checksum for this data
	for (i = 0; i < charsForThisPage; i++)
	{
		*verifyCheckSum = ((*verifyCheckSum << 5) + *verifyCheckSum) + hexData[i];
	}

	checkSum = bootloaderChecksum(checkSum, hexData, 0, charsForThisPage, 0, 1);
	unsigned char checkSumHex[3];
	bootloader_snprintf((char*)checkSumHex, 3, "%.2X", checkSum);
	if (serialPortWriteAndWaitFor(s, checkSumHex, 2, (unsigned char*)".\r\n", 3) == 0)
	{
		bootloader_perror(s, "Failed to write checksum %s at offset %d", checkSumHex, *currentOffset);
		return 0;
	}

	*totalBytes += byteCount;
	*currentOffset += byteCount;

	return 1;
}

static int bootloaderUploadHexData(serial_port_t* s, unsigned char* hexData, int charCount, int* currentOffset, int* currentPage, int* totalBytes, int* verifyCheckSum)
{
	if (charCount > MAX_SEND_COUNT)
	{
		bootloader_perror(s, "Unexpected char count of %d for page %d at offset %X", charCount, *currentPage, *currentOffset);
		return 0;
	}
	else if (charCount == 0)
	{
		return 1;
	}

	int byteCount = charCount / 2;

	// check if we will overrun the current page
	if (*currentOffset + byteCount > PAGE_SIZE)
	{
		int pageByteCount = PAGE_SIZE - *currentOffset;
		if (bootloaderUploadHexDataPage(s, hexData, pageByteCount, currentOffset, totalBytes, verifyCheckSum) == 0)
		{
			bootloader_perror(s, "Failed to upload %d bytes to page %d at offset %d", pageByteCount, *currentPage, *currentOffset);
			return 0;
		}
		hexData += (pageByteCount * 2);
		charCount -= (pageByteCount * 2);

		// change to the next page
		*currentOffset = 0;
		(*currentPage)++;
		bootloaderSelectPage(s, *currentPage);
		bootloaderBeginProgramForCurrentPage(s, 0, PAGE_SIZE - 1);
	}

	if (charCount != 0 && bootloaderUploadHexDataPage(s, hexData, charCount / 2, currentOffset, totalBytes, verifyCheckSum) == 0)
	{
		bootloader_perror(s, "Failed to upload %d bytes to page %d at offset %d", charCount / 2, *currentPage, *currentOffset);
		return 0;
	}

	return 1;
}

static int bootloaderFillCurrentPage(serial_port_t* s, int* currentPage, int* currentOffset, int* totalBytes, int* verifyCheckSum)
{
	if (*currentOffset < PAGE_SIZE)
	{
		unsigned char hexData[256];
		memset(hexData, 'F', 256);

		while (*currentOffset < PAGE_SIZE)
		{
			int byteCount = (PAGE_SIZE - *currentOffset) * 2;
			if (byteCount > 256)
			{
				byteCount = 256;
			}
			memset(hexData, 'F', byteCount);

			if (bootloaderUploadHexDataPage(s, hexData, byteCount / 2, currentOffset, totalBytes, verifyCheckSum) == 0)
			{
				bootloader_perror(s, "Failed to fill page %d with %d bytes at offset %d", *currentPage, byteCount, *currentOffset);
				return 0;
			}
		}
	}

	return 1;
}

static int bootloaderDownloadData(serial_port_t* s, int startOffset, int endOffset)
{
	// Atmel download data command is 0x03, different from standard intel hex where command 0x03 is start segment address
	unsigned char programLine[24];
	int n;
	n = bootloader_snprintf((char*)programLine, 24, ":0500000300%.4X%.4XCC", startOffset, endOffset);
	programLine[n] = 0;
	bootloaderChecksum(0, programLine, 1, 19, 19, 1);
	if (serialPortWrite(s, programLine, 21) != 21)
	{
		bootloader_perror(s, "Failed to download offsets %X to %X", startOffset, endOffset);
		return 0;
	}

	return 1;
}


static int bootloaderVerify(serial_port_t* s, int lastPage, int checkSum, const void* obj, pfnBootloadProgress verifyProgress, bootloader_state_t* state, const char* verifyFileName)
{
	int verifyChunkSize = state->verifyChunkSize;
	int chunkSize = bootloader_min(PAGE_SIZE, verifyChunkSize);
	int realCheckSum = 5381;
	int totalCharCount = state->firstPageSkipBytes * 2;
	int grandTotalCharCount = (lastPage + 1) * PAGE_SIZE * 2; // char count
	int i, pageOffset, readCount, actualPageOffset, pageChars, chunkIndex, lines;
	int verifyByte = -1;
	unsigned char chunkBuffer[(MAX_VERIFY_CHUNK_SIZE * 2) + 64]; // extra space for overhead
	float percent = 0.0f;
	unsigned char c;
	FILE* verifyFile = 0;

	if (!g_enableBootloaderVerify)
		return 1;	// Skip verify

	if (verifyFileName != 0)
	{

#ifdef _MSC_VER

        fopen_s(&verifyFile, verifyFileName, "wb");

#else

        verifyFile = fopen(verifyFileName, "wb");

#endif

    }

	for (i = 0; i <= lastPage; i++)
	{
		if (bootloaderSelectPage(s, i) == 0)
		{
			bootloader_perror(s, "Failure issuing select page command");
			return 0;
		}
		pageOffset = (i == 0 ? state->firstPageSkipBytes : 0);
		while (pageOffset < PAGE_SIZE)
		{
			readCount = bootloader_min(chunkSize, PAGE_SIZE - pageOffset);

			// range is inclusive on the uINS, so subtract one
			if (bootloaderDownloadData(s, pageOffset, pageOffset + readCount - 1) == 0)
			{
				bootloader_perror(s, "Failure issuing download data command");
				return 0;
			}

			// each line has 7 overhead bytes, plus two bytes (hex) for each byte on the page and max 255 bytes per line
			lines = (int)ceilf((float)readCount / 255.0f);
			readCount = serialPortRead(s, chunkBuffer, (7 * lines) + (readCount * 2));
			chunkIndex = 0;

			while (chunkIndex < readCount)
			{
				if (chunkIndex > readCount - 5)
				{
					bootloader_perror(s, "Unexpected start line during validation");
					return 0;
				}

				// skip the first 5 chars, they are simply ####=
				if (chunkBuffer[chunkIndex] == 'X')
				{
					bootloader_perror(s, "Invalid checksum during validation");
					return 0;
				}
				else if (chunkBuffer[chunkIndex += 4] != '=')
				{
					bootloader_perror(s, "Unexpected start line during validation");
					return 0;
				}
				chunkBuffer[chunkIndex] = '\0';
				actualPageOffset = strtol((char*)(chunkBuffer + chunkIndex - 4), 0, 16);
				if (actualPageOffset != pageOffset)
				{
					bootloader_perror(s, "Unexpected offset during validation");
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
						bootloader_perror(s, "Unexpected hex data during validation");
						return 0;
					}
				}

				if (c != '\n')
				{
					bootloader_perror(s, "Unexpected end line character found during validation: 0x%02x [%c]", c, c);
					return 0;
				}

				// increment page offset
				pageOffset += (pageChars / 2);

				if (verifyProgress != NULL)
				{
					percent = (float)totalCharCount / (float)grandTotalCharCount;
					if (verifyProgress(obj, percent) == 0)
					{
						bootloader_perror(s, "Validate firmware cancelled");
						return 0;
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
		bootloader_perror(s, "Download checksum 0x%08x != calculated checksum 0x%08x", realCheckSum, checkSum);
		return 0;
	}

	return 1;
}


static int bootloaderProcessHexFile(FILE* file, serial_port_t* s, const void* obj, pfnBootloadProgress uploadProgress, pfnBootloadProgress verifyProgress, bootloader_state_t* state, const char* verifyFileName)
{

#define BUFFER_SIZE 1024

	int currentPage = 0;
	int currentOffset = state->firstPageSkipBytes;
	int lastSubOffset = currentOffset;
	int subOffset;
	int totalBytes = state->firstPageSkipBytes;

	int verifyCheckSum = 5381;
	int lineLength;
	float percent = 0.0f;
	char line[BUFFER_SIZE];
	unsigned char output[BUFFER_SIZE * 2]; // big enough to store an entire extra line of buffer if needed
	unsigned char* outputPtr = output;
	int outputSize;
	int page;
	int pad;
	int fileSize;
	unsigned char tmp[5];
	int i;

	fseek(file, 0, SEEK_END);
	fileSize = ftell(file);
	fseek(file, 0, SEEK_SET);

	while ((lineLength = bootloaderReadLine(file, line))!= 0)
	{
		if (lineLength > 12 && line[7] == '0' && line[8] == '0')
		{
			// we need to know the offset that this line was supposed to be stored at so we can check if offsets are skipped
			memcpy(tmp, line + 3, 4);
			tmp[4] = '\0';
			subOffset = strtol((char*)tmp, NULL, 16);

			// check if we skipped an offset, the intel hex file format can do this, in which case we need to make sure
			// that the bytes that were skipped get set to something
			if (subOffset > lastSubOffset)
			{
				// pad with FF bytes, this is an internal implementation detail to how the device stores unused memory
				pad = (subOffset - lastSubOffset);
				while (pad-- != 0)
				{
					*outputPtr++ = 'F';
					*outputPtr++ = 'F';
				}
			}

			// skip the first 9 chars which are not data, then take everything else minus the last two chars which are a checksum
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
			else if (!bootloaderUploadHexData(s, output, (outputSize > MAX_SEND_COUNT ? MAX_SEND_COUNT : outputSize), &currentOffset, &currentPage, &totalBytes, &verifyCheckSum))
			{
				return 0;
			}

			outputSize -= MAX_SEND_COUNT;

			if (outputSize < 0 || outputSize > BUFFER_SIZE)
			{
				bootloader_perror(s, "Output size of %d was too large", outputSize);
				return 0;
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
			page = strtol((char*)tmp, NULL, 16);

			if (page != 0)
			{
				// we found a change page command beyond the first, let's finish up this page and fill it to the max
				lastSubOffset = 0;
				outputSize = (int)(outputPtr - output);

				if (outputSize < 0 || outputSize > BUFFER_SIZE)
				{
					bootloader_perror(s, "Output size of %d was too large", outputSize);
					return 0;
				}

				// flush the remainder of data to the page
				else if (bootloaderUploadHexData(s, output, outputSize, &currentOffset, &currentPage, &totalBytes, &verifyCheckSum) == 0)
				{
					return 0;
				}

				// fill the remainder of the current page, the next time that bytes try to be written the page will be automatically incremented
				else if (bootloaderFillCurrentPage(s, &currentPage, &currentOffset, &totalBytes, &verifyCheckSum) == 0)
				{
					return 0;
				}

				// set the output ptr back to the beginning, no more data is in the queue
				outputPtr = output;
			}
		}

		if (uploadProgress != NULL)
		{
			percent = (float)ftell(file) / (float)fileSize;	// Dummy line to call ftell() once
			percent = (float)ftell(file) / (float)fileSize;
			if (uploadProgress(obj, percent) == 0)
			{
				bootloader_perror(s, "Upload firmware cancelled");
				return 0;
			}
		}
	}
	
	// upload any left over data
	outputSize = (int)(outputPtr - output);
	if (bootloaderUploadHexData(s, output, outputSize, &currentOffset, &currentPage, &totalBytes, &verifyCheckSum) == 0)
	{
		return 0;
	}

	// pad the remainder of the page with fill bytes
	if (currentOffset != 0 && bootloaderFillCurrentPage(s, &currentPage, &currentOffset, &totalBytes, &verifyCheckSum) == 0)
	{
		return 0;
	}

	if (uploadProgress != NULL && percent != 1.0f)
	{
		uploadProgress(obj, 1.0f);
	}

	if (g_enableBootloaderVerify)
	{
		if (verifyProgress != NULL && bootloaderVerify(s, currentPage, verifyCheckSum, obj, verifyProgress, state, verifyFileName) == 0)
		{
			return 0;
		}
	}

	// send the "reboot to program mode" command and the device should start in program mode
	serialPortWrite(s, (unsigned char*)":020000040300F7", 15);
	serialPortSleep(s, 250);

	return 1;
}

int bootloaderProcessBinFile(FILE* file, serial_port_t* s, const void* obj, pfnBootloadProgress uploadProgress, pfnBootloadProgress verifyProgress, const char* verifyFileName)
{
	bootloader_state_t state = { 0 };
	unsigned char c;
	unsigned char hexLookupTable[] = { '0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', 'B', 'C', 'D', 'E', 'F' };
	int dataLength;
	int verifyCheckSum;
	int lastPage;
	int commandCount;
	int commandLength;
	unsigned char buf[16];
	unsigned char commandType;
	float percent = 0.0f;
	fseek(file, 0, SEEK_END);
	int fileSize = ftell(file);
	fseek(file, 0, SEEK_SET);

	// verify checksum is first four bytes
	verifyCheckSum = fgetc(file) | (fgetc(file) << 8) | (fgetc(file) << 16) | (fgetc(file) << 24);

	// last page is next two bytes
	lastPage = fgetc(file) | (fgetc(file) << 8);

	// the file is formatted like this:
	// 4 bytes  : verify checksum
	// 2 bytes  : last page
	// LOOP:
	// 1 byte - command type (0 = data, 1 = raw)
	// 2 bytes - number of commands
	// n commands...
	// Data command (0)
	//	2 bytes : Offset
	//  1 byte  : Data Length
	//  n bytes : Data
	//  1 byte  : Checksum
	// Raw command (1)
	//  1 byte  : Command length
	//  n bytes : Command characters
	//  1 byte  : wait for character, '\0' for none
	//  1 byte  : sleep time after command is sent, in 250 millisecond intervals, 0 for no sleep
	//  1 byte  : wait for character timeout time in 250 millisecond intervals, 0 for no wait
	// if not EOF, go to LOOP

	while (ftell(file) != fileSize)
	{
		commandType = fgetc(file);
		commandCount = fgetc(file) | (fgetc(file) << 8);

		if (commandType == 0)
		{
			while (commandCount-- > 0)
			{
				// write the :, data length, offset and 00 (data command)
				commandLength = (fgetc(file) | (fgetc(file) << 8));
				dataLength = fgetc(file); // data length
				bootloader_snprintf((char*)buf, 16, ":%.2X%.4X00", dataLength, commandLength);
				serialPortWrite(s, buf, 9);

				// write the data - we use the fact that the checksum byte is after the data bytes to loop it in with this loop
				while (dataLength-- > -1)
				{
					c = fgetc(file);
					buf[0] = hexLookupTable[(c >> 4) & 0x0F];
					buf[1] = hexLookupTable[(c & 0x0F)];
					serialPortWrite(s, buf, 2);
				}

				serialPortWaitFor(s, (unsigned char*)".\r\n", 3);

				if (uploadProgress != NULL)
				{
					percent = (float)ftell(file) / (float)fileSize;
					uploadProgress(obj, percent);
				}
			}
		}
		else if (commandType == 1)
		{
			// raw command, send it straight to the serial port as is
			while (commandCount-- > 0)
			{
				commandLength = fgetc(file);
				c = fgetc(file);

				// handshake char, ignore
				if (commandLength == 1 && c == 'U')
				{
					// read sleep interval and timeout interval, ignored
					c = fgetc(file);
					c = fgetc(file);
					continue;
				}

				// write command to serial port
				serialPortWrite(s, &c, 1);
				while (--commandLength > 0)
				{
					c = fgetc(file);
					serialPortWrite(s, &c, 1);
				}

				// read sleep interval and sleep
				c = fgetc(file);
				commandLength = fgetc(file) * 250;
				if (commandLength != 0)
				{
					serialPortSleep(s, commandLength);
				}

				// read timeout interval
				commandLength = fgetc(file) * 250;
				if (commandLength != 0)
				{
					unsigned char waitFor[4];
					bootloader_snprintf((char*)waitFor, 4, "%c\r\n", c);
					serialPortWaitForTimeout(s, waitFor, 3, commandLength);
				}
			}

			if (uploadProgress != NULL)
			{
				percent = (float)ftell(file) / (float)fileSize;
				uploadProgress(obj, percent);
			}
		}
		else
		{
			bootloader_perror(s, "Invalid data in .bin file");
			return 0;
		}
	}

	if (uploadProgress != NULL && percent != 1.0f)
	{
		uploadProgress(obj, 1.0f);
	}

	// re-purpose this variable for the return code
	dataLength = 1;

	if (g_enableBootloaderVerify && verifyProgress != NULL)
	{
		dataLength = bootloaderVerify(s, lastPage, verifyCheckSum, obj, verifyProgress, &state, verifyFileName);
	}

	if (dataLength == 1)
	{
		// send the "reboot to program mode" command and the device should start in program mode
		serialPortWrite(s, (unsigned char*)":020000040300F7", 15);
	}

	serialPortSleep(s, 250);

	return dataLength;
}

static int bootloaderSync(serial_port_t* s)
{
	static const unsigned char handshakerChar = 'U';

	// reboot the device in case it is stuck
	serialPortWrite(s, (unsigned char*)":020000040500F5", 15);

	// give the device time to start up
	serialPortSleep(s, BOOTLOADER_REFRESH_DELAY);

	// write a 'U' to handshake with the boot loader - once we get a 'U' back we are ready to go
	for (int i = 0; i < BOOTLOADER_RETRIES; i++)
	{
		if (serialPortWriteAndWaitForTimeout(s, &handshakerChar, 1, &handshakerChar, 1, BOOTLOADER_RESPONSE_DELAY))
			return 1;
	}

	serialPortSleep(s, BOOTLOADER_REFRESH_DELAY);

	return 0;
}

static int bootloaderHandshake(serial_port_t* s)
{
	static int baud = BAUD_RATE_BOOTLOADER;

#ifdef ENABLE_BOOTLOADER_BAUD_DETECTION
	// try handshaking at each baud rate at least twice
	for (int i = 0; i < 4; i++)
#endif // ENABLE_BOOTLOADER_DETECTION
	{
		if (bootloaderSync(s))
		{
			return 1;
		}

#ifdef ENABLE_BOOTLOADER_BAUD_DETECTION
		// retry at other baud rate
		serialPortClose(s);
		baud = (baud == BAUD_RATE_BOOTLOADER) ? BAUD_RATE_BOOTLOADER_RS232 : BAUD_RATE_BOOTLOADER;
		serialPortOpen(s, s->port, baud, 1);
#endif // ENABLE_BOOTLOADER_DETECTION
	}

	// well we've tried at 460800 and again at 2000000, time to give up
	bootloader_perror(s, "Unable to handshake with bootloader");
	return 0;
}

static int bootloadFileInternal(FILE* file, const char* fileName, serial_port_t* s, const void* obj, pfnBootloadProgress uploadProgress, pfnBootloadProgress verifyProgress, const char* verifyFileName)
{
	// flush device input buffer with invalid chars to reset state machine
	for (int i = 0; i < 64; i++)
	{
		serialPortWrite(s, (unsigned char*)"!!!!!!!!", 8);
		serialPortSleep(s, 1);
	}

	if (!bootloaderHandshake(s))
	{
		return 0;
	}

	int fileNameLength = (int)strnlen(fileName, 255);
	if (fileNameLength > 4 && strncmp(fileName + fileNameLength - 4, ".bin", 4) == 0)
	{
		// it's a .bin file, we will use a far more optimized and memory efficient uploader
		return bootloaderProcessBinFile(file, s, obj, uploadProgress, verifyProgress, verifyFileName);
	}
	else
	{

#if ENABLE_HEX_BOOT_LOADER

		bootloader_state_t state = { 0 };

		if
		(
			// negotiate version
			!bootloaderNegotiateVersion(s, &state) ||
			// erase all flash
			!bootloaderEraseFlash(s) ||
			// select the first page
			!bootloaderSelectPage(s, 0) ||
			// begin programming the first page
			!bootloaderBeginProgramForCurrentPage(s, state.firstPageSkipBytes, PAGE_SIZE - 1)
		)
		{
			return 0;
		}

		return bootloaderProcessHexFile(file, s, obj, uploadProgress, verifyProgress, &state, verifyFileName);

#else

		bootloader_snprintf(lastError, ERROR_BUFFER_SIZE, "Hex bootloader is disabled");
		return 0;

#endif

	}
}

int bootloadFile(const char* fileName, serial_port_t* port, char* error, int errorLength, const void* obj, pfnBootloadProgress uploadProgress, pfnBootloadProgress verifyProgress)
{
	bootload_params_t params = { 0 };
	params.fileName = fileName;
	params.port = port;
	params.error = error;
	params.errorLength = errorLength;
	params.obj = obj;
	params.uploadProgress = uploadProgress;
	params.verifyProgress = verifyProgress;

	return bootloadFileEx(&params);
}

int bootloadFileEx(bootload_params_t* params)
{
	int result = 0;

	// open the serial port
	if (serialPortOpenInternal(params->port, BAUD_RATE_BOOTLOADER, params->error, params->errorLength) == 0)
	{
		return result;
	}

	// open the file
	FILE* file = 0;
	
#ifdef _MSC_VER

	fopen_s(&file, params->fileName, "rb");
	
#else
	
	file = fopen(params->fileName, "rb");

#endif

	if (file == 0)
	{
		if (params->error != 0)
		{
			bootloader_snprintf(params->error, params->errorLength, "Unable to open file: %s", params->fileName);
		}
		return result;
	}

	result = bootloadFileInternal(file, params->fileName, params->port, params->obj, params->uploadProgress, params->verifyProgress, params->verifyFileName);
	if (result == 0)
	{
		// reboot the device back into boot-loader mode
		serialPortWrite(params->port, (unsigned char*)":020000040500F5", 15);
		serialPortSleep(params->port, 500);
	}

	fclose(file);
	serialPortClose(params->port);

	return result;
}

int enableBootloader(serial_port_t* port, char* error, int errorLength)
{
	// open the serial port
	if (serialPortOpenInternal(port, BAUD_RATE_STANDARD, error, errorLength) == 0)
	{
		return 0;
	}
	
	// enable the boot loader
	serialPortWrite(port, (unsigned char*)"$BLEN*05\r\n", 10);
	serialPortSleep(port, 250);
	serialPortClose(port);

	return 1;
}

static int disableBootloaderInternal(serial_port_t* port, char* error, int errorLength, int baud)
{
	// open the serial port
	if (serialPortOpenInternal(port, baud, error, errorLength) == 0)
	{
		return 0;
	}

	// send the "reboot to program mode" command and the device should start in program mode
	serialPortWrite(port, (unsigned char*)":020000040300F7", 15);
	serialPortSleep(port, 250);
	serialPortClose(port);
	return 1;
}


int disableBootloader(serial_port_t* port, char* error, int errorLength)
{
	int result = 0;
	result |= disableBootloaderInternal(port, error, errorLength, BAUD_RATE_BOOTLOADER);
	result |= disableBootloaderInternal(port, error, errorLength, BAUD_RATE_BOOTLOADER_RS232);
	return result;
}
