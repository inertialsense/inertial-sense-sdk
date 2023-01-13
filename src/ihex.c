/**
 * @file ihex.c
 * @author Dave Cutting (davidcutting42@gmail.com)
 * @author Paul Stoffregen (paul@ece.orst.edu)
 * @brief Intel HEX file read routines for embedded firmware
 * @note based on https://www.pjrc.com/tech/8051/ihex.c
 * 
 */

/*
MIT LICENSE

Copyright (c) 2014-2023 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/
 
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "ihex.h"

static int parse_hex_line(char* theline, uint8_t bytes[], int* addr, int* num, int* code)
{
    int sum, len, cksum;
    char *ptr;
    
    *num = 0;
    if (theline[0] != ':') return 0;
    if (strlen(theline) < 11) return 0;		
    ptr = theline+1;
    if (!sscanf(ptr, "%02x", &len)) return 0;
    ptr += 2;
    if ( strlen(theline) < (11 + ((size_t)len * 2)) ) return 0;
    if (!sscanf(ptr, "%04x", addr)) return 0;
    ptr += 4;
      /* printf("Line: length=%d Addr=%d\n", len, *addr); */
    if (!sscanf(ptr, "%02x", code)) return 0;
    ptr += 2;
    sum = (len & 255) + ((*addr >> 8) & 255) + (*addr & 255) + (*code & 255);
    while(*num != len) 
    {
        if (!sscanf(ptr, "%02hhx", &bytes[*num])) return 0;
        ptr += 2;
        sum += bytes[*num] & 0xFF;
        (*num)++;
        if (*num >= 256) return 0;
    }
    if (!sscanf(ptr, "%02x", &cksum)) return 0;
    if ( ((sum & 255) + (cksum & 255)) & 255 ) return 0; /* checksum error */
    return 1;
}

static int ihex_load_section(FILE** ihex_file, ihex_image_section_t* section)
{
    char line[512];	// Max line length is 256
    int addr, n, status;
    uint8_t bytes[256];	
    int i, lineno=1;
    int minaddr=65536, maxaddr=0;
    bool alreadysetaddr = false;
    bool eof = false;
    long last_line;
    uint32_t address = 0;

    uint8_t* image_local = malloc(MAX_IHEX_SECTION_LEN);
    memset(image_local, 0xFF, MAX_IHEX_SECTION_LEN);		// Any unspecified bytes default to 0xFF

    while (!feof(*ihex_file) && !ferror(*ihex_file)) {

        // Open file and get a line
        line[0] = '\0';

        last_line = ftell(*ihex_file);
        if (fgets(line, 512, *ihex_file)){}

        // Turn end of line characters into cstring terminators
        if (line[strlen(line) - 1] == '\n') line[strlen(line) - 1] = '\0';
        if (line[strlen(line) - 1] == '\r') line[strlen(line) - 1] = '\0';

        // Parse the line
        if (parse_hex_line(line, bytes, &addr, &n, &status))
        {
            if (status == 0)	// Copy data into memory 
            {
                for (i = 0; i < n; i++)
                {
                    image_local[addr] = bytes[i];
                    if (addr < minaddr) minaddr = addr;
                    if (addr > maxaddr) maxaddr = addr;
                    addr++;

                    if (addr > MAX_IHEX_SECTION_LEN) return -1;	// Buffer exceeded
                }
            }
            else if (status == 1)	// End of file 
            {
                eof = true;
                break;
            }
            else if (status == 4)	// Programming location. *should* also work if there are no code == 4 in the hex file
            {
                if (alreadysetaddr)
                {
                    // Go back one line, so the next call to this function gets the address to write to
                    fseek(*ihex_file, last_line, SEEK_SET);

                    break;	// Not finished with file, but reached end of sector	
                }

                address = bytes[1] << 16 | bytes[0] << 24;
                alreadysetaddr = true;
            }
        }
        else
        {
            printf("   Error line: %d\n", lineno);
        }

        lineno++;
    }
    
    // Base the section length on what address the end is at, since hex file may skip over unused bytes in some lines
    section->len = maxaddr - minaddr + 1;

    // Allocate a buffer for the image to reside in
    section->image = malloc(section->len);

    if (section->image != NULL && section->len)
    {
        section->address = address;

        // printf("   Loaded %d bytes between:", section->len);
        // printf(" 0x%04X to 0x%04X at address:", minaddr, maxaddr);
        // printf(" 0x%08X\n", section->address);

        memcpy(section->image, &image_local[minaddr], section->len);

        if(image_local) free(image_local);
        
        return eof ? 1 : 0; // Return 1 if end of file reached
    }

    if(image_local) free(image_local);
    
    return -1;	// Malloc failed or nothing in sector
}

static void ihex_unload_section(ihex_image_section_t* section)
{
    if (section->image != NULL)
    {
        free(section->image);
        section->image = NULL;
        section->address = 0;
        section->len = 0;
    }
}

size_t ihex_load_sections(const char* ihex_filename, ihex_image_section_t* image, size_t num_slots)
{
    FILE* ihex_file;
    ihex_file = fopen(ihex_filename, "r");
    if(ihex_file==NULL) return 0;

    size_t iter = 0;
    size_t numSections = 0;

    // printf("Loading sections from %s\n", ihex_filename);

    // Find sections
    do {
        int ret = ihex_load_section(&ihex_file, &image[iter]);
        
        if (ret < 0) break;	// Error
        numSections++;
        if (ret == 1) break;	    // Last sector in file found (EOF)
    } while (++iter < num_slots);

    // ACT ON SECTIONS HERE
    fclose(ihex_file);

    // printf("\n");

    return numSections;
}

void ihex_unload_sections(ihex_image_section_t* section, size_t num)
{
    // Free section memory
    for (size_t i = 0; i < num; i++)
    {
        ihex_unload_section(&section[i]);
    }
}
