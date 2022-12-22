/**
 * @file ihex.h
 * @author Dave Cutting (davidcutting42@gmail.com)
 * @brief Intel HEX file read routines for embedded firmware
 * 
 */

/*
MIT LICENSE

Copyright (c) 2014-2023 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#ifndef __IHEX_H_
#define __IHEX_H_

#include <stdint.h>
#include <stdio.h>

#ifdef __cplusplus
extern "C" { 
#endif

#define MAX_NUM_IHEX_SECTIONS 	1024		
#define MAX_IHEX_SECTION_LEN 	0x020000	// 128K TODO: Reduce to 64k if possible

typedef struct
{
    /* Address the image section needs to be programmed to */
    uint32_t address;

    /* Pointer to the section data */
    uint8_t* image;

    /* Length of this section*/
    uint32_t len;
} ihex_image_section_t;

/**
 * @brief Load an intel hex file into a struct representing the full image
 * 
 * @param ihex_filename a string representing the filename, including path, where the hex file resides at. 
 * @param image array of ihex_image_section_t that will be filled with the image
 * @param num_slots the maximum number of sections to read from the file
 * @return int the number of sections actually read
 */
size_t ihex_load_sections(const char* ihex_filename, ihex_image_section_t* image, size_t num_slots);

/**
 * @brief Free the memory associated with an image
 * 
 * @param image the struct containing the full image 
 * @param num number of populated sections in the image. Obtained from return value of ihex_load_sections
 */
void ihex_unload_sections(ihex_image_section_t* image, size_t num);

#ifdef __cplusplus
}
#endif

#endif  // __IHEX_H_
