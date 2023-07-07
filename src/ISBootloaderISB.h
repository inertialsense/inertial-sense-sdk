/**
 * @file ISBootloaderISB.h
 * @author Dave Cutting (davidcutting42@gmail.com)
 * @brief Inertial Sense routines for updating application images using the ISB
 *  (Inertial Sense Bootloader) protocol.
 * 
 */

/*
MIT LICENSE

Copyright (c) 2014-2023 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#ifndef __IS_BOOTLOADER_ISB_H
#define __IS_BOOTLOADER_ISB_H

#include "ISBootloaderBase.h"

#include <mutex>

class cISBootloaderISB : public ISBootloader::cISBootloaderBase
{
public:
    cISBootloaderISB(
        ISBootloader::pfnBootloadProgress upload_cb,
        ISBootloader::pfnBootloadProgress verify_cb,
        ISBootloader::pfnBootloadStatus info_cb,
        serial_port_t* port
    ) : cISBootloaderBase{ upload_cb, verify_cb, info_cb } 
    {
        m_port = port;
        m_device_type = ISBootloader::IS_DEV_TYPE_ISB;
    }
    
    ~cISBootloaderISB() 
    {
        
    }

    is_operation_result match_test(void* param);
    
    is_operation_result reboot();
    is_operation_result reboot_force();
    is_operation_result reboot_up();
    is_operation_result reboot_down(uint8_t major = 0, char minor = 0, bool force = false);

    uint32_t get_device_info();

    ISBootloader::eImageSignature check_is_compatible();
    
    is_operation_result download_image(std::string image);
    is_operation_result upload_image(std::string image) { return IS_OP_OK; }
    is_operation_result verify_image(std::string image);
    
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
    static is_operation_result get_version_from_file(const char* filename, uint8_t* major, char* minor);

    static is_operation_result sync(serial_port_t* s);

    static void reset_serial_list() { serial_list_mutex.lock(); serial_list.clear(); serial_list_mutex.unlock(); }

private:
    
    /**
     * @brief Calculate checksum for ISB
     * 
     * @param checkSum 
     * @param ptr 
     * @param start INCLUSIVE
     * @param end EXCLUSIVE
     * @param checkSumPosition if not 0, the checkSum is written to ptr + checkSumPosition
     * @param finalCheckSum 
     * @return int 
     */
    int checksum(int checkSum, uint8_t* ptr, int start, int end, int checkSumPosition, int finalCheckSum);

    is_operation_result erase_flash();
    is_operation_result select_page(int page);
    is_operation_result begin_program_for_current_page(int startOffset, int endOffset);
    
    int is_isb_read_line(FILE* file, char line[1024]);

    is_operation_result upload_hex_page(unsigned char* hexData, int byteCount, int* currentOffset, int* totalBytes, int* verifyCheckSum);
    is_operation_result upload_hex(unsigned char* hexData, int charCount, int* currentOffset, int* currentPage, int* totalBytes, int* verifyCheckSum);
    is_operation_result fill_current_page(int* currentPage, int* currentOffset, int* totalBytes, int* verifyCheckSum);
    is_operation_result download_data(int startOffset, int endOffset);

    // Verification parameters
    int m_currentPage;
    int m_verifyCheckSum;

    is_operation_result process_hex_file(FILE* file);

    struct {
        bool is_evb;                    // Available on version 6+, otherwise false
        ISBootloader::eProcessorType processor;       // Differentiates between uINS-3 and IMX-5
        bool rom_available;             // ROM bootloader is available on this port
        
        uint32_t app_offset;            // Helps in loading bin files
        uint32_t verify_size;           // Chunk size, limited on Windows
    } m_isb_props;

    static std::vector<uint32_t> serial_list;
    static std::mutex serial_list_mutex;

    static std::vector<uint32_t> rst_serial_list;
    static std::mutex rst_serial_list_mutex;
};

#endif	// __IS_BOOTLOADER_ISB_H
