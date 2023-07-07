/**
 * @file ISBootloaderDFU.h
 * @author Dave Cutting (davidcutting42@gmail.com)
 * @brief Inertial Sense routines for updating ISB (Inertial Sense Bootloader)
 *  images using the DFU protocol.
 * 
 */

/*
MIT LICENSE

Copyright (c) 2014-2023 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#ifndef __IS_BOOTLOADER_DFU_H
#define __IS_BOOTLOADER_DFU_H

#include "ISBootloaderBase.h"
#include "libusb.h"

#include <mutex>

namespace ISBootloader {

static constexpr int IS_DFU_UID_MAX_SIZE = 20;
static constexpr int IS_DFU_LIST_LEN = 256;
    
enum
{
    STM32_DESCRIPTOR_VENDOR_ID = 0x0483,
    STM32_DESCRIPTOR_PRODUCT_ID = 0xdf11
};

typedef struct 
{
    // Recipe for DFU UID number:
    // sprintf(ctx->match_props.uid, "%X%X", manufacturing_info->uid[0] + manufacturing_info->uid[2], (uint16_t)(manufacturing_info->uid[1] >> 16));
    char uid[IS_DFU_UID_MAX_SIZE];      // DFU device serial number, from descriptors
    uint32_t sn;                        // Inertial Sense serial number
    uint16_t vid;
    uint16_t pid;
    libusb_device_handle* handle_libusb;
    uint8_t iSerialNumber;
} is_dfu_id;

typedef struct 
{
    is_dfu_id id[IS_DFU_LIST_LEN];
    size_t present;
} is_dfu_list;

class cISBootloaderDFU : public ISBootloader::cISBootloaderBase
{
public:
    cISBootloaderDFU(
        pfnBootloadProgress upload_cb,
        pfnBootloadProgress verify_cb,
        pfnBootloadStatus info_cb,
        libusb_device_handle* handle
    ) : cISBootloaderBase{ upload_cb, verify_cb, info_cb } 
    {
        m_dfu.handle_libusb = handle;
        m_device_type = IS_DEV_TYPE_DFU;
    }

    ~cISBootloaderDFU() 
    {
        // TODO: Close DFU device?
    }
    
    is_operation_result reboot();
    is_operation_result reboot_up();
    is_operation_result reboot_down(uint8_t major = 0, char minor = 0, bool force = false) { (void)major; (void)minor; (void)force; return IS_OP_OK; }

    is_operation_result match_test(void* param);

    uint32_t get_device_info();

    ISBootloader::eImageSignature check_is_compatible();
    
    is_operation_result download_image(std::string image);
    is_operation_result upload_image(std::string image) { return IS_OP_OK; }
    is_operation_result verify_image(std::string image) { return IS_OP_OK; }

    static int get_num_devices();
    static is_operation_result list_devices(is_dfu_list* list);

    bool is_serial_device() { return false; }

    static std::mutex m_DFUmutex;

private:
    typedef enum	// Internal only, can change as needed
    {
        DFU_ERROR_NONE = 0,
        DFU_ERROR_NO_DEVICE = -1,
        DFU_ERROR_LIBUSB = -2,
        DFU_ERROR_STATUS = -3,
        DFU_ERROR_INVALID_ARG = -4,
        DFU_ERROR_NO_FILE = -5,
        DFU_ERROR_TIMEOUT = -6,
    } dfu_error;

    typedef enum	// From DFU manual, do not change
    {
        DFU_STATUS_OK = 0,
        DFU_STATUS_ERR_TARGET,
        DFU_STATUS_ERR_FILE,
        DFU_STATUS_ERR_WRITE,
        DFU_STATUS_ERR_ERASED,
        DFU_STATUS_ERR_CHECK_ERASED,
        DFU_STATUS_ERR_PROG,
        DFU_STATUS_ERR_VERIFY,
        DFU_STATUS_ERR_ADDRESS,
        DFU_STATUS_ERR_NOTDONE,
        DFU_STATUS_ERR_FIRMWARE,
        DFU_STATUS_ERR_VENDOR,
        DFU_STATUS_ERR_USBR,
        DFU_STATUS_ERR_POR,
        DFU_STATUS_ERR_UNKNOWN,
        DFU_STATUS_ERR_STALLEDPKT,
        
        DFU_STATUS_NUM,
    } dfu_status;

    typedef enum	// From DFU manual, do not change
    {
        DFU_STATE_APP_IDLE = 0,
        DFU_STATE_APP_DETACH,
        DFU_STATE_IDLE,
        DFU_STATE_DNLOAD_SYNC,
        DFU_STATE_DNBUSY, 
        DFU_STATE_DNLOAD_IDLE,
        DFU_STATE_MANIFEST_SYNC,
        DFU_STATE_MANIFEST,
        DFU_STATE_MANIFEST_WAIT_RESET,
        DFU_STATE_UPLOAD_IDLE,
        DFU_STATE_ERROR,

        DFU_STATE_NUM,
    } dfu_state;

    typedef enum
    {
        STM32_DFU_INTERFACE_FLASH    = 0, // @Internal Flash  /0x08000000/0256*0002Kg
        STM32_DFU_INTERFACE_OPTIONS  = 1, // @Option Bytes  /0x1FFF7800/01*040 e
        STM32_DFU_INTERFACE_OTP      = 2, // @OTP Memory /0x1FFF7000/01*0001Ke
        STM32_DFU_INTERFACE_FEATURES = 3  // @Device Feature/0xFFFF0000/01*004 e
    } is_dfu_interface_alternatives;

    static int dfu_DETACH(libusb_device_handle** handle, uint8_t timeout);
    static int dfu_DNLOAD(libusb_device_handle** handle, uint8_t wValue, uint8_t* buf, uint16_t len);
    static int dfu_UPLOAD(libusb_device_handle** handle, uint8_t wValue, uint8_t* buf, uint16_t len);
    static int dfu_GETSTATUS(libusb_device_handle** handle, dfu_status* status, uint32_t *delay, dfu_state* state, uint8_t *i_string);
    static int dfu_CLRSTATUS(libusb_device_handle** handle);
    static int dfu_GETSTATE(libusb_device_handle** handle, uint8_t* buf);
    static int dfu_ABORT(libusb_device_handle** handle);
    
    static is_operation_result get_serial_number_libusb(libusb_device_handle** handle, uint32_t& sn, std::string& uid, uint8_t sn_idx); 

    static dfu_error dfu_set_address_pointer(libusb_device_handle** dev_handle, uint32_t address);
    static dfu_error dfu_wait_for_state(libusb_device_handle** dev_handle, dfu_state required_state);

    is_dfu_id m_dfu;
};

}

#endif	// __IS_BOOTLOADER_DFU_H
