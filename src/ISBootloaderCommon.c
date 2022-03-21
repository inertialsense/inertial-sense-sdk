/**
 * @file ISBootloaderCommon.c
 * @author Dave Cutting (davidcutting42@gmail.com)
 * @brief Inertial Sense routines for updating embedded systems
 * 
 */

/*
MIT LICENSE

Copyright (c) 2014-2021 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include "ISBootloaderCommon.h"

#include "serialPort.h"
#include "serialPortPlatform.h"
#include "inertialSenseBootLoader.h"
#include "ISBootloaderDfu.h"
#include "ISBootloaderSamba.h"

is_operation_result is_destroy_device_context(is_device_context* ctx)
{
    free(ctx);

    return IS_OP_OK;
}

int is_get_handles(is_device_context** ctx, int max_num_handles)
{
    libusb_device** device_list;
    libusb_device* dev;
    libusb_device_handle* dev_handle;
    struct libusb_device_descriptor* desc;
    struct libusb_config_descriptor* cfg;
    int ret_libusb;
    int match_count = 0;

    if(max_num_handles <= 0) return 0;

    ret_libusb = libusb_init(NULL);
    if(ret_libusb < 0) return -1;

    int device_count = libusb_get_device_list(NULL, &device_list);

    for (size_t i = 0; i < device_count; ++i) {
        dev = device_list[i];

        ret_libusb = libusb_get_device_descriptor(dev, &desc);
        if(ret_libusb < 0) continue;

        if ((desc->idVendor != ctx[match_count]->match_props.vid) && (ctx[match_count]->match_props.match & IS_DEVICE_MATCH_FLAG_VID))
            continue;   // must be some other usb device

        if ((desc->idProduct != ctx[match_count]->match_props.pid) && (ctx[match_count]->match_props.match & IS_DEVICE_MATCH_FLAG_PID))
            continue;   // must be some other usb device

        ret_libusb = libusb_open(dev, &dev_handle);
        if (ret_libusb < 0)
        {
            libusb_close(dev_handle);
            continue;
        }

        unsigned char serial_number[IS_SN_MAX_SIZE];

        // Get the string containing the serial number from the device
        ret_libusb = libusb_get_string_descriptor_ascii(dev_handle, desc->iSerialNumber, serial_number, sizeof(serial_number));
        if (ret_libusb < LIBUSB_SUCCESS) 
        {   // Set the serial number as none
            serial_number[0] = '\0';
        }

        bool sn_matches = !(ctx[match_count]->match_props.match & IS_DEVICE_MATCH_FLAG_SN) || 
            (strncmp(serial_number, ctx[match_count]->match_props.serial_number, IS_SN_MAX_SIZE) == 0) ||
            ctx[match_count]->match_props.serial_number[0] == '\0';

        if(cfg->interface->altsetting[0].bInterfaceClass == 0xFE && 
            cfg->interface->altsetting[0].bInterfaceSubClass == 0x01 && 
            cfg->interface->altsetting[0].bInterfaceProtocol == 0x02 &&
            sn_matches 
        )
        {   // Add 
            ctx[match_count++]->handle.handle.libusb = dev_handle;

            if(match_count >= max_num_handles)
            {
                libusb_close(dev_handle);
                libusb_free_device_list(device_list, 1);
                return match_count;
            }
        }
        else
        {   // Not a DFU device, or serial number doesn't match
            libusb_close(dev_handle);
            continue;
        }
    }

    libusb_free_device_list(device_list, 1);

    return match_count;
}

is_operation_result is_release_handles(
    libusb_device** device_list, 
    libusb_device_handle** match_list,
    size_t match_count
)
{
    for (size_t i = 0; i < match_count; ++i) {
        libusb_close(match_list[i]);
    }

    libusb_exit(NULL);

    return IS_OP_OK;
}

void is_update_flash(void* context)
{
    is_device_context* ctx = (is_device_context*)context;
    int ret = IS_OP_ERROR;
    
    if(!ctx->firmware_file_path) return ret;  // No firmware present

    if(ctx->scheme == IS_SCHEME_DFU)
    {
        int ihex_ret;

        // Load the firmware into memory. Even if we don't use it now (SAM-BA), this is a good check
        ihex_image_section_t image[MAX_NUM_IHEX_SECTIONS];
        ihex_ret = ihex_load_sections(ctx->firmware_file_path, image, MAX_NUM_IHEX_SECTIONS);
        if(ihex_ret <= 0) return ret;

        ret = is_dfu_flash(ctx, image, ihex_ret, (libusb_device_handle*)ctx->handle.handle.libusb);

        ihex_unload_sections(image, ihex_ret);
    }
    else if(ctx->scheme == IS_SCHEME_SAMBA)
    {
        ret = is_samba_flash(ctx);
    }

    ctx->success = (ret == IS_OP_OK ? true : false);

    return;
}