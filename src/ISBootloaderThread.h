/**
 * @file ISBootloaderThread.h
 * @author Dave Cutting
 * @brief Inertial Sense routines for updating embedded systems in parallel
 * 
 */

/*
MIT LICENSE

Copyright (c) 2014-2025 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#ifndef __IS_BOOTLOADER_THREAD_H_
#define __IS_BOOTLOADER_THREAD_H_

#include <stdio.h>
#include <stdlib.h>
#include <stddef.h>
#include <string>
#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>
#include <mutex>

#include "ISUtilities.h"
#include "ISBootloaderBase.h"

class cISBootloaderThread
{
public:
    cISBootloaderThread() {};
    ~cISBootloaderThread() {};

    typedef struct 
    {
        uint32_t sn;
        uint8_t major;
        char minor;
    } confirm_bootload_t;

    static bool set_mode_and_check_devices(
        std::vector<std::string>&               comPorts,
        int                                     baudRate,
        const ISBootloader::firmwares_t&        firmware,
        ISBootloader::pfnBootloadProgress       uploadProgress, 
        ISBootloader::pfnBootloadProgress       verifyProgress,
        ISBootloader::pfnBootloadStatus         infoProgress,
        void						            (*waitAction)() = NULL,
        std::vector<confirm_bootload_t>*        updatesPending = NULL
    );

    static is_operation_result update(
        std::vector<std::string>&               comPorts,
        bool                                    force_isb_update,
        int                                     baudRate,
        const ISBootloader::firmwares_t&        firmware,
        ISBootloader::pfnBootloadProgress       uploadProgress, 
        ISBootloader::pfnBootloadProgress       verifyProgress,
        ISBootloader::pfnBootloadStatus         infoProgress,
        void						            (*waitAction)()
    );

    static void cancel_update();

    typedef struct 
    {
        void* thread;
        char serial_name[100];
        ISBootloader::cISBootloaderBase* ctx;
        bool done;
        bool reuse_port;
        bool force_isb;
        is_operation_result result;      // defaults to IS_OP_OK
    } thread_serial_t;

    typedef struct 
    {
        void* thread;
        libusb_device_handle* handle;
        char uid[100];
        ISBootloader::cISBootloaderBase* ctx;
        bool done;
        is_operation_result result;      // defaults to IS_OP_OK
    } thread_libusb_t;

    static std::vector<ISBootloader::cISBootloaderBase*> ctx;
    static std::mutex m_ctx_mutex;

    static bool m_update_in_progress;
    
private:
    static void create_and_start_serial_thread(const std::string& port, void(*function)(void*), bool force_isb_update = false);
    static void create_and_start_libusb_thread(void(*function)(void*), libusb_device_handle* handle);
    static void get_device_isb_version_thread(void* context);
    static void mode_thread_serial_app(void* context);
    static void mode_thread_serial_isb(void* context);
    static void update_thread_serial(void* context);
    static void update_thread_libusb(void* context);
    static void mgmt_thread_libusb(void* context);
    static bool true_if_cancelled(void);

    static ISBootloader::firmwares_t m_firmware;
    
    static int m_baudRate;

    static std::mutex m_update_mutex;
    
    static ISBootloader::pfnBootloadProgress m_uploadProgress; 
    static ISBootloader::pfnBootloadProgress m_verifyProgress;
    static ISBootloader::pfnBootloadStatus m_infoProgress;
    static void (*m_waitAction)();

    static uint32_t m_timeStart;
    static bool m_use_dfu;
    static uint32_t m_libusb_devicesActive;
    static uint32_t m_serial_devicesActive;

    static bool m_continue_update;

    static std::vector<thread_serial_t*> m_serial_threads;    // List of all serial threads that have run or are running
    static std::vector<thread_libusb_t*> m_libusb_threads;    // List of all libusb threads that have run or are running
    static std::mutex m_serial_thread_mutex;
    static std::mutex m_libusb_thread_mutex;
};

#endif // __IS_BOOTLOADER_THREAD_H_
