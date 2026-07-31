/**
 * @file types.h
 * @brief Foundational enums and log-facility bitmask constants shared across the SDK core: the
 *        IS_LOG_ facility bits consumed by msg_logger.h, the eLogLevel severity enum, and the
 *        eBootLoaderType bootloader-mode enum.
 *
 * @author Kyle Mallory on 7/3/24.
 * @copyright Copyright (c) 2024 Inertial Sense, Inc. All rights reserved.
 */

#ifndef IS_CORE_TYPES_H
#define IS_CORE_TYPES_H

#include <inttypes.h>

// Define facility bitmasks. Use powers of 2 for unique bits.
#define IS_LOG_FACILITY_NONE       0x0000                          //!< this facility is ALWAYS enabled (but still follows IS_LOG_LEVEL)
#define IS_LOG_PORT                0x0001                          //!< generic port I/O (base_port_t) logging
#define IS_LOG_ISCOMM              ((IS_LOG_PORT << 1))            //!< ISComm protocol parsing/framing logging
#define IS_LOG_FWUPDATE            ((IS_LOG_ISCOMM << 1))          //!< firmware update logging
#define IS_LOG_ISDEVICE            ((IS_LOG_FWUPDATE << 1))        //!< ISDevice logging
#define IS_LOG_PORT_FACTORY        ((IS_LOG_ISDEVICE << 1))        //!< port factory/discovery logging
#define IS_LOG_PORT_MANAGER        ((IS_LOG_PORT_FACTORY << 1))    //!< port manager logging
#define IS_LOG_DEVICE_FACTORY      ((IS_LOG_PORT_MANAGER << 1))    //!< device factory logging
#define IS_LOG_DEVICE_MANAGER      ((IS_LOG_DEVICE_FACTORY << 1))  //!< device manager logging
#define IS_LOG_CHRONO_STATS        ((IS_LOG_DEVICE_MANAGER << 1))  //!< timing/chrono statistics logging
#define IS_LOG_FN_PROFILER         ((IS_LOG_CHRONO_STATS << 1))    //!< FnProfiler function-timing logging
#define IS_LOG_MDNS_CACHE          ((IS_LOG_FN_PROFILER << 1))     //!< mDNS cache logging
#define IS_LOG_HTTP_REQUEST        ((IS_LOG_MDNS_CACHE << 1))      //!< HTTP request logging
#define IS_LOG_CORRECTIONS         ((IS_LOG_HTTP_REQUEST << 1))    //!< GNSS corrections logging
#define IS_LOG_CALIBRATION         ((IS_LOG_CORRECTIONS << 1))     //!< calibration logging
#define IS_LOG_ISLOG               ((IS_LOG_CALIBRATION << 1))     //!< ISLog / ISDeviceLog / ISLogReader file parse + load logging
// APP-specific stuff here goes here
#define IS_LOG_APP_EVALTOOL        ((IS_LOG_ISLOG << 1))           //!< EvalTool application logging
#define IS_LOG_APP_CLTOOL          ((IS_LOG_APP_EVALTOOL << 1))    //!< cltool application logging
#define IS_LOG_APP_LOGALYZER       ((IS_LOG_APP_CLTOOL << 1))      //!< Logalyzer application logging

#define IS_LOG_FACILITY_ALL        0xFFFF     //!< bitmask enabling every defined logging facility

/** Severity levels for the SDK's logging macros (see msg_logger.h); higher values are more verbose. */
typedef enum {
    IS_LOG_LEVEL_NONE  = 0,         //!< use this to disable all log messages
    IS_LOG_LEVEL_ERROR = 1,         //!< errors - that that prevents the application from proceeding as normal
    IS_LOG_LEVEL_WARN  = 2,         //!< warnings - important to now, but we can move on anyway
    IS_LOG_LEVEL_INFO  = 3,         //!< informative for the customer regarding normal operations
    IS_LOG_LEVEL_MORE_INFO = 4,     //!< additional information for the customer regarding normal operations - might be annoying
    IS_LOG_LEVEL_DEBUG = 5,         //!< informative for support & basic troubleshooting
    IS_LOG_LEVEL_MORE_DEBUG = 6,    //!< informative for advanced troubleshooting - maybe annoying
    IS_LOG_LEVEL_BOMBASTIC = 7      //!< excessive on all but the most extreme cases - will definitely be annoying
} eLogLevel;

/** Identifies which bootloader protocol/mode a device is running, or should be flashed with. */
typedef enum {
    IS_BL_TYPE_NONE = 0,    //!< no bootloader / not applicable
    IS_BL_TYPE_SAMBA,       //!< Atmel/Microchip SAM-BA bootloader
    IS_BL_TYPE_ISB,         //!< Inertial Sense proprietary ISB bootloader
    IS_BL_TYPE_APP,         //!< not a bootloader; the device is running its application firmware
    IS_BL_TYPE_DFU,         //!< USB DFU (Device Firmware Upgrade) bootloader
} eBootLoaderType;

#endif //IS_CORE_TYPES_H
