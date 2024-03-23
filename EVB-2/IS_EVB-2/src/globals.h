/*
MIT LICENSE

Copyright 2014-2019 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT, IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#ifndef __GLOBALS_H_
#define __GLOBALS_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <asf.h>
#include "ISComm.h"
#include "data_sets.h"
#include "ISConstants.h"
#include "bootloaderShared.h"
#include "rtos.h"
#include "d_serial.h"
#include "d_time.h"
#include "conf_board.h"

#define USE_RTC_DATE_TIME       1   // Use RTC for system data and time

#define STREAM_INS_FOR_TIME_SYNC        1
#define SKI_BOX_STATUS_LED_PIN          GPIO_10_PIN
#define UBLOX_LOG_ENABLE			    0

// EVB Realtime Message Controller (ERMC) - message broadcast mechanism. 
#define ERMC_BITS_DEV_INFO              0x0000000000000001
#define ERMC_BITS_FLASH_CFG             0x0000000000000002
#define ERMC_BITS_STATUS                0x0000000000000004
#define ERMC_BITS_WHEEL_ENCODER         0x0000000000000008
#define ERMC_BITS_DEBUG_ARRAY           0x0000000000000100


typedef void (*VoidFuncPtrVoid)(void);

PUSH_PACK_1

typedef struct
{
    uint32_t year, month, day, week, hour, minute, second;
} date_time_t;

typedef struct
{
    dev_info_t              uInsInfo;
    ins_1_t                 ins1;
    ins_2_t                 ins2;
    inl2_states_t           inl2States;
    pimu_t     pImu;
    nvm_flash_cfg_t         flashCfg;
	bool					refLlaValid;
} uins_msg_t;

typedef struct PACKED      // Non-volatile memory state
{
    uint32_t                flash_write_needed;                 // 0=No write; 1=config write needed; 2=config write needed without backup 0xFFFFFFFF=reset defaults
    uint32_t                flash_write_count;                  // Number of times flash is written to since reset
    uint32_t                flash_write_enable_timeMs;          // Local time when enabled.  Flash will happen 1-2 seconds after this enable time.  Reset to zero following flash write.
} nvr_manage_t;

typedef struct PACKED      // 
{
    uint64_t                bits;
    uint32_t				options;
    uint32_t                periodMultiple[DID_COUNT];
} ermc_t;

// All Flash Parameters - config max size is 8K for ARM
typedef struct PACKED
{
    union
    {	// Standard EVB-2
	    evb_flash_cfg_t m;
	    uint32_t padding[BOOTLOADER_FLASH_BLOCK_SIZE / 2 / sizeof(uint32_t)];  // 4096 bytes
    } g0;

    union
    {	// Reserved for Luna
	    uint32_t padding[BOOTLOADER_FLASH_BLOCK_SIZE / 2 / sizeof(uint32_t)];  // 4096 bytes
    } g1;
	
} nvm_config_t;

POP_PACK

// Defined in init.c
void init_set_board_IO_config_callback(VoidFuncPtrVoid fpIoCfg);
void refresh_led_cfg(void);
void board_IO_config(void);
//

#define STREAM_BUFFER_SIZE      4096

extern uint8_t                      g_hdw_detect;
extern dev_info_t                   g_evbDevInfo;
extern wheel_encoder_t              g_wheelEncoder;
extern uint32_t                     g_wheelEncoderTimeMs;
extern evb_status_t                 g_status;
extern bool                         g_statusToWlocal;
extern evb_flash_cfg_t*             g_flashCfg;
extern nvr_manage_t                 g_nvr_manage_config;
extern nvm_config_t                 g_userPage;
extern uins_msg_t                   g_uins;
extern imu_t                        g_imu;
extern uint32_t                     g_insUpdateTimeMs;
extern uint32_t                     g_imuUpdateTimeMs;
extern debug_array_t                g_debug;
extern evb_rtos_info_t              g_rtos;
extern date_time_t                  g_gps_date_time;
//extern uint32_t					g_CANbaud_kbps;
//extern uint32_t					g_can_receive_address;
extern bool                         g_gpsTimeSync;
extern uint32_t                     g_comm_time_ms;
extern double                       g_comm_time;
extern double                       g_towOffset;
extern bool                         g_loggerEnabled;
extern uint32_t                     g_uInsBootloaderEnableTimeMs;
extern bool                         g_enRtosStats;
extern ermc_t    			        g_ermc;

void globals_init(void);
void com_bridge_apply_preset(evb_flash_cfg_t* cfg);
void reset_evb_flash_cfg_defaults(evb_flash_cfg_t* cfg);
int comWrite(int serialNum, const unsigned char *buf, int size, uint32_t ledPin );
int comRead(int serialNum, unsigned char *buf, int size, uint32_t ledPin);
void com_bridge_forward(uint32_t srcPort, uint8_t *buf, int len);
void com_bridge_smart_forward(uint32_t srcPort, uint32_t ledPin);
void concatStringWithSpace(char* buf, size_t bufLen, const char* concat);

bool nvr_validate_config_integrity(evb_flash_cfg_t* cfg);
void nvr_init(void);
bool nvr_slow_maintenance(void);
void nvr_flash_config_write_needed(void);
void nvr_flash_config_write_enable(void);

int error_check_evb_flash_cfg(evb_flash_cfg_t *cfg);

void setBuildDateTimeFromCompileTime(uint8_t buildDate[4], uint8_t buildTime[4]);

#define FLASH_WRITE_IN_PROGRESS()   (g_nvr_manage_config.flash_write_enable_timeMs)

#ifdef __cplusplus
}
#endif

#endif // __GLOBALS_H_
