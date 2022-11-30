/*
MIT LICENSE

Copyright 2014-2019 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT, IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include <flash_efc.h>
#include "ASF/common/components/wifi/winc3400/wifi_drv/socket/include/socket.h"
#include "d_flash.h"
#include "version/version.h"
#include "version/repositoryInfo.h"
#include "version/buildInfo.h"
#include "globals.h"


dev_info_t                  g_evbDevInfo = {0};
wheel_encoder_t				g_wheelEncoder = {0};
uint32_t                    g_wheelEncoderTimeMs = 0;
evb_status_t                g_status = {0};
bool                        g_statusToWlocal = true;
evb_flash_cfg_t*            g_flashCfg;
nvr_manage_t                g_nvr_manage_config;
nvm_config_t                g_userPage = {0};
uins_msg_t                  g_uins = {0};
imu_t                       g_imu = {0};
uint32_t                    g_insUpdateTimeMs = 0;
uint32_t                    g_imuUpdateTimeMs = 0;
debug_array_t               g_debug = {0};
evb_rtos_info_t             g_rtos = {0};
date_time_t                 g_gps_date_time = {0};
//uint32_t					g_CANbaud_kbps = 500;
//uint32					g_can_receive_address = 0;
bool                        g_gpsTimeSync = false;
uint32_t                    g_comm_time_ms = 0;
double                      g_comm_time = 0;
double                      g_towOffset = 0;
bool                        g_loggerEnabled = false;
uint32_t                    g_uInsBootloaderEnableTimeMs = 0;	// 0 = disabled
bool                        g_enRtosStats = 0;
ermc_t                      g_ermc = {0};


void globals_init(void)
{
	// Init Device Info struct
	g_evbDevInfo.hardwareVer[0] = 2;
	switch(g_hdw_detect)
	{
    default:
    	g_evbDevInfo.hardwareVer[1] = 0;
        break;
	case HDW_DETECT_VER_EVB_2_1_0:
	case HDW_DETECT_VER_EVB_2_1_1:
	    g_evbDevInfo.hardwareVer[1] = 1;
        break;
	case HDW_DETECT_VER_EVB_2_2_0:
	    g_evbDevInfo.hardwareVer[1] = 2;
        break;
	}
	switch(g_hdw_detect)
	{
    default:
    	g_evbDevInfo.hardwareVer[2] = 0;
        break;
	case HDW_DETECT_VER_EVB_2_0_1:
	case HDW_DETECT_VER_EVB_2_1_1:
	    g_evbDevInfo.hardwareVer[2] = 1;	
        break;
	}
	g_evbDevInfo.hardwareVer[3] = 0;

	g_evbDevInfo.firmwareVer[0] = FIRMWARE_VERSION_CHAR0;       // Major
	g_evbDevInfo.firmwareVer[1] = FIRMWARE_VERSION_CHAR1;       // Minor
	g_evbDevInfo.firmwareVer[2] = FIRMWARE_VERSION_CHAR2;       // Revision
	g_evbDevInfo.firmwareVer[3] = FIRMWARE_VERSION_CHAR3;       // firmware specific revision

	g_evbDevInfo.protocolVer[0] = PROTOCOL_VERSION_CHAR0;		// in com_manager.h
	g_evbDevInfo.protocolVer[1] = PROTOCOL_VERSION_CHAR1;
	g_evbDevInfo.protocolVer[2] = PROTOCOL_VERSION_CHAR2;		// in data_sets.h
	g_evbDevInfo.protocolVer[3] = PROTOCOL_VERSION_CHAR3;

	g_evbDevInfo.repoRevision = REPO_HEAD_COUNT;
	g_evbDevInfo.buildNumber  = BUILD_NUMBER;

#if defined(DEBUG)
	g_evbDevInfo.buildDate[0] = 'd';					// Debug
#else
	g_evbDevInfo.buildDate[0] = 'r';					// Release
#endif

#if 0
    setBuildDateTimeFromCompileTime(g_evbDevInfo.buildDate, g_evbDevInfo.buildTime);
#else
	g_evbDevInfo.buildDate[1] = BUILD_DATE_YEAR-2000;
	g_evbDevInfo.buildDate[2] = BUILD_DATE_MONTH;
	g_evbDevInfo.buildDate[3] = BUILD_DATE_DAY;

	g_evbDevInfo.buildTime[0] = BUILD_TIME_HOUR;
	g_evbDevInfo.buildTime[1] = BUILD_TIME_MINUTE;
	g_evbDevInfo.buildTime[2] = BUILD_TIME_SECOND;
	g_evbDevInfo.buildTime[3] = (uint8_t)BUILD_TIME_MILLISECOND;
#endif

	strncpy(g_evbDevInfo.manufacturer, "Inertial Sense INC", DEVINFO_MANUFACTURER_STRLEN);
}


void com_bridge_apply_preset_uins_com(evb_flash_cfg_t* cfg, uint8_t uinsComPort);
void com_bridge_apply_preset_uins_com(evb_flash_cfg_t* cfg, uint8_t uinsComPort)
{
	if(uinsComPort >= EVB2_PORT_COUNT)
	{
		return;
	}
		
	if(uinsComPort != EVB2_PORT_USB)
	{
		cfg->cbf[uinsComPort]		|= (1<<EVB2_PORT_USB);
		cfg->cbf[EVB2_PORT_USB]     |= (1<<uinsComPort);
	}

	if(uinsComPort != EVB2_PORT_SP330)
	{
		cfg->cbf[uinsComPort]		|= (1<<EVB2_PORT_SP330);
		cfg->cbf[EVB2_PORT_SP330]   |= (1<<uinsComPort);
	}
}


void com_bridge_apply_preset_uins_aux(evb_flash_cfg_t* cfg, uint8_t uinsAuxPort);
void com_bridge_apply_preset_uins_aux(evb_flash_cfg_t* cfg, uint8_t uinsAuxPort)
{
// 	if (cfg->uinsComPort == uinsAuxPort)
// 	{	// Using UINS-AUX as primary communication link.  Skip this section. 
// 		return;
// 	}
	
	if (uinsAuxPort >= EVB2_PORT_COUNT)
	{	// Port disabled
		return;
	}

	// XRadio Forwarding		
    if (uinsAuxPort != EVB2_PORT_XRADIO)
    {
		switch(cfg->cbPreset)
		{
		case EVB2_CB_PRESET_RS232:
		case EVB2_CB_PRESET_RS232_XBEE:
		case EVB2_CB_PRESET_RS422_WIFI:
			cfg->cbf[uinsAuxPort]		|= (1<<EVB2_PORT_XRADIO);
			cfg->cbf[EVB2_PORT_XRADIO]	|= (1<<uinsAuxPort);
		}
	}

	// XBee Forwarding
    if ((uinsAuxPort != EVB2_PORT_XBEE) &&
		(cfg->cbPreset == EVB2_CB_PRESET_RS232_XBEE))
    {
		cfg->cbf[uinsAuxPort]		|= (1<<EVB2_PORT_XBEE);
		cfg->cbf[EVB2_PORT_XBEE]	|= (1<<uinsAuxPort);
	}
	
	// WiFi Forwarding
    if ((uinsAuxPort != EVB2_PORT_WIFI) &&
		(cfg->cbPreset == EVB2_CB_PRESET_RS422_WIFI))
    {
		cfg->cbf[uinsAuxPort]		|= (1<<EVB2_PORT_WIFI);
		cfg->cbf[EVB2_PORT_WIFI]	|= (1<<uinsAuxPort);
	}
}


void com_bridge_apply_preset(evb_flash_cfg_t* cfg)
{
    if(cfg->cbPreset==EVB2_CB_PRESET_NA)
    {   // Do nothing
        return;
    }
    
    // Comm Bridge
    memset(cfg->cbf, 0, sizeof(cfg->cbf));      // Clear com bridge settings
    switch(cfg->cbPreset)
    {
    case EVB2_CB_PRESET_RS232:		
    case EVB2_CB_PRESET_RS232_XBEE:
    case EVB2_CB_PRESET_RS422_WIFI:
		com_bridge_apply_preset_uins_com(cfg, cfg->uinsComPort);		// UINS communication port used for SD logging.
		com_bridge_apply_preset_uins_aux(cfg, cfg->uinsAuxPort);		// UINS auxiliary port used for RTK corrections.
        break;

    case EVB2_CB_PRESET_SPI_RS232:
		// SPI mode must be on port UINS1
        cfg->cbf[EVB2_PORT_UINS1]   |= (1<<EVB2_PORT_USB);
        cfg->cbf[EVB2_PORT_USB]     |= (1<<EVB2_PORT_UINS1);

        cfg->cbf[EVB2_PORT_UINS1]   |= (1<<EVB2_PORT_SP330);
        cfg->cbf[EVB2_PORT_SP330]   |= (1<<EVB2_PORT_UINS1);
        break;
        
    case EVB2_CB_PRESET_USB_HUB_RS232:
		 cfg->cbf[EVB2_PORT_USB]    |= (1<<EVB2_PORT_XBEE);
		 cfg->cbf[EVB2_PORT_XBEE]   |= (1<<EVB2_PORT_USB);

        cfg->cbf[EVB2_PORT_USB]     |= (1<<EVB2_PORT_SP330);
        cfg->cbf[EVB2_PORT_SP330]   |= (1<<EVB2_PORT_USB);

        cfg->cbf[EVB2_PORT_USB]     |= (1<<EVB2_PORT_GPIO_H8);
        cfg->cbf[EVB2_PORT_GPIO_H8] |= (1<<EVB2_PORT_USB);

        cfg->cbf[EVB2_PORT_USB]     |= (1<<EVB2_PORT_XRADIO);
        cfg->cbf[EVB2_PORT_XRADIO]  |= (1<<EVB2_PORT_USB);
		break;
		 
    case EVB2_CB_PRESET_USB_HUB_RS422:		
        cfg->cbf[EVB2_PORT_USB]     |= (1<<EVB2_PORT_SP330);
        cfg->cbf[EVB2_PORT_SP330]   |= (1<<EVB2_PORT_USB);

        cfg->cbf[EVB2_PORT_USB]     |= (1<<EVB2_PORT_GPIO_H8);
        cfg->cbf[EVB2_PORT_GPIO_H8] |= (1<<EVB2_PORT_USB);

        cfg->cbf[EVB2_PORT_USB]     |= (1<<EVB2_PORT_XRADIO);
        cfg->cbf[EVB2_PORT_XRADIO]  |= (1<<EVB2_PORT_USB);

//         cfg->cbf[EVB2_PORT_USB]     |= (1<<EVB2_PORT_BLE);
//         cfg->cbf[EVB2_PORT_BLE]     |= (1<<EVB2_PORT_USB);
		break;
         
#ifdef CONF_BOARD_CAN1
	//case  EVB2_CB_PRESET_CAN:
		//cfg->cbf[EVB2_PORT_UINS0]  |= (1<<EVB2_PORT_USB);
		//cfg->cbf[EVB2_PORT_UINS0]  |= (1<<EVB2_PORT_SP330);
//
		//cfg->cbf[EVB2_PORT_USB]    |= (1<<EVB2_PORT_UINS0);
        //cfg->cbf[EVB2_PORT_SP330]  |= (1<<EVB2_PORT_UINS0);
//
		//cfg->cbf[EVB2_PORT_UINS1]  |= (1<<EVB2_PORT_CAN);
		//cfg->cbf[EVB2_PORT_CAN]	   |= (1<<EVB2_PORT_USB);
		//break;
#endif
    }        

    // Clear existing
    cfg->cbOptions = 0;

	// Tri-state uINS
	switch(cfg->cbPreset)
	{
	case EVB2_CB_PRESET_NA:
	case EVB2_CB_PRESET_ALL_OFF:
	case EVB2_CB_PRESET_USB_HUB_RS232:
	case EVB2_CB_PRESET_USB_HUB_RS422:
	case EVB2_CB_PRESET_COUNT:
		cfg->cbOptions |= EVB2_CB_OPTIONS_TRISTATE_UINS_IO;
		break;
	}
        
    // SP330 Options
    switch(cfg->cbPreset)
    {
    case EVB2_CB_PRESET_RS422_WIFI:
    case EVB2_CB_PRESET_USB_HUB_RS422:
        cfg->cbOptions |= EVB2_CB_OPTIONS_SP330_RS422;
        break;
    }

    // XBee and WiFi Power Options
    switch(cfg->cbPreset)
    {
    case EVB2_CB_PRESET_RS232_XBEE:
    case EVB2_CB_PRESET_USB_HUB_RS232:
        cfg->cbOptions |= EVB2_CB_OPTIONS_XBEE_ENABLE;
        break;

    case EVB2_CB_PRESET_RS422_WIFI:
        cfg->cbOptions |= EVB2_CB_OPTIONS_WIFI_ENABLE;
        break;        
    }

    // Enable SPI example
#ifdef CONF_BOARD_SPI_UINS
    switch(cfg->cbPreset)
    {
    case EVB2_CB_PRESET_SPI_RS232:
        cfg->cbOptions |= EVB2_CB_OPTIONS_SPI_ENABLE;
        break;
    }
#endif

	// Enable CAN
//#ifdef CONF_BOARD_CAN1
	//switch(cfg->cbPreset)
	//{
		//case EVB2_CB_PRESET_CAN:
		//cfg->cbOptions |= EVB2_CB_OPTIONS_CAN_ENABLE;
		//break;
	//}
//#endif
}


void concatStringWithSpace(char* buf, size_t bufLen, const char* concat)
{
	if (bufLen > 0)
	{
		size_t len = strnlen(buf, bufLen);
		if (len != 0 && len < bufLen - 1)
		{
			buf[len++] = ' ';
			buf[len] = '\0';
		}
		strncat(buf, concat, bufLen - len - 1);
	}    
}


bool nvr_validate_config_integrity(evb_flash_cfg_t* cfg)
{
    evb_flash_cfg_t defaults;
    memset(&defaults, 0, sizeof(evb_flash_cfg_t));    
    reset_config_defaults(&defaults);

    bool valid = true;
    if (cfg->checksum != flashChecksum32(cfg, sizeof(evb_flash_cfg_t)) || cfg->key != defaults.key)
    {   // checksum failure
        valid = false;
    }

    if (error_check_config(cfg))
    {   // Values are outside valid ranges
        valid = false;        
    }

	// Flash Config
	if (!valid)    
    {   // Reset to defaults
        *cfg = defaults;
        nvr_flash_config_write_needed();
        nvr_flash_config_write_enable();
    }   
    
    // Disable cbPresets is necessary
#ifndef CONF_BOARD_SPI_ATWINC_WIFI
    if (cfg->cbPreset == EVB2_CB_PRESET_RS422_WIFI){ cfg->cbPreset = EVB2_CB_PRESET_DEFAULT; }
#endif
#ifndef CONF_BOARD_SPI_UINS
    if (cfg->cbPreset == EVB2_CB_PRESET_SPI_RS232){ cfg->cbPreset = EVB2_CB_PRESET_DEFAULT; }
#endif

    return valid;
}


void nvr_init(void)
{
    // Initialize flash: 6 wait states for flash writing
    uint32_t rc = flash_init(FLASH_ACCESS_MODE_128, 6);
    configASSERT(rc == FLASH_RC_OK);
    
    g_flashCfg = &(g_userPage.g0.m);
    
    // Update RAM from FLASH
    memcpy(&g_userPage, (void*)BOOTLOADER_FLASH_CONFIG_BASE_ADDRESS, sizeof(g_userPage));

    // Disable flash writes.  We require the user to initiate each write with this option.
    g_nvr_manage_config.flash_write_enable_timeMs = 0;
    g_status.evbStatus &= ~EVB_STATUS_FLASH_WRITE_IN_PROGRESS;

    // Reset to defaults if checksum or keys don't match
    nvr_validate_config_integrity(g_flashCfg);    
}


bool nvr_slow_maintenance(void)
{
    if (g_nvr_manage_config.flash_write_enable_timeMs==0)
    {
        return false;
    }

	uint32_t dtMs = time_msec() - g_nvr_manage_config.flash_write_enable_timeMs;
	if (dtMs < 1000)
	{	// Wait 1 second following flash_write_enable_timeMs
		return false;
	}
    
    bool flash_write = false;

    if (g_nvr_manage_config.flash_write_needed)
    {
        // Ensure flash config isn't larger than block
        STATIC_ASSERT(sizeof(nvm_config_t) <= BOOTLOADER_FLASH_BLOCK_SIZE);

#if 0

        // copy contents, set migration marker and re-calculate checksums inside critical section
        BEGIN_CRITICAL_SECTION
        
        nvm_config_t* copyOfRam = (nvm_config_t*)MALLOC(BOOTLOADER_FLASH_BLOCK_SIZE);
        
        // Copy config data, zero pad, update checksum
        memcpy(copyOfRam, &g_userPage, sizeof(g_userPage));
        memset((uint8_t*)copyOfRam + sizeof(g_userPage), 0, BOOTLOADER_FLASH_BLOCK_SIZE - sizeof(g_userPage));
        copyOfRam->g0.m.checksum = flashChecksum32(copyOfRam, sizeof(evb_flash_cfg_t));

        // Update userPage config
		memcpy(&g_userPage, copyOfRam, sizeof(g_userPage));
        
        END_CRITICAL_SECTION

        uint32_t address = BOOTLOADER_FLASH_CONFIG_BASE_ADDRESS;
        uint32_t memsize = BOOTLOADER_FLASH_BLOCK_SIZE;
//         uint32_t memsize = IFLASH_PAGE_SIZE;

#if 0

	    if (flash_unlock(address, address + memsize - 1, 0, 0) == FLASH_RC_OK) 
        {
	        if (flash_erase_page(address, IFLASH_ERASE_PAGES_16) == FLASH_RC_OK)
	        {
	            if (flash_write(address, &copyOfRam, memsize, 0) == FLASH_RC_OK)
	            {
                    if(	flash_lock(address, address + memsize - 1, 0, 0) == FLASH_RC_OK)
                    {   // indicate attempted flash write
	                    g_nvr_manage_config.flash_write_count++;
                    }                        
	            }
	        }
	    }
        
#else
        // write to primary location
        if( flash_update_block(address, copyOfRam, BOOTLOADER_FLASH_BLOCK_SIZE, 0) != FLASH_RC_OK)
        {
            // error
            int j=0;
            j++;
        }                    
              
#endif      

		FREE(copyOfRam);
		
#else

        BEGIN_CRITICAL_SECTION

		// calculate checksum
		g_userPage.g0.m.size = sizeof(evb_flash_cfg_t);
		g_userPage.g0.m.checksum = flashChecksum32(&g_userPage.g0.m, sizeof(evb_flash_cfg_t));

// 		flash_update_block(BOOTLOADER_FLASH_CONFIG_BASE_ADDRESS, &g_userPage, sizeof(nvm_config_t), 0);
		flash_update_block(BOOTLOADER_FLASH_CONFIG_BASE_ADDRESS, &g_userPage, BOOTLOADER_FLASH_BLOCK_SIZE, 0);
		
        END_CRITICAL_SECTION

#endif

		g_nvr_manage_config.flash_write_count++;
        g_nvr_manage_config.flash_write_needed = 0;
        // Disable following each flash write.  We require users to re-enable for each write.
        g_nvr_manage_config.flash_write_enable_timeMs = 0;
        g_status.evbStatus &= ~EVB_STATUS_FLASH_WRITE_IN_PROGRESS;
    	flash_write = false;
    }    

    return flash_write;
}


void nvr_flash_config_write_needed(void)
{
	g_nvr_manage_config.flash_write_needed++;
}


// Used to enabled flash writes at controlled times.  The system may identify when a flash write is needed. 
// However, this will not occur until flash is enabled (at time EKF can tolerate stutters in RTOS update).
void nvr_flash_config_write_enable(void)
{
    g_nvr_manage_config.flash_write_enable_timeMs = time_msec();
    g_status.evbStatus |= EVB_STATUS_FLASH_WRITE_IN_PROGRESS;
}


// Returns 0 on success
int error_check_config(evb_flash_cfg_t *cfg)
{
    if(cfg == NULLPTR)
    {
        return -1;
    }
    
    int failure = 0;

    if (cfg->radioPID > 9 ||
        cfg->radioNID > 0x7FFF ||
        cfg->radioPowerLevel > 2 )
    {
        failure = -1;
    }
    if(EVB_CFG_BITS_IDX_WIFI(cfg->bits) >= NUM_WIFI_PRESETS)
    {
        EVB_CFG_BITS_SET_IDX_WIFI(cfg->bits,0);
        failure = -1;
    }        
    if(EVB_CFG_BITS_IDX_SERVER(cfg->bits) >= NUM_WIFI_PRESETS)
    {
        EVB_CFG_BITS_SET_IDX_SERVER(cfg->bits,0);
        failure = -1;
    }
    
    return failure;
}


void reset_config_defaults( evb_flash_cfg_t *cfg )
{
	if (cfg == NULL)
		return;

	memset(cfg, 0, sizeof(evb_flash_cfg_t));
	cfg->size						= sizeof(evb_flash_cfg_t);
	cfg->key						= 3;			// increment key to force config to revert to defaults (overwrites customer's settings)

	cfg->cbPreset = EVB2_CB_PRESET_DEFAULT;

	cfg->uinsComPort = EVB2_PORT_UINS0;				// Default port for uINS communications and SD card logging
	cfg->uinsAuxPort = EVB2_PORT_UINS0;				// Default port for RTK corrections and wireless interface
	cfg->portOptions = EVB2_PORT_OPTIONS_DEFAULT;	// Enable radio RTK filter
	cfg->h3sp330BaudRate = 921600;
	cfg->h4xRadioBaudRate = 115200;
	cfg->h8gpioBaudRate = 921600;

	cfg->radioPID = 2;          // 0x0 to 0x9
	cfg->radioNID = 72;         // 0x0 to 0x7FFF
	cfg->radioPowerLevel = 0;	// 0=20dBm use low power so radio can run from USB supply.  1=27dBm and 2=30dBm are too powerful for USB supply.
	
	cfg->server[0].ipAddr.u32 = nmi_inet_addr((void*)"69.167.49.43");
	cfg->server[0].port = 7778;
	cfg->server[1].ipAddr.u32 = nmi_inet_addr((void*)"192.168.1.144");
	cfg->server[1].port = 2000;
// 	cfg->encoderTickToWheelRad = 0.0359998f;	// Husqvarna lawnmower
	cfg->encoderTickToWheelRad = 0.108329996f;	// Husqvarna lawnmower
	// cfg->encoderTickToWheelRad = 0.00523598775598298873f;	// = 2 Pi / (400 count encoder x 3 gear ratio), (ZT mower)
	
    cfg->velocityControlPeriodMs = 50;    // 20 Hz

	com_bridge_apply_preset(cfg);
	
	cfg->checksum = flashChecksum32(cfg, sizeof(evb_flash_cfg_t));
}


void setBuildDateTimeFromCompileTime(uint8_t buildDate[4], uint8_t buildTime[4])
{
    char date[20] = __DATE__;        //  mmm dd yyyy (e.g. "Jan 14 2012")
    int day=0, month=0, year=0, hour=0, minute=0, second=0;
		if (date[0] == 'J' && 
			date[1] == 'a' && 
			date[2] == 'n'){ month = 1; }           // month
    else if(date[0] == 'F' && 
			date[1] == 'e' && 
			date[2] == 'b'){ month = 2; }
    else if(date[0] == 'M' && 
			date[1] == 'a' && 
			date[2] == 'r'){ month = 3; }
    else if(date[0] == 'A' && 
			date[1] == 'p' && 
			date[2] == 'r'){ month = 4; }
    else if(date[0] == 'M' && 
			date[1] == 'a' && 
			date[2] == 'y'){ month = 5; }
    else if(date[0] == 'J' && 
			date[1] == 'u' && 
			date[2] == 'n'){ month = 6; }
    else if(date[0] == 'J' && 
			date[1] == 'u' && 
			date[2] == 'l'){ month = 7; }
    else if(date[0] == 'A' && 
			date[1] == 'u' && 
			date[2] == 'g'){ month = 8; }
    else if(date[0] == 'S' && 
			date[1] == 'e' && 
			date[2] == 'p'){ month = 9; }
    else if(date[0] == 'O' && 
			date[1] == 'c' && 
			date[2] == 't'){ month = 10; }
    else if(date[0] == 'N' && 
			date[1] == 'o' && 
			date[2] == 'v'){ month = 11; }
    else if(date[0] == 'D' && 
			date[1] == 'e' && 
			date[2] == 'c'){ month = 12; }
    day  = atoi(&date[4]);     // day
    year = atoi(&date[7]);     // year

    char time[20] = __TIME__;  // hh:mm:ss in 24 hour time (e.g. "22:29:12")
    hour   = atoi(&time[0]);   // hour
    minute = atoi(&time[3]);   // minute
    second = atoi(&time[6]);   // second

    buildDate[1] = year - 2000;
    buildDate[2] = month;
    buildDate[3] = day;

    buildTime[0] = hour;
    buildTime[1] = minute;
    buildTime[2] = second;
    buildTime[3] = 0;
}
