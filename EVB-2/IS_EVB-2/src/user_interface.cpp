/*
MIT LICENSE

Copyright 2014-2019 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT, IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include <asf.h>
#include "globals.h"
#include "sd_card_logger.h"
#include "communications.h"
#include "user_interface.h"


static VoidFuncPtrVoid s_funcPtrCfgButtonPressed = NULL;
static VoidFuncPtrVoid s_funcPtrCfgButtonRelease = NULL;
static VoidFuncPtrVoid s_funcPtrLogButtonPressed = NULL;
static VoidFuncPtrVoid s_funcPtrLogButtonRelease = NULL;
static VoidFuncPtrVoid s_funcPtrBothButtonsPressed = NULL;
static VoidFuncPtrVoid s_funcPtrBothButtonsRelease = NULL;

static VoidFuncPtrVoid s_funcPtrRefreshLedCfg = NULL;
static VoidFuncPtrVoid s_funcPtrRefreshLedLog = NULL;

bool logger_ready()
{
    return (g_status.evbStatus&EVB_STATUS_SD_CARD_READY) && !(g_status.evbStatus&EVB_STATUS_SD_ERR_CARD_MASK);
}


static void on_cfg_button_pressed()
{
	// Indicate button is pressed by turning off LED
	LED_CFG_OFF();
}

static void on_cfg_button_release()
{    
    // Increment config
    g_flashCfg->cbPreset++;

    switch(g_flashCfg->cbPreset)
    {   // Skip these configs
#ifndef CONF_BOARD_SPI_ATWINC_WIFI
        case EVB2_CB_PRESET_RS422_WIFI:
#endif
#ifndef CONF_BOARD_SPI_UINS
        case EVB2_CB_PRESET_SPI_RS232:
#endif
        g_flashCfg->cbPreset++;
    }

    // Restart config
    if(g_flashCfg->cbPreset >= EVB2_CB_PRESET_COUNT)
    {
        g_flashCfg->cbPreset = 1;
    }

    com_bridge_apply_preset(g_flashCfg);
    board_IO_config();
    nvr_flash_config_write_needed();
    nvr_flash_config_write_enable();
	evbUiRefreshLedCfg();
}

static void on_log_button_pressed()
{
	// Indicate button is pressed by turning off LED
	LED_LOG_OFF();    
}

static void on_log_button_release()
{
    if( logger_ready() )
    {
        if (!(g_flashCfg->bits & EVB_CFG_BITS_NO_STREAM_PPD_ON_LOG_BUTTON))
        {
            uINS_stream_enable_PPD();
        }
        
        // Toggle logger enable
        enable_logger(!g_loggerEnabled);
    }
}

static void on_both_buttons_pressed()
{   // Reset uINS
	ioport_set_pin_output_mode(INS_RESET_PIN_PIN, IOPORT_PIN_LEVEL_LOW);
}

static void on_both_buttons_release()
{   // De-assert uINS reset
    ioport_set_pin_input_mode(INS_RESET_PIN_PIN, 0, 0);
	evbUiRefreshLedCfg();
}


#define BUTTON_DEBOUNCE_TIME_MS 100
void step_user_interface(uint32_t time_ms)
{
    bool cfgBtnDown = !ioport_get_pin_level(BUTTON_CFG_PIN);
    bool logBtnDown = !ioport_get_pin_level(BUTTON_LOG_PIN);
    static bool bothBtnsDown = false;
    static bool cfgBtnDownLast = cfgBtnDown;
    static bool logBtnDownLast = logBtnDown;
    static bool bothBtnsDownLast = bothBtnsDown;
	static uint32_t cfgBtnTimeMs = 0;	// for button debouncing
	static uint32_t logBtnTimeMs = 0;
	static uint32_t bothBtnTimeMs = 0;
    static bool ignoreEstopBtnRelease=false;
    static bool ignorePauseBtnRelease=false;

    if (cfgBtnDownLast != cfgBtnDown && time_ms-cfgBtnTimeMs > BUTTON_DEBOUNCE_TIME_MS)
    {   // Button toggled        
        cfgBtnDownLast = cfgBtnDown;
		cfgBtnTimeMs = time_ms;
        if(cfgBtnDown)
        {      
            if(s_funcPtrCfgButtonPressed){ s_funcPtrCfgButtonPressed(); }
        }
        else
        {
            if(!ignoreEstopBtnRelease)
            {
                if(s_funcPtrCfgButtonRelease){ s_funcPtrCfgButtonRelease(); }
            }
            ignoreEstopBtnRelease = false;
        }
    }

    if(logBtnDownLast != logBtnDown && time_ms-logBtnTimeMs > BUTTON_DEBOUNCE_TIME_MS)
    {   // Button toggled
        logBtnDownLast = logBtnDown;
		logBtnTimeMs = time_ms;
        if(logBtnDown)
        {      
            if(s_funcPtrLogButtonPressed){ s_funcPtrLogButtonPressed(); }
        }
        else
        {              
            if(!ignorePauseBtnRelease)
            {
                if(s_funcPtrLogButtonRelease){ s_funcPtrLogButtonRelease(); }
            }
            ignorePauseBtnRelease = false;
        }            
    }
    
	// Only toggle down on both down and up on both up.
	if (bothBtnsDown)
	{
		if(!cfgBtnDown && !logBtnDown)
		{
			bothBtnsDown = false;
		}
	}
	else
	{
		if(cfgBtnDown && logBtnDown)
		{
			bothBtnsDown = true;
		}
	}

    if(bothBtnsDownLast != bothBtnsDown && time_ms-bothBtnTimeMs > BUTTON_DEBOUNCE_TIME_MS)
    {   // Both buttons toggled
        bothBtnsDownLast = bothBtnsDown;
		bothBtnTimeMs = time_ms;
        if(bothBtnsDown)
        {                
            if(s_funcPtrBothButtonsPressed){ s_funcPtrBothButtonsPressed(); } 
            ignoreEstopBtnRelease = true;
            ignorePauseBtnRelease = true;
        }
        else
        {
            if(s_funcPtrBothButtonsRelease){ s_funcPtrBothButtonsRelease(); }  
        }
    }
    
    if( logger_ready() )
    {   // Handle logger commands via g_status.loggerMode
        if(g_status.loggerMode!=EVB2_LOG_NA)
        {
            switch(g_status.loggerMode)
            {
                case EVB2_LOG_CMD_START:    enable_logger(true);    break;
                case EVB2_LOG_CMD_STOP:     enable_logger(false);   break;  // disable logger
            }
            g_status.loggerMode = EVB2_LOG_NA;
        }
    }
    else
    {   
        if(g_loggerEnabled==1)
        {   // Disable logger
            enable_logger(false);
        }
    }    
}


void evbUiDefaults()
{
    s_funcPtrCfgButtonPressed = on_cfg_button_pressed;
    s_funcPtrCfgButtonRelease = on_cfg_button_release;
    s_funcPtrLogButtonPressed = on_log_button_pressed;
    s_funcPtrLogButtonRelease = on_log_button_release;
    s_funcPtrBothButtonsPressed = on_both_buttons_pressed;
    s_funcPtrBothButtonsRelease = on_both_buttons_release;

    s_funcPtrRefreshLedCfg = refresh_led_cfg;
    s_funcPtrRefreshLedLog = refresh_led_log;
}

void evbUiButtonCallbacks(
    VoidFuncPtrVoid fpCfgButtonPressed, VoidFuncPtrVoid fpCfgButtonRelease, 
    VoidFuncPtrVoid fpLogButtonPressed, VoidFuncPtrVoid fpLogButtonRelease, 
    VoidFuncPtrVoid fpBothButtonsPressed, VoidFuncPtrVoid fpBothButtonsRelease )
{
    s_funcPtrCfgButtonPressed = fpCfgButtonPressed;
    s_funcPtrCfgButtonRelease = fpCfgButtonRelease;
    s_funcPtrLogButtonPressed = fpLogButtonPressed;
    s_funcPtrLogButtonRelease = fpLogButtonRelease;
    s_funcPtrBothButtonsPressed = fpBothButtonsPressed;
    s_funcPtrBothButtonsRelease = fpBothButtonsRelease;
}

void evbUiLedCallbacks(VoidFuncPtrVoid fpLedCfg, VoidFuncPtrVoid fpLedLog )
{
    s_funcPtrRefreshLedCfg = fpLedCfg;
    s_funcPtrRefreshLedLog = fpLedLog;
}

void evbUiRefreshLedCfg()
{
    if(s_funcPtrRefreshLedCfg){ s_funcPtrRefreshLedCfg(); }  
}

void evbUiRefreshLedLog()
{
    if(s_funcPtrRefreshLedLog){ s_funcPtrRefreshLedLog(); }  
}

