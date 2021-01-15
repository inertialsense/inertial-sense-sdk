/*
MIT LICENSE

Copyright 2014-2019 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT, IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include "evb_tasks.h"
#include "../../../hw-libs/drivers/d_i2c.h"


is_comm_instance_t& evbTaskCommInit(void *pvParameters)
{	
    UNUSED(pvParameters);
    static is_comm_instance_t   comm;
    static uint8_t              comm_buffer[PKT_BUF_SIZE];
    is_comm_init(&comm, comm_buffer, PKT_BUF_SIZE);

#ifdef CONF_BOARD_CAN_TEST
	//if(/*g_can_test == CAN_TEST_MASTER - or something like that*/ 1)
		//mcan_test_master();
	if(/*g_can_test == CAN_TEST_SLAVE - or something like that*/ 1)
		mcan_test_slave();
#endif

	// Start USB CDC after everything is running
#ifdef USB_PORT_NUM
	serInit(USB_PORT_NUM, 0, NULL, 0);
#endif

    vTaskDelay(200);
    refresh_CFG_LED();

    return comm;
}


void evbTaskComm(rtos_task_t &task, is_comm_instance_t &comm)
{
    vTaskDelay(task.periodMs);

    g_comm_time_ms = time_msec();
    
#ifdef ENABLE_WDT
    // Feed Watchdog to prevent reset
    wdt_restart(WDT);
#endif
    
    // Turn off communications LEDs - first thing after vTaskDelay()
    LED_OFF(LED_INS_RXD_PIN);
    LED_OFF(LED_INS_TXD_PIN);
    
    // Forward data between communications ports
    step_com_bridge(comm);
    velocity_control(comm);

    // Read buttons and update LEDs
    step_user_interface(g_comm_time_ms);
}


is_comm_instance_t& evbTaskLoggerInit(void *pvParameters)
{
    UNUSED(pvParameters);
    static is_comm_instance_t   comm;
    static uint8_t              comm_buffer[PKT_BUF_SIZE];
    is_comm_init(&comm, comm_buffer, PKT_BUF_SIZE);

    uINS_stream_stop_all(comm);

    vTaskDelay(200);
    LED_LOG_OFF();
    vTaskDelay(800);

#if STREAM_INS_FOR_TIME_SYNC  // Stream INS message on startup.  Necessary to update EVB RTC for correct data log date and time.
    //uINS0_stream_stop_all(comm);
    //uINS0_stream_enable_std(comm);
#endif

    return comm;
}


void evbTaskLogger(rtos_task_t &task, is_comm_instance_t &comm)
{
    static cISLogger logger = {};

    vTaskDelay(task.periodMs);

    step_logger_control(logger, comm);
    
    // Ready uINS data from com task.  Log to file.
    log_uINS_data(logger, comm);
    
    // Mount/unmount SD card
    sd_card_maintenance();

    update_led_log();
}


void evbTaskMaintInit(void *pvParameters)
{
	UNUSED(pvParameters);
}


int evbTaskMaint(rtos_task_t &task)
{
    static uint32_t m2sPeriodMs = 0;    

    vTaskDelay(task.periodMs);
    
    //////////////////////////////////////////////////////////////////////////
    // Fast Maintenance - 10ms period


    //////////////////////////////////////////////////////////////////////////
    // Slow Maintenance - 1000ms period 
    if ((m2sPeriodMs += TASK_MAINT_PERIOD_MS) < TASK_MAINT_SLOW_SEC_PERIOD_MS || g_loggerEnabled)
    {
        return 1;
    }
    m2sPeriodMs = 0;
    
    // Sync local time from uINS
    time_sync_from_uINS();

    nvr_slow_maintenance();

    // Update RTOS stats
    if (g_enRtosStats)
    {
        rtos_monitor(EVB_RTOS_NUM_TASKS);
    }
    
    if (g_uInsBootloaderEnableTimeMs)
    {	// uINS bootloader mode enabled
        if ( (g_comm_time_ms-g_uInsBootloaderEnableTimeMs) > 180000 )
        {	// Automatically disable uINS after 3 minutes 
            g_uInsBootloaderEnableTimeMs = 0;
        }			
    }

    return 0;
}


void evbMainInit(pdTASK_CODE pxTaskComm,
				pdTASK_CODE pxTaskLogg,
				pdTASK_CODE pxTaskWifi,
				pdTASK_CODE pxTaskMant )
{
	//XDMAC channel interrupt enables do not get cleared by a software reset. Clear them before they cause issues.
	XDMAC->XDMAC_GID = 0xFFFFFFFF;
	for(int i=0;i<XDMACCHID_NUMBER;i++)
    {
		XDMAC->XDMAC_CHID[i].XDMAC_CID = 0xFFFFFFFF;
    }
		
	// Force USB to disconnect. Helps to make sure the USB port starts up correctly when debugging.
	udc_stop();
	    
	// Initialize the SAM system
	board_init();
    
    // Init globals and flash parameters
    globals_init();
    nvr_init();

    // Hold config while resetting
    if(!ioport_get_pin_level(BUTTON_CFG_PIN))
    {   
        g_flashCfg->cbPreset = EVB2_CB_PRESET_DEFAULT;
    }
    
	// Init hardware I/O, SD card logger, and communications
    board_IO_config();
	init_control();	
    sd_card_logger_init();
    communications_init();
	adc_init();
	i2cInit();
	
	// Create RTOS tasks
	createTask(EVB_TASK_COMMUNICATIONS, pxTaskComm,  "COMM",   TASK_COMM_STACK_SIZE,  NULL, TASK_COMM_PRIORITY,  TASK_COMM_PERIOD_MS);
	createTask(EVB_TASK_LOGGER,         pxTaskLogg,"LOGGER", TASK_LOGGER_STACK_SIZE, NULL, TASK_LOGGER_PRIORITY, TASK_LOGGER_PERIOD_MS);
#ifdef CONF_BOARD_SPI_ATWINC_WIFI       // ATWINC WIFI
	createTask(EVB_TASK_WIFI,           pxTaskWifi,  "WIFI",   TASK_WIFI_STACK_SIZE,  NULL, TASK_WIFI_PRIORITY,  TASK_WIFI_PERIOD_MS);
#endif
	createTask(EVB_TASK_MAINTENANCE,    pxTaskMant, "MAINT",  TASK_MAINT_STACK_SIZE, NULL, TASK_MAINT_PRIORITY, TASK_MAINT_PERIOD_MS);
	strncpy(g_rtos.task[EVB_TASK_IDLE].name,        "IDLE",   MAX_TASK_NAME_LEN);
	strncpy(g_rtos.task[EVB_TASK_TIMER].name,       "TIMER",  MAX_TASK_NAME_LEN);
	strncpy(g_rtos.task[EVB_TASK_SPI_UINS_COM].name,"INSSPI", MAX_TASK_NAME_LEN);

#ifdef ENABLE_WDT
	// Setup Watchdog
	uint32_t timeout_value = wdt_get_timeout_value(1000000, BOARD_FREQ_SLCK_XTAL);	//Timeout in us, configured for 1 second.
	wdt_init(WDT, WDT_MR_WDRSTEN | WDT_MR_WDDBGHLT, timeout_value, timeout_value);
#endif
}


int evbMain(void)
{	
	// Start the scheduler
	printf("Starting FreeRTOS\n\r");
	vTaskStartScheduler((TaskHandle_t*)&g_rtos.task[EVB_TASK_IDLE].handle, (TaskHandle_t*)&g_rtos.task[EVB_TASK_TIMER].handle);

	// Will only get here if there was insufficient memory to create the idle task.
    return 0;
}




