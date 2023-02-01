#include "rtos.h"
#include "conf_debug.h"
#include "bootloaderApp.h"
#include "ISBoards.h"

static void setGpbrWithTaskInfo(void);

uint32_t g_faultLineNumber;
uint32_t g_faultFileNumber;

static void setGpbrWithTaskInfo(void)
{
    uint32_t task;
    if((uint32_t)xTaskGetCurrentTaskHandle() == (uint32_t)g_rtos.task[TASK_SAMPLE].handle){ task = TASK_SAMPLE; }
    if((uint32_t)xTaskGetCurrentTaskHandle() == (uint32_t)g_rtos.task[TASK_NAV].handle){ task = TASK_NAV; }
    if((uint32_t)xTaskGetCurrentTaskHandle() == (uint32_t)g_rtos.task[TASK_COMMUNICATIONS].handle){ task = TASK_COMMUNICATIONS; }
    if((uint32_t)xTaskGetCurrentTaskHandle() == (uint32_t)g_rtos.task[TASK_MAINTENANCE].handle){ task = TASK_MAINTENANCE; }

#if !defined(IMX_5) && !defined(GPX_1)	// not STM32
    GPBR->SYS_GPBR[GPBR_IDX_G1_TASK] = task;
    GPBR->SYS_GPBR[GPBR_IDX_G2_FILE_NUM] = g_faultFileNumber;
    GPBR->SYS_GPBR[GPBR_IDX_G3_LINE_NUM] = g_faultLineNumber;
#else
    BKUP_PERIPH->BKP1R = task;
    BKUP_PERIPH->BKP2R = g_faultFileNumber;
    BKUP_PERIPH->BKP3R = g_faultLineNumber;
#endif
}

void MemManage_Handler(void) 
{	
#if defined(PLATFORM_IS_EVB_2) || defined(DEBUG)

    for (;;) { }

#else  

#if !defined(IMX_5) && !defined(GPX_1)	// not STM32
    GPBR->SYS_GPBR[GPBR_IDX_STATUS] |= SYS_FAULT_STATUS_MEM_MANGE;
#else
    BKUP_PERIPH->BKP0R |= SYS_FAULT_STATUS_MEM_MANGE;
#endif

    setGpbrWithTaskInfo();
    soft_reset_no_backup_register();

#endif
}

void BusFault_Handler(void) 
{ 
#if defined(PLATFORM_IS_EVB_2) || defined(DEBUG)

    for (;;) { }

#else  

#if !defined(IMX_5) && !defined(GPX_1)	// not STM32
    GPBR->SYS_GPBR[GPBR_IDX_STATUS] |= SYS_FAULT_STATUS_BUS_FAULT;
#else
    BKUP_PERIPH->BKP0R |= SYS_FAULT_STATUS_BUS_FAULT;
#endif

    setGpbrWithTaskInfo();
    soft_reset_no_backup_register();

#endif
}

void UsageFault_Handler(void) 
{ 
#if defined(PLATFORM_IS_EVB_2) || defined(DEBUG)

    for (;;) { }

#else  

#if !defined(IMX_5) && !defined(GPX_1)	// not STM32
    GPBR->SYS_GPBR[GPBR_IDX_STATUS] |= SYS_FAULT_STATUS_USAGE_FAULT;
#else
    BKUP_PERIPH->BKP0R |= SYS_FAULT_STATUS_USAGE_FAULT;
#endif

    setGpbrWithTaskInfo();
    soft_reset_no_backup_register();

#endif
}

void prvGetRegistersFromStack(uint32_t *pulFaultStackAddress)
{
    setGpbrWithTaskInfo();
#if !defined(IMX_5) && !defined(GPX_1)	// not STM32
    GPBR->SYS_GPBR[GPBR_IDX_G5_LR]  = pulFaultStackAddress[5]; // link reg
    GPBR->SYS_GPBR[GPBR_IDX_PC]  = pulFaultStackAddress[6]; // program counter
    GPBR->SYS_GPBR[GPBR_IDX_PSR] = pulFaultStackAddress[7]; // program status register
#else
    BKUP_PERIPH->BKP5R = pulFaultStackAddress[5]; // link reg
    BKUP_PERIPH->BKP6R = pulFaultStackAddress[6]; // program counter
    BKUP_PERIPH->BKP7R = pulFaultStackAddress[7]; // program status register
#endif

    soft_reset_no_backup_register();
}

void HardFault_Handler(void)
{
#if defined(PLATFORM_IS_EVB_2) || defined(DEBUG)

    for (;;) { }

#else

#if !defined(IMX_5) && !defined(GPX_1)	// not STM32
    GPBR->SYS_GPBR[GPBR_IDX_STATUS] |= SYS_FAULT_STATUS_HARD_FAULT;
#else
    BKUP_PERIPH->BKP0R |= SYS_FAULT_STATUS_HARD_FAULT;
#endif
    
	__ASM volatile
	(
		" tst lr, #4                                                        \n"
		" ite eq                                                            \n"
		" mrseq r0, msp                                                     \n"
		" mrsne r0, psp                                                     \n"
		" ldr r1, [r0, #24]                                                 \n"
		" ldr r2, handler_address_const_hard_fault                          \n"
		" bx r2                                                             \n"
		" handler_address_const_hard_fault: .word prvGetRegistersFromStack  \n"
	);

#endif
}

void vApplicationStackOverflowHook(TaskHandle_t pxTask, char* pcTaskName)
{
#if defined(PLATFORM_IS_EVB_2) || defined(DEBUG)

    for (;;) { }

#else

#if !defined(IMX_5) && !defined(GPX_1)	// not STM32
	GPBR->SYS_GPBR[GPBR_IDX_STATUS] |= SYS_FAULT_STATUS_STACK_OVERFLOW;
#else
	BKUP_PERIPH->BKP0R |= SYS_FAULT_STATUS_STACK_OVERFLOW;
#endif

	setGpbrWithTaskInfo();
	soft_reset_no_backup_register();

#endif    
}

void vApplicationTickHook(void)
{
#if !defined(PLATFORM_IS_EVB_2) && !defined(TESTBED)
    DBGPIO_TOGGLE(DBG_RTOS_TICK_HOOK_PIN);  // Debug used to monitor RTOS tick execution
#endif
}

void vApplicationIdleHook(void)
{
    // Sleep core to reduce power consumption
#if !defined(IMX_5) && !defined(GPX_1)	// not STM32
	PMC->PMC_FSMR &= (uint32_t) ~PMC_FSMR_LPM; // Enter Sleep mode
#endif
	// TODO: Make sure this is right for Cortex-M33. Same works on CM4F and CM7 already.
    SCB->SCR &= (uint32_t) ~SCB_SCR_SLEEPDEEP_Msk;
    __DSB();
    __WFI();
}

void vApplicationDaemonTaskStartupHook(void)
{
	g_rtos.task[TASK_TIMER].handle = (uint32_t)xTaskGetCurrentTaskHandle();
}

void vApplicationMallocFailedHook(uint32_t size, uint32_t remaining, uint32_t prevLR)
{
#if defined(PLATFORM_IS_EVB_2) || defined(DEBUG)
	for (;;) {}
#else

#if !defined(IMX_5) && !defined(GPX_1)	// not STM32
    GPBR->SYS_GPBR[GPBR_IDX_STATUS] |= SYS_FAULT_STATUS_MALLOC_FAILED;
    GPBR->SYS_GPBR[GPBR_IDX_G4_FLASH_MIG] = size;
    GPBR->SYS_GPBR[GPBR_IDX_G5_LR] = remaining;

	// Capture call stack
    GPBR->SYS_GPBR[GPBR_IDX_PC]  = prevLR;		// program counter of call to malloc
#else
    BKUP_PERIPH->BKP0R |= SYS_FAULT_STATUS_MALLOC_FAILED;
    BKUP_PERIPH->BKP4R = size;
    BKUP_PERIPH->BKP5R = remaining;

	// Capture call stack
    BKUP_PERIPH->BKP6R = prevLR;		// program counter of call to malloc
#endif

	soft_reset_no_backup_register();

#endif
}

/*
 * Call this from the timer overflow interrupt to reset everything at the same time.
 */
void rtosResetTaskCounters(void)
{
#if (configGENERATE_RUN_TIME_STATS == 1)
	for (int i=0; i<RTOS_NUM_TASKS; i++)
	{
		void* handle = (void*)g_rtos.task[i].handle;
		if (handle)
		{
			vTaskResetRunTimeCounter(handle);
		}
	}
#endif // (configGENERATE_RUN_TIME_STATS == 1)
}

void rtosResetStats(void)
{
	for (size_t i = 0; i < RTOS_NUM_TASKS; i++)
	{
		g_rtos.task[i].maxRunTimeUs = 0;
	}
}

void rtos_monitor(int numRtosTasks)
{
	int i;

#if (configGENERATE_RUN_TIME_STATS == 1)
	uint32_t ulTotalRunTime = portGET_RUN_TIME_COUNTER_VALUE();			// uint64_t gets truncated to uint32_t
	float fTotalRunTime = ((float)ulTotalRunTime) * 1e-2;				// Percentage, so divide by 100
#endif // (configGENERATE_RUN_TIME_STATS == 1)

	TaskStatus_t status;

	for (i=0; i<numRtosTasks; i++)
	{
		// If updating free rtos, the idle and timer handles need to be passed through and set properly
		// tasks.h: void vTaskStartScheduler( TaskHandle_t* idleTaskHandle, TaskHandle_t* timerTaskHandle ) PRIVILEGED_FUNCTION;
		// timers.h: BaseType_t xTimerCreateTimerTask( TaskHandle_t* ) PRIVILEGED_FUNCTION;

		void* handle = (void*)g_rtos.task[i].handle;
		if (handle)
		{
			vTaskGetInfo(handle, &status, 1, eRunning);
			g_rtos.task[i].stackUnused = status.usStackHighWaterMark * sizeof(uint32_t);
			g_rtos.task[i].priority    = status.uxCurrentPriority;

#if (configGENERATE_RUN_TIME_STATS == 1)
			if (ulTotalRunTime) // Divide by zero
			{
				g_rtos.task[i].cpuUsage = (float)status.ulRunTimeCounter / fTotalRunTime;
			}
#endif // (configGENERATE_RUN_TIME_STATS == 1)
		}
	}

	g_rtos.freeHeapSize = xPortGetMinimumEverFreeHeapSize();
}

#if configSUPPORT_DYNAMIC_ALLOCATION == 1
int createTask
(
	int index,
	pdTASK_CODE pxTaskCode,
	const char * const pcName,
	unsigned short usStackDepth,
	void *pvParameters,
	unsigned portBASE_TYPE uxPriority,
	portTickType xTimeIncrement
)
{
	if (!(index < RTOS_NUM_TASKS))
	{
		return -1;
	}

	// Task call period - used within task and for CPU usage
	g_rtos.task[index].periodMs = xTimeIncrement;
#ifndef __INERTIAL_SENSE_EVB_2__
	g_rtos_pro[index].periodTicks = time_usec_to_ticks(xTimeIncrement * 1000U);
#endif

	// Task name
	if (MAX_TASK_NAME_LEN > configMAX_TASK_NAME_LEN)
	{
		memset(&g_rtos.task[index].name[configMAX_TASK_NAME_LEN], 0, MAX_TASK_NAME_LEN - configMAX_TASK_NAME_LEN);
	}
	strncpy(g_rtos.task[index].name, pcName, configMAX_TASK_NAME_LEN);

	// Create RTOS Task
	if (xTaskCreate(pxTaskCode, pcName, usStackDepth, pvParameters, uxPriority, (TaskHandle_t * const)&g_rtos.task[index].handle) != pdPASS)
	{
		return -1;
	}
	return 0;
}
#endif

#if configSUPPORT_STATIC_ALLOCATION == 1
int createTaskStatic
(
	int index,
	TaskFunction_t pxTaskCode,
	const char * const pcName,
	unsigned short usStackDepth,
	void *pvParameters,
	unsigned portBASE_TYPE uxPriority,
	TickType_t xTimeIncrement,
    StackType_t *const puxStackBuffer,
    StaticTask_t *const pxTaskBuffer
)
{
	if (index >= RTOS_NUM_TASKS)
	{
		return -1;
	}

	// Task call period - used within task and for CPU usage
	g_rtos.task[index].periodMs = xTimeIncrement;

	g_rtos_pro[index].periodTicks = time_usec_to_ticks(xTimeIncrement * 1000U);

	// Task name
	if (MAX_TASK_NAME_LEN > configMAX_TASK_NAME_LEN)
	{
		memset(&g_rtos.task[index].name[configMAX_TASK_NAME_LEN], 0, MAX_TASK_NAME_LEN - configMAX_TASK_NAME_LEN);
	}
	strncpy(g_rtos.task[index].name, pcName, configMAX_TASK_NAME_LEN);

	// Create RTOS Task
	g_rtos.task[index].handle = (uint32_t)xTaskCreateStatic(pxTaskCode, pcName, usStackDepth, pvParameters, uxPriority, puxStackBuffer, pxTaskBuffer);

	return 0;
}

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer,
                                    StackType_t **ppxIdleTaskStackBuffer,
                                    uint32_t *pulIdleTaskStackSize )
{
    static StaticTask_t xIdleTaskTCB;
    static StackType_t uxIdleTaskStack[ configMINIMAL_STACK_SIZE ];

    /* Pass out a pointer to the StaticTask_t structure in which the Idle task's
    state will be stored. */
    *ppxIdleTaskTCBBuffer = &xIdleTaskTCB;

    /* Pass out the array that will be used as the Idle task's stack. */
    *ppxIdleTaskStackBuffer = uxIdleTaskStack;

    /* Pass out the size of the array pointed to by *ppxIdleTaskStackBuffer.
    Note that, as the array is necessarily of type StackType_t,
    configMINIMAL_STACK_SIZE is specified in words, not bytes. */
    *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
}
#endif
