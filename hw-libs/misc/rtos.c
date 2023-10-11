/*
MIT LICENSE

Copyright 2014-2019 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT, IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include "FreeRTOS.h"
#include "task.h"
#if ((configUSE_TIMERS == 1) && (configSUPPORT_STATIC_ALLOCATION == 1))
#include "timers.h"
#endif
#include "rtos.h"

#if !defined(PLATFORM_IS_EVB_2) && !defined(TESTBED)
#include "globals.h"
#include "IS_internal.h"
#endif

#ifdef TESTBED
#include "globals.h"
#endif

#include "bootloaderApp.h"

uint32_t g_faultLineNumber;
uint32_t g_faultFileNumber;


int createTask
(
	int index,
	pdTASK_CODE pxTaskCode,
	const char * const pcName,
	unsigned short usStackDepth,
	void *pvParameters,
	unsigned portBASE_TYPE uxPriority,
	portTickType xTimeIncrement,
	uint32_t overrunMs
)
{
	if (!(index < RTOS_NUM_TASKS))
	{
		return -1;
	}

	// Task call period - used within task and for CPU usage
	g_rtos.task[index].periodMs = xTimeIncrement;
#ifndef __INERTIAL_SENSE_EVB_2__
	g_rtos_pro[index].periodTicks = time_usec_to_ticks(overrunMs * 1000U);
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


void rtosResetStats(void)
{
	for (size_t i = 0; i < RTOS_NUM_TASKS; i++)
	{
		g_rtos.task[i].maxRuntimeUs = 0;
		g_rtos.task[i].gapCount = 0;
		g_rtos.task[i].doubleGapCount = 0;
#ifndef __INERTIAL_SENSE_EVB_2__
		g_rtos_pro[i].maxRuntimeTicks = 0;
		g_rtos_pro[i].gapCount = 0;
		g_rtos_pro[i].doubleGapCount = 0;
#endif
	}
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

void vApplicationIdleHook(void)
{
    // Sleep to reduce power consumption
#ifndef IMX_5
	PMC->PMC_FSMR &= (uint32_t) ~PMC_FSMR_LPM; // Enter Sleep mode
#endif
    SCB->SCR &= (uint32_t) ~SCB_SCR_SLEEPDEEP_Msk;	// Common to both Cortex M4 and M7
    __DSB();
    __WFI();
}

#ifndef IMX_5
void vApplicationTickHook(void)
{
#if !defined(PLATFORM_IS_EVB_2) && !defined(TESTBED)
    DBGPIO_TOGGLE(DBG_RTOS_TICK_HOOK_PIN);  // Debug used to monitor RTOS tick execution
#endif
}
#endif

static void setGpbrWithTaskInfo(void)
{
    uint32_t task;
    if((uint32_t)xTaskGetCurrentTaskHandle() == (uint32_t)g_rtos.task[TASK_SAMPLE].handle){ task = TASK_SAMPLE; }
    if((uint32_t)xTaskGetCurrentTaskHandle() == (uint32_t)g_rtos.task[TASK_NAV].handle){ task = TASK_NAV; }
    if((uint32_t)xTaskGetCurrentTaskHandle() == (uint32_t)g_rtos.task[TASK_COMMUNICATIONS].handle){ task = TASK_COMMUNICATIONS; }
    if((uint32_t)xTaskGetCurrentTaskHandle() == (uint32_t)g_rtos.task[TASK_MAINTENANCE].handle){ task = TASK_MAINTENANCE; }

#ifndef IMX_5
    GPBR->SYS_GPBR[GPBR_IDX_G1_TASK] = task;
    GPBR->SYS_GPBR[GPBR_IDX_G2_FILE_NUM] = g_faultFileNumber;
    GPBR->SYS_GPBR[GPBR_IDX_G3_LINE_NUM] = g_faultLineNumber;
#else
    RTC->BKP1R = task;
    RTC->BKP2R = g_faultFileNumber;
    RTC->BKP3R = g_faultLineNumber;
#endif
}

// implementation below acquired from
// http://www.freertos.org/Debugging-Hard-Faults-On-Cortex-M-Microcontrollers.html
void prvGetRegistersFromStack(uint32_t *pulFaultStackAddress);
void prvGetRegistersFromStack(uint32_t *pulFaultStackAddress)
{
    setGpbrWithTaskInfo();
#ifndef IMX_5
    GPBR->SYS_GPBR[GPBR_IDX_G5_LR]  = pulFaultStackAddress[5]; // link reg
    GPBR->SYS_GPBR[GPBR_IDX_PC]  = pulFaultStackAddress[6]; // program counter
    GPBR->SYS_GPBR[GPBR_IDX_PSR] = pulFaultStackAddress[7]; // program status register
#else
    RTC->BKP5R = pulFaultStackAddress[5]; // link reg
    RTC->BKP6R = pulFaultStackAddress[6]; // program counter
    RTC->BKP7R = pulFaultStackAddress[7]; // program status register
#endif

    soft_reset_no_backup_register();
}

void vApplicationStackOverflowHook(TaskHandle_t pxTask, char* pcTaskName)
{
#if defined(PLATFORM_IS_EVB_2) || defined(DEBUG)

//   	printf("stack overflow %x %s\r\n", (unsigned int)pxTask, (portCHAR *)pcTaskName);
    for (;;) { }

#else   // uINS

#ifndef IMX_5
	GPBR->SYS_GPBR[GPBR_IDX_STATUS] |= SYS_FAULT_STATUS_STACK_OVERFLOW;
#else
	RTC->BKP0R |= SYS_FAULT_STATUS_STACK_OVERFLOW;
#endif

	setGpbrWithTaskInfo();
	soft_reset_no_backup_register();

#endif    
}


void vApplicationMallocFailedHook(uint32_t size, uint32_t remaining, uint32_t prevLR)
{
#if defined(PLATFORM_IS_EVB_2) || defined(DEBUG)

	/* Called if a call to pvPortMalloc() fails because there is insufficient
	free memory available in the FreeRTOS heap.  pvPortMalloc() is called
	internally by FreeRTOS API functions that create tasks, queues, software
	timers, and semaphores.  The size of the FreeRTOS heap is set by the
	configTOTAL_HEAP_SIZE configuration constant in FreeRTOSConfig.h. */

	/* Force an assert. */
//   	printf("stack overflow %x %s\r\n", (unsigned int)pxTask, (portCHAR *)pcTaskName);
// 	configASSERT( ( volatile void * ) NULL );
    for (;;) { }

#else   // uINS

#ifndef IMX_5
    GPBR->SYS_GPBR[GPBR_IDX_STATUS] |= SYS_FAULT_STATUS_MALLOC_FAILED;
    GPBR->SYS_GPBR[GPBR_IDX_G4_FLASH_MIG] = size;
    GPBR->SYS_GPBR[GPBR_IDX_G5_LR] = remaining;

	// Capture call stack
    GPBR->SYS_GPBR[GPBR_IDX_PC]  = prevLR;		// program counter of call to malloc
#else
    RTC->BKP0R |= SYS_FAULT_STATUS_MALLOC_FAILED;
    RTC->BKP4R = size;
    RTC->BKP5R = remaining;

	// Capture call stack
    RTC->BKP6R = prevLR;		// program counter of call to malloc
#endif

	soft_reset_no_backup_register();

#endif
}

void vApplicationDaemonTaskStartupHook(void)
{
	g_rtos.task[TASK_TIMER].handle = (uint32_t)xTaskGetCurrentTaskHandle();
}

#if (configSUPPORT_STATIC_ALLOCATION == 1)
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

#if (configUSE_TIMERS == 1)
/* configSUPPORT_STATIC_ALLOCATION and configUSE_TIMERS are both set to 1, so the
application must provide an implementation of vApplicationGetTimerTaskMemory()
to provide the memory that is used by the Timer service task. */
void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer,
                                     StackType_t **ppxTimerTaskStackBuffer,
                                     uint32_t *pulTimerTaskStackSize )
{
	/* If the buffers to be provided to the Timer task are declared inside this
	function then they must be declared static - otherwise they will be allocated on
	the stack and so not exists after this function exits. */
	static StaticTask_t xTimerTaskTCB;
	static StackType_t uxTimerTaskStack[ configTIMER_TASK_STACK_DEPTH ];

    /* Pass out a pointer to the StaticTask_t structure in which the Timer
    task's state will be stored. */
    *ppxTimerTaskTCBBuffer = &xTimerTaskTCB;

    /* Pass out the array that will be used as the Timer task's stack. */
    *ppxTimerTaskStackBuffer = uxTimerTaskStack;

    /* Pass out the size of the array pointed to by *ppxTimerTaskStackBuffer.
    Note that, as the array is necessarily of type StackType_t,
    configTIMER_TASK_STACK_DEPTH is specified in words, not bytes. */
    *pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
}
#endif
#endif

#if 1
void MemManage_Handler(void) 
{	
#if defined(PLATFORM_IS_EVB_2) || defined(DEBUG)

    for (;;) { }

#else   // uINS

#ifndef IMX_5
    GPBR->SYS_GPBR[GPBR_IDX_STATUS] |= SYS_FAULT_STATUS_MEM_MANGE;
#else
    RTC->BKP0R |= SYS_FAULT_STATUS_MEM_MANGE;
#endif

    setGpbrWithTaskInfo();
    soft_reset_no_backup_register();

#endif
}

void BusFault_Handler(void) 
{ 
#if defined(PLATFORM_IS_EVB_2) || defined(DEBUG)

    for (;;) { }

#else   // uINS

#ifndef IMX_5
    GPBR->SYS_GPBR[GPBR_IDX_STATUS] |= SYS_FAULT_STATUS_BUS_FAULT;
#else
    RTC->BKP0R |= SYS_FAULT_STATUS_BUS_FAULT;
#endif

    setGpbrWithTaskInfo();
    soft_reset_no_backup_register();

#endif
}

void UsageFault_Handler(void) 
{ 
#if defined(PLATFORM_IS_EVB_2) || defined(DEBUG)

    for (;;) { }

#else   // uINS

#ifndef IMX_5
    GPBR->SYS_GPBR[GPBR_IDX_STATUS] |= SYS_FAULT_STATUS_USAGE_FAULT;
#else
    RTC->BKP0R |= SYS_FAULT_STATUS_USAGE_FAULT;
#endif

    setGpbrWithTaskInfo();
    soft_reset_no_backup_register();

#endif
}
#endif

#if 1

#pragma GCC push_options
#pragma GCC optimize ("O0")

void HardFault_Handler(void)
{
#if defined(PLATFORM_IS_EVB_2) || defined(DEBUG)

    for (;;) { }

#else   // uINS

#ifndef IMX_5
    GPBR->SYS_GPBR[GPBR_IDX_STATUS] |= SYS_FAULT_STATUS_HARD_FAULT;
#else
    RTC->BKP0R |= SYS_FAULT_STATUS_HARD_FAULT;
#endif
    
	__asm volatile
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


#else

// implementation below acquired from
// https://mcuoneclipse.com/2012/11/24/debugging-hard-faults-on-arm-cortex-m/
/**
 * HardFaultHandler_C:
 * This is called from the HardFault_HandlerAsm with a pointer the Fault stack
 * as the parameter. We can then read the values from the stack and place them
 * into local variables for ease of reading.
 * We then read the various Fault Status and Address Registers to help decode
 * cause of the fault.
 * The function ends with a BKPT instruction to force control back into the debugger
 */
void HardFault_HandlerC(unsigned long *hardfault_args){
	volatile unsigned long stacked_r0;
	volatile unsigned long stacked_r1;
	volatile unsigned long stacked_r2;
	volatile unsigned long stacked_r3;
	volatile unsigned long stacked_r12;
	volatile unsigned long stacked_lr;
	volatile unsigned long stacked_pc;
	volatile unsigned long stacked_psr;
	volatile unsigned long _CFSR;
	volatile unsigned long _HFSR;
	volatile unsigned long _DFSR;
	volatile unsigned long _AFSR;
	volatile unsigned long _BFAR;
	volatile unsigned long _MMAR;

	stacked_r0 = ((unsigned long)hardfault_args[0]);
	stacked_r1 = ((unsigned long)hardfault_args[1]);
	stacked_r2 = ((unsigned long)hardfault_args[2]);
	stacked_r3 = ((unsigned long)hardfault_args[3]);
	stacked_r12 = ((unsigned long)hardfault_args[4]);
	stacked_lr = ((unsigned long)hardfault_args[5]);
	stacked_pc = ((unsigned long)hardfault_args[6]);
	stacked_psr = ((unsigned long)hardfault_args[7]);

	// Configurable Fault Status Register
	// Consists of MMSR, BFSR and UFSR
	_CFSR = (*((volatile unsigned long *)(0xE000ED28)));

	// Hard Fault Status Register
	_HFSR = (*((volatile unsigned long *)(0xE000ED2C)));

	// Debug Fault Status Register
	_DFSR = (*((volatile unsigned long *)(0xE000ED30)));

	// Auxiliary Fault Status Register
	_AFSR = (*((volatile unsigned long *)(0xE000ED3C)));

	// Read the Fault Address Registers. These may not contain valid values.
	// Check BFARVALID/MMARVALID to see if they are valid values
	// MemManage Fault Address Register
	_MMAR = (*((volatile unsigned long *)(0xE000ED34)));
	// Bus Fault Address Register
	_BFAR = (*((volatile unsigned long *)(0xE000ED38)));

	__asm("BKPT #0\n"); // Break into the debugger
}
 

void HardFault_Handler(void)
{
    __asm volatile
    (
        " movs r0,#4          \n"
        " movs r1, lr         \n"
        " tst r0, r1          \n"
        " beq _MSP            \n"
        " mrs r0, psp         \n"
        " b _HALT             \n"
        "_MSP:                \n"
        " mrs r0, msp         \n"
        "_HALT:               \n"
        " ldr r1,[r0,#20]     \n"
        " b HardFault_HandlerC\n"
        " bkpt #0             \n"
    );
}

#endif // 0


#pragma GCC pop_options

#ifndef IMX_5
static void HardFault_Test(void)
{
#if 0
#define TESTCASE 0
#if TESTCASE == 0
	volatile uint8_t* ptr = (volatile uint8_t*)0x30000000; // access memory beyond the end of ram
	volatile uint8_t val = *ptr;
	// examine the psr
	
#elif TESTCASE == 1
	typedef void (*t_funcPtr)(void);
	t_funcPtr MyFunc = (t_funcPtr)(0x0001000 | 0x0);  // INVSTATE because jumping to valid address but thumb bit not set
	MyFunc();
	// examine the pc

#elif TESTCASE == 2
	volatile int i, j = 0;
	SCB->CCR = 0x210; // enable div-by-zero trap in processor core
	i =i/j; // i and j are 0 initialized -> Div/0
#endif
#endif
	
	for (;;) {}
}
#endif


// enable to detect which handler was called as opposed to the default Dummy_Handler
#if defined(PLATFORM_IS_EVB_2)

/* Cortex-M7 core handlers */
void NMI_Handler        (void) { for (;;); }
//void HardFault_Handler  (void) { for (;;); }
// void MemManage_Handler  (void) { for (;;); }
// void BusFault_Handler   (void) { for (;;); }
// void UsageFault_Handler (void) { for (;;); }
//void SVC_Handler        (void) { for (;;); }
void DebugMon_Handler   (void) { for (;;); }
//void PendSV_Handler     (void) { for (;;); }
//void SysTick_Handler    (void) { for (;;); }

/* Peripherals handlers */
void SUPC_Handler   (void) { for (;;); }
void RSTC_Handler   (void) { for (;;); }
void RTC_Handler    (void) { for (;;); }
//void RTT_Handler    (void) { for (;;); }
void WDT_Handler    (void) { for (;;); }
void PMC_Handler    (void) { for (;;); }
void EFC_Handler    (void) { for (;;); }
//void UART0_Handler  (void) { for (;;); }
//void UART1_Handler  (void) { for (;;); }
//void USART0_Handler (void) { for (;;); }
//void USART1_Handler (void) { for (;;); }
//void USART2_Handler (void) { for (;;); }
#ifdef _SAME70_HSMCI_INSTANCE_
void HSMCI_Handler  (void) { for (;;); }
#endif /* _SAME70_HSMCI_INSTANCE_ */
//void TWIHS0_Handler (void) { for (;;); }
void TWIHS1_Handler (void) { for (;;); }
//void SPI0_Handler   (void) { for (;;); }
void SSC_Handler    (void) { for (;;); }
//void TC0_Handler    (void) { for (;;); }
//void TC1_Handler    (void) { for (;;); }
void TC2_Handler    (void) { for (;;); }
#ifdef _SAME70_TC1_INSTANCE_
void TC3_Handler    (void) { for (;;); }
#endif /* _SAME70_TC1_INSTANCE_ */
#ifdef _SAME70_TC1_INSTANCE_
void TC4_Handler    (void) { for (;;); }
#endif /* _SAME70_TC1_INSTANCE_ */
#ifdef _SAME70_TC1_INSTANCE_
// void TC5_Handler    (void) { for (;;); }
#endif /* _SAME70_TC1_INSTANCE_ */
// void AFEC0_Handler  (void) { for (;;); }
#ifdef _SAME70_DACC_INSTANCE_
void DACC_Handler   (void) { for (;;); }
#endif /* _SAME70_DACC_INSTANCE_ */
void PWM0_Handler   (void) { for (;;); }
void ICM_Handler    (void) { for (;;); }
void ACC_Handler    (void) { for (;;); }
// void USBHS_Handler  (void) { for (;;); }
void MCAN0_Handler(void);
void MCAN1_Handler(void);
void MCAN0_Handler  (void) { for (;;); }
void MCAN1_Handler  (void) { for (;;); }
void GMAC_Handler   (void) { for (;;); }
// void AFEC1_Handler  (void) { for (;;); }
#ifdef _SAME70_TWIHS2_INSTANCE_
void TWIHS2_Handler (void) { for (;;); }
#endif /* _SAME70_TWIHS2_INSTANCE_ */
// void SPI1_Handler   (void) { for (;;); }
void QSPI_Handler   (void) { for (;;); }
//void UART2_Handler  (void) { for (;;); }
//void UART3_Handler  (void) { for (;;); }
//void UART4_Handler  (void) { for (;;); }
#ifdef _SAME70_TC2_INSTANCE_
void TC6_Handler    (void) { for (;;); }
#endif /* _SAME70_TC2_INSTANCE_ */
#ifdef _SAME70_TC2_INSTANCE_
void TC7_Handler    (void) { for (;;); }
#endif /* _SAME70_TC2_INSTANCE_ */
#ifdef _SAME70_TC2_INSTANCE_
void TC8_Handler    (void) { for (;;); }
#endif /* _SAME70_TC2_INSTANCE_ */
// void TC9_Handler    (void) { for (;;); }
void TC10_Handler   (void) { for (;;); }
void TC11_Handler   (void) { for (;;); }
void AES_Handler    (void) { for (;;); }
void TRNG_Handler   (void) { for (;;); }
// void XDMAC_Handler  (void) { for (;;); }
void ISI_Handler    (void) { for (;;); }
void PWM1_Handler   (void) { for (;;); }
#ifdef _SAME70_SDRAMC_INSTANCE_
void SDRAMC_Handler (void) { for (;;); }
#endif /* _SAME70_SDRAMC_INSTANCE_ */
void RSWDT_Handler  (void) { for (;;); }
    
#endif // 0
