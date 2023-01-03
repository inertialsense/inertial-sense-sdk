/**
 * @file rtos_static.c
 * @brief FreeRTOS helper functions using static allocation only
 * 
 * @copyright Copyright (c) 2022 Inertial Sense, Inc.
 * 
 */

#include "rtos_static.h"
#include "debug_gpio.h"

#include <stdint.h>

extern rtos_info_t g_rtos;

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

void vApplicationIdleHook(void)
{
    // Sleep to reduce power consumption
    SCB->SCR &= (uint32_t) ~SCB_SCR_SLEEPDEEP_Msk;	// TODO: Make sure this is right for CM33
    __DSB();
    __WFI();
}

void vApplicationTickHook(void)
{
    DBGPIO_TOGGLE(DBG_RTOS_TICK_HOOK_PIN);  // Debug used to monitor RTOS tick execution
}

void vApplicationMallocFailedHook( void )
{
	while(1);
}

void vApplicationStackOverflowHook( TaskHandle_t xTask, char *pcTaskName )
{
	while(1);
}
