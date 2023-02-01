/*
MIT LICENSE

Copyright 2014-2019 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT, IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include "rtos_static.h"
#include "conf_debug.h"
#include "globals.h"

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
