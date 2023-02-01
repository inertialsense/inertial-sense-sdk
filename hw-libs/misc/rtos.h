/*
MIT LICENSE

Copyright 2014-2019 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT, IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#ifndef __RTOS_STATIC_H_
#define __RTOS_STATIC_H_

#ifndef BOOTLOADER

#ifdef __cplusplus
extern "C" {
#endif

#include "FreeRTOS.h"
#include "task.h"
#include "data_sets.h"
#include "globals.h"

#if defined(__INERTIAL_SENSE_EVB_2__)
#define RTOS_NUM_TASKS	(EVB_RTOS_NUM_TASKS)
#elif defined(GPX_1)
#define RTOS_NUM_TASKS	(GPX_RTOS_NUM_TASKS)
#else
#define RTOS_NUM_TASKS	(IMX_RTOS_NUM_TASKS)
#endif

#if defined(DBGPIO_START) && defined(DBGPIO_END)
#define BEGIN_CRITICAL_SECTION	{vTaskSuspendAll(); taskENTER_CRITICAL(); DBGPIO_START(DBG_CRITICAL_SECTION_PIN);}
#define END_CRITICAL_SECTION	{DBGPIO_END(DBG_CRITICAL_SECTION_PIN); taskEXIT_CRITICAL(); xTaskResumeAll();}
#else
#define BEGIN_CRITICAL_SECTION	{vTaskSuspendAll(); taskENTER_CRITICAL();}
#define END_CRITICAL_SECTION	{taskEXIT_CRITICAL(); xTaskResumeAll();}
#endif

#if configSUPPORT_DYNAMIC_ALLOCATION
int createTask(
	int index,
	pdTASK_CODE pxTaskCode,
	const char * const pcName,
	unsigned short usStackDepth,
	void *pvParameters,
	unsigned portBASE_TYPE uxPriority,
	portTickType xTimeIncrement
);
#endif

#if configSUPPORT_STATIC_ALLOCATION
int createTaskStatic(
	int index,
	TaskFunction_t pxTaskCode,
	const char * const pcName,
	unsigned short usStackDepth,
	void *pvParameters,
	unsigned portBASE_TYPE uxPriority,
	TickType_t xTimeIncrement,
    StackType_t *const puxStackBuffer,
    StaticTask_t *const pxTaskBuffer
);
#endif

void rtos_monitor(int numRtosTasks);
void rtosResetTaskCounters(void);
void rtosResetStats(void);

#define GPBR_IDX_STATUS             0
#define GPBR_IDX_G1_TASK            1
#define GPBR_IDX_G2_FILE_NUM        2
#define GPBR_IDX_G3_LINE_NUM        3
#define GPBR_IDX_G4_FLASH_MIG       4
#define GPBR_IDX_G5_LR              5
#define GPBR_IDX_PC                 6
#define GPBR_IDX_PSR                7

extern uint32_t g_faultLineNumber;
extern uint32_t g_faultFileNumber;

#ifdef __cplusplus
}
#endif

#endif 	// BOOTLOADER

#endif 	// __RTOS_STATIC_H_
