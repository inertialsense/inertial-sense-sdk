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

#if defined(PLATFORM_IS_EVB_2) && PLATFORM_IS_EVB_2
#define RTOS_NUM_TASKS	(EVB_RTOS_NUM_TASKS)
#elif defined (GPX_1)
//#include "gpx_data_sets.h"
#define RTOS_NUM_TASKS	(GPX_RTOS_NUM_TASKS)
#else
#define RTOS_NUM_TASKS	(UINS_RTOS_NUM_TASKS)
#endif

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
);

void rtosResetTaskCounters(void);

#ifdef __cplusplus
}
#endif

#endif 	// BOOTLOADER

#endif 	// __RTOS_STATIC_H_
