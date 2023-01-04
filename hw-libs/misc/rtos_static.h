/**
 * @file rtos_static.h
 * @brief FreeRTOS helper functions using static allocation only
 * 
 * @copyright Copyright (c) 2022 Inertial Sense, Inc.
 * 
 */

#ifndef __RTOS_STATIC_H_
#define __RTOS_STATIC_H_
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
#define RTOS_NUM_TASKS	(SN_RTOS_NUM_TASKS)
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
#endif
