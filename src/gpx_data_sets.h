/*
MIT LICENSE

Copyright (c) 2014-2022 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#ifndef GPX_DATA_SETS_H
#define GPX_DATA_SETS_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** RTOS tasks */
typedef enum
{
	/** Task 0: Communications	*/
	GPX_TASK_COMM = 0,

	/** Task 1: Nav */
//	GPX_TASK_RTK,
//
//	/** Task 2: Maintenance */
//	GPX_TASK_MAINT,
//
//	/** Task 3: Idle */
//	GPX_TASK_IDLE,
//
//	/** Task 4: Timer */
//	GPX_TASK_TIMER,

	/** Number of RTOS tasks */
	GPX_RTOS_NUM_TASKS                 // Keep last
} eGpxRtosTask;

/** Max task name length - do not change */
#define GPX_MAX_TASK_NAME_LEN 12

/** RTOS task info */
typedef struct PACKED
{
	/** Task name */
	char                    name[GPX_MAX_TASK_NAME_LEN];

	/** Task priority (0 - 8) */
	uint32_t                priority;

	/** Stack high water mark bytes */
	uint32_t                stackUnused;

	/** Task period ms */
	uint32_t                periodMs;

	/** Last run time microseconds */
	uint32_t                runTimeUs;

	/** Max run time microseconds */
	uint32_t                maxRunTimeUs;
	
	/** Rolling average over last 1000 executions */
	float					averageRunTimeUs;
	
	/** Counter of times task took too long to run */
	uint32_t				gapCount;

	/** Cpu usage percent */
    float					cpuUsage;

	/** Handle */
	uint32_t                handle;

	/** Local time when task loop started (following delay) */
	uint32_t                profileStartTimeUs;

    /** Task info for static memory allocation */
    StaticTask_t            taskBuffer;
	
} gpx_rtos_task_t;

typedef struct PACKED
{
    /** Heap high water mark bytes */
    uint32_t                freeHeapSize;

    /** Total memory allocated using RTOS pvPortMalloc() */
    uint32_t				mallocSize;
    
	/** Total memory freed using RTOS vPortFree() */
	uint32_t				freeSize;

	/** Tasks */
	gpx_rtos_task_t         task[GPX_RTOS_NUM_TASKS];

} gpx_rtos_info_t;

#ifdef __cplusplus
}
#endif

#endif  // GPX_DATA_SETS_H

