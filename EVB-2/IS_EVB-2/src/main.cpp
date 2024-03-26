/*
MIT LICENSE

Copyright 2014-2019 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT, IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/
 
#include "evb_tasks.h"


static void vTaskComm(void *pvParameters)
{
	rtos_task_t &task = g_rtos.task[EVB_TASK_COMMUNICATIONS];
    is_comm_instance_t &comm = evbTaskCommInit(pvParameters);
	
	while(1)
	{
		// EVB communications task 
		evbTaskComm(task, comm);

		//////////////////////////////////////////////////////////////////////////
		// Suggested USER CODE Section
		// Update period:	1ms			(Adjust by changing TASK_COMM_PERIOD_MS)
		// Priority:		high
		//
		// Ensure code added here does not run longer than 1ms.  Consider
		// adding code to vTaskMaint if it runs longer than 1ms and does not
		// require a high priority.
    
		// Add code here...
		//////////////////////////////////////////////////////////////////////////
	}
}


/**
 * \brief RTOS logger task.
 */
static void vTaskLogger(void *pvParameters)
{
	rtos_task_t &task = g_rtos.task[EVB_TASK_LOGGER];
    is_comm_instance_t &comm = evbTaskLoggerInit(pvParameters);

	while(1)
	{
		// EVB logger task
		evbTaskLogger(task, comm);
    }        
}


/**
 * \brief RTOS maintenance task.
 */
static void vTaskMaint(void *pvParameters)
{
	rtos_task_t &task = g_rtos.task[EVB_TASK_MAINTENANCE];
	evbTaskMaintInit(pvParameters);

	for (;;)
	{    
		if(evbTaskMaint(task))
		{
	        nvr_slow_maintenance();

			//////////////////////////////////////////////////////////////////////////
			// Slow Maintenance - 1000ms period - Suggested USER CODE Section
			// Update period:	1000ms		(Adjust by changing TASK_MAINT_SUB_TASK_PERIOD_MS)
			// Priority:		low
			//
			// Consider adding code to vTaskComm if it needs to run faster than every
			// 10ms or requires a higher priority.
    
			// Add code here...
			//////////////////////////////////////////////////////////////////////////		
			
			//Test code for ADC
			// g_debug.f[0] = adc_voltage();
		}
		
		//////////////////////////////////////////////////////////////////////////
		// Fast Maintenance - 10ms period - Suggested USER CODE Section
		// Update period:	10ms		(Adjust by changing TASK_MAINT_PERIOD_MS)
		// Priority:		low
		//
		// Consider adding code to vTaskComm if it needs to run faster than every
		// 10ms or requires a higher priority.
		    
		// Add code here...
		//////////////////////////////////////////////////////////////////////////
	}
}


int main(void)
{
	evbMainInitBoard();
	evbMainInitNvr();
	evbMainInitIO();
	evbMainInitComm();
	evbMainInitRTOS(vTaskComm, vTaskLogger, vTaskWiFi, vTaskMaint);
	
	int result = evbMain();
	
	// Will only get here if there was insufficient memory to create the idle task.
	return result;
}
