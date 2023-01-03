/*
MIT LICENSE

Copyright 2014-2019 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT, IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#ifndef D_TIME_H
#define D_TIME_H

#ifdef __cplusplus
extern "C" {
#endif

#include "ISBoards.h"

#include <stdint.h>

#if !defined(TIM_CLK_FREQ) || !defined(TIM_PRESC)
#error TIM_CLK_FREQ and TIM_PRESC must be set in ISBoards.h or its dependencies
#endif

#define TIME_TICKS_PER_SEC		(TIM_CLK_FREQ/TIM_PRESC)
#define TIME_TICKS_PER_MS       (TIME_TICKS_PER_SEC/1000U)
#define TIME_TICKS_PER_US       (TIME_TICKS_PER_SEC/1000000U)

#define TIME_SECS_PER_TICK_F   	((float)TIM_PRESC/(float)TIM_CLK_FREQ)
#define TIME_MS_PER_TICK_F     	(TIME_SECS_PER_TICK_F*1000.0f)
#define TIME_US_PER_TICK_F     	(TIME_SECS_PER_TICK_F*1000000.0f)

#if !defined(BOOTLOADER) || !defined(IMX_5)	// Don't use doubles in IMX-5 bootloader
#define TIME_SECS_PER_TICK_LF   ((double)TIM_PRESC/(double)TIM_CLK_FREQ)
#define TIME_MS_PER_TICK_LF     (TIME_SECS_PER_TICK_LF*(double)1000.0)
#define TIME_US_PER_TICK_LF     (TIME_SECS_PER_TICK_LF*(double)1000000.0)
#endif

/**
 * @brief Initialize the timebase peripheral
 * 
 */
void time_init(void);

/**
 * @brief Get the number of ticks since timebase was started as uint64_t
 * 
 * @return uint64_t number of ticks
 */
volatile uint64_t time_ticks_u64(void);

/**
 * @brief Get the number of ticks since timebase was started as uint32_t
 * @see time_ticks_u64() if you need a timer that will run for a long time
 * 
 * @return uint32_t number of ticks
 */
volatile uint32_t time_ticks_u32(void);

/**
 * @brief Convert ticks into microseconds
 * 
 * @param ticks number of ticks to convert
 * @return uint32_t usec count output, if > 2^32 will wrap to 0
 */
uint32_t time_ticks_to_usec(uint32_t ticks);

/**
 * @brief Convert usec into ticks
 * 
 * @param usec number of microseconds to convert
 * @return uint32_t tick count output, if > 2^32 will wrap to 0
 */
uint32_t time_usec_to_ticks(uint32_t usec);

/**
 * @brief Convert ticks into microseconds (float-to-float implementation)
 * 
 * @param ticks number of ticks to convert
 * @return float usec count output
 */
float time_ticksf_to_usecf(float ticks);

/**
 * @brief Convert ticks into microseconds (float-to-float implementation)
 * 
 * @param ticks number of microseconds to convert
 * @return float tick count output
 */
float time_usecf_to_ticksf(float usec);

/**
 * @brief Delay for a number of milliseconds
 * 
 * @param ms number of milliseconds to delay for
 */
void time_delay_msec(uint32_t ms);

/**
 * @brief Delay for a number of microseconds
 * 
 * @param us number of microseconds to delay for
 */
void time_delay_usec(uint32_t us);

/**
 * @brief Get current number of seconds since init as a float
 * 
 * @return float number of seconds
 */
float time_secf(void);

/**
 * @brief Get current number of seconds since init as a double
 * 
 * @return double number of seconds
 */
double time_seclf(void);

/**
 * @brief Get current number of milliseconds since init as a uint32_t
 * 
 * @return uint32_t number of milliseconds (truncated to fit 32-bit datatype)
 */
uint32_t time_msec(void);

/**
 * @brief Get current number of milliseconds since init as a float
 * 
 * @return float number of milliseconds
 */
float time_msecf(void);

/**
 * @brief Get current number of milliseconds since init as a double (64 bit float)
 * 
 * @return double number of milliseconds
 */
double time_mseclf(void);

/**
 * @brief Get current number of microseconds since init as a uint32_t
 * 
 * @return uint32_t number of microseconds (truncated to fit 32-bit datatype)
 */
uint32_t time_usec(void);

/**
 * @brief Get current number of microseconds since init as a float
 * 
 * @return float number of microseconds
 */
float time_usecf(void);

/**
 * @brief Get current number of microseconds since init as a double (64 bit float)
 * 
 * @return double number of microseconds
 */
double time_useclf(void);

#ifdef __cplusplus
}
#endif  // __cplusplus 

#endif 	// D_TIME_H
