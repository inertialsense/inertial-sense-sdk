#ifndef _D_CLOCK_H
#define _D_CLOCK_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

extern uint32_t SystemCoreClock;
extern uint32_t g_clocksStatus;

void clocks_init(void);
void disable_clock_switching(void);

#define CLK_SHORT_WAIT  0
#define CLK_LONG_WAIT   1

enum
{
    IS_CLOCK_STATUS_HIGH_ACCURACY = 0x01,
    IS_CLOCK_STATUS_FAULT = 0x02,
};

/**
 * @brief Waits for a register value to match a mask and value without using a timer
 * 
 * @param longer 0 for short wait (~100ms), 1 for long wait (~1s)
 * @return uint8_t 0 if success, 1 if timeout
 */
static inline uint8_t clocks_wait_for(volatile uint32_t *reg, uint32_t mask, uint32_t value, uint8_t longer)
{
    // Set the wait time based on the speed of the clock
    // divide system clock by 2 for the short wait, divide by 32 for the long wait
    // reason: we don't have a timebase running at this point and we have to 
    //  use a loop to wait, and the loop will take a few cycles (2?) to execute
    volatile int32_t counter = longer ? (SystemCoreClock >> 1) : (SystemCoreClock >> 5);

    // Wait for the reset event flag
    while (((*reg) & mask) != value)
    {
        if(--counter < 0)
        {
            return 1;
        }
    }
    
    return 0;
}

#ifdef __cplusplus
}
#endif

#endif	// _D_CLOCK_H
