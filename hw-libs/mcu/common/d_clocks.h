#ifndef _D_CLOCK_H
#define _D_CLOCK_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

extern uint32_t SystemCoreClock;

void clocks_init(void);

#define CLK_SHORT_WAIT  0
#define CLK_LONG_WAIT   1

/**
 * @brief Waits for a register value to match a mask and value
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
