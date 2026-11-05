/**
 * @file DfuResetThrottle.h
 * @brief Spaces out USB resets of DFU devices across all threads in this process.
 *
 * Boards programmed in parallel finish within a moment of each other, so without this a fixture resets
 * and re-enumerates all at once. That burst behind a shared hub has been seen to precede xHCI host
 * controller failures ("HC died"), which also takes out every other device on that controller.
 */

#ifndef DFU_RESET_THROTTLE_H
#define DFU_RESET_THROTTLE_H

#include <algorithm>
#include <chrono>
#include <mutex>
#include <thread>

#define DFU_USB_RESET_SPACING_MS    250     // (ms) minimum time between USB resets of DFU devices

/**
 * Blocks until this caller's turn to issue a USB reset. The slot is reserved under the lock but the wait
 * happens outside it, so a burst of N callers takes (N-1) * DFU_USB_RESET_SPACING_MS and a lone caller
 * does not wait at all.
 */
inline void dfuThrottleUsbReset()
{
    using clock = std::chrono::steady_clock;
    static std::mutex mutex;
    static clock::time_point nextSlot;

    clock::time_point slot;
    {
        std::lock_guard<std::mutex> lock(mutex);
        slot = std::max(clock::now(), nextSlot);
        nextSlot = slot + std::chrono::milliseconds(DFU_USB_RESET_SPACING_MS);
    }
    std::this_thread::sleep_until(slot);
}

#endif // DFU_RESET_THROTTLE_H
