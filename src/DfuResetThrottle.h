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

#include <chrono>
#include <mutex>
#include <thread>

#define DFU_USB_RESET_SPACING_MS    250     // (ms) minimum quiet time between USB resets of DFU devices

/**
 * Scoped gate around any operation that makes a DFU device reset and re-enumerate. Construction blocks until
 * this caller's turn; the operation runs while the gate is held, and destruction starts the spacing interval.
 *
 * Holding the lock through the operation is deliberate: pre-reserved time slots only space out the deadlines,
 * so callers delayed past theirs (scheduling, or an earlier reset running long) could still reset back-to-back.
 * Measuring from when the operation actually finished guarantees the spacing. A lone caller does not wait.
 *
 * Keep the scope to the reset-triggering call only: everything else waits on the shared lock meanwhile. The lock
 * is not recursive, so never nest gates.
 */
class DfuUsbResetGate
{
public:
    DfuUsbResetGate() : m_lock(state().mutex) { std::this_thread::sleep_until(state().nextAllowed); }
    ~DfuUsbResetGate() { state().nextAllowed = clock::now() + std::chrono::milliseconds(DFU_USB_RESET_SPACING_MS); }

    DfuUsbResetGate(const DfuUsbResetGate&) = delete;
    DfuUsbResetGate& operator=(const DfuUsbResetGate&) = delete;

private:
    using clock = std::chrono::steady_clock;

    struct State
    {
        std::mutex mutex;
        clock::time_point nextAllowed;
    };

    static State& state()
    {
        static State s;
        return s;
    }

    std::lock_guard<std::mutex> m_lock;
};

#endif // DFU_RESET_THROTTLE_H
