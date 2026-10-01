/*
MIT LICENSE

Copyright (c) 2014-2025 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

/**
 * @file serialPortPlatform.h
 * @brief Platform-specific initialization and baud-rate helpers backing the serialPort.h function-pointer API.
 *
 * @author Inertial Sense, Inc.
 * @copyright Copyright (c) 2014-2025 Inertial Sense, Inc. All rights reserved.
 */

#ifndef __IS_SERIALPORT_PLATFORM_H
#define __IS_SERIALPORT_PLATFORM_H

#include "serialPort.h"
#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * SN-8650: per-port state for the duplicate-error log-suppression gate below. Opaque to callers
 * other than serialPortErrorDedupGate() itself; zero-initialize to start (NULL `action` means no
 * run is open).
 */
typedef struct
{
    const char* action;
    int errorCode;
    uint32_t count;
    uint64_t firstMs;
    uint64_t deadlineMs;
    int pending;    //!< cheap flag: 1 while a run is open, so activity-path callers (see
                    //!< serialPortErrorDedupCheckStale()) can bail out with a single int compare
                    //!< on the (overwhelmingly common) case where nothing is pending.
} serial_port_error_dedup_t;

enum
{
    SERIAL_PORT_DEDUP_SUPPRESSED = 0,          //!< duplicate; caller should log nothing
    SERIAL_PORT_DEDUP_IMMEDIATE,               //!< no run was pending; caller should log this error immediately
    SERIAL_PORT_DEDUP_SUMMARY_THEN_IMMEDIATE,  //!< caller should log the (now-filled) summary, then this error
    SERIAL_PORT_DEDUP_SUMMARY_FOLDED,          //!< this duplicate closed an expired run; caller should log only the summary
};

/**
 * SN-8650: decision logic for collapsing a burst of identical port errors (same action, same
 * code, same port) into one immediate report plus at most one trailing summary, instead of one
 * log line per occurrence. Pure state transition -- no I/O, no OS time calls -- so it can be
 * driven directly by unit tests with synthetic timestamps. See serialPortPlatform.c's
 * serialPortReportError() for how real call sites use it, and its own doc comment for the full
 * state-machine description. Exposed (non-static) for unit testing.
 *
 * @param state the port's dedup state; persists for the life of the handle it's embedded in.
 * @param action a stable (e.g. string-literal) description of the failed operation, compared by
 *   pointer -- reuse the same literal for the same operation every call.
 * @param errorCode the platform error code for this occurrence.
 * @param nowMs current time in milliseconds, supplied by the caller.
 * @param windowMs the one-shot duration: the max gap before a run (or a lone immediate report
 *   waiting for a first duplicate) is considered closed -- a later recurrence of the same
 *   action+code arriving after this gap is treated as a fresh error, not a duplicate.
 * @param summaryOut buffer to receive the formatted summary text when a run is flushed; untouched
 *   otherwise. May be NULL to skip formatting.
 * @param summaryOutLen size of summaryOut in bytes.
 * @return one of the SERIAL_PORT_DEDUP_* values, describing what the caller should log.
 */
int serialPortErrorDedupGate(serial_port_error_dedup_t* state, const char* action, int errorCode, uint64_t nowMs, uint32_t windowMs, char* summaryOut, size_t summaryOutLen);

/**
 * SN-8650: opportunistic flush for a pending dedup run, driven by *any* port activity rather than
 * only by the next error. serialPortErrorDedupGate() alone only ever re-checks a run's deadline
 * when another error of some kind arrives on that port -- fine for a dense burst (the next
 * duplicate arrives well inside the burst), but a small trailing run on an otherwise-healthy port
 * would sit open, with no deadline check ever happening, until the port's next unrelated error
 * (which might be minutes away, or might never come). Call this at the top of every port
 * operation (open/close/read/write/flush/drain/...) so a stale run gets flushed on the very next
 * successful operation, not just the next failure.
 *
 * `state->pending` makes the common case (no run open) a single int compare -- this is meant to
 * be cheap enough to call unconditionally on every operation, not just on errors.
 *
 * @param state the port's dedup state.
 * @param nowMs current time in milliseconds, supplied by the caller.
 * @param summaryOut buffer to receive the formatted summary text if a stale run is flushed;
 *   untouched otherwise. May be NULL to skip formatting.
 * @param summaryOutLen size of summaryOut in bytes.
 * @return 1 if a stale run was flushed (summaryOut is filled), 0 otherwise (nothing to do).
 */
int serialPortErrorDedupCheckStale(serial_port_error_dedup_t* state, uint64_t nowMs, char* summaryOut, size_t summaryOutLen);

/**
 * Zeros the serial_port_t struct then assigns the function-pointer table for the current
 * platform (e.g. Windows, Linux, macOS).
 * @param port the port to initialize
 * @return non-zero if success, 0 if the current platform is not implemented
 */
int serialPortPlatformInit(port_handle_t port);

#if !defined(_WIN32)
/**
 * SN-8239: returns the termios Bxxx constant for a known standard baud rate. Declared only
 * off-Windows to match its definition (the Windows branch of serialPortPlatform.c does not
 * define it), so a stray Windows caller fails at compile time rather than link time. Exposed
 * (non-static) for unit testing.
 * @param baudRate the baud rate to look up
 * @return the termios Bxxx constant for baudRate, or 0 if the rate must use the custom baud path
 */
int serialPortStandardBaudRate(int baudRate);

/**
 * SN-8239: reports whether baudRate can be opened, either via a standard termios Bxxx constant or
 * via the platform custom-rate path. Declared only off-Windows to match its definition (the Windows
 * branch of serialPortPlatform.c does not define it), so a stray Windows caller fails at compile
 * time rather than link time. Exposed (non-static) for unit testing.
 * @param baudRate the baud rate to check
 * @return 1 if baudRate is in (0, SERIAL_PORT_BAUDRATE_MAX], 0 otherwise
 */
int serialPortBaudRateSupported(int baudRate);
#endif

#if defined(__linux__)
/**
 * SN-8239: sets an arbitrary custom baud rate on an open fd via termios2/BOTHER (see
 * serialPortLinuxCustomBaud.c). Used for rates that have no standard termios Bxxx constant.
 * @param fd the open file descriptor of the serial device
 * @param baudRate the desired baud rate
 * @return 0 on success, -1 on failure (errno set by the underlying ioctl)
 */
int serialPortSetCustomBaudLinux(int fd, int baudRate);
#endif

#ifdef __cplusplus
}
#endif

#endif // __IS_SERIALPORT_PLATFORM_H
