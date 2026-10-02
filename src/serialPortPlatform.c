/*
MIT LICENSE

Copyright (c) 2014-2025 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include "core/base_port.h"
#include "core/msg_logger.h"
#include "serialPort.h"
#include "serialPortPlatform.h"
#include "ISConstants.h"

#if PLATFORM_IS_LINUX || PLATFORM_IS_APPLE

#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/stat.h>
#include <sys/statvfs.h>
#include <sys/file.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>
#include <poll.h>

// cygwin defines FIONREAD in socket.h instead of ioctl.h
#ifndef FIONREAD
#include <sys/socket.hh>
#endif

#if PLATFORM_IS_LINUX
#include <linux/serial.h>
#endif

#if PLATFORM_IS_APPLE
#include <CoreFoundation/CoreFoundation.h>
#include <IOKit/IOKitLib.h>
#include <IOKit/serial/IOSerialKeys.h>
#include <IOKit/serial/ioss.h>
#include <IOKit/IOBSD.h>
#endif

#ifndef B460800
#define B460800 460800
#endif
#ifndef B921600
#define B921600 921600
#endif
#ifndef B1000000
#define B1000000 1000000
#endif
#ifndef B1500000
#define B1500000 1500000
#endif
#ifndef B2000000
#define B2000000 2000000
#endif
#ifndef B2500000
#define B2500000 2500000
#endif
#ifndef B3000000
#define B3000000 3000000
#endif
#ifndef B4000000
#define B4000000 4000000
#endif

#endif

/**
 * @brief Tunable: the maximum gap, in milliseconds, between two occurrences of the identical
 * port error (same port, same action, same code) before the run is force-flushed as a single
 * INFO+ summary. Each duplicate that arrives before this elapses pushes the deadline back out,
 * so a dense burst (e.g. a shared-USB-hub contention event producing one failure per comm tick)
 * collapses into one ERROR line plus one trailing summary regardless of how long the burst runs;
 * a gap at least this wide, or a change in error, closes the run (SN-8650).
 */
#ifndef SERIAL_PORT_ERROR_DEDUP_WINDOW_MS
#define SERIAL_PORT_ERROR_DEDUP_WINDOW_MS   1000
#endif

// serial_port_error_dedup_t and the SERIAL_PORT_DEDUP_* enum are declared in serialPortPlatform.h
// (exposed there, non-static, for unit testing -- see serialPortErrorDedupGate()'s doc comment).

/**
 * @brief Formats the trailing summary line for a dedup run: the action/code it was tracking, how
 * many duplicates it absorbed, and how long (from the first suppressed duplicate to now) it ran.
 */
static void serialPortFormatDedupSummary(char* buf, size_t len, const serial_port_error_dedup_t* state, uint64_t nowMs)
{
    if (state->detail[0] != '\0')
        snprintf(buf, len, "%s: %s (%d) - message duplicated %u time(s) over the last %llums.",
            state->action, state->detail, state->errorCode, (unsigned)state->count, (unsigned long long)(nowMs - state->firstMs));
    else
        snprintf(buf, len, "%s (%d) - message duplicated %u time(s) over the last %llums.",
            state->action, state->errorCode, (unsigned)state->count, (unsigned long long)(nowMs - state->firstMs));
}

/**
 * @brief Pure decision logic for SN-8650 duplicate-error log suppression -- no I/O, no logging,
 * no OS calls, just state transitions, so it can be exercised directly by unit tests with
 * synthetic timestamps. See serialPortReportError() for the logging wrapper real call sites use.
 *
 * The first report of any error (or of an error that differs from the one currently tracked) is
 * always reported immediately, exactly as before this change, and arms a windowMs-wide "watch
 * period" during which a matching duplicate will be suppressed. Each subsequent duplicate that
 * actually arrives inside that window is counted and pushes the window back out instead of being
 * reported. The run is closed out -- as a single summary describing the count and the elapsed
 * time since the first suppressed duplicate -- as soon as either a different error arrives, or
 * a matching occurrence arrives after the window has already lapsed (whether or not any
 * duplicate was ever actually counted). A closed run's identity is forgotten, so the very next
 * occurrence (even an identical one) is always reported immediately rather than silently
 * resuming suppression -- this applies equally to a lone immediate report that never gained a
 * duplicate before its watch period lapsed, so a sparse, isolated recurrence of the same error
 * (e.g. hours apart) is never silently absorbed as "just another duplicate" of a long-past one.
 *
 * @param state the port's dedup state (one instance per serialPortHandle; persists for the life
 *   of the handle, so it naturally resets whenever the port is reopened).
 * @param action a *stable* (e.g. string-literal) description of the operation that failed;
 *   compared by pointer, not content, so callers must reuse the same literal for the same
 *   operation every time, not re-format it.
 * @param errorCode the platform error code for this occurrence (Win32 GetLastError() value or
 *   errno, matching whatever the caller already logs).
 * @param nowMs current time in milliseconds (caller-supplied so this function has no clock
 *   dependency -- see serialPortNowMs()).
 * @param windowMs the tunable one-shot duration; see SERIAL_PORT_ERROR_DEDUP_WINDOW_MS.
 * @param summaryOut buffer to receive the formatted summary text when a run is flushed (either
 *   SERIAL_PORT_DEDUP_SUMMARY_THEN_IMMEDIATE or SERIAL_PORT_DEDUP_SUMMARY_FOLDED); untouched
 *   otherwise. May be NULL to skip formatting (the caller must then ignore the summary).
 * @param summaryOutLen size of summaryOut in bytes.
 * @return one of the SERIAL_PORT_DEDUP_* values above, describing what the caller should log.
 */
int serialPortErrorDedupGate(serial_port_error_dedup_t* state, const char* action, int errorCode, uint64_t nowMs, uint32_t windowMs, char* summaryOut, size_t summaryOutLen)
{
    int isSameTracked = (state->action != 0) && (action == state->action) && (errorCode == state->errorCode);

    // `deadlineMs` does double duty: while count==0 it's "how long this single immediate report
    // stays eligible to gain a first duplicate"; once count>0 it's the usual sliding dup window.
    // Without this, a later, isolated recurrence of the same action+code -- arriving long after
    // a lone immediate report that never got a duplicate -- would match isSameTracked forever
    // and be silently absorbed as "duplicate #1" instead of getting its own fresh report.
    if (isSameTracked && nowMs < state->deadlineMs)
    {
        if (state->count == 0)
        {
            state->firstMs = nowMs;
            state->pending = 1;
        }
        state->count++;
        state->deadlineMs = nowMs + windowMs;
        return SERIAL_PORT_DEDUP_SUPPRESSED;
    }

    if (isSameTracked && state->count > 0)
    {
        // same error, but the gap since the previous duplicate exceeded windowMs -- fold this
        // one in and close the run
        state->count++;
        if (summaryOut)
            serialPortFormatDedupSummary(summaryOut, summaryOutLen, state, nowMs);
        state->action = 0;
        state->count = 0;
        state->pending = 0;
        return SERIAL_PORT_DEDUP_SUMMARY_FOLDED;
    }

    // either a different error, or the same one arriving too late to count as a duplicate of an
    // untouched lone immediate report -- flush whatever run was actually open (if any), then
    // report this occurrence immediately and start tracking it fresh
    int hadPendingRun = (state->action != 0) && (state->count > 0);
    if (hadPendingRun && summaryOut)
        serialPortFormatDedupSummary(summaryOut, summaryOutLen, state, nowMs);

    state->action = action;
    state->errorCode = errorCode;
    state->count = 0;
    state->pending = 0;
    state->deadlineMs = nowMs + windowMs;   // arm the window for a potential first duplicate

    return hadPendingRun ? SERIAL_PORT_DEDUP_SUMMARY_THEN_IMMEDIATE : SERIAL_PORT_DEDUP_IMMEDIATE;
}

/**
 * @brief SN-8650: opportunistic, activity-driven staleness check -- see the doc comment in
 * serialPortPlatform.h. Pure (no I/O), like serialPortErrorDedupGate(), so it's unit-testable
 * with synthetic timestamps.
 */
int serialPortErrorDedupCheckStale(serial_port_error_dedup_t* state, uint64_t nowMs, char* summaryOut, size_t summaryOutLen)
{
    if (!state->pending)
        return 0;   // cheap bail-out: the common case on a healthy, or even a bursty-but-still-within-window, port

    if (nowMs < state->deadlineMs)
        return 0;   // run is still within its window; nothing to do yet

    if (summaryOut)
        serialPortFormatDedupSummary(summaryOut, summaryOutLen, state, nowMs);
    state->action = 0;
    state->count = 0;
    state->pending = 0;
    return 1;
}

/**
 * @brief Current monotonic time in milliseconds, for serialPortErrorDedupGate()'s nowMs.
 */
static uint64_t serialPortNowMs(void)
{
#if PLATFORM_IS_WINDOWS
    return (uint64_t)GetTickCount64();
#else
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000ULL + (uint64_t)(ts.tv_nsec / 1000000ULL);
#endif
}

/**
 * @brief Report a port error, running it through serialPortErrorDedupGate() first so a burst of
 * identical failures on the same port collapses into one ERROR line plus (at most) one trailing
 * INFO+ summary instead of one ERROR line per occurrence (SN-8650).
 *
 * @param port the port, for portName() in the log line(s).
 * @param dedup the handle's dedup state (serialPortHandle::errDedup).
 * @param action a stable, static description of the failed operation -- see
 *   serialPortErrorDedupGate()'s `action` parameter.
 * @param errorCode the platform error code to report.
 * @param detail optional display-only text to accompany `action` in the log output (e.g. a POSIX
 *   `strerror(errorCode)` string). `action` alone remains the stable dedup key -- passed straight
 *   through to serialPortErrorDedupGate() unchanged -- so callers that don't have extra text
 *   (e.g. Windows call sites, whose `action` is already self-descriptive, such as "WriteFile()
 *   failed") can simply pass NULL. (Review follow-up on SN-8650: the original common logger
 *   dropped the POSIX strerror() text that call sites used to log directly.)
 */
static void serialPortReportError(port_handle_t port, serial_port_error_dedup_t* dedup, const char* action, int errorCode, const char* detail)
{
    char summary[192];
    int mode = serialPortErrorDedupGate(dedup, action, errorCode, serialPortNowMs(), SERIAL_PORT_ERROR_DEDUP_WINDOW_MS, summary, sizeof(summary));

    // A fresh run (no fold/suppress) starts being tracked by this call -- stash its display
    // detail now so a later fold/flush of *this* run (serialPortFormatDedupSummary(), above) can
    // still include it. Must happen after the gate call: for SUMMARY_THEN_IMMEDIATE the summary
    // just formatted describes the *previous* (different) run, so overwriting detail beforehand
    // would corrupt that summary with this occurrence's text instead.
    if (mode == SERIAL_PORT_DEDUP_IMMEDIATE || mode == SERIAL_PORT_DEDUP_SUMMARY_THEN_IMMEDIATE)
        snprintf(dedup->detail, sizeof(dedup->detail), "%s", detail ? detail : "");

    if (mode == SERIAL_PORT_DEDUP_SUMMARY_THEN_IMMEDIATE || mode == SERIAL_PORT_DEDUP_SUMMARY_FOLDED)
        log_more_info(IS_LOG_PORT, "[%s] %s", portName(port), summary);
    if (mode == SERIAL_PORT_DEDUP_IMMEDIATE || mode == SERIAL_PORT_DEDUP_SUMMARY_THEN_IMMEDIATE)
    {
        if (detail && detail[0] != '\0')
            log_error(IS_LOG_PORT, "[%s] %s: %s (%d)", portName(port), action, detail, errorCode);
        else
            log_error(IS_LOG_PORT, "[%s] %s (%d)", portName(port), action, errorCode);
    }
}

/**
 * @brief Flush (log) a pending SN-8650 duplicate-error run for a port, without waiting for a
 * change in error or the one-shot window to lapse. Called at close so a run that was still
 * accumulating when the port was closed isn't silently dropped along with the handle.
 */
static void serialPortFlushPendingErrorDedup(port_handle_t port, serial_port_error_dedup_t* dedup)
{
    if (dedup->action == 0 || dedup->count == 0)
        return;

    char summary[192];
    serialPortFormatDedupSummary(summary, sizeof(summary), dedup, serialPortNowMs());
    log_more_info(IS_LOG_PORT, "[%s] %s", portName(port), summary);
    dedup->action = 0;
    dedup->count = 0;
    dedup->pending = 0;
}

/**
 * @brief Opportunistic, activity-driven flush: call at the top of every port operation (open,
 * close, read, write, flush, drain, byte-count queries, ...) so a run that's still pending gets
 * flushed on the very next successful operation, not just the next failure (SN-8650). The common
 * case -- nothing pending -- is a single int compare inside serialPortErrorDedupCheckStale(), so
 * this is cheap enough to call unconditionally.
 */
static void serialPortCheckActivity(port_handle_t port, serial_port_error_dedup_t* dedup)
{
    char summary[192];
    if (serialPortErrorDedupCheckStale(dedup, serialPortNowMs(), summary, sizeof(summary)))
        log_more_info(IS_LOG_PORT, "[%s] %s", portName(port), summary);
}

typedef struct
{
    int blocking;

    // SN-8650: duplicate-error log suppression state (see serialPortErrorDedupGate() /
    // serialPortReportError()). One run per handle; naturally resets whenever the port is
    // reopened, since a fresh handle is calloc'd on each successful open.
    serial_port_error_dedup_t errDedup;

#if PLATFORM_IS_WINDOWS

    void* platformHandle;
    OVERLAPPED ovRead;
    OVERLAPPED ovWrite;

#else

    int fd;

#endif

} serialPortHandle;

/**
 * @brief Sleep for a specified number of milliseconds.
 * This function is a simple wrapper around the platform-specific sleep function.
 * On Windows, it uses `Sleep()`, and on other platforms, it uses `usleep()`.
 *
 * @param sleepMilliseconds The number of milliseconds to sleep.
 * @return int 1 on success.
 */
static int serialPortSleepPlatform(int sleepMilliseconds);
/**
 * @brief Flush the serial port.
 * This function clears the serial port's receive buffer.
 * On Windows, it uses `PurgeComm` with `PURGE_RXCLEAR`.
 * On other platforms, it uses `tcflush` with `TCIOFLUSH`.
 *
 * @param port The port handle.
 * @return int 1 on success, 0 on failure.
 */
static int serialPortFlushPlatform(port_handle_t port);
/**
 * @brief Drain the serial port.
 * This function waits for all written data to be transmitted.
 * On Windows, it uses `PurgeComm` with `PURGE_TXCLEAR`.
 * On other platforms, it uses `tcdrain`.
 *
 * @param port The port handle.
 * @return int 1 on success, 0 on failure.
 */
static int serialPortDrainPlatform(port_handle_t port);
/**
 * @brief Read from the serial port with a timeout.
 * This function reads a specified number of bytes from the serial port, with a timeout.
 * It is a wrapper around the platform-specific read functions.
 * If the timeout is negative, a default timeout is used.
 *
 * @param port The port handle.
 * @param buffer The buffer to read into.
 * @param readCount The number of bytes to read.
 * @param timeoutMilliseconds The timeout in milliseconds.
 * @return int The number of bytes read, or -1 on error.
 */
static int serialPortReadTimeoutPlatform(port_handle_t port, unsigned char* buffer, unsigned int readCount, int timeoutMilliseconds);
// static int serialPortReadTimeoutPlatformLinux(serialPortHandle* handle, unsigned char* buffer, int readCount, int timeoutMilliseconds);

#if PLATFORM_IS_WINDOWS
// Forward-declared: serialPortReadTimeoutPlatform() (above, used well before its own definition)
// calls this, but its definition lives with the other win32-error-classification helpers much
// later in the file. Without this, the call at line ~1059 precedes any declaration in this
// translation unit (PR #1331 review).
static int win32ErrorIndicatesDeviceLost(DWORD err);
#endif


// #define DEBUG_COMMS   // Enabling this will cause all traffic to be printed on the console, with timestamps and direction (<< = received, >> = transmitted).
#ifdef DEBUG_COMMS
    #define debugDumpBuffer(...) static_log_buffer(__VA_ARGS__)
#else
    #define debugDumpBuffer(...)
#endif


#if PLATFORM_IS_WINDOWS

#define WINDOWS_OVERLAPPED_BUFFER_SIZE 8192

typedef struct {
    OVERLAPPED ov;
    pfnSerialPortAsyncReadCompletion externalCompletion;
    port_handle_t port;
    unsigned char* buffer;
} readFileExCompletionStruct;

/**
 * @brief Completion routine for ReadFileEx.
 * This function is called when an asynchronous read operation completes.
 * It calls the external completion function and frees the completion structure.
 *
 * @param errorCode The error code.
 * @param bytesTransferred The number of bytes transferred.
 * @param ov The overlapped structure.
 */
static void CALLBACK readFileExCompletion(DWORD errorCode, DWORD bytesTransferred, LPOVERLAPPED ov)
{
    readFileExCompletionStruct* c = (readFileExCompletionStruct*)ov;
    c->externalCompletion(c->port, c->buffer, bytesTransferred, errorCode);
    free(c);
}

#else

/**
 * @brief Map a baud rate to its standard termios Bxxx constant, if one exists.
 * Returns the corresponding termios speed constant for a known standard rate, or 0 if the rate
 * has no standard constant. A 0 return does NOT mean "invalid" (see serialPortBaudRateSupported):
 * it means the rate must be applied via the platform custom-rate path (Linux termios2/BOTHER,
 * macOS IOSSIOSPEED). SN-8239: added 1000000 and 4000000; note 1220000/1440000 have no Bxxx
 * constant on Linux and therefore intentionally return 0 (custom path).
 *
 * @param baudRate The requested baud rate (bits/sec).
 * @return int The termios Bxxx constant, or 0 if the rate is not a standard enumerated rate.
 */
int serialPortStandardBaudRate(int baudRate)
{
    switch (baudRate)
    {
    default:      return 0;
    case 300:     return B300;
    case 600:     return B600;
    case 1200:    return B1200;
    case 2400:    return B2400;
    case 4800:    return B4800;
    case 9600:    return B9600;
    case 19200:   return B19200;
    case 38400:   return B38400;
    case 57600:   return B57600;
    case 115200:  return B115200;
    case 230400:  return B230400;
    case 460800:  return B460800;
    case 921600:  return B921600;
    case 1000000: return B1000000;
    case 1500000: return B1500000;
    case 2000000: return B2000000;
    case 2500000: return B2500000;
    case 3000000: return B3000000;
    case 4000000: return B4000000;
    }
}

/**
 * @brief Report whether a baud rate is supported by the SDK serial layer.
 * Accepts any positive rate up to SERIAL_PORT_BAUDRATE_MAX (10 Mbaud). Standard rates are applied
 * via their Bxxx constant; anything else is applied as a custom rate (Linux termios2/BOTHER,
 * macOS IOSSIOSPEED). Pure decision logic, separated so it is unit-testable without USB hardware.
 *
 * @param baudRate The requested baud rate (bits/sec).
 * @return int 1 if the rate is supported, 0 otherwise.
 */
int serialPortBaudRateSupported(int baudRate)
{
    return (baudRate > 0 && baudRate <= SERIAL_PORT_BAUDRATE_MAX) ? 1 : 0;
}

/**
 * @brief Configure the serial port.
 * This function configures the serial port with the specified baud rate and other settings.
 * It sets the port to 8N1, disables flow control, and sets the port to raw mode.
 * On Apple platforms, there is a special hack to set high baud rates.
 *
 * @param fd The file descriptor.
 * @param baudRate The baud rate.
 * @return int 0 on success, -1 on failure.
 */
static int configure_serial_port(int fd, int baudRate)
{
    struct termios tty = {};

    if (tcgetattr(fd, &tty) != 0) 
    {
        log_error(IS_LOG_PORT, "config_serial_port():: tcgetattr() : error getting tty settings: %s (%d)", strerror(errno), errno);
        return -1;
    }

    // SN-8239: accept standard rates (mapped to a termios Bxxx constant) and arbitrary custom rates
    // up to SERIAL_PORT_BAUDRATE_MAX (10 Mbaud). stdBaud == 0 means "no standard constant" -> use the
    // platform custom-rate path. Note baudRate keeps the raw requested rate throughout.
    if (!serialPortBaudRateSupported(baudRate))
    {
        log_error(IS_LOG_PORT, "config_serial_port():: unsupported baudrate: %d (max %d)", baudRate, SERIAL_PORT_BAUDRATE_MAX);
        return -1;
    }
    int stdBaud = serialPortStandardBaudRate(baudRate);

    // Set Baud Rate
#if PLATFORM_IS_APPLE

    // HACK: Mac will not allow higher baud rate until after set lower valid rate: e.g. 230400
    cfsetospeed(&tty, 230400);
    cfsetispeed(&tty, 230400);
    // IOSSIOSPEED takes the actual integer speed, so both standard and custom rates go through here.
    speed_t appleSpeed = (speed_t)baudRate;
    if (ioctl(fd, IOSSIOSPEED, &appleSpeed) == -1)
    {
        // SN-8239: with arbitrary rates now accepted, a rate the hardware can't produce must NOT be
        // reported as success — propagate the failure so the caller doesn't run at an unintended speed.
        log_error(IS_LOG_PORT, "config_serial_port():: error %d from ioctl IOSSIOSPEED", errno);
        return -1;
    }

#else

    // Linux: standard rates are set here via the Bxxx constant. Custom rates cannot be expressed by
    // cfsetospeed/cfsetispeed (Bxxx tops out at B4000000), so we set a valid placeholder speed now and
    // apply the real rate via termios2/BOTHER (serialPortSetCustomBaudLinux) after tcsetattr below.
    speed_t setSpeed = (stdBaud != 0) ? (speed_t)stdBaud : B38400;
    if (cfsetospeed(&tty, setSpeed) != 0 || cfsetispeed(&tty, setSpeed) != 0)
    {
        // SN-8239: the Bxxx constant was rejected by this platform (e.g. a B4000000 fallback that is
        // not a real termios speed on this kernel). Don't leave the port at an unintended speed and
        // report success: apply a safe placeholder and route through the custom-baud path below, which
        // sets the real rate via termios2/BOTHER and works for any rate.
        cfsetospeed(&tty, B38400);
        cfsetispeed(&tty, B38400);
        stdBaud = 0;
    }

    // Attempt to configure LOW_LATENCY for UART/serial ports - though doesn't appear to improve things much.
    struct serial_struct serial;
    ioctl(fd, TIOCGSERIAL, &serial);
    serial.flags |= ASYNC_LOW_LATENCY;
    serial.closing_wait = ASYNC_CLOSING_WAIT_NONE;
    ioctl(fd, TIOCSSERIAL, &serial);

#endif

    // Control Flags: Set 8N1 (8 data bits, No parity, 1 stop bit)
    tty.c_cflag &= ~PARENB;                     // Clear parity bit, disabling parity (most common)
    tty.c_cflag &= ~CSTOPB;                     // Clear stop field, only one stop bit used in communication (most common)
    tty.c_cflag &= ~CSIZE;                      // Clear all bits that set the data size
    tty.c_cflag |= CS8;                         // 8 bits per byte (most common)
    tty.c_cflag &= ~CRTSCTS;                    // Disable RTS/CTS hardware flow control (most common)
    tty.c_cflag |= CREAD | CLOCAL;              // Turn on READ & ignore model ctrl lines (CLOCAL = 1)

    // Local Modes: Set in non-canonical mode - Canonical mode is line-by-line processing; we want this DISABLED
    tty.c_lflag &= ~ICANON;                     // Disable Canonical Mode
    tty.c_lflag &= ~ECHO;                       // Disable echo
    tty.c_lflag &= ~ECHOE;                      // Disable erasure
    tty.c_lflag &= ~ECHONL;                     // Disable new-line echo
    tty.c_lflag &= ~ISIG;                       // Disable interpretation of INTR, QUIT and SUSP

    // Disable input processing options (raw mode)
    tty.c_iflag &= ~(IGNBRK | BRKINT);          // Disable break processing
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);     // Turn off xon/xoff software flow ctrl
    tty.c_iflag &= ~(ICRNL | INLCR | IGNCR );   // Disable any special handling of received bytes

    // Disable output processing options (raw mode)
    tty.c_oflag &= ~OPOST;                      // Prevent special interpretation of output bytes (e.g. newline chars)
    tty.c_oflag &= ~ONLCR;                      // Prevent conversion of newline to carriage return/line feed

    // Set the timeout and minimum characters.  Read doesn't block
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 0;

    // Save tty settings, also checking for error
    if (tcsetattr(fd, TCSANOW, &tty) != 0) 
    {
        log_error(IS_LOG_PORT, "config_serial_port():: tcsetattr() : error setting tty settings: %s (%d)", strerror(errno), errno);
        return -1;
    } else {
        // TODO: Note that tcsetattr() returns success if any of the requested changes could be successfully carried out.
        //  Therefore, when making multiple changes it may be necessary to follow this call with a further call to
        //  tcgetattr() to check that all changes have been performed successfully.
        //  Ie, we are probably not seeing this error as we think we are...

        struct termios new_tty = {};
        if (tcgetattr(fd, &new_tty) != 0)
        {
            log_error(IS_LOG_PORT, "config_serial_port():: tcgetattr() : error confirming successful setting of tty settings: %s (%d)", strerror(errno), errno);
            return -1;
        }
        if (memcmp(&new_tty, &tty, sizeof(struct termios)) != 0) {
            // what was set didn't match what was just read back (confirmation failed);
            // Let's figure out what didn't get set correctly...
            log_error(IS_LOG_PORT, "config_serial_port():: termios confirmation failed to match expected values:");
            if (new_tty.c_iflag != tty.c_iflag) { log_error(IS_LOG_PORT, "config_serial_port():: setting c_iflag mismatch: expected: %x, actual: %x", tty.c_iflag, new_tty.c_iflag); }
            if (new_tty.c_oflag != tty.c_oflag) { log_error(IS_LOG_PORT, "config_serial_port():: setting c_oflag mismatch: expected: %x, actual: %x", tty.c_oflag, new_tty.c_oflag); }
            if (new_tty.c_cflag != tty.c_cflag) { log_error(IS_LOG_PORT, "config_serial_port():: setting c_cflag mismatch: expected: %x, actual: %x", tty.c_cflag, new_tty.c_cflag); }
            if (new_tty.c_lflag != tty.c_lflag) { log_error(IS_LOG_PORT, "config_serial_port():: setting c_lflag mismatch: expected: %x, actual: %x", tty.c_lflag, new_tty.c_lflag); }
            for (int i = 0; i < 32; i++)
                if (new_tty.c_cc[i] != tty.c_cc[i]) { log_error(IS_LOG_PORT, "config_serial_port():: setting c_cc[%d] mismatch: expected: %d, actual: %d", i, tty.c_cc[i], new_tty.c_cc[i]); }

            #if PLATFORM_IS_LINUX
            if (new_tty.c_line != tty.c_line) { log_error(IS_LOG_PORT, "config_serial_port():: setting c_line mismatch: expected: %d, actual: %d", tty.c_line, new_tty.c_line); }
            #endif

            return -1;
        }
    }

#if PLATFORM_IS_LINUX
    // SN-8239: a custom (non-standard) rate can't be expressed by Bxxx/cfsetospeed, so the flags above
    // were applied with a placeholder speed; now set the real rate via termios2/BOTHER. This preserves
    // the just-applied line flags (it reads them back via TCGETS2 and only overrides the speed).
    if (stdBaud == 0)
    {
        if (serialPortSetCustomBaudLinux(fd, baudRate) != 0)
        {
            log_error(IS_LOG_PORT, "config_serial_port():: failed to set custom baud %d: %s (%d)", baudRate, strerror(errno), errno);
            return -1;
        }
    }
#endif

    return 0;
}

/**
 * @brief configures flow control on the specified file descriptor
 * @param fd
 * @param control
 * @return 0 one success or errno on failure
 */
int set_flowcontrol(int fd, int control)
{
    struct termios tty;
    memset(&tty, 0, sizeof tty);
    if (tcgetattr(fd, &tty) != 0)
    {
        log_more_debug(IS_LOG_PORT, "set_flowcontrol():: error calling tcgetattr : %s (%d)", strerror(errno), errno);
        return errno;
    }

    if(control) tty.c_cflag |= CRTSCTS;
    else tty.c_cflag &= ~CRTSCTS;

    if (tcsetattr(fd, TCSANOW, &tty) != 0)
    {
        log_more_debug(IS_LOG_PORT, "set_flowcontrol():: error calling tcsetattr : %s (%d)", strerror(errno), errno);
        return errno;
    }
    return 0;
}

/**
 * @brief Set the serial port to non-blocking mode.
 * @brief Use modern O_NONBLOCK instead of legacy O_NDELAY. Because of non-blocking mode, we have to retry serial write() to handle partial writes until all data received by the OS.
 * This is done by getting the current flags, adding O_NONBLOCK, and then setting the new flags.
 * It also uses `flock` to get an exclusive, non-blocking lock on the file descriptor.
 *
 * @param fd The file descriptor.
 * @return int 0 on success, errno on failure.
 */
int set_nonblocking(int fd) 
{
    int flags = fcntl(fd, F_GETFL, 0);
    if (flags == -1) 
    {
        log_error(IS_LOG_PORT, "set_nonblocking():: error fcntl F_GETFL : %s (%d)", strerror(errno), errno);
        return errno;
    }

    flags |= O_NONBLOCK;
    if (fcntl(fd, F_SETFL, flags) == -1) 
    {
        log_error(IS_LOG_PORT, "set_nonblocking():: error setting O_NONBLOCK : %s (%d)", strerror(errno), errno);
        return errno;
    }

    // Alternate method - this maybe redundant, but better to be safe, eh?
    flock(fd, LOCK_EX | LOCK_NB);

    return 0;
}

#endif

/**
 * @brief Open the serial port.
 * This function opens and configures the serial port with the specified parameters.
 * On Windows, it handles the `\\\\.\\` prefix for COM ports above 9.
 * It also sets up the port for overlapped I/O if non-blocking.
 * On other platforms, it opens the port and then configures it using `configure_serial_port`.
 *
 * @param port The port handle.
 * @param portName The name of the port to open.
 * @param baudRate The baud rate.
 * @param blocking 1 for blocking, 0 for non-blocking.
 * @return int 1 on success, 0 on failure.
 */
static int serialPortOpenPlatform(port_handle_t port, const char* portName, int baudRate, int blocking)
{
    log_debug(IS_LOG_PORT, "serialPortOpenPlatform(%s:B%d, %sblocking) called.", portName, baudRate, blocking ? "" : "non-");

    serial_port_t* serialPort = (serial_port_t*)port;
    if (serialPort->handle != 0)
    {
        // already open --  FIXME: Should we be closing the port and then reopen??
        // serialPortClose(serialPort);
        return 1;
    }

    serialPortSetName(port, portName);
    serialPort->baudRate = baudRate;

#if PLATFORM_IS_WINDOWS

    void* platformHandle = 0;
    platformHandle = CreateFileA(portName, GENERIC_READ | GENERIC_WRITE, 0, 0, OPEN_EXISTING, !blocking ? FILE_FLAG_OVERLAPPED : 0, 0);
    serialPort->errorCode = (int)GetLastError();
    if (platformHandle == INVALID_HANDLE_VALUE)
    {
        // don't modify the originally requested port value, just create a new value that Windows needs for COM10 and above
        char tmpPort[MAX_SERIAL_PORT_NAME_LENGTH];
        sprintf_s(tmpPort, sizeof(tmpPort), "\\\\.\\%s", portName);
        platformHandle = CreateFileA(tmpPort, GENERIC_READ | GENERIC_WRITE, 0, 0, OPEN_EXISTING, !blocking ? FILE_FLAG_OVERLAPPED : 0, 0);
        if (platformHandle == INVALID_HANDLE_VALUE)
        {
            // SN-8650: CreateFileA() is a Win32 API; its error is GetLastError(), not errno (an
            // unrelated CRT global -- see the SN-8697 comment on the write path for the same
            // mistake). Must re-fetch here: the errorCode stashed after the *first* CreateFileA()
            // attempt, above, is stale by the time this second attempt fails.
            serialPort->errorCode = (int)GetLastError();
            serialPort->error = "CreateFileA() failed";
            log_error(IS_LOG_PORT, "[%s] serialPortOpenPlatform() failed to open port: %s (%d)", portName, serialPort->error, serialPort->errorCode);
            return 0;
        }
    }

    // clear any pending data associated with the port, incoming or outgoing; and ignore any errors
    PurgeComm(platformHandle, PURGE_RXCLEAR | PURGE_TXCLEAR);

    DCB serialParams;
    serialParams.DCBlength = sizeof(DCB);
    if (GetCommState(platformHandle, &serialParams))
    {
        serialParams.BaudRate = baudRate;
        serialParams.ByteSize = DATABITS_8;
        serialParams.StopBits = ONESTOPBIT;

        switch(serialPort->options & OPT_PARITY_MASK)
        {
        case OPT_PARITY_EVEN:
            serialParams.Parity = EVENPARITY;
            break;
        case OPT_PARITY_ODD:    
            serialParams.Parity = ODDPARITY;
            break;
        case OPT_PARITY_NONE:
        default:
            serialParams.Parity = NOPARITY;
            break;
        }

        serialParams.fBinary = 1;
        serialParams.fInX = 0;
        serialParams.fOutX = 0;
        serialParams.fAbortOnError = 0;
        serialParams.fNull = 0;
        serialParams.fErrorChar = 0;
        serialParams.fDtrControl = DTR_CONTROL_ENABLE;
        serialParams.fRtsControl = RTS_CONTROL_ENABLE;
        if (!SetCommState(platformHandle, &serialParams))
        {
            // SN-8650: SetCommState() is a Win32 API -- GetLastError(), not errno (see the
            // CreateFileA() comment above for the same mistake, and the SN-8697 comment on the
            // write path where this was first fixed). This is what produced the nonsensical
            // "No error (0)" / "File exists (17)" ERROR lines in the field log: errno just held
            // whatever unrelated CRT call last touched it, not SetCommState()'s actual failure.
            serialPort->errorCode = (int)GetLastError();
            serialPort->error = "SetCommState() failed";
            log_error(IS_LOG_PORT, "[%s] serialPortOpenPlatform() failed to set COMM port parameters: %s (%d)", portName, serialPort->error, serialPort->errorCode);
            CloseHandle(platformHandle);  // serialPort->handle not yet assigned; close raw handle directly
            return 0;
        }
    }
    else
    {
        // SN-8650: GetCommState() is a Win32 API -- GetLastError(), not errno; see above.
        serialPort->errorCode = (int)GetLastError();
        serialPort->error = "GetCommState() failed";
        log_error(IS_LOG_PORT, "[%s] serialPortOpenPlatform() failed to retreive COMM port parameters: %s (%d)", portName, serialPort->error, serialPort->errorCode);
        CloseHandle(platformHandle);  // serialPort->handle not yet assigned; close raw handle directly
        return 0;
    }

    COMMTIMEOUTS timeouts;
    if (!GetCommTimeouts(platformHandle, &timeouts))
    {
        // SN-8650: GetCommTimeouts() is a Win32 API -- GetLastError(), not errno; see above.
        serialPort->errorCode = (int)GetLastError();
        serialPort->error = "GetCommTimeouts() failed";
        log_error(IS_LOG_PORT, "[%s] serialPortOpenPlatform() failed to retrieve COMM port timeouts: %s (%d)", portName, serialPort->error, serialPort->errorCode);
        CloseHandle(platformHandle);  // serialPort->handle not yet assigned; close raw handle directly
        return 0;
    }

    // COMMTIMEOUTS timeouts = { (blocking ? 1 : MAXDWORD), (blocking ? 1 : 0), (blocking ? 1 : 0), (blocking ? 1 : 0), (blocking ? 10 : 0) };

    if (blocking)
    {
        // For blocking, ReadFile returns immediately with whatever is in the buffer.
        // The read loop in serialPortReadTimeoutPlatformWindows will poll.
        timeouts.ReadIntervalTimeout = 1;
        timeouts.ReadTotalTimeoutMultiplier = 1;
        timeouts.ReadTotalTimeoutConstant = 1;
    }
    else // non-blocking
    {
        // For non-blocking (overlapped), we want ReadFile to pend if no data is available.
        // Setting read timeouts to 0 disables them, and the wait is handled by WaitForSingleObject.
        timeouts.ReadIntervalTimeout = MAXDWORD;
        timeouts.ReadTotalTimeoutMultiplier = 0;
        timeouts.ReadTotalTimeoutConstant = 0;
    }

    // Set a reasonable write timeout for both modes. - In our baud rates of 921600, these numbers should be more then sufficient to ensure data is sent
    timeouts.WriteTotalTimeoutConstant = 30;            // a minimum timeout of 30 milliseconds, per write (but maybe less to actually send) - this should support large sends upto 2.7k if necessary
    timeouts.WriteTotalTimeoutMultiplier = 0;           // plus an additional 0 milliseconds per byte sent

    if (!SetCommTimeouts(platformHandle, &timeouts))
    {
        // SN-8650: SetCommTimeouts() is a Win32 API -- GetLastError(), not errno; see above.
        serialPort->errorCode = (int)GetLastError();
        serialPort->error = "SetCommTimeouts() failed";
        log_error(IS_LOG_PORT, "[%s] serialPortOpenPlatform() failed to configure COMM port timeouts: %s (%d)", portName, serialPort->error, serialPort->errorCode);
        CloseHandle(platformHandle);  // serialPort->handle not yet assigned; close raw handle directly
        return 0;
    }

    serialPortHandle* handle = (serialPortHandle*)calloc(sizeof(serialPortHandle), 1);
    handle->blocking = blocking;
    handle->platformHandle = platformHandle;
    if (!blocking)
    {
        handle->ovRead.hEvent = CreateEvent(0, 1, 0, 0);
        handle->ovWrite.hEvent = CreateEvent(0, 1, 0, 0);
    }
    serialPort->handle = handle;

#else

    int fd = open(portName, O_RDWR | O_NOCTTY | O_NONBLOCK);     // enable read/write and disable flow control
    if (fd < 0)
    {
        serialPort->errorCode = errno;
        serialPort->error = strerror(serialPort->errorCode);
        // Stamp the generic base-port error so cross-transport consumers (e.g. UI
        // port-status markup) can see "this port failed to open" without having to
        // know about platform-specific errno values. The platform-specific detail
        // (errno + strerror) remains in serialPort->errorCode/error.
        serialPort->base.perror = PORT_ERROR__OPEN_FAILURE;
        log_error(IS_LOG_PORT, "[%s] serialPortOpenPlatform():: Error opening port: %s (%d)", portName, serialPort->error, serialPort->errorCode);
        return 0;
    }

    if (configure_serial_port(fd, baudRate) != 0)
    {
        serialPort->errorCode = errno;
        serialPort->error = strerror(serialPort->errorCode);
        serialPort->base.perror = PORT_ERROR__OPEN_FAILURE;
        log_error(IS_LOG_PORT, "[%s] serialPortOpenPlatform():: Error configuring port: %s (%d)", port, serialPort->error, serialPort->errorCode);
        return 0;
    }

    ioctl(fd, TIOCEXCL);            // Exclusive Access Mode: prevent other processes from opening the port while its open
    flock(fd, LOCK_EX | LOCK_NB);   // Exclusive & Non-Blocking Lock: prevent other process read/write of the fd file

    // Disable blocking port reads and writes.
    if (set_nonblocking(fd) != 0) 
    {
        close(fd);
        return 0;
    }

    serialPortHandle* handle = (serialPortHandle*)calloc(sizeof(serialPortHandle), 1);
    handle->fd = fd;
    handle->blocking = blocking;
    serialPort->handle = handle;

    // we're doing a quick and dirty check to make sure we can even attempt to read data successfully.  Some bad devices will fail here if they aren't initialized correctly
    uint8_t tmp = 0;
    if (serialPortReadTimeoutPlatform(port, &tmp, 1, 10) < 0) {
        if (serialPort->errorCode == ENOENT) {
            serialPortClose(port);
            return 0;
        }
    }

#endif

    return 1;    // success
}

/**
 * @brief Check if the serial port is open.
 * This function checks if the serial port is open and valid.
 * On Windows, it uses `GetCommState`.
 * On other platforms, it uses `fstat`.
 *
 * @param port The port handle.
 * @return int 1 if open, 0 if not.
 */
static int serialPortIsOpenPlatform(port_handle_t port)
{
    serial_port_t* serialPort = (serial_port_t*)port;
    if (!serialPort->handle)
        return 0;

    // SN-8650: opportunistic flush, driven by this (very frequently polled) activity rather
    // than waiting for the port's next error.
    serialPortCheckActivity(port, &((serialPortHandle*)serialPort->handle)->errDedup);

    log_more_debug(IS_LOG_PORT, "[%s] serialPortIsOpenPlatform() called.", portName(port));

#if PLATFORM_IS_WINDOWS

    DCB serialParams;
    serialParams.DCBlength = sizeof(DCB);
    serialPortHandle* handle = (serialPortHandle*)serialPort->handle;
    return GetCommState(handle->platformHandle, &serialParams);

#else

    struct stat sb;
    if (fstat(((serialPortHandle*)(serialPort->handle))->fd, &sb) != 0) {
        serialPort->errorCode = errno;
        serialPort->error = strerror(serialPort->errorCode);
        return 0;
    }
    return 1; // return success
#endif

}

/**
 * @brief Close the serial port.
 * This function closes the serial port and frees the associated handle.
 * On Windows, it cancels any pending I/O and closes the handle.
 * On other platforms, it simply closes the file descriptor.
 *
 * @param port The port handle.
 * @return int 1 on success, 0 on failure.
 */
static int serialPortClosePlatform(port_handle_t port)
{
    serial_port_t* serialPort = (serial_port_t*)port;
    serialPortHandle* handle = (serialPortHandle*)serialPort->handle;
    if (handle == 0)
    {
        // not open, no close needed
        return 0;
    }

    log_debug(IS_LOG_PORT, "[%s] serialPortClosePlatform() called.", portName(port));

    // SN-8650: flush any still-accumulating duplicate-error run before the handle (and its
    // dedup state) is freed below, so the last burst on this handle isn't silently dropped.
    serialPortFlushPendingErrorDedup(port, &handle->errDedup);

    #if PLATFORM_IS_WINDOWS

    //DWORD dwRead = 0;
    //DWORD error = 0;

    CancelIo(handle->platformHandle);
    //GetOverlappedResult(handle->platformHandle, &handle->ovRead, &dwRead, 1);
    /*if ((error = GetLastError()) != ERROR_SUCCESS)
    {
        while (1) {}
    }*/
    if (!handle->blocking)
    {
        CloseHandle(handle->ovRead.hEvent);
        CloseHandle(handle->ovWrite.hEvent);
    }
    CloseHandle(handle->platformHandle);
    handle->platformHandle = 0;

#else

    // we need to do some extended checking here... It seems linux/posix close(fd) can block if data is in the TX buffer, but can't be sent
    if (set_flowcontrol(handle->fd, 0) == 0) {
        // no point is doing this if the previous call had an error -- this will probably fail too.
        if (tcflush(handle->fd, TCIOFLUSH) < 0) {
            // something bad happened...
            serialPort->errorCode = errno;
            serialPort->error = strerror(serialPort->errorCode);
            // FIXME: This should *probably* be a silent error - if we're closing a port that was previously closed or lost (USB disconnected, device reset, etc), then do we need to report an error?
            log_error(IS_LOG_PORT, "[%s] serialPortClosePlatform():: Error flushing: %s (%d)", portName(port), serialPort->error, serialPort->errorCode);
        }
    }

    // Clear HUPCL so the kernel doesn't drop DTR on close. USB CDC-ACM
    // devices (IMX-5, GPX-1) treat a DTR transition as a reset, which
    // used to force a 500 ms settling wait before the port could be
    // reopened. Preserving DTR across close removes that need.
    struct termios tty;
    if (tcgetattr(handle->fd, &tty) == 0) {
        tty.c_cflag &= ~HUPCL;
        tcsetattr(handle->fd, TCSANOW, &tty);
    }

    close(handle->fd);
    handle->fd = -1;

#endif

    free(serialPort->handle);
    serialPort->handle = 0;

    return 1;
}

/**
 * @brief Flush the serial port.
 * This function clears the serial port's receive buffer.
 * On Windows, it uses `PurgeComm` with `PURGE_RXCLEAR`.
 * On other platforms, it uses `tcflush` with `TCIOFLUSH`.
 *
 * @param port The port handle.
 * @return int 1 on success, 0 on failure.
 */
static int serialPortFlushPlatform(port_handle_t port)
{
    serial_port_t* serialPort = (serial_port_t*)port;
    serialPortHandle* handle = (serialPortHandle*)serialPort->handle;
    if (handle == 0)
    {
        // not open, no close needed
        return 0;
    }

    serialPortCheckActivity(port, &handle->errDedup);   // SN-8650: opportunistic flush

    log_more_debug(IS_LOG_PORT, "[%s] serialPortFlushPlatform() called.", portName(port));

#if PLATFORM_IS_WINDOWS

    // Use PurgeComm to clear receive (RX) buffer.
    if (!PurgeComm(handle->platformHandle, PURGE_RXCLEAR))
    {
        // SN-8650: PurgeComm() is a Win32 API -- GetLastError(), not errno; see the CreateFileA()
        // comment in serialPortOpenPlatform() for the same mistake. Routed through
        // serialPortReportError() so a burst of these collapses to one line instead of one per call.
        serialPort->errorCode = (int)GetLastError();
        serialPort->error = "PurgeComm() failed";
        serialPortReportError(port, &handle->errDedup, "serialPortFlushPlatform():: Error flushing: PurgeComm() failed", serialPort->errorCode, NULL);
        return 0;
    }

#else

    if (tcflush(handle->fd, TCIOFLUSH) < 0) {
        serialPort->errorCode = errno;
        serialPort->error = strerror(serialPort->errorCode);
        serialPortReportError(port, &handle->errDedup, "serialPortFlushPlatform():: Error flushing: tcflush() failed", serialPort->errorCode, serialPort->error);
    }

#endif

    return 1;
}

/**
 * @brief Drain the serial port.
 * This function waits for all written data to be transmitted.
 * On Windows, it uses `PurgeComm` with `PURGE_TXCLEAR`.
 * On other platforms, it uses `tcdrain`.
 *
 * @param port The port handle.
 * @return int 1 on success, 0 on failure.
 */
static int serialPortDrainPlatform(port_handle_t port)
{
    serial_port_t* serialPort = (serial_port_t*)port;
    serialPortHandle* handle = (serialPortHandle*)serialPort->handle;
    if (handle == 0)
    {
        // not open, no close needed
        return 0;
    }

    serialPortCheckActivity(port, &handle->errDedup);   // SN-8650: opportunistic flush

    log_more_debug(IS_LOG_PORT, "[%s] serialPortDrainPlatform() called.", portName(port));

#if PLATFORM_IS_WINDOWS

    // Use PurgeComm to clear transmit (TX) buffer.
    if (!PurgeComm(handle->platformHandle, PURGE_TXCLEAR))
    {
        // SN-8650: PurgeComm() is a Win32 API -- GetLastError(), not errno; see
        // serialPortOpenPlatform()'s CreateFileA() comment for the same mistake.
        serialPort->errorCode = (int)GetLastError();
        serialPort->error = "PurgeComm() failed";
        serialPortReportError(port, &handle->errDedup, "serialPortDrainPlatform():: Error draining: PurgeComm() failed", serialPort->errorCode, NULL);
        return 0;
    }

#else

    if (tcdrain(handle->fd) < 0) {
        serialPort->errorCode = errno;
        serialPort->error = strerror(serialPort->errorCode);
        serialPortReportError(port, &handle->errDedup, "serialPortDrainPlatform():: Error draining: tcdrain() failed", serialPort->errorCode, serialPort->error);
    }

#endif

    return 1;
}


#if PLATFORM_IS_WINDOWS

/**
 * @brief Reads a specified number of bytes from the serial port with a timeout.
 * This function reads from the serial port, handling both blocking and non-blocking (overlapped) I/O.
 * For non-blocking I/O, it uses `WaitForSingleObject` to wait for the read to complete.
 * If the read times out, it cancels the I/O and returns the bytes read so far.
 *
 * @param serialPort The serial port (used only to record errorCode/error on a hard failure -- see
 *   the two SN-8697 comments below; every other line is unchanged from the original handle-only form).
 * @param buffer    Pointer to the destination buffer.
 * @param readCount   The number of bytes requested to read.
 * @param timeoutMilliseconds Maximum time to wait in milliseconds.
 * @return          The actual number of bytes read (may be less than readCount on timeout), or -1
 *   if ReadFile()/GetOverlappedResult() failed outright (not a timeout) -- matching the negative-on-
 *   hard-failure contract serialPortReadTimeoutPlatformLinux() already uses.
 */
static int serialPortReadTimeoutPlatformWindows(serial_port_t* serialPort, unsigned char* buffer, int readCount, int timeoutMilliseconds)
{
    serialPortHandle* handle = (serialPortHandle*)serialPort->handle;

    if (readCount < 1)
    {
        return 0;
    }

    DWORD dwRes = 0;
    DWORD dwRead = 0;
    int totalRead = 0;
    ULONGLONG startTime = GetTickCount64();
    do {
        if (ReadFile(handle->platformHandle, buffer + totalRead, readCount - totalRead, &dwRead, !handle->blocking ? &handle->ovRead : 0)) {
            dwRes = GetLastError();
            if (!handle->blocking) {
                GetOverlappedResult(handle->platformHandle, &handle->ovRead, &dwRead, 1);
            }
            totalRead += dwRead;
        } else if (!handle->blocking) {
            dwRes = GetLastError();
            if (dwRes == ERROR_IO_PENDING) {
                dwRes = WaitForSingleObject(handle->ovRead.hEvent, _MAX(5, timeoutMilliseconds - (int)(GetTickCount64() - startTime)));
                switch (dwRes) {
                    case WAIT_OBJECT_0:
                        if (!GetOverlappedResult(handle->platformHandle, &handle->ovRead, &dwRead, 1)) {
                            // SN-8697: this failure was previously discarded silently (just
                            // CancelIo(), no error recorded, loop continues as if nothing
                            // happened). Propagate it so a genuinely lost device (e.g. mid-reboot)
                            // is observable via errorCode instead of looking identical to a
                            // benign short read.
                            DWORD result = GetLastError();
                            serialPort->errorCode = (int)result;
                            serialPort->error = "GetOverlappedResult() failed";
                            serialPortReportError((port_handle_t)serialPort, &handle->errDedup, "serialPortReadTimeoutPlatform():: Error fetching 'overlapped result': GetOverlappedResult() failed", serialPort->errorCode, NULL);
                            CancelIo(handle->platformHandle);
                            return -1;
                        }
                        else
                            totalRead += dwRead;
                        break;
                    case WAIT_TIMEOUT:
                    default:
                        // cancel io and just take whatever was in the buffer
                        CancelIo(handle->platformHandle);
                        GetOverlappedResult(handle->platformHandle, &handle->ovRead, &dwRead, 0);
                        totalRead += dwRead;
                        break;
                }
            } else {
                // SN-8697: ReadFile() failing outright (not ERROR_IO_PENDING) was previously
                // discarded silently the same way -- see the comment above.
                serialPort->errorCode = (int)dwRes;
                serialPort->error = "ReadFile() failed";
                serialPortReportError((port_handle_t)serialPort, &handle->errDedup, "serialPortReadTimeoutPlatform():: Error reading: ReadFile() failed", serialPort->errorCode, NULL);
                CancelIo(handle->platformHandle);
                return -1;
            }
        }
    } while ((totalRead < readCount) && (GetTickCount64() - startTime < timeoutMilliseconds));

    return totalRead;
}

#else

/**
 * @brief Read from the serial port with a timeout on Linux.
 * This function reads from the serial port using `poll` to wait for data to become available.
 * It loops until the requested number of bytes are read or the timeout is reached.
 * It handles `EAGAIN` and `EWOULDBLOCK` errors by continuing to try and read.
 *
 * When a timeout occurs, this function will return the number of bytes received so far -
 * this may mean that the function returns 0 or a positive number less than readCount. This
 * is NOT an error condition, since zero or more bytes, had they been available, could have
 * been read.
 *
 * @param serialPort The serial port.
 * @param buffer The buffer to read into.
 * @param readCount The number of bytes to read.
 * @param timeoutMilliseconds The timeout in milliseconds.
 * @return int The number of bytes read, or a PORT_ERROR__* code if an error occurred.
 */
static int serialPortReadTimeoutPlatformLinux(serial_port_t* serialPort, unsigned char* buffer, int readCount, int timeoutMs)
{
    int totalRead = 0;
    int dtMs = 0;
    int n = 0;
    struct timeval start, curr;

    if (!serialPort || !serialPort->handle || !buffer)
        return PORT_ERROR__INVALID_PARAMETER;

    serialPortHandle* handle = (serialPortHandle*)serialPort->handle;
    gettimeofday(&start, NULL);

    while (1) {
        if (timeoutMs > 0) {
            struct pollfd fds[1];
            fds[0].fd = handle->fd;
            fds[0].events = POLLIN;

            // we will poll, for upto timeoutMs for any number of bytes.
            int pollrc = poll(fds, 1, timeoutMs);
            if (pollrc <= 0 || !(fds[0].revents & POLLIN)) {
                if (fds[0].revents & POLLIN) {
                    // do nothing - we'll fall thru to the read() below...
                } else if (fds[0].revents & POLLERR) {
                    return PORT_ERROR__READ_FAILURE; // more than a timeout occurred.
                } else {
                    break;  // no data before timeout expired
                }
            }
        }

        if ((n = read(handle->fd, buffer + totalRead, readCount - totalRead)) < 0) {
            if ((errno != EAGAIN) && (errno != EWOULDBLOCK)) {
                // SN-8650 (review follow-up): record the error, but don't log it here. The only
                // caller, serialPortReadTimeoutPlatform(), re-derives the same errno/strerror()
                // immediately after this returns and reports it through serialPortReportError()
                // -- the single SN-8650 dedup gate. Logging unconditionally here as well meant
                // every repeated read failure was reported twice: once ungated (right here, on
                // every single occurrence) and once through the gate (suppressed after the
                // first), so the read path was never actually deduplicated -- the ungated line
                // flooded regardless.
                serialPort->errorCode = errno;
                serialPort->error = strerror(errno);
            }
            return PORT_ERROR__TIMEOUT;
        } else if (n > 0) {
            totalRead += n;
        }

        if ((timeoutMs > 0) && (totalRead < readCount))
        {
            gettimeofday(&curr, NULL);
            dtMs = ((curr.tv_sec - start.tv_sec) * 1000) + ((curr.tv_usec - start.tv_usec) / 1000);
            if (dtMs >= timeoutMs)
            {
                break;
            }

            // try for another loop around with a lower timeout
            timeoutMs = _MAX(0, timeoutMs - dtMs);
        }
        else
        {
            break;
        }
    }
    // debugDumpBuffer("{{ ", buffer, totalRead);
    return totalRead;
}

#endif

/**
 * @brief Read from the serial port with a timeout.
 * This function reads a specified number of bytes from the serial port, with a timeout.
 * It is a wrapper around the platform-specific read functions.
 * If the timeout is negative, a default timeout is used.
 *
 * @param port The port handle.
 * @param buffer The buffer to read into.
 * @param readCount The number of bytes to read.
 * @param timeoutMilliseconds The timeout in milliseconds.
 * @return int The number of bytes read, or -1 on error.
 */
static int serialPortReadTimeoutPlatform(port_handle_t port, unsigned char* buffer, unsigned int readCount, int timeoutMs)
{
    log_bombastic(IS_LOG_PORT, "[%s] serialPortReadTimeoutPlatform() called.", portName(port));

    serial_port_t* serialPort = (serial_port_t*)port;
    serialPortHandle* handle = (serialPortHandle*)serialPort->handle;
    if (!handle) {
        serialPort->errorCode = ENOENT;
        serialPort->error = "Internal port handle is NULL; Port is closed.";
        return PORT_ERROR__NOT_CONNECTED;
    }

    serialPortCheckActivity(port, &handle->errDedup);   // SN-8650: opportunistic flush

    if (timeoutMs < 0)
    {
        timeoutMs = (handle->blocking ? SERIAL_PORT_DEFAULT_TIMEOUT : 0);
    }

#if PLATFORM_IS_WINDOWS
    int result = serialPortReadTimeoutPlatformWindows(serialPort, buffer, readCount, timeoutMs);

    // SN-8697: serialPortReadTimeoutPlatformWindows() already records the real Win32 error
    // directly into errorCode/error on a hard failure (matching how serialPortWritePlatform()
    // handles WriteFile()/GetOverlappedResult() failures) -- errno is a CRT global unrelated to a
    // WinAPI failure, so re-reading it here (the way the POSIX branch below does) would clobber a
    // correctly-set error with a meaningless value. Only the success case needs handling here.
    if (result >= 0) {
        serialPort->errorCode = 0; // clear any previous errorcode
        serialPort->error = NULL;
    } else if (win32ErrorIndicatesDeviceLost((DWORD)serialPort->errorCode)) {
        // SN-8697: mirror the write path — a device-lost read failure must invalidate the port
        // so the firmware updater can detect the disconnect and rediscover the device.
        portClose(port);
        portInvalidate(port);
    }
#else
    int result = serialPortReadTimeoutPlatformLinux(serialPort, buffer, readCount, timeoutMs);

    if ((result < 0) && !((errno == EAGAIN) && !handle->blocking)) {
        serialPort->errorCode = errno;  // NOTE: If you are here looking at errno = -11 (EAGAIN) remember that if this is a non-blocking tty, returning EAGAIN on a read() just means there was no data available.
        serialPort->error = strerror(serialPort->errorCode);
        serialPortReportError(port, &handle->errDedup, "serialPortReadTimeoutPlatform():: Error reading", serialPort->errorCode, serialPort->error);
    } else {
        serialPort->errorCode = 0; // clear any previous errorcode
        serialPort->error = NULL;
    }
#endif

    log_bombastic(IS_LOG_PORT, "[%s] serialPortReadTimeoutPlatform() received %d bytes", portName(port), result);
    debugDumpBuffer("<< ", buffer, result);
    return result;
}

/**
 * @brief Read from the serial port.
 * This function is a convenience wrapper around `serialPortReadTimeoutPlatform` with a timeout of 0.
 * This means it will return immediately with any available data.
 *
 * @param port The port handle.
 * @param buffer The buffer to read into.
 * @param readCount The number of bytes to read.
 * @return int The number of bytes read, or -1 on error.
 */
static int serialPortReadPlatform(port_handle_t port, unsigned char* buffer, unsigned int readCount) {
    return serialPortReadTimeoutPlatform(port, buffer, readCount, 0);
}

/**
 * @brief Asynchronously read from the serial port.
 * This function initiates an asynchronous read from the serial port.
 * On Windows, it uses `ReadFileEx` and a completion routine.
 * On other platforms, it performs a simple blocking read and calls the completion routine directly.
 *
 * @param port The port handle.
 * @param buffer The buffer to read into.
 * @param readCount The number of bytes to read.
 * @param completion The completion routine.
 * @return int 1 on success, -1 on failure.
 */
static int serialPortAsyncReadPlatform(port_handle_t port, unsigned char* buffer, unsigned int readCount, pfnSerialPortAsyncReadCompletion completion)
{
    serial_port_t* serialPort = (serial_port_t*)port;
    serialPortHandle* handle = (serialPortHandle*)serialPort->handle;
    if (!handle) {
        serialPort->errorCode = ENODEV;
        serialPort->error = strerror(serialPort->errorCode);
        return -1;
    }

    serialPortCheckActivity(port, &handle->errDedup);   // SN-8650: opportunistic flush

#if PLATFORM_IS_WINDOWS

    readFileExCompletionStruct c;
    c.externalCompletion = completion;
    c.port = port;
    c.buffer = buffer;
    memset(&(c.ov), 0, sizeof(c.ov));

    if (!ReadFileEx(handle->platformHandle, buffer, readCount, (LPOVERLAPPED)&c, readFileExCompletion))
    {
        return 0;
    }

#else

    // no support for async, just call the completion right away
    int n = read(handle->fd, buffer, readCount);
    if (n < 0) {
        serialPort->errorCode = errno;
        serialPort->error = strerror(serialPort->errorCode);
    }

    completion(port, buffer, (n < 0 ? 0 : n), (n >= 0 ? 0 : n));

#endif

    return 1;
}

#if PLATFORM_IS_WINDOWS
/**
 * @brief Whether a Win32 error code indicates the underlying serial device/handle no longer exists
 * (SN-8697, revised SN-8650).
 *
 * A read/write failure is never a general indicator of an invalid port -- often it is, but not
 * always, and it depends entirely on the nature of the failure. The test that matters is whether the
 * code's own documented meaning asserts that the object (device/handle) no longer exists -- the Win32
 * analogue of POSIX ENOENT/EBADF -- versus merely that this one I/O attempt could not complete for
 * some local or transient reason (a full buffer, a timeout, a cancelled operation). Only the former
 * belongs here; the latter should be reported back to the caller as an ordinary failed attempt; a
 * caller that wants to retry (contention, or any other recoverable cause) remains free to.
 *
 * Kept (explicit "does not exist" semantics, confirmed against Win32/driver documentation):
 *  - ERROR_NOT_SAME_DEVICE, 433 (undocumented STATUS_NO_SUCH_DEVICE) -- pre-existing checks.
 *  - ERROR_DEVICE_NOT_CONNECTED -- returned specifically when a device disappears mid-transfer.
 *  - ERROR_INVALID_HANDLE -- the handle itself, which only this code owns the lifecycle of, is no
 *    longer valid; not explained by contention or a busy peer.
 *
 * Removed (SN-8650; do not re-add without re-litigating this comment):
 *  - ERROR_SEM_TIMEOUT -- "the semaphore timeout period has expired": a wait-didn't-complete-in-time
 *    signal, not a device-existence signal. Fires on a still-present device that's simply slow to
 *    answer (field case: many devices sharing one USB hub/testbed under write contention). This is
 *    true regardless of transport or of whether the target happens to be rebooting -- a write
 *    timing out during contention is not evidence about the port at all, on any transport.
 *  - ERROR_GEN_FAILURE -- "a device attached to the system is not functioning": commonly reported for
 *    a blocked/stalled USB-CDC link (a driver-level hiccup), not confirmed device absence; widely
 *    documented as clearing on its own or via replug without the device having actually left.
 *  - ERROR_OPERATION_ABORTED -- Microsoft's own documentation: "this is not usually a hardware
 *    failure" and is the expected, routine result of ANY CancelIo(), for reasons unrelated to device
 *    removal (a thread exiting, a handle closing, or the OS aborting a pending op for its own
 *    unrelated reasons) -- not exclusively an OS-initiated abort due to the device going away, contra
 *    the reasoning this code carried before.
 */
static int win32ErrorIndicatesDeviceLost(DWORD err)
{
    switch (err)
    {
    case ERROR_NOT_SAME_DEVICE:        // pre-existing check
    case 433:                          // undocumented STATUS_NO_SUCH_DEVICE, pre-existing check
    case ERROR_DEVICE_NOT_CONNECTED:
    case ERROR_INVALID_HANDLE:
        return 1;
    default:
        return 0;
    }
}
#endif // PLATFORM_IS_WINDOWS

/**
 * @brief Write to the serial port.
 * This function writes a buffer of data to the serial port.
 * On Windows, it handles overlapped I/O for non-blocking writes.
 * On other platforms, it retries on partial writes and handles `EINTR`, `EAGAIN`, and `EWOULDBLOCK` errors.
 *
 * @param port The port handle.
 * @param buffer The buffer to write from.
 * @param writeCount The number of bytes to write.
 * @return int The number of bytes written, or -1 on error.
 */
static int serialPortWritePlatform(port_handle_t port, const unsigned char* buffer, unsigned int writeCount)
{
    log_bombastic(IS_LOG_PORT, "[%s] serialPortWritePlatform() called.", portName(port));

    serial_port_t* serialPort = (serial_port_t*)port;
    serialPortHandle* handle = (serialPortHandle*)serialPort->handle;
    if (!handle) {
        serialPort->errorCode = ENODEV;
        serialPort->error = strerror(serialPort->errorCode);
        return -1;
    }

    serialPortCheckActivity(port, &handle->errDedup);   // SN-8650: opportunistic flush -- this is
                                                         // the dominant call site in practice, since
                                                         // writes happen every comm tick regardless
                                                         // of whether the previous one failed.

#if PLATFORM_IS_WINDOWS

    DWORD dwWritten;
    if (!WriteFile(handle->platformHandle, buffer, writeCount, &dwWritten, !handle->blocking ? &handle->ovWrite : 0))
    {
        DWORD result = GetLastError();
        if (result != ERROR_IO_PENDING)
        {
            // SN-8697: errorCode must carry the real Win32 error (result), not errno -- errno is
            // an unrelated CRT global on Windows and holds whatever a prior C-runtime call left it
            // at, not this WriteFile()'s failure.
            serialPort->errorCode = (int)result;
            serialPort->error = "WriteFile() failed";
            // SN-8650: this is the dominant source of log noise under multi-device USB-hub
            // contention (one failure per comm tick, per port, for as long as the contention
            // lasts) -- route through serialPortReportError() to collapse a burst into one
            // ERROR line plus a trailing summary instead of one ERROR line per occurrence.
            serialPortReportError(port, &handle->errDedup, "serialPortWrite():: Error writing: WriteFile() failed", serialPort->errorCode, NULL);
            CancelIo(handle->platformHandle);
            if (win32ErrorIndicatesDeviceLost(result)) {
                // this indicates the handle is invalid. The port should be closed and invalidated.
                portClose(port);
                portInvalidate(port);
            }
            return 0;
        }
    }

    if (!handle->blocking)
    {
        if (!GetOverlappedResult(handle->platformHandle, &handle->ovWrite, &dwWritten, 1))
        {
            DWORD result = GetLastError();  // read this before we call CancelIo
            serialPort->errorCode = (int)result;   // SN-8697: real Win32 error, not errno -- see above
            serialPort->error = "GetOverlappedResult() failed";
            serialPortReportError(port, &handle->errDedup, "serialPortWrite():: Error fetching 'overlapped result': GetOverlappedResult() failed", serialPort->errorCode, NULL);
            CancelIo(handle->platformHandle);
            if (win32ErrorIndicatesDeviceLost(result)) {
                // this indicates the handle is invalid. The port should be closed and invalidated.
                portClose(port);
                portInvalidate(port);
            }
            return 0;
        }
    }

    if (dwWritten != writeCount)
        log_bombastic(IS_LOG_PORT, "[%s] serialPortWritePlatform() wrote %d bytes (%d requested)", portName(port), dwWritten, writeCount);

    debugDumpBuffer(">> ", buffer, dwWritten);
    return dwWritten;

#else

    struct stat sb;
    errno = 0;
    if (fstat(((serialPortHandle*)serialPort->handle)->fd, &sb) != 0)
    {   // Serial port not open
        serialPort->errorCode = errno;
        serialPort->error = strerror(serialPort->errorCode);
        return 0;
    }

    // Ensure all data is queued by OS for sending.  This step is necessary because of O_NONBLOCK non-blocking mode. 
    // Note that this only blocks for partial writes until the OS accepts all input data.  This does NOT block until 
    // the data is physically transmitted.
    uint32_t bytes_written = 0, retry = 0;
    while ((bytes_written < writeCount) && (retry < 10))
    {
        ssize_t result = write(handle->fd, buffer + bytes_written, writeCount - bytes_written);
        if (result < 0) 
        {
            if ((errno == EINTR) ||     // Interrupted by signal, continue writing
                (errno == EAGAIN) || (errno == EWOULDBLOCK))  // Non-blocking mode, and no data written, continue trying
            {
                serialPortSleepPlatform(1);
                retry++;
                continue;
            }
            // Other errors
            serialPort->errorCode = errno;
            serialPort->error = strerror(serialPort->errorCode);
            serialPortReportError(port, &handle->errDedup, "serialPortWritePlatform():: Error writing", serialPort->errorCode, serialPort->error);
            if ((errno == ENOENT) || (errno == ENODEV) || (errno ==  EIO)) {
                // these errors indicate the underlying OS port is bad, and needs to be closed/invalidated - there is usually no other recovery from here.
                portClose(port);
                portInvalidate(port);
            }
            return -1;
        }
        bytes_written += result;
    }

    if (handle->blocking)
    {   // Block until output data has been physically transmitted 
        int error = tcdrain(handle->fd);
        if (error != 0)
        {   // Drain error
            // TODO: report the error (probably as a warning)
            return 0;
        }
    }

    debugDumpBuffer(">> ", buffer, bytes_written);
    return bytes_written;

#endif

}

/**
 * @brief Get the number of bytes available to read from the serial port.
 * This function returns the number of bytes available to be read from the serial port.
 * On Windows, it uses `ClearCommError` and the `COMSTAT` structure.
 * On other platforms, it uses `poll` and `ioctl` with `FIONREAD`.
 *
 * @param port The port handle.
 * @return int The number of bytes available to read, PORT_ERROR__INVALID if the port
 *         handle is invalid, or PORT_ERROR__NOT_CONNECTED if the internal handle is NULL
 *         (i.e., the port has not been opened or has been closed).
 */
static int serialPortGetByteCountAvailableToReadPlatform(port_handle_t port)
{
    if (!port || !portIsValid(port))
        return PORT_ERROR__INVALID;

    log_bombastic(IS_LOG_PORT, "[%s] serialPortGetByteCountAvailableToReadPlatform() called.", portName(port));

    serial_port_t* serialPort = (serial_port_t*)port;
    serialPortHandle* handle = (serialPortHandle*)serialPort->handle;
    if (!handle)
    {
        serialPort->errorCode = ENOENT;
        serialPort->error = "Internal port handle is NULL; Port is closed.";
        return PORT_ERROR__NOT_CONNECTED;
    }

    serialPortCheckActivity(port, &handle->errDedup);   // SN-8650: opportunistic flush

#if PLATFORM_IS_WINDOWS

    COMSTAT commStat;
    if (ClearCommError(handle->platformHandle, 0, &commStat))
    {
        return commStat.cbInQue;
    }
    return 0;

#else

    int bytesAvailable = 0;
    struct pollfd p = { .fd = handle->fd, .events = POLLIN };
    int rc;

again:
    rc = poll(&p, 1, 0);
    if (rc > 0) {
        /* Treat POLLIN or urgent/hangup with data as readable */
        if (p.revents & (POLLIN | POLLPRI)) {
            if (ioctl(handle->fd, FIONREAD, &bytesAvailable) < 0) {
                serialPort->errorCode = errno;
                serialPort->error = strerror(serialPort->errorCode);
            }
            return bytesAvailable;
        }
        if (p.revents & (POLLHUP | POLLERR | POLLNVAL)) {
            errno = EIO;
            return -1;
        }
        return 0; // unexpected, but keep contract
    } else if (rc == 0) {
        return 0; // timeout
    } else { // rc < 0
        if (errno == EINTR) goto again;
        return -1;
    }
#endif

}

/**
 * @brief Get the number of bytes available to write to the serial port.
 * This function returns the number of bytes that can be written to the serial port without blocking.
 * Currently, it returns a fixed value of 65536.
 * The commented-out code shows how it could be implemented on Linux using `ioctl`.
 *
 * @param port The port handle.
 * @return int The number of bytes available to write, or PORT_ERROR__INVALID on error.
 */
static int serialPortGetByteCountAvailableToWritePlatform(port_handle_t port)
{
    if (!port || !portIsValid(port))
        return PORT_ERROR__INVALID;

    log_bombastic(IS_LOG_PORT, "[%s] serialPortGetByteCountAvailableToWritePlatform() called.", portName(port));

    serial_port_t* serialPort = (serial_port_t*)port;
    // serialPortHandle* handle = (serialPortHandle*)serialPort->handle;
    (void)serialPort;

    return 65536;

    /*
    int bytesUsed;
    struct serial_struct serinfo;
    memset(&serinfo, 0, sizeof(serial_struct));
    ioctl(handle->fd, TIOCGSERIAL, &serinfo);
    ioctl(handle->fd, TIOCOUTQ, &bytesUsed);
    return serinfo.xmit_fifo_size - bytesUsed;
    */
}

/**
 * @brief Sleep for a specified number of milliseconds.
 * This function is a simple wrapper around the platform-specific sleep function.
 * On Windows, it uses `Sleep()`, and on other platforms, it uses `usleep()`.
 *
 * @param sleepMilliseconds The number of milliseconds to sleep.
 * @return int 1 on success.
 */
static int serialPortSleepPlatform(int sleepMilliseconds)
{
#if PLATFORM_IS_WINDOWS

    Sleep(sleepMilliseconds);

#else

    usleep(sleepMilliseconds * 1000);

#endif

    return 1;
}

/**
 * @brief Initialize the serial port platform.
 * This function initializes the serial port structure with platform-specific function pointers.
 * It also sets the default baud rate and initializes the base port structure.
 * It is important that the serial port structure is zeroed out before calling this function.
 *
 * @param port The port handle.
 * @return int 0 on success.
 */
int serialPortPlatformInit(port_handle_t port) // unsigned int portOptions
{
    serial_port_t* serialPort = (serial_port_t*)port;
    // very important - the serial port must be initialized to zeros
    base_port_t tmp = { .pnum = portId(port), .ptype = portType(port), .pflags = portFlags(port), .chksum = BASE_PORT(port)->chksum };

    // FIXME:  I really don't like this having to copy and clean, and copy back.  It shouldn't be necessary.
    char tmpName[64] = {0};
    memcpy(tmpName, serialPort->portName, _MIN(sizeof(serialPort->portName), sizeof(tmpName)));
    memset(serialPort, 0, sizeof(serial_port_t));
    memcpy(serialPort->portName, tmpName, _MIN(sizeof(serialPort->portName), sizeof(tmpName)));
    log_more_debug(IS_LOG_PORT, "serialPortPlatformInit() called [%s].", serialPort->portName);

    serialPort->base = tmp;
    portRecalcChksum(port);

    serialPort->base.portName = serialPortName;
    // serialPort->base.portValidate = serialPortValidate;
    serialPort->base.portOpen = serialPortOpen_internal;
    serialPort->base.portClose = serialPortClose;
    serialPort->base.portFree = serialPortGetByteCountAvailableToWrite;
    serialPort->base.portAvailable = serialPortGetByteCountAvailableToRead;
    serialPort->base.portFlush = serialPortFlush;
    serialPort->base.portDrain = serialPortDrain;
    serialPort->base.portRead = serialPortRead;
    serialPort->base.portWrite = serialPortWrite;
    serialPort->base.portReadTimeout = (pfnPortReadTimeout)serialPortReadTimeout;

    serialPort->base.stats = (port_stats_t*)&serialPort->stats;

    if (portType(port) & PORT_TYPE__COMM)
        is_comm_port_init(COMM_PORT(port), NULL);

    serialPort->baudRate = 921600; // default for InertialSense

    // platform specific functions
    serialPort->pfnOpen = serialPortOpenPlatform;
    serialPort->pfnIsOpen = serialPortIsOpenPlatform;
    serialPort->pfnReadTimeout = serialPortReadTimeoutPlatform;
    serialPort->pfnAsyncRead = serialPortAsyncReadPlatform;
    serialPort->pfnFlush = serialPortFlushPlatform;
    serialPort->pfnDrain = serialPortDrainPlatform;
    serialPort->pfnClose = serialPortClosePlatform;
    serialPort->pfnGetByteCountAvailableToWrite = serialPortGetByteCountAvailableToWritePlatform;
    serialPort->pfnGetByteCountAvailableToRead = serialPortGetByteCountAvailableToReadPlatform;
    serialPort->pfnRead = serialPortReadPlatform;
    serialPort->pfnWrite = serialPortWritePlatform;
    serialPort->pfnSleep = serialPortSleepPlatform;
    return 0;
}
