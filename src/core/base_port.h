/**
 * @file base_port.h 
 * @brief ${BRIEF_DESC}
 *
 * @author Kyle Mallory on 5/9/25.
 * @copyright Copyright (c) 2025 Inertial Sense, Inc. All rights reserved.
 */

#ifndef IS_CORE__BASE_PORT_H
#define IS_CORE__BASE_PORT_H

#include <string.h>

#include "core/types.h"
#include "core/msg_logger.h"
#include "ISConstants.h"

/**
 * Port definitions used across the entire product line & SDK.
 */
#define PORT_TYPE__UNKNOWN          0xFFFF      //!< Invalid or unknown port type
#define PORT_TYPE__UART             0x0001      //!< this port wraps the UART protocol
#define PORT_TYPE__USB              0x0002      //!< this port wraps the USB_CDC protocol
#define PORT_TYPE__SPI              0x0003      //!< this port wraps the SPI protocol
#define PORT_TYPE__CAN              0x0004      //!< this port wraps the CAN protocol
#define PORT_TYPE__TCP              0x0005      //!< this port wraps a TCP-based network socket
#define PORT_TYPE__UDP              0x0006      //!< this port wraps a UDP-based network socket
#define PORT_TYPE__FILE             0x0007      //!< this port wraps a OS file handle/stream
#define PORT_TYPE__LOOPBACK         0x00FE      //!< this port is a loopback to another port.
#define PORT_TYPE__COMM             0x1000      //!< this is a modifier for other port types, indicating that the port is a communication port, with a is_comm instance and message/packet parsing capabilities

#define PORT_FLAG__HDW              0x0001      //!< bit indicates that this port is static/hardware-defined
#define PORT_FLAG__VALID            0x0002      //!< bit indicates that this port programmatically marked as valid; this allows an implementation to mark invalidate a port
#define PORT_FLAG__OPENED           0x0004      //!< bit indicates that this port is opened, and able to process data
#define PORT_FLAG__NO_ISDEVICE      0x0008      //!< bit indicates that this port cannot be used to connect an ISDevice -- Calls to ISDevice::assignPort() will fail.
#define PORT_FLAG__GNSS             0x0010      //!< bit indicates that this port is a GNSS receiver port
#define PORT_FLAG__BLOCKING         0x0020      //!< bit indicates that this port uses blocking I/O

#define PORT_ERROR__NONE                 0      //!< No error and/or successful execution
#define PORT_ERROR__NOT_SUPPORTED       -1      //!< The operation requested/called was not support by the specified port
#define PORT_ERROR__INVALID             -2      //!< The port specified in the operation was invalid; The port_handle_t should probably be abandoned, or revalidated.
#define PORT_ERROR__NOT_CONNECTED       -3      //!< attempt to read/write/access a port failed because the port has not been opened
#define PORT_ERROR__OPEN_FAILURE        -4      //!< Attempt to open the port failed.
#define PORT_ERROR__WRITE_FAILURE       -5      //!< Attempt to write to the port failed.
#define PORT_ERROR__READ_FAILURE        -6      //!< Attempt to read from the port failed.
#define PORT_ERROR__TIMEOUT             -7      //!< The port operation reported a timeout (could be read, write, open, etc)
#define PORT_ERROR__INVALID_PARAMETER   -8      //!< The port was called with an invalid parameter

#define PORT_OP__READ               0x00        //!< A portLogger operation flag indicating a READ/RX was performed
#define PORT_OP__WRITE              0x01        //!< A portLogger operation flag indicating a WRITE/TX was performed
#define PORT_OP__OPEN               0x02        //!< A portLogger operation flag indicating a OPEN was performed
#define PORT_OP__CLOSE              0x03        //!< A portLogger operation flag indicating a CLOSE was performed
#define PORT_OP__FLUSH              0x04        //!< A portLogger operation flag indicating a FLUSH was performed (all pending RX data discarded)
#define PORT_OP__DRAIN              0x05        //!< A portLogger operation flag indicating a DRAIN was performed (block until all TX data is sent, and discard after timeout)

#define PORT_DEFAULT_TIMEOUT        1000        //!< A default timeout period (in milliseconds) for READ and other operations.

typedef void* port_handle_t;

typedef const char*(*pfnPortName)(port_handle_t port);
typedef int(*pfnPortValidate)(port_handle_t port);
typedef int(*pfnPortOpen)(port_handle_t port);
typedef int(*pfnPortClose)(port_handle_t port);
typedef int(*pfnPortFree)(port_handle_t port);
typedef int(*pfnPortAvailable)(port_handle_t port);
typedef int(*pfnPortFlush)(port_handle_t port);
typedef int(*pfnPortDrain)(port_handle_t port, uint32_t timeout);
typedef int(*pfnPortRead)(port_handle_t port, uint8_t* buf, unsigned int len);
typedef int(*pfnPortReadTimeout)(port_handle_t port, uint8_t* buf, unsigned int len, uint32_t timeout);
typedef int(*pfnPortWrite)(port_handle_t port, const uint8_t* buf, unsigned int len);
typedef int(*pfnPortLogger)(port_handle_t port, uint8_t op, const uint8_t* buf, unsigned int len, void* userData);
// typedef int(*pfnPortSetName)(port_handle_t port, const char* name, unsigned int len);

PUSH_PACK_1
typedef struct
{
    uint8_t         portInfo;               //!< High nib port type (see ePortMonPortType) low nib index
    uint32_t        status;                 //!< Status

    uint32_t        txBytesPerSec;          //!< Tx data rate (bytes/s)
    uint32_t        rxBytesPerSec;          //!< Rx data rate (bytes/s)

    uint32_t        txBytes;                //!< Tx byte count
    uint32_t        rxBytes;                //!< Rx byte count

    uint32_t        txDataDrops;            //!< Tx buffer data drop occurrences, times portWrite could not send all data */
    uint32_t        rxOverflows;            //!< Rx buffer overflow occurrences, times that the receive buffer reduced in size due to overflow */

    uint32_t        txBytesDropped;         //!< Tx number of bytes that were not sent
    uint32_t        rxChecksumErrors;       //!< Rx number of errors while reading (not bytes)
} port_stats_t;
POP_PACK


typedef struct base_port_s {
    uint16_t pnum;                          //!< an identifier for a specific port that belongs to this device
    uint16_t ptype;                         //!< an indicator of the type of port
    uint16_t pflags;                        //!< a bitmask of flags, indicating state of special capabilities for this port
    uint16_t perror;                        //!< a non-zero value indicating an error for the last operation attempted for this port

    pfnPortName portName;                   //!< a function which returns an optional name to (ideally) uniquely identify this port
    pfnPortValidate portValidate;           //!< a function which confirms the viability of the port - this does not open or connect the port
    pfnPortOpen portOpen;                   //!< a function to open/connect the specified port - may not be supported by all implementations
    pfnPortClose portClose;                 //!< a function to close/disconnect the specified port - may not be supported by all implementations
    pfnPortFree portFree;                   //!< a function which returns the number of bytes which can safely be written
    pfnPortAvailable portAvailable;         //!< a function which returns the number of bytes currently available, waiting to be read
    pfnPortFlush portFlush;                 //!< a function to flush all data currently waiting to be read
    pfnPortDrain portDrain;                 //!< a function to clear/drain all data currently waiting to be written/sent to the port
    pfnPortRead portRead;                   //!< a function to return copy some number of bytes available for reading into a local buffer (and removed from the ports read buffer)
    pfnPortReadTimeout portReadTimeout;     //!< a function to return copy some number of bytes available for reading into a local buffer (and removed from the ports read buffer), but will only block at most timeout milliseconds
    pfnPortWrite portWrite;                 //!< a function to copy some number of bytes from a local buffer into the ports write buffer (when and how this data is actually "sent" is implementation specific)
    pfnPortLogger portLogger;               //!< a function, if set, to be called anytime a portRead or portWrite call is made; used to monitor/copy all data that goes through the port

    void *portLoggerData;                   //!< an opaque pointer of "user data" associated with the portLogger that is passed whenever the portLogger() callback function is called
    uint32_t chksum;                        //!< to be valid:  chksum == ^~((pflags << 16) | ptype) - this is a validation mechanism to help ensure that this is a valid port
    port_stats_t* stats;                    //!< if not-null, contains the stats associated with this port (bytes sent/received, etc)
} base_port_t;
#define BASE_PORT(n)        ((base_port_t*)(n))

#ifdef __cplusplus
extern "C" {
#endif

int portReadTimeout_internal(port_handle_t port, uint8_t *buffer, unsigned int readCount, unsigned int timeoutMs);     // DO NOT EXPORT
int portWaitForTimeout(port_handle_t port, const uint8_t* waitFor, unsigned int waitForLength, unsigned int timeoutMs);
int portWaitFor(port_handle_t port, const uint8_t* waitFor, unsigned int waitForLength);

int portReadCharTimeout(port_handle_t port, unsigned char* c, unsigned int timeoutMs);
int portReadChar(port_handle_t port, unsigned char* c);

int portReadLineTimeout(port_handle_t port, unsigned char* buffer, unsigned int bufferLength, unsigned int timeoutMs);
int portReadLine(port_handle_t port, unsigned char* buffer, unsigned int bufferLength);

int portReadAsciiTimeout(port_handle_t port, unsigned char* buffer, unsigned int bufferLength, unsigned int timeoutMs, unsigned char** asciiData);
int portReadAscii(port_handle_t port, unsigned char* buffer, unsigned int bufferLength, unsigned char** asciiData);

int portWriteLine(port_handle_t port, const unsigned char* buffer, unsigned int writeCount);
int portWriteAscii(port_handle_t port, const char* buffer, unsigned int bufferLength);

int portWriteAndWaitForTimeout(port_handle_t port, const unsigned char* buffer, unsigned int writeCount, const unsigned char* waitFor, unsigned int waitForLength, const unsigned int timeoutMs);
int portWriteAndWaitFor(port_handle_t port, const unsigned char* buffer, unsigned int writeCount, const unsigned char* waitFor, unsigned int waitForLength);

/**
 * returns the Port ID for the specified port, or 0xFFFF (-1) if the port is invalid/null
 * @param port the port handle
 * @return the port ID
 */
static inline uint16_t portId(port_handle_t port) {
    return (port) ? BASE_PORT(port)->pnum : 0xFFFF;
}

/**
 * returns the Port Type for the specified port, or 0xFFFF (-1) if the port is invalid/null.
 * The port type is a combination of ID + flags which defines the capability of the port.
 * Bits 0-3 determine the hardware implementation of the port (UART, SPI, etc).
 * Bits 4-7 are flags indicating the functional type of the port.
 * Bits 8-16 are reserved for future use
 * @param port the port handle
 * @return the port type
 */
static inline uint16_t portType(port_handle_t port) {
    return (port) ? BASE_PORT(port)->ptype : 0xFFFF;
}

/**
 * Recalculates the port's internal checksum - this is a validation technique use to ensure
 * the pointer and settings of the port are valid. This should be called anytime the port
 * id, ptype, or pflags is updated.
 * WARNING: This call makes no assumptions about the validity/state of the port - if the
 * port is referencing an invalid port or bad address, this will calculate a checksum (and
 * attempt to assign it) and subsequently indicate that the invalid port is now valid.
 * @param port the port to recalculate the checksum for
 */
static inline void portRecalcChksum(port_handle_t port) {
    BASE_PORT(port)->chksum = ~((BASE_PORT(port)->pnum << 16) | BASE_PORT(port)->ptype);
}

/**
 * Calculates the current checksum of the port, and validates that it matches the internal
 * chksum value. This is the primary means for determine if a port handle is valid.
 * @param port the port and test
 * @return non-zero (true) if the port_handle maintains integrity, otherwise zero (false)
 */
static inline int portCheckIntegrity(port_handle_t port) {
    return (port && (BASE_PORT(port)->chksum == ~(uint32_t)((BASE_PORT(port)->pnum << 16) | BASE_PORT(port)->ptype)));
}

#define NOT_GNSS_PORT(port) ((portType(port) & PORT_TYPE__GNSS) == 0)

/**
 * @brief sets the specified bit-flags on the port, without modifying previously set flags
 * @param port the port to modify
 * @param f a bit mask of flags to set
 */
static inline void portFlagsSet(port_handle_t port, uint16_t f) {
    if (!port) return;
    BASE_PORT(port)->pflags |= f;
    portRecalcChksum(port);
}

/**
 * @brief clears the specified bit-flags on the port, without modifying previously set flags
 * @param port the port to modify
 * @param f the bit mask of flags to clear
 */
static inline void portFlagsClear(port_handle_t port, uint16_t f) {
    if (!port) return;
    BASE_PORT(port)->pflags &= ~f;
    portRecalcChksum(port);
}

/**
 * @brief checks if the specified bit-flags on the port are set
 * @param port the port to check
 * @param f the flags to check
 * @returns turns if all bits specified by f are also set on the port, otherwise false
 */
static inline int portFlagsIsSet(port_handle_t port, uint16_t f) {
    return port && ((BASE_PORT(port)->pflags & f) == f);
}

/**
 * Verifies the state of the port.
 * @param port the port handle
 * @return true (non-zero) if the port's passes an integrity check and has the PORT_FLAG__VALID set, otherwise false (zero)
 */
static inline uint8_t portIsValid(port_handle_t port) {
    if (!port) return 0;
    int integrity = portCheckIntegrity(port);
    int validFlag = portFlagsIsSet(port, PORT_FLAG__VALID);
    return (integrity && validFlag);
}

/**
 * Invalidates the port checksum, indicating that this port is no longer valid.
 * NOTE: When marking a port as invalid, you are indicating that it is no longer suitable for use and
 * its state can not be trusted. You should release the port handle and reallocate it before attempting
 * to use this port again.
 * @param port the port handle
 */
static inline void portInvalidate(port_handle_t port) {
    if (portIsValid(port)) { BASE_PORT(port)->chksum = UINT32_MAX; portFlagsClear(port, PORT_FLAG__VALID); }
}

/**
 * Determines viability of the specified port. Does not connect or otherwise interface with the port
 * directly, but generally indicates whether the underlying OS devices and resources exist in order to
 * successfully open and operate on the port. If the port is successfully validated, its cksum field updated
 * to reflect a valid indication of the port, until cleared by portInvalidate().
 * @param port the port to validate
 * @return 1 if the port is determined to be viable, 0 if the port is invalid, or <0 if an error occurred
 */
static inline int portValidate(port_handle_t port) {
    if (!portCheckIntegrity(port)) return 0;
    if (BASE_PORT(port)->portValidate && BASE_PORT(port)->portValidate(port))
        portFlagsSet(port, PORT_FLAG__VALID);
    else
        portFlagsClear(port, PORT_FLAG__VALID);

    return portFlagsIsSet(port, PORT_FLAG__VALID);
}

/**
 * returns true if the port's ptype's has the PORT_FLAG__OPENED bit set
 * @param port the port handle
 * @return the port type
 */
static inline uint8_t portIsOpened(port_handle_t port) {
    return (portIsValid(port) && portFlagsIsSet(port, PORT_FLAG__OPENED));
}

/**
 * returns the port flags for the specified port.
 * Note that any flags which does not have the lsb0 bit set, indicates the port is invalid.
 * @param port the port handle
 * @return the port flags
 */
static inline uint16_t portFlags(port_handle_t port) {
    return (port) ? BASE_PORT(port)->pflags : 0;
}

/**
 * returns the most recent operational error number (typically errno) for this port, or 0 if successful.
 * @param port the port handle
 * @return a PORT_ERROR__* number, or PORT_ERROR__NONE (0) if no error
 */
static inline uint16_t portError(port_handle_t port) {
    return (port) ? BASE_PORT(port)->perror : 0;
}

/**
 * Returns the name, if any, associated with this port
 * @param port
 * @return a pointer to a string representing the name of the port, or a nullptr if not supported
 */
static inline const char *portName(port_handle_t port) {
    // we intentionally do NOT check if port is valid here... This maybe a problem, but it causes even more if we do
    return (port && BASE_PORT(port)->portName) ? BASE_PORT(port)->portName(port) : "";
}

/**
 * Returns the associated port_stats_t pointer, if any that is attached to this port
 * @param port the port to query
 * @return a port_stats_t*
 */
static inline port_stats_t* portStats(port_handle_t port) { return portIsValid(port) ? BASE_PORT(port)->stats : (port_stats_t *)0; }

/**
 * Resets the associated port stats, if enabled for this port.
 * @param port the port to reset
 * @return a PORT_ERROR__* number, or PORT_ERROR__NONE (0) if no error
 */
static inline uint16_t portStatsReset(port_handle_t port) {
    if (!portIsValid(port)) return (uint16_t)PORT_ERROR__INVALID;
    if (!BASE_PORT(port)->stats) return (uint16_t)PORT_ERROR__NOT_SUPPORTED;

    memset(BASE_PORT(port)->stats, 0, sizeof(port_stats_t)) ;
    return PORT_ERROR__NONE;
}

/**
 * Opens or establishes a connection to port. This function may not be supported on all port implementations.
 * @param port the port to open
 * @return a PORT_ERROR__* number, or PORT_ERROR__NONE (0) if no error
 */
static inline int portOpen(port_handle_t port) {
    if (!portIsValid(port)) return PORT_ERROR__INVALID;
    return (BASE_PORT(port)->portOpen) ? BASE_PORT(port)->portOpen(port) : PORT_ERROR__NOT_SUPPORTED;
}

/**
 * Attempts to open the specified port, until a timeout occurs.
 * @param port the port to open
 * @param timeoutMs the maximum time to wait for the port to open/connect
 * @param retryDelayMs the number of milliseconds to wait between failed open attempts
 * @return PORT_ERROR__NONE if successful, else of PORT_ERROR__* indicating the reason for failure.
 */
int portOpenRetry(port_handle_t port, unsigned int timeoutMs, unsigned int retryDelayMs);

/**
 * Closes or disconnects a connection to port. This function may not be supported on all port implementations.
 * @param port the port to close
 * @return a PORT_ERROR__* number, or PORT_ERROR__NONE (0) if no error
 */
static inline int portClose(port_handle_t port) {
    if (!portIsValid(port)) return PORT_ERROR__INVALID;
    return (BASE_PORT(port)->portClose) ? BASE_PORT(port)->portClose(port) : PORT_ERROR__NOT_SUPPORTED;
}

/**
 * returns the number of free bytes in the underlying TX buffer, for new data to be queued for
 * sending. Attempting to send more than portFree() bytes will result in a TX_OVERFLOW being raised
 * on the port. It is the callers responsibility to detect this, and retain unsent data until all
 * data can be transmitted, or to discard the excess information.
 * @param port the port to query
 * @return the number of bytes which be be safely written to the port without data drop, or a PORT_ERROR__* number (<0)
 */
static inline int portFree(port_handle_t port) {
    if (!portIsValid(port)) return PORT_ERROR__INVALID;
    return (BASE_PORT(port)->portFree) ? BASE_PORT(port)->portFree(port) : PORT_ERROR__NOT_SUPPORTED;
}

/**
 * returns the number of bytes available to be read from the underlying RX buffer, if any.
 * @param port the port to query
 * @return the number of bytes which are available to be read from the port, or a PORT_ERROR__* number (<0)
 */
static inline int portAvailable(port_handle_t port) {
    if (!portIsValid(port)) return PORT_ERROR__INVALID;
    return (BASE_PORT(port)->portAvailable) ? BASE_PORT(port)->portAvailable(port) : PORT_ERROR__NOT_SUPPORTED;
}

/**
 * Blocks upto timeoutMs for all queued TX data to be sent to the physical device. No guarantee is made
 * about the delivery state of that data. This effectively flushes the TX buffer. If timeoutMs is exceeded,
 * all remaining data in the buffer will be dropped.
 * @param port the port to query
 * @param timeoutMs the maximum number of milliseconds to allow data to be sent before dropping all remaining data.
 * @return if >= 0, the number of bytes that were dropped if any, otherwise a PORT_ERROR__ indicating the error
 */
static inline int portDrain(port_handle_t port, uint32_t timeoutMs) {
    if (!portIsValid(port)) return PORT_ERROR__INVALID;
    return (BASE_PORT(port)->portDrain) ? BASE_PORT(port)->portDrain(port, timeoutMs) : PORT_ERROR__NOT_SUPPORTED;
}

/**
 * flushes/removes all bytes currently waiting to be read from the port
 * @param port the port to query
 * @return if >= 0, the number of bytes that were flushed, otherwise a PORT_ERROR__ indicating the error
 */
static inline int portFlush(port_handle_t port) {
    if (!portIsValid(port)) return PORT_ERROR__INVALID;
    return (BASE_PORT(port)->portFlush) ? BASE_PORT(port)->portFlush(port) : PORT_ERROR__NOT_SUPPORTED;
}

/**
 * Returns the portLogger function associated with this port, if any
 * @param port the port to query
 * @return the pointer to the pfnPortLogger function, or NULL if none.
 */
static inline pfnPortLogger portLogger(port_handle_t port) {
    return (BASE_PORT(port)->portLogger) ? BASE_PORT(port)->portLogger : 0;
}

/**
 * Sets the portLogger function to be associated with this port, or to clear it if NULL.
 * @param port the port to associate the logger with
 * @param portLogger a pointer to the pfnRportLogger function
 * @param loggerData an opaque "user data" pointer that will be passed to future calls to the portLogger function
 */
static inline void portSetLogger(port_handle_t port, pfnPortLogger portLogger, void* loggerData) {
    if (portIsValid(port)) {
        BASE_PORT(port)->portLogger = portLogger;
        BASE_PORT(port)->portLoggerData = loggerData;
    }
}

/**
 * A callback function called internally when a particular action (usually read or write) are
 * performed on the port, which is useful in logging or monitoring the data which goes through
 * the port
 * @param port the port which the operation was performed on
 * @param op an identifier of the operation that was performed
 * @param buf a pointer to the underlying data buffer used in the operation (you should avoid modifying this data unless you know what you are doing)
 * @param len the number of bytes of data associated with the operation
 * @param userData an opaque "user data" pointer which is provided by the port implementation
 * @return an implementation specific number
 */
static inline int portLog(port_handle_t port, uint8_t op, const uint8_t* buf, unsigned int len, void *userData) {
    if (!portIsValid(port)) return PORT_ERROR__INVALID;
    return (BASE_PORT(port)->portLogger) ? BASE_PORT(port)->portLogger(port, op, buf, len, userData) : PORT_ERROR__NOT_SUPPORTED;
}


/**
 * Reads upto/at most 'len' number of bytes, copying those bytes into the buffer pointed to by 'buf'. Data copied into 'buf' is
 * removed from the internal RX buffer of the port, releasing 'len' bytes from the underlying buffer. It cannot be read again.
 * @param port the port from which to read data
 * @param buf the buffer to place a copy of the data into
 * @param len the maximum number of bytes to read; if fewer than 'len' bytes are available, only those bytes available will be returned.
 * @return the number of actual bytes read from the internal RX buffer.
 */
static inline int portRead(port_handle_t port, uint8_t* buf, unsigned int len) 
{
    int bytesRead = 0;

    // If the port is not valid, return an error
    if (!portIsValid(port)) return PORT_ERROR__INVALID;

    // If the port does not support reading, return an error
    if (!BASE_PORT(port)->portRead) return PORT_ERROR__NOT_SUPPORTED;

    // Read the port, copying the data into the buffer
    bytesRead = BASE_PORT(port)->portRead(port, buf, len);

    // If stats are enabled, update the stats
    if (BASE_PORT(port)->stats) {
        if (bytesRead >= 0) BASE_PORT(port)->stats->rxBytes += bytesRead;
        else                BASE_PORT(port)->stats->rxOverflows++;  // Note the error  FIXME: I'm not sure this will actually work - since we don't actually know they type of error
    }

    // If the portLogger is set, log the read operation
    if ((bytesRead > 0) && (BASE_PORT(port)->portLogger)) portLog(port, PORT_OP__READ, buf, bytesRead, BASE_PORT(port)->portLoggerData);

    return bytesRead;
}

/**
 * Reads upto/at most 'len' number of bytes, copying those bytes into the buffer pointed to by 'buf'. Data copied into 'buf' is
 * removed from the internal RX buffer of the port, releasing 'len' bytes from the underlying buffer. It cannot be read again.
 * @param port the port from which to read data
 * @param buf the buffer to place a copy of the data into
 * @param len the maximum number of bytes to read; if fewer than 'len' bytes are available, only those bytes available will be returned.
 * @param timeout the maximum time, in milliseconds, to wait for 'len' bytes to be received before returning.
 * @return the number of actual bytes read from the internal RX buffer.
 */
static inline int portReadTimeout(port_handle_t port, uint8_t* buf, unsigned int len, unsigned int timeout) 
{
    int bytesRead = 0;

    // If the port is not valid, return an error
    if (!portIsValid(port)) return PORT_ERROR__INVALID;

    // If the port does not support reading, return an error
    if (!BASE_PORT(port)->portReadTimeout) return portReadTimeout_internal(port, buf, len, timeout); // PORT_ERROR__NOT_SUPPORTED;

    // Read the port, copying the data into the buffer
    bytesRead = BASE_PORT(port)->portReadTimeout(port, buf, len, timeout);

    // If stats are enabled, update the stats
    if (BASE_PORT(port)->stats) {
        if (bytesRead >= 0)  BASE_PORT(port)->stats->rxBytes += bytesRead;
        else                BASE_PORT(port)->stats->rxOverflows++;  // note the error  FIXME: I'm not sure this will actually work - since we don't actually know they type of error
    }

    // If the portLogger is set, log the read operation
    if ((bytesRead > 0) && (BASE_PORT(port)->portLogger)) portLog(port, PORT_OP__READ, buf, bytesRead, BASE_PORT(port)->portLoggerData);

    return bytesRead;
}

/**
 * Write the specified len bytes pointed to by buf to the provided port
 * @param port the port to send the data to
 * @param buf the buffer to send data from
 * @param len the number of bytes to send
 * @return the number of bytes sent (0 is valid), or <0 in the event of an error
 */
static inline int portWrite(port_handle_t port, const uint8_t* buf, unsigned int len) {
    if (!portIsValid(port)) return PORT_ERROR__INVALID;
    if (BASE_PORT(port)->portLogger) portLog(port, PORT_OP__WRITE, buf, len, BASE_PORT(port)->portLoggerData);
    int bytesWritten = (BASE_PORT(port)->portWrite) ? BASE_PORT(port)->portWrite(port, buf, len) : PORT_ERROR__NOT_SUPPORTED;
    if (BASE_PORT(port)->stats) 
    {
        if (bytesWritten >= 0) 
        {
            BASE_PORT(port)->stats->txBytes += bytesWritten;
            if (bytesWritten < (int)len)
            {
                BASE_PORT(port)->stats->txDataDrops++;
                BASE_PORT(port)->stats->txBytesDropped += (len - bytesWritten);
            }
        }
        else BASE_PORT(port)->stats->txDataDrops++, BASE_PORT(port)->stats->txBytesDropped += len;  // note the error, and the bytes dropped  FIXME: I'm not sure this will actually work - since we don't actually know they type of error
    }
    return bytesWritten;
}


#ifdef __cplusplus
}
#endif

#endif //IS_CORE__BASE_PORT_H
