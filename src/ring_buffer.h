/**
 * @file ring_buffer.h
 * @brief Lock-free ring buffer (circular buffer) for embedded and POSIX targets.
 *
 * Provides a byte-oriented ring buffer with separate read, write, and search
 * (peek/find) operations.  All operations are pointer-arithmetic based and do
 * not allocate memory; the caller supplies the backing buffer via ringBufInit().
 *
 * @author Walt Johnson
 * @copyright Copyright (c) 2014-2025 Inertial Sense, Inc. - http://inertialsense.com
 */

#ifndef _RING_BUFFER_H_
#define _RING_BUFFER_H_

#ifdef __cplusplus
extern "C" {
#endif

/** Ring buffer state structure. Callers must not modify fields directly. */
typedef struct
{
    unsigned char *startPtr;    //!< Pointer to buffer start (lowest address)
    unsigned char *endPtr;      //!< Pointer to one past buffer end (startPtr + bufSize)
    unsigned char *rdPtr;       //!< Current read pointer
    unsigned char *wrPtr;       //!< Current write pointer
    int bufSize;                //!< Total byte capacity of the backing buffer
    int wordByteSize;           //!< Byte size of a single logical element (1 for raw byte buffers)
} ring_buf_t;

/**
 * @brief Initialize a ring buffer.
 * @param rbuf         Ring buffer instance to initialize.
 * @param buf          Caller-provided backing byte array.
 * @param bufSize      Size in bytes of @p buf.
 * @param wordByteSize Size in bytes of one logical element (use 1 for byte-oriented buffers).
 */
void ringBufInit(ring_buf_t *rbuf, unsigned char* buf, int bufSize, int wordByteSize);

/**
 * @brief Return the number of bytes currently stored in the buffer.
 * @param rbuf Ring buffer instance.
 * @return Number of bytes available to read.
 */
int ringBufUsed(const ring_buf_t *rbuf);

/**
 * @brief Return the number of bytes free in the buffer.
 * @param rbuf Ring buffer instance.
 * @return Number of bytes that can be written before the buffer is full.
 */
int ringBufFree(const ring_buf_t *rbuf);

/**
 * @brief Write bytes into the ring buffer.
 * @param rbuf     Ring buffer instance.
 * @param buf      Source data to write.
 * @param numBytes Number of bytes to write from @p buf.
 * @return Number of bytes actually written (may be less than @p numBytes if the buffer is full).
 */
int ringBufWrite(ring_buf_t *rbuf, unsigned char *buf, int numBytes);

/**
 * @brief Read and consume bytes from the ring buffer.
 * @param rbuf Ring buffer instance.
 * @param buf  Destination buffer to copy data into.
 * @param len  Maximum number of bytes to read.
 * @return Number of bytes read.
 */
int ringBufRead(ring_buf_t *rbuf, unsigned char *buf, int len);

/**
 * @brief Peek at bytes in the ring buffer without consuming them.
 * @param rbuf   Ring buffer instance.
 * @param buf    Destination buffer for the peeked bytes.
 * @param len    Maximum number of bytes to copy.
 * @param offset Byte offset from the current read pointer at which to begin peeking.
 * @return Number of bytes copied into @p buf.
 */
int ringBufPeek(const ring_buf_t *rbuf, unsigned char *buf, int len, int offset);

/**
 * @brief Read bytes up to and including a delimiter byte.
 * @param rbuf      Ring buffer instance.
 * @param buf       Destination buffer.
 * @param len       Maximum bytes to copy.
 * @param character Delimiter byte; reading stops after this byte is found.
 * @return Number of bytes read.
 */
int ringBufReadToChar(ring_buf_t *rbuf, unsigned char *buf, int len, unsigned char character);

/**
 * @brief Read bytes up to and including either of two delimiter bytes.
 * @param rbuf       Ring buffer instance.
 * @param buf        Destination buffer.
 * @param len        Maximum bytes to copy.
 * @param character1 First delimiter byte.
 * @param character2 Second delimiter byte.
 * @return Number of bytes read.
 */
int ringBufReadToChar2(ring_buf_t *rbuf, unsigned char *buf, int len, unsigned char character1, unsigned char character2);

/**
 * @brief Search for a byte sequence in the ring buffer without consuming data.
 * @param rbuf Ring buffer instance.
 * @param str  Byte sequence to search for.
 * @param len  Length of @p str in bytes.
 * @return Byte offset from the read pointer where the sequence begins, or -1 if not found.
 */
int ringBufFind(const ring_buf_t *rbuf, const unsigned char *str, int len);

/**
 * @brief Discard bytes from the front of the ring buffer.
 * @param rbuf Ring buffer instance.
 * @param len  Number of bytes to discard.
 * @return Number of bytes actually removed.
 */
int ringBufRemove(ring_buf_t *rbuf, int len);

/**
 * @brief Reset the ring buffer to empty without zeroing the backing storage.
 * @param rbuf Ring buffer instance.
 * @return 0 on success.
 */
int ringBufClear(ring_buf_t *rbuf);

/**
 * @brief Check whether the ring buffer is empty.
 * @param rbuf Ring buffer instance.
 * @return Non-zero (true) if empty, 0 if data is available.
 */
int ringBufEmpty(const ring_buf_t *rbuf);



#ifdef __cplusplus
}
#endif

#endif  // _RING_BUFFER_H_
