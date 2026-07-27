/**
 * @file ISLogFileBase.h
 * @brief Abstract file I/O interface used by the logger and its file-backed helpers.
 *
 * Defines the contract a "log file" must satisfy regardless of the underlying storage: a
 * standard stdio-backed file (cISLogFile) on host platforms, or a FatFs-backed implementation
 * on embedded targets (cISLogFileFatFs, EVB-2). Callers program against this interface so the
 * concrete implementation is chosen once, at construction (see ISLogFileFactory.h).
 *
 * @author Inertial Sense, Inc.
 * @copyright Copyright (c) 2026 Inertial Sense, Inc. All rights reserved.
 */

#ifndef _IS_SDK_IS_LOG_FILE_BASE_H_
#define _IS_SDK_IS_LOG_FILE_BASE_H_

#include "ISConstants.h"
#include <cstddef>
#include <cstdarg>
#include <string>

#define LOG_DEBUG_FILE_WRITE    0   // Enable file debug printout
#define LOG_DEBUG_FILE_READ     0   //
#define LOG_DEBUG_CHUNK_WRITE   0   // Enable chunk debug printout
#define LOG_DEBUG_CHUNK_READ    0
#define LOG_CHUNK_STATS         0   // 0 = disabled, 1 = summary, 2 = detailed


/**
 * @brief Abstract interface for a single open file used by the logger and the `.idx`/chunk readers and writers.
 *
 * Implementors wrap one underlying file handle and expose byte-oriented, C-stdio-style
 * read/write/seek primitives. Every method here MUST fail soft when no file is open (or the
 * operation fails) rather than throw: writes/reads are no-ops that report zero bytes moved,
 * and character/formatted output returns `EOF`/a negative value, matching stdio conventions.
 */
class cISLogFileBase
{
public:
    virtual ~cISLogFileBase() {}

    /**
     * @brief Open the file at @p filePath.
     * @param filePath path to the file to open.
     * @param mode stdio-style `fopen()` mode string (e.g. "rb", "wb").
     * @return true if the file was opened successfully, false otherwise.
     */
    virtual bool open(const char* filePath, const char* mode)       = 0;

    /**
     * @brief Close the underlying file, if one is open.
     * @return true on success, false if no file was open or the close failed.
     */
    virtual bool close()                                            = 0;

    /**
     * @brief Flush any buffered writes to the underlying storage.
     * @return true on success, false if no file is open or the flush failed.
     */
    virtual bool flush()                                            = 0;

    /**
     * @brief Check whether the underlying file is free of a recorded error.
     * @return true if no error has been recorded on the file, false otherwise (including when no file is open).
     */
    virtual bool good()                                             = 0;

    /**
     * @brief Check whether a file is currently open on this instance.
     * @return true if a file handle is open, false otherwise.
     */
    virtual bool isOpened()                                         = 0;

    /**
     * @brief Write a single character to the file.
     * @param ch the character to write.
     * @return the character written, or EOF if no file is open or the write failed.
     */
    virtual int putch(char ch)                                      = 0;

    /**
     * @brief Write a null-terminated string to the file.
     * @param str the string to write.
     * @return a non-negative value on success, or EOF if no file is open or the write failed.
     */
    virtual int puts(const char* str)                               = 0;

    /**
     * @brief Write raw bytes to the file.
     * @param bytes pointer to the data to write.
     * @param len number of bytes to write.
     * @return the number of bytes actually written; 0 if no file is open.
     */
    virtual std::size_t write(const void* bytes, std::size_t len)   = 0;

    /**
     * @brief Write a `printf()`-style formatted string to the file.
     * @param format printf-style format string, followed by its matching arguments.
     * @return the number of characters written, or a negative value if no file is open or the write failed.
     */
    virtual int lprintf(const char* format, ...)                    = 0;

    /**
     * @brief Write a `printf()`-style formatted string to the file using an already-initialized argument list.
     * @param format printf-style format string.
     * @param args argument list initialized with `va_start()`, matching @p format.
     * @return the number of characters written, or a negative value if no file is open or the write failed.
     */
    virtual int vprintf(const char* format, va_list args)           = 0;

    /**
     * @brief Read a single character from the file.
     * @return the character read, or EOF if no file is open or the end of the file has been reached.
     */
    virtual int getch()                                             = 0;

    /**
     * @brief Read raw bytes from the file.
     * @param bytes destination buffer to fill.
     * @param len maximum number of bytes to read.
     * @return the number of bytes actually read; 0 if no file is open or the end of the file has been reached.
     */
    virtual std::size_t read(void* bytes, std::size_t len)          = 0;

    /**
     * @brief Reposition the file's read/write position indicator.
     *
     * @note This default argument is evaluated statically per call site, not virtually — code
     *       calling `seek(offset)` through a `cISLogFileBase*`/`&` gets *this* class's default
     *       (`SEEK_SET`), even if the concrete implementation declares a different default for
     *       direct calls on its own type. Pass @p origin explicitly when the call site's static
     *       type might not be `cISLogFileBase`.
     *
     * @param offset offset in bytes, interpreted relative to @p origin.
     * @param origin one of `SEEK_SET` (from the start of the file), `SEEK_CUR` (from the current
     *        position), or `SEEK_END` (from the end of the file).
     * @return 0 on success, non-zero on failure.
     */
    virtual int seek(long int offset, int origin = SEEK_SET)        = 0;

    /**
     * @brief Get the read/write position indicator's current byte offset from the start of the file.
     * @return the current offset in bytes.
     */
    virtual long int tell()                                         = 0;

    /**
     * @brief Check whether the end of the file has been reached.
     * @return non-zero if at the end of the file, 0 otherwise.
     */
    virtual int eof()                                               = 0;

};


#endif //_IS_SDK_IS_LOG_FILE_BASE_H_
