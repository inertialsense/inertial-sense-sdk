/**
 * @file ISLogFile.h
 * @brief `cISLogFileBase` implementation backed by standard C stdio (`FILE*`).
 *
 * The default `cISLogFileBase` implementation on every host platform (see ISLogFileFactory.h,
 * which selects this over the FatFs-backed implementation on EVB-2). Every method is a thin,
 * null-checked pass-through to the corresponding stdio function (`fopen`, `fread`, `fseek`, etc.).
 *
 * @author robb on 11/5/18.
 * @copyright Copyright (c) 2026 Inertial Sense, Inc. All rights reserved.
 */

#ifndef _IS_SDK_IS_LOG_FILE_H_
#define _IS_SDK_IS_LOG_FILE_H_

#include "ISLogFileBase.h"



/** @brief `cISLogFileBase` implementation over a standard C stdio `FILE*`. */
class cISLogFile : public cISLogFileBase
{
public:
    /** Construct without opening a file; call open() before using any I/O method. */
    cISLogFile();

    /** Construct and immediately open() @p filePath in the given @p mode. Check isOpened() to confirm success. */
    cISLogFile(const std::string& filePath, const char* mode);

    /** Construct and immediately open() @p filePath in the given @p mode. Check isOpened() to confirm success. */
    cISLogFile(const char* filePath, const char* mode);

    /** Closes the underlying file if still open. */
    ~cISLogFile();

    bool open(const char* filePath, const char* mode) OVERRIDE;
    bool close() OVERRIDE;
    bool flush() OVERRIDE;
    bool good() OVERRIDE;
    bool isOpened() OVERRIDE;

    int putch(char ch) OVERRIDE;
    int puts(const char* str) OVERRIDE;
    std::size_t write(const void* bytes, std::size_t len) OVERRIDE;
    int lprintf(const char* format, ...) OVERRIDE;
    int vprintf(const char* format, va_list args) OVERRIDE;

    /**
     * @brief Formatted print directly to `stdout`, bypassing any `cISLogFile` instance.
     * @param format printf-style format string, followed by its matching arguments.
     * @return the number of characters written, as returned by the underlying `vprintf()`.
     */
    static int lprintfStdout(const char* format, ...);

    /**
     * @brief Format a `printf()`-style string into a `std::string`.
     * @param format printf-style format string, followed by its matching arguments.
     * @return the formatted string, truncated to 255 characters if the formatted output would exceed that length.
     */
    static std::string formatString(const char* format, ...);

    int getch() OVERRIDE;
    std::size_t read(void* bytes, std::size_t len) OVERRIDE;

    /**
     * @note Defaults to `SEEK_CUR` here, unlike `cISLogFileBase::seek()`'s `SEEK_SET` default —
     *       see the note on the base declaration. Pass @p origin explicitly when it matters.
     */
    int seek(long int offset, int origin = SEEK_CUR) OVERRIDE;
    long int tell() OVERRIDE;
    int eof() OVERRIDE;

private:
    FILE *m_file;   //!< underlying stdio file handle; null when no file is open
};


#endif //_IS_SDK_IS_LOG_FILE_H_
