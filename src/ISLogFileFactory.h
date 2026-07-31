/**
 * @file ISLogFileFactory.h
 * @brief Platform-selecting factory for `cISLogFileBase` instances.
 *
 * Callers that need a log file should go through `CreateISLogFile()` / `CloseISLogFile()` rather
 * than constructing a concrete `cISLogFileBase` implementation directly, so the platform choice
 * (stdio-backed `cISLogFile` vs. FatFs-backed `cISLogFileFatFs` on EVB-2) stays centralized here.
 *
 * @author Inertial Sense, Inc.
 * @copyright Copyright (c) 2026 Inertial Sense, Inc. All rights reserved.
 */

#ifndef IS_SDK_CREATE_IS_LOG_FILE_H_
#define IS_SDK_CREATE_IS_LOG_FILE_H_

#include "ISConstants.h"

#if PLATFORM_IS_EVB_2
#include "ISLogFileFatFs.h"
#else
#include "ISLogFile.h"
#endif


/**
 * @brief Allocate a new, unopened log file for the current platform.
 * @return a heap-allocated `cISLogFileBase*` (concrete type depends on platform); never null.
 *         Caller owns the returned pointer and must release it via CloseISLogFile().
 */
inline cISLogFileBase* CreateISLogFile()
{
#if PLATFORM_IS_EVB_2
    return new cISLogFileFatFs;
#else
    return new cISLogFile;
#endif
}

/**
 * @brief Allocate and open a log file for the current platform.
 * @param filePath path to the file to open.
 * @param mode stdio-style `fopen()` mode string (e.g. "rb", "wb").
 * @return a heap-allocated `cISLogFileBase*` (concrete type depends on platform); never null.
 *         Caller owns the returned pointer and must release it via CloseISLogFile(). Check
 *         `isOpened()` on the result to confirm the open succeeded.
 */
inline cISLogFileBase* CreateISLogFile(const char* filePath, const char* mode)
{
#if PLATFORM_IS_EVB_2
    return new cISLogFileFatFs(filePath, mode);
#else
    return new cISLogFile(filePath, mode);
#endif
}

/**
 * @brief Allocate and open a log file for the current platform.
 * @param filePath path to the file to open.
 * @param mode stdio-style `fopen()` mode string (e.g. "rb", "wb").
 * @return a heap-allocated `cISLogFileBase*` (concrete type depends on platform); never null.
 *         Caller owns the returned pointer and must release it via CloseISLogFile(). Check
 *         `isOpened()` on the result to confirm the open succeeded.
 */
inline cISLogFileBase* CreateISLogFile(const std::string& filePath, const char* mode)
{
#if PLATFORM_IS_EVB_2
    return new cISLogFileFatFs(filePath, mode);
#else
    return new cISLogFile(filePath, mode);
#endif
}

/**
 * @brief Close and free a log file allocated by `CreateISLogFile()`.
 * @param logFile reference to the pointer to close and delete; set to null on return. No-op if already null.
 */
inline void CloseISLogFile(cISLogFileBase*& logFile)
{
    if (logFile != NULLPTR)
    {
        logFile->close();
        delete logFile;
        logFile = NULLPTR;
    }
}


#endif //IS_SDK_CREATE_IS_LOG_FILE_H_
