/**
 * @file ISFileManager.h
 * @brief Cross-platform (Windows/Linux/macOS/EVB-2 FatFs) directory and file utilities used by
 *        the logger for drive-usage accounting and file culling.
 *
 * Every function here has a platform-specific implementation behind `#if PLATFORM_IS_*`
 * branches in ISFileManager.cpp — this header documents the common contract, not any one
 * platform's implementation detail, unless a behavior genuinely differs by platform.
 *
 * @author Inertial Sense, Inc.
 * @copyright Copyright (c) 2026 Inertial Sense, Inc. All rights reserved.
 */

#ifndef IS_SDK_IS_FILE_MANAGER_H_
#define IS_SDK_IS_FILE_MANAGER_H_

#include "ISConstants.h"
#include <string>
#include <vector>
#include <cstdint>

namespace ISFileManager
{
    /** @brief Name, size, and last-modification time of one file, as returned by GetAllFilesInDirectory()/GetDirectorySpaceUsed(). */
    typedef struct
    {
        std::string name;                  //!< full path to the file
        uint64_t size;                     //!< file size in bytes
        time_t lastModificationDate;        //!< last-modification time
    } file_info_t;

    /**
     * @brief Check whether a path is a directory.
     * @param path the path to check.
     * @return true if the path is a directory, false otherwise (including if it doesn't exist).
     */
    bool PathIsDir(const std::string& path);

    /**
     * @brief Get every file in a directory, as full paths.
     * @param directory directory to search.
     * @param recursive if true, also search subdirectories.
     * @param[out] files matching file paths are appended; NOT cleared before appending.
     * @return true if any file was appended, false if none were found (or the directory couldn't be opened).
     */
    bool GetAllFilesInDirectory(const std::string& directory, bool recursive, std::vector<std::string>& files);

    /**
     * @brief Get every file in a directory, with size/modification-time info.
     * @param directory directory to search.
     * @param recursive if true, also search subdirectories.
     * @param[out] files matching files are appended; NOT cleared before appending.
     * @return true if any file was appended, false if none were found (or the directory couldn't be opened).
     */
    bool GetAllFilesInDirectory(const std::string& directory, bool recursive, std::vector<file_info_t>& files);

    /**
     * @brief Get every file in a directory whose full path matches a regular expression, as full paths.
     *
     * @note On Linux, when @p regexPattern is empty, this overload matches nothing (returns no
     *       files) rather than matching everything — unlike the `file_info_t` overload below,
     *       which treats an empty pattern as "match all" on every platform. Pass a `file_info_t`
     *       vector instead (and extract names from it) if you need an unfiltered listing on Linux.
     *
     * @param directory directory to search.
     * @param recursive if true, also search subdirectories.
     * @param regexPattern case-insensitive regular expression matched against each file's full path.
     * @param[out] files matching file paths are appended; NOT cleared before appending.
     * @return true if any file was appended, false if none matched (or the directory couldn't be opened).
     */
    bool GetAllFilesInDirectory(const std::string& directory, bool recursive, const std::string& regexPattern, std::vector<std::string>& files);

    /**
     * @brief Get every file in a directory whose full path matches a regular expression, with size/modification-time info.
     * @param directory directory to search.
     * @param recursive if true, also search subdirectories.
     * @param regexPattern case-insensitive regular expression matched against each file's full path; an empty pattern matches every file.
     * @param[out] files matching files are appended; NOT cleared before appending.
     * @return true if any file was appended, false if none matched (or the directory couldn't be opened).
     */
    bool GetAllFilesInDirectory(const std::string& directory, bool recursive, const std::string& regexPattern, std::vector<file_info_t>& files);

    /** @brief Delete a single file. @param fullFilePath path to the file to delete. @return true on success, false otherwise. */
    bool DeleteFile(const std::string& fullFilePath);

    /**
     * @brief Delete a directory and its contents.
     * @param directory directory to delete.
     * @param recursive if true, delete subdirectories and their contents too. @note ignored on EVB-2/FatFs, which always deletes recursively.
     */
    void DeleteDirectory(const std::string& directory, bool recursive = true);

    /**
     * @brief Get the total size of every file in a directory.
     * @param directory directory to scan.
     * @param recursive if true, also scan subdirectories.
     * @return total size in bytes of every file found.
     */
    uint64_t GetDirectorySpaceUsed(const std::string& directory, bool recursive = true);

    /**
     * @brief Get the total size of every file in a directory, and the files themselves, sorted.
     * @param directory directory to scan.
     * @param[out] files every file found, sorted by @p sortByDate (ascending modification time) or by name; NOT cleared before appending.
     * @param sortByDate if true, sort @p files by modification date (oldest first); if false, sort by name.
     * @param recursive if true, also scan subdirectories.
     * @return total size in bytes of every file found.
     */
    uint64_t GetDirectorySpaceUsed(const std::string& directory, std::vector<file_info_t>& files, bool sortByDate = true, bool recursive = true);

    /**
     * @brief Get the total size of every file in a directory matching a pattern, and the files themselves, sorted.
     * @param directory directory to scan.
     * @param regexPattern case-insensitive regular expression matched against each file's full path; an empty pattern matches every file.
     * @param[out] files every matching file, sorted by @p sortByDate (ascending modification time) or by name; NOT cleared before appending.
     * @param sortByDate if true, sort @p files by modification date (oldest first); if false, sort by name.
     * @param recursive if true, also scan subdirectories.
     * @return total size in bytes of every matching file found.
     */
    uint64_t GetDirectorySpaceUsed(const std::string& directory, std::string regexPattern, std::vector<file_info_t>& files, bool sortByDate = true, bool recursive = true);

    /**
     * @brief Get the free space available on the disk/volume containing a directory.
     * @param directory directory identifying the disk/volume to query; created temporarily (and removed afterward) if it doesn't already exist.
     * @return free space in bytes, or 0 on error.
     */
    uint64_t GetDirectorySpaceAvailable(const std::string& directory);

    /**
     * @brief Get the total size of the disk/volume containing a directory.
     * @param directory directory identifying the disk/volume to query; created temporarily (and removed afterward) if it doesn't already exist.
     * @return total size in bytes, or 0 on error.
     */
    uint64_t GetDirectoryDriveTotalSize(const std::string& directory);

    /** @brief Get just the file name (with extension) from a path, stripping any directory components. @return the file name, or the whole input if it contains no path separator. */
    std::string GetFileName(const std::string& path);

    /**
     * @brief Get the parent directory from a path.
     * @note Despite the name, this returns the same thing as GetFileName() — the component
     *       *after* the last path separator, not before it. It does NOT return the parent
     *       directory. Use the lowercase-`g` getParentDirectory() below for the actual parent path.
     * @return the file name (with extension), or the whole input if it contains no path separator.
     */
    std::string GetParentDirectory(const std::string& path);

    /** @brief Get the current working directory. @return the current working directory, or an empty string on error. */
    std::string CurrentWorkingDirectory();

    /**
     * @brief Check whether a path is absolute.
     * @note Relies on `path_seperator` (`/` on Linux, `\` on Windows), which is null on other
     *       platforms (e.g. EVB-2) — calling this there is undefined behavior.
     * @param path the path to check.
     * @return true if absolute, false if relative.
     */
    bool isPathAbsolute(const std::string& path);

    /**
     * @brief Get an absolute path to the parent directory of a path.
     *
     * If @p path is relative, it is first resolved against the current working directory.
     * @note Relies on `path_seperator`, which is null on platforms other than Linux/Windows
     *       (e.g. EVB-2) — calling this there is undefined behavior.
     *
     * @param path a complete path.
     * @param[out] parent the path to the parent directory.
     * @return true on success, false if no path separator could be found (even after resolving against the current working directory).
     */
    bool getParentDirectory(const std::string& path, std::string& parent);

    /**
     * @brief Extract the parent directory, file name, and extension components from a path.
     *
     * @note Relies on getParentDirectory() and `path_seperator` internally — see its note on
     *       undefined behavior on platforms other than Linux/Windows.
     *
     * @param path the path to parse.
     * @param[out] parent the parent directory containing the file; always an absolute path (see getParentDirectory()).
     * @param[out] file the filename the path references, including its extension, with no directory/path information.
     * @param[out] ext the filename's extension, including the leading dot.
     * @return true if there was sufficient path information to extract the parent of the path, otherwise false. This is NOT an error condition — it indicates (if true) that @p parent contains meaningful data.
     */
    bool getPathComponents(const std::string& path, std::string& parent, std::string& file, std::string& ext);

    /** @brief Update an existing file's last-access/modification time to now; does not create the file if it doesn't already exist. @return true on success. */
    bool TouchFile(const std::string& path);

    /**
     * @brief Create a directory if it does not already exist, creating any missing parent directories along the way.
     *
     * @param path relative or absolute path for the directory to be created.
     * @return true on success, including if @p path already existed as a directory; false if
     *         @p path exists but isn't a directory, or the directory couldn't be created.
     */
    bool CreateDirectory(const std::string& path);

    /**
     * @brief Delete the oldest files (by modification time) in a directory and its subdirectories until the total size is at or under a target.
     *
     * @param directory directory to search, recursively, for files to remove.
     * @param target_size total size, in bytes, the directory should be reduced to (or below) by removing its oldest files. No-op if the directory is already at or under this size.
     */
    void RemoveOldestFiles(const std::string& directory, std::uintmax_t target_size);

    /**
     * @brief Recursively remove a directory and any of its subdirectories that contain no files (only recursively-empty subdirectories).
     * @param directory directory to check and, if empty, remove.
     * @return true if @p directory (after recursively removing empty subdirectories) itself ended up empty and was removed; false if it contains any file, or couldn't be opened.
     */
    bool RemoveEmptyDirectories(const std::string& directory);

}

#endif //IS_SDK_IS_FILE_MANAGER_H_
