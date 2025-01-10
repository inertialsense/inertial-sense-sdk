/*
MIT LICENSE

Copyright (c) 2014-2025 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#ifndef IS_SDK_IS_FILE_MANAGER_H_
#define IS_SDK_IS_FILE_MANAGER_H_

#include "ISConstants.h"
#include <string>
#include <vector>
#include <cstdint>

namespace ISFileManager
{
    typedef struct
    {
        std::string name;
        uint64_t size;
        time_t lastModificationDate;
    } file_info_t;

    /**
     * Is this path directory?
     * @param path the path to check
     * @return true if the path is a directory, false otherwise
     */
    bool PathIsDir(const std::string& path);

    // get all files in a folder - files parameter is not cleared in this function
    // files contains the full path to the file
    // return false if no files found, true otherwise
    bool GetAllFilesInDirectory(const std::string& directory, bool recursive, std::vector<std::string>& files);
    bool GetAllFilesInDirectory(const std::string& directory, bool recursive, const std::string& regexPattern, std::vector<std::string>& files);

    bool DeleteFile(const std::string& fullFilePath);
    void DeleteDirectory(const std::string& directory, bool recursive = true);

    // get space used in a directory recursively - files is filled with all files in the directory, sorted by modification date, files is NOT cleared beforehand. sortByDate of false sorts by file name
    uint64_t GetDirectorySpaceUsed(const std::string& directory, bool recursive = true);
    uint64_t GetDirectorySpaceUsed(const std::string& directory, std::vector<file_info_t>& files, bool sortByDate = true, bool recursive = true);
    uint64_t GetDirectorySpaceUsed(const std::string& directory, std::string regexPattern, std::vector<file_info_t>& files, bool sortByDate = true, bool recursive = true);

    // get free space for the disk that the specified directory exists on
    uint64_t GetDirectorySpaceAvailable(const std::string& directory);

    // get drive total size that the specified directory exists on
    uint64_t GetDirectoryDriveTotalSize(const std::string& directory);

    // get just the file name from a path
    std::string GetFileName(const std::string& path);

    // get just the parent/base path from a path
    std::string GetParentDirectory(const std::string& path);

    // get the current working directory
    std::string CurrentWorkingDirectory();

    /**
     * Returns true if the specified path is consider an absolute path
     * @param path
     * @return true if absolute, false if relative
     */
    bool isPathAbsolute(const std::string& path);

    /**
     * returns an absolute path to the parent directory of the specified path
     * If 'path' is a relative path, it is applied to the current working directory
     * @param path a complete path
     * @param parent a string representing the path to the parent directory
     * @return true on success, otherwise false
     */
    bool getParentDirectory(const std::string& path, std::string& parent);

    /**
     * Extracts and stores the base, filename, and ext(ension) components from the specified path
     * @param path the path to parse
     * @param parent the base or parent directory which contains the file. This is always an absolute path.
     * @param file the filename which the path references. This is just the filename with the extension, and does not include any directory/path information
     * @param ext the filename extension of the filename. This is the same as the last matching characters of the filename, and is included for convenience.
     * @return true if there was sufficient path information to extract the parent of the path, otherwise false.  This is NOT an error condition,
     * but instead indicates (if true) that parent contains meaningful data.
     */
    bool getPathComponents(const std::string& path, std::string& parent, std::string& file, std::string& ext);

    bool TouchFile(const std::string& path);

    /**
     * @brief Create directory specified if it does not exist.
     * 
     * @param path relative or absolute path for directory to be created.
     * @return true on success 
     */
    bool CreateDirectory(const std::string& path);

    /**
     * @brief Remove oldest files in the specified directory including all subdirectories until the target size in bytes is met.
     * 
     * @param directory path to search to file oldest files to be removed.
     * @param target_size in bytes that the specified directory must achieve by removing oldest files.
     */
    void RemoveOldestFiles(const std::string& directory, std::uintmax_t target_size);

    /**
     * @brief Function to recursively remove empty directories.
     * 
     * @param directory path to search where files should be removed.
     */
    bool RemoveEmptyDirectories(const std::string& directory);

}

#endif //IS_SDK_IS_FILE_MANAGER_H_
