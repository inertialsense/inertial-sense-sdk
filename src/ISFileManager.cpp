/*
MIT LICENSE

Copyright (c) 2014-2025 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include "ISFileManager.h"
#include "ISUtilities.h"
#include <regex>


#if PLATFORM_IS_EVB_2
#include <ff.h>
#else
#include <sys/stat.h>
#include <cstdio>
#include <time.h>
#endif

#include <iostream>
#include <algorithm>
#include <cstring>

#if PLATFORM_IS_WINDOWS
    #include <windows.h>
#endif

#if PLATFORM_IS_LINUX || PLATFORM_IS_APPLE
#include <sys/statvfs.h>
#include <dirent.h>
#include <unistd.h>
#endif

#if 0
    #define DEBUG_PRINT(x) std::cout << x ;
#else
    #define DEBUG_PRINT(x)  // Do nothing if DEBUG is not defined
#endif

namespace ISFileManager {

    #if PLATFORM_IS_LINUX
    const char* path_seperator = "/";
    #elif PLATFORM_IS_WINDOWS
    const char* path_seperator = "\\";
    #else
    const char* path_seperator = nullptr;
    #endif

    bool PathIsDir(const std::string& path)
    {
    #if PLATFORM_IS_EVB_2
        FILINFO info;
        FRESULT result = f_stat(path.c_str(), &info);
        return (result == FR_OK) && (info.fattrib & AM_DIR);
    #else
        struct stat sb;
        return (stat(path.c_str(), &sb) == 0 && S_ISDIR(sb.st_mode));
    #endif
    }

    bool GetAllFilesInDirectory(const std::string& directory, bool recursive, const std::string& regexPattern, std::vector<std::string>& files)
    {
        size_t startSize = files.size();
        std::regex* rePtr = NULL;
        std::regex re;
        if (regexPattern.length() != 0)
        {
            re = std::regex(regexPattern, std::regex::icase);
            rePtr = &re;
        }

    #if PLATFORM_IS_EVB_2

        {
            FRESULT result;
            FILINFO info;
            DIR dir;
            char *file_name;
    #if _USE_LFN
            static char longFileName[_MAX_LFN + 1];
            info.lfname = longFileName;
            info.lfsize = sizeof(longFileName);
    #endif


            result = f_opendir(&dir, directory.c_str());
            if (result == FR_OK) {
                while (true) {
                    result = f_readdir(&dir, &info);
                    if (result != FR_OK || info.fname[0] == 0)
                    {
                        break;
                    }
    #if _USE_LFN
                    file_name = *info.lfname ? info.lfname : info.fname;
    #else
                    file_name = info.fname;
    #endif
                    std::string full_file_name = directory + "/" + file_name;


                    if (file_name[0] == '.' || (rePtr != NULL && !regex_search(full_file_name, re)))
                    {
                        continue;
                    }
                    else if (info.fattrib & AM_DIR) {
                        if (recursive)
                        {
                            GetAllFilesInDirectory(full_file_name, true, files);
                        }
                        continue;
                    }
                    files.push_back(full_file_name);
                }
            }

        }

    #elif PLATFORM_IS_WINDOWS

        HANDLE dir;
        WIN32_FIND_DATAA file_data;
        if ((dir = FindFirstFileA((directory + "/*").c_str(), &file_data)) == INVALID_HANDLE_VALUE)
        {
            return false;
        }
        do
        {
            std::string file_name = file_data.cFileName;
            std::string full_file_name = directory + "/" + file_name;
            if (file_name[0] == '.' || (rePtr != NULL && !regex_search(full_file_name, re)))
            {
                continue;
            }
            else if ((file_data.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY) != 0)
            {
                if (recursive)
                {
                    GetAllFilesInDirectory(full_file_name, true, files);
                }
                continue;
            }
            files.push_back(full_file_name);
        } while (FindNextFileA(dir, &file_data));
        FindClose(dir);

    #else // Linux

        class dirent* ent;
        class stat st;
        DIR* dir = opendir(directory.c_str());

        if (dir == NULL) {
            return false;
        }

        while ((ent = readdir(dir)) != NULL) {
            const std::string file_name = ent->d_name;
            const std::string full_file_name = directory + "/" + file_name;

            // if file is current path or does not exist (-1) then continue
            if (file_name[0] == '.' || stat(full_file_name.c_str(), &st) == -1 ||
                (rePtr != NULL && !regex_search(full_file_name, re))) {
                continue;
            }
            else if ((st.st_mode & S_IFDIR) != 0) {
                if (recursive) {
                    GetAllFilesInDirectory(full_file_name, true, files);
                }
                continue;
            }

            files.push_back(full_file_name);
        }
        closedir(dir);

    #endif

        return (files.size() != startSize);
    }

    bool GetAllFilesInDirectory(const std::string& directory, bool recursive, std::vector<std::string>& files)
    {
        return GetAllFilesInDirectory(directory, recursive, "", files);
    }

    bool DeleteFile(const std::string& fullFilePath)
    {
    #if PLATFORM_IS_EVB_2
        return (f_unlink(fullFilePath.c_str()) == FR_OK);
    #else
        return (std::remove(fullFilePath.c_str()) == 0);
    #endif

    }

    void DeleteDirectory(const std::string& directory, bool recursive)
    {

    #if PLATFORM_IS_EVB_2
        // Always recursive
        f_unlink(directory.c_str());
    #elif PLATFORM_IS_WINDOWS

        HANDLE dir;
        WIN32_FIND_DATAA file_data;
        if ((dir = FindFirstFileA((directory + "/*").c_str(), &file_data)) == INVALID_HANDLE_VALUE)
        {
            return;
        }
        do
        {
            const std::string file_name = file_data.cFileName;
            const std::string full_file_name = directory + "/" + file_name;
            const bool is_directory = (file_data.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY) != 0;
            if (file_name[0] == '.')
            {
                continue;
            }
            else if (is_directory)
            {
                if (recursive)
                {
                    DeleteDirectory(full_file_name, true);
                }
                continue;
            }
            std::remove(full_file_name.c_str());
        } while (FindNextFileA(dir, &file_data));
        FindClose(dir);

    #else

        class dirent* ent;
        class stat st;

        DIR* dir;
        if ((dir = opendir(directory.c_str())) == NULL) {
            return;
        }

        while ((ent = readdir(dir)) != NULL) {
            const std::string file_name = ent->d_name;
            const std::string full_file_name = directory + "/" + file_name;

            // if file is current path or does not exist (-1) then continue
            if (file_name[0] == '.' || stat(full_file_name.c_str(), &st) == -1) {
                continue;
            } else if ((st.st_mode & S_IFDIR) != 0) {
                if (recursive) {
                    DeleteDirectory(full_file_name, true);
                }
                continue;
            }
            std::remove(full_file_name.c_str());
        }
        closedir(dir);

    #endif

        _RMDIR(directory.c_str());
    }

    uint64_t GetDirectorySpaceUsed(const std::string& directory, bool recursive)
    {
        std::vector<file_info_t> files;
        return GetDirectorySpaceUsed(directory, "", files, true, recursive);
    }

    uint64_t GetDirectorySpaceUsed(const std::string& directory, std::vector<file_info_t>& files, bool sortByDate, bool recursive)
    {
        return GetDirectorySpaceUsed(directory, "", files, sortByDate, recursive);
    }

    uint64_t GetDirectorySpaceUsed(const std::string& directory, std::string regexPattern, std::vector<file_info_t>& files,
                                                bool sortByDate, bool recursive)
    {
    #if PLATFORM_IS_EVB_2
        static_assert((sizeof(time_t) >= 2 * sizeof(WORD)), "time_t must be at least 2* DWORD, since we store the FatFs date/time (DWORD) in a time_t here.");
    #endif

        std::vector<std::string> fileNames;
        ISFileManager::GetAllFilesInDirectory(directory, recursive, regexPattern, fileNames);
        uint64_t spaceUsed = 0;
        for (unsigned int i = 0; i < fileNames.size(); i++)
        {
            file_info_t info;
            info.name = fileNames[i];
    #if PLATFORM_IS_EVB_2
            FILINFO filInfo;
            f_stat(info.name.c_str(), &filInfo);
            info.size = filInfo.fsize;
            info.lastModificationDate = static_cast<time_t>(filInfo.ftime);
            info.lastModificationDate |= static_cast<time_t>(filInfo.fdate) << (sizeof(filInfo.ftime) * 8);
    #else
            struct stat st;
            stat(info.name.c_str(), &st);
            info.size = st.st_size;
            info.lastModificationDate = st.st_mtime;
    #endif
            files.push_back(info);
            spaceUsed += info.size;
        }

        if (sortByDate) {
            struct
            {
                bool operator()(const file_info_t& a, const file_info_t& b)
                {
                    int compare = (a.lastModificationDate == b.lastModificationDate ? 0 : (a.lastModificationDate < b.lastModificationDate ? -1 : 1));
                    if (compare == 0) {
                        compare = a.name.compare(b.name);
                    }
                    return compare < 0;
                }
            } customSortDate;
            sort(files.begin(), files.end(), customSortDate);
        } else {
            struct
            {
                bool operator()(const file_info_t& a, const file_info_t& b)
                {
                    return a.name < b.name;
                }
            } customSortName;
            sort(files.begin(), files.end(), customSortName);
        }

        return spaceUsed;
    }

    uint64_t GetDirectorySpaceAvailable(const std::string& directory)
    {
    #if PLATFORM_IS_EVB_2
        FATFS *fs;
        DWORD free_clusters;
        DWORD sector_size;

        /* Get volume information and free clusters of drive 1 */
        FRESULT result = f_getfree("0:", &free_clusters, &fs);
        if (result != FR_OK)
        {
            return 0;
        }
    #if _MAX_SS == 512
        sector_size = 512;
    #else
        sector_size = fs->ssize;
    #endif

        return free_clusters * fs->csize * sector_size;

    #elif PLATFORM_IS_WINDOWS

        ULARGE_INTEGER space;
        memset(&space, 0, sizeof(space));
        char fullPath[MAX_PATH];
        GetFullPathNameA(directory.c_str(), MAX_PATH, fullPath, NULL);
        bool created = (_MKDIR(fullPath) == 0);
        GetDiskFreeSpaceExA(fullPath, &space, NULL, NULL);
        if (created)
        {
            _RMDIR(fullPath);
        }
        return (uint64_t)space.QuadPart;

    #else

        struct statvfs stat;
        memset(&stat, 0, sizeof(stat));
        char fullPath[PATH_MAX];
        bool created = (_MKDIR(directory.c_str()) == 0);

        if(!created)
            CreateDirectory(directory.c_str());

        if (realpath(directory.c_str(), fullPath) == NULL)
        {
            printf("GetDirectorySpaceAvailable error %s\n", directory.c_str());
            return 0;
        }
        statvfs(fullPath, &stat);
        if (created)
        {
            _RMDIR(directory.c_str());
        }

    #if (LOG_DEBUG_GEN == 2)
        // advance_cursor();
    #elif LOG_DEBUG_GEN
        printf("GetDirectorySpaceAvailable %s %d\n", directory.c_str(), created);
    #endif
        return (uint64_t)stat.f_bsize * (uint64_t)stat.f_bavail;

    #endif

    }

    uint64_t GetDirectoryDriveTotalSize(const std::string& directory)
    {
    #if PLATFORM_IS_EVB_2
        FATFS *fs;
        DWORD free_clusters;
        DWORD sector_size;

        /* Get volume information and free clusters of drive 1 */
        FRESULT result = f_getfree("0:", &free_clusters, &fs);
        if (result != FR_OK)
        {
            return 0;
        }
    #if _MAX_SS == 512
        sector_size = 512;
    #else
        sector_size = fs->ssize;
    #endif

        return free_clusters * fs->csize * sector_size;

    #elif PLATFORM_IS_WINDOWS

        ULARGE_INTEGER space, totalBytes, totalFreeBytes;
        memset(&space, 0, sizeof(space));
        char fullPath[MAX_PATH];
        GetFullPathNameA(directory.c_str(), MAX_PATH, fullPath, NULL);
        bool created = (_MKDIR(fullPath) == 0);
        GetDiskFreeSpaceExA(fullPath, &space, &totalBytes, &totalFreeBytes);
        if (created)
        {
            _RMDIR(fullPath);
        }
        return (uint64_t)totalBytes.QuadPart;

    #else

        struct statvfs stat;
        memset(&stat, 0, sizeof(stat));
        char fullPath[PATH_MAX];
        bool created = (_MKDIR(directory.c_str()) == 0);

        if(!created)
            CreateDirectory(directory.c_str());

        if (realpath(directory.c_str(), fullPath) == NULL)
        {
            printf("GetDirectoryDriveTotalSize error %s\n", directory.c_str());
            return 0;
        }
        statvfs(fullPath, &stat);
        if (created)
        {
            _RMDIR(directory.c_str());
        }

    #if (LOG_DEBUG_GEN == 2)
        // advance_cursor();
    #elif LOG_DEBUG_GEN
        printf("GetDirectoryDriveTotalSize %s %d\n", directory.c_str(), created);
    #endif
        return (uint64_t)stat.f_frsize * (uint64_t)stat.f_blocks;

    #endif

    }

    bool CreateDirectory(const std::string& path)
    {
    #if defined(_WIN32)
        int ret = _mkdir(path.c_str());
    #elif PLATFORM_IS_EVB_2
        int ret = 0;
    #else
        mode_t mode = 0755;
        int ret = mkdir(path.c_str(), mode);
    #endif
        if (ret == 0)
            return true;

        switch (errno)
        {
        case ENOENT:
            // parent didn't exist, try to create it
            {
                size_t pos = path.find_last_of('/');
                if (pos == std::string::npos)
    #if defined(_WIN32)
                    pos = path.find_last_of('\\');
                if (pos == std::string::npos)
    #endif
                    return false;
                if (!CreateDirectory( path.substr(0, pos) ))
                    return false;
            }
            // now, try to create again
    #if defined(_WIN32)
            return 0 == _mkdir(path.c_str());
    #elif PLATFORM_IS_EVB_2

    #else
            return 0 == mkdir(path.c_str(), mode);

    #endif

        case EEXIST:
            // done!
            return PathIsDir(path);

        default:
            return false;
        }
    }

    std::string GetFileName(const std::string& path)
    {
        size_t lastSepIndex = path.find_last_of("\\/");
        if (lastSepIndex != std::string::npos)
        {
            return path.substr(lastSepIndex + 1);
        }
        return path;
    }

    std::string GetParentDirectory(const std::string& path)
    {
        size_t lastSepIndex = path.find_last_of("\\/");
        if (lastSepIndex != std::string::npos)
        {
            return path.substr(lastSepIndex + 1);
        }
        return path;
    }

    /***
     * Returns the current working directory as a std::string.
     * @return Current working directory.
    */
    std::string CurrentWorkingDirectory()
    {
        char curdir[256] = { 0 };
        if (_GETCWD(curdir, sizeof(curdir)) != nullptr)
        {
            return std::string(curdir);
        }

        // Error
        return "";
    }

    /**
     * Returns true if the specified path is consider an absolute path
     * @param path
     * @return true if absolute, false if relative
     */
    bool isPathAbsolute(const std::string& path) {
        #if PLATFORM_IS_WINDOWS
        if (path[1] == ':') return true;
        #else
        if (path.find(path_seperator) == 0)
            return true;
        #endif
        return false;
    }

    /**
     * returns an absolute path to the parent directory of the specified path
     * If 'path' is a relative path, it is applied to the current working directory
     * @param path a complete path
     * @param parent a string representing the path to the parent directory
     * @return true on success, otherwise false
     */
    bool getParentDirectory(const std::string& path, std::string& parent) {
        std::string realPath = path;
        // first check if path is absolute
        if (!isPathAbsolute(path)) {
            realPath = CurrentWorkingDirectory() + path_seperator + path;
        }

        auto pos = realPath.rfind(path_seperator);
        if (pos == std::string::npos)
            return false;

        parent = std::string(path, 0, pos);
        return true;
    }

    /**
     * Extracts and stores the base, filename, and ext(ension) components from the specified path
     * @param path the path to parse
     * @param parent the base or parent directory which contains the file. This is always an absolute path.
     * @param file the filename which the path references. This is just the filename with the extension, and does not include any directory/path information.
     * @param ext the filename extension of the filename. This is the same as the last matching characters of the filename, and is included for convenience.
     * @return true if there was sufficient path information to extract the parent of the path, otherwise false.  This is NOT an error condition,
     * but instead indicates (if true) that parent contains meaningful data.
     */
    bool getPathComponents(const std::string& path, std::string& parent, std::string& file, std::string& ext) {
        bool hasParent = getParentDirectory(path, parent);

        auto filePos = path.rfind(path_seperator);
        if (filePos == std::string::npos) filePos = 0;
        file = std::string(path, filePos + 1, path.length() - filePos);

        auto extPos = path.rfind('.');
        if (extPos == std::string::npos) extPos = 0;
        ext = std::string(path, extPos, path.length() - extPos);

        return hasParent;
    }

    bool TouchFile(const std::string& path)
    {
    #if PLATFORM_IS_EVB_2
        DWORD datetime = get_fattime();
        FILINFO info;
        info.fdate = static_cast<WORD>(datetime >> sizeof(WORD));
        info.ftime = static_cast<WORD>(datetime);
        f_utime(path.c_str(), &info);
        return true;
    #else
        _UTIMEBUF buf{time(NULL), time(NULL)};
        return (_UTIME(path.c_str(), &buf) == 0);
    #endif
    }

    // Struct to hold file information
    struct FileInfo {
        std::string path;
        std::uintmax_t size;
        std::time_t last_modified;

        bool operator<(const FileInfo& other) const {
            return last_modified < other.last_modified;
        }
    };

#if PLATFORM_IS_WINDOWS
    std::string ConvertWCHARToString(const WCHAR* wideCharArray) {
        int bufferLength = WideCharToMultiByte(CP_UTF8, 0, wideCharArray, -1, NULL, 0, NULL, NULL);
        std::string str(bufferLength - 1, 0);
        WideCharToMultiByte(CP_UTF8, 0, wideCharArray, -1, &str[0], bufferLength - 1, NULL, NULL);
        return str;
    }

    std::wstring ConvertStringToWCHAR(const std::string& utf8String) {
        int bufferLength = MultiByteToWideChar(CP_UTF8, 0, utf8String.c_str(), -1, NULL, 0);
        std::wstring wideString(bufferLength, 0);
        MultiByteToWideChar(CP_UTF8, 0, utf8String.c_str(), -1, &wideString[0], bufferLength);
        return wideString;
    }

#ifdef UNICODE  // Convert WCHAR <-> std::string
#define WCHAR_TO_STRING(str)   ConvertWCHARToString(str)
#define STRING_TO_WCHAR(str)   ConvertStringToWCHAR(str)
#else   // No conversion
#define WCHAR_TO_STRING(str)   (str)
#define STRING_TO_WCHAR(str)   (str)
#endif

#endif

    /**
     * @brief Function to get the size and modification time of a file
     * 
     * @param path File path
     * @param file_info output for file info
     * @return true if file info could be populated, false if not.
     */
    bool get_file_info(const std::string& path, FileInfo& file_info) {
#if PLATFORM_IS_WINDOWS
        WIN32_FILE_ATTRIBUTE_DATA file_data;
        if (GetFileAttributesEx(STRING_TO_WCHAR(path).c_str(), GetFileExInfoStandard, &file_data)) {
            ULARGE_INTEGER file_size;
            file_size.LowPart = file_data.nFileSizeLow;
            file_size.HighPart = file_data.nFileSizeHigh;

            file_info.path = path;
            file_info.size = file_size.QuadPart;

            // Convert FILETIME to Unix epoch time
            FILETIME ft = file_data.ftLastWriteTime;
            ULARGE_INTEGER ull;
            ull.LowPart = ft.dwLowDateTime;
            ull.HighPart = ft.dwHighDateTime;
            file_info.last_modified = ull.QuadPart / 10000000ULL - 11644473600ULL;

            return true;
        }
        return false;
#else
        struct stat file_stat;
        if (stat(path.c_str(), &file_stat) == 0) {
            if (S_ISREG(file_stat.st_mode)) {
                file_info.path = path;
                file_info.size = file_stat.st_size;
                file_info.last_modified = file_stat.st_mtime;
                return true;
            }
        }
        return false;
#endif
    }

    /**
     * @brief Function to recursively gather files and their information into the files 
     * 
     * @param directory Directory where files are to be searched. 
     * @param files Output list of all files found.
     */
    void get_all_files(const std::string& directory, std::vector<FileInfo>& files) {
#if PLATFORM_IS_WINDOWS
        WIN32_FIND_DATA find_file_data;
        HANDLE hFind = FindFirstFile(STRING_TO_WCHAR(directory + "\\*").c_str(), &find_file_data);

        if (hFind == INVALID_HANDLE_VALUE) {
            std::cerr << "Error: Could not open directory " << directory << std::endl;
            return;
        }

        do {
            const std::string file_name = WCHAR_TO_STRING(find_file_data.cFileName);
            if (file_name == "." || file_name == "..") continue;

            const std::string full_path = directory + "\\" + file_name;

            if (find_file_data.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY) {
                // Recurse into subdirectories
                get_all_files(full_path, files);
            } else {
                // Regular file
                FileInfo info;
                if (get_file_info(full_path, info)) {
                    files.push_back(info);
                }
            }
        } while (FindNextFile(hFind, &find_file_data) != 0);

        FindClose(hFind);
#else
        DIR* dir = opendir(directory.c_str());
        if (!dir) {
            std::cerr << "Error: Could not open directory " << directory << std::endl;
            return;
        }

        struct dirent* entry;
        while ((entry = readdir(dir)) != nullptr) {
            std::string name = entry->d_name;

            // Skip "." and ".."
            if (name == "." || name == "..")
                continue;

            std::string path = directory + "/" + name;
            
            if (entry->d_type == DT_DIR) {
                // Recurse into subdirectories
                get_all_files(path, files);
            } else if (entry->d_type == DT_REG) {
                // Regular file
                FileInfo info;
                if (get_file_info(path, info)) {
                    files.push_back(info);
                }
            }
        }

        closedir(dir);
#endif
    }

    // Function to remove files until total size is under the limit
    void RemoveOldestFiles(const std::string& directory, std::uintmax_t target_size) {
        std::vector<FileInfo> files;

        // Get all files in the directory and subdirectories
        get_all_files(directory, files);

        // Calculate total size
        std::uintmax_t total_size = 0;
        for (const auto& file : files) {
            total_size += file.size;
        }

        DEBUG_PRINT("Initial total size: " << total_size << " bytes\n");

        // If total size is already under the target size, no files need to be removed
        if (total_size <= target_size) {
            DEBUG_PRINT("Total size is already under the target size.\n");
            return;
        }

        // Sort files by their last modified time (oldest first)
        std::sort(files.begin(), files.end());

        // Remove the oldest files until the total size is under the target size
        for (const auto& file : files) {
            if (total_size <= target_size) {
                break;  // Stop when the total size is within the target limit
            }

            DEBUG_PRINT("Removing: " << file.path << " (" << file.size << " bytes)\n");
#if PLATFORM_IS_WINDOWS
            if (DeleteFile(file.path.c_str())) {
#else
            if (remove(file.path.c_str()) == 0) {
#endif
                total_size -= file.size;
            } else {
                std::cerr << "Error removing file: " << file.path << std::endl;
            }
        }

        DEBUG_PRINT("Final total size: " << total_size << " bytes\n");
    }

    // Function to remove empty directories
    bool RemoveEmptyDirectories(const std::string& directory) {
    #if PLATFORM_IS_WINDOWS
        WIN32_FIND_DATA find_file_data;
        HANDLE hFind = FindFirstFile(STRING_TO_WCHAR(directory + "\\*").c_str(), &find_file_data);

        if (hFind == INVALID_HANDLE_VALUE) {
            return false;  // Couldn't open directory
        }

        bool is_empty = true;

        do {
            const std::string file_name = WCHAR_TO_STRING(find_file_data.cFileName);
            if (file_name == "." || file_name == "..") continue;

            const std::string full_path = directory + "\\" + file_name;

            if (find_file_data.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY) {
                // Recursively check if the subdirectory is empty
                if (!RemoveEmptyDirectories(full_path)) {
                    is_empty = false;
                }
            } else {
                // Found a file, directory is not empty
                is_empty = false;
            }
        } while (FindNextFile(hFind, &find_file_data) != 0);

        FindClose(hFind);

        // If the directory is empty, remove it
        if (is_empty) {
            if (RemoveDirectory(STRING_TO_WCHAR(directory).c_str())) {
                DEBUG_PRINT("Removed empty directory: " << directory << std::endl);
            } else {
                std::cerr << "Error removing directory: " << directory << std::endl;
            }
        }

        return is_empty;
#else
        DIR* dir = opendir(directory.c_str());
        if (!dir) {
            return false;  // Couldn't open directory
        }

        struct dirent* entry;
        bool is_empty = true;

        while ((entry = readdir(dir)) != nullptr) {
            std::string name = entry->d_name;

            // Skip "." and ".."
            if (name == "." || name == "..")
                continue;

            std::string path = directory + "/" + name;

            if (entry->d_type == DT_DIR) {
                // Recursively check if the subdirectory is empty
                if (!RemoveEmptyDirectories(path)) {
                    is_empty = false;
                }
            } else {
                // Found a file, directory is not empty
                is_empty = false;
            }
        }

        closedir(dir);

        // If the directory is empty, remove it
        if (is_empty) {
            if (rmdir(directory.c_str()) == 0) {
                DEBUG_PRINT("Removed empty directory: " << directory << std::endl);
            } else {
                std::cerr << "Error removing directory: " << directory << std::endl;
            }
        }

        return is_empty;
#endif
    }

} // namespace ISFileManager
