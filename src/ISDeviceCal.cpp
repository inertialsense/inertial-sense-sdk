/**
 * @file ISDeviceCal.cpp
 * @brief This file contains the implementation of the ISDeviceCal class, which provides
 * functionality to load and save device calibration data from/to JSON files. It handles
 * different versions of calibration data formats and provides a structured way to manage
 * sensor calibration information, including orthonormalization, temperature compensation,
 * and motion calibration.
 *
 * The file includes helper functions for string manipulation, file system operations,
 * and matrix/vector conversions, which are essential for parsing and generating the
 * JSON calibration files.
 *
 * @author Inertial Sense, Inc.
 * @copyright Copyright (c) 2025 Inertial Sense, Inc. All rights reserved.
 */

#include "ISDeviceCal.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <cstring>
#include <cstdarg>
#include <regex>
#include <sys/stat.h>
#include <sys/types.h>
#ifdef _WIN32
#include <direct.h>
#define mkdir(dir, mode) _mkdir(dir)
#else
#include <errno.h>
#endif

#include "com_manager.h"
#include "ISDevice.h"
#include "ISLogFile.h"

using namespace std;
using json = nlohmann::json;

// Macro for checking if a 3-element vector is all zeros
#define VEC3_ALL_ZERO(v) ((v)[0] == 0.0f && (v)[1] == 0.0f && (v)[2] == 0.0f)

// Helper function prototypes
static std::vector<std::string> split(const std::string& s, char delimiter);
static std::string getFilename(const std::string& path);
static std::string getParentPath(const std::string& path);
static bool createDirectoryRecursive(const std::string& path);
static bool mat3x3_IsIdentity(const float* mat);

/**
 * @brief Splits a string by a delimiter.
 * @param s The string to split.
 * @param delimiter The character to split the string by.
 * @return A vector of strings.
 */
static std::vector<std::string> split(const std::string& s, char delimiter) {
    std::vector<std::string> tokens;
    std::string token;
    std::istringstream tokenStream(s);
    while (std::getline(tokenStream, token, delimiter)) {
        tokens.push_back(token);
    }
    return tokens;
}

/**
 * @brief Extracts the filename from a file path.
 * @param path The file path.
 * @return The filename.
 */
static std::string getFilename(const std::string& path) {
    size_t pos = path.find_last_of("/\\");
    if (pos != std::string::npos) {
        return path.substr(pos + 1);
    }
    return path;
}

/**
 * @brief Gets the parent directory of a file path.
 * @param path The file path.
 * @return The parent directory path.
 */
static std::string getParentPath(const std::string& path) {
    size_t pos = path.find_last_of("/\\");
    if (pos != std::string::npos) {
        return path.substr(0, pos);
    }
    return ".";
}

/**
 * @brief Recursively creates a directory.
 * @param path The directory path to create.
 * @return True if the directory was created successfully or already exists, false otherwise.
 */
static bool createDirectoryRecursive(const std::string& path) {
    if (path.empty()) return true;
    
    struct stat info;
    if (stat(path.c_str(), &info) == 0 && (info.st_mode & S_IFDIR)) {
        return true; // Directory already exists
    }
    
    // Create parent directory first
    std::string parent = getParentPath(path);
    if (!parent.empty() && parent != "." && parent != path) {
        if (!createDirectoryRecursive(parent)) {
            return false;
        }
    }
    
    // Create this directory
    return mkdir(path.c_str(), 0755) == 0 || errno == EEXIST;
}

/**
 * @brief Checks if a 3x3 matrix is an identity matrix.
 * @param mat Pointer to the 3x3 matrix (row-major).
 * @return True if the matrix is an identity matrix, false otherwise.
 */
static bool mat3x3_IsIdentity(const float* mat) {
    const float epsilon = 1e-6f;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            float expected = (i == j) ? 1.0f : 0.0f;
            if (fabs(mat[i * 3 + j] - expected) > epsilon) {
                return false;
            }
        }
    }
    return true;
}

/**
 * @brief Converts a float vector to a space-delimited string.
 * @param vec Pointer to the float vector.
 * @param len The number of elements in the vector.
 * @return The string representation of the vector.
 */
std::string VectorToString(const float* vec, int len) {
    std::ostringstream oss;
    for (int i = 0; i < len; i++) {
        if (i > 0) oss << " ";
        oss << vec[i];
    }
    return oss.str();
}

/**
 * @brief Converts a space-delimited string to a float vector.
 * @param str The space-delimited string.
 * @param vec Pointer to the float vector to store the result.
 * @param len The number of elements in the vector.
 */
void StringToVector(const std::string& str, float* vec, int len) {
    std::vector<std::string> vals = split(str, ' ');
    for (int i = 0; i < len && i < (int)vals.size(); i++) {
        vec[i] = std::stof(vals[i]);
    }
}

// Default row terminator for matrix string representations. Use '\n' to keep
// each row on its own line; some tools may prefer ';' as a row separator,
// in which case this value can be updated accordingly.
#define MATRIX_ROW_END_CHAR     '\n'

/**
 * @brief Converts a matrix to a string.
 * @param mat Pointer to the matrix (row-major).
 * @param rows The number of rows in the matrix.
 * @param cols The number of columns in the matrix.
 * @return The string representation of the matrix.
 */
std::string MatrixToString(const float* mat, int rows, int cols) {
    std::ostringstream oss;
    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
            if (j > 0) oss << " ";
            oss << mat[i * cols + j];
        }
        if (i < rows - 1) oss << MATRIX_ROW_END_CHAR;
    }
    return oss.str();
}

/**
 * @brief Converts a string to a matrix.
 * @param str The string representation of the matrix.
 * @param mat Pointer to the matrix to store the result (row-major).
 * @param rows The number of rows in the matrix.
 * @param cols The number of columns in the matrix.
 */
void StringToMatrix(const std::string& str, float* mat, int rows, int cols) {
    std::string normalized = str;
    // Normalize row separators to ';'
    for (char& ch : normalized) {
        if (ch == '\n') {
            ch = ';';
        }
    }
    std::vector<std::string> rowStrs = split(normalized, ';');
    int idx = 0;
    for (const auto& rowStr : rowStrs) {
        std::vector<std::string> vals = split(rowStr, ' ');
        for (const auto& val : vals) {
            if (idx < rows * cols) {
                mat[idx++] = std::stof(val);
            }
        }
    }
}

/**
 * @brief Converts sensor calibration info version to a string.
 * @param info The sensor calibration info.
 * @return The version string.
 */
static std::string calInfoVersionToString(const sensor_cal_info_t& info) {
    return cISLogFile::formatString("%d.%d.%d %d", info.version[0], info.version[1], info.version[2], info.version[3]);
}

/**
 * @brief Converts sensor calibration info date to a string.
 * @param info The sensor calibration info.
 * @return The date string.
 */
static std::string calInfoDateToString(const sensor_cal_info_t& info) {
    return cISLogFile::formatString("%04d-%02d-%02d %d", info.calDate[0] + 2000, info.calDate[1], info.calDate[2], info.calDate[3]);
}

/**
 * @brief Converts sensor calibration info time to a string.
 * @param info The sensor calibration info.
 * @return The time string.
 */
static std::string calInfoTimeToString(const sensor_cal_info_t& info) {
    return cISLogFile::formatString("%02d:%02d:%02d %d", info.calTime[0], info.calTime[1], info.calTime[2], info.calTime[3]);
}

/**
 * @brief Parses a JSON object and populates a 3-axis temperature calibration data structure.
 * @param jTC The JSON object containing temperature calibration data.
 * @param key The key for the specific sensor data (e.g., "gyr1").
 * @param dev The destination nvm_sensor_tcal_3axis_t structure.
 */
static void JsonObjToTcal3axis(const json& jTC, const std::string& key, nvm_sensor_tcal_3axis_t &dev)
{
    if (jTC.contains(key))
    {
        const json& jPts = jTC[key];

        // Error check that indices match
        for (size_t p = 0; p < jPts.size(); p++)
        {
            const json& jPt = jPts[p];
            if (!jPt.contains("index") || jPt["index"].get<int>() != (int)p)
            {
                log_error(IS_LOG_CALIBRATION, "TC index mismatch.");
                return;
            }
        }

        // Error check number of points
        if (jPts.size() > TCAL_MAX_NUM_POINTS)
        {
            log_error(IS_LOG_CALIBRATION, "TC point index from file too large.");
            return;
        }

        dev.numPts = jPts.size();
        for (size_t p = 0; p < jPts.size(); p++)
        {
            const json& jPt = jPts[p];
            if (!jPt.contains("tmp"))
            {
                log_error(IS_LOG_CALIBRATION, "TC point tmp value missing.");
                return;
            }
            dev.pt[p].temp = jPt["tmp"].get<float>();
            if (!jPt.contains("SS"))
            {
                log_error(IS_LOG_CALIBRATION, "TC point SS array missing.");
                return;
            }
            const json& jSS = jPt["SS"];
            if (jSS.size() != 3)
            {
                log_error(IS_LOG_CALIBRATION, "TC point SS array wrong size.");
                return;
            }
            for (int i = 0; i < 3; i++)
            {
                dev.pt[p].ss[i] = jSS[i].get<float>();
            }
        }
    }
}

/**
 * @brief Parses a JSON object and populates a sensor temperature calibration group.
 * @param jTC The JSON object containing temperature calibration data for multiple sensors.
 * @param tcal The destination sensor_tcal_group_t structure.
 */
static void JsonObjToSensor(const json& jTC, sensor_tcal_group_t& tcal)
{
    for (int d = 0; d < MAX_IMU_DEVICES; d++)
    {
        JsonObjToTcal3axis(jTC, cISLogFile::formatString("gyr%d", d + 1), tcal.gyr[d]);
        JsonObjToTcal3axis(jTC, cISLogFile::formatString("acc%d", d + 1), tcal.acc[d]);
    }
    for (int d = 0; d < MAX_MAG_DEVICES; d++)
    {
        JsonObjToTcal3axis(jTC, cISLogFile::formatString("mag%d", d + 1), tcal.mag[d]);
    }
}

/**
 * @brief Parses a legacy (v1.2) JSON object and populates a sensor temperature calibration group.
 * @param jTC The JSON object containing legacy temperature calibration data.
 * @param device The device index.
 * @param tcal The destination sensor_tcal_group_t structure.
 */
static void JsonObjv1p2ToSensor(const json& jTC, int device, sensor_tcal_group_t &tcal)
{
    // Double check that indices match up
    for (size_t i = 0; i < jTC.size(); i++)
        if (jTC[i]["index"].get<int>() != (int)i)
        {
            log_error(IS_LOG_CALIBRATION, "TC index mismatch.");
            return;
        }

    if (jTC.size() > TCAL_MAX_NUM_POINTS)
    {
        log_error(IS_LOG_CALIBRATION, "TC point index from file too large.");
        return;
    }

    int numPts = jTC.size();
    tcal.gyr[device].numPts = numPts;
    tcal.acc[device].numPts = numPts;
    tcal.mag[device].numPts = numPts;

    for (size_t i = 0; i < jTC.size(); i++)
    {
        // Read tc point
        const json& jPt = jTC[i];

        if (jPt.contains("tmp"))
        {
            float temp = jPt["tmp"].get<float>();
            tcal.gyr[device].pt[i].temp = temp;
            tcal.acc[device].pt[i].temp = temp;
            tcal.mag[device].pt[i].temp = temp;
        }

        // Gyros
        if (jPt.contains("gyrS") && jPt.contains("gyrK"))
        {
            const json& jS = jPt["gyrS"];
            const json& jK = jPt["gyrK"];

            if (jS.size() == 3 || jK.size() == 3)
            {
                for (int j = 0; j < 3; j++)
                {
                    tcal.gyr[device].pt[i].ss[j] = jS[j].get<float>();
                }
            }
        }

        // Accels
        if (jPt.contains("accS") && jPt.contains("accK"))
        {
            const json& jS = jPt["accS"];
            const json& jK = jPt["accK"];

            if (jS.size() == 3 || jK.size() == 3)
            {
                for (int j = 0; j < 3; j++)
                {
                    tcal.acc[device].pt[i].ss[j] = jS[j].get<float>();
                }
            }
        }

        // Magnetometers
        if (jPt.contains("magS") && jPt.contains("magK"))
        {
            const json& jS = jPt["magS"];
            const json& jK = jPt["magK"];

            if (jS.size() == 3 || jK.size() == 3)
            {
                for (int j = 0; j < 3; j++)
                {
                    tcal.mag[device].pt[i].ss[j] = jS[j].get<float>();
                }
            }
        }
    }
}

/**
 * @class ISDeviceCal
 * @brief Provides methods to handle device calibration data.
 *
 * This class includes functions to load calibration data from JSON objects, strings,
 * and files, and to save calibration data back to JSON format. It supports various
 * calibration types, including orthonormalization, temperature compensation, and
 * motion calibration.
 */

/**
 * @brief Loads calibration data from a JSON object.
 * @param jObj The JSON object containing the calibration data.
 * @param ocal Pointer to sOrthoCal structure to store orthonormalization calibration data. Can be nullptr.
 * @param info Pointer to sensor_cal_info_t structure to store calibration info. Can be nullptr.
 * @param dinfo Pointer to sensor_data_info_t structure to store data info. Can be nullptr.
 * @param tcal Pointer to sensor_tcal_group_t structure to store temperature calibration data. Can be nullptr.
 * @param mcal Pointer to sensor_mcal_group_t structure to store motion calibration data. Can be nullptr.
 * @param pose Pointer to an integer to store the current pose. Can be nullptr.
 * @param filePath The file path of the JSON file, used for fallback date/time parsing.
 * @return True if loading is successful, false otherwise.
 */
bool ISDeviceCal::loadCalibrationFromJsonObj(const json& jObj, sOrthoCal *ocal, sensor_cal_info_t *info, sensor_data_info_t *dinfo, sensor_tcal_group_t *tcal, sensor_mcal_group_t* mcal, int* pose, const std::string& filePath)
{
    if (jObj.empty())
        return false;

    //  Calibration Info
    if (info)
    {
        if (jObj.contains("info"))
        {
            const json& jInf = jObj["info"];

            if (jInf.contains("version"))
            {
                std::string str = jInf["version"].get<std::string>();
                std::vector<std::string> strl = split(str, '.');
                if (strl.size()>=3)
                {
                    info->version[0] = std::stoi(strl[0]);
                    info->version[1] = std::stoi(strl[1]);
                    std::vector<std::string> parts = split(strl[2], ' ');
                    info->version[2] = std::stoi(parts[0]);
                }
                strl = split(str, ' ');
                if (strl.size()>=2)
                {
                    info->version[3] = std::stoi(strl[1]);
                }
            }
            if (jInf.contains("calDate"))
            {
                std::string str = jInf["calDate"].get<std::string>();
                std::vector<std::string> strl = split(str, '-');
                if (strl.size()>=3)
                {
                    info->calDate[0] = std::stoi(strl[0]) - 2000;
                    info->calDate[1] = std::stoi(strl[1]);
                    std::vector<std::string> parts = split(strl[2], ' ');
                    info->calDate[2] = std::stoi(parts[0]);
                }
                strl = split(str, ' ');
                if (strl.size()>=2)
                {
                    info->calDate[3] = std::stoi(strl[1]);
                }
            }
            if (jInf.contains("calTime"))
            {
                std::string str = jInf["calTime"].get<std::string>();
                std::vector<std::string> strl = split(str, ':');
                if (strl.size()>=3)
                {
                    info->calTime[0] = std::stoi(strl[0]);
                    info->calTime[1] = std::stoi(strl[1]);
                    std::vector<std::string> parts = split(strl[2], ' ');
                    info->calTime[2] = std::stoi(parts[0]);
                }
                strl = split(str, ' ');
                if (strl.size()>=2)
                {
                    info->calTime[3] = std::stoi(strl[1]);
                }
            }
            if (jInf.contains("devSerialNum"))
            {
                info->devSerialNum = jInf["devSerialNum"].get<int>();
            }
        }
        else if (!filePath.empty())
        {   // Fallback: Get date and time from filename if "info" block is missing
            memset(info, 0, sizeof(sensor_cal_info_t));

            info->version[0] = SENSOR_CAL_VER0;
            info->version[1] = SENSOR_CAL_VER1;
            info->version[2] = SENSOR_CAL_VER2;

            std::string fileName = getFilename(filePath);

            // Serial number - pattern: "SN(\d+)"
            std::regex rx("SN(\\d+)");
            std::smatch match;
            if (std::regex_search(fileName, match, rx) && match.size() > 1)
            {
                info->devSerialNum = std::stoi(match[1].str());
            }

            // Date from filename (e.g., ..._YYYYMMDD_...)
            std::vector<std::string> parts = split(fileName, '_');
            if (parts.size() > 2)
            {
                std::string str = parts[2];
                if (str.length() >= 8)
                {
                    info->calDate[0] = std::stoi(str.substr(0, 4)) - 2000;    // year
                    info->calDate[1] = std::stoi(str.substr(4, 2));           // month
                    info->calDate[2] = std::stoi(str.substr(6, 2));           // day
                    info->calDate[3] = 0;
                }
            }

            // Time from filename (e.g., ..._HHMM_...)
            if (parts.size() > 3)
            {
                std::string str = parts[3];
                if (str.length() >= 4)
                {
                    info->calTime[0] = std::stoi(str.substr(0, 2));        // hour
                    info->calTime[1] = std::stoi(str.substr(2, 2));        // minute
                    info->calTime[2] = 0;
                    info->calTime[3] = 0;
                }
            }
        }

        // Update info size and checksum
        info->size = sizeof(sensor_cal_info_t);
        info->checksum = flashChecksum32(info, info->size);
    }

    //  Orthonormalization and Motion Calibration
    if (ocal || mcal)
    {
        if (jObj.contains("calibration"))
        {
            loadMcFromJsonObj(jObj["calibration"], pose, ocal, mcal);
        }
    }

    //  Temperature Compensation
    if (tcal)
    {
        if (jObj.contains("tempComp"))
        {   // New file format
            JsonObjToSensor(jObj["tempComp"], *tcal);
        }
        else
        {   // Old file format (v1.2)
            if (jObj.contains("tempComp1"))
                JsonObjv1p2ToSensor(jObj["tempComp1"], 0, *tcal);

            if (jObj.contains("tempComp2"))
                JsonObjv1p2ToSensor(jObj["tempComp2"], 1, *tcal);
        }
    }

    if (info)
    {   // Update info size and checksum
        info->size = sizeof(sensor_cal_info_t);
        info->checksum = flashChecksum32(info, info->size);
    }
    if (dinfo)
    {   // Recompute data info size and checksum
        dinfo->size = sizeof(sensor_cal_v1p3_data_t);
        dinfo->checksum = flashChecksum32(dinfo, dinfo->size);
    }

    return true;
}

/**
 * @brief Loads calibration data from a JSON string.
 * @param jsonString The JSON string containing the calibration data.
 * @param ocal Pointer to sOrthoCal structure to store orthonormalization calibration data. Can be nullptr.
 * @param info Pointer to sensor_cal_info_t structure to store calibration info. Can be nullptr.
 * @param dinfo Pointer to sensor_data_info_t structure to store data info. Can be nullptr.
 * @param tcal Pointer to sensor_tcal_group_t structure to store temperature calibration data. Can be nullptr.
 * @param mcal Pointer to sensor_mcal_group_t structure to store motion calibration data. Can be nullptr.
 * @param pose Pointer to an integer to store the current pose. Can be nullptr.
 * @return True if loading is successful, false otherwise.
 */
bool ISDeviceCal::loadCalibrationFromJsonString(const std::string& jsonString, sOrthoCal *ocal, sensor_cal_info_t *info, sensor_data_info_t *dinfo, sensor_tcal_group_t *tcal, sensor_mcal_group_t* mcal, int* pose)
{
    json jObj;
    try {
        jObj = json::parse(jsonString);
    } catch (const json::parse_error& e) {
        log_error(IS_LOG_CALIBRATION, "Error parsing calibration data: %s", e.what());
        return false;
    }

    if (jObj.empty())
        return false;

    return loadCalibrationFromJsonObj(jObj, ocal, info, dinfo, tcal, mcal, pose);
}

/**
 * @brief Loads calibration data from a JSON file.
 * @param filePath The path to the JSON file.
 * @param ocal Pointer to sOrthoCal structure to store orthonormalization calibration data. Can be nullptr.
 * @param info Pointer to sensor_cal_info_t structure to store calibration info. Can be nullptr.
 * @param dinfo Pointer to sensor_data_info_t structure to store data info. Can be nullptr.
 * @param tcal Pointer to sensor_tcal_group_t structure to store temperature calibration data. Can be nullptr.
 * @param mcal Pointer to sensor_mcal_group_t structure to store motion calibration data. Can be nullptr.
 * @param pose Pointer to an integer to store the current pose. Can be nullptr.
 * @return True if loading is successful, false otherwise.
 */
bool ISDeviceCal::loadCalibrationFromJsonFile(const std::string& filePath, sOrthoCal *ocal, sensor_cal_info_t *info, sensor_data_info_t *dinfo, sensor_tcal_group_t *tcal, sensor_mcal_group_t* mcal, int* pose)
{
    // Read file
    std::ifstream jsonFile(filePath);
    if (!jsonFile.is_open())
    {
        log_error(IS_LOG_CALIBRATION, "Unable to open calibration file: %s", filePath.c_str());
        return false;
    }

    log_info(IS_LOG_CALIBRATION, "Loading calibration: %s", filePath.c_str());

    // Parse JSON
    json jObj;
    try {
        jsonFile >> jObj;
    } catch (const json::parse_error& e) {
        log_error(IS_LOG_CALIBRATION, "Error parsing calibration data: %s", e.what());
        return false;
    }
    jsonFile.close();

    if (jObj.empty())
    {   // Error in file.  Likely invalid data (i.e. Infinity or NAN).
        return false;
    }

    return loadCalibrationFromJsonObj(jObj, ocal, info, dinfo, tcal, mcal, pose, filePath);
}

/**
 * @brief Saves 3-axis sensor temperature calibration data to a JSON object.
 * @param sensor The nvm_sensor_tcal_3axis_t structure to save.
 * @return The JSON object representing the sensor data.
 */
static json saveSensorToJsonObj(nvm_sensor_tcal_3axis_t &sensor)
{
    json jTC = json::array();

    // Temp Comp
    for (int p = 0; p < (int)sensor.numPts; p++)
    {
        json jSS = json::array();
        for (int j = 0; j < 3; j++)
        {
            jSS.push_back(sensor.pt[p].ss[j]);
        }

        json jPt;
        jPt["index"] = p;
        jPt["tmp"] = sensor.pt[p].temp;
        jPt["SS"] = jSS;

        jTC.push_back(jPt);
    }

    return jTC;
}

/**
 * @brief Saves a group of sensor temperature calibration data to a JSON object.
 * @param tcal Pointer to the sensor_tcal_group_t structure to save.
 * @return The JSON object representing the temperature calibration data.
 */
static json saveTcToJsonObj(sensor_tcal_group_t* tcal)
{
    if (!tcal)
        return json::object();

    json jTC = json::object();

    for (int d = 0; d < MAX_IMU_DEVICES; d++)
    {
        if (tcal->gyr[d].numPts > 0)
        {
            jTC[cISLogFile::formatString("gyr%d", d + 1)] = saveSensorToJsonObj(tcal->gyr[d]);
        }
        if (tcal->acc[d].numPts > 0)
        {
            jTC[cISLogFile::formatString("acc%d", d + 1)] = saveSensorToJsonObj(tcal->acc[d]);
        }
    }

    for (int d = 0; d < MAX_MAG_DEVICES; d++)
    {
        if (tcal->mag[d].numPts > 0)
        {
            jTC[cISLogFile::formatString("mag%d", d + 1)] = saveSensorToJsonObj(tcal->mag[d]);
        }
    }

    return jTC;
}

/**
 * @brief Saves calibration data to a JSON file.
 * @param filePath The path to the JSON file.
 * @param cal Pointer to sOrthoCal structure containing orthonormalization calibration data. Can be nullptr.
 * @param info Pointer to sensor_cal_info_t structure containing calibration info. Can be nullptr.
 * @param tcal Pointer to sensor_tcal_group_t structure containing temperature calibration data. Can be nullptr.
 * @param mcal Pointer to sensor_mcal_group_t structure containing motion calibration data. Can be nullptr.
 * @param pose The current pose.
 * @return True if saving is successful, false otherwise.
 */
bool ISDeviceCal::saveCalibrationToJsonObj(const std::string& filePath, sOrthoCal* cal, sensor_cal_info_t* info, sensor_tcal_group_t* tcal, sensor_mcal_group_t* mcal, int pose)
{
    log_debug(IS_LOG_CALIBRATION, "saveCalibrationToJsonObj: %s\n", filePath.c_str());

    // Create parent directory if needed
    std::string parentPath = getParentPath(filePath);
    if (!parentPath.empty() && parentPath != ".") {
        createDirectoryRecursive(parentPath);
    }

    json jTop = json::object();

    //  Calibration Info
    if (info)
    {
        json jInf = json::object();
        jInf["version"] = calInfoVersionToString(*info);
        jInf["calDate"] = calInfoDateToString(*info);
        jInf["calTime"] = calInfoTimeToString(*info);
        jInf["devSerialNum"] = (int)(info->devSerialNum);
        jTop["info"] = jInf;
    }

    //  Temperature Compensation
    if (tcal)
    {
        jTop["tempComp"] = saveTcToJsonObj(tcal);
    }

    //  Orthonormalization
    if (mcal)
    {
        jTop["calibration"] = saveMcToJsonObj(filePath, pose, cal, mcal);
    }

    // Save to file
    std::ofstream file(filePath);
    if (!file.is_open())
        return false;

    file << jTop.dump(2);  // Pretty print with 2 space indent
    file.close();

    return true;
}

/**
 * @brief Checks if motion calibration data is present (i.e., not identity/zero).
 * @param orth Pointer to the 3x3 orthonormalization matrix.
 * @param bias Pointer to the 3-element bias vector.
 * @return True if motion calibration data is present, false otherwise.
 */
static bool motionCalPresent(const float* orth, const float* bias)
{
    return !mat3x3_IsIdentity(orth) || !VEC3_ALL_ZERO(bias);
}

/**
 * @brief Parses a JSON object and populates an sCalData structure for least squares data.
 * @param jObj The JSON object.
 * @param sen The destination sCalData structure.
 */
void OrthoLeastSquaresDataJsonObjToSensor(const json& jObj, sCalData &sen)
{
    // Y data
    if (jObj.contains("Y"))
        sen.Y.fromString(jObj["Y"].get<std::string>());

    // X data
    if (jObj.contains("Xhat"))
        sen.Xhat.fromString(jObj["Xhat"].get<std::string>());

    // A data
    if (jObj.contains("Ahat"))
        sen.Ahat.fromString(jObj["Ahat"].get<std::string>());
}

/**
 * @brief Saves an sCalData structure for least squares data to a JSON object.
 * @param jObj The destination JSON object.
 * @param sen The sCalData structure to save.
 * @return The updated JSON object.
 */
static json OrthoLeastSquaresDataSensorToJsonObj(const json& jObj, sCalData &sen)
{
    json result = jObj;
    result["Y"] = sen.Y.toString();
    result["Xhat"] = sen.Xhat.toString();
    result["Ahat"] = sen.Ahat.toString();

    return result;
}

/**
 * @brief Parses a JSON object and populates orthonormalization matrix and bias vector.
 * @param jObj The JSON object.
 * @param orth Pointer to the 3x3 orthonormalization matrix.
 * @param bias Pointer to the 3-element bias vector.
 */
static void OrthoJsonObjToSensor(const json& jObj, float* orth, float* bias)
{
    // Ortho matrix
    if (jObj.contains("A"))
        StringToMatrix(jObj["A"].get<std::string>(), orth, 3, 3);

    // Bias vector
    if (jObj.contains("bias"))
        StringToVector(jObj["bias"].get<std::string>(), bias, 3);
}

/**
 * @brief Saves orthonormalization matrix and bias vector to a JSON object.
 * @param jObj The destination JSON object.
 * @param orth Pointer to the 3x3 orthonormalization matrix.
 * @param bias Pointer to the 3-element bias vector.
 * @return The updated JSON object.
 */
static json OrthoSensorToJsonObj(json jObj, const float* orth, const float* bias)
{
    jObj["A"] = MatrixToString(orth, 3, 3);
    jObj["bias"] = VectorToString(bias, 3);

    return jObj;
}

/**
 * @brief Loads motion calibration data from a JSON object.
 * @param jCal The JSON object containing the motion calibration data.
 * @param pose Pointer to an integer to store the current pose. Can be nullptr.
 * @param ocal Pointer to sOrthoCal structure to store orthonormalization calibration data. Can be nullptr.
 * @param mcal Pointer to sensor_mcal_group_t structure to store motion calibration data. Can be nullptr.
 */
void ISDeviceCal::loadMcFromJsonObj(const json& jCal, int *pose, sOrthoCal *ocal, sensor_mcal_group_t *mcal)
{
    // Current pose
    if (pose && jCal.contains("curPose"))
    {
        *pose = jCal["curPose"].get<int>();
    }

    for (int d = 0; d < MAX_IMU_DEVICES; d++)
    {
        // Set default identity matrix for motion cal
        for (int i = 0; i < 9; i++)
        {
            float val = ((i == 0 || i == 4 || i == 8) ? 1.0f : 0.0f);
            mcal->pqr[d].orth[i] = val;
            mcal->acc[d].orth[i] = val;
        }

        // Gyro Data
        std::string key = cISLogFile::formatString("gyr%d", d+1);
        if (jCal.contains(key))
        {
			if (ocal) OrthoLeastSquaresDataJsonObjToSensor(jCal[key], ocal->gyr[d]);
            if (mcal) OrthoJsonObjToSensor(jCal[key], mcal->pqr[d].orth, mcal->pqr[d].bias);
        }

        // Accel Data
        key = cISLogFile::formatString("acc%d", d+1);
        if (jCal.contains(key))
        {
			if (ocal) OrthoLeastSquaresDataJsonObjToSensor(jCal[key], ocal->acc[d]);
            if (mcal) OrthoJsonObjToSensor(jCal[key], mcal->acc[d].orth, mcal->acc[d].bias);
        }
    }

    for (int d = 0; d < MAX_MAG_DEVICES; d++)
    {   // Mag Data
        // Set default identity matrix for motion cal
        for (int i = 0; i < 9; i++)
        {
            float val = ((i == 0 || i == 4 || i == 8) ? 1.0f : 0.0f);
            mcal->mag[d].orth[i] = val;
        }

        std::string key = cISLogFile::formatString("mag%d", d+1);
        if (jCal.contains(key))
        {
			if (ocal) OrthoLeastSquaresDataJsonObjToSensor(jCal[key], ocal->mag[d]);
            if (mcal) OrthoJsonObjToSensor(jCal[key], mcal->mag[d].orth, mcal->mag[d].bias);
        }
    }
}

/**
 * @brief Saves motion calibration data to a JSON object.
 * @param filePath The file path, used for comments in the JSON.
 * @param pose The current pose.
 * @param cal Pointer to sOrthoCal structure containing orthonormalization calibration data. Can be nullptr.
 * @param mcal Pointer to sensor_mcal_group_t structure containing motion calibration data. Can be nullptr.
 * @return The JSON object representing the motion calibration data.
 */
json ISDeviceCal::saveMcToJsonObj(const std::string& filePath, int pose, sOrthoCal *cal, sensor_mcal_group_t *mcal)
{
    json jCal = json::object();
    jCal["comment_line1"] = cISLogFile::formatString(" Sensor Calibration Data - %s ", filePath.c_str());
    jCal["comment_line2"] = " Y = A * X.  Y = truth data, X = adc measured data, A = calibration data. ";

    // Add last pose saved to json. If we crash, we can restart from here.
    if (pose != -1)
    {
        jCal["curPose"] = pose;
    }

    for (int d = 0; d < MAX_IMU_DEVICES; d++)
    {
        // Gyro Data
        if (motionCalPresent(mcal->pqr[d].orth, mcal->pqr[d].bias))
        {
            json jGyr = json::object();
            if (cal)   jGyr = OrthoLeastSquaresDataSensorToJsonObj(jGyr, cal->gyr[d]);
            if (mcal)  jGyr = OrthoSensorToJsonObj(jGyr, mcal->pqr[d].orth, mcal->pqr[d].bias);
            jCal[cISLogFile::formatString("gyr%d", d + 1)] = jGyr;
        }

        // Accel Data
        if (motionCalPresent(mcal->acc[d].orth, mcal->acc[d].bias))
        {
            json jAcc = json::object();
            if (cal)   jAcc = OrthoLeastSquaresDataSensorToJsonObj(jAcc, cal->acc[d]);
            if (mcal)  jAcc = OrthoSensorToJsonObj(jAcc, mcal->acc[d].orth, mcal->acc[d].bias);
            jCal[cISLogFile::formatString("acc%d", d + 1)] = jAcc;
        }
    }

    for (int d = 0; d < MAX_MAG_DEVICES; d++)
    {
        // Mag Data
        if (motionCalPresent(mcal->mag[d].orth, mcal->mag[d].bias))
        {
            json jMag = json::object();
            if (cal)   jMag = OrthoLeastSquaresDataSensorToJsonObj(jMag, cal->mag[d]);
            if (mcal)  jMag = OrthoSensorToJsonObj(jMag, mcal->mag[d].orth, mcal->mag[d].bias);
            jCal[cISLogFile::formatString("mag%d", d + 1)] = jMag;
        }
    }

    return jCal;
}

/**
 * @brief Uploads sensor calibration data to a device in steps.
 * @param port The communication port handle.
 * @param calUploadState The current state of the upload process. This is updated by the function.
 * @param cal The sensor_cal_t structure containing the calibration data to upload.
 * @return 1 if the upload is complete, 0 if it's in progress, -1 on error.
 */
int ISDeviceCal::uploadSensorCalStep(port_handle_t port, int &calUploadState, sensor_cal_t &cal)
{
    switch (calUploadState++)
    {
    // Upload Calibration Info
    case 0:     // General Info
        if (comManagerSendData(port, &(cal.info), DID_CAL_SC, sizeof(sensor_cal_info_t), offsetof(sensor_cal_t, info)) != 0) { return 0; }
        break;

    case 1:     // Data Info
        if (comManagerSendData(port, &(cal.data.dinfo), DID_CAL_SC, sizeof(sensor_data_info_t), offsetof(sensor_cal_t, data.dinfo)) != 0) { return 0; }
        break;

    // Upload Temperature Cal
    case 2:     // Temp comp - Gyros
        if (comManagerSendData(port, cal.data.tcal.gyr, DID_CAL_TEMP_COMP, MAX_IMU_DEVICES * sizeof(nvm_sensor_tcal_3axis_t), offsetof(sensor_tcal_group_t, gyr)) != 0) { return 0; }
        break;

    case 3:     // Temp comp - Accelerometers
        if (comManagerSendData(port, cal.data.tcal.acc, DID_CAL_TEMP_COMP, MAX_IMU_DEVICES * sizeof(nvm_sensor_tcal_3axis_t), offsetof(sensor_tcal_group_t, acc)) != 0) { return 0; }
        break;

    case 4:     // Temp comp - Magnetometers
        if (comManagerSendData(port, cal.data.tcal.mag, DID_CAL_TEMP_COMP, MAX_MAG_DEVICES * sizeof(nvm_sensor_tcal_3axis_t), offsetof(sensor_tcal_group_t, mag)) != 0) { return 0; }
        break;  

    // Upload Motion Cal
    case 5:     // Motion cal - Gyros
        if (comManagerSendData(port, cal.data.mcal.pqr, DID_CAL_MOTION, MAX_IMU_DEVICES * sizeof(sensor_motion_cal_t), offsetof(sensor_mcal_group_t, pqr)) != 0) { return 0; }
        break;

    case 6:     // Motion cal - Accelerometers
        if (comManagerSendData(port, cal.data.mcal.acc, DID_CAL_MOTION, MAX_IMU_DEVICES * sizeof(sensor_motion_cal_t), offsetof(sensor_mcal_group_t, acc)) != 0) { return 0; }
        break;

    case 7:     // Motion cal - Magnetometers
        if (comManagerSendData(port, cal.data.mcal.mag, DID_CAL_MOTION, MAX_MAG_DEVICES * sizeof(sensor_motion_cal_t), offsetof(sensor_mcal_group_t, mag)) != 0) { return 0; }
        // log_debug(IS_LOG_CALIBRATION, "Done uploadSensorCal() - hdl: %d, serial#: %d, devSerialNum: %d\n", port, serialNum, cal.info.devSerialNum);
        return 1;    // Done
        
    default:    // Error
        return -1;
    }

    return 0;
}


ISHttpRequest::Response ISDeviceCal::loadFromURL(const std::string& restBaseUrl, const dev_info_t& devInfo)
{
    using json = nlohmann::json;
    ISHttpRequest::Response resp;

    // // Build hardware type string (e.g., "IMX-5.0")
    // const char *typeName = "???";
    // switch (devInfo.hardwareType) {
    //     case IS_HARDWARE_TYPE_UINS: typeName = "uINS"; break;
    //     case IS_HARDWARE_TYPE_IMX:  typeName = "IMX"; break;
    //     case IS_HARDWARE_TYPE_GPX:  typeName = "GPX"; break;
    //     default: break;
    // }
    // std::string hwType = std::string(typeName) + "-" + std::to_string(devInfo.hardwareVer[0]) + "." + std::to_string(devInfo.hardwareVer[1]);
    std::string hwType = utils::getHardwareAsString(devInfo, false);

    log_info(IS_LOG_CALIBRATION, "[%s] Fetching calibration from DB for %s SN%ld", ISDevice::getIdAsString(devInfo).c_str(), hwType.c_str(), devInfo.serialNumber);

    // Step 1: Fetch device calibration list
    std::string deviceUrl = restBaseUrl + "/api/calibration/device/" + hwType + "/" + std::to_string(devInfo.serialNumber);
    ISHttpRequest::Response listResp = ISHttpRequest::get(deviceUrl);

    if (listResp.statusCode == -1)
    {
        log_error(IS_LOG_CALIBRATION, "[%s] Failed to connect to calibration DB at %s", ISDevice::getIdAsString(devInfo).c_str(), deviceUrl.c_str());
        return listResp;
    }

    if (listResp.statusCode == 404)
    {
        log_warn(IS_LOG_CALIBRATION, "[%s] Device not found in calibration DB (HTTP 404)", ISDevice::getIdAsString(devInfo).c_str());
        return listResp;
    }

    if (listResp.statusCode != 200)
    {
        log_error(IS_LOG_CALIBRATION, "[%s] Calibration DB returned HTTP %d: %s", ISDevice::getIdAsString(devInfo).c_str(), listResp.statusCode, listResp.statusMessage.c_str());
        return listResp;
    }

    // Step 2: Parse response to find most recent calibration UUID
    json listJson;
    try {
        listJson = json::parse(listResp.body);
    } catch (const json::parse_error& e) {
        return ISHttpRequest::Response {.statusCode = -1, .statusMessage = utils::string_format("Failed to parse calibration list JSON: %s", e.what()) };
    }

    // Find most recent calibration by calDateTime
    std::string bestUuid;
    std::string bestDateTime;

    // API returns { "<deviceUuid>": [ {calUuid, calDateTime, ...}, ... ] }
    // Iterate all values in the top-level object to find calibration entries
    if (listJson.is_object())
    {
        for (auto& [devUuid, calArray] : listJson.items())
        {
            if (!calArray.is_array())
                continue;

            for (const auto& cal : calArray)
            {
                std::string calUuid;
                if (cal.contains("calUuid"))
                    calUuid = cal["calUuid"].get<std::string>();
                else if (cal.contains("uuid"))
                    calUuid = cal["uuid"].get<std::string>();
                else
                    continue;

                std::string dateTime;
                if (cal.contains("calDateTime"))
                    dateTime = cal["calDateTime"].get<std::string>();

                if (bestUuid.empty() || dateTime > bestDateTime)
                {
                    bestUuid = calUuid;
                    bestDateTime = dateTime;
                }
            }
        }
    }
    else if (listJson.is_array())
    {
        // Fallback: top-level array of calibration entries
        for (const auto& cal : listJson)
        {
            std::string calUuid;
            if (cal.contains("calUuid"))
                calUuid = cal["calUuid"].get<std::string>();
            else if (cal.contains("uuid"))
                calUuid = cal["uuid"].get<std::string>();
            else
                continue;

            std::string dateTime;
            if (cal.contains("calDateTime"))
                dateTime = cal["calDateTime"].get<std::string>();

            if (bestUuid.empty() || dateTime > bestDateTime)
            {
                bestUuid = calUuid;
                bestDateTime = dateTime;
            }
        }
    }

    if (bestUuid.empty())
    {
        return ISHttpRequest::Response {.statusCode = 404, .statusMessage = "Received valid JSON response from server, but no suitable calibration found (Has this device ever been calibrated?)."};
    }

    log_info(IS_LOG_CALIBRATION, "Found calibration UUID: %s (date: %s)", bestUuid.c_str(), bestDateTime.c_str());

    // Step 3: Fetch full calibration JSON
    std::string calUrl = restBaseUrl + "/api/calibration/" + bestUuid;
    ISHttpRequest::Response calResp = ISHttpRequest::get(calUrl);

    if (calResp.statusCode == -1)
    {
        std::string msg = utils::string_format("Failed to fetch calibration data from %s", calUrl.c_str());
        log_error(IS_LOG_CALIBRATION, "%s", msg.c_str());
        return ISHttpRequest::Response {.statusCode = 404, .statusMessage = msg };
    }

    if (calResp.statusCode != 200)
    {
        std::string msg = utils::string_format("Calibration DB returned HTTP %d for UUID %s: %s", calResp.statusCode, bestUuid.c_str(), calResp.statusMessage.c_str());
        log_error(IS_LOG_CALIBRATION, "%s", msg.c_str());
        return ISHttpRequest::Response {.statusCode = calResp.statusCode, .statusMessage = msg };
    }

    // Step 4: Parse and upload calibration
    json calJson;
    try {
        calJson = json::parse(calResp.body);
    } catch (const json::parse_error& e) {
        return ISHttpRequest::Response {.statusCode = -1, .statusMessage = utils::string_format("Failed to parse calibration data from %s", calUrl.c_str()) };
    }

    if (!loadCalibrationFromJsonObj(calJson,  &ocal, &info, &data.dinfo, &data.tcal, &data.mcal, &pose))
    {
        return ISHttpRequest::Response {.statusCode = -1, .statusMessage = utils::string_format("[%s] Failed to parse calibration from DB", ISDevice::getIdAsString(devInfo).c_str()) };
    }

    std::string msg = utils::string_format("[%s] Successfully parsed calibration from DB (UUID: %s)", ISDevice::getIdAsString(devInfo).c_str(), bestUuid.c_str());
    log_info(IS_LOG_CALIBRATION, "%s", msg.c_str());
    return ISHttpRequest::Response {.statusCode = 200, .statusMessage = msg };
}
