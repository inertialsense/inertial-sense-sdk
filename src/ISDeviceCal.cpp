/**
 * @file ISDeviceCal.cpp
 * @brief Device calibration load/save functions for JSON format
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
#include "ISLogFile.h"

using namespace std;
using json = nlohmann::json;

// Macro for checking if vector is all zeros
#define VEC3_ALL_ZERO(v) ((v)[0] == 0.0f && (v)[1] == 0.0f && (v)[2] == 0.0f)

// Helper function prototypes
static std::vector<std::string> split(const std::string& s, char delimiter);
static std::string getFilename(const std::string& path);
static std::string getParentPath(const std::string& path);
static bool createDirectoryRecursive(const std::string& path);
static bool mat3x3_IsIdentity(const float* mat);

// Helper function implementations
static std::vector<std::string> split(const std::string& s, char delimiter) {
    std::vector<std::string> tokens;
    std::string token;
    std::istringstream tokenStream(s);
    while (std::getline(tokenStream, token, delimiter)) {
        tokens.push_back(token);
    }
    return tokens;
}

static std::string getFilename(const std::string& path) {
    size_t pos = path.find_last_of("/\\");
    if (pos != std::string::npos) {
        return path.substr(pos + 1);
    }
    return path;
}

static std::string getParentPath(const std::string& path) {
    size_t pos = path.find_last_of("/\\");
    if (pos != std::string::npos) {
        return path.substr(0, pos);
    }
    return ".";
}

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

std::string VectorToString(const float* vec, int len) {
    std::ostringstream oss;
    for (int i = 0; i < len; i++) {
        if (i > 0) oss << " ";
        oss << vec[i];
    }
    return oss.str();
}

void StringToVector(const std::string& str, float* vec, int len) {
    std::vector<std::string> vals = split(str, ' ');
    for (int i = 0; i < len && i < (int)vals.size(); i++) {
        vec[i] = std::stof(vals[i]);
    }
}

#define MATRIX_ROW_END_CHAR     '\n'
// #define MATRIX_ROW_END_CHAR     ';'

// Matrix/Vector string conversion stubs (these would need full implementation if sOrthoCal is used)
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

void StringToMatrix(const std::string& str, float* mat, int rows, int cols) {
    std::string normalized = str;
    for (char& ch : normalized) {   // Use either '\n' or ';' as the end of row character
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

static std::string calInfoVersionToString(const sensor_cal_info_t& info) {
    return cISLogFile::formatString("%d.%d.%d %d", info.version[0], info.version[1], info.version[2], info.version[3]);
}

static std::string calInfoDateToString(const sensor_cal_info_t& info) {
    return cISLogFile::formatString("%04d-%02d-%02d %d", info.calDate[0] + 2000, info.calDate[1], info.calDate[2], info.calDate[3]);
}

static std::string calInfoTimeToString(const sensor_cal_info_t& info) {
    return cISLogFile::formatString("%02d:%02d:%02d %d", info.calTime[0], info.calTime[1], info.calTime[2], info.calTime[3]);
}

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
                printf("TC index mismatch.\n");
                return;
            }
        }

        // Error check number of points
        if (jPts.size() > TCAL_MAX_NUM_POINTS)
        {
            printf("TC point index from file too large.\n");
            return;
        }

        dev.numPts = jPts.size();
        for (size_t p = 0; p < jPts.size(); p++)
        {
            const json& jPt = jPts[p];
            if (!jPt.contains("tmp"))
            {
                printf("TC point tmp value missing.\n");
                return;
            }
            dev.pt[p].temp = jPt["tmp"].get<float>();
            if (!jPt.contains("SS"))
            {
                printf("TC point SS array missing.\n");
                return;
            }
            const json& jSS = jPt["SS"];
            if (jSS.size() != 3)
            {
                printf("TC point SS array wrong size.\n");
                return;
            }
            for (int i = 0; i < 3; i++)
            {
                dev.pt[p].ss[i] = jSS[i].get<float>();
            }
        }
    }
}

static void JsonObjToSensor(const json& jTC, sensor_tcal_group_t& tcal)
{
    for (int d = 0; d < NUM_IMU_DEVICES; d++)
    {
        JsonObjToTcal3axis(jTC, cISLogFile::formatString("gyr%d", d + 1), tcal.gyr[d]);
        JsonObjToTcal3axis(jTC, cISLogFile::formatString("acc%d", d + 1), tcal.acc[d]);
    }
    for (int d = 0; d < NUM_MAG_DEVICES; d++)
    {
        JsonObjToTcal3axis(jTC, cISLogFile::formatString("mag%d", d + 1), tcal.mag[d]);
    }
}

static void JsonObjv1p2ToSensor(const json& jTC, int device, sensor_tcal_group_t &tcal)
{
    // Double check that indices match up
    for (size_t i = 0; i < jTC.size(); i++)
        if (jTC[i]["index"].get<int>() != (int)i)
        {
            printf("TC index mismatch.\n");
            return;
        }

    if (jTC.size() > TCAL_MAX_NUM_POINTS)
    {
        printf("TC point index from file too large.\n");
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

        //////////////////////////////////////////////////////////////////////////
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

        //////////////////////////////////////////////////////////////////////////
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

        //////////////////////////////////////////////////////////////////////////
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

bool ISDeviceCal::loadCalibrationFromJsonObj(const std::string& filePath, sOrthoCal *ocal, sensor_cal_info_t *info, sensor_data_info_t *dinfo, sensor_tcal_group_t *tcal, sensor_mcal_group_t* mcal, int* pose)
{
    // Read file
    std::ifstream jsonFile(filePath);
    if (!jsonFile.is_open())
    {
        cout << "Calibration file not found: " << filePath << endl;
        return false;
    }

    // cout << "Loading calibration: " << filePath << endl;
    printf("Loading calibration: %s\n", filePath.c_str());

    // Parse JSON
    json jObj;
    try {
        jsonFile >> jObj;
    } catch (const json::parse_error& e) {
        cout << "JSON parse error: " << e.what() << endl;
        return false;
    }
    jsonFile.close();

    if (jObj.empty())
    {   // Error in file.  Likely invalid data (i.e. Infinity or NAN).
        return false;
    }

    //////////////////////////////////////////////////////////////////////////
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
        else
        {   // Get date and time from filename
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

            // Date
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

            // Time
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

    //////////////////////////////////////////////////////////////////////////
    //  Orthonormalization
    if (ocal || mcal)
    {
        if (jObj.contains("calibration"))
        {
            loadMcFromJsonObj(jObj["calibration"], pose, ocal, mcal);
        }
    }

    //////////////////////////////////////////////////////////////////////////
    //  Temperature Compensation
    if (tcal)
    {
        if (jObj.contains("tempComp"))
        {   // New file format
            JsonObjToSensor(jObj["tempComp"], *tcal);
        }
        else
        {   // Old file format
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

static json saveSensorToJsonObj(nvm_sensor_tcal_3axis_t &sensor)
{
    json jTC = json::array();

    // Temp Comp - Gyro
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

static json saveTcToJsonObj(sensor_tcal_group_t* tcal)
{
    if (!tcal)
        return json::object();

    json jTC = json::object();

    for (int d = 0; d < NUM_IMU_DEVICES; d++)
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

    for (int d = 0; d < NUM_MAG_DEVICES; d++)
    {
        if (tcal->mag[d].numPts > 0)
        {
            jTC[cISLogFile::formatString("mag%d", d + 1)] = saveSensorToJsonObj(tcal->mag[d]);
        }
    }

    return jTC;
}

bool ISDeviceCal::saveCalibrationToJsonObj(const std::string& filePath, sOrthoCal* cal, sensor_cal_info_t* info, sensor_tcal_group_t* tcal, sensor_mcal_group_t* mcal, int pose)
{
    printf("saveCalibrationToJsonObj: %s\n", filePath.c_str());

    // Create parent directory if needed
    std::string parentPath = getParentPath(filePath);
    if (!parentPath.empty() && parentPath != ".") {
        createDirectoryRecursive(parentPath);
    }

    json jTop = json::object();

    //////////////////////////////////////////////////////////////////////////
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

    //////////////////////////////////////////////////////////////////////////
    //  Temperature Compensation
    if (tcal)
    {
        jTop["tempComp"] = saveTcToJsonObj(tcal);
    }

    //////////////////////////////////////////////////////////////////////////
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

static bool motionCalPresent(const float* orth, const float* bias)
{
    return !mat3x3_IsIdentity(orth) || !VEC3_ALL_ZERO(bias);
}

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

static json OrthoLeastSquaresDataSensorToJsonObj(const json& jObj, sCalData &sen)
{
    json result = jObj;
    result["Y"] = sen.Y.toString();
    result["Xhat"] = sen.Xhat.toString();
    result["Ahat"] = sen.Ahat.toString();

    return result;
}

static void OrthoJsonObjToSensor(const json& jObj, float* orth, float* bias)
{
    // Ortho
    if (jObj.contains("A"))
        StringToMatrix(jObj["A"].get<std::string>(), orth, 3, 3);

    // Bias
    if (jObj.contains("bias"))
        StringToVector(jObj["bias"].get<std::string>(), bias, 3);
}

static json OrthoSensorToJsonObj(json jObj, const float* orth, const float* bias)
{
    jObj["A"] = MatrixToString(orth, 3, 3);
    jObj["bias"] = VectorToString(bias, 3);

    return jObj;
}

void ISDeviceCal::loadMcFromJsonObj(const json& jCal, int *pose, sOrthoCal *ocal, sensor_mcal_group_t *mcal)
{
    // Current pose
    if (pose && jCal.contains("curPose"))
    {
        *pose = jCal["curPose"].get<int>();
    }

    for (int d = 0; d < NUM_IMU_DEVICES; d++)
    {
        // Set default identity matrix
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

    for (int d = 0; d < NUM_MAG_DEVICES; d++)
    {   // Mag Data
        // Set default identity matrix
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

json ISDeviceCal::saveMcToJsonObj(const std::string& filePath, int pose, sOrthoCal *cal, sensor_mcal_group_t *mcal)
{
    json jCal = json::object();
    jCal["comment_line1"] = cISLogFile::formatString(" Sensor Calibration Data - %s ", filePath.c_str());
    jCal["comment_line2"] = " Y = A * X.  Y = truth data, X = adc measured data, A = calibration data. ";

    // Add last pose saved to xml.  If we crash, we can restart from here.
    if (pose != -1)
    {
        jCal["curPose"] = pose;
    }

    for (int d = 0; d < NUM_IMU_DEVICES; d++)
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

    for (int d = 0; d < NUM_MAG_DEVICES; d++)
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
        if (comManagerSendData(port, cal.data.tcal.gyr, DID_CAL_TEMP_COMP, NUM_IMU_DEVICES * sizeof(nvm_sensor_tcal_3axis_t), offsetof(sensor_tcal_group_t, gyr)) != 0) { return 0; }
        break;

    case 3:     // Temp comp - Accelerometers
        if (comManagerSendData(port, cal.data.tcal.acc, DID_CAL_TEMP_COMP, NUM_IMU_DEVICES * sizeof(nvm_sensor_tcal_3axis_t), offsetof(sensor_tcal_group_t, acc)) != 0) { return 0; }
        break;

    case 4:     // Temp comp - Magnetometers
        if (comManagerSendData(port, cal.data.tcal.mag, DID_CAL_TEMP_COMP, NUM_MAG_DEVICES * sizeof(nvm_sensor_tcal_3axis_t), offsetof(sensor_tcal_group_t, mag)) != 0) { return 0; }
        break;  

    // Upload Motion Cal
    case 5:     // Motion cal - Gyros
        if (comManagerSendData(port, cal.data.mcal.pqr, DID_CAL_MOTION, NUM_IMU_DEVICES * sizeof(sensor_motion_cal_t), offsetof(sensor_mcal_group_t, pqr)) != 0) { return 0; }
        break;

    case 6:     // Motion cal - Accelerometers
        if (comManagerSendData(port, cal.data.mcal.acc, DID_CAL_MOTION, NUM_IMU_DEVICES * sizeof(sensor_motion_cal_t), offsetof(sensor_mcal_group_t, acc)) != 0) { return 0; }
        break;

    case 7:     // Motion cal - Magnetometers
        if (comManagerSendData(port, cal.data.mcal.mag, DID_CAL_MOTION, NUM_MAG_DEVICES * sizeof(sensor_motion_cal_t), offsetof(sensor_mcal_group_t, mag)) != 0) { return 0; }
        // printf("Done uploadSensorCal() - hdl: %d, serial#: %d, devSerialNum: %d\n", port, serialNum, cal.info.devSerialNum);
        return 1;    // Done
        
    default:    // Error
        return -1;
    }

    return 0;
}


