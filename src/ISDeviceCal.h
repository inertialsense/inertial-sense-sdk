/**
 * @file ISDeviceCal.h
 * @brief Device calibration load/save functions for JSON format
 *
 * NOTE: This file contains Qt-dependent code and should probably be moved to EvalTool.
 *       It is not used by the core SDK and requires Qt libraries to compile.
 *       This file is kept here for historical reasons but may be removed in the future.
 *
 * @author Inertial Sense, Inc.
 * @copyright Copyright (c) 2025 Inertial Sense, Inc. All rights reserved.
 */

#ifndef IS_SDK_ISDEVICECAL_H
#define IS_SDK_ISDEVICECAL_H

#include <string>
#include <vector>
#include "json.hpp"
#include "IS_calibration.h"
#include "ISHttpRequest.h"

#define NUM_POSES                           18
#define NUM_RATES_PER_POSE                  3       // rates per pose, each stage has a rate
#define NUM_HEADINGS_PER_POSE               8       // headings per pose, each stage has a rate
#define NUM_PQR_SAMPLES                     (NUM_POSES*NUM_RATES_PER_POSE)
#define NUM_HDG_SAMPLES                     (NUM_POSES*NUM_HEADINGS_PER_POSE)


using json = nlohmann::json;

// Convert Vector to and from String
std::string VectorToString(const float* vec, int len);
void StringToVector(const std::string& str, float* vec, int len);

// Convert matrix [m x n] to and from string
std::string MatrixToString(const float* mat, int rows, int cols);
void StringToMatrix(const std::string& str, float* mat, int rows, int cols);

struct sMatrix
{
    std::vector<float> mat;    // [m x n] matrix
    int m;                      // number of rows
    int n;                      // number of columns

    sMatrix(){}

    sMatrix(int rows, int cols): m(rows), n(cols)
    {
        mat.resize(m*n);
        zero();
    }

    void resize(int rows, int cols)
    {
        m = rows;
        n = cols;
        mat.resize(m*n);
        zero();
    }

    float *matrix()
    { 
        return &mat[0]; 
    }

    void zero()
    {
        for (size_t i=0; i<mat.size(); i++)
            mat[i] = 0;
    }

    void debug()
    {
        for (size_t i=0; i<mat.size(); i++)
            mat[i] = i+1;
    }

    void fromString(std::string str)
    {
        StringToMatrix(str, &mat[0], m, n);
    }

    std::string toString()
    {
        return MatrixToString(&mat[0], m, n);
    }
};

struct sCalData 
{
    sMatrix                 Y;          // truth data
    sMatrix                 Ahat;       // matrix we're solving for
    sMatrix                 Xhat;       // sampled/uncalibrated data
    int                     len;

    sCalData()
    {

    }

    sCalData(int size)
    {
        len = size;
        Y.resize(3, size);
        Ahat.resize(3, 4);
        Xhat.resize(4, size);
    }

    void resize(int size)
    {
        len = size;
        Y.resize(3, size);
        Ahat.resize(3, 4);
        Xhat.resize(4, size);
    }

    void zero()
    {
        Y.zero();
        Ahat.zero();
        Xhat.zero();
    }

    void debug()
    {
        Y.debug();
        Ahat.debug();
        Xhat.debug();
    }
};

struct sOrthoCal
{
    sCalData gyr[MAX_IMU_DEVICES];
    sCalData acc[MAX_IMU_DEVICES];
    sCalData mag[MAX_MAG_DEVICES];

    sOrthoCal()
    {
        for (int d = 0; d < MAX_IMU_DEVICES; d++)
        {
            gyr[d].resize(NUM_PQR_SAMPLES);
            gyr[d].zero();
            acc[d].resize(NUM_POSES);
            acc[d].zero();
        }
        for (int d = 0; d < MAX_MAG_DEVICES; d++)
        {
            mag[d].resize(NUM_HDG_SAMPLES);
            mag[d].zero();
        }
    }
};

/**
 * @brief Device calibration class for loading and saving calibration data
 * 
 * This class provides functionality to load and save device calibration data
 * from/to JSON files, including sensor calibration info, temperature compensation,
 * and motion calibration (orthonormalization).
 */
class ISDeviceCal : public sensor_cal_t
{
public:
    enum AsyncState {
        ASYNC_STATE__FAILURE     = -1,          //!< general failure state indicating an error condition, but otherwise a completed async cycle
        ASYNC_STATE__PENDING     = 0,           //!< indicates that the async operation is still in progress, and should be called again soon
        ASYNC_STATE__SUCCESS     = 1,           //!< indicates that the async operation was successful, and no further actions are necessary
    };

    /**
     * Creates an empty calibration object - this will need to be populated
     */
    explicit ISDeviceCal() : sensor_cal_t({}) {};

    /**
     * Creates a calibration object populated from parsing the provided JSON object
     * @param jObj json object containing all the calibration details
     */
    ISDeviceCal(const json& jObj) : sensor_cal_t({}) {
        loadCalibrationFromJsonObj(jObj, &ocal, &info, &data.dinfo, &data.tcal, &data.mcal, &pose);
    }


    /**
     * Creates a calibration object populated by parsing JSON contents from the specified file
     * @param filePath a path to a JSON file which contains the calibration details
     */
    ISDeviceCal(const std::string& filePath) : sensor_cal_t({}) {
        loadCalibrationFromJsonFile(filePath, &ocal, &info, &data.dinfo, &data.tcal, &data.mcal, &pose);
    }

    ISHttpRequest::Response loadFromURL(const std::string& restBaseUrl, const dev_info_t& devInfo);


    /**
     * @brief Load calibration data from a JSON file
     *
     * @param filePath Path to the JSON file to load
     * @param ocal Pointer to orthonormalization calibration data (can be NULL)
     * @param info Pointer to sensor calibration info (can be NULL)
     * @param dinfo Pointer to sensor data info (can be NULL)
     * @param tcal Pointer to temperature compensation calibration (can be NULL)
     * @param mcal Pointer to motion calibration (can be NULL)
     * @param pose Pointer to current pose value (can be NULL)
     * @return true if calibration was successfully loaded, false otherwise
     */
    static bool loadCalibrationFromJsonFile(const std::string& filePath,
                                       sOrthoCal *ocal,
                                       sensor_cal_info_t *info = NULL,
                                       sensor_data_info_t *dinfo = NULL,
                                       sensor_tcal_group_t *tcal = NULL,
                                       sensor_mcal_group_t* mcal = NULL,
                                       int* pose = NULL);

    /**
     * @brief Load calibration data from a pre-parsed JSON object
     *
     * @param jObj The parsed JSON object containing calibration data
     * @param ocal Pointer to orthonormalization calibration data (can be NULL)
     * @param info Pointer to sensor calibration info (can be NULL)
     * @param dinfo Pointer to sensor data info (can be NULL)
     * @param tcal Pointer to temperature compensation calibration (can be NULL)
     * @param mcal Pointer to motion calibration (can be NULL)
     * @param pose Pointer to current pose value (can be NULL)
     * @param filePath Optional file path used for fallback serial/date extraction (empty if not from file)
     * @return true if calibration was successfully loaded, false otherwise
     */
    static bool loadCalibrationFromJsonObj(const json& jObj,
                                       sOrthoCal *ocal,
                                       sensor_cal_info_t *info = NULL,
                                       sensor_data_info_t *dinfo = NULL,
                                       sensor_tcal_group_t *tcal = NULL,
                                       sensor_mcal_group_t* mcal = NULL,
                                       int* pose = NULL,
                                       const std::string& filePath = "");

    /**
     * @brief Load calibration data from a JSON string
     *
     * @param jsonString The JSON string to parse
     * @param ocal Pointer to orthonormalization calibration data (can be NULL)
     * @param info Pointer to sensor calibration info (can be NULL)
     * @param dinfo Pointer to sensor data info (can be NULL)
     * @param tcal Pointer to temperature compensation calibration (can be NULL)
     * @param mcal Pointer to motion calibration (can be NULL)
     * @param pose Pointer to current pose value (can be NULL)
     * @return true if calibration was successfully loaded, false otherwise
     */
    static bool loadCalibrationFromJsonString(const std::string& jsonString,
                                       sOrthoCal *ocal,
                                       sensor_cal_info_t *info = NULL,
                                       sensor_data_info_t *dinfo = NULL,
                                       sensor_tcal_group_t *tcal = NULL,
                                       sensor_mcal_group_t* mcal = NULL,
                                       int* pose = NULL);

    /**
     * @brief Save calibration data to a JSON file
     * 
     * @param filePath Path to the JSON file to save
     * @param cal Pointer to orthonormalization calibration data (can be NULL)
     * @param info Pointer to sensor calibration info (can be NULL)
     * @param tcal Pointer to temperature compensation calibration (can be NULL)
     * @param mcal Pointer to motion calibration (can be NULL)
     * @param pose Current pose value (default -1)
     * @return true if calibration was successfully saved, false otherwise
     */
    static bool saveCalibrationToJsonObj(const std::string& filePath, 
                                     sOrthoCal* cal, 
                                     sensor_cal_info_t* info = NULL, 
                                     sensor_tcal_group_t* tcal = NULL, 
                                     sensor_mcal_group_t* mcal = NULL, 
                                     int pose = -1);

    /**
     * @brief Load motion calibration from JSON object
     * 
     * @param jCal JSON object containing calibration data
     * @param pose Pointer to current pose value (can be NULL)
     * @param ocal Pointer to orthonormalization calibration data (can be NULL)
     * @param mcal Pointer to motion calibration (can be NULL)
     */
    static void loadMcFromJsonObj(const json& jCal, 
                              int *pose, 
                              sOrthoCal *ocal, 
                              sensor_mcal_group_t *mcal);

    /**
     * @brief Save motion calibration to JSON object
     * 
     * @param filePath File path (used for comment in JSON)
     * @param pose Current pose value
     * @param cal Pointer to orthonormalization calibration data (can be NULL)
     * @param mcal Pointer to motion calibration (can be NULL)
     * @return JSON object containing the motion calibration data
     */
    static json saveMcToJsonObj(const std::string& filePath, 
                                   int pose, 
                                   sOrthoCal *cal, 
                                   sensor_mcal_group_t *mcal);

    /**
     * @brief Per-upload context for the step machine.
     *
     * Holds state that must persist across step() invocations for a single device's upload,
     * but must NOT be shared across concurrent uploads to different devices. Previously held
     * in thread_local storage, which was correct under a one-task-per-worker-thread model but
     * unsafe under main-thread-driven concurrent uploads. Callers either store one of these
     * per in-flight upload (async path: ISDevice::m_calibration owns this via ISDeviceCal),
     * or stack-allocate one for the duration of a synchronous upload.
     */
    struct cal_upload_ctx_t {
        sensor_cal_v1p3_t v1p3StagingBuf = {};  //!< downgrade staging buffer for IMX-5 (v1.3) targets
        int retryCount = 0;                      //!< consecutive send failures on the current step
    };

    /**
     * @brief Uploads sensor calibration to a device in steps, with version-aware conversion.
     *
     * Resolves the on-device cal version from devInfo.hardwareVer[0] (5 -> v1.3, 6 -> v1.4)
     * and transmits a payload sized/laid out for that version. For IMX-5 targets, the in-memory
     * v1.4 calibration is downgraded to v1.3 on the host before sending. Refuses upload when the
     * hardware version is unresolved (e.g. devInfo unpopulated). (SN-7966)
     *
     * @param port Communication port handle
     * @param calUploadState State machine cursor (caller-owned, init to 0)
     * @param cal In-memory calibration (v1.4 format)
     * @param devInfo Target device info; hardwareVer[0] selects v1.3 vs v1.4 transmission
     * @param ctx Per-upload context (caller-owned; one per concurrent upload)
     * @return ASYNC_STATE__PENDING (in progress), ASYNC_STATE__SUCCESS (done), ASYNC_STATE__FAILURE (error)
     */
    static AsyncState uploadSensorCalStep(port_handle_t port, int &calUploadState, sensor_cal_t &cal, const dev_info_t &devInfo, cal_upload_ctx_t &ctx);

    static const int CAL_UPLOAD_SLEEP_MS = 150;

    cal_upload_ctx_t uploadCtx = {};    //!< per-upload state; owned by ISDeviceCal so async owners (ISDevice::m_calibration) have stable storage

protected:
    sOrthoCal ocal = {};                //!< orthonormalization calibration data
    int pose = -1;                      //!< Current pose value
};


#endif // IS_SDK_ISDEVICECAL_H
