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

/**
 * @brief Convert a float vector to a comma-separated string.
 * @param vec pointer to the first element of the vector.
 * @param len number of elements in vec.
 * @return comma-separated string representation of vec.
 */
std::string VectorToString(const float* vec, int len);

/**
 * @brief Parse a comma-separated string back into a float vector.
 * @param str comma-separated string representation, as produced by VectorToString().
 * @param vec output buffer, must have room for len elements.
 * @param len number of elements to parse into vec.
 */
void StringToVector(const std::string& str, float* vec, int len);

/**
 * @brief Convert a row-major float matrix to a string.
 * @param mat pointer to the first element of the matrix, row-major, rows x cols.
 * @param rows number of rows.
 * @param cols number of columns.
 * @return string representation of mat.
 */
std::string MatrixToString(const float* mat, int rows, int cols);

/**
 * @brief Parse a string back into a row-major float matrix.
 * @param str string representation, as produced by MatrixToString().
 * @param mat output buffer, must have room for rows*cols elements.
 * @param rows number of rows to parse into mat.
 * @param cols number of columns to parse into mat.
 */
void StringToMatrix(const std::string& str, float* mat, int rows, int cols);

/** @brief A resizable [m x n] row-major float matrix, with string (de)serialization for JSON storage. */
struct sMatrix
{
    std::vector<float> mat;    //!< [m x n] matrix, row-major
    int m;                      //!< number of rows
    int n;                      //!< number of columns

    /** @brief Default-construct an empty (unsized) matrix. */
    sMatrix(){}

    /**
     * @brief Construct and zero-initialize a matrix of the given size.
     * @param rows number of rows.
     * @param cols number of columns.
     */
    sMatrix(int rows, int cols): m(rows), n(cols)
    {
        mat.resize(m*n);
        zero();
    }

    /**
     * @brief Resize this matrix, discarding previous contents and zero-initializing.
     * @param rows new number of rows.
     * @param cols new number of columns.
     */
    void resize(int rows, int cols)
    {
        m = rows;
        n = cols;
        mat.resize(m*n);
        zero();
    }

    /**
     * @brief Get a raw pointer to the underlying row-major data.
     * @return pointer to the first element.
     */
    float *matrix()
    {
        return &mat[0];
    }

    /** @brief Set every element of this matrix to zero. */
    void zero()
    {
        for (size_t i=0; i<mat.size(); i++)
            mat[i] = 0;
    }

    /** @brief Fill this matrix with sequential debug values (1, 2, 3, ...) for testing. */
    void debug()
    {
        for (size_t i=0; i<mat.size(); i++)
            mat[i] = i+1;
    }

    /**
     * @brief Parse this matrix's contents from a string, using its current m/n dimensions.
     * @param str string representation, as produced by toString().
     */
    void fromString(std::string str)
    {
        StringToMatrix(str, &mat[0], m, n);
    }

    /**
     * @brief Serialize this matrix's contents to a string.
     * @return string representation of this matrix.
     */
    std::string toString()
    {
        return MatrixToString(&mat[0], m, n);
    }
};

/** @brief Least-squares calibration working set for one sensor: truth data (Y), the matrix being solved for (Ahat), and the sampled/uncalibrated input (Xhat). */
struct sCalData
{
    sMatrix                 Y;          //!< Truth data
    sMatrix                 Ahat;       //!< Matrix being solved for
    sMatrix                 Xhat;       //!< Sampled/uncalibrated data
    int                     len;        //!< Number of samples

    /** @brief Default-construct without allocating (call resize() before use). */
    sCalData()
    {

    }

    /**
     * @brief Construct and size the working matrices for size samples.
     * @param size number of samples.
     */
    sCalData(int size)
    {
        len = size;
        Y.resize(3, size);
        Ahat.resize(3, 4);
        Xhat.resize(4, size);
    }

    /**
     * @brief Resize the working matrices, discarding previous contents.
     * @param size number of samples.
     */
    void resize(int size)
    {
        len = size;
        Y.resize(3, size);
        Ahat.resize(3, 4);
        Xhat.resize(4, size);
    }

    /** @brief Zero all three working matrices (Y, Ahat, Xhat). */
    void zero()
    {
        Y.zero();
        Ahat.zero();
        Xhat.zero();
    }

    /** @brief Fill all three working matrices with sequential debug values, for testing. */
    void debug()
    {
        Y.debug();
        Ahat.debug();
        Xhat.debug();
    }
};

/** @brief Per-sensor motion-calibration (orthonormalization) working sets: one sCalData per gyro, accelerometer, and magnetometer. */
struct sOrthoCal
{
    sCalData gyr[MAX_IMU_DEVICES];    //!< Per-IMU gyro calibration working set
    sCalData acc[MAX_IMU_DEVICES];    //!< Per-IMU accelerometer calibration working set
    sCalData mag[MAX_MAG_DEVICES];    //!< Per-magnetometer calibration working set

    /** @brief Allocates and zeroes each sensor's working set at its expected sample count (NUM_PQR_SAMPLES for gyro, NUM_POSES for accel, NUM_HDG_SAMPLES for mag). */
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
    /** @brief Result/progress states for the step-based (non-blocking) upload and load operations below. */
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

    /**
     * @brief Fetch the most recent calibration for a device from a REST calibration database and
     * populate this object with it. Looks up the device by hardware type + serial number, finds the
     * calibration entry with the latest calDateTime, then downloads and parses that entry.
     * @param restBaseUrl Base URL of the calibration REST API (e.g. "https://cal.example.com").
     * @param devInfo Target device info; used to build the hardware-type/serial-number lookup.
     * @return the HTTP response from the final calibration fetch. statusCode == 200 on success;
     *         -1 indicates a connection/parse failure, 404 that the device has no calibration on file.
     */
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

    static const int CAL_UPLOAD_SLEEP_MS = 150;    //!< Recommended delay (ms) between successive uploadSensorCalStep() calls while an upload is in progress

    cal_upload_ctx_t uploadCtx = {};    //!< per-upload state; owned by ISDeviceCal so async owners (ISDevice::m_calibration) have stable storage

protected:
    sOrthoCal ocal = {};                //!< orthonormalization calibration data
    int pose = -1;                      //!< Current pose value
};


#endif // IS_SDK_ISDEVICECAL_H
