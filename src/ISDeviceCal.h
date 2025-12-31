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
#include "IS_calibration.h"
#include "json.hpp"

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
class ISDeviceCal
{
public:
    /**
     * @brief Load calibration data from a JSON file
     * 
     * @param filePath Path to the JSON file to load
     * @param ocal Pointer to orthonormalization calibration data (can be NULL)
     * @param info Pointer to sensor calibration info (can be NULL)
     * @param tcal Pointer to temperature compensation calibration (can be NULL)
     * @param mcal Pointer to motion calibration (can be NULL)
     * @param pose Pointer to current pose value (can be NULL)
     * @return true if calibration was successfully loaded, false otherwise
     */
    static bool loadCalibrationFromJsonObj(const std::string& filePath, 
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

    static int uploadSensorCalStep(port_handle_t port, int &calUploadState, sensor_cal_t &cal);

    static const int CAL_UPLOAD_SLEEP_MS = 150;

};


#endif // IS_SDK_ISDEVICECAL_H
