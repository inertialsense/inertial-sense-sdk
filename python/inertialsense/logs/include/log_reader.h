#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <algorithm>
#include <cstring>

#include "ISLogger.h"
#include "InertialSense.h"
#include "data_sets.h"
#include "luna_data_sets.h"

// #include "Eigen/Core"
// #include "Eigen/StdVector"

#ifdef WIN32
#pragma comment(lib, "SHELL32.LIB")
#endif

#define LOG_READER_DEBUG_ENABLED    0       // Set to 1 to enable debug print statements in LogReader, or 0 to disable them. These statements can be helpful for development and troubleshooting, but may produce a large amount of output for long logs.

namespace py = pybind11;

typedef struct {
    std::vector<std::vector<obsd_t>> obs;
    std::vector<eph_t> eph;
    std::vector<geph_t> gloEph;
    std::vector<sbsmsg_t> sbas;
    std::vector<sta_t> sta;
    std::vector<ion_model_utc_alm_t> ion;
} gps_raw_wrapper_t;

struct DeviceLog {
    std::vector<dev_info_t> devInfo;
    std::vector<system_fault_t> sysFault;
    std::vector<system_fault_t> gpxSysFault;
    std::vector<pimu_t> pimu;
    std::vector<ins_1_t> ins1;
    std::vector<ins_2_t> ins2;
    std::vector<gps_pos_t> gps1UbxPos;
    std::vector<system_command_t> sysCmd;
    //    std::vector<nmea_msgs_t> nmeaBcastPeriod;
    //    std::vector<rmc_t> rmc;
    std::vector<sys_params_t> sysParams;
    std::vector<sys_sensors_t> sysSensors;
    std::vector<nvm_flash_cfg_t> flashCfg;
    std::vector<gps_pos_t> gps1Pos;
    std::vector<gps_pos_t> gps2Pos;
    std::vector<gps_sat_t> gps1Sat;
    std::vector<gps_sat_t> gps2Sat;
    std::vector<gps_version_t> gps1Version;
    std::vector<gps_version_t> gps2Version;
    std::vector<mag_cal_t> magCal;
    std::vector<gps_rtk_rel_t> gps1RtkPosRel;
    std::vector<gps_rtk_rel_t> gps1RtkCmpRel;
    std::vector<gps_rtk_misc_t> gps1RtkPosMisc;
    std::vector<gps_rtk_misc_t> gps1RtkCmpMisc;
    // std::vector<feature_bits_t> featureBits;
    std::vector<sensors_w_temp_t> sensorsUcal;
    std::vector<sensors_w_temp_t> sensorsTcal;
    std::vector<sensors_w_temp_t> sensorsMcal;
    std::vector<sensors_t> sensorsTcBias;

    // std::vector<sys_sensors_adc_t> sensorsAdc;
    std::vector<sensor_compensation_t> scomp;
    std::vector<imu_t> refImu;
    std::vector<pimu_t> refPImu;
    std::vector<magnetometer_t> refMag;
    std::vector<gps_vel_t> gps1Vel;
    std::vector<gps_vel_t> gps2Vel;
    // std::vector<hdw_params_t> hdwParams;
    // std::vector<nvr_manage_t> nvrManageUserpage;
    // std::vector<nvm_group_sn_t> nvrUserpageSn;
    // std::vector<nvm_group_0_t> nvrUserpageG0;
    // std::vector<nvm_group_1_t> nvrUserpageG1;
    // std::vector<rtos_info_t> rtosInfo;
    std::vector<debug_string_t> debugString;
    std::vector<debug_array_t> debugArray;
    // std::vector<sensor_cal_v1p2_t> calSc;
    // std::vector<sensor_cal_mpu_t> calSc1;
    // std::vector<sensor_cal_mpu_t> calSc2;
    std::vector<sys_sensors_adc_t> sensorsAdcSigma;
    std::vector<inl2_states_t> inl2States;
    std::vector<inl2_status_t> inl2Status;
    // std::vector<inl2_misc_t> inl2Misc;
    std::vector<magnetometer_t> magnetometer;
    std::vector<barometer_t> barometer;
    std::vector<gps_pos_t> gps1RtkPos;
    std::vector<imus_t> imusUncal;
    std::vector<imus_t> imusRaw;
    std::vector<imu_t> imuRaw;
    std::vector<imu_t> imu;
    std::vector<imus_t> imus;
    std::vector<inl2_mag_obs_info_t> inl2MagObsInfo;
    std::vector<gps_raw_wrapper_t> gpsBaseRaw{1};
    //    std::vector<gps_rtk_opt_t> gpsRtkOpt;
    std::vector<manufacturing_info_t> manufacturingInfo;
    std::vector<bit_t> bit;
    std::vector<ins_3_t> ins3;
    std::vector<ins_4_t> ins4;
    std::vector<inl2_ned_sigma_t> inl2NedSigma;
    std::vector<strobe_in_time_t> strobeInTime;
    std::vector<gps_raw_wrapper_t> gps1Raw{1};
    std::vector<gps_raw_wrapper_t> gps2Raw{1};
    std::vector<wheel_encoder_t> wheelEncoder;
    std::vector<ground_vehicle_t> groundVehicle;
    std::vector<evb_luna_velocity_control_t> evbVelocityControl;
    std::vector<diag_msg_t> diagnosticMessage;
    std::vector<survey_in_t> surveyIn;
    //    std::vector<evb2_t> evb2;
    // std::vector<rtk_state_t> rtkState;
    std::vector<rtk_residual_t> rtkCodeResidual;
    std::vector<rtk_residual_t> rtkPhaseResidual;
    std::vector<rtk_debug_t> rtkDebug;
    // std::vector<rtk_debug_2_t> rtkDebug2;
    std::vector<port_monitor_t> portMonitor;

    std::vector<dev_info_t> gpxDevInfo;
    std::vector<gpx_flash_cfg_t> gpxFlashCfg;
    std::vector<gpx_status_t> gpxStatus;
    std::vector<gpx_rtos_info_t> gpxRtosInfo;
    std::vector<gpx_bit_t> gpxBit;
    std::vector<port_monitor_t> gpxPortMonitor;
    std::vector<debug_array_t> gpxDebugArray;
};

template <typename T>
struct DataLog {
    int id;
    std::vector<T> data;

    DataLog(int _id) { id = _id; }
};

class LogReader {
   public:
    LogReader();
    ~LogReader();
    bool init(py::object python_class, std::string log_directory, pybind11::list serials);
    bool load();
    pybind11::list serialNumbers() { return serialNumbers_; }
    pybind11::list protocolVersion();
    void ins1ToIns2(int device_id = 0);
    void exitHack(int exit_code = 0);
    // Perform final cleanup of internal resources associated with this LogReader.
    // After calling this function, the LogReader instance must not be used for any
    // callback-driven operations or further log-processing calls. It is intended
    // to be called once at the end of the LogReader's lifetime.
    void cleanup();

    int numImuDevices_ = MAX_IMU_DEVICES;

    template <typename T>
    void forward_message(eDataIDs did, std::vector<T>& vec, int device_id);

    template <typename T>
    void log_message(int did, const uint8_t* msg, uint32_t size, std::vector<T>& vec) {
        T tmp{};
        memcpy(&tmp, msg, std::min(sizeof(T), (size_t)size));
        vec.push_back(tmp);
#if LOG_READER_DEBUG_ENABLED
        printf("Logged message with DID %d, size %d, vector now has %zu entries\n", did, size, vec.size());
#endif
    }

   private:
    void organizeData(std::shared_ptr<cDeviceLog>);
    void forwardData(int device_id);
    void logReaderDebugPrint(const std::string& message);

    cISLogger logger_;
    DeviceLog* dev_log_ = nullptr;
    pybind11::list serialNumbers_;
    py::object python_parent_;  // Instance-specific python parent reference

};

// Template specialization for gps_raw_wrapper_t (implemented in .cpp)
template <>
void LogReader::log_message(int did, const uint8_t* msg, uint32_t size, std::vector<gps_raw_wrapper_t>& vec);
