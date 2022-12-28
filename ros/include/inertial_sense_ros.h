#pragma once

#include <stdio.h>
#include <iostream>
#include <algorithm>
#include <string>
#include <cstdlib>
#include <yaml-cpp/yaml.h>

#include "InertialSense.h"
#include "ros/ros.h"
#include "ros/timer.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/MagneticField.h"
#include "sensor_msgs/FluidPressure.h"
#include "sensor_msgs/JointState.h"
#include "sensor_msgs/NavSatFix.h"
#include "inertial_sense_ros/GPS.h"
#include "data_sets.h"
#include "inertial_sense_ros/GPSInfo.h"
#include "inertial_sense_ros/PreIntIMU.h"
#include "inertial_sense_ros/FirmwareUpdate.h"
#include "inertial_sense_ros/refLLAUpdate.h"
#include "inertial_sense_ros/RTKRel.h"
#include "inertial_sense_ros/RTKInfo.h"
#include "inertial_sense_ros/GNSSEphemeris.h"
#include "inertial_sense_ros/GlonassEphemeris.h"
#include "inertial_sense_ros/GNSSObservation.h"
#include "inertial_sense_ros/GNSSObsVec.h"
#include "inertial_sense_ros/INL2States.h"
#include "inertial_sense_ros/DID_INS2.h"
#include "inertial_sense_ros/DID_INS1.h"
#include "inertial_sense_ros/DID_INS4.h"
#include "nav_msgs/Odometry.h"
#include "std_srvs/Trigger.h"
#include "std_msgs/Header.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "diagnostic_msgs/DiagnosticArray.h"
#include <tf/transform_broadcaster.h>
#include "ISConstants.h"
//#include "geometry/xform.h"

#define GPS_UNIX_OFFSET 315964800 // GPS time started on 6/1/1980 while UNIX time started 1/1/1970 this is the difference between those in seconds
#define LEAP_SECONDS 18           // GPS time does not have leap seconds, UNIX does (as of 1/1/2017 - next one is probably in 2020 sometime unless there is some crazy earthquake or nuclear blast)
#define UNIX_TO_GPS_OFFSET (GPS_UNIX_OFFSET - LEAP_SECONDS)
#define FIRMWARE_VERSION_CHAR0 1
#define FIRMWARE_VERSION_CHAR1 9
#define FIRMWARE_VERSION_CHAR2 0

#define SET_CALLBACK(DID, __type, __cb_fun, __periodmultiple)                          \
    IS_.BroadcastBinaryData(DID, __periodmultiple,                                     \
                            [this](InertialSense *i, p_data_t *data, int pHandle)      \
                            {                                                          \
                                /* ROS_INFO("Got message %d", DID);*/                  \
                                this->__cb_fun(DID, reinterpret_cast<__type *>(data->buf)); \
                            })

class InertialSenseROS //: SerialListener
{
public:
    typedef enum
    {
        NMEA_GPGGA = 0x01,
        NMEA_GPGLL = 0x02,
        NMEA_GPGSA = 0x04,
        NMEA_GPRMC = 0x08,
        NMEA_SER0 = 0x01,
        NMEA_SER1 = 0x02
    } NMEA_message_config_t;

    InertialSenseROS(YAML::Node paramNode = YAML::Node(YAML::NodeType::Undefined), bool configFlashParameters = true);
    void callback(p_data_t *data);
    void update();

    void load_params_srv();
    void load_params_yaml(YAML::Node node);
    template <typename Type>
    bool get_node_param_yaml(YAML::Node node, const std::string key, Type &val);
    template <typename Derived1>
    bool get_node_vector_yaml(YAML::Node node, const std::string key, int size, Derived1 &val);
    void connect();
    bool firmware_compatiblity_check();
    void set_navigation_dt_ms();
    void configure_flash_parameters();
    void configure_rtk();
    void connect_rtk_client(const std::string &RTK_correction_protocol, const std::string &RTK_server_IP, const int RTK_server_port);
    void start_rtk_server(const std::string &RTK_server_IP, const int RTK_server_port);

    void configure_data_streams(bool startup);
    void configure_data_streams(const ros::TimerEvent& event);
    void configure_ascii_output();
    void start_log();

    template <typename T>
    void get_vector_flash_config(std::string param_name, uint32_t size, T &data);
    //void set_vector_flash_config(std::string param_name, uint32_t size, uint32_t offset);
    void get_flash_config();
    void reset_device();
    void flash_config_callback(eDataIDs DID, const nvm_flash_cfg_t *const msg);
    bool flashConfigStreaming_ = false;
    // Serial Port Configuration
    std::string port_ = "/dev/ttyACM0";
    int baudrate_ = 921600;
    bool initialized_;
    bool log_enabled_ = false;
    bool covariance_enabled_ = false;

    std::string frame_id_ = "body";

    // ROS Stream handling
    typedef struct
    {
        bool enabled = false;
        ros::Publisher pub;
        ros::Publisher pub2;
        ros::Publisher pub3;
        int period_multiple = 1;
    } ros_stream_t;


    tf::TransformBroadcaster br;
    bool publishTf_ = true;
    tf::Transform transform_NED;
    tf::Transform transform_ENU;
    tf::Transform transform_ECEF;
    enum
    {
        NED,
        ENU
    } ltcf;

    ros::Publisher did_ins_1_pub_;
    ros::Publisher did_ins_2_pub_;
    ros::Publisher odom_ins_ned_pub_;
    ros::Publisher odom_ins_ecef_pub_;
    ros::Publisher odom_ins_enu_pub_;
    ros::Publisher strobe_pub_;
    ros::Timer obs_bundle_timer_;
    ros::Time last_obs_time_1_;
    ros::Time last_obs_time_2_;
    ros::Time last_obs_time_base_;
    ros::Timer data_stream_timer_;
    ros::Timer diagnostics_timer_;
    inertial_sense_ros::GNSSObsVec gps1_obs_Vec_;
    inertial_sense_ros::GNSSObsVec gps2_obs_Vec_;
    inertial_sense_ros::GNSSObsVec base_obs_Vec_;

    bool rtk_connecting_ = false;
    int RTK_connection_attempt_limit_ = 1;
    int RTK_connection_attempt_backoff_ = 2;
    int rtk_traffic_total_byte_count_ = 0;
    int rtk_data_transmission_interruption_count_ = 0;
    bool rtk_connectivity_watchdog_enabled_ = true;
    float rtk_connectivity_watchdog_timer_frequency_ = 1;
    int rtk_data_transmission_interruption_limit_ = 5;
    std::string RTK_server_mount_ = "";
    std::string RTK_server_username_ = "";
    std::string RTK_server_password_ = "";
    std::string RTK_correction_protocol_ = "RTCM3";
    std::string RTK_server_IP_ = "127.0.0.1";
    int RTK_server_port_ = 7777;
    bool RTK_rover_ = false;
    bool RTK_rover_radio_enable_ = false;
    bool RTK_base_USB_ = false;
    bool RTK_base_serial_ = false;
    bool RTK_base_TCP_ = false;
    bool GNSS_Compass_ = false;

    std::string gps1_type_ = "F9P";
    std::string gps1_topic_ = "gps1";
    std::string gps2_type_ = "F9P";
    std::string gps2_topic_ = "gps2";

    ros::Timer rtk_connectivity_watchdog_timer_;
    void start_rtk_connectivity_watchdog_timer();
    void stop_rtk_connectivity_watchdog_timer();
    void rtk_connectivity_watchdog_timer_callback(const ros::TimerEvent &timer_event);

    void INS1_callback(eDataIDs DID, const ins_1_t *const msg);
    void INS2_callback(eDataIDs DID, const ins_2_t *const msg);
    void INS4_callback(eDataIDs DID, const ins_4_t *const msg);
    void INL2_states_callback(eDataIDs DID, const inl2_states_t *const msg);
    void INS_covariance_callback(eDataIDs DID, const ros_covariance_pose_twist_t *const msg);
    void odom_ins_ned_callback(eDataIDs DID, const ins_2_t *const msg);
    void odom_ins_ecef_callback(eDataIDs DID, const ins_2_t *const msg);
    void odom_ins_enu_callback(eDataIDs DID, const ins_2_t *const msg);
    void GPS_info_callback(eDataIDs DID, const gps_sat_t *const msg);
    void mag_callback(eDataIDs DID, const magnetometer_t *const msg);
    void baro_callback(eDataIDs DID, const barometer_t *const msg);
    void preint_IMU_callback(eDataIDs DID, const pimu_t *const msg);
    void strobe_in_time_callback(eDataIDs DID, const strobe_in_time_t *const msg);
    void diagnostics_callback(const ros::TimerEvent &event);
    void GPS_pos_callback(eDataIDs DID, const gps_pos_t *const msg);
    void GPS_vel_callback(eDataIDs DID, const gps_vel_t *const msg);
    void GPS_raw_callback(eDataIDs DID, const gps_raw_t *const msg);
    void GPS_obs_callback(eDataIDs DID, const obsd_t *const msg, int nObs);
    void GPS_obs_bundle_timer_callback(const ros::TimerEvent &e);
    void GPS_eph_callback(eDataIDs DID, const eph_t *const msg);
    void GPS_geph_callback(eDataIDs DID, const geph_t *const msg);
    void RTK_Misc_callback(eDataIDs DID, const gps_rtk_misc_t *const msg);
    void RTK_Rel_callback(eDataIDs DID, const gps_rtk_rel_t *const msg);

    float diagnostic_ar_ratio_, diagnostic_differential_age_, diagnostic_heading_base_to_rover_;
    uint diagnostic_fix_type_;

    ros_stream_t DID_INS_1_;
    ros_stream_t DID_INS_2_;
    ros_stream_t DID_INS_4_;
    ros_stream_t INL2_states_;
    ros_stream_t odom_ins_ned_;
    ros_stream_t odom_ins_ecef_;
    ros_stream_t odom_ins_enu_;
    ros_stream_t IMU_;
    ros_stream_t mag_;
    ros_stream_t baro_;
    ros_stream_t preint_IMU_;
    ros_stream_t diagnostics_;
    ros_stream_t GPS1_;
    ros_stream_t GPS1_info_;
    ros_stream_t GPS1_raw_;
    ros_stream_t GPS2_;
    ros_stream_t GPS2_info_;
    ros_stream_t GPS2_raw_;
    ros_stream_t GPS_base_raw_;
    ros_stream_t RTK_pos_;
    ros_stream_t RTK_cmp_;
    ros_stream_t NavSatFix_;
    bool NavSatFixConfigured = false;
    int gps_raw_period_multiple = 1;
    int gps_info_period_multiple = 1;

    bool ins1Streaming_ = false;
    bool ins2Streaming_ = false;
    bool ins4Streaming_ = false;
    bool inl2StatesStreaming_ = false;
    bool insCovarianceStreaming_ = false;
    bool magStreaming_ = false;
    bool baroStreaming_ = false;
    bool preintImuStreaming_ = false;
    bool imuStreaming_ = false;
    bool strobeInStreaming_ = false;
    bool diagnosticsStreaming_ = false;
    // NOTE: that GPS streaming flags are applicable for all GPS devices/receivers
    bool gps1PosStreaming_ = false;
    bool gps1VelStreaming_ = false;
    bool gps2PosStreaming_ = false;
    bool gps2VelStreaming_ = false;
    bool gps1RawStreaming_ = false;
    bool gps2RawStreaming_ = false;
    bool gps1InfoStreaming_ = false;
    bool gps2InfoStreaming_ = false;
    bool rtkPosMiscStreaming_ = false;
    bool rtkPosRelStreaming_ = false;
    bool rtkCmpMiscStreaming_ = false;
    bool rtkCmpRelStreaming_ = false;
    bool data_streams_enabled_ = false;

    // Services
    ros::ServiceServer mag_cal_srv_;
    ros::ServiceServer multi_mag_cal_srv_;
    ros::ServiceServer firmware_update_srv_;
    ros::ServiceServer refLLA_set_current_srv_;
    ros::ServiceServer refLLA_set_value_srv_;
    bool set_current_position_as_refLLA(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
    bool set_refLLA_to_value(inertial_sense_ros::refLLAUpdate::Request &req, inertial_sense_ros::refLLAUpdate::Response &res);
    bool perform_mag_cal_srv_callback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
    bool perform_multi_mag_cal_srv_callback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
    bool update_firmware_srv_callback(inertial_sense_ros::FirmwareUpdate::Request &req, inertial_sense_ros::FirmwareUpdate::Response &res);

    void publishGPS1();
    void publishGPS2();

    enum PositionCovarianceType
    {
        COVARIANCE_TYPE_UNKNOWN = 0,
        COVARIANCE_TYPE_APPROXIMATED = 1,
        COVARIANCE_TYPE_DIAGONAL_KNOWN = 2,
        COVARIANCE_TYPE_KNOWN = 3
    };

    enum NavSatFixStatusFixType
    {
        STATUS_NO_FIX = -1,  // unable to fix position
        STATUS_FIX = 0,      // unaugmented fix
        STATUS_SBAS_FIX = 1, // with satellite-based augmentation
        STATUS_GBAS_FIX = 2  // with ground-based augmentation
    };

    // Bits defining which Global Navigation Satellite System signals were used by the receiver.
    enum NavSatFixService
    {
        SERVICE_GPS = 0x1,
        SERVICE_GLONASS = 0x2,
        SERVICE_COMPASS = 0x4, // includes BeiDou.
        SERVICE_GALILEO = 0x8
    };

    /**
     * @brief ros_time_from_week_and_tow
     * Get current ROS time from week and tow
     * @param week Weeks since January 6th, 1980
     * @param timeOfWeek Time of week (since Sunday morning) in seconds, GMT
     * @return equivalent ros::Time
     */
    ros::Time ros_time_from_week_and_tow(const uint32_t week, const double timeOfWeek);

    /**
     * @brief ros_time_from_start_time
     * @param time - Time since boot up in seconds - Convert to GPS time of week by adding gps.towOffset
     * @return equivalent ros::Time
     */
    ros::Time ros_time_from_start_time(const double time);

    /**
     * @brief ros_time_from_tow
     * Get equivalent ros time from tow and internal week counter
     * @param tow Time of Week (seconds)
     * @return equivalent ros::Time
     */
    ros::Time ros_time_from_tow(const double tow);

    double tow_from_ros_time(const ros::Time &rt);
    ros::Time ros_time_from_gtime(const uint64_t sec, double subsec);

    double GPS_towOffset_ = 0; // The offset between GPS time-of-week and local time on the uINS
                               //  If this number is 0, then we have not yet got a fix
    uint64_t GPS_week_ = 0;    // Week number to start of GPS_towOffset_ in GPS time
    // Time sync variables
    double INS_local_offset_ = 0.0;  // Current estimate of the uINS start time in ROS time seconds
    bool got_first_message_ = false; // Flag to capture first uINS start time guess

    /**
     * @brief LD2Cov
     * Transform array of covariance lower diagonals itno the full covariance matrix
     * @param LD array of lower diagonals
     * @param Cov full covariance matrix
     * @param width size (width or height) of the covariance matrix
     */
    void LD2Cov(const float *LD, float *Cov, int width);

    /**
     * @brief rotMatB2R
     * Make a rotation matrix body-to-reference from quaternion
     * @param quat attitude quaternion (rotation from the reference frame to body)
     * @param R rotation matrix body-to-reference
     */
    void rotMatB2R(const ixVector4 quat, ixMatrix3 R);

    /**
     * @brief transform_6x6_covariance
     * Transform covariance matrix due to the change of coordinates, such that
     * the fisrt 3 coordinates are rotated by R1 and the last 3 coordinates are rotated by R2
     * @param Pout output covariance matrix (in the new coordinates)
     * @param Pin  input covariance matrix (in the old coordinates)
     * @param R1   rotation matrix describing transformation of the first 3 coordinates
     * @param R2   rotation matrix describing transformation of the last 3 coordinates
     */
    void transform_6x6_covariance(float Pout[36], float Pin[36], ixMatrix3 R1, ixMatrix3 R2);

    // Data to hold on to in between callbacks
    double lla_[3];
    double ecef_[3];
    sensor_msgs::Imu imu_msg;
    nav_msgs::Odometry ned_odom_msg;
    nav_msgs::Odometry ecef_odom_msg;
    nav_msgs::Odometry enu_odom_msg;
    sensor_msgs::NavSatFix NavSatFix_msg;
    inertial_sense_ros::GPS gps1_msg;
    geometry_msgs::Vector3Stamped gps1_velEcef;
    float gps1_sAcc;
    float gps2_sAcc;
    inertial_sense_ros::GPSInfo gps_info_msg;
    inertial_sense_ros::GPS gps2_msg;
    geometry_msgs::Vector3Stamped gps2_velEcef;
    inertial_sense_ros::GPSInfo gps2_info_msg;
    inertial_sense_ros::INL2States inl2_states_msg;
    inertial_sense_ros::DID_INS1 did_ins_1_msg;
    inertial_sense_ros::DID_INS2 did_ins_2_msg;
    inertial_sense_ros::DID_INS4 did_ins_4_msg;
    inertial_sense_ros::PreIntIMU preintIMU_msg;

    float poseCov[36], twistCov[36];

    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    // Connection to the uINS
    InertialSense IS_;

    //Flash parameters

    int navigation_dt_ms_ = 4;
    float insRotation_[3] = {0, 0, 0};
    float insOffset_[3] = {0, 0, 0};
    float gps1AntOffset_[3] = {0, 0, 0};
    float gps2AntOffset_[3] = {0, 0, 0};
    double refLla_[3] = {0, 0, 0};
    float magInclination_ = 0;
    float magDeclination_ = 0;
    int insDynModel_ = INS_DYN_MODEL_AIRBORNE_4G;
    bool refLLA_known = false;
    int ioConfig_ = 39624800; //F9P RUG2 RTK CMP: 0x025ca060
    float gpsTimeUserDelay_ = 0;

};
