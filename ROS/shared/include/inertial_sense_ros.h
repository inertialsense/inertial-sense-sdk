/***************************************************************************************
 *
 * @Copyright 2023, Inertial Sense Inc. <devteam@inertialsense.com>
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ***************************************************************************************/

#pragma once
#ifndef INERTIAL_SENSE_ROS_H
#define INERTIAL_SENSE_ROS_H

#include <stdio.h>
#include <iostream>
#include <algorithm>
#include <string>
#include <cstdlib>
#include <yaml-cpp/yaml.h>

#include "data_sets.h"
#include "InertialSense.h"

#include "RtkBase.h"
#include "RtkRover.h"
#include "TopicHelper.h"

#ifdef ROS2
#include <chrono>
#include <memory>
#include "rclcpp/rclcpp/rclcpp.hpp"
#include "rclcpp/rclcpp/timer.hpp"
#include "rclcpp/rclcpp/time.hpp"
#include "rclcpp/rclcpp/publisher.hpp"
#include "std_msgs/std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include "sensor_msgs/msg/fluid_pressure.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "inertial_sense_ros2/msg/gps.hpp"
#include "inertial_sense_ros2/msg/gps_info.hpp"
#include "inertial_sense_ros2/msg/pimu.hpp"
#include "inertial_sense_ros2/srv/firmware_update.hpp"
#include "inertial_sense_ros2/srv/ref_lla_update.hpp"
#include "inertial_sense_ros2/msg/rtk_rel.hpp"
#include "inertial_sense_ros2/msg/rtk_info.hpp"
#include "inertial_sense_ros2/msg/gnss_ephemeris.hpp"
#include "inertial_sense_ros2/msg/glonass_ephemeris.hpp"
#include "inertial_sense_ros2/msg/gnss_observation.hpp"
#include "inertial_sense_ros2/msg/gnss_obs_vec.hpp"
#include "inertial_sense_ros2/msg/inl2_states.hpp"
#include "inertial_sense_ros2/msg/didins2.hpp"
#include "inertial_sense_ros2/msg/didins1.hpp"
#include "inertial_sense_ros2/msg/didins4.hpp"
#include "nav_msgs/nav_msgs/msg/odometry.hpp"
#include "std_srvs/std_srvs/srv/trigger.hpp"
#include "std_msgs/std_msgs/msg/header.hpp"
#include "geometry_msgs/geometry_msgs/msg/vector3_stamped.hpp"
#include "geometry_msgs/geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "diagnostic_msgs/diagnostic_msgs/msg/diagnostic_array.hpp"
using namespace rclcpp;
using namespace inertial_sense_ros2;
#endif

#ifdef ROS1
#include "ros/ros.h"
#include "ros/timer.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/MagneticField.h"
#include "sensor_msgs/FluidPressure.h"
#include "sensor_msgs/JointState.h"
#include "sensor_msgs/NavSatFix.h"
#include "inertial_sense_ros/GPS.h"
#include "inertial_sense_ros/GPSInfo.h"
#include "inertial_sense_ros/PIMU.h"
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
using namespace ros;
using namespace inertial_sense_ros;
#endif

using namespace std::chrono_literals;
#define GPS_UNIX_OFFSET 315964800 // GPS time started on 6/1/1980 while UNIX time started 1/1/1970 this is the difference between those in seconds
#define LEAP_SECONDS 18           // GPS time does not have leap seconds, UNIX does (as of 1/1/2017 - next one is probably in 2020 sometime unless there is some crazy earthquake or nuclear blast)
#define UNIX_TO_GPS_OFFSET (GPS_UNIX_OFFSET - LEAP_SECONDS)
#define REPO_VERSION_MAJOR 2
#define REPO_VERSION_MINOR 5   // [UPDATE_RELEASE_VERSION_HERE] The repo/firmware version should originate from git tag (like repositoryInfo.h used in EvalTool).  For now we set these manually.
#define REPO_VERSION_REVIS 0

#define SET_CALLBACK(DID, __type, __cb_fun, __periodmultiple)                               \
    IS_.BroadcastBinaryData((DID), (__periodmultiple),                                      \
                            [this](InertialSense *i, p_data_t *data, int pHandle)           \
                            {                                                               \
                              /* RCLCPP_INFO(rclcpp::get_logger("got_message"),"Got message %d", DID);      */                     \
                                this->__cb_fun(DID, reinterpret_cast<__type *>(data->ptr)); \
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

   ~InertialSenseROS() { terminate(); }

    void initializeIS(bool configFlashParameters = true);
    void initializeROS();
    void initialize(bool configFlashParameters = true);
    void terminate();

    void callback(p_data_t *data);
    void update();

    void load_params(YAML::Node &node);
    bool connect(float timeout = 10.f);
    bool firmware_compatiblity_check();
    void set_navigation_dt_ms();
    void configure_flash_parameters();
    void configure_rtk();
    void connect_rtk_client(RtkRoverCorrectionProvider_Ntrip& config);
    void start_rtk_server(RtkBaseCorrectionProvider_Ntrip& config);

    void configure_data_streams(bool firstrun);
#ifdef ROS2
    void configure_data_streams();
#endif
#ifdef ROS1
  void configure_data_streams(const ros::TimerEvent& event);
#endif
    void configure_ascii_output();
    void start_log();

    void get_flash_config();
    void reset_device();
    void flash_config_callback(eDataIDs DID, const nvm_flash_cfg_t *const msg);
    void setRefLla(const double refLla[3]);

    bool flashConfigStreaming_ = false;
    bool factory_reset_ = false;        // Apply factory reset on startup

    // Serial Port Configuration
    std::vector<std::string> ports_;    // a collection of ports which will be attempted, in order until a connection is made
    std::string port_;                  // the actual port we connected with
    int baudrate_;                      // the baudrate to connect with

    bool sdk_connected_ = false;
    bool log_enabled_ = false;
    bool covariance_enabled_;
    int platformConfig_ = 0;
    bool setPlatformConfig_ = false;

    std::string frame_id_;

#ifdef ROS1
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
#endif

    Time last_obs_time_1_;
    Time last_obs_time_2_;
    Time last_obs_time_base_;
#ifdef ROS2
    Publisher<std_msgs::msg::String>::SharedPtr did_ins_1_pub_;
    Publisher<std_msgs::msg::String>::SharedPtr did_ins_2_pub_;
    Publisher<std_msgs::msg::String>::SharedPtr odom_ins_ned_pub_;
    Publisher<std_msgs::msg::String>::SharedPtr odom_ins_ecef_pub_;
    Publisher<std_msgs::msg::String>::SharedPtr odom_ins_enu_pub_;
    Publisher<std_msgs::msg::Header>::SharedPtr strobe_pub_;

    TimerBase::SharedPtr obs_bundle_timer_;
    TimerBase::SharedPtr data_stream_timer_;
    TimerBase::SharedPtr diagnostics_timer_;

    msg::GNSSObsVec gps1_obs_Vec_;
    msg::GNSSObsVec gps2_obs_Vec_;
    msg::GNSSObsVec base_obs_Vec_;
#endif

#ifdef ROS1
    Publisher did_ins_1_pub_;
    Publisher did_ins_2_pub_;
    Publisher odom_ins_ned_pub_;
    Publisher odom_ins_ecef_pub_;
    Publisher odom_ins_enu_pub_;
    Publisher strobe_pub_;

    Timer obs_bundle_timer_;
    Timer data_stream_timer_;
    Timer diagnostics_timer_;

    GNSSObsVec gps1_obs_Vec_;
    GNSSObsVec gps2_obs_Vec_;
    GNSSObsVec base_obs_Vec_;
#endif
    RtkRoverProvider* RTK_rover_;
    RtkBaseProvider* RTK_base_;

    bool GNSS_Compass_ = false;

    ROS_TIMER rtk_connectivity_watchdog_timer_;
    void start_rtk_connectivity_watchdog_timer();
    void stop_rtk_connectivity_watchdog_timer();
    void rtk_connectivity_watchdog_timer_callback( ROS1_TIMEREVENT_ARG );

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
    void imu_raw_callback(eDataIDs DID, const imu_t *const msg);
    void strobe_in_time_callback(eDataIDs DID, const strobe_in_time_t *const msg);
    void diagnostics_callback( ROS1_TIMEREVENT_ARG );
    void GPS_pos_callback(eDataIDs DID, const gps_pos_t *const msg);
    void GPS_vel_callback(eDataIDs DID, const gps_vel_t *const msg);
    void GPS_raw_callback(eDataIDs DID, const gps_raw_t *const msg);
    void GPS_obs_callback(eDataIDs DID, const obsd_t *const msg, int nObs);
    void GPS_obs_bundle_timer_callback( ROS1_TIMEREVENT_ARG );
    void GPS_eph_callback(eDataIDs DID, const eph_t *const msg);
    void GPS_geph_callback(eDataIDs DID, const geph_t *const msg);
    void RTK_Misc_callback(eDataIDs DID, const gps_rtk_misc_t *const msg);
    void RTK_Rel_callback(eDataIDs DID, const gps_rtk_rel_t *const msg);

#ifdef ROS2
    Node::SharedPtr nh_;
    Node::SharedPtr nh_private_;
#endif

#ifdef ROS1
    NodeHandle nh_;
    NodeHandle nh_private_;
#endif
    struct
    {
    	ins_1_t ins1;
    } did_;

    struct
    {
        TopicHelper did_ins1;
        TopicHelper did_ins2;
        TopicHelper did_ins4;
        TopicHelper odom_ins_ned;
        TopicHelper odom_ins_ecef;
        TopicHelper odom_ins_enu;
        TopicHelper inl2_states;

        TopicHelper imu;
        TopicHelper imu_raw;
        TopicHelper pimu;
        TopicHelper magnetometer;
        TopicHelper barometer;
        TopicHelper strobe_in;

        TopicHelperGps gps1;
        TopicHelperGps gps2;
        TopicHelper gps1_navsatfix;
        TopicHelper gps2_navsatfix;
        TopicHelper gps1_info;
        TopicHelper gps2_info;
        TopicHelperGpsRaw gps1_raw;
        TopicHelperGpsRaw gps2_raw;
        TopicHelperGpsRaw gpsbase_raw;
        TopicHelperGpsRtk rtk_pos;
        TopicHelperGpsRtk rtk_cmp;

        TopicHelper diagnostics;
    } rs_;

    bool NavSatFixConfigured = false;

    bool insCovarianceStreaming_ = false;
    bool imuStreaming_ = false;
    bool strobeInStreaming_ = false;
    bool diagnosticsStreaming_ = false;
    // NOTE: that GPS streaming flags are applicable for all GPS devices/receivers
    bool data_streams_enabled_ = false;

    // Services
#ifdef ROS2
   Service<std_srvs::srv::Trigger>:: SharedPtr mag_cal_srv_;
   Service<std_srvs::srv::Trigger>:: SharedPtr multi_mag_cal_srv_;
   Service<srv::FirmwareUpdate>::SharedPtr firmware_update_srv_;
   Service<std_srvs::srv::Trigger>::SharedPtr refLLA_set_current_srv_;
   Service<srv::RefLLAUpdate>::SharedPtr refLLA_set_value_srv_;

    bool set_current_position_as_refLLA(std_srvs::srv::Trigger::Request &req, std_srvs::srv::Trigger::Response &res);
    bool set_refLLA_to_value(srv::RefLLAUpdate::Request::SharedPtr req, srv::RefLLAUpdate::Response::SharedPtr res);
    bool perform_mag_cal_srv_callback(std_srvs::srv::Trigger::Request::SharedPtr req, std_srvs::srv::Trigger::Response::SharedPtr res);
    bool perform_multi_mag_cal_srv_callback(std_srvs::srv::Trigger::Request::SharedPtr req, std_srvs::srv::Trigger::Response::SharedPtr res);
    bool update_firmware_srv_callback(srv::FirmwareUpdate::Request &req, srv::FirmwareUpdate::Response &res);
#endif
#ifdef ROS1
    ServiceServer mag_cal_srv_;
    ServiceServer multi_mag_cal_srv_;
    ServiceServer firmware_update_srv_;
    ServiceServer refLLA_set_current_srv_;
    ServiceServer refLLA_set_value_srv_;
    bool set_current_position_as_refLLA(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
    bool set_refLLA_to_value(refLLAUpdate::Request &req, refLLAUpdate::Response &res);
    bool perform_mag_cal_srv_callback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
    bool perform_multi_mag_cal_srv_callback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
    bool update_firmware_srv_callback(FirmwareUpdate::Request &req, FirmwareUpdate::Response &res);
#endif

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

    Time ros_time_from_week_and_tow(const uint32_t week, const double timeOfWeek);

    /**
     * @brief ros_time_from_start_time
     * @param time - Time since boot up in seconds - Convert to GPS time of week by adding gps.towOffset
     * @return equivalent ros::Time
     */
    Time ros_time_from_start_time(const double time);

    /**
     * @brief ros_time_from_tow
     * Get equivalent ros time from tow and internal week counter
     * @param tow Time of Week (seconds)
     * @return equivalent ros::Time
     */
    Time ros_time_from_tow(const double tow);

    double tow_from_ros_time(const Time &rt);
    Time ros_time_from_gtime(const uint64_t sec, double subsec);



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

    typedef struct
    {
        uint32_t rtkPos_timeStamp = 0;
        float rtkPos_arRatio = 0;
        float rtkPos_diffAge = 0;
        std::string rtkPos_fixType = "";
        float rtkPos_hdgBaseToRov = 0;
        float rtkPos_distanceToRover = 0;

        uint32_t rtkCmp_timeStamp = 0;
        float rtkCmp_arRatio = 0;
        float rtkCmp_diffAge = 0;
        std::string rtkCmp_fixType = "";
        float rtkCmp_hdgBaseToRov = 0;
        float rtkCmp_distanceToRover = 0;

    } diagnostics_container;

    diagnostics_container diagnostics_;

    // Data to hold on to in between callbacks
    double lla_[3];
    double ecef_[3];
#ifdef ROS2
    msg::DIDINS1 msg_did_ins1;
    msg::DIDINS2 msg_did_ins2;
    msg::DIDINS4 msg_did_ins4;
    nav_msgs::msg::Odometry msg_odom_ned;
    nav_msgs::msg::Odometry msg_odom_ecef;
    nav_msgs::msg::Odometry msg_odom_enu;
    msg::INL2States msg_inl2_states;
    sensor_msgs::msg::Imu msg_imu;
    sensor_msgs::msg::Imu msg_imu_raw;
    msg::PIMU msg_pimu;
    msg::GPS msg_gps1;
    msg::GPS msg_gps2;
    sensor_msgs::msg::NavSatFix msg_NavSatFix;

    geometry_msgs::msg::Vector3Stamped gps1_velEcef;
    geometry_msgs::msg::Vector3Stamped gps2_velEcef;
    msg::GPSInfo msg_gps1_info;
    msg::GPSInfo msg_gps2_info;
#endif

#ifdef ROS1
    DID_INS1 msg_did_ins1;
    DID_INS2 msg_did_ins2;
    DID_INS4 msg_did_ins4;
    nav_msgs::Odometry msg_odom_ned;
    nav_msgs::Odometry msg_odom_ecef;
    nav_msgs::Odometry msg_odom_enu;
    INL2States msg_inl2_states;
    sensor_msgs::Imu msg_imu;
    PIMU msg_pimu;
    GPS msg_gps1;
    GPS msg_gps2;
    sensor_msgs::NavSatFix msg_NavSatFix;
    geometry_msgs::Vector3Stamped gps1_velEcef;
    geometry_msgs::Vector3Stamped gps2_velEcef;
    GPSInfo msg_gps1_info;
    GPSInfo msg_gps2_info;
#endif
    gps_pos_t gps1_pos;
    gps_pos_t gps2_pos;
    gps_vel_t gps1_vel;
    gps_vel_t gps2_vel;
    float poseCov_[36], twistCov_[36];

    // Connection to the uINS
    InertialSense IS_;

    // Flash parameters
    // navigation_dt_ms, EKF update period.  IMX-5:  16 default, 8 max.  Use `msg/ins.../period` to reduce INS output data rate.
    // navigation_dt_ms, EKF update period.  uINS-3: 4  default, 1 max.  Use `msg/ins.../period` to reduce INS output data rate.
    int ins_nav_dt_ms_;
    float insRotation_[3] = {0, 0, 0};
    float insOffset_[3] = {0, 0, 0};
    double refLla_[3] = {0, 0, 0};      // Upload disabled if all zero
    bool refLLA_valid = false;
    float magDeclination_;
    int dynamicModel_;

    uint32_t ioConfigBits_ = 0;                 // this is read directly from the config,
    bool setIoConfigBits_ = false;
    uint32_t rtkConfigBits_ = 0;                // this is read directly from the config
    uint32_t wheelConfigBits_ = 0;              // this is read directly from the config

    float gpsTimeUserDelay_ = 0;

    // EVB Flash Parameters
    struct evb_flash_parameters
    {
        int cb_preset;
        int cb_options;
    } evb_ = {};

};

#endif



