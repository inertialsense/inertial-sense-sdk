#include "inertial_sense_ros.h"
#include <chrono>
#include <stddef.h>
#include <unistd.h>
#include <tf/tf.h>
#include <ros/console.h>
#include <ISPose.h>
#include <ISEarth.h>
#include <tf2/LinearMath/Quaternion.h>
#include "ISMatrix.h"
#include "ISEarth.h"

#define STREAMING_CHECK(streaming, DID)      if(!streaming){ streaming = true; ROS_INFO("%s response received", cISDataMappings::GetDataSetName(DID)); }


InertialSenseROS::InertialSenseROS(YAML::Node paramNode, bool configFlashParameters) : nh_(), nh_private_("~"), initialized_(false), rtk_connectivity_watchdog_timer_()
{
    ROS_INFO("======  Starting Inertial Sense ROS  ======");

    // Should always be enabled by default
    rs_.ins1.enabled = true;
    rs_.gps1.enabled = true;

    load_params(paramNode);

    connect();

    // Check protocol and firmware version
    firmware_compatiblity_check();

    //////////////////////////////////////////////////////////
    // Start Up ROS service servers
    refLLA_set_current_srv_         = nh_.advertiseService("set_refLLA_current", &InertialSenseROS::set_current_position_as_refLLA, this);
    refLLA_set_value_srv_           = nh_.advertiseService("set_refLLA_value", &InertialSenseROS::set_refLLA_to_value, this);
    mag_cal_srv_                    = nh_.advertiseService("single_axis_mag_cal", &InertialSenseROS::perform_mag_cal_srv_callback, this);
    multi_mag_cal_srv_              = nh_.advertiseService("multi_axis_mag_cal", &InertialSenseROS::perform_multi_mag_cal_srv_callback, this);
    //firmware_update_srv_          = nh_.advertiseService("firmware_update", &InertialSenseROS::update_firmware_srv_callback, this);
    
    SET_CALLBACK(DID_STROBE_IN_TIME, strobe_in_time_t, strobe_in_time_callback, 0); // we always want the strobe

    //////////////////////////////////////////////////////////
    // Publishers
    strobe_pub_ = nh_.advertise<std_msgs::Header>("strobe_time", 1);

    if (rs_.ins1.enabled)           { rs_.ins1.pub = nh_.advertise<inertial_sense_ros::DID_INS1>("DID_INS_1", 1); }
    if (rs_.ins2.enabled)           { rs_.ins2.pub = nh_.advertise<inertial_sense_ros::DID_INS1>("DID_INS_2", 1); }
    if (rs_.ins4.enabled)           { rs_.ins4.pub = nh_.advertise<inertial_sense_ros::DID_INS1>("DID_INS_4", 1); }
    if (rs_.odom_ins_ned.enabled)   { rs_.odom_ins_ned.pub = nh_.advertise<nav_msgs::Odometry>("odom_ins_ned", 1); }
    if (rs_.odom_ins_enu.enabled)   { rs_.odom_ins_enu.pub = nh_.advertise<nav_msgs::Odometry>("odom_ins_enu", 1); }
    if (rs_.odom_ins_ecef.enabled)  { rs_.odom_ins_ecef.pub = nh_.advertise<nav_msgs::Odometry>("odom_ins_ecef", 1); }
    if (rs_.inl2_states.enabled)    { rs_.inl2_states.pub = nh_.advertise<inertial_sense_ros::INL2States>("inl2_states", 1); }
    if (rs_.navsatfix.enabled)      { rs_.navsatfix.pub = nh_.advertise<sensor_msgs::NavSatFix>("NavSatFix", 1); }

    if (rs_.pimu.enabled)           { rs_.pimu.pub = nh_.advertise<inertial_sense_ros::PreIntIMU>("preint_imu", 1); }
    if (rs_.imu.enabled)            { rs_.imu.pub = nh_.advertise<sensor_msgs::Imu>("imu", 1); }
    if (rs_.mag.enabled)            { rs_.mag.pub = nh_.advertise<sensor_msgs::MagneticField>("mag", 1); }
    if (rs_.baro.enabled)           { rs_.baro.pub = nh_.advertise<sensor_msgs::FluidPressure>("baro", 1); }

    if (rs_.gps1.enabled)           { rs_.gps1.pub = nh_.advertise<inertial_sense_ros::GPS>(gps1_topic_, 1); }
    if (rs_.gps2.enabled)           { rs_.gps2.pub = nh_.advertise<inertial_sense_ros::GPS>(gps2_topic_, 1); }
    if (rs_.gps1_info.enabled)      { rs_.gps1_info.pub = nh_.advertise<inertial_sense_ros::GPSInfo>(gps1_topic_ + "/info", 1); }
    if (rs_.gps2_info.enabled)      { rs_.gps2_info.pub = nh_.advertise<inertial_sense_ros::GPSInfo>(gps2_topic_ + "/info", 1); }

    if (RTK_rover_ || RTK_rover_radio_enable_)
    {
        rs_.rtk_pos.pubInfo = nh_.advertise<inertial_sense_ros::RTKInfo>("RTK_pos/info", 10);
        rs_.rtk_pos.pubRel = nh_.advertise<inertial_sense_ros::RTKRel>("RTK_pos/rel", 10);
    }
    if (gnss_compass_)
    {
        rs_.rtk_cmp.pubInfo = nh_.advertise<inertial_sense_ros::RTKInfo>("RTK_cmp/info", 10);
        rs_.rtk_cmp.pubRel = nh_.advertise<inertial_sense_ros::RTKRel>("RTK_cmp/rel", 10);
    }

    if (rs_.gps1_raw.enabled)
    {
        rs_.gps1_raw.pubObs = nh_.advertise<inertial_sense_ros::GNSSObsVec>(gps1_topic_ + "/obs", 50);
        rs_.gps1_raw.pubEph = nh_.advertise<inertial_sense_ros::GNSSEphemeris>(gps1_topic_ + "/eph", 50);
        rs_.gps1_raw.pubGEp = nh_.advertise<inertial_sense_ros::GlonassEphemeris>(gps1_topic_ + "/geph", 50);
        obs_bundle_timer_ = nh_.createTimer(ros::Duration(0.001), InertialSenseROS::GPS_obs_bundle_timer_callback, this);
    }
    if (rs_.gps2_raw.enabled)
    {
        rs_.gps2_raw.pubObs = nh_.advertise<inertial_sense_ros::GNSSObsVec>(gps2_topic_ + "/obs", 50);
        rs_.gps2_raw.pubEph = nh_.advertise<inertial_sense_ros::GNSSEphemeris>(gps2_topic_ + "/eph", 50);
        rs_.gps2_raw.pubGEp = nh_.advertise<inertial_sense_ros::GlonassEphemeris>(gps2_topic_ + "/geph", 50);
        obs_bundle_timer_ = nh_.createTimer(ros::Duration(0.001), InertialSenseROS::GPS_obs_bundle_timer_callback, this);
    }
    if (rs_.gpsbase_raw.enabled)
    {
        rs_.gpsbase_raw.pubObs = nh_.advertise<inertial_sense_ros::GlonassEphemeris>(gps1_topic_ + "/base_geph", 50);
        rs_.gpsbase_raw.pubEph = nh_.advertise<inertial_sense_ros::GNSSEphemeris>(gps1_topic_ + "/base_eph", 50);
        rs_.gpsbase_raw.pubGEp = nh_.advertise<inertial_sense_ros::GlonassEphemeris>(gps1_topic_ + "/base_geph", 50);
        obs_bundle_timer_ = nh_.createTimer(ros::Duration(0.001), InertialSenseROS::GPS_obs_bundle_timer_callback, this);
    }
    if (rs_.diagnostics.enabled)
    {
        rs_.diagnostics.pub = nh_.advertise<diagnostic_msgs::DiagnosticArray>("diagnostics", 1);
        diagnostics_timer_ = nh_.createTimer(ros::Duration(0.5), &InertialSenseROS::diagnostics_callback, this); // 2 Hz
    }

    data_stream_timer_ = nh_.createTimer(ros::Duration(1), configure_data_streams, this);

    IS_.StopBroadcasts(true);
    configure_data_streams(true);
    configure_rtk();
    IS_.SavePersistent();

    if (configFlashParameters)
    {   // Set IMX flash parameters (flash write) after everything else so processor stall doesn't interfere with communications.
        configure_flash_parameters();
    }

    if (log_enabled_)
    {
        start_log();    // Start log should happen last
    }

    // configure_ascii_output(); // Currently not functional

    initialized_ = true;
}

#define GET_PARAM(name, var) \
    if (useYamlNode){ \
        get_node_param_yaml(node, name, var); \
    } else { \
        getParam(name, var); \
    }

#define GET_PARAM_VEC(name, size, vec) \
    if (useYamlNode){ \
        get_node_vector_yaml(node, name, size, vec); \
    } else { \
        getParamVector(name, size, vec); \
    }

#define GET_PARAMS_RS(name) \
    if (useYamlNode){ \
        get_node_param_yaml(node, "msg/"#name"/enable", rs_.name.enabled); \
        get_node_param_yaml(node, "msg/"#name"/period", rs_.name.period); \
    } else { \
        getParam("/msg/"#name"/enable", rs_.name.enabled); \
        getParam("/msg/"#name"/period", rs_.name.period); \
    }

void InertialSenseROS::load_params(YAML::Node &node)
{
    bool useYamlNode = node.IsDefined();

    ROS_INFO( (useYamlNode ? "Load from YAML node" : "Load from Param Server") );

    GET_PARAM("port", port_);
    GET_PARAM("baudrate", baudrate_);
    GET_PARAM("frame_id", frame_id_);

    GET_PARAMS_RS(ins1);
    GET_PARAMS_RS(ins2);
    GET_PARAMS_RS(ins4);
    GET_PARAMS_RS(odom_ins_ned);
    GET_PARAMS_RS(odom_ins_enu);
    GET_PARAMS_RS(odom_ins_ecef);
    GET_PARAMS_RS(inl2_states);
    GET_PARAM("stream_ins_covariance", covariance_enabled_);

    GET_PARAMS_RS(imu);
    GET_PARAMS_RS(pimu);
    GET_PARAMS_RS(mag);
    GET_PARAMS_RS(baro);

    GET_PARAMS_RS(gps1);
    GET_PARAMS_RS(gps2);
    GET_PARAM("msg/gps1/topic", gps1_topic_);
    GET_PARAM("msg/gps2/topic", gps2_topic_);
    GET_PARAMS_RS(navsatfix);
    GET_PARAMS_RS(gps1_raw);
    GET_PARAMS_RS(gps2_raw);
    GET_PARAMS_RS(gpsbase_raw);
    GET_PARAMS_RS(gps1_info);
    GET_PARAMS_RS(gps2_info);
    GET_PARAMS_RS(rtk_pos);
    GET_PARAMS_RS(rtk_cmp);

    GET_PARAMS_RS(diagnostics);

    GET_PARAM("publishTf", publishTf_);
    GET_PARAM("enable_log", log_enabled_);
    GET_PARAM("rtk_server_mount", RTK_server_mount_);
    GET_PARAM("rtk_server_username", RTK_server_username_);
    GET_PARAM("rtk_server_password", RTK_server_password_);
    GET_PARAM("rtk_connection_attempt_limit", RTK_connection_attempt_limit_);
    GET_PARAM("rtk_connection_attempt_backoff", RTK_connection_attempt_backoff_);
    GET_PARAM("rtk_connectivity_watchdog_enabled", rtk_connectivity_watchdog_enabled_);
    GET_PARAM("rtk_connectivity_watchdog_timer_frequency", rtk_connectivity_watchdog_timer_frequency_);
    GET_PARAM("rtk_data_transmission_interruption_limit", rtk_data_transmission_interruption_limit_);
    GET_PARAM("rtk_correction_protocol", RTK_correction_protocol_);
    GET_PARAM("rtk_server_IP", RTK_server_IP_);
    GET_PARAM("rtk_server_port", RTK_server_port_);
    GET_PARAM("gnss_compass", gnss_compass_);
    GET_PARAM("rtk_compass", RTK_rover_);
    GET_PARAM("rtk_rover_radio_enable", RTK_rover_radio_enable_);
    GET_PARAM("rtk_base_USB", RTK_base_USB_);
    GET_PARAM("rtk_base_serial", RTK_base_serial_);
    GET_PARAM("rtk_base_TCP", RTK_base_TCP_);

    GET_PARAM("navigation_dt_ms", navigation_dt_ms_);   // EKF update period
    GET_PARAM("ioConfig", ioConfig_);
    GET_PARAM("gps1_type", gps1_type_);
    GET_PARAM("gps2_type", gps2_type_);
    GET_PARAM("gpsTimeUserDelay", gpsTimeUserDelay_);
    GET_PARAM("mag_declination", magDeclination_);
    GET_PARAM("dynamic_model", insDynModel_);
    GET_PARAM_VEC("ins_rotation", 3, insRotation_);
    GET_PARAM_VEC("ins_offset", 3, insOffset_);
    GET_PARAM_VEC("gps1_ant_xyz", 3, gps1AntOffset_);
    GET_PARAM_VEC("gps2_ant_xyz", 3, gps2AntOffset_);
    GET_PARAM_VEC("ref_lla", 3, refLla_);
}


void InertialSenseROS::configure_data_streams(const ros::TimerEvent& event)
{
    configure_data_streams(false);
}

#define CONFIG_STREAM(stream, did, type, cb_fun) \
    if((stream.enabled) && !(stream.streaming)){ \
        ROS_INFO("Attempting to enable %s data stream.", cISDataMappings::GetDataSetName(did)); \
        SET_CALLBACK(did, type, cb_fun, stream.period); \
        if (!firstrun) \
            return; \
    }

#define CONFIG_STREAM_GPS(stream, did_pos, cb_fun_pos, did_vel, cb_fun_vel) \
    if((stream.enabled) && !(stream.streaming_pos)){ \
        ROS_INFO("Attempting to enable %s data stream.", cISDataMappings::GetDataSetName(did_pos)); \
        SET_CALLBACK(did_pos, gps_pos_t, cb_fun_pos, stream.period); \
        if (!firstrun) \
            return; \
    } \
    if((stream.enabled) && !(stream.streaming_vel)){ \
        ROS_INFO("Attempting to enable %s data stream.", cISDataMappings::GetDataSetName(did_vel)); \
        SET_CALLBACK(did_vel, gps_vel_t, cb_fun_vel, stream.period); \
        if (!firstrun) \
            return; \
    }

void InertialSenseROS::configure_data_streams(bool firstrun) // if firstrun is true each step will be attempted without returning
{
    if (!rs_.gps1.streaming_pos) // we always need GPS for Fix status
    {
        ROS_INFO("Attempting to enable GPS1 Pos data stream.");
        SET_CALLBACK(DID_GPS1_POS, gps_pos_t, GPS_pos_callback, rs_.gps1.period);
    }
    if (!flashConfigStreaming_)
    {
        ROS_INFO("Attempting to enable flash config data stream.");
        SET_CALLBACK(DID_FLASH_CONFIG, nvm_flash_cfg_t, flash_config_callback, 0);
        if (!firstrun)
            return;
    }

    bool covarianceConfiged = (covariance_enabled_ && insCovarianceStreaming_) || !covariance_enabled_;

    if (rs_.odom_ins_ned.enabled && !(rs_.ins4.streaming && imuStreaming_ && covarianceConfiged))
    {
        ROS_INFO("Attempting to enable odom INS NED data stream.");

        SET_CALLBACK(DID_INS_4, ins_4_t, INS4_callback, rs_.ins4.period);                                                   // Need NED
        if (covariance_enabled_)
            SET_CALLBACK(DID_ROS_COVARIANCE_POSE_TWIST, ros_covariance_pose_twist_t, INS_covariance_callback, 200); // Need Covariance data
        SET_CALLBACK(DID_PIMU, pimu_t, preint_IMU_callback, rs_.pimu.period);                     // Need angular rate data from IMU
        rs_.imu.enabled = true;
        // Create Identity Matrix
        //
        for (int row = 0; row < 6; row++)
        {
            for (int col = 0; col < 6; col++)
            {
                if (row == col)
                {
                    ned_odom_msg.pose.covariance[row * 6 + col] = 1;
                    ned_odom_msg.twist.covariance[row * 6 + col] = 1;
                }
                else
                {
                    ned_odom_msg.pose.covariance[row * 6 + col] = 0;
                    ned_odom_msg.twist.covariance[row * 6 + col] = 0;
                }
            }
        }
        if (!firstrun)
            return;;
    }

    if (rs_.odom_ins_ecef.enabled && !(rs_.ins4.streaming && imuStreaming_ && covarianceConfiged))
    {
        ROS_INFO("Attempting to enable odom INS ECEF data stream.");
        SET_CALLBACK(DID_INS_4, ins_4_t, INS4_callback, rs_.ins4.period);                                                   // Need quaternion and ecef
        if (covariance_enabled_)
            SET_CALLBACK(DID_ROS_COVARIANCE_POSE_TWIST, ros_covariance_pose_twist_t, INS_covariance_callback, 200); // Need Covariance data
        SET_CALLBACK(DID_PIMU, pimu_t, preint_IMU_callback, rs_.pimu.period);                                              // Need angular rate data from IMU
        rs_.imu.enabled = true;
        // Create Identity Matrix
        //
        for (int row = 0; row < 6; row++)
        {
            for (int col = 0; col < 6; col++)
            {
                if (row == col)
                {
                    ecef_odom_msg.pose.covariance[row * 6 + col] = 1;
                    ecef_odom_msg.twist.covariance[row * 6 + col] = 1;
                }
                else
                {
                    ecef_odom_msg.pose.covariance[row * 6 + col] = 0;
                    ecef_odom_msg.twist.covariance[row * 6 + col] = 0;
                }
            }
        }
        if (!firstrun)
            return;
    }

    if (rs_.odom_ins_enu.enabled  && !(rs_.ins4.streaming && imuStreaming_ && covarianceConfiged))
    {
        ROS_INFO("Attempting to enable odom INS ENU data stream.");
        SET_CALLBACK(DID_INS_4, ins_4_t, INS4_callback, rs_.ins4.period);                                                   // Need ENU
        if (covariance_enabled_)
            SET_CALLBACK(DID_ROS_COVARIANCE_POSE_TWIST, ros_covariance_pose_twist_t, INS_covariance_callback, 200); // Need Covariance data
        SET_CALLBACK(DID_PIMU, pimu_t, preint_IMU_callback, rs_.pimu.period);                                              // Need angular rate data from IMU
        rs_.imu.enabled = true;
        // Create Identity Matrix
        //
        for (int row = 0; row < 6; row++)
        {
            for (int col = 0; col < 6; col++)
            {
                if (row == col)
                {
                    enu_odom_msg.pose.covariance[row * 6 + col] = 1;
                    enu_odom_msg.twist.covariance[row * 6 + col] = 1;
                }
                else
                {
                    enu_odom_msg.pose.covariance[row * 6 + col] = 0;
                    enu_odom_msg.twist.covariance[row * 6 + col] = 0;
                }
            }
        }
        if (!firstrun)
            return;
    }

    CONFIG_STREAM(rs_.ins1, DID_INS_1, ins_1_t, INS1_callback);
    CONFIG_STREAM(rs_.ins2, DID_INS_2, ins_2_t, INS2_callback);
    CONFIG_STREAM(rs_.ins4, DID_INS_4, ins_4_t, INS4_callback);
    CONFIG_STREAM(rs_.inl2_states, DID_INL2_STATES, inl2_states_t, INL2_states_callback);

    if (rs_.navsatfix.enabled && !NavSatFixConfigured)
    {
        ROS_INFO("Attempting to enable NavSatFix.");

        // Satellite system constellation used in GNSS solution.  (see eGnssSatSigConst) 0x0003=GPS, 0x000C=QZSS, 0x0030=Galileo, 0x00C0=Beidou, 0x0300=GLONASS, 0x1000=SBAS
        uint16_t gnssSatSigConst = IS_.GetFlashConfig().gnssSatSigConst;

        if (gnssSatSigConst & GNSS_SAT_SIG_CONST_GPS)
        {
            NavSatFix_msg.status.service |= NavSatFixService::SERVICE_GPS;
        }
        if (gnssSatSigConst & GNSS_SAT_SIG_CONST_GLO)
        {
            NavSatFix_msg.status.service |= NavSatFixService::SERVICE_GLONASS;
        }
        if (gnssSatSigConst & GNSS_SAT_SIG_CONST_BDS)
        {
            NavSatFix_msg.status.service |= NavSatFixService::SERVICE_COMPASS; // includes BeiDou.
        }
        if (gnssSatSigConst & GNSS_SAT_SIG_CONST_GAL)
        {
            NavSatFix_msg.status.service |= NavSatFixService::SERVICE_GALILEO;
        }
        NavSatFixConfigured = true;
        // DID_GPS1_POS and DID_GPS1_VEL are always streamed for fix status. See below
        if (!firstrun)
            return;
    }

    if (rs_.gps1.enabled)
    {   // Set up the GPS ROS stream - we always need GPS information for time sync, just don't always need to publish it
        CONFIG_STREAM_GPS(rs_.gps1, DID_GPS1_POS, GPS_pos_callback, DID_GPS1_VEL, GPS_vel_callback);
        CONFIG_STREAM(rs_.gps1_raw, DID_GPS1_RAW, gps_raw_t, GPS_raw_callback);
        CONFIG_STREAM(rs_.gps1_info, DID_GPS1_SAT, gps_sat_t, GPS_info_callback);
    }

    if (rs_.gps2.enabled)
    {
        CONFIG_STREAM_GPS(rs_.gps2, DID_GPS2_POS, GPS_pos_callback, DID_GPS2_VEL, GPS_vel_callback);
        CONFIG_STREAM(rs_.gps2_raw, DID_GPS2_RAW, gps_raw_t, GPS_raw_callback);
        CONFIG_STREAM(rs_.gps2_info, DID_GPS2_SAT, gps_sat_t, GPS_info_callback);
    }

    CONFIG_STREAM(rs_.gpsbase_raw, DID_GPS_BASE_RAW, gps_raw_t, GPS_raw_callback);

    CONFIG_STREAM(rs_.mag, DID_MAGNETOMETER, magnetometer_t, mag_callback);
    CONFIG_STREAM(rs_.baro, DID_BAROMETER, barometer_t, baro_callback);
    CONFIG_STREAM(rs_.pimu, DID_PIMU, pimu_t, preint_IMU_callback);
    CONFIG_STREAM(rs_.imu, DID_PIMU, pimu_t, preint_IMU_callback);  // We read PIMU and convert to IMU 

    if (!firstrun)
    {
        data_streams_enabled_ = true;
        data_stream_timer_.stop();
        ROS_INFO("Inertial Sense ROS data streams successfully enabled.");
        return;
    }
}

void InertialSenseROS::start_log()
{
    std::string filename = getenv("HOME");
    filename += "/Documents/Inertial_Sense/Logs/" + cISLogger::CreateCurrentTimestamp();
    ROS_INFO_STREAM("Creating log in " << filename << " folder");
    IS_.SetLoggerEnabled(true, filename, cISLogger::LOGTYPE_DAT, RMC_PRESET_PPD_GROUND_VEHICLE);
}

void InertialSenseROS::configure_ascii_output()
{
    //  ascii_msgs_t msgs = {};
    //  msgs.options = (NMEA_message_ports & NMEA_SER0) ? RMC_OPTIONS_PORT_SER0 : 0; // output on serial 0
    //  msgs.options |= (NMEA_message_ports & NMEA_SER1) ? RMC_OPTIONS_PORT_SER1 : 0; // output on serial 1
    //  msgs.gpgga = (NMEA_message_configuration & NMEA_GPGGA) ? NMEA_rate : 0;
    //  msgs.gpgll = (NMEA_message_configuration & NMEA_GPGLL) ? NMEA_rate : 0;
    //  msgs.gpgsa = (NMEA_message_configuration & NMEA_GPGSA) ? NMEA_rate : 0;
    //  msgs.gprmc = (NMEA_message_configuration & NMEA_GPRMC) ? NMEA_rate : 0;
    //  IS_.SendData(DID_ASCII_BCAST_PERIOD, (uint8_t*)(&msgs), sizeof(ascii_msgs_t), 0);
}

void InertialSenseROS::connect()
{
    /// Connect to the uINS
    ROS_INFO("Connecting to serial port \"%s\", at %d baud", port_.c_str(), baudrate_);
    if (!IS_.Open(port_.c_str(), baudrate_))
    {
        ROS_FATAL("inertialsense: Unable to open serial port \"%s\", at %d baud", port_.c_str(), baudrate_);
        exit(0);
    }
    else
    {
        // Print if Successful
        ROS_INFO("Connected to uINS %d on \"%s\", at %d baud", IS_.GetDeviceInfo().serialNumber, port_.c_str(), baudrate_);
    }
}

bool InertialSenseROS::firmware_compatiblity_check()
{
    if( IS_.GetDeviceInfo().protocolVer[0] != PROTOCOL_VERSION_CHAR0 || \
        IS_.GetDeviceInfo().protocolVer[1] != PROTOCOL_VERSION_CHAR1 || \
        IS_.GetDeviceInfo().protocolVer[2] != PROTOCOL_VERSION_CHAR2 || \
        IS_.GetDeviceInfo().protocolVer[3] != PROTOCOL_VERSION_CHAR3 || \
        IS_.GetDeviceInfo().firmwareVer[0] != FIRMWARE_VERSION_CHAR0 || \
        IS_.GetDeviceInfo().firmwareVer[1] != FIRMWARE_VERSION_CHAR1 || \
        IS_.GetDeviceInfo().firmwareVer[2] != FIRMWARE_VERSION_CHAR2)
    {
        ROS_FATAL(
            "Protocol version mismatch: \n"
            "   protocol %d.%d.%d.%d  firmware %d.%d.%d  (SDK)\n"
            "   protocol %d.%d.%d.%d  firmware %d.%d.%d  (device)", 
            PROTOCOL_VERSION_CHAR0,
            PROTOCOL_VERSION_CHAR1,
            PROTOCOL_VERSION_CHAR2,
            PROTOCOL_VERSION_CHAR3,
            FIRMWARE_VERSION_CHAR0,
            FIRMWARE_VERSION_CHAR1, 
            FIRMWARE_VERSION_CHAR2,
            IS_.GetDeviceInfo().protocolVer[0],
            IS_.GetDeviceInfo().protocolVer[1],
            IS_.GetDeviceInfo().protocolVer[2],
            IS_.GetDeviceInfo().protocolVer[3],
            IS_.GetDeviceInfo().firmwareVer[0],
            IS_.GetDeviceInfo().firmwareVer[1],
            IS_.GetDeviceInfo().firmwareVer[2]      
            );
        return false;
    }
    else
    {
        return true;
    }
}

void InertialSenseROS::configure_flash_parameters()
{
    bool reboot = false;
    nvm_flash_cfg_t current_flash_cfg = IS_.GetFlashConfig();
    //ROS_INFO("Configuring flash: \nCurrent: %i, \nDesired: %i\n", current_flash_cfg.ioConfig, ioConfig_);

    if (current_flash_cfg.startupNavDtMs != navigation_dt_ms_)
    {
        reboot = true;
        ROS_INFO("navigation rate change from %dms to %dms, resetting uINS to make change", current_flash_cfg.startupNavDtMs, navigation_dt_ms_);
    }
    if (current_flash_cfg.ioConfig != ioConfig_)
    {
        ROS_INFO("ioConfig change from %x to %x, resetting uINS to make change", current_flash_cfg.ioConfig, ioConfig_);
        reboot = true;
    }

    if (current_flash_cfg.startupNavDtMs != navigation_dt_ms_ ||\
        current_flash_cfg.insRotation != insRotation_ ||\
        current_flash_cfg.insOffset != insOffset_ ||\
        current_flash_cfg.gps1AntOffset != gps1AntOffset_ ||\
        current_flash_cfg.gps2AntOffset != gps2AntOffset_ ||\
        current_flash_cfg.refLla != refLla_ ||\
        current_flash_cfg.insDynModel != insDynModel_ ||\
        current_flash_cfg.ioConfig != ioConfig_)
    {
        current_flash_cfg.startupNavDtMs = navigation_dt_ms_;
        memcpy(current_flash_cfg.insRotation, insRotation_, sizeof(insRotation_));
        memcpy(current_flash_cfg.insOffset, insOffset_, sizeof(insOffset_));
        memcpy(current_flash_cfg.gps1AntOffset, gps1AntOffset_, sizeof(gps1AntOffset_));
        memcpy(current_flash_cfg.gps2AntOffset, gps2AntOffset_, sizeof(gps2AntOffset_));
        memcpy(current_flash_cfg.refLla, refLla_, sizeof(refLla_));
        current_flash_cfg.ioConfig = ioConfig_;
        current_flash_cfg.gpsTimeUserDelay = gpsTimeUserDelay_;
        current_flash_cfg.magDeclination = magDeclination_;
        current_flash_cfg.insDynModel = insDynModel_;

        IS_.SendData(DID_FLASH_CONFIG, (uint8_t *)(&current_flash_cfg), sizeof (nvm_flash_cfg_t), 0);
    }

    if  (reboot)
    {
        sleep(3);
        reset_device();
    }
}

void InertialSenseROS::connect_rtk_client(const std::string &rtk_correction_protocol, const std::string &rtk_server_IP, const int rtk_server_port)
{
    rtk_connecting_ = true;

    // [type]:[protocol]:[ip/url]:[port]:[mountpoint]:[username]:[password]
    std::string RTK_connection = "TCP:" + RTK_correction_protocol_ + ":" + rtk_server_IP + ":" + std::to_string(rtk_server_port);
    if (!RTK_server_mount_.empty() && !RTK_server_username_.empty())
    { // NTRIP options
        RTK_connection += ":" + RTK_server_mount_ + ":" + RTK_server_username_ + ":" + RTK_server_password_;
    }

    int RTK_connection_attempt_count = 0;
    while (RTK_connection_attempt_count < RTK_connection_attempt_limit_)
    {
        ++RTK_connection_attempt_count;

        bool connected = IS_.OpenConnectionToServer(RTK_connection);

        if (connected)
        {
            ROS_INFO_STREAM("Successfully connected to " << RTK_connection << " RTK server");
            break;
        }
        else
        {
            ROS_ERROR_STREAM("Failed to connect to base server at " << RTK_connection);

            if (RTK_connection_attempt_count >= RTK_connection_attempt_limit_)
            {
                ROS_ERROR_STREAM("Giving up after " << RTK_connection_attempt_count << " failed attempts");
            }
            else
            {
                int sleep_duration = RTK_connection_attempt_count * RTK_connection_attempt_backoff_;
                ROS_WARN_STREAM("Retrying connection in " << sleep_duration << " seconds");
                ros::Duration(sleep_duration).sleep();
            }
        }
    }

    rtk_connecting_ = false;
}

void InertialSenseROS::start_rtk_server(const std::string &rtk_server_IP, const int rtk_server_port)
{
    // [type]:[ip/url]:[port]
    std::string RTK_connection = "TCP:" + rtk_server_IP + ":" + std::to_string(rtk_server_port);

    if (IS_.CreateHost(RTK_connection))
    {
        ROS_INFO_STREAM("Successfully created " << RTK_connection << " as RTK server");
        initialized_ = true;
        return;
    }
    else
        ROS_ERROR_STREAM("Failed to create base server at " << RTK_connection);
}

void InertialSenseROS::start_rtk_connectivity_watchdog_timer()
{

    if (!rtk_connectivity_watchdog_enabled_)
    {
        return;
    }

    if (!rtk_connectivity_watchdog_timer_.isValid())
    {
        rtk_connectivity_watchdog_timer_ = nh_.createTimer(ros::Duration(rtk_connectivity_watchdog_timer_frequency_), InertialSenseROS::rtk_connectivity_watchdog_timer_callback, this);
    }

    rtk_connectivity_watchdog_timer_.start();
}

void InertialSenseROS::stop_rtk_connectivity_watchdog_timer()
{
    rtk_traffic_total_byte_count_ = 0;
    rtk_data_transmission_interruption_count_ = 0;
    rtk_connectivity_watchdog_timer_.stop();
}

void InertialSenseROS::rtk_connectivity_watchdog_timer_callback(const ros::TimerEvent &timer_event)
{
    if (rtk_connecting_)
    {
        return;
    }

    int latest_byte_count = IS_.GetClientServerByteCount();
    if (rtk_traffic_total_byte_count_ == latest_byte_count)
    {
        ++rtk_data_transmission_interruption_count_;

        if (rtk_data_transmission_interruption_count_ >= rtk_data_transmission_interruption_limit_)
        {
            ROS_WARN("RTK transmission interruption, reconnecting...");

            connect_rtk_client(RTK_correction_protocol_, RTK_server_IP_, RTK_server_port_);
        }
    }
    else
    {
        rtk_traffic_total_byte_count_ = latest_byte_count;
        rtk_data_transmission_interruption_count_ = 0;
    }
}

void InertialSenseROS::configure_rtk()
{
    uint32_t RTKCfgBits = 0;
    if (gps1_type_ == "F9P")
    {
        if (RTK_rover_)
        {
            ROS_INFO("InertialSense: RTK Rover Configured.");
            rs_.rtk_pos.enabled = true;
            connect_rtk_client(RTK_correction_protocol_, RTK_server_IP_, RTK_server_port_);
            RTKCfgBits |= RTK_CFG_BITS_ROVER_MODE_RTK_POSITIONING_EXTERNAL;
            SET_CALLBACK(DID_GPS1_RTK_POS_MISC, gps_rtk_misc_t, RTK_Misc_callback, rs_.rtk_pos.period);
            SET_CALLBACK(DID_GPS1_RTK_POS_REL, gps_rtk_rel_t, RTK_Rel_callback, rs_.rtk_pos.period);

            start_rtk_connectivity_watchdog_timer();
        }
        if (gnss_compass_)
        {
            ROS_INFO("InertialSense: Dual GNSS (compassing) configured");
            rs_.rtk_cmp.enabled = true;
            RTKCfgBits |= RTK_CFG_BITS_ROVER_MODE_RTK_COMPASSING_F9P;
            SET_CALLBACK(DID_GPS2_RTK_CMP_MISC, gps_rtk_misc_t, RTK_Misc_callback, rs_.rtk_cmp.period);
            SET_CALLBACK(DID_GPS2_RTK_CMP_REL, gps_rtk_rel_t, RTK_Rel_callback, rs_.rtk_cmp.period);
        }
        if (RTK_rover_radio_enable_)
        {
            ROS_INFO("InertialSense: Configured as RTK Rover with radio enabled");
            rs_.rtk_pos.enabled = true;
            RTK_base_USB_ = RTK_base_USB_= false;
            RTK_base_serial_ = RTK_base_serial_= false;
            RTKCfgBits |= RTK_CFG_BITS_ROVER_MODE_RTK_POSITIONING_EXTERNAL;
            SET_CALLBACK(DID_GPS1_RTK_POS_MISC, gps_rtk_misc_t, RTK_Misc_callback, rs_.rtk_pos.period);
            SET_CALLBACK(DID_GPS1_RTK_POS_REL, gps_rtk_rel_t, RTK_Rel_callback, rs_.rtk_pos.period);
        }
        if (RTK_base_USB_)
        {
            ROS_INFO("InertialSense: Base Configured.");
            RTKCfgBits |= RTK_CFG_BITS_BASE_OUTPUT_GPS1_RTCM3_USB;
        }
        if (RTK_base_serial_)
        {
            ROS_INFO("InertialSense: Base Configured.");
            RTKCfgBits |= RTK_CFG_BITS_BASE_OUTPUT_GPS1_RTCM3_SER2;
        }
        if (RTK_base_TCP_)
        {
            start_rtk_server(RTK_server_IP_, RTK_server_port_);
        }

        IS_.SendData(DID_FLASH_CONFIG, reinterpret_cast<uint8_t *>(&RTKCfgBits), sizeof(RTKCfgBits), offsetof(nvm_flash_cfg_t, RTKCfgBits));
    }

    else
    {
        ROS_ERROR_COND(RTK_rover_ && (RTK_base_serial_ || RTK_base_USB_ || RTK_base_TCP_), "unable to configure onboard receiver to be both RTK rover and base - default to rover");
        ROS_ERROR_COND(RTK_rover_ && gnss_compass_, "unable to configure onboard receiver to be both RTK rover as dual GNSS - default to dual GNSS");

        uint32_t RTKCfgBits = 0;
        if (gnss_compass_)
        {
            ROS_INFO("InertialSense: Configured as dual GNSS (compassing)");
            RTK_rover_ = false;
            RTKCfgBits |= RTK_CFG_BITS_ROVER_MODE_RTK_COMPASSING;
            SET_CALLBACK(DID_GPS2_RTK_CMP_MISC, gps_rtk_misc_t, RTK_Misc_callback, rs_.rtk_cmp.period);
            SET_CALLBACK(DID_GPS2_RTK_CMP_REL, gps_rtk_rel_t, RTK_Rel_callback, rs_.rtk_cmp.period);
        }

        if (RTK_rover_radio_enable_)
        {
            ROS_INFO("InertialSense: Configured as RTK Rover with radio enabled");
            RTK_base_serial_ = false;
            RTK_base_USB_ = false;
            RTK_base_TCP_ = false;
            RTKCfgBits |= (gps1_type_ == "F9P" ? RTK_CFG_BITS_ROVER_MODE_RTK_POSITIONING_EXTERNAL : RTK_CFG_BITS_ROVER_MODE_RTK_POSITIONING);
            SET_CALLBACK(DID_GPS1_RTK_POS_MISC, gps_rtk_misc_t, RTK_Misc_callback, rs_.rtk_pos.period);
            SET_CALLBACK(DID_GPS1_RTK_POS_REL, gps_rtk_rel_t, RTK_Rel_callback, rs_.rtk_pos.period);
        }
        else if (RTK_rover_)
        {
            ROS_INFO("InertialSense: Configured as RTK Rover");
            RTK_base_serial_ = false;
            RTK_base_USB_ = false;
            RTK_base_TCP_ = false;
            RTKCfgBits |= (gps1_type_ == "F9P" ? RTK_CFG_BITS_ROVER_MODE_RTK_POSITIONING_EXTERNAL : RTK_CFG_BITS_ROVER_MODE_RTK_POSITIONING);
            connect_rtk_client(RTK_correction_protocol_, RTK_server_IP_, RTK_server_port_);
            SET_CALLBACK(DID_GPS1_RTK_POS_MISC, gps_rtk_misc_t, RTK_Misc_callback, rs_.rtk_pos.period);
            SET_CALLBACK(DID_GPS1_RTK_POS_REL, gps_rtk_rel_t, RTK_Rel_callback, rs_.rtk_pos.period);

            start_rtk_connectivity_watchdog_timer();
        }
        else if (RTK_base_USB_ || RTK_base_serial_ || RTK_base_TCP_)
        {
            ROS_INFO("InertialSense: Configured as RTK Base");
            if (RTK_base_serial_)
                RTKCfgBits |= RTK_CFG_BITS_BASE_OUTPUT_GPS1_UBLOX_SER0;
            if (RTK_base_USB_)
                RTKCfgBits |= RTK_CFG_BITS_BASE_OUTPUT_GPS1_UBLOX_USB;
            if (RTK_base_TCP_)
                start_rtk_server(RTK_server_IP_, RTK_server_port_);
        }
        IS_.SendData(DID_FLASH_CONFIG, reinterpret_cast<uint8_t *>(&RTKCfgBits), sizeof(RTKCfgBits), offsetof(nvm_flash_cfg_t, RTKCfgBits));
    }
    ROS_INFO("Setting RTKCfgBits: 0x%08x", RTKCfgBits);
}

void InertialSenseROS::flash_config_callback(eDataIDs DID, const nvm_flash_cfg_t *const msg)
{
    STREAMING_CHECK(flashConfigStreaming_, DID);

    refLla_[0] = msg->refLla[0];
    refLla_[1] = msg->refLla[1];
    refLla_[2] = msg->refLla[2];
    refLLA_known = true;
    ROS_INFO("refLla was set");
}

void InertialSenseROS::INS1_callback(eDataIDs DID, const ins_1_t *const msg)
{
    STREAMING_CHECK(rs_.ins1.streaming, DID);

    // Standard DID_INS_1 message
    if (rs_.ins1.enabled)
    {
        did_ins_1_msg.header.stamp = ros_time_from_week_and_tow(msg->week, msg->timeOfWeek);
        did_ins_1_msg.header.frame_id = frame_id_;
        did_ins_1_msg.week = msg->week;
        did_ins_1_msg.timeOfWeek = msg->timeOfWeek;
        did_ins_1_msg.insStatus = msg->insStatus;
        did_ins_1_msg.hdwStatus = msg->hdwStatus;
        did_ins_1_msg.theta[0] = msg->theta[0];
        did_ins_1_msg.theta[1] = msg->theta[1];
        did_ins_1_msg.theta[2] = msg->theta[2];
        did_ins_1_msg.uvw[0] = msg->uvw[0];
        did_ins_1_msg.uvw[1] = msg->uvw[1];
        did_ins_1_msg.uvw[2] = msg->uvw[2];
        did_ins_1_msg.lla[0] = msg->lla[0];
        did_ins_1_msg.lla[1] = msg->lla[1];
        did_ins_1_msg.lla[2] = msg->lla[2];
        did_ins_1_msg.ned[0] = msg->ned[0];
        did_ins_1_msg.ned[1] = msg->ned[1];
        did_ins_1_msg.ned[2] = msg->ned[2];
        rs_.ins1.pub.publish(did_ins_1_msg);
    }
}

void InertialSenseROS::INS2_callback(eDataIDs DID, const ins_2_t *const msg)
{
    STREAMING_CHECK(rs_.ins2.streaming, DID);

    if (rs_.ins2.enabled)
    {
        // Standard DID_INS_2 message
        did_ins_2_msg.header.frame_id = frame_id_;
        did_ins_2_msg.week = msg->week;
        did_ins_2_msg.timeOfWeek = msg->timeOfWeek;
        did_ins_2_msg.insStatus = msg->insStatus;
        did_ins_2_msg.hdwStatus = msg->hdwStatus;
        did_ins_2_msg.qn2b[0] = msg->qn2b[0];
        did_ins_2_msg.qn2b[1] = msg->qn2b[1];
        did_ins_2_msg.qn2b[2] = msg->qn2b[2];
        did_ins_2_msg.qn2b[3] = msg->qn2b[3];
        did_ins_2_msg.uvw[0] = msg->uvw[0];
        did_ins_2_msg.uvw[1] = msg->uvw[1];
        did_ins_2_msg.uvw[2] = msg->uvw[2];
        did_ins_2_msg.lla[0] = msg->lla[0];
        did_ins_2_msg.lla[1] = msg->lla[1];
        did_ins_2_msg.lla[2] = msg->lla[2];
        rs_.ins2.pub.publish(did_ins_2_msg);
    }
}

void InertialSenseROS::INS4_callback(eDataIDs DID, const ins_4_t *const msg)
{
    STREAMING_CHECK(rs_.ins4.streaming, DID);

    if (!refLLA_known)
    {
        ROS_INFO("REFERENCE LLA MUST BE RECEIVED");
        return;
    }
    if (rs_.ins4.enabled)
    {
        // Standard DID_INS_2 message
        did_ins_4_msg.header.frame_id = frame_id_;
        did_ins_4_msg.week = msg->week;
        did_ins_4_msg.timeOfWeek = msg->timeOfWeek;
        did_ins_4_msg.insStatus = msg->insStatus;
        did_ins_4_msg.hdwStatus = msg->hdwStatus;
        did_ins_4_msg.qe2b[0] = msg->qe2b[0];
        did_ins_4_msg.qe2b[1] = msg->qe2b[1];
        did_ins_4_msg.qe2b[2] = msg->qe2b[2];
        did_ins_4_msg.qe2b[3] = msg->qe2b[3];
        did_ins_4_msg.ve[0] = msg->ve[0];
        did_ins_4_msg.ve[1] = msg->ve[1];
        did_ins_4_msg.ve[2] = msg->ve[2];
        did_ins_4_msg.ecef[0] = msg->ecef[0];
        did_ins_4_msg.ecef[1] = msg->ecef[1];
        did_ins_4_msg.ecef[2] = msg->ecef[2];
        rs_.ins4.pub.publish(did_ins_4_msg);
    }


    if (rs_.odom_ins_ned.enabled || rs_.odom_ins_enu.enabled || rs_.odom_ins_ecef.enabled)
    {
        // Note: the covariance matrices need to be transformed into required frames of reference before publishing the ROS message!
        ixMatrix3  Rb2e, I;
        ixVector4  qe2b, qe2n;
        ixVector3d Pe, lla;
        float Pout[36];

        eye_MatN(I, 3);
        qe2b[0] = msg->qe2b[0];
        qe2b[1] = msg->qe2b[1];
        qe2b[2] = msg->qe2b[2];
        qe2b[3] = msg->qe2b[3];

        rotMatB2R(qe2b, Rb2e);
        Pe[0] = msg->ecef[0];
        Pe[1] = msg->ecef[1];
        Pe[2] = msg->ecef[2];
        ecef2lla(Pe, lla, 5);
        quat_ecef2ned(lla[0], lla[1], qe2n);

        if (rs_.odom_ins_ecef.enabled)
        {
            // Pose
            // Transform attitude body to ECEF
            transform_6x6_covariance(Pout, poseCov_, I, Rb2e);
            for (int i = 0; i < 36; i++)
            {
                ecef_odom_msg.pose.covariance[i] = Pout[i];
            }
            // Twist
            // Transform angular_rate from body to ECEF
            transform_6x6_covariance(Pout, twistCov, I, Rb2e);
            for (int i = 0; i < 36; i++)
            {
                ecef_odom_msg.twist.covariance[i] = Pout[i];
            }
            ecef_odom_msg.header.stamp = ros_time_from_week_and_tow(msg->week, msg->timeOfWeek);
            ecef_odom_msg.header.frame_id = frame_id_;

            // Position
            ecef_odom_msg.pose.pose.position.x = msg->ecef[0];
            ecef_odom_msg.pose.pose.position.y = msg->ecef[1];
            ecef_odom_msg.pose.pose.position.z = -msg->ecef[2];

            // Attitude
            ecef_odom_msg.pose.pose.orientation.w = msg->qe2b[0];
            ecef_odom_msg.pose.pose.orientation.x = msg->qe2b[1];
            ecef_odom_msg.pose.pose.orientation.y = msg->qe2b[2];
            ecef_odom_msg.pose.pose.orientation.z = msg->qe2b[3];

            // Linear Velocity
            ecef_odom_msg.twist.twist.linear.x = msg->ve[0];
            ecef_odom_msg.twist.twist.linear.y = msg->ve[1];
            ecef_odom_msg.twist.twist.linear.z = msg->ve[2];

            // Angular Velocity
            ixVector3 result;
            ixEuler theta;
            quat2euler(msg->qe2b, theta);
            ixVector3 angVelImu = {(f_t)imu_msg.angular_velocity.x, (f_t)imu_msg.angular_velocity.y, (f_t)imu_msg.angular_velocity.z};
            vectorBodyToReference(angVelImu, theta, result);

            ecef_odom_msg.twist.twist.angular.x = result[0];
            ecef_odom_msg.twist.twist.angular.y = result[1];
            ecef_odom_msg.twist.twist.angular.z = result[2];

            rs_.odom_ins_ecef.pub.publish(ecef_odom_msg);

            if (publishTf_)
            {
                // Calculate the TF from the pose...
                transform_ECEF.setOrigin(tf::Vector3(ecef_odom_msg.pose.pose.position.x, ecef_odom_msg.pose.pose.position.y, ecef_odom_msg.pose.pose.position.z));
                tf::Quaternion q;
                tf::quaternionMsgToTF(ecef_odom_msg.pose.pose.orientation, q);
                transform_ECEF.setRotation(q);

                br.sendTransform(tf::StampedTransform(transform_ECEF, ros::Time::now(), "ins_ecef", "ins_base_link_ecef"));
            }
        }

        if (rs_.odom_ins_ned.enabled)
        {
            ixVector4 qn2b;
            ixMatrix3 Rb2n, Re2n, buf;

            // NED-to-body quaternion
            mul_Quat_ConjQuat(qn2b, qe2b, qe2n);
            // Body-to-NED rotation matrix
            rotMatB2R(qn2b, Rb2n);
            // ECEF-to-NED rotation matrix
            rotMatB2R(qe2n, buf);
            transpose_Mat3(Re2n, buf);

            // Pose
            // Transform position from ECEF to NED and attitude from body to NED
            transform_6x6_covariance(Pout, poseCov_, Re2n, Rb2n);
            for (int i = 0; i < 36; i++)
            {
                ned_odom_msg.pose.covariance[i] = Pout[i];
            }
            // Twist
            // Transform velocity from ECEF to NED and angular rate from body to NED
            transform_6x6_covariance(Pout, twistCov, Re2n, Rb2n);
            for (int i = 0; i < 36; i++)
            {
                ned_odom_msg.twist.covariance[i] = Pout[i];
            }

            ned_odom_msg.header.stamp = ros_time_from_week_and_tow(msg->week, msg->timeOfWeek);
            ned_odom_msg.header.frame_id = frame_id_;

            // Position
            ixVector3d llaPosRadians;
                //ecef to lla (rad,rad,m)
            ecef2lla(msg->ecef, llaPosRadians, 5);
            ixVector3 ned;
            ixVector3d refLlaRadians;
                //convert refLla_ to radians
            lla_Deg2Rad_d(refLlaRadians, refLla_);
                //lla to ned
            lla2ned_d(refLlaRadians, llaPosRadians, ned);

            ned_odom_msg.pose.pose.position.x = ned[0];
            ned_odom_msg.pose.pose.position.y = ned[1];
            ned_odom_msg.pose.pose.position.z = ned[2];

            // Attitude
            ned_odom_msg.pose.pose.orientation.w = qn2b[0]; // w
            ned_odom_msg.pose.pose.orientation.x = qn2b[1]; // x
            ned_odom_msg.pose.pose.orientation.y = qn2b[2]; // y
            ned_odom_msg.pose.pose.orientation.z = qn2b[3]; // z

            // Linear Velocity
            ixVector3 result, theta;

            quatConjRot(result, qe2n, msg->ve);

            ned_odom_msg.twist.twist.linear.x = result[0];
            ned_odom_msg.twist.twist.linear.y = result[1];
            ned_odom_msg.twist.twist.linear.z = result[2];

            // Angular Velocity
            // Transform from body frame to NED
            ixVector3 angVelImu = {(f_t)imu_msg.angular_velocity.x, (f_t)imu_msg.angular_velocity.y, (f_t)imu_msg.angular_velocity.z};
            quatRot(result, qn2b, angVelImu);

            ned_odom_msg.twist.twist.angular.x = result[0];
            ned_odom_msg.twist.twist.angular.y = result[1];
            ned_odom_msg.twist.twist.angular.z = result[2];
            rs_.odom_ins_ned.pub.publish(ned_odom_msg);

            if (publishTf_)
            {
                // Calculate the TF from the pose...
                transform_NED.setOrigin(tf::Vector3(ned_odom_msg.pose.pose.position.x, ned_odom_msg.pose.pose.position.y, ned_odom_msg.pose.pose.position.z));
                tf::Quaternion q;
                tf::quaternionMsgToTF(ned_odom_msg.pose.pose.orientation, q);
                transform_NED.setRotation(q);

                br.sendTransform(tf::StampedTransform(transform_NED, ros::Time::now(), "ins_ned", "ins_base_link_ned"));
            }
        }

        if (rs_.odom_ins_enu.enabled)
        {
            ixVector4 qn2b, qn2enu, qe2enu, qenu2b;
            ixMatrix3 Rb2enu, Re2enu, buf;
            ixEuler eul = {M_PI, 0, 0.5 * M_PI};
            // ENU-to-NED quaternion
            euler2quat(eul, qn2enu);
            // NED-to-body quaternion
            mul_Quat_ConjQuat(qn2b, qe2b, qe2n);
            // ENU-to-body quaternion
            mul_Quat_ConjQuat(qenu2b, qn2b, qn2enu);
            // ECEF-to-ENU quaternion
            mul_Quat_Quat(qe2enu, qn2enu, qe2n);
            // Body-to-ENU rotation matrix
            rotMatB2R(qenu2b, Rb2enu);
            // ECEF-to-ENU rotation matrix
            rotMatB2R(qe2enu, buf);
            transpose_Mat3(Re2enu, buf);

            // Pose
            // Transform position from ECEF to ENU and attitude from body to ENU
            transform_6x6_covariance(Pout, poseCov_, Re2enu, Rb2enu);
            for (int i = 0; i < 36; i++)
            {
                enu_odom_msg.pose.covariance[i] = Pout[i];
            }
            // Twist
            // Transform velocity from ECEF to ENU and angular rate from body to ENU
            transform_6x6_covariance(Pout, twistCov, Re2enu, Rb2enu);
            for (int i = 0; i < 36; i++)
            {
                enu_odom_msg.twist.covariance[i] = Pout[i];
            }

            enu_odom_msg.header.stamp = ros_time_from_week_and_tow(msg->week, msg->timeOfWeek);
            enu_odom_msg.header.frame_id = frame_id_;

            // Position
                //Calculate in NED then convert
            ixVector3d llaPosRadians;
                //ecef to lla (rad,rad,m)
            ecef2lla(msg->ecef, llaPosRadians, 5);
            ixVector3 ned;
            ixVector3d refLlaRadians;
                //convert refLla_ to radians
            lla_Deg2Rad_d(refLlaRadians, refLla_);
                //lla to ned
            lla2ned_d(refLlaRadians, llaPosRadians, ned);

            // Rearrange from NED to ENU
            enu_odom_msg.pose.pose.position.x = ned[1];
            enu_odom_msg.pose.pose.position.y = ned[0];
            enu_odom_msg.pose.pose.position.z = -ned[2];

            // Attitude
            enu_odom_msg.pose.pose.orientation.w = qenu2b[0];
            enu_odom_msg.pose.pose.orientation.x = qenu2b[1];
            enu_odom_msg.pose.pose.orientation.y = qenu2b[2];
            enu_odom_msg.pose.pose.orientation.z = qenu2b[3];

            // Linear Velocity
                //same as NED but rearranged.
            ixVector3 result, theta;
            quatConjRot(result, qe2n, msg->ve);

            enu_odom_msg.twist.twist.linear.x = result[1];
            enu_odom_msg.twist.twist.linear.y = result[0];
            enu_odom_msg.twist.twist.linear.z = -result[2];

            // Angular Velocity
            // Transform from body frame to ENU
            ixVector3 angVelImu = {(f_t)imu_msg.angular_velocity.x, (f_t)imu_msg.angular_velocity.y, (f_t)imu_msg.angular_velocity.z};
            quatRot(result, qenu2b, angVelImu);

            enu_odom_msg.twist.twist.angular.x = result[0];
            enu_odom_msg.twist.twist.angular.y = result[1];
            enu_odom_msg.twist.twist.angular.z = result[2];

            rs_.odom_ins_enu.pub.publish(enu_odom_msg);
            if (publishTf_)
            {
                // Calculate the TF from the pose...
                transform_ENU.setOrigin(tf::Vector3(enu_odom_msg.pose.pose.position.x, enu_odom_msg.pose.pose.position.y, enu_odom_msg.pose.pose.position.z));
                tf::Quaternion q;
                tf::quaternionMsgToTF(enu_odom_msg.pose.pose.orientation, q);
                transform_ENU.setRotation(q);

                br.sendTransform(tf::StampedTransform(transform_ENU, ros::Time::now(), "ins_enu", "ins_base_link_enu"));
            }
        }
    }
}

void InertialSenseROS::INL2_states_callback(eDataIDs DID, const inl2_states_t *const msg)
{
    STREAMING_CHECK(rs_.inl2_states.streaming, DID);

    inl2_states_msg.header.stamp = ros_time_from_tow(msg->timeOfWeek);
    inl2_states_msg.header.frame_id = frame_id_;

    inl2_states_msg.quatEcef.w = msg->qe2b[0];
    inl2_states_msg.quatEcef.x = msg->qe2b[1];
    inl2_states_msg.quatEcef.y = msg->qe2b[2];
    inl2_states_msg.quatEcef.z = msg->qe2b[3];

    inl2_states_msg.velEcef.x = msg->ve[0];
    inl2_states_msg.velEcef.y = msg->ve[1];
    inl2_states_msg.velEcef.z = msg->ve[2];

    inl2_states_msg.posEcef.x = msg->ecef[0];
    inl2_states_msg.posEcef.y = msg->ecef[1];
    inl2_states_msg.posEcef.z = msg->ecef[2];

    inl2_states_msg.gyroBias.x = msg->biasPqr[0];
    inl2_states_msg.gyroBias.y = msg->biasPqr[1];
    inl2_states_msg.gyroBias.z = msg->biasPqr[2];

    inl2_states_msg.accelBias.x = msg->biasAcc[0];
    inl2_states_msg.accelBias.y = msg->biasAcc[1];
    inl2_states_msg.accelBias.z = msg->biasAcc[2];

    inl2_states_msg.baroBias = msg->biasBaro;
    inl2_states_msg.magDec = msg->magDec;
    inl2_states_msg.magInc = msg->magInc;

    // Use custom INL2 states message
    if (rs_.inl2_states.enabled)
    {
        rs_.inl2_states.pub.publish(inl2_states_msg);
    }
}

void InertialSenseROS::INS_covariance_callback(eDataIDs DID, const ros_covariance_pose_twist_t *const msg)
{
    STREAMING_CHECK(insCovarianceStreaming_, DID);
    
    float poseCovIn[36];
    int ind1, ind2;

    // Pose and twist covariances unwrapped from LD
    LD2Cov(msg->covPoseLD, poseCovIn, 6);
    LD2Cov(msg->covTwistLD, twistCov, 6);

    // Need to change order of variables.
    // Incoming order for msg->covPoseLD is [attitude, position]. Outgoing should be [position, attitude] => need to swap
    // Incoming order for msg->covTwistLD is [lin_velocity, ang_rate]. Outgoing should be [lin_velocity, ang_rate] => no change
    // Order change (block swap) in covariance matrix:
    // |A  C| => |B  C'|
    // |C' B|    |C  A |
    // where A and B are symetric, C' is transposed C

    for (int i = 0; i < 3; i++) {
        for (int j = 0; j <= i; j++) {
            // Swap blocks A and B
            ind1 = (i + 3) * 6 + j + 3;
            ind2 = i * 6 + j;
            poseCov_[ind2] = poseCovIn[ind1];
            poseCov_[ind1] = poseCovIn[ind2];
            if (i != j) {
                // Copy lower diagonals to upper diagonals
                poseCov_[j * 6 + i] = poseCov_[ind2];
                poseCov_[(j + 3) * 6 + (i + 3)] = poseCov_[ind1];
            }
        }
        // Swap blocks C and C'
        for (int j = 0; j < 3; j++) {
            ind1 = (i + 3) * 6 + j;
            ind2 = i * 6 + j + 3;
            poseCov_[ind2] = poseCovIn[ind1];
            poseCov_[ind1] = poseCovIn[ind2];
        }
    }
}


void InertialSenseROS::GPS_pos_callback(eDataIDs DID, const gps_pos_t *const msg)
{
    static eDataIDs primaryGpsDid = DID_GPS2_POS;  // Use GPS2 if GPS1 is disabled 

    switch (DID)
    {
    case DID_GPS1_POS:
        STREAMING_CHECK(rs_.gps1.streaming_pos, DID);
        primaryGpsDid = DID;

        if (rs_.gps1.enabled && msg->status & GPS_STATUS_FIX_MASK)
        {
            gps1_msg.header.stamp = ros_time_from_week_and_tow(msg->week, msg->timeOfWeekMs / 1.0e3);
            gps1_msg.week = msg->week;
            gps1_msg.status = msg->status;
            gps1_msg.header.frame_id = frame_id_;
            gps1_msg.num_sat = (uint8_t)(msg->status & GPS_STATUS_NUM_SATS_USED_MASK);
            gps1_msg.cno = msg->cnoMean;
            gps1_msg.latitude = msg->lla[0];
            gps1_msg.longitude = msg->lla[1];
            gps1_msg.altitude = msg->lla[2];
            gps1_msg.posEcef.x = ecef_[0] = msg->ecef[0];
            gps1_msg.posEcef.y = ecef_[1] = msg->ecef[1];
            gps1_msg.posEcef.z = ecef_[2] = msg->ecef[2];
            gps1_msg.hMSL = msg->hMSL;
            gps1_msg.hAcc = msg->hAcc;
            gps1_msg.vAcc = msg->vAcc;
            gps1_msg.pDop = msg->pDop;
            publishGPS1();
        }
        break;

    case DID_GPS2_POS:
        STREAMING_CHECK(rs_.gps2.streaming_pos, DID);
        if (rs_.gps2.enabled && msg->status & GPS_STATUS_FIX_MASK)
        {
            gps2_msg.header.stamp = ros_time_from_week_and_tow(msg->week, msg->timeOfWeekMs / 1.0e3);
            gps2_msg.week = msg->week;
            gps2_msg.status = msg->status;
            gps2_msg.header.frame_id = frame_id_;
            gps2_msg.num_sat = (uint8_t)(msg->status & GPS_STATUS_NUM_SATS_USED_MASK);
            gps2_msg.cno = msg->cnoMean;
            gps2_msg.latitude = msg->lla[0];
            gps2_msg.longitude = msg->lla[1];
            gps2_msg.altitude = msg->lla[2];
            gps2_msg.posEcef.x = ecef_[0] = msg->ecef[0];
            gps2_msg.posEcef.y = ecef_[1] = msg->ecef[1];
            gps2_msg.posEcef.z = ecef_[2] = msg->ecef[2];
            gps2_msg.hMSL = msg->hMSL;
            gps2_msg.hAcc = msg->hAcc;
            gps2_msg.vAcc = msg->vAcc;
            gps2_msg.pDop = msg->pDop;
            publishGPS2();
        }
        break;
    }

    if (primaryGpsDid == DID)
    {
        GPS_week_ = msg->week;
        GPS_towOffset_ = msg->towOffset;

        if (rs_.navsatfix.enabled)
        {
            NavSatFix_msg.header.stamp = ros_time_from_week_and_tow(msg->week, msg->timeOfWeekMs / 1.0e3);
            NavSatFix_msg.header.frame_id = frame_id_;
            NavSatFix_msg.status.status = -1;                           // Assume no Fix
            if (msg->status & GPS_STATUS_FIX_MASK >= GPS_STATUS_FIX_2D) // Check for fix and set
            {
                NavSatFix_msg.status.status = NavSatFixStatusFixType::STATUS_FIX;
            }

            if (msg->status & GPS_STATUS_FIX_SBAS) // Check for SBAS only fix
            {
                NavSatFix_msg.status.status = NavSatFixStatusFixType::STATUS_SBAS_FIX;
            }

            if (msg->status & GPS_STATUS_FIX_MASK >= GPS_STATUS_FIX_RTK_SINGLE) // Check for any RTK fix
            {
                NavSatFix_msg.status.status = NavSatFixStatusFixType::STATUS_GBAS_FIX;
            }

            // NavSatFix_msg.status.service - Service set at Node Startup
            NavSatFix_msg.latitude = msg->lla[0];
            NavSatFix_msg.longitude = msg->lla[1];
            NavSatFix_msg.altitude = msg->lla[2];

            // Diagonal Known
            const double varH = pow(msg->hAcc / 1000.0, 2);
            const double varV = pow(msg->vAcc / 1000.0, 2);
            NavSatFix_msg.position_covariance[0] = varH;
            NavSatFix_msg.position_covariance[4] = varH;
            NavSatFix_msg.position_covariance[8] = varV;
            NavSatFix_msg.position_covariance_type = COVARIANCE_TYPE_DIAGONAL_KNOWN;
            rs_.navsatfix.pub.publish(NavSatFix_msg);
        }
    }
}

void InertialSenseROS::GPS_vel_callback(eDataIDs DID, const gps_vel_t *const msg)
{
    switch (DID)
    {
    case DID_GPS1_VEL:
        STREAMING_CHECK(rs_.gps1.streaming_vel, DID);
        if (rs_.gps1.enabled && abs(GPS_towOffset_) > 0.001)
        {
            gps1_velEcef.header.stamp = ros_time_from_week_and_tow(GPS_week_, msg->timeOfWeekMs / 1.0e3);
            gps1_velEcef.vector.x = msg->vel[0];
            gps1_velEcef.vector.y = msg->vel[1];
            gps1_velEcef.vector.z = msg->vel[2];
            gps1_sAcc = msg->sAcc;
            publishGPS1();
        }
        break;

    case DID_GPS2_VEL:
        STREAMING_CHECK(rs_.gps2.streaming_vel, DID);
        if (rs_.gps2.enabled && abs(GPS_towOffset_) > 0.001)
        {
            gps2_velEcef.header.stamp = ros_time_from_week_and_tow(GPS_week_, msg->timeOfWeekMs / 1.0e3);
            gps2_velEcef.vector.x = msg->vel[0];
            gps2_velEcef.vector.y = msg->vel[1];
            gps2_velEcef.vector.z = msg->vel[2];
            gps2_sAcc = msg->sAcc;
            publishGPS2();
        }
        break;
    }
}

void InertialSenseROS::publishGPS1()
{
    double dt = (gps1_velEcef.header.stamp - gps1_msg.header.stamp).toSec();
    if (abs(dt) < 2.0e-3)
    {
        gps1_msg.velEcef = gps1_velEcef.vector;
        gps1_msg.sAcc = gps1_sAcc;
        rs_.gps1.pub.publish(gps1_msg);
    }
}

void InertialSenseROS::publishGPS2()
{
    double dt = (gps2_velEcef.header.stamp - gps2_msg.header.stamp).toSec();
    if (abs(dt) < 2.0e-3)
    {
        gps2_msg.velEcef = gps2_velEcef.vector;
        gps2_msg.sAcc = gps2_sAcc;
        rs_.gps2.pub.publish(gps2_msg);
    }
}

void InertialSenseROS::update()
{
    IS_.Update();
}

void InertialSenseROS::strobe_in_time_callback(eDataIDs DID, const strobe_in_time_t *const msg)
{
    switch (DID)
    {
    case DID_STROBE_IN_TIME:
        STREAMING_CHECK(strobeInStreaming_, DID);

        if (abs(GPS_towOffset_) > 0.001)
        {
            std_msgs::Header strobe_msg;
            strobe_msg.stamp = ros_time_from_week_and_tow(msg->week, msg->timeOfWeekMs * 1.0e-3);
            strobe_pub_.publish(strobe_msg);
        }
        break;
    }
}

void InertialSenseROS::GPS_info_callback(eDataIDs DID, const gps_sat_t *const msg)
{
    switch (DID)
    {
    case DID_GPS1_SAT:  STREAMING_CHECK(rs_.gps1_info.streaming, DID);   break;
    case DID_GPS2_SAT:  STREAMING_CHECK(rs_.gps2_info.streaming, DID);   break;
    default: return;
    }

    if (abs(GPS_towOffset_) < 0.001) 
    { // Wait for valid msg->timeOfWeekMs
        return;
    }

    gps_info_msg.header.stamp = ros_time_from_tow(msg->timeOfWeekMs / 1.0e3);
    gps_info_msg.header.frame_id = frame_id_;
    gps_info_msg.num_sats = msg->numSats;
    for (int i = 0; i < 50; i++) {
        gps_info_msg.sattelite_info[i].sat_id = msg->sat[i].svId;
        gps_info_msg.sattelite_info[i].cno = msg->sat[i].cno;
    }

    switch (DID)
    {
    case DID_GPS1_SAT:  rs_.gps1_info.pub.publish(gps_info_msg);    break;
    case DID_GPS2_SAT:  rs_.gps2_info.pub.publish(gps_info_msg);    break;
    }
}

void InertialSenseROS::mag_callback(eDataIDs DID, const magnetometer_t *const msg)
{
    if (DID != DID_MAGNETOMETER)
    {
        return;
    }

    STREAMING_CHECK(rs_.mag.streaming, DID);
    sensor_msgs::MagneticField mag_msg;
    mag_msg.header.stamp = ros_time_from_start_time(msg->time);
    mag_msg.header.frame_id = frame_id_;
    mag_msg.magnetic_field.x = msg->mag[0];
    mag_msg.magnetic_field.y = msg->mag[1];
    mag_msg.magnetic_field.z = msg->mag[2];
    rs_.mag.pub.publish(mag_msg);
}

void InertialSenseROS::baro_callback(eDataIDs DID, const barometer_t *const msg)
{
    if (DID != DID_BAROMETER)
    {
        return;
    }

    STREAMING_CHECK(rs_.baro.streaming, DID);
    sensor_msgs::FluidPressure baro_msg;
    baro_msg.header.stamp = ros_time_from_start_time(msg->time);
    baro_msg.header.frame_id = frame_id_;
    baro_msg.fluid_pressure = msg->bar;
    baro_msg.variance = msg->barTemp;
    rs_.baro.pub.publish(baro_msg);
}

void InertialSenseROS::preint_IMU_callback(eDataIDs DID, const pimu_t *const msg)
{
    if (rs_.pimu.enabled)
    {
        STREAMING_CHECK(rs_.pimu.streaming, DID);
        preintIMU_msg.header.stamp = ros_time_from_start_time(msg->time);
        preintIMU_msg.header.frame_id = frame_id_;
        preintIMU_msg.dtheta.x = msg->theta[0];
        preintIMU_msg.dtheta.y = msg->theta[1];
        preintIMU_msg.dtheta.z = msg->theta[2];
        preintIMU_msg.dvel.x = msg->vel[0];
        preintIMU_msg.dvel.y = msg->vel[1];
        preintIMU_msg.dvel.z = msg->vel[2];
        preintIMU_msg.dt = msg->dt;
        rs_.pimu.pub.publish(preintIMU_msg);
    }

    if (rs_.imu.enabled)
    {
        STREAMING_CHECK(rs_.imu.streaming, DID);
        imu_msg.header.stamp = ros_time_from_start_time(msg->time);
        imu_msg.header.frame_id = frame_id_;
        imu_msg.angular_velocity.x = msg->theta[0] /msg->dt;
        imu_msg.angular_velocity.y = msg->theta[1] /msg->dt;
        imu_msg.angular_velocity.z = msg->theta[2] /msg->dt;
        imu_msg.linear_acceleration.x = msg->vel[0]/msg->dt;
        imu_msg.linear_acceleration.y = msg->vel[1]/msg->dt;
        imu_msg.linear_acceleration.z = msg->vel[2]/msg->dt;
        rs_.imu.pub.publish(imu_msg);
    }
}

void InertialSenseROS::RTK_Misc_callback(eDataIDs DID, const gps_rtk_misc_t *const msg)
{
    inertial_sense_ros::RTKInfo rtk_info;
    if (abs(GPS_towOffset_) > 0.001)
    {
        rtk_info.header.stamp = ros_time_from_week_and_tow(GPS_week_, msg->timeOfWeekMs / 1000.0);
        rtk_info.baseAntcount = msg->baseAntennaCount;
        rtk_info.baseEph = msg->baseBeidouEphemerisCount + msg->baseGalileoEphemerisCount + msg->baseGlonassEphemerisCount + msg->baseGpsEphemerisCount;
        rtk_info.baseObs = msg->baseBeidouObservationCount + msg->baseGalileoObservationCount + msg->baseGlonassObservationCount + msg->baseGpsObservationCount;
        rtk_info.BaseLLA[0] = msg->baseLla[0];
        rtk_info.BaseLLA[1] = msg->baseLla[1];
        rtk_info.BaseLLA[2] = msg->baseLla[2];

        rtk_info.roverEph = msg->roverBeidouEphemerisCount + msg->roverGalileoEphemerisCount + msg->roverGlonassEphemerisCount + msg->roverGpsEphemerisCount;
        rtk_info.roverObs = msg->roverBeidouObservationCount + msg->roverGalileoObservationCount + msg->roverGlonassObservationCount + msg->roverGpsObservationCount;
        rtk_info.cycle_slip_count = msg->cycleSlipCount;
    }

    switch (DID)
    {
    case DID_GPS1_RTK_POS_MISC:
        STREAMING_CHECK(rtkPosMiscStreaming_, DID);
        rs_.rtk_pos.pubInfo.publish(rtk_info);
        break;

    case DID_GPS2_RTK_CMP_MISC:
        STREAMING_CHECK(rtkCmpMiscStreaming_, DID);
        rs_.rtk_cmp.pubInfo.publish(rtk_info);
        break;
    }
}

void InertialSenseROS::RTK_Rel_callback(eDataIDs DID, const gps_rtk_rel_t *const msg)
{
    inertial_sense_ros::RTKRel rtk_rel;
    if (abs(GPS_towOffset_) > 0.001)
    {
        rtk_rel.header.stamp = ros_time_from_week_and_tow(GPS_week_, msg->timeOfWeekMs / 1000.0);
        rtk_rel.differential_age = msg->differentialAge;
        rtk_rel.ar_ratio = msg->arRatio;
        uint32_t fixStatus = msg->status & GPS_STATUS_FIX_MASK;
        if (fixStatus == GPS_STATUS_FIX_3D)
        {
            rtk_rel.eGpsNavFixStatus = inertial_sense_ros::RTKRel::GPS_STATUS_FIX_3D;
        }
        else if (fixStatus == GPS_STATUS_FIX_RTK_SINGLE)
        {
            rtk_rel.eGpsNavFixStatus = inertial_sense_ros::RTKRel::GPS_STATUS_FIX_RTK_SINGLE;
        }
        else if (fixStatus == GPS_STATUS_FIX_RTK_FLOAT)
        {
            rtk_rel.eGpsNavFixStatus = inertial_sense_ros::RTKRel::GPS_STATUS_FIX_RTK_FLOAT;
        }
        else if (fixStatus == GPS_STATUS_FIX_RTK_FIX)
        {
            rtk_rel.eGpsNavFixStatus = inertial_sense_ros::RTKRel::GPS_STATUS_FIX_RTK_FIX;
        }
        else if (msg->status & GPS_STATUS_FLAGS_RTK_FIX_AND_HOLD)
        {
            rtk_rel.eGpsNavFixStatus = inertial_sense_ros::RTKRel::GPS_STATUS_FLAGS_RTK_FIX_AND_HOLD;
        }

        rtk_rel.vector_base_to_rover.x = msg->baseToRoverVector[0];
        rtk_rel.vector_base_to_rover.y = msg->baseToRoverVector[1];
        rtk_rel.vector_base_to_rover.z = msg->baseToRoverVector[2];
        rtk_rel.distance_base_to_rover = msg->baseToRoverDistance;
        rtk_rel.heading_base_to_rover = msg->baseToRoverHeading;
    }

    switch (DID)
    {
    case DID_GPS1_RTK_POS_REL:
        STREAMING_CHECK(rtkPosRelStreaming_, DID);
        rs_.rtk_pos.pubRel.publish(rtk_rel);
        break;

    case DID_GPS2_RTK_CMP_REL:
        STREAMING_CHECK(rtkCmpRelStreaming_, DID);
        rs_.rtk_cmp.pubRel.publish(rtk_rel);
        break;
    }

    // save for diagnostics TODO - Add more diagnostic info
    diagnostic_ar_ratio_ = rtk_rel.ar_ratio;
    diagnostic_differential_age_ = rtk_rel.differential_age;
    diagnostic_heading_base_to_rover_ = rtk_rel.heading_base_to_rover;
    diagnostic_fix_type_ = rtk_rel.eGpsNavFixStatus;
}

void InertialSenseROS::GPS_raw_callback(eDataIDs DID, const gps_raw_t *const msg)
{
    switch (DID)
    {
    case DID_GPS1_RAW:        STREAMING_CHECK(rs_.gps1_raw.streaming, DID);     break;
    case DID_GPS2_RAW:        STREAMING_CHECK(rs_.gps2_raw.streaming, DID);     break;
    case DID_GPS_BASE_RAW:    STREAMING_CHECK(rs_.gpsbase_raw.streaming, DID); break;
    }

    switch (msg->dataType)
    {
    case raw_data_type_observation:
        GPS_obs_callback(DID, (obsd_t *)&msg->data.obs, msg->obsCount);
        break;

    case raw_data_type_ephemeris:
        GPS_eph_callback(DID, (eph_t *)&msg->data.eph);
        break;

    case raw_data_type_glonass_ephemeris:
        GPS_geph_callback(DID, (geph_t *)&msg->data.gloEph);
        break;

    default:
        break;
    }
}

void InertialSenseROS::GPS_obs_callback(eDataIDs DID, const obsd_t *const msg, int nObs)
{
    switch (DID)
    {
    case DID_GPS1_RAW:
        if (gps1_obs_Vec_.obs.size() > 0 &&
            (msg[0].time.time != gps1_obs_Vec_.obs[0].time.time ||
            msg[0].time.sec != gps1_obs_Vec_.obs[0].time.sec))
        {
            GPS_obs_bundle_timer_callback(ros::TimerEvent());
        }
        break;

    case DID_GPS2_RAW:
        if (gps2_obs_Vec_.obs.size() > 0 &&
            (msg[0].time.time != gps2_obs_Vec_.obs[0].time.time ||
            msg[0].time.sec != gps2_obs_Vec_.obs[0].time.sec))
        {
            GPS_obs_bundle_timer_callback(ros::TimerEvent());
        }
        break;

    case DID_GPS_BASE_RAW:
       if (base_obs_Vec_.obs.size() > 0 &&
            (msg[0].time.time != base_obs_Vec_.obs[0].time.time ||
            msg[0].time.sec != base_obs_Vec_.obs[0].time.sec))
        {
            GPS_obs_bundle_timer_callback(ros::TimerEvent());
        }
        break;
    }

    for (int i = 0; i < nObs; i++)
    {
        inertial_sense_ros::GNSSObservation obs;
        obs.header.stamp = ros_time_from_gtime(msg[i].time.time, msg[i].time.sec);
        obs.time.time = msg[i].time.time;
        obs.time.sec = msg[i].time.sec;
        obs.sat = msg[i].sat;
        obs.rcv = msg[i].rcv;
        obs.SNR = msg[i].SNR[0];
        obs.LLI = msg[i].LLI[0];
        obs.code = msg[i].code[0];
        obs.qualL = msg[i].qualL[0];
        obs.qualP = msg[i].qualP[0];
        obs.L = msg[i].L[0];
        obs.P = msg[i].P[0];
        obs.D = msg[i].D[0];
        switch (DID)
        {
        case DID_GPS1_RAW:
            gps1_obs_Vec_.obs.push_back(obs);
            last_obs_time_1_ = ros::Time::now();
            break;
        case DID_GPS2_RAW:
            gps2_obs_Vec_.obs.push_back(obs);
            last_obs_time_2_ = ros::Time::now();
            break;
        case DID_GPS_BASE_RAW:
            base_obs_Vec_.obs.push_back(obs);
            last_obs_time_base_ = ros::Time::now();
            break;
        }
    }
}

void InertialSenseROS::GPS_obs_bundle_timer_callback(const ros::TimerEvent &e)
{
    if (gps1_obs_Vec_.obs.size() != 0)
    {
        if (abs((ros::Time::now() - last_obs_time_1_).toSec()) > 1e-2)
        {
            gps1_obs_Vec_.header.stamp = ros_time_from_gtime(gps1_obs_Vec_.obs[0].time.time, gps1_obs_Vec_.obs[0].time.sec);
            gps1_obs_Vec_.time = gps1_obs_Vec_.obs[0].time;
            rs_.gps1_raw.pubObs.publish(gps1_obs_Vec_);
            gps1_obs_Vec_.obs.clear();
        }
    }
    if (gps2_obs_Vec_.obs.size() != 0)
    {
        if (abs((ros::Time::now() - last_obs_time_2_).toSec()) > 1e-2)
        {
            gps2_obs_Vec_.header.stamp = ros_time_from_gtime(gps2_obs_Vec_.obs[0].time.time, gps2_obs_Vec_.obs[0].time.sec);
            gps2_obs_Vec_.time = gps2_obs_Vec_.obs[0].time;
            rs_.gps2_raw.pubObs.publish(gps2_obs_Vec_);
            gps2_obs_Vec_.obs.clear();
        }
    }
    if (base_obs_Vec_.obs.size() != 0)
    {
        if (abs((ros::Time::now() - last_obs_time_base_).toSec()) > 1e-2)
        {
            base_obs_Vec_.header.stamp = ros_time_from_gtime(base_obs_Vec_.obs[0].time.time, base_obs_Vec_.obs[0].time.sec);
            base_obs_Vec_.time = base_obs_Vec_.obs[0].time;
            rs_.gpsbase_raw.pubObs.publish(base_obs_Vec_);
            base_obs_Vec_.obs.clear();
        }
    }
}

void InertialSenseROS::GPS_eph_callback(eDataIDs DID, const eph_t *const msg)
{
    inertial_sense_ros::GNSSEphemeris eph;
    eph.sat = msg->sat;
    eph.iode = msg->iode;
    eph.iodc = msg->iodc;
    eph.sva = msg->sva;
    eph.svh = msg->svh;
    eph.week = msg->week;
    eph.code = msg->code;
    eph.flag = msg->flag;
    eph.toe.time = msg->toe.time;
    eph.toc.time = msg->toc.time;
    eph.ttr.time = msg->ttr.time;
    eph.toe.sec = msg->toe.sec;
    eph.toc.sec = msg->toc.sec;
    eph.ttr.sec = msg->ttr.sec;
    eph.A = msg->A;
    eph.e = msg->e;
    eph.i0 = msg->i0;
    eph.OMG0 = msg->OMG0;
    eph.omg = msg->omg;
    eph.M0 = msg->M0;
    eph.deln = msg->deln;
    eph.OMGd = msg->OMGd;
    eph.idot = msg->idot;
    eph.crc = msg->crc;
    eph.crs = msg->crs;
    eph.cuc = msg->cuc;
    eph.cus = msg->cus;
    eph.cic = msg->cic;
    eph.cis = msg->cis;
    eph.toes = msg->toes;
    eph.fit = msg->fit;
    eph.f0 = msg->f0;
    eph.f1 = msg->f1;
    eph.f2 = msg->f2;
    eph.tgd[0] = msg->tgd[0];
    eph.tgd[1] = msg->tgd[1];
    eph.tgd[2] = msg->tgd[2];
    eph.tgd[3] = msg->tgd[3];
    eph.Adot = msg->Adot;
    eph.ndot = msg->ndot;
    switch (DID)
    { 
    case DID_GPS1_RAW:      rs_.gps1_raw.pubEph.publish(eph);        break;
    case DID_GPS2_RAW:      rs_.gps2_raw.pubEph.publish(eph);        break;
    case DID_GPS_BASE_RAW:  rs_.gpsbase_raw.pubEph.publish(eph);    break;
    }
}

void InertialSenseROS::GPS_geph_callback(eDataIDs DID, const geph_t *const msg)
{
    inertial_sense_ros::GlonassEphemeris geph;
    geph.sat = msg->sat;
    geph.iode = msg->iode;
    geph.frq = msg->frq;
    geph.svh = msg->svh;
    geph.sva = msg->sva;
    geph.age = msg->age;
    geph.toe.time = msg->toe.time;
    geph.tof.time = msg->tof.time;
    geph.toe.sec = msg->toe.sec;
    geph.tof.sec = msg->tof.sec;
    geph.pos[0] = msg->pos[0];
    geph.pos[1] = msg->pos[1];
    geph.pos[2] = msg->pos[2];
    geph.vel[0] = msg->vel[0];
    geph.vel[1] = msg->vel[1];
    geph.vel[2] = msg->vel[2];
    geph.acc[0] = msg->acc[0];
    geph.acc[1] = msg->acc[1];
    geph.acc[2] = msg->acc[2];
    geph.taun = msg->taun;
    geph.gamn = msg->gamn;
    geph.dtaun = msg->dtaun;
    switch (DID)
    { 
    case DID_GPS1_RAW:      rs_.gps1_raw.pubGEp.publish(geph);       break;
    case DID_GPS2_RAW:      rs_.gps2_raw.pubGEp.publish(geph);       break;
    case DID_GPS_BASE_RAW:  rs_.gpsbase_raw.pubGEp.publish(geph);   break;
    }
}

void InertialSenseROS::diagnostics_callback(const ros::TimerEvent &event)
{
    if (!diagnosticsStreaming_)
        ROS_INFO("Diagnostics response received");
    diagnosticsStreaming_ = true;
    // Create diagnostic objects
    diagnostic_msgs::DiagnosticArray diag_array;
    diag_array.header.stamp = ros::Time::now();

    // CNO mean
    diagnostic_msgs::DiagnosticStatus cno_mean;
    cno_mean.name = "CNO Mean";
    cno_mean.level = diagnostic_msgs::DiagnosticStatus::OK;
    cno_mean.message = std::to_string(gps1_msg.cno);
    diag_array.status.push_back(cno_mean);

    if (rs_.rtk_pos.enabled)
    {
        diagnostic_msgs::DiagnosticStatus rtk_status;
        rtk_status.name = "RTK";
        rtk_status.level = diagnostic_msgs::DiagnosticStatus::OK;
        std::string rtk_message;

        // AR ratio
        diagnostic_msgs::KeyValue ar_ratio;
        ar_ratio.key = "AR Ratio";
        ar_ratio.value = std::to_string(diagnostic_ar_ratio_);
        rtk_status.values.push_back(ar_ratio);
        if (diagnostic_fix_type_ == inertial_sense_ros::RTKRel::GPS_STATUS_FIX_3D)
        {
            rtk_status.level = diagnostic_msgs::DiagnosticStatus::WARN;
            rtk_message = "3D: " + std::to_string(diagnostic_ar_ratio_);
        }
        else if (diagnostic_fix_type_ == inertial_sense_ros::RTKRel::GPS_STATUS_FIX_RTK_SINGLE)
        {
            rtk_status.level = diagnostic_msgs::DiagnosticStatus::WARN;
            rtk_message = "Single: " + std::to_string(diagnostic_ar_ratio_);
        }
        else if (diagnostic_fix_type_ == inertial_sense_ros::RTKRel::GPS_STATUS_FIX_RTK_FLOAT)
        {
            rtk_message = "Float: " + std::to_string(diagnostic_ar_ratio_);
        }
        else if (diagnostic_fix_type_ == inertial_sense_ros::RTKRel::GPS_STATUS_FIX_RTK_FIX)
        {
            rtk_message = "Fix: " + std::to_string(diagnostic_ar_ratio_);
        }
        else if (diagnostic_fix_type_ == inertial_sense_ros::RTKRel::GPS_STATUS_FLAGS_RTK_FIX_AND_HOLD)
        {
            rtk_message = "Fix and Hold: " + std::to_string(diagnostic_ar_ratio_);
        }
        else
        {
            rtk_message = "Unknown Fix: " + std::to_string(diagnostic_ar_ratio_);
        }

        // Differential age
        diagnostic_msgs::KeyValue differential_age;
        differential_age.key = "Differential Age";
        differential_age.value = std::to_string(diagnostic_differential_age_);
        rtk_status.values.push_back(differential_age);
        if (diagnostic_differential_age_ > 1.5)
        {
            rtk_status.level = diagnostic_msgs::DiagnosticStatus::WARN;
            rtk_message += " Differential Age Large";
        }

        // Heading base to rover
        diagnostic_msgs::KeyValue heading_base_to_rover;
        heading_base_to_rover.key = "Heading Base to Rover (rad)";
        heading_base_to_rover.value = std::to_string(diagnostic_heading_base_to_rover_);
        rtk_status.values.push_back(heading_base_to_rover);

        rtk_status.message = rtk_message;
        diag_array.status.push_back(rtk_status);
    }

    rs_.diagnostics.pub.publish(diag_array);
}

bool InertialSenseROS::set_current_position_as_refLLA(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
    (void)req;
    double current_lla_[3];
    current_lla_[0] = lla_[0];
    current_lla_[1] = lla_[1];
    current_lla_[2] = lla_[2];

    IS_.SendData(DID_FLASH_CONFIG, reinterpret_cast<uint8_t *>(&current_lla_), sizeof(current_lla_), offsetof(nvm_flash_cfg_t, refLla));

    comManagerGetData(0, DID_FLASH_CONFIG, 0, 0, 1);

    int i = 0;
    nvm_flash_cfg_t current_flash = IS_.GetFlashConfig();
    while (current_flash.refLla[0] == IS_.GetFlashConfig().refLla[0] && current_flash.refLla[1] == IS_.GetFlashConfig().refLla[1] && current_flash.refLla[2] == IS_.GetFlashConfig().refLla[2])
    {
        comManagerStep();
        i++;
        if (i > 100)
        {
            break;
        }
    }

    if (current_lla_[0] == IS_.GetFlashConfig().refLla[0] && current_lla_[1] == IS_.GetFlashConfig().refLla[1] && current_lla_[2] == IS_.GetFlashConfig().refLla[2])
    {
        comManagerGetData(0, DID_FLASH_CONFIG, 0, 0, 0);
        res.success = true;
        res.message = ("Update was succesful.  refLla: Lat: " + std::to_string(current_lla_[0]) + "  Lon: " + std::to_string(current_lla_[1]) + "  Alt: " + std::to_string(current_lla_[2]));
    }
    else
    {
        comManagerGetData(0, DID_FLASH_CONFIG, 0, 0, 0);
        res.success = false;
        res.message = "Unable to update refLLA. Please try again.";
    }

    return true;
}

bool InertialSenseROS::set_refLLA_to_value(inertial_sense_ros::refLLAUpdate::Request &req, inertial_sense_ros::refLLAUpdate::Response &res)
{
    IS_.SendData(DID_FLASH_CONFIG, reinterpret_cast<uint8_t *>(&req.lla), sizeof(req.lla), offsetof(nvm_flash_cfg_t, refLla));

    comManagerGetData(0, DID_FLASH_CONFIG, 0, 0, 1);

    int i = 0;
    nvm_flash_cfg_t current_flash = IS_.GetFlashConfig();
    while (current_flash.refLla[0] == IS_.GetFlashConfig().refLla[0] && current_flash.refLla[1] == IS_.GetFlashConfig().refLla[1] && current_flash.refLla[2] == IS_.GetFlashConfig().refLla[2])
    {
        comManagerStep();
        i++;
        if (i > 100)
        {
            break;
        }
    }

    if (req.lla[0] == IS_.GetFlashConfig().refLla[0] && req.lla[1] == IS_.GetFlashConfig().refLla[1] && req.lla[2] == IS_.GetFlashConfig().refLla[2])
    {
        comManagerGetData(0, DID_FLASH_CONFIG, 0, 0, 0);
        res.success = true;
        res.message = ("Update was succesful.  refLla: Lat: " + std::to_string(req.lla[0]) + "  Lon: " + std::to_string(req.lla[1]) + "  Alt: " + std::to_string(req.lla[2]));
    }
    else
    {
        comManagerGetData(0, DID_FLASH_CONFIG, 0, 0, 0);
        res.success = false;
        res.message = "Unable to update refLLA. Please try again.";
    }

    return true;
}

bool InertialSenseROS::perform_mag_cal_srv_callback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
    (void)req;
    uint32_t single_axis_command = 2;
    IS_.SendData(DID_MAG_CAL, reinterpret_cast<uint8_t *>(&single_axis_command), sizeof(uint32_t), offsetof(mag_cal_t, state));

    is_comm_instance_t comm;
    uint8_t buffer[2048];
    is_comm_init(&comm, buffer, sizeof(buffer));
    serial_port_t *serialPort = IS_.GetSerialPort();
    uint8_t inByte;
    int n;

    while ((n = serialPortReadCharTimeout(serialPort, &inByte, 20)) > 0)
    {
        // Search comm buffer for valid packets
        if (is_comm_parse_byte(&comm, inByte) == _PTYPE_INERTIAL_SENSE_DATA && comm.dataHdr.id == DID_INS_1)
        {
            ins_1_t *msg = (ins_1_t *)(comm.dataPtr + comm.dataHdr.offset);
            if (msg->insStatus & 0x00400000)
            {
                res.success = true;
                res.message = "Successfully initiated mag recalibration.";
                return true;
            }
        }
    }

    return true;
}

bool InertialSenseROS::perform_multi_mag_cal_srv_callback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
    (void)req;
    uint32_t multi_axis_command = 1;
    IS_.SendData(DID_MAG_CAL, reinterpret_cast<uint8_t *>(&multi_axis_command), sizeof(uint32_t), offsetof(mag_cal_t, state));

    is_comm_instance_t comm;
    uint8_t buffer[2048];
    is_comm_init(&comm, buffer, sizeof(buffer));
    serial_port_t *serialPort = IS_.GetSerialPort();
    uint8_t inByte;
    int n;

    while ((n = serialPortReadCharTimeout(serialPort, &inByte, 20)) > 0)
    {
        // Search comm buffer for valid packets
        if (is_comm_parse_byte(&comm, inByte) == _PTYPE_INERTIAL_SENSE_DATA && comm.dataHdr.id == DID_INS_1)
        {
            ins_1_t *msg = (ins_1_t *)(comm.dataPtr + comm.dataHdr.offset);
            if (msg->insStatus & 0x00400000)
            {
                res.success = true;
                res.message = "Successfully initiated mag recalibration.";
                return true;
            }
        }
    }

    return true;
}

void InertialSenseROS::reset_device()
{
    // send reset command
    system_command_t reset_command;
    reset_command.command = 99;
    reset_command.invCommand = ~reset_command.command;
    IS_.SendData(DID_SYS_CMD, reinterpret_cast<uint8_t *>(&reset_command), sizeof(system_command_t), 0);
    ROS_WARN("Device reset required.\n\nShutting down Node.\n");
    ros::shutdown();
}

bool InertialSenseROS::update_firmware_srv_callback(inertial_sense_ros::FirmwareUpdate::Request &req, inertial_sense_ros::FirmwareUpdate::Response &res)
{
    //   IS_.Close();
    //   vector<InertialSense::bootload_result_t> results = IS_.BootloadFile("*", req.filename, 921600);
    //   if (!results[0].error.empty())
    //   {
    //     res.success = false;
    //     res.message = results[0].error;
    //     return false;
    //   }
    //   IS_.Open(port_.c_str(), baudrate_);

    return true;
}

ros::Time InertialSenseROS::ros_time_from_week_and_tow(const uint32_t week, const double timeOfWeek)
{
    ros::Time rostime(0, 0);
    //  If we have a GPS fix, then use it to set timestamp
    if (abs(GPS_towOffset_) > 0.001)
    {
        uint64_t sec = UNIX_TO_GPS_OFFSET + floor(timeOfWeek) + week * 7 * 24 * 3600;
        uint64_t nsec = (timeOfWeek - floor(timeOfWeek)) * 1e9;
        rostime = ros::Time(sec, nsec);
    }
    else
    {
        // Otherwise, estimate the uINS boot time and offset the messages
        if (!got_first_message_)
        {
            got_first_message_ = true;
            INS_local_offset_ = ros::Time::now().toSec() - timeOfWeek;
        }
        else // low-pass filter offset to account for drift
        {
            double y_offset = ros::Time::now().toSec() - timeOfWeek;
            INS_local_offset_ = 0.005 * y_offset + 0.995 * INS_local_offset_;
        }
        // Publish with ROS time
        rostime = ros::Time(INS_local_offset_ + timeOfWeek);
    }
    return rostime;
}

ros::Time InertialSenseROS::ros_time_from_start_time(const double time)
{
    ros::Time rostime(0, 0);

    //  If we have a GPS fix, then use it to set timestamp
    if (abs(GPS_towOffset_) > 0.001)
    {
        double timeOfWeek = time + GPS_towOffset_;
        uint64_t sec = (uint64_t)(UNIX_TO_GPS_OFFSET + floor(timeOfWeek) + GPS_week_ * 7 * 24 * 3600);
        uint64_t nsec = (uint64_t)((timeOfWeek - floor(timeOfWeek)) * 1.0e9);
        rostime = ros::Time(sec, nsec);
    }
    else
    {
        // Otherwise, estimate the uINS boot time and offset the messages
        if (!got_first_message_)
        {
            got_first_message_ = true;
            INS_local_offset_ = ros::Time::now().toSec() - time;
        }
        else // low-pass filter offset to account for drift
        {
            double y_offset = ros::Time::now().toSec() - time;
            INS_local_offset_ = 0.005 * y_offset + 0.995 * INS_local_offset_;
        }
        // Publish with ROS time
        rostime = ros::Time(INS_local_offset_ + time);
    }
    return rostime;
}

ros::Time InertialSenseROS::ros_time_from_tow(const double tow)
{
    return ros_time_from_week_and_tow(GPS_week_, tow);
}

double InertialSenseROS::tow_from_ros_time(const ros::Time &rt)
{
    return (rt.sec - UNIX_TO_GPS_OFFSET - GPS_week_ * 604800) + rt.nsec * 1.0e-9;
}

ros::Time InertialSenseROS::ros_time_from_gtime(const uint64_t sec, double subsec)
{
    ros::Time out;
    out.sec = sec - LEAP_SECONDS;
    out.nsec = subsec * 1e9;
    return out;
}


void InertialSenseROS::LD2Cov(const float *LD, float *Cov, int width)
{
    for (int j = 0; j < width; j++) {
        for (int i = 0; i < width; i++) {
            if (i < j) {
                Cov[i * width + j] = Cov[j * width + i];
            }
            else {
                Cov[i * width + j] = LD[(i * i + i) / 2 + j];
            }
        }
    }
}

void InertialSenseROS::rotMatB2R(const ixVector4 quat, ixMatrix3 R)
{
    R[0] = 1.0f - 2.0f * (quat[2]*quat[2] + quat[3]*quat[3]);
    R[1] =        2.0f * (quat[1]*quat[2] - quat[0]*quat[3]);
    R[2] =        2.0f * (quat[1]*quat[3] + quat[0]*quat[2]);
    R[3] =        2.0f * (quat[1]*quat[2] + quat[0]*quat[3]);
    R[4] = 1.0f - 2.0f * (quat[1]*quat[1] + quat[3]*quat[3]);
    R[5] =        2.0f * (quat[2]*quat[3] - quat[0]*quat[1]);
    R[6] =        2.0f * (quat[1]*quat[3] - quat[0]*quat[2]);
    R[7] =        2.0f * (quat[2]*quat[3] + quat[0]*quat[1]);
    R[8] = 1.0f - 2.0f * (quat[1]*quat[1] + quat[2]*quat[2]);
}

void InertialSenseROS::transform_6x6_covariance(float Pout[36], float Pin[36], ixMatrix3 R1, ixMatrix3 R2)
{
    // Assumption: input covariance matrix is transformed due to change of coordinates,
    // so that fisrt 3 coordinates are rotated by R1 and the last 3 coordinates are rotated by R2
    // This is how the transformation looks:
    // |R1  0 | * |Pxx  Pxy'| * |R1' 0  | = |R1*Pxx*R1'  R1*Pxy'*R2'|
    // |0   R2|   |Pxy  Pyy |   |0   R2'|   |R2*Pxy*R1'  R2*Pyy*R2' |

    ixMatrix3 Pxx_in, Pxy_in, Pyy_in, Pxx_out, Pxy_out, Pyy_out, buf;

    // Extract 3x3 blocks from input covariance
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            // Upper diagonal block in old frame
            Pxx_in[i * 3 + j] = Pin[i * 6 + j];
            // Lower left block of in old frame
            Pxy_in[i * 3 + j] = Pin[(i+3) * 6 + j];
            // Lower diagonal block in old frame
            Pyy_in[i * 3 + j] = Pin[(i+3) * 6 + j + 3];
        }
    }
    // Transform the 3x3 covariance blocks
    // New upper diagonal block
    mul_Mat3x3_Mat3x3(buf, R1, Pxx_in);
    mul_Mat3x3_Mat3x3_Trans(Pxx_out, buf, R1);
    // New lower left block
    mul_Mat3x3_Mat3x3(buf, R2, Pxy_in);
    mul_Mat3x3_Mat3x3_Trans(Pxy_out, buf, R1);
    // New lower diagonal  block
    mul_Mat3x3_Mat3x3(buf, R2, Pyy_in);
    mul_Mat3x3_Mat3x3_Trans(Pyy_out, buf, R2);

    // Copy the computed transformed blocks into output 6x6 covariance matrix
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            // Upper diagonal block in the new frame
            Pout[i * 6 + j] = Pxx_in[i * 3 + j];
            // Lower left block in the new frame
            Pout[(i+3) * 6 + j] = Pxy_in[i * 3 + j];
            // Upper right block in the new frame
            Pout[i * 6 + j + 3] = Pxy_in[j * 3 + i];
            // Lower diagonal block in the new frame
            Pout[(i+3) * 6 + j + 3] = Pyy_in[i * 3 + j];
        }
    }
}

#define PARAM_PRINT_USER    "            "
#define PARAM_PRINT_DEFAULT "  (default) "

template <typename Type>
bool InertialSenseROS::get_node_param_yaml(YAML::Node node, const std::string key, Type &val)
{
    bool success = false;

    if (node[key])
    {
        try
        {
            val = node[key].as<Type>();
            success = true;
        }
        catch (const YAML::KeyNotFound &knf)
        {
            // std::cout << "get_node_param_yaml(): Key \"" + key + "\" not found.  Using default value.\n";
        }
    }
    else
    {
        // std::cout << "get_node_param_yaml(): Key \"" + key + "\" not in yaml node.  Using default value.\n";
    }

    // Display parameter
    if (success)
    {
        std::cout << PARAM_PRINT_USER;
    }
    else
    {
        std::cout << PARAM_PRINT_DEFAULT;
    }
    std::cout << key + ": " << val << "\n";

    return success;
}

template <typename Derived1>
bool InertialSenseROS::get_node_vector_yaml(YAML::Node node, const std::string key, int size, Derived1 &val)
{
    bool success = false;

    if (node[key])
    {
        try
        {
            std::vector<double> vec;
            vec = node[key].as<std::vector<double>>();
            for (int i = 0; i < size; i++)
            {
                val[i] = vec[i];
            }
            success = true;
        }
        catch (...)
        {
            // std::cout << "get_node_param_yaml(): Key \"" + key + "\" not found.  Using default value.\n";
        }
    }
    else
    {
        // std::cout << "get_node_param_yaml(): Key \"" + key + "\" not in yaml node.  Using default value.\n";
    }

    // Display parameter
    if (success)
    {
        std::cout << PARAM_PRINT_USER;
    }
    else
    {
        std::cout << PARAM_PRINT_DEFAULT;
    }
    std::cout << key + ": [";
    for (int i = 0; i < size; i++)
    {
        std::cout << val[i] << ((i<size-1) ? "," : "");
    }
    std::cout << "]\n";

    return success;
}

bool InertialSenseROS::getParam(const std::string &key, std::string &s)
{
    bool success = nh_private_.getParam(key, s);

    // Display parameter
    if (success)
    {
        std::cout << PARAM_PRINT_USER;
    }
    else
    {
        std::cout << PARAM_PRINT_DEFAULT;
    }
    std::cout << key + ": " << s << "\n";
    return  success;
}

bool InertialSenseROS::getParam(const std::string &key, double &d)
{
    bool success = nh_private_.getParam(key, d);

    // Display parameter
    if (success)
    {
        std::cout << PARAM_PRINT_USER;
    }
    else
    {
        std::cout << PARAM_PRINT_DEFAULT;
    }
    std::cout << key + ": " << d << "\n";
    return success;
}

bool InertialSenseROS::getParam(const std::string &key, float &f)
{
    bool success = nh_private_.getParam(key, f);

    // Display parameter
    if (success)
    {
        std::cout << PARAM_PRINT_USER;
    }
    else
    {
        std::cout << PARAM_PRINT_DEFAULT;
    }
    std::cout << key + ": " << f << "\n";
    return success;
}

bool InertialSenseROS::getParam(const std::string &key, int &i)
{
    bool success = nh_private_.getParam(key, i);

    // Display parameter
    if (success)
    {
        std::cout << PARAM_PRINT_USER;
    }
    else
    {
        std::cout << PARAM_PRINT_DEFAULT;
    }
    std::cout << key + ": " << i << "\n";
    return success;
}

bool InertialSenseROS::getParam(const std::string &key, bool &b)
{
    bool success = nh_private_.getParam(key, b);

    // Display parameter
    if (success)
    {
        std::cout << PARAM_PRINT_USER;
    }
    else
    {
        std::cout << PARAM_PRINT_DEFAULT;
    }
    std::cout << key + ": " << b << "\n";
    return success;
}

bool InertialSenseROS::getParam(const std::string &key, XmlRpc::XmlRpcValue &v)
{
    bool success = nh_private_.getParam(key, v);

    // Display parameter
    if (success)
    {
        std::cout << PARAM_PRINT_USER;
    }
    else
    {
        std::cout << PARAM_PRINT_DEFAULT;
    }
    std::cout << key + ": " << v << "\n";
    return success;
}

template <typename T>
bool InertialSenseROS::getParamVector(const std::string &key, uint32_t size, T &data)
{
    std::vector<double> vec(size, 0);
    bool success = nh_private_.getParam(key, vec);

    if (success)
    {
        for (int i = 0; i < size; i++)
        {
            data[i] = vec[i];
        }
        std::cout << PARAM_PRINT_USER;
    }
    else
    {
        std::cout << PARAM_PRINT_DEFAULT;
    }
    std::cout << key + ": [";
    for (int i = 0; i < size; i++)
    {
        std::cout << data[i] << ((i<size-1) ? "," : "");
    }
    std::cout << "]\n";

    return success;
}

