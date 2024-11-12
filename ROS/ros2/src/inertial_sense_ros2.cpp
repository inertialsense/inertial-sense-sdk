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

#include "inertial_sense_ros.h"

#include <chrono>
#include <stddef.h>
#include <unistd.h>
//#include <ISPose.h>
//#include <duration.hpp>
#include "ISEarth.h"
#include <memory>
#include <boost/chrono/duration.hpp>

//#include "ISMatrix.h"

#include "ParamHelper.h"

#define STREAMING_CHECK(streaming, DID)      if(!streaming){ streaming = true; rclcpp::Logger logger_IS_resp_rec = rclcpp::get_logger("IS_response_received"); logger_IS_resp_rec.set_level(rclcpp::Logger::Level::Debug); RCLCPP_DEBUG(logger_IS_resp_rec,"InertialSenseROS: %s response received", cISDataMappings::DataName(DID)); }
//#define STREAMING_CHECK(streaming, DID)      if(!streaming){ streaming = true; RCLCPP_DEBUG("InertialSenseROS: %s response received", cISDataMappings::DataName(DID)); }
//auto nh_ = std::make_shared<rclcpp::Node>("nh_");
/**
 * Assigns an identity to the passed ROS:nav_msgs::Odometry pose/twist covariance matrix
 * @param msg_odom - the nav_msgs::Odometry message to set the identity on.
 */
void odometryIdentity(nav_msgs::msg::Odometry& msg_odom) {
    for (int row = 0; row < 6; row++) {
        for (int col = 0; col < 6; col++) {
            msg_odom.pose.covariance[row * 6 + col] = (row == col ? 1 : 0);
            msg_odom.twist.covariance[row * 6 + col] = (row == col ? 1 : 0);
        }
    }
}

InertialSenseROS::InertialSenseROS(YAML::Node paramNode, bool configFlashParameters): nh_(rclcpp::Node::make_shared("nh_"))
{
    // Should always be enabled by default
    rs_.did_ins1.enabled = true;
    rs_.did_ins1.topic = "did_ins1";
    rs_.gps1.enabled = true;
    rs_.gps1.topic = "/gps";

   //if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
   //{
   //    ros::console::notifyLoggerLevelsChanged();
   //}
    load_params(paramNode);
}

void InertialSenseROS::initialize(bool configFlashParameters)
{
    RCLCPP_INFO(rclcpp::get_logger("start"),"======  Starting Inertial Sense ROS2  ======");

    initializeIS(true);
    if (sdk_connected_)
    {
        initializeROS();

        if (log_enabled_) 
        {
            start_log();    // Start log should happen last
        }

        // configure_ascii_output(); // Currently not functional
    }
}

void InertialSenseROS::terminate()
{
    IS_.Close();
    IS_.CloseServerConnection();
    sdk_connected_ = false;

    // ROS equivalent to shutdown advertisers, etc.
}

void InertialSenseROS::initializeIS(bool configFlashParameters)
{
    if (factory_reset_)
    {
        if (connect())
        {   // Apply factory reset
            RCLCPP_INFO(rclcpp::get_logger("factory_reset"), "InertialSenseROS2: Applying factory reset.");
            IS_.StopBroadcasts(true);
            IS_.SetSysCmd(SYS_CMD_MANF_UNLOCK);
            IS_.SetSysCmd(SYS_CMD_MANF_FACTORY_RESET);
            sleep(3);
        }
    }

    if (connect())
    {
        // Check protocol and firmware version
        firmware_compatiblity_check();

        IS_.StopBroadcasts(true);
        initializeROS();
        configure_data_streams(true);
        //configure_rtk();
        IS_.SavePersistent();

        if (configFlashParameters)
        {   // Set IMX flash parameters (flash write) after everything else so processor stall doesn't interfere with communications.
            //configure_flash_parameters();
        }
    }
}

void InertialSenseROS::initializeROS()
{
    //////////////////////////////////////////////////////////
    // Start Up ROS service servers
    refLLA_set_value_srv_           = nh_->create_service<inertial_sense_ros2::srv::RefLLAUpdate>("set_refLLA_value", std::bind(&InertialSenseROS::set_refLLA_to_value, this, std::placeholders::_1, std::placeholders::_2));
    mag_cal_srv_                    = nh_->create_service<std_srvs::srv::Trigger>("single_axis_mag_cal", std::bind(&InertialSenseROS::perform_mag_cal_srv_callback, this, std::placeholders::_1, std::placeholders::_2));
    multi_mag_cal_srv_              = nh_->create_service<std_srvs::srv::Trigger>("multi_axis_mag_cal", std::bind(&InertialSenseROS::perform_multi_mag_cal_srv_callback, this, std::placeholders::_1, std::placeholders::_2));
    //firmware_update_srv_            = nh_.advertiseService("firmware_update", &InertialSenseROS::update_firmware_srv_callback, this);

    SET_CALLBACK(DID_STROBE_IN_TIME, strobe_in_time_t, strobe_in_time_callback, 0); // we always want the strobe

    //////////////////////////////////////////////////////////
    // Publishers
    strobe_pub_ = nh_->create_publisher<std_msgs::msg::Header>(rs_.strobe_in.topic, 1);

    if (rs_.did_ins1.enabled)
        { rs_.did_ins1.pub_didins1    = nh_->create_publisher<inertial_sense_ros2::msg::DIDINS1>(rs_.did_ins1.topic, 1); }

    if (rs_.did_ins2.enabled)               { rs_.did_ins2.pub_didins2      = nh_->create_publisher<inertial_sense_ros2::msg::DIDINS2>(rs_.did_ins2.topic, 1); }
    if (rs_.did_ins4.enabled)               { rs_.did_ins4.pub_didins4      = nh_->create_publisher<inertial_sense_ros2::msg::DIDINS4>(rs_.did_ins4.topic, 1); }
    if (rs_.odom_ins_ned.enabled)           { rs_.odom_ins_ned.pub_odometry  = nh_->create_publisher<nav_msgs::msg::Odometry>(rs_.odom_ins_ned.topic, 1); }
    if (rs_.odom_ins_enu.enabled)           { rs_.odom_ins_enu.pub_odometry  = nh_->create_publisher<nav_msgs::msg::Odometry>(rs_.odom_ins_enu.topic, 1); }
    if (rs_.odom_ins_ecef.enabled)          { rs_.odom_ins_ecef.pub_odometry = nh_->create_publisher<nav_msgs::msg::Odometry>(rs_.odom_ins_ecef.topic, 1); }
    if (rs_.inl2_states.enabled)            { rs_.inl2_states.pub_inl2   = nh_->create_publisher<inertial_sense_ros2::msg::INL2States>(rs_.inl2_states.topic, 1); }

   if (rs_.pimu.enabled)                   { rs_.pimu.pub_pimu = nh_->create_publisher<inertial_sense_ros2::msg::PIMU>(rs_.pimu.topic, 1); }
   if (rs_.imu.enabled)                    { rs_.imu.pub_imu = nh_->create_publisher<sensor_msgs::msg::Imu>(rs_.imu.topic, 1); }
   if (rs_.magnetometer.enabled)           { rs_.magnetometer.pub_bfield = nh_->create_publisher<sensor_msgs::msg::MagneticField>(rs_.magnetometer.topic, 1); }
   if (rs_.barometer.enabled)              { rs_.barometer.pub_fpres = nh_->create_publisher<sensor_msgs::msg::FluidPressure>(rs_.barometer.topic, 1); }
   if (rs_.gps1.enabled)                   { rs_.gps1.pub_gps = nh_->create_publisher<inertial_sense_ros2::msg::GPS>(rs_.gps1.topic, 1); }
   if (rs_.gps1_navsatfix.enabled)         { rs_.gps1_navsatfix.pub_nsf = nh_->create_publisher<sensor_msgs::msg::NavSatFix>(rs_.gps1_navsatfix.topic, 1); }
   if (rs_.gps1_info.enabled)              { rs_.gps1_info.pub_gpsinfo1 = nh_->create_publisher<inertial_sense_ros2::msg::GPSInfo>(rs_.gps1_info.topic, 1); }
   if (rs_.gps2.enabled)                   { rs_.gps2.pub_gps = nh_->create_publisher<inertial_sense_ros2::msg::GPS>(rs_.gps2.topic, 1); }
   if (rs_.gps2_navsatfix.enabled)         { rs_.gps2_navsatfix.pub_nsf = nh_->create_publisher<sensor_msgs::msg::NavSatFix>(rs_.gps2_navsatfix.topic, 1); }
   if (rs_.gps2_info.enabled)              { rs_.gps2_info.pub_gpsinfo2 = nh_->create_publisher<inertial_sense_ros2::msg::GPSInfo>(rs_.gps2_info.topic, 1); }

    if (RTK_rover_ && RTK_rover_->positioning_enable )
    {
        rs_.rtk_pos.pubInfo = nh_->create_publisher<inertial_sense_ros2::msg::RTKInfo>("RTK_pos/info", 10);
        rs_.rtk_pos.pubRel = nh_->create_publisher<inertial_sense_ros2::msg::RTKRel>("RTK_pos/rel", 10);
    }
    if (GNSS_Compass_)
    {
        rs_.rtk_cmp.pubInfo = nh_->create_publisher<inertial_sense_ros2::msg::RTKInfo>("RTK_cmp/info", 10);
        rs_.rtk_cmp.pubRel = nh_->create_publisher<inertial_sense_ros2::msg::RTKRel>("RTK_cmp/rel", 10);
    }

    if (rs_.gps1_raw.enabled)
    {
        rs_.gps1_raw.pubObs = nh_->create_publisher<inertial_sense_ros2::msg::GNSSObsVec>(rs_.gps1_raw.topic + "/obs", 50);
        rs_.gps1_raw.pubEph = nh_->create_publisher<inertial_sense_ros2::msg::GNSSEphemeris>(rs_.gps1_raw.topic + "/eph", 50);
        rs_.gps1_raw.pubGEp = nh_->create_publisher<inertial_sense_ros2::msg::GlonassEphemeris>(rs_.gps1_raw.topic + "/geph", 50);
        //obs_bundle_timer_ = nh_->create_wall_timer(0.001s, std::bind(InertialSenseROS::GPS_obs_bundle_timer_callback, this));
        obs_bundle_timer_ = nh_->create_wall_timer(1s, [this]() { this->GPS_obs_bundle_timer_callback(); });
    }

    if (rs_.gps2_raw.enabled)
    {
        rs_.gps2_raw.pubObs = nh_->create_publisher<inertial_sense_ros2::msg::GNSSObsVec>(rs_.gps2_raw.topic + "/obs", 50);
        rs_.gps2_raw.pubEph = nh_->create_publisher<inertial_sense_ros2::msg::GNSSEphemeris>(rs_.gps2_raw.topic + "/eph", 50);
        rs_.gps2_raw.pubGEp = nh_->create_publisher<inertial_sense_ros2::msg::GlonassEphemeris>(rs_.gps2_raw.topic + "/geph", 50);
        obs_bundle_timer_ = nh_->create_wall_timer(1s, [this]() { this->GPS_obs_bundle_timer_callback(); });
    }

    if (rs_.gpsbase_raw.enabled)
    {
        rs_.gpsbase_raw.pubObs = nh_->create_publisher<inertial_sense_ros2::msg::GNSSObsVec>("gps/base_obs", 50);
        rs_.gpsbase_raw.pubEph = nh_->create_publisher<inertial_sense_ros2::msg::GNSSEphemeris>("gps/base_eph", 50);
        rs_.gpsbase_raw.pubGEp = nh_->create_publisher<inertial_sense_ros2::msg::GlonassEphemeris>("gps/base_geph", 50);
        obs_bundle_timer_ = nh_->create_wall_timer(1s, [this]() { this->GPS_obs_bundle_timer_callback(); });
    }

    if (rs_.diagnostics.enabled)
    {
        rs_.diagnostics.pub_diagnostics = nh_->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("diagnostics", 1);
        diagnostics_timer_ = nh_->create_timer(0.5s, std::bind(InertialSenseROS::diagnostics_callback, this)); // 2 Hz
    }

    data_stream_timer_ = nh_->create_wall_timer(1s, [this]() { this->configure_data_streams(false); });

}

void InertialSenseROS::load_params(YAML::Node &node)
{
    // Load parameters from yaml node if provided.  Otherwise load from ROS parameter server.
    bool useParamSvr = !node.IsDefined();
    //ros::NodeHandle nh;

    if (useParamSvr)
    {
        RCLCPP_INFO(rclcpp::get_logger("load_config_ros_param"), "InertialSenseROS: Loading configuration from ROS Parameter Server." );
        ParamHelper::paramServerToYamlNode(node, "/");
    }
    else
    {
        RCLCPP_INFO(rclcpp::get_logger("load_config_yaml"), "InertialSenseROS: Loading configuration from YAML tree." );
    }

    // Default values appear in the 3rd parameter
    // Order of function calls matters because ParamHelper::node() sets the current node accessed by param functions nodeParam(), nodeParamVec(), msgParams(). 

    // General parameters
    ParamHelper ph(node);

    YAML::Node portNode = node["port"];
    if (portNode.IsSequence()) {
        for (auto it = portNode.begin(); it != portNode.end(); it++)
            ports_.push_back((*it).as<std::string>());
    } else if (portNode.IsScalar()) {
        std::string param = nh_->declare_parameter<std::string>("port", "/dev/ttyACM0");
        ph.nodeParam("port", param, param);
        ports_.push_back(param);
    }

    if(ports_.size() < 1)
    {
        //No ports specified. Use default
        std::string param_1 = nh_->declare_parameter<std::string>("port_1", "/dev/ttyACM0");
        ph.nodeParam("port_1", param_1, param_1);
        ports_.push_back(param_1);
    }

    bool factory_reset = nh_->declare_parameter<bool>("factory_reset", false);
    ph.nodeParam("factory_reset", factory_reset_, factory_reset);

    int baud_rate = nh_->declare_parameter<int>("baudrate", 921600);
    ph.nodeParam("baudrate", baudrate_, baud_rate);


    std::string frame_id = nh_->declare_parameter<std::string>("frame_id", "body");
    ph.nodeParam("frame_id", frame_id_, frame_id);
    bool log_enabled = nh_->declare_parameter<bool>("enable_log", false);
    ph.nodeParam("enable_log", log_enabled_, log_enabled);


    // advanced Parameters
    int io_config_bits = nh_->declare_parameter<int>("io_config", 39624800);
    setIoConfigBits_ = ph.nodeParam("ioConfig", ioConfigBits_, io_config_bits);

    int rtk_config_bits = nh_->declare_parameter<int>("RTKCfgBits", 0);
    ph.nodeParam("RTKCfgBits", rtkConfigBits_, rtk_config_bits);            // rtk config bits

    int wheel_config_bits = nh_->declare_parameter<int>("wheelCfgBits", 0);
    ph.nodeParam("wheelCfgBits", wheelConfigBits_, wheel_config_bits);        // wheel-encoder config bits

    float mag_declination = nh_->declare_parameter<float>("mag_declination", 0.20007290992);
    ph.nodeParam("mag_declination", magDeclination_, mag_declination);

    std::vector<double> refLla_vector = nh_->declare_parameter<std::vector<double>>("ref_lla", {0.0, 0.0, 0.0});
    if (refLla_vector.size() == 3) {
        std::copy(refLla_vector.begin(), refLla_vector.end(), refLla_);
    }
    ph.nodeParamVec("ref_lla", 3, refLla_, refLla_);

    //ph.nodeParam("publishTf", publishTf_);

    int platform_config = nh_->declare_parameter<int>("platformConfig", 0);
    setPlatformConfig_ = ph.nodeParam("platformConfig", platformConfig_, platform_config);

    // Sensors
    YAML::Node sensorsNode = ph.node(node, "sensors");
    YAML::Node sensorsMsgs = ph.node(sensorsNode, "messages", 2);

    bool imu_enable = nh_->declare_parameter<bool>("msg/imu/enable", false);
    int imu_period = nh_->declare_parameter<int>("msg/imu/period", 1);
    ph.msgParams(rs_.imu, "imu", "", false, imu_period, imu_enable);

    bool pimu_enable = nh_->declare_parameter<bool>("msg/pimu/enable", false);
    int pimu_period = nh_->declare_parameter<int>("msg/pimu/period", 1);
    ph.msgParams(rs_.pimu, "pimu", "", false, pimu_period, pimu_enable);

    bool mag_enable = nh_->declare_parameter<bool>("msg/mag/enable", false);
    int mag_period = nh_->declare_parameter<int>("msg/mag/period", 1);
    ph.msgParams(rs_.magnetometer, "magnetometer", "mag", false, mag_period, mag_enable);

    bool baro_enable = nh_->declare_parameter<bool>("msg/baro/enable", false);
    int baro_period = nh_->declare_parameter<int>("msg/baro/period", 1);
    ph.msgParams(rs_.barometer, "barometer", "baro", false, baro_period, baro_enable);

    ph.msgParams(rs_.strobe_in, "strobe_in");
    node["sensors"]["messages"] = sensorsMsgs;

    // INS
    YAML::Node insNode = ph.node(node, "ins");

    std::vector<double> insRotation_vector = nh_->declare_parameter<std::vector<double>>("ins_rotation", {0.0, 0.0, 0.0});
    if (insRotation_vector.size() == 3) {
        std::copy(insRotation_vector.begin(), insRotation_vector.end(), insRotation_);
    }
    ph.nodeParamVec("rotation", 3, insRotation_, insRotation_);

    std::vector<double> insOffset_vector = nh_->declare_parameter<std::vector<double>>("ins_offset", {0.0, 0.0, 0.0});
    if (insOffset_vector.size() == 3) {
        std::copy(insOffset_vector.begin(), insOffset_vector.end(), insOffset_);
    }
    ph.nodeParamVec("offset", 3, insOffset_, insOffset_);

    float ins_nav_dt_ms = nh_->declare_parameter<float>("navigation_dt_ms", 8);
    ph.nodeParam("navigation_dt_ms", ins_nav_dt_ms_, ins_nav_dt_ms);

    std::vector<std::string> dyn_model_set{ "DYNAMIC_MODEL_PORTABLE",
                                            " << UNKNOWN >> ",
                                            "DYNAMIC_MODEL_STATIONARY",
                                            "DYNAMIC_MODEL_PEDESTRIAN",
                                            "DYNAMIC_MODEL_GROUND_VEHICLE",
                                            "DYNAMIC_MODEL_MARINE",
                                            "DYNAMIC_MODEL_AIRBORNE_1G",
                                            "DYNAMIC_MODEL_AIRBORNE_2G",
                                            "DYNAMIC_MODEL_AIRBORNE_4G",
                                            "DYNAMIC_MODEL_WRIST",
                                            "DYNAMIC_MODEL_INDOOR" };

    int dynamic_model = nh_->declare_parameter<int>("dynamic_model", DYNAMIC_MODEL_AIRBORNE_4G);
    ph.nodeParamEnum("dynamic_model", dynamicModel_, dyn_model_set, dynamic_model);

    bool cov_enabled = nh_->declare_parameter<bool>("enable_covariance", false);
    ph.nodeParam("enable_covariance", covariance_enabled_, cov_enabled);

    YAML::Node insMsgs = ph.node(insNode, "messages", 2);
    bool rs_odom_ins_enu_enable = nh_->declare_parameter<bool>("msg/odom_ins_enu/enable", false);
    int rs_odom_ins_enu_period = nh_->declare_parameter<int>("msg/odom_ins_enu/period", 1);
    ph.msgParams(rs_.odom_ins_enu, "odom_ins_enu", "", false, rs_odom_ins_enu_period, rs_odom_ins_enu_enable);

    bool rs_odom_ins_ned_enable = nh_->declare_parameter<bool>("msg/odom_ins_ned/enable", true);
    int rs_odom_ins_ned_period = nh_->declare_parameter<int>("msg/odom_ins_ned/period", 1);
    ph.msgParams(rs_.odom_ins_ned, "odom_ins_ned", "", true, rs_odom_ins_ned_period, rs_odom_ins_ned_enable);

    bool rs_odom_ins_ecef_enable = nh_->declare_parameter<bool>("msg/odom_ins_ecef/enable", false);
    int rs_odom_ins_ecef_period = nh_->declare_parameter<int>("msg/odom_ins_ecef/period", 1);
    ph.msgParams(rs_.odom_ins_ecef, "odom_ins_ecef", "", false, rs_odom_ins_ecef_period, rs_odom_ins_ecef_enable);

    bool did_ins1_enable = nh_->declare_parameter<bool>("msg/did_ins1/enable", false);
    int did_ins1_period = nh_->declare_parameter<int>("msg/did_ins1/period", 1);
    ph.msgParams(rs_.did_ins1, "msg/did_ins1/enable", "ins_eul_uvw_ned", false, did_ins1_period, did_ins1_enable);

    bool did_ins2_enable = nh_->declare_parameter<bool>("msg/did_ins2/enable", false);
    int did_ins2_period = nh_->declare_parameter<int>("msg/did_ins2/period", 1);
    ph.msgParams(rs_.did_ins2, "did_ins2", "ins_quat_uvw_lla", false, did_ins2_period, did_ins2_enable);

    bool did_ins4_enable = nh_->declare_parameter<bool>("msg/did_ins4/enable", false);
    int did_ins4_period = nh_->declare_parameter<int>("msg/did_ins4/period", 1);
    ph.msgParams(rs_.did_ins4, "did_ins4", "ins_quat_ve_ecef", false, did_ins4_period, did_ins4_enable);

    bool did_inl2_enable = nh_->declare_parameter<bool>("msg/inl2_states/enable", false);
    int did_inl2_period = nh_->declare_parameter<int>("msg/inls2_states/period", 1);
    ph.msgParams(rs_.inl2_states, "inl2_states", "", false, did_inl2_period, did_inl2_enable);
    insNode["messages"] = insMsgs;
    node["ins"] = insNode;

    // GPS 1
    YAML::Node gps1Node = ph.node(node, "gps1");
    std::string rs_gps1_type = nh_->declare_parameter<std::string>("gps1_type", "F9P");
    ph.nodeParam("type", rs_.gps1.type, rs_gps1_type);

    float gps_time_user_delay = nh_->declare_parameter<float>("gpsTimeUserDelay", 0);
    ph.nodeParam("gpsTimeUserDelay", gpsTimeUserDelay_, gps_time_user_delay);

    std::vector<double>offset_vec = nh_->declare_parameter<std::vector<double>>("antenna_offset_gps1", {0,0,0});
    if (offset_vec.size() == 3) {
        std::copy(offset_vec.begin(), offset_vec.end(), rs_.gps1.antennaOffset);
    }
    ph.nodeParamVec("antenna_offset", 3, rs_.gps1.antennaOffset, rs_.gps1.antennaOffset);
    YAML::Node gps1Msgs = ph.node(gps1Node, "messages", 2);

    bool rs_gps1_enable = nh_->declare_parameter<bool>("msg/gps1/enable", true);
    int rs_gps1_period = nh_->declare_parameter<int>("msg/gps1/period", 1) ;
    ph.msgParams(rs_.gps1, "pos_vel", "gps1/pos_vel", true, rs_gps1_period, rs_gps1_enable);

    bool rs_gps1_info_enable = nh_->declare_parameter<bool>("msg/gps1_info/enable", false);
    int rs_gps1_info_period = nh_->declare_parameter<int>("msg/gps1_info/period", 1);
    ph.msgParams(rs_.gps1_info, "info", "gps1/info", false, rs_gps1_info_period, rs_gps1_info_enable);

    bool rs_gps1_raw_enable = nh_->declare_parameter<bool>("msg/gps1_raw/enable", false);
    int rs_gps1_raw_period = nh_->declare_parameter<int>("msg/gps1_raw/period", 1);
    ph.msgParams(rs_.gps1_raw, "raw", "gps1/raw", false, rs_gps1_raw_period, rs_gps1_raw_enable);

    bool rs_gps1_navsatfix_enable = nh_->declare_parameter<bool>("msg/gps1_navsatfix/enable", false);
    int rs_gps1_navsatfix_period = nh_->declare_parameter<int>("msg/gps1_navsatfix/period", 1);
    ph.msgParams(rs_.gps1_navsatfix, "navsatfix", "gps1/NavSatFix", false, rs_gps1_navsatfix_period, rs_gps1_navsatfix_enable);

    gps1Node["messages"] = gps1Msgs;
    node["gps1"] = gps1Node;

    // GPS 2
    YAML::Node gps2Node = ph.node(node, "gps2");

    rs_.gps2.type = nh_->declare_parameter<std::string>("gps2_type", "F9P");
    ph.nodeParam("type", rs_.gps2.type);

    std::vector<double>offset_vec2 = nh_->declare_parameter<std::vector<double>>("antenna_offset_gps2", {0,0,0});
    if (offset_vec.size() == 3) {
        std::copy(offset_vec2.begin(), offset_vec2.end(), rs_.gps2.antennaOffset);
    }
    ph.nodeParamVec("antenna_offset", 3, rs_.gps2.antennaOffset);
    YAML::Node gps2Msgs = ph.node(gps2Node, "messages", 2);

    bool rs_gps2_enable = nh_->declare_parameter<bool>("msg/gps2/enable", false);
    int rs_gps2_period = nh_->declare_parameter<int>("msg/gps2/period", 1);
    ph.msgParams(rs_.gps2, "pos_vel", "gps2/pos_vel", false, rs_gps2_period, rs_gps2_enable);

    bool rs_gps2_info = nh_->declare_parameter<bool>("msg/gps2_info/enable", false);
    int rs_gps2_info_period = nh_->declare_parameter<int>("msg/gps2_info/period", 1);
    ph.msgParams(rs_.gps2_info, "info", "gps2/info", false, rs_gps2_info_period, rs_gps2_info);

    bool rs_gps2_raw = nh_->declare_parameter<bool>("msg/gps2_raw/enable", false);
    int rs_gps2_raw_period = nh_->declare_parameter<int>("gps2/raw/period", 1);
    ph.msgParams(rs_.gps2_raw, "raw", "gps2/raw", false, rs_gps2_raw_period, rs_gps2_raw);

    bool rs_gps2_nsf = nh_->declare_parameter<bool>("msg/gps2_navsatfix/enable", false);
    int rs_gps2_nsf_period = nh_->declare_parameter<int>("msg/gps2_navsatfix/period", 1);
    ph.msgParams(rs_.gps2_navsatfix, "navsatfix", "gps2/NavSatFix", false, rs_gps2_nsf_period, rs_gps2_nsf);

    gps2Node["messages"] = gps2Msgs;
    node["gps2"] = gps2Node;

    YAML::Node evbNode = ph.node(node, "evb");
    int evb_cb_preset = nh_->declare_parameter<int>("cb_preset", 2);
    ph.nodeParam("cb_preset", evb_.cb_preset, evb_cb_preset);        // 2=RS232(default), 3=XBee Radio On, 4=WiFi On & RS422, 5=SPI, 6=USB hub, 7=USB hub w/ RS422, 8=all off but USB

    int evb_cb_options = nh_->declare_parameter<int>("cb_options", 0);
    ph.nodeParam("cb_options", evb_.cb_options, evb_cb_options);

    YAML::Node rtkRoverNode = ph.node(node, "rtk_rover");
    if (rtkRoverNode.IsDefined() && !rtkRoverNode.IsNull())
        RTK_rover_ = new RtkRoverProvider(rtkRoverNode);

    YAML::Node rtkBaseNode = ph.node(node, "rtk_base");
    if (rtkBaseNode.IsDefined() && !rtkBaseNode.IsNull())
        RTK_base_ = new RtkBaseProvider(rtkBaseNode);

    YAML::Node diagNode = ph.node(node, "diagnostics");
    bool rs_diagnostics_enabled = nh_->declare_parameter<bool>("msg/diagnostics/enable", false);
    ph.nodeParam("enable", rs_.diagnostics.enabled, rs_diagnostics_enabled);

    // Print entire yaml node tree
     //printf("Node Tree:\n");
     //std::cout << node << "\n\n=====================  EXIT  =====================\n\n";

    // exit(1);
}


void InertialSenseROS::configure_data_streams()
{
    configure_data_streams(false);
}

#define CONFIG_STREAM(stream, did, type, cb_fun) \
    if((stream.enabled) && !(stream.streaming)){ \
        rclcpp::Logger logger_conf_str = rclcpp::get_logger("config_stream"); \
        logger_conf_str.set_level(rclcpp::Logger::Level::Debug); \
        RCLCPP_DEBUG(logger_conf_str,"InertialSenseROS: Attempting to enable %s (%d) data stream", cISDataMappings::DataName(did), did); \
        SET_CALLBACK(did, type, cb_fun, stream.period); \
        if (!firstrun) \
            return; \
    }

#define CONFIG_STREAM_GPS(stream, did_pos, cb_fun_pos, did_vel, cb_fun_vel) \
    if((stream.enabled) && !(stream.streaming_pos)){ \
        rclcpp::Logger logger_conf_str_gps_pos = rclcpp::get_logger("config_stream_gps_pos"); \
        logger_conf_str_gps_pos.set_level(rclcpp::Logger::Level::Debug); \
        RCLCPP_DEBUG(logger_conf_str_gps_pos,"InertialSenseROS: Attempting to enable %s (%d) data stream", cISDataMappings::DataName(did_pos), did_pos); \
        SET_CALLBACK(did_pos, gps_pos_t, cb_fun_pos, stream.period); \
        if (!firstrun) \
            return; \
    } \
    if((stream.enabled) && !(stream.streaming_vel)){ \
        rclcpp::Logger logger_conf_str_gps_vel = rclcpp::get_logger("config_stream_gps_vel"); \
        logger_conf_str_gps_vel.set_level(rclcpp::Logger::Level::Debug); \
        RCLCPP_DEBUG(logger_conf_str_gps_vel,"InertialSenseROS: Attempting to enable %s (%d) data stream", cISDataMappings::DataName(did_vel), did_vel); \
        SET_CALLBACK(did_vel, gps_vel_t, cb_fun_vel, stream.period); \
        if (!firstrun) \
            return; \
    }

void InertialSenseROS::configure_data_streams(bool firstrun) // if firstrun is true each step will be attempted without returning
{
    if (!rs_.gps1.streaming_pos) // we always need GPS for Fix status
    {
        rclcpp::Logger logger_gps1pos = rclcpp::get_logger("gps1_pos");
        logger_gps1pos.set_level(rclcpp::Logger::Level::Debug);
        RCLCPP_DEBUG(logger_gps1pos,"InertialSenseROS: Attempting to enable GPS1 Pos data stream");
        SET_CALLBACK(DID_GPS1_POS, gps_pos_t, GPS_pos_callback, rs_.gps1.period);
    }
    if (!flashConfigStreaming_)
    {
        rclcpp::Logger logger_flash_conf = rclcpp::get_logger("flash_config");
        logger_flash_conf.set_level(rclcpp::Logger::Level::Debug);
        RCLCPP_DEBUG(logger_flash_conf,"InertialSenseROS: Attempting to enable flash config data stream");
        SET_CALLBACK(DID_FLASH_CONFIG, nvm_flash_cfg_t, flash_config_callback, 0);
        if (!firstrun)
            return;
    }

    if (rs_.odom_ins_ned.enabled && !(rs_.did_ins4.streaming && imuStreaming_))
    {
        rclcpp::Logger logger_odom_ins_ned = rclcpp::get_logger("odom_ins_ned");
        logger_odom_ins_ned.set_level(rclcpp::Logger::Level::Debug);
        RCLCPP_DEBUG(logger_odom_ins_ned,"InertialSenseROS: Attempting to enable odom INS NED data stream");

        SET_CALLBACK(DID_INS_4, ins_4_t, INS4_callback, rs_.did_ins4.period);                     // Need NED
        SET_CALLBACK(DID_PIMU, pimu_t, preint_IMU_callback, rs_.pimu.period);                     // Need angular rate data from IMU
        rs_.imu.enabled = true;
        odometryIdentity(msg_odom_ned);
        if (!firstrun)
            return;;
    }

    if (rs_.odom_ins_ecef.enabled && !(rs_.did_ins4.streaming && imuStreaming_))
    {
        rclcpp::Logger logger_odom_ins_ecef = rclcpp::get_logger("odom_ins_ecef");
        logger_odom_ins_ecef.set_level(rclcpp::Logger::Level::Debug);
        RCLCPP_DEBUG(logger_odom_ins_ecef,"InertialSenseROS: Attempting to enable odom INS ECEF data stream");
        SET_CALLBACK(DID_INS_4, ins_4_t, INS4_callback, rs_.did_ins4.period);                     // Need quaternion and ecef
        SET_CALLBACK(DID_PIMU, pimu_t, preint_IMU_callback, rs_.pimu.period);                     // Need angular rate data from IMU
        rs_.imu.enabled = true;
        odometryIdentity(msg_odom_ecef);
        if (!firstrun)
            return;
    }

    if (rs_.odom_ins_enu.enabled  && !(rs_.did_ins4.streaming && imuStreaming_))
    {
        rclcpp::Logger logger_odom_ins_enu = rclcpp::get_logger("odom_ins_enu");
        logger_odom_ins_enu.set_level(rclcpp::Logger::Level::Debug);
        RCLCPP_DEBUG(logger_odom_ins_enu,"InertialSenseROS: Attempting to enable odom INS ENU data stream");
        SET_CALLBACK(DID_INS_4, ins_4_t, INS4_callback, rs_.did_ins4.period);                     // Need ENU
        SET_CALLBACK(DID_PIMU, pimu_t, preint_IMU_callback, rs_.pimu.period);                     // Need angular rate data from IMU
        rs_.imu.enabled = true;
        odometryIdentity(msg_odom_enu);
        if (!firstrun)
            return;
    }

    if (covariance_enabled_ && !insCovarianceStreaming_)
    {
        rclcpp::Logger logger_covariance = rclcpp::get_logger("covariance_stream");
        logger_covariance.set_level(rclcpp::Logger::Level::Debug);
        RCLCPP_DEBUG(logger_covariance, "InertialSenseROS: Attempting to enable %s data stream", cISDataMappings::DataName(DID_ROS_COVARIANCE_POSE_TWIST));
        SET_CALLBACK(DID_ROS_COVARIANCE_POSE_TWIST, ros_covariance_pose_twist_t, INS_covariance_callback, 200);
    }

    CONFIG_STREAM(rs_.did_ins1, DID_INS_1, ins_1_t, INS1_callback);
    CONFIG_STREAM(rs_.did_ins2, DID_INS_2, ins_2_t, INS2_callback);
    CONFIG_STREAM(rs_.did_ins4, DID_INS_4, ins_4_t, INS4_callback);
    CONFIG_STREAM(rs_.inl2_states, DID_INL2_STATES, inl2_states_t, INL2_states_callback);

    nvm_flash_cfg_t flashCfg;
    IS_.FlashConfig(flashCfg);
    if (!NavSatFixConfigured)
    {
        if (rs_.gps1_navsatfix.enabled) {
            rclcpp::Logger logger_nsfgps1 = rclcpp::get_logger("enable_nav_sat_fix_gps1");
            logger_nsfgps1.set_level(rclcpp::Logger::Level::Debug);
            RCLCPP_DEBUG(logger_nsfgps1,"InertialSenseROS: Attempting to enable gps1/NavSatFix");
            // Satellite system constellation used in GNSS solution.  (see eGnssSatSigConst) 0x0003=GPS, 0x000C=QZSS, 0x0030=Galileo, 0x00C0=Beidou, 0x0300=GLONASS, 0x1000=SBAS
            uint16_t gnssSatSigConst = flashCfg.gnssSatSigConst;

            if (gnssSatSigConst & GNSS_SAT_SIG_CONST_GPS) {
                msg_NavSatFix.status.service |= NavSatFixService::SERVICE_GPS;
            }
            if (gnssSatSigConst & GNSS_SAT_SIG_CONST_GLO) {
                msg_NavSatFix.status.service |= NavSatFixService::SERVICE_GLONASS;
            }
            if (gnssSatSigConst & GNSS_SAT_SIG_CONST_BDS) {
                msg_NavSatFix.status.service |= NavSatFixService::SERVICE_COMPASS; // includes BeiDou.
            }
            if (gnssSatSigConst & GNSS_SAT_SIG_CONST_GAL) {
                msg_NavSatFix.status.service |= NavSatFixService::SERVICE_GALILEO;
            }
        }
        if (rs_.gps2_navsatfix.enabled) {
            rclcpp::Logger logger_nsfgps2 = rclcpp::get_logger("enable_nav_sat_fix_gps2");
            logger_nsfgps2.set_level(rclcpp::Logger::Level::Debug);
            RCLCPP_DEBUG(logger_nsfgps2,"InertialSenseROS: Attempting to enable gps2/NavSatFix");
            // Satellite system constellation used in GNSS solution.  (see eGnssSatSigConst) 0x0003=GPS, 0x000C=QZSS, 0x0030=Galileo, 0x00C0=Beidou, 0x0300=GLONASS, 0x1000=SBAS
            uint16_t gnssSatSigConst = flashCfg.gnssSatSigConst;

            if (gnssSatSigConst & GNSS_SAT_SIG_CONST_GPS) {
                msg_NavSatFix.status.service |= NavSatFixService::SERVICE_GPS;
            }
            if (gnssSatSigConst & GNSS_SAT_SIG_CONST_GLO) {
                msg_NavSatFix.status.service |= NavSatFixService::SERVICE_GLONASS;
            }
            if (gnssSatSigConst & GNSS_SAT_SIG_CONST_BDS) {
                msg_NavSatFix.status.service |= NavSatFixService::SERVICE_COMPASS; // includes BeiDou.
            }
            if (gnssSatSigConst & GNSS_SAT_SIG_CONST_GAL) {
                msg_NavSatFix.status.service |= NavSatFixService::SERVICE_GALILEO;
            }
        }
        NavSatFixConfigured = true;
        // DID_GPS1_POS and DID_GPS1_VEL are always streamed for fix status. See below
        if (!firstrun)
            return;
    }

    if (rs_.gps1.enabled)
    {   // Set up the GPS ROS stream - we always need GPS information for time sync, just don't always need to publish it
        CONFIG_STREAM_GPS(rs_.gps1, DID_GPS1_POS, GPS_pos_callback, DID_GPS1_VEL, GPS_vel_callback);
        //CONFIG_STREAM(rs_.gps1_raw, DID_GPS1_RAW, gps_raw_t, GPS_raw_callback); // Requires IG-2.0
        CONFIG_STREAM(rs_.gps1_info, DID_GPS1_SAT, gps_sat_t, GPS_info_callback);
    }
    if (rs_.gps2.enabled)
    {
        CONFIG_STREAM_GPS(rs_.gps2, DID_GPS2_POS, GPS_pos_callback, DID_GPS2_VEL, GPS_vel_callback);
        //CONFIG_STREAM(rs_.gps2_raw, DID_GPS2_RAW, gps_raw_t, GPS_raw_callback);   //Requires IG-2.0
        //CONFIG_STREAM(rs_.gps2_info, DID_GPS2_SAT, gps_sat_t, GPS_info_callback); //not currrently working - ignore for now
    }
    //CONFIG_STREAM(rs_.gpsbase_raw, DID_GPS_BASE_RAW, gps_raw_t, GPS_raw_callback); //not currrently working

    CONFIG_STREAM(rs_.magnetometer, DID_MAGNETOMETER, magnetometer_t, mag_callback);
    CONFIG_STREAM(rs_.barometer, DID_BAROMETER, barometer_t, baro_callback);
    CONFIG_STREAM(rs_.pimu, DID_PIMU, pimu_t, preint_IMU_callback);

    if (!firstrun)
    {
        data_streams_enabled_ = true;
        data_stream_timer_->cancel();
        RCLCPP_INFO(rclcpp::get_logger("data_streams_enabled"),"InertialSenseROS: All data streams successfully enabled");
        return;
    }
}

void InertialSenseROS::start_log()
{
    std::string filename = getenv("HOME");
    filename += "/Documents/Inertial_Sense/Logs/" + cISLogger::CreateCurrentTimestamp();
    RCLCPP_INFO_STREAM(rclcpp::get_logger("creating_log"),"InertialSenseROS: Creating log in " << filename << " folder");
    cISLogger::sSaveOptions options;
    options.logType = cISLogger::LOGTYPE_RAW;
    IS_.EnableLogger(true, filename, options, RMC_PRESET_IMX_PPD_GROUND_VEHICLE);
}

void InertialSenseROS::configure_ascii_output()
{
    //  nmea_msgs_t msgs = {};
    //  msgs.options = (NMEA_message_ports & NMEA_SER0) ? RMC_OPTIONS_PORT_SER0 : 0; // output on serial 0
    //  msgs.options |= (NMEA_message_ports & NMEA_SER1) ? RMC_OPTIONS_PORT_SER1 : 0; // output on serial 1
    //  msgs.gpgga = (NMEA_message_configuration & NMEA_GPGGA) ? NMEA_rate : 0;
    //  msgs.gpgll = (NMEA_message_configuration & NMEA_GPGLL) ? NMEA_rate : 0;
    //  msgs.gpgsa = (NMEA_message_configuration & NMEA_GPGSA) ? NMEA_rate : 0;
    //  msgs.gprmc = (NMEA_message_configuration & NMEA_GPRMC) ? NMEA_rate : 0;
    //  IS_.SendData(DID_NMEA_BCAST_PERIOD, (uint8_t*)(&msgs), sizeof(nmea_msgs_t), 0);
}

/**
 * Connects to the Inertial Sense hardware
 * Will attempt to connect using a list of multiple ports if specified,
 * @param timeout specifies the maximum duration (in seconds) before a returning false and reporting an ERROR
 * @return return true when a connection has been made, or false if not
 */
bool InertialSenseROS::connect(float timeout)
{
    uint32_t end_time = nh_->now().seconds() + timeout;
    auto ports_iterator = ports_.begin();

    do {
        std::string cur_port = *ports_iterator;
        /// Connect to the IMX
        RCLCPP_INFO(rclcpp::get_logger("connect_to_serial"),"InertialSenseROS: Connecting to serial port \"%s\", at %d baud", cur_port.c_str(), baudrate_);
        sdk_connected_ = IS_.Open(cur_port.c_str(), baudrate_);
        if (!sdk_connected_) {
            RCLCPP_ERROR(rclcpp::get_logger("open_port_error"),"InertialSenseROS: Unable to open serial port \"%s\", at %d baud", cur_port.c_str(), baudrate_);
            sleep(1); // is this a good idea?
        } else {
            RCLCPP_INFO(rclcpp::get_logger("serial_port_connected_info"),"InertialSenseROS: Connected to IMX SN%d on \"%s\", at %d baud", IS_.DeviceInfo().serialNumber, cur_port.c_str(), baudrate_);
            port_ = cur_port;
            break;
        }
        if ((ports_.size() > 1) && (ports_iterator != ports_.end()))
            ports_iterator++;
        else
            ports_iterator = ports_.begin(); // just keep looping until we timeout below
    } while (nh_->now().seconds() < end_time);

    return sdk_connected_;
}

bool InertialSenseROS::firmware_compatiblity_check()
{
    char local_protocol[4] = { PROTOCOL_VERSION_CHAR0, PROTOCOL_VERSION_CHAR1, PROTOCOL_VERSION_CHAR2, PROTOCOL_VERSION_CHAR3 };
    char diff_protocol[4] = { 0, 0, 0, 0 };
    for (int i = 0; i < sizeof(local_protocol); i++)  diff_protocol[i] = local_protocol[i] - IS_.DeviceInfo().protocolVer[i];

    char local_firmware[3] = { REPO_VERSION_MAJOR, REPO_VERSION_MINOR, REPO_VERSION_REVIS };
    char diff_firmware[3] = { 0, 0 ,0 };
    for (int i = 0; i < sizeof(local_firmware); i++)  diff_firmware[i] = local_firmware[i] - IS_.DeviceInfo().firmwareVer[i];

    rclcpp::Logger::Level protocol_fault = rclcpp::Logger::Level::Debug; // none
    if (diff_protocol[0] != 0) protocol_fault = rclcpp::Logger::Level::Fatal; // major protocol changes -- BREAKING
    else if (diff_protocol[1] != 0) protocol_fault = rclcpp::Logger::Level::Error; // minor protocol changes -- New parameters/features
    else if (diff_protocol[2] != 0) protocol_fault = rclcpp::Logger::Level::Warn; // patch changes - the shouldn't be significant, but still important
    else if (diff_protocol[3] != 0) protocol_fault = rclcpp::Logger::Level::Info; // this is essentially trivial, but good to know.

    rclcpp::Logger::Level firmware_fault = rclcpp::Logger::Level::Debug; // none
    if (diff_firmware[0] != 0) firmware_fault = rclcpp::Logger::Level::Fatal;  // major protocol changes -- BREAKING
    else if (diff_firmware[1] != 0) firmware_fault = rclcpp::Logger::Level::Error;  // minor protocol changes -- New parameters/features
    else if (diff_firmware[2] != 0) firmware_fault = rclcpp::Logger::Level::Warn; // patch changes - the shouldn't be significant, but still important

    rclcpp::Logger::Level final_fault = std::max(firmware_fault, protocol_fault);
  // ROS_LOG_COND(final_fault != rclcpp::Logger::Level::Debug, final_fault, ROSCONSOLE_DEFAULT_NAME,
  //         "Protocol version mismatch: \n"
  //         "   protocol %d.%d.%d.%d  firmware %d.%d.%d  (SDK)\n"
  //         "   protocol %d.%d.%d.%d  firmware %d.%d.%d  (device)",
  //         PROTOCOL_VERSION_CHAR0,
  //         PROTOCOL_VERSION_CHAR1,
  //         PROTOCOL_VERSION_CHAR2,
  //         PROTOCOL_VERSION_CHAR3,
  //         REPO_VERSION_MAJOR,
  //         REPO_VERSION_MINOR,
  //         REPO_VERSION_REVIS,
  //         IS_.DeviceInfo().protocolVer[0],
  //         IS_.DeviceInfo().protocolVer[1],
  //         IS_.DeviceInfo().protocolVer[2],
  //         IS_.DeviceInfo().protocolVer[3],
  //         IS_.DeviceInfo().firmwareVer[0],
  //         IS_.DeviceInfo().firmwareVer[1],
  //         IS_.DeviceInfo().firmwareVer[2]
  // );
    if (final_fault != rclcpp::Logger::Level::Debug) {
        RCLCPP_INFO(rclcpp::get_logger("final_fault_logger"), "Protocol version mismatch: \n"
            "   protocol %d.%d.%d.%d  firmware %d.%d.%d  (SDK)\n"
            "   protocol %d.%d.%d.%d  firmware %d.%d.%d  (device)",
            PROTOCOL_VERSION_CHAR0,
            PROTOCOL_VERSION_CHAR1,
            PROTOCOL_VERSION_CHAR2,
            PROTOCOL_VERSION_CHAR3,
            REPO_VERSION_MAJOR,
            REPO_VERSION_MINOR,
            REPO_VERSION_REVIS,
            IS_.DeviceInfo().protocolVer[0],
            IS_.DeviceInfo().protocolVer[1],
            IS_.DeviceInfo().protocolVer[2],
            IS_.DeviceInfo().protocolVer[3],
            IS_.DeviceInfo().firmwareVer[0],
            IS_.DeviceInfo().firmwareVer[1],
            IS_.DeviceInfo().firmwareVer[2]);
    }
    return final_fault == rclcpp::Logger::Level::Debug; // true if they match, false if they don't.
}

bool vecF32Match(float v1[], float v2[], int size=3)
{
    for (int i=0; i<size; i++)
    {
        if (v1[i] != v2[i])
        {
            return false;
        }
    }
    return true;
}

bool vecF64Match(double v1[], double v2[], int size=3)
{
    for (int i=0; i<size; i++)
    {
        if (v1[i] != v2[i])
        {
            return false;
        }
    }
    return true;
}

void InertialSenseROS::configure_flash_parameters()
{
    bool reboot = false;
    nvm_flash_cfg_t current_flash_cfg;
    IS_.FlashConfig(current_flash_cfg);
    //RCLCPP_INFO(rclcpp::get_logger("E"),"InertialSenseROS: Configuring flash: \nCurrent: %i, \nDesired: %i\n", current_flash_cfg.ioConfig, ioConfig_);

    if (current_flash_cfg.startupNavDtMs != ins_nav_dt_ms_)
    {
        RCLCPP_INFO(rclcpp::get_logger("nav_rate_change_reset"),"InertialSenseROS: Navigation rate change from %dms to %dms, resetting IMX to make change", current_flash_cfg.startupNavDtMs, ins_nav_dt_ms_);
        reboot = true;
    }
    if (setIoConfigBits_ && ioConfigBits_ != current_flash_cfg.ioConfig)
    {
        RCLCPP_INFO(rclcpp::get_logger("io_config_change_reset"),"InertialSenseROS: ioConfig change from 0x%08X to 0x%08X, resetting IMX to make change", current_flash_cfg.ioConfig, ioConfigBits_);
        reboot = true;
    }
    else
    {   // Don't change
        ioConfigBits_ = current_flash_cfg.ioConfig;
    }
    if (setPlatformConfig_ && platformConfig_ != current_flash_cfg.platformConfig &&
        !(current_flash_cfg.platformConfig & PLATFORM_CFG_TYPE_FROM_MANF_OTP))
    {
        reboot = true;
    }
    else
    {   // Don't change
        platformConfig_ = current_flash_cfg.platformConfig;
    }

    if (!vecF32Match(current_flash_cfg.insRotation, insRotation_) ||
        !vecF32Match(current_flash_cfg.insOffset, insOffset_) ||
        !vecF32Match(current_flash_cfg.gps1AntOffset, rs_.gps1.antennaOffset) ||
        !vecF32Match(current_flash_cfg.gps2AntOffset, rs_.gps2.antennaOffset) ||
        (refLLA_valid && !vecF64Match(current_flash_cfg.refLla, refLla_)) ||
        current_flash_cfg.startupNavDtMs != ins_nav_dt_ms_ ||
        current_flash_cfg.ioConfig != ioConfigBits_ ||
        current_flash_cfg.gpsTimeUserDelay != gpsTimeUserDelay_ ||
        // current_flash_cfg.magDeclination != magDeclination_ ||
        current_flash_cfg.dynamicModel != dynamicModel_ ||
        current_flash_cfg.platformConfig != platformConfig_
        )
    {
        for (int i=0; i<3; i++)
        {
            current_flash_cfg.insRotation[i] = insRotation_[i];
            current_flash_cfg.insOffset[i] = insOffset_[i];
            current_flash_cfg.gps1AntOffset[i] = rs_.gps1.antennaOffset[i];
            current_flash_cfg.gps2AntOffset[i] = rs_.gps2.antennaOffset[i];
            if (refLLA_valid)
            {
                current_flash_cfg.refLla[i] = refLla_[i];
            }
        }
        current_flash_cfg.startupNavDtMs = ins_nav_dt_ms_;
        current_flash_cfg.ioConfig = ioConfigBits_;
        current_flash_cfg.gpsTimeUserDelay = gpsTimeUserDelay_;
        current_flash_cfg.magDeclination = magDeclination_;
        current_flash_cfg.dynamicModel = dynamicModel_;
        current_flash_cfg.platformConfig = platformConfig_;

        IS_.SendData(DID_FLASH_CONFIG, (uint8_t *)(&current_flash_cfg), sizeof (nvm_flash_cfg_t), 0);
    }

    if  (reboot)
    {
        sleep(3);
        reset_device();
    }
}

// FIXME:: THESE SHOULD BE IN RtkRoverCorrectionProvider_Ntrip
void InertialSenseROS::connect_rtk_client(RtkRoverCorrectionProvider_Ntrip& config)
{
    config.connecting_ = true;

    // [type]:[protocol]:[ip/url]:[port]:[mountpoint]:[username]:[password]
    std::string RTK_connection = config.get_connection_string();

    int RTK_connection_attempt_count = 0;
    while (++RTK_connection_attempt_count < config.connection_attempt_limit_)
    {
        config.connected_ = IS_.OpenConnectionToServer(RTK_connection);

        int sleep_duration = RTK_connection_attempt_count * config.connection_attempt_backoff_;
        if (config.connected_) {
            RCLCPP_INFO_STREAM(rclcpp::get_logger("successfully_connected_rtk"),"InertialSenseROS: Successfully connected to RTK server [" << RTK_connection  << "]. [Attempt " << RTK_connection_attempt_count << "]");
            break;
        }
        // fall-through

        RCLCPP_ERROR_STREAM(rclcpp::get_logger("failed_to_connect_base"),"Failed to connect to base server at " << RTK_connection);
        if (RTK_connection_attempt_count < config.connection_attempt_limit_) {
            RCLCPP_WARN_STREAM(rclcpp::get_logger("unable_to_connect_reattempt"),"InertialSenseROS: Unable to establish connection with RTK server [" << RTK_connection << "] after attempt " << RTK_connection_attempt_count << ". Will try again in " << sleep_duration << " seconds.");
       } else {
           RCLCPP_ERROR_STREAM(rclcpp::get_logger("unable_to_connect_giveup"),"InertialSenseROS: Unable to establish connection with RTK server [" << RTK_connection << "] after attempt " << RTK_connection_attempt_count << ". Giving up.");
       }
       //rclcpp::Duration(sleep_duration,0).sleep(); // we will always sleep on a failure...
       rclcpp::Rate r(sleep_duration); r.sleep();
   }

   config.connecting_ = false;
}

void InertialSenseROS::rtk_connectivity_watchdog_timer_callback()
{
    if ((RTK_rover_ == nullptr) || (RTK_rover_->correction_input == nullptr) || (RTK_rover_->correction_input->type_ != "ntrip"))
        return;

    RtkRoverCorrectionProvider_Ntrip& config = *(RtkRoverCorrectionProvider_Ntrip*)(RTK_rover_->correction_input);
    if (config.connecting_)
    {
        return;
    }

    int latest_byte_count = IS_.ClientServerByteCount();
    if (config.traffic_total_byte_count_ == latest_byte_count)
    {
        ++config.data_transmission_interruption_count_;

        if (config.data_transmission_interruption_count_ >= config.data_transmission_interruption_limit_)
        {
            if (config.traffic_time > 0.0)
                RCLCPP_WARN_STREAM(rclcpp::get_logger("rtk_correction_try_again"),"Last received RTK correction data was " << (nh_->now().seconds() - config.traffic_time) << " seconds ago. Attempting to re-establish connection.");
            connect_rtk_client(config);
            if (config.connected_) {
                config.traffic_total_byte_count_ = latest_byte_count;
                config.data_transmission_interruption_count_ = 0;
            }
        } else {
            if (config.traffic_time > 0.0)
                RCLCPP_WARN_STREAM(rclcpp::get_logger("rtk_correction"),"Last received RTK correction data was " << (nh_->now().seconds() - config.traffic_time) << " seconds ago.");
        }
    }
    else
    {
        config.traffic_time = nh_->now().seconds();
        config.traffic_total_byte_count_ = latest_byte_count;
        config.data_transmission_interruption_count_ = 0;
    }
}

void InertialSenseROS::start_rtk_connectivity_watchdog_timer()
{

    if ((RTK_rover_ == nullptr) || (RTK_rover_->correction_input == nullptr) || (RTK_rover_->correction_input->type_ != "ntrip"))
        return;

    RtkRoverCorrectionProvider_Ntrip& config = *(RtkRoverCorrectionProvider_Ntrip*)(RTK_rover_->correction_input);
    rtk_connectivity_watchdog_timer_ = nh_->create_wall_timer(std::chrono::duration<float>(config.connectivity_watchdog_timer_frequency_) , std::bind(InertialSenseROS::rtk_connectivity_watchdog_timer_callback, this));
    rtk_connectivity_watchdog_timer_->cancel();
    if (!config.connectivity_watchdog_enabled_) {
        return;
    }

    if (!rtk_connectivity_watchdog_timer_->is_canceled() == false) {
        rtk_connectivity_watchdog_timer_ = nh_->create_wall_timer(std::chrono::duration<float>(config.connectivity_watchdog_timer_frequency_) , std::bind(InertialSenseROS::rtk_connectivity_watchdog_timer_callback, this));
    }

    rtk_connectivity_watchdog_timer_->reset();
}

void InertialSenseROS::stop_rtk_connectivity_watchdog_timer()
{
    rtk_connectivity_watchdog_timer_->cancel();
    if ((RTK_rover_ != nullptr) && (RTK_rover_->correction_input != nullptr) && (RTK_rover_->correction_input->type_ != "ntrip")) {
        RtkRoverCorrectionProvider_Ntrip& config = *(RtkRoverCorrectionProvider_Ntrip*)(RTK_rover_->correction_input);
        config.traffic_total_byte_count_ = 0;
        config.data_transmission_interruption_count_ = 0;
    }
}

// FIXME:: THIS SHOULD BE IN RtkBaseCorrectionProvider_Ntrip
void InertialSenseROS::start_rtk_server(RtkBaseCorrectionProvider_Ntrip& config)
{
    // [type]:[ip/url]:[port]
    std::string RTK_connection = config.get_connection_string();
    if (IS_.CreateHost(RTK_connection))
        RCLCPP_INFO_STREAM(rclcpp::get_logger("started_rtk_ntrip_server"),"InertialSenseROS: Successfully started RTK Base NTRIP correction server at" << RTK_connection);
    else
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("failed_to_start_ntrip_server"),"InertialSenseROS: Failed to start RTK Base NTRIP correction server at " << RTK_connection);
}


void InertialSenseROS::configure_rtk()
{
    rtkConfigBits_ = 0;
    if (rs_.gps1.type == "F9P")
    {
        if (RTK_rover_)
        {
            if (RTK_rover_->correction_input && (RTK_rover_->correction_input->type_ == "ntrip")) {
                RCLCPP_INFO(rclcpp::get_logger("config_rtk_rover"),"InertialSenseROS: Configuring RTK Rover");
                rs_.rtk_pos.enabled = true;
                connect_rtk_client(*(RtkRoverCorrectionProvider_Ntrip *) RTK_rover_->correction_input);
                rtkConfigBits_ |= RTK_CFG_BITS_ROVER_MODE_RTK_POSITIONING_EXTERNAL;
                SET_CALLBACK(DID_GPS1_RTK_POS_MISC, gps_rtk_misc_t, RTK_Misc_callback, rs_.rtk_pos.period);
                SET_CALLBACK(DID_GPS1_RTK_POS_REL, gps_rtk_rel_t, RTK_Rel_callback, rs_.rtk_pos.period);

                start_rtk_connectivity_watchdog_timer();
            }
            if (RTK_rover_->correction_input && (RTK_rover_->correction_input->type_ == "evb")) {
                RCLCPP_INFO(rclcpp::get_logger("config_rtk_rover_w_radio"),"InertialSenseROS: Configuring RTK Rover with radio enabled");
                rs_.rtk_pos.enabled = true;
                if (RTK_base_) RTK_base_->enable = false;
                rtkConfigBits_ |= RTK_CFG_BITS_ROVER_MODE_RTK_POSITIONING_EXTERNAL;
                SET_CALLBACK(DID_GPS1_RTK_POS_MISC, gps_rtk_misc_t, RTK_Misc_callback, rs_.rtk_pos.period);
                SET_CALLBACK(DID_GPS1_RTK_POS_REL, gps_rtk_rel_t, RTK_Rel_callback, rs_.rtk_pos.period);
            }
        }
        if (GNSS_Compass_)
        {
            RCLCPP_INFO(rclcpp::get_logger("config_dual_gnss"),"InertialSenseROS: Configuring Dual GNSS (compassing)");
            rs_.rtk_cmp.enabled = true;
            rtkConfigBits_ |= RTK_CFG_BITS_ROVER_MODE_RTK_COMPASSING_F9P;
            SET_CALLBACK(DID_GPS2_RTK_CMP_MISC, gps_rtk_misc_t, RTK_Misc_callback, rs_.rtk_cmp.period);
            SET_CALLBACK(DID_GPS2_RTK_CMP_REL, gps_rtk_rel_t, RTK_Rel_callback, rs_.rtk_cmp.period);
        }

        if (RTK_base_ && RTK_base_->enable) {
            RCLCPP_INFO(rclcpp::get_logger("config_rtk_base"),"InertialSenseROS: Configuring RTK Base");
            if (RTK_base_->source_gps__usb_) {
                rtkConfigBits_ |= RTK_CFG_BITS_BASE_OUTPUT_GPS1_RTCM3_USB;
            }
            if (RTK_base_->source_gps__serial0_) {
                rtkConfigBits_ |= RTK_CFG_BITS_BASE_OUTPUT_GPS1_RTCM3_SER2;
            }
            RtkBaseCorrectionProvider_Ntrip* ntrip_provider = (RtkBaseCorrectionProvider_Ntrip*)RTK_base_->getProvidersByType("ntrip");
            if (ntrip_provider != nullptr) {
                start_rtk_server(*ntrip_provider);
            }
        }

        IS_.SendData(DID_FLASH_CONFIG, reinterpret_cast<uint8_t *>(&rtkConfigBits_), sizeof(rtkConfigBits_), offsetof(nvm_flash_cfg_t, RTKCfgBits));
    }
    else
    {
        if (RTK_rover_ && RTK_rover_->enable && RTK_base_ && RTK_base_->enable) {
            RCLCPP_ERROR(rclcpp::get_logger("default_to_rover"), "unable to configure onboard receiver to be both RTK rover and base - default to rover");
        }
       // RCLCPP_ERROR_COND(RTK_rover_ && RTK_rover_->enable && RTK_base_ && RTK_base_->enable, "unable to configure onboard receiver to be both RTK rover and base - default to rover");
        if (RTK_rover_ && RTK_rover_->enable && GNSS_Compass_) {
            RCLCPP_ERROR(rclcpp::get_logger("default_to_dual_GNSS"), "unable to configure onboard receiver to be both RTK rover as dual GNSS - default to dual GNSS");
        }
        //ROS_ERROR_COND(RTK_rover_  && RTK_rover_->enable && GNSS_Compass_, "unable to configure onboard receiver to be both RTK rover as dual GNSS - default to dual GNSS");

        if (GNSS_Compass_)
        {
            RCLCPP_INFO(rclcpp::get_logger("config_dual_gnss"),"InertialSenseROS: Configuring Dual GNSS (compassing)");
            RTK_rover_->enable = false; // FIXME:  Is this right?  Rover is disabled when in Compassing?
            rtkConfigBits_ |= RTK_CFG_BITS_ROVER_MODE_RTK_COMPASSING;
            SET_CALLBACK(DID_GPS2_RTK_CMP_MISC, gps_rtk_misc_t, RTK_Misc_callback, rs_.rtk_cmp.period);
            SET_CALLBACK(DID_GPS2_RTK_CMP_REL, gps_rtk_rel_t, RTK_Rel_callback, rs_.rtk_cmp.period);
        }

        if (RTK_rover_ && RTK_rover_->enable && RTK_rover_->correction_input && RTK_rover_->correction_input->type_ == "evb")
        {
            RCLCPP_INFO(rclcpp::get_logger("config_rtk_rover_w_radio"),"InertialSenseROS: Configuring RTK Rover with radio enabled");
            if (RTK_base_) RTK_base_->enable = false;
            rtkConfigBits_ |= (rs_.gps1.type == "F9P" ? RTK_CFG_BITS_ROVER_MODE_RTK_POSITIONING_EXTERNAL : RTK_CFG_BITS_ROVER_MODE_RTK_POSITIONING);
            SET_CALLBACK(DID_GPS1_RTK_POS_MISC, gps_rtk_misc_t, RTK_Misc_callback, rs_.rtk_pos.period);
            SET_CALLBACK(DID_GPS1_RTK_POS_REL, gps_rtk_rel_t, RTK_Rel_callback, rs_.rtk_pos.period);
        }
        else if (RTK_rover_ && RTK_rover_->enable && RTK_rover_->correction_input && RTK_rover_->correction_input->type_ == "ntrip")
        {
            RCLCPP_INFO(rclcpp::get_logger("config_as_rtk_rover"),"InertialSenseROS: Configuring as RTK Rover");
            if (RTK_base_) RTK_base_->enable = false;
            rtkConfigBits_ |= (rs_.gps1.type == "F9P" ? RTK_CFG_BITS_ROVER_MODE_RTK_POSITIONING_EXTERNAL : RTK_CFG_BITS_ROVER_MODE_RTK_POSITIONING);
            connect_rtk_client((RtkRoverCorrectionProvider_Ntrip&)*RTK_rover_->correction_input);
            SET_CALLBACK(DID_GPS1_RTK_POS_MISC, gps_rtk_misc_t, RTK_Misc_callback, rs_.rtk_pos.period);
            SET_CALLBACK(DID_GPS1_RTK_POS_REL, gps_rtk_rel_t, RTK_Rel_callback, rs_.rtk_pos.period);

            start_rtk_connectivity_watchdog_timer();
        }
        else if (RTK_base_ && RTK_base_->enable)
        {
            RCLCPP_INFO(rclcpp::get_logger("config_as_rtk_base"),"InertialSenseROS: Configured as RTK Base");
            if (RTK_base_->source_gps__serial0_)
                rtkConfigBits_ |= RTK_CFG_BITS_BASE_OUTPUT_GPS1_UBLOX_SER0;
            if (RTK_base_->source_gps__usb_)
                rtkConfigBits_ |= RTK_CFG_BITS_BASE_OUTPUT_GPS1_UBLOX_USB;

            RtkBaseCorrectionProvider_Ntrip* ntrip_provider = (RtkBaseCorrectionProvider_Ntrip*)RTK_base_->getProvidersByType("ntrip");
            if (ntrip_provider != nullptr)
                start_rtk_server(*ntrip_provider);
        }
        IS_.SendData(DID_FLASH_CONFIG, reinterpret_cast<uint8_t *>(&rtkConfigBits_), sizeof(rtkConfigBits_), offsetof(nvm_flash_cfg_t, RTKCfgBits));
    }
    RCLCPP_INFO(rclcpp::get_logger("set_rtkConfigBits"),"InertialSenseROS: Setting rtkConfigBits: 0x%08x", rtkConfigBits_);
}

void InertialSenseROS::flash_config_callback(eDataIDs DID, const nvm_flash_cfg_t *const msg)
{
    STREAMING_CHECK(flashConfigStreaming_, DID);

    setRefLla(msg->refLla);
}

void InertialSenseROS::setRefLla(const double refLla[3])
{
    refLla_[0] = refLla[0];
    refLla_[1] = refLla[1];
    refLla_[2] = refLla[2];
    if (!refLLA_valid)
    {
        rclcpp::Logger logger_reffla_set = rclcpp::get_logger("reflla_set");
        logger_reffla_set.set_level(rclcpp::Logger::Level::Debug);
        RCLCPP_DEBUG(logger_reffla_set,"InertialSenseROS: refLla was set");
    }
    refLLA_valid = true;
}

void InertialSenseROS::INS1_callback(eDataIDs DID, const ins_1_t *const msg)
{
    rs_.did_ins1.streamingCheck(DID);

    // Standard DID_INS_1 message
    if (rs_.did_ins1.enabled)
    {
        msg_did_ins1.header.stamp = ros_time_from_week_and_tow(msg->week, msg->timeOfWeek);
        msg_did_ins1.header.frame_id = frame_id_;
        msg_did_ins1.week = msg->week;
        msg_did_ins1.time_of_week = msg->timeOfWeek;
        msg_did_ins1.ins_status = msg->insStatus;
        msg_did_ins1.hdw_status = msg->hdwStatus;
        msg_did_ins1.theta[0] = msg->theta[0];
        msg_did_ins1.theta[1] = msg->theta[1];
        msg_did_ins1.theta[2] = msg->theta[2];
        msg_did_ins1.uvw[0] = msg->uvw[0];
        msg_did_ins1.uvw[1] = msg->uvw[1];
        msg_did_ins1.uvw[2] = msg->uvw[2];
        msg_did_ins1.lla[0] = msg->lla[0];
        msg_did_ins1.lla[1] = msg->lla[1];
        msg_did_ins1.lla[2] = msg->lla[2];
        msg_did_ins1.ned[0] = msg->ned[0];
        msg_did_ins1.ned[1] = msg->ned[1];
        msg_did_ins1.ned[2] = msg->ned[2];
       if(rs_.did_ins1.pub_didins1 != NULL) {
           if (rs_.did_ins1.pub_didins1->get_subscription_count() > 0)
               rs_.did_ins1.pub_didins1->publish(msg_did_ins1);
       }

    }
}

void InertialSenseROS::INS2_callback(eDataIDs DID, const ins_2_t *const msg)
{
    rs_.did_ins2.streamingCheck(DID);

    if (rs_.did_ins2.enabled)
    {
        // Standard DID_INS_2 message
        msg_did_ins2.header.frame_id = frame_id_;
        msg_did_ins2.week = msg->week;
        msg_did_ins2.time_of_week = msg->timeOfWeek;
        msg_did_ins2.ins_status = msg->insStatus;
        msg_did_ins2.hdw_status = msg->hdwStatus;
        msg_did_ins2.qn2b[0] = msg->qn2b[0];
        msg_did_ins2.qn2b[1] = msg->qn2b[1];
        msg_did_ins2.qn2b[2] = msg->qn2b[2];
        msg_did_ins2.qn2b[3] = msg->qn2b[3];
        msg_did_ins2.uvw[0] = msg->uvw[0];
        msg_did_ins2.uvw[1] = msg->uvw[1];
        msg_did_ins2.uvw[2] = msg->uvw[2];
        msg_did_ins2.lla[0] = msg->lla[0];
        msg_did_ins2.lla[1] = msg->lla[1];
        msg_did_ins2.lla[2] = msg->lla[2];
        if (rs_.did_ins2.pub_didins2 != NULL) {
            if (rs_.did_ins2.pub_didins2->get_subscription_count() > 0)
                rs_.did_ins2.pub_didins2->publish(msg_did_ins2);
        }

    }
}

void InertialSenseROS::INS4_callback(eDataIDs DID, const ins_4_t *const msg)
{
    rs_.did_ins4.streamingCheck(DID);

    if (rs_.did_ins4.enabled)
    {
        // Standard DID_INS_2 message
        msg_did_ins4.header.frame_id = frame_id_;
        msg_did_ins4.week = msg->week;
        msg_did_ins4.time_of_week = msg->timeOfWeek;
        msg_did_ins4.ins_status = msg->insStatus;
        msg_did_ins4.hdw_status = msg->hdwStatus;
        msg_did_ins4.qe2b[0] = msg->qe2b[0];
        msg_did_ins4.qe2b[1] = msg->qe2b[1];
        msg_did_ins4.qe2b[2] = msg->qe2b[2];
        msg_did_ins4.qe2b[3] = msg->qe2b[3];
        msg_did_ins4.ve[0] = msg->ve[0];
        msg_did_ins4.ve[1] = msg->ve[1];
        msg_did_ins4.ve[2] = msg->ve[2];
        msg_did_ins4.ecef[0] = msg->ecef[0];
        msg_did_ins4.ecef[1] = msg->ecef[1];
        msg_did_ins4.ecef[2] = msg->ecef[2];
        if (rs_.did_ins4.pub_didins4 != NULL) {
            if (rs_.did_ins4.pub_didins4->get_subscription_count() > 0)
                rs_.did_ins4.pub_didins4->publish(msg_did_ins4);
        }

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
        ecef2lla(Pe, lla);
        quat_ecef2ned(lla[0], lla[1], qe2n);

        if (rs_.odom_ins_ecef.enabled)
        {
            // Pose
            // Transform attitude body to ECEF
            transform_6x6_covariance(Pout, poseCov_, I, Rb2e);
            for (int i = 0; i < 36; i++)
            {
                msg_odom_ecef.pose.covariance[i] = Pout[i];
            }
            // Twist
            // Transform angular_rate from body to ECEF
            transform_6x6_covariance(Pout, twistCov_, I, Rb2e);
            for (int i = 0; i < 36; i++)
            {
                msg_odom_ecef.twist.covariance[i] = Pout[i];
            }
            msg_odom_ecef.header.stamp = ros_time_from_week_and_tow(msg->week, msg->timeOfWeek);
            msg_odom_ecef.header.frame_id = frame_id_;

            // Position
            msg_odom_ecef.pose.pose.position.x = msg->ecef[0];
            msg_odom_ecef.pose.pose.position.y = msg->ecef[1];
            msg_odom_ecef.pose.pose.position.z = -msg->ecef[2];

            // Attitude
            msg_odom_ecef.pose.pose.orientation.w = msg->qe2b[0];
            msg_odom_ecef.pose.pose.orientation.x = msg->qe2b[1];
            msg_odom_ecef.pose.pose.orientation.y = msg->qe2b[2];
            msg_odom_ecef.pose.pose.orientation.z = msg->qe2b[3];

            // Linear Velocity
            msg_odom_ecef.twist.twist.linear.x = msg->ve[0];
            msg_odom_ecef.twist.twist.linear.y = msg->ve[1];
            msg_odom_ecef.twist.twist.linear.z = msg->ve[2];

            // Angular Velocity
            ixVector3 result;
            ixEuler theta;
            quat2euler(msg->qe2b, theta);
            ixVector3 angVelImu = {(f_t)msg_imu.angular_velocity.x, (f_t)msg_imu.angular_velocity.y, (f_t)msg_imu.angular_velocity.z};
            vectorBodyToReference(angVelImu, theta, result);

            msg_odom_ecef.twist.twist.angular.x = result[0];
            msg_odom_ecef.twist.twist.angular.y = result[1];
            msg_odom_ecef.twist.twist.angular.z = result[2];
            rs_.odom_ins_ecef.pub_odometry->publish(msg_odom_ecef);

           // if (publishTf_)
           // {
           //     // Calculate the TF from the pose...
           //     transform_ECEF.setOrigin(tf::Vector3(msg_odom_ecef.pose.pose.position.x, msg_odom_ecef.pose.pose.position.y, msg_odom_ecef.pose.pose.position.z));
           //     tf::Quaternion q;
           //     tf::quaternionMsgToTF(msg_odom_ecef.pose.pose.orientation, q);
           //     transform_ECEF.setRotation(q);
//
           //     br.sendTransform(tf::StampedTransform(transform_ECEF, ros::Time::now(), "ins_ecef", "ins_base_link_ecef"));
           // }
        }


        if (rs_.odom_ins_ned.enabled)
        {
            if (!refLLA_valid)
            {
                RCLCPP_INFO(rclcpp::get_logger("wait_for_reflla"),"InertialSenseROS: Waiting for refLLA to be received from IMX");
            }
            else
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
                    msg_odom_ned.pose.covariance[i] = Pout[i];
                }
                // Twist
                // Transform velocity from ECEF to NED and angular rate from body to NED
                transform_6x6_covariance(Pout, twistCov_, Re2n, Rb2n);
                for (int i = 0; i < 36; i++)
                {
                    msg_odom_ned.twist.covariance[i] = Pout[i];
                }

                msg_odom_ned.header.stamp = ros_time_from_week_and_tow(msg->week, msg->timeOfWeek);
                msg_odom_ned.header.frame_id = frame_id_;

                // Position
                ixVector3d llaPosRadians;
                ecef2lla(msg->ecef, llaPosRadians);
                ixVector3 ned;
                ixVector3d refLlaRadians;
                lla_Deg2Rad_d(refLlaRadians, refLla_);
                lla2ned_d(refLlaRadians, llaPosRadians, ned);

                msg_odom_ned.pose.pose.position.x = ned[0];
                msg_odom_ned.pose.pose.position.y = ned[1];
                msg_odom_ned.pose.pose.position.z = ned[2];

                // Attitude
                msg_odom_ned.pose.pose.orientation.w = qn2b[0]; // w
                msg_odom_ned.pose.pose.orientation.x = qn2b[1]; // x
                msg_odom_ned.pose.pose.orientation.y = qn2b[2]; // y
                msg_odom_ned.pose.pose.orientation.z = qn2b[3]; // z

                // Linear Velocity
                ixVector3 result, theta;

                quatConjRot(result, qe2n, msg->ve);

                msg_odom_ned.twist.twist.linear.x = result[0];
                msg_odom_ned.twist.twist.linear.y = result[1];
                msg_odom_ned.twist.twist.linear.z = result[2];

                // Angular Velocity
                // Transform from body frame to NED
                ixVector3 angVelImu = {(f_t)msg_imu.angular_velocity.x, (f_t)msg_imu.angular_velocity.y, (f_t)msg_imu.angular_velocity.z};
                quatRot(result, qn2b, angVelImu);

                msg_odom_ned.twist.twist.angular.x = result[0];
                msg_odom_ned.twist.twist.angular.y = result[1];
                msg_odom_ned.twist.twist.angular.z = result[2];
                rs_.odom_ins_ned.pub_odometry->publish(msg_odom_ned);

               // if (publishTf_)
               // {
               //     // Calculate the TF from the pose...
               //     transform_NED.setOrigin(tf::Vector3(msg_odom_ned.pose.pose.position.x, msg_odom_ned.pose.pose.position.y, msg_odom_ned.pose.pose.position.z));
               //     tf::Quaternion q;
               //     tf::quaternionMsgToTF(msg_odom_ned.pose.pose.orientation, q);
               //     transform_NED.setRotation(q);
//
               //     br.sendTransform(tf::StampedTransform(transform_NED, ros::Time::now(), "ins_ned", "ins_base_link_ned"));
               // }
            }
        }

        if (rs_.odom_ins_enu.enabled)
        {
            if (!refLLA_valid)
            {
                RCLCPP_INFO(rclcpp::get_logger("wait_for_reflla"),"InertialSenseROS: Waiting for refLLA to be received from IMX");
            }
            else
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
                    msg_odom_enu.pose.covariance[i] = Pout[i];
                }
                // Twist
                // Transform velocity from ECEF to ENU and angular rate from body to ENU
                transform_6x6_covariance(Pout, twistCov_, Re2enu, Rb2enu);
                for (int i = 0; i < 36; i++)
                {
                    msg_odom_enu.twist.covariance[i] = Pout[i];
                }

                msg_odom_enu.header.stamp = ros_time_from_week_and_tow(msg->week, msg->timeOfWeek);
                msg_odom_enu.header.frame_id = frame_id_;

                // Position
                ixVector3d llaPosRadians;
                ecef2lla(msg->ecef, llaPosRadians);
                ixVector3 ned;
                ixVector3d refLlaRadians;
                lla_Deg2Rad_d(refLlaRadians, refLla_);
                lla2ned_d(refLlaRadians, llaPosRadians, ned);

                // Rearrange from NED to ENU
                msg_odom_enu.pose.pose.position.x = ned[1];
                msg_odom_enu.pose.pose.position.y = ned[0];
                msg_odom_enu.pose.pose.position.z = -ned[2];

                // Attitude
                msg_odom_enu.pose.pose.orientation.w = qenu2b[0];
                msg_odom_enu.pose.pose.orientation.x = qenu2b[1];
                msg_odom_enu.pose.pose.orientation.y = qenu2b[2];
                msg_odom_enu.pose.pose.orientation.z = qenu2b[3];

                // Linear Velocity
                    //same as NED but rearranged.
                ixVector3 result, theta;
                quatConjRot(result, qe2n, msg->ve);

                msg_odom_enu.twist.twist.linear.x = result[1];
                msg_odom_enu.twist.twist.linear.y = result[0];
                msg_odom_enu.twist.twist.linear.z = -result[2];

                // Angular Velocity
                // Transform from body frame to ENU
                ixVector3 angVelImu = {(f_t)msg_imu.angular_velocity.x, (f_t)msg_imu.angular_velocity.y, (f_t)msg_imu.angular_velocity.z};
                quatRot(result, qenu2b, angVelImu);

                msg_odom_enu.twist.twist.angular.x = result[0];
                msg_odom_enu.twist.twist.angular.y = result[1];
                msg_odom_enu.twist.twist.angular.z = result[2];
                rs_.odom_ins_enu.pub_odometry->publish(msg_odom_enu);

               // if (publishTf_)
               // {
               //     // Calculate the TF from the pose...
               //     transform_ENU.setOrigin(tf::Vector3(msg_odom_enu.pose.pose.position.x, msg_odom_enu.pose.pose.position.y, msg_odom_enu.pose.pose.position.z));
               //     tf::Quaternion q;
               //     tf::quaternionMsgToTF(msg_odom_enu.pose.pose.orientation, q);
               //     transform_ENU.setRotation(q);
//
               //     br.sendTransform(tf::StampedTransform(transform_ENU, ros::Time::now(), "ins_enu", "ins_base_link_enu"));
               // }
            }
        }
    }
}

void InertialSenseROS::INL2_states_callback(eDataIDs DID, const inl2_states_t *const msg)
{
    rs_.inl2_states.streamingCheck(DID);

    msg_inl2_states.header.stamp = ros_time_from_tow(msg->timeOfWeek);
    msg_inl2_states.header.frame_id = frame_id_;

    msg_inl2_states.quat_ecef.w = msg->qe2b[0];
    msg_inl2_states.quat_ecef.x = msg->qe2b[1];
    msg_inl2_states.quat_ecef.y = msg->qe2b[2];
    msg_inl2_states.quat_ecef.z = msg->qe2b[3];

    msg_inl2_states.vel_ecef.x = msg->ve[0];
    msg_inl2_states.vel_ecef.y = msg->ve[1];
    msg_inl2_states.vel_ecef.z = msg->ve[2];

    msg_inl2_states.pos_ecef.x = msg->ecef[0];
    msg_inl2_states.pos_ecef.y = msg->ecef[1];
    msg_inl2_states.pos_ecef.z = msg->ecef[2];

    msg_inl2_states.gyro_bias.x = msg->biasPqr[0];
    msg_inl2_states.gyro_bias.y = msg->biasPqr[1];
    msg_inl2_states.gyro_bias.z = msg->biasPqr[2];

    msg_inl2_states.accel_bias.x = msg->biasAcc[0];
    msg_inl2_states.accel_bias.y = msg->biasAcc[1];
    msg_inl2_states.accel_bias.z = msg->biasAcc[2];

    msg_inl2_states.baro_bias = msg->biasBaro;
    msg_inl2_states.mag_dec = msg->magDec;
    msg_inl2_states.mag_inc = msg->magInc;

    // Use custom INL2 states message
    if (rs_.inl2_states.enabled)
    {
       if (rs_.inl2_states.pub_inl2 != NULL) {
           if (rs_.inl2_states.pub_inl2->get_subscription_count() > 0)
               rs_.inl2_states.pub_inl2->publish(msg_inl2_states);
       }

    }
}

void InertialSenseROS::INS_covariance_callback(eDataIDs DID, const ros_covariance_pose_twist_t *const msg)
{
    STREAMING_CHECK(insCovarianceStreaming_, DID);

    float poseCovIn[36];
    int ind1, ind2;

    // Pose and twist covariances unwrapped from LD
    LD2Cov(msg->covPoseLD, poseCovIn, 6);
    LD2Cov(msg->covTwistLD, twistCov_, 6);

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
        rs_.gps1.streamingCheck(DID, rs_.gps1.streaming_pos);
        gps1_pos = *msg;
        primaryGpsDid = DID;

        if (rs_.gps1.enabled && msg->status & GPS_STATUS_FIX_MASK)
        {
            msg_gps1.header.stamp = ros_time_from_week_and_tow(msg->week, msg->timeOfWeekMs / 1.0e3);
            msg_gps1.week = msg->week;
            msg_gps1.status = msg->status;
            msg_gps1.header.frame_id = frame_id_;
            msg_gps1.num_sat = (uint8_t)(msg->status & GPS_STATUS_NUM_SATS_USED_MASK);
            msg_gps1.cno = msg->cnoMean;
            msg_gps1.latitude = msg->lla[0];
            msg_gps1.longitude = msg->lla[1];
            msg_gps1.altitude = msg->lla[2];
            msg_gps1.pos_ecef.x = ecef_[0] = msg->ecef[0];
            msg_gps1.pos_ecef.y = ecef_[1] = msg->ecef[1];
            msg_gps1.pos_ecef.z = ecef_[2] = msg->ecef[2];
            msg_gps1.hmsl = msg->hMSL;
            msg_gps1.hacc = msg->hAcc;
            msg_gps1.vacc = msg->vAcc;
            msg_gps1.pdop = msg->pDop;
            publishGPS1();
        }
        break;

    case DID_GPS2_POS:
        rs_.gps2.streamingCheck(DID, rs_.gps2.streaming_pos);
        gps2_pos = *msg;
        if (rs_.gps2.enabled && msg->status & GPS_STATUS_FIX_MASK)
        {
            msg_gps2.header.stamp = ros_time_from_week_and_tow(msg->week, msg->timeOfWeekMs / 1.0e3);
            msg_gps2.week = msg->week;
            msg_gps2.status = msg->status;
            msg_gps2.header.frame_id = frame_id_;
            msg_gps2.num_sat = (uint8_t)(msg->status & GPS_STATUS_NUM_SATS_USED_MASK);
            msg_gps2.cno = msg->cnoMean;
            msg_gps2.latitude = msg->lla[0];
            msg_gps2.longitude = msg->lla[1];
            msg_gps2.altitude = msg->lla[2];
            msg_gps2.pos_ecef.x = ecef_[0] = msg->ecef[0];
            msg_gps2.pos_ecef.y = ecef_[1] = msg->ecef[1];
            msg_gps2.pos_ecef.z = ecef_[2] = msg->ecef[2];
            msg_gps2.hmsl = msg->hMSL;
            msg_gps2.hacc = msg->hAcc;
            msg_gps2.vacc = msg->vAcc;
            msg_gps2.pdop = msg->pDop;
            publishGPS2();
        }
        break;
    }

    if (primaryGpsDid == DID)
    {
        GPS_week_ = msg->week;
        GPS_towOffset_ = msg->towOffset;

        if (rs_.gps1_navsatfix.enabled)
        {
            msg_NavSatFix.header.stamp = ros_time_from_week_and_tow(msg->week, msg->timeOfWeekMs / 1.0e3);
            msg_NavSatFix.header.frame_id = frame_id_;
            msg_NavSatFix.status.status = -1;                           // Assume no Fix
            if (msg->status & GPS_STATUS_FIX_MASK >= GPS_STATUS_FIX_2D) // Check for fix and set
            {
                msg_NavSatFix.status.status = NavSatFixStatusFixType::STATUS_FIX;
            }

            if (msg->status & GPS_STATUS_FIX_SBAS) // Check for SBAS only fix
            {
                msg_NavSatFix.status.status = NavSatFixStatusFixType::STATUS_SBAS_FIX;
            }

            if (msg->status & GPS_STATUS_FIX_MASK >= GPS_STATUS_FIX_RTK_SINGLE) // Check for any RTK fix
            {
                msg_NavSatFix.status.status = NavSatFixStatusFixType::STATUS_GBAS_FIX;
            }

            // msg_NavSatFix.status.service - Service set at Node Startup
            msg_NavSatFix.latitude = msg->lla[0];
            msg_NavSatFix.longitude = msg->lla[1];
            msg_NavSatFix.altitude = msg->lla[2];

            // Diagonal Known
            const double varH = pow(msg->hAcc / 1000.0, 2);
            const double varV = pow(msg->vAcc / 1000.0, 2);
            msg_NavSatFix.position_covariance[0] = varH;
            msg_NavSatFix.position_covariance[4] = varH;
            msg_NavSatFix.position_covariance[8] = varV;
            msg_NavSatFix.position_covariance_type = COVARIANCE_TYPE_DIAGONAL_KNOWN;
            rs_.gps1_navsatfix.pub_nsf->publish(msg_NavSatFix);
        }
    }
}

void InertialSenseROS::GPS_vel_callback(eDataIDs DID, const gps_vel_t *const msg)
{
    switch (DID)
    {
    case DID_GPS1_VEL:
        rs_.gps1.streamingCheck(DID, rs_.gps1.streaming_vel);
        gps1_vel = *msg;
        if (rs_.gps1.enabled && abs(GPS_towOffset_) > 0.001)
        {
            gps1_velEcef.header.stamp = ros_time_from_week_and_tow(GPS_week_, msg->timeOfWeekMs / 1.0e3);
            gps1_velEcef.vector.x = msg->vel[0];
            gps1_velEcef.vector.y = msg->vel[1];
            gps1_velEcef.vector.z = msg->vel[2];
            publishGPS1();
        }
        break;

    case DID_GPS2_VEL:
        rs_.gps2.streamingCheck(DID, rs_.gps2.streaming_vel);
        gps2_vel = *msg;
        if (rs_.gps2.enabled && abs(GPS_towOffset_) > 0.001)
        {
            gps2_velEcef.header.stamp = ros_time_from_week_and_tow(GPS_week_, msg->timeOfWeekMs / 1.0e3);
            gps2_velEcef.vector.x = msg->vel[0];
            gps2_velEcef.vector.y = msg->vel[1];
            gps2_velEcef.vector.z = msg->vel[2];
            publishGPS2();





        }
        break;
    }
}

void InertialSenseROS::publishGPS1()
{
    double dt = (gps1_velEcef.header.stamp.sec - msg_gps1.header.stamp.sec);
    if (abs(dt) < 2.0e-3)
    {
        msg_gps1.vel_ecef = gps1_velEcef.vector;
        msg_gps1.sacc = gps1_vel.sAcc;
        if (rs_.gps1.pub_gps == NULL) {
            initialize();
        }
        if (rs_.gps1.pub_gps != NULL) {
            if (rs_.gps1.pub_gps->get_subscription_count() > 0)
                rs_.gps1.pub_gps->publish(msg_gps1);
        }

    }
}

void InertialSenseROS::publishGPS2()
{
    double dt = (gps2_velEcef.header.stamp.sec - msg_gps2.header.stamp.sec);
    if (abs(dt) < 2.0e-3)
    {
        msg_gps2.vel_ecef = gps2_velEcef.vector;
        msg_gps2.sacc = gps2_vel.sAcc;
        if (rs_.gps2.pub_gps != NULL) {
            if (rs_.gps2.pub_gps->get_subscription_count() > 0)
                rs_.gps2.pub_gps->publish(msg_gps2);
        }

    }
}

void InertialSenseROS::update()
{
    if (!IS_.IsOpen()) {
        IS_.Close();
        sdk_connected_ = false;
        sleep(1);
        initializeIS();
    }

    IS_.Update();
}

void InertialSenseROS::strobe_in_time_callback(eDataIDs DID, const strobe_in_time_t *const msg)
{
    switch (DID)
    {
    case DID_STROBE_IN_TIME:
       // STREAMING_CHECK(strobeInStreaming_, DID);

        if (abs(GPS_towOffset_) > 0.001)
        {
            auto strobe_msg = std_msgs::msg::Header();
            strobe_msg.stamp = ros_time_from_week_and_tow(msg->week, msg->timeOfWeekMs * 1.0e-3);
            strobe_pub_->publish(strobe_msg);
        }
        break;
    }
}

void InertialSenseROS::GPS_info_callback(eDataIDs DID, const gps_sat_t *const msg)
{
    switch (DID)
    {
    case DID_GPS1_SAT:  rs_.gps1_info.streamingCheck(DID); break;
    case DID_GPS2_SAT:  rs_.gps2_info.streamingCheck(DID); break;
    default: return;
    }
    // rs_.gps1_info.streamingCheck(DID);
    // rs_.gps2_info.streamingCheck(DID);
    if (abs(GPS_towOffset_) < 0.001)
    { // Wait for valid msg->timeOfWeekMs
        return;
    }

    msg_gps1_info.header.stamp = ros_time_from_tow(msg->timeOfWeekMs / 1.0e3);
    msg_gps1_info.header.frame_id = frame_id_;
    msg_gps1_info.num_sats = msg->numSats;
    for (int i = 0; i < 50; i++) {
        msg_gps1_info.sattelite_info[i].sat_id = msg->sat[i].svId;
        msg_gps1_info.sattelite_info[i].cno = msg->sat[i].cno;
    }

    switch (DID)
    {
    case DID_GPS1_SAT:  rs_.gps1_info.pub_gpsinfo1->publish(msg_gps1_info); break;
    case DID_GPS2_SAT:  rs_.gps2_info.pub_gpsinfo2->publish(msg_gps1_info); break;
    }
   // rs_.gps1_info.pub_gpsinfo1->publish(msg_gps1_info);
   // rs_.gps2_info.pub_gpsinfo2->publish(msg_gps1_info);
}

void InertialSenseROS::mag_callback(eDataIDs DID, const magnetometer_t *const msg)
{
    if (DID != DID_MAGNETOMETER)
    {
        return;
    }

    rs_.magnetometer.streamingCheck(DID);
    sensor_msgs::msg::MagneticField mag_msg;
    mag_msg.header.stamp = ros_time_from_start_time(msg->time);
    mag_msg.header.frame_id = frame_id_;
    mag_msg.magnetic_field.x = msg->mag[0];
    mag_msg.magnetic_field.y = msg->mag[1];
    mag_msg.magnetic_field.z = msg->mag[2];

    rs_.magnetometer.pub_bfield->publish(mag_msg);
}

void InertialSenseROS::baro_callback(eDataIDs DID, const barometer_t *const msg)
{
    if (DID != DID_BAROMETER)
    {
        return;
    }

    rs_.barometer.streamingCheck(DID);
    sensor_msgs::msg::FluidPressure baro_msg;
    baro_msg.header.stamp = ros_time_from_start_time(msg->time);
    baro_msg.header.frame_id = frame_id_;
    baro_msg.fluid_pressure = msg->bar;
    baro_msg.variance = msg->barTemp;
    if (rs_.barometer.pub_fpres != NULL)
        rs_.barometer.pub_fpres->publish(baro_msg);
}

void InertialSenseROS::preint_IMU_callback(eDataIDs DID, const pimu_t *const msg)
{
    imuStreaming_ = true;

    if (rs_.pimu.enabled)
    {
        rs_.pimu.streamingCheck(DID);
        msg_pimu.header.stamp = ros_time_from_start_time(msg->time);
        msg_pimu.header.frame_id = frame_id_;
        msg_pimu.dtheta.x = msg->theta[0];
        msg_pimu.dtheta.y = msg->theta[1];
        msg_pimu.dtheta.z = msg->theta[2];
        msg_pimu.dvel.x = msg->vel[0];
        msg_pimu.dvel.y = msg->vel[1];
        msg_pimu.dvel.z = msg->vel[2];
        msg_pimu.dt = msg->dt;
        rs_.pimu.pub_pimu->publish(msg_pimu);
    }

    if (rs_.imu.enabled)
    {
        rs_.imu.streamingCheck(DID);
        msg_imu.header.stamp = ros_time_from_start_time(msg->time);
        msg_imu.header.frame_id = frame_id_;
        if (msg->dt != 0.0f)
        {
            float div = 1.0f/msg->dt;
            msg_imu.angular_velocity.x = msg->theta[0]  * div;
            msg_imu.angular_velocity.y = msg->theta[1]  * div;
            msg_imu.angular_velocity.z = msg->theta[2]  * div;
            msg_imu.linear_acceleration.x = msg->vel[0] * div;
            msg_imu.linear_acceleration.y = msg->vel[1] * div;
            msg_imu.linear_acceleration.z = msg->vel[2] * div;
            rs_.imu.pub_imu->publish(msg_imu);
        }
    }
}

void InertialSenseROS::RTK_Misc_callback(eDataIDs DID, const gps_rtk_misc_t *const msg)
{
    inertial_sense_ros2::msg::RTKInfo rtk_info;
    if (abs(GPS_towOffset_) > 0.001)
    {
        rtk_info.header.stamp = ros_time_from_week_and_tow(GPS_week_, msg->timeOfWeekMs / 1000.0);
        rtk_info.base_ant_count = msg->baseAntennaCount;
        rtk_info.base_eph = msg->baseBeidouEphemerisCount + msg->baseGalileoEphemerisCount + msg->baseGlonassEphemerisCount + msg->baseGpsEphemerisCount;
        rtk_info.base_obs = msg->baseBeidouObservationCount + msg->baseGalileoObservationCount + msg->baseGlonassObservationCount + msg->baseGpsObservationCount;
        rtk_info.base_lla[0] = msg->baseLla[0];
        rtk_info.base_lla[1] = msg->baseLla[1];
        rtk_info.base_lla[2] = msg->baseLla[2];

        rtk_info.rover_eph = msg->roverBeidouEphemerisCount + msg->roverGalileoEphemerisCount + msg->roverGlonassEphemerisCount + msg->roverGpsEphemerisCount;
        rtk_info.rover_obs = msg->roverBeidouObservationCount + msg->roverGalileoObservationCount + msg->roverGlonassObservationCount + msg->roverGpsObservationCount;
        rtk_info.cycle_slip_count = msg->cycleSlipCount;
    }

    switch (DID)
    {
    case DID_GPS1_RTK_POS_MISC:
        rs_.rtk_pos.streamingCheck(DID);
        rs_.rtk_pos.pubInfo->publish(rtk_info);
        break;

    case DID_GPS2_RTK_CMP_MISC:
        rs_.rtk_cmp.streamingCheck(DID);
        rs_.rtk_cmp.pubInfo->publish(rtk_info);
        break;
    }
}

void InertialSenseROS::RTK_Rel_callback(eDataIDs DID, const gps_rtk_rel_t *const msg)
{
    inertial_sense_ros2::msg::RTKRel rtk_rel;
    std::string fixStatusString;
    if (abs(GPS_towOffset_) > 0.001)
    {
        rtk_rel.header.stamp = ros_time_from_week_and_tow(GPS_week_, msg->timeOfWeekMs / 1000.0);
        rtk_rel.differential_age = msg->differentialAge;
        rtk_rel.ar_ratio = msg->arRatio;
        uint32_t fixStatus = msg->status & GPS_STATUS_FIX_MASK;
        if (fixStatus == GPS_STATUS_FIX_3D)
        {
            rtk_rel.e_gps_status_fix = inertial_sense_ros2::msg::RTKRel::GPS_STATUS_FIX_3D;
            fixStatusString = "GPS_STATUS_FIX_3D";
        }
        else if (fixStatus == GPS_STATUS_FIX_RTK_SINGLE)
        {
            rtk_rel.e_gps_status_fix = inertial_sense_ros2::msg::RTKRel::GPS_STATUS_FIX_RTK_SINGLE;
            fixStatusString = "GPS_STATUS_FIX_RTK_SINGLE";
        }
        else if (fixStatus == GPS_STATUS_FIX_RTK_FLOAT)
        {
            rtk_rel.e_gps_status_fix = inertial_sense_ros2::msg::RTKRel::GPS_STATUS_FIX_RTK_FLOAT;
            fixStatusString = "GPS_STATUS_FIX_RTK_FLOAT";
        }
        else if (fixStatus == GPS_STATUS_FIX_RTK_FIX)
        {
            rtk_rel.e_gps_status_fix = inertial_sense_ros2::msg::RTKRel::GPS_STATUS_FIX_RTK_FIX;
            fixStatusString = "GPS_STATUS_FIX_RTK_FIX";
        }
        else if (msg->status & GPS_STATUS_FLAGS_RTK_FIX_AND_HOLD)
        {
            rtk_rel.e_gps_status_fix = inertial_sense_ros2::msg::RTKRel::GPS_STATUS_FLAGS_RTK_FIX_AND_HOLD;
            fixStatusString = "GPS_STATUS_FLAGS_RTK_FIX_AND_HOLD";
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
        rs_.rtk_pos.streamingCheck(DID, rs_.rtk_pos.streamingRel);
        rs_.rtk_pos.pubRel->publish(rtk_rel);

        diagnostics_.rtkPos_timeStamp = msg->timeOfWeekMs;
        diagnostics_.rtkPos_arRatio = rtk_rel.ar_ratio;
        diagnostics_.rtkPos_diffAge = rtk_rel.differential_age;
        diagnostics_.rtkPos_fixType = fixStatusString;
        diagnostics_.rtkPos_hdgBaseToRov = rtk_rel.heading_base_to_rover;
        diagnostics_.rtkPos_distanceToRover = rtk_rel.distance_base_to_rover;

        break;

    case DID_GPS2_RTK_CMP_REL:
        rs_.rtk_cmp.streamingCheck(DID, rs_.rtk_cmp.streamingRel);
        rs_.rtk_cmp.pubRel->publish(rtk_rel);

        diagnostics_.rtkCmp_timeStamp = msg->timeOfWeekMs;
        diagnostics_.rtkCmp_arRatio = rtk_rel.ar_ratio;
        diagnostics_.rtkCmp_diffAge = rtk_rel.differential_age;
        diagnostics_.rtkCmp_fixType = fixStatusString;
        diagnostics_.rtkCmp_hdgBaseToRov = rtk_rel.heading_base_to_rover;
        diagnostics_.rtkCmp_distanceToRover = rtk_rel.distance_base_to_rover;
        break;
    }

}

void InertialSenseROS::GPS_raw_callback(eDataIDs DID, const gps_raw_t *const msg)
{
    switch (DID)
    {
    case DID_GPS1_RAW:        rs_.gps1_raw.streamingCheck(DID);     break;
    case DID_GPS2_RAW:        rs_.gps2_raw.streamingCheck(DID);     break;
    case DID_GPS_BASE_RAW:    rs_.gpsbase_raw.streamingCheck(DID);  break;
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
            GPS_obs_bundle_timer_callback(/*ros::TimerEvent()*/);
        }
        break;

    case DID_GPS2_RAW:
        if (gps2_obs_Vec_.obs.size() > 0 &&
            (msg[0].time.time != gps2_obs_Vec_.obs[0].time.time ||
            msg[0].time.sec != gps2_obs_Vec_.obs[0].time.sec))
        {
            GPS_obs_bundle_timer_callback(/*ros::TimerEvent()*/);
        }
        break;

    case DID_GPS_BASE_RAW:
       if (base_obs_Vec_.obs.size() > 0 &&
            (msg[0].time.time != base_obs_Vec_.obs[0].time.time ||
            msg[0].time.sec != base_obs_Vec_.obs[0].time.sec))
        {
            GPS_obs_bundle_timer_callback(/*ros::TimerEvent()*/);
        }
        break;
    }

    for (int i = 0; i < nObs; i++)
    {
        inertial_sense_ros2::msg::GNSSObservation obs;
        obs.header.stamp = ros_time_from_gtime(msg[i].time.time, msg[i].time.sec);
        obs.time.time = msg[i].time.time;
        obs.time.sec = msg[i].time.sec;
        obs.sat = msg[i].sat;
        obs.rcv = msg[i].rcv;
        obs.snrr = msg[i].SNR[0];
        obs.lli = msg[i].LLI[0];
        obs.code = msg[i].code[0];
        obs.qual_l = msg[i].qualL[0];
        obs.qual_p = msg[i].qualP[0];
        obs.l = msg[i].L[0];
        obs.p = msg[i].P[0];
        obs.d = msg[i].D[0];
        switch (DID)
        {
        case DID_GPS1_RAW:
            gps1_obs_Vec_.obs.push_back(obs);
            last_obs_time_1_ = nh_->now();
            break;
        case DID_GPS2_RAW:
            gps2_obs_Vec_.obs.push_back(obs);
            last_obs_time_2_ = nh_->now();
            break;
        case DID_GPS_BASE_RAW:
            base_obs_Vec_.obs.push_back(obs);
            last_obs_time_base_ = nh_->now();
            break;
        }
    }
}

void InertialSenseROS::GPS_obs_bundle_timer_callback()
{
    if (gps1_obs_Vec_.obs.size() != 0)
    {
        if (abs((nh_->now().seconds() - last_obs_time_1_.seconds()) > 1e-2))
        {
            gps1_obs_Vec_.header.stamp = ros_time_from_gtime(gps1_obs_Vec_.obs[0].time.time, gps1_obs_Vec_.obs[0].time.sec);
            gps1_obs_Vec_.time = gps1_obs_Vec_.obs[0].time;
            rs_.gps1_raw.pubObs->publish(gps1_obs_Vec_);
            gps1_obs_Vec_.obs.clear();
        }
    }
    if (gps2_obs_Vec_.obs.size() != 0)
    {
        if (abs((nh_->now().seconds() - last_obs_time_2_.seconds()) > 1e-2))
        {
            gps2_obs_Vec_.header.stamp = ros_time_from_gtime(gps2_obs_Vec_.obs[0].time.time, gps2_obs_Vec_.obs[0].time.sec);
            gps2_obs_Vec_.time = gps2_obs_Vec_.obs[0].time;
            rs_.gps2_raw.pubObs->publish(gps2_obs_Vec_);
            gps2_obs_Vec_.obs.clear();
        }
    }
    if (base_obs_Vec_.obs.size() != 0)
    {
        if (abs((nh_->now().seconds() - last_obs_time_base_.seconds()) > 1e-2))
        {
            base_obs_Vec_.header.stamp = ros_time_from_gtime(base_obs_Vec_.obs[0].time.time, base_obs_Vec_.obs[0].time.sec);
            base_obs_Vec_.time = base_obs_Vec_.obs[0].time;
            rs_.gpsbase_raw.pubObs->publish(base_obs_Vec_);
            base_obs_Vec_.obs.clear();
        }
    }
}

void InertialSenseROS::GPS_eph_callback(eDataIDs DID, const eph_t *const msg)
{
    inertial_sense_ros2::msg::GNSSEphemeris eph;
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
    eph.a = msg->A;
    eph.e = msg->e;
    eph.i0 = msg->i0;
    eph.omg_0 = msg->OMG0;
    eph.omg = msg->omg;
    eph.m_0 = msg->M0;
    eph.deln = msg->deln;
    eph.omg_d = msg->OMGd;
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
    eph.a_dot = msg->Adot;
    eph.ndot = msg->ndot;
    switch (DID)
    {
    case DID_GPS1_RAW:      rs_.gps1_raw.pubEph->publish(eph);        break;
    case DID_GPS2_RAW:      rs_.gps2_raw.pubEph->publish(eph);        break;
    case DID_GPS_BASE_RAW:  rs_.gpsbase_raw.pubEph->publish(eph);    break;
    }
}

void InertialSenseROS::GPS_geph_callback(eDataIDs DID, const geph_t *const msg)
{
    inertial_sense_ros2::msg::GlonassEphemeris geph;
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
    case DID_GPS1_RAW:      rs_.gps1_raw.pubGEp->publish(geph);       break;
    case DID_GPS2_RAW:      rs_.gps2_raw.pubGEp->publish(geph);       break;
    case DID_GPS_BASE_RAW:  rs_.gpsbase_raw.pubGEp->publish(geph);   break;
    }
}

void InertialSenseROS::diagnostics_callback(/*const ros::TimerEvent &event */)
{
    if (!diagnosticsStreaming_)
    {
        diagnosticsStreaming_ = true;
        RCLCPP_INFO(rclcpp::get_logger("diag_response_received"),"InertialSenseROS: Diagnostics response received");
    }
    // Create diagnostic objects
    diagnostic_msgs::msg::DiagnosticArray diag_array;
    diag_array.header.stamp = nh_->now();

    //Create Status
    diagnostic_msgs::msg::DiagnosticStatus rtkDiagnostics;
    rtkDiagnostics.name = "RTK Diagnostics";
    rtkDiagnostics.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    rtkDiagnostics.message = "RTK Rel Data";

    // CNO mean
    diagnostic_msgs::msg::KeyValue cno_mean;
    cno_mean.key = "CNO Mean";
    cno_mean.value = std::to_string(msg_gps1.cno);
    rtkDiagnostics.values.push_back(cno_mean);

    //Timestamp
    diagnostic_msgs::msg::KeyValue rtkPos_timeStamp;
    rtkPos_timeStamp.key = "RTK Pos Timestamp";
    rtkPos_timeStamp.value = std::to_string(diagnostics_.rtkPos_timeStamp);
    rtkDiagnostics.values.push_back(rtkPos_timeStamp);

    // RTK Pos AR Ratio
    diagnostic_msgs::msg::KeyValue rtkPos_arRatio;
    rtkPos_arRatio.key = "RTK Pos AR Ratio";
    rtkPos_arRatio.value = std::to_string(diagnostics_.rtkPos_arRatio);
    rtkDiagnostics.values.push_back(rtkPos_arRatio);

    // RTK Pos Diff Age
    diagnostic_msgs::msg::KeyValue rtkPos_diffAge;
    rtkPos_diffAge.key = "RTK Pos Diff Age";
    rtkPos_diffAge.value = std::to_string(diagnostics_.rtkPos_diffAge);
    rtkDiagnostics.values.push_back(rtkPos_diffAge);

    // RTK Pos Fix Type
    diagnostic_msgs::msg::KeyValue rtkPos_fixType;
    rtkPos_fixType.key = "RTK Pos Fix Type";
    rtkPos_fixType.value = diagnostics_.rtkPos_fixType;
    rtkDiagnostics.values.push_back(rtkPos_fixType);

    // RTK Pos Hdg Base to Rover
    diagnostic_msgs::msg::KeyValue rtkPos_hdgBaseToRov;
    rtkPos_hdgBaseToRov.key = "RTK Pos Hdg Base to Rover";
    rtkPos_hdgBaseToRov.value = std::to_string(diagnostics_.rtkPos_hdgBaseToRov);
    rtkDiagnostics.values.push_back(rtkPos_hdgBaseToRov);

    // RTK Pos Distance Base to Rover
    diagnostic_msgs::msg::KeyValue rtkPos_distanceToRover;
    rtkPos_distanceToRover.key = "RTK Pos Distance to Rover";
    rtkPos_distanceToRover.value = std::to_string(diagnostics_.rtkPos_distanceToRover);
    rtkDiagnostics.values.push_back(rtkPos_distanceToRover);

    //Timestamp
    diagnostic_msgs::msg::KeyValue rtkCmp_timeStamp;
    rtkCmp_timeStamp.key = "RTK Cmp Timestamp";
    rtkCmp_timeStamp.value = std::to_string(diagnostics_.rtkCmp_timeStamp);
    rtkDiagnostics.values.push_back(rtkCmp_timeStamp);

    // RTK Cmp AR Ratio
    diagnostic_msgs::msg::KeyValue rtkCmp_arRatio;
    rtkCmp_arRatio.key = "RTK Cmp AR Ratio";
    rtkCmp_arRatio.value = std::to_string(diagnostics_.rtkCmp_arRatio);
    rtkDiagnostics.values.push_back(rtkCmp_arRatio);

    // RTK Cmp Diff Age
    diagnostic_msgs::msg::KeyValue rtkCmp_diffAge;
    rtkCmp_diffAge.key = "RTK Cmp Diff Age";
    rtkCmp_diffAge.value = std::to_string(diagnostics_.rtkCmp_diffAge);
    rtkDiagnostics.values.push_back(rtkCmp_diffAge);

    // RTK Cmp Fix Type
    diagnostic_msgs::msg::KeyValue rtkCmp_fixType;
    rtkCmp_fixType.key = "RTK Cmp Fix Type";
    rtkCmp_fixType.value = diagnostics_.rtkCmp_fixType;
    rtkDiagnostics.values.push_back(rtkCmp_fixType);

    // RTK Cmp Hdg Base to Rover
    diagnostic_msgs::msg::KeyValue rtkCmp_hdgBaseToRov;
    rtkCmp_hdgBaseToRov.key = "RTK Cmp Hdg Base to Rover";
    rtkCmp_hdgBaseToRov.value = std::to_string(diagnostics_.rtkCmp_hdgBaseToRov);
    rtkDiagnostics.values.push_back(rtkCmp_hdgBaseToRov);

    // RTK Cmp Distance Base to Rover
    diagnostic_msgs::msg::KeyValue rtkCmp_distanceToRover;
    rtkCmp_distanceToRover.key = "RTK Cmp Distance to Rover";
    rtkCmp_distanceToRover.value = std::to_string(diagnostics_.rtkCmp_distanceToRover);
    rtkDiagnostics.values.push_back(rtkCmp_distanceToRover);

    diag_array.status.push_back(rtkDiagnostics);
    rs_.diagnostics.pub_diagnostics->publish(diag_array);
}

bool InertialSenseROS::set_current_position_as_refLLA(std_srvs::srv::Trigger::Request &req, std_srvs::srv::Trigger::Response &res)
{
    (void)req;
    double current_lla_[3];
    current_lla_[0] = lla_[0];
    current_lla_[1] = lla_[1];
    current_lla_[2] = lla_[2];

    IS_.SendData(DID_FLASH_CONFIG, reinterpret_cast<uint8_t *>(&current_lla_), sizeof(current_lla_), offsetof(nvm_flash_cfg_t, refLla));

    comManagerGetData(0, DID_FLASH_CONFIG, 0, 0, 0);

    int i = 0;
    nvm_flash_cfg_t current_flash;
    IS_.FlashConfig(current_flash);
    while (current_flash.refLla[0] == current_flash.refLla[0] && current_flash.refLla[1] == current_flash.refLla[1] && current_flash.refLla[2] == current_flash.refLla[2])
    {
        comManagerStep();
        i++;
        if (i > 100)
        {
            break;
        }
    }

    if (current_lla_[0] == current_flash.refLla[0] && current_lla_[1] == current_flash.refLla[1] && current_lla_[2] == current_flash.refLla[2])
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

bool InertialSenseROS::set_refLLA_to_value(inertial_sense_ros2::srv::RefLLAUpdate::Request::SharedPtr req, inertial_sense_ros2::srv::RefLLAUpdate::Response::SharedPtr res)
{
    IS_.SendData(DID_FLASH_CONFIG, reinterpret_cast<uint8_t *>(&req->lla), sizeof(req->lla), offsetof(nvm_flash_cfg_t, refLla));

    comManagerGetData(0, DID_FLASH_CONFIG, 0, 0, 0);

    int i = 0;
    nvm_flash_cfg_t current_flash;
    IS_.FlashConfig(current_flash);
    while (current_flash.refLla[0] == current_flash.refLla[0] && current_flash.refLla[1] == current_flash.refLla[1] && current_flash.refLla[2] == current_flash.refLla[2])
    {
        comManagerStep();
        i++;
        if (i > 100)
        {
            break;
        }
    }

   if (req->lla[0] == current_flash.refLla[0] && req->lla[1] == current_flash.refLla[1] && req->lla[2] == current_flash.refLla[2])
   {
       comManagerGetData(0, DID_FLASH_CONFIG, 0, 0, 0);
       res->success = true;
       res->message = ("Update was succesful.  refLla: Lat: " + std::to_string(req->lla[0]) + "  Lon: " + std::to_string(req->lla[1]) + "  Alt: " + std::to_string(req->lla[2]));
   }
   else
   {
       comManagerGetData(0, DID_FLASH_CONFIG, 0, 0, 0);
       res->success = false;
       res->message = "Unable to update refLLA. Please try again.";
   }

    return true;
}

bool InertialSenseROS::perform_mag_cal_srv_callback(std_srvs::srv::Trigger::Request::SharedPtr req, std_srvs::srv::Trigger::Response::SharedPtr res)
{
    (void)req;
    uint32_t single_axis_command = 2;
    IS_.SendData(DID_MAG_CAL, reinterpret_cast<uint8_t *>(&single_axis_command), sizeof(uint32_t), offsetof(mag_cal_t, state));

    is_comm_instance_t comm;
    uint8_t buffer[2048];
    is_comm_init(&comm, buffer, sizeof(buffer));
    serial_port_t *serialPort = IS_.SerialPort();
    uint8_t inByte;
    int n;

    while ((n = serialPortReadCharTimeout(serialPort, &inByte, 20)) > 0)
    {
        // Search comm buffer for valid packets
        if (is_comm_parse_byte(&comm, inByte) == _PTYPE_INERTIAL_SENSE_DATA && comm.rxPkt.dataHdr.id == DID_INS_1)
        {
            ins_1_t *msg = (ins_1_t *)(comm.rxPkt.data.ptr + comm.rxPkt.offset);
            if (msg->insStatus & 0x00400000)
            {
                res->success = true;
                res->message = "Successfully initiated mag recalibration.";
                return true;
            }
        }
    }

    return true;
}

bool InertialSenseROS::perform_multi_mag_cal_srv_callback(std_srvs::srv::Trigger::Request::SharedPtr req, std_srvs::srv::Trigger::Response::SharedPtr res)
{
    (void)req;
    uint32_t multi_axis_command = 1;
    IS_.SendData(DID_MAG_CAL, reinterpret_cast<uint8_t *>(&multi_axis_command), sizeof(uint32_t), offsetof(mag_cal_t, state));

    is_comm_instance_t comm;
    uint8_t buffer[2048];
    is_comm_init(&comm, buffer, sizeof(buffer));
    serial_port_t *serialPort = IS_.SerialPort();
    uint8_t inByte;
    int n;

    while ((n = serialPortReadCharTimeout(serialPort, &inByte, 20)) > 0)
    {
        // Search comm buffer for valid packets
        if (is_comm_parse_byte(&comm, inByte) == _PTYPE_INERTIAL_SENSE_DATA && comm.rxPkt.dataHdr.id == DID_INS_1)
        {
            ins_1_t *msg = (ins_1_t *)(comm.rxPkt.data.ptr + comm.rxPkt.offset);
            if (msg->insStatus & 0x00400000)
            {
                res->success = true;
                res->message = "Successfully initiated mag recalibration.";
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

    RCLCPP_WARN(rclcpp::get_logger("device_reset_required"),"Device reset required.\n\nDisconnecting from device.\n");
    sleep(2);
    IS_.Close();
}

bool InertialSenseROS::update_firmware_srv_callback(inertial_sense_ros2::srv::FirmwareUpdate::Request &req, inertial_sense_ros2::srv::FirmwareUpdate::Response &res)
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

rclcpp::Time InertialSenseROS::ros_time_from_week_and_tow(const uint32_t week, const double timeOfWeek)
{
    rclcpp::Time rostime(0, 0);
    //  If we have a GPS fix, then use it to set timestamp
    if (abs(GPS_towOffset_) > 0.001)
    {
        uint64_t sec = UNIX_TO_GPS_OFFSET + floor(timeOfWeek) + week * 7 * 24 * 3600;
        uint64_t nsec = (timeOfWeek - floor(timeOfWeek)) * 1e9;
        rostime = rclcpp::Time(sec, nsec);
    }
    else
    {
        // Otherwise, estimate the IMX boot time and offset the messages
        if (!got_first_message_)
        {
            got_first_message_ = true;
            INS_local_offset_ = nh_->now().seconds() - timeOfWeek;
        }
        else // low-pass filter offset to account for drift
        {
            double y_offset = nh_->now().seconds() - timeOfWeek;
            INS_local_offset_ = 0.005 * y_offset + 0.995 * INS_local_offset_;
        }
        // Publish with ROS time
        rostime = rclcpp::Time(INS_local_offset_ + timeOfWeek);
    }
    return rostime;
}

rclcpp::Time InertialSenseROS::ros_time_from_start_time(const double time)
{
    rclcpp::Time rostime(0, 0);

    //  If we have a GPS fix, then use it to set timestamp
    if (abs(GPS_towOffset_) > 0.001)
    {
        double timeOfWeek = time + GPS_towOffset_;
        uint64_t sec = (uint64_t)(UNIX_TO_GPS_OFFSET + floor(timeOfWeek) + GPS_week_ * 7 * 24 * 3600);
        uint64_t nsec = (uint64_t)((timeOfWeek - floor(timeOfWeek)) * 1.0e9);
        rostime = rclcpp::Time(sec, nsec);
    }
    else
    {
        // Otherwise, estimate the IMX boot time and offset the messages
        if (!got_first_message_)
        {
            got_first_message_ = true;
            INS_local_offset_ = nh_->now().seconds() - time;
        }
        else // low-pass filter offset to account for drift
        {
            double y_offset = nh_->now().seconds() - time;
            INS_local_offset_ = 0.005 * y_offset + 0.995 * INS_local_offset_;
        }
        // Publish with ROS time
        rostime = rclcpp::Time(INS_local_offset_ + time);
    }
    return rostime;
}

rclcpp::Time InertialSenseROS::ros_time_from_tow(const double tow)
{
    return ros_time_from_week_and_tow(GPS_week_, tow);
}

double InertialSenseROS::tow_from_ros_time(const rclcpp::Time &rt)
{
    return ((uint64_t)rt.seconds() - UNIX_TO_GPS_OFFSET - GPS_week_ * 604800) + rt.nanoseconds() * 1.0e-9;
}

rclcpp::Time InertialSenseROS::ros_time_from_gtime(const uint64_t sec, double subsec)
{
    int32_t seconds = sec - LEAP_SECONDS;
    uint32_t nanosec = subsec * 1e9;
    rclcpp::Time out(seconds, nanosec);


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


