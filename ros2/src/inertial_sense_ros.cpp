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
#include <ISPose.h>
#include <ISEarth.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "ISMatrix.h"
#include "ISEarth.h"

#include "ParamHelper.h"

using namespace std::chrono_literals;

#define STREAMING_CHECK(streaming, DID)      if(!streaming){ streaming = true; RCLCPP_DEBUG(this->get_logger(), "InertialSenseROS: %s response received", cISDataMappings::GetDataSetName(DID)); }

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

InertialSenseROS::InertialSenseROS(YAML::Node paramNode) : Node("inertial_sense")
{
    nh_private_ = this->create_sub_node("sub");
    br = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    // Should always be enabled by default
    rs_.did_ins1.enabled = true;
    rs_.did_ins1.topic = "did_ins1";
    rs_.gps1.enabled = true;
    rs_.gps1.topic = "/gps";

    if (rcutils_logging_set_logger_level(this->get_logger().get_name(), RCUTILS_LOG_SEVERITY_DEBUG) != RCUTILS_RET_OK){
        RCLCPP_ERROR(this->get_logger(), "Error setting severity: %s", rcutils_get_error_string().str);
        rcutils_reset_error();
    }
    load_params(paramNode);
}

void InertialSenseROS::initialize(bool configFlashParameters)
{
    RCLCPP_INFO(this->get_logger(), "======  Starting Inertial Sense ROS2  ======");

    initializeIS(configFlashParameters);
    if (sdk_connected_) {
        initializeROS();

        if (log_enabled_) {
            start_log();    // Start log should happen last
        }

        // configure_ascii_output(); // Currently not functional
    }
}

void InertialSenseROS::terminate() {
    IS_.Close();
    IS_.CloseServerConnection();
    sdk_connected_ = false;

    // ROS equivelant to shutdown advertisers, etc.
}

void InertialSenseROS::initializeIS(bool configFlashParameters) {
    if (connect()) {
        // Check protocol and firmware version
        firmware_compatiblity_check();

        IS_.StopBroadcasts(true);
        configure_data_streams(true);
        configure_rtk();
        IS_.SavePersistent();

        if (configFlashParameters)
        {   // Set IMX flash parameters (flash write) after everything else so processor stall doesn't interfere with communications.
            configure_flash_parameters();
        }
    }
}

void InertialSenseROS::initializeROS() {
    //////////////////////////////////////////////////////////
    // Start Up ROS service servers
    refLLA_set_current_srv_         = create_service<std_srvs::srv::Trigger>("set_refLLA_current", std::bind(&InertialSenseROS::set_current_position_as_refLLA, this, _1, _2));
    refLLA_set_value_srv_           = create_service<inertial_sense_ros::srv::RefLLAUpdate>("set_refLLA_value", std::bind(&InertialSenseROS::set_refLLA_to_value, this, _1, _2));
    mag_cal_srv_                    = create_service<std_srvs::srv::Trigger>("single_axis_mag_cal", std::bind(&InertialSenseROS::perform_mag_cal_srv_callback, this, _1, _2));
    multi_mag_cal_srv_              = create_service<std_srvs::srv::Trigger>("multi_axis_mag_cal", std::bind(&InertialSenseROS::perform_multi_mag_cal_srv_callback, this, _1, _2));
    //firmware_update_srv_            = create_service<inertial_sense_ros::srv::FirmwareUpdate>("firmware_update", std::bind(&InertialSenseROS::update_firmware_srv_callback, this, _1, _2));

    SET_CALLBACK(DID_STROBE_IN_TIME, strobe_in_time_t, strobe_in_time_callback, 1); // we always want the strobe

    //////////////////////////////////////////////////////////
    // Publishers
    strobe_pub_ = create_publisher<std_msgs::msg::Header>(rs_.strobe_in.topic, 1);

    if (rs_.did_ins1.enabled)               { rs_.did_ins1.pub = create_publisher<inertial_sense_ros::msg::DIDINS1>(rs_.did_ins1.topic, 1); }
    if (rs_.did_ins2.enabled)               { rs_.did_ins2.pub = create_publisher<inertial_sense_ros::msg::DIDINS2>(rs_.did_ins2.topic, 1); }
    if (rs_.did_ins4.enabled)               { rs_.did_ins4.pub = create_publisher<inertial_sense_ros::msg::DIDINS4>(rs_.did_ins4.topic, 1); }
    if (rs_.odom_ins_ned.enabled)           { rs_.odom_ins_ned.pub = create_publisher<nav_msgs::msg::Odometry>(rs_.odom_ins_ned.topic, 1); }
    if (rs_.odom_ins_enu.enabled)           { rs_.odom_ins_enu.pub = create_publisher<nav_msgs::msg::Odometry>(rs_.odom_ins_enu.topic, 1); }
    if (rs_.odom_ins_ecef.enabled)          { rs_.odom_ins_ecef.pub = create_publisher<nav_msgs::msg::Odometry>(rs_.odom_ins_ecef.topic, 1); }
    if (rs_.inl2_states.enabled)            { rs_.inl2_states.pub = create_publisher<inertial_sense_ros::msg::INL2States>(rs_.inl2_states.topic, 1); }

    if (rs_.pimu.enabled)                   { rs_.pimu.pub = create_publisher<inertial_sense_ros::msg::PIMU>(rs_.pimu.topic, 1); }
    if (rs_.imu.enabled)                    { rs_.imu.pub = create_publisher<sensor_msgs::msg::Imu>(rs_.imu.topic, 1); }
    if (rs_.magnetometer.enabled)           { rs_.magnetometer.pub = create_publisher<sensor_msgs::msg::MagneticField>(rs_.magnetometer.topic, 1); }
    if (rs_.barometer.enabled)              { rs_.barometer.pub = create_publisher<sensor_msgs::msg::FluidPressure>(rs_.barometer.topic, 1); }

    if (rs_.gps1.enabled)                   { rs_.gps1.pub = create_publisher<inertial_sense_ros::msg::GPS>(rs_.gps1.topic, 1); }
    if (rs_.gps1_navsatfix.enabled)         { rs_.gps1_navsatfix.pub = create_publisher<sensor_msgs::msg::NavSatFix>(rs_.gps1_navsatfix.topic, 1); }
    if (rs_.gps1_info.enabled)              { rs_.gps1_info.pub = create_publisher<inertial_sense_ros::msg::GPSInfo>(rs_.gps1_info.topic, 1); }

    if (rs_.gps2.enabled)                   { rs_.gps2.pub = create_publisher<inertial_sense_ros::msg::GPS>(rs_.gps2.topic, 1); }
    if (rs_.gps2_navsatfix.enabled)         { rs_.gps2_navsatfix.pub = create_publisher<sensor_msgs::msg::NavSatFix>(rs_.gps2_navsatfix.topic, 1); }
    if (rs_.gps2_info.enabled)              { rs_.gps2_info.pub = create_publisher<inertial_sense_ros::msg::GPSInfo>(rs_.gps2_info.topic, 1); }

    if (RTK_rover_ && RTK_rover_->positioning_enable )
    {
        rs_.rtk_pos.pubInfo = create_publisher<inertial_sense_ros::msg::RTKInfo>("RTK_pos/info", 10);
        rs_.rtk_pos.pubRel = create_publisher<inertial_sense_ros::msg::RTKRel>("RTK_pos/rel", 10);
    }
    if (GNSS_Compass_)
    {
        rs_.rtk_cmp.pubInfo = create_publisher<inertial_sense_ros::msg::RTKInfo>("RTK_cmp/info", 10);
        rs_.rtk_cmp.pubRel = create_publisher<inertial_sense_ros::msg::RTKRel>("RTK_cmp/rel", 10);
    }

    if (rs_.gps1_raw.enabled)
    {
        rs_.gps1_raw.pubObs = create_publisher<inertial_sense_ros::msg::GNSSObsVec>(rs_.gps1_raw.topic + "/obs", 50);
        rs_.gps1_raw.pubEph = create_publisher<inertial_sense_ros::msg::GNSSEphemeris>(rs_.gps1_raw.topic + "/eph", 50);
        rs_.gps1_raw.pubGEp = create_publisher<inertial_sense_ros::msg::GlonassEphemeris>(rs_.gps1_raw.topic + "/geph", 50);
        obs_bundle_timer_ = create_wall_timer(1ms, std::bind(&InertialSenseROS::GPS_obs_bundle_timer_callback, this));
    }
    if (rs_.gps2_raw.enabled)
    {
        rs_.gps2_raw.pubObs = create_publisher<inertial_sense_ros::msg::GNSSObsVec>(rs_.gps2_raw.topic + "/obs", 50);
        rs_.gps2_raw.pubEph = create_publisher<inertial_sense_ros::msg::GNSSEphemeris>(rs_.gps2_raw.topic + "/eph", 50);
        rs_.gps2_raw.pubGEp = create_publisher<inertial_sense_ros::msg::GlonassEphemeris>(rs_.gps2_raw.topic + "/geph", 50);
        obs_bundle_timer_ = create_wall_timer(1ms, std::bind(&InertialSenseROS::GPS_obs_bundle_timer_callback, this));
    }
    if (rs_.gpsbase_raw.enabled)
    {
        rs_.gpsbase_raw.pubObs = create_publisher<inertial_sense_ros::msg::GNSSObsVec>("gps/base_gobs", 50);
        rs_.gpsbase_raw.pubEph = create_publisher<inertial_sense_ros::msg::GNSSEphemeris>("gps/base_eph", 50);
        rs_.gpsbase_raw.pubGEp = create_publisher<inertial_sense_ros::msg::GlonassEphemeris>("gps/base_geph", 50);
        obs_bundle_timer_ = create_wall_timer(1ms, std::bind(&InertialSenseROS::GPS_obs_bundle_timer_callback, this));
    }
    if (rs_.diagnostics.enabled)
    {
        rs_.diagnostics.pub = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("diagnostics", 1);
        diagnostics_timer_ = create_wall_timer(500ms, std::bind(&InertialSenseROS::diagnostics_callback, this)); // 2 Hz
    }

    data_stream_timer_ = create_wall_timer(1s, std::bind(static_cast<void (InertialSenseROS::*)()>(&InertialSenseROS::configure_data_streams), this));
}

void InertialSenseROS::load_params(YAML::Node &node)
{
    // Load parameters from yaml node if provided.  Otherwise load from ROS parameter server.
    bool useParamSvr = !node.IsDefined();

    if (useParamSvr)
    {
        RCLCPP_WARN(this->get_logger(), "Loading for ROS Parameter Server is Not implemented yet." );
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "InertialSenseROS: Loading configuration from YAML tree." );
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
        std::string param = "";
        ph.nodeParam("port", param, "/dev/ttyACM0");
        ports_.push_back(param);
    }

    ph.nodeParam("baudrate", baudrate_, 921600);
    ph.nodeParam("frame_id", frame_id_, "body");
    ph.nodeParam("enable_log", log_enabled_, false);

    // advanced Parameters
    ph.nodeParam("ioConfig", ioConfigBits_, 0x0244a060);     // EVB2: GPS1 Ser1 F9P, GPS2 disabled F9P, PPS G8
    ph.nodeParam("RTKCfgBits", rtkConfigBits_, 0);            // rtk config bits
    ph.nodeParam("wheelCfgBits", wheelConfigBits_, 0);        // wheel-encoder config bits

    ph.nodeParam("mag_declination", magDeclination_);
    ph.nodeParamVec("ref_lla", 3, refLla_);
    ph.nodeParam("publishTf", publishTf_);
    ph.nodeParam("platformConfig", platformConfig_);

    // Sensors
    YAML::Node sensorsNode = ph.node(node, "sensors");
    YAML::Node sensorsMsgs = ph.node(sensorsNode, "messages", 2);
    ph.msgParams(rs_.imu, "imu");
    ph.msgParams(rs_.pimu, "pimu");
    ph.msgParams(rs_.magnetometer, "magnetometer", "mag");
    ph.msgParams(rs_.barometer, "barometer", "baro");
    ph.msgParams(rs_.strobe_in, "strobe_in");
    node["sensors"]["messages"] = sensorsMsgs;

    // INS
    YAML::Node insNode = ph.node(node, "ins");
    ph.nodeParamVec("rotation", 3, insRotation_);
    ph.nodeParamVec("offset", 3, insOffset_);
    ph.nodeParam("navigation_dt_ms", ins_nav_dt_ms_, 4);

    std::vector<std::string> dyn_model_set{ "INS_DYN_MODEL_PORTABLE",
                                            " << UNKNOWN >> ",
                                            "INS_DYN_MODEL_STATIONARY",
                                            "INS_DYN_MODEL_PEDESTRIAN",
                                            "INS_DYN_MODEL_GROUND_VEHICLE",
                                            "INS_DYN_MODEL_MARINE",
                                            "INS_DYN_MODEL_AIRBORNE_1G",
                                            "INS_DYN_MODEL_AIRBORNE_2G",
                                            "INS_DYN_MODEL_AIRBORNE_4G",
                                            "INS_DYN_MODEL_WRIST",
                                            "INS_DYN_MODEL_INDOOR" };

    ph.nodeParamEnum("dynamic_model", insDynModel_, dyn_model_set, INS_DYN_MODEL_AIRBORNE_4G);
    ph.nodeParam("enable_covariance", covariance_enabled_, false);
    YAML::Node insMsgs = ph.node(insNode, "messages", 2);
    ph.msgParams(rs_.odom_ins_enu, "odom_ins_enu");
    ph.msgParams(rs_.odom_ins_ned, "odom_ins_ned");
    ph.msgParams(rs_.odom_ins_ecef, "odom_ins_ecef");
    ph.msgParams(rs_.did_ins1, "did_ins1", "ins_eul_uvw_ned");
    ph.msgParams(rs_.did_ins2, "did_ins2", "ins_quat_uvw_lla");
    ph.msgParams(rs_.did_ins4, "did_ins4", "ins_quat_ve_ecef", true);
    ph.msgParams(rs_.inl2_states, "inl2_states");
    insNode["messages"] = insMsgs;
    node["ins"] = insNode;

    // GPS 1
    YAML::Node gps1Node = ph.node(node, "gps1");
    ph.nodeParam("type", rs_.gps1.type);
    ph.nodeParam("gpsTimeUserDelay", gpsTimeUserDelay_);
    ph.nodeParamVec("antenna_offset", 3, rs_.gps1.antennaOffset);
    YAML::Node gps1Msgs = ph.node(gps1Node, "messages", 2);
    ph.msgParams(rs_.gps1, "pos_vel", "gps1/pos_vel");
    ph.msgParams(rs_.gps1_info, "info", "gps1/info");
    ph.msgParams(rs_.gps1_raw, "raw", "gps1/raw");
    ph.msgParams(rs_.gps1_navsatfix, "navsatfix", "gps1/NavSatFix");
    gps1Node["messages"] = gps1Msgs;
    node["gps1"] = gps1Node;

    // GPS 2
    YAML::Node gps2Node = ph.node(node, "gps2");
    ph.nodeParam("type", rs_.gps2.type);
    ph.nodeParamVec("antenna_offset", 3, rs_.gps2.antennaOffset);
    YAML::Node gps2Msgs = ph.node(gps2Node, "messages", 2);
    ph.msgParams(rs_.gps2, "pos_vel", "gps2/pos_vel");
    ph.msgParams(rs_.gps2_info, "info", "gps2/info");
    ph.msgParams(rs_.gps2_raw, "raw", "gps2/raw");
    ph.msgParams(rs_.gps2_navsatfix, "navsatfix", "gps2/NavSatFix");
    gps2Node["messages"] = gps2Msgs;
    node["gps2"] = gps2Node;

    YAML::Node evbNode = ph.node(node, "evb");
    ph.nodeParam("cb_preset", evb_.cb_preset, 2);        // 2=RS232(default), 3=XBee Radio On, 4=WiFi On & RS422, 5=SPI, 6=USB hub, 7=USB hub w/ RS422, 8=all off but USB
    ph.nodeParam("cb_options", evb_.cb_options, 0);

    YAML::Node rtkRoverNode = ph.node(node, "rtk_rover");
    if (rtkRoverNode.IsDefined() && !rtkRoverNode.IsNull())
        RTK_rover_ = new RtkRoverProvider(this->get_logger(), rtkRoverNode);

    YAML::Node rtkBaseNode = ph.node(node, "rtk_base");
    if (rtkBaseNode.IsDefined() && !rtkBaseNode.IsNull())
        RTK_base_ = new RtkBaseProvider(this->get_logger(), rtkBaseNode);

    // Print entire yaml node tree
    // printf("Node Tree:\n");
    // std::cout << node << "\n\n=====================  EXIT  =====================\n\n";

    // exit(1);
}

void InertialSenseROS::configure_data_streams()
{
    configure_data_streams(false);
}

#define CONFIG_STREAM(stream, did, type, cb_fun) \
    if((stream.enabled) && !(stream.streaming)){ \
        RCLCPP_DEBUG(this->get_logger(), "InertialSenseROS: Attempting to enable %s (%d) data stream", cISDataMappings::GetDataSetName(did), did); \
        SET_CALLBACK(did, type, cb_fun, stream.period); \
        if (!firstrun) \
            return; \
    }

#define CONFIG_STREAM_GPS(stream, did_pos, cb_fun_pos, did_vel, cb_fun_vel) \
    if((stream.enabled) && !(stream.streaming_pos)){ \
        RCLCPP_DEBUG(this->get_logger(), "InertialSenseROS: Attempting to enable %s (%d) data stream", cISDataMappings::GetDataSetName(did_pos), did_pos); \
        SET_CALLBACK(did_pos, gps_pos_t, cb_fun_pos, stream.period); \
        if (!firstrun) \
            return; \
    } \
    if((stream.enabled) && !(stream.streaming_vel)){ \
        RCLCPP_DEBUG(this->get_logger(), "InertialSenseROS: Attempting to enable %s (%d) data stream", cISDataMappings::GetDataSetName(did_vel), did_vel); \
        SET_CALLBACK(did_vel, gps_vel_t, cb_fun_vel, stream.period); \
        if (!firstrun) \
            return; \
    }

void InertialSenseROS::configure_data_streams(bool firstrun) // if firstrun is true each step will be attempted without returning
{
    if (!rs_.gps1.streaming_pos) // we always need GPS for Fix status
    {
        //RCLCPP_DEBUG("InertialSenseROS: Attempting to enable GPS1 Pos data stream");
        SET_CALLBACK(DID_GPS1_POS, gps_pos_t, GPS_pos_callback, rs_.gps1.period);
    }
    if (!flashConfigStreaming_)
    {
        RCLCPP_DEBUG(this->get_logger(), "InertialSenseROS: Attempting to enable flash config data stream");
        SET_CALLBACK(DID_FLASH_CONFIG, nvm_flash_cfg_t, flash_config_callback, 0);
        if (!firstrun)
            return;
    }

    if (rs_.odom_ins_ned.enabled && !(rs_.did_ins4.streaming && imuStreaming_))
    {
        RCLCPP_DEBUG(this->get_logger(), "InertialSenseROS: Attempting to enable odom INS NED data stream");

        SET_CALLBACK(DID_INS_4, ins_4_t, INS4_callback, rs_.did_ins4.period);                     // Need NED
        SET_CALLBACK(DID_PIMU, pimu_t, preint_IMU_callback, rs_.pimu.period);                     // Need angular rate data from IMU
        rs_.imu.enabled = true;
        odometryIdentity(msg_odom_ned);
        if (!firstrun)
            return;;
    }

    if (rs_.odom_ins_ecef.enabled && !(rs_.did_ins4.streaming && imuStreaming_))
    {
        RCLCPP_DEBUG(this->get_logger(), "InertialSenseROS: Attempting to enable odom INS ECEF data stream");
        SET_CALLBACK(DID_INS_4, ins_4_t, INS4_callback, rs_.did_ins4.period);                     // Need quaternion and ecef
        SET_CALLBACK(DID_PIMU, pimu_t, preint_IMU_callback, rs_.pimu.period);                     // Need angular rate data from IMU
        rs_.imu.enabled = true;
        odometryIdentity(msg_odom_ecef);
        if (!firstrun)
            return;
    }

    if (rs_.odom_ins_enu.enabled  && !(rs_.did_ins4.streaming && imuStreaming_))
    {
        RCLCPP_DEBUG(this->get_logger(), "InertialSenseROS: Attempting to enable odom INS ENU data stream");
        SET_CALLBACK(DID_INS_4, ins_4_t, INS4_callback, rs_.did_ins4.period);                     // Need ENU
        SET_CALLBACK(DID_PIMU, pimu_t, preint_IMU_callback, rs_.pimu.period);                     // Need angular rate data from IMU
        rs_.imu.enabled = true;
        odometryIdentity(msg_odom_enu);
        if (!firstrun)
            return;
    }

    if (covariance_enabled_ && !insCovarianceStreaming_)
    {
        RCLCPP_DEBUG(this->get_logger(), "InertialSenseROS: Attempting to enable %s data stream", cISDataMappings::GetDataSetName(DID_ROS_COVARIANCE_POSE_TWIST));
        SET_CALLBACK(DID_ROS_COVARIANCE_POSE_TWIST, ros_covariance_pose_twist_t, INS_covariance_callback, 200);
    }

    CONFIG_STREAM(rs_.did_ins1, DID_INS_1, ins_1_t, INS1_callback);
    CONFIG_STREAM(rs_.did_ins2, DID_INS_2, ins_2_t, INS2_callback);
    CONFIG_STREAM(rs_.did_ins4, DID_INS_4, ins_4_t, INS4_callback);
    CONFIG_STREAM(rs_.inl2_states, DID_INL2_STATES, inl2_states_t, INL2_states_callback);

    nvm_flash_cfg_t flashCfg;
    IS_.GetFlashConfig(flashCfg);
    if (!NavSatFixConfigured)
    {
        if (rs_.gps1_navsatfix.enabled) {
            RCLCPP_DEBUG(this->get_logger(), "InertialSenseROS: Attempting to enable gps1/NavSatFix");
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
            RCLCPP_DEBUG(this->get_logger(), "InertialSenseROS: Attempting to enable gps2/NavSatFix");
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

    CONFIG_STREAM(rs_.magnetometer, DID_MAGNETOMETER, magnetometer_t, mag_callback);
    CONFIG_STREAM(rs_.barometer, DID_BAROMETER, barometer_t, baro_callback);
    CONFIG_STREAM(rs_.pimu, DID_PIMU, pimu_t, preint_IMU_callback);

    if (!firstrun)
    {
        data_streams_enabled_ = true;
        data_stream_timer_->cancel();
        RCLCPP_INFO(this->get_logger(), "InertialSenseROS: All data streams successfully enabled");
        return;
    }
}

void InertialSenseROS::start_log()
{
    std::string filename = getenv("HOME");
    filename += "/Documents/Inertial_Sense/Logs/" + cISLogger::CreateCurrentTimestamp();
    RCLCPP_INFO_STREAM(this->get_logger(), "InertialSenseROS: Creating log in " << filename << " folder");
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

/**
 * Connects to the Inertial Sense hardware
 * Will attempt to connect using a list of multiple ports if specified,
 * @param timeout specifies the maximum duration (in seconds) before a returning false and reporting an ERROR
 * @return return true when a connection has been made, or false if not
 */
bool InertialSenseROS::connect(float timeout)
{
    uint32_t end_time = this->get_clock()->now().seconds() + timeout;
    auto ports_iterator = ports_.begin();

    do {
        std::string cur_port = *ports_iterator;
        /// Connect to the uINS
        RCLCPP_INFO(this->get_logger(), "InertialSenseROS: Connecting to serial port \"%s\", at %d baud", cur_port.c_str(), baudrate_);
        sdk_connected_ = IS_.Open(cur_port.c_str(), baudrate_);
        if (!sdk_connected_) {
            RCLCPP_ERROR(this->get_logger(), "InertialSenseROS: Unable to open serial port \"%s\", at %d baud", cur_port.c_str(), baudrate_);
            sleep(1); // is this a good idea?
        } else {
            RCLCPP_INFO(this->get_logger(), "InertialSenseROS: Connected to uINS %d on \"%s\", at %d baud", IS_.GetDeviceInfo().serialNumber, cur_port.c_str(), baudrate_);
            port_ = cur_port;
            break;
        }
        if ((ports_.size() > 1) && (ports_iterator != ports_.end()))
            ports_iterator++;
        else
            ports_iterator = ports_.begin(); // just keep looping until we timeout below
    } while (this->get_clock()->now().seconds() < end_time);

    return sdk_connected_;
}

bool InertialSenseROS::firmware_compatiblity_check()
{
    char local_protocol[4] = { PROTOCOL_VERSION_CHAR0, PROTOCOL_VERSION_CHAR1, PROTOCOL_VERSION_CHAR2, PROTOCOL_VERSION_CHAR3 };
    char diff_protocol[4] = { 0, 0, 0, 0 };
    for (size_t i = 0; i < sizeof(local_protocol); i++)  diff_protocol[i] = local_protocol[i] - IS_.GetDeviceInfo().protocolVer[i];

    char local_firmware[3] = { FIRMWARE_VERSION_CHAR0, FIRMWARE_VERSION_CHAR1, FIRMWARE_VERSION_CHAR2 };
    char diff_firmware[3] = { 0, 0 ,0 };
    for (size_t i = 0; i < sizeof(local_firmware); i++)  diff_firmware[i] = local_firmware[i] - IS_.GetDeviceInfo().firmwareVer[i];

    auto protocol_fault = RCUTILS_LOG_SEVERITY_DEBUG; // none
    if (diff_protocol[0] != 0) protocol_fault = RCUTILS_LOG_SEVERITY_FATAL; // major protocol changes -- BREAKING
    else if (diff_protocol[1] != 0) protocol_fault = RCUTILS_LOG_SEVERITY_ERROR; // minor protocol changes -- New parameters/features
    else if (diff_protocol[2] != 0) protocol_fault = RCUTILS_LOG_SEVERITY_WARN; // patch changes - the shouldn't be significant, but still important
    else if (diff_protocol[3] != 0) protocol_fault = RCUTILS_LOG_SEVERITY_INFO; // this is essentially trivial, but good to know.

    auto firmware_fault = RCUTILS_LOG_SEVERITY_DEBUG; // none
    if (diff_firmware[0] != 0) firmware_fault = RCUTILS_LOG_SEVERITY_FATAL;  // major protocol changes -- BREAKING
    else if (diff_firmware[1] != 0) firmware_fault = RCUTILS_LOG_SEVERITY_ERROR;  // minor protocol changes -- New parameters/features
    else if (diff_firmware[2] != 0) firmware_fault = RCUTILS_LOG_SEVERITY_WARN; // patch changes - the shouldn't be significant, but still important

    auto final_fault = std::max(firmware_fault, protocol_fault);
    if(final_fault != RCUTILS_LOG_SEVERITY_DEBUG){
        RCUTILS_LOG_COND_NAMED(final_fault, RCUTILS_LOG_CONDITION_EMPTY, RCUTILS_LOG_CONDITION_EMPTY, this->get_logger().get_name(),
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
    }
    return final_fault == RCUTILS_LOG_SEVERITY_DEBUG; // true if they match, false if they don't.
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
    IS_.GetFlashConfig(current_flash_cfg);
    //RCLCPP_INFO("InertialSenseROS: Configuring flash: \nCurrent: %i, \nDesired: %i\n", current_flash_cfg.ioConfig, ioConfig_);

    if (current_flash_cfg.startupNavDtMs != ins_nav_dt_ms_)
    {
        RCLCPP_INFO(this->get_logger(), "InertialSenseROS: Navigation rate change from %dms to %dms, resetting uINS to make change", current_flash_cfg.startupNavDtMs, ins_nav_dt_ms_);
        reboot = true;
    }
    if (current_flash_cfg.ioConfig != ioConfigBits_)
    {
        RCLCPP_INFO(this->get_logger(), "InertialSenseROS: ioConfig change from 0x%08X to 0x%08X, resetting uINS to make change", current_flash_cfg.ioConfig, ioConfigBits_);
        reboot = true;
    }

    bool setRefLla = false;
    for (int i=0; i<3; i++)
    {
        if (refLla_[0] != 0.0)
        {
            setRefLla = true;
        }
    }
    if (!vecF32Match(current_flash_cfg.insRotation, insRotation_) ||
        !vecF32Match(current_flash_cfg.insOffset, insOffset_) ||
        !vecF32Match(current_flash_cfg.gps1AntOffset, rs_.gps1.antennaOffset) ||
        !vecF32Match(current_flash_cfg.gps2AntOffset, rs_.gps2.antennaOffset) ||
        (setRefLla && !vecF64Match(current_flash_cfg.refLla, refLla_)) ||
        current_flash_cfg.startupNavDtMs != ins_nav_dt_ms_ ||
        current_flash_cfg.ioConfig != ioConfigBits_ ||
        current_flash_cfg.gpsTimeUserDelay != gpsTimeUserDelay_ ||
        // current_flash_cfg.magDeclination != magDeclination_ ||
        current_flash_cfg.insDynModel != insDynModel_ ||
        current_flash_cfg.platformConfig != platformConfig_
        )
    {
        for (int i=0; i<3; i++)
        {
            current_flash_cfg.insRotation[i] = insRotation_[i];
            current_flash_cfg.insOffset[i] = insOffset_[i];
            current_flash_cfg.gps1AntOffset[i] = rs_.gps1.antennaOffset[i];
            current_flash_cfg.gps2AntOffset[i] = rs_.gps2.antennaOffset[i];
            if (setRefLla)
            {
                current_flash_cfg.refLla[i] = refLla_[i];
            }
        }
        current_flash_cfg.startupNavDtMs = ins_nav_dt_ms_;
        current_flash_cfg.ioConfig = ioConfigBits_;
        current_flash_cfg.gpsTimeUserDelay = gpsTimeUserDelay_;
        current_flash_cfg.magDeclination = magDeclination_;
        current_flash_cfg.insDynModel = insDynModel_;
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
            RCLCPP_INFO_STREAM(this->get_logger(), "InertialSenseROS: Successfully connected to RTK server [" << RTK_connection  << "]. [Attempt " << RTK_connection_attempt_count << "]");
            break;
        }
        // fall-through

        // RCLCPP_ERROR_STREAM("Failed to connect to base server at " << RTK_connection);
        if (RTK_connection_attempt_count < config.connection_attempt_limit_) {
            RCLCPP_WARN_STREAM(this->get_logger(), "InertialSenseROS: Unable to establish connection with RTK server [" << RTK_connection << "] after attempt " << RTK_connection_attempt_count << ". Will try again in " << sleep_duration << " seconds.");
        } else {
            RCLCPP_ERROR_STREAM(this->get_logger(), "InertialSenseROS: Unable to establish connection with RTK server [" << RTK_connection << "] after attempt " << RTK_connection_attempt_count << ". Giving up.");
        }
        rclcpp::sleep_for(std::chrono::seconds(sleep_duration)); // we will always sleep on a failure...
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

    int latest_byte_count = IS_.GetClientServerByteCount();
    if (config.traffic_total_byte_count_ == latest_byte_count)
    {
        ++config.data_transmission_interruption_count_;

        if (config.data_transmission_interruption_count_ >= config.data_transmission_interruption_limit_)
        {
            if (config.traffic_time > 0.0)
                RCLCPP_WARN_STREAM(this->get_logger(), "Last received RTK correction data was " << (this->get_clock()->now().seconds() - config.traffic_time) << " seconds ago. Attempting to re-establish connection.");
            connect_rtk_client(config);
            if (config.connected_) {
                config.traffic_total_byte_count_ = latest_byte_count;
                config.data_transmission_interruption_count_ = 0;
            }
        } else {
            if (config.traffic_time > 0.0)
                RCLCPP_WARN_STREAM(this->get_logger(), "Last received RTK correction data was " << (this->get_clock()->now().seconds() - config.traffic_time) << " seconds ago.");
        }
    }
    else
    {
        config.traffic_time = this->get_clock()->now().seconds();
        config.traffic_total_byte_count_ = latest_byte_count;
        config.data_transmission_interruption_count_ = 0;
    }
}

void InertialSenseROS::start_rtk_connectivity_watchdog_timer()
{
    if ((RTK_rover_ == nullptr) || (RTK_rover_->correction_input == nullptr) || (RTK_rover_->correction_input->type_ != "ntrip"))
        return;

    RtkRoverCorrectionProvider_Ntrip& config = *(RtkRoverCorrectionProvider_Ntrip*)(RTK_rover_->correction_input);
    if (!config.connectivity_watchdog_enabled_) {
        return;
    }

    if (rtk_connectivity_watchdog_timer_) {
        rtk_connectivity_watchdog_timer_ = this->create_wall_timer(rclcpp::Rate(config.connectivity_watchdog_timer_frequency_).period(), std::bind(&InertialSenseROS::rtk_connectivity_watchdog_timer_callback, this));
    }

    rtk_connectivity_watchdog_timer_.reset();
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
        RCLCPP_INFO_STREAM(this->get_logger(), "InertialSenseROS: Successfully started RTK Base NTRIP correction server at" << RTK_connection);
    else
        RCLCPP_ERROR_STREAM(this->get_logger(), "InertialSenseROS: Failed to start RTK Base NTRIP correction server at " << RTK_connection);
}


void InertialSenseROS::configure_rtk()
{
    rtkConfigBits_ = 0;
    if (rs_.gps1.type == "F9P")
    {
        if (RTK_rover_)
        {
            if (RTK_rover_->correction_input && (RTK_rover_->correction_input->type_ == "ntrip")) {
                RCLCPP_INFO(this->get_logger(), "InertialSenseROS: Configuring RTK Rover");
                rs_.rtk_pos.enabled = true;
                connect_rtk_client(*(RtkRoverCorrectionProvider_Ntrip *) RTK_rover_->correction_input);
                rtkConfigBits_ |= RTK_CFG_BITS_ROVER_MODE_RTK_POSITIONING_EXTERNAL;
                SET_CALLBACK(DID_GPS1_RTK_POS_MISC, gps_rtk_misc_t, RTK_Misc_callback, rs_.rtk_pos.period);
                SET_CALLBACK(DID_GPS1_RTK_POS_REL, gps_rtk_rel_t, RTK_Rel_callback, rs_.rtk_pos.period);

                start_rtk_connectivity_watchdog_timer();
            }
            if (RTK_rover_->correction_input && (RTK_rover_->correction_input->type_ == "evb")) {
                RCLCPP_INFO(this->get_logger(), "InertialSenseROS: Configuring RTK Rover with radio enabled");
                rs_.rtk_pos.enabled = true;
                if (RTK_base_) RTK_base_->enable = false;
                rtkConfigBits_ |= RTK_CFG_BITS_ROVER_MODE_RTK_POSITIONING_EXTERNAL;
                SET_CALLBACK(DID_GPS1_RTK_POS_MISC, gps_rtk_misc_t, RTK_Misc_callback, rs_.rtk_pos.period);
                SET_CALLBACK(DID_GPS1_RTK_POS_REL, gps_rtk_rel_t, RTK_Rel_callback, rs_.rtk_pos.period);
            }
        }
        if (GNSS_Compass_)
        {
            RCLCPP_INFO(this->get_logger(), "InertialSenseROS: Configuring Dual GNSS (compassing)");
            rs_.rtk_cmp.enabled = true;
            rtkConfigBits_ |= RTK_CFG_BITS_ROVER_MODE_RTK_COMPASSING_F9P;
            SET_CALLBACK(DID_GPS2_RTK_CMP_MISC, gps_rtk_misc_t, RTK_Misc_callback, rs_.rtk_cmp.period);
            SET_CALLBACK(DID_GPS2_RTK_CMP_REL, gps_rtk_rel_t, RTK_Rel_callback, rs_.rtk_cmp.period);
        }

        if (RTK_base_ && RTK_base_->enable) {
            RCLCPP_INFO(this->get_logger(), "InertialSenseROS: Configuring RTK Base");
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

        RCLCPP_ERROR_EXPRESSION(this->get_logger(), RTK_rover_ && RTK_rover_->enable && RTK_base_ && RTK_base_->enable, "unable to configure onboard receiver to be both RTK rover and base - default to rover");
        RCLCPP_ERROR_EXPRESSION(this->get_logger(), RTK_rover_ && RTK_rover_->enable && GNSS_Compass_, "unable to configure onboard receiver to be both RTK rover as dual GNSS - default to dual GNSS");

        if (GNSS_Compass_)
        {
            RCLCPP_INFO(this->get_logger(), "InertialSenseROS: Configuring Dual GNSS (compassing)");
            RTK_rover_->enable = false; // FIXME:  Is this right?  Rover is disabled when in Compassing?
            rtkConfigBits_ |= RTK_CFG_BITS_ROVER_MODE_RTK_COMPASSING;
            SET_CALLBACK(DID_GPS2_RTK_CMP_MISC, gps_rtk_misc_t, RTK_Misc_callback, rs_.rtk_cmp.period);
            SET_CALLBACK(DID_GPS2_RTK_CMP_REL, gps_rtk_rel_t, RTK_Rel_callback, rs_.rtk_cmp.period);
        }

        if (RTK_rover_ && RTK_rover_->enable && RTK_rover_->correction_input && RTK_rover_->correction_input->type_ == "evb")
        {
            RCLCPP_INFO(this->get_logger(), "InertialSenseROS: Configuring RTK Rover with radio enabled");
            if (RTK_base_) RTK_base_->enable = false;
            rtkConfigBits_ |= (rs_.gps1.type == "F9P" ? RTK_CFG_BITS_ROVER_MODE_RTK_POSITIONING_EXTERNAL : RTK_CFG_BITS_ROVER_MODE_RTK_POSITIONING);
            SET_CALLBACK(DID_GPS1_RTK_POS_MISC, gps_rtk_misc_t, RTK_Misc_callback, rs_.rtk_pos.period);
            SET_CALLBACK(DID_GPS1_RTK_POS_REL, gps_rtk_rel_t, RTK_Rel_callback, rs_.rtk_pos.period);
        }
        else if (RTK_rover_ && RTK_rover_->enable && RTK_rover_->correction_input && RTK_rover_->correction_input->type_ == "ntrip")
        {
            RCLCPP_INFO(this->get_logger(), "InertialSenseROS: Configuring as RTK Rover");
            if (RTK_base_) RTK_base_->enable = false;
            rtkConfigBits_ |= (rs_.gps1.type == "F9P" ? RTK_CFG_BITS_ROVER_MODE_RTK_POSITIONING_EXTERNAL : RTK_CFG_BITS_ROVER_MODE_RTK_POSITIONING);
            connect_rtk_client((RtkRoverCorrectionProvider_Ntrip&)*RTK_rover_->correction_input);
            SET_CALLBACK(DID_GPS1_RTK_POS_MISC, gps_rtk_misc_t, RTK_Misc_callback, rs_.rtk_pos.period);
            SET_CALLBACK(DID_GPS1_RTK_POS_REL, gps_rtk_rel_t, RTK_Rel_callback, rs_.rtk_pos.period);

            start_rtk_connectivity_watchdog_timer();
        }
        else if (RTK_base_ && RTK_base_->enable)
        {
            RCLCPP_INFO(this->get_logger(), "InertialSenseROS: Configured as RTK Base");
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
    RCLCPP_INFO(this->get_logger(), "InertialSenseROS: Setting rtkConfigBits: 0x%08x", rtkConfigBits_);
}

void InertialSenseROS::flash_config_callback(eDataIDs DID, const nvm_flash_cfg_t *const msg)
{
    STREAMING_CHECK(flashConfigStreaming_, DID);

    refLla_[0] = msg->refLla[0];
    refLla_[1] = msg->refLla[1];
    refLla_[2] = msg->refLla[2];
    refLLA_known = true;
    RCLCPP_DEBUG(this->get_logger(), "InertialSenseROS: refLla was set");
}

void InertialSenseROS::INS1_callback(eDataIDs DID, const ins_1_t *const msg)
{
    rs_.did_ins1.streamingCheck(this->get_logger(), DID);

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
        if (rs_.did_ins1.pub->get_subscription_count() > 0)
            rs_.did_ins1.pub->publish(msg_did_ins1);
    }
}

void InertialSenseROS::INS2_callback(eDataIDs DID, const ins_2_t *const msg)
{
    rs_.did_ins2.streamingCheck(this->get_logger(), DID);

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
        if (rs_.did_ins2.pub->get_subscription_count() > 0)
            rs_.did_ins2.pub->publish(msg_did_ins2);
    }
}

void InertialSenseROS::INS4_callback(eDataIDs DID, const ins_4_t *const msg)
{
    rs_.did_ins4.streamingCheck(this->get_logger(), DID);

    if (!refLLA_known)
    {
        RCLCPP_INFO(this->get_logger(), "InertialSenseROS: Waiting for refLLA to be received from uINS/IMX");
        return;
    }
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
        if (rs_.did_ins4.pub->get_subscription_count() > 0)
            rs_.did_ins4.pub->publish(msg_did_ins4);
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

            rs_.odom_ins_ecef.pub->publish(msg_odom_ecef);

            if (publishTf_)
            {
                broadcat_tf2(transform_ECEF, msg_odom_ecef, "ins_base_link_ecef", "ins_ecef");
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
                //ecef to lla (rad,rad,m)
            ecef2lla(msg->ecef, llaPosRadians);
            ixVector3 ned;
            ixVector3d refLlaRadians;
                //convert refLla_ to radians
            lla_Deg2Rad_d(refLlaRadians, refLla_);
                //lla to ned
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
            ixVector3 result;

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
            rs_.odom_ins_ned.pub->publish(msg_odom_ned);

            if (publishTf_)
            {
                broadcat_tf2(transform_NED, msg_odom_ned, "ins_base_link_ned", "ins_ned");
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
                //Calculate in NED then convert
            ixVector3d llaPosRadians;
                //ecef to lla (rad,rad,m)
            ecef2lla(msg->ecef, llaPosRadians);
            ixVector3 ned;
            ixVector3d refLlaRadians;
                //convert refLla_ to radians
            lla_Deg2Rad_d(refLlaRadians, refLla_);
                //lla to ned
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
            ixVector3 result;
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

            rs_.odom_ins_enu.pub->publish(msg_odom_enu);
            if (publishTf_)
            {
                broadcat_tf2(transform_ENU, msg_odom_enu, "ins_base_link_enu", "ins_enu");
            }
        }
    }
}

void InertialSenseROS::INL2_states_callback(eDataIDs DID, const inl2_states_t *const msg)
{
    rs_.inl2_states.streamingCheck(this->get_logger(), DID);

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
        rs_.inl2_states.pub->publish(msg_inl2_states);
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
        rs_.gps1.streamingCheck(this->get_logger(), DID, rs_.gps1.streaming_pos);
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
            msg_gps1.h_msl = msg->hMSL;
            msg_gps1.h_acc = msg->hAcc;
            msg_gps1.v_acc = msg->vAcc;
            msg_gps1.p_dop = msg->pDop;
            publishGPS1();
        }
        break;

    case DID_GPS2_POS:
        rs_.gps2.streamingCheck(this->get_logger(), DID, rs_.gps2.streaming_pos);
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
            msg_gps2.h_msl = msg->hMSL;
            msg_gps2.h_acc = msg->hAcc;
            msg_gps2.v_acc = msg->vAcc;
            msg_gps2.p_dop = msg->pDop;
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
            if ((msg->status & GPS_STATUS_FIX_MASK) >= GPS_STATUS_FIX_2D) // Check for fix and set
            {
                msg_NavSatFix.status.status = NavSatFixStatusFixType::STATUS_FIX;
            }

            if (msg->status & GPS_STATUS_FIX_SBAS) // Check for SBAS only fix
            {
                msg_NavSatFix.status.status = NavSatFixStatusFixType::STATUS_SBAS_FIX;
            }

            if ((msg->status & GPS_STATUS_FIX_MASK) >= GPS_STATUS_FIX_RTK_SINGLE) // Check for any RTK fix
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
            rs_.gps1_navsatfix.pub->publish(msg_NavSatFix);
        }
    }
}

void InertialSenseROS::GPS_vel_callback(eDataIDs DID, const gps_vel_t *const msg)
{
    switch (DID)
    {
    case DID_GPS1_VEL:
        rs_.gps1.streamingCheck(this->get_logger(), DID, rs_.gps1.streaming_vel);
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
        rs_.gps2.streamingCheck(this->get_logger(), DID, rs_.gps2.streaming_vel);
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
    double dt = (gps1_velEcef.header.stamp.nanosec - msg_gps1.header.stamp.nanosec) / 10.0e9;
    if (abs(dt) < 2.0e-3)
    {
        msg_gps1.vel_ecef = gps1_velEcef.vector;
        msg_gps1.s_acc = gps1_vel.sAcc;
        if (rs_.gps1.pub->get_subscription_count() > 0)
            rs_.gps1.pub->publish(msg_gps1);
    }
}

void InertialSenseROS::publishGPS2()
{
    double dt = (gps2_velEcef.header.stamp.nanosec - msg_gps2.header.stamp.nanosec)/10.0e9;
    if (abs(dt) < 2.0e-3)
    {
        msg_gps2.vel_ecef = gps2_velEcef.vector;
        msg_gps2.s_acc = gps2_vel.sAcc;
        if (rs_.gps2.pub->get_subscription_count() > 0)
            rs_.gps2.pub->publish(msg_gps2);
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
        STREAMING_CHECK(strobeInStreaming_, DID);

        if (abs(GPS_towOffset_) > 0.001)
        {
            std_msgs::msg::Header strobe_msg;
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
    case DID_GPS1_SAT:  rs_.gps1_info.streamingCheck(this->get_logger(), DID);   break;
    case DID_GPS2_SAT:  rs_.gps2_info.streamingCheck(this->get_logger(), DID);   break;
    default: return;
    }

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
    case DID_GPS1_SAT:  rs_.gps1_info.pub->publish(msg_gps1_info);    break;
    case DID_GPS2_SAT:  rs_.gps2_info.pub->publish(msg_gps1_info);    break;
    }
}

void InertialSenseROS::mag_callback(eDataIDs DID, const magnetometer_t *const msg)
{
    if (DID != DID_MAGNETOMETER)
    {
        return;
    }

    rs_.magnetometer.streamingCheck(this->get_logger(), DID);
    sensor_msgs::msg::MagneticField mag_msg;
    mag_msg.header.stamp = ros_time_from_start_time(msg->time);
    mag_msg.header.frame_id = frame_id_;
    mag_msg.magnetic_field.x = msg->mag[0];
    mag_msg.magnetic_field.y = msg->mag[1];
    mag_msg.magnetic_field.z = msg->mag[2];
    rs_.magnetometer.pub->publish(mag_msg);
}

void InertialSenseROS::baro_callback(eDataIDs DID, const barometer_t *const msg)
{
    if (DID != DID_BAROMETER)
    {
        return;
    }

    rs_.barometer.streamingCheck(this->get_logger(), DID);
    sensor_msgs::msg::FluidPressure baro_msg;
    baro_msg.header.stamp = ros_time_from_start_time(msg->time);
    baro_msg.header.frame_id = frame_id_;
    baro_msg.fluid_pressure = msg->bar;
    baro_msg.variance = msg->barTemp;
    rs_.barometer.pub->publish(baro_msg);
}

void InertialSenseROS::preint_IMU_callback(eDataIDs DID, const pimu_t *const msg)
{
    imuStreaming_ = true;

    if (rs_.pimu.enabled)
    {
        rs_.pimu.streamingCheck(this->get_logger(), DID);
        msg_pimu.header.stamp = ros_time_from_start_time(msg->time);
        msg_pimu.header.frame_id = frame_id_;
        msg_pimu.dtheta.x = msg->theta[0];
        msg_pimu.dtheta.y = msg->theta[1];
        msg_pimu.dtheta.z = msg->theta[2];
        msg_pimu.dvel.x = msg->vel[0];
        msg_pimu.dvel.y = msg->vel[1];
        msg_pimu.dvel.z = msg->vel[2];
        msg_pimu.dt = msg->dt;
        rs_.pimu.pub->publish(msg_pimu);
    }

    if (rs_.imu.enabled)
    {
        rs_.imu.streamingCheck(this->get_logger(), DID);
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
            rs_.imu.pub->publish(msg_imu);
        }
    }
}

void InertialSenseROS::RTK_Misc_callback(eDataIDs DID, const gps_rtk_misc_t *const msg)
{
    inertial_sense_ros::msg::RTKInfo rtk_info;
    if (abs(GPS_towOffset_) > 0.001)
    {
        rtk_info.header.stamp = ros_time_from_week_and_tow(GPS_week_, msg->timeOfWeekMs / 1000.0);
        rtk_info.base_antcount = msg->baseAntennaCount;
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
        rs_.rtk_pos.streamingCheck(this->get_logger(), DID);
        rs_.rtk_pos.pubInfo->publish(rtk_info);
        break;

    case DID_GPS2_RTK_CMP_MISC:
        rs_.rtk_cmp.streamingCheck(this->get_logger(), DID);
        rs_.rtk_cmp.pubInfo->publish(rtk_info);
        break;
    }
}

void InertialSenseROS::RTK_Rel_callback(eDataIDs DID, const gps_rtk_rel_t *const msg)
{
    inertial_sense_ros::msg::RTKRel rtk_rel;
    if (abs(GPS_towOffset_) > 0.001)
    {
        rtk_rel.header.stamp = ros_time_from_week_and_tow(GPS_week_, msg->timeOfWeekMs / 1000.0);
        rtk_rel.differential_age = msg->differentialAge;
        rtk_rel.ar_ratio = msg->arRatio;
        uint32_t fixStatus = msg->status & GPS_STATUS_FIX_MASK;
        if (fixStatus == GPS_STATUS_FIX_3D)
        {
            rtk_rel.e_gps_status_fix = inertial_sense_ros::msg::RTKRel::GPS_STATUS_FIX_3D;
        }
        else if (fixStatus == GPS_STATUS_FIX_RTK_SINGLE)
        {
            rtk_rel.e_gps_status_fix = inertial_sense_ros::msg::RTKRel::GPS_STATUS_FIX_RTK_SINGLE;
        }
        else if (fixStatus == GPS_STATUS_FIX_RTK_FLOAT)
        {
            rtk_rel.e_gps_status_fix = inertial_sense_ros::msg::RTKRel::GPS_STATUS_FIX_RTK_FLOAT;
        }
        else if (fixStatus == GPS_STATUS_FIX_RTK_FIX)
        {
            rtk_rel.e_gps_status_fix = inertial_sense_ros::msg::RTKRel::GPS_STATUS_FIX_RTK_FIX;
        }
        else if (msg->status & GPS_STATUS_FLAGS_RTK_FIX_AND_HOLD)
        {
            rtk_rel.e_gps_status_fix = inertial_sense_ros::msg::RTKRel::GPS_STATUS_FLAGS_RTK_FIX_AND_HOLD;
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
        rs_.rtk_pos.streamingCheck(this->get_logger(), DID, rs_.rtk_pos.streamingRel);
        rs_.rtk_pos.pubRel->publish(rtk_rel);
        break;

    case DID_GPS2_RTK_CMP_REL:
        rs_.rtk_cmp.streamingCheck(this->get_logger(), DID, rs_.rtk_cmp.streamingRel);
        rs_.rtk_cmp.pubRel->publish(rtk_rel);
        break;
    }

    // save for diagnostics TODO - Add more diagnostic info
    diagnostic_ar_ratio_ = rtk_rel.ar_ratio;
    diagnostic_differential_age_ = rtk_rel.differential_age;
    diagnostic_heading_base_to_rover_ = rtk_rel.heading_base_to_rover;
    diagnostic_fix_type_ = rtk_rel.e_gps_status_fix;
}

void InertialSenseROS::GPS_raw_callback(eDataIDs DID, const gps_raw_t *const msg)
{
    switch (DID)
    {
    case DID_GPS1_RAW:        rs_.gps1_raw.streamingCheck(this->get_logger(), DID);     break;
    case DID_GPS2_RAW:        rs_.gps2_raw.streamingCheck(this->get_logger(), DID);     break;
    case DID_GPS_BASE_RAW:    rs_.gpsbase_raw.streamingCheck(this->get_logger(), DID);  break;
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
            GPS_obs_bundle_timer_callback();
        }
        break;

    case DID_GPS2_RAW:
        if (gps2_obs_Vec_.obs.size() > 0 &&
            (msg[0].time.time != gps2_obs_Vec_.obs[0].time.time ||
            msg[0].time.sec != gps2_obs_Vec_.obs[0].time.sec))
        {
            GPS_obs_bundle_timer_callback();
        }
        break;

    case DID_GPS_BASE_RAW:
       if (base_obs_Vec_.obs.size() > 0 &&
            (msg[0].time.time != base_obs_Vec_.obs[0].time.time ||
            msg[0].time.sec != base_obs_Vec_.obs[0].time.sec))
        {
            GPS_obs_bundle_timer_callback();
        }
        break;
    }

    for (int i = 0; i < nObs; i++)
    {
        inertial_sense_ros::msg::GNSSObservation obs;
        obs.header.stamp = ros_time_from_gtime(msg[i].time.time, msg[i].time.sec);
        obs.time.time = msg[i].time.time;
        obs.time.sec = msg[i].time.sec;
        obs.sat = msg[i].sat;
        obs.rcv = msg[i].rcv;
        obs.snr = msg[i].SNR[0];
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
            last_obs_time_1_ = this->get_clock()->now();
            break;
        case DID_GPS2_RAW:
            gps2_obs_Vec_.obs.push_back(obs);
            last_obs_time_2_ = this->get_clock()->now();
            break;
        case DID_GPS_BASE_RAW:
            base_obs_Vec_.obs.push_back(obs);
            last_obs_time_base_ = this->get_clock()->now();
            break;
        }
    }
}

void InertialSenseROS::GPS_obs_bundle_timer_callback()
{
    if (gps1_obs_Vec_.obs.size() != 0)
    {
        if (abs(this->get_clock()->now().seconds() - last_obs_time_1_.seconds()) > 1e-2)
        {
            gps1_obs_Vec_.header.stamp = ros_time_from_gtime(gps1_obs_Vec_.obs[0].time.time, gps1_obs_Vec_.obs[0].time.sec);
            gps1_obs_Vec_.time = gps1_obs_Vec_.obs[0].time;
            rs_.gps1_raw.pubObs->publish(gps1_obs_Vec_);
            gps1_obs_Vec_.obs.clear();
        }
    }
    if (gps2_obs_Vec_.obs.size() != 0)
    {
        if (abs(this->get_clock()->now().seconds() - last_obs_time_2_.seconds()) > 1e-2)
        {
            gps2_obs_Vec_.header.stamp = ros_time_from_gtime(gps2_obs_Vec_.obs[0].time.time, gps2_obs_Vec_.obs[0].time.sec);
            gps2_obs_Vec_.time = gps2_obs_Vec_.obs[0].time;
            rs_.gps2_raw.pubObs->publish(gps2_obs_Vec_);
            gps2_obs_Vec_.obs.clear();
        }
    }
    if (base_obs_Vec_.obs.size() != 0)
    {
        if (abs(this->get_clock()->now().seconds() - last_obs_time_base_.seconds()) > 1e-2)
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
    inertial_sense_ros::msg::GNSSEphemeris eph;
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
    eph.omg0 = msg->OMG0;
    eph.omg = msg->omg;
    eph.m0 = msg->M0;
    eph.deln = msg->deln;
    eph.omgd = msg->OMGd;
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
    eph.adot = msg->Adot;
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
    inertial_sense_ros::msg::GlonassEphemeris geph;
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

void InertialSenseROS::diagnostics_callback()
{
    if (!diagnosticsStreaming_)
    {
        diagnosticsStreaming_ = true;
        RCLCPP_INFO(this->get_logger(), "InertialSenseROS: Diagnostics response received");
    }
    // Create diagnostic objects
    diagnostic_msgs::msg::DiagnosticArray diag_array;
    diag_array.header.stamp = this->get_clock()->now();

    // CNO mean
    diagnostic_msgs::msg::DiagnosticStatus cno_mean;
    cno_mean.name = "CNO Mean";
    cno_mean.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    cno_mean.message = std::to_string(msg_gps1.cno);
    diag_array.status.push_back(cno_mean);

    if (rs_.rtk_pos.enabled)
    {
        diagnostic_msgs::msg::DiagnosticStatus rtk_status;
        rtk_status.name = "RTK";
        rtk_status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
        std::string rtk_message;

        // AR ratio
        diagnostic_msgs::msg::KeyValue ar_ratio;
        ar_ratio.key = "AR Ratio";
        ar_ratio.value = std::to_string(diagnostic_ar_ratio_);
        rtk_status.values.push_back(ar_ratio);
        if (diagnostic_fix_type_ == inertial_sense_ros::msg::RTKRel::GPS_STATUS_FIX_3D)
        {
            rtk_status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
            rtk_message = "3D: " + std::to_string(diagnostic_ar_ratio_);
        }
        else if (diagnostic_fix_type_ == inertial_sense_ros::msg::RTKRel::GPS_STATUS_FIX_RTK_SINGLE)
        {
            rtk_status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
            rtk_message = "Single: " + std::to_string(diagnostic_ar_ratio_);
        }
        else if (diagnostic_fix_type_ == inertial_sense_ros::msg::RTKRel::GPS_STATUS_FIX_RTK_FLOAT)
        {
            rtk_message = "Float: " + std::to_string(diagnostic_ar_ratio_);
        }
        else if (diagnostic_fix_type_ == inertial_sense_ros::msg::RTKRel::GPS_STATUS_FIX_RTK_FIX)
        {
            rtk_message = "Fix: " + std::to_string(diagnostic_ar_ratio_);
        }
        else if (diagnostic_fix_type_ == inertial_sense_ros::msg::RTKRel::GPS_STATUS_FLAGS_RTK_FIX_AND_HOLD)
        {
            rtk_message = "Fix and Hold: " + std::to_string(diagnostic_ar_ratio_);
        }
        else
        {
            rtk_message = "Unknown Fix: " + std::to_string(diagnostic_ar_ratio_);
        }

        // Differential age
        diagnostic_msgs::msg::KeyValue differential_age;
        differential_age.key = "Differential Age";
        differential_age.value = std::to_string(diagnostic_differential_age_);
        rtk_status.values.push_back(differential_age);
        if (diagnostic_differential_age_ > 1.5)
        {
            rtk_status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
            rtk_message += " Differential Age Large";
        }

        // Heading base to rover
        diagnostic_msgs::msg::KeyValue heading_base_to_rover;
        heading_base_to_rover.key = "Heading Base to Rover (rad)";
        heading_base_to_rover.value = std::to_string(diagnostic_heading_base_to_rover_);
        rtk_status.values.push_back(heading_base_to_rover);

        rtk_status.message = rtk_message;
        diag_array.status.push_back(rtk_status);
    }

    rs_.diagnostics.pub->publish(diag_array);
}

void InertialSenseROS::set_current_position_as_refLLA(const std::shared_ptr<std_srvs::srv::Trigger::Request> req, std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
    (void)req;
    double current_lla_[3];
    current_lla_[0] = lla_[0];
    current_lla_[1] = lla_[1];
    current_lla_[2] = lla_[2];

    IS_.SendData(DID_FLASH_CONFIG, reinterpret_cast<uint8_t *>(&current_lla_), sizeof(current_lla_), offsetof(nvm_flash_cfg_t, refLla));

    comManagerGetData(0, DID_FLASH_CONFIG, 0, 0, 1);

    int i = 0;
    nvm_flash_cfg_t current_flash;
    IS_.GetFlashConfig(current_flash);
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
        res->success = true;
        res->message = ("Update was succesful.  refLla: Lat: " + std::to_string(current_lla_[0]) + "  Lon: " + std::to_string(current_lla_[1]) + "  Alt: " + std::to_string(current_lla_[2]));
    }
    else
    {
        comManagerGetData(0, DID_FLASH_CONFIG, 0, 0, 0);
        res->success = false;
        res->message = "Unable to update refLLA. Please try again.";
    }

}

void InertialSenseROS::set_refLLA_to_value(const std::shared_ptr<inertial_sense_ros::srv::RefLLAUpdate::Request> req, std::shared_ptr<inertial_sense_ros::srv::RefLLAUpdate::Response> res)
{
    IS_.SendData(DID_FLASH_CONFIG, reinterpret_cast<uint8_t *>(&req->lla), sizeof(req->lla), offsetof(nvm_flash_cfg_t, refLla));

    comManagerGetData(0, DID_FLASH_CONFIG, 0, 0, 1);

    int i = 0;
    nvm_flash_cfg_t current_flash;
    IS_.GetFlashConfig(current_flash);
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
}

void InertialSenseROS::perform_mag_cal_srv_callback(const std::shared_ptr<std_srvs::srv::Trigger::Request> req, std::shared_ptr<std_srvs::srv::Trigger::Response>  res)
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
                res->success = true;
                res->message = "Successfully initiated mag recalibration.";
                return;
            }
        }
    }
}

void InertialSenseROS::perform_multi_mag_cal_srv_callback(const std::shared_ptr<std_srvs::srv::Trigger::Request> req, std::shared_ptr<std_srvs::srv::Trigger::Response> res)
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
                res->success = true;
                res->message = "Successfully initiated mag recalibration.";
                return;
            }
        }
    }
}

void InertialSenseROS::reset_device()
{
    // send reset command
    system_command_t reset_command;
    reset_command.command = 99;
    reset_command.invCommand = ~reset_command.command;
    IS_.SendData(DID_SYS_CMD, reinterpret_cast<uint8_t *>(&reset_command), sizeof(system_command_t), 0);

    RCLCPP_WARN(this->get_logger(), "Device reset required.\n\nDisconnecting from device.\n");
    sleep(2);
    IS_.Close();
}

void InertialSenseROS::update_firmware_srv_callback(const std::shared_ptr<inertial_sense_ros::srv::FirmwareUpdate::Request> req, std::shared_ptr<inertial_sense_ros::srv::FirmwareUpdate::Response> res)
{
    (void)res;(void)req;
    //   IS_.Close();
    //   vector<InertialSense::bootload_result_t> results = IS_.BootloadFile("*", req->filename, 921600);
    //   if (!results[0].error.empty())
    //   {
    //     res->success = false;
    //     res->message = results[0].error;
    //     return false;
    //   }
    //   IS_.Open(port_.c_str(), baudrate_);
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
        // Otherwise, estimate the uINS boot time and offset the messages
        if (!got_first_message_)
        {
            got_first_message_ = true;
            INS_local_offset_ = this->get_clock()->now().seconds() - timeOfWeek;
        }
        else // low-pass filter offset to account for drift
        {
            double y_offset = this->get_clock()->now().seconds() - timeOfWeek;
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
        // Otherwise, estimate the uINS boot time and offset the messages
        if (!got_first_message_)
        {
            got_first_message_ = true;
            INS_local_offset_ = this->get_clock()->now().seconds() - time;
        }
        else // low-pass filter offset to account for drift
        {
            double y_offset = this->get_clock()->now().seconds() - time;
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
    return rclcpp::Time(sec - LEAP_SECONDS, subsec * 1e9);
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

void InertialSenseROS::broadcat_tf2(tf2::Transform &tf, const nav_msgs::msg::Odometry &msg, const std::string& parent, const std::string& child)
{
    auto &pose = msg.pose.pose;
    tf.setOrigin(tf2::Vector3(pose.position.x, pose.position.y, pose.position.z));
    tf2::Quaternion q;
    tf2::fromMsg(pose.orientation, q);
    tf.setRotation(q);

    geometry_msgs::msg::TransformStamped t;
    tf2::toMsg(tf, t.transform);
    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = parent;
    t.child_frame_id = child;
    br->sendTransform(t);
};
