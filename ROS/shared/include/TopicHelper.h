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

#ifndef INERTIAL_SENSE_IMX_TOPICHELPER_H
#define INERTIAL_SENSE_IMX_TOPICHELPER_H

//#include <std_msgs/msg/detail/string__struct.hpp>

#include "InertialSense.h"
#ifdef ROS2
#include "rclcpp/rclcpp/rclcpp.hpp"
#include <inertial_sense_ros2/msg/didins1.hpp>
#include <inertial_sense_ros2/msg/didins2.hpp>
#include <inertial_sense_ros2/msg/didins4.hpp>
#include <inertial_sense_ros2/msg/gps_info.hpp>
#include <inertial_sense_ros2/msg/gps.hpp>
#include <inertial_sense_ros2/msg/inl2_states.hpp>
#include <inertial_sense_ros2/msg/pimu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/detail/fluid_pressure__traits.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include "diagnostic_msgs/diagnostic_msgs/msg/diagnostic_array.hpp"
#include "inertial_sense_ros2/msg/rtk_rel.hpp"
#include "inertial_sense_ros2/msg/rtk_info.hpp"
#include "inertial_sense_ros2/msg/glonass_ephemeris.hpp"
#include "inertial_sense_ros2/msg/gnss_ephemeris.hpp"
#include "inertial_sense_ros2/msg/gnss_observation.hpp"
#include "inertial_sense_ros2/msg/gnss_obs_vec.hpp"
#endif

#ifdef ROS1
#include "ros/ros.h"
#endif
class TopicHelper
{
public:

    void streamingCheck(eDataIDs did)
    {
        streamingCheck(did, streaming);
    }
    void streamingCheck(eDataIDs did, bool &stream)
    {
        if (!stream)
        {
            stream = true;
#ifdef ROS2
            rclcpp::Logger logger_stream_check = rclcpp::get_logger("stream_check");
            logger_stream_check.set_level(rclcpp::Logger::Level::Debug);
            RCLCPP_DEBUG(logger_stream_check,"%s response received", cISDataMappings::DataName(did)); //???
#endif
#ifdef ROS1
            ROS_DEBUG("%s response received", cISDataMappings::DataName(did));
#endif
        }
    }

    std::string topic;
    bool enabled = false;
    bool streaming = false;
    int period = 1;             // Period multiple (data rate divisor)
#ifdef ROS2
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr pub_diagnostics;
    rclcpp::Publisher<inertial_sense_ros2::msg::DIDINS1>::SharedPtr pub_didins1;
    rclcpp::Publisher<inertial_sense_ros2::msg::DIDINS2>::SharedPtr pub_didins2;
    rclcpp::Publisher<inertial_sense_ros2::msg::DIDINS4>::SharedPtr pub_didins4;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odometry;
    rclcpp::Publisher<inertial_sense_ros2::msg::INL2States>::SharedPtr pub_inl2;
    rclcpp::Publisher<inertial_sense_ros2::msg::PIMU>::SharedPtr pub_pimu;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu;
    rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr pub_bfield;
    rclcpp::Publisher<sensor_msgs::msg::FluidPressure>::SharedPtr pub_fpres;
    rclcpp::Publisher<inertial_sense_ros2::msg::GPS>::SharedPtr pub_gps;
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr pub_nsf;
    rclcpp::Publisher<inertial_sense_ros2::msg::GPSInfo>::SharedPtr pub_gpsinfo1;
    rclcpp::Publisher<inertial_sense_ros2::msg::GPSInfo>::SharedPtr pub_gpsinfo2;
#endif
#ifdef ROS1
    ros::Publisher pub;
#endif
};

class TopicHelperGps: public TopicHelper
{
public:
    bool streaming_pos = false;
    bool streaming_vel = false;
    std::string type = "F9P";
    float antennaOffset[3] = {0, 0, 0};
};

class TopicHelperGpsRtk: public TopicHelper
{
public:
    bool streamingMisc = false;
    bool streamingRel = false;
#ifdef ROS2
    rclcpp::Publisher<inertial_sense_ros2::msg::RTKInfo>::SharedPtr pubInfo;
    rclcpp::Publisher<inertial_sense_ros2::msg::RTKRel>::SharedPtr pubRel;
#endif
#ifdef ROS1
    ros::Publisher pubInfo;
    ros::Publisher pubRel;
#endif
};

class TopicHelperGpsRaw: public TopicHelper
{
public:
    std::string topicObs;
    std::string topicEph;
    std::string topicGEp;
#ifdef ROS2
    rclcpp::Publisher<inertial_sense_ros2::msg::GNSSObsVec>::SharedPtr pubObs;
    rclcpp::Publisher<inertial_sense_ros2::msg::GNSSEphemeris>::SharedPtr pubEph;
    rclcpp::Publisher<inertial_sense_ros2::msg::GlonassEphemeris>::SharedPtr pubGEp;
    rclcpp::TimerBase::SharedPtr obs_bundle_timer;
    rclcpp::Time last_obs_time;
#endif
#ifdef ROS1
    ros::Publisher pubObs;
    ros::Publisher pubEph;
    ros::Publisher pubGEp;
    ros::Timer obs_bundle_timer;
    ros::Time last_obs_time;
#endif
};


#endif //INERTIAL_SENSE_IMX_TOPICHELPER_H
