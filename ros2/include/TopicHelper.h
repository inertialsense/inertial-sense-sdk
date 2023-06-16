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

#include "InertialSense.h"
#include "inertial_sense_ros/msg/rtk_rel.hpp"
#include "inertial_sense_ros/msg/rtk_info.hpp"
#include "inertial_sense_ros/msg/gnss_ephemeris.hpp"
#include "inertial_sense_ros/msg/glonass_ephemeris.hpp"
#include <rclcpp/rclcpp.hpp>

template <typename T = std_msgs::msg::Header>
class TopicHelper
{
public:

    void streamingCheck(rclcpp::Node::SharedPtr nh_, eDataIDs did)
    {
        streamingCheck(nh_, did, streaming);
    }
    void streamingCheck(rclcpp::Node::ConstSharedPtr nh_, eDataIDs did, bool &stream)
    {
        if (!stream)
        {
            stream = true;
            RCLCPP_DEBUG(nh_->get_logger(), "%s response received", cISDataMappings::GetDataSetName(did));
        }
    }

    std::string topic;
    bool enabled = false;
    bool streaming = false;
    int period = 1;             // Period multiple (data rate divisor)
    typename rclcpp::Publisher<T>::SharedPtr  pub;
};


template <typename T>
class TopicHelperGps: public TopicHelper<T>
{
public:
    bool streaming_pos = false;
    bool streaming_vel = false;
    std::string type = "F9P";
    float antennaOffset[3] = {0, 0, 0};
};

class TopicHelperGpsRtk: public TopicHelper<>
{
public:
    bool streamingMisc = false;
    bool streamingRel = false;
    rclcpp::Publisher<inertial_sense_ros::msg::RTKInfo>::SharedPtr pubInfo;
    rclcpp::Publisher<inertial_sense_ros::msg::RTKRel>::SharedPtr pubRel;
};

template <typename T>
class TopicHelperGpsRaw: public TopicHelper<>
{
public:
    std::string topicObs;
    std::string topicEph;
    std::string topicGEp;
    typename rclcpp::Publisher<T>::SharedPtr pubObs;
    rclcpp::Publisher<inertial_sense_ros::msg::GNSSEphemeris>::SharedPtr pubEph;
    rclcpp::Publisher<inertial_sense_ros::msg::GlonassEphemeris>::SharedPtr pubGEp;
    rclcpp::TimerBase::ConstSharedPtr obs_bundle_timer;
    rclcpp::Time last_obs_time;
};
