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

#include "RtkRover.h"

void RtkRoverProvider::configure(YAML::Node& node) {
    if (node.IsDefined() && !node.IsNull()) {
        ph_.setCurrentNode(node);
        ph_.nodeParam(node, "compassing_enable", compassing_enable, false);
        ph_.nodeParam(node, "positioning_enable", positioning_enable, false);

        if (node["correction_input"].IsDefined() && !node["correction_input"].IsNull()) {
            std::string correction_src = node["correction_input"]["select"].as<std::string>();
            YAML::Node inputNode = node["correction_input"][correction_src];
            if (inputNode.IsDefined() && !inputNode.IsNull()) {
                correction_input = RtkRoverCorrectionProviderFactory::buildCorrectionProvider(l_, inputNode);
                if (correction_input == nullptr) {
                    RCLCPP_ERROR_STREAM(l_,"Unable to configure RosRoverCorrectionProviders. Please validate the configuration:\n\n" << node << "\n\n");
                }
            } else {
                RCLCPP_ERROR_STREAM(l_,"The specified Correction Provider [" << correction_src << "] can't be located in the config. Please validate the configuration:\n\n" << node << "\n\n");
            }
        } else {
            RCLCPP_ERROR_STREAM(l_,"No \"correction_input\" configuration has been provided.  Please validate the configuration:\n\n" << node << "\n\n");
        }
    } else {
        RCLCPP_ERROR(l_,"Unable to configure RosRoverProvider. The YAML node was null or undefined.");
    }
}

/*
 *==================  RtkRoverCorrectionProvider_Ntrip ==================*
 */
void RtkRoverCorrectionProvider_Ntrip::configure(YAML::Node& node) {
    if (node.IsDefined() && !node.IsNull()) {
        ph_.setCurrentNode(node);
        ph_.nodeParam("type", type_, "NTRIP");
        ph_.nodeParam("format", protocol_, "RTCM3");
        ph_.nodeParam("ip_address", ip_);
        ph_.nodeParam("ip_port", port_);
        ph_.nodeParam("mount_point", mount_point_);
        ph_.nodeParam("username", username_);
        ph_.nodeParam("password", password_);
        ph_.setCurrentNode(node["connection_attempts"]);
        ph_.nodeParam("limit", connection_attempt_limit_, 1);
        ph_.nodeParam("backoff", connection_attempt_backoff_, 2);
        ph_.setCurrentNode(node["watchdog"]);
        ph_.nodeParam("enable", connectivity_watchdog_enabled_, true);
        ph_.nodeParam("interval", connectivity_watchdog_timer_frequency_, 1);
    } else {
        RCLCPP_ERROR(l_,"Unable to configure RtkRoverCorrectionProvider_Ntrip. The YAML node was null or undefined.");
    }
}

std::string RtkRoverCorrectionProvider_Ntrip::get_connection_string() {
    std::string RTK_connection = "TCP:" + protocol_ + ":" + ip_ + ":" + std::to_string(port_);
    if (!mount_point_.empty() || !username_.empty())
        RTK_connection.append(":" + mount_point_);
    if (!username_.empty()) {
        RTK_connection.append(":" + username_);
        if (!password_.empty())
            RTK_connection.append(":" + password_);
    }

    return RTK_connection;
}

void RtkRoverCorrectionProvider_Ntrip::connect_rtk_client()
{
    if (is_ == nullptr) {
        RCLCPP_FATAL(l_, "RTK Client connection requested, but configureIS() hasn't been called in the provider.");
        rclcpp::shutdown();
        connecting_ = false;
        return;
    }
    connecting_ = true;

    // [type]:[protocol]:[ip/url]:[port]:[mountpoint]:[username]:[password]
    std::string RTK_connection = get_connection_string();

    int RTK_connection_attempt_count = 0;
    while (RTK_connection_attempt_count < connection_attempt_limit_)
    {
        ++RTK_connection_attempt_count;

        bool connected = is_->OpenConnectionToServer(RTK_connection);

        if (connected)
        {
            RCLCPP_INFO_STREAM(l_, "Successfully connected to " << RTK_connection << " RTK server");
            break;
        }
        else
        {
            RCLCPP_ERROR_STREAM(l_,"Failed to connect to base server at " << RTK_connection);

            if (RTK_connection_attempt_count >= connection_attempt_limit_)
            {
                RCLCPP_ERROR_STREAM(l_,"Giving up after " << RTK_connection_attempt_count << " failed attempts");
            }
            else
            {
                int sleep_duration = RTK_connection_attempt_count * connection_attempt_backoff_;
                RCLCPP_WARN_STREAM(l_, "Retrying connection in " << sleep_duration << " seconds");
                rclcpp::sleep_for(std::chrono::seconds(sleep_duration));
            }
        }
    }

    connecting_ = false;
}

void RtkRoverCorrectionProvider_Ntrip::connectivity_watchdog_timer_callback()
{
    if (connecting_ && (is_ != nullptr))
        return;

    int latest_byte_count = is_->GetClientServerByteCount();
    if (traffic_total_byte_count_ == latest_byte_count)
    {
        ++data_transmission_interruption_count_;

        if (data_transmission_interruption_count_ >= data_transmission_interruption_limit_)
        {
            RCLCPP_WARN(l_, "RTK transmission interruption, reconnecting...");
            connect_rtk_client();
        }
    }
    else
    {
        traffic_total_byte_count_ = latest_byte_count;
        data_transmission_interruption_count_ = 0;
    }
}

//void RtkRoverCorrectionProvider_Ntrip::start_connectivity_watchdog_timer(rclcpp::Node::SharedPtr nh)
//{
    //if (!connectivity_watchdog_enabled_) {
        //return;
    //}

    //if(!connectivity_watchdog_timer_) {
        //auto period = std::chrono::duration<float>(1/connectivity_watchdog_timer_frequency_);
        //connectivity_watchdog_timer_ = nh->create_wall_timer(period, std::bind(&RtkRoverCorrectionProvider_Ntrip::connectivity_watchdog_timer_callback, this));
    //}

    //connectivity_watchdog_timer_->reset();
//}

//void RtkRoverCorrectionProvider_Ntrip::stop_connectivity_watchdog_timer()
//{
    //connectivity_watchdog_timer_->cancel();
    //traffic_total_byte_count_ = 0;
    //data_transmission_interruption_count_ = 0;
//}

/*
 *==================  RtkRoverCorrectionProvider_Serial ==================*
 */
void RtkRoverCorrectionProvider_Serial::configure(YAML::Node& node) {
    if (node.IsDefined() && !node.IsNull()) {
        ph_.setCurrentNode(node);
        ph_.nodeParam("format", protocol_, "RTCM3");
        ph_.nodeParam("port", port_);
        ph_.nodeParam("baud_rate", baud_rate_);
    } else {
        RCLCPP_ERROR(l_,"Unable to configure RtkRoverCorrectionProvider_Serial. The YAML node was null or undefined.");
    }
}

/*
 *==================  RtkRoverCorrectionProvider_ROS ==================*
 */
void RtkRoverCorrectionProvider_ROS::configure(YAML::Node& node) {
    if (node.IsDefined() && !node.IsNull()) {
        ph_.setCurrentNode(node);
        ph_.nodeParam("format", protocol_, "RTCM3");
        ph_.nodeParam("topic", topic_);
    } else {
        RCLCPP_ERROR(l_,"Unable to configure RtkRoverCorrectionProvider_ROS. The YAML node was null or undefined.");
    }
}

/*
 *==================  RtkRoverCorrectionProvider_EVB ==================*
 */
void RtkRoverCorrectionProvider_EVB::configure(YAML::Node& node) {
    if (node.IsDefined() && !node.IsNull()) {
        ph_.setCurrentNode(node);
        ph_.nodeParam("format", protocol_, "RTCM3");
        ph_.nodeParam("port", port_);
    } else {
        RCLCPP_ERROR(l_,"Unable to configure RtkRoverCorrectionProvider_EVB. The YAML node was null or undefined.");
    }
}

