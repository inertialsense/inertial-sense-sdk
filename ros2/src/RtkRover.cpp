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
        compassing_enable = nh_->declare_parameter<bool>("rtk_compass", false);
        ph_.nodeParam(node, "compassing_enable", compassing_enable, compassing_enable);
        ph_.nodeParam(node, "positioning_enable", positioning_enable, false);

        if (node["correction_input"].IsDefined() && !node["correction_input"].IsNull()) {
            std::string correction_src = node["correction_input"]["select"].as<std::string>();
            YAML::Node inputNode = node["correction_input"][correction_src];
            if (inputNode.IsDefined() && !inputNode.IsNull()) {
                correction_input = RtkRoverCorrectionProviderFactory::buildCorrectionProvider(inputNode);
                if (correction_input == nullptr) {
                    RCLCPP_ERROR_STREAM(rclcpp::get_logger("unable_to_configure_RRCP"),"Unable to configure RosRoverCorrectionProviders. Please validate the configuration:\n\n" << node << "\n\n");
                }
            } else {
                RCLCPP_ERROR_STREAM(rclcpp::get_logger("scp_can't_be_located"),"The specified Correction Provider [" << correction_src << "] can't be located in the config. Please validate the configuration:\n\n" << node << "\n\n");
            }
        } else {
            RCLCPP_ERROR_STREAM(rclcpp::get_logger("no_config_provided"),"No \"correction_input\" configuration has been provided.  Please validate the configuration:\n\n" << node << "\n\n");
        }
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("unable_to_config_RRP"),"Unable to configure RosRoverProvider. The YAML node was null or undefined.");
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
        ip_ = nh_->declare_parameter<std::string>("rtk_server_IP", "127.0.0.1");
        ph_.nodeParam("ip_address", ip_);
        port_ = nh_->declare_parameter<int>("rtk_server_port", 7777);
        ph_.nodeParam("ip_port", port_);
        ph_.nodeParam("mount_point", mount_point_);
        ph_.nodeParam("username", username_);
        ph_.nodeParam("password", password_);
        ph_.setCurrentNode(node["connection_attempts"]);
        connection_attempt_limit_ = nh_->declare_parameter<int>("rtk_connection_attempt_limit", 1);
        ph_.nodeParam("limit", connection_attempt_limit_, connection_attempt_limit_);
        connection_attempt_backoff_ = nh_->declare_parameter<int>("rtk_connection_attempt_backoff", 2);
        ph_.nodeParam("backoff", connection_attempt_backoff_, connection_attempt_backoff_);
        ph_.setCurrentNode(node["watchdog"]);
        connectivity_watchdog_enabled_ = nh_->declare_parameter<bool>("rtk_connectivity_watchdog_enabled", false);
        ph_.nodeParam("enable", connectivity_watchdog_enabled_, connectivity_watchdog_enabled_);
        connectivity_watchdog_timer_frequency_ = nh_->declare_parameter<int>("rtk_connectivity_watchdog_timer_frequency", 1);
        ph_.nodeParam("interval", connectivity_watchdog_timer_frequency_, connectivity_watchdog_timer_frequency_);
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("unable_to_config_RRCP_Ntrip"),"Unable to configure RtkRoverCorrectionProvider_Ntrip. The YAML node was null or undefined.");
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
        RCLCPP_FATAL(rclcpp::get_logger("RTK_requested_configureIS_not_called"),"RTK Client connection requested, but configureIS() hasn't been called in the provider.");
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
            RCLCPP_INFO_STREAM(rclcpp::get_logger("success_connect_RTK"),"Successfully connected to " << RTK_connection << " RTK server");
            break;
        }
        else
        {
            RCLCPP_ERROR_STREAM(rclcpp::get_logger("failed_to_connect_base"),"Failed to connect to base server at " << RTK_connection);

            if (RTK_connection_attempt_count >= connection_attempt_limit_)
            {
                RCLCPP_ERROR_STREAM(rclcpp::get_logger("give_up_failed_attempts"),"Giving up after " << RTK_connection_attempt_count << " failed attempts");
            }
            else
            {
                int sleep_duration = RTK_connection_attempt_count * connection_attempt_backoff_;
                RCLCPP_WARN_STREAM(rclcpp::get_logger("retrying_connection"),"Retrying connection in " << sleep_duration << " seconds");
                //rclcpp::Duration(sleep_duration).sleep();
                rclcpp::Rate r(sleep_duration); r.sleep();
            }
        }
    }

    connecting_ = false;
}

void RtkRoverCorrectionProvider_Ntrip::connectivity_watchdog_timer_callback()
{
    if (connecting_ && (is_ != nullptr))
        return;

    int latest_byte_count = is_->ClientServerByteCount();
    if (traffic_total_byte_count_ == latest_byte_count)
    {
        ++data_transmission_interruption_count_;

        if (data_transmission_interruption_count_ >= data_transmission_interruption_limit_)
        {
            RCLCPP_WARN(rclcpp::get_logger("RTK_transmission_interrupt"),"RTK transmission interruption, reconnecting...");
            connect_rtk_client();
        }
    }
    else
    {
        traffic_total_byte_count_ = latest_byte_count;
        data_transmission_interruption_count_ = 0;
    }
}

void RtkRoverCorrectionProvider_Ntrip::start_connectivity_watchdog_timer()
{
    if (!connectivity_watchdog_enabled_) {
        return;
    }

    if (connectivity_watchdog_timer_->is_canceled()) {
        connectivity_watchdog_timer_ = nh_->create_wall_timer(std::chrono::duration<float>(connectivity_watchdog_timer_frequency_), std::bind(&RtkRoverCorrectionProvider_Ntrip::connectivity_watchdog_timer_callback, this));
    }

    //connectivity_watchdog_timer_.();
}

void RtkRoverCorrectionProvider_Ntrip::stop_connectivity_watchdog_timer()
{
    connectivity_watchdog_timer_->cancel();
    traffic_total_byte_count_ = 0;
    data_transmission_interruption_count_ = 0;
}




/*
 *==================  RtkRoverCorrectionProvider_Serial ==================*
 */
void RtkRoverCorrectionProvider_Serial::configure(YAML::Node& node) {
    if (node.IsDefined() && !node.IsNull()) {
        ph_.setCurrentNode(node);
        protocol_ = nh_->declare_parameter<std::string>("rtk_correction_protocol", "RTCM3");
        ph_.nodeParam("format", protocol_, protocol_);
        ph_.nodeParam("port", port_);
        ph_.nodeParam("baud_rate", baud_rate_);
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("unable_to_config_RRCP_serial"),"Unable to configure RtkRoverCorrectionProvider_Serial. The YAML node was null or undefined.");
    }
}

/*
 *==================  RtkRoverCorrectionProvider_ROS ==================*
 */
void RtkRoverCorrectionProvider_ROS::configure(YAML::Node& node) {
    if (node.IsDefined() && !node.IsNull()) {
        ph_.setCurrentNode(node);
        protocol_ = nh_->declare_parameter<std::string>("rtk_correction_protocol", "RTCM3");
        ph_.nodeParam("format", protocol_, protocol_);
        ph_.nodeParam("topic", topic_);
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("unable_to_config_RRCP_ROS"),"Unable to configure RtkRoverCorrectionProvider_ROS. The YAML node was null or undefined.");
    }
}

/*
 *==================  RtkRoverCorrectionProvider_EVB ==================*
 */
void RtkRoverCorrectionProvider_EVB::configure(YAML::Node& node) {
    if (node.IsDefined() && !node.IsNull()) {
        ph_.setCurrentNode(node);
        protocol_ = nh_->declare_parameter<std::string>("rtk_correction_protocol", "RTCM3");
        ph_.nodeParam("format", protocol_, protocol_);
        ph_.nodeParam("port", port_);
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("unable_to_config_RRCP_ROS"), "Unable to configure RtkRoverCorrectionProvider_EVB. The YAML node was null or undefined.");
    }
}

