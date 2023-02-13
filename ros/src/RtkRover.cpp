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

void RtkRoverProvider::configure(YAML::Node& n) {
    ph_.setCurrentNode(n);
    ph_.nodeParam("compassing_enable", compassing_enable, false);
    ph_.nodeParam("positioning_enable", positioning_enable, false);

    YAML::Node roverInput = ph_.node(n, "correction_input");
    YAML::Node roverSettings = ph_.node(roverInput, roverInput["select"].as<std::string>());
    correction_input = RtkRoverCorrectionProviderFactory::buildCorrectionProvider(roverSettings);
}

/*
 *==================  RtkRoverCorrectionProvider_Ntrip ==================*
 */
void RtkRoverCorrectionProvider_Ntrip::configure(YAML::Node& node) {
    ph_.setCurrentNode(node);
    ph_.nodeParam("type", type_, "NTRIP");
    ph_.nodeParam("format", protocol_, "RTCM3");
    ph_.nodeParam("ip_address", ip_);
    ph_.nodeParam("ip_port", port_);
    ph_.nodeParam("mount_point", mount_point_);
    ph_.nodeParam("username", username_);
    ph_.nodeParam("password", password_);
}

std::string RtkRoverCorrectionProvider_Ntrip::get_connection_string() {
    std::string RTK_connection = "TCP:" + protocol_ + ":" + ip_ + ":" + std::to_string(port_);
    if (!mount_point_.empty() && !username_.empty())
    { // NTRIP options
        RTK_connection += ":" + mount_point_ + ":" + username_ + ":" + password_;
    }
    return RTK_connection;
}

void RtkRoverCorrectionProvider_Ntrip::connect_rtk_client()
{
    if (is_ == nullptr) {
        ROS_FATAL("RTK Client connection requested, but configureIS() hasn't been called in the provider.");
        ros::shutdown();
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
            ROS_INFO_STREAM("Successfully connected to " << RTK_connection << " RTK server");
            break;
        }
        else
        {
            ROS_ERROR_STREAM("Failed to connect to base server at " << RTK_connection);

            if (RTK_connection_attempt_count >= connection_attempt_limit_)
            {
                ROS_ERROR_STREAM("Giving up after " << RTK_connection_attempt_count << " failed attempts");
            }
            else
            {
                int sleep_duration = RTK_connection_attempt_count * connection_attempt_backoff_;
                ROS_WARN_STREAM("Retrying connection in " << sleep_duration << " seconds");
                ros::Duration(sleep_duration).sleep();
            }
        }
    }

    connecting_ = false;
}

void RtkRoverCorrectionProvider_Ntrip::connectivity_watchdog_timer_callback(const ros::TimerEvent &timer_event)
{
    if (connecting_ && (is_ != nullptr))
        return;

    int latest_byte_count = is_->GetClientServerByteCount();
    if (traffic_total_byte_count_ == latest_byte_count)
    {
        ++data_transmission_interruption_count_;

        if (data_transmission_interruption_count_ >= data_transmission_interruption_limit_)
        {
            ROS_WARN("RTK transmission interruption, reconnecting...");
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

    if (!connectivity_watchdog_timer_.isValid()) {
        connectivity_watchdog_timer_ = nh_->createTimer(ros::Duration(connectivity_watchdog_timer_frequency_), &RtkRoverCorrectionProvider_Ntrip::connectivity_watchdog_timer_callback, this);
    }

    connectivity_watchdog_timer_.start();
}

void RtkRoverCorrectionProvider_Ntrip::stop_connectivity_watchdog_timer()
{
    connectivity_watchdog_timer_.stop();
    traffic_total_byte_count_ = 0;
    data_transmission_interruption_count_ = 0;
}

/*
 *==================  RtkRoverCorrectionProvider_Serial ==================*
 */
void RtkRoverCorrectionProvider_Serial::configure(YAML::Node& node) {
    ph_.setCurrentNode(node);
    ph_.nodeParam("port", port_);
    ph_.nodeParam("baud_rate", baud_rate_);
}

/*
 *==================  RtkRoverCorrectionProvider_ROS ==================*
 */
void RtkRoverCorrectionProvider_ROS::configure(YAML::Node& node) {
    ph_.setCurrentNode(node);
    ph_.nodeParam("topic", topic_);
}

/*
 *==================  RtkRoverCorrectionProvider_EVB ==================*
 */
void RtkRoverCorrectionProvider_EVB::configure(YAML::Node& node) {
    ph_.setCurrentNode(node);
    ph_.nodeParam("port", port_);
}

