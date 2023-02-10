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

#ifndef INERTIAL_SENSE_IMX_RTKROVER_H
#define INERTIAL_SENSE_IMX_RTKROVER_H

#include "ros/ros.h"

#include "ParamHelper.h"

class RtkRoverCorrectionProvider {
protected:
    ParamHelper ph_;

public:
    std::string type_;
    std::string protocol_; // format
    RtkRoverCorrectionProvider(YAML::Node& node, const std::string& t) : ph_(node), type_(t) { };
    virtual void configure(YAML::Node& node) = 0;
};

class RtkRoverCorrectionProvider_Ntrip : public RtkRoverCorrectionProvider {
public:
    std::string correction_type_;
    std::string correction_protocol_;
    std::string ip_ = "127.0.0.1";
    int port_ = 7777;
    std::string mount_point_;
    std::string username_;
    std::string password_;

    // todo: move these to private fields
    bool connecting_ = false;

    int connection_attempt_limit_ = 1;
    int connection_attempt_backoff_ = 2;
    int traffic_total_byte_count_ = 0;
    int data_transmission_interruption_count_ = 0;
    int data_transmission_interruption_limit_ = 5;
    bool connectivity_watchdog_enabled_ = true;
    float connectivity_watchdog_timer_frequency_ = 1;

    RtkRoverCorrectionProvider_Ntrip(YAML::Node& node) : RtkRoverCorrectionProvider(node, "ntrip") { configure(node); }
    void configure(YAML::Node& node);
    std::string get_connection_string();
};

class RtkRoverCorrectionProvider_Serial : public RtkRoverCorrectionProvider {
public:
    std::string port_ = "/dev/ttyACM0";
    int baud_rate_ = 115200;
    RtkRoverCorrectionProvider_Serial(YAML::Node& node) : RtkRoverCorrectionProvider(node, "serial") { configure(node); }
    virtual void configure(YAML::Node& node);
};

class RtkRoverCorrectionProvider_ROS : public RtkRoverCorrectionProvider {
public:
    std::string topic_ = "/rtcm3_corrections";
    RtkRoverCorrectionProvider_ROS(YAML::Node& node) : RtkRoverCorrectionProvider(node, "ros_topic"){ configure(node); }
    virtual void configure(YAML::Node& node);
};

class RtkRoverCorrectionProvider_EVB : public RtkRoverCorrectionProvider {
public:
    std::string port_ = "xbee";
    RtkRoverCorrectionProvider_EVB(YAML::Node &node) : RtkRoverCorrectionProvider(node, "xbee_radio") { configure(node); }
    virtual void configure(YAML::Node &node);
};


class RtkRoverProvider {
protected:
    ParamHelper ph_;

public:
    bool enable = false;               // Enables/Disables the entire provider
    bool compassing_enable = false;     // Enable RTK compassing (dual GNSS moving baseline RTK) at GPS2
    bool positioning_enable = false;    // Enable RTK precision positioning at GPS1

    RtkRoverCorrectionProvider* correction_input;
    RtkRoverProvider(YAML::Node node) : ph_((YAML::Node&)node) { configure(node); }
    void configure(YAML::Node& n);
};

class RtkRoverCorrectionProviderFactory {
public:
    static RtkRoverCorrectionProvider* buildCorrectionProvider(YAML::Node &node) {
        std::string type = node["type"].as<std::string>();
        std::transform(type.begin(), type.end(), type.begin(), ::tolower);
        if (type == "ntrip") return new RtkRoverCorrectionProvider_Ntrip(node);
        else if (type == "serial") return new RtkRoverCorrectionProvider_Serial(node);
        else return nullptr;
    }
};

#endif //INERTIAL_SENSE_IMX_RTKROVER_H
