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

#include "../include/RtkBase.h"
#ifdef ROS2
#define ROS_ERROR(msg) RCLCPP_ERROR(rclcpp::get_logger("Inertial_Sense_ROS"), msg)
#define ROS_ERROR_STREAM(msg) RCLCPP_ERROR_STREAM(rclcpp::get_logger("Inertial_Sense_ROS"), msg)
#endif
//rclcpp::Node::SharedPtr rtk_base_node = std::make_shared<rclcpp::Node>("rtk_node");
//rclcpp::spin_some(rtk_base_node);
RtkBaseProvider::base_gps_source RtkBaseProvider::gpsSourceParamToEnum(YAML::Node& n, std::string v, std::string d) {
    std::string param = d;

    ph_.nodeParam(n, v, param, d);
    std::transform(param.begin(), param.end(), param.begin(), ::tolower);
    return (param == "gps1" ? GPS1 : (param == "gps2" ? GPS2 : OFF));
}

void RtkBaseProvider::configure(YAML::Node& node) {
    if (node.IsDefined() && !node.IsNull()) {
        ph_.setCurrentNode(node);
        ph_.nodeParam(node, "enable", enable, true);
        ph_.nodeParam(node, "compassing_enable", compassing_enable_, false);
        ph_.nodeParam(node, "positioning_enable", positioning_enable_, false);

        YAML::Node rtkBaseSource = ph_.node(node, "correction_source");
        source_gps__serial0_ = gpsSourceParamToEnum(rtkBaseSource, "imx_serial_0");
        source_gps__serial1_ = gpsSourceParamToEnum(rtkBaseSource, "imx_serial_1");
        source_gps__serial2_ = gpsSourceParamToEnum(rtkBaseSource, "imx_serial_2");
        source_gps__usb_ = gpsSourceParamToEnum(rtkBaseSource, "imx_usb");

        if (node["correction_outputs"].IsDefined() && !node["correction_outputs"].IsNull()) {
            YAML::Node rtkBaseOutputs = ph_.node(node, "correction_outputs");

            ph_.nodeParam("format", protocol_, "RTCM3");
            YAML::Node rtkBase_providers = ph_.node(rtkBaseOutputs, "select");
            if (rtkBase_providers.IsSequence()) {
                for (auto it = rtkBase_providers.begin(); it != rtkBase_providers.end(); it++) {
                    const YAML::Node &provider_node = ph_.node(rtkBaseOutputs, (*it).as<std::string>());
                    RtkBaseCorrectionProvider *correctionProvider = RtkBaseCorrectionProviderFactory::buildProvider(*this, (YAML::Node &) provider_node);
                    if (correctionProvider == nullptr) {
                        ROS_ERROR_STREAM("Unable to configure RosBaseCorrectionProvider [" << (*it).as<std::string>() << "]. Please validate the configuration:\n\n" << node << "\n\n");
                    } else {
                        correction_outputs_.push_back(reinterpret_cast<const RtkBaseCorrectionProvider *const>(correctionProvider));
                    }
                }
            } else if (rtkBase_providers.IsScalar()) {
                const YAML::Node &provider_node = ph_.node(rtkBaseOutputs, rtkBase_providers.as<std::string>());
                RtkBaseCorrectionProvider *correctionProvider = RtkBaseCorrectionProviderFactory::buildProvider(*this, (YAML::Node &) provider_node);
                if (correctionProvider == nullptr) {
                    ROS_ERROR_STREAM("Unable to configure RosBaseCorrectionProvider [" << rtkBase_providers.as<std::string>() << "]. Please validate the configuration:\n\n" << node << "\n\n");
                } else {
                    correction_outputs_.push_back(reinterpret_cast<const RtkBaseCorrectionProvider *const>(correctionProvider));
                }
            }
        } else {
            ROS_ERROR_STREAM("No \"correction_outputs\" configuration has been provided.  Please validate the configuration:\n\n" << node << "\n\n");
        }
    } else {
        ROS_ERROR("Unable to configure RosBaseProvider. The YAML node was null or undefined.");
    }
}

RtkBaseCorrectionProvider* RtkBaseProvider::getProvidersByType(std::string type) {
    std::vector<const RtkBaseCorrectionProvider*>::iterator iter;
    for (iter = correction_outputs_.begin(); iter != correction_outputs_.end(); iter++) {
        if ((*iter)->type_ == type)
            return (RtkBaseCorrectionProvider*)*iter;
    }

    return nullptr;
}

/*
 *==================  RtkBaseCorrectionProvider_Ntrip ==================*
 */
void RtkBaseCorrectionProvider_Ntrip::configure(YAML::Node& node) {
    if (node.IsDefined() && !node.IsNull()) {
        ph_.setCurrentNode(node);
        ph_.nodeParam("ip_address", ip_);
        ph_.nodeParam("ip_port", port_);
        ph_.nodeParam("mount_mount", mount_point_);
        ph_.nodeParam("username", username_);
        ph_.nodeParam("password", password_);
        ph_.nodeParam("credentials", credentials_);
    } else {
        ROS_ERROR("Unable to configure RtkBaseCorrectionProvider_Ntrip. The YAML node was null or undefined.");
    }
}

std::string RtkBaseCorrectionProvider_Ntrip::get_connection_string() {
    std::string RTK_connection = "TCP:" + protocol_ + ":" + ip_ + ":" + std::to_string(port_);
    return RTK_connection;
}

/*
 *==================  RtkBaseCorrectionProvider_Serial ==================*
 */
void RtkBaseCorrectionProvider_Serial::configure(YAML::Node& node) {
    if (node.IsDefined() && !node.IsNull()) {
        ph_.setCurrentNode(node);
        ph_.nodeParam("port", port_);
        ph_.nodeParam("baud_rate", baud_rate_);
    } else {
        ROS_ERROR("Unable to configure RtkBaseCorrectionProvider_Serial. The YAML node was null or undefined.");
    }
}

/*
 *==================  RtkBaseCorrectionProvider_ROS ==================*
 */
void RtkBaseCorrectionProvider_ROS::configure(YAML::Node& node) {
    if (node.IsDefined() && !node.IsNull()) {
        ph_.setCurrentNode(node);
        ph_.nodeParam("topic", topic_);
    } else {
        ROS_ERROR("Unable to configure RtkBaseCorrectionProvider_ROS. The YAML node was null or undefined.");
    }
}

/*
 *==================  RtkBaseCorrectionProvider_EVB ==================*
 */
void RtkBaseCorrectionProvider_EVB::configure(YAML::Node& node) {
    if (node.IsDefined() && !node.IsNull()) {
        ph_.setCurrentNode(node);
        ph_.nodeParam("port", port_);
    } else {
        ROS_ERROR("Unable to configure RtkBaseCorrectionProvider_EVB. The YAML node was null or undefined.");
    }
}

