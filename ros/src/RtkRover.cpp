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
    ph_.nodeParam("type", correction_type_, "NTRIP");
    ph_.nodeParam("format", correction_protocol_, "RTCM3");
    ph_.nodeParam("ip_address", ip_);
    ph_.nodeParam("ip_port", port_);
    ph_.nodeParam("mount_point", mount_point_);
    ph_.nodeParam("username", username_);
    ph_.nodeParam("password", password_);
}

std::string RtkRoverCorrectionProvider_Ntrip::get_connection_string() {
    std::string RTK_connection = "TCP:" + correction_protocol_ + ":" + ip_ + ":" + std::to_string(port_);
    if (!mount_point_.empty() && !username_.empty())
    { // NTRIP options
        RTK_connection += ":" + mount_point_ + ":" + username_ + ":" + password_;
    }
    return RTK_connection;
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

