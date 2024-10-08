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
#ifdef ROS2
using namespace rclcpp;
#endif
#ifdef ROS1
using namespace ros;
#endif
int main(int argc, char**argv)
{
    InertialSenseROS* thing;
    init(argc, argv
#ifdef ROS1
                   , "inertial_sense_node"
#endif
                   );
    //auto nh_ = std::make_shared<rclcpp::Node>("nh_");
    if (argc > 1)
    {
        std::string paramYamlPath = argv[1];
        std::cout << "\n\nLoading YAML paramfile: " << paramYamlPath << "\n\n";
        YAML::Node node;
        try
        {
            node = YAML::LoadFile(paramYamlPath);
        }
        catch (const YAML::BadFile &bf)
        {
            std::cout << "Loading file \"" << paramYamlPath << "\" failed.  Using default parameters.\n\n";
            node = YAML::Node(YAML::NodeType::Undefined);
        }

        thing = new InertialSenseROS(node);
    }
    else
    {
        thing = new InertialSenseROS;
    }

    thing->initialize();
    while (ok())
    {
#ifdef ROS2
        spin_some(thing->nh_);
#endif
#ifdef ROS1
        ros::spinOnce();
#endif
        thing->update();
    }
    return 0;
}
