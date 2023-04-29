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

#include "InertialSense.h"
#include "ros/ros.h"



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
            ROS_DEBUG("%s response received", cISDataMappings::GetDataSetName(did));
        }
    }

    std::string topic;
    bool enabled = false;
    bool streaming = false;
    int period = 1;             // Period multiple (data rate divisor)
    ros::Publisher pub;
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
    ros::Publisher pubInfo;
    ros::Publisher pubRel;
};

class TopicHelperGpsRaw: public TopicHelper
{
public:
    std::string topicObs;
    std::string topicEph;
    std::string topicGEp;
    ros::Publisher pubObs;
    ros::Publisher pubEph;
    ros::Publisher pubGEp;
    ros::Timer obs_bundle_timer;
    ros::Time last_obs_time;
};


#endif //INERTIAL_SENSE_IMX_TOPICHELPER_H
