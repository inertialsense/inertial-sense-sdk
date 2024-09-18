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

#include "ParamHelper.h"

void ParamHelper::setCurrentNode(YAML::Node node)
{
    node ? currentNode_.reset(node) : currentNode_.reset(YAML::Node());
}

YAML::Node ParamHelper::node(YAML::Node &node, std::string key, int indent)
{
    setCurrentNode(node[key]);
    return currentNode_;
}

/**
 * Attempts to populate a TopicHelper properties, if stanza specified by 'key' exists.  If the key exists,
 * but no other parameter nodes beneath it, those values will default. If the key does not exist, it is automatically
 * @param th
 * @param key
 * @param topicDefault
 * @param enabledDefault
 * @param periodDefault
 * @return
 */
bool ParamHelper::msgParams(TopicHelper &th, std::string key, std::string topicDefault, bool enabledDefault, int periodDefault, bool implicitEnable)
{
    YAML::Node msgNode;
    if (!currentNode_[key]) {
        th.topic = (topicDefault.empty() ? key : topicDefault);
        th.enabled = implicitEnable;
        th.period = periodDefault;
        return false;
    }

    msgNode = currentNode_[key];
    nodeParam(msgNode, "topic",  th.topic, (topicDefault.empty() ? key : topicDefault));
    nodeParam(msgNode, "enable", th.enabled, enabledDefault);
    nodeParam(msgNode, "period", th.period, periodDefault);
    return true;
}

bool ParamHelper::msgParamsImplicit(TopicHelper &th, std::string key, std::string topicDefault, bool enabledDefault, int periodDefault) {
    return msgParams(th, key, topicDefault, enabledDefault, periodDefault, enabledDefault);
}

#ifdef ROS1
YAML::Node xmlRpcToYamlNode(XmlRpc::XmlRpcValue &v)
{
    YAML::Node node;

    switch(v.getType())
    {
        case XmlRpc::XmlRpcValue::TypeString:   node = static_cast<std::string>(v);    break;
        case XmlRpc::XmlRpcValue::TypeBoolean:  node = static_cast<bool>(v);           break;
        case XmlRpc::XmlRpcValue::TypeDouble:   node = static_cast<double>(v);         break;
        case XmlRpc::XmlRpcValue::TypeInt:      node = static_cast<int>(v);            break;
        case XmlRpc::XmlRpcValue::TypeArray:
            for (int i=0; i<v.size(); i++)
            {
                node[i] = xmlRpcToYamlNode(v[i]);
            }
        break;
    }

    return node;
}
#endif
bool ParamHelper::paramServerToYamlNode(YAML::Node &node, std::string nhKey, std::string indentStr)
{
#ifdef ROS2
    auto nh_ = std::make_shared<rclcpp::Node>("nh_");;
#endif
#ifdef ROS1
    ros::NodeHandle nh;
#endif
    std::vector <std::string> nhKeyList;
#ifdef ROS2
    nh_->get_parameters(nhKeyList);
#endif
#ifdef ROS1
    nh.getParamNames(nhKeyList);
#endif
    for (std::string key: nhKeyList)
    {
        std::string::size_type pos = key.find(nhKey);
        if (pos != 0)
        {   // key path doesn't match
            continue;
        }

        // Get string following nkKey
        std::string name = key.substr(pos+nhKey.size());

        pos = name.find('/');
        if (pos == std::string::npos)
        {   // Param name

#ifdef ROS1
            XmlRpc::XmlRpcValue v;
            if (nh.getParam(key, v))
            {
                node[name] = xmlRpcToYamlNode(v);
                continue;
            }
#endif
        }
        else
        {   // Node key
            std::string nodeName = name.substr(0, pos);

            if (!node[nodeName])
            {   // Create new child node
                YAML::Node childNode;
                ParamHelper::paramServerToYamlNode(childNode, nhKey + nodeName + "/", indentStr + "  ");
                node[nodeName] = childNode;
            }
        }
    }

    return true;
}
