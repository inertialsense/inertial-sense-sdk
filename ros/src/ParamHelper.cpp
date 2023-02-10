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

YAML::Node ParamHelper::node(YAML::Node &node, std::string key, int indent)
{
    print_indent(indent);
    indent_ = indent+1;
    std::cout << key << ":\n";
    setCurrentNode(node[key]);
    return currentNode_;
}

void ParamHelper::setCurrentNode(YAML::Node node)
{
    node ? currentNode_.reset(node) : currentNode_.reset(YAML::Node());
}

#define PH_DEFAULT_MSG      "   (default)"

bool ParamHelper::msgParams(TopicHelper &th, std::string key, std::string topicDefault, bool enabledDefault, int periodDefault)
{
    print_indent(indent_++);
    std::cout << key << ":\n";

    YAML::Node msgNode;
    if (currentNode_[key])
    {
        msgNode = currentNode_[key];
    }
    else
    {
        indent_--;
        return false;
    }
    nodeParam(msgNode, "topic",  th.topic, (topicDefault.empty() ? key : topicDefault));
    nodeParam(msgNode, "enable", th.enabled, enabledDefault);
    nodeParam(msgNode, "period", th.period, periodDefault);
    indent_--;

    return true;
}


void ParamHelper::print_indent(int indent)
{
    for (int i=0; i<indent; i++)
    {
        std::cout << "  ";
    }
}


bool ParamHelper::paramServerToYamlNode(YAML::Node &node, std::string nhKey, std::string indentStr)
{
#define ENABLE_DEBUG_PRINT_TREE     0
    ros::NodeHandle nh;

    std::vector <std::string> nhKeyList;
    nh.getParamNames(nhKeyList);
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

            XmlRpc::XmlRpcValue v;
            if (nh.getParam(key, v))
            {
#if ENABLE_DEBUG_PRINT_TREE
                std::cout << indentStr << name << ": " << v << "\n";
#endif
                node[name] = xmlRpcToYamlNode(v);
                continue;
            }
        }
        else
        {   // Node key
            std::string nodeName = name.substr(0, pos);

            if (!node[nodeName])
            {   // Create new child node
#if ENABLE_DEBUG_PRINT_TREE
                std::cout << indentStr << nodeName << "\n";
#endif
                YAML::Node childNode;
                ParamHelper::paramServerToYamlNode(childNode, nhKey + nodeName + "/", indentStr + "  ");
                node[nodeName] = childNode;
            }
        }
    }

    return true;
}
