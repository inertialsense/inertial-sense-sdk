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

#ifndef INERTIAL_SENSE_IMX_PARAMHELPER_H
#define INERTIAL_SENSE_IMX_PARAMHELPER_H

#include <yaml-cpp/yaml.h>
#include "TopicHelper.h"

class ParamHelper
{
public:
    YAML::Node currentNode_;

    ParamHelper(YAML::Node &node){ setCurrentNode(node); }

    YAML::Node node(YAML::Node &node, std::string key, int indent=1);
    void setCurrentNode(YAML::Node node);

    bool msgParams(TopicHelper &th, std::string key, std::string topicDefault="", bool enabledDefault=true, int periodDefault=1, bool implicitEnable=false);
    bool msgParamsImplicit(TopicHelper &th, std::string key, std::string topicDefault, bool enabledDefault=true, int periodDefault=1);

//    template <typename Type>
//    bool param(YAML::Node &node, const std::string key, Type &val, Type &valDefault);
//
//    template <typename Derived1>
//    bool paramVec(YAML::Node &node, const std::string key, int size, Derived1 &val, Derived1 &valDefault);

    bool nodeParam(const std::string key, std::string &val, std::string valDefault=""){ return param(currentNode_, key, val, valDefault); }
    bool nodeParam(const std::string key, double &val, double valDefault=0.0){ return param(currentNode_, key, val, valDefault); }
    bool nodeParam(const std::string key, float &val, float valDefault=0.0f){ return param(currentNode_, key, val, valDefault); }
    bool nodeParam(const std::string key, bool &val, bool valDefault=false){ return param(currentNode_, key, val, valDefault); }
    bool nodeParam(const std::string key, int &val, int valDefault=0){ return param(currentNode_, key, val, valDefault); }
    bool nodeParam(const std::string key, uint32_t &val, uint32_t valDefault=0){ return param(currentNode_, key, val, valDefault); }

    bool nodeParam(YAML::Node &node, const std::string key, std::string &val, std::string valDefault=""){ return param(node, key, val, valDefault); }
    bool nodeParam(YAML::Node &node, const std::string key, double &val, double valDefault=0.0){ return param(node, key, val, valDefault); }
    bool nodeParam(YAML::Node &node, const std::string key, float &val, float valDefault=0.0f){ return param(node, key, val, valDefault); }
    bool nodeParam(YAML::Node &node, const std::string key, bool &val, bool valDefault=false){ return param(node, key, val, valDefault); }
    bool nodeParam(YAML::Node &node, const std::string key, int &val, int valDefault=0){ return param(node, key, val, valDefault); }
    bool nodeParam(YAML::Node &node, const std::string key, uint32_t &val, uint32_t valDefault=0){ return param(node, key, val, valDefault); }

    bool nodeParamVec(const std::string key, int size, double val[], double valDefault[] = NULL){ return paramVec(currentNode_, key, size, val, valDefault); }
    bool nodeParamVec(const std::string key, int size, float val[], float valDefault[] = NULL){ return paramVec(currentNode_, key, size, val, valDefault); }
    bool nodeParamVec(YAML::Node &node, const std::string key, int size, double val[], double valDefault[] = NULL){ return paramVec(node, key, size, val, valDefault); }
    bool nodeParamVec(YAML::Node &node, const std::string key, int size, float val[], float valDefault[] = NULL){ return paramVec(node, key, size, val, valDefault); }

    bool nodeParamEnum(const std::string key, int &val, std::vector<std::string> set, int defaultVal){ return paramEnum(currentNode_, key, val, set, defaultVal); }
    bool nodeParamEnum(YAML::Node &node, const std::string key, int &val, std::vector<std::string> set, int defaultVal){ return paramEnum(node, key, val, set, defaultVal); }
    bool nodeParamEnum(const std::string key, int &val, std::map<std::string, int> set, int defaultVal){ return paramEnum(currentNode_, key, val, set, defaultVal); }
    bool nodeParamEnum(YAML::Node &node, const std::string key, int &val, std::map<std::string, int> set, int defaultVal){ return paramEnum(node, key, val, set, defaultVal); }

    static bool paramServerToYamlNode(YAML::Node &node, std::string nhKey="", std::string indentStr="");

    template <typename Type>
    bool param(YAML::Node &node, const std::string key, Type &val, Type &valDefault)
    {
        bool success = false;

        if (node[key])
        {
            val = node[key].as<Type>();
            success = true;
        }

        if (!success)
        {   // Use default
            val = valDefault;
            node[key] = valDefault;
        }

        return success;
    }

    template <typename Derived1>
    bool paramVec(YAML::Node &node, const std::string key, int size, Derived1 &val, Derived1 &valDefault)
    {
        bool success = false;

        if (node[key])
        {
            std::vector<double> vec;
            vec = node[key].as<std::vector<double>>();
            for (int i = 0; i < size; i++)
            {
                val[i] = vec[i];
            }
            success = true;
        }

        if (!success)
        {
            std::vector<double> vec;
            for (int i = 0; i < size; i++)
            {
                val[i] = (valDefault==NULL ? 0.0 : valDefault[i]);
                vec.push_back(double(val[i]));
            }
            node[key] = vec;
        }

        return success;
    }

    bool paramEnum(YAML::Node &node, const std::string key, int& val, std::vector<std::string>& set, int defaultVal) {
        bool success = false;
        if (node[key]) {
            std::string str_val = node[key].as<std::string>();
            for (int i = 0 ; i < set.size(); i++) {
                if (str_val == set[i]) {
                    val = i;
                    success = true;
                    return true;
                }
            }
            // before we fall to a failure (not the 'return true' above)... check it str_val is a number, and if so, use it directly
            if (!str_val.empty() && std::find_if(str_val.begin(),str_val.end(), [](unsigned char c) { return !std::isdigit(c); }) == str_val.end()) {
                try {
                    val = std::stoi(str_val);
                    success = true;
                } catch (const std::invalid_argument& ia) {
                    // a parse error should fall back to the default, but return a failure
                    val = defaultVal;
                }
            }
        } else {
            // the key wasn't found, so also a failure.
            node[key] = defaultVal;
            val = defaultVal;
        }
        return success;
    }

    bool paramEnum(YAML::Node &node, const std::string key, int& val, std::map<std::string, int>& set, int defaultVal) {
        bool success = false;
        if (node[key]) {
            std::string str_val = node[key].as<std::string>();
            for (auto it = set.begin(); it != set.end(); it++) {
                if (it->first == str_val) {
                    val = it->second;
                    return true;
                }
            }
            // before we fall to a failure (not the 'return true' above)... check it str_val is a number, and if so, use it directly
            if (!str_val.empty() && std::find_if(str_val.begin(),str_val.end(), [](unsigned char c) { return !std::isdigit(c); }) == str_val.end()) {
                try {
                    val = std::stoi(str_val);
                    success = true;
                } catch (const std::invalid_argument& ia) {
                    // a parse error should fall back to the default, but return a failure
                    val = defaultVal;
                }
            }
        } else {
            node[key] = defaultVal;
            val = defaultVal;
        }
        return false;
    }

};

#endif //INERTIAL_SENSE_IMX_PARAMHELPER_H
