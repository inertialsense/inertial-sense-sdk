// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from inertial_sense_ros2:msg/DIDINS4.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "inertial_sense_ros2/msg/didins4.hpp"


#ifndef INERTIAL_SENSE_ROS2__MSG__DETAIL__DIDINS4__TRAITS_HPP_
#define INERTIAL_SENSE_ROS2__MSG__DETAIL__DIDINS4__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "inertial_sense_ros2/msg/detail/didins4__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace inertial_sense_ros2
{

namespace msg
{

inline void to_flow_style_yaml(
  const DIDINS4 & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: week
  {
    out << "week: ";
    rosidl_generator_traits::value_to_yaml(msg.week, out);
    out << ", ";
  }

  // member: time_of_week
  {
    out << "time_of_week: ";
    rosidl_generator_traits::value_to_yaml(msg.time_of_week, out);
    out << ", ";
  }

  // member: ins_status
  {
    out << "ins_status: ";
    rosidl_generator_traits::value_to_yaml(msg.ins_status, out);
    out << ", ";
  }

  // member: hdw_status
  {
    out << "hdw_status: ";
    rosidl_generator_traits::value_to_yaml(msg.hdw_status, out);
    out << ", ";
  }

  // member: qe2b
  {
    if (msg.qe2b.size() == 0) {
      out << "qe2b: []";
    } else {
      out << "qe2b: [";
      size_t pending_items = msg.qe2b.size();
      for (auto item : msg.qe2b) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: ve
  {
    if (msg.ve.size() == 0) {
      out << "ve: []";
    } else {
      out << "ve: [";
      size_t pending_items = msg.ve.size();
      for (auto item : msg.ve) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: ecef
  {
    if (msg.ecef.size() == 0) {
      out << "ecef: []";
    } else {
      out << "ecef: [";
      size_t pending_items = msg.ecef.size();
      for (auto item : msg.ecef) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const DIDINS4 & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: header
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "header:\n";
    to_block_style_yaml(msg.header, out, indentation + 2);
  }

  // member: week
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "week: ";
    rosidl_generator_traits::value_to_yaml(msg.week, out);
    out << "\n";
  }

  // member: time_of_week
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "time_of_week: ";
    rosidl_generator_traits::value_to_yaml(msg.time_of_week, out);
    out << "\n";
  }

  // member: ins_status
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "ins_status: ";
    rosidl_generator_traits::value_to_yaml(msg.ins_status, out);
    out << "\n";
  }

  // member: hdw_status
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "hdw_status: ";
    rosidl_generator_traits::value_to_yaml(msg.hdw_status, out);
    out << "\n";
  }

  // member: qe2b
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.qe2b.size() == 0) {
      out << "qe2b: []\n";
    } else {
      out << "qe2b:\n";
      for (auto item : msg.qe2b) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: ve
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.ve.size() == 0) {
      out << "ve: []\n";
    } else {
      out << "ve:\n";
      for (auto item : msg.ve) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: ecef
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.ecef.size() == 0) {
      out << "ecef: []\n";
    } else {
      out << "ecef:\n";
      for (auto item : msg.ecef) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const DIDINS4 & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace inertial_sense_ros2

namespace rosidl_generator_traits
{

[[deprecated("use inertial_sense_ros2::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const inertial_sense_ros2::msg::DIDINS4 & msg,
  std::ostream & out, size_t indentation = 0)
{
  inertial_sense_ros2::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use inertial_sense_ros2::msg::to_yaml() instead")]]
inline std::string to_yaml(const inertial_sense_ros2::msg::DIDINS4 & msg)
{
  return inertial_sense_ros2::msg::to_yaml(msg);
}

template<>
inline const char * data_type<inertial_sense_ros2::msg::DIDINS4>()
{
  return "inertial_sense_ros2::msg::DIDINS4";
}

template<>
inline const char * name<inertial_sense_ros2::msg::DIDINS4>()
{
  return "inertial_sense_ros2/msg/DIDINS4";
}

template<>
struct has_fixed_size<inertial_sense_ros2::msg::DIDINS4>
  : std::integral_constant<bool, has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<inertial_sense_ros2::msg::DIDINS4>
  : std::integral_constant<bool, has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<inertial_sense_ros2::msg::DIDINS4>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // INERTIAL_SENSE_ROS2__MSG__DETAIL__DIDINS4__TRAITS_HPP_
