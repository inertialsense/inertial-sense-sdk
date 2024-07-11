// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from inertial_sense_ros2:msg/RTKInfo.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "inertial_sense_ros2/msg/rtk_info.hpp"


#ifndef INERTIAL_SENSE_ROS2__MSG__DETAIL__RTK_INFO__TRAITS_HPP_
#define INERTIAL_SENSE_ROS2__MSG__DETAIL__RTK_INFO__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "inertial_sense_ros2/msg/detail/rtk_info__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace inertial_sense_ros2
{

namespace msg
{

inline void to_flow_style_yaml(
  const RTKInfo & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: base_lla
  {
    if (msg.base_lla.size() == 0) {
      out << "base_lla: []";
    } else {
      out << "base_lla: [";
      size_t pending_items = msg.base_lla.size();
      for (auto item : msg.base_lla) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: cycle_slip_count
  {
    out << "cycle_slip_count: ";
    rosidl_generator_traits::value_to_yaml(msg.cycle_slip_count, out);
    out << ", ";
  }

  // member: rover_obs
  {
    out << "rover_obs: ";
    rosidl_generator_traits::value_to_yaml(msg.rover_obs, out);
    out << ", ";
  }

  // member: base_obs
  {
    out << "base_obs: ";
    rosidl_generator_traits::value_to_yaml(msg.base_obs, out);
    out << ", ";
  }

  // member: rover_eph
  {
    out << "rover_eph: ";
    rosidl_generator_traits::value_to_yaml(msg.rover_eph, out);
    out << ", ";
  }

  // member: base_eph
  {
    out << "base_eph: ";
    rosidl_generator_traits::value_to_yaml(msg.base_eph, out);
    out << ", ";
  }

  // member: base_ant_count
  {
    out << "base_ant_count: ";
    rosidl_generator_traits::value_to_yaml(msg.base_ant_count, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const RTKInfo & msg,
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

  // member: base_lla
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.base_lla.size() == 0) {
      out << "base_lla: []\n";
    } else {
      out << "base_lla:\n";
      for (auto item : msg.base_lla) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: cycle_slip_count
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "cycle_slip_count: ";
    rosidl_generator_traits::value_to_yaml(msg.cycle_slip_count, out);
    out << "\n";
  }

  // member: rover_obs
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "rover_obs: ";
    rosidl_generator_traits::value_to_yaml(msg.rover_obs, out);
    out << "\n";
  }

  // member: base_obs
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "base_obs: ";
    rosidl_generator_traits::value_to_yaml(msg.base_obs, out);
    out << "\n";
  }

  // member: rover_eph
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "rover_eph: ";
    rosidl_generator_traits::value_to_yaml(msg.rover_eph, out);
    out << "\n";
  }

  // member: base_eph
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "base_eph: ";
    rosidl_generator_traits::value_to_yaml(msg.base_eph, out);
    out << "\n";
  }

  // member: base_ant_count
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "base_ant_count: ";
    rosidl_generator_traits::value_to_yaml(msg.base_ant_count, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const RTKInfo & msg, bool use_flow_style = false)
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
  const inertial_sense_ros2::msg::RTKInfo & msg,
  std::ostream & out, size_t indentation = 0)
{
  inertial_sense_ros2::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use inertial_sense_ros2::msg::to_yaml() instead")]]
inline std::string to_yaml(const inertial_sense_ros2::msg::RTKInfo & msg)
{
  return inertial_sense_ros2::msg::to_yaml(msg);
}

template<>
inline const char * data_type<inertial_sense_ros2::msg::RTKInfo>()
{
  return "inertial_sense_ros2::msg::RTKInfo";
}

template<>
inline const char * name<inertial_sense_ros2::msg::RTKInfo>()
{
  return "inertial_sense_ros2/msg/RTKInfo";
}

template<>
struct has_fixed_size<inertial_sense_ros2::msg::RTKInfo>
  : std::integral_constant<bool, has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<inertial_sense_ros2::msg::RTKInfo>
  : std::integral_constant<bool, has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<inertial_sense_ros2::msg::RTKInfo>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // INERTIAL_SENSE_ROS2__MSG__DETAIL__RTK_INFO__TRAITS_HPP_
