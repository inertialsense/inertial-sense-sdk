// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from inertial_sense_ros2:msg/GNSSObsVec.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "inertial_sense_ros2/msg/gnss_obs_vec.hpp"


#ifndef INERTIAL_SENSE_ROS2__MSG__DETAIL__GNSS_OBS_VEC__TRAITS_HPP_
#define INERTIAL_SENSE_ROS2__MSG__DETAIL__GNSS_OBS_VEC__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "inertial_sense_ros2/msg/detail/gnss_obs_vec__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'time'
#include "inertial_sense_ros2/msg/detail/g_time__traits.hpp"
// Member 'obs'
#include "inertial_sense_ros2/msg/detail/gnss_observation__traits.hpp"

namespace inertial_sense_ros2
{

namespace msg
{

inline void to_flow_style_yaml(
  const GNSSObsVec & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: time
  {
    out << "time: ";
    to_flow_style_yaml(msg.time, out);
    out << ", ";
  }

  // member: obs
  {
    if (msg.obs.size() == 0) {
      out << "obs: []";
    } else {
      out << "obs: [";
      size_t pending_items = msg.obs.size();
      for (auto item : msg.obs) {
        to_flow_style_yaml(item, out);
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
  const GNSSObsVec & msg,
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

  // member: time
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "time:\n";
    to_block_style_yaml(msg.time, out, indentation + 2);
  }

  // member: obs
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.obs.size() == 0) {
      out << "obs: []\n";
    } else {
      out << "obs:\n";
      for (auto item : msg.obs) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const GNSSObsVec & msg, bool use_flow_style = false)
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
  const inertial_sense_ros2::msg::GNSSObsVec & msg,
  std::ostream & out, size_t indentation = 0)
{
  inertial_sense_ros2::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use inertial_sense_ros2::msg::to_yaml() instead")]]
inline std::string to_yaml(const inertial_sense_ros2::msg::GNSSObsVec & msg)
{
  return inertial_sense_ros2::msg::to_yaml(msg);
}

template<>
inline const char * data_type<inertial_sense_ros2::msg::GNSSObsVec>()
{
  return "inertial_sense_ros2::msg::GNSSObsVec";
}

template<>
inline const char * name<inertial_sense_ros2::msg::GNSSObsVec>()
{
  return "inertial_sense_ros2/msg/GNSSObsVec";
}

template<>
struct has_fixed_size<inertial_sense_ros2::msg::GNSSObsVec>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<inertial_sense_ros2::msg::GNSSObsVec>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<inertial_sense_ros2::msg::GNSSObsVec>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // INERTIAL_SENSE_ROS2__MSG__DETAIL__GNSS_OBS_VEC__TRAITS_HPP_
