// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from inertial_sense_ros2:msg/GPSInfo.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "inertial_sense_ros2/msg/gps_info.hpp"


#ifndef INERTIAL_SENSE_ROS2__MSG__DETAIL__GPS_INFO__TRAITS_HPP_
#define INERTIAL_SENSE_ROS2__MSG__DETAIL__GPS_INFO__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "inertial_sense_ros2/msg/detail/gps_info__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'sattelite_info'
#include "inertial_sense_ros2/msg/detail/sat_info__traits.hpp"

namespace inertial_sense_ros2
{

namespace msg
{

inline void to_flow_style_yaml(
  const GPSInfo & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: num_sats
  {
    out << "num_sats: ";
    rosidl_generator_traits::value_to_yaml(msg.num_sats, out);
    out << ", ";
  }

  // member: sattelite_info
  {
    if (msg.sattelite_info.size() == 0) {
      out << "sattelite_info: []";
    } else {
      out << "sattelite_info: [";
      size_t pending_items = msg.sattelite_info.size();
      for (auto item : msg.sattelite_info) {
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
  const GPSInfo & msg,
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

  // member: num_sats
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "num_sats: ";
    rosidl_generator_traits::value_to_yaml(msg.num_sats, out);
    out << "\n";
  }

  // member: sattelite_info
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.sattelite_info.size() == 0) {
      out << "sattelite_info: []\n";
    } else {
      out << "sattelite_info:\n";
      for (auto item : msg.sattelite_info) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const GPSInfo & msg, bool use_flow_style = false)
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
  const inertial_sense_ros2::msg::GPSInfo & msg,
  std::ostream & out, size_t indentation = 0)
{
  inertial_sense_ros2::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use inertial_sense_ros2::msg::to_yaml() instead")]]
inline std::string to_yaml(const inertial_sense_ros2::msg::GPSInfo & msg)
{
  return inertial_sense_ros2::msg::to_yaml(msg);
}

template<>
inline const char * data_type<inertial_sense_ros2::msg::GPSInfo>()
{
  return "inertial_sense_ros2::msg::GPSInfo";
}

template<>
inline const char * name<inertial_sense_ros2::msg::GPSInfo>()
{
  return "inertial_sense_ros2/msg/GPSInfo";
}

template<>
struct has_fixed_size<inertial_sense_ros2::msg::GPSInfo>
  : std::integral_constant<bool, has_fixed_size<inertial_sense_ros2::msg::SatInfo>::value && has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<inertial_sense_ros2::msg::GPSInfo>
  : std::integral_constant<bool, has_bounded_size<inertial_sense_ros2::msg::SatInfo>::value && has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<inertial_sense_ros2::msg::GPSInfo>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // INERTIAL_SENSE_ROS2__MSG__DETAIL__GPS_INFO__TRAITS_HPP_
