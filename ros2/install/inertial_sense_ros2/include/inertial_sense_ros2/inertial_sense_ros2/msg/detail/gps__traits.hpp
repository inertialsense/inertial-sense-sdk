// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from inertial_sense_ros2:msg/GPS.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "inertial_sense_ros2/msg/gps.hpp"


#ifndef INERTIAL_SENSE_ROS2__MSG__DETAIL__GPS__TRAITS_HPP_
#define INERTIAL_SENSE_ROS2__MSG__DETAIL__GPS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "inertial_sense_ros2/msg/detail/gps__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'pos_ecef'
// Member 'vel_ecef'
#include "geometry_msgs/msg/detail/vector3__traits.hpp"

namespace inertial_sense_ros2
{

namespace msg
{

inline void to_flow_style_yaml(
  const GPS & msg,
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

  // member: num_sat
  {
    out << "num_sat: ";
    rosidl_generator_traits::value_to_yaml(msg.num_sat, out);
    out << ", ";
  }

  // member: status
  {
    out << "status: ";
    rosidl_generator_traits::value_to_yaml(msg.status, out);
    out << ", ";
  }

  // member: cno
  {
    out << "cno: ";
    rosidl_generator_traits::value_to_yaml(msg.cno, out);
    out << ", ";
  }

  // member: latitude
  {
    out << "latitude: ";
    rosidl_generator_traits::value_to_yaml(msg.latitude, out);
    out << ", ";
  }

  // member: longitude
  {
    out << "longitude: ";
    rosidl_generator_traits::value_to_yaml(msg.longitude, out);
    out << ", ";
  }

  // member: altitude
  {
    out << "altitude: ";
    rosidl_generator_traits::value_to_yaml(msg.altitude, out);
    out << ", ";
  }

  // member: pos_ecef
  {
    out << "pos_ecef: ";
    to_flow_style_yaml(msg.pos_ecef, out);
    out << ", ";
  }

  // member: vel_ecef
  {
    out << "vel_ecef: ";
    to_flow_style_yaml(msg.vel_ecef, out);
    out << ", ";
  }

  // member: hmsl
  {
    out << "hmsl: ";
    rosidl_generator_traits::value_to_yaml(msg.hmsl, out);
    out << ", ";
  }

  // member: hacc
  {
    out << "hacc: ";
    rosidl_generator_traits::value_to_yaml(msg.hacc, out);
    out << ", ";
  }

  // member: vacc
  {
    out << "vacc: ";
    rosidl_generator_traits::value_to_yaml(msg.vacc, out);
    out << ", ";
  }

  // member: sacc
  {
    out << "sacc: ";
    rosidl_generator_traits::value_to_yaml(msg.sacc, out);
    out << ", ";
  }

  // member: pdop
  {
    out << "pdop: ";
    rosidl_generator_traits::value_to_yaml(msg.pdop, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const GPS & msg,
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

  // member: num_sat
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "num_sat: ";
    rosidl_generator_traits::value_to_yaml(msg.num_sat, out);
    out << "\n";
  }

  // member: status
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "status: ";
    rosidl_generator_traits::value_to_yaml(msg.status, out);
    out << "\n";
  }

  // member: cno
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "cno: ";
    rosidl_generator_traits::value_to_yaml(msg.cno, out);
    out << "\n";
  }

  // member: latitude
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "latitude: ";
    rosidl_generator_traits::value_to_yaml(msg.latitude, out);
    out << "\n";
  }

  // member: longitude
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "longitude: ";
    rosidl_generator_traits::value_to_yaml(msg.longitude, out);
    out << "\n";
  }

  // member: altitude
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "altitude: ";
    rosidl_generator_traits::value_to_yaml(msg.altitude, out);
    out << "\n";
  }

  // member: pos_ecef
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pos_ecef:\n";
    to_block_style_yaml(msg.pos_ecef, out, indentation + 2);
  }

  // member: vel_ecef
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "vel_ecef:\n";
    to_block_style_yaml(msg.vel_ecef, out, indentation + 2);
  }

  // member: hmsl
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "hmsl: ";
    rosidl_generator_traits::value_to_yaml(msg.hmsl, out);
    out << "\n";
  }

  // member: hacc
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "hacc: ";
    rosidl_generator_traits::value_to_yaml(msg.hacc, out);
    out << "\n";
  }

  // member: vacc
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "vacc: ";
    rosidl_generator_traits::value_to_yaml(msg.vacc, out);
    out << "\n";
  }

  // member: sacc
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "sacc: ";
    rosidl_generator_traits::value_to_yaml(msg.sacc, out);
    out << "\n";
  }

  // member: pdop
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pdop: ";
    rosidl_generator_traits::value_to_yaml(msg.pdop, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const GPS & msg, bool use_flow_style = false)
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
  const inertial_sense_ros2::msg::GPS & msg,
  std::ostream & out, size_t indentation = 0)
{
  inertial_sense_ros2::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use inertial_sense_ros2::msg::to_yaml() instead")]]
inline std::string to_yaml(const inertial_sense_ros2::msg::GPS & msg)
{
  return inertial_sense_ros2::msg::to_yaml(msg);
}

template<>
inline const char * data_type<inertial_sense_ros2::msg::GPS>()
{
  return "inertial_sense_ros2::msg::GPS";
}

template<>
inline const char * name<inertial_sense_ros2::msg::GPS>()
{
  return "inertial_sense_ros2/msg/GPS";
}

template<>
struct has_fixed_size<inertial_sense_ros2::msg::GPS>
  : std::integral_constant<bool, has_fixed_size<geometry_msgs::msg::Vector3>::value && has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<inertial_sense_ros2::msg::GPS>
  : std::integral_constant<bool, has_bounded_size<geometry_msgs::msg::Vector3>::value && has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<inertial_sense_ros2::msg::GPS>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // INERTIAL_SENSE_ROS2__MSG__DETAIL__GPS__TRAITS_HPP_
