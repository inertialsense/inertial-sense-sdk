// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from inertial_sense_ros2:msg/INL2States.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "inertial_sense_ros2/msg/inl2_states.hpp"


#ifndef INERTIAL_SENSE_ROS2__MSG__DETAIL__INL2_STATES__TRAITS_HPP_
#define INERTIAL_SENSE_ROS2__MSG__DETAIL__INL2_STATES__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "inertial_sense_ros2/msg/detail/inl2_states__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'quat_ecef'
#include "geometry_msgs/msg/detail/quaternion__traits.hpp"
// Member 'vel_ecef'
// Member 'pos_ecef'
// Member 'gyro_bias'
// Member 'accel_bias'
#include "geometry_msgs/msg/detail/vector3__traits.hpp"

namespace inertial_sense_ros2
{

namespace msg
{

inline void to_flow_style_yaml(
  const INL2States & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: quat_ecef
  {
    out << "quat_ecef: ";
    to_flow_style_yaml(msg.quat_ecef, out);
    out << ", ";
  }

  // member: vel_ecef
  {
    out << "vel_ecef: ";
    to_flow_style_yaml(msg.vel_ecef, out);
    out << ", ";
  }

  // member: pos_ecef
  {
    out << "pos_ecef: ";
    to_flow_style_yaml(msg.pos_ecef, out);
    out << ", ";
  }

  // member: gyro_bias
  {
    out << "gyro_bias: ";
    to_flow_style_yaml(msg.gyro_bias, out);
    out << ", ";
  }

  // member: accel_bias
  {
    out << "accel_bias: ";
    to_flow_style_yaml(msg.accel_bias, out);
    out << ", ";
  }

  // member: baro_bias
  {
    out << "baro_bias: ";
    rosidl_generator_traits::value_to_yaml(msg.baro_bias, out);
    out << ", ";
  }

  // member: mag_dec
  {
    out << "mag_dec: ";
    rosidl_generator_traits::value_to_yaml(msg.mag_dec, out);
    out << ", ";
  }

  // member: mag_inc
  {
    out << "mag_inc: ";
    rosidl_generator_traits::value_to_yaml(msg.mag_inc, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const INL2States & msg,
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

  // member: quat_ecef
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "quat_ecef:\n";
    to_block_style_yaml(msg.quat_ecef, out, indentation + 2);
  }

  // member: vel_ecef
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "vel_ecef:\n";
    to_block_style_yaml(msg.vel_ecef, out, indentation + 2);
  }

  // member: pos_ecef
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pos_ecef:\n";
    to_block_style_yaml(msg.pos_ecef, out, indentation + 2);
  }

  // member: gyro_bias
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "gyro_bias:\n";
    to_block_style_yaml(msg.gyro_bias, out, indentation + 2);
  }

  // member: accel_bias
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "accel_bias:\n";
    to_block_style_yaml(msg.accel_bias, out, indentation + 2);
  }

  // member: baro_bias
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "baro_bias: ";
    rosidl_generator_traits::value_to_yaml(msg.baro_bias, out);
    out << "\n";
  }

  // member: mag_dec
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "mag_dec: ";
    rosidl_generator_traits::value_to_yaml(msg.mag_dec, out);
    out << "\n";
  }

  // member: mag_inc
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "mag_inc: ";
    rosidl_generator_traits::value_to_yaml(msg.mag_inc, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const INL2States & msg, bool use_flow_style = false)
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
  const inertial_sense_ros2::msg::INL2States & msg,
  std::ostream & out, size_t indentation = 0)
{
  inertial_sense_ros2::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use inertial_sense_ros2::msg::to_yaml() instead")]]
inline std::string to_yaml(const inertial_sense_ros2::msg::INL2States & msg)
{
  return inertial_sense_ros2::msg::to_yaml(msg);
}

template<>
inline const char * data_type<inertial_sense_ros2::msg::INL2States>()
{
  return "inertial_sense_ros2::msg::INL2States";
}

template<>
inline const char * name<inertial_sense_ros2::msg::INL2States>()
{
  return "inertial_sense_ros2/msg/INL2States";
}

template<>
struct has_fixed_size<inertial_sense_ros2::msg::INL2States>
  : std::integral_constant<bool, has_fixed_size<geometry_msgs::msg::Quaternion>::value && has_fixed_size<geometry_msgs::msg::Vector3>::value && has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<inertial_sense_ros2::msg::INL2States>
  : std::integral_constant<bool, has_bounded_size<geometry_msgs::msg::Quaternion>::value && has_bounded_size<geometry_msgs::msg::Vector3>::value && has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<inertial_sense_ros2::msg::INL2States>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // INERTIAL_SENSE_ROS2__MSG__DETAIL__INL2_STATES__TRAITS_HPP_
