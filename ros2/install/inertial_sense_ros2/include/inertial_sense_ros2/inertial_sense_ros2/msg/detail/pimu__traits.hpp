// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from inertial_sense_ros2:msg/PIMU.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "inertial_sense_ros2/msg/pimu.hpp"


#ifndef INERTIAL_SENSE_ROS2__MSG__DETAIL__PIMU__TRAITS_HPP_
#define INERTIAL_SENSE_ROS2__MSG__DETAIL__PIMU__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "inertial_sense_ros2/msg/detail/pimu__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'dtheta'
// Member 'dvel'
#include "geometry_msgs/msg/detail/vector3__traits.hpp"

namespace inertial_sense_ros2
{

namespace msg
{

inline void to_flow_style_yaml(
  const PIMU & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: dtheta
  {
    out << "dtheta: ";
    to_flow_style_yaml(msg.dtheta, out);
    out << ", ";
  }

  // member: dvel
  {
    out << "dvel: ";
    to_flow_style_yaml(msg.dvel, out);
    out << ", ";
  }

  // member: dt
  {
    out << "dt: ";
    rosidl_generator_traits::value_to_yaml(msg.dt, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const PIMU & msg,
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

  // member: dtheta
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "dtheta:\n";
    to_block_style_yaml(msg.dtheta, out, indentation + 2);
  }

  // member: dvel
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "dvel:\n";
    to_block_style_yaml(msg.dvel, out, indentation + 2);
  }

  // member: dt
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "dt: ";
    rosidl_generator_traits::value_to_yaml(msg.dt, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const PIMU & msg, bool use_flow_style = false)
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
  const inertial_sense_ros2::msg::PIMU & msg,
  std::ostream & out, size_t indentation = 0)
{
  inertial_sense_ros2::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use inertial_sense_ros2::msg::to_yaml() instead")]]
inline std::string to_yaml(const inertial_sense_ros2::msg::PIMU & msg)
{
  return inertial_sense_ros2::msg::to_yaml(msg);
}

template<>
inline const char * data_type<inertial_sense_ros2::msg::PIMU>()
{
  return "inertial_sense_ros2::msg::PIMU";
}

template<>
inline const char * name<inertial_sense_ros2::msg::PIMU>()
{
  return "inertial_sense_ros2/msg/PIMU";
}

template<>
struct has_fixed_size<inertial_sense_ros2::msg::PIMU>
  : std::integral_constant<bool, has_fixed_size<geometry_msgs::msg::Vector3>::value && has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<inertial_sense_ros2::msg::PIMU>
  : std::integral_constant<bool, has_bounded_size<geometry_msgs::msg::Vector3>::value && has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<inertial_sense_ros2::msg::PIMU>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // INERTIAL_SENSE_ROS2__MSG__DETAIL__PIMU__TRAITS_HPP_
