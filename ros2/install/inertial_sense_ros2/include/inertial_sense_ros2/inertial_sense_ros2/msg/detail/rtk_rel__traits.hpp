// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from inertial_sense_ros2:msg/RTKRel.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "inertial_sense_ros2/msg/rtk_rel.hpp"


#ifndef INERTIAL_SENSE_ROS2__MSG__DETAIL__RTK_REL__TRAITS_HPP_
#define INERTIAL_SENSE_ROS2__MSG__DETAIL__RTK_REL__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "inertial_sense_ros2/msg/detail/rtk_rel__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'vector_base_to_rover'
#include "geometry_msgs/msg/detail/vector3__traits.hpp"

namespace inertial_sense_ros2
{

namespace msg
{

inline void to_flow_style_yaml(
  const RTKRel & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: differential_age
  {
    out << "differential_age: ";
    rosidl_generator_traits::value_to_yaml(msg.differential_age, out);
    out << ", ";
  }

  // member: ar_ratio
  {
    out << "ar_ratio: ";
    rosidl_generator_traits::value_to_yaml(msg.ar_ratio, out);
    out << ", ";
  }

  // member: e_gps_status_fix
  {
    out << "e_gps_status_fix: ";
    rosidl_generator_traits::value_to_yaml(msg.e_gps_status_fix, out);
    out << ", ";
  }

  // member: vector_base_to_rover
  {
    out << "vector_base_to_rover: ";
    to_flow_style_yaml(msg.vector_base_to_rover, out);
    out << ", ";
  }

  // member: distance_base_to_rover
  {
    out << "distance_base_to_rover: ";
    rosidl_generator_traits::value_to_yaml(msg.distance_base_to_rover, out);
    out << ", ";
  }

  // member: heading_base_to_rover
  {
    out << "heading_base_to_rover: ";
    rosidl_generator_traits::value_to_yaml(msg.heading_base_to_rover, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const RTKRel & msg,
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

  // member: differential_age
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "differential_age: ";
    rosidl_generator_traits::value_to_yaml(msg.differential_age, out);
    out << "\n";
  }

  // member: ar_ratio
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "ar_ratio: ";
    rosidl_generator_traits::value_to_yaml(msg.ar_ratio, out);
    out << "\n";
  }

  // member: e_gps_status_fix
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "e_gps_status_fix: ";
    rosidl_generator_traits::value_to_yaml(msg.e_gps_status_fix, out);
    out << "\n";
  }

  // member: vector_base_to_rover
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "vector_base_to_rover:\n";
    to_block_style_yaml(msg.vector_base_to_rover, out, indentation + 2);
  }

  // member: distance_base_to_rover
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "distance_base_to_rover: ";
    rosidl_generator_traits::value_to_yaml(msg.distance_base_to_rover, out);
    out << "\n";
  }

  // member: heading_base_to_rover
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "heading_base_to_rover: ";
    rosidl_generator_traits::value_to_yaml(msg.heading_base_to_rover, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const RTKRel & msg, bool use_flow_style = false)
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
  const inertial_sense_ros2::msg::RTKRel & msg,
  std::ostream & out, size_t indentation = 0)
{
  inertial_sense_ros2::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use inertial_sense_ros2::msg::to_yaml() instead")]]
inline std::string to_yaml(const inertial_sense_ros2::msg::RTKRel & msg)
{
  return inertial_sense_ros2::msg::to_yaml(msg);
}

template<>
inline const char * data_type<inertial_sense_ros2::msg::RTKRel>()
{
  return "inertial_sense_ros2::msg::RTKRel";
}

template<>
inline const char * name<inertial_sense_ros2::msg::RTKRel>()
{
  return "inertial_sense_ros2/msg/RTKRel";
}

template<>
struct has_fixed_size<inertial_sense_ros2::msg::RTKRel>
  : std::integral_constant<bool, has_fixed_size<geometry_msgs::msg::Vector3>::value && has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<inertial_sense_ros2::msg::RTKRel>
  : std::integral_constant<bool, has_bounded_size<geometry_msgs::msg::Vector3>::value && has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<inertial_sense_ros2::msg::RTKRel>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // INERTIAL_SENSE_ROS2__MSG__DETAIL__RTK_REL__TRAITS_HPP_
