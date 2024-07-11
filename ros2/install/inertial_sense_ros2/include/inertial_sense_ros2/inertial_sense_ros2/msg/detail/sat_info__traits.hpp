// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from inertial_sense_ros2:msg/SatInfo.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "inertial_sense_ros2/msg/sat_info.hpp"


#ifndef INERTIAL_SENSE_ROS2__MSG__DETAIL__SAT_INFO__TRAITS_HPP_
#define INERTIAL_SENSE_ROS2__MSG__DETAIL__SAT_INFO__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "inertial_sense_ros2/msg/detail/sat_info__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace inertial_sense_ros2
{

namespace msg
{

inline void to_flow_style_yaml(
  const SatInfo & msg,
  std::ostream & out)
{
  out << "{";
  // member: sat_id
  {
    out << "sat_id: ";
    rosidl_generator_traits::value_to_yaml(msg.sat_id, out);
    out << ", ";
  }

  // member: cno
  {
    out << "cno: ";
    rosidl_generator_traits::value_to_yaml(msg.cno, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const SatInfo & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: sat_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "sat_id: ";
    rosidl_generator_traits::value_to_yaml(msg.sat_id, out);
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
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const SatInfo & msg, bool use_flow_style = false)
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
  const inertial_sense_ros2::msg::SatInfo & msg,
  std::ostream & out, size_t indentation = 0)
{
  inertial_sense_ros2::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use inertial_sense_ros2::msg::to_yaml() instead")]]
inline std::string to_yaml(const inertial_sense_ros2::msg::SatInfo & msg)
{
  return inertial_sense_ros2::msg::to_yaml(msg);
}

template<>
inline const char * data_type<inertial_sense_ros2::msg::SatInfo>()
{
  return "inertial_sense_ros2::msg::SatInfo";
}

template<>
inline const char * name<inertial_sense_ros2::msg::SatInfo>()
{
  return "inertial_sense_ros2/msg/SatInfo";
}

template<>
struct has_fixed_size<inertial_sense_ros2::msg::SatInfo>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<inertial_sense_ros2::msg::SatInfo>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<inertial_sense_ros2::msg::SatInfo>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // INERTIAL_SENSE_ROS2__MSG__DETAIL__SAT_INFO__TRAITS_HPP_
