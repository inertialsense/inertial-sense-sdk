// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from inertial_sense_ros2:msg/GNSSObservation.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "inertial_sense_ros2/msg/gnss_observation.hpp"


#ifndef INERTIAL_SENSE_ROS2__MSG__DETAIL__GNSS_OBSERVATION__TRAITS_HPP_
#define INERTIAL_SENSE_ROS2__MSG__DETAIL__GNSS_OBSERVATION__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "inertial_sense_ros2/msg/detail/gnss_observation__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'time'
#include "inertial_sense_ros2/msg/detail/g_time__traits.hpp"

namespace inertial_sense_ros2
{

namespace msg
{

inline void to_flow_style_yaml(
  const GNSSObservation & msg,
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

  // member: sat
  {
    out << "sat: ";
    rosidl_generator_traits::value_to_yaml(msg.sat, out);
    out << ", ";
  }

  // member: rcv
  {
    out << "rcv: ";
    rosidl_generator_traits::value_to_yaml(msg.rcv, out);
    out << ", ";
  }

  // member: snrr
  {
    out << "snrr: ";
    rosidl_generator_traits::value_to_yaml(msg.snrr, out);
    out << ", ";
  }

  // member: lli
  {
    out << "lli: ";
    rosidl_generator_traits::value_to_yaml(msg.lli, out);
    out << ", ";
  }

  // member: code
  {
    out << "code: ";
    rosidl_generator_traits::value_to_yaml(msg.code, out);
    out << ", ";
  }

  // member: qual_l
  {
    out << "qual_l: ";
    rosidl_generator_traits::value_to_yaml(msg.qual_l, out);
    out << ", ";
  }

  // member: qual_p
  {
    out << "qual_p: ";
    rosidl_generator_traits::value_to_yaml(msg.qual_p, out);
    out << ", ";
  }

  // member: l
  {
    out << "l: ";
    rosidl_generator_traits::value_to_yaml(msg.l, out);
    out << ", ";
  }

  // member: p
  {
    out << "p: ";
    rosidl_generator_traits::value_to_yaml(msg.p, out);
    out << ", ";
  }

  // member: d
  {
    out << "d: ";
    rosidl_generator_traits::value_to_yaml(msg.d, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const GNSSObservation & msg,
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

  // member: sat
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "sat: ";
    rosidl_generator_traits::value_to_yaml(msg.sat, out);
    out << "\n";
  }

  // member: rcv
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "rcv: ";
    rosidl_generator_traits::value_to_yaml(msg.rcv, out);
    out << "\n";
  }

  // member: snrr
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "snrr: ";
    rosidl_generator_traits::value_to_yaml(msg.snrr, out);
    out << "\n";
  }

  // member: lli
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "lli: ";
    rosidl_generator_traits::value_to_yaml(msg.lli, out);
    out << "\n";
  }

  // member: code
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "code: ";
    rosidl_generator_traits::value_to_yaml(msg.code, out);
    out << "\n";
  }

  // member: qual_l
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "qual_l: ";
    rosidl_generator_traits::value_to_yaml(msg.qual_l, out);
    out << "\n";
  }

  // member: qual_p
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "qual_p: ";
    rosidl_generator_traits::value_to_yaml(msg.qual_p, out);
    out << "\n";
  }

  // member: l
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "l: ";
    rosidl_generator_traits::value_to_yaml(msg.l, out);
    out << "\n";
  }

  // member: p
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "p: ";
    rosidl_generator_traits::value_to_yaml(msg.p, out);
    out << "\n";
  }

  // member: d
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "d: ";
    rosidl_generator_traits::value_to_yaml(msg.d, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const GNSSObservation & msg, bool use_flow_style = false)
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
  const inertial_sense_ros2::msg::GNSSObservation & msg,
  std::ostream & out, size_t indentation = 0)
{
  inertial_sense_ros2::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use inertial_sense_ros2::msg::to_yaml() instead")]]
inline std::string to_yaml(const inertial_sense_ros2::msg::GNSSObservation & msg)
{
  return inertial_sense_ros2::msg::to_yaml(msg);
}

template<>
inline const char * data_type<inertial_sense_ros2::msg::GNSSObservation>()
{
  return "inertial_sense_ros2::msg::GNSSObservation";
}

template<>
inline const char * name<inertial_sense_ros2::msg::GNSSObservation>()
{
  return "inertial_sense_ros2/msg/GNSSObservation";
}

template<>
struct has_fixed_size<inertial_sense_ros2::msg::GNSSObservation>
  : std::integral_constant<bool, has_fixed_size<inertial_sense_ros2::msg::GTime>::value && has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<inertial_sense_ros2::msg::GNSSObservation>
  : std::integral_constant<bool, has_bounded_size<inertial_sense_ros2::msg::GTime>::value && has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<inertial_sense_ros2::msg::GNSSObservation>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // INERTIAL_SENSE_ROS2__MSG__DETAIL__GNSS_OBSERVATION__TRAITS_HPP_
