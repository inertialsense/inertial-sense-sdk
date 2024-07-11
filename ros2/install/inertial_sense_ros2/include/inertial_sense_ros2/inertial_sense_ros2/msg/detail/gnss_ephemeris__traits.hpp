// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from inertial_sense_ros2:msg/GNSSEphemeris.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "inertial_sense_ros2/msg/gnss_ephemeris.hpp"


#ifndef INERTIAL_SENSE_ROS2__MSG__DETAIL__GNSS_EPHEMERIS__TRAITS_HPP_
#define INERTIAL_SENSE_ROS2__MSG__DETAIL__GNSS_EPHEMERIS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "inertial_sense_ros2/msg/detail/gnss_ephemeris__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'toe'
// Member 'toc'
// Member 'ttr'
#include "inertial_sense_ros2/msg/detail/g_time__traits.hpp"

namespace inertial_sense_ros2
{

namespace msg
{

inline void to_flow_style_yaml(
  const GNSSEphemeris & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: sat
  {
    out << "sat: ";
    rosidl_generator_traits::value_to_yaml(msg.sat, out);
    out << ", ";
  }

  // member: iode
  {
    out << "iode: ";
    rosidl_generator_traits::value_to_yaml(msg.iode, out);
    out << ", ";
  }

  // member: iodc
  {
    out << "iodc: ";
    rosidl_generator_traits::value_to_yaml(msg.iodc, out);
    out << ", ";
  }

  // member: sva
  {
    out << "sva: ";
    rosidl_generator_traits::value_to_yaml(msg.sva, out);
    out << ", ";
  }

  // member: svh
  {
    out << "svh: ";
    rosidl_generator_traits::value_to_yaml(msg.svh, out);
    out << ", ";
  }

  // member: week
  {
    out << "week: ";
    rosidl_generator_traits::value_to_yaml(msg.week, out);
    out << ", ";
  }

  // member: code
  {
    out << "code: ";
    rosidl_generator_traits::value_to_yaml(msg.code, out);
    out << ", ";
  }

  // member: flag
  {
    out << "flag: ";
    rosidl_generator_traits::value_to_yaml(msg.flag, out);
    out << ", ";
  }

  // member: toe
  {
    out << "toe: ";
    to_flow_style_yaml(msg.toe, out);
    out << ", ";
  }

  // member: toc
  {
    out << "toc: ";
    to_flow_style_yaml(msg.toc, out);
    out << ", ";
  }

  // member: ttr
  {
    out << "ttr: ";
    to_flow_style_yaml(msg.ttr, out);
    out << ", ";
  }

  // member: a
  {
    out << "a: ";
    rosidl_generator_traits::value_to_yaml(msg.a, out);
    out << ", ";
  }

  // member: e
  {
    out << "e: ";
    rosidl_generator_traits::value_to_yaml(msg.e, out);
    out << ", ";
  }

  // member: i0
  {
    out << "i0: ";
    rosidl_generator_traits::value_to_yaml(msg.i0, out);
    out << ", ";
  }

  // member: omg_0
  {
    out << "omg_0: ";
    rosidl_generator_traits::value_to_yaml(msg.omg_0, out);
    out << ", ";
  }

  // member: omg
  {
    out << "omg: ";
    rosidl_generator_traits::value_to_yaml(msg.omg, out);
    out << ", ";
  }

  // member: m_0
  {
    out << "m_0: ";
    rosidl_generator_traits::value_to_yaml(msg.m_0, out);
    out << ", ";
  }

  // member: deln
  {
    out << "deln: ";
    rosidl_generator_traits::value_to_yaml(msg.deln, out);
    out << ", ";
  }

  // member: omg_d
  {
    out << "omg_d: ";
    rosidl_generator_traits::value_to_yaml(msg.omg_d, out);
    out << ", ";
  }

  // member: idot
  {
    out << "idot: ";
    rosidl_generator_traits::value_to_yaml(msg.idot, out);
    out << ", ";
  }

  // member: crc
  {
    out << "crc: ";
    rosidl_generator_traits::value_to_yaml(msg.crc, out);
    out << ", ";
  }

  // member: crs
  {
    out << "crs: ";
    rosidl_generator_traits::value_to_yaml(msg.crs, out);
    out << ", ";
  }

  // member: cuc
  {
    out << "cuc: ";
    rosidl_generator_traits::value_to_yaml(msg.cuc, out);
    out << ", ";
  }

  // member: cus
  {
    out << "cus: ";
    rosidl_generator_traits::value_to_yaml(msg.cus, out);
    out << ", ";
  }

  // member: cic
  {
    out << "cic: ";
    rosidl_generator_traits::value_to_yaml(msg.cic, out);
    out << ", ";
  }

  // member: cis
  {
    out << "cis: ";
    rosidl_generator_traits::value_to_yaml(msg.cis, out);
    out << ", ";
  }

  // member: toes
  {
    out << "toes: ";
    rosidl_generator_traits::value_to_yaml(msg.toes, out);
    out << ", ";
  }

  // member: fit
  {
    out << "fit: ";
    rosidl_generator_traits::value_to_yaml(msg.fit, out);
    out << ", ";
  }

  // member: f0
  {
    out << "f0: ";
    rosidl_generator_traits::value_to_yaml(msg.f0, out);
    out << ", ";
  }

  // member: f1
  {
    out << "f1: ";
    rosidl_generator_traits::value_to_yaml(msg.f1, out);
    out << ", ";
  }

  // member: f2
  {
    out << "f2: ";
    rosidl_generator_traits::value_to_yaml(msg.f2, out);
    out << ", ";
  }

  // member: tgd
  {
    if (msg.tgd.size() == 0) {
      out << "tgd: []";
    } else {
      out << "tgd: [";
      size_t pending_items = msg.tgd.size();
      for (auto item : msg.tgd) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: a_dot
  {
    out << "a_dot: ";
    rosidl_generator_traits::value_to_yaml(msg.a_dot, out);
    out << ", ";
  }

  // member: ndot
  {
    out << "ndot: ";
    rosidl_generator_traits::value_to_yaml(msg.ndot, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const GNSSEphemeris & msg,
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

  // member: sat
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "sat: ";
    rosidl_generator_traits::value_to_yaml(msg.sat, out);
    out << "\n";
  }

  // member: iode
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "iode: ";
    rosidl_generator_traits::value_to_yaml(msg.iode, out);
    out << "\n";
  }

  // member: iodc
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "iodc: ";
    rosidl_generator_traits::value_to_yaml(msg.iodc, out);
    out << "\n";
  }

  // member: sva
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "sva: ";
    rosidl_generator_traits::value_to_yaml(msg.sva, out);
    out << "\n";
  }

  // member: svh
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "svh: ";
    rosidl_generator_traits::value_to_yaml(msg.svh, out);
    out << "\n";
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

  // member: code
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "code: ";
    rosidl_generator_traits::value_to_yaml(msg.code, out);
    out << "\n";
  }

  // member: flag
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "flag: ";
    rosidl_generator_traits::value_to_yaml(msg.flag, out);
    out << "\n";
  }

  // member: toe
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "toe:\n";
    to_block_style_yaml(msg.toe, out, indentation + 2);
  }

  // member: toc
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "toc:\n";
    to_block_style_yaml(msg.toc, out, indentation + 2);
  }

  // member: ttr
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "ttr:\n";
    to_block_style_yaml(msg.ttr, out, indentation + 2);
  }

  // member: a
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "a: ";
    rosidl_generator_traits::value_to_yaml(msg.a, out);
    out << "\n";
  }

  // member: e
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "e: ";
    rosidl_generator_traits::value_to_yaml(msg.e, out);
    out << "\n";
  }

  // member: i0
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "i0: ";
    rosidl_generator_traits::value_to_yaml(msg.i0, out);
    out << "\n";
  }

  // member: omg_0
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "omg_0: ";
    rosidl_generator_traits::value_to_yaml(msg.omg_0, out);
    out << "\n";
  }

  // member: omg
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "omg: ";
    rosidl_generator_traits::value_to_yaml(msg.omg, out);
    out << "\n";
  }

  // member: m_0
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "m_0: ";
    rosidl_generator_traits::value_to_yaml(msg.m_0, out);
    out << "\n";
  }

  // member: deln
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "deln: ";
    rosidl_generator_traits::value_to_yaml(msg.deln, out);
    out << "\n";
  }

  // member: omg_d
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "omg_d: ";
    rosidl_generator_traits::value_to_yaml(msg.omg_d, out);
    out << "\n";
  }

  // member: idot
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "idot: ";
    rosidl_generator_traits::value_to_yaml(msg.idot, out);
    out << "\n";
  }

  // member: crc
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "crc: ";
    rosidl_generator_traits::value_to_yaml(msg.crc, out);
    out << "\n";
  }

  // member: crs
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "crs: ";
    rosidl_generator_traits::value_to_yaml(msg.crs, out);
    out << "\n";
  }

  // member: cuc
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "cuc: ";
    rosidl_generator_traits::value_to_yaml(msg.cuc, out);
    out << "\n";
  }

  // member: cus
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "cus: ";
    rosidl_generator_traits::value_to_yaml(msg.cus, out);
    out << "\n";
  }

  // member: cic
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "cic: ";
    rosidl_generator_traits::value_to_yaml(msg.cic, out);
    out << "\n";
  }

  // member: cis
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "cis: ";
    rosidl_generator_traits::value_to_yaml(msg.cis, out);
    out << "\n";
  }

  // member: toes
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "toes: ";
    rosidl_generator_traits::value_to_yaml(msg.toes, out);
    out << "\n";
  }

  // member: fit
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "fit: ";
    rosidl_generator_traits::value_to_yaml(msg.fit, out);
    out << "\n";
  }

  // member: f0
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "f0: ";
    rosidl_generator_traits::value_to_yaml(msg.f0, out);
    out << "\n";
  }

  // member: f1
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "f1: ";
    rosidl_generator_traits::value_to_yaml(msg.f1, out);
    out << "\n";
  }

  // member: f2
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "f2: ";
    rosidl_generator_traits::value_to_yaml(msg.f2, out);
    out << "\n";
  }

  // member: tgd
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.tgd.size() == 0) {
      out << "tgd: []\n";
    } else {
      out << "tgd:\n";
      for (auto item : msg.tgd) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: a_dot
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "a_dot: ";
    rosidl_generator_traits::value_to_yaml(msg.a_dot, out);
    out << "\n";
  }

  // member: ndot
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "ndot: ";
    rosidl_generator_traits::value_to_yaml(msg.ndot, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const GNSSEphemeris & msg, bool use_flow_style = false)
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
  const inertial_sense_ros2::msg::GNSSEphemeris & msg,
  std::ostream & out, size_t indentation = 0)
{
  inertial_sense_ros2::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use inertial_sense_ros2::msg::to_yaml() instead")]]
inline std::string to_yaml(const inertial_sense_ros2::msg::GNSSEphemeris & msg)
{
  return inertial_sense_ros2::msg::to_yaml(msg);
}

template<>
inline const char * data_type<inertial_sense_ros2::msg::GNSSEphemeris>()
{
  return "inertial_sense_ros2::msg::GNSSEphemeris";
}

template<>
inline const char * name<inertial_sense_ros2::msg::GNSSEphemeris>()
{
  return "inertial_sense_ros2/msg/GNSSEphemeris";
}

template<>
struct has_fixed_size<inertial_sense_ros2::msg::GNSSEphemeris>
  : std::integral_constant<bool, has_fixed_size<inertial_sense_ros2::msg::GTime>::value && has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<inertial_sense_ros2::msg::GNSSEphemeris>
  : std::integral_constant<bool, has_bounded_size<inertial_sense_ros2::msg::GTime>::value && has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<inertial_sense_ros2::msg::GNSSEphemeris>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // INERTIAL_SENSE_ROS2__MSG__DETAIL__GNSS_EPHEMERIS__TRAITS_HPP_
