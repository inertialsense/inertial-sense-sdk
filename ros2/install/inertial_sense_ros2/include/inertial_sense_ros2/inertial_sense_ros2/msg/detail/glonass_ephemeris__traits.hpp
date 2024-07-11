// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from inertial_sense_ros2:msg/GlonassEphemeris.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "inertial_sense_ros2/msg/glonass_ephemeris.hpp"


#ifndef INERTIAL_SENSE_ROS2__MSG__DETAIL__GLONASS_EPHEMERIS__TRAITS_HPP_
#define INERTIAL_SENSE_ROS2__MSG__DETAIL__GLONASS_EPHEMERIS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "inertial_sense_ros2/msg/detail/glonass_ephemeris__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'toe'
// Member 'tof'
#include "inertial_sense_ros2/msg/detail/g_time__traits.hpp"

namespace inertial_sense_ros2
{

namespace msg
{

inline void to_flow_style_yaml(
  const GlonassEphemeris & msg,
  std::ostream & out)
{
  out << "{";
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

  // member: frq
  {
    out << "frq: ";
    rosidl_generator_traits::value_to_yaml(msg.frq, out);
    out << ", ";
  }

  // member: svh
  {
    out << "svh: ";
    rosidl_generator_traits::value_to_yaml(msg.svh, out);
    out << ", ";
  }

  // member: sva
  {
    out << "sva: ";
    rosidl_generator_traits::value_to_yaml(msg.sva, out);
    out << ", ";
  }

  // member: age
  {
    out << "age: ";
    rosidl_generator_traits::value_to_yaml(msg.age, out);
    out << ", ";
  }

  // member: toe
  {
    out << "toe: ";
    to_flow_style_yaml(msg.toe, out);
    out << ", ";
  }

  // member: tof
  {
    out << "tof: ";
    to_flow_style_yaml(msg.tof, out);
    out << ", ";
  }

  // member: pos
  {
    if (msg.pos.size() == 0) {
      out << "pos: []";
    } else {
      out << "pos: [";
      size_t pending_items = msg.pos.size();
      for (auto item : msg.pos) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: vel
  {
    if (msg.vel.size() == 0) {
      out << "vel: []";
    } else {
      out << "vel: [";
      size_t pending_items = msg.vel.size();
      for (auto item : msg.vel) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: acc
  {
    if (msg.acc.size() == 0) {
      out << "acc: []";
    } else {
      out << "acc: [";
      size_t pending_items = msg.acc.size();
      for (auto item : msg.acc) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: taun
  {
    out << "taun: ";
    rosidl_generator_traits::value_to_yaml(msg.taun, out);
    out << ", ";
  }

  // member: gamn
  {
    out << "gamn: ";
    rosidl_generator_traits::value_to_yaml(msg.gamn, out);
    out << ", ";
  }

  // member: dtaun
  {
    out << "dtaun: ";
    rosidl_generator_traits::value_to_yaml(msg.dtaun, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const GlonassEphemeris & msg,
  std::ostream & out, size_t indentation = 0)
{
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

  // member: frq
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "frq: ";
    rosidl_generator_traits::value_to_yaml(msg.frq, out);
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

  // member: sva
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "sva: ";
    rosidl_generator_traits::value_to_yaml(msg.sva, out);
    out << "\n";
  }

  // member: age
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "age: ";
    rosidl_generator_traits::value_to_yaml(msg.age, out);
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

  // member: tof
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "tof:\n";
    to_block_style_yaml(msg.tof, out, indentation + 2);
  }

  // member: pos
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.pos.size() == 0) {
      out << "pos: []\n";
    } else {
      out << "pos:\n";
      for (auto item : msg.pos) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: vel
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.vel.size() == 0) {
      out << "vel: []\n";
    } else {
      out << "vel:\n";
      for (auto item : msg.vel) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: acc
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.acc.size() == 0) {
      out << "acc: []\n";
    } else {
      out << "acc:\n";
      for (auto item : msg.acc) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: taun
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "taun: ";
    rosidl_generator_traits::value_to_yaml(msg.taun, out);
    out << "\n";
  }

  // member: gamn
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "gamn: ";
    rosidl_generator_traits::value_to_yaml(msg.gamn, out);
    out << "\n";
  }

  // member: dtaun
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "dtaun: ";
    rosidl_generator_traits::value_to_yaml(msg.dtaun, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const GlonassEphemeris & msg, bool use_flow_style = false)
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
  const inertial_sense_ros2::msg::GlonassEphemeris & msg,
  std::ostream & out, size_t indentation = 0)
{
  inertial_sense_ros2::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use inertial_sense_ros2::msg::to_yaml() instead")]]
inline std::string to_yaml(const inertial_sense_ros2::msg::GlonassEphemeris & msg)
{
  return inertial_sense_ros2::msg::to_yaml(msg);
}

template<>
inline const char * data_type<inertial_sense_ros2::msg::GlonassEphemeris>()
{
  return "inertial_sense_ros2::msg::GlonassEphemeris";
}

template<>
inline const char * name<inertial_sense_ros2::msg::GlonassEphemeris>()
{
  return "inertial_sense_ros2/msg/GlonassEphemeris";
}

template<>
struct has_fixed_size<inertial_sense_ros2::msg::GlonassEphemeris>
  : std::integral_constant<bool, has_fixed_size<inertial_sense_ros2::msg::GTime>::value> {};

template<>
struct has_bounded_size<inertial_sense_ros2::msg::GlonassEphemeris>
  : std::integral_constant<bool, has_bounded_size<inertial_sense_ros2::msg::GTime>::value> {};

template<>
struct is_message<inertial_sense_ros2::msg::GlonassEphemeris>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // INERTIAL_SENSE_ROS2__MSG__DETAIL__GLONASS_EPHEMERIS__TRAITS_HPP_
