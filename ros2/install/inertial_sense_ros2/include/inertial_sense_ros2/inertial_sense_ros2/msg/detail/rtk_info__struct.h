// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from inertial_sense_ros2:msg/RTKInfo.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "inertial_sense_ros2/msg/rtk_info.h"


#ifndef INERTIAL_SENSE_ROS2__MSG__DETAIL__RTK_INFO__STRUCT_H_
#define INERTIAL_SENSE_ROS2__MSG__DETAIL__RTK_INFO__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"

/// Struct defined in msg/RTKInfo in the package inertial_sense_ros2.
typedef struct inertial_sense_ros2__msg__RTKInfo
{
  std_msgs__msg__Header header;
  /// base position in lat-lon-altitude (deg, deg, m)
  float base_lla[3];
  /// number of cycle slips detected
  uint32_t cycle_slip_count;
  /// number of observations from rover (GPS, Glonass, Gallileo, Beidou, Qzs)
  uint32_t rover_obs;
  /// number of observations from base (GPS, Glonass, Gallileo, Beidou, Qzs)
  uint32_t base_obs;
  /// number of ephemeris messages from rover (GPS, Glonass, Gallileo, Beidou, Qzs)
  uint32_t rover_eph;
  /// number of ephemeris messages from rover (GPS, Glonass, Gallileo, Beidou, Qzs)
  uint32_t base_eph;
  /// number of base station antenna position measurements
  uint32_t base_ant_count;
} inertial_sense_ros2__msg__RTKInfo;

// Struct for a sequence of inertial_sense_ros2__msg__RTKInfo.
typedef struct inertial_sense_ros2__msg__RTKInfo__Sequence
{
  inertial_sense_ros2__msg__RTKInfo * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} inertial_sense_ros2__msg__RTKInfo__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // INERTIAL_SENSE_ROS2__MSG__DETAIL__RTK_INFO__STRUCT_H_
