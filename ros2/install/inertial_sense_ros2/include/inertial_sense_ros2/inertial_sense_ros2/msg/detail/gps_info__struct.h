// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from inertial_sense_ros2:msg/GPSInfo.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "inertial_sense_ros2/msg/gps_info.h"


#ifndef INERTIAL_SENSE_ROS2__MSG__DETAIL__GPS_INFO__STRUCT_H_
#define INERTIAL_SENSE_ROS2__MSG__DETAIL__GPS_INFO__STRUCT_H_

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
// Member 'sattelite_info'
#include "inertial_sense_ros2/msg/detail/sat_info__struct.h"

/// Struct defined in msg/GPSInfo in the package inertial_sense_ros2.
typedef struct inertial_sense_ros2__msg__GPSInfo
{
  std_msgs__msg__Header header;
  /// number of sattelites in the sky
  uint32_t num_sats;
  /// information about sattelites
  inertial_sense_ros2__msg__SatInfo sattelite_info[50];
} inertial_sense_ros2__msg__GPSInfo;

// Struct for a sequence of inertial_sense_ros2__msg__GPSInfo.
typedef struct inertial_sense_ros2__msg__GPSInfo__Sequence
{
  inertial_sense_ros2__msg__GPSInfo * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} inertial_sense_ros2__msg__GPSInfo__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // INERTIAL_SENSE_ROS2__MSG__DETAIL__GPS_INFO__STRUCT_H_
