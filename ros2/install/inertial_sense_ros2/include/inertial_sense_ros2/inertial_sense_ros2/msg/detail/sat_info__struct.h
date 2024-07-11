// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from inertial_sense_ros2:msg/SatInfo.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "inertial_sense_ros2/msg/sat_info.h"


#ifndef INERTIAL_SENSE_ROS2__MSG__DETAIL__SAT_INFO__STRUCT_H_
#define INERTIAL_SENSE_ROS2__MSG__DETAIL__SAT_INFO__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

/// Struct defined in msg/SatInfo in the package inertial_sense_ros2.
typedef struct inertial_sense_ros2__msg__SatInfo
{
  /// sattelite id
  uint32_t sat_id;
  /// Carrier to noise ratio
  uint32_t cno;
} inertial_sense_ros2__msg__SatInfo;

// Struct for a sequence of inertial_sense_ros2__msg__SatInfo.
typedef struct inertial_sense_ros2__msg__SatInfo__Sequence
{
  inertial_sense_ros2__msg__SatInfo * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} inertial_sense_ros2__msg__SatInfo__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // INERTIAL_SENSE_ROS2__MSG__DETAIL__SAT_INFO__STRUCT_H_
