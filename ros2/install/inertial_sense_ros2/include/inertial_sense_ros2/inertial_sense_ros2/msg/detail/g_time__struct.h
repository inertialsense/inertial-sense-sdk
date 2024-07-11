// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from inertial_sense_ros2:msg/GTime.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "inertial_sense_ros2/msg/g_time.h"


#ifndef INERTIAL_SENSE_ROS2__MSG__DETAIL__G_TIME__STRUCT_H_
#define INERTIAL_SENSE_ROS2__MSG__DETAIL__G_TIME__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

/// Struct defined in msg/GTime in the package inertial_sense_ros2.
typedef struct inertial_sense_ros2__msg__GTime
{
  int64_t time;
  double sec;
} inertial_sense_ros2__msg__GTime;

// Struct for a sequence of inertial_sense_ros2__msg__GTime.
typedef struct inertial_sense_ros2__msg__GTime__Sequence
{
  inertial_sense_ros2__msg__GTime * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} inertial_sense_ros2__msg__GTime__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // INERTIAL_SENSE_ROS2__MSG__DETAIL__G_TIME__STRUCT_H_
