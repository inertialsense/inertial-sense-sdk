// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from inertial_sense_ros2:msg/PIMU.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "inertial_sense_ros2/msg/pimu.h"


#ifndef INERTIAL_SENSE_ROS2__MSG__DETAIL__PIMU__STRUCT_H_
#define INERTIAL_SENSE_ROS2__MSG__DETAIL__PIMU__STRUCT_H_

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
// Member 'dtheta'
// Member 'dvel'
#include "geometry_msgs/msg/detail/vector3__struct.h"

/// Struct defined in msg/PIMU in the package inertial_sense_ros2.
typedef struct inertial_sense_ros2__msg__PIMU
{
  std_msgs__msg__Header header;
  /// change in angle over time period (rodriguez vector)
  geometry_msgs__msg__Vector3 dtheta;
  /// change in velocity over time period (m/s)
  geometry_msgs__msg__Vector3 dvel;
  /// length of time period (s)
  double dt;
} inertial_sense_ros2__msg__PIMU;

// Struct for a sequence of inertial_sense_ros2__msg__PIMU.
typedef struct inertial_sense_ros2__msg__PIMU__Sequence
{
  inertial_sense_ros2__msg__PIMU * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} inertial_sense_ros2__msg__PIMU__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // INERTIAL_SENSE_ROS2__MSG__DETAIL__PIMU__STRUCT_H_
