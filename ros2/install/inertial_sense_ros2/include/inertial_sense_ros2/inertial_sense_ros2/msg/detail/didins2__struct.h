// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from inertial_sense_ros2:msg/DIDINS2.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "inertial_sense_ros2/msg/didins2.h"


#ifndef INERTIAL_SENSE_ROS2__MSG__DETAIL__DIDINS2__STRUCT_H_
#define INERTIAL_SENSE_ROS2__MSG__DETAIL__DIDINS2__STRUCT_H_

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

/// Struct defined in msg/DIDINS2 in the package inertial_sense_ros2.
typedef struct inertial_sense_ros2__msg__DIDINS2
{
  std_msgs__msg__Header header;
  uint32_t week;
  double time_of_week;
  uint32_t ins_status;
  uint32_t hdw_status;
  float qn2b[4];
  float uvw[3];
  double lla[3];
} inertial_sense_ros2__msg__DIDINS2;

// Struct for a sequence of inertial_sense_ros2__msg__DIDINS2.
typedef struct inertial_sense_ros2__msg__DIDINS2__Sequence
{
  inertial_sense_ros2__msg__DIDINS2 * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} inertial_sense_ros2__msg__DIDINS2__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // INERTIAL_SENSE_ROS2__MSG__DETAIL__DIDINS2__STRUCT_H_
