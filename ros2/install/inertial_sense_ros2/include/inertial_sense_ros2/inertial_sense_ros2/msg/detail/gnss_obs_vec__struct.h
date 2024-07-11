// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from inertial_sense_ros2:msg/GNSSObsVec.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "inertial_sense_ros2/msg/gnss_obs_vec.h"


#ifndef INERTIAL_SENSE_ROS2__MSG__DETAIL__GNSS_OBS_VEC__STRUCT_H_
#define INERTIAL_SENSE_ROS2__MSG__DETAIL__GNSS_OBS_VEC__STRUCT_H_

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
// Member 'time'
#include "inertial_sense_ros2/msg/detail/g_time__struct.h"
// Member 'obs'
#include "inertial_sense_ros2/msg/detail/gnss_observation__struct.h"

/// Struct defined in msg/GNSSObsVec in the package inertial_sense_ros2.
typedef struct inertial_sense_ros2__msg__GNSSObsVec
{
  std_msgs__msg__Header header;
  /// time of all contained observations (UTC Time w/o Leap Seconds)
  inertial_sense_ros2__msg__GTime time;
  /// Vector of observations
  inertial_sense_ros2__msg__GNSSObservation__Sequence obs;
} inertial_sense_ros2__msg__GNSSObsVec;

// Struct for a sequence of inertial_sense_ros2__msg__GNSSObsVec.
typedef struct inertial_sense_ros2__msg__GNSSObsVec__Sequence
{
  inertial_sense_ros2__msg__GNSSObsVec * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} inertial_sense_ros2__msg__GNSSObsVec__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // INERTIAL_SENSE_ROS2__MSG__DETAIL__GNSS_OBS_VEC__STRUCT_H_
