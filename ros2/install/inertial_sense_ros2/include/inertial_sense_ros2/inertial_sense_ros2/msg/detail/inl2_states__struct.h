// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from inertial_sense_ros2:msg/INL2States.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "inertial_sense_ros2/msg/inl2_states.h"


#ifndef INERTIAL_SENSE_ROS2__MSG__DETAIL__INL2_STATES__STRUCT_H_
#define INERTIAL_SENSE_ROS2__MSG__DETAIL__INL2_STATES__STRUCT_H_

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
// Member 'quat_ecef'
#include "geometry_msgs/msg/detail/quaternion__struct.h"
// Member 'vel_ecef'
// Member 'pos_ecef'
// Member 'gyro_bias'
// Member 'accel_bias'
#include "geometry_msgs/msg/detail/vector3__struct.h"

/// Struct defined in msg/INL2States in the package inertial_sense_ros2.
typedef struct inertial_sense_ros2__msg__INL2States
{
  /// GPS time of week (since Sunday morning) in seconds
  std_msgs__msg__Header header;
  /// Quaternion body rotation with respect to ECEF
  geometry_msgs__msg__Quaternion quat_ecef;
  /// (m/s) Velocity in ECEF frame
  geometry_msgs__msg__Vector3 vel_ecef;
  /// (m) Position in ECEF frame
  geometry_msgs__msg__Vector3 pos_ecef;
  /// (rad/s) Gyro bias
  geometry_msgs__msg__Vector3 gyro_bias;
  /// (m/s^2) Accelerometer bias
  geometry_msgs__msg__Vector3 accel_bias;
  /// (m) Barometer bias
  float baro_bias;
  /// (rad) Magnetic declination
  float mag_dec;
  /// (rad) Magnetic inclination
  float mag_inc;
} inertial_sense_ros2__msg__INL2States;

// Struct for a sequence of inertial_sense_ros2__msg__INL2States.
typedef struct inertial_sense_ros2__msg__INL2States__Sequence
{
  inertial_sense_ros2__msg__INL2States * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} inertial_sense_ros2__msg__INL2States__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // INERTIAL_SENSE_ROS2__MSG__DETAIL__INL2_STATES__STRUCT_H_
