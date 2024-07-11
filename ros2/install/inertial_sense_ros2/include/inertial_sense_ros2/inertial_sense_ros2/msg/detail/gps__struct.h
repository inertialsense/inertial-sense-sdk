// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from inertial_sense_ros2:msg/GPS.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "inertial_sense_ros2/msg/gps.h"


#ifndef INERTIAL_SENSE_ROS2__MSG__DETAIL__GPS__STRUCT_H_
#define INERTIAL_SENSE_ROS2__MSG__DETAIL__GPS__STRUCT_H_

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
// Member 'pos_ecef'
// Member 'vel_ecef'
#include "geometry_msgs/msg/detail/vector3__struct.h"

/// Struct defined in msg/GPS in the package inertial_sense_ros2.
typedef struct inertial_sense_ros2__msg__GPS
{
  std_msgs__msg__Header header;
  /// GPS number of weeks since January 6th, 1980
  uint32_t week;
  /// Number of satellites used in solution
  int8_t num_sat;
  /// Fix type and status flags.  See GPS_STATUS_FIX_... and GPS_STATUS_FLAGS_... in sdk data_sets.h
  uint32_t status;
  /// Average of all non-zero satellite carrier to noise ratios, signal strengths (dBHz)
  int32_t cno;
  /// Latitude (degrees)
  double latitude;
  /// Longitude (degrees)
  double longitude;
  /// Height above ellipsoid (not MSL) (m)
  double altitude;
  /// Position (m) in ECEF
  geometry_msgs__msg__Vector3 pos_ecef;
  /// Velocity (m/s) in ECEF
  geometry_msgs__msg__Vector3 vel_ecef;
  /// Height above MSL
  float hmsl;
  /// Horizontal accuracy
  float hacc;
  /// Vertical accuracy
  float vacc;
  /// Speed accuracy (m/s)
  float sacc;
  /// Position Dilution of Precision (m)
  float pdop;
} inertial_sense_ros2__msg__GPS;

// Struct for a sequence of inertial_sense_ros2__msg__GPS.
typedef struct inertial_sense_ros2__msg__GPS__Sequence
{
  inertial_sense_ros2__msg__GPS * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} inertial_sense_ros2__msg__GPS__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // INERTIAL_SENSE_ROS2__MSG__DETAIL__GPS__STRUCT_H_
