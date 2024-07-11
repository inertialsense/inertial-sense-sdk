// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from inertial_sense_ros2:msg/GlonassEphemeris.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "inertial_sense_ros2/msg/glonass_ephemeris.h"


#ifndef INERTIAL_SENSE_ROS2__MSG__DETAIL__GLONASS_EPHEMERIS__STRUCT_H_
#define INERTIAL_SENSE_ROS2__MSG__DETAIL__GLONASS_EPHEMERIS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

// Include directives for member types
// Member 'toe'
// Member 'tof'
#include "inertial_sense_ros2/msg/detail/g_time__struct.h"

/// Struct defined in msg/GlonassEphemeris in the package inertial_sense_ros2.
typedef struct inertial_sense_ros2__msg__GlonassEphemeris
{
  /// satellite number
  int32_t sat;
  /// IODE (0-6 bit of tb field)
  int32_t iode;
  /// satellite frequency number
  int32_t frq;
  /// satellite health
  int32_t svh;
  /// satellite accuracy
  int32_t sva;
  /// satellite age of operation
  int32_t age;
  /// epoch of epherides (gpst)
  inertial_sense_ros2__msg__GTime toe;
  /// message frame time (gpst)
  inertial_sense_ros2__msg__GTime tof;
  /// satellite position (ecef) (m)
  double pos[3];
  /// satellite velocity (ecef) (m/s)
  double vel[3];
  /// satellite acceleration (ecef) (m/s^2)
  double acc[3];
  /// SV clock bias (s)
  double taun;
  /// relative freq bias
  double gamn;
  /// delay between L1 and L2 (s)
  double dtaun;
} inertial_sense_ros2__msg__GlonassEphemeris;

// Struct for a sequence of inertial_sense_ros2__msg__GlonassEphemeris.
typedef struct inertial_sense_ros2__msg__GlonassEphemeris__Sequence
{
  inertial_sense_ros2__msg__GlonassEphemeris * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} inertial_sense_ros2__msg__GlonassEphemeris__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // INERTIAL_SENSE_ROS2__MSG__DETAIL__GLONASS_EPHEMERIS__STRUCT_H_
