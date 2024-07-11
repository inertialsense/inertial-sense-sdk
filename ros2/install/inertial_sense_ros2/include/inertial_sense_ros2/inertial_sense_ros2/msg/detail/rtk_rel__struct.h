// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from inertial_sense_ros2:msg/RTKRel.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "inertial_sense_ros2/msg/rtk_rel.h"


#ifndef INERTIAL_SENSE_ROS2__MSG__DETAIL__RTK_REL__STRUCT_H_
#define INERTIAL_SENSE_ROS2__MSG__DETAIL__RTK_REL__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

/// Constant 'GPS_STATUS_FIX_3D'.
enum
{
  inertial_sense_ros2__msg__RTKRel__GPS_STATUS_FIX_3D = 1
};

/// Constant 'GPS_STATUS_FIX_RTK_SINGLE'.
enum
{
  inertial_sense_ros2__msg__RTKRel__GPS_STATUS_FIX_RTK_SINGLE = 2
};

/// Constant 'GPS_STATUS_FIX_RTK_FLOAT'.
enum
{
  inertial_sense_ros2__msg__RTKRel__GPS_STATUS_FIX_RTK_FLOAT = 3
};

/// Constant 'GPS_STATUS_FIX_RTK_FIX'.
enum
{
  inertial_sense_ros2__msg__RTKRel__GPS_STATUS_FIX_RTK_FIX = 4
};

/// Constant 'GPS_STATUS_FLAGS_RTK_FIX_AND_HOLD'.
enum
{
  inertial_sense_ros2__msg__RTKRel__GPS_STATUS_FLAGS_RTK_FIX_AND_HOLD = 5
};

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'vector_base_to_rover'
#include "geometry_msgs/msg/detail/vector3__struct.h"

/// Struct defined in msg/RTKRel in the package inertial_sense_ros2.
typedef struct inertial_sense_ros2__msg__RTKRel
{
  std_msgs__msg__Header header;
  /// Age of differential (seconds)
  float differential_age;
  /// Ambiguity resolution ratio factor for validation
  float ar_ratio;
  /// GPS navigation fix type
  uint8_t e_gps_status_fix;
  /// Vector to base (m) in ECEF - If Compassing enabled, this is the 3-vector from antenna 2 to antenna 1
  geometry_msgs__msg__Vector3 vector_base_to_rover;
  /// Distance to Base (m)
  float distance_base_to_rover;
  /// Angle from north to vectorToBase in local tangent plane. (rad)
  float heading_base_to_rover;
} inertial_sense_ros2__msg__RTKRel;

// Struct for a sequence of inertial_sense_ros2__msg__RTKRel.
typedef struct inertial_sense_ros2__msg__RTKRel__Sequence
{
  inertial_sense_ros2__msg__RTKRel * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} inertial_sense_ros2__msg__RTKRel__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // INERTIAL_SENSE_ROS2__MSG__DETAIL__RTK_REL__STRUCT_H_
