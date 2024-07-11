// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from inertial_sense_ros2:msg/GNSSObservation.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "inertial_sense_ros2/msg/gnss_observation.h"


#ifndef INERTIAL_SENSE_ROS2__MSG__DETAIL__GNSS_OBSERVATION__STRUCT_H_
#define INERTIAL_SENSE_ROS2__MSG__DETAIL__GNSS_OBSERVATION__STRUCT_H_

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

/// Struct defined in msg/GNSSObservation in the package inertial_sense_ros2.
typedef struct inertial_sense_ros2__msg__GNSSObservation
{
  std_msgs__msg__Header header;
  /// time of observation
  inertial_sense_ros2__msg__GTime time;
  /// satellite number
  uint8_t sat;
  /// receiver number
  uint8_t rcv;
  /// Signal Strength (0.25 dBHz)
  uint8_t snrr;
  /// Loss-of-Lock Indicator (bit1=loss-of-lock, bit2=half-cycle-invalid)
  uint8_t lli;
  /// code indicator (BeiDou: CODE_L1I, Other: CODE_L1C )
  uint8_t code;
  /// Estimated carrier phase measurement standard deviation (0.004 cycles)
  uint8_t qual_l;
  /// Estimated pseudorange measurement standard deviation (0.01 m)
  uint8_t qual_p;
  /// observation data carrier-phase (cycle)
  double l;
  /// observation data pseudorange (m)
  double p;
  /// observation data doppler frequency (0.002 Hz)
  float d;
} inertial_sense_ros2__msg__GNSSObservation;

// Struct for a sequence of inertial_sense_ros2__msg__GNSSObservation.
typedef struct inertial_sense_ros2__msg__GNSSObservation__Sequence
{
  inertial_sense_ros2__msg__GNSSObservation * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} inertial_sense_ros2__msg__GNSSObservation__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // INERTIAL_SENSE_ROS2__MSG__DETAIL__GNSS_OBSERVATION__STRUCT_H_
