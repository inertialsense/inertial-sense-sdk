// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from inertial_sense_ros2:msg/GNSSEphemeris.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "inertial_sense_ros2/msg/gnss_ephemeris.h"


#ifndef INERTIAL_SENSE_ROS2__MSG__DETAIL__GNSS_EPHEMERIS__STRUCT_H_
#define INERTIAL_SENSE_ROS2__MSG__DETAIL__GNSS_EPHEMERIS__STRUCT_H_

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
// Member 'toe'
// Member 'toc'
// Member 'ttr'
#include "inertial_sense_ros2/msg/detail/g_time__struct.h"

/// Struct defined in msg/GNSSEphemeris in the package inertial_sense_ros2.
typedef struct inertial_sense_ros2__msg__GNSSEphemeris
{
  std_msgs__msg__Header header;
  /// satellite number
  int32_t sat;
  /// IODE Issue of Data, Ephemeris (ephemeris version)
  int32_t iode;
  /// IODC Issue of Data, Clock (clock version)
  int32_t iodc;
  /// SV accuracy (URA index) IRN-IS-200H p.97
  int32_t sva;
  /// SV health GPS/QZS (0:ok)
  int32_t svh;
  /// GPS/QZS: gps week, GAL: galileo week
  int32_t week;
  /// GPS/QZS: code on L2 * (00=Invalid, 01 = P Code ON, 11 = C/A code ON, 11 = Invalid) * GAL/CMP: data sources
  int32_t code;
  /// GPS/QZS: L2 P data flag (indicates that the NAV data stream was commanded OFF on the P-code of the in-phase component of the L2 channel) *  CMP: nav type
  int32_t flag;
  /// Toe
  inertial_sense_ros2__msg__GTime toe;
  /// clock data reference time (s) (20.3.4.5)
  inertial_sense_ros2__msg__GTime toc;
  /// T_trans
  inertial_sense_ros2__msg__GTime ttr;
  /// Semi-Major Axis m
  double a;
  /// Eccentricity (no units)
  double e;
  /// Inclination Angle at Reference Time (rad)
  double i0;
  /// Longitude of Ascending Node of Orbit Plane at Weekly Epoch (rad)
  double omg_0;
  /// Argument of Perigee (rad)
  double omg;
  /// Mean Anomaly at Reference Time (rad)
  double m_0;
  /// Mean Motion Difference From Computed Value (rad)
  double deln;
  /// Rate of Right Ascension (rad/s)
  double omg_d;
  /// Rate of Inclination Angle (rad/s)
  double idot;
  /// Amplitude of the Cosine Harmonic Correction Term to the Orbit Radius
  double crc;
  /// Amplitude of the Sine Harmonic Correction Term to the Orbit Radius (m)
  double crs;
  /// Amplitude of the Cosine Harmonic Correction Term to the Argument of Latitude (rad)
  double cuc;
  /// Amplitude of the Sine Harmonic Correction Term to the Argument of Latitude (rad)
  double cus;
  /// Amplitude of the Cosine Harmonic Correction Term to the Angle of Inclination (rad)
  double cic;
  /// Amplitude of the Sine Harmonic Correction Term to the Angle of Inclination (rad)
  double cis;
  /// Reference Time Ephemeris in week (s)
  double toes;
  /// fit interval (h) (0: 4 hours, 1:greater than 4 hours)
  double fit;
  /// SV clock parameters - af0
  double f0;
  /// SV clock parameters - af1
  double f1;
  /// SV clock parameters - af2
  double f2;
  /// group delay parameters: GPS/QZS:tgd[0]=TGD (IRN-IS-200H p.103) * GAL:tgd[0]=BGD E5a/E1,tgd[1]=BGD E5b/E1 * CMP :tgd[0]=BGD1,tgd[1]=BGD2
  double tgd[4];
  /// Adot for CNAV
  double a_dot;
  /// ndot for CNAV
  double ndot;
} inertial_sense_ros2__msg__GNSSEphemeris;

// Struct for a sequence of inertial_sense_ros2__msg__GNSSEphemeris.
typedef struct inertial_sense_ros2__msg__GNSSEphemeris__Sequence
{
  inertial_sense_ros2__msg__GNSSEphemeris * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} inertial_sense_ros2__msg__GNSSEphemeris__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // INERTIAL_SENSE_ROS2__MSG__DETAIL__GNSS_EPHEMERIS__STRUCT_H_
