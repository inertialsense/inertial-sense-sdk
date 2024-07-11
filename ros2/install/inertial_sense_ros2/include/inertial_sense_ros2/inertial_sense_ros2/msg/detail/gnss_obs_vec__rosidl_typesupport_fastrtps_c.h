// generated from rosidl_typesupport_fastrtps_c/resource/idl__rosidl_typesupport_fastrtps_c.h.em
// with input from inertial_sense_ros2:msg/GNSSObsVec.idl
// generated code does not contain a copyright notice
#ifndef INERTIAL_SENSE_ROS2__MSG__DETAIL__GNSS_OBS_VEC__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_
#define INERTIAL_SENSE_ROS2__MSG__DETAIL__GNSS_OBS_VEC__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_


#include <stddef.h>
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "inertial_sense_ros2/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "inertial_sense_ros2/msg/detail/gnss_obs_vec__struct.h"
#include "fastcdr/Cdr.h"

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_inertial_sense_ros2
bool cdr_serialize_inertial_sense_ros2__msg__GNSSObsVec(
  const inertial_sense_ros2__msg__GNSSObsVec * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_inertial_sense_ros2
bool cdr_deserialize_inertial_sense_ros2__msg__GNSSObsVec(
  eprosima::fastcdr::Cdr &,
  inertial_sense_ros2__msg__GNSSObsVec * ros_message);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_inertial_sense_ros2
size_t get_serialized_size_inertial_sense_ros2__msg__GNSSObsVec(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_inertial_sense_ros2
size_t max_serialized_size_inertial_sense_ros2__msg__GNSSObsVec(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_inertial_sense_ros2
bool cdr_serialize_key_inertial_sense_ros2__msg__GNSSObsVec(
  const inertial_sense_ros2__msg__GNSSObsVec * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_inertial_sense_ros2
size_t get_serialized_size_key_inertial_sense_ros2__msg__GNSSObsVec(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_inertial_sense_ros2
size_t max_serialized_size_key_inertial_sense_ros2__msg__GNSSObsVec(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_inertial_sense_ros2
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, inertial_sense_ros2, msg, GNSSObsVec)();

#ifdef __cplusplus
}
#endif

#endif  // INERTIAL_SENSE_ROS2__MSG__DETAIL__GNSS_OBS_VEC__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_
