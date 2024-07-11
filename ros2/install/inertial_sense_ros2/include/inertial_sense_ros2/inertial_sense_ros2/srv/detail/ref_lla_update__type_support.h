// generated from rosidl_generator_c/resource/idl__type_support.h.em
// with input from inertial_sense_ros2:srv/RefLLAUpdate.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "inertial_sense_ros2/srv/ref_lla_update.h"


#ifndef INERTIAL_SENSE_ROS2__SRV__DETAIL__REF_LLA_UPDATE__TYPE_SUPPORT_H_
#define INERTIAL_SENSE_ROS2__SRV__DETAIL__REF_LLA_UPDATE__TYPE_SUPPORT_H_

#include "rosidl_typesupport_interface/macros.h"

#include "inertial_sense_ros2/msg/rosidl_generator_c__visibility_control.h"

#ifdef __cplusplus
extern "C"
{
#endif

#include "rosidl_runtime_c/message_type_support_struct.h"

// Forward declare the get type support functions for this type.
ROSIDL_GENERATOR_C_PUBLIC_inertial_sense_ros2
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
  rosidl_typesupport_c,
  inertial_sense_ros2,
  srv,
  RefLLAUpdate_Request
)(void);

// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"

// Forward declare the get type support functions for this type.
ROSIDL_GENERATOR_C_PUBLIC_inertial_sense_ros2
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
  rosidl_typesupport_c,
  inertial_sense_ros2,
  srv,
  RefLLAUpdate_Response
)(void);

// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"

// Forward declare the get type support functions for this type.
ROSIDL_GENERATOR_C_PUBLIC_inertial_sense_ros2
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
  rosidl_typesupport_c,
  inertial_sense_ros2,
  srv,
  RefLLAUpdate_Event
)(void);

#include "rosidl_runtime_c/service_type_support_struct.h"

// Forward declare the get type support functions for this type.
ROSIDL_GENERATOR_C_PUBLIC_inertial_sense_ros2
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(
  rosidl_typesupport_c,
  inertial_sense_ros2,
  srv,
  RefLLAUpdate
)(void);

// Forward declare the function to create a service event message for this type.
ROSIDL_GENERATOR_C_PUBLIC_inertial_sense_ros2
void *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_CREATE_EVENT_MESSAGE_SYMBOL_NAME(
  rosidl_typesupport_c,
  inertial_sense_ros2,
  srv,
  RefLLAUpdate
)(
  const rosidl_service_introspection_info_t * info,
  rcutils_allocator_t * allocator,
  const void * request_message,
  const void * response_message);

// Forward declare the function to destroy a service event message for this type.
ROSIDL_GENERATOR_C_PUBLIC_inertial_sense_ros2
bool
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_DESTROY_EVENT_MESSAGE_SYMBOL_NAME(
  rosidl_typesupport_c,
  inertial_sense_ros2,
  srv,
  RefLLAUpdate
)(
  void * event_msg,
  rcutils_allocator_t * allocator);

#ifdef __cplusplus
}
#endif

#endif  // INERTIAL_SENSE_ROS2__SRV__DETAIL__REF_LLA_UPDATE__TYPE_SUPPORT_H_
