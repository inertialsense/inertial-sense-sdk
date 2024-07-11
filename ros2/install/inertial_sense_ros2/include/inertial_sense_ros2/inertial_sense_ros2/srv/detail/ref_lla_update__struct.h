// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from inertial_sense_ros2:srv/RefLLAUpdate.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "inertial_sense_ros2/srv/ref_lla_update.h"


#ifndef INERTIAL_SENSE_ROS2__SRV__DETAIL__REF_LLA_UPDATE__STRUCT_H_
#define INERTIAL_SENSE_ROS2__SRV__DETAIL__REF_LLA_UPDATE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/RefLLAUpdate in the package inertial_sense_ros2.
typedef struct inertial_sense_ros2__srv__RefLLAUpdate_Request
{
  double lla[3];
} inertial_sense_ros2__srv__RefLLAUpdate_Request;

// Struct for a sequence of inertial_sense_ros2__srv__RefLLAUpdate_Request.
typedef struct inertial_sense_ros2__srv__RefLLAUpdate_Request__Sequence
{
  inertial_sense_ros2__srv__RefLLAUpdate_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} inertial_sense_ros2__srv__RefLLAUpdate_Request__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'message'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/RefLLAUpdate in the package inertial_sense_ros2.
typedef struct inertial_sense_ros2__srv__RefLLAUpdate_Response
{
  bool success;
  rosidl_runtime_c__String message;
} inertial_sense_ros2__srv__RefLLAUpdate_Response;

// Struct for a sequence of inertial_sense_ros2__srv__RefLLAUpdate_Response.
typedef struct inertial_sense_ros2__srv__RefLLAUpdate_Response__Sequence
{
  inertial_sense_ros2__srv__RefLLAUpdate_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} inertial_sense_ros2__srv__RefLLAUpdate_Response__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.h"

// constants for array fields with an upper bound
// request
enum
{
  inertial_sense_ros2__srv__RefLLAUpdate_Event__request__MAX_SIZE = 1
};
// response
enum
{
  inertial_sense_ros2__srv__RefLLAUpdate_Event__response__MAX_SIZE = 1
};

/// Struct defined in srv/RefLLAUpdate in the package inertial_sense_ros2.
typedef struct inertial_sense_ros2__srv__RefLLAUpdate_Event
{
  service_msgs__msg__ServiceEventInfo info;
  inertial_sense_ros2__srv__RefLLAUpdate_Request__Sequence request;
  inertial_sense_ros2__srv__RefLLAUpdate_Response__Sequence response;
} inertial_sense_ros2__srv__RefLLAUpdate_Event;

// Struct for a sequence of inertial_sense_ros2__srv__RefLLAUpdate_Event.
typedef struct inertial_sense_ros2__srv__RefLLAUpdate_Event__Sequence
{
  inertial_sense_ros2__srv__RefLLAUpdate_Event * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} inertial_sense_ros2__srv__RefLLAUpdate_Event__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // INERTIAL_SENSE_ROS2__SRV__DETAIL__REF_LLA_UPDATE__STRUCT_H_
