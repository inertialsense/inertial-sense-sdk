// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from inertial_sense_ros2:msg/PIMU.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "inertial_sense_ros2/msg/detail/pimu__rosidl_typesupport_introspection_c.h"
#include "inertial_sense_ros2/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "inertial_sense_ros2/msg/detail/pimu__functions.h"
#include "inertial_sense_ros2/msg/detail/pimu__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `dtheta`
// Member `dvel`
#include "geometry_msgs/msg/vector3.h"
// Member `dtheta`
// Member `dvel`
#include "geometry_msgs/msg/detail/vector3__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void inertial_sense_ros2__msg__PIMU__rosidl_typesupport_introspection_c__PIMU_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  inertial_sense_ros2__msg__PIMU__init(message_memory);
}

void inertial_sense_ros2__msg__PIMU__rosidl_typesupport_introspection_c__PIMU_fini_function(void * message_memory)
{
  inertial_sense_ros2__msg__PIMU__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember inertial_sense_ros2__msg__PIMU__rosidl_typesupport_introspection_c__PIMU_message_member_array[4] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(inertial_sense_ros2__msg__PIMU, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "dtheta",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(inertial_sense_ros2__msg__PIMU, dtheta),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "dvel",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(inertial_sense_ros2__msg__PIMU, dvel),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "dt",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(inertial_sense_ros2__msg__PIMU, dt),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers inertial_sense_ros2__msg__PIMU__rosidl_typesupport_introspection_c__PIMU_message_members = {
  "inertial_sense_ros2__msg",  // message namespace
  "PIMU",  // message name
  4,  // number of fields
  sizeof(inertial_sense_ros2__msg__PIMU),
  false,  // has_any_key_member_
  inertial_sense_ros2__msg__PIMU__rosidl_typesupport_introspection_c__PIMU_message_member_array,  // message members
  inertial_sense_ros2__msg__PIMU__rosidl_typesupport_introspection_c__PIMU_init_function,  // function to initialize message memory (memory has to be allocated)
  inertial_sense_ros2__msg__PIMU__rosidl_typesupport_introspection_c__PIMU_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t inertial_sense_ros2__msg__PIMU__rosidl_typesupport_introspection_c__PIMU_message_type_support_handle = {
  0,
  &inertial_sense_ros2__msg__PIMU__rosidl_typesupport_introspection_c__PIMU_message_members,
  get_message_typesupport_handle_function,
  &inertial_sense_ros2__msg__PIMU__get_type_hash,
  &inertial_sense_ros2__msg__PIMU__get_type_description,
  &inertial_sense_ros2__msg__PIMU__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_inertial_sense_ros2
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, inertial_sense_ros2, msg, PIMU)() {
  inertial_sense_ros2__msg__PIMU__rosidl_typesupport_introspection_c__PIMU_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  inertial_sense_ros2__msg__PIMU__rosidl_typesupport_introspection_c__PIMU_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, Vector3)();
  inertial_sense_ros2__msg__PIMU__rosidl_typesupport_introspection_c__PIMU_message_member_array[2].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, Vector3)();
  if (!inertial_sense_ros2__msg__PIMU__rosidl_typesupport_introspection_c__PIMU_message_type_support_handle.typesupport_identifier) {
    inertial_sense_ros2__msg__PIMU__rosidl_typesupport_introspection_c__PIMU_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &inertial_sense_ros2__msg__PIMU__rosidl_typesupport_introspection_c__PIMU_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
