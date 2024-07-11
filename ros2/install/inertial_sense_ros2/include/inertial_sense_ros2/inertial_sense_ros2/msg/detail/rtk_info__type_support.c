// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from inertial_sense_ros2:msg/RTKInfo.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "inertial_sense_ros2/msg/detail/rtk_info__rosidl_typesupport_introspection_c.h"
#include "inertial_sense_ros2/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "inertial_sense_ros2/msg/detail/rtk_info__functions.h"
#include "inertial_sense_ros2/msg/detail/rtk_info__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void inertial_sense_ros2__msg__RTKInfo__rosidl_typesupport_introspection_c__RTKInfo_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  inertial_sense_ros2__msg__RTKInfo__init(message_memory);
}

void inertial_sense_ros2__msg__RTKInfo__rosidl_typesupport_introspection_c__RTKInfo_fini_function(void * message_memory)
{
  inertial_sense_ros2__msg__RTKInfo__fini(message_memory);
}

size_t inertial_sense_ros2__msg__RTKInfo__rosidl_typesupport_introspection_c__size_function__RTKInfo__base_lla(
  const void * untyped_member)
{
  (void)untyped_member;
  return 3;
}

const void * inertial_sense_ros2__msg__RTKInfo__rosidl_typesupport_introspection_c__get_const_function__RTKInfo__base_lla(
  const void * untyped_member, size_t index)
{
  const float * member =
    (const float *)(untyped_member);
  return &member[index];
}

void * inertial_sense_ros2__msg__RTKInfo__rosidl_typesupport_introspection_c__get_function__RTKInfo__base_lla(
  void * untyped_member, size_t index)
{
  float * member =
    (float *)(untyped_member);
  return &member[index];
}

void inertial_sense_ros2__msg__RTKInfo__rosidl_typesupport_introspection_c__fetch_function__RTKInfo__base_lla(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const float * item =
    ((const float *)
    inertial_sense_ros2__msg__RTKInfo__rosidl_typesupport_introspection_c__get_const_function__RTKInfo__base_lla(untyped_member, index));
  float * value =
    (float *)(untyped_value);
  *value = *item;
}

void inertial_sense_ros2__msg__RTKInfo__rosidl_typesupport_introspection_c__assign_function__RTKInfo__base_lla(
  void * untyped_member, size_t index, const void * untyped_value)
{
  float * item =
    ((float *)
    inertial_sense_ros2__msg__RTKInfo__rosidl_typesupport_introspection_c__get_function__RTKInfo__base_lla(untyped_member, index));
  const float * value =
    (const float *)(untyped_value);
  *item = *value;
}

static rosidl_typesupport_introspection_c__MessageMember inertial_sense_ros2__msg__RTKInfo__rosidl_typesupport_introspection_c__RTKInfo_message_member_array[8] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(inertial_sense_ros2__msg__RTKInfo, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "base_lla",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    true,  // is array
    3,  // array size
    false,  // is upper bound
    offsetof(inertial_sense_ros2__msg__RTKInfo, base_lla),  // bytes offset in struct
    NULL,  // default value
    inertial_sense_ros2__msg__RTKInfo__rosidl_typesupport_introspection_c__size_function__RTKInfo__base_lla,  // size() function pointer
    inertial_sense_ros2__msg__RTKInfo__rosidl_typesupport_introspection_c__get_const_function__RTKInfo__base_lla,  // get_const(index) function pointer
    inertial_sense_ros2__msg__RTKInfo__rosidl_typesupport_introspection_c__get_function__RTKInfo__base_lla,  // get(index) function pointer
    inertial_sense_ros2__msg__RTKInfo__rosidl_typesupport_introspection_c__fetch_function__RTKInfo__base_lla,  // fetch(index, &value) function pointer
    inertial_sense_ros2__msg__RTKInfo__rosidl_typesupport_introspection_c__assign_function__RTKInfo__base_lla,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "cycle_slip_count",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(inertial_sense_ros2__msg__RTKInfo, cycle_slip_count),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "rover_obs",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(inertial_sense_ros2__msg__RTKInfo, rover_obs),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "base_obs",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(inertial_sense_ros2__msg__RTKInfo, base_obs),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "rover_eph",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(inertial_sense_ros2__msg__RTKInfo, rover_eph),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "base_eph",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(inertial_sense_ros2__msg__RTKInfo, base_eph),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "base_ant_count",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(inertial_sense_ros2__msg__RTKInfo, base_ant_count),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers inertial_sense_ros2__msg__RTKInfo__rosidl_typesupport_introspection_c__RTKInfo_message_members = {
  "inertial_sense_ros2__msg",  // message namespace
  "RTKInfo",  // message name
  8,  // number of fields
  sizeof(inertial_sense_ros2__msg__RTKInfo),
  false,  // has_any_key_member_
  inertial_sense_ros2__msg__RTKInfo__rosidl_typesupport_introspection_c__RTKInfo_message_member_array,  // message members
  inertial_sense_ros2__msg__RTKInfo__rosidl_typesupport_introspection_c__RTKInfo_init_function,  // function to initialize message memory (memory has to be allocated)
  inertial_sense_ros2__msg__RTKInfo__rosidl_typesupport_introspection_c__RTKInfo_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t inertial_sense_ros2__msg__RTKInfo__rosidl_typesupport_introspection_c__RTKInfo_message_type_support_handle = {
  0,
  &inertial_sense_ros2__msg__RTKInfo__rosidl_typesupport_introspection_c__RTKInfo_message_members,
  get_message_typesupport_handle_function,
  &inertial_sense_ros2__msg__RTKInfo__get_type_hash,
  &inertial_sense_ros2__msg__RTKInfo__get_type_description,
  &inertial_sense_ros2__msg__RTKInfo__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_inertial_sense_ros2
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, inertial_sense_ros2, msg, RTKInfo)() {
  inertial_sense_ros2__msg__RTKInfo__rosidl_typesupport_introspection_c__RTKInfo_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  if (!inertial_sense_ros2__msg__RTKInfo__rosidl_typesupport_introspection_c__RTKInfo_message_type_support_handle.typesupport_identifier) {
    inertial_sense_ros2__msg__RTKInfo__rosidl_typesupport_introspection_c__RTKInfo_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &inertial_sense_ros2__msg__RTKInfo__rosidl_typesupport_introspection_c__RTKInfo_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
