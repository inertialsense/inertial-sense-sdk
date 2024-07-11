// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from inertial_sense_ros2:msg/DIDINS1.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "inertial_sense_ros2/msg/detail/didins1__rosidl_typesupport_introspection_c.h"
#include "inertial_sense_ros2/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "inertial_sense_ros2/msg/detail/didins1__functions.h"
#include "inertial_sense_ros2/msg/detail/didins1__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void inertial_sense_ros2__msg__DIDINS1__rosidl_typesupport_introspection_c__DIDINS1_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  inertial_sense_ros2__msg__DIDINS1__init(message_memory);
}

void inertial_sense_ros2__msg__DIDINS1__rosidl_typesupport_introspection_c__DIDINS1_fini_function(void * message_memory)
{
  inertial_sense_ros2__msg__DIDINS1__fini(message_memory);
}

size_t inertial_sense_ros2__msg__DIDINS1__rosidl_typesupport_introspection_c__size_function__DIDINS1__theta(
  const void * untyped_member)
{
  (void)untyped_member;
  return 3;
}

const void * inertial_sense_ros2__msg__DIDINS1__rosidl_typesupport_introspection_c__get_const_function__DIDINS1__theta(
  const void * untyped_member, size_t index)
{
  const float * member =
    (const float *)(untyped_member);
  return &member[index];
}

void * inertial_sense_ros2__msg__DIDINS1__rosidl_typesupport_introspection_c__get_function__DIDINS1__theta(
  void * untyped_member, size_t index)
{
  float * member =
    (float *)(untyped_member);
  return &member[index];
}

void inertial_sense_ros2__msg__DIDINS1__rosidl_typesupport_introspection_c__fetch_function__DIDINS1__theta(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const float * item =
    ((const float *)
    inertial_sense_ros2__msg__DIDINS1__rosidl_typesupport_introspection_c__get_const_function__DIDINS1__theta(untyped_member, index));
  float * value =
    (float *)(untyped_value);
  *value = *item;
}

void inertial_sense_ros2__msg__DIDINS1__rosidl_typesupport_introspection_c__assign_function__DIDINS1__theta(
  void * untyped_member, size_t index, const void * untyped_value)
{
  float * item =
    ((float *)
    inertial_sense_ros2__msg__DIDINS1__rosidl_typesupport_introspection_c__get_function__DIDINS1__theta(untyped_member, index));
  const float * value =
    (const float *)(untyped_value);
  *item = *value;
}

size_t inertial_sense_ros2__msg__DIDINS1__rosidl_typesupport_introspection_c__size_function__DIDINS1__uvw(
  const void * untyped_member)
{
  (void)untyped_member;
  return 3;
}

const void * inertial_sense_ros2__msg__DIDINS1__rosidl_typesupport_introspection_c__get_const_function__DIDINS1__uvw(
  const void * untyped_member, size_t index)
{
  const float * member =
    (const float *)(untyped_member);
  return &member[index];
}

void * inertial_sense_ros2__msg__DIDINS1__rosidl_typesupport_introspection_c__get_function__DIDINS1__uvw(
  void * untyped_member, size_t index)
{
  float * member =
    (float *)(untyped_member);
  return &member[index];
}

void inertial_sense_ros2__msg__DIDINS1__rosidl_typesupport_introspection_c__fetch_function__DIDINS1__uvw(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const float * item =
    ((const float *)
    inertial_sense_ros2__msg__DIDINS1__rosidl_typesupport_introspection_c__get_const_function__DIDINS1__uvw(untyped_member, index));
  float * value =
    (float *)(untyped_value);
  *value = *item;
}

void inertial_sense_ros2__msg__DIDINS1__rosidl_typesupport_introspection_c__assign_function__DIDINS1__uvw(
  void * untyped_member, size_t index, const void * untyped_value)
{
  float * item =
    ((float *)
    inertial_sense_ros2__msg__DIDINS1__rosidl_typesupport_introspection_c__get_function__DIDINS1__uvw(untyped_member, index));
  const float * value =
    (const float *)(untyped_value);
  *item = *value;
}

size_t inertial_sense_ros2__msg__DIDINS1__rosidl_typesupport_introspection_c__size_function__DIDINS1__lla(
  const void * untyped_member)
{
  (void)untyped_member;
  return 3;
}

const void * inertial_sense_ros2__msg__DIDINS1__rosidl_typesupport_introspection_c__get_const_function__DIDINS1__lla(
  const void * untyped_member, size_t index)
{
  const double * member =
    (const double *)(untyped_member);
  return &member[index];
}

void * inertial_sense_ros2__msg__DIDINS1__rosidl_typesupport_introspection_c__get_function__DIDINS1__lla(
  void * untyped_member, size_t index)
{
  double * member =
    (double *)(untyped_member);
  return &member[index];
}

void inertial_sense_ros2__msg__DIDINS1__rosidl_typesupport_introspection_c__fetch_function__DIDINS1__lla(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    inertial_sense_ros2__msg__DIDINS1__rosidl_typesupport_introspection_c__get_const_function__DIDINS1__lla(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void inertial_sense_ros2__msg__DIDINS1__rosidl_typesupport_introspection_c__assign_function__DIDINS1__lla(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    inertial_sense_ros2__msg__DIDINS1__rosidl_typesupport_introspection_c__get_function__DIDINS1__lla(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

size_t inertial_sense_ros2__msg__DIDINS1__rosidl_typesupport_introspection_c__size_function__DIDINS1__ned(
  const void * untyped_member)
{
  (void)untyped_member;
  return 3;
}

const void * inertial_sense_ros2__msg__DIDINS1__rosidl_typesupport_introspection_c__get_const_function__DIDINS1__ned(
  const void * untyped_member, size_t index)
{
  const float * member =
    (const float *)(untyped_member);
  return &member[index];
}

void * inertial_sense_ros2__msg__DIDINS1__rosidl_typesupport_introspection_c__get_function__DIDINS1__ned(
  void * untyped_member, size_t index)
{
  float * member =
    (float *)(untyped_member);
  return &member[index];
}

void inertial_sense_ros2__msg__DIDINS1__rosidl_typesupport_introspection_c__fetch_function__DIDINS1__ned(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const float * item =
    ((const float *)
    inertial_sense_ros2__msg__DIDINS1__rosidl_typesupport_introspection_c__get_const_function__DIDINS1__ned(untyped_member, index));
  float * value =
    (float *)(untyped_value);
  *value = *item;
}

void inertial_sense_ros2__msg__DIDINS1__rosidl_typesupport_introspection_c__assign_function__DIDINS1__ned(
  void * untyped_member, size_t index, const void * untyped_value)
{
  float * item =
    ((float *)
    inertial_sense_ros2__msg__DIDINS1__rosidl_typesupport_introspection_c__get_function__DIDINS1__ned(untyped_member, index));
  const float * value =
    (const float *)(untyped_value);
  *item = *value;
}

static rosidl_typesupport_introspection_c__MessageMember inertial_sense_ros2__msg__DIDINS1__rosidl_typesupport_introspection_c__DIDINS1_message_member_array[9] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(inertial_sense_ros2__msg__DIDINS1, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "week",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(inertial_sense_ros2__msg__DIDINS1, week),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "time_of_week",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(inertial_sense_ros2__msg__DIDINS1, time_of_week),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "ins_status",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(inertial_sense_ros2__msg__DIDINS1, ins_status),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "hdw_status",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(inertial_sense_ros2__msg__DIDINS1, hdw_status),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "theta",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    true,  // is array
    3,  // array size
    false,  // is upper bound
    offsetof(inertial_sense_ros2__msg__DIDINS1, theta),  // bytes offset in struct
    NULL,  // default value
    inertial_sense_ros2__msg__DIDINS1__rosidl_typesupport_introspection_c__size_function__DIDINS1__theta,  // size() function pointer
    inertial_sense_ros2__msg__DIDINS1__rosidl_typesupport_introspection_c__get_const_function__DIDINS1__theta,  // get_const(index) function pointer
    inertial_sense_ros2__msg__DIDINS1__rosidl_typesupport_introspection_c__get_function__DIDINS1__theta,  // get(index) function pointer
    inertial_sense_ros2__msg__DIDINS1__rosidl_typesupport_introspection_c__fetch_function__DIDINS1__theta,  // fetch(index, &value) function pointer
    inertial_sense_ros2__msg__DIDINS1__rosidl_typesupport_introspection_c__assign_function__DIDINS1__theta,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "uvw",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    true,  // is array
    3,  // array size
    false,  // is upper bound
    offsetof(inertial_sense_ros2__msg__DIDINS1, uvw),  // bytes offset in struct
    NULL,  // default value
    inertial_sense_ros2__msg__DIDINS1__rosidl_typesupport_introspection_c__size_function__DIDINS1__uvw,  // size() function pointer
    inertial_sense_ros2__msg__DIDINS1__rosidl_typesupport_introspection_c__get_const_function__DIDINS1__uvw,  // get_const(index) function pointer
    inertial_sense_ros2__msg__DIDINS1__rosidl_typesupport_introspection_c__get_function__DIDINS1__uvw,  // get(index) function pointer
    inertial_sense_ros2__msg__DIDINS1__rosidl_typesupport_introspection_c__fetch_function__DIDINS1__uvw,  // fetch(index, &value) function pointer
    inertial_sense_ros2__msg__DIDINS1__rosidl_typesupport_introspection_c__assign_function__DIDINS1__uvw,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "lla",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    true,  // is array
    3,  // array size
    false,  // is upper bound
    offsetof(inertial_sense_ros2__msg__DIDINS1, lla),  // bytes offset in struct
    NULL,  // default value
    inertial_sense_ros2__msg__DIDINS1__rosidl_typesupport_introspection_c__size_function__DIDINS1__lla,  // size() function pointer
    inertial_sense_ros2__msg__DIDINS1__rosidl_typesupport_introspection_c__get_const_function__DIDINS1__lla,  // get_const(index) function pointer
    inertial_sense_ros2__msg__DIDINS1__rosidl_typesupport_introspection_c__get_function__DIDINS1__lla,  // get(index) function pointer
    inertial_sense_ros2__msg__DIDINS1__rosidl_typesupport_introspection_c__fetch_function__DIDINS1__lla,  // fetch(index, &value) function pointer
    inertial_sense_ros2__msg__DIDINS1__rosidl_typesupport_introspection_c__assign_function__DIDINS1__lla,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "ned",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    true,  // is array
    3,  // array size
    false,  // is upper bound
    offsetof(inertial_sense_ros2__msg__DIDINS1, ned),  // bytes offset in struct
    NULL,  // default value
    inertial_sense_ros2__msg__DIDINS1__rosidl_typesupport_introspection_c__size_function__DIDINS1__ned,  // size() function pointer
    inertial_sense_ros2__msg__DIDINS1__rosidl_typesupport_introspection_c__get_const_function__DIDINS1__ned,  // get_const(index) function pointer
    inertial_sense_ros2__msg__DIDINS1__rosidl_typesupport_introspection_c__get_function__DIDINS1__ned,  // get(index) function pointer
    inertial_sense_ros2__msg__DIDINS1__rosidl_typesupport_introspection_c__fetch_function__DIDINS1__ned,  // fetch(index, &value) function pointer
    inertial_sense_ros2__msg__DIDINS1__rosidl_typesupport_introspection_c__assign_function__DIDINS1__ned,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers inertial_sense_ros2__msg__DIDINS1__rosidl_typesupport_introspection_c__DIDINS1_message_members = {
  "inertial_sense_ros2__msg",  // message namespace
  "DIDINS1",  // message name
  9,  // number of fields
  sizeof(inertial_sense_ros2__msg__DIDINS1),
  false,  // has_any_key_member_
  inertial_sense_ros2__msg__DIDINS1__rosidl_typesupport_introspection_c__DIDINS1_message_member_array,  // message members
  inertial_sense_ros2__msg__DIDINS1__rosidl_typesupport_introspection_c__DIDINS1_init_function,  // function to initialize message memory (memory has to be allocated)
  inertial_sense_ros2__msg__DIDINS1__rosidl_typesupport_introspection_c__DIDINS1_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t inertial_sense_ros2__msg__DIDINS1__rosidl_typesupport_introspection_c__DIDINS1_message_type_support_handle = {
  0,
  &inertial_sense_ros2__msg__DIDINS1__rosidl_typesupport_introspection_c__DIDINS1_message_members,
  get_message_typesupport_handle_function,
  &inertial_sense_ros2__msg__DIDINS1__get_type_hash,
  &inertial_sense_ros2__msg__DIDINS1__get_type_description,
  &inertial_sense_ros2__msg__DIDINS1__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_inertial_sense_ros2
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, inertial_sense_ros2, msg, DIDINS1)() {
  inertial_sense_ros2__msg__DIDINS1__rosidl_typesupport_introspection_c__DIDINS1_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  if (!inertial_sense_ros2__msg__DIDINS1__rosidl_typesupport_introspection_c__DIDINS1_message_type_support_handle.typesupport_identifier) {
    inertial_sense_ros2__msg__DIDINS1__rosidl_typesupport_introspection_c__DIDINS1_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &inertial_sense_ros2__msg__DIDINS1__rosidl_typesupport_introspection_c__DIDINS1_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
