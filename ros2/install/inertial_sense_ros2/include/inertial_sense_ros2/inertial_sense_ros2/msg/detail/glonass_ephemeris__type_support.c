// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from inertial_sense_ros2:msg/GlonassEphemeris.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "inertial_sense_ros2/msg/detail/glonass_ephemeris__rosidl_typesupport_introspection_c.h"
#include "inertial_sense_ros2/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "inertial_sense_ros2/msg/detail/glonass_ephemeris__functions.h"
#include "inertial_sense_ros2/msg/detail/glonass_ephemeris__struct.h"


// Include directives for member types
// Member `toe`
// Member `tof`
#include "inertial_sense_ros2/msg/g_time.h"
// Member `toe`
// Member `tof`
#include "inertial_sense_ros2/msg/detail/g_time__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void inertial_sense_ros2__msg__GlonassEphemeris__rosidl_typesupport_introspection_c__GlonassEphemeris_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  inertial_sense_ros2__msg__GlonassEphemeris__init(message_memory);
}

void inertial_sense_ros2__msg__GlonassEphemeris__rosidl_typesupport_introspection_c__GlonassEphemeris_fini_function(void * message_memory)
{
  inertial_sense_ros2__msg__GlonassEphemeris__fini(message_memory);
}

size_t inertial_sense_ros2__msg__GlonassEphemeris__rosidl_typesupport_introspection_c__size_function__GlonassEphemeris__pos(
  const void * untyped_member)
{
  (void)untyped_member;
  return 3;
}

const void * inertial_sense_ros2__msg__GlonassEphemeris__rosidl_typesupport_introspection_c__get_const_function__GlonassEphemeris__pos(
  const void * untyped_member, size_t index)
{
  const double * member =
    (const double *)(untyped_member);
  return &member[index];
}

void * inertial_sense_ros2__msg__GlonassEphemeris__rosidl_typesupport_introspection_c__get_function__GlonassEphemeris__pos(
  void * untyped_member, size_t index)
{
  double * member =
    (double *)(untyped_member);
  return &member[index];
}

void inertial_sense_ros2__msg__GlonassEphemeris__rosidl_typesupport_introspection_c__fetch_function__GlonassEphemeris__pos(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    inertial_sense_ros2__msg__GlonassEphemeris__rosidl_typesupport_introspection_c__get_const_function__GlonassEphemeris__pos(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void inertial_sense_ros2__msg__GlonassEphemeris__rosidl_typesupport_introspection_c__assign_function__GlonassEphemeris__pos(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    inertial_sense_ros2__msg__GlonassEphemeris__rosidl_typesupport_introspection_c__get_function__GlonassEphemeris__pos(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

size_t inertial_sense_ros2__msg__GlonassEphemeris__rosidl_typesupport_introspection_c__size_function__GlonassEphemeris__vel(
  const void * untyped_member)
{
  (void)untyped_member;
  return 3;
}

const void * inertial_sense_ros2__msg__GlonassEphemeris__rosidl_typesupport_introspection_c__get_const_function__GlonassEphemeris__vel(
  const void * untyped_member, size_t index)
{
  const double * member =
    (const double *)(untyped_member);
  return &member[index];
}

void * inertial_sense_ros2__msg__GlonassEphemeris__rosidl_typesupport_introspection_c__get_function__GlonassEphemeris__vel(
  void * untyped_member, size_t index)
{
  double * member =
    (double *)(untyped_member);
  return &member[index];
}

void inertial_sense_ros2__msg__GlonassEphemeris__rosidl_typesupport_introspection_c__fetch_function__GlonassEphemeris__vel(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    inertial_sense_ros2__msg__GlonassEphemeris__rosidl_typesupport_introspection_c__get_const_function__GlonassEphemeris__vel(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void inertial_sense_ros2__msg__GlonassEphemeris__rosidl_typesupport_introspection_c__assign_function__GlonassEphemeris__vel(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    inertial_sense_ros2__msg__GlonassEphemeris__rosidl_typesupport_introspection_c__get_function__GlonassEphemeris__vel(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

size_t inertial_sense_ros2__msg__GlonassEphemeris__rosidl_typesupport_introspection_c__size_function__GlonassEphemeris__acc(
  const void * untyped_member)
{
  (void)untyped_member;
  return 3;
}

const void * inertial_sense_ros2__msg__GlonassEphemeris__rosidl_typesupport_introspection_c__get_const_function__GlonassEphemeris__acc(
  const void * untyped_member, size_t index)
{
  const double * member =
    (const double *)(untyped_member);
  return &member[index];
}

void * inertial_sense_ros2__msg__GlonassEphemeris__rosidl_typesupport_introspection_c__get_function__GlonassEphemeris__acc(
  void * untyped_member, size_t index)
{
  double * member =
    (double *)(untyped_member);
  return &member[index];
}

void inertial_sense_ros2__msg__GlonassEphemeris__rosidl_typesupport_introspection_c__fetch_function__GlonassEphemeris__acc(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    inertial_sense_ros2__msg__GlonassEphemeris__rosidl_typesupport_introspection_c__get_const_function__GlonassEphemeris__acc(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void inertial_sense_ros2__msg__GlonassEphemeris__rosidl_typesupport_introspection_c__assign_function__GlonassEphemeris__acc(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    inertial_sense_ros2__msg__GlonassEphemeris__rosidl_typesupport_introspection_c__get_function__GlonassEphemeris__acc(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

static rosidl_typesupport_introspection_c__MessageMember inertial_sense_ros2__msg__GlonassEphemeris__rosidl_typesupport_introspection_c__GlonassEphemeris_message_member_array[14] = {
  {
    "sat",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(inertial_sense_ros2__msg__GlonassEphemeris, sat),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "iode",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(inertial_sense_ros2__msg__GlonassEphemeris, iode),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "frq",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(inertial_sense_ros2__msg__GlonassEphemeris, frq),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "svh",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(inertial_sense_ros2__msg__GlonassEphemeris, svh),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "sva",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(inertial_sense_ros2__msg__GlonassEphemeris, sva),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "age",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(inertial_sense_ros2__msg__GlonassEphemeris, age),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "toe",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(inertial_sense_ros2__msg__GlonassEphemeris, toe),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "tof",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(inertial_sense_ros2__msg__GlonassEphemeris, tof),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "pos",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    true,  // is array
    3,  // array size
    false,  // is upper bound
    offsetof(inertial_sense_ros2__msg__GlonassEphemeris, pos),  // bytes offset in struct
    NULL,  // default value
    inertial_sense_ros2__msg__GlonassEphemeris__rosidl_typesupport_introspection_c__size_function__GlonassEphemeris__pos,  // size() function pointer
    inertial_sense_ros2__msg__GlonassEphemeris__rosidl_typesupport_introspection_c__get_const_function__GlonassEphemeris__pos,  // get_const(index) function pointer
    inertial_sense_ros2__msg__GlonassEphemeris__rosidl_typesupport_introspection_c__get_function__GlonassEphemeris__pos,  // get(index) function pointer
    inertial_sense_ros2__msg__GlonassEphemeris__rosidl_typesupport_introspection_c__fetch_function__GlonassEphemeris__pos,  // fetch(index, &value) function pointer
    inertial_sense_ros2__msg__GlonassEphemeris__rosidl_typesupport_introspection_c__assign_function__GlonassEphemeris__pos,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "vel",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    true,  // is array
    3,  // array size
    false,  // is upper bound
    offsetof(inertial_sense_ros2__msg__GlonassEphemeris, vel),  // bytes offset in struct
    NULL,  // default value
    inertial_sense_ros2__msg__GlonassEphemeris__rosidl_typesupport_introspection_c__size_function__GlonassEphemeris__vel,  // size() function pointer
    inertial_sense_ros2__msg__GlonassEphemeris__rosidl_typesupport_introspection_c__get_const_function__GlonassEphemeris__vel,  // get_const(index) function pointer
    inertial_sense_ros2__msg__GlonassEphemeris__rosidl_typesupport_introspection_c__get_function__GlonassEphemeris__vel,  // get(index) function pointer
    inertial_sense_ros2__msg__GlonassEphemeris__rosidl_typesupport_introspection_c__fetch_function__GlonassEphemeris__vel,  // fetch(index, &value) function pointer
    inertial_sense_ros2__msg__GlonassEphemeris__rosidl_typesupport_introspection_c__assign_function__GlonassEphemeris__vel,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "acc",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    true,  // is array
    3,  // array size
    false,  // is upper bound
    offsetof(inertial_sense_ros2__msg__GlonassEphemeris, acc),  // bytes offset in struct
    NULL,  // default value
    inertial_sense_ros2__msg__GlonassEphemeris__rosidl_typesupport_introspection_c__size_function__GlonassEphemeris__acc,  // size() function pointer
    inertial_sense_ros2__msg__GlonassEphemeris__rosidl_typesupport_introspection_c__get_const_function__GlonassEphemeris__acc,  // get_const(index) function pointer
    inertial_sense_ros2__msg__GlonassEphemeris__rosidl_typesupport_introspection_c__get_function__GlonassEphemeris__acc,  // get(index) function pointer
    inertial_sense_ros2__msg__GlonassEphemeris__rosidl_typesupport_introspection_c__fetch_function__GlonassEphemeris__acc,  // fetch(index, &value) function pointer
    inertial_sense_ros2__msg__GlonassEphemeris__rosidl_typesupport_introspection_c__assign_function__GlonassEphemeris__acc,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "taun",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(inertial_sense_ros2__msg__GlonassEphemeris, taun),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "gamn",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(inertial_sense_ros2__msg__GlonassEphemeris, gamn),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "dtaun",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(inertial_sense_ros2__msg__GlonassEphemeris, dtaun),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers inertial_sense_ros2__msg__GlonassEphemeris__rosidl_typesupport_introspection_c__GlonassEphemeris_message_members = {
  "inertial_sense_ros2__msg",  // message namespace
  "GlonassEphemeris",  // message name
  14,  // number of fields
  sizeof(inertial_sense_ros2__msg__GlonassEphemeris),
  false,  // has_any_key_member_
  inertial_sense_ros2__msg__GlonassEphemeris__rosidl_typesupport_introspection_c__GlonassEphemeris_message_member_array,  // message members
  inertial_sense_ros2__msg__GlonassEphemeris__rosidl_typesupport_introspection_c__GlonassEphemeris_init_function,  // function to initialize message memory (memory has to be allocated)
  inertial_sense_ros2__msg__GlonassEphemeris__rosidl_typesupport_introspection_c__GlonassEphemeris_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t inertial_sense_ros2__msg__GlonassEphemeris__rosidl_typesupport_introspection_c__GlonassEphemeris_message_type_support_handle = {
  0,
  &inertial_sense_ros2__msg__GlonassEphemeris__rosidl_typesupport_introspection_c__GlonassEphemeris_message_members,
  get_message_typesupport_handle_function,
  &inertial_sense_ros2__msg__GlonassEphemeris__get_type_hash,
  &inertial_sense_ros2__msg__GlonassEphemeris__get_type_description,
  &inertial_sense_ros2__msg__GlonassEphemeris__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_inertial_sense_ros2
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, inertial_sense_ros2, msg, GlonassEphemeris)() {
  inertial_sense_ros2__msg__GlonassEphemeris__rosidl_typesupport_introspection_c__GlonassEphemeris_message_member_array[6].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, inertial_sense_ros2, msg, GTime)();
  inertial_sense_ros2__msg__GlonassEphemeris__rosidl_typesupport_introspection_c__GlonassEphemeris_message_member_array[7].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, inertial_sense_ros2, msg, GTime)();
  if (!inertial_sense_ros2__msg__GlonassEphemeris__rosidl_typesupport_introspection_c__GlonassEphemeris_message_type_support_handle.typesupport_identifier) {
    inertial_sense_ros2__msg__GlonassEphemeris__rosidl_typesupport_introspection_c__GlonassEphemeris_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &inertial_sense_ros2__msg__GlonassEphemeris__rosidl_typesupport_introspection_c__GlonassEphemeris_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
