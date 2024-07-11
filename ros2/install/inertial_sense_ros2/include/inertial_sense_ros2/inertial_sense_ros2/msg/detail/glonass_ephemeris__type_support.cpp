// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from inertial_sense_ros2:msg/GlonassEphemeris.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "inertial_sense_ros2/msg/detail/glonass_ephemeris__functions.h"
#include "inertial_sense_ros2/msg/detail/glonass_ephemeris__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace inertial_sense_ros2
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void GlonassEphemeris_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) inertial_sense_ros2::msg::GlonassEphemeris(_init);
}

void GlonassEphemeris_fini_function(void * message_memory)
{
  auto typed_message = static_cast<inertial_sense_ros2::msg::GlonassEphemeris *>(message_memory);
  typed_message->~GlonassEphemeris();
}

size_t size_function__GlonassEphemeris__pos(const void * untyped_member)
{
  (void)untyped_member;
  return 3;
}

const void * get_const_function__GlonassEphemeris__pos(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<double, 3> *>(untyped_member);
  return &member[index];
}

void * get_function__GlonassEphemeris__pos(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<double, 3> *>(untyped_member);
  return &member[index];
}

void fetch_function__GlonassEphemeris__pos(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const double *>(
    get_const_function__GlonassEphemeris__pos(untyped_member, index));
  auto & value = *reinterpret_cast<double *>(untyped_value);
  value = item;
}

void assign_function__GlonassEphemeris__pos(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<double *>(
    get_function__GlonassEphemeris__pos(untyped_member, index));
  const auto & value = *reinterpret_cast<const double *>(untyped_value);
  item = value;
}

size_t size_function__GlonassEphemeris__vel(const void * untyped_member)
{
  (void)untyped_member;
  return 3;
}

const void * get_const_function__GlonassEphemeris__vel(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<double, 3> *>(untyped_member);
  return &member[index];
}

void * get_function__GlonassEphemeris__vel(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<double, 3> *>(untyped_member);
  return &member[index];
}

void fetch_function__GlonassEphemeris__vel(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const double *>(
    get_const_function__GlonassEphemeris__vel(untyped_member, index));
  auto & value = *reinterpret_cast<double *>(untyped_value);
  value = item;
}

void assign_function__GlonassEphemeris__vel(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<double *>(
    get_function__GlonassEphemeris__vel(untyped_member, index));
  const auto & value = *reinterpret_cast<const double *>(untyped_value);
  item = value;
}

size_t size_function__GlonassEphemeris__acc(const void * untyped_member)
{
  (void)untyped_member;
  return 3;
}

const void * get_const_function__GlonassEphemeris__acc(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<double, 3> *>(untyped_member);
  return &member[index];
}

void * get_function__GlonassEphemeris__acc(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<double, 3> *>(untyped_member);
  return &member[index];
}

void fetch_function__GlonassEphemeris__acc(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const double *>(
    get_const_function__GlonassEphemeris__acc(untyped_member, index));
  auto & value = *reinterpret_cast<double *>(untyped_value);
  value = item;
}

void assign_function__GlonassEphemeris__acc(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<double *>(
    get_function__GlonassEphemeris__acc(untyped_member, index));
  const auto & value = *reinterpret_cast<const double *>(untyped_value);
  item = value;
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember GlonassEphemeris_message_member_array[14] = {
  {
    "sat",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(inertial_sense_ros2::msg::GlonassEphemeris, sat),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "iode",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(inertial_sense_ros2::msg::GlonassEphemeris, iode),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "frq",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(inertial_sense_ros2::msg::GlonassEphemeris, frq),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "svh",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(inertial_sense_ros2::msg::GlonassEphemeris, svh),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "sva",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(inertial_sense_ros2::msg::GlonassEphemeris, sva),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "age",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(inertial_sense_ros2::msg::GlonassEphemeris, age),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "toe",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<inertial_sense_ros2::msg::GTime>(),  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(inertial_sense_ros2::msg::GlonassEphemeris, toe),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "tof",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<inertial_sense_ros2::msg::GTime>(),  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(inertial_sense_ros2::msg::GlonassEphemeris, tof),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "pos",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    true,  // is array
    3,  // array size
    false,  // is upper bound
    offsetof(inertial_sense_ros2::msg::GlonassEphemeris, pos),  // bytes offset in struct
    nullptr,  // default value
    size_function__GlonassEphemeris__pos,  // size() function pointer
    get_const_function__GlonassEphemeris__pos,  // get_const(index) function pointer
    get_function__GlonassEphemeris__pos,  // get(index) function pointer
    fetch_function__GlonassEphemeris__pos,  // fetch(index, &value) function pointer
    assign_function__GlonassEphemeris__pos,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "vel",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    true,  // is array
    3,  // array size
    false,  // is upper bound
    offsetof(inertial_sense_ros2::msg::GlonassEphemeris, vel),  // bytes offset in struct
    nullptr,  // default value
    size_function__GlonassEphemeris__vel,  // size() function pointer
    get_const_function__GlonassEphemeris__vel,  // get_const(index) function pointer
    get_function__GlonassEphemeris__vel,  // get(index) function pointer
    fetch_function__GlonassEphemeris__vel,  // fetch(index, &value) function pointer
    assign_function__GlonassEphemeris__vel,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "acc",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    true,  // is array
    3,  // array size
    false,  // is upper bound
    offsetof(inertial_sense_ros2::msg::GlonassEphemeris, acc),  // bytes offset in struct
    nullptr,  // default value
    size_function__GlonassEphemeris__acc,  // size() function pointer
    get_const_function__GlonassEphemeris__acc,  // get_const(index) function pointer
    get_function__GlonassEphemeris__acc,  // get(index) function pointer
    fetch_function__GlonassEphemeris__acc,  // fetch(index, &value) function pointer
    assign_function__GlonassEphemeris__acc,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "taun",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(inertial_sense_ros2::msg::GlonassEphemeris, taun),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "gamn",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(inertial_sense_ros2::msg::GlonassEphemeris, gamn),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "dtaun",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(inertial_sense_ros2::msg::GlonassEphemeris, dtaun),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers GlonassEphemeris_message_members = {
  "inertial_sense_ros2::msg",  // message namespace
  "GlonassEphemeris",  // message name
  14,  // number of fields
  sizeof(inertial_sense_ros2::msg::GlonassEphemeris),
  false,  // has_any_key_member_
  GlonassEphemeris_message_member_array,  // message members
  GlonassEphemeris_init_function,  // function to initialize message memory (memory has to be allocated)
  GlonassEphemeris_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t GlonassEphemeris_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &GlonassEphemeris_message_members,
  get_message_typesupport_handle_function,
  &inertial_sense_ros2__msg__GlonassEphemeris__get_type_hash,
  &inertial_sense_ros2__msg__GlonassEphemeris__get_type_description,
  &inertial_sense_ros2__msg__GlonassEphemeris__get_type_description_sources,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace inertial_sense_ros2


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<inertial_sense_ros2::msg::GlonassEphemeris>()
{
  return &::inertial_sense_ros2::msg::rosidl_typesupport_introspection_cpp::GlonassEphemeris_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, inertial_sense_ros2, msg, GlonassEphemeris)() {
  return &::inertial_sense_ros2::msg::rosidl_typesupport_introspection_cpp::GlonassEphemeris_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
