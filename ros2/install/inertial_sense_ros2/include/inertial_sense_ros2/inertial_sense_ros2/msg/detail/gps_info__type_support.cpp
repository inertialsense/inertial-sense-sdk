// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from inertial_sense_ros2:msg/GPSInfo.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "inertial_sense_ros2/msg/detail/gps_info__functions.h"
#include "inertial_sense_ros2/msg/detail/gps_info__struct.hpp"
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

void GPSInfo_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) inertial_sense_ros2::msg::GPSInfo(_init);
}

void GPSInfo_fini_function(void * message_memory)
{
  auto typed_message = static_cast<inertial_sense_ros2::msg::GPSInfo *>(message_memory);
  typed_message->~GPSInfo();
}

size_t size_function__GPSInfo__sattelite_info(const void * untyped_member)
{
  (void)untyped_member;
  return 50;
}

const void * get_const_function__GPSInfo__sattelite_info(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<inertial_sense_ros2::msg::SatInfo, 50> *>(untyped_member);
  return &member[index];
}

void * get_function__GPSInfo__sattelite_info(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<inertial_sense_ros2::msg::SatInfo, 50> *>(untyped_member);
  return &member[index];
}

void fetch_function__GPSInfo__sattelite_info(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const inertial_sense_ros2::msg::SatInfo *>(
    get_const_function__GPSInfo__sattelite_info(untyped_member, index));
  auto & value = *reinterpret_cast<inertial_sense_ros2::msg::SatInfo *>(untyped_value);
  value = item;
}

void assign_function__GPSInfo__sattelite_info(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<inertial_sense_ros2::msg::SatInfo *>(
    get_function__GPSInfo__sattelite_info(untyped_member, index));
  const auto & value = *reinterpret_cast<const inertial_sense_ros2::msg::SatInfo *>(untyped_value);
  item = value;
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember GPSInfo_message_member_array[3] = {
  {
    "header",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<std_msgs::msg::Header>(),  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(inertial_sense_ros2::msg::GPSInfo, header),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "num_sats",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(inertial_sense_ros2::msg::GPSInfo, num_sats),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "sattelite_info",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<inertial_sense_ros2::msg::SatInfo>(),  // members of sub message
    false,  // is key
    true,  // is array
    50,  // array size
    false,  // is upper bound
    offsetof(inertial_sense_ros2::msg::GPSInfo, sattelite_info),  // bytes offset in struct
    nullptr,  // default value
    size_function__GPSInfo__sattelite_info,  // size() function pointer
    get_const_function__GPSInfo__sattelite_info,  // get_const(index) function pointer
    get_function__GPSInfo__sattelite_info,  // get(index) function pointer
    fetch_function__GPSInfo__sattelite_info,  // fetch(index, &value) function pointer
    assign_function__GPSInfo__sattelite_info,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers GPSInfo_message_members = {
  "inertial_sense_ros2::msg",  // message namespace
  "GPSInfo",  // message name
  3,  // number of fields
  sizeof(inertial_sense_ros2::msg::GPSInfo),
  false,  // has_any_key_member_
  GPSInfo_message_member_array,  // message members
  GPSInfo_init_function,  // function to initialize message memory (memory has to be allocated)
  GPSInfo_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t GPSInfo_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &GPSInfo_message_members,
  get_message_typesupport_handle_function,
  &inertial_sense_ros2__msg__GPSInfo__get_type_hash,
  &inertial_sense_ros2__msg__GPSInfo__get_type_description,
  &inertial_sense_ros2__msg__GPSInfo__get_type_description_sources,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace inertial_sense_ros2


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<inertial_sense_ros2::msg::GPSInfo>()
{
  return &::inertial_sense_ros2::msg::rosidl_typesupport_introspection_cpp::GPSInfo_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, inertial_sense_ros2, msg, GPSInfo)() {
  return &::inertial_sense_ros2::msg::rosidl_typesupport_introspection_cpp::GPSInfo_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
