// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from inertial_sense_ros2:msg/GNSSObsVec.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "inertial_sense_ros2/msg/detail/gnss_obs_vec__functions.h"
#include "inertial_sense_ros2/msg/detail/gnss_obs_vec__struct.hpp"
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

void GNSSObsVec_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) inertial_sense_ros2::msg::GNSSObsVec(_init);
}

void GNSSObsVec_fini_function(void * message_memory)
{
  auto typed_message = static_cast<inertial_sense_ros2::msg::GNSSObsVec *>(message_memory);
  typed_message->~GNSSObsVec();
}

size_t size_function__GNSSObsVec__obs(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<inertial_sense_ros2::msg::GNSSObservation> *>(untyped_member);
  return member->size();
}

const void * get_const_function__GNSSObsVec__obs(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<inertial_sense_ros2::msg::GNSSObservation> *>(untyped_member);
  return &member[index];
}

void * get_function__GNSSObsVec__obs(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<inertial_sense_ros2::msg::GNSSObservation> *>(untyped_member);
  return &member[index];
}

void fetch_function__GNSSObsVec__obs(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const inertial_sense_ros2::msg::GNSSObservation *>(
    get_const_function__GNSSObsVec__obs(untyped_member, index));
  auto & value = *reinterpret_cast<inertial_sense_ros2::msg::GNSSObservation *>(untyped_value);
  value = item;
}

void assign_function__GNSSObsVec__obs(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<inertial_sense_ros2::msg::GNSSObservation *>(
    get_function__GNSSObsVec__obs(untyped_member, index));
  const auto & value = *reinterpret_cast<const inertial_sense_ros2::msg::GNSSObservation *>(untyped_value);
  item = value;
}

void resize_function__GNSSObsVec__obs(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<inertial_sense_ros2::msg::GNSSObservation> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember GNSSObsVec_message_member_array[3] = {
  {
    "header",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<std_msgs::msg::Header>(),  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(inertial_sense_ros2::msg::GNSSObsVec, header),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "time",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<inertial_sense_ros2::msg::GTime>(),  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(inertial_sense_ros2::msg::GNSSObsVec, time),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "obs",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<inertial_sense_ros2::msg::GNSSObservation>(),  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(inertial_sense_ros2::msg::GNSSObsVec, obs),  // bytes offset in struct
    nullptr,  // default value
    size_function__GNSSObsVec__obs,  // size() function pointer
    get_const_function__GNSSObsVec__obs,  // get_const(index) function pointer
    get_function__GNSSObsVec__obs,  // get(index) function pointer
    fetch_function__GNSSObsVec__obs,  // fetch(index, &value) function pointer
    assign_function__GNSSObsVec__obs,  // assign(index, value) function pointer
    resize_function__GNSSObsVec__obs  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers GNSSObsVec_message_members = {
  "inertial_sense_ros2::msg",  // message namespace
  "GNSSObsVec",  // message name
  3,  // number of fields
  sizeof(inertial_sense_ros2::msg::GNSSObsVec),
  false,  // has_any_key_member_
  GNSSObsVec_message_member_array,  // message members
  GNSSObsVec_init_function,  // function to initialize message memory (memory has to be allocated)
  GNSSObsVec_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t GNSSObsVec_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &GNSSObsVec_message_members,
  get_message_typesupport_handle_function,
  &inertial_sense_ros2__msg__GNSSObsVec__get_type_hash,
  &inertial_sense_ros2__msg__GNSSObsVec__get_type_description,
  &inertial_sense_ros2__msg__GNSSObsVec__get_type_description_sources,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace inertial_sense_ros2


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<inertial_sense_ros2::msg::GNSSObsVec>()
{
  return &::inertial_sense_ros2::msg::rosidl_typesupport_introspection_cpp::GNSSObsVec_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, inertial_sense_ros2, msg, GNSSObsVec)() {
  return &::inertial_sense_ros2::msg::rosidl_typesupport_introspection_cpp::GNSSObsVec_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
