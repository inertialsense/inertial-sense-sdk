// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from inertial_sense_ros2:srv/FirmwareUpdate.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "inertial_sense_ros2/srv/detail/firmware_update__rosidl_typesupport_introspection_c.h"
#include "inertial_sense_ros2/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "inertial_sense_ros2/srv/detail/firmware_update__functions.h"
#include "inertial_sense_ros2/srv/detail/firmware_update__struct.h"


// Include directives for member types
// Member `filename`
#include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void inertial_sense_ros2__srv__FirmwareUpdate_Request__rosidl_typesupport_introspection_c__FirmwareUpdate_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  inertial_sense_ros2__srv__FirmwareUpdate_Request__init(message_memory);
}

void inertial_sense_ros2__srv__FirmwareUpdate_Request__rosidl_typesupport_introspection_c__FirmwareUpdate_Request_fini_function(void * message_memory)
{
  inertial_sense_ros2__srv__FirmwareUpdate_Request__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember inertial_sense_ros2__srv__FirmwareUpdate_Request__rosidl_typesupport_introspection_c__FirmwareUpdate_Request_message_member_array[1] = {
  {
    "filename",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(inertial_sense_ros2__srv__FirmwareUpdate_Request, filename),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers inertial_sense_ros2__srv__FirmwareUpdate_Request__rosidl_typesupport_introspection_c__FirmwareUpdate_Request_message_members = {
  "inertial_sense_ros2__srv",  // message namespace
  "FirmwareUpdate_Request",  // message name
  1,  // number of fields
  sizeof(inertial_sense_ros2__srv__FirmwareUpdate_Request),
  false,  // has_any_key_member_
  inertial_sense_ros2__srv__FirmwareUpdate_Request__rosidl_typesupport_introspection_c__FirmwareUpdate_Request_message_member_array,  // message members
  inertial_sense_ros2__srv__FirmwareUpdate_Request__rosidl_typesupport_introspection_c__FirmwareUpdate_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  inertial_sense_ros2__srv__FirmwareUpdate_Request__rosidl_typesupport_introspection_c__FirmwareUpdate_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t inertial_sense_ros2__srv__FirmwareUpdate_Request__rosidl_typesupport_introspection_c__FirmwareUpdate_Request_message_type_support_handle = {
  0,
  &inertial_sense_ros2__srv__FirmwareUpdate_Request__rosidl_typesupport_introspection_c__FirmwareUpdate_Request_message_members,
  get_message_typesupport_handle_function,
  &inertial_sense_ros2__srv__FirmwareUpdate_Request__get_type_hash,
  &inertial_sense_ros2__srv__FirmwareUpdate_Request__get_type_description,
  &inertial_sense_ros2__srv__FirmwareUpdate_Request__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_inertial_sense_ros2
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, inertial_sense_ros2, srv, FirmwareUpdate_Request)() {
  if (!inertial_sense_ros2__srv__FirmwareUpdate_Request__rosidl_typesupport_introspection_c__FirmwareUpdate_Request_message_type_support_handle.typesupport_identifier) {
    inertial_sense_ros2__srv__FirmwareUpdate_Request__rosidl_typesupport_introspection_c__FirmwareUpdate_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &inertial_sense_ros2__srv__FirmwareUpdate_Request__rosidl_typesupport_introspection_c__FirmwareUpdate_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "inertial_sense_ros2/srv/detail/firmware_update__rosidl_typesupport_introspection_c.h"
// already included above
// #include "inertial_sense_ros2/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "inertial_sense_ros2/srv/detail/firmware_update__functions.h"
// already included above
// #include "inertial_sense_ros2/srv/detail/firmware_update__struct.h"


// Include directives for member types
// Member `message`
// already included above
// #include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void inertial_sense_ros2__srv__FirmwareUpdate_Response__rosidl_typesupport_introspection_c__FirmwareUpdate_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  inertial_sense_ros2__srv__FirmwareUpdate_Response__init(message_memory);
}

void inertial_sense_ros2__srv__FirmwareUpdate_Response__rosidl_typesupport_introspection_c__FirmwareUpdate_Response_fini_function(void * message_memory)
{
  inertial_sense_ros2__srv__FirmwareUpdate_Response__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember inertial_sense_ros2__srv__FirmwareUpdate_Response__rosidl_typesupport_introspection_c__FirmwareUpdate_Response_message_member_array[2] = {
  {
    "success",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(inertial_sense_ros2__srv__FirmwareUpdate_Response, success),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "message",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(inertial_sense_ros2__srv__FirmwareUpdate_Response, message),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers inertial_sense_ros2__srv__FirmwareUpdate_Response__rosidl_typesupport_introspection_c__FirmwareUpdate_Response_message_members = {
  "inertial_sense_ros2__srv",  // message namespace
  "FirmwareUpdate_Response",  // message name
  2,  // number of fields
  sizeof(inertial_sense_ros2__srv__FirmwareUpdate_Response),
  false,  // has_any_key_member_
  inertial_sense_ros2__srv__FirmwareUpdate_Response__rosidl_typesupport_introspection_c__FirmwareUpdate_Response_message_member_array,  // message members
  inertial_sense_ros2__srv__FirmwareUpdate_Response__rosidl_typesupport_introspection_c__FirmwareUpdate_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  inertial_sense_ros2__srv__FirmwareUpdate_Response__rosidl_typesupport_introspection_c__FirmwareUpdate_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t inertial_sense_ros2__srv__FirmwareUpdate_Response__rosidl_typesupport_introspection_c__FirmwareUpdate_Response_message_type_support_handle = {
  0,
  &inertial_sense_ros2__srv__FirmwareUpdate_Response__rosidl_typesupport_introspection_c__FirmwareUpdate_Response_message_members,
  get_message_typesupport_handle_function,
  &inertial_sense_ros2__srv__FirmwareUpdate_Response__get_type_hash,
  &inertial_sense_ros2__srv__FirmwareUpdate_Response__get_type_description,
  &inertial_sense_ros2__srv__FirmwareUpdate_Response__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_inertial_sense_ros2
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, inertial_sense_ros2, srv, FirmwareUpdate_Response)() {
  if (!inertial_sense_ros2__srv__FirmwareUpdate_Response__rosidl_typesupport_introspection_c__FirmwareUpdate_Response_message_type_support_handle.typesupport_identifier) {
    inertial_sense_ros2__srv__FirmwareUpdate_Response__rosidl_typesupport_introspection_c__FirmwareUpdate_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &inertial_sense_ros2__srv__FirmwareUpdate_Response__rosidl_typesupport_introspection_c__FirmwareUpdate_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "inertial_sense_ros2/srv/detail/firmware_update__rosidl_typesupport_introspection_c.h"
// already included above
// #include "inertial_sense_ros2/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "inertial_sense_ros2/srv/detail/firmware_update__functions.h"
// already included above
// #include "inertial_sense_ros2/srv/detail/firmware_update__struct.h"


// Include directives for member types
// Member `info`
#include "service_msgs/msg/service_event_info.h"
// Member `info`
#include "service_msgs/msg/detail/service_event_info__rosidl_typesupport_introspection_c.h"
// Member `request`
// Member `response`
#include "inertial_sense_ros2/srv/firmware_update.h"
// Member `request`
// Member `response`
// already included above
// #include "inertial_sense_ros2/srv/detail/firmware_update__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void inertial_sense_ros2__srv__FirmwareUpdate_Event__rosidl_typesupport_introspection_c__FirmwareUpdate_Event_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  inertial_sense_ros2__srv__FirmwareUpdate_Event__init(message_memory);
}

void inertial_sense_ros2__srv__FirmwareUpdate_Event__rosidl_typesupport_introspection_c__FirmwareUpdate_Event_fini_function(void * message_memory)
{
  inertial_sense_ros2__srv__FirmwareUpdate_Event__fini(message_memory);
}

size_t inertial_sense_ros2__srv__FirmwareUpdate_Event__rosidl_typesupport_introspection_c__size_function__FirmwareUpdate_Event__request(
  const void * untyped_member)
{
  const inertial_sense_ros2__srv__FirmwareUpdate_Request__Sequence * member =
    (const inertial_sense_ros2__srv__FirmwareUpdate_Request__Sequence *)(untyped_member);
  return member->size;
}

const void * inertial_sense_ros2__srv__FirmwareUpdate_Event__rosidl_typesupport_introspection_c__get_const_function__FirmwareUpdate_Event__request(
  const void * untyped_member, size_t index)
{
  const inertial_sense_ros2__srv__FirmwareUpdate_Request__Sequence * member =
    (const inertial_sense_ros2__srv__FirmwareUpdate_Request__Sequence *)(untyped_member);
  return &member->data[index];
}

void * inertial_sense_ros2__srv__FirmwareUpdate_Event__rosidl_typesupport_introspection_c__get_function__FirmwareUpdate_Event__request(
  void * untyped_member, size_t index)
{
  inertial_sense_ros2__srv__FirmwareUpdate_Request__Sequence * member =
    (inertial_sense_ros2__srv__FirmwareUpdate_Request__Sequence *)(untyped_member);
  return &member->data[index];
}

void inertial_sense_ros2__srv__FirmwareUpdate_Event__rosidl_typesupport_introspection_c__fetch_function__FirmwareUpdate_Event__request(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const inertial_sense_ros2__srv__FirmwareUpdate_Request * item =
    ((const inertial_sense_ros2__srv__FirmwareUpdate_Request *)
    inertial_sense_ros2__srv__FirmwareUpdate_Event__rosidl_typesupport_introspection_c__get_const_function__FirmwareUpdate_Event__request(untyped_member, index));
  inertial_sense_ros2__srv__FirmwareUpdate_Request * value =
    (inertial_sense_ros2__srv__FirmwareUpdate_Request *)(untyped_value);
  *value = *item;
}

void inertial_sense_ros2__srv__FirmwareUpdate_Event__rosidl_typesupport_introspection_c__assign_function__FirmwareUpdate_Event__request(
  void * untyped_member, size_t index, const void * untyped_value)
{
  inertial_sense_ros2__srv__FirmwareUpdate_Request * item =
    ((inertial_sense_ros2__srv__FirmwareUpdate_Request *)
    inertial_sense_ros2__srv__FirmwareUpdate_Event__rosidl_typesupport_introspection_c__get_function__FirmwareUpdate_Event__request(untyped_member, index));
  const inertial_sense_ros2__srv__FirmwareUpdate_Request * value =
    (const inertial_sense_ros2__srv__FirmwareUpdate_Request *)(untyped_value);
  *item = *value;
}

bool inertial_sense_ros2__srv__FirmwareUpdate_Event__rosidl_typesupport_introspection_c__resize_function__FirmwareUpdate_Event__request(
  void * untyped_member, size_t size)
{
  inertial_sense_ros2__srv__FirmwareUpdate_Request__Sequence * member =
    (inertial_sense_ros2__srv__FirmwareUpdate_Request__Sequence *)(untyped_member);
  inertial_sense_ros2__srv__FirmwareUpdate_Request__Sequence__fini(member);
  return inertial_sense_ros2__srv__FirmwareUpdate_Request__Sequence__init(member, size);
}

size_t inertial_sense_ros2__srv__FirmwareUpdate_Event__rosidl_typesupport_introspection_c__size_function__FirmwareUpdate_Event__response(
  const void * untyped_member)
{
  const inertial_sense_ros2__srv__FirmwareUpdate_Response__Sequence * member =
    (const inertial_sense_ros2__srv__FirmwareUpdate_Response__Sequence *)(untyped_member);
  return member->size;
}

const void * inertial_sense_ros2__srv__FirmwareUpdate_Event__rosidl_typesupport_introspection_c__get_const_function__FirmwareUpdate_Event__response(
  const void * untyped_member, size_t index)
{
  const inertial_sense_ros2__srv__FirmwareUpdate_Response__Sequence * member =
    (const inertial_sense_ros2__srv__FirmwareUpdate_Response__Sequence *)(untyped_member);
  return &member->data[index];
}

void * inertial_sense_ros2__srv__FirmwareUpdate_Event__rosidl_typesupport_introspection_c__get_function__FirmwareUpdate_Event__response(
  void * untyped_member, size_t index)
{
  inertial_sense_ros2__srv__FirmwareUpdate_Response__Sequence * member =
    (inertial_sense_ros2__srv__FirmwareUpdate_Response__Sequence *)(untyped_member);
  return &member->data[index];
}

void inertial_sense_ros2__srv__FirmwareUpdate_Event__rosidl_typesupport_introspection_c__fetch_function__FirmwareUpdate_Event__response(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const inertial_sense_ros2__srv__FirmwareUpdate_Response * item =
    ((const inertial_sense_ros2__srv__FirmwareUpdate_Response *)
    inertial_sense_ros2__srv__FirmwareUpdate_Event__rosidl_typesupport_introspection_c__get_const_function__FirmwareUpdate_Event__response(untyped_member, index));
  inertial_sense_ros2__srv__FirmwareUpdate_Response * value =
    (inertial_sense_ros2__srv__FirmwareUpdate_Response *)(untyped_value);
  *value = *item;
}

void inertial_sense_ros2__srv__FirmwareUpdate_Event__rosidl_typesupport_introspection_c__assign_function__FirmwareUpdate_Event__response(
  void * untyped_member, size_t index, const void * untyped_value)
{
  inertial_sense_ros2__srv__FirmwareUpdate_Response * item =
    ((inertial_sense_ros2__srv__FirmwareUpdate_Response *)
    inertial_sense_ros2__srv__FirmwareUpdate_Event__rosidl_typesupport_introspection_c__get_function__FirmwareUpdate_Event__response(untyped_member, index));
  const inertial_sense_ros2__srv__FirmwareUpdate_Response * value =
    (const inertial_sense_ros2__srv__FirmwareUpdate_Response *)(untyped_value);
  *item = *value;
}

bool inertial_sense_ros2__srv__FirmwareUpdate_Event__rosidl_typesupport_introspection_c__resize_function__FirmwareUpdate_Event__response(
  void * untyped_member, size_t size)
{
  inertial_sense_ros2__srv__FirmwareUpdate_Response__Sequence * member =
    (inertial_sense_ros2__srv__FirmwareUpdate_Response__Sequence *)(untyped_member);
  inertial_sense_ros2__srv__FirmwareUpdate_Response__Sequence__fini(member);
  return inertial_sense_ros2__srv__FirmwareUpdate_Response__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember inertial_sense_ros2__srv__FirmwareUpdate_Event__rosidl_typesupport_introspection_c__FirmwareUpdate_Event_message_member_array[3] = {
  {
    "info",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(inertial_sense_ros2__srv__FirmwareUpdate_Event, info),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "request",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    true,  // is array
    1,  // array size
    true,  // is upper bound
    offsetof(inertial_sense_ros2__srv__FirmwareUpdate_Event, request),  // bytes offset in struct
    NULL,  // default value
    inertial_sense_ros2__srv__FirmwareUpdate_Event__rosidl_typesupport_introspection_c__size_function__FirmwareUpdate_Event__request,  // size() function pointer
    inertial_sense_ros2__srv__FirmwareUpdate_Event__rosidl_typesupport_introspection_c__get_const_function__FirmwareUpdate_Event__request,  // get_const(index) function pointer
    inertial_sense_ros2__srv__FirmwareUpdate_Event__rosidl_typesupport_introspection_c__get_function__FirmwareUpdate_Event__request,  // get(index) function pointer
    inertial_sense_ros2__srv__FirmwareUpdate_Event__rosidl_typesupport_introspection_c__fetch_function__FirmwareUpdate_Event__request,  // fetch(index, &value) function pointer
    inertial_sense_ros2__srv__FirmwareUpdate_Event__rosidl_typesupport_introspection_c__assign_function__FirmwareUpdate_Event__request,  // assign(index, value) function pointer
    inertial_sense_ros2__srv__FirmwareUpdate_Event__rosidl_typesupport_introspection_c__resize_function__FirmwareUpdate_Event__request  // resize(index) function pointer
  },
  {
    "response",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    true,  // is array
    1,  // array size
    true,  // is upper bound
    offsetof(inertial_sense_ros2__srv__FirmwareUpdate_Event, response),  // bytes offset in struct
    NULL,  // default value
    inertial_sense_ros2__srv__FirmwareUpdate_Event__rosidl_typesupport_introspection_c__size_function__FirmwareUpdate_Event__response,  // size() function pointer
    inertial_sense_ros2__srv__FirmwareUpdate_Event__rosidl_typesupport_introspection_c__get_const_function__FirmwareUpdate_Event__response,  // get_const(index) function pointer
    inertial_sense_ros2__srv__FirmwareUpdate_Event__rosidl_typesupport_introspection_c__get_function__FirmwareUpdate_Event__response,  // get(index) function pointer
    inertial_sense_ros2__srv__FirmwareUpdate_Event__rosidl_typesupport_introspection_c__fetch_function__FirmwareUpdate_Event__response,  // fetch(index, &value) function pointer
    inertial_sense_ros2__srv__FirmwareUpdate_Event__rosidl_typesupport_introspection_c__assign_function__FirmwareUpdate_Event__response,  // assign(index, value) function pointer
    inertial_sense_ros2__srv__FirmwareUpdate_Event__rosidl_typesupport_introspection_c__resize_function__FirmwareUpdate_Event__response  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers inertial_sense_ros2__srv__FirmwareUpdate_Event__rosidl_typesupport_introspection_c__FirmwareUpdate_Event_message_members = {
  "inertial_sense_ros2__srv",  // message namespace
  "FirmwareUpdate_Event",  // message name
  3,  // number of fields
  sizeof(inertial_sense_ros2__srv__FirmwareUpdate_Event),
  false,  // has_any_key_member_
  inertial_sense_ros2__srv__FirmwareUpdate_Event__rosidl_typesupport_introspection_c__FirmwareUpdate_Event_message_member_array,  // message members
  inertial_sense_ros2__srv__FirmwareUpdate_Event__rosidl_typesupport_introspection_c__FirmwareUpdate_Event_init_function,  // function to initialize message memory (memory has to be allocated)
  inertial_sense_ros2__srv__FirmwareUpdate_Event__rosidl_typesupport_introspection_c__FirmwareUpdate_Event_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t inertial_sense_ros2__srv__FirmwareUpdate_Event__rosidl_typesupport_introspection_c__FirmwareUpdate_Event_message_type_support_handle = {
  0,
  &inertial_sense_ros2__srv__FirmwareUpdate_Event__rosidl_typesupport_introspection_c__FirmwareUpdate_Event_message_members,
  get_message_typesupport_handle_function,
  &inertial_sense_ros2__srv__FirmwareUpdate_Event__get_type_hash,
  &inertial_sense_ros2__srv__FirmwareUpdate_Event__get_type_description,
  &inertial_sense_ros2__srv__FirmwareUpdate_Event__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_inertial_sense_ros2
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, inertial_sense_ros2, srv, FirmwareUpdate_Event)() {
  inertial_sense_ros2__srv__FirmwareUpdate_Event__rosidl_typesupport_introspection_c__FirmwareUpdate_Event_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, service_msgs, msg, ServiceEventInfo)();
  inertial_sense_ros2__srv__FirmwareUpdate_Event__rosidl_typesupport_introspection_c__FirmwareUpdate_Event_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, inertial_sense_ros2, srv, FirmwareUpdate_Request)();
  inertial_sense_ros2__srv__FirmwareUpdate_Event__rosidl_typesupport_introspection_c__FirmwareUpdate_Event_message_member_array[2].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, inertial_sense_ros2, srv, FirmwareUpdate_Response)();
  if (!inertial_sense_ros2__srv__FirmwareUpdate_Event__rosidl_typesupport_introspection_c__FirmwareUpdate_Event_message_type_support_handle.typesupport_identifier) {
    inertial_sense_ros2__srv__FirmwareUpdate_Event__rosidl_typesupport_introspection_c__FirmwareUpdate_Event_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &inertial_sense_ros2__srv__FirmwareUpdate_Event__rosidl_typesupport_introspection_c__FirmwareUpdate_Event_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "inertial_sense_ros2/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "inertial_sense_ros2/srv/detail/firmware_update__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers inertial_sense_ros2__srv__detail__firmware_update__rosidl_typesupport_introspection_c__FirmwareUpdate_service_members = {
  "inertial_sense_ros2__srv",  // service namespace
  "FirmwareUpdate",  // service name
  // the following fields are initialized below on first access
  NULL,  // request message
  // inertial_sense_ros2__srv__detail__firmware_update__rosidl_typesupport_introspection_c__FirmwareUpdate_Request_message_type_support_handle,
  NULL,  // response message
  // inertial_sense_ros2__srv__detail__firmware_update__rosidl_typesupport_introspection_c__FirmwareUpdate_Response_message_type_support_handle
  NULL  // event_message
  // inertial_sense_ros2__srv__detail__firmware_update__rosidl_typesupport_introspection_c__FirmwareUpdate_Response_message_type_support_handle
};


static rosidl_service_type_support_t inertial_sense_ros2__srv__detail__firmware_update__rosidl_typesupport_introspection_c__FirmwareUpdate_service_type_support_handle = {
  0,
  &inertial_sense_ros2__srv__detail__firmware_update__rosidl_typesupport_introspection_c__FirmwareUpdate_service_members,
  get_service_typesupport_handle_function,
  &inertial_sense_ros2__srv__FirmwareUpdate_Request__rosidl_typesupport_introspection_c__FirmwareUpdate_Request_message_type_support_handle,
  &inertial_sense_ros2__srv__FirmwareUpdate_Response__rosidl_typesupport_introspection_c__FirmwareUpdate_Response_message_type_support_handle,
  &inertial_sense_ros2__srv__FirmwareUpdate_Event__rosidl_typesupport_introspection_c__FirmwareUpdate_Event_message_type_support_handle,
  ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_CREATE_EVENT_MESSAGE_SYMBOL_NAME(
    rosidl_typesupport_c,
    inertial_sense_ros2,
    srv,
    FirmwareUpdate
  ),
  ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_DESTROY_EVENT_MESSAGE_SYMBOL_NAME(
    rosidl_typesupport_c,
    inertial_sense_ros2,
    srv,
    FirmwareUpdate
  ),
  &inertial_sense_ros2__srv__FirmwareUpdate__get_type_hash,
  &inertial_sense_ros2__srv__FirmwareUpdate__get_type_description,
  &inertial_sense_ros2__srv__FirmwareUpdate__get_type_description_sources,
};

// Forward declaration of message type support functions for service members
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, inertial_sense_ros2, srv, FirmwareUpdate_Request)(void);

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, inertial_sense_ros2, srv, FirmwareUpdate_Response)(void);

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, inertial_sense_ros2, srv, FirmwareUpdate_Event)(void);

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_inertial_sense_ros2
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, inertial_sense_ros2, srv, FirmwareUpdate)(void) {
  if (!inertial_sense_ros2__srv__detail__firmware_update__rosidl_typesupport_introspection_c__FirmwareUpdate_service_type_support_handle.typesupport_identifier) {
    inertial_sense_ros2__srv__detail__firmware_update__rosidl_typesupport_introspection_c__FirmwareUpdate_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)inertial_sense_ros2__srv__detail__firmware_update__rosidl_typesupport_introspection_c__FirmwareUpdate_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, inertial_sense_ros2, srv, FirmwareUpdate_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, inertial_sense_ros2, srv, FirmwareUpdate_Response)()->data;
  }
  if (!service_members->event_members_) {
    service_members->event_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, inertial_sense_ros2, srv, FirmwareUpdate_Event)()->data;
  }

  return &inertial_sense_ros2__srv__detail__firmware_update__rosidl_typesupport_introspection_c__FirmwareUpdate_service_type_support_handle;
}
