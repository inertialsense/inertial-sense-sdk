// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from inertial_sense_ros2:msg/GTime.idl
// generated code does not contain a copyright notice

#include "inertial_sense_ros2/msg/detail/g_time__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_inertial_sense_ros2
const rosidl_type_hash_t *
inertial_sense_ros2__msg__GTime__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x4e, 0x16, 0x46, 0x23, 0xff, 0x6e, 0x2b, 0x23,
      0x0d, 0xd7, 0xa9, 0x4b, 0x62, 0x65, 0xed, 0x89,
      0x8f, 0x1a, 0xce, 0x3c, 0x8e, 0x12, 0x4b, 0x16,
      0x50, 0xdb, 0x7a, 0xb3, 0xc2, 0xb5, 0xa1, 0xa9,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char inertial_sense_ros2__msg__GTime__TYPE_NAME[] = "inertial_sense_ros2/msg/GTime";

// Define type names, field names, and default values
static char inertial_sense_ros2__msg__GTime__FIELD_NAME__time[] = "time";
static char inertial_sense_ros2__msg__GTime__FIELD_NAME__sec[] = "sec";

static rosidl_runtime_c__type_description__Field inertial_sense_ros2__msg__GTime__FIELDS[] = {
  {
    {inertial_sense_ros2__msg__GTime__FIELD_NAME__time, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {inertial_sense_ros2__msg__GTime__FIELD_NAME__sec, 3, 3},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
inertial_sense_ros2__msg__GTime__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {inertial_sense_ros2__msg__GTime__TYPE_NAME, 29, 29},
      {inertial_sense_ros2__msg__GTime__FIELDS, 2, 2},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "int64 time\n"
  "float64 sec";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
inertial_sense_ros2__msg__GTime__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {inertial_sense_ros2__msg__GTime__TYPE_NAME, 29, 29},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 22, 22},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
inertial_sense_ros2__msg__GTime__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *inertial_sense_ros2__msg__GTime__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
