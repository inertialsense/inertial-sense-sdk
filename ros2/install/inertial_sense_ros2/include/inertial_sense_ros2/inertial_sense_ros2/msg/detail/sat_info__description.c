// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from inertial_sense_ros2:msg/SatInfo.idl
// generated code does not contain a copyright notice

#include "inertial_sense_ros2/msg/detail/sat_info__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_inertial_sense_ros2
const rosidl_type_hash_t *
inertial_sense_ros2__msg__SatInfo__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x98, 0xb3, 0x5c, 0x79, 0x68, 0x2b, 0xcb, 0x05,
      0xed, 0x1c, 0x96, 0x6c, 0x0b, 0x96, 0xc0, 0x9d,
      0x56, 0xd7, 0x1c, 0xab, 0xc0, 0x63, 0xc3, 0x55,
      0xf9, 0x46, 0x63, 0xd7, 0xf3, 0x30, 0x16, 0x6e,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char inertial_sense_ros2__msg__SatInfo__TYPE_NAME[] = "inertial_sense_ros2/msg/SatInfo";

// Define type names, field names, and default values
static char inertial_sense_ros2__msg__SatInfo__FIELD_NAME__sat_id[] = "sat_id";
static char inertial_sense_ros2__msg__SatInfo__FIELD_NAME__cno[] = "cno";

static rosidl_runtime_c__type_description__Field inertial_sense_ros2__msg__SatInfo__FIELDS[] = {
  {
    {inertial_sense_ros2__msg__SatInfo__FIELD_NAME__sat_id, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {inertial_sense_ros2__msg__SatInfo__FIELD_NAME__cno, 3, 3},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
inertial_sense_ros2__msg__SatInfo__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {inertial_sense_ros2__msg__SatInfo__TYPE_NAME, 31, 31},
      {inertial_sense_ros2__msg__SatInfo__FIELDS, 2, 2},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "uint32 sat_id # sattelite id\n"
  "uint32 cno    # Carrier to noise ratio";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
inertial_sense_ros2__msg__SatInfo__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {inertial_sense_ros2__msg__SatInfo__TYPE_NAME, 31, 31},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 67, 67},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
inertial_sense_ros2__msg__SatInfo__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *inertial_sense_ros2__msg__SatInfo__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
