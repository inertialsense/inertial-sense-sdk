// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from inertial_sense_ros2:msg/GPSInfo.idl
// generated code does not contain a copyright notice

#include "inertial_sense_ros2/msg/detail/gps_info__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_inertial_sense_ros2
const rosidl_type_hash_t *
inertial_sense_ros2__msg__GPSInfo__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x91, 0xed, 0x85, 0x9f, 0x07, 0x40, 0x3c, 0xa2,
      0xda, 0x5c, 0x8e, 0x1b, 0x2d, 0x71, 0xe2, 0xde,
      0xaf, 0x10, 0xf6, 0x26, 0x94, 0xe5, 0xe3, 0x50,
      0x5a, 0x53, 0x3b, 0xf7, 0x95, 0x9e, 0xac, 0x53,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types
#include "builtin_interfaces/msg/detail/time__functions.h"
#include "inertial_sense_ros2/msg/detail/sat_info__functions.h"
#include "std_msgs/msg/detail/header__functions.h"

// Hashes for external referenced types
#ifndef NDEBUG
static const rosidl_type_hash_t builtin_interfaces__msg__Time__EXPECTED_HASH = {1, {
    0xb1, 0x06, 0x23, 0x5e, 0x25, 0xa4, 0xc5, 0xed,
    0x35, 0x09, 0x8a, 0xa0, 0xa6, 0x1a, 0x3e, 0xe9,
    0xc9, 0xb1, 0x8d, 0x19, 0x7f, 0x39, 0x8b, 0x0e,
    0x42, 0x06, 0xce, 0xa9, 0xac, 0xf9, 0xc1, 0x97,
  }};
static const rosidl_type_hash_t inertial_sense_ros2__msg__SatInfo__EXPECTED_HASH = {1, {
    0x98, 0xb3, 0x5c, 0x79, 0x68, 0x2b, 0xcb, 0x05,
    0xed, 0x1c, 0x96, 0x6c, 0x0b, 0x96, 0xc0, 0x9d,
    0x56, 0xd7, 0x1c, 0xab, 0xc0, 0x63, 0xc3, 0x55,
    0xf9, 0x46, 0x63, 0xd7, 0xf3, 0x30, 0x16, 0x6e,
  }};
static const rosidl_type_hash_t std_msgs__msg__Header__EXPECTED_HASH = {1, {
    0xf4, 0x9f, 0xb3, 0xae, 0x2c, 0xf0, 0x70, 0xf7,
    0x93, 0x64, 0x5f, 0xf7, 0x49, 0x68, 0x3a, 0xc6,
    0xb0, 0x62, 0x03, 0xe4, 0x1c, 0x89, 0x1e, 0x17,
    0x70, 0x1b, 0x1c, 0xb5, 0x97, 0xce, 0x6a, 0x01,
  }};
#endif

static char inertial_sense_ros2__msg__GPSInfo__TYPE_NAME[] = "inertial_sense_ros2/msg/GPSInfo";
static char builtin_interfaces__msg__Time__TYPE_NAME[] = "builtin_interfaces/msg/Time";
static char inertial_sense_ros2__msg__SatInfo__TYPE_NAME[] = "inertial_sense_ros2/msg/SatInfo";
static char std_msgs__msg__Header__TYPE_NAME[] = "std_msgs/msg/Header";

// Define type names, field names, and default values
static char inertial_sense_ros2__msg__GPSInfo__FIELD_NAME__header[] = "header";
static char inertial_sense_ros2__msg__GPSInfo__FIELD_NAME__num_sats[] = "num_sats";
static char inertial_sense_ros2__msg__GPSInfo__FIELD_NAME__sattelite_info[] = "sattelite_info";

static rosidl_runtime_c__type_description__Field inertial_sense_ros2__msg__GPSInfo__FIELDS[] = {
  {
    {inertial_sense_ros2__msg__GPSInfo__FIELD_NAME__header, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {std_msgs__msg__Header__TYPE_NAME, 19, 19},
    },
    {NULL, 0, 0},
  },
  {
    {inertial_sense_ros2__msg__GPSInfo__FIELD_NAME__num_sats, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {inertial_sense_ros2__msg__GPSInfo__FIELD_NAME__sattelite_info, 14, 14},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_ARRAY,
      50,
      0,
      {inertial_sense_ros2__msg__SatInfo__TYPE_NAME, 31, 31},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription inertial_sense_ros2__msg__GPSInfo__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {inertial_sense_ros2__msg__SatInfo__TYPE_NAME, 31, 31},
    {NULL, 0, 0},
  },
  {
    {std_msgs__msg__Header__TYPE_NAME, 19, 19},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
inertial_sense_ros2__msg__GPSInfo__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {inertial_sense_ros2__msg__GPSInfo__TYPE_NAME, 31, 31},
      {inertial_sense_ros2__msg__GPSInfo__FIELDS, 3, 3},
    },
    {inertial_sense_ros2__msg__GPSInfo__REFERENCED_TYPE_DESCRIPTIONS, 3, 3},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&inertial_sense_ros2__msg__SatInfo__EXPECTED_HASH, inertial_sense_ros2__msg__SatInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[1].fields = inertial_sense_ros2__msg__SatInfo__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&std_msgs__msg__Header__EXPECTED_HASH, std_msgs__msg__Header__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[2].fields = std_msgs__msg__Header__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "\n"
  "std_msgs/Header header\n"
  "uint32 num_sats            \\t\\t# number of sattelites in the sky\n"
  "SatInfo[50] sattelite_info\\t \\t# information about sattelites";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
inertial_sense_ros2__msg__GPSInfo__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {inertial_sense_ros2__msg__GPSInfo__TYPE_NAME, 31, 31},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 147, 147},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
inertial_sense_ros2__msg__GPSInfo__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[4];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 4, 4};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *inertial_sense_ros2__msg__GPSInfo__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *inertial_sense_ros2__msg__SatInfo__get_individual_type_description_source(NULL);
    sources[3] = *std_msgs__msg__Header__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
