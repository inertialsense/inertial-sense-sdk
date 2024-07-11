// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from inertial_sense_ros2:msg/RTKInfo.idl
// generated code does not contain a copyright notice

#include "inertial_sense_ros2/msg/detail/rtk_info__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_inertial_sense_ros2
const rosidl_type_hash_t *
inertial_sense_ros2__msg__RTKInfo__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x8c, 0xe8, 0x10, 0xb6, 0x41, 0xf8, 0x1b, 0x1e,
      0xf6, 0xbf, 0x67, 0xbc, 0x37, 0x2a, 0xdb, 0xa4,
      0x24, 0x43, 0x8e, 0x9b, 0x77, 0x18, 0x2c, 0x62,
      0x26, 0x83, 0xb4, 0x1e, 0x02, 0xab, 0x4d, 0x2c,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types
#include "builtin_interfaces/msg/detail/time__functions.h"
#include "std_msgs/msg/detail/header__functions.h"

// Hashes for external referenced types
#ifndef NDEBUG
static const rosidl_type_hash_t builtin_interfaces__msg__Time__EXPECTED_HASH = {1, {
    0xb1, 0x06, 0x23, 0x5e, 0x25, 0xa4, 0xc5, 0xed,
    0x35, 0x09, 0x8a, 0xa0, 0xa6, 0x1a, 0x3e, 0xe9,
    0xc9, 0xb1, 0x8d, 0x19, 0x7f, 0x39, 0x8b, 0x0e,
    0x42, 0x06, 0xce, 0xa9, 0xac, 0xf9, 0xc1, 0x97,
  }};
static const rosidl_type_hash_t std_msgs__msg__Header__EXPECTED_HASH = {1, {
    0xf4, 0x9f, 0xb3, 0xae, 0x2c, 0xf0, 0x70, 0xf7,
    0x93, 0x64, 0x5f, 0xf7, 0x49, 0x68, 0x3a, 0xc6,
    0xb0, 0x62, 0x03, 0xe4, 0x1c, 0x89, 0x1e, 0x17,
    0x70, 0x1b, 0x1c, 0xb5, 0x97, 0xce, 0x6a, 0x01,
  }};
#endif

static char inertial_sense_ros2__msg__RTKInfo__TYPE_NAME[] = "inertial_sense_ros2/msg/RTKInfo";
static char builtin_interfaces__msg__Time__TYPE_NAME[] = "builtin_interfaces/msg/Time";
static char std_msgs__msg__Header__TYPE_NAME[] = "std_msgs/msg/Header";

// Define type names, field names, and default values
static char inertial_sense_ros2__msg__RTKInfo__FIELD_NAME__header[] = "header";
static char inertial_sense_ros2__msg__RTKInfo__FIELD_NAME__base_lla[] = "base_lla";
static char inertial_sense_ros2__msg__RTKInfo__FIELD_NAME__cycle_slip_count[] = "cycle_slip_count";
static char inertial_sense_ros2__msg__RTKInfo__FIELD_NAME__rover_obs[] = "rover_obs";
static char inertial_sense_ros2__msg__RTKInfo__FIELD_NAME__base_obs[] = "base_obs";
static char inertial_sense_ros2__msg__RTKInfo__FIELD_NAME__rover_eph[] = "rover_eph";
static char inertial_sense_ros2__msg__RTKInfo__FIELD_NAME__base_eph[] = "base_eph";
static char inertial_sense_ros2__msg__RTKInfo__FIELD_NAME__base_ant_count[] = "base_ant_count";

static rosidl_runtime_c__type_description__Field inertial_sense_ros2__msg__RTKInfo__FIELDS[] = {
  {
    {inertial_sense_ros2__msg__RTKInfo__FIELD_NAME__header, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {std_msgs__msg__Header__TYPE_NAME, 19, 19},
    },
    {NULL, 0, 0},
  },
  {
    {inertial_sense_ros2__msg__RTKInfo__FIELD_NAME__base_lla, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT_ARRAY,
      3,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {inertial_sense_ros2__msg__RTKInfo__FIELD_NAME__cycle_slip_count, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {inertial_sense_ros2__msg__RTKInfo__FIELD_NAME__rover_obs, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {inertial_sense_ros2__msg__RTKInfo__FIELD_NAME__base_obs, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {inertial_sense_ros2__msg__RTKInfo__FIELD_NAME__rover_eph, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {inertial_sense_ros2__msg__RTKInfo__FIELD_NAME__base_eph, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {inertial_sense_ros2__msg__RTKInfo__FIELD_NAME__base_ant_count, 14, 14},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription inertial_sense_ros2__msg__RTKInfo__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {std_msgs__msg__Header__TYPE_NAME, 19, 19},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
inertial_sense_ros2__msg__RTKInfo__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {inertial_sense_ros2__msg__RTKInfo__TYPE_NAME, 31, 31},
      {inertial_sense_ros2__msg__RTKInfo__FIELDS, 8, 8},
    },
    {inertial_sense_ros2__msg__RTKInfo__REFERENCED_TYPE_DESCRIPTIONS, 2, 2},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&std_msgs__msg__Header__EXPECTED_HASH, std_msgs__msg__Header__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[1].fields = std_msgs__msg__Header__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "std_msgs/Header header\n"
  "\n"
  "float32[3] base_lla \\t\\t\\t# base position in lat-lon-altitude (deg, deg, m)\n"
  "uint32 cycle_slip_count \\t# number of cycle slips detected\n"
  "uint32 rover_obs\\t\\t\\t\\t# number of observations from rover (GPS, Glonass, Gallileo, Beidou, Qzs)\n"
  "uint32 base_obs\\t\\t\\t\\t# number of observations from base (GPS, Glonass, Gallileo, Beidou, Qzs)\n"
  "uint32 rover_eph\\t\\t\\t\\t# number of ephemeris messages from rover (GPS, Glonass, Gallileo, Beidou, Qzs)\n"
  "uint32 base_eph\\t\\t\\t\\t# number of ephemeris messages from rover (GPS, Glonass, Gallileo, Beidou, Qzs)\n"
  "uint32 base_ant_count\\t\\t\\t# number of base station antenna position measurements\n"
  "";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
inertial_sense_ros2__msg__RTKInfo__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {inertial_sense_ros2__msg__RTKInfo__TYPE_NAME, 31, 31},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 620, 620},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
inertial_sense_ros2__msg__RTKInfo__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[3];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 3, 3};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *inertial_sense_ros2__msg__RTKInfo__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *std_msgs__msg__Header__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
