// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from inertial_sense_ros2:msg/GlonassEphemeris.idl
// generated code does not contain a copyright notice

#include "inertial_sense_ros2/msg/detail/glonass_ephemeris__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_inertial_sense_ros2
const rosidl_type_hash_t *
inertial_sense_ros2__msg__GlonassEphemeris__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x30, 0x14, 0xe6, 0x1b, 0x12, 0x92, 0x17, 0x69,
      0x72, 0xe7, 0xbe, 0xa2, 0x39, 0x13, 0xa0, 0x4a,
      0x35, 0x19, 0x51, 0x79, 0xa8, 0x53, 0x78, 0x6c,
      0xa3, 0x75, 0x69, 0xd5, 0x2b, 0x8a, 0xf7, 0x15,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types
#include "inertial_sense_ros2/msg/detail/g_time__functions.h"

// Hashes for external referenced types
#ifndef NDEBUG
static const rosidl_type_hash_t inertial_sense_ros2__msg__GTime__EXPECTED_HASH = {1, {
    0x4e, 0x16, 0x46, 0x23, 0xff, 0x6e, 0x2b, 0x23,
    0x0d, 0xd7, 0xa9, 0x4b, 0x62, 0x65, 0xed, 0x89,
    0x8f, 0x1a, 0xce, 0x3c, 0x8e, 0x12, 0x4b, 0x16,
    0x50, 0xdb, 0x7a, 0xb3, 0xc2, 0xb5, 0xa1, 0xa9,
  }};
#endif

static char inertial_sense_ros2__msg__GlonassEphemeris__TYPE_NAME[] = "inertial_sense_ros2/msg/GlonassEphemeris";
static char inertial_sense_ros2__msg__GTime__TYPE_NAME[] = "inertial_sense_ros2/msg/GTime";

// Define type names, field names, and default values
static char inertial_sense_ros2__msg__GlonassEphemeris__FIELD_NAME__sat[] = "sat";
static char inertial_sense_ros2__msg__GlonassEphemeris__FIELD_NAME__iode[] = "iode";
static char inertial_sense_ros2__msg__GlonassEphemeris__FIELD_NAME__frq[] = "frq";
static char inertial_sense_ros2__msg__GlonassEphemeris__FIELD_NAME__svh[] = "svh";
static char inertial_sense_ros2__msg__GlonassEphemeris__FIELD_NAME__sva[] = "sva";
static char inertial_sense_ros2__msg__GlonassEphemeris__FIELD_NAME__age[] = "age";
static char inertial_sense_ros2__msg__GlonassEphemeris__FIELD_NAME__toe[] = "toe";
static char inertial_sense_ros2__msg__GlonassEphemeris__FIELD_NAME__tof[] = "tof";
static char inertial_sense_ros2__msg__GlonassEphemeris__FIELD_NAME__pos[] = "pos";
static char inertial_sense_ros2__msg__GlonassEphemeris__FIELD_NAME__vel[] = "vel";
static char inertial_sense_ros2__msg__GlonassEphemeris__FIELD_NAME__acc[] = "acc";
static char inertial_sense_ros2__msg__GlonassEphemeris__FIELD_NAME__taun[] = "taun";
static char inertial_sense_ros2__msg__GlonassEphemeris__FIELD_NAME__gamn[] = "gamn";
static char inertial_sense_ros2__msg__GlonassEphemeris__FIELD_NAME__dtaun[] = "dtaun";

static rosidl_runtime_c__type_description__Field inertial_sense_ros2__msg__GlonassEphemeris__FIELDS[] = {
  {
    {inertial_sense_ros2__msg__GlonassEphemeris__FIELD_NAME__sat, 3, 3},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {inertial_sense_ros2__msg__GlonassEphemeris__FIELD_NAME__iode, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {inertial_sense_ros2__msg__GlonassEphemeris__FIELD_NAME__frq, 3, 3},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {inertial_sense_ros2__msg__GlonassEphemeris__FIELD_NAME__svh, 3, 3},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {inertial_sense_ros2__msg__GlonassEphemeris__FIELD_NAME__sva, 3, 3},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {inertial_sense_ros2__msg__GlonassEphemeris__FIELD_NAME__age, 3, 3},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {inertial_sense_ros2__msg__GlonassEphemeris__FIELD_NAME__toe, 3, 3},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {inertial_sense_ros2__msg__GTime__TYPE_NAME, 29, 29},
    },
    {NULL, 0, 0},
  },
  {
    {inertial_sense_ros2__msg__GlonassEphemeris__FIELD_NAME__tof, 3, 3},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {inertial_sense_ros2__msg__GTime__TYPE_NAME, 29, 29},
    },
    {NULL, 0, 0},
  },
  {
    {inertial_sense_ros2__msg__GlonassEphemeris__FIELD_NAME__pos, 3, 3},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE_ARRAY,
      3,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {inertial_sense_ros2__msg__GlonassEphemeris__FIELD_NAME__vel, 3, 3},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE_ARRAY,
      3,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {inertial_sense_ros2__msg__GlonassEphemeris__FIELD_NAME__acc, 3, 3},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE_ARRAY,
      3,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {inertial_sense_ros2__msg__GlonassEphemeris__FIELD_NAME__taun, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {inertial_sense_ros2__msg__GlonassEphemeris__FIELD_NAME__gamn, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {inertial_sense_ros2__msg__GlonassEphemeris__FIELD_NAME__dtaun, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription inertial_sense_ros2__msg__GlonassEphemeris__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {inertial_sense_ros2__msg__GTime__TYPE_NAME, 29, 29},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
inertial_sense_ros2__msg__GlonassEphemeris__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {inertial_sense_ros2__msg__GlonassEphemeris__TYPE_NAME, 40, 40},
      {inertial_sense_ros2__msg__GlonassEphemeris__FIELDS, 14, 14},
    },
    {inertial_sense_ros2__msg__GlonassEphemeris__REFERENCED_TYPE_DESCRIPTIONS, 1, 1},
  };
  if (!constructed) {
    assert(0 == memcmp(&inertial_sense_ros2__msg__GTime__EXPECTED_HASH, inertial_sense_ros2__msg__GTime__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = inertial_sense_ros2__msg__GTime__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "int32 sat \\t# satellite number \n"
  "int32 iode \\t# IODE (0-6 bit of tb field) \n"
  "int32 frq \\t# satellite frequency number \n"
  "int32 svh \\t# satellite health \n"
  "int32 sva \\t# satellite accuracy \n"
  "int32 age \\t# satellite age of operation \n"
  "GTime toe \\t# epoch of epherides (gpst) \n"
  "GTime tof \\t# message frame time (gpst) \n"
  "float64[3] pos \\t# satellite position (ecef) (m) \n"
  "float64[3] vel \\t# satellite velocity (ecef) (m/s) \n"
  "float64[3] acc \\t# satellite acceleration (ecef) (m/s^2) \n"
  "float64 taun \\t# SV clock bias (s) \n"
  "float64 gamn \\t# relative freq bias \n"
  "float64 dtaun \\t# delay between L1 and L2 (s) ";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
inertial_sense_ros2__msg__GlonassEphemeris__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {inertial_sense_ros2__msg__GlonassEphemeris__TYPE_NAME, 40, 40},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 572, 572},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
inertial_sense_ros2__msg__GlonassEphemeris__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[2];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 2, 2};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *inertial_sense_ros2__msg__GlonassEphemeris__get_individual_type_description_source(NULL),
    sources[1] = *inertial_sense_ros2__msg__GTime__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
