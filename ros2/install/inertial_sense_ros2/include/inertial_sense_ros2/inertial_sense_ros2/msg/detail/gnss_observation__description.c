// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from inertial_sense_ros2:msg/GNSSObservation.idl
// generated code does not contain a copyright notice

#include "inertial_sense_ros2/msg/detail/gnss_observation__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_inertial_sense_ros2
const rosidl_type_hash_t *
inertial_sense_ros2__msg__GNSSObservation__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x6f, 0x0f, 0x11, 0x67, 0x05, 0xa1, 0xb7, 0x7c,
      0x4f, 0x33, 0xed, 0xb2, 0xc0, 0x8b, 0xdc, 0xd0,
      0x03, 0x82, 0x5d, 0x00, 0xb3, 0xfe, 0x81, 0xea,
      0x81, 0x07, 0x00, 0x6e, 0x26, 0x49, 0x37, 0x24,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types
#include "builtin_interfaces/msg/detail/time__functions.h"
#include "inertial_sense_ros2/msg/detail/g_time__functions.h"
#include "std_msgs/msg/detail/header__functions.h"

// Hashes for external referenced types
#ifndef NDEBUG
static const rosidl_type_hash_t builtin_interfaces__msg__Time__EXPECTED_HASH = {1, {
    0xb1, 0x06, 0x23, 0x5e, 0x25, 0xa4, 0xc5, 0xed,
    0x35, 0x09, 0x8a, 0xa0, 0xa6, 0x1a, 0x3e, 0xe9,
    0xc9, 0xb1, 0x8d, 0x19, 0x7f, 0x39, 0x8b, 0x0e,
    0x42, 0x06, 0xce, 0xa9, 0xac, 0xf9, 0xc1, 0x97,
  }};
static const rosidl_type_hash_t inertial_sense_ros2__msg__GTime__EXPECTED_HASH = {1, {
    0x4e, 0x16, 0x46, 0x23, 0xff, 0x6e, 0x2b, 0x23,
    0x0d, 0xd7, 0xa9, 0x4b, 0x62, 0x65, 0xed, 0x89,
    0x8f, 0x1a, 0xce, 0x3c, 0x8e, 0x12, 0x4b, 0x16,
    0x50, 0xdb, 0x7a, 0xb3, 0xc2, 0xb5, 0xa1, 0xa9,
  }};
static const rosidl_type_hash_t std_msgs__msg__Header__EXPECTED_HASH = {1, {
    0xf4, 0x9f, 0xb3, 0xae, 0x2c, 0xf0, 0x70, 0xf7,
    0x93, 0x64, 0x5f, 0xf7, 0x49, 0x68, 0x3a, 0xc6,
    0xb0, 0x62, 0x03, 0xe4, 0x1c, 0x89, 0x1e, 0x17,
    0x70, 0x1b, 0x1c, 0xb5, 0x97, 0xce, 0x6a, 0x01,
  }};
#endif

static char inertial_sense_ros2__msg__GNSSObservation__TYPE_NAME[] = "inertial_sense_ros2/msg/GNSSObservation";
static char builtin_interfaces__msg__Time__TYPE_NAME[] = "builtin_interfaces/msg/Time";
static char inertial_sense_ros2__msg__GTime__TYPE_NAME[] = "inertial_sense_ros2/msg/GTime";
static char std_msgs__msg__Header__TYPE_NAME[] = "std_msgs/msg/Header";

// Define type names, field names, and default values
static char inertial_sense_ros2__msg__GNSSObservation__FIELD_NAME__header[] = "header";
static char inertial_sense_ros2__msg__GNSSObservation__FIELD_NAME__time[] = "time";
static char inertial_sense_ros2__msg__GNSSObservation__FIELD_NAME__sat[] = "sat";
static char inertial_sense_ros2__msg__GNSSObservation__FIELD_NAME__rcv[] = "rcv";
static char inertial_sense_ros2__msg__GNSSObservation__FIELD_NAME__snrr[] = "snrr";
static char inertial_sense_ros2__msg__GNSSObservation__FIELD_NAME__lli[] = "lli";
static char inertial_sense_ros2__msg__GNSSObservation__FIELD_NAME__code[] = "code";
static char inertial_sense_ros2__msg__GNSSObservation__FIELD_NAME__qual_l[] = "qual_l";
static char inertial_sense_ros2__msg__GNSSObservation__FIELD_NAME__qual_p[] = "qual_p";
static char inertial_sense_ros2__msg__GNSSObservation__FIELD_NAME__l[] = "l";
static char inertial_sense_ros2__msg__GNSSObservation__FIELD_NAME__p[] = "p";
static char inertial_sense_ros2__msg__GNSSObservation__FIELD_NAME__d[] = "d";

static rosidl_runtime_c__type_description__Field inertial_sense_ros2__msg__GNSSObservation__FIELDS[] = {
  {
    {inertial_sense_ros2__msg__GNSSObservation__FIELD_NAME__header, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {std_msgs__msg__Header__TYPE_NAME, 19, 19},
    },
    {NULL, 0, 0},
  },
  {
    {inertial_sense_ros2__msg__GNSSObservation__FIELD_NAME__time, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {inertial_sense_ros2__msg__GTime__TYPE_NAME, 29, 29},
    },
    {NULL, 0, 0},
  },
  {
    {inertial_sense_ros2__msg__GNSSObservation__FIELD_NAME__sat, 3, 3},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {inertial_sense_ros2__msg__GNSSObservation__FIELD_NAME__rcv, 3, 3},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {inertial_sense_ros2__msg__GNSSObservation__FIELD_NAME__snrr, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {inertial_sense_ros2__msg__GNSSObservation__FIELD_NAME__lli, 3, 3},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {inertial_sense_ros2__msg__GNSSObservation__FIELD_NAME__code, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {inertial_sense_ros2__msg__GNSSObservation__FIELD_NAME__qual_l, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {inertial_sense_ros2__msg__GNSSObservation__FIELD_NAME__qual_p, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {inertial_sense_ros2__msg__GNSSObservation__FIELD_NAME__l, 1, 1},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {inertial_sense_ros2__msg__GNSSObservation__FIELD_NAME__p, 1, 1},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {inertial_sense_ros2__msg__GNSSObservation__FIELD_NAME__d, 1, 1},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription inertial_sense_ros2__msg__GNSSObservation__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {inertial_sense_ros2__msg__GTime__TYPE_NAME, 29, 29},
    {NULL, 0, 0},
  },
  {
    {std_msgs__msg__Header__TYPE_NAME, 19, 19},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
inertial_sense_ros2__msg__GNSSObservation__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {inertial_sense_ros2__msg__GNSSObservation__TYPE_NAME, 39, 39},
      {inertial_sense_ros2__msg__GNSSObservation__FIELDS, 12, 12},
    },
    {inertial_sense_ros2__msg__GNSSObservation__REFERENCED_TYPE_DESCRIPTIONS, 3, 3},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&inertial_sense_ros2__msg__GTime__EXPECTED_HASH, inertial_sense_ros2__msg__GTime__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[1].fields = inertial_sense_ros2__msg__GTime__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&std_msgs__msg__Header__EXPECTED_HASH, std_msgs__msg__Header__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[2].fields = std_msgs__msg__Header__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "std_msgs/Header header\n"
  "GTime time              # time of observation\n"
  "uint8 sat \\t\\t# satellite number\n"
  "uint8 rcv \\t\\t# receiver number\n"
  "uint8 snrr \\t\\t# Signal Strength (0.25 dBHz)\n"
  "uint8 lli \\t\\t# Loss-of-Lock Indicator (bit1=loss-of-lock, bit2=half-cycle-invalid)\n"
  "uint8 code \\t\\t# code indicator (BeiDou: CODE_L1I, Other: CODE_L1C )\n"
  "uint8 qual_l \\t# Estimated carrier phase measurement standard deviation (0.004 cycles)\n"
  "uint8 qual_p \\t# Estimated pseudorange measurement standard deviation (0.01 m)\n"
  "float64 l      \\t# observation data carrier-phase (cycle)\n"
  "float64 p      \\t# observation data pseudorange (m)\n"
  "float32 d      \\t# observation data doppler frequency (0.002 Hz)";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
inertial_sense_ros2__msg__GNSSObservation__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {inertial_sense_ros2__msg__GNSSObservation__TYPE_NAME, 39, 39},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 658, 658},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
inertial_sense_ros2__msg__GNSSObservation__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[4];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 4, 4};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *inertial_sense_ros2__msg__GNSSObservation__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *inertial_sense_ros2__msg__GTime__get_individual_type_description_source(NULL);
    sources[3] = *std_msgs__msg__Header__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
