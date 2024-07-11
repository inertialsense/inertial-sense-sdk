// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from inertial_sense_ros2:msg/GNSSObsVec.idl
// generated code does not contain a copyright notice

#include "inertial_sense_ros2/msg/detail/gnss_obs_vec__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_inertial_sense_ros2
const rosidl_type_hash_t *
inertial_sense_ros2__msg__GNSSObsVec__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x33, 0x40, 0x6d, 0x50, 0xca, 0xb3, 0xb2, 0x28,
      0xbd, 0x8f, 0xd2, 0x3a, 0xcc, 0x03, 0xf3, 0x98,
      0x88, 0x90, 0x16, 0x2b, 0x70, 0xae, 0x23, 0xec,
      0x30, 0xbd, 0x89, 0x3b, 0x9f, 0x65, 0x56, 0x4c,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types
#include "builtin_interfaces/msg/detail/time__functions.h"
#include "inertial_sense_ros2/msg/detail/g_time__functions.h"
#include "inertial_sense_ros2/msg/detail/gnss_observation__functions.h"
#include "std_msgs/msg/detail/header__functions.h"

// Hashes for external referenced types
#ifndef NDEBUG
static const rosidl_type_hash_t builtin_interfaces__msg__Time__EXPECTED_HASH = {1, {
    0xb1, 0x06, 0x23, 0x5e, 0x25, 0xa4, 0xc5, 0xed,
    0x35, 0x09, 0x8a, 0xa0, 0xa6, 0x1a, 0x3e, 0xe9,
    0xc9, 0xb1, 0x8d, 0x19, 0x7f, 0x39, 0x8b, 0x0e,
    0x42, 0x06, 0xce, 0xa9, 0xac, 0xf9, 0xc1, 0x97,
  }};
static const rosidl_type_hash_t inertial_sense_ros2__msg__GNSSObservation__EXPECTED_HASH = {1, {
    0x6f, 0x0f, 0x11, 0x67, 0x05, 0xa1, 0xb7, 0x7c,
    0x4f, 0x33, 0xed, 0xb2, 0xc0, 0x8b, 0xdc, 0xd0,
    0x03, 0x82, 0x5d, 0x00, 0xb3, 0xfe, 0x81, 0xea,
    0x81, 0x07, 0x00, 0x6e, 0x26, 0x49, 0x37, 0x24,
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

static char inertial_sense_ros2__msg__GNSSObsVec__TYPE_NAME[] = "inertial_sense_ros2/msg/GNSSObsVec";
static char builtin_interfaces__msg__Time__TYPE_NAME[] = "builtin_interfaces/msg/Time";
static char inertial_sense_ros2__msg__GNSSObservation__TYPE_NAME[] = "inertial_sense_ros2/msg/GNSSObservation";
static char inertial_sense_ros2__msg__GTime__TYPE_NAME[] = "inertial_sense_ros2/msg/GTime";
static char std_msgs__msg__Header__TYPE_NAME[] = "std_msgs/msg/Header";

// Define type names, field names, and default values
static char inertial_sense_ros2__msg__GNSSObsVec__FIELD_NAME__header[] = "header";
static char inertial_sense_ros2__msg__GNSSObsVec__FIELD_NAME__time[] = "time";
static char inertial_sense_ros2__msg__GNSSObsVec__FIELD_NAME__obs[] = "obs";

static rosidl_runtime_c__type_description__Field inertial_sense_ros2__msg__GNSSObsVec__FIELDS[] = {
  {
    {inertial_sense_ros2__msg__GNSSObsVec__FIELD_NAME__header, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {std_msgs__msg__Header__TYPE_NAME, 19, 19},
    },
    {NULL, 0, 0},
  },
  {
    {inertial_sense_ros2__msg__GNSSObsVec__FIELD_NAME__time, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {inertial_sense_ros2__msg__GTime__TYPE_NAME, 29, 29},
    },
    {NULL, 0, 0},
  },
  {
    {inertial_sense_ros2__msg__GNSSObsVec__FIELD_NAME__obs, 3, 3},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_UNBOUNDED_SEQUENCE,
      0,
      0,
      {inertial_sense_ros2__msg__GNSSObservation__TYPE_NAME, 39, 39},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription inertial_sense_ros2__msg__GNSSObsVec__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {inertial_sense_ros2__msg__GNSSObservation__TYPE_NAME, 39, 39},
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
inertial_sense_ros2__msg__GNSSObsVec__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {inertial_sense_ros2__msg__GNSSObsVec__TYPE_NAME, 34, 34},
      {inertial_sense_ros2__msg__GNSSObsVec__FIELDS, 3, 3},
    },
    {inertial_sense_ros2__msg__GNSSObsVec__REFERENCED_TYPE_DESCRIPTIONS, 4, 4},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&inertial_sense_ros2__msg__GNSSObservation__EXPECTED_HASH, inertial_sense_ros2__msg__GNSSObservation__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[1].fields = inertial_sense_ros2__msg__GNSSObservation__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&inertial_sense_ros2__msg__GTime__EXPECTED_HASH, inertial_sense_ros2__msg__GTime__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[2].fields = inertial_sense_ros2__msg__GTime__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&std_msgs__msg__Header__EXPECTED_HASH, std_msgs__msg__Header__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[3].fields = std_msgs__msg__Header__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "std_msgs/Header header\n"
  "GTime time              # time of all contained observations (UTC Time w/o Leap Seconds)\n"
  "GNSSObservation[] obs   # Vector of observations";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
inertial_sense_ros2__msg__GNSSObsVec__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {inertial_sense_ros2__msg__GNSSObsVec__TYPE_NAME, 34, 34},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 160, 160},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
inertial_sense_ros2__msg__GNSSObsVec__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[5];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 5, 5};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *inertial_sense_ros2__msg__GNSSObsVec__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *inertial_sense_ros2__msg__GNSSObservation__get_individual_type_description_source(NULL);
    sources[3] = *inertial_sense_ros2__msg__GTime__get_individual_type_description_source(NULL);
    sources[4] = *std_msgs__msg__Header__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
