// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from inertial_sense_ros2:srv/FirmwareUpdate.idl
// generated code does not contain a copyright notice

#include "inertial_sense_ros2/srv/detail/firmware_update__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_inertial_sense_ros2
const rosidl_type_hash_t *
inertial_sense_ros2__srv__FirmwareUpdate__get_type_hash(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x2e, 0xa2, 0x0c, 0x70, 0xb9, 0xf4, 0xde, 0x12,
      0xa3, 0x85, 0x40, 0x55, 0x91, 0x95, 0x55, 0xbf,
      0xf2, 0x6e, 0x10, 0x5b, 0x04, 0x58, 0x8c, 0x6a,
      0x38, 0xda, 0x8f, 0x3a, 0x34, 0xd0, 0xea, 0x90,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_inertial_sense_ros2
const rosidl_type_hash_t *
inertial_sense_ros2__srv__FirmwareUpdate_Request__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xf0, 0xe3, 0x1e, 0x3a, 0x8c, 0xcb, 0xed, 0x5c,
      0x4e, 0xe6, 0x99, 0xd7, 0xb9, 0xc3, 0xe1, 0x8d,
      0xb8, 0x25, 0xbe, 0xbb, 0x24, 0x3e, 0xb2, 0xe0,
      0xc3, 0x37, 0xc7, 0x14, 0x8a, 0xa6, 0x21, 0x0e,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_inertial_sense_ros2
const rosidl_type_hash_t *
inertial_sense_ros2__srv__FirmwareUpdate_Response__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xde, 0xea, 0x21, 0x77, 0xb0, 0xde, 0xd3, 0xce,
      0xbc, 0x7e, 0x88, 0x59, 0xde, 0x40, 0x7d, 0x33,
      0xb0, 0x3b, 0xbc, 0x30, 0x00, 0xdf, 0x70, 0x74,
      0x49, 0x20, 0xda, 0x2b, 0xf2, 0xcd, 0x8d, 0xdf,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_inertial_sense_ros2
const rosidl_type_hash_t *
inertial_sense_ros2__srv__FirmwareUpdate_Event__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x91, 0xed, 0x5c, 0x07, 0x11, 0xed, 0x34, 0x6c,
      0xb7, 0x72, 0xac, 0xf3, 0x86, 0x06, 0x54, 0x2a,
      0x5e, 0x1f, 0xd4, 0xa7, 0x58, 0x0a, 0x13, 0x87,
      0x07, 0x7d, 0x40, 0x83, 0x17, 0x37, 0x39, 0x1c,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types
#include "builtin_interfaces/msg/detail/time__functions.h"
#include "service_msgs/msg/detail/service_event_info__functions.h"

// Hashes for external referenced types
#ifndef NDEBUG
static const rosidl_type_hash_t builtin_interfaces__msg__Time__EXPECTED_HASH = {1, {
    0xb1, 0x06, 0x23, 0x5e, 0x25, 0xa4, 0xc5, 0xed,
    0x35, 0x09, 0x8a, 0xa0, 0xa6, 0x1a, 0x3e, 0xe9,
    0xc9, 0xb1, 0x8d, 0x19, 0x7f, 0x39, 0x8b, 0x0e,
    0x42, 0x06, 0xce, 0xa9, 0xac, 0xf9, 0xc1, 0x97,
  }};
static const rosidl_type_hash_t service_msgs__msg__ServiceEventInfo__EXPECTED_HASH = {1, {
    0x41, 0xbc, 0xbb, 0xe0, 0x7a, 0x75, 0xc9, 0xb5,
    0x2b, 0xc9, 0x6b, 0xfd, 0x5c, 0x24, 0xd7, 0xf0,
    0xfc, 0x0a, 0x08, 0xc0, 0xcb, 0x79, 0x21, 0xb3,
    0x37, 0x3c, 0x57, 0x32, 0x34, 0x5a, 0x6f, 0x45,
  }};
#endif

static char inertial_sense_ros2__srv__FirmwareUpdate__TYPE_NAME[] = "inertial_sense_ros2/srv/FirmwareUpdate";
static char builtin_interfaces__msg__Time__TYPE_NAME[] = "builtin_interfaces/msg/Time";
static char inertial_sense_ros2__srv__FirmwareUpdate_Event__TYPE_NAME[] = "inertial_sense_ros2/srv/FirmwareUpdate_Event";
static char inertial_sense_ros2__srv__FirmwareUpdate_Request__TYPE_NAME[] = "inertial_sense_ros2/srv/FirmwareUpdate_Request";
static char inertial_sense_ros2__srv__FirmwareUpdate_Response__TYPE_NAME[] = "inertial_sense_ros2/srv/FirmwareUpdate_Response";
static char service_msgs__msg__ServiceEventInfo__TYPE_NAME[] = "service_msgs/msg/ServiceEventInfo";

// Define type names, field names, and default values
static char inertial_sense_ros2__srv__FirmwareUpdate__FIELD_NAME__request_message[] = "request_message";
static char inertial_sense_ros2__srv__FirmwareUpdate__FIELD_NAME__response_message[] = "response_message";
static char inertial_sense_ros2__srv__FirmwareUpdate__FIELD_NAME__event_message[] = "event_message";

static rosidl_runtime_c__type_description__Field inertial_sense_ros2__srv__FirmwareUpdate__FIELDS[] = {
  {
    {inertial_sense_ros2__srv__FirmwareUpdate__FIELD_NAME__request_message, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {inertial_sense_ros2__srv__FirmwareUpdate_Request__TYPE_NAME, 46, 46},
    },
    {NULL, 0, 0},
  },
  {
    {inertial_sense_ros2__srv__FirmwareUpdate__FIELD_NAME__response_message, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {inertial_sense_ros2__srv__FirmwareUpdate_Response__TYPE_NAME, 47, 47},
    },
    {NULL, 0, 0},
  },
  {
    {inertial_sense_ros2__srv__FirmwareUpdate__FIELD_NAME__event_message, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {inertial_sense_ros2__srv__FirmwareUpdate_Event__TYPE_NAME, 44, 44},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription inertial_sense_ros2__srv__FirmwareUpdate__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {inertial_sense_ros2__srv__FirmwareUpdate_Event__TYPE_NAME, 44, 44},
    {NULL, 0, 0},
  },
  {
    {inertial_sense_ros2__srv__FirmwareUpdate_Request__TYPE_NAME, 46, 46},
    {NULL, 0, 0},
  },
  {
    {inertial_sense_ros2__srv__FirmwareUpdate_Response__TYPE_NAME, 47, 47},
    {NULL, 0, 0},
  },
  {
    {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
inertial_sense_ros2__srv__FirmwareUpdate__get_type_description(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {inertial_sense_ros2__srv__FirmwareUpdate__TYPE_NAME, 38, 38},
      {inertial_sense_ros2__srv__FirmwareUpdate__FIELDS, 3, 3},
    },
    {inertial_sense_ros2__srv__FirmwareUpdate__REFERENCED_TYPE_DESCRIPTIONS, 5, 5},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[1].fields = inertial_sense_ros2__srv__FirmwareUpdate_Event__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[2].fields = inertial_sense_ros2__srv__FirmwareUpdate_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[3].fields = inertial_sense_ros2__srv__FirmwareUpdate_Response__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[4].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char inertial_sense_ros2__srv__FirmwareUpdate_Request__FIELD_NAME__filename[] = "filename";

static rosidl_runtime_c__type_description__Field inertial_sense_ros2__srv__FirmwareUpdate_Request__FIELDS[] = {
  {
    {inertial_sense_ros2__srv__FirmwareUpdate_Request__FIELD_NAME__filename, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
inertial_sense_ros2__srv__FirmwareUpdate_Request__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {inertial_sense_ros2__srv__FirmwareUpdate_Request__TYPE_NAME, 46, 46},
      {inertial_sense_ros2__srv__FirmwareUpdate_Request__FIELDS, 1, 1},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char inertial_sense_ros2__srv__FirmwareUpdate_Response__FIELD_NAME__success[] = "success";
static char inertial_sense_ros2__srv__FirmwareUpdate_Response__FIELD_NAME__message[] = "message";

static rosidl_runtime_c__type_description__Field inertial_sense_ros2__srv__FirmwareUpdate_Response__FIELDS[] = {
  {
    {inertial_sense_ros2__srv__FirmwareUpdate_Response__FIELD_NAME__success, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {inertial_sense_ros2__srv__FirmwareUpdate_Response__FIELD_NAME__message, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
inertial_sense_ros2__srv__FirmwareUpdate_Response__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {inertial_sense_ros2__srv__FirmwareUpdate_Response__TYPE_NAME, 47, 47},
      {inertial_sense_ros2__srv__FirmwareUpdate_Response__FIELDS, 2, 2},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char inertial_sense_ros2__srv__FirmwareUpdate_Event__FIELD_NAME__info[] = "info";
static char inertial_sense_ros2__srv__FirmwareUpdate_Event__FIELD_NAME__request[] = "request";
static char inertial_sense_ros2__srv__FirmwareUpdate_Event__FIELD_NAME__response[] = "response";

static rosidl_runtime_c__type_description__Field inertial_sense_ros2__srv__FirmwareUpdate_Event__FIELDS[] = {
  {
    {inertial_sense_ros2__srv__FirmwareUpdate_Event__FIELD_NAME__info, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    },
    {NULL, 0, 0},
  },
  {
    {inertial_sense_ros2__srv__FirmwareUpdate_Event__FIELD_NAME__request, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {inertial_sense_ros2__srv__FirmwareUpdate_Request__TYPE_NAME, 46, 46},
    },
    {NULL, 0, 0},
  },
  {
    {inertial_sense_ros2__srv__FirmwareUpdate_Event__FIELD_NAME__response, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {inertial_sense_ros2__srv__FirmwareUpdate_Response__TYPE_NAME, 47, 47},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription inertial_sense_ros2__srv__FirmwareUpdate_Event__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {inertial_sense_ros2__srv__FirmwareUpdate_Request__TYPE_NAME, 46, 46},
    {NULL, 0, 0},
  },
  {
    {inertial_sense_ros2__srv__FirmwareUpdate_Response__TYPE_NAME, 47, 47},
    {NULL, 0, 0},
  },
  {
    {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
inertial_sense_ros2__srv__FirmwareUpdate_Event__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {inertial_sense_ros2__srv__FirmwareUpdate_Event__TYPE_NAME, 44, 44},
      {inertial_sense_ros2__srv__FirmwareUpdate_Event__FIELDS, 3, 3},
    },
    {inertial_sense_ros2__srv__FirmwareUpdate_Event__REFERENCED_TYPE_DESCRIPTIONS, 4, 4},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[1].fields = inertial_sense_ros2__srv__FirmwareUpdate_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[2].fields = inertial_sense_ros2__srv__FirmwareUpdate_Response__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[3].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "string filename\n"
  "---\n"
  "bool success\n"
  "string message";

static char srv_encoding[] = "srv";
static char implicit_encoding[] = "implicit";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
inertial_sense_ros2__srv__FirmwareUpdate__get_individual_type_description_source(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {inertial_sense_ros2__srv__FirmwareUpdate__TYPE_NAME, 38, 38},
    {srv_encoding, 3, 3},
    {toplevel_type_raw_source, 47, 47},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
inertial_sense_ros2__srv__FirmwareUpdate_Request__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {inertial_sense_ros2__srv__FirmwareUpdate_Request__TYPE_NAME, 46, 46},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
inertial_sense_ros2__srv__FirmwareUpdate_Response__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {inertial_sense_ros2__srv__FirmwareUpdate_Response__TYPE_NAME, 47, 47},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
inertial_sense_ros2__srv__FirmwareUpdate_Event__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {inertial_sense_ros2__srv__FirmwareUpdate_Event__TYPE_NAME, 44, 44},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
inertial_sense_ros2__srv__FirmwareUpdate__get_type_description_sources(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[6];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 6, 6};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *inertial_sense_ros2__srv__FirmwareUpdate__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *inertial_sense_ros2__srv__FirmwareUpdate_Event__get_individual_type_description_source(NULL);
    sources[3] = *inertial_sense_ros2__srv__FirmwareUpdate_Request__get_individual_type_description_source(NULL);
    sources[4] = *inertial_sense_ros2__srv__FirmwareUpdate_Response__get_individual_type_description_source(NULL);
    sources[5] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
inertial_sense_ros2__srv__FirmwareUpdate_Request__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *inertial_sense_ros2__srv__FirmwareUpdate_Request__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
inertial_sense_ros2__srv__FirmwareUpdate_Response__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *inertial_sense_ros2__srv__FirmwareUpdate_Response__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
inertial_sense_ros2__srv__FirmwareUpdate_Event__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[5];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 5, 5};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *inertial_sense_ros2__srv__FirmwareUpdate_Event__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *inertial_sense_ros2__srv__FirmwareUpdate_Request__get_individual_type_description_source(NULL);
    sources[3] = *inertial_sense_ros2__srv__FirmwareUpdate_Response__get_individual_type_description_source(NULL);
    sources[4] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
