// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from inertial_sense_ros2:srv/RefLLAUpdate.idl
// generated code does not contain a copyright notice

#include "inertial_sense_ros2/srv/detail/ref_lla_update__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_inertial_sense_ros2
const rosidl_type_hash_t *
inertial_sense_ros2__srv__RefLLAUpdate__get_type_hash(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x9b, 0xe8, 0x03, 0x1c, 0xdb, 0x9f, 0xc1, 0xbb,
      0xf3, 0x28, 0xd9, 0x2d, 0x09, 0x06, 0xfc, 0xca,
      0x30, 0xdb, 0x94, 0x99, 0xe8, 0x76, 0x88, 0xf3,
      0x12, 0x19, 0xce, 0x79, 0xda, 0xb5, 0xf4, 0x25,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_inertial_sense_ros2
const rosidl_type_hash_t *
inertial_sense_ros2__srv__RefLLAUpdate_Request__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x02, 0xd7, 0xfc, 0xa1, 0x79, 0xf2, 0x12, 0x5b,
      0xf0, 0x9d, 0xb3, 0xcf, 0x63, 0xff, 0x46, 0x5d,
      0x6b, 0x1d, 0xb7, 0x2d, 0x9a, 0xab, 0x2b, 0x0f,
      0x73, 0xc7, 0xcc, 0xd7, 0x21, 0x07, 0xe2, 0xec,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_inertial_sense_ros2
const rosidl_type_hash_t *
inertial_sense_ros2__srv__RefLLAUpdate_Response__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x49, 0xff, 0x79, 0xfb, 0x5c, 0x02, 0x11, 0x98,
      0xcd, 0x70, 0x82, 0x86, 0x68, 0x0b, 0x2b, 0x28,
      0xe3, 0x6e, 0xda, 0xe5, 0x50, 0x3c, 0x85, 0xeb,
      0xbd, 0x8d, 0x9c, 0x6d, 0x23, 0x8f, 0x8f, 0xbb,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_inertial_sense_ros2
const rosidl_type_hash_t *
inertial_sense_ros2__srv__RefLLAUpdate_Event__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x96, 0x01, 0x22, 0x5a, 0x94, 0x88, 0xf7, 0x5c,
      0xb4, 0x59, 0x93, 0xb9, 0x54, 0xe7, 0x50, 0x41,
      0x1a, 0x48, 0x43, 0x4c, 0x72, 0x43, 0x86, 0x73,
      0x32, 0x7a, 0xb5, 0x84, 0x20, 0xf6, 0x6a, 0xf8,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types
#include "service_msgs/msg/detail/service_event_info__functions.h"
#include "builtin_interfaces/msg/detail/time__functions.h"

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

static char inertial_sense_ros2__srv__RefLLAUpdate__TYPE_NAME[] = "inertial_sense_ros2/srv/RefLLAUpdate";
static char builtin_interfaces__msg__Time__TYPE_NAME[] = "builtin_interfaces/msg/Time";
static char inertial_sense_ros2__srv__RefLLAUpdate_Event__TYPE_NAME[] = "inertial_sense_ros2/srv/RefLLAUpdate_Event";
static char inertial_sense_ros2__srv__RefLLAUpdate_Request__TYPE_NAME[] = "inertial_sense_ros2/srv/RefLLAUpdate_Request";
static char inertial_sense_ros2__srv__RefLLAUpdate_Response__TYPE_NAME[] = "inertial_sense_ros2/srv/RefLLAUpdate_Response";
static char service_msgs__msg__ServiceEventInfo__TYPE_NAME[] = "service_msgs/msg/ServiceEventInfo";

// Define type names, field names, and default values
static char inertial_sense_ros2__srv__RefLLAUpdate__FIELD_NAME__request_message[] = "request_message";
static char inertial_sense_ros2__srv__RefLLAUpdate__FIELD_NAME__response_message[] = "response_message";
static char inertial_sense_ros2__srv__RefLLAUpdate__FIELD_NAME__event_message[] = "event_message";

static rosidl_runtime_c__type_description__Field inertial_sense_ros2__srv__RefLLAUpdate__FIELDS[] = {
  {
    {inertial_sense_ros2__srv__RefLLAUpdate__FIELD_NAME__request_message, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {inertial_sense_ros2__srv__RefLLAUpdate_Request__TYPE_NAME, 44, 44},
    },
    {NULL, 0, 0},
  },
  {
    {inertial_sense_ros2__srv__RefLLAUpdate__FIELD_NAME__response_message, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {inertial_sense_ros2__srv__RefLLAUpdate_Response__TYPE_NAME, 45, 45},
    },
    {NULL, 0, 0},
  },
  {
    {inertial_sense_ros2__srv__RefLLAUpdate__FIELD_NAME__event_message, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {inertial_sense_ros2__srv__RefLLAUpdate_Event__TYPE_NAME, 42, 42},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription inertial_sense_ros2__srv__RefLLAUpdate__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {inertial_sense_ros2__srv__RefLLAUpdate_Event__TYPE_NAME, 42, 42},
    {NULL, 0, 0},
  },
  {
    {inertial_sense_ros2__srv__RefLLAUpdate_Request__TYPE_NAME, 44, 44},
    {NULL, 0, 0},
  },
  {
    {inertial_sense_ros2__srv__RefLLAUpdate_Response__TYPE_NAME, 45, 45},
    {NULL, 0, 0},
  },
  {
    {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
inertial_sense_ros2__srv__RefLLAUpdate__get_type_description(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {inertial_sense_ros2__srv__RefLLAUpdate__TYPE_NAME, 36, 36},
      {inertial_sense_ros2__srv__RefLLAUpdate__FIELDS, 3, 3},
    },
    {inertial_sense_ros2__srv__RefLLAUpdate__REFERENCED_TYPE_DESCRIPTIONS, 5, 5},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[1].fields = inertial_sense_ros2__srv__RefLLAUpdate_Event__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[2].fields = inertial_sense_ros2__srv__RefLLAUpdate_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[3].fields = inertial_sense_ros2__srv__RefLLAUpdate_Response__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[4].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char inertial_sense_ros2__srv__RefLLAUpdate_Request__FIELD_NAME__lla[] = "lla";

static rosidl_runtime_c__type_description__Field inertial_sense_ros2__srv__RefLLAUpdate_Request__FIELDS[] = {
  {
    {inertial_sense_ros2__srv__RefLLAUpdate_Request__FIELD_NAME__lla, 3, 3},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE_ARRAY,
      3,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
inertial_sense_ros2__srv__RefLLAUpdate_Request__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {inertial_sense_ros2__srv__RefLLAUpdate_Request__TYPE_NAME, 44, 44},
      {inertial_sense_ros2__srv__RefLLAUpdate_Request__FIELDS, 1, 1},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char inertial_sense_ros2__srv__RefLLAUpdate_Response__FIELD_NAME__success[] = "success";
static char inertial_sense_ros2__srv__RefLLAUpdate_Response__FIELD_NAME__message[] = "message";

static rosidl_runtime_c__type_description__Field inertial_sense_ros2__srv__RefLLAUpdate_Response__FIELDS[] = {
  {
    {inertial_sense_ros2__srv__RefLLAUpdate_Response__FIELD_NAME__success, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {inertial_sense_ros2__srv__RefLLAUpdate_Response__FIELD_NAME__message, 7, 7},
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
inertial_sense_ros2__srv__RefLLAUpdate_Response__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {inertial_sense_ros2__srv__RefLLAUpdate_Response__TYPE_NAME, 45, 45},
      {inertial_sense_ros2__srv__RefLLAUpdate_Response__FIELDS, 2, 2},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char inertial_sense_ros2__srv__RefLLAUpdate_Event__FIELD_NAME__info[] = "info";
static char inertial_sense_ros2__srv__RefLLAUpdate_Event__FIELD_NAME__request[] = "request";
static char inertial_sense_ros2__srv__RefLLAUpdate_Event__FIELD_NAME__response[] = "response";

static rosidl_runtime_c__type_description__Field inertial_sense_ros2__srv__RefLLAUpdate_Event__FIELDS[] = {
  {
    {inertial_sense_ros2__srv__RefLLAUpdate_Event__FIELD_NAME__info, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    },
    {NULL, 0, 0},
  },
  {
    {inertial_sense_ros2__srv__RefLLAUpdate_Event__FIELD_NAME__request, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {inertial_sense_ros2__srv__RefLLAUpdate_Request__TYPE_NAME, 44, 44},
    },
    {NULL, 0, 0},
  },
  {
    {inertial_sense_ros2__srv__RefLLAUpdate_Event__FIELD_NAME__response, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {inertial_sense_ros2__srv__RefLLAUpdate_Response__TYPE_NAME, 45, 45},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription inertial_sense_ros2__srv__RefLLAUpdate_Event__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {inertial_sense_ros2__srv__RefLLAUpdate_Request__TYPE_NAME, 44, 44},
    {NULL, 0, 0},
  },
  {
    {inertial_sense_ros2__srv__RefLLAUpdate_Response__TYPE_NAME, 45, 45},
    {NULL, 0, 0},
  },
  {
    {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
inertial_sense_ros2__srv__RefLLAUpdate_Event__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {inertial_sense_ros2__srv__RefLLAUpdate_Event__TYPE_NAME, 42, 42},
      {inertial_sense_ros2__srv__RefLLAUpdate_Event__FIELDS, 3, 3},
    },
    {inertial_sense_ros2__srv__RefLLAUpdate_Event__REFERENCED_TYPE_DESCRIPTIONS, 4, 4},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[1].fields = inertial_sense_ros2__srv__RefLLAUpdate_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[2].fields = inertial_sense_ros2__srv__RefLLAUpdate_Response__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[3].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "float64[3] lla\n"
  "---\n"
  "bool success\n"
  "string message";

static char srv_encoding[] = "srv";
static char implicit_encoding[] = "implicit";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
inertial_sense_ros2__srv__RefLLAUpdate__get_individual_type_description_source(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {inertial_sense_ros2__srv__RefLLAUpdate__TYPE_NAME, 36, 36},
    {srv_encoding, 3, 3},
    {toplevel_type_raw_source, 46, 46},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
inertial_sense_ros2__srv__RefLLAUpdate_Request__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {inertial_sense_ros2__srv__RefLLAUpdate_Request__TYPE_NAME, 44, 44},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
inertial_sense_ros2__srv__RefLLAUpdate_Response__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {inertial_sense_ros2__srv__RefLLAUpdate_Response__TYPE_NAME, 45, 45},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
inertial_sense_ros2__srv__RefLLAUpdate_Event__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {inertial_sense_ros2__srv__RefLLAUpdate_Event__TYPE_NAME, 42, 42},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
inertial_sense_ros2__srv__RefLLAUpdate__get_type_description_sources(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[6];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 6, 6};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *inertial_sense_ros2__srv__RefLLAUpdate__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *inertial_sense_ros2__srv__RefLLAUpdate_Event__get_individual_type_description_source(NULL);
    sources[3] = *inertial_sense_ros2__srv__RefLLAUpdate_Request__get_individual_type_description_source(NULL);
    sources[4] = *inertial_sense_ros2__srv__RefLLAUpdate_Response__get_individual_type_description_source(NULL);
    sources[5] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
inertial_sense_ros2__srv__RefLLAUpdate_Request__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *inertial_sense_ros2__srv__RefLLAUpdate_Request__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
inertial_sense_ros2__srv__RefLLAUpdate_Response__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *inertial_sense_ros2__srv__RefLLAUpdate_Response__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
inertial_sense_ros2__srv__RefLLAUpdate_Event__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[5];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 5, 5};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *inertial_sense_ros2__srv__RefLLAUpdate_Event__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *inertial_sense_ros2__srv__RefLLAUpdate_Request__get_individual_type_description_source(NULL);
    sources[3] = *inertial_sense_ros2__srv__RefLLAUpdate_Response__get_individual_type_description_source(NULL);
    sources[4] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
