// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from inertial_sense_ros2:msg/GNSSEphemeris.idl
// generated code does not contain a copyright notice

#include "inertial_sense_ros2/msg/detail/gnss_ephemeris__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_inertial_sense_ros2
const rosidl_type_hash_t *
inertial_sense_ros2__msg__GNSSEphemeris__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x98, 0x67, 0x67, 0xe1, 0x30, 0x9a, 0xd4, 0x4e,
      0x33, 0xa4, 0x44, 0xd5, 0x57, 0x90, 0x67, 0x16,
      0x5e, 0x45, 0x09, 0xb3, 0xfe, 0x61, 0xc9, 0xd9,
      0x3a, 0x74, 0xf4, 0x92, 0xbe, 0x09, 0xdb, 0xb5,
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

static char inertial_sense_ros2__msg__GNSSEphemeris__TYPE_NAME[] = "inertial_sense_ros2/msg/GNSSEphemeris";
static char builtin_interfaces__msg__Time__TYPE_NAME[] = "builtin_interfaces/msg/Time";
static char inertial_sense_ros2__msg__GTime__TYPE_NAME[] = "inertial_sense_ros2/msg/GTime";
static char std_msgs__msg__Header__TYPE_NAME[] = "std_msgs/msg/Header";

// Define type names, field names, and default values
static char inertial_sense_ros2__msg__GNSSEphemeris__FIELD_NAME__header[] = "header";
static char inertial_sense_ros2__msg__GNSSEphemeris__FIELD_NAME__sat[] = "sat";
static char inertial_sense_ros2__msg__GNSSEphemeris__FIELD_NAME__iode[] = "iode";
static char inertial_sense_ros2__msg__GNSSEphemeris__FIELD_NAME__iodc[] = "iodc";
static char inertial_sense_ros2__msg__GNSSEphemeris__FIELD_NAME__sva[] = "sva";
static char inertial_sense_ros2__msg__GNSSEphemeris__FIELD_NAME__svh[] = "svh";
static char inertial_sense_ros2__msg__GNSSEphemeris__FIELD_NAME__week[] = "week";
static char inertial_sense_ros2__msg__GNSSEphemeris__FIELD_NAME__code[] = "code";
static char inertial_sense_ros2__msg__GNSSEphemeris__FIELD_NAME__flag[] = "flag";
static char inertial_sense_ros2__msg__GNSSEphemeris__FIELD_NAME__toe[] = "toe";
static char inertial_sense_ros2__msg__GNSSEphemeris__FIELD_NAME__toc[] = "toc";
static char inertial_sense_ros2__msg__GNSSEphemeris__FIELD_NAME__ttr[] = "ttr";
static char inertial_sense_ros2__msg__GNSSEphemeris__FIELD_NAME__a[] = "a";
static char inertial_sense_ros2__msg__GNSSEphemeris__FIELD_NAME__e[] = "e";
static char inertial_sense_ros2__msg__GNSSEphemeris__FIELD_NAME__i0[] = "i0";
static char inertial_sense_ros2__msg__GNSSEphemeris__FIELD_NAME__omg_0[] = "omg_0";
static char inertial_sense_ros2__msg__GNSSEphemeris__FIELD_NAME__omg[] = "omg";
static char inertial_sense_ros2__msg__GNSSEphemeris__FIELD_NAME__m_0[] = "m_0";
static char inertial_sense_ros2__msg__GNSSEphemeris__FIELD_NAME__deln[] = "deln";
static char inertial_sense_ros2__msg__GNSSEphemeris__FIELD_NAME__omg_d[] = "omg_d";
static char inertial_sense_ros2__msg__GNSSEphemeris__FIELD_NAME__idot[] = "idot";
static char inertial_sense_ros2__msg__GNSSEphemeris__FIELD_NAME__crc[] = "crc";
static char inertial_sense_ros2__msg__GNSSEphemeris__FIELD_NAME__crs[] = "crs";
static char inertial_sense_ros2__msg__GNSSEphemeris__FIELD_NAME__cuc[] = "cuc";
static char inertial_sense_ros2__msg__GNSSEphemeris__FIELD_NAME__cus[] = "cus";
static char inertial_sense_ros2__msg__GNSSEphemeris__FIELD_NAME__cic[] = "cic";
static char inertial_sense_ros2__msg__GNSSEphemeris__FIELD_NAME__cis[] = "cis";
static char inertial_sense_ros2__msg__GNSSEphemeris__FIELD_NAME__toes[] = "toes";
static char inertial_sense_ros2__msg__GNSSEphemeris__FIELD_NAME__fit[] = "fit";
static char inertial_sense_ros2__msg__GNSSEphemeris__FIELD_NAME__f0[] = "f0";
static char inertial_sense_ros2__msg__GNSSEphemeris__FIELD_NAME__f1[] = "f1";
static char inertial_sense_ros2__msg__GNSSEphemeris__FIELD_NAME__f2[] = "f2";
static char inertial_sense_ros2__msg__GNSSEphemeris__FIELD_NAME__tgd[] = "tgd";
static char inertial_sense_ros2__msg__GNSSEphemeris__FIELD_NAME__a_dot[] = "a_dot";
static char inertial_sense_ros2__msg__GNSSEphemeris__FIELD_NAME__ndot[] = "ndot";

static rosidl_runtime_c__type_description__Field inertial_sense_ros2__msg__GNSSEphemeris__FIELDS[] = {
  {
    {inertial_sense_ros2__msg__GNSSEphemeris__FIELD_NAME__header, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {std_msgs__msg__Header__TYPE_NAME, 19, 19},
    },
    {NULL, 0, 0},
  },
  {
    {inertial_sense_ros2__msg__GNSSEphemeris__FIELD_NAME__sat, 3, 3},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {inertial_sense_ros2__msg__GNSSEphemeris__FIELD_NAME__iode, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {inertial_sense_ros2__msg__GNSSEphemeris__FIELD_NAME__iodc, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {inertial_sense_ros2__msg__GNSSEphemeris__FIELD_NAME__sva, 3, 3},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {inertial_sense_ros2__msg__GNSSEphemeris__FIELD_NAME__svh, 3, 3},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {inertial_sense_ros2__msg__GNSSEphemeris__FIELD_NAME__week, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {inertial_sense_ros2__msg__GNSSEphemeris__FIELD_NAME__code, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {inertial_sense_ros2__msg__GNSSEphemeris__FIELD_NAME__flag, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {inertial_sense_ros2__msg__GNSSEphemeris__FIELD_NAME__toe, 3, 3},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {inertial_sense_ros2__msg__GTime__TYPE_NAME, 29, 29},
    },
    {NULL, 0, 0},
  },
  {
    {inertial_sense_ros2__msg__GNSSEphemeris__FIELD_NAME__toc, 3, 3},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {inertial_sense_ros2__msg__GTime__TYPE_NAME, 29, 29},
    },
    {NULL, 0, 0},
  },
  {
    {inertial_sense_ros2__msg__GNSSEphemeris__FIELD_NAME__ttr, 3, 3},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {inertial_sense_ros2__msg__GTime__TYPE_NAME, 29, 29},
    },
    {NULL, 0, 0},
  },
  {
    {inertial_sense_ros2__msg__GNSSEphemeris__FIELD_NAME__a, 1, 1},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {inertial_sense_ros2__msg__GNSSEphemeris__FIELD_NAME__e, 1, 1},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {inertial_sense_ros2__msg__GNSSEphemeris__FIELD_NAME__i0, 2, 2},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {inertial_sense_ros2__msg__GNSSEphemeris__FIELD_NAME__omg_0, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {inertial_sense_ros2__msg__GNSSEphemeris__FIELD_NAME__omg, 3, 3},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {inertial_sense_ros2__msg__GNSSEphemeris__FIELD_NAME__m_0, 3, 3},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {inertial_sense_ros2__msg__GNSSEphemeris__FIELD_NAME__deln, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {inertial_sense_ros2__msg__GNSSEphemeris__FIELD_NAME__omg_d, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {inertial_sense_ros2__msg__GNSSEphemeris__FIELD_NAME__idot, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {inertial_sense_ros2__msg__GNSSEphemeris__FIELD_NAME__crc, 3, 3},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {inertial_sense_ros2__msg__GNSSEphemeris__FIELD_NAME__crs, 3, 3},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {inertial_sense_ros2__msg__GNSSEphemeris__FIELD_NAME__cuc, 3, 3},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {inertial_sense_ros2__msg__GNSSEphemeris__FIELD_NAME__cus, 3, 3},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {inertial_sense_ros2__msg__GNSSEphemeris__FIELD_NAME__cic, 3, 3},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {inertial_sense_ros2__msg__GNSSEphemeris__FIELD_NAME__cis, 3, 3},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {inertial_sense_ros2__msg__GNSSEphemeris__FIELD_NAME__toes, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {inertial_sense_ros2__msg__GNSSEphemeris__FIELD_NAME__fit, 3, 3},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {inertial_sense_ros2__msg__GNSSEphemeris__FIELD_NAME__f0, 2, 2},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {inertial_sense_ros2__msg__GNSSEphemeris__FIELD_NAME__f1, 2, 2},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {inertial_sense_ros2__msg__GNSSEphemeris__FIELD_NAME__f2, 2, 2},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {inertial_sense_ros2__msg__GNSSEphemeris__FIELD_NAME__tgd, 3, 3},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE_ARRAY,
      4,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {inertial_sense_ros2__msg__GNSSEphemeris__FIELD_NAME__a_dot, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {inertial_sense_ros2__msg__GNSSEphemeris__FIELD_NAME__ndot, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription inertial_sense_ros2__msg__GNSSEphemeris__REFERENCED_TYPE_DESCRIPTIONS[] = {
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
inertial_sense_ros2__msg__GNSSEphemeris__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {inertial_sense_ros2__msg__GNSSEphemeris__TYPE_NAME, 37, 37},
      {inertial_sense_ros2__msg__GNSSEphemeris__FIELDS, 35, 35},
    },
    {inertial_sense_ros2__msg__GNSSEphemeris__REFERENCED_TYPE_DESCRIPTIONS, 3, 3},
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
  "int32 sat \\t\\t# satellite number\n"
  "int32 iode \\t\\t# IODE Issue of Data, Ephemeris (ephemeris version)\n"
  "int32 iodc \\t\\t# IODC Issue of Data, Clock (clock version)\n"
  "int32 sva \\t\\t# SV accuracy (URA index) IRN-IS-200H p.97            \n"
  "int32 svh \\t\\t# SV health GPS/QZS (0:ok)            \n"
  "int32 week \\t\\t# GPS/QZS: gps week, GAL: galileo week\n"
  "int32 code \\t\\t# GPS/QZS: code on L2 * (00=Invalid, 01 = P Code ON, 11 = C/A code ON, 11 = Invalid) * GAL/CMP: data sources\n"
  "int32 flag \\t\\t# GPS/QZS: L2 P data flag (indicates that the NAV data stream was commanded OFF on the P-code of the in-phase component of the L2 channel) *  CMP: nav type\n"
  "GTime toe \\t\\t# Toe\n"
  "GTime toc \\t\\t# clock data reference time (s) (20.3.4.5)\n"
  "GTime ttr \\t\\t# T_trans\n"
  "float64 a \\t\\t# Semi-Major Axis m\n"
  "float64 e \\t\\t# Eccentricity (no units) \n"
  "float64 i0 \\t\\t# Inclination Angle at Reference Time (rad)\n"
  "float64 omg_0 \\t# Longitude of Ascending Node of Orbit Plane at Weekly Epoch (rad)\n"
  "float64 omg \\t# Argument of Perigee (rad)\n"
  "float64 m_0 \\t\\t# Mean Anomaly at Reference Time (rad)\n"
  "float64 deln \\t# Mean Motion Difference From Computed Value (rad)\n"
  "float64 omg_d \\t# Rate of Right Ascension (rad/s)\n"
  "float64 idot \\t# Rate of Inclination Angle (rad/s)\n"
  "float64 crc \\t# Amplitude of the Cosine Harmonic Correction Term to the Orbit Radius\n"
  "float64 crs \\t# Amplitude of the Sine Harmonic Correction Term to the Orbit Radius (m)\n"
  "float64 cuc \\t# Amplitude of the Cosine Harmonic Correction Term to the Argument of Latitude (rad) \n"
  "float64 cus \\t# Amplitude of the Sine Harmonic Correction Term to the Argument of Latitude (rad)\n"
  "float64 cic \\t# Amplitude of the Cosine Harmonic Correction Term to the Angle of Inclination (rad)\n"
  "float64 cis \\t# Amplitude of the Sine Harmonic Correction Term to the Angle of Inclination (rad)\n"
  "float64 toes \\t# Reference Time Ephemeris in week (s)\n"
  "float64 fit \\t# fit interval (h) (0: 4 hours, 1:greater than 4 hours)\n"
  "float64 f0 \\t\\t# SV clock parameters - af0\n"
  "float64 f1 \\t\\t# SV clock parameters - af1\n"
  "float64 f2 \\t\\t# SV clock parameters - af2\n"
  "float64[4] tgd \\t# group delay parameters: GPS/QZS:tgd[0]=TGD (IRN-IS-200H p.103) * GAL:tgd[0]=BGD E5a/E1,tgd[1]=BGD E5b/E1\\t* CMP :tgd[0]=BGD1,tgd[1]=BGD2\n"
  "float64 a_dot \\t# Adot for CNAV\n"
  "float64 ndot \\t# ndot for CNAV";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
inertial_sense_ros2__msg__GNSSEphemeris__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {inertial_sense_ros2__msg__GNSSEphemeris__TYPE_NAME, 37, 37},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 2219, 2219},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
inertial_sense_ros2__msg__GNSSEphemeris__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[4];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 4, 4};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *inertial_sense_ros2__msg__GNSSEphemeris__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *inertial_sense_ros2__msg__GTime__get_individual_type_description_source(NULL);
    sources[3] = *std_msgs__msg__Header__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
