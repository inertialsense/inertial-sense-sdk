/**
 * @file cbor_registry_names.c
 * @brief Field name lookup across every DID in the registry. SDK only: firmware does not carry the names.
 *
 * Generated from the CBOR key registry by is-common scripts/cbor_registry.py -- do not edit.
 *
 * @copyright Copyright (c) 2026 Inertial Sense, Inc.
 */

#include <string.h>
#include "cbor/cbor_registry.h"

/** One DID's name table */
typedef struct
{
    uint16_t                    did;    //!< root DID key
    const cbor_registry_name_t *names;  //!< the DID's field names
    const uint16_t             *parents; //!< the parent of each name, or 0xFFFF
    uint16_t                    count;  //!< entries in names
} cbor_registry_did_names_t;

static const cbor_registry_did_names_t s_dids[] = {
    { CBOR_DID_MAG_CAL_MAP, g_cborNames_MAG_CAL_MAP, g_cborNameParents_MAG_CAL_MAP, CBOR_MAG_CAL_MAP_NAME_COUNT },
    { CBOR_DID_CONFIG_SYSTEM, g_cborNames_CONFIG_SYSTEM, g_cborNameParents_CONFIG_SYSTEM, CBOR_CONFIG_SYSTEM_NAME_COUNT },
    { CBOR_DID_CONFIG_INS, g_cborNames_CONFIG_INS, g_cborNameParents_CONFIG_INS, CBOR_CONFIG_INS_NAME_COUNT },
    { CBOR_DID_CONFIG_MAG, g_cborNames_CONFIG_MAG, g_cborNameParents_CONFIG_MAG, CBOR_CONFIG_MAG_NAME_COUNT },
    { CBOR_DID_CONFIG_GROUND_VEHICLE, g_cborNames_CONFIG_GROUND_VEHICLE, g_cborNameParents_CONFIG_GROUND_VEHICLE, CBOR_CONFIG_GROUND_VEHICLE_NAME_COUNT },
    { CBOR_DID_CONFIG_PORTS, g_cborNames_CONFIG_PORTS, g_cborNameParents_CONFIG_PORTS, CBOR_CONFIG_PORTS_NAME_COUNT },
    { CBOR_DID_CONFIG_SENSORS, g_cborNames_CONFIG_SENSORS, g_cborNameParents_CONFIG_SENSORS, CBOR_CONFIG_SENSORS_NAME_COUNT },
    { CBOR_DID_CONFIG_IO, g_cborNames_CONFIG_IO, g_cborNameParents_CONFIG_IO, CBOR_CONFIG_IO_NAME_COUNT },
    { CBOR_DID_CONFIG_GNSS, g_cborNames_CONFIG_GNSS, g_cborNameParents_CONFIG_GNSS, CBOR_CONFIG_GNSS_NAME_COUNT },
    { CBOR_DID_CONFIG_CAN, g_cborNames_CONFIG_CAN, g_cborNameParents_CONFIG_CAN, CBOR_CONFIG_CAN_NAME_COUNT },
    { CBOR_DID_STATE, g_cborNames_STATE, g_cborNameParents_STATE, CBOR_STATE_NAME_COUNT },
    { CBOR_DID_RMC_STORED, g_cborNames_RMC_STORED, g_cborNameParents_RMC_STORED, CBOR_RMC_STORED_NAME_COUNT },
    { CBOR_DID_RMC, g_cborNames_RMC, g_cborNameParents_RMC, CBOR_RMC_NAME_COUNT },
    { CBOR_DID_ROUTE_FILTER, g_cborNames_ROUTE_FILTER, g_cborNameParents_ROUTE_FILTER, CBOR_ROUTE_FILTER_NAME_COUNT },
};

/**
 * @brief Finds a DID's name table.
 * @param did  root DID key
 * @return the table, or NULL if the DID has no field keys
 */
static const cbor_registry_did_names_t *did_names(uint16_t did)
{
    for (size_t i = 0; i < sizeof(s_dids) / sizeof(s_dids[0]); i++)
        if (s_dids[i].did == did)
            return &s_dids[i];
    return NULL;
}

const char *cbor_registry_field_name(uint16_t did, uint16_t key)
{
    const cbor_registry_did_names_t *t = did_names(did);
    for (uint16_t i = 0; t && i < t->count; i++)
        if (t->names[i].key == key)
            return t->names[i].name;
    return NULL;
}

int cbor_registry_field_key(uint16_t did, const char *name)
{
    const cbor_registry_did_names_t *t = did_names(did);
    for (uint16_t i = 0; t && i < t->count; i++)
        if (strcmp(t->names[i].name, name) == 0)
            return t->names[i].key;
    return -1;
}

/** @return parent as stored in a name table: 0xFFFF for CBOR_REGISTRY_NO_PARENT */
static uint16_t stored_parent(int parent)
{
    return parent < 0 ? 0xFFFF : (uint16_t)parent;
}

int cbor_registry_field_index(uint16_t did, int parent, uint16_t key)
{
    const cbor_registry_did_names_t *t = did_names(did);
    for (uint16_t i = 0; t && i < t->count; i++)
        if (t->parents[i] == stored_parent(parent) && t->names[i].key == key)
            return i;
    return -1;
}

int cbor_registry_field_index_by_name(uint16_t did, int parent, const char *name)
{
    const cbor_registry_did_names_t *t = did_names(did);
    for (uint16_t i = 0; t && i < t->count; i++)
        if (t->parents[i] == stored_parent(parent) && strcmp(t->names[i].name, name) == 0)
            return i;
    return -1;
}

const cbor_registry_name_t *cbor_registry_field_at(uint16_t did, int index)
{
    const cbor_registry_did_names_t *t = did_names(did);
    return (t && index >= 0 && index < t->count) ? &t->names[index] : NULL;
}

/* Root DID keys and names */
static const cbor_registry_name_t s_didNames[] = {
    { CBOR_DID_CALIBRATION, "CALIBRATION" },
    { CBOR_DID_MAG_CAL_MAP, "MAG_CAL_MAP" },
    { CBOR_DID_CONFIG_SYSTEM, "CONFIG_SYSTEM" },
    { CBOR_DID_CONFIG_INS, "CONFIG_INS" },
    { CBOR_DID_CONFIG_MAG, "CONFIG_MAG" },
    { CBOR_DID_CONFIG_GROUND_VEHICLE, "CONFIG_GROUND_VEHICLE" },
    { CBOR_DID_CONFIG_PORTS, "CONFIG_PORTS" },
    { CBOR_DID_CONFIG_SENSORS, "CONFIG_SENSORS" },
    { CBOR_DID_CONFIG_IO, "CONFIG_IO" },
    { CBOR_DID_CONFIG_GNSS, "CONFIG_GNSS" },
    { CBOR_DID_CONFIG_CAN, "CONFIG_CAN" },
    { CBOR_DID_STATE, "STATE" },
    { CBOR_DID_NMEA_PIMU, "NMEA_PIMU" },
    { CBOR_DID_NMEA_PPIMU, "NMEA_PPIMU" },
    { CBOR_DID_NMEA_PRIMU, "NMEA_PRIMU" },
    { CBOR_DID_NMEA_PINS1, "NMEA_PINS1" },
    { CBOR_DID_NMEA_PINS2, "NMEA_PINS2" },
    { CBOR_DID_NMEA_PGPSP, "NMEA_PGPSP" },
    { CBOR_DID_NMEA_GNGGA, "NMEA_GNGGA" },
    { CBOR_DID_NMEA_GNGLL, "NMEA_GNGLL" },
    { CBOR_DID_NMEA_GNGSA, "NMEA_GNGSA" },
    { CBOR_DID_NMEA_GNRMC, "NMEA_GNRMC" },
    { CBOR_DID_NMEA_GNZDA, "NMEA_GNZDA" },
    { CBOR_DID_NMEA_PASHR, "NMEA_PASHR" },
    { CBOR_DID_NMEA_PSTRB, "NMEA_PSTRB" },
    { CBOR_DID_NMEA_INFO, "NMEA_INFO" },
    { CBOR_DID_NMEA_GNGSV, "NMEA_GNGSV" },
    { CBOR_DID_NMEA_GNVTG, "NMEA_GNVTG" },
    { CBOR_DID_NMEA_INTEL, "NMEA_INTEL" },
    { CBOR_DID_NMEA_POWGPS, "NMEA_POWGPS" },
    { CBOR_DID_NMEA_POWTLV, "NMEA_POWTLV" },
    { CBOR_DID_CAPABILITIES, "CAPABILITIES" },
    { CBOR_DID_RMC_STORED, "RMC_STORED" },
    { CBOR_DID_RMC, "RMC" },
    { CBOR_DID_ROUTE_FILTER, "ROUTE_FILTER" },
};

/* Message keys and message-type option keys, with their names */
static const cbor_registry_name_t s_msgKeyNames[] = {
    { 0xC000, "MSG_TYPE" },
    { 0xC001, "MSG_SRC_DEV_ID" },
    { 0xC002, "MSG_DST_DEV_FILTER" },
    { 0xC003, "MSG_REQ_ID" },
    { 0xC004, "MSG_ACK_TYPE" },
    { 0xC005, "MSG_ROUTE" },
    { 0xC006, "MSG_TIME_GPS_NS" },
    { 0xC007, "MSG_UPTIME_NS" },
    { 0xC008, "MSG_STATUS" },
    { 0xC009, "MSG_ERR_KEYS" },
    { 0xC00A, "MSG_FIELDS" },
    { 0xC00B, "MSG_CURSOR" },
    { 0xC00C, "MSG_OPTION_NO_TIME" },
    { 0x9300, "SUB_RATE_PERIOD" },
    { 0x9301, "SUB_RATE_MULTI" },
    { 0x9302, "SUB_PORTS" },
    { 0x9303, "SUB_OPTION_PERSIST" },
};

static const cbor_registry_name_t s_enum_can_message[] = {
    { 0, "INS_TIME" },
    { 1, "INS_STATUS" },
    { 2, "INS_EULER" },
    { 3, "INS_QUATN2B" },
    { 4, "INS_QUATE2B" },
    { 5, "INS_UVW" },
    { 6, "INS_VE" },
    { 7, "INS_LAT" },
    { 8, "INS_LON" },
    { 9, "INS_ALT" },
    { 10, "INS_NORTH_EAST" },
    { 11, "INS_DOWN" },
    { 12, "INS_ECEF_X" },
    { 13, "INS_ECEF_Y" },
    { 14, "INS_ECEF_Z" },
    { 15, "INS_MSL" },
    { 16, "PREINT_PX" },
    { 17, "PREINT_QY" },
    { 18, "PREINT_RZ" },
    { 19, "DUAL_PX" },
    { 20, "DUAL_QY" },
    { 21, "DUAL_RZ" },
    { 22, "GNSS1_POS" },
    { 23, "GNSS2_POS" },
    { 24, "GNSS1_RTK_POS_REL" },
    { 25, "GNSS2_RTK_CMP_REL" },
    { 26, "ROLL_ROLLRATE" },
};
static const cbor_registry_name_t s_enum_receiver_id[] = {
    { 0, "GNSS1" },
    { 1, "GNSS2" },
};
static const cbor_registry_name_t s_enum_antenna_id[] = {
    { 0, "ANT1" },
    { 1, "ANT2" },
};
static const cbor_registry_name_t s_enum_base_stream[] = {
    { 0, "GNSS1_UBLOX" },
    { 1, "GNSS1_RTCM3" },
    { 2, "GNSS2_UBLOX" },
    { 3, "GNSS2_RTCM3" },
};
static const cbor_registry_name_t s_enum_pin_id[] = {
    { 0, "G1G2" },
    { 1, "G5G8" },
    { 2, "G6G7" },
    { 3, "G9" },
    { 4, "G11" },
    { 5, "G12" },
    { 6, "G13" },
    { 7, "G15" },
};
static const cbor_registry_name_t s_enum_imu_slot[] = {
    { 0, "IMU1" },
    { 1, "IMU2" },
    { 2, "IMU3" },
    { 3, "IMU4" },
    { 4, "IMU5" },
};
static const cbor_registry_name_t s_enum_imu_channel[] = {
    { 0, "PRIMARY" },
    { 1, "HIGH_G" },
};
static const cbor_registry_name_t s_enum_mag_id[] = {
    { 0, "MAG1" },
};
static const cbor_registry_name_t s_enum_baro_id[] = {
    { 0, "BARO1" },
};
static const cbor_registry_name_t s_enum_port_id[] = {
    { 0, "SER0" },
    { 1, "SER1" },
    { 2, "SER2" },
    { 3, "USB" },
    { 4, "SPI" },
};
static const cbor_registry_name_t s_enum_msg_type[] = {
    { 1, "DATA" },
    { 2, "GET" },
    { 3, "SUBSCRIBE" },
};
static const cbor_registry_name_t s_enum_msg_type_flag[] = {
    { 128, "REPLY" },
};
static const cbor_registry_name_t s_enum_msg_status[] = {
    { 0, "OK" },
    { 1, "BAD_REQUEST" },
    { 2, "BAD_VALUE" },
    { 3, "UNSUPPORTED" },
    { 4, "RESP_TOO_LARGE" },
    { 5, "FORWARD_DENIED" },
    { 6, "READ_ONLY" },
    { 7, "ERROR" },
    { 8, "REQ_TOO_LARGE" },
    { 9, "UNREACHABLE" },
};
static const cbor_registry_name_t s_enum_ack_type[] = {
    { 0, "ACK_NONE" },
    { 1, "ACK_MINIMAL" },
    { 2, "ACK_DETAILED" },
};
static const cbor_registry_name_t s_enum_route_action[] = {
    { 0, "DENY" },
    { 1, "ALLOW" },
};
static const cbor_registry_name_t s_enum_rmc_preset[] = {
    { 0, "NONE" },
    { 1, "IMX_PPD_NO_IMU" },
    { 2, "IMX_PPD" },
    { 3, "IMX_PPD_IMUS_RAW" },
    { 4, "IMX_PPD_IMUS_UNCAL" },
    { 5, "INS" },
    { 6, "IMX_PPD_RTK_DBG" },
    { 7, "IMX_PPD_GROUND_VEHICLE" },
    { 8, "ALLAN_VARIANCE" },
    { 9, "ALLAN_VARIANCE_IMUS" },
    { 10, "GNSS" },
    { 11, "GPX_PPD" },
};
static const cbor_registry_name_t s_enum_nmea_speed_filter[] = {
    { 0, "DEFAULT" },
    { 1, "ENABLE" },
    { 2, "DISABLE" },
};

/* Every registry enum */
static const cbor_registry_enum_t s_enums[] = {
    { "can_message", s_enum_can_message, sizeof(s_enum_can_message) / sizeof(s_enum_can_message[0]) },
    { "receiver_id", s_enum_receiver_id, sizeof(s_enum_receiver_id) / sizeof(s_enum_receiver_id[0]) },
    { "antenna_id", s_enum_antenna_id, sizeof(s_enum_antenna_id) / sizeof(s_enum_antenna_id[0]) },
    { "base_stream", s_enum_base_stream, sizeof(s_enum_base_stream) / sizeof(s_enum_base_stream[0]) },
    { "pin_id", s_enum_pin_id, sizeof(s_enum_pin_id) / sizeof(s_enum_pin_id[0]) },
    { "imu_slot", s_enum_imu_slot, sizeof(s_enum_imu_slot) / sizeof(s_enum_imu_slot[0]) },
    { "imu_channel", s_enum_imu_channel, sizeof(s_enum_imu_channel) / sizeof(s_enum_imu_channel[0]) },
    { "mag_id", s_enum_mag_id, sizeof(s_enum_mag_id) / sizeof(s_enum_mag_id[0]) },
    { "baro_id", s_enum_baro_id, sizeof(s_enum_baro_id) / sizeof(s_enum_baro_id[0]) },
    { "port_id", s_enum_port_id, sizeof(s_enum_port_id) / sizeof(s_enum_port_id[0]) },
    { "msg_type", s_enum_msg_type, sizeof(s_enum_msg_type) / sizeof(s_enum_msg_type[0]) },
    { "msg_type_flag", s_enum_msg_type_flag, sizeof(s_enum_msg_type_flag) / sizeof(s_enum_msg_type_flag[0]) },
    { "msg_status", s_enum_msg_status, sizeof(s_enum_msg_status) / sizeof(s_enum_msg_status[0]) },
    { "ack_type", s_enum_ack_type, sizeof(s_enum_ack_type) / sizeof(s_enum_ack_type[0]) },
    { "route_action", s_enum_route_action, sizeof(s_enum_route_action) / sizeof(s_enum_route_action[0]) },
    { "rmc_preset", s_enum_rmc_preset, sizeof(s_enum_rmc_preset) / sizeof(s_enum_rmc_preset[0]) },
    { "nmea_speed_filter", s_enum_nmea_speed_filter, sizeof(s_enum_nmea_speed_filter) / sizeof(s_enum_nmea_speed_filter[0]) },
};

/* Keys whose value is an enum: DID (0xFFFF for message and message-type keys), key, enum */
typedef struct
{
    uint16_t    did;    //!< root DID key, or 0xFFFF
    uint16_t    key;    //!< the key
    uint8_t     e;      //!< index into s_enums
} key_enum_t;

static const key_enum_t s_keyEnums[] = {
    { 0xFFFF, 0xC000, 10 },
    { 0xFFFF, 0xC004, 13 },
    { 0xFFFF, 0xC008, 12 },
    { CBOR_DID_RMC_STORED, 0x8001, 15 },
    { CBOR_DID_RMC_STORED, 0x8002, 16 },
    { CBOR_DID_RMC, 0x8001, 15 },
    { CBOR_DID_RMC, 0x8002, 16 },
    { CBOR_DID_ROUTE_FILTER, 0x0003, 14 },
};

/* Domains with a schema, over their view structs */
typedef struct
{
    uint16_t                did;    //!< root DID key
    const cbor_schema_t    *schema; //!< its schema
} did_schema_t;

static const did_schema_t s_schemas[] = {
    { CBOR_DID_CONFIG_SYSTEM, &g_cborSchema_CONFIG_SYSTEM },
    { CBOR_DID_CONFIG_INS, &g_cborSchema_CONFIG_INS },
    { CBOR_DID_CONFIG_MAG, &g_cborSchema_CONFIG_MAG },
    { CBOR_DID_CONFIG_GROUND_VEHICLE, &g_cborSchema_CONFIG_GROUND_VEHICLE },
    { CBOR_DID_CONFIG_PORTS, &g_cborSchema_CONFIG_PORTS },
    { CBOR_DID_CONFIG_SENSORS, &g_cborSchema_CONFIG_SENSORS },
    { CBOR_DID_CONFIG_IO, &g_cborSchema_CONFIG_IO },
    { CBOR_DID_CONFIG_GNSS, &g_cborSchema_CONFIG_GNSS },
    { CBOR_DID_CONFIG_CAN, &g_cborSchema_CONFIG_CAN },
    { CBOR_DID_STATE, &g_cborSchema_STATE },
};

#define COUNT(a)    (sizeof(a) / sizeof((a)[0]))

const char *cbor_registry_did_name(uint16_t did)
{
    for (size_t i = 0; i < COUNT(s_didNames); i++)
        if (s_didNames[i].key == did)
            return s_didNames[i].name;
    return NULL;
}

int cbor_registry_did_key(const char *name)
{
    for (size_t i = 0; i < COUNT(s_didNames); i++)
        if (strcmp(s_didNames[i].name, name) == 0)
            return s_didNames[i].key;
    return -1;
}

const char *cbor_registry_msg_key_name(uint16_t key)
{
    for (size_t i = 0; i < COUNT(s_msgKeyNames); i++)
        if (s_msgKeyNames[i].key == key)
            return s_msgKeyNames[i].name;
    return NULL;
}

int cbor_registry_msg_key(const char *name)
{
    for (size_t i = 0; i < COUNT(s_msgKeyNames); i++)
        if (strcmp(s_msgKeyNames[i].name, name) == 0)
            return s_msgKeyNames[i].key;
    return -1;
}

const cbor_registry_enum_t *cbor_registry_key_enum(uint16_t did, uint16_t key)
{
    uint16_t scope = (key >= 0x9000) ? 0xFFFF : did;
    for (size_t i = 0; i < COUNT(s_keyEnums); i++)
        if (s_keyEnums[i].did == scope && s_keyEnums[i].key == key)
            return &s_enums[s_keyEnums[i].e];
    return NULL;
}

const cbor_schema_t *cbor_registry_did_schema(uint16_t did)
{
    for (size_t i = 0; i < COUNT(s_schemas); i++)
        if (s_schemas[i].did == did)
            return s_schemas[i].schema;
    return NULL;
}
