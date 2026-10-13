/**
 * @file flash_cfg_cbor.c
 * @brief CBOR configuration encoding/decoding implementation
 *
 * @copyright Copyright (c) 2026 Inertial Sense, Inc.
 */

#include <string.h>
#include <math.h>
#include "flash_cfg_cbor.h"
#include "nanocbor/nanocbor.h"
#include "ISConstants.h"

/*===========================================================================
 * Field descriptor tables
 *===========================================================================*/

#define FIELD(id, ftype, count, struct_type, member) \
    { (id), (ftype), (count), offsetof(struct_type, member) }

static const flash_cfg_field_desc_t s_imx_fields[] = {
    FIELD(IMX_CFG_FIELD_STARTUP_IMU_DT,       FCTYPE_UINT32,       1, nvm_flash_cfg_t, startupImuDtMs),
    FIELD(IMX_CFG_FIELD_STARTUP_NAV_DT,       FCTYPE_UINT32,       1, nvm_flash_cfg_t, startupNavDtMs),
    FIELD(IMX_CFG_FIELD_SER0_BAUD,            FCTYPE_UINT32,       1, nvm_flash_cfg_t, ser0BaudRate),
    FIELD(IMX_CFG_FIELD_SER1_BAUD,            FCTYPE_UINT32,       1, nvm_flash_cfg_t, ser1BaudRate),
    FIELD(IMX_CFG_FIELD_INS_ROTATION,         FCTYPE_FLOAT_ARRAY,  3, nvm_flash_cfg_t, insRotation),
    FIELD(IMX_CFG_FIELD_INS_OFFSET,           FCTYPE_FLOAT_ARRAY,  3, nvm_flash_cfg_t, insOffset),
    FIELD(IMX_CFG_FIELD_GNSS1_ANT_OFFSET,     FCTYPE_FLOAT_ARRAY,  3, nvm_flash_cfg_t, gnss1AntOffset),
    FIELD(IMX_CFG_FIELD_DYNAMIC_MODEL,        FCTYPE_UINT8,        1, nvm_flash_cfg_t, dynamicModel),
    FIELD(IMX_CFG_FIELD_DEBUG,                FCTYPE_UINT8,        1, nvm_flash_cfg_t, debug),
    FIELD(IMX_CFG_FIELD_GNSS_SAT_SIG_CONST,  FCTYPE_UINT16,       1, nvm_flash_cfg_t, gnssSatSigConst),
    FIELD(IMX_CFG_FIELD_SYS_CFG_BITS,        FCTYPE_UINT32,       1, nvm_flash_cfg_t, sysCfgBits),
    FIELD(IMX_CFG_FIELD_REF_LLA,             FCTYPE_DOUBLE_ARRAY, 3, nvm_flash_cfg_t, refLla),
    FIELD(IMX_CFG_FIELD_LAST_LLA,            FCTYPE_DOUBLE_ARRAY, 3, nvm_flash_cfg_t, lastLla),
    FIELD(IMX_CFG_FIELD_LAST_LLA_TOW_MS,     FCTYPE_UINT32,       1, nvm_flash_cfg_t, lastLlaTimeOfWeekMs),
    FIELD(IMX_CFG_FIELD_LAST_LLA_WEEK,       FCTYPE_UINT32,       1, nvm_flash_cfg_t, lastLlaWeek),
    FIELD(IMX_CFG_FIELD_LAST_LLA_UPDATE_DIST,FCTYPE_FLOAT,        1, nvm_flash_cfg_t, lastLlaUpdateDistance),
    FIELD(IMX_CFG_FIELD_IO_CONFIG,           FCTYPE_UINT32,       1, nvm_flash_cfg_t, ioConfig),
    FIELD(IMX_CFG_FIELD_PLATFORM_CONFIG,     FCTYPE_UINT32,       1, nvm_flash_cfg_t, platformConfig),
    FIELD(IMX_CFG_FIELD_GNSS2_ANT_OFFSET,    FCTYPE_FLOAT_ARRAY,  3, nvm_flash_cfg_t, gnss2AntOffset),
    FIELD(IMX_CFG_FIELD_ZERO_VEL_ROTATION,   FCTYPE_FLOAT_ARRAY,  3, nvm_flash_cfg_t, zeroVelRotation),
    FIELD(IMX_CFG_FIELD_ZERO_VEL_OFFSET,     FCTYPE_FLOAT_ARRAY,  3, nvm_flash_cfg_t, zeroVelOffset),
    FIELD(IMX_CFG_FIELD_GPS_TIME_USER_DELAY, FCTYPE_FLOAT,        1, nvm_flash_cfg_t, gnssTimeUserDelay),
    FIELD(IMX_CFG_FIELD_MAG_DECLINATION,     FCTYPE_FLOAT,        1, nvm_flash_cfg_t, magDeclination),
    FIELD(IMX_CFG_FIELD_GPS_TIME_SYNC_PERIOD,FCTYPE_UINT32,       1, nvm_flash_cfg_t, gnssTimeSyncPeriodMs),
    FIELD(IMX_CFG_FIELD_STARTUP_GNSS_DT,     FCTYPE_UINT32,       1, nvm_flash_cfg_t, startupGnssDtMs),
    FIELD(IMX_CFG_FIELD_RTK_CFG_BITS,        FCTYPE_UINT32,       1, nvm_flash_cfg_t, RTKCfgBits),
    FIELD(IMX_CFG_FIELD_SENSOR_CONFIG,       FCTYPE_UINT32,       1, nvm_flash_cfg_t, sensorConfig),
    FIELD(IMX_CFG_FIELD_GNSS_MIN_ELEVATION,  FCTYPE_FLOAT,        1, nvm_flash_cfg_t, gnssMinimumElevation),
    FIELD(IMX_CFG_FIELD_SER2_BAUD,           FCTYPE_UINT32,       1, nvm_flash_cfg_t, ser2BaudRate),
    /* wheel_config_t — flattened */
    FIELD(IMX_CFG_FIELD_WHEEL_CFG_BITS,      FCTYPE_UINT32,       1, nvm_flash_cfg_t, wheelConfig.bits),
    FIELD(IMX_CFG_FIELD_WHEEL_E_B2W,         FCTYPE_FLOAT_ARRAY,  3, nvm_flash_cfg_t, wheelConfig.transform.e_b2w),
    FIELD(IMX_CFG_FIELD_WHEEL_E_B2W_SIGMA,   FCTYPE_FLOAT_ARRAY,  3, nvm_flash_cfg_t, wheelConfig.transform.e_b2w_sigma),
    FIELD(IMX_CFG_FIELD_WHEEL_T_B2W,         FCTYPE_FLOAT_ARRAY,  3, nvm_flash_cfg_t, wheelConfig.transform.t_b2w),
    FIELD(IMX_CFG_FIELD_WHEEL_T_B2W_SIGMA,   FCTYPE_FLOAT_ARRAY,  3, nvm_flash_cfg_t, wheelConfig.transform.t_b2w_sigma),
    FIELD(IMX_CFG_FIELD_WHEEL_TRACK_WIDTH,   FCTYPE_FLOAT,        1, nvm_flash_cfg_t, wheelConfig.track_width),
    FIELD(IMX_CFG_FIELD_WHEEL_RADIUS,        FCTYPE_FLOAT,        1, nvm_flash_cfg_t, wheelConfig.radius),
    /* remaining scalar fields */
    FIELD(IMX_CFG_FIELD_MAG_INTERFERENCE_THRESH, FCTYPE_FLOAT,    1, nvm_flash_cfg_t, magInterferenceThreshold),
    FIELD(IMX_CFG_FIELD_MAG_CAL_QUAL_THRESH,     FCTYPE_FLOAT,    1, nvm_flash_cfg_t, magCalibrationQualityThreshold),
    FIELD(IMX_CFG_FIELD_GNSS_CN0_MINIMUM,        FCTYPE_UINT8,    1, nvm_flash_cfg_t, gnssCn0Minimum),
    FIELD(IMX_CFG_FIELD_GNSS_CN0_DYN_MIN_OFFSET, FCTYPE_UINT8,    1, nvm_flash_cfg_t, gnssCn0DynMinOffset),
    FIELD(IMX_CFG_FIELD_IMU_REJECT_GYRO_LOW,     FCTYPE_UINT8,    1, nvm_flash_cfg_t, imuRejectThreshGyroLow),
    FIELD(IMX_CFG_FIELD_IMU_REJECT_GYRO_HIGH,    FCTYPE_UINT8,    1, nvm_flash_cfg_t, imuRejectThreshGyroHigh),
    FIELD(IMX_CFG_FIELD_IMU_SHOCK_DETECT_LAT,    FCTYPE_UINT8,    1, nvm_flash_cfg_t, imuShockDetectLatencyMs),
    FIELD(IMX_CFG_FIELD_IMU_SHOCK_REJECT_LATCH,  FCTYPE_UINT8,    1, nvm_flash_cfg_t, imuShockRejectLatchMs),
    FIELD(IMX_CFG_FIELD_IMU_SHOCK_OPTIONS,       FCTYPE_UINT8,    1, nvm_flash_cfg_t, imuShockOptions),
    FIELD(IMX_CFG_FIELD_IMU_SHOCK_DELTA_ACC_HI,  FCTYPE_UINT8,    1, nvm_flash_cfg_t, imuShockDeltaAccHighThreshold),
    FIELD(IMX_CFG_FIELD_IMU_SHOCK_DELTA_ACC_LO,  FCTYPE_UINT8,    1, nvm_flash_cfg_t, imuShockDeltaAccLowThreshold),
    FIELD(IMX_CFG_FIELD_IMU_SHOCK_DELTA_GYRO_HI, FCTYPE_UINT8,    1, nvm_flash_cfg_t, imuShockDeltaGyroHighThreshold),
    FIELD(IMX_CFG_FIELD_IMU_SHOCK_DELTA_GYRO_LO, FCTYPE_UINT8,    1, nvm_flash_cfg_t, imuShockDeltaGyroLowThreshold),
    FIELD(IMX_CFG_FIELD_IO_CONFIG_2,             FCTYPE_UINT8,    1, nvm_flash_cfg_t, ioConfig2),
};

static const flash_cfg_field_desc_t s_gpx_fields[] = {
    FIELD(GPX_CFG_FIELD_SER0_BAUD,            FCTYPE_UINT32,       1, gpx_flash_cfg_t, ser0BaudRate),
    FIELD(GPX_CFG_FIELD_SER1_BAUD,            FCTYPE_UINT32,       1, gpx_flash_cfg_t, ser1BaudRate),
    FIELD(GPX_CFG_FIELD_SER2_BAUD,            FCTYPE_UINT32,       1, gpx_flash_cfg_t, ser2BaudRate),
    FIELD(GPX_CFG_FIELD_STARTUP_GNSS_DT,      FCTYPE_UINT32,       1, gpx_flash_cfg_t, startupGnssDtMs),
    FIELD(GPX_CFG_FIELD_GNSS1_ANT_OFFSET,     FCTYPE_FLOAT_ARRAY,  3, gpx_flash_cfg_t, gnss1AntOffset),
    FIELD(GPX_CFG_FIELD_GNSS2_ANT_OFFSET,     FCTYPE_FLOAT_ARRAY,  3, gpx_flash_cfg_t, gnss2AntOffset),
    FIELD(GPX_CFG_FIELD_GNSS_SAT_SIG_CONST,   FCTYPE_UINT16,       1, gpx_flash_cfg_t, gnssSatSigConst),
    FIELD(GPX_CFG_FIELD_DYNAMIC_MODEL,        FCTYPE_UINT8,        1, gpx_flash_cfg_t, dynamicModel),
    FIELD(GPX_CFG_FIELD_DEBUG,                FCTYPE_UINT8,        1, gpx_flash_cfg_t, debug),
    FIELD(GPX_CFG_FIELD_GPS_TIME_SYNC_PERIOD, FCTYPE_UINT32,       1, gpx_flash_cfg_t, gnssTimeSyncPeriodMs),
    FIELD(GPX_CFG_FIELD_GPS_TIME_USER_DELAY,  FCTYPE_FLOAT,        1, gpx_flash_cfg_t, gnssTimeUserDelay),
    FIELD(GPX_CFG_FIELD_GNSS_MIN_ELEVATION,    FCTYPE_FLOAT,        1, gpx_flash_cfg_t, gnssMinimumElevation),
    FIELD(GPX_CFG_FIELD_RTK_CFG_BITS,         FCTYPE_UINT32,       1, gpx_flash_cfg_t, RTKCfgBits),
    FIELD(GPX_CFG_FIELD_GNSS_CN0_MINIMUM,     FCTYPE_UINT8,        1, gpx_flash_cfg_t, gnssCn0Minimum),
    FIELD(GPX_CFG_FIELD_GNSS_CN0_DYN_MIN_OFFSET, FCTYPE_UINT8,    1, gpx_flash_cfg_t, gnssCn0DynMinOffset),
    FIELD(GPX_CFG_FIELD_SYS_CFG_BITS,         FCTYPE_UINT32,       1, gpx_flash_cfg_t, sysCfgBits),
    FIELD(GPX_CFG_FIELD_REF_LLA,              FCTYPE_DOUBLE_ARRAY, 3, gpx_flash_cfg_t, refLla),
};

static const flash_cfg_field_desc_t s_can_fields[] = {
    FIELD(CAN_CFG_FIELD_PERIOD_MULT,   FCTYPE_UINT16_ARRAY, NUM_CIDS, can_config_t, can_period_mult),
    FIELD(CAN_CFG_FIELD_TRANSMIT_ADDR, FCTYPE_UINT32_ARRAY, NUM_CIDS, can_config_t, can_transmit_address),
    FIELD(CAN_CFG_FIELD_SETTING,       FCTYPE_UINT16,       1,        can_config_t, can_setting),
    FIELD(CAN_CFG_FIELD_RECEIVE_ADDR,  FCTYPE_UINT32,       1,        can_config_t, can_receive_address),
};

static const flash_cfg_field_desc_t s_rmc_port_fields[] = {
    FIELD(RMC_CFG_FIELD_BITS,        FCTYPE_UINT64,      1,                 rmci_t, rmc.bits),
    FIELD(RMC_CFG_FIELD_OPTIONS,     FCTYPE_UINT32,      1,                 rmci_t, rmc.options),
    FIELD(RMC_CFG_FIELD_PERIOD_MULT, FCTYPE_UINT8_ARRAY, DID_COUNT,         rmci_t, periodMultiple),
    FIELD(RMC_CFG_FIELD_NMEA_BITS,   FCTYPE_UINT32,      1,                 rmci_t, rmcNmea.nmeaBits),
    FIELD(RMC_CFG_FIELD_NMEA_PERIOD, FCTYPE_UINT8_ARRAY, NMEA_MSG_ID_COUNT, rmci_t, rmcNmea.nmeaPeriod),
};

#undef FIELD

/* flash_cfg_field_desc_t.count is a uint8_t */
STATIC_ASSERT(DID_COUNT <= UINT8_MAX);
STATIC_ASSERT(NMEA_MSG_ID_COUNT <= UINT8_MAX);
STATIC_ASSERT(NUM_CIDS <= UINT8_MAX);
STATIC_ASSERT(sizeof(s_rmc_port_fields) / sizeof(s_rmc_port_fields[0]) == RMC_CFG_FIELDS_PER_PORT);
STATIC_ASSERT(NUM_COM_PORTS <= RMC_CFG_MAX_PORTS);

#define TABLE_COUNT(table) ((uint16_t)(sizeof(table) / sizeof((table)[0])))

const flash_cfg_field_desc_t *flash_cfg_cbor_imx_fields(uint16_t *count)
{
    *count = TABLE_COUNT(s_imx_fields);
    return s_imx_fields;
}

const flash_cfg_field_desc_t *flash_cfg_cbor_gpx_fields(uint16_t *count)
{
    *count = TABLE_COUNT(s_gpx_fields);
    return s_gpx_fields;
}

const flash_cfg_field_desc_t *flash_cfg_cbor_can_fields(uint16_t *count)
{
    *count = TABLE_COUNT(s_can_fields);
    return s_can_fields;
}

const flash_cfg_field_desc_t *flash_cfg_cbor_rmc_port_fields(uint16_t *count)
{
    *count = TABLE_COUNT(s_rmc_port_fields);
    return s_rmc_port_fields;
}

/*===========================================================================
 * Field helpers
 *===========================================================================*/

/**
 * @brief Returns the in-memory byte size of a single element of the given field type.
 *
 * @param type - field type (an FCTYPE_* value)
 *
 * @return element size in bytes (1, 2, 4, or 8), or 0 for an unrecognized type
 */
static size_t field_element_size(uint8_t type)
{
    switch (type) {
    case FCTYPE_UINT8:          return 1;
    case FCTYPE_UINT16:         return 2;
    case FCTYPE_UINT32:         return 4;
    case FCTYPE_UINT64:         return 8;
    case FCTYPE_FLOAT:
    case FCTYPE_FLOAT_ARRAY:    return 4;
    case FCTYPE_DOUBLE:
    case FCTYPE_DOUBLE_ARRAY:   return 8;
    case FCTYPE_UINT8_ARRAY:    return 1;
    case FCTYPE_UINT16_ARRAY:   return 2;
    case FCTYPE_UINT32_ARRAY:   return 4;
    default:                    return 0;
    }
}

/**
 * @brief Returns whether a field type is encoded as a CBOR array.
 *
 * @param type - field type (an FCTYPE_* value)
 *
 * @return true for the *_ARRAY types
 */
static bool field_is_array(uint8_t type)
{
    switch (type) {
    case FCTYPE_FLOAT_ARRAY:
    case FCTYPE_DOUBLE_ARRAY:
    case FCTYPE_UINT8_ARRAY:
    case FCTYPE_UINT16_ARRAY:
    case FCTYPE_UINT32_ARRAY:   return true;
    default:                    return false;
    }
}

/**
 * @brief Returns the total byte size occupied by a field, accounting for array count.
 *
 * @param f - field descriptor to size
 *
 * @return field_element_size(f->type) * f->count
 */
static size_t field_total_size(const flash_cfg_field_desc_t *f)
{
    return field_element_size(f->type) * f->count;
}

/**
 * @brief Linearly searches a field descriptor table for the entry with a matching field id.
 *
 * @param fields - field descriptor table to search
 * @param count - number of entries in fields
 * @param id - field id to look up
 *
 * @return pointer to the matching descriptor, or NULL if no entry has that id
 */
static const flash_cfg_field_desc_t *find_field_by_id(
    const flash_cfg_field_desc_t *fields, size_t count, uint32_t id)
{
    for (size_t i = 0; i < count; i++) {
        if (fields[i].id == id)
            return &fields[i];
    }
    return NULL;
}

/**
 * @brief Finds the group and field that a CBOR key addresses.
 *
 * @param groups - groups to search, in order
 * @param groupCount - number of entries in groups
 * @param key - CBOR map key (group idBase + field id)
 * @param group - [out] set to the group holding the field when found
 *
 * @return the field descriptor, or NULL if no group defines the key
 */
static const flash_cfg_field_desc_t *find_key(const flash_cfg_cbor_group_t *groups, size_t groupCount,
                                              uint32_t key, const flash_cfg_cbor_group_t **group)
{
    for (size_t g = 0; g < groupCount; g++) {
        if (key < groups[g].idBase)
            continue;
        const flash_cfg_field_desc_t *f = find_field_by_id(groups[g].fields, groups[g].count, key - groups[g].idBase);
        if (f != NULL) {
            *group = &groups[g];
            return f;
        }
    }
    return NULL;
}

/**
 * @brief Finds the field at a position in group-table order (group 0's fields first).
 *
 * @param groups - groups to index
 * @param groupCount - number of entries in groups
 * @param pos - zero-based field position across all groups
 * @param group - [out] set to the group holding the field when found
 *
 * @return the field descriptor, or NULL if pos is past the last field
 */
static const flash_cfg_field_desc_t *field_at(const flash_cfg_cbor_group_t *groups, size_t groupCount,
                                              size_t pos, const flash_cfg_cbor_group_t **group)
{
    for (size_t g = 0; g < groupCount; g++) {
        if (pos < groups[g].count) {
            *group = &groups[g];
            return &groups[g].fields[pos];
        }
        pos -= groups[g].count;
    }
    return NULL;
}

/*===========================================================================
 * Encode helpers
 *===========================================================================*/

/**
 * @brief Encodes a single config field's value into a CBOR encoder, dispatching on the field's type.
 * @note Values are staged into a local, naturally-aligned variable via memcpy before encoding because the source struct field may live at an offset with looser alignment than its type requires (STM32U5/Cortex-M33 faults on unaligned LDRD/STRD).
 *
 * @param enc - CBOR encoder to write the value into
 * @param f - descriptor of the field being encoded (determines type and, for arrays, count)
 * @param val - pointer to the field's raw value within the source config struct
 *
 * @return 0 on success, or -1 if f->type is not a recognized field type
 */
static int encode_field_value(nanocbor_encoder_t *enc,
                              const flash_cfg_field_desc_t *f,
                              const void *val)
{
    switch (f->type) {
    case FCTYPE_UINT8: {
        uint8_t v;
        memcpy(&v, val, sizeof v);
        nanocbor_fmt_uint(enc, v);
        break;
    }
    case FCTYPE_UINT16: {
        uint16_t v;
        memcpy(&v, val, sizeof v);
        nanocbor_fmt_uint(enc, v);
        break;
    }
    case FCTYPE_UINT32: {
        uint32_t v;
        memcpy(&v, val, sizeof v);
        nanocbor_fmt_uint(enc, v);
        break;
    }
    case FCTYPE_UINT64: {
        uint64_t v;
        memcpy(&v, val, sizeof v);
        nanocbor_fmt_uint(enc, v);
        break;
    }
    case FCTYPE_FLOAT: {
        float v;
        memcpy(&v, val, sizeof v);
        nanocbor_fmt_float(enc, v);
        break;
    }
    case FCTYPE_DOUBLE: {
        double v;
        memcpy(&v, val, sizeof v);
        nanocbor_fmt_double(enc, v);
        break;
    }
    case FCTYPE_FLOAT_ARRAY: {
        nanocbor_fmt_array(enc, f->count);
        for (uint8_t i = 0; i < f->count; i++) {
            float v;
            memcpy(&v, (const uint8_t *)val + i * sizeof(float), sizeof v);
            nanocbor_fmt_float(enc, v);
        }
        break;
    }
    case FCTYPE_DOUBLE_ARRAY: {
        nanocbor_fmt_array(enc, f->count);
        for (uint8_t i = 0; i < f->count; i++) {
            double v;
            memcpy(&v, (const uint8_t *)val + i * sizeof(double), sizeof v);
            nanocbor_fmt_double(enc, v);
        }
        break;
    }
    case FCTYPE_UINT8_ARRAY: {
        nanocbor_fmt_array(enc, f->count);
        for (uint8_t i = 0; i < f->count; i++) {
            uint8_t v;
            memcpy(&v, (const uint8_t *)val + i * sizeof(uint8_t), sizeof v);
            nanocbor_fmt_uint(enc, v);
        }
        break;
    }
    case FCTYPE_UINT16_ARRAY: {
        nanocbor_fmt_array(enc, f->count);
        for (uint8_t i = 0; i < f->count; i++) {
            uint16_t v;
            memcpy(&v, (const uint8_t *)val + i * sizeof(uint16_t), sizeof v);
            nanocbor_fmt_uint(enc, v);
        }
        break;
    }
    case FCTYPE_UINT32_ARRAY: {
        nanocbor_fmt_array(enc, f->count);
        for (uint8_t i = 0; i < f->count; i++) {
            uint32_t v;
            memcpy(&v, (const uint8_t *)val + i * sizeof(uint32_t), sizeof v);
            nanocbor_fmt_uint(enc, v);
        }
        break;
    }
    default:
        return -1;
    }
    return 0;
}

/**
 * @brief Returns the field to encode at a cursor position, or NULL if the position encodes nothing (a key
 * no group defines, or a field equal to its group's defaults).
 *
 * @param groups - groups to encode from
 * @param groupCount - number of entries in groups
 * @param keys - requested keys, or NULL for group-table order
 * @param pos - cursor position (index into keys, or field position)
 * @param group - [out] set to the field's group when a field is returned
 *
 * @return the field descriptor to encode, or NULL
 */
static const flash_cfg_field_desc_t *item_at(const flash_cfg_cbor_group_t *groups, size_t groupCount,
                                             const uint16_t *keys, size_t pos,
                                             const flash_cfg_cbor_group_t **group)
{
    const flash_cfg_field_desc_t *f = keys ? find_key(groups, groupCount, keys[pos], group)
                                           : field_at(groups, groupCount, pos, group);
    if (f != NULL && (*group)->defaults != NULL &&
        memcmp((const uint8_t *)(*group)->data + f->offset,
               (const uint8_t *)(*group)->defaults + f->offset, field_total_size(f)) == 0)
        return NULL;
    return f;
}

/**
 * @brief Encodes one map entry (key, then value) for a field.
 *
 * @param enc - CBOR encoder to write into
 * @param group - group holding the field
 * @param f - field to encode
 */
static void encode_item(nanocbor_encoder_t *enc, const flash_cfg_cbor_group_t *group,
                        const flash_cfg_field_desc_t *f)
{
    nanocbor_fmt_uint(enc, (uint32_t)group->idBase + f->id);
    encode_field_value(enc, f, (const uint8_t *)group->data + f->offset);
}

/**
 * @brief Returns the encoded size of one map entry, without writing it anywhere.
 *
 * @param group - group holding the field
 * @param f - field to measure
 *
 * @return size in bytes of the entry's key and value
 */
static size_t item_size(const flash_cfg_cbor_group_t *group, const flash_cfg_field_desc_t *f)
{
    /* nanocbor counts the length of everything formatted, including what does not fit */
    uint8_t none;
    nanocbor_encoder_t enc;
    nanocbor_encoder_init(&enc, &none, 0);
    encode_item(&enc, group, f);
    return nanocbor_encoded_len(&enc);
}

/**
 * @brief Returns the size of a definite-length CBOR map header.
 *
 * @param entries - number of map entries
 *
 * @return header size in bytes
 */
static size_t map_header_size(size_t entries)
{
    if (entries < 24)       return 1;
    if (entries <= 0xFF)    return 2;
    if (entries <= 0xFFFF)  return 3;
    return 5;
}

/*===========================================================================
 * Decode helpers
 *===========================================================================*/

/**
 * @brief Returns whether iteration of a container stopped because the input ran out before the
 * container ended (nanocbor_at_end() reports both cases as the end).
 *
 * @param c - container iterator for which nanocbor_at_end() returned true
 *
 * @return true if the container is truncated
 */
static bool container_truncated(const nanocbor_value_t *c)
{
    if (nanocbor_container_indefinite(c))
        return c->cur >= c->end;            /* end marker not reached */
    return nanocbor_container_remaining(c) != 0;
}

/**
 * @brief Decodes a CBOR number (half, single or double float, or integer) as a double.
 *
 * @param it - CBOR decoder positioned at the value
 * @param value - [out] decoded value
 *
 * @return nanocbor result: >= 0 on success (decoder advanced), negative on error (decoder not advanced)
 */
static int decode_real(nanocbor_value_t *it, double *value)
{
    int rc = nanocbor_get_double(it, value);
    if (rc >= 0)
        return rc;

    int64_t i;
    rc = nanocbor_get_int64(it, &i);
    if (rc >= 0)
        *value = (double)i;
    return rc;
}

/**
 * @brief Decodes one scalar CBOR value of a field's element type.
 * @note Values are decoded into a local, naturally-aligned variable and then memcpy'd into dst, since dst may not meet the type's natural alignment (see encode_field_value).
 *
 * @param it - CBOR decoder positioned at the value
 * @param type - field type (an FCTYPE_* value; array types decode one element)
 * @param dst - where to store the element, or NULL to validate only
 *
 * @return nanocbor result: >= 0 on success, negative if the value has the wrong type or is out of range
 */
static int decode_element(nanocbor_value_t *it, uint8_t type, void *dst)
{
    int rc;

    switch (type) {
    case FCTYPE_UINT8:
    case FCTYPE_UINT8_ARRAY: {
        uint8_t v;
        rc = nanocbor_get_uint8(it, &v);
        if (rc >= 0 && dst) memcpy(dst, &v, sizeof v);
        return rc;
    }
    case FCTYPE_UINT16:
    case FCTYPE_UINT16_ARRAY: {
        uint16_t v;
        rc = nanocbor_get_uint16(it, &v);
        if (rc >= 0 && dst) memcpy(dst, &v, sizeof v);
        return rc;
    }
    case FCTYPE_UINT32:
    case FCTYPE_UINT32_ARRAY: {
        uint32_t v;
        rc = nanocbor_get_uint32(it, &v);
        if (rc >= 0 && dst) memcpy(dst, &v, sizeof v);
        return rc;
    }
    case FCTYPE_UINT64: {
        uint64_t v;
        rc = nanocbor_get_uint64(it, &v);
        if (rc >= 0 && dst) memcpy(dst, &v, sizeof v);
        return rc;
    }
    case FCTYPE_FLOAT:
    case FCTYPE_FLOAT_ARRAY: {
        /* Half/single values decode bit-exactly; a round trip through double would quiet a signaling NaN */
        float v;
        rc = nanocbor_get_float(it, &v);
        if (rc < 0) {
            double d;
            rc = decode_real(it, &d);
            if (rc < 0) return rc;
            v = (float)d;
            if (isinf(v) && !isinf(d))
                return NANOCBOR_ERR_OVERFLOW;   /* finite value outside float range */
        }
        if (dst) memcpy(dst, &v, sizeof v);
        return rc;
    }
    case FCTYPE_DOUBLE:
    case FCTYPE_DOUBLE_ARRAY: {
        double v;
        rc = decode_real(it, &v);
        if (rc >= 0 && dst) memcpy(dst, &v, sizeof v);
        return rc;
    }
    default:
        return NANOCBOR_ERR_INVALID_TYPE;
    }
}

/**
 * @brief Decodes a CBOR value into a config field. Array values may be shorter than the field (only the
 * leading elements are written) or longer (extra elements are skipped).
 *
 * @param it - CBOR decoder positioned at the value
 * @param f - descriptor of the field being decoded
 * @param dst - pointer to the field within the destination struct, or NULL to validate only
 *
 * @return >= 0 on success, negative if the value has the wrong type, is out of range, or is malformed
 */
static int decode_field_value(nanocbor_value_t *it, const flash_cfg_field_desc_t *f, void *dst)
{
    if (!field_is_array(f->type))
        return decode_element(it, f->type, dst);

    nanocbor_value_t arr;
    int rc = nanocbor_enter_array(it, &arr);
    if (rc < 0)
        return rc;

    size_t elemSize = field_element_size(f->type);
    for (size_t i = 0; !nanocbor_at_end(&arr); i++) {
        if (i < f->count)
            rc = decode_element(&arr, f->type, dst ? (uint8_t *)dst + i * elemSize : NULL);
        else
            rc = nanocbor_skip(&arr);
        if (rc < 0)
            return rc;
    }
    if (container_truncated(&arr))
        return NANOCBOR_ERR_END;
    return nanocbor_leave_container(it, &arr);
}

/*===========================================================================
 * Public API: group encode/decode
 *===========================================================================*/

size_t flash_cfg_cbor_field_count(const flash_cfg_cbor_group_t *groups, size_t groupCount)
{
    size_t total = 0;
    for (size_t g = 0; g < groupCount; g++)
        total += groups[g].count;
    return total;
}

int flash_cfg_cbor_encode_groups(const flash_cfg_cbor_group_t *groups, size_t groupCount,
                                 const uint16_t *keys, size_t keyCount,
                                 uint16_t *cursor, bool *complete,
                                 uint8_t *buf, size_t bufSize)
{
    if ((groups == NULL && groupCount) || cursor == NULL || buf == NULL)
        return FLASH_CFG_CBOR_ERR_ARG;

    size_t total = keys ? keyCount : flash_cfg_cbor_field_count(groups, groupCount);
    if (*cursor > total)
        return FLASH_CFG_CBOR_ERR_ARG;

    /* Find how many positions fit, measuring each entry before committing to it */
    const flash_cfg_cbor_group_t *g;
    size_t entries = 0;
    size_t entryBytes = 0;
    size_t end = *cursor;
    for (; end < total; end++) {
        const flash_cfg_field_desc_t *f = item_at(groups, groupCount, keys, end, &g);
        if (f == NULL)
            continue;
        size_t sz = item_size(g, f);
        if (map_header_size(entries + 1) + entryBytes + sz > bufSize)
            break;
        entries++;
        entryBytes += sz;
    }

    if (end < total && entries == 0)
        return FLASH_CFG_CBOR_ERR_TOO_LARGE;
    if (map_header_size(entries) + entryBytes > bufSize)
        return FLASH_CFG_CBOR_ERR_BUFFER;

    nanocbor_encoder_t enc;
    nanocbor_encoder_init(&enc, buf, bufSize);
    nanocbor_fmt_map(&enc, entries);
    for (size_t pos = *cursor; pos < end; pos++) {
        const flash_cfg_field_desc_t *f = item_at(groups, groupCount, keys, pos, &g);
        if (f != NULL)
            encode_item(&enc, g, f);
    }

    *cursor = (uint16_t)end;
    if (complete)
        *complete = (end == total);
    return (int)nanocbor_encoded_len(&enc);
}

int flash_cfg_cbor_decode_groups(const flash_cfg_cbor_group_t *groups, size_t groupCount,
                                 const uint8_t *buf, size_t bufSize,
                                 uint16_t *appliedKeys, size_t appliedMax, size_t *appliedCount)
{
    if ((groups == NULL && groupCount) || (buf == NULL && bufSize))
        return FLASH_CFG_CBOR_ERR_ARG;

    size_t applied = 0;

    /* Pass 0 validates the whole map; pass 1 applies it. Nothing is written unless pass 0 succeeds. */
    for (int pass = 0; pass < 2; pass++) {
        bool apply = (pass == 1);

        nanocbor_value_t dec;
        nanocbor_decoder_init(&dec, buf, bufSize);

        nanocbor_value_t map;
        if (nanocbor_enter_map(&dec, &map) < 0)
            return FLASH_CFG_CBOR_ERR_NOT_MAP;

        while (!nanocbor_at_end(&map)) {
            uint32_t key;
            const flash_cfg_cbor_group_t *g = NULL;
            const flash_cfg_field_desc_t *f = NULL;

            if (nanocbor_get_uint32(&map, &key) >= 0)
                f = find_key(groups, groupCount, key, &g);
            else if (nanocbor_skip(&map) < 0)       /* key of another type: ignore it */
                return FLASH_CFG_CBOR_ERR_MALFORMED;

            if (f == NULL) {
                if (nanocbor_skip(&map) < 0)
                    return FLASH_CFG_CBOR_ERR_MALFORMED;
                continue;
            }

            if (decode_field_value(&map, f, apply ? (uint8_t *)g->data + f->offset : NULL) < 0)
                return FLASH_CFG_CBOR_ERR_VALUE;

            if (apply && appliedKeys) {
                size_t i = 0;
                while (i < applied && appliedKeys[i] != key)
                    i++;
                if (i == applied && applied < appliedMax)
                    appliedKeys[applied++] = (uint16_t)key;
            }
        }

        if (container_truncated(&map) || nanocbor_leave_container(&dec, &map) < 0)
            return FLASH_CFG_CBOR_ERR_MALFORMED;
    }

    if (appliedCount)
        *appliedCount = applied;
    return FLASH_CFG_CBOR_OK;
}

int flash_cfg_cbor_encode_keys(const uint16_t *keys, size_t keyCount, uint8_t *buf, size_t bufSize)
{
    if ((keys == NULL && keyCount) || buf == NULL)
        return FLASH_CFG_CBOR_ERR_ARG;

    nanocbor_encoder_t enc;
    nanocbor_encoder_init(&enc, buf, bufSize);
    nanocbor_fmt_array(&enc, keyCount);
    for (size_t i = 0; i < keyCount; i++)
        nanocbor_fmt_uint(&enc, keys[i]);

    size_t len = nanocbor_encoded_len(&enc);
    if (len > bufSize)
        return FLASH_CFG_CBOR_ERR_BUFFER;
    return (int)len;
}

int flash_cfg_cbor_decode_keys(const uint8_t *buf, size_t bufSize, uint16_t *keys, size_t keysMax)
{
    if ((buf == NULL && bufSize) || (keys == NULL && keysMax))
        return FLASH_CFG_CBOR_ERR_ARG;

    nanocbor_value_t dec;
    nanocbor_decoder_init(&dec, buf, bufSize);

    nanocbor_value_t arr;
    if (nanocbor_enter_array(&dec, &arr) < 0)
        return FLASH_CFG_CBOR_ERR_NOT_ARRAY;

    size_t n = 0;
    while (!nanocbor_at_end(&arr)) {
        uint16_t key;
        if (nanocbor_get_uint16(&arr, &key) < 0)
            return FLASH_CFG_CBOR_ERR_VALUE;
        if (n >= keysMax)
            return FLASH_CFG_CBOR_ERR_TOO_MANY;
        keys[n++] = key;
    }

    if (container_truncated(&arr) || nanocbor_leave_container(&dec, &arr) < 0)
        return FLASH_CFG_CBOR_ERR_MALFORMED;
    return (int)n;
}

/*===========================================================================
 * Public API: flash storage encode/decode
 *===========================================================================*/

/**
 * @brief Delta-encodes one struct against its defaults as a complete (unpaged) CBOR map.
 *
 * @param fields - field table describing the struct
 * @param count - number of entries in fields
 * @param cfg - struct to encode
 * @param defaults - defaults; fields equal to these are omitted
 * @param buf - output buffer
 * @param buf_size - size of buf in bytes
 *
 * @return encoded size in bytes, or negative if the encoding does not fit in buf
 */
static int encode_delta(const flash_cfg_field_desc_t *fields, uint16_t count,
                        const void *cfg, const void *defaults, uint8_t *buf, size_t buf_size)
{
    flash_cfg_cbor_group_t group = { fields, count, 0, (void *)cfg, defaults };
    uint16_t cursor = 0;
    bool complete = false;
    int rc = flash_cfg_cbor_encode_groups(&group, 1, NULL, 0, &cursor, &complete, buf, buf_size);
    if (rc >= 0 && !complete)
        return FLASH_CFG_CBOR_ERR_BUFFER;
    return rc;
}

/**
 * @brief Decodes a delta map into one struct, starting from defaults.
 *
 * @param fields - field table describing the struct
 * @param count - number of entries in fields
 * @param cfg - [out] struct to populate; equal to defaults if decoding fails
 * @param defaults - defaults the delta is applied to
 * @param cfg_size - size of the struct in bytes
 * @param buf - CBOR map to decode
 * @param buf_size - size of buf in bytes
 *
 * @return FLASH_CFG_CBOR_OK on success, or a negative eFlashCfgCborResult
 */
static int decode_delta(const flash_cfg_field_desc_t *fields, uint16_t count,
                        void *cfg, const void *defaults, size_t cfg_size,
                        const uint8_t *buf, size_t buf_size)
{
    memcpy(cfg, defaults, cfg_size);
    flash_cfg_cbor_group_t group = { fields, count, 0, cfg, NULL };
    return flash_cfg_cbor_decode_groups(&group, 1, buf, buf_size, NULL, 0, NULL);
}

int flash_cfg_cbor_encode_imx(const nvm_flash_cfg_t *cfg,
                               const nvm_flash_cfg_t *defaults,
                               uint8_t *buf, size_t buf_size)
{
    return encode_delta(s_imx_fields, TABLE_COUNT(s_imx_fields), cfg, defaults, buf, buf_size);
}

int flash_cfg_cbor_decode_imx(nvm_flash_cfg_t *cfg,
                               const nvm_flash_cfg_t *defaults,
                               const uint8_t *buf, size_t buf_size)
{
    return decode_delta(s_imx_fields, TABLE_COUNT(s_imx_fields), cfg, defaults, sizeof(nvm_flash_cfg_t),
                        buf, buf_size);
}

int flash_cfg_cbor_encode_gpx(const gpx_flash_cfg_t *cfg,
                               const gpx_flash_cfg_t *defaults,
                               uint8_t *buf, size_t buf_size)
{
    return encode_delta(s_gpx_fields, TABLE_COUNT(s_gpx_fields), cfg, defaults, buf, buf_size);
}

int flash_cfg_cbor_decode_gpx(gpx_flash_cfg_t *cfg,
                               const gpx_flash_cfg_t *defaults,
                               const uint8_t *buf, size_t buf_size)
{
    return decode_delta(s_gpx_fields, TABLE_COUNT(s_gpx_fields), cfg, defaults, sizeof(gpx_flash_cfg_t),
                        buf, buf_size);
}

/*===========================================================================
 * Block-level read/write
 *===========================================================================*/

/* Use flashChecksum32 from data_sets.c */
extern uint32_t flashChecksum32(const void* data, int size);

/**
 * @brief Writes a flash_cfg_hdr_t header followed by the given CBOR payload into a flash block buffer, filling in size, the CBOR flag, version, and a checksum covering the whole block.
 *
 * @param block - output buffer to receive the header + payload
 * @param block_size - size of block in bytes
 * @param payload - CBOR-encoded payload bytes to embed after the header
 * @param payload_size - size of payload in bytes
 * @param major_ver - major version to record in the header
 * @param minor_ver - minor version to record in the header
 *
 * @return total number of bytes written (header + payload), or -1 if the header plus payload would not fit in block_size
 */
int flash_cfg_cbor_write_block(uint8_t *block, size_t block_size,
                                const uint8_t *payload, size_t payload_size,
                                uint8_t major_ver, uint8_t minor_ver)
{
    size_t total = sizeof(flash_cfg_hdr_t) + payload_size;
    if (total > block_size)
        return -1;

    flash_cfg_hdr_t *hdr = (flash_cfg_hdr_t *)block;
    hdr->size = (uint32_t)total;
    hdr->flags = FLASH_CFG_CBOR_FLAG;
    hdr->version[0] = major_ver;
    hdr->version[1] = minor_ver;

    /* Copy payload after header */
    memcpy(block + sizeof(flash_cfg_hdr_t), payload, payload_size);

    /* Checksum covers bytes 8..total (same convention as flashChecksum32: skip size+checksum) */
    hdr->checksum = flashChecksum32(hdr, (int)total);

    return (int)total;
}

/**
 * @brief Validates a flash block's header (CBOR flag, size bounds, checksum) and, on success, returns a pointer to and size of the CBOR payload embedded after the header.
 *
 * @param block - flash block buffer containing a flash_cfg_hdr_t followed by a CBOR payload
 * @param block_size - size of block in bytes
 * @param payload_out - [out] set to point at the payload bytes within block on success
 * @param payload_size_out - [out] set to the payload size in bytes on success
 *
 * @return 0 on success, -1 if block_size is smaller than the header, -2 if the CBOR flag is not set, -3 if the header's recorded size is out of bounds, -4 if the checksum does not match
 */
int flash_cfg_cbor_read_block(const uint8_t *block, size_t block_size,
                               const uint8_t **payload_out,
                               size_t *payload_size_out)
{
    if (block_size < sizeof(flash_cfg_hdr_t))
        return -1;

    const flash_cfg_hdr_t *hdr = (const flash_cfg_hdr_t *)block;

    /* Check CBOR flag */
    if (!flash_cfg_is_cbor(hdr))
        return -2;

    /* Sanity check size */
    if (hdr->size < sizeof(flash_cfg_hdr_t) || hdr->size > block_size)
        return -3;

    /* Verify checksum */
    uint32_t calc = flashChecksum32(hdr, (int)hdr->size);
    if (calc != hdr->checksum)
        return -4;

    *payload_out = block + sizeof(flash_cfg_hdr_t);
    *payload_size_out = hdr->size - sizeof(flash_cfg_hdr_t);
    return 0;
}
