/**
 * @file flash_cfg_cbor.h
 * @brief CBOR encoding/decoding of device configuration (IMX, GPX, CAN and RMC fields)
 *
 * Configuration fields are described by tables of flash_cfg_field_desc_t, each entry giving the field's
 * stable integer CBOR map key, its type and its offset within a config struct. A flash_cfg_cbor_group_t
 * binds one table to one struct instance; a set of groups describes everything a device can encode or
 * accept.
 *
 * Two uses share this codec:
 *  - Flash storage: a delta encoding (only fields that differ from factory defaults) wrapped in a
 *    flash_cfg_hdr_t block.
 *  - Messaging: a full or key-selected encoding, paged to fit a packet, and an atomic decode that
 *    applies a received map on top of the current configuration.
 *
 * Field keys are stable: never reorder or reuse them. Keys are scoped to a device type (IMX and GPX keys
 * overlap), so a key is only meaningful together with the device it was read from or sent to.
 *
 * @copyright Copyright (c) 2026 Inertial Sense, Inc.
 */

#ifndef FLASH_CFG_CBOR_H
#define FLASH_CFG_CBOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "data_sets.h"

/*===========================================================================
 * Common CBOR header — overlays the first 12 bytes of config structs
 *===========================================================================*/

/** CBOR format flag in the flags field */
#define FLASH_CFG_CBOR_FLAG     0x8000

typedef struct PACKED {
    uint32_t size;          /**< Total size: header + CBOR payload */
    uint32_t checksum;      /**< flashChecksum32() of bytes 8..size */
    union {
        struct {
            uint16_t flags;         /**< bit 15 set = CBOR format */
            uint8_t  version[2];    /**< [major, minor] schema version */
        };
        uint32_t key;               /**< Legacy overlap — existing keys (36/IMX, 2/GPX) have bit 15 clear */
    };
} flash_cfg_hdr_t;

/** Check if a flash block header indicates CBOR format */
static inline bool flash_cfg_is_cbor(const flash_cfg_hdr_t *hdr)
{
    return (hdr->flags & FLASH_CFG_CBOR_FLAG) != 0;
}

/*===========================================================================
 * IMX flash config field IDs (CBOR integer map keys)
 *
 * One entry per nvm_flash_cfg_t user field (excluding size/checksum/key).
 * wheel_config_t is flattened into individual field IDs.
 * Values are stable — never reorder or reuse IDs.
 *===========================================================================*/

typedef enum {
    IMX_CFG_FIELD_STARTUP_IMU_DT        = 0,
    IMX_CFG_FIELD_STARTUP_NAV_DT        = 1,
    IMX_CFG_FIELD_SER0_BAUD             = 2,
    IMX_CFG_FIELD_SER1_BAUD             = 3,
    IMX_CFG_FIELD_INS_ROTATION          = 4,    /* float[3] */
    IMX_CFG_FIELD_INS_OFFSET            = 5,    /* float[3] */
    IMX_CFG_FIELD_GNSS1_ANT_OFFSET      = 6,    /* float[3] */
    IMX_CFG_FIELD_DYNAMIC_MODEL         = 7,
    IMX_CFG_FIELD_DEBUG                 = 8,
    IMX_CFG_FIELD_GNSS_SAT_SIG_CONST   = 9,
    IMX_CFG_FIELD_SYS_CFG_BITS         = 10,
    IMX_CFG_FIELD_REF_LLA              = 11,    /* double[3] */
    IMX_CFG_FIELD_LAST_LLA             = 12,    /* double[3] */
    IMX_CFG_FIELD_LAST_LLA_TOW_MS      = 13,
    IMX_CFG_FIELD_LAST_LLA_WEEK        = 14,
    IMX_CFG_FIELD_LAST_LLA_UPDATE_DIST = 15,
    IMX_CFG_FIELD_IO_CONFIG            = 16,
    IMX_CFG_FIELD_PLATFORM_CONFIG      = 17,
    IMX_CFG_FIELD_GNSS2_ANT_OFFSET     = 18,    /* float[3] */
    IMX_CFG_FIELD_ZERO_VEL_ROTATION    = 19,    /* float[3] */
    IMX_CFG_FIELD_ZERO_VEL_OFFSET      = 20,    /* float[3] */
    IMX_CFG_FIELD_GPS_TIME_USER_DELAY  = 21,
    IMX_CFG_FIELD_MAG_DECLINATION      = 22,
    IMX_CFG_FIELD_GPS_TIME_SYNC_PERIOD = 23,
    IMX_CFG_FIELD_STARTUP_GNSS_DT      = 24,
    IMX_CFG_FIELD_RTK_CFG_BITS         = 25,
    IMX_CFG_FIELD_SENSOR_CONFIG        = 26,
    IMX_CFG_FIELD_GNSS_MIN_ELEVATION   = 27,
    IMX_CFG_FIELD_SER2_BAUD            = 28,
    /* wheel_config_t — flattened */
    IMX_CFG_FIELD_WHEEL_CFG_BITS       = 29,
    IMX_CFG_FIELD_WHEEL_E_B2W          = 30,    /* float[3] */
    IMX_CFG_FIELD_WHEEL_E_B2W_SIGMA    = 31,    /* float[3] */
    IMX_CFG_FIELD_WHEEL_T_B2W          = 32,    /* float[3] */
    IMX_CFG_FIELD_WHEEL_T_B2W_SIGMA    = 33,    /* float[3] */
    IMX_CFG_FIELD_WHEEL_TRACK_WIDTH    = 34,
    IMX_CFG_FIELD_WHEEL_RADIUS         = 35,
    /* remaining scalar fields */
    IMX_CFG_FIELD_MAG_INTERFERENCE_THRESH   = 36,
    IMX_CFG_FIELD_MAG_CAL_QUAL_THRESH       = 37,
    IMX_CFG_FIELD_GNSS_CN0_MINIMUM          = 38,
    IMX_CFG_FIELD_GNSS_CN0_DYN_MIN_OFFSET   = 39,
    IMX_CFG_FIELD_IMU_REJECT_GYRO_LOW       = 40,
    IMX_CFG_FIELD_IMU_REJECT_GYRO_HIGH      = 41,
    IMX_CFG_FIELD_IMU_SHOCK_DETECT_LAT      = 42,
    IMX_CFG_FIELD_IMU_SHOCK_REJECT_LATCH    = 43,
    IMX_CFG_FIELD_IMU_SHOCK_OPTIONS         = 44,
    IMX_CFG_FIELD_IMU_SHOCK_DELTA_ACC_HI    = 45,
    IMX_CFG_FIELD_IMU_SHOCK_DELTA_ACC_LO    = 46,
    IMX_CFG_FIELD_IMU_SHOCK_DELTA_GYRO_HI   = 47,
    IMX_CFG_FIELD_IMU_SHOCK_DELTA_GYRO_LO   = 48,
    IMX_CFG_FIELD_IO_CONFIG_2               = 49,

    IMX_CFG_FIELD_COUNT
} eImxFlashCfgFieldId;

/*===========================================================================
 * GPX flash config field IDs (CBOR integer map keys)
 *
 * One entry per gpx_flash_cfg_t user field (excluding size/checksum/key/reserved).
 *===========================================================================*/

typedef enum {
    GPX_CFG_FIELD_SER0_BAUD            = 0,
    GPX_CFG_FIELD_SER1_BAUD            = 1,
    GPX_CFG_FIELD_SER2_BAUD            = 2,
    GPX_CFG_FIELD_STARTUP_GNSS_DT      = 3,
    GPX_CFG_FIELD_GNSS1_ANT_OFFSET     = 4,    /* float[3] */
    GPX_CFG_FIELD_GNSS2_ANT_OFFSET     = 5,    /* float[3] */
    GPX_CFG_FIELD_GNSS_SAT_SIG_CONST   = 6,
    GPX_CFG_FIELD_DYNAMIC_MODEL        = 7,
    GPX_CFG_FIELD_DEBUG                = 8,
    GPX_CFG_FIELD_GPS_TIME_SYNC_PERIOD = 9,
    GPX_CFG_FIELD_GPS_TIME_USER_DELAY  = 10,
    GPX_CFG_FIELD_GNSS_MIN_ELEVATION   = 11,
    GPX_CFG_FIELD_RTK_CFG_BITS         = 12,
    GPX_CFG_FIELD_GNSS_CN0_MINIMUM     = 13,
    GPX_CFG_FIELD_GNSS_CN0_DYN_MIN_OFFSET = 14,
    GPX_CFG_FIELD_SYS_CFG_BITS         = 15,
    GPX_CFG_FIELD_REF_LLA              = 16,    /* double[3] */

    GPX_CFG_FIELD_COUNT
} eGpxFlashCfgFieldId;

/*===========================================================================
 * CAN config CBOR field IDs (encoded in the main IMX CBOR block alongside
 * nvm_flash_cfg_t fields; IDs start at 50 to avoid collision with
 * eImxFlashCfgFieldId values 0–49).
 *
 * Maps directly to can_config_t fields.
 * Values are stable — never reorder or reuse IDs.
 *===========================================================================*/

typedef enum {
    CAN_CFG_FIELD_PERIOD_MULT    = 50,  /**< uint16[NUM_CIDS] — can_period_mult */
    CAN_CFG_FIELD_TRANSMIT_ADDR  = 51,  /**< uint32[NUM_CIDS] — can_transmit_address */
    CAN_CFG_FIELD_SETTING        = 52,  /**< uint16 — baud rate kbps + FD enable bit */
    CAN_CFG_FIELD_RECEIVE_ADDR   = 53,  /**< uint32 — can_receive_address */
} eCanFlashCfgFieldId;

/*===========================================================================
 * RMC config CBOR field IDs (encoded in the main IMX CBOR block; IDs start
 * at 54 to avoid collision with IMX IDs 0–49 and CAN IDs 50–53).
 *
 * Five fields per port × up to 6 ports (IDs 54–83). Port n's fields are
 * RMC_CFG_PORT0_BITS + n * RMC_CFG_FIELDS_PER_PORT + eRmcPortCfgFieldId.
 * Values are stable — never reorder or reuse IDs.
 *===========================================================================*/

/** Offset of each per-port RMC field from its port's first key */
typedef enum {
    RMC_CFG_FIELD_BITS          = 0,    /**< uint64  — rmc.bits */
    RMC_CFG_FIELD_OPTIONS       = 1,    /**< uint32  — rmc.options */
    RMC_CFG_FIELD_PERIOD_MULT   = 2,    /**< uint8[DID_COUNT] — periodMultiple */
    RMC_CFG_FIELD_NMEA_BITS     = 3,    /**< uint32  — rmcNmea.nmeaBits */
    RMC_CFG_FIELD_NMEA_PERIOD   = 4,    /**< uint8[NMEA_MSG_ID_COUNT] — rmcNmea.nmeaPeriod */

    RMC_CFG_FIELDS_PER_PORT
} eRmcPortCfgFieldId;

/** Maximum number of ports with an RMC key range */
#define RMC_CFG_MAX_PORTS           6

typedef enum {
    /* Port 0 */
    RMC_CFG_PORT0_BITS         = 54,  /**< uint64  — rmc.bits */
    RMC_CFG_PORT0_OPTIONS      = 55,  /**< uint32  — rmc.options */
    RMC_CFG_PORT0_PERIOD_MULT  = 56,  /**< uint8[DID_COUNT] — periodMultiple */
    RMC_CFG_PORT0_NMEA_BITS    = 57,  /**< uint32  — rmcNmea.nmeaBits */
    RMC_CFG_PORT0_NMEA_PERIOD  = 58,  /**< uint8[NMEA_MSG_ID_COUNT] — rmcNmea.nmeaPeriod */
    /* Port 1 */
    RMC_CFG_PORT1_BITS         = 59,
    RMC_CFG_PORT1_OPTIONS      = 60,
    RMC_CFG_PORT1_PERIOD_MULT  = 61,
    RMC_CFG_PORT1_NMEA_BITS    = 62,
    RMC_CFG_PORT1_NMEA_PERIOD  = 63,
    /* Port 2 */
    RMC_CFG_PORT2_BITS         = 64,
    RMC_CFG_PORT2_OPTIONS      = 65,
    RMC_CFG_PORT2_PERIOD_MULT  = 66,
    RMC_CFG_PORT2_NMEA_BITS    = 67,
    RMC_CFG_PORT2_NMEA_PERIOD  = 68,
    /* Port 3 */
    RMC_CFG_PORT3_BITS         = 69,
    RMC_CFG_PORT3_OPTIONS      = 70,
    RMC_CFG_PORT3_PERIOD_MULT  = 71,
    RMC_CFG_PORT3_NMEA_BITS    = 72,
    RMC_CFG_PORT3_NMEA_PERIOD  = 73,
    /* Port 4 (only on targets with NUM_COM_PORTS >= 5) */
    RMC_CFG_PORT4_BITS         = 74,
    RMC_CFG_PORT4_OPTIONS      = 75,
    RMC_CFG_PORT4_PERIOD_MULT  = 76,
    RMC_CFG_PORT4_NMEA_BITS    = 77,
    RMC_CFG_PORT4_NMEA_PERIOD  = 78,
    /* Port 5 (only on targets with NUM_COM_PORTS >= 6) */
    RMC_CFG_PORT5_BITS         = 79,
    RMC_CFG_PORT5_OPTIONS      = 80,
    RMC_CFG_PORT5_PERIOD_MULT  = 81,
    RMC_CFG_PORT5_NMEA_BITS    = 82,
    RMC_CFG_PORT5_NMEA_PERIOD  = 83,
} eRmcFlashCfgFieldId;

/*===========================================================================
 * Field descriptor table types
 *===========================================================================*/

typedef enum {
    FCTYPE_UINT8 = 0,
    FCTYPE_UINT16,
    FCTYPE_UINT32,
    FCTYPE_UINT64,
    FCTYPE_FLOAT,
    FCTYPE_DOUBLE,
    FCTYPE_FLOAT_ARRAY,     /**< count > 1 */
    FCTYPE_DOUBLE_ARRAY,    /**< count > 1 */
    FCTYPE_UINT8_ARRAY,     /**< count > 1 */
    FCTYPE_UINT16_ARRAY,    /**< count > 1 */
    FCTYPE_UINT32_ARRAY,    /**< count > 1 */
} flash_cfg_field_type_t;

typedef struct {
    uint16_t                id;     /**< Field ID enum value (CBOR map key, relative to the group's idBase) */
    uint8_t                 type;   /**< flash_cfg_field_type_t */
    uint8_t                 count;  /**< 1 for scalars, N for arrays */
    uint16_t                offset; /**< offsetof() into config struct */
} flash_cfg_field_desc_t;

/**
 * A field table bound to the struct instance it describes. A field's CBOR key is idBase + desc.id.
 * For decoding, data must be writable; encoding only reads it.
 */
typedef struct {
    const flash_cfg_field_desc_t   *fields;     //!< Field descriptor table
    uint16_t                        count;      //!< Number of entries in fields
    uint16_t                        idBase;     //!< Added to each descriptor id to form its CBOR key
    void                           *data;       //!< Struct instance the descriptor offsets address
    const void                     *defaults;   //!< When non-NULL, encoding omits fields equal to these defaults
} flash_cfg_cbor_group_t;

/*===========================================================================
 * Field tables
 *===========================================================================*/

/**
 * Returns the field table for nvm_flash_cfg_t (IMX keys 0–49).
 *
 * @param count  [out] number of entries in the returned table
 * @return pointer to the static field table
 */
const flash_cfg_field_desc_t *flash_cfg_cbor_imx_fields(uint16_t *count);

/**
 * Returns the field table for gpx_flash_cfg_t (GPX keys 0–16).
 *
 * @param count  [out] number of entries in the returned table
 * @return pointer to the static field table
 */
const flash_cfg_field_desc_t *flash_cfg_cbor_gpx_fields(uint16_t *count);

/**
 * Returns the field table for can_config_t (IMX keys 50–53).
 *
 * @param count  [out] number of entries in the returned table
 * @return pointer to the static field table
 */
const flash_cfg_field_desc_t *flash_cfg_cbor_can_fields(uint16_t *count);

/**
 * Returns the per-port field table for rmci_t. Ids are eRmcPortCfgFieldId values; bind it with
 * flash_cfg_cbor_rmc_port_id_base() to place a port in the IMX key space (keys 54–83).
 *
 * @param count  [out] number of entries in the returned table
 * @return pointer to the static field table
 */
const flash_cfg_field_desc_t *flash_cfg_cbor_rmc_port_fields(uint16_t *count);

/**
 * Returns the first CBOR key of a port's RMC fields.
 *
 * @param port  port index, 0 to RMC_CFG_MAX_PORTS-1
 * @return RMC_CFG_PORT0_BITS + port * RMC_CFG_FIELDS_PER_PORT
 */
static inline uint16_t flash_cfg_cbor_rmc_port_id_base(unsigned port)
{
    return (uint16_t)(RMC_CFG_PORT0_BITS + port * RMC_CFG_FIELDS_PER_PORT);
}

/*===========================================================================
 * Group encode / decode API
 *===========================================================================*/

/** Return codes (negative values are errors) */
typedef enum {
    FLASH_CFG_CBOR_OK                =  0,
    FLASH_CFG_CBOR_ERR_NOT_MAP       = -1,  //!< Input is not a CBOR map
    FLASH_CFG_CBOR_ERR_NOT_ARRAY     = -2,  //!< Input is not a CBOR array
    FLASH_CFG_CBOR_ERR_VALUE         = -3,  //!< A known field's value (or a requested key) has the wrong type or is out of range
    FLASH_CFG_CBOR_ERR_MALFORMED     = -4,  //!< Input is not well-formed CBOR
    FLASH_CFG_CBOR_ERR_BUFFER        = -5,  //!< Output buffer too small for the complete encoding
    FLASH_CFG_CBOR_ERR_TOO_LARGE     = -6,  //!< A single field does not fit in the output buffer on its own
    FLASH_CFG_CBOR_ERR_TOO_MANY      = -7,  //!< More keys than the caller's key buffer holds
    FLASH_CFG_CBOR_ERR_ARG           = -8,  //!< Invalid argument (NULL pointer, cursor past the end)
} eFlashCfgCborResult;

/**
 * Returns the total number of fields across a set of groups (the cursor range for a full encode).
 *
 * @param groups      groups to count
 * @param groupCount  number of entries in groups
 * @return sum of each group's field count
 */
size_t flash_cfg_cbor_field_count(const flash_cfg_cbor_group_t *groups, size_t groupCount);

/**
 * Encodes fields from a set of groups as one CBOR map, in pages that fit buf.
 *
 * Without keys, fields are encoded in group-table order; with keys, in the order given, skipping keys no
 * group defines. Encoding starts at position *cursor (a field index, or an index into keys) and stops
 * before the first field that would overflow buf. Groups with non-NULL defaults omit fields equal to
 * them.
 *
 * @param groups      groups to encode from
 * @param groupCount  number of entries in groups
 * @param keys        keys to encode, or NULL to encode every field
 * @param keyCount    number of entries in keys (ignored when keys is NULL)
 * @param cursor      [in/out] position to start at; set to the first position not encoded
 * @param complete    [out, optional] set true when every position from *cursor to the end was handled
 * @param buf         output buffer for the CBOR map
 * @param bufSize     size of buf in bytes
 * @return encoded size in bytes on success; FLASH_CFG_CBOR_ERR_TOO_LARGE if the field at *cursor does not
 *         fit even alone; FLASH_CFG_CBOR_ERR_ARG on invalid arguments
 */
int flash_cfg_cbor_encode_groups(const flash_cfg_cbor_group_t *groups, size_t groupCount,
                                 const uint16_t *keys, size_t keyCount,
                                 uint16_t *cursor, bool *complete,
                                 uint8_t *buf, size_t bufSize);

/**
 * Decodes a CBOR map and applies its known fields to the groups' data, atomically.
 *
 * The whole map is validated before anything is written: if any known field has the wrong type or an
 * out-of-range value, or the map is malformed, no group is modified. Unknown keys, including keys that are
 * not unsigned integers, are ignored. Array values
 * shorter than the field update only their leading elements; extra elements are ignored. Fields absent
 * from the map are left unchanged.
 *
 * @param groups        groups to apply to
 * @param groupCount    number of entries in groups
 * @param buf           CBOR map to decode
 * @param bufSize       size of buf in bytes
 * @param appliedKeys   [out, optional] receives each distinct key that was applied, in map order
 * @param appliedMax    capacity of appliedKeys; further applied keys are applied but not reported
 * @param appliedCount  [out, optional] number of keys written to appliedKeys
 * @return FLASH_CFG_CBOR_OK on success, or a negative eFlashCfgCborResult
 */
int flash_cfg_cbor_decode_groups(const flash_cfg_cbor_group_t *groups, size_t groupCount,
                                 const uint8_t *buf, size_t bufSize,
                                 uint16_t *appliedKeys, size_t appliedMax, size_t *appliedCount);

/**
 * Encodes a list of keys as a CBOR array (the body of a key-selected request).
 *
 * @param keys      keys to encode
 * @param keyCount  number of entries in keys
 * @param buf       output buffer
 * @param bufSize   size of buf in bytes
 * @return encoded size in bytes, or FLASH_CFG_CBOR_ERR_BUFFER if it does not fit
 */
int flash_cfg_cbor_encode_keys(const uint16_t *keys, size_t keyCount, uint8_t *buf, size_t bufSize);

/**
 * Decodes a CBOR array of keys.
 *
 * @param buf      CBOR array to decode
 * @param bufSize  size of buf in bytes
 * @param keys     [out] receives the keys
 * @param keysMax  capacity of keys
 * @return number of keys decoded, FLASH_CFG_CBOR_ERR_TOO_MANY if the array holds more than keysMax, or
 *         another negative eFlashCfgCborResult
 */
int flash_cfg_cbor_decode_keys(const uint8_t *buf, size_t bufSize, uint16_t *keys, size_t keysMax);

/*===========================================================================
 * Flash storage encode/decode
 *===========================================================================*/

/**
 * Delta-encode IMX flash config as CBOR map.
 * Only fields differing from defaults are included.
 *
 * @param cfg       Current config
 * @param defaults  Factory defaults
 * @param buf       Output buffer for CBOR payload (no header)
 * @param buf_size  Size of output buffer
 * @return Encoded CBOR payload size on success, negative on error
 */
int flash_cfg_cbor_encode_imx(const nvm_flash_cfg_t *cfg,
                               const nvm_flash_cfg_t *defaults,
                               uint8_t *buf, size_t buf_size);

/**
 * Delta-encode GPX flash config as CBOR map.
 * Only fields differing from defaults are included.
 *
 * @param cfg       Current config
 * @param defaults  Factory defaults
 * @param buf       Output buffer for CBOR payload (no header)
 * @param buf_size  Size of output buffer
 * @return Encoded CBOR payload size on success, negative on error
 */
int flash_cfg_cbor_encode_gpx(const gpx_flash_cfg_t *cfg,
                               const gpx_flash_cfg_t *defaults,
                               uint8_t *buf, size_t buf_size);

/**
 * Delta-decode IMX flash config from CBOR map.
 * Starts from defaults, applies CBOR delta overrides.
 * Unknown field IDs are silently skipped (forward compatibility).
 *
 * @param cfg       Output config struct (populated)
 * @param defaults  Factory defaults
 * @param buf       CBOR payload (no header)
 * @param buf_size  Size of CBOR payload
 * @return 0 on success, negative on error (cfg is then left equal to defaults)
 */
int flash_cfg_cbor_decode_imx(nvm_flash_cfg_t *cfg,
                               const nvm_flash_cfg_t *defaults,
                               const uint8_t *buf, size_t buf_size);

/**
 * Delta-decode GPX flash config from CBOR map.
 * Starts from defaults, applies CBOR delta overrides.
 * Unknown field IDs are silently skipped (forward compatibility).
 *
 * @param cfg       Output config struct (populated)
 * @param defaults  Factory defaults
 * @param buf       CBOR payload (no header)
 * @param buf_size  Size of CBOR payload
 * @return 0 on success, negative on error (cfg is then left equal to defaults)
 */
int flash_cfg_cbor_decode_gpx(gpx_flash_cfg_t *cfg,
                               const gpx_flash_cfg_t *defaults,
                               const uint8_t *buf, size_t buf_size);

/**
 * Write a CBOR config block: prepend flash_cfg_hdr_t to payload.
 *
 * @param block       Output buffer (must be large enough for header + payload)
 * @param block_size  Size of output buffer
 * @param payload     CBOR payload from encode function
 * @param payload_size Size of CBOR payload
 * @param major_ver   CBOR schema major version
 * @param minor_ver   CBOR schema minor version
 * @return Total block size (header + payload) on success, negative on error
 */
int flash_cfg_cbor_write_block(uint8_t *block, size_t block_size,
                                const uint8_t *payload, size_t payload_size,
                                uint8_t major_ver, uint8_t minor_ver);

/**
 * Read and validate a CBOR config block.
 *
 * @param block         Input block data
 * @param block_size    Size of input block
 * @param payload_out   Receives pointer to CBOR payload within block
 * @param payload_size_out Receives size of CBOR payload
 * @return 0 on success, negative on error (not CBOR, bad checksum, etc.)
 */
int flash_cfg_cbor_read_block(const uint8_t *block, size_t block_size,
                               const uint8_t **payload_out,
                               size_t *payload_size_out);

#ifdef __cplusplus
}
#endif

#endif /* FLASH_CFG_CBOR_H */
