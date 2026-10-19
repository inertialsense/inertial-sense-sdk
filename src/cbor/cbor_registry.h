/**
 * @file cbor_registry.h
 * @brief CBOR key registry: DID keys, message keys and enums shared by every DID. Each DID's field keys, schema and
 *        defaults are in its own header under dids/, included at the end.
 *
 * Generated from the CBOR key registry by is-common scripts/cbor_registry.py -- do not edit.
 *
 * @copyright Copyright (c) 2026 Inertial Sense, Inc.
 */

#ifndef CBOR_REGISTRY_H
#define CBOR_REGISTRY_H

#include <stdint.h>
#include "data_sets.h"
#include "cbor/cbor_codec.h"

#ifdef __cplusplus
extern "C" {
#endif

#define CBOR_REGISTRY_VERSION                        1        //!< registry format version
#define CBOR_ISB_DID                                 129      //!< the ISB DID that carries every CBOR message (DID_CBOR)

#define RMC_CFG_MAX_PORTS                            6        //!< protocol limit defined by the registry

/** @name Root DID keys */
/** @{ */
#define CBOR_DID_FLASH_CFG                           0x000C   //!< FLASH_CFG: legacy DID_FLASH_CONFIG (nvm_flash_cfg_t), served by IMX-5, IMX-6
#define CBOR_DID_CAN_CONFIG                          0x005A   //!< CAN_CONFIG: legacy DID_CAN_CONFIG (can_config_t), served by IMX-5, IMX-6
#define CBOR_DID_GPX_FLASH_CFG                       0x0079   //!< GPX_FLASH_CFG: legacy DID_GPX_FLASH_CFG (gpx_flash_cfg_t), served by GPX-1
#define CBOR_DID_CALIBRATION                         0x0100   //!< CALIBRATION: CBOR-only DID, reserved
#define CBOR_DID_NMEA_PIMU                           0x3F01   //!< NMEA_PIMU: NMEA sentence pseudo-DID
#define CBOR_DID_NMEA_PPIMU                          0x3F02   //!< NMEA_PPIMU: NMEA sentence pseudo-DID
#define CBOR_DID_NMEA_PRIMU                          0x3F03   //!< NMEA_PRIMU: NMEA sentence pseudo-DID
#define CBOR_DID_NMEA_PINS1                          0x3F04   //!< NMEA_PINS1: NMEA sentence pseudo-DID
#define CBOR_DID_NMEA_PINS2                          0x3F05   //!< NMEA_PINS2: NMEA sentence pseudo-DID
#define CBOR_DID_NMEA_PGPSP                          0x3F06   //!< NMEA_PGPSP: NMEA sentence pseudo-DID
#define CBOR_DID_NMEA_GNGGA                          0x3F07   //!< NMEA_GNGGA: NMEA sentence pseudo-DID
#define CBOR_DID_NMEA_GNGLL                          0x3F08   //!< NMEA_GNGLL: NMEA sentence pseudo-DID
#define CBOR_DID_NMEA_GNGSA                          0x3F09   //!< NMEA_GNGSA: NMEA sentence pseudo-DID
#define CBOR_DID_NMEA_GNRMC                          0x3F0A   //!< NMEA_GNRMC: NMEA sentence pseudo-DID
#define CBOR_DID_NMEA_GNZDA                          0x3F0B   //!< NMEA_GNZDA: NMEA sentence pseudo-DID
#define CBOR_DID_NMEA_PASHR                          0x3F0C   //!< NMEA_PASHR: NMEA sentence pseudo-DID
#define CBOR_DID_NMEA_PSTRB                          0x3F0D   //!< NMEA_PSTRB: NMEA sentence pseudo-DID
#define CBOR_DID_NMEA_INFO                           0x3F0E   //!< NMEA_INFO: NMEA sentence pseudo-DID
#define CBOR_DID_NMEA_GNGSV                          0x3F0F   //!< NMEA_GNGSV: NMEA sentence pseudo-DID
#define CBOR_DID_NMEA_GNVTG                          0x3F10   //!< NMEA_GNVTG: NMEA sentence pseudo-DID
#define CBOR_DID_NMEA_INTEL                          0x3F11   //!< NMEA_INTEL: NMEA sentence pseudo-DID
#define CBOR_DID_NMEA_POWGPS                         0x3F12   //!< NMEA_POWGPS: NMEA sentence pseudo-DID
#define CBOR_DID_NMEA_POWTLV                         0x3F13   //!< NMEA_POWTLV: NMEA sentence pseudo-DID
#define CBOR_DID_CAPABILITIES                        0x7FFC   //!< CAPABILITIES: CBOR-only DID, served by IMX-5, IMX-6, GPX-1
#define CBOR_DID_RMC_STORED                          0x7FFD   //!< RMC_STORED: CBOR-only DID, served by IMX-5, IMX-6, GPX-1
#define CBOR_DID_RMC                                 0x7FFE   //!< RMC: CBOR-only DID, served by IMX-5, IMX-6, GPX-1
#define CBOR_DID_ROUTE_FILTER                        0x7FFF   //!< ROUTE_FILTER: CBOR-only DID, served by IMX-5, IMX-6
/** @} */

/** @name General message keys (MSG_*) */
/** @{ */
#define CBOR_MSG_TYPE                                0xC000   //!< uint8 (msg_type), default set by the protocol
#define CBOR_MSG_SRC_DEV_ID                          0xC001   //!< uint64, default 0
#define CBOR_MSG_DST_DEV_FILTER                      0xC002   //!< uint64, default set by the protocol
#define CBOR_MSG_REQ_ID                              0xC003   //!< uint16, default set by the protocol
#define CBOR_MSG_ACK_TYPE                            0xC004   //!< uint8 (ack_type), default ACK_MINIMAL
#define CBOR_MSG_ROUTE                               0xC005   //!< array of uint8, up to 4, default set by the protocol
#define CBOR_MSG_TIME_GPS_NS                         0xC006   //!< uint64, default set by the protocol
#define CBOR_MSG_UPTIME_NS                           0xC007   //!< uint64, default set by the protocol
#define CBOR_MSG_STATUS                              0xC008   //!< uint8 (msg_status), default OK
#define CBOR_MSG_ERR_KEYS                            0xC009   //!< key_paths, default set by the protocol
#define CBOR_MSG_FIELDS                              0xC00A   //!< key_paths, default set by the protocol
#define CBOR_MSG_CURSOR                              0xC00B   //!< uint32, default set by the protocol, reserved
#define CBOR_MSG_OPTION_NO_TIME                      0xC00C   //!< bool, default False
/** @} */

/** @name Message-type option keys */
/** @{ */
#define CBOR_SUB_RATE_PERIOD                         0x9300   //!< SUBSCRIBE: uint32, default set by the protocol
#define CBOR_SUB_RATE_MULTI                          0x9301   //!< SUBSCRIBE: uint32, default set by the protocol
#define CBOR_SUB_PORTS                               0x9302   //!< SUBSCRIBE: array of uint8, up to 16, default set by the protocol, reserved
#define CBOR_SUB_OPTION_PERSIST                      0x9303   //!< SUBSCRIBE: bool, default False, reserved
/** @} */

/** Registry enum `msg_type` */
typedef enum
{
    CBOR_MSG_TYPE_DATA = 1,
    CBOR_MSG_TYPE_GET = 2,
    CBOR_MSG_TYPE_SUBSCRIBE = 3,
} eCborMsgType;

/** Registry enum `msg_type_flag` */
typedef enum
{
    CBOR_MSG_TYPE_FLAG_REPLY = 128,
} eCborMsgTypeFlag;

/** Registry enum `msg_status` */
typedef enum
{
    CBOR_MSG_STATUS_OK = 0,
    CBOR_MSG_STATUS_BAD_REQUEST = 1,
    CBOR_MSG_STATUS_BAD_VALUE = 2,
    CBOR_MSG_STATUS_UNSUPPORTED = 3,
    CBOR_MSG_STATUS_RESP_TOO_LARGE = 4,
    CBOR_MSG_STATUS_FORWARD_DENIED = 5,
    CBOR_MSG_STATUS_READ_ONLY = 6,
    CBOR_MSG_STATUS_ERROR = 7,
    CBOR_MSG_STATUS_REQ_TOO_LARGE = 8,
} eCborMsgStatus;

/** Registry enum `ack_type` */
typedef enum
{
    CBOR_ACK_TYPE_ACK_NONE = 0,
    CBOR_ACK_TYPE_ACK_MINIMAL = 1,
    CBOR_ACK_TYPE_ACK_DETAILED = 2,
} eCborAckType;

/** Registry enum `route_action` */
typedef enum
{
    CBOR_ROUTE_ACTION_DENY = 0,
    CBOR_ROUTE_ACTION_ALLOW = 1,
} eCborRouteAction;

/** Registry enum `rmc_preset` */
typedef enum
{
    CBOR_RMC_PRESET_NONE = 0,
    CBOR_RMC_PRESET_IMX_PPD_NO_IMU = 1,
    CBOR_RMC_PRESET_IMX_PPD = 2,
    CBOR_RMC_PRESET_IMX_PPD_IMUS_RAW = 3,
    CBOR_RMC_PRESET_IMX_PPD_IMUS_UNCAL = 4,
    CBOR_RMC_PRESET_INS = 5,
    CBOR_RMC_PRESET_IMX_PPD_RTK_DBG = 6,
    CBOR_RMC_PRESET_IMX_PPD_GROUND_VEHICLE = 7,
    CBOR_RMC_PRESET_ALLAN_VARIANCE = 8,
    CBOR_RMC_PRESET_ALLAN_VARIANCE_IMUS = 9,
    CBOR_RMC_PRESET_GNSS = 10,
    CBOR_RMC_PRESET_GPX_PPD = 11,
} eCborRmcPreset;

/** Registry enum `nmea_speed_filter` */
typedef enum
{
    CBOR_NMEA_SPEED_FILTER_DEFAULT = 0,
    CBOR_NMEA_SPEED_FILTER_ENABLE = 1,
    CBOR_NMEA_SPEED_FILTER_DISABLE = 2,
} eCborNmeaSpeedFilter;

#define CBOR_MSG_TYPE_OP_MASK                        0x7F     //!< MSG_TYPE = operation | flags; the operation is the bits below the lowest flag

/** A field key and its name in the registry */
typedef struct
{
    uint16_t    key;    //!< field key
    const char *name;   //!< field name
} cbor_registry_name_t;

/**
 * @brief Looks up a field's name (cbor_registry_names.c, SDK only).
 * @param did  root DID key
 * @param key  field key within the DID
 * @return the field name, or NULL if the registry has no such field
 */
const char *cbor_registry_field_name(uint16_t did, uint16_t key);

/**
 * @brief Looks up a field's key by name (cbor_registry_names.c, SDK only).
 * @param did   root DID key
 * @param name  field name
 * @return the field key, or -1 if the registry has no such field
 */
int cbor_registry_field_key(uint16_t did, const char *name);

#ifdef __cplusplus
}
#endif

#include "cbor/dids/cbor_did_flash_cfg.h"
#include "cbor/dids/cbor_did_can_config.h"
#include "cbor/dids/cbor_did_gpx_flash_cfg.h"
#include "cbor/dids/cbor_did_rmc_stored.h"
#include "cbor/dids/cbor_did_rmc.h"
#include "cbor/dids/cbor_did_route_filter.h"

#endif /* CBOR_REGISTRY_H */
