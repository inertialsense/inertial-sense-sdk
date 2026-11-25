/**
 * @file cbor_did_config_can.h
 * @brief CONFIG_CAN: CBOR-only DID, served by IMX-5, IMX-6. Field keys.
 *
 * Generated from the CBOR key registry by is-common scripts/cbor_registry.py -- do not edit.
 *
 * @copyright Copyright (c) 2026 Inertial Sense, Inc.
 */

#ifndef CBOR_DID_CONFIG_CAN_H
#define CBOR_DID_CONFIG_CAN_H

#include "cbor/cbor_registry.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @name CONFIG_CAN field keys */
/** @{ */
#define CBOR_CONFIG_CAN__BITRATEKBPS                 0        //!< bitRateKbps: uint16, default 100
#define CBOR_CONFIG_CAN__FDENABLED                   1        //!< fdEnabled: bool, default false
#define CBOR_CONFIG_CAN__RECEIVEADDRESS              2        //!< receiveAddress: uint32, default 0
#define CBOR_CONFIG_CAN__MESSAGE                     3        //!< Message: map
#define CBOR_CONFIG_CAN__MESSAGE_INS_TIME            0        //!< INS_TIME: map
#define CBOR_CONFIG_CAN__MESSAGE_INS_TIME_PERIODMULTIPLE 0        //!< periodMultiple: uint16, default 0
#define CBOR_CONFIG_CAN__MESSAGE_INS_TIME_TRANSMITADDRESS 1        //!< transmitAddress: uint32, default 0
#define CBOR_CONFIG_CAN__MESSAGE_INS_STATUS          1        //!< INS_STATUS: map
#define CBOR_CONFIG_CAN__MESSAGE_INS_STATUS_PERIODMULTIPLE 0        //!< periodMultiple: uint16, default 0
#define CBOR_CONFIG_CAN__MESSAGE_INS_STATUS_TRANSMITADDRESS 1        //!< transmitAddress: uint32, default 0
#define CBOR_CONFIG_CAN__MESSAGE_INS_EULER           2        //!< INS_EULER: map
#define CBOR_CONFIG_CAN__MESSAGE_INS_EULER_PERIODMULTIPLE 0        //!< periodMultiple: uint16, default 0
#define CBOR_CONFIG_CAN__MESSAGE_INS_EULER_TRANSMITADDRESS 1        //!< transmitAddress: uint32, default 0
#define CBOR_CONFIG_CAN__MESSAGE_INS_QUATN2B         3        //!< INS_QUATN2B: map
#define CBOR_CONFIG_CAN__MESSAGE_INS_QUATN2B_PERIODMULTIPLE 0        //!< periodMultiple: uint16, default 0
#define CBOR_CONFIG_CAN__MESSAGE_INS_QUATN2B_TRANSMITADDRESS 1        //!< transmitAddress: uint32, default 0
#define CBOR_CONFIG_CAN__MESSAGE_INS_QUATE2B         4        //!< INS_QUATE2B: map
#define CBOR_CONFIG_CAN__MESSAGE_INS_QUATE2B_PERIODMULTIPLE 0        //!< periodMultiple: uint16, default 0
#define CBOR_CONFIG_CAN__MESSAGE_INS_QUATE2B_TRANSMITADDRESS 1        //!< transmitAddress: uint32, default 0
#define CBOR_CONFIG_CAN__MESSAGE_INS_UVW             5        //!< INS_UVW: map
#define CBOR_CONFIG_CAN__MESSAGE_INS_UVW_PERIODMULTIPLE 0        //!< periodMultiple: uint16, default 0
#define CBOR_CONFIG_CAN__MESSAGE_INS_UVW_TRANSMITADDRESS 1        //!< transmitAddress: uint32, default 0
#define CBOR_CONFIG_CAN__MESSAGE_INS_VE              6        //!< INS_VE: map
#define CBOR_CONFIG_CAN__MESSAGE_INS_VE_PERIODMULTIPLE 0        //!< periodMultiple: uint16, default 0
#define CBOR_CONFIG_CAN__MESSAGE_INS_VE_TRANSMITADDRESS 1        //!< transmitAddress: uint32, default 0
#define CBOR_CONFIG_CAN__MESSAGE_INS_LAT             7        //!< INS_LAT: map
#define CBOR_CONFIG_CAN__MESSAGE_INS_LAT_PERIODMULTIPLE 0        //!< periodMultiple: uint16, default 0
#define CBOR_CONFIG_CAN__MESSAGE_INS_LAT_TRANSMITADDRESS 1        //!< transmitAddress: uint32, default 0
#define CBOR_CONFIG_CAN__MESSAGE_INS_LON             8        //!< INS_LON: map
#define CBOR_CONFIG_CAN__MESSAGE_INS_LON_PERIODMULTIPLE 0        //!< periodMultiple: uint16, default 0
#define CBOR_CONFIG_CAN__MESSAGE_INS_LON_TRANSMITADDRESS 1        //!< transmitAddress: uint32, default 0
#define CBOR_CONFIG_CAN__MESSAGE_INS_ALT             9        //!< INS_ALT: map
#define CBOR_CONFIG_CAN__MESSAGE_INS_ALT_PERIODMULTIPLE 0        //!< periodMultiple: uint16, default 0
#define CBOR_CONFIG_CAN__MESSAGE_INS_ALT_TRANSMITADDRESS 1        //!< transmitAddress: uint32, default 0
#define CBOR_CONFIG_CAN__MESSAGE_INS_NORTH_EAST      10       //!< INS_NORTH_EAST: map
#define CBOR_CONFIG_CAN__MESSAGE_INS_NORTH_EAST_PERIODMULTIPLE 0        //!< periodMultiple: uint16, default 0
#define CBOR_CONFIG_CAN__MESSAGE_INS_NORTH_EAST_TRANSMITADDRESS 1        //!< transmitAddress: uint32, default 0
#define CBOR_CONFIG_CAN__MESSAGE_INS_DOWN            11       //!< INS_DOWN: map
#define CBOR_CONFIG_CAN__MESSAGE_INS_DOWN_PERIODMULTIPLE 0        //!< periodMultiple: uint16, default 0
#define CBOR_CONFIG_CAN__MESSAGE_INS_DOWN_TRANSMITADDRESS 1        //!< transmitAddress: uint32, default 0
#define CBOR_CONFIG_CAN__MESSAGE_INS_ECEF_X          12       //!< INS_ECEF_X: map
#define CBOR_CONFIG_CAN__MESSAGE_INS_ECEF_X_PERIODMULTIPLE 0        //!< periodMultiple: uint16, default 0
#define CBOR_CONFIG_CAN__MESSAGE_INS_ECEF_X_TRANSMITADDRESS 1        //!< transmitAddress: uint32, default 0
#define CBOR_CONFIG_CAN__MESSAGE_INS_ECEF_Y          13       //!< INS_ECEF_Y: map
#define CBOR_CONFIG_CAN__MESSAGE_INS_ECEF_Y_PERIODMULTIPLE 0        //!< periodMultiple: uint16, default 0
#define CBOR_CONFIG_CAN__MESSAGE_INS_ECEF_Y_TRANSMITADDRESS 1        //!< transmitAddress: uint32, default 0
#define CBOR_CONFIG_CAN__MESSAGE_INS_ECEF_Z          14       //!< INS_ECEF_Z: map
#define CBOR_CONFIG_CAN__MESSAGE_INS_ECEF_Z_PERIODMULTIPLE 0        //!< periodMultiple: uint16, default 0
#define CBOR_CONFIG_CAN__MESSAGE_INS_ECEF_Z_TRANSMITADDRESS 1        //!< transmitAddress: uint32, default 0
#define CBOR_CONFIG_CAN__MESSAGE_INS_MSL             15       //!< INS_MSL: map
#define CBOR_CONFIG_CAN__MESSAGE_INS_MSL_PERIODMULTIPLE 0        //!< periodMultiple: uint16, default 0
#define CBOR_CONFIG_CAN__MESSAGE_INS_MSL_TRANSMITADDRESS 1        //!< transmitAddress: uint32, default 0
#define CBOR_CONFIG_CAN__MESSAGE_PREINT_PX           16       //!< PREINT_PX: map
#define CBOR_CONFIG_CAN__MESSAGE_PREINT_PX_PERIODMULTIPLE 0        //!< periodMultiple: uint16, default 0
#define CBOR_CONFIG_CAN__MESSAGE_PREINT_PX_TRANSMITADDRESS 1        //!< transmitAddress: uint32, default 0
#define CBOR_CONFIG_CAN__MESSAGE_PREINT_QY           17       //!< PREINT_QY: map
#define CBOR_CONFIG_CAN__MESSAGE_PREINT_QY_PERIODMULTIPLE 0        //!< periodMultiple: uint16, default 0
#define CBOR_CONFIG_CAN__MESSAGE_PREINT_QY_TRANSMITADDRESS 1        //!< transmitAddress: uint32, default 0
#define CBOR_CONFIG_CAN__MESSAGE_PREINT_RZ           18       //!< PREINT_RZ: map
#define CBOR_CONFIG_CAN__MESSAGE_PREINT_RZ_PERIODMULTIPLE 0        //!< periodMultiple: uint16, default 0
#define CBOR_CONFIG_CAN__MESSAGE_PREINT_RZ_TRANSMITADDRESS 1        //!< transmitAddress: uint32, default 0
#define CBOR_CONFIG_CAN__MESSAGE_DUAL_PX             19       //!< DUAL_PX: map
#define CBOR_CONFIG_CAN__MESSAGE_DUAL_PX_PERIODMULTIPLE 0        //!< periodMultiple: uint16, default 0
#define CBOR_CONFIG_CAN__MESSAGE_DUAL_PX_TRANSMITADDRESS 1        //!< transmitAddress: uint32, default 0
#define CBOR_CONFIG_CAN__MESSAGE_DUAL_QY             20       //!< DUAL_QY: map
#define CBOR_CONFIG_CAN__MESSAGE_DUAL_QY_PERIODMULTIPLE 0        //!< periodMultiple: uint16, default 0
#define CBOR_CONFIG_CAN__MESSAGE_DUAL_QY_TRANSMITADDRESS 1        //!< transmitAddress: uint32, default 0
#define CBOR_CONFIG_CAN__MESSAGE_DUAL_RZ             21       //!< DUAL_RZ: map
#define CBOR_CONFIG_CAN__MESSAGE_DUAL_RZ_PERIODMULTIPLE 0        //!< periodMultiple: uint16, default 0
#define CBOR_CONFIG_CAN__MESSAGE_DUAL_RZ_TRANSMITADDRESS 1        //!< transmitAddress: uint32, default 0
#define CBOR_CONFIG_CAN__MESSAGE_GNSS1_POS           22       //!< GNSS1_POS: map
#define CBOR_CONFIG_CAN__MESSAGE_GNSS1_POS_PERIODMULTIPLE 0        //!< periodMultiple: uint16, default 0
#define CBOR_CONFIG_CAN__MESSAGE_GNSS1_POS_TRANSMITADDRESS 1        //!< transmitAddress: uint32, default 0
#define CBOR_CONFIG_CAN__MESSAGE_GNSS2_POS           23       //!< GNSS2_POS: map
#define CBOR_CONFIG_CAN__MESSAGE_GNSS2_POS_PERIODMULTIPLE 0        //!< periodMultiple: uint16, default 0
#define CBOR_CONFIG_CAN__MESSAGE_GNSS2_POS_TRANSMITADDRESS 1        //!< transmitAddress: uint32, default 0
#define CBOR_CONFIG_CAN__MESSAGE_GNSS1_RTK_POS_REL   24       //!< GNSS1_RTK_POS_REL: map
#define CBOR_CONFIG_CAN__MESSAGE_GNSS1_RTK_POS_REL_PERIODMULTIPLE 0        //!< periodMultiple: uint16, default 0
#define CBOR_CONFIG_CAN__MESSAGE_GNSS1_RTK_POS_REL_TRANSMITADDRESS 1        //!< transmitAddress: uint32, default 0
#define CBOR_CONFIG_CAN__MESSAGE_GNSS2_RTK_CMP_REL   25       //!< GNSS2_RTK_CMP_REL: map
#define CBOR_CONFIG_CAN__MESSAGE_GNSS2_RTK_CMP_REL_PERIODMULTIPLE 0        //!< periodMultiple: uint16, default 0
#define CBOR_CONFIG_CAN__MESSAGE_GNSS2_RTK_CMP_REL_TRANSMITADDRESS 1        //!< transmitAddress: uint32, default 0
#define CBOR_CONFIG_CAN__MESSAGE_ROLL_ROLLRATE       26       //!< ROLL_ROLLRATE: map
#define CBOR_CONFIG_CAN__MESSAGE_ROLL_ROLLRATE_PERIODMULTIPLE 0        //!< periodMultiple: uint16, default 0
#define CBOR_CONFIG_CAN__MESSAGE_ROLL_ROLLRATE_TRANSMITADDRESS 1        //!< transmitAddress: uint32, default 0
/** @} */

#define CBOR_CONFIG_CAN_NAME_COUNT                   85       //!< entries in g_cborNames_CONFIG_CAN

/** CONFIG_CAN field names, in registry order */
extern const cbor_registry_name_t g_cborNames_CONFIG_CAN[CBOR_CONFIG_CAN_NAME_COUNT];

/** CONFIG_CAN: one member per key, in registry types */
typedef struct
{
    uint16_t        bitRateKbps;            //!< key 0: uint16, default 100
    uint8_t         fdEnabled;              //!< key 1: bool, default false
    uint32_t        receiveAddress;         //!< key 2: uint32, default 0
    cbor_keyset_can_message_t Message[27];  //!< key 3: map of can_message by can_message
} cbor_config_can_t;

/** Schema of every CONFIG_CAN key, over cbor_config_can_t; a device stores and accepts a subset of them */
extern const cbor_schema_t g_cborSchema_CONFIG_CAN;

/** The registry's recorded defaults; keys whose default the firmware sets are zero */
extern const cbor_config_can_t g_cborDefaults_CONFIG_CAN;

/** One per leaf key, depth first in key order: 1 if the registry records a fixed default, 0 if the firmware sets it */
extern const uint8_t g_cborDefaultFixed_CONFIG_CAN[57];

#ifdef __cplusplus
}
#endif

#endif /* CBOR_DID_CONFIG_CAN_H */
