/**
 * @file cbor_did_config_gnss.h
 * @brief CONFIG_GNSS: CBOR-only DID, served by IMX-5, IMX-6, GPX-1. Field keys.
 *
 * Generated from the CBOR key registry by is-common scripts/cbor_registry.py -- do not edit.
 *
 * @copyright Copyright (c) 2026 Inertial Sense, Inc.
 */

#ifndef CBOR_DID_CONFIG_GNSS_H
#define CBOR_DID_CONFIG_GNSS_H

#include "cbor/cbor_registry.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @name CONFIG_GNSS field keys */
/** @{ */
#define CBOR_CONFIG_GNSS__PERIODMS                   0        //!< periodMs: uint32, default 200
#define CBOR_CONFIG_GNSS__PRIMARYRECEIVER            1        //!< primaryReceiver: uint8, default 0
#define CBOR_CONFIG_GNSS__TIMESYNC                   2        //!< timeSync: map
#define CBOR_CONFIG_GNSS__TIMESYNC_PERIODMS          0        //!< periodMs: uint32, default 1000
#define CBOR_CONFIG_GNSS__TIMESYNC_USERDELAYS        1        //!< userDelayS: float32, default 0
#define CBOR_CONFIG_GNSS__EXTERNALAIDING             3        //!< externalAiding: map
#define CBOR_CONFIG_GNSS__EXTERNALAIDING_FIXEDLATENCY 0        //!< fixedLatency: bool, default false
#define CBOR_CONFIG_GNSS__RECEIVERS                  4        //!< Receivers: map
#define CBOR_CONFIG_GNSS__RECEIVERS_TYPE             0        //!< type: uint8, default set by the firmware
#define CBOR_CONFIG_GNSS__RECEIVERS_PORT             1        //!< port: uint8, default set by the firmware
#define CBOR_CONFIG_GNSS__RECEIVERS_PPSPIN           2        //!< ppsPin: uint8, default set by the firmware
#define CBOR_CONFIG_GNSS__RECEIVERS_CONFIGURE        3        //!< configure: bool, default set by the firmware
#define CBOR_CONFIG_GNSS__RECEIVERS_FUSIONENABLED    4        //!< fusionEnabled: bool, default true
#define CBOR_CONFIG_GNSS__RECEIVERS_SATSIGNALS       5        //!< satSignals: uint16, default 5119
#define CBOR_CONFIG_GNSS__RECEIVERS_MINELEVATION     6        //!< minElevation: float32, default 0.17453292
#define CBOR_CONFIG_GNSS__RECEIVERS_CN0MIN           7        //!< cn0Min: uint8, default 30
#define CBOR_CONFIG_GNSS__RECEIVERS_CN0DYNOFFSET     8        //!< cn0DynOffset: uint8, default 20
#define CBOR_CONFIG_GNSS__RECEIVERS_DYNAMICMODEL     9        //!< dynamicModel: uint8, default 8
#define CBOR_CONFIG_GNSS__RECEIVER                   5        //!< Receiver: map
#define CBOR_CONFIG_GNSS__RECEIVER_GNSS1             0        //!< GNSS1: map
#define CBOR_CONFIG_GNSS__RECEIVER_GNSS1_TYPE        0        //!< type: uint8, default set by the firmware
#define CBOR_CONFIG_GNSS__RECEIVER_GNSS1_PORT        1        //!< port: uint8, default set by the firmware
#define CBOR_CONFIG_GNSS__RECEIVER_GNSS1_PPSPIN      2        //!< ppsPin: uint8, default set by the firmware
#define CBOR_CONFIG_GNSS__RECEIVER_GNSS1_CONFIGURE   3        //!< configure: bool, default set by the firmware
#define CBOR_CONFIG_GNSS__RECEIVER_GNSS1_FUSIONENABLED 4        //!< fusionEnabled: bool, default true
#define CBOR_CONFIG_GNSS__RECEIVER_GNSS1_SATSIGNALS  5        //!< satSignals: uint16, default 5119
#define CBOR_CONFIG_GNSS__RECEIVER_GNSS1_MINELEVATION 6        //!< minElevation: float32, default 0.17453292
#define CBOR_CONFIG_GNSS__RECEIVER_GNSS1_CN0MIN      7        //!< cn0Min: uint8, default 30
#define CBOR_CONFIG_GNSS__RECEIVER_GNSS1_CN0DYNOFFSET 8        //!< cn0DynOffset: uint8, default 20
#define CBOR_CONFIG_GNSS__RECEIVER_GNSS1_DYNAMICMODEL 9        //!< dynamicModel: uint8, default 8
#define CBOR_CONFIG_GNSS__RECEIVER_GNSS2             1        //!< GNSS2: map
#define CBOR_CONFIG_GNSS__RECEIVER_GNSS2_TYPE        0        //!< type: uint8, default set by the firmware
#define CBOR_CONFIG_GNSS__RECEIVER_GNSS2_PORT        1        //!< port: uint8, default set by the firmware
#define CBOR_CONFIG_GNSS__RECEIVER_GNSS2_PPSPIN      2        //!< ppsPin: uint8, default set by the firmware
#define CBOR_CONFIG_GNSS__RECEIVER_GNSS2_CONFIGURE   3        //!< configure: bool, default set by the firmware
#define CBOR_CONFIG_GNSS__RECEIVER_GNSS2_FUSIONENABLED 4        //!< fusionEnabled: bool, default true
#define CBOR_CONFIG_GNSS__RECEIVER_GNSS2_SATSIGNALS  5        //!< satSignals: uint16, default 5119
#define CBOR_CONFIG_GNSS__RECEIVER_GNSS2_MINELEVATION 6        //!< minElevation: float32, default 0.17453292
#define CBOR_CONFIG_GNSS__RECEIVER_GNSS2_CN0MIN      7        //!< cn0Min: uint8, default 30
#define CBOR_CONFIG_GNSS__RECEIVER_GNSS2_CN0DYNOFFSET 8        //!< cn0DynOffset: uint8, default 20
#define CBOR_CONFIG_GNSS__RECEIVER_GNSS2_DYNAMICMODEL 9        //!< dynamicModel: uint8, default 8
#define CBOR_CONFIG_GNSS__ANTENNAS                   6        //!< Antennas: map
#define CBOR_CONFIG_GNSS__ANTENNAS_RECEIVER          0        //!< receiver: uint8, default set by the firmware
#define CBOR_CONFIG_GNSS__ANTENNAS_OFFSET            1        //!< offset: float32[3], default [0, 0, 0]
#define CBOR_CONFIG_GNSS__ANTENNAS_OFFSETESTIMATION  2        //!< offsetEstimation: bool, default false
#define CBOR_CONFIG_GNSS__ANTENNAS_POWERENABLED      3        //!< powerEnabled: bool, default true
#define CBOR_CONFIG_GNSS__ANTENNA                    7        //!< Antenna: map
#define CBOR_CONFIG_GNSS__ANTENNA_ANT1               0        //!< ANT1: map
#define CBOR_CONFIG_GNSS__ANTENNA_ANT1_RECEIVER      0        //!< receiver: uint8, default set by the firmware
#define CBOR_CONFIG_GNSS__ANTENNA_ANT1_OFFSET        1        //!< offset: float32[3], default [0, 0, 0]
#define CBOR_CONFIG_GNSS__ANTENNA_ANT1_OFFSETESTIMATION 2        //!< offsetEstimation: bool, default false
#define CBOR_CONFIG_GNSS__ANTENNA_ANT1_POWERENABLED  3        //!< powerEnabled: bool, default true
#define CBOR_CONFIG_GNSS__ANTENNA_ANT2               1        //!< ANT2: map
#define CBOR_CONFIG_GNSS__ANTENNA_ANT2_RECEIVER      0        //!< receiver: uint8, default set by the firmware
#define CBOR_CONFIG_GNSS__ANTENNA_ANT2_OFFSET        1        //!< offset: float32[3], default [0, 0, 0]
#define CBOR_CONFIG_GNSS__ANTENNA_ANT2_OFFSETESTIMATION 2        //!< offsetEstimation: bool, default false
#define CBOR_CONFIG_GNSS__ANTENNA_ANT2_POWERENABLED  3        //!< powerEnabled: bool, default true
#define CBOR_CONFIG_GNSS__RTK                        8        //!< Rtk: map
#define CBOR_CONFIG_GNSS__RTK_MODE                   0        //!< mode: uint8, default 0
#define CBOR_CONFIG_GNSS__RTK_BASEPOSITION           1        //!< basePosition: float64[3], default [0, 0, 0]
#define CBOR_CONFIG_GNSS__RTK_BASEPOSITIONMOVING     2        //!< basePositionMoving: bool, default false
#define CBOR_CONFIG_GNSS__RTK_BASEIDENTICALTOROVER   3        //!< baseIdenticalToRover: bool, default false
#define CBOR_CONFIG_GNSS__RTK_PASSTHROUGH            4        //!< passThrough: bool, default false
#define CBOR_CONFIG_GNSS__RTK_MULTIPATHMITIGATION    5        //!< multipathMitigation: bool, default false
#define CBOR_CONFIG_GNSS__RTK_BASEOUTPUT             6        //!< BaseOutput: map
#define CBOR_CONFIG_GNSS__RTK_BASEOUTPUT_GNSS1_UBLOX 0        //!< GNSS1_UBLOX: map
#define CBOR_CONFIG_GNSS__RTK_BASEOUTPUT_GNSS1_UBLOX_SER0 0        //!< ser0: bool, default false
#define CBOR_CONFIG_GNSS__RTK_BASEOUTPUT_GNSS1_UBLOX_SER1 1        //!< ser1: bool, default false
#define CBOR_CONFIG_GNSS__RTK_BASEOUTPUT_GNSS1_UBLOX_SER2 2        //!< ser2: bool, default false
#define CBOR_CONFIG_GNSS__RTK_BASEOUTPUT_GNSS1_UBLOX_USB 3        //!< usb: bool, default false
#define CBOR_CONFIG_GNSS__RTK_BASEOUTPUT_GNSS1_RTCM3 1        //!< GNSS1_RTCM3: map
#define CBOR_CONFIG_GNSS__RTK_BASEOUTPUT_GNSS1_RTCM3_SER0 0        //!< ser0: bool, default false
#define CBOR_CONFIG_GNSS__RTK_BASEOUTPUT_GNSS1_RTCM3_SER1 1        //!< ser1: bool, default false
#define CBOR_CONFIG_GNSS__RTK_BASEOUTPUT_GNSS1_RTCM3_SER2 2        //!< ser2: bool, default false
#define CBOR_CONFIG_GNSS__RTK_BASEOUTPUT_GNSS1_RTCM3_USB 3        //!< usb: bool, default false
#define CBOR_CONFIG_GNSS__RTK_BASEOUTPUT_GNSS2_UBLOX 2        //!< GNSS2_UBLOX: map
#define CBOR_CONFIG_GNSS__RTK_BASEOUTPUT_GNSS2_UBLOX_SER0 0        //!< ser0: bool, default false
#define CBOR_CONFIG_GNSS__RTK_BASEOUTPUT_GNSS2_UBLOX_SER1 1        //!< ser1: bool, default false
#define CBOR_CONFIG_GNSS__RTK_BASEOUTPUT_GNSS2_UBLOX_SER2 2        //!< ser2: bool, default false
#define CBOR_CONFIG_GNSS__RTK_BASEOUTPUT_GNSS2_UBLOX_USB 3        //!< usb: bool, default false
#define CBOR_CONFIG_GNSS__RTK_BASEOUTPUT_GNSS2_RTCM3 3        //!< GNSS2_RTCM3: map
#define CBOR_CONFIG_GNSS__RTK_BASEOUTPUT_GNSS2_RTCM3_SER0 0        //!< ser0: bool, default false
#define CBOR_CONFIG_GNSS__RTK_BASEOUTPUT_GNSS2_RTCM3_SER1 1        //!< ser1: bool, default false
#define CBOR_CONFIG_GNSS__RTK_BASEOUTPUT_GNSS2_RTCM3_SER2 2        //!< ser2: bool, default false
#define CBOR_CONFIG_GNSS__RTK_BASEOUTPUT_GNSS2_RTCM3_USB 3        //!< usb: bool, default false
/** @} */

#define CBOR_CONFIG_GNSS_NAME_COUNT                  85       //!< entries in g_cborNames_CONFIG_GNSS

/** CONFIG_GNSS field names, in registry order */
extern const cbor_registry_name_t g_cborNames_CONFIG_GNSS[CBOR_CONFIG_GNSS_NAME_COUNT];

/** The parent of each g_cborNames_CONFIG_GNSS entry: the index of its enclosing map's entry, or 0xFFFF */
extern const uint16_t g_cborNameParents_CONFIG_GNSS[CBOR_CONFIG_GNSS_NAME_COUNT];

/** CONFIG_GNSS.timeSync: one member per key, in registry types */
typedef struct
{
    uint32_t        periodMs;               //!< key 0: uint32, default 1000
    float           userDelayS;             //!< key 1: float32, default 0
} cbor_config_gnss_timesync_t;

/** CONFIG_GNSS.externalAiding: one member per key, in registry types */
typedef struct
{
    uint8_t         fixedLatency;           //!< key 0: bool, default false
} cbor_config_gnss_externalaiding_t;

/** CONFIG_GNSS.Rtk: one member per key, in registry types */
typedef struct
{
    uint8_t         mode;                   //!< key 0: uint8, default 0
    double          basePosition[3];        //!< key 1: float64[3], default [0, 0, 0]
    uint8_t         basePositionMoving;     //!< key 2: bool, default false
    uint8_t         baseIdenticalToRover;   //!< key 3: bool, default false
    uint8_t         passThrough;            //!< key 4: bool, default false
    uint8_t         multipathMitigation;    //!< key 5: bool, default false
    cbor_keyset_base_output_t BaseOutput[4];  //!< key 6: map of base_output by base_stream
} cbor_config_gnss_rtk_t;

/** CONFIG_GNSS: one member per key, in registry types */
typedef struct
{
    uint32_t        periodMs;               //!< key 0: uint32, default 200
    uint8_t         primaryReceiver;        //!< key 1: uint8, default 0
    cbor_config_gnss_timesync_t timeSync;  //!< key 2: map
    cbor_config_gnss_externalaiding_t externalAiding;  //!< key 3: map
    cbor_keyset_receiver_t Receivers;  //!< key 4: receiver, common to every instance
    cbor_keyset_receiver_t Receiver[2];  //!< key 5: map of receiver by receiver_id
    cbor_keyset_antenna_t Antennas;  //!< key 6: antenna, common to every instance
    cbor_keyset_antenna_t Antenna[2];  //!< key 7: map of antenna by antenna_id
    cbor_config_gnss_rtk_t Rtk;  //!< key 8: map
} cbor_config_gnss_t;

/** Schema of every CONFIG_GNSS key, over cbor_config_gnss_t; a device stores and accepts a subset of them */
extern const cbor_schema_t g_cborSchema_CONFIG_GNSS;

/** The registry's recorded defaults; keys whose default the firmware sets are zero */
extern const cbor_config_gnss_t g_cborDefaults_CONFIG_GNSS;

/** One per leaf key, depth first in key order: 1 if the registry records a fixed default, 0 if the firmware sets it */
extern const uint8_t g_cborDefaultFixed_CONFIG_GNSS[69];

#ifdef __cplusplus
}
#endif

#endif /* CBOR_DID_CONFIG_GNSS_H */
