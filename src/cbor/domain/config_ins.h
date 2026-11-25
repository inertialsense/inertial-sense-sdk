/**
 * @file config_ins.h
 * @brief CONFIG_INS: domain, served by IMX-5, IMX-6. Field keys, view types, schemas and recorded defaults.
 *
 * Generated from the CBOR key registry by is-common scripts/cbor_registry.py -- do not edit.
 *
 * @copyright Copyright (c) 2026 Inertial Sense, Inc.
 */

#ifndef CBOR_DOMAIN_CONFIG_INS_H
#define CBOR_DOMAIN_CONFIG_INS_H

#include "cbor/cbor_registry.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @name CONFIG_INS field keys */
/** @{ */
#define CBOR_CONFIG_INS__ROTATION                    0        //!< rotation: float32[3], default [0, 0, 0]
#define CBOR_CONFIG_INS__OFFSET                      1        //!< offset: float32[3], default [0, 0, 0]
#define CBOR_CONFIG_INS__NEDORIGIN                   2        //!< nedOrigin: float64[3], default set by the firmware
#define CBOR_CONFIG_INS__DYNAMICMODEL                3        //!< dynamicModel: uint8, default 8
#define CBOR_CONFIG_INS__EKFENABLED                  4        //!< ekfEnabled: bool, default true
#define CBOR_CONFIG_INS__DEADRECKONINGTIMEOUTS       5        //!< deadReckoningTimeoutS: uint8, default 30
#define CBOR_CONFIG_INS__LASTPOSITIONUPDATEDISTANCEM 6        //!< lastPositionUpdateDistanceM: float32, default 1000
#define CBOR_CONFIG_INS__REFERENCEIMU                7        //!< referenceImu: bool, default false
#define CBOR_CONFIG_INS__STATIONARYONSTROBE          8        //!< stationaryOnStrobe: bool, default false
#define CBOR_CONFIG_INS__FUSION                      9        //!< fusion: map
#define CBOR_CONFIG_INS__FUSION_MAG                  0        //!< mag: bool, default true
#define CBOR_CONFIG_INS__FUSION_BARO                 1        //!< baro: bool, default true
#define CBOR_CONFIG_INS__FUSION_WHEEL                2        //!< wheel: bool, default true
#define CBOR_CONFIG_INS__ZEROVELOCITY                10       //!< zeroVelocity: map
#define CBOR_CONFIG_INS__ZEROVELOCITY_ROTATION       0        //!< rotation: float32[3], default [0, 0, 0]
#define CBOR_CONFIG_INS__ZEROVELOCITY_OFFSET         1        //!< offset: float32[3], default [0, 0, 0]
#define CBOR_CONFIG_INS__ZEROVELOCITY_AUTOZUPT       2        //!< autoZupt: bool, default true
#define CBOR_CONFIG_INS__ZEROVELOCITY_AUTOZARU       3        //!< autoZaru: bool, default true
#define CBOR_CONFIG_INS__SHOCK                       11       //!< shock: map
#define CBOR_CONFIG_INS__SHOCK_ENABLED               0        //!< enabled: bool, default false
#define CBOR_CONFIG_INS__SHOCK_FASTRECOVERY          1        //!< fastRecovery: bool, default true
#define CBOR_CONFIG_INS__SHOCK_DETECTLATENCYMS       2        //!< detectLatencyMs: uint8, default 10
#define CBOR_CONFIG_INS__SHOCK_REJECTLATCHMS         3        //!< rejectLatchMs: uint8, default 5
#define CBOR_CONFIG_INS__SHOCK_DELTAACCHIGH          4        //!< deltaAccHigh: uint8, default 20
#define CBOR_CONFIG_INS__SHOCK_DELTAACCLOW           5        //!< deltaAccLow: uint8, default 3
#define CBOR_CONFIG_INS__SHOCK_DELTAGYROHIGH         6        //!< deltaGyroHigh: uint8, default 120
#define CBOR_CONFIG_INS__SHOCK_DELTAGYROLOW          7        //!< deltaGyroLow: uint8, default 12
#define CBOR_CONFIG_INS__FAULTDETECT                 12       //!< faultDetect: map
#define CBOR_CONFIG_INS__FAULTDETECT_GYR             0        //!< gyr: bool, default false
#define CBOR_CONFIG_INS__FAULTDETECT_ACC             1        //!< acc: bool, default false
#define CBOR_CONFIG_INS__FAULTDETECT_GYROREJECTTHRESHLOW 2        //!< gyroRejectThreshLow: uint8, default 2
#define CBOR_CONFIG_INS__FAULTDETECT_GYROREJECTTHRESHHIGH 3        //!< gyroRejectThreshHigh: uint8, default 3
#define CBOR_CONFIG_INS__COMPLEMENTARYFILTER         13       //!< complementaryFilter: map
#define CBOR_CONFIG_INS__COMPLEMENTARYFILTER_BKPQR   0        //!< bKpqr: float32, default set by the firmware
#define CBOR_CONFIG_INS__COMPLEMENTARYFILTER_BKUVW   1        //!< bKuvw: float32, default set by the firmware
#define CBOR_CONFIG_INS__COMPLEMENTARYFILTER_OKAT1   2        //!< oKat1: float32, default set by the firmware
#define CBOR_CONFIG_INS__COMPLEMENTARYFILTER_OKAT2   3        //!< oKat2: float32, default set by the firmware
#define CBOR_CONFIG_INS__COMPLEMENTARYFILTER_OKUVW   4        //!< oKuvw: float32, default set by the firmware
#define CBOR_CONFIG_INS__COMPLEMENTARYFILTER_OKLLA   5        //!< oKlla: float32, default set by the firmware
/** @} */

#define CBOR_CONFIG_INS_NAME_COUNT                   39       //!< entries in g_cborNames_CONFIG_INS

/** CONFIG_INS field names, in registry order */
extern const cbor_registry_name_t g_cborNames_CONFIG_INS[CBOR_CONFIG_INS_NAME_COUNT];

/** The parent of each g_cborNames_CONFIG_INS entry: the index of its enclosing map's entry, or 0xFFFF */
extern const uint16_t g_cborNameParents_CONFIG_INS[CBOR_CONFIG_INS_NAME_COUNT];

/** CONFIG_INS.fusion: one member per key, in registry types */
typedef struct
{
    uint8_t         mag;                    //!< key 0: bool, default true
    uint8_t         baro;                   //!< key 1: bool, default true
    uint8_t         wheel;                  //!< key 2: bool, default true
} cbor_config_ins_fusion_t;

/** CONFIG_INS.zeroVelocity: one member per key, in registry types */
typedef struct
{
    float           rotation[3];            //!< key 0: float32[3], default [0, 0, 0]
    float           offset[3];              //!< key 1: float32[3], default [0, 0, 0]
    uint8_t         autoZupt;               //!< key 2: bool, default true
    uint8_t         autoZaru;               //!< key 3: bool, default true
} cbor_config_ins_zerovelocity_t;

/** CONFIG_INS.shock: one member per key, in registry types */
typedef struct
{
    uint8_t         enabled;                //!< key 0: bool, default false
    uint8_t         fastRecovery;           //!< key 1: bool, default true
    uint8_t         detectLatencyMs;        //!< key 2: uint8, default 10
    uint8_t         rejectLatchMs;          //!< key 3: uint8, default 5
    uint8_t         deltaAccHigh;           //!< key 4: uint8, default 20
    uint8_t         deltaAccLow;            //!< key 5: uint8, default 3
    uint8_t         deltaGyroHigh;          //!< key 6: uint8, default 120
    uint8_t         deltaGyroLow;           //!< key 7: uint8, default 12
} cbor_config_ins_shock_t;

/** CONFIG_INS.faultDetect: one member per key, in registry types */
typedef struct
{
    uint8_t         gyr;                    //!< key 0: bool, default false
    uint8_t         acc;                    //!< key 1: bool, default false
    uint8_t         gyroRejectThreshLow;    //!< key 2: uint8, default 2
    uint8_t         gyroRejectThreshHigh;   //!< key 3: uint8, default 3
} cbor_config_ins_faultdetect_t;

/** CONFIG_INS.complementaryFilter: one member per key, in registry types */
typedef struct
{
    float           bKpqr;                  //!< key 0: float32, default set by the firmware
    float           bKuvw;                  //!< key 1: float32, default set by the firmware
    float           oKat1;                  //!< key 2: float32, default set by the firmware
    float           oKat2;                  //!< key 3: float32, default set by the firmware
    float           oKuvw;                  //!< key 4: float32, default set by the firmware
    float           oKlla;                  //!< key 5: float32, default set by the firmware
} cbor_config_ins_complementaryfilter_t;

/** CONFIG_INS: one member per key, in registry types */
typedef struct
{
    float           rotation[3];            //!< key 0: float32[3], default [0, 0, 0]
    float           offset[3];              //!< key 1: float32[3], default [0, 0, 0]
    double          nedOrigin[3];           //!< key 2: float64[3], default set by the firmware
    uint8_t         dynamicModel;           //!< key 3: uint8, default 8
    uint8_t         ekfEnabled;             //!< key 4: bool, default true
    uint8_t         deadReckoningTimeoutS;  //!< key 5: uint8, default 30
    float           lastPositionUpdateDistanceM; //!< key 6: float32, default 1000
    uint8_t         referenceImu;           //!< key 7: bool, default false
    uint8_t         stationaryOnStrobe;     //!< key 8: bool, default false
    cbor_config_ins_fusion_t fusion;  //!< key 9: map
    cbor_config_ins_zerovelocity_t zeroVelocity;  //!< key 10: map
    cbor_config_ins_shock_t shock;  //!< key 11: map
    cbor_config_ins_faultdetect_t faultDetect;  //!< key 12: map
    cbor_config_ins_complementaryfilter_t complementaryFilter;  //!< key 13: map
} cbor_config_ins_t;

/** Schema of every CONFIG_INS key, over cbor_config_ins_t; a device stores and accepts a subset of them */
extern const cbor_schema_t g_cborSchema_CONFIG_INS;

/** The registry's recorded defaults; keys whose default the firmware sets are zero */
extern const cbor_config_ins_t g_cborDefaults_CONFIG_INS;

/** One per leaf key, depth first in key order: 1 if the registry records a fixed default, 0 if the firmware sets it */
extern const uint8_t g_cborDefaultFixed_CONFIG_INS[34];

/** Schema of the CONFIG_INS keys the IMX family stores and accepts, over cbor_config_ins_t */
extern const cbor_schema_t g_cborSchema_CONFIG_INS_imx;

#ifdef __cplusplus
}
#endif

#endif /* CBOR_DOMAIN_CONFIG_INS_H */
