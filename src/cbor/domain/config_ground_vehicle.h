/**
 * @file config_ground_vehicle.h
 * @brief CONFIG_GROUND_VEHICLE: domain, served by IMX-5, IMX-6. Field keys, view types, schemas and recorded defaults.
 *
 * Generated from the CBOR key registry by is-common scripts/cbor_registry.py -- do not edit.
 *
 * @copyright Copyright (c) 2026 Inertial Sense, Inc.
 */

#ifndef CBOR_DOMAIN_CONFIG_GROUND_VEHICLE_H
#define CBOR_DOMAIN_CONFIG_GROUND_VEHICLE_H

#include "cbor/cbor_registry.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @name CONFIG_GROUND_VEHICLE field keys */
/** @{ */
#define CBOR_CONFIG_GROUND_VEHICLE__TRANSFORM        0        //!< transform: map
#define CBOR_CONFIG_GROUND_VEHICLE__TRANSFORM_ROTATION 0        //!< rotation: float32[3], default [0, 0, 0]
#define CBOR_CONFIG_GROUND_VEHICLE__TRANSFORM_ROTATIONSIGMA 1        //!< rotationSigma: float32[3], default [0.0174532924, 0.0174532924, 0.0174532924]
#define CBOR_CONFIG_GROUND_VEHICLE__TRANSFORM_TRANSLATION 2        //!< translation: float32[3], default [0, 0, 0]
#define CBOR_CONFIG_GROUND_VEHICLE__TRANSFORM_TRANSLATIONSIGMA 3        //!< translationSigma: float32[3], default [0.300000012, 0.300000012, 0.300000012]
#define CBOR_CONFIG_GROUND_VEHICLE__TRACKWIDTHM      1        //!< trackWidthM: float32, default 0
#define CBOR_CONFIG_GROUND_VEHICLE__RADIUSM          2        //!< radiusM: float32, default 0
#define CBOR_CONFIG_GROUND_VEHICLE__CONTROLENABLED   3        //!< controlEnabled: bool, default false
#define CBOR_CONFIG_GROUND_VEHICLE__ENCODER          4        //!< encoder: map
#define CBOR_CONFIG_GROUND_VEHICLE__ENCODER_ENABLED  0        //!< enabled: bool, default false
#define CBOR_CONFIG_GROUND_VEHICLE__ENCODER_SOURCE   1        //!< source: uint8, default 0
#define CBOR_CONFIG_GROUND_VEHICLE__ENCODER_REVERSELEFT 2        //!< reverseLeft: bool, default false
#define CBOR_CONFIG_GROUND_VEHICLE__ENCODER_REVERSERIGHT 3        //!< reverseRight: bool, default false
/** @} */

#define CBOR_CONFIG_GROUND_VEHICLE_NAME_COUNT        13       //!< entries in g_cborNames_CONFIG_GROUND_VEHICLE

/** CONFIG_GROUND_VEHICLE field names, in registry order */
extern const cbor_registry_name_t g_cborNames_CONFIG_GROUND_VEHICLE[CBOR_CONFIG_GROUND_VEHICLE_NAME_COUNT];

/** The parent of each g_cborNames_CONFIG_GROUND_VEHICLE entry: the index of its enclosing map's entry, or 0xFFFF */
extern const uint16_t g_cborNameParents_CONFIG_GROUND_VEHICLE[CBOR_CONFIG_GROUND_VEHICLE_NAME_COUNT];

/** CONFIG_GROUND_VEHICLE.transform: one member per key, in registry types */
typedef struct
{
    float           rotation[3];            //!< key 0: float32[3], default [0, 0, 0]
    float           rotationSigma[3];       //!< key 1: float32[3], default [0.0174532924, 0.0174532924, 0.0174532924]
    float           translation[3];         //!< key 2: float32[3], default [0, 0, 0]
    float           translationSigma[3];    //!< key 3: float32[3], default [0.300000012, 0.300000012, 0.300000012]
} cbor_config_ground_vehicle_transform_t;

/** CONFIG_GROUND_VEHICLE.encoder: one member per key, in registry types */
typedef struct
{
    uint8_t         enabled;                //!< key 0: bool, default false
    uint8_t         source;                 //!< key 1: uint8, default 0
    uint8_t         reverseLeft;            //!< key 2: bool, default false
    uint8_t         reverseRight;           //!< key 3: bool, default false
} cbor_config_ground_vehicle_encoder_t;

/** CONFIG_GROUND_VEHICLE: one member per key, in registry types */
typedef struct
{
    cbor_config_ground_vehicle_transform_t transform;  //!< key 0: map
    float           trackWidthM;            //!< key 1: float32, default 0
    float           radiusM;                //!< key 2: float32, default 0
    uint8_t         controlEnabled;         //!< key 3: bool, default false
    cbor_config_ground_vehicle_encoder_t encoder;  //!< key 4: map
} cbor_config_ground_vehicle_t;

/** Schema of every CONFIG_GROUND_VEHICLE key, over cbor_config_ground_vehicle_t; a device stores and accepts a subset of them */
extern const cbor_schema_t g_cborSchema_CONFIG_GROUND_VEHICLE;

/** The registry's recorded defaults; keys whose default the firmware sets are zero */
extern const cbor_config_ground_vehicle_t g_cborDefaults_CONFIG_GROUND_VEHICLE;

/** One per leaf key, depth first in key order: 1 if the registry records a fixed default, 0 if the firmware sets it */
extern const uint8_t g_cborDefaultFixed_CONFIG_GROUND_VEHICLE[11];

/** Schema of the CONFIG_GROUND_VEHICLE keys the IMX family stores and accepts, over cbor_config_ground_vehicle_t */
extern const cbor_schema_t g_cborSchema_CONFIG_GROUND_VEHICLE_imx;

#ifdef __cplusplus
}
#endif

#endif /* CBOR_DOMAIN_CONFIG_GROUND_VEHICLE_H */
