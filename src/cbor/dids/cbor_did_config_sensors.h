/**
 * @file cbor_did_config_sensors.h
 * @brief CONFIG_SENSORS: CBOR-only DID, served by IMX-5, IMX-6. Field keys.
 *
 * Generated from the CBOR key registry by is-common scripts/cbor_registry.py -- do not edit.
 *
 * @copyright Copyright (c) 2026 Inertial Sense, Inc.
 */

#ifndef CBOR_DID_CONFIG_SENSORS_H
#define CBOR_DID_CONFIG_SENSORS_H

#include "cbor/cbor_registry.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @name CONFIG_SENSORS field keys */
/** @{ */
#define CBOR_CONFIG_SENSORS__ROTATION                0        //!< rotation: uint8, default 0
#define CBOR_CONFIG_SENSORS__IMUS                    1        //!< Imus: map
#define CBOR_CONFIG_SENSORS__IMUS_ENABLED            0        //!< enabled: bool, default true
#define CBOR_CONFIG_SENSORS__IMUS_GYR                1        //!< Gyr: map
#define CBOR_CONFIG_SENSORS__IMUS_GYR_PRIMARY        0        //!< PRIMARY: map
#define CBOR_CONFIG_SENSORS__IMUS_GYR_PRIMARY_FULLSCALE 0        //!< fullScale: uint8, default set by the firmware
#define CBOR_CONFIG_SENSORS__IMUS_GYR_PRIMARY_DLPF   1        //!< dlpf: uint8, default 0
#define CBOR_CONFIG_SENSORS__IMUS_GYR_HIGH_G         1        //!< HIGH_G: map
#define CBOR_CONFIG_SENSORS__IMUS_GYR_HIGH_G_FULLSCALE 0        //!< fullScale: uint8, default set by the firmware
#define CBOR_CONFIG_SENSORS__IMUS_GYR_HIGH_G_DLPF    1        //!< dlpf: uint8, default 0
#define CBOR_CONFIG_SENSORS__IMUS_ACC                2        //!< Acc: map
#define CBOR_CONFIG_SENSORS__IMUS_ACC_PRIMARY        0        //!< PRIMARY: map
#define CBOR_CONFIG_SENSORS__IMUS_ACC_PRIMARY_FULLSCALE 0        //!< fullScale: uint8, default set by the firmware
#define CBOR_CONFIG_SENSORS__IMUS_ACC_PRIMARY_DLPF   1        //!< dlpf: uint8, default 0
#define CBOR_CONFIG_SENSORS__IMUS_ACC_HIGH_G         1        //!< HIGH_G: map
#define CBOR_CONFIG_SENSORS__IMUS_ACC_HIGH_G_FULLSCALE 0        //!< fullScale: uint8, default set by the firmware
#define CBOR_CONFIG_SENSORS__IMUS_ACC_HIGH_G_DLPF    1        //!< dlpf: uint8, default 0
#define CBOR_CONFIG_SENSORS__IMU                     2        //!< Imu: map
#define CBOR_CONFIG_SENSORS__IMU_IMU1                0        //!< IMU1: map
#define CBOR_CONFIG_SENSORS__IMU_IMU1_ENABLED        0        //!< enabled: bool, default true
#define CBOR_CONFIG_SENSORS__IMU_IMU1_GYR            1        //!< Gyr: map
#define CBOR_CONFIG_SENSORS__IMU_IMU1_GYR_PRIMARY    0        //!< PRIMARY: map
#define CBOR_CONFIG_SENSORS__IMU_IMU1_GYR_PRIMARY_FULLSCALE 0        //!< fullScale: uint8, default set by the firmware
#define CBOR_CONFIG_SENSORS__IMU_IMU1_GYR_PRIMARY_DLPF 1        //!< dlpf: uint8, default 0
#define CBOR_CONFIG_SENSORS__IMU_IMU1_GYR_HIGH_G     1        //!< HIGH_G: map
#define CBOR_CONFIG_SENSORS__IMU_IMU1_GYR_HIGH_G_FULLSCALE 0        //!< fullScale: uint8, default set by the firmware
#define CBOR_CONFIG_SENSORS__IMU_IMU1_GYR_HIGH_G_DLPF 1        //!< dlpf: uint8, default 0
#define CBOR_CONFIG_SENSORS__IMU_IMU1_ACC            2        //!< Acc: map
#define CBOR_CONFIG_SENSORS__IMU_IMU1_ACC_PRIMARY    0        //!< PRIMARY: map
#define CBOR_CONFIG_SENSORS__IMU_IMU1_ACC_PRIMARY_FULLSCALE 0        //!< fullScale: uint8, default set by the firmware
#define CBOR_CONFIG_SENSORS__IMU_IMU1_ACC_PRIMARY_DLPF 1        //!< dlpf: uint8, default 0
#define CBOR_CONFIG_SENSORS__IMU_IMU1_ACC_HIGH_G     1        //!< HIGH_G: map
#define CBOR_CONFIG_SENSORS__IMU_IMU1_ACC_HIGH_G_FULLSCALE 0        //!< fullScale: uint8, default set by the firmware
#define CBOR_CONFIG_SENSORS__IMU_IMU1_ACC_HIGH_G_DLPF 1        //!< dlpf: uint8, default 0
#define CBOR_CONFIG_SENSORS__IMU_IMU2                1        //!< IMU2: map
#define CBOR_CONFIG_SENSORS__IMU_IMU2_ENABLED        0        //!< enabled: bool, default true
#define CBOR_CONFIG_SENSORS__IMU_IMU2_GYR            1        //!< Gyr: map
#define CBOR_CONFIG_SENSORS__IMU_IMU2_GYR_PRIMARY    0        //!< PRIMARY: map
#define CBOR_CONFIG_SENSORS__IMU_IMU2_GYR_PRIMARY_FULLSCALE 0        //!< fullScale: uint8, default set by the firmware
#define CBOR_CONFIG_SENSORS__IMU_IMU2_GYR_PRIMARY_DLPF 1        //!< dlpf: uint8, default 0
#define CBOR_CONFIG_SENSORS__IMU_IMU2_GYR_HIGH_G     1        //!< HIGH_G: map
#define CBOR_CONFIG_SENSORS__IMU_IMU2_GYR_HIGH_G_FULLSCALE 0        //!< fullScale: uint8, default set by the firmware
#define CBOR_CONFIG_SENSORS__IMU_IMU2_GYR_HIGH_G_DLPF 1        //!< dlpf: uint8, default 0
#define CBOR_CONFIG_SENSORS__IMU_IMU2_ACC            2        //!< Acc: map
#define CBOR_CONFIG_SENSORS__IMU_IMU2_ACC_PRIMARY    0        //!< PRIMARY: map
#define CBOR_CONFIG_SENSORS__IMU_IMU2_ACC_PRIMARY_FULLSCALE 0        //!< fullScale: uint8, default set by the firmware
#define CBOR_CONFIG_SENSORS__IMU_IMU2_ACC_PRIMARY_DLPF 1        //!< dlpf: uint8, default 0
#define CBOR_CONFIG_SENSORS__IMU_IMU2_ACC_HIGH_G     1        //!< HIGH_G: map
#define CBOR_CONFIG_SENSORS__IMU_IMU2_ACC_HIGH_G_FULLSCALE 0        //!< fullScale: uint8, default set by the firmware
#define CBOR_CONFIG_SENSORS__IMU_IMU2_ACC_HIGH_G_DLPF 1        //!< dlpf: uint8, default 0
#define CBOR_CONFIG_SENSORS__IMU_IMU3                2        //!< IMU3: map
#define CBOR_CONFIG_SENSORS__IMU_IMU3_ENABLED        0        //!< enabled: bool, default true
#define CBOR_CONFIG_SENSORS__IMU_IMU3_GYR            1        //!< Gyr: map
#define CBOR_CONFIG_SENSORS__IMU_IMU3_GYR_PRIMARY    0        //!< PRIMARY: map
#define CBOR_CONFIG_SENSORS__IMU_IMU3_GYR_PRIMARY_FULLSCALE 0        //!< fullScale: uint8, default set by the firmware
#define CBOR_CONFIG_SENSORS__IMU_IMU3_GYR_PRIMARY_DLPF 1        //!< dlpf: uint8, default 0
#define CBOR_CONFIG_SENSORS__IMU_IMU3_GYR_HIGH_G     1        //!< HIGH_G: map
#define CBOR_CONFIG_SENSORS__IMU_IMU3_GYR_HIGH_G_FULLSCALE 0        //!< fullScale: uint8, default set by the firmware
#define CBOR_CONFIG_SENSORS__IMU_IMU3_GYR_HIGH_G_DLPF 1        //!< dlpf: uint8, default 0
#define CBOR_CONFIG_SENSORS__IMU_IMU3_ACC            2        //!< Acc: map
#define CBOR_CONFIG_SENSORS__IMU_IMU3_ACC_PRIMARY    0        //!< PRIMARY: map
#define CBOR_CONFIG_SENSORS__IMU_IMU3_ACC_PRIMARY_FULLSCALE 0        //!< fullScale: uint8, default set by the firmware
#define CBOR_CONFIG_SENSORS__IMU_IMU3_ACC_PRIMARY_DLPF 1        //!< dlpf: uint8, default 0
#define CBOR_CONFIG_SENSORS__IMU_IMU3_ACC_HIGH_G     1        //!< HIGH_G: map
#define CBOR_CONFIG_SENSORS__IMU_IMU3_ACC_HIGH_G_FULLSCALE 0        //!< fullScale: uint8, default set by the firmware
#define CBOR_CONFIG_SENSORS__IMU_IMU3_ACC_HIGH_G_DLPF 1        //!< dlpf: uint8, default 0
#define CBOR_CONFIG_SENSORS__IMU_IMU4                3        //!< IMU4: map
#define CBOR_CONFIG_SENSORS__IMU_IMU4_ENABLED        0        //!< enabled: bool, default true
#define CBOR_CONFIG_SENSORS__IMU_IMU4_GYR            1        //!< Gyr: map
#define CBOR_CONFIG_SENSORS__IMU_IMU4_GYR_PRIMARY    0        //!< PRIMARY: map
#define CBOR_CONFIG_SENSORS__IMU_IMU4_GYR_PRIMARY_FULLSCALE 0        //!< fullScale: uint8, default set by the firmware
#define CBOR_CONFIG_SENSORS__IMU_IMU4_GYR_PRIMARY_DLPF 1        //!< dlpf: uint8, default 0
#define CBOR_CONFIG_SENSORS__IMU_IMU4_GYR_HIGH_G     1        //!< HIGH_G: map
#define CBOR_CONFIG_SENSORS__IMU_IMU4_GYR_HIGH_G_FULLSCALE 0        //!< fullScale: uint8, default set by the firmware
#define CBOR_CONFIG_SENSORS__IMU_IMU4_GYR_HIGH_G_DLPF 1        //!< dlpf: uint8, default 0
#define CBOR_CONFIG_SENSORS__IMU_IMU4_ACC            2        //!< Acc: map
#define CBOR_CONFIG_SENSORS__IMU_IMU4_ACC_PRIMARY    0        //!< PRIMARY: map
#define CBOR_CONFIG_SENSORS__IMU_IMU4_ACC_PRIMARY_FULLSCALE 0        //!< fullScale: uint8, default set by the firmware
#define CBOR_CONFIG_SENSORS__IMU_IMU4_ACC_PRIMARY_DLPF 1        //!< dlpf: uint8, default 0
#define CBOR_CONFIG_SENSORS__IMU_IMU4_ACC_HIGH_G     1        //!< HIGH_G: map
#define CBOR_CONFIG_SENSORS__IMU_IMU4_ACC_HIGH_G_FULLSCALE 0        //!< fullScale: uint8, default set by the firmware
#define CBOR_CONFIG_SENSORS__IMU_IMU4_ACC_HIGH_G_DLPF 1        //!< dlpf: uint8, default 0
#define CBOR_CONFIG_SENSORS__IMU_IMU5                4        //!< IMU5: map
#define CBOR_CONFIG_SENSORS__IMU_IMU5_ENABLED        0        //!< enabled: bool, default true
#define CBOR_CONFIG_SENSORS__IMU_IMU5_GYR            1        //!< Gyr: map
#define CBOR_CONFIG_SENSORS__IMU_IMU5_GYR_PRIMARY    0        //!< PRIMARY: map
#define CBOR_CONFIG_SENSORS__IMU_IMU5_GYR_PRIMARY_FULLSCALE 0        //!< fullScale: uint8, default set by the firmware
#define CBOR_CONFIG_SENSORS__IMU_IMU5_GYR_PRIMARY_DLPF 1        //!< dlpf: uint8, default 0
#define CBOR_CONFIG_SENSORS__IMU_IMU5_GYR_HIGH_G     1        //!< HIGH_G: map
#define CBOR_CONFIG_SENSORS__IMU_IMU5_GYR_HIGH_G_FULLSCALE 0        //!< fullScale: uint8, default set by the firmware
#define CBOR_CONFIG_SENSORS__IMU_IMU5_GYR_HIGH_G_DLPF 1        //!< dlpf: uint8, default 0
#define CBOR_CONFIG_SENSORS__IMU_IMU5_ACC            2        //!< Acc: map
#define CBOR_CONFIG_SENSORS__IMU_IMU5_ACC_PRIMARY    0        //!< PRIMARY: map
#define CBOR_CONFIG_SENSORS__IMU_IMU5_ACC_PRIMARY_FULLSCALE 0        //!< fullScale: uint8, default set by the firmware
#define CBOR_CONFIG_SENSORS__IMU_IMU5_ACC_PRIMARY_DLPF 1        //!< dlpf: uint8, default 0
#define CBOR_CONFIG_SENSORS__IMU_IMU5_ACC_HIGH_G     1        //!< HIGH_G: map
#define CBOR_CONFIG_SENSORS__IMU_IMU5_ACC_HIGH_G_FULLSCALE 0        //!< fullScale: uint8, default set by the firmware
#define CBOR_CONFIG_SENSORS__IMU_IMU5_ACC_HIGH_G_DLPF 1        //!< dlpf: uint8, default 0
#define CBOR_CONFIG_SENSORS__MAGS                    3        //!< Mags: map
#define CBOR_CONFIG_SENSORS__MAGS_ENABLED            0        //!< enabled: bool, default true
#define CBOR_CONFIG_SENSORS__MAG                     4        //!< Mag: map
#define CBOR_CONFIG_SENSORS__MAG_MAG1                0        //!< MAG1: map
#define CBOR_CONFIG_SENSORS__MAG_MAG1_ENABLED        0        //!< enabled: bool, default true
#define CBOR_CONFIG_SENSORS__BAROS                   5        //!< Baros: map
#define CBOR_CONFIG_SENSORS__BAROS_ENABLED           0        //!< enabled: bool, default true
#define CBOR_CONFIG_SENSORS__BARO                    6        //!< Baro: map
#define CBOR_CONFIG_SENSORS__BARO_BARO1              0        //!< BARO1: map
#define CBOR_CONFIG_SENSORS__BARO_BARO1_ENABLED      0        //!< enabled: bool, default true
/** @} */

#define CBOR_CONFIG_SENSORS_NAME_COUNT               108      //!< entries in g_cborNames_CONFIG_SENSORS

/** CONFIG_SENSORS field names, in registry order */
extern const cbor_registry_name_t g_cborNames_CONFIG_SENSORS[CBOR_CONFIG_SENSORS_NAME_COUNT];

/** CONFIG_SENSORS: one member per key, in registry types */
typedef struct
{
    uint8_t         rotation;               //!< key 0: uint8, default 0
    cbor_keyset_imu_t Imus;  //!< key 1: imu, common to every instance
    cbor_keyset_imu_t Imu[5];  //!< key 2: map of imu by imu_slot
    cbor_keyset_aux_t Mags;  //!< key 3: aux, common to every instance
    cbor_keyset_aux_t Mag[1];  //!< key 4: map of aux by mag_id
    cbor_keyset_aux_t Baros;  //!< key 5: aux, common to every instance
    cbor_keyset_aux_t Baro[1];  //!< key 6: map of aux by baro_id
} cbor_config_sensors_t;

/** Schema of every CONFIG_SENSORS key, over cbor_config_sensors_t; a device stores and accepts a subset of them */
extern const cbor_schema_t g_cborSchema_CONFIG_SENSORS;

/** The registry's recorded defaults; keys whose default the firmware sets are zero */
extern const cbor_config_sensors_t g_cborDefaults_CONFIG_SENSORS;

/** One per leaf key, depth first in key order: 1 if the registry records a fixed default, 0 if the firmware sets it */
extern const uint8_t g_cborDefaultFixed_CONFIG_SENSORS[59];

#ifdef __cplusplus
}
#endif

#endif /* CBOR_DID_CONFIG_SENSORS_H */
