/**
 * @file config_system.h
 * @brief CONFIG_SYSTEM: domain, served by IMX-5, IMX-6, GPX-1. Field keys, view types, schemas and recorded defaults.
 *
 * Generated from the CBOR key registry by is-common scripts/cbor_registry.py -- do not edit.
 *
 * @copyright Copyright (c) 2026 Inertial Sense, Inc.
 */

#ifndef CBOR_DOMAIN_CONFIG_SYSTEM_H
#define CBOR_DOMAIN_CONFIG_SYSTEM_H

#include "cbor/cbor_registry.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @name CONFIG_SYSTEM field keys */
/** @{ */
#define CBOR_CONFIG_SYSTEM__NAVPERIODMS              0        //!< navPeriodMs: uint32, default set by the firmware
#define CBOR_CONFIG_SYSTEM__IMUODR                   1        //!< imuOdr: uint32, default 1
#define CBOR_CONFIG_SYSTEM__LEDSENABLED              2        //!< ledsEnabled: bool, default true
#define CBOR_CONFIG_SYSTEM__BROWNOUTLEVEL            3        //!< brownOutLevel: uint8, default 0
#define CBOR_CONFIG_SYSTEM__BITONSTARTUP             4        //!< bitOnStartup: bool, default true
#define CBOR_CONFIG_SYSTEM__PLATFORM                 5        //!< platform: map
#define CBOR_CONFIG_SYSTEM__PLATFORM_TYPE            0        //!< type: uint8, default set by the firmware
#define CBOR_CONFIG_SYSTEM__PLATFORM_PRESET          1        //!< preset: uint8, default 0
#define CBOR_CONFIG_SYSTEM__PLATFORM_IOEXPANDERBITS  2        //!< ioExpanderBits: uint8, default 0
#define CBOR_CONFIG_SYSTEM__DEBUG                    6        //!< debug: uint8, default set by the firmware
/** @} */

#define CBOR_CONFIG_SYSTEM_NAME_COUNT                10       //!< entries in g_cborNames_CONFIG_SYSTEM

/** CONFIG_SYSTEM field names, in registry order */
extern const cbor_registry_name_t g_cborNames_CONFIG_SYSTEM[CBOR_CONFIG_SYSTEM_NAME_COUNT];

/** The parent of each g_cborNames_CONFIG_SYSTEM entry: the index of its enclosing map's entry, or 0xFFFF */
extern const uint16_t g_cborNameParents_CONFIG_SYSTEM[CBOR_CONFIG_SYSTEM_NAME_COUNT];

/** CONFIG_SYSTEM.platform: one member per key, in registry types */
typedef struct
{
    uint8_t         type;                   //!< key 0: uint8, default set by the firmware
    uint8_t         preset;                 //!< key 1: uint8, default 0
    uint8_t         ioExpanderBits;         //!< key 2: uint8, default 0
} cbor_config_system_platform_t;

/** CONFIG_SYSTEM: one member per key, in registry types */
typedef struct
{
    uint32_t        navPeriodMs;            //!< key 0: uint32, default set by the firmware
    uint32_t        imuOdr;                 //!< key 1: uint32, default 1
    uint8_t         ledsEnabled;            //!< key 2: bool, default true
    uint8_t         brownOutLevel;          //!< key 3: uint8, default 0
    uint8_t         bitOnStartup;           //!< key 4: bool, default true
    cbor_config_system_platform_t platform;  //!< key 5: map
    uint8_t         debug;                  //!< key 6: uint8, default set by the firmware
} cbor_config_system_t;

/** Schema of every CONFIG_SYSTEM key, over cbor_config_system_t; a device stores and accepts a subset of them */
extern const cbor_schema_t g_cborSchema_CONFIG_SYSTEM;

/** The registry's recorded defaults; keys whose default the firmware sets are zero */
extern const cbor_config_system_t g_cborDefaults_CONFIG_SYSTEM;

/** One per leaf key, depth first in key order: 1 if the registry records a fixed default, 0 if the firmware sets it */
extern const uint8_t g_cborDefaultFixed_CONFIG_SYSTEM[9];

/** Schema of the CONFIG_SYSTEM keys the IMX family stores and accepts, over cbor_config_system_t */
extern const cbor_schema_t g_cborSchema_CONFIG_SYSTEM_imx;

/** Schema of the CONFIG_SYSTEM keys the GPX family stores and accepts, over cbor_config_system_t */
extern const cbor_schema_t g_cborSchema_CONFIG_SYSTEM_gpx;

#ifdef __cplusplus
}
#endif

#endif /* CBOR_DOMAIN_CONFIG_SYSTEM_H */
