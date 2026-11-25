/**
 * @file cbor_did_config_ports.h
 * @brief CONFIG_PORTS: CBOR-only DID, served by IMX-5, IMX-6, GPX-1. Field keys.
 *
 * Generated from the CBOR key registry by is-common scripts/cbor_registry.py -- do not edit.
 *
 * @copyright Copyright (c) 2026 Inertial Sense, Inc.
 */

#ifndef CBOR_DID_CONFIG_PORTS_H
#define CBOR_DID_CONFIG_PORTS_H

#include "cbor/cbor_registry.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @name CONFIG_PORTS field keys */
/** @{ */
#define CBOR_CONFIG_PORTS__PORTS                     0        //!< Ports: map
#define CBOR_CONFIG_PORTS__PORTS_BAUDRATE            0        //!< baudRate: uint32, default 921600
#define CBOR_CONFIG_PORTS__PORTS_GSVBANDS            1        //!< gsvBands: uint8[SAT_SV_GNSS_ID_COUNT], default set by the firmware
#define CBOR_CONFIG_PORTS__PORT                      1        //!< Port: map
#define CBOR_CONFIG_PORTS__PORT_SER0                 0        //!< SER0: map
#define CBOR_CONFIG_PORTS__PORT_SER0_BAUDRATE        0        //!< baudRate: uint32, default 921600
#define CBOR_CONFIG_PORTS__PORT_SER0_GSVBANDS        1        //!< gsvBands: uint8[SAT_SV_GNSS_ID_COUNT], default set by the firmware
#define CBOR_CONFIG_PORTS__PORT_SER1                 1        //!< SER1: map
#define CBOR_CONFIG_PORTS__PORT_SER1_BAUDRATE        0        //!< baudRate: uint32, default 921600
#define CBOR_CONFIG_PORTS__PORT_SER1_GSVBANDS        1        //!< gsvBands: uint8[SAT_SV_GNSS_ID_COUNT], default set by the firmware
#define CBOR_CONFIG_PORTS__PORT_SER2                 2        //!< SER2: map
#define CBOR_CONFIG_PORTS__PORT_SER2_BAUDRATE        0        //!< baudRate: uint32, default 921600
#define CBOR_CONFIG_PORTS__PORT_SER2_GSVBANDS        1        //!< gsvBands: uint8[SAT_SV_GNSS_ID_COUNT], default set by the firmware
#define CBOR_CONFIG_PORTS__PORT_USB                  3        //!< USB: map
#define CBOR_CONFIG_PORTS__PORT_USB_BAUDRATE         0        //!< baudRate: uint32, default 921600
#define CBOR_CONFIG_PORTS__PORT_USB_GSVBANDS         1        //!< gsvBands: uint8[SAT_SV_GNSS_ID_COUNT], default set by the firmware
#define CBOR_CONFIG_PORTS__PORT_SPI                  4        //!< SPI: map
#define CBOR_CONFIG_PORTS__PORT_SPI_BAUDRATE         0        //!< baudRate: uint32, default 921600
#define CBOR_CONFIG_PORTS__PORT_SPI_GSVBANDS         1        //!< gsvBands: uint8[SAT_SV_GNSS_ID_COUNT], default set by the firmware
/** @} */

#define CBOR_CONFIG_PORTS_NAME_COUNT                 19       //!< entries in g_cborNames_CONFIG_PORTS

/** CONFIG_PORTS field names, in registry order */
extern const cbor_registry_name_t g_cborNames_CONFIG_PORTS[CBOR_CONFIG_PORTS_NAME_COUNT];

/** The parent of each g_cborNames_CONFIG_PORTS entry: the index of its enclosing map's entry, or 0xFFFF */
extern const uint16_t g_cborNameParents_CONFIG_PORTS[CBOR_CONFIG_PORTS_NAME_COUNT];

/** CONFIG_PORTS: one member per key, in registry types */
typedef struct
{
    cbor_keyset_port_t Ports;  //!< key 0: port, common to every instance
    cbor_keyset_port_t Port[5];  //!< key 1: map of port by port_id
} cbor_config_ports_t;

/** Schema of every CONFIG_PORTS key, over cbor_config_ports_t; a device stores and accepts a subset of them */
extern const cbor_schema_t g_cborSchema_CONFIG_PORTS;

/** The registry's recorded defaults; keys whose default the firmware sets are zero */
extern const cbor_config_ports_t g_cborDefaults_CONFIG_PORTS;

/** One per leaf key, depth first in key order: 1 if the registry records a fixed default, 0 if the firmware sets it */
extern const uint8_t g_cborDefaultFixed_CONFIG_PORTS[12];

#ifdef __cplusplus
}
#endif

#endif /* CBOR_DID_CONFIG_PORTS_H */
