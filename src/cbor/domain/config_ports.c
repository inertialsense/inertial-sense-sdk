/**
 * @file config_ports.c
 * @brief CONFIG_PORTS: schemas, recorded defaults and field names.
 *
 * Generated from the CBOR key registry by is-common scripts/cbor_registry.py -- do not edit.
 *
 * @copyright Copyright (c) 2026 Inertial Sense, Inc.
 */

#include <stddef.h>
#include "cbor/cbor_registry.h"

static const cbor_field_t s_fields_config_ports_ports[] = {
    { 0, CBOR_T_UINT, 4, 0, offsetof(cbor_keyset_port_t, baudRate), NULL },
    { 1, CBOR_T_UINT, 1, 9, offsetof(cbor_keyset_port_t, gsvBands), NULL },
};
static const cbor_schema_t s_schema_config_ports_ports = { s_fields_config_ports_ports, sizeof(s_fields_config_ports_ports) / sizeof(s_fields_config_ports_ports[0]) };

static const cbor_field_t s_fields_config_ports_port_ser0[] = {
    { 0, CBOR_T_UINT, 4, 0, offsetof(cbor_keyset_port_t, baudRate), NULL },
    { 1, CBOR_T_UINT, 1, 9, offsetof(cbor_keyset_port_t, gsvBands), NULL },
};
static const cbor_schema_t s_schema_config_ports_port_ser0 = { s_fields_config_ports_port_ser0, sizeof(s_fields_config_ports_port_ser0) / sizeof(s_fields_config_ports_port_ser0[0]) };

static const cbor_field_t s_fields_config_ports_port_ser1[] = {
    { 0, CBOR_T_UINT, 4, 0, offsetof(cbor_keyset_port_t, baudRate), NULL },
    { 1, CBOR_T_UINT, 1, 9, offsetof(cbor_keyset_port_t, gsvBands), NULL },
};
static const cbor_schema_t s_schema_config_ports_port_ser1 = { s_fields_config_ports_port_ser1, sizeof(s_fields_config_ports_port_ser1) / sizeof(s_fields_config_ports_port_ser1[0]) };

static const cbor_field_t s_fields_config_ports_port_ser2[] = {
    { 0, CBOR_T_UINT, 4, 0, offsetof(cbor_keyset_port_t, baudRate), NULL },
    { 1, CBOR_T_UINT, 1, 9, offsetof(cbor_keyset_port_t, gsvBands), NULL },
};
static const cbor_schema_t s_schema_config_ports_port_ser2 = { s_fields_config_ports_port_ser2, sizeof(s_fields_config_ports_port_ser2) / sizeof(s_fields_config_ports_port_ser2[0]) };

static const cbor_field_t s_fields_config_ports_port_usb[] = {
    { 0, CBOR_T_UINT, 4, 0, offsetof(cbor_keyset_port_t, baudRate), NULL },
    { 1, CBOR_T_UINT, 1, 9, offsetof(cbor_keyset_port_t, gsvBands), NULL },
};
static const cbor_schema_t s_schema_config_ports_port_usb = { s_fields_config_ports_port_usb, sizeof(s_fields_config_ports_port_usb) / sizeof(s_fields_config_ports_port_usb[0]) };

static const cbor_field_t s_fields_config_ports_port_spi[] = {
    { 0, CBOR_T_UINT, 4, 0, offsetof(cbor_keyset_port_t, baudRate), NULL },
    { 1, CBOR_T_UINT, 1, 9, offsetof(cbor_keyset_port_t, gsvBands), NULL },
};
static const cbor_schema_t s_schema_config_ports_port_spi = { s_fields_config_ports_port_spi, sizeof(s_fields_config_ports_port_spi) / sizeof(s_fields_config_ports_port_spi[0]) };

static const cbor_field_t s_fields_config_ports_port[] = {
    { 0, CBOR_T_MAP, 0, 0, CBOR_PORT_ID_SER0 * sizeof(cbor_keyset_port_t), &s_schema_config_ports_port_ser0 },
    { 1, CBOR_T_MAP, 0, 0, CBOR_PORT_ID_SER1 * sizeof(cbor_keyset_port_t), &s_schema_config_ports_port_ser1 },
    { 2, CBOR_T_MAP, 0, 0, CBOR_PORT_ID_SER2 * sizeof(cbor_keyset_port_t), &s_schema_config_ports_port_ser2 },
    { 3, CBOR_T_MAP, 0, 0, CBOR_PORT_ID_USB * sizeof(cbor_keyset_port_t), &s_schema_config_ports_port_usb },
    { 4, CBOR_T_MAP, 0, 0, CBOR_PORT_ID_SPI * sizeof(cbor_keyset_port_t), &s_schema_config_ports_port_spi },
};
static const cbor_schema_t s_schema_config_ports_port = { s_fields_config_ports_port, sizeof(s_fields_config_ports_port) / sizeof(s_fields_config_ports_port[0]) };

static const cbor_field_t s_fields_config_ports[] = {
    { 0, CBOR_T_MAP, 0, 0, offsetof(cbor_config_ports_t, Ports), &s_schema_config_ports_ports },
    { 1, CBOR_T_MAP, 0, 0, offsetof(cbor_config_ports_t, Port), &s_schema_config_ports_port },
};
const cbor_schema_t g_cborSchema_CONFIG_PORTS = { s_fields_config_ports, sizeof(s_fields_config_ports) / sizeof(s_fields_config_ports[0]) };

static const cbor_field_t s_fields_config_ports_imx_ports[] = {
    { 1, CBOR_T_UINT, 1, 9, offsetof(cbor_keyset_port_t, gsvBands), NULL },
};
static const cbor_schema_t s_schema_config_ports_imx_ports = { s_fields_config_ports_imx_ports, sizeof(s_fields_config_ports_imx_ports) / sizeof(s_fields_config_ports_imx_ports[0]) };

static const cbor_field_t s_fields_config_ports_imx_port_ser0[] = {
    { 0, CBOR_T_UINT, 4, 0, offsetof(cbor_keyset_port_t, baudRate), NULL },
};
static const cbor_schema_t s_schema_config_ports_imx_port_ser0 = { s_fields_config_ports_imx_port_ser0, sizeof(s_fields_config_ports_imx_port_ser0) / sizeof(s_fields_config_ports_imx_port_ser0[0]) };

static const cbor_field_t s_fields_config_ports_imx_port_ser1[] = {
    { 0, CBOR_T_UINT, 4, 0, offsetof(cbor_keyset_port_t, baudRate), NULL },
};
static const cbor_schema_t s_schema_config_ports_imx_port_ser1 = { s_fields_config_ports_imx_port_ser1, sizeof(s_fields_config_ports_imx_port_ser1) / sizeof(s_fields_config_ports_imx_port_ser1[0]) };

static const cbor_field_t s_fields_config_ports_imx_port_ser2[] = {
    { 0, CBOR_T_UINT, 4, 0, offsetof(cbor_keyset_port_t, baudRate), NULL },
};
static const cbor_schema_t s_schema_config_ports_imx_port_ser2 = { s_fields_config_ports_imx_port_ser2, sizeof(s_fields_config_ports_imx_port_ser2) / sizeof(s_fields_config_ports_imx_port_ser2[0]) };

static const cbor_field_t s_fields_config_ports_imx_port[] = {
    { 0, CBOR_T_MAP, 0, 0, CBOR_PORT_ID_SER0 * sizeof(cbor_keyset_port_t), &s_schema_config_ports_imx_port_ser0 },
    { 1, CBOR_T_MAP, 0, 0, CBOR_PORT_ID_SER1 * sizeof(cbor_keyset_port_t), &s_schema_config_ports_imx_port_ser1 },
    { 2, CBOR_T_MAP, 0, 0, CBOR_PORT_ID_SER2 * sizeof(cbor_keyset_port_t), &s_schema_config_ports_imx_port_ser2 },
};
static const cbor_schema_t s_schema_config_ports_imx_port = { s_fields_config_ports_imx_port, sizeof(s_fields_config_ports_imx_port) / sizeof(s_fields_config_ports_imx_port[0]) };

static const cbor_field_t s_fields_config_ports_imx[] = {
    { 0, CBOR_T_MAP, 0, 0, offsetof(cbor_config_ports_t, Ports), &s_schema_config_ports_imx_ports },
    { 1, CBOR_T_MAP, 0, 0, offsetof(cbor_config_ports_t, Port), &s_schema_config_ports_imx_port },
};
const cbor_schema_t g_cborSchema_CONFIG_PORTS_imx = { s_fields_config_ports_imx, sizeof(s_fields_config_ports_imx) / sizeof(s_fields_config_ports_imx[0]) };

static const cbor_field_t s_fields_config_ports_gpx_port_ser0[] = {
    { 0, CBOR_T_UINT, 4, 0, offsetof(cbor_keyset_port_t, baudRate), NULL },
};
static const cbor_schema_t s_schema_config_ports_gpx_port_ser0 = { s_fields_config_ports_gpx_port_ser0, sizeof(s_fields_config_ports_gpx_port_ser0) / sizeof(s_fields_config_ports_gpx_port_ser0[0]) };

static const cbor_field_t s_fields_config_ports_gpx_port_ser1[] = {
    { 0, CBOR_T_UINT, 4, 0, offsetof(cbor_keyset_port_t, baudRate), NULL },
};
static const cbor_schema_t s_schema_config_ports_gpx_port_ser1 = { s_fields_config_ports_gpx_port_ser1, sizeof(s_fields_config_ports_gpx_port_ser1) / sizeof(s_fields_config_ports_gpx_port_ser1[0]) };

static const cbor_field_t s_fields_config_ports_gpx_port_ser2[] = {
    { 0, CBOR_T_UINT, 4, 0, offsetof(cbor_keyset_port_t, baudRate), NULL },
};
static const cbor_schema_t s_schema_config_ports_gpx_port_ser2 = { s_fields_config_ports_gpx_port_ser2, sizeof(s_fields_config_ports_gpx_port_ser2) / sizeof(s_fields_config_ports_gpx_port_ser2[0]) };

static const cbor_field_t s_fields_config_ports_gpx_port[] = {
    { 0, CBOR_T_MAP, 0, 0, CBOR_PORT_ID_SER0 * sizeof(cbor_keyset_port_t), &s_schema_config_ports_gpx_port_ser0 },
    { 1, CBOR_T_MAP, 0, 0, CBOR_PORT_ID_SER1 * sizeof(cbor_keyset_port_t), &s_schema_config_ports_gpx_port_ser1 },
    { 2, CBOR_T_MAP, 0, 0, CBOR_PORT_ID_SER2 * sizeof(cbor_keyset_port_t), &s_schema_config_ports_gpx_port_ser2 },
};
static const cbor_schema_t s_schema_config_ports_gpx_port = { s_fields_config_ports_gpx_port, sizeof(s_fields_config_ports_gpx_port) / sizeof(s_fields_config_ports_gpx_port[0]) };

static const cbor_field_t s_fields_config_ports_gpx[] = {
    { 1, CBOR_T_MAP, 0, 0, offsetof(cbor_config_ports_t, Port), &s_schema_config_ports_gpx_port },
};
const cbor_schema_t g_cborSchema_CONFIG_PORTS_gpx = { s_fields_config_ports_gpx, sizeof(s_fields_config_ports_gpx) / sizeof(s_fields_config_ports_gpx[0]) };

const cbor_config_ports_t g_cborDefaults_CONFIG_PORTS = {
    .Ports.baudRate = 921600,
    .Port[CBOR_PORT_ID_SER0].baudRate = 921600,
    .Port[CBOR_PORT_ID_SER1].baudRate = 921600,
    .Port[CBOR_PORT_ID_SER2].baudRate = 921600,
    .Port[CBOR_PORT_ID_USB].baudRate = 921600,
    .Port[CBOR_PORT_ID_SPI].baudRate = 921600,
};

const uint8_t g_cborDefaultFixed_CONFIG_PORTS[12] = {
    1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0,
};


const cbor_registry_name_t g_cborNames_CONFIG_PORTS[CBOR_CONFIG_PORTS_NAME_COUNT] = {
    { 0x0000, "Ports" },
    { 0x0000, "baudRate" },
    { 0x0001, "gsvBands" },
    { 0x0001, "Port" },
    { 0x0000, "SER0" },
    { 0x0000, "baudRate" },
    { 0x0001, "gsvBands" },
    { 0x0001, "SER1" },
    { 0x0000, "baudRate" },
    { 0x0001, "gsvBands" },
    { 0x0002, "SER2" },
    { 0x0000, "baudRate" },
    { 0x0001, "gsvBands" },
    { 0x0003, "USB" },
    { 0x0000, "baudRate" },
    { 0x0001, "gsvBands" },
    { 0x0004, "SPI" },
    { 0x0000, "baudRate" },
    { 0x0001, "gsvBands" },
};

const uint16_t g_cborNameParents_CONFIG_PORTS[CBOR_CONFIG_PORTS_NAME_COUNT] = {
    0xFFFF, 0, 0, 0xFFFF, 3, 4, 4, 3, 7, 7, 3, 10, 10, 3, 13, 13,
    3, 16, 16,
};
