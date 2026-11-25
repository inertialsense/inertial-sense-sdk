/**
 * @file config_io.c
 * @brief CONFIG_IO: schemas, recorded defaults and field names.
 *
 * Generated from the CBOR key registry by is-common scripts/cbor_registry.py -- do not edit.
 *
 * @copyright Copyright (c) 2026 Inertial Sense, Inc.
 */

#include <stddef.h>
#include "cbor/cbor_registry.h"

static const cbor_field_t s_fields_config_io_pin_g1g2[] = {
    { 0, CBOR_T_UINT, 1, 0, offsetof(cbor_keyset_pin_t, function), NULL },
};
static const cbor_schema_t s_schema_config_io_pin_g1g2 = { s_fields_config_io_pin_g1g2, sizeof(s_fields_config_io_pin_g1g2) / sizeof(s_fields_config_io_pin_g1g2[0]) };

static const cbor_field_t s_fields_config_io_pin_g5g8[] = {
    { 0, CBOR_T_UINT, 1, 0, offsetof(cbor_keyset_pin_t, function), NULL },
};
static const cbor_schema_t s_schema_config_io_pin_g5g8 = { s_fields_config_io_pin_g5g8, sizeof(s_fields_config_io_pin_g5g8) / sizeof(s_fields_config_io_pin_g5g8[0]) };

static const cbor_field_t s_fields_config_io_pin_g6g7[] = {
    { 0, CBOR_T_UINT, 1, 0, offsetof(cbor_keyset_pin_t, function), NULL },
};
static const cbor_schema_t s_schema_config_io_pin_g6g7 = { s_fields_config_io_pin_g6g7, sizeof(s_fields_config_io_pin_g6g7) / sizeof(s_fields_config_io_pin_g6g7[0]) };

static const cbor_field_t s_fields_config_io_pin_g9[] = {
    { 0, CBOR_T_UINT, 1, 0, offsetof(cbor_keyset_pin_t, function), NULL },
};
static const cbor_schema_t s_schema_config_io_pin_g9 = { s_fields_config_io_pin_g9, sizeof(s_fields_config_io_pin_g9) / sizeof(s_fields_config_io_pin_g9[0]) };

static const cbor_field_t s_fields_config_io_pin_g11[] = {
    { 0, CBOR_T_UINT, 1, 0, offsetof(cbor_keyset_pin_t, function), NULL },
};
static const cbor_schema_t s_schema_config_io_pin_g11 = { s_fields_config_io_pin_g11, sizeof(s_fields_config_io_pin_g11) / sizeof(s_fields_config_io_pin_g11[0]) };

static const cbor_field_t s_fields_config_io_pin_g12[] = {
    { 0, CBOR_T_UINT, 1, 0, offsetof(cbor_keyset_pin_t, function), NULL },
};
static const cbor_schema_t s_schema_config_io_pin_g12 = { s_fields_config_io_pin_g12, sizeof(s_fields_config_io_pin_g12) / sizeof(s_fields_config_io_pin_g12[0]) };

static const cbor_field_t s_fields_config_io_pin_g13[] = {
    { 0, CBOR_T_UINT, 1, 0, offsetof(cbor_keyset_pin_t, function), NULL },
};
static const cbor_schema_t s_schema_config_io_pin_g13 = { s_fields_config_io_pin_g13, sizeof(s_fields_config_io_pin_g13) / sizeof(s_fields_config_io_pin_g13[0]) };

static const cbor_field_t s_fields_config_io_pin_g15[] = {
    { 0, CBOR_T_UINT, 1, 0, offsetof(cbor_keyset_pin_t, function), NULL },
};
static const cbor_schema_t s_schema_config_io_pin_g15 = { s_fields_config_io_pin_g15, sizeof(s_fields_config_io_pin_g15) / sizeof(s_fields_config_io_pin_g15[0]) };

static const cbor_field_t s_fields_config_io_pin[] = {
    { 0, CBOR_T_MAP, 0, 0, CBOR_PIN_ID_G1G2 * sizeof(cbor_keyset_pin_t), &s_schema_config_io_pin_g1g2 },
    { 1, CBOR_T_MAP, 0, 0, CBOR_PIN_ID_G5G8 * sizeof(cbor_keyset_pin_t), &s_schema_config_io_pin_g5g8 },
    { 2, CBOR_T_MAP, 0, 0, CBOR_PIN_ID_G6G7 * sizeof(cbor_keyset_pin_t), &s_schema_config_io_pin_g6g7 },
    { 3, CBOR_T_MAP, 0, 0, CBOR_PIN_ID_G9 * sizeof(cbor_keyset_pin_t), &s_schema_config_io_pin_g9 },
    { 4, CBOR_T_MAP, 0, 0, CBOR_PIN_ID_G11 * sizeof(cbor_keyset_pin_t), &s_schema_config_io_pin_g11 },
    { 5, CBOR_T_MAP, 0, 0, CBOR_PIN_ID_G12 * sizeof(cbor_keyset_pin_t), &s_schema_config_io_pin_g12 },
    { 6, CBOR_T_MAP, 0, 0, CBOR_PIN_ID_G13 * sizeof(cbor_keyset_pin_t), &s_schema_config_io_pin_g13 },
    { 7, CBOR_T_MAP, 0, 0, CBOR_PIN_ID_G15 * sizeof(cbor_keyset_pin_t), &s_schema_config_io_pin_g15 },
};
static const cbor_schema_t s_schema_config_io_pin = { s_fields_config_io_pin, sizeof(s_fields_config_io_pin) / sizeof(s_fields_config_io_pin[0]) };

static const cbor_field_t s_fields_config_io[] = {
    { 0, CBOR_T_BOOL, 1, 0, offsetof(cbor_config_io_t, strobeActiveHigh), NULL },
    { 1, CBOR_T_MAP, 0, 0, offsetof(cbor_config_io_t, Pin), &s_schema_config_io_pin },
};
const cbor_schema_t g_cborSchema_CONFIG_IO = { s_fields_config_io, sizeof(s_fields_config_io) / sizeof(s_fields_config_io[0]) };

static const cbor_field_t s_fields_config_io_imx_pin_g1g2[] = {
    { 0, CBOR_T_UINT, 1, 0, offsetof(cbor_keyset_pin_t, function), NULL },
};
static const cbor_schema_t s_schema_config_io_imx_pin_g1g2 = { s_fields_config_io_imx_pin_g1g2, sizeof(s_fields_config_io_imx_pin_g1g2) / sizeof(s_fields_config_io_imx_pin_g1g2[0]) };

static const cbor_field_t s_fields_config_io_imx_pin_g5g8[] = {
    { 0, CBOR_T_UINT, 1, 0, offsetof(cbor_keyset_pin_t, function), NULL },
};
static const cbor_schema_t s_schema_config_io_imx_pin_g5g8 = { s_fields_config_io_imx_pin_g5g8, sizeof(s_fields_config_io_imx_pin_g5g8) / sizeof(s_fields_config_io_imx_pin_g5g8[0]) };

static const cbor_field_t s_fields_config_io_imx_pin_g6g7[] = {
    { 0, CBOR_T_UINT, 1, 0, offsetof(cbor_keyset_pin_t, function), NULL },
};
static const cbor_schema_t s_schema_config_io_imx_pin_g6g7 = { s_fields_config_io_imx_pin_g6g7, sizeof(s_fields_config_io_imx_pin_g6g7) / sizeof(s_fields_config_io_imx_pin_g6g7[0]) };

static const cbor_field_t s_fields_config_io_imx_pin_g9[] = {
    { 0, CBOR_T_UINT, 1, 0, offsetof(cbor_keyset_pin_t, function), NULL },
};
static const cbor_schema_t s_schema_config_io_imx_pin_g9 = { s_fields_config_io_imx_pin_g9, sizeof(s_fields_config_io_imx_pin_g9) / sizeof(s_fields_config_io_imx_pin_g9[0]) };

static const cbor_field_t s_fields_config_io_imx_pin_g11[] = {
    { 0, CBOR_T_UINT, 1, 0, offsetof(cbor_keyset_pin_t, function), NULL },
};
static const cbor_schema_t s_schema_config_io_imx_pin_g11 = { s_fields_config_io_imx_pin_g11, sizeof(s_fields_config_io_imx_pin_g11) / sizeof(s_fields_config_io_imx_pin_g11[0]) };

static const cbor_field_t s_fields_config_io_imx_pin_g12[] = {
    { 0, CBOR_T_UINT, 1, 0, offsetof(cbor_keyset_pin_t, function), NULL },
};
static const cbor_schema_t s_schema_config_io_imx_pin_g12 = { s_fields_config_io_imx_pin_g12, sizeof(s_fields_config_io_imx_pin_g12) / sizeof(s_fields_config_io_imx_pin_g12[0]) };

static const cbor_field_t s_fields_config_io_imx_pin_g13[] = {
    { 0, CBOR_T_UINT, 1, 0, offsetof(cbor_keyset_pin_t, function), NULL },
};
static const cbor_schema_t s_schema_config_io_imx_pin_g13 = { s_fields_config_io_imx_pin_g13, sizeof(s_fields_config_io_imx_pin_g13) / sizeof(s_fields_config_io_imx_pin_g13[0]) };

static const cbor_field_t s_fields_config_io_imx_pin_g15[] = {
    { 0, CBOR_T_UINT, 1, 0, offsetof(cbor_keyset_pin_t, function), NULL },
};
static const cbor_schema_t s_schema_config_io_imx_pin_g15 = { s_fields_config_io_imx_pin_g15, sizeof(s_fields_config_io_imx_pin_g15) / sizeof(s_fields_config_io_imx_pin_g15[0]) };

static const cbor_field_t s_fields_config_io_imx_pin[] = {
    { 0, CBOR_T_MAP, 0, 0, CBOR_PIN_ID_G1G2 * sizeof(cbor_keyset_pin_t), &s_schema_config_io_imx_pin_g1g2 },
    { 1, CBOR_T_MAP, 0, 0, CBOR_PIN_ID_G5G8 * sizeof(cbor_keyset_pin_t), &s_schema_config_io_imx_pin_g5g8 },
    { 2, CBOR_T_MAP, 0, 0, CBOR_PIN_ID_G6G7 * sizeof(cbor_keyset_pin_t), &s_schema_config_io_imx_pin_g6g7 },
    { 3, CBOR_T_MAP, 0, 0, CBOR_PIN_ID_G9 * sizeof(cbor_keyset_pin_t), &s_schema_config_io_imx_pin_g9 },
    { 4, CBOR_T_MAP, 0, 0, CBOR_PIN_ID_G11 * sizeof(cbor_keyset_pin_t), &s_schema_config_io_imx_pin_g11 },
    { 5, CBOR_T_MAP, 0, 0, CBOR_PIN_ID_G12 * sizeof(cbor_keyset_pin_t), &s_schema_config_io_imx_pin_g12 },
    { 6, CBOR_T_MAP, 0, 0, CBOR_PIN_ID_G13 * sizeof(cbor_keyset_pin_t), &s_schema_config_io_imx_pin_g13 },
    { 7, CBOR_T_MAP, 0, 0, CBOR_PIN_ID_G15 * sizeof(cbor_keyset_pin_t), &s_schema_config_io_imx_pin_g15 },
};
static const cbor_schema_t s_schema_config_io_imx_pin = { s_fields_config_io_imx_pin, sizeof(s_fields_config_io_imx_pin) / sizeof(s_fields_config_io_imx_pin[0]) };

static const cbor_field_t s_fields_config_io_imx[] = {
    { 0, CBOR_T_BOOL, 1, 0, offsetof(cbor_config_io_t, strobeActiveHigh), NULL },
    { 1, CBOR_T_MAP, 0, 0, offsetof(cbor_config_io_t, Pin), &s_schema_config_io_imx_pin },
};
const cbor_schema_t g_cborSchema_CONFIG_IO_imx = { s_fields_config_io_imx, sizeof(s_fields_config_io_imx) / sizeof(s_fields_config_io_imx[0]) };

const cbor_config_io_t g_cborDefaults_CONFIG_IO = { 0 };

const uint8_t g_cborDefaultFixed_CONFIG_IO[9] = {
    0, 0, 0, 0, 0, 0, 0, 0, 0,
};


const cbor_registry_name_t g_cborNames_CONFIG_IO[CBOR_CONFIG_IO_NAME_COUNT] = {
    { 0x0000, "strobeActiveHigh" },
    { 0x0001, "Pin" },
    { 0x0000, "G1G2" },
    { 0x0000, "function" },
    { 0x0001, "G5G8" },
    { 0x0000, "function" },
    { 0x0002, "G6G7" },
    { 0x0000, "function" },
    { 0x0003, "G9" },
    { 0x0000, "function" },
    { 0x0004, "G11" },
    { 0x0000, "function" },
    { 0x0005, "G12" },
    { 0x0000, "function" },
    { 0x0006, "G13" },
    { 0x0000, "function" },
    { 0x0007, "G15" },
    { 0x0000, "function" },
};

const uint16_t g_cborNameParents_CONFIG_IO[CBOR_CONFIG_IO_NAME_COUNT] = {
    0xFFFF, 0xFFFF, 1, 2, 1, 4, 1, 6, 1, 8, 1, 10, 1, 12, 1, 14,
    1, 16,
};
