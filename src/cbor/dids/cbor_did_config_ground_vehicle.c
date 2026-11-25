/**
 * @file cbor_did_config_ground_vehicle.c
 * @brief CONFIG_GROUND_VEHICLE: field names.
 *
 * Generated from the CBOR key registry by is-common scripts/cbor_registry.py -- do not edit.
 *
 * @copyright Copyright (c) 2026 Inertial Sense, Inc.
 */

#include <stddef.h>
#include "cbor/cbor_registry.h"

static const cbor_field_t s_fields_config_ground_vehicle_transform[] = {
    { 0, CBOR_T_FLOAT, 4, 3, offsetof(cbor_config_ground_vehicle_transform_t, rotation), NULL },
    { 1, CBOR_T_FLOAT, 4, 3, offsetof(cbor_config_ground_vehicle_transform_t, rotationSigma), NULL },
    { 2, CBOR_T_FLOAT, 4, 3, offsetof(cbor_config_ground_vehicle_transform_t, translation), NULL },
    { 3, CBOR_T_FLOAT, 4, 3, offsetof(cbor_config_ground_vehicle_transform_t, translationSigma), NULL },
};
static const cbor_schema_t s_schema_config_ground_vehicle_transform = { s_fields_config_ground_vehicle_transform, sizeof(s_fields_config_ground_vehicle_transform) / sizeof(s_fields_config_ground_vehicle_transform[0]) };

static const cbor_field_t s_fields_config_ground_vehicle_encoder[] = {
    { 0, CBOR_T_BOOL, 1, 0, offsetof(cbor_config_ground_vehicle_encoder_t, enabled), NULL },
    { 1, CBOR_T_UINT, 1, 0, offsetof(cbor_config_ground_vehicle_encoder_t, source), NULL },
    { 2, CBOR_T_BOOL, 1, 0, offsetof(cbor_config_ground_vehicle_encoder_t, reverseLeft), NULL },
    { 3, CBOR_T_BOOL, 1, 0, offsetof(cbor_config_ground_vehicle_encoder_t, reverseRight), NULL },
};
static const cbor_schema_t s_schema_config_ground_vehicle_encoder = { s_fields_config_ground_vehicle_encoder, sizeof(s_fields_config_ground_vehicle_encoder) / sizeof(s_fields_config_ground_vehicle_encoder[0]) };

static const cbor_field_t s_fields_config_ground_vehicle[] = {
    { 0, CBOR_T_MAP, 0, 0, offsetof(cbor_config_ground_vehicle_t, transform), &s_schema_config_ground_vehicle_transform },
    { 1, CBOR_T_FLOAT, 4, 0, offsetof(cbor_config_ground_vehicle_t, trackWidthM), NULL },
    { 2, CBOR_T_FLOAT, 4, 0, offsetof(cbor_config_ground_vehicle_t, radiusM), NULL },
    { 3, CBOR_T_BOOL, 1, 0, offsetof(cbor_config_ground_vehicle_t, controlEnabled), NULL },
    { 4, CBOR_T_MAP, 0, 0, offsetof(cbor_config_ground_vehicle_t, encoder), &s_schema_config_ground_vehicle_encoder },
};
const cbor_schema_t g_cborSchema_CONFIG_GROUND_VEHICLE = { s_fields_config_ground_vehicle, sizeof(s_fields_config_ground_vehicle) / sizeof(s_fields_config_ground_vehicle[0]) };

const cbor_config_ground_vehicle_t g_cborDefaults_CONFIG_GROUND_VEHICLE = {
    .transform.rotation = { 0, 0, 0 },
    .transform.rotationSigma = { 0.0174532924f, 0.0174532924f, 0.0174532924f },
    .transform.translation = { 0, 0, 0 },
    .transform.translationSigma = { 0.300000012f, 0.300000012f, 0.300000012f },
    .trackWidthM = 0,
    .radiusM = 0,
    .controlEnabled = 0,
    .encoder.enabled = 0,
    .encoder.source = 0,
    .encoder.reverseLeft = 0,
    .encoder.reverseRight = 0,
};

const uint8_t g_cborDefaultFixed_CONFIG_GROUND_VEHICLE[11] = {
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
};


const cbor_registry_name_t g_cborNames_CONFIG_GROUND_VEHICLE[CBOR_CONFIG_GROUND_VEHICLE_NAME_COUNT] = {
    { 0x0000, "transform" },
    { 0x0000, "rotation" },
    { 0x0001, "rotationSigma" },
    { 0x0002, "translation" },
    { 0x0003, "translationSigma" },
    { 0x0001, "trackWidthM" },
    { 0x0002, "radiusM" },
    { 0x0003, "controlEnabled" },
    { 0x0004, "encoder" },
    { 0x0000, "enabled" },
    { 0x0001, "source" },
    { 0x0002, "reverseLeft" },
    { 0x0003, "reverseRight" },
};

const uint16_t g_cborNameParents_CONFIG_GROUND_VEHICLE[CBOR_CONFIG_GROUND_VEHICLE_NAME_COUNT] = {
    0xFFFF, 0, 0, 0, 0, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 8, 8, 8, 8,
};
