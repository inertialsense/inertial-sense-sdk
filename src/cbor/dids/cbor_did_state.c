/**
 * @file cbor_did_state.c
 * @brief STATE: field names.
 *
 * Generated from the CBOR key registry by is-common scripts/cbor_registry.py -- do not edit.
 *
 * @copyright Copyright (c) 2026 Inertial Sense, Inc.
 */

#include <stddef.h>
#include "cbor/cbor_registry.h"

static const cbor_field_t s_fields_state_lastposition[] = {
    { 0, CBOR_T_FLOAT, 8, 3, offsetof(cbor_state_lastposition_t, lla), NULL },
    { 1, CBOR_T_UINT, 4, 0, offsetof(cbor_state_lastposition_t, timeOfWeekMs), NULL },
    { 2, CBOR_T_UINT, 4, 0, offsetof(cbor_state_lastposition_t, week), NULL },
};
static const cbor_schema_t s_schema_state_lastposition = { s_fields_state_lastposition, sizeof(s_fields_state_lastposition) / sizeof(s_fields_state_lastposition[0]) };

static const cbor_field_t s_fields_state_mag_mag1_coverage[] = {
    { 0, CBOR_T_UINT, 1, 0, offsetof(cbor_keyset_mag_state_coverage_t, grid), NULL },
    { 1, CBOR_T_UINT, 1, 64, offsetof(cbor_keyset_mag_state_coverage_t, counts), NULL },
    { 2, CBOR_T_UINT, 4, 0, offsetof(cbor_keyset_mag_state_coverage_t, sampleCount), NULL },
};
static const cbor_schema_t s_schema_state_mag_mag1_coverage = { s_fields_state_mag_mag1_coverage, sizeof(s_fields_state_mag_mag1_coverage) / sizeof(s_fields_state_mag_mag1_coverage[0]) };

static const cbor_field_t s_fields_state_mag_mag1_solver[] = {
    { 0, CBOR_T_FLOAT, 4, 100, offsetof(cbor_keyset_mag_state_solver_t, dtd), NULL },
};
static const cbor_schema_t s_schema_state_mag_mag1_solver = { s_fields_state_mag_mag1_solver, sizeof(s_fields_state_mag_mag1_solver) / sizeof(s_fields_state_mag_mag1_solver[0]) };

static const cbor_field_t s_fields_state_mag_mag1[] = {
    { 0, CBOR_T_MAP, 0, 0, offsetof(cbor_keyset_mag_state_t, coverage), &s_schema_state_mag_mag1_coverage },
    { 1, CBOR_T_MAP, 0, 0, offsetof(cbor_keyset_mag_state_t, solver), &s_schema_state_mag_mag1_solver },
};
static const cbor_schema_t s_schema_state_mag_mag1 = { s_fields_state_mag_mag1, sizeof(s_fields_state_mag_mag1) / sizeof(s_fields_state_mag_mag1[0]) };

static const cbor_field_t s_fields_state_mag[] = {
    { 0, CBOR_T_MAP, 0, 0, CBOR_MAG_ID_MAG1 * sizeof(cbor_keyset_mag_state_t), &s_schema_state_mag_mag1 },
};
static const cbor_schema_t s_schema_state_mag = { s_fields_state_mag, sizeof(s_fields_state_mag) / sizeof(s_fields_state_mag[0]) };

static const cbor_field_t s_fields_state[] = {
    { 0, CBOR_T_MAP, 0, 0, offsetof(cbor_state_t, lastPosition), &s_schema_state_lastposition },
    { 1, CBOR_T_MAP, 0, 0, offsetof(cbor_state_t, Mag), &s_schema_state_mag },
};
const cbor_schema_t g_cborSchema_STATE = { s_fields_state, sizeof(s_fields_state) / sizeof(s_fields_state[0]) };

const cbor_state_t g_cborDefaults_STATE = {
    .lastPosition.lla = { 0, 0, 0 },
    .lastPosition.timeOfWeekMs = 0,
    .lastPosition.week = 0,
    .Mag[CBOR_MAG_ID_MAG1].coverage.grid = 0,
    .Mag[CBOR_MAG_ID_MAG1].coverage.sampleCount = 0,
};

const uint8_t g_cborDefaultFixed_STATE[7] = {
    1, 1, 1, 1, 1, 1, 1,
};


const cbor_registry_name_t g_cborNames_STATE[CBOR_STATE_NAME_COUNT] = {
    { 0x0000, "lastPosition" },
    { 0x0000, "lla" },
    { 0x0001, "timeOfWeekMs" },
    { 0x0002, "week" },
    { 0x0001, "Mag" },
    { 0x0000, "MAG1" },
    { 0x0000, "coverage" },
    { 0x0000, "grid" },
    { 0x0001, "counts" },
    { 0x0002, "sampleCount" },
    { 0x0001, "solver" },
    { 0x0000, "dtd" },
};
