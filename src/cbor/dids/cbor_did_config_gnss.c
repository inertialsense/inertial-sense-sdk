/**
 * @file cbor_did_config_gnss.c
 * @brief CONFIG_GNSS: field names.
 *
 * Generated from the CBOR key registry by is-common scripts/cbor_registry.py -- do not edit.
 *
 * @copyright Copyright (c) 2026 Inertial Sense, Inc.
 */

#include <stddef.h>
#include "cbor/cbor_registry.h"

static const cbor_field_t s_fields_config_gnss_timesync[] = {
    { 0, CBOR_T_UINT, 4, 0, offsetof(cbor_config_gnss_timesync_t, periodMs), NULL },
    { 1, CBOR_T_FLOAT, 4, 0, offsetof(cbor_config_gnss_timesync_t, userDelayS), NULL },
};
static const cbor_schema_t s_schema_config_gnss_timesync = { s_fields_config_gnss_timesync, sizeof(s_fields_config_gnss_timesync) / sizeof(s_fields_config_gnss_timesync[0]) };

static const cbor_field_t s_fields_config_gnss_externalaiding[] = {
    { 0, CBOR_T_BOOL, 1, 0, offsetof(cbor_config_gnss_externalaiding_t, fixedLatency), NULL },
};
static const cbor_schema_t s_schema_config_gnss_externalaiding = { s_fields_config_gnss_externalaiding, sizeof(s_fields_config_gnss_externalaiding) / sizeof(s_fields_config_gnss_externalaiding[0]) };

static const cbor_field_t s_fields_config_gnss_receivers[] = {
    { 0, CBOR_T_UINT, 1, 0, offsetof(cbor_keyset_receiver_t, type), NULL },
    { 1, CBOR_T_UINT, 1, 0, offsetof(cbor_keyset_receiver_t, port), NULL },
    { 2, CBOR_T_UINT, 1, 0, offsetof(cbor_keyset_receiver_t, ppsPin), NULL },
    { 3, CBOR_T_BOOL, 1, 0, offsetof(cbor_keyset_receiver_t, configure), NULL },
    { 4, CBOR_T_BOOL, 1, 0, offsetof(cbor_keyset_receiver_t, fusionEnabled), NULL },
    { 5, CBOR_T_UINT, 2, 0, offsetof(cbor_keyset_receiver_t, signals), NULL },
    { 6, CBOR_T_FLOAT, 4, 0, offsetof(cbor_keyset_receiver_t, minElevation), NULL },
    { 7, CBOR_T_UINT, 1, 0, offsetof(cbor_keyset_receiver_t, cn0Min), NULL },
    { 8, CBOR_T_UINT, 1, 0, offsetof(cbor_keyset_receiver_t, cn0DynOffset), NULL },
    { 9, CBOR_T_UINT, 1, 0, offsetof(cbor_keyset_receiver_t, dynamicModel), NULL },
};
static const cbor_schema_t s_schema_config_gnss_receivers = { s_fields_config_gnss_receivers, sizeof(s_fields_config_gnss_receivers) / sizeof(s_fields_config_gnss_receivers[0]) };

static const cbor_field_t s_fields_config_gnss_receiver_gnss1[] = {
    { 0, CBOR_T_UINT, 1, 0, offsetof(cbor_keyset_receiver_t, type), NULL },
    { 1, CBOR_T_UINT, 1, 0, offsetof(cbor_keyset_receiver_t, port), NULL },
    { 2, CBOR_T_UINT, 1, 0, offsetof(cbor_keyset_receiver_t, ppsPin), NULL },
    { 3, CBOR_T_BOOL, 1, 0, offsetof(cbor_keyset_receiver_t, configure), NULL },
    { 4, CBOR_T_BOOL, 1, 0, offsetof(cbor_keyset_receiver_t, fusionEnabled), NULL },
    { 5, CBOR_T_UINT, 2, 0, offsetof(cbor_keyset_receiver_t, signals), NULL },
    { 6, CBOR_T_FLOAT, 4, 0, offsetof(cbor_keyset_receiver_t, minElevation), NULL },
    { 7, CBOR_T_UINT, 1, 0, offsetof(cbor_keyset_receiver_t, cn0Min), NULL },
    { 8, CBOR_T_UINT, 1, 0, offsetof(cbor_keyset_receiver_t, cn0DynOffset), NULL },
    { 9, CBOR_T_UINT, 1, 0, offsetof(cbor_keyset_receiver_t, dynamicModel), NULL },
};
static const cbor_schema_t s_schema_config_gnss_receiver_gnss1 = { s_fields_config_gnss_receiver_gnss1, sizeof(s_fields_config_gnss_receiver_gnss1) / sizeof(s_fields_config_gnss_receiver_gnss1[0]) };

static const cbor_field_t s_fields_config_gnss_receiver_gnss2[] = {
    { 0, CBOR_T_UINT, 1, 0, offsetof(cbor_keyset_receiver_t, type), NULL },
    { 1, CBOR_T_UINT, 1, 0, offsetof(cbor_keyset_receiver_t, port), NULL },
    { 2, CBOR_T_UINT, 1, 0, offsetof(cbor_keyset_receiver_t, ppsPin), NULL },
    { 3, CBOR_T_BOOL, 1, 0, offsetof(cbor_keyset_receiver_t, configure), NULL },
    { 4, CBOR_T_BOOL, 1, 0, offsetof(cbor_keyset_receiver_t, fusionEnabled), NULL },
    { 5, CBOR_T_UINT, 2, 0, offsetof(cbor_keyset_receiver_t, signals), NULL },
    { 6, CBOR_T_FLOAT, 4, 0, offsetof(cbor_keyset_receiver_t, minElevation), NULL },
    { 7, CBOR_T_UINT, 1, 0, offsetof(cbor_keyset_receiver_t, cn0Min), NULL },
    { 8, CBOR_T_UINT, 1, 0, offsetof(cbor_keyset_receiver_t, cn0DynOffset), NULL },
    { 9, CBOR_T_UINT, 1, 0, offsetof(cbor_keyset_receiver_t, dynamicModel), NULL },
};
static const cbor_schema_t s_schema_config_gnss_receiver_gnss2 = { s_fields_config_gnss_receiver_gnss2, sizeof(s_fields_config_gnss_receiver_gnss2) / sizeof(s_fields_config_gnss_receiver_gnss2[0]) };

static const cbor_field_t s_fields_config_gnss_receiver[] = {
    { 0, CBOR_T_MAP, 0, 0, CBOR_RECEIVER_ID_GNSS1 * sizeof(cbor_keyset_receiver_t), &s_schema_config_gnss_receiver_gnss1 },
    { 1, CBOR_T_MAP, 0, 0, CBOR_RECEIVER_ID_GNSS2 * sizeof(cbor_keyset_receiver_t), &s_schema_config_gnss_receiver_gnss2 },
};
static const cbor_schema_t s_schema_config_gnss_receiver = { s_fields_config_gnss_receiver, sizeof(s_fields_config_gnss_receiver) / sizeof(s_fields_config_gnss_receiver[0]) };

static const cbor_field_t s_fields_config_gnss_antennas[] = {
    { 0, CBOR_T_UINT, 1, 0, offsetof(cbor_keyset_antenna_t, receiver), NULL },
    { 1, CBOR_T_FLOAT, 4, 3, offsetof(cbor_keyset_antenna_t, offset), NULL },
    { 2, CBOR_T_BOOL, 1, 0, offsetof(cbor_keyset_antenna_t, offsetEstimation), NULL },
    { 3, CBOR_T_BOOL, 1, 0, offsetof(cbor_keyset_antenna_t, powerEnabled), NULL },
};
static const cbor_schema_t s_schema_config_gnss_antennas = { s_fields_config_gnss_antennas, sizeof(s_fields_config_gnss_antennas) / sizeof(s_fields_config_gnss_antennas[0]) };

static const cbor_field_t s_fields_config_gnss_antenna_ant1[] = {
    { 0, CBOR_T_UINT, 1, 0, offsetof(cbor_keyset_antenna_t, receiver), NULL },
    { 1, CBOR_T_FLOAT, 4, 3, offsetof(cbor_keyset_antenna_t, offset), NULL },
    { 2, CBOR_T_BOOL, 1, 0, offsetof(cbor_keyset_antenna_t, offsetEstimation), NULL },
    { 3, CBOR_T_BOOL, 1, 0, offsetof(cbor_keyset_antenna_t, powerEnabled), NULL },
};
static const cbor_schema_t s_schema_config_gnss_antenna_ant1 = { s_fields_config_gnss_antenna_ant1, sizeof(s_fields_config_gnss_antenna_ant1) / sizeof(s_fields_config_gnss_antenna_ant1[0]) };

static const cbor_field_t s_fields_config_gnss_antenna_ant2[] = {
    { 0, CBOR_T_UINT, 1, 0, offsetof(cbor_keyset_antenna_t, receiver), NULL },
    { 1, CBOR_T_FLOAT, 4, 3, offsetof(cbor_keyset_antenna_t, offset), NULL },
    { 2, CBOR_T_BOOL, 1, 0, offsetof(cbor_keyset_antenna_t, offsetEstimation), NULL },
    { 3, CBOR_T_BOOL, 1, 0, offsetof(cbor_keyset_antenna_t, powerEnabled), NULL },
};
static const cbor_schema_t s_schema_config_gnss_antenna_ant2 = { s_fields_config_gnss_antenna_ant2, sizeof(s_fields_config_gnss_antenna_ant2) / sizeof(s_fields_config_gnss_antenna_ant2[0]) };

static const cbor_field_t s_fields_config_gnss_antenna[] = {
    { 0, CBOR_T_MAP, 0, 0, CBOR_ANTENNA_ID_ANT1 * sizeof(cbor_keyset_antenna_t), &s_schema_config_gnss_antenna_ant1 },
    { 1, CBOR_T_MAP, 0, 0, CBOR_ANTENNA_ID_ANT2 * sizeof(cbor_keyset_antenna_t), &s_schema_config_gnss_antenna_ant2 },
};
static const cbor_schema_t s_schema_config_gnss_antenna = { s_fields_config_gnss_antenna, sizeof(s_fields_config_gnss_antenna) / sizeof(s_fields_config_gnss_antenna[0]) };

static const cbor_field_t s_fields_config_gnss_rtk_baseoutput_gnss1_ublox[] = {
    { 0, CBOR_T_BOOL, 1, 0, offsetof(cbor_keyset_base_output_t, ser0), NULL },
    { 1, CBOR_T_BOOL, 1, 0, offsetof(cbor_keyset_base_output_t, ser1), NULL },
    { 2, CBOR_T_BOOL, 1, 0, offsetof(cbor_keyset_base_output_t, ser2), NULL },
    { 3, CBOR_T_BOOL, 1, 0, offsetof(cbor_keyset_base_output_t, usb), NULL },
};
static const cbor_schema_t s_schema_config_gnss_rtk_baseoutput_gnss1_ublox = { s_fields_config_gnss_rtk_baseoutput_gnss1_ublox, sizeof(s_fields_config_gnss_rtk_baseoutput_gnss1_ublox) / sizeof(s_fields_config_gnss_rtk_baseoutput_gnss1_ublox[0]) };

static const cbor_field_t s_fields_config_gnss_rtk_baseoutput_gnss1_rtcm3[] = {
    { 0, CBOR_T_BOOL, 1, 0, offsetof(cbor_keyset_base_output_t, ser0), NULL },
    { 1, CBOR_T_BOOL, 1, 0, offsetof(cbor_keyset_base_output_t, ser1), NULL },
    { 2, CBOR_T_BOOL, 1, 0, offsetof(cbor_keyset_base_output_t, ser2), NULL },
    { 3, CBOR_T_BOOL, 1, 0, offsetof(cbor_keyset_base_output_t, usb), NULL },
};
static const cbor_schema_t s_schema_config_gnss_rtk_baseoutput_gnss1_rtcm3 = { s_fields_config_gnss_rtk_baseoutput_gnss1_rtcm3, sizeof(s_fields_config_gnss_rtk_baseoutput_gnss1_rtcm3) / sizeof(s_fields_config_gnss_rtk_baseoutput_gnss1_rtcm3[0]) };

static const cbor_field_t s_fields_config_gnss_rtk_baseoutput_gnss2_ublox[] = {
    { 0, CBOR_T_BOOL, 1, 0, offsetof(cbor_keyset_base_output_t, ser0), NULL },
    { 1, CBOR_T_BOOL, 1, 0, offsetof(cbor_keyset_base_output_t, ser1), NULL },
    { 2, CBOR_T_BOOL, 1, 0, offsetof(cbor_keyset_base_output_t, ser2), NULL },
    { 3, CBOR_T_BOOL, 1, 0, offsetof(cbor_keyset_base_output_t, usb), NULL },
};
static const cbor_schema_t s_schema_config_gnss_rtk_baseoutput_gnss2_ublox = { s_fields_config_gnss_rtk_baseoutput_gnss2_ublox, sizeof(s_fields_config_gnss_rtk_baseoutput_gnss2_ublox) / sizeof(s_fields_config_gnss_rtk_baseoutput_gnss2_ublox[0]) };

static const cbor_field_t s_fields_config_gnss_rtk_baseoutput_gnss2_rtcm3[] = {
    { 0, CBOR_T_BOOL, 1, 0, offsetof(cbor_keyset_base_output_t, ser0), NULL },
    { 1, CBOR_T_BOOL, 1, 0, offsetof(cbor_keyset_base_output_t, ser1), NULL },
    { 2, CBOR_T_BOOL, 1, 0, offsetof(cbor_keyset_base_output_t, ser2), NULL },
    { 3, CBOR_T_BOOL, 1, 0, offsetof(cbor_keyset_base_output_t, usb), NULL },
};
static const cbor_schema_t s_schema_config_gnss_rtk_baseoutput_gnss2_rtcm3 = { s_fields_config_gnss_rtk_baseoutput_gnss2_rtcm3, sizeof(s_fields_config_gnss_rtk_baseoutput_gnss2_rtcm3) / sizeof(s_fields_config_gnss_rtk_baseoutput_gnss2_rtcm3[0]) };

static const cbor_field_t s_fields_config_gnss_rtk_baseoutput[] = {
    { 0, CBOR_T_MAP, 0, 0, CBOR_BASE_STREAM_GNSS1_UBLOX * sizeof(cbor_keyset_base_output_t), &s_schema_config_gnss_rtk_baseoutput_gnss1_ublox },
    { 1, CBOR_T_MAP, 0, 0, CBOR_BASE_STREAM_GNSS1_RTCM3 * sizeof(cbor_keyset_base_output_t), &s_schema_config_gnss_rtk_baseoutput_gnss1_rtcm3 },
    { 2, CBOR_T_MAP, 0, 0, CBOR_BASE_STREAM_GNSS2_UBLOX * sizeof(cbor_keyset_base_output_t), &s_schema_config_gnss_rtk_baseoutput_gnss2_ublox },
    { 3, CBOR_T_MAP, 0, 0, CBOR_BASE_STREAM_GNSS2_RTCM3 * sizeof(cbor_keyset_base_output_t), &s_schema_config_gnss_rtk_baseoutput_gnss2_rtcm3 },
};
static const cbor_schema_t s_schema_config_gnss_rtk_baseoutput = { s_fields_config_gnss_rtk_baseoutput, sizeof(s_fields_config_gnss_rtk_baseoutput) / sizeof(s_fields_config_gnss_rtk_baseoutput[0]) };

static const cbor_field_t s_fields_config_gnss_rtk[] = {
    { 0, CBOR_T_UINT, 1, 0, offsetof(cbor_config_gnss_rtk_t, mode), NULL },
    { 1, CBOR_T_FLOAT, 8, 3, offsetof(cbor_config_gnss_rtk_t, basePosition), NULL },
    { 2, CBOR_T_BOOL, 1, 0, offsetof(cbor_config_gnss_rtk_t, basePositionMoving), NULL },
    { 3, CBOR_T_BOOL, 1, 0, offsetof(cbor_config_gnss_rtk_t, baseIdenticalToRover), NULL },
    { 4, CBOR_T_BOOL, 1, 0, offsetof(cbor_config_gnss_rtk_t, passThrough), NULL },
    { 5, CBOR_T_BOOL, 1, 0, offsetof(cbor_config_gnss_rtk_t, multipathMitigation), NULL },
    { 6, CBOR_T_MAP, 0, 0, offsetof(cbor_config_gnss_rtk_t, BaseOutput), &s_schema_config_gnss_rtk_baseoutput },
};
static const cbor_schema_t s_schema_config_gnss_rtk = { s_fields_config_gnss_rtk, sizeof(s_fields_config_gnss_rtk) / sizeof(s_fields_config_gnss_rtk[0]) };

static const cbor_field_t s_fields_config_gnss[] = {
    { 0, CBOR_T_UINT, 4, 0, offsetof(cbor_config_gnss_t, periodMs), NULL },
    { 1, CBOR_T_UINT, 1, 0, offsetof(cbor_config_gnss_t, primaryReceiver), NULL },
    { 2, CBOR_T_MAP, 0, 0, offsetof(cbor_config_gnss_t, timeSync), &s_schema_config_gnss_timesync },
    { 3, CBOR_T_MAP, 0, 0, offsetof(cbor_config_gnss_t, externalAiding), &s_schema_config_gnss_externalaiding },
    { 4, CBOR_T_MAP, 0, 0, offsetof(cbor_config_gnss_t, Receivers), &s_schema_config_gnss_receivers },
    { 5, CBOR_T_MAP, 0, 0, offsetof(cbor_config_gnss_t, Receiver), &s_schema_config_gnss_receiver },
    { 6, CBOR_T_MAP, 0, 0, offsetof(cbor_config_gnss_t, Antennas), &s_schema_config_gnss_antennas },
    { 7, CBOR_T_MAP, 0, 0, offsetof(cbor_config_gnss_t, Antenna), &s_schema_config_gnss_antenna },
    { 8, CBOR_T_MAP, 0, 0, offsetof(cbor_config_gnss_t, Rtk), &s_schema_config_gnss_rtk },
};
const cbor_schema_t g_cborSchema_CONFIG_GNSS = { s_fields_config_gnss, sizeof(s_fields_config_gnss) / sizeof(s_fields_config_gnss[0]) };

const cbor_config_gnss_t g_cborDefaults_CONFIG_GNSS = {
    .periodMs = 200,
    .primaryReceiver = 0,
    .timeSync.periodMs = 1000,
    .timeSync.userDelayS = 0,
    .externalAiding.fixedLatency = 0,
    .Receivers.fusionEnabled = 1,
    .Receivers.signals = 5119,
    .Receivers.minElevation = 0.17453292f,
    .Receivers.cn0Min = 30,
    .Receivers.cn0DynOffset = 20,
    .Receivers.dynamicModel = 8,
    .Receiver[CBOR_RECEIVER_ID_GNSS1].fusionEnabled = 1,
    .Receiver[CBOR_RECEIVER_ID_GNSS1].signals = 5119,
    .Receiver[CBOR_RECEIVER_ID_GNSS1].minElevation = 0.17453292f,
    .Receiver[CBOR_RECEIVER_ID_GNSS1].cn0Min = 30,
    .Receiver[CBOR_RECEIVER_ID_GNSS1].cn0DynOffset = 20,
    .Receiver[CBOR_RECEIVER_ID_GNSS1].dynamicModel = 8,
    .Receiver[CBOR_RECEIVER_ID_GNSS2].fusionEnabled = 1,
    .Receiver[CBOR_RECEIVER_ID_GNSS2].signals = 5119,
    .Receiver[CBOR_RECEIVER_ID_GNSS2].minElevation = 0.17453292f,
    .Receiver[CBOR_RECEIVER_ID_GNSS2].cn0Min = 30,
    .Receiver[CBOR_RECEIVER_ID_GNSS2].cn0DynOffset = 20,
    .Receiver[CBOR_RECEIVER_ID_GNSS2].dynamicModel = 8,
    .Antennas.offset = { 0, 0, 0 },
    .Antennas.offsetEstimation = 0,
    .Antennas.powerEnabled = 1,
    .Antenna[CBOR_ANTENNA_ID_ANT1].offset = { 0, 0, 0 },
    .Antenna[CBOR_ANTENNA_ID_ANT1].offsetEstimation = 0,
    .Antenna[CBOR_ANTENNA_ID_ANT1].powerEnabled = 1,
    .Antenna[CBOR_ANTENNA_ID_ANT2].offset = { 0, 0, 0 },
    .Antenna[CBOR_ANTENNA_ID_ANT2].offsetEstimation = 0,
    .Antenna[CBOR_ANTENNA_ID_ANT2].powerEnabled = 1,
    .Rtk.mode = 0,
    .Rtk.basePosition = { 0, 0, 0 },
    .Rtk.basePositionMoving = 0,
    .Rtk.baseIdenticalToRover = 0,
    .Rtk.passThrough = 0,
    .Rtk.multipathMitigation = 0,
    .Rtk.BaseOutput[CBOR_BASE_STREAM_GNSS1_UBLOX].ser0 = 0,
    .Rtk.BaseOutput[CBOR_BASE_STREAM_GNSS1_UBLOX].ser1 = 0,
    .Rtk.BaseOutput[CBOR_BASE_STREAM_GNSS1_UBLOX].ser2 = 0,
    .Rtk.BaseOutput[CBOR_BASE_STREAM_GNSS1_UBLOX].usb = 0,
    .Rtk.BaseOutput[CBOR_BASE_STREAM_GNSS1_RTCM3].ser0 = 0,
    .Rtk.BaseOutput[CBOR_BASE_STREAM_GNSS1_RTCM3].ser1 = 0,
    .Rtk.BaseOutput[CBOR_BASE_STREAM_GNSS1_RTCM3].ser2 = 0,
    .Rtk.BaseOutput[CBOR_BASE_STREAM_GNSS1_RTCM3].usb = 0,
    .Rtk.BaseOutput[CBOR_BASE_STREAM_GNSS2_UBLOX].ser0 = 0,
    .Rtk.BaseOutput[CBOR_BASE_STREAM_GNSS2_UBLOX].ser1 = 0,
    .Rtk.BaseOutput[CBOR_BASE_STREAM_GNSS2_UBLOX].ser2 = 0,
    .Rtk.BaseOutput[CBOR_BASE_STREAM_GNSS2_UBLOX].usb = 0,
    .Rtk.BaseOutput[CBOR_BASE_STREAM_GNSS2_RTCM3].ser0 = 0,
    .Rtk.BaseOutput[CBOR_BASE_STREAM_GNSS2_RTCM3].ser1 = 0,
    .Rtk.BaseOutput[CBOR_BASE_STREAM_GNSS2_RTCM3].ser2 = 0,
    .Rtk.BaseOutput[CBOR_BASE_STREAM_GNSS2_RTCM3].usb = 0,
};

const uint8_t g_cborDefaultFixed_CONFIG_GNSS[69] = {
    1, 1, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
};


const cbor_registry_name_t g_cborNames_CONFIG_GNSS[CBOR_CONFIG_GNSS_NAME_COUNT] = {
    { 0x0000, "periodMs" },
    { 0x0001, "primaryReceiver" },
    { 0x0002, "timeSync" },
    { 0x0000, "periodMs" },
    { 0x0001, "userDelayS" },
    { 0x0003, "externalAiding" },
    { 0x0000, "fixedLatency" },
    { 0x0004, "Receivers" },
    { 0x0000, "type" },
    { 0x0001, "port" },
    { 0x0002, "ppsPin" },
    { 0x0003, "configure" },
    { 0x0004, "fusionEnabled" },
    { 0x0005, "signals" },
    { 0x0006, "minElevation" },
    { 0x0007, "cn0Min" },
    { 0x0008, "cn0DynOffset" },
    { 0x0009, "dynamicModel" },
    { 0x0005, "Receiver" },
    { 0x0000, "GNSS1" },
    { 0x0000, "type" },
    { 0x0001, "port" },
    { 0x0002, "ppsPin" },
    { 0x0003, "configure" },
    { 0x0004, "fusionEnabled" },
    { 0x0005, "signals" },
    { 0x0006, "minElevation" },
    { 0x0007, "cn0Min" },
    { 0x0008, "cn0DynOffset" },
    { 0x0009, "dynamicModel" },
    { 0x0001, "GNSS2" },
    { 0x0000, "type" },
    { 0x0001, "port" },
    { 0x0002, "ppsPin" },
    { 0x0003, "configure" },
    { 0x0004, "fusionEnabled" },
    { 0x0005, "signals" },
    { 0x0006, "minElevation" },
    { 0x0007, "cn0Min" },
    { 0x0008, "cn0DynOffset" },
    { 0x0009, "dynamicModel" },
    { 0x0006, "Antennas" },
    { 0x0000, "receiver" },
    { 0x0001, "offset" },
    { 0x0002, "offsetEstimation" },
    { 0x0003, "powerEnabled" },
    { 0x0007, "Antenna" },
    { 0x0000, "ANT1" },
    { 0x0000, "receiver" },
    { 0x0001, "offset" },
    { 0x0002, "offsetEstimation" },
    { 0x0003, "powerEnabled" },
    { 0x0001, "ANT2" },
    { 0x0000, "receiver" },
    { 0x0001, "offset" },
    { 0x0002, "offsetEstimation" },
    { 0x0003, "powerEnabled" },
    { 0x0008, "Rtk" },
    { 0x0000, "mode" },
    { 0x0001, "basePosition" },
    { 0x0002, "basePositionMoving" },
    { 0x0003, "baseIdenticalToRover" },
    { 0x0004, "passThrough" },
    { 0x0005, "multipathMitigation" },
    { 0x0006, "BaseOutput" },
    { 0x0000, "GNSS1_UBLOX" },
    { 0x0000, "ser0" },
    { 0x0001, "ser1" },
    { 0x0002, "ser2" },
    { 0x0003, "usb" },
    { 0x0001, "GNSS1_RTCM3" },
    { 0x0000, "ser0" },
    { 0x0001, "ser1" },
    { 0x0002, "ser2" },
    { 0x0003, "usb" },
    { 0x0002, "GNSS2_UBLOX" },
    { 0x0000, "ser0" },
    { 0x0001, "ser1" },
    { 0x0002, "ser2" },
    { 0x0003, "usb" },
    { 0x0003, "GNSS2_RTCM3" },
    { 0x0000, "ser0" },
    { 0x0001, "ser1" },
    { 0x0002, "ser2" },
    { 0x0003, "usb" },
};
