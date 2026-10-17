/* Generated from src/cbor/registry.yaml by scripts/cbor_registry.py -- do not edit. */

#include <string.h>
#include "cbor/cbor_registry.h"

typedef struct { uint16_t did; uint16_t key; const char *name; } cbor_registry_name_t;

static const cbor_registry_name_t s_names[] = {
    { 0x000C, 0, "startupImuDtMs" },
    { 0x000C, 1, "startupNavDtMs" },
    { 0x000C, 2, "ser0BaudRate" },
    { 0x000C, 3, "ser1BaudRate" },
    { 0x000C, 4, "insRotation" },
    { 0x000C, 5, "insOffset" },
    { 0x000C, 6, "gnss1AntOffset" },
    { 0x000C, 7, "dynamicModel" },
    { 0x000C, 8, "debug" },
    { 0x000C, 9, "gnssSatSigConst" },
    { 0x000C, 10, "sysCfgBits" },
    { 0x000C, 11, "refLla" },
    { 0x000C, 12, "lastLla" },
    { 0x000C, 13, "lastLlaTimeOfWeekMs" },
    { 0x000C, 14, "lastLlaWeek" },
    { 0x000C, 15, "lastLlaUpdateDistance" },
    { 0x000C, 16, "ioConfig" },
    { 0x000C, 17, "platformConfig" },
    { 0x000C, 18, "gnss2AntOffset" },
    { 0x000C, 19, "zeroVelRotation" },
    { 0x000C, 20, "zeroVelOffset" },
    { 0x000C, 21, "gnssTimeUserDelay" },
    { 0x000C, 22, "magDeclination" },
    { 0x000C, 23, "gnssTimeSyncPeriodMs" },
    { 0x000C, 24, "startupGnssDtMs" },
    { 0x000C, 25, "RTKCfgBits" },
    { 0x000C, 26, "sensorConfig" },
    { 0x000C, 27, "gnssMinimumElevation" },
    { 0x000C, 28, "ser2BaudRate" },
    { 0x000C, 29, "wheelConfig.bits" },
    { 0x000C, 30, "wheelConfig.transform.e_b2w" },
    { 0x000C, 31, "wheelConfig.transform.e_b2w_sigma" },
    { 0x000C, 32, "wheelConfig.transform.t_b2w" },
    { 0x000C, 33, "wheelConfig.transform.t_b2w_sigma" },
    { 0x000C, 34, "wheelConfig.track_width" },
    { 0x000C, 35, "wheelConfig.radius" },
    { 0x000C, 36, "magInterferenceThreshold" },
    { 0x000C, 37, "magCalibrationQualityThreshold" },
    { 0x000C, 38, "gnssCn0Minimum" },
    { 0x000C, 39, "gnssCn0DynMinOffset" },
    { 0x000C, 40, "imuRejectThreshGyroLow" },
    { 0x000C, 41, "imuRejectThreshGyroHigh" },
    { 0x000C, 42, "imuShockDetectLatencyMs" },
    { 0x000C, 43, "imuShockRejectLatchMs" },
    { 0x000C, 44, "imuShockOptions" },
    { 0x000C, 45, "imuShockDeltaAccHighThreshold" },
    { 0x000C, 46, "imuShockDeltaAccLowThreshold" },
    { 0x000C, 47, "imuShockDeltaGyroHighThreshold" },
    { 0x000C, 48, "imuShockDeltaGyroLowThreshold" },
    { 0x000C, 49, "ioConfig2" },
    { 0x005A, 0, "can_period_mult" },
    { 0x005A, 1, "can_transmit_address" },
    { 0x005A, 2, "can_setting" },
    { 0x005A, 3, "can_receive_address" },
    { 0x0079, 0, "ser0BaudRate" },
    { 0x0079, 1, "ser1BaudRate" },
    { 0x0079, 2, "ser2BaudRate" },
    { 0x0079, 3, "startupGnssDtMs" },
    { 0x0079, 4, "gnss1AntOffset" },
    { 0x0079, 5, "gnss2AntOffset" },
    { 0x0079, 6, "gnssSatSigConst" },
    { 0x0079, 7, "dynamicModel" },
    { 0x0079, 8, "debug" },
    { 0x0079, 9, "gnssTimeSyncPeriodMs" },
    { 0x0079, 10, "gnssTimeUserDelay" },
    { 0x0079, 11, "gnssMinimumElevation" },
    { 0x0079, 12, "RTKCfgBits" },
    { 0x0079, 13, "gnssCn0Minimum" },
    { 0x0079, 14, "gnssCn0DynMinOffset" },
    { 0x0079, 15, "sysCfgBits" },
    { 0x0079, 16, "refLla" },
    { 0x7FFF, 0, "MATCH_DEV" },
    { 0x7FFF, 1, "MATCH_MSG" },
    { 0x7FFF, 2, "MATCH_DIDS" },
    { 0x7FFF, 3, "ACTION" },
};

const char *cbor_registry_field_name(uint16_t did, uint16_t key)
{
    for (size_t i = 0; i < sizeof(s_names) / sizeof(s_names[0]); i++)
        if (s_names[i].did == did && s_names[i].key == key)
            return s_names[i].name;
    return NULL;
}

int cbor_registry_field_key(uint16_t did, const char *name)
{
    for (size_t i = 0; i < sizeof(s_names) / sizeof(s_names[0]); i++)
        if (s_names[i].did == did && strcmp(s_names[i].name, name) == 0)
            return s_names[i].key;
    return -1;
}
