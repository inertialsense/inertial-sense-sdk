/* Generated from src/cbor/registry.yaml by scripts/cbor_registry.py -- do not edit. */

#include <string.h>
#include "cbor/cbor_registry.h"

typedef struct { uint16_t did; uint16_t key; const char *name; } cbor_registry_name_t;

static const cbor_registry_name_t s_names[] = {
    { 0x000C, 0x0000, "startupImuDtMs" },
    { 0x000C, 0x0001, "startupNavDtMs" },
    { 0x000C, 0x0002, "ser0BaudRate" },
    { 0x000C, 0x0003, "ser1BaudRate" },
    { 0x000C, 0x0004, "insRotation" },
    { 0x000C, 0x0005, "insOffset" },
    { 0x000C, 0x0006, "gnss1AntOffset" },
    { 0x000C, 0x0007, "dynamicModel" },
    { 0x000C, 0x0008, "debug" },
    { 0x000C, 0x0009, "gnssSatSigConst" },
    { 0x000C, 0x000A, "sysCfgBits" },
    { 0x000C, 0x000B, "refLla" },
    { 0x000C, 0x000C, "lastLla" },
    { 0x000C, 0x000D, "lastLlaTimeOfWeekMs" },
    { 0x000C, 0x000E, "lastLlaWeek" },
    { 0x000C, 0x000F, "lastLlaUpdateDistance" },
    { 0x000C, 0x0010, "ioConfig" },
    { 0x000C, 0x0011, "platformConfig" },
    { 0x000C, 0x0012, "gnss2AntOffset" },
    { 0x000C, 0x0013, "zeroVelRotation" },
    { 0x000C, 0x0014, "zeroVelOffset" },
    { 0x000C, 0x0015, "gnssTimeUserDelay" },
    { 0x000C, 0x0016, "magDeclination" },
    { 0x000C, 0x0017, "gnssTimeSyncPeriodMs" },
    { 0x000C, 0x0018, "startupGnssDtMs" },
    { 0x000C, 0x0019, "RTKCfgBits" },
    { 0x000C, 0x001A, "sensorConfig" },
    { 0x000C, 0x001B, "gnssMinimumElevation" },
    { 0x000C, 0x001C, "ser2BaudRate" },
    { 0x000C, 0x001D, "wheelConfig.bits" },
    { 0x000C, 0x001E, "wheelConfig.transform.e_b2w" },
    { 0x000C, 0x001F, "wheelConfig.transform.e_b2w_sigma" },
    { 0x000C, 0x0020, "wheelConfig.transform.t_b2w" },
    { 0x000C, 0x0021, "wheelConfig.transform.t_b2w_sigma" },
    { 0x000C, 0x0022, "wheelConfig.track_width" },
    { 0x000C, 0x0023, "wheelConfig.radius" },
    { 0x000C, 0x0024, "magInterferenceThreshold" },
    { 0x000C, 0x0025, "magCalibrationQualityThreshold" },
    { 0x000C, 0x0026, "gnssCn0Minimum" },
    { 0x000C, 0x0027, "gnssCn0DynMinOffset" },
    { 0x000C, 0x0028, "imuRejectThreshGyroLow" },
    { 0x000C, 0x0029, "imuRejectThreshGyroHigh" },
    { 0x000C, 0x002A, "imuShockDetectLatencyMs" },
    { 0x000C, 0x002B, "imuShockRejectLatchMs" },
    { 0x000C, 0x002C, "imuShockOptions" },
    { 0x000C, 0x002D, "imuShockDeltaAccHighThreshold" },
    { 0x000C, 0x002E, "imuShockDeltaAccLowThreshold" },
    { 0x000C, 0x002F, "imuShockDeltaGyroHighThreshold" },
    { 0x000C, 0x0030, "imuShockDeltaGyroLowThreshold" },
    { 0x000C, 0x0031, "ioConfig2" },
    { 0x005A, 0x0000, "can_period_mult" },
    { 0x005A, 0x0001, "can_transmit_address" },
    { 0x005A, 0x0002, "can_setting" },
    { 0x005A, 0x0003, "can_receive_address" },
    { 0x0079, 0x0000, "ser0BaudRate" },
    { 0x0079, 0x0001, "ser1BaudRate" },
    { 0x0079, 0x0002, "ser2BaudRate" },
    { 0x0079, 0x0003, "startupGnssDtMs" },
    { 0x0079, 0x0004, "gnss1AntOffset" },
    { 0x0079, 0x0005, "gnss2AntOffset" },
    { 0x0079, 0x0006, "gnssSatSigConst" },
    { 0x0079, 0x0007, "dynamicModel" },
    { 0x0079, 0x0008, "debug" },
    { 0x0079, 0x0009, "gnssTimeSyncPeriodMs" },
    { 0x0079, 0x000A, "gnssTimeUserDelay" },
    { 0x0079, 0x000B, "gnssMinimumElevation" },
    { 0x0079, 0x000C, "RTKCfgBits" },
    { 0x0079, 0x000D, "gnssCn0Minimum" },
    { 0x0079, 0x000E, "gnssCn0DynMinOffset" },
    { 0x0079, 0x000F, "sysCfgBits" },
    { 0x0079, 0x0010, "refLla" },
    { 0x7FFF, 0x0000, "MATCH_DEV" },
    { 0x7FFF, 0x0001, "MATCH_MSG" },
    { 0x7FFF, 0x0002, "MATCH_DIDS" },
    { 0x7FFF, 0x0003, "ACTION" },
    { 0x7FFE, 0x8001, "RMC_PRESET" },
    { 0x7FFE, 0x8002, "NMEA_SPEED_FILTER" },
    { 0x7FFD, 0x8000, "RMC_SAVE_PORTS" },
    { 0x7FFD, 0x8001, "RMC_PRESET" },
    { 0x7FFD, 0x8002, "NMEA_SPEED_FILTER" },
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
