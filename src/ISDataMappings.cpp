/*
MIT LICENSE

Copyright (c) 2014-2025 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include <map>
#include <vector>
#include <set>
#include <algorithm>
#include <stdlib.h>
#include <stddef.h>
#include <inttypes.h>
#include <stdio.h>
#include <assert.h>
#include <string.h>
#include <iostream>

#include "ISDataMappings.h"
#include "DataJSON.h"
#include "ISUtilities.h"
#include "ISConstants.h"
#include "data_sets.h"

using namespace std;

#define SYM_DEG             "°"
#define SYM_DEG_C           "°C"
#define SYM_DEG_DEG_M       "°,°,m"
#define SYM_DEG_PER_S       "°/s"
#define SYM_M_PER_S         "m/s"
#define SYM_M_PER_S_2       "m/s²"
#define SYM_DEG_C_PER_S     "°C/s"

const char s_insStatusDescription[] = "INS Status flags [0,0,MagStatus,SolStatus,     NavMode,GpsMagUsed,Variance,VarianceCoarse]";
const char s_hdwStatusDescription[] = "Hdw Status flags [Fault,BIT,RxErrCount,ComErr, SenSatHist,SensorSat,GpsSatRx,Motion]";
const char s_imuStatusDescription[] = "IMU Status flags [Sensor saturation]";

// Stringify the macro value
#define STRINGIFY(x) #x
#define TOSTRING(x) STRINGIFY(x)

const unsigned char g_asciiToLowerMap[256] =
{
    0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32,33,34,35,36,37,38,39,40,
    41,42,43,44,45,46,47,48,49,50,51,52,53,54,55,56,57,58,59,60,61,62,63,64,'a','b','c','d','e','f','g','h','i','j','k','l','m','n','o','p','q',
    'r','s','t','u','v','w','x','y','z',91,92,93,94,95,96,97,98,99,100,101,102,103,104,105,106,107,108,109,110,111,112,113,114,115,116,117,118,
    119,120,121,122,123,124,125,126,127,128,129,130,131,132,133,134,135,136,137,138,139,140,141,142,143,144,145,146,147,148,149,150,151,152,153,
    154,155,156,157,158,159,160,161,162,163,164,165,166,167,168,169,170,171,172,173,174,175,176,177,178,179,180,181,182,183,184,185,186,187,188,
    189,190,191,192,193,194,195,196,197,198,199,200,201,202,203,204,205,206,207,208,209,210,211,212,213,214,215,216,217,218,219,220,221,222,223,
    224,225,226,227,228,229,230,231,232,233,234,235,236,237,238,239,240,241,242,243,244,245,246,247,248,249,250,251,252,253,254,255
};

static void PopulateMapTimestampField(data_set_t data_set[DID_COUNT], uint32_t did)
{
    static const string timestampFields[] = { "time", "timeOfWeek", "timeOfWeekMs", "seconds" };
    const map_name_to_info_t& offsetMap = data_set[did].nameToInfo;

    if (offsetMap.size() != 0)
    {
        for (size_t i = 0; i < _ARRAY_ELEMENT_COUNT(timestampFields); i++)
        {
            map_name_to_info_t::const_iterator timestampField = offsetMap.find(timestampFields[i]);
            if (timestampField != offsetMap.end())
            {
                data_set[did].timestampFields = (const data_info_t*)&timestampField->second;
                return;
            }
        }
    }

    data_set[did].timestampFields = NULLPTR;  // ensure value is not garbage
}

static void PopulateMapDeviceInfo(data_set_t data_set[DID_COUNT], uint32_t did)
{
    DataMapper<dev_info_t> mapper(data_set, did);
    mapper.AddMember("reserved", &dev_info_t::reserved, DATA_TYPE_UINT16);
    mapper.AddMember("reserved2", &dev_info_t::reserved2, DATA_TYPE_UINT8);
    mapper.AddMember("hardwareType", &dev_info_t::hardwareType, DATA_TYPE_UINT8,  "", "Hardware type: 1=uINS, 2=EVB, 3=IMX, 4=GPX", DATA_FLAGS_READ_ONLY);
    mapper.AddMember("serialNumber", &dev_info_t::serialNumber, DATA_TYPE_UINT32, "", "Serial number", DATA_FLAGS_READ_ONLY);
    mapper.AddArray("hardwareVer", &dev_info_t::hardwareVer, DATA_TYPE_UINT8, 4, {""}, {"Hardware version"}, DATA_FLAGS_READ_ONLY);
    mapper.AddArray("firmwareVer", &dev_info_t::firmwareVer, DATA_TYPE_UINT8, 4, {""}, {"Firmware version"}, DATA_FLAGS_READ_ONLY);
    mapper.AddMember("buildNumber", &dev_info_t::buildNumber, DATA_TYPE_UINT32, "", "Build number (0xFFFFF000 = Host key, 0x00000FFF = Build #)", DATA_FLAGS_READ_ONLY | DATA_FLAGS_DISPLAY_HEX);
    mapper.AddArray("protocolVer", &dev_info_t::protocolVer, DATA_TYPE_UINT8, 4, {""}, {"Communications protocol version"}, DATA_FLAGS_READ_ONLY);
    mapper.AddMember("repoRevision", &dev_info_t::repoRevision, DATA_TYPE_UINT32, "", "Repo revision", DATA_FLAGS_READ_ONLY | DATA_FLAGS_DISPLAY_HEX);
    mapper.AddMember("manufacturer", &dev_info_t::manufacturer, DATA_TYPE_STRING, "", "manufacturer", DATA_FLAGS_READ_ONLY);
    mapper.AddMember("buildType", &dev_info_t::buildType, DATA_TYPE_UINT8, "", "'a'(97)=ALPHA, 'b'(98)=BETA, 'c'(99)=CANDIDATE, 'r'(114)=PRODUCTION, 'd'(100)=develop, 's'(115)=snapshot, '^'(94)=dirty", DATA_FLAGS_READ_ONLY);
    mapper.AddMember("buildYear", &dev_info_t::buildYear, DATA_TYPE_UINT8, "", "Build year-2000", DATA_FLAGS_READ_ONLY);
    mapper.AddMember("buildMonth", &dev_info_t::buildMonth, DATA_TYPE_UINT8, "", "Build month", DATA_FLAGS_READ_ONLY);
    mapper.AddMember("buildDay", &dev_info_t::buildDay, DATA_TYPE_UINT8, "", "Build day", DATA_FLAGS_READ_ONLY);
    mapper.AddMember("buildHour", &dev_info_t::buildHour, DATA_TYPE_UINT8, "", "Build hour", DATA_FLAGS_READ_ONLY);
    mapper.AddMember("buildMinute", &dev_info_t::buildMinute, DATA_TYPE_UINT8, "", "Build minute", DATA_FLAGS_READ_ONLY);
    mapper.AddMember("buildSecond", &dev_info_t::buildSecond, DATA_TYPE_UINT8, "", "Build second", DATA_FLAGS_READ_ONLY);
    mapper.AddMember("buildMillisecond", &dev_info_t::buildMillisecond, DATA_TYPE_UINT8, "", "Build millisecond", DATA_FLAGS_READ_ONLY);
    mapper.AddMember("addInfo", &dev_info_t::addInfo, DATA_TYPE_STRING, "", "Additional info", DATA_FLAGS_READ_ONLY);
}

static void PopulateMapManufacturingInfo(data_set_t data_set[DID_COUNT], uint32_t did)
{
    DataMapper<manufacturing_info_t> mapper(data_set, did);
    mapper.AddMember("serialNumber", &manufacturing_info_t::serialNumber, DATA_TYPE_UINT32, "", "Serial number", DATA_FLAGS_READ_ONLY);
    mapper.AddMember("hardwareId", &manufacturing_info_t::hardwareId, DATA_TYPE_UINT16, "", "Hardware Id", DATA_FLAGS_READ_ONLY | DATA_FLAGS_DISPLAY_HEX);
    mapper.AddMember("lotNumber", &manufacturing_info_t::lotNumber, DATA_TYPE_UINT16, "", "Lot number", DATA_FLAGS_READ_ONLY);
    mapper.AddMember("date", &manufacturing_info_t::date, DATA_TYPE_STRING, "", "Manufacturing date (YYYYMMDDHHMMSS)", DATA_FLAGS_READ_ONLY);
    mapper.AddMember("key", &manufacturing_info_t::key, DATA_TYPE_UINT32, "", "key (times OTP area was set, 15 max)", DATA_FLAGS_READ_ONLY);
    mapper.AddMember("platformType", &manufacturing_info_t::platformType, DATA_TYPE_INT32, "", "Platform type (carrier board)", DATA_FLAGS_READ_ONLY);
    mapper.AddMember("reserved", &manufacturing_info_t::reserved, DATA_TYPE_INT32, "", "Reserved", DATA_FLAGS_READ_ONLY);
    mapper.AddArray("uid", &manufacturing_info_t::uid, DATA_TYPE_UINT32, 4, {""}, {"Unique microcontroller identifier"}, DATA_FLAGS_READ_ONLY);
}

static void PopulateMapBit(data_set_t data_set[DID_COUNT], uint32_t did)
{
    DataMapper<bit_t> mapper(data_set, did);
    mapper.AddMember("command", &bit_t::command, DATA_TYPE_UINT8, "", "[cmd: " + std::to_string(BIT_CMD_FULL_STATIONARY) + "=start full, " + std::to_string(BIT_CMD_BASIC_MOVING) + "=start basic, " + std::to_string(BIT_CMD_FULL_STATIONARY_HIGH_ACCURACY) + "=start full HA, " + std::to_string(BIT_CMD_OFF) + "=off]");
    mapper.AddMember("lastCommand", &bit_t::lastCommand, DATA_TYPE_UINT8, "", "Last input command", DATA_FLAGS_READ_ONLY);
    mapper.AddMember("state", &bit_t::state, DATA_TYPE_UINT8, "", "[state: " + std::to_string(BIT_STATE_RUNNING) + "=running " + std::to_string(BIT_STATE_DONE) + "=done]", DATA_FLAGS_READ_ONLY);
    mapper.AddMember("reserved", &bit_t::reserved, DATA_TYPE_UINT8);
    mapper.AddMember("hdwBitStatus", &bit_t::hdwBitStatus, DATA_TYPE_UINT32, "", "Hardware built-in test status. See eHdwBitStatusFlags for info.", DATA_FLAGS_READ_ONLY | DATA_FLAGS_DISPLAY_HEX);
    mapper.AddMember("calBitStatus", &bit_t::calBitStatus, DATA_TYPE_UINT32, "", "Calibration built-in test status. See eCalBitStatusFlags for info.", DATA_FLAGS_READ_ONLY | DATA_FLAGS_DISPLAY_HEX);
    mapper.AddMember("tcPqrBias", &bit_t::tcPqrBias, DATA_TYPE_F32, SYM_DEG_PER_S, "Gyro temp cal bias", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_4, C_RAD2DEG);
    mapper.AddMember("tcAccBias", &bit_t::tcAccBias, DATA_TYPE_F32, SYM_DEG_PER_S "/C", "Gyro temp cal slope", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_4, C_RAD2DEG);
    mapper.AddMember("tcPqrSlope", &bit_t::tcPqrSlope, DATA_TYPE_F32, SYM_DEG_PER_S "/C", "Gyro temp cal linearity", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_4, C_RAD2DEG);
    mapper.AddMember("tcAccSlope", &bit_t::tcAccSlope, DATA_TYPE_F32, SYM_M_PER_S, "Accel temp cal bias", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_4);
    mapper.AddMember("tcPqrLinearity", &bit_t::tcPqrLinearity, DATA_TYPE_F32, SYM_M_PER_S "/C", "Accel temp cal slope", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_4);
    mapper.AddMember("tcAccLinearity", &bit_t::tcAccLinearity, DATA_TYPE_F32, SYM_M_PER_S "/C", "Accel temp cal linearity", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_4);
    mapper.AddMember("pqr", &bit_t::pqr, DATA_TYPE_F32, SYM_DEG_PER_S, "Angular rate error", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_4, C_RAD2DEG);
    mapper.AddMember("acc", &bit_t::acc, DATA_TYPE_F32, SYM_M_PER_S, "Acceleration error", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_4);
    mapper.AddMember("pqrSigma", &bit_t::pqrSigma, DATA_TYPE_F32, SYM_DEG_PER_S, "Angular rate standard deviation", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_4, C_RAD2DEG);
    mapper.AddMember("accSigma", &bit_t::accSigma, DATA_TYPE_F32, SYM_M_PER_S, "Acceleration standard deviation", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_4);
    mapper.AddMember("testMode", &bit_t::testMode, DATA_TYPE_UINT8, "", "Test Mode: " + std::to_string(BIT_TEST_MODE_SIM_GPS_NOISE) + "=GPS noise, " + std::to_string(BIT_TEST_MODE_SERIAL_DRIVER_RX_OVERFLOW) + "=Rx overflow, " + std::to_string(BIT_TEST_MODE_SERIAL_DRIVER_TX_OVERFLOW) + "=Tx overflow");
    mapper.AddMember("testVar", &bit_t::testVar, DATA_TYPE_UINT8, "", "Test Mode variable (port number)");
    mapper.AddMember("detectedHardwareId", &bit_t::detectedHardwareId, DATA_TYPE_UINT16, "", "Hardware ID detected (see eIsHardwareType) used to validate correct firmware use.", DATA_FLAGS_READ_ONLY | DATA_FLAGS_DISPLAY_HEX);
}

static void PopulateMapGpxBit(data_set_t data_set[DID_COUNT], uint32_t did)
{
    DataMapper<gpx_bit_t> mapper(data_set, did);
    mapper.AddMember("results", &gpx_bit_t::results, DATA_TYPE_UINT32, "", "GPX BIT test status (see eGPXBit_results)", DATA_FLAGS_DISPLAY_HEX);
    mapper.AddMember("command", &gpx_bit_t::command, DATA_TYPE_UINT8, "", "Command (see eGPXBit_CMD)");
    mapper.AddMember("port", &gpx_bit_t::port, DATA_TYPE_UINT8, "", "Port used with the test");
    mapper.AddMember("testMode", &gpx_bit_t::testMode, DATA_TYPE_UINT8, "", "Self-test mode: 102=TxOverflow, 103=RxOverflow (see eGPXBit_test_mode)");
    mapper.AddMember("state", &gpx_bit_t::state, DATA_TYPE_UINT8, "", "Built-in self-test state (see eGPXBit_state)");
    mapper.AddMember("detectedHardwareId", &gpx_bit_t::detectedHardwareId, DATA_TYPE_UINT16, "", "Hardware ID detected (see eIsHardwareType) used to validate correct firmware use.", DATA_FLAGS_DISPLAY_HEX);
    mapper.AddArray("reserved", &gpx_bit_t::reserved, DATA_TYPE_UINT8, 2);
}

static void PopulateMapSystemFault(data_set_t data_set[DID_COUNT], uint32_t did)
{
    DataMapper<system_fault_t> mapper(data_set, did);
    int flags = DATA_FLAGS_DISPLAY_HEX;
    mapper.AddMember("status", &system_fault_t::status, DATA_TYPE_UINT32, "", "Bits: 23:20[flashMigMrk, code, stkOverflow, malloc] 19:16[busFlt, memMng, usageFlt, hardFlt] 7:4[flashMig, softRst] 3:0[bootldrRst, userRst]", DATA_FLAGS_DISPLAY_HEX);
    mapper.AddMember("g1Task", &system_fault_t::g1Task, DATA_TYPE_UINT32, "", "Active task at fault");
    mapper.AddMember("g2FileNum", &system_fault_t::g2FileNum, DATA_TYPE_UINT32, "", "File number at fault");
    mapper.AddMember("g3LineNum", &system_fault_t::g3LineNum, DATA_TYPE_UINT32, "", "Line number at fault");
    mapper.AddMember("g4", &system_fault_t::g4, DATA_TYPE_UINT32, "", "value at fault", flags);
    mapper.AddMember("g5Lr", &system_fault_t::g5Lr, DATA_TYPE_UINT32, "", "Load register at fault", flags);
    mapper.AddMember("pc", &system_fault_t::pc, DATA_TYPE_UINT32, "", "program counter at fault", flags);
    mapper.AddMember("psr", &system_fault_t::psr, DATA_TYPE_UINT32, "", "program status register at fault", flags);
}

void PopulateMapPortMonitor(data_set_t data_set[DID_COUNT], uint32_t did)
{
    DataMapper<port_monitor_t> mapper(data_set, did);
    mapper.AddMember("activePorts", &port_monitor_t::activePorts, DATA_TYPE_UINT8, "", "Number of active ports", DATA_FLAGS_READ_ONLY);    
    for( int i=0; i<NUM_SERIAL_PORTS; i++)
    {
        mapper.AddMember2("[" + std::to_string(i) + "].portInfo",           i*sizeof(port_monitor_set_t) + offsetof(port_monitor_set_t, portInfo), DATA_TYPE_UINT8, "", "High nib port type (see ePortMonPortType) low nib index.", DATA_FLAGS_READ_ONLY | DATA_FLAGS_DISPLAY_HEX);
        mapper.AddMember2("[" + std::to_string(i) + "].status",             i*sizeof(port_monitor_set_t) + offsetof(port_monitor_set_t, status), DATA_TYPE_UINT32, "", "", DATA_FLAGS_DISPLAY_HEX);
        mapper.AddMember2("[" + std::to_string(i) + "].txBytesPerSec",      i*sizeof(port_monitor_set_t) + offsetof(port_monitor_set_t, txBytesPerSec), DATA_TYPE_UINT32, "bytes/s", "Tx data rate", DATA_FLAGS_READ_ONLY);
        mapper.AddMember2("[" + std::to_string(i) + "].rxBytesPerSec",      i*sizeof(port_monitor_set_t) + offsetof(port_monitor_set_t, rxBytesPerSec), DATA_TYPE_UINT32, "bytes/s", "Rx data rate", DATA_FLAGS_READ_ONLY);
        mapper.AddMember2("[" + std::to_string(i) + "].txBytes",            i*sizeof(port_monitor_set_t) + offsetof(port_monitor_set_t, txBytes), DATA_TYPE_UINT32, "bytes", "Tx byte count");
        mapper.AddMember2("[" + std::to_string(i) + "].rxBytes",            i*sizeof(port_monitor_set_t) + offsetof(port_monitor_set_t, rxBytes), DATA_TYPE_UINT32, "bytes", "Rx byte count");
        mapper.AddMember2("[" + std::to_string(i) + "].txDataDrops",        i*sizeof(port_monitor_set_t) + offsetof(port_monitor_set_t, txDataDrops), DATA_TYPE_UINT32, "", "Tx buffer data drop occurrences");			
        mapper.AddMember2("[" + std::to_string(i) + "].rxOverflows",        i*sizeof(port_monitor_set_t) + offsetof(port_monitor_set_t, rxOverflows), DATA_TYPE_UINT32, "", "Rx buffer overflow occurrences");        
        mapper.AddMember2("[" + std::to_string(i) + "].txBytesDropped",     i*sizeof(port_monitor_set_t) + offsetof(port_monitor_set_t, txBytesDropped), DATA_TYPE_UINT32, "bytes", "Tx number of bytes that were not sent");
        mapper.AddMember2("[" + std::to_string(i) + "].rxChecksumErrors",   i*sizeof(port_monitor_set_t) + offsetof(port_monitor_set_t, rxChecksumErrors), DATA_TYPE_UINT32, "", "Rx number of checksum failures");
    }
}

void PopulateMapNmeaMsgs(data_set_t data_set[DID_COUNT], uint32_t did)
{
    DataMapper<nmea_msgs_t> mapper(data_set, did);
    mapper.AddMember("options", &nmea_msgs_t::options, DATA_TYPE_UINT32, "", "Port selection[0x0=current, 0xFF=all, 0x1=ser0, 0x2=ser1, 0x4=ser2, 0x8=USB]. (see RMC_OPTIONS_...)", DATA_FLAGS_DISPLAY_HEX);    
    for( int i=0; i<MAX_nmeaBroadcastMsgPairs; i++)
    {
        mapper.AddMember2( "[" + std::to_string(i) + "].ID",     i*sizeof(nmeaBroadcastMsgPair_t) + offsetof(nmeaBroadcastMsgPair_t, msgID),     DATA_TYPE_UINT8, "", "NMEA_ID (See eNmeaAsciiMsgId)");
        mapper.AddMember2( "[" + std::to_string(i) + "].Period", i*sizeof(nmeaBroadcastMsgPair_t) + offsetof(nmeaBroadcastMsgPair_t, msgPeriod), DATA_TYPE_UINT8, "", "NMEA_PerieNmeaMsgIdin multiples of 200ms. Ie value of 1 is 200ms or 5 is 1000ms/1s. A value of 0 stops the message broadcast.");
    }
}

static void PopulateMapImu(data_set_t data_set[DID_COUNT], uint32_t did, string description)
{
    DataMapper<imu_t> mapper(data_set, did);
    mapper.AddMember("time", &imu_t::time, DATA_TYPE_F64, "s", "Time since boot up", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_4);
    mapper.AddArray2("pqr", offsetof(imu_t, I.pqr), DATA_TYPE_F32, 3, {SYM_DEG_PER_S}, {"Angular rate.  " + description}, DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_2, C_RAD2DEG);
    mapper.AddArray2("acc", offsetof(imu_t, I.acc), DATA_TYPE_F32, 3, {SYM_M_PER_S_2}, {"Linear acceleration.  " + description}, DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3);
    mapper.AddMember("status", &imu_t::status, DATA_TYPE_UINT32, "", s_imuStatusDescription, DATA_FLAGS_DISPLAY_HEX);
}

static void PopulateMapImu3(data_set_t data_set[DID_COUNT], uint32_t did, string description)
{
    DataMapper<imu3_t> mapper(data_set, did);
    mapper.AddMember("time", &imu3_t::time, DATA_TYPE_F64, "s", "Time since boot up", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_4);
    mapper.AddMember("status", &imu3_t::status, DATA_TYPE_UINT32, "", s_imuStatusDescription, DATA_FLAGS_DISPLAY_HEX);
    mapper.AddArray2("I0.pqr", offsetof(imu3_t, I[0].pqr), DATA_TYPE_F32, 3, {SYM_DEG_PER_S}, {"IMU 1 angular rate.  " + description}, DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_2, C_RAD2DEG);
    mapper.AddArray2("I0.acc", offsetof(imu3_t, I[0].acc), DATA_TYPE_F32, 3, {SYM_M_PER_S_2}, {"IMU 1 linear acceleration.  " + description}, DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3);
    mapper.AddArray2("I1.pqr", offsetof(imu3_t, I[1].pqr), DATA_TYPE_F32, 3, {SYM_DEG_PER_S}, {"IMU 2 angular rate.  " + description}, DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_2, C_RAD2DEG);
    mapper.AddArray2("I1.acc", offsetof(imu3_t, I[1].acc), DATA_TYPE_F32, 3, {SYM_M_PER_S_2}, {"IMU 2 linear acceleration.  " + description}, DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3);
    mapper.AddArray2("I2.pqr", offsetof(imu3_t, I[2].pqr), DATA_TYPE_F32, 3, {SYM_DEG_PER_S}, {"IMU 3 angular rate.  " + description}, DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_2, C_RAD2DEG);
    mapper.AddArray2("I2.acc", offsetof(imu3_t, I[2].acc), DATA_TYPE_F32, 3, {SYM_M_PER_S_2}, {"IMU 3 linear acceleration.  " + description}, DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3);
}

static void PopulateMapSysParams(data_set_t data_set[DID_COUNT], uint32_t did)
{
    DataMapper<sys_params_t> mapper(data_set, did);
    mapper.AddMember("timeOfWeekMs",        &sys_params_t::timeOfWeekMs, DATA_TYPE_UINT32, "ms", "Time of week since Sunday morning, GMT", DATA_FLAGS_READ_ONLY);
    mapper.AddMember("insStatus",           &sys_params_t::insStatus, DATA_TYPE_UINT32, "", s_insStatusDescription, DATA_FLAGS_DISPLAY_HEX | DATA_FLAGS_INS_STATUS);
    mapper.AddMember("hdwStatus",           &sys_params_t::hdwStatus, DATA_TYPE_UINT32, "", s_hdwStatusDescription, DATA_FLAGS_DISPLAY_HEX);
    mapper.AddMember("imuTemp",             &sys_params_t::imuTemp, DATA_TYPE_F32, SYM_DEG_C, "IMU temperature", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_1);
    mapper.AddMember("baroTemp",            &sys_params_t::baroTemp, DATA_TYPE_F32, SYM_DEG_C, "IMU temperature", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_1);
    mapper.AddMember("mcuTemp",             &sys_params_t::mcuTemp, DATA_TYPE_F32, SYM_DEG_C, "Barometer temperature", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_1);
    mapper.AddMember("sysStatus",           &sys_params_t::sysStatus, DATA_TYPE_UINT32, "", "System Status Flags (See eSysStatusFlags)", DATA_FLAGS_DISPLAY_HEX);
    mapper.AddMember("imuSamplePeriodMs",   &sys_params_t::imuSamplePeriodMs, DATA_TYPE_UINT32, "ms", "IMU sample period. Zero disables sensor sampling");
    mapper.AddMember("navOutputPeriodMs",   &sys_params_t::navOutputPeriodMs, DATA_TYPE_UINT32, "ms", "Navigation/AHRS filter ouput period");
    mapper.AddMember("navUpdatePeriodMs",   &sys_params_t::navUpdatePeriodMs, DATA_TYPE_UINT32, "ms", "Navigation/AHRS filter update period", DATA_FLAGS_READ_ONLY);
    mapper.AddMember("sensorTruePeriod",    &sys_params_t::sensorTruePeriod, DATA_TYPE_F64, "us", "Actual sample period relative to GPS PPS", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_4, 1.0e6);
    mapper.AddMember("flashCfgChecksum",    &sys_params_t::flashCfgChecksum, DATA_TYPE_UINT32, "", "Flash config checksum used with host SDK synchronization", DATA_FLAGS_READ_ONLY);
    mapper.AddMember("genFaultCode",        &sys_params_t::genFaultCode, DATA_TYPE_UINT32, "", "General fault code descriptor", DATA_FLAGS_DISPLAY_HEX);
    mapper.AddMember("upTime",              &sys_params_t::upTime, DATA_TYPE_F64, "s", "Local time since startup", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_2);
}

static void PopulateMapSysSensors(data_set_t data_set[DID_COUNT], uint32_t did)
{
    DataMapper<sys_sensors_t> mapper(data_set, did);
    mapper.AddMember("time", &sys_sensors_t::time, DATA_TYPE_F64, "s", "Time since boot up", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3);
    mapper.AddMember("temp", &sys_sensors_t::temp, DATA_TYPE_F32, SYM_DEG_C, "System temperature", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3);
    mapper.AddArray("pqr", &sys_sensors_t::pqr, DATA_TYPE_F32, 3, {SYM_DEG_PER_S}, {"Angular rate"}, DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_4, C_RAD2DEG);
    mapper.AddArray("acc", &sys_sensors_t::acc, DATA_TYPE_F32, 3, {SYM_M_PER_S}, {"Linear acceleration"}, DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_4);
    mapper.AddArray("mag", &sys_sensors_t::mag, DATA_TYPE_F32, 3, {""}, {"Magnetometer normalized gauss"}, DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_4);
    mapper.AddMember("bar", &sys_sensors_t::bar, DATA_TYPE_F32, "kPa", "Barometric pressure", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3 );
    mapper.AddMember("barTemp", &sys_sensors_t::barTemp, DATA_TYPE_F32, "m", "Barometer MSL altitude", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_2 );
    mapper.AddMember("mslBar", &sys_sensors_t::mslBar, DATA_TYPE_F32, "C", "Barometer temperature", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3);
    mapper.AddMember("humidity", &sys_sensors_t::humidity, DATA_TYPE_F32, "%rH", "Relative humidity, 0%-100%", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3);
    mapper.AddMember("vin", &sys_sensors_t::vin, DATA_TYPE_F32, "V", "System input voltage", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_2 );
    mapper.AddMember("ana1", &sys_sensors_t::ana1, DATA_TYPE_F32, "V", "ADC analog 1 input voltage", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3 );
    mapper.AddMember("ana3", &sys_sensors_t::ana1, DATA_TYPE_F32, "V", "ADC analog 3 input voltage", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3 );
    mapper.AddMember("ana4", &sys_sensors_t::ana1, DATA_TYPE_F32, "V", "ADC analog 4 input voltage", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3 );
}

static void PopulateMapRmc(data_set_t data_set[DID_COUNT], uint32_t did)
{
    DataMapper<rmc_t> mapper(data_set, did);
    mapper.AddMember("bits", &rmc_t::bits, DATA_TYPE_UINT64, "", "Data stream enable bits for the specified ports.  (see RMC_BITS_...)", DATA_FLAGS_DISPLAY_HEX);
    mapper.AddMember("options", &rmc_t::options, DATA_TYPE_UINT32, "", "Options to select alternate ports to output data, etc.  (see RMC_OPTIONS_...)", DATA_FLAGS_DISPLAY_HEX);
}

void PopulateMapIns1(data_set_t data_set[DID_COUNT], uint32_t did)
{
    std::string str;
    DataMapper<ins_1_t> mapper(data_set, did);
    uint32_t flags = DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3;
    mapper.AddMember("week", &ins_1_t::week, DATA_TYPE_UINT32, "week", "Weeks since Jan 6, 1980", flags );
    mapper.AddMember("timeOfWeek", &ins_1_t::timeOfWeek, DATA_TYPE_F64, "s", "Time of week since Sunday morning, GMT", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_4 );
    mapper.AddMember("insStatus", &ins_1_t::insStatus, DATA_TYPE_UINT32,  "", s_insStatusDescription, DATA_FLAGS_DISPLAY_HEX | DATA_FLAGS_INS_STATUS);
    mapper.AddMember("hdwStatus", &ins_1_t::hdwStatus, DATA_TYPE_UINT32,  "", s_hdwStatusDescription, DATA_FLAGS_DISPLAY_HEX);
    mapper.AddVec3Rpy("theta", offsetof(ins_1_t,theta), DATA_TYPE_F32, SYM_DEG, "euler angle", flags | DATA_FLAGS_ANGLE, C_RAD2DEG); // ERROR_THRESH_ROLLPITCH);
    mapper.AddVec3Xyz("uvw", offsetof(ins_1_t,uvw), DATA_TYPE_F32, "m/s", "velocity in body frame", flags);
    str = " offset from reference LLA";
    mapper.AddArray("ned", &ins_1_t::ned, DATA_TYPE_F32, 3, {"m"}, {"North"+str, "East"+str, "Down"+str}, flags);
    mapper.AddLlaDegM("lla", offsetof(ins_1_t, lla), "WGS84 coordinate", "ellipsoid altitude", DATA_FLAGS_READ_ONLY);
}

static void PopulateMapIns2(data_set_t data_set[DID_COUNT], uint32_t did)
{
    DataMapper<ins_2_t> mapper(data_set, did);
    uint32_t flags = DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3;
    mapper.AddMember("week", &ins_2_t::week, DATA_TYPE_UINT32, "week", "Weeks since Jan 6, 1980", flags );
    mapper.AddMember("timeOfWeek", &ins_2_t::timeOfWeek, DATA_TYPE_F64, "s", "Time of week since Sunday morning, GMT", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_4 );
    mapper.AddMember("insStatus", &ins_2_t::insStatus, DATA_TYPE_UINT32,  "", s_insStatusDescription, DATA_FLAGS_DISPLAY_HEX | DATA_FLAGS_INS_STATUS);
    mapper.AddMember("hdwStatus", &ins_2_t::hdwStatus, DATA_TYPE_UINT32,  "", s_hdwStatusDescription, DATA_FLAGS_DISPLAY_HEX);
    std::string str = " quaternion body rotation with respect to NED";
    mapper.AddArray("qn2b", &ins_2_t::qn2b, DATA_TYPE_F32, 4, {""}, {"W"+str, "X"+str, "Y"+str, "Z"+str}, DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_4);
    mapper.AddVec3Xyz("uvw", offsetof(ins_2_t, uvw), DATA_TYPE_F32, "m/s", "velocity in body frame", flags);
    mapper.AddLlaDegM("lla", offsetof(ins_2_t, lla), "WGS84 coordinate", "ellipsoid altitude", DATA_FLAGS_READ_ONLY);
}

static void PopulateMapIns3(data_set_t data_set[DID_COUNT], uint32_t did)
{
    DataMapper<ins_3_t> mapper(data_set, did);
    uint32_t flags = DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3;
    mapper.AddMember("week", &ins_3_t::week, DATA_TYPE_UINT32, "week", "Weeks since Jan 6, 1980", flags );
    mapper.AddMember("timeOfWeek", &ins_3_t::timeOfWeek, DATA_TYPE_F64, "s", "Time of week since Sunday morning, GMT", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_4 );
    mapper.AddMember("insStatus", &ins_3_t::insStatus, DATA_TYPE_UINT32,  "", s_insStatusDescription, DATA_FLAGS_DISPLAY_HEX | DATA_FLAGS_INS_STATUS);
    mapper.AddMember("hdwStatus", &ins_3_t::hdwStatus, DATA_TYPE_UINT32,  "", s_hdwStatusDescription, DATA_FLAGS_DISPLAY_HEX);
    std::string str = " quaternion body rotation with respect to NED";
    mapper.AddArray("qn2b", &ins_3_t::qn2b, DATA_TYPE_F32, 4, {""}, {"W"+str, "X"+str, "Y"+str, "Z"+str}, DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_4);
    mapper.AddVec3Xyz("uvw", offsetof(ins_3_t, uvw), DATA_TYPE_F32, "m/s", "velocity in body frame", flags);
    mapper.AddLlaDegM("lla", offsetof(ins_3_t, lla), "WGS84 coordinate", "ellipsoid altitude", DATA_FLAGS_READ_ONLY);
    mapper.AddMember("msl", &ins_3_t::msl, DATA_TYPE_F32, "m", "Height above mean sea level (MSL)", flags);
}

static void PopulateMapIns4(data_set_t data_set[DID_COUNT], uint32_t did)
{
    DataMapper<ins_4_t> mapper(data_set, did);
    uint32_t flags = DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3;
    mapper.AddMember("week", &ins_4_t::week, DATA_TYPE_UINT32, "week", "Weeks since Jan 6, 1980", flags );
    mapper.AddMember("timeOfWeek", &ins_4_t::timeOfWeek, DATA_TYPE_F64, "s", "Time of week since Sunday morning, GMT", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_4 );
    mapper.AddMember("insStatus", &ins_4_t::insStatus, DATA_TYPE_UINT32,  "", s_insStatusDescription, DATA_FLAGS_DISPLAY_HEX | DATA_FLAGS_INS_STATUS);
    mapper.AddMember("hdwStatus", &ins_4_t::hdwStatus, DATA_TYPE_UINT32,  "", s_hdwStatusDescription, DATA_FLAGS_DISPLAY_HEX);
    std::string str = " quaternion body rotation with respect to ECEF";
    mapper.AddArray("qe2b", &ins_4_t::qe2b, DATA_TYPE_F32, 4, {""}, {"W"+str, "X"+str, "Y"+str, "Z"+str}, DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_4);
    mapper.AddVec3Xyz("ve", offsetof(ins_4_t,ve), DATA_TYPE_F32, "m/s", "velocity in ECEF (earth-centered earth-fixed) frame", flags);
    mapper.AddVec3Xyz("ecef", offsetof(ins_4_t,ecef), DATA_TYPE_F64, "m", "position in ECEF (earth-centered earth-fixed) frame", flags);
}

static void PopulateMapGpsPos(data_set_t data_set[DID_COUNT], uint32_t did)
{
    DataMapper<gps_pos_t> mapper(data_set, did);
    mapper.AddMember("week", &gps_pos_t::week, DATA_TYPE_UINT32, "week", "Weeks since Jan 6, 1980", DATA_FLAGS_READ_ONLY);
    mapper.AddMember("timeOfWeekMs", &gps_pos_t::timeOfWeekMs, DATA_TYPE_UINT32, "ms", "Time of week since Sunday morning", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_4);
    mapper.AddMember("status", &gps_pos_t::status, DATA_TYPE_UINT32, "", "GPS status: [0x000000xx] number of satellites used, [0x0000xx00] fix type, [0x00xx0000] status flags", DATA_FLAGS_READ_ONLY | DATA_FLAGS_DISPLAY_HEX | DATA_FLAGS_GPS_STATUS);
    mapper.AddArray("ecef", &gps_pos_t::ecef, DATA_TYPE_F64, 3, {"m"}, {"Position in ECEF {x,y,z}"}, DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3);
    mapper.AddLlaDegM("lla", offsetof(gps_pos_t, lla), "", "ellipsoid altitude", DATA_FLAGS_READ_ONLY);
    mapper.AddMember("hMSL", &gps_pos_t::hMSL, DATA_TYPE_F32, "m", "Meters above sea level", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_4);
    mapper.AddMember("hAcc", &gps_pos_t::hAcc, DATA_TYPE_F32, "m", "Position horizontal accuracy", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_4);
    mapper.AddMember("vAcc", &gps_pos_t::vAcc, DATA_TYPE_F32, "m", "Position vertical accuracy", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3);
    mapper.AddMember("pDop", &gps_pos_t::pDop, DATA_TYPE_F32, "m", "Position dilution of precision", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3);
    mapper.AddMember("cnoMean", &gps_pos_t::cnoMean, DATA_TYPE_F32, "dBHz", "Average of non-zero satellite carrier to noise ratios (signal strengths)", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_1);
    mapper.AddMember("towOffset", &gps_pos_t::towOffset, DATA_TYPE_F64, "sec", "Time sync offset from local clock", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_5);
    mapper.AddMember("leapS", &gps_pos_t::leapS, DATA_TYPE_UINT8, "", "GPS leap seconds (GPS-UTC). Receiver's best knowledge of the leap seconds offset from UTC to GPS time.", DATA_FLAGS_READ_ONLY);
    mapper.AddMember("satsUsed", &gps_pos_t::satsUsed, DATA_TYPE_UINT8, "", "Number of satellites used in the solution", DATA_FLAGS_READ_ONLY);
    mapper.AddMember("cnoMeanSigma", &gps_pos_t::cnoMeanSigma, DATA_TYPE_UINT8, "10dBHz", "10x standard deviation of CNO mean over past 5 seconds", DATA_FLAGS_READ_ONLY);
    mapper.AddMember("status2", &gps_pos_t::status2, DATA_TYPE_UINT8, "", "(see eGpsStatus2) GPS status2: [0x0X] Spoofing/Jamming status, [0xX0] Unused", DATA_FLAGS_READ_ONLY | DATA_FLAGS_DISPLAY_HEX );
}

static void PopulateMapGpsVel(data_set_t data_set[DID_COUNT], uint32_t did)
{
    DataMapper<gps_vel_t> mapper(data_set, did);
    mapper.AddMember("timeOfWeekMs", &gps_vel_t::timeOfWeekMs, DATA_TYPE_UINT32, "ms", "Time of week since Sunday morning", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_4);
    mapper.AddArray("vel", &gps_vel_t::vel, DATA_TYPE_F32, 3, {"m/s"}, {"Velocity in ECEF {vx,vy,vz} or NED {vN, vE, 0} if status GPS_STATUS_FLAGS_GPS_NMEA_DATA = 0 or 1"}, DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_2);
    mapper.AddMember("sAcc", &gps_vel_t::sAcc, DATA_TYPE_F32, "m/s", "Speed accuracy", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_2);
    mapper.AddMember("status", &gps_vel_t::status, DATA_TYPE_UINT32, "", "GPS status: NMEA input if status flag GPS_STATUS_FLAGS_GPS_NMEA_DATA", DATA_FLAGS_READ_ONLY | DATA_FLAGS_DISPLAY_HEX);
}

static void PopulateMapGpsSat(data_set_t data_set[DID_COUNT], uint32_t did)
{
    DataMapper<gps_sat_t> mapper(data_set, did);
    mapper.AddMember("timeOfWeekMs", &gps_sat_t::timeOfWeekMs, DATA_TYPE_UINT32, "ms", "Time of week since Sunday morning, GMT", DATA_FLAGS_READ_ONLY);
    mapper.AddMember("numSats", &gps_sat_t::numSats, DATA_TYPE_UINT32, "", "Number of satellites in sky", DATA_FLAGS_READ_ONLY);

    for( int n=0; n<MAX_NUM_SATELLITES; n++)
    {
        mapper.AddMember2("sat" + std::to_string(n) + ".gnssId",    n*sizeof(gps_sat_sv_t) + offsetof(gps_sat_t, sat[0].gnssId),    DATA_TYPE_UINT8);
        mapper.AddMember2("sat" + std::to_string(n) + ".svId",      n*sizeof(gps_sat_sv_t) + offsetof(gps_sat_t, sat[0].svId),      DATA_TYPE_UINT8);
        mapper.AddMember2("sat" + std::to_string(n) + ".elev",      n*sizeof(gps_sat_sv_t) + offsetof(gps_sat_t, sat[0].elev),      DATA_TYPE_INT8);
        mapper.AddMember2("sat" + std::to_string(n) + ".azim",      n*sizeof(gps_sat_sv_t) + offsetof(gps_sat_t, sat[0].azim),      DATA_TYPE_INT16);
        mapper.AddMember2("sat" + std::to_string(n) + ".cno",       n*sizeof(gps_sat_sv_t) + offsetof(gps_sat_t, sat[0].cno),       DATA_TYPE_UINT8);
        mapper.AddMember2("sat" + std::to_string(n) + ".status",    n*sizeof(gps_sat_sv_t) + offsetof(gps_sat_t, sat[0].status),    DATA_TYPE_UINT16);
    }
}

static void PopulateMapGpsSig(data_set_t data_set[DID_COUNT], uint32_t did)
{
    DataMapper<gps_sig_t> mapper(data_set, did);
    mapper.AddMember("timeOfWeekMs", &gps_sig_t::timeOfWeekMs, DATA_TYPE_UINT32, "ms", "Time of week since Sunday morning, GMT", DATA_FLAGS_READ_ONLY);
    mapper.AddMember("numSigs", &gps_sig_t::numSigs, DATA_TYPE_UINT32, "", "Number of signals in sky", DATA_FLAGS_READ_ONLY);

    for( int n=0; n<MAX_NUM_SAT_SIGNALS; n++)
    {
        mapper.AddMember2("sig" + std::to_string(n) + ".gnssId",    n*sizeof(gps_sig_sv_t) + offsetof(gps_sig_t, sig[0].gnssId),    DATA_TYPE_UINT8);
        mapper.AddMember2("sig" + std::to_string(n) + ".svId",      n*sizeof(gps_sig_sv_t) + offsetof(gps_sig_t, sig[0].svId),      DATA_TYPE_UINT8);
        mapper.AddMember2("sig" + std::to_string(n) + ".sigId",     n*sizeof(gps_sig_sv_t) + offsetof(gps_sig_t, sig[0].sigId),     DATA_TYPE_UINT8);
        mapper.AddMember2("sig" + std::to_string(n) + ".cno",       n*sizeof(gps_sig_sv_t) + offsetof(gps_sig_t, sig[0].cno),       DATA_TYPE_UINT8);
        mapper.AddMember2("sig" + std::to_string(n) + ".quality",   n*sizeof(gps_sig_sv_t) + offsetof(gps_sig_t, sig[0].quality),   DATA_TYPE_UINT8);
        mapper.AddMember2("sig" + std::to_string(n) + ".status",    n*sizeof(gps_sig_sv_t) + offsetof(gps_sig_t, sig[0].status),    DATA_TYPE_UINT16);
    }
}

static void PopulateMapGpsVersion(data_set_t data_set[DID_COUNT], uint32_t did)
{
    DataMapper<gps_version_t> mapper(data_set, did);
    mapper.AddMember("swVersion", &gps_version_t::swVersion, DATA_TYPE_STRING, "", "Software version");
    mapper.AddMember("hwVersion", &gps_version_t::hwVersion, DATA_TYPE_STRING, "", "Hardware version");
    mapper.AddArray("extension", &gps_version_t::extension, DATA_TYPE_STRING, GPS_VER_NUM_EXTENSIONS, {""}, {"Extension 30 bytes array description."});
}

static void PopulateMapGpsTimepulse(data_set_t data_set[DID_COUNT], uint32_t did)
{
    DataMapper<gps_timepulse_t> mapper(data_set, did);
    mapper.AddMember("towOffset", &gps_timepulse_t::towOffset, DATA_TYPE_F64, "s", "Week seconds offset from MCU to GPS time.", DATA_FLAGS_FIXED_DECIMAL_4);
    mapper.AddMember("towGps", &gps_timepulse_t::towGps, DATA_TYPE_F64, "s", "Week seconds for next timepulse (from start of GPS week)", DATA_FLAGS_FIXED_DECIMAL_4);
    mapper.AddMember("timeMcu", &gps_timepulse_t::timeMcu, DATA_TYPE_F64, "s", "Local MCU week seconds.", DATA_FLAGS_FIXED_DECIMAL_4);
    mapper.AddMember("msgTimeMs", &gps_timepulse_t::msgTimeMs, DATA_TYPE_UINT32, "ms", "Local timestamp of TIM-TP message used to validate timepulse.");
    mapper.AddMember("plsTimeMs", &gps_timepulse_t::plsTimeMs, DATA_TYPE_UINT32, "ms", "Local timestamp of time sync pulse external interrupt used to validate timepulse.");
    mapper.AddMember("syncCount", &gps_timepulse_t::syncCount, DATA_TYPE_UINT8, "", "Counter for successful timesync events.");
    mapper.AddMember("badPulseAgeCount", &gps_timepulse_t::badPulseAgeCount, DATA_TYPE_UINT8, "", "Counter for failed timesync events.");
    mapper.AddMember("ppsInterruptReinitCount", &gps_timepulse_t::ppsInterruptReinitCount, DATA_TYPE_UINT8, "", "Counter for GPS PPS interrupt re-initalization.");
    mapper.AddMember("plsCount", &gps_timepulse_t::plsCount, DATA_TYPE_UINT8, "", "");
    mapper.AddMember("lastSyncTimeMs", &gps_timepulse_t::lastSyncTimeMs, DATA_TYPE_UINT32, "ms", "Local timestamp of last valid PPS sync.");
    mapper.AddMember("sinceLastSyncTimeMs", &gps_timepulse_t::sinceLastSyncTimeMs, DATA_TYPE_UINT32, "ms", "Time since last valid PPS sync.");			
}

static void PopulateMapMagnetometer(data_set_t data_set[DID_COUNT], uint32_t did)
{
    DataMapper<magnetometer_t> mapper(data_set, did);
    mapper.AddMember("time", &magnetometer_t::time, DATA_TYPE_F64, "s", "Time since boot up", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_4);
    mapper.AddArray("mag", &magnetometer_t::mag, DATA_TYPE_F32, 3, {SYM_M_PER_S}, {"Normalized gauss"}, DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3);
}

static void PopulateMapBarometer(data_set_t data_set[DID_COUNT], uint32_t did)
{
    DataMapper<barometer_t> mapper(data_set, did);
    mapper.AddMember("time", &barometer_t::time, DATA_TYPE_F64, "s", "Time since boot up", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_4);
    mapper.AddMember("bar", &barometer_t::bar, DATA_TYPE_F32, "kPa", "Barometric pressure", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3);
    mapper.AddMember("mslBar", &barometer_t::mslBar, DATA_TYPE_F32, "m", "MSL altitude from barometric pressure sensor", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_2);
    mapper.AddMember("barTemp", &barometer_t::barTemp, DATA_TYPE_F32, SYM_DEG_C, "Barometer temperature", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3);
    mapper.AddMember("humidity", &barometer_t::humidity, DATA_TYPE_F32, "%rH", "Relative humidity, 0%-100%", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3);
}

static void PopulateMapPimu(data_set_t data_set[DID_COUNT], uint32_t did, string description)
{
    DataMapper<pimu_t> mapper(data_set, did);
    mapper.AddMember("time", &pimu_t::time, DATA_TYPE_F64, "s", "Local time since startup.", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_4);
    mapper.AddMember("dt", &pimu_t::dt, DATA_TYPE_F32, "s", "Integration period.", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_4);
    mapper.AddMember("status", &pimu_t::status, DATA_TYPE_UINT32, "", s_imuStatusDescription, DATA_FLAGS_DISPLAY_HEX);
    mapper.AddArray("theta", &pimu_t::theta, DATA_TYPE_F32, 3, {SYM_DEG}, {"IMU delta theta coning and sculling integrals in body/IMU frame.  " + description}, DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_4, C_RAD2DEG);
    mapper.AddArray("vel", &pimu_t::vel, DATA_TYPE_F32, 3, {"m/s"}, {"IMU delta velocity coning and sculling integrals in body/IMU frame.  " + description}, DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_5);
}

static void PopulateMapPimuMag(data_set_t data_set[DID_COUNT], uint32_t did)
{
    DataMapper<pimu_mag_t> mapper(data_set, did);
    mapper.AddMember2("imutime", offsetof(pimu_mag_t, pimu.time), DATA_TYPE_F64, "s", "Local time since startup.", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_4);
    mapper.AddArray2("theta", offsetof(pimu_mag_t, pimu.theta), DATA_TYPE_F32, 3, {SYM_DEG}, {"IMU delta theta coning and sculling integrals in body/IMU frame."}, DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_4, C_RAD2DEG);
    mapper.AddArray2("vel", offsetof(pimu_mag_t, pimu.vel), DATA_TYPE_F32, 3, {"m/s"}, {"IMU delta velocity coning and sculling integrals in body/IMU frame."}, DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_5);
    mapper.AddMember2("dt", offsetof(pimu_mag_t, pimu.dt), DATA_TYPE_F32, "s", "Integration period.", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_4);
    mapper.AddMember2("imustatus", offsetof(pimu_mag_t, pimu.status), DATA_TYPE_UINT32, "", s_imuStatusDescription, DATA_FLAGS_DISPLAY_HEX);
    mapper.AddMember2("magtime", offsetof(pimu_mag_t, mag.time), DATA_TYPE_F64, "s", "Local time since startup.", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_4);
    mapper.AddArray2("mag", offsetof(pimu_mag_t, mag.mag), DATA_TYPE_F32, 3, {""}, {"Normalized gauss"}, DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3);
}

static void PopulateMapImuMag(data_set_t data_set[DID_COUNT], uint32_t did)
{
    DataMapper<imu_mag_t> mapper(data_set, did);
    mapper.AddMember2("imutime", offsetof(imu_mag_t, imu.time), DATA_TYPE_F64, "s", "Time since boot up", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_4);
    mapper.AddArray2("pqr", offsetof(imu_mag_t, imu.I.pqr), DATA_TYPE_F32, 3, {SYM_DEG_PER_S}, {"IMU Angular rate"}, DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_2, C_RAD2DEG);
    mapper.AddArray2("acc", offsetof(imu_mag_t, imu.I.acc), DATA_TYPE_F32, 3, {SYM_M_PER_S_2}, {"IMU Linear acceleration"}, DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3);
    mapper.AddMember2("imustatus", offsetof(imu_mag_t, imu.status), DATA_TYPE_UINT32, "", s_imuStatusDescription, DATA_FLAGS_DISPLAY_HEX);
    mapper.AddMember2("magtime", offsetof(imu_mag_t, mag.time), DATA_TYPE_F64, "s", "Local time since startup.", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_4);
    mapper.AddArray2("mag", offsetof(imu_mag_t, mag.mag), DATA_TYPE_F32, 3, {""}, {"Normalized gauss"}, DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3);
}

static void PopulateMapMagCal(data_set_t data_set[DID_COUNT], uint32_t did)
{
    DataMapper<mag_cal_t> mapper(data_set, did);
    mapper.AddMember("state", &mag_cal_t::state, DATA_TYPE_UINT32, "", "Mag recalibration state.  COMMANDS: 1=multi-axis, 2=single-axis, 101=abort, STATUS: 200=running, 201=done (see eMagCalState)");
    mapper.AddMember("progress", &mag_cal_t::progress, DATA_TYPE_F32, "%", "Mag recalibration progress indicator: 0-100 %", DATA_FLAGS_FIXED_DECIMAL_1);
    mapper.AddMember("declination", &mag_cal_t::declination, DATA_TYPE_F32, SYM_DEG, "Magnetic declination estimate", DATA_FLAGS_FIXED_DECIMAL_2, C_RAD2DEG);
}

static void PopulateMapInfieldCal(data_set_t data_set[DID_COUNT], uint32_t did)
{
    DataMapper<infield_cal_t> mapper(data_set, did);
    mapper.AddMember("state", &infield_cal_t::state, DATA_TYPE_UINT32, "", "0=off, init[1=IMU, 2=gyro, 3=accel], init align INS[4, 5=+IMU, 6=+gyro, 7=+accel], 8=sample, 9=finish (see eInfieldCalState)");
    mapper.AddMember("status", &infield_cal_t::status, DATA_TYPE_UINT32, "", "Infield cal status (see eInfieldCalStatus)", DATA_FLAGS_DISPLAY_HEX);
    mapper.AddMember("sampleTimeMs", &infield_cal_t::sampleTimeMs, DATA_TYPE_UINT32, "ms", "Duration of IMU sample averaging. sampleTimeMs = 0 means \"imu\" member contains the IMU bias from flash.");

    for (int i=0; i<NUM_IMU_DEVICES; i++)
    {
        mapper.AddArray2("imu" + std::to_string(i) + ".pqr", i*sizeof(imus_t) + offsetof(infield_cal_t, imu[0].pqr), DATA_TYPE_F32, 3, {SYM_DEG_PER_S}, {"Sampled angular rate.  IMU bias when state=INFIELD_CAL_STATE_SAVED_AND_FINISHED"}, DATA_FLAGS_FIXED_DECIMAL_3, C_RAD2DEG);
        mapper.AddArray2("imu" + std::to_string(i) + ".acc", i*sizeof(imus_t) + offsetof(infield_cal_t, imu[0].acc), DATA_TYPE_F32, 3, {SYM_M_PER_S_2}, {"Sampled linear acceleration.  IMU bias when state=INFIELD_CAL_STATE_SAVED_AND_FINISHED"}, DATA_FLAGS_FIXED_DECIMAL_4);
    }

    for (int i=0; i<3; i++)
    {
        mapper.AddArray2("calData" + std::to_string(i) + ".down.dev[0].acc", i*sizeof(infield_cal_vaxis_t) + offsetof(infield_cal_t, calData[0].down.dev[0].acc), DATA_TYPE_F32, 3, {SYM_M_PER_S_2}, {"Linear acceleration"}, DATA_FLAGS_FIXED_DECIMAL_4);
        mapper.AddArray2("calData" + std::to_string(i) + ".down.dev[1].acc", i*sizeof(infield_cal_vaxis_t) + offsetof(infield_cal_t, calData[0].down.dev[1].acc), DATA_TYPE_F32, 3, {SYM_M_PER_S_2}, {"Linear acceleration"}, DATA_FLAGS_FIXED_DECIMAL_4);
        mapper.AddArray2("calData" + std::to_string(i) + ".down.dev[2].acc", i*sizeof(infield_cal_vaxis_t) + offsetof(infield_cal_t, calData[0].down.dev[2].acc), DATA_TYPE_F32, 3, {SYM_M_PER_S_2}, {"Linear acceleration"}, DATA_FLAGS_FIXED_DECIMAL_4);
        mapper.AddMember2("calData" + std::to_string(i) + ".down.yaw",        i*sizeof(infield_cal_vaxis_t) + offsetof(infield_cal_t, calData[0].down.yaw), DATA_TYPE_F32, SYM_DEG, "Yaw angle. >=999 means two samples have been averaged.", DATA_FLAGS_FIXED_DECIMAL_1, C_RAD2DEG);
        mapper.AddArray2("calData" + std::to_string(i) + ".up.dev[0].acc",   i*sizeof(infield_cal_vaxis_t) + offsetof(infield_cal_t, calData[0].up.dev[0].acc), DATA_TYPE_F32, 3, {SYM_M_PER_S_2}, {"Linear acceleration"}, DATA_FLAGS_FIXED_DECIMAL_4);
        mapper.AddArray2("calData" + std::to_string(i) + ".up.dev[1].acc",   i*sizeof(infield_cal_vaxis_t) + offsetof(infield_cal_t, calData[0].up.dev[1].acc), DATA_TYPE_F32, 3, {SYM_M_PER_S_2}, {"Linear acceleration"}, DATA_FLAGS_FIXED_DECIMAL_4);
        mapper.AddArray2("calData" + std::to_string(i) + ".up.dev[2].acc",   i*sizeof(infield_cal_vaxis_t) + offsetof(infield_cal_t, calData[0].up.dev[2].acc), DATA_TYPE_F32, 3, {SYM_M_PER_S_2}, {"Linear acceleration"}, DATA_FLAGS_FIXED_DECIMAL_4);
        mapper.AddMember2("calData" + std::to_string(i) + ".up.yaw",          i*sizeof(infield_cal_vaxis_t) + offsetof(infield_cal_t, calData[0].up.yaw), DATA_TYPE_F32, SYM_DEG, "Yaw angle. >=999 means two samples have been averaged.", DATA_FLAGS_FIXED_DECIMAL_1, C_RAD2DEG);
    }
}

static void PopulateMapWheelEncoder(data_set_t data_set[DID_COUNT], uint32_t did)
{
    DataMapper<wheel_encoder_t> mapper(data_set, did);
    mapper.AddMember("timeOfWeek", &wheel_encoder_t::timeOfWeek, DATA_TYPE_F64, "s", "Time of measurement wrt current week", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_4);
    mapper.AddMember("status", &wheel_encoder_t::status, DATA_TYPE_UINT32, "", "", DATA_FLAGS_DISPLAY_HEX);
    mapper.AddMember("theta_l", &wheel_encoder_t::theta_l, DATA_TYPE_F32, SYM_DEG, "Left wheel angle", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3, C_RAD2DEG);
    mapper.AddMember("omega_l", &wheel_encoder_t::omega_l, DATA_TYPE_F32, SYM_DEG_PER_S, "Left wheel angular rate", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_2, C_RAD2DEG);
    mapper.AddMember("theta_r", &wheel_encoder_t::theta_r, DATA_TYPE_F32, SYM_DEG, "Right wheel angle", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3, C_RAD2DEG);
    mapper.AddMember("omega_r", &wheel_encoder_t::omega_r, DATA_TYPE_F32, SYM_DEG_PER_S, "Right wheel angular rate", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_2, C_RAD2DEG);
    mapper.AddMember("wrap_count_l", &wheel_encoder_t::wrap_count_l, DATA_TYPE_UINT32, "", "");
    mapper.AddMember("wrap_count_r", &wheel_encoder_t::wrap_count_r, DATA_TYPE_UINT32, "", "");
    mapper.AddMember("var_wheel_omega", &wheel_encoder_t::var_wheel_omega, DATA_TYPE_F32, "rad^2/s^2", "Wheel encoder velocity noise variance");
    mapper.AddMember("var_wheel_theta", &wheel_encoder_t::var_wheel_theta, DATA_TYPE_F32, "rad^2",     "Wheel encoder angle noise variance");
}

static void PopulateMapGroundVehicle(data_set_t data_set[DID_COUNT], uint32_t did)
{
    DataMapper<ground_vehicle_t> mapper(data_set, did);
    mapper.AddMember("timeOfWeekMs", &ground_vehicle_t::timeOfWeekMs, DATA_TYPE_UINT32, "ms", "Time of week since Sunday morning, GMT", DATA_FLAGS_READ_ONLY);
    mapper.AddMember("status", &ground_vehicle_t::status, DATA_TYPE_UINT32, "", "", DATA_FLAGS_DISPLAY_HEX | DATA_FLAGS_READ_ONLY);
    mapper.AddMember("mode", &ground_vehicle_t::mode, DATA_TYPE_UINT32, "", "1=learning; Commands[2=start, 3=resume, 4=clear&start, 5=stop&save, 6=cancel]");
    mapper.AddMember2("wheelConfig.bits", offsetof(ground_vehicle_t, wheelConfig.bits), DATA_TYPE_UINT32, "", "", DATA_FLAGS_DISPLAY_HEX);
    mapper.AddArray2("wheelConfig.transform.e_b2w", offsetof(ground_vehicle_t, wheelConfig.transform.e_b2w), DATA_TYPE_F32, 3, { "rad" }, { "Euler angle rotation from imu (body) to wheel frame (center of non-steering axle)" }, DATA_FLAGS_FIXED_DECIMAL_3);
    mapper.AddArray2("wheelConfig.transform.e_b2w_sigma", offsetof(ground_vehicle_t, wheelConfig.transform.e_b2w_sigma), DATA_TYPE_F32, 3, { "rad" }, { "Standard deviation of Euler angles describing rotation from imu (body) to wheel frame (center of non-steering axle)" }, DATA_FLAGS_FIXED_DECIMAL_3);
    mapper.AddArray2("wheelConfig.transform.t_b2w", offsetof(ground_vehicle_t, wheelConfig.transform.t_b2w), DATA_TYPE_F32, 3, { "m" }, { "Translation from imu (body) to wheel frame origin (center of non-steering axle), expressed in imu (body) frame" }, DATA_FLAGS_FIXED_DECIMAL_3);
    mapper.AddArray2("wheelConfig.transform.t_b2w_sigma", offsetof(ground_vehicle_t, wheelConfig.transform.t_b2w_sigma), DATA_TYPE_F32, 3, { "m" }, { "Standard deviation of translation from imu (body) to wheel frame origin (center of non-steering axle), expressed in imu (body) frame" }, DATA_FLAGS_FIXED_DECIMAL_3);
    mapper.AddMember2("wheelConfig.track_width", offsetof(ground_vehicle_t, wheelConfig.track_width), DATA_TYPE_F32, "m", "Distance between left and right wheels");
    mapper.AddMember2("wheelConfig.radius", offsetof(ground_vehicle_t, wheelConfig.radius), DATA_TYPE_F32, "m", "Wheel radius");
}

static void PopulateMapSystemCommand(data_set_t data_set[DID_COUNT], uint32_t did)
{
    DataMapper<system_command_t> mapper(data_set, did);
    mapper.AddMember("command", &system_command_t::command, DATA_TYPE_INT32, "", "99=software reset, 5=zero sensors");
    mapper.AddMember("invCommand", &system_command_t::invCommand, DATA_TYPE_INT32, "", "Bitwise inverse of command ((-command) - 1) required to process command.");
}

static void PopulateMapNvmFlashCfg(data_set_t data_set[DID_COUNT], uint32_t did)
{
    DataMapper<nvm_flash_cfg_t> mapper(data_set, did);
    string str;
    mapper.AddMember("startupImuDtMs", &nvm_flash_cfg_t::startupImuDtMs, DATA_TYPE_UINT32, "ms", "IMU sample (system input data) period set on startup. Cannot be larger than startupInsDtMs. Zero disables sensor/IMU sampling.");
    mapper.AddMember("startupNavDtMs", &nvm_flash_cfg_t::startupNavDtMs, DATA_TYPE_UINT32, "ms", "GPS measurement (system input data) update period in milliseconds set on startup. 200ms minimum (5Hz max).");
    mapper.AddMember("startupGPSDtMs", &nvm_flash_cfg_t::startupGPSDtMs, DATA_TYPE_UINT32, "ms", "Nav filter (system output data) update period set on startup. 1ms min (1KHz max).");
    mapper.AddMember("ser0BaudRate", &nvm_flash_cfg_t::ser0BaudRate, DATA_TYPE_UINT32, "bps", "Serial port 0 baud rate");
    mapper.AddMember("ser1BaudRate", &nvm_flash_cfg_t::ser1BaudRate, DATA_TYPE_UINT32, "bps", "Serial port 1 baud rate");
    mapper.AddMember("ser2BaudRate", &nvm_flash_cfg_t::ser2BaudRate, DATA_TYPE_UINT32, "bps", "Serial port 2 baud rate");
    mapper.AddVec3Rpy("insRotation", offsetof(nvm_flash_cfg_t, insRotation), DATA_TYPE_F32, SYM_DEG, "rotation from INS Sensor Frame to Intermediate Output Frame.  Order applied: yaw, pitch, roll.", 0, C_RAD2DEG);
    mapper.AddVec3Xyz("insOffset", offsetof(nvm_flash_cfg_t, insOffset), DATA_TYPE_F32, "m", "offset from Intermediate Output Frame to INS Output Frame.  INS rotation is applied before this.");
    mapper.AddVec3Xyz("gps1AntOffset", offsetof(nvm_flash_cfg_t,gps1AntOffset), DATA_TYPE_F32, "m", "offset from Sensor Frame origin to GPS1 antenna.");
    mapper.AddVec3Xyz("gps2AntOffset", offsetof(nvm_flash_cfg_t,gps2AntOffset), DATA_TYPE_F32, "m", "offset from Sensor Frame origin to GPS2 antenna.");

    mapper.AddMember("gpsTimeSyncPeriodMs", &nvm_flash_cfg_t::gpsTimeSyncPeriodMs, DATA_TYPE_UINT32, "ms", "GPS time synchronization pulse period.", 0, 1.0);
    mapper.AddMember("gpsTimeUserDelay", &nvm_flash_cfg_t::gpsTimeUserDelay, DATA_TYPE_F32, "s", "User defined delay for GPS time.  This parameter can be used to account for GPS antenna cable delay.", DATA_FLAGS_FIXED_DECIMAL_3, 1.0);
    mapper.AddMember("gpsMinimumElevation", &nvm_flash_cfg_t::gpsMinimumElevation, DATA_TYPE_F32, SYM_DEG, "GPS minimum elevation of a satellite above the horizon to be used in the solution.", DATA_FLAGS_FIXED_DECIMAL_1, C_RAD2DEG);
    mapper.AddMember("gnssSatSigConst", &nvm_flash_cfg_t::gnssSatSigConst, DATA_TYPE_UINT16, "", "GNSS constellations used. 0x0003=GPS, 0x000C=QZSS, 0x0030=Galileo, 0x00C0=Beidou, 0x0300=GLONASS, 0x1000=SBAS (see eGnssSatSigConst)", DATA_FLAGS_DISPLAY_HEX, 1.0);

    mapper.AddMember("dynamicModel", &nvm_flash_cfg_t::dynamicModel, DATA_TYPE_UINT8, "", "0:port, 2:stationary, 3:walk, 4:ground vehicle, 5:sea, 6:air<1g, 7:air<2g, 8:air<4g, 9:wrist", 0, 1.0);
    str = "AutobaudOff [0x1=Ser0, 0x2=Ser1], 0x4=AutoMagRecal, 0x8=DisableMagDecEst, ";
    str += "0x10=DisableLeds, ";
    str += "0x100=1AxisMagRecal, ";
    str += "FusionOff [0x1000=Mag, 0x2000=Baro, 0x4000=GPS], ";
    str += "0x10000=enZeroVel, 0x100000=enNavStrobeOutput";
    mapper.AddMember("sysCfgBits", &nvm_flash_cfg_t::sysCfgBits, DATA_TYPE_UINT32, "", str, DATA_FLAGS_DISPLAY_HEX);
    str = "Rover [0x1=G1, 0x2=G2], 0x8=GCompass, ";
    str += "BaseOutG1 [0x10=UbxS0, 0x20=UbxS1, 0x40=RtcmS0, 0x80=RtcmS1], ";
    str += "BaseOutG2 [0x100=UbxS0, 0x200=UbxS1, 0x400=RtcmS0, 0x800=RtcmS1], ";
    str += "0x1000=MovingBasePos, 0x4000=SameHdwRvrBase";
    mapper.AddMember("RTKCfgBits", &nvm_flash_cfg_t::RTKCfgBits, DATA_TYPE_UINT32, "", str, DATA_FLAGS_DISPLAY_HEX);
    mapper.AddMember("ioConfig",  &nvm_flash_cfg_t::ioConfig, DATA_TYPE_UINT32, "", "(see enum eIoConfig) IMU disable: 0x1000000,0x20000000,0x4000000", DATA_FLAGS_DISPLAY_HEX);
    mapper.AddMember("ioConfig2", &nvm_flash_cfg_t::ioConfig2, DATA_TYPE_UINT8, "", "GNSS2 PPS/Strobe configuration. (see enum eIoConfig)", DATA_FLAGS_DISPLAY_HEX);
    mapper.AddMember("platformConfig", &nvm_flash_cfg_t::platformConfig, DATA_TYPE_UINT32, "", "Hardware platform (IMX carrier board, i.e. RUG, EVB, IG) configuration bits (see ePlatformConfig)", DATA_FLAGS_DISPLAY_HEX);
    str =  "Gyr FS (deg/s) 0x7:[0=250, 1=500, 2=1000, 3=2000, 4=4000], ";
    str += "Acc FS 0x30:[0=2g, 1=4g, 2=8g, 3=16g], ";
    str += "Gyr DLPF (Hz) 0x0F00:[0=250, 1=184, 2=92, 3=41, 4=20, 5=10, 6=5], ";
    str += "Acc DLPF (Hz) 0xF000:[0=218, 1=218, 2=99, 3=45, 4=21, 5=10, 6=5], ";
    mapper.AddMember("sensorConfig", &nvm_flash_cfg_t::sensorConfig, DATA_TYPE_UINT32, "", str, DATA_FLAGS_DISPLAY_HEX);

    mapper.AddLlaDegM("refLla", offsetof(nvm_flash_cfg_t, refLla), "Reference for north east down (NED) calculations" , "ellipsoid altitude", DATA_FLAGS_READ_ONLY);
    mapper.AddLlaDegM("lastLla", offsetof(nvm_flash_cfg_t, lastLla), "Last known position (Aids GPS startup)", "ellipsoid altitude", DATA_FLAGS_READ_ONLY);
    mapper.AddMember("lastLlaTimeOfWeekMs", &nvm_flash_cfg_t::lastLlaTimeOfWeekMs, DATA_TYPE_UINT32, "ms", "Last LLA update time since Sunday morning");
    mapper.AddMember("lastLlaWeek", &nvm_flash_cfg_t::lastLlaWeek, DATA_TYPE_UINT32, "week", "Last LLA update Weeks since Jan 6, 1980");
    mapper.AddMember("lastLlaUpdateDistance", &nvm_flash_cfg_t::lastLlaUpdateDistance, DATA_TYPE_F32, "m", "Distance between current and last LLA that triggers an update of lastLLA)");
    mapper.AddMember("magDeclination", &nvm_flash_cfg_t::magDeclination, DATA_TYPE_F32,  SYM_DEG, "Magnetic north declination (heading offset from true north)", DATA_FLAGS_FIXED_DECIMAL_2, C_RAD2DEG);
	mapper.AddMember("magInterferenceThreshold", &nvm_flash_cfg_t::magInterferenceThreshold, DATA_TYPE_F32, "", "Magnetometer interference sensitivity threshold. Typical range is 2-10 (3 default) and 1000 to disable mag interference detection.");
	mapper.AddMember("magCalibrationQualityThreshold", &nvm_flash_cfg_t::magCalibrationQualityThreshold, DATA_TYPE_F32, "", "Magnetometer calibration quality sensitivity threshold. Typical range is 10-20 (10 default) and 1000 to disable mag calibration quality check, forcing it to be always good.");
    str = "rotation from INS Sensor Frame to Intermediate Output Frame.  Order applied: heading, pitch, roll.";
    mapper.AddArray("zeroVelRotation", &nvm_flash_cfg_t::zeroVelRotation, DATA_TYPE_F32, 3, {SYM_DEG}, {"Roll, pitch, yaw " + str}, 0, C_RAD2DEG);
    str = "offset from Intermediate Output Frame to INS Output Frame.  INS rotation is applied before this.";
    mapper.AddArray("zeroVelOffset", &nvm_flash_cfg_t::zeroVelOffset, DATA_TYPE_F32, 3, {"m"}, {"X,Y,Z " + str}, 0);
    mapper.AddMember2("wheelConfig.bits", offsetof(nvm_flash_cfg_t, wheelConfig.bits), DATA_TYPE_UINT32, "", "Wheel encoder config bits: 0x1 enable encoders, 0x2 kinematic constraints", DATA_FLAGS_DISPLAY_HEX);
    mapper.AddArray2("wheelConfig.transform.e_b2w", offsetof(nvm_flash_cfg_t, wheelConfig.transform.e_b2w), DATA_TYPE_F32, 3, {"rad"}, {"Euler angles describing the rotation from imu (body) to the wheel frame (center of the non-steering axle)"}, DATA_FLAGS_FIXED_DECIMAL_2);
    mapper.AddArray2("wheelConfig.transform.e_b2w_sigma", offsetof(nvm_flash_cfg_t, wheelConfig.transform.e_b2w_sigma), DATA_TYPE_F32, 3, {"rad"}, {"Standard deviation of Euler angles describing the rotation from imu (body) to the wheel frame (center of the non-steering axle)"}, DATA_FLAGS_FIXED_DECIMAL_2);
    mapper.AddArray2("wheelConfig.transform.t_b2w", offsetof(nvm_flash_cfg_t, wheelConfig.transform.t_b2w), DATA_TYPE_F32, 3, {"m"}, {"Translation from the imu (body) to the wheel frame origin (center of the non-steering axle), expressed in the imu (body) frame"}, DATA_FLAGS_FIXED_DECIMAL_3);
    mapper.AddArray2("wheelConfig.transform.t_b2w_sigma", offsetof(nvm_flash_cfg_t, wheelConfig.transform.t_b2w_sigma), DATA_TYPE_F32, 3, {"rad"}, {"Standard deviation of translation from the imu (body) to the wheel frame origin (center of the non-steering axle), expressed in the imu (body) frame"}, DATA_FLAGS_FIXED_DECIMAL_3);
    mapper.AddMember2("wheelConfig.track_width", offsetof(nvm_flash_cfg_t, wheelConfig.track_width), DATA_TYPE_F32, "m", "Distance between the left and right wheels");
    mapper.AddMember2("wheelConfig.radius", offsetof(nvm_flash_cfg_t, wheelConfig.radius), DATA_TYPE_F32, "m", "Wheel radius");
    mapper.AddMember("debug", &nvm_flash_cfg_t::debug, DATA_TYPE_UINT8);
    mapper.AddMember("gnssCn0Minimum", &nvm_flash_cfg_t::gnssCn0Minimum, DATA_TYPE_UINT8, "dBHZ", "GNSS CN0 absolute minimum threshold for signals.  Used to filter signals in RTK solution.");
    mapper.AddMember("gnssCn0DynMinOffset", &nvm_flash_cfg_t::gnssCn0DynMinOffset, DATA_TYPE_UINT8, "dBHZ", "GNSS CN0 dynamic minimum threshold offset below max CN0 across all satellites. Used to filter signals used in RTK solution. To disable, set gnssCn0DynMinOffset to zero and increase gnssCn0Minimum.");
    mapper.AddMember("imuRejectThreshGyroLow", &nvm_flash_cfg_t::imuRejectThreshGyroLow, DATA_TYPE_UINT8, "", "IMU gyro rejection threshold.");
    mapper.AddMember("imuRejectThreshGyroHigh", &nvm_flash_cfg_t::imuRejectThreshGyroHigh, DATA_TYPE_UINT8, "", "IMU gyro rejection threshold.");
    mapper.AddMember("imuShockDetectLatencyMs", &nvm_flash_cfg_t::imuShockDetectLatencyMs, DATA_TYPE_UINT8, "ms", "IMU shock detection latency.  Time used for EKF rewind to prevent shock from influencing EKF estimates.");
    mapper.AddMember("imuShockRejectLatchMs", &nvm_flash_cfg_t::imuShockRejectLatchMs, DATA_TYPE_UINT8, "ms", "IMU shock rejection latch time.  Time required following detected end of shock to disable shock rejection.");
    mapper.AddMember("imuShockOptions", &nvm_flash_cfg_t::imuShockOptions, DATA_TYPE_UINT8, "", "IMU shock rejection options (see eImuShockOptions).", DATA_FLAGS_DISPLAY_HEX);
    mapper.AddMember("imuShockDeltaAccHighThreshold", &nvm_flash_cfg_t::imuShockDeltaAccHighThreshold, DATA_TYPE_UINT8, "m/s^2", "IMU shock detection. Min acceleration difference between the 3 IMUs to detect start of a shock.");
    mapper.AddMember("imuShockDeltaAccLowThreshold", &nvm_flash_cfg_t::imuShockDeltaAccLowThreshold, DATA_TYPE_UINT8, "m/s^2", "IMU shock detection. Max acceleration difference between the 3 IMUs within the latch time to detect end of a shock.");
    mapper.AddMember("imuShockDeltaGyroHighThreshold", &nvm_flash_cfg_t::imuShockDeltaGyroHighThreshold, DATA_TYPE_UINT8, "deg/s", "IMU shock detection. Min angular rate difference between the 3 IMUs to detect start of a shock.");
    mapper.AddMember("imuShockDeltaGyroLowThreshold", &nvm_flash_cfg_t::imuShockDeltaGyroLowThreshold, DATA_TYPE_UINT8, "deg/s", "IMU shock detection. Max angular rate difference between the 3 IMUs within the latch time to detect end of a shock.");
 
    // Keep at end
    mapper.AddMember("size", &nvm_flash_cfg_t::size, DATA_TYPE_UINT32, "", "Flash group size. Set to 1 to reset this flash group.");
    mapper.AddMember("checksum", &nvm_flash_cfg_t::checksum, DATA_TYPE_UINT32, "", "Flash checksum");
    mapper.AddMember("key", &nvm_flash_cfg_t::key, DATA_TYPE_UINT32, "", "Flash key");
}

/**
 * Maps DID_EVENT for SDK
*/
static void PopulateMapISEvent(data_set_t data_set[DID_COUNT], uint32_t did)
{
    DataMapper<did_event_t> mapper(data_set, did);
    mapper.AddMember("Time stamp of message (System Up seconds)", &did_event_t::time, DATA_TYPE_F64, "sec", "", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_1);
    mapper.AddMember("Senders serial number", &did_event_t::senderSN, DATA_TYPE_UINT32, "", "");
    mapper.AddMember("Sender hardware type", &did_event_t::senderHdwId, DATA_TYPE_UINT16, "", "Hardware: 0=Host, 1=uINS, 2=EVB, 3=IMX, 4=GPX (see eIsHardwareType)");
    mapper.AddMember("Message ID", &did_event_t::msgTypeID, DATA_TYPE_UINT16, "", "(see eEventMsgTypeID)");
    mapper.AddMember("Priority", &did_event_t::priority, DATA_TYPE_UINT8, "", "see eEventPriority");
    mapper.AddMember("Length", &did_event_t::length, DATA_TYPE_UINT16, "bytes", "");
    mapper.AddMember("data", &did_event_t::data, DATA_TYPE_STRING, "", "", DATA_FLAGS_DISPLAY_HEX);
    mapper.AddMember("Reserved 8 bit", &did_event_t::res8, DATA_TYPE_UINT8, "", "");
}

static void PopulateMapGpxFlashCfg(data_set_t data_set[DID_COUNT], uint32_t did)
{
    DataMapper<gpx_flash_cfg_t> mapper(data_set, did);
    string str;
    mapper.AddMember("ser0BaudRate", &gpx_flash_cfg_t::ser0BaudRate, DATA_TYPE_UINT32, "bps", "Serial port 0 baud rate");
    mapper.AddMember("ser1BaudRate", &gpx_flash_cfg_t::ser1BaudRate, DATA_TYPE_UINT32, "bps", "Serial port 1 baud rate");
    mapper.AddMember("ser2BaudRate", &gpx_flash_cfg_t::ser2BaudRate, DATA_TYPE_UINT32, "bps", "Serial port 2 baud rate");
    mapper.AddMember("startupGPSDtMs", &gpx_flash_cfg_t::startupGPSDtMs, DATA_TYPE_UINT32, "ms", "GPS measurement (system input data) update period in milliseconds set on startup. 200ms minimum (5Hz max).");
    str = " offset from Sensor Frame origin to GPS1 antenna.";
    mapper.AddArray("gps1AntOffset", &gpx_flash_cfg_t::gps1AntOffset, DATA_TYPE_F32, 3, {"m"}, {"X" + str, "Y" + str, "Z" + str});
    str = " offset from Sensor Frame origin to GPS2 antenna.";
    mapper.AddArray("gps2AntOffset", &gpx_flash_cfg_t::gps2AntOffset, DATA_TYPE_F32, 3, {"m"}, {"X" + str, "Y" + str, "Z" + str});
    mapper.AddMember("gnssSatSigConst", &gpx_flash_cfg_t::gnssSatSigConst, DATA_TYPE_UINT16, "", "GNSS constellations used. 0x0003=GPS, 0x000C=QZSS, 0x0030=Galileo, 0x00C0=Beidou, 0x0300=GLONASS, 0x1000=SBAS (see eGnssSatSigConst)", DATA_FLAGS_DISPLAY_HEX);
    mapper.AddMember("dynamicModel", &gpx_flash_cfg_t::dynamicModel, DATA_TYPE_UINT8, "", "0:port, 2:stationary, 3:walk, 4:ground vehicle, 5:sea, 6:air<1g, 7:air<2g, 8:air<4g, 9:wrist");
    mapper.AddMember("debug", &gpx_flash_cfg_t::debug, DATA_TYPE_UINT8, "", "Reserved", DATA_FLAGS_DISPLAY_HEX);
    mapper.AddMember("gpsTimeSyncPeriodMs", &gpx_flash_cfg_t::gpsTimeSyncPeriodMs, DATA_TYPE_UINT32, "ms", "GPS time synchronization pulse period.");
    mapper.AddMember("gpsTimeUserDelay", &gpx_flash_cfg_t::gpsTimeUserDelay, DATA_TYPE_F32, "s", "User defined delay for GPS time.  This parameter can be used to account for GPS antenna cable delay.", DATA_FLAGS_FIXED_DECIMAL_3);
    mapper.AddMember("gpsMinimumElevation", &gpx_flash_cfg_t::gpsMinimumElevation, DATA_TYPE_F32, SYM_DEG, "GPS minimum elevation of a satellite above the horizon to be used in the solution.", DATA_FLAGS_FIXED_DECIMAL_1, C_RAD2DEG);
    str = "Rover [0x1=G1, 0x2=G2], 0x8=GCompass, ";
    str += "BaseOutG1 [0x10=UbxS0, 0x20=UbxS1, 0x40=RtcmS0, 0x80=RtcmS1], ";
    str += "BaseOutG2 [0x100=UbxS0, 0x200=UbxS1, 0x400=RtcmS0, 0x800=RtcmS1], ";
    str += "0x1000=MovingBasePos, 0x4000=SameHdwRvrBase";
    mapper.AddMember("RTKCfgBits", &gpx_flash_cfg_t::RTKCfgBits, DATA_TYPE_UINT32, "", str, DATA_FLAGS_DISPLAY_HEX);
    mapper.AddMember("gnssCn0Minimum", &gpx_flash_cfg_t::gnssCn0Minimum, DATA_TYPE_UINT8, "dBHZ", "GNSS CN0 absolute minimum threshold for signals.  Used to filter signals in RTK solution.");
    mapper.AddMember("gnssCn0DynMinOffset", &gpx_flash_cfg_t::gnssCn0DynMinOffset, DATA_TYPE_UINT8, "dBHZ", "GNSS CN0 dynamic minimum threshold offset below max CN0 across all satellites. Used to filter signals used in RTK solution. To disable, set gnssCn0DynMinOffset to zero and increase gnssCn0Minimum.");
    mapper.AddArray("reserved1", &gpx_flash_cfg_t::reserved1, DATA_TYPE_UINT8, 2);
    mapper.AddMember("sysCfgBits", &gpx_flash_cfg_t::sysCfgBits, DATA_TYPE_UINT32, "", "", DATA_FLAGS_DISPLAY_HEX);
    mapper.AddMember("reserved2", &gpx_flash_cfg_t::reserved2, DATA_TYPE_UINT32);

    // Keep at end
    mapper.AddMember("size", &gpx_flash_cfg_t::size, DATA_TYPE_UINT32, "", "Flash group size. Set to 1 to reset this flash group.");
    mapper.AddMember("checksum", &gpx_flash_cfg_t::checksum, DATA_TYPE_UINT32, "", "Flash checksum");
    mapper.AddMember("key", &gpx_flash_cfg_t::key, DATA_TYPE_UINT32, "", "Flash key");
}

static void PopulateMapGpxStatus(data_set_t data_set[DID_COUNT], uint32_t did)
{
    DataMapper<gpx_status_t> mapper(data_set, did);
    mapper.AddMember("timeOfWeekMs", &gpx_status_t::timeOfWeekMs, DATA_TYPE_UINT32, "ms", "Time of week since Sunday morning", DATA_FLAGS_READ_ONLY);
    mapper.AddMember("status",       &gpx_status_t::status,       DATA_TYPE_UINT32, "", "General status flags (see eGpxStatus)", DATA_FLAGS_DISPLAY_HEX);
    mapper.AddMember("hdwStatus",    &gpx_status_t::hdwStatus,    DATA_TYPE_UINT32, "", "Hardware status flags (see eGPXHdwStatusFlags)", DATA_FLAGS_DISPLAY_HEX);
    mapper.AddMember("grmcBitsSer0", &gpx_status_t::grmcBitsSer0, DATA_TYPE_UINT64, "", "GPX RMC bit Serial 0", DATA_FLAGS_READ_ONLY | DATA_FLAGS_DISPLAY_HEX);
    mapper.AddMember("grmcBitsSer1", &gpx_status_t::grmcBitsSer1, DATA_TYPE_UINT64, "", "GPX RMC bit Serial 1", DATA_FLAGS_READ_ONLY | DATA_FLAGS_DISPLAY_HEX);
    mapper.AddMember("grmcBitsSer2", &gpx_status_t::grmcBitsSer2, DATA_TYPE_UINT64, "", "GPX RMC bit Serial 2", DATA_FLAGS_READ_ONLY | DATA_FLAGS_DISPLAY_HEX);
    mapper.AddMember("grmcBitsUSB",  &gpx_status_t::grmcBitsUSB,  DATA_TYPE_UINT64, "", "GPX RMC bit USB.",     DATA_FLAGS_READ_ONLY | DATA_FLAGS_DISPLAY_HEX);
 
    mapper.AddMember("grmcNMEABitsSer0", &gpx_status_t::grmcNMEABitsSer0, DATA_TYPE_UINT64, "", "GPX RMC NMEA bit Serial 0", DATA_FLAGS_READ_ONLY | DATA_FLAGS_DISPLAY_HEX);
    mapper.AddMember("grmcNMEABitsSer1", &gpx_status_t::grmcNMEABitsSer1, DATA_TYPE_UINT64, "", "GPX RMC NMEA bit Serial 1", DATA_FLAGS_READ_ONLY | DATA_FLAGS_DISPLAY_HEX);
    mapper.AddMember("grmcNMEABitsSer2", &gpx_status_t::grmcNMEABitsSer2, DATA_TYPE_UINT64, "", "GPX RMC NMEA bit Serial 2", DATA_FLAGS_READ_ONLY | DATA_FLAGS_DISPLAY_HEX);
    mapper.AddMember("grmcNMEABitsUSB",  &gpx_status_t::grmcNMEABitsUSB,  DATA_TYPE_UINT64, "", "GPX RMC NMEA bit USB.",     DATA_FLAGS_READ_ONLY | DATA_FLAGS_DISPLAY_HEX);
 
    mapper.AddMember("mcuTemp", &gpx_status_t::mcuTemp, DATA_TYPE_F32, SYM_DEG_C, "MCU temperature", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_1);
    mapper.AddMember("navOutputPeriodMs", &gpx_status_t::navOutputPeriodMs, DATA_TYPE_UINT32, "ms", "Nav output period (ms)", DATA_FLAGS_READ_ONLY);
    mapper.AddMember("flashCfgChecksum", &gpx_status_t::flashCfgChecksum, DATA_TYPE_UINT32, "", "Flash config validation", DATA_FLAGS_READ_ONLY);
 
    string str = "Rover [0x1=G1, 0x2=G2], 0x8=GCompass, ";
    str += "BaseOutG1 [0x10=UbxS0, 0x20=UbxS1, 0x40=RtcmS0, 0x80=RtcmS1], ";
    str += "BaseOutG2 [0x100=UbxS0, 0x200=UbxS1, 0x400=RtcmS0, 0x800=RtcmS1], ";
    str += "0x1000=MovingBasePos, 0x4000=SameHdwRvrBase";
    mapper.AddMember("rtkMode", &gpx_status_t::rtkMode, DATA_TYPE_UINT32, "", str, DATA_FLAGS_READ_ONLY | DATA_FLAGS_DISPLAY_HEX);
    for (int i=0; i<GNSS_RECEIVER_COUNT; i++)
    {
        mapper.AddMember2("gnssStatus" + std::to_string(i) + ".lastRstCause", i*sizeof(gpx_gnss_status_t) + offsetof(gpx_status_t, gnssStatus[0].lastRstCause), DATA_TYPE_UINT8, "", "GNSS last reset cause (see eGnssResetCause)");
        mapper.AddMember2("gnssStatus" + std::to_string(i) + ".fwUpdateState",  i*sizeof(gpx_gnss_status_t) + offsetof(gpx_status_t, gnssStatus[0].fwUpdateState),  DATA_TYPE_UINT8, "", "GNSS FW update status (see FirmwareUpdateState)");
        mapper.AddMember2("gnssStatus" + std::to_string(i) + ".initState",      i*sizeof(gpx_gnss_status_t) + offsetof(gpx_status_t, gnssStatus[0].initState),      DATA_TYPE_UINT8, "", "GNSS init status (see InitSteps)");
        mapper.AddMember2("gnssStatus" + std::to_string(i) + ".runState",       i*sizeof(gpx_gnss_status_t) + offsetof(gpx_status_t, gnssStatus[0].runState),       DATA_TYPE_UINT8, "", "GNSS run status (see eGPXGnssRunState)");
    }
    mapper.AddMember("gpxSourcePort", &gpx_status_t::gpxSourcePort, DATA_TYPE_UINT8, "", "Port", DATA_FLAGS_READ_ONLY);
    mapper.AddMember("upTime", &gpx_status_t::upTime, DATA_TYPE_F64, "s", "Local time since startup.", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_1);
}

static void PopulateMapSurveyIn(data_set_t data_set[DID_COUNT], uint32_t did)
{
    DataMapper<survey_in_t> mapper(data_set, did);
    string str = "[status: " + to_string(SURVEY_IN_STATE_OFF) + "=off, " + to_string(SURVEY_IN_STATE_RUNNING_3D) + "-" + to_string(SURVEY_IN_STATE_RUNNING_FIX) + "=running, " + 
        to_string(SURVEY_IN_STATE_DONE) + "=done], [cmd: " + to_string(SURVEY_IN_STATE_CANCEL) + "=cancel, start: " + to_string(SURVEY_IN_STATE_START_3D) + "=3D," + 
        to_string(SURVEY_IN_STATE_START_FLOAT) + "=float," + to_string(SURVEY_IN_STATE_START_FIX) + "=fix]";
    mapper.AddMember("state", &survey_in_t::state, DATA_TYPE_UINT32, "", str);
    mapper.AddMember("maxDurationSec", &survey_in_t::maxDurationSec, DATA_TYPE_UINT32, "s", "Maximum time survey will run if minAccuracy is not achieved.");
    mapper.AddMember("minAccuracy", &survey_in_t::minAccuracy, DATA_TYPE_F32, "m", "Required horizontal accuracy for survey to complete before maxDuration.", DATA_FLAGS_FIXED_DECIMAL_4);
    mapper.AddMember("elapsedTimeSec", &survey_in_t::elapsedTimeSec, DATA_TYPE_UINT32, "s", "Elapsed time of the survey.");
    mapper.AddMember("currentAccuracy", &survey_in_t::hAccuracy, DATA_TYPE_F32, "m", "Approximate horizontal accuracy of the survey.", DATA_FLAGS_FIXED_DECIMAL_4);
    mapper.AddLlaDegM("lla", offsetof(survey_in_t, lla), "Surveyed", "ellipsoid altitude", DATA_FLAGS_READ_ONLY);
}

static void PopulateMapEvbStatus(data_set_t data_set[DID_COUNT], uint32_t did)
{
    DataMapper<evb_status_t> mapper(data_set, did);
    mapper.AddMember("week", &evb_status_t::week, DATA_TYPE_UINT32, "week", "Weeks since Jan 6, 1980", DATA_FLAGS_READ_ONLY);
    mapper.AddMember("timeOfWeekMs", &evb_status_t::timeOfWeekMs, DATA_TYPE_UINT32, "ms", "Time of week since Sunday morning", DATA_FLAGS_READ_ONLY);
    mapper.AddArray("firmwareVer", &evb_status_t::firmwareVer, DATA_TYPE_UINT8, 4, {""}, {"Firmware version"}, DATA_FLAGS_READ_ONLY);
    mapper.AddMember("evbStatus", &evb_status_t::evbStatus, DATA_TYPE_UINT32, "", "EVB status bits", DATA_FLAGS_DISPLAY_HEX);
    mapper.AddMember("loggerMode", &evb_status_t::loggerMode, DATA_TYPE_UINT32, "", std::to_string(EVB2_LOG_CMD_START) + "=start, " + std::to_string(EVB2_LOG_CMD_STOP) + "=stop");
    mapper.AddMember("loggerElapsedTimeMs", &evb_status_t::loggerElapsedTimeMs, DATA_TYPE_UINT32, "ms", "Elapsed time of the current data log.");
    mapper.AddMember("wifiIpAddr", &evb_status_t::wifiIpAddr, DATA_TYPE_UINT32, "", "WiFi IP address", DATA_FLAGS_DISPLAY_HEX);
    mapper.AddMember("sysCommand", &evb_status_t::sysCommand, DATA_TYPE_UINT32, "", "99=software reset, 1122334455=unlock, 1357924681=chip erase");
    mapper.AddMember("towOffset", &evb_status_t::towOffset, DATA_TYPE_F64, "sec", "Time sync offset from local clock", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_5);
}

static void PopulateMapEvbFlashCfg(data_set_t data_set[DID_COUNT], uint32_t did)
{
    DataMapper<evb_flash_cfg_t> mapper(data_set, did);
    string str = to_string(EVB2_CB_PRESET_RS232) + "=Wireless Off, " 
        + to_string(EVB2_CB_PRESET_RS232_XBEE) + "=XBee On, " 
        + to_string(EVB2_CB_PRESET_RS422_WIFI) + "=WiFi On & RS422, " 
        + to_string(EVB2_CB_PRESET_USB_HUB_RS232) + "=USB hub, " 
        + to_string(EVB2_CB_PRESET_USB_HUB_RS422) + "=USB hub w/ RS422";
    mapper.AddMember("cbPreset", &evb_flash_cfg_t::cbPreset, DATA_TYPE_UINT8, "", str);
    mapper.AddArray("reserved1", &evb_flash_cfg_t::reserved1, DATA_TYPE_UINT8, 3);
    mapper.AddArray("cbf", &evb_flash_cfg_t::cbf, DATA_TYPE_UINT32, EVB2_PORT_COUNT, {""}, {"Communications bridge forwarding"}, DATA_FLAGS_DISPLAY_HEX);
    mapper.AddMember("cbOptions", &evb_flash_cfg_t::cbOptions, DATA_TYPE_UINT32, "", "Communications bridge options (see eEvb2ComBridgeOptions)", DATA_FLAGS_DISPLAY_HEX);
    mapper.AddMember("uinsComPort", &evb_flash_cfg_t::uinsComPort, DATA_TYPE_UINT8, "", "EVB port for uINS communications and SD card logging. 0=uINS0 (default), 1=uINS1, SP330=5, 6=GPIO_H8 (use eEvb2CommPorts)");
    mapper.AddMember("uinsAuxPort", &evb_flash_cfg_t::uinsAuxPort, DATA_TYPE_UINT8, "", "EVB port for uINS aux com and RTK corrections. 0=uINS0, 1=uINS1 (default), 5=SP330, 6=GPIO_H8 (use eEvb2CommPorts)");
    mapper.AddMember("portOptions", &evb_flash_cfg_t::portOptions, DATA_TYPE_UINT32, "", "EVB port options:  0x1=radio RTK filter ", DATA_FLAGS_DISPLAY_HEX);
    mapper.AddMember("bits", &evb_flash_cfg_t::bits, DATA_TYPE_UINT32, "", "Configuration bits (see eEvb2ConfigBits). 0x10=stream PPD on log button", DATA_FLAGS_DISPLAY_HEX);
    mapper.AddMember("radioPID", &evb_flash_cfg_t::radioPID, DATA_TYPE_UINT32, "", "Radio Preamble ID in hexadecimal. 0x0 to 0x9", DATA_FLAGS_DISPLAY_HEX);
    mapper.AddMember("radioNID", &evb_flash_cfg_t::radioNID, DATA_TYPE_UINT32, "", "Radio Network ID in hexadecimal. 0x0 to 0x7FFF", DATA_FLAGS_DISPLAY_HEX);
    mapper.AddMember("radioPowerLevel", &evb_flash_cfg_t::radioPowerLevel, DATA_TYPE_UINT32, "", "Radio transmitter output power level. (XBee PRO SX 0=20dBm, 1=27dBm, 2=30dBm)");

    for (int i=0; i<NUM_WIFI_PRESETS; i++)
    {
        mapper.AddMember2("wifi[0].ssid", i*sizeof(evb_wifi_t) + offsetof(evb_flash_cfg_t, wifi[0].ssid), DATA_TYPE_STRING, "", "WiFi Service Set Identifier (SSID) or network name.", 0, 1.0, WIFI_SSID_PSK_SIZE);
        mapper.AddMember2("wifi[0].psk",  i*sizeof(evb_wifi_t) + offsetof(evb_flash_cfg_t, wifi[0].psk),  DATA_TYPE_STRING, "", "WiFi Pre-Shared Key (PSK) authentication or network password.", 0, 1.0, WIFI_SSID_PSK_SIZE);
    }

    for (int i=0; i<NUM_WIFI_PRESETS; i++)
    {
        mapper.AddArray2("server" + to_string(i) + ".ipAddr", i*sizeof(evb_server_t) + offsetof(evb_flash_cfg_t, server[0].ipAddr.u8), DATA_TYPE_UINT8, 4, {""}, {"Server IP address"});
        mapper.AddMember2("server" + to_string(i) + ".port",   i*sizeof(evb_server_t) + offsetof(evb_flash_cfg_t, server[0].port), DATA_TYPE_UINT32, "", "Sever port");
    }

    mapper.AddMember("encoderTickToWheelRad", &evb_flash_cfg_t::encoderTickToWheelRad, DATA_TYPE_F32, "rad/tick", "Wheel encoder tick to wheel rotation scalar");
    mapper.AddMember("CANbaud_kbps", &evb_flash_cfg_t::CANbaud_kbps, DATA_TYPE_UINT32, "kbps", "CAN baud rate");
    mapper.AddMember("can_receive_address", &evb_flash_cfg_t::can_receive_address, DATA_TYPE_UINT32, "", "CAN Receive Address", DATA_FLAGS_DISPLAY_HEX);
    mapper.AddArray("reserved2", &evb_flash_cfg_t::reserved2, DATA_TYPE_UINT8, 2);

    mapper.AddMember("h3sp330BaudRate", &evb_flash_cfg_t::h3sp330BaudRate, DATA_TYPE_UINT32, "", "Baud rate for EVB serial port on H3 (SP330 RS233 and RS485/422).");
    mapper.AddMember("h4xRadioBaudRate", &evb_flash_cfg_t::h4xRadioBaudRate, DATA_TYPE_UINT32, "", "Baud rate for EVB serial port H4 (TLL to external radio).");
    mapper.AddMember("h8gpioBaudRate", &evb_flash_cfg_t::h8gpioBaudRate, DATA_TYPE_UINT32, "", "Baud rate for EVB serial port H8 (TLL).");
    mapper.AddMember("wheelCfgBits", &evb_flash_cfg_t::wheelCfgBits, DATA_TYPE_UINT32, "", "(eWheelCfgBits). Reverse encoder [0x100 left, 0x200 right, 0x300 both], enable [0x2 encoder, 0x4 wheel]", DATA_FLAGS_DISPLAY_HEX);
    mapper.AddMember("velocityControlPeriodMs", &evb_flash_cfg_t::velocityControlPeriodMs, DATA_TYPE_UINT32, "ms", "Wheel encoder and control update period");

    mapper.AddMember("size", &evb_flash_cfg_t::size, DATA_TYPE_UINT32, "", "Flash group size. Set to 1 to reset this flash group.");
    mapper.AddMember("checksum", &evb_flash_cfg_t::checksum, DATA_TYPE_UINT32, "", "Flash checksum");
    mapper.AddMember("key", &evb_flash_cfg_t::key, DATA_TYPE_UINT32, "", "Flash key");
}

static void PopulateMapDebugArray(data_set_t data_set[DID_COUNT], uint32_t did)
{
    DataMapper<debug_array_t> mapper(data_set, did);
    mapper.AddArray("i", &debug_array_t::i, DATA_TYPE_INT32, DEBUG_I_ARRAY_SIZE);
    mapper.AddArray("f", &debug_array_t::f, DATA_TYPE_F32, DEBUG_F_ARRAY_SIZE, {""}, {""}, DATA_FLAGS_FIXED_DECIMAL_4);
    mapper.AddArray("lf", &debug_array_t::lf, DATA_TYPE_F64, DEBUG_LF_ARRAY_SIZE, {""}, {""}, DATA_FLAGS_FIXED_DECIMAL_9);
}

static void PopulateMapDebugString(data_set_t data_set[DID_COUNT], uint32_t did)
{
    DataMapper<debug_string_t> mapper(data_set, did);
    mapper.AddMember("s", &debug_string_t::s, DATA_TYPE_STRING, "", "");
}

#if defined(INCLUDE_LUNA_DATA_SETS)

static void PopulateMapEvbLunaFlashCfg(data_set_t data_set[DID_COUNT], uint32_t did)
{
    DataMapper<evb_luna_flash_cfg_t> mapper(data_set, did); 
    mapper.AddMember("bits", bits, DATA_TYPE_UINT32, "", "0x1 geof, 0x2 bump, 0x4 prox, 0x100 rkill, 0x200 rkclient, 0x400 rkclient2", DATA_FLAGS_DISPLAY_HEX);
    mapper.AddMember("minLatGeofence", minLatGeofence, DATA_TYPE_F64, "deg", "Geofence Min Latitude");
    mapper.AddMember("maxLatGeofence", maxLatGeofence, DATA_TYPE_F64, "deg", "Geofence Max Latitude");
    mapper.AddMember("minLonGeofence", minLonGeofence, DATA_TYPE_F64, "deg", "Geofence Min Longitude");
    mapper.AddMember("maxLonGeofence", maxLonGeofence, DATA_TYPE_F64, "deg", "Geofence Max Longitude");
    mapper.AddMember("remoteKillTimeoutMs", remoteKillTimeoutMs, DATA_TYPE_UINT32, "", "");
    mapper.AddMember("bumpSensitivity", bumpSensitivity, DATA_TYPE_F32, "", "");
    mapper.AddMember("minProxDistance", minProxDistance, DATA_TYPE_F32, "", "");
    mapper.AddMember("velControl.config", velControl.config, DATA_TYPE_UINT32, "", "0=hoverbot, 1=ZTM, 2=PWM", DATA_FLAGS_DISPLAY_HEX);
    mapper.AddMember("velControl.cmdTimeoutMs", velControl.cmdTimeoutMs, DATA_TYPE_UINT32, "", "");
    mapper.AddMember("velControl.wheelRadius", velControl.wheelRadius, DATA_TYPE_F32, "rad", "Wheel radius");
    mapper.AddMember("velControl.wheelBaseline", velControl.wheelBaseline, DATA_TYPE_F32, "m", "Wheel baseline, distance between wheels");
    mapper.AddMember("velControl.engine_rpm", velControl.engine_rpm, DATA_TYPE_F32, "", "");
    mapper.AddMember("velControl.vehicle.u_min", velControl.vehicle.u_min, DATA_TYPE_F32, "m/s", "");
    mapper.AddMember("velControl.vehicle.u_cruise", velControl.vehicle.u_cruise, DATA_TYPE_F32, "m/s", "");
    mapper.AddMember("velControl.vehicle.u_max", velControl.vehicle.u_max, DATA_TYPE_F32, "m/s", "");
    mapper.AddMember("velControl.vehicle.u_slewLimit", velControl.vehicle.u_slewLimit, DATA_TYPE_F32, "m/s", "");
    mapper.AddMember("velControl.vehicle.w_max_autonomous", velControl.vehicle.w_max_autonomous, DATA_TYPE_F32, "rad/s", "");
    mapper.AddMember("velControl.vehicle.w_max", velControl.vehicle.w_max, DATA_TYPE_F32, "rad/s", "");
    mapper.AddMember("velControl.vehicle.w_slewLimit", velControl.vehicle.w_slewLimit, DATA_TYPE_F32, "rad/s", "");
    mapper.AddMember("velControl.vehicle.u_FB_Kp", velControl.vehicle.u_FB_Kp, DATA_TYPE_F32, "", "");
    mapper.AddMember("velControl.vehicle.w_FB_Kp", velControl.vehicle.w_FB_Kp, DATA_TYPE_F32, "", "");
    mapper.AddMember("velControl.vehicle.w_FB_Ki", velControl.vehicle.w_FB_Ki, DATA_TYPE_F32, "", "");
    mapper.AddMember("velControl.vehicle.w_FF_c0", velControl.vehicle.w_FF_c0, DATA_TYPE_F32, "", "");
    mapper.AddMember("velControl.vehicle.w_FF_c1", velControl.vehicle.w_FF_c1, DATA_TYPE_F32, "", "");
    mapper.AddMember("velControl.vehicle.w_FF_deadband", velControl.vehicle.w_FF_deadband, DATA_TYPE_F32, "", "");
    mapper.AddMember("velControl.vehicle.testSweepRate", velControl.vehicle.testSweepRate, DATA_TYPE_F32, "m/s/s", "");
    mapper.AddMember("velControl.wheel.slewRate", velControl.wheel.slewRate, DATA_TYPE_F32, "rad/s/s", "");
    mapper.AddMember("velControl.wheel.velMax", velControl.wheel.velMax, DATA_TYPE_F32, "rad/s", "");
    mapper.AddMember("velControl.wheel.FF_vel_deadband", velControl.wheel.FF_vel_deadband, DATA_TYPE_F32, "rad/s", "");
    mapper.AddMember("velControl.wheel.FF_c_est_Ki[0]", velControl.wheel.FF_c_est_Ki[0], DATA_TYPE_F32, "", "");
    mapper.AddMember("velControl.wheel.FF_c_est_Ki[1]", velControl.wheel.FF_c_est_Ki[1], DATA_TYPE_F32, "", "");
    mapper.AddMember("velControl.wheel.FF_c_est_max[0]", velControl.wheel.FF_c_est_max[0], DATA_TYPE_F32, "", "");
    mapper.AddMember("velControl.wheel.FF_c_est_max[1]", velControl.wheel.FF_c_est_max[1], DATA_TYPE_F32, "", "");
    mapper.AddMember("velControl.wheel.FF_c_l[0]", velControl.wheel.FF_c_l[0], DATA_TYPE_F32, "", "");
    mapper.AddMember("velControl.wheel.FF_c_l[1]", velControl.wheel.FF_c_l[1], DATA_TYPE_F32, "", "");
    mapper.AddMember("velControl.wheel.FF_c_r[0]", velControl.wheel.FF_c_r[0], DATA_TYPE_F32, "", "");
    mapper.AddMember("velControl.wheel.FF_c_r[1]", velControl.wheel.FF_c_r[1], DATA_TYPE_F32, "", "");
    mapper.AddMember("velControl.wheel.FF_FB_engine_rpm", velControl.wheel.FF_FB_engine_rpm, DATA_TYPE_F32, "", "");
    mapper.AddMember("velControl.wheel.FB_Kp", velControl.wheel.FB_Kp, DATA_TYPE_F32, "", "");
    mapper.AddMember("velControl.wheel.FB_Ki", velControl.wheel.FB_Ki, DATA_TYPE_F32, "", "");
    mapper.AddMember("velControl.wheel.FB_Kd", velControl.wheel.FB_Kd, DATA_TYPE_F32, "", "");
    mapper.AddMember("velControl.wheel.FB_gain_deadband", velControl.wheel.FB_gain_deadband, DATA_TYPE_F32, "rad/s", "");
    mapper.AddMember("velControl.wheel.FB_gain_deadband_reduction", velControl.wheel.FB_gain_deadband_reduction, DATA_TYPE_F32, "", "");
    mapper.AddMember("velControl.wheel.InversePlant_l[0]", velControl.wheel.InversePlant_l[0], DATA_TYPE_F32, "", "");
    mapper.AddMember("velControl.wheel.InversePlant_l[1]", velControl.wheel.InversePlant_l[1], DATA_TYPE_F32, "", "");
    mapper.AddMember("velControl.wheel.InversePlant_l[2]", velControl.wheel.InversePlant_l[2], DATA_TYPE_F32, "", "");
    mapper.AddMember("velControl.wheel.InversePlant_l[3]", velControl.wheel.InversePlant_l[3], DATA_TYPE_F32, "", "");
    mapper.AddMember("velControl.wheel.InversePlant_l[4]", velControl.wheel.InversePlant_l[4], DATA_TYPE_F32, "", "");
    mapper.AddMember("velControl.wheel.InversePlant_r[0]", velControl.wheel.InversePlant_r[0], DATA_TYPE_F32, "", "");
    mapper.AddMember("velControl.wheel.InversePlant_r[1]", velControl.wheel.InversePlant_r[1], DATA_TYPE_F32, "", "");
    mapper.AddMember("velControl.wheel.InversePlant_r[2]", velControl.wheel.InversePlant_r[2], DATA_TYPE_F32, "", "");
    mapper.AddMember("velControl.wheel.InversePlant_r[3]", velControl.wheel.InversePlant_r[3], DATA_TYPE_F32, "", "");
    mapper.AddMember("velControl.wheel.InversePlant_r[4]", velControl.wheel.InversePlant_r[4], DATA_TYPE_F32, "", "");
    mapper.AddMember("velControl.wheel.actuatorTrim_l", velControl.wheel.actuatorTrim_l, DATA_TYPE_F32, "", "");
    mapper.AddMember("velControl.wheel.actuatorTrim_r", velControl.wheel.actuatorTrim_r, DATA_TYPE_F32, "", "");
    mapper.AddMember("velControl.wheel.actuatorLimits_l[0]", velControl.wheel.actuatorLimits_l[0], DATA_TYPE_F32, "", "");
    mapper.AddMember("velControl.wheel.actuatorLimits_l[1]", velControl.wheel.actuatorLimits_l[1], DATA_TYPE_F32, "", "");
    mapper.AddMember("velControl.wheel.actuatorLimits_r[0]", velControl.wheel.actuatorLimits_r[0], DATA_TYPE_F32, "", "");
    mapper.AddMember("velControl.wheel.actuatorLimits_r[1]", velControl.wheel.actuatorLimits_r[1], DATA_TYPE_F32, "", "");
    mapper.AddMember("velControl.wheel.actuatorDeadbandDuty_l", velControl.wheel.actuatorDeadbandDuty_l, DATA_TYPE_F32, "", "");
    mapper.AddMember("velControl.wheel.actuatorDeadbandDuty_r", velControl.wheel.actuatorDeadbandDuty_r, DATA_TYPE_F32, "", "");
    mapper.AddMember("velControl.wheel.actuatorDeadbandVel", velControl.wheel.actuatorDeadbandVel, DATA_TYPE_F32, "", "");

    mapper.AddMember("size", size, DATA_TYPE_UINT32, "", "Flash group size. Set to 1 to reset this flash group.");
    mapper.AddMember("checksum", checksum, DATA_TYPE_UINT32, "", "Flash checksum");
    mapper.AddMember("key", key, DATA_TYPE_UINT32, "", "Flash key");
}

static void PopulateMapCoyoteStatus(data_set_t data_set[DID_COUNT], uint32_t did)
{
    DataMapper<evb_luna_status_t> mapper(data_set, did);
    mapper.AddMember("timeOfWeekMs", timeOfWeekMs, DATA_TYPE_UINT32, "ms", "GPS time of week (since Sunday morning).");
    mapper.AddMember("evbLunaStatus", evbLunaStatus, DATA_TYPE_UINT32, "", "", DATA_FLAGS_DISPLAY_HEX);
    mapper.AddMember("motorState", motorState, DATA_TYPE_UINT32, "", "");
    mapper.AddMember("remoteKillMode", remoteKillMode, DATA_TYPE_UINT32, "", "Motor state (eLunaMotorState)");
    mapper.AddMember("supplyVoltage", supplyVoltage, DATA_TYPE_F32, "V", "", DATA_FLAGS_FIXED_DECIMAL_1);
}

static void PopulateMapEvbLunaSensors(data_set_t data_set[DID_COUNT], uint32_t did)
{
    DataMapper<evb_luna_sensors_t> mapper(data_set, did);
    mapper.AddMember("timeOfWeekMs", timeOfWeekMs, DATA_TYPE_UINT32, "s", "GPS time of week (since Sunday morning).");
    mapper.AddMember("proxSensorOutput[0]", proxSensorOutput[0], DATA_TYPE_F32, "", "", DATA_FLAGS_FIXED_DECIMAL_4);
    mapper.AddMember("proxSensorOutput[1]", proxSensorOutput[1], DATA_TYPE_F32, "", "", DATA_FLAGS_FIXED_DECIMAL_4);
    mapper.AddMember("proxSensorOutput[2]", proxSensorOutput[2], DATA_TYPE_F32, "", "", DATA_FLAGS_FIXED_DECIMAL_4);
    mapper.AddMember("proxSensorOutput[3]", proxSensorOutput[3], DATA_TYPE_F32, "", "", DATA_FLAGS_FIXED_DECIMAL_4);
    mapper.AddMember("proxSensorOutput[4]", proxSensorOutput[4], DATA_TYPE_F32, "", "", DATA_FLAGS_FIXED_DECIMAL_4);
    mapper.AddMember("proxSensorOutput[5]", proxSensorOutput[5], DATA_TYPE_F32, "", "", DATA_FLAGS_FIXED_DECIMAL_4);
    mapper.AddMember("proxSensorOutput[6]", proxSensorOutput[6], DATA_TYPE_F32, "", "", DATA_FLAGS_FIXED_DECIMAL_4);
    mapper.AddMember("proxSensorOutput[7]", proxSensorOutput[7], DATA_TYPE_F32, "", "", DATA_FLAGS_FIXED_DECIMAL_4);
    mapper.AddMember("proxSensorOutput[8]", proxSensorOutput[8], DATA_TYPE_F32, "", "", DATA_FLAGS_FIXED_DECIMAL_4);
    mapper.AddMember("bumpEvent", bumpEvent, DATA_TYPE_INT32, int32_t);
}

static void PopulateMapEvbLunaVelocityControl(data_set_t data_set[DID_COUNT], uint32_t did)
{
    DataMapper<evb_luna_velocity_control_t> mapper(data_set, did);
    mapper.AddMember("timeMs", timeMs, DATA_TYPE_UINT32, "ms", "");
    mapper.AddMember("dt", dt, DATA_TYPE_F32, "s", "", DATA_FLAGS_FIXED_DECIMAL_3);
    mapper.AddMember("status", status, DATA_TYPE_UINT32, "", "", DATA_FLAGS_DISPLAY_HEX);
    mapper.AddMember("current_mode", current_mode, DATA_TYPE_UINT32, "", "0 disable, 1 stop, 2 enable, test vel[3 dual, 4 single, 5 sweep], 6 test eff, test duty [7 single, 8 sweep] (eLunaWheelControllerMode)");
    mapper.AddMember("vehicle.velCmd_f", vehicle.velCmd_f, DATA_TYPE_F32, "m/s", "", DATA_FLAGS_FIXED_DECIMAL_4);
    mapper.AddMember("vehicle.velCmd_w", vehicle.velCmd_w, DATA_TYPE_F32, "rad/s", "", DATA_FLAGS_FIXED_DECIMAL_4);
    mapper.AddMember("vehicle.velCmdMnl_f", vehicle.velCmdMnl_f, DATA_TYPE_F32, "m/s", "", DATA_FLAGS_FIXED_DECIMAL_4);
    mapper.AddMember("vehicle.velCmdMnl_w", vehicle.velCmdMnl_w, DATA_TYPE_F32, "rad/s", "", DATA_FLAGS_FIXED_DECIMAL_4);
    mapper.AddMember("vehicle.velCmdSlew_f", vehicle.velCmdSlew_f, DATA_TYPE_F32, "m/s", "", DATA_FLAGS_FIXED_DECIMAL_4);
    mapper.AddMember("vehicle.velCmdSlew_w", vehicle.velCmdSlew_w, DATA_TYPE_F32, "rad/s", "", DATA_FLAGS_FIXED_DECIMAL_4);
    mapper.AddMember("vehicle.vel_f", vehicle.vel_f, DATA_TYPE_F32, "m/s", "", DATA_FLAGS_FIXED_DECIMAL_4);
    mapper.AddMember("vehicle.vel_w", vehicle.vel_w, DATA_TYPE_F32, "rad/s", "", DATA_FLAGS_FIXED_DECIMAL_4);
    mapper.AddMember("vehicle.err_f", vehicle.err_f, DATA_TYPE_F32, "m/s", "", DATA_FLAGS_FIXED_DECIMAL_4);
    mapper.AddMember("vehicle.err_w", vehicle.err_w, DATA_TYPE_F32, "rad/s", "", DATA_FLAGS_FIXED_DECIMAL_4);
    mapper.AddMember("vehicle.eff_f", vehicle.eff_f, DATA_TYPE_F32, "m/s", "", DATA_FLAGS_FIXED_DECIMAL_4);
    mapper.AddMember("vehicle.eff_w", vehicle.eff_w, DATA_TYPE_F32, "rad/s", "", DATA_FLAGS_FIXED_DECIMAL_4);
    mapper.AddMember("wheel_l.velCmd", wheel_l.velCmd, DATA_TYPE_F32, "rad/s", "", DATA_FLAGS_FIXED_DECIMAL_4);
    mapper.AddMember("wheel_l.velCmdSlew", wheel_l.velCmdSlew, DATA_TYPE_F32, "rad/s", "", DATA_FLAGS_FIXED_DECIMAL_4);
    mapper.AddMember("wheel_l.vel", wheel_l.vel, DATA_TYPE_F32, "rad/s", "", DATA_FLAGS_FIXED_DECIMAL_4);
    mapper.AddMember("wheel_l.err", wheel_l.err, DATA_TYPE_F32, "rad/s", "", DATA_FLAGS_FIXED_DECIMAL_4);
    mapper.AddMember("wheel_l.ff_eff", wheel_l.ff_eff, DATA_TYPE_F32, "rad", "", DATA_FLAGS_FIXED_DECIMAL_4);
    mapper.AddMember("wheel_l.fb_eff", wheel_l.fb_eff, DATA_TYPE_F32, "rad", "", DATA_FLAGS_FIXED_DECIMAL_4);
    mapper.AddMember("wheel_l.eff", wheel_l.eff, DATA_TYPE_F32, "rad", "Control effort = ff_eff_l + fb_eff", DATA_FLAGS_FIXED_DECIMAL_4);
    mapper.AddMember("wheel_l.effInt", wheel_l.effInt, DATA_TYPE_F32, "rad", "Control effort intermediate", DATA_FLAGS_FIXED_DECIMAL_4);
    mapper.AddMember("wheel_l.effDuty", wheel_l.effDuty, DATA_TYPE_F32, "%", "Duty cycle 0-100", DATA_FLAGS_FIXED_DECIMAL_2);
    mapper.AddMember("wheel_r.velCmd", wheel_r.velCmd, DATA_TYPE_F32, "rad/s", "", DATA_FLAGS_FIXED_DECIMAL_4);
    mapper.AddMember("wheel_r.velCmdSlew", wheel_r.velCmdSlew, DATA_TYPE_F32, "rad/s", "", DATA_FLAGS_FIXED_DECIMAL_4);
    mapper.AddMember("wheel_r.vel", wheel_r.vel, DATA_TYPE_F32, "rad/s", "", DATA_FLAGS_FIXED_DECIMAL_4);
    mapper.AddMember("wheel_r.err", wheel_r.err, DATA_TYPE_F32, "rad/s", "", DATA_FLAGS_FIXED_DECIMAL_4);
    mapper.AddMember("wheel_r.ff_eff", wheel_r.ff_eff, DATA_TYPE_F32, "rad", "", DATA_FLAGS_FIXED_DECIMAL_4);
    mapper.AddMember("wheel_r.fb_eff", wheel_r.fb_eff, DATA_TYPE_F32, "rad", "", DATA_FLAGS_FIXED_DECIMAL_4);
    mapper.AddMember("wheel_r.eff", wheel_r.eff, DATA_TYPE_F32, "rad", "Control effort = ff_eff_l + fb_eff", DATA_FLAGS_FIXED_DECIMAL_4);
    mapper.AddMember("wheel_r.effInt", wheel_r.effInt, DATA_TYPE_F32, "rad", "Control effort intermediate", DATA_FLAGS_FIXED_DECIMAL_4);
    mapper.AddMember("wheel_r.effDuty", wheel_r.effDuty, DATA_TYPE_F32, "%", "Duty cycle 0-100", DATA_FLAGS_FIXED_DECIMAL_2);
    mapper.AddMember("potV_l", potV_l, DATA_TYPE_F32, "V", "Left potentiometer input", DATA_FLAGS_FIXED_DECIMAL_3);
    mapper.AddMember("potV_r", potV_r, DATA_TYPE_F32, "V", "Right potentiometer input", DATA_FLAGS_FIXED_DECIMAL_3);
}

static void PopulateMapEvbLunaVelocityCommand(data_set_t data_set[DID_COUNT], uint32_t did)
{
    DataMapper<evb_luna_velocity_command_t> mapper(data_set, did);
    mapper.AddMember("timeMs", timeMs, DATA_TYPE_UINT32, "ms", "");
    mapper.AddMember("modeCmd", modeCmd, DATA_TYPE_UINT32, "", "0 disable, 1 stop, 2 enable, 3 enable w/o watchdog (eLunaVelocityControlMode)");
    mapper.AddMember("fwd_vel", fwd_vel, DATA_TYPE_F32, "rad/s", "", DATA_FLAGS_FIXED_DECIMAL_4);
    mapper.AddMember("turn_rate", turn_rate, DATA_TYPE_F32, "rad/s", "", DATA_FLAGS_FIXED_DECIMAL_4);
}

static void PopulateMapEvbLunaAuxCmd(data_set_t data_set[DID_COUNT], uint32_t did)
{
    DataMapper<evb_luna_aux_command_t> mapper(data_set, did);
    
    mapper.AddMember("command", command, DATA_TYPE_UINT32, "", "0 blade off, 1 blade on, 2 ebrake on, 3 ebrake off, 4 beep (eLunaAuxCommands)");

}

#endif

static void PopulateMapGpsRtkRel(data_set_t data_set[DID_COUNT], uint32_t did)
{
    DataMapper<gps_rtk_rel_t> mapper(data_set, did);
    mapper.AddMember("timeOfWeekMs", &gps_rtk_rel_t::timeOfWeekMs, DATA_TYPE_UINT32,  "ms", "Time of week since Sunday morning", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_4);
    mapper.AddArray("baseToRoverVector", &gps_rtk_rel_t::baseToRoverVector, DATA_TYPE_F32, 3, {"m"}, {"Vector from base to rover in ECEF."}, DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_2);
    mapper.AddMember("differentialAge", &gps_rtk_rel_t::differentialAge, DATA_TYPE_F32, "s", "Age of differential signal received.", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3);
    mapper.AddMember("arRatio", &gps_rtk_rel_t::arRatio, DATA_TYPE_F32, "", "Ambiguity resolution ratio factor for validation.", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_1);
    mapper.AddMember("baseToRoverDistance", &gps_rtk_rel_t::baseToRoverDistance, DATA_TYPE_F32, "", "baseToRoverDistance (m)", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3);
    mapper.AddMember("baseToRoverHeading", &gps_rtk_rel_t::baseToRoverHeading, DATA_TYPE_F32, SYM_DEG, "Angle from north to baseToRoverVector in local tangent plane.", DATA_FLAGS_READ_ONLY | DATA_FLAGS_ANGLE | DATA_FLAGS_FIXED_DECIMAL_4, C_RAD2DEG);
    mapper.AddMember("baseToRoverHeadingAcc", &gps_rtk_rel_t::baseToRoverHeadingAcc, DATA_TYPE_F32, SYM_DEG, "Accuracy of baseToRoverHeading.", DATA_FLAGS_READ_ONLY | DATA_FLAGS_ANGLE | DATA_FLAGS_FIXED_DECIMAL_6, C_RAD2DEG);
    mapper.AddMember("status", &gps_rtk_rel_t::status, DATA_TYPE_UINT32, "", "GPS status: [0x000000xx] number of satellites used, [0x0000xx00] fix type, [0x00xx0000] status flags", DATA_FLAGS_READ_ONLY | DATA_FLAGS_DISPLAY_HEX | DATA_FLAGS_GPS_STATUS);
}

static void PopulateMapGpsRtkMisc(data_set_t data_set[DID_COUNT], uint32_t did)
{
    DataMapper<gps_rtk_misc_t> mapper(data_set, did);
    mapper.AddMember("timeOfWeekMs", &gps_rtk_misc_t::timeOfWeekMs, DATA_TYPE_UINT32, "ms", "Time of week since Sunday morning", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_4);
    mapper.AddMember("timeToFirstFixMs", &gps_rtk_misc_t::timeToFirstFixMs, DATA_TYPE_UINT32, "ms", "Time to first RTK fix", DATA_FLAGS_READ_ONLY);
    mapper.AddArray("accuracyPos", &gps_rtk_misc_t::accuracyPos, DATA_TYPE_F32, 3, {"m"}, {"Accuracy in meters north, east, up (standard deviation)"}, DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_4);
    mapper.AddArray("accuracyCov", &gps_rtk_misc_t::accuracyCov, DATA_TYPE_F32, 3, {"m"}, {"Absolute value of means square root of estimated covariance NE, EU, UN"}, DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_4);
    mapper.AddMember("arThreshold", &gps_rtk_misc_t::arThreshold, DATA_TYPE_F32, "", "Ambiguity resolution threshold for validation", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3);
    mapper.AddMember("gDop", &gps_rtk_misc_t::gDop, DATA_TYPE_F32, "m", "Geomatic dilution of precision", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_4);
    mapper.AddMember("hDop", &gps_rtk_misc_t::hDop, DATA_TYPE_F32, "m", "Horizontal dilution of precision", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_4);
    mapper.AddMember("vDop", &gps_rtk_misc_t::vDop, DATA_TYPE_F32, "m", "Vertical dilution of precision", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_4);
    mapper.AddLlaDegM("baseLla", offsetof(gps_rtk_misc_t, baseLla), "Base position", "altitude", DATA_FLAGS_READ_ONLY);
    mapper.AddMember("cycleSlipCount", &gps_rtk_misc_t::cycleSlipCount, DATA_TYPE_UINT32, "int", "Cycle slip counter", DATA_FLAGS_READ_ONLY);

    mapper.AddMember("roverGpsObservationCount", &gps_rtk_misc_t::roverGpsObservationCount, DATA_TYPE_UINT32, "int", "Rover gps observation element counter", DATA_FLAGS_READ_ONLY);
    mapper.AddMember("baseGpsObservationCount", &gps_rtk_misc_t::baseGpsObservationCount, DATA_TYPE_UINT32, "int", "Base gps observation element counter", DATA_FLAGS_READ_ONLY);
    mapper.AddMember("roverGlonassObservationCount", &gps_rtk_misc_t::roverGlonassObservationCount, DATA_TYPE_UINT32, "int", "Rover glonass observation element counter", DATA_FLAGS_READ_ONLY);
    mapper.AddMember("baseGlonassObservationCount", &gps_rtk_misc_t::baseGlonassObservationCount, DATA_TYPE_UINT32, "int", "Base glonass observation element counter", DATA_FLAGS_READ_ONLY);

    mapper.AddMember("roverGalileoObservationCount", &gps_rtk_misc_t::roverGalileoObservationCount, DATA_TYPE_UINT32, "int", "Rover galileo observation element counter", DATA_FLAGS_READ_ONLY);
    mapper.AddMember("baseGalileoObservationCount", &gps_rtk_misc_t::baseGalileoObservationCount, DATA_TYPE_UINT32, "int", "Base galileo observation element counter", DATA_FLAGS_READ_ONLY);
    mapper.AddMember("roverBeidouObservationCount", &gps_rtk_misc_t::roverBeidouObservationCount, DATA_TYPE_UINT32, "int", "Rover beidou observation element counter", DATA_FLAGS_READ_ONLY);
    mapper.AddMember("baseBeidouObservationCount", &gps_rtk_misc_t::baseBeidouObservationCount, DATA_TYPE_UINT32, "int", "Base beidou observation element counter", DATA_FLAGS_READ_ONLY);

    mapper.AddMember("roverQzsObservationCount", &gps_rtk_misc_t::roverQzsObservationCount, DATA_TYPE_UINT32, "int", "Rover qzs observation element counter", DATA_FLAGS_READ_ONLY);
    mapper.AddMember("baseQzsObservationCount", &gps_rtk_misc_t::baseQzsObservationCount, DATA_TYPE_UINT32, "int", "Base qzs observation element counter", DATA_FLAGS_READ_ONLY);
    mapper.AddMember("roverGpsEphemerisCount", &gps_rtk_misc_t::roverGpsEphemerisCount, DATA_TYPE_UINT32, "int", "Rover gps ephemeris element counter", DATA_FLAGS_READ_ONLY);
    mapper.AddMember("baseGpsEphemerisCount", &gps_rtk_misc_t::baseGpsEphemerisCount, DATA_TYPE_UINT32, "int", "Base gps ephemeris element counter", DATA_FLAGS_READ_ONLY);

    mapper.AddMember("roverGlonassEphemerisCount", &gps_rtk_misc_t::roverGlonassEphemerisCount, DATA_TYPE_UINT32, "int", "Rover glonass ephemeris element counter", DATA_FLAGS_READ_ONLY);
    mapper.AddMember("baseGlonassEphemerisCount", &gps_rtk_misc_t::baseGlonassEphemerisCount, DATA_TYPE_UINT32, "int", "Base glonass ephemeris element counter", DATA_FLAGS_READ_ONLY);
    mapper.AddMember("roverGalileoEphemerisCount", &gps_rtk_misc_t::roverGalileoEphemerisCount, DATA_TYPE_UINT32, "int", "Rover galileo ephemeris element counter", DATA_FLAGS_READ_ONLY);
    mapper.AddMember("baseGalileoEphemerisCount", &gps_rtk_misc_t::baseGalileoEphemerisCount, DATA_TYPE_UINT32, "int", "Base galileo ephemeris element counter", DATA_FLAGS_READ_ONLY);

    mapper.AddMember("roverBeidouEphemerisCount", &gps_rtk_misc_t::roverBeidouEphemerisCount, DATA_TYPE_UINT32, "int", "Rover beidou ephemeris element counter", DATA_FLAGS_READ_ONLY);
    mapper.AddMember("baseBeidouEphemerisCount", &gps_rtk_misc_t::baseBeidouEphemerisCount, DATA_TYPE_UINT32, "int", "Base beidou ephemeris element counter", DATA_FLAGS_READ_ONLY);
    mapper.AddMember("roverQzsEphemerisCount", &gps_rtk_misc_t::roverQzsEphemerisCount, DATA_TYPE_UINT32, "int", "Rover qzs ephemeris element counter", DATA_FLAGS_READ_ONLY);
    mapper.AddMember("baseQzsEphemerisCount", &gps_rtk_misc_t::baseQzsEphemerisCount, DATA_TYPE_UINT32, "int", "Base qzs ephemeris element counter", DATA_FLAGS_READ_ONLY);

    mapper.AddMember("roverSbasCount", &gps_rtk_misc_t::roverSbasCount, DATA_TYPE_UINT32, "int", "Rover sbas element counter", DATA_FLAGS_READ_ONLY);
    mapper.AddMember("baseSbasCount", &gps_rtk_misc_t::baseSbasCount, DATA_TYPE_UINT32, "int", "Base sbas element counter", DATA_FLAGS_READ_ONLY);
    mapper.AddMember("baseAntennaCount", &gps_rtk_misc_t::baseAntennaCount, DATA_TYPE_UINT32, "int", "Base antenna position element counter", DATA_FLAGS_READ_ONLY);
    mapper.AddMember("ionUtcAlmCount", &gps_rtk_misc_t::ionUtcAlmCount, DATA_TYPE_UINT32, "int", "Ion model / utc / alm element counter", DATA_FLAGS_READ_ONLY);

    mapper.AddMember("correctionChecksumFailures", &gps_rtk_misc_t::correctionChecksumFailures, DATA_TYPE_UINT32, "int", "Correction input checksum failures", DATA_FLAGS_READ_ONLY);
}

static void PopulateMapGpsRaw(data_set_t data_set[DID_COUNT], uint32_t did)
{
    DataMapper<gps_raw_t> mapper(data_set, did);
    mapper.AddMember("receiveIndex", &gps_raw_t::receiverIndex, DATA_TYPE_UINT8, "", "Receiver index (1=Rover, 2=Base). RTK positioning or RTK compassing must be enabled to stream raw GPS data.", DATA_FLAGS_READ_ONLY);
    mapper.AddMember("dataType", &gps_raw_t::dataType, DATA_TYPE_UINT8, "", "Type of data (eRawDataType: 1=observations, 2=ephemeris, 3=glonassEphemeris, 4=SBAS, 5=baseAntenna, 6=IonosphereModel)", DATA_FLAGS_READ_ONLY);
    mapper.AddMember("obsCount", &gps_raw_t::obsCount, DATA_TYPE_UINT8, "", "Number of observations in array (obsd_t) when dataType==1", DATA_FLAGS_READ_ONLY);
    mapper.AddMember("reserved", &gps_raw_t::reserved, DATA_TYPE_UINT8, "", "Reserved", DATA_FLAGS_READ_ONLY);
    mapper.AddMember2("dataBuf", offsetof(gps_raw_t, data.buf), DATA_TYPE_BINARY, "", "", 0, 1.0, GPS_RAW_MESSAGE_BUF_SIZE);
}

static void PopulateMapStrobeInTime(data_set_t data_set[DID_COUNT], uint32_t did)
{
    DataMapper<strobe_in_time_t> mapper(data_set, did);
    mapper.AddMember("week", &strobe_in_time_t::week, DATA_TYPE_UINT32, "week", "Weeks since Jan 6, 1980", DATA_FLAGS_READ_ONLY);
    mapper.AddMember("timeOfWeekMs", &strobe_in_time_t::timeOfWeekMs, DATA_TYPE_UINT32, "ms", "Time of week since Sunday morning", DATA_FLAGS_READ_ONLY);
    mapper.AddMember("pin", &strobe_in_time_t::pin, DATA_TYPE_UINT16, "", "STROBE input pin number", DATA_FLAGS_READ_ONLY);
    mapper.AddMember("count", &strobe_in_time_t::count, DATA_TYPE_UINT16, "", "STROBE input serial index number", DATA_FLAGS_READ_ONLY);
}

static void PopulateMapRtosInfo(data_set_t data_set[DID_COUNT], uint32_t did)
{
    DataMapper<rtos_info_t> mapper(data_set, did);
    mapper.AddMember("freeHeapSize", &rtos_info_t::freeHeapSize, DATA_TYPE_UINT32, "", "Heap unused bytes (high-water mark)", DATA_FLAGS_READ_ONLY);
    mapper.AddMember("mallocSize", &rtos_info_t::mallocSize, DATA_TYPE_UINT32, "", "Total memory allocated using malloc()", DATA_FLAGS_READ_ONLY);
    mapper.AddMember("freeSize", &rtos_info_t::freeSize, DATA_TYPE_UINT32, "", "Total memory freed using free()", DATA_FLAGS_READ_ONLY);

    for (int i=0; i<IMX_RTOS_NUM_TASKS; i++) { mapper.AddMember2("T" + to_string(i) + ".name",                 i*sizeof(rtos_task_t) + offsetof(rtos_info_t, task[0].name), DATA_TYPE_STRING, "", "Task name", 0, 1.0, MAX_TASK_NAME_LEN); }
    for (int i=0; i<IMX_RTOS_NUM_TASKS; i++) { mapper.AddMember2("T" + to_string(i) + ".cpuUsage",             i*sizeof(rtos_task_t) + offsetof(rtos_info_t, task[0].cpuUsage), DATA_TYPE_F32, "%", "CPU usage", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_1); }
    for (int i=0; i<IMX_RTOS_NUM_TASKS; i++) { mapper.AddMember2("T" + to_string(i) + ".stackUnused",          i*sizeof(rtos_task_t) + offsetof(rtos_info_t, task[0].stackUnused), DATA_TYPE_UINT32, "", "Task stack unused bytes (high-water mark)"); }
    for (int i=0; i<IMX_RTOS_NUM_TASKS; i++) { mapper.AddMember2("T" + to_string(i) + ".priority",             i*sizeof(rtos_task_t) + offsetof(rtos_info_t, task[0].priority), DATA_TYPE_UINT32, "", "Task priority"); }
    for (int i=0; i<IMX_RTOS_NUM_TASKS; i++) { mapper.AddMember2("T" + to_string(i) + ".periodMs",             i*sizeof(rtos_task_t) + offsetof(rtos_info_t, task[0].periodMs), DATA_TYPE_UINT32, "ms", "Task period", DATA_FLAGS_READ_ONLY); }
    for (int i=0; i<IMX_RTOS_NUM_TASKS; i++) { mapper.AddMember2("T" + to_string(i) + ".runtimeUs",            i*sizeof(rtos_task_t) + offsetof(rtos_info_t, task[0].runtimeUs), DATA_TYPE_UINT32, "us", "Task execution time"); }
    for (int i=0; i<IMX_RTOS_NUM_TASKS; i++) { mapper.AddMember2("T" + to_string(i) + ".avgRuntimeUs",         i*sizeof(rtos_task_t) + offsetof(rtos_info_t, task[0].avgRuntimeUs), DATA_TYPE_F32, "us", "Average runtime", DATA_FLAGS_FIXED_DECIMAL_0); }
    for (int i=0; i<IMX_RTOS_NUM_TASKS; i++) { mapper.AddMember2("T" + to_string(i) + ".avgLowerRuntimeUs",    i*sizeof(rtos_task_t) + offsetof(rtos_info_t, task[0].lowerRuntimeUs), DATA_TYPE_F32, "us", "Average of runtimes less than avgRuntimeUs", DATA_FLAGS_FIXED_DECIMAL_0); }
    for (int i=0; i<IMX_RTOS_NUM_TASKS; i++) { mapper.AddMember2("T" + to_string(i) + ".avgUpperRuntimeUs",    i*sizeof(rtos_task_t) + offsetof(rtos_info_t, task[0].upperRuntimeUs), DATA_TYPE_F32, "us", "Average of runtimes greater than avgRuntimeUs", DATA_FLAGS_FIXED_DECIMAL_0); }
    for (int i=0; i<IMX_RTOS_NUM_TASKS; i++) { mapper.AddMember2("T" + to_string(i) + ".maxRuntimeUs",         i*sizeof(rtos_task_t) + offsetof(rtos_info_t, task[0].maxRuntimeUs), DATA_TYPE_UINT32, "us", "Task max execution time"); }
    for (int i=0; i<IMX_RTOS_NUM_TASKS; i++) { mapper.AddMember2("T" + to_string(i) + ".startTimeUs",          i*sizeof(rtos_task_t) + offsetof(rtos_info_t, task[0].startTimeUs), DATA_TYPE_UINT32, "us", ""); }
    for (int i=0; i<IMX_RTOS_NUM_TASKS; i++) { mapper.AddMember2("T" + to_string(i) + ".gapCount",             i*sizeof(rtos_task_t) + offsetof(rtos_info_t, task[0].gapCount), DATA_TYPE_UINT16, "", "Number of times task took too long"); }
    for (int i=0; i<IMX_RTOS_NUM_TASKS; i++) { mapper.AddMember2("T" + to_string(i) + ".doubleGapCount",       i*sizeof(rtos_task_t) + offsetof(rtos_info_t, task[0].doubleGapCount), DATA_TYPE_UINT8, "", "Number of times task took too long twice in a row"); }
    for (int i=0; i<IMX_RTOS_NUM_TASKS; i++) { mapper.AddMember2("T" + to_string(i) + ".reserved",             i*sizeof(rtos_task_t) + offsetof(rtos_info_t, task[0].reserved), DATA_TYPE_UINT8, "", "", DATA_FLAGS_HIDDEN); }
    for (int i=0; i<IMX_RTOS_NUM_TASKS; i++) { mapper.AddMember2("T" + to_string(i) + ".handle",               i*sizeof(rtos_task_t) + offsetof(rtos_info_t, task[0].handle), DATA_TYPE_UINT32); }
}

static void PopulateMapGpxRtosInfo(data_set_t data_set[DID_COUNT], uint32_t did)
{
    DataMapper<gpx_rtos_info_t> mapper(data_set, did);
    mapper.AddMember("freeHeapSize", &gpx_rtos_info_t::freeHeapSize, DATA_TYPE_UINT32, "", "Heap unused bytes (high-water mark)", DATA_FLAGS_READ_ONLY);
    mapper.AddMember("mallocSize", &gpx_rtos_info_t::mallocSize, DATA_TYPE_UINT32, "", "Total memory allocated using malloc()", DATA_FLAGS_READ_ONLY);
    mapper.AddMember("freeSize", &gpx_rtos_info_t::freeSize, DATA_TYPE_UINT32, "", "Total memory freed using free()", DATA_FLAGS_READ_ONLY);


    for (int i=0; i<GPX_RTOS_NUM_TASKS; i++) { mapper.AddMember2("T" + to_string(i) + ".name",                 i*sizeof(rtos_task_t) + offsetof(rtos_info_t, task[0].name), DATA_TYPE_STRING, "", "Task name", 0, 1.0, MAX_TASK_NAME_LEN); }
    for (int i=0; i<GPX_RTOS_NUM_TASKS; i++) { mapper.AddMember2("T" + to_string(i) + ".cpuUsage",             i*sizeof(rtos_task_t) + offsetof(rtos_info_t, task[0].cpuUsage), DATA_TYPE_F32, "%", "CPU usage", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_1); }
    for (int i=0; i<GPX_RTOS_NUM_TASKS; i++) { mapper.AddMember2("T" + to_string(i) + ".stackUnused",          i*sizeof(rtos_task_t) + offsetof(rtos_info_t, task[0].stackUnused), DATA_TYPE_UINT32, "", "Task stack unused bytes (high-water mark)"); }
    for (int i=0; i<GPX_RTOS_NUM_TASKS; i++) { mapper.AddMember2("T" + to_string(i) + ".priority",             i*sizeof(rtos_task_t) + offsetof(rtos_info_t, task[0].priority), DATA_TYPE_UINT32, "", "Task priority"); }
    for (int i=0; i<GPX_RTOS_NUM_TASKS; i++) { mapper.AddMember2("T" + to_string(i) + ".periodMs",             i*sizeof(rtos_task_t) + offsetof(rtos_info_t, task[0].periodMs), DATA_TYPE_UINT32, "ms", "Task period", DATA_FLAGS_READ_ONLY); }
    for (int i=0; i<GPX_RTOS_NUM_TASKS; i++) { mapper.AddMember2("T" + to_string(i) + ".runtimeUs",            i*sizeof(rtos_task_t) + offsetof(rtos_info_t, task[0].runtimeUs), DATA_TYPE_UINT32, "us", "Task execution time"); }
    for (int i=0; i<GPX_RTOS_NUM_TASKS; i++) { mapper.AddMember2("T" + to_string(i) + ".avgRuntimeUs",         i*sizeof(rtos_task_t) + offsetof(rtos_info_t, task[0].avgRuntimeUs), DATA_TYPE_F32, "us", "Average runtime", DATA_FLAGS_FIXED_DECIMAL_0); }
    for (int i=0; i<GPX_RTOS_NUM_TASKS; i++) { mapper.AddMember2("T" + to_string(i) + ".avgLowerRuntimeUs",    i*sizeof(rtos_task_t) + offsetof(rtos_info_t, task[0].lowerRuntimeUs), DATA_TYPE_F32, "us", "Average of runtimes less than avgRuntimeUs", DATA_FLAGS_FIXED_DECIMAL_0); }
    for (int i=0; i<GPX_RTOS_NUM_TASKS; i++) { mapper.AddMember2("T" + to_string(i) + ".avgUpperRuntimeUs",    i*sizeof(rtos_task_t) + offsetof(rtos_info_t, task[0].upperRuntimeUs), DATA_TYPE_F32, "us", "Average of runtimes greater than avgRuntimeUs", DATA_FLAGS_FIXED_DECIMAL_0); }
    for (int i=0; i<GPX_RTOS_NUM_TASKS; i++) { mapper.AddMember2("T" + to_string(i) + ".maxRuntimeUs",         i*sizeof(rtos_task_t) + offsetof(rtos_info_t, task[0].maxRuntimeUs), DATA_TYPE_UINT32, "us", "Task max execution time"); }
    for (int i=0; i<GPX_RTOS_NUM_TASKS; i++) { mapper.AddMember2("T" + to_string(i) + ".startTimeUs",          i*sizeof(rtos_task_t) + offsetof(rtos_info_t, task[0].startTimeUs), DATA_TYPE_UINT32, "us", ""); }
    for (int i=0; i<GPX_RTOS_NUM_TASKS; i++) { mapper.AddMember2("T" + to_string(i) + ".gapCount",             i*sizeof(rtos_task_t) + offsetof(rtos_info_t, task[0].gapCount), DATA_TYPE_UINT16, "", "Number of times task took too long"); }
    for (int i=0; i<GPX_RTOS_NUM_TASKS; i++) { mapper.AddMember2("T" + to_string(i) + ".doubleGapCount",       i*sizeof(rtos_task_t) + offsetof(rtos_info_t, task[0].doubleGapCount), DATA_TYPE_UINT8, "", "Number of times task took too long twice in a row"); }
    for (int i=0; i<GPX_RTOS_NUM_TASKS; i++) { mapper.AddMember2("T" + to_string(i) + ".reserved",             i*sizeof(rtos_task_t) + offsetof(rtos_info_t, task[0].reserved), DATA_TYPE_UINT8, "", "", DATA_FLAGS_HIDDEN); }
    for (int i=0; i<GPX_RTOS_NUM_TASKS; i++) { mapper.AddMember2("T" + to_string(i) + ".handle",               i*sizeof(rtos_task_t) + offsetof(rtos_info_t, task[0].handle), DATA_TYPE_UINT32); }
}

static void PopulateMapCanConfig(data_set_t data_set[DID_COUNT], uint32_t did)
{
    DataMapper<can_config_t> mapper(data_set, did);
    
    mapper.AddMember2("can_period_mult[CID_INS_TIME]",              offsetof(can_config_t, can_period_mult) + sizeof(uint16_t) * CID_INS_TIME, DATA_TYPE_UINT16, "", "Period multiplier for INS time");
    mapper.AddMember2("can_period_mult[CIDINS_STATUS]",             offsetof(can_config_t, can_period_mult) + sizeof(uint16_t) * CID_INS_STATUS, DATA_TYPE_UINT16, "", "Perid multiplier for INS status");
    mapper.AddMember2("can_period_mult[CIDINS_EULER]",              offsetof(can_config_t, can_period_mult) + sizeof(uint16_t) * CID_INS_EULER, DATA_TYPE_UINT16, "", "Period multiplier for INS Euler angles");
    mapper.AddMember2("can_period_mult[CIDINS_QUATN2B]",            offsetof(can_config_t, can_period_mult) + sizeof(uint16_t) * CID_INS_QUATN2B, DATA_TYPE_UINT16, "", "Period multiplier for INS quaternion");
    mapper.AddMember2("can_period_mult[CIDINS_QUATE2B]",            offsetof(can_config_t, can_period_mult) + sizeof(uint16_t) * CID_INS_QUATE2B, DATA_TYPE_UINT16, "", "Period multiplier for INS quaternion");
    mapper.AddMember2("can_period_mult[CIDINS_UVW]",                offsetof(can_config_t, can_period_mult) + sizeof(uint16_t) * CID_INS_UVW, DATA_TYPE_UINT16, "", "Period multiplier for INS velocity");
    mapper.AddMember2("can_period_mult[CIDINS_VE]",                 offsetof(can_config_t, can_period_mult) + sizeof(uint16_t) * CID_INS_VE, DATA_TYPE_UINT16, "", "Period multiplier for INS velocity");
    mapper.AddMember2("can_period_mult[CIDINS_LAT]",                offsetof(can_config_t, can_period_mult) + sizeof(uint16_t) * CID_INS_LAT, DATA_TYPE_UINT16, "", "Period multiplier for INS latitude");
    mapper.AddMember2("can_period_mult[CIDINS_LON]",                offsetof(can_config_t, can_period_mult) + sizeof(uint16_t) * CID_INS_LON, DATA_TYPE_UINT16, "", "Period multiplier for INS longitude");
    mapper.AddMember2("can_period_mult[CIDINS_ALT]",                offsetof(can_config_t, can_period_mult) + sizeof(uint16_t) * CID_INS_ALT, DATA_TYPE_UINT16, "", "Period multiplier for INS altitude");
    mapper.AddMember2("can_period_mult[CIDINS_NORTH_EAST]",         offsetof(can_config_t, can_period_mult) + sizeof(uint16_t) * CID_INS_NORTH_EAST, DATA_TYPE_UINT16, "", "Period multiplier for INS north east");
    mapper.AddMember2("can_period_mult[CIDINS_DOWN]",               offsetof(can_config_t, can_period_mult) + sizeof(uint16_t) * CID_INS_DOWN, DATA_TYPE_UINT16, "", "Period multiplier for INS down");
    mapper.AddMember2("can_period_mult[CIDINS_ECEF_X]",             offsetof(can_config_t, can_period_mult) + sizeof(uint16_t) * CID_INS_ECEF_X, DATA_TYPE_UINT16, "", "Period multiplier for INS ECEF X");
    mapper.AddMember2("can_period_mult[CIDINS_ECEF_Y]",             offsetof(can_config_t, can_period_mult) + sizeof(uint16_t) * CID_INS_ECEF_Y, DATA_TYPE_UINT16, "", "Period multiplier for INS ECEF Y");
    mapper.AddMember2("can_period_mult[CIDINS_ECEF_Z]",             offsetof(can_config_t, can_period_mult) + sizeof(uint16_t) * CID_INS_ECEF_Z, DATA_TYPE_UINT16, "", "Period multiplier for INS ECEF Z");
    mapper.AddMember2("can_period_mult[CIDINS_L]",                  offsetof(can_config_t, can_period_mult) + sizeof(uint16_t) * CID_INS_MSL, DATA_TYPE_UINT16, "", "Period multiplier for INS MSL");
    mapper.AddMember2("can_period_mult[CIDPREINT_PX]",              offsetof(can_config_t, can_period_mult) + sizeof(uint16_t) * CID_PREINT_PX, DATA_TYPE_UINT16, "", "Period multiplier for INS preintegrated PX");
    mapper.AddMember2("can_period_mult[CIDPREINT_QY]",              offsetof(can_config_t, can_period_mult) + sizeof(uint16_t) * CID_PREINT_QY, DATA_TYPE_UINT16, "", "Period multiplier for INS preintegrated QY");
    mapper.AddMember2("can_period_mult[CIDPREINT_RZ]",              offsetof(can_config_t, can_period_mult) + sizeof(uint16_t) * CID_PREINT_RZ, DATA_TYPE_UINT16, "", "Period multiplier for INS preintegrated RZ");
    mapper.AddMember2("can_period_mult[CIDDUAL_PX]",                offsetof(can_config_t, can_period_mult) + sizeof(uint16_t) * CID_DUAL_PX, DATA_TYPE_UINT16, "", "Period multiplier for INS dual PX");
    mapper.AddMember2("can_period_mult[CIDDUAL_QY]",                offsetof(can_config_t, can_period_mult) + sizeof(uint16_t) * CID_DUAL_QY, DATA_TYPE_UINT16, "", "Period multiplier for INS dual QY");
    mapper.AddMember2("can_period_mult[CIDDUAL_RZ]",                offsetof(can_config_t, can_period_mult) + sizeof(uint16_t) * CID_DUAL_RZ, DATA_TYPE_UINT16, "", "Period multiplier for INS dual RZ");
    mapper.AddMember2("can_period_mult[CIDGPS1_POS]",               offsetof(can_config_t, can_period_mult) + sizeof(uint16_t) * CID_GPS1_POS, DATA_TYPE_UINT16, "", "Period multiplier for GPS1 position");
    mapper.AddMember2("can_period_mult[CIDGPS2_POS]",               offsetof(can_config_t, can_period_mult) + sizeof(uint16_t) * CID_GPS2_POS, DATA_TYPE_UINT16, "", "Period multiplier for GPS2 position");
    mapper.AddMember2("can_period_mult[CIDGPS1_RTK_POS_REL]",       offsetof(can_config_t, can_period_mult) + sizeof(uint16_t) * CID_GPS1_RTK_POS_REL, DATA_TYPE_UINT16, "", "Period multiplier for GPS1 RTK position relative");
    mapper.AddMember2("can_period_mult[CIDGPS2_RTK_CMP_REL]",       offsetof(can_config_t, can_period_mult) + sizeof(uint16_t) * CID_GPS2_RTK_CMP_REL, DATA_TYPE_UINT16, "", "Period multiplier for GPS2 RTK compass relative");
    mapper.AddMember2("can_period_mult[CIDROLL_ROLLRATE]",          offsetof(can_config_t, can_period_mult) + sizeof(uint16_t) * CID_ROLL_ROLLRATE, DATA_TYPE_UINT16, "", "Period multiplier for roll rate");
    mapper.AddMember2("cantransmit_address[CID_INS_TIME]",          offsetof(can_config_t, can_transmit_address) + sizeof(uint32_t) * CID_INS_TIME, DATA_TYPE_UINT32, "", "Address for INS time", DATA_FLAGS_DISPLAY_HEX);
    mapper.AddMember2("cantransmit_address[CID_INS_STATUS]",        offsetof(can_config_t, can_transmit_address) + sizeof(uint32_t) * CID_INS_STATUS, DATA_TYPE_UINT32, "", "Address for INS Status", DATA_FLAGS_DISPLAY_HEX);
    mapper.AddMember2("cantransmit_address[CID_INS_EULER]",         offsetof(can_config_t, can_transmit_address) + sizeof(uint32_t) * CID_INS_EULER, DATA_TYPE_UINT32, "", "Address for INS Euler Angles", DATA_FLAGS_DISPLAY_HEX);
    mapper.AddMember2("cantransmit_address[CID_INS_QUATN2B]",       offsetof(can_config_t, can_transmit_address) + sizeof(uint32_t) * CID_INS_QUATN2B, DATA_TYPE_UINT32, "", "Address for INS Quaternion", DATA_FLAGS_DISPLAY_HEX);
    mapper.AddMember2("cantransmit_address[CID_INS_QUATE2B]",       offsetof(can_config_t, can_transmit_address) + sizeof(uint32_t) * CID_INS_QUATE2B, DATA_TYPE_UINT32, "", "Address for INS Quaternion", DATA_FLAGS_DISPLAY_HEX);
    mapper.AddMember2("cantransmit_address[CID_INS_UVW]",           offsetof(can_config_t, can_transmit_address) + sizeof(uint32_t) * CID_INS_UVW, DATA_TYPE_UINT32, "", "Address for INS Velocity", DATA_FLAGS_DISPLAY_HEX);
    mapper.AddMember2("cantransmit_address[CID_INS_VE]",            offsetof(can_config_t, can_transmit_address) + sizeof(uint32_t) * CID_INS_VE, DATA_TYPE_UINT32, "", "Address for INS Velocity", DATA_FLAGS_DISPLAY_HEX);
    mapper.AddMember2("cantransmit_address[CID_INS_LAT]",           offsetof(can_config_t, can_transmit_address) + sizeof(uint32_t) * CID_INS_LAT, DATA_TYPE_UINT32, "", "Address for INS Latitude", DATA_FLAGS_DISPLAY_HEX);
    mapper.AddMember2("cantransmit_address[CID_INS_LON]",           offsetof(can_config_t, can_transmit_address) + sizeof(uint32_t) * CID_INS_LON, DATA_TYPE_UINT32, "", "Address for INS Longitude", DATA_FLAGS_DISPLAY_HEX);
    mapper.AddMember2("cantransmit_address[CID_INS_ALT]",           offsetof(can_config_t, can_transmit_address) + sizeof(uint32_t) * CID_INS_ALT, DATA_TYPE_UINT32, "", "Address for INS Altitude", DATA_FLAGS_DISPLAY_HEX);
    mapper.AddMember2("cantransmit_address[CID_INS_NORTH_EAST]",    offsetof(can_config_t, can_transmit_address) + sizeof(uint32_t) * CID_INS_NORTH_EAST, DATA_TYPE_UINT32, "", "Address for INS North/East", DATA_FLAGS_DISPLAY_HEX);
    mapper.AddMember2("cantransmit_address[CID_INS_DOWN]",          offsetof(can_config_t, can_transmit_address) + sizeof(uint32_t) * CID_INS_DOWN, DATA_TYPE_UINT32, "", "Addres for INS Down", DATA_FLAGS_DISPLAY_HEX);
    mapper.AddMember2("cantransmit_address[CID_INS_ECEF_X]",        offsetof(can_config_t, can_transmit_address) + sizeof(uint32_t) * CID_INS_ECEF_X, DATA_TYPE_UINT32, "", "Address for ECEF X", DATA_FLAGS_DISPLAY_HEX);
    mapper.AddMember2("cantransmit_address[CID_INS_ECEF_Y]",        offsetof(can_config_t, can_transmit_address) + sizeof(uint32_t) * CID_INS_ECEF_Y, DATA_TYPE_UINT32, "", "Address for ECEF Y", DATA_FLAGS_DISPLAY_HEX);
    mapper.AddMember2("cantransmit_address[CID_INS_ECEF_Z]",        offsetof(can_config_t, can_transmit_address) + sizeof(uint32_t) * CID_INS_ECEF_Z, DATA_TYPE_UINT32, "", "Address for ECEF Z", DATA_FLAGS_DISPLAY_HEX);
    mapper.AddMember2("cantransmit_address[CID_INS_MSL]",           offsetof(can_config_t, can_transmit_address) + sizeof(uint32_t) * CID_INS_MSL, DATA_TYPE_UINT32, "", "Address for Mean Sea Level", DATA_FLAGS_DISPLAY_HEX);
    mapper.AddMember2("cantransmit_address[CID_PREINT_PX]",         offsetof(can_config_t, can_transmit_address) + sizeof(uint32_t) * CID_PREINT_PX, DATA_TYPE_UINT32, "", "Address for INS preintegrated IMU PX", DATA_FLAGS_DISPLAY_HEX);
    mapper.AddMember2("cantransmit_address[CID_PREINT_QY]",         offsetof(can_config_t, can_transmit_address) + sizeof(uint32_t) * CID_PREINT_QY, DATA_TYPE_UINT32, "", "Address for INS preintegrated IMU QY", DATA_FLAGS_DISPLAY_HEX);
    mapper.AddMember2("cantransmit_address[CID_PREINT_RZ]",         offsetof(can_config_t, can_transmit_address) + sizeof(uint32_t) * CID_PREINT_RZ, DATA_TYPE_UINT32, "", "Address for INS preintegrated IMU RZ", DATA_FLAGS_DISPLAY_HEX);
    mapper.AddMember2("cantransmit_address[CID_DUAL_PX]",           offsetof(can_config_t, can_transmit_address) + sizeof(uint32_t) * CID_DUAL_PX, DATA_TYPE_UINT32, "", "Address for dual IMU PX", DATA_FLAGS_DISPLAY_HEX);
    mapper.AddMember2("cantransmit_address[CID_DUAL_QY]",           offsetof(can_config_t, can_transmit_address) + sizeof(uint32_t) * CID_DUAL_QY, DATA_TYPE_UINT32, "", "Address for INS dual IMU QY", DATA_FLAGS_DISPLAY_HEX);
    mapper.AddMember2("cantransmit_address[CID_DUAL_RZ]",           offsetof(can_config_t, can_transmit_address) + sizeof(uint32_t) * CID_DUAL_RZ, DATA_TYPE_UINT32, "", "Address for INS dual IMU RZ", DATA_FLAGS_DISPLAY_HEX);
    mapper.AddMember2("cantransmit_address[CID_GPS1_POS]",          offsetof(can_config_t, can_transmit_address) + sizeof(uint32_t) * CID_GPS1_POS, DATA_TYPE_UINT32, "", "Address for GPS1_POS", DATA_FLAGS_DISPLAY_HEX);
    mapper.AddMember2("cantransmit_address[CID_GPS2_POS]",          offsetof(can_config_t, can_transmit_address) + sizeof(uint32_t) * CID_GPS2_POS, DATA_TYPE_UINT32, "", "Address for GPS2 POS", DATA_FLAGS_DISPLAY_HEX);
    mapper.AddMember2("cantransmit_address[CID_GPS1_RTK_POS_REL]",  offsetof(can_config_t, can_transmit_address) + sizeof(uint32_t) * CID_GPS1_RTK_POS_REL, DATA_TYPE_UINT32, "", "Adress for GPS1 RTK POS REL", DATA_FLAGS_DISPLAY_HEX);
    mapper.AddMember2("cantransmit_address[CID_GPS2_RTK_CMP_REL]",  offsetof(can_config_t, can_transmit_address) + sizeof(uint32_t) * CID_GPS2_RTK_CMP_REL, DATA_TYPE_UINT32, "", "Address for GPS2 RTK CMP REL", DATA_FLAGS_DISPLAY_HEX);
    mapper.AddMember2("cantransmit_address[CID_ROLL_ROLLRATE]",     offsetof(can_config_t, can_transmit_address) + sizeof(uint32_t) * CID_ROLL_ROLLRATE, DATA_TYPE_UINT32, "", "Address for Roll Rate", DATA_FLAGS_DISPLAY_HEX);
    mapper.AddMember("can_baudrate_kbps", &can_config_t::can_baudrate_kbps, DATA_TYPE_UINT16, "kbps", "CAN baud rate");
    mapper.AddMember("can_receive_address", &can_config_t::can_receive_address, DATA_TYPE_UINT32, "", "CAN Receive Address", DATA_FLAGS_DISPLAY_HEX);
}

static void PopulateMapDiagMsg(data_set_t data_set[DID_COUNT], uint32_t did)
{
    DataMapper<diag_msg_t> mapper(data_set, did);
    mapper.AddMember("timeOfWeekMs", &diag_msg_t::timeOfWeekMs, DATA_TYPE_UINT32);
    mapper.AddMember("messageLength", &diag_msg_t::messageLength, DATA_TYPE_UINT32);
    mapper.AddMember("message", &diag_msg_t::message, DATA_TYPE_STRING);
}

static void PopulateMapSensorsADC(data_set_t data_set[DID_COUNT], uint32_t did)
{
    DataMapper<sys_sensors_adc_t> mapper(data_set, did);
    mapper.AddMember("time", &sys_sensors_adc_t::time, DATA_TYPE_F64, "s", "Time since boot up", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3);    
    for (int i=0; i<NUM_IMU_DEVICES; i++)
    {
        mapper.AddArray2("imu" + to_string(i) + ".pqr",   i*sizeof(sensors_imu_w_temp_t) + offsetof(sys_sensors_adc_t, imu[0].pqr), DATA_TYPE_F32, 3, {"LSB"}, {"Uncalibrated sensor output."}, DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_2);
        mapper.AddArray2("imu" + to_string(i) + ".acc",   i*sizeof(sensors_imu_w_temp_t) + offsetof(sys_sensors_adc_t, imu[0].acc), DATA_TYPE_F32, 3, {"LSB"}, {"Uncalibrated sensor output."}, DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_2);
        mapper.AddMember2("imu" + to_string(i) + ".temp", i*sizeof(sensors_imu_w_temp_t) + offsetof(sys_sensors_adc_t, imu[0].temp), DATA_TYPE_F32, "LSB", "", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3);
    }
    for (int i=0; i<NUM_MAG_DEVICES; i++)
    {
        mapper.AddArray2("mag" + to_string(i) + ".mag", i*sizeof(sensors_mag_t) + offsetof(sys_sensors_adc_t, mag[0].mag), DATA_TYPE_F32, 3, {"LSB"}, {"Uncalibrated sensor output."}, DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_2);
    }
    mapper.AddMember("bar", &sys_sensors_adc_t::bar, DATA_TYPE_F32, "kPa", "Barometric pressure", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3);
    mapper.AddMember("barTemp", &sys_sensors_adc_t::barTemp, DATA_TYPE_F32, SYM_DEG_C, "Barometer temperature", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3);
    mapper.AddMember("humidity", &sys_sensors_adc_t::humidity, DATA_TYPE_F32, "%rH", "Relative humidity, 0%-100%", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3);
    mapper.AddArray("ana", &sys_sensors_adc_t::ana, DATA_TYPE_F32, NUM_ANA_CHANNELS, {"V"}, {"ADC analog 0 input voltage"}, DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3);
}

static void PopulateMapSensorsWTemp(data_set_t data_set[DID_COUNT], uint32_t did)
{
    DataMapper<sensors_w_temp_t> mapper(data_set, did);
    int flags = DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_2;
    mapper.AddMember2("imu3.time",   offsetof(sensors_w_temp_t, imu3.time), DATA_TYPE_F64, "s", "Time since boot up", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3);
    mapper.AddMember2("imu3.status", offsetof(sensors_w_temp_t, imu3.status), DATA_TYPE_UINT32, "", "Status", DATA_FLAGS_READ_ONLY | DATA_FLAGS_DISPLAY_HEX);
    for (int i=0; i<NUM_IMU_DEVICES; i++)
    {
        mapper.AddArray2("imu" + to_string(i) + ".pqr", i*sizeof(imus_t) + offsetof(sensors_w_temp_t, imu3.I[0].pqr), DATA_TYPE_F32, 3, {SYM_DEG_PER_S}, {"Uncalibrated sensor output."}, flags, C_RAD2DEG);
        mapper.AddArray2("imu" + to_string(i) + ".acc", i*sizeof(imus_t) + offsetof(sensors_w_temp_t, imu3.I[0].acc), DATA_TYPE_F32, 3, {SYM_M_PER_S_2}, {"Uncalibrated sensor output."}, flags);
    }
    mapper.AddArray("temp", &sensors_w_temp_t::temp, DATA_TYPE_F32, 3, {SYM_DEG_C}, {"Uncalibrated sensor output."}, flags);
    for (int i=0; i<NUM_MAG_DEVICES; i++)
    {
        mapper.AddArray2("mag" + to_string(i) + ".xyz", i*sizeof(mag_xyz_t) + offsetof(sensors_w_temp_t, mag[0].xyz), DATA_TYPE_F32, 3, {""}, {"Uncalibrated sensor output."}, flags);
    }
}

static void PopulateMapSensors(data_set_t data_set[DID_COUNT], uint32_t did)
{
    DataMapper<sensors_t> mapper(data_set, did);
    int flags = DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3;
    mapper.AddMember("time", &sensors_t::time, DATA_TYPE_F64, "s", "GPS time of week (since Sunday morning).");
    for (int i=0; i<NUM_IMU_DEVICES; i++)
    {
        mapper.AddArray2("pqr" + to_string(i), i*sizeof(sensors_mpu_t) + offsetof(sensors_t, mpu[0].pqr), DATA_TYPE_F32, 3, {SYM_DEG_PER_S}, {"Temperature compensation bias"}, flags);
        mapper.AddArray2("acc" + to_string(i), i*sizeof(sensors_mpu_t) + offsetof(sensors_t, mpu[0].acc), DATA_TYPE_F32, 3, {SYM_M_PER_S_2}, {"Temperature compensation bias"}, flags);
        mapper.AddArray2("mag" + to_string(i), i*sizeof(sensors_mpu_t) + offsetof(sensors_t, mpu[0].mag), DATA_TYPE_F32, 3, {""}, {"Temperature compensation bias"}, flags);
    }
}

static void PopulateMapInl2MagObsInfo(data_set_t data_set[DID_COUNT], uint32_t did)
{
    DataMapper<inl2_mag_obs_info_t> mapper(data_set, did);
    mapper.AddMember("timeOfWeekMs", &inl2_mag_obs_info_t::timeOfWeekMs, DATA_TYPE_UINT32, "ms", "Time of week since Sunday morning, GMT", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_4);
    mapper.AddMember("Ncal_samples", &inl2_mag_obs_info_t::Ncal_samples, DATA_TYPE_UINT32, "", "");
    mapper.AddMember("ready", &inl2_mag_obs_info_t::ready, DATA_TYPE_UINT32, "", "Data ready to be processed");
    mapper.AddMember("calibrated", &inl2_mag_obs_info_t::calibrated, DATA_TYPE_UINT32, "", "Calibration data present");
    mapper.AddMember("auto_recal", &inl2_mag_obs_info_t::auto_recal, DATA_TYPE_UINT32, "", "Allow mag to auto-recalibrate");
    mapper.AddMember("outlier", &inl2_mag_obs_info_t::outlier, DATA_TYPE_UINT32, "", "Bad sample data");
    mapper.AddMember("magHdg", &inl2_mag_obs_info_t::magHdg, DATA_TYPE_F32, "deg", "Heading from magnetometer", DATA_FLAGS_FIXED_DECIMAL_3 | DATA_FLAGS_ANGLE);
    mapper.AddMember("insHdg", &inl2_mag_obs_info_t::insHdg, DATA_TYPE_F32, "deg", "Heading from INS", DATA_FLAGS_FIXED_DECIMAL_3 | DATA_FLAGS_ANGLE);
    mapper.AddMember("magInsHdgDelta", &inl2_mag_obs_info_t::magInsHdgDelta, DATA_TYPE_F32, "deg", "Difference between magHdg and insHdg", DATA_FLAGS_FIXED_DECIMAL_3 | DATA_FLAGS_ANGLE);
    mapper.AddMember("nis", &inl2_mag_obs_info_t::nis, DATA_TYPE_F32, "", "", DATA_FLAGS_FIXED_DECIMAL_5);
    mapper.AddMember("nis_threshold", &inl2_mag_obs_info_t::nis_threshold, DATA_TYPE_F32, "", "", DATA_FLAGS_FIXED_DECIMAL_3);
    mapper.AddArray("Wcal", &inl2_mag_obs_info_t::Wcal, DATA_TYPE_F32, 9, {""}, {""}, DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3);
    mapper.AddMember("activeCalSet", &inl2_mag_obs_info_t::activeCalSet, DATA_TYPE_UINT32, "", "Active calibration set (0 or 1)");
    mapper.AddMember("magHdgOffset", &inl2_mag_obs_info_t::magHdgOffset, DATA_TYPE_F32, "deg", "Offset from mag heading to ins heading estimate", DATA_FLAGS_FIXED_DECIMAL_3);
    mapper.AddMember("Tcal", &inl2_mag_obs_info_t::Tcal, DATA_TYPE_F32, "", "Scaled computed variance of calibrated magnetometer samples. Above 5 is bad.", DATA_FLAGS_FIXED_DECIMAL_3);
    mapper.AddArray("bias_cal", &inl2_mag_obs_info_t::bias_cal, DATA_TYPE_F32, 3, {""}, {""}, DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_5);
}

static void PopulateMapInl2States(data_set_t data_set[DID_COUNT], uint32_t did)
{
    DataMapper<inl2_states_t> mapper(data_set, did);
    int flags = DATA_FLAGS_FIXED_DECIMAL_4;
    mapper.AddMember("timeOfWeek", &inl2_states_t::timeOfWeek, DATA_TYPE_F64, "s", "Time of week since Sunday morning, GMT", flags);
    mapper.AddArray("qe2b", &inl2_states_t::qe2b, DATA_TYPE_F32, 4, {""}, {"Quaternion rotation from ECEF to body frame"}, flags);
    mapper.AddArray("ve", &inl2_states_t::ve, DATA_TYPE_F32, 3, {"m/s"}, {"Velocity in ECEF frame"}, flags);
    mapper.AddArray("ecef", &inl2_states_t::ecef, DATA_TYPE_F64, 3, {"m"}, {"Position in ECEF frame"}, DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3);
    mapper.AddArray("biasPqr", &inl2_states_t::biasPqr, DATA_TYPE_F32, 3, {SYM_DEG_PER_S}, {"Gyro bias"}, DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3, C_RAD2DEG);
    mapper.AddArray("biasAcc", &inl2_states_t::biasAcc, DATA_TYPE_F32, 3, {SYM_M_PER_S_2}, {"Accelerometer bias"}, flags);
    mapper.AddMember("biasBaro", &inl2_states_t::biasBaro, DATA_TYPE_F32, "m", "Barometer bias", flags);
    mapper.AddMember("magDec", &inl2_states_t::magDec, DATA_TYPE_F32, SYM_DEG, "Magnetic declination", flags, C_RAD2DEG);
    mapper.AddMember("magInc", &inl2_states_t::magInc, DATA_TYPE_F32, SYM_DEG, "Magnetic inclination", flags, C_RAD2DEG);
}

static void PopulateMapInl2NedSigma(data_set_t data_set[DID_COUNT], uint32_t did)
{
    DataMapper<inl2_ned_sigma_t> mapper(data_set, did);
    int flags = DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_5;
    mapper.AddMember("timeOfWeekMs", &inl2_ned_sigma_t::timeOfWeekMs, DATA_TYPE_UINT32, "ms", "Time of week since Sunday morning, GMT", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_4);
    mapper.AddArray("StdAttNed", &inl2_ned_sigma_t::StdAttNed, DATA_TYPE_F32, 3, {SYM_DEG}, {"NED attitude error standard deviation"}, flags, C_RAD2DEG);
    mapper.AddArray("StdVelNed", &inl2_ned_sigma_t::StdVelNed, DATA_TYPE_F32, 3, {"m/s"}, {"NED velocity error standard deviation"}, flags);
    mapper.AddArray("StdPosNed", &inl2_ned_sigma_t::StdPosNed, DATA_TYPE_F32, 3, {"m"}, {"NED position error standard deviation"}, flags);
    mapper.AddArray("StdAccBias", &inl2_ned_sigma_t::StdAccBias, DATA_TYPE_F32, 3, {SYM_M_PER_S_2}, {"Acceleration bias error standard deviation"}, flags);
    mapper.AddArray("StdGyrBias", &inl2_ned_sigma_t::StdGyrBias, DATA_TYPE_F32, 3, {SYM_DEG}, {"Angular rate bias error standard deviation"}, flags, C_RAD2DEG);
    mapper.AddMember("StdBarBias", &inl2_ned_sigma_t::StdBarBias, DATA_TYPE_F32, "m", "Barometric altitude bias error standard deviation", flags);
    mapper.AddMember("StdMagDeclination", &inl2_ned_sigma_t::StdMagDeclination, DATA_TYPE_F32, SYM_DEG, "Mag declination error standard deviation", flags, C_RAD2DEG);
}

static void PopulateMapRosCovariancePoseTwist(data_set_t data_set[DID_COUNT], uint32_t did)
{
#define COV_POSE_UNITS      "rad², m²"
#define COV_TWIST_UNITS     "(m/s)², (rad/s)²"

    DataMapper<ros_covariance_pose_twist_t> mapper(data_set, did);
    int flags = DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_5;
    mapper.AddMember("timeOfWeek", &ros_covariance_pose_twist_t::timeOfWeek, DATA_TYPE_F64, "s", "Time of week since Sunday morning, GMT", flags);
    mapper.AddArray("covPoseLD", &ros_covariance_pose_twist_t::covPoseLD, DATA_TYPE_F32, 21, {COV_POSE_UNITS}, {"EKF attitude and position error covariance matrix lower diagonal in body (attitude) and ECEF (position) frames"}, flags);
    mapper.AddArray("covTwistLD", &ros_covariance_pose_twist_t::covTwistLD, DATA_TYPE_F32, 21, {COV_TWIST_UNITS}, {"EKF velocity and angular rate error covariance matrix lower diagonal in ECEF (velocity) and body (attitude) frames"}, flags);
}

#if PLATFORM_IS_EMBEDDED
cISDataMappings* cISDataMappings::s_map;
#else
cISDataMappings cISDataMappings::s_map;
#endif

const char* const cISDataMappings::m_dataIdNames[] =
{    // Matches data identifier list (eDataIDs) in data_sets.h
    "DID_NULL",                         // 0
    "DID_DEV_INFO",                     // 1
    "DID_SYS_FAULT",                    // 2
    "DID_PIMU",                         // 3
    "DID_INS_1",                        // 4
    "DID_INS_2",                        // 5
    "DID_GPS1_RCVR_POS",                // 6
    "DID_SYS_CMD",                      // 7
    "DID_NMEA_BCAST_PERIOD",            // 8
    "DID_RMC",                          // 9
    "DID_SYS_PARAMS",                   // 10
    "DID_SYS_SENSORS",                  // 11
    "DID_FLASH_CONFIG",                 // 12
    "DID_GPS1_POS",                     // 13
    "DID_GPS2_POS",                     // 14
    "DID_GPS1_SAT",                     // 15
    "DID_GPS2_SAT",                     // 16
    "DID_GPS1_VERSION",                 // 17
    "DID_GPS2_VERSION",                 // 18
    "DID_MAG_CAL",                      // 19
    "DID_UNUSED_20",                    // 20
    "DID_GPS1_RTK_POS_REL",             // 21
    "DID_GPS1_RTK_POS_MISC",            // 22
    "DID_FEATURE_BITS",                 // 23
    "DID_SENSORS_UCAL",                 // 24
    "DID_SENSORS_TCAL",                 // 25
    "DID_SENSORS_TC_BIAS",              // 26
    "DID_IO",                           // 27
    "DID_SENSORS_ADC",                  // 28
    "DID_SCOMP",                        // 29
    "DID_GPS1_VEL",                     // 30
    "DID_GPS2_VEL",                     // 31
    "DID_HDW_PARAMS",                   // 32
    "DID_NVR_MANAGE_USERPAGE",          // 33
    "DID_NVR_USERPAGE_SN",              // 34
    "DID_NVR_USERPAGE_G0",              // 35
    "DID_NVR_USERPAGE_G1",              // 36
    "DID_DEBUG_STRING",                 // 37
    "DID_RTOS_INFO",                    // 38
    "DID_DEBUG_ARRAY",                  // 39
    "DID_SENSORS_MCAL",                 // 40
    "DID_GPS1_TIMEPULSE",               // 41
    "DID_CAL_SC",                       // 42
    "DID_CAL_TEMP_COMP",                // 43
    "DID_CAL_MOTION",                   // 44
    "DID_GPS1_SIG",                     // 45
    "DID_SENSORS_ADC_SIGMA",            // 46
    "DID_REFERENCE_MAGNETOMETER",       // 47
    "DID_INL2_STATES",                  // 48
    "DID_INL2_COVARIANCE_LD",           // 49
    "DID_INL2_STATUS",                  // 50
    "DID_INL2_MISC",                    // 51
    "DID_MAGNETOMETER",                 // 52
    "DID_BAROMETER",                    // 53
    "DID_GPS1_RTK_POS",                 // 54
    "DID_ROS_COVARIANCE_POSE_TWIST",    // 55
    "DID_COMMUNICATIONS_LOOPBACK",      // 56
    "DID_IMU3_UNCAL",                   // 57
    "DID_IMU",                          // 58
    "DID_INL2_MAG_OBS_INFO",            // 59
    "DID_GPS_BASE_RAW",                 // 60
    "DID_GPS_RTK_OPT",                  // 61
    "DID_REFERENCE_PIMU",               // 62
    "DID_MANUFACTURING_INFO",           // 63
    "DID_BIT",                          // 64
    "DID_INS_3",                        // 65
    "DID_INS_4",                        // 66
    "DID_INL2_NED_SIGMA",               // 67
    "DID_STROBE_IN_TIME",               // 68
    "DID_GPS1_RAW",                     // 69
    "DID_GPS2_RAW",                     // 70
    "DID_WHEEL_ENCODER",                // 71
    "DID_DIAGNOSTIC_MESSAGE",           // 72
    "DID_SURVEY_IN",                    // 73
    "DID_CAL_SC_INFO",                  // 74
    "DID_PORT_MONITOR",                 // 75
    "DID_RTK_STATE",                    // 76
    "DID_RTK_PHASE_RESIDUAL",           // 77
    "DID_RTK_CODE_RESIDUAL",            // 78
    "DID_RTK_DEBUG",                    // 79
    "DID_EVB_STATUS",                   // 80
    "DID_EVB_FLASH_CFG",                // 81
    "DID_EVB_DEBUG_ARRAY",              // 82
    "DID_EVB_RTOS_INFO",                // 83
    "DID_GPS2_SIG",                     // 84
    "DID_IMU_MAG",                      // 85
    "DID_PIMU_MAG",                     // 86
    "DID_GROUND_VEHICLE",               // 87
    "DID_POSITION_MEASUREMENT",         // 88
    "DID_RTK_DEBUG_2",                  // 89
    "DID_CAN_CONFIG",                   // 90
    "DID_GPS2_RTK_CMP_REL",             // 91
    "DID_GPS2_RTK_CMP_MISC",            // 92
    "DID_EVB_DEV_INFO",                 // 93
    "DID_INFIELD_CAL",                  // 94 
    "DID_REFERENCE_IMU",                // 95 
    "DID_IMU3_RAW",                     // 96 
    "DID_IMU_RAW",                      // 97 
    "DID_FIRMWARE_UPDATE",              // 98 
    "DID_RUNTIME_PROFILER",             // 99 
    "UNUSED_100",                       // 100
    "UNUSED_101",                       // 101
    "UNUSED_102",                       // 102
    "UNUSED_103",                       // 103
    "UNUSED_104",                       // 104
    "UNUSED_105",                       // 105
    "UNUSED_106",                       // 106
    "UNUSED_107",                       // 107
    "UNUSED_108",                       // 108
    "UNUSED_109",                       // 109
    "DID_EVB_LUNA_FLASH_CFG",           // 110
    "DID_EVB_LUNA_STATUS",              // 111
    "DID_EVB_LUNA_SENSORS",             // 112
    "DID_EVB_LUNA_REMOTE_KILL",         // 113
    "DID_EVB_LUNA_VELOCITY_CONTROL",    // 114
    "DID_EVB_LUNA_VELOCITY_COMMAND",    // 115
    "DID_EVB_LUNA_AUX_COMMAND",         // 116
    "",                                 // 117
    "",                                 // 118
    "DID_EVENT",                        // 119
    "DID_GPX_DEV_INFO",                 // 120
    "DID_GPX_FLASH_CFG",                // 121
    "DID_GPX_RTOS_INFO",                // 122
    "DID_GPX_STATUS",                   // 123
    "DID_GPX_DEBUG_ARRAY",              // 124
    "DID_GPX_BIT",                      // 125
    "DID_GPX_RMC",                      // 126
    "DID_GPX_PORT_MONITOR",             // 127
    "",                                 // 128
    "",                                 // 129
    "",                                 // 130
    ""                                  // 131
};


cISDataMappings::cISDataMappings()
{
    for(eDataIDs i=0; i<DID_COUNT; i++)
    {   // Init to zero
        m_data_set[i] = {};
    }

    // CONFIG
    PopulateMapNvmFlashCfg(         m_data_set, DID_FLASH_CONFIG);
    PopulateMapGpxFlashCfg(         m_data_set, DID_GPX_FLASH_CFG);

    // DEBUG
    PopulateMapDebugString(         m_data_set, DID_DEBUG_STRING);
    PopulateMapDebugArray(          m_data_set, DID_DEBUG_ARRAY);
    PopulateMapDebugArray(          m_data_set, DID_GPX_DEBUG_ARRAY);
    // PopulateMapDiagMsg(          m_data_set, DID_DIAGNOSTIC_MESSAGE);

    // SOLUTION
    PopulateMapIns1(                m_data_set, DID_INS_1);
    PopulateMapIns2(                m_data_set, DID_INS_2);
    PopulateMapIns3(                m_data_set, DID_INS_3);
    PopulateMapIns4(                m_data_set, DID_INS_4);
    PopulateMapSysParams(           m_data_set, DID_SYS_PARAMS);

    // EKF
    PopulateMapInl2States(          m_data_set, DID_INL2_STATES);
    PopulateMapInl2NedSigma(        m_data_set, DID_INL2_NED_SIGMA);
    PopulateMapInl2MagObsInfo(      m_data_set, DID_INL2_MAG_OBS_INFO);
    PopulateMapRosCovariancePoseTwist(m_data_set, DID_ROS_COVARIANCE_POSE_TWIST);
    
    // SENSORS
    PopulateMapPimu(                m_data_set, DID_PIMU, "Preintegrated IMU.");
    PopulateMapImu(                 m_data_set, DID_IMU, "IMU data down-sampled from IMU rate to navigation rate.");
    PopulateMapImu(                 m_data_set, DID_IMU_RAW, "IMU data averaged from DID_IMU3_RAW.");
    PopulateMapImu3(                m_data_set, DID_IMU3_RAW, "Triple IMU data calibrated from DID_IMU3_UNCAL.");
    PopulateMapImu3(                m_data_set, DID_IMU3_UNCAL, "Triple IMU data directly from sensor (uncalibrated).");

    PopulateMapImu(                 m_data_set, DID_REFERENCE_IMU, "Reference IMU.");
    PopulateMapPimu(                m_data_set, DID_REFERENCE_PIMU, "Reference PIMU.");
    PopulateMapMagnetometer(        m_data_set, DID_REFERENCE_MAGNETOMETER);

    PopulateMapMagnetometer(        m_data_set, DID_MAGNETOMETER);
    PopulateMapBarometer(           m_data_set, DID_BAROMETER);
    PopulateMapWheelEncoder(        m_data_set, DID_WHEEL_ENCODER);

    PopulateMapGpsPos(              m_data_set, DID_GPS1_RTK_POS);
    PopulateMapGpsRtkRel(           m_data_set, DID_GPS1_RTK_POS_REL);
    PopulateMapGpsRtkMisc(          m_data_set, DID_GPS1_RTK_POS_MISC);
    PopulateMapGpsRtkRel(           m_data_set, DID_GPS2_RTK_CMP_REL);
    PopulateMapGpsRtkMisc(          m_data_set, DID_GPS2_RTK_CMP_MISC);

    PopulateMapGpsPos(              m_data_set, DID_GPS1_POS);
    PopulateMapGpsPos(              m_data_set, DID_GPS2_POS);
    PopulateMapGpsVel(              m_data_set, DID_GPS1_VEL);
    PopulateMapGpsVel(              m_data_set, DID_GPS2_VEL);
    PopulateMapGpsPos(              m_data_set, DID_GPS1_RCVR_POS);

#if 0	// Too much data, we don't want to log this. WHJ
    PopulateMapGpsSat(              m_data_set, DID_GPS1_SAT);
    PopulateMapGpsSat(              m_data_set, DID_GPS2_SAT);
    PopulateMapGpsSig(              m_data_set, DID_GPS1_SIG);
    PopulateMapGpsSig(              m_data_set, DID_GPS2_SIG);
#endif

    PopulateMapGpsVersion(          m_data_set, DID_GPS1_VERSION);
    PopulateMapGpsVersion(          m_data_set, DID_GPS2_VERSION);
    PopulateMapGpsTimepulse(        m_data_set, DID_GPS1_TIMEPULSE);

    PopulateMapGpsRaw(              m_data_set, DID_GPS1_RAW);
    PopulateMapGpsRaw(              m_data_set, DID_GPS2_RAW);
    PopulateMapGpsRaw(              m_data_set, DID_GPS_BASE_RAW);

    PopulateMapStrobeInTime(        m_data_set, DID_STROBE_IN_TIME);
    PopulateMapSysSensors(          m_data_set, DID_SYS_SENSORS);
    PopulateMapSensorsADC(          m_data_set, DID_SENSORS_ADC);
    PopulateMapSensorsADC(          m_data_set, DID_SENSORS_ADC_SIGMA);

    PopulateMapPimuMag(             m_data_set, DID_PIMU_MAG);
    PopulateMapImuMag(              m_data_set, DID_IMU_MAG);

    // CALIBRATION
    PopulateMapMagCal(              m_data_set, DID_MAG_CAL);
    PopulateMapInfieldCal(          m_data_set, DID_INFIELD_CAL);
    PopulateMapGroundVehicle(       m_data_set, DID_GROUND_VEHICLE);
    
    // SYSTEM
    PopulateMapDeviceInfo(          m_data_set, DID_DEV_INFO);
    PopulateMapDeviceInfo(          m_data_set, DID_GPX_DEV_INFO);

    PopulateMapSystemCommand(       m_data_set, DID_SYS_CMD);
    PopulateMapBit(                 m_data_set, DID_BIT);
    PopulateMapGpxBit(              m_data_set, DID_GPX_BIT);
    PopulateMapGpxStatus(           m_data_set, DID_GPX_STATUS);
    PopulateMapSurveyIn(            m_data_set, DID_SURVEY_IN);
    PopulateMapRtosInfo(            m_data_set, DID_RTOS_INFO);
    PopulateMapGpxRtosInfo(         m_data_set, DID_GPX_RTOS_INFO);
    PopulateMapSystemFault(         m_data_set, DID_SYS_FAULT);

    // COMMUNICATIONS
    PopulateMapPortMonitor(         m_data_set, DID_PORT_MONITOR);
    PopulateMapPortMonitor(         m_data_set, DID_GPX_PORT_MONITOR);
    PopulateMapNmeaMsgs(            m_data_set, DID_NMEA_BCAST_PERIOD);
    PopulateMapCanConfig(           m_data_set, DID_CAN_CONFIG);
    PopulateMapRmc(                 m_data_set, DID_RMC);
    PopulateMapRmc(                 m_data_set, DID_GPX_RMC);
    PopulateMapISEvent(             m_data_set, DID_EVENT);

    // EVB
    PopulateMapEvbStatus(           m_data_set, DID_EVB_STATUS);
    PopulateMapEvbFlashCfg(         m_data_set, DID_EVB_FLASH_CFG);
    PopulateMapDebugArray(          m_data_set, DID_EVB_DEBUG_ARRAY);
    // PopulateMapEvbRtosInfo(      m_data_set, DID_EVB_RTOS_INFO);
    PopulateMapDeviceInfo(          m_data_set, DID_EVB_DEV_INFO);

    // MANUFACTURING
    PopulateMapManufacturingInfo(   m_data_set, DID_MANUFACTURING_INFO);
    PopulateMapSensorsWTemp(        m_data_set, DID_SENSORS_UCAL);
    PopulateMapSensorsWTemp(        m_data_set, DID_SENSORS_TCAL);
    PopulateMapSensorsWTemp(        m_data_set, DID_SENSORS_MCAL);
    PopulateMapSensors(             m_data_set, DID_SENSORS_TC_BIAS);

    // This must come last
    for (uint32_t did = 0; did < DID_COUNT; did++)
    {
        PopulateMapTimestampField(m_data_set, did);
    }
}

data_set_t* cISDataMappings::DataSet(uint32_t did)
{
    if (did >= DID_COUNT)
    {
        return NULL;
    }

#if PLATFORM_IS_EMBEDDED
    if (s_map == NULLPTR)
    {
        s_map = new cISDataMappings();
    }

    return &(s_map->m_data_set[did]);
#else
    return &(s_map.m_data_set[did]);
#endif
}

const char* cISDataMappings::DataName(uint32_t did)
{
    STATIC_ASSERT(_ARRAY_ELEMENT_COUNT(m_dataIdNames) == DID_COUNT);
    if (did >= DID_COUNT)
    {
        return "unknown";
    }

    return m_dataIdNames[did];
}

uint32_t cISDataMappings::Did(string s)
{
    // Try to use DID numbers
    uint32_t did = strtol(s.c_str(), NULL, 10);

    if (did <= DID_NULL || did >= DID_COUNT)
    {   // Number is invalid.  Use DID name.
        string name = s;
        std::string::size_type pos = s.find('=');
        if (pos != std::string::npos)
        {   // Remove equal sign
            name = s.substr(0, pos);
        }
        did = cISDataMappings::NameToDid(name);
        return did;
    }

    if (did > DID_NULL && did < DID_COUNT)
    {
        return did;         // Valid DID
    }
    else
    {
        return DID_NULL;    // Invalid DID
    }
}

uint32_t cISDataMappings::NameToDid(string name)
{
    for (eDataIDs did = 0; did < DID_COUNT; did++)
    {
        if (strcmp(name.c_str(), m_dataIdNames[did]) == 0)
        {    // Found match
            return did;
        }
    }

    return 0;
}

uint32_t cISDataMappings::DataSize(uint32_t did)
{
    data_set_t* ds = DataSet(did);
    if (!ds) { return 0; }
    return ds->size;
}

const map_name_to_info_t* cISDataMappings::NameToInfoMap(uint32_t did)
{
    data_set_t* ds = DataSet(did);
    if (!ds) { return NULLPTR; }
    return &(ds->nameToInfo);
}

const map_index_to_info_t* cISDataMappings::IndexToInfoMap(uint32_t did)
{
    data_set_t* ds = DataSet(did);
    if (!ds) { return NULLPTR; }
    return &(ds->indexToInfo);
}

const data_info_t* cISDataMappings::ElementToInfo(uint32_t did, uint32_t element, uint32_t &arrayIndex)
{
    data_set_t* ds = DataSet(did);
    if (!ds) { return NULLPTR; }
	if (ds->elementToInfo.find(element) == ds->elementToInfo.end()) { return NULLPTR; }
    arrayIndex = ds->elementToArraySize[element];
    return ds->elementToInfo[element];
}

uint32_t cISDataMappings::ElementCount(uint32_t did)
{
    data_set_t* ds = DataSet(did);
    if (!ds) { return 0; }
    return ds->elementCount;
}

uint32_t cISDataMappings::DefaultPeriodMultiple(uint32_t did)
{
    switch (did)
    {
    case DID_DEV_INFO:
    case DID_GPS1_VERSION:
    case DID_GPS2_VERSION:
    case DID_GPS1_TIMEPULSE:
    case DID_SYS_SENSORS:
    case DID_SENSORS_ADC:
    case DID_SENSORS_ADC_SIGMA:
    case DID_SENSORS_TC_BIAS:
    case DID_SENSORS_UCAL:
    case DID_SENSORS_TCAL:
    case DID_SENSORS_MCAL:
    case DID_SCOMP:
    case DID_HDW_PARAMS:
    case DID_SYS_PARAMS:
    case DID_NVR_MANAGE_USERPAGE:
    case DID_NVR_USERPAGE_SN:
    case DID_NVR_USERPAGE_G0:
    case DID_NVR_USERPAGE_G1:
    case DID_FLASH_CONFIG:
    case DID_CAL_SC_INFO:
    case DID_CAL_SC:
    case DID_CAL_TEMP_COMP:
    case DID_CAL_MOTION:
    case DID_RTOS_INFO:
    case DID_SYS_CMD:
    case DID_NMEA_BCAST_PERIOD:
    case DID_RMC:
    case DID_DEBUG_STRING:
    case DID_DEBUG_ARRAY:
    case DID_MAG_CAL:
    case DID_COMMUNICATIONS_LOOPBACK:
    case DID_BIT:
    case DID_WHEEL_ENCODER:
    case DID_SYS_FAULT:
    case DID_SURVEY_IN:
    case DID_PORT_MONITOR:
    case DID_CAN_CONFIG:
    case DID_INFIELD_CAL:
    case DID_REFERENCE_IMU:
    case DID_REFERENCE_PIMU:
    case DID_REFERENCE_MAGNETOMETER:
    case DID_RUNTIME_PROFILER:
    case DID_INL2_COVARIANCE_LD:
    case DID_INL2_STATUS:
    case DID_INL2_MISC:
    case DID_INL2_STATES:
    case DID_ROS_COVARIANCE_POSE_TWIST:
    case DID_INL2_MAG_OBS_INFO:
    case DID_GPX_DEV_INFO:
    case DID_GPX_FLASH_CFG:
    case DID_GPX_RTOS_INFO:
    case DID_GPX_STATUS:
    case DID_GPX_DEBUG_ARRAY:
    case DID_GPX_BIT:
    case DID_GPX_RMC:
    case DID_GPX_PORT_MONITOR:
        return 100;     // (100ms, 10 Hz)

    default:    // DIDs not listed above should be 1.  This includes DIDs that use RMC.
        return 1;
    }
}


bool cISDataMappings::StringToData(const char* stringBuffer, int stringLength, const p_data_hdr_t* hdr, uint8_t* datasetBuffer, const data_info_t& info, unsigned int arrayIndex, bool json, bool useConversion)
{
    const uint8_t* ptr = FieldData(info, arrayIndex, hdr, datasetBuffer);
    if (ptr == NULL)
    {
        return false;
    }

#if 1   // Use display type
    int radix = (info.flags&DATA_FLAGS_DISPLAY_HEX ? 16 : 10);
#else   // Hex if first two characters are 0x
    int radix = ((stringBuffer[0] == '0' && stringBuffer[1] == 'x') == 0 ? 16 : 10);
#endif

    double conversion = useConversion ? info.conversion : 1.0;      // When useConversion is false, don't convert units.  Used for CSV logs. (WHJ)

    return StringToVariable(stringBuffer, stringLength, ptr, info.type, info.size, radix, conversion, json);
}

bool cISDataMappings::StringToVariable(const char* stringBuffer, int stringLength, const uint8_t* dataBuffer, eDataType dataType, uint32_t dataSize, int radix, double conversion, bool json)
{
    // Reset errno before calling strtol/strtoul/strtod.  There are cases in which it only gets set and not reset.
    errno = 0;

    float valuef32;
    double valuef64;
    switch (dataType)
    {
    case DATA_TYPE_INT8:
        protectUnalignedAssign<int8_t>((void*)dataBuffer, (int8_t)strtol(stringBuffer, NULL, radix));
        break;

    case DATA_TYPE_INT16:
        protectUnalignedAssign<int16_t>((void*)dataBuffer, (int16_t)strtol(stringBuffer, NULL, radix));
        break;

    case DATA_TYPE_INT32:
        protectUnalignedAssign<int32_t>((void*)dataBuffer, (int32_t)strtol(stringBuffer, NULL, radix));
        break;

    case DATA_TYPE_UINT8:
        protectUnalignedAssign<uint8_t>((void*)dataBuffer, (uint8_t)strtoul(stringBuffer, NULL, radix));
        break;

    case DATA_TYPE_UINT16:
        protectUnalignedAssign<uint16_t>((void*)dataBuffer, (uint16_t)strtoul(stringBuffer, NULL, radix));
        break;

    case DATA_TYPE_UINT32:
        protectUnalignedAssign<uint32_t>((void*)dataBuffer, (uint32_t)strtoul(stringBuffer, NULL, radix));
        break;

    case DATA_TYPE_INT64:
        protectUnalignedAssign<int64_t>((void*)dataBuffer, (int64_t)strtoll(stringBuffer, NULL, radix));
        break;

    case DATA_TYPE_UINT64:
        protectUnalignedAssign<uint64_t>((void*)dataBuffer, (uint64_t)strtoull(stringBuffer, NULL, radix));
        break;

    case DATA_TYPE_F32:
        protectUnalignedAssign<float>(&valuef32, (float)strtod(stringBuffer, NULL));
        (*(float*)dataBuffer) = valuef32 / conversion;
        break;

    case DATA_TYPE_F64:
        protectUnalignedAssign<double>(&valuef64, (double)strtod(stringBuffer, NULL));
        (*(double*)dataBuffer) = valuef64 / conversion;
        break;

    case DATA_TYPE_STRING:
    {
        string s2(stringBuffer);
        if (json)
        {
            char c;
            bool escaped = false;
            for (size_t i = 0; i < s2.size(); i++)
            {
                c = s2[i];
                if (c == '\\')
                {
                    if (!escaped)
                    {
                        escaped = true;
                        s2.erase(i);
                        continue;
                    }
                }
                escaped = false;
            }
            s2 = stringBuffer;
        }
        else
        {
            s2.erase(std::remove(s2.begin(), s2.end(), '"'), s2.end());
        }
        // ensure string fits with null terminator
        s2.resize(dataSize - 1);
        memcpy((void*)dataBuffer, s2.data(), s2.length());
        memset((uint8_t*)dataBuffer + s2.length(), 0, dataSize - s2.length());
    } break;

    case DATA_TYPE_BINARY:
    {
        // convert hex data back to binary
        size_t len = _MIN(1020, stringLength);
        len -= (len % 2);
        uint8_t* ptr2 = (uint8_t*)dataBuffer;
        for (size_t i = 0; i != len; i += 2)
        {
            *ptr2 = (getHexValue(stringBuffer[i + 1]) << 4) | getHexValue(stringBuffer[i + 2]);
        }
    } break;

    default:
        return false;
    }

    return true;
}


bool cISDataMappings::DataToString(const data_info_t& info, const p_data_hdr_t* hdr, const uint8_t* datasetBuffer, data_mapping_string_t stringBuffer, unsigned int arrayIndex, bool json, bool useConversion)
{
    const uint8_t* ptr = FieldData(info, arrayIndex, hdr, datasetBuffer);
    if (ptr == NULL)
    {
        // pick a default string
        if (info.type == DATA_TYPE_STRING)
        {
            stringBuffer[0] = '"';
            stringBuffer[1] = '"';
            stringBuffer[2] = '\0';
        }
        else if (info.type == DATA_TYPE_BINARY)
        {
            if (json)
            {
                stringBuffer[0] = '"';
                stringBuffer[1] = '"';
                stringBuffer[2] = '\0';
            }
            else
            {
                stringBuffer[0] = '\0';
            }
        }
        else
        {
            stringBuffer[0] = '0';
            stringBuffer[1] = '\0';
        }
        return false;
    }

    double conversion = info.conversion;
    uint32_t flags = (uint32_t)info.flags;
    if (!useConversion)
    {   // Don't convert units or reduce precision.  Used for CSV logs. (WHJ)
        conversion = 1.0;
        flags &= (~DATA_FLAGS_FIXED_DECIMAL_MASK);
    }

    return VariableToString(info.type, (eDataFlags)flags, ptr, info.size, stringBuffer, conversion, json);
}


bool cISDataMappings::VariableToString(eDataType dataType, eDataFlags dataFlags, const uint8_t* dataBuffer, uint32_t dataSize, data_mapping_string_t stringBuffer, double conversion, bool json)
{
    int precision;
    float valuef32;
    double valuef64;
    switch (dataType)
    {
    case DATA_TYPE_INT8:
        if (dataFlags & DATA_FLAGS_DISPLAY_HEX) SNPRINTF(stringBuffer, IS_DATA_MAPPING_MAX_STRING_LENGTH, "0x%02X", *(int8_t*)dataBuffer);
        else                                    SNPRINTF(stringBuffer, IS_DATA_MAPPING_MAX_STRING_LENGTH, "%d", (int)*(int8_t*)dataBuffer);
        break;
    case DATA_TYPE_UINT8:
        if (dataFlags & DATA_FLAGS_DISPLAY_HEX) SNPRINTF(stringBuffer, IS_DATA_MAPPING_MAX_STRING_LENGTH, "0x%02X", *(uint8_t*)dataBuffer);
        else                                    SNPRINTF(stringBuffer, IS_DATA_MAPPING_MAX_STRING_LENGTH, "%u", (unsigned int)*(uint8_t*)dataBuffer);
        break;
    case DATA_TYPE_INT16:
        if (dataFlags & DATA_FLAGS_DISPLAY_HEX) SNPRINTF(stringBuffer, IS_DATA_MAPPING_MAX_STRING_LENGTH, "0x%04X", *(int16_t*)dataBuffer);
        else                                    SNPRINTF(stringBuffer, IS_DATA_MAPPING_MAX_STRING_LENGTH, "%d", (int)*(int16_t*)dataBuffer);
        break;
    case DATA_TYPE_UINT16:
        if (dataFlags & DATA_FLAGS_DISPLAY_HEX) SNPRINTF(stringBuffer, IS_DATA_MAPPING_MAX_STRING_LENGTH, "0x%04X", *(uint16_t*)dataBuffer);
        else                                    SNPRINTF(stringBuffer, IS_DATA_MAPPING_MAX_STRING_LENGTH, "%u", (unsigned int)*(uint16_t*)dataBuffer);
        break;
    case DATA_TYPE_INT32:
        if (dataFlags & DATA_FLAGS_DISPLAY_HEX) SNPRINTF(stringBuffer, IS_DATA_MAPPING_MAX_STRING_LENGTH, "0x%08X", *(int32_t*)dataBuffer);
        else                                    SNPRINTF(stringBuffer, IS_DATA_MAPPING_MAX_STRING_LENGTH, "%d", (int)*(int32_t*)dataBuffer);
        break;
    case DATA_TYPE_UINT32:
        if (dataFlags & DATA_FLAGS_DISPLAY_HEX) SNPRINTF(stringBuffer, IS_DATA_MAPPING_MAX_STRING_LENGTH, "0x%08X", *(uint32_t*)dataBuffer);
        else                                    SNPRINTF(stringBuffer, IS_DATA_MAPPING_MAX_STRING_LENGTH, "%u", (unsigned int)*(uint32_t*)dataBuffer);        
        break;
    case DATA_TYPE_INT64:
        if (dataFlags & DATA_FLAGS_DISPLAY_HEX) SNPRINTF(stringBuffer, IS_DATA_MAPPING_MAX_STRING_LENGTH, "0x%016llX", (long long)*(uint64_t*)dataBuffer);
        else                                    SNPRINTF(stringBuffer, IS_DATA_MAPPING_MAX_STRING_LENGTH, "%lld", (long long)*(int64_t*)dataBuffer);
        break;
    case DATA_TYPE_UINT64:
        if (dataFlags & DATA_FLAGS_DISPLAY_HEX) SNPRINTF(stringBuffer, IS_DATA_MAPPING_MAX_STRING_LENGTH, "0x%016llX", (unsigned long long)*(uint64_t*)dataBuffer);
        else                                    SNPRINTF(stringBuffer, IS_DATA_MAPPING_MAX_STRING_LENGTH, "%llu", (unsigned long long)*(uint64_t*)dataBuffer);
        break;
    case DATA_TYPE_F32:
        precision = (dataFlags&DATA_FLAGS_FIXED_DECIMAL_MASK);
        valuef32 = (*(float*)dataBuffer) * conversion;
        if (precision)
        {
            precision -= 1;
            SNPRINTF(stringBuffer, IS_DATA_MAPPING_MAX_STRING_LENGTH, "%.*f", precision, valuef32);
        }
        else
        {
            precision -= 1;
            SNPRINTF(stringBuffer, IS_DATA_MAPPING_MAX_STRING_LENGTH, "%.8g", valuef32);
        }
        break;
    case DATA_TYPE_F64:                             
        precision = (dataFlags&DATA_FLAGS_FIXED_DECIMAL_MASK);
        valuef64 = (*(double*)dataBuffer) * conversion;
        if (precision)
        {   // Fixed precision
            SNPRINTF(stringBuffer, IS_DATA_MAPPING_MAX_STRING_LENGTH, "%.*f", precision, valuef64);
        }
        else
        {   // Variable precision
            SNPRINTF(stringBuffer, IS_DATA_MAPPING_MAX_STRING_LENGTH, "%.8g", valuef64);
        }
        break;

    case DATA_TYPE_STRING:
    {
        int n = 0;
        char* bufPtr2 = (char*)(dataBuffer);
        char* bufPtrEnd = bufPtr2 + _MIN(IS_DATA_MAPPING_MAX_STRING_LENGTH, dataSize) - 1;
        for (; bufPtr2 < bufPtrEnd && *bufPtr2 != '\0'; bufPtr2++)
        {
            if (json)
            {
                if (IS_JSON_ESCAPE_CHAR(*bufPtr2))
                {
                    if (bufPtr2 < bufPtrEnd - 1)
                    {
                        stringBuffer[n++] = '\\';
                    }
                    else
                    {
                        break;
                    }
                }
            }
            stringBuffer[n++] = *bufPtr2;
        }
        stringBuffer[n++] = '\0';
    } break;

    case DATA_TYPE_BINARY:
    {
        size_t hexIndex = 1;
        if (json)
        {
            stringBuffer[0] = '"';
        }
        // convert to hex
        const unsigned char* hexTable = getHexLookupTable();
        for (size_t i = 0; i < dataSize; i++)
        {
            stringBuffer[hexIndex++] = hexTable[0x0F & (dataBuffer[i] >> 4)];
            stringBuffer[hexIndex++] = hexTable[0x0F & dataBuffer[i]];
        }
        if (json)
        {
            stringBuffer[hexIndex++] = '"';
        }
        stringBuffer[hexIndex] = '\0';
    } break;

    default:
        stringBuffer[0] = '\0';
        return false;
    }
    return true;
}

bool cISDataMappings::DataToYaml(int did, const uint8_t* dataPtr, YAML::Node& output)
{
    return DataToYaml(did, dataPtr, output, YAML::Node());
}

bool cISDataMappings::DataToYaml(int did, const uint8_t* dataPtr, YAML::Node& output, const YAML::Node& filter)
{
    const auto& dataSetMap = *NameToInfoMap(did);
    data_mapping_string_t stringBuffer;
    const std::string didName = cISDataMappings::DataName(did);
    const YAML::Node didFilter = filter[didName];

    bool showAll = !filter || filter.IsNull() || !didFilter || didFilter.IsNull();
    std::set<std::string> requestedFields;

    if (!showAll && didFilter.IsMap())
    {
        for (const auto& entry : didFilter)
        {
            requestedFields.insert(entry.first.as<std::string>());
        }
    }

    YAML::Node map(YAML::NodeType::Map);

    for (const auto& entry : dataSetMap)
    {
        const data_info_t& info = entry.second;

        if (!showAll && requestedFields.find(info.name) == requestedFields.end())
        {
            continue;
        }

        if (info.arraySize > 0)
        {
            YAML::Node arr(YAML::NodeType::Sequence);
            arr.SetStyle(YAML::EmitterStyle::Flow);     // Make array display on one line
            for (unsigned int i = 0; i < info.arraySize; ++i)
            {
                if (!DataToString(info, nullptr, dataPtr, stringBuffer, i))
                {
                    return false;
                }
                arr.push_back(stringBuffer);
            }
            if (arr.size() > 0)
            {
                map[info.name] = arr;
            }
        }
        else
        {
            if (!DataToString(info, nullptr, dataPtr, stringBuffer))
            {
                return false;
            }

            map[info.name] = stringBuffer;
        }
    }

    if (map.size() > 0)
    {
        output[didName] = map;
        return true;
    }

    return false;
}

bool cISDataMappings::YamlToData(int did, const YAML::Node& yaml, uint8_t* dataPtr, std::vector<MemoryUsage>* usageVec)
{
    const std::string didName = std::string(cISDataMappings::DataName(did));
    const auto& dataSetMap = *NameToInfoMap(did);
    const YAML::Node& map = yaml[didName];
    bool success = false;

    if (!map || !map.IsMap()) {
        return false;
    }

    if (usageVec)
    {
        usageVec->clear();
    }

    for (const auto& kv : map)
    {
        const std::string name = kv.first.as<std::string>();
        const YAML::Node& valueNode = kv.second;

        auto it = dataSetMap.find(name);
        if (it == dataSetMap.end()) continue;

        const data_info_t& info = it->second;

        if (info.arraySize > 0)
        {
            if (!valueNode.IsSequence()) continue;

            size_t count = _MIN(static_cast<size_t>(info.arraySize), valueNode.size());
            for (size_t i = 0; i < count; ++i)
            {
                if (valueNode[i].IsNull()) continue;
                std::string valStr = valueNode[i].as<std::string>();
                // std::cout << "Parsing array " << name << "[" << i << "]: " << valStr << std::endl;
                if (!StringToData(valStr.c_str(), static_cast<int>(valStr.length()), nullptr, dataPtr, info, static_cast<unsigned int>(i)))
                {
                    return false;
                }
                if (usageVec)
                {
                    AppendMemoryUsage(*usageVec, dataPtr + info.offset + i * info.elementSize, info.elementSize);
                }
                success = true;
            }
        }
        else
        {
            std::string valStr = valueNode.as<std::string>();
            // std::cout << "Parsing " << name << ": " << valStr << std::endl;
            if (!StringToData(valStr.c_str(), static_cast<int>(valStr.length()), nullptr, dataPtr, info))
            {
                return false;
            }
            if (usageVec)
            {
                AppendMemoryUsage(*usageVec, dataPtr + info.offset, info.size);
            }
            success = true;
        }
    }

    return success;
}

void cISDataMappings::AppendMemoryUsage(std::vector<MemoryUsage>& usageVec, void* newPtr, size_t newSize)
{
    uint8_t* newPtr8 = static_cast<uint8_t*>(newPtr);
    uint8_t* newEnd = newPtr8 + newSize;

    // First, try to merge with any existing overlapping or contiguous region
    for (size_t i = 0; i < usageVec.size(); ++i)
    {
        MemoryUsage& existing = usageVec[i];
        uint8_t* existingEnd = existing.end();

        // Check for overlap or contiguous range
        if (!(newEnd < existing.ptr || newPtr8 > existingEnd))
        {
            // Expand existing range to include new range
            uint8_t* newStart = _MIN(existing.ptr, newPtr8);
            uint8_t* mergedEnd = _MAX(existingEnd, newEnd);

            existing.ptr = newStart;
            existing.size = mergedEnd - newStart;

            // Now check for any other blocks that may overlap with the new merged one
            for (size_t j = 0; j < usageVec.size(); )
            {
                if (j != i)
                {
                    MemoryUsage& other = usageVec[j];
                    if (!(existing.end() < other.ptr || existing.ptr > other.end()))
                    {
                        // Merge this block too
                        uint8_t* mergedStart = _MIN(existing.ptr, other.ptr);
                        uint8_t* mergedEnd = _MAX(existing.end(), other.end());

                        existing.ptr = mergedStart;
                        existing.size = mergedEnd - mergedStart;

                        usageVec.erase(usageVec.begin() + j);
                        if (j < i) --i;  // Adjust i if we removed an earlier element
                        continue;
                    }
                }
                ++j;
            }

            return;  // Done: merged successfully
        }
    }

    // No overlap, add as new entry
    usageVec.push_back({ newPtr8, newSize });
}

void splitStringMulti(const std::string& input, const std::string& delimiters, std::vector<std::string>& output)
{
    size_t start = input.find_first_not_of(delimiters), end = 0;
    while ((end = input.find_first_of(delimiters, start)) != std::string::npos)
    {
        output.push_back(input.substr(start, end - start));
        start = input.find_first_not_of(delimiters, end);
    }
    if (start != std::string::npos)
        output.push_back(input.substr(start));
}

bool cISDataMappings::DidBufferToString(int did, const uint8_t* dataPtr, string &output, std::string fields)
{
    std::ostringstream oss, ossHdr;
    const map_name_to_info_t& dataSetMap = *NameToInfoMap(did);
    data_mapping_string_t stringBuffer;

    // Build a map: baseFieldName → list of requested indices (-1 = entire array)
    std::map<std::string, std::set<int>> requested;
    bool showAll = fields.empty();

    if (!showAll)
    {
        std::vector<std::string> splitFields;
        splitStringMulti(fields, "\n,|", splitFields);

        for (std::string f : splitFields)
        {
            std::string base = f;
            int index = ExtractArrayIndex(base);    // base is mutated to strip [n]
            if (index >= 0)
            {
                requested[base].insert(index);      // Request specific element
            }
            else
            {
                requested[base].insert(-1);         // Special marker: request full array or scalar
            }
        }
    }

    for (const auto& entry : dataSetMap)
    {
        const data_info_t& info = entry.second;

        if (!showAll)
        {
            auto it = requested.find(info.name);
            if (it == requested.end())
            {
                continue;  // Not requested
            }

            const std::set<int>& indices = it->second;

            if (info.arraySize > 0)
            {
                if (indices.count(-1))  // Full array requested
                {
                    for (int i = 0; i < int(info.arraySize); i++)
                    {
                        if (DataToString(info, nullptr, dataPtr, stringBuffer, i))
                        {
                            oss << info.name << "[" << i << "] = " << stringBuffer << std::endl;
                        }
                    }
                }
                else
                {
                    for (int idx : indices)
                    {
                        if (idx < 0 || idx >= int(info.arraySize))
                        {
                            oss << info.name << "[" << idx << "] = <invalid index>" << std::endl;
                            continue;
                        }

                        if (DataToString(info, nullptr, dataPtr, stringBuffer, idx))
                        {
                            oss << info.name << "[" << idx << "] = " << stringBuffer << std::endl;
                        }
                    }
                }
            }
            else
            {
                if (DataToString(info, nullptr, dataPtr, stringBuffer))
                {
                    oss << info.name << " = " << stringBuffer << std::endl;
                }
            }
        }
        else
        {
            // No filtering, dump all
            if (info.arraySize > 0)
            {
                for (int i = 0; i < int(info.arraySize); i++)
                {
                    if (DataToString(info, nullptr, dataPtr, stringBuffer, i))
                    {
                        oss << info.name << "[" << i << "] = " << stringBuffer << std::endl;
                    }
                }
            }
            else
            {
                if (DataToString(info, nullptr, dataPtr, stringBuffer))
                {
                    oss << info.name << " = " << stringBuffer << std::endl;
                }
            }
        }
    }

    if (!oss.str().empty())
    {
        ossHdr << DataName(did) << " (" << did << ")" << std::endl;
        output = ossHdr.str() + oss.str();
        return true;
    }

    return false;
}

bool cISDataMappings::StringToDidBuffer(int did, const std::string& fields, uint8_t* dataPtr)
{
    const map_name_to_info_t& dataSetMap = *NameToInfoMap(did);
    if (!dataPtr || dataSetMap.empty())
    {
        return false;
    }

    bool success = false;
    std::vector<std::string> keyValues;
    splitStringMulti(fields, "\n,|", keyValues);

    for (const std::string& keyValue : keyValues)
    {
        std::vector<std::string> pair;
        splitString(keyValue, '=', pair);
        if (pair.size() != 2)
        {
            continue; // Invalid format, skip
        }

        std::string fieldName = pair[0];
        std::string value = pair[1];

        // Remove all trailing whitespace
        fieldName.erase(fieldName.find_last_not_of(" \t\n\r\f\v") + 1);
        value.erase(value.find_last_not_of(" \t\n\r\f\v") + 1);

        // Extract array index if specified (e.g. theta[1])
        int arrayIndex = ExtractArrayIndex(fieldName);

        auto it = dataSetMap.find(fieldName);
        if (it == dataSetMap.end())
        {
            // Unknown field
            continue;
        }

        const data_info_t& info = it->second;

        // Check array bounds if applicable
        if (arrayIndex >= int(info.arraySize))
        {
            if (info.arraySize > 0)
            {   // Array index out of bounds
                // cout << "Invalid array index for " << info.name << ": " << arrayIndex << std::endl;
                return false;
            }
            else
            {   // If no array size specified, default to 0
                arrayIndex = 0; 
            }
        }

        // Clean up value (optional: remove "0x" if present)
        if (value.size() > 2 && value[0] == '0' && (value[1] == 'x' || value[1] == 'X'))
        {
            value = value.substr(2);
        }

        // Write to field
        bool fieldSuccess = StringToData(
            value.c_str(), int(value.length()),
            nullptr,
            dataPtr,
            info,
            _MAX(0, arrayIndex)
        );

        success |= fieldSuccess;
    }

    return success;
}


// Return the index into an array if specified and remove from string.  i.e. `insOffset[2]` returns 2 and str is reduced to `insOffset`.
int cISDataMappings::ExtractArrayIndex(std::string& str)
{
    int arrayIndex = -1;
    size_t openBracketPos  = str.find('[');
    size_t closeBracketPos = str.find(']');

    if (openBracketPos != std::string::npos && closeBracketPos != std::string::npos && openBracketPos < closeBracketPos)
    {
        std::string indexStr = str.substr(openBracketPos + 1, closeBracketPos - openBracketPos - 1);
        arrayIndex = std::stoi(indexStr);
        str = str.substr(0, openBracketPos);  // Strip the "[n]" part from the name
    }

    return arrayIndex;
}


double cISDataMappings::Timestamp(const p_data_hdr_t* hdr, const uint8_t* buf)
{
    if (hdr == NULL || buf == NULL || hdr->id == 0 || hdr->id >= DID_COUNT || hdr->size == 0)
    {
        return 0.0;
    }
    
    // raw data types with observation use a custom timestamp function
    if (hdr->id == DID_GPS1_RAW || hdr->id == DID_GPS2_RAW || hdr->id == DID_GPS_BASE_RAW)
    {
        gps_raw_t* raw = (gps_raw_t*)buf;
        if (raw->dataType == eRawDataType::raw_data_type_observation && raw->obsCount>0)
        {
            const obsd_t& obs = raw->data.obs[0];
            return obs.time.sec + (double)obs.time.time;
        }
        return 0.0;
    }

    data_set_t* ds = DataSet(hdr->id);
    if (!ds) { return 0; }
    if (ds->timestampFields != NULLPTR)
    {
        const uint8_t* ptr = FieldData(*ds->timestampFields, 0, hdr, (uint8_t*)buf);
        if (ptr)
        {
            if (ds->timestampFields->type == DATA_TYPE_F64)
            {   // field is seconds, use as is
                return protectUnalignedAssign<double>((void *)ptr);
            }
            else if (ds->timestampFields->type == DATA_TYPE_UINT32)
            {   // field is milliseconds, convert to seconds
                return 0.001 * (*(uint32_t*)ptr);
            }
        }
    }
    return 0.0;
}

double cISDataMappings::TimestampOrCurrentTime(const p_data_hdr_t* hdr, const uint8_t* buf)
{
    double timestamp = Timestamp(hdr, buf);
    if (timestamp==0.0) { timestamp = current_timeSecD(); }
    return timestamp;
}

const uint8_t* cISDataMappings::FieldData(const data_info_t& info, uint32_t arrayIndex, const p_data_hdr_t* hdr, const uint8_t* buf)
{
    if (info.arraySize && arrayIndex >= info.arraySize)
    {
        return NULL;
    }

    if (buf == NULL)
    {
        return NULL;
    }

    if (hdr == NULL)
    {   // Assume buf is large enough for the full data structure
        return buf + info.offset + arrayIndex*info.elementSize;
    }

    int32_t fullSize = (hdr->size == 0 ? DataSize(hdr->id) : hdr->size);
    int32_t offset = (int32_t)info.offset + arrayIndex*info.elementSize - (int32_t)hdr->offset;
    if ((offset >= 0) && (offset < fullSize))
    {
        return buf + offset;
    }

    return NULL;
}
