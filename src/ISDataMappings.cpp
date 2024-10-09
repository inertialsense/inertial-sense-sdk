/*
MIT LICENSE

Copyright (c) 2014-2024 Inertial Sense, Inc. - http://inertialsense.com

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

#include "ISDataMappings.h"
#include "DataJSON.h"
#include "ISUtilities.h"
#include "ISConstants.h"
#include "data_sets.h"

// #define USE_IS_INTERNAL

#ifdef USE_IS_INTERNAL
#include "../../cpp/libs/families/imx/IS_internal.h"
#include "../../cpp/libs/families/imx/ISDataMappingsInternal.h"
#endif

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

#if PLATFORM_IS_EMBEDDED
cISDataMappings* cISDataMappings::s_map;
#else
cISDataMappings cISDataMappings::s_map;
#endif

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
    const map_name_to_info_t& offsetMap = data_set[did].nameInfo;

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
    mapper.AddArray("hardwareVer", &dev_info_t::hardwareVer, DATA_TYPE_UINT8, 4, "", "Hardware version", DATA_FLAGS_READ_ONLY);
    mapper.AddArray("firmwareVer", &dev_info_t::firmwareVer, DATA_TYPE_UINT8, 4, "", "Firmware version", DATA_FLAGS_READ_ONLY);
    mapper.AddMember("buildNumber", &dev_info_t::buildNumber, DATA_TYPE_UINT32, "", "Build number (0xFFFFF000 = Host key, 0x00000FFF = Build #)", DATA_FLAGS_READ_ONLY | DATA_FLAGS_DISPLAY_HEX);
    mapper.AddArray("protocolVer", &dev_info_t::protocolVer, DATA_TYPE_UINT8, 4, "", "Communications protocol version", DATA_FLAGS_READ_ONLY);
    mapper.AddMember("repoRevision", &dev_info_t::repoRevision, DATA_TYPE_UINT32, "", "Repo revision", DATA_FLAGS_READ_ONLY | DATA_FLAGS_DISPLAY_HEX);
    mapper.AddMember("manufacturer", &dev_info_t::manufacturer, DATA_TYPE_STRING, "", "manufacturer", DATA_FLAGS_READ_ONLY);
    mapper.AddMember("buildType", &dev_info_t::buildType, DATA_TYPE_UINT8, "", "'a'(97)=ALPHA, 'b'(98)=BETA, 'c'(99)=CANDIDATE, 'r'(114)=PRODUCTION, 'd'(100)=develop, 's'(115)=snapshot, '*'(42)=dirty", DATA_FLAGS_READ_ONLY);
    mapper.AddMember("buildYear", &dev_info_t::buildYear, DATA_TYPE_UINT8, "", "Build year-2000", DATA_FLAGS_READ_ONLY);
    mapper.AddMember("buildMonth", &dev_info_t::buildMonth, DATA_TYPE_UINT8, "", "Build month", DATA_FLAGS_READ_ONLY);
    mapper.AddMember("buildDay", &dev_info_t::buildDay, DATA_TYPE_UINT8, "", "Build day", DATA_FLAGS_READ_ONLY);
    mapper.AddMember("buildHour", &dev_info_t::buildHour, DATA_TYPE_UINT8, "", "Build hour", DATA_FLAGS_READ_ONLY);
    mapper.AddMember("buildMinute", &dev_info_t::buildMinute, DATA_TYPE_UINT8, "", "Build minute", DATA_FLAGS_READ_ONLY);
    mapper.AddMember("buildSecond", &dev_info_t::buildSecond, DATA_TYPE_UINT8, "", "Build second", DATA_FLAGS_READ_ONLY);
    mapper.AddMember("buildMillisecond", &dev_info_t::buildMillisecond, DATA_TYPE_UINT8, "", "Build millisecond", DATA_FLAGS_READ_ONLY);
    mapper.AddMember("addInfo", &dev_info_t::addInfo, DATA_TYPE_STRING, "", "Additional info", DATA_FLAGS_READ_ONLY);
}

static void PopulateMapHdwParams(data_set_t data_set[DID_COUNT], uint32_t did)
{
#if 0
    DataMapper<hdw_params_t> mapper(data_set, did);
    mapper.AddMember("timeOfWeekMs", &hdw_params_t::timeOfWeekMs, DATA_TYPE_UINT32, "ms", "Time of week since Sunday morning, GMT", DATA_FLAGS_READ_ONLY);
    mapper.AddMember("I.pqrDot", &hdw_params_t::I.pqrDot, DATA_TYPE_F32, SYM_DEG_PER_S + "/s", "IMU1 PQR derivative", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3, C_RAD2DEG);
    mapper.AddMember("I.accDot", &hdw_params_t::I.accDot, DATA_TYPE_F32, SYM_M_PER_S_2 + "/s", "IMU1 accel derivative", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3);
    mapper.AddMember("I.pqrSigma", &hdw_params_t::I.pqrSigma, DATA_TYPE_F32, SYM_DEG_PER_S, "Max standard deviation", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3, C_RAD2DEG);
    mapper.AddMember("I.accSigma", &hdw_params_t::I.accSigma, DATA_TYPE_F32, SYM_M_PER_S_2, "Max standard deviation", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3);
    mapper.AddMember("I.pqrMean", &hdw_params_t::I.mean.pqr, DATA_TYPE_F32, SYM_DEG_PER_S, "IMU1 angular rate average", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_2, C_RAD2DEG, ERROR_THRESH_PQR);
    mapper.AddMember("I.accMean", &hdw_params_t::I.mean.acc, DATA_TYPE_F32, SYM_M_PER_S_2, "IMU1 linear acceleration average", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3, 1, ERROR_THRESH_ACC, DECOR_QCOLOR_YELLOW);

    mapper.AddMember("I.pqrDRef[0]", &hdw_params_t::I.dref.pqr[0], DATA_TYPE_F32, SYM_DEG_PER_S, "IMU1 angular rate delta reference IMU", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_2, C_RAD2DEG, ERROR_THRESH_PQR);
    mapper.AddMember("I.pqrDRef[1]", &hdw_params_t::I.dref.pqr[1], DATA_TYPE_F32, SYM_DEG_PER_S, "IMU1 angular rate delta reference IMU", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_2, C_RAD2DEG, ERROR_THRESH_PQR);
    mapper.AddMember("I.pqrDRef[2]", &hdw_params_t::I.dref.pqr[2], DATA_TYPE_F32, SYM_DEG_PER_S, "IMU1 angular rate delta reference IMU", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_2, C_RAD2DEG, ERROR_THRESH_PQR);
    storage->setMeanErrMode(3, Parcel::MEAN_ERR_MODE_ABS, 0.3f * C_DEG2RAD_F);      // rad/s
    mapper.AddMember("I.accDRef[0]", &hdw_params_t::I.dref.acc[0], DATA_TYPE_F32, SYM_M_PER_S_2, "IMU1 linear acceleration delta reference IMU", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3, 1, ERROR_THRESH_ACC);
    mapper.AddMember("I.accDRef[1]", &hdw_params_t::I.dref.acc[1], DATA_TYPE_F32, SYM_M_PER_S_2, "IMU1 linear acceleration delta reference IMU", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3, 1, ERROR_THRESH_ACC);
    mapper.AddMember("I.accDRef[2]", &hdw_params_t::I.dref.acc[2], DATA_TYPE_F32, SYM_M_PER_S_2, "IMU1 linear acceleration delta reference IMU", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3, 1, ERROR_THRESH_ACC);
    storage->setMeanErrMode(3, Parcel::MEAN_ERR_MODE_ABS, 0.02f);                   // m/s^2

    mapper.AddMember("gpsCnoSigma", &hdw_params_t:gpsCnoSigma, DATA_TYPE_F32, "dBHz", "GPS CNO max standard deviation", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3);
    mapper.AddMember("gpsCnoMean", &hdw_params_t:gpsCnoMean, DATA_TYPE_F32, "dBHz", "GPS CNO average", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3);
    mapper.AddMember("barMslDot", &hdw_params_t:barMslDot, DATA_TYPE_F32, "m/s", "Barometer derivative", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3);
    mapper.AddMember("barMslSigma", &hdw_params_t:barMslSigma, DATA_TYPE_F32, "m", "Barometer standard deviation", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3);
    mapper.AddMember("barMslMean", &hdw_params_t:barMslMean, DATA_TYPE_F32, "m", "Barometer CNO average", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3, 1, ERROR_THRESH_BAR);
    mapper.AddArray("mag", &hdw_params_t::mag, DATA_TYPE_F32, 3, "", "Magnetometer", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_2, 1, ERROR_THRESH_MAG);
#endif
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
    mapper.AddArray("uid", &manufacturing_info_t::uid, DATA_TYPE_UINT32, 4, "", "Unique microcontroller identifier", DATA_FLAGS_READ_ONLY);
}

static void PopulateMapIO(data_set_t data_set[DID_COUNT], uint32_t did)
{
    DataMapper<io_t> mapper(data_set, did);
    mapper.AddMember("timeOfWeekMs", &io_t::timeOfWeekMs, DATA_TYPE_UINT32, "ms", "Time of week since Sunday morning, GMT", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_4);
    mapper.AddMember("gpioStatus", &io_t::gpioStatus, DATA_TYPE_UINT32, "", "Use to read and control GPIO input and output.", DATA_FLAGS_DISPLAY_HEX);
}

static void PopulateMapBit(data_set_t data_set[DID_COUNT], uint32_t did)
{
    DataMapper<bit_t> mapper(data_set, did);
    mapper.AddMember("command", &bit_t::command, DATA_TYPE_UINT8, "", "[cmd: " TOSTRING(BIT_CMD_FULL_STATIONARY) "=start full, " TOSTRING(BIT_CMD_BASIC_MOVING) "=start basic, " TOSTRING(BIT_CMD_FULL_STATIONARY_HIGH_ACCURACY) "=start full HA, " TOSTRING(BIT_CMD_OFF) "=off]");
    mapper.AddMember("lastCommand", &bit_t::lastCommand, DATA_TYPE_UINT8, "", "Last input command", DATA_FLAGS_READ_ONLY);
    mapper.AddMember("state", &bit_t::state, DATA_TYPE_UINT8, "", "[state: " TOSTRING(BIT_STATE_RUNNING) "=running " TOSTRING(BIT_STATE_DONE) "=done]", DATA_FLAGS_READ_ONLY);
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
    mapper.AddMember("testMode", &bit_t::testMode, DATA_TYPE_UINT8, "", "Test Mode: " TOSTRING(BIT_TEST_MODE_SIM_GPS_NOISE) "=GPS noise, " TOSTRING(BIT_TEST_MODE_SERIAL_DRIVER_RX_OVERFLOW) "=Rx overflow, " TOSTRING(BIT_TEST_MODE_SERIAL_DRIVER_TX_OVERFLOW) "=Tx overflow");
    mapper.AddMember("testVar", &bit_t::testVar, DATA_TYPE_UINT8, "", "Test Mode variable (port number)", DATA_FLAGS_READ_ONLY);
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
    INIT_MAP(port_monitor_t, did);
    ADD_MAP_7("activePorts", activePorts, DATA_TYPE_UINT8, uint8_t, "", "Number of active ports", DATA_FLAGS_READ_ONLY);    

    ADD_MAP_7("[0].portInfo",           port[0].portInfo, DATA_TYPE_UINT8, uint8_t, "", "High nib port type (see ePortMonPortType) low nib index.", DATA_FLAGS_READ_ONLY | DATA_FLAGS_DISPLAY_HEX);
    ADD_MAP_7("[0].status",             port[0].status, DATA_TYPE_UINT32, uint32_t, "", "", DATA_FLAGS_DISPLAY_HEX);
    ADD_MAP_7("[0].txBytesPerSec",      port[0].txBytesPerSec, DATA_TYPE_UINT32, uint32_t, "bytes/s", "Tx data rate", DATA_FLAGS_READ_ONLY);
    ADD_MAP_7("[0].rxBytesPerSec",      port[0].rxBytesPerSec, DATA_TYPE_UINT32, uint32_t, "bytes/s", "Rx data rate", DATA_FLAGS_READ_ONLY);
    ADD_MAP_6("[0].txBytes",            port[0].txBytes, DATA_TYPE_UINT32, uint32_t, "bytes", "Tx byte count");
    ADD_MAP_6("[0].rxBytes",            port[0].rxBytes, DATA_TYPE_UINT32, uint32_t, "bytes", "Rx byte count");
    ADD_MAP_6("[0].txOverflows",        port[0].txOverflows, DATA_TYPE_UINT32, uint32_t, "", "Tx buffer overflow occurrences");			
    ADD_MAP_6("[0].rxOverflows",        port[0].rxOverflows, DATA_TYPE_UINT32, uint32_t, "", "Rx buffer overflow occurrences");        
    ADD_MAP_6("[0].txBytesDropped",     port[0].txBytesDropped, DATA_TYPE_UINT32, uint32_t, "bytes", "Tx number of bytes that were not sent");
    ADD_MAP_6("[0].rxChecksumErrors",   port[0].rxChecksumErrors, DATA_TYPE_UINT32, uint32_t, "", "Rx number of checksum failures");

    ADD_MAP_7("[1].portInfo",           port[1].portInfo, DATA_TYPE_UINT8, uint8_t, "", "High nib port type (see ePortMonPortType) low nib index.", DATA_FLAGS_READ_ONLY | DATA_FLAGS_DISPLAY_HEX);
    ADD_MAP_7("[1].status",             port[1].status, DATA_TYPE_UINT32, uint32_t, "", "", DATA_FLAGS_DISPLAY_HEX);
    ADD_MAP_7("[1].txBytesPerSec",      port[1].txBytesPerSec, DATA_TYPE_UINT32, uint32_t, "bytes/s", "Tx data rate", DATA_FLAGS_READ_ONLY);
    ADD_MAP_7("[1].rxBytesPerSec",      port[1].rxBytesPerSec, DATA_TYPE_UINT32, uint32_t, "bytes/s", "Rx data rate", DATA_FLAGS_READ_ONLY);
    ADD_MAP_6("[1].txBytes",            port[1].txBytes, DATA_TYPE_UINT32, uint32_t, "bytes", "Tx byte count");
    ADD_MAP_6("[1].rxBytes",            port[1].rxBytes, DATA_TYPE_UINT32, uint32_t, "bytes", "Rx byte count");
    ADD_MAP_6("[1].txOverflows",        port[1].txOverflows, DATA_TYPE_UINT32, uint32_t, "", "Tx buffer overflow occurrences");			
    ADD_MAP_6("[1].rxOverflows",        port[1].rxOverflows, DATA_TYPE_UINT32, uint32_t, "", "Rx buffer overflow occurrences");        
    ADD_MAP_6("[1].txBytesDropped",     port[1].txBytesDropped, DATA_TYPE_UINT32, uint32_t, "bytes", "Tx number of bytes that were not sent");
    ADD_MAP_6("[1].rxChecksumErrors",   port[1].rxChecksumErrors, DATA_TYPE_UINT32, uint32_t, "", "Rx number of checksum failures");

    ADD_MAP_7("[2].portInfo",           port[2].portInfo, DATA_TYPE_UINT8, uint8_t, "", "High nib port type (see ePortMonPortType) low nib index.", DATA_FLAGS_READ_ONLY | DATA_FLAGS_DISPLAY_HEX);
    ADD_MAP_7("[2].status",             port[2].status, DATA_TYPE_UINT32, uint32_t, "", "", DATA_FLAGS_DISPLAY_HEX);
    ADD_MAP_7("[2].txBytesPerSec",      port[2].txBytesPerSec, DATA_TYPE_UINT32, uint32_t, "bytes/s", "Tx data rate", DATA_FLAGS_READ_ONLY);
    ADD_MAP_7("[2].rxBytesPerSec",      port[2].rxBytesPerSec, DATA_TYPE_UINT32, uint32_t, "bytes/s", "Rx data rate", DATA_FLAGS_READ_ONLY);
    ADD_MAP_6("[2].txBytes",            port[2].txBytes, DATA_TYPE_UINT32, uint32_t, "bytes", "Tx byte count");
    ADD_MAP_6("[2].rxBytes",            port[2].rxBytes, DATA_TYPE_UINT32, uint32_t, "bytes", "Rx byte count");
    ADD_MAP_6("[2].txOverflows",        port[2].txOverflows, DATA_TYPE_UINT32, uint32_t, "", "Tx buffer overflow occurrences");			
    ADD_MAP_6("[2].rxOverflows",        port[2].rxOverflows, DATA_TYPE_UINT32, uint32_t, "", "Rx buffer overflow occurrences");        
    ADD_MAP_6("[2].txBytesDropped",     port[2].txBytesDropped, DATA_TYPE_UINT32, uint32_t, "bytes", "Tx number of bytes that were not sent");
    ADD_MAP_6("[2].rxChecksumErrors",   port[2].rxChecksumErrors, DATA_TYPE_UINT32, uint32_t, "", "Rx number of checksum failures");

    ADD_MAP_7("[3].portInfo",           port[3].portInfo, DATA_TYPE_UINT8, uint8_t, "", "High nib port type (see ePortMonPortType) low nib index.", DATA_FLAGS_READ_ONLY | DATA_FLAGS_DISPLAY_HEX);
    ADD_MAP_7("[3].status",             port[3].status, DATA_TYPE_UINT32, uint32_t, "", "", DATA_FLAGS_DISPLAY_HEX);
    ADD_MAP_7("[3].txBytesPerSec",      port[3].txBytesPerSec, DATA_TYPE_UINT32, uint32_t, "bytes/s", "Tx data rate", DATA_FLAGS_READ_ONLY);
    ADD_MAP_7("[3].rxBytesPerSec",      port[3].rxBytesPerSec, DATA_TYPE_UINT32, uint32_t, "bytes/s", "Rx data rate", DATA_FLAGS_READ_ONLY);
    ADD_MAP_6("[3].txBytes",            port[3].txBytes, DATA_TYPE_UINT32, uint32_t, "bytes", "Tx byte count");
    ADD_MAP_6("[3].rxBytes",            port[3].rxBytes, DATA_TYPE_UINT32, uint32_t, "bytes", "Rx byte count");
    ADD_MAP_6("[3].txOverflows",        port[3].txOverflows, DATA_TYPE_UINT32, uint32_t, "", "Tx buffer overflow occurrences");			
    ADD_MAP_6("[3].rxOverflows",        port[3].rxOverflows, DATA_TYPE_UINT32, uint32_t, "", "Rx buffer overflow occurrences");        
    ADD_MAP_6("[3].txBytesDropped",     port[3].txBytesDropped, DATA_TYPE_UINT32, uint32_t, "bytes", "Tx number of bytes that were not sent");
    ADD_MAP_6("[3].rxChecksumErrors",   port[3].rxChecksumErrors, DATA_TYPE_UINT32, uint32_t, "", "Rx number of checksum failures");

    ADD_MAP_7("[4].portInfo",           port[4].portInfo, DATA_TYPE_UINT8, uint8_t, "", "High nib port type (see ePortMonPortType) low nib index.", DATA_FLAGS_READ_ONLY | DATA_FLAGS_DISPLAY_HEX);
    ADD_MAP_7("[4].status",             port[4].status, DATA_TYPE_UINT32, uint32_t, "", "", DATA_FLAGS_DISPLAY_HEX);
    ADD_MAP_7("[4].txBytesPerSec",      port[4].txBytesPerSec, DATA_TYPE_UINT32, uint32_t, "bytes/s", "Tx data rate", DATA_FLAGS_READ_ONLY);
    ADD_MAP_7("[4].rxBytesPerSec",      port[4].rxBytesPerSec, DATA_TYPE_UINT32, uint32_t, "bytes/s", "Rx data rate", DATA_FLAGS_READ_ONLY);
    ADD_MAP_6("[4].txBytes",            port[4].txBytes, DATA_TYPE_UINT32, uint32_t, "bytes", "Tx byte count");
    ADD_MAP_6("[4].rxBytes",            port[4].rxBytes, DATA_TYPE_UINT32, uint32_t, "bytes", "Rx byte count");
    ADD_MAP_6("[4].txOverflows",        port[4].txOverflows, DATA_TYPE_UINT32, uint32_t, "", "Tx buffer overflow occurrences");			
    ADD_MAP_6("[4].rxOverflows",        port[4].rxOverflows, DATA_TYPE_UINT32, uint32_t, "", "Rx buffer overflow occurrences");        
    ADD_MAP_6("[4].txBytesDropped",     port[4].txBytesDropped, DATA_TYPE_UINT32, uint32_t, "bytes", "Tx number of bytes that were not sent");
    ADD_MAP_6("[4].rxChecksumErrors",   port[4].rxChecksumErrors, DATA_TYPE_UINT32, uint32_t, "", "Rx number of checksum failures");

    ADD_MAP_7("[5].portInfo",           port[5].portInfo, DATA_TYPE_UINT8, uint8_t, "", "High nib port type (see ePortMonPortType) low nib index.", DATA_FLAGS_READ_ONLY | DATA_FLAGS_DISPLAY_HEX);
    ADD_MAP_7("[5].status",             port[5].status, DATA_TYPE_UINT32, uint32_t, "", "", DATA_FLAGS_DISPLAY_HEX);
    ADD_MAP_7("[5].txBytesPerSec",      port[5].txBytesPerSec, DATA_TYPE_UINT32, uint32_t, "bytes/s", "Tx data rate", DATA_FLAGS_READ_ONLY);
    ADD_MAP_7("[5].rxBytesPerSec",      port[5].rxBytesPerSec, DATA_TYPE_UINT32, uint32_t, "bytes/s", "Rx data rate", DATA_FLAGS_READ_ONLY);
    ADD_MAP_6("[5].txBytes",            port[5].txBytes, DATA_TYPE_UINT32, uint32_t, "bytes", "Tx byte count");
    ADD_MAP_6("[5].rxBytes",            port[5].rxBytes, DATA_TYPE_UINT32, uint32_t, "bytes", "Rx byte count");
    ADD_MAP_6("[5].txOverflows",        port[5].txOverflows, DATA_TYPE_UINT32, uint32_t, "", "Tx buffer overflow occurrences");			
    ADD_MAP_6("[5].rxOverflows",        port[5].rxOverflows, DATA_TYPE_UINT32, uint32_t, "", "Rx buffer overflow occurrences");        
    ADD_MAP_6("[5].txBytesDropped",     port[5].txBytesDropped, DATA_TYPE_UINT32, uint32_t, "bytes", "Tx number of bytes that were not sent");
    ADD_MAP_6("[5].rxChecksumErrors",   port[5].rxChecksumErrors, DATA_TYPE_UINT32, uint32_t, "", "Rx number of checksum failures");

    ASSERT_SIZE(totalSize);
}

void PopulateMapNmeaMsgs(data_set_t data_set[DID_COUNT], uint32_t did)
{
    INIT_MAP(nmea_msgs_t, did);
//    int flags = DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_4;
    ADD_MAP_7("options", options, DATA_TYPE_UINT32, uint32_t, "", "Port selection[0x0=current, 0xFF=all, 0x1=ser0, 0x2=ser1, 0x4=ser2, 0x8=USB]. (see RMC_OPTIONS_...)", DATA_FLAGS_DISPLAY_HEX);    
    // for( int i=0; i<MAX_nmeaBroadcastMsgPairs; i++)
    // {
    //     ADD_MAP_6("[0].ID",     nmeaBroadcastMsgs[0].msgID,     DATA_TYPE_UINT8, uint8_t, "", "NMEA_ID (See eNmeaAsciiMsgId)");
    //     ADD_MAP_6("[0].Period", nmeaBroadcastMsgs[0].msgPeriod, DATA_TYPE_UINT8, uint8_t, "", "NMEA_PerieNmeaMsgIdin multiples of 200ms. Ie value of 1 is 200ms or 5 is 1000ms/1s. A value of 0 stops the message broadcast.");
    // }
}

static void PopulateMapImu(data_set_t data_set[DID_COUNT], uint32_t did, string description)
{
    INIT_MAP(imu_t, did);
    ADD_MAP_7("time", time, DATA_TYPE_F64, double, "s", "Time since boot up", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_8("pqr[0]", I.pqr[0], DATA_TYPE_F32, float&, SYM_DEG_PER_S, "Angular rate.  " + description, DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_2, C_RAD2DEG);
    ADD_MAP_8("pqr[1]", I.pqr[1], DATA_TYPE_F32, float&, SYM_DEG_PER_S, "Angular rate.  " + description, DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_2, C_RAD2DEG);
    ADD_MAP_8("pqr[2]", I.pqr[2], DATA_TYPE_F32, float&, SYM_DEG_PER_S, "Angular rate.  " + description, DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_2, C_RAD2DEG);
    ADD_MAP_7("acc[0]", I.acc[0], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "Linear acceleration.  " + description, DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3);
    ADD_MAP_7("acc[1]", I.acc[1], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "Linear acceleration.  " + description, DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3);
    ADD_MAP_7("acc[2]", I.acc[2], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "Linear acceleration.  " + description, DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3);
    ADD_MAP_7("status", status, DATA_TYPE_UINT32, uint32_t, "", s_imuStatusDescription, DATA_FLAGS_DISPLAY_HEX);
    ASSERT_SIZE(totalSize);
}

static void PopulateMapImu3(data_set_t data_set[DID_COUNT], uint32_t did, string description)
{
    INIT_MAP(imu3_t, did);
    ADD_MAP_7("time", time, DATA_TYPE_F64, double, "s", "Time since boot up", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_7("status", status, DATA_TYPE_UINT32, uint32_t, "", s_imuStatusDescription, DATA_FLAGS_DISPLAY_HEX);
    ADD_MAP_8("I0.pqr[0]", I[0].pqr[0], DATA_TYPE_F32, float&, SYM_DEG_PER_S, "IMU 1 angular rate.  " + description, DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_2, C_RAD2DEG);
    ADD_MAP_8("I0.pqr[1]", I[0].pqr[1], DATA_TYPE_F32, float&, SYM_DEG_PER_S, "IMU 1 angular rate.  " + description, DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_2, C_RAD2DEG);
    ADD_MAP_8("I0.pqr[2]", I[0].pqr[2], DATA_TYPE_F32, float&, SYM_DEG_PER_S, "IMU 1 angular rate.  " + description, DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_2, C_RAD2DEG);
    ADD_MAP_7("I0.acc[0]", I[0].acc[0], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "IMU 1 linear acceleration.  " + description, DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3);
    ADD_MAP_7("I0.acc[1]", I[0].acc[1], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "IMU 1 linear acceleration.  " + description, DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3);
    ADD_MAP_7("I0.acc[2]", I[0].acc[2], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "IMU 1 linear acceleration.  " + description, DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3);
    ADD_MAP_8("I1.pqr[0]", I[1].pqr[0], DATA_TYPE_F32, float&, SYM_DEG_PER_S, "IMU 2 angular rate.  " + description, DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_2, C_RAD2DEG);
    ADD_MAP_8("I1.pqr[1]", I[1].pqr[1], DATA_TYPE_F32, float&, SYM_DEG_PER_S, "IMU 2 angular rate.  " + description, DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_2, C_RAD2DEG);
    ADD_MAP_8("I1.pqr[2]", I[1].pqr[2], DATA_TYPE_F32, float&, SYM_DEG_PER_S, "IMU 2 angular rate.  " + description, DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_2, C_RAD2DEG);
    ADD_MAP_7("I1.acc[0]", I[1].acc[0], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "IMU 2 linear acceleration.  " + description, DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3);
    ADD_MAP_7("I1.acc[1]", I[1].acc[1], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "IMU 2 linear acceleration.  " + description, DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3);
    ADD_MAP_7("I1.acc[2]", I[1].acc[2], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "IMU 2 linear acceleration.  " + description, DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3);
    ADD_MAP_8("I2.pqr[0]", I[2].pqr[0], DATA_TYPE_F32, float&, SYM_DEG_PER_S, "IMU 3 angular rate.  " + description, DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_2, C_RAD2DEG);
    ADD_MAP_8("I2.pqr[1]", I[2].pqr[1], DATA_TYPE_F32, float&, SYM_DEG_PER_S, "IMU 3 angular rate.  " + description, DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_2, C_RAD2DEG);
    ADD_MAP_8("I2.pqr[2]", I[2].pqr[2], DATA_TYPE_F32, float&, SYM_DEG_PER_S, "IMU 3 angular rate.  " + description, DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_2, C_RAD2DEG);
    ADD_MAP_7("I2.acc[0]", I[2].acc[0], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "IMU 3 linear acceleration.  " + description, DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3);
    ADD_MAP_7("I2.acc[1]", I[2].acc[1], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "IMU 3 linear acceleration.  " + description, DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3);
    ADD_MAP_7("I2.acc[2]", I[2].acc[2], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "IMU 3 linear acceleration.  " + description, DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3);
    ASSERT_SIZE(totalSize);
}

static void PopulateMapSysParams(data_set_t data_set[DID_COUNT], uint32_t did)
{
    DataMapper<sys_params_t> mapper(data_set, did);
    mapper.AddMember("timeOfWeekMs", &sys_params_t::timeOfWeekMs, DATA_TYPE_UINT32, "ms", "Time of week since Sunday morning, GMT", DATA_FLAGS_READ_ONLY);
    mapper.AddMember("insStatus", &sys_params_t::insStatus, DATA_TYPE_UINT32, "", s_insStatusDescription, DATA_FLAGS_DISPLAY_HEX | DATA_FLAGS_INS_STATUS);
    mapper.AddMember("hdwStatus", &sys_params_t::hdwStatus, DATA_TYPE_UINT32, "", s_hdwStatusDescription, DATA_FLAGS_DISPLAY_HEX);
    mapper.AddMember("imuTemp", &sys_params_t::imuTemp, DATA_TYPE_F32, "", "Sys status flags", DATA_FLAGS_DISPLAY_HEX);
    mapper.AddMember("baroTemp", &sys_params_t::baroTemp, DATA_TYPE_F32,  SYM_DEG_C, "IMU temperature", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_1);
    mapper.AddMember("mcuTemp", &sys_params_t::mcuTemp, DATA_TYPE_F32,  SYM_DEG_C, "Barometer temperature", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_1);
    mapper.AddMember("sysStatus", &sys_params_t::sysStatus, DATA_TYPE_UINT32,  SYM_DEG_C, "MCU temperature", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_1);
    mapper.AddMember("imuSamplePeriodMs", &sys_params_t::imuSamplePeriodMs, DATA_TYPE_UINT32, "ms", "IMU sample period. Zero disables sensor sampling.");
    mapper.AddMember("navOutputPeriodMs", &sys_params_t::navOutputPeriodMs, DATA_TYPE_UINT32, "ms", "Nav/AHRS filter ouput period.");
    mapper.AddMember("sensorTruePeriod", &sys_params_t::sensorTruePeriod, DATA_TYPE_F64, "ms", "Nav/AHRS filter update period.", DATA_FLAGS_READ_ONLY);
    mapper.AddMember("flashCfgChecksum", &sys_params_t::flashCfgChecksum, DATA_TYPE_UINT32, "us", "Actual sample period relative to GPS PPS.", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3, 1.0e6);
    mapper.AddMember("navUpdatePeriodMs", &sys_params_t::navUpdatePeriodMs, DATA_TYPE_UINT32,  "", "Flash config checksum used for synchronization", DATA_FLAGS_READ_ONLY);
    mapper.AddMember("genFaultCode", &sys_params_t::genFaultCode, DATA_TYPE_UINT32, "", "General fault code descriptor", DATA_FLAGS_DISPLAY_HEX);
    mapper.AddMember("upTime", &sys_params_t::upTime, DATA_TYPE_F64, "s", "Local time since startup.", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_1);
}

static void PopulateMapSysSensors(data_set_t data_set[DID_COUNT], uint32_t did)
{
    DataMapper<sys_sensors_t> mapper(data_set, did);
    mapper.AddMember("time", &sys_sensors_t::time, DATA_TYPE_F64, "s", "Time since boot up", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3);
    mapper.AddMember("temp", &sys_sensors_t::temp, DATA_TYPE_F32, SYM_DEG_C, "System temperature", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3);
    mapper.AddArray("pqr", &sys_sensors_t::pqr, DATA_TYPE_F32, 3, SYM_DEG_PER_S, "Angular rate", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_4, C_RAD2DEG);
    mapper.AddArray("acc", &sys_sensors_t::acc, DATA_TYPE_F32, 3, SYM_M_PER_S, "Linear acceleration", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_4);
    mapper.AddArray("mag", &sys_sensors_t::mag, DATA_TYPE_F32, 3, "", "Magnetometer normalized gauss", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_4);
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
    mapper.AddMember("bits", &rmc_t::bits, DATA_TYPE_UINT64, "", "", DATA_FLAGS_DISPLAY_HEX);
    mapper.AddMember("options", &rmc_t::options, DATA_TYPE_UINT32, "", "", DATA_FLAGS_DISPLAY_HEX);
}

void PopulateMapIns1(data_set_t data_set[DID_COUNT], uint32_t did)
{
    DataMapper<ins_1_t> mapper(data_set, did);
    uint32_t flags = DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3;
    mapper.AddMember("week", &ins_1_t::week, DATA_TYPE_UINT32, "week", "Weeks since Jan 6, 1980", flags );
    mapper.AddMember("timeOfWeek", &ins_1_t::timeOfWeek, DATA_TYPE_F64, "s", "Time of week since Sunday morning, GMT", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_4 );
    mapper.AddMember("insStatus", &ins_1_t::insStatus, DATA_TYPE_UINT32,  "", s_insStatusDescription, DATA_FLAGS_DISPLAY_HEX | DATA_FLAGS_INS_STATUS);
    mapper.AddMember("hdwStatus", &ins_1_t::hdwStatus, DATA_TYPE_UINT32,  "", s_hdwStatusDescription, DATA_FLAGS_DISPLAY_HEX);
    mapper.AddArray("theta", &ins_1_t::theta, DATA_TYPE_F32, 3, SYM_DEG, "Euler angle - roll",   flags | DATA_FLAGS_ANGLE, C_RAD2DEG); // ERROR_THRESH_ROLLPITCH);
    mapper.AddArray("uvw", &ins_1_t::uvw, DATA_TYPE_F32, 3, "m/s", "Velocity in body frame", flags);
    mapper.AddArray("ned", &ins_1_t::ned, DATA_TYPE_F32, 3, "m", "North offset from reference LLA", flags);
    mapper.AddArray("lla", &ins_1_t::lla, DATA_TYPE_F64, 3, SYM_DEG, "WGS84 coordinate - latitude", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_8);
}

static void PopulateMapIns2(data_set_t data_set[DID_COUNT], uint32_t did)
{
    DataMapper<ins_2_t> mapper(data_set, did);
    uint32_t flags = DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3;
    mapper.AddMember("week", &ins_2_t::week, DATA_TYPE_UINT32, "week", "Weeks since Jan 6, 1980", flags );
    mapper.AddMember("timeOfWeek", &ins_2_t::timeOfWeek, DATA_TYPE_F64, "s", "Time of week since Sunday morning, GMT", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_4 );
    mapper.AddMember("insStatus", &ins_2_t::insStatus, DATA_TYPE_UINT32,  "", s_insStatusDescription, DATA_FLAGS_DISPLAY_HEX | DATA_FLAGS_INS_STATUS);
    mapper.AddMember("hdwStatus", &ins_2_t::hdwStatus, DATA_TYPE_UINT32,  "", s_hdwStatusDescription, DATA_FLAGS_DISPLAY_HEX);
    mapper.AddArray("qn2b", &ins_2_t::qn2b, DATA_TYPE_F32, 4, "", "Quaternion body rotation with respect to NED: W, X, Y, Z", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_4);
    mapper.AddArray("uvw", &ins_2_t::uvw, DATA_TYPE_F32, 3, "m/s", "Velocity in body frame", flags);
    mapper.AddArray("lla", &ins_2_t::lla, DATA_TYPE_F64, 3, SYM_DEG_DEG_M, "WGS84 latitude, longitude, ellipsoid altitude", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_8);
}

static void PopulateMapIns3(data_set_t data_set[DID_COUNT], uint32_t did)
{
    DataMapper<ins_3_t> mapper(data_set, did);
    uint32_t flags = DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3;
    mapper.AddMember("week", &ins_3_t::week, DATA_TYPE_UINT32, "week", "Weeks since Jan 6, 1980", flags );
    mapper.AddMember("timeOfWeek", &ins_3_t::timeOfWeek, DATA_TYPE_F64, "s", "Time of week since Sunday morning, GMT", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_4 );
    mapper.AddMember("insStatus", &ins_3_t::insStatus, DATA_TYPE_UINT32,  "", s_insStatusDescription, DATA_FLAGS_DISPLAY_HEX | DATA_FLAGS_INS_STATUS);
    mapper.AddMember("hdwStatus", &ins_3_t::hdwStatus, DATA_TYPE_UINT32,  "", s_hdwStatusDescription, DATA_FLAGS_DISPLAY_HEX);
    mapper.AddArray("qn2b", &ins_3_t::qn2b, DATA_TYPE_F32, 4, "", "Quaternion body rotation with respect to NED: W, X, Y, Z", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_4);
    mapper.AddArray("uvw", &ins_3_t::uvw, DATA_TYPE_F32, 3, "m/s", "Velocity in body frame", flags);
    mapper.AddArray("lla", &ins_3_t::lla, DATA_TYPE_F64, 3, SYM_DEG_DEG_M, "WGS84 latitude, longitude, ellipsoid altitude", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_8);
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
    mapper.AddArray("qe2b", &ins_4_t::qe2b, DATA_TYPE_F32, 4, "", "Quaternion body rotation with respect to ECEF: W, X, Y, Z", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_4);
    mapper.AddArray("ve", &ins_4_t::ve, DATA_TYPE_F32, 3, "m/s", "Velocity in ECEF (earth-centered earth-fixed) frame", flags);
    mapper.AddArray("ecef", &ins_4_t::ecef, DATA_TYPE_F64, 3, "m", "Position in ECEF (earth-centered earth-fixed) frame", flags);
}

static void PopulateMapGpsPos(data_set_t data_set[DID_COUNT], uint32_t did)
{
    DataMapper<gps_pos_t> mapper(data_set, did);
    mapper.AddMember("week", &gps_pos_t::week, DATA_TYPE_UINT32, "week", "Weeks since Jan 6, 1980", DATA_FLAGS_READ_ONLY);
    mapper.AddMember("timeOfWeekMs", &gps_pos_t::timeOfWeekMs, DATA_TYPE_UINT32, "ms", "Time of week since Sunday morning", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_4);
    mapper.AddMember("status", &gps_pos_t::status, DATA_TYPE_UINT32, "", "GPS status: [0x000000xx] number of satellites used, [0x0000xx00] fix type, [0x00xx0000] status flags", DATA_FLAGS_READ_ONLY | DATA_FLAGS_DISPLAY_HEX);
    mapper.AddArray("ecef", &gps_pos_t::ecef, DATA_TYPE_F64, 3, "m", "Position in ECEF {x,y,z}", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3);
    mapper.AddArray("lla", &gps_pos_t::lla, DATA_TYPE_F64, 3, SYM_DEG_DEG_M, "Latitude, longitude, ellipsoid altitude", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_7);
    mapper.AddMember("hMSL", &gps_pos_t::hMSL, DATA_TYPE_F32, "m", "Meters above sea level", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_4);
    mapper.AddMember("hAcc", &gps_pos_t::hAcc, DATA_TYPE_F32, "m", "Position horizontal accuracy", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_4);
    mapper.AddMember("vAcc", &gps_pos_t::vAcc, DATA_TYPE_F32, "m", "Position vertical accuracy", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3);
    mapper.AddMember("pDop", &gps_pos_t::pDop, DATA_TYPE_F32, "m", "Position dilution of precision", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3);
    mapper.AddMember("cnoMean", &gps_pos_t::cnoMean, DATA_TYPE_F32, "dBHz", "Average of non-zero satellite carrier to noise ratios (signal strengths)", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_1);
    mapper.AddMember("towOffset", &gps_pos_t::towOffset, DATA_TYPE_F64, "sec", "Time sync offset from local clock", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_5);
    mapper.AddMember("leapS", &gps_pos_t::leapS, DATA_TYPE_UINT8, "", "GPS leap seconds (GPS-UTC). Receiver's best knowledge of the leap seconds offset from UTC to GPS time.", DATA_FLAGS_READ_ONLY);
    mapper.AddMember("satsUsed", &gps_pos_t::satsUsed, DATA_TYPE_UINT8, "", "Number of satellites used in the solution", DATA_FLAGS_READ_ONLY);
    mapper.AddMember("cnoMeanSigma", &gps_pos_t::cnoMeanSigma, DATA_TYPE_UINT8, "10dBHz", "10x standard deviation of CNO mean over past 5 seconds", DATA_FLAGS_READ_ONLY);
    mapper.AddMember("reserved", &gps_pos_t::reserved, DATA_TYPE_UINT8, "", "", DATA_FLAGS_READ_ONLY);
}

static void PopulateMapGpsVel(data_set_t data_set[DID_COUNT], uint32_t did)
{
    DataMapper<gps_vel_t> mapper(data_set, did);
    mapper.AddMember("timeOfWeekMs", &gps_vel_t::timeOfWeekMs, DATA_TYPE_UINT32, "ms", "Time of week since Sunday morning", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_4);
    mapper.AddArray("vel", &gps_vel_t::vel, DATA_TYPE_F32, 3, "m/s", "Velocity in ECEF {vx,vy,vz} or NED {vN, vE, 0} if status GPS_STATUS_FLAGS_GPS_NMEA_DATA = 0 or 1", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_2);
    mapper.AddMember("sAcc", &gps_vel_t::sAcc, DATA_TYPE_F32, "m/s", "Speed accuracy", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_2);
    mapper.AddMember("status", &gps_vel_t::status, DATA_TYPE_UINT32, "", "GPS status: NMEA input if status flag GPS_STATUS_FLAGS_GPS_NMEA_DATA", DATA_FLAGS_READ_ONLY | DATA_FLAGS_DISPLAY_HEX);
}

static void PopulateMapGpsSat(data_set_t data_set[DID_COUNT], uint32_t did)
{
    INIT_MAP(gps_sat_t, did);
    ADD_MAP_7("timeOfWeekMs", timeOfWeekMs, DATA_TYPE_UINT32, uint32_t, "ms", "Time of week since Sunday morning, GMT", DATA_FLAGS_READ_ONLY);
    ADD_MAP_7("numSats", numSats, DATA_TYPE_UINT32, uint32_t, "", "Number of satellites in sky", DATA_FLAGS_READ_ONLY);

#define ADD_MAP_SAT_INFO(n) \
    ADD_MAP_4("sat" #n ".gnssId",    sat[n].gnssId,    DATA_TYPE_UINT8, uint8_t); \
    ADD_MAP_4("sat" #n ".svId",      sat[n].svId,      DATA_TYPE_UINT8, uint8_t); \
    ADD_MAP_4("sat" #n ".elev",      sat[n].elev,      DATA_TYPE_INT8,  int8_t); \
    ADD_MAP_4("sat" #n ".azim",      sat[n].azim,      DATA_TYPE_INT16, int16_t); \
    ADD_MAP_4("sat" #n ".cno",       sat[n].cno,       DATA_TYPE_UINT8, uint8_t); \
    ADD_MAP_4("sat" #n ".status",    sat[n].status,    DATA_TYPE_UINT16, uint16_t);

    ADD_MAP_SAT_INFO(0);
    ADD_MAP_SAT_INFO(1);
    ADD_MAP_SAT_INFO(2);
    ADD_MAP_SAT_INFO(3);
    ADD_MAP_SAT_INFO(4);
    ADD_MAP_SAT_INFO(5);
    ADD_MAP_SAT_INFO(6);
    ADD_MAP_SAT_INFO(7);
    ADD_MAP_SAT_INFO(8);
    ADD_MAP_SAT_INFO(9);

    ADD_MAP_SAT_INFO(10);
    ADD_MAP_SAT_INFO(11);
    ADD_MAP_SAT_INFO(12);
    ADD_MAP_SAT_INFO(13);
    ADD_MAP_SAT_INFO(14);
    ADD_MAP_SAT_INFO(15);
    ADD_MAP_SAT_INFO(16);
    ADD_MAP_SAT_INFO(17);
    ADD_MAP_SAT_INFO(18);
    ADD_MAP_SAT_INFO(19);

    ADD_MAP_SAT_INFO(20);
    ADD_MAP_SAT_INFO(21);
    ADD_MAP_SAT_INFO(22);
    ADD_MAP_SAT_INFO(23);
    ADD_MAP_SAT_INFO(24);
    ADD_MAP_SAT_INFO(25);
    ADD_MAP_SAT_INFO(26);
    ADD_MAP_SAT_INFO(27);
    ADD_MAP_SAT_INFO(28);
    ADD_MAP_SAT_INFO(29);

    ADD_MAP_SAT_INFO(30);
    ADD_MAP_SAT_INFO(31);
    ADD_MAP_SAT_INFO(32);
    ADD_MAP_SAT_INFO(33);
    ADD_MAP_SAT_INFO(34);
    ADD_MAP_SAT_INFO(35);
    ADD_MAP_SAT_INFO(36);
    ADD_MAP_SAT_INFO(37);
    ADD_MAP_SAT_INFO(38);
    ADD_MAP_SAT_INFO(39);

    ADD_MAP_SAT_INFO(40);
    ADD_MAP_SAT_INFO(41);
    ADD_MAP_SAT_INFO(42);
    ADD_MAP_SAT_INFO(43);
    ADD_MAP_SAT_INFO(44);
    ADD_MAP_SAT_INFO(45);
    ADD_MAP_SAT_INFO(46);
    ADD_MAP_SAT_INFO(47);
    ADD_MAP_SAT_INFO(48);
    ADD_MAP_SAT_INFO(49);
    ASSERT_SIZE(totalSize);
}

static void PopulateMapGpsSig(data_set_t data_set[DID_COUNT], uint32_t did)
{
    INIT_MAP(gps_sig_t, did);
    ADD_MAP_7("timeOfWeekMs", timeOfWeekMs, DATA_TYPE_UINT32, uint32_t, "ms", "Time of week since Sunday morning, GMT", DATA_FLAGS_READ_ONLY);
    ADD_MAP_7("numSigs", numSigs, DATA_TYPE_UINT32, uint32_t, "", "Number of signals in sky", DATA_FLAGS_READ_ONLY);

#define ADD_MAP_SAT_SIG(n) \
    ADD_MAP_4("sig" #n ".gnssId",    sig[n].gnssId,    DATA_TYPE_UINT8, uint8_t); \
    ADD_MAP_4("sig" #n ".svId",      sig[n].svId,      DATA_TYPE_UINT8, uint8_t); \
    ADD_MAP_4("sig" #n ".sigId",     sig[n].sigId,     DATA_TYPE_UINT8, uint8_t); \
    ADD_MAP_4("sig" #n ".cno",       sig[n].cno,       DATA_TYPE_UINT8, uint8_t); \
    ADD_MAP_4("sig" #n ".quality",   sig[n].quality,   DATA_TYPE_UINT8, uint8_t); \
    ADD_MAP_4("sig" #n ".status",    sig[n].status,    DATA_TYPE_UINT16, uint16_t);

    ADD_MAP_SAT_SIG(0);
    ADD_MAP_SAT_SIG(1);
    ADD_MAP_SAT_SIG(2);
    ADD_MAP_SAT_SIG(3);
    ADD_MAP_SAT_SIG(4);
    ADD_MAP_SAT_SIG(5);
    ADD_MAP_SAT_SIG(6);
    ADD_MAP_SAT_SIG(7);
    ADD_MAP_SAT_SIG(8);
    ADD_MAP_SAT_SIG(9);

    ADD_MAP_SAT_SIG(10);
    ADD_MAP_SAT_SIG(11);
    ADD_MAP_SAT_SIG(12);
    ADD_MAP_SAT_SIG(13);
    ADD_MAP_SAT_SIG(14);
    ADD_MAP_SAT_SIG(15);
    ADD_MAP_SAT_SIG(16);
    ADD_MAP_SAT_SIG(17);
    ADD_MAP_SAT_SIG(18);
    ADD_MAP_SAT_SIG(19);

    ADD_MAP_SAT_SIG(20);
    ADD_MAP_SAT_SIG(21);
    ADD_MAP_SAT_SIG(22);
    ADD_MAP_SAT_SIG(23);
    ADD_MAP_SAT_SIG(24);
    ADD_MAP_SAT_SIG(25);
    ADD_MAP_SAT_SIG(26);
    ADD_MAP_SAT_SIG(27);
    ADD_MAP_SAT_SIG(28);
    ADD_MAP_SAT_SIG(29);

    ADD_MAP_SAT_SIG(30);
    ADD_MAP_SAT_SIG(31);
    ADD_MAP_SAT_SIG(32);
    ADD_MAP_SAT_SIG(33);
    ADD_MAP_SAT_SIG(34);
    ADD_MAP_SAT_SIG(35);
    ADD_MAP_SAT_SIG(36);
    ADD_MAP_SAT_SIG(37);
    ADD_MAP_SAT_SIG(38);
    ADD_MAP_SAT_SIG(39);

    ADD_MAP_SAT_SIG(40);
    ADD_MAP_SAT_SIG(41);
    ADD_MAP_SAT_SIG(42);
    ADD_MAP_SAT_SIG(43);
    ADD_MAP_SAT_SIG(44);
    ADD_MAP_SAT_SIG(45);
    ADD_MAP_SAT_SIG(46);
    ADD_MAP_SAT_SIG(47);
    ADD_MAP_SAT_SIG(48);
    ADD_MAP_SAT_SIG(49);
    ASSERT_SIZE(totalSize);
}

static void PopulateMapGpsVersion(data_set_t data_set[DID_COUNT], uint32_t did)
{
    INIT_MAP(gps_version_t, did);
    ADD_MAP_6("swVersion", swVersion, DATA_TYPE_STRING, uint8_t[30], "", "Software version");
    ADD_MAP_6("hwVersion", hwVersion, DATA_TYPE_STRING, uint8_t[10], "", "Hardware version");
//    ADD_MAP_6("extension[0]", extension[0], DATA_TYPE_STRING, uint8_t[30], "", "Extension 30 bytes array description.");
//    ADD_MAP_6("extension[1]", extension[1], DATA_TYPE_STRING, uint8_t[30], "", "Extension 30 bytes array description.");
//    ADD_MAP_6("extension[2]", extension[2], DATA_TYPE_STRING, uint8_t[30], "", "Extension 30 bytes array description.");
//    ADD_MAP_6("extension[3]", extension[3], DATA_TYPE_STRING, uint8_t[30], "", "Extension 30 bytes array description.");
//    ADD_MAP_6("extension[4]", extension[4], DATA_TYPE_STRING, uint8_t[30], "", "Extension 30 bytes array description.");
//    ADD_MAP_6("extension[5]", extension[5], DATA_TYPE_STRING, uint8_t[30], "", "Extension 30 bytes array description.");
//    ASSERT_SIZE(totalSize);
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
    mapper.AddArray("mag", &magnetometer_t::mag, DATA_TYPE_F32, 3, "", "Normalized gauss", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3);
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
    mapper.AddArray("theta", &pimu_t::theta, DATA_TYPE_F32, 3, SYM_DEG, "IMU delta theta coning and sculling integrals in body/IMU frame.  " + description, DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_4, C_RAD2DEG);
    mapper.AddArray("vel", &pimu_t::vel, DATA_TYPE_F32, 3, "m/s", "IMU delta velocity coning and sculling integrals in body/IMU frame.  " + description, DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_5);
}

static void PopulateMapPimuMag(data_set_t data_set[DID_COUNT], uint32_t did)
{
    INIT_MAP(pimu_mag_t, did);
    ADD_MAP_7("imutime", pimu.time, DATA_TYPE_F64, double, "s", "Local time since startup.", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_8("theta[0]", pimu.theta[0], DATA_TYPE_F32, float&, SYM_DEG, "IMU delta theta coning and sculling integrals in body/IMU frame.", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_4, C_RAD2DEG);
    ADD_MAP_8("theta[1]", pimu.theta[1], DATA_TYPE_F32, float&, SYM_DEG, "IMU delta theta coning and sculling integrals in body/IMU frame.", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_4, C_RAD2DEG);
    ADD_MAP_8("theta[2]", pimu.theta[2], DATA_TYPE_F32, float&, SYM_DEG, "IMU delta theta coning and sculling integrals in body/IMU frame.", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_4, C_RAD2DEG);
    ADD_MAP_7("vel[0]", pimu.vel[0], DATA_TYPE_F32, float&, "m/s", "IMU delta velocity coning and sculling integrals in body/IMU frame.", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_5);
    ADD_MAP_7("vel[1]", pimu.vel[1], DATA_TYPE_F32, float&, "m/s", "IMU delta velocity coning and sculling integrals in body/IMU frame.", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_5);
    ADD_MAP_7("vel[2]", pimu.vel[2], DATA_TYPE_F32, float&, "m/s", "IMU delta velocity coning and sculling integrals in body/IMU frame.", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_5);
    ADD_MAP_7("dt", pimu.dt, DATA_TYPE_F32, float, "s", "Integration period.", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_7("imustatus", pimu.status, DATA_TYPE_UINT32, uint32_t, "", s_imuStatusDescription, DATA_FLAGS_DISPLAY_HEX);
    ADD_MAP_7("magtime", mag.time, DATA_TYPE_F64, double, "s", "Local time since startup.", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_7("mag[0]", mag.mag[0], DATA_TYPE_F32, float&, "", "Normalized gauss", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3);
    ADD_MAP_7("mag[1]", mag.mag[1], DATA_TYPE_F32, float&, "", "Normalized gauss", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3);
    ADD_MAP_7("mag[2]", mag.mag[2], DATA_TYPE_F32, float&, "", "Normalized gauss", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3);
    ASSERT_SIZE(totalSize);
}

static void PopulateMapImuMag(data_set_t data_set[DID_COUNT], uint32_t did)
{
    INIT_MAP(imu_mag_t, did);
    ADD_MAP_7("imutime", imu.time, DATA_TYPE_F64, double, "s", "Time since boot up", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_8("pqr[0]", imu.I.pqr[0], DATA_TYPE_F32, float&, SYM_DEG_PER_S, "IMU Angular rate", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_2, C_RAD2DEG);
    ADD_MAP_8("pqr[1]", imu.I.pqr[1], DATA_TYPE_F32, float&, SYM_DEG_PER_S, "IMU Angular rate", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_2, C_RAD2DEG);
    ADD_MAP_8("pqr[2]", imu.I.pqr[2], DATA_TYPE_F32, float&, SYM_DEG_PER_S, "IMU Angular rate", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_2, C_RAD2DEG);
    ADD_MAP_7("acc[0]", imu.I.acc[0], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "IMU Linear acceleration", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3);
    ADD_MAP_7("acc[1]", imu.I.acc[1], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "IMU Linear acceleration", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3);
    ADD_MAP_7("acc[2]", imu.I.acc[2], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "IMU Linear acceleration", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3);
    ADD_MAP_7("imustatus", imu.status, DATA_TYPE_UINT32, uint32_t, "", s_imuStatusDescription, DATA_FLAGS_DISPLAY_HEX);
    ADD_MAP_7("magtime", mag.time, DATA_TYPE_F64, double, "s", "Local time since startup.", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_7("mag[0]", mag.mag[0], DATA_TYPE_F32, float&, "", "Normalized gauss", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3);
    ADD_MAP_7("mag[1]", mag.mag[1], DATA_TYPE_F32, float&, "", "Normalized gauss", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3);
    ADD_MAP_7("mag[2]", mag.mag[2], DATA_TYPE_F32, float&, "", "Normalized gauss", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3);
    ASSERT_SIZE(totalSize);
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
    INIT_MAP(infield_cal_t, did);
    ADD_MAP_6("state", state, DATA_TYPE_UINT32, uint32_t, "", "0=off, init[1=IMU, 2=gyro, 3=accel], init align INS[4, 5=+IMU, 6=+gyro, 7=+accel], 8=sample, 9=finish (see eInfieldCalState)");
    ADD_MAP_7("status", status, DATA_TYPE_UINT32, uint32_t, "", "Infield cal status (see eInfieldCalStatus)", DATA_FLAGS_DISPLAY_HEX);
    ADD_MAP_6("sampleTimeMs", sampleTimeMs, DATA_TYPE_UINT32, uint32_t, "ms", "Duration of IMU sample averaging. sampleTimeMs = 0 means \"imu\" member contains the IMU bias from flash.");

    ADD_MAP_8("imu[0].pqr[0]", imu[0].pqr[0], DATA_TYPE_F32, float&, SYM_DEG_PER_S, "Sampled angular rate.  IMU bias when state=INFIELD_CAL_STATE_SAVED_AND_FINISHED", DATA_FLAGS_FIXED_DECIMAL_3, C_RAD2DEG);
    ADD_MAP_8("imu[0].pqr[1]", imu[0].pqr[1], DATA_TYPE_F32, float&, SYM_DEG_PER_S, "Sampled angular rate.  IMU bias when state=INFIELD_CAL_STATE_SAVED_AND_FINISHED", DATA_FLAGS_FIXED_DECIMAL_3, C_RAD2DEG);
    ADD_MAP_8("imu[0].pqr[2]", imu[0].pqr[2], DATA_TYPE_F32, float&, SYM_DEG_PER_S, "Sampled angular rate.  IMU bias when state=INFIELD_CAL_STATE_SAVED_AND_FINISHED", DATA_FLAGS_FIXED_DECIMAL_3, C_RAD2DEG);
    ADD_MAP_7("imu[0].acc[0]", imu[0].acc[0], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "Sampled linear acceleration.  IMU bias when state=INFIELD_CAL_STATE_SAVED_AND_FINISHED", DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_7("imu[0].acc[1]", imu[0].acc[1], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "Sampled linear acceleration.  IMU bias when state=INFIELD_CAL_STATE_SAVED_AND_FINISHED", DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_7("imu[0].acc[2]", imu[0].acc[2], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "Sampled linear acceleration.  IMU bias when state=INFIELD_CAL_STATE_SAVED_AND_FINISHED", DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_8("imu[1].pqr[0]", imu[1].pqr[0], DATA_TYPE_F32, float&, SYM_DEG_PER_S, "Sampled angular rate.  IMU bias when state=INFIELD_CAL_STATE_SAVED_AND_FINISHED", DATA_FLAGS_FIXED_DECIMAL_3, C_RAD2DEG);
    ADD_MAP_8("imu[1].pqr[1]", imu[1].pqr[1], DATA_TYPE_F32, float&, SYM_DEG_PER_S, "Sampled angular rate.  IMU bias when state=INFIELD_CAL_STATE_SAVED_AND_FINISHED", DATA_FLAGS_FIXED_DECIMAL_3, C_RAD2DEG);
    ADD_MAP_8("imu[1].pqr[2]", imu[1].pqr[2], DATA_TYPE_F32, float&, SYM_DEG_PER_S, "Sampled angular rate.  IMU bias when state=INFIELD_CAL_STATE_SAVED_AND_FINISHED", DATA_FLAGS_FIXED_DECIMAL_3, C_RAD2DEG);
    ADD_MAP_7("imu[1].acc[0]", imu[1].acc[0], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "Sampled linear acceleration.  IMU bias when state=INFIELD_CAL_STATE_SAVED_AND_FINISHED", DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_7("imu[1].acc[1]", imu[1].acc[1], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "Sampled linear acceleration.  IMU bias when state=INFIELD_CAL_STATE_SAVED_AND_FINISHED", DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_7("imu[1].acc[2]", imu[1].acc[2], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "Sampled linear acceleration.  IMU bias when state=INFIELD_CAL_STATE_SAVED_AND_FINISHED", DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_8("imu[2].pqr[0]", imu[2].pqr[0], DATA_TYPE_F32, float&, SYM_DEG_PER_S, "Sampled angular rate.  IMU bias when state=INFIELD_CAL_STATE_SAVED_AND_FINISHED", DATA_FLAGS_FIXED_DECIMAL_3, C_RAD2DEG);
    ADD_MAP_8("imu[2].pqr[1]", imu[2].pqr[1], DATA_TYPE_F32, float&, SYM_DEG_PER_S, "Sampled angular rate.  IMU bias when state=INFIELD_CAL_STATE_SAVED_AND_FINISHED", DATA_FLAGS_FIXED_DECIMAL_3, C_RAD2DEG);
    ADD_MAP_8("imu[2].pqr[2]", imu[2].pqr[2], DATA_TYPE_F32, float&, SYM_DEG_PER_S, "Sampled angular rate.  IMU bias when state=INFIELD_CAL_STATE_SAVED_AND_FINISHED", DATA_FLAGS_FIXED_DECIMAL_3, C_RAD2DEG);
    ADD_MAP_7("imu[2].acc[0]", imu[2].acc[0], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "Sampled linear acceleration.  IMU bias when state=INFIELD_CAL_STATE_SAVED_AND_FINISHED", DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_7("imu[2].acc[1]", imu[2].acc[1], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "Sampled linear acceleration.  IMU bias when state=INFIELD_CAL_STATE_SAVED_AND_FINISHED", DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_7("imu[2].acc[2]", imu[2].acc[2], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "Sampled linear acceleration.  IMU bias when state=INFIELD_CAL_STATE_SAVED_AND_FINISHED", DATA_FLAGS_FIXED_DECIMAL_4);

    ADD_MAP_7("calData[0].down.dev[0].acc[0]", calData[0].down.dev[0].acc[0], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "Linear acceleration", DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_7("calData[0].down.dev[0].acc[1]", calData[0].down.dev[0].acc[1], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "Linear acceleration", DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_7("calData[0].down.dev[0].acc[2]", calData[0].down.dev[0].acc[2], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "Linear acceleration", DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_7("calData[0].down.dev[1].acc[0]", calData[0].down.dev[1].acc[0], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "Linear acceleration", DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_7("calData[0].down.dev[1].acc[1]", calData[0].down.dev[1].acc[1], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "Linear acceleration", DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_7("calData[0].down.dev[1].acc[2]", calData[0].down.dev[1].acc[2], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "Linear acceleration", DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_7("calData[0].down.dev[2].acc[0]", calData[0].down.dev[2].acc[0], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "Linear acceleration", DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_7("calData[0].down.dev[2].acc[1]", calData[0].down.dev[2].acc[1], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "Linear acceleration", DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_7("calData[0].down.dev[2].acc[2]", calData[0].down.dev[2].acc[2], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "Linear acceleration", DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_8("calData[0].down.yaw", calData[0].down.yaw, DATA_TYPE_F32, float, SYM_DEG, "Yaw angle. >=999 means two samples have been averaged.", DATA_FLAGS_FIXED_DECIMAL_1, C_RAD2DEG);
    ADD_MAP_7("calData[0].up.dev[0].acc[0]", calData[0].up.dev[0].acc[0], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "Linear acceleration", DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_7("calData[0].up.dev[0].acc[1]", calData[0].up.dev[0].acc[1], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "Linear acceleration", DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_7("calData[0].up.dev[0].acc[2]", calData[0].up.dev[0].acc[2], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "Linear acceleration", DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_7("calData[0].up.dev[1].acc[0]", calData[0].up.dev[1].acc[0], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "Linear acceleration", DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_7("calData[0].up.dev[1].acc[1]", calData[0].up.dev[1].acc[1], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "Linear acceleration", DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_7("calData[0].up.dev[1].acc[2]", calData[0].up.dev[1].acc[2], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "Linear acceleration", DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_7("calData[0].up.dev[2].acc[0]", calData[0].up.dev[2].acc[0], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "Linear acceleration", DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_7("calData[0].up.dev[2].acc[1]", calData[0].up.dev[2].acc[1], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "Linear acceleration", DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_7("calData[0].up.dev[2].acc[2]", calData[0].up.dev[2].acc[2], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "Linear acceleration", DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_8("calData[0].up.yaw", calData[0].up.yaw, DATA_TYPE_F32, float, SYM_DEG, "Yaw angle. >=999 means two samples have been averaged.", DATA_FLAGS_FIXED_DECIMAL_1, C_RAD2DEG);

    ADD_MAP_7("calData[1].down.dev[0].acc[0]", calData[1].down.dev[0].acc[0], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "Linear acceleration", DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_7("calData[1].down.dev[0].acc[1]", calData[1].down.dev[0].acc[1], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "Linear acceleration", DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_7("calData[1].down.dev[0].acc[2]", calData[1].down.dev[0].acc[2], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "Linear acceleration", DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_7("calData[1].down.dev[1].acc[0]", calData[1].down.dev[1].acc[0], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "Linear acceleration", DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_7("calData[1].down.dev[1].acc[1]", calData[1].down.dev[1].acc[1], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "Linear acceleration", DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_7("calData[1].down.dev[1].acc[2]", calData[1].down.dev[1].acc[2], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "Linear acceleration", DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_7("calData[1].down.dev[2].acc[0]", calData[1].down.dev[2].acc[0], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "Linear acceleration", DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_7("calData[1].down.dev[2].acc[1]", calData[1].down.dev[2].acc[1], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "Linear acceleration", DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_7("calData[1].down.dev[2].acc[2]", calData[1].down.dev[2].acc[2], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "Linear acceleration", DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_8("calData[1].down.yaw", calData[1].down.yaw, DATA_TYPE_F32, float, SYM_DEG, "Yaw angle. >=999 means two samples have been averaged.", DATA_FLAGS_FIXED_DECIMAL_1, C_RAD2DEG);
    ADD_MAP_7("calData[1].up.dev[0].acc[0]", calData[1].up.dev[0].acc[0], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "Linear acceleration", DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_7("calData[1].up.dev[0].acc[1]", calData[1].up.dev[0].acc[1], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "Linear acceleration", DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_7("calData[1].up.dev[0].acc[2]", calData[1].up.dev[0].acc[2], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "Linear acceleration", DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_7("calData[1].up.dev[1].acc[0]", calData[1].up.dev[1].acc[0], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "Linear acceleration", DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_7("calData[1].up.dev[1].acc[1]", calData[1].up.dev[1].acc[1], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "Linear acceleration", DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_7("calData[1].up.dev[1].acc[2]", calData[1].up.dev[1].acc[2], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "Linear acceleration", DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_7("calData[1].up.dev[2].acc[0]", calData[1].up.dev[2].acc[0], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "Linear acceleration", DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_7("calData[1].up.dev[2].acc[1]", calData[1].up.dev[2].acc[1], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "Linear acceleration", DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_7("calData[1].up.dev[2].acc[2]", calData[1].up.dev[2].acc[2], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "Linear acceleration", DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_8("calData[1].up.yaw", calData[1].up.yaw, DATA_TYPE_F32, float, SYM_DEG, "Yaw angle. >=999 means two samples have been averaged.", DATA_FLAGS_FIXED_DECIMAL_1, C_RAD2DEG);

    ADD_MAP_7("calData[2].down.dev[0].acc[0]", calData[2].down.dev[0].acc[0], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "Linear acceleration", DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_7("calData[2].down.dev[0].acc[1]", calData[2].down.dev[0].acc[1], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "Linear acceleration", DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_7("calData[2].down.dev[0].acc[2]", calData[2].down.dev[0].acc[2], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "Linear acceleration", DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_7("calData[2].down.dev[1].acc[0]", calData[2].down.dev[1].acc[0], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "Linear acceleration", DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_7("calData[2].down.dev[1].acc[1]", calData[2].down.dev[1].acc[1], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "Linear acceleration", DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_7("calData[2].down.dev[1].acc[2]", calData[2].down.dev[1].acc[2], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "Linear acceleration", DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_7("calData[2].down.dev[2].acc[0]", calData[2].down.dev[2].acc[0], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "Linear acceleration", DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_7("calData[2].down.dev[2].acc[1]", calData[2].down.dev[2].acc[1], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "Linear acceleration", DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_7("calData[2].down.dev[2].acc[2]", calData[2].down.dev[2].acc[2], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "Linear acceleration", DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_8("calData[2].down.yaw", calData[2].down.yaw, DATA_TYPE_F32, float, SYM_DEG, "Yaw angle. >=999 means two samples have been averaged.", DATA_FLAGS_FIXED_DECIMAL_1, C_RAD2DEG);
    ADD_MAP_7("calData[2].up.dev[0].acc[0]", calData[2].up.dev[0].acc[0], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "Linear acceleration", DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_7("calData[2].up.dev[0].acc[1]", calData[2].up.dev[0].acc[1], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "Linear acceleration", DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_7("calData[2].up.dev[0].acc[2]", calData[2].up.dev[0].acc[2], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "Linear acceleration", DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_7("calData[2].up.dev[1].acc[0]", calData[2].up.dev[1].acc[0], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "Linear acceleration", DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_7("calData[2].up.dev[1].acc[1]", calData[2].up.dev[1].acc[1], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "Linear acceleration", DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_7("calData[2].up.dev[1].acc[2]", calData[2].up.dev[1].acc[2], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "Linear acceleration", DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_7("calData[2].up.dev[2].acc[0]", calData[2].up.dev[2].acc[0], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "Linear acceleration", DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_7("calData[2].up.dev[2].acc[1]", calData[2].up.dev[2].acc[1], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "Linear acceleration", DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_7("calData[2].up.dev[2].acc[2]", calData[2].up.dev[2].acc[2], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "Linear acceleration", DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_8("calData[2].up.yaw", calData[2].up.yaw, DATA_TYPE_F32, float, SYM_DEG, "Yaw angle. >=999 means two samples have been averaged.", DATA_FLAGS_FIXED_DECIMAL_1, C_RAD2DEG);
    ASSERT_SIZE(totalSize);
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
}

static void PopulateMapGroundVehicle(data_set_t data_set[DID_COUNT], uint32_t did)
{
    INIT_MAP(ground_vehicle_t, did);
    ADD_MAP_7("timeOfWeekMs", timeOfWeekMs, DATA_TYPE_UINT32, uint32_t, "ms", "Time of week since Sunday morning, GMT", DATA_FLAGS_READ_ONLY);
    ADD_MAP_7("status", status, DATA_TYPE_UINT32, uint32_t, "", "", DATA_FLAGS_DISPLAY_HEX | DATA_FLAGS_READ_ONLY);
    ADD_MAP_6("mode", mode, DATA_TYPE_UINT32, uint32_t, "", "1=learning; Commands[2=start, 3=resume, 4=clear&start, 5=stop&save, 6=cancel]");
    ADD_MAP_5("wheelConfig.bits", wheelConfig.bits, DATA_TYPE_UINT32, uint32_t, DATA_FLAGS_DISPLAY_HEX);
    ADD_MAP_7("wheelConfig.transform.e_b2w[0]", wheelConfig.transform.e_b2w[0], DATA_TYPE_F32, float&, "rad", "Euler angle rotation from imu (body) to wheel frame (center of non-steering axle)", DATA_FLAGS_FIXED_DECIMAL_3);
    ADD_MAP_7("wheelConfig.transform.e_b2w[1]", wheelConfig.transform.e_b2w[1], DATA_TYPE_F32, float&, "rad", "Euler angle rotation from imu (body) to wheel frame (center of non-steering axle)", DATA_FLAGS_FIXED_DECIMAL_3);
    ADD_MAP_7("wheelConfig.transform.e_b2w[2]", wheelConfig.transform.e_b2w[2], DATA_TYPE_F32, float&, "rad", "Euler angle rotation from imu (body) to wheel frame (center of non-steering axle)", DATA_FLAGS_FIXED_DECIMAL_3);
    ADD_MAP_7("wheelConfig.transform.e_b2w_sigma[0]", wheelConfig.transform.e_b2w_sigma[0], DATA_TYPE_F32, float&, "rad", "Standard deviation of Euler angles describing rotation from imu (body) to wheel frame (center of non-steering axle)", DATA_FLAGS_FIXED_DECIMAL_3);
    ADD_MAP_7("wheelConfig.transform.e_b2w_sigma[1]", wheelConfig.transform.e_b2w_sigma[1], DATA_TYPE_F32, float&, "rad", "Standard deviation of Euler angles describing rotation from imu (body) to wheel frame (center of non-steering axle)", DATA_FLAGS_FIXED_DECIMAL_3);
    ADD_MAP_7("wheelConfig.transform.e_b2w_sigma[2]", wheelConfig.transform.e_b2w_sigma[2], DATA_TYPE_F32, float&, "rad", "Standard deviation of Euler angles describing rotation from imu (body) to wheel frame (center of non-steering axle)", DATA_FLAGS_FIXED_DECIMAL_3);
    ADD_MAP_7("wheelConfig.transform.t_b2w[0]", wheelConfig.transform.t_b2w[0], DATA_TYPE_F32, float&, "m", "Translation from imu (body) to wheel frame origin (center of non-steering axle), expressed in imu (body) frame", DATA_FLAGS_FIXED_DECIMAL_3);
    ADD_MAP_7("wheelConfig.transform.t_b2w[1]", wheelConfig.transform.t_b2w[1], DATA_TYPE_F32, float&, "m", "Translation from imu (body) to wheel frame origin (center of non-steering axle), expressed in imu (body) frame", DATA_FLAGS_FIXED_DECIMAL_3);
    ADD_MAP_7("wheelConfig.transform.t_b2w[2]", wheelConfig.transform.t_b2w[2], DATA_TYPE_F32, float&, "m", "Translation from imu (body) to wheel frame origin (center of non-steering axle), expressed in imu (body) frame", DATA_FLAGS_FIXED_DECIMAL_3);
    ADD_MAP_7("wheelConfig.transform.t_b2w_sigma[0]", wheelConfig.transform.t_b2w_sigma[0], DATA_TYPE_F32, float&, "m", "Standard deviation of translation from imu (body) to wheel frame origin (center of non-steering axle), expressed in imu (body) frame", DATA_FLAGS_FIXED_DECIMAL_3);
    ADD_MAP_7("wheelConfig.transform.t_b2w_sigma[1]", wheelConfig.transform.t_b2w_sigma[1], DATA_TYPE_F32, float&, "m", "Standard deviation of translation from imu (body) to wheel frame origin (center of non-steering axle), expressed in imu (body) frame", DATA_FLAGS_FIXED_DECIMAL_3);
    ADD_MAP_7("wheelConfig.transform.t_b2w_sigma[2]", wheelConfig.transform.t_b2w_sigma[2], DATA_TYPE_F32, float&, "m", "Standard deviation of translation from imu (body) to wheel frame origin (center of non-steering axle), expressed in imu (body) frame", DATA_FLAGS_FIXED_DECIMAL_3);
    ADD_MAP_6("wheelConfig.track_width", wheelConfig.track_width, DATA_TYPE_F32, float, "m", "Distance between left and right wheels");
    ADD_MAP_6("wheelConfig.radius", wheelConfig.radius, DATA_TYPE_F32, float, "m", "Wheel radius");
    ASSERT_SIZE(totalSize);
}

static void PopulateMapSystemCommand(data_set_t data_set[DID_COUNT], uint32_t did)
{
    DataMapper<system_command_t> mapper(data_set, did);
    mapper.AddMember("command", &system_command_t::command, DATA_TYPE_UINT32, "", "99=software reset, 5=zero sensors");
    mapper.AddMember("invCommand", &system_command_t::invCommand, DATA_TYPE_UINT32, "", "Bitwise inverse of command (-command - 1) required to process command.");
}

static void PopulateMapNvmFlashCfg(data_set_t data_set[DID_COUNT], uint32_t did)
{
    INIT_MAP(nvm_flash_cfg_t, did);
    string str;
    ADD_MAP_6("startupImuDtMs", startupImuDtMs, DATA_TYPE_UINT32, uint32_t, "ms", "IMU sample (system input data) period set on startup. Cannot be larger than startupInsDtMs. Zero disables sensor/IMU sampling.");
    ADD_MAP_6("startupNavDtMs", startupNavDtMs, DATA_TYPE_UINT32, uint32_t, "ms", "GPS measurement (system input data) update period in milliseconds set on startup. 200ms minimum (5Hz max).");
    ADD_MAP_6("startupGPSDtMs", startupGPSDtMs, DATA_TYPE_UINT32, uint32_t, "ms", "Nav filter (system output data) update period set on startup. 1ms min (1KHz max).");
    ADD_MAP_6("ser0BaudRate", ser0BaudRate, DATA_TYPE_UINT32, uint32_t, "bps", "Serial port 0 baud rate");
    ADD_MAP_6("ser1BaudRate", ser1BaudRate, DATA_TYPE_UINT32, uint32_t, "bps", "Serial port 1 baud rate");
    ADD_MAP_6("ser2BaudRate", ser2BaudRate, DATA_TYPE_UINT32, uint32_t, "bps", "Serial port 2 baud rate");
    str = "rotation from INS Sensor Frame to Intermediate Output Frame.  Order applied: yaw, pitch, roll.";
    ADD_MAP_8("insRotation[0]", insRotation[0], DATA_TYPE_F32, float&, SYM_DEG, "Roll "  + str, 0, C_RAD2DEG);
    ADD_MAP_8("insRotation[1]", insRotation[1], DATA_TYPE_F32, float&, SYM_DEG, "Pitch " + str, 0, C_RAD2DEG);
    ADD_MAP_8("insRotation[2]", insRotation[2], DATA_TYPE_F32, float&, SYM_DEG, "Yaw "   + str, 0, C_RAD2DEG);
    str = "offset from Intermediate Output Frame to INS Output Frame.  INS rotation is applied before this.";
    ADD_MAP_6("insOffset[0]", insOffset[0], DATA_TYPE_F32, float&, "m", "X " + str);
    ADD_MAP_6("insOffset[1]", insOffset[1], DATA_TYPE_F32, float&, "m", "Y " + str);
    ADD_MAP_6("insOffset[2]", insOffset[2], DATA_TYPE_F32, float&, "m", "Z " + str);
    str = "offset from Sensor Frame origin to GPS1 antenna.";
    ADD_MAP_6("gps1AntOffset[0]", gps1AntOffset[0], DATA_TYPE_F32, float&, "m", "X " + str);
    ADD_MAP_6("gps1AntOffset[1]", gps1AntOffset[1], DATA_TYPE_F32, float&, "m", "Y " + str);
    ADD_MAP_6("gps1AntOffset[2]", gps1AntOffset[2], DATA_TYPE_F32, float&, "m", "Z " + str);
    str = "offset from Sensor Frame origin to GPS2 antenna.";
    ADD_MAP_6("gps2AntOffset[0]", gps2AntOffset[0], DATA_TYPE_F32, float&, "m", "X " + str);
    ADD_MAP_6("gps2AntOffset[1]", gps2AntOffset[1], DATA_TYPE_F32, float&, "m", "Y " + str);
    ADD_MAP_6("gps2AntOffset[2]", gps2AntOffset[2], DATA_TYPE_F32, float&, "m", "Z " + str);

    ADD_MAP_8("gpsTimeSyncPeriodMs", gpsTimeSyncPeriodMs, DATA_TYPE_UINT32, uint32_t, "ms", "GPS time synchronization pulse period.", 0, 1.0);
    ADD_MAP_8("gpsTimeUserDelay", gpsTimeUserDelay, DATA_TYPE_F32, float, "s", "User defined delay for GPS time.  This parameter can be used to account for GPS antenna cable delay.", DATA_FLAGS_FIXED_DECIMAL_3, 1.0);
    ADD_MAP_8("gpsMinimumElevation", gpsMinimumElevation, DATA_TYPE_F32, float, SYM_DEG, "GPS minimum elevation of a satellite above the horizon to be used in the solution.", DATA_FLAGS_FIXED_DECIMAL_1, C_RAD2DEG);
    ADD_MAP_8("gnssSatSigConst", gnssSatSigConst, DATA_TYPE_UINT16, uint16_t, "", "GNSS constellations used. 0x0003=GPS, 0x000C=QZSS, 0x0030=Galileo, 0x00C0=Beidou, 0x0300=GLONASS, 0x1000=SBAS (see eGnssSatSigConst)", DATA_FLAGS_DISPLAY_HEX, 1.0);

    ADD_MAP_8("dynamicModel", dynamicModel, DATA_TYPE_UINT8, uint8_t, "", "0:port, 2:stationary, 3:walk, 4:ground vehicle, 5:sea, 6:air<1g, 7:air<2g, 8:air<4g, 9:wrist", 0, 1.0);
    str = "AutobaudOff [0x1=Ser0, 0x2=Ser1], 0x4=AutoMagRecal, 0x8=DisableMagDecEst, ";
    str += "0x10=DisableLeds, ";
    str += "0x100=1AxisMagRecal, ";
    str += "FusionOff [0x1000=Mag, 0x2000=Baro, 0x4000=GPS], ";
    str += "0x10000=enZeroVel, 0x100000=enNavStrobeOutput";
    ADD_MAP_7("sysCfgBits", sysCfgBits, DATA_TYPE_UINT32, uint32_t, "", str, DATA_FLAGS_DISPLAY_HEX);
    str = "Rover [0x1=G1, 0x2=G2], 0x8=GCompass, ";
    str += "BaseOutG1 [0x10=UbxS0, 0x20=UbxS1, 0x40=RtcmS0, 0x80=RtcmS1], ";
    str += "BaseOutG2 [0x100=UbxS0, 0x200=UbxS1, 0x400=RtcmS0, 0x800=RtcmS1], ";
    str += "0x1000=MovingBasePos, 0x4000=SameHdwRvrBase";
    ADD_MAP_7("RTKCfgBits", RTKCfgBits, DATA_TYPE_UINT32, uint32_t, "", str, DATA_FLAGS_DISPLAY_HEX);
    ADD_MAP_7("ioConfig", ioConfig, DATA_TYPE_UINT32, uint32_t, "", "(see enum eIoConfig) IMU disable: 0x1000000,0x20000000,0x4000000", DATA_FLAGS_DISPLAY_HEX);
    ADD_MAP_7("platformConfig", platformConfig, DATA_TYPE_UINT32, uint32_t, "", "Hardware platform (IMX carrier board, i.e. RUG, EVB, IG) configuration bits (see ePlatformConfig)", DATA_FLAGS_DISPLAY_HEX);
    str =  "Gyr FS (deg/s) 0x7:[0=250, 1=500, 2=1000, 3=2000, 4=4000], ";
    str += "Acc FS 0x30:[0=2g, 1=4g, 2=8g, 3=16g], ";
    str += "Gyr DLPF (Hz) 0x0F00:[0=250, 1=184, 2=92, 3=41, 4=20, 5=10, 6=5], ";
    str += "Acc DLPF (Hz) 0xF000:[0=218, 1=218, 2=99, 3=45, 4=21, 5=10, 6=5], ";
    ADD_MAP_7("sensorConfig", sensorConfig, DATA_TYPE_UINT32, uint32_t, "", str, DATA_FLAGS_DISPLAY_HEX);

    ADD_MAP_7("refLla[0]", refLla[0], DATA_TYPE_F64, double&, SYM_DEG_DEG_M, "Reference latitude, longitude, ellipsoid altitude for north east down (NED) calculations", DATA_FLAGS_FIXED_DECIMAL_9);
    ADD_MAP_7("refLla[1]", refLla[1], DATA_TYPE_F64, double&, SYM_DEG_DEG_M, "Reference latitude, longitude, ellipsoid altitude for north east down (NED) calculations", DATA_FLAGS_FIXED_DECIMAL_9);
    ADD_MAP_7("refLla[2]", refLla[2], DATA_TYPE_F64, double&, SYM_DEG_DEG_M, "Reference latitude, longitude, ellipsoid altitude for north east down (NED) calculations", DATA_FLAGS_FIXED_DECIMAL_9);
    ADD_MAP_7("lastLla[0]", lastLla[0], DATA_TYPE_F64, double&, SYM_DEG_DEG_M, "Last latitude, longitude, ellipsoid altitude. (Aids GPS startup)", DATA_FLAGS_FIXED_DECIMAL_9);
    ADD_MAP_7("lastLla[1]", lastLla[1], DATA_TYPE_F64, double&, SYM_DEG_DEG_M, "Last latitude, longitude, ellipsoid altitude. (Aids GPS startup)", DATA_FLAGS_FIXED_DECIMAL_9);
    ADD_MAP_7("lastLla[2]", lastLla[2], DATA_TYPE_F64, double&, SYM_DEG_DEG_M, "Last latitude, longitude, ellipsoid altitude. (Aids GPS startup)", DATA_FLAGS_FIXED_DECIMAL_9);
    ADD_MAP_6("lastLlaTimeOfWeekMs", lastLlaTimeOfWeekMs, DATA_TYPE_UINT32, uint32_t, "ms", "Last LLA update time since Sunday morning");
    ADD_MAP_6("lastLlaWeek", lastLlaWeek, DATA_TYPE_UINT32, uint32_t, "week", "Last LLA update Weeks since Jan 6, 1980");
    ADD_MAP_6("lastLlaUpdateDistance", lastLlaUpdateDistance, DATA_TYPE_F32, float, "m", "Distance between current and last LLA that triggers an update of lastLLA)");
    ADD_MAP_8("magDeclination", magDeclination, DATA_TYPE_F32, float,  SYM_DEG, "Magnetic north declination (heading offset from true north)", DATA_FLAGS_FIXED_DECIMAL_2, C_RAD2DEG);
	ADD_MAP_6("magInterferenceThreshold", magInterferenceThreshold, DATA_TYPE_F32, float, "", "Magnetometer interference sensitivity threshold. Typical range is 2-10 (3 default) and 1000 to disable mag interference detection.");
	ADD_MAP_6("magCalibrationQualityThreshold", magCalibrationQualityThreshold, DATA_TYPE_F32, float, "", "Magnetometer calibration quality sensitivity threshold. Typical range is 10-20 (10 default) and 1000 to disable mag calibration quality check, forcing it to be always good.");
    str = "rotation from INS Sensor Frame to Intermediate Output Frame.  Order applied: heading, pitch, roll.";
    ADD_MAP_8("zeroVelRotation[0]", zeroVelRotation[0], DATA_TYPE_F32, float&, SYM_DEG, "Roll " + str, 0, C_RAD2DEG);
    ADD_MAP_8("zeroVelRotation[1]", zeroVelRotation[1], DATA_TYPE_F32, float&, SYM_DEG, "Pitch " + str, 0, C_RAD2DEG);
    ADD_MAP_8("zeroVelRotation[2]", zeroVelRotation[2], DATA_TYPE_F32, float&, SYM_DEG, "Yaw " + str, 0, C_RAD2DEG);
    str = "offset from Intermediate Output Frame to INS Output Frame.  INS rotation is applied before this.";
    ADD_MAP_6("zeroVelOffset[0]", zeroVelOffset[0], DATA_TYPE_F32, float&, "m", "X " + str);
    ADD_MAP_6("zeroVelOffset[1]", zeroVelOffset[1], DATA_TYPE_F32, float&, "m", "Y " + str);
    ADD_MAP_6("zeroVelOffset[2]", zeroVelOffset[2], DATA_TYPE_F32, float&, "m", "Z " + str);
    ADD_MAP_7("wheelConfig.bits", wheelConfig.bits, DATA_TYPE_UINT32, uint32_t, "", "Wheel encoder config bits: 0x1 enable encoders, 0x2 kinematic constraints", DATA_FLAGS_DISPLAY_HEX);
    ADD_MAP_7("wheelConfig.transform.e_b2w[0]", wheelConfig.transform.e_b2w[0], DATA_TYPE_F32, float&, "rad", "Euler angles describing the rotation from imu (body) to the wheel frame (center of the non-steering axle)", DATA_FLAGS_FIXED_DECIMAL_2);
    ADD_MAP_7("wheelConfig.transform.e_b2w[1]", wheelConfig.transform.e_b2w[1], DATA_TYPE_F32, float&, "rad", "Euler angles describing the rotation from imu (body) to the wheel frame (center of the non-steering axle)", DATA_FLAGS_FIXED_DECIMAL_2);
    ADD_MAP_7("wheelConfig.transform.e_b2w[2]", wheelConfig.transform.e_b2w[2], DATA_TYPE_F32, float&, "rad", "Euler angles describing the rotation from imu (body) to the wheel frame (center of the non-steering axle)", DATA_FLAGS_FIXED_DECIMAL_2);
    ADD_MAP_7("wheelConfig.transform.e_b2w_sigma[0]", wheelConfig.transform.e_b2w_sigma[0], DATA_TYPE_F32, float&, "m", "Standard deviation of Euler angles describing the rotation from imu (body) to the wheel frame (center of the non-steering axle)", DATA_FLAGS_FIXED_DECIMAL_2);
    ADD_MAP_7("wheelConfig.transform.e_b2w_sigma[1]", wheelConfig.transform.e_b2w_sigma[1], DATA_TYPE_F32, float&, "m", "Standard deviation of Euler angles describing the rotation from imu (body) to the wheel frame (center of the non-steering axle)", DATA_FLAGS_FIXED_DECIMAL_2);
    ADD_MAP_7("wheelConfig.transform.e_b2w_sigma[2]", wheelConfig.transform.e_b2w_sigma[2], DATA_TYPE_F32, float&, "m", "Standard deviation of Euler angles describing the rotation from imu (body) to the wheel frame (center of the non-steering axle)", DATA_FLAGS_FIXED_DECIMAL_2);
    ADD_MAP_7("wheelConfig.transform.t_b2w[0]", wheelConfig.transform.t_b2w[0], DATA_TYPE_F32, float&, "m", "Translation from the imu (body) to the wheel frame origin (center of the non-steering axle), expressed in the imu (body) frame", DATA_FLAGS_FIXED_DECIMAL_3);
    ADD_MAP_7("wheelConfig.transform.t_b2w[1]", wheelConfig.transform.t_b2w[1], DATA_TYPE_F32, float&, "m", "Translation from the imu (body) to the wheel frame origin (center of the non-steering axle), expressed in the imu (body) frame", DATA_FLAGS_FIXED_DECIMAL_3);
    ADD_MAP_7("wheelConfig.transform.t_b2w[2]", wheelConfig.transform.t_b2w[2], DATA_TYPE_F32, float&, "m", "Translation from the imu (body) to the wheel frame origin (center of the non-steering axle), expressed in the imu (body) frame", DATA_FLAGS_FIXED_DECIMAL_3);
    ADD_MAP_7("wheelConfig.transform.t_b2w_sigma[0]", wheelConfig.transform.t_b2w_sigma[0], DATA_TYPE_F32, float&, "rad", "Standard deviation of translation from the imu (body) to the wheel frame origin (center of the non-steering axle), expressed in the imu (body) frame", DATA_FLAGS_FIXED_DECIMAL_3);
    ADD_MAP_7("wheelConfig.transform.t_b2w_sigma[1]", wheelConfig.transform.t_b2w_sigma[1], DATA_TYPE_F32, float&, "rad", "Standard deviation of translation from the imu (body) to the wheel frame origin (center of the non-steering axle), expressed in the imu (body) frame", DATA_FLAGS_FIXED_DECIMAL_3);
    ADD_MAP_7("wheelConfig.transform.t_b2w_sigma[2]", wheelConfig.transform.t_b2w_sigma[2], DATA_TYPE_F32, float&, "rad", "Standard deviation of translation from the imu (body) to the wheel frame origin (center of the non-steering axle), expressed in the imu (body) frame", DATA_FLAGS_FIXED_DECIMAL_3);
    ADD_MAP_6("wheelConfig.track_width", wheelConfig.track_width, DATA_TYPE_F32, float, "m", "Distance between the left and right wheels");
    ADD_MAP_6("wheelConfig.radius", wheelConfig.radius, DATA_TYPE_F32, float, "m", "Wheel radius");
    ADD_MAP_4("debug", debug, DATA_TYPE_UINT8, uint8_t);

    // Keep at end
    ADD_MAP_6("size", size, DATA_TYPE_UINT32, uint32_t, "", "Flash group size. Set to 1 to reset this flash group.");
    ADD_MAP_6("checksum", checksum, DATA_TYPE_UINT32, uint32_t, "", "Flash checksum");
    ADD_MAP_6("key", key, DATA_TYPE_UINT32, uint32_t, "", "Flash key");
    ASSERT_SIZE(totalSize);
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
    str = "offset from Sensor Frame origin to GPS1 antenna.";
    mapper.AddArray("gps1AntOffset", &gpx_flash_cfg_t::gps1AntOffset, DATA_TYPE_F32, 3, "m", "X " + str);
    str = "offset from Sensor Frame origin to GPS2 antenna.";
    mapper.AddArray("gps2AntOffset", &gpx_flash_cfg_t::gps2AntOffset, DATA_TYPE_F32, 3, "m", "X " + str);
    mapper.AddMember("gnssSatSigConst", &gpx_flash_cfg_t::gnssSatSigConst, DATA_TYPE_UINT16, "", "GNSS constellations used. 0x0003=GPS, 0x000C=QZSS, 0x0030=Galileo, 0x00C0=Beidou, 0x0300=GLONASS, 0x1000=SBAS (see eGnssSatSigConst)", DATA_FLAGS_DISPLAY_HEX);
    mapper.AddMember("dynamicModel", &gpx_flash_cfg_t::dynamicModel, DATA_TYPE_UINT8, "", "0:port, 2:stationary, 3:walk, 4:ground vehicle, 5:sea, 6:air<1g, 7:air<2g, 8:air<4g, 9:wrist");
    mapper.AddMember("debug", &gpx_flash_cfg_t::debug, DATA_TYPE_UINT8, "", "Reserved");
    mapper.AddMember("gpsTimeSyncPeriodMs", &gpx_flash_cfg_t::gpsTimeSyncPeriodMs, DATA_TYPE_UINT32, "ms", "GPS time synchronization pulse period.");
    mapper.AddMember("gpsTimeUserDelay", &gpx_flash_cfg_t::gpsTimeUserDelay, DATA_TYPE_F32, "s", "User defined delay for GPS time.  This parameter can be used to account for GPS antenna cable delay.", DATA_FLAGS_FIXED_DECIMAL_3);
    mapper.AddMember("gpsMinimumElevation", &gpx_flash_cfg_t::gpsMinimumElevation, DATA_TYPE_F32, SYM_DEG, "GPS minimum elevation of a satellite above the horizon to be used in the solution.", DATA_FLAGS_FIXED_DECIMAL_1, C_RAD2DEG);
    str = "Rover [0x1=G1, 0x2=G2], 0x8=GCompass, ";
    str += "BaseOutG1 [0x10=UbxS0, 0x20=UbxS1, 0x40=RtcmS0, 0x80=RtcmS1], ";
    str += "BaseOutG2 [0x100=UbxS0, 0x200=UbxS1, 0x400=RtcmS0, 0x800=RtcmS1], ";
    str += "0x1000=MovingBasePos, 0x4000=SameHdwRvrBase";
    mapper.AddMember("RTKCfgBits", &gpx_flash_cfg_t::RTKCfgBits, DATA_TYPE_UINT32, "", str, DATA_FLAGS_DISPLAY_HEX);

    // Keep at end
    mapper.AddMember("size", &gpx_flash_cfg_t::size, DATA_TYPE_UINT32, "", "Flash group size. Set to 1 to reset this flash group.");
    mapper.AddMember("checksum", &gpx_flash_cfg_t::checksum, DATA_TYPE_UINT32, "", "Flash checksum");
    mapper.AddMember("key", &gpx_flash_cfg_t::key, DATA_TYPE_UINT32, "", "Flash key");
}

static void PopulateMapGpxStatus(data_set_t data_set[DID_COUNT], uint32_t did)
{
    INIT_MAP(gpx_status_t, did);
    ADD_MAP_7("timeOfWeekMs", timeOfWeekMs, DATA_TYPE_UINT32, uint32_t, "ms", "Time of week since Sunday morning", DATA_FLAGS_READ_ONLY);
    ADD_MAP_7("status", status, DATA_TYPE_UINT32, uint32_t, "", "(see eGpxStatus)", DATA_FLAGS_DISPLAY_HEX);
    ADD_MAP_7("grmcBitsSer0", grmcBitsSer0, DATA_TYPE_UINT64, uint64_t, "", "GPX RMC bit Serial 0", DATA_FLAGS_READ_ONLY | DATA_FLAGS_DISPLAY_HEX);
    ADD_MAP_7("grmcBitsSer1", grmcBitsSer1, DATA_TYPE_UINT64, uint64_t, "", "GPX RMC bit Serial 1", DATA_FLAGS_READ_ONLY | DATA_FLAGS_DISPLAY_HEX);
    ADD_MAP_7("grmcBitsSer2", grmcBitsSer2, DATA_TYPE_UINT64, uint64_t, "", "GPX RMC bit Serial 2", DATA_FLAGS_DISPLAY_HEX);
    ADD_MAP_7("grmcBitsUSB",  grmcBitsUSB,  DATA_TYPE_UINT64, uint64_t, "", "GPX RMC bit USB.", DATA_FLAGS_READ_ONLY | DATA_FLAGS_DISPLAY_HEX);
 
    ADD_MAP_7("grmcNMEABitsSer0", grmcNMEABitsSer0, DATA_TYPE_UINT64, uint64_t, "", "GPX RMC NMEA bit Serial 0", DATA_FLAGS_READ_ONLY | DATA_FLAGS_DISPLAY_HEX);
    ADD_MAP_7("grmcNMEABitsSer1", grmcNMEABitsSer1, DATA_TYPE_UINT64, uint64_t, "", "GPX RMC NMEA bit Serial 1", DATA_FLAGS_READ_ONLY | DATA_FLAGS_DISPLAY_HEX);
    ADD_MAP_7("grmcNMEABitsSer2", grmcNMEABitsSer2, DATA_TYPE_UINT64, uint64_t, "", "GPX RMC NMEA bit Serial 2", DATA_FLAGS_READ_ONLY | DATA_FLAGS_DISPLAY_HEX);
    ADD_MAP_7("grmcNMEABitsUSB",  grmcNMEABitsUSB,  DATA_TYPE_UINT64, uint64_t, "", "GPX RMC NMEA bit USB.", DATA_FLAGS_READ_ONLY | DATA_FLAGS_DISPLAY_HEX);
 
    ADD_MAP_7("hdwStatus", hdwStatus, DATA_TYPE_UINT32, uint32_t, "", "Hardware status eHdwStatusFlags", DATA_FLAGS_DISPLAY_HEX);
    ADD_MAP_7("mcuTemp", mcuTemp, DATA_TYPE_F32, float, SYM_DEG_C, "MCU temperature", DATA_FLAGS_FIXED_DECIMAL_3 | DATA_FLAGS_READ_ONLY);
    ADD_MAP_7("navOutputPeriodMs", navOutputPeriodMs, DATA_TYPE_UINT32, uint32_t, "ms", "Nav output period (ms)", DATA_FLAGS_READ_ONLY);
    ADD_MAP_7("flashCfgChecksum", flashCfgChecksum, DATA_TYPE_UINT32, uint32_t, "", "Flash config validation", DATA_FLAGS_READ_ONLY);
 
    string str = "Rover [0x1=G1, 0x2=G2], 0x8=GCompass, ";
    str += "BaseOutG1 [0x10=UbxS0, 0x20=UbxS1, 0x40=RtcmS0, 0x80=RtcmS1], ";
    str += "BaseOutG2 [0x100=UbxS0, 0x200=UbxS1, 0x400=RtcmS0, 0x800=RtcmS1], ";
    str += "0x1000=MovingBasePos, 0x4000=SameHdwRvrBase";
    ADD_MAP_7("rtkMode", rtkMode, DATA_TYPE_UINT32, uint32_t, "", str, DATA_FLAGS_READ_ONLY | DATA_FLAGS_DISPLAY_HEX);
    ADD_MAP_6("gnssStatus[0].reserved",      gnssStatus[0].reserved,      DATA_TYPE_UINT8, uint8_t, "", "");
    ADD_MAP_6("gnssStatus[0].fwUpdateState", gnssStatus[0].fwUpdateState, DATA_TYPE_UINT8, uint8_t, "", "GNSS FW update status (see FirmwareUpdateState)");
    ADD_MAP_6("gnssStatus[0].initState",     gnssStatus[0].initState,     DATA_TYPE_UINT8, uint8_t, "", "GNSS init status (see InitSteps)");
    ADD_MAP_6("gnssStatus[0].runState",      gnssStatus[0].runState,      DATA_TYPE_UINT8, uint8_t, "", "GNSS run status (see eGPXGnssRunState)");
    ADD_MAP_6("gnssStatus[1].reserved",      gnssStatus[1].reserved,      DATA_TYPE_UINT8, uint8_t, "", "");
    ADD_MAP_6("gnssStatus[1].fwUpdateState", gnssStatus[1].fwUpdateState, DATA_TYPE_UINT8, uint8_t, "", "GNSS FW update status (see FirmwareUpdateState)");
    ADD_MAP_6("gnssStatus[1].initState",     gnssStatus[1].initState,     DATA_TYPE_UINT8, uint8_t, "", "GNSS init status (see InitSteps)");
    ADD_MAP_6("gnssStatus[1].runState",      gnssStatus[1].runState,      DATA_TYPE_UINT8, uint8_t, "", "GNSS run status (see eGPXGnssRunState)");
    ADD_MAP_7("SourcePort", gpxSourcePort, DATA_TYPE_UINT8, uint8_t, "", "Port", DATA_FLAGS_READ_ONLY);
    ADD_MAP_7("upTime", upTime, DATA_TYPE_F64, double, "s", "Local time since startup.", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_1);

    ASSERT_SIZE(totalSize);
}

static void PopulateMapSurveyIn(data_set_t data_set[DID_COUNT], uint32_t did)
{
#if 0
    DataMapper<survey_in_t> mapper(data_set, did);
    {
    storage->registerInt32("state", offsetof(survey_in_t, state), "", QString().asprintf("[status: %d=off, %d-%d=running, %d=done], [cmd: %d=cancel, start: %d=3D,%d=float,%d=fix]",
        SURVEY_IN_STATE_OFF, SURVEY_IN_STATE_RUNNING_3D, SURVEY_IN_STATE_RUNNING_FIX, SURVEY_IN_STATE_DONE, 
        SURVEY_IN_STATE_CANCEL, 
        SURVEY_IN_STATE_START_3D, SURVEY_IN_STATE_START_FLOAT, SURVEY_IN_STATE_START_FIX));
    storage->registerInt32("maxDurationSec", offsetof(survey_in_t, maxDurationSec), "s", "Maximum time survey will run if minAccuracy is not achieved.");
    storage->registerFloat("minAccuracy", offsetof(survey_in_t, minAccuracy), "m", "Required horizontal accuracy for survey to complete before maxDuration.", DATA_FLAGS_FIXED_DECIMAL_4);
    storage->registerInt32("elapsedTimeSec", offsetof(survey_in_t, elapsedTimeSec), "s", "Elapsed time of the survey.");
    storage->registerFloat("currentAccuracy", offsetof(survey_in_t, hAccuracy), "m", "Approximate horizontal accuracy of the survey.", DATA_FLAGS_FIXED_DECIMAL_4);
    storage->registerDoubleArray("lla", offsetof(survey_in_t, lla), 24, g_degdegm, "Surveyed latitude, longitude, ellipsoid altitude", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_9);
    }
#endif
}

static void PopulateMapEvbStatus(data_set_t data_set[DID_COUNT], uint32_t did)
{
    DataMapper<evb_status_t> mapper(data_set, did);
    mapper.AddMember("week", &evb_status_t::week, DATA_TYPE_UINT32, "week", "Weeks since Jan 6, 1980", DATA_FLAGS_READ_ONLY);
    mapper.AddMember("timeOfWeekMs", &evb_status_t::timeOfWeekMs, DATA_TYPE_UINT32, "ms", "Time of week since Sunday morning", DATA_FLAGS_READ_ONLY);
    mapper.AddArray("firmwareVer", &evb_status_t::firmwareVer, DATA_TYPE_UINT8, 4, "", "Firmware version", DATA_FLAGS_READ_ONLY);
    mapper.AddMember("evbStatus", &evb_status_t::evbStatus, DATA_TYPE_UINT32, "", "EVB status bits", DATA_FLAGS_DISPLAY_HEX);
    mapper.AddMember("loggerMode", &evb_status_t::loggerMode, DATA_TYPE_UINT32, "", TOSTRING(EVB2_LOG_CMD_START) "=start, " TOSTRING(EVB2_LOG_CMD_STOP) "=stop");
    mapper.AddMember("loggerElapsedTimeMs", &evb_status_t::loggerElapsedTimeMs, DATA_TYPE_UINT32, "ms", "Elapsed time of the current data log.");
    mapper.AddMember("wifiIpAddr", &evb_status_t::wifiIpAddr, DATA_TYPE_UINT32, "", "WiFi IP address", DATA_FLAGS_DISPLAY_HEX);
    mapper.AddMember("sysCommand", &evb_status_t::sysCommand, DATA_TYPE_UINT32, "", "99=software reset, 1122334455=unlock, 1357924681=chip erase");
    mapper.AddMember("towOffset", &evb_status_t::towOffset, DATA_TYPE_F64, "sec", "Time sync offset from local clock", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_5);
}

static void PopulateMapEvbFlashCfg(data_set_t data_set[DID_COUNT], uint32_t did)
{
    INIT_MAP(evb_flash_cfg_t, did);
    ADD_MAP_6("cbPreset", cbPreset, DATA_TYPE_UINT8, uint8_t, "", 
        TOSTRING(EVB2_CB_PRESET_RS232) "=Wireless Off, " 
        TOSTRING(EVB2_CB_PRESET_RS232_XBEE) "=XBee On, " 
        TOSTRING(EVB2_CB_PRESET_RS422_WIFI) "=WiFi On & RS422, " 
        TOSTRING(EVB2_CB_PRESET_USB_HUB_RS232) "=USB hub, " 
        TOSTRING(EVB2_CB_PRESET_USB_HUB_RS422) "=USB hub w/ RS422");
    ADD_MAP_4("reserved1[0]", reserved1[0], DATA_TYPE_UINT8, uint8_t&);
    ADD_MAP_4("reserved1[1]", reserved1[1], DATA_TYPE_UINT8, uint8_t&);
    ADD_MAP_4("reserved1[2]", reserved1[2], DATA_TYPE_UINT8, uint8_t&);
    ADD_MAP_7("cbf[0]", cbf[0], DATA_TYPE_UINT32, uint32_t&, "", "Communications bridge forwarding", DATA_FLAGS_DISPLAY_HEX);
    ADD_MAP_7("cbf[1]", cbf[1], DATA_TYPE_UINT32, uint32_t&, "", "Communications bridge forwarding", DATA_FLAGS_DISPLAY_HEX);
    ADD_MAP_7("cbf[2]", cbf[2], DATA_TYPE_UINT32, uint32_t&, "", "Communications bridge forwarding", DATA_FLAGS_DISPLAY_HEX);
    ADD_MAP_7("cbf[3]", cbf[3], DATA_TYPE_UINT32, uint32_t&, "", "Communications bridge forwarding", DATA_FLAGS_DISPLAY_HEX);
    ADD_MAP_7("cbf[4]", cbf[4], DATA_TYPE_UINT32, uint32_t&, "", "Communications bridge forwarding", DATA_FLAGS_DISPLAY_HEX);
    ADD_MAP_7("cbf[5]", cbf[5], DATA_TYPE_UINT32, uint32_t&, "", "Communications bridge forwarding", DATA_FLAGS_DISPLAY_HEX);
    ADD_MAP_7("cbf[6]", cbf[6], DATA_TYPE_UINT32, uint32_t&, "", "Communications bridge forwarding", DATA_FLAGS_DISPLAY_HEX);
    ADD_MAP_7("cbf[7]", cbf[7], DATA_TYPE_UINT32, uint32_t&, "", "Communications bridge forwarding", DATA_FLAGS_DISPLAY_HEX);
    ADD_MAP_7("cbf[8]", cbf[8], DATA_TYPE_UINT32, uint32_t&, "", "Communications bridge forwarding", DATA_FLAGS_DISPLAY_HEX);
    ADD_MAP_7("cbf[9]", cbf[9], DATA_TYPE_UINT32, uint32_t&, "", "Communications bridge forwarding", DATA_FLAGS_DISPLAY_HEX);
    ADD_MAP_7("cbOptions", cbOptions, DATA_TYPE_UINT32, uint32_t, "", "Communications bridge options (see eEvb2ComBridgeOptions)", DATA_FLAGS_DISPLAY_HEX);
    ADD_MAP_6("uinsComPort", uinsComPort, DATA_TYPE_UINT8, uint8_t, "", "EVB port for uINS communications and SD card logging. 0=uINS0 (default), 1=uINS1, SP330=5, 6=GPIO_H8 (use eEvb2CommPorts)");
    ADD_MAP_6("uinsAuxPort", uinsAuxPort, DATA_TYPE_UINT8, uint8_t, "", "EVB port for uINS aux com and RTK corrections. 0=uINS0, 1=uINS1 (default), 5=SP330, 6=GPIO_H8 (use eEvb2CommPorts)");
    ADD_MAP_7("portOptions", portOptions, DATA_TYPE_UINT32, uint32_t, "", "EVB port options:  0x1=radio RTK filter ", DATA_FLAGS_DISPLAY_HEX);
    ADD_MAP_7("bits", bits, DATA_TYPE_UINT32, uint32_t, "", "Configuration bits (see eEvb2ConfigBits). 0x10=stream PPD on log button", DATA_FLAGS_DISPLAY_HEX);
    ADD_MAP_7("radioPID", radioPID, DATA_TYPE_UINT32, uint32_t, "", "Radio Preamble ID in hexadecimal. 0x0 to 0x9", DATA_FLAGS_DISPLAY_HEX);
    ADD_MAP_7("radioNID", radioNID, DATA_TYPE_UINT32, uint32_t, "", "Radio Network ID in hexadecimal. 0x0 to 0x7FFF", DATA_FLAGS_DISPLAY_HEX);
    ADD_MAP_6("radioPowerLevel", radioPowerLevel, DATA_TYPE_UINT32, uint32_t, "", "Radio transmitter output power level. (XBee PRO SX 0=20dBm, 1=27dBm, 2=30dBm)");

    ADD_MAP_6("wifi[0].ssid", wifi[0].ssid, DATA_TYPE_STRING, char[WIFI_SSID_PSK_SIZE], "", "WiFi Service Set Identifier (SSID) or network name.");
    ADD_MAP_6("wifi[0].psk",  wifi[0].psk,  DATA_TYPE_STRING, char[WIFI_SSID_PSK_SIZE], "", "WiFi Pre-Shared Key (PSK) authentication or network password.");
    ADD_MAP_6("wifi[1].ssid", wifi[1].ssid, DATA_TYPE_STRING, char[WIFI_SSID_PSK_SIZE], "", "WiFi Service Set Identifier (SSID) or network name.");
    ADD_MAP_6("wifi[1].psk",  wifi[1].psk,  DATA_TYPE_STRING, char[WIFI_SSID_PSK_SIZE], "", "WiFi Pre-Shared Key (PSK) authentication or network password.");
    ADD_MAP_6("wifi[2].ssid", wifi[2].ssid, DATA_TYPE_STRING, char[WIFI_SSID_PSK_SIZE], "", "WiFi Service Set Identifier (SSID) or network name.");
    ADD_MAP_6("wifi[2].psk",  wifi[2].psk,  DATA_TYPE_STRING, char[WIFI_SSID_PSK_SIZE], "", "WiFi Pre-Shared Key (PSK) authentication or network password.");
    ADD_MAP_6("server[0].ipAddr[0]", server[0].ipAddr.u8[0], DATA_TYPE_UINT8, uint8_t&, "", "Server IP address");
    ADD_MAP_6("server[0].ipAddr[1]", server[0].ipAddr.u8[1], DATA_TYPE_UINT8, uint8_t&, "", "Server IP address");
    ADD_MAP_6("server[0].ipAddr[2]", server[0].ipAddr.u8[2], DATA_TYPE_UINT8, uint8_t&, "", "Server IP address");
    ADD_MAP_6("server[0].ipAddr[3]", server[0].ipAddr.u8[3], DATA_TYPE_UINT8, uint8_t&, "", "Server IP address");
    ADD_MAP_6("server[0].port", server[0].port, DATA_TYPE_UINT32, uint32_t, "", "Sever port");
    ADD_MAP_6("server[1].ipAddr[0]", server[1].ipAddr.u8[0], DATA_TYPE_UINT8, uint8_t&, "", "Server IP address");
    ADD_MAP_6("server[1].ipAddr[1]", server[1].ipAddr.u8[1], DATA_TYPE_UINT8, uint8_t&, "", "Server IP address");
    ADD_MAP_6("server[1].ipAddr[2]", server[1].ipAddr.u8[2], DATA_TYPE_UINT8, uint8_t&, "", "Server IP address");
    ADD_MAP_6("server[1].ipAddr[3]", server[1].ipAddr.u8[3], DATA_TYPE_UINT8, uint8_t&, "", "Server IP address");
    ADD_MAP_6("server[1].port", server[1].port, DATA_TYPE_UINT32, uint32_t, "", "Sever port");
    ADD_MAP_6("server[2].ipAddr[0]", server[2].ipAddr.u8[0], DATA_TYPE_UINT8, uint8_t&, "", "Server IP address");
    ADD_MAP_6("server[2].ipAddr[1]", server[2].ipAddr.u8[1], DATA_TYPE_UINT8, uint8_t&, "", "Server IP address");
    ADD_MAP_6("server[2].ipAddr[2]", server[2].ipAddr.u8[2], DATA_TYPE_UINT8, uint8_t&, "", "Server IP address");
    ADD_MAP_6("server[2].ipAddr[3]", server[2].ipAddr.u8[3], DATA_TYPE_UINT8, uint8_t&, "", "Server IP address");
    ADD_MAP_6("server[2].port", server[2].port, DATA_TYPE_UINT32, uint32_t, "", "Sever port");

    ADD_MAP_6("encoderTickToWheelRad", encoderTickToWheelRad, DATA_TYPE_F32, float, "rad/tick", "Wheel encoder tick to wheel rotation scalar");
    ADD_MAP_6("CANbaud_kbps", CANbaud_kbps, DATA_TYPE_UINT32, uint32_t, "kbps", "CAN baud rate");
    ADD_MAP_7("can_receive_address", can_receive_address, DATA_TYPE_UINT32, uint32_t, "", "CAN Receive Address", DATA_FLAGS_DISPLAY_HEX);
    ADD_MAP_4("reserved2[0]", reserved2[0], DATA_TYPE_UINT8, uint8_t&);
    ADD_MAP_4("reserved2[1]", reserved2[1], DATA_TYPE_UINT8, uint8_t&);

    ADD_MAP_6("h3sp330BaudRate", h3sp330BaudRate, DATA_TYPE_UINT32, uint32_t, "", "Baud rate for EVB serial port on H3 (SP330 RS233 and RS485/422).");
    ADD_MAP_6("h4xRadioBaudRate", h4xRadioBaudRate, DATA_TYPE_UINT32, uint32_t, "", "Baud rate for EVB serial port H4 (TLL to external radio).");
    ADD_MAP_6("h8gpioBaudRate", h8gpioBaudRate, DATA_TYPE_UINT32, uint32_t, "", "Baud rate for EVB serial port H8 (TLL).");
    ADD_MAP_7("wheelCfgBits", wheelCfgBits, DATA_TYPE_UINT32, uint32_t, "", "(eWheelCfgBits). Reverse encoder [0x100 left, 0x200 right, 0x300 both], enable [0x2 encoder, 0x4 wheel]", DATA_FLAGS_DISPLAY_HEX);
    ADD_MAP_6("velocityControlPeriodMs", velocityControlPeriodMs, DATA_TYPE_UINT32, uint32_t, "ms", "Wheel encoder and control update period");

    ADD_MAP_6("size", size, DATA_TYPE_UINT32, uint32_t, "", "Flash group size. Set to 1 to reset this flash group.");
    ADD_MAP_6("checksum", checksum, DATA_TYPE_UINT32, uint32_t, "", "Flash checksum");
    ADD_MAP_6("key", key, DATA_TYPE_UINT32, uint32_t, "", "Flash key");
    ASSERT_SIZE(totalSize);
}

static void PopulateMapDebugArray(data_set_t data_set[DID_COUNT], uint32_t did)
{
    DataMapper<debug_array_t> mapper(data_set, did);
    mapper.AddArray("i", &debug_array_t::i, DATA_TYPE_UINT32, DEBUG_I_ARRAY_SIZE);
    mapper.AddArray("f", &debug_array_t::f, DATA_TYPE_F32, DEBUG_F_ARRAY_SIZE);
    mapper.AddArray("lf", &debug_array_t::lf, DATA_TYPE_F64, DEBUG_LF_ARRAY_SIZE);
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
    mapper.AddArray("baseToRoverVector", &gps_rtk_rel_t::baseToRoverVector, DATA_TYPE_F32, 3, "m", "Vector from base to rover in ECEF.", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_2);
    mapper.AddMember("differentialAge", &gps_rtk_rel_t::differentialAge, DATA_TYPE_F32, "s", "Age of differential signal received.", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3);
    mapper.AddMember("arRatio", &gps_rtk_rel_t::arRatio, DATA_TYPE_F32, "", "Ambiguity resolution ratio factor for validation.", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_1);
    mapper.AddMember("baseToRoverDistance", &gps_rtk_rel_t::baseToRoverDistance, DATA_TYPE_F32, "", "baseToRoverDistance (m)", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3);
    mapper.AddMember("baseToRoverHeading", &gps_rtk_rel_t::baseToRoverHeading, DATA_TYPE_F32, SYM_DEG, "Angle from north to baseToRoverVector in local tangent plane.", DATA_FLAGS_READ_ONLY | DATA_FLAGS_ANGLE | DATA_FLAGS_FIXED_DECIMAL_4, C_RAD2DEG);
    mapper.AddMember("baseToRoverHeadingAcc", &gps_rtk_rel_t::baseToRoverHeadingAcc, DATA_TYPE_F32, SYM_DEG, "Accuracy of baseToRoverHeading.", DATA_FLAGS_READ_ONLY | DATA_FLAGS_ANGLE | DATA_FLAGS_FIXED_DECIMAL_6, C_RAD2DEG);
    mapper.AddMember("status", &gps_rtk_rel_t::status, DATA_TYPE_UINT32, "", "GPS status: [0x000000xx] number of satellites used, [0x0000xx00] fix type, [0x00xx0000] status flags", DATA_FLAGS_READ_ONLY | DATA_FLAGS_DISPLAY_HEX);
}

static void PopulateMapGpsRtkMisc(data_set_t data_set[DID_COUNT], uint32_t did)
{
    DataMapper<gps_rtk_misc_t> mapper(data_set, did);
    mapper.AddMember("timeOfWeekMs", &gps_rtk_misc_t::timeOfWeekMs, DATA_TYPE_UINT32, "ms", "Time of week since Sunday morning", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_4);
    mapper.AddArray("accuracyPos", &gps_rtk_misc_t::accuracyPos, DATA_TYPE_F32, 3, "m", "Accuracy in meters north, east, up (standard deviation)", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_4);
    mapper.AddArray("accuracyCov", &gps_rtk_misc_t::accuracyCov, DATA_TYPE_F32, 3, "m", "Absolute value of means square root of estimated covariance NE, EU, UN", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_4);
    mapper.AddMember("arThreshold", &gps_rtk_misc_t::arThreshold, DATA_TYPE_F32, "", "Ambiguity resolution threshold for validation", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3);
    mapper.AddMember("gDop", &gps_rtk_misc_t::gDop, DATA_TYPE_F32, "m", "Geomatic dilution of precision", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_4);
    mapper.AddMember("hDop", &gps_rtk_misc_t::hDop, DATA_TYPE_F32, "m", "Horizontal dilution of precision", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_4);
    mapper.AddMember("vDop", &gps_rtk_misc_t::vDop, DATA_TYPE_F32, "m", "Vertical dilution of precision", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_4);
    mapper.AddArray("baseLla", &gps_rtk_misc_t::baseLla, DATA_TYPE_F64, 3, SYM_DEG_DEG_M, "Base position in latitude, longitude, altitude", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_7);
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
    mapper.AddMember("timeToFirstFixMs", &gps_rtk_misc_t::timeToFirstFixMs, DATA_TYPE_UINT32, "ms", "Time to first RTK fix", DATA_FLAGS_READ_ONLY);
}

static void PopulateMapGpsRaw(data_set_t data_set[DID_COUNT], uint32_t did)
{
    INIT_MAP(gps_raw_t, did);
    ADD_MAP_7("receiveIndex", receiverIndex, DATA_TYPE_UINT8, uint8_t, "", "Receiver index (1=Rover, 2=Base). RTK positioning or RTK compassing must be enabled to stream raw GPS data.", DATA_FLAGS_READ_ONLY);
    ADD_MAP_7("dataType", dataType, DATA_TYPE_UINT8, uint8_t, "", "Type of data (eRawDataType: 1=observations, 2=ephemeris, 3=glonassEphemeris, 4=SBAS, 5=baseAntenna, 6=IonosphereModel)", DATA_FLAGS_READ_ONLY);
    ADD_MAP_7("obsCount", obsCount, DATA_TYPE_UINT8, uint8_t, "", "Number of observations in array (obsd_t) when dataType==1", DATA_FLAGS_READ_ONLY);
    ADD_MAP_7("reserved", reserved, DATA_TYPE_UINT8, uint8_t, "", "Reserved", DATA_FLAGS_READ_ONLY);
    ADD_MAP_4("dataBuf", data.buf, DATA_TYPE_BINARY, uint8_t[MEMBERSIZE(MAP_TYPE, data.buf)]);
    ASSERT_SIZE(totalSize);
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
    INIT_MAP(rtos_info_t, did);
    ADD_MAP_7("freeHeapSize", freeHeapSize, DATA_TYPE_UINT32, uint32_t, "", "Heap unused bytes (high-water mark)", DATA_FLAGS_READ_ONLY);
    ADD_MAP_7("mallocSize", mallocSize, DATA_TYPE_UINT32, uint32_t, "", "Total memory allocated using malloc()", DATA_FLAGS_READ_ONLY);
    ADD_MAP_7("freeSize", freeSize, DATA_TYPE_UINT32, uint32_t, "", "Total memory freed using free()", DATA_FLAGS_READ_ONLY);

    ADD_MAP_6("T0_name", task[0].name, DATA_TYPE_STRING, char[MAX_TASK_NAME_LEN], "", "Task name");
    ADD_MAP_7("T0_cpuUsage", task[0].cpuUsage, DATA_TYPE_F32, f_t, "%", "CPU usage", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3);
    ADD_MAP_6("T0_stackUnused", task[0].stackUnused, DATA_TYPE_UINT32, uint32_t, "", "Task stack unused bytes (high-water mark)");
    ADD_MAP_6("T0_priority", task[0].priority, DATA_TYPE_UINT32, uint32_t, "", "Task priority");
    ADD_MAP_7("T0_periodMs", task[0].periodMs, DATA_TYPE_UINT32, uint32_t, "ms", "Task period", DATA_FLAGS_READ_ONLY);
    ADD_MAP_6("T0_runtimeUs", task[0].runtimeUs, DATA_TYPE_UINT32, uint32_t, "us", "Task execution time");
    ADD_MAP_7("T0_avgRuntimeUs", task[0].avgRuntimeUs, DATA_TYPE_F32, float, "us", "Average runtime", DATA_FLAGS_FIXED_DECIMAL_2);
    ADD_MAP_7("T0_avgLowerRuntimeUs", task[0].lowerRuntimeUs, DATA_TYPE_F32, float, "us", "Average of runtimes less than avgRuntimeUs", DATA_FLAGS_FIXED_DECIMAL_2);
    ADD_MAP_7("T0_avgUpperRuntimeUs", task[0].upperRuntimeUs, DATA_TYPE_F32, float, "us", "Average of runtimes greater than avgRuntimeUs", DATA_FLAGS_FIXED_DECIMAL_2);
    ADD_MAP_6("T0_maxRuntimeUs", task[0].maxRuntimeUs, DATA_TYPE_UINT32, uint32_t, "us", "Task max execution time");
    ADD_MAP_6("T0_startTimeUs", task[0].startTimeUs, DATA_TYPE_UINT32, uint32_t, "us", "");
    ADD_MAP_6("T0_gapCount", task[0].gapCount, DATA_TYPE_UINT16, uint16_t, "", "Number of times task took too long");
    ADD_MAP_6("T0_doubleGapCount", task[0].doubleGapCount, DATA_TYPE_UINT8, uint8_t, "", "Number of times task took too long twice in a row");
    ADD_MAP_4("T0_reserved", task[0].reserved, DATA_TYPE_UINT8, uint8_t);
    ADD_MAP_4("T0_handle", task[0].handle, DATA_TYPE_UINT32, uint32_t);

    ADD_MAP_6("T1_name", task[1].name, DATA_TYPE_STRING, char[MAX_TASK_NAME_LEN], "", "Task name");
    ADD_MAP_7("T1_cpuUsage", task[1].cpuUsage, DATA_TYPE_F32, f_t, "%", "CPU usage", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3);
    ADD_MAP_6("T1_stackUnused", task[1].stackUnused, DATA_TYPE_UINT32, uint32_t, "", "Task stack unused bytes (high-water mark)");
    ADD_MAP_6("T1_priority", task[1].priority, DATA_TYPE_UINT32, uint32_t, "", "Task priority");
    ADD_MAP_7("T1_periodMs", task[1].periodMs, DATA_TYPE_UINT32, uint32_t, "ms", "Task period", DATA_FLAGS_READ_ONLY);
    ADD_MAP_6("T1_runtimeUs", task[1].runtimeUs, DATA_TYPE_UINT32, uint32_t, "us", "Task execution time");
    ADD_MAP_7("T1_avgRuntimeUs", task[1].avgRuntimeUs, DATA_TYPE_F32, float, "us", "Average runtime", DATA_FLAGS_FIXED_DECIMAL_2);
    ADD_MAP_7("T1_avgLowerRuntimeUs", task[1].lowerRuntimeUs, DATA_TYPE_F32, float, "us", "Average of runtimes less than avgRuntimeUs", DATA_FLAGS_FIXED_DECIMAL_2);
    ADD_MAP_7("T1_avgUpperRuntimeUs", task[1].upperRuntimeUs, DATA_TYPE_F32, float, "us", "Average of runtimes greater than avgRuntimeUs", DATA_FLAGS_FIXED_DECIMAL_2);
    ADD_MAP_6("T1_maxRuntimeUs", task[1].maxRuntimeUs, DATA_TYPE_UINT32, uint32_t, "us", "Task max execution time");
    ADD_MAP_6("T1_startTimeUs", task[1].startTimeUs, DATA_TYPE_UINT32, uint32_t, "us", "");
    ADD_MAP_6("T1_gapCount", task[1].gapCount, DATA_TYPE_UINT16, uint16_t, "", "Number of times task took too long");
    ADD_MAP_6("T1_doubleGapCount", task[1].doubleGapCount, DATA_TYPE_UINT8, uint8_t, "", "Number of times task took too long twice in a row");
    ADD_MAP_4("T1_reserved", task[1].reserved, DATA_TYPE_UINT8, uint8_t);
    ADD_MAP_4("T1_handle", task[1].handle, DATA_TYPE_UINT32, uint32_t);

    ADD_MAP_6("T2_name", task[2].name, DATA_TYPE_STRING, char[MAX_TASK_NAME_LEN], "", "Task name");
    ADD_MAP_7("T2_cpuUsage", task[2].cpuUsage, DATA_TYPE_F32, f_t, "%", "CPU usage", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3);
    ADD_MAP_6("T2_stackUnused", task[2].stackUnused, DATA_TYPE_UINT32, uint32_t, "", "Task stack unused bytes (high-water mark)");
    ADD_MAP_6("T2_priority", task[2].priority, DATA_TYPE_UINT32, uint32_t, "", "Task priority");
    ADD_MAP_7("T2_periodMs", task[2].periodMs, DATA_TYPE_UINT32, uint32_t, "ms", "Task period", DATA_FLAGS_READ_ONLY);
    ADD_MAP_6("T2_runtimeUs", task[2].runtimeUs, DATA_TYPE_UINT32, uint32_t, "us", "Task execution time");
    ADD_MAP_7("T2_avgRuntimeUs", task[2].avgRuntimeUs, DATA_TYPE_F32, float, "us", "Average runtime", DATA_FLAGS_FIXED_DECIMAL_2);
    ADD_MAP_7("T2_avgLowerRuntimeUs", task[2].lowerRuntimeUs, DATA_TYPE_F32, float, "us", "Average of runtimes less than avgRuntimeUs", DATA_FLAGS_FIXED_DECIMAL_2);
    ADD_MAP_7("T2_avgUpperRuntimeUs", task[2].upperRuntimeUs, DATA_TYPE_F32, float, "us", "Average of runtimes greater than avgRuntimeUs", DATA_FLAGS_FIXED_DECIMAL_2);
    ADD_MAP_6("T2_maxRuntimeUs", task[2].maxRuntimeUs, DATA_TYPE_UINT32, uint32_t, "us", "Task max execution time");
    ADD_MAP_6("T2_startTimeUs", task[2].startTimeUs, DATA_TYPE_UINT32, uint32_t, "us", "");
    ADD_MAP_6("T2_gapCount", task[2].gapCount, DATA_TYPE_UINT16, uint16_t, "", "Number of times task took too long");
    ADD_MAP_6("T2_doubleGapCount", task[2].doubleGapCount, DATA_TYPE_UINT8, uint8_t, "", "Number of times task took too long twice in a row");
    ADD_MAP_4("T2_reserved", task[2].reserved, DATA_TYPE_UINT8, uint8_t);
    ADD_MAP_4("T2_handle", task[2].handle, DATA_TYPE_UINT32, uint32_t);

    ADD_MAP_6("T3_name", task[3].name, DATA_TYPE_STRING, char[MAX_TASK_NAME_LEN], "", "Task name");
    ADD_MAP_7("T3_cpuUsage", task[3].cpuUsage, DATA_TYPE_F32, f_t, "%", "CPU usage", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3);
    ADD_MAP_6("T3_stackUnused", task[3].stackUnused, DATA_TYPE_UINT32, uint32_t, "", "Task stack unused bytes (high-water mark)");
    ADD_MAP_6("T3_priority", task[3].priority, DATA_TYPE_UINT32, uint32_t, "", "Task priority");
    ADD_MAP_7("T3_periodMs", task[3].periodMs, DATA_TYPE_UINT32, uint32_t, "ms", "Task period", DATA_FLAGS_READ_ONLY);
    ADD_MAP_6("T3_runtimeUs", task[3].runtimeUs, DATA_TYPE_UINT32, uint32_t, "us", "Task execution time");
    ADD_MAP_7("T3_avgRuntimeUs", task[3].avgRuntimeUs, DATA_TYPE_F32, float, "us", "Average runtime", DATA_FLAGS_FIXED_DECIMAL_2);
    ADD_MAP_7("T3_avgLowerRuntimeUs", task[3].lowerRuntimeUs, DATA_TYPE_F32, float, "us", "Average of runtimes less than avgRuntimeUs", DATA_FLAGS_FIXED_DECIMAL_2);
    ADD_MAP_7("T3_avgUpperRuntimeUs", task[3].upperRuntimeUs, DATA_TYPE_F32, float, "us", "Average of runtimes greater than avgRuntimeUs", DATA_FLAGS_FIXED_DECIMAL_2);
    ADD_MAP_6("T3_maxRuntimeUs", task[3].maxRuntimeUs, DATA_TYPE_UINT32, uint32_t, "us", "Task max execution time");
    ADD_MAP_6("T3_startTimeUs", task[3].startTimeUs, DATA_TYPE_UINT32, uint32_t, "us", "");
    ADD_MAP_6("T3_gapCount", task[3].gapCount, DATA_TYPE_UINT16, uint16_t, "", "Number of times task took too long");
    ADD_MAP_6("T3_doubleGapCount", task[3].doubleGapCount, DATA_TYPE_UINT8, uint8_t, "", "Number of times task took too long twice in a row");
    ADD_MAP_4("T3_reserved", task[3].reserved, DATA_TYPE_UINT8, uint8_t);
    ADD_MAP_4("T3_handle", task[3].handle, DATA_TYPE_UINT32, uint32_t);

    ADD_MAP_6("T4_name", task[4].name, DATA_TYPE_STRING, char[MAX_TASK_NAME_LEN], "", "Task name");
    ADD_MAP_7("T4_cpuUsage", task[4].cpuUsage, DATA_TYPE_F32, f_t, "%", "CPU usage", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3);
    ADD_MAP_6("T4_stackUnused", task[4].stackUnused, DATA_TYPE_UINT32, uint32_t, "", "Task stack unused bytes (high-water mark)");
    ADD_MAP_6("T4_priority", task[4].priority, DATA_TYPE_UINT32, uint32_t, "", "Task priority");
    ADD_MAP_7("T4_periodMs", task[4].periodMs, DATA_TYPE_UINT32, uint32_t, "ms", "Task period", DATA_FLAGS_READ_ONLY);
    ADD_MAP_6("T4_runtimeUs", task[4].runtimeUs, DATA_TYPE_UINT32, uint32_t, "us", "Task execution time");
    ADD_MAP_7("T4_avgRuntimeUs", task[4].avgRuntimeUs, DATA_TYPE_F32, float, "us", "Average runtime", DATA_FLAGS_FIXED_DECIMAL_2);
    ADD_MAP_7("T4_avgLowerRuntimeUs", task[4].lowerRuntimeUs, DATA_TYPE_F32, float, "us", "Average of runtimes less than avgRuntimeUs", DATA_FLAGS_FIXED_DECIMAL_2);
    ADD_MAP_7("T4_avgUpperRuntimeUs", task[4].upperRuntimeUs, DATA_TYPE_F32, float, "us", "Average of runtimes greater than avgRuntimeUs", DATA_FLAGS_FIXED_DECIMAL_2);
    ADD_MAP_6("T4_maxRuntimeUs", task[4].maxRuntimeUs, DATA_TYPE_UINT32, uint32_t, "us", "Task max execution time");
    ADD_MAP_6("T4_startTimeUs", task[4].startTimeUs, DATA_TYPE_UINT32, uint32_t, "us", "");
    ADD_MAP_6("T4_gapCount", task[4].gapCount, DATA_TYPE_UINT16, uint16_t, "", "Number of times task took too long");
    ADD_MAP_6("T4_doubleGapCount", task[4].doubleGapCount, DATA_TYPE_UINT8, uint8_t, "", "Number of times task took too long twice in a row");
    ADD_MAP_4("T4_reserved", task[4].reserved, DATA_TYPE_UINT8, uint8_t);
    ADD_MAP_4("T4_handle", task[4].handle, DATA_TYPE_UINT32, uint32_t);

    ADD_MAP_6("T5_name", task[5].name, DATA_TYPE_STRING, char[MAX_TASK_NAME_LEN], "", "Task name");
    ADD_MAP_7("T5_cpuUsage", task[5].cpuUsage, DATA_TYPE_F32, f_t, "%", "CPU usage", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3);
    ADD_MAP_6("T5_stackUnused", task[5].stackUnused, DATA_TYPE_UINT32, uint32_t, "", "Task stack unused bytes (high-water mark)");
    ADD_MAP_6("T5_priority", task[5].priority, DATA_TYPE_UINT32, uint32_t, "", "Task priority");
    ADD_MAP_7("T5_periodMs", task[5].periodMs, DATA_TYPE_UINT32, uint32_t, "ms", "Task period", DATA_FLAGS_READ_ONLY);
    ADD_MAP_6("T5_runtimeUs", task[5].runtimeUs, DATA_TYPE_UINT32, uint32_t, "us", "Task execution time");
    ADD_MAP_7("T5_avgRuntimeUs", task[5].avgRuntimeUs, DATA_TYPE_F32, float, "us", "Average runtime", DATA_FLAGS_FIXED_DECIMAL_2);
    ADD_MAP_7("T5_avgLowerRuntimeUs", task[5].lowerRuntimeUs, DATA_TYPE_F32, float, "us", "Average of runtimes less than avgRuntimeUs", DATA_FLAGS_FIXED_DECIMAL_2);
    ADD_MAP_7("T5_avgUpperRuntimeUs", task[5].upperRuntimeUs, DATA_TYPE_F32, float, "us", "Average of runtimes greater than avgRuntimeUs", DATA_FLAGS_FIXED_DECIMAL_2);
    ADD_MAP_6("T5_maxRuntimeUs", task[5].maxRuntimeUs, DATA_TYPE_UINT32, uint32_t, "us", "Task max execution time");
    ADD_MAP_6("T5_startTimeUs", task[5].startTimeUs, DATA_TYPE_UINT32, uint32_t, "us", "");
    ADD_MAP_6("T5_gapCount", task[5].gapCount, DATA_TYPE_UINT16, uint16_t, "", "Number of times task took too long");
    ADD_MAP_6("T5_doubleGapCount", task[5].doubleGapCount, DATA_TYPE_UINT8, uint8_t, "", "Number of times task took too long twice in a row");
    ADD_MAP_4("T5_reserved", task[5].reserved, DATA_TYPE_UINT8, uint8_t);
    ADD_MAP_4("T5_handle", task[5].handle, DATA_TYPE_UINT32, uint32_t);
    ASSERT_SIZE(totalSize);
}
static void PopulateMapCanConfig(data_set_t data_set[DID_COUNT], uint32_t did)
{
    DataMapper<can_config_t> mapper(data_set, did);
    mapper.AddArray("can_period_mult", &can_config_t::can_period_mult, DATA_TYPE_UINT16, NUM_CIDS, " ", "Broadcast Period Multiple for CID_INS_TIME Messages");
    mapper.AddArray("can_transmit_address", &can_config_t::can_transmit_address, DATA_TYPE_UINT32, NUM_CIDS, "", "CAN Address CID_INS_TIME Messages", DATA_FLAGS_DISPLAY_HEX);

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
    INIT_MAP(sys_sensors_adc_t, DID_SENSORS_ADC);
    ADD_MAP_7("time", time, DATA_TYPE_F64, double, "s", "Time since boot up", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3);
    ADD_MAP_7("pqr1[0]", imu[0].pqr[0], DATA_TYPE_F32, float&, "LSB", "", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_2);
    ADD_MAP_7("pqr1[1]", imu[0].pqr[1], DATA_TYPE_F32, float&, "LSB", "", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_2);
    ADD_MAP_7("pqr1[2]", imu[0].pqr[2], DATA_TYPE_F32, float&, "LSB", "", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_2);
    ADD_MAP_7("acc1[0]", imu[0].acc[0], DATA_TYPE_F32, float&, "LSB", "", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_2);
    ADD_MAP_7("acc1[1]", imu[0].acc[1], DATA_TYPE_F32, float&, "LSB", "", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_2);
    ADD_MAP_7("acc1[2]", imu[0].acc[2], DATA_TYPE_F32, float&, "LSB", "", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_2);
    ADD_MAP_7("temp1",   imu[0].temp, DATA_TYPE_F32, float, "LSB", "", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3);
    ADD_MAP_7("pqr2[0]", imu[1].pqr[0], DATA_TYPE_F32, float&, "LSB", "", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_2);
    ADD_MAP_7("pqr2[1]", imu[1].pqr[1], DATA_TYPE_F32, float&, "LSB", "", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_2);
    ADD_MAP_7("pqr2[2]", imu[1].pqr[2], DATA_TYPE_F32, float&, "LSB", "", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_2);
    ADD_MAP_7("acc2[0]", imu[1].acc[0], DATA_TYPE_F32, float&, "LSB", "", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_2);
    ADD_MAP_7("acc2[1]", imu[1].acc[1], DATA_TYPE_F32, float&, "LSB", "", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_2);
    ADD_MAP_7("acc2[2]", imu[1].acc[2], DATA_TYPE_F32, float&, "LSB", "", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_2);
    ADD_MAP_7("temp2", imu[1].temp, DATA_TYPE_F32, float, "LSB", "", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3);
    ADD_MAP_7("pqr3[0]", imu[2].pqr[0], DATA_TYPE_F32, float&, "LSB", "", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_2);
    ADD_MAP_7("pqr3[1]", imu[2].pqr[1], DATA_TYPE_F32, float&, "LSB", "", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_2);
    ADD_MAP_7("pqr3[2]", imu[2].pqr[2], DATA_TYPE_F32, float&, "LSB", "", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_2);
    ADD_MAP_7("acc3[0]", imu[2].acc[0], DATA_TYPE_F32, float&, "LSB", "", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_2);
    ADD_MAP_7("acc3[1]", imu[2].acc[1], DATA_TYPE_F32, float&, "LSB", "", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_2);
    ADD_MAP_7("acc3[2]", imu[2].acc[2], DATA_TYPE_F32, float&, "LSB", "", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_2);
    ADD_MAP_7("temp3", imu[2].temp, DATA_TYPE_F32, float, "LSB", "", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3);
    ADD_MAP_7("mag1[0]", mag[0].mag[0], DATA_TYPE_F32, float&, "LSB", "", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_2);
    ADD_MAP_7("mag1[1]", mag[0].mag[1], DATA_TYPE_F32, float&, "LSB", "", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_2);
    ADD_MAP_7("mag1[2]", mag[0].mag[2], DATA_TYPE_F32, float&, "LSB", "", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_2);
    ADD_MAP_7("mag2[0]", mag[1].mag[0], DATA_TYPE_F32, float&, "LSB", "", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_2);
    ADD_MAP_7("mag2[1]", mag[1].mag[1], DATA_TYPE_F32, float&, "LSB", "", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_2);
    ADD_MAP_7("mag2[2]", mag[1].mag[2], DATA_TYPE_F32, float&, "LSB", "", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_2);
    ADD_MAP_7("bar", bar, DATA_TYPE_F32, float, "kPa", "Barometric pressure", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3);
    ADD_MAP_7("barTemp", barTemp, DATA_TYPE_F32, float, SYM_DEG_C, "Barometer temperature", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3);
    ADD_MAP_7("humidity", humidity, DATA_TYPE_F32, float, "%rH", "Relative humidity, 0%-100%", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3);
    ADD_MAP_7("ana[0]", ana[0], DATA_TYPE_F32, float&, "V", "ADC analog 0 input voltage", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3);
    ADD_MAP_7("ana[1]", ana[1], DATA_TYPE_F32, float&, "V", "ADC analog 1 input voltage", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3);
    ADD_MAP_7("ana[2]", ana[2], DATA_TYPE_F32, float&, "V", "ADC analog 3 input voltage", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3);
    ADD_MAP_7("ana[3]", ana[3], DATA_TYPE_F32, float&, "V", "ADC analog 4 input voltage", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3);
    ASSERT_SIZE(totalSize);
}

static void PopulateMapSensorsWTemp(data_set_t data_set[DID_COUNT], uint32_t did)
{
    INIT_MAP(sensors_w_temp_t, did);
    int flags = DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_2;
    ADD_MAP_7("imu3.time", imu3.time, DATA_TYPE_F64, double, "s", "Time since boot up", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3);
    ADD_MAP_7("imu3.status", imu3.status, DATA_TYPE_UINT32, uint32_t, "", "Status", DATA_FLAGS_READ_ONLY | DATA_FLAGS_DISPLAY_HEX);
    ADD_MAP_8("pqr0[0]", imu3.I[0].pqr[0], DATA_TYPE_F32, float&, SYM_DEG_PER_S, "", flags, C_RAD2DEG);
    ADD_MAP_8("pqr0[1]", imu3.I[0].pqr[1], DATA_TYPE_F32, float&, SYM_DEG_PER_S, "", flags, C_RAD2DEG);
    ADD_MAP_8("pqr0[2]", imu3.I[0].pqr[2], DATA_TYPE_F32, float&, SYM_DEG_PER_S, "", flags, C_RAD2DEG);
    ADD_MAP_7("acc0[0]", imu3.I[0].acc[0], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "", flags);
    ADD_MAP_7("acc0[1]", imu3.I[0].acc[1], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "", flags);
    ADD_MAP_7("acc0[2]", imu3.I[0].acc[2], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "", flags);
    ADD_MAP_8("pqr1[0]", imu3.I[1].pqr[0], DATA_TYPE_F32, float&, SYM_DEG_PER_S, "", flags, C_RAD2DEG);
    ADD_MAP_8("pqr1[1]", imu3.I[1].pqr[1], DATA_TYPE_F32, float&, SYM_DEG_PER_S, "", flags, C_RAD2DEG);
    ADD_MAP_8("pqr1[2]", imu3.I[1].pqr[2], DATA_TYPE_F32, float&, SYM_DEG_PER_S, "", flags, C_RAD2DEG);
    ADD_MAP_7("acc1[0]", imu3.I[1].acc[0], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "", flags);
    ADD_MAP_7("acc1[1]", imu3.I[1].acc[1], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "", flags);
    ADD_MAP_7("acc1[2]", imu3.I[1].acc[2], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "", flags);
    ADD_MAP_8("pqr2[0]", imu3.I[2].pqr[0], DATA_TYPE_F32, float&, SYM_DEG_PER_S, "", flags, C_RAD2DEG);
    ADD_MAP_8("pqr2[1]", imu3.I[2].pqr[1], DATA_TYPE_F32, float&, SYM_DEG_PER_S, "", flags, C_RAD2DEG);
    ADD_MAP_8("pqr2[2]", imu3.I[2].pqr[2], DATA_TYPE_F32, float&, SYM_DEG_PER_S, "", flags, C_RAD2DEG);
    ADD_MAP_7("acc2[0]", imu3.I[2].acc[0], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "", flags);
    ADD_MAP_7("acc2[1]", imu3.I[2].acc[1], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "", flags);
    ADD_MAP_7("acc2[2]", imu3.I[2].acc[2], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "", flags);
    ADD_MAP_7("temp0", temp[0], DATA_TYPE_F32, float&, SYM_DEG_C, "Uncalibrated sensor output.", flags);
    ADD_MAP_7("temp1", temp[1], DATA_TYPE_F32, float&, SYM_DEG_C, "Uncalibrated sensor output.", flags);
    ADD_MAP_7("temp2", temp[2], DATA_TYPE_F32, float&, SYM_DEG_C, "Uncalibrated sensor output.", flags);
    ADD_MAP_7("mag0[0]", mag[0].xyz[0], DATA_TYPE_F32, float&, "", "", flags);
    ADD_MAP_7("mag0[1]", mag[0].xyz[1], DATA_TYPE_F32, float&, "", "", flags);
    ADD_MAP_7("mag0[2]", mag[0].xyz[2], DATA_TYPE_F32, float&, "", "", flags);
    ADD_MAP_7("mag1[0]", mag[1].xyz[0], DATA_TYPE_F32, float&, "", "", flags);
    ADD_MAP_7("mag1[1]", mag[1].xyz[1], DATA_TYPE_F32, float&, "", "", flags);
    ADD_MAP_7("mag1[2]", mag[1].xyz[2], DATA_TYPE_F32, float&, "", "", flags);
    ASSERT_SIZE(totalSize);
}

static void PopulateMapSensors(data_set_t data_set[DID_COUNT], uint32_t did)
{
    INIT_MAP(sensors_t, did);
    int flags = DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3;
    ADD_MAP_6("time", time, DATA_TYPE_F64, double, "s", "GPS time of week (since Sunday morning).");
    ADD_MAP_7("pqr0[0]", mpu[0].pqr[0], DATA_TYPE_F32, float&, SYM_DEG_PER_S, "Temperature compensation bias", flags);
    ADD_MAP_7("pqr0[1]", mpu[0].pqr[1], DATA_TYPE_F32, float&, SYM_DEG_PER_S, "Temperature compensation bias", flags);
    ADD_MAP_7("pqr0[2]", mpu[0].pqr[2], DATA_TYPE_F32, float&, SYM_DEG_PER_S, "Temperature compensation bias", flags);
    ADD_MAP_7("acc0[0]", mpu[0].acc[0], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "Temperature compensation bias", flags);
    ADD_MAP_7("acc0[1]", mpu[0].acc[1], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "Temperature compensation bias", flags);
    ADD_MAP_7("acc0[2]", mpu[0].acc[2], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "Temperature compensation bias", flags);
    ADD_MAP_7("mag0[0]", mpu[0].mag[0], DATA_TYPE_F32, float&, "", "Temperature compensation bias", flags);
    ADD_MAP_7("mag0[1]", mpu[0].mag[1], DATA_TYPE_F32, float&, "", "Temperature compensation bias", flags);
    ADD_MAP_7("mag0[2]", mpu[0].mag[2], DATA_TYPE_F32, float&, "", "Temperature compensation bias", flags);
    ADD_MAP_7("pqr1[0]", mpu[1].pqr[0], DATA_TYPE_F32, float&, SYM_DEG_PER_S, "Temperature compensation bias", flags);
    ADD_MAP_7("pqr1[1]", mpu[1].pqr[1], DATA_TYPE_F32, float&, SYM_DEG_PER_S, "Temperature compensation bias", flags);
    ADD_MAP_7("pqr1[2]", mpu[1].pqr[2], DATA_TYPE_F32, float&, SYM_DEG_PER_S, "Temperature compensation bias", flags);
    ADD_MAP_7("acc1[0]", mpu[1].acc[0], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "Temperature compensation bias", flags);
    ADD_MAP_7("acc1[1]", mpu[1].acc[1], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "Temperature compensation bias", flags);
    ADD_MAP_7("acc1[2]", mpu[1].acc[2], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "Temperature compensation bias", flags);
    ADD_MAP_7("mag1[0]", mpu[1].mag[0], DATA_TYPE_F32, float&, "", "Temperature compensation bias", flags);
    ADD_MAP_7("mag1[1]", mpu[1].mag[1], DATA_TYPE_F32, float&, "", "Temperature compensation bias", flags);
    ADD_MAP_7("mag1[2]", mpu[1].mag[2], DATA_TYPE_F32, float&, "", "Temperature compensation bias", flags);
    ADD_MAP_7("pqr2[0]", mpu[2].pqr[0], DATA_TYPE_F32, float&, SYM_DEG_PER_S, "Temperature compensation bias", flags);
    ADD_MAP_7("pqr2[1]", mpu[2].pqr[1], DATA_TYPE_F32, float&, SYM_DEG_PER_S, "Temperature compensation bias", flags);
    ADD_MAP_7("pqr2[2]", mpu[2].pqr[2], DATA_TYPE_F32, float&, SYM_DEG_PER_S, "Temperature compensation bias", flags);
    ADD_MAP_7("acc2[0]", mpu[2].acc[0], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "Temperature compensation bias", flags);
    ADD_MAP_7("acc2[1]", mpu[2].acc[1], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "Temperature compensation bias", flags);
    ADD_MAP_7("acc2[2]", mpu[2].acc[2], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "Temperature compensation bias", flags);
    ADD_MAP_7("mag2[0]", mpu[2].mag[0], DATA_TYPE_F32, float&, "", "Temperature compensation bias", flags);
    ADD_MAP_7("mag2[1]", mpu[2].mag[1], DATA_TYPE_F32, float&, "", "Temperature compensation bias", flags);
    ADD_MAP_7("mag2[2]", mpu[2].mag[2], DATA_TYPE_F32, float&, "", "Temperature compensation bias", flags);
    ASSERT_SIZE(totalSize);
}

static void PopulateMapSensorCompensation(data_set_t data_set[DID_COUNT], uint32_t did)
{
    INIT_MAP(sensor_compensation_t, did);
    int flags = DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_4;
    string str = 
        TOSTRING(SC_RUNTIME) "=Runtime, Tcal["
        TOSTRING(SC_TCAL_INIT) "=Init, "
        TOSTRING(SC_TCAL_RUNNING) "=Running, "
        TOSTRING(SC_TCAL_STOP) "=Stop, "
        TOSTRING(SC_TCAL_DONE) "=Done], MCAL["
        TOSTRING(SC_MCAL_SAMPLE_INIT) "=Init, "
        TOSTRING(SC_MCAL_SAMPLE_MEAN_UCAL) "=UCAL, "
        TOSTRING(SC_MCAL_SAMPLE_MEAN_TCAL) "=TCAL, "
        TOSTRING(SC_MCAL_SAMPLE_MEAN_MCAL) "=MCAL, "
        TOSTRING(SC_LPF_SAMPLE) "=LPF 0.01Hz, "
        TOSTRING(SC_LPF_SAMPLE_FAST) "=LPF 1Hz ";

    ADD_MAP_7("timeMs", timeMs, DATA_TYPE_UINT32, uint32_t, "ms", "Time since boot up", DATA_FLAGS_READ_ONLY);
    ADD_MAP_6("calState", calState, DATA_TYPE_UINT32, uint32_t, "", str);
    ADD_MAP_7("status", status, DATA_TYPE_UINT32, uint32_t, "", "", DATA_FLAGS_DISPLAY_HEX);
    ADD_MAP_6("sampleCount", sampleCount, DATA_TYPE_UINT32, uint32_t, "", "Used in averaging");
    ADD_MAP_7("alignAccel[0]", alignAccel[0], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "Alignment acceleration", flags);
    ADD_MAP_7("alignAccel[1]", alignAccel[1], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "Alignment acceleration", flags);
    ADD_MAP_7("alignAccel[2]", alignAccel[2], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "Alignment acceleration", flags);

    // Gyros
    ADD_MAP_8("pqr0.lpfLsb[0]", pqr[0].lpfLsb[0], DATA_TYPE_F32, float&, SYM_DEG_PER_S, "Low-pass filtered LSB", flags, C_RAD2DEG);
    ADD_MAP_8("pqr0.lpfLsb[1]", pqr[0].lpfLsb[1], DATA_TYPE_F32, float&, SYM_DEG_PER_S, "Low-pass filtered LSB", flags, C_RAD2DEG);
    ADD_MAP_8("pqr0.lpfLsb[2]", pqr[0].lpfLsb[2], DATA_TYPE_F32, float&, SYM_DEG_PER_S, "Low-pass filtered LSB", flags, C_RAD2DEG);
    ADD_MAP_7("pqr0.lpfTemp", pqr[0].lpfTemp, DATA_TYPE_F32, float, SYM_DEG_C, "Low-pass filtered temperature", flags);
    ADD_MAP_8("pqr0.k[0]", pqr[0].k[0], DATA_TYPE_F32, float&, SYM_DEG_PER_S, "Slope", flags, C_RAD2DEG);
    ADD_MAP_8("pqr0.k[1]", pqr[0].k[1], DATA_TYPE_F32, float&, SYM_DEG_PER_S, "Slope", flags, C_RAD2DEG);
    ADD_MAP_8("pqr0.k[2]", pqr[0].k[2], DATA_TYPE_F32, float&, SYM_DEG_PER_S, "Slope", flags, C_RAD2DEG);
    ADD_MAP_7("pqr0.temp", pqr[0].temp, DATA_TYPE_F32, float, SYM_DEG_C, "Temperature", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_2);
    ADD_MAP_7("pqr0.tempRampRate", pqr[0].tempRampRate, DATA_TYPE_F32, float, SYM_DEG_C_PER_S, "Temperature ramp rate", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3);
    ADD_MAP_7("pqr0.tci", pqr[0].tci, DATA_TYPE_UINT32, uint32_t, "", "Temp comp index", DATA_FLAGS_READ_ONLY);
    ADD_MAP_7("pqr0.numTcPts", pqr[0].numTcPts, DATA_TYPE_UINT32, uint32_t, "", "Number of temp comp points", DATA_FLAGS_READ_ONLY);
    ADD_MAP_7("pqr0.dtTemp", pqr[0].dtTemp, DATA_TYPE_F32, float, SYM_DEG_C, "Delta from last tc point to current temperature", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3);

    ADD_MAP_8("pqr1.lpfLsb[0]", pqr[1].lpfLsb[0], DATA_TYPE_F32, float&, SYM_DEG_PER_S, "Low-pass filtered LSB", flags, C_RAD2DEG);
    ADD_MAP_8("pqr1.lpfLsb[1]", pqr[1].lpfLsb[1], DATA_TYPE_F32, float&, SYM_DEG_PER_S, "Low-pass filtered LSB", flags, C_RAD2DEG);
    ADD_MAP_8("pqr1.lpfLsb[2]", pqr[1].lpfLsb[2], DATA_TYPE_F32, float&, SYM_DEG_PER_S, "Low-pass filtered LSB", flags, C_RAD2DEG);
    ADD_MAP_7("pqr1.lpfTemp", pqr[1].lpfTemp, DATA_TYPE_F32, float, SYM_DEG_C, "Low-pass filtered temperature", flags);
    ADD_MAP_8("pqr1.k[0]", pqr[1].k[0], DATA_TYPE_F32, float&, SYM_DEG_PER_S, "Slope", flags, C_RAD2DEG);
    ADD_MAP_8("pqr1.k[1]", pqr[1].k[1], DATA_TYPE_F32, float&, SYM_DEG_PER_S, "Slope", flags, C_RAD2DEG);
    ADD_MAP_8("pqr1.k[2]", pqr[1].k[2], DATA_TYPE_F32, float&, SYM_DEG_PER_S, "Slope", flags, C_RAD2DEG);
    ADD_MAP_7("pqr1.temp", pqr[1].temp, DATA_TYPE_F32, float, SYM_DEG_C, "Temperature", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_2);
    ADD_MAP_7("pqr1.tempRampRate", pqr[1].tempRampRate, DATA_TYPE_F32, float, SYM_DEG_C_PER_S, "Temperature ramp rate", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3);
    ADD_MAP_7("pqr1.tci", pqr[1].tci, DATA_TYPE_UINT32, uint32_t, "", "Temp comp index", DATA_FLAGS_READ_ONLY);
    ADD_MAP_7("pqr1.numTcPts", pqr[1].numTcPts, DATA_TYPE_UINT32, uint32_t, "", "Number of temp comp points", DATA_FLAGS_READ_ONLY);
    ADD_MAP_7("pqr1.dtTemp", pqr[1].dtTemp, DATA_TYPE_F32, float, SYM_DEG_C, "Delta from last tc point to current temperature", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3);

    ADD_MAP_8("pqr2.lpfLsb[0]", pqr[2].lpfLsb[0], DATA_TYPE_F32, float&, SYM_DEG_PER_S, "Low-pass filtered LSB", flags, C_RAD2DEG);
    ADD_MAP_8("pqr2.lpfLsb[1]", pqr[2].lpfLsb[1], DATA_TYPE_F32, float&, SYM_DEG_PER_S, "Low-pass filtered LSB", flags, C_RAD2DEG);
    ADD_MAP_8("pqr2.lpfLsb[2]", pqr[2].lpfLsb[2], DATA_TYPE_F32, float&, SYM_DEG_PER_S, "Low-pass filtered LSB", flags, C_RAD2DEG);
    ADD_MAP_7("pqr2.lpfTemp", pqr[2].lpfTemp, DATA_TYPE_F32, float, SYM_DEG_C, "Low-pass filtered temperature", flags);
    ADD_MAP_8("pqr2.k[0]", pqr[2].k[0], DATA_TYPE_F32, float&, SYM_DEG_PER_S, "Slope", flags, C_RAD2DEG);
    ADD_MAP_8("pqr2.k[1]", pqr[2].k[1], DATA_TYPE_F32, float&, SYM_DEG_PER_S, "Slope", flags, C_RAD2DEG);
    ADD_MAP_8("pqr2.k[2]", pqr[2].k[2], DATA_TYPE_F32, float&, SYM_DEG_PER_S, "Slope", flags, C_RAD2DEG);
    ADD_MAP_7("pqr2.temp", pqr[2].temp, DATA_TYPE_F32, float, SYM_DEG_C, "Temperature", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_2);
    ADD_MAP_7("pqr2.tempRampRate", pqr[2].tempRampRate, DATA_TYPE_F32, float, SYM_DEG_C_PER_S, "Temperature ramp rate", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3);
    ADD_MAP_7("pqr2.tci", pqr[2].tci, DATA_TYPE_UINT32, uint32_t, "", "Temp comp index", DATA_FLAGS_READ_ONLY);
    ADD_MAP_7("pqr2.numTcPts", pqr[2].numTcPts, DATA_TYPE_UINT32, uint32_t, "", "Number of temp comp points", DATA_FLAGS_READ_ONLY);
    ADD_MAP_7("pqr2.dtTemp", pqr[2].dtTemp, DATA_TYPE_F32, float, SYM_DEG_C, "Delta from last tc point to current temperature", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3);

    // Accels
    ADD_MAP_7("acc0.lpfLsb[0]", acc[0].lpfLsb[0], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "Low-pass filtered LSB", flags);
    ADD_MAP_7("acc0.lpfLsb[1]", acc[0].lpfLsb[1], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "Low-pass filtered LSB", flags);
    ADD_MAP_7("acc0.lpfLsb[2]", acc[0].lpfLsb[2], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "Low-pass filtered LSB", flags);
    ADD_MAP_7("acc0.lpfTemp", acc[0].lpfTemp, DATA_TYPE_F32, float, SYM_DEG_C, "Low-pass filtered temperature", flags);
    ADD_MAP_7("acc0.k[0]", acc[0].k[0], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "Slope", flags);
    ADD_MAP_7("acc0.k[1]", acc[0].k[1], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "Slope", flags);
    ADD_MAP_7("acc0.k[2]", acc[0].k[2], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "Slope", flags);
    ADD_MAP_7("acc0.temp", acc[0].temp, DATA_TYPE_F32, float, SYM_DEG_C, "Temperature", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_2);
    ADD_MAP_7("acc0.tempRampRate", acc[0].tempRampRate, DATA_TYPE_F32, float, SYM_M_PER_S_2, "Temperature ramp rate", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3);
    ADD_MAP_7("acc0.tci", acc[0].tci, DATA_TYPE_UINT32, uint32_t, "", "Temp comp index", DATA_FLAGS_READ_ONLY);
    ADD_MAP_7("acc0.numTcPts", acc[0].numTcPts, DATA_TYPE_UINT32, uint32_t, "", "Number of temp comp points", DATA_FLAGS_READ_ONLY);
    ADD_MAP_7("acc0.dtTemp", acc[0].dtTemp, DATA_TYPE_F32, float, SYM_DEG_C, "Delta from last tc point to current temperature", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3);

    ADD_MAP_7("acc1.lpfLsb[0]", acc[1].lpfLsb[0], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "Low-pass filtered LSB", flags);
    ADD_MAP_7("acc1.lpfLsb[1]", acc[1].lpfLsb[1], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "Low-pass filtered LSB", flags);
    ADD_MAP_7("acc1.lpfLsb[2]", acc[1].lpfLsb[2], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "Low-pass filtered LSB", flags);
    ADD_MAP_7("acc1.lpfTemp", acc[1].lpfTemp, DATA_TYPE_F32, float, SYM_DEG_C, "Low-pass filtered temperature", flags);
    ADD_MAP_7("acc1.k[0]", acc[1].k[0], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "Slope", flags);
    ADD_MAP_7("acc1.k[1]", acc[1].k[1], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "Slope", flags);
    ADD_MAP_7("acc1.k[2]", acc[1].k[2], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "Slope", flags);
    ADD_MAP_7("acc1.temp", acc[1].temp, DATA_TYPE_F32, float, SYM_DEG_C, "Temperature", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_2);
    ADD_MAP_7("acc1.tempRampRate", acc[1].tempRampRate, DATA_TYPE_F32, float, SYM_M_PER_S_2, "Temperature ramp rate", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3);
    ADD_MAP_7("acc1.tci", acc[1].tci, DATA_TYPE_UINT32, uint32_t, "", "Temp comp index", DATA_FLAGS_READ_ONLY);
    ADD_MAP_7("acc1.numTcPts", acc[1].numTcPts, DATA_TYPE_UINT32, uint32_t, "", "Number of temp comp points", DATA_FLAGS_READ_ONLY);
    ADD_MAP_7("acc1.dtTemp", acc[1].dtTemp, DATA_TYPE_F32, float, SYM_DEG_C, "Delta from last tc point to current temperature", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3);

    ADD_MAP_7("acc2.lpfLsb[0]", acc[2].lpfLsb[0], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "Low-pass filtered LSB", flags);
    ADD_MAP_7("acc2.lpfLsb[1]", acc[2].lpfLsb[1], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "Low-pass filtered LSB", flags);
    ADD_MAP_7("acc2.lpfLsb[2]", acc[2].lpfLsb[2], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "Low-pass filtered LSB", flags);
    ADD_MAP_7("acc2.lpfTemp", acc[2].lpfTemp, DATA_TYPE_F32, float, SYM_DEG_C, "Low-pass filtered temperature", flags);
    ADD_MAP_7("acc2.k[0]", acc[2].k[0], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "Slope", flags);
    ADD_MAP_7("acc2.k[1]", acc[2].k[1], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "Slope", flags);
    ADD_MAP_7("acc2.k[2]", acc[2].k[2], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "Slope", flags);
    ADD_MAP_7("acc2.temp", acc[2].temp, DATA_TYPE_F32, float, SYM_DEG_C, "Temperature", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_2);
    ADD_MAP_7("acc2.tempRampRate", acc[2].tempRampRate, DATA_TYPE_F32, float, SYM_M_PER_S_2, "Temperature ramp rate", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3);
    ADD_MAP_7("acc2.tci", acc[2].tci, DATA_TYPE_UINT32, uint32_t, "", "Temp comp index", DATA_FLAGS_READ_ONLY);
    ADD_MAP_7("acc2.numTcPts", acc[2].numTcPts, DATA_TYPE_UINT32, uint32_t, "", "Number of temp comp points", DATA_FLAGS_READ_ONLY);
    ADD_MAP_7("acc2.dtTemp", acc[2].dtTemp, DATA_TYPE_F32, float, SYM_DEG_C, "Delta from last tc point to current temperature", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3);

    // Magnetometers
    ADD_MAP_7("mag0.lpfLsb[0]", mag[0].lpfLsb[0], DATA_TYPE_F32, float&, "", "Low-pass filtered LSB", flags);
    ADD_MAP_7("mag0.lpfLsb[1]", mag[0].lpfLsb[1], DATA_TYPE_F32, float&, "", "Low-pass filtered LSB", flags);
    ADD_MAP_7("mag0.lpfLsb[2]", mag[0].lpfLsb[2], DATA_TYPE_F32, float&, "", "Low-pass filtered LSB", flags);
    ADD_MAP_7("mag0.lpfTemp", mag[0].lpfTemp, DATA_TYPE_F32, float, SYM_DEG_C, "Low-pass filtered temperature", flags);
    ADD_MAP_7("mag0.k[0]", mag[0].k[0], DATA_TYPE_F32, float&, "", "Slope", flags);
    ADD_MAP_7("mag0.k[1]", mag[0].k[1], DATA_TYPE_F32, float&, "", "Slope", flags);
    ADD_MAP_7("mag0.k[2]", mag[0].k[2], DATA_TYPE_F32, float&, "", "Slope", flags);
    ADD_MAP_7("mag0.temp", mag[0].temp, DATA_TYPE_F32, float, SYM_DEG_C, "Temperature", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_2);
    ADD_MAP_7("mag0.tempRampRate", mag[0].tempRampRate, DATA_TYPE_F32, float, "", "Temperature ramp rate", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3);
    ADD_MAP_7("mag0.tci", mag[0].tci, DATA_TYPE_UINT32, uint32_t, "", "Temp comp index", DATA_FLAGS_READ_ONLY);
    ADD_MAP_7("mag0.numTcPts", mag[0].numTcPts, DATA_TYPE_UINT32, uint32_t, "", "Number of temp comp points", DATA_FLAGS_READ_ONLY);
    ADD_MAP_7("mag0.dtTemp", mag[0].dtTemp, DATA_TYPE_F32, float, SYM_DEG_C, "Delta from last tc point to current temperature", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3);

    ADD_MAP_7("mag1.lpfLsb[0]", mag[1].lpfLsb[0], DATA_TYPE_F32, float&, "", "Low-pass filtered LSB", flags);
    ADD_MAP_7("mag1.lpfLsb[1]", mag[1].lpfLsb[1], DATA_TYPE_F32, float&, "", "Low-pass filtered LSB", flags);
    ADD_MAP_7("mag1.lpfLsb[2]", mag[1].lpfLsb[2], DATA_TYPE_F32, float&, "", "Low-pass filtered LSB", flags);
    ADD_MAP_7("mag1.lpfTemp", mag[1].lpfTemp, DATA_TYPE_F32, float, SYM_DEG_C, "Low-pass filtered temperature", flags);
    ADD_MAP_7("mag1.k[0]", mag[1].k[0], DATA_TYPE_F32, float&, "", "Slope", flags);
    ADD_MAP_7("mag1.k[1]", mag[1].k[1], DATA_TYPE_F32, float&, "", "Slope", flags);
    ADD_MAP_7("mag1.k[2]", mag[1].k[2], DATA_TYPE_F32, float&, "", "Slope", flags);
    ADD_MAP_7("mag1.temp", mag[1].temp, DATA_TYPE_F32, float, SYM_DEG_C, "Temperature", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_2);
    ADD_MAP_7("mag1.tempRampRate", mag[1].tempRampRate, DATA_TYPE_F32, float, "", "Temperature ramp rate", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3);
    ADD_MAP_7("mag1.tci", mag[1].tci, DATA_TYPE_UINT32, uint32_t, "", "Temp comp index", DATA_FLAGS_READ_ONLY);
    ADD_MAP_7("mag1.numTcPts", mag[1].numTcPts, DATA_TYPE_UINT32, uint32_t, "", "Number of temp comp points", DATA_FLAGS_READ_ONLY);
    ADD_MAP_7("mag1.dtTemp", mag[1].dtTemp, DATA_TYPE_F32, float, SYM_DEG_C, "Delta from last tc point to current temperature", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3);

    // Reference IMU
    ADD_MAP_8("referenceImu.pqr[0]", referenceImu.pqr[0], DATA_TYPE_F32, float&, SYM_DEG_PER_S, "Reference IMU angular rate", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3, C_RAD2DEG);
    ADD_MAP_8("referenceImu.pqr[1]", referenceImu.pqr[1], DATA_TYPE_F32, float&, SYM_DEG_PER_S, "Reference IMU angular rate", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3, C_RAD2DEG);
    ADD_MAP_8("referenceImu.pqr[2]", referenceImu.pqr[2], DATA_TYPE_F32, float&, SYM_DEG_PER_S, "Reference IMU angular rate", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3, C_RAD2DEG);
    ADD_MAP_7("referenceImu.acc[0]", referenceImu.acc[0], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "Reference IMU linear acceleration", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_7("referenceImu.acc[1]", referenceImu.acc[1], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "Reference IMU linear acceleration", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_7("referenceImu.acc[2]", referenceImu.acc[2], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "Reference IMU linear acceleration", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_4);
    // Reference Mag
    ADD_MAP_7("referenceMag[0]", referenceMag[0], DATA_TYPE_F32, float&, "", "Reference magnetometer", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3);
    ADD_MAP_7("referenceMag[1]", referenceMag[1], DATA_TYPE_F32, float&, "", "Reference magnetometer", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3);
    ADD_MAP_7("referenceMag[2]", referenceMag[2], DATA_TYPE_F32, float&, "", "Reference magnetometer", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3);
    ASSERT_SIZE(totalSize);
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
    mapper.AddArray("Wcal", &inl2_mag_obs_info_t::Wcal, DATA_TYPE_F32, 9, "", "", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3);
    mapper.AddMember("activeCalSet", &inl2_mag_obs_info_t::activeCalSet, DATA_TYPE_UINT32, "", "Active calibration set (0 or 1)");
    mapper.AddMember("magHdgOffset", &inl2_mag_obs_info_t::magHdgOffset, DATA_TYPE_F32, "deg", "Offset from mag heading to ins heading estimate", DATA_FLAGS_FIXED_DECIMAL_3);
    mapper.AddMember("Tcal", &inl2_mag_obs_info_t::Tcal, DATA_TYPE_F32, "", "Scaled computed variance of calibrated magnetometer samples. Above 5 is bad.", DATA_FLAGS_FIXED_DECIMAL_3);
    mapper.AddArray("bias_cal", &inl2_mag_obs_info_t::bias_cal, DATA_TYPE_F32, 3, "", "", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_5);
}

static void PopulateMapInl2States(data_set_t data_set[DID_COUNT], uint32_t did)
{
    DataMapper<inl2_states_t> mapper(data_set, did);
    int flags = DATA_FLAGS_FIXED_DECIMAL_4;
    mapper.AddMember("timeOfWeek", &inl2_states_t::timeOfWeek, DATA_TYPE_F64, "s", "Time of week since Sunday morning, GMT", flags);
    mapper.AddArray("qe2b", &inl2_states_t::qe2b, DATA_TYPE_F32, 4, "", "Quaternion rotation from ECEF to body frame", flags);
    mapper.AddArray("ve", &inl2_states_t::ve, DATA_TYPE_F32, 3, "m/s", "Velocity in ECEF frame", flags);
    mapper.AddArray("ecef", &inl2_states_t::ecef, DATA_TYPE_F64, 3, "m", "Position in ECEF frame", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3);
    mapper.AddArray("biasPqr", &inl2_states_t::biasPqr, DATA_TYPE_F32, 3, SYM_DEG_PER_S, "Gyro bias", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3, C_RAD2DEG);
    mapper.AddArray("biasAcc", &inl2_states_t::biasAcc, DATA_TYPE_F32, 3, SYM_M_PER_S_2, "Accelerometer bias", flags);
    mapper.AddMember("biasBaro", &inl2_states_t::biasBaro, DATA_TYPE_F32, "m", "Barometer bias", flags);
    mapper.AddMember("magDec", &inl2_states_t::magDec, DATA_TYPE_F32, SYM_DEG, "Magnetic declination", flags, C_RAD2DEG);
    mapper.AddMember("magInc", &inl2_states_t::magInc, DATA_TYPE_F32, SYM_DEG, "Magnetic inclination", flags, C_RAD2DEG);
}

static void PopulateMapInl2NedSigma(data_set_t data_set[DID_COUNT], uint32_t did)
{
    DataMapper<inl2_ned_sigma_t> mapper(data_set, did);
    int flags = DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_5;
    mapper.AddMember("timeOfWeekMs", &inl2_ned_sigma_t::timeOfWeekMs, DATA_TYPE_UINT32, "ms", "Time of week since Sunday morning, GMT", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_4);
    mapper.AddArray("StdAttNed", &inl2_ned_sigma_t::StdAttNed, DATA_TYPE_F32, 3, SYM_DEG, "NED attitude error standard deviation", flags, C_RAD2DEG);
    mapper.AddArray("StdVelNed", &inl2_ned_sigma_t::StdVelNed, DATA_TYPE_F32, 3, "m/s", "NED velocity error standard deviation", flags);
    mapper.AddArray("StdPosNed", &inl2_ned_sigma_t::StdPosNed, DATA_TYPE_F32, 3, "m", "NED position error standard deviation", flags);
    mapper.AddArray("StdAccBias", &inl2_ned_sigma_t::StdAccBias, DATA_TYPE_F32, 3, SYM_M_PER_S_2, "Acceleration bias error standard deviation", flags);
    mapper.AddArray("StdGyrBias", &inl2_ned_sigma_t::StdGyrBias, DATA_TYPE_F32, 3, SYM_DEG, "Angular rate bias error standard deviation", flags, C_RAD2DEG);
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
    mapper.AddArray("covPoseLD", &ros_covariance_pose_twist_t::covPoseLD, DATA_TYPE_F32, 21, COV_POSE_UNITS, "EKF attitude and position error covariance matrix lower diagonal in body (attitude) and ECEF (position) frames", flags);
    mapper.AddArray("covTwistLD", &ros_covariance_pose_twist_t::covTwistLD, DATA_TYPE_F32, 21, COV_TWIST_UNITS, "EKF velocity and angular rate error covariance matrix lower diagonal in ECEF (velocity) and body (attitude) frames", flags);
}


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
    "DID_CAL_SC1",                      // 43
    "DID_CAL_SC2",                      // 44
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
#ifdef USE_IS_INTERNAL
    PopulateMapRtkDebug(            m_data_set, DID_RTK_DEBUG);
    PopulateMapRtkDebug(            m_data_set, DID_RTK_DEBUG);
    // PopulateMapRtkDebug2(        m_data_set, DID_RTK_DEBUG_2);
    PopulateMapRuntimeProfile(      m_data_set, DID_RUNTIME_PROFILER);
#endif
    // PopulateMapDiagMsg(          m_data_set, DID_DIAGNOSTIC_MESSAGE);

#if defined(INCLUDE_LUNA_DATA_SETS)
    // LUNA
    PopulateMapEvbLunaFlashCfg(        m_data_set, DID_EVB_LUNA_FLASH_CFG);
    PopulateMapCoyoteStatus(           m_data_set, DID_EVB_LUNA_STATUS);
    PopulateMapEvbLunaSensors(         m_data_set, DID_EVB_LUNA_SENSORS);
    PopulateMapEvbLunaVelocityControl( m_data_set, DID_EVB_LUNA_VELOCITY_CONTROL);
    PopulateMapEvbLunaVelocityCommand( m_data_set, DID_EVB_LUNA_VELOCITY_COMMAND);
    PopulateMapEvbLunaAuxCmd(          m_data_set, DID_EVB_LUNA_AUX_COMMAND);
#endif

    // SOLUTION
    PopulateMapIns1(                m_data_set, DID_INS_1);
    PopulateMapIns2(                m_data_set, DID_INS_2);
    PopulateMapIns3(                m_data_set, DID_INS_3);
    PopulateMapIns4(                m_data_set, DID_INS_4);
    PopulateMapSysParams(           m_data_set, DID_SYS_PARAMS);

    // EKF
    PopulateMapInl2States(          m_data_set, DID_INL2_STATES);
#ifdef USE_IS_INTERNAL
    PopulateMapInl2Status(          m_data_set, DID_INL2_STATUS);
    PopulateMapInl2Misc(            m_data_set, DID_INL2_MISC);
#endif
    PopulateMapInl2NedSigma(        m_data_set, DID_INL2_NED_SIGMA);
    PopulateMapInl2MagObsInfo(      m_data_set, DID_INL2_MAG_OBS_INFO);
    PopulateMapRosCovariancePoseTwist(m_data_set, DID_ROS_COVARIANCE_POSE_TWIST);
#ifdef USE_IS_INTERNAL
    PopulateMapInl2Misc(        m_data_set, DID_INL2_MISC);
#endif
    
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
    PopulateMapGpsSat(              m_data_set, DID_GPS1_SAT, DID_GPS1_SAT);
    PopulateMapGpsSat(              m_data_set, DID_GPS2_SAT, DID_GPS2_SAT);
    PopulateMapGpsSig(              m_data_set, DID_GPS1_SIG, DID_GPS1_SIG);
    PopulateMapGpsSig(              m_data_set, DID_GPS2_SIG, DID_GPS2_SIG);
#endif

    PopulateMapGpsVersion(          m_data_set, DID_GPS1_VERSION);
    PopulateMapGpsVersion(          m_data_set, DID_GPS2_VERSION);
    PopulateMapGpsTimepulse(        m_data_set, DID_GPS1_TIMEPULSE);

    PopulateMapGpsRaw(              m_data_set, DID_GPS1_RAW);
    PopulateMapGpsRaw(              m_data_set, DID_GPS2_RAW);
    PopulateMapGpsRaw(              m_data_set, DID_GPS_BASE_RAW);

#ifdef USE_IS_INTERNAL
//  m_data_set[DID_RTK_STATE].size = sizeof(rtk_state_t);
    m_data_set[DID_RTK_CODE_RESIDUAL].size = sizeof(rtk_residual_t);
    m_data_set[DID_RTK_PHASE_RESIDUAL].size = sizeof(rtk_residual_t);
#endif

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
    // m_data_set[DID_GPX_RTOS_INFO].size = sizeof(gpx_rtos_info_t);
    PopulateMapSystemFault(         m_data_set, DID_SYS_FAULT);

#ifdef USE_IS_INTERNAL
    PopulateMapUserPage0(           m_data_set, DID_NVR_USERPAGE_G0);
    PopulateMapUserPage1(           m_data_set, DID_NVR_USERPAGE_G1);
#endif

    // COMMUNICATIONS
    PopulateMapPortMonitor(         m_data_set, DID_PORT_MONITOR);
    PopulateMapPortMonitor(         m_data_set, DID_GPX_PORT_MONITOR);
    PopulateMapNmeaMsgs(            m_data_set, DID_NMEA_BCAST_PERIOD);
    PopulateMapCanConfig(           m_data_set, DID_CAN_CONFIG);
    PopulateMapRmc(                 m_data_set, DID_RMC);
    PopulateMapRmc(                 m_data_set, DID_GPX_RMC);
    PopulateMapIO(                  m_data_set, DID_IO);
    PopulateMapISEvent(             m_data_set, DID_EVENT);

    // EVB
    PopulateMapEvbStatus(           m_data_set, DID_EVB_STATUS);
    PopulateMapEvbFlashCfg(         m_data_set, DID_EVB_FLASH_CFG);
    PopulateMapDebugArray(          m_data_set, DID_EVB_DEBUG_ARRAY);
    // PopulateMapEvbRtosInfo(      m_data_set, DID_EVB_RTOS_INFO);
    PopulateMapDeviceInfo(          m_data_set, DID_EVB_DEV_INFO);

    // MANUFACTURING
    PopulateMapHdwParams(           m_data_set, DID_HDW_PARAMS);
    PopulateMapManufacturingInfo(   m_data_set, DID_MANUFACTURING_INFO);
    PopulateMapSensorsWTemp(        m_data_set, DID_SENSORS_UCAL);
    PopulateMapSensorsWTemp(        m_data_set, DID_SENSORS_TCAL);
    PopulateMapSensorsWTemp(        m_data_set, DID_SENSORS_MCAL);
    PopulateMapSensors(             m_data_set, DID_SENSORS_TC_BIAS);
    PopulateMapSensorCompensation(  m_data_set, DID_SCOMP);
#ifdef USE_IS_INTERNAL
    // DID_CAL_SC_INFO
    PopulateMapSensorTCalGroup(     m_data_set, DID_CAL_TEMP_COMP);
    PopulateMapSensorMCalGroup(     m_data_set, DID_CAL_MOTION);
#endif

#ifdef USE_IS_INTERNAL
//     PopulateMapRtkState(            m_data_set, DID_RTK_STATE);
//     PopulateMapRtkResidual(         m_data_set, DID_RTK_CODE_RESIDUAL);
//     PopulateMapRtkResidual(         m_data_set, DID_RTK_PHASE_RESIDUAL);
#endif

    // This must come last
    for (uint32_t did = 0; did < DID_COUNT; did++)
    {
        PopulateMapTimestampField(m_data_set, did);
    }
}

cISDataMappings::~cISDataMappings()
{
}

const char* cISDataMappings::GetName(uint32_t did)
{
    STATIC_ASSERT(_ARRAY_ELEMENT_COUNT(m_dataIdNames) == DID_COUNT);

    if (did >= DID_COUNT)
    {
        return "unknown";
    }

    return m_dataIdNames[did];
}

uint32_t cISDataMappings::GetId(string name)
{
//     transform(name.begin(), name.end(), name.begin(), ::toupper);

    for (eDataIDs id = 0; id < DID_COUNT; id++)
    {
        if (strcmp(name.c_str(), m_dataIdNames[id]) == 0)
        {    // Found match
            return id;
        }
    }

    return 0;
}

const map_name_to_info_t* cISDataMappings::GetMapInfo(uint32_t did)
{
    if (did >= DID_COUNT)
    {
        return NULLPTR;
    }

#if PLATFORM_IS_EMBEDDED

    if (s_map == NULLPTR)
    {
        s_map = new cISDataMappings();
    }

    return &s_map->m_data_set[did].nameInfo;

#else

    return &s_map.m_data_set[did].nameInfo;

#endif
}

const map_index_to_info_t* cISDataMappings::GetIndexMapInfo(uint32_t did)
{
    if (did >= DID_COUNT)
    {
        return NULLPTR;
    }

#if PLATFORM_IS_EMBEDDED

    if (s_map == NULLPTR)
    {
        s_map = new cISDataMappings();
    }

    return &s_map->m_data_set[did].indexInfo;

#else

    return &s_map.m_data_set[did].indexInfo;

#endif
}

// const data_info_t* cISDataMappings::GetFieldDataInfo(uint32_t did, uint32_t field)
// {
//     if (did >= DID_COUNT)
//     {
//         return NULLPTR;
//     }

// #if PLATFORM_IS_EMBEDDED

//     if (s_map == NULLPTR)
//     {
//         s_map = new cISDataMappings();
//     }

//     return s_map->indexInfo[did][field];

// #else

//     return s_map.indexInfo[did][field];

// #endif
// }

uint32_t cISDataMappings::GetSize(uint32_t did)
{
    if (did >= DID_COUNT)
    {
        return 0;
    }

#if PLATFORM_IS_EMBEDDED
    if (s_map == NULLPTR)
    {
        s_map = new cISDataMappings();
    }

    return s_map->m_data_set[did].size;
#else
    return s_map.m_data_set[did].size;
#endif
}

uint32_t cISDataMappings::TotalElementCount(uint32_t did)
{
    if (did >= DID_COUNT)
    {
        return 0;
    }

#if PLATFORM_IS_EMBEDDED
    if (s_map == NULLPTR)
    {
        s_map = new cISDataMappings();
    }

    return s_map->m_data_set[did].totalElementCount;
#else
    return s_map.m_data_set[did].totalElementCount;
#endif
}

uint32_t cISDataMappings::DefaultPeriodMultiple(uint32_t dataId)
{
    switch (dataId)
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
    case DID_IO:
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


bool cISDataMappings::StringToData(const char* stringBuffer, int stringLength, const p_data_hdr_t* hdr, uint8_t* datasetBuffer, const data_info_t& info, int elementIndex, int elementSize, int radix, bool json)
{
    const uint8_t* ptr;
    if (!CanGetFieldData(info, hdr, datasetBuffer, ptr))
    {
        return false;
    }

    return StringToVariable(stringBuffer, stringLength, ptr, info.dataType, info.dataSize, elementIndex, elementSize, radix, json);
}

bool cISDataMappings::StringToVariable(const char* stringBuffer, int stringLength, const uint8_t* dataBuffer, eDataType dataType, uint32_t dataSize, int elementIndex, int elementSize, int radix, bool json)
{
    if (elementIndex)
    {   // Offset pointer into array
        dataBuffer += elementSize * elementIndex;
    }

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
        protectUnalignedAssign<float>((void*)dataBuffer, (float)strtod(stringBuffer, NULL));
        break;

    case DATA_TYPE_F64:
        protectUnalignedAssign<double>((void*)dataBuffer, (double)strtod(stringBuffer, NULL));
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


bool cISDataMappings::DataToString(const data_info_t& info, const p_data_hdr_t* hdr, const uint8_t* datasetBuffer, data_mapping_string_t stringBuffer, int elementIndex, int elementSize, bool json)
{
    const uint8_t* ptr;
    if (!CanGetFieldData(info, hdr, datasetBuffer, ptr))
    {
        // pick a default string
        if (info.dataType == DATA_TYPE_STRING)
        {
            stringBuffer[0] = '"';
            stringBuffer[1] = '"';
            stringBuffer[2] = '\0';
        }
        else if (info.dataType == DATA_TYPE_BINARY)
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

    return VariableToString(info.dataType, info.dataFlags, ptr, datasetBuffer, info.dataSize, stringBuffer, elementIndex, elementSize, json);
}


bool cISDataMappings::VariableToString(eDataType dataType, eDataFlags dataFlags, const uint8_t* ptr, const uint8_t* dataBuffer, uint32_t dataSize, data_mapping_string_t stringBuffer, int elementIndex, int elementSize, bool json)
{
    if (elementIndex)
    {   // Offset pointer into array
        ptr += elementSize * elementIndex;
    }

    int precision;
    switch (dataType)
    {
    case DATA_TYPE_INT8:
        if (dataFlags & DATA_FLAGS_DISPLAY_HEX) SNPRINTF(stringBuffer, IS_DATA_MAPPING_MAX_STRING_LENGTH, "0x%02X", *(int8_t*)ptr);
        else                                    SNPRINTF(stringBuffer, IS_DATA_MAPPING_MAX_STRING_LENGTH, "%d", (int)*(int8_t*)ptr);
        break;
    case DATA_TYPE_UINT8:
        if (dataFlags & DATA_FLAGS_DISPLAY_HEX) SNPRINTF(stringBuffer, IS_DATA_MAPPING_MAX_STRING_LENGTH, "0x%02X", *(uint8_t*)ptr);
        else                                    SNPRINTF(stringBuffer, IS_DATA_MAPPING_MAX_STRING_LENGTH, "%u", (unsigned int)*(uint8_t*)ptr);
        break;
    case DATA_TYPE_INT16:
        if (dataFlags & DATA_FLAGS_DISPLAY_HEX) SNPRINTF(stringBuffer, IS_DATA_MAPPING_MAX_STRING_LENGTH, "0x%04X", *(int16_t*)ptr);
        else                                    SNPRINTF(stringBuffer, IS_DATA_MAPPING_MAX_STRING_LENGTH, "%d", (int)*(int16_t*)ptr);
        break;
    case DATA_TYPE_UINT16:
        if (dataFlags & DATA_FLAGS_DISPLAY_HEX) SNPRINTF(stringBuffer, IS_DATA_MAPPING_MAX_STRING_LENGTH, "0x%04X", *(uint16_t*)ptr);
        else                                    SNPRINTF(stringBuffer, IS_DATA_MAPPING_MAX_STRING_LENGTH, "%u", (unsigned int)*(uint16_t*)ptr);
        break;
    case DATA_TYPE_INT32:
        if (dataFlags & DATA_FLAGS_DISPLAY_HEX) SNPRINTF(stringBuffer, IS_DATA_MAPPING_MAX_STRING_LENGTH, "0x%08X", *(int32_t*)ptr);
        else                                    SNPRINTF(stringBuffer, IS_DATA_MAPPING_MAX_STRING_LENGTH, "%d", (int)*(int32_t*)ptr);
        break;
    case DATA_TYPE_UINT32:
        if (dataFlags & DATA_FLAGS_DISPLAY_HEX) SNPRINTF(stringBuffer, IS_DATA_MAPPING_MAX_STRING_LENGTH, "0x%08X", *(uint32_t*)ptr);
        else                                    SNPRINTF(stringBuffer, IS_DATA_MAPPING_MAX_STRING_LENGTH, "%u", (unsigned int)*(uint32_t*)ptr);        
        break;
    case DATA_TYPE_INT64:
        if (dataFlags & DATA_FLAGS_DISPLAY_HEX) SNPRINTF(stringBuffer, IS_DATA_MAPPING_MAX_STRING_LENGTH, "0x%016llX", (long long)*(uint64_t*)ptr);
        else                                    SNPRINTF(stringBuffer, IS_DATA_MAPPING_MAX_STRING_LENGTH, "%lld", (long long)*(int64_t*)ptr);
        break;
    case DATA_TYPE_UINT64:
        if (dataFlags & DATA_FLAGS_DISPLAY_HEX) SNPRINTF(stringBuffer, IS_DATA_MAPPING_MAX_STRING_LENGTH, "0x%016llX", (unsigned long long)*(uint64_t*)ptr);
        else                                    SNPRINTF(stringBuffer, IS_DATA_MAPPING_MAX_STRING_LENGTH, "%llu", (unsigned long long)*(uint64_t*)ptr);
        break;
    case DATA_TYPE_F32:
        precision = (dataFlags&DATA_FLAGS_FIXED_DECIMAL_MASK);
        if (precision == 0) { precision = 9;  }     // Default to 9 digits
        SNPRINTF(stringBuffer, IS_DATA_MAPPING_MAX_STRING_LENGTH, "%.*f", precision, *(float*)ptr);
        break;
    case DATA_TYPE_F64:                             
        precision = (dataFlags&DATA_FLAGS_FIXED_DECIMAL_MASK);
        if (precision == 0) { precision = 17; }     // Default to 17 digits
        SNPRINTF(stringBuffer, IS_DATA_MAPPING_MAX_STRING_LENGTH, "%.*f", precision, *(double*)ptr);
        break;

    case DATA_TYPE_STRING:
    {
        stringBuffer[0] = '"';
        int tempIndex = 1;
        char* bufPtr2 = (char*)(ptr);
        char* bufPtrEnd = bufPtr2 + _MIN(IS_DATA_MAPPING_MAX_STRING_LENGTH, dataSize) - 3;
        for (; bufPtr2 < bufPtrEnd && *bufPtr2 != '\0'; bufPtr2++)
        {
            if (json)
            {
                if (IS_JSON_ESCAPE_CHAR(*bufPtr2))
                {
                    if (bufPtr2 < bufPtrEnd - 1)
                    {
                        stringBuffer[tempIndex++] = '\\';
                    }
                    else
                    {
                        break;
                    }
                }
            }
            stringBuffer[tempIndex++] = *bufPtr2;
        }
        stringBuffer[tempIndex++] = '"';
        stringBuffer[tempIndex] = '\0';
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

double cISDataMappings::GetTimestamp(const p_data_hdr_t* hdr, const uint8_t* buf)
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

#if PLATFORM_IS_EMBEDDED

    if (s_map == NULLPTR)
    {
        s_map = new cISDataMappings();
    }

#endif

    const data_info_t* timeStampField =
    
#if PLATFORM_IS_EMBEDDED

        s_map->m_data_set[hdr->id].timestampFields;

#else

        s_map.m_data_set[hdr->id].timestampFields;

#endif

    if (timeStampField != NULLPTR)
    {
        const uint8_t* ptr;
        if (CanGetFieldData(*timeStampField, hdr, (uint8_t*)buf, ptr))
        {
            if (timeStampField->dataType == DATA_TYPE_F64)
            {
                // field is seconds, use as is
                return protectUnalignedAssign<double>((void *)ptr);
            }
            else if (timeStampField->dataType == DATA_TYPE_UINT32)
            {
                // field is milliseconds, convert to seconds
                return 0.001 * (*(uint32_t*)ptr);
            }
        }
    }
    return 0.0;
}

bool cISDataMappings::CanGetFieldData(const data_info_t& info, const p_data_hdr_t* hdr, const uint8_t* buf, const uint8_t*& ptr)
{
    if (buf == NULL)
    {
        return false;
    }
    else if (hdr == NULL)
    {
        // assume buf is large enough for the full data structure
        ptr = buf + info.dataOffset;
        return true;
    }
    int32_t fullSize = (hdr->size == 0 ? GetSize(hdr->id) : hdr->size);
    int32_t offset = (int32_t)info.dataOffset - (int32_t)hdr->offset;
    if (offset >= 0 && 
        offset <= (fullSize - (int32_t)info.dataSize))
    {
        ptr = buf + offset;
        return true;
    }
    ptr = NULL;
    return false;
}
