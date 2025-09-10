#include <gtest/gtest.h>
#include <deque>
#include "ISDataMappings.h"

using namespace std;


TEST(ISDataMappings, StringToDataToString)
{
	string str1;
	string str2;
	uDatasets d = {};

	const map_name_to_info_t& flashMap = *cISDataMappings::NameToInfoMap(DID_FLASH_CONFIG);

	{	// Integer Test
		const data_info_t& info = flashMap.at("ser0BaudRate");
		int baudrate = 921600;
		str1 = to_string(baudrate);

		// Integer - string to data
		cISDataMappings::StringToData(str1.c_str(), (int)str1.size(), NULL, (uint8_t*)&d, info, 0);

		EXPECT_EQ(d.flashCfg.ser0BaudRate, baudrate);

		// Integer - data to string
		data_mapping_string_t stringBuffer;
		cISDataMappings::DataToString(info, NULL, (uint8_t*)&d, stringBuffer);

		str2 = string(stringBuffer);
		EXPECT_EQ(str1, str2);
	}

#if 0	// Print keys for flashMap
	for (const auto& [key, value] : flashMap) 
	{
        std::cout << key << std::endl;
    }
#endif

	{	// Floating Point Test
		string key = "gps1AntOffset";
		if (flashMap.find(key) == flashMap.end())
		{	// Key not present.  Include brackets.  In ISDataMappings, we use both multi-element single-variables and single-element multi-variables to represent arrays.
			key += "[0]";
		}
		const data_info_t& info = flashMap.at(key);
		float gps1AntOffset1 = 1.234f;
		str1 = to_string(gps1AntOffset1);

		// float - string to data
		cISDataMappings::StringToData(str1.c_str(), (int)str1.size(), NULL, (uint8_t*)&d, info, 1);

		EXPECT_EQ(d.flashCfg.gps1AntOffset[1], gps1AntOffset1);

		// float - data to string
		data_mapping_string_t stringBuffer;
		cISDataMappings::DataToString(info, NULL, (uint8_t*)&d, stringBuffer, 1);

		str2 = string(stringBuffer);
		float f1 = std::stof(str1);
		float f2 = std::stof(str2);
		EXPECT_TRUE( fabs(f1-f2) < 0.0000001f);
	}
}


void PrintYamlNode(const YAML::Node& node)
{
    std::cout << node << std::endl;
}


void testDataToYamlToData(const uDatasets& d1, uint32_t did)
{
	// Run the conversion from data to YAML and back to data twice to ensure small rounding does not affect the result.
	YAML::Node yaml1 = YAML::Node();
	EXPECT_TRUE(cISDataMappings::DataToYaml(did, (uint8_t*)&d1, yaml1));
	uDatasets d2 = {};
	EXPECT_TRUE(cISDataMappings::YamlToData(did, yaml1, (uint8_t*)&d2));
	YAML::Node yaml2 = YAML::Node();
	EXPECT_TRUE(cISDataMappings::DataToYaml(did, (uint8_t*)&d2, yaml2));
	uDatasets result = {};
	EXPECT_TRUE(cISDataMappings::YamlToData(did, yaml2, (uint8_t*)&result));

	// PrintYamlNode(yaml1);

	EXPECT_EQ(0, memcmp(&d2, &result, cISDataMappings::DataSize(did)));
}

TEST(ISDataMappings, DataToYamlToData)
{
	uDatasets d = {};

	d.ins1.timeOfWeek = 123456.789;
	d.ins1.lla[0] = 37.7749; // Latitude
	d.ins1.lla[1] = -122.4194; // Longitude
	d.ins1.lla[2] = 30.0; // Height above ellipsoid
	d.ins1.uvw[0] = 1.0; // U velocity
	d.ins1.uvw[1] = 2.0; // V velocity
	d.ins1.uvw[2] = 3.0; // W velocity
	d.ins1.theta[0] = 0.1; // Roll
	d.ins1.theta[1] = 0.2; // Pitch
	d.ins1.theta[2] = 0.3; // Yaw
	d.ins1.week = 1234; // GPS week
	d.ins1.insStatus = 0x01020304; // INS status flags
	d.ins1.hdwStatus = 0x05060708; // Hardware status flags
	testDataToYamlToData(d, DID_INS_1);

	d.sysParams.insStatus = 0x01020304; // INS status flags
	d.sysParams.hdwStatus = 0x05060708; // Hardware status flags
	d.sysParams.imuTemp = 25.0f; // IMU temperature in degrees
	d.sysParams.baroTemp = 20.0f; // Barometer temperature in degrees
	d.sysParams.mcuTemp = 30.0f; // MCU temperature in degrees
	d.sysParams.sysStatus = 0x090A0B0C; // System status
	d.sysParams.imuSamplePeriodMs = 50; // IMU sample period in milliseconds
	d.sysParams.navOutputPeriodMs = 100; // Navigation output period in milliseconds
	d.sysParams.sensorTruePeriod = 0.001; // Sensor true period in seconds
	d.sysParams.flashCfgChecksum = 0x12345678; // Flash config checksum
	d.sysParams.navUpdatePeriodMs = 100; // Navigation update period in milliseconds
	d.sysParams.genFaultCode = 0x0F0E0D0C; // General fault code
	d.sysParams.upTime = 3600.0; // System up time in seconds
	testDataToYamlToData(d, DID_SYS_PARAMS);

	d.flashCfg.startupImuDtMs = 100;
	d.flashCfg.startupNavDtMs = 200;
	d.flashCfg.ser0BaudRate = 115200;
	d.flashCfg.ser1BaudRate = 230400;
	d.flashCfg.insRotation[0] = 0.01f; // Rotation about X
	d.flashCfg.insRotation[1] = 0.02f; // Rotation about Y
	d.flashCfg.insRotation[2] = 0.03f; // Rotation about Z
	d.flashCfg.insOffset[0] = 0.1f; // X offset
	d.flashCfg.insOffset[1] = 0.2f; // Y offset
	d.flashCfg.insOffset[2] = 0.3f; // Z offset
	d.flashCfg.gps1AntOffset[0] = 0.01f; // X antenna offset
	d.flashCfg.gps1AntOffset[1] = 0.02f; // Y antenna offset
	d.flashCfg.gps1AntOffset[2] = 0.03f; // Z antenna offset
	d.flashCfg.dynamicModel = 4; // Ground vehicle
	d.flashCfg.debug = 1; // Debug enabled
	d.flashCfg.gnssSatSigConst = 0x0003; // GPS constellation
	d.flashCfg.sysCfgBits = 0x00000001; // System configuration bits
	d.flashCfg.refLla[0] = 37.7749; // Reference latitude
	d.flashCfg.refLla[1] = -122.4194; // Reference longitude
	d.flashCfg.refLla[2] = 30.0; // Reference height above ellipsoid
	d.flashCfg.lastLla[0] = 37.7749; // Last latitude
	d.flashCfg.lastLla[1] = -122.4194; // Last longitude
	d.flashCfg.lastLla[2] = 30.0; // Last height above ellipsoid
	d.flashCfg.lastLlaTimeOfWeekMs = 123456; // Last LLA time of week in milliseconds
	d.flashCfg.lastLlaWeek = 123; // Last LLA GPS week
	d.flashCfg.lastLlaUpdateDistance = 100.0f; // Last LLA update distance
	d.flashCfg.ioConfig = 0x00000001; // IO configuration bits
	d.flashCfg.platformConfig = 0x00000002; // Platform configuration bits
	testDataToYamlToData(d, DID_FLASH_CONFIG);
}


TEST(ISDataMappings, MemoryUsageTracking)
{
	std::vector<cISDataMappings::MemoryUsage> usage;

	uDatasets d = {};

	// size: 1, adds new
	cISDataMappings::AppendMemoryUsage(usage, &d.flashCfg.ser2BaudRate, sizeof(d.flashCfg.ser2BaudRate));
	EXPECT_EQ(usage.size(), 1);

	// size: 1, adjacent to ser2BaudRate, should merge with existing
	cISDataMappings::AppendMemoryUsage(usage, &d.flashCfg.gpsMinimumElevation, sizeof(d.flashCfg.gpsMinimumElevation));		
	EXPECT_EQ(usage.size(), 1);

	// size: 2, adds new
	cISDataMappings::AppendMemoryUsage(usage, &d.flashCfg.insRotation[0], sizeof(d.flashCfg.insRotation[0]));
	EXPECT_EQ(usage.size(), 2);

	// size: 3, adds new
	cISDataMappings::AppendMemoryUsage(usage, &d.flashCfg.insRotation[2], sizeof(d.flashCfg.insRotation[2]));
	EXPECT_EQ(usage.size(), 3);

	// size: 2, adjacent to insRotation[0] and insRotation[2], should cause two to merge
	cISDataMappings::AppendMemoryUsage(usage, &d.flashCfg.insRotation[1], sizeof(d.flashCfg.insRotation[1]));   
	EXPECT_EQ(usage.size(), 2);

	// size 2, adjacent to ser2BaudRate, should merge with existing
	cISDataMappings::AppendMemoryUsage(usage, &d.flashCfg.ser1BaudRate, sizeof(d.flashCfg.ser1BaudRate));		
	EXPECT_EQ(usage.size(), 2);

	EXPECT_EQ((void*)usage[0].ptr, (void*)&d.flashCfg.gpsMinimumElevation);
	EXPECT_EQ(usage[0].size, sizeof(d.flashCfg.gpsMinimumElevation) + sizeof(d.flashCfg.ser2BaudRate));

	EXPECT_EQ((void*)usage[1].ptr, (void*)&d.flashCfg.ser1BaudRate);
	EXPECT_EQ(usage[1].size, sizeof(d.flashCfg.ser1BaudRate) + 3*sizeof(d.flashCfg.insRotation[0]));
}


void PrintUsageVec(const std::vector<cISDataMappings::MemoryUsage>& usageVec) {
	std::cout << "Memory Usage size: " <<  usageVec.size() << std::endl;
    for (size_t i = 0; i < usageVec.size(); ++i) {
        const auto& usage = usageVec[i];
        std::cout << "usageVec[" << i << "]: ptr = "
                  << static_cast<const void*>(usage.ptr)
                  << ", size = " << usage.size
                  << ", end = " << static_cast<const void*>(usage.end())
                  << std::endl;
    }
}


TEST(ISDataMappings, YamlToDataMemoryUsage)
{
    YAML::Node root;
	uDatasets d = {};
	std::vector<cISDataMappings::MemoryUsage> usageVec;

	YAML::Node cfg = YAML::Node(YAML::NodeType::Map);
    cfg["gpsMinimumElevation"] = 0.1;
	cfg["insRotation"] = YAML::Load("[0.1, 0.2, 0.3]");
    cfg["ser2BaudRate"] = 9600;
    cfg["ser1BaudRate"] = 115200;
    root["DID_FLASH_CONFIG"] = cfg;

	YAML::Node sys = YAML::Node(YAML::NodeType::Map);
	sys["timeOfWeekMs"] = 123456;
    sys["sysStatus"] = 0x00000004;
	sys["insStatus"] = 0x01020304;
    sys["hdwStatus"] = 0x05060708;
    sys["navUpdatePeriodMs"] = 5;
	sys["genFaultCode"] = 0;
    root["DID_SYS_PARAMS"] = sys;
	// PrintYamlNode(root);

	// Convert YAML to data.  This clears usageVec.
	EXPECT_TRUE(cISDataMappings::YamlToData(DID_FLASH_CONFIG, root, (uint8_t*)&d, &usageVec));
	// PrintUsageVec(usageVec);
	EXPECT_EQ(usageVec.size(), 2);
	// Group 1:	
	EXPECT_EQ(usageVec[0].ptr, reinterpret_cast<uint8_t*>(&d.flashCfg.gpsMinimumElevation));
	EXPECT_EQ(usageVec[0].size, sizeof(d.flashCfg.gpsMinimumElevation) + sizeof(d.flashCfg.ser2BaudRate));
	// Group 2:
	EXPECT_EQ(usageVec[1].ptr, reinterpret_cast<uint8_t*>(&d.flashCfg.ser1BaudRate));
	EXPECT_EQ(usageVec[1].size, sizeof(d.flashCfg.ser1BaudRate) + 3 * sizeof(d.flashCfg.insRotation[0]));

	// Convert YAML to data.  This clears usageVec.
	EXPECT_TRUE(cISDataMappings::YamlToData(DID_SYS_PARAMS, root, (uint8_t*)&d, &usageVec));
	// PrintUsageVec(usageVec);
	EXPECT_EQ(usageVec.size(), 3);
	// Group 1:	
	EXPECT_EQ(usageVec[0].ptr, reinterpret_cast<uint8_t*>(&d.sysParams.timeOfWeekMs));
	EXPECT_EQ(usageVec[0].size, sizeof(d.sysParams.timeOfWeekMs) + sizeof(d.sysParams.insStatus) + sizeof(d.sysParams.hdwStatus));
	// Group 2:
	EXPECT_EQ(usageVec[1].ptr, reinterpret_cast<uint8_t*>(&d.sysParams.sysStatus));
	EXPECT_EQ(usageVec[1].size, sizeof(d.sysParams.sysStatus));
	// Group 3:
	EXPECT_EQ(usageVec[2].ptr, reinterpret_cast<uint8_t*>(&d.sysParams.navUpdatePeriodMs));
	EXPECT_EQ(usageVec[2].size, sizeof(d.sysParams.navUpdatePeriodMs) + sizeof(d.sysParams.genFaultCode));
}

void testDidBufToStringToDidBuf(const uDatasets& d1, uint32_t did)
{
	std::string str1;
	EXPECT_TRUE(cISDataMappings::DidBufferToString(did, (uint8_t*)&d1, str1));
	uDatasets d2 = {};
	EXPECT_TRUE(cISDataMappings::StringToDidBuffer(did, str1, (uint8_t*)&d2));
	std::string str2;
	EXPECT_TRUE(cISDataMappings::DidBufferToString(did, (uint8_t*)&d2, str2));
	uDatasets result = {};
	EXPECT_TRUE(cISDataMappings::StringToDidBuffer(did, str2, (uint8_t*)&result));

	EXPECT_EQ(0, memcmp(&d2, &result, cISDataMappings::DataSize(did)));
}

TEST(ISDataMappings, DidBufferToStringToDidBuffer)
{
	uDatasets d = {};

	d.ins1.timeOfWeek = 123456.789;
	d.ins1.lla[0] = 37.7749; // Latitude
	d.ins1.lla[1] = -122.4194; // Longitude
	d.ins1.lla[2] = 30.0; // Height above ellipsoid
	d.ins1.uvw[0] = 1.0; // U velocity
	d.ins1.uvw[1] = 2.0; // V velocity
	d.ins1.uvw[2] = 3.0; // W velocity
	d.ins1.theta[0] = 0.1; // Roll
	d.ins1.theta[1] = 0.2; // Pitch
	d.ins1.theta[2] = 0.3; // Yaw
	d.ins1.week = 1234; // GPS week
	d.ins1.insStatus = 0x01020304; // INS status flags
	d.ins1.hdwStatus = 0x05060708; // Hardware status flags
	testDidBufToStringToDidBuf(d, DID_INS_1);

	d.sysParams.navOutputPeriodMs = 100; // Navigation output period in milliseconds
	d.sysParams.insStatus = 0x01020304; // INS status flags
	d.sysParams.hdwStatus = 0x05060708; // Hardware status flags
	d.sysParams.imuTemp = 25.0f; // IMU temperature in degrees
	d.sysParams.baroTemp = 20.0f; // Barometer temperature in degrees
	d.sysParams.mcuTemp = 30.0f; // MCU temperature in degrees
	d.sysParams.sysStatus = 0x090A0B0C; // System status
	d.sysParams.imuSamplePeriodMs = 50; // IMU sample period in milliseconds
	d.sysParams.navOutputPeriodMs = 100; // Navigation output period in milliseconds
	d.sysParams.sensorTruePeriod = 0.001; // Sensor true period in seconds
	d.sysParams.flashCfgChecksum = 0x12345678; // Flash config checksum
	d.sysParams.navUpdatePeriodMs = 200; // Navigation update period in milliseconds
	d.sysParams.genFaultCode = 0x0F0E0D0C; // General fault code
	d.sysParams.upTime = 3600.0; // System up time in seconds
	testDidBufToStringToDidBuf(d, DID_SYS_PARAMS);

	d.flashCfg.startupImuDtMs = 100;
	d.flashCfg.startupNavDtMs = 200;
	d.flashCfg.ser0BaudRate = 115200;
	d.flashCfg.ser1BaudRate = 230400;
	d.flashCfg.insRotation[0] = 0.01f; // Rotation about X
	d.flashCfg.insRotation[1] = 0.02f; // Rotation about Y
	d.flashCfg.insRotation[2] = 0.03f; // Rotation about Z
	d.flashCfg.insOffset[0] = 0.1f; // X offset
	d.flashCfg.insOffset[1] = 0.2f; // Y offset
	d.flashCfg.insOffset[2] = 0.3f; // Z offset
	d.flashCfg.gps1AntOffset[0] = 0.01f; // X antenna offset
	d.flashCfg.gps1AntOffset[1] = 0.02f; // Y antenna offset
	d.flashCfg.gps1AntOffset[2] = 0.03f; // Z antenna offset
	d.flashCfg.dynamicModel = 4; // Ground vehicle
	d.flashCfg.debug = 1; // Debug enabled
	d.flashCfg.gnssSatSigConst = 0x0003; // GPS constellation
	d.flashCfg.sysCfgBits = 0x00000001; // System configuration bits
	d.flashCfg.refLla[0] = 37.7749; // Reference latitude
	d.flashCfg.refLla[1] = -122.4194; // Reference longitude
	d.flashCfg.refLla[2] = 30.0; // Reference height above ellipsoid
	d.flashCfg.lastLla[0] = 37.7749; // Last latitude
	d.flashCfg.lastLla[1] = -122.4194; // Last longitude
	d.flashCfg.lastLla[2] = 30.0; // Last height above ellipsoid
	d.flashCfg.lastLlaTimeOfWeekMs = 123456; // Last LLA time of week in milliseconds
	d.flashCfg.lastLlaWeek = 123; // Last LLA GPS week
	d.flashCfg.lastLlaUpdateDistance = 100.0f; // Last LLA update distance
	d.flashCfg.ioConfig = 0x00000001; // IO configuration bits
	d.flashCfg.platformConfig = 0x00000002; // Platform configuration bits
	testDidBufToStringToDidBuf(d, DID_FLASH_CONFIG);
}