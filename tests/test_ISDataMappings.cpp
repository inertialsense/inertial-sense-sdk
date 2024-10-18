#include <gtest/gtest.h>
#include <deque>
#include "ISDataMappings.h"

using namespace std;


TEST(ISDataMappings, StringToDataToString)
{
	string str1;
	string str2;
	uDatasets d = {};

	const map_name_to_info_t& flashMap = *cISDataMappings::NameToInfo(DID_FLASH_CONFIG);

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

	{	// Floating Point Test
		const data_info_t& info = flashMap.at("gps1AntOffset");
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

