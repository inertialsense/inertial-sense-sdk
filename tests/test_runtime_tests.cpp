#include <gtest/gtest.h>
#include "ISDataMappings.h"
#include "ISFileManager.h"
#include "ISLogger.h"
#include "test_data_utils.h"

using namespace std;



TEST(runtime_tests, bad_nmea)
{
	string logPath = "test_log";
	GenerateLogFiles(3, logPath, cISLogger::eLogType::LOGTYPE_RAW, 0);
}

