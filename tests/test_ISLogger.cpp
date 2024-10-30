#include <gtest/gtest.h>
#include "ISLogger.h"
#include "ISDataMappings.h"
#include "ISFileManager.h"
#include "test_data_utils.h"

#if 1
#define DELETE_DIRECTORY(d)		ISFileManager::DeleteDirectory(d)
#else
#define DELETE_DIRECTORY(d)		// Leave test data in place for inspection
#endif

using namespace std;

static const int s_maxFileSize = DEFAULT_LOGS_MAX_FILE_SIZE;
//static const int s_maxFileSize = 100000;	// Make many small files
static const float s_logDiskUsageLimitPercent = 0.5f;
static const bool s_useTimestampSubFolder = false;


static dev_info_t CreateDeviceInfo(uint32_t serial)
{
	dev_info_t d = { 0 };
	d.serialNumber = serial;
	return d;
}

static bool LogData(cISLogger& logger, std::shared_ptr<cDeviceLog> device, uint32_t id, uint32_t offset, uint32_t size, void* data)
{
	p_data_hdr_t hdr;
	hdr.id = id;
	hdr.offset = offset;
	hdr.size = size;
	return logger.LogData(device, &hdr, (uint8_t*)data);
}

static void TestConvertLog(string inputPath, cISLogger::eLogType inputLogType, cISLogger::eLogType convertLogType, bool showParseErrors=true)
{
	vector<ISFileManager::file_info_t> originalFiles;
	vector<ISFileManager::file_info_t> outputFiles;

	string outputPath1 = inputPath+"_test1";
	string outputPath2 = inputPath+"_test2";
	// Clean up old files
	ISFileManager::DeleteDirectory(outputPath1);
	ISFileManager::DeleteDirectory(outputPath2);

	// first convert the log to the intermediate format
	cISLogger logger1;
	cISLogger logger2;
	EXPECT_TRUE(logger1.LoadFromDirectory(inputPath, inputLogType));
	logger1.ShowParseErrors(showParseErrors);			// Allow garbage data tests to hide parse errors
	EXPECT_TRUE(logger2.CopyLog(logger1, cISLogger::g_emptyString, outputPath1, convertLogType, s_maxFileSize, s_logDiskUsageLimitPercent, s_useTimestampSubFolder, false));
	logger1.CloseAllFiles();
	logger2.CloseAllFiles();

	// convert the intermediate format back to the original format
	cISLogger logger3;
	cISLogger logger4;
	EXPECT_TRUE(logger3.LoadFromDirectory(outputPath1, convertLogType));
	logger3.ShowParseErrors(showParseErrors);			// Allow garbage data tests to hide parse errors
	EXPECT_TRUE(logger4.CopyLog(logger3, cISLogger::g_emptyString, outputPath2, inputLogType, s_maxFileSize, s_logDiskUsageLimitPercent, s_useTimestampSubFolder, false));
	logger3.CloseAllFiles();
	logger4.CloseAllFiles();

	// loop through the original log and final log and ensure all entries match
	cISLogger finalLogger1;
	cISLogger finalLogger2;
	finalLogger1.LoadFromDirectory(inputPath, inputLogType);
	finalLogger2.LoadFromDirectory(outputPath1, convertLogType);
	finalLogger1.ShowParseErrors(showParseErrors);		// Allow garbage data tests to hide parse errors
	finalLogger2.ShowParseErrors(showParseErrors);		// Allow garbage data tests to hide parse errors
	p_data_buf_t* data1;
	p_data_buf_t* data2;
	size_t devIndex1 = 0;
	size_t devIndex2 = 0;
	int dataIndex = -1;

	while (1)
	{
		data1 = finalLogger1.ReadNextData(devIndex1);
		dataIndex++;

		if (data1 != NULL && 
			cISDataMappings::DataSize(data1->hdr.id) == 0 &&
			convertLogType == cISLogger::eLogType::LOGTYPE_CSV)
		{	// CSV logs don't save DIDs not defined in ISDataMapping.  Skip this one.
			continue;
		}

		data2 = finalLogger2.ReadNextData(devIndex2);

		// Compare device indices.  cISLogger::ReadNextData() can increment this.  
		EXPECT_TRUE(devIndex1 == devIndex2);

		if (data1 == NULL || data2 == NULL)
		{	// No more data.  Ensure both logs are empty.
			EXPECT_EQ(data1, data2);
			break;
		}

		// Compare DIDs
		if (data1->hdr.id != data2->hdr.id)
		{
			EXPECT_EQ(data1->hdr.id, data2->hdr.id) << "MISMATCHED DID: " << (int)(data1->hdr.id) << "," << (int)(data2->hdr.id) << " size: " << data1->hdr.size << "," << data2->hdr.size << " offset:" << data1->hdr.offset << "," << data2->hdr.offset << " dataIndex: " << dataIndex << std::endl;
			// break;
		}

		// Compare Timestamps
		double timestamp1 = cISDataMappings::Timestamp(&(data1->hdr), data1->buf);
		double timestamp2 = cISDataMappings::Timestamp(&(data2->hdr), data2->buf);
		if (timestamp1 != timestamp2)
		{
			EXPECT_DOUBLE_EQ(timestamp1, timestamp2) << "MISMATCHED TIMESTAMPS: " << timestamp1 << " " << timestamp1 << " dataIndex: " << dataIndex << std::endl;
			// break;
		}

		// Debug
		//printf("DEBUG %d  (DID, time) data1: %2d, %.4lf  data2:  %2d, %.4lf\n", dataIndex, data1->hdr.id, timestamp1, data2->hdr.id, timestamp2);

		if (inputLogType == cISLogger::eLogType::LOGTYPE_CSV ||
			convertLogType == cISLogger::eLogType::LOGTYPE_CSV)
		{	// Exclude CSV logs from byte comparison as float rounding causes mismatch
			continue;
		}

		uint32_t i1 = 0;
		uint32_t i2 = 0;

		// Compare size and offset
		if ((data1->hdr.size != data2->hdr.size) ||
			(data1->hdr.offset != data2->hdr.offset))
		{
			EXPECT_EQ(data1->hdr.offset, data2->hdr.offset) << "MISMATCHED offset: " << (int)data1->hdr.id << "," << (int)data2->hdr.id << " offset:" << data1->hdr.offset << "," << data2->hdr.offset << " dataIndex: " << dataIndex << std::endl;
			EXPECT_EQ(data1->hdr.size, data2->hdr.size) << "MISMATCHED size: " << (int)data1->hdr.id << "," << (int)data2->hdr.id << " size: " << data1->hdr.size << "," << data2->hdr.size << " dataIndex: " << dataIndex << std::endl;
			break;
		}

		// Compare byte-for-byte content
		uint32_t end = _MIN(data1->hdr.size, data2->hdr.size);
		for (; i1 < end; i1++, i2++)
		{
			uint8_t b1 = data1->buf[i1];
			uint8_t b2 = data2->buf[i2];
			if (b1 != b2)
			{
				EXPECT_EQ(b1, b2) << "MISMATCHED data (DID " << (int)data1->hdr.id << "," << (int)data2->hdr.id << ") byte index: " << i1 << ", value: " << "'" << (int)b1 << "' != '" << (int)b2 << "'" << std::endl;
				break;
			}
		}
	}

	finalLogger1.CloseAllFiles();
	finalLogger2.CloseAllFiles();

	// Cleanup test files
	DELETE_DIRECTORY(outputPath1);
	DELETE_DIRECTORY(outputPath2);

	// Print newline to print test results at start of line
	printf("\n");
}

void TestParseFileName(string filename, int rSerialNum, string rDate, string rTime, int rIndex, bool rResults=true)
{
	int serialNum, index; 
	string date, time;
	bool results = cISLogger::ParseFilename(filename, serialNum, date, time, index);
	EXPECT_EQ(results, rResults);
	EXPECT_EQ(serialNum, rSerialNum);
	EXPECT_EQ(date, rDate);
	EXPECT_EQ(time, rTime);
	EXPECT_EQ(index, rIndex);
}


#if 1	// Enabled

TEST(ISLogger, parse_filename)
{
	TestParseFileName("base_station.raw", 0, "", "", -1, false);
	TestParseFileName("LOG_SN60339_20240311_132545_0000.RAW", 60339, "20240311", "132545", 0);
	TestParseFileName("LOG_SN60339_20240311_132545_0001.RAW", 60339, "20240311", "132545", 1);
	TestParseFileName("LOG_SN60339_20240311_132545_0002.raw", 60339, "20240311", "132545", 2);
	TestParseFileName("LOG_SN60339123_20240311_132545_0992.raw", 60339123, "20240311", "132545", 992);
	TestParseFileName("LOG_SN60339_20240311_214365.raw", 60339, "20240311", "214365", -1, false);
	TestParseFileName("LOG_SN60339_20240311_0007.raw", 60339, "20240311", "0007", -1, false);
	TestParseFileName("LOG_SN60339_20240311.raw", 60339, "20240311", "", -1, false);
	TestParseFileName("LOG_SN60339_.raw", 60339, "", "", -1, false);
	TestParseFileName("LOG_SN60339.raw", 60339, "", "", -1, false);
	TestParseFileName("00000000.RAW", 0, "", "", 0);
	TestParseFileName("00000001.RAW", 0, "", "", 1);
	TestParseFileName("00000002.raw", 0, "", "", 2);
	TestParseFileName("12345678.RAW", 0, "", "", 12345678);
}

TEST(ISLogger, dat_conversion)
{
	string logPath = "test_log";
	GenerateDataLogFiles(3, logPath, cISLogger::eLogType::LOGTYPE_DAT);
	TestConvertLog(logPath, cISLogger::eLogType::LOGTYPE_DAT, cISLogger::eLogType::LOGTYPE_RAW);
	TestConvertLog(logPath, cISLogger::eLogType::LOGTYPE_DAT, cISLogger::eLogType::LOGTYPE_CSV);
	DELETE_DIRECTORY(logPath);
}

TEST(ISLogger, raw_conversion)
{
	string logPath = "test_log";
	GenerateDataLogFiles(3, logPath, cISLogger::eLogType::LOGTYPE_RAW);
	TestConvertLog(logPath, cISLogger::eLogType::LOGTYPE_RAW, cISLogger::eLogType::LOGTYPE_DAT);
	TestConvertLog(logPath, cISLogger::eLogType::LOGTYPE_RAW, cISLogger::eLogType::LOGTYPE_CSV);
	DELETE_DIRECTORY(logPath);
}

TEST(ISLogger, raw_conversion_with_garbage)
{
	string logPath = "test_log";
	GenerateDataLogFiles(1, logPath, cISLogger::eLogType::LOGTYPE_RAW, 20, GEN_LOG_OPTIONS_INSERT_GARBAGE_BETWEEN_MSGS);
	TestConvertLog(logPath, cISLogger::eLogType::LOGTYPE_RAW, cISLogger::eLogType::LOGTYPE_DAT, false);
	DELETE_DIRECTORY(logPath);
}

#else	// Disabled

#pragma message("-------------------------------------------------------------------------------------------")
#pragma message("WARNING!!! test_ISLogger.cpp tests have been temporarily disabled.  Please re-enable them!!!")
#pragma message("-------------------------------------------------------------------------------------------")

#endif