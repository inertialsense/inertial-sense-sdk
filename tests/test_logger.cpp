#include <gtest/gtest.h>
#include "../../../SDK/src/ISLogger.h"
#include "../../../SDK/src/ISDataMappings.h"
#include "../../../SDK/src/ISFileManager.h"
#include "test_data_utils.h"

#if defined(_WIN32)
#define DATA_DIR "../logger_test_data/"
#elif defined(__GNUC__)
#define DATA_DIR "../logger_test_data/"
#endif

#define CLEANUP_TEST_FILES 	1

using namespace std;

static const int s_maxFileSize = 5242880;
//static const int s_maxFileSize = 100000;	// Make many small files
static const float s_maxDiskSpacePercent = 0.5f;
static const float s_maxDiskSpaceMBLarge = 1024.0f * 1024.0f * 10.0f;
static const bool s_useTimestampSubFolder = false;


static dev_info_t CreateDeviceInfo(uint32_t serial)
{
	dev_info_t d = { 0 };
	d.serialNumber = serial;
	return d;
}

static bool LogData(cISLogger& logger, uint32_t device, uint32_t id, uint32_t offset, uint32_t size, void* data)
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
	EXPECT_TRUE(logger2.CopyLog(logger1, cISLogger::g_emptyString, outputPath1, convertLogType, s_maxDiskSpacePercent, s_maxFileSize, s_useTimestampSubFolder, false));
	logger1.CloseAllFiles();
	logger2.CloseAllFiles();

	// convert the intermediate format back to the original format
	cISLogger logger3;
	cISLogger logger4;
	EXPECT_TRUE(logger3.LoadFromDirectory(outputPath1, convertLogType));
	logger3.ShowParseErrors(showParseErrors);			// Allow garbage data tests to hide parse errors
	EXPECT_TRUE(logger4.CopyLog(logger3, cISLogger::g_emptyString, outputPath2, inputLogType, s_maxDiskSpacePercent, s_maxFileSize, s_useTimestampSubFolder, false));
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
	unsigned int devIndex1 = 0;
	unsigned int devIndex2 = 0;
	int dataIndex = -1;

	while (1)
	{
		data1 = finalLogger1.ReadNextData(devIndex1);
		dataIndex++;

		if (data1 != NULL && 
			cISDataMappings::GetSize(data1->hdr.id) == 0 &&
			convertLogType == cISLogger::eLogType::LOGTYPE_CSV)
		{	// CSV logs don't save DIDs not defined in ISDataMapping.  Skip this one.
			continue;
		}

		data2 = finalLogger2.ReadNextData(devIndex2);

		// Compare device indices.  cISLogger::ReadNextData() can increment this.  
		EXPECT_TRUE(devIndex1 == devIndex2);

		if (data1 == NULL || data2 == NULL)
		{	// No more data.  Ensure both logs are empty.
			EXPECT_TRUE(data1 == data2);
			break;
		}

		// Compare DIDs
		if (data1->hdr.id != data2->hdr.id)
		{
			EXPECT_TRUE(data1->hdr.id == data2->hdr.id);
			std::cout << "MISMATCHED DID: " << data1->hdr.id << "," << data2->hdr.id << " size: " << data1->hdr.size << "," << data2->hdr.size << " offset:" << data1->hdr.offset << "," << data2->hdr.offset << " dataIndex: " << dataIndex << std::endl;
			break;
		}

		// Compare Timestamps
		double timestamp1 = cISDataMappings::GetTimestamp(&(data1->hdr), data1->buf);
		double timestamp2 = cISDataMappings::GetTimestamp(&(data2->hdr), data2->buf);
		if (timestamp1 != timestamp2)
		{
			EXPECT_TRUE(timestamp1 == timestamp2);
			std::cout << "MISMATCHED TIMESTAMPS: " << timestamp1 << " " << timestamp1 << " dataIndex: " << dataIndex << std::endl;
			break;
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
			EXPECT_TRUE(data1->hdr.offset == data2->hdr.offset);
			EXPECT_TRUE(data1->hdr.size == data2->hdr.size);
			std::cout << "MISMATCHED size/offset: " << data1->hdr.id << "," << data2->hdr.id << " size: " << data1->hdr.size << "," << data2->hdr.size << " offset:" << data1->hdr.offset << "," << data2->hdr.offset << " dataIndex: " << dataIndex << std::endl;
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
				EXPECT_TRUE(b1 == b2);
				std::cout << "MISMATCHED data (DID " << data1->hdr.id << "," << data2->hdr.id << ") byte index: " << i1 << ", value: " << "'" << (int)b1 << "' != '" << (int)b2 << "'" << std::endl;
				break;
			}
		}
	}

	finalLogger1.CloseAllFiles();
	finalLogger2.CloseAllFiles();

#if CLEANUP_TEST_FILES==1
	// Cleanup test files
	ISFileManager::DeleteDirectory(outputPath1);
	ISFileManager::DeleteDirectory(outputPath2);
#endif

	// Print newline to print test results at start of line
	printf("\n");
}


#if 1	// Enabled

TEST(Logger, dat_conversion)
{
	string logPath = "test_log";
	GenerateLogFiles(3, logPath, cISLogger::eLogType::LOGTYPE_DAT);
	TestConvertLog(logPath, cISLogger::eLogType::LOGTYPE_DAT, cISLogger::eLogType::LOGTYPE_DAT);
	TestConvertLog(logPath, cISLogger::eLogType::LOGTYPE_DAT, cISLogger::eLogType::LOGTYPE_SDAT);
	TestConvertLog(logPath, cISLogger::eLogType::LOGTYPE_DAT, cISLogger::eLogType::LOGTYPE_CSV);
#if CLEANUP_TEST_FILES==1
	ISFileManager::DeleteDirectory(logPath);
#endif
}

TEST(Logger, sdat_conversion)
{
	string logPath = "test_log";
	GenerateLogFiles(1, logPath, cISLogger::eLogType::LOGTYPE_SDAT);
	TestConvertLog(logPath, cISLogger::eLogType::LOGTYPE_SDAT, cISLogger::eLogType::LOGTYPE_DAT);
	TestConvertLog(logPath, cISLogger::eLogType::LOGTYPE_SDAT, cISLogger::eLogType::LOGTYPE_SDAT);
	TestConvertLog(logPath, cISLogger::eLogType::LOGTYPE_SDAT, cISLogger::eLogType::LOGTYPE_CSV);
#if CLEANUP_TEST_FILES==1
	ISFileManager::DeleteDirectory(logPath);
#endif
}

TEST(Logger, raw_conversion)
{
	string logPath = "test_log";
	GenerateLogFiles(3, logPath, cISLogger::eLogType::LOGTYPE_RAW);
	TestConvertLog(logPath, cISLogger::eLogType::LOGTYPE_RAW, cISLogger::eLogType::LOGTYPE_DAT);
	TestConvertLog(logPath, cISLogger::eLogType::LOGTYPE_RAW, cISLogger::eLogType::LOGTYPE_SDAT);
	TestConvertLog(logPath, cISLogger::eLogType::LOGTYPE_RAW, cISLogger::eLogType::LOGTYPE_CSV);
#if CLEANUP_TEST_FILES==1
	ISFileManager::DeleteDirectory(logPath);
#endif
}

TEST(Logger, raw_conversion_with_garbage)
{
	string logPath = "test_log";
	GenerateLogFiles(1, logPath, cISLogger::eLogType::LOGTYPE_RAW, 20, GEN_LOG_OPTIONS_INSERT_GARBAGE_BETWEEN_MSGS);
	TestConvertLog(logPath, cISLogger::eLogType::LOGTYPE_RAW, cISLogger::eLogType::LOGTYPE_DAT, false);
#if CLEANUP_TEST_FILES==1
	ISFileManager::DeleteDirectory(logPath);
#endif
}

// TEST(Logger, dat_conversion_with_multiple_files_issue_Aug_2017)
// {
// 	TestConvertLog(DATA_DIR"logger_dat3", cISLogger::eLogType::LOGTYPE_DAT, cISLogger::eLogType::LOGTYPE_DAT);
// 	TestConvertLog(DATA_DIR"logger_dat3", cISLogger::eLogType::LOGTYPE_DAT, cISLogger::eLogType::LOGTYPE_SDAT);
// 	TestConvertLog(DATA_DIR"logger_dat3", cISLogger::eLogType::LOGTYPE_DAT, cISLogger::eLogType::LOGTYPE_CSV);
// }

#else	// Disabled

#pragma message("-------------------------------------------------------------------------------------------")
#pragma message("WARNING!!! test_logger.cpp tests have been temporarily disabled.  Please re-enable them!!!")
#pragma message("-------------------------------------------------------------------------------------------")

#endif