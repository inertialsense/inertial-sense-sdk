#include <gtest/gtest.h>
#include "../ISLogger.h"
#include "../ISDataMappings.h"
#include "../ISFileManager.h"

static const string outputPath1 = "./tests/data/temp";
static const string outputPath2 = "./tests/data/temp2";
static const string outputPath3 = "./tests/data/temp3";
static const string outputPath4 = "./tests/data/temp4";
static const int maxFileSize = 5242880;
static const float maxDiskSpacePercent = 0.5f;
static const float maxDiskSpaceMBLarge = 1024.0f * 1024.0f * 10.0f;
static const bool useTimestampSubFolder = false;

class cTempFileCleaner
{
public:
	cTempFileCleaner()
	{
		CleanupTempFiles();
	}

	~cTempFileCleaner()
	{
		CleanupTempFiles();
	}

	static void CleanupTempFiles()
	{
		ISFileManager::DeleteDirectory(outputPath1);
		ISFileManager::DeleteDirectory(outputPath2);
		ISFileManager::DeleteDirectory(outputPath3);
		ISFileManager::DeleteDirectory(outputPath4);
	}
};

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

static void TestSdatSerials()
{
	cTempFileCleaner cleanup;
	cISLogger logger;
	logger.InitSave(cISLogger::eLogType::LOGTYPE_SDAT, outputPath1, 3, maxDiskSpacePercent, maxFileSize, useTimestampSubFolder);
	logger.EnableLogging(true);
	dev_info_t dev1 = CreateDeviceInfo(1);
	dev_info_t dev2 = CreateDeviceInfo(2);
	dev_info_t dev3 = CreateDeviceInfo(2);
	EXPECT_TRUE(LogData(logger, 0, DID_DEV_INFO, 0, sizeof(dev_info_t), &dev1));
	EXPECT_TRUE(LogData(logger, 0, DID_DEV_INFO, 0, sizeof(dev_info_t), &dev2));
	EXPECT_TRUE(LogData(logger, 0, DID_DEV_INFO, 0, sizeof(dev_info_t), &dev3));
	const cLogStats& stats = logger.GetStats();
	EXPECT_TRUE(stats.count == 3);
	EXPECT_TRUE(stats.dataIdStats[DID_DEV_INFO].count == 3);
}

static void TestConvertLog(string inputPath, cISLogger::eLogType srcLogType, cISLogger::eLogType convertLogType, bool compareOffsetAndSize = true)
{
	vector<ISFileManager::file_info_t> originalFiles;
	vector<ISFileManager::file_info_t> outputFiles;
	cTempFileCleaner cleanup;

	// first convert the log to the intermediate format
	cISLogger logger;
	EXPECT_TRUE(logger.LoadFromDirectory(inputPath, srcLogType));
	cISLogger logger2;
	EXPECT_TRUE(logger2.CopyLog(logger, cISLogger::g_emptyString, outputPath1, convertLogType, maxDiskSpacePercent, maxFileSize, useTimestampSubFolder));

	// convert the intermediate format back to the original format
	cISLogger logger3;
	EXPECT_TRUE(logger3.LoadFromDirectory(outputPath1, convertLogType));
	cISLogger logger4;
	EXPECT_TRUE(logger4.CopyLog(logger3, cISLogger::g_emptyString, outputPath2, srcLogType, maxDiskSpacePercent, maxFileSize, useTimestampSubFolder));

	// loop through the original log and final log and ensure all entries match
	cISLogger finalLogger1;
	finalLogger1.LoadFromDirectory(inputPath, srcLogType);
	cISLogger finalLogger2;
	finalLogger2.LoadFromDirectory(outputPath2, srcLogType);
	p_data_t* data;
	p_data_t* data2;
	unsigned int devIndex1 = 0;
	unsigned int devIndex2 = 0;
	while (1)
	{
		data = finalLogger1.ReadNextData(devIndex1);
		if (data == NULL)
		{
			data = finalLogger2.ReadNextData(devIndex2);

			// the other file had also better end
			EXPECT_TRUE(data == NULL);
			break;
		}
		else
		{
			uint32_t i = 0;
			uint32_t i2;
			data2 = finalLogger2.ReadNextData(devIndex2);
			EXPECT_TRUE(data2 != NULL);
			if (compareOffsetAndSize)
			{
				EXPECT_TRUE(data->hdr.id == data2->hdr.id);
				EXPECT_TRUE(data->hdr.offset == data2->hdr.offset);
				i2 = 0;
			}
			else
			{
				// data2 lost the offset and size and has the entire packet, so we need to offset into it from data offset
				EXPECT_TRUE(data2->hdr.offset == 0);
				EXPECT_TRUE(data2->hdr.size == cISDataMappings::GetSize(data2->hdr.id));
				i2 = data->hdr.offset;
			}
			uint32_t end = _MIN(data->hdr.size, data2->hdr.size);
			for (; i < end; i++, i2++)
			{
				uint8_t b1 = data->buf[i];
				uint8_t b2 = data2->buf[i2];
				if (b1 != b2)
				{
					std::cout << "Mismatching data from log at index " << i << ", id1: " << data->hdr.id << ", id2: " << data2->hdr.id << ", " << "'" << (int)b1 << "' != '" << (int)b2 << "'";
				}
			}
		}
	}
}

TEST(Logger, sdat_serials)
{
	TestSdatSerials();
}

TEST(Logger, reading_corrupt_data)
{
	TestConvertLog("./tests/data/logger_corrupt", cISLogger::eLogType::LOGTYPE_DAT, cISLogger::eLogType::LOGTYPE_DAT);
}

TEST(Logger, dat_conversion_with_one_file)
{
	TestConvertLog("./tests/data/logger_dat", cISLogger::eLogType::LOGTYPE_DAT, cISLogger::eLogType::LOGTYPE_DAT);
	TestConvertLog("./tests/data/logger_dat", cISLogger::eLogType::LOGTYPE_DAT, cISLogger::eLogType::LOGTYPE_SDAT);
	TestConvertLog("./tests/data/logger_dat", cISLogger::eLogType::LOGTYPE_DAT, cISLogger::eLogType::LOGTYPE_CSV);
}

TEST(Logger, dat_conversion_with_multiple_files)
{
	TestConvertLog("./tests/data/logger_dat2", cISLogger::eLogType::LOGTYPE_DAT, cISLogger::eLogType::LOGTYPE_DAT);
	TestConvertLog("./tests/data/logger_dat2", cISLogger::eLogType::LOGTYPE_DAT, cISLogger::eLogType::LOGTYPE_SDAT);
	TestConvertLog("./tests/data/logger_dat2", cISLogger::eLogType::LOGTYPE_DAT, cISLogger::eLogType::LOGTYPE_CSV);
}

TEST(Logger, dat_conversion_with_multiple_files_issue_Aug_2017)
{
	TestConvertLog("./tests/data/logger_dat3", cISLogger::eLogType::LOGTYPE_DAT, cISLogger::eLogType::LOGTYPE_DAT);
	TestConvertLog("./tests/data/logger_dat3", cISLogger::eLogType::LOGTYPE_DAT, cISLogger::eLogType::LOGTYPE_SDAT);
// 	TestConvertLog("./tests/data/logger_dat3", cISLogger::eLogType::LOGTYPE_DAT, cISLogger::eLogType::LOGTYPE_CSV, false);
}

TEST(Logger, sdat_conversion)
{
	TestConvertLog("./tests/data/logger_sdat", cISLogger::eLogType::LOGTYPE_SDAT, cISLogger::eLogType::LOGTYPE_DAT);
	TestConvertLog("./tests/data/logger_sdat", cISLogger::eLogType::LOGTYPE_SDAT, cISLogger::eLogType::LOGTYPE_SDAT);
	TestConvertLog("./tests/data/logger_sdat", cISLogger::eLogType::LOGTYPE_SDAT, cISLogger::eLogType::LOGTYPE_CSV);
}
