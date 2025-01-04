/**
 * @file test_data_utils.h 
 * @brief a collection of functions and classes that might be useful when writing/running unit tests
 *
 * @author Walt Johnson on 3/6/24
 * @copyright Copyright (c) 2024 Inertial Sense, Inc. All rights reserved.
 */

#ifndef IS_SDK_UNIT_TESTS_TEST_DATA_UTILS_H
#define IS_SDK_UNIT_TESTS_TEST_DATA_UTILS_H

#include <list>

#include "data_sets.h"
#include "ISComm.h"
#include "ISLogger.h"
#include "time_conversion.h"

/**
 * A macro to generate a random integer number between two values.
 */
#define RAND_RANGE(MIN, MAX) (MIN + rand() / (RAND_MAX / (MAX - MIN + 1) + 1))

typedef struct
{
    protocol_type_t         ptype;      // Data start byte
    p_data_hdr_t            dataHdr;
    uDatasets               data;
    is_comm_instance_t      comm;
    int                     pktSize;
} test_message_t;

enum eTestGenDataOptions
{
    GEN_LOG_OPTIONS_NONE                                = 0x00000000,
    GEN_LOG_OPTIONS_INSERT_GARBAGE_BETWEEN_MSGS         = 0x00000001,
    GEN_LOG_OPTIONS_MISSING_MESSAGE_END                 = 0x00000002,
    GEN_LOG_OPTIONS_TIMESTAMP_DUPLICATE                 = 0x00000008,
    GEN_LOG_OPTIONS_TIMESTAMP_REVERSE                   = 0x00000010,
};

void init_test_comm_instance(is_comm_instance_t* c, uint8_t *buffer, int bufferSize);

void CurrentGpsTimeMs(uint32_t &gpsTimeOfWeekMs, uint32_t &gpsWeek);
void PrintUtcDateTime(utc_date_t &utcDate, utc_time_t &utcTime);
void PrintUtcStdTm(std::tm &utcTime, uint32_t milliseconds=0);
bool GenerateMessage(test_message_t &msg, protocol_type_t ptype=_PTYPE_NONE);
void GenerateDataLogFiles(int numDevices, const std::string& directory, cISLogger::eLogType logType, float logSizeMB=20, eTestGenDataOptions options=GEN_LOG_OPTIONS_NONE);
int GenerateDataStream(uint8_t *buffer, int bufferSize, eTestGenDataOptions options=GEN_LOG_OPTIONS_NONE);
uint32_t GenerateRawLogData(std::list<std::vector<uint8_t>*>& msgs, float logSizeMB=20, eTestGenDataOptions options=GEN_LOG_OPTIONS_NONE);


/**
 * Generates a string of some arbitrary length, containing some number of random sentences/paragraphs of "Lorem Ipsum" text.
 * This is primarily used to generate files of random content.
 * @param minWords  the minimum number of words per sentence to generate
 * @param maxWords  the maximum number of words per sentence to generate
 * @param minSentences  the minimum number of sentences per paragraph to generate
 * @param maxSentences  the maximum number of sentences per paragraph to generate
 * @param numLines  the actual number of lines (paragraphs?) to generate (this is NOT random).
 * @return a std::string containing the generated content
 */
std::string LoremIpsum(int minWords, int maxWords, int minSentences, int maxSentences, int numLines);


#endif //IS_SDK_UNIT_TESTS_TEST_DATA_UTILS_H
