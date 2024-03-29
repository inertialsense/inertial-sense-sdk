/**
 * @file test_data_utils.h 
 * @brief a collection of functions and classes that might be useful when writing/running unit tests
 *
 * @author Walt Johnson on 3/6/24
 * @copyright Copyright (c) 2024 Inertial Sense, Inc. All rights reserved.
 */

#ifndef IS_SDK_UNIT_TESTS_TEST_UTILS_H
#define IS_SDK_UNIT_TESTS_TEST_UTILS_H

#include "data_sets.h"
#include "ISComm.h"


typedef struct
{
    protocol_type_t         ptype;	    // Data start byte
    p_data_hdr_t            dataHdr;
    uDatasets               data;
    is_comm_instance_t      comm;
    int                     pktSize;
} test_message_t;

typedef enum
{
    GEN_LOG_OPTIONS_NONE = 0,
    GEN_LOG_OPTIONS_INSERT_GARBAGE_BETWEEN_MSGS,
} test_gen_log_options_t;

void GenerateMessage(test_message_t &msg, protocol_type_t ptype);
void GenerateLogFiles(int numDevices, std::string directory, cISLogger::eLogType logType, float logSizeMB=20, int options=GEN_LOG_OPTIONS_NONE);







#endif //IS_SDK_UNIT_TESTS_TEST_UTILS_H
