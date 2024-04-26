#include <gtest/gtest.h>
#include "ISDataMappings.h"
#include "ISFileManager.h"
#include "ISLogger.h"
#include "test_data_utils.h"
#include "device_runtime_tests.h"

using namespace std;


static is_comm_instance_t s_comm;
static uint8_t s_comBuf[PKT_BUF_SIZE];

void run_realtime_test(DeviceRuntimeTests &RuntimeTest, uint8_t *stream, int streamSize)
{
	// printf("%.*s\n", streamSize, stream);

    is_comm_init(&s_comm, s_comBuf, PKT_BUF_SIZE);

	const int chunkSize = 100;
	for (int i=0; i<streamSize; )
	{
		// Get available size of comm buffer
		int n = _MIN(is_comm_free(&s_comm), chunkSize);

		// Don't exceed streamSize;
		n = _MIN(n, streamSize-i);

		// Copy data directly into comm buffer
		memcpy(s_comm.rxBuf.tail, stream+i, n);

		// Update comm buffer tail pointer
		s_comm.rxBuf.tail += n;
		i += n;

		// Search comm buffer for valid packets
		protocol_type_t ptype;
		while ((ptype = is_comm_parse(&s_comm)) != _PTYPE_NONE)
		{
            switch (ptype)
            {
			case _PTYPE_PARSE_ERROR:
				RuntimeTest.ProcessParseError(s_comm);
				break;

			case _PTYPE_INERTIAL_SENSE_DATA:
				RuntimeTest.ProcessISB(s_comm.rxPkt.dataHdr, s_comm.rxPkt.data.ptr);
				break;

			case _PTYPE_NMEA:
				RuntimeTest.ProcessNMEA(s_comm.rxPkt.data.ptr, s_comm.rxPkt.data.size);
				break;

			case _PTYPE_RTCM3:
			case _PTYPE_UBLOX:
				break;
			}
		}

		n += chunkSize;
	}
}

#define BUFFER_SIZE		500000
// #define BUFFER_SIZE		5000

TEST(runtime_tests, data_with_no_errors)
{
	DeviceRuntimeTests RuntimeTest;
	RuntimeTest.Enable();
	uint8_t stream[BUFFER_SIZE] = {0};
	int streamSize = GenerateDataStream(stream, sizeof(stream));
	run_realtime_test(RuntimeTest, stream, streamSize);
	ASSERT_EQ( RuntimeTest.ErrorCount(), 0 );
}

TEST(runtime_tests, timestamp_duplicates)
{
	DeviceRuntimeTests RuntimeTest;
	RuntimeTest.Enable();
	uint8_t stream[BUFFER_SIZE] = {0};
	int streamSize = GenerateDataStream(stream, sizeof(stream), GEN_LOG_OPTIONS_TIMESTAMP_DUPLICATE);
	run_realtime_test(RuntimeTest, stream, streamSize);	
	ASSERT_NE( RuntimeTest.ErrorCount(), 0 );
	ASSERT_NE( RuntimeTest.m_errorCount.nmeaGgaTime, 0 );
}

TEST(runtime_tests, timestamp_reverse)
{
	DeviceRuntimeTests RuntimeTest;
	RuntimeTest.Enable();
	uint8_t stream[BUFFER_SIZE] = {0};
	int streamSize = GenerateDataStream(stream, sizeof(stream), GEN_LOG_OPTIONS_TIMESTAMP_REVERSE);
	run_realtime_test(RuntimeTest, stream, streamSize);	
	ASSERT_NE( RuntimeTest.ErrorCount(), 0 );
	ASSERT_NE( RuntimeTest.m_errorCount.nmeaGgaTime, 0 );
}

TEST(runtime_tests, truncate_message_end)
{
	DeviceRuntimeTests RuntimeTest;
	RuntimeTest.Enable();
	uint8_t stream[BUFFER_SIZE] = {0};
	int streamSize = GenerateDataStream(stream, sizeof(stream), GEN_LOG_OPTIONS_MISSING_MESSAGE_END);
	run_realtime_test(RuntimeTest, stream, streamSize);
	ASSERT_NE( RuntimeTest.ErrorCount(), 0 );
	ASSERT_NE( RuntimeTest.m_errorCount.parse, 0 );
}
