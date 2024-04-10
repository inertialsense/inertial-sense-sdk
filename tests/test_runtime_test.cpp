#include <gtest/gtest.h>
#include "ISDataMappings.h"
#include "ISFileManager.h"
#include "ISLogger.h"
#include "test_data_utils.h"
#include "device_runtime_test.h"

using namespace std;


void run_realtime_test(uint8_t *stream, int streamSize)
{
    is_comm_instance_t comm;
	uint8_t comBuf[PKT_BUF_SIZE];
    is_comm_init(&comm, comBuf, PKT_BUF_SIZE);

	DeviceRuntimeTest runtimeTest;

	const int chunkSize = 100;
	for (int i=0; i<streamSize; )
	{
		// Get available size of comm buffer
		int n = _MIN(is_comm_free(&comm), chunkSize);

		// Copy data directly into comm buffer
		memcpy(comm.rxBuf.tail, stream+i, n);

		// Update comm buffer tail pointer
		comm.rxBuf.tail += n;
		i += n;

		// Search comm buffer for valid packets
		protocol_type_t ptype;
		while ((ptype = is_comm_parse(&comm)) != _PTYPE_NONE)
		{
            switch (ptype)
            {
			case _PTYPE_INERTIAL_SENSE_DATA:
				runtimeTest.ProcessISB(comm.rxPkt.dataHdr, comm.rxPkt.data.ptr);
				break;

			case _PTYPE_NMEA:
				runtimeTest.ProcessNMEA(comm.rxPkt.data.ptr, comm.rxPkt.data.size);
				break;

			case _PTYPE_RTCM3:
			case _PTYPE_UBLOX:
				break;
			}
		}

		runtimeTest.ProcessRaw(stream+n, chunkSize);
		n += chunkSize;
	}
}


TEST(runtime_tests, data_with_no_errors)
{
	uint8_t stream[1000] = {0};
	int streamSize = GenerateDataStream(stream, sizeof(stream));
	run_realtime_test(stream, streamSize);
}

TEST(runtime_tests, timestamp_duplicates)
{
	// uint8_t stream[5000000] = {0};
	// int streamSize = GenerateDataStream(stream, sizeof(stream), GEN_LOG_OPTIONS_TIMESTAMP_DUPLICATE);
	// run_realtime_test(stream, streamSize);
}
