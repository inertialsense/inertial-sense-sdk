#include "gtest/gtest.h"
#include <deque>
#include "../com_manager.h"
#include "../ring_buffer.h"

#define TASK_PERIOD_MS		1				// 1 KHz
#if 0
#define DEBUG_PRINTF	printf
#else
#define DEBUG_PRINTF	
#endif

typedef struct
{
	com_manager_t			cm;
	ring_buffer_t			cmBufRingBuffer[NUM_COM_PORTS] = { 0 };
	com_manager_status_t	cmBufStatus[NUM_COM_PORTS] = { 0 };
	broadcast_msg_t			cmBufBcastMsg[MAX_NUM_BCAST_MSGS] = { 0 };
	struct  
	{
		dev_info_t			devInfo;
		nvm_flash_cfg_t		nvmFlashCfg;
		ascii_msgs_t		asciiMsgs;
	}						msgs = { 0 };

	// Used to simulate serial ports
	ring_buf_t				portRxBuf;
	uint8_t					portRxBuffer[8192];
	ring_buf_t				portTxBuf;
	uint8_t					portTxBuffer[8192];
} test_data_t;

typedef struct
{
	uint32_t did;
	uint32_t size;
	uDatasets data;
} data_holder_t;

test_data_t tcm = {};
std::deque<data_holder_t> g_testRxDeque;
std::deque<data_holder_t> g_testTxDeque;


int portRead(CMHANDLE cmHandle, int pHandle, unsigned char* buf, int len)
{
	return ringBufRead(&tcm.portRxBuf, buf, len);
}

int portWrite(CMHANDLE cmHandle, int pHandle, buffer_t* packet)
{
	return ringBufWrite(&tcm.portTxBuf, packet->buf, packet->size);
}

void postRxRead(CMHANDLE cmHandle, int pHandle, p_data_t* dataRead)
{
	data_holder_t td = g_testRxDeque.front();
	g_testRxDeque.pop_front();

	DEBUG_PRINTF("DID: %3d, size: %3d\n", td.did, td.size);

	EXPECT_EQ(td.did, dataRead->hdr.id);
	EXPECT_EQ(td.size, dataRead->hdr.size);
	EXPECT_TRUE(memcmp(&td.data, dataRead->buf, td.size)==0);
}

void disableBroadcasts(CMHANDLE cmHandle, int pHandle)
{
}

void prepDevInfo(CMHANDLE cmHandle, int pHandle)
{
}

void writeNvrUserpageFlashCfg(CMHANDLE cmHandle, int pHandle, p_data_t* data)
{
}

// return 1 on success, 0 on failure
int asciiMessageHandler(CMHANDLE cmHandle, int pHandle, unsigned char* messageId, unsigned char* line, int lineLength)
{
	int messageIdUInt = ASCII_MESSAGEID_TO_UINT(messageId);
// 	comWrite(pHandle, line, lineLength); // echo back
// 	time_delay(50); // give time for the echo to come back

	switch (messageIdUInt)
	{
	case 0x53545042: // "STPB" - stop all broadcasts on all ports
		disableBroadcasts(cmHandle, -1);
		break;

	case 0x53545043: // "STPC" - stop all broadcasts on current port
		disableBroadcasts(cmHandle, pHandle);
		break;

	case 0x424c454e: // "BLEN" - bootloader enable
		break;

	case 0x4e454c42: // "NELB" - SAM bootloader assistant (SAM-BA) enable
		break;

	case 0x53525354: // "SRST" - soft reset
		break;

	case 0x494e464f: // "INFO" - query device version information
		break;

	case 0x50455253: // "PERS" - Save persistent messages to flash memory
		break;

	default:
		return 0;
	}

	return 1;
}


int asciiMessageHandlerPost(CMHANDLE cmHandle, int pHandle, unsigned char* messageId, unsigned char* line, int lineLength)
{
	int messageIdUInt = ASCII_MESSAGEID_TO_UINT(messageId);
	switch (messageIdUInt)
	{
		// post parse, just pretend the binary ascii message was sent
	case 0x41534342: // "ASCB"
// 		writeAsciiBcastPeriod(cmHandle, pHandle, NULLPTR);
		break;
	}

	return 0;
}


bool initComManager(test_data_t &t)
{
	// Init ComManager
	com_manager_buffers_t cmBuffers = { 0 };
	cmBuffers.ringBuffer = t.cmBufRingBuffer;
	cmBuffers.ringBufferSize = sizeof(t.cmBufRingBuffer);
	cmBuffers.status = t.cmBufStatus;
	cmBuffers.statusSize = sizeof(t.cmBufStatus);
	cmBuffers.broadcastMsg = t.cmBufBcastMsg;
	cmBuffers.broadcastMsgSize = sizeof(t.cmBufBcastMsg);
	if (comManagerInitInstance(&(t.cm), 1, 0, TASK_PERIOD_MS, 0, portRead, portWrite, 0, postRxRead, 0, disableBroadcasts, &cmBuffers))
	{	// Fail to init
		return false;
	}

	comManagerRegister(DID_DEV_INFO, prepDevInfo, 0, &(t.msgs.devInfo), 0, sizeof(dev_info_t), 0);
	comManagerRegister(DID_FLASH_CONFIG, 0, writeNvrUserpageFlashCfg, &t.msgs.nvmFlashCfg, 0, sizeof(nvm_flash_cfg_t), 0);

#define NUMBER_OF_ASCII_BROADCAST_MESSAGES 11

	STATIC_ASSERT(sizeof(ascii_msgs_t) == NUMBER_OF_ASCII_BROADCAST_MESSAGES * sizeof(uint32_t));
	static uint16_t ASCB_fieldsAndOffsets[NUMBER_OF_ASCII_BROADCAST_MESSAGES] = { 0x0001, 0x0401, 0x0801, 0x0c01, 0x1001, 0x1401, 0x1801, 0x1c01, 0x2001, 0x2401, 0x2801 };
	static asciiMessageMap_t s_asciiMessages[] =
	{
		// ASCB - Change ASCII broadcast periods, fields: options, imu, ins1, ins2, gps pos, gps vel, gga, gll, gsa, gtv, rmc all 4 byte unsigned int
		{ { 'A', 'S', 'C', 'B' }, (uint8_t*)&t.msgs.asciiMsgs, sizeof(ascii_msgs_t), _ARRAY_ELEMENT_COUNT(ASCB_fieldsAndOffsets), ASCB_fieldsAndOffsets }
	};
	comManagerRegisterASCII(s_asciiMessages, _ARRAY_ELEMENT_COUNT(s_asciiMessages), asciiMessageHandler, asciiMessageHandlerPost);

	return true;
}

bool init(test_data_t &t)
{
	// Init Port Buffers
	ringBufInit(&(t.portTxBuf), t.portTxBuffer, sizeof(t.portTxBuffer));
	ringBufInit(&(t.portRxBuf), t.portRxBuffer, sizeof(t.portRxBuffer));

	return initComManager(t);
}

void generateData(std::deque<data_holder_t> &testDeque)
{
#define NUM_DATA_SETS		40
	testDeque.clear();

	// Generate data and add to test deque
	for (int i = 0; i < NUM_DATA_SETS; i++)
	{
		data_holder_t td = { 0 };

		if (i % 3 == 0)
		{	// GPS
			td.did = DID_GPS1_POS;
			td.size = sizeof(gps_pos_t);

			td.data.gpsPos.timeOfWeekMs = i * 1000;
			td.data.gpsPos.week = i * 10;
			td.data.gpsPos.status = i;
			td.data.gpsPos.ecef[0] = (double)i*1.234;
			td.data.gpsPos.ecef[1] = (double)i*2.345;
			td.data.gpsPos.ecef[2] = (double)i*3.456;
			td.data.gpsPos.lla[0] = (double)i*1.234;
			td.data.gpsPos.lla[1] = (double)i*2.345;
			td.data.gpsPos.lla[2] = (double)i*3.456;
			td.data.gpsPos.hAcc = (float)i;
			td.data.gpsPos.cnoMean = (float)i;
			td.data.gpsPos.hMSL = i;
			td.data.gpsPos.pDop = (float)i;
			td.data.gpsPos.towOffset = (double)i*123.4;
			td.data.gpsPos.leapS = (uint8_t)i;
		}
		else
		{	// INS 1
			td.did = DID_INS_1;
			td.size = sizeof(ins_1_t);

			td.data.ins1.timeOfWeek = (double)i;
			td.data.ins1.week = i;
			td.data.ins1.insStatus = i;
			td.data.ins1.hdwStatus = i;
			td.data.ins1.theta[0] = i * 2.0f;
			td.data.ins1.theta[1] = i * 3.0f;
			td.data.ins1.theta[2] = i * 4.0f;
			td.data.ins1.uvw[0] = i * 5.0f;
			td.data.ins1.uvw[1] = i * 6.0f;
			td.data.ins1.uvw[2] = i * 7.0f;
			td.data.ins1.lla[0] = 40.330565516;
			td.data.ins1.lla[1] = -111.725787806;
			td.data.ins1.lla[2] = 1408.565264;
			td.data.ins1.ned[0] = i * 1.234f;
			td.data.ins1.ned[1] = i * 2.345f;
			td.data.ins1.ned[2] = i * 3.456f;
		}

		DEBUG_PRINTF("DID: %3d, size: %3d\n", td.did, td.size);

		// Add data to deque
		testDeque.push_back(td);
	}
}


void addDequeToRingBuf(std::deque<data_holder_t> &testDeque, ring_buf_t *rbuf)
{
#define COMM_BUFFER_SIZE    2048
	is_comm_instance_t		comm;
	uint8_t					comm_buffer[COMM_BUFFER_SIZE];
	comm.buffer = comm_buffer;
	comm.bufferSize = COMM_BUFFER_SIZE;
	is_comm_init(&comm);

	int n, k=0;

	for (int i = 0; i < testDeque.size(); i++)
	{
		data_holder_t td = testDeque[i];

		// Packetize data 
		n = is_comm_data(&comm, td.did, 0, td.size, (void*)&(td.data));

		// Add packetized data to ring buffer
		ringBufWrite(rbuf, comm.buffer, n);
	}

	// Buffer overflow not allowed for test
	ASSERT_TRUE(ringBufFree(rbuf));
}


void parseDataPortTxBuf(std::deque<data_holder_t> &testDeque, test_data_t &t)
{
#define COMM_BUFFER_SIZE    2048
	is_comm_instance_t		comm;
	uint8_t					comm_buffer[COMM_BUFFER_SIZE];
	comm.buffer = comm_buffer;
	comm.bufferSize = COMM_BUFFER_SIZE;
	is_comm_init(&comm);
	unsigned char c;
	int did;
	uDatasets dataWritten;

	while (ringBufUsed(&t.portTxBuf)>0)
	{
		ringBufRead(&t.portTxBuf, &c, 1);

		if ((did = is_comm_parse(&comm, c)) != DID_NULL)
		{	// Found data
			data_holder_t td = testDeque.front();
			testDeque.pop_front();

			is_comm_copy_to_struct(&dataWritten, &comm, sizeof(uDatasets));

			EXPECT_EQ(td.did, did);
			EXPECT_EQ(td.size, comm.dataSize);
			EXPECT_TRUE(memcmp(&td.data, comm.buffer, td.size) == 0);
		}
	}
}


int ringBuftoRingBufWrite(ring_buf_t *dst, ring_buf_t *src, int len)
{
	uint8_t *buf = new uint8_t[len];

	len = ringBufRead(src, buf, len);
	return ringBufWrite(dst, buf, len);
}


TEST(ComManager, BasicTxTest)
{
	// Initialize Com Manager
	init(tcm);

	// Generate and add data to deque
	generateData(g_testTxDeque);

	// Use Com Manager to send deque data to Tx port ring buffer
	for(int i=0; i<g_testTxDeque.size(); i++)
	{
		data_holder_t td = g_testTxDeque[i];

		// Send data - writes data to tcm.txBuf
		comManagerSendDataNoAckInstance(&tcm.cm, 0, td.did, &td.data, td.size, 0);
	}

	// Test that data parsed from Tx port matches deque data
	parseDataPortTxBuf(g_testTxDeque, tcm);

	// Check that we got all data
	EXPECT_TRUE(g_testTxDeque.empty());
	EXPECT_TRUE(ringBufUsed(&tcm.portTxBuf) == 0);
}


TEST(ComManager, BasicRxTest)
{
	// Initialize Com Manager
	init(tcm);

	// Generate and add data to deque
	generateData(g_testRxDeque);

	// Add deque data to Rx port ring buffer
	addDequeToRingBuf(g_testRxDeque, &tcm.portRxBuf);

	DEBUG_PRINTF("Checking Data.  Size: %d\n", ringBufUsed(&tcm.portRxBuf));

	while (!ringBufEmpty(&tcm.portRxBuf))
	{
		// Step Com Manager and check that was received correctly inside postRxRead()
		comManagerStepInstance(&tcm.cm);	// 2048 byte limit each step
	}

	// Check that no data was left behind 
	EXPECT_TRUE(g_testRxDeque.empty());
	EXPECT_TRUE(ringBufUsed(&tcm.portRxBuf) == 0);
}


#if 0
// Tests that ComManager handles segmented serial data properly
TEST(ComManager, SegmentedRxTest)
{
	ring_buf_t tmpRBuf;
	uint8_t buffer[8192];

	// Initialize temporary ring buffer
	ringBufInit(&tmpRBuf, buffer, sizeof(buffer));

	// Initialize Com Manager
	init(tcm);

	// Generate and add data to deque
	generateData(g_testRxDeque);

	// Add deque data to temporary ring buffer
	addDequeToRingBuf(g_testRxDeque, &tmpRBuf);

	DEBUG_PRINTF("Checking Data:\n");

	// Divide data written to Com Manager into pieces
#define TIMES_TO_DIVIDE_DATA 10
	int bytesToWrite = ringBufUsed(&tmpRBuf) / TIMES_TO_DIVIDE_DATA;
	while (!ringBufEmpty(&tmpRBuf) && !g_testRxDeque.empty())
	{
		// Partial write of data
		ringBuftoRingBufWrite(&tcm.portRxBuf, &tmpRBuf, bytesToWrite);

		while (!ringBufEmpty(&tcm.portRxBuf))
		{
			// Step Com Manager and check that was received correctly inside postRxRead()
			comManagerStepInstance(&tcm.cm);
			comManagerStepInstance(&tcm.cm);
		}
	}

	// Check that no data was left behind 
	EXPECT_TRUE(g_testRxDeque.empty());
	EXPECT_TRUE(ringBufEmpty(&tcm.portRxBuf));
}
#endif