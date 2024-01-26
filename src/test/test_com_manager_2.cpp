#include <gtest/gtest.h>
#include <stdlib.h>
#include "../com_manager.h"
#include "../protocol_nmea.h"

static imu_t g_imu;
static ins_1_t g_ins1;
static ins_2_t g_ins2;
dev_info_t g_devInfo;
static sys_sensors_t g_sensor_sys;
static sys_sensors_adc_t g_sensor_adc;
static sys_sensors_adc_t g_sensor_lsb;
static sys_sensors_adc_t g_adcSigma;
static sys_params_t	g_sysParams;
static nvm_flash_cfg_t g_nvmFlashCfg;
static rtos_info_t g_rtos;
static system_command_t g_sysCmd;
static debug_array_t g_debug;
static io_t g_IO;
static mag_cal_t g_magCal;

#define BUFFER_SIZE 8192

struct comManagerTest
{
	com_manager_t cm;
	uint8_t buffer[BUFFER_SIZE];
	int bufferSize;

	// flags for functions being called
	uint32_t readFncCallCount;
	uint32_t sendFncCallCount;
	uint32_t txFreeFncCallCount;
	uint32_t pstRxFncCallCount;
	uint32_t pstAckFncCallCount;
	uint32_t disableBcastFncCallCount;

	// com manager on the other end
	comManagerTest* cm2;
};

static comManagerTest cm1, cm2;

static int readFnc(CMHANDLE cmHandle, int pHandle, unsigned char* buf, int len)
{
	comManagerTest* t = (comManagerTest*)comManagerGetUserPointer(cmHandle);
	t->readFncCallCount++;
	int c = _MIN(t->cm2->bufferSize, len);
	memcpy(buf, t->cm2->buffer, c);
	memmove(t->cm2->buffer, t->cm2->buffer + c, (t->cm2->bufferSize -= c));
	return c;
}

static int sendFnc(CMHANDLE cmHandle, int pHandle, unsigned char* buf, int len)
{
	comManagerTest* t = (comManagerTest*)comManagerGetUserPointer(cmHandle);
	t->sendFncCallCount++;
	int sendCount = _MIN(len, (BUFFER_SIZE - t->bufferSize));
	if (sendCount != len)
	{
		throw "Overflowed test buffer!";
	}
	memcpy(t->buffer + t->bufferSize, buf, sendCount);
	t->bufferSize += sendCount;

	return sendCount;
}

static int txFreeFnc(CMHANDLE cmHandle, int pHandle)
{
	comManagerTest* t = (comManagerTest*)comManagerGetUserPointer(cmHandle);
	t->txFreeFncCallCount++;
	return (BUFFER_SIZE - t->bufferSize);
}

static void pstRxFnc(CMHANDLE cmHandle, int pHandle, p_data_t* dataRead)
{
	comManagerTest* t = (comManagerTest*)comManagerGetUserPointer(cmHandle);
	t->pstRxFncCallCount++;
}

static void pstAckFnc(CMHANDLE cmHandle, int pHandle, p_ack_t* ack, unsigned char packetIdentifier)
{
	comManagerTest* t = (comManagerTest*)comManagerGetUserPointer(cmHandle);
	t->pstAckFncCallCount++;
}

static void disableBcastFnc(CMHANDLE cmHandle, int pHandle)
{
	comManagerTest* t = (comManagerTest*)comManagerGetUserPointer(cmHandle);
	t->disableBcastFncCallCount++;
}

static int asciiMessageHandler(CMHANDLE cmHandle, int pHandle, unsigned char* messageId, unsigned char* line, int lineLength)
{
	// comWrite(ASCII_COM_USART_NUM, line, lineLength); // echo back

	switch (getNmeaMsgId(messageId, lineLength))
	{
	case NMEA_MSG_ID_STPB: // "STPB" - stop all broadcasts
		// disableBroadcasts(getGlobalComManager(), 0);
		break;

	case NMEA_MSG_ID_BLEN: // "BLEN" - boot loader enable
		break;

	// case NMEA_MSG_ID_PROF: // "PROF" - enable profile message
	// {
	// 	int enableProfiler;
	// 	if (sscanf((const char*)line, "$PROF,%d*", &enableProfiler) == 1)
	// 	{

	// 	}
	// } break;

	default:
		return 0;
	}

	return 1;
}

#define MAX_NUM_DEVICES			1

static void setupComManagers(comManagerTest* cm1, comManagerTest* cm2)
{
	memset(cm1, 0, sizeof(comManagerTest));
	memset(cm2, 0, sizeof(comManagerTest));

	com_manager_init_t cmBuffers = { 0 };
	cmBuffers.broadcastMsgSize = COM_MANAGER_BUF_SIZE_BCAST_MSG(MAX_NUM_BCAST_MSGS);
	cmBuffers.broadcastMsg = new broadcast_msg_t[MAX_NUM_BCAST_MSGS];
#define NUM_ENSURED_PKTS 20
	cmBuffers.ensuredPacketsSize = COM_MANAGER_BUF_SIZE_ENSURED_PKTS(NUM_ENSURED_PKTS);
	cmBuffers.ensuredPackets = new ensured_pkt_t[NUM_ENSURED_PKTS];
	com_manager_port_t *cmPort = new com_manager_port_t();

	comManagerInitInstance(&(cm1->cm), 1, 10, 10, 10, readFnc, sendFnc, txFreeFnc, pstRxFnc, pstAckFnc, disableBcastFnc, &cmBuffers, cmPort);
	comManagerInitInstance(&(cm2->cm), 1, 2, 5, 3, readFnc, sendFnc, txFreeFnc, pstRxFnc, pstAckFnc, disableBcastFnc, &cmBuffers, cmPort);
	cm1->cm2 = cm2;
	cm2->cm2 = cm1;
	comManagerAssignUserPointer(&(cm1->cm), cm1);
	comManagerAssignUserPointer(&(cm2->cm), cm2);

	// cm2 will act as uINS
	comManagerRegisterInstance(&(cm2->cm), DID_INS_1, 0, 0, &g_ins1, &g_ins1, sizeof(ins_1_t), 0);
	comManagerRegisterInstance(&(cm2->cm), DID_INS_2, 0, 0, &g_ins2, &g_ins2, sizeof(ins_2_t), 0);
	comManagerRegisterInstance(&(cm2->cm), DID_IMU, 0, 0, 0, &g_imu, sizeof(imu_t), 0);
	comManagerRegisterInstance(&(cm2->cm), DID_DEV_INFO, 0, 0, &g_devInfo, 0, sizeof(dev_info_t), 0);
	comManagerRegisterInstance(&(cm2->cm), DID_SYS_SENSORS, 0, 0, &g_sensor_sys, 0, sizeof(sys_sensors_t), 0);
	comManagerRegisterInstance(&(cm2->cm), DID_SENSORS_ADC, 0, 0, &g_sensor_lsb, 0, sizeof(sys_sensors_adc_t), 0);
	comManagerRegisterInstance(&(cm2->cm), DID_SENSORS_ADC_SIGMA, 0, 0, &g_adcSigma, 0, sizeof(sys_sensors_adc_t), 0);
// 	comManagerRegisterInstance(&(cm2->cm), DID_SENSORS_TC_BIAS, 0, 0, &g_tcBias, &g_tcBias, sizeof(sensors_t), 0);
// 	comManagerRegisterInstance(&(cm2->cm), DID_SENSORS_UCAL, 0, 0, &g_sensor_is1, 0, sizeof(sensors_w_temp_t), 0);
// 	comManagerRegisterInstance(&(cm2->cm), DID_SENSORS_TCAL, 0, 0, &g_sensor_is2, 0, sizeof(sensors_t), 0);
// 	comManagerRegisterInstance(&(cm2->cm), DID_SCOMP, 0, 0, &g_sc, &g_sc, sizeof(sensor_compensation_t), 0);
// 	comManagerRegisterInstance(&(cm2->cm), DID_HDW_PARAMS, 0, 0, &g_hdwParams, &g_hdwParams, sizeof(hdw_params_t), 0);
	comManagerRegisterInstance(&(cm2->cm), DID_SYS_PARAMS, 0, 0, &g_sysParams, &g_sysParams, sizeof(sys_params_t), 0);
// 	comManagerRegisterInstance(&(cm2->cm), DID_NVR_MANAGE_USERPAGE, 0, 0, &g_nvr_manage_userpage, &g_nvr_manage_userpage, sizeof(nvr_manage_t), 0);
// 	comManagerRegisterInstance(&(cm2->cm), DID_NVR_USERPAGE_SN, 0, 0, g_serialNumber, 0, sizeof(nvm_group_sn_t), 0);
// 	comManagerRegisterInstance(&(cm2->cm), DID_NVR_USERPAGE_G0, 0, 0, g_nvmU0, 0, sizeof(nvm_group_0_t), 0);
// 	comManagerRegisterInstance(&(cm2->cm), DID_NVR_USERPAGE_G1, 0, 0, g_nvmU1, 0, sizeof(nvm_group_1_t), 0);
	comManagerRegisterInstance(&(cm2->cm), DID_FLASH_CONFIG, 0, 0, &g_nvmFlashCfg, 0, sizeof(nvm_flash_cfg_t), 0);
	comManagerRegisterInstance(&(cm2->cm), DID_RTOS_INFO, 0, 0, &g_rtos, 0, sizeof(rtos_info_t), 0);
	comManagerRegisterInstance(&(cm2->cm), DID_SYS_CMD, 0, 0, &g_sysCmd, &g_sysCmd, sizeof(system_command_t), 0);
	comManagerRegisterInstance(&(cm2->cm), DID_DEBUG_ARRAY, 0, 0, &g_debug, &g_debug, sizeof(debug_array_t), 0);
// 	comManagerRegisterInstance(&(cm2->cm), DID_FEATURE_BITS, 0, 0, 0, 0, sizeof(feature_bits_t), 0);
	comManagerRegisterInstance(&(cm2->cm), DID_IO, 0, 0, &g_IO, &g_IO, sizeof(io_t), 0);
	comManagerRegisterInstance(&(cm2->cm), DID_MAG_CAL, 0, 0, &g_magCal, &g_magCal, sizeof(mag_cal_t), 0);
}

class cComManagerInit
{
public:
	cComManagerInit()
	{
		setupComManagers(&cm1, &cm2);
	}

	~cComManagerInit()
	{

	}
};

TEST(ComManager2, Bad_offset_packet)
{
	cComManagerInit cInit;

	// raspberry pi packet with did 2, size of 32 and offset of 3221225472, total size of 51 (1 encoded byte marker)                               v
	unsigned char dataPacket[] = { 0xff,0x04,0x82,0x03,0x02,0x00,0x00,0x00,0x20,0x00,0x00,0x00,0x00,0x00,0x00,0xc0,0xf9,0xe0,0xb3,0x10,0x81,0x40,0xfd,0x01,0xa4,0xa5,0x3c,0x34,0x3c,0x51,0xbb,0x85,0xcc,0x3f,0xbc,0xcd,0x7c,0xbe,0x3f,0x33,0xf3,0xde,0x3e,0x00,0x98,0x17,0xc1,0x00,0x99,0xcf,0xfe };

	// fill the send buffer of com manger 1 with garbage
	memcpy(cm1.buffer, dataPacket, sizeof(dataPacket));
	cm1.bufferSize = sizeof(dataPacket);

	// have com manager 1 send it's buffer
	comManagerStepInstance(&(cm1.cm));

	// consume entire buffer on other com manager
	while (cm1.bufferSize > 0)
	{
		comManagerStepInstance(&(cm2.cm));
	}

	EXPECT_TRUE(true);
}

TEST(ComManager2, Garbage_data_should_not_crash)
{
	// TODO: test fails with random value 1486253608
	unsigned int randomizer = (unsigned int)time(NULL);
	srand(randomizer);

	printf("Randomizer: %u\n", randomizer);

	cComManagerInit cInit;

	for (int i = 0; i < 1000; i++)
	{
		// fill the send buffer of com manger 1 with garbage
		for (int i = 0; i < BUFFER_SIZE - (rand() & 0xFF); i++)
		{
			uint8_t c = (rand() & 0xFF);

			// TODO: These bytes can exist in garbage data but will reset the com manager state, causing the actual message in this test to get skipped most
			// of the time, which is the behavior we want, but causes the test to fail
			// in the future, create 3 tests, one with this random data without special bytes, and another with special bytes that causes the real packet to
			// be skipped, and a third with random bytes that does not skip the real packet
			if (c == UBLOX_START_BYTE1 || c == RTCM3_START_BYTE || c == PSC_ASCII_START_BYTE || c == PSC_START_BYTE || c == PSC_ASCII_END_BYTE || c == PSC_END_BYTE)
			{
				c = '0';
			}
			cm1.buffer[i] = c;
			cm1.bufferSize = BUFFER_SIZE;
		}
		comManagerStepInstance(&(cm1.cm));

		// consume entire buffer on other com manager
		while (cm1.bufferSize > 0)
		{
			comManagerStepInstance(&(cm2.cm));
		}
	}

	// try to send a real message now, it should get parsed and received properly
	g_imu.I.acc[0] = 99.00f;
	comManagerSendDataInstance(&(cm1.cm), 0, DID_IMU, &g_imu, sizeof(g_imu), 0);
	comManagerStepInstance(&(cm1.cm));
	comManagerStepInstance(&(cm1.cm));
	g_imu.I.acc[0] = 0.0f;

	for (int i = 0; i < 512; i++)
	{
		comManagerStepInstance(&(cm2.cm));
	}

	EXPECT_TRUE(g_imu.I.acc[0] == 99.0f);
}
