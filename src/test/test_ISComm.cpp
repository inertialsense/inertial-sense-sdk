#include <gtest/gtest.h>
#include "../ISComm.h"
#include "../protocol_nmea.h"


// Tests
#define TEST_ALTERNATING_ISB_NMEA_PARSE_ERRORS 	1


static is_comm_instance_t g_comm;

#if 1
uint8_t rxBuf[8192] = {0};
uint8_t txBuf[1024] = {0};

#define BUF_FREE    (txEnd-txPtr)
#define BUF_USED    (txPtr-txBuf)
#define WHILE_FULL  if (txPtr > txEnd - 100) break;

TEST(ISComm, alternating_isb_nmea_parse_error_check)
{
    int n;

    is_comm_init(&g_comm, rxBuf, sizeof(rxBuf));

    uint8_t *txPtr = txBuf;
    uint8_t *txEnd = txBuf + sizeof(txBuf);

    int msgCntIsb = 0;
    int msgCntNmea = 0;

    for (int i=0;; i++)
    {
        // append NMEA dev info
        dev_info_t info = {};
        info.serialNumber = 123456 + i;
        info.buildNumber = 789 + i;
        n = nmea_dev_info((char*)txPtr, BUF_FREE, info);
        txPtr += n;
        msgCntNmea++;
        WHILE_FULL;

        // append ISB get data DEV_INFO
        n = is_comm_get_data(&g_comm, DID_DEV_INFO, 0, sizeof(dev_info_t), 0);
        memcpy(txPtr, g_comm.buf.start, n);
        txPtr += n;
        msgCntIsb++;
        WHILE_FULL;

        // append NMEA query dev info
        memcpy(txPtr, NMEA_CMD_QUERY_DEVICE_INFO, n = NMEA_CMD_SIZE);
        txPtr += n;
        msgCntNmea++;
        WHILE_FULL;

        // append ISB DID_INS_1
        ins_1_t ins1 = {};
        ins1.timeOfWeek = 123.456 + i*2;
        ins1.hdwStatus = 78 + i*2;
        ins1.insStatus = 90 + i*2;
        n = is_comm_data(&g_comm, DID_INS_1, 0, sizeof(ins_1_t), &ins1);
        memcpy(txPtr, g_comm.buf.start, n);
        txPtr += n;
        msgCntIsb++;
        WHILE_FULL;
    }


    for (txPtr = txBuf; txPtr < txEnd; )
    {
		// Get available size of g_comm buffer.
		n = _MIN(is_comm_free(&g_comm), 10);

		// Read data directly into g_comm buffer
        memcpy(g_comm.buf.tail, txPtr, n);

        // Update g_comm buffer tail pointer
        txPtr += n;
        g_comm.buf.tail += n;

        // Search g_comm buffer for valid packets
        protocol_type_t ptype;
        while ((ptype = is_comm_parse(&g_comm)) != _PTYPE_NONE)
        {
            if (g_comm.rxErrorCount)
            {
                int j=0; j++;
            }
            ASSERT_EQ(g_comm.rxErrorCount, 0);

            uint8_t error = 0;
            uint8_t *dataPtr = g_comm.dataPtr + g_comm.dataHdr.offset;
            uint32_t dataSize = g_comm.dataHdr.size;

            switch (ptype)
            {
            case _PTYPE_PARSE_ERROR:
                error = 1;
                break;

            case _PTYPE_INERTIAL_SENSE_DATA:
            case _PTYPE_INERTIAL_SENSE_CMD:
                msgCntIsb--;
                if (!msgCntIsb && !msgCntNmea) { ASSERT_TRUE(true); return; } // Done
                break;

            case _PTYPE_NMEA:
                msgCntNmea--;
                if (!msgCntIsb && !msgCntNmea) { ASSERT_TRUE(true); return; } // Done
                break;

            default:    // We shouldn't get here
                ASSERT_TRUE(false);
            }
        }
    }

    ASSERT_EQ(g_comm.rxErrorCount, 0);
    ASSERT_EQ(msgCntIsb, 0);
    ASSERT_EQ(msgCntNmea, 0);
}
#endif
