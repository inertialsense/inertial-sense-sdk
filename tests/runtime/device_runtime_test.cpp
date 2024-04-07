#include <stdarg.h>
#include "protocol_nmea.h"
#include "device_runtime_test.h"



void DeviceRuntimeTest::ProcessRaw(uint8_t *data, int dataSize)
{

}


void DeviceRuntimeTest::ProcessISB(const p_data_hdr_t &dataHdr, const uint8_t *dataBuf)
{
    if (!m_enable)
    {
        return;
    }
    
    // printf("ISB: ID %d  Size %d\n", dataHdr.id, dataHdr.size);

    switch(dataHdr.id)
    {
    case DID_GPS1_POS:      copyDataPToStructP2(&m_hist.gpsPos, &dataHdr, dataBuf, sizeof(gps_pos_t));       break;
    }
}

void DeviceRuntimeTest::ProcessNMEA(const uint8_t* msg, int msgSize)
{
    if (!m_enable)
    {
        return;
    }

    // printf("NMEA (%d): %.*s", msgSize, msgSize, msg);
    
    int id = getNmeaMsgId(msg, msgSize);
    switch(id)
    {
    case NMEA_MSG_ID_GxGGA:     TestNmeaGga(msg, msgSize);      break;
    }
}

void DeviceRuntimeTest::TestNmeaGga(const uint8_t* msg, int msgSize)
{
    gps_pos_t &histPos = m_hist.nmea.gga.gpsPos;
    gps_pos_t gpsPos = {};
    nmea_parse_gga_to_did_gps(gpsPos, (char*)msg, msgSize, m_hist.gpsPos.week);

    printf("NMEA (%d): %.*s", msgSize, msgSize, msg);

    if (m_hist.gpsPos.week)
    {   // Require week for time conversion
        if (gpsPos.timeOfWeekMs == histPos.timeOfWeekMs)
        {   // Duplicate time
            LogErrorNMEA(msg, msgSize, "Duplicate time: %d ms >> %d ms", histPos.timeOfWeekMs, gpsPos.timeOfWeekMs);
        }
        else if (gpsPos.timeOfWeekMs < histPos.timeOfWeekMs)
        {   // Regressed time
            LogErrorNMEA(msg, msgSize, "Time reversed direction: %d ms >> %d ms", histPos.timeOfWeekMs, gpsPos.timeOfWeekMs);
        }
    }

    // Update history
    histPos = gpsPos;
}

void DeviceRuntimeTest::LogErrorNMEA(const uint8_t* msg, int msgSize, const char *format, ...)
{
    va_list args;
    va_start(args, format);

    printf("NMEA Error: ");
    vprintf(format, args);
    printf("\n");
    // printf("%.*s", msgSize, msg);   
}
