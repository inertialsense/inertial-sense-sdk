#include <stdarg.h>
#include "protocol_nmea.h"
#include "device_runtime_test.h"



DeviceRuntimeTest::DeviceRuntimeTest()
{
    m_hist.gps1Pos.week = 2309;
}

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
    case DID_GPS1_POS:      copyDataPToStructP2(&m_hist.gps1Pos, &dataHdr, dataBuf, sizeof(gps_pos_t));       break;
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
    if (m_hist.gps1Pos.week == 0)
    {   // Require week for time conversion
        return;
    }

    gga_nmea_history_t &ggaHist = m_hist.nmea.gga;
    gps_pos_t gpsPos = {};

    uint32_t weekday = m_hist.gps1Pos.timeOfWeekMs / 86400000;
    // uint32_t weekday = 0;
    utc_time_t t;
    nmea_parse_gga_to_did_gps(gpsPos, t, (char*)msg, msgSize, weekday);

    printf("NMEA (%d ms %d): %.*s", gpsPos.timeOfWeekMs, weekday, msgSize, msg);

    if (gpsPos.timeOfWeekMs == ggaHist.gpsPos.timeOfWeekMs)
    {   // Duplicate time
        LogEvent("NMEA Error: GGA duplicate time: %d ms >> %d ms\n", ggaHist.gpsPos.timeOfWeekMs, gpsPos.timeOfWeekMs);
        LogEvent("  1: %.*s  2: %.*s", ggaHist.msgSize, ggaHist.msg, msgSize, msg);
    }
    else if (gpsPos.timeOfWeekMs < ggaHist.gpsPos.timeOfWeekMs)
    {   // Regressed time
        LogEvent("NMEA Error: GGA time reversed direction: %d ms >> %d ms\n", ggaHist.gpsPos.timeOfWeekMs, gpsPos.timeOfWeekMs);
        LogEvent("  1: %.*s  2: %.*s", ggaHist.msgSize, ggaHist.msg, msgSize, msg);
    }

    // Update history
    ggaHist.gpsPos = gpsPos;
    memcpy(ggaHist.msg, msg, _MIN(msgSize, MAX_MSG_LENGTH_NMEA));
    ggaHist.msgSize = msgSize;
}

void DeviceRuntimeTest::LogEvent(const char *format, ...)
{
    va_list args;
    va_start(args, format);

    vprintf(format, args);

    FILE *file;
    if (file = fopen("realtime_test_log.txt", "a"))
    {
        fprintf(file, "NMEA Error: ");
        vfprintf(file, format, args);
        fclose(file);
    }
}

