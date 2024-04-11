#include <stdarg.h>
#include "protocol_nmea.h"
#include "device_runtime_test.h"



DeviceRuntimeTest::DeviceRuntimeTest()
{
    m_hist.gps1Pos.leapS = C_GPS_LEAP_SECONDS;
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
    case NMEA_MSG_ID_GxZDA:     TestNmeaZda(msg, msgSize);      break;
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

    utc_time_t t;
    int utcWeekday = gpsTowMsToUtcWeekday(gpsPos.timeOfWeekMs, m_hist.gps1Pos.leapS);
    nmea_parse_gga((const char *)msg, msgSize, gpsPos, t, utcWeekday);

    printf("NMEA GGA (%d ms, %d wkday): %.*s", gpsPos.timeOfWeekMs, utcWeekday, msgSize, msg);

    if (CheckGpsTimeMs("NEA GGA Error", gpsPos.timeOfWeekMs, gpsPos.week, ggaHist.gpsPos.timeOfWeekMs, ggaHist.gpsPos.week))
    {
        LogEvent("  1: %.*s  2: %.*s", ggaHist.msgSize, ggaHist.msg, msgSize, msg);
    }

    // Update history
    ggaHist.gpsPos = gpsPos;
    memcpy(ggaHist.msg, msg, _MIN(msgSize, MAX_MSG_LENGTH_NMEA));
    ggaHist.msgSize = msgSize;
}

void DeviceRuntimeTest::TestNmeaZda(const uint8_t* msg, int msgSize)
{
    zda_nmea_history_t &zdaHist = m_hist.nmea.zda;

    uint32_t gpsTowMs;
    uint32_t gpsWeek;
    utc_date_t utcDate; 
    utc_time_t utcTime;
    int leapS;
    nmea_parse_zda((char*)msg, msgSize, gpsTowMs, gpsWeek, utcDate, utcTime, C_GPS_LEAP_SECONDS);

    printf("NMEA ZDA (%d ms): %.*s", gpsTowMs, msgSize, msg);

    if (CheckGpsTimeMs("NMEA ZDA Error", gpsTowMs, gpsWeek, zdaHist.gpsTowMs, zdaHist.gpsWeek))
    {
        LogEvent("  1: %.*s  2: %.*s", zdaHist.msgSize, zdaHist.msg, msgSize, msg);
    }

    // Update history
    zdaHist.gpsTowMs = gpsTowMs;
    zdaHist.gpsWeek = gpsWeek;
    memcpy(zdaHist.msg, msg, _MIN(msgSize, MAX_MSG_LENGTH_NMEA));
    zdaHist.msgSize = msgSize;
}

bool DeviceRuntimeTest::CheckGpsTimeMs(char* description, uint32_t towMs, uint32_t gpsWeek, uint32_t histTowMs, uint32_t histWeek)
{
    int toyMs = towMs + gpsWeek * C_MILLISECONDS_PER_WEEK;
    int histToyMs = histTowMs + histWeek * C_MILLISECONDS_PER_WEEK;

    if (toyMs == histToyMs)
    {   // Duplicate time
        LogEvent("%s: Duplicate time: %d ms %d week >> %d ms %d week\n", description, histTowMs, histWeek, towMs, gpsWeek);
        return true;
    }
    else if (toyMs < histToyMs)
    {   // Reversed time
        LogEvent("%s: Reversed time: %d ms %d week >> %d ms %d week\n", description, histTowMs, histWeek, towMs, gpsWeek);
        return true;
    }

    return false;
}

void DeviceRuntimeTest::LogEvent(const char *format, ...)
{
    va_list args;
    va_start(args, format);

    vprintf(format, args);

#if 0
    FILE *file = fopen("realtime_test_log.txt", "a");
    if (file != NULL)
    {
        // fprintf(file, "NMEA Error: ");
        vfprintf(file, format, args);
        fclose(file);
    }
#endif

    va_end(args); // Clean up the variable arguments list
}

