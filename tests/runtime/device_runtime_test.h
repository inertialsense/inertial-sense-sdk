#include <cstdint>
#include "ISComm.h"
#include "time_conversion.h"


class DeviceRuntimeTest
{
    typedef struct
    {
        utc_date_t date;
        utc_time_t time;
        uint32_t gpsTowMs;
        uint32_t gpsWeek;
        uint8_t msg[MAX_MSG_LENGTH_NMEA];
        int msgSize;
    } zda_nmea_history_t;

    typedef struct
    {
        gps_pos_t gpsPos;
        uint8_t msg[MAX_MSG_LENGTH_NMEA];
        int msgSize;
    } gga_nmea_history_t;

public:
    DeviceRuntimeTest();
    
    void ProcessRaw(uint8_t *data, int dataSize);
    void ProcessISB(const p_data_hdr_t &dataHdr, const uint8_t *dataBuf);
    void ProcessNMEA(const uint8_t* msg, int msgSize);

private:
    void TestNmeaGga(const uint8_t* msg, int msgSize);
    void TestNmeaZda(const uint8_t* msg, int msgSize);
    bool CheckGpsTimeMs(char* description, uint32_t towMs, uint32_t gpsWeek, uint32_t histTowMs, uint32_t histWeek);
    void LogEvent(const char *format, ...);

    bool m_enable = true;

    struct
    {
        gps_pos_t                   gps1Pos;
        
        struct
        {
            zda_nmea_history_t      zda;
            gga_nmea_history_t      gga;
        } nmea;
    } m_hist = {};
    

};