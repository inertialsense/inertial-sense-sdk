#include <cstdint>
#include "ISComm.h"


class DeviceRuntimeTest
{
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
    void LogEvent(const char *format, ...);

    bool m_enable = true;

    struct
    {
        gps_pos_t gps1Pos;
        
        struct device_runtime_test
        {
            /* data */
        };        

        struct
        {
            gga_nmea_history_t gga;
        } nmea;
    } m_hist = {};
    

};