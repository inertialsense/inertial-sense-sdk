#include <cstdint>
#include "ISComm.h"


class DeviceRuntimeTest
{
public:

    void ProcessRaw(uint8_t *data, int dataSize);
    void ProcessISB(const p_data_hdr_t &dataHdr, const uint8_t *dataBuf);
    void ProcessNMEA(const uint8_t* msg, int msgSize);

private:
    void TestNmeaGga(const uint8_t* msg, int msgSize);
    void LogErrorNMEA(const uint8_t* msg, int msgSize, const char *format, ...);

    bool m_enable = true;

    struct
    {
        gps_pos_t gpsPos;
        
        struct
        {
            struct 
            {
                gps_pos_t gpsPos;
            } gga;
        } nmea;
    } m_hist = {};
    

};