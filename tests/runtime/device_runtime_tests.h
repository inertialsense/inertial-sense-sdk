#include <cstdint>
#include "ISComm.h"
#include "time_conversion.h"


class RuntimeTest
{
public:
    RuntimeTest(std::string name) 
    { 
        m_name = name; 
    }

    std::string     m_name;
    int             m_failCount;
};


class DeviceRuntimeTests
{
    typedef struct sMsgHistory
    {
        void update(uint32_t gpsTowMs_, uint32_t gpsWeek_, uint8_t *msg_, int msgSize_)
        {
            gpsTowMs = gpsTowMs_;
            gpsWeek = gpsWeek_;
            memcpy(msg, msg_, _MIN(msgSize_, MAX_MSG_LENGTH_NMEA));
            msgSize = msgSize_;
        }

        utc_date_t date;
        utc_time_t time;
        uint32_t gpsTowMs;
        uint32_t gpsWeek;
        uint8_t msg[MAX_MSG_LENGTH_NMEA];
        int msgSize;
    } msg_history_t;

public:
    DeviceRuntimeTests();
    std::string CreateLogFilename(const std::string path, int serialNumber=0);
    void ProcessParseError(is_comm_instance_t &comm);
    void ProcessISB(const p_data_hdr_t &dataHdr, const uint8_t *dataBuf);
    void ProcessNMEA(const uint8_t* msg, int msgSize);
    int ErrorCount() 
    {
        return  m_errorCount.parse +
                m_errorCount.nmeaGgaTime +
                m_errorCount.nmeaZdaTime;
    }
    bool HasLog(){ return m_log.size() != 0; }
    std::string Log()
    {
        std::string log = m_log;
        m_log.clear();
        return log; 
    }
    void SetPortName(std::string portName){ m_portName = portName; };

    struct error_count
    {
        int parse;
        int nmeaGgaTime;
        int nmeaZdaTime;
    } m_errorCount = {};
    
private:
    void TestNmeaGga(const uint8_t* msg, int msgSize);
    void TestNmeaZda(const uint8_t* msg, int msgSize);
    bool CheckGpsDuplicate(const char* description, int &count, uint32_t towMs, uint32_t gpsWeek, const uint8_t* msg, int msgSize, msg_history_t &hist);
    bool CheckGpsTimeReverse(const char* description, int &count, uint32_t towMs, uint32_t gpsWeek, const uint8_t* msg, int msgSize, msg_history_t &hist);
    void LogEvent(std::string str);
    void LogEvent(const char *format, ...);
    std::string Timestamp();

    std::string m_filename;
    std::string m_log;
    bool m_enable = true;
    dev_info_t m_devInfo = {};
    std::string m_portName;

    RuntimeTest     m_testGgaDuplicate{"GGA Duplicate"};
    RuntimeTest     m_testZdaDuplicate{"ZDA Duplicate"};
    RuntimeTest     m_testGgaTimeReverse{"GGA Time Reverse"};
    RuntimeTest     m_testZdaTimeReverse{"ZDA Time Reverse"};

    struct
    {
        gps_pos_t           gps1Pos;
        
        struct
        {
            msg_history_t   zda;
            msg_history_t   gga;
        } nmea;
    } m_hist = {};
    

};
