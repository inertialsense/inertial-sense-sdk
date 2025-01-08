#include <cstdint>
#include <chrono>
#include <ctime>
#include <deque>
#include "ISComm.h"
#include "time_conversion.h"

#define RUNTIME_TEST_MAX_LOG_SIZE   500000
#define SYS_TIME_NOW                std::chrono::system_clock::now()

typedef std::chrono::time_point<std::chrono::system_clock> system_time_t;

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
        sMsgHistory(uint32_t gpsTowMs_, uint32_t gpsWeek_, uint8_t *msg_ = NULL, int msgSize_ = 0)
        {
            localTime = SYS_TIME_NOW;
            gpsTowMs = gpsTowMs_;
            gpsWeek = gpsWeek_;
            msgSize = msgSize_;
            memcpy(msg, msg_, _MIN(msgSize_, MAX_MSG_LENGTH_NMEA));
        }
        sMsgHistory(gps_pos_t *gps, uint8_t *msg_ = NULL, int msgSize_ = 0)
        {
            localTime = SYS_TIME_NOW;
            gpsTowMs = gps->timeOfWeekMs;
            gpsWeek = gps->week;
            msgSize = msgSize_;
            memcpy(msg, msg_, _MIN(msgSize_, MAX_MSG_LENGTH_NMEA));
        }
        sMsgHistory(uint32_t gpsTowMs_, uint32_t gpsWeek_, p_data_hdr_t &dataHdr_, const uint8_t *dataBuf_)
        {
            localTime = SYS_TIME_NOW;
            gpsTowMs = gpsTowMs_;
            gpsWeek = gpsWeek_;
            dataHdr = dataHdr_;
            memcpy(msg, dataBuf_, _MIN(dataHdr.size, MAX_MSG_LENGTH_NMEA));
        }

        system_time_t   localTime;
        utc_date_t      date;
        utc_time_t      time;
        uint32_t        gpsTowMs;
        uint32_t        gpsWeek;
        uint8_t         msg[MAX_MSG_LENGTH_NMEA];
        int             msgSize = 0;
        p_data_hdr_t    dataHdr = {};
        bool            timeIrregular = false;    // used to prevent redundant error logging
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
    bool HasStatus(){ return m_status.size() != 0; }
    std::string Log()
    {
        std::string log = m_log;
        m_log.clear();
        return log; 
    }
    std::string Status()
    {
        std::string status = m_status;
        m_status.clear();
        return status; 
    }
    void SetPortName(std::string portName){ m_portName = portName; };
    void Enable(bool enable=true)
    { 
        m_enable = enable;
        LogEvent(SYS_TIME_NOW, (enable ? "Tests enabled..." : "Tests disabled..."));
    }
    void Verbose(bool enable=true){ m_verbose = enable; };

    struct error_count
    {
        int parse;
        int isbGpsTime;
        int nmeaGgaTime;
        int nmeaZdaTime;
    } m_errorCount = {};
    
private:
    std::deque<DeviceRuntimeTests::msg_history_t>& AddMsgHistory(std::deque<DeviceRuntimeTests::msg_history_t> &hist, DeviceRuntimeTests::msg_history_t msgHist);
    void TestIsbGps(const p_data_hdr_t &dataHdr, const uint8_t *dataBuf);
    void TestNmeaGga(const uint8_t* msg, int msgSize);
    void TestNmeaZda(const uint8_t* msg, int msgSize);
    void TestNmeaIntel(const uint8_t* msg, int msgSize);
    bool CheckGpsTime(const char* description, int &count, std::deque<msg_history_t> &hist);
    bool CheckGpsTimeDuplicate(const char* description, int &count, std::deque<msg_history_t> &hist);
    bool CheckGpsTimeReversed(const char* description, int &count, std::deque<msg_history_t> &hist);
    bool CheckGpsTimeIrregular(const char* description, int &count, std::deque<msg_history_t> &hist);
    void LogEvent(system_time_t time, std::string str);
    void LogEvent(system_time_t time, const char *format, ...);
    void WriteStatus(std::string str);
    void WriteStatus(const char *format, ...);
    std::string Timestamp(system_time_t time);

    std::string m_filename;
    std::string m_log;
    std::string m_status;
    bool m_enable = false;
    bool m_verbose = false;
    dev_info_t m_devInfo = {};
    std::string m_portName;

    RuntimeTest     m_testGgaDuplicate{"GGA Duplicate"};
    RuntimeTest     m_testZdaDuplicate{"ZDA Duplicate"};
    RuntimeTest     m_testGgaTimeReverse{"GGA Time Reverse"};
    RuntimeTest     m_testZdaTimeReverse{"ZDA Time Reverse"};
    RuntimeTest     m_testZdaTimeJump{"ZDA Time Jump"};

    struct
    {
        struct
        {
            std::deque<msg_history_t>   gps1Pos;
        } isb;

        struct
        {
            std::deque<msg_history_t>   gga;
            std::deque<msg_history_t>   intel;
            std::deque<msg_history_t>   zda;
        } nmea;
    } m_hist = {};
    

};
