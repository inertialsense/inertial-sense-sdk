#include <gtest/gtest.h>
#include <deque>
#include <string>
#include "InertialSense.h"
#include "TcpPortFactory.h"

// NOTE: must follow the SDK headers -- PLATFORM_IS_LINUX comes from ISConstants.h via those.
#if !PLATFORM_IS_WINDOWS
#   include <arpa/inet.h>
#   include <netinet/in.h>
#   include <sys/select.h>
#   include <sys/socket.h>
#   include <unistd.h>
#endif


TEST(InertialSense, General)
{
//     InertialSense inertialSenseInterface();

    EXPECT_TRUE(true);
}


namespace {

#if PLATFORM_IS_WINDOWS
typedef SOCKET test_socket_t;
static const test_socket_t TEST_INVALID_SOCKET = INVALID_SOCKET;
static void closeTestSocket(test_socket_t s) { closesocket(s); }
#else
typedef int test_socket_t;
static const test_socket_t TEST_INVALID_SOCKET = -1;
static void closeTestSocket(test_socket_t s) { close(s); }
#endif

// A plain (non-SDK) TCP listener on 127.0.0.1, standing in for a device that doesn't speak ISB.  It never answers device
// validation, and records every byte the host sends it.
class FakeTcpDevice
{
public:
    FakeTcpDevice()
    {
        m_listen = socket(AF_INET, SOCK_STREAM, 0);
        sockaddr_in addr = {};
        addr.sin_family = AF_INET;
        addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
        addr.sin_port = 0;      // let the OS pick a free port
        socklen_t len = sizeof(addr);
        if (m_listen == TEST_INVALID_SOCKET ||
            bind(m_listen, (sockaddr*)&addr, sizeof(addr)) != 0 ||
            listen(m_listen, 1) != 0 ||
            getsockname(m_listen, (sockaddr*)&addr, &len) != 0)
            return;
        m_port = ntohs(addr.sin_port);
    }

    ~FakeTcpDevice()
    {
        if (m_conn != TEST_INVALID_SOCKET)      closeTestSocket(m_conn);
        if (m_listen != TEST_INVALID_SOCKET)    closeTestSocket(m_listen);
    }

    int port() const { return m_port; }
    std::string url() const { return "tcp://127.0.0.1:" + std::to_string(m_port); }

    /** @return true if the host connected within timeoutMs. */
    bool accept(int timeoutMs)
    {
        if (!readable(m_listen, timeoutMs))
            return false;
        m_conn = ::accept(m_listen, nullptr, nullptr);
        return m_conn != TEST_INVALID_SOCKET;
    }

    bool send(const std::string& data) { return ::send(m_conn, data.c_str(), (int)data.size(), 0) == (int)data.size(); }

    /** @return everything the host has sent, waiting up to timeoutMs for the first byte. */
    std::string received(int timeoutMs)
    {
        std::string out;
        char buf[256];
        while (readable(m_conn, timeoutMs))
        {
            int n = (int)recv(m_conn, buf, sizeof(buf), 0);
            if (n <= 0)
                break;
            out.append(buf, n);
            timeoutMs = 0;
        }
        return out;
    }

private:
    static bool readable(test_socket_t s, int timeoutMs)
    {
        fd_set fds;
        FD_ZERO(&fds);
        FD_SET(s, &fds);
        timeval tv = { timeoutMs / 1000, (timeoutMs % 1000) * 1000 };
        return select((int)s + 1, &fds, nullptr, nullptr, &tv) > 0;
    }

    test_socket_t m_listen = TEST_INVALID_SOCKET;
    test_socket_t m_conn = TEST_INVALID_SOCKET;
    int m_port = 0;
};

// PortManager, DeviceManager and the global com manager outlive the InertialSense instance (SN-8660), and Open() leaves
// the com manager calling back into it.  Release and reset them so the next test starts clean.
void releaseAllPortsAndDevices()
{
    DeviceManager::getInstance().clear(true);
    auto& pm = PortManager::getInstance();
    std::vector<port_handle_t> ports(pm.begin(), pm.end());
    for (auto port : ports)
        pm.releasePort(port);
    pm.clear();
    pm.clearPortFactories();
    DeviceManager::getInstance().clearDeviceFactories();

    comManagerInit(NULL, 10, 0, 0, 0, 0, 0);
    for (int ptype : {_PTYPE_NMEA, _PTYPE_UBLOX, _PTYPE_RTCM3, _PTYPE_SPARTN, _PTYPE_SEPTENTRIO_SBF, _PTYPE_SEPTENTRIO_REPLY})
        comManagerRegisterProtocolHandler(ptype, NULL);
}

// Accepts any device, like cltool's CltoolDeviceFactory.  The default ImxDeviceFactory only allocates for a known IMX
// hardware type, which an unvalidated port can't report.
DeviceFactory& anyDeviceFactory()
{
    static DeviceFactory factory;
    return factory;
}

TcpPortFactory& tcpPortFactory()
{
    TcpPortFactory& tpf = TcpPortFactory::getInstance();
    tpf.portOptions.defaultBlocking = false;    // as cltool configures it
    return tpf;
}

} // namespace


// SN-8678/SN-8896: With device validation disabled (cltool -lm, -vd, and listen-only -nmea), Open() binds a device to
// each port without probing it, and validation is what normally opens a port.  cltool then calls connect() on each
// device, which must open the port and -- since the far end may not speak ISB at all -- send nothing to it.
TEST(InertialSense, ValidationDisabled_ConnectOpensTcpPortWithoutSendingAnything)
{
    releaseAllPortsAndDevices();
    FakeTcpDevice fake;
    ASSERT_NE(fake.port(), 0) << "could not create the local TCP listener";

    {
        InertialSense is({&tcpPortFactory()}, {&anyDeviceFactory()});
        is.EnableDeviceValidation(false);
        ASSERT_TRUE(is.Open(fake.url().c_str()));

        ASSERT_EQ(is.DeviceCount(), 1);
        device_handle_t device = is.getDevices().front();
        EXPECT_TRUE(device->validationDisabled());
        EXPECT_TRUE(device->connect());
        EXPECT_TRUE(device->isConnected());
        EXPECT_TRUE(is.IsOpen());
        ASSERT_TRUE(fake.accept(1000)) << "the host never connected to the fake device";

        // Data from the device reaches the host
        const std::string nmea = "$GNGGA,test,line*50\r\n";
        ASSERT_TRUE(fake.send(nmea));
        bool updated = true;
        uint32_t endMs = current_timeMs() + 300;
        while (current_timeMs() < endMs)
        {
            updated &= is.Update();
            SLEEP_MS(10);
        }
        EXPECT_TRUE(updated) << "Update() reported the device disconnected";
        port_stats_t* stats = portStats(device->port);
        if (stats)
            EXPECT_GE(stats->rxBytes, nmea.size());

        // ...and the host sent nothing: no validation probes, no stop-broadcast
        EXPECT_EQ(fake.received(200), "");

        // SN-8896: cltool -vd then asks the unvalidated device to stream data, which must reach it
        is.BroadcastBinaryData(DID_INS_1, 10);
        EXPECT_NE(fake.received(500), "") << "the data request was never sent";

        is.Close();
    }
    releaseAllPortsAndDevices();
}

// The same path when the port can't be opened: connect() must give up within its timeout rather than hang, and must
// not report the device connected.
TEST(InertialSense, ValidationDisabled_ConnectToRefusedTcpPortFailsWithoutHanging)
{
    releaseAllPortsAndDevices();
    int refusedPort;
    {
        FakeTcpDevice unused;       // take a free port number, then close the listener so connecting is refused
        refusedPort = unused.port();
    }
    ASSERT_NE(refusedPort, 0);

    {
        InertialSense is({&tcpPortFactory()}, {&anyDeviceFactory()});
        is.EnableDeviceValidation(false);
        is.Open(("tcp://127.0.0.1:" + std::to_string(refusedPort)).c_str());
        uint32_t startMs = current_timeMs();
        for (auto device : is.getDevices())
        {
            EXPECT_FALSE(device->connect());
            EXPECT_FALSE(device->isConnected());
        }
        EXPECT_LT(current_timeMs() - startMs, 2000u);
        EXPECT_FALSE(is.IsOpen());
        is.Close();
    }
    releaseAllPortsAndDevices();
}
