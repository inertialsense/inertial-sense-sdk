/**
 * @file test_PortFactory.cpp
 * @brief SDK unit tests for the Port Factory
 *
 * @author Kyle Mallory on 10/20/2025
 * @copyright Copyright (c) 2025 Inertial Sense, Inc. All rights reserved.
 */

#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include <gtest/gtest.h>
#include "gtest_helpers.h"
#include "test_data_utils.h"

#include "core/types.h"
#include "core/base_port.h"
#include "core/tcpPort.h"
#include "PortFactory.h"
#include "TcpPortFactory.h"
#include "TcpServerPortFactory.h"
#include "PortManager.h"
#include "Rtcm3CorrectionServer.h"

// NOTE: must follow the SDK headers -- PLATFORM_IS_LINUX comes from ISConstants.h via those.
#if PLATFORM_IS_LINUX
#   include <unistd.h>
#   include <stdlib.h>
#   include <limits.h>
#   include <sys/stat.h>
#endif


// SN-8478: Rtcm3CorrectionServer/cltool silently succeeded when the RTCM3 listener failed to
// bind/listen. Force a real bind() failure and verify it's observable both via the pre-existing
// forensic accessor and via configure()'s (previously discarded) return value.
//
// The port is occupied by a plain, non-SDK socket rather than a second Rtcm3CorrectionServer:
// TcpServerPortFactory::startListening() unconditionally sets SO_REUSEADDR on Windows, which
// (unlike POSIX) is permissive enough to let a second SDK listener bind the same address:port
// that's already actively listening -- so two SDK listeners on one port doesn't reliably fail
// there. SO_EXCLUSIVEADDRUSE on the occupier blocks that regardless of the later socket's options.
TEST(test_PortFactory, tcpServerPortFactory_reportsListenFailure) {
    const int testPort = 14478; // arbitrary, distinct from other tests in this binary

#ifdef PLATFORM_IS_WINDOWS
    SOCKET occupierFd = socket(AF_INET, SOCK_STREAM, 0);
    ASSERT_NE(occupierFd, INVALID_SOCKET);
    int exclusive = 1;
    ASSERT_EQ(setsockopt(occupierFd, SOL_SOCKET, SO_EXCLUSIVEADDRUSE, (char*)&exclusive, sizeof(exclusive)), 0);
#else
    int occupierFd = socket(AF_INET, SOCK_STREAM, 0);
    ASSERT_GT(occupierFd, 0);
#endif

    sockaddr_in occupierAddr = {};
    occupierAddr.sin_family = AF_INET;
    occupierAddr.sin_addr.s_addr = htonl(INADDR_ANY);
    occupierAddr.sin_port = htons((uint16_t)testPort);
    ASSERT_EQ(bind(occupierFd, (sockaddr*)&occupierAddr, sizeof(occupierAddr)), 0);
    ASSERT_EQ(listen(occupierFd, 1), 0);

    Rtcm3CorrectionServer server(testPort, "127.0.0.1");
    auto listenError = server.getLastListenError();
    EXPECT_EQ(listenError.first, TCP_LISTEN_CTX__BIND)
        << "listener should fail at bind() -- the port is already occupied";
    EXPECT_NE(listenError.second, 0) << "a platform errno/WSAGetLastError() should be recorded";

    // configure() must propagate the same failure via its own return value now (SN-8478),
    // not leave it discoverable only via the separate getLastListenError() accessor.
    EXPECT_FALSE(server.configure(testPort, "127.0.0.1"));

#ifdef PLATFORM_IS_WINDOWS
    closesocket(occupierFd);
#else
    close(occupierFd);
#endif
}

TEST(test_PortFactory, tcpServerPortFactory) {
    
    port_handle_t clientPort = nullptr;

    TcpServerPortFactory serverFactory(4321);

    TEST_COUT << "Creating a TcpServerPortFactory on 127.0.0.1, listening for connections on port 4321." << std::endl;
    auto& pm = PortManager::getInstance();
    pm.clearPortFactories();
    pm.addPortFactory(&serverFactory);
    pm.addPortListener([](PortManager::port_event_e event, uint16_t pId, std::string pName, port_handle_t port, PortFactory& portFactory) {
        switch (event) {
            case PortManager::PORT_ADDED:
                TEST_COUT << "Received incoming connection request from " << pName << std::endl;
                break;
            case PortManager::PORT_REMOVED:
                break;
        }
    });

    // run a test for 5 seconds, every 1000ms (1sec) attempt to connect to the local port
    // test succeeds when the port is discovered, and fails if the test times-out.
    uint32_t timeout = current_timeMs() + 60000;
    uint32_t nextConnect = current_timeMs() + 1000;
    auto& clientFactory = TcpPortFactory::getInstance();
    while ((current_timeMs() < timeout) && (pm.size() == 0)) {
        if (current_timeMs() > nextConnect) {
            if (!clientPort) {
                TEST_COUT << "Creating tcpPort to connect to localhost." << std::endl;
                clientPort = clientFactory.bindPort("tcp://127.0.0.1:4321", PORT_TYPE__TCP | PORT_TYPE__COMM);
            }

            if (!clientPort) {
                SLEEP_MS(500);
            } else {
                // portClose(clientPort);
                portOpen(clientPort);   // make an attempt to connect
            }
        }

        pm.discoverPorts();
        SLEEP_MS(10);
    }

    EXPECT_GT(pm.size(), 0);

    // AT THIS POINT - Because we directly created/bound the clientPort from the factory, the PortManager does not know about it.
    // This means, that the PortManager should only know about the server-side port...  We can use this to test end-to-end connectivity
    auto serverPort = pm.getPort(0);    // also, note that we don't need to open a server port - its already opened when the client connects

    int sent = 0;       // number of messages sent
    int received = 0;   // number of lines received
    int matched = 0;    // number of received lines which matched the number of sent lines

    // let's send some data to the client, and then parse it from the client port
    TEST_COUT << "Sending 100 test-strings to the client; expecting 100 test-string to be received." << std::endl;
    for (int i = 0; i < 100; i++) {
        char buffer[64];    // a buffer to read into
        std::string outString = utils::string_format("Hello world! %02d\r\n", i);
        int bytes_sent = portWrite(serverPort, (const uint8_t*)outString.c_str(), outString.length());
        sent++;
        EXPECT_EQ(bytes_sent, outString.length());
        outString = outString.substr(0, outString.length() - 2); // we're not really interested in the newlines anymore, because portReadLineTimeout() strips them

        int bytes = portReadLineTimeout(clientPort, reinterpret_cast<unsigned char *>(buffer), sizeof(buffer), 100);
        if (bytes >= 0) received++;
        if (bytes == outString.length()) {
            std::string tmp(buffer, bytes);
            if (tmp == outString) {
                matched++;
            }
        }
    }

    EXPECT_EQ(sent, received);
    EXPECT_EQ(sent, matched);

    portClose(clientPort);
    clientFactory.releasePort(clientPort);

    portClose(serverPort);
    serverFactory.releasePort(serverPort);
}

// SN-8473: SerialPortFactory::getComPorts() is now the single OS-enumeration implementation for
// serial ports (previously duplicated in the now-deprecated cISSerialPort::GetComPorts()). This is
// a basic coverage test -- it can't assert on which ports exist (that's host-hardware-dependent,
// and CI runners typically have none), only that enumeration runs cleanly and the returned count
// is consistent with the populated vector.
TEST(test_PortFactory, serialPortFactory_getComPorts) {
    std::vector<std::string> portNames;
    int count = SerialPortFactory::getComPorts(portNames);
    EXPECT_EQ(count, (int)portNames.size());
    EXPECT_GE(count, 0);
}

// ---------------------------------------------------------------------------------------------
// SN-8575: InertialSense::Open() failed for symlinked serial ports on Linux.
//
// A udev rule carrying SYMLINK+="imx5" yields /dev/imx5 -> ttyACM0, but port discovery enumerates
// canonical kernel names from /sys/class/tty and validation keys its sysfs lookup on that same
// canonical name. Given an alias, both rejected a device that open(2) handles without complaint.
// SerialPortFactory::resolvePortName() resolves the alias; these tests cover it in three tiers:
//
//   1. the resolution logic itself, hermetically -- no device and no root needed
//   2. non-regression on pattern semantics, which is where the risk in the change actually sits
//   3. end-to-end against a real tty, skipped when the host has none
// ---------------------------------------------------------------------------------------------

#if PLATFORM_IS_LINUX

namespace {

/** Private temp directory for one test; removes its recorded entries and itself on destruction. */
class TempDir {
public:
    TempDir() {
        char tmpl[] = "/tmp/sn8575_portfactory_XXXXXX";
        const char* d = mkdtemp(tmpl);
        if (d != nullptr)
            path_ = d;
    }

    ~TempDir() {
        // Reverse order so a chain's intermediate links go before what they point at.
        for (auto it = entries_.rbegin(); it != entries_.rend(); ++it)
            unlink(it->c_str());
        if (!path_.empty())
            rmdir(path_.c_str());
    }

    TempDir(const TempDir&) = delete;
    TempDir& operator=(const TempDir&) = delete;

    bool valid() const { return !path_.empty(); }
    const std::string& path() const { return path_; }

    /** symlink(target, <dir>/name). @return the link's path, or "" on failure. */
    std::string link(const std::string& name, const std::string& target) {
        std::string p = path_ + "/" + name;
        if (symlink(target.c_str(), p.c_str()) != 0)
            return "";
        entries_.push_back(p);
        return p;
    }

    /** Creates a small regular file. @return its path. */
    std::string file(const std::string& name) {
        std::string p = path_ + "/" + name;
        std::ofstream(p) << "not-a-device";
        entries_.push_back(p);
        return p;
    }

private:
    std::string path_;
    std::vector<std::string> entries_;
};

/**
 * realpath() of @p p. Used for expectations rather than comparing against the path we constructed,
 * because /tmp is itself a symlink on some distributions -- in which case resolvePortName() would
 * (correctly) return a path that differs from the one the test built.
 */
std::string realOf(const std::string& p) {
    char buf[PATH_MAX] = {};
    return (realpath(p.c_str(), buf) != nullptr) ? std::string(buf) : std::string();
}

/** Every port name SerialPortFactory emits for @p pattern. */
std::vector<std::string> located(const std::string& pattern) {
    std::vector<std::string> names;
    SerialPortFactory::getInstance().locatePorts(
        [&names](PortFactory*, uint16_t, std::string name) { names.push_back(name); },
        pattern, PORT_TYPE__UART);
    return names;
}

} // namespace


// --- Tier 1: resolution logic, hermetic -------------------------------------------------------

// udev writes a RELATIVE target: a SYMLINK+="imx5" rule produces /dev/imx5 -> ttyACM0, NOT
// -> /dev/ttyACM0. This is the form observed on hardware for SN-8575, and the reason
// resolvePortName() uses realpath() rather than readlink() -- the latter yields a bare name that
// would need a "/dev/" prefix guessed back onto it.
TEST(test_PortFactory, resolvePortName_resolvesRelativeSymlink) {
    TempDir tmp;
    ASSERT_TRUE(tmp.valid());

    const std::string target = tmp.file("ttyFAKE0");
    const std::string link = tmp.link("imx5", "ttyFAKE0");     // relative, as udev writes it
    ASSERT_FALSE(link.empty());

    EXPECT_EQ(SerialPortFactory::resolvePortName(link), realOf(target));
}

TEST(test_PortFactory, resolvePortName_resolvesAbsoluteSymlink) {
    TempDir tmp;
    ASSERT_TRUE(tmp.valid());

    const std::string target = tmp.file("ttyFAKE0");
    const std::string link = tmp.link("imx5", target);         // absolute target
    ASSERT_FALSE(link.empty());

    EXPECT_EQ(SerialPortFactory::resolvePortName(link), realOf(target));
}

// A multi-hop chain must collapse to the final device, not to the next link.
TEST(test_PortFactory, resolvePortName_resolvesChainedSymlinks) {
    TempDir tmp;
    ASSERT_TRUE(tmp.valid());

    const std::string target = tmp.file("ttyFAKE0");
    ASSERT_FALSE(tmp.link("middle", "ttyFAKE0").empty());
    const std::string link = tmp.link("imx5", "middle");
    ASSERT_FALSE(link.empty());

    EXPECT_EQ(SerialPortFactory::resolvePortName(link), realOf(target));
}

// Anything that is not a symlink comes back byte-identical, so existing callers see no change.
TEST(test_PortFactory, resolvePortName_leavesNonSymlinkUnchanged) {
    TempDir tmp;
    ASSERT_TRUE(tmp.valid());

    const std::string regular = tmp.file("ttyFAKE0");
    EXPECT_EQ(SerialPortFactory::resolvePortName(regular), regular);
}

// A dangling link is left alone deliberately: the caller's own existence checks should reject it,
// rather than resolvePortName() inventing a path for a device that isn't there.
TEST(test_PortFactory, resolvePortName_leavesDanglingSymlinkUnchanged) {
    TempDir tmp;
    ASSERT_TRUE(tmp.valid());

    const std::string link = tmp.link("imx5", "no_such_target");
    ASSERT_FALSE(link.empty());

    EXPECT_EQ(SerialPortFactory::resolvePortName(link), link);
}

TEST(test_PortFactory, resolvePortName_leavesNonexistentPathUnchanged) {
    const std::string missing = "/dev/sn8575_definitely_does_not_exist";
    EXPECT_EQ(SerialPortFactory::resolvePortName(missing), missing);
}

// The property that keeps pattern-based discovery working: callers pass regexes through the same
// argument as literal paths, and a regex never names a file, so the lstat() gate declines it.
TEST(test_PortFactory, resolvePortName_leavesRegexPatternsUnchanged) {
    const std::vector<std::string> patterns = {
        "*",                            // cltool's all-ports token
        "(.+)",                         // PortManager::discoverPorts()'s own default
        ".*",                           // what globToRegex() turns "*" into
        "/dev/tty(ACM|USB)[0-9]+",      // a genuine regex over device names
        "/dev/ttyACM0,/dev/ttyUSB0",    // a comma-separated list
        "",                             // empty
    };
    for (const auto& p : patterns)
        EXPECT_EQ(SerialPortFactory::resolvePortName(p), p) << "pattern was mangled: " << p;
}


// --- Tier 2: non-regression on pattern semantics ----------------------------------------------

// The wildcard reaches locatePorts() by two different routes and must behave the same as before on
// both: verbatim from OpenPorts() as "*", which is an INVALID regex handled by locatePorts()'s
// regex_error fallback, and as ".*" from cltool via utils::globToRegex(). Both must agree with the
// explicit match-everything pattern. Correct (if weak) on a host with no serial ports at all.
TEST(test_PortFactory, locatePorts_wildcardsStillMatchAllPorts) {
    const std::vector<std::string> all = located("(.+)");
    TEST_COUT << "host enumerates " << all.size() << " serial port(s)" << std::endl;

    EXPECT_EQ(located("*"), all);      // invalid regex -> fallback path
    EXPECT_EQ(located(".*"), all);     // globToRegex("*")
}

TEST(test_PortFactory, locatePorts_nonMatchingPatternYieldsNothing) {
    EXPECT_TRUE(located("/dev/sn8575_no_such_port_[0-9]+").empty());
}


// --- Tier 3: end-to-end against a real tty, skipped when the host has none --------------------

// The whole bug, start to finish: alias a real port, then assert both gates that used to reject it
// now accept it. validatePort() covers the bindPort() entry point (which never goes through
// locatePorts()), and locatePorts() covers Open() and CorrectionService.
TEST(test_PortFactory, symlinkedPort_isValidatedAndLocated) {
    const std::vector<std::string> all = located("(.+)");
    if (all.empty())
        GTEST_SKIP() << "no serial ports on this host; nothing to alias";

    const std::string canonical = all.front();
    TEST_COUT << "aliasing " << canonical << std::endl;

    TempDir tmp;
    ASSERT_TRUE(tmp.valid());
    const std::string link = tmp.link("imx5", canonical);
    ASSERT_FALSE(link.empty());

    // Pre-fix both of these failed: validatePort() built /sys/class/tty/imx5/... (missing), and
    // locatePorts() regex-matched the alias against canonical names it could never equal.
    EXPECT_TRUE(SerialPortFactory::getInstance().validatePort(link, PORT_TYPE__UART));

    const std::vector<std::string> viaAlias = located(link);
    ASSERT_EQ(viaAlias.size(), 1u);
    EXPECT_EQ(viaAlias.front(), canonical);   // reported under its canonical name, not the alias
}

#endif // PLATFORM_IS_LINUX
