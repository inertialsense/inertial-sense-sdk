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