/**
 * @file tcp_test_helpers.h
 * @brief Loopback helpers shared by the TCP port lifecycle tests (descriptor counting, a one-shot listener, raw peers).
 */

#ifndef IS_SDK_TCP_TEST_HELPERS_H
#define IS_SDK_TCP_TEST_HELPERS_H

#if !defined(_WIN32)

#include <arpa/inet.h>
#include <dirent.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <string>

namespace tcp_test {

/** @return the number of file descriptors currently open in this process. */
inline int countOpenFds() {
    int count = 0;
    if (DIR* dir = opendir("/proc/self/fd")) {
        while (readdir(dir)) count++;
        closedir(dir);
    }
    return count;
}

/** @return true if @p fd is an open descriptor in this process. */
inline bool fdIsOpen(int fd) { return fcntl(fd, F_GETFD) != -1; }

/** Loopback listener on an ephemeral port, standing in for a remote TCP server (or, via connectTo(), a remote client). */
class Listener {
public:
    Listener() {
        fd = socket(AF_INET, SOCK_STREAM, 0);
        sockaddr_in addr = {};
        addr.sin_family = AF_INET;
        addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
        addr.sin_port = 0;
        bind(fd, (sockaddr*)&addr, sizeof(addr));
        listen(fd, 16);
        socklen_t len = sizeof(addr);
        getsockname(fd, (sockaddr*)&addr, &len);
        port = ntohs(addr.sin_port);
    }
    ~Listener() { if (fd >= 0) close(fd); }
    Listener(const Listener&) = delete;
    Listener& operator=(const Listener&) = delete;

    std::string url() const { return "tcp://127.0.0.1:" + std::to_string(port); }

    /** @return the accepted peer's socket (waiting up to @p timeoutMs), or -1 */
    int acceptPeer(int timeoutMs = 2000) {
        pollfd pfd = { fd, POLLIN, 0 };
        if (poll(&pfd, 1, timeoutMs) != 1) return -1;
        return accept(fd, nullptr, nullptr);
    }

    int fd = -1;
    uint16_t port = 0;
};

/** @return a blocking socket connected to 127.0.0.1:@p port, or -1 */
inline int connectTo(uint16_t port) {
    int s = socket(AF_INET, SOCK_STREAM, 0);
    sockaddr_in addr = {};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    addr.sin_port = htons(port);
    if (connect(s, (sockaddr*)&addr, sizeof(addr)) != 0) { close(s); return -1; }
    return s;
}

/** Closes @p fd with an RST (SO_LINGER 0) instead of a FIN, so the other end sees ECONNRESET. */
inline void closeWithReset(int fd) {
    linger lg = { 1, 0 };
    setsockopt(fd, SOL_SOCKET, SO_LINGER, &lg, sizeof(lg));
    close(fd);
}

/** @return true if the peer socket @p fd reports EOF (orderly close) within @p timeoutMs. */
inline bool peerSeesEof(int fd, int timeoutMs = 1000) {
    pollfd pfd = { fd, POLLIN, 0 };
    if (poll(&pfd, 1, timeoutMs) != 1) return false;
    char c;
    return recv(fd, &c, 1, 0) == 0;
}

} // namespace tcp_test

#endif // !_WIN32
#endif // IS_SDK_TCP_TEST_HELPERS_H
