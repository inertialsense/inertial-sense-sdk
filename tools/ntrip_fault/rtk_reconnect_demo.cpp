// Minimal end-to-end reproducer for the Inertial Sense SDK NTRIP/RTK reconnect behaviour.
//
// Runs a fake NTRIP caster on 127.0.0.1, points an NtripCorrectionService at it, injects one
// network fault, and reports whether the correction stream ever comes back. No GNSS receiver, no
// antenna, no root, no outside network.
//
// See README.md for the scenarios and their expected output.

#include <inertial-sense-sdk/ISComm.h>
#include <inertial-sense-sdk/NtripCorrectionService.h>
#include <inertial-sense-sdk/core/tcpPort.h>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace {

using clock_type = std::chrono::steady_clock;
using std::chrono::duration_cast;
using std::chrono::milliseconds;

enum Fault { FAULT_NONE, FAULT_RESET, FAULT_FIN, FAULT_STALL, FAULT_EPIPE };

const char* faultName(Fault f)
{
    switch (f) {
        case FAULT_RESET: return "reset";
        case FAULT_FIN: return "fin";
        case FAULT_STALL: return "stall";
        case FAULT_EPIPE: return "epipe";
        default: return "none";
    }
}

clock_type::time_point g_start;

double elapsed()
{
    return duration_cast<milliseconds>(clock_type::now() - g_start).count() / 1000.0;
}

void logLine(const char* fmt, ...) __attribute__((format(printf, 1, 2)));

void logLine(const char* fmt, ...)
{
    char msg[512];
    va_list args;
    va_start(args, fmt);
    vsnprintf(msg, sizeof(msg), fmt, args);
    va_end(args);
    printf("[%7.2fs] %s\n", elapsed(), msg);
    fflush(stdout);
}

// -------------------------------------------------------------------------------------------------
// RTCM3 frame construction
// -------------------------------------------------------------------------------------------------

/// Builds one syntactically valid RTCM3 frame: 0xD3, 6 reserved bits, a 10-bit payload length, the
/// payload, then a 24-bit CRC-24Q over everything before it. The SDK's parser validates exactly
/// this framing and the CRC, so the payload contents past the 12-bit message number are irrelevant
/// to whether the frame is accepted and counted.
std::vector<uint8_t> buildRtcm3Frame(uint16_t messageNumber, uint16_t payloadLength)
{
    std::vector<uint8_t> frame;
    frame.reserve(payloadLength + 6);
    frame.push_back(0xD3);
    frame.push_back(static_cast<uint8_t>((payloadLength >> 8) & 0x03));
    frame.push_back(static_cast<uint8_t>(payloadLength & 0xFF));

    frame.push_back(static_cast<uint8_t>(messageNumber >> 4));
    frame.push_back(static_cast<uint8_t>((messageNumber & 0x0F) << 4));
    for (uint16_t i = 2; i < payloadLength; ++i) frame.push_back(static_cast<uint8_t>(i));

    const unsigned int crc = calculate24BitCRCQ(frame.data(), frame.size());
    frame.push_back(static_cast<uint8_t>((crc >> 16) & 0xFF));
    frame.push_back(static_cast<uint8_t>((crc >> 8) & 0xFF));
    frame.push_back(static_cast<uint8_t>(crc & 0xFF));
    return frame;
}

// -------------------------------------------------------------------------------------------------
// Fake NTRIP caster
// -------------------------------------------------------------------------------------------------

/// A single-client NTRIP 1.0 caster on the loopback interface. Accepts a client, answers the
/// mount-point request with "ICY 200 OK", then streams RTCM3 until told to misbehave. After a fault
/// it goes back to accepting, so a client that recovers correctly gets served again.
///
/// It runs on its own thread because the mount-point request is only written from inside
/// NtripCorrectionService::connect()/step(), which then blocks on the reply: driving both sides from
/// one thread deadlocks.
class FakeCaster {
public:
    FakeCaster()
    {
        listenFd_ = ::socket(AF_INET, SOCK_STREAM, 0);
        if (listenFd_ < 0) die("socket");

        int reuse = 1;
        ::setsockopt(listenFd_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

        sockaddr_in addr = {};
        addr.sin_family = AF_INET;
        addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
        addr.sin_port = 0;
        if (::bind(listenFd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0) die("bind");
        if (::listen(listenFd_, 4) != 0) die("listen");

        socklen_t len = sizeof(addr);
        if (::getsockname(listenFd_, reinterpret_cast<sockaddr*>(&addr), &len) != 0)
            die("getsockname");
        port_ = ntohs(addr.sin_port);
    }

    ~FakeCaster()
    {
        stop();
        if (listenFd_ >= 0) ::close(listenFd_);
    }

    uint16_t port() const { return port_; }
    int handshakes() const { return handshakes_.load(); }
    long framesSent() const { return framesSent_.load(); }

    void start() { thread_ = std::thread([this] { run(); }); }

    void stop()
    {
        stop_ = true;
        if (thread_.joinable()) thread_.join();
        closeClientFd(false);
    }

    void requestFault(Fault f) { faultRequest_ = f; }

private:
    [[noreturn]] static void die(const char* what)
    {
        perror(what);
        exit(2);
    }

    void run()
    {
        auto lastFrame = clock_type::now();
        while (!stop_) {
            // A client that reconnects shows up as a new connection, and a real caster serves it
            // and abandons the old one. Checking for an inbound connection on every pass matters
            // for the stall scenario, where the previous connection is still open: a caster that
            // only accepted when it had no client would never serve the reconnect.
            if (clientFd_ < 0 || connectionPending()) {
                closeClientFd(false);
                if (!acceptClient(100)) continue;
                if (!serveHandshake()) {
                    closeClientFd(false);
                    continue;
                }
                ++handshakes_;
                stalled_ = false;
                logLine("caster: served mount-point request #%d", handshakes_.load());
            }

            const Fault f = faultRequest_.exchange(FAULT_NONE);
            if (f == FAULT_RESET || f == FAULT_EPIPE) {
                // EPIPE needs the peer fully gone while the client still believes it may write, so
                // both of these tear the connection down; they differ only in what the demo does
                // next (see runDemo()).
                const bool rst = (f == FAULT_RESET);
                logLine("caster: dropping the connection (%s)", rst ? "RST" : "FIN, then gone");
                closeClientFd(rst);
                continue;
            }
            if (f == FAULT_FIN) {
                logLine("caster: closing the connection cleanly (FIN)");
                closeClientFd(false);
                continue;
            }
            if (f == FAULT_STALL) {
                logLine("caster: keeping the socket open but sending no more corrections");
                stalled_ = true;
            }

            drainClient();

            if (!stalled_
                && duration_cast<milliseconds>(clock_type::now() - lastFrame) >= frameInterval_) {
                sendFrame();
                lastFrame = clock_type::now();
            }
            std::this_thread::sleep_for(milliseconds(10));
        }
    }

    bool connectionPending()
    {
        pollfd pfd = {listenFd_, POLLIN, 0};
        return ::poll(&pfd, 1, 0) > 0;
    }

    bool acceptClient(int timeoutMs)
    {
        pollfd pfd = {listenFd_, POLLIN, 0};
        if (::poll(&pfd, 1, timeoutMs) <= 0) return false;
        clientFd_ = ::accept(listenFd_, nullptr, nullptr);
        return clientFd_ >= 0;
    }

    bool serveHandshake()
    {
        std::string request;
        for (int waited = 0; waited < 2000; waited += 20) {
            char buf[1024];
            const ssize_t n = ::recv(clientFd_, buf, sizeof(buf), MSG_DONTWAIT);
            if (n > 0) request.append(buf, static_cast<size_t>(n));
            if (request.find("\r\n\r\n") != std::string::npos) break;
            std::this_thread::sleep_for(milliseconds(20));
        }
        if (request.find("\r\n\r\n") == std::string::npos) return false;

        static const char kOk[] = "ICY 200 OK\r\n\r\n";
        return ::send(clientFd_, kOk, sizeof(kOk) - 1, MSG_NOSIGNAL) > 0;
    }

    /// Consumes whatever the client sends (NMEA GGA position updates) so its writes keep succeeding
    /// and the behaviour under test is the reconnect logic, not a filled socket buffer.
    void drainClient()
    {
        char buf[1024];
        while (::recv(clientFd_, buf, sizeof(buf), MSG_DONTWAIT) > 0) {}
    }

    void sendFrame()
    {
        const std::vector<uint8_t> frame = buildRtcm3Frame(1005, 19);
        if (::send(clientFd_, frame.data(), frame.size(), MSG_NOSIGNAL)
            == static_cast<ssize_t>(frame.size()))
            ++framesSent_;
    }

    /// @param withReset send a RST instead of a FIN. SO_LINGER with a zero timeout makes close()
    ///        discard any queued data and reset the connection, which is what a carrier-grade NAT
    ///        or a restarted caster looks like from the client's side.
    void closeClientFd(bool withReset)
    {
        if (clientFd_ < 0) return;
        if (withReset) {
            linger lin = {1, 0};
            ::setsockopt(clientFd_, SOL_SOCKET, SO_LINGER, &lin, sizeof(lin));
        }
        ::close(clientFd_);
        clientFd_ = -1;
    }

    int listenFd_ = -1;
    int clientFd_ = -1;
    uint16_t port_ = 0;
    std::thread thread_;
    std::atomic<bool> stop_{false};
    std::atomic<bool> stalled_{false};
    std::atomic<Fault> faultRequest_{FAULT_NONE};
    std::atomic<int> handshakes_{0};
    std::atomic<long> framesSent_{0};
    milliseconds frameInterval_{200};
};

// -------------------------------------------------------------------------------------------------
// Observing the port
// -------------------------------------------------------------------------------------------------

/// Reads the raw socket descriptor the SDK is holding. A healthy open port has a non-negative
/// descriptor; a negative value that is not -EBADF (-9) is an errno the SDK stored in place of the
/// descriptor, which is the defect this program is built to show.
int rawSocket(port_handle_t port)
{
    return port ? TCP_PORT(port)->socket : 0;
}

std::string describeSocket(int fd)
{
    if (fd >= 0) return std::to_string(fd);
    const int err = -fd;
    return std::to_string(fd) + " (-" + strerrorname_np(err) + ")";
}

/// The kernel's own view of the connection. A positive descriptor says nothing about whether the
/// connection is up: a reconnect whose SYN is being dropped also has a perfectly valid descriptor,
/// and sits in SYN_SENT until the link returns. Without this, that case is indistinguishable from
/// a healthy connection that has simply gone quiet.
std::string describeTcpState(int fd)
{
    if (fd < 0) return "-";

    struct tcp_info info = {};
    socklen_t len = sizeof(info);
    if (::getsockopt(fd, IPPROTO_TCP, TCP_INFO, &info, &len) != 0) return "?";

    static const char* const names[] = {
      "?",         "ESTABLISHED", "SYN_SENT",  "SYN_RECV",  "FIN_WAIT1", "FIN_WAIT2",
      "TIME_WAIT", "CLOSE",       "CLOSE_WAIT", "LAST_ACK", "LISTEN",    "CLOSING"};
    if (info.tcpi_state >= sizeof(names) / sizeof(names[0])) return "?";
    return names[info.tcpi_state];
}

/// Prints the keep-alive options actually set on the live correction socket. Upstream sets none;
/// the fix sets all four. This is the cheapest way to confirm which library got linked, and it
/// needs no fault at all.
void reportKeepAlive(port_handle_t port)
{
    const int fd = rawSocket(port);
    if (fd < 0) {
        logLine("keepalive: no open socket to inspect");
        return;
    }

    struct Option {
        const char* name;
        int level;
        int option;
    };
    const Option options[] = {
      {"SO_KEEPALIVE", SOL_SOCKET, SO_KEEPALIVE},
      {"TCP_KEEPIDLE", IPPROTO_TCP, TCP_KEEPIDLE},
      {"TCP_KEEPINTVL", IPPROTO_TCP, TCP_KEEPINTVL},
      {"TCP_KEEPCNT", IPPROTO_TCP, TCP_KEEPCNT},
      {"TCP_USER_TIMEOUT", IPPROTO_TCP, TCP_USER_TIMEOUT},
    };

    std::string summary;
    for (const Option& o : options) {
        int value = -1;
        socklen_t len = sizeof(value);
        if (::getsockopt(fd, o.level, o.option, &value, &len) != 0) value = -1;
        summary += std::string(summary.empty() ? "" : " ") + o.name + "=" + std::to_string(value);
    }
    logLine("keepalive: %s", summary.c_str());
}

// -------------------------------------------------------------------------------------------------
// Rover position
// -------------------------------------------------------------------------------------------------

/// A fixed NMEA GGA sentence with a correct XOR checksum. Real VRS mount points stop streaming
/// unless the rover keeps reporting where it is, and these writes are also what exercises the
/// SDK's send path during a fault.
std::string buildGgaSentence()
{
    const std::string body = "GPGGA,120000.00,5005.0000,N,01425.0000,E,1,12,0.80,200.0,M,45.0,M,,";
    uint8_t checksum = 0;
    for (char c : body) checksum ^= static_cast<uint8_t>(c);

    char out[160];
    snprintf(out, sizeof(out), "$%s*%02X\r\n", body.c_str(), checksum);
    return out;
}

// -------------------------------------------------------------------------------------------------
// Options
// -------------------------------------------------------------------------------------------------

struct Options {
    Fault fault = FAULT_RESET;
    double faultAtSec = 3.0;
    double durationSec = 25.0;
    double positionPeriodSec = 5.0;
    double tailSec = 8.0;
    long watchdogMs = 30000;
    std::string caster;
    bool casterFromEnv = false;
};

void printUsage(const char* argv0)
{
    printf(
      "Usage: %s [options]\n"
      "\n"
      "  --fault=reset|fin|stall|epipe|none   network fault to inject (default: reset)\n"
      "  --fault-at=<seconds>                 when to inject it (default: 3)\n"
      "  --duration=<seconds>                 total run time (default: 25)\n"
      "  --position-period=<seconds>          how often to send NMEA GGA (default: 5)\n"
      "  --tail=<seconds>                     corrections must arrive within this much of the end\n"
      "                                       of the run to count as recovered (default: 8)\n"
      "  --watchdog=<milliseconds>            close a silent stream after this long, 0 to\n"
      "                                       disable (default: 30000)\n"
      "  --caster=ntrip://user:pass@host:port/MOUNT\n"
      "                                       use a real caster instead of the built-in fake one;\n"
      "                                       inject the fault yourself (see README). Can also be\n"
      "                                       given in RTK_DEMO_CASTER, which keeps real\n"
      "                                       credentials out of the process command line.\n"
      "  --help                               this message\n"
      "\n"
      "Exit status: 0 recovered, 1 did not recover, 2 bad usage or no connection, 4 the caster\n"
      "never streamed anything so nothing was tested. Recovery means corrections were still\n"
      "arriving at the end of the run; check the reported longest silence to confirm the fault\n"
      "actually interrupted the stream in the first place.\n",
      argv0);
}

bool parseOptions(int argc, char** argv, Options& opt)
{
    if (const char* fromEnv = getenv("RTK_DEMO_CASTER")) {
        opt.caster = fromEnv;
        opt.casterFromEnv = true;
    }

    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        const size_t eq = arg.find('=');
        const std::string key = arg.substr(0, eq);
        const std::string value = (eq == std::string::npos) ? "" : arg.substr(eq + 1);

        if (key == "--help") {
            printUsage(argv[0]);
            exit(0);
        } else if (key == "--fault") {
            if (value == "reset") opt.fault = FAULT_RESET;
            else if (value == "fin") opt.fault = FAULT_FIN;
            else if (value == "stall") opt.fault = FAULT_STALL;
            else if (value == "epipe") opt.fault = FAULT_EPIPE;
            else if (value == "none") opt.fault = FAULT_NONE;
            else {
                fprintf(stderr, "unknown fault `%s`\n", value.c_str());
                return false;
            }
        } else if (key == "--fault-at") {
            opt.faultAtSec = atof(value.c_str());
        } else if (key == "--duration") {
            opt.durationSec = atof(value.c_str());
        } else if (key == "--position-period") {
            opt.positionPeriodSec = atof(value.c_str());
        } else if (key == "--tail") {
            opt.tailSec = atof(value.c_str());
        } else if (key == "--watchdog") {
            opt.watchdogMs = atol(value.c_str());
        } else if (key == "--caster") {
            opt.caster = value;
            opt.casterFromEnv = false;
        } else {
            fprintf(stderr, "unknown option `%s`\n", arg.c_str());
            return false;
        }
    }
    return true;
}

// -------------------------------------------------------------------------------------------------
// The scenario
// -------------------------------------------------------------------------------------------------

struct Counters {
    std::atomic<long> rtcm{0};
};

int runDemo(const Options& opt)
{
    Counters counters;
    FakeCaster* caster = nullptr;
    std::string url = opt.caster;

    if (url.empty()) {
        caster = new FakeCaster();
        url = "ntrip://user:pass@127.0.0.1:" + std::to_string(caster->port()) + "/MOUNT";
        caster->start();
        logLine("fake caster listening on 127.0.0.1:%u", caster->port());
    } else if (opt.casterFromEnv) {
        // Easy to leave exported in a shell and then wonder why the fake caster stopped reacting
        // to faults, so it is called out rather than mentioned in passing.
        logLine("using the real caster from RTK_DEMO_CASTER; faults must be injected externally");
    } else {
        logLine("using the real caster from --caster; faults must be injected externally");
    }

    NtripCorrectionService ntrip;
    ntrip.addRTCM3PacketListeners(
      [&counters](uint16_t, const void*, uint32_t) { ++counters.rtcm; });

    logLine("connecting ...");
    if (!ntrip.connect(url)) {
        logLine(
          "FAILED to connect: %s",
          ntrip.getLastResultString().empty() ? "(no detail)"
                                              : ntrip.getLastResultString().c_str());
        delete caster;
        return 2;
    }
    logLine("connected, negotiating done");
    reportKeepAlive(ntrip.getSourcePort());

    const std::string gga = buildGgaSentence();
    auto lastPosition = clock_type::now() - std::chrono::hours(1);
    auto lastReport = clock_type::now() - std::chrono::hours(1);
    long lastWatchdogLog = 0;

    bool faultInjected = false;
    long rtcmAtFault = 0;

    // The verdict is taken from the end of the run, not from the first frames that show up after
    // the fault. Frames already in flight when the fault lands arrive milliseconds later and say
    // nothing about reconnection: an earlier version of this program counted them and cheerfully
    // reported a recovery for a run that stayed dead for the next two minutes.
    long lastSeenRtcm = 0;
    double lastFrameAt = 0.0;
    double longestGap = 0.0;
    double resumedAt = 0.0;

    while (elapsed() < opt.durationSec) {
        if (!faultInjected && elapsed() >= opt.faultAtSec && opt.fault != FAULT_NONE) {
            faultInjected = true;
            rtcmAtFault = counters.rtcm.load();
            logLine("--- injecting fault: %s ---", faultName(opt.fault));
            if (caster) {
                caster->requestFault(opt.fault);
            } else {
                logLine("inject the fault on the real caster path now (see README)");
            }

            if (opt.fault == FAULT_EPIPE) {
                // EPIPE is only reachable while the SDK still holds a valid descriptor: once a read
                // has observed the error, tcpPortWrite() returns early on the negative descriptor
                // and never reaches its EPIPE branch. So write a few times without stepping and let
                // the peer's RST land between two of them. Which error surfaces first is a race, so
                // both outcomes are reported rather than asserted.
                std::this_thread::sleep_for(milliseconds(200));
                for (int i = 0; i < 10; ++i) {
                    ntrip.updatePosition(gga);
                    const int fd = rawSocket(ntrip.getSourcePort());
                    logLine(
                      "  write %d: socket=%s opened=%d", i + 1, describeSocket(fd).c_str(),
                      portIsOpened(ntrip.getSourcePort()) ? 1 : 0);
                    if (fd < 0) break;
                    std::this_thread::sleep_for(milliseconds(100));
                }
            }
        }

        ntrip.step();

        if (duration_cast<milliseconds>(clock_type::now() - lastPosition)
            >= milliseconds(static_cast<long>(opt.positionPeriodSec * 1000))) {
            lastPosition = clock_type::now();
            if (ntrip.isConnected()) ntrip.updatePosition(gga);
        }

        // The SDK has no stall detection of its own, so a caster that holds the socket open and
        // stops streaming can only be recovered by the application closing the port. Gated on
        // isReceivingCorrections() because getLastRtcm3PacketAge() is measured against the UNIX
        // epoch and is therefore enormous before the first packet ever arrives.
        if (opt.watchdogMs > 0 && ntrip.isConnected() && ntrip.isReceivingCorrections()
            && ntrip.getLastRtcm3PacketAge() > static_cast<uint32_t>(opt.watchdogMs)) {
            const long now = static_cast<long>(elapsed());
            if (now != lastWatchdogLog) {
                lastWatchdogLog = now;
                logLine(
                  "watchdog: no corrections for %ums, closing the port so step() rebuilds it",
                  ntrip.getLastRtcm3PacketAge());
            }
            portClose(ntrip.getSourcePort());
        }

        const long rtcmNow = counters.rtcm.load();
        if (rtcmNow != lastSeenRtcm) {
            const double now = elapsed();
            if (faultInjected) {
                const double gap = now - lastFrameAt;
                if (gap > longestGap) {
                    longestGap = gap;
                    resumedAt = now;
                    if (gap > 1.0) logLine("*** corrections resumed after %.1fs of silence", gap);
                }
            }
            lastSeenRtcm = rtcmNow;
            lastFrameAt = now;
        }

        if (duration_cast<milliseconds>(clock_type::now() - lastReport) >= milliseconds(1000)) {
            lastReport = clock_type::now();
            const port_handle_t port = ntrip.getSourcePort();
            const int fd = rawSocket(port);
            const uint32_t age = ntrip.getLastRtcm3PacketAge();
            char ageText[32] = "-";
            if (rtcmNow > 0) snprintf(ageText, sizeof(ageText), "%.1fs", age / 1000.0);
            logLine(
              "opened=%d valid=%d socket=%-18s %-11s connected=%d rtcm=%ld age=%s",
              portIsOpened(port) ? 1 : 0, portIsValid(port) ? 1 : 0, describeSocket(fd).c_str(),
              describeTcpState(fd).c_str(), ntrip.isConnected() ? 1 : 0, rtcmNow, ageText);
        }

        std::this_thread::sleep_for(milliseconds(10));
    }

    const double finishedAt = elapsed();
    const double silentFor = finishedAt - lastFrameAt;
    const bool flowingAtEnd = silentFor < opt.tailSec;

    printf("\n");
    printf("fault injected .......... %s\n", faultName(opt.fault));
    printf("mount-point handshakes .. %d\n", caster ? caster->handshakes() : -1);
    printf("rtcm3 packets received .. %ld (%ld before the fault)\n", counters.rtcm.load(),
           rtcmAtFault);
    printf("longest silence ......... %.1fs\n", longestGap > silentFor ? longestGap : silentFor);
    printf("last correction ......... %.1fs before the end of the run\n", silentFor);
    printf("port still claims open .. %s\n", portIsOpened(ntrip.getSourcePort()) ? "yes" : "no");
    printf("raw socket descriptor ... %s\n", describeSocket(rawSocket(ntrip.getSourcePort())).c_str());
    printf("tcp state ............... %s\n", describeTcpState(rawSocket(ntrip.getSourcePort())).c_str());

    int status = 0;
    if (counters.rtcm.load() == 0) {
        // Not a verdict about the library: nothing was ever measured. A caster can accept the
        // connection and then stream nothing at all, which otherwise looks exactly like a port
        // that failed to recover.
        printf(
          "\nRESULT: NO CORRECTIONS AT ALL, nothing was tested. Check the credentials, the mount\n"
          "point, whether the GGA position is inside the caster's network, and whether the account\n"
          "is over its session limit.\n");
        status = 4;
    } else if (opt.fault == FAULT_NONE) {
        printf("\nRESULT: no fault requested, %ld corrections received\n", counters.rtcm.load());
        status = counters.rtcm.load() > 0 ? 0 : 1;
    } else if (!flowingAtEnd) {
        printf(
          "\nRESULT: DID NOT RECOVER, no corrections for the last %.0fs of the run\n", silentFor);
        status = 1;
    } else {
        // Deliberately not reported as a single "recovered in N seconds": when the stream is fast
        // and the reconnect quick, every gap is the same size as normal jitter and picking one of
        // them to call the recovery would be inventing precision.
        printf(
          "\nRESULT: RECOVERED, corrections still flowing at the end of the run "
          "(longest silence %.1fs, ending at T+%.1fs)\n",
          longestGap, resumedAt);
    }

    if (caster) {
        caster->stop();
        delete caster;
    }
    return status;
}

}   // namespace

int main(int argc, char** argv)
{
    g_start = clock_type::now();

    Options opt;
    if (!parseOptions(argc, argv, opt)) return 2;

    printf("Inertial Sense SDK RTK reconnect reproducer\n");
    printf(
      "fault=%s at=%.1fs duration=%.1fs position-period=%.1fs watchdog=%ldms\n\n",
      faultName(opt.fault), opt.faultAtSec, opt.durationSec, opt.positionPeriodSec, opt.watchdogMs);

    return runDemo(opt);
}
