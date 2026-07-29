/**
 * @file NtripCorrectionService.h
 * @brief CorrectionService specialization that connects to an NTRIP caster over TCP, negotiates
 * the mount-point request, and forwards the resulting RTCM3 correction stream.
 *
 * @author Kyle Mallory on 1/17/26.
 * @copyright Copyright (c) 2026 Inertial Sense, Inc. All rights reserved.
 */

#ifndef EVALTOOL_NTRIPCORRECTIONSERVICE_H
#define EVALTOOL_NTRIPCORRECTIONSERVICE_H

#include "ISConstants.h"
#include "CorrectionService.h"
#include "core/tcpPort.h"

/**
 * The NtripCorrectionService is a specialized CorrectionService which connects to a TCP port
 * and requests a particular "mount point" (as part of the URL), before entering a "streaming"
 * state where all data received over the port is forwarded to the consumer. Periodically,
 * it may be necessary to send back to the NTRIP caster, the current location of the base.
 * This may slightly complicate things, since a CorrectionService is a one-to-many feed. Its
 * generally understood that the many devices will share a similar (enough) position, but
 * its not a requirement, and if devices have significantly differing positions, it could
 * confuse the caster.
 */
class NtripCorrectionService : public CorrectionService {

    public:
        /**
         * Default empty constructor.
         */
        NtripCorrectionService() { setMessageStats(&srcStats); };

        /**
         * @brief typical constructor which creates the service and immediate connects to the requested URL
         * @param connectUrl the NTRIP caster URL to connect to, e.g. "ntrip://user:pass\@host:port/mountpoint"
         */
        NtripCorrectionService(const std::string& connectUrl) { setMessageStats(&srcStats); connect(connectUrl); }

        ~NtripCorrectionService() = default;

        /**
         * @brief Opens a TCP connection to the NTRIP caster and performs the NTRIP mount-point
         * request/negotiation. This does not itself override CorrectionService::packetTransformer()
         * -- NTRIP's TCP stream, once connected, carries RTCM3 directly, so the base class's normal
         * RTCM3 handling (onRtcm3Handler()) is sufficient once connect() succeeds.
         * @param connectUrl the NTRIP caster URL to connect to, e.g. "ntrip://user:pass\@host:port/mountpoint".
         *   Defaults to port 2101 (the standard NTRIP port) if the URL omits one.
         * @param userAgent the User-Agent string sent in the NTRIP request headers.
         * @return true if the connection and mount-point negotiation succeeded, false otherwise.
         */
        bool connect(const std::string& connectUrl, std::string userAgent = "NTRIP Inertial Sense");

        /**
         * @brief Reads/forwards corrections, and (re)establishes the caster connection when needed.
         * Overrides CorrectionService::step() because reconnecting an NTRIP source requires re-sending
         * the mount-point request after the TCP socket reconnects -- the base class only reopens the
         * socket, which would leave us connected to the caster but never streaming. The TCP (re)connect
         * itself is non-blocking (polled across successive step() calls); only the bounded mount-point
         * negotiation blocks briefly, once per (re)connection.
         * @return 0 if no packets processed, >0 number of packets processed, <0 on error / not connected.
         */
        int step() override;

        /**
         * @return true if there is an open connection/socket with the NTRIP server
         */
        bool isConnected() { return portIsOpened(source); }

        /**
         * @return true if at least one RTCM3 message has been received from the caster since connecting.
         */
        bool isReceivingCorrections() { return srcStats.rtcm3.size() > 0; }

        /**
         * @brief updates the GNSS position that is reported back to the NTRIP caster. Depending on the caster
         *   this may not be necessary, however many services will use the GNSS position to provide more accurate
         *   correction information (virtual stations), or route to physical stations nearer to the rover.
         * @param position a gnss_pos_t structure that will be sent to the caster. This is converted to the NMEA
         *   GGA sentence structure prior to sending to the caster.
         * @return true if the gnss_pos_t was successfully converted and sent, false otherwise.
         */
        bool updatePosition(const gnss_pos_t& position);

        /**
         * @brief updates the GNSS position that is reported back to the NTRIP caster. Depending on the caster
         *   this may not be necessary, however many services will use the GNSS position to provide more accurate
         *   correction information (virtual stations), or route to physical stations nearer to the rover.
         * @param nmeaGGA a NMEA GGA sentence which describes the position of the rover. This is sent verbatim to the caster.
         * @return true if the gnss_pos_t was successfully sent, false otherwise.
         */
        bool updatePosition(const std::string nmeaGGA);

        /**
         * Sets a set of additional headers which will be sent on the connection request to the NTRIP caster.
         * This must be called prior to calling connect().
         * @param hdrs a map of header names to header values.
         */
        void setConnectionRequestHeaders(std::map<std::string, std::string> hdrs);

    private:
        /**
         * @brief Sends the cached NTRIP mount-point request over the (already-open) source port and waits
         * for the caster's response. Shared by connect() (initial) and step() (reconnect). Requires the
         * source port to be open and requestMsg to have been built by a prior connect().
         * @return true if the request was sent and a response was received, false otherwise.
         */
        bool negotiateMountPoint();

        MessageStats::mul_stats_t srcStats;         //!< Per-protocol message statistics for the caster connection; CorrectionService only holds a pointer to stats, so this instance owns the storage.
        std::map<std::string, std::string> headers; //!< Custom headers which will be sent to the NTRIP caster when connecting
        std::string requestMsg;                      //!< Cached mount-point HTTP GET request, built in connect(), re-sent by negotiateMountPoint() on (re)connect
        std::string casterUrl;                       //!< Cached caster URL, retained for log messages on reconnect
};


#endif //EVALTOOL_NTRIPCORRECTIONSERVICE_H
