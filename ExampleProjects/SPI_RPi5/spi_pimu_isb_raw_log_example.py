#!/usr/bin/env python3
"""
InertialSense SPI DID_PIMU Example — log to ISB_RAW (.raw) file
Raspberry Pi 5

Requests a continuous DID_PIMU broadcast over SPI using fixed-size polling
(Strategy A — no Data Ready pin required, same approach as
spi_pimu_example.py) and writes every raw byte received from the device into
an Inertial Sense ".raw" log file using the exact on-disk chunk format
produced by cISLogger::LOGTYPE_RAW (see cDeviceLogRaw::SaveData /
cDataChunk::WriteToFile in is-common/SDK/src/DeviceLogRaw.cpp and
is-common/SDK/src/DataChunk.cpp). The resulting file can be opened by the
desktop SDK's log reader / cltool / EvalTool exactly like a raw log captured
over UART or USB.

Note: because Strategy A has no Data Ready pin, the IMX clocks out 0x00
filler bytes whenever its transmit buffer is empty, and those filler bytes
are captured into the log verbatim along with real ISB packets (this mirrors
exactly what would be received on the wire). This is harmless — the parser
and any downstream tools simply skip bytes that aren't part of a valid ISB
packet — but it does make the .raw file larger than the DR-gated approach.

ISB_RAW chunk format (sChunkHeader, packed, little-endian, version 2):
    offset  size  field
    0       4     marker            0xFC05EA32
    4       1     version           2
    5       1     dataOffset        34  (bytes from here to start of data)
    6       2     protocolVersion   PROTOCOL_VERSION_CHAR0, CHAR1 (2, 2)
    8       4     name              4-char chunk tag, e.g. b'PDAT'
    12      4     invName           bitwise-NOT of each byte in `name`
    16      4     dataSize          uint32, bytes of raw data following hdr
    20      4     invDataSize       bitwise-NOT of dataSize
    24      4     grpNum            0 = serial data
    28      4     devSerialNum      device serial number (0 if unknown)
    32      2     portId            0xFFFF = unknown
    34      2     portType          0x0003 = PORT_TYPE__SPI
    36      4     fwVersion         device firmware version (0xFF... if unknown)
    -- sizeof(sChunkHeader) == 40 bytes --
    40      N     data              raw bytes exactly as received from device

A file is a sequence of these [header][data] chunks back to back; the last
chunk (on close) may be smaller than CHUNK_DATA_SIZE. This script chunks at
the same boundary cDeviceLogRaw uses: a chunk is flushed to disk once its
buffered data would exceed CHUNK_DATA_SIZE, and any remaining partial chunk
is flushed on exit (mirrors cDeviceLogRaw::CloseAllFiles() -> FlushToFile()).

ISB packet format (protocol version 2) carried inside the raw data:
    Bytes 0-1  : Preamble 0xEF 0x49
    Byte  2    : Flags  (lower 4 bits = packet type, upper 4 bits = flags)
    Byte  3    : Data ID in header (0 for command packets)
    Bytes 4-5  : Payload size (little-endian uint16)
    Bytes 6-N  : Payload
    Bytes N+1-2: Fletcher-16 checksum (little-endian, a=low byte, b=high byte)

Hardware connections (IMX <-> Raspberry Pi 5 40-pin header):
    IMX SPI_SCLK -> Pin 23  (GPIO11 / SPI0_CLK)
    IMX SPI_MOSI -> Pin 19  (GPIO10 / SPI0_MOSI)
    IMX SPI_MISO -> Pin 21  (GPIO9  / SPI0_MISO)
    IMX SPI_nCS  -> Pin 24  (GPIO8  / SPI0_CE0)
    IMX nSPI_EN  -> GND     (hold low at power-up to enable SPI)

Raspberry Pi 5 setup:
    1. Enable SPI0 in /boot/firmware/config.txt, then reboot:
           dtoverlay=spi0-1cs
    2. Install dependencies:
           sudo apt install python3-spidev
    3. Add your user to the spi group:
           sudo usermod -aG spi $USER   # log out and back in

Run:
    python3 spi_pimu_isb_raw_log_example.py [output_directory]
"""

import os
import sys
import math
import time
import struct
import select
import tty
import termios
import spidev

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------

SPI_BUS      = 0            # SPI bus number -> /dev/spidevBUS.DEVICE
SPI_DEVICE   = 0            # SPI device (chip-select index)
SPI_MODE     = 3            # CPOL=1, CPHA=1  required by the IMX
SPI_SPEED_HZ = 20_000_000   # 20 MHz; exceeds the documented 3 MHz max for
                            # Strategy A (no DR) — may cause corrupted/missed
                            # reads since the IMX SPI slave has no Data Ready
                            # handshake to throttle the host at this rate

SPI_READ_SIZE = 250         # bytes to read each poll tick

NAV_DT_MS       = 4         # IMX-6 nav period in ms (used for period calculation)
PIMU_PERIOD_MS  = 4         # requested DID_PIMU broadcast period (~250 Hz)
SEND_INTERVAL_S = 1.0       # how often to (re)send the GET_DATA command
POLL_INTERVAL_S = 0.003     # delay between SPI read ticks

LOG_DEVICE_SERIAL_NUM = 0   # set to the device's serial number if known (DID_DEV_INFO)
LOG_DIRECTORY_DEFAULT = "./IS_logs"

EXIT_KEY = "q"

# ---------------------------------------------------------------------------
# ISB protocol constants
# ---------------------------------------------------------------------------

ISB_PREAMBLE = bytes([0xEF, 0x49])     # PSC_ISB_PREAMBLE_BYTE1, BYTE2

PKT_TYPE_GET_DATA            = 3       # request a DID to be broadcast
PKT_TYPE_DATA                = 4       # response carrying a DID payload
PKT_TYPE_STOP_BROADCASTS_ALL = 6       # stop all broadcasts on all ports

DID_PIMU = 3                           # Preintegrated IMU (coning & sculling)

# pimu_t wire layout (40 bytes, little-endian):
#   double   time      offset  0   Time since boot (s)
#   float    dt        offset  8   Integration period (s)
#   uint32   status    offset 12   eImuStatus flags
#   float[3] theta     offset 16   Delta-theta (gyro integral, rad)
#   float[3] vel       offset 28   Delta-velocity (accel integral, m/s)
_PIMU_FMT = struct.Struct("<d f I 3f 3f")
PIMU_SIZE  = _PIMU_FMT.size            # 40

# ---------------------------------------------------------------------------
# ISB packet builder (see spi_pimu_dr_example.py for full description)
# ---------------------------------------------------------------------------

def _fletcher16(data: bytes) -> bytes:
    a = 0
    b = 0
    for byte in data:
        a = (a + byte) & 0xFF
        b = (b + a) & 0xFF
    return bytes([a, b])


def _build_packet(pkt_type: int, payload: bytes = b"") -> bytes:
    header = (
        ISB_PREAMBLE
        + bytes([pkt_type & 0x0F, 0x00])
        + struct.pack("<H", len(payload))
    )
    body = header + payload
    return body + _fletcher16(body)


def build_stop_broadcasts() -> bytes:
    return _build_packet(PKT_TYPE_STOP_BROADCASTS_ALL)


def build_get_data(did: int, period_ms: int) -> bytes:
    period = max(1, math.ceil(period_ms / NAV_DT_MS))
    payload = struct.pack("<HHHH", did, 0, 0, period)
    return _build_packet(PKT_TYPE_GET_DATA, payload)

# ---------------------------------------------------------------------------
# ISB packet parser
# ---------------------------------------------------------------------------

def parse_isb_packets(data: bytes):
    """Scan raw bytes for complete ISB packets. Yields (pkt_type, did, payload)."""
    i = 0
    while i < len(data):
        idx = data.find(ISB_PREAMBLE, i)
        if idx == -1:
            break
        i = idx

        if i + 8 > len(data):
            break

        flags        = data[i + 2]
        pkt_type     = flags & 0x0F
        did          = data[i + 3]
        payload_size = struct.unpack_from("<H", data, i + 4)[0]

        total = 6 + payload_size + 2
        if i + total > len(data):
            break

        payload = data[i + 6: i + 6 + payload_size]
        i += total

        yield pkt_type, did, payload

# ---------------------------------------------------------------------------
# ISB_RAW (.raw) chunked log writer — mirrors cDeviceLogRaw / cDataChunk
# ---------------------------------------------------------------------------

DATA_CHUNK_MARKER      = 0xFC05EA32
CHUNK_DATA_SIZE         = 128 * 1024     # DEFAULT_CHUNK_DATA_SIZE (DataChunk.h)
CHUNK_NAME              = b"PDAT"        # matches cDataChunk's default chunk name
PROTOCOL_VERSION_CHAR0  = 2
PROTOCOL_VERSION_CHAR1  = 2
PORT_TYPE__SPI          = 0x0003
PORT_ID_UNKNOWN         = 0xFFFF

# sChunkHeader (v2), packed 1, little-endian — 40 bytes total:
#   marker(I) version(B) dataOffset(B) protocolVersion(2s) name(4s) invName(4s)
#   dataSize(I) invDataSize(I) grpNum(I) devSerialNum(I) portId(H) portType(H) fwVersion(4s)
_CHUNK_HDR_FMT = struct.Struct("<I B B 2s 4s 4s I I I I H H 4s")
assert _CHUNK_HDR_FMT.size == 40


def _invert_bytes(b: bytes) -> bytes:
    return bytes((~c) & 0xFF for c in b)


class RawChunkLogger:
    """
    Buffers raw device bytes and writes them to disk in the same
    [sChunkHeader][data] layout cDeviceLogRaw::WriteChunkToFile() produces,
    flushing whenever the buffered data would exceed CHUNK_DATA_SIZE
    (mirrors cDeviceLogRaw::SaveData()'s GetBuffFree() check), so the
    resulting .raw file is interchangeable with one captured by the SDK.
    """

    def __init__(self, file_path: str, dev_serial_num: int = 0,
                 fw_version: bytes = b"\x00\x00\x00\xff"):
        self._f = open(file_path, "wb")
        self._buf = bytearray()
        self._dev_serial_num = dev_serial_num
        self._fw_version = fw_version
        self.bytes_written = 0
        self.chunks_written = 0

    def log_data(self, data: bytes) -> None:
        """Append raw bytes; flushes a full chunk to disk first if needed."""
        if len(self._buf) + len(data) > CHUNK_DATA_SIZE:
            self.flush()
        self._buf.extend(data)

    def flush(self) -> None:
        """Write any buffered data as one chunk (header + data) to disk."""
        if not self._buf:
            return

        data_size = len(self._buf)
        name     = CHUNK_NAME
        inv_name = _invert_bytes(name)

        header = _CHUNK_HDR_FMT.pack(
            DATA_CHUNK_MARKER,
            2,                                          # version
            34,                                         # dataOffset
            bytes([PROTOCOL_VERSION_CHAR0, PROTOCOL_VERSION_CHAR1]),
            name,
            inv_name,
            data_size,
            (~data_size) & 0xFFFFFFFF,                   # invDataSize
            0,                                            # grpNum (0 = serial data)
            self._dev_serial_num,
            PORT_ID_UNKNOWN,
            PORT_TYPE__SPI,
            self._fw_version,
        )

        self._f.write(header)
        self._f.write(self._buf)
        self.bytes_written += len(header) + data_size
        self.chunks_written += 1
        self._buf.clear()

    def close(self) -> None:
        self.flush()      # mirrors cDeviceLogRaw::CloseAllFiles() -> FlushToFile()
        self._f.close()

# ---------------------------------------------------------------------------
# Non-blocking keyboard helpers
# ---------------------------------------------------------------------------

def key_pressed() -> bool:
    return bool(select.select([sys.stdin], [], [], 0)[0])


def check_exit() -> bool:
    if key_pressed():
        return sys.stdin.read(1).lower() == EXIT_KEY
    return False

# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main() -> None:
    log_dir = sys.argv[1] if len(sys.argv) > 1 else LOG_DIRECTORY_DEFAULT
    os.makedirs(log_dir, exist_ok=True)

    timestamp = time.strftime("%Y%m%d_%H%M%S")
    log_path = os.path.join(
        log_dir, f"LOG_SN{LOG_DEVICE_SERIAL_NUM}_{timestamp}_0000.raw"
    )
    logger = RawChunkLogger(log_path, dev_serial_num=LOG_DEVICE_SERIAL_NUM)

    spi = spidev.SpiDev()
    spi.open(SPI_BUS, SPI_DEVICE)
    spi.max_speed_hz = SPI_SPEED_HZ
    spi.mode = SPI_MODE

    stop_pkt = build_stop_broadcasts()
    get_pkt  = build_get_data(DID_PIMU, PIMU_PERIOD_MS)

    saved_termios = termios.tcgetattr(sys.stdin)
    try:
        tty.setcbreak(sys.stdin.fileno())

        period_actual = math.ceil(PIMU_PERIOD_MS / NAV_DT_MS) * NAV_DT_MS
        print(f"InertialSense SPI DID_PIMU -> ISB_RAW log example  "
              f"|  /dev/spidev{SPI_BUS}.{SPI_DEVICE}  |  {SPI_SPEED_HZ // 1_000} kHz")
        print(f"Logging raw bytes to: {log_path}")
        print(f"Requested period: ~{period_actual} ms")
        print(f"Read block      : {SPI_READ_SIZE} bytes per poll tick")
        print(f"Press '{EXIT_KEY.upper()}' to stop and close the log.\n")

        # Clear any stale broadcasts left over from a previous session.
        spi.xfer2(list(stop_pkt))
        time.sleep(0.05)

        count     = 0
        last_send = time.monotonic() - SEND_INTERVAL_S

        while True:
            now = time.monotonic()

            # Periodically (re)send the GET_DATA broadcast request.
            if now - last_send >= SEND_INTERVAL_S:
                spi.xfer2(list(get_pkt))
                last_send = now

            # Read a fixed block of SPI bytes (Strategy A — no DR pin).
            # The IMX clocks out 0x00 when its buffer is empty; non-zero
            # bytes are parsed as ISB packets below. Every byte received,
            # filler included, is written into the ISB_RAW log, mirroring
            # cDeviceLogRaw::SaveData()'s pass-through of whatever bytes
            # arrived on the wire.
            rx_buf = bytes(spi.readbytes(SPI_READ_SIZE))
            logger.log_data(rx_buf)

            for pkt_type, did, payload in parse_isb_packets(rx_buf):
                if pkt_type != PKT_TYPE_DATA or did != DID_PIMU:
                    continue
                if len(payload) < PIMU_SIZE:
                    continue

                count += 1
                t_time, dt, _, *rest = _PIMU_FMT.unpack_from(payload)
                theta = rest[0:3]
                vel   = rest[3:6]

                print(
                    f"[{count:5d}]  t={t_time:10.3f} s  dt={dt:.4f} s  "
                    f"dTheta={theta[0]:9.5f}, {theta[1]:9.5f}, {theta[2]:9.5f} rad  "
                    f"dVel={vel[0]:9.5f}, {vel[1]:9.5f}, {vel[2]:9.5f} m/s  "
                    f"(log: {logger.bytes_written + len(logger._buf)} bytes, "
                    f"{logger.chunks_written} chunk(s) flushed)",
                    flush=True,
                )

            if check_exit():
                break

            time.sleep(POLL_INTERVAL_S)

    except KeyboardInterrupt:
        pass    # Ctrl+C falls through to finally

    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, saved_termios)
        spi.close()
        logger.close()
        print(f"\nExiting. Wrote {logger.bytes_written} bytes "
              f"({logger.chunks_written} chunk(s)) to {log_path}")


if __name__ == "__main__":
    main()
