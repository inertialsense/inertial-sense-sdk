#!/usr/bin/env python3
"""
InertialSense SPI DID_PIMU Example - Raspberry Pi 5

Sends PKT_TYPE_GET_DATA for DID_PIMU once per second (Strategy A — fixed-size
polling, no Data Ready pin required), reads a fixed block of SPI bytes each
loop tick, parses any ISB DATA packets that arrive, and prints the pimu_t
fields.  Runs continuously until the user presses Q.

ISB packet format used (protocol version 2):
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
    python3 spi_pimu_example.py
"""

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
SPI_SPEED_HZ = 1_000_000   # 1 MHz; Strategy A (no DR) is limited to 3 MHz max

SPI_READ_SIZE = 512         # bytes to read each poll tick — large enough to
                            # hold several PIMU packets (one packet = 48 bytes)

NAV_DT_MS     = 7          # IMX-5 nav period in ms; adjust to 4 for IMX-6
PIMU_PERIOD_MS = 1000       # desired DID_PIMU broadcast period in ms (~1 Hz)
SEND_INTERVAL_S = 1.0       # how often to (re)send the GET_DATA command (seconds)

EXIT_KEY = "q"              # press this key (case-insensitive) to quit

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
# ISB packet builder
# ---------------------------------------------------------------------------

def _fletcher16(data: bytes) -> bytes:
    """
    Fletcher-16 checksum as implemented by the IS SDK (is_comm_fletcher16).
    Returns 2 bytes: [a, b] where a = low byte, b = high byte.
    Covers the entire packet including the header.
    """
    a = 0
    b = 0
    for byte in data:
        a = (a + byte) & 0xFF
        b = (b + a) & 0xFF
    return bytes([a, b])


def _build_packet(pkt_type: int, payload: bytes = b"") -> bytes:
    """Assemble a complete ISB packet with Fletcher-16 checksum."""
    header = (
        ISB_PREAMBLE
        + bytes([pkt_type & 0x0F, 0x00])    # flags byte, id byte (0 for cmds)
        + struct.pack("<H", len(payload))    # payload size
    )
    body = header + payload
    return body + _fletcher16(body)


def build_stop_broadcasts() -> bytes:
    """PKT_TYPE_STOP_BROADCASTS_ALL_PORTS — no payload."""
    return _build_packet(PKT_TYPE_STOP_BROADCASTS_ALL)


def build_get_data(did: int, period_ms: int) -> bytes:
    """
    PKT_TYPE_GET_DATA — requests 'did' to be broadcast every ~period_ms ms.

    The period field is a multiple of NAV_DT_MS.  We round up so the output
    rate never exceeds the requested rate.

    p_data_get_t payload layout (8 bytes, little-endian uint16 each):
        id      — Data ID being requested
        size    — byte count (0 = full structure)
        offset  — byte offset (0 = start of structure)
        period  — period multiple of nav DT
    """
    period = max(1, math.ceil(period_ms / NAV_DT_MS))
    payload = struct.pack("<HHHH", did, 0, 0, period)
    return _build_packet(PKT_TYPE_GET_DATA, payload)

# ---------------------------------------------------------------------------
# ISB packet parser
# ---------------------------------------------------------------------------

def parse_isb_packets(data: bytes):
    """
    Scan raw bytes for complete ISB packets.  Yields (pkt_type, did, payload).
    Silently skips malformed or truncated packets.
    """
    i = 0
    while i < len(data):
        idx = data.find(ISB_PREAMBLE, i)
        if idx == -1:
            break
        i = idx

        if i + 8 > len(data):           # need at least 6-byte header + 2-byte cksum
            break

        flags        = data[i + 2]
        pkt_type     = flags & 0x0F
        did          = data[i + 3]
        payload_size = struct.unpack_from("<H", data, i + 4)[0]

        total = 6 + payload_size + 2    # header + payload + checksum
        if i + total > len(data):
            break

        payload = data[i + 6: i + 6 + payload_size]
        i += total

        yield pkt_type, did, payload

# ---------------------------------------------------------------------------
# Non-blocking keyboard helpers
# ---------------------------------------------------------------------------

def key_pressed() -> bool:
    """Return True if a character is waiting on stdin."""
    return bool(select.select([sys.stdin], [], [], 0)[0])


def check_exit() -> bool:
    """Consume one character from stdin and return True if it is the exit key."""
    if key_pressed():
        return sys.stdin.read(1).lower() == EXIT_KEY
    return False

# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main() -> None:
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
        print(f"InertialSense SPI DID_PIMU Example  "
              f"|  /dev/spidev{SPI_BUS}.{SPI_DEVICE}  "
              f"|  {SPI_SPEED_HZ // 1_000} kHz")
        print(f"STOP_BROADCASTS : {stop_pkt.hex(' ').upper()}")
        print(f"GET_DATA packet : {get_pkt.hex(' ').upper()}")
        print(f"Requested period: ~{period_actual} ms  "
              f"({math.ceil(PIMU_PERIOD_MS / NAV_DT_MS)} x {NAV_DT_MS} ms nav DT)")
        print(f"Read block      : {SPI_READ_SIZE} bytes per poll tick")
        print(f"Press '{EXIT_KEY.upper()}' to exit.\n")

        # Stop any broadcasts left over from a previous session.
        print("Sending STOP_BROADCASTS ...", flush=True)
        spi.xfer2(list(stop_pkt))
        time.sleep(0.05)            # brief pause so the device processes the command

        print(f"Sending GET_DATA for DID_PIMU @ ~{period_actual} ms ...\n", flush=True)

        count     = 0
        last_send = time.monotonic() - SEND_INTERVAL_S  # trigger immediately on first tick

        while True:
            now = time.monotonic()

            # ------------------------------------------------------------------
            # Send GET_DATA every SEND_INTERVAL_S seconds regardless of whether
            # the device has responded.
            # ------------------------------------------------------------------
            if now - last_send >= SEND_INTERVAL_S:
                spi.xfer2(list(get_pkt))
                last_send = now

            # ------------------------------------------------------------------
            # Read a fixed block of SPI bytes (Strategy A — no DR pin).
            # The IMX clocks out 0x00 when its buffer is empty; non-zero bytes
            # are parsed as ISB packets below.
            # ------------------------------------------------------------------
            rx = spi.readbytes(SPI_READ_SIZE)

            for pkt_type, did, payload in parse_isb_packets(bytes(rx)):
                if pkt_type != PKT_TYPE_DATA or did != DID_PIMU:
                    continue
                if len(payload) < PIMU_SIZE:
                    print(f"  [!] Short payload ({len(payload)} < {PIMU_SIZE} bytes), skipping")
                    continue

                count += 1
                t_time, dt, _, *rest = _PIMU_FMT.unpack_from(payload)
                theta = rest[0:3]
                vel   = rest[3:6]

                print(
                    f"[{count:4d}]  "
                    f"t={t_time:10.3f} s  "
                    f"dt={dt:.4f} s  "
                    f"dTheta={theta[0]:9.5f}, {theta[1]:9.5f}, {theta[2]:9.5f} rad  "
                    f"dVel={vel[0]:9.5f}, {vel[1]:9.5f}, {vel[2]:9.5f} m/s",
                    flush=True,
                )

            if check_exit():
                break

            time.sleep(0.001)

    except KeyboardInterrupt:
        pass    # Ctrl+C handled by the finally block

    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, saved_termios)
        spi.close()
        print("\nExiting.")


if __name__ == "__main__":
    main()
