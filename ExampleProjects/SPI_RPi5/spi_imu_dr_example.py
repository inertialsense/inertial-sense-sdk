#!/usr/bin/env python3
"""
InertialSense SPI DID_IMU Example — key-triggered, Data Ready pin
Raspberry Pi 5

Press 'R' to request one DID_IMU sample (rate-limited to once per second).
The request uses period=0 (single shot, no repeating broadcast).  After each
request the loop waits for the Data Ready GPIO pin to assert HIGH, then reads
SPI bytes in chunks until DR de-asserts LOW or 250 ms elapses, parses any ISB
DATA packets, and prints the imu_t fields.  Press 'Q' to quit.

ISB packet format (protocol version 2):
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
    IMX DR       -> Pin 22  (GPIO25)              <-- Data Ready, active HIGH

Raspberry Pi 5 setup:
    1. Enable SPI0 in /boot/firmware/config.txt, then reboot:
           dtoverlay=spi0-1cs
    2. Install dependencies:
           sudo apt install python3-spidev python3-lgpio
    3. Add your user to the spi and gpio groups:
           sudo usermod -aG spi,gpio $USER   # log out and back in

Run:
    python3 spi_imu_dr_example.py
"""

import sys
import time
import struct
import select
import tty
import termios
import spidev
import lgpio

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------

SPI_BUS      = 0            # SPI bus number -> /dev/spidevBUS.DEVICE
SPI_DEVICE   = 0            # SPI device (chip-select index)
SPI_MODE     = 3            # CPOL=1, CPHA=1  required by the IMX
SPI_SPEED_HZ = 1_000_000   # 1 MHz

GPIO_CHIP    = 4            # /dev/gpiochip4 on Raspberry Pi 5
DR_GPIO_LINE = 25           # GPIO25 (Pin 22) — IMX Data Ready, active HIGH

SPI_CHUNK_SIZE  = 64        # bytes per SPI read while DR is asserted
DR_WAIT_S       = 0.250     # max time to wait for DR to assert after a request
DR_READ_S       = 0.250     # max time to spend reading while DR is asserted

REQUEST_MIN_S   = 1.0       # minimum interval between requests (1 Hz max)

EXIT_KEY    = "q"
REQUEST_KEY = "r"

# ---------------------------------------------------------------------------
# ISB protocol constants
# ---------------------------------------------------------------------------

ISB_PREAMBLE = bytes([0xEF, 0x49])     # PSC_ISB_PREAMBLE_BYTE1, BYTE2

PKT_TYPE_GET_DATA            = 3       # request a DID
PKT_TYPE_DATA                = 4       # response carrying a DID payload
PKT_TYPE_STOP_BROADCASTS_ALL = 6       # stop all broadcasts on all ports

DID_IMU = 58                           # IMU (gyro + accel, compensated)

# imu_t wire layout (36 bytes, little-endian / PACKED):
#   double    time      offset  0   Time since boot (s)
#   uint32    status    offset  8   eImuStatus flags
#   float[3]  pqr       offset 12   Angular rate (rad/s)
#   float[3]  acc       offset 24   Acceleration (m/s^2)
_IMU_FMT = struct.Struct("<d I 3f 3f")
IMU_SIZE  = _IMU_FMT.size              # 36

# ---------------------------------------------------------------------------
# ISB packet builder
# ---------------------------------------------------------------------------

def _fletcher16(data: bytes) -> bytes:
    """Fletcher-16 checksum (is_comm_fletcher16). Returns [a, b] low/high."""
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


def build_get_data_once(did: int) -> bytes:
    """
    PKT_TYPE_GET_DATA with period=0 — requests a single one-shot response.

    p_data_get_t payload layout (8 bytes, little-endian uint16 each):
        id      — Data ID being requested
        size    — byte count (0 = full structure)
        offset  — byte offset (0 = start of structure)
        period  — 0 = single shot
    """
    payload = struct.pack("<HHHH", did, 0, 0, 0)
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


def read_key() -> str | None:
    """Return the next key from stdin (lowercase), or None if none pending."""
    if key_pressed():
        return sys.stdin.read(1).lower()
    return None

# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main() -> None:
    spi = spidev.SpiDev()
    spi.open(SPI_BUS, SPI_DEVICE)
    spi.max_speed_hz = SPI_SPEED_HZ
    spi.mode = SPI_MODE

    gpio_h = lgpio.gpiochip_open(GPIO_CHIP)
    lgpio.gpio_claim_input(gpio_h, DR_GPIO_LINE)

    stop_pkt     = build_stop_broadcasts()
    get_pkt      = build_get_data_once(DID_IMU)
    last_request = -REQUEST_MIN_S       # allow immediate first request
    count        = 0

    saved_termios = termios.tcgetattr(sys.stdin)
    try:
        tty.setcbreak(sys.stdin.fileno())

        print(f"InertialSense SPI DID_IMU Example  (DR pin, key-triggered)  "
              f"|  /dev/spidev{SPI_BUS}.{SPI_DEVICE}  "
              f"|  {SPI_SPEED_HZ // 1_000} kHz  "
              f"|  DR = GPIO{DR_GPIO_LINE}")
        print(f"GET_DATA packet : {get_pkt.hex(' ').upper()}  (period=0, single shot)")
        print(f"Press '{REQUEST_KEY.upper()}' to request DID_IMU  |  "
              f"Press '{EXIT_KEY.upper()}' to exit.\n")

        # Clear any stale broadcasts left over from a previous session.
        spi.xfer2(list(stop_pkt))
        time.sleep(0.05)

        while True:
            key = read_key()

            if key == EXIT_KEY:
                break

            if key == REQUEST_KEY:
                now = time.monotonic()
                elapsed = now - last_request
                if elapsed < REQUEST_MIN_S:
                    remaining_ms = (REQUEST_MIN_S - elapsed) * 1000
                    print(f"  [rate limit] retry in {remaining_ms:.0f} ms")
                    time.sleep(0.005)
                    continue

                last_request = now

                # ---- Send one-shot GET_DATA --------------------------------
                spi.xfer2(list(get_pkt))
                print(f"  >> GET_DATA sent", flush=True)

                # ---- Wait for DR to assert (HIGH) --------------------------
                dr_deadline = time.monotonic() + DR_WAIT_S
                while lgpio.gpio_read(gpio_h, DR_GPIO_LINE) == 0:
                    if time.monotonic() >= dr_deadline:
                        print("  [!] Timeout waiting for DR to assert — no response")
                        break
                else:
                    # ---- Read SPI chunks while DR stays HIGH ---------------
                    rx_buf        = bytearray()
                    read_deadline = time.monotonic() + DR_READ_S

                    while (lgpio.gpio_read(gpio_h, DR_GPIO_LINE) == 1
                           and time.monotonic() < read_deadline):
                        rx_buf.extend(spi.readbytes(SPI_CHUNK_SIZE))

                    # ---- Parse and print -----------------------------------
                    found = False
                    for pkt_type, did, payload in parse_isb_packets(bytes(rx_buf)):
                        if pkt_type != PKT_TYPE_DATA or did != DID_IMU:
                            continue
                        if len(payload) < IMU_SIZE:
                            print(f"  [!] Short payload ({len(payload)} < {IMU_SIZE} bytes), skipping")
                            continue

                        found = True
                        count += 1
                        t_time, status, *rest = _IMU_FMT.unpack_from(payload)
                        pqr = rest[0:3]
                        acc = rest[3:6]

                        print(
                            f"[{count:4d}]  "
                            f"t={t_time:10.3f} s  "
                            f"pqr={pqr[0]:9.5f}, {pqr[1]:9.5f}, {pqr[2]:9.5f} rad/s  "
                            f"acc={acc[0]:9.5f}, {acc[1]:9.5f}, {acc[2]:9.5f} m/s^2  "
                            f"({len(rx_buf)} bytes read)",
                            flush=True,
                        )

                    if not found:
                        print(f"  [!] No DID_IMU packet found in {len(rx_buf)} bytes received")

            time.sleep(0.005)           # 5 ms idle poll — keeps CPU load low

    except KeyboardInterrupt:
        pass    # Ctrl+C falls through to finally

    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, saved_termios)
        lgpio.gpio_free(gpio_h, DR_GPIO_LINE)
        lgpio.gpiochip_close(gpio_h)
        spi.close()
        print("\nExiting.")


if __name__ == "__main__":
    main()
