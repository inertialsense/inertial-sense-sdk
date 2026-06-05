#!/usr/bin/env python3
"""
InertialSense SPI DID_DEV_INFO Example — key-triggered, Data Ready pin
Raspberry Pi 5

Press 'R' to request one DID_DEV_INFO sample (rate-limited to once per
second).  The request uses period=0 (single shot, no repeating broadcast).
After each request the loop waits for the Data Ready GPIO pin to assert HIGH,
then reads SPI bytes in chunks until DR de-asserts LOW or 250 ms elapses,
parses any ISB DATA packets, and prints the dev_info_t fields.  Press 'Q' to
quit.

ISB packet format (protocol version 2):
    Bytes 0-1  : Preamble 0xEF 0x49
    Byte  2    : Flags  (lower 4 bits = packet type, upper 4 bits = flags)
    Byte  3    : Data ID in header (0 for command packets)
    Bytes 4-5  : Payload size (little-endian uint16)
    Bytes 6-N  : Payload
    Bytes N+1-2: Fletcher-16 checksum (little-endian, a=low byte, b=high byte)

dev_info_t wire layout (84 bytes, little-endian / PACKED):
    uint8     reserved          offset  0
    uint8     buildFlags        offset  1
    uint8     hardwareType      offset  2
    uint8     hdwRunState       offset  3
    uint32    serialNumber      offset  4
    uint8[4]  hardwareVer       offset  8
    uint8[4]  firmwareVer       offset 12
    uint32    buildNumber       offset 16
    uint8[4]  protocolVer       offset 20
    uint32    repoRevision      offset 24
    char[24]  manufacturer      offset 28
    uint8     buildType         offset 52
    uint8     buildYear         offset 53
    uint8     buildMonth        offset 54
    uint8     buildDay          offset 55
    uint8     buildHour         offset 56
    uint8     buildMinute       offset 57
    uint8     buildSecond       offset 58
    uint8     buildMillisecond  offset 59
    char[24]  addInfo           offset 60
    (total: 84 bytes)

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
    python3 spi_dev_info_dr_example.py
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

EXIT_KEY     = "q"
REQUEST_KEY  = "r"

# ---------------------------------------------------------------------------
# ISB protocol constants
# ---------------------------------------------------------------------------

ISB_PREAMBLE = bytes([0xEF, 0x49])     # PSC_ISB_PREAMBLE_BYTE1, BYTE2

PKT_TYPE_GET_DATA            = 3       # request a DID
PKT_TYPE_DATA                = 4       # response carrying a DID payload
PKT_TYPE_STOP_BROADCASTS_ALL = 6       # stop all broadcasts on all ports

DID_DEV_INFO = 1                       # Device information

# dev_info_t: 84 bytes, PACKED, little-endian
#   4B  = reserved, buildFlags, hardwareType, hdwRunState
#   I   = serialNumber
#   4B  = hardwareVer[4]
#   4B  = firmwareVer[4]
#   I   = buildNumber
#   4B  = protocolVer[4]
#   I   = repoRevision
#   24s = manufacturer[24]
#   8B  = buildType, buildYear, buildMonth, buildDay,
#          buildHour, buildMinute, buildSecond, buildMillisecond
#   24s = addInfo[24]
_DEV_INFO_FMT = struct.Struct("<4B I 4B 4B I 4B I 24s 8B 24s")
DEV_INFO_SIZE = _DEV_INFO_FMT.size     # 84

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
# dev_info_t parser
# ---------------------------------------------------------------------------

def parse_dev_info(payload: bytes) -> dict | None:
    """Unpack a dev_info_t payload. Returns a dict or None if too short."""
    if len(payload) < DEV_INFO_SIZE:
        return None

    fields = _DEV_INFO_FMT.unpack_from(payload)
    (reserved, build_flags, hw_type, hdw_run_state,
     serial_number,
     hw0, hw1, hw2, hw3,
     fw0, fw1, fw2, fw3,
     build_number,
     proto0, proto1, proto2, proto3,
     repo_revision,
     manufacturer_raw,
     build_type, build_year, build_month, build_day,
     build_hour, build_minute, build_second, build_ms,
     add_info_raw) = fields

    return {
        "serialNumber":   serial_number,
        "hardwareVer":    (hw0, hw1, hw2, hw3),
        "firmwareVer":    (fw0, fw1, fw2, fw3),
        "buildNumber":    build_number,
        "protocolVer":    (proto0, proto1, proto2, proto3),
        "repoRevision":   repo_revision,
        "manufacturer":   manufacturer_raw.rstrip(b"\x00").decode("ascii", errors="replace"),
        "buildDate":      f"20{build_year:02d}-{build_month:02d}-{build_day:02d}",
        "buildTime":      f"{build_hour:02d}:{build_minute:02d}:{build_second:02d}.{build_ms:03d}",
        "buildType":      build_type,
        "buildFlags":     build_flags,
        "hardwareType":   hw_type,
        "hdwRunState":    hdw_run_state,
        "addInfo":        add_info_raw.rstrip(b"\x00").decode("ascii", errors="replace"),
    }


def print_dev_info(count: int, info: dict, rx_bytes: int) -> None:
    """Print dev_info_t fields in a readable format."""
    fw  = info["firmwareVer"]
    hw  = info["hardwareVer"]
    proto = info["protocolVer"]
    print(
        f"[{count:4d}]  "
        f"SN={info['serialNumber']}  "
        f"FW={fw[0]}.{fw[1]}.{fw[2]}.{fw[3]}  "
        f"HW={hw[0]}.{hw[1]}.{hw[2]}.{hw[3]}  "
        f"Build={info['buildNumber']}  "
        f"Proto={proto[0]}.{proto[1]}.{proto[2]}.{proto[3]}  "
        f"Repo={info['repoRevision']}",
        flush=True,
    )
    print(
        f"       "
        f"Mfr=\"{info['manufacturer']}\"  "
        f"Date={info['buildDate']}  "
        f"Time={info['buildTime']}  "
        f"AddInfo=\"{info['addInfo']}\"  "
        f"({rx_bytes} bytes read)",
        flush=True,
    )

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
    get_pkt      = build_get_data_once(DID_DEV_INFO)
    last_request = -REQUEST_MIN_S       # allow immediate first request
    count        = 0

    saved_termios = termios.tcgetattr(sys.stdin)
    try:
        tty.setcbreak(sys.stdin.fileno())

        print(f"InertialSense SPI DID_DEV_INFO Example  (DR pin, key-triggered)  "
              f"|  /dev/spidev{SPI_BUS}.{SPI_DEVICE}  "
              f"|  {SPI_SPEED_HZ // 1_000} kHz  "
              f"|  DR = GPIO{DR_GPIO_LINE}")
        print(f"GET_DATA packet : {get_pkt.hex(' ').upper()}  (period=0, single shot)")
        print(f"Press '{REQUEST_KEY.upper()}' to request DID_DEV_INFO  |  "
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
                        if pkt_type != PKT_TYPE_DATA or did != DID_DEV_INFO:
                            continue

                        info = parse_dev_info(payload)
                        if info is None:
                            print(f"  [!] Short payload ({len(payload)} < {DEV_INFO_SIZE} bytes), skipping")
                            continue

                        found = True
                        count += 1
                        print_dev_info(count, info, len(rx_buf))

                    if not found:
                        print(f"  [!] No DID_DEV_INFO packet found in {len(rx_buf)} bytes received")

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
