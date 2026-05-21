#!/usr/bin/env python3
"""
InertialSense SPI DID_DEV_INFO One-Shot - Raspberry Pi 5

Sends a single PKT_TYPE_GET_DATA for DID_DEV_INFO (period=0), waits 200 ms,
reads 90 bytes from the SPI bus, parses the dev_info_t response, prints the
result, and exits.

Hardware connections (IMX <-> Raspberry Pi 5 40-pin header):
    IMX SPI_SCLK -> Pin 23  (GPIO11 / SPI0_CLK)
    IMX SPI_MOSI -> Pin 19  (GPIO10 / SPI0_MOSI)
    IMX SPI_MISO -> Pin 21  (GPIO9  / SPI0_MISO)
    IMX SPI_nCS  -> Pin 24  (GPIO8  / SPI0_CE0)
    IMX nSPI_EN  -> GND     (hold low at power-up to enable SPI)
"""

import time
import struct
import spidev

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------

SPI_BUS      = 0
SPI_DEVICE   = 0
SPI_MODE     = 3            # CPOL=1, CPHA=1  required by the IMX
SPI_SPEED_HZ = 1_000_000   # 1 MHz

READ_DELAY_S = 0.200        # wait after request before reading
READ_SIZE    = 90           # bytes to read (84-byte payload + 6-byte header)

# ---------------------------------------------------------------------------
# ISB protocol constants
# ---------------------------------------------------------------------------

ISB_PREAMBLE = bytes([0xEF, 0x49])

PKT_TYPE_GET_DATA            = 3
PKT_TYPE_DATA                = 4
PKT_TYPE_STOP_BROADCASTS_ALL = 6

DID_DEV_INFO = 1

_DEV_INFO_FMT = struct.Struct("<4B I 4B 4B I 4B I 24s 8B 24s")
DEV_INFO_SIZE  = _DEV_INFO_FMT.size    # 84

# ---------------------------------------------------------------------------
# ISB packet builder
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


def build_get_data_once(did: int) -> bytes:
    """GET_DATA with period=0 — single shot."""
    payload = struct.pack("<HHHH", did, 0, 0, 0)
    return _build_packet(PKT_TYPE_GET_DATA, payload)

# ---------------------------------------------------------------------------
# ISB packet parser
# ---------------------------------------------------------------------------

def parse_isb_packets(data: bytes):
    """Yields (pkt_type, did, payload) for each complete ISB packet found."""
    i = 0
    while i < len(data):
        idx = data.find(ISB_PREAMBLE, i)
        if idx == -1:
            break
        i = idx

        if i + 8 > len(data):
            break

        pkt_type     = data[i + 2] & 0x0F
        did          = data[i + 3]
        payload_size = struct.unpack_from("<H", data, i + 4)[0]

        total = 6 + payload_size + 2
        if i + total > len(data):
            break

        yield pkt_type, did, data[i + 6: i + 6 + payload_size]
        i += total

# ---------------------------------------------------------------------------
# dev_info_t parser / printer
# ---------------------------------------------------------------------------

def parse_dev_info(payload: bytes) -> dict | None:
    if len(payload) < DEV_INFO_SIZE:
        return None

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
     add_info_raw) = _DEV_INFO_FMT.unpack_from(payload)

    return {
        "serialNumber": serial_number,
        "hardwareVer":  (hw0, hw1, hw2, hw3),
        "firmwareVer":  (fw0, fw1, fw2, fw3),
        "buildNumber":  build_number,
        "protocolVer":  (proto0, proto1, proto2, proto3),
        "repoRevision": repo_revision,
        "manufacturer": manufacturer_raw.rstrip(b"\x00").decode("ascii", errors="replace"),
        "buildDate":    f"20{build_year:02d}-{build_month:02d}-{build_day:02d}",
        "buildTime":    f"{build_hour:02d}:{build_minute:02d}:{build_second:02d}.{build_ms:03d}",
        "addInfo":      add_info_raw.rstrip(b"\x00").decode("ascii", errors="replace"),
    }


def print_dev_info(info: dict) -> None:
    fw    = info["firmwareVer"]
    hw    = info["hardwareVer"]
    proto = info["protocolVer"]
    print(f"  Serial Number : {info['serialNumber']}")
    print(f"  Firmware Ver  : {fw[0]}.{fw[1]}.{fw[2]}.{fw[3]}")
    print(f"  Hardware Ver  : {hw[0]}.{hw[1]}.{hw[2]}.{hw[3]}")
    print(f"  Build Number  : {info['buildNumber']}")
    print(f"  Protocol Ver  : {proto[0]}.{proto[1]}.{proto[2]}.{proto[3]}")
    print(f"  Repo Revision : {info['repoRevision']}")
    print(f"  Manufacturer  : {info['manufacturer']}")
    print(f"  Build Date    : {info['buildDate']}")
    print(f"  Build Time    : {info['buildTime']}")
    print(f"  Add Info      : {info['addInfo']}")

# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main() -> None:
    spi = spidev.SpiDev()
    spi.open(SPI_BUS, SPI_DEVICE)
    spi.max_speed_hz = SPI_SPEED_HZ
    spi.mode = SPI_MODE

    try:
        # Stop any stale broadcasts before requesting fresh data.
        stop_pkt = build_stop_broadcasts()
        spi.xfer2(list(stop_pkt))
        time.sleep(0.05)

        # Send single one-shot GET_DATA request.
        get_pkt = build_get_data_once(DID_DEV_INFO)
        print(f"Sending GET_DATA for DID_DEV_INFO ...")
        spi.xfer2(list(get_pkt))

        # Wait 200 ms for the device to prepare the response.
        time.sleep(READ_DELAY_S)

        # Read 90 bytes and parse.
        rx = bytes(spi.readbytes(READ_SIZE))
        print(f"Read {READ_SIZE} bytes  ({sum(1 for b in rx if b != 0)} non-zero)\n")

        found = False
        for pkt_type, did, payload in parse_isb_packets(rx):
            if pkt_type != PKT_TYPE_DATA or did != DID_DEV_INFO:
                continue
            info = parse_dev_info(payload)
            if info is None:
                print(f"[!] Short payload ({len(payload)} < {DEV_INFO_SIZE} bytes)")
                continue
            found = True
            print("DID_DEV_INFO:")
            print_dev_info(info)

        if not found:
            print("[!] No DID_DEV_INFO packet found in response")
            print("Raw bytes:", rx.hex(" ").upper())

    finally:
        spi.close()


if __name__ == "__main__":
    main()
