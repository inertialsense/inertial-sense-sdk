#!/usr/bin/env python3
"""
InertialSense SPI Raw Read Example - Raspberry Pi 5

Reads 250 bytes from SPI every 250 ms and prints them as a hex dump.
No packets are sent to the device; this is a passive read useful for
verifying SPI wiring and signal integrity.  Press Q to quit.

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
    python3 spi_raw_read_example.py
"""

import sys
import time
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
SPI_SPEED_HZ = 1_000_000   # 1 MHz

READ_SIZE    = 250          # bytes to read each tick
READ_INTERVAL_S = 10.0       # seconds between reads

EXIT_KEY = "q"

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
# Hex dump
# ---------------------------------------------------------------------------

def hex_dump(data: bytes, indent: str = "  ") -> None:
    """Print data as an 16-byte-wide annotated hex dump."""
    for i in range(0, len(data), 16):
        chunk = data[i:i + 16]
        hex_part  = " ".join(f"{b:02X}" for b in chunk)
        ascii_part = "".join(chr(b) if 32 <= b < 127 else "." for b in chunk)
        print(f"{indent}{i:04X}  {hex_part:<47}  {ascii_part}")

# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main() -> None:
    spi = spidev.SpiDev()
    spi.open(SPI_BUS, SPI_DEVICE)
    spi.max_speed_hz = SPI_SPEED_HZ
    spi.mode = SPI_MODE

    saved_termios = termios.tcgetattr(sys.stdin)
    try:
        tty.setcbreak(sys.stdin.fileno())

        print(f"InertialSense SPI Raw Read Example  "
              f"|  /dev/spidev{SPI_BUS}.{SPI_DEVICE}  "
              f"|  {SPI_SPEED_HZ // 1_000} kHz")
        print(f"Reading {READ_SIZE} bytes every {READ_INTERVAL_S * 1000:.0f} ms")
        print(f"Press '{EXIT_KEY.upper()}' to exit.\n")

        tick = 0
        while True:
            rx = bytes(spi.readbytes(READ_SIZE))
            tick += 1

            non_zero = sum(1 for b in rx if b != 0)
            print(f"--- tick {tick:4d}  {time.monotonic():.3f} s  "
                  f"{non_zero}/{READ_SIZE} non-zero bytes ---")
            hex_dump(rx)

            if check_exit():
                break

            time.sleep(READ_INTERVAL_S)

    except KeyboardInterrupt:
        pass

    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, saved_termios)
        spi.close()
        print("\nExiting.")


if __name__ == "__main__":
    main()
