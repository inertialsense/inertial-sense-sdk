#!/usr/bin/env python3
"""
InertialSense SPI NMEA Example - Raspberry Pi 5

Sends the NMEA command "$ASCE,2,INFO,1*35\r\n" over SPI, waits for the
Data Ready (DR) line to assert, reads the response while DR is HIGH, then
prints the output to the console.

Hardware connections (IMX <-> Raspberry Pi 5 40-pin header):
    IMX SPI_SCLK -> Pin 23  (GPIO11 / SPI0_CLK)
    IMX SPI_MOSI -> Pin 19  (GPIO10 / SPI0_MOSI)
    IMX SPI_MISO -> Pin 21  (GPIO9  / SPI0_MISO)
    IMX SPI_nCS  -> Pin 24  (GPIO8  / SPI0_CE0)
    IMX DR       -> Pin 22  (GPIO25 / Data Ready, active high)
    IMX nSPI_EN  -> GND     (hold low at power-up to enable SPI)

IMPORTANT - G9 / nSPI_EN conflict with GPS PPS:
    G9 doubles as the GPS PPS input. If a GPS receiver drives a PPS signal on
    that pin it silently disables SPI regardless of how the pin is pulled.
    Disconnect or inhibit the GPS PPS output before enabling SPI mode.

Raspberry Pi 5 setup:
    1. Enable SPI0 in /boot/firmware/config.txt, then reboot:
           dtoverlay=spi0-1cs
    2. Install dependencies:
           sudo apt install python3-spidev python3-lgpio
    3. Add your user to the spi and gpio groups:
           sudo usermod -aG spi,gpio $USER   # log out and back in

Run:
    python3 spi_nmea_example.py
"""

import sys
import time
import spidev
import lgpio

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------

SPI_BUS       = 0           # SPI bus number  -> /dev/spidevBUS.DEVICE
SPI_DEVICE    = 0           # SPI device (chip-select index)
SPI_MODE      = 3           # CPOL=1, CPHA=1  required by the IMX
SPI_SPEED_HZ  = 1_000_000  # 1 MHz; conservative, safe for NMEA command/response

SPI_CHUNK     = 64          # bytes to read per SPI transaction while DR is HIGH

GPIO_CHIP     = 4           # gpiochip4 on Raspberry Pi 5
                            # Change to 0 for Raspberry Pi 4
DR_GPIO       = 25          # BCM GPIO25 = physical pin 22

DR_TIMEOUT_S  = 5.0        # seconds to wait for DR before giving up

COMMAND       = b"$ASCE,2,INFO,1*35\r\n"

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def open_spi():
    spi = spidev.SpiDev()
    spi.open(SPI_BUS, SPI_DEVICE)
    spi.max_speed_hz = SPI_SPEED_HZ
    spi.mode = SPI_MODE
    return spi


def open_gpio():
    gpio = lgpio.gpiochip_open(GPIO_CHIP)
    lgpio.gpio_claim_input(gpio, DR_GPIO)
    return gpio


def wait_for_dr_high(gpio, timeout_s):
    """Block until GPIO25 goes HIGH or timeout expires. Returns True if HIGH."""
    deadline = time.monotonic() + timeout_s
    while lgpio.gpio_read(gpio, DR_GPIO) == 0:
        if time.monotonic() > deadline:
            return False
        time.sleep(0.001)
    return True


def read_while_dr_high(spi, gpio):
    """
    Read SPI data in SPI_CHUNK-byte chunks while DR is HIGH.

    Per the IS SPI spec, DR goes inactive 1-2 bytes before the last byte of
    the packet is clocked out.  One extra read after DR drops captures those
    trailing bytes (mirrors the behaviour of SPIReadDR() in IS_SPI_Dev_Example).
    """
    rx = bytearray()
    while lgpio.gpio_read(gpio, DR_GPIO) == 1:
        rx.extend(spi.readbytes(SPI_CHUNK))

    # One extra chunk after DR drops to capture the 1-2 tail bytes.
    rx.extend(spi.readbytes(SPI_CHUNK))
    return rx


def print_response(raw):
    """Strip null padding and print; fall back to hex if not printable ASCII."""
    # The IMX clocks out 0x00 bytes when its transmit buffer is empty.
    stripped = bytes(b for b in raw if b != 0x00)

    print(f"\n--- Response  ({len(raw)} raw bytes, {len(stripped)} non-null) ---")
    if not stripped:
        print("(no data received)")
        return

    try:
        print(stripped.decode("ascii"))
    except UnicodeDecodeError:
        # Binary / ISB packet - print as hex groups
        hex_str = " ".join(f"{b:02X}" for b in stripped)
        print(f"[hex] {hex_str}")

# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    spi  = open_spi()
    gpio = open_gpio()

    try:
        # --- Step 1: Send the NMEA command ------------------------------------
        #
        # SPI is full-duplex: while we clock out the command on MOSI the IMX
        # may clock bytes back on MISO.  At this point the IMX hasn't processed
        # the command yet so MISO is typically 0x00; those bytes are discarded.
        print(f"TX: {COMMAND!r}")
        spi.xfer2(list(COMMAND))

        # --- Step 2: Wait for DR (GPIO25) to go HIGH --------------------------
        #
        # The IMX asserts DR (active HIGH) when a complete response packet is
        # ready in its 4096-byte internal buffer.
        print(f"Waiting for DR (GPIO{DR_GPIO}, pin 22) to go HIGH ...", flush=True)

        if not wait_for_dr_high(gpio, DR_TIMEOUT_S):
            sys.exit(
                f"\nERROR: DR did not go HIGH within {DR_TIMEOUT_S} s.\n"
                "  Check:\n"
                "    - nSPI_EN (G9) is held LOW (not driven by GPS PPS)\n"
                "    - DR wired to the correct GPIO pin\n"
                "    - SPI enabled in /boot/firmware/config.txt\n"
            )

        # --- Step 3: Read SPI data while DR is HIGH ---------------------------
        print("DR HIGH - reading response ...", flush=True)
        rx = read_while_dr_high(spi, gpio)

        # --- Step 4: Print the response ---------------------------------------
        print_response(rx)

    finally:
        spi.close()
        lgpio.gpiochip_close(gpio)


if __name__ == "__main__":
    main()
