#!/usr/bin/env python3
"""
InertialSense SPI NMEA Example - Raspberry Pi 5

Repeatedly sends "$ASCE,2,INFO,1*35\r\n" over SPI, waits for the Data Ready
(DR) line to assert, reads the response while DR is HIGH, and prints the
output.  Runs continuously until the user presses Q.

Hardware connections (IMX <-> Raspberry Pi 5 40-pin header):
    IMX SPI_SCLK -> Pin 23  (GPIO11 / SPI0_CLK)
    IMX SPI_MOSI -> Pin 19  (GPIO10 / SPI0_MOSI)
    IMX SPI_MISO -> Pin 21  (GPIO9  / SPI0_MISO)
    IMX SPI_nCS  -> Pin 24  (GPIO8  / SPI0_CE0)
    IMX DR       -> Pin 22  (GPIO25 / Data Ready, active high)
    IMX nSPI_EN  -> GND     (hold low at power-up to enable SPI)

IMPORTANT - G9 / nSPI_EN conflict with GPS PPS:
    G9 doubles as the GPS PPS input.  If a GPS receiver drives a PPS signal on
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
import select
import tty
import termios
import spidev
import lgpio

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------

SPI_BUS      = 0            # SPI bus number  -> /dev/spidevBUS.DEVICE
SPI_DEVICE   = 0            # SPI device (chip-select index)
SPI_MODE     = 3            # CPOL=1, CPHA=1  required by the IMX
SPI_SPEED_HZ = 1_000_000   # 1 MHz; conservative, safe for NMEA command/response

SPI_CHUNK    = 64           # bytes per SPI read transaction while DR is HIGH

GPIO_CHIP    = 4            # gpiochip4 on Raspberry Pi 5 (use 0 for RPi4)
DR_GPIO      = 25           # BCM GPIO25 = physical pin 22

DR_TIMEOUT_S  = 5.0        # seconds to wait for DR before flagging a warning
LOOP_DELAY_S  = 1.0        # seconds between successive command sends

COMMAND  = b"$ASCE,2,INFO,1*35\r\n"
EXIT_KEY = "q"              # press this key (case-insensitive) to quit

# ---------------------------------------------------------------------------
# Non-blocking keyboard helpers
#
# tty.setcbreak() puts stdin into cbreak mode: each keystroke is delivered
# immediately without waiting for Enter, while output behaviour (LF -> CR+LF
# translation) and Ctrl+C signal generation are preserved unchanged.
# select.select() with a zero timeout polls stdin without blocking so the
# main loop can check for a keypress between each step.
# ---------------------------------------------------------------------------

def key_pressed() -> bool:
    """Return True if a character is waiting on stdin (non-blocking)."""
    return bool(select.select([sys.stdin], [], [], 0)[0])


def read_key() -> str:
    """Read exactly one character from stdin."""
    return sys.stdin.read(1)


def check_exit(*, consume: bool = True) -> bool:
    """Return True if the exit key is currently waiting in stdin."""
    if key_pressed():
        key = read_key() if consume else ""
        if key.lower() == EXIT_KEY:
            return True
    return False


def sleep_interruptible(seconds: float) -> bool:
    """
    Sleep for up to 'seconds', waking every 50 ms to check for the exit key.
    Returns True if the exit key was pressed during the sleep.
    """
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        if check_exit():
            return True
        time.sleep(0.05)
    return False

# ---------------------------------------------------------------------------
# SPI / GPIO helpers
# ---------------------------------------------------------------------------

def wait_for_dr_high(gpio, timeout_s: float) -> str:
    """
    Poll GPIO25 until DR goes HIGH or timeout expires, checking for the exit
    key between polls.

    Returns:
        'high'    - DR asserted, safe to read
        'timeout' - DR did not assert within timeout_s
        'exit'    - user pressed the exit key
    """
    deadline = time.monotonic() + timeout_s
    while lgpio.gpio_read(gpio, DR_GPIO) == 0:
        if time.monotonic() > deadline:
            return "timeout"
        if check_exit():
            return "exit"
        time.sleep(0.001)
    return "high"


def read_while_dr_high(spi, gpio) -> bytearray:
    """
    Read SPI data in SPI_CHUNK-byte bursts while DR is HIGH, then one extra
    burst after DR drops.

    Per the IS SPI spec DR goes inactive 1-2 bytes before the final byte of
    the packet is clocked out.  The extra read captures those trailing bytes
    (mirrors SPIReadDR() in IS_SPI_Dev_Example).
    """
    rx = bytearray()
    while lgpio.gpio_read(gpio, DR_GPIO) == 1:
        rx.extend(spi.readbytes(SPI_CHUNK))

    rx.extend(spi.readbytes(SPI_CHUNK))    # tail bytes after DR drops
    return rx


def print_response(raw: bytearray, iteration: int) -> None:
    """Strip null-byte padding and print; fall back to hex for binary data."""
    stripped = bytes(b for b in raw if b != 0x00)

    print(f"\n--- [{iteration}] Response  "
          f"({len(raw)} raw bytes, {len(stripped)} non-null) ---")

    if not stripped:
        print("  (no data received)")
        return

    try:
        print(stripped.decode("ascii"), end="")
        if not stripped.endswith(b"\n"):
            print()                         # ensure newline after NMEA sentence
    except UnicodeDecodeError:
        # Binary / ISB packet — print as spaced hex
        print("  [hex]", " ".join(f"{b:02X}" for b in stripped))

# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main() -> None:
    spi = spidev.SpiDev()
    spi.open(SPI_BUS, SPI_DEVICE)
    spi.max_speed_hz = SPI_SPEED_HZ
    spi.mode = SPI_MODE

    gpio = lgpio.gpiochip_open(GPIO_CHIP)
    lgpio.gpio_claim_input(gpio, DR_GPIO)

    # Save terminal state so we can restore it on exit.
    saved_termios = termios.tcgetattr(sys.stdin)

    try:
        # Switch stdin to cbreak mode: one keypress at a time, no Enter needed.
        tty.setcbreak(sys.stdin.fileno())

        print(f"InertialSense SPI NMEA Example  |  /dev/spidev{SPI_BUS}.{SPI_DEVICE}"
              f"  |  {SPI_SPEED_HZ // 1_000} kHz")
        print(f"Command : {COMMAND!r}")
        print(f"DR GPIO : GPIO{DR_GPIO} (pin 22)")
        print(f"Press '{EXIT_KEY.upper()}' to exit.\n")

        iteration = 0

        while True:
            iteration += 1

            # ------------------------------------------------------------------
            # Step 1 - Send the NMEA command
            #
            # SPI is full-duplex.  While we clock the command out on MOSI the
            # IMX simultaneously clocks bytes back on MISO; at this point the
            # device has not yet processed the command so MISO is 0x00 and the
            # received bytes are discarded.
            # ------------------------------------------------------------------
            print(f"[{iteration}] TX: {COMMAND!r}", flush=True)
            spi.xfer2(list(COMMAND))

            # ------------------------------------------------------------------
            # Step 2 - Wait for DR (GPIO25) to go HIGH
            #
            # The IMX asserts DR when a complete response packet is ready in its
            # 4096-byte internal FIFO.
            # ------------------------------------------------------------------
            print(f"      Waiting for DR (GPIO{DR_GPIO}) HIGH ...", flush=True)
            result = wait_for_dr_high(gpio, DR_TIMEOUT_S)

            if result == "exit":
                break

            if result == "timeout":
                print(f"      WARNING: DR did not go HIGH within {DR_TIMEOUT_S} s.\n"
                      "      Check: nSPI_EN (G9) is held LOW, DR wired correctly, "
                      "SPI enabled in config.txt.")
            else:
                # --------------------------------------------------------------
                # Step 3 - Read SPI data while DR is HIGH
                # --------------------------------------------------------------
                print("      DR HIGH - reading ...", flush=True)
                rx = read_while_dr_high(spi, gpio)

                # --------------------------------------------------------------
                # Step 4 - Print the response
                # --------------------------------------------------------------
                print_response(rx, iteration)

            # ------------------------------------------------------------------
            # Wait LOOP_DELAY_S before the next send, exiting immediately if the
            # user presses the exit key during the pause.
            # ------------------------------------------------------------------
            print(f"\n      Next send in {LOOP_DELAY_S:.0f} s  "
                  f"(press '{EXIT_KEY.upper()}' to exit) ...\n", flush=True)
            if sleep_interruptible(LOOP_DELAY_S):
                break

    except KeyboardInterrupt:
        pass    # Ctrl+C handled gracefully by the finally block below

    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, saved_termios)
        spi.close()
        lgpio.gpiochip_close(gpio)
        print("\nExiting.")


if __name__ == "__main__":
    main()
