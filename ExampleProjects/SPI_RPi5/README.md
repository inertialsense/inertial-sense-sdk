# InertialSense SPI Examples — Raspberry Pi 5

Python examples for communicating with the **IMX-6** over SPI from a Raspberry Pi 5.
All examples use the **ISB (InertialSense Binary) protocol** over SPI0.

---

## Hardware Connections

### Signal Table

| IMX-6 Signal | IMX-6 Pad | RPi 5 Header Pin | RPi 5 GPIO | Notes                                   |
|--------------|-----------|------------------|------------|-----------------------------------------|
| SPI_SCLK     | G5        | Pin 23           | GPIO11     | SPI0 clock                              |
| SPI_MOSI     | G6        | Pin 19           | GPIO10     | IMX RX — data from RPi to IMX           |
| SPI_MISO     | G7        | Pin 21           | GPIO9      | IMX TX — data from IMX to RPi           |
| SPI_nCS      | G8        | Pin 24           | GPIO8      | Chip select, active LOW (SPI0_CE0)      |
| DR           | G9        | Pin 22           | GPIO25     | Data Ready, active HIGH (DR-pin examples only) |

> **nSPI_EN** must be tied to GND (or driven LOW by the host) before the IMX-6
> powers on.  If left floating the module will not activate its SPI interface.
> DR is only needed for the `*_dr_example.py` scripts.

### Raspberry Pi 5 — 40-Pin Header Reference

```
        3V3  [ 1] [ 2]  5V
      GPIO2  [ 3] [ 4]  5V
      GPIO3  [ 5] [ 6]  GND  <-- tie IMX nSPI_EN here
      GPIO4  [ 7] [ 8]  GPIO14
        GND  [ 9] [10]  GPIO15
     GPIO17  [11] [12]  GPIO18
     GPIO27  [13] [14]  GND
     GPIO22  [15] [16]  GPIO23
        3V3  [17] [18]  GPIO24
GPIO10/MOSI  [19] [20]  GND
 GPIO9/MISO  [21] [22]  GPIO25  <-- IMX DR
GPIO11/SCLK  [23] [24]  GPIO8/CE0  <-- IMX nCS
        GND  [25] [26]  GPIO7/CE1
```

### IMX-6 Module — Connector Pins Used

| IMX-6 Pad | Function   | Direction (from RPi perspective) |
|-----------|------------|----------------------------------|
| PA5       | SPI1_SCK   | Output (RPi drives clock)        |
| PA7       | SPI1_MOSI  | Output (RPi → IMX)               |
| PA6       | SPI1_MISO  | Input  (IMX → RPi)               |
| PA4       | SPI1_NSS   | Output (RPi drives chip select)  |
| PB0       | nSPI_EN    | Input  (tie to GND)              |
| PA15      | DR / DRDY  | Input  (IMX asserts HIGH when data ready) |

---

## Raspberry Pi 5 Setup

```bash
# 1. Enable SPI0 — add to /boot/firmware/config.txt, then reboot
dtoverlay=spi0-1cs

# 2. Install Python dependencies
sudo apt install python3-spidev          # all examples
sudo apt install python3-lgpio           # DR-pin examples only

# 3. Add user to required groups
sudo usermod -aG spi $USER               # all examples
sudo usermod -aG gpio $USER              # DR-pin examples only
# Log out and back in for group changes to take effect
```

---

## SPI Settings

| Parameter   | Value  |
|-------------|--------|
| Bus         | 0      |
| Device      | 0      |
| Mode        | 3 (CPOL=1, CPHA=1) |
| Speed       | 1 MHz  |

---

## Examples

### `spi_dev_info_example.py` — Device Info, Polling (no DR pin)

Auto-sends `GET_DATA(DID_DEV_INFO)` every **1 second**.  Reads **200 bytes**
every **250 ms** (Strategy A — no Data Ready pin required).  Parses and
prints device info each time a response arrives.

**DID:** `DID_DEV_INFO = 1` | **Struct size:** 84 bytes
**Run:** `python3 spi_dev_info_example.py`
**Dependencies:** `python3-spidev`
**Press `Q` to quit.**

---

### `spi_dev_info_dr_example.py` — Device Info, Data Ready Pin

Press **`R`** to send a single-shot `GET_DATA(DID_DEV_INFO)` request
(rate-limited to **1 Hz**).  Waits up to 250 ms for the DR pin to assert
HIGH, then reads 64-byte SPI chunks until DR de-asserts LOW or 250 ms
elapses.

**DID:** `DID_DEV_INFO = 1` | **Struct size:** 84 bytes
**Run:** `python3 spi_dev_info_dr_example.py`
**Dependencies:** `python3-spidev`, `python3-lgpio`
**Press `R` to request, `Q` to quit.**

---

### `spi_pimu_example.py` — PIMU, Polling (no DR pin)

Auto-sends `GET_DATA(DID_PIMU)` every **1 second**.  Reads **200 bytes**
every **50 ms** (Strategy A).  Prints delta-theta and delta-velocity each
time a response arrives.

```
[   1]  t=   123.456 s  dt=0.0070 s  dTheta=  0.00012,  0.00034,  0.00056 rad  dVel=  0.00001,  0.00002,  0.06807 m/s
```

**DID:** `DID_PIMU = 3` | **Struct size:** 40 bytes
**Run:** `python3 spi_pimu_example.py`
**Dependencies:** `python3-spidev`
**Press `Q` to quit.**

---

### `spi_pimu_dr_example.py` — PIMU, Data Ready Pin

Press **`R`** to send a single-shot `GET_DATA(DID_PIMU)` request
(rate-limited to **250 ms**).  Waits for DR to assert, then reads until DR
de-asserts or 250 ms elapses.

**DID:** `DID_PIMU = 3` | **Struct size:** 40 bytes
**Run:** `python3 spi_pimu_dr_example.py`
**Dependencies:** `python3-spidev`, `python3-lgpio`
**Press `R` to request, `Q` to quit.**

---

## Strategy A vs DR-Pin

| | Strategy A (no DR) | DR-Pin |
|---|---|---|
| Extra wiring | None | DR signal required |
| Dependencies | `python3-spidev` | + `python3-lgpio` |
| How it works | Fixed-size read every N ms | Reads only while DR is HIGH |
| Best for | Quick bring-up, simple setups | Accurate timing, lower CPU |

---

## ISB Packet Format

```
Byte 0-1 : Preamble  0xEF 0x49
Byte 2   : Flags     [7:4] = flags, [3:0] = packet type
Byte 3   : Data ID   (0 for command packets)
Byte 4-5 : Payload size (uint16, little-endian)
Byte 6-N : Payload
Byte N+1 : Fletcher-16 checksum low byte
Byte N+2 : Fletcher-16 checksum high byte
```

| Packet Type | Value | Description |
|-------------|-------|-------------|
| `GET_DATA`  | 3     | Request a DID broadcast |
| `DATA`      | 4     | DID payload response |
| `STOP_ALL`  | 6     | Stop all broadcasts |

`GET_DATA` payload (`p_data_get_t`, 8 bytes):

| Field    | Type   | Description |
|----------|--------|-------------|
| `id`     | uint16 | Data ID to request |
| `size`   | uint16 | Byte count (0 = full struct) |
| `offset` | uint16 | Byte offset (0 = start) |
| `period` | uint16 | Nav-DT multiples (0 = single shot) |
