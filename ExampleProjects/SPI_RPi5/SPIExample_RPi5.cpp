/*
MIT LICENSE

Copyright 2026 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
associated documentation files (the "Software"), to deal in the Software without restriction,
including without limitation the rights to use, copy, modify, merge, publish, distribute,
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or
substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

/*
 * SPI Interface Example for Raspberry Pi 5
 *
 * Demonstrates connecting to an InertialSense IMX device over SPI using the
 * SDK's SpiPortFactory and ISComm APIs, with Data Ready GPIO gating (Strategy B).
 *
 * ---------------------------------------------------------------------------------
 * Hardware wiring (IMX <-> Raspberry Pi 5 40-pin header)
 * ---------------------------------------------------------------------------------
 *
 *   IMX Signal   RPi5 Header Pin   BCM GPIO   Description
 *   ----------   ---------------   --------   -----------
 *   SPI_SCLK  ->   Pin 23           GPIO11    SPI0 Clock
 *   SPI_MOSI  ->   Pin 19           GPIO10    SPI0 Master-Out Slave-In
 *   SPI_MISO  ->   Pin 21           GPIO9     SPI0 Master-In Slave-Out
 *   SPI_nCS   ->   Pin 24           GPIO8     SPI0 Chip-Select 0 (active low)
 *   DR        ->   Pin 22           GPIO25    Data Ready output (active high)
 *   nSPI_EN   ->   GND                        Hold G9 LOW at power-up to enable SPI
 *   GND       ->   Pin 6                      Ground
 *
 * IMPORTANT - G9 / nSPI_EN conflict with GPS PPS:
 *   G9 doubles as the external GPS PPS input.  If a connected GPS receiver drives
 *   a PPS signal on this pin it overrides nSPI_EN and SPI will be silently disabled
 *   regardless of how the pin is pulled.  Disconnect or inhibit the PPS output from
 *   the GPS receiver before enabling SPI mode.
 *
 * ---------------------------------------------------------------------------------
 * Raspberry Pi 5 setup
 * ---------------------------------------------------------------------------------
 *  1. Enable SPI0 - add ONE of these to /boot/firmware/config.txt, then reboot:
 *       dtoverlay=spi0-1cs          # recommended: CS0 only, no conflicts
 *     or:
 *       dtparam=spi=on              # enables both CS0 and CS1
 *
 *  2. Verify the device node exists after reboot:
 *       ls /dev/spidev0.*           # should show /dev/spidev0.0
 *
 *  3. Add your user to the spi and gpio groups (no sudo needed at runtime):
 *       sudo usermod -aG spi,gpio $USER
 *       # log out and back in for the groups to take effect
 *
 * ---------------------------------------------------------------------------------
 * Build
 * ---------------------------------------------------------------------------------
 *   cd ExampleProjects/SPI_RPi5
 *   mkdir build && cd build
 *   cmake .. -DCMAKE_BUILD_TYPE=Release
 *   make -j$(nproc)
 *
 * Run:
 *   ./SPIExample_RPi5
 * ---------------------------------------------------------------------------------
 */

#include <stdio.h>
#include <signal.h>

// STEP 1: Add Includes
// Adjust paths if your project layout differs from the SDK example directory structure.
#include "ISComm.h"
#include "ISPose.h"
#include "ISUtilities.h"
#include "PortFactory.h"      // SpiPortFactory
#include "core/spiPort.h"     // spiPortSetDataReady, SPI_PORT_DEFAULT_MODE

// =============================================================================
// Configuration - adjust to match your hardware
// =============================================================================

// Linux SPI device node.  For Raspberry Pi 5, SPI bus 0, chip-select 0:
#define SPI_DEVICE          "/dev/spidev0.0"

// SPI clock speed in Hz.
//   Strategy A (fixed-size polling, no DR): up to 3 MHz
//   Strategy B (Data Ready gated, used here): up to 5 MHz
#define SPI_SPEED_HZ        3000000         // 3 MHz

// BCM GPIO number connected to the IMX Data Ready (DR) output.
// DR is active-high: the IMX asserts it when a complete ISB packet is ready.
// Physical pin 22 on the RPi5 40-pin header = BCM GPIO25.
// Change this to match your wiring.
#define DR_GPIO_NUM         25

// Approximate IMX navigation period in milliseconds.
//   IMX-5: ~7 ms (143 Hz nav rate, startupNavDtMs default)
//   IMX-6: ~4 ms (250 Hz nav rate)
// is_comm_get_data() period argument is a multiple of this value.
#define NAV_DT_MS           7

// Convert a desired output period (ms) to the is_comm_get_data() period-multiple,
// rounding up so the actual rate never exceeds the requested rate.
#define MS_TO_PERIOD(ms)    (((ms) + NAV_DT_MS - 1) / NAV_DT_MS)

// =============================================================================
// Graceful shutdown on SIGINT / SIGTERM
// =============================================================================

static volatile bool g_running = true;

static void sigHandler(int) { g_running = false; }

// =============================================================================
// Message handlers - called once per received packet
// =============================================================================

static void handleIns1(ins_1_t* ins)
{
    printf("[INS1]  tow=%10.3f s   LLA=%11.7f, %11.7f, %8.2f m   "
           "euler=%6.1f, %6.1f, %6.1f deg\n",
        ins->timeOfWeek,
        ins->lla[0], ins->lla[1], ins->lla[2],
        ins->theta[0] * C_RAD2DEG_F,
        ins->theta[1] * C_RAD2DEG_F,
        ins->theta[2] * C_RAD2DEG_F);
}

static void handlePimu(pimu_t* pimu)
{
    // PIMU contains pre-integrated (coning & sculling) IMU data.
    // Divide by dt to recover instantaneous rates/accelerations.
    printf("[PIMU]  t=%10.3f s   dt=%.4f s   "
           "dTheta=%7.4f, %7.4f, %7.4f rad   "
           "dVel=%7.4f, %7.4f, %7.4f m/s\n",
        pimu->time, pimu->dt,
        pimu->theta[0], pimu->theta[1], pimu->theta[2],
        pimu->vel[0],   pimu->vel[1],   pimu->vel[2]);
}

static void handleDevInfo(dev_info_t* info)
{
    printf("[DEV_INFO]  SN=%u   HW=%u.%u.%u.%u   FW=%u.%u.%u.%u\n",
        info->serialNumber,
        info->hardwareVer[0], info->hardwareVer[1],
        info->hardwareVer[2], info->hardwareVer[3],
        info->firmwareVer[0], info->firmwareVer[1],
        info->firmwareVer[2], info->firmwareVer[3]);
}

static void handleGps1Pos(gps_pos_t* pos)
{
    uint32_t fixType = pos->status & GPS_STATUS_FIX_MASK;
    uint32_t numSats = (pos->status & GPS_STATUS_NUM_SATS_USED_MASK)
                        >> GPS_STATUS_NUM_SATS_USED_OFFSET;

    printf("[GPS1_POS]  towMs=%u ms   LLA=%11.7f, %11.7f, %8.2f m   "
           "fix=%u   sats=%u\n",
        pos->timeOfWeekMs,
        pos->lla[0], pos->lla[1], pos->lla[2],
        fixType, numSats);
}

// =============================================================================
// ISB data callback
//
// Registered via is_comm_register_port_isb_handler().  Called by
// is_comm_port_parse_messages() once for each successfully decoded ISB packet.
// =============================================================================

int isbDataHandler(void* ctx, p_data_t* data, port_handle_t port)
{
    (void)ctx;
    (void)port;

    switch (data->hdr.id)
    {
    case DID_INS_1:    handleIns1    ((ins_1_t*)    data->ptr);  break;
    case DID_PIMU:     handlePimu    ((pimu_t*)     data->ptr);  break;
    case DID_DEV_INFO: handleDevInfo ((dev_info_t*) data->ptr);  break;
    case DID_GPS1_POS: handleGps1Pos ((gps_pos_t*)  data->ptr);  break;
    // Add additional DID cases here as needed
    }

    return 0;
}

// =============================================================================
// main
// =============================================================================

int main()
{
    signal(SIGINT,  sigHandler);
    signal(SIGTERM, sigHandler);

    // -------------------------------------------------------------------------
    // STEP 2: Configure the SPI port factory
    //
    // SpiPortFactory is a singleton.  Set the SPI clock speed and mode before
    // calling bindPort().  The IMX requires SPI Mode 3 (CPOL=1, CPHA=1).
    // -------------------------------------------------------------------------

    SpiPortFactory& spif = SpiPortFactory::getInstance();
    spif.setSpeedHz(SPI_SPEED_HZ);
    spif.setMode(SPI_PORT_DEFAULT_MODE);    // Mode 3 - required by IMX

    // -------------------------------------------------------------------------
    // STEP 3: Bind (allocate) the SPI port
    //
    // bindPort() allocates the internal spi_port_t structure and records the
    // device path and configuration.  It does NOT open the device yet.
    // -------------------------------------------------------------------------

    port_handle_t port = spif.bindPort(SPI_DEVICE);
    if (port == nullptr)
    {
        printf("ERROR: Failed to allocate SPI port '%s'.\n"
               "       Verify the device node exists:  ls /dev/spidev*\n",
               SPI_DEVICE);
        return -1;
    }

    // -------------------------------------------------------------------------
    // STEP 4: Configure the Data Ready (DR) GPIO
    //
    // The IMX drives the DR line HIGH when a complete ISB packet is available
    // on MISO, and LOW after the last byte is clocked out.
    //
    // spiPortSetDataReady() exports the GPIO via the Linux sysfs interface,
    // configures it as an input with rising-edge detection, and stores the fd.
    //
    // After this call:
    //   - portAvailable(port) returns non-zero only while DR is HIGH
    //   - is_comm_port_parse_messages() blocks on the DR edge before each read,
    //     avoiding unnecessary SPI transfers when the bus is idle.
    //
    // This is "Strategy B: Data Ready gated" from the IS SPI protocol spec and
    // allows bus speeds up to 5 MHz.  Without DR gating the device would be
    // continuously polled (Strategy A, up to 3 MHz).
    //
    // NOTE - DR early deassert: per the IS SPI spec, DR goes inactive 1-2 bytes
    // BEFORE the last byte of the packet is clocked out.  Those trailing bytes
    // remain in the IMX's 4096-byte internal buffer and will be delivered at the
    // start of the next SPI transaction.  The ISB parser handles this correctly
    // because it accumulates bytes across calls; no data is lost, but there is
    // one parse-cycle of latency on the final bytes of each packet.  If your
    // application is sensitive to this, implement a "one extra read after DR
    // drops" loop similar to the Atmel IS_SPI_Dev_Example.
    // -------------------------------------------------------------------------

    if (spiPortSetDataReady(port, DR_GPIO_NUM) != 0)
    {
        printf("WARNING: Could not configure Data Ready GPIO %d.\n"
               "         Falling back to continuous polling (Strategy A).\n"
               "         Check:  /sys/class/gpio/gpio%d exists after export,\n"
               "                 or your user is in the 'gpio' group.\n",
               DR_GPIO_NUM, DR_GPIO_NUM);
        // Non-fatal: SPI communication will still work, just without DR gating.
    }

    // -------------------------------------------------------------------------
    // STEP 5: Open the SPI device
    //
    // portOpen() issues ioctl calls to set the SPI mode, bits-per-word, and
    // clock speed on the /dev/spidevX.Y file descriptor.
    // -------------------------------------------------------------------------

    if (!portIsOpened(port) && portOpen(port) != PORT_ERROR__NONE)
    {
        printf("ERROR: Failed to open '%s'.\n"
               "       Enable SPI in /boot/firmware/config.txt:\n"
               "         dtoverlay=spi0-1cs\n"
               "       then reboot and verify:  ls /dev/spidev0.*\n",
               SPI_DEVICE);
        spif.releasePort(port);
        return -2;
    }

    printf("SPI port opened: %s  speed=%u Hz  mode=%u  DR GPIO=%d\n\n",
           SPI_DEVICE, SPI_SPEED_HZ, SPI_PORT_DEFAULT_MODE, DR_GPIO_NUM);

    // -------------------------------------------------------------------------
    // STEP 6: Stop all existing device broadcasts
    //
    // Sends the stop-broadcast command to the IMX so it stops streaming any
    // messages left over from a previous session.  Always do this before
    // requesting a new set of messages.
    // -------------------------------------------------------------------------

    is_comm_stop_broadcasts_all_ports(port);

    // -------------------------------------------------------------------------
    // STEP 7: Register the ISB data callback
    //
    // All decoded InertialSense Binary (ISB) packets will be routed to
    // isbDataHandler().
    // -------------------------------------------------------------------------

    is_comm_register_port_isb_handler(port, isbDataHandler);

    // -------------------------------------------------------------------------
    // STEP 8: Request message broadcasts
    //
    // is_comm_get_data(port, dataId, offset, size, periodMultiple)
    //   offset = 0, size = 0  -> stream the complete message
    //   periodMultiple        -> output every (periodMultiple * NAV_DT_MS) ms
    //
    // Requested rates:
    //   DID_INS_1     MS_TO_PERIOD(14)   = 2  -> 2 * 7ms =  14 ms  (~71 Hz)
    //   DID_PIMU      MS_TO_PERIOD(14)   = 2  -> 2 * 7ms =  14 ms  (~71 Hz)
    //   DID_DEV_INFO  MS_TO_PERIOD(2000) = 286 -> 286 * 7ms ~= 2 s  (0.5 Hz)
    //   DID_GPS1_POS  MS_TO_PERIOD(500)  = 72  -> 72 * 7ms = 504 ms (~2 Hz)
    // -------------------------------------------------------------------------

    is_comm_get_data(port, DID_INS_1,    0, 0, MS_TO_PERIOD(14));
    is_comm_get_data(port, DID_PIMU,     0, 0, MS_TO_PERIOD(14));
    is_comm_get_data(port, DID_DEV_INFO, 0, 0, MS_TO_PERIOD(2000));
    is_comm_get_data(port, DID_GPS1_POS, 0, 0, MS_TO_PERIOD(500));

    printf("Streaming started.  Press Ctrl+C to stop.\n"
           "  DID_INS_1    @ ~14 ms  (~71 Hz)\n"
           "  DID_PIMU     @ ~14 ms  (~71 Hz)\n"
           "  DID_DEV_INFO @ ~2 s    (0.5 Hz)\n"
           "  DID_GPS1_POS @ ~504 ms (~2 Hz)\n\n");

    // -------------------------------------------------------------------------
    // STEP 9: Main loop - parse incoming messages
    //
    // is_comm_port_parse_messages() drives the receive pipeline:
    //   1. Calls portAvailable() which (because DR is configured) returns
    //      non-zero only while the DR GPIO is HIGH - no SPI transfer occurs
    //      when the IMX has nothing ready, keeping CPU and bus load low.
    //   2. When DR is asserted, reads bytes from the SPI port and feeds them
    //      through the ISB parser.
    //   3. Calls isbDataHandler() for each fully decoded packet.
    //
    // Run this loop at a rate fast enough to drain the receive buffer before
    // the next packet arrives (~1 ms is typical).
    // -------------------------------------------------------------------------

    while (g_running && portIsOpened(port))
    {
        is_comm_port_parse_messages(port);
        SLEEP_MS(1);
    }

    // -------------------------------------------------------------------------
    // Cleanup
    // -------------------------------------------------------------------------

    printf("\nShutting down...\n");
    is_comm_stop_broadcasts_all_ports(port);
    portClose(port);
    spif.releasePort(port);     // also unexports the DR GPIO via sysfs

    return 0;
}
