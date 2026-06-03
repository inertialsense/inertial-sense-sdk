/**
 * @file PortLocator.h 
 * @brief ${BRIEF_DESC}
 *
 * @author Kyle Mallory on 2/20/25.
 * @copyright Copyright (c) 2025 Inertial Sense, Inc. All rights reserved.
 */

#ifndef IS_SDK__PORT_FACTORY_H
#define IS_SDK__PORT_FACTORY_H

#ifdef __cplusplus

#include <unordered_set>
#include <functional>
#include <string>
#include <cctype>

#include "core/base_port.h"
#include "core/msg_logger.h"
#include "core/spiPort.h"
#include "ISConstants.h"

#include "serialPort.h"

/**
 * PortFactory is an abstract class that is responsible for discovery available ports of a particular type.
 * There should be one implementation for each type of discoverable port.  This does NOT return a port,
 * only a name or some other identifier that can be used by the port implementation to create the port.
 * As an abstract class, this allows for third-party locators to be implemented for custom port types.
 */
class PortFactory {
public:
    // virtual ~PortFactory() = default;

    /**
     * @param portCallback - A function to be called when this Factory identifies a possible port; callback parameters are port-type and name
     */
    virtual void locatePorts(std::function<void(PortFactory*, uint16_t, std::string)> portCallback, const std::string& pattern = "", uint16_t pType = PORT_TYPE__UNKNOWN) = 0;

    /**
     * Checks to determine if "the essence" of a port is valid. This should probably not perform any operation on the port
     * that could impact the ability of the port to operate. Rather, perform any reasonable checks to confirm if the port
     * actually exists and can be operated on (ie, does the device exist in the OS, or does the target host respond to a ping?).
     * Note that this is a factory-specific function typically called by the PortManager in order to determine if a port is
     * no longer viable as a precursory check
     * @param pName the string identifier of the port - this must be unique and is required
     * @param pType the type of port to validate - this is optional, but maybe modified by the underlying implementation
     * @return true if the port is viable/valid, otherwise false
     */
    virtual bool validatePort(const std::string& pName, uint16_t pType = 0) = 0;

    /**
     * A function responsible for allocating the underlying port type and returning a port_handle_t to it
     * This function should NOT manipulate the underlying port, such as opening, etc.
     * @param pType the type of the port being allocated
     * @param pName the binding name of the port to be allocated.
     * @return a port_handle_t to the allocated port
     */
    virtual port_handle_t bindPort(const std::string& pName, uint16_t pType = 0) = 0;

    /**
     * A function responsible for freeing the allocated memory of the underlying port.
     * This function should NOT manipulate the underlying port, such as flushing, closing it, etc.
     * @param port the handle of the port to release
     * @return true if the port specified was a valid port, and it was successfully released, otherwise false.
     */
    virtual bool releasePort(port_handle_t port) = 0;

    /**
     * Called by PortManager when this factory's locatePorts() emitted a port that was
     * already bound under a *different* factory. PortManager skips this factory's bindPort()
     * to avoid duplicate allocations, but still gives the factory a chance to perform
     * post-bind decoration on the existing port — e.g. RelayPortFactory uses this to seed
     * a device hint into DeviceManager even when TcpPortFactory got there first to claim
     * the port handle. Default: no-op. The factory must NOT take ownership of the port.
     *
     * @param existing the port handle that was already bound under another factory
     * @param pName the canonical port name (URL or device path)
     * @param pType the port type bitmask
     */
    virtual void onPortAlias(port_handle_t existing, const std::string& pName, uint16_t pType) {
        (void)existing; (void)pName; (void)pType;
    }
};

class SerialPortFactory : public PortFactory {
public:
    struct {
        int defaultBaudRate = BAUDRATE_921600;
        bool defaultBlocking = false;
    } portOptions = {};

    static SerialPortFactory& getInstance() {
        static SerialPortFactory instance;
        return instance;
    }

    SerialPortFactory(SerialPortFactory const &) = delete;
    SerialPortFactory& operator=(SerialPortFactory const&) = delete;

    void locatePorts(std::function<void(PortFactory*, uint16_t, std::string)> portCallback, const std::string& pattern, uint16_t pType) override;

    bool validatePort(const std::string& pName, uint16_t pType = 0) override;

    port_handle_t bindPort(const std::string& pName, uint16_t pType = 0) override;

    bool releasePort(port_handle_t port) override;

    SerialPortFactory& setBaudRate(uint32_t baud) { portOptions.defaultBaudRate = baud; return *this; }
    SerialPortFactory& setBlocking(bool block) { portOptions.defaultBlocking = block; return *this; }

private:
    SerialPortFactory() = default;
    ~SerialPortFactory() = default;

    /**
     * A static function which is used to report errors that occur on a port created by this factory
     * @param port the port the error occurred on
     * @param errCode the error code (usually errno) of the error that occurred
     * @param errMsg an optional string message which describes the error the occurred
     * @return
     */
    static int onPortError(port_handle_t port, int errCode, const char *errMsg);

    std::vector<std::string> portNames = {};

    /**
     * An internal static function which identifies all available serial ports on the host device. It populates a referenced
     * std::vector<std::string> with their names, as suitable identifiers. This does NOT do any port_handle allocation, validation,
     * or other operations necessary to USE the port - it merely identifies them.
     * @param portNames a reference to a vector of strings which will be cleared, and populated with UART/Serial ports known to
     *  the host operating system.
     * @return the number of port names populated into the vector.
     */
    static int getComPorts(std::vector<std::string>& portNames);


#if PLATFORM_IS_LINUX
    static std::string get_driver__linux(const std::string& tty);
    static void register_comport__linux(std::vector<std::string>& comList, std::vector<std::string>& comList8250, const std::string& dir);
    static void probe_serial8250_comports__linux(std::vector<std::string>& comList, std::vector<std::string> comList8250);

    static bool validate_port__linux(uint16_t pType, const std::string& pName);
#elif PLATFORM_IS_WINDOWS
#endif

    // THESE ARE LOCALIZED HELPER FUNCTIONS to provide basic functionality that is not normally provided by the original SerialPort/SerialPortPlatform implementation
    // TODO: at some point, these should be moved into the implementation directly, and removed from the factory
    static int validate_port(port_handle_t port) { return SerialPortFactory::getInstance().validatePort(portName(port), portType(port)); }
    static int open_port(port_handle_t port) { return serialPortOpen(port, SERIAL_PORT(port)->portName, SERIAL_PORT(port)->baudRate, SERIAL_PORT(port)->blocking); }

};


/**
 * PortFactory implementation for Linux SPI character devices (spidev kernel driver).
 *
 * Port names accepted by this factory:
 *   - Plain device path:  "/dev/spidev0.0"
 *   - URL with options:   "spi:///dev/spidev0.0[b<hz>,d<gpio>,m<mode>]"
 *
 * Bracket options (all optional, comma-separated):
 *   b<HZ>    — SPI clock speed in Hz  (default: SPI_PORT_DEFAULT_SPEED_HZ)
 *   d<GPIO>  — data-ready GPIO number (default: -1, disabled)
 *   m<MODE>  — SPI mode 0-3 (CPOL/CPHA) (default: SPI_PORT_DEFAULT_MODE = 3)
 *
 * Options embedded in the port name override portOptions defaults on a per-port basis,
 * so multiple SPI devices with different speeds can be opened from a single factory instance.
 */
class SpiPortFactory : public PortFactory {
public:
    /** Default SPI parameters applied when binding a new port. Per-port bracket opts override these. */
    struct {
        uint32_t defaultSpeedHz  = SPI_PORT_DEFAULT_SPEED_HZ; ///< SPI clock speed in Hz
        uint8_t  defaultMode     = SPI_PORT_DEFAULT_MODE;     ///< SPI mode 0-3 (CPOL/CPHA)
        int      dataReadyGpio   = -1;                        ///< data-ready GPIO number, -1 = disabled
    } portOptions = {};

    /** Returns the process-wide singleton SpiPortFactory instance. */
    static SpiPortFactory& getInstance() {
        static SpiPortFactory instance;
        return instance;
    }

    SpiPortFactory(SpiPortFactory const&)            = delete;
    SpiPortFactory& operator=(SpiPortFactory const&) = delete;

    /**
     * Scans for SPI devices matching @p pattern and invokes @p portCallback for each match.
     * Accepts bare device paths or "spi://<devpath>[opts]" URLs; bracket opts are stripped for
     * matching but reconstructed in the name forwarded to bindPort so options are not lost.
     */
    void locatePorts(std::function<void(PortFactory*, uint16_t, std::string)> portCallback,
                     const std::string& pattern, uint16_t pType) override;

    /**
     * Returns true if @p pName refers to an existing SPI character device (stat + S_ISCHR check).
     * @p pName must be a plain device path — the "spi://" prefix and bracket opts must be stripped first.
     */
    bool validatePort(const std::string& pName, uint16_t pType = 0) override;

    /**
     * Allocates and initialises a spi_port_t for the given port name.
     * @p pName may be a plain device path or a "spi://<devpath>[opts]" URL; bracket opts override
     * portOptions defaults for this port only. Does NOT open the device.
     * @return allocated port handle, or nullptr if validatePort fails.
     */
    port_handle_t bindPort(const std::string& pName, uint16_t pType = 0) override;

    /**
     * Frees the spi_port_t allocated by bindPort. Does NOT flush or close the device first.
     * @return true if @p port was non-null and successfully freed.
     */
    bool releasePort(port_handle_t port) override;

    /** Sets the default SPI clock speed (Hz) used when no 'b' bracket opt is present. */
    SpiPortFactory& setSpeedHz(uint32_t hz)    { portOptions.defaultSpeedHz = hz;   return *this; }
    /** Sets the default SPI mode 0-3 (CPOL/CPHA) used when no 'm' bracket opt is present. */
    SpiPortFactory& setMode(uint8_t mode)      { portOptions.defaultMode    = mode; return *this; }
    /** Sets the default data-ready GPIO number used when no 'd' bracket opt is present. -1 disables. */
    SpiPortFactory& setDataReady(int gpio)     { portOptions.dataReadyGpio  = gpio; return *this; }

private:
    SpiPortFactory()  = default;
    ~SpiPortFactory() = default;

    /** Cached list of SPI device paths, refreshed on each locatePorts() call. */
    std::vector<std::string> portNames = {};

    /**
     * Parses a "spi://<devpath>[b<hz>,d<gpio>,m<mode>]" port string into a plain device path.
     * Out-params @p speedHz, @p mode, and @p dataReadyGpio are updated from bracket opts;
     * if a key is absent the caller's existing value (typically from portOptions) is preserved.
     * @return plain device path with the "spi://" prefix and brackets removed.
     */
    static std::string parseSpiPortString(const std::string& portStr, uint32_t& speedHz, uint8_t& mode, int& dataReadyGpio);

    /**
     * Static port-callback wrapper that routes base.portValidate calls back through the
     * factory's validatePort(), mirroring the SerialPortFactory pattern so PortManager
     * always uses factory-level OS existence checks rather than the spiPortValidate default.
     */
    static int validate_port(port_handle_t port) { return SpiPortFactory::getInstance().validatePort(portName(port), portType(port)); }

#if PLATFORM_IS_LINUX
    /**
     * Enumerates SPI character devices under /dev, populating @p names with their full paths.
     * Clears @p names before scanning. Returns the number of devices found.
     */
    static int getSpiDevices(std::vector<std::string>& names);
#endif
};

#endif

#endif // IS_SDK__PORT_FACTORY_H
