/**
 * @file PortLocator.cpp 
 * @brief ${BRIEF_DESC}
 *
 * @author Kyle Mallory on 2/20/25.
 * @copyright Copyright (c) 2025 Inertial Sense, Inc. All rights reserved.
 */

#include "PortFactory.h"

#include <string>
#include <vector>
#include <regex>

#if PLATFORM_IS_LINUX
#   include <dirent.h>
#   include <sys/stat.h>
#endif

// #define REMOTE_SOCAT_PORTS      // only does anything on linux

#ifdef REMOTE_SOCAT_PORTS
    #include <filesystem>
#endif

#include "util/util.h"
#include "ISUtilities.h"

#include "PortManager.h"
#include "serialPort.h"
#include "serialPortPlatform.h"



port_handle_t SerialPortFactory::bindPort(const std::string& pName, uint16_t pType) {
    if (!validatePort(pName, pType))
        return nullptr;

    serial_port_t* serialPort = new serial_port_t;
    port_handle_t port = (port_handle_t)serialPort;

    *serialPort = {};
    serialPortInit(port, (uint16_t)PortManager::getInstance().getPortCount(), (pType | PORT_TYPE__UART | PORT_TYPE__COMM), 0);
    strncpy(serialPort->portName, pName.c_str(), pName.length());

    // serialPort->base.portOpen = SerialPortFactory::open_port;
    serialPort->base.portValidate = SerialPortFactory::validate_port;
    serialPort->pfnError = SerialPortFactory::onPortError;

    serialPort->baudRate = portOptions.defaultBaudRate;
    serialPort->blocking = portOptions.defaultBlocking;

    portValidate(port);

    log_more_debug(IS_LOG_PORT_FACTORY, "Allocated new serial port '%s'", portName(port));
    return port;
}

bool SerialPortFactory::releasePort(port_handle_t port) {
    if (!port)
        return false;

    log_more_debug(IS_LOG_PORT_FACTORY, "Releasing serial port '%s'", ((serial_port_t*)port)->portName);
    memset(port, 0, sizeof(serial_port_t));
    delete (serial_port_t*)port;

    return true;
}

bool SerialPortFactory::validatePort(const std::string& pName, uint16_t pType) {
#if PLATFORM_IS_WINDOWS
    char targetPath[256];
    return (QueryDosDeviceA(pName.c_str(), targetPath, sizeof(targetPath)) != 0);
#elif PLATFORM_IS_LINUX
    return validate_port__linux(pType, pName);
#endif
}

void SerialPortFactory::locatePorts(std::function<void(PortFactory*, uint16_t, std::string)> portCallback, const std::string& pattern, uint16_t pType) {
    std::regex matchPattern(pattern);
    getComPorts(portNames);
    for (auto& name : portNames) {
        auto match = std::regex_match(name, matchPattern);
        if (validatePort(name, PORT_TYPE__UART) && match)
            portCallback(this, PORT_TYPE__UART, name);
    }
}

int SerialPortFactory::onPortError(port_handle_t port, int errCode, const char *errMsg) {
    // const char* portStr = portName(port);
    // const char* safeErrMsg = errMsg ? errMsg : "";

    static int lastErrorCode = 0;       // the previous error code
    static int repeatCount = 0;         // number of time the same code has repeated
    static uint32_t lastErrorMs = 0;    // the time when the lastErrorCode changed to the current error code

    if (errCode != lastErrorCode) {
        repeatCount = 0;
        lastErrorMs = current_timeMs();
        lastErrorCode = errCode;

        // General errors should already be reported by the underlying port implementation (if IS_LOG_PORT is configured)
        // log_error(IS_LOG_PORT_FACTORY, "%s :: Error %d : %s", portStr, errCode, safeErrMsg);
    } else {
        // Split the printf into two calls (helps avoid inlining inference)
        // log_error(IS_LOG_PORT_FACTORY, "%s :: Error %d : %s (%d count)", portStr, errCode, safeErrMsg, ++repeatCount);

        if ((current_timeMs() - lastErrorMs > 30000) && (repeatCount++ >= 10)){
            // any error which repeats for more than 30 seconds, and more than 10 times, close & invalidate
            portClose(port);
            portInvalidate(port);
            return 0;
        }
    }

    // decide which of these should result in a port-closure, vs a port invalid, vs nothing...
    switch (errCode) {
        // close but don't invalidate
        case EIO:       /* I/O error */
        case ENXIO:     /* No such device or address */
        case E2BIG:     /* Argument list too long */
        case ENOEXEC:   /* Exec format error */
        case EBADF:     /* Bad file number */
        case ECHILD:    /* No child processes */
        case ENOMEM:    /* Out of memory */
        case EACCES:    /* Permission denied */
        case EFAULT:    /* Bad address */
        case ENFILE:    /* File table overflow */
        case EMFILE:    /* Too many open files */
        case EFBIG:     /* File too large */
        case ENOSPC:    /* No space left on device */
        case ESPIPE:    /* Illegal seek */
        case EROFS:     /* Read-only file system */
        case EMLINK:    /* Too many links */
            portClose(port);
            break;

            // close and invalidate
        case ENOENT:    /* No such file or directory */
        case ESRCH:     /* No such process */
        case ENODEV:    /* No such device */
        case ENOTDIR:   /* Not a directory */
        case EISDIR:    /* Is a directory */
#if PLATFORM_IS_LINUX
        case ENOTBLK:   /* Block device required */
#endif
        case EPIPE:     /* Broken pipe */
            portClose(port);
            portInvalidate(port);
            break;


        // ignore
        case EBUSY:     /* Device or resource busy */
        case EAGAIN:    /* Try again */
        case EPERM:     /* Operation not permitted */
        case EINTR:     /* Interrupted system call */

        case EEXIST:    /* File exists */
        case EXDEV:     /* Cross-device link */
        case EINVAL:    /* Invalid argument */
        case ENOTTY:    /* Not a typewriter */
        case ETXTBSY:   /* Text file busy */
        case EDOM:      /* Math argument out of domain of func */
        case ERANGE:    /* Math result not representable */
        default:
            // do nothing (try again??)
            break;
    }
    return 0;
}

/**
 * Populates a vector of string identifiers for all available Serial/TTY/UART devices on the host system.
 * This does not open, access, or configure the devices, nor does it make any guarantee about the availability
 * of the ports (only that the OS has registered/enumerated it).
 * @param portNames a reference to a vector of strings, which will be populated with names identifiers of available ports
 * @return the number of ports found on the host
 */
int SerialPortFactory::getComPorts(std::vector<std::string>& portNames)
{
    portNames.clear();

#if PLATFORM_IS_WINDOWS

    char comPort[64];
    char targetPath[256];

    for (int i = 0; i < 256; i++) // checking ports from COM0 to COM255
    {
        snprintf(comPort, sizeof(comPort), "COM%d", i);
        if (QueryDosDeviceA(comPort, targetPath, 256))
        {
            portNames.push_back(comPort);
        }
    }

#elif PLATFORM_IS_LINUX

    struct dirent **namelist = NULL;
    std::vector<std::string> comList8250;
    const char* sysdir = "/sys/class/tty/";


    #ifdef REMOTE_SOCAT_PORTS
        try
        {
            for (std::filesystem::recursive_directory_iterator i("/dev/remote"), end; i != end; ++i)
            {
                if (!std::filesystem::is_directory(i->path()) && std::filesystem::is_character_file(i->path()))
                {
                    portNames.push_back(i->path().string());
                }
            }
        } catch (const std::filesystem::filesystem_error&) {}
    #endif


    // Scan through /sys/class/tty - it contains all tty-devices in the system
    int n = scandir(sysdir, &namelist, NULL, NULL);
    if ((n < 0) || (namelist == NULL))
    {
        perror("scandir");
    }
    else
    {
        while (n--)
        {
            if (strcmp(namelist[n]->d_name,"..") && strcmp(namelist[n]->d_name,"."))
            {   // Construct full absolute file path
                std::string devicedir = sysdir;
                devicedir += namelist[n]->d_name;

                // Register the device
                register_comport__linux(portNames, comList8250, devicedir);
            }
            free(namelist[n]);
            namelist[n] = nullptr;
        }
        free(namelist);
        namelist = nullptr;
    }

    // Only non-serial8250 has been added to comList without any further testing
    // serial8250-devices must be probe to check for validity
    probe_serial8250_comports__linux(portNames, comList8250);

#endif

    return portNames.size();
}


#if PLATFORM_IS_LINUX
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/serial.h>

/**
 * Performs an Linux OS-level check to determine the validity of a port, by checking for existence
 *  in /dev and in /sys/class/tty and that it has the correct driver attributes, and that the
 *  associate devices has correct file permissions, etc.
 * @param pName
 * @return
 */
bool SerialPortFactory::validate_port__linux(uint16_t pType, const std::string& pName) {
    struct stat st;
    char buffer[1024];

    // check first for /dev/<pName> and that its a character device
    if (! (!stat(pName.c_str(), &st) && S_ISCHR(st.st_mode) && st.st_rdev))
        return false;

#ifdef REMOTE_SOCAT_PORTS
    if (pName.rfind("/dev/remote", 0) == 0)
        return true;
#endif

    std::string devdir = utils::string_format("/sys/class/tty/%s/device/driver", basename(pName.c_str()));
    if (! (!lstat(devdir.c_str(), &st) && S_ISLNK(st.st_mode) && st.st_nlink))
        return false;

    memset(buffer, 0, sizeof(buffer));
    if (readlink(devdir.c_str(), buffer, sizeof(buffer)) <= 0)
        return false;

    std::string driver = std::string(basename(buffer));
    if (driver == "port")
        return false;   // these are not valid ports

    if (driver == "serial8250") {
        // do additional validation
    }
    return true;
}

std::string SerialPortFactory::get_driver__linux(const std::string& tty)
{
    struct stat st;
    std::string devicedir = tty;

    // Append '/device' to the tty-path
    devicedir += "/device";

    if (lstat(devicedir.c_str(), &st)==0 && S_ISLNK(st.st_mode))
    {   // Stat the devicedir and handle it if it is a symlink
        char buffer[1024];
        memset(buffer, 0, sizeof(buffer));

        // Append '/driver' and return basename of the target
        devicedir += "/driver";

        if (readlink(devicedir.c_str(), buffer, sizeof(buffer)) > 0)
        {
            return basename(buffer);
        }
    }
    return "";
}

void SerialPortFactory::register_comport__linux(std::vector<std::string>& comList, std::vector<std::string>& comList8250, const std::string& dir)
{
    // Get the driver the device is using
    std::string driver = get_driver__linux(dir);

    if (driver.size() > 0)
    {   // Skip devices without a driver
        std::string devfile = std::string("/dev/") + basename(dir.c_str());

        if (driver == "serial8250")
        {   // Put serial8250-devices in a seperate list
            comList8250.push_back(devfile);
        }
        else if (driver != "port")
        {
            comList.push_back(devfile);
        }
    }
}

void SerialPortFactory::probe_serial8250_comports__linux(std::vector<std::string>& comList, std::vector<std::string> comList8250)
{
    struct serial_struct serinfo;
    std::vector<std::string>::iterator it = comList8250.begin();

    // Iterate over all serial8250-devices
    while (it != comList8250.end())
    {   // Try to open the device
        int fd = open((*it).c_str(), O_RDWR | O_NONBLOCK | O_NOCTTY);

        if (fd >= 0)
        {   // Get serial_info
            if (ioctl(fd, TIOCGSERIAL, &serinfo)==0)
            {
                if (serinfo.type != PORT_UNKNOWN)
                {   // device type is no PORT_UNKNOWN we accept the port
                    comList.push_back(*it);
                }
            }
            close(fd);
        }
        it ++;
    }
}

#endif // #if PLATFORM_IS_LINUX

// =======================================================================
// SpiPortFactory
// =======================================================================

/**
 * Parses a "spi://<devpath>[b<hz>,d<gpio>,m<mode>]" port string into a plain device path.
 * Bracket key-value pairs update the corresponding out-params; absent keys leave the caller's
 * value unchanged so portOptions defaults flow through unmodified.
 */
std::string SpiPortFactory::parseSpiPortString(const std::string& portStr, uint32_t& speedHz, uint8_t& mode, int& dataReadyGpio)
{
    std::string dev = portStr;
    if (dev.size() > 6 && dev.substr(0, 6) == "spi://")
        dev = dev.substr(6); // strip "spi://", leaving "/dev/spi0.0[...]"

    size_t bracket = dev.find('[');
    if (bracket != std::string::npos)
    {
        std::string opts = dev.substr(bracket + 1);
        dev = dev.substr(0, bracket);
        size_t close = opts.find(']');
        if (close != std::string::npos) opts.resize(close);

        size_t pos = 0;
        while (pos < opts.size())
        {
            size_t comma = opts.find(',', pos);
            std::string tok = opts.substr(pos, comma == std::string::npos ? std::string::npos : comma - pos);
            pos = (comma == std::string::npos) ? opts.size() : comma + 1;
            if (tok.size() < 2) continue;
            const char* val = tok.c_str() + 1;
            switch (tok[0])
            {
            case 'b': speedHz      = (uint32_t)strtoul(val, nullptr, 10); break;
            case 'd': dataReadyGpio = (int)strtol(val, nullptr, 10);       break;
            case 'm': mode         = (uint8_t)strtoul(val, nullptr, 10);  break;
            }
        }
    }
    return dev;
}

/**
 * Allocates a spi_port_t, parsing @p pName for embedded bracket opts that override portOptions
 * defaults. Wires validate_port as the per-port portValidate callback so PortManager uses
 * factory-level OS existence checks. Does NOT open the device.
 */
port_handle_t SpiPortFactory::bindPort(const std::string& pName, uint16_t pType)
{
    uint32_t speedHz      = portOptions.defaultSpeedHz;
    uint8_t  mode         = portOptions.defaultMode;
    int      dataReadyGpio = portOptions.dataReadyGpio;
    std::string devPath   = parseSpiPortString(pName, speedHz, mode, dataReadyGpio);

    if (!validatePort(devPath, pType))
        return nullptr;

    spi_port_t* spiPort = new spi_port_t;
    port_handle_t port  = (port_handle_t)spiPort;

    *spiPort = {};
    spiPortInit(port,
                (uint16_t)PortManager::getInstance().getPortCount(),
                devPath.c_str(),
                speedHz,
                mode,
                PORT_TYPE__COMM);

    spiPort->base.portValidate = SpiPortFactory::validate_port;

    if (dataReadyGpio >= 0)
        spiPortSetDataReady(port, dataReadyGpio);

    portValidate(port);

    log_more_debug(IS_LOG_PORT_FACTORY, "Allocated new SPI port '%s'", portName(port));
    return port;
}

/** Frees the spi_port_t allocated by bindPort. Does not flush or close the device. */
bool SpiPortFactory::releasePort(port_handle_t port)
{
    if (!port) return false;

    log_more_debug(IS_LOG_PORT_FACTORY, "Releasing SPI port '%s'", SPI_PORT(port)->name);
    memset(port, 0, sizeof(spi_port_t));
    delete (spi_port_t*)port;
    return true;
}

/**
 * Returns true if @p pName exists in the filesystem and is a character device.
 * @p pName must be a plain device path — call parseSpiPortString() first if the
 * caller may have a "spi://..." URL.
 */
bool SpiPortFactory::validatePort(const std::string& pName, uint16_t pType)
{
#if PLATFORM_IS_LINUX
    struct stat st;
    return (stat(pName.c_str(), &st) == 0 && S_ISCHR(st.st_mode));
#else
    (void)pName; (void)pType;
    return false;
#endif
}

/**
 * Enumerates SPI devices and invokes @p portCallback for each one matching @p pattern.
 * If @p pattern carries the "spi://" prefix, it is stripped for device-path matching and
 * then reconstructed on the matched name before the callback fires, so bindPort receives
 * the full URL (including bracket opts) and can parse per-port parameters itself.
 * If @p pattern is a bare device path with brackets (e.g. "/dev/spi0.0[opts]") the brackets
 * are stripped before regex matching and the opts are still forwarded to bindPort.
 * A plain device path with no brackets is used as-is for regex matching.
 */
void SpiPortFactory::locatePorts(std::function<void(PortFactory*, uint16_t, std::string)> portCallback,
                                  const std::string& pattern, uint16_t pType)
{
#if PLATFORM_IS_LINUX
    static const std::string SPI_PREFIX = "spi://";
    bool hasSpiPrefix = (pattern.size() > SPI_PREFIX.size() && pattern.substr(0, SPI_PREFIX.size()) == SPI_PREFIX);
    std::string devPattern = hasSpiPrefix ? pattern.substr(SPI_PREFIX.size()) : pattern;
    std::string spiOpts;

    // Bracket opts are never part of a device path; strip them unconditionally.
    // This also handles bare "/dev/spi0.0[opts]" passed without the spi:// prefix.
    size_t bracket = devPattern.find('[');
    if (bracket != std::string::npos)
    {
        spiOpts    = devPattern.substr(bracket);
        devPattern = devPattern.substr(0, bracket);
        hasSpiPrefix = true; // ensure fullName is reconstructed for bindPort
    }

    std::regex matchPattern(devPattern.empty() ? ".*" : devPattern);
    getSpiDevices(portNames);
    for (auto& name : portNames)
    {
        if (validatePort(name, PORT_TYPE__SPI) && std::regex_match(name, matchPattern))
        {
            // Preserve the full spi:// string so bindPort can parse the opts.
            std::string fullName = hasSpiPrefix ? (SPI_PREFIX + name + spiOpts) : name;
            portCallback(this, PORT_TYPE__SPI, fullName);
        }
    }
#else
    (void)portCallback; (void)pattern; (void)pType;
#endif
}

#if PLATFORM_IS_LINUX
/**
 * Scans /dev for SPI character devices, populating @p names with their full paths
 * (e.g. "/dev/spi0.0"). Clears @p names before scanning.
 * @return number of devices found.
 */
int SpiPortFactory::getSpiDevices(std::vector<std::string>& names)
{
    names.clear();

    // spidev devices appear as /dev/spidevB.C (bus B, chip-select C)
    DIR* dir = opendir("/dev");
    if (!dir) return 0;

    struct dirent* entry;
    while ((entry = readdir(dir)) != nullptr)
    {
        if (strncmp(entry->d_name, "spidev", 6) == 0)
            names.push_back(std::string("/dev/") + entry->d_name);
    }
    closedir(dir);

    std::sort(names.begin(), names.end());
    return (int)names.size();
}
#endif
