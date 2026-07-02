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
#include <stdlib.h>
#include <limits.h>
#include <unistd.h>

/**
 * Performs an Linux OS-level check to determine the validity of a port, by checking for existence
 *  in /dev and in /sys/class/tty and that it has the correct driver attributes, and that the
 *  associate devices has correct file permissions, etc.
 * @param pName
 * @return
 */
bool SerialPortFactory::validate_port__linux(uint16_t pType, const std::string& pName) {
    struct stat st = {};

    // check first for /dev/<pName> and that its a character device
    if (! (!stat(pName.c_str(), &st) && S_ISCHR(st.st_mode) && st.st_rdev))
        return false;

#ifdef REMOTE_SOCAT_PORTS
    if (pName.rfind("/dev/remote", 0) == 0)
        return true;
#endif

    // Resolve the real hardware driver (walks past the kernel >= 6.8 "serial-base"
    // port/ctrl layers). An empty/"port" result means there is no genuine backing
    // driver, so the port is not valid.
    std::string driver = get_driver__linux(utils::string_format("/sys/class/tty/%s", basename(pName.c_str())));
    if (driver.empty() || driver == "port")
        return false;   // these are not valid ports

    if (driver == "serial8250") {
        // do additional validation
    }
    return true;
}

std::string SerialPortFactory::get_driver__linux(const std::string& tty)
{
    struct stat st = {};
    std::string devlink = tty + "/device";

    // The tty's "device" entry must be a symlink to the underlying device node
    if (lstat(devlink.c_str(), &st) != 0 || !S_ISLNK(st.st_mode))
        return "";

    // Resolve to the real device path so we can walk its parents if needed
    char real[PATH_MAX] = {};
    if (realpath(devlink.c_str(), real) == nullptr)
        return "";
    std::string devpath(real);

    // Reads the driver name (basename of the device's "driver" symlink target)
    auto driver_at = [](const std::string& p) -> std::string {
        char buffer[1024] = {};
        std::string driverlink = p + "/driver";
        if (readlink(driverlink.c_str(), buffer, sizeof(buffer) - 1) <= 0)
            return "";
        return basename(buffer);
    };

    std::string driver = driver_at(devpath);

    // Kernels >= 6.8 interpose a "serial-base" bus with intermediate "port" and
    // "ctrl" devices between the tty and the real controller, so the immediate
    // driver is always "port". Walk up the device parent chain past those layers
    // to recover the actual hardware driver (e.g. serial8250, uart-pl011). Older
    // kernels expose the real driver directly, so this loop is skipped.
    while (driver == "port" || driver == "ctrl")
    {
        size_t slash = devpath.find_last_of('/');
        if (slash == std::string::npos || slash == 0)
            break;
        devpath.resize(slash);
        driver = driver_at(devpath);
    }

    return driver;
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
    struct serial_struct serinfo = {};

    // The PORT_UNKNOWN filter below exists to reject the legacy static /dev/ttyS0..N
    // "phantom" nodes that the x86 ISA 8250 driver pre-creates without backing
    // hardware. Those phantoms only occur on PC/BIOS (non-device-tree) systems.
    // On a device-tree platform (e.g. Raspberry Pi) a serial8250 port that the
    // kernel bound a driver to is real hardware, even though its SoC UART reports
    // TIOCGSERIAL type == PORT_UNKNOWN (it never populates the legacy serial_struct).
    // So on device-tree systems we accept the port regardless of the reported type.
    bool deviceTreePlatform = (access("/proc/device-tree", F_OK) == 0);

    // Iterate over all serial8250-devices
    for (auto& dev : comList8250)
    {   // Try to open the device
        int fd = open(dev.c_str(), O_RDWR | O_NONBLOCK | O_NOCTTY);

        if (fd >= 0)
        {   // Get serial_info
            if (ioctl(fd, TIOCGSERIAL, &serinfo) == 0)
            {
                if (serinfo.type != PORT_UNKNOWN || deviceTreePlatform)
                {   // Accept known port types, or any driver-bound port on a device-tree platform
                    comList.push_back(dev);
                }
            }
            close(fd);
        }
    }
}

#endif // #if PLATFORM_IS_LINUX
