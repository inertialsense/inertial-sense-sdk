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

#define REMOTE_SOCAT_PORTS      // only does anything on linux

#ifdef REMOTE_SOCAT_PORTS
    #include <filesystem>
#endif

#include "util/util.h"

#include "PortManager.h"
#include "serialPort.h"
#include "serialPortPlatform.h"



port_handle_t SerialPortFactory::bindPort(uint16_t pType, const std::string& pName) {
    serial_port_t* serialPort = new serial_port_t;
    port_handle_t port = (port_handle_t)serialPort;

    *serialPort = {};
    serialPort->base.pnum = (uint16_t)PortManager::getInstance().getPortCount();
    serialPort->base.ptype = (pType | PORT_TYPE__UART | PORT_TYPE__COMM | PORT_FLAG__VALID);
    strncpy(serialPort->portName, pName.c_str(), pName.length());

    serialPortPlatformInit(port);

    serialPort->base.portOpen = SerialPortFactory::open_port;
    serialPort->base.portValidate = SerialPortFactory::validate_port;
    serialPort->pfnError = SerialPortFactory::onPortError;

    serialPort->baudRate = portOptions.defaultBaudRate;
    serialPort->blocking = portOptions.defaultBlocking

    debug_message("[DBG] Allocated new serial port '%s'\n", portName(port));
    return port;
}

bool SerialPortFactory::releasePort(port_handle_t port) {
    if (!port)
        return false;

    debug_message("[DBG] Releasing serial port '%s'\n", ((serial_port_t*)port)->portName);
    memset(port, 0, sizeof(serial_port_t));
    delete (serial_port_t*)port;

    return true;
}

bool SerialPortFactory::validatePort(uint16_t pType, const std::string& pName) {
#if PLATFORM_IS_WINDOWS
    char targetPath[256];
    return (QueryDosDeviceA(pName.c_str(), targetPath, sizeof(targetPath)) != 0);
#else   // Linux
    return validate_port__linux(pType, pName);
#endif
}

void SerialPortFactory::locatePorts(std::function<void(PortFactory*, uint16_t, std::string)> portCallback, const std::string& pattern, uint16_t pType) {
    std::regex matchPattern(pattern);
    getComPorts(ports);
    for (auto& np : ports) {
        auto match = std::regex_match(np, matchPattern);
        if (validatePort(PORT_TYPE__UART, np) && match)
            portCallback(this, PORT_TYPE__UART, np);
    }
}

int SerialPortFactory::onPortError(port_handle_t port, int errCode, const char *errMsg) {
    printf("%s :: Error %d : %s\n", portName(port), errCode, errMsg);
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
        case ENOTBLK:   /* Block device required */
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
 * @param ports a reference to a vector of strings, which will be populated with available serial ports
 * @return the number of ports found on the host
 */
int SerialPortFactory::getComPorts(std::vector<std::string>& ports)
{
    ports.clear();

#if PLATFORM_IS_WINDOWS

    char comPort[64];
    char targetPath[256];

    for (int i = 0; i < 256; i++) // checking ports from COM0 to COM255
    {
        snprintf(comPort, sizeof(comPort), "COM%d", i);
        if (QueryDosDeviceA(comPort, targetPath, 256))
        {
            ports.push_back(comPort);
        }
    }

#else   // Linux

    struct dirent **namelist;
    std::vector<std::string> comList8250;
    const char* sysdir = "/sys/class/tty/";


    #ifdef REMOTE_SOCAT_PORTS
        try
        {
            for (std::filesystem::recursive_directory_iterator i("/dev/remote"), end; i != end; ++i)
            {
                if (!std::filesystem::is_directory(i->path()) && std::filesystem::is_character_file(i->path()))
                {
                    ports.push_back(i->path().string());
                }
            }
        } catch (const std::filesystem::filesystem_error&) {}
    #endif


    // Scan through /sys/class/tty - it contains all tty-devices in the system
    int n = scandir(sysdir, &namelist, NULL, NULL);
    if (n < 0)
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
                register_comport__linux(ports, comList8250, devicedir);
            }
            free(namelist[n]);
            namelist[n] = nullptr;
        }
        free(namelist);
        namelist = nullptr;
    }

    // Only non-serial8250 has been added to comList without any further testing
    // serial8250-devices must be probe to check for validity
    probe_serial8250_comports__linux(ports, comList8250);

#endif

    return ports.size();
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
    if (pName.starts_with("/dev/remote"))
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
