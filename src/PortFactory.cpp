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

#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/serial.h>

#include "PortManager.h"
#include "serialPort.h"
#include "serialPortPlatform.h"

port_handle_t SerialPortFactory::bindPort(u_int16_t pType, std::string pName) {
    // TODO :: TRYING SOMETHING DIFFERENT
    //   -- rather than using new, m_serialPorts will allocate a new port using a copy constructor.
    //   Can we dereference the internal copy in the vector to a port_handle_t? -- this avoids us using new/delete or malloc/free
    serial_port_t* serialPort = new serial_port_t;
    *serialPort = {};
    serialPort->base.pnum = (uint16_t)PortManager::getInstance().getPortCount();
    serialPort->base.ptype = (pType | PORT_TYPE__UART | PORT_TYPE__COMM | PORT_FLAG__VALID);
    strncpy(serialPort->portName, pName.c_str(), pName.length());

    serialPort->pfnError = SerialPortFactory::onPortError;

    port_handle_t port = (port_handle_t)serialPort;
    serialPortPlatformInit(port);

    return port;
}

bool SerialPortFactory::releasePort(port_handle_t port) {
    if (!port)
        return false;

    // its annoying that its not "easy" to remove a vector element by value, especially when the value is complex
    //    auto p = std::find(begin(), end(), port);
    //    if ((p == end()) || (*p != port))
    //        return false;

    // TODO: there should be some kind of a callback/notify system where a port can directly notify listeners that its being closed/destroyed
    // serialPortClose(port);

    /*
     -- We don't do ANYTHING with the device at this level... that should be handle by the port callback.
    ISDevice* device = getDevice(port);
    if (device && releaseDevice) {
        device->port = NULL;    // so port to NULL so releaseDevice() below won't double-free it.
        InertialSense::releaseDevice(device, false);
    }
    // PortManager::erase(port);
    */

    // TODO: Don't need this, if we stick with the vector/copy allocation/initialization
    memset(port, 0, sizeof(serial_port_t));
    delete (serial_port_t*)port;

    return true;
}

void SerialPortFactory::locatePorts(std::function<void(PortFactory*, std::string)> portCallback, const std::string& pattern, uint16_t pType) {
    std::regex matchPattern(pattern);
    getComPorts(ports);
    for (auto& np : ports) {
        auto match = std::regex_match(np, matchPattern);
        if (match)
            portCallback(this, np);
    }
}

int SerialPortFactory::onPortError(port_handle_t port, int errCode, const char *errMsg) {
    printf("%s\n", errMsg);
    return 0;
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

#if 0
    cout << "Available ports: " << endl;
    for (int i = 0; i < ports.size(); i++)
    {
        cout << ports[i] << endl;
    }
#endif
    return ports.size();
}


#endif // #if PLATFORM_IS_LINUX
