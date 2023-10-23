/*
MIT LICENSE

Copyright (c) 2014-2023 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include "ISSerialPort.h"
#include "ISLogger.h"
#include "ISFileManager.h"

#if PLATFORM_IS_LINUX
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/serial.h>
#endif

using namespace std;


cISSerialPort::cISSerialPort(serial_port_t* serial) : cISStream()
{
	if (serial != NULLPTR)
	{
		m_serial = *serial;
	}
	else
	{
		serialPortPlatformInit(&m_serial);
	}
	Close();
}

cISSerialPort::~cISSerialPort()
{
	Close();
}

bool cISSerialPort::Open(const std::string& portName, int baudRate, int timeout, bool blocking)
{
	m_timeout = timeout;
	m_blocking = blocking;
    return (serialPortOpen(&m_serial, portName.c_str(), baudRate, (int)m_blocking) != 0);
}

int cISSerialPort::Close()
{
	return serialPortClose(&m_serial);
}

int cISSerialPort::Read(void* data, int dataLength)
{
	return serialPortReadTimeout(&m_serial, (unsigned char*)data, dataLength, m_timeout);
}

int cISSerialPort::Write(const void* data, int dataLength)
{
	return serialPortWrite(&m_serial, (const unsigned char*)data, dataLength);
}


#if PLATFORM_IS_LINUX

static string get_driver(const string& tty) 
{
    struct stat st;
    string devicedir = tty;

    // Append '/device' to the tty-path
    devicedir += "/device";
    
    if (lstat(devicedir.c_str(), &st)==0 && S_ISLNK(st.st_mode)) 
	{	// Stat the devicedir and handle it if it is a symlink
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

static void register_comport( vector<string>& comList, vector<string>& comList8250, const string& dir) 
{
    // Get the driver the device is using
    string driver = get_driver(dir);
    
    if (driver.size() > 0) 
	{	// Skip devices without a driver
        string devfile = string("/dev/") + basename(dir.c_str());

        if (driver == "serial8250") 
		{	// Put serial8250-devices in a seperate list
            comList8250.push_back(devfile);
        } else
		{
            comList.push_back(devfile);
		}
    }
}

static void probe_serial8250_comports(vector<string>& comList, vector<string> comList8250) 
{
    struct serial_struct serinfo;
    vector<string>::iterator it = comList8250.begin();

    // Iterate over all serial8250-devices
    while (it != comList8250.end()) 
	{   // Try to open the device
        int fd = open((*it).c_str(), O_RDWR | O_NONBLOCK | O_NOCTTY);

        if (fd >= 0) 
		{   // Get serial_info
            if (ioctl(fd, TIOCGSERIAL, &serinfo)==0) 
			{   
                if (serinfo.type != PORT_UNKNOWN)
				{	// device type is no PORT_UNKNOWN we accept the port
                    comList.push_back(*it);
				}
            }
            close(fd);
        }
        it ++;
    }
}

#endif // #if PLATFORM_IS_LINUX

void cISSerialPort::GetComPorts(vector<string>& ports)
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

#else	// Linux

    struct dirent **namelist;
    vector<string> comList8250;
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
                string devicedir = sysdir;
                devicedir += namelist[n]->d_name;

                // Register the device
                register_comport(ports, comList8250, devicedir);
            }
            free(namelist[n]);
        }
        free(namelist);
    }

    // Only non-serial8250 has been added to comList without any further testing
    // serial8250-devices must be probe to check for validity
    probe_serial8250_comports(ports, comList8250);

#endif

#if 0
	cout << "Available ports: " << endl;
    for (int i = 0; i < ports.size(); i++)
	{
        cout << ports[i] << endl;
    }
#endif
}

std::string cISSerialPort::ConnectionInfo()
{
	return string(m_serial.port);
}
