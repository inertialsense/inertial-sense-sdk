/*
MIT LICENSE

Copyright 2014 Inertial Sense, LLC - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include "ISSerialPort.h"
#include "ISLogger.h"

cISSerialPort::cISSerialPort(serial_port_t* serial, bool ownsSerial, int timeout, bool blocking)
{
	m_serial = serial;
	m_ownsSerial = ownsSerial;
	m_timeout = timeout;
	m_blocking = blocking;
	if (serial == NULL)
	{
		m_serial = (serial_port_t*)MALLOC(sizeof(serial_port_t));
		memset(m_serial, 0, sizeof(serial_port_t));
		m_ownsSerial = true;
		serialPortPlatformInit(m_serial);
	}
}

cISSerialPort::~cISSerialPort()
{
	Close();
	if (m_ownsSerial)
	{
		FREE(m_serial);
	}
}

bool cISSerialPort::Open(const char* portName, int baudRate)
{
	return (serialPortOpen(m_serial, portName, baudRate, (int)m_blocking) != 0);
}

int cISSerialPort::Close()
{
	return serialPortClose(m_serial);
}

int cISSerialPort::Read(uint8_t* data, int dataLength)
{
	return serialPortReadTimeout(m_serial, data, dataLength, m_timeout);
}

int cISSerialPort::Write(const uint8_t* data, int dataLength)
{
	return serialPortWrite(m_serial, data, dataLength);
}

void cISSerialPort::GetComPorts(vector<string>& ports)
{
	ports.clear();

#if PLATFORM_IS_WINDOWS

	char comPort[64];
	char targetPath[256];

	for (int i = 0; i < 255; i++) // checking ports from COM0 to COM255
	{
		SNPRINTF(comPort, sizeof(comPort), "COM%d", i);
		if (QueryDosDeviceA(comPort, targetPath, 256))
		{
			ports.push_back(comPort);
		}
	}

#else

	cISLogger::GetAllFilesInDirectory("/dev", false, "^/dev/tty(USB|S)", ports);

#endif

}
