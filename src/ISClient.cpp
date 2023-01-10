/*
MIT LICENSE

Copyright (c) 2014-2023 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include "ISTcpClient.h"
#include "ISSerialPort.h"
#include "ISUtilities.h"
#include "ISClient.h"

using namespace std;

// connectionString usage:
// [type]:[protocol]:[ip/url]:[port]:[mountpoint]:[username]:[password]
// [TCP]:[RTCM3]:[ip/url]:[port]:[mountpoint]:[username]:[password]
// [TCP]:[RTCM3]:[ip/url]:[port]
// [SERIAL]:[RTCM3]:[serial port]:[baudrate]
cISStream* cISClient::OpenConnectionToServer(const string& connectionString, bool *enableGpggaForwarding)
{
	vector<string> pieces;
	splitString(connectionString, ':', pieces);
	if (pieces.size() < 4)
	{
		return NULLPTR;
	}

	string type     = pieces[0];	// TCP, SERIAL
	string protocol = pieces[1];	// RTCM3, UBLOX, IS

	if (type == "SERIAL")
	{
		cISSerialPort *clientStream = new cISSerialPort();

		string portName = pieces[2];	// /dev/ttyACM0
		string baudrate = pieces[3];	// 921600

		if (clientStream->Open(portName, atoi(baudrate.c_str())))
		{
			return clientStream;
		}
	}
	else if(type == "TCP" || type == "NTRIP")
	{
		cISTcpClient *clientStream = new cISTcpClient();

		string host     = (pieces[2].size()>0 ? pieces[2] : "127.0.0.1");	// ipAddr/URL
		string port     = (pieces[3]);
		string subUrl   = (pieces.size() > 4 ? pieces[4] : "");
		string username = (pieces.size() > 5 ? pieces[5] : "");
		string password = (pieces.size() > 6 ? pieces[6] : "");

		if (clientStream->Open(host, atoi(port.c_str()), 100) != 0)
		{
			return NULLPTR;
		}

		if (subUrl.size() != 0)
		{	// Connect NTRIP if specified - https://igs.bkg.bund.de/root_ftp/NTRIP/documentation/NtripDocumentation.pdf
			string userAgent = "NTRIP Inertial Sense";			// NTRIP standard requires "NTRIP" to be at the start of the User-Agent string.
			clientStream->HttpGet(subUrl, userAgent, username, password);
			if (enableGpggaForwarding != NULL)
			{
				*enableGpggaForwarding = true;
			}
		}

		return clientStream;
	}

	return NULLPTR;
}

