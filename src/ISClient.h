/*
MIT LICENSE

Copyright (c) 2014-2023 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#ifndef __IS_CLIENT__H__
#define __IS_CLIENT__H__

#include "ISStream.h"


class cISClient
{
private:

public:
	/**
	* Constructor
	*/
	cISClient(){}

	/**
	* Opens an ISStream (TCP or Serial Port) client
	* @param connectionString Colon delimited string containing connection info, 
	* [type]:[protocol]:[ip/url]:[port]:[mountpoint]:[username]:[password]
	*    type:		TCP, SERIAL
	*    protocol:	RTCM3, UBLOX, IS
	*	[type]:[protocol]:[ip/url]:[port]:[mountpoint]:[username]:[password]
	*	[TCP]:[RTCM3]:[ip/url]:[port]:[mountpoint]:[username]:[password]
	*	[TCP]:[RTCM3]:[ip/url]:[port]
	*	[SERIAL]:[RTCM3]:[serial port]:[baudrate]
	* @param enableGpggaForwarding Return value indicating that GPGGA GNSS messages should sent for VRS base stations. 
	* @return cISStream pointer if successful, otherwise NULLPTR
	*/
	static cISStream* OpenConnectionToServer(const std::string& connectionString, bool *enableGpggaForwarding=NULL);
};



#endif // __IS_CLIENT__H__
