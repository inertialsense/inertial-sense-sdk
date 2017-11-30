/*
MIT LICENSE

Copyright 2014 Inertial Sense, LLC - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#ifndef _ISTCPSERVER__H__
#define _ISTCPSERVER__H__

#include <string>
#include <inttypes.h>
#include <vector>

using namespace std;

#include "ISTcpClient.h"

class cISTcpServer;

class iISTcpServerDelegate
{
protected:
	/*!
	* Executes when client data is received
	*/
        virtual void OnClientDataReceived(socket_t socket, uint8_t* data, int dataLength)
        {
            (void)socket;
            (void)data;
            (void)dataLength;
        }

	/*!
	* Executes when a client disconnects
	* @param socket the socket that disconnected
	*/
        virtual void OnClientDisconnected(socket_t socket)
        {
            (void)socket;
        }

	friend class cISTcpServer;
};

class cISTcpServer : public cISStreamWriter
{
public:
	/*!
	* Constructor
	*/
	cISTcpServer(iISTcpServerDelegate* delegate = NULL);

	/*!
	* Destructor
	*/
	virtual ~cISTcpServer();

	/*!
	* Closes, then opens a tcp server
	* @param ipAddress the ip address to bind to, empty for auto
	* @param port the port to bind to
	* @return 0 if success, otherwise an error code
	*/
	int Open(const string& ipAddress, int port);

	/*!
	* Close the server
	* @return 0 if success, otherwise an error code
	*/
	int Close();

	/*!
	* Update the server, receive connections, etc. Any clients that are disconnected will be closed and removed.
	*/
	void Update();

	/*!
	* Write data to all connected clients - any clients that are disconnected will be closed and removed
	* @param data the data to write
	* @param dataLength the number of bytes in data
	* @return the number of bytes written
	*/
	int Write(const uint8_t* data, int dataLength);

	/*!
	* Get whether the server is open
	* @return true if server open, false if not
	*/
	bool IsOpen() { return m_socket != 0; }

private:
	socket_t m_socket;
	vector<socket_t> m_clients;
	string m_ipAddress;
	int32_t m_port;
	iISTcpServerDelegate* m_delegate;
};

#endif
