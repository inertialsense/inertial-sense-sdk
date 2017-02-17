#ifdef _WIN32
/* See http://stackoverflow.com/questions/12765743/getaddrinfo-on-win32 */
#ifndef _WIN32_WINNT
#define _WIN32_WINNT 0x0501  /* Windows XP. */
#endif
#include <winsock2.h>
#include <Ws2tcpip.h>

#define PLATFORM_IS_WINDOWS 1
#define SOCKET_TYPE SOCKET

#else
/* Assume that any non-Windows platform uses POSIX-style sockets instead. */
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <arpa/inet.h>
#include <netdb.h>  /* Needed for getaddrinfo() and freeaddrinfo() */
#include <unistd.h> /* Needed for close() */
#include <errno.h>

#define PLATFORM_IS_WINDOWS 0
#define SOCKET_TYPE int

#endif

#include "ISTcpClient.h"
#include "ISUtilities.h"

#if PLATFORM_IS_WINDOWS

static uint32_t s_socketInitialized;

#endif

void ISSocketInitialize()
{

#if PLATFORM_IS_WINDOWS

	if (++s_socketInitialized != 1)
	{
		return;
	}
	WSADATA wsa_data;
	WSAStartup(MAKEWORD(1, 1), &wsa_data);

#endif

}

void ISSocketShutdown()
{

#if PLATFORM_IS_WINDOWS

	if (s_socketInitialized == 0)
	{
		return;
	}
	else if (--s_socketInitialized == 0)
	{
		WSACleanup();
	}

#endif

}

ISTcpClient::ISTcpClient()
{
	m_socket = 0;
	m_port = 0;
	ISSocketInitialize();
}

ISTcpClient::~ISTcpClient()
{
	Close();
	ISSocketShutdown();
}

int ISTcpClient::Open(const string& host, int port)
{
	Close();
	int status;
	this->m_host = host;
	m_port = port;
	char portString[64];
	SNPRINTF(portString, sizeof(portString), "%ld", (long)m_port);
	struct addrinfo* result = NULL;
	struct addrinfo hints = {};
	hints.ai_family = AF_UNSPEC;
	hints.ai_socktype = SOCK_STREAM;
	hints.ai_protocol = IPPROTO_TCP;
	status = getaddrinfo(m_host.c_str(), portString, &hints, &result);
	if (status != 0)
	{
		Close();
		return status;
	}
	m_socket = (uint64_t)socket(result->ai_family, result->ai_socktype, result->ai_protocol);
	if ((uint32_t*)m_socket == (uint32_t*)0xFFFFFFFF)
	{
		freeaddrinfo(result);
		Close();
		return -1;
	}

	const unsigned long SOCKET_READ_TIMEOUT_SEC = 1;

#if PLATFORM_IS_WINDOWS

	DWORD timeout = SOCKET_READ_TIMEOUT_SEC * 1000;
	status = setsockopt((SOCKET_TYPE)m_socket, SOL_SOCKET, SO_RCVTIMEO, (const char*)&timeout, sizeof(timeout));

#else

	struct timeval tv;
	tv.tv_sec = SOCKET_READ_TIMEOUT_SEC;
	tv.tv_usec = 0;
	status = setsockopt((SOCKET_TYPE)m_socket, SOL_SOCKET, SO_RCVTIMEO, (const char*)&tv, sizeof(tv));

#endif

	if (status != 0)
	{
		Close();
		return status;
	}

	status = connect((SOCKET_TYPE)m_socket, result->ai_addr, result->ai_addrlen);
	freeaddrinfo(result);
	if (status != 0)
	{
		Close();
		return status;
	}

	return status;
}

int ISTcpClient::Close()
{
	int status = 0;
	if (m_socket != 0)
	{

#if PLATFORM_IS_WINDOWS

		status = shutdown((SOCKET_TYPE)m_socket, SD_BOTH);
		if (status == 0)
		{
			status = closesocket((SOCKET_TYPE)m_socket);
		}

#else

		status = shutdown((SOCKET_TYPE)m_socket, SHUT_RDWR);
		if (status == 0)
		{
			status = close((SOCKET_TYPE)m_socket);
		}

#endif

	}

	m_socket = 0;

	return status;
}

int ISTcpClient::Read(uint8_t* data, int dataLength, bool blocking)
{
	if (!blocking)
	{

#if PLATFORM_IS_WINDOWS

		u_long bytesAvailable = 0;
		ioctlsocket((SOCKET_TYPE)m_socket, FIONREAD, &bytesAvailable);

#else

		int bytesAvailable = 0;
		ioctl((SOCKET_TYPE)m_socket, FIONREAD, &bytesAvailable);

#endif

		if (bytesAvailable == 0)
		{
			return 0;
		}
	}

	int count = recv((SOCKET_TYPE)m_socket, (char*)data, dataLength, 0);
	if (count < 0)
	{

#if PLATFORM_IS_WINDOWS

		DWORD err = GetLastError();
		if (err == SO_RCVTIMEO || err == WSAETIMEDOUT)
		{
			return 0;
		}

#else

		if (errno == EAGAIN || errno == EWOULDBLOCK)
		{
			return 0;
		}

#endif

		Close();
	}
	return count;
}

int ISTcpClient::Write(uint8_t* data, int dataLength)
{
	int count = send((SOCKET_TYPE)m_socket, (const char*)data, dataLength, 0);
	if (count < 0)
	{
		Close();
	}
	return count;
}