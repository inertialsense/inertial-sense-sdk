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

static const string s_base64_chars = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
static inline bool is_base64(unsigned char c)
{
	return (isalnum(c) || (c == '+') || (c == '/'));
}

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

cISTcpClient::cISTcpClient()
{
	m_socket = 0;
	m_port = 0;
	ISSocketInitialize();
}

cISTcpClient::~cISTcpClient()
{
	Close();
	ISSocketShutdown();
}

int cISTcpClient::Open(const string& host, int port)
{
	Close();
	int status;
	this->m_host = host;
	m_port = port;
	char portString[64];
	SNPRINTF(portString, sizeof(portString), "%ld", (long)m_port);
	addrinfo* result = NULL;
	addrinfo hints = addrinfo();
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
	if (m_socket == 0 || m_socket == (uint64_t)(~0))
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

	status = connect((SOCKET_TYPE)m_socket, result->ai_addr, (int)result->ai_addrlen);
	freeaddrinfo(result);
	if (status != 0)
	{
		Close();
		return status;
	}

	return status;
}

int cISTcpClient::Close()
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

int cISTcpClient::Read(uint8_t* data, int dataLength, bool blocking)
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

int cISTcpClient::Write(uint8_t* data, int dataLength)
{
	int count = send((SOCKET_TYPE)m_socket, (const char*)data, dataLength, 0);
	if (count < 0)
	{
		Close();
	}
	return count;
}

void cISTcpClient::HttpGet(const string& subUrl, const string& userAgent, const string& userName, const string& password)
{
	string msg = "GET /" + subUrl + " HTTP/1.1\r\n";
	msg += "User-Agent: " + userAgent + "\r\n";
	if (userName.size() != 0 && password.size() != 0)
	{
		string auth = userName + ":" + password;
		msg += "Authorization: Basic " + Base64Encode((const unsigned char*)auth.data(), (int)auth.size()) + "\r\n";
	}
	msg += "Accept: */*\r\nConnection: close\r\n\r\n";
	Write((uint8_t*)msg.data(), (int)msg.size());
}

string cISTcpClient::Base64Encode(const unsigned char* bytes_to_encode, unsigned int in_len)
{
	string ret;
	int i = 0;
	int j = 0;
	unsigned char char_array_3[3];
	unsigned char char_array_4[4];

	while (in_len--)
	{
		char_array_3[i++] = *(bytes_to_encode++);
		if (i == 3)
		{
			char_array_4[0] = (char_array_3[0] & 0xfc) >> 2;
			char_array_4[1] = ((char_array_3[0] & 0x03) << 4) + ((char_array_3[1] & 0xf0) >> 4);
			char_array_4[2] = ((char_array_3[1] & 0x0f) << 2) + ((char_array_3[2] & 0xc0) >> 6);
			char_array_4[3] = char_array_3[2] & 0x3f;
			for (i = 0; (i < 4); i++)
			{
				ret += s_base64_chars[char_array_4[i]];
			}
			i = 0;
		}
	}

	if (i)
	{
		for (j = i; j < 3; j++)
		{
			char_array_3[j] = '\0';
		}

		char_array_4[0] = (char_array_3[0] & 0xfc) >> 2;
		char_array_4[1] = ((char_array_3[0] & 0x03) << 4) + ((char_array_3[1] & 0xf0) >> 4);
		char_array_4[2] = ((char_array_3[1] & 0x0f) << 2) + ((char_array_3[2] & 0xc0) >> 6);
		char_array_4[3] = char_array_3[2] & 0x3f;

		for (j = 0; (j < i + 1); j++)
		{
			ret += s_base64_chars[char_array_4[j]];
		}

		while ((i++ < 3))
		{
			ret += '=';
		}
	}

	return ret;

}

string cISTcpClient::Base64Decode(const string& encoded_string)
{
	int in_len = (int)encoded_string.size();
	int i = 0;
	int j = 0;
	int in_ = 0;
	unsigned char char_array_4[4], char_array_3[3];
	string ret;

	while (in_len-- && (encoded_string[in_] != '=') && is_base64(encoded_string[in_]))
	{
		char_array_4[i++] = encoded_string[in_];
		in_++;
		if (i == 4)
		{
			for (i = 0; i < 4; i++)
			{
				char_array_4[i] = (unsigned char)s_base64_chars.find(char_array_4[i]);
			}
			char_array_3[0] = (char_array_4[0] << 2) + ((char_array_4[1] & 0x30) >> 4);
			char_array_3[1] = ((char_array_4[1] & 0xf) << 4) + ((char_array_4[2] & 0x3c) >> 2);
			char_array_3[2] = ((char_array_4[2] & 0x3) << 6) + char_array_4[3];
			for (i = 0; (i < 3); i++)
			{
				ret += char_array_3[i];
			}
			i = 0;
		}
	}

	if (i)
	{
		for (j = i; j < 4; j++)
		{
			char_array_4[j] = 0;
		}
		for (j = 0; j < 4; j++)
		{
			char_array_4[j] = (unsigned char)s_base64_chars.find(char_array_4[j]);
		}
		char_array_3[0] = (char_array_4[0] << 2) + ((char_array_4[1] & 0x30) >> 4);
		char_array_3[1] = ((char_array_4[1] & 0xf) << 4) + ((char_array_4[2] & 0x3c) >> 2);
		char_array_3[2] = ((char_array_4[2] & 0x3) << 6) + char_array_4[3];
		for (j = 0; (j < i - 1); j++)
		{
			ret += char_array_3[j];
		}
	}

	return ret;
}
