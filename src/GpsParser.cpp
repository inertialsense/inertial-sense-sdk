/*
MIT LICENSE

Copyright 2014 Inertial Sense, LLC - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include "GpsParser.h"
#include "com_manager.h"

#if defined(__GNUC__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#endif

#define UBLOX_PREAMBLE1 0xb5
#define UBLOX_PREAMBLE2 0x62
#define UBLOX_HEADER_SIZE 6
#define UBLOX_CHECKSUM_SIZE 2

#if defined(RTK_EMBEDDED)

#define INIT_M_RTCM init_rtcm(&m_rtcm, 0, 1); SetReceiverIndex(RECEIVER_INDEX_BASE_STATION);
#define ALLOCATE_M_RTCM if (!m_rtcm.ownsBuff) { m_rtcm.buff = (unsigned char*)RTK_MALLOC(MAXRTCMLEN); m_rtcm.ownsBuff = 1; }
#define INIT_M_RAW(fmt) init_raw(&m_raw, fmt, 0); SetReceiverIndex(RECEIVER_INDEX_ROVER);
#define ALLOCATE_M_RAW if (!m_raw.ownsBuff) { m_raw.buff = (unsigned char*)RTK_MALLOC(MAXRAWLEN); m_raw.ownsBuff = 1; }
#define DECLARE_RTCM_DESTRUCTOR(class_name) ~class_name() OVERRIDE { free_rtcm(&m_rtcm); }
#define DECLARE_RAW_DESTRUCTOR(class_name) ~class_name() OVERRIDE { free_raw(&m_raw); }
#define DECLARE_M_RTCM rtcm_t m_rtcm;
#define DECLARE_M_RAW raw_t m_raw;

extern "C"
{
	extern int decode_rtcm3(rtcm_t *rtcm);
	extern int decode_ubx(raw_t* raw, int doChecksum);
	gtime_t g_gps_latest_time;
}

#else

#define INIT_M_RTCM m_rtcm.len = 0; m_rtcm.nbyte = 0;
#define ALLOCATE_M_RTCM
#define INIT_M_RAW(fmt) m_raw.len = 0; m_raw.nbyte = 0;
#define ALLOCATE_M_RAW
#define DECLARE_RTCM_DESTRUCTOR(c)
#define DECLARE_RAW_DESTRUCTOR(c)
#define MAXRTCMLEN 1200
#define MAXRAWLEN 4096
#define DECLARE_M_RTCM struct { uint8_t buff[MAXRTCMLEN]; int len; int nbyte; } m_rtcm;
#define DECLARE_M_RAW struct { uint8_t buff[MAXRAWLEN]; int len; int nbyte; } m_raw;

#endif

class cInertialSenseParser : public cGpsParser
{
public:
	cInertialSenseParser(iGpsParserDelegate* delegate) : cGpsParser(delegate)
	{
		m_len = 0;
	}

	void WriteByte(uint8_t b) OVERRIDE
	{
		// note that no checksum is done, we assume a valid packet which should be the case 99.99% of the time
		// other types of parsers cannot do this because they do not have the concept of an end packet byte
		if (b == PSC_START_BYTE)
		{
			// write the start byte if we haven't written any bytes
			if (m_len == 0)
			{
				// write the start byte
				m_buf[m_len++] = b;
			}
			else
			{
				// clear, corrupt data
				m_len = 0;
			}
		}
		else if (m_len == _ARRAY_ELEMENT_COUNT(m_buf))
		{
			// corrupt data
			m_len = 0;
		}
		else
		{
			// write the byte
			m_buf[m_len++] = b;

			if (b == PSC_END_BYTE)
			{
				// data packet must be 4 bytes header + 4 bytes footer + 12 bytes data header + min 4 bytes data
				if (m_len >= 24)
				{
					GetDelegate()->OnPacketReceived(this, m_buf, m_len);
				}
				m_len = 0;
			}
		}
	}

private:
	uint8_t m_buf[PKT_BUF_SIZE];
	uint32_t m_len;
};

class cRtcmParser : public cGpsParser
{
public:
	cRtcmParser(iGpsParserDelegate* delegate) : cGpsParser(delegate)
	{
		INIT_M_RTCM;
	}

	DECLARE_RTCM_DESTRUCTOR(cRtcmParser);

	void WriteByte(uint8_t b) OVERRIDE
	{
		ALLOCATE_M_RTCM;

		// if we have no bytes yet, check for header
		// if we have exactly 3 bytes, we have enough to calculate the length
		// fewer than 3 bytes and we want to exit out early
		if (m_rtcm.len <= 3)
		{
			// check for header byte
			if (m_rtcm.len == 0)
			{
				// 0xD3 is the first byte in an RTCM3 message
				if (b != 0xD3)
				{
					// corrupt data
					return;
				}
			}
			else if (m_rtcm.len == 3)
			{
				uint32_t msgLength = getBitsAsUInt32(m_rtcm.buff, 14, 10);
				if (msgLength > 1023)
				{
					// corrupt data
					m_rtcm.len = 0;
					return;
				}
				else
				{
					// set the bytes left to read (nbyte)
					// use + 2 because we already read the first byte of the message here to bring us
					// to a total length of 4
					m_rtcm.nbyte = msgLength + 2;
				}
			}

			// append the byte
			m_rtcm.buff[m_rtcm.len++] = b;
			return;
		}

		// append the byte
		m_rtcm.buff[m_rtcm.len++] = b;

		// see if we are done reading bytes, if so calculate CRC and forward the message on to the uINS if it is valid
		if (--m_rtcm.nbyte == 0)
		{
			// if the message is large enough, forward it on
			if (m_rtcm.len > 5)
			{
				// calculate length without crc bytes
				uint32_t lenWithoutCrc = m_rtcm.len - 3;

				// the CRC calculation does not include the 3 CRC bytes at the end for obvious reasons
				uint32_t actualCRC = calculate24BitCRCQ(m_rtcm.buff, lenWithoutCrc);

				// get the CRC from the last 3 bytes of the message
				uint32_t correctCRC = getBitsAsUInt32(m_rtcm.buff + lenWithoutCrc, 0, 24);

				// if valid CRC, forward mesasge to uINS
				if (actualCRC == correctCRC)
				{
					if (GetDelegate()->OnPacketReceived(this, m_rtcm.buff, m_rtcm.len))
					{
						// remove checksum from length
						m_rtcm.len = lenWithoutCrc;
						ParseMessage(m_rtcm.buff, m_rtcm.len);
					}
				}
				else
				{
					// corrupt data
				}
			}
			m_rtcm.len = 0;
		}
	}

	void ParseMessage(const uint8_t* data, int dataLength) OVERRIDE
	{
        (void)data;
        (void)dataLength;

#if defined(RTK_EMBEDDED)

#if PLATFORM_IS_ARM

		m_rtcm.time = g_gps_latest_time;

#else

		if (g_gps_latest_time.time == 0)
		{
			m_rtcm.time = utc2gpst(timeget());
		}
		else
		{
			m_rtcm.time = g_gps_latest_time;
		}

#endif

		if (m_rtcm.time.time == 0)
		{
			// cannot parse until start time is set
			return;
		}
		else if (m_rtcm.buff != data)
		{
			if (m_rtcm.ownsBuff)
			{
				RTK_FREE(m_rtcm.buff);
			}
			m_rtcm.ownsBuff = 0;
			m_rtcm.buff = (unsigned char*)data;
		}
		m_rtcm.len = dataLength;
		switch (decode_rtcm3(&m_rtcm))
		{
		case DATA_TYPE_OBSERVATION:
			for (obsd_t* ptr = m_rtcm.obs.data, *ptrEnd = m_rtcm.obs.data + m_rtcm.obs.n; ptr != ptrEnd; ptr++)
			{
				ptr->rcv = GetReceiverIndex();
			}
			GetDelegate()->OnObservationReceived(this, &m_rtcm.obs);
			break;

		case DATA_TYPE_EPHEMERIS:
			int prn;
			switch (satsys(m_rtcm.ephsat, &prn))
			{
			case SYS_GPS:
			case SYS_GAL:
			case SYS_QZS:
			case SYS_CMP:
				GetDelegate()->OnGpsEphemerisReceived(this, &m_rtcm.eph, prn);
				break;

			case SYS_GLO:
				GetDelegate()->OnGlonassEphemerisReceived(this, &m_rtcm.geph, prn);
				break;

			}
			break;

		case DATA_TYPE_ANTENNA_POSITION:
			GetDelegate()->OnStationReceived(this, &m_rtcm.sta);
			break;
		}

		m_rtcm.len = 0;

#endif

	}

private:
	DECLARE_M_RTCM;
};

class cUbloxParser : public cGpsParser
{
private:
	uint64_t checksumFailures;

public:
	cUbloxParser(iGpsParserDelegate* delegate) : cGpsParser(delegate)
	{
		Reset();
		INIT_M_RAW(STRFMT_UBX);
		checksumFailures = 0;
	}

	DECLARE_RAW_DESTRUCTOR(cUbloxParser);

	void WriteByte(uint8_t b) OVERRIDE
	{
		ALLOCATE_M_RAW;

		switch (m_raw.len)
		{
		case 0:
			if (b == UBLOX_PREAMBLE1)
			{
				m_raw.buff[m_raw.len++] = b;
			}
			return;
		case 1:
			if (b != UBLOX_PREAMBLE2)
			{
				// corrupt data
				Reset();
				return;
			}
			m_raw.buff[m_raw.len++] = b;
			return;
		case 2:
			m_classId = b;
			m_raw.buff[m_raw.len++] = b;
			return;
		case 3:
			m_messageId = b;
		case 4: // payload 1
		case 5: // payload 2
			m_raw.buff[m_raw.len++] = b;
			return;
		case UBLOX_HEADER_SIZE:
			// add 2 bytes for the checksum
			m_raw.nbyte = BE_SWAP16(*(uint16_t*)(m_raw.buff + 4)) + UBLOX_CHECKSUM_SIZE;

			// if packet size is larger than buffer minus header and checksum
			if (m_raw.nbyte < UBLOX_CHECKSUM_SIZE || m_raw.nbyte > (MAXRAWLEN - UBLOX_HEADER_SIZE))
			{
				// corrupt data
				Reset();
				return;
			}
			break;
		}
		m_raw.buff[m_raw.len++] = b;
		if (--m_raw.nbyte == 0)
		{
			// get actual checksum
			uint8_t checksum1 = m_raw.buff[m_raw.len - 2];
			uint8_t checksum2 = m_raw.buff[m_raw.len - 1];
			uint8_t actualChecksum1, actualChecksum2;
			// checksum does not include the two preamble bytes
			CalculateChecksum(m_raw.buff, m_raw.len, actualChecksum1, actualChecksum2);
			if (checksum1 == actualChecksum1 && checksum2 == actualChecksum2)
			{
				if (GetDelegate()->OnPacketReceived(this, m_raw.buff, m_raw.len))
				{
					ParseMessage(m_raw.buff, m_raw.len);
				}
			}
			else
			{
				// corrupt data
				checksumFailures++;
			}
			Reset();
		}
	}

	void ParseMessage(const uint8_t* data, int dataLength) OVERRIDE
	{
        (void)data;
        (void)dataLength;

#if defined(RTK_EMBEDDED)

		if (m_raw.buff != data)
		{
			if (m_raw.ownsBuff)
			{
				RTK_FREE(m_raw.buff);
			}
			m_raw.ownsBuff = 0;
			m_raw.buff = (unsigned char*)data;
		}
		m_raw.len = dataLength;

		switch (decode_ubx(&m_raw, 0))
		{
		case DATA_TYPE_OBSERVATION:
			for (obsd_t* ptr = m_raw.obs.data, *ptrEnd = m_raw.obs.data + m_raw.obs.n; ptr != ptrEnd; ptr++)
			{
				ptr->rcv = GetReceiverIndex();
			}
			GetDelegate()->OnObservationReceived(this, &m_raw.obs);
			break;

		case DATA_TYPE_EPHEMERIS:
			int prn;
			switch (satsys(m_raw.ephsat, &prn))
			{
			case SYS_GPS:
			case SYS_GAL:
			case SYS_QZS:
			case SYS_CMP:
				GetDelegate()->OnGpsEphemerisReceived(this, &m_raw.eph, prn);
				break;

			case SYS_GLO:
				GetDelegate()->OnGlonassEphemerisReceived(this, &m_raw.geph, prn);
				break;

			}
			break;

		case DATA_TYPE_SBS:
			GetDelegate()->OnSbsReceived(this, &m_raw.sbsmsg);
			break;

		case DATA_TYPE_ANTENNA_POSITION:
			GetDelegate()->OnStationReceived(this, &m_raw.sta);
			break;

		case DATA_TYPE_ION_UTC_ALMANAC:
			GetDelegate()->OnIonosphereModelUtcAlmanacReceived(this, &m_raw.ionUtcAlm);
			break;
		}

		Reset();

#endif

	}

private:
	void Reset()
	{
		m_raw.len = m_raw.nbyte = 0;
		m_messageId = 0;
		m_classId = 0;
	}

	void CalculateChecksum(const uint8_t* data, int dataLength, uint8_t& checksum1, uint8_t& checksum2)
	{
		checksum1 = 0;
		checksum2 = 0;

		// skip the first two preamble bytes and the last two checksum bytes, they are not part of the checksum
		for (const uint8_t* ptr = data + 2, *ptrEnd = data + (dataLength - UBLOX_CHECKSUM_SIZE); ptr != ptrEnd; ptr++)
		{
			checksum1 += *ptr;
			checksum2 += checksum1;
		}
	}

	DECLARE_M_RAW;
	uint8_t m_messageId;
	uint8_t m_classId;
};

cGpsParser* cGpsParser::CreateParser(eGpsParserType type, iGpsParserDelegate* delegate)
{
	cGpsParser* parser = NULLPTR;
	
	switch (type)
	{
	case GpsParserTypeInertialSense:
		parser = new cInertialSenseParser(delegate);
		break;

	case GpsParserTypeUblox:
		parser = new cUbloxParser(delegate);
		break;

	case GpsParserTypeRtcm3:
		parser = new cRtcmParser(delegate);
		break;

	default:
		return parser;
	}
	
	parser->m_receiverIndex = 1;
	return parser;
}

cGpsParser::cGpsParser(iGpsParserDelegate* delegate)
{
	assert(delegate != NULL);

	m_delegate = delegate;
}

#if defined(__GNUC__)
#pragma GCC diagnostic pop
#endif
