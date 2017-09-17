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

#define INIT_M_RTCM init_rtcm(&m_rtcm, 0); SetReceiverIndex(RECEIVER_INDEX_BASE_STATION);
#define ALLOCATE_M_RTCM if (!m_rtcm.ownsBuff) { m_rtcm.buff = (unsigned char*)RTK_MALLOC(MAXRTCMLEN); m_rtcm.ownsBuff = 1; }
#define INIT_M_RAW(fmt) init_raw(&m_raw, fmt, 0); SetReceiverIndex(RECEIVER_INDEX_ROVER);
#define ALLOCATE_M_RAW if (!m_raw.ownsBuff) { m_raw.buff = (unsigned char*)RTK_MALLOC(MAXRAWLEN);	m_raw.ownsBuff = 1; }
#define DECLARE_RTCM_DESTRUCTOR(class_name) ~class_name() OVERRIDE { free_rtcm(&m_rtcm); }
#define DECLARE_RAW_DESTRUCTOR(class_name) ~class_name() OVERRIDE { free_raw(&m_raw); }
#define DECLARE_M_RTCM rtcm_t m_rtcm;
#define DECLARE_M_RAW raw_t m_raw;
#define calculate24BitCRCQ rtk_crc24q
#define getBitsAsUInt32 getbitu

extern "C"
{
	extern int decode_rtcm3(rtcm_t *rtcm);
	extern int decode_ubx(raw_t* raw, int doChecksum);
}

gtime_t g_gps_latest_time;

#else

#define INIT_M_RTCM m_rtcm.len = 0; m_rtcm.nbyte = 0;
#define ALLOCATE_M_RTCM
#define INIT_M_RAW(fmt) m_raw.len = 0; m_raw.nbyte = 0;
#define ALLOCATE_M_RAW
#define DECLARE_RTCM_DESTRUCTOR(c)
#define DECLARE_RAW_DESTRUCTOR(c)
#define DECLARE_M_RTCM struct { uint8_t buff[2048]; int len; int nbyte; } m_rtcm;
#define DECLARE_M_RAW struct { uint8_t buff[2048]; int len; int nbyte; } m_raw;

/*!
* Calculate 24 bit crc used in formats like RTCM3 - note that no bounds checking is done on buffer
* @param buffer the buffer to calculate the CRC for
* @param len the number of bytes to calculate the CRC for
* @return the CRC value
*/
static uint32_t calculate24BitCRCQ(uint8_t* buffer, uint32_t len)
{
	static const uint32_t TABLE_CRC24Q[] =
	{
		0x000000,0x864CFB,0x8AD50D,0x0C99F6,0x93E6E1,0x15AA1A,0x1933EC,0x9F7F17,
		0xA18139,0x27CDC2,0x2B5434,0xAD18CF,0x3267D8,0xB42B23,0xB8B2D5,0x3EFE2E,
		0xC54E89,0x430272,0x4F9B84,0xC9D77F,0x56A868,0xD0E493,0xDC7D65,0x5A319E,
		0x64CFB0,0xE2834B,0xEE1ABD,0x685646,0xF72951,0x7165AA,0x7DFC5C,0xFBB0A7,
		0x0CD1E9,0x8A9D12,0x8604E4,0x00481F,0x9F3708,0x197BF3,0x15E205,0x93AEFE,
		0xAD50D0,0x2B1C2B,0x2785DD,0xA1C926,0x3EB631,0xB8FACA,0xB4633C,0x322FC7,
		0xC99F60,0x4FD39B,0x434A6D,0xC50696,0x5A7981,0xDC357A,0xD0AC8C,0x56E077,
		0x681E59,0xEE52A2,0xE2CB54,0x6487AF,0xFBF8B8,0x7DB443,0x712DB5,0xF7614E,
		0x19A3D2,0x9FEF29,0x9376DF,0x153A24,0x8A4533,0x0C09C8,0x00903E,0x86DCC5,
		0xB822EB,0x3E6E10,0x32F7E6,0xB4BB1D,0x2BC40A,0xAD88F1,0xA11107,0x275DFC,
		0xDCED5B,0x5AA1A0,0x563856,0xD074AD,0x4F0BBA,0xC94741,0xC5DEB7,0x43924C,
		0x7D6C62,0xFB2099,0xF7B96F,0x71F594,0xEE8A83,0x68C678,0x645F8E,0xE21375,
		0x15723B,0x933EC0,0x9FA736,0x19EBCD,0x8694DA,0x00D821,0x0C41D7,0x8A0D2C,
		0xB4F302,0x32BFF9,0x3E260F,0xB86AF4,0x2715E3,0xA15918,0xADC0EE,0x2B8C15,
		0xD03CB2,0x567049,0x5AE9BF,0xDCA544,0x43DA53,0xC596A8,0xC90F5E,0x4F43A5,
		0x71BD8B,0xF7F170,0xFB6886,0x7D247D,0xE25B6A,0x641791,0x688E67,0xEEC29C,
		0x3347A4,0xB50B5F,0xB992A9,0x3FDE52,0xA0A145,0x26EDBE,0x2A7448,0xAC38B3,
		0x92C69D,0x148A66,0x181390,0x9E5F6B,0x01207C,0x876C87,0x8BF571,0x0DB98A,
		0xF6092D,0x7045D6,0x7CDC20,0xFA90DB,0x65EFCC,0xE3A337,0xEF3AC1,0x69763A,
		0x578814,0xD1C4EF,0xDD5D19,0x5B11E2,0xC46EF5,0x42220E,0x4EBBF8,0xC8F703,
		0x3F964D,0xB9DAB6,0xB54340,0x330FBB,0xAC70AC,0x2A3C57,0x26A5A1,0xA0E95A,
		0x9E1774,0x185B8F,0x14C279,0x928E82,0x0DF195,0x8BBD6E,0x872498,0x016863,
		0xFAD8C4,0x7C943F,0x700DC9,0xF64132,0x693E25,0xEF72DE,0xE3EB28,0x65A7D3,
		0x5B59FD,0xDD1506,0xD18CF0,0x57C00B,0xC8BF1C,0x4EF3E7,0x426A11,0xC426EA,
		0x2AE476,0xACA88D,0xA0317B,0x267D80,0xB90297,0x3F4E6C,0x33D79A,0xB59B61,
		0x8B654F,0x0D29B4,0x01B042,0x87FCB9,0x1883AE,0x9ECF55,0x9256A3,0x141A58,
		0xEFAAFF,0x69E604,0x657FF2,0xE33309,0x7C4C1E,0xFA00E5,0xF69913,0x70D5E8,
		0x4E2BC6,0xC8673D,0xC4FECB,0x42B230,0xDDCD27,0x5B81DC,0x57182A,0xD154D1,
		0x26359F,0xA07964,0xACE092,0x2AAC69,0xB5D37E,0x339F85,0x3F0673,0xB94A88,
		0x87B4A6,0x01F85D,0x0D61AB,0x8B2D50,0x145247,0x921EBC,0x9E874A,0x18CBB1,
		0xE37B16,0x6537ED,0x69AE1B,0xEFE2E0,0x709DF7,0xF6D10C,0xFA48FA,0x7C0401,
		0x42FA2F,0xC4B6D4,0xC82F22,0x4E63D9,0xD11CCE,0x575035,0x5BC9C3,0xDD8538
	};

	uint32_t crc = 0;
	for (uint32_t i = 0; i != len; i++)
	{
		crc = ((crc << 8) & 0xFFFFFF) ^ TABLE_CRC24Q[(crc >> 16) ^ buffer[i]];
	}
	return crc;
}

/*!
* Retrieve the 32 bit unsigned integer value of the specified bits - note that no bounds checking is done on buffer
* @param buffer the buffer containing the bits
* @param pos the start bit position in buffer to read at
* @param len the number of bits to read
* @return the 32 bit unsigned integer value
*/
static uint32_t getBitsAsUInt32(const uint8_t* buffer, uint32_t pos, uint32_t len)
{
	uint32_t bits = 0;
	for (uint32_t i = pos; i < pos + len; i++)
	{
		bits = (bits << 1) + ((buffer[i / 8] >> (7 - i % 8)) & 1u);
	}
	return bits;
}

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
				if (m_len > 23)
				{
					GetDelegate()->OnPacketReceived(this, m_buf, m_len);
				}
				m_len = 0;
			}
		}
	}

private:
	uint8_t m_buf[2048];
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
					// message length only includes the message bytes, not the header or 3 CRC bytes at the end
					// plus 2 because we will be appending the first byte of the message or CRC bytes
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
public:
	cUbloxParser(iGpsParserDelegate* delegate) : cGpsParser(delegate)
	{
		Reset();
		INIT_M_RAW(STRFMT_UBX);
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
			if (m_raw.nbyte < UBLOX_CHECKSUM_SIZE || (unsigned int)m_raw.nbyte > sizeof(m_raw.buff) - UBLOX_HEADER_SIZE)
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
	switch (type)
	{
	case GpsParserTypeInertialSense:
		return new cInertialSenseParser(delegate);

	case GpsParserTypeUblox:
		return new cUbloxParser(delegate);

	case GpsParserTypeRtcm3:
		return new cRtcmParser(delegate);

	default:
		return NULL;
	}
}

cGpsParser::cGpsParser(iGpsParserDelegate* delegate)
{
	assert(delegate != NULL);

	m_delegate = delegate;
}

#if defined(__GNUC__)
#pragma GCC diagnostic pop
#endif
