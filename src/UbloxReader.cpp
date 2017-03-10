#include "UbloxReader.h"
#include <assert.h>
#include <algorithm>

#define UBLOX_PREAMBLE1 0xb5
#define UBLOX_PREAMBLE2 0x62
#define UBLOX_HEADER_SIZE 6
#define UBLOX_CHECKSUM_SIZE 2

#if defined(RTK_EMBEDDED)

extern "C"
{
	extern int decode_ubx(raw_t* raw, int doChecksum);
}

#endif

cUbloxReader::cUbloxReader(iUbloxReaderDelegate* delegate)
{
	assert(delegate != NULL);
	m_delegate = delegate;
	Reset();

#if defined(RTK_EMBEDDED)

	m_receiverIndex = RECEIVER_INDEX_ROVER;
	init_raw(&m_raw, STRFMT_UBX);

#endif

}

cUbloxReader::~cUbloxReader()
{

#if defined(RTK_EMBEDDED)

	free_raw(&m_raw);

#endif

}

void cUbloxReader::Write(const uint8_t* data, int dataLength)
{
	for (const uint8_t* ptr = data, *ptrEnd = data + dataLength; ptr != ptrEnd; ptr++)
	{
		WriteByte(*ptr);
	}
}

void cUbloxReader::WriteByte(uint8_t b)
{
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
		if (m_raw.nbyte < UBLOX_CHECKSUM_SIZE || m_raw.nbyte > sizeof(m_raw.buff) - UBLOX_HEADER_SIZE)
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
			if (m_delegate->OnPacketReceived(this, m_raw.buff, m_raw.len))
			{
				ParseMessage();
			}
		}
		else
		{
			// corrupt data
		}
		Reset();
	}
}

void cUbloxReader::Reset()
{
	m_raw.len = m_raw.nbyte = 0;
	m_messageId = 0;
	m_classId = 0;
}

void cUbloxReader::ParseMessage()
{

#if defined(RTK_EMBEDDED)

	switch (decode_ubx(&m_raw, 0))
	{
	case DATA_TYPE_OBSERVATION:
		for (obsd_t* ptr = m_raw.obs.data, *ptrEnd = m_raw.obs.data + m_raw.obs.n; ptr != ptrEnd; ptr++)
		{
			ptr->rcv = GetReceiverIndex();
		}
		m_delegate->OnObservationReceived(this, &m_raw.obs);
		break;

	case DATA_TYPE_EPHEMERIS:
		int prn;
		switch (satsys(m_raw.ephsat, &prn))
		{
		case SYS_GPS:
			m_delegate->OnGpsEphemerisReceived(this, &m_raw.eph, prn);
			break;

		case SYS_GLO:
			m_delegate->OnGlonassEphemerisReceived(this, &m_raw.geph, prn);
			break;
		}
		break;

	case DATA_TYPE_SBS:
		m_delegate->OnSbsReceived(this, &m_raw.sbsmsg);
		break;

	case DATA_TYPE_ANTENNA_POSITION:
		m_delegate->OnStationReceived(this, &m_raw.sta);
		break;
	}

#endif

}

void cUbloxReader::CalculateChecksum(const uint8_t* data, int dataLength, uint8_t& checksum1, uint8_t& checksum2)
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
