/*
MIT LICENSE

Copyright (c) 2014-2023 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include "ISComm.h"

#define MASK_CONCURRENT_PARSE_ERRORS		1		// Do not report parse errors that while other parers where running.
#define MAX_MSG_LENGTH_ISB					PKT_BUF_SIZE
#define MAX_MSG_LENGTH_NMEA					200
#define MAX_MSG_LENGTH_RTCM					1024
#define MAX_MSG_LENGTH_UBX					1024


typedef union 
{
	uint16_t ck;
	struct
	{
		uint8_t a;	// Lower 8 bits
		uint8_t b;	// Upper 8 bits
	};	
} checksum16_u;

const unsigned int g_validBaudRates[IS_BAUDRATE_COUNT] = {
	// Actual on uINS:
	IS_BAUDRATE_18750000,   // 18750000 (uINS ser1 only)
	IS_BAUDRATE_9375000,    // 9375000
	IS_BAUDRATE_3125000,    // 3125000
	IS_BAUDRATE_921600,     // 937734 (default)
	IS_BAUDRATE_460800,     // 468600
	IS_BAUDRATE_230400,     // 232700
	IS_BAUDRATE_115200,
	IS_BAUDRATE_57600,
	IS_BAUDRATE_38400,
	IS_BAUDRATE_19200,
	IS_BAUDRATE_9600 
};

/**
* Calculate 24 bit crc used in formats like RTCM3 - note that no bounds checking is done on buffer
* @param buffer the buffer to calculate the CRC for
* @param len the number of bytes to calculate the CRC for
* @return the CRC value
*/
unsigned int calculate24BitCRCQ(unsigned char* buffer, unsigned int len)
{
	static const unsigned int TABLE_CRC24Q[] =
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

	unsigned int crc = 0;
	for (uint32_t i = 0; i != len; i++)
	{
		crc = ((crc << 8) & 0xFFFFFF) ^ TABLE_CRC24Q[(crc >> 16) ^ buffer[i]];
	}
	return crc;
}

uint16_t is_comm_fletcher16(uint16_t cksum_init, const void* data, uint32_t size)
{
	checksum16_u cksum;
	cksum.ck = cksum_init;
	for (uint32_t i=0; i<size; i++)
	{
		cksum.a += ((uint8_t*)data)[i];
		cksum.b += cksum.a;
	}	
	return cksum.ck;
}

uint16_t is_comm_xor16(uint16_t cksum_init, const void* data, uint32_t size)
{	
	checksum16_u cksum;
	cksum.ck = cksum_init;
	for (uint32_t i=0; i<size; i++)
	{
		cksum.a ^= ((uint8_t*)data)[i];
		cksum.b ^= cksum.a;
	}
	return cksum.ck;
}

/**
* Retrieve the 32 bit unsigned integer value of the specified bits - note that no bounds checking is done on buffer
* @param buffer the buffer containing the bits
* @param pos the start bit position in buffer to read at
* @param len the number of bits to read
* @return the 32 bit unsigned integer value
*/
unsigned int getBitsAsUInt32(const unsigned char* buffer, unsigned int pos, unsigned int len)
{
	unsigned int bits = 0;
	for (unsigned int i = pos; i < pos + len; i++)
	{
		bits = (bits << 1) + ((buffer[i / 8] >> (7 - i % 8)) & 1u);
	}
	return bits;
}

int validateBaudRate(unsigned int baudRate)
{
	// Valid baudrates for InertialSense hardware
	for (size_t i = 0; i < _ARRAY_ELEMENT_COUNT(g_validBaudRates); i++)
	{
		if (g_validBaudRates[i] == baudRate)
		{
			return 0;
		}
	}
	return -1;
}

static int dataIdShouldSwap(uint32_t dataId)
{
	switch (dataId)
	{
	case DID_GPS1_VERSION:
	case DID_GPS2_VERSION:
		return 0;
	}
	return 1;
}

void is_comm_init(is_comm_instance_t* c, uint8_t *buffer, int bufferSize, uint32_t *timeMsPtr)
{
	// Clear buffer and initialize buffer pointers
	memset(buffer, 0, bufferSize);
	c->rxBuf.size = bufferSize;
	c->rxBuf.start = buffer;
	c->rxBuf.end = buffer + bufferSize;
	c->rxBuf.tail = c->rxBuf.scan = buffer;
	c->timeMs = timeMsPtr;
	
	// Set parse enable flags
	c->config.enabledMask = 
		ENABLE_PROTOCOL_ISB |
		ENABLE_PROTOCOL_NMEA |
		ENABLE_PROTOCOL_UBLOX |
		ENABLE_PROTOCOL_RTCM3 |
		ENABLE_PROTOCOL_SPARTN |
		ENABLE_PROTOCOL_SONY;
	
	c->txPktCount = 0;
	c->rxPktCount = 0;
	c->rxErrorCount = 0;
	c->hasStartByte = 0;
    c->ackNeeded = 0;
	memset(&c->rxPkt, 0, sizeof(packet_t));
	c->rxPkt.data.ptr = c->rxBuf.start;
	c->altDecodeBuf = NULL;
}

static inline void reset_parser(is_comm_instance_t *c)
{
	c->hasStartByte = 0;
}

static inline void resetParserState(is_comm_instance_t *c, protocol_type_t ptype)
{
	if (ptype != _PTYPE_PARSE_ERROR)
	{
		c->rxBuf.scan++;		// increment scan pointer now because it won't happen in for loop 
		c->rxBuf.head = c->rxBuf.scan;
		c->isb.state = 0;
		c->nmea.state = 0;
		c->rtcm.state = 0;
		c->sony.state = 0;
		c->sprt.state = 0;
		c->ubx.state = 0;
	}
}

static inline int32_t parserRunning(is_comm_instance_t *c)
{
	return 
		c->isb.state || 
		c->nmea.state || 
		c->rtcm.state ||
		c->sony.state ||
		c->sprt.state ||
		c->ubx.state;
}

static protocol_type_t processIsbPkt(is_comm_instance_t* c)
{
	is_comm_parser_t* p = &(c->isb);
	int numBytes;
	
	switch (p->state)
	{
	case 0:
		if (*(c->rxBuf.scan) == PSC_ISB_PREAMBLE_BYTE1)
		{
			p->state++;
		}
		return _PTYPE_NONE;

	case 1:
		if (*(c->rxBuf.scan) == PSC_ISB_PREAMBLE_BYTE2)
		{	// Found complete preamble 
			p->head = c->rxBuf.scan-1;
			p->state++;
		}
		else
		{	// Invalid preamble - Reset state
			p->state = 0;
		}
		return _PTYPE_NONE;

	case 2:		// Wait for packet header
		numBytes = (int)(c->rxBuf.scan - p->head);
		if (numBytes < (sizeof(packet_hdr_t)-1))
		{	
			return _PTYPE_NONE;
		}

		// Parse header 
		packet_buf_t *isbPkt = (packet_buf_t*)(p->head);
		packet_hdr_t *hdr = &(c->rxPkt.hdr);
		// hdr->payloadSize = isbPkt->hdr.payloadSize;
		p->size = sizeof(packet_hdr_t) + isbPkt->hdr.payloadSize + 2;		// Header + payload + footer (checksum)
		p->state++;
		return _PTYPE_NONE;

	case 3:		// Wait for entire packet 
		numBytes = (int)(c->rxBuf.scan - p->head) + 1;
		if (numBytes < p->size)
		{
			return _PTYPE_NONE;
		}
		// Found packet end
		break;
	}

	// Reset state
	p->state = 0;

	// Validate checksum
	packet_buf_t *isbPkt = (packet_buf_t*)(p->head);
	uint16_t payloadSize = isbPkt->hdr.payloadSize;
	uint8_t *payload = p->head + sizeof(packet_hdr_t);
	checksum16_u *cksum = (checksum16_u*)(payload + payloadSize);
	int bytes_cksum = p->size - 2;
	uint16_t calcCksum = is_comm_isb_checksum16(0, p->head, bytes_cksum);
	if (cksum->ck != calcCksum)
	{	// Invalid checksum

#if MASK_CONCURRENT_PARSE_ERRORS
		if (parserRunning(c))
		{	// Concurrent parsing happening
			return _PTYPE_NONE;
		}
#endif
		c->rxErrorCount++;
		return _PTYPE_PARSE_ERROR;
	}

	/////////////////////////////////////////////////////////
	// Valid packet found - Checksum passed - Populate rxPkt

	// Increment valid Rx packet count
	c->rxPktCount++;

	packet_t *pkt = &(c->rxPkt);

	// Header
	pkt->hdr.preamble      = isbPkt->hdr.preamble;
	pkt->hdr.flags         = isbPkt->hdr.flags;
	pkt->hdr.id            = isbPkt->hdr.id;
	pkt->hdr.payloadSize   = payloadSize;

	// Payload
	if (pkt->hdr.flags & ISB_FLAGS_PAYLOAD_W_OFFSET)
	{	// Offset is first two bytes in payload  
		pkt->data.size = _MAX(payloadSize-2, 0);
		pkt->data.ptr  = (pkt->data.size ? payload+2 : NULL);
		pkt->offset    = *((uint16_t*)payload);
		// Data starts after offset if data size is non-zero
	}
	else
	{	// No offset
		pkt->data.size = payloadSize;
		pkt->data.ptr  = (payloadSize ? payload : NULL);
		pkt->offset    = 0;
	}

	// Footer
	pkt->checksum = cksum->ck;

	c->ackNeeded = 0;

	uint8_t ptype = pkt->hdr.flags & PKT_TYPE_MASK;
	switch (ptype)
	{
	case PKT_TYPE_SET_DATA:
	case PKT_TYPE_DATA: 
		// ensure offset and size are in bounds - check the size independent of offset because the size could be a
		//  negative number in case of corrupt data
		if (pkt->hdr.id > DID_NULL &&
// 				pkt->hdr.id < DID_COUNT &&		// Commented out to allow support for Luna EVB data sets
			pkt->data.size <= MAX_DATASET_SIZE //&&
// 					pkt->offset <= MAX_DATASET_SIZE &&
// 					pkt->offset + pkt->payload.size <= MAX_DATASET_SIZE
			)
		{
			if (ptype==PKT_TYPE_SET_DATA)
			{	// acknowledge valid data received
				c->ackNeeded = PKT_TYPE_ACK;
			}
				
			return _PTYPE_INERTIAL_SENSE_DATA;
		}
		else
		{	// negative acknowledge data received
			c->ackNeeded = PKT_TYPE_NACK;
		}
		break;
			
	case PKT_TYPE_GET_DATA:
		{
			p_data_get_t *get = (p_data_get_t*)&(isbPkt->payload.data);
			if (get->id > DID_NULL &&
				get->size <= MAX_DATASET_SIZE )
			{
				// Update data pointer
				return _PTYPE_INERTIAL_SENSE_CMD;
			}
		}
		break;

	case PKT_TYPE_STOP_BROADCASTS_ALL_PORTS:
	case PKT_TYPE_STOP_DID_BROADCAST:
	case PKT_TYPE_STOP_BROADCASTS_CURRENT_PORT:
		return _PTYPE_INERTIAL_SENSE_CMD;
	case PKT_TYPE_ACK:
	case PKT_TYPE_NACK:
		return _PTYPE_INERTIAL_SENSE_ACK;
	}                    

	// Invalid data or checksum failure.
	c->rxErrorCount++;
	return _PTYPE_PARSE_ERROR;
}

static protocol_type_t processNmeaPkt(is_comm_instance_t* c)
{
	is_comm_parser_t* p = &(c->nmea);
	int numBytes;

	switch (p->state)
	{
	case 0:	// Find start
		if (*(c->rxBuf.scan) == PSC_NMEA_START_BYTE)
		{	// Found
			p->head = c->rxBuf.scan;
			p->state++;
		}
		return _PTYPE_NONE;

	case 1:	// Find byte before end
		if (*(c->rxBuf.scan) == PSC_NMEA_PRE_END_BYTE)
		{ 	// Found
			p->state++;
		}
		  
		numBytes = c->rxBuf.scan - p->head;
		if (numBytes > MAX_MSG_LENGTH_NMEA)
		{	// Exceeds max length - Reset state
			p->state = 0;
		}
		return _PTYPE_NONE;

	case 2:	// Find end
		if (*(c->rxBuf.scan) != PSC_NMEA_END_BYTE)
		{	// Invalid end - Reset state
			p->state = 0;
			return _PTYPE_NONE;
		}
		// Found packet end
		break;
	}

	// Reset state
	p->state = 0;

	// Validate length
	numBytes = c->rxBuf.scan - p->head + 1;
	if (numBytes < 8)
	{	// Packet length too short
		return _PTYPE_NONE;
	}

	// Validate checksum
	uint8_t tmp = *(c->rxBuf.scan-1);	// Backup value
	*(c->rxBuf.scan-1) = 0;				// Null terminate hex string for strtol()
	int msgChecksum = (int)strtol((const char*)c->rxBuf.scan-3, NULL, 16);
	*(c->rxBuf.scan-1) = tmp;			// Restore value
	int calChecksum = 0;
	for (uint8_t* ptr = p->head + 1, *ptrEnd = c->rxBuf.scan - 4; ptr < ptrEnd; ptr++)
	{
		calChecksum ^= (int)*ptr;
	}
	if (msgChecksum != calChecksum)
	{	// Invalid checksum

#if MASK_CONCURRENT_PARSE_ERRORS
		if (parserRunning(c))
		{	// Concurrent parsing happening
			return _PTYPE_NONE;
		}
#endif
		c->rxErrorCount++;
		return _PTYPE_PARSE_ERROR;
	}

	/////////////////////////////////////////////////////////
	// Valid packet found - Checksum passed - Populate rxPkt

	// Update data pointer and info
	c->rxPkt.data.ptr  = p->head;
	c->rxPkt.data.size = c->rxPkt.size = (uint32_t)numBytes;
	c->rxPkt.hdr.id    = 0;
	c->rxPkt.offset    = 0;

	// Increment valid Rx packet count
	c->rxPktCount++;
	return _PTYPE_NMEA;
}

enum
{
	UBX_PARSE_STATE_PREAMBLE    = 1,
	UBX_PARSE_STATE_CLASS_ID    = 2,
	UBX_PARSE_STATE_MSG_ID      = 3,
	UBX_PARSE_STATE_LENGTH_1    = 4,
	UBX_PARSE_STATE_LENGTH_2    = 5,
};

static protocol_type_t processUbloxPkt(is_comm_instance_t* c)
{
	is_comm_parser_t* p = &(c->ubx);
	int numBytes;

	switch (p->state)
	{
	case 0:
		if (*(c->rxBuf.scan) == UBLOX_START_BYTE1)
		{
			p->state++;
		}
		return _PTYPE_NONE;

	case 1:
		if (*(c->rxBuf.scan) == UBLOX_START_BYTE2)
		{	// Found complete preamble 
			p->head = c->rxBuf.scan-1;
			p->state++;
		}
		else
		{	// Invalid preamble - Reset state
			p->state = 0;
		}
		return _PTYPE_NONE;

	case 2:		// Wait for packet header
		numBytes = (int)(c->rxBuf.scan - p->head);
		if (numBytes < (sizeof(packet_hdr_t)-1))
		{	
			return _PTYPE_NONE;
		}

		// Parse header 
		ubx_pkt_hdr_t *hdr = (ubx_pkt_hdr_t*)(p->head);
		p->size = sizeof(ubx_pkt_hdr_t) + hdr->payloadSize + 2;		// Header + payload + footer (checksum)
		p->state++;
		return _PTYPE_NONE;

	case 3:		// Wait for entire packet 
		numBytes = (int)(c->rxBuf.scan - p->head) + 1;
		if (numBytes < p->size)
		{
			return _PTYPE_NONE;
		}
		// Found packet end
		break;
	}

	// Reset state
	p->state = 0;

	// Validate checksum
	uint16_t pktChecksum = *((uint16_t*)(c->rxBuf.scan - 1));
	uint8_t* cksum_start = p->head + 2;
	uint8_t* cksum_end   = c->rxBuf.scan - 1;
	uint32_t cksum_size  = cksum_end - cksum_start;
	checksum16_u cksum;
	cksum.ck = is_comm_fletcher16(0, cksum_start, cksum_size);
	if (pktChecksum != cksum.ck)
	{	// Invalid checksum

#if MASK_CONCURRENT_PARSE_ERRORS
		if (parserRunning(c))
		{	// Concurrent parsing happening
			return _PTYPE_NONE;
		}
#endif
		c->rxErrorCount++;
		return _PTYPE_PARSE_ERROR;
	}

	/////////////////////////////////////////////////////////
	// Valid packet found - Checksum passed - Populate rxPkt

	// Update data pointer and info
	c->rxPkt.data.ptr  = p->head;
	c->rxPkt.data.size = p->size = numBytes;
	c->rxPkt.hdr.id    = 0;
	c->rxPkt.offset    = 0;

	// Increment valid Rx packet count
	c->rxPktCount++;
	return _PTYPE_UBLOX;
}

static protocol_type_t processRtcm3Pkt(is_comm_instance_t* c)
{
	is_comm_parser_t* p = &(c->rtcm);
	int numBytes;

	switch (p->state)
	{
	case 0:		// Find start
		if (*(c->rxBuf.scan) == RTCM3_START_BYTE)
		{	// Found
			p->head = c->rxBuf.scan;
			p->state++;
		}
		return _PTYPE_NONE;

	case 1:		// Wait for packet header
		p->state++;
		return _PTYPE_NONE;

	case 2:
		p->size = (int)getBitsAsUInt32(p->head, 14, 10) + 6;		// Header + payload + footer (checksum)

		// Validate packet length
		if (p->size > MAX_MSG_LENGTH_RTCM || p->size > c->rxBuf.size - 6)
		{	// Corrupt data - Reset parser
			p->state = 0;
			return _PTYPE_NONE;
		}

		p->state++;
		return _PTYPE_NONE;

	case 3:		// Wait for entire packet 
		numBytes = (int)(c->rxBuf.scan - p->head) + 1;
		if (numBytes < p->size)
		{
			return _PTYPE_NONE;
		}
		// Found packet end
		break;
	}

	// Reset state
	p->state = 0;

	// Validate checksum - len without 3 crc bytes
	int lenWithoutCrc = (int)(p->size - 3);
	uint32_t actualCRC = calculate24BitCRCQ(p->head, lenWithoutCrc);
	uint32_t correctCRC = getBitsAsUInt32(p->head + lenWithoutCrc, 0, 24);

	if (actualCRC != correctCRC)
	{	// Invalid checksum

#if MASK_CONCURRENT_PARSE_ERRORS
		if (parserRunning(c))
		{	// Concurrent parsing happening
			return _PTYPE_NONE;
		}
#endif
		c->rxErrorCount++;
		return _PTYPE_PARSE_ERROR;
	}

	/////////////////////////////////////////////////////////
	// Valid packet found - Checksum passed - Populate rxPkt

	// Update data pointer and info
	c->rxPkt.data.ptr  = p->head;
	c->rxPkt.data.size = p->size = numBytes;
	c->rxPkt.hdr.id    = 0;
	c->rxPkt.offset    = 0;

	// Increment valid Rx packet count
	c->rxPktCount++;
	return _PTYPE_RTCM3;
}

static const uint8_t u8CRC_4_TABLE[] = {
    0x00U, 0x0BU, 0x05U, 0x0EU, 0x0AU, 0x01U, 0x0FU, 0x04U,
    0x07U, 0x0CU, 0x02U, 0x09U, 0x0DU, 0x06U, 0x08U, 0x03U,
    0x0EU, 0x05U, 0x0BU, 0x00U, 0x04U, 0x0FU, 0x01U, 0x0AU,
    0x09U, 0x02U, 0x0CU, 0x07U, 0x03U, 0x08U, 0x06U, 0x0DU,
    0x0FU, 0x04U, 0x0AU, 0x01U, 0x05U, 0x0EU, 0x00U, 0x0BU,
    0x08U, 0x03U, 0x0DU, 0x06U, 0x02U, 0x09U, 0x07U, 0x0CU,
    0x01U, 0x0AU, 0x04U, 0x0FU, 0x0BU, 0x00U, 0x0EU, 0x05U,
    0x06U, 0x0DU, 0x03U, 0x08U, 0x0CU, 0x07U, 0x09U, 0x02U,
    0x0DU, 0x06U, 0x08U, 0x03U, 0x07U, 0x0CU, 0x02U, 0x09U,
    0x0AU, 0x01U, 0x0FU, 0x04U, 0x00U, 0x0BU, 0x05U, 0x0EU,
    0x03U, 0x08U, 0x06U, 0x0DU, 0x09U, 0x02U, 0x0CU, 0x07U,
    0x04U, 0x0FU, 0x01U, 0x0AU, 0x0EU, 0x05U, 0x0BU, 0x00U,
    0x02U, 0x09U, 0x07U, 0x0CU, 0x08U, 0x03U, 0x0DU, 0x06U,
    0x05U, 0x0EU, 0x00U, 0x0BU, 0x0FU, 0x04U, 0x0AU, 0x01U,
    0x0CU, 0x07U, 0x09U, 0x02U, 0x06U, 0x0DU, 0x03U, 0x08U,
    0x0BU, 0x00U, 0x0EU, 0x05U, 0x01U, 0x0AU, 0x04U, 0x0FU,
    0x09U, 0x02U, 0x0CU, 0x07U, 0x03U, 0x08U, 0x06U, 0x0DU,
    0x0EU, 0x05U, 0x0BU, 0x00U, 0x04U, 0x0FU, 0x01U, 0x0AU,
    0x07U, 0x0CU, 0x02U, 0x09U, 0x0DU, 0x06U, 0x08U, 0x03U,
    0x00U, 0x0BU, 0x05U, 0x0EU, 0x0AU, 0x01U, 0x0FU, 0x04U,
    0x06U, 0x0DU, 0x03U, 0x08U, 0x0CU, 0x07U, 0x09U, 0x02U,
    0x01U, 0x0AU, 0x04U, 0x0FU, 0x0BU, 0x00U, 0x0EU, 0x05U,
    0x08U, 0x03U, 0x0DU, 0x06U, 0x02U, 0x09U, 0x07U, 0x0CU,
    0x0FU, 0x04U, 0x0AU, 0x01U, 0x05U, 0x0EU, 0x00U, 0x0BU,
    0x04U, 0x0FU, 0x01U, 0x0AU, 0x0EU, 0x05U, 0x0BU, 0x00U,
    0x03U, 0x08U, 0x06U, 0x0DU, 0x09U, 0x02U, 0x0CU, 0x07U,
    0x0AU, 0x01U, 0x0FU, 0x04U, 0x00U, 0x0BU, 0x05U, 0x0EU,
    0x0DU, 0x06U, 0x08U, 0x03U, 0x07U, 0x0CU, 0x02U, 0x09U,
    0x0BU, 0x00U, 0x0EU, 0x05U, 0x01U, 0x0AU, 0x04U, 0x0FU,
    0x0CU, 0x07U, 0x09U, 0x02U, 0x06U, 0x0DU, 0x03U, 0x08U,
    0x05U, 0x0EU, 0x00U, 0x0BU, 0x0FU, 0x04U, 0x0AU, 0x01U,
    0x02U, 0x09U, 0x07U, 0x0CU, 0x08U, 0x03U, 0x0DU, 0x06U
};

static uint8_t computeCrc4Ccitt(const uint8_t *buf, const uint32_t numBytes)
{
    // Initialize local variables
    uint8_t tableRemainder;
    uint8_t remainder = 0U; // Initial remainder

    // Compute the CRC value
    // Divide each byte of the message by the corresponding polynomial
    for (uint32_t ctr = 0U; ctr < numBytes; ctr++)
    {
        tableRemainder = buf[ctr] ^ remainder;
        remainder = u8CRC_4_TABLE[tableRemainder];
    }

    return remainder & 0x0FU;
}

static protocol_type_t processSonyByte(is_comm_instance_t* c)
{
	is_comm_parser_t* p = &(c->sony);
	int numBytes;

	switch (p->state)
	{
	case 0:
	case 1:
	case 2:
	case 3:
		p->state++;
		break;

	case 4:
	{
        uint16_t msgLength = p->head[1] | (p->head[2] << 8);

    	uint8_t checksum = 0x00;
		for (size_t i = 0; i < 4; i++)
		{
			checksum += p->head[i];
		}

		if(msgLength > 4090 || msgLength > c->rxBuf.size || checksum != p->head[4])
		{
			// corrupt data
			c->rxErrorCount++;
			reset_parser(c);
			return _PTYPE_PARSE_ERROR;
		}

		// parse the message plus 1 check byte
        p->state = -((int32_t)msgLength + 1);
	} break;

	default:
		if (++p->state == 0)
		{
			uint16_t msgLength = p->head[1] | (p->head[2] << 8);

			uint8_t checksum = 0x00;
			for (size_t i = 0; i < msgLength; i++)
			{
				checksum += p->head[i + 5];
			}

			if(checksum != c->rxBuf.scan[-1])
			{
				// corrupt data
				c->rxErrorCount++;
				reset_parser(c);
				return _PTYPE_PARSE_ERROR;
			}
			else
			{	// Checksum passed - Valid packet

				// Update data pointer and info
				c->rxPkt.data.ptr  = p->head;
				c->rxPkt.data.size = p->size = (uint32_t)(c->rxBuf.scan - p->head);
				c->rxPkt.hdr.id    = 0;
				c->rxPkt.offset    = 0;

				// Increment valid Rx packet count
				c->rxPktCount++;
				reset_parser(c);
				return _PTYPE_SONY;
			}
		}
	}

	return _PTYPE_NONE;
}

static protocol_type_t processSpartnByte(is_comm_instance_t* c)
{
	is_comm_parser_t* p = &(c->sprt);
	int numBytes;

	switch (p->state)
	{
	case 0:
	case 1:
	case 2:
	// case 3 is below this to catch bad CRCs before any more is parsed. Can be adapted to filter messages later.
	case 4:
	case 5:
	case 6:
		p->state++;
		break;

	case 3: {
		// Check length and header CRC4
		const uint8_t dbuf[3] = { p->head[1], p->head[2], p->head[3] & 0xF0 };
        uint8_t calc = computeCrc4Ccitt(dbuf, 3);
        if((p->head[3] & 0x0F) != calc)
        {
        	// corrupt data
			c->rxErrorCount++;
			reset_parser(c);
			return _PTYPE_PARSE_ERROR;
        }

        p->state++;
	} break;

	case 7:			// byte 7 (8th byte) is minimum header, but depending on what bits are set...
	case 8:
	case 9:
	case 10:
	case 11: {		// we may need to parse up to byte 11 (12th byte) to get the timestamp and encryption length
		uint16_t payloadLen = ((((uint16_t)(p->head[1]) & 0x01) << 9) |
						(((uint16_t)(p->head[2])) << 1) |
						((p->head[3] & 0x80) >> 7)) & 0x3FF;

		// Variable length CRC {0x0, 0x1, 0x2, 0x3} = {1, 2, 3, 4}bytes - appears at end of message
		payloadLen += (((p->head[3] >> 4) & 0x03) + 1);

		uint8_t extendedTs = p->head[4] & 0x08;
		uint8_t encrypt = p->head[3] & 0x40;
		uint8_t *encryptPtr = NULL;

		if(extendedTs)
		{
			// Timestamp is 32 bit

			if(!encrypt && p->state == 9)
			{
				// Encryption is disabled, we are ready to go to payload bytes
				p->state = -((int32_t)payloadLen);
				break;
			}
			else if(encrypt && p->state == 11)
			{
				// Encryption is ENABLED, and we have all the bytes we need to compute the length of payload
				encryptPtr = &p->head[10];
				// Don't break yet; continue to calculate encryption
			}
			else
			{
				// Not ready yet
				p->state++;
				break;
			}
		}
		else
		{
			// Timestamp is 16 bit

			if(!encrypt && p->state == 7)
			{
				// Encryption is disabled, we are ready to go to payload bytes
				p->state = -((int32_t)payloadLen);
				break;
			}
			else if(encrypt && p->state == 9)
			{
				// Encryption is ENABLED, and we have all the bytes we need to compute the length of payload
				encryptPtr = &p->head[8];
				// Don't break yet; continue to calculate encryption
			}
			else
			{
				// Not ready yet
				p->state++;
				break;
			}
		}

		// Add encryption authentication bytes
		if(encryptPtr)
		{
			// If the message contains an embedded authentication sequence, add the length
			if(((encryptPtr[1] >> 3) & 0x07) > 1)
			{
				switch(encryptPtr[1] & 0x07)
				{
				case 0:
					payloadLen += 8;
					break;
				case 1:
					payloadLen += 12;
					break;
				case 2:
					payloadLen += 16;
					break;
				case 3:
					payloadLen += 32;
					break;
				case 4:
					payloadLen += 64;
					break;
				default:
					break;
				}
			}
		}
		else
		{
			// corrupt data
			c->rxErrorCount++;
			reset_parser(c);
			return _PTYPE_PARSE_ERROR;
		}

		p->state = -((int32_t)payloadLen);

	} break;


	default:
		p->state++;

		if (p->state == 0)
		{	// Valid packet
			c->rxPkt.data.ptr  = p->head;
			c->rxPkt.data.size = c->rxPkt.size = (uint32_t)(c->rxBuf.scan - p->head);
			c->rxPkt.hdr.id    = 0;
			c->rxPkt.offset    = 0;

			// Increment valid Rx packet count
			c->rxPktCount++;
			reset_parser(c);

			return _PTYPE_SPARTN;
		}
		else if(p->state > 0)
		{
			// corrupt data or bad state
			c->rxErrorCount++;
			reset_parser(c);
			return _PTYPE_PARSE_ERROR;
		}

		break;
	}

	return _PTYPE_NONE;
}

int is_comm_free(is_comm_instance_t* c)
{
// 	if (c == 0 || c->buf.start == 0)
// 	{
// 		return -1;
// 	}

	is_comm_buffer_t *buf = &(c->rxBuf);

	int bytesFree = (int)(buf->end - buf->tail);

	// if we are out of free space, we need to either move bytes over or start over
	if (bytesFree == 0)
	{
		if ((int)(buf->head - buf->start) < (int)(buf->size / 3))	// if ring buffer start index is less than this and no space is left, clear the entire ring buffer
		{	// we will be hung unless we flush the ring buffer, we have to drop bytes in this case and the caller
			//  will need to resend the data
			buf->head = buf->start;
			buf->tail = buf->start;
			buf->scan = buf->start;
			resetParserState(c, _PTYPE_NONE);
		}
		else
		{	// shift over the remaining data in the hopes that we will get a valid packet by appending the next read call
			memmove(buf->start, buf->head, buf->tail - buf->head);
			int shift = (int)(buf->head - buf->start);
			buf->head -= shift;
			buf->tail -= shift;
			buf->scan -= shift;
			c->isb.head -= shift;
			c->ubx.head -= shift;
			c->nmea.head -= shift;
			c->rtcm.head -= shift;
			c->sprt.head -= shift;
			c->sony.head -= shift;
		}

		// re-calculate free byte count
		bytesFree = (int)(buf->end - buf->tail);
	}

	return bytesFree;
}

protocol_type_t is_comm_parse_byte(is_comm_instance_t* c, uint8_t byte)
{
	// Reset buffer if needed
	is_comm_free(c);
	
	// Add byte to buffer
	*(c->rxBuf.tail) = byte;
	c->rxBuf.tail++;
	
	return is_comm_parse(c);
}

protocol_type_t is_comm_parse(is_comm_instance_t* c)
{
	is_comm_buffer_t *buf = &(c->rxBuf);
	protocol_type_t ptype;

	// Search for packet
	for (; buf->scan < buf->tail; buf->scan++)
	{
		if (c->config.enabledMask & ENABLE_PROTOCOL_ISB)
		{
			ptype = processIsbPkt(c);
			if (ptype != _PTYPE_NONE) { resetParserState(c, ptype);	return ptype; }
		}

		if (c->config.enabledMask & ENABLE_PROTOCOL_NMEA)
		{
			ptype = processNmeaPkt(c);
			if (ptype != _PTYPE_NONE) {	resetParserState(c, ptype); return ptype; }			
		}

		if (c->config.enabledMask & ENABLE_PROTOCOL_UBLOX)
		{
			ptype = processUbloxPkt(c);
			if (ptype != _PTYPE_NONE) 
			{	
				resetParserState(c, ptype); return ptype; 
			}
		}

		if (c->config.enabledMask & ENABLE_PROTOCOL_RTCM3)
		{
			ptype = processRtcm3Pkt(c);
			if (ptype != _PTYPE_NONE) 
			{	
				resetParserState(c, ptype); return ptype; 
			}
		}

		if (c->config.enabledMask & ENABLE_PROTOCOL_SPARTN)
		{
			ptype = processSpartnByte(c);
			if (ptype != _PTYPE_NONE) {	resetParserState(c, ptype); return ptype; }			
		}

		if (c->config.enabledMask & ENABLE_PROTOCOL_SONY)
		{
			ptype = processSonyByte(c);
			if (ptype != _PTYPE_NONE) {	resetParserState(c, ptype); return ptype; }			
		}

#if 0
		// If we have a start byte, process the data type
		switch (c->hasStartByte)
		{
		case RTCM3_START_BYTE:
			ptype = processRtcm3Pkt(c);
			if (ptype != _PTYPE_NONE)
			{
				return ptype;
			}
			break;
		case SPARTN_START_BYTE:
			ptype = processSpartnByte(c);
			if(ptype == _PTYPE_PARSE_ERROR)
			{
				//time_delay_usec(500);	// Temporary test code
			}
			else if (ptype != _PTYPE_NONE)
			{
				return ptype;
			}
			break;
		case SONY_START_BYTE:
			ptype = processSonyByte(c);
			if (ptype != _PTYPE_NONE)
			{
				return ptype;
			}
			break;
		default:
			break;
		}
#endif

	}

	// No valid data yet...
	return _PTYPE_NONE;
}

int is_comm_get_data_to_buf(uint8_t *buf, uint32_t buf_size, is_comm_instance_t* comm, uint32_t did, uint32_t offset, uint32_t size, uint32_t periodMultiple)
{
	p_data_get_t get;

	get.id = did;
	get.offset = offset;
	get.size = size;
	get.period = periodMultiple;

	return is_comm_write_to_buf(buf, buf_size, comm, PKT_TYPE_GET_DATA, 0, sizeof(p_data_get_t), 0, &get);
}

int is_comm_get_data(pfnIsCommPortWrite portWrite, int port, is_comm_instance_t* comm, uint32_t did, uint32_t offset, uint32_t size, uint32_t periodMultiple)
{
	p_data_get_t get;

	get.id = did;
	get.offset = offset;
	get.size = size;
	get.period = periodMultiple;

	return is_comm_write(portWrite, port, comm, PKT_TYPE_GET_DATA, 0, sizeof(p_data_get_t), 0, &get);
}

void is_comm_encode_hdr(packet_t *pkt, uint8_t flags, uint16_t did, uint16_t data_size, uint16_t offset, void* data)
{
	// Header
	pkt->hdr.preamble = PSC_ISB_PREAMBLE;
	pkt->hdr.flags = flags;
	pkt->hdr.id = did;
	pkt->hdr.payloadSize = data_size;

	// Payload
	pkt->offset = offset;
	if (offset)
	{	// Offset in payload
		pkt->hdr.flags |= ISB_FLAGS_PAYLOAD_W_OFFSET;
		pkt->hdr.payloadSize += 2;
	}
	pkt->data.ptr = data;
	pkt->data.size = data_size;
	pkt->size = pkt->hdr.payloadSize + sizeof(packet_hdr_t) + 2;	// Pkt header + payload + checksum

	// Header checksum
	pkt->hdrCksum = is_comm_isb_checksum16(0, &pkt->hdr, sizeof(pkt->hdr));
	if (offset)
	{
		pkt->hdrCksum = is_comm_isb_checksum16(pkt->hdrCksum, &pkt->offset, sizeof(pkt->offset));
	}
}

int is_comm_write_isb_precomp_to_buffer(uint8_t *buf, uint32_t buf_size, is_comm_instance_t* comm, packet_t *pkt)
{
	if (pkt->size > buf_size)
	{	// Packet doesn't fit in buffer
		return 0;
	}

	// Update checksum using precomputed header checksum and new data
    pkt->checksum = is_comm_isb_checksum16(pkt->hdrCksum, (uint8_t*)pkt->data.ptr, pkt->data.size);

 	// Write packet to buffer
#define MEMCPY_INC(dst, src, size)    memcpy((dst), (src), (size)); (dst) += (size);
	MEMCPY_INC(buf, (uint8_t*)&(pkt->hdr), sizeof(packet_hdr_t));   // Header
	if (pkt->offset)
	{
		MEMCPY_INC(buf, (uint8_t*)&(pkt->offset), 2);               // Offset (optional)
    }
	MEMCPY_INC(buf, (uint8_t*)pkt->data.ptr, pkt->data.size);       // Payload
	MEMCPY_INC(buf, (uint8_t*)&(pkt->checksum), 2);                 // Footer (checksum)

	// Increment Tx count
	comm->txPktCount++;

	return pkt->size;
}

// Returns number of bytes written
int is_comm_write_isb_precomp_to_port(pfnIsCommPortWrite portWrite, int port, is_comm_instance_t* comm, packet_t *pkt)
{
	// Compute checksum using precomputed header checksum
    pkt->checksum = is_comm_isb_checksum16(pkt->hdrCksum, (uint8_t*)pkt->data.ptr, pkt->data.size);

 	// Write packet to port
	int n = portWrite(port, (uint8_t*)&(pkt->hdr), sizeof(packet_hdr_t));  // Header
	if (pkt->offset)
	{
		n += portWrite(port, (uint8_t*)&(pkt->offset), 2);                 // Offset (optional)
    }
    if (pkt->data.size)
    {
        n += portWrite(port, (uint8_t*)pkt->data.ptr, pkt->data.size);     // Payload
    }
	n += portWrite(port, (uint8_t*)&(pkt->checksum), 2);                   // Footer (checksum)

	// Increment Tx count
	comm->txPktCount++;

	return n;
}

int is_comm_write_to_buf(uint8_t* buf, uint32_t buf_size, is_comm_instance_t* comm, uint8_t flags, uint16_t did, uint16_t data_size, uint16_t offset, void* data)
{
	packet_t txPkt;

	// Encode header and header checksum
	is_comm_encode_hdr(&txPkt, flags, did, data_size, offset, data);

	// Update checksum and write packet to buffer
	return is_comm_write_isb_precomp_to_buffer(buf, buf_size, comm, &txPkt);
}

int is_comm_write(pfnIsCommPortWrite portWrite, int port, is_comm_instance_t* comm, uint8_t flags, uint16_t did, uint16_t data_size, uint16_t offset, void* data)
{
	packet_t txPkt;

	// Encode header and header checksum
	is_comm_encode_hdr(&txPkt, flags, did, data_size, offset, data);

	// Update checksum and write packet to port
	return is_comm_write_isb_precomp_to_port(portWrite, port, comm, &txPkt);
}

int is_comm_set_data_to_buf(uint8_t* buf, uint32_t buf_size, is_comm_instance_t* comm, uint16_t did, uint16_t size, uint16_t offset, void* data)
{
    return is_comm_write_to_buf(buf, buf_size, comm, PKT_TYPE_SET_DATA, did, size, offset, data);    
}    

int is_comm_set_data(pfnIsCommPortWrite portWrite, int port, is_comm_instance_t* comm, uint16_t did, uint16_t size, uint16_t offset, void* data)
{
    return is_comm_write(portWrite, port, comm, PKT_TYPE_SET_DATA, did, size, offset, data);    
}    

int is_comm_data_to_buf(uint8_t* buf, uint32_t buf_size, is_comm_instance_t* comm, uint16_t did, uint16_t size, uint16_t offset, void* data)
{
    return is_comm_write_to_buf(buf, buf_size, comm, PKT_TYPE_DATA, did, size, offset, data);    
}    

int is_comm_data(pfnIsCommPortWrite portWrite, int port, is_comm_instance_t* comm, uint16_t did, uint16_t size, uint16_t offset, void* data)
{
    return is_comm_write(portWrite, port, comm, PKT_TYPE_DATA, did, size, offset, data);    
}    

int is_comm_stop_broadcasts_all_ports(pfnIsCommPortWrite portWrite, int port, is_comm_instance_t* comm)
{
    return is_comm_write(portWrite, port, comm, PKT_TYPE_STOP_BROADCASTS_ALL_PORTS, 0, 0, 0, NULL);    
}

int is_comm_stop_broadcasts_current_ports(pfnIsCommPortWrite portWrite, int port, is_comm_instance_t* comm)
{
    return is_comm_write(portWrite, port, comm, PKT_TYPE_STOP_BROADCASTS_CURRENT_PORT, 0, 0, 0, NULL);    
}

char copyStructPToDataP(p_data_t *data, const void *sptr, const unsigned int maxsize)
{
    if ((data->hdr.size + data->hdr.offset) <= maxsize)
    {
        memcpy((uint8_t*)(data->ptr), (uint8_t*)sptr + data->hdr.offset, data->hdr.size);
        return 0;
    }
    else
    {
        return -1;
    }
}

char copyDataPToStructP(void *sptr, const p_data_t *data, const unsigned int maxsize)
{
    if ((data->hdr.size + data->hdr.offset) <= maxsize)
    {
        memcpy((uint8_t*)sptr + data->hdr.offset, data->ptr, data->hdr.size);
        return 0;
    }
    else
    {
        return -1;
    }
}

char copyDataBufPToStructP(void *sptr, const p_data_buf_t *data, const unsigned int maxsize)
{
    if ((data->hdr.size + data->hdr.offset) <= maxsize)
    {
        memcpy((uint8_t*)sptr + data->hdr.offset, data->buf, data->hdr.size);
        return 0;
    }
    else
    {
        return -1;
    }
}

/** Copies packet data into a data structure.  Returns 0 on success, -1 on failure. */
char copyDataPToStructP2(void *sptr, const p_data_hdr_t *dataHdr, const uint8_t *dataBuf, const unsigned int maxsize)
{
    if ((dataHdr->size + dataHdr->offset) <= maxsize)
    {
        memcpy((uint8_t*)sptr + dataHdr->offset, dataBuf, dataHdr->size);
        return 0;
    }
    else
    {
        return -1;
    }
}

/** Copies packet data into a data structure.  Returns 0 on success, -1 on failure. */
char is_comm_copy_to_struct(void *sptr, const is_comm_instance_t *c, const unsigned int maxsize)
{   
	const bufPtr_t *data = &(c->rxPkt.data);
    if ((data->size + c->rxPkt.offset) <= maxsize)
    {
        memcpy((uint8_t*)sptr + c->rxPkt.offset, data->ptr, data->size);
        return 0;
    }
    else
    {
        return -1;
    }
}
