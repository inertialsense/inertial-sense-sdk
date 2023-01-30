/*
MIT LICENSE

Copyright (c) 2014-2023 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#ifndef IS_SIMPLE_INTERFACE_H
#define IS_SIMPLE_INTERFACE_H

#include "data_sets.h"
#include "stddef.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Inertial Sense simple communications interface (ISComm)
 *	
 * The simple comm interface does not require any of the com manager APIs and is 
 * designed for simple or lightweight scenarios, tiny embedded platforms, etc.
 *
 * DEFINITIONS
 *	INS		= inertial navigation system
 *	AHRS	= attitude heading reference system
 *	IMU		= inertial measurement unit: gyros (rad/s), accelerometers (m/s^2)
 *	ECEF	= earth-centered earth fixed: x,y,z or vx,vy,vz (m or m/s)
 *	LLA		= latitude, longitude, altitude (degrees,m)
 *	NED		= north, east, down (m or m/s)
 *	QE2B	= quaternion rotation from ECEF frame to local frame.
 *	QN2B	= quaternion rotation from NED frame to local frame.
 *	UVW		= velocities in local frame.
*/

/** Protocol Type */
typedef enum
{
	_PTYPE_PARSE_ERROR = -1,							/** Invalid data or checksum error */
	_PTYPE_NONE = 0,									/** No complete valid data available yet */
	
	_PTYPE_IS_V1_DATA,									/** Protocol Type: Inertial Sense binary data (PID_SET_DATA, PID_DATA) */
	_PTYPE_IS_V1_CMD,									/** Protocol Type: Inertial Sense binary command (PID_GET_DATA, PID_STOP_BROADCASTS...) */
	_PTYPE_IS_V1_ACK,									/** Protocol Type: Inertial Sense binary ACK or NACK  */
	_PTYPE_ASCII_NMEA,									/** Protocol Type: ASCII NMEA */
	_PTYPE_UBLOX,										/** Protocol Type: uBlox binary */
	_PTYPE_RTCM3,										/** Protocol Type: RTCM3 binary */
	_PTYPE_SONYBIN,										/** Protocol Type: Sony GNSS binary */
	_PTYPE_IS_V2_PKT,									/** Protocol Type: Inertial Sense binary V2 packet */
} protocol_type_t;

/** uINS default baud rate */
#define IS_COM_BAUDRATE_DEFAULT IS_BAUDRATE_921600

/** The maximum allowable dataset size */
#define MAX_DATASET_SIZE        1024

/** The decoded overhead involved in sending a packet - 4 bytes for header, 4 bytes for footer */
#define PKT_OVERHEAD_SIZE       8       // = START_BYTE + INFO_BYTE + COUNTER_BYTE + FLAGS_BYTE + CHECKSUM_BYTE_1 + CHECKSUM_BYTE_2 + CHECKSUM_BYTE_3 + END_BYTE

/** The maximum buffer space that is used for sending and receiving packets */
#ifndef PKT_BUF_SIZE
#define PKT_BUF_SIZE            2048
#endif

/** The maximum encoded overhead size in sending a packet (7 bytes for header, 7 bytes for footer). The packet start and end bytes are never encoded. */
#define MAX_PKT_OVERHEAD_SIZE   (PKT_OVERHEAD_SIZE + PKT_OVERHEAD_SIZE - 2)  // worst case for packet encoding header / footer

/** The maximum size of an decoded packet body */
#define MAX_PKT_BODY_SIZE       (((PKT_BUF_SIZE - MAX_PKT_OVERHEAD_SIZE) / 2) & 0xFFFFFFFE) // worst case for packet encoding body, rounded down to even number

/** The maximum size of decoded data in a packet body */
#define MAX_P_DATA_BODY_SIZE    (MAX_PKT_BODY_SIZE-sizeof(p_data_hdr_t))    // Data size limit

/** The maximum size of a decoded ACK message */
#define MAX_P_ACK_BODY_SIZE     (MAX_PKT_BODY_SIZE-sizeof(p_ack_hdr_t))     // Ack data size

/** Binary checksum start value */
#define CHECKSUM_SEED 0x00AAAAAA

/** Defines the 4 parts to the communications version. Major changes involve changes to the com manager. Minor changes involve additions to data structures */

// Major (in com_manager.h)
#define PROTOCOL_VERSION_CHAR0			(2)

// version 1: initial release
// version 2: 24 bit checksum support
// version 3: ioConfig change
#define PROTOCOL_VERSION_CHAR1			(0)

// Minor (in data_sets.h)
// #define PROTOCOL_VERSION_CHAR2		0
// #define PROTOCOL_VERSION_CHAR3		0

#define UBLOX_HEADER_SIZE 6
#define RTCM3_HEADER_SIZE 3

/** No padding - structs must match exactly in memory on all devices */
PUSH_PACK_1

/** Valid baud rates for Inertial Sense hardware */
typedef enum
{
	IS_BAUDRATE_9600        = 9600,
	IS_BAUDRATE_19200       = 19200,
	IS_BAUDRATE_38400       = 38400,
	IS_BAUDRATE_57600       = 57600,
	IS_BAUDRATE_115200      = 115200,		// Actual on uINS:
	IS_BAUDRATE_230400      = 230400,		// 232700
	IS_BAUDRATE_460800      = 460800,		// 468600
	IS_BAUDRATE_921600      = 921600,		// 937734 (default)
	IS_BAUDRATE_3125000     = 3125000,		// 3125000
	IS_BAUDRATE_9375000     = 9375000,		// 9375000
	IS_BAUDRATE_18750000    = 18750000,		// 18750000 (uINS ser1 only)

	// Keep at end and increment/decrement with number of list items
	IS_BAUDRATE_COUNT 		= 11
} baud_rate_t;

/** List of valid baud rates */
extern const uint32_t g_validBaudRates[IS_BAUDRATE_COUNT];

// Packet IDs	
typedef uint32_t ePacketIDs;

#define PID_INVALID                         (ePacketIDs)0   /** Invalid packet id */
#define PID_ACK                             (ePacketIDs)1   /** (ACK) received valid packet */
#define PID_NACK                            (ePacketIDs)2   /** (NACK) received invalid packet */
#define PID_GET_DATA                        (ePacketIDs)3   /** Request for data to be broadcast, response is PID_DATA. See data structures for list of possible broadcast data. */
#define PID_DATA                            (ePacketIDs)4   /** Data sent in response to PID_GET_DATA (no PID_ACK is sent) */
#define PID_SET_DATA                        (ePacketIDs)5   /** Data sent, such as configuration options. PID_ACK is sent in response. */
#define PID_STOP_BROADCASTS_ALL_PORTS       (ePacketIDs)6   /** Stop all data broadcasts on all ports. Responds with an ACK */
#define PID_STOP_DID_BROADCAST              (ePacketIDs)7   /** Stop a specific broadcast */
#define PID_STOP_BROADCASTS_CURRENT_PORT    (ePacketIDs)8   /** Stop all data broadcasts on current port. Responds with an ACK */
#define PID_COUNT                           (ePacketIDs)9   /** The number of packet identifiers, keep this at the end! */
#define PID_MAX_COUNT                       (ePacketIDs)32  /** The maximum count of packet identifiers, 0x1F (PACKET_INFO_ID_MASK) */

/** Represents size number of bytes in memory, up to a maximum of PKT_BUF_SIZE */
typedef struct
{
	/** Number of bytes - for partial data requests, this will be less than the size of the data structure */
	uint32_t            size;

	/** Buffer to hold the bytes */
	uint8_t             buf[PKT_BUF_SIZE];
} buffer_t;

/** Represents size number of bytes in memory, pointing to a BYTE pointer that is owned elsewhere */
typedef struct
{
	/** External bytes owned elsewhere */
	uint8_t             *ptr;

	/** Number of bytes in ptr */
	uint32_t            size;
} bufPtr_t;

/** Represents both a send and receive buffer */
typedef struct
{
	/** send buffer */
	uint8_t             *txPtr;

	/** receive buffer */
	uint8_t             *rxPtr;

	/** size of both buffers */
	uint32_t            size;
} bufTxRxPtr_t;

/** Types of values allowed in ASCII data */
typedef enum
{
	/** 32 bit integer */
	asciiTypeInt = 0,

	/** 32 bit unsigned integer */
	asciiTypeUInt = 1,

	/** 32 bit floating point */
	asciiTypeFloat = 2,

	/** 64 bit floating point */
	asciiTypeDouble = 3
} asciiDataType;

enum ePktStartChars
{
	/** Dollar sign ($), used by ASCII protocol to signify start of message (36) */
	PSC_ASCII_START_BYTE = 0x24,

	/** New line (\n), used by ASCII protocol to signify end of message (10) */
	PSC_ASCII_END_BYTE = 0x0A,

	/** Inertial Sense binary preamble byte */
	PSC_IS_PREAMBLE = 0xEF,

	/** Ublox start byte 1 */
	PSC_UBLOX_START_BYTE1 = 0xB5,

	/** Ublox start byte 2 */
	PSC_UBLOX_START_BYTE2 = 0x62,

	/** RTCM3 start byte */
	PSC_RTCM3_START_BYTE = 0xD3,

	/** SPARTN start byte */
	PSC_SPARTN_START_BYTE = 0x73,

	/** SONY binary Start Byte */
	PSC_SONY_START_BYTE = 0x7F,

	/** Binary packet start byte, used in Inertial Sense protocols prior to release v2.0.0 */
	PSC_START_BYTE_V1 = 0xFF,

	/** Binary packet end byte, used in Inertial Sense protocols prior to release v2.0.0 */
	PSC_END_BYTE_V1 = 0xFE,
};

/** Represents an ASCII message and how it is mapped to a structure in memory */
typedef struct
{
	/** the message, always 4 characters long */
	unsigned char messageId[4];

	/** the ptr of the start of the struct to modify */
	uint8_t* ptr;

	/** the total size of the structure that ptr points to */
	int ptrSize;

	/** field count - the number of items in fieldsAndOffsets */
	int fieldCount;

	/** an array of 1 byte asciiDataType and 1 byte offset (shifted << 8) */
	uint16_t* fieldsAndOffsets;
} asciiMessageMap_t;

typedef struct
{
	/** Preamble, always 0xEF */
	uint8_t preamble;

	/** Reserved for future expansion */
	uint8_t reserved : 4;

	/** Packet ID - determines format and type of data */
	uint8_t packetID : 4;

	/** Packet counter */
	uint16_t packetCtr : 5;

	/** DID this packet represents */
	uint16_t did : 11;

	/** Flags for this packet */
	uint16_t flags : 5;

	/** Size of payload (max 2048) */
	uint16_t payloadSiz : 11;

	/** Checksum for the header, sum all other bytes in this struct */	
	uint8_t checksum;

} isbin_pkt_hdr_t;

static_assert(sizeof(isbin_pkt_hdr_t) == 7U, "Bitfield resulted in incorrect size");

typedef struct
{
	/** Start of available buffer */
	uint8_t* start;

	/** End of available buffer */
	uint8_t* end;

	/** Size of buffer */
	uint32_t size;

	/** Start of data in buffer */
	uint8_t* head;

	/** End of data in buffer */
	uint8_t* tail;

	/** Search pointer in data (head <= scan <= tail) */
	uint8_t* scan;

} is_comm_buffer_t;

/** Bitfield definition for enabling/disabling protocols on specific parser instances */
typedef enum
{
	ENABLE_PROTOCOL_ISB_V1 	= 0x00000001,	// LEGACY V1 protocol
	ENABLE_PROTOCOL_ASCII 	= 0x00000002,
	ENABLE_PROTOCOL_UBLOX 	= 0x00000004,
	ENABLE_PROTOCOL_RTCM3 	= 0x00000008,
	ENABLE_PROTOCOL_SPARTN 	= 0x00000010,
	ENABLE_PROTOCOL_SONY 	= 0x00000020,
	ENABLE_PROTOCOL_ISB_V2 	= 0x00000040,	// Current
} eProtocolMask;

/** An instance of an is_comm interface.  Do not modify these values. */
typedef struct
{
	/** The buffer to use for communications send and receive - this buffer should be large enough to handle the largest data structure you expect * 2 + 32 for worst case packet encoding
		A minimum of 128 is recommended. Set once before calling `is_comm_init` */		
	is_comm_buffer_t buf;
	
	/** Enable/disable protocol parsing (see eProtocolMask) */
	uint32_t enabledMask;

	/** Number of packets written */
	uint32_t txPktCount;

	/** Communications error counter */
	uint32_t rxErrorCount;

	/** Start byte */
	uint8_t hasStartByte;

	/** Current byte we are parsing. If it is positive, it is an offset from the start of the packet. Negative number denotes number of bytes to end of packet. */
	int16_t scanPos;

	/** If > 0, this many bytes will be skipped in parsing before calling the parse function again. Helps reduce parsing overhead. */
	uint16_t needBytes;

	/** Inertial Sense header info */
	isbin_pkt_hdr_t isbinDataHdr;

	/** Pointer to start of packet header */
	uint8_t *pktPtr;

	/** Pointer to start of payload */
	uint8_t *payloadPtr;
	
	/** Alternate buffer location to decode packets.  This buffer must be PKT_BUF_SIZE in size.  NULL value will caused packet decode to occour at head of is_comm_instance_t.buf.  Using an alternate buffer will preserve the original packet (as used in EVB-2 com_bridge).  */
	uint8_t *altDecodeBuf;

} is_comm_instance_t;

/** Pop off the packing argument, we can safely allow packing and shifting in memory at this point */
POP_PACK

/**
 * @brief Init simple communications interface
 * @note Call this before doing anything else
 * 
 * @param instance pointer to ISComm instance
 * @param buffer pointer to start of buffer
 * @param bufferSize number of bytes in buffer
 */
void is_comm_init(is_comm_instance_t *instance, uint8_t *buffer, uint16_t bufferSize);

/**
 * @brief Decode packet data
 * @details When data is available, the comm instance dataPtr will point to the 
 * 	start of the valid data. For Inertial Sense binary protocol, comm instance 
 * 	dataHdr contains the data ID (DID), size, and offset.
 * 
 * @param instance pointer to ISComm instance
 * @param byte value of the byte to parse
 * @return protocol_type_t returns NONE or ERROR unless a packet is finished 
 *	parsing
 */
protocol_type_t is_comm_parse_byte(is_comm_instance_t *instance, uint8_t byte);

/**
 * @brief Decode packet data
 * @details When data is available, the comm instance dataPtr will point to the 
 * 	start of the valid data. For Inertial Sense binary protocol, comm instance 
 * 	dataHdr contains the data ID (DID), size, and offset.
 * 
 * @param instance pointer to ISComm instance
 * @return protocol_type_t returns NONE or ERROR unless a packet is finished 
 *	parsing
 */
protocol_type_t is_comm_parse(is_comm_instance_t *instance);

/**
 * @brief Removes old data and shift unparsed data to the the buffer start
 * 
 * @param instance pointer to ISComm instance
 * @return number of bytes available in the comm buffer 
 */
int is_comm_free(is_comm_instance_t *instance);

/**
 * @brief Encode a Inertial Sense binary packet to get data from the device.
 * 	Puts the data ready to send into the buffer passed into is_comm_init.
 * 	Pass an offset and length of 0 to request the entire data structure.
 * 
 * @param instance pointer to ISComm instance
 * @param dataId the data id to request (see DID_* at top of this file)
 * @param offset the offset into data to request. 0 has special behavior
 * @param length the length into data from offset to request. 0 has special behavior
 * @param periodMultiple sets period of message (base rate * periodMultiple), 0 for a one time message with subsequent shutoff
 * @return the number of bytes written to the instance's buffer, will be less than 1 if error
 */
int is_comm_get_data(is_comm_instance_t *instance, uint16_t dataId, uint16_t offset, uint16_t size, uint16_t periodMultiple);

/**
 * @brief Encode a binary packet to get predefined list of data sets from the 
 * 	Puts the data ready to send into the buffer passed into is_comm_init.
 * 
 * @param instance pointer to ISComm instance
 * @param rmcBits data messages to stream. See presets in data_sets.h (RMC_PRESET_...)
 * @return the number of bytes written to the comm buffer (from is_comm_init), will be less than 1 if error
 */
int is_comm_get_data_rmc(is_comm_instance_t *instance, uint64_t rmcBits);

/**
 * @brief Encode a binary packet to set data on the device. 
 * @note Use is_comm_set_data_ack if you need a ACK from the device, otherwise use is_comm_set_data and the device will not send an ACK
 * 
 * @param instance pointer to ISComm instance
 * @param dataId the data id to set on the device (see DID_* at top of this file)
 * @param offset the offset to start setting data at on the data structure on the device
 * @param size the number of bytes to set on the data structure on the device
 * @param data the actual data to change on the data structure on the device - this should have at least size bytes available
 * @return the number of bytes written to the comm buffer (from is_comm_init), will be less than 1 if error
 */
int is_comm_set_data_ack(is_comm_instance_t *instance, uint32_t dataId, uint32_t offset, uint32_t size, void* data);
int is_comm_set_data(is_comm_instance_t *instance, uint32_t dataId, uint32_t offset, uint32_t size, void* data);

/**
 * @brief Encode a binary packet to stop all messages being broadcast on the device on all ports - puts the data ready to send into the buffer passed into is_comm_init
 * 
 * @param instance pointer to ISComm instance
 * @return 0 if success, otherwise an error code
 */
int is_comm_stop_broadcasts_all_ports(is_comm_instance_t *instance);

/**
 * @brief Encode a binary packet to stop all messages being broadcast on the device on this port - puts the data ready to send into the buffer passed into is_comm_init
 * 
 * @param instance pointer to ISComm instance
 * @return 0 if success, otherwise an error code
 */
int is_comm_stop_broadcasts_current_port(is_comm_instance_t *instance);


// -------------------------------------------------------------------------------------------------------------------------------
// Common packet encode / decode functions
// -------------------------------------------------------------------------------------------------------------------------------
int is_encode_binary_packet(void* srcBuffer, uint16_t srcBufferLength, packet_hdr_t* hdr, uint8_t additionalPktFlags, void* encodedPacket, int encodedPacketLength);
int is_decode_binary_packet(packet_t *pkt, unsigned char* pbuf, int pbufSize);
int is_decode_binary_packet_byte(uint8_t** _ptrSrc, uint8_t** _ptrDest, uint32_t* checksum, uint32_t shift);
void is_decode_binary_packet_footer(packet_ftr_t* ftr, uint8_t* ptrSrc, uint8_t** ptrSrcEnd, uint32_t* checksum);
void is_enable_packet_encoding(int enabled); // default is enabled

/**
 * @brief Calculate 24 bit crc used in formats like RTCM3 - note that no bounds checking is done on buffer
 * 
 * @param buffer the buffer to calculate the CRC for
 * @param len the number of bytes to calculate the CRC for
 * @return the CRC value
 */
uint32_t calculate24BitCRCQ(unsigned char* buffer, uint16_t len);

/**
 * @brief Retrieve the 32 bit unsigned integer value of the specified bits - note that no bounds checking is done on buffer
 * 
 * @param buffer the buffer containing the bits
 * @param pos the start bit position in buffer to read at
 * @param len the number of bits to read
 * @return the 32 bit unsigned integer value
 */
uint32_t getBitsAsUInt32(const uint8_t* buffer, uint16_t pos, uint16_t len);

/** Copies data structure into packet data.  Data copied is limited to the size and offset specified in p_data_t *data.  Returns 0 on success, -1 on failure. */
int8_t copyStructPToDataP(void *data, const void *sptr, const uint16_t srclen, const uint16_t destlen);

/** Copies packet data into a data structure.  Returns 0 on success, -1 on failure. */
int8_t copyDataPToStructP(void *sptr, const void *data, const uint16_t srclen, const uint16_t destlen);

/** Copies packet data into a data structure.  Returns 0 on success, -1 on failure. */
int8_t copyDataPToStructP2(void *sptr, const p_data_hdr_t *dataHdr, const uint8_t *dataBuf, const uint16_t maxsize);

/** Copies is_comm_instance data into a data structure.  Returns 0 on success, -1 on failure. */
int8_t is_comm_copy_to_struct(void *sptr, const is_comm_instance_t *com, const uint16_t maxsize);

/** Returns -1 if the baudrate is not a standard baudrate. */
int validateBaudRate(uint32_t baudRate);

/** create a uint32_t from an ASCII message id that is the same, regardless of CPU architecture */
inline uint32_t ascii_msgID_to_uint32(const char c[4])
{
	return  ((uint32_t)c[0] << 24) |
			((uint32_t)c[1] << 16) |
			((uint32_t)c[2] << 8) |
			((uint32_t)c[3]);
}

#ifdef __cplusplus
}
#endif

#endif // IS_SIMPLE_INTERFACE_H
