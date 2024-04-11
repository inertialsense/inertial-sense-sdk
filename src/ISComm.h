/*
MIT LICENSE

Copyright (c) 2014-2024 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#ifndef IS_COMM_H
#define IS_COMM_H

#include "data_sets.h"
#include "stddef.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 *	DEFINITIONS AND CONVENTIONS
 *	
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

// -------------------------------------------------------------------------------------------------------------------------------
// Inertial Sense simple communications interface --------------------------------------------------------------------------------
// -------------------------------------------------------------------------------------------------------------------------------
// The simple comm interface does not require any of the com manager APIs and is designed for simple or lightweight scenarios, tiny embedded platforms, etc.
// *****************************************************************************
// ****** Binary messages                                                 ******
// *****************************************************************************

/** INS/AHRS */
#define _DID_INS_LLA_EULER_NED		DID_INS_1				/** (see ins_1_t) INS/AHRS output: euler from NED, LLA (degrees,m), NED pos (m) and vel (m/s) from refLLA */
#define _DID_INS_LLA_QN2B			DID_INS_2				/** (see ins_2_t) INS/AHRS output: quaternion from NED, LLA (degrees,m) */
#define _DID_INS_LLA_QN2B_MSL		DID_INS_3				/** (see ins_3_t) INS/AHRS output: quaternion from NED, LLA (degrees,m), and MSL altitude */
#define _DID_INS_ECEF_QE2B			DID_INS_4				/** (see ins_4_t) INS output: ECEF position (m) and velocity (m/s), quaternion from ECEF */

/** IMU */
#define _DID_IMU					DID_IMU					/** (see imu_t) IMU output: angular rate (rad/s) and linear acceleration (m/s^2) */
#define _DID_PIMU					DID_PIMU				/** (see pimu_t) IMU output: Coning and sculling integrated at IMU update rate. */	

/** GPS */
#define _DID_GPS1_POS				DID_GPS1_POS			/** (see gps_pos_t) GPS output */

/** Magnetometer, Barometer, and other Sensor */
#define _DID_MAG_CAL				DID_MAG_CAL				/** (see mag_cal_t) Magnetometer calibration */
#define _DID_MAGNETOMETER			DID_MAGNETOMETER		/** (see magnetometer_t) Magnetometer sensor output */
#define _DID_BAROMETER				DID_BAROMETER			/** (see barometer_t) Barometric pressure sensor data */
#define _DID_WHEEL_ENCODER			DID_WHEEL_ENCODER		/** (see wheel_encoder_t) Wheel encoder sensor data */
#define _DID_POS_MEASUREMENT		DID_POSITION_MEASUREMENT/** (see pos_measurement_t) Position Measurement data*/

/** Utilities */
#define _DID_DEV_INFO				DID_DEV_INFO			/** (see dev_info_t) Device information */
#define _DID_BIT					DID_BIT					/** (see bit_t) System built-in self-test */
#define _DID_STROBE_IN_TIME			DID_STROBE_IN_TIME		/** (see strobe_in_time_t) Timestamp for input strobe */

/** Configuration */
#define _DID_FLASH_CONFIG			DID_FLASH_CONFIG 		/** (see nvm_flash_cfg_t) Flash memory configuration */
#define _DID_RMC					DID_RMC					/** (see rmc_t) Realtime message controller */

#define ZEPHYR_SUCCESS_CODE     	0

/** Protocol Type */
typedef enum
{
	_PTYPE_NONE                 = 0,						/** No complete valid data available yet */
	_PTYPE_PARSE_ERROR          = 1,						/** Invalid data or checksum error */
	_PTYPE_INERTIAL_SENSE_ACK   = 2,						/** Protocol Type: Inertial Sense binary acknowledge (ack) or negative acknowledge (PID_ACK, PID_NACK)  */
	_PTYPE_INERTIAL_SENSE_CMD   = 3,						/** Protocol Type: Inertial Sense binary command (PID_GET_DATA, PID_STOP_BROADCASTS...) */
	_PTYPE_INERTIAL_SENSE_DATA  = 4,						/** Protocol Type: Inertial Sense binary data (PID_SET_DATA, PID_DATA) */
	_PTYPE_NMEA                 = 5,						/** Protocol Type: NMEA (National Marine Electronics Association) */
	_PTYPE_UBLOX                = 6,						/** Protocol Type: uBlox binary */
	_PTYPE_RTCM3                = 7,						/** Protocol Type: RTCM3 binary (Radio Technical Commission for Maritime Services) */
	_PTYPE_SPARTN               = 8,						/** Protocol Type: SPARTN binary */
	_PTYPE_SONY                 = 9,						/** Protocol Type: Sony binary */
	_PTYPE_FIRST_DATA           = _PTYPE_INERTIAL_SENSE_DATA,
	_PTYPE_LAST_DATA            = _PTYPE_SONY
} protocol_type_t;

/** The maximum allowable dataset size */
#define MAX_DATASET_SIZE        1024

/** The decoded overhead involved in sending a packet - 4 bytes for header, 4 bytes for footer */
#define PKT_OVERHEAD_SIZE       8       // = START_BYTE + INFO_BYTE + COUNTER_BYTE + FLAGS_BYTE + CHECKSUM_BYTE_1 + CHECKSUM_BYTE_2 + CHECKSUM_BYTE_3 + END_BYTE

/** The maximum buffer space that is used for sending and receiving packets */
#ifndef PKT_BUF_SIZE
#define PKT_BUF_SIZE            2048
#endif

/** The maximum time between received data that will reset in the parser */
#define MAX_PARSER_GAP_TIME_MS  100

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
#define PROTOCOL_VERSION_CHAR1			(0)

// Minor (in data_sets.h)
// #define PROTOCOL_VERSION_CHAR2		0
// #define PROTOCOL_VERSION_CHAR3		0

#define UBLOX_HEADER_SIZE 6
#define RTCM3_HEADER_SIZE 3
#define MAX_MSG_LENGTH_NMEA					200

/** Send data to the serial port.  Returns number of bytes written. */ 
typedef int(*pfnIsCommPortWrite)(int port, const uint8_t* buf, int len);

/** We must not allow any packing or shifting as these data structures must match exactly in memory on all devices */
PUSH_PACK_1

/** Valid baud rates for Inertial Sense hardware */
typedef enum
{
    IS_BAUDRATE_9600            = 9600,
    IS_BAUDRATE_19200           = 19200,
    IS_BAUDRATE_38400           = 38400,
    IS_BAUDRATE_57600           = 57600,
    IS_BAUDRATE_115200          = 115200,       //  IMX-5.0,  uINS-3,  Actual baudrates                                             
    IS_BAUDRATE_230400          = 230400,       //   230547,  232700, 
    IS_BAUDRATE_460800          = 460800,       //   462428,  468600, 
    IS_BAUDRATE_921600          = 921600,       //   930233,  937734,
    IS_BAUDRATE_10000000        = 10000000,     // 10000000  ( IMX-5 only)
    IS_BAUDRATE_COUNT           = 9,
	IS_BAUDRATE_DEFAULT         = IS_BAUDRATE_921600,
	IS_BAUDRATE_STANDARD_MIN    = IS_BAUDRATE_9600,
	IS_BAUDRATE_STANDARD_MAX    = IS_BAUDRATE_921600,
	IS_BAUDRATE_MAX             = IS_BAUDRATE_10000000,
} baud_rate_t;

typedef struct
{
    uint32_t 			   baudRate;
    uint8_t 			   parity;
    uint8_t 			   stopBits;
} serial_options_t;

/** List of valid baud rates */
extern const unsigned int g_validBaudRates[IS_BAUDRATE_COUNT];

/*
Packet Overview

Byte
0			Packet start byte
1			Packet indo: ID (mask 0x1F) | reserved bits (mask 0xE)
2			Packet counter (for ACK and retry)
3			Packet flags

// packet body, may or may not exist depending on packet id - packet body is made up of 4 byte or 8 byte values.
4-7			Data identifier
8-11		Data length
12-15		Data offset
16-19		Data start
(n-8)-(n-5)	Last piece of data
// end data

n-4			Reserved
n-3			Checksum high byte
n-2			Checksum low byte
n-1			Packet end byte
*/

// Packet IDs
// typedef uint32_t ePacketIDs;

typedef enum
{
	PKT_TYPE_INVALID                        = 0,    // Invalid packet id
	PKT_TYPE_ACK                            = 1,    // (ACK) received valid packet
	PKT_TYPE_NACK                           = 2,    // (NACK) received invalid packet
	PKT_TYPE_GET_DATA                       = 3,    // Request for data to be broadcast, response is PKT_TYPE_DATA. See data structures for list of possible broadcast data.
	PKT_TYPE_DATA                           = 4,    // Data sent in response to PKT_TYPE_GET_DATA (no PKT_TYPE_ACK is sent)
	PKT_TYPE_SET_DATA                       = 5,    // Data sent, such as configuration options.  PKT_TYPE_ACK is sent in response.
	PKT_TYPE_STOP_BROADCASTS_ALL_PORTS      = 6,    // Stop all data broadcasts on all ports. Responds with an ACK
	PKT_TYPE_STOP_DID_BROADCAST             = 7,    // Stop a specific broadcast
	PKT_TYPE_STOP_BROADCASTS_CURRENT_PORT   = 8,    // Stop all data broadcasts on current port. Responds with an ACK
	PKT_TYPE_COUNT                          = 9,    // The number of packet identifiers, keep this at the end!
	PKT_TYPE_MAX_COUNT                      = 16,   // The maximum count of packet identifiers, 0x1F (PACKET_INFO_ID_MASK)
	PKT_TYPE_MASK                           = 0x0F, // ISB packet type bitmask

	ISB_FLAGS_MASK                          = 0xF0, // ISB packet flags bitmask (4 bits upper nibble)
	ISB_FLAGS_EXTENDED_PAYLOAD              = 0x10, // Payload is larger than 2048 bytes and extends into next packet.
	ISB_FLAGS_PAYLOAD_W_OFFSET              = 0x20, // The first two bytes of the payload are the byte offset of the payload data into the data set.
} eISBPacketFlags;

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

/** Types of values allowed in NMEA data */
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

/**
Built in special bytes that will need to be encoded in the binary packet format. This is not an exhaustive list, as other bytes such as ublox and rtcm preambles
will be encoded as well, but these messages are not parsed and handled in the com manager, rather they are forwarded via the pass through handler.
A byte is encoded by writing a 0xFD byte (encoded byte marker), followed by the encoded byte, which is created by inverting all the bits of the original byte.
These bytes are not encoded when written in the proper spot in the packet (i.e. when writing the first byte for a binary packet, the 0xFF byte, no encoding
is performed).
*/
enum ePktSpecialChars
{
	/** Dollar sign ($), used by NMEA protocol to signify start of message (36) */
	PSC_NMEA_START_BYTE = 0x24,

	/** Carriage return (\r), used by NMEA protocol to signify one byte before end of message (10) */
	PSC_NMEA_PRE_END_BYTE = 0x0D,

	/** New line (\n), used by NMEA protocol to signify end of message (10) */
	PSC_NMEA_END_BYTE = 0x0A,

	/** Inertial Sense Binary packet preamble (start) byte 1 (239) */
	PSC_ISB_PREAMBLE_BYTE1 = 0xEF,

	/** Inertial Sense Binary packet preamble (start) byte 2 (73) */
	PSC_ISB_PREAMBLE_BYTE2 = 0x49,

	/** Inertial Sense Binary packet start preamble (239, 74) */
	PSC_ISB_PREAMBLE = PSC_ISB_PREAMBLE_BYTE2<<8 | PSC_ISB_PREAMBLE_BYTE1,

	/** Ublox start byte 1 (181) */
	UBLOX_START_BYTE1 = 0xB5,

	/** Ublox start byte 2 (98) */
	UBLOX_START_BYTE2 = 0x62,

	/** Rtcm3 start byte (211) */
	RTCM3_START_BYTE = 0xD3,

	/** SPARTN start byte */
	SPARTN_START_BYTE = 0x73,

	/** Sony GNSS start byte */
	SONY_START_BYTE = 0x7F,
};

/** Represents an NMEA message and how it is mapped to a structure in memory */
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

/** Represents the 4 bytes that begin each binary packet */
typedef struct
{
	/** Packet start bytes, always 0x49EF */
	uint16_t            preamble;

	/** Packet identifier (see eISBPacketFlags) */
	uint8_t             flags;

	/** Data ID */
	uint8_t             id;

	/** Payload size */
	uint16_t            payloadSize;

} packet_hdr_t;

/** Specifies the data id, size and offset of a PKT_TYPE_DATA and PKT_TYPE_DATA_SET packet */
typedef struct
{
	/** Data identifier (see eDataIDs) */
	uint8_t             id;

	/** Size of data, for partial requests this will be less than the size of the data structure */
	uint16_t            size;

	/** Offset into data structure */
	uint16_t            offset;
} p_data_hdr_t;

#define MIN_PACKET_SIZE (sizeof(packet_hdr_t) + 2)		// Packet header + checksum, no payload

/** Represents a packet header and body */
typedef struct
{
	union 
	{
		struct 
		{
			/** Packet header */
			packet_hdr_t    hdr;

			/** Data offset (optional) */
			uint16_t        offset;
		};

		struct 
		{
			/** Packet start bytes, always 0x49EF */
			uint16_t        preamble;

			/** Packet identifier (see eISBPacketFlags) */
			uint8_t         flags;

			/** Data offset (optional) */
			p_data_hdr_t 	dataHdr;
		};
	};

	/** Packet data location and size.  For ISB packets this is the payload.  For non-ISB packets (NMEA, UBX, RTCM, etc.) this points to the entire packet. */
	bufPtr_t            data;

	/** Packet header checksum, including offset in payload if it exists */
	uint16_t            hdrCksum;

	/** Packet checksum */
	uint16_t            checksum;

	/** Packet size including header and checksum */
	uint16_t            size;
} packet_t;

/** Represents a packet header and body */
typedef struct
{
	/** Packet header */
	packet_hdr_t        hdr;

	/** Packet body */
	union
	{
		uint8_t         data;
		uint16_t        offset;
	}                   payload;
} packet_buf_t;

typedef struct
{
	/** Header with id, size and offset */
	p_data_hdr_t        hdr;

	/** Data pointer */
	uint8_t             *ptr;
} p_data_t;

typedef struct
{
	/** Header with id, size and offset */
	p_data_hdr_t        hdr;

	/** Data buffer */
	uint8_t             buf[MAX_DATASET_SIZE];
} p_data_buf_t;

/** Represents the complete body of a PKT_TYPE_GET_DATA packet */
typedef struct
{
    /** Data ID being requested */
    uint16_t            id;

    /** Byte length of data from offset */
    uint16_t            size;

    /** Byte offset into data */
    uint16_t            offset;

    /**	The broadcast source period multiple.  0 for a one-time broadcast.  */
	uint16_t            period;
} p_data_get_t;

/** Represents the body header of an ACK or NACK packet */
typedef struct
{
	/** Packet info of the received packet */
	uint16_t            pktInfo;

	/** Packet counter of the received packet */
	uint16_t            pktCounter;
} p_ack_hdr_t;

/** Represents the entire body of an ACK or NACK packet */
typedef struct
{
	/** Body header */
	p_ack_hdr_t         hdr;

	/** Body buffer */
	union 
	{
		uint8_t         buf[sizeof(p_data_hdr_t)];
		p_data_hdr_t	dataHdr;
	}					body;
} p_ack_t, p_nack_t;

/** Ublox binary packet header */
typedef struct
{
	/** Packet start bytes, always 0x62b5 */
	uint16_t            preamble;

	/** Message class */
	uint8_t             classId;

	/** Message ID */
	uint8_t             id;

	/** Payload size */
	uint16_t            payloadSize;

} ubx_pkt_hdr_t;

/** Sony binary packet header */
typedef struct
{
	/** Packet start bytes, always 0x7F */
	uint8_t             preamble;

	/** Data size */
	uint16_t            dataSize;

	/** Opcode */
	uint8_t             opc;

	/** Header checksum */
	uint8_t             fcsh;

} sony_pkt_hdr_t;

typedef struct
{
	/** Start of available buffer */
	uint8_t* start;

	/** End of available buffer */
	uint8_t* end;

	/** Size of buffer */
	uint32_t size;

	/** Start of data in buffer. Data is read from here. */
	uint8_t* head;	// TODO remove this once we have all of the separate parser points in place. WHJ

	/** End of data in buffer. New data is written here. */
	uint8_t* tail;

	/** Search pointer in data (head <= scan <= tail) */
	uint8_t* scan;

	/** Search pointer prior to reset (used to identify errors) */
	uint8_t* scanPrior;

} is_comm_buffer_t;

typedef enum
{
	ENABLE_PROTOCOL_ISB         = 0x00000001,
	ENABLE_PROTOCOL_NMEA  	    = 0x00000002,
	ENABLE_PROTOCOL_UBLOX       = 0x00000004,
	ENABLE_PROTOCOL_RTCM3       = 0x00000008,
	ENABLE_PROTOCOL_SPARTN      = 0x00000010,
	ENABLE_PROTOCOL_SONY        = 0x00000020,
} eProtocolMask;

typedef struct  
{
	/** See eProtocolMask */
	uint32_t enabledMask;
} is_comm_config_t;

typedef struct  
{
	int16_t     state;
	uint16_t    size;
	uint32_t    timeMs;		// Time of last parse
} is_comm_parser_t;

typedef protocol_type_t (*pFnProcessPkt)(void*);

/** An instance of an is_comm interface.  Do not modify these values. */
typedef struct
{
	/** Receive data buffer. Data received is aggregate into this buffer until an entire packet is read. */		
	is_comm_buffer_t rxBuf;
	
	/** Enable/disable protocol parsing */
	is_comm_config_t config;

	/** Number of packets sent */
	uint32_t txPktCount;

	/** Number of valid packets received */
	uint32_t rxPktCount;

	/** Communications error counter */
	uint32_t rxErrorCount;

	/** Process packet function pointer.  Null pointer indicates no parsing is in progress. */
	pFnProcessPkt processPkt;

	/** Protocol parser state */
	is_comm_parser_t parser;

	/** Acknowledge packet needed in response to the last packet received */
	uint32_t ackNeeded;

	/** Receive packet */
	packet_t rxPkt;

	/** Used to prevent counting more than one error count between valid packets */
	uint8_t rxErrorState;

} is_comm_instance_t;

/** Pop off the packing argument, we can safely allow packing and shifting in memory at this point */
POP_PACK

/**
* Init simple communications interface - call this before doing anything else
* @param instance communications instance, please ensure that you have set the buffer and bufferSize
*/
void is_comm_init(is_comm_instance_t* instance, uint8_t *buffer, int bufferSize);

/**
* Decode packet data - when data is available, return value will be the protocol type (see protocol_type_t) and the comm instance dataPtr will point to the start of the valid data.  For Inertial Sense binary protocol, comm instance dataHdr contains the data ID (DID), size, and offset.
* @param instance the comm instance passed to is_comm_init
* @param byte the byte to decode
* @param timeMs current time in milliseconds used for paser timeout.  Used to invalidate packet parsing if PKT_PARSER_TIMEOUT_MS time has lapsed since any data has been received.  
* @return protocol type when complete valid data is found, otherwise _PTYPE_NONE (0) (see protocol_type_t)
* @remarks when data is available, you can cast the comm instance dataPtr into the appropriate data structure pointer (see binary messages above and data_sets.h)
*/
protocol_type_t is_comm_parse_byte_timeout(is_comm_instance_t* instance, uint8_t byte, uint32_t timeMs);

/**
* Decode packet data - when data is available, return value will be the protocol type (see protocol_type_t) and the comm instance dataPtr will point to the start of the valid data.  For Inertial Sense binary protocol, comm instance dataHdr contains the data ID (DID), size, and offset.
* @param instance the comm instance passed to is_comm_init
* @param byte the byte to decode
* @return protocol type when complete valid data is found, otherwise _PTYPE_NONE (0) (see protocol_type_t)
* @remarks when data is available, you can cast the comm instance dataPtr into the appropriate data structure pointer (see binary messages above and data_sets.h)
  For example usage, see comManagerStepRxInstance() in com_manager.c.

	// Read one byte (simple method)
	uint8_t c;
	protocol_type_t ptype;
	// Read from serial buffer until empty
	while (mySerialPortRead(&c, 1))
	{
		if ((ptype = is_comm_parse_byte(comm, c)) != _PTYPE_NONE)
		{
			switch (ptype)
			{
			case _PTYPE_INERTIAL_SENSE_DATA:
			case _PTYPE_INERTIAL_SENSE_CMD:
			case _PTYPE_INERTIAL_SENSE_ACK:
				break;
			case _PTYPE_UBLOX:
				break;
			case _PTYPE_RTCM3:
				break;
			case _PTYPE_NMEA:
				break;
			}
		}
	}
*/
static inline protocol_type_t is_comm_parse_byte(is_comm_instance_t* instance, uint8_t byte)
{
	return is_comm_parse_byte_timeout(instance, byte, 0);
}

/**
* Decode packet data - when data is available, return value will be the protocol type (see protocol_type_t) and the comm instance dataPtr will point to the start of the valid data.  For Inertial Sense binary protocol, comm instance dataHdr contains the data ID (DID), size, and offset.
* @param instance the comm instance passed to is_comm_init
* @param timeMs current time in milliseconds used for paser timeout.  Used to invalidate packet parsing if PKT_PARSER_TIMEOUT_MS time has lapsed since any data has been received.  
* @return protocol type when complete valid data is found, otherwise _PTYPE_NONE (0) (see protocol_type_t)
* @remarks when data is available, you can cast the comm instance dataPtr into the appropriate data structure pointer (see binary messages above and data_sets.h)
*/
protocol_type_t is_comm_parse_timeout(is_comm_instance_t* c, uint32_t timeMs);

/**
* Decode packet data - when data is available, return value will be the protocol type (see protocol_type_t) and the comm instance dataPtr will point to the start of the valid data.  For Inertial Sense binary protocol, comm instance dataHdr contains the data ID (DID), size, and offset.
* @param instance the comm instance passed to is_comm_init
* @return protocol type when complete valid data is found, otherwise _PTYPE_NONE (0) (see protocol_type_t)
* @remarks when data is available, you can cast the comm instance dataPtr into the appropriate data structure pointer (see binary messages above and data_sets.h)
  For example usage, see comManagerStepRxInstance() in com_manager.c.

	// Read a set of bytes (fast method)
	protocol_type_t ptype;

	// Get available size of comm buffer
	int n = is_comm_free(comm);

	// Read data directly into comm buffer
	if ((n = mySerialPortRead(comm->buf.tail, n)))
	{
		// Update comm buffer tail pointer
		comm->buf.tail += n;

		// Search comm buffer for valid packets
		while ((ptype = is_comm_parse(comm)) != _PTYPE_NONE)
		{
			switch (ptype)
			{
			case _PTYPE_INERTIAL_SENSE_DATA:
			case _PTYPE_INERTIAL_SENSE_CMD:
			case _PTYPE_INERTIAL_SENSE_ACK:
				break;
			case _PTYPE_UBLOX:
				break;
			case _PTYPE_RTCM3:
				break;
			case _PTYPE_NMEA:
				break;
			}
		}
	}
*/
static inline protocol_type_t is_comm_parse(is_comm_instance_t* instance)
{
	return is_comm_parse_timeout(instance, 0);
}

/**
* Removed old data and shift unparsed data to the the buffer start if running out of space at the buffer end.  Returns number of bytes available in the bufer.
* @param instance the comm instance passed to is_comm_init
* @return the number of bytes available in the comm buffer 
*/
int is_comm_free(is_comm_instance_t* instance);

/**
* Encode a binary packet to get data from the device - puts the data ready to send into the buffer passed into is_comm_init
* @param instance the comm instance passed to is_comm_init
* @param dataId the data id to request (see DID_* at top of this file)
* @param offset the offset into data to request. Set offset and length to 0 for entire data structure.
* @param length the length into data from offset to request. Set offset and length to 0 for entire data structure.
* @param periodMultiple how often you want the data to stream out, 0 for a one time message and turn off.
* @return the number of bytes written to the comm buffer (from is_comm_init), will be less than 1 if error
* @remarks pass an offset and length of 0 to request the entire data structure
*/
int is_comm_get_data_to_buf(uint8_t *buf, uint32_t buf_size, is_comm_instance_t* comm, uint32_t did, uint32_t offset, uint32_t size, uint32_t periodMultiple);
int is_comm_get_data(pfnIsCommPortWrite portWrite, int port, is_comm_instance_t* comm, uint32_t did, uint32_t offset, uint32_t size, uint32_t periodMultiple);

// /**
// * Encode a binary packet to get predefined list of data sets from the device - puts the data ready to send into the buffer passed into is_comm_init
// * @param comm the comm instance passed to is_comm_init
// * @param RMC bits specifying data messages to stream.  See presets: RMC_PRESET_PPD_BITS = post processing data, RMC_PRESET_INS_BITS = INS2 and GPS data at full rate
// * @return the number of bytes written to the comm buffer (from is_comm_init), will be less than 1 if error
// * @remarks pass an offset and length of 0 to request the entire data structure
// */
// int is_comm_get_data_rmc(is_comm_instance_t* instance, uint64_t rmcBits);

/**
* Encode a binary packet to set data on the device - puts the data ready to send into the buffer passed into is_comm_init.  An acknowledge packet is sent in response to this packet.
* @param comm the comm instance passed to is_comm_init
* @param did the data id to set on the device (see DID_* at top of this file)
* @param size the number of bytes to set on the data structure on the device
* @param offset the offset to start setting data at on the data structure on the device
* @param data the actual data to change on the data structure on the device - this should have at least size bytes available
* @return the number of bytes written to the comm buffer (from is_comm_init), will be less than 1 if error
* @remarks pass an offset and length of 0 to set the entire data structure, in which case data needs to have the full number of bytes available for the appropriate struct matching the dataId parameter.
*/
int is_comm_set_data_to_buf(uint8_t* buf, uint32_t buf_size, is_comm_instance_t* comm, uint16_t did, uint16_t size, uint16_t offset, void* data);
int is_comm_set_data(pfnIsCommPortWrite portWrite, int port, is_comm_instance_t* comm, uint16_t did, uint16_t size, uint16_t offset, void* data);

/**
* Same as is_comm_set_data() except NO acknowledge packet is sent in response to this packet.
*/
int is_comm_data_to_buf(uint8_t* buf, uint32_t buf_size, is_comm_instance_t* comm, uint16_t did, uint16_t size, uint16_t offset, void* data);
int is_comm_data(pfnIsCommPortWrite portWrite, int port, is_comm_instance_t* comm, uint16_t did, uint16_t size, uint16_t offset, void* data);

/**
* Encode a binary packet to stop all messages being broadcast on the device on all ports
* @param comm the comm instance passed to is_comm_init
* @return 0 if success, otherwise an error code
*/
int is_comm_stop_broadcasts_all_ports(pfnIsCommPortWrite portWrite, int port, is_comm_instance_t* comm);

/**
* Encode a binary packet to stop all messages being broadcast on the device on this port
* @param comm the comm instance passed to is_comm_init
* @return 0 if success, otherwise an error code
*/
int is_comm_stop_broadcasts_current_ports(pfnIsCommPortWrite portWrite, int port, is_comm_instance_t* comm);

/**
 * @brief Compute the fletcher 16 bit checksum for the given data array.
 * 
 * @param cksum_init initial value for the checksum.
 * @param data data array used for checksum.
 * @param size size of data arary.
 * @return uint16_t 
 */
uint16_t is_comm_fletcher16(uint16_t cksum_init, const void* data, uint32_t size);

/**
 * @brief Compute the xor 16 bit checksum for the given data array.
 * 
 * @param cksum_init initial value for the checksum.
 * @param data data array used for checksum.
 * @param size size of data arary.
 * @return uint16_t 
 */
uint16_t is_comm_xor16(uint16_t cksum_init, const void* data, uint32_t size);
#define is_comm_isb_checksum16  is_comm_fletcher16
// #define is_comm_isb_checksum16  is_comm_xor16


// -------------------------------------------------------------------------------------------------------------------------------
// Common packet encode / decode functions
// -------------------------------------------------------------------------------------------------------------------------------

/**
 * @brief Encode ISB InertialSense binary (ISB) packet header.
 * 
 * @param pkt Packet storage location.
 * @param flags ISB packet flags which includes the packet type (see eISBPacketFlags).
 * @param did ISB data ID
 * @param data_size Size in bytes of the payload data.
 * @param offset Offset of the payload data into the data set structure.
 * @param data Pointer to payload data.
 */
void is_comm_encode_hdr(packet_t *pkt, uint8_t flags, uint16_t did, uint16_t data_size, uint16_t offset, void* data);
// Returns number of bytes written
int is_comm_write_isb_precomp_to_port(pfnIsCommPortWrite portWrite, int port, is_comm_instance_t* comm, packet_t *pkt);


/**
 * @brief Generate InertialSense binary (ISB) packet.
 * 
 * @param buf Buffer to write to.
 * @param buf_size Available size of buffer.
 * @param comm ISComm instance.
 * @param flags ISB packet flags which includes the packet type (see eISBPacketFlags).
 * @param did ISB data ID
 * @param data_size Size in bytes of the payload data.
 * @param offset Offset of the payload data into the data set structure.
 * @param data Pointer to payload data.
 * @return number of bytes written 
 */
int is_comm_write_to_buf(uint8_t* buf, uint32_t buf_size, is_comm_instance_t* comm, uint8_t flags, uint16_t did, uint16_t data_size, uint16_t offset, void* data);
int is_comm_write(pfnIsCommPortWrite portWrite, int port, is_comm_instance_t* comm, uint8_t flags, uint16_t did, uint16_t data_size, uint16_t offset, void* data);

unsigned int calculate24BitCRCQ(unsigned char* buffer, unsigned int len);
unsigned int getBitsAsUInt32(const unsigned char* buffer, unsigned int pos, unsigned int len);

int validateBaudRate(unsigned int baudRate);

/** Copies data structure into packet data.  Data copied is limited to the size and offset specified in p_data_t *data.  Returns 0 on success, -1 on failure. */
char copyStructPToDataP(p_data_t *data, const void *sptr, const unsigned int maxsize);

/** Copies packet data into a data structure.  Returns 0 on success, -1 on failure. */
char copyDataPToStructP(void *sptr, const p_data_t *data, const unsigned int maxsize);
char copyDataBufPToStructP(void *sptr, const p_data_buf_t *data, const unsigned int maxsize);

/** Copies packet data into a data structure.  Returns 0 on success, -1 on failure. */
char copyDataPToStructP2(void *sptr, const p_data_hdr_t *dataHdr, const uint8_t *dataBuf, const unsigned int maxsize);

/** Indicates whether there is overlap in the data received and the backing data structure */
static inline uint8_t dataOverlap( uint32_t dstOffset, uint32_t dstSize, p_data_t* src)
{
	return _MAX(dstOffset, (uint32_t)(src->hdr.offset)) < _MIN(dstOffset + dstSize, (uint32_t)(src->hdr.offset + src->hdr.size));
}

/** Reset parser state */
static inline void is_comm_reset_parser(is_comm_instance_t* c)
{
	c->parser.state = 0;
	c->rxBuf.scanPrior = c->rxBuf.scan;
	c->rxBuf.scan = c->rxBuf.head;
	c->processPkt = NULL;
}

/** Copies is_comm_instance data into a data structure.  Returns 0 on success, -1 on failure. */
char is_comm_copy_to_struct(void *sptr, const is_comm_instance_t *com, const unsigned int maxsize);

/** Returns -1 if the baudrate is not a standard baudrate. */
int validateBaudRate(unsigned int baudRate);

#ifdef __cplusplus
}
#endif

#endif // IS_COMM_H
