/*
MIT LICENSE

Copyright (c) 2014-2025 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

/**
 * @file ISComm.h
 * @brief Inertial Sense Binary (ISB) framing, multi-protocol parser, and low-level
 *        port communication interface.
 *
 * Defines the packet structures, protocol type enumeration, and the stateful
 * is_comm_instance_t parser used by all IS SDK layers.  The parser supports
 * simultaneous detection of ISB, NMEA, u-blox, RTCM3, SPARTN, Sony, and
 * Septentrio protocols from a single byte stream.
 *
 * ### Quick-start (byte-by-byte parsing)
 * @code
 * is_comm_instance_t comm;
 * uint8_t buffer[PKT_BUF_SIZE];
 * is_comm_init(&comm, buffer, sizeof(buffer), NULL);
 *
 * uint8_t c;
 * protocol_type_t ptype;
 * while (mySerialPortRead(&c, 1)) {
 *     if ((ptype = is_comm_parse_byte(&comm, c)) != _PTYPE_NONE) {
 *         // process ptype, comm.rxPkt, etc.
 *     }
 * }
 * @endcode
 *
 * ### Quick-start (bulk-read parsing)
 * @code
 * int n = is_comm_free(&comm);
 * if ((n = mySerialPortRead(comm.rxBuf.tail, n)) > 0) {
 *     comm.rxBuf.tail += n;
 *     while ((ptype = is_comm_parse(&comm)) != _PTYPE_NONE) { ... }
 * }
 * @endcode
 *
 * @author Inertial Sense, Inc.
 * @copyright Copyright (c) 2014-2026 Inertial Sense, Inc. - http://inertialsense.com
 */

#ifndef IS_COMM_H
#define IS_COMM_H

#include "data_sets.h"
#include "stddef.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 *    DEFINITIONS AND CONVENTIONS
 *    
 *    INS   = inertial navigation system
 *    AHRS  = attitude heading reference system
 *    IMU   = inertial measurement unit: gyros (rad/s), accelerometers (m/s^2)
 *    ECEF  = earth-centered earth fixed: x,y,z or vx,vy,vz (m or m/s)
 *    LLA   = latitude, longitude, altitude (degrees,m)
 *    NED   = north, east, down (m or m/s)
 *    QE2B  = quaternion rotation from ECEF frame to local frame.
 *    QN2B  = quaternion rotation from NED frame to local frame.
 *    UVW   = velocities in local frame.
*/

// -------------------------------------------------------------------------------------------------------------------------------
// Inertial Sense simple communications interface --------------------------------------------------------------------------------
// -------------------------------------------------------------------------------------------------------------------------------
// The simple comm interface does not require any of the com manager APIs and is designed for simple or lightweight scenarios, tiny embedded platforms, etc.
// *****************************************************************************
// ****** Binary messages                                                 ******
// *****************************************************************************

#define _DID_INS_LLA_EULER_NED  DID_INS_1                   //!< (see ins_1_t) INS/AHRS output: euler from NED, LLA (degrees,m), NED pos (m) and vel (m/s) from refLLA
#define _DID_INS_LLA_QN2B       DID_INS_2                   //!< (see ins_2_t) INS/AHRS output: quaternion from NED, LLA (degrees,m)
#define _DID_INS_LLA_QN2B_MSL   DID_INS_3                   //!< (see ins_3_t) INS/AHRS output: quaternion from NED, LLA (degrees,m), and MSL altitude
#define _DID_INS_ECEF_QE2B      DID_INS_4                   //!< (see ins_4_t) INS output: ECEF position (m) and velocity (m/s), quaternion from ECEF

#define _DID_IMU                DID_IMU                     //!< (see imu_t) IMU output: angular rate (rad/s) and linear acceleration (m/s^2)
#define _DID_PIMU               DID_PIMU                    //!< (see pimu_t) IMU output: Coning and sculling integrated at IMU update rate

#define _DID_GNSS1_POS          DID_GNSS1_POS               //!< (see gnss_pos_t) GPS output

#define _DID_MAG_CAL            DID_MAG_CAL                 //!< (see mag_cal_t) Magnetometer calibration
#define _DID_MAGNETOMETER       DID_MAGNETOMETER            //!< (see magnetometer_t) Magnetometer sensor output
#define _DID_BAROMETER          DID_BAROMETER               //!< (see barometer_t) Barometric pressure sensor data
#define _DID_WHEEL_ENCODER      DID_WHEEL_ENCODER           //!< (see wheel_encoder_t) Wheel encoder sensor data
#define _DID_POS_MEASUREMENT    DID_POSITION_MEASUREMENT    //!< (see pos_measurement_t) Position Measurement data

#define _DID_DEV_INFO           DID_DEV_INFO                //!< (see dev_info_t) Device information
#define _DID_BIT                DID_BIT                     //!< (see bit_t) System built-in self-test
#define _DID_STROBE_IN_TIME     DID_STROBE_IN_TIME          //!< (see strobe_in_time_t) Timestamp for input strobe

#define _DID_FLASH_CONFIG       DID_FLASH_CONFIG            //!< (see nvm_flash_cfg_t) Flash memory configuration
#define _DID_RMC                DID_RMC                     //!< (see rmc_t) Realtime message controller

#define ZEPHYR_SUCCESS_CODE     0                           //!< Return code for success on Zephyr RTOS (equivalent to 0 on other platforms)

/** Default protocol enable mask for a new port (ISB + NMEA + u-blox + RTCM3). */
#define DEFAULT_PORT_PROTO_CFG  (ENABLE_PROTOCOL_ISB | ENABLE_PROTOCOL_NMEA | ENABLE_PROTOCOL_UBLOX | ENABLE_PROTOCOL_RTCM3)

/** Protocol Type */
typedef enum
{
    _PTYPE_NONE                 = 0,    /** No complete valid data available yet */
    _PTYPE_PARSE_ERROR          = 1,    /** Invalid data or checksum error */
    _PTYPE_INERTIAL_SENSE_ACK   = 2,    /** Protocol Type: Inertial Sense binary acknowledge (ack) or negative acknowledge (PID_ACK, PID_NACK)  */
    _PTYPE_INERTIAL_SENSE_CMD   = 3,    /** Protocol Type: Inertial Sense binary command (PID_GET_DATA, PID_STOP_BROADCASTS...) */
    _PTYPE_INERTIAL_SENSE_DATA  = 4,    /** Protocol Type: Inertial Sense binary data (PID_SET_DATA, PID_DATA) */
    _PTYPE_NMEA                 = 5,    /** Protocol Type: NMEA (National Marine Electronics Association) */
    _PTYPE_UBLOX                = 6,    /** Protocol Type: uBlox binary */
    _PTYPE_RTCM3                = 7,    /** Protocol Type: RTCM3 binary (Radio Technical Commission for Maritime Services) */
    _PTYPE_SPARTN               = 8,    /** Protocol Type: SPARTN binary */
    _PTYPE_SONY                 = 9,    /** Protocol Type: Sony binary */
    _PTYPE_SEPTENTRIO_SBF       = 10,   /** Protocol Type: Septentrio binary */
    _PTYPE_SEPTENTRIO_REPLY     = 11,   /** Protocol Type: Septentrio reply msg */
    _PTYPE_FIRST_DATA           = _PTYPE_INERTIAL_SENSE_DATA,  //!< First protocol type that carries data payload
    _PTYPE_LAST_DATA            = _PTYPE_SEPTENTRIO_REPLY,      //!< Last protocol type that carries data payload
    _PTYPE_SIZE                 = _PTYPE_LAST_DATA + 1,         //!< Total number of protocol type values
} protocol_type_t;

/** Default protocol enable mask used by is_comm_init() when no explicit mask is set. */
#define DEFAULT_PROTO_MASK (ENABLE_PROTOCOL_ISB | ENABLE_PROTOCOL_NMEA | ENABLE_PROTOCOL_UBLOX | ENABLE_PROTOCOL_RTCM3)

/** The maximum buffer space that is used for sending and receiving packets */
#ifndef PKT_BUF_SIZE
#define PKT_BUF_SIZE            2048
#endif

/** The maximum time between received data that will reset in the parser */
#define MAX_PARSER_GAP_TIME_MS  100

// MAX_DATASET_SIZE, PKT_OVERHEAD_SIZE, MAX_PKT_OVERHEAD_SIZE, MAX_PKT_BODY_SIZE, and
// MAX_P_DATA_BODY_SIZE are defined below, once packet_hdr_t and p_data_hdr_t exist, since they're
// computed from sizeof() those structs. MAX_P_ACK_BODY_SIZE is defined further below, after p_ack_hdr_t.

/** Binary checksum start value */
#define CHECKSUM_SEED 0x00AAAAAA

/** Communications Protocol Version. See release notes. */

// Increment w/ breaking changes (in ISComm.cpp) that prevent backwards compatibility with older protocols. 
#define PROTOCOL_VERSION_CHAR0      2   // Breaking changes (Packet)
// #define PROTOCOL_VERSION_CHAR1      .   // Breaking changes (Payload)       (defined in data_sets.h)

// Increment w/ non-breaking changes (in data_sets.h) that would still backward compatibility with older protocols
#define PROTOCOL_VERSION_CHAR2      0   //!< Non-breaking protocol version increment (packet layer)
// #define PROTOCOL_VERSION_CHAR3      .   // Non-breaking changes (Payload)   (defined in data_sets.h)

#define UBLOX_HEADER_SIZE           6   //!< Byte size of the u-blox binary packet header
#define RTCM3_HEADER_SIZE           3   //!< Byte size of the RTCM3 packet header
#define MAX_MSG_LENGTH_NMEA         200 //!< Maximum byte length of a single NMEA sentence

/** Send data to the serial port.  Returns number of bytes written. */ 
typedef int(*pfnIsCommPortWrite)(port_handle_t port, const uint8_t* buf, int len);

/** Read data from the serial port.  Returns number of bytes read. */ 
typedef int(*pfnIsCommPortRead)(port_handle_t port, uint8_t* buf, int bufLen);

/** We must not allow any packing or shifting as these data structures must match exactly in memory on all devices */
PUSH_PACK_1

/** Valid baud rates for Inertial Sense hardware */
typedef enum
{
    IS_BAUDRATE_9600            = 9600,
    IS_BAUDRATE_19200           = 19200,
    IS_BAUDRATE_38400           = 38400,
    IS_BAUDRATE_57600           = 57600,
    IS_BAUDRATE_115200          = 115200,       //  IMX-5,  uINS-3,  Actual baudrates                                             
    IS_BAUDRATE_230400          = 230400,       //   230547,  232700, 
    IS_BAUDRATE_460800          = 460800,       //   462428,  468600, 
    IS_BAUDRATE_921600          = 921600,       //   930233,  937734,
    IS_BAUDRATE_10000000        = 10000000,     // 10000000  (IMX-5 only)
    IS_BAUDRATE_COUNT           = 9,
    IS_BAUDRATE_DEFAULT         = IS_BAUDRATE_921600,
    IS_BAUDRATE_STANDARD_MIN    = IS_BAUDRATE_9600,
    IS_BAUDRATE_STANDARD_MAX    = IS_BAUDRATE_921600,
    IS_BAUDRATE_MAX             = IS_BAUDRATE_10000000,
} baud_rate_t;

/** Serial port framing options. */
typedef struct
{
    uint32_t    baudRate;   //!< Baud rate (see @ref baud_rate_t for valid values)
    uint8_t     parity;     //!< Parity setting: 0 = none, 1 = odd, 2 = even
    uint8_t     stopBits;   //!< Number of stop bits: 1 or 2
} serial_options_t;

/** List of valid baud rates */
extern const unsigned int g_validBaudRates[IS_BAUDRATE_COUNT];

/*
Packet Overview

Byte
0               Packet start byte
1               Packet indo: ID (mask 0x1F) | reserved bits (mask 0xE)
2               Packet counter (for ACK and retry)
3               Packet flags

// packet body, may or may not exist depending on packet id - packet body is made up of 4 byte or 8 byte values.
4-7             Data identifier
8-11            Data length
12-15           Data offset
16-19           Data start
(n-8)-(n-5)     Last piece of data
// end data

n-4             Reserved
n-3             Checksum high byte
n-2             Checksum low byte
n-1             Packet end byte
*/

// Packet IDs
// typedef uint32_t ePacketIDs;

/** ISB packet type identifiers and header flag bits. The lower nibble of packet_hdr_t::flags holds the type; the upper nibble holds the flags. */
typedef enum
{
    PKT_TYPE_INVALID                        = 0,    //!< Invalid packet id
    PKT_TYPE_ACK                            = 1,    //!< Positive acknowledgement of a received packet
    PKT_TYPE_NACK                           = 2,    //!< Negative acknowledgement (parse or checksum error)
    PKT_TYPE_GET_DATA                       = 3,    //!< Request to broadcast a data set; response is PKT_TYPE_DATA
    PKT_TYPE_DATA                           = 4,    //!< Data packet sent in response to PKT_TYPE_GET_DATA; no ACK sent
    PKT_TYPE_SET_DATA                       = 5,    //!< Write data to the device; device responds with PKT_TYPE_ACK
    PKT_TYPE_STOP_BROADCASTS_ALL_PORTS      = 6,    //!< Stop all data broadcasts on all ports; responds with ACK
    PKT_TYPE_STOP_DID_BROADCAST             = 7,    //!< Stop a specific DID broadcast
    PKT_TYPE_STOP_BROADCASTS_CURRENT_PORT   = 8,    //!< Stop all data broadcasts on the current port; responds with ACK
    PKT_TYPE_COUNT                          = 9,    //!< Total number of defined packet types
    PKT_TYPE_MAX_COUNT                      = 16,   //!< Maximum supported packet type count (4-bit type field)
    PKT_TYPE_MASK                           = 0x0F, //!< Bitmask for the packet type field in the flags byte

    ISB_FLAGS_MASK                          = 0xF0, //!< Bitmask for the packet flags field (upper nibble)
    ISB_FLAGS_EXTENDED_PAYLOAD              = 0x10, //!< Payload exceeds 2048 bytes and continues in the next packet
    ISB_FLAGS_PAYLOAD_W_OFFSET              = 0x20, //!< First two bytes of the payload contain the data-set byte offset

    /**
     * @brief Only meaningful on a PKT_TYPE_GET_DATA packet whose p_data_get_t::period is 0. A
     *        plain period=0 GET_DATA is a genuine one-shot request (deliver the current value
     *        once) -- but on a device that also honors this flag, it ALSO implicitly stops any
     *        existing broadcast for that DID on the requesting port, since there was previously
     *        no way to distinguish "just poll me a value" from "stop this stream" (SN-8471).
     *        Setting this bit tells a device that supports it: if this DID's broadcast bit is
     *        already set on this port, leave it alone (deliver the one-shot reply without
     *        disturbing the existing stream). Unset (the default for every existing caller),
     *        behavior is unchanged. Ignored on older firmware that doesn't recognize it -- an
     *        unrecognized upper-nibble flag bit is never validated/rejected by the parser, so a
     *        period=0 GET_DATA from a newer client talking to older firmware still stops any
     *        existing stream exactly as it always has.
     */
    ISB_FLAGS_GET_DATA_PRESERVE_STREAM      = 0x40,
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
    /** Dollar sign ($), used by NMEA and Septentrio protocol to signify start of message (36) */
    PSC_PRE_ASCII_START_BYTE = 0x24,

    /** Dollar sign ($), used by NMEA protocol to signify start of message (36) */
    PSC_NMEA_START_BYTE = 0x24,

    /** Carriage return (CR, 0x0D), used by NMEA protocol to signify one byte before end of message */
    PSC_NMEA_PRE_END_BYTE = 0x0D,

    /** New line (LF, 0x0A), used by NMEA protocol to signify end of message */
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

    /** Septentrio GNSS Second bytes */
    /** Dollar sign ($), used by Septentrio protocol to signify start of message (36) */
    SEPT_PROTO_START_BYTE   = 0x24, // 0x24 = '$'
    SEPT_SBF_PREAMBLE_BYTE2 = 0x40, // 0x40 = '@'
    SEPT_REPLY_BYTE2        = 0x52, // 0x52 = 'R'
    SEPT_REPLY_PRE_END_BYTE = 0x0d, // 0x0d = '\r' <CR>
    SEPT_REPLY_END_BYTE     = 0x0a, // 0x0a = '\n' <LF>
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

#define ISB_MIN_PACKET_SIZE             (sizeof(packet_hdr_t) + 2)                                      //!< Minimum ISB packet size: header + 2-byte checksum, no payload
#define ISB_HDR_TO_PACKET_SIZE(hdr)     ((hdr).size + ISB_MIN_PACKET_SIZE + ((hdr).offset ? 2 : 0))     //!< Compute total ISB packet byte size from a packet_hdr_t

/** The overhead involved in sending a packet: @ref packet_hdr_t header + 2-byte checksum footer.
 *  Same computation as @ref ISB_MIN_PACKET_SIZE (a packet with zero-length payload is pure overhead). */
#define PKT_OVERHEAD_SIZE       ISB_MIN_PACKET_SIZE

/** The maximum overhead size in sending a packet. Equal to @ref PKT_OVERHEAD_SIZE: protocol 2.x
 *  (see PROTOCOL_VERSION_CHAR0) is a length-prefixed binary format with no byte-stuffing/escaping,
 *  so unlike the old v1 protocol's PSC_RESERVED_KEY escaping, there is no worst-case encoding growth
 *  to account for here. */
#define MAX_PKT_OVERHEAD_SIZE   PKT_OVERHEAD_SIZE

/** The maximum size of a decoded packet body: full buffer minus header/footer overhead, rounded down to an even number. */
#define MAX_PKT_BODY_SIZE       ((PKT_BUF_SIZE - MAX_PKT_OVERHEAD_SIZE) & 0xFFFFFFFE)

/** The maximum size of decoded data in a packet body */
#define MAX_P_DATA_BODY_SIZE    (MAX_PKT_BODY_SIZE-sizeof(p_data_hdr_t))    // Data size limit

/** The maximum allowable dataset size: tied directly to the maximum packet payload capacity.
 *  Note: for packets with ISB_FLAGS_PAYLOAD_W_OFFSET set, pkt->data.size already excludes the 2-byte offset.
 *  This limit intentionally bounds pkt->data.size (dataset bytes), not on-wire payloadSize. */
#define MAX_DATASET_SIZE        MAX_PKT_BODY_SIZE

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
            p_data_hdr_t    dataHdr;
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

    /** Packet ID */
    uint16_t            id;
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

/** ISB data packet: pointer form — header plus a pointer to an external payload buffer. */
typedef struct
{
    /** Header with id, size and offset */
    p_data_hdr_t        hdr;

    /** Data pointer */
    uint8_t             *ptr;
} p_data_t;

/** ISB data packet: buffer form — header plus an inline payload buffer up to @ref MAX_DATASET_SIZE bytes. */
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

    /**    The broadcast source period multiple.  0 for a one-time broadcast.  */
    uint16_t            period;
} p_data_get_t;

/** Represents the body header of an ACK or NACK packet */
typedef struct
{
    /** Packet info of the received packet */
    struct ISComm
    {
        uint8_t         flags;             //!< Packet flags of received packet (see eISBPacketFlags)
        uint8_t         id;                //!< DID of received packet
    }                   pktInfo;

    /** Packet counter of the received packet */
    uint16_t            pktCounter;
} p_ack_hdr_t;

/** The maximum size of a decoded ACK message */
#define MAX_P_ACK_BODY_SIZE     (MAX_PKT_BODY_SIZE-sizeof(p_ack_hdr_t))     // Ack data size

/** Represents the entire body of an ACK or NACK packet */
typedef struct
{
    /** Body header */
    p_ack_hdr_t         hdr;

    /** Body buffer */
    union 
    {
        uint8_t         buf[sizeof(p_data_hdr_t)];
        p_data_hdr_t    dataHdr;
    }                   body;
} p_ack_t,
  p_nack_t; //!< Alias for p_ack_t used when the packet represents a negative acknowledgement

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


/** Septentrio SBF binary packet header. */
typedef struct
{
    uint8_t syncChar1;      //!< First sync byte, always 0x24 ('$')
    uint8_t syncChar2;      //!< Second sync byte, always 0x40 ('@')
    uint16_t crc;           //!< CRC-CCITT checksum of the payload bytes
    uint16_t msgID;         //!< SBF block identifier
    uint16_t payloadSize;   //!< Size of the payload in bytes
} sept_pkt_hdr_t;

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

/** Sliding-window receive buffer used by is_comm_instance_t to accumulate inbound bytes. */
typedef struct
{
    /** Start of available buffer */
    uint8_t* start;

    /** End of available buffer */
    uint8_t* end;

    /** Size of buffer */
    uint32_t size;

    /** Start of data in buffer. Data is read from here. */
    uint8_t* head;

    /** End of data in buffer. New data is written here. */
    uint8_t* tail;

    /** Search pointer in data (head <= scan <= tail) */
    uint8_t* scan;

    /** Search pointer prior to reset (used to identify errors) */
    uint8_t* scanPrior;

} is_comm_buffer_t;

/** Bitmask constants for enabling/disabling protocol parsers in is_comm_instance_t::cb::protocolMask. */
typedef enum
{
    ENABLE_PROTOCOL_ISB         = (0x00000001 << _PTYPE_INERTIAL_SENSE_DATA),  //!< Enable ISB parser
    ENABLE_PROTOCOL_NMEA        = (0x00000001 << _PTYPE_NMEA),                 //!< Enable NMEA parser
    ENABLE_PROTOCOL_UBLOX       = (0x00000001 << _PTYPE_UBLOX),                //!< Enable u-blox binary parser
    ENABLE_PROTOCOL_RTCM3       = (0x00000001 << _PTYPE_RTCM3),                //!< Enable RTCM3 parser
    ENABLE_PROTOCOL_SPARTN      = (0x00000001 << _PTYPE_SPARTN),               //!< Enable SPARTN parser
    ENABLE_PROTOCOL_SONY        = (0x00000001 << _PTYPE_SONY),                 //!< Enable Sony GNSS binary parser
    ENABLE_PROTOCOL_SBF         = (0x00000001 << _PTYPE_SEPTENTRIO_SBF),       //!< Enable Septentrio SBF parser
    ENABLE_PROTOCOL_SEPT_REPLY  = (0x00000001 << _PTYPE_SEPTENTRIO_REPLY),     //!< Enable Septentrio reply parser
    ENABLE_PROTOCOL_SEPT        = (ENABLE_PROTOCOL_SBF| ENABLE_PROTOCOL_SEPT_REPLY), //!< Enable all Septentrio parsers
} eProtocolMask;

/** Parse error type codes stored in is_comm_instance_t::rxErrorType. */
typedef enum {
    EPARSE_INVALID_PREAMBLE,        //!< Packet preamble bytes were not recognized
    EPARSE_INVALID_SIZE,            //!< Declared payload size exceeds buffer limits
    EPARSE_INVALID_CHKSUM,          //!< Checksum verification failed
    EPARSE_INVALID_DATATYPE,        //!< Data type/DID is outside the valid range
    EPARSE_MISSING_EOS_MARKER,      //!< End-of-sentence marker (NMEA CRLF) was not found
    EPARSE_INCOMPLETE_PACKET,       //!< Stream/sentence is too short to be a valid packet
    EPARSE_INVALID_HEADER,          //!< Packet header fields are malformed
    EPARSE_INVALID_PAYLOAD,         //!< Payload content failed validation
    EPARSE_RXBUFFER_FLUSHED,        //!< RX buffer was flushed because the packet exceeded its capacity
    EPARSE_STREAM_UNPARSABLE,       //!< Byte stream does not match any supported protocol
    NUM_EPARSE_ERRORS               //!< Sentinel: total number of parse error types
} eParseErrorType;

/** Internal per-protocol parser state used by is_comm_instance_t. */
typedef struct
{
    int16_t     state;      //!< Protocol-specific parser FSM state variable
    uint16_t    size;       //!< Byte count accumulated by the active parser
    uint32_t    timeMs;     //!< Timestamp of the last byte received (ms), used for gap detection
} is_comm_parser_t;

/** Internal function pointer type for per-protocol packet parsing steps. */
typedef protocol_type_t (*pFnProcessPkt)(void*);

// broadcast message handler
// typedef int(*pfnIsCommAsapMsg)(p_data_get_t* req, port_handle_t port);

/**
 * InertialSense binary (ISB) data message handler function
 * @returns 0 if this message was successfully processed by the handler and should not be processed further;
 *   any other value will cause the message to be processed by additional handlers (if registered), and may
 *   cause "duplication of data"-type errors (ie, double accumulation of values, etc).
 */
typedef int(*pfnIsCommIsbDataHandler)(void* ctx, p_data_t* data, port_handle_t port);

/**
 * Generic message handler function with message pointer and size - this is used for non ISB messages
 * such as NMEA, Ublox, and other "packet-like" messages in which the entire message can be reduced to a
 * byte stream of msgSize length.
 * @returns 0 if this message was successfully processed by the handler and should not be processed further;
 *   any other value will cause the message to be processed by additional handlers (if registered), and may
 *   cause "duplication of data"-type errors (ie, double accumulation of values, etc).
 */
typedef int(*pfnIsCommGenMsgHandler)(void* ctx, const unsigned char* msg, int msgSize, port_handle_t port);

/**
 * raw packet handler function with is_comm_instance_t
 * @returns 0 if this message was successfully processed by the handler and should not be processed further;
 *   any other value will cause the message to be processed by additional handlers (if registered), and may
 *   cause "duplication of data"-type errors (ie, double accumulation of values, etc).
 */
typedef int(*pfnIsCommHandler)(void* ctx, protocol_type_t ptype, packet_t *pkt, port_handle_t port);


/** Set of per-protocol callback functions attached to an is_comm_instance_t. */
typedef struct
{
    /** Bitmask of enabled protocol types (see @ref eProtocolMask). Each bit enables parsing of the corresponding @ref protocol_type_t. */
    uint32_t                        protocolMask;
    void*                           context;            //!< User context pointer passed to every callback
    pfnIsCommHandler                all;                //!< Called for every parsed packet of any protocol; NULL to skip
    pfnIsCommIsbDataHandler         isbData;            //!< Called for every ISB data packet; NULL to skip
    pfnIsCommGenMsgHandler          generic[_PTYPE_SIZE]; //!< Per-protocol callbacks indexed by @ref protocol_type_t; NULL entries are skipped
} is_comm_callbacks_t;


/** An instance of an is_comm interface.  Do not modify these values. */
typedef struct
{
    /** Receive data buffer. Data received is aggregate into this buffer until an entire packet is read. */        
    is_comm_buffer_t rxBuf;

    /** Number of packets sent */
    uint32_t txPktCount;

    /** Number of valid packets received */
    uint32_t rxPktCount;

    /** Communications error counter */
    uint32_t rxErrorCount;

    /** Communications error type (most recent) */
    eParseErrorType rxErrorType;

    /** Communications error counter, by type */
    uint32_t rxErrorTypeCount[NUM_EPARSE_ERRORS];

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

    /** a set of callbacks that will be called each time a protocol of the corresponding type is parsed */
    is_comm_callbacks_t cb;
} is_comm_instance_t;

static const uint8_t COMM_PORT_FLAG__EXPLICIT_READ  = 0x01;     //!< When set, ISComm::is_comm_port_parse_messages() will not read/parse from this port; the operator must readPort() and then is_comm_buffer_parse_messages in separate, explicit steps.

/**
 * @brief Composite port type that bundles a base_port_t with an embedded ISComm parser instance.
 *
 * comm_port_t is layout-compatible with base_port_t (first member), so a pointer to
 * comm_port_t can be cast to base_port_t* and used with the generic port API.
 */
typedef struct {
    base_port_t base;
    is_comm_instance_t comm;                //!< Comm instance
#if defined(GPX_1)
    #define GPX_COM_BUFFER_SIZE 2800
    uint8_t buffer[GPX_COM_BUFFER_SIZE];    //!< Comm instance data buffer
    uint8_t flags;                          //!< COMM_PORT flags (ie, EXPLICIT, etc)
#else
    uint8_t buffer[PKT_BUF_SIZE];           //!< Comm instance data buffer
    uint8_t flags;                          //!< COMM_PORT flags (ie, EXPLICIT, etc)
#endif
} comm_port_t;
#define COMM_PORT(n)    ((comm_port_t*)(n))  //!< Cast a port_handle_t to a comm_port_t pointer


/** Pop off the packing argument, we can safely allow packing and shifting in memory at this point */
POP_PACK

/**
 * @brief Initialize an is_comm_instance_t. Call this before using any other is_comm_* functions.
 * @param instance   ISComm instance to initialize.
 * @param buffer     Caller-provided receive buffer; must remain valid for the lifetime of @p instance.
 * @param bufferSize Size of @p buffer in bytes; should be at least @ref PKT_BUF_SIZE.
 * @param pktHandler Optional: called for every fully parsed packet; may be NULL.
 */
void is_comm_init(is_comm_instance_t* instance, uint8_t *buffer, int bufferSize, pfnIsCommHandler pktHandler);

/**
 * @brief Initialize an is_comm_instance_t embedded inside a comm_port_t.
 * @param port       comm_port_t to initialize; the embedded is_comm_instance_t and
 *                   the internal buffer are configured automatically.
 * @param pktHandler Optional packet handler called for every fully parsed packet; may be NULL.
 */
void is_comm_port_init(comm_port_t* port, pfnIsCommHandler pktHandler);

/**
 * @brief Retrieve the is_comm_instance_t embedded in a port handle.
 * @param port Port handle previously initialized with is_comm_port_init() or equivalent.
 * @return Pointer to the embedded is_comm_instance_t, or NULL if @p port is NULL.
 */
is_comm_instance_t* is_comm_get_port_instance(port_handle_t port);

/**
 * @brief Register a handler called for every successfully parsed packet (any protocol).
 * @param comm      ISComm instance to configure.
 * @param cbHandler New handler function; replaces any previously registered all-protocol handler.
 * @return The previously registered all-protocol handler, or NULL if none was set.
 */
pfnIsCommHandler is_comm_register_all_handler(is_comm_instance_t* comm, pfnIsCommHandler cbHandler);

/**
 * @brief Register a handler called for every received ISB data packet.
 * @param comm      ISComm instance to configure.
 * @param cbHandler New ISB data handler; replaces any previously registered handler.
 * @return The previously registered ISB data handler, or NULL if none was set.
 */
pfnIsCommIsbDataHandler is_comm_register_isb_handler(is_comm_instance_t* comm, pfnIsCommIsbDataHandler cbHandler);

/**
 * @brief Register an ISB data handler on a port handle.
 * @param port      Port handle whose embedded is_comm_instance_t is configured.
 * @param cbHandler New ISB data handler.
 * @return The previously registered ISB data handler, or NULL if none was set.
 */
pfnIsCommIsbDataHandler is_comm_register_port_isb_handler(port_handle_t port, pfnIsCommIsbDataHandler cbHandler);

/**
 * @brief Register a handler for a specific non-ISB protocol type.
 * @param comm      ISComm instance to configure.
 * @param ptype     Protocol type to handle (see @ref protocol_type_t; must not be _PTYPE_INERTIAL_SENSE_DATA).
 * @param cbHandler New handler for @p ptype; replaces any previously registered handler for that type.
 * @return The previously registered handler for @p ptype, or NULL if none was set.
 */
pfnIsCommGenMsgHandler is_comm_register_msg_handler(is_comm_instance_t* comm, int ptype, pfnIsCommGenMsgHandler cbHandler);

/**
 * @brief Register a protocol-specific handler on a port handle.
 * @param port      Port handle whose embedded is_comm_instance_t is configured.
 * @param ptype     Protocol type to handle (see @ref protocol_type_t).
 * @param cbHandler New handler for @p ptype.
 * @return The previously registered handler for @p ptype, or NULL if none was set.
 */
pfnIsCommGenMsgHandler is_comm_register_port_msg_handler(port_handle_t port, int ptype, pfnIsCommGenMsgHandler cbHandler);

/**
 * @brief Replace the full callback table on an ISComm instance.
 * @param instance  ISComm instance to configure.
 * @param callbacks Pointer to a caller-owned callback structure copied into @p instance.
 */
void is_comm_register_callbacks(is_comm_instance_t* instance, is_comm_callbacks_t *callbacks);

/**
 * @brief Replace the full callback table on a port handle.
 * @param port      Port handle whose embedded is_comm_instance_t is configured.
 * @param callbacks Pointer to a caller-owned callback structure.
 */
void is_comm_register_port_callbacks(port_handle_t port, is_comm_callbacks_t *callbacks);

/**
 * @brief Enable parsing of a specific protocol type on an ISComm instance.
 * @param instance ISComm instance to modify.
 * @param ptype    Protocol type to enable (sets the corresponding bit in protocolMask).
 */
void is_comm_enable_protocol(is_comm_instance_t* instance, protocol_type_t ptype);

/**
 * @brief Disable parsing of a specific protocol type on an ISComm instance.
 * @param instance ISComm instance to modify.
 * @param ptype    Protocol type to disable (clears the corresponding bit in protocolMask).
 */
void is_comm_disable_protocol(is_comm_instance_t* instance, protocol_type_t ptype);

/**
 * @brief Set the complete protocol enable bitmask on an ISComm instance.
 * @param instance     ISComm instance to modify.
 * @param protocolMask Bitmask of enabled protocol types (see @ref eProtocolMask).
 */
void is_comm_set_protocol_mask(is_comm_instance_t* instance, uint32_t protocolMask);

/**
 * @brief Get the current protocol enable bitmask from an ISComm instance.
 * @param instance ISComm instance to query.
 * @return Current protocolMask value (see @ref eProtocolMask).
 */
uint32_t is_comm_get_protocol_mask(is_comm_instance_t* instance);

/**
 * @brief Parse a pre-filled byte buffer for complete packets, invoking registered callbacks.
 * @param buf      Pointer to the byte buffer to parse.
 * @param buf_size Number of valid bytes in @p buf.
 * @param comm     ISComm instance that holds parser state and callbacks.
 */
void is_comm_buffer_parse_messages(uint8_t *buf, uint32_t buf_size, is_comm_instance_t* comm);

/**
 * @brief Read available bytes from a port and parse for complete packets.
 *
 * Reads from the port's hardware buffer into the ISComm receive buffer, then
 * invokes registered callbacks for any complete packets found.
 *
 * @param port Port handle to read and parse.  The embedded is_comm_instance_t
 *             must have been initialized via is_comm_port_init() or equivalent.
 */
void is_comm_port_parse_messages(port_handle_t port);

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
  For example usage, see comManagerStepRxInstance() in com_manager.cpp.

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
 * @brief Parse the next complete packet from the ISComm buffer, applying a receive timeout.
 *
 * Returns the protocol type when a full packet is available; otherwise returns @ref _PTYPE_NONE.
 * On return, comm->rxPkt contains the parsed packet and comm->rxPkt.data.ptr points to the payload.
 *
 * @param c      ISComm instance (initialized with is_comm_init()).
 * @param timeMs Current time in milliseconds. If non-zero and > MAX_PARSER_GAP_TIME_MS since the
 *               last received byte, the parser state is reset to prevent stale partial packets.
 * @return Protocol type of the completed packet, or @ref _PTYPE_NONE (0) if no complete packet yet.
 */
protocol_type_t is_comm_parse_timeout(is_comm_instance_t* c, uint32_t timeMs);

/**
* Decode packet data - when data is available, return value will be the protocol type (see protocol_type_t) and the comm instance dataPtr will point to the start of the valid data.  For Inertial Sense binary protocol, comm instance dataHdr contains the data ID (DID), size, and offset.
* @param instance the comm instance passed to is_comm_init
* @return protocol type when complete valid data is found, otherwise _PTYPE_NONE (0) (see protocol_type_t)
* @remarks when data is available, you can cast the comm instance dataPtr into the appropriate data structure pointer (see binary messages above and data_sets.h)
  For example usage, see comManagerStepRxInstance() in com_manager.cpp.

    // Read a set of bytes (fast method)
    protocol_type_t ptype;

    // Get available size of comm buffer.  is_comm_free() modifies comm->rxBuf pointers, call it before using comm->rxBuf.tail.
    int n = is_comm_free(comm);

    // Read data directly into comm buffer
    if ((n = mySerialPortRead(comm->rxBuf.tail, n)))
    {
        // Update comm buffer tail pointer
        comm->rxBuf.tail += n;

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
 * @brief Generate InertialSense binary (ISB) packet.
 * @param buf Buffer to write to.
 * @param buf_size Available size of buffer.
 * @param comm ISComm instance
 * @param flags ISB packet flags which includes the packet type (see eISBPacketFlags).
 * @param did ISB data ID
 * @param data_size Size in bytes of the payload data.
 * @param offset Offset of the payload data into the data set structure.
 * @param data Pointer to payload data.
 * @return int Number of bytes written on success or -1 on failure
 */
int is_comm_write_to_buf(uint8_t* buf, uint32_t buf_size, is_comm_instance_t* comm, uint8_t flags, uint16_t did, uint16_t data_size, uint16_t offset, const void* data);

/**
 * @brief Encode and write an ISB packet directly to a port.
 * @param port      Port handle to write the packet to.
 * @param flags     ISB packet flags including packet type (see @ref eISBPacketFlags).
 * @param did       Data ID of the payload.
 * @param data_size Size of the payload in bytes.
 * @param offset    Byte offset into the data set structure; 0 for no offset.
 * @param data      Pointer to the payload data; may be NULL for packets with no body.
 * @return Number of bytes written on success, or -1 on failure.
 */
int is_comm_write(port_handle_t port, uint8_t flags, uint16_t did, uint16_t data_size, uint16_t offset, const void* data);

/**
 * @brief Encode and write an ISB packet to a port, using a pre-allocated packet_t.
 * @param port      Port handle to write to.
 * @param txPkt     Caller-provided packet_t used as workspace for encoding.
 * @param flags     ISB packet flags including packet type (see @ref eISBPacketFlags).
 * @param did       Data ID of the payload.
 * @param data_size Size of the payload in bytes.
 * @param offset    Byte offset into the data set structure; 0 for no offset.
 * @param data      Pointer to the payload data.
 * @return Number of bytes written on success, or -1 on failure.
 */
int is_comm_write_pkt(port_handle_t port, packet_t *txPkt, uint8_t flags, uint16_t did, uint16_t data_size, uint16_t offset, const void* data);

/**
 * Removed old data and shift unparsed data to the the buffer start if running out of space at the buffer end.  Returns number of bytes available in the bufer.
 * @param instance the comm instance passed to is_comm_init
 * @return the number of bytes available in the comm buffer 
 */
int is_comm_free(is_comm_instance_t* instance);

/**
 * @brief Encode a GET_DATA request packet into a caller-provided buffer.
 * @param buf            Destination buffer for the encoded packet.
 * @param buf_size       Capacity of @p buf in bytes.
 * @param comm           ISComm instance (provides the Tx packet counter).
 * @param did            Data ID to request (see DID_* in data_sets.h).
 * @param size           Number of bytes to request from @p offset; 0 = full structure.
 * @param offset         Byte offset into the data structure; 0 = start.
 * @param periodMultiple Broadcast period multiplier; 0 = one-shot (stop after one packet).
 * @return Number of bytes written on success, or -1 on failure.
 */
int is_comm_get_data_to_buf(uint8_t *buf, uint32_t buf_size, is_comm_instance_t* comm, uint32_t did, uint32_t size, uint32_t offset, uint32_t periodMultiple);

/**
 * @brief Encode and write a GET_DATA request packet directly to a port.
 * @param port           Port handle to write the request to.
 * @param did            Data ID to request.
 * @param size           Number of bytes to request from @p offset; 0 = full structure.
 * @param offset         Byte offset into the data structure; 0 = start.
 * @param periodMultiple Broadcast period multiplier; 0 = one-shot.
 * @return Number of bytes written on success, or -1 on failure.
 */
int is_comm_get_data(port_handle_t port, uint32_t did, uint32_t size, uint32_t offset, uint32_t periodMultiple);

/**
 * @brief Encode a binary packet to set data on the device - puts the data ready to send into the buffer passed into is_comm_init.  An acknowledge packet is sent in response to this packet.
 * @param buf Buffer to write to.
 * @param buf_size Available size of buffer.
 * @param comm the ISComm instance. 
 * @param did the data id to set on the device (see DID_* at top of this file)
 * @param size the number of bytes to set on the data structure on the device
 * @param offset the offset to start setting data at on the data structure on the device
 * @param data the actual data to change on the data structure on the device - this should have at least size bytes available
 * @return int Number of bytes written on success or -1 on failure
 * @remarks pass an offset and length of 0 to set the entire data structure, in which case data needs to have the full number of bytes available for the appropriate struct matching the dataId parameter.
 */
static inline int is_comm_set_data_to_buf(uint8_t* buf, uint32_t buf_size, is_comm_instance_t* comm, uint16_t did, uint16_t size, uint16_t offset, void* data)
{
    return is_comm_write_to_buf(buf, buf_size, comm, PKT_TYPE_SET_DATA, did, size, offset, data);    
}    

/**
 * @brief Encode and write a SET_DATA packet to a port. Device responds with an ACK.
 * @param port   Port handle to send to.
 * @param did    Data ID of the structure to set.
 * @param size   Number of bytes to send from @p data; 0 = full structure.
 * @param offset Byte offset into the data structure; 0 = start.
 * @param data   Pointer to the data to send.
 * @return Number of bytes written on success, or -1 on failure.
 */
static inline int is_comm_set_data(port_handle_t port, uint16_t did, uint16_t size, uint16_t offset, void* data)
{
    return is_comm_write(port, PKT_TYPE_SET_DATA, did, size, offset, data);
}

/**
 * @brief Encode a DATA packet into a buffer (no ACK sent by device).
 * @param buf      Destination buffer.
 * @param buf_size Capacity of @p buf.
 * @param comm     ISComm instance.
 * @param did      Data ID.
 * @param size     Payload size in bytes.
 * @param offset   Byte offset into the data structure.
 * @param data     Pointer to payload data.
 * @return Number of bytes written on success, or -1 on failure.
 */
static inline int is_comm_data_to_buf(uint8_t* buf, uint32_t buf_size, is_comm_instance_t* comm, uint16_t did, uint16_t size, uint16_t offset, void* data)
{
    return is_comm_write_to_buf(buf, buf_size, comm, PKT_TYPE_DATA, did, size, offset, data);
}

/**
 * @brief Encode and write a DATA packet to a port (no ACK sent by device).
 * @param port   Port handle to send to.
 * @param did    Data ID.
 * @param size   Payload size in bytes.
 * @param offset Byte offset into the data structure.
 * @param data   Pointer to payload data.
 * @return Number of bytes written on success, or -1 on failure.
 */
static inline int is_comm_data(port_handle_t port, uint16_t did, uint16_t size, uint16_t offset, void* data)
{
    return is_comm_write(port, PKT_TYPE_DATA, did, size, offset, data);
}

/**
 * @brief Encode and write a DATA packet using a pre-allocated packet_t.
 * @param port   Port handle to send to.
 * @param txPkt  Caller-provided packet_t used as workspace.
 * @param did    Data ID.
 * @param size   Payload size in bytes.
 * @param offset Byte offset into the data structure.
 * @param data   Pointer to payload data.
 * @return Number of bytes written on success, or -1 on failure.
 */
static inline int is_comm_data_pkt(port_handle_t port, packet_t *txPkt, uint16_t did, uint16_t size, uint16_t offset, void* data)
{
    return is_comm_write_pkt(port, txPkt, PKT_TYPE_DATA, did, size, offset, data);
}

/**
 * @brief Send a command to stop all broadcasts on all ports.
 * @param port Port handle to send the stop command to.
 * @return Number of bytes written on success, or -1 on failure.
 */
static inline int is_comm_stop_broadcasts_all_ports(port_handle_t port)
{
    return is_comm_write(port, PKT_TYPE_STOP_BROADCASTS_ALL_PORTS, 0, 0, 0, NULL);
}

/**
 * @brief Send a command to stop all broadcasts on the current port only.
 * @param port Port handle to send the stop command to.
 * @return Number of bytes written on success, or -1 on failure.
 */
static inline int is_comm_stop_broadcasts_current_port(port_handle_t port)
{
    return is_comm_write(port, PKT_TYPE_STOP_BROADCASTS_CURRENT_PORT, 0, 0, 0, NULL);
}

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
 * @param cksum_init initial value for the checksum.
 * @param data data array used for checksum.
 * @param size size of data arary.
 * @return uint16_t 
 */
uint16_t is_comm_xor16(uint16_t cksum_init, const void* data, uint32_t size);

/**
 * @brief crc_ccitt - Calculate the CRC-CCITT checksum for a given data buffer. Used for Septentrio SBF packets. 0 seed 
 * 
 * @param data 
 * @param length 
 * @return uint16_t crc_ccitt checksum
 */
uint16_t crc_ccitt(const uint8_t *data, size_t length);

#define is_comm_isb_checksum16  is_comm_fletcher16  //!< Checksum algorithm used for ISB packets (currently Fletcher-16)
// #define is_comm_isb_checksum16  is_comm_xor16

// -------------------------------------------------------------------------------------------------------------------------------
// Common packet encode / decode functions
// -------------------------------------------------------------------------------------------------------------------------------

/**
 * @brief Encode InertialSense binary (ISB) packet header.
 * @param pkt Packet storage location.
 * @param flags ISB packet flags which includes the packet type (see eISBPacketFlags).
 * @param did ISB data ID
 * @param data_size Size in bytes of the payload data.
 * @param offset Offset of the payload data into the data set structure.
 * @param data Pointer to payload data.
 */
void is_comm_encode_hdr(packet_t *pkt, uint8_t flags, uint16_t did, uint16_t data_size, uint16_t offset, const void* data);

/**
 * @brief Write a precomputed ISB packet to a port after updating its checksum.
 * @param port Port handle to write to.
 * @param pkt  Pointer to a precomputed packet_t (header and data must already be filled in).
 * @return Number of bytes written on success, or -1 on failure.
 */
int is_comm_write_isb_precomp_to_port(port_handle_t port, packet_t *pkt);

/**
 * @brief Compute a 24-bit CRC using the QUALCOMM polynomial.
 * @param buffer Data buffer to checksum.
 * @param len    Number of bytes in @p buffer.
 * @return 24-bit CRC value.
 */
unsigned int calculate24BitCRCQ(const unsigned char* buffer, unsigned int len);

/**
 * @brief Extract an arbitrary bit field from a byte buffer as a uint32_t.
 * @param buffer Source byte buffer.
 * @param pos    Bit position (0 = LSB of first byte) at which to start extraction.
 * @param len    Number of bits to extract (1 to 32).
 * @return Extracted value right-justified in a uint32_t.
 */
unsigned int getBitsAsUInt32(const unsigned char* buffer, unsigned int pos, unsigned int len);

/**
 * @brief Validate that a baud rate is supported by IS hardware.
 * @param baudRate Baud rate to check.
 * @return 0 if valid, -1 if not a recognized IS baud rate.
 */
int validateBaudRate(unsigned int baudRate);

/**
 * @brief Copy from a data structure into a p_data_t packet payload.
 *
 * Only the region described by data->hdr.size and data->hdr.offset is copied
 * from @p sptr; the copy is bounds-checked against @p maxsize.
 *
 * @param data    Destination packet data (hdr.size/offset must be set by caller).
 * @param sptr    Source data structure pointer.
 * @param maxsize Total size of the source structure for bounds checking.
 * @return 0 on success, -1 on failure (e.g., out-of-range offset/size).
 */
char copyStructPToDataP(p_data_t *data, const void *sptr, const unsigned int maxsize);

/**
 * @brief Copy from a p_data_t packet payload into a data structure.
 * @param sptr    Destination data structure pointer.
 * @param data    Source packet data.
 * @param maxsize Total size of the destination structure for bounds checking.
 * @return 0 on success, -1 on failure.
 */
char copyDataPToStructP(void *sptr, const p_data_t *data, const unsigned int maxsize);

/**
 * @brief Copy from a p_data_buf_t packet buffer into a data structure.
 * @param sptr    Destination data structure pointer.
 * @param data    Source packet buffer.
 * @param maxsize Total size of the destination structure for bounds checking.
 * @return 0 on success, -1 on failure.
 */
char copyDataBufPToStructP(void *sptr, const p_data_buf_t *data, const unsigned int maxsize);

/**
 * @brief Copy from one p_data_t payload into another.
 * @param dst     Destination packet data (hdr must describe the desired copy region).
 * @param src     Source packet data.
 * @param maxsize Maximum bytes to copy (bounds check).
 * @return 0 on success, -1 on failure.
 */
char copyDataPToDataP(p_data_t *dst, const p_data_t *src, const unsigned int maxsize);

/**
 * @brief Copy from a separate header + buffer pair into a data structure.
 * @param sptr    Destination data structure pointer.
 * @param dataHdr Header describing the data ID, size, and offset.
 * @param dataBuf Raw payload bytes corresponding to @p dataHdr.
 * @param maxsize Total size of the destination structure for bounds checking.
 * @return 0 on success, -1 on failure.
 */
char copyDataPToStructP2(void *sptr, const p_data_hdr_t *dataHdr, const uint8_t *dataBuf, const unsigned int maxsize);

/**
 * @brief Test whether a received data region overlaps with a backing data structure region.
 * @param dstOffset Byte offset of the destination region within the data structure.
 * @param dstSize   Byte size of the destination region.
 * @param src       Source p_data_t describing the received data's offset and size.
 * @return Non-zero if the regions overlap, 0 if they do not.
 */
static inline uint8_t dataOverlap(uint32_t dstOffset, uint32_t dstSize, p_data_t* src)
{
    return _MAX(dstOffset, (uint32_t)(src->hdr.offset)) < _MIN(dstOffset + dstSize, (uint32_t)(src->hdr.offset + src->hdr.size));
}

/**
 * @brief Reset the protocol parser to its initial state.
 *
 * Discards any partially accumulated packet and repositions the scan pointer.
 *
 * @param c ISComm instance to reset.
 */
static inline void is_comm_reset_parser(is_comm_instance_t* c)
{
    c->parser.state = 0;
    c->rxBuf.scanPrior = c->rxBuf.scan;
    c->rxBuf.scan = c->rxBuf.head;
    c->processPkt = NULL;
}

/**
 * @brief Copy data from is_comm_instance_t into a caller-provided structure.
 * @param sptr    Destination structure pointer.
 * @param comm    Source ISComm instance.
 * @param maxsize Total byte size of the destination structure (bounds check).
 * @return 0 on success, -1 on failure.
 */
char is_comm_copy_to_struct(void *sptr, const is_comm_instance_t *comm, const unsigned int maxsize);

/**
 * @brief Extract the most recently parsed ISB p_data_t from an is_comm_instance_t.
 * @param comm  ISComm instance containing the parsed receive packet.
 * @param data  Output p_data_t populated with the DID, offset, size, and data pointer.
 */
static inline void is_comm_to_isb_p_data(const is_comm_instance_t *comm, p_data_t *data)
{
    data->hdr.id        = comm->rxPkt.dataHdr.id;
    data->hdr.offset    = comm->rxPkt.offset;
    data->hdr.size      = comm->rxPkt.data.size;
    data->ptr           = comm->rxPkt.data.ptr;
}

/**
 * @brief Extract the ISB packet type from an is_comm_instance_t's most recent receive packet.
 * @param comm ISComm instance containing the parsed receive packet.
 * @return ISB packet type flags (see @ref eISBPacketFlags).
 */
static inline eISBPacketFlags is_comm_to_isb_pkt_type(const is_comm_instance_t *comm)
{
    return (eISBPacketFlags)(comm->rxPkt.hdr.flags&PKT_TYPE_MASK);
}

/** @brief Validate that a baud rate is in the IS-supported standard range; returns -1 if not. */
int validateBaudRate(unsigned int baudRate);

#ifdef __cplusplus
}
#endif

#endif // IS_COMM_H
