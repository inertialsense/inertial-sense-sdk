# Data Sets

## Comparison between old and new protocol:

- New protocol allows for packet routing to testbeds with multiple devices without further encapsulation.
- Old protocol isn't ideal for I2C communication. New protocol has a separate header with an independent checksum. I2C can read header to learn how long the payload is before setting up a payload read of that size.
- New protocol shortens communications overhead to 15 bytes per packet, from 20 bytes. No "escape bytes" are defined, further reducing the overhead.
- New protocol uses a different preamble byte (0xEF), so it is not compatible with the old protocol (0xFF). The SDK can maintain a compatibility layer that will work for most users, while adding support for the new features.
- All data in new protocol is transmitted little-endian (LE) over the communication link. The SDK may implement LE/BE swapping on the host, but the device only communicates using LE.
- New protocol uses a CRC-8 checksum for the header and a CRC-32 checksum header for the payload. The STM32 MCUs we use provide hardware acceleration for computing both checksums.

## Packet Format

**Old packet header:**

```C
/**
 * Preamble (0xFF)  | Reserved (0b000)  | Packet ID (ePacketIDs) 	| Packet counter    | Flags (ePktHdrFlags)
 * 8 bits           | 3 bits            | 5 bits    				| 8 bits            | 8 bits
 * 
 */

// Packet IDs	
typedef uint32_t ePacketIDs;

#define PID_INVALID                         (ePacketIDs)0   /** Invalid packet id */
#define PID_ACK                             (ePacketIDs)1   /** (ACK) received valid packet */
#define PID_NACK                            (ePacketIDs)2   /** (NACK) received invalid packet */
#define PID_GET_DATA                        (ePacketIDs)3   /** Request for data to be broadcast, response is PID_DATA. See data structures for list of possible broadcast data. */
#define PID_DATA                            (ePacketIDs)4   /** Data sent in response to PID_GET_DATA (no PID_ACK is sent) */
#define PID_SET_DATA                        (ePacketIDs)5   /** Data sent, such as configuration options.  PID_ACK is sent in response. */
#define PID_STOP_BROADCASTS_ALL_PORTS       (ePacketIDs)6   /** Stop all data broadcasts on all ports. Responds with an ACK */
#define PID_STOP_DID_BROADCAST              (ePacketIDs)7   /** Stop a specific broadcast */
#define PID_STOP_BROADCASTS_CURRENT_PORT    (ePacketIDs)8   /** Stop all data broadcasts on current port. Responds with an ACK */
#define PID_COUNT                           (ePacketIDs)9   /** The number of packet identifiers, keep this at the end! */
#define PID_MAX_COUNT                       (ePacketIDs)32  /** The maximum count of packet identifiers, 0x1F (PACKET_INFO_ID_MASK) */

enum ePktHdrFlags
{
	// bit set for little endian, bit cleared for big endian
	CM_PKT_FLAGS_LITTLE_ENDIAN = 0x01,
	CM_PKT_FLAGS_ENDIANNESS_MASK = 0x01,

	// has any valid packet been received
	CM_PKT_FLAGS_RX_VALID_DATA = 0x02,

	// multi-packet data set
	CM_PKT_FLAGS_MORE_DATA_AVAILABLE = 0x04,

	// Allow for arbitrary length in bytes of data, not necessarily multiple of 4. Don't auto-swap bytes for endianness
	CM_PKT_FLAGS_RAW_DATA_NO_SWAP = 0x08,

	// Checksum is the new 24 bit algorithm instead of the old 16 bit algorithm
	CM_PKT_FLAGS_CHECKSUM_24_BIT = 0x10
};

```

**New packet header:**

```C
/**
 * NEW PACKET HEADER FORMAT - 11 BYTES
 * 
 * !!! ALL DATA IS LITTLE ENDIAN OVER THE COMMUNICATIONS LINK !!!
 * 
 * The first four bytes tell us about the packet type and properties:
 * 
 * Preamble (0xEF)  | Reserved (0b000)  | Packet ID (ePacketIDs) 	| Packet counter    | Flags (ePktHdrFlagsV2)
 * 8 bits           | 3 bits            | 5 bits   		 			| 8 bits            | 8 bits
 * 
 * ---------------
 * 
 * NOTES ON PACKET COUNTER
 * 
 * The packet counter is per-device. If a host communicates with multiple devices, the host must treat these
 * as separately incrementing counters.
 * 
 */

enum ePktHdrFlagsV2		// V1 flags are re-used for new functions
{
    // Testbed location data follows if set
    CM_PKT_FLAGS_TESTBED_LOCATION = 0x01,
};

/**
 * The next 7 bytes tell us about the packet data:
 * 
 * DID (0-4095) | Reserved (0x0)    | Data size (0-4095)    | Data offset (0-4095)
 * 12 bits      | 4 bits            | 12 bits               | 12 bits               
 * 
 * 
 * ---------------
 * 
 * TESTBED LOCATION BYTES
 * 
 * If `CM_PKT_FLAGS_TESTBED_LOCATION` is set, the next two header bytes identify which testbed and slot 
 * the data is addressed to or originates from. These bytes count towards the data size bytes 
 * in the header.
 * 
 * | Testbed SN (0-4095)  | Testbed slot (0-15)
 * | 12 bits              | 4 bits
 * 
 * If this bit is not set, these bytes are reserved and should be set to 0 when sending data to the
 * device. These bytes may not be omitted, since the header must be 11 bytes when read over I2C.
 * 
 * | Reserved
 * | 16 bits
 * 
 * ---------------
 * 
 * CHECKSUM BYTE
 * 
 * The checksum byte is calculated by performing the CRC-8 algorithm on each byte of the header, inclusive 
 * of the preamble and going through the flags byte. This is an important feature for I2C, since 
 * the header is read in a separate read transaction from the rest of the packet.
 * 
 * | Checksum (CRC-8)
 * | 8 bits
 * 
 */

```

**Old packet payload:**

```C
/**
 * Data         | Checksum (custom)	| Stop byte (0xFE)
 * n*8 bits     | 24 bits			| 8 bits
 * 
 */

```

**New packet payload:**

```C
/**
 * !!! NO SPECIAL BYTES (i.e. bytes following 0xFD in old protocol) !!!
 * 
 * Data         | Checksum (CRC-32)
 * n*8 bits     | 32 bits
 * 
 * ---------------
 * 
 * CHECKSUM NOTES
 * 
 * The checksum is computed on the entire payload, but not the header bytes (the header 
 * has a separate, more simple checksum). The checksum is CRC-32 instead of the custom 
 * 24-bit checksum of the previous protocol because the CRC-32 is supported in hardware 
 * on the STM32L4 and STM32U5 processors, reducing processor overhead.
 * 
 */

```

## I2C Support

With the new packet format, I2C can be supported easily. When the data ready pin goes high, 9 bytes are read out. If the preamble and offset match, the packet can be read in with a second read transaction. Because the DID, packet type, offset, and length are known beforehand, DMA may be used to transfer the incoming I2C data directly to where it is stored, rather than to an intermediate buffer. If this technique is used with SPI or UART, interrupt-based processing of the header bytes is required. A DMA transfer can be started once the header is parsed. 

