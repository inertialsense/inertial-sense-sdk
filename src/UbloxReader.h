#ifndef __UBLOX_READER_H_
#define __UBLOX_READER_H_

#include <inttypes.h>
#include <stdlib.h>
#include <string.h>
#include <string>
#include <vector>
#include "data_sets.h"

#if defined(RTK_EMBEDDED)

#include "rtklib.h"

#endif

using namespace std;

class cUbloxReader;

class iUbloxReaderDelegate
{
protected:
	/*!
	* Executes when a valid ublox packet is received. Data is not valid after this callback completes.
	* @param reader the ublox reader
	* @param data the ublox packet, including header and checksum
	* @param dataLength the length of the ublox data
	* @return true to parse the packet, false to discard it
	*/
	virtual bool OnPacketReceived(const cUbloxReader* reader, const uint8_t* data, uint32_t dataLength)
	{
		(void)reader;
		(void)data;
		(void)dataLength;
		return true;
	}

#if defined(RTK_EMBEDDED)

	/*!
	* Executes when observation data is received. Data is not valid after this callback completes.
	* @param reader the ublox reader
	* @param obs the observation data
	*/
	virtual void OnObservationReceived(const cUbloxReader* reader, const obs_t* obs)
	{
		(void)reader;
		(void)obs;
	}

	/*!
	* Executes when GPS ephemeris data is received. Data is not valid after this callback completes.
	* @param reader the ublox reader
	* @param obs the observation data
	* @param prn the prn identifier
	*/
	virtual void OnGpsEphemerisReceived(const cUbloxReader* reader, const eph_t* eph, int prn)
	{
		(void)reader;
		(void)eph;
		(void)prn;
	}

	/*!
	* Executes when Glonass ephemeris data is received. Data is not valid after this callback completes.
	* @param reader the ublox reader
	* @param obs the observation data
	* @param prn the prn identifier
	*/
	virtual void OnGlonassEphemerisReceived(const cUbloxReader* reader, const geph_t* geph, int prn)
	{
		(void)reader;
		(void)geph;
		(void)prn;
	}

	/*!
	* Executes when sbs message received
	* @param reader the ublox reader
	* @param sbs the sbs message
	*/
	virtual void OnSbsReceived(const cUbloxReader* reader, const sbsmsg_t* sbs)
	{
		(void)reader;
		(void)sbs;
	}

	/*!
	* Executes when station info is received. Data is not valid after this callback completes.
	* @param reader the ublox reader
	* @param sta the station info
	*/
	virtual void OnStationReceived(const cUbloxReader* reader, sta_t* sta)
	{
		(void)reader;
		(void)sta;
	}

#endif

	friend class cUbloxReader;
};

class cUbloxReader
{
public:
	/*!
	* Constructor
	* @param delegate the delegate to receive callbacks on ublox packets, must not be NULL
	*/
	cUbloxReader(iUbloxReaderDelegate* delegate);
	virtual ~cUbloxReader();

	/*!
	* Add data to the reader
	* @param data the data to write
	* @param dataLength the number of bytes in data
	*/
	void Write(const uint8_t* data, int dataLength);

	/*!
	* Add a byte to the reader
	* @param b the byte to write
	*/
	void WriteByte(uint8_t b);

#if defined(RTK_EMBEDDED)

	/*!
	* Set the receiver index
	* @param index the receiver index, RECEIVER_INDEX_ROVER or RECEIVER_INDEX_BASE_STATION
	*/
	void SetReceiverIndex(uint8_t index) { m_receiverIndex = index; }

	/*!
	* Get the receiver index
	* @return the receiver index, RECEIVER_INDEX_ROVER or RECEIVER_INDEX_BASE_STATION
	*/
	uint8_t GetReceiverIndex() const { return m_receiverIndex; }

#endif

private:
	void Reset();
	void ParseMessage();
	void CalculateChecksum(const uint8_t* data, int dataLength, uint8_t& checksum1, uint8_t& checksum2);

	iUbloxReaderDelegate* m_delegate;
	uint32_t m_bytesRemaining; // bytes left in packet
	uint8_t m_messageId;
	uint8_t m_classId;

#if defined(RTK_EMBEDDED)

	uint8_t m_receiverIndex;
	raw_t m_raw;

#else

	struct
	{
		uint8_t buff[2048];
		int len;
		int nbyte;
	} m_raw;

#endif

};

#endif
