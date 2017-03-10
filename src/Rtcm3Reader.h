#ifndef __RTCM3_READER_H_
#define __RTCM3_READER_H_

#include <inttypes.h>

#if defined(RTK_EMBEDDED)

#include "rtklib.h"

#endif

using namespace std;

class cRtcm3Reader;

class iRtcm3ReaderDelegate
{
protected:
	/*!
	* Executes when a valid rtcm3 packet is received. Data is not valid after this callback completes.
	* @param reader the rtcm3 reader
	* @param data the rtcm3 data
	* @param dataLength the length of the rtcm3 data
	* @return true to parse the packet, false to discard it
	*/
    virtual bool OnPacketReceived(const cRtcm3Reader* reader, const uint8_t* data, uint32_t dataLength)
    {
        (void)reader;
        (void)data;
        (void)dataLength;
        return true;
    }

#if defined(RTK_EMBEDDED)

	/*!
	* Executes when observation data is received. Data is not valid after this callback completes.
	* @param reader the rtcm3 reader
	* @param obs the observation data
	*/
    virtual void OnObservationReceived(const cRtcm3Reader* reader, const obs_t* obs)
    {
        (void)reader;
        (void)obs;
    }

	/*!
	* Executes when GPS ephemeris data is received. Data is not valid after this callback completes.
	* @param reader the rtcm3 reader
	* @param eph the ephemeris data
	* @param prn the prn identifier
	*/
    virtual void OnGpsEphemerisReceived(const cRtcm3Reader* reader, const eph_t* eph, int prn)
    {
        (void)reader;
        (void)eph;
		(void)prn;
    }

	/*!
	* Executes when Glonass ephemeris data is received. Data is not valid after this callback completes.
	* @param reader the rtcm3 reader
	* @param geph the ephemeris data
	* @param prn the prn identifier
	*/
    virtual void OnGlonassEphemerisReceived(const cRtcm3Reader* reader, const geph_t* geph, int prn)
    {
        (void)reader;
        (void)geph;
		(void)prn;
    }

	/*!
	* Executes when station info is received. Data is not valid after this callback completes.
	* @param reader the rtcm3 reader
	* @param sta the station info
	*/
    virtual void OnStationReceived(const cRtcm3Reader* reader, const sta_t* sta)
    {
        (void)reader;
        (void)sta;
    }

#endif

	friend class cRtcm3Reader;
};

class cRtcm3Reader
{
public:
	/*!
	* Constructor
	* @param delegate the delegate to receive callbacks on rtcm3 packets, must not be NULL
	*/
	cRtcm3Reader(iRtcm3ReaderDelegate* delegate);
	virtual ~cRtcm3Reader();

	/*!
	* Add data to the reader
	* @param data the data to write
	* @param dataLength the number of bytes in data
	*/
	void Write(const uint8_t* data, int dataLength);

	/*!
	* Add a byte to the reader
	*/
	void WriteByte(uint8_t b);

private:
	void ParseMessage();

	iRtcm3ReaderDelegate* m_delegate;
	uint32_t m_bytesRemaining; // bytes left in packet

#if defined(RTK_EMBEDDED)

	rtcm_t m_rtcm; // rtcm parse state

#else

	struct
	{
		unsigned char buff[1200];
		int len;
	} m_rtcm;

#endif

};

#endif

