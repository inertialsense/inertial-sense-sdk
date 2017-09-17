/*
MIT LICENSE

Copyright 2014 Inertial Sense, LLC - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#ifndef __GPS_PARSER__H__
#define __GPS_PARSER__H__

#include "data_sets.h"

#if defined(RTK_EMBEDDED)

#include "rtklib.h"

#endif

class cGpsParser;

/*!
* Gps parser types
*/
typedef enum
{
	/*!
	* Inertial sense format
	*/
	GpsParserTypeInertialSense = 1,

	/*!
	* Ublox format
	*/
	GpsParserTypeUblox = 2,

	/*!
	* Rtcm3 format
	*/
	GpsParserTypeRtcm3 = 3
} eGpsParserType;

/*!
* Gps parser interface. A gps parser handles parsing messages common to all types of gps data formats.
*/
class iGpsParserDelegate
{
public:
	/*!
	* Executes when a valid packet is received. Data is not valid after this callback completes.
	* @param parser the ublox reader
	* @param data the packet, including all headers and checksums
	* @param dataLength the length of the data
	* @return true to parse the packet, false to discard it
	*/
	virtual bool OnPacketReceived(const cGpsParser* parser, const uint8_t* data, uint32_t dataLength)
	{
		(void)parser;
		(void)data;
		(void)dataLength;
		return true;
	}

	/*!
	* Executes when observation data is received. Data is not valid after this callback completes.
	* @param parser the parser
	* @param obs the observation data
	*/
	virtual void OnObservationReceived(const cGpsParser* parser, const obs_t* obs)
	{
		(void)parser;
		(void)obs;
	}

	/*!
	* Executes when GPS ephemeris data is received. Data is not valid after this callback completes.
	* @param parser the parser
	* @param obs the observation data
	* @param prn the prn identifier
	*/
	virtual void OnGpsEphemerisReceived(const cGpsParser* parser, const eph_t* eph, int prn)
	{
		(void)parser;
		(void)eph;
		(void)prn;
	}

	/*!
	* Executes when Glonass ephemeris data is received. Data is not valid after this callback completes.
	* @param parser the parser
	* @param obs the observation data
	* @param prn the prn identifier
	*/
	virtual void OnGlonassEphemerisReceived(const cGpsParser* parser, const geph_t* geph, int prn)
	{
		(void)parser;
		(void)geph;
		(void)prn;
	}

	/*!
	* Executes when sbs message received
	* @param parser the parser
	* @param sbs the sbs message
	*/
	virtual void OnSbsReceived(const cGpsParser* parser, const sbsmsg_t* sbs)
	{
		(void)parser;
		(void)sbs;
	}

	/*!
	* Executes when station info is received. Data is not valid after this callback completes.
	* @param parser the parser
	* @param sta the station info
	*/
	virtual void OnStationReceived(const cGpsParser* parser, sta_t* sta)
	{
		(void)parser;
		(void)sta;
	}
};

class cGpsParser
{
public:
	/*!
	* Create a GPS parser
	* @param type the type of parser
	* @param delegate the delegate for callbacks
	* @return the parser or NULL if error
	*/
	static cGpsParser* CreateParser(eGpsParserType type, iGpsParserDelegate* delegate);

	virtual ~cGpsParser() {}

	/*!
	* Add data to the parser
	* @param data the data to write
	* @param dataLength the number of bytes in data
	*/
	void Write(const uint8_t* data, int dataLength)
	{
		for (const uint8_t* ptr = data, *ptrEnd = data + dataLength; ptr != ptrEnd; ptr++)
		{
			WriteByte(*ptr);
		}
	}

	/*!
	* Add a byte to the parser
	* @param b the byte to write
	*/
	virtual void WriteByte(uint8_t b) = 0;

	/*!
	* Get the delegate for the parser
	* @return the delegate
	*/
	iGpsParserDelegate* GetDelegate() { return m_delegate; }

	/*!
	* Set the receiver index
	* @param index the receiver index, RECEIVER_INDEX_ROVER or RECEIVER_INDEX_BASE_STATION
	*/
	void SetReceiverIndex(int32_t index) { m_receiverIndex = index; }

	/*!
	* Get the receiver index
	* @return the receiver index, RECEIVER_INDEX_ROVER or RECEIVER_INDEX_BASE_STATION
	*/
	int32_t GetReceiverIndex() const { return m_receiverIndex; }

	/*!
	* Parse a full formed message. This is normally called during the course of Write and WriteByte, but you can pass in a fully formed message as well
	* @param data the full formed message data
	* @param dataLength the number of bytes in data
	*/
    virtual void ParseMessage(const uint8_t* data, int dataLength) { (void)data; (void)dataLength; }

protected:
	/*!
	* Constructor
	* @param delegate the delegate to receive callbacks on packets, must not be NULL
	*/
	cGpsParser(iGpsParserDelegate* delegate);

private:
	iGpsParserDelegate* m_delegate;
	int32_t m_receiverIndex;

};

#endif

