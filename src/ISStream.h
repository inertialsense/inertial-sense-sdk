/*
MIT LICENSE

Copyright 2014 Inertial Sense, LLC - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#ifndef __ISSTREAM_H__
#define __ISSTREAM_H__

#include <stdio.h>
#include <iostream>
#include <string>
#include <inttypes.h>

#include "ISConstants.h"

/*!
* Interface for closing a stream
*/
class cISStreamClose
{
public:
	/*!
	* Close the stream
	* @return 0 if success, otherwise error code
	*/
	virtual int Close() PURE_VIRTUAL;
};

/*!
* Interface to read data from a stream
*/
class cISStreamReader : public cISStreamClose
{
public:
	/*!
	* Read n bytes from the stream
	* @param buffer the buffer to read into
	* @param count the max count of bytes to read
	* @return the number of bytes read, less than 0 if error
	*/
	virtual int Read(uint8_t* buffer, int count) PURE_VIRTUAL;
};

/*!
* Interface to write data to a stream
*/
class cISStreamWriter : public cISStreamClose
{
public:
	/*!
	* Write n bytes from the stream to buffer
	* @param buffer the buffer to write into
	* @param count the max count of bytes to write
	* @return the number of bytes written, less than 0 if error
	*/
	virtual int Write(const uint8_t* buffer, int count) PURE_VIRTUAL;
};

/*!
* Interface to read and write data using stream interfaces
*/
class cISStream : public cISStreamReader, public cISStreamWriter
{
private:
	cISStreamReader* m_reader;
	cISStreamWriter* m_writer;
	bool m_ownsReader;
	bool m_ownsWriter;
	bool m_canRead;
	bool m_canWrite;

protected:
	/*!
	* Constructor for derived classes that have custom read / write logic
	*/
	cISStream() {}

public:
	/*!
	* Constructor
	* @param reader the reader to read from or NULL for no reading
	* @param writer the writer to write from or NULL for no writing
	* @param ownsReader whether this instance owns (and deletes on destruction) reader
	* @param ownsWriter whether this instance owns (and deletes on destruction) writer
	*/
	cISStream(cISStreamReader* reader, cISStreamWriter* writer, bool ownsReader = true, bool ownsWriter = true);

	/*!
	* Destructor
	* If this instance owns the reader and/or writer streams, they are deleted
	*/
	virtual ~cISStream();

	/*!
	* Read n bytes from the stream
	* @param buffer the buffer to read into
	* @param count the max count of bytes to read
	* @return the number of bytes read, -1 if error, 0 bytes if can't read
	*/
	int Read(uint8_t* buffer, int count) OVERRIDE { return m_reader->Read(buffer, count); }

	/*!
	* Write n bytes from the stream to buffer
	* @param buffer the buffer to write into
	* @param count the max count of bytes to write
	* @return the number of bytes written, -1 if error, 0 bytes if can't write
	*/
	int Write(const uint8_t* buffer, int count) OVERRIDE { return m_writer->Write(buffer, count); }

	/*!
	* Close the stream
	* @return 0 if success, otherwise an error code
	*/
	int Close();

	/*!
	* Gets whether this stream can read
	* @return true if can read, false if not
	*/
	bool CanRead() { return m_canRead; }

	/*!
	* Gets whether this stream can write
	* @return true if can write, false if not
	*/
	bool CanWrite() { return m_canWrite; }
};

#endif // __BASE_STREAM_H__
