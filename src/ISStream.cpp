/*
MIT LICENSE

Copyright 2014 Inertial Sense, LLC - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include "ISStream.h"

class cNullStreamReader : public cISStreamReader
{
	int Read(uint8_t* buffer, int count) OVERRIDE
	{
        (void)buffer;
        (void)count;
		return 0;
	}

	int Close() { return 0; }
};

class cNullStreamWriter : public cISStreamWriter
{
	int Write(const uint8_t* buffer, int count) OVERRIDE
	{
        (void)buffer;
        (void)count;
		return 0;
	}

	int Close() { return 0; }
};

cISStream::cISStream(cISStreamReader* reader, cISStreamWriter* writer, bool ownsReader, bool ownsWriter)
{
	m_reader = reader;
	m_writer = writer;
	m_ownsReader = ownsReader;
	m_ownsWriter = ownsWriter;
	if (m_reader == NULL)
	{
		m_reader = new cNullStreamReader();
		m_ownsReader = true;
		m_canRead = false;
	}
	if (m_writer == NULL)
	{
		m_writer = new cNullStreamWriter();
		m_ownsWriter = true;
		m_canWrite = false;
	}
}

cISStream::~cISStream()
{
	Close();
	if (m_ownsReader)
	{
		delete m_reader;
	}
	if (m_ownsWriter)
	{
		delete m_writer;
	}
}

int cISStream::Close()
{
	int returnCode = 0;
	if (m_reader != NULL)
	{
		returnCode |= m_reader->Close();
	}
	if (m_writer != NULL)
	{
		returnCode |= m_writer->Close();
	}
	return returnCode;
}

