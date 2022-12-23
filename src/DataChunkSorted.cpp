/*
MIT LICENSE

Copyright (c) 2014-2023 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include <stdio.h>
#include <string.h>
#include "DataChunkSorted.h"

using namespace std;


cSortedDataChunk::cSortedDataChunk(const char* name) : cDataChunk()
{
	memset(&m_subHdr, 0, sizeof(sChunkSubHeader));
	SetName(name);
}


void cSortedDataChunk::Clear()
{
	cDataChunk::Clear();
	m_subHdr.dCount = 0;
}


// This function reads the next sorted chunk matching the specified DID from all of the files.  The file pointers are advanced 
// if the current chunk dataSerNum is older than the current m_dataSerNum.  Files are closed if the last chuch dataSerNum 
// is older than the current m_dataSerNum.
// Returns number of bytes read, or -1 for error
int32_t cSortedDataChunk::ReadFromFiles(vector<cISLogFileBase*>& pFiles, uint32_t id, uint32_t dataSerNum)
{
	if (pFiles.size() == 0 || pFiles.front() == NULLPTR)
	{
		return -1;
	}

	// reset state, prepare to read from file
	Clear();

	int32_t nBytes = -1;
	long int restorePos;

	// Search through all files for chunk with matching DID
	for (unsigned int i = 0; i < pFiles.size(); )
	{
		cISLogFileBase* pFile = pFiles[i];

		// Set backup file pointer
		restorePos = pFile->tell();

		// Search through single file for chunk with matching DID
		bool allowFileTrim = true;	// This flag indicates there is/are new chunk(s) that should not be trimmed from the restorePos file pointer position. 
		while(1)
		{	
			nBytes = ReadFromFile(pFile);

			if (nBytes <= 0)
			{	// Match not found in file
				break;
			}

			p_cnk_data_t* cnkData = (p_cnk_data_t*)GetDataPtr();
			if (cnkData == NULLPTR)
			{	// No more data.  Match not found
				break;
			}

			if (m_subHdr.dHdr.id == id && GetDataSerNum() >= dataSerNum)
			{	// Found matching chunk
				if (allowFileTrim)
				{	
					pFile = TrimFile(i, pFiles, restorePos);
				}
				break;
			}
			else
			{	// Chunk not found
				nBytes = -1;
			}

			if (allowFileTrim)
			{
				uint32_t readDataSerNum = GetDataSerNum();
				if (readDataSerNum < dataSerNum || readDataSerNum == UINT_MAX)
				{	// Move file pointer past old/invalid chunks
					pFile = TrimFile(i, pFiles, restorePos);
				}
				else
				{
					allowFileTrim = false;
				}
			}
		}

		if (pFile)
		{ 	// Restore file pointer if file is still opened
			pFile->seek(restorePos);
		}

		if (nBytes > 0)
		{	// Found chunk
			return nBytes;
		}

		// No matching chunks in file
		i++;
	}

	return -1;
}


cISLogFileBase* cSortedDataChunk::TrimFile(unsigned int i, vector<cISLogFileBase*>& pFiles, long int &restorePos)
{
	cISLogFileBase* pFile = pFiles[i];

	// Move file pointer past old chunks
	//pFile->getpos(&restorePos);
	restorePos = pFile->tell();

	if (pFile->eof())
	{	// No more data in file
		pFile->close();

		// Remove file at index from vector
		pFiles.erase(pFiles.begin() + i);
	}

	if (i < pFiles.size())
	{	// More files available
		return pFiles[i];
	}
	else
	{	// No more files available
		return NULLPTR;
	}
}


int32_t cSortedDataChunk::WriteAdditionalChunkHeader(cISLogFileBase* pFile)
{
	// Write sub header to file
	return static_cast<int32_t>(pFile->write(&m_subHdr, sizeof(m_subHdr)));
}


int32_t cSortedDataChunk::ReadAdditionalChunkHeader(cISLogFileBase* pFile)
{
	// Read chunk header
	return static_cast<int32_t>(pFile->read(&m_subHdr, sizeof(m_subHdr)));
}


int32_t cSortedDataChunk::GetHeaderSize()
{
    return int32_t((sizeof(sChunkHeader) + sizeof(sChunkSubHeader)));
}


uint32_t cSortedDataChunk::GetDataSerNum()
{
	p_cnk_data_t* cnkData = (p_cnk_data_t*)GetDataPtr();
	if (cnkData == NULLPTR)
	{
		return UINT_MAX;
	}
	else
	{
		return cnkData->dataSerNum;
	}
}
