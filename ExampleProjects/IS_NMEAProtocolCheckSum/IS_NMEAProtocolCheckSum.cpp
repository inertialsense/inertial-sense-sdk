/*
MIT LICENSE

Copyright (c) 2014-2025 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include <stdio.h>
#include <vector>
#include <string.h>
#include <algorithm>
#include <stdlib.h>
#include <stddef.h>
#include <fstream>
#include <iostream>
#include <sstream>
#include <mutex>

using namespace std;


unsigned int ASCIIComputeChecksum(uint8_t* str, int size)
{
	unsigned int checksum = 0;

	uint8_t* end = str + size;
	for (uint8_t* ptr = str; ptr < end; ptr++)
	{
		checksum ^= *ptr;
	}

	return checksum;
}

int main(int argc, char* argv[])
{
	string fileName = "../../../NMEAReads/Serial38400.txt";
	string line = "";
	char inChecksumTmpStr[3];
	int lineNum = 1;
	int failCnt = 0;

	unsigned int calcChecksum;
	unsigned int inChecksum;

	// Null terminate temp string
	inChecksumTmpStr[2] = 0;

	// check if we are using a static COM port
	if (argc == 2)
	{
		fileName = argv[1];
	}

	// print COM port to console
	printf("File name: %s\r\n", fileName.c_str());

	// create ifstream
	std::ifstream srcFile(fileName);

	// Go through file line by line
	while (std::getline(srcFile,line))
	{
		// calculate checksum
		calcChecksum = ASCIIComputeChecksum((uint8_t*)(line.c_str() + 1), line.size()-4);

		// copy expected checksum to temp
		memcpy(inChecksumTmpStr, line.c_str() + line.size() - 2, 2);

		// Extract checksum
		inChecksum = strtol(inChecksumTmpStr, 0, 16);

		// Check for chekcsum missmatch
		if (inChecksum != calcChecksum)
		{
			//increment fail count
			failCnt++;

			// print current line
			printf("Mismatch %d:\r\nLine: %d Expected: %d Calculated: %d\r\nString:\r\n%s\r\n", failCnt, lineNum, inChecksum, calcChecksum, line.c_str());
		}

		lineNum++;
	}

	printf("\r\nRun finished with %d lines run with %d checksum mismatches!\r\n\r\n", lineNum, failCnt);

	return 0;
}