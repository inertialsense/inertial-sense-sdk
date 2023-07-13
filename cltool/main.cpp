/*
MIT LICENSE

Copyright (c) 2014-2023 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

// cltool_main.cpp in the SDK source contains the example code
#include "../src/cltool.h"

int main(int argc, char* argv[])
{
#if 0
	return cltool_main(argc, argv);
#else
	int margc = 5;
	char* margv[10];
	margv[0] = argv[0];
	margv[1] = "-c";
	margv[2] = "com8";
	//margv[3] = "-did";
	//margv[4] = "4";
	margv[3] = "-uf";
	margv[4] = "C:\\_IS\\imx\\cpp\\hdw-src\\IMX-5\\Release\\IS_IMX-5.hex";
	return cltool_main(margc, margv);
#endif
}