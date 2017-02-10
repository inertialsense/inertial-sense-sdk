/*
MIT LICENSE

Copyright 2014 Inertial Sense, LLC - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#ifndef IS_UTILITIES_H
#define IS_UTILITIES_H

#include <stdlib.h>
#include <inttypes.h>

int current_timeSec();
int current_timeMs();
int current_weekMs();
uint64_t current_weekUs();

uint64_t timerUsStart();
uint64_t timerUsEnd(uint64_t start);
uint64_t timerRawStart();
uint64_t timerRawEnd(uint64_t start);

#ifndef _MAX 
#define _MAX(a,b) (((a) > (b)) ? (a) : (b))
#endif

#ifndef _MIN
#define _MIN(a,b) (((a) < (b)) ? (a) : (b))
#endif

#ifndef C_PI_F
#define C_PI_F          3.14159265358979323846264338327950288419716939937511f
#endif

#ifndef C_TWOPI_F
#define C_TWOPI_F       6.283185307179586476925286766559f
#endif

#if defined(_WIN32) 
//   =========  Windows  =========

#include <windows.h>

void usleep(__int64 usec);

#ifndef STRINGIFY
#define STRINGIFY(x) #x
#endif

#define DEFAULT_COM_PORT "COM4"

#ifndef SLEEP_MS
#define SLEEP_MS(milliseconds) Sleep(milliseconds);
#endif

#ifndef SLEEP_US
#define SLEEP_US(timeUs) usleep(timeUs);
#endif

#ifndef SSCANF
#define SSCANF(src, fmt, ...) sscanf_s(src, fmt, __VA_ARGS__);
#endif

#ifndef SNPRINTF
#define SNPRINTF(a, b, c, ...) _snprintf_s(a, b, b, c, __VA_ARGS__)
#endif

#ifndef STRNCPY
#define STRNCPY(a, b, c) strncpy_s(a, c, b, c);
#endif

#else
//   ==========  Linux  ==========

#include <unistd.h>
#include <sys/time.h>
#include <stdarg.h>

// int SSCANF(const char * s, const char * format, ...)
// {
// 	int n;
// 	va_list ap;
// 
// 	va_start(ap, format);
// 	n = sscanf(s, format, ap);
// 	va_end(ap);
// 
// 	return n;
// }

#define DEFAULT_COM_PORT "/dev/ttyUSB0"

#ifndef SLEEP_MS
#define SLEEP_MS(timeMs) usleep(timeMs * 1000);
#endif

#ifndef SLEEP_US
#define SLEEP_US(timeUs) usleep(timeUs);
#endif

#ifndef SSCANF
#define SSCANF sscanf
#endif

#ifndef SNPRINTF
#define SNPRINTF snprintf
#endif

#ifndef STRNCPY
#define STRNCPY(a, b, c) strncpy((char*)a, (char*)b, c)
#endif

#endif

int bootloadUploadProgress(const void* port, float percent);
int bootloadVerifyProgress(const void* port, float percent);

float step_sinwave(float *sig_gen, float freqHz, float amplitude, float periodSec);

#endif // IS_UTILITIES_H
