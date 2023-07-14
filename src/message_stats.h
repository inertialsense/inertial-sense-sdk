/*
MIT LICENSE

Copyright (c) 2014-2023 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#ifndef __GPS_STATS_H__
#define __GPS_STATS_H__

#include <string>


typedef struct
{
    int count;
    int timeMs;
    int prevTimeMs;
    std::string description;
} msg_stats_t;

typedef struct
{
    std::map<int, msg_stats_t> isb;
    std::map<int, msg_stats_t> nmea;
    std::map<int, msg_stats_t> ublox;
    std::map<int, msg_stats_t> rtcm3;
    msg_stats_t ack;
    msg_stats_t parseError;
} mul_msg_stats_t;


unsigned int messageStatsGetbitu(const unsigned char *buff, int pos, int len);
std::string messageDescriptionUblox(uint8_t msgClass, uint8_t msgID);
std::string messageDescriptionRtcm3(int id);
void messageStatsAppend(std::string message, mul_msg_stats_t &msgStats, unsigned int ptype, int id, int timeMs);
std::string messageStatsSummary(mul_msg_stats_t &msgStats);


#endif // __GPS_STATS_H__
