/*
MIT LICENSE

Copyright (c) 2014-2023 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include <map>

#include "ISComm.h"
#include "ISDataMappings.h"
#include "message_stats.h"

using namespace std;


string messageDescriptionUblox(uint8_t msgClass, uint8_t msgID)
{
	string s;
	switch (msgClass)
	{
	case 0x01:	s.append("UBX-NAV");
		switch (msgID)
		{
		case 0x01:	s.append("-POSECEF");	break;
		case 0x02:	s.append("-POSLLH");	break;
		}
		break;

	case 0x02:	s.append("UBX-RXM");
		switch (msgID)
		{
		case 0x13:	s.append("-SFRBX");		break;
		case 0x15:	s.append("-RAWX");		break;
		case 0x32:	s.append("-RTCM");		break;
		}
		break;
	case 0x06:	s.append("UBX-CFG");		break;
	case 0x0D:	s.append("UBX-TIM");		break;
	}

	return s;
}

string messageDescriptionRtcm3(int id)
{
	// RTCM3 descriptions: https://www.use-snip.com/kb/knowledge-base/rtcm-3-message-list/
	switch (id)
	{
	case 1002:	return string("Extended L1-Only GPS RTK Observables");
	case 1004:	return string("Extended L1&L2 GPS RTK Observables");
	case 1005:	return string("Stationary RTK reference station ARP");
	case 1007:	return string("Antenna Descriptor");
	case 1010:	return string("Extended L1-Only GLONASS RTK Observables");
	case 1012:	return string("Extended L1&L2 GLONASS RTK Observables");
	case 1019:	return string("GPS ephemerides");
	case 1020:	return string("GLONASS ephemerides");
	case 1023:	return string("Residuals, Ellipsoidal Grid Representation");
	case 1029:	return string("Text");
	case 1030:	return string("GPS Network RTK Residual");
	case 1031:	return string("GLONASS Network RTK Residual");
	case 1032:	return string("Physical Reference Station Position");
	case 1033:	return string("Receiver and Antenna Descriptors");
	case 1045:	return string("Galileo ephemerides");
	case 1074:	return string("GPS MSM4");
	case 1075:	return string("GPS MSM5");
	case 1077:	return string("GPS MSM7");
	case 1084:	return string("GLONASS MSM4");
	case 1085:	return string("GLONASS MSM5");
	case 1087:	return string("GLONASS MSM7");
	case 1094:	return string("Galileo MSM4");
	case 1095:	return string("Galileo MSM5");
	case 1097:	return string("Galileo MSM7");
	case 1124:	return string("BeiDou MSM4");
	case 1127:	return string("BeiDou MSM7");
	case 1230:	return string("GLONASS code-phase biases");
	case 4094:	return string("Assigned to : Trimble Navigation Ltd.");
	}

	return "";
}

unsigned int messageStatsGetbitu(const unsigned char *buff, int pos, int len)
{
	unsigned int bits = 0;
	int i;
	for (i = pos; i < pos + len; i++) bits = (bits << 1) + ((buff[i / 8] >> (7 - i % 8)) & 1u);
	return bits;
}

static msg_stats_t createNewMsgStats(int timeMs, string description = "")
{
	msg_stats_t s = {};
	s.prevTimeMs = s.timeMs = timeMs;
	s.description = description;
	return s;
}

static void updateTimeMs(msg_stats_t &s, int timeMs)
{
	s.count++;
	s.prevTimeMs = s.timeMs;
	s.timeMs = timeMs;
}

void messageStatsAppend(string message, mul_msg_stats_t &msgStats, unsigned int ptype, int id, int timeMs)
{
	switch (ptype)
	{
	case _PTYPE_INERTIAL_SENSE_CMD:
	case _PTYPE_INERTIAL_SENSE_DATA:
		if (msgStats.isb.find(id) == msgStats.isb.end())
		{	// Create new 
			msgStats.isb[id] = createNewMsgStats(timeMs, cISDataMappings::GetDataSetName(id));
		}

		{	// Update count and timestamps
			updateTimeMs(msgStats.isb[id], timeMs);
		}
		break;

	case _PTYPE_NMEA:
		if (msgStats.nmea.find(id) == msgStats.nmea.end())
		{	// Create new 
			msgStats.nmea[id] = createNewMsgStats(timeMs);
		}

		{	// Update count and timestamps
			updateTimeMs(msgStats.nmea[id], timeMs);
		}
		break;

	case _PTYPE_UBLOX:
		if (msgStats.ublox.find(id) == msgStats.ublox.end())
		{	// Create new 
			uint8_t msgClass = (uint8_t)id;
			uint8_t msgID = (uint8_t)(id >> 8);
			msgStats.ublox[id] = createNewMsgStats(timeMs, messageDescriptionUblox(msgClass, msgID));
		}

		{	// Update count and timestamps
			updateTimeMs(msgStats.ublox[id], timeMs);
		}
		break;

	case _PTYPE_RTCM3:
		if (msgStats.rtcm3.find(id) == msgStats.rtcm3.end())
		{	// Create new 
			msgStats.rtcm3[id] = createNewMsgStats(timeMs, messageDescriptionRtcm3(id));
		}

		{	// Update count and timestamps
			msg_stats_t &s = msgStats.rtcm3[id];
			updateTimeMs(s, timeMs);

			if (id == 1029)
			{
				s.description = string("Text String: ") + message;
			}
		}
		break;

	case _PTYPE_INERTIAL_SENSE_ACK:
		{
			updateTimeMs(msgStats.ack, timeMs);
		}
		break;

	default:
	case _PTYPE_PARSE_ERROR:
		{
			updateTimeMs(msgStats.parseError, timeMs);
		}
		break;
	}
}

string messageStatsSummary(mul_msg_stats_t &msgStats)
{
	string str;
#define BUF_SIZE 512
	char buf[BUF_SIZE];

	if (!msgStats.isb.empty())
	{
		str.append("Inertial Sense Binary: __________________\n");
		str.append(" DID   Count  dtMs  Description\n");
		std::map<int, msg_stats_t>::iterator it;
		for (it = msgStats.isb.begin(); it != msgStats.isb.end(); it++)
		{
			int did = it->first;
			msg_stats_t &s = it->second;
			int dtMs = (s.prevTimeMs ? (s.timeMs - s.prevTimeMs) : 0);

			SNPRINTF(buf, BUF_SIZE, "%4d %7d %5d  %s\n", did, s.count, dtMs, s.description.c_str());
            str.append(string(buf));
		}
	}

	if (!msgStats.nmea.empty())
	{
		str.append("NMEA: __________________________________\n");
		str.append("  ID   Count  dtMs  Description\n");
		std::map<int, msg_stats_t>::iterator it;
		for (it = msgStats.nmea.begin(); it != msgStats.nmea.end(); it++)
		{
			msg_stats_t &s = it->second;
			int dtMs = (s.prevTimeMs ? (s.timeMs - s.prevTimeMs) : 0);
			union
			{
				int id;
				char str[8];
			} val = {};
			val.id = it->first;
			SNPRINTF(buf, BUF_SIZE, "%s %7d %5d  %s\n", val.str, s.count, dtMs, s.description.c_str());
            str.append(string(buf));
		}
	}

	if (!msgStats.ublox.empty())
	{
		str.append("Ublox: __________________________________\n");
		str.append("(Class  ID)   Count  dtMs  Description\n");
		std::map<int, msg_stats_t>::iterator it;
		for (it = msgStats.ublox.begin(); it != msgStats.ublox.end(); it++)
		{
			int id = it->first;
			msg_stats_t &s = it->second;
			uint8_t msgClass = (uint8_t)id;
			uint8_t msgID = (uint8_t)(id >> 8);
			int dtMs = (s.prevTimeMs ? (s.timeMs - s.prevTimeMs) : 0);
			SNPRINTF(buf, BUF_SIZE, "(0x%02x 0x%02x) %7d %5d  %s\n", msgClass, msgID, s.count, dtMs, s.description.c_str());
            str.append(string(buf));
		}
	}

	if (!msgStats.rtcm3.empty())
	{
		str.append("RTCM3: __________________________________\n");
		str.append("  ID   Count  dtMs  Description\n");
		std::map<int, msg_stats_t>::iterator it;
		for (it = msgStats.rtcm3.begin(); it != msgStats.rtcm3.end(); it++)
		{
			int id = it->first;
			msg_stats_t &s = it->second;
			int dtMs = (s.prevTimeMs ? (s.timeMs - s.prevTimeMs) : 0);
			SNPRINTF(buf, BUF_SIZE, "%3d %7d %5d  %s\n", id, s.count, dtMs, s.description.c_str());
            str.append(string(buf));
		}
	}

	if (msgStats.ack.count>5)
	{
		str.append("Acknowledge: ____________________________\n");
		str.append("   Count  dtMs\n");
		msg_stats_t &s = msgStats.ack;
		int dtMs = (s.prevTimeMs ? (s.timeMs - s.prevTimeMs) : 0);
		SNPRINTF(buf, BUF_SIZE, "%8d %5d\n", s.count, dtMs);
		str.append(string(buf));
	}

#ifdef DEBUG
	if (msgStats.parseError.count>5)
	{
		str.append("Parse Error: ____________________________\n");
		str.append("   Count   dtMs\n");
		msg_stats_t &s = msgStats.parseError;
		int dtMs = (s.prevTimeMs ? (s.timeMs - s.prevTimeMs) : 0);
		SNPRINTF(buf, BUF_SIZE, "%8d %5d\n", s.count, dtMs);
		str.append(string(buf));
	}
#endif
	return str;
}
