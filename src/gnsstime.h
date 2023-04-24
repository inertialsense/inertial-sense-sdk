/**
 * @file gnsstime.h
 * @author Tomoji Takasu (ttaka@gpspp.sakura.ne.jp) (www.rtklib.com)
 * @author Dave Cutting (davidcutting42@gmail.com)
 * @brief Common time-keeping functions, mostly based on RTKLIB, but
 *      used for all GNSS related stuff.
 * @date 2023-04-23
 *
 * @copyright Copyright (C) 2007-2013, T. Takasu, All rights reserved.
 * @copyright Copyright (C) 2023 Inertial Sense, Inc.
 *
 */

#ifndef IS_LIBS_GNSSTIME_H_
#define IS_LIBS_GNSSTIME_H_

#include <stdint.h>

#include "ISConstants.h"

#define MAXLEAPS 64 // max number of leap seconds table

typedef struct
{
    int64_t time; // seconds
    double sec;   // fractional seconds
} gtime_t;

/**
 * @brief String to time (from rtklib)
 *
 * @param s string ("... yyyy mm dd hh mm ss ...")
 * @param i substring position
 * @param n substring width
 * @param t output struct
 * @return int status (0:ok,>0:error)
 */
int str2time(const char *s, int i, int n, gtime_t *t);

/**
 * @brief Convert calendar day/time to time
 *
 * @param ep day/time {year,month,day,hour,min,sec}
 * @return gtime_t output struct
 */
gtime_t epoch2time(const double *ep);

/**
 * @brief Time to calendar day/time
 *
 * @param t input time
 * @param ep output day/time {year,month,day,hour,min,sec}
 */
void time2epoch(gtime_t t, double *ep);

/**
 * @brief Time to calendar day/time, limited to a number of decimals for formatted output
 *
 * @param t input time
 * @param ep output day/time {year,month,day,hour,min,sec}
 * @param n number of decimal points to calculate
 */
void time2epoch_n(gtime_t t, double *ep, int n);

/**
 * @brief convert week and tow in gps time to gtime_t struct
 *
 * @param week week number in gps time
 * @param sec time of week in gps time (s)
 * @return gtime_t output time
 */
gtime_t gpst2time(int week, double sec);

/**
 * @brief convert gtime_t struct to week and tow in gps time
 *
 * @param t input time
 * @param week output week number in gps time (NULL: no output)
 * @return double
 */
double time2gpst(gtime_t t, int *week);

/**
 * @brief convert week and tow in galileo system time (gst) to gtime_t struct
 *
 * @param week week number in gst
 * @param sec time of week in gst (s)
 * @return gtime_t output time
 */
gtime_t gst2time(int week, double sec);

/**
 * @brief convert gtime_t struct to week and tow in galileo system time (gst)
 *
 * @param t time input
 * @param week output week number in gst (NULL: no output)
 * @return double output TOW in gst (s)
 */
double time2gst(gtime_t t, int *week);

/**
 * @brief convert week and tow in beidou time (bdt) to gtime_t struct
 *
 * @param week input week number in bdt
 * @param sec input time of week in bdt (s)
 * @return gtime_t time output
 */
gtime_t bdt2time(int week, double sec);

/**
 * @brief convert gtime_t struct to week and tow in beidou time (bdt)
 *
 * @param t input time
 * @param week output week number in bdt (NULL: no output)
 * @return double time of week in bdt (s)
 */
double time2bdt(gtime_t t, int *week);

/**
 * @brief add time to gtime_t struct
 *
 * @param t time
 * @param sec time to add (s)
 * @return gtime_t gtime_t struct (t+sec)
 */
gtime_t timeadd(gtime_t t, double sec);

/**
 * @brief difference between gtime_t structs
 *
 * @param t1 time 1
 * @param t2 time 2
 * @return double time difference (t1-t2) (s)
 */
double timediff(gtime_t t1, gtime_t t2);

/**
 * @brief Get current system time
 *
 * @return gtime_t output time
 */
gtime_t timeget(void);

/**
 * @brief set current time
 *
 * @param t current time in utc
 */
void timeset(gtime_t t);

/**
 * @brief reset current time to underlying system time
 *
 */
void timereset(void);

/**
 * @brief gps time to utc time considering leap seconds
 * @note ignore slight time offset under 100 ns
 *
 * @param t time expressed in gpstime
 * @return gtime_t time expressed in utc
 */
gtime_t gpst2utc(gtime_t t);

/**
 * @brief convert utc to gpstime considering leap seconds
 * @note ignore slight time offset under 100 ns
 *
 * @param t time time expressed in utc
 * @return gtime_t time expressed in gpstime
 */
gtime_t utc2gpst(gtime_t t);

/**
 * @brief convert gps time to beidou time
 * @note ref [8] 3.3, 2006/1/1 00:00 BDT = 2006/1/1 00:00 UTC
 * @note no leap seconds in BDT
 * @note ignore slight time offset under 100 ns
 *
 * @param t time expressed in gpstime
 * @return gtime_t time expressed in bdt
 */
gtime_t gpst2bdt(gtime_t t);

/**
 * @brief convert beidou time to gps time
 * @note see notes for gpst2bdt()
 *
 * @param t time expressed in bdt
 * @return gtime_t time expressed in gpstime
 */
gtime_t bdt2gpst(gtime_t t);

/**
 * @brief convert time struct to number of seconds elapsed in current day
 *
 * @param time input time
 * @param day output time, minus return value
 * @return double
 */
double time2sec(gtime_t time, gtime_t *day);

/**
 * @brief convert utc to gmst (Greenwich mean sidereal time)
 *
 * @param t input time expressed in utc
 * @param ut1_utc input UT1-UTC (s)
 * @return double gmst (rad)
 */
double utc2gmst(gtime_t t, double ut1_utc);

/**
 * @brief convert gtime_t struct to string
 *
 * @param t time struct
 * @param s output string ("yyyy/mm/dd hh:mm:ss.ssss")
 * @param n number of decimals for seconds
 */
void time2str(gtime_t t, char *s, int n);

/**
 * @brief get time string
 * @warning not reentrant, do not use multiple in a function
 *
 * @param t time struct
 * @param n number of decimals
 * @return char* time string
 */
char *time_str(gtime_t t, int n);

/**
 * @brief convert time to day of year
 *
 * @param t time struct
 * @return double day of year (days)
 */
double time2doy(gtime_t t);

/**
 * @brief adjust gps week number using cpu time
 *
 * @param week not-adjusted gps week number (0-1023)
 * @return int adjusted gps week number
 */
int adjgpsweek(int week);

/**
 * @brief Get current tick time
 *
 * @return uint32_t tick time in milliseconds
 */
uint32_t tickget(void);

/**
 * @brief Sleep for milliseconds
 *
 * @param ms milliseconds to sleep (<0:no sleep)
 */
void sleepms(int ms);

#endif // IS_LIBS_GNSSTIME_H_
