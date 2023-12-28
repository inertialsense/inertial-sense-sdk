// TAKEN FROM https://svn.code.sf.net/p/gnsstk/code/trunk/src/time_conversion.h

/**
\file    time_conversion.h
\brief   GNSS core 'c' function library: converting time information.
\author  Glenn D. MacGougan (GDM)
\date    2007-11-29
\since   2005-07-30

\b REFERENCES     
- Hofmann-Wellenhof, B., H. Lichtenegger, and J. Collins (1994). GPS Theory and 
  Practice, Third, revised edition. Springer-Verlag, Wien New York. pp. 38-42     
- http://aa.usno.navy.mil/data/docs/JulianDate.html - Julian Date Converter     
- http://aa.usno.navy.mil/faq/docs/UT.html     
- http://wwwmacho.mcmaster.ca/JAVA/JD.html     
- Raquet, J. F. (2002), GPS Receiver Design Lecture Notes. Geomatics Engineering, 
  University of Calgary Graduate Course.     

\b "LICENSE INFORMATION"     
Copyright (c) 2007, refer to 'author' doxygen tags     
All rights reserved.     

Redistribution and use in source and binary forms, with or without
modification, are permitted provided the following conditions are met:     

- Redistributions of source code must retain the above copyright
  notice, this list of conditions and the following disclaimer.     
- Redistributions in binary form must reproduce the above copyright
  notice, this list of conditions and the following disclaimer in the
  documentation and/or other materials provided with the distribution.     
- The name(s) of the contributor(s) may not be used to endorse or promote 
  products derived from this software without specific prior written 
  permission.     

THIS SOFTWARE IS PROVIDED BY THE CONTRIBUTORS ``AS IS'' AND ANY EXPRESS 
OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR 
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER 
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT 
LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY 
OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF 
SUCH DAMAGE.
*/

#ifndef _C_TIMECONV_H_
#define _C_TIMECONV_H_

#ifdef __cplusplus
extern "C" {
#endif

/*************************************************************************************************/
// preprocessor constant definitions, any related enumerations and descriptors

// error, define to printf or something if you want
#define GNSS_ERROR_MSG(errorMsg)

#ifndef SECONDS_IN_DAY
#define SECONDS_IN_DAY (86400.0)
#endif

#ifndef SECONDS_IN_WEEK
#define SECONDS_IN_WEEK (604800.0)
#endif

/**
\brief    Obtains the UTC time, GPS time, and Julian date from PC system time.

\author   Glenn D. MacGougan (GDM)
\date     2006-11-10
\since    2005-08-22
\return   1(1) if successful, 0(0) otherwise.
\remarks  (1) Millisecond time is obtained
*/
int TIMECONV_GetSystemTime(
  unsigned short*     utc_year,     //!< Universal Time Coordinated    [year]
  unsigned char*      utc_month,    //!< Universal Time Coordinated    [1-12 months] 
  unsigned char*      utc_day,      //!< Universal Time Coordinated    [1-31 days]
  unsigned char*      utc_hour,     //!< Universal Time Coordinated    [hours]
  unsigned char*      utc_minute,   //!< Universal Time Coordinated    [minutes]
  float*              utc_seconds,  //!< Universal Time Coordinated    [s]
  unsigned char*      utc_offset,   //!< Integer seconds that GPS is ahead of UTC time, always positive             [s], obtained from a look up table
  double*             julian_date,  //!< Number of days since noon Universal Time Jan 1, 4713 BCE (Julian calendar) [days]
  unsigned short*     gps_week,     //!< GPS week (0-1024+)            [week]
  double*             gps_tow       //!< GPS time of week (0-604800.0) [s]
  );


#ifdef WIN32
/**
\brief    Sets the PC time to the UTC time provided.

\author   Glenn D. MacGougan (GDM)
\date     2008-12-03
\since    2008-12-03
\return   1(1) if successful, 0(0) otherwise.
*/
int TIMECONV_SetSystemTime(
  const unsigned short  utc_year,     //!< Universal Time Coordinated    [year]
  const unsigned char   utc_month,    //!< Universal Time Coordinated    [1-12 months] 
  const unsigned char   utc_day,      //!< Universal Time Coordinated    [1-31 days]
  const unsigned char   utc_hour,     //!< Universal Time Coordinated    [hours]
  const unsigned char   utc_minute,   //!< Universal Time Coordinated    [minutes]
  const float           utc_seconds   //!< Universal Time Coordinated    [s]
  );
#endif


/**
\brief    Computes the day of the week from the Julian date.

\author   Glenn D. MacGougan (GDM)
\date     2008-12-03
\since    2008-12-03
\return   1(1) if successful, 0(0) otherwise.

\b REFERENCES     
http://en.wikipedia.org/wiki/Julian_day
*/
int TIMECONV_GetDayOfWeekFromJulianDate(
  const double julian_date,   //!< Number of days since noon Universal Time Jan 1, 4713 BCE (Julian calendar) [days]
  unsigned char *day_of_week  //!< 0-Sunday, 1-Monday, 2-Tuesday, 3-Wednesday, 4-Thursday, 5-Friday, 6-Saturday [].
  );


/**
\brief    Computes the Julian date from GPS time

\author   Glenn D. MacGougan (GDM)
\date     2007-11-29
\since    2005-08-22
\return   1(1) if successful, 0(0) otherwise.

\b REFERENCES     
- Hofmann-Wellenhof, B., H. Lichtenegger, and J. Collins (1994). GPS Theory and 
  Practice, Third, revised edition. Springer-Verlag, Wien New York. pp. 38-42     
*/
int TIMECONV_GetJulianDateFromGPSTime(
  const unsigned short    gps_week,      //!< GPS week (0-1024+)             [week]
  const double            gps_tow,       //!< GPS time of week (0-604800.0)  [s]
  const unsigned char     utc_offset,    //!< Integer seconds that GPS is ahead of UTC time, always positive [s]
  double*                 julian_date    //!< Number of days since noon Universal Time Jan 1, 4713 BCE (Julian calendar) [days]
  );

/**
\brief    Computes the Julian date from UTC time

\author   Glenn D. MacGougan (GDM)
\date     2007-11-29
\since    2005-08-22
\return   1(1) if successful, 0(0) otherwise.

\remarks
- Verified calculation using http://aa.usno.navy.mil/data/docs/JulianDate.html,
  a Julian Date Converter and http://wwwmacho.mcmaster.ca/JAVA/JD.html,
  another online converter tool.     

\b REFERENCES     
- Hofmann-Wellenhof, B., H. Lichtenegger, and J. Collins (1994). GPS Theory and 
Practice, Third, revised edition. Springer-Verlag, Wien New York. pp. 38-42     
*/
int TIMECONV_GetJulianDateFromUTCTime(
 const unsigned short     utc_year,      //!< Universal Time Coordinated  [year]
 const unsigned char      utc_month,     //!< Universal Time Coordinated  [1-12 months] 
 const unsigned char      utc_day,       //!< Universal Time Coordinated  [1-31 days]
 const unsigned char      utc_hour,      //!< Universal Time Coordinated  [hours]
 const unsigned char      utc_minute,    //!< Universal Time Coordinated  [minutes]
 const float              utc_seconds,   //!< Universal Time Coordinated  [s]
 double*                  julian_date    //!< Number of days since noon Universal Time Jan 1, 4713 BCE (Julian calendar) [days]
 );




/**
\brief    Computes GPS time from the Julian date

\author   Glenn D. MacGougan (GDM)
\date     2007-11-29
\since    2005-08-22
\return   1(1) if successful, 0(0) otherwise.

\b REFERENCES     
- Hofmann-Wellenhof, B., H. Lichtenegger, and J. Collins (1994). GPS Theory and 
  Practice, Third, revised edition. Springer-Verlag, Wien New York. pp. 38-42     
*/
int TIMECONV_GetGPSTimeFromJulianDate(
  const double            julian_date, //!< Number of days since noon Universal Time Jan 1, 4713 BCE (Julian calendar) [days]
  const unsigned char     utc_offset,  //!< Integer seconds that GPS is ahead of UTC time, always positive [s]
  unsigned short*         gps_week,    //!< GPS week (0-1024+)            [week]
  double*                 gps_tow      //!< GPS time of week [s]
  );

/**
\brief    Computes UTC time from the Julian date

\author   Glenn D. MacGougan (GDM)
\date     2007-11-29
\since    2005-08-22
\return   1(1) if successful, 0(0) otherwise.

\b REFERENCES     
- Hofmann-Wellenhof, B., H. Lichtenegger, and J. Collins (1994). GPS Theory and 
  Practice, Third, revised edition. Springer-Verlag, Wien New York. pp. 38-42     
*/
int TIMECONV_GetUTCTimeFromJulianDate(
  const double        julian_date,  //!< Number of days since noon Universal Time Jan 1, 4713 BCE (Julian calendar) [days]
  unsigned short*     utc_year,     //!< Universal Time Coordinated    [year]
  unsigned char*      utc_month,    //!< Universal Time Coordinated    [1-12 months] 
  unsigned char*      utc_day,      //!< Universal Time Coordinated    [1-31 days]
  unsigned char*      utc_hour,     //!< Universal Time Coordinated    [hours]
  unsigned char*      utc_minute,   //!< Universal Time Coordinated    [minutes]
  float*              utc_seconds   //!< Universal Time Coordinated    [s]
  );

/**
\brief    Computes GPS time from UTC time

\author   Glenn D. MacGougan (GDM)
\date     2007-11-29
\since    2005-08-22
\return   1(1) if successful, 0(0) otherwise.

\remarks
(1) The utc offset is determined automatically from a look up table

\b REFERENCES     
- Hofmann-Wellenhof, B., H. Lichtenegger, and J. Collins (1994). GPS Theory and 
  Practice, Third, revised edition. Springer-Verlag, Wien New York. pp. 38-42     
*/
int TIMECONV_GetGPSTimeFromUTCTime(
  unsigned short     utc_year,     //!< Universal Time Coordinated    [year]
  unsigned char      utc_month,    //!< Universal Time Coordinated    [1-12 months] 
  unsigned char      utc_day,      //!< Universal Time Coordinated    [1-31 days]
  unsigned char      utc_hour,     //!< Universal Time Coordinated    [hours]
  unsigned char      utc_minute,   //!< Universal Time Coordinated    [minutes]
  float              utc_seconds,  //!< Universal Time Coordinated    [s]
  unsigned short*    gps_week,     //!< GPS week (0-1024+)            [week]
  double*            gps_tow       //!< GPS time of week (0-604800.0) [s]
  );


/**
\brief    Computes GPS time from RINEX time. RINEX time looks like UTC
          but it is GPS time in year, month, day, hours, minutes, seconds.

\author   Glenn D. MacGougan (GDM)
\date     2007-12-07
\since    2007-12-07
\return   1(1) if successful, 0(0) otherwise.

\remarks
- There is no UTC offset to apply
- The RINEX time system must be the GPS Time system to use this function.

\b REFERENCES     
- Hofmann-Wellenhof, B., H. Lichtenegger, and J. Collins (1994). GPS Theory and 
  Practice, Third, revised edition. Springer-Verlag, Wien New York. pp. 38-42     
- RINEX version 2.11, (http://www.aiub-download.unibe.ch/rinex/rinex211.txt)
*/
int TIMECONV_GetGPSTimeFromRinexTime(
  unsigned short     utc_year,     //!< Universal Time Coordinated    [year]
  unsigned char      utc_month,    //!< Universal Time Coordinated    [1-12 months] 
  unsigned char      utc_day,      //!< Universal Time Coordinated    [1-31 days]
  unsigned char      utc_hour,     //!< Universal Time Coordinated    [hours]
  unsigned char      utc_minute,   //!< Universal Time Coordinated    [minutes]
  float              utc_seconds,  //!< Universal Time Coordinated    [s]
  unsigned short*    gps_week,     //!< GPS week (0-1024+)            [week]
  double*            gps_tow       //!< GPS time of week (0-604800.0) [s]
  );


/**
\brief    Computes UTC time from GPS time

\author   Glenn D. MacGougan (GDM)
\date     2007-11-29
\since    2005-08-22
\return   1(1) if successful, 0(0) otherwise.

\remarks
- The utc offset is determined automatically from a look up table

\b REFERENCES     
- Hofmann-Wellenhof, B., H. Lichtenegger, and J. Collins (1994). GPS Theory and 
  Practice, Third, revised edition. Springer-Verlag, Wien New York. pp. 38-42     
*/
int TIMECONV_GetUTCTimeFromGPSTime(
  unsigned short     gps_week,     //!< GPS week (0-1024+)            [week]
  double             gps_tow,      //!< GPS time of week (0-604800.0) [s]
  unsigned short*    utc_year,     //!< Universal Time Coordinated    [year]
  unsigned char*     utc_month,    //!< Universal Time Coordinated    [1-12 months] 
  unsigned char*     utc_day,      //!< Universal Time Coordinated    [1-31 days]
  unsigned char*     utc_hour,     //!< Universal Time Coordinated    [hours]
  unsigned char*     utc_minute,   //!< Universal Time Coordinated    [minutes]
  float*             utc_seconds   //!< Universal Time Coordinated    [s]
  );



/**
\brief    This function is a look up table to determine the UTC offset from the Julian Date.

\author   Glenn D. MacGougan (GDM)
\date     2007-11-29
\since    2005-08-22
\return   1(1) if successful, 0(0) otherwise.

\remarks
- This function must be updated when the next UTC *utc_offset step occurs. Current max is (13).
 
\b REFERENCES     
- Raquet, J. F. (2002), GPS Receiver Design Lecture Notes. Geomatics Engineering, 
  University of Calgary Graduate Course.     

\b "Offset Table"     
UTCOffset, UTC Date, Julian Date [days]     
0,    Jan 06 1980 00:00:00.0,    2444244.5000     
1,    Jul 01 1981 00:00:00.0,    2444786.5000     
2,    Jul 01 1982 00:00:00.0,    2445151.5000     
3,    Jul 01 1983 00:00:00.0,    2445516.5000     
4,    Jul 01 1985 00:00:00.0,    2446247.5000     
5,    Jan 01 1988 00:00:00.0,    2447161.5000     
6,    Jan 01 1990 00:00:00.0,    2447892.5000     
7,    Jan 01 1991 00:00:00.0,    2448257.5000     
8,    Jul 01 1992 00:00:00.0,    2448804.5000     
9,    Jul 01 1993 00:00:00.0,    2449169.5000     
10,   Jul 01 1994 00:00:00.0,    2449534.5000     
11,   Jan 01 1996 00:00:00.0,    2450083.5000     
12,   Jul 01 1997 00:00:00.0,    2450630.5000     
13,   Jan 01 1999 00:00:00.0,    2451179.5000     
14,   Jan 01 2006 00:00:00.0,    2453736.5000     
*/
int TIMECONV_DetermineUTCOffset(
  double julian_date,       //!< Number of days since noon Universal Time Jan 1, 4713 BCE (Julian calendar) [days]
  unsigned char* utc_offset //!< Integer seconds that GPS is ahead of UTC time, always positive             [s], obtained from a look up table
  );

/**
\brief    Determines the number of days in a month, given the month and year.

\author   Glenn D. MacGougan (GDM)
\date     2007-11-29
\since    2005-08-22
\return   1(1) if successful, 0(0) otherwise.

\b REFERENCES     
- Hofmann-Wellenhof, B., H. Lichtenegger, and J. Collins (1994). GPS Theory and 
  Practice, Third, revised edition. Springer-Verlag, Wien New York. pp. 38-42     
*/
int TIMECONV_GetNumberOfDaysInMonth(
  const unsigned short year,        //!< Universal Time Coordinated    [year]
  const unsigned char month,        //!< Universal Time Coordinated    [1-12 months] 
  unsigned char* days_in_month      //!< Days in the specified month   [1-28|29|30|31 days]
  );


/**
\brief    Determines if the given year is a leap year

\author   Glenn D. MacGougan (GDM)
\date     2007-11-29
\since    2005-08-22
\returns  1(1) if the given year is a leap year, 0(0) otherwise

- Hofmann-Wellenhof, B., H. Lichtenegger, and J. Collins (1994). GPS Theory and 
  Practice, Third, revised edition. Springer-Verlag, Wien New York. pp. 38-42     
*/
int TIMECONV_IsALeapYear( const unsigned short year );


/**
\brief    Determines the day of year given the year, month, and day

\author   Glenn D. MacGougan (GDM)
\date     2007-11-29
\since    2005-08-22
\return   1(1) if successful, 0(0) otherwise.

\remarks
(1) Performed independant comparison with http://www.mbari.org/staff/coletti/doytable.html
*/
int TIMECONV_GetDayOfYear(
 const unsigned short utc_year,    // Universal Time Coordinated           [year]
 const unsigned char  utc_month,   // Universal Time Coordinated           [1-12 months] 
 const unsigned char  utc_day,     // Universal Time Coordinated           [1-31 days]
 unsigned short*      dayofyear    // number of days into the year (1-366) [days]
 );


/**
\brief    Determines the GPS time of the start of a day from the day of year and the year.

\author   Glenn D. MacGougan (GDM)
\date     2007-12-07
\since    2007-12-07
\return   1(1) if successful, 0(0) otherwise.
*/
int TIMECONV_GetGPSTimeFromYearAndDayOfYear(
 const unsigned short year,      // The year [year]
 const unsigned short dayofyear, // The number of days into the year (1-366) [days]
 unsigned short*      gps_week,  //!< GPS week (0-1024+)            [week]
 double*              gps_tow    //!< GPS time of week (0-604800.0) [s]
 );

 
  
#ifdef __cplusplus
}
#endif


#endif // _C_TIMECONV_H_
