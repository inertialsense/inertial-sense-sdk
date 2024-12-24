#ifndef _C_TIMECONV_H_
#define _C_TIMECONV_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <ctime>
#include "stdint.h"

#define C_SECONDS_PER_WEEK          (   604800)        // (60 * 60 * 24 * 7)
#define C_SECONDS_PER_DAY           (    86400)
#define C_MILLISECONDS_PER_WEEK     (604800000)     // (60 * 60 * 24 * 7 * 1000)
#define C_MILLISECONDS_PER_DAY      ( 86400000)
#define C_MILLISECONDS_PER_HOUR     (  3600000)
#define C_MILLISECONDS_PER_MINUTE   (    60000)
#define C_MILLISECONDS_PER_SECOND   (     1000)
#define C_DAYS_PER_SECOND           (1.1574074074074074074074074074074e-5)
#define C_GPS_TO_UNIX_OFFSET_S      (315964800)

typedef struct
{
    int year;
    int month;
    int day;        // Day of month
    int weekday;    // Day of week
} utc_date_t;

typedef struct
{
    int hour;
    int minute;
    int second;
    int millisecond;
} utc_time_t;

/**
 * @brief Set and revert the UTC Time Zone object
 */
void SetUtcTimeZone(); 
void RevertUtcTimeZone();

/** Convert GPS time of week in milliseconds to UTC time */
void gpsTowMsToUtcTime(uint32_t gpsTimeOfWeekMs, int gpsLeapS, utc_time_t *time);

/** Convert GPS week and time of week in milliseconds to UTC date and time */
void gpsWeekTowMsToUtcDateTime(uint32_t gpsWeek, uint32_t gpsTowMs, int gpsLeapS, utc_date_t *date, utc_time_t *time, uint32_t *milliseconds);

/** Convert UTC time to GPS time of week in milliseconds */
void utcTimeToGpsTowMs(utc_time_t *time, int utcWeekday, uint32_t *gpsTimeOfWeekMs, int gpsLeapS);

/** Convert GPS time in milliseconds to UTC weekday */
int gpsTowMsToUtcWeekday(int gpsTowMs, int leapS);

/**
 * @brief Convert GPS time of week in milliseconds to UTC date and time.
 * 
 * @param gpsSecondsOfWeek Output GPS seconds of week.
 * @param gpsWeek Output GPS week number since January 6th, 1980.
 * @param leapSeconds Leap seconds to account for difference between GPS and UTC time (18s by default).
 * @return std::tm UTC time.
 */
std::tm stdGpsTimeToUtcDateTime(uint32_t gpsSecondsOfWeek, uint32_t gpsWeek, int leapSeconds);

/**
 * @brief Convert UTC date and time to GPS seconds in week and week number.  This function is 8x computationally 
 * more intensive than UtcDateTimeToGpsTime(). 
 * 
 * @param utcTime UTC time as std::tm.
 * @param gpsSecondsOfWeek Output GPS seconds of week.
 * @param gpsWeek Output GPS week number since January 6th, 1980.
 * @param leapSeconds Leap seconds to account for difference between GPS and UTC time (18s by default).
 */
void stdUtcDateTimeToGpsTime(const std::tm &utcTime, int leapSeconds, uint32_t &gpsSecondsOfWeek, uint32_t &gpsWeek);

/**
 * @brief Convert UTC date and time to GPS time in time of week in milliseconds and number of weeks.  
 * This function is equivalent to and computationally faster than stdUtcDateTimeToGpsTime().
 * 
 * @param datetime[7] input int array of UTC time at GMT time zone {year,month,day,hour,min,sec,msec}
 * @param gpsLeapSeconds input number GPS leap seconds to convert to UTC time
 * @param gpsTowMs output GPS time of week in milliseconds 
 * @param gpsWeek output GPS week number
 */
void UtcDateTimeToGpsTime(const int datetime[7], int leapSeconds, uint32_t &gpsTowMs, uint32_t &gpsWeek);

/** Convert Julian Date to calendar date. */
void julianToDate(double julian, uint32_t* year, uint32_t* month, uint32_t* day, uint32_t* hour, uint32_t* minute, uint32_t* second, uint32_t* millisecond);

/** Convert GPS Week and Ms and leapSeconds to Unix seconds**/
double gpsToUnix(uint32_t gpsWeek, uint32_t gpsTimeofWeekMS, uint8_t leapSeconds);

/** Convert GPS Week and Seconds to Julian Date.  Leap seconds are the GPS-UTC offset (18 seconds as of December 31, 2016). */
double gpsToJulian(uint32_t gpsWeek, uint32_t gpsMilliseconds, uint32_t leapSeconds);



#if 0

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
int TIMECONV_UTCTimeFromGPSTime(
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

#endif  // #if 0

#ifdef __cplusplus
}
#endif

#endif // _C_TIMECONV_H_
