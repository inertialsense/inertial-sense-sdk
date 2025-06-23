#include <string>
#include "ISConstants.h"
#include "time_conversion.h"

#define GNSS_ERROR_MSG(errorMsg)

#define TIMECONV_JULIAN_DATE_START_OF_GPS_TIME (2444244.5)  // [days]
#define TIMECONV_JULIAN_DATE_START_OF_PC_TIME  (2440587.5)  // [days]
#define TIMECONV_DAYS_IN_JAN 31
#define TIMECONV_DAYS_IN_MAR 31
#define TIMECONV_DAYS_IN_APR 30
#define TIMECONV_DAYS_IN_MAY 31
#define TIMECONV_DAYS_IN_JUN 30
#define TIMECONV_DAYS_IN_JUL 31
#define TIMECONV_DAYS_IN_AUG 31
#define TIMECONV_DAYS_IN_SEP 30
#define TIMECONV_DAYS_IN_OCT 31
#define TIMECONV_DAYS_IN_NOV 30
#define TIMECONV_DAYS_IN_DEC 31

void gpsTowMsToUtcTime(uint32_t gpsTimeOfWeekMs, int gpsLeapS, utc_time_t *time)
{
    int leapMs = gpsLeapS * 1000;
    int towMsMinusLeapMs = gpsTimeOfWeekMs - leapMs;
    if (towMsMinusLeapMs < 0)
    {   // Handle week wrap
        towMsMinusLeapMs += C_MILLISECONDS_PER_WEEK;
    }
    int todayMs = (towMsMinusLeapMs) % C_MILLISECONDS_PER_DAY;
	time->hour   = (todayMs / C_MILLISECONDS_PER_HOUR);
	time->minute = (todayMs / 60000) % 60;
	time->second = (todayMs / 1000) % 60;
	time->millisecond = todayMs % 1000;
}

void gpsWeekTowMsToUtcDateTime(uint32_t gpsWeek, uint32_t gpsTowMs, int gpsLeapS, utc_date_t *date, utc_time_t *time, uint32_t *milliseconds)
{
    double julian = gpsToJulian(gpsWeek, gpsTowMs, gpsLeapS);
    julianToDate(julian, 
        (uint32_t*)&date->year, 
        (uint32_t*)&date->month, 
        (uint32_t*)&date->day, 
        (uint32_t*)&time->hour, 
        (uint32_t*)&time->minute, 
        (uint32_t*)&time->second, 
        (uint32_t*)milliseconds);
    date->weekday = gpsTowMsToUtcWeekday(gpsTowMs, gpsLeapS);
}

void utcTimeToGpsTowMs(utc_time_t *time, int utcWeekday, uint32_t *gpsTimeOfWeekMs, int gpsLeapS)
{
    int towMs = 
        utcWeekday   * C_MILLISECONDS_PER_DAY +
        time->hour   * C_MILLISECONDS_PER_HOUR +
        time->minute * C_MILLISECONDS_PER_MINUTE +
        time->second * C_MILLISECONDS_PER_SECOND +
        time->millisecond +
        gpsLeapS * 1000;

    // Handle week wrap
    if (towMs >= C_MILLISECONDS_PER_WEEK)
    {   
        towMs -= C_MILLISECONDS_PER_WEEK;
    } 
    else if (towMs < 0)
    {
        towMs += C_MILLISECONDS_PER_WEEK;
    }

    *gpsTimeOfWeekMs = towMs;
}

int gpsTowMsToUtcWeekday(int gpsTowMs, int leapS)
{
    int utcTowMs = gpsTowMs - (leapS*1000);
    if (utcTowMs < 0)
    {   // Handle wrap
        utcTowMs -= C_MILLISECONDS_PER_DAY;
    }

    int utcWeekday = utcTowMs / C_MILLISECONDS_PER_DAY;
    return utcWeekday;
}

#if !PLATFORM_IS_EMBEDDED
static std::string s_savedTZ;

void SetUtcTimeZone()
{
#if PLATFORM_IS_LINUX
    const char* oldTZ = getenv("TZ");
    s_savedTZ = oldTZ ? oldTZ : "";

    setenv("TZ", "UTC", 1);
    tzset();
#endif
}

void RevertUtcTimeZone()
{
#if PLATFORM_IS_LINUX
    // Restore the original timezone if necessary
    if (s_savedTZ.empty()) 
    {
        unsetenv("TZ");
    } 
    else 
    {
        setenv("TZ", s_savedTZ.c_str(), 1);
    }
    tzset();  // Reapply the original timezone settings
#endif
}
#endif

std::tm stdGpsTimeToUtcDateTime(uint32_t gpsSecondsOfWeek, uint32_t gpsWeek, int leapSeconds) 
{
    std::tm gpsEpoch = {};
    gpsEpoch.tm_year = 80;  // Year 1980
    gpsEpoch.tm_mon = 0;    // January
    gpsEpoch.tm_mday = 6;   // 6th
    gpsEpoch.tm_hour = 0;   // 00:00:00
    std::time_t epochTime = std::mktime(&gpsEpoch);

    // Total seconds since GPS epoch
    std::time_t totalSeconds = epochTime + gpsWeek * C_SECONDS_PER_WEEK + gpsSecondsOfWeek;

    // Adjust for leap seconds
    totalSeconds -= leapSeconds;

    // Convert to UTC
    std::tm *utcTime = std::gmtime(&totalSeconds);

    return *utcTime;
}

void stdUtcDateTimeToGpsTime(const std::tm &utcTime, int leapSeconds, uint32_t &gpsSecondsOfWeek, uint32_t &gpsWeek)
{
    std::tm gpsEpoch = {};
    gpsEpoch.tm_year = 80;  // Year 1980
    gpsEpoch.tm_mon = 0;    // January
    gpsEpoch.tm_mday = 6;   // 6th
    gpsEpoch.tm_hour = 0;   // 00:00:00
    std::time_t epochTime = std::mktime(&gpsEpoch);

    // Convert UTC tm structure to time_t
    std::time_t utcTimeT = std::mktime(const_cast<std::tm*>(&utcTime));

    // Calculate the difference in seconds from GPS epoch
    long long secondsSinceGpsEpoch = (long long)std::difftime(utcTimeT, epochTime);

    // Subtract leap seconds (update this value based on the current number of leap seconds)
    secondsSinceGpsEpoch += leapSeconds;

    // Calculate GPS week and seconds of week
    gpsWeek = (uint32_t)(secondsSinceGpsEpoch / C_SECONDS_PER_WEEK);
    gpsSecondsOfWeek = (uint32_t)(secondsSinceGpsEpoch - gpsWeek*C_SECONDS_PER_WEEK);
}

/* convert calendar day/time to time -------------------------------------------
* convert calendar day/time to gtime_t struct
* args   : int *ep       I   day/time {year,month,day,hour,min,sec,msec}
* return : gtime_t struct
* notes  : proper in 1970-2037 or 1970-2099 (64bit time_t)
*-----------------------------------------------------------------------------*/
gtime_t epochToTime(const int *ep)
{
    const int doy[] = { 1,32,60,91,121,152,182,213,244,274,305,335 };
    gtime_t time = { 0 };
    int days;
    int year    = ep[0];
    int mon     = ep[1];
    int day     = ep[2];
    int hour    = ep[3];
    int minute  = ep[4];
    int sec     = ep[5];
    int msec    = ep[6];

    if (year < 1970 || 2099 < year || mon < 1 || 12 < mon) 
        return time;

    /* leap year if year%4==0 in 1901-2099 */
    days = (year - 1970) * 365 + (year - 1969) / 4 + doy[mon - 1] + day - 2 + (year % 4 == 0 && mon >= 3 ? 1 : 0);
    time.time = (time_t)days * 86400 + hour * 3600 + minute * 60 + sec;
    time.sec = msec * 0.001;
    return time;
}

static const gtime_t gpsRefT0 = { 315964800, 0 };	// (gtime) gps reference time = epochToTime(gpst0) at 1980

/* time to gps time ------------------------------------------------------------
* convert gtime_t struct to week and tow in gps time
* args   : gtime_t t        I   gtime_t struct
*          int    *week     IO  week number in gps time (NULL: no output)
* return : time of week in gps time (s)
*-----------------------------------------------------------------------------*/
double timeToGpst(gtime_t t, int *week)
{
	time_t sec = t.time - gpsRefT0.time;
	time_t w = (time_t)(sec / C_SECONDS_PER_WEEK);
	
	if (week){ *week = (int)w; }
	return (double)(sec - (double)w * C_SECONDS_PER_WEEK) + t.sec;
}

void UtcDateTimeToGpsTime(const int dateTime[7], int leapSeconds, uint32_t &gpsTowMs, uint32_t &gpsWeek)
{
    gtime_t gtm = epochToTime(dateTime);

    int week;
    gtm.time += leapSeconds;
    double iTOWd = timeToGpst(gtm, &week);
    gpsTowMs = (uint32_t)((iTOWd * 1000.0) + 0.5);  // Ensure double rounds properly to nearest integer millisecond
    gpsWeek = week;
}

void julianToDate(double julian, uint32_t* year, uint32_t* month, uint32_t* day, uint32_t* hour, uint32_t* minute, uint32_t* second, uint32_t* millisecond) {
    // Adjust from Julian date to a more computationally-friendly epoch
    double jd = julian + 0.5;
    uint32_t Z = (uint32_t)jd;
    double F = jd - Z;

    uint32_t A;
    if (Z < 2299161) {
        A = Z;
    } else {
        uint32_t alpha = (uint32_t)((Z - 1867216.25) / 36524.25);
        A = Z + 1 + alpha - (alpha / 4);
    }

    uint32_t B = A + 1524;
    uint32_t C = (uint32_t)((B - 122.1) / 365.25);
    uint32_t D = (uint32_t)(365.25 * C);
    uint32_t E = (uint32_t)((B - D) / 30.6001);

    double fractionalDay = B - D - (uint32_t)(30.6001 * E) + F;

    *day = (uint32_t)fractionalDay;

    // Calculate the month and year
    if (E < 14) {
        *month = E - 1;
    } else {
        *month = E - 13;
    }

    if (*month > 2) {
        *year = C - 4716;
    } else {
        *year = C - 4715;
    }

    // Calculate the time components
    double fractionalDayPart = fractionalDay - *day;
    double totalSeconds = fractionalDayPart * 86400.0; // 86400 seconds in a day

    *hour = (uint32_t)(totalSeconds / 3600);
    totalSeconds -= *hour * 3600;

    *minute = (uint32_t)(totalSeconds / 60);
    totalSeconds -= *minute * 60;

    *second = (uint32_t)totalSeconds;
    *millisecond = (uint32_t)(round((totalSeconds - *second) * 1000.0)); // Use rounding

    // Ensure millisecond rollover correctness
    if (*millisecond == 1000) {
        *millisecond = 0;
        *second += 1;

        if (*second == 60) {
            *second = 0;
            *minute += 1;

            if (*minute == 60) {
                *minute = 0;
                *hour += 1;

                if (*hour == 24) {
                    *hour = 0;
                    *day += 1;

                    // Handle month and year increment for the day overflow
                    static const uint32_t daysInMonth[] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};

                    uint32_t daysThisMonth = daysInMonth[*month - 1];
                    if (*month == 2 && ((*year % 4 == 0 && *year % 100 != 0) || *year % 400 == 0)) {
                        daysThisMonth = 29; // Leap year for February
                    }

                    if (*day > daysThisMonth) {
                        *day = 1;
                        *month += 1;

                        if (*month > 12) {
                            *month = 1;
                            *year += 1;
                        }
                    }
                }
            }
        }
    }
}

double gpsToUnix(uint32_t gpsWeek, uint32_t gpsTimeofWeekMs, uint8_t leapSeconds)
{
    uint32_t gpsTow = gpsTimeofWeekMs / 1000;
	uint32_t gpsTime = gpsWeek * C_SECONDS_PER_WEEK + gpsTow;
	double unixSeconds = (double)(gpsTime + C_GPS_TO_UNIX_OFFSET_S - leapSeconds);
#if 1   // Include fractional seconds
    double gpsFracS = (gpsTimeofWeekMs - gpsTow) * 0.001;
    unixSeconds += gpsFracS;
#endif

	return unixSeconds;
}

double gpsToJulian(uint32_t gpsWeek, uint32_t gpsMilliseconds, uint32_t leapSeconds)
{
	double gpsDays = (double)(gpsWeek * 7);
	gpsDays += ((((double)gpsMilliseconds) * 0.001) - (double)leapSeconds) * C_DAYS_PER_SECOND;
	return TIMECONV_JULIAN_DATE_START_OF_GPS_TIME + gpsDays; // 2444244.500000 Julian date for Jan 6, 1980 midnight - start of gps time
}

#if 0

#include <sys/timeb.h>
#ifndef _WIN32
#include <sys/time.h>
#endif
#include <time.h>
#include <math.h> // for fmod()

// A static function to check if the utc input values are valid.
// \return 1 if valid, 0 otherwise.
static int TIMECONV_IsUTCTimeValid( 
    const unsigned short     utc_year,      //!< Universal Time Coordinated  [year]
    const unsigned char      utc_month,     //!< Universal Time Coordinated  [1-12 months] 
    const unsigned char      utc_day,       //!< Universal Time Coordinated  [1-31 days]
    const unsigned char      utc_hour,      //!< Universal Time Coordinated  [hours]
    const unsigned char      utc_minute,    //!< Universal Time Coordinated  [minutes]
    const float              utc_seconds    //!< Universal Time Coordinated  [s]
 );
 

int TIMECONV_IsUTCTimeValid( 
    const unsigned short     utc_year,      //!< Universal Time Coordinated  [year]
    const unsigned char      utc_month,     //!< Universal Time Coordinated  [1-12 months] 
    const unsigned char      utc_day,       //!< Universal Time Coordinated  [1-31 days]
    const unsigned char      utc_hour,      //!< Universal Time Coordinated  [hours]
    const unsigned char      utc_minute,    //!< Universal Time Coordinated  [minutes]
    const float              utc_seconds    //!< Universal Time Coordinated  [s]
 )
{
    unsigned char daysInMonth;
    int result;
    if( utc_month == 0 || utc_month > 12 )
    {
        GNSS_ERROR_MSG( "if( utc_month == 0 || utc_month > 12 )" );
        return 0;
    }
    result = TIMECONV_GetNumberOfDaysInMonth( utc_year, utc_month, &daysInMonth );
    if( result == 0 )
    {
        GNSS_ERROR_MSG( "TIMECONV_GetNumberOfDaysInMonth returned 0." );       
        return 0;
    }
    if( utc_day == 0 || utc_day > daysInMonth )
    {
        GNSS_ERROR_MSG( "if( utc_day == 0 || utc_day > daysInMonth )" );
        return 0;
    }
    if( utc_hour > 23 )
    {
        GNSS_ERROR_MSG( "if( utc_hour > 23 )" );
        return 0;
    }
    if( utc_minute > 59 )
    {
        GNSS_ERROR_MSG( "if( utc_minute > 59 )" );
        return 0;
    }
    if( utc_seconds > 60 )
    {
        GNSS_ERROR_MSG( "if( utc_seconds > 60 )" );
        return 0;
    }

    return 1;
}


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
    )
{
    int result;
    double timebuffer_time_in_days;
    double timebuffer_time_in_seconds;
    //char *timeline; // for debugging

#ifdef _WIN32

    struct _timeb timebuffer; // found in <sys/timeb.h>   
    _ftime( &timebuffer );
    timebuffer_time_in_seconds = timebuffer.time + timebuffer.millitm * 1.0e-3; // [s] with ms resolution

#else

    struct timeval current_time;
    gettimeofday(&current_time, NULL);
    timebuffer_time_in_seconds = current_time.tv_sec + current_time.tv_usec * 1.0e-6; // [s] with us resolution

#endif 


    // timebuffer_time_in_seconds is the time in seconds since midnight (00:00:00), January 1, 1970, 
    // coordinated universal time (UTC). Julian date for (00:00:00), January 1, 1970 is: 2440587.5 [days]

    // convert timebuffer.time from seconds to days
    timebuffer_time_in_days = timebuffer_time_in_seconds/C_SECONDS_PER_DAY; // days since julian date 2440587.5000000 [days]

    // convert to julian date
    *julian_date = TIMECONV_JULIAN_DATE_START_OF_PC_TIME + timebuffer_time_in_days;

    result = TIMECONV_DetermineUTCOffset( *julian_date, utc_offset );
    if( result == 0 )
    {
        GNSS_ERROR_MSG( "TIMECONV_DetermineUTCOffset returned 0." );
        return 0;
    }

    result = TIMECONV_GetGPSTimeFromJulianDate(
        *julian_date,
        *utc_offset,
        gps_week,
        gps_tow );
    if( result == 0 )
    {
        GNSS_ERROR_MSG( "TIMECONV_GetGPSTimeFromJulianDate returned 0." );
        return 0;
    }

    result = TIMECONV_GetUTCTimeFromJulianDate(
        *julian_date,
        utc_year,
        utc_month,
        utc_day,
        utc_hour,
        utc_minute,
        utc_seconds );
    if( result == 0 )
    {
        GNSS_ERROR_MSG( "TIMECONV_GetUTCTimeFromJulianDate" );
        return 0;
    }

    return 1;
}


#ifdef _WIN32
int TIMECONV_SetSystemTime(
    const unsigned short  utc_year,     //!< Universal Time Coordinated    [year]
    const unsigned char   utc_month,    //!< Universal Time Coordinated    [1-12 months] 
    const unsigned char   utc_day,      //!< Universal Time Coordinated    [1-31 days]
    const unsigned char   utc_hour,     //!< Universal Time Coordinated    [hours]
    const unsigned char   utc_minute,   //!< Universal Time Coordinated    [minutes]
    const float           utc_seconds   //!< Universal Time Coordinated    [s]
    )
{
    int result;
    SYSTEMTIME t;
    double julian_date = 0;
    unsigned char day_of_week = 0;

    result = TIMECONV_GetJulianDateFromUTCTime(
        utc_year,
        utc_month,
        utc_day,
        utc_hour,
        utc_minute,
        utc_seconds,
        &julian_date
        );
    if( !result )
    {
        GNSS_ERROR_MSG( "TIMECONV_GetJulianDateFromUTCTime returned 0.");
        return 0;
    }

    result = TIMECONV_GetDayOfWeekFromJulianDate( julian_date, &day_of_week );
    if( !result )
    {
        GNSS_ERROR_MSG( "TIMECONV_GetDayOfWeekFromJulianDate returned 0.");
        return 0;
    }
    
    t.wDayOfWeek = day_of_week;
    t.wYear = utc_year;
    t.wMonth = utc_month;
    t.wDay = utc_day;
    t.wHour = utc_hour;
    t.wMinute = utc_minute;
    t.wSecond = (WORD)(floor(utc_seconds));
    t.wMilliseconds = (WORD)((utc_seconds - t.wSecond)*1000);

    // Set the PC system time.
    result = SetSystemTime( &t );
    
    return result;
}
#endif


int TIMECONV_GetDayOfWeekFromJulianDate(
    const double julian_date,   //!< Number of days since noon Universal Time Jan 1, 4713 BCE (Julian calendar) [days]
    unsigned char *day_of_week  //!< 0-Sunday, 1-Monday, 2-Tuesday, 3-Wednesday, 4-Thursday, 5-Friday, 6-Saturday [].
    )
{
    // "If the Julian date of noon is applied to the entire midnight-to-midnight civil 
    // day centered on that noon,[5] rounding Julian dates (fractional days) for the 
    // twelve hours before noon up while rounding those after noon down, then the remainder 
    // upon division by 7 represents the day of the week, with 0 representing Monday, 
    // 1 representing Tuesday, and so forth. Now at 17:48, Wednesday December 3 2008 (UTC) 
    // the nearest noon JDN is 2454804 yielding a remainder of 2." (http://en.wikipedia.org/wiki/Julian_day, 2008-12-03)
    int dow = 0;
    int jd = 0;

    if( julian_date - floor(julian_date) > 0.5 )
    {
        jd = (int)floor(julian_date+0.5);
    }
    else
    {
        jd = (int)floor(julian_date);
    }
    dow = jd%7; // 0 is monday, 1 is tuesday, etc

    switch( dow )
    {
        case 0: *day_of_week = 1; break;
        case 1: *day_of_week = 2; break;
        case 2: *day_of_week = 3; break;
        case 3: *day_of_week = 4; break;
        case 4: *day_of_week = 5; break;
        case 5: *day_of_week = 6; break;
        case 6: *day_of_week = 0; break;
        default: return 0; break;
    }
    
    return 1;
}


int TIMECONV_GetJulianDateFromGPSTime(
    const unsigned short    gps_week,      //!< GPS week (0-1024+)             [week]
    const double            gps_tow,       //!< GPS time of week (0-604800.0)  [s]
    const unsigned char     utc_offset,    //!< Integer seconds that GPS is ahead of UTC time, always positive [s]
    double*                 julian_date    //!< Number of days since noon Universal Time Jan 1, 4713 BCE (Julian calendar) [days]
    )
{   
    if( gps_tow < 0.0  || gps_tow > 604800.0 )
    {
        GNSS_ERROR_MSG( "if( gps_tow < 0.0  || gps_tow > 604800.0 )" );
        return 0;  
    }

    // GPS time is ahead of UTC time and Julian time by the UTC offset
    *julian_date = ((double)gps_week + (gps_tow-(double)utc_offset)/604800.0)*7.0 + TIMECONV_JULIAN_DATE_START_OF_GPS_TIME;  
    return 1;
}


int TIMECONV_GetJulianDateFromUTCTime(
    const unsigned short     utc_year,      //!< Universal Time Coordinated  [year]
    const unsigned char      utc_month,     //!< Universal Time Coordinated  [1-12 months] 
    const unsigned char      utc_day,       //!< Universal Time Coordinated  [1-31 days]
    const unsigned char      utc_hour,      //!< Universal Time Coordinated  [hours]
    const unsigned char      utc_minute,    //!< Universal Time Coordinated  [minutes]
    const float              utc_seconds,   //!< Universal Time Coordinated  [s]
    double*                  julian_date    //!< Number of days since noon Universal Time Jan 1, 4713 BCE (Julian calendar) [days]
    )
{   
    double y; // temp for year
    double m; // temp for month
    int result;

    // Check the input.
    result = TIMECONV_IsUTCTimeValid( utc_year, utc_month, utc_day, utc_hour, utc_minute, utc_seconds );
    if( result == 0 )
    {
        GNSS_ERROR_MSG( "TIMECONV_IsUTCTimeValid returned 0." );
        return 0;
    }

    if( utc_month <= 2 )
    {
        y = utc_year - 1;
        m = utc_month + 12;
    }
    else 
    {
        y = utc_year;
        m = utc_month;
    }

    *julian_date = (int)(365.25*y) + (int)(30.6001*(m+1.0)) + utc_day + utc_hour/24.0 + utc_minute/1440.0 + utc_seconds/86400.0 + 1720981.5;
    return 1;
}



int TIMECONV_GetGPSTimeFromJulianDate(
    const double            julian_date, //!< Number of days since noon Universal Time Jan 1, 4713 BCE (Julian calendar) [days]
    const unsigned char     utc_offset,  //!< Integer seconds that GPS is ahead of UTC time, always positive [s]
    unsigned short*         gps_week,    //!< GPS week (0-1024+)            [week]
    double*                 gps_tow      //!< GPS time of week [s]
    )
{
    // Check the input.
    if( julian_date < 0.0 )
    {
        GNSS_ERROR_MSG( "if( julian_date < 0.0 )" );
        return 0;
    }

    *gps_week = (unsigned short)((julian_date - TIMECONV_JULIAN_DATE_START_OF_GPS_TIME)/7.0); //

    *gps_tow   = (julian_date - TIMECONV_JULIAN_DATE_START_OF_GPS_TIME)*SECONDS_IN_DAY; // seconds since start of gps time [s]
    *gps_tow  -= (*gps_week)*C_SECONDS_PER_WEEK;                                  // seconds into the current week [s] 

    // however, GPS time is ahead of utc time by the UTC offset (and thus the Julian date as well)
    *gps_tow += utc_offset;
    if( *gps_tow > C_SECONDS_PER_WEEK )
    {
        *gps_tow  -= C_SECONDS_PER_WEEK;
        *gps_week += 1;
    }
    return 1;
}


int TIMECONV_GetUTCTimeFromJulianDate(
    const double        julian_date,  //!< Number of days since noon Universal Time Jan 1, 4713 BCE (Julian calendar) [days]
    unsigned short*     utc_year,     //!< Universal Time Coordinated    [year]
    unsigned char*      utc_month,    //!< Universal Time Coordinated    [1-12 months] 
    unsigned char*      utc_day,      //!< Universal Time Coordinated    [1-31 days]
    unsigned char*      utc_hour,     //!< Universal Time Coordinated    [hours]
    unsigned char*      utc_minute,   //!< Universal Time Coordinated    [minutes]
    float*              utc_seconds   //!< Universal Time Coordinated    [s]
    )
{
    int a, b, c, d, e; // temporary values
    
    unsigned short year;  
    unsigned char month;
    unsigned char day;
    unsigned char hour;
    unsigned char minute;        
    unsigned char days_in_month = 0;  
    double td; // temporary double
    double seconds;
    int result;

    // Check the input.
    if( julian_date < 0.0 )
    {
        GNSS_ERROR_MSG( "if( julian_date < 0.0 )" );
        return 0;
    }
    
    a = (int)(julian_date+0.5);
    b = a + 1537;
    c = (int)( ((double)b-122.1)/365.25 );
    d = (int)(365.25*c);
    e = (int)( ((double)(b-d))/30.6001 );
    
    td      = b - d - (int)(30.6001*e) + fmod( julian_date+0.5, 1.0 );   // [days]
    day     = (unsigned char)td;   
    td     -= day;
    td     *= 24.0;        // [hours]
    hour    = (unsigned char)td;
    td     -= hour;
    td     *= 60.0;        // [minutes]
    minute  = (unsigned char)td;
    td     -= minute;
    td     *= 60.0;        // [s]
    seconds = td;
    month   = (unsigned char)(e - 1 - 12*(int)(e/14));
    year    = (unsigned short)(c - 4715 - (int)( (7.0+(double)month) / 10.0 ));
    
    // check for rollover issues
    if( seconds >= 60.0 )
    {
        seconds -= 60.0;
        minute++;
        if( minute >= 60 )
        {
            minute -= 60;
            hour++;
            if( hour >= 24 )
            {
                hour -= 24;
                day++;
                
                result = TIMECONV_GetNumberOfDaysInMonth( year, month, &days_in_month );
                if( result == 0 )
                {
                    GNSS_ERROR_MSG( "TIMECONV_GetNumberOfDaysInMonth returned 0." );
                    return 0;
                }
                
                if( day > days_in_month )
                {
                    day = 1;
                    month++;
                    if( month > 12 )
                    {
                        month = 1;
                        year++;
                    }
                }
            }
        }
    }   
    
    *utc_year       = year;
    *utc_month      = month;
    *utc_day        = day;
    *utc_hour       = hour;
    *utc_minute     = minute;
    *utc_seconds    = (float)seconds;   

    return 1;
}

int TIMECONV_GetGPSTimeFromUTCTime(
    unsigned short     utc_year,     //!< Universal Time Coordinated    [year]
    unsigned char      utc_month,    //!< Universal Time Coordinated    [1-12 months] 
    unsigned char      utc_day,      //!< Universal Time Coordinated    [1-31 days]
    unsigned char      utc_hour,     //!< Universal Time Coordinated    [hours]
    unsigned char      utc_minute,   //!< Universal Time Coordinated    [minutes]
    float              utc_seconds,  //!< Universal Time Coordinated    [s]
    unsigned short*    gps_week,     //!< GPS week (0-1024+)            [week]
    double*            gps_tow       //!< GPS time of week (0-604800.0) [s]
    )
{
    double julian_date=0.0;
    unsigned char utc_offset=0;
    int result;

    // Check the input.
    result = TIMECONV_IsUTCTimeValid( utc_year, utc_month, utc_day, utc_hour, utc_minute, utc_seconds );
    if( result == 0 )
    {
        GNSS_ERROR_MSG( "TIMECONV_IsUTCTimeValid returned 0." );
        return 0;
    }

    result = TIMECONV_GetJulianDateFromUTCTime(
        utc_year,
        utc_month,
        utc_day,
        utc_hour,
        utc_minute,
        utc_seconds,
        &julian_date );
    if( result == 0 )
    {
        GNSS_ERROR_MSG( "TIMECONV_GetJulianDateFromUTCTime returned 0." );
        return 0;
    }

    result = TIMECONV_DetermineUTCOffset( julian_date, &utc_offset );
    if( result == 0 )
    {
        GNSS_ERROR_MSG( "TIMECONV_DetermineUTCOffset returned 0." );
        return 0;
    }

    result = TIMECONV_GetGPSTimeFromJulianDate(
        julian_date,
        utc_offset,
        gps_week,
        gps_tow );
    if( result == 0 )
    {
        GNSS_ERROR_MSG( "TIMECONV_GetGPSTimeFromJulianDate returned 0." );
        return 0;
    }

    return 1;
}



int TIMECONV_GetGPSTimeFromRinexTime(
    unsigned short     utc_year,     //!< Universal Time Coordinated    [year]
    unsigned char      utc_month,    //!< Universal Time Coordinated    [1-12 months] 
    unsigned char      utc_day,      //!< Universal Time Coordinated    [1-31 days]
    unsigned char      utc_hour,     //!< Universal Time Coordinated    [hours]
    unsigned char      utc_minute,   //!< Universal Time Coordinated    [minutes]
    float              utc_seconds,  //!< Universal Time Coordinated    [s]
    unsigned short*    gps_week,     //!< GPS week (0-1024+)            [week]
    double*            gps_tow       //!< GPS time of week (0-604800.0) [s]
    )
{
    double julian_date=0.0;
    unsigned char utc_offset=0;
    int result;

    // Check the input.
    result = TIMECONV_IsUTCTimeValid( utc_year, utc_month, utc_day, utc_hour, utc_minute, utc_seconds );
    if( result == 0 )
    {
        GNSS_ERROR_MSG( "TIMECONV_IsUTCTimeValid returned 0." );
        return 0;
    }

    result = TIMECONV_GetJulianDateFromUTCTime(
        utc_year,
        utc_month,
        utc_day,
        utc_hour,
        utc_minute,
        utc_seconds,
        &julian_date );
    if( result == 0 )
    {
        GNSS_ERROR_MSG( "TIMECONV_GetJulianDateFromUTCTime returned 0." );
        return 0;
    }

    result = TIMECONV_GetGPSTimeFromJulianDate(
        julian_date,
        utc_offset,
        gps_week,
        gps_tow );
    if( result == 0 )
    {
        GNSS_ERROR_MSG( "TIMECONV_GetGPSTimeFromJulianDate returned 0." );
        return 0;
    }

    return 1;
}

int TIMECONV_UTCTimeFromGPSTime(
    unsigned short     gps_week,     //!< GPS week (0-1024+)            [week]
    double             gps_tow,      //!< GPS time of week (0-604800.0) [s]
    unsigned short*    utc_year,     //!< Universal Time Coordinated    [year]
    unsigned char*     utc_month,    //!< Universal Time Coordinated    [1-12 months] 
    unsigned char*     utc_day,      //!< Universal Time Coordinated    [1-31 days]
    unsigned char*     utc_hour,     //!< Universal Time Coordinated    [hours]
    unsigned char*     utc_minute,   //!< Universal Time Coordinated    [minutes]
    float*             utc_seconds   //!< Universal Time Coordinated    [s]
    )
{
    double julian_date = 0.0; 
    unsigned char utc_offset = 0;
    int i;
    int result;

    if( gps_tow < 0.0 || gps_tow > 604800.0 )
    {
        GNSS_ERROR_MSG( "if( gps_tow < 0.0 || gps_tow > 604800.0 )" );
        return 0;  
    }

    // iterate to get the right utc offset
    for( i = 0; i < 4; i++ )
    {
        result = TIMECONV_GetJulianDateFromGPSTime(
            gps_week,
            gps_tow,
            utc_offset,
            &julian_date );
        if( result == 0 )
        {
            GNSS_ERROR_MSG( "TIMECONV_GetJulianDateFromGPSTime returned 0." );
            return 0;
        }

        result = TIMECONV_DetermineUTCOffset( julian_date, &utc_offset );
        if( result == 0 )
        {
            GNSS_ERROR_MSG( "TIMECONV_DetermineUTCOffset returned 0." );
            return 0;
        }
    }

    result = TIMECONV_GetUTCTimeFromJulianDate(
        julian_date,
        utc_year,
        utc_month,
        utc_day,
        utc_hour,
        utc_minute,
        utc_seconds );
    if( result == 0 )
    {
        GNSS_ERROR_MSG( "TIMECONV_GetUTCTimeFromJulianDate returned 0." );
        return 0;
    }

    return 1;
}

int TIMECONV_DetermineUTCOffset(
    double julian_date,       //!< Number of days since noon Universal Time Jan 1, 4713 BCE (Julian calendar) [days]
    unsigned char* utc_offset //!< Integer seconds that GPS is ahead of UTC time, always positive             [s], obtained from a look up table
    )
{
    if( julian_date < 0.0 )
    {
        GNSS_ERROR_MSG( "if( julian_date < 0.0 )" );
        return 0;
    }

    if(      julian_date < 2444786.5000 ) *utc_offset = 0;
    else if( julian_date < 2445151.5000 ) *utc_offset = 1;
    else if( julian_date < 2445516.5000 ) *utc_offset = 2;
    else if( julian_date < 2446247.5000 ) *utc_offset = 3;
    else if( julian_date < 2447161.5000 ) *utc_offset = 4;
    else if( julian_date < 2447892.5000 ) *utc_offset = 5;
    else if( julian_date < 2448257.5000 ) *utc_offset = 6;
    else if( julian_date < 2448804.5000 ) *utc_offset = 7;
    else if( julian_date < 2449169.5000 ) *utc_offset = 8;
    else if( julian_date < 2449534.5000 ) *utc_offset = 9;
    else if( julian_date < 2450083.5000 ) *utc_offset = 10;
    else if( julian_date < 2450630.5000 ) *utc_offset = 11;
    else if( julian_date < 2451179.5000 ) *utc_offset = 12;  
    else if( julian_date < 2453736.5000 ) *utc_offset = 13;  
    else                                  *utc_offset = 14;

    return 1;
}  


    

int TIMECONV_GetNumberOfDaysInMonth(
    const unsigned short year,        //!< Universal Time Coordinated    [year]
    const unsigned char month,        //!< Universal Time Coordinated    [1-12 months] 
    unsigned char* days_in_month      //!< Days in the specified month   [1-28|29|30|31 days]
    )
{
    int is_a_leapyear;
    unsigned char utmp = 0;
    
    is_a_leapyear = TIMECONV_IsALeapYear( year );
    
    switch(month)
    {
    case  1: utmp = TIMECONV_DAYS_IN_JAN; break;
    case  2: if( is_a_leapyear ){ utmp = 29; }else{ utmp = 28; }break;    
    case  3: utmp = TIMECONV_DAYS_IN_MAR; break;
    case  4: utmp = TIMECONV_DAYS_IN_APR; break;
    case  5: utmp = TIMECONV_DAYS_IN_MAY; break;
    case  6: utmp = TIMECONV_DAYS_IN_JUN; break;
    case  7: utmp = TIMECONV_DAYS_IN_JUL; break;
    case  8: utmp = TIMECONV_DAYS_IN_AUG; break;
    case  9: utmp = TIMECONV_DAYS_IN_SEP; break;
    case 10: utmp = TIMECONV_DAYS_IN_OCT; break;
    case 11: utmp = TIMECONV_DAYS_IN_NOV; break;
    case 12: utmp = TIMECONV_DAYS_IN_DEC; break;
    default: 
        { 
            GNSS_ERROR_MSG( "unexpected default case." ); 
            return 0; 
            break;    
        }
    }
    
    *days_in_month = utmp;

    return 1;
}

    


int TIMECONV_IsALeapYear( const unsigned short year )
{
    int is_a_leap_year = 0;

    if( (year%4) == 0 )
    {
        is_a_leap_year = 1;
        if( (year%100) == 0 )
        {
            if( (year%400) == 0 )
            {
                is_a_leap_year = 1;
            }
            else
            {
                is_a_leap_year = 0;
            }
        }
    }
    if( is_a_leap_year )
    {
        return 1;
    }
    else
    {    
        return 0;
    }
}





int TIMECONV_GetDayOfYear(
    const unsigned short utc_year,    // Universal Time Coordinated           [year]
    const unsigned char  utc_month,   // Universal Time Coordinated           [1-12 months] 
    const unsigned char  utc_day,     // Universal Time Coordinated           [1-31 days]
    unsigned short*      dayofyear    // number of days into the year (1-366) [days]
    )
{
    unsigned char days_in_feb = 0;
    int result;
    result = TIMECONV_GetNumberOfDaysInMonth( utc_year, 2, &days_in_feb );
    if( result == 0 )
    {
        GNSS_ERROR_MSG( "TIMECONV_GetNumberOfDaysInMonth returned 0." );
        return 0;
    }

    switch( utc_month )
    {
    case  1: *dayofyear = utc_day; break;
    case  2: *dayofyear = (unsigned short)(TIMECONV_DAYS_IN_JAN               + utc_day); break;
    case  3: *dayofyear = (unsigned short)(TIMECONV_DAYS_IN_JAN + days_in_feb + utc_day); break;
    case  4: *dayofyear = (unsigned short)(62          + days_in_feb + utc_day); break;
    case  5: *dayofyear = (unsigned short)(92          + days_in_feb + utc_day); break;
    case  6: *dayofyear = (unsigned short)(123         + days_in_feb + utc_day); break;
    case  7: *dayofyear = (unsigned short)(153         + days_in_feb + utc_day); break;
    case  8: *dayofyear = (unsigned short)(184         + days_in_feb + utc_day); break;
    case  9: *dayofyear = (unsigned short)(215         + days_in_feb + utc_day); break;
    case 10: *dayofyear = (unsigned short)(245         + days_in_feb + utc_day); break;
    case 11: *dayofyear = (unsigned short)(276         + days_in_feb + utc_day); break;
    case 12: *dayofyear = (unsigned short)(306         + days_in_feb + utc_day); break;
    default: 
        {
            GNSS_ERROR_MSG( "unexpected default case." );
            return 0; 
            break;
        }
    }

    return 1;
}


int TIMECONV_GetGPSTimeFromYearAndDayOfYear(
    const unsigned short year,      // The year [year]
    const unsigned short dayofyear, // The number of days into the year (1-366) [days]
    unsigned short*      gps_week,  //!< GPS week (0-1024+)            [week]
    double*              gps_tow    //!< GPS time of week (0-604800.0) [s]
    )
{
    int result;
    double julian_date = 0;

    if( gps_week == NULL )
    {
        GNSS_ERROR_MSG( "if( gps_week == NULL )" );
        return 0;
    }
    if( gps_tow == NULL )
    {
        GNSS_ERROR_MSG( "if( gps_tow == NULL )" );
        return 0;
    }
    if( dayofyear > 366 )
    {
        GNSS_ERROR_MSG( "if( dayofyear > 366 )" );
        return 0;
    }

    result = TIMECONV_GetJulianDateFromUTCTime(
        year,
        1,
        1,
        0,
        0,
        0,
        &julian_date 
        );
    if( result == 0 )
    {
        GNSS_ERROR_MSG( "TIMECONV_GetJulianDateFromUTCTime returned 0." );
        return 0;
    }

    julian_date += dayofyear - 1; // at the start of the day so -1.

    result = TIMECONV_GetGPSTimeFromJulianDate(
        julian_date,
        0,
        gps_week,
        gps_tow );
    if( result == 0 )
    {
        GNSS_ERROR_MSG( "TIMECONV_GetGPSTimeFromJulianDate returned 0." );
        return 0;
    }

    return 1;
}

#endif
