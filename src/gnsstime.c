/**
 * @file gnsstime.c
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

#include "gnsstime.h"

static double s_timeoffset = 0.0;

static const double gpst0[] = {1980, 1, 6, 0, 0, 0}; /* gps time reference */
static const double gst0[] = {1999, 8, 22, 0, 0, 0}; /* galileo system time reference */
static const double bdt0[] = {2006, 1, 1, 0, 0, 0};  /* beidou time reference */

static double leaps[MAXLEAPS + 1][7] = {/* leap seconds (y,m,d,h,m,s,utc-gpst) */
                                        {2017, 1, 1, 0, 0, 0, -18},
                                        {2015, 7, 1, 0, 0, 0, -17},
                                        {2012, 7, 1, 0, 0, 0, -16},
                                        {2009, 1, 1, 0, 0, 0, -15},
                                        {2006, 1, 1, 0, 0, 0, -14},
                                        {1999, 1, 1, 0, 0, 0, -13},
                                        {1997, 7, 1, 0, 0, 0, -12},
                                        {1996, 1, 1, 0, 0, 0, -11},
                                        {1994, 7, 1, 0, 0, 0, -10},
                                        {1993, 7, 1, 0, 0, 0, -9},
                                        {1992, 7, 1, 0, 0, 0, -8},
                                        {1991, 1, 1, 0, 0, 0, -7},
                                        {1990, 1, 1, 0, 0, 0, -6},
                                        {1988, 1, 1, 0, 0, 0, -5},
                                        {1985, 7, 1, 0, 0, 0, -4},
                                        {1983, 7, 1, 0, 0, 0, -3},
                                        {1982, 7, 1, 0, 0, 0, -2},
                                        {1981, 7, 1, 0, 0, 0, -1},
                                        {0}};

int str2time(const char *s, int i, int n, gtime_t *t)
{
    double ep[6];
    char str[256], *p = str;

    if (i < 0 || (int)strlen(s) < i || (int)sizeof(str) - 1 < i)
        return -1;
    for (s += i; *s && --n >= 0;)
        *p++ = *s++;
    *p = '\0';
    if (sscanf(str, "%lf %lf %lf %lf %lf %lf", ep, ep + 1, ep + 2, ep + 3, ep + 4, ep + 5) < 6)
        return -1;
    if (ep[0] < 100.0)
        ep[0] += ep[0] < 80.0 ? 2000.0 : 1900.0;
    *t = epoch2time(ep);
    return 0;
}

gtime_t epoch2time(const double *ep)
{
    const int doy[] = {1, 32, 60, 91, 121, 152, 182, 213, 244, 274, 305, 335};
    gtime_t time = {0};
    int days, sec, year = (int)ep[0], mon = (int)ep[1], day = (int)ep[2];

    if (year < 1970 || 2099 < year || mon < 1 || 12 < mon)
        return time;

    /* leap year if year%4==0 in 1901-2099 */
    days = (year - 1970) * 365 + (year - 1969) / 4 + doy[mon - 1] + day - 2 + (year % 4 == 0 && mon >= 3 ? 1 : 0);
    sec = (int)floor(ep[5]);
    time.time = (int64_t)days * 86400 + (int)ep[3] * 3600 + (int)ep[4] * 60 + sec;
    time.sec = ep[5] - sec;
    return time;
}

void time2epoch(gtime_t t, double *ep)
{
    const int mday[] = {/* # of days in a month */
                        31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31, 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31,
                        31, 29, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31, 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
    int days, sec, mon, day;

    /* leap year if year%4==0 in 1901-2099 */
    days = (int)(t.time / 86400);
    sec = (int)(t.time - (int64_t)days * 86400);
    for (day = days % 1461, mon = 0; mon < 48; mon++)
    {
        if (day >= mday[mon])
            day -= mday[mon];
        else
            break;
    }
    ep[0] = 1970 + days / 1461 * 4 + mon / 12;
    ep[1] = mon % 12 + 1;
    ep[2] = day + 1;
    ep[3] = sec / 3600;
    ep[4] = sec % 3600 / 60;
    ep[5] = sec % 60 + t.sec;
}

void time2epoch_n(gtime_t t, double *ep, int n)
{
    if (n < 0)
        n = 0;
    else if (n > 12)
        n = 12;
    if (1.0 - t.sec < 0.5 / pow(10.0, n))
    {
        t.time++;
        t.sec = 0.0;
    };
    time2epoch(t, ep);
}

gtime_t gpst2time(int week, double sec)
{
    gtime_t t = epoch2time(gpst0);

    if (sec < -1E9 || 1E9 < sec)
        sec = 0.0;
    t.time += (int64_t)86400 * 7 * week + (int)sec;
    t.sec = sec - (int)sec;
    return t;
}

double time2gpst(gtime_t t, int *week)
{
    gtime_t t0 = epoch2time(gpst0);
    int64_t sec = t.time - t0.time;
    int w = (int)(sec / (86400 * 7));

    if (week)
        *week = w;
    return (double)(sec - (double)w * 86400 * 7) + t.sec;
}

gtime_t gst2time(int week, double sec)
{
    gtime_t t = epoch2time(gst0);

    if (sec < -1E9 || 1E9 < sec)
        sec = 0.0;
    t.time += (int64_t)86400 * 7 * week + (int)sec;
    t.sec = sec - (int)sec;
    return t;
}

double time2gst(gtime_t t, int *week)
{
    gtime_t t0 = epoch2time(gst0);
    int64_t sec = t.time - t0.time;
    int w = (int)(sec / (86400 * 7));

    if (week)
        *week = w;
    return (double)(sec - (double)w * 86400 * 7) + t.sec;
}

gtime_t bdt2time(int week, double sec)
{
    gtime_t t = epoch2time(bdt0);

    if (sec < -1E9 || 1E9 < sec)
        sec = 0.0;
    t.time += (int64_t)86400 * 7 * week + (int)sec;
    t.sec = sec - (int)sec;
    return t;
}

double time2bdt(gtime_t t, int *week)
{
    gtime_t t0 = epoch2time(bdt0);
    int64_t sec = t.time - t0.time;
    int w = (int)(sec / (86400 * 7));

    if (week)
        *week = w;
    return (double)(sec - (double)w * 86400 * 7) + t.sec;
}

gtime_t timeadd(gtime_t t, double sec)
{
    double tt;

    t.sec += sec;
    tt = floor(t.sec);
    t.time += (int)tt;
    t.sec -= tt;
    return t;
}

double timediff(gtime_t t1, gtime_t t2)
{
    return difftime(t1.time, t2.time) + t1.sec - t2.sec;
}

gtime_t timeget(void)
{
    gtime_t time;
    double ep[6] = {0};
#ifdef WIN32
    SYSTEMTIME ts;

    GetSystemTime(&ts); /* utc */
    ep[0] = ts.wYear;
    ep[1] = ts.wMonth;
    ep[2] = ts.wDay;
    ep[3] = ts.wHour;
    ep[4] = ts.wMinute;
    ep[5] = ts.wSecond + ts.wMilliseconds * 1E-3;
#elif defined(RTK_EMBEDDED)
    // TODO: Implement time
#else
    struct timeval tv;
    struct tm *tt;

    if (!gettimeofday(&tv, NULL) && (tt = gmtime(&tv.tv_sec)))
    {
        ep[0] = tt->tm_year + 1900;
        ep[1] = tt->tm_mon + 1;
        ep[2] = tt->tm_mday;
        ep[3] = tt->tm_hour;
        ep[4] = tt->tm_min;
        ep[5] = tt->tm_sec + tv.tv_usec * 1E-6;
    }
#endif
    time = epoch2time(ep);

#ifdef CPUTIME_IN_GPST /* cputime operated in gpst */
    time = gpst2utc(time);
#endif
    return timeadd(time, s_timeoffset);
}

void timeset(gtime_t t)
{
    s_timeoffset += timediff(t, timeget());
}

void timereset(void)
{
    s_timeoffset = 0.0;
}

gtime_t gpst2utc(gtime_t t)
{
    gtime_t tu;
    int i;

    for (i = 0; leaps[i][0] > 0; i++)
    {
        tu = timeadd(t, leaps[i][6]);
        if (timediff(tu, epoch2time(leaps[i])) >= 0.0)
            return tu;
    }
    return t;
}

gtime_t utc2gpst(gtime_t t)
{
    int i;

    for (i = 0; leaps[i][0] > 0; i++)
    {
        if (timediff(t, epoch2time(leaps[i])) >= 0.0)
            return timeadd(t, -leaps[i][6]);
    }
    return t;
}

gtime_t gpst2bdt(gtime_t t)
{
    return timeadd(t, -14.0);
}

gtime_t bdt2gpst(gtime_t t)
{
    return timeadd(t, 14.0);
}

static double time2sec(gtime_t time, gtime_t *day)
{
    double ep[6], sec;
    time2epoch(time, ep);
    sec = ep[3] * 3600.0 + ep[4] * 60.0 + ep[5];
    ep[3] = ep[4] = ep[5] = 0.0;
    *day = epoch2time(ep);
    return sec;
}

double utc2gmst(gtime_t t, double ut1_utc)
{
    const double ep2000[] = {2000, 1, 1, 12, 0, 0};
    gtime_t tut, tut0;
    double ut, t1, t2, t3, gmst0, gmst;

    tut = timeadd(t, ut1_utc);
    ut = time2sec(tut, &tut0);
    t1 = timediff(tut0, epoch2time(ep2000)) / 86400.0 / 36525.0;
    t2 = t1 * t1;
    t3 = t2 * t1;
    gmst0 = 24110.54841 + 8640184.812866 * t1 + 0.093104 * t2 - 6.2E-6 * t3;
    gmst = gmst0 + 1.002737909350795 * ut;

    return fmod(gmst, 86400.0) * PI / 43200.0; /* 0 <= gmst <= 2*PI */
}

void time2str(gtime_t t, char *s, int n)
{
    double ep[6];

    if (n < 0)
        n = 0;
    else if (n > 12)
        n = 12;
    if (1.0 - t.sec < 0.5 / pow(10.0, n))
    {
        t.time++;
        t.sec = 0.0;
    };
    time2epoch(t, ep);
    sprintf(s, "%04.0f/%02.0f/%02.0f %02.0f:%02.0f:%0*.*f", ep[0], ep[1], ep[2],
            ep[3], ep[4], n <= 0 ? 2 : n + 3, n <= 0 ? 0 : n, ep[5]);
}

char *time_str(gtime_t t, int n)
{
    static char buff[64];
    time2str(t, buff, n);
    return buff;
}

double time2doy(gtime_t t)
{
    double ep[6];

    time2epoch(t, ep);
    ep[1] = ep[2] = 1.0;
    ep[3] = ep[4] = ep[5] = 0.0;
    return timediff(t, epoch2time(ep)) / 86400.0 + 1.0;
}

int adjgpsweek(int week)
{
    int w;
    (void)time2gpst(utc2gpst(timeget()), &w);
    if (w < 1560)
        w = 1560; /* use 2009/12/1 if time is earlier than 2009/12/1 */
    return week + (w - week + 1) / 1024 * 1024;
}

uint32_t tickget(void)
{
#ifdef WIN32
    return (uint32_t)timeGetTime();
#elif defined(RTK_EMBEDDED)
    // TODO: Implement time
    return 0;
#else
    struct timespec tp = {0};
    struct timeval tv = {0};

#ifdef CLOCK_MONOTONIC_RAW
    /* linux kernel > 2.6.28 */
    if (!clock_gettime(CLOCK_MONOTONIC_RAW, &tp))
    {
        return tp.tv_sec * 1000u + tp.tv_nsec / 1000000u;
    }
    else
    {
        gettimeofday(&tv, NULL);
        return tv.tv_sec * 1000u + tv.tv_usec / 1000u;
    }
#else
    gettimeofday(&tv, NULL);
    return tv.tv_sec * 1000u + tv.tv_usec / 1000u;
#endif
#endif /* WIN32 */
}

void sleepms(int ms)
{
#ifdef WIN32
    if (ms < 5)
        Sleep(1);
    else
        Sleep(ms);
#elif defined(RTK_EMBEDDED)
    // TODO: Implement time
#else
    struct timespec ts;
    if (ms <= 0)
        return;
    ts.tv_sec = (time_t)(ms / 1000);
    ts.tv_nsec = (long)(ms % 1000 * 1000000);
    nanosleep(&ts, NULL);
#endif
}
