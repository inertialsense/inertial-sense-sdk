import datetime
import numpy as np

# Set Reference LLA (deg, deg, m) used for NED - Provo, UT
refLla = np.r_[40.19815560, -111.6209952, 1373.021]
gpsWeek = 0
showUtcTime = 0

# GPS Epoch: January 6, 1980
GPS_EPOCH = datetime.datetime(1980, 1, 6, 0, 0, 0)

def gpsTowToUtc(gps_week, gps_tow, gps_utc_offset=18):
    """
    Convert GPS week and time of week in milliseconds to UTC time.
    
    Args:
        gps_week (array-like): GPS week numbers (e.g., list, numpy array, etc.).
        gps_tow_ms (array-like): GPS time of week in milliseconds (e.g., list, numpy array, etc.).
        gps_utc_offset (int): Difference between GPS time and UTC time in seconds. Default is 18.
    
    Returns:
        numpy.ndarray: Array of UTC times as datetime objects.
    """
    # Convert inputs to numpy arrays
    gps_week = np.asarray(gps_week)
    gps_tow = np.asarray(gps_tow)
        
    # Calculate total seconds from GPS epoch
    total_seconds = gps_week * 7 * 24 * 3600 + gps_tow
    
    # Calculate GPS times as timedelta from GPS epoch
    gps_times = np.array([GPS_EPOCH + datetime.timedelta(seconds=sec) for sec in total_seconds])
    
    # Subtract GPS-UTC offset to get UTC times
    utc_times = gps_times - np.array([datetime.timedelta(seconds=gps_utc_offset)] * len(gps_times))
    
    return utc_times

# Set Reference latitude, longitude, height above ellipsoid (deg, deg, m) used for NED calculations
def setRefLla(lla):
    global refLla
    refLla = lla

def setShowUtcTime(show):
    global showUtcTime
    showUtcTime = show

WEEK_TIME = []
def setGpsWeek(week):
    global gpsWeek
    global WEEK_TIME
    # Search for a valid GPS week
    size = np.shape(week)
    if size and size[0] > 1:
        week = np.max(week)
    if week > gpsWeek:
        gpsWeek = week

    GPS_start_Time = datetime.datetime.strptime('6/Jan/1980', "%d/%b/%Y")
    WEEK_TIME = GPS_start_Time + (datetime.timedelta(weeks=int(week)))

def getTimeFromGpsTowMs(ms, fixTimeBeforeGnss=False, week=None):
    global WEEK_TIME
    if len(ms) == 0:
        return ms
    if "WEEK_TIME" in locals():
        # GPS time available
        gpsTime = [WEEK_TIME + datetime.timedelta(milliseconds=int(i)) for i in ms]
    else:   
        # GPS time is NOT available
        if fixTimeBeforeGnss:
            # Backpropagate first available GNSS time to convert time since start to GNSS time
            dms = np.diff(ms)
            ind = np.flatnonzero(abs(dms - dms[-1]) > 0.1)
            if len(ind) > 0:
                for i in reversed(range(ind[-1]+1)):
                    ms[i] = ms[i+1] - dms[-1]
        gpsTime = ms * 0.001

    if showUtcTime:
        if week is None:
            week = gpsWeek
        return gpsTowToUtc(week, gpsTime)
    else:
        return gpsTime

def getTimeFromGpsTow(s, fixTimeBeforeGnss=False, week=None):
    global WEEK_TIME
    if len(s) == 0:
        return s
    if "WEEK_TIME" in locals():
        # GPS time available
        gpsTime = [WEEK_TIME + datetime.timedelta(seconds=float(i)) for i in s]
    else:
        if fixTimeBeforeGnss:
            # Back propagate first available GNSS time to convert time since start to GNSS time
            ds = np.diff(s)
            ind = np.flatnonzero(abs(ds - ds[-1]) > 10) # jump in time stamp due to GNSS ToW offset
            if len(ind) > 0:
                for i in reversed(range(ind[-1]+1)):
                    s[i] = s[i+1] - ds[-1]
        gpsTime = s
    
    if showUtcTime:
        if week is None:
            week = gpsWeek
        return gpsTowToUtc(week, gpsTime)
    else:
        return gpsTime

def getTimeFromGTime(gtime):
    GPS_start_Time = datetime.datetime.strptime('1/Jan/1970', "%d/%b/%Y")
    return [GPS_start_Time + datetime.timedelta(seconds=float(t['time'] + t['sec'])) for t in gtime]
