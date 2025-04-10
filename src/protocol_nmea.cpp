#include <stdint.h>
#include <stdarg.h>
#include <cctype>
#include "protocol_nmea.h"
#include "time_conversion.h"
#include "ISPose.h"
#include "ISEarth.h"
#include "data_sets.h"
#include "util/md5.h"

#if defined(IMX_5)
#include "drivers/d_time.h"
#include "globals.h"
#endif

static int s_protocol_version = 0;
static uint8_t s_gnssId = SAT_SV_GNSS_ID_GNSS;

static struct  
{
    uint32_t 		timeOfWeekMs;
    ixVector3 		velNed;
    float			speed2dMps;
    float			speed2dKnots;
} s_dataSpeed;

uint8_t nmea2p3_svid_to_sigId(uint8_t gnssId, uint16_t svId);
bool gsv_freq_ena(gps_sig_sv_t* sig);

static gsvMask_t s_gsvMask = {0};

//////////////////////////////////////////////////////////////////////////
// Utility functions
//////////////////////////////////////////////////////////////////////////

void nmea_set_protocol_version(int protocol_version)
{ 
    s_protocol_version = protocol_version; 
}

void nmea_set_gnss_id(int gnssId)
{
    s_gnssId = gnssId;
}

// Safe snprintf that prevents use of invalid size.
// snprintf size (size_t) is unsigned and can wrap very large.
int ssnprintf(char buf[], int bufSize, const char *fmt, ...) 
{
    if (bufSize<=0) return 0;		// Prevent snprintf w/ invalid size 
    va_list args;
    va_start(args, fmt);
    int l = VSNPRINTF(buf, bufSize, fmt, args);
    va_end(args);
    return l;
}

/**
 * Sets Gsv filter. 
 *  
 * @param filter - filters for each constellation (uint8_t seedArr[SAT_SV_GNSS_ID_COUNT])
 * @param constellation - sets specific constellation (see eSatSvGnssId)
 */
void nmea_setGsvFilter(int constellation, uint8_t filter)
{
    if (constellation >= SAT_SV_GNSS_ID_GNSS && constellation < SAT_SV_GNSS_ID_COUNT)
        s_gsvMask.constMask[constellation] = filter;
}

/**
 * Sets Gsv filter. 
 *  
 * @param filter - filters for each constellation (uint8_t seedArr[SAT_SV_GNSS_ID_COUNT])
 */
void nmea_setGsvFilter(uint8_t* filters)
{
    for (int i = 0; i < SAT_SV_GNSS_ID_COUNT && filters != nullptr; i++)
    {
        s_gsvMask.constMask[i] = filters[i];
    }
}

/**
 * Gets Gsv filter. 
 * 
 * @param constellation - gets specific constellation (see eSatSvGnssId)
 * 
 * @return filter for requested constellation or -1 if invalid constellation is passed
 */
int nmea_getGsvFilter(int constellation)
{
    if (constellation >= SAT_SV_GNSS_ID_GNSS && constellation < SAT_SV_GNSS_ID_COUNT)
        return s_gsvMask.constMask[constellation];
    
    return -1;
}

/**
 * Gets Gsv filters. 
 * 
 * @param filters - filters for each constellation (uint8_t seedArr[SAT_SV_GNSS_ID_COUNT])
 */
void nmea_getGsvFilter(uint8_t* filters)
{
    for (int i = 0; i < SAT_SV_GNSS_ID_COUNT && filters != nullptr; i++)
    {
        filters[i] = s_gsvMask.constMask[i];
    }
}

void nmea_sprint(char buf[], int bufSize, int &offset, const char *fmt, ...) 
{
    bufSize -= offset;
    if (bufSize<=0) return;		// Prevent snprintf w/ invalid size 
    buf += offset;
    va_list args;
    va_start(args, fmt);
    offset += VSNPRINTF(buf, bufSize, fmt, args);
    va_end(args);
}

void nmea_print_u32(char buf[], int bufSize, int &offset, int precision, uint32_t value)
{
    bufSize -= offset;
    if (bufSize<=0) return;		// Prevent snprintf w/ invalid size
    buf += offset;

    if (value)
    {	// Non-zero
        offset += ssnprintf(buf, bufSize, ",%0*u", precision, value);
    }
    else
    {	// Print nothing for Zero
        buf[0] = ',';
        offset++;
    }
}

void nmea_print_i32(char buf[], int bufSize, int &offset, int precision, uint32_t value)
{
    bufSize -= offset;
    if (bufSize<=0) return;		// Prevent snprintf w/ invalid size
    buf += offset;

    if (value)
    {	// Non-zero
        offset += ssnprintf(buf, bufSize, ",%0*ld", precision, value);
    }
    else
    {	// Print nothing for Zero
        buf[0] = ',';
        offset++;
    }
}

uint32_t nmea_compute_checksum(uint8_t* str, int size)
{
    uint32_t checksum = 0;
    
    uint8_t *end = str + size;
    for(uint8_t *ptr=str; ptr<end; ptr++)
    {
        checksum ^= *ptr;
    }

    return checksum;
}

static int gnssId_to_talkerId(char* a, uint8_t gnssId)
{
    a[0] = 'G';

    switch (gnssId)
    {
    case SAT_SV_GNSS_ID_GPS:
    case SAT_SV_GNSS_ID_SBS:
        a[1] = 'P';
        break;
    case SAT_SV_GNSS_ID_GAL:
        a[1] = 'A';
        break;
    case SAT_SV_GNSS_ID_BEI:
        a[1] = 'B';
        break;
    case SAT_SV_GNSS_ID_QZS:
    case SAT_SV_GNSS_ID_IME:
        a[1] = 'Q';
        break;
    case SAT_SV_GNSS_ID_GLO:
        a[1] = 'L';
        break;
    case SAT_SV_GNSS_ID_IRN:
        a[1] = 'I';
        break;
    default:
        a[1] = 'N';
        break;
    }

    return 2;
}

void talkerId_to_gnssId(const char a[], uint8_t &gnssId, uint16_t &svId, uint8_t &sigId)
{
    uint16_t svIdLast = svId;
    if (s_protocol_version < NMEA_PROTOCOL_4P10)
    {
        if (svId >= 512)
        {	// < NMEA 4.10 method of detecting mult-frequency
            svId -= 512;
        }
        else if (svId >= 256)
        {	// < NMEA 4.10 method of detecting mult-frequency
            svId -= 256;
        }
        else
        {
            sigId = 0;
        }
    }

    switch(a[2])	// $Gx
    {
    case 'P':		// GPS, SBAS, QZSS
        if (svId >= 193)
        {
            gnssId = SAT_SV_GNSS_ID_QZS;
            if (svId >= 193){ svId -= 192; }
        }
        else if (svId > 32)
        {
            gnssId = SAT_SV_GNSS_ID_SBS;
            if (svId <= 64){ svId += 87; }
        }
        else{ gnssId = SAT_SV_GNSS_ID_GPS; }		break;
    case 'A':	gnssId = SAT_SV_GNSS_ID_GAL;		break;
    case 'B':	gnssId = SAT_SV_GNSS_ID_BEI;		break;	
    case 'L':	gnssId = SAT_SV_GNSS_ID_GLO; 		break;
    case 'I':	gnssId = SAT_SV_GNSS_ID_IRN; 		break;
    case 'Q':	gnssId = SAT_SV_GNSS_ID_QZS;		break;
    default:	gnssId = SAT_SV_GNSS_ID_UNKNOWN;	break;
    }

    sigId = nmea2p3_svid_to_sigId(gnssId, svIdLast);
}

static int nmea_talker(char* a, int aSize, uint8_t gnssId=s_gnssId)
{
    if (aSize < 2)
    {
        return 0;
    }
    a[0] = '$';
    return gnssId_to_talkerId(a+1, gnssId) + 1;
}

int nmea_sprint_footer(char* a, int aSize, int &n)
{
    unsigned int checkSum = nmea_compute_checksum((uint8_t*)(a+1), n-1);	
    n += ssnprintf(a+n, aSize-n, "*%.2X\r\n", checkSum);
    return n;
}

char *ASCII_to_u8(uint8_t *val, char *ptr)
{
    val[0] = (uint8_t)atoi(ptr);	ptr = ASCII_find_next_field(ptr);
    return ptr;
}

char *ASCII_to_u16(uint16_t *val, char *ptr)
{
    val[0] = (uint16_t)atoi(ptr);	ptr = ASCII_find_next_field(ptr);
    return ptr;
}

char *ASCII_to_u32(uint32_t *val, char *ptr)
{
    val[0] = (uint32_t)atoi(ptr);	ptr = ASCII_find_next_field(ptr);
    return ptr;
}

char *ASCII_to_u64(uint64_t *val, char *ptr)
{
    val[0] = (uint64_t)atol(ptr);	ptr = ASCII_find_next_field(ptr);
    return ptr;
}

char *ASCII_to_i32(int32_t *val, char *ptr)
{
    val[0] = (int32_t)atoi(ptr);	ptr = ASCII_find_next_field(ptr);
    return ptr;
}

char *ASCII_to_f32(float *vec, char *ptr)
{
    vec[0] = (float)atof(ptr);		ptr = ASCII_find_next_field(ptr);
    return ptr;
}

char *ASCII_to_f64(double *vec, char *ptr)
{
    vec[0] = atof(ptr);				ptr = ASCII_find_next_field(ptr);
    return ptr;
}

char *ASCII_to_ver4u8(uint8_t vec[], char *ptr)
{
    unsigned int v[4];
    SSCANF(ptr, "%u.%u.%u.%u", &v[0], &v[1], &v[2], &v[3]);
    vec[0] = (uint8_t)v[0];
    vec[1] = (uint8_t)v[1];
    vec[2] = (uint8_t)v[2];
    vec[3] = (uint8_t)v[3];
    ptr = ASCII_find_next_field(ptr);
    return ptr;
}

char *ASCII_to_vec3f(float vec[], char *ptr)
{
    vec[0] = (float)atof(ptr);		ptr = ASCII_find_next_field(ptr);
    vec[1] = (float)atof(ptr);		ptr = ASCII_find_next_field(ptr);
    vec[2] = (float)atof(ptr);		ptr = ASCII_find_next_field(ptr);
    return ptr;
}

char *ASCII_to_vec4f(float vec[], char *ptr)
{
    vec[0] = (float)atof(ptr);		ptr = ASCII_find_next_field(ptr);
    vec[1] = (float)atof(ptr);		ptr = ASCII_find_next_field(ptr);
    vec[2] = (float)atof(ptr);		ptr = ASCII_find_next_field(ptr);
    vec[3] = (float)atof(ptr);		ptr = ASCII_find_next_field(ptr);
    return ptr;
}

char *ASCII_to_vec3d(double vec[], char *ptr)
{
    vec[0] = atof(ptr);				ptr = ASCII_find_next_field(ptr);
    vec[1] = atof(ptr);				ptr = ASCII_find_next_field(ptr);
    vec[2] = atof(ptr);				ptr = ASCII_find_next_field(ptr);
    return ptr;
}

char *ASCII_to_MD5(uint32_t md5hash[4], char *ptr)
{
    md5_from_char_array(*(md5hash_t*)md5hash, ptr);
    ptr = ASCII_find_next_field(ptr);
    return ptr;
}

char *ASCII_DegMin_to_Lat(double *vec, char *ptr)
{
    int degrees;
    SSCANF(ptr, "%02d", &degrees);	ptr += 2;
    double minutes = atof(ptr);		ptr = ASCII_find_next_field(ptr);
    double decdegrees = ((double)degrees) + (minutes*0.01666666666666666666666666666666666);
    if (ptr[0] == 'S') 	{ vec[0] = -decdegrees; }	// south
    else 				{ vec[0] =  decdegrees; }	// north
    ptr += 2;

    return ptr;
}

char *ASCII_DegMin_to_Lon(double *vec, char *ptr)
{
    int degrees;
    SSCANF(ptr, "%03d", &degrees);	ptr += 3;
    double minutes = atof(ptr);		ptr = ASCII_find_next_field(ptr);
    double decdegrees = ((double)degrees) + (minutes*0.01666666666666666666666666666666666);
    if (ptr[0] == 'W') 	{ vec[0] = -decdegrees; }	// west
    else 				{ vec[0] =  decdegrees; }	// east
    ptr += 2;

    return ptr;
}

char *ASCII_to_char_array(char *dst, char *ptr, int max_len)
{
    char *ptr2 = ASCII_find_next_field(ptr);
    int len = _MIN(max_len, (int)(ptr2-ptr)) - 1;
    len = _MAX(0, len);		// prevent negative
    memcpy(dst, ptr, len);
    dst[len] = 0;			// Must be null terminated
    return ptr2;
}

char *ASCII_to_hours_minutes_seconds(int *hours, int *minutes, float *seconds, char *ptr)
{
    // HHMMSS.sss
#if 1
    SSCANF(ptr, "%02d%02d%f", hours, minutes, seconds);
#else
    double UTCtime = atof(ptr);
    *hours = ((int)UTCtime / 10000) % 100;
    *minutes = ((int)UTCtime / 100) % 100;
    float subSec = UTCtime - (int)UTCtime;
    *seconds = (float)((int)UTCtime % 100) + subSec;
#endif
    *seconds += 0.00005f;   // add a 0.05ms to address float-conversion aliasing
    ptr = ASCII_find_next_field(ptr);
    return ptr;
}

char *ASCII_UtcTimeToGpsTowMs(uint32_t *gpsTimeOfWeekMs, utc_time_t *utcTime, char *ptr, int utcWeekday, uint32_t leapS)
{
    // HHMMSS.sss
    float fsecond;
    SSCANF(ptr, "%02d%02d%f", &utcTime->hour, &utcTime->minute, &fsecond);
    fsecond += 0.00005f;	/// add a 0.05ms to address float-conversion aliasing
    utcTime->second = (uint32_t)fsecond;
    fsecond *= 1000.0f;
    utcTime->millisecond = (uint32_t)fsecond;
    utcTime->millisecond = utcTime->millisecond%1000;
    utcTimeToGpsTowMs(utcTime, utcWeekday, gpsTimeOfWeekMs, leapS);
    ptr = ASCII_find_next_field(ptr);
    return ptr;
}

char *ASCII_find_next_field(char *str)
{
    while(*str != 0 && *str != ',' && *str != '*') // move down looking for end of string.
    ++str;

    if(*str == ',') //move past comma (if not at end of string)
    ++str;

    return str;
}

double ddmm2deg(double ddmm)
{
    double deg = (int)ddmm / 100 ;
    ddmm -= deg * 100 ;
    return deg + (ddmm / 60) ;
}

void set_gpsPos_status_mask(uint32_t *status, uint32_t state, uint32_t mask)
{
    *status &= ~mask;
    *status |= state & mask;
}

void nmea_enable_stream(uint32_t& bits, uint8_t* period, uint32_t nmeaId, uint8_t periodMultiple)
{
    uint32_t nmeaBits = (1<<nmeaId);
    period[nmeaId] = periodMultiple;

    if (periodMultiple)
        bits |= (nmeaBits);
    else
        bits &= ~(nmeaBits);
}

//////////////////////////////////////////////////////////////////////////
// Binary to NMEA
//////////////////////////////////////////////////////////////////////////

int nmea_dev_info(char a[], const int aSize, dev_info_t &info)
{
    int n = ssnprintf(a, aSize, "$INFO"
        ",%d"                   // 1
        ",%d.%d.%d.%d"          // 2
        ",%d.%d.%d.%d"          // 3
        ",%d"                   // 4
        ",%d.%d.%d.%d"          // 5
        ",%d"                   // 6
        ",%s"                   // 7
        ",%04d-%02d-%02d"       // 8
        ",%02d:%02d:%02d.%02d"  // 9
        ",%s"                   // 10
        ",%d"                   // 11
        ",%d"                   // 12
        ",%c"                   // 13
        // TODO: dev_info_t.firmwareMD5Hash support
        // ",%08x%08x%08x%08x"     // 14
        , (int)info.serialNumber // 1
        , info.hardwareVer[0], info.hardwareVer[1], info.hardwareVer[2], info.hardwareVer[3] // 2
        , info.firmwareVer[0], info.firmwareVer[1], info.firmwareVer[2], info.firmwareVer[3] // 3
        , (int)info.buildNumber  // 4
        , info.protocolVer[0], info.protocolVer[1], info.protocolVer[2], info.protocolVer[3] // 5
        , (int)info.repoRevision // 6
        , info.manufacturer      // 7
        , info.buildYear+2000, info.buildMonth, info.buildDay // 8
        , info.buildHour, info.buildMinute, info.buildSecond, info.buildMillisecond // 9
        , info.addInfo           // 10
        , info.hardwareType          // 11
        , info.reserved          // 12
        , (info.buildType ? info.buildType : ' ') // 13
        // , info.firmwareMD5Hash[0], info.firmwareMD5Hash[1], info.firmwareMD5Hash[2], info.firmwareMD5Hash[3]	// 14
        );

    return nmea_sprint_footer(a, aSize, n);
}

/**
 * Generates NMEA ASCE request response
*/
int nmea_ASCE(char a[], const int aSize, rmcNmea_t* nRMC)
{
    nmeaBroadcastMsgPair_t pairs[MAX_nmeaBroadcastMsgPairs];
    int activeRMC = 0;

    for(int i = 0; (i < NMEA_MSG_ID_COUNT) && (activeRMC < MAX_nmeaBroadcastMsgPairs); i++)
    {
        if(((nRMC->nmeaBits & (0x01 << i)) != 0) && (nRMC->nmeaPeriod[i] > 0))
        {
            pairs[activeRMC].msgID = i;
            pairs[activeRMC].msgPeriod = nRMC->nmeaPeriod[i];
            activeRMC++;
        }
    }

    // Base msg with current port set
    int n = ssnprintf(a, aSize, "$ASCE,0");

    // finish populating msg
    for(int i = 0; (i < activeRMC) && (i < MAX_nmeaBroadcastMsgPairs); i++)
        n += ssnprintf(a+n, aSize-n, ",%d,%d", pairs[i].msgID, pairs[i].msgPeriod);

    return nmea_sprint_footer(a, aSize, n);
}


int tow_to_nmea_ptow(char a[], const int aSize, double imuTow, double insTow, unsigned int gpsWeek)
{
    int n = ssnprintf(a, aSize, "$PTOW");
    nmea_sprint(a, aSize, n, ",%.6lf", imuTow);			// 1
    nmea_sprint(a, aSize, n, ",%.6lf", insTow);			// 2
    nmea_sprint(a, aSize, n, ",%u", gpsWeek);			// 3	

    return nmea_sprint_footer(a, aSize, n);
}

int nmea_pimu(char a[], const int aSize, imu_t &imu, const char name[])
{
    int n = ssnprintf(a, aSize, "%s", name);
    nmea_sprint(a, aSize, n, ",%.3lf", imu.time);			// 1
    
    nmea_sprint(a, aSize, n, ",%.4f", imu.I.pqr[0]);		// 2
    nmea_sprint(a, aSize, n, ",%.4f", imu.I.pqr[1]);		// 3
    nmea_sprint(a, aSize, n, ",%.4f", imu.I.pqr[2]);		// 4

    nmea_sprint(a, aSize, n, ",%.3f", imu.I.acc[0]);		// 5
    nmea_sprint(a, aSize, n, ",%.3f", imu.I.acc[1]);		// 6
    nmea_sprint(a, aSize, n, ",%.3f", imu.I.acc[2]);		// 7
    
    return nmea_sprint_footer(a, aSize, n);
}

int nmea_ppimu(char a[], const int aSize, pimu_t &pimu)
{
    int n = ssnprintf(a, aSize, "$PPIMU");
    nmea_sprint(a, aSize, n, ",%.3lf", pimu.time);		// 1
    
    nmea_sprint(a, aSize, n, ",%.4f", pimu.theta[0]);	// 2
    nmea_sprint(a, aSize, n, ",%.4f", pimu.theta[1]);	// 3
    nmea_sprint(a, aSize, n, ",%.4f", pimu.theta[2]);	// 4

    nmea_sprint(a, aSize, n, ",%.4f", pimu.vel[0]);		// 5
    nmea_sprint(a, aSize, n, ",%.4f", pimu.vel[1]);		// 6
    nmea_sprint(a, aSize, n, ",%.4f", pimu.vel[2]);		// 7

    nmea_sprint(a, aSize, n, ",%.3f", pimu.dt);			// 8
    
    return nmea_sprint_footer(a, aSize, n);
}

int nmea_pins1(char a[], const int aSize, ins_1_t &ins1)
{
    int n = ssnprintf(a, aSize, "$PINS1");
    nmea_sprint(a, aSize, n, ",%.3lf", ins1.timeOfWeek);			// 1

    nmea_sprint(a, aSize, n, ",%u", (unsigned int)ins1.week);		// 2
    nmea_sprint(a, aSize, n, ",%u", (unsigned int)ins1.insStatus);	// 3
    nmea_sprint(a, aSize, n, ",%u", (unsigned int)ins1.hdwStatus);	// 4

    nmea_sprint(a, aSize, n, ",%.4f", ins1.theta[0]);				// 5
    nmea_sprint(a, aSize, n, ",%.4f", ins1.theta[1]);				// 6
    nmea_sprint(a, aSize, n, ",%.4f", ins1.theta[2]);				// 7

    nmea_sprint(a, aSize, n, ",%.3f", ins1.uvw[0]);					// 8
    nmea_sprint(a, aSize, n, ",%.3f", ins1.uvw[1]);					// 9
    nmea_sprint(a, aSize, n, ",%.3f", ins1.uvw[2]);					// 10

    nmea_sprint(a, aSize, n, ",%.8lf", ins1.lla[0]);				// 11
    nmea_sprint(a, aSize, n, ",%.8lf", ins1.lla[1]);				// 12
    nmea_sprint(a, aSize, n, ",%.3lf", ins1.lla[2]);				// 13

    nmea_sprint(a, aSize, n, ",%.3f", ins1.ned[0]);					// 14
    nmea_sprint(a, aSize, n, ",%.3f", ins1.ned[1]);					// 15
    nmea_sprint(a, aSize, n, ",%.3f", ins1.ned[2]);					// 16
    
    return nmea_sprint_footer(a, aSize, n);
}

int nmea_pins2(char a[], const int aSize, ins_2_t &ins2)
{
    int n = ssnprintf(a, aSize, "$PINS2");
    nmea_sprint(a, aSize, n, ",%.3lf", ins2.timeOfWeek);			// 1

    nmea_sprint(a, aSize, n, ",%u", (unsigned int)ins2.week);		// 2
    nmea_sprint(a, aSize, n, ",%u", (unsigned int)ins2.insStatus);	// 3
    nmea_sprint(a, aSize, n, ",%u", (unsigned int)ins2.hdwStatus);	// 4
    
    nmea_sprint(a, aSize, n, ",%.4f", ins2.qn2b[0]);				// 5
    nmea_sprint(a, aSize, n, ",%.4f", ins2.qn2b[1]);				// 6
    nmea_sprint(a, aSize, n, ",%.4f", ins2.qn2b[2]);				// 7
    nmea_sprint(a, aSize, n, ",%.4f", ins2.qn2b[3]);				// 8

    nmea_sprint(a, aSize, n, ",%.3f", ins2.uvw[0]);					// 9
    nmea_sprint(a, aSize, n, ",%.3f", ins2.uvw[1]);					// 10
    nmea_sprint(a, aSize, n, ",%.3f", ins2.uvw[2]);					// 11

    nmea_sprint(a, aSize, n, ",%.8lf", ins2.lla[0]);				// 12
    nmea_sprint(a, aSize, n, ",%.8lf", ins2.lla[1]);				// 13
    nmea_sprint(a, aSize, n, ",%.3lf", ins2.lla[2]);				// 14
    
    return nmea_sprint_footer(a, aSize, n);
}

int nmea_pstrb(char a[], const int aSize, strobe_in_time_t &strobe)
{
    int n = ssnprintf(a, aSize, "$PSTRB");
    nmea_sprint(a, aSize, n, ",%u", (unsigned int)strobe.week);			// 1
    nmea_sprint(a, aSize, n, ",%u", (unsigned int)strobe.timeOfWeekMs);	// 2
    nmea_sprint(a, aSize, n, ",%u", (unsigned int)strobe.pin);			// 3
    nmea_sprint(a, aSize, n, ",%u", (unsigned int)strobe.count);		// 4
    
    return nmea_sprint_footer(a, aSize, n);
}

int nmea_pgpsp(char a[], const int aSize, gps_pos_t &pos, gps_vel_t &vel)
{
    int n = ssnprintf(a, aSize, "$PGPSP");
    nmea_sprint(a, aSize, n, ",%u", (unsigned int)pos.timeOfWeekMs);	// 1
    nmea_sprint(a, aSize, n, ",%u", (unsigned int)pos.week);			// 2
    nmea_sprint(a, aSize, n, ",%u", (unsigned int)pos.status);			// 3

    nmea_sprint(a, aSize, n, ",%.8lf", pos.lla[0]);						// 4
    nmea_sprint(a, aSize, n, ",%.8lf", pos.lla[1]);						// 5
    nmea_sprint(a, aSize, n, ",%.2lf", pos.lla[2]);						// 6
    
    nmea_sprint(a, aSize, n, ",%.2f", pos.hMSL);						// 7
    nmea_sprint(a, aSize, n, ",%.2f", pos.pDop);						// 8
    nmea_sprint(a, aSize, n, ",%.2f", pos.hAcc);						// 9
    nmea_sprint(a, aSize, n, ",%.2f", pos.vAcc);						// 10

    nmea_sprint(a, aSize, n, ",%.2f", vel.vel[0]);						// 11
    nmea_sprint(a, aSize, n, ",%.2f", vel.vel[1]);						// 12
    nmea_sprint(a, aSize, n, ",%.2f", vel.vel[2]);						// 13
    nmea_sprint(a, aSize, n, ",%.2f", vel.sAcc);						// 14

    nmea_sprint(a, aSize, n, ",%.1f", pos.cnoMean);						// 15
    nmea_sprint(a, aSize, n, ",%.4lf", pos.towOffset);					// 16
    nmea_sprint(a, aSize, n, ",%u", (unsigned int)pos.leapS);			// 17
    
    return nmea_sprint_footer(a, aSize, n);
}

static void nmea_latToDegMin(char* a, int aSize, int &offset, double v)
{
    aSize -= offset;
    a += offset;

    // Prevent truncation error when rounding
    if (v < 0)
    {
        v -= 1.0E-10;
    }
    else
    {
        v += 1.0E-10;
    }
    int degrees = int(v);
    double minutes = (v-((double)degrees))*60.0;
    
    offset += ssnprintf(a, aSize, ",%02d%08.5lf,%c", abs(degrees), fabs(minutes), (v >= 0 ? 'N' : 'S'));
}

static void nmea_lonToDegMin(char* a, int aSize, int &offset, double v)
{
    aSize -= offset;
    a += offset;

    // Prevent truncation error when rounding
    if (v < 0)
    {
        v -= 1.0E-10;
    }
    else
    {
        v += 1.0E-10;
    }
    int degrees = int(v);
    double minutes = (v-((double)degrees))*60.0;
    
    offset += ssnprintf(a, aSize, ",%03d%08.5lf,%c", abs(degrees), fabs(minutes), (v >= 0 ? 'E' : 'W'));
}

static void nmea_GPSTimeToUTCTime(char* a, int aSize, int &offset, gps_pos_t &pos)
{
    aSize -= offset;
    a += offset;
    utc_time_t t;
    gpsTowMsToUtcTime(pos.timeOfWeekMs, pos.leapS, &t);
    
    offset += ssnprintf(a, aSize, ",%02u%02u%02u", t.hour, t.minute, t.second);
}

void nmea_GPSTimeToUTCTimeMsPrecision(char* a, int aSize, int &offset, gps_pos_t &pos)
{
    aSize -= offset;
    a += offset;
    utc_time_t t;
    gpsTowMsToUtcTime(pos.timeOfWeekMs, pos.leapS, &t);

    offset += ssnprintf(a, aSize, ",%02u%02u%02u.%03u", t.hour, t.minute, t.second, t.millisecond);
}

// TODO: Remove after ZDA issue is resolved.
#if defined(IMX_5) || defined(SDK_UNIT_TEST)
extern uint32_t g_cpu_msec;
extern sys_params_t g_sysParams;
extern debug_array_t g_debug;
#endif

// TODO: Remove after ZDA issue is resolved.
int millisecondsToSeconds(int milliseconds) 
{
    if (milliseconds >= 0)  { return (milliseconds + 500) / 1000; } 
    else                    { return (milliseconds - 500) / 1000; }
}

// TODO: Remove after ZDA issue is resolved.
void nmea_GPSTimeToUTCTimeMsPrecision_ZDA_debug(char* a, int aSize, int &offset, gps_pos_t &pos)
{
    aSize -= offset;
    a += offset;
    utc_time_t t;
    gpsTowMsToUtcTime(pos.timeOfWeekMs, pos.leapS, &t);

#if 0
// #if defined(IMX_5) || defined(SDK_UNIT_TEST)
    ///////////////////////////////////////////////////////////////////////
    // TODO: (WHJ) ZDA debug.  Remove after ZDA time skip issue is resolved. (SN-6066)

#if defined(IMX_5)
    int32_t cpuMs = (int32_t)time_msec();
#else
    int32_t cpuMs = (int32_t)g_cpu_msec;
#endif
    int32_t utcMs = 
        t.hour*C_MILLISECONDS_PER_HOUR + 
        t.minute*C_MILLISECONDS_PER_MINUTE + 
        t.second*C_MILLISECONDS_PER_SECOND + 
        t.millisecond;
    int32_t gpsMs = (int32_t)pos.timeOfWeekMs;

    g_debug.i[0] = cpuMs;
    g_debug.i[1] = utcMs;
    g_debug.i[2] = gpsMs;

    static int32_t lastCpuMs = cpuMs - 1000;
    static int32_t lastUtcMs = utcMs - 1000;
    static int lastUtcHour = t.hour;
    static int32_t utcOffsetSec = 0;

    // Check for irregular update timing
    int32_t cpuDtMs = cpuMs - lastCpuMs;
    bool cpuDtMsGood = _ABS(cpuDtMs) < 5000;

    // Check for skip in ZDA time
    int32_t utcDtMs = utcMs - lastUtcMs;
    bool utcDtMsGood = (t.hour >= lastUtcHour) && (_ABS(utcDtMs) < 5000); 

    // Ensure time increments linearly
    int32_t ddtMs = utcDtMs - cpuDtMs;
    if (cpuDtMsGood && utcDtMsGood)
    {   // No time wrap
        g_debug.i[3] = utcDtMs;
        // g_debug.i[4] = cpuDtMs;
        int adjOffsetSec = millisecondsToSeconds(ddtMs);
        if (adjOffsetSec)
        {
            utcOffsetSec += adjOffsetSec;

            g_sysParams.genFaultCode |= GFC_GNSS_RECEIVER_TIME;
#if PLATFORM_IS_EMBEDDED
            g_gnssTimeFaultTimeMs = g_timeMs;
#endif
            //g_debug.i[5] = utcMs;
            g_debug.f[5] = utcDtMs;
            g_debug.f[6] = cpuDtMs;
            g_debug.f[8] += 1.0f;
            if (_ABS(utcOffsetSec) > 2)
            {   // Offset exceeded limit
                utcOffsetSec = 0;
            }
        }
    }
    g_debug.f[7] = utcOffsetSec;

    // Update history
    lastCpuMs = cpuMs;
    lastUtcMs = utcMs;
    lastUtcHour = t.hour;

#if 0    // Apply correction offset
    if (utcOffsetSec)
    {
        t.second -= utcOffsetSec;
        if (t.second >= 60)
        {   // Wrap 
            t.second -= 60;
            t.minute++;
            if (t.minute >= 60)
            {   // Wrap
                t.minute -= 60;
                t.hour++;
            }
        }
        if (t.second < 0)
        {   // Wrap 
            t.second += 60;
            t.minute--;
            if (t.minute < 0)
            {   // Wrap
                t.minute += 60;
                t.hour--;
            }
        }
    }
#endif

    // TODO: (WHJ) End of debug section
    ///////////////////////////////////////////////////////////////////////
#endif

    offset += ssnprintf(a, aSize, ",%02u%02u%02u.%03u", t.hour, t.minute, t.second, t.millisecond);
}

static void nmea_GPSDateOfLastFix(char* a, int aSize, int &offset, gps_pos_t &pos)
{
    aSize -= offset;
    a += offset;
    double julian = gpsToJulian(pos.week, pos.timeOfWeekMs, pos.leapS);
    uint32_t year, month, day, hours, minutes, seconds, milliseconds;
    julianToDate(julian, &year, &month, &day, &hours, &minutes, &seconds, &milliseconds);
    
    offset += ssnprintf(a, aSize, ",%02u%02u%02u", (unsigned int)day, (unsigned int)month, (unsigned int)(year-2000));
}

static void nmea_GPSDateOfLastFixCSV(char* a, int aSize, int &offset, gps_pos_t &pos)	//Comma Separated Values
{
    aSize -= offset;
    a += offset;
    double julian = gpsToJulian(pos.week, pos.timeOfWeekMs, pos.leapS);
    uint32_t year, month, day, hours, minutes, seconds, milliseconds;
    julianToDate(julian, &year, &month, &day, &hours, &minutes, &seconds, &milliseconds);
    
    offset += ssnprintf(a, aSize, ",%02u,%02u,%04u", (unsigned int)day, (unsigned int)month, (unsigned int)year);
}

int nmea_gga(char a[], const int aSize, gps_pos_t &pos)
{
    int fixQuality = 0;
    switch((pos.status&GPS_STATUS_FIX_MASK))
    {
    default:
    case GPS_STATUS_FIX_NONE:                   fixQuality = 0;	break;
    case GPS_STATUS_FIX_SBAS:
    case GPS_STATUS_FIX_2D:
    case GPS_STATUS_FIX_RTK_SINGLE:
    case GPS_STATUS_FIX_3D:                     fixQuality = 1;	break;
    case GPS_STATUS_FIX_DGPS:                   fixQuality = 2;	break;
    case GPS_STATUS_FIX_TIME_ONLY:              fixQuality = 3;	break;   
    case GPS_STATUS_FIX_RTK_FIX:                fixQuality = 4;	break;
    case GPS_STATUS_FIX_RTK_FLOAT:              fixQuality = 5;	break;	
    case GPS_STATUS_FIX_DEAD_RECKONING_ONLY:
    case GPS_STATUS_FIX_GPS_PLUS_DEAD_RECK:     fixQuality = 6;	break;
    }
        
    // NMEA GGA line - http://www.gpsinformation.org/dale/nmea.htm#GGA
    /*
    GGA          Global Positioning System Fix Data
    123519       Fix taken at 12:35:19 UTC
    4807.038,N   Latitude 48 deg 07.038' N
    01131.000,E  Longitude 11 deg 31.000' E
    .            Fix quality:	0 = invalid
    .							1 = GPS fix (SPS)
    .							2 = DGPS fix
    .							3 = PPS fix
    .							4 = Real Time Kinematic
    .							5 = Float RTK
    .							6 = estimated (dead reckoning) (2.3 feature)
    .							7 = Manual input mode
    .							8 = Simulation mode
    08           Number of satellites being tracked
    0.9          Horizontal dilution of position
    545.4,M      MSL altitude in meters 
    46.9,M       HAE altitude (above geoid / WGS84 ellipsoid)
    ellipsoid
    (empty field) time in seconds since last DGPS update
    (empty field) DGPS station ID number
    *47          the checksum data, always begins with *
    */

    int n = nmea_talker(a, aSize);
    nmea_sprint(a, aSize, n, "GGA");
    nmea_GPSTimeToUTCTimeMsPrecision(a, aSize, n, pos);                                             // 1
    nmea_latToDegMin(a, aSize, n, pos.lla[0]);                                                      // 2,3
    nmea_lonToDegMin(a, aSize, n, pos.lla[1]);                                                      // 4,5
    nmea_sprint(a, aSize, n, ",%01u", (unsigned int)(fixQuality & 0xF));                            // 6 - GPS quality -- limit to available options (TODO: Overkill and probably unnecessary)
    nmea_sprint(a, aSize, n, ",%02u", (unsigned int)(pos.status&GPS_STATUS_NUM_SATS_USED_MASK));    // 7 - Satellites used
    nmea_sprint(a, aSize, n, ",%.2f", pos.pDop);                                                    // 8 - HDop
    nmea_sprint(a, aSize, n, ",%.2f,M", pos.hMSL);                                                  // 9,10 - MSL altitude
    nmea_sprint(a, aSize, n, ",%.2f,M", pos.lla[2] - pos.hMSL);                                     // 11,12 - Geoid separation
    nmea_sprint(a, aSize, n, ",,");                                                                 // 13,14 - Age of differential, DGPS station ID number
    return nmea_sprint_footer(a, aSize, n);
}

int nmea_gll(char a[], const int aSize, gps_pos_t &pos)
{
    // NMEA GLL line - http://www.gpsinformation.org/dale/nmea.htm#GLL
    /*
         GLL          Geographic position, Latitude and Longitude
         4916.46,N    Latitude 49 deg. 16.45 min. North
         12311.12,W   Longitude 123 deg. 11.12 min. West
         225444.800   Fix taken at 22:54:44.8 UTC
         A            Data Active or V (void)
         *iD          checksum data
    */

    int n = nmea_talker(a, aSize);
    nmea_sprint(a, aSize, n, "GLL");
    nmea_latToDegMin(a, aSize, n, pos.lla[0]);                      // 1,2
    nmea_lonToDegMin(a, aSize, n, pos.lla[1]);                      // 3,4
    nmea_GPSTimeToUTCTimeMsPrecision(a, aSize, n, pos);                        // 5
    nmea_sprint(a, aSize, n, ",A");                                 // 6
    return nmea_sprint_footer(a, aSize, n);
}

int nmea_gsa(char a[], const int aSize, gps_pos_t &pos, gps_sat_t &sat)
{
    int fixQuality;
    switch((pos.status&GPS_STATUS_FIX_MASK))
    {
    default:
        fixQuality = 0;	break;
    case GPS_STATUS_FIX_2D:					
        fixQuality = 2;	break;

    case GPS_STATUS_FIX_3D:					
    case GPS_STATUS_FIX_SBAS:
    case GPS_STATUS_FIX_DGPS:					
    case GPS_STATUS_FIX_RTK_FIX:
    case GPS_STATUS_FIX_RTK_SINGLE:
    case GPS_STATUS_FIX_RTK_FLOAT:				
        fixQuality = 3;	break;
    }
        
    // NMEA GSA line - http://www.gpsinformation.org/dale/nmea.htm#GSA
    /*
        eg1. $GPGSA,A,3,,,,,,16,18,,22,24,,,3.6,2.1,2.2*3C
        eg2. $GPGSA,A,3,19,28,14,18,27,22,31,39,,,,,1.7,1.0,1.3*35

        1    = Mode:
        .		M=Manual, forced to operate in 2D or 3D
        .		A=Automatic, 3D/2D
        2    = Mode:
        .		1=Fix not available
        .		2=2D
        .		3=3D
        3-14 = IDs of SVs used in position fix (null for unused fields)
        15   = PDOP
        16   = HDOP
        17   = VDOP
    */

    int n = nmea_talker(a, aSize);
    nmea_sprint(a, aSize, n, "GSA");
    nmea_sprint(a, aSize, n, ",A,%02u",	(unsigned int)fixQuality);		// 1,2
        
    for (uint32_t i = 0; i < 12; i++)									// 3-14
    {
        if(sat.sat[i].svId)
        {
            nmea_sprint(a, aSize, n, ",%02u", (unsigned)(sat.sat[i].svId));
        }
        else
        {
            nmea_sprint(a, aSize, n, ",");
        }
    }
        
    nmea_sprint(a, aSize, n,
        ",%.1f"		// 15
        ",%.1f"		// 16
        ",%.1f",	// 17
        pos.pDop,	// 15
        pos.hAcc,	// 16
        pos.vAcc);	// 17	

    return nmea_sprint_footer(a, aSize, n);
}

void update_nmea_speed(gps_pos_t &pos, gps_vel_t &vel)
{
    if (s_dataSpeed.timeOfWeekMs != pos.timeOfWeekMs)
    {
        s_dataSpeed.timeOfWeekMs = pos.timeOfWeekMs;

        if (vel.status & GPS_STATUS_FLAGS_GPS_NMEA_DATA)
        {	// NED velocity
            cpy_Vec3_Vec3(s_dataSpeed.velNed, vel.vel);
        }
        else
        {	// ECEF velocity
            ixQuat qe2n;
            quat_ecef2ned(C_DEG2RAD_F*(float)pos.lla[0], C_DEG2RAD_F*(float)pos.lla[1], qe2n);
            quatConjRot(s_dataSpeed.velNed, qe2n, vel.vel);
        }
        s_dataSpeed.speed2dMps = mag_Vec2(s_dataSpeed.velNed);
        s_dataSpeed.speed2dKnots = C_METERS_KNOTS_F * s_dataSpeed.speed2dMps;
    }
}

int nmea_rmc(char a[], const int aSize, gps_pos_t &pos, gps_vel_t &vel, float magDeclination)
{
    update_nmea_speed(pos, vel);

    int n = nmea_talker(a, aSize);
    nmea_sprint(a, aSize, n, "RMC");
    nmea_GPSTimeToUTCTime(a, aSize, n, pos);										// 1 - UTC time of last fix
    if((pos.status&GPS_STATUS_FIX_MASK)!=GPS_STATUS_FIX_NONE)
    {
        nmea_sprint(a, aSize, n, ",A");												// 2 - A=active (good)
    }
    else
    {
        nmea_sprint(a, aSize, n, ",V");												// 2 - V=void (bad,warning)
    }
    nmea_latToDegMin(a, aSize, n, pos.lla[0]);										// 3,4 - lat (degrees minutes)
    nmea_lonToDegMin(a, aSize, n, pos.lla[1]);										// 5,6 - lon (degrees minutes)
    
    float courseMadeTrue = atan2f(s_dataSpeed.velNed[1], s_dataSpeed.velNed[0]);
    nmea_sprint(a, aSize, n,
    ",%05.1f"		// 7
    ",%05.1f",		// 8
    s_dataSpeed.speed2dKnots,														// 7 - speed in knots
    courseMadeTrue*C_RAD2DEG_F);													// 8 - course made true
    
    nmea_GPSDateOfLastFix(a, aSize, n, pos);										// 9 - date of last fix UTC
    
    // Magnetic variation degrees (Easterly var. subtracts from true course), i.e. 020.3,E - left pad to 3 zero
    float magDec = magDeclination * C_RAD2DEG_F;
    bool positive = (magDec >= 0.0);
    
    nmea_sprint(a, aSize, n,
    ",%05.1f"	// 10
    ",%s",		// 11
    fabsf(magDec),																	// 10 - Magnetic variation
    (positive ? "E" : "W"));														// 11
    
    return nmea_sprint_footer(a, aSize, n);
}

int nmea_zda(char a[], const int aSize, gps_pos_t &pos)
{
    // NMEA ZDA line - http://www.gpsinformation.org/dale/nmea.htm#ZDA
    /*
        HHMMSS.sss    HrMinSec(UTC)
        dd,mm,yyy     Day,Month,Year
        xx            local zone hours -13..13 - Fixed field: 00
        yy            local zone minutes 0..59 - Fixed field: 00
        *CC           checksum
    */

    int n = nmea_talker(a, aSize);
    nmea_sprint(a, aSize, n, "ZDA");
    nmea_GPSTimeToUTCTimeMsPrecision_ZDA_debug(a, aSize, n, pos);								// 1
    nmea_GPSDateOfLastFixCSV(a, aSize, n, pos);										// 2,3,4
    nmea_sprint(a, aSize, n, ",00,00");												// 5,6
    
    return nmea_sprint_footer(a, aSize, n);
}

int nmea_vtg(char a[], const int aSize, gps_pos_t &pos, gps_vel_t &vel, float magVarCorrectionRad)
{
    /*
        0	Message ID $GPVTG
        1	Track made good (degrees true)
        2	T: track made good is relative to true north
        3	Track made good (degrees magnetic)
        4	M: track made good is relative to magnetic north 
        5	Speed, in knots
        6	N: speed is measured in knots
        7	Speed over ground in kilometers/hour (kph)
        8	K: speed over ground is measured in kph
        9	Mode indicator:
            A: Autonomous mode
            D: Differential mode
            E: Estimated (dead reckoning) mode
            M: Manual Input mode
            S: Simulator mode
            N: Data not valid
        10	The checksum data, always begins with *

        Example: $GPVTG,140.88,T,,M,8.04,N,14.89,K,D*05
    */
    update_nmea_speed(pos, vel);

    int n = nmea_talker(a, aSize);
    nmea_sprint(a, aSize, n, "VTG");
    float courseMadeTrue = atan2f(s_dataSpeed.velNed[1], s_dataSpeed.velNed[0]);
    nmea_sprint(a, aSize, n, ",%.2f", C_RAD2DEG_F * courseMadeTrue);				// 1
    nmea_sprint(a, aSize, n, ",T");													// 2
    if (magVarCorrectionRad == 0.0f)												// 3
    {
        nmea_sprint(a, aSize, n, ",");
    }
    else
    {
        nmea_sprint(a, aSize, n, ",%.2f", courseMadeTrue + magVarCorrectionRad*C_RAD2DEG_F);
    }
    nmea_sprint(a, aSize, n, ",M");													// 4
    nmea_sprint(a, aSize, n, ",%.2f", s_dataSpeed.speed2dKnots);					// 5
    nmea_sprint(a, aSize, n, ",N");													// 6
    nmea_sprint(a, aSize, n, ",%.2f", s_dataSpeed.speed2dMps*C_MPS2KMPH_F);			// 7
    nmea_sprint(a, aSize, n, ",K");													// 8
    switch(pos.status & GPS_STATUS_FIX_MASK)										// 9
    {
    case GPS_STATUS_FIX_2D:
    case GPS_STATUS_FIX_3D:
        nmea_sprint(a, aSize, n, ",A");
        break;
    case GPS_STATUS_FIX_GPS_PLUS_DEAD_RECK:
    case GPS_STATUS_FIX_DEAD_RECKONING_ONLY:
        nmea_sprint(a, aSize, n, ",E");
        break;
    case GPS_STATUS_FIX_DGPS:
    case GPS_STATUS_FIX_RTK_SINGLE:
    case GPS_STATUS_FIX_RTK_FLOAT:
    case GPS_STATUS_FIX_RTK_FIX:
        nmea_sprint(a, aSize, n, ",D");
        break;
    default:
        nmea_sprint(a, aSize, n, ",N");
        break;
    }
    return nmea_sprint_footer(a, aSize, n);
}

int nmea_pashr(char a[], const int aSize, gps_pos_t &pos, ins_1_t &ins1, float heave, inl2_ned_sigma_t &sigma)
{
    // NMEA PASHR - RT300 proprietary roll and pitch sentence
    /*
        hhmmss.sss - UTC time
        hhh.hh - Heading in degrees
        T - flag to indicate that the Heading is True Heading (i.e. to True North)
        rrr.rr - Roll Angle in degrees
        ppp.pp - Pitch Angle in degrees
        xxx.xx - Heave
        a.aaa - Roll Angle Accuracy Estimate (Stdev) in degrees
        b.bbb - Pitch Angle Accuracy Estimate (Stdev) in degrees
        c.ccc - Heading Angle Accuracy Estimate (Stdev) in degrees
        d - Aiding Status
        e - IMU Status
        hh - Checksum
    */
    
    int n = ssnprintf(a, aSize, "$PASHR");											// 1 - Name
    nmea_GPSTimeToUTCTimeMsPrecision(a, aSize, n, pos);										// 2 - UTC Time

    nmea_sprint(a, aSize, n, ",%.2f", RAD2DEG(ins1.theta[2]));						// 3 - Heading value in decimal degrees.
    nmea_sprint(a, aSize, n, ",T");													// 4 - T (heading respect to True North)
    nmea_sprint(a, aSize, n, ",%+.2f", RAD2DEG(ins1.theta[0]));						// 5 - Roll in degrees
    nmea_sprint(a, aSize, n, ",%+.2f", RAD2DEG(ins1.theta[1]));						// 6 - Pitch in degrees
    nmea_sprint(a, aSize, n, ",%+.2f", heave);										// 7 - Heave
    
    nmea_sprint(a, aSize, n, ",%.3f", RAD2DEG(sigma.StdAttNed[0])); 				// 8 - roll accuracy
    nmea_sprint(a, aSize, n, ",%.3f", RAD2DEG(sigma.StdAttNed[1])); 				// 9 - pitch accuracy
    nmea_sprint(a, aSize, n, ",%.3f", RAD2DEG(sigma.StdAttNed[2])); 				// 10 - heading accuracy
    
    int fix = 0;
    if(INS_STATUS_NAV_FIX_STATUS(ins1.insStatus) >= GPS_NAV_FIX_POSITIONING_RTK_FLOAT)
    {
        fix = 2;
    }
    else if(INS_STATUS_NAV_FIX_STATUS(ins1.insStatus) >= GPS_NAV_FIX_POSITIONING_3D)
    {
        fix = 1;
    }
    nmea_sprint(a, aSize, n, ",%d", fix);																// 11 - GPS Quality
    nmea_sprint(a, aSize, n, ",%d", INS_STATUS_SOLUTION(ins1.insStatus) >= INS_STATUS_SOLUTION_NAV); 	// 12 - INS Status
    
    return nmea_sprint_footer(a, aSize, n);
}

int nmea_intel(char a[], const int aSize, dev_info_t &info, gps_pos_t &pos, gps_vel_t &vel)
{
    /*  $INTEL prorietary NMEA message
        0	Message ID $INTEL
        1	Message ID KIM
        2	Fimrware version of KIM
        3	GPS Time of Week (seconds, no decimal)
        4	GPS week number
        5	GPS leap seconds
        6	1PPS phase 1 (ns)
        7	1PPS phase 2 (ns)
        8	Quantization error of time pulse (ns)
        9	ECEF X velocity (m/s)
        10	ECEF Y velocity (m/s)
        11	ECEF Z velocity (m/s)
        12	North veocity (m/s)
        13	East velocity (m/s)
        14	Down velocity (m/s)
        15	Checksum, begins with *

        Example: $INTEL, *05
    */
    update_nmea_speed(pos, vel);

    int n = ssnprintf(a, aSize, "$INTEL,KIM");										// 0,1

    nmea_sprint(a, aSize, n, ",%d.%d.%d.%d", 
        info.firmwareVer[0], 
        info.firmwareVer[1], 
        info.firmwareVer[2], 
        info.firmwareVer[3]);														// 2
    nmea_sprint(a, aSize, n, ",%d", pos.timeOfWeekMs/1000);							// 3
    nmea_sprint(a, aSize, n, ",%d", pos.week);										// 4
    nmea_sprint(a, aSize, n, ",%d", pos.leapS);										// 5

    nmea_sprint(a, aSize, n, ",%.3f", 0);											// 6
    nmea_sprint(a, aSize, n, ",%.3f", 0);											// 7
    nmea_sprint(a, aSize, n, ",0"); 												// 8

    nmea_sprint(a, aSize, n, ",%.3f", vel.vel[0]);									// 9
    nmea_sprint(a, aSize, n, ",%.3f", vel.vel[1]);									// 10
    nmea_sprint(a, aSize, n, ",%.3f", vel.vel[2]);									// 11

    nmea_sprint(a, aSize, n, ",%.3f", s_dataSpeed.velNed[0]);						// 12
    nmea_sprint(a, aSize, n, ",%.3f", s_dataSpeed.velNed[1]);						// 13
    nmea_sprint(a, aSize, n, ",%.3f", s_dataSpeed.velNed[2]);						// 14

    return nmea_sprint_footer(a, aSize, n);
}

/**
 * @brief Preps fields 1-6 of $POWxxx prorietary NMEA message
 * 
 * @param a[] - output buffer
 * @param startN - starting index in output buffer
 * @param aSize - size of output buffer
 * @param pos - gps position data
 * 
 * @note output message format: 
 *  1   GPS Time Quality (0=invalid, 1=valid)
 *  2   GPS Week Number
 *  3   GPS Time of Week (micro seconds)
 *  4   GPS leap seconds validity (0=invalid, 1=valid)
 *  5   GPS leap seconds
 *  6   Holdover flag (0=no holdover, 1=EGR is in holdover)
 */
int nmea_powPrep(char a[], int startN, const int aSize, gps_pos_t &pos)
{  
    int n = startN;
    int valid = (pos.week > 2359) ? 1 : 0; // assume time is valid if week > 2359 (03/23/2025)

    nmea_sprint(a, aSize, n, ",%d", valid);                 // 1
    nmea_sprint(a, aSize, n, ",%d", pos.week);              // 2
    nmea_sprint(a, aSize, n, ",%lu", ((uint64_t)pos.timeOfWeekMs)*1000); // 3 

    valid = (pos.leapS > 10 && pos.leapS < 30) ? 1 : 0;     // should be ~18 so give a little leeway
    nmea_sprint(a, aSize, n, ",%d", valid);                 // 4
    nmea_sprint(a, aSize, n, ",%d", pos.leapS);             // 5

    nmea_sprint(a, aSize, n, ",%d", 0);                     // 6

    return n;
}

/**
 * @brief creates $POWGPS prorietary NMEA message
 * 
 * @param a[] - output buffer
 * @param aSize - size of output buffer
 * @param pos - gps position data
 * 
 * @note output message format:
 *  0   Message ID $POWGPS
 *  1   GPS Time Quality (0=invalid, 1=valid)
 *  2   GPS Week Number
 *  3   GPS Time of Week (micro seconds)
 *  4   GPS leap seconds validity (0=invalid, 1=valid)
 *  5   GPS leap seconds
 *  6   Holdover flag (0=no holdover, 1=EGR is in holdover)
 *  7   Checksum, begins with *
 */
int nmea_powgps(char a[], const int aSize, gps_pos_t &pos)
{
    int n = ssnprintf(a, aSize, "$POWGPS");     // 0

    n = nmea_powPrep(a, n, aSize, pos);         // 1-6

    return nmea_sprint_footer(a, aSize, n);
}

/**
 * @brief $POWTLV prorietary NMEA message
 * 
 * @param a[] - output buffer
 * @param aSize - size of output buffer
 * @param pos - gps position data
 * @param vel - gps velocity data
 * 
 * @note output message format: 
 *  0   Message ID $POWGPS
 *  1   GPS Time Quality (0=invalid, 1=valid)
 *  2   GPS Week Number
 *  3   GPS Time of Week (micro seconds)
 *  4   GPS leap seconds validity (0=invalid, 1=valid)
 *  5   GPS leap seconds
 *  6   Holdover flag (0=no holdover, 1=EGR is in holdover)
 *  7   Latitude ddmm.mmmm
 *  8   North/South indicator (N/S)
 *  9   Longitude dddmm.mmmm
 *  10  East/West indicator (E/W)
 *  11  Altitude (x.xxx meters)
 *  12  Mean Sea Level (MSL) (x.xxx meters)
 *  13  Horizontal Speed (x.xxx m/s)
 *  14  Vertical Speed (x.xxx m/s)
 *  15  Heading (x.xxx degrees)
 *  16  Checksum, begins with *
 */
int nmea_powtlv(char a[], const int aSize, gps_pos_t &pos, gps_vel_t &vel)
{    
    float horVel = MAG_VEC2(vel.vel);
    float groundTrackHeading = 0;

    int n = ssnprintf(a, aSize, "$POWTLV");                     // 0
    
    update_nmea_speed(pos, vel);

    n = nmea_powPrep(a, n, aSize, pos);                         // 1-6

    nmea_latToDegMin(a, aSize, n, pos.lla[0]);                  // 7,8
    nmea_lonToDegMin(a, aSize, n, pos.lla[1]);                  // 9,10

    nmea_sprint(a, aSize, n, ",%.3f", pos.lla[2]);              // 11
    nmea_sprint(a, aSize, n, ",%.3f", pos.hMSL);                // 12

    nmea_sprint(a, aSize, n, ",%.3f", horVel);                  // 13

    nmea_sprint(a, aSize, n, ",%.3f", vel.vel[2]);              // 14

    groundTrackHeading = C_RAD2DEG_F * atan2f(vel.vel[1], vel.vel[0]);
    nmea_sprint(a, aSize, n, ",%.3f", groundTrackHeading);      // 15

    return nmea_sprint_footer(a, aSize, n);                     // 16
}

void print_string_n(char a[], int n)
{
    a[n] = '\0'; 
    printf("%s", a);
}

int prnToSvId(int gnssId, int prn)
{
    switch (gnssId)
    {
    case SAT_SV_GNSS_ID_SBS: return prn-87;
    }

    return prn;
}

bool gsv_sig_match(uint8_t gnssId, uint8_t sigId, gps_sig_sv_t &s, bool noCno=false)
{
    if ((s.cno==0) != noCno)
    {	// cno doesn't matches
        return false;
    }

    switch (gnssId)
    {
    case SAT_SV_GNSS_ID_GPS:
        if ((s.gnssId != gnssId) && (s.gnssId != SAT_SV_GNSS_ID_SBS))
        {
            return false;
        }
        break;

    case SAT_SV_GNSS_ID_SBS:
        return false;

    default:
        if (s.gnssId != gnssId)
        {
            return false;
        }
        break;
    }

    return (sigId == 0xFF) || (s.sigId == sigId);
}

int nmea_gsv_num_sat_sigs(uint8_t gnssId, uint8_t sigId, gps_sig_t &sig, bool noCno=false)
{
    int numSigs = 0;

    for (uint32_t i=0; i<sig.numSigs; i++)
    {
        gps_sig_sv_t &s = sig.sig[i];
        if (gsv_freq_ena(&sig.sig[i]) && gsv_sig_match(gnssId, sigId, s, noCno))
        {
            numSigs++;
        }
    }

    return numSigs;
}

uint8_t sigId_to_nmea4p11_signalId(uint8_t gnssId, uint8_t sigId)
{
    switch(gnssId)
    {
    case SAT_SV_GNSS_ID_GPS:
        switch(sigId)
        {
            case SAT_SV_SIG_ID_GPS_L1CA:		return '1';
            case SAT_SV_SIG_ID_GPS_L2CL:		return '6';
            case SAT_SV_SIG_ID_GPS_L2CM:		return '5';
            case SAT_SV_SIG_ID_GPS_L5I:			return '7';
            case SAT_SV_SIG_ID_GPS_L5Q:			return '8';
        }
        break;
    case SAT_SV_GNSS_ID_SBS:
        return 1;
    case SAT_SV_GNSS_ID_GAL:
        switch(sigId)
        {
            case SAT_SV_SIG_ID_Galileo_E1C2:
            case SAT_SV_SIG_ID_Galileo_E1B2:	return '7';
            case SAT_SV_SIG_ID_Galileo_E5aI:
            case SAT_SV_SIG_ID_Galileo_E5aQ:	return '1';
            case SAT_SV_SIG_ID_Galileo_E5bI:
            case SAT_SV_SIG_ID_Galileo_E5bQ:	return '2';
        }
        break;
    case SAT_SV_GNSS_ID_BEI:
        switch(sigId)
        {
            case SAT_SV_SIG_ID_BeiDou_B1D1:
            case SAT_SV_SIG_ID_BeiDou_B1D2:		return '1';
            case SAT_SV_SIG_ID_BeiDou_B2D1:
            case SAT_SV_SIG_ID_BeiDou_B2D2:		return 'B';
            case SAT_SV_SIG_ID_BeiDou_B1C:		return '3';
            case SAT_SV_SIG_ID_BeiDou_B2a:		return '5';
        }
        break;
    case SAT_SV_GNSS_ID_QZS:
        switch(sigId)
        {
            case SAT_SV_SIG_ID_QZSS_L1CA:		return '1';
            case SAT_SV_SIG_ID_QZSS_L1S:		return '4';
            case SAT_SV_SIG_ID_QZSS_L2CM:		return '5';
            case SAT_SV_SIG_ID_QZSS_L2CL:		return '6';
            case SAT_SV_SIG_ID_QZSS_L5I:		return '7';
            case SAT_SV_SIG_ID_QZSS_L5Q:		return '8';
        }
        break;
    case SAT_SV_GNSS_ID_GLO:
        switch(sigId)
        {
            case SAT_SV_SIG_ID_GLONASS_L1OF:	return '1';
            case SAT_SV_SIG_ID_GLONASS_L2OF:	return '3';
        }
        break;
    case SAT_SV_GNSS_ID_IRN:	// NavIC
        switch(sigId)
        {
            case SAT_SV_SIG_ID_NAVIC_L5A:		return '7';
        }
        break;
    }

    return '0';
}

uint8_t nmea4p11_signalId_to_sigId(uint8_t gnssId, char nmeaSignalId)
{
    switch(gnssId)
    {
    case SAT_SV_GNSS_ID_GPS:
        switch(nmeaSignalId)
        {
        case '1':	return SAT_SV_SIG_ID_GPS_L1CA;
        case '6':	return SAT_SV_SIG_ID_GPS_L2CL;
        case '5':	return SAT_SV_SIG_ID_GPS_L2CM;
        case '7':	return SAT_SV_SIG_ID_GPS_L5I;
        case '8':	return SAT_SV_SIG_ID_GPS_L5Q;
        }
        break;
    case SAT_SV_GNSS_ID_SBS:
        return 0;
    case SAT_SV_GNSS_ID_GAL:
        switch(nmeaSignalId)
        {
        case '7':	return SAT_SV_SIG_ID_Galileo_E1C2;
        // case '7':	return SAT_SV_SIG_ID_Galileo_E1B2;
        case '1':	return SAT_SV_SIG_ID_Galileo_E5aI;
        // case '1':	return SAT_SV_SIG_ID_Galileo_E5aQ;
        // case '2':	return SAT_SV_SIG_ID_Galileo_E5bI;
        case '2':	return SAT_SV_SIG_ID_Galileo_E5bQ;
        }
        break;
    case SAT_SV_GNSS_ID_BEI:
        switch(nmeaSignalId)
        {
        case '1':	return SAT_SV_SIG_ID_BeiDou_B1D1;
        // case '1': 	return SAT_SV_SIG_ID_BeiDou_B1D2;
        case 'B':	return SAT_SV_SIG_ID_BeiDou_B2D1;
        // case 'B':	return SAT_SV_SIG_ID_BeiDou_B2D2;
        case '3':	return SAT_SV_SIG_ID_BeiDou_B1C;
        case '5':	return SAT_SV_SIG_ID_BeiDou_B2a;
        }
        break;
    case SAT_SV_GNSS_ID_QZS:
        switch(nmeaSignalId)
        {
        case '1':	return SAT_SV_SIG_ID_QZSS_L1CA;
        case '4':	return SAT_SV_SIG_ID_QZSS_L1S;
        case '5':	return SAT_SV_SIG_ID_QZSS_L2CM;
        case '6':	return SAT_SV_SIG_ID_QZSS_L2CL;
        case '7':	return SAT_SV_SIG_ID_QZSS_L5I;
        case '8':	return SAT_SV_SIG_ID_QZSS_L5Q;
        }
        break;
    case SAT_SV_GNSS_ID_GLO:
        switch(nmeaSignalId)
        {
        case '1':	return SAT_SV_SIG_ID_GLONASS_L1OF;
        case '3':	return SAT_SV_SIG_ID_GLONASS_L2OF;
        }
        break;
    case SAT_SV_GNSS_ID_IRN:	// NavIC
        switch(nmeaSignalId)
        {
        case '7': 	return SAT_SV_SIG_ID_NAVIC_L5A;
        }
        break;
    }

    return '0';
}

uint16_t sigId_to_nmea2p3_svId(uint8_t gnssId, uint8_t sigId, uint16_t svId)
{
    switch(gnssId)
    {
    case SAT_SV_GNSS_ID_GPS:
        switch(sigId)
        {
        case SAT_SV_SIG_ID_GPS_L1CA:		return svId;
        case SAT_SV_SIG_ID_GPS_L2CL:		
        case SAT_SV_SIG_ID_GPS_L2CM:		return svId + 256;
        case SAT_SV_SIG_ID_GPS_L5I:			
        case SAT_SV_SIG_ID_GPS_L5Q:			return svId + 512;
        }
        break;
    case SAT_SV_GNSS_ID_SBS:
        return 1;
    case SAT_SV_GNSS_ID_GAL:
        switch(sigId)
        {
        case SAT_SV_SIG_ID_Galileo_E1C2:
        case SAT_SV_SIG_ID_Galileo_E1B2:	return svId;
        case SAT_SV_SIG_ID_Galileo_E5aI:
        case SAT_SV_SIG_ID_Galileo_E5aQ:	return svId + 256;
        case SAT_SV_SIG_ID_Galileo_E5bI:
        case SAT_SV_SIG_ID_Galileo_E5bQ:	return svId + 512;
        }
        break;
    case SAT_SV_GNSS_ID_BEI:
        switch(sigId)
        {
        case SAT_SV_SIG_ID_BeiDou_B1D1:
        case SAT_SV_SIG_ID_BeiDou_B1D2:		return svId;
        case SAT_SV_SIG_ID_BeiDou_B2D1:
        case SAT_SV_SIG_ID_BeiDou_B2D2:		return svId + 256;
        case SAT_SV_SIG_ID_BeiDou_B1C:		return svId;
        case SAT_SV_SIG_ID_BeiDou_B2a:		return svId + 512;
        }
        break;
    case SAT_SV_GNSS_ID_QZS:
        switch(sigId)
        {
        case SAT_SV_SIG_ID_QZSS_L1CA:
        case SAT_SV_SIG_ID_QZSS_L1S:		return svId;
        case SAT_SV_SIG_ID_QZSS_L2CM:
        case SAT_SV_SIG_ID_QZSS_L2CL:		return svId + 256;
        case SAT_SV_SIG_ID_QZSS_L5I:
        case SAT_SV_SIG_ID_QZSS_L5Q:		return svId + 512;
        }
        break;
    case SAT_SV_GNSS_ID_GLO:
        switch(sigId)
        {
        case SAT_SV_SIG_ID_GLONASS_L1OF:	return svId;
        case SAT_SV_SIG_ID_GLONASS_L2OF:	return svId + 256;
        }
        break;
    case SAT_SV_GNSS_ID_IRN:	// NavIC
        switch(sigId)
        {
        case SAT_SV_SIG_ID_NAVIC_L5A:		return svId + 512;
        }
        break;
    }

    return 0;
}

uint8_t nmea2p3_svid_to_sigId(uint8_t gnssId, uint16_t svId)
{
    if (svId<256)
    {	// L1/E1
        return 0;
    }

    if (svId>=512)
    {	// L5/E5 - most common band
        switch(gnssId)
        {
        case SAT_SV_GNSS_ID_GPS:	return SAT_SV_SIG_ID_GPS_L5Q;
        case SAT_SV_GNSS_ID_SBS:	return 0;
        case SAT_SV_GNSS_ID_GAL:	return SAT_SV_SIG_ID_Galileo_E5;
        case SAT_SV_GNSS_ID_BEI:	return SAT_SV_SIG_ID_BeiDou_B2a;
        case SAT_SV_GNSS_ID_QZS:	return SAT_SV_SIG_ID_QZSS_L5;
        case SAT_SV_GNSS_ID_GLO:	return SAT_SV_SIG_ID_GLONASS_L2OF;
        case SAT_SV_GNSS_ID_IRN:	return SAT_SV_SIG_ID_NAVIC_L5A;  // NavIC
        }
    }
    else
    {	// L2/E2 - most common band
        switch(gnssId)
        {
        case SAT_SV_GNSS_ID_GPS:	return SAT_SV_SIG_ID_GPS_L2CL;
        case SAT_SV_GNSS_ID_SBS:	return 0;
        case SAT_SV_GNSS_ID_GAL:	return SAT_SV_SIG_ID_Galileo_E5a;
        case SAT_SV_GNSS_ID_BEI:	return SAT_SV_SIG_ID_BeiDou_B2;
        case SAT_SV_GNSS_ID_QZS:	return SAT_SV_SIG_ID_QZSS_L2;
        case SAT_SV_GNSS_ID_GLO:	return SAT_SV_SIG_ID_GLONASS_L2OF;
        case SAT_SV_GNSS_ID_IRN:	return SAT_SV_SIG_ID_NAVIC_L5A;  // NavIC
        }
    }

    return 0;
}

/**
 * Gets GSV constellation mask for a given constellation ID
 * returns mask if value constellation passed and 0 if
 * an invalid constellation is passed
*/
uint8_t gsv_get_const_mask(uint8_t constellation)
{
    if (constellation < SAT_SV_GNSS_ID_COUNT)
    {
        return s_gsvMask.constMask[constellation];
    }

    return 0;
}

/**
 * Checks if for a given gps_sig_sv_t* sig 
 * the freqency accociated with sig is enabled
*/
bool gsv_freq_ena(gps_sig_sv_t* sig)
{
    if(sig->gnssId >= SAT_SV_GNSS_ID_COUNT)
        return false;

    uint8_t mask = s_gsvMask.constMask[sig->gnssId];

    switch(sig->gnssId)
    {
        case SAT_SV_GNSS_ID_GPS:
            switch(sig->sigId)
            {
                case SAT_SV_SIG_ID_GPS_L1CA:
                    return (bool)(mask & NMEA_GNGSV_FREQ_BAND1_BIT);
            
                case SAT_SV_SIG_ID_GPS_L2CL:
                case SAT_SV_SIG_ID_GPS_L2CM:
                    return (bool)(mask & NMEA_GNGSV_FREQ_BAND2_BIT);
                
                case SAT_SV_SIG_ID_GPS_L5I:
                case SAT_SV_SIG_ID_GPS_L5Q:
                    return (bool)(mask & NMEA_GNGSV_FREQ_5_BIT);

                default: return false;
            }
            
        case SAT_SV_GNSS_ID_SBS:
            switch(sig->sigId)
            {
                case SAT_SV_SIG_ID_SBAS_L1CA:
                    return (bool)(mask & NMEA_GNGSV_FREQ_BAND1_BIT);

                case SAT_SV_SIG_ID_SBAS_L2:
                    return (bool)(mask & NMEA_GNGSV_FREQ_BAND2_BIT);
                    
                case SAT_SV_SIG_ID_SBAS_L5:
                    return (bool)(mask & NMEA_GNGSV_FREQ_5_BIT);

                default: return false;
            }
            
        case SAT_SV_GNSS_ID_GAL:
            switch(sig->sigId)
            {
                case SAT_SV_SIG_ID_Galileo_E1C2:
                case SAT_SV_SIG_ID_Galileo_E1B2:
                    return (bool)(mask & NMEA_GNGSV_FREQ_BAND1_BIT);

                case SAT_SV_SIG_ID_Galileo_E5aI:
                case SAT_SV_SIG_ID_Galileo_E5aQ:
                case SAT_SV_SIG_ID_Galileo_E5bI:
                case SAT_SV_SIG_ID_Galileo_E5bQ:
                    return (bool)(mask & NMEA_GNGSV_FREQ_5_BIT);

                default: return false;
            }

        case SAT_SV_GNSS_ID_BEI:
            switch(sig->sigId)
            {
                case SAT_SV_SIG_ID_BeiDou_B1D1:
                case SAT_SV_SIG_ID_BeiDou_B1D2:
                case SAT_SV_SIG_ID_BeiDou_B1C:
                    return (bool)(mask & NMEA_GNGSV_FREQ_BAND1_BIT);

                case SAT_SV_SIG_ID_BeiDou_B2D1:
                case SAT_SV_SIG_ID_BeiDou_B2D2:
                case SAT_SV_SIG_ID_BeiDou_B2a:
                    return (bool)(mask & NMEA_GNGSV_FREQ_BAND2_BIT);

                default: return false;
            }

        case SAT_SV_GNSS_ID_QZS:
            switch(sig->sigId)
            {
                case SAT_SV_SIG_ID_QZSS_L1CA:
                case SAT_SV_SIG_ID_QZSS_L1S:
                    return (bool)(mask & NMEA_GNGSV_FREQ_BAND1_BIT);

                case SAT_SV_SIG_ID_QZSS_L2CM:
                case SAT_SV_SIG_ID_QZSS_L2CL:
                    return (bool)(mask & NMEA_GNGSV_FREQ_BAND2_BIT);
                    
                case SAT_SV_SIG_ID_QZSS_L5I:
                case SAT_SV_SIG_ID_QZSS_L5Q:
                    return (bool)(mask & NMEA_GNGSV_FREQ_5_BIT);

                default: return false;
            }
        
        case SAT_SV_GNSS_ID_GLO:
            switch(sig->sigId)
            {
                case SAT_SV_SIG_ID_GLONASS_L1OF:
                    return (bool)(mask & NMEA_GNGSV_FREQ_BAND1_BIT);

                case SAT_SV_SIG_ID_GLONASS_L2OF:
                    return (bool)(mask & NMEA_GNGSV_FREQ_BAND2_BIT);
                
                default: return false;
            }
        default: 
            return false;
    }
    
    return false;
}

int nmea_gsv_group(char a[], int aSize, gps_sat_t &gsat, gps_sig_t &gsig, uint8_t gnssId, uint8_t sigId=0xFF, bool noCno=false)
{
    char *bufStart = a;

    int numSigs = nmea_gsv_num_sat_sigs(gnssId, sigId, gsig);
    int numMsgs = (numSigs+3) >> 2;	// divide by 4

    uint32_t i=0;
    for (int sigNum = 0; sigNum<numSigs; sigNum+=4)
    {
        int msgNum = (sigNum >> 2) + 1;	// (divide by 4) + 1

        // Message fields: $xxGSV,numMsg,msgNum,numSats,{svid,elv,az,cno,}signalId*cs\r\n
        // Write message header: $xxGSV,numMsg,msgNum,numSats
        int n = nmea_talker(a, aSize, gnssId);
        nmea_sprint(a, aSize, n, "GSV");
        nmea_sprint(a, aSize, n, ",%u,%u,%02u", numMsgs, msgNum, numSigs);		// 1,2,3 - numMsgs, msgNum, numSats in view

        // Write message payload: {svid,elv,az,cno} up to 4x
        for (int cnt=0; cnt<4 && i<=gsig.numSigs; i++)
        {
            gps_sig_sv_t &sig = gsig.sig[i];

            // check if freqency is enabled and that the signals match
            if (gsv_freq_ena(&gsig.sig[i]) && gsv_sig_match(gnssId, sigId, sig, noCno))
            {	
                for (uint32_t j=0; j<=gsat.numSats; j++)
                {
                    gps_sat_sv_t &sat = gsat.sat[j];
                    if ((sat.gnssId == sig.gnssId) && (sat.svId == sig.svId))
                    {
                        uint16_t svId = prnToSvId(sig.gnssId, sig.svId);
                        if (s_protocol_version < NMEA_PROTOCOL_4P10)
                        {
                            svId = sigId_to_nmea2p3_svId(gnssId, sig.sigId, svId);
                        }
                        nmea_sprint(a, aSize, n, ",%02u", svId);				// 4 + 4*msgNum... svid
                        nmea_print_i32(a, aSize, n, 2, sat.elev);				// 5 + 4*msgNum... elevation
                        nmea_print_i32(a, aSize, n, 3, sat.azim);				// 6 + 4*msgNum... azimuth
                        nmea_print_u32(a, aSize, n, 2, sig.cno);				// 7 + 4*msgNum... cno
                        ++cnt;
                        break;
                    }
                }
            }
        }

        // Write message footer
        if (s_protocol_version >= NMEA_PROTOCOL_4P10)
        {
            nmea_sprint(a, aSize, n, ",%c", sigId_to_nmea4p11_signalId(gnssId, sigId));	// Signal ID
        }
        nmea_sprint_footer(a, aSize, n);

        // Move buffer pointer
        a += n;
        aSize -= n;
    } 

    return (int)(a - bufStart);
}


int nmea_gsv_gnss(char a[], int aSize, gps_sat_t &gsat, gps_sig_t &gsig, uint8_t gnssId, bool noCno)
{
    (void)noCno;
    if (s_protocol_version < NMEA_PROTOCOL_4P10)
    {
        return nmea_gsv_group(a, aSize, gsat, gsig, gnssId);
    }

    uint8_t *sigIds;
    uint8_t gpsSigIds[] = { 
        SAT_SV_SIG_ID_GPS_L1CA,
        SAT_SV_SIG_ID_GPS_L2CL,
        SAT_SV_SIG_ID_GPS_L2CM,
        SAT_SV_SIG_ID_GPS_L5I,
        SAT_SV_SIG_ID_GPS_L5Q
    };		
    uint8_t galSigIds[] = { 
        SAT_SV_SIG_ID_Galileo_E1C2,
        SAT_SV_SIG_ID_Galileo_E1B2,
        SAT_SV_SIG_ID_Galileo_E5aI,
        SAT_SV_SIG_ID_Galileo_E5aQ,
        SAT_SV_SIG_ID_Galileo_E5bI,
        SAT_SV_SIG_ID_Galileo_E5bQ,
    };		
    uint8_t beiSigIds[] = { 
        SAT_SV_SIG_ID_BeiDou_B1D1,
        SAT_SV_SIG_ID_BeiDou_B1D2,
        SAT_SV_SIG_ID_BeiDou_B2D1,
        SAT_SV_SIG_ID_BeiDou_B2D2,
        SAT_SV_SIG_ID_BeiDou_B1C,
        SAT_SV_SIG_ID_BeiDou_B2a,
    };		
    uint8_t qzsSigIds[] = { 
        SAT_SV_SIG_ID_QZSS_L1CA,
        SAT_SV_SIG_ID_QZSS_L1S,
        SAT_SV_SIG_ID_QZSS_L2CM,
        SAT_SV_SIG_ID_QZSS_L2CL,
        SAT_SV_SIG_ID_QZSS_L5I,
        SAT_SV_SIG_ID_QZSS_L5Q,
    };		
    uint8_t gloSigIds[] = { 
        SAT_SV_SIG_ID_GLONASS_L1OF,
        SAT_SV_SIG_ID_GLONASS_L2OF,
    };		
    uint8_t nvcSigIds[] = { 
        SAT_SV_SIG_ID_NAVIC_L5A,
    };		
    int numSigIds = 0;

    int n = 0;
    switch(gnssId)
    {
    case SAT_SV_GNSS_ID_GPS:	sigIds = gpsSigIds;		numSigIds = sizeof(gpsSigIds);	break;
    case SAT_SV_GNSS_ID_GAL:	sigIds = galSigIds;		numSigIds = sizeof(galSigIds);	break;
    case SAT_SV_GNSS_ID_BEI:	sigIds = beiSigIds;		numSigIds = sizeof(beiSigIds);	break;
    case SAT_SV_GNSS_ID_QZS:	sigIds = qzsSigIds;		numSigIds = sizeof(qzsSigIds);	break;
    case SAT_SV_GNSS_ID_GLO:	sigIds = gloSigIds;		numSigIds = sizeof(gloSigIds);	break;
    case SAT_SV_GNSS_ID_IRN:	sigIds = nvcSigIds;		numSigIds = sizeof(nvcSigIds);	break;
    default: return 0;
    }

    for (int i = 0; i<numSigIds; i++)
    {
        n += nmea_gsv_group(a+n, aSize-n, gsat, gsig, gnssId, sigIds[i]);
    }

    return n;
}

int nmea_gsv(char a[], const int aSize, gps_sat_t &gsat, gps_sig_t &gsig)
{
    int n = 0;

    // eSatSvGnssId
    for (int gnssId=1; gnssId<=SAT_SV_GNSS_ID_IRN; gnssId++)
    {
        if (gnssId != SAT_SV_GNSS_ID_SBS && (s_gsvMask.constMask[gnssId])) 
        {
            // printf("gnssId: %d\n", gnssId);

            // With CNO
            if((aSize - n) > 0)
                n += nmea_gsv_gnss(a+n, aSize - n, gsat, gsig, gnssId);
            else 
                break;

            // Zero CNO
            // nmea_gsv_gnss(a, aSize, n, gsat, gsig, gnssId, true);
        }
    }

    return n;
}

//////////////////////////////////////////////////////////////////////////
// NMEA to Binary
//////////////////////////////////////////////////////////////////////////

int nmea_parse_info(dev_info_t &info, const char a[], const int aSize)
{
    (void)aSize;
    char *ptr = (char *)&a[6];	// $INFO,
    
    // uint32_t        serialNumber;
    ptr = ASCII_to_u32(&info.serialNumber, ptr);

    // uint8_t         hardwareVer[4];
    ptr = ASCII_to_ver4u8(info.hardwareVer, ptr);

    // uint8_t         firmwareVer[4];
    ptr = ASCII_to_ver4u8(info.firmwareVer, ptr);

    // uint32_t        buildNumber;
    ptr = ASCII_to_u32(&info.buildNumber, ptr);

    // uint8_t         protocolVer[4];
    ptr = ASCII_to_ver4u8(info.protocolVer, ptr);

    // uint32_t        repoRevision;
    ptr = ASCII_to_u32(&info.repoRevision, ptr);

    // char            manufacturer[DEVINFO_MANUFACTURER_STRLEN];
    ptr = ASCII_to_char_array(info.manufacturer, ptr, DEVINFO_MANUFACTURER_STRLEN);

    // uint8_t         buildDate[4];	YYYY-MM-DD
    unsigned int year, month, day;
    SSCANF(ptr, "%04d-%02u-%02u", &year, &month, &day);
    info.buildType = ' ';
    info.buildYear = (uint8_t)(year >= 2000 ? (year - 2000):year);
    info.buildMonth = (uint8_t)(month);
    info.buildDay = (uint8_t)(day);
    ptr = ASCII_find_next_field(ptr);
    
    // uint8_t         buildTime[4];	hh:mm:ss.ms
    unsigned int hour, minute, second, ms;
    SSCANF(ptr, "%02u:%02u:%03u.%02u", &hour, &minute, &second, &ms);
    info.buildHour = (uint8_t)hour;
    info.buildMinute = (uint8_t)minute;
    info.buildSecond = (uint8_t)second;
    info.buildMillisecond = (uint8_t)ms;
    ptr = ASCII_find_next_field(ptr);
    
    // char            addInfo[DEVINFO_ADDINFO_STRLEN];
    ptr = ASCII_to_char_array(info.addInfo, ptr, DEVINFO_ADDINFO_STRLEN);

    // uint16_t        hardware;
    ptr = ASCII_to_u8(&info.hardwareType, ptr);

    // uint16_t        reserved;
    ptr = ASCII_to_u16(&info.reserved, ptr);

    // uint8_t         build type;
    info.buildType = (uint8_t)*ptr;
    if (info.buildType==0) { info.buildType = ' '; }

    // ptr = ASCII_find_next_field(ptr);

    // TODO: dev_info_t.firmwareMD5Hash support
    // uint32_t         firmwareMD5Hash[4];
    // ptr = ASCII_to_MD5(info.firmwareMD5Hash, ptr);

    // Populate missing hardware descriptor
    devInfoPopulateMissingHardware(&info);

    return 0;
}

int nmea_parse_pimu(imu_t &imu, const char a[], const int aSize)
{
    (void)aSize;
    char *ptr = (char *)&a[6];	// $PIMU,
    
    // Time since system powerup 
    ptr = ASCII_to_f64(&(imu.time), ptr);

    // PQR angular rate
    ptr = ASCII_to_vec3f(imu.I.pqr, ptr);
    // XYZ linear acceleration
    ptr = ASCII_to_vec3f(imu.I.acc, ptr);

    return 0;
}

int nmea_parse_pimu_to_rimu(imu_t &imu, const char a[], const int aSize)
{
    (void)aSize;
    char *ptr = (char *)&a[7];	// $PRIMU,
    
    // Time since system powerup 
    ptr = ASCII_to_f64(&(imu.time), ptr);

    // PQR angular rate
    ptr = ASCII_to_vec3f(imu.I.pqr, ptr);
    // XYZ linear acceleration
    ptr = ASCII_to_vec3f(imu.I.acc, ptr);

    return 0;
}

int nmea_parse_ppimu(pimu_t &pimu, const char a[], const int aSize)
{
    (void)aSize;
    char *ptr = (char *)&a[7];	// $PPIMU,
    
    // Time since system powerup 
    ptr = ASCII_to_f64(&(pimu.time), ptr);

    // PQR angular rate
    ptr = ASCII_to_vec3f(pimu.theta, ptr);
    // XYZ linear acceleration
    ptr = ASCII_to_vec3f(pimu.vel, ptr);

    // Integration period 
    ptr = ASCII_to_f32(&(pimu.dt), ptr);

    return 0;
}

int nmea_parse_pins1(ins_1_t &ins, const char a[], const int aSize)
{
    (void)aSize;
    char *ptr = (char *)&a[7];	// $PINS1,
    
    // GPS timeOfWeek, week 
    ptr = ASCII_to_f64(&(ins.timeOfWeek), ptr);
    ptr = ASCII_to_u32(&(ins.week), ptr);

    // insStatus, hdwStatus
    ptr = ASCII_to_u32(&(ins.insStatus), ptr);
    ptr = ASCII_to_u32(&(ins.hdwStatus), ptr);

    // Roll, Pitch, Yaw
    ptr = ASCII_to_vec3f(ins.theta, ptr);
    // UVW
    ptr = ASCII_to_vec3f(ins.uvw, ptr);
    // LLA
    ptr = ASCII_to_vec3d(ins.lla, ptr);
    // NED
    ptr = ASCII_to_vec3f(ins.ned, ptr);

    return 0;
}

int nmea_parse_pins2(ins_2_t &ins, const char a[], const int aSize)
{
    (void)aSize;
    char *ptr = (char *)&a[7];	// $PINS2,
    
    // GPS timeOfWeek, week 
    ptr = ASCII_to_f64(&(ins.timeOfWeek), ptr);
    ptr = ASCII_to_u32(&(ins.week), ptr);

    // insStatus, hdwStatus
    ptr = ASCII_to_u32(&(ins.insStatus), ptr);
    ptr = ASCII_to_u32(&(ins.hdwStatus), ptr);

    // Quaternion
    ptr = ASCII_to_vec4f(ins.qn2b, ptr);
    // UVW
    ptr = ASCII_to_vec3f(ins.uvw, ptr);
    // LLA
    ptr = ASCII_to_vec3d(ins.lla, ptr);

    return 0;
}

int nmea_parse_pgpsp(gps_pos_t &gpsPos, gps_vel_t &gpsVel, const char a[], const int aSize)
{
    (void)aSize;
    char *ptr = (char *)&a[7];	// $PGPSP,
    
    // GPS timeOfWeekMs, week 
    ptr = ASCII_to_u32(&(gpsPos.timeOfWeekMs), ptr);
    ptr = ASCII_to_u32(&(gpsPos.week), ptr);
    gpsVel.timeOfWeekMs = gpsPos.timeOfWeekMs;

    // status
    ptr = ASCII_to_u32(&(gpsPos.status), ptr);
    gpsPos.satsUsed = gpsPos.status & GPS_STATUS_NUM_SATS_USED_MASK;

    // LLA, MSL altitude
    ptr = ASCII_to_vec3d(gpsPos.lla, ptr);
    ptr = ASCII_to_f32(&(gpsPos.hMSL), ptr);

    // pDop, hAcc, vAcc
    ptr = ASCII_to_f32(&(gpsPos.pDop), ptr);
    ptr = ASCII_to_f32(&(gpsPos.hAcc), ptr);
    ptr = ASCII_to_f32(&(gpsPos.vAcc), ptr);

    // Velocity, sAcc
    ptr = ASCII_to_vec3f(gpsVel.vel, ptr);
    ptr = ASCII_to_f32(&(gpsVel.sAcc), ptr);

    // cnoMean
    ptr = ASCII_to_f32(&(gpsPos.cnoMean), ptr);

    // Time of Week offset, leapS
    ptr = ASCII_to_f64(&(gpsPos.towOffset), ptr);
    ptr = ASCII_to_u8(&(gpsPos.leapS), ptr);

    return 0;
}

/**
 * Sets ASCE special case for GSV messages.
 * returns NMEA_MSG_ID_GNGSV if successful 
 * returns 0 if unsuccessful
*/
int parseASCE_GSV(int inId, int period)
{
    uint8_t constTarget = (inId & 0xf0) >> 4;
    uint8_t freqMask = (inId & 0x0f);

    if(inId < NMEA_MSG_ID_GNGSV_START || inId > NMEA_MSG_ID_GNGSV_END)
        return 0;

    if (period != 0)
    {
        switch (constTarget)
        {
            case SAT_SV_GNSS_ID_GNSS:
                s_gsvMask.constMask[SAT_SV_GNSS_ID_GNSS] = freqMask;
                s_gsvMask.constMask[SAT_SV_GNSS_ID_GPS] = freqMask;
                s_gsvMask.constMask[SAT_SV_GNSS_ID_SBS] = freqMask;
                s_gsvMask.constMask[SAT_SV_GNSS_ID_GAL] = freqMask;
                s_gsvMask.constMask[SAT_SV_GNSS_ID_BEI] = freqMask;
                s_gsvMask.constMask[SAT_SV_GNSS_ID_QZS] = freqMask;
                s_gsvMask.constMask[SAT_SV_GNSS_ID_GLO] = freqMask;
                break;
            case SAT_SV_GNSS_ID_GPS:
                s_gsvMask.constMask[SAT_SV_GNSS_ID_GPS] = freqMask;
                break;
            case SAT_SV_GNSS_ID_GAL:
                s_gsvMask.constMask[SAT_SV_GNSS_ID_GAL] = freqMask;
                break;
            case SAT_SV_GNSS_ID_BEI:
                s_gsvMask.constMask[SAT_SV_GNSS_ID_BEI] = freqMask;
                break;
            case SAT_SV_GNSS_ID_QZS:
                s_gsvMask.constMask[SAT_SV_GNSS_ID_QZS] = freqMask;
                break;
            case SAT_SV_GNSS_ID_GLO:
                s_gsvMask.constMask[SAT_SV_GNSS_ID_GLO] = freqMask;
                break;
            default:
                return 0;
        }
    }

    return NMEA_MSG_ID_GNGSV;
}

uint32_t nmea_parse_asce(int pHandle, const char a[], int aSize, rmci_t rmci[NUM_COM_PORTS])
{
    (void)aSize;

    uint32_t options = 0;
    uint32_t id;
    uint32_t ports;
    uint8_t period;

    if(pHandle >= NUM_COM_PORTS)
    {
        return 0;
    }
    
    char *ptr = (char*)&a[6];				// $ASCE
    char *end = (char*)&a[aSize];
    
    // check if next index is ','
    if(*ptr != ',')
        options = (uint32_t)atoi(ptr);
    
    // get next uint32_t and assign it to options and move pointer
    ptr = ASCII_to_u32(&options, ptr);

    // extract port from options
    ports = options&RMC_OPTIONS_PORT_MASK;
    
    for (int i=0; i<20; i++)
    {
        // end of nmea string
        if(*ptr == '*')
             break;
        
        // set id and increment ptr to next field
        if (isdigit(*ptr))
        {	// Is a number.  Read NMEA ID directly
            id = ((*ptr == ',') ? 0 : atoi(ptr));
        }
        else
        {	// Is a letter.  Convert talker string to NMEA ID
            char *ptr2 = ptr-1;
            id = getNmeaMsgId(ptr2, end-ptr2);
        }

        ptr = ASCII_find_next_field(ptr);

        // end of nmea string
        if(*ptr=='*')
            break;
        
        // set period multiple and increament ptr to next field
        period = ((*ptr==',') ? 0 : (uint8_t)atoi(ptr));	
        ptr = ASCII_find_next_field(ptr);

        // handle GSV cases
        if (id == NMEA_MSG_ID_GNGSV)
            parseASCE_GSV(NMEA_MSG_ID_GNGSV_5_3_2_1, period);
        else if(id >= NMEA_MSG_ID_GNGSV_START && id <= NMEA_MSG_ID_GNGSV_END)
            id = parseASCE_GSV(id, period);

        // Copy tmp to corresponding port(s)
        switch (ports)
        {	
        case RMC_OPTIONS_PORT_CURRENT:	nmea_enable_stream(rmci[pHandle].rmcNmea.nmeaBits, rmci[pHandle].rmcNmea.nmeaPeriod, id, period); break;
        case RMC_OPTIONS_PORT_ALL:
            for(int i=0; i<NUM_COM_PORTS; i++) 
            { 
                nmea_enable_stream(rmci[i].rmcNmea.nmeaBits, rmci[i].rmcNmea.nmeaPeriod, id,  period);                 
            } 

            if (id == NMEA_MSG_ID_GNGSV && period == 0)
            {
                for(int i = SAT_SV_GNSS_ID_GNSS; i < SAT_SV_GNSS_ID_COUNT; i++) 
                    s_gsvMask.constMask[i] = 0;
            }

            break;
            
        default:	// Current port
            if (ports & RMC_OPTIONS_PORT_SER0)     { nmea_enable_stream(rmci[0].rmcNmea.nmeaBits, rmci[0].rmcNmea.nmeaPeriod, id, period); }
            if (ports & RMC_OPTIONS_PORT_SER1)     { nmea_enable_stream(rmci[1].rmcNmea.nmeaBits, rmci[1].rmcNmea.nmeaPeriod, id, period); }
            if (ports & RMC_OPTIONS_PORT_SER2)     { nmea_enable_stream(rmci[2].rmcNmea.nmeaBits, rmci[2].rmcNmea.nmeaPeriod, id, period); }
            if (ports & RMC_OPTIONS_PORT_USB)      { nmea_enable_stream(rmci[3].rmcNmea.nmeaBits, rmci[3].rmcNmea.nmeaPeriod, id, period); }
            break;
        }
    }
        
    return options;
}

uint32_t nmea_parse_asce_grmci(int pHandle, const char a[], int aSize, grmci_t rmci[NUM_COM_PORTS])
{
    (void)aSize;

    uint32_t options = 0;
    uint32_t id;
    uint32_t ports;
    uint8_t period;

    if(pHandle >= NUM_COM_PORTS)
        return 0;
    
    char *ptr = (char*)&a[6];				// $ASCE
    char *end = (char*)&a[aSize];
    
    // check if next index is ','
    if(*ptr != ',')
        options = (uint32_t)atoi(ptr);
    
    // get next uint32_t and assign it to options and move pointer
    ptr = ASCII_to_u32(&options, ptr);

    // extract port from options
    ports = options & RMC_OPTIONS_PORT_MASK;
    
    for (int i = 0; i < 20; i++)
    {
        // end of nmea string
        if(*ptr == '*')
             break;
        
        // set id and increament ptr to next field
        if (isdigit(*ptr))
        {	// Is a number.  Read NMEA ID directly
            id = ((*ptr == ',') ? 0 : atoi(ptr));
        }
        else
        {	// Is a letter.  Convert talker string to NMEA ID
            char *ptr2 = ptr-1;
            id = getNmeaMsgId(ptr2, end-ptr2);
        }
        
        ptr = ASCII_find_next_field(ptr);

        // end of nmea string
        if(*ptr=='*')
            break;
        
        // set period multiple and increament ptr to next field
        period = ((*ptr==',') ? 0 : (uint8_t)atoi(ptr));	
        ptr = ASCII_find_next_field(ptr);

        // handle GSV cases
        if (id == NMEA_MSG_ID_GNGSV)
            parseASCE_GSV(NMEA_MSG_ID_GNGSV, period);
        else if(id >= NMEA_MSG_ID_SPECIAL_CASE_START) 
            id = parseASCE_GSV(id, period);

        // Copy tmp to corresponding port(s)
        switch (ports)
        {	
        case RMC_OPTIONS_PORT_CURRENT:	
            nmea_enable_stream(rmci[pHandle].rmcNmea.nmeaBits, rmci[pHandle].rmcNmea.nmeaPeriod, id, period);
            rmci[pHandle].rmc.options |= (options & RMC_OPTIONS_PERSISTENT);
            break;
        
        case RMC_OPTIONS_PORT_ALL:		
            for(int i=0; i<NUM_COM_PORTS; i++) 
            { 
                nmea_enable_stream(rmci[i].rmcNmea.nmeaBits, rmci[i].rmcNmea.nmeaPeriod, id,  period); 
                rmci[i].rmc.options |= (options & RMC_OPTIONS_PERSISTENT);
            } 
            
            if (id == NMEA_MSG_ID_GNGSV && period == 0)
            {
                for(int i = SAT_SV_GNSS_ID_GNSS; i < SAT_SV_GNSS_ID_COUNT; i++) 
                    s_gsvMask.constMask[i] = 0;
            }
            break;
            
        default:	// Current port
            if (ports & RMC_OPTIONS_PORT_SER0)     
            { 
                nmea_enable_stream(rmci[0].rmcNmea.nmeaBits, rmci[0].rmcNmea.nmeaPeriod, id, period);
                rmci[0].rmc.options |= (options & RMC_OPTIONS_PERSISTENT);
            }
            if (ports & RMC_OPTIONS_PORT_SER1)    
            { 
                nmea_enable_stream(rmci[1].rmcNmea.nmeaBits, rmci[1].rmcNmea.nmeaPeriod, id, period);
                rmci[1].rmc.options |= (options & RMC_OPTIONS_PERSISTENT);
            }
            if (ports & RMC_OPTIONS_PORT_SER2)     
            { 
                nmea_enable_stream(rmci[2].rmcNmea.nmeaBits, rmci[2].rmcNmea.nmeaPeriod, id, period);
                rmci[2].rmc.options |= (options & RMC_OPTIONS_PERSISTENT); 
            }
            if (ports & RMC_OPTIONS_PORT_USB)      
            { 
                nmea_enable_stream(rmci[3].rmcNmea.nmeaBits, rmci[3].rmcNmea.nmeaPeriod, id, period);
                rmci[3].rmc.options |= (options & RMC_OPTIONS_PERSISTENT); 
            }
            break;
        }
    }
        
    return options;
}

/* G_GNS Message
* Provides position data (newer message) using the following:
*   Time
*   Position (lat, lon)
*   Positioning mode (fix type)
*   Altitude & Geoid separation
*/
int nmea_parse_gns(const char a[], const int aSize, gps_pos_t &gpsPos, utc_time_t &utcTime, int utcWeekday, uint32_t statusFlags)
{
    (void)aSize;
    char *ptr = (char *)&a[7];	// $GxGNS,
    //$xxGNS,time,lat,NS,lon,EW,posMode,numSV,HDOP,alt,sep,diffAge,diffStation,navStatus*cs<CR><LF>

    // 1 - UTC time HHMMSS.sss
    ptr = ASCII_UtcTimeToGpsTowMs(&gpsPos.timeOfWeekMs, &utcTime, ptr, utcWeekday, gpsPos.leapS);
        
    //Latitude
    ixVector3d lla;
    lla[0] = ddmm2deg(atof(ptr));
    ptr = ASCII_find_next_field(ptr);
    if(*ptr == 'S')
        lla[0] = -lla[0];
    ptr = ASCII_find_next_field(ptr);

    //Longitude
    lla[1] = ddmm2deg(atof(ptr));
    ptr = ASCII_find_next_field(ptr);
    if(*ptr == 'W')
        lla[1] = -lla[1];
    ptr = ASCII_find_next_field(ptr);

    //Positioning Mode
    char pMode[4] = {0,0,0,0};
    if(*ptr != ',')
        pMode[0] = *ptr++;
    if(*ptr != ',')
        pMode[1] = *ptr++;
    if(*ptr != ',')
        pMode[2] = *ptr++;
    if(*ptr != ',')
        pMode[3] = *ptr++;
    ptr = ASCII_find_next_field(ptr);
        
    //Based off of ZED-F9P datasheet
    uint32_t fixType = GPS_STATUS_FIX_NONE;
    statusFlags |= GPS_STATUS_FLAGS_GPS_NMEA_DATA;
    gpsPos.hAcc = 0.0f;
    if(pMode[0] == 'R' || pMode[1] == 'R' || pMode[2] == 'R' || pMode[3] == 'R')		// RTK fix
    {
        fixType = GPS_STATUS_FIX_RTK_FIX;
        statusFlags |= 
            GPS_STATUS_FLAGS_FIX_OK |
            GPS_STATUS_FLAGS_GPS1_RTK_POSITION_ENABLED |
            GPS_STATUS_FLAGS_GPS1_RTK_POSITION_VALID |
            GPS_STATUS_FLAGS_RTK_FIX_AND_HOLD |
            GPS_STATUS_FLAGS_DGPS_USED;
        gpsPos.hAcc = 0.05f;
    }
    else if(pMode[0] == 'F' || pMode[1] == 'F' || pMode[2] == 'F' || pMode[3] == 'F')	// RTK float
    {
        fixType = GPS_STATUS_FIX_RTK_FLOAT;
        statusFlags |=
            GPS_STATUS_FLAGS_FIX_OK |
            GPS_STATUS_FLAGS_GPS1_RTK_POSITION_ENABLED |
            GPS_STATUS_FLAGS_DGPS_USED;
        gpsPos.hAcc = 0.4f;
    }
    else if(pMode[0] == 'D' || pMode[1] == 'D' || pMode[2] == 'D' || pMode[3] == 'D')	// Differential (DGPS)
    {
        fixType = GPS_STATUS_FIX_DGPS;
        statusFlags |= 
            GPS_STATUS_FLAGS_FIX_OK |
            GPS_STATUS_FLAGS_DGPS_USED;
        gpsPos.hAcc = 0.8f;
    }
    else if(pMode[0] == 'A' || pMode[1] == 'A' || pMode[2] == 'A' || pMode[3] == 'A')	// Autonomous, 2D/3D
    {
        fixType = GPS_STATUS_FIX_3D;
        statusFlags |= GPS_STATUS_FLAGS_FIX_OK;
        gpsPos.hAcc = 1.5f;
    }
    else if(pMode[0] == 'E' || pMode[1] == 'E' || pMode[2] == 'E' || pMode[3] == 'E')	// Dead reckoning
    {
        fixType = GPS_STATUS_FIX_DEAD_RECKONING_ONLY;
    }
    gpsPos.vAcc = 1.4f * gpsPos.hAcc;
            
    //Number of satellites used in solution
    gpsPos.satsUsed = atoi(ptr);
    ptr = ASCII_find_next_field(ptr);
        
    //HDOP
    ptr = ASCII_find_next_field(ptr);
        
    //MSL Altitude (altitude above mean sea level)
    lla[2] = atof(ptr);
    gpsPos.hMSL = (float)lla[2];
    ptr = ASCII_find_next_field(ptr);

    //Geoid separation (difference between ellipsoid and mean sea level)
    double sep = atof(ptr);
        
    //Store data		
    set_gpsPos_status_mask(&(gpsPos.status), gpsPos.satsUsed, (uint32_t)GPS_STATUS_NUM_SATS_USED_MASK);
    set_gpsPos_status_mask(&(gpsPos.status), statusFlags, (uint32_t)GPS_STATUS_FLAGS_MASK);
    set_gpsPos_status_mask(&(gpsPos.status), fixType, (uint32_t)GPS_STATUS_FIX_MASK);
        
    gpsPos.lla[0] = lla[0];
    gpsPos.lla[1] = lla[1];
    gpsPos.lla[2] = lla[2] + sep;

    //Change LLA to radians
    lla[0] = DEG2RAD(lla[0]);
    lla[1] = DEG2RAD(lla[1]);
    lla[2] = gpsPos.lla[2];	// Use ellipsoid alt
        
    //Convert LLA to ECEF.  Ensure LLA uses ellipsoid alt 
    ixVector3d ecef;
    lla2ecef(lla, ecef);
                
    gpsPos.ecef[0] = ecef[0];
    gpsPos.ecef[1] = ecef[1];
    gpsPos.ecef[2] = ecef[2];	

    return 0;	
}

int nmea_parse_gga(const char a[], const int aSize, gps_pos_t &gpsPos, utc_time_t &utcTime, int utcWeekday, uint32_t statusFlags)
{
    (void)aSize;
    char *ptr = (char *)&a[7];	// $GxGGA,
    
    // 1 - UTC time HHMMSS.sss
    ptr = ASCII_UtcTimeToGpsTowMs(&gpsPos.timeOfWeekMs, &utcTime, ptr, utcWeekday, gpsPos.leapS);
    // 2,3 - Latitude (deg)
    ptr = ASCII_DegMin_to_Lat(&(gpsPos.lla[0]), ptr);
    // 4,5 - Longitude (deg)
    ptr = ASCII_DegMin_to_Lon(&(gpsPos.lla[1]), ptr);

    // 6 - Fix quality
    uint32_t fixQuality;
    ptr = ASCII_to_u32(&fixQuality, ptr);
    gpsPos.hAcc = 0.0f;
    gpsPos.vAcc = 0.0f;

    uint32_t fixType = GPS_STATUS_FIX_NONE;
    switch(fixQuality)
    {
    case 6:		// Dead reckoning
        fixType = GPS_STATUS_FIX_DEAD_RECKONING_ONLY;
        break;

    case 5:		// RTK float
        fixType = GPS_STATUS_FIX_RTK_FLOAT;
        statusFlags |=
            GPS_STATUS_FLAGS_FIX_OK |
            GPS_STATUS_FLAGS_GPS1_RTK_POSITION_ENABLED |
            GPS_STATUS_FLAGS_DGPS_USED;
        gpsPos.hAcc = 0.4f;
        break;
    
    case 4:		// RTK fix
        fixType = GPS_STATUS_FIX_RTK_FIX;
        statusFlags |= 
            GPS_STATUS_FLAGS_FIX_OK |
            GPS_STATUS_FLAGS_GPS1_RTK_POSITION_ENABLED |
            GPS_STATUS_FLAGS_GPS1_RTK_POSITION_VALID |
            GPS_STATUS_FLAGS_RTK_FIX_AND_HOLD |
            GPS_STATUS_FLAGS_DGPS_USED;
        gpsPos.hAcc = 0.05f;
        break;

    case 3:		// Time only
        fixType = GPS_STATUS_FIX_TIME_ONLY;
        gpsPos.hAcc = 0.8f;
        break;

    case 2:		// Differential
        fixType = GPS_STATUS_FIX_DGPS;
        statusFlags |= 
            GPS_STATUS_FLAGS_FIX_OK |
            GPS_STATUS_FLAGS_DGPS_USED;
        gpsPos.hAcc = 0.8f;
        break;

    case 1:		// Autonomous
        fixType = GPS_STATUS_FIX_3D;
        statusFlags |= GPS_STATUS_FLAGS_FIX_OK;
        gpsPos.hAcc = 1.5f;
        break;

    default:	break;
    }
    
    // 7 - Satellites used
    ptr = ASCII_to_u8(&(gpsPos.satsUsed), ptr);

    gpsPos.status = statusFlags | fixType | GPS_STATUS_FLAGS_GPS_NMEA_DATA;
    gpsPos.status |= gpsPos.satsUsed;

    // 8 - hDop
    ptr = ASCII_to_f32(&(gpsPos.pDop), ptr);

    // 9,10 - MSL altitude
    ptr = ASCII_to_f32(&(gpsPos.hMSL), ptr);
    ptr = ASCII_find_next_field(ptr);

    // 11,12 - Geoid separation = alt(HAE) - alt(MSL)
    double geoidSep;
    ptr = ASCII_to_f64(&(geoidSep), ptr);
    gpsPos.lla[2] = gpsPos.hMSL + geoidSep;

    // Convert LLA to ECEF.  Ensure LLA uses ellipsoid altitude
    ixVector3d lla;
    lla[0] = DEG2RAD(gpsPos.lla[0]);
    lla[1] = DEG2RAD(gpsPos.lla[1]);
    lla[2] = gpsPos.lla[2];		// Use ellipsoid altitude
    lla2ecef(lla, gpsPos.ecef);

    // 13 - time since last DGPS update
    // 14 - DGPS station ID number

    return 0;
}

int nmea_parse_gll(const char a[], const int aSize, gps_pos_t &gpsPos, utc_time_t &utcTime, int utcWeekday)
{
    (void)aSize;
    char *ptr = (char *)&a[7];	// $GxGLL,
    
    // 1,2 - Latitude (deg)
    ptr = ASCII_DegMin_to_Lat(&(gpsPos.lla[0]), ptr);
    // 3,4 - Longitude (deg)
    ptr = ASCII_DegMin_to_Lon(&(gpsPos.lla[1]), ptr);
    // 5 - UTC time HHMMSS.sss
    ptr = ASCII_UtcTimeToGpsTowMs(&gpsPos.timeOfWeekMs, &utcTime, ptr, utcWeekday, gpsPos.leapS);
    // 6 - Valid (A=active, V=void)

    return 0;
}

/* G_GSA Message
* Provides pDOP and navigation mode (saved to determine 2D/3D mode)
*/
int nmea_parse_gsa(const char a[], const int aSize, gps_pos_t &gpsPos, gps_sat_t *sat)
{
    (void)aSize;
    char *ptr = (char *)&a[7];	// $GxGSA,
    //$xxGSA,opMode,navMode{,svid},PDOP,HDOP,VDOP,systemId*cs<CR><LF>
    
    // 1 - Auto selection of 2D or 3D
    ptr = ASCII_find_next_field(ptr);

    // 2 - Fix quality
    uint32_t fixQuality;
    ptr = ASCII_to_u32(&fixQuality, ptr);
    gpsPos.status &= ~GPS_STATUS_FIX_MASK;
    switch(fixQuality)
    {
    default:														break;
    case 2:	gpsPos.status |= GPS_STATUS_FIX_2D;						break;
    case 3:	gpsPos.status |= GPS_STATUS_FIX_3D;						break;
    }

    // 3-14 - Sat ID
    if (sat)
    {
        for (uint32_t i = 0; i < 12; i++)
        {
            ptr = ASCII_to_u8(&(sat->sat[i].svId), ptr);
        }
    }

    // 15 - pDop
    ptr = ASCII_to_f32(&(gpsPos.pDop), ptr);

    // 16 - hDop (hAcc)
    ptr = ASCII_to_f32(&(gpsPos.hAcc), ptr);

    // 17 - vDop (vAcc)
    ptr = ASCII_to_f32(&(gpsPos.vAcc), ptr);

    return 0;
}

/* G_GSV Message
* Provides satellite information
* Multiple GSV messages will come in a block. We wait until block is finished before flagging data is ready.
*/
char* nmea_parse_gsv(const char a[], const int aSize, gps_sat_t *gpsSat, gps_sig_t *gpsSig, uint32_t *cnoSum, uint32_t *cnoCount)
{
    if (gpsSat == NULL || gpsSig == NULL)
    {	// Don't parse
        return (char*)a;
    }

    (void)aSize;
    char *ptr = (char *)&a[7];	// $GNGSV,
    // $GNGSV, numMsgs, msgNum, numSats, {,svid,elv,az,cno}, signalId *checksum <CR><LF>
        
    int32_t numMsgs, msgNum, numSigs;
    ptr = ASCII_to_i32(&numMsgs, ptr);			// 1 - number of messages
    ptr = ASCII_to_i32(&msgNum, ptr);			// 2 - message number
    ptr = ASCII_to_i32(&numSigs, ptr);			// 3 - number of satellite signals in view

    uint8_t gnssId = SAT_SV_GNSS_ID_UNKNOWN;
    uint8_t sigId = 0;
    uint8_t *sigIds[4] = { NULL };
    uint8_t **sigIdPtr = sigIds;
            
    // Process up to 4 satellites
    int satCnt = _MIN(numSigs - (msgNum - 1) * 4, 4);

    // Payload: {svid,elv,az,cno} up to 4x
    for(int i=0; i<satCnt; ++i)
    {
        uint8_t elev, cno;
        uint16_t svId, azim;

        ptr = ASCII_to_u16(&svId, ptr);		// 4 + 4x   svId
        ptr = ASCII_to_u8(&elev, ptr);		// 5 + 4x   elevation
        ptr = ASCII_to_u16(&azim, ptr);		// 6 + 4x   azimuth
        ptr = ASCII_to_u8(&cno, ptr);		// 7 + 4x   cno

        talkerId_to_gnssId(a, gnssId, svId, sigId);
                
        // Add to satellite info list
        for (uint32_t j=0;; j++)
        {
            if (j >= gpsSat->numSats)
            {	// Not in list
                if(gpsSat->numSats < MAX_NUM_SATELLITES)
                {	// Add to list
                    auto& dst = gpsSat->sat[gpsSat->numSats];
                    gpsSat->numSats++;
                    
                    dst.gnssId = gnssId;
                    dst.svId = (uint8_t)svId;
                    dst.elev = elev;
                    dst.azim = azim;
                    dst.cno = cno;
                    dst.status = 
                        SAT_SV_STATUS_SIGNAL_QUALITY_MASK |
                        SAT_SV_STATUS_USED_IN_SOLUTION | 
                        SAT_SV_STATUS_HEALTH_GOOD;
                }
                break;
            }
            if ((gnssId == gpsSat->sat[j].gnssId) && (svId == gpsSat->sat[j].svId)) 
            { 
                break; 
            }	// already in list
        }

        // Add to satellite signal list
        for (uint32_t j=0;; j++)
        {
            if (j >= gpsSig->numSigs)
            {	// Not in list
                if(gpsSig->numSigs < MAX_NUM_SAT_SIGNALS)
                {	// Add to list
                    auto& dst = gpsSig->sig[gpsSig->numSigs];
                    gpsSig->numSigs++;
                    
                    dst.gnssId = gnssId;
                    dst.svId = (uint8_t)svId;
                    dst.sigId = sigId;	// Gets set at function end if using protocol > NMEA 4.1
                    *sigIdPtr = &(dst.sigId);
                    sigIdPtr++;
                    dst.quality = SAT_SIG_QUALITY_CODE_CARRIER_TIME_SYNC_3;
                    dst.cno = cno;
                    dst.status = SAT_SIG_STATUS_HEALTH_GOOD | SAT_SIG_STATUS_USED_IN_SOLUTION;
                }
                break;
            }
        }

        // Calculate the sum and count of non-zero cno values, in order to calculate the cnoMean
        if (cno != 0)
        {
            (*cnoSum) += cno;
            ++(*cnoCount);
        }
    }

    if (s_protocol_version >= NMEA_PROTOCOL_4P10)
    {
        uint8_t sigId = nmea4p11_signalId_to_sigId(gnssId, *ptr);	// nmea signal ID
        ptr = ASCII_find_next_field(ptr);
        for (int i=0; i<4 && sigIds[i]!=NULL; i++)
        {	// Retroactivly set sigId satSig list
            *(sigIds[i]) = sigId;
        }
    }
    
    // Move past checksum
    return ptr + 5;
}

int nmea_parse_intel(const char a[], const int aSize, dev_info_t &info, gps_pos_t &pos, gps_vel_t &vel, float ppsPhase[2], uint32_t ppsNoiseNs[1])
{
    (void)aSize;
    char *ptr = (char *)&a[7];	// $INTEL,
    
    // 1 - Message ID KIM
    ptr = ASCII_find_next_field(ptr);

    // 2 -	Fimrware version of KIM
    ptr = ASCII_to_ver4u8(info.firmwareVer, ptr);
    
    // 3 -	GPS Time of Week (seconds)
    uint32_t timeSec;
    ptr = ASCII_to_u32(&timeSec, ptr);
    pos.timeOfWeekMs = 1000*timeSec;

    // 4 -	GPS week number
    ptr = ASCII_to_u32(&(pos.week), ptr);
    
    // 5 -	GPS leap seconds
    ptr = ASCII_to_u8(&(pos.leapS), ptr);
    
    // 6 -	1PPS phase 1 (ns)
    ptr = ASCII_to_f32(&(ppsPhase[0]), ptr);
    
    // 7 -	1PPS phase 2 (ns)
    ptr = ASCII_to_f32(&(ppsPhase[1]), ptr);
    
    // 8 -	Quantization error of time pulse (ns)
    ptr = ASCII_to_u32(&(ppsNoiseNs[0]), ptr);
    
    // 9-11 - ECEF X,Y,Z velocity (m/s)
    ptr = ASCII_to_vec3f(vel.vel, ptr);
        
    // 12-14 - NED veocity (m/s)
    // float velNed[3];
    // ptr = ASCII_to_vec3f(velNed, ptr);

    return 0;
}

/**
 * @brief $POWGPS prorietary NMEA message
 * 
 * @param a[]  NMEA string
 * @param aSize  NMEA string size
 * @param pos  GPS position structure
 * 
 * @note
 *  0   Message ID $POWGPS
 *  1   GPS Time Quality (0=invalid, 1=valid)
 *  2   GPS Week Number
 *  3   GPS Time of Week (micro seconds)
 *  4   GPS leap seconds validity (0=invalid, 1=valid)
 *  5   GPS leap seconds
 *  6   Holdover flag (0=no holdover, 1=EGR is in holdover)
 *  7  Checksum, begins with *
 */
int nmea_parse_powgps(const char a[], const int aSize, gps_pos_t &pos)
{
    /*  $POWGPS prorietary NMEA message
            0   Message ID $POWGPS
            1   GPS Time Quality (0=invalid, 1=valid)
            2   GPS Week Number
            3   GPS Time of Week (micro seconds)
            4   GPS leap seconds validity (0=invalid, 1=valid)
            5   GPS leap seconds
            6   Holdover flag (0=no holdover, 1=EGR is in holdover)
            7   Checksum, begins with *
    */
    (void)aSize;
    uint64_t TOWus;
    char *ptr = (char *)&a[8];	// $POWGPS,
    uint32_t timeValid;
    uint32_t lsValid;
    
    // 1 -	GPS Time valid
    ptr = ASCII_to_u32(&timeValid, ptr);

    // 2 -	GPS week number
    ptr = ASCII_to_u32(&(pos.week), ptr);

    // 3 -	GPS Time of Week (us)
    ptr = ASCII_to_u64(&TOWus, ptr);
    pos.timeOfWeekMs = TOWus/1000;
    
    // 4 -	GPS leap seconds valid
    ptr = ASCII_to_u32(&lsValid, ptr);
    
    // 5 -	GPS leap seconds
    ptr = ASCII_to_u8(&(pos.leapS), ptr);

    // 6 -	Holdover flag (0=no holdover, 1=EGR is in holdover)

    if (lsValid == 0) { pos.leapS = 0; }
    if (timeValid == 0) { pos.timeOfWeekMs = 0; pos.week = 0; }

    return 0;
}

/**
 * @brief $POWTLV prorietary NMEA message
 * 
 * @param a[]  NMEA string
 * @param aSize  NMEA string size
 * @param pos  GPS position structure
 * @param vel  GPS velocity structure
 * 
 * @note
 *  0   Message ID $POWGPS
 *  1   GPS Time Quality (0=invalid, 1=valid)
 *  2   GPS Week Number
 *  3   GPS Time of Week (micro seconds)
 *  4   GPS leap seconds validity (0=invalid, 1=valid)
 *  5   GPS leap seconds
 *  6   Holdover flag (0=no holdover, 1=EGR is in holdover)
 *  7   Latitude ddmm.mmmm
 *  8   North/South indicator (N/S)
 *  9   Longitude dddmm.mmmm
 *  10  East/West indicator (E/W)
 *  11  Altitude (x.xxx meters)
 *  12  Mean Sea Level (MSL) (x.xxx meters)
 *  13  Horizontal Speed (x.xxx m/s)
 *  14  Vertical Speed (x.xxx m/s)
 *  15  Heading (x.xxx degrees)
 *  16  Checksum, begins with *
 */
int nmea_parse_powtlv(const char a[], const int aSize, gps_pos_t &pos, gps_vel_t &vel)
{
    (void)aSize;
    uint64_t TOWus;
    char *ptr = (char *)&a[8];	// $POWGPS,
    uint32_t temp;
    float horVel, courseMadeTrue;
    
    // 1 -	GPS Time valid
    ptr = ASCII_to_u32(&temp, ptr);

    // 2 -	GPS week number
    ptr = ASCII_to_u32(&(pos.week), ptr);

    // 3 -	GPS Time of Week (us)
    ptr = ASCII_to_u64(&TOWus, ptr);
    pos.timeOfWeekMs = TOWus/1000;	// convert to seconds

    // if time is not valid, set time to 0
    if (temp == 0) { pos.timeOfWeekMs = 0; pos.week = 0; }
    
    // 4 -	GPS leap seconds valid
    ptr = ASCII_to_u32(&temp, ptr);
    
    // 5 -	GPS leap seconds
    ptr = ASCII_to_u8(&(pos.leapS), ptr);

    // if LS not valid, set to 0
    if (temp == 0) { pos.leapS = 0; }

    // 6 -	Holdover flag (0=no holdover, 1=EGR is in holdover)
    ptr = ASCII_to_u32(&temp, ptr);

    // 7,8 -  Latitude ddmm.mmmm, North/South indicator (N/S)
    ptr = ASCII_DegMin_to_Lat(&(pos.lla[0]), ptr);
    
    // 9,10 -  Longitude dddmm.mmmm, East/West indicator (E/W)
    ptr = ASCII_DegMin_to_Lon(&(pos.lla[1]), ptr);

    // 11 - Altitude (x.xxx meters)
    ptr = ASCII_to_f64(&(pos.lla[2]), ptr);

    // 12 - Mean Sea Level (MSL) (x.xxx meters)
    ptr = ASCII_to_f32(&(pos.hMSL), ptr);

    // 13 - Horizontal Speed (x.xxx m/s)
    ptr = ASCII_to_f32(&horVel, ptr);

    // 14 - Vertical Speed (x.xxx m/s)
    ptr = ASCII_to_f32(&vel.vel[2], ptr);

    // 15 - Heading (x.xxx degrees)
    ptr = ASCII_to_f32(&courseMadeTrue, ptr);
    courseMadeTrue *= C_DEG2RAD_F;

    vel.vel[0] = horVel * cosf(courseMadeTrue);
    vel.vel[1] = horVel * sinf(courseMadeTrue);

    return 0;
}

/* G_RMC Message
* Provides speed (speed and course over ground)
*/
int nmea_parse_rmc(const char a[], int aSize, gps_vel_t &gpsVel, utc_time_t &utcTime, int utcWeekday, int leapS, uint32_t statusFlags)
{
    (void)aSize;
    char *ptr = (char *)&a[7];
    //$xxRMC,time,status,lat,NS,lon,EW,spd,cog,date,mv,mvEW,posMode,navStatus*cs<CR><LF>

    // 1 - UTC time HHMMSS.sss
    ptr = ASCII_UtcTimeToGpsTowMs(&gpsVel.timeOfWeekMs, &utcTime, ptr, utcWeekday, leapS);

    //Skip 5
    for(int i=0;i<5;++i)
    {
        ptr = ASCII_find_next_field(ptr);
    }

    //spd & cog
    float spdm_s = (float)atof(ptr) * C_KNOTS_METERS_F;
    ptr = ASCII_find_next_field(ptr);
    float cogRad = DEG2RAD((float)atof(ptr));

    //Speed data in NED
    gpsVel.vel[0] = spdm_s * cosf(cogRad);
    gpsVel.vel[1] = spdm_s * sinf(cogRad);
    gpsVel.vel[2] = 0;
    //dependencies_.gpsVel.sAcc = 0;
            
    //Indicate it is coming from NMEA
    gpsVel.status = GPS_STATUS_FLAGS_GPS_NMEA_DATA | statusFlags;

    return 0;	
}

int nmea_parse_vtg(const char a[], int aSize, gps_vel_t &vel, const double refLla[3])
{
    (void)aSize;
    char *ptr = (char *)&a[7];	// $GxVTG,

    // 1 - Track made good (degrees true)
    float courseMadeTrue;
    ptr = ASCII_to_f32(&courseMadeTrue, ptr);
    courseMadeTrue *= C_DEG2RAD_F;
    // 2 - T: track made good is relative to true north
    ptr = ASCII_find_next_field(ptr);

    // 3 - Track made good (degrees magnetic)
    ptr = ASCII_find_next_field(ptr);
    // 4 - M: track made good is relative to magnetic north 
    ptr = ASCII_find_next_field(ptr);

    // 5 - Speed, in knots
    float speed2dKnots;
    ptr = ASCII_to_f32(&speed2dKnots, ptr);
    float speed2dMps;
    speed2dMps = C_KNOTS_METERS_F * speed2dKnots;

    ixVector3 velNed;
    velNed[0] = speed2dMps * cosf(courseMadeTrue);
    velNed[1] = speed2dMps * sinf(courseMadeTrue);
    velNed[2] = 0.0f;
    if (vel.status & GPS_STATUS_FLAGS_GPS_NMEA_DATA)
    {	// NED velocity
        cpy_Vec3_Vec3(vel.vel, velNed);
    }
    else
    {	// ECEF velocity
        ixQuat qe2n;
        quat_ecef2ned(C_DEG2RAD_F*(float)refLla[0], C_DEG2RAD_F*(float)refLla[1], qe2n);
        quatRot(vel.vel, qe2n, velNed);
    }

    // 6 - N: speed is measured in knots
    ptr = ASCII_find_next_field(ptr);

    // 7 - Speed over ground in kilometers/hour (kph)
    ptr = ASCII_find_next_field(ptr);

    // 8 - K: speed over ground is measured in kph
    ptr = ASCII_find_next_field(ptr);

    // 9 - Mode indicator:
    // 		A: Autonomous mode
    // 		D: Differential mode
    // 		E: Estimated (dead reckoning) mode
    // 		M: Manual Input mode
    // 		S: Simulator mode
    // 		N: Data not valid
    ptr = ASCII_find_next_field(ptr);

    return 0;
}

int nmea_parse_zda(const char a[], int aSize, uint32_t &gpsTowMs, uint32_t &gpsWeek, utc_date_t &date, utc_time_t &time, int leapS)
{
    (void)aSize;
    char *ptr = (char *)&a[7];	// $GxZDA,

    // 1 - UTC time HHMMSS
    float second;
    ptr = ASCII_to_hours_minutes_seconds(&time.hour, &time.minute, &second, ptr);
    time.second = (int)second;
    time.millisecond = (int)(second*1000.0f) - 1000*time.second;

    // 2,3,4 - dd,mm,yyy (Day,Month,Year)
    ptr = ASCII_to_i32((int32_t*)&(date.day), ptr);
    ptr = ASCII_to_i32((int32_t*)&(date.month), ptr);
    ptr = ASCII_to_i32((int32_t*)&(date.year), ptr);

    // Convert UTC date and time to GPS time of week and number of weeks		
    int datetime[7] = { date.year, date.month, date.day, time.hour, time.minute, time.second, time.millisecond };		// year,month,day,hour,min,sec,msec
    UtcDateTimeToGpsTime(datetime, leapS, gpsTowMs, gpsWeek);
    date.weekday = gpsTowMsToUtcWeekday(gpsTowMs, leapS);

    // 5,6 - Local time zone offset from GMT (00,00)
    return 0;
}

void gsv_clear_const_mask()
{
    memset(&s_gsvMask, 0, sizeof(gsvMask_t));
}


