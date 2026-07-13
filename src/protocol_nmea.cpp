#include <stdint.h>
#include <stdarg.h>
#include <cctype>
#include <vector>
#include "protocol_nmea.h"
#include "time_conversion.h"
#include "ISPose.h"
#include "ISEarth.h"
#include "data_sets.h"
#include "util/md5.h"

#ifdef IS_IMX
#include "drivers/d_time.h"
#include "globals.h"
#endif

static int s_protocol_version = NMEA_PROTOCOL_2P3;  // Default to protocol version 2.3
static uint8_t s_gnssId = SAT_SV_GNSS_ID_GNSS;

#define HISTORY_SIZE    5

static struct  
{
    uint32_t    timeOfWeekMs;
    ixVector3   velNed;
    float       speed2dMpsHistory[HISTORY_SIZE];
    float       speed2dMps;
    float       speed2dKnots;
    bool        enableSpeedFilter;
} s_dataSpeed = {0};

uint8_t nmea2p3_svid_to_sigId(uint8_t gnssId, uint16_t svId);
bool gsv_freq_ena(gnss_sig_sv_t* sig);

static gsvMask_t s_gsvMask = {0};

//////////////////////////////////////////////////////////////////////////
// Utility functions
//////////////////////////////////////////////////////////////////////////

/**
 * @brief Sets the NMEA protocol version used to encode/decode messages (e.g. multi-frequency GSV/GNS field formats).
 *
 * @param protocol_version - protocol version identifier (see NMEA_PROTOCOL_2P3, NMEA_PROTOCOL_4P10, etc.)
 */
void nmea_set_protocol_version(int protocol_version)
{
    s_protocol_version = protocol_version;
}

/**
 * @brief Sets the default GNSS constellation ID used when generating the NMEA talker ID (e.g. for $GxGSV messages).
 *
 * @param gnssId - GNSS constellation ID (see eSatSvGnssId)
 */
void nmea_set_gnss_id(int gnssId)
{
    s_gnssId = gnssId;
}

/**
 * @brief Safe snprintf wrapper that prevents use of an invalid/negative size.
 * @note snprintf's size parameter (size_t) is unsigned and can wrap to a very large value if a negative bufSize is passed in; this guards against that.
 *
 * @param buf - output buffer
 * @param bufSize - size of output buffer in bytes
 * @param fmt - printf-style format string
 * @param ... - format arguments
 *
 * @return number of characters written (excluding null terminator), or 0 if bufSize <= 0
 */
int ssnprintf(char buf[], int bufSize, const char *fmt, ...)
{
    if (bufSize<=0) return 0; // Prevent snprintf w/ invalid size 
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

/**
 * @brief Appends a printf-style formatted field to an in-progress NMEA sentence buffer at the given running offset.
 *
 * @param buf[] - output buffer holding the sentence being built
 * @param bufSize - total size of output buffer in bytes
 * @param offset - [in/out] current write offset into buf; advanced by the number of characters written
 * @param fmt - printf-style format string (typically starts with "," to delimit the new field)
 * @param ... - format arguments
 */
void nmea_sprint(char buf[], int bufSize, int &offset, const char *fmt, ...)
{
    bufSize -= offset;
    if (bufSize<=0) return; // Prevent snprintf w/ invalid size
    buf += offset;
    va_list args;
    va_start(args, fmt);
    offset += VSNPRINTF(buf, bufSize, fmt, args);
    va_end(args);
}

/**
 * @brief Appends a comma-delimited unsigned integer field, printing an empty field (just the comma) when the value is zero.
 * @note Several NMEA sentences (e.g. $POWGPS/$POWTLV family) omit zero-valued numeric fields rather than printing "0".
 *
 * @param buf[] - output buffer holding the sentence being built
 * @param bufSize - total size of output buffer in bytes
 * @param offset - [in/out] current write offset into buf; advanced by the number of characters written
 * @param precision - minimum digit width (zero-padded) used when the value is non-zero
 * @param value - unsigned value to print
 */
void nmea_print_u32(char buf[], int bufSize, int &offset, int precision, uint32_t value)
{
    bufSize -= offset;
    if (bufSize<=0) return; // Prevent snprintf w/ invalid size
    buf += offset;

    if (value)
    {   // Non-zero
        offset += ssnprintf(buf, bufSize, ",%0*u", precision, value);
    }
    else
    {   // Print nothing for Zero
        buf[0] = ',';
        offset++;
    }
}

/**
 * @brief Appends a comma-delimited signed integer field, printing an empty field (just the comma) when the value is zero.
 * @note Value is passed/treated as uint32_t but formatted with "%ld" (signed); used for elevation/azimuth fields that may be negative.
 *
 * @param buf[] - output buffer holding the sentence being built
 * @param bufSize - total size of output buffer in bytes
 * @param offset - [in/out] current write offset into buf; advanced by the number of characters written
 * @param precision - minimum digit width (zero-padded) used when the value is non-zero
 * @param value - value to print, interpreted as signed
 */
void nmea_print_i32(char buf[], int bufSize, int &offset, int precision, uint32_t value)
{
    bufSize -= offset;
    if (bufSize<=0) return; // Prevent snprintf w/ invalid size
    buf += offset;

    if (value)
    {   // Non-zero
        offset += ssnprintf(buf, bufSize, ",%0*ld", precision, value);
    }
    else
    {   // Print nothing for Zero
        buf[0] = ',';
        offset++;
    }
}

/**
 * @brief Computes the NMEA XOR checksum (8-bit XOR of all bytes) used in the "*hh" trailer of a sentence.
 *
 * @param str - pointer to the first byte to checksum (typically the character after '$')
 * @param size - number of bytes to XOR
 *
 * @return XOR checksum value (only the low 8 bits are meaningful; printed as 2 hex digits)
 */
uint32_t nmea_compute_checksum(uint8_t* str, int size)
{
    uint32_t checksum = 0;
    
    uint8_t *end = str + size;
    for (uint8_t *ptr=str; ptr<end; ptr++)
    {
        checksum ^= *ptr;
    }

    return checksum;
}

/**
 * @brief Converts a GNSS constellation ID to its 2-character NMEA talker ID suffix (e.g. "GP", "GA", "GB").
 *
 * @param a - output buffer, must have room for 2 characters (not null-terminated by this function)
 * @param gnssId - GNSS constellation ID (see eSatSvGnssId): GPS/SBAS->"GP", Galileo->"GA", BeiDou->"GB", QZSS/IMES->"GQ", GLONASS->"GL", NavIC->"GI", unknown->"GN"
 *
 * @return number of characters written (always 2)
 */
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

/**
 * @brief Converts an incoming NMEA talker ID ("$Gx...") and (protocol-version-dependent) svId encoding into a GNSS constellation ID and signal ID.
 * @note For protocol versions older than NMEA 4.10, multi-frequency signals are encoded by offsetting svId by 256 or 512; this function decodes that offset back into sigId and normalizes svId, then also derives QZSS/SBAS sub-ranges from the raw GPS ("GP") talker svId range.
 *
 * @param a[] - incoming NMEA sentence buffer; a[2] holds the talker's constellation letter ('P','A','B','L','I','Q')
 * @param gnssId - [out] decoded GNSS constellation ID (see eSatSvGnssId)
 * @param svId - [in/out] satellite ID; on input may be offset-encoded (protocol < 4.10), on output is normalized to the plain PRN/slot number
 * @param sigId - [out] decoded signal ID (see eSatSvSigId), derived from the pre-normalization svId offset
 */
void talkerId_to_gnssId(const char a[], uint8_t &gnssId, uint16_t &svId, uint8_t &sigId)
{
    uint16_t svIdLast = svId;
    if (s_protocol_version < NMEA_PROTOCOL_4P10)
    {
        if (svId >= 512)
        {   // < NMEA 4.10 method of detecting mult-frequency
            svId -= 512;
        }
        else if (svId >= 256)
        {   // < NMEA 4.10 method of detecting mult-frequency
            svId -= 256;
        }
        else
        {
            sigId = 0;
        }
    }

    switch(a[2]) // $Gx
    {
        case 'P': // GPS, SBAS, QZSS
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
            else{ gnssId = SAT_SV_GNSS_ID_GPS; }        break;
        case 'A':    gnssId = SAT_SV_GNSS_ID_GAL;       break;
        case 'B':    gnssId = SAT_SV_GNSS_ID_BEI;       break;    
        case 'L':    gnssId = SAT_SV_GNSS_ID_GLO;       break;
        case 'I':    gnssId = SAT_SV_GNSS_ID_IRN;       break;
        case 'Q':    gnssId = SAT_SV_GNSS_ID_QZS;       break;
        default:    gnssId = SAT_SV_GNSS_ID_UNKNOWN;    break;
    }

    sigId = nmea2p3_svid_to_sigId(gnssId, svIdLast);
}

/**
 * @brief Writes the leading "$Gx" talker prefix (3 characters: '$' + 2-letter talker ID) for a new NMEA sentence.
 *
 * @param a - output buffer, must have room for at least 2 characters (aSize < 2 aborts)
 * @param aSize - size of output buffer in bytes
 * @param gnssId - GNSS constellation ID used to select the talker letters (see eSatSvGnssId); defaults to the last value set by nmea_set_gnss_id()
 *
 * @return number of characters written (3), or 0 if aSize < 2
 */
static int nmea_talker(char* a, int aSize, uint8_t gnssId=s_gnssId)
{
    if (aSize < 2)
    {
        return 0;
    }
    a[0] = '$';
    return gnssId_to_talkerId(a+1, gnssId) + 1;
}

/**
 * @brief Appends the NMEA sentence trailer: "*" + 2-digit hex checksum + "\r\n". The checksum is the XOR of all bytes between (but excluding) '$' and '*'.
 *
 * @param a - buffer containing the sentence built so far (starting with '$')
 * @param aSize - total size of the buffer in bytes
 * @param n - [in/out] current length of the sentence in a (excluding the leading '$' is NOT excluded from n, but IS excluded from the checksum); updated to include the appended trailer
 *
 * @return final sentence length n (including the appended trailer)
 */
int nmea_sprint_footer(char* a, int aSize, int &n)
{
    unsigned int checkSum = nmea_compute_checksum((uint8_t*)(a+1), n-1);
    n += ssnprintf(a+n, aSize-n, "*%.2X\r\n", checkSum);
    return n;
}

/**
 * @brief Parses a comma-delimited decimal field as an 8-bit unsigned integer and advances to the next field.
 *
 * @param val - [out] parsed value
 * @param ptr - pointer to the start of the field to parse
 *
 * @return pointer to the start of the next comma-delimited field
 */
char *ASCII_to_u8(uint8_t *val, char *ptr)
{
    val[0] = (uint8_t)atoi(ptr);    ptr = ASCII_find_next_field(ptr);
    return ptr;
}

/**
 * @brief Parses a comma-delimited decimal field as a 16-bit unsigned integer and advances to the next field.
 *
 * @param val - [out] parsed value
 * @param ptr - pointer to the start of the field to parse
 *
 * @return pointer to the start of the next comma-delimited field
 */
char *ASCII_to_u16(uint16_t *val, char *ptr)
{
    val[0] = (uint16_t)atoi(ptr);   ptr = ASCII_find_next_field(ptr);
    return ptr;
}

/**
 * @brief Parses a comma-delimited decimal field as a 32-bit unsigned integer and advances to the next field.
 *
 * @param val - [out] parsed value
 * @param ptr - pointer to the start of the field to parse
 *
 * @return pointer to the start of the next comma-delimited field
 */
char *ASCII_to_u32(uint32_t *val, char *ptr)
{
    val[0] = (uint32_t)atoi(ptr);   ptr = ASCII_find_next_field(ptr);
    return ptr;
}

/**
 * @brief Parses a comma-delimited options/bitmask field as a 32-bit unsigned integer, accepting either decimal or "0x"/"0X"-prefixed hexadecimal, and advances to the next field.
 * @note Used for the $ASCE options field (see RMC_OPTIONS_* bit definitions: port mask, persistence flag, NMEA speed-filter enable bits, etc.).
 *
 * @param options - [out] parsed bitmask value; left unmodified if the field is empty (i.e. next char is ',')
 * @param ptr - pointer to the start of the field to parse
 *
 * @return pointer to the start of the next comma-delimited field
 */
char *ASCII_options_to_u32(uint32_t *options, char *ptr)
{
    // check if next index is ','
    if (*ptr != ',')
    {
        // Check if string starts with "0x" or "0X" for hexadecimal
        if ((ptr[0] == '0') && (ptr[1] == 'x' || ptr[1] == 'X'))
            *options = (uint32_t)strtoul(ptr, NULL, 16);  // Parse as hexadecimal
        else
            *options = (uint32_t)atoi(ptr);               // Parse as decimal
    }

    ptr = ASCII_find_next_field(ptr);
    return ptr;
}

/**
 * @brief Parses a comma-delimited decimal field as a 64-bit unsigned integer and advances to the next field.
 *
 * @param val - [out] parsed value
 * @param ptr - pointer to the start of the field to parse
 *
 * @return pointer to the start of the next comma-delimited field
 */
char *ASCII_to_u64(uint64_t *val, char *ptr)
{
    char *endPtr = nullptr;
    val[0] = (uint64_t)std::strtoll(ptr, &endPtr, 10);
    ptr = ASCII_find_next_field(ptr);
    return ptr;
}

/**
 * @brief Parses a comma-delimited decimal field as a signed 32-bit integer and advances to the next field.
 *
 * @param val - [out] parsed value
 * @param ptr - pointer to the start of the field to parse
 *
 * @return pointer to the start of the next comma-delimited field
 */
char *ASCII_to_i32(int32_t *val, char *ptr)
{
    val[0] = (int32_t)atoi(ptr);    ptr = ASCII_find_next_field(ptr);
    return ptr;
}

/**
 * @brief Parses a comma-delimited field as a 32-bit float and advances to the next field.
 *
 * @param vec - [out] parsed value
 * @param ptr - pointer to the start of the field to parse
 *
 * @return pointer to the start of the next comma-delimited field
 */
char *ASCII_to_f32(float *vec, char *ptr)
{
    vec[0] = strtof(ptr, nullptr);  ptr = ASCII_find_next_field(ptr);
    return ptr;
}

/**
 * @brief Parses a comma-delimited field as a 64-bit double and advances to the next field.
 *
 * @param vec - [out] parsed value
 * @param ptr - pointer to the start of the field to parse
 *
 * @return pointer to the start of the next comma-delimited field
 */
char *ASCII_to_f64(double *vec, char *ptr)
{
    vec[0] = atof(ptr);             ptr = ASCII_find_next_field(ptr);
    return ptr;
}

/**
 * @brief Parses a "major.minor.rev.build" (4-part dot-separated) version field into a 4-byte array and advances to the next field.
 *
 * @param vec[] - [out] 4-element version byte array {major, minor, revision, build}
 * @param ptr - pointer to the start of the field to parse
 *
 * @return pointer to the start of the next comma-delimited field
 */
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

/**
 * @brief Parses 3 consecutive comma-delimited float fields into a 3-element vector and advances past them.
 *
 * @param vec[] - [out] 3-element float vector
 * @param ptr - pointer to the start of the first field to parse
 *
 * @return pointer to the start of the field following the 3 parsed fields
 */
char *ASCII_to_vec3f(float vec[], char *ptr)
{
    vec[0] = strtof(ptr, nullptr);  ptr = ASCII_find_next_field(ptr);
    vec[1] = strtof(ptr, nullptr);  ptr = ASCII_find_next_field(ptr);
    vec[2] = strtof(ptr, nullptr);  ptr = ASCII_find_next_field(ptr);
    return ptr;
}

/**
 * @brief Parses 4 consecutive comma-delimited float fields into a 4-element vector (e.g. a quaternion) and advances past them.
 *
 * @param vec[] - [out] 4-element float vector
 * @param ptr - pointer to the start of the first field to parse
 *
 * @return pointer to the start of the field following the 4 parsed fields
 */
char *ASCII_to_vec4f(float vec[], char *ptr)
{
    vec[0] = strtof(ptr, nullptr);  ptr = ASCII_find_next_field(ptr);
    vec[1] = strtof(ptr, nullptr);  ptr = ASCII_find_next_field(ptr);
    vec[2] = strtof(ptr, nullptr);  ptr = ASCII_find_next_field(ptr);
    vec[3] = strtof(ptr, nullptr);  ptr = ASCII_find_next_field(ptr);
    return ptr;
}

/**
 * @brief Parses 3 consecutive comma-delimited double fields into a 3-element vector and advances past them.
 *
 * @param vec[] - [out] 3-element double vector
 * @param ptr - pointer to the start of the first field to parse
 *
 * @return pointer to the start of the field following the 3 parsed fields
 */
char *ASCII_to_vec3d(double vec[], char *ptr)
{
    vec[0] = atof(ptr);     ptr = ASCII_find_next_field(ptr);
    vec[1] = atof(ptr);     ptr = ASCII_find_next_field(ptr);
    vec[2] = atof(ptr);     ptr = ASCII_find_next_field(ptr);
    return ptr;
}

/**
 * @brief Parses a hex-encoded MD5 hash string field into a 4-word (128-bit) hash and advances to the next field.
 *
 * @param md5hash[4] - [out] 128-bit MD5 hash, as 4 32-bit words
 * @param ptr - pointer to the start of the field to parse
 *
 * @return pointer to the start of the next comma-delimited field
 */
char *ASCII_to_MD5(uint32_t md5hash[4], char *ptr)
{
    md5_from_char_array(*(md5hash_t*)md5hash, ptr);
    ptr = ASCII_find_next_field(ptr);
    return ptr;
}

/**
 * @brief Parses an NMEA latitude field pair: "ddmm.mmmm" degrees+minutes followed by an "N"/"S" hemisphere field, into signed decimal degrees.
 *
 * @param vec - [out] latitude in decimal degrees, negative for southern hemisphere
 * @param ptr - pointer to the start of the "ddmm.mmmm" field
 *
 * @return pointer to the start of the field following the hemisphere character
 */
char *ASCII_DegMin_to_Lat(double *vec, char *ptr)
{
    int degrees;
    SSCANF(ptr, "%02d", &degrees);  ptr += 2;
    double minutes = atof(ptr);     ptr = ASCII_find_next_field(ptr);
    double decdegrees = ((double)degrees) + (minutes*0.01666666666666666666666666666666666);
    if (ptr[0] == 'S')  { vec[0] = -decdegrees; } // south
    else                { vec[0] =  decdegrees; } // north
    ptr += 2;
    
    return ptr;
}

/**
 * @brief Parses an NMEA longitude field pair: "dddmm.mmmm" degrees+minutes followed by an "E"/"W" hemisphere field, into signed decimal degrees.
 *
 * @param vec - [out] longitude in decimal degrees, negative for western hemisphere
 * @param ptr - pointer to the start of the "dddmm.mmmm" field
 *
 * @return pointer to the start of the field following the hemisphere character
 */
char *ASCII_DegMin_to_Lon(double *vec, char *ptr)
{
    int degrees;
    SSCANF(ptr, "%03d", &degrees);  ptr += 3;
    double minutes = atof(ptr);     ptr = ASCII_find_next_field(ptr);
    double decdegrees = ((double)degrees) + (minutes*0.01666666666666666666666666666666666);
    if (ptr[0] == 'W')  { vec[0] = -decdegrees; } // west
    else                { vec[0] =  decdegrees; } // east
    ptr += 2;

    return ptr;
}

/**
 * @brief Copies a comma-delimited text field into a fixed-size null-terminated destination buffer, truncating if necessary.
 *
 * @param dst - [out] destination buffer, must be at least max_len bytes
 * @param ptr - pointer to the start of the field to copy
 * @param max_len - size of dst in bytes, including the null terminator
 *
 * @return pointer to the start of the next comma-delimited field
 */
char *ASCII_to_char_array(char *dst, char *ptr, int max_len)
{
    char *ptr2 = ASCII_find_next_field(ptr);
    int len = _MIN(max_len, (int)(ptr2-ptr)) - 1;
    len = _MAX(0, len);        // prevent negative
    memcpy(dst, ptr, len);
    dst[len] = 0;            // Must be null terminated
    return ptr2;
}

/**
 * @brief Parses an NMEA "HHMMSS.sss" UTC time field into separate hour, minute, and floating-point seconds components.
 * @note A 0.05ms bias is added to the seconds value to counteract float-conversion aliasing when the value is later truncated.
 *
 * @param hours - [out] UTC hour (0-23)
 * @param minutes - [out] UTC minute (0-59)
 * @param seconds - [out] UTC seconds + fractional seconds
 * @param ptr - pointer to the start of the "HHMMSS.sss" field
 *
 * @return pointer to the start of the next comma-delimited field
 */
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

/**
 * @brief Parses an NMEA "HHMMSS.sss" UTC time field, populates the utc_time_t breakdown, and converts it to a GPS time-of-week in milliseconds.
 * @note A 0.05ms bias is added to the fractional seconds to counteract float-conversion aliasing before truncation to milliseconds.
 *
 * @param gpsTimeOfWeekMs - [out] resulting GPS time of week, in milliseconds
 * @param utcTime - [out] parsed UTC hour/minute/second/millisecond
 * @param ptr - pointer to the start of the "HHMMSS.sss" field
 * @param utcWeekday - UTC day of week (0=Sunday) used, together with the time, to compute the GPS time of week
 * @param leapS - current GPS-UTC leap second offset
 *
 * @return pointer to the start of the next comma-delimited field
 */
char *ASCII_UtcTimeToGpsTowMs(uint32_t *gpsTimeOfWeekMs, utc_time_t *utcTime, char *ptr, int utcWeekday, uint32_t leapS)
{
    // HHMMSS.sss
    float fsecond;
    SSCANF(ptr, "%02d%02d%f", &utcTime->hour, &utcTime->minute, &fsecond);
    fsecond += 0.00005f;    /// add a 0.05ms to address float-conversion aliasing
    utcTime->second = (uint32_t)fsecond;
    fsecond *= 1000.0f;
    utcTime->millisecond = (uint32_t)fsecond;
    utcTime->millisecond = utcTime->millisecond%1000;
    utcTimeToGpsTowMs(utcTime, utcWeekday, gpsTimeOfWeekMs, leapS);
    ptr = ASCII_find_next_field(ptr);
    return ptr;
}

/**
 * @brief Advances a pointer within an NMEA sentence to the start of the next comma-delimited field.
 *
 * @param str - pointer into the current field
 *
 * @return pointer to the start of the next field, or to the terminating '\\0'/'*' if no more fields remain (does not move past '*')
 */
char *ASCII_find_next_field(char *str)
{
    while (*str != 0 && *str != ',' && *str != '*') // move down looking for end of string.
        ++str;

    if (*str == ',') //move past comma (if not at end of string)
        ++str;

    return str;
}

/**
 * @brief Converts a value in NMEA "ddmm.mmmm" (degrees + decimal minutes packed together) format to plain decimal degrees.
 *
 * @param ddmm - value in "ddmm.mmmm" format (degrees in the integer hundreds/thousands digits, minutes in the remainder)
 *
 * @return equivalent value in decimal degrees
 */
double ddmm2deg(double ddmm)
{
    double deg = (int)ddmm / 100 ;
    ddmm -= deg * 100 ;
    return deg + (ddmm / 60) ;
}

/**
 * @brief Replaces the bits of *status covered by mask with the corresponding bits of state, leaving all other bits unchanged.
 * @note Used to update a sub-field (e.g. fix type, flags, or sats-used) of a gnss_pos_t::status bitmask (see eGnssStatus) without disturbing the other sub-fields.
 *
 * @param status - [in/out] status bitmask to update
 * @param state - new value for the bits selected by mask (bits outside mask are ignored)
 * @param mask - bitmask selecting which bits of *status are replaced
 */
void set_gnssPos_status_mask(uint32_t *status, uint32_t state, uint32_t mask)
{
    *status &= ~mask;
    *status |= state & mask;
}

/**
 * @brief Enables periodic broadcast of an NMEA message ID by setting its bit in the enable bitmask and recording its period multiple.
 * @note If periodMultiple is 0, the message is still flagged to send once; the caller is expected to clear the enable bit after that single send.
 *
 * @param bits - [in/out] per-message enable bitmask (bit N corresponds to nmeaId N, see eNmeaMsgIdInx)
 * @param period[] - [out] per-message period-multiple array, indexed by nmeaId
 * @param nmeaId - NMEA message ID (index into eNmeaMsgIdInx) to enable
 * @param periodMultiple - output period, as a multiple of the base broadcast period (0 = send once)
 */
void nmea_enable_stream(uint32_t& bits, uint8_t* period, uint32_t nmeaId, uint8_t periodMultiple)
{
    uint32_t nmeaBits = (1<<nmeaId);
    period[nmeaId] = periodMultiple;

    // Always set bit.  If period multiple is zero, this bit will get cleared after sending message once.
    bits |= (nmeaBits);
}

//////////////////////////////////////////////////////////////////////////
// Binary to NMEA
//////////////////////////////////////////////////////////////////////////

/**
 * @brief Builds a $INFO proprietary NMEA message reporting device identification and firmware build information.
 *
 * @param a[] - output buffer
 * @param aSize - size of output buffer
 * @param info - device info structure (dev_info_t) to encode
 *
 * @note output message format:
 *  0   Message ID $INFO
 *  1   Serial number
 *  2   Hardware version (major.minor.rev.build)
 *  3   Firmware version (major.minor.rev.build)
 *  4   Build number
 *  5   Communications protocol version (major.minor.rev.build)
 *  6   Repository revision
 *  7   Manufacturer name string
 *  8   Build date (YYYY-MM-DD)
 *  9   Build time (HH:MM:SS.ss)
 *  10  Additional info string
 *  11  Hardware type: 1=uINS, 2=EVB, 3=IMX, 4=GPX (see eIsHardwareType)
 *  12  Hardware run state: 0=unknown, 1=bootloader, 2=app (see eHdwRunStates)
 *  13  Build type character: 0/space=production, 'c'=release candidate, 'b'=beta, 'a'=alpha, 'd'=developer, 's'=snapshot, '^'=dirty
 *  14  Build flags bitmask (see eBuildFlags):
 *        bit0 (0x1) = debug mode
 *        bit1 (0x2) = dirty working tree
 *  15  Checksum, begins with *
 *
 * @return length of the generated NMEA sentence, including the checksum trailer
 */
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
        ",%d"                   // 14

        , (int)info.serialNumber                                                                // 1
        , info.hardwareVer[0], info.hardwareVer[1], info.hardwareVer[2], info.hardwareVer[3]    // 2
        , info.firmwareVer[0], info.firmwareVer[1], info.firmwareVer[2], info.firmwareVer[3]    // 3
        , (int)info.buildNumber                                                                 // 4
        , info.protocolVer[0], info.protocolVer[1], info.protocolVer[2], info.protocolVer[3]    // 5
        , (int)info.repoRevision                                                                // 6
        , info.manufacturer                                                                     // 7
        , info.buildYear+2000, info.buildMonth, info.buildDay                                   // 8
        , info.buildHour, info.buildMinute, info.buildSecond, info.buildMillisecond             // 9
        , info.addInfo                                                                          // 10
        , info.hardwareType                                                                     // 11
        , info.hdwRunState                                                                      // 12
        , (info.buildType ? info.buildType : ' ')                                               // 13
        , info.buildFlags                                                                       // 14
    );

    return nmea_sprint_footer(a, aSize, n);
}

/**
 * @brief Builds a $ASCE (Ask/Set Communications Enable) response NMEA message listing the currently enabled NMEA messages and their broadcast periods.
 *
 * @param a[] - output buffer
 * @param aSize - size of output buffer
 * @param nRMC - per-port NMEA broadcast configuration (nmeaBits enable bitmask + nmeaPeriod[] array) to encode
 *
 * @note output message format: $ASCE,port,{msgID,msgPeriod}...*cs
 *  0    Message ID $ASCE
 *  1    Port index this response applies to (always 0; the currently-configured port)
 *  2n   msgID  - NMEA message ID of an enabled message (index into eNmeaMsgIdInx, see nRMC->nmeaBits bit position)
 *  2n+1 msgPeriod - broadcast period multiple for that message (see nRMC->nmeaPeriod[msgID])
 *      (the msgID,msgPeriod pair repeats once for every message with its enable bit set in nmeaBits and a non-zero period, up to MAX_nmeaBroadcastMsgPairs)
 *  last Checksum, begins with *
 *
 * @return length of the generated NMEA sentence, including the checksum trailer
 */
int nmea_ASCE(char a[], const int aSize, rmcNmea_t* nRMC)
{
    nmeaBroadcastMsgPair_t pairs[MAX_nmeaBroadcastMsgPairs];
    int activeRMC = 0;

    for (int i = 0; (i < NMEA_MSG_ID_COUNT) && (activeRMC < MAX_nmeaBroadcastMsgPairs); i++)
    {
        if (((nRMC->nmeaBits & (0x01 << i)) != 0) && (nRMC->nmeaPeriod[i] > 0))
        {
            pairs[activeRMC].msgID = i;
            pairs[activeRMC].msgPeriod = nRMC->nmeaPeriod[i];
            activeRMC++;
        }
    }

    // Base msg with current port set
    int n = ssnprintf(a, aSize, "$ASCE,0");

    // finish populating msg
    for (int i = 0; (i < activeRMC) && (i < MAX_nmeaBroadcastMsgPairs); i++)
        n += ssnprintf(a+n, aSize-n, ",%d,%d", pairs[i].msgID, pairs[i].msgPeriod);

    return nmea_sprint_footer(a, aSize, n);
}


/**
 * @brief Builds a $PTOW proprietary NMEA message reporting the IMU and INS time-of-week values used to timestamp navigation output.
 *
 * @param a[] - output buffer
 * @param aSize - size of output buffer
 * @param imuTow - IMU time of week, in seconds
 * @param insTow - INS/AHRS time of week, in seconds
 * @param gpsWeek - GPS week number
 *
 * @note output message format:
 *  0   Message ID $PTOW
 *  1   IMU time of week (seconds)
 *  2   INS time of week (seconds)
 *  3   GPS week number
 *  4   Checksum, begins with *
 *
 * @return length of the generated NMEA sentence, including the checksum trailer
 */
int tow_to_nmea_ptow(char a[], const int aSize, double imuTow, double insTow, unsigned int gpsWeek)
{
    int n = ssnprintf(a, aSize, "$PTOW");
    nmea_sprint(a, aSize, n, ",%.6lf", imuTow); // 1
    nmea_sprint(a, aSize, n, ",%.6lf", insTow); // 2
    nmea_sprint(a, aSize, n, ",%u", gpsWeek);   // 3

    return nmea_sprint_footer(a, aSize, n);
}

/**
 * @brief Builds a $PIMU (or $PRIMU when name="$PRIMU") proprietary NMEA message reporting raw/reference IMU angular rate and acceleration.
 *
 * @param a[] - output buffer
 * @param aSize - size of output buffer
 * @param imu - IMU sample (imu_t) to encode
 * @param name[] - message name to write as field 0, e.g. "$PIMU" or "$PRIMU" (allows this builder to be shared by both message types)
 *
 * @note output message format:
 *  0   Message ID (name, e.g. $PIMU or $PRIMU)
 *  1   Time since system power-up (seconds)
 *  2   PQR angular rate, X/roll axis (rad/s)
 *  3   PQR angular rate, Y/pitch axis (rad/s)
 *  4   PQR angular rate, Z/yaw axis (rad/s)
 *  5   Linear acceleration, X axis (m/s^2)
 *  6   Linear acceleration, Y axis (m/s^2)
 *  7   Linear acceleration, Z axis (m/s^2)
 *  8   Checksum, begins with *
 *
 * @return length of the generated NMEA sentence, including the checksum trailer
 */
int nmea_pimu(char a[], const int aSize, imu_t &imu, const char name[])
{
    int n = ssnprintf(a, aSize, "%s", name);
    nmea_sprint(a, aSize, n, ",%.3lf", imu.time);       // 1

    nmea_sprint_f(a, aSize, n, ",%.4f", imu.I.pqr[0]);    // 2
    nmea_sprint_f(a, aSize, n, ",%.4f", imu.I.pqr[1]);    // 3
    nmea_sprint_f(a, aSize, n, ",%.4f", imu.I.pqr[2]);    // 4

    nmea_sprint_f(a, aSize, n, ",%.3f", imu.I.acc[0]);    // 5
    nmea_sprint_f(a, aSize, n, ",%.3f", imu.I.acc[1]);    // 6
    nmea_sprint_f(a, aSize, n, ",%.3f", imu.I.acc[2]);    // 7

    return nmea_sprint_footer(a, aSize, n);
}

/**
 * @brief Builds a $PPIMU proprietary NMEA message reporting preintegrated IMU (delta theta / delta velocity) data.
 *
 * @param a[] - output buffer
 * @param aSize - size of output buffer
 * @param pimu - preintegrated IMU sample (pimu_t) to encode
 *
 * @note output message format:
 *  0   Message ID $PPIMU
 *  1   Time since system power-up (seconds)
 *  2   Delta theta, X/roll axis (rad), integrated over dt
 *  3   Delta theta, Y/pitch axis (rad), integrated over dt
 *  4   Delta theta, Z/yaw axis (rad), integrated over dt
 *  5   Delta velocity, X axis (m/s), integrated over dt
 *  6   Delta velocity, Y axis (m/s), integrated over dt
 *  7   Delta velocity, Z axis (m/s), integrated over dt
 *  8   Integration period dt (seconds)
 *  9   Checksum, begins with *
 *
 * @return length of the generated NMEA sentence, including the checksum trailer
 */
int nmea_ppimu(char a[], const int aSize, pimu_t &pimu)
{
    int n = ssnprintf(a, aSize, "$PPIMU");
    nmea_sprint(a, aSize, n, ",%.3lf", pimu.time);      // 1

    nmea_sprint_f(a, aSize, n, ",%.4f", pimu.theta[0]);   // 2
    nmea_sprint_f(a, aSize, n, ",%.4f", pimu.theta[1]);   // 3
    nmea_sprint_f(a, aSize, n, ",%.4f", pimu.theta[2]);   // 4

    nmea_sprint_f(a, aSize, n, ",%.4f", pimu.vel[0]);     // 5
    nmea_sprint_f(a, aSize, n, ",%.4f", pimu.vel[1]);     // 6
    nmea_sprint_f(a, aSize, n, ",%.4f", pimu.vel[2]);     // 7

    nmea_sprint_f(a, aSize, n, ",%.3f", pimu.dt);         // 8

    return nmea_sprint_footer(a, aSize, n);
}

/**
 * @brief Builds a $PINS1 proprietary NMEA message reporting the primary INS navigation solution (Euler angles + NED-referenced position/velocity).
 *
 * @param a[] - output buffer
 * @param aSize - size of output buffer
 * @param ins1 - INS solution (ins_1_t) to encode
 *
 * @note output message format:
 *  0   Message ID $PINS1
 *  1   GPS time of week (seconds)
 *  2   GPS week number
 *  3   INS status bitmask (insStatus), see eInsStatusFlags in data_sets.h:
 *        bit0        = heading estimate coarse
 *        bit1        = velocity estimate coarse
 *        bit2        = position estimate coarse
 *        bit3        = velocity aided by wheel sensor
 *        bit4        = heading estimate fine
 *        bit5        = velocity estimate fine
 *        bit6        = position estimate fine
 *        bit7        = heading aided by GNSS
 *        bit8        = position aided by GNSS
 *        bit9        = GNSS update event occurred in solution
 *        bit10       = reference IMU used in EKF
 *        bit11       = heading aided by magnetic heading
 *        bit12       = nav mode (set) vs. AHRS mode (cleared)
 *        bit13       = stationary mode
 *        bit14       = velocity aided by GNSS velocity
 *        bit15       = kinematic calibration good
 *        bits[19:16] = solution status (INS_STATUS_SOLUTION_MASK):
 *                        0 = off
 *                        1 = aligning
 *                        3 = nav mode, solution good
 *                        4 = nav mode, attitude uncertainty exceeded threshold
 *                        5 = AHRS mode, solution good
 *                        6 = AHRS mode, attitude uncertainty exceeded threshold
 *                        7 = VRS mode, roll/pitch good
 *                        8 = VRS mode, roll/pitch uncertainty exceeded threshold
 *        bit20       = RTK compassing baseline unset
 *        bit21       = RTK compassing baseline bad
 *        bit22       = magnetometer recalibrating
 *        bit23       = magnetometer interference / bad or no calibration
 *        bits[25:24] = GNSS navigation fix status (INS_STATUS_GNSS_NAV_FIX_MASK, see eGnssNavFixStatus): 0=none, 1=3D, 2=RTK float, 3=RTK fix
 *        bit26       = RTK compassing heading valid
 *        bit27       = RTK error: raw GNSS observations/ephemeris invalid or not received
 *        bits[29:28] = RTK error: base data missing/moving/invalid (INS_STATUS_RTK_ERR_BASE_MASK)
 *        bit30       = RTOS task ran longer than its allotted period
 *        bit31       = general fault (see sys_params_t.genFaultCode)
 *  4   Hardware status bitmask (hdwStatus), see eHdwStatusFlags in data_sets.h:
 *        bits[1:0] = motion detected (gyro/accel)
 *        bits[3:2] = IMU fault rejection (gyro/accel)
 *        bit4      = GNSS satellite signal valid
 *        bit5      = strobe input event occurred
 *        bit6      = GPS time of week valid
 *        bit7      = reference IMU data being received
 *        bits[11:8] = sensor saturation (gyro/accel/mag/baro)
 *  5   Roll (rad)
 *  6   Pitch (rad)
 *  7   Yaw (rad)
 *  8   Velocity U, body X axis (m/s)
 *  9   Velocity V, body Y axis (m/s)
 *  10  Velocity W, body Z axis (m/s)
 *  11  Latitude (deg)
 *  12  Longitude (deg)
 *  13  Ellipsoid altitude (m)
 *  14  Position North offset from reference LLA (m)
 *  15  Position East offset from reference LLA (m)
 *  16  Position Down offset from reference LLA (m)
 *  17  Checksum, begins with *
 *
 * @return length of the generated NMEA sentence, including the checksum trailer
 */
int nmea_pins1(char a[], const int aSize, ins_1_t &ins1)
{
    int n = ssnprintf(a, aSize, "$PINS1");
    nmea_sprint(a, aSize, n, ",%.3lf", ins1.timeOfWeek);            // 1

    nmea_sprint(a, aSize, n, ",%u", (unsigned int)ins1.week);       // 2
    nmea_sprint(a, aSize, n, ",%u", (unsigned int)ins1.insStatus);  // 3
    nmea_sprint(a, aSize, n, ",%u", (unsigned int)ins1.hdwStatus);  // 4

    nmea_sprint_f(a, aSize, n, ",%.4f", ins1.theta[0]);             // 5
    nmea_sprint_f(a, aSize, n, ",%.4f", ins1.theta[1]);             // 6
    nmea_sprint_f(a, aSize, n, ",%.4f", ins1.theta[2]);             // 7

    nmea_sprint_f(a, aSize, n, ",%.3f", ins1.uvw[0]);               // 8
    nmea_sprint_f(a, aSize, n, ",%.3f", ins1.uvw[1]);               // 9
    nmea_sprint_f(a, aSize, n, ",%.3f", ins1.uvw[2]);               // 10

    nmea_sprint(a, aSize, n, ",%.8lf", ins1.lla[0]);                // 11
    nmea_sprint(a, aSize, n, ",%.8lf", ins1.lla[1]);                // 12
    nmea_sprint(a, aSize, n, ",%.3lf", ins1.lla[2]);                // 13

    nmea_sprint_f(a, aSize, n, ",%.3f", ins1.ned[0]);               // 14
    nmea_sprint_f(a, aSize, n, ",%.3f", ins1.ned[1]);               // 15
    nmea_sprint_f(a, aSize, n, ",%.3f", ins1.ned[2]);               // 16

    return nmea_sprint_footer(a, aSize, n);
}

/**
 * @brief Builds a $PINS2 proprietary NMEA message reporting the primary INS navigation solution (quaternion attitude + LLA position).
 *
 * @param a[] - output buffer
 * @param aSize - size of output buffer
 * @param ins2 - INS solution (ins_2_t) to encode
 *
 * @note output message format:
 *  0   Message ID $PINS2
 *  1   GPS time of week (seconds)
 *  2   GPS week number
 *  3   INS status bitmask (insStatus) - see eInsStatusFlags in data_sets.h (same layout as $PINS1 field 3)
 *  4   Hardware status bitmask (hdwStatus) - see eHdwStatusFlags in data_sets.h (same layout as $PINS1 field 4)
 *  5   Attitude quaternion W component (body-to-NED, qn2b)
 *  6   Attitude quaternion X component
 *  7   Attitude quaternion Y component
 *  8   Attitude quaternion Z component
 *  9   Velocity U, body X axis (m/s)
 *  10  Velocity V, body Y axis (m/s)
 *  11  Velocity W, body Z axis (m/s)
 *  12  Latitude (deg)
 *  13  Longitude (deg)
 *  14  Ellipsoid altitude (m)
 *  15  Checksum, begins with *
 *
 * @return length of the generated NMEA sentence, including the checksum trailer
 */
int nmea_pins2(char a[], const int aSize, ins_2_t &ins2)
{
    int n = ssnprintf(a, aSize, "$PINS2");
    nmea_sprint(a, aSize, n, ",%.3lf", ins2.timeOfWeek);                // 1

    nmea_sprint(a, aSize, n, ",%u", (unsigned int)ins2.week);           // 2
    nmea_sprint(a, aSize, n, ",%u", (unsigned int)ins2.insStatus);      // 3
    nmea_sprint(a, aSize, n, ",%u", (unsigned int)ins2.hdwStatus);      // 4

    nmea_sprint_f(a, aSize, n, ",%.4f", ins2.qn2b[0]);                  // 5
    nmea_sprint_f(a, aSize, n, ",%.4f", ins2.qn2b[1]);                  // 6
    nmea_sprint_f(a, aSize, n, ",%.4f", ins2.qn2b[2]);                  // 7
    nmea_sprint_f(a, aSize, n, ",%.4f", ins2.qn2b[3]);                  // 8

    nmea_sprint_f(a, aSize, n, ",%.3f", ins2.uvw[0]);                   // 9
    nmea_sprint_f(a, aSize, n, ",%.3f", ins2.uvw[1]);                   // 10
    nmea_sprint_f(a, aSize, n, ",%.3f", ins2.uvw[2]);                   // 11

    nmea_sprint(a, aSize, n, ",%.8lf", ins2.lla[0]);                    // 12
    nmea_sprint(a, aSize, n, ",%.8lf", ins2.lla[1]);                    // 13
    nmea_sprint(a, aSize, n, ",%.3lf", ins2.lla[2]);                    // 14

    return nmea_sprint_footer(a, aSize, n);
}

/**
 * @brief Builds a $PSTRB proprietary NMEA message reporting a strobe (external event) input timestamp.
 *
 * @param a[] - output buffer
 * @param aSize - size of output buffer
 * @param strobe - strobe event data (strobe_in_time_t) to encode
 *
 * @note output message format:
 *  0   Message ID $PSTRB
 *  1   GPS week number
 *  2   GPS time of week (milliseconds)
 *  3   Strobe input pin number
 *  4   Strobe event count (increments each time the pin is triggered)
 *  5   Checksum, begins with *
 *
 * @return length of the generated NMEA sentence, including the checksum trailer
 */
int nmea_pstrb(char a[], const int aSize, strobe_in_time_t &strobe)
{
    int n = ssnprintf(a, aSize, "$PSTRB");
    nmea_sprint(a, aSize, n, ",%u", (unsigned int)strobe.week);         // 1
    nmea_sprint(a, aSize, n, ",%u", (unsigned int)strobe.timeOfWeekMs); // 2
    nmea_sprint(a, aSize, n, ",%u", (unsigned int)strobe.pin);          // 3
    nmea_sprint(a, aSize, n, ",%u", (unsigned int)strobe.count);        // 4

    return nmea_sprint_footer(a, aSize, n);
}

/**
 * @brief Builds a $PGPSP proprietary NMEA message reporting the raw GNSS position and velocity solution (with accuracy/DOP metrics).
 *
 * @param a[] - output buffer
 * @param aSize - size of output buffer
 * @param pos - GNSS position data (gnss_pos_t) to encode
 * @param vel - GNSS velocity data (gnss_vel_t) to encode
 *
 * @note output message format:
 *  0   Message ID $PGPSP
 *  1   GPS time of week (milliseconds)
 *  2   GPS week number
 *  3   GNSS status bitmask (status), see eGnssStatus in data_sets.h:
 *        bits[7:0]   = satellites used (deprecated, see satsUsed)
 *        bits[12:8]  = fix type (GNSS_STATUS_FIX_MASK):
 *                        0  = none
 *                        1  = dead reckoning only
 *                        2  = 2D
 *                        3  = 3D
 *                        4  = GNSS + dead reckoning
 *                        5  = time only
 *                        8  = DGPS
 *                        9  = SBAS
 *                        10 = RTK single
 *                        11 = RTK float
 *                        12 = RTK fix
 *        bit13       = GNSS2 RTK compassing baseline bad
 *        bit14       = GNSS2 RTK compassing baseline unset
 *        bit15       = data sourced from NMEA (GNSS_STATUS_FLAGS_GNSS_NMEA_DATA); velocity is NED instead of ECEF
 *        bit16       = fix OK (within DOP/accuracy limits)
 *        bit17       = DGPS used
 *        bit18       = RTK fix-and-hold
 *        bit20       = GNSS1 RTK precision positioning mode enabled
 *        bit21       = static mode
 *        bit22       = GNSS2 RTK moving-base (compassing) mode enabled
 *        bit23       = GNSS1 RTK error: raw observations/ephemeris invalid or not received
 *        bits[25:24] = GNSS1 RTK base-position error (GNSS_STATUS_FLAGS_GNSS1_RTK_BASE_POSITION_MASK): 1=data missing, 2=base moving, 3=base position invalid
 *        bit26       = GNSS1 RTK position valid (fixed-ambiguity, <6cm horizontal accuracy)
 *        bit27       = GNSS2 RTK compassing heading valid
 *        bit28       = time synchronized by GNSS PPS
 *  4   Latitude (deg)
 *  5   Longitude (deg)
 *  6   Ellipsoid altitude (m)
 *  7   Height above mean sea level, MSL (m)
 *  8   Position dilution of precision (pDOP)
 *  9   Horizontal accuracy estimate (m)
 *  10  Vertical accuracy estimate (m)
 *  11  Velocity, X/North axis (m/s)
 *  12  Velocity, Y/East axis (m/s)
 *  13  Velocity, Z/Down axis (m/s)
 *  14  Speed accuracy estimate (m/s)
 *  15  Mean carrier-to-noise ratio (dB-Hz) across tracked signals
 *  16  GPS time-of-week offset/bias (seconds)
 *  17  GPS leap seconds (GPS-UTC offset)
 *  18  Checksum, begins with *
 *
 * @return length of the generated NMEA sentence, including the checksum trailer
 */
int nmea_pgpsp(char a[], const int aSize, gnss_pos_t &pos, gnss_vel_t &vel)
{
    int n = ssnprintf(a, aSize, "$PGPSP");
    nmea_sprint(a, aSize, n, ",%u", (unsigned int)pos.timeOfWeekMs);    // 1
    nmea_sprint(a, aSize, n, ",%u", (unsigned int)pos.week);            // 2
    nmea_sprint(a, aSize, n, ",%u", (unsigned int)pos.status);          // 3

    nmea_sprint(a, aSize, n, ",%.8lf", pos.lla[0]);                     // 4
    nmea_sprint(a, aSize, n, ",%.8lf", pos.lla[1]);                     // 5
    nmea_sprint(a, aSize, n, ",%.2lf", pos.lla[2]);                     // 6
    
    nmea_sprint_f(a, aSize, n, ",%.2f", pos.hMSL);                      // 7
    nmea_sprint_f(a, aSize, n, ",%.2f", pos.pDop);                      // 8
    nmea_sprint_f(a, aSize, n, ",%.2f", pos.hAcc);                      // 9
    nmea_sprint_f(a, aSize, n, ",%.2f", pos.vAcc);                      // 10

    nmea_sprint_f(a, aSize, n, ",%.2f", vel.vel[0]);                    // 11
    nmea_sprint_f(a, aSize, n, ",%.2f", vel.vel[1]);                    // 12
    nmea_sprint_f(a, aSize, n, ",%.2f", vel.vel[2]);                    // 13
    nmea_sprint_f(a, aSize, n, ",%.2f", vel.sAcc);                      // 14

    nmea_sprint_f(a, aSize, n, ",%.1f", pos.cnoMean);                   // 15
    nmea_sprint(a, aSize, n, ",%.4lf", pos.towOffset);                  // 16
    nmea_sprint(a, aSize, n, ",%u", (unsigned int)pos.leapS);           // 17
    
    return nmea_sprint_footer(a, aSize, n);
}

/**
 * @brief Appends an NMEA-format latitude field pair (",ddmm.mmmmm,N" or ",ddmm.mmmmm,S") to a sentence being built.
 * @note A small epsilon is added/subtracted before truncation to prevent rounding from truncating a value like 59.99999 minutes down to 59 instead of rolling to the next degree.
 *
 * @param a - output buffer holding the sentence being built
 * @param aSize - total size of output buffer in bytes
 * @param offset - [in/out] current write offset into a; advanced by the number of characters written
 * @param v - latitude in decimal degrees (positive=North, negative=South)
 */
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

/**
 * @brief Appends an NMEA-format longitude field pair (",dddmm.mmmmm,E" or ",dddmm.mmmmm,W") to a sentence being built.
 * @note A small epsilon is added/subtracted before truncation to prevent rounding from truncating a value like 59.99999 minutes down to 59 instead of rolling to the next degree.
 *
 * @param a - output buffer holding the sentence being built
 * @param aSize - total size of output buffer in bytes
 * @param offset - [in/out] current write offset into a; advanced by the number of characters written
 * @param v - longitude in decimal degrees (positive=East, negative=West)
 */
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

/**
 * @brief Appends an NMEA-format UTC time field (",HHMMSS", second precision) computed from a GNSS position's GPS time-of-week and leap seconds.
 *
 * @param a - output buffer holding the sentence being built
 * @param aSize - total size of output buffer in bytes
 * @param offset - [in/out] current write offset into a; advanced by the number of characters written
 * @param pos - GNSS position data providing timeOfWeekMs and leapS used to compute UTC time
 */
static void nmea_GPSTimeToUTCTime(char* a, int aSize, int &offset, gnss_pos_t &pos)
{
    aSize -= offset;
    a += offset;
    utc_time_t t;
    gpsTowMsToUtcTime(pos.timeOfWeekMs, pos.leapS, &t);

    offset += ssnprintf(a, aSize, ",%02u%02u%02u", t.hour, t.minute, t.second);
}

/**
 * @brief Appends an NMEA-format UTC time field (",HHMMSS.sss", millisecond precision) computed from a GNSS position's GPS time-of-week and leap seconds.
 *
 * @param a - output buffer holding the sentence being built
 * @param aSize - total size of output buffer in bytes
 * @param offset - [in/out] current write offset into a; advanced by the number of characters written
 * @param pos - GNSS position data providing timeOfWeekMs and leapS used to compute UTC time
 */
void nmea_GPSTimeToUTCTimeMsPrecision(char* a, int aSize, int &offset, gnss_pos_t &pos)
{
    aSize -= offset;
    a += offset;
    utc_time_t t;
    gpsTowMsToUtcTime(pos.timeOfWeekMs, pos.leapS, &t);
    offset += ssnprintf(a, aSize, ",%02u%02u%02u.%03u", t.hour, t.minute, t.second, t.millisecond);
}

/**
 * @brief Appends an NMEA-format UTC date field (",ddmmyy") computed from a GNSS position's GPS week/time-of-week and leap seconds.
 *
 * @param a - output buffer holding the sentence being built
 * @param aSize - total size of output buffer in bytes
 * @param offset - [in/out] current write offset into a; advanced by the number of characters written
 * @param pos - GNSS position data providing week, timeOfWeekMs, and leapS used to compute the UTC calendar date
 */
static void nmea_GPSDateOfLastFix(char* a, int aSize, int &offset, gnss_pos_t &pos)
{
    aSize -= offset;
    a += offset;
    double julian = gpsToJulian(pos.week, pos.timeOfWeekMs, pos.leapS);
    uint32_t year, month, day, hours, minutes, seconds, milliseconds;
    julianToDate(julian, &year, &month, &day, &hours, &minutes, &seconds, &milliseconds);

    offset += ssnprintf(a, aSize, ",%02u%02u%02u", (unsigned int)day, (unsigned int)month, (unsigned int)(year-2000));
}

/**
 * @brief Appends comma-separated NMEA-format UTC date fields (",dd,mm,yyyy") computed from a GNSS position's GPS week/time-of-week and leap seconds.
 * @note Unlike nmea_GPSDateOfLastFix(), this emits day/month/year as 3 separate fields (used by $ZDA) rather than one packed "ddmmyy" field (used by $RMC).
 *
 * @param a - output buffer holding the sentence being built
 * @param aSize - total size of output buffer in bytes
 * @param offset - [in/out] current write offset into a; advanced by the number of characters written
 * @param pos - GNSS position data providing week, timeOfWeekMs, and leapS used to compute the UTC calendar date
 */
static void nmea_GPSDateOfLastFixCSV(char* a, int aSize, int &offset, gnss_pos_t &pos)    //Comma Separated Values
{
    aSize -= offset;
    a += offset;
    double julian = gpsToJulian(pos.week, pos.timeOfWeekMs, pos.leapS);
    uint32_t year, month, day, hours, minutes, seconds, milliseconds;
    julianToDate(julian, &year, &month, &day, &hours, &minutes, &seconds, &milliseconds);
    
    offset += ssnprintf(a, aSize, ",%02u,%02u,%04u", (unsigned int)day, (unsigned int)month, (unsigned int)year);
}

/**
 * @brief Builds a $GxGGA (Global Positioning System Fix Data) NMEA message from a GNSS position solution.
 * @note pos.status's fix-type sub-field (GNSS_STATUS_FIX_MASK, see eGnssStatus in data_sets.h) is remapped to the standard NMEA GGA fix-quality codes (0=invalid,...,6=dead reckoning); see field 6 below.
 *
 * @param a[] - output buffer
 * @param aSize - size of output buffer
 * @param pos - GNSS position data (gnss_pos_t) to encode
 *
 * @return length of the generated NMEA sentence, including the checksum trailer
 */
int nmea_gga(char a[], const int aSize, gnss_pos_t &pos)
{
    int fixQuality = 0;
    switch((pos.status&GNSS_STATUS_FIX_MASK))
    {
    default:
    case GNSS_STATUS_FIX_NONE:                   fixQuality = 0;    break;
    case GNSS_STATUS_FIX_SBAS:
    case GNSS_STATUS_FIX_2D:
    case GNSS_STATUS_FIX_RTK_SINGLE:
    case GNSS_STATUS_FIX_3D:                     fixQuality = 1;    break;
    case GNSS_STATUS_FIX_DGPS:                   fixQuality = 2;    break;
    case GNSS_STATUS_FIX_TIME_ONLY:              fixQuality = 3;    break;   
    case GNSS_STATUS_FIX_RTK_FIX:                fixQuality = 4;    break;
    case GNSS_STATUS_FIX_RTK_FLOAT:              fixQuality = 5;    break;
    case GNSS_STATUS_FIX_DEAD_RECKONING_ONLY:
    case GNSS_STATUS_FIX_GNSS_PLUS_DEAD_RECK:    fixQuality = 6;    break;
    }
        
    // NMEA GGA line - http://www.gpsinformation.org/dale/nmea.htm#GGA
    /*
    GGA          Global Positioning System Fix Data
    123519       Fix taken at 12:35:19 UTC
    4807.038,N   Latitude 48 deg 07.038' N
    01131.000,E  Longitude 11 deg 31.000' E
    .            Fix quality:    0 = invalid
    .                            1 = GNSS fix (SPS)
    .                            2 = DGPS fix
    .                            3 = PPS fix
    .                            4 = Real Time Kinematic
    .                            5 = Float RTK
    .                            6 = estimated (dead reckoning) (2.3 feature)
    .                            7 = Manual input mode
    .                            8 = Simulation mode
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
    nmea_sprint(a, aSize, n, ",%01u", (unsigned int)(fixQuality & 0xF));                            // 6 - GNSS quality -- limit to available options (TODO: Overkill and probably unnecessary)
    nmea_sprint(a, aSize, n, ",%02u", (unsigned int)(pos.status&GNSS_STATUS_NUM_SATS_USED_MASK));   // 7 - Satellites used
    nmea_sprint_f(a, aSize, n, ",%.2f", pos.pDop);                                                  // 8 - HDop
    nmea_sprint_f(a, aSize, n, ",%.2f,M", pos.hMSL);                                                // 9,10 - MSL altitude
    nmea_sprint_f(a, aSize, n, ",%.2f,M", float(pos.lla[2]) - pos.hMSL);                            // 11,12 - Geoid separation
    nmea_sprint(a, aSize, n, ",,");                                                                 // 13,14 - Age of differential, DGPS station ID number
    return nmea_sprint_footer(a, aSize, n);
}

/**
 * @brief Builds a $GxGLL (Geographic Position, Latitude/Longitude) NMEA message from a GNSS position solution.
 * @note Fix validity is derived from pos.status's fix-type sub-field (GNSS_STATUS_FIX_MASK, see eGnssStatus): if no fix bits are set, lat/lon fields are emitted empty; the data-valid field (6) is derived from pos.week rather than the fix status.
 *
 * @param a[] - output buffer
 * @param aSize - size of output buffer
 * @param pos - GNSS position data (gnss_pos_t) to encode
 *
 * @return length of the generated NMEA sentence, including the checksum trailer
 */
int nmea_gll(char a[], const int aSize, gnss_pos_t &pos)
{
    // NMEA GLL line - http://www.gpsinformation.org/dale/nmea.htm#GLL
    /*
         GLL          Geographic position, Latitude and Longitude
         4916.46,N    Latitude 49 deg. 16.45 min. North
         12311.12,W   Longitude 123 deg. 11.12 min. West
         225444.800   Fix taken at 22:54:44.8 UTC
         A            Data status: A (active) or V (void)
         *iD          checksum data
    */
    
    int n = nmea_talker(a, aSize);
    nmea_sprint(a, aSize, n, "GLL");

    if (pos.status&GNSS_STATUS_FIX_MASK)
    {   // Valid lat/lon
        nmea_latToDegMin(a, aSize, n, pos.lla[0]);      // 1,2
        nmea_lonToDegMin(a, aSize, n, pos.lla[1]);      // 3,4
    }
    else // Invalid lat/lon
        nmea_sprint(a, aSize, n, ",,,,");               // 1,2,3,4
        
    nmea_GPSTimeToUTCTimeMsPrecision(a, aSize, n, pos); // 5

    if (pos.week > 2269) // Time is valid so set to active
        nmea_sprint(a, aSize, n, ",A");                 // 6
    else // Time is invalid so set to void
        nmea_sprint(a, aSize, n, ",V");                 // 6

    return nmea_sprint_footer(a, aSize, n);
}

/**
 * @brief Builds a $GxGSA (GNSS DOP and Active Satellites) NMEA message listing satellites used in the solution and dilution-of-precision values.
 * @note pos.status's fix-type sub-field (GNSS_STATUS_FIX_MASK, see eGnssStatus) is remapped to the standard NMEA GSA fix-mode codes (1=no fix, 2=2D, 3=3D); see field 2 below.
 *
 * @param a[] - output buffer
 * @param aSize - size of output buffer
 * @param pos - GNSS position data (gnss_pos_t) providing fix status and DOP/accuracy values
 * @param sat - GNSS satellite list (gnss_sat_t) providing the satellite IDs used in the solution
 *
 * @return length of the generated NMEA sentence, including the checksum trailer
 */
int nmea_gsa(char a[], const int aSize, gnss_pos_t &pos, gnss_sat_t &sat)
{
    int fixQuality;
    switch((pos.status&GNSS_STATUS_FIX_MASK))
    {
        default:                        fixQuality = 0; break;
        case GNSS_STATUS_FIX_2D:         fixQuality = 2; break;
        case GNSS_STATUS_FIX_3D:         // FALL THROUGH                    
        case GNSS_STATUS_FIX_SBAS:       // FALL THROUGH
        case GNSS_STATUS_FIX_DGPS:       // FALL THROUGH                    
        case GNSS_STATUS_FIX_RTK_FIX:    // FALL THROUGH
        case GNSS_STATUS_FIX_RTK_SINGLE: // FALL THROUGH
        case GNSS_STATUS_FIX_RTK_FLOAT:  fixQuality = 3; break;
    }
        
    // NMEA GSA line - http://www.gpsinformation.org/dale/nmea.htm#GSA
    /*
        eg1. $GPGSA,A,3,,,,,,16,18,,22,24,,,3.6,2.1,2.2*3C
        eg2. $GPGSA,A,3,19,28,14,18,27,22,31,39,,,,,1.7,1.0,1.3*35

        1    = Mode:
        .        M=Manual, forced to operate in 2D or 3D
        .        A=Automatic, 3D/2D
        2    = Mode:
        .        1=Fix not available
        .        2=2D
        .        3=3D
        3-14 = IDs of SVs used in position fix (null for unused fields)
        15   = PDOP
        16   = HDOP
        17   = VDOP
    */

    int n = nmea_talker(a, aSize);
    nmea_sprint(a, aSize, n, "GSA");
    nmea_sprint(a, aSize, n, ",A,%02u",    (unsigned int)fixQuality);        // 1,2
        
    for (uint32_t i = 0; i < 12; i++)                                    // 3-14
    {
        if (sat.sat[i].svId)
        {
            nmea_sprint(a, aSize, n, ",%02u", (unsigned)(sat.sat[i].svId));
        }
        else
        {
            nmea_sprint(a, aSize, n, ",");
        }
    }
        
    nmea_sprint(a, aSize, n,
        ",%.1f"             // 15
        ",%.1f"             // 16
        ",%.1f",            // 17
        (double)pos.pDop,   // 15
        (double)pos.hAcc,   // 16
        (double)pos.vAcc);  // 17    

    return nmea_sprint_footer(a, aSize, n);
}

/**
 * @brief Pushes a new sample into a rolling history buffer and returns the median value of the history using a sorted scratch copy.
 *
 * @param newValue - new sample value to push into the front of the history
 * @param history[] - [in/out] rolling sample history buffer of size historySize, shifted by one each call (index 0 = most recent)
 * @param sorted[] - scratch buffer of size historySize used to sort a copy of history without disturbing sample order
 * @param historySize - number of elements in history[] and sorted[]
 * @param halfHistorySize - index of the median element in the sorted array (typically historySize/2)
 *
 * @return median value of the updated history
 */
float median_filter(float newValue, float history[], float sorted[], int historySize, int halfHistorySize)
{
    // Update history buffer
    for (int i = historySize-1; i > 0; i--)
    {
        history[i] = history[i-1];
    }
    history[0] = newValue;

    // Create temporary array for sorting
    for (int i = 0; i < historySize; i++)
    {
        sorted[i] = history[i];
    }

    // Simple bubble sort
    for (int i = 0; i < historySize - 1; i++)
    {
        for (int j = 0; j < historySize - i - 1; j++)
        {
            if (sorted[j] > sorted[j + 1])
            {
                float temp = sorted[j];
                sorted[j] = sorted[j + 1];
                sorted[j + 1] = temp;
            }
        }
    }

    // Return median value
    return sorted[halfHistorySize];
}

/**
 * @brief Recomputes cached NED velocity/2D-speed values (used by $RMC, $VTG, $INTEL) when a new GNSS position/velocity sample arrives.
 * @note Recomputation only occurs once per unique pos.timeOfWeekMs (cached in the static s_dataSpeed) to avoid redundant work when multiple NMEA messages are built from the same sample. Velocity is converted from ECEF to NED unless vel.status's GNSS_STATUS_FLAGS_GNSS_NMEA_DATA bit indicates it is already NED. 2D speed is optionally smoothed with a median filter when s_dataSpeed.enableSpeedFilter is set (see nmea_parse_asce()'s NMEA speed-filter option).
 *
 * @param pos - GNSS position data providing the timeOfWeekMs used to detect a new sample, and lat/lon used for ECEF->NED conversion
 * @param vel - GNSS velocity data (ECEF or NED, per vel.status GNSS_STATUS_FLAGS_GNSS_NMEA_DATA bit) to convert/filter
 */
void update_nmea_speed(gnss_pos_t &pos, gnss_vel_t &vel)
{
    if (s_dataSpeed.timeOfWeekMs != pos.timeOfWeekMs)
    {
        s_dataSpeed.timeOfWeekMs = pos.timeOfWeekMs;

        if (vel.status & GNSS_STATUS_FLAGS_GNSS_NMEA_DATA)
        {   // NED velocity
            cpy_Vec3_Vec3(s_dataSpeed.velNed, vel.vel);
        }
        else
        {   // ECEF velocity
            ixQuat qe2n;
            quat_ecef2ned(C_DEG2RAD_F*(float)pos.lla[0], C_DEG2RAD_F*(float)pos.lla[1], qe2n);
            quatConjRot(s_dataSpeed.velNed, qe2n, vel.vel);
        }

        float speed2dMps = mag_Vec2(s_dataSpeed.velNed);
        float sorted[HISTORY_SIZE];
        s_dataSpeed.speed2dMps = s_dataSpeed.enableSpeedFilter ? median_filter(speed2dMps, s_dataSpeed.speed2dMpsHistory, sorted, HISTORY_SIZE, HISTORY_SIZE/2) : speed2dMps;
        s_dataSpeed.speed2dKnots = C_METERS_KNOTS_F * s_dataSpeed.speed2dMps;
    }
}

/**
 * @brief Builds a $GxRMC (Recommended Minimum Navigation Information) NMEA message from a GNSS position/velocity solution.
 * @note Field 2 (status A/V) is derived from pos.status's fix-type sub-field (GNSS_STATUS_FIX_MASK, see eGnssStatus): any fix other than GNSS_STATUS_FIX_NONE reports "A" (active).
 *
 * @param a[] - output buffer
 * @param aSize - size of output buffer
 * @param pos - GNSS position data (gnss_pos_t) to encode
 * @param vel - GNSS velocity data (gnss_vel_t), used (via update_nmea_speed) to derive speed/course made good
 * @param magDeclination - magnetic declination (radians), used to compute the magnetic variation field
 *
 * @note output message format:
 *  0    Message ID $GxRMC
 *  1    UTC time of fix (HHMMSS)
 *  2    Status: A=active/valid fix, V=void (derived from pos.status GNSS_STATUS_FIX_MASK)
 *  3,4  Latitude (ddmm.mmmmm, N/S)
 *  5,6  Longitude (dddmm.mmmmm, E/W)
 *  7    Speed over ground (knots)
 *  8    Course made good (degrees true)
 *  9    Date of fix (ddmmyy)
 *  10   Magnetic variation (degrees, unsigned)
 *  11   Magnetic variation direction: E=easterly (subtracts from true course), W=westerly
 *  12   Checksum, begins with *
 *
 * @return length of the generated NMEA sentence, including the checksum trailer
 */
int nmea_rmc(char a[], const int aSize, gnss_pos_t &pos, gnss_vel_t &vel, float magDeclination)
{
    update_nmea_speed(pos, vel);

    int n = nmea_talker(a, aSize);
    nmea_sprint(a, aSize, n, "RMC");
    nmea_GPSTimeToUTCTime(a, aSize, n, pos);    // 1 - UTC time of last fix
    if ((pos.status&GNSS_STATUS_FIX_MASK)!=GNSS_STATUS_FIX_NONE)
    {
        nmea_sprint(a, aSize, n, ",A");         // 2 - A=active (good)
    }
    else
    {
        nmea_sprint(a, aSize, n, ",V");         // 2 - V=void (bad,warning)
    }
    nmea_latToDegMin(a, aSize, n, pos.lla[0]);  // 3,4 - lat (degrees minutes)
    nmea_lonToDegMin(a, aSize, n, pos.lla[1]);  // 5,6 - lon (degrees minutes)
    
    float courseMadeTrue = atan2f(s_dataSpeed.velNed[1], s_dataSpeed.velNed[0]);
    nmea_sprint(a, aSize, n,
    ",%05.1f"                                   // 7
    ",%05.1f",                                  // 8
    double(s_dataSpeed.speed2dKnots),                   // 7 - speed in knots
    double(courseMadeTrue*C_RAD2DEG_F));                // 8 - course made true
    
    nmea_GPSDateOfLastFix(a, aSize, n, pos);    // 9 - date of last fix UTC
    
    // Magnetic variation degrees (Easterly var. subtracts from true course), i.e. 020.3,E - left pad to 3 zero
    float magDec = magDeclination * C_RAD2DEG_F;
    bool positive = (magDec >= 0.0f);
    
    nmea_sprint(a, aSize, n,
    ",%05.1f"                                   // 10
    ",%s",                                      // 11
    double(fabsf(magDec)),                      // 10 - Magnetic variation
    (positive ? "E" : "W"));                    // 11
    
    return nmea_sprint_footer(a, aSize, n);
}

/**
 * @brief Builds a $GxZDA (Time and Date) NMEA message from a GNSS position solution's GPS time.
 *
 * @param a[] - output buffer
 * @param aSize - size of output buffer
 * @param pos - GNSS position data (gnss_pos_t) providing the GPS time-of-week/week/leap-seconds used to compute UTC time and date
 *
 * @return length of the generated NMEA sentence, including the checksum trailer
 */
int nmea_zda(char a[], const int aSize, gnss_pos_t &pos)
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
    nmea_GPSTimeToUTCTimeMsPrecision(a, aSize, n, pos);                                // 1
    nmea_GPSDateOfLastFixCSV(a, aSize, n, pos);                                        // 2,3,4
    nmea_sprint(a, aSize, n, ",00,00");                                                // 5,6
    
    return nmea_sprint_footer(a, aSize, n);
}

/**
 * @brief Builds a $GxVTG (Course Over Ground and Ground Speed) NMEA message from a GNSS position/velocity solution.
 * @note Field 9's mode indicator is derived from pos.status's fix-type sub-field (GNSS_STATUS_FIX_MASK, see eGnssStatus).
 *
 * @param a[] - output buffer
 * @param aSize - size of output buffer
 * @param pos - GNSS position data (gnss_pos_t), used for the fix-type mode indicator
 * @param vel - GNSS velocity data (gnss_vel_t), used (via update_nmea_speed) to derive speed/course made good
 * @param magVarCorrectionRad - magnetic variation correction (radians); if 0.0, the magnetic-track field is left empty
 *
 * @return length of the generated NMEA sentence, including the checksum trailer
 */
int nmea_vtg(char a[], const int aSize, gnss_pos_t &pos, gnss_vel_t &vel, float magVarCorrectionRad)
{
    /*
        0    Message ID $GPVTG
        1    Track made good (degrees true)
        2    T: track made good is relative to true north
        3    Track made good (degrees magnetic)
        4    M: track made good is relative to magnetic north 
        5    Speed, in knots
        6    N: speed is measured in knots
        7    Speed over ground in kilometers/hour (kph)
        8    K: speed over ground is measured in kph
        9    Mode indicator:
            A: Autonomous mode
            D: Differential mode
            E: Estimated (dead reckoning) mode
            M: Manual Input mode
            S: Simulator mode
            N: Data not valid
        10    The checksum data, always begins with *

        Example: $GPVTG,140.88,T,,M,8.04,N,14.89,K,D*05
    */
    update_nmea_speed(pos, vel);

    int n = nmea_talker(a, aSize);
    nmea_sprint(a, aSize, n, "VTG");
    float courseMadeTrue = atan2f(s_dataSpeed.velNed[1], s_dataSpeed.velNed[0]);
    nmea_sprint_f(a, aSize, n, ",%.2f", C_RAD2DEG_F * courseMadeTrue);          // 1
    nmea_sprint(a, aSize, n, ",T");                                             // 2
    if (magVarCorrectionRad == 0.0f)                                            // 3
    {
        nmea_sprint(a, aSize, n, ",");
    }
    else
    {
        nmea_sprint_f(a, aSize, n, ",%.2f", courseMadeTrue + magVarCorrectionRad*C_RAD2DEG_F);
    }
    nmea_sprint(a, aSize, n, ",M");                                             // 4
    nmea_sprint_f(a, aSize, n, ",%.2f", s_dataSpeed.speed2dKnots);              // 5
    nmea_sprint(a, aSize, n, ",N");                                             // 6
    nmea_sprint_f(a, aSize, n, ",%.2f", s_dataSpeed.speed2dMps*C_MPS2KMPH_F);   // 7
    nmea_sprint(a, aSize, n, ",K");                                             // 8
    switch(pos.status & GNSS_STATUS_FIX_MASK)                                    // 9
    {
    case GNSS_STATUS_FIX_2D:
    case GNSS_STATUS_FIX_3D:
        nmea_sprint(a, aSize, n, ",A");
        break;
    case GNSS_STATUS_FIX_GNSS_PLUS_DEAD_RECK:
    case GNSS_STATUS_FIX_DEAD_RECKONING_ONLY:
        nmea_sprint(a, aSize, n, ",E");
        break;
    case GNSS_STATUS_FIX_DGPS:
    case GNSS_STATUS_FIX_RTK_SINGLE:
    case GNSS_STATUS_FIX_RTK_FLOAT:
    case GNSS_STATUS_FIX_RTK_FIX:
        nmea_sprint(a, aSize, n, ",D");
        break;
    default:
        nmea_sprint(a, aSize, n, ",N");
        break;
    }
    return nmea_sprint_footer(a, aSize, n);
}

/**
 * @brief Builds a $PASHR (RT300-compatible proprietary) NMEA message reporting heading, roll, pitch, heave, and their accuracy estimates.
 * @note Fields 11/12 (GNSS quality / INS status) are derived from ins1.insStatus's GNSS nav-fix sub-field (INS_STATUS_GNSS_NAV_FIX_MASK) and solution sub-field (INS_STATUS_SOLUTION_MASK), see eInsStatusFlags in data_sets.h.
 *
 * @param a[] - output buffer
 * @param aSize - size of output buffer
 * @param pos - GNSS position data (gnss_pos_t), used for the UTC time field
 * @param ins1 - INS solution (ins_1_t), providing heading/roll/pitch and status used to derive fields 11/12
 * @param heave - heave value (m)
 * @param sigma - INS NED attitude standard deviations (inl2_ned_sigma_t), used for the roll/pitch/heading accuracy fields
 *
 * @note output message format:
 *  0   Message ID $PASHR
 *  1   UTC time (HHMMSS.sss)
 *  2   Heading (degrees)
 *  3   "T" - flag indicating heading is relative to True North
 *  4   Roll angle (degrees, signed)
 *  5   Pitch angle (degrees, signed)
 *  6   Heave (m, signed)
 *  7   Roll angle accuracy estimate / std dev (degrees)
 *  8   Pitch angle accuracy estimate / std dev (degrees)
 *  9   Heading angle accuracy estimate / std dev (degrees)
 *  10  GNSS quality/aiding status: 0=no aiding, 1=3D GNSS position aiding, 2=RTK float/fix aiding (derived from INS_STATUS_GNSS_NAV_FIX_MASK)
 *  11  INS navigation status: 1=in navigation mode with good solution (INS_STATUS_SOLUTION_MASK == INS_STATUS_SOLUTION_NAV or better), 0=otherwise
 *  12  Checksum, begins with *
 *
 * @return length of the generated NMEA sentence, including the checksum trailer
 */
int nmea_pashr(char a[], const int aSize, gnss_pos_t &pos, ins_1_t &ins1, float heave, inl2_ned_sigma_t &sigma)
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
    
    int n = ssnprintf(a, aSize, "$PASHR");                                  // 1 - Name
    nmea_GPSTimeToUTCTimeMsPrecision(a, aSize, n, pos);                     // 2 - UTC Time

    nmea_sprint_f(a, aSize, n, ",%.2f", C_RAD2DEG_F * ins1.theta[2]);       // 3 - Heading value in decimal degrees.
    nmea_sprint(a, aSize, n, ",T");                                         // 4 - T (heading respect to True North)
    nmea_sprint_f(a, aSize, n, ",%+.2f", C_RAD2DEG_F * ins1.theta[0]);      // 5 - Roll in degrees
    nmea_sprint_f(a, aSize, n, ",%+.2f", C_RAD2DEG_F * ins1.theta[1]);      // 6 - Pitch in degrees
    nmea_sprint_f(a, aSize, n, ",%+.2f", heave);                            // 7 - Heave

    nmea_sprint_f(a, aSize, n, ",%.3f", C_RAD2DEG_F * sigma.StdAttNed[0]);  // 8 - roll accuracy
    nmea_sprint_f(a, aSize, n, ",%.3f", C_RAD2DEG_F * sigma.StdAttNed[1]);  // 9 - pitch accuracy
    nmea_sprint_f(a, aSize, n, ",%.3f", C_RAD2DEG_F * sigma.StdAttNed[2]);  // 10 - heading accuracy

    int fix = 0;
    if (INS_STATUS_NAV_FIX_STATUS(ins1.insStatus) >= GNSS_NAV_FIX_POSITIONING_RTK_FLOAT)
    {
        fix = 2;
    }
    else if (INS_STATUS_NAV_FIX_STATUS(ins1.insStatus) >= GNSS_NAV_FIX_POSITIONING_3D)
    {
        fix = 1;
    }
    nmea_sprint(a, aSize, n, ",%d", fix);                                                                // 11 - GNSS Quality
    nmea_sprint(a, aSize, n, ",%d", INS_STATUS_SOLUTION(ins1.insStatus) >= INS_STATUS_SOLUTION_NAV);     // 12 - INS Status
    
    return nmea_sprint_footer(a, aSize, n);
}

/**
 * @brief Builds a $INTEL proprietary NMEA message reporting KIM sub-module firmware version, GPS time, and velocity (ECEF and NED).
 * @note Fields 6-8 (PPS phase/quantization error) are currently always emitted as 0 - not yet implemented/wired up.
 *
 * @param a[] - output buffer
 * @param aSize - size of output buffer
 * @param info - device info (dev_info_t), providing the KIM firmware version
 * @param pos - GNSS position data (gnss_pos_t), providing GPS time of week/week/leap seconds
 * @param vel - GNSS velocity data (gnss_vel_t, ECEF), also used (via update_nmea_speed) to derive the cached NED velocity
 *
 * @return length of the generated NMEA sentence, including the checksum trailer
 */
int nmea_intel(char a[], const int aSize, dev_info_t &info, gnss_pos_t &pos, gnss_vel_t &vel)
{
    /*  $INTEL prorietary NMEA message
        0    Message ID $INTEL
        1    Message ID KIM
        2    Fimrware version of KIM
        3    GPS Time of Week (seconds, no decimal)
        4    GPS week number
        5    GPS leap seconds
        6    1PPS phase 1 (ns)
        7    1PPS phase 2 (ns)
        8    Quantization error of time pulse (ns)
        9    ECEF X velocity (m/s)
        10    ECEF Y velocity (m/s)
        11    ECEF Z velocity (m/s)
        12    North veocity (m/s)
        13    East velocity (m/s)
        14    Down velocity (m/s)
        15    Checksum, begins with *

        Example: $INTEL, *05
    */
    update_nmea_speed(pos, vel);

    int n = ssnprintf(a, aSize, "$INTEL,KIM");                      // 0,1

    nmea_sprint(a, aSize, n, ",%d.%d.%d.%d", 
        info.firmwareVer[0], 
        info.firmwareVer[1], 
        info.firmwareVer[2], 
        info.firmwareVer[3]);                                       // 2
    nmea_sprint(a, aSize, n, ",%d", pos.timeOfWeekMs/1000);         // 3
    nmea_sprint(a, aSize, n, ",%d", pos.week);                      // 4
    nmea_sprint(a, aSize, n, ",%d", pos.leapS);                     // 5

    nmea_sprint(a, aSize, n, ",%.3f", 0.0);                         // 6
    nmea_sprint(a, aSize, n, ",%.3f", 0.0);                         // 7
    nmea_sprint(a, aSize, n, ",0");                                 // 8

    nmea_sprint_f(a, aSize, n, ",%.3f", vel.vel[0]);                // 9
    nmea_sprint_f(a, aSize, n, ",%.3f", vel.vel[1]);                // 10
    nmea_sprint_f(a, aSize, n, ",%.3f", vel.vel[2]);                // 11

    nmea_sprint_f(a, aSize, n, ",%.3f", s_dataSpeed.velNed[0]);     // 12
    nmea_sprint_f(a, aSize, n, ",%.3f", s_dataSpeed.velNed[1]);     // 13
    nmea_sprint_f(a, aSize, n, ",%.3f", s_dataSpeed.velNed[2]);     // 14

    return nmea_sprint_footer(a, aSize, n);
}

/**
 * @brief Preps fields 1-6 of $POWxxx prorietary NMEA message
 * 
 * @param a[] - output buffer
 * @param startN - starting index in output buffer
 * @param aSize - size of output buffer
 * @param pos - gnss position data
 * 
 * @note output message format: 
 *  1   GPS Time Quality (0=invalid, 1=valid)
 *  2   GPS Week Number
 *  3   GPS Time of Week (micro seconds)
 *  4   GPS leap seconds validity (0=invalid, 1=valid)
 *  5   GPS leap seconds
 *  6   Holdover flag (0=no holdover, 1=EGR is in holdover)
 */
int nmea_powPrep(char a[], int startN, const int aSize, gnss_pos_t &pos)
{  
    int n = startN;
    int valid = (pos.week > 2359) ? 1 : 0; // assume time is valid if week > 2359 (03/23/2025)

    nmea_sprint(a, aSize, n, ",%d", valid);                 // 1
    nmea_sprint(a, aSize, n, ",%d", pos.week);              // 2
    nmea_sprint(a, aSize, n, ",%" PRIu64, ((uint64_t)pos.timeOfWeekMs)*1000); // 3

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
int nmea_powgps(char a[], const int aSize, gnss_pos_t &pos)
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
 * @param pos - gnss position data
 * @param vel - gnss velocity data
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
int nmea_powtlv(char a[], const int aSize, gnss_pos_t &pos, gnss_vel_t &vel)
{    
    float horVel = MAG_VEC2(vel.vel);
    float groundTrackHeading = 0;

    int n = ssnprintf(a, aSize, "$POWTLV");                     // 0
    
    update_nmea_speed(pos, vel);

    n = nmea_powPrep(a, n, aSize, pos);                         // 1-6

    nmea_latToDegMin(a, aSize, n, pos.lla[0]);                  // 7,8
    nmea_lonToDegMin(a, aSize, n, pos.lla[1]);                  // 9,10

    nmea_sprint(a, aSize, n, ",%.3f", pos.lla[2]);              // 11
    nmea_sprint_f(a, aSize, n, ",%.3f", pos.hMSL);              // 12

    nmea_sprint_f(a, aSize, n, ",%.3f", horVel);                // 13

    nmea_sprint_f(a, aSize, n, ",%.3f", vel.vel[2]);            // 14
    
    groundTrackHeading = C_RAD2DEG_F * atan2f(vel.vel[1], vel.vel[0]);

    if (groundTrackHeading < 0.0f)  groundTrackHeading += 360.0f;
    
    nmea_sprint_f(a, aSize, n, ",%.3f", groundTrackHeading);    // 15

    return nmea_sprint_footer(a, aSize, n);                     // 16
}

/**
 * @brief Null-terminates a buffer at the given length and prints it to stdout. Debug helper.
 *
 * @param a[] - buffer to print
 * @param n - index at which to write the null terminator (i.e. the string length)
 */
void print_string_n(char a[], int n)
{
    a[n] = '\0';
    printf("%s", a);
}

/**
 * @brief Converts an internal PRN number to the NMEA "svId" numbering used in $GxGSV messages, applying the SBAS PRN offset.
 * @note SBAS PRNs are offset by -87 for NMEA reporting (mirrors the +87 offset applied when parsing $GxGSV in talkerId_to_gnssId()).
 *
 * @param gnssId - GNSS constellation ID (see eSatSvGnssId)
 * @param prn - internal PRN/satellite number
 *
 * @return NMEA svId for the given constellation/PRN
 */
int prnToSvId(int gnssId, int prn)
{
    switch (gnssId)
    {
    case SAT_SV_GNSS_ID_SBS: return prn-87;
    }

    return prn;
}

/**
 * @brief Tests whether a tracked satellite signal matches a target GNSS constellation and signal ID for $GxGSV grouping, and matches the requested C/N0-presence filter.
 * @note GPS also matches SBAS-tagged signals (SBAS satellites are reported under the GPS talker per the NMEA spec). sigId==0xFF acts as a wildcard matching any signal ID.
 *
 * @param gnssId - target GNSS constellation ID (see eSatSvGnssId)
 * @param sigId - target signal ID (see eSatSvSigId), or 0xFF to match any signal
 * @param s - candidate satellite signal record to test
 * @param noCno - if true, only match signals with cno==0 (no C/N0 reported); if false, only match signals with a non-zero cno
 *
 * @return true if the signal matches the requested constellation/signal/C-N0 criteria
 */
bool gsv_sig_match(uint8_t gnssId, uint8_t sigId, gnss_sig_sv_t &s, bool noCno=false)
{
    if ((s.cno==0) != noCno)
    {   // cno doesn't matches
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

/**
 * @brief Counts the number of tracked satellite signals matching a given GNSS constellation/signal ID, used to determine how many $GxGSV messages are needed (4 satellites per message).
 *
 * @param gnssId - target GNSS constellation ID (see eSatSvGnssId)
 * @param sigId - target signal ID (see eSatSvSigId), or 0xFF to match any signal
 * @param sig - full tracked-signal list (gnss_sig_t) to search
 * @param noCno - if true, count only signals with no C/N0 reported; if false (default), count only signals with a non-zero C/N0
 *
 * @return number of matching, frequency-enabled signals
 */
int nmea_gsv_num_sat_sigs(uint8_t gnssId, uint8_t sigId, gnss_sig_t &sig, bool noCno=false)
{
    int numSigs = 0;

    for (uint32_t i=0; i<sig.numSigs; i++)
    {
        gnss_sig_sv_t &s = sig.sig[i];
        if (gsv_freq_ena(&sig.sig[i]) && gsv_sig_match(gnssId, sigId, s, noCno))
        {
            numSigs++;
        }
    }

    return numSigs;
}

/**
 * @brief Converts an internal signal ID to the single-character "Signal ID" field used in the NMEA 4.11 $GxGSV message trailer.
 * @note Mapping is constellation-specific (see eSatSvSigId for the internal enum); returns '0' for GNSS/signal combinations with no defined NMEA 4.11 signal-ID code.
 *
 * @param gnssId - GNSS constellation ID (see eSatSvGnssId)
 * @param sigId - internal signal ID (see eSatSvSigId)
 *
 * @return NMEA 4.11 signal ID character (e.g. '1','5','6','7','8','B', etc.), or '0' if unmapped
 */
uint8_t sigId_to_nmea4p11_signalId(uint8_t gnssId, uint8_t sigId)
{
    switch(gnssId)
    {
        case SAT_SV_GNSS_ID_GPS:
            switch(sigId)
            {
                case SAT_SV_SIG_ID_GPS_L1CA:        return '1';
                case SAT_SV_SIG_ID_GPS_L2CL:        return '6';
                case SAT_SV_SIG_ID_GPS_L2CM:        return '5';
                case SAT_SV_SIG_ID_GPS_L5I:         return '7';
                case SAT_SV_SIG_ID_GPS_L5Q:         return '8';
            }
            break;
        case SAT_SV_GNSS_ID_SBS:
            return 1;
        case SAT_SV_GNSS_ID_GAL:
            switch(sigId)
            {
                case SAT_SV_SIG_ID_Galileo_E1C2:    // FALL THROUGH
                case SAT_SV_SIG_ID_Galileo_E1B2:    return '7';
                case SAT_SV_SIG_ID_Galileo_E5aI:    // FALL THROUGH
                case SAT_SV_SIG_ID_Galileo_E5aQ:    return '1';
                case SAT_SV_SIG_ID_Galileo_E5bI:    // FALL THROUGH
                case SAT_SV_SIG_ID_Galileo_E5bQ:    return '2';
            }
            break;
        case SAT_SV_GNSS_ID_BEI:
            switch(sigId)
            {
                case SAT_SV_SIG_ID_BeiDou_B1D1:     // FALL THROUGH
                case SAT_SV_SIG_ID_BeiDou_B1D2:     return '1';
                case SAT_SV_SIG_ID_BeiDou_B2D1:     // FALL THROUGH
                case SAT_SV_SIG_ID_BeiDou_B2D2:     return 'B';
                case SAT_SV_SIG_ID_BeiDou_B1C:      return '3';
                case SAT_SV_SIG_ID_BeiDou_B2a:      return '5';
            }
            break;
        case SAT_SV_GNSS_ID_QZS:
            switch(sigId)
            {
                case SAT_SV_SIG_ID_QZSS_L1CA:       return '1';
                case SAT_SV_SIG_ID_QZSS_L1S:        return '4';
                case SAT_SV_SIG_ID_QZSS_L2CM:       return '5';
                case SAT_SV_SIG_ID_QZSS_L2CL:       return '6';
                case SAT_SV_SIG_ID_QZSS_L5I:        return '7';
                case SAT_SV_SIG_ID_QZSS_L5Q:        return '8';
            }
            break;
        case SAT_SV_GNSS_ID_GLO:
            switch(sigId)
            {
                case SAT_SV_SIG_ID_GLONASS_L1OF:    return '1';
                case SAT_SV_SIG_ID_GLONASS_L2OF:    return '3';
            }
            break;
        case SAT_SV_GNSS_ID_IRN: // NavIC
            switch(sigId)
            {
                case SAT_SV_SIG_ID_NAVIC_L5A:        return '7';
            }
            break;
    }

    return '0';
}

/**
 * @brief Converts an NMEA 4.11 $GxGSV "Signal ID" character back to an internal signal ID for a given GNSS constellation.
 * @note Inverse of sigId_to_nmea4p11_signalId(); several internal signal IDs alias to the same NMEA character (see commented-out cases), so only one representative internal ID is returned per character.
 *
 * @param gnssId - GNSS constellation ID (see eSatSvGnssId)
 * @param nmeaSignalId - NMEA 4.11 signal ID character from the incoming sentence
 *
 * @return internal signal ID (see eSatSvSigId), or '0'/0 if unmapped
 */
uint8_t nmea4p11_signalId_to_sigId(uint8_t gnssId, char nmeaSignalId)
{
    switch(gnssId)
    {
    case SAT_SV_GNSS_ID_GPS:
        switch(nmeaSignalId)
        {
            case '1':   return SAT_SV_SIG_ID_GPS_L1CA;
            case '6':   return SAT_SV_SIG_ID_GPS_L2CL;
            case '5':   return SAT_SV_SIG_ID_GPS_L2CM;
            case '7':   return SAT_SV_SIG_ID_GPS_L5I;
            case '8':   return SAT_SV_SIG_ID_GPS_L5Q;
        }
        break;
    case SAT_SV_GNSS_ID_SBS:
        return 0;
    case SAT_SV_GNSS_ID_GAL:
        switch(nmeaSignalId)
        {
            case '7':   return SAT_SV_SIG_ID_Galileo_E1C2;
            // case '7':    return SAT_SV_SIG_ID_Galileo_E1B2;
            case '1':   return SAT_SV_SIG_ID_Galileo_E5aI;
            // case '1':    return SAT_SV_SIG_ID_Galileo_E5aQ;
            // case '2':    return SAT_SV_SIG_ID_Galileo_E5bI;
            case '2':   return SAT_SV_SIG_ID_Galileo_E5bQ;
        }
        break;
    case SAT_SV_GNSS_ID_BEI:
        switch(nmeaSignalId)
        {
            case '1':   return SAT_SV_SIG_ID_BeiDou_B1D1;
            // case '1':     return SAT_SV_SIG_ID_BeiDou_B1D2;
            case 'B':   return SAT_SV_SIG_ID_BeiDou_B2D1;
            // case 'B':    return SAT_SV_SIG_ID_BeiDou_B2D2;
            case '3':   return SAT_SV_SIG_ID_BeiDou_B1C;
            case '5':   return SAT_SV_SIG_ID_BeiDou_B2a;
        }
        break;
    case SAT_SV_GNSS_ID_QZS:
        switch(nmeaSignalId)
        {
            case '1':   return SAT_SV_SIG_ID_QZSS_L1CA;
            case '4':   return SAT_SV_SIG_ID_QZSS_L1S;
            case '5':   return SAT_SV_SIG_ID_QZSS_L2CM;
            case '6':   return SAT_SV_SIG_ID_QZSS_L2CL;
            case '7':   return SAT_SV_SIG_ID_QZSS_L5I;
            case '8':   return SAT_SV_SIG_ID_QZSS_L5Q;
        }
        break;
    case SAT_SV_GNSS_ID_GLO:
        switch(nmeaSignalId)
        {
            case '1':   return SAT_SV_SIG_ID_GLONASS_L1OF;
            case '3':   return SAT_SV_SIG_ID_GLONASS_L2OF;
        }
        break;
    case SAT_SV_GNSS_ID_IRN:    // NavIC
        switch(nmeaSignalId)
        {
            case '7':   return SAT_SV_SIG_ID_NAVIC_L5A;
        }
        break;
    }

    return '0';
}

/**
 * @brief Encodes a signal ID into the svId offset scheme used by NMEA protocol versions older than 4.10 (which lack a dedicated Signal ID field).
 * @note Since older NMEA protocols cannot report a separate signal/frequency band, the band is instead encoded by offsetting svId: +0 for the primary band (e.g. L1/E1), +256 for a secondary band (e.g. L2/E2), +512 for a third band (e.g. L5/E5). See talkerId_to_gnssId()/nmea2p3_svid_to_sigId() for the corresponding decode.
 *
 * @param gnssId - GNSS constellation ID (see eSatSvGnssId)
 * @param sigId - internal signal ID (see eSatSvSigId) whose frequency band determines the offset
 * @param svId - plain (unoffset) satellite ID
 *
 * @return svId offset-encoded to indicate the signal's frequency band, or 0 if the gnssId/sigId combination is unmapped
 */
uint16_t sigId_to_nmea2p3_svId(uint8_t gnssId, uint8_t sigId, uint16_t svId)
{
    switch(gnssId)
    {
    case SAT_SV_GNSS_ID_GPS:
        switch(sigId)
        {
            case SAT_SV_SIG_ID_GPS_L1CA:    return svId;
            case SAT_SV_SIG_ID_GPS_L2CL:    // FALL THROUGH        
            case SAT_SV_SIG_ID_GPS_L2CM:    return svId + 256;
            case SAT_SV_SIG_ID_GPS_L5I:     // FALL THROUGH            
            case SAT_SV_SIG_ID_GPS_L5Q:     return svId + 512;
        }
        break;
    case SAT_SV_GNSS_ID_SBS:
        return 1;
    case SAT_SV_GNSS_ID_GAL:
        switch(sigId)
        {
            case SAT_SV_SIG_ID_Galileo_E1C2:    // FALL THROUGH
            case SAT_SV_SIG_ID_Galileo_E1B2:    return svId;
            case SAT_SV_SIG_ID_Galileo_E5aI:    // FALL THROUGH
            case SAT_SV_SIG_ID_Galileo_E5aQ:    return svId + 256;
            case SAT_SV_SIG_ID_Galileo_E5bI:    // FALL THROUGH
            case SAT_SV_SIG_ID_Galileo_E5bQ:    return svId + 512;
        }
        break;
    case SAT_SV_GNSS_ID_BEI:
        switch(sigId)
        {
            case SAT_SV_SIG_ID_BeiDou_B1D1:    // FALL THROUGH
            case SAT_SV_SIG_ID_BeiDou_B1D2:     return svId;
            case SAT_SV_SIG_ID_BeiDou_B2D1:    // FALL THROUGH
            case SAT_SV_SIG_ID_BeiDou_B2D2:     return svId + 256;
            case SAT_SV_SIG_ID_BeiDou_B1C:      return svId;
            case SAT_SV_SIG_ID_BeiDou_B2a:      return svId + 512;
        }
        break;
    case SAT_SV_GNSS_ID_QZS:
        switch(sigId)
        {
            case SAT_SV_SIG_ID_QZSS_L1CA:   // FALL THROUGH
            case SAT_SV_SIG_ID_QZSS_L1S:    return svId;
            case SAT_SV_SIG_ID_QZSS_L2CM:   // FALL THROUGH
            case SAT_SV_SIG_ID_QZSS_L2CL:   return svId + 256;
            case SAT_SV_SIG_ID_QZSS_L5I:    // FALL THROUGH
            case SAT_SV_SIG_ID_QZSS_L5Q:    return svId + 512;
        }
        break;
    case SAT_SV_GNSS_ID_GLO:
        switch(sigId)
        {
            case SAT_SV_SIG_ID_GLONASS_L1OF:    return svId;
            case SAT_SV_SIG_ID_GLONASS_L2OF:    return svId + 256;
        }
        break;
    case SAT_SV_GNSS_ID_IRN:    // NavIC
        switch(sigId)
        {
            case SAT_SV_SIG_ID_NAVIC_L5A:   return svId + 512;
        }
        break;
    }

    return 0;
}

/**
 * @brief Decodes the pre-NMEA-4.10 svId frequency-band offset (see sigId_to_nmea2p3_svId()) back into an internal signal ID.
 * @note svId < 256 implies the primary band (L1/E1, sigId=0); svId in [256,512) implies the secondary band (e.g. L2/E2); svId >= 512 implies the third band (e.g. L5/E5), selected per-constellation.
 *
 * @param gnssId - GNSS constellation ID (see eSatSvGnssId)
 * @param svId - offset-encoded satellite ID as received in a pre-4.10 $GxGSV message
 *
 * @return internal signal ID (see eSatSvSigId) corresponding to the encoded frequency band, or 0 if unmapped
 */
uint8_t nmea2p3_svid_to_sigId(uint8_t gnssId, uint16_t svId)
{
    if (svId<256)
    {   // L1/E1
        return 0;
    }

    if (svId>=512)
    {   // L5/E5 - most common band
        switch(gnssId)
        {
            case SAT_SV_GNSS_ID_GPS:    return SAT_SV_SIG_ID_GPS_L5Q;
            case SAT_SV_GNSS_ID_SBS:    return 0;
            case SAT_SV_GNSS_ID_GAL:    return SAT_SV_SIG_ID_Galileo_E5;
            case SAT_SV_GNSS_ID_BEI:    return SAT_SV_SIG_ID_BeiDou_B2a;
            case SAT_SV_GNSS_ID_QZS:    return SAT_SV_SIG_ID_QZSS_L5;
            case SAT_SV_GNSS_ID_GLO:    return SAT_SV_SIG_ID_GLONASS_L2OF;
            case SAT_SV_GNSS_ID_IRN:    return SAT_SV_SIG_ID_NAVIC_L5A;  // NavIC
        }
    }
    else
    {   // L2/E2 - most common band
        switch(gnssId)
        {
            case SAT_SV_GNSS_ID_GPS:    return SAT_SV_SIG_ID_GPS_L2CL;
            case SAT_SV_GNSS_ID_SBS:    return 0;
            case SAT_SV_GNSS_ID_GAL:    return SAT_SV_SIG_ID_Galileo_E5a;
            case SAT_SV_GNSS_ID_BEI:    return SAT_SV_SIG_ID_BeiDou_B2;
            case SAT_SV_GNSS_ID_QZS:    return SAT_SV_SIG_ID_QZSS_L2;
            case SAT_SV_GNSS_ID_GLO:    return SAT_SV_SIG_ID_GLONASS_L2OF;
            case SAT_SV_GNSS_ID_IRN:    return SAT_SV_SIG_ID_NAVIC_L5A;  // NavIC
        }
    }

    return 0;
}

/**
 * @brief Gets the GSV frequency-band filter mask for a given constellation ID.
 *
 * @param constellation - GNSS constellation ID (see eSatSvGnssId)
 *
 * @return filter mask (bits: NMEA_GNGSV_FREQ_BAND1_BIT/BAND2_BIT/FREQ_5_BIT) for the requested constellation, or 0 if an invalid constellation is passed
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
 * @brief Checks whether the frequency band associated with a given tracked signal is enabled in the GSV filter mask (s_gsvMask).
 *
 * @param sig - tracked satellite signal (gnssId + sigId) to test
 *
 * @return true if sig->gnssId is valid and the constellation's filter mask has the bit set corresponding to sig->sigId's frequency band; false otherwise
*/
bool gsv_freq_ena(gnss_sig_sv_t* sig)
{
    if (sig->gnssId >= SAT_SV_GNSS_ID_COUNT)
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

/**
 * @brief Builds one or more $GxGSV (Satellites in View) NMEA messages for a single GNSS constellation + signal ID group, 4 satellites per message.
 *
 * @param a[] - output buffer for all generated $GxGSV messages (concatenated)
 * @param aSize - size of output buffer
 * @param gsat - GNSS satellite list (gnss_sat_t) providing elevation/azimuth for matched satellites
 * @param gsig - GNSS tracked-signal list (gnss_sig_t) providing svId/C-N0 for matched signals
 * @param gnssId - GNSS constellation ID to filter on (see eSatSvGnssId)
 * @param sigId - signal ID to filter on (see eSatSvSigId), or 0xFF (default) to match any signal
 * @param noCno - if true, only include signals with no C/N0; if false (default) only include signals with a non-zero C/N0
 *
 * @note output message format, repeated once per generated $GxGSV message:
 *  0        Message ID $GxGSV
 *  1        Total number of GSV messages in this group
 *  2        This message's sequence number within the group
 *  3        Total number of satellite signals in view (across the whole group)
 *  4+4n     Satellite ID (svId; offset-encoded by frequency band for protocol < 4.10, see sigId_to_nmea2p3_svId())
 *  5+4n     Elevation (degrees, 0-90)
 *  6+4n     Azimuth (degrees, 0-359)
 *  7+4n     C/N0, carrier-to-noise ratio (dB-Hz), empty if not tracking
 *           (fields 4-7 repeat for up to 4 satellites per message, n=0..3)
 *  last-1   Signal ID character (NMEA protocol >= 4.10 only; see sigId_to_nmea4p11_signalId())
 *  last     Checksum, begins with *
 *
 * @return total number of characters written to a[] across all generated messages
 */
int nmea_gsv_group(char a[], int aSize, gnss_sat_t &gsat, gnss_sig_t &gsig, uint8_t gnssId, uint8_t sigId=0xFF, bool noCno=false)
{
    char *bufStart = a;

    int numSigs = nmea_gsv_num_sat_sigs(gnssId, sigId, gsig);
    int numMsgs = (numSigs+3) >> 2;    // divide by 4

    uint32_t i=0;
    for (int sigNum = 0; sigNum<numSigs; sigNum+=4)
    {
        int msgNum = (sigNum >> 2) + 1;    // (divide by 4) + 1

        // Message fields: $xxGSV,numMsg,msgNum,numSats,{svid,elv,az,cno,}signalId*cs\r\n
        // Write message header: $xxGSV,numMsg,msgNum,numSats
        int n = nmea_talker(a, aSize, gnssId);
        nmea_sprint(a, aSize, n, "GSV");
        nmea_sprint(a, aSize, n, ",%u,%u,%02u", numMsgs, msgNum, numSigs);        // 1,2,3 - numMsgs, msgNum, numSats in view

        // Write message payload: {svid,elv,az,cno} up to 4x
        for (int cnt=0; cnt<4 && i<=gsig.numSigs; i++)
        {
            gnss_sig_sv_t &sig = gsig.sig[i];

            // check if freqency is enabled and that the signals match
            if (gsv_freq_ena(&gsig.sig[i]) && gsv_sig_match(gnssId, sigId, sig, noCno))
            {    
                for (uint32_t j=0; j<=gsat.numSats; j++)
                {
                    gnss_sat_sv_t &sat = gsat.sat[j];
                    if ((sat.gnssId == sig.gnssId) && (sat.svId == sig.svId))
                    {
                        uint16_t svId = prnToSvId(sig.gnssId, sig.svId);
                        if (s_protocol_version < NMEA_PROTOCOL_4P10)
                        {
                            svId = sigId_to_nmea2p3_svId(gnssId, sig.sigId, svId);
                        }
                        nmea_sprint(a, aSize, n, ",%02u", svId);    // 4 + 4*msgNum... svid
                        nmea_print_i32(a, aSize, n, 2, sat.elev);   // 5 + 4*msgNum... elevation
                        nmea_print_i32(a, aSize, n, 3, sat.azim);   // 6 + 4*msgNum... azimuth
                        nmea_print_u32(a, aSize, n, 2, sig.cno);    // 7 + 4*msgNum... cno
                        ++cnt;
                        break;
                    }
                }
            }
        }

        // Write message footer
        if (s_protocol_version >= NMEA_PROTOCOL_4P10)
        {
            nmea_sprint(a, aSize, n, ",%c", sigId_to_nmea4p11_signalId(gnssId, sigId));    // Signal ID
        }
        nmea_sprint_footer(a, aSize, n);

        // Move buffer pointer
        a += n;
        aSize -= n;
    } 

    return (int)(a - bufStart);
}


/**
 * @brief Builds all $GxGSV messages for a single GNSS constellation, one signal group at a time.
 * @note For NMEA protocol versions < 4.10 (no per-signal grouping/Signal ID field), all signals for the constellation are combined into a single call to nmea_gsv_group() with sigId=0xFF (wildcard). For protocol >= 4.10, one nmea_gsv_group() call (and thus one or more $GxGSV messages) is made per known signal ID for the constellation, so each frequency band gets its own message group with the Signal ID field populated.
 *
 * @param a[] - output buffer for all generated $GxGSV messages (concatenated)
 * @param aSize - size of output buffer
 * @param gsat - GNSS satellite list (gnss_sat_t)
 * @param gsig - GNSS tracked-signal list (gnss_sig_t)
 * @param gnssId - GNSS constellation ID to generate messages for (see eSatSvGnssId); GPS/Galileo/BeiDou/QZSS/GLONASS/NavIC supported, others return 0
 * @param noCno - unused (reserved; always passed through as false internally to nmea_gsv_group)
 *
 * @return total number of characters written to a[] across all generated messages
 */
int nmea_gsv_gnss(char a[], int aSize, gnss_sat_t &gsat, gnss_sig_t &gsig, uint8_t gnssId, bool noCno)
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
        case SAT_SV_GNSS_ID_GPS:    sigIds = gpsSigIds; numSigIds = sizeof(gpsSigIds);  break;
        case SAT_SV_GNSS_ID_GAL:    sigIds = galSigIds; numSigIds = sizeof(galSigIds);  break;
        case SAT_SV_GNSS_ID_BEI:    sigIds = beiSigIds; numSigIds = sizeof(beiSigIds);  break;
        case SAT_SV_GNSS_ID_QZS:    sigIds = qzsSigIds; numSigIds = sizeof(qzsSigIds);  break;
        case SAT_SV_GNSS_ID_GLO:    sigIds = gloSigIds; numSigIds = sizeof(gloSigIds);  break;
        case SAT_SV_GNSS_ID_IRN:    sigIds = nvcSigIds; numSigIds = sizeof(nvcSigIds);  break;
        default: return 0;
    }

    for (int i = 0; i<numSigIds; i++)
    {
        n += nmea_gsv_group(a+n, aSize-n, gsat, gsig, gnssId, sigIds[i]);
    }

    return n;
}

/**
 * @brief Builds $GxGSV (Satellites in View) NMEA messages for every enabled GNSS constellation.
 * @note SBAS is skipped (SBAS satellites are reported under the GPS talker, see gsv_sig_match()). A constellation is only included if its s_gsvMask filter bits are non-zero (see nmea_setGsvFilter()/gsv_get_const_mask()).
 *
 * @param a[] - output buffer for all generated $GxGSV messages (concatenated)
 * @param aSize - size of output buffer
 * @param gsat - GNSS satellite list (gnss_sat_t)
 * @param gsig - GNSS tracked-signal list (gnss_sig_t)
 *
 * @return total number of characters written to a[] across all generated messages
 */
int nmea_gsv(char a[], const int aSize, gnss_sat_t &gsat, gnss_sig_t &gsig)
{
    int n = 0;

    // eSatSvGnssId
    for (int gnssId=1; gnssId<=SAT_SV_GNSS_ID_IRN; gnssId++)
    {
        if (gnssId != SAT_SV_GNSS_ID_SBS && (s_gsvMask.constMask[gnssId])) 
        {
            // printf("gnssId: %d\n", gnssId);

            // With CNO
            if ((aSize - n) > 0)
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

/**
 * @brief Parses a $INFO proprietary NMEA message into a device info structure.
 * @note Field order/meaning mirrors nmea_dev_info() (the corresponding builder). Older firmware ($INFO from firmware <= 2.6) is known to misreport hdwRunState as bootloader; this is corrected to HDW_STATE_APP. Trailing fields (build date/time/addInfo/hardware/run state/build type/flags) are only parsed if present in the buffer, for compatibility with older/shorter $INFO messages.
 *
 * @param info - [out] parsed device info (dev_info_t)
 * @param a[] - incoming NMEA sentence buffer
 * @param aSize - size of a[]
 *
 * @note parsed message fields (see nmea_dev_info() for full field list):
 *  1   Serial number -> info.serialNumber
 *  2   Hardware version -> info.hardwareVer[4]
 *  3   Firmware version -> info.firmwareVer[4]
 *  4   Build number -> info.buildNumber
 *  5   Protocol version -> info.protocolVer[4]
 *  6   Repository revision -> info.repoRevision
 *  7   Manufacturer -> info.manufacturer
 *  8   Build date (YYYY-MM-DD) -> info.buildYear/buildMonth/buildDay
 *  9   Build time (HH:MM:SS.ms) -> info.buildHour/buildMinute/buildSecond/buildMillisecond
 *  10  Additional info string -> info.addInfo
 *  11  Hardware type -> info.hardwareType (see eIsHardwareType)
 *  12  Hardware run state -> info.hdwRunState (see eHdwRunStates)
 *  13  Build type (numeric or single character) -> info.buildType
 *  14  Build flags bitmask -> info.buildFlags (see eBuildFlags):
 *        bit0 = debug mode
 *        bit1 = dirty working tree
 *
 * @return 0 always
 */
int nmea_parse_info(dev_info_t &info, const char a[], const int aSize)
{
    (void)aSize;
    char *ptr = (char *)&a[6];  // $INFO,
    
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
    if (ptr < a + aSize)
        ptr = ASCII_to_char_array(info.manufacturer, ptr, DEVINFO_MANUFACTURER_STRLEN);

    // uint8_t         buildDate[4];    YYYY-MM-DD
    if (ptr < a + aSize) {
        unsigned int year, month, day;
        SSCANF(ptr, "%04d-%02u-%02u", &year, &month, &day);
        info.buildType = ' ';
        info.buildYear = (uint8_t) (year >= 2000 ? (year - 2000) : year);
        info.buildMonth = (uint8_t) (month);
        info.buildDay = (uint8_t) (day);
        ptr = ASCII_find_next_field(ptr);
    }

    // uint8_t         buildTime[4];    hh:mm:ss.ms
    if (ptr < a + aSize) {
        unsigned int hour, minute, second, ms;
        SSCANF(ptr, "%02u:%02u:%03u.%02u", &hour, &minute, &second, &ms);
        info.buildHour = (uint8_t) hour;
        info.buildMinute = (uint8_t) minute;
        info.buildSecond = (uint8_t) second;
        info.buildMillisecond = (uint8_t) ms;
        ptr = ASCII_find_next_field(ptr);
    }

    // char            addInfo[DEVINFO_ADDINFO_STRLEN];
    if (ptr < a + aSize)
        ptr = ASCII_to_char_array(info.addInfo, ptr, DEVINFO_ADDINFO_STRLEN);

    // uint8_t        hardware;
    if (ptr < a + aSize)
        ptr = ASCII_to_u8(&info.hardwareType, ptr);

    // uint8_t        hdwRunState;
    if (ptr < a + aSize) {
        ptr = ASCII_to_u8(&info.hdwRunState, ptr);
        // some older firmware incorrectly report this as '1' (HDW_STATE_BOOTLOADER), so correct them
        if ((info.firmwareVer[0] <= 2) && (info.firmwareVer[1] <= 6))
            info.hdwRunState = HDW_STATE_APP;
    }

    // uint8_t         build type;
    if (ptr < a + aSize) {
        if (std::isdigit((unsigned char)*ptr)) {
            ptr = ASCII_to_u8(&info.buildType, ptr);
        }
        else {
            info.buildType = (uint8_t)*ptr;
            ptr = ASCII_find_next_field(ptr);
        }
    }
    if (info.buildType == ' ' || info.buildType == 'r') { info.buildType = 0; }  // normalize legacy/encoded production values

    // uint8_t         buildFlags;
    if (ptr < a + aSize) {
        ptr = ASCII_to_u8(&info.buildFlags, ptr);
    }

    // Populate missing hardware descriptor
    devInfoPopulateMissingHardware(&info);

    return 0;
}

/**
 * @brief Parses a $PIMU proprietary NMEA message into a raw IMU sample. See nmea_pimu() for the wire field order.
 *
 * @param imu - [out] parsed IMU sample (imu_t): imu.time (field 1), imu.I.pqr[3] angular rate (fields 2-4), imu.I.acc[3] linear acceleration (fields 5-7)
 * @param a[] - incoming NMEA sentence buffer
 * @param aSize - size of a[] (unused)
 *
 * @return 0 always
 */
int nmea_parse_pimu(imu_t &imu, const char a[], const int aSize)
{
    (void)aSize;
    char *ptr = (char *)&a[6];    // $PIMU,

    // Time since system powerup
    ptr = ASCII_to_f64(&(imu.time), ptr);

    // PQR angular rate
    ptr = ASCII_to_vec3f(imu.I.pqr, ptr);
    // XYZ linear acceleration
    ptr = ASCII_to_vec3f(imu.I.acc, ptr);

    return 0;
}

/**
 * @brief Parses a $PRIMU (reference IMU) proprietary NMEA message into a raw IMU sample. Same field layout as $PIMU, parsed into a reference-IMU struct. See nmea_pimu() for the wire field order.
 *
 * @param imu - [out] parsed IMU sample (imu_t): imu.time (field 1), imu.I.pqr[3] angular rate (fields 2-4), imu.I.acc[3] linear acceleration (fields 5-7)
 * @param a[] - incoming NMEA sentence buffer
 * @param aSize - size of a[] (unused)
 *
 * @return 0 always
 */
int nmea_parse_pimu_to_rimu(imu_t &imu, const char a[], const int aSize)
{
    (void)aSize;
    char *ptr = (char *)&a[7];    // $PRIMU,

    // Time since system powerup
    ptr = ASCII_to_f64(&(imu.time), ptr);

    // PQR angular rate
    ptr = ASCII_to_vec3f(imu.I.pqr, ptr);
    // XYZ linear acceleration
    ptr = ASCII_to_vec3f(imu.I.acc, ptr);

    return 0;
}

/**
 * @brief Parses a $PPIMU proprietary NMEA message into a preintegrated IMU sample. See nmea_ppimu() for the wire field order.
 *
 * @param pimu - [out] parsed preintegrated IMU sample (pimu_t): pimu.time (field 1), pimu.theta[3] delta theta (fields 2-4), pimu.vel[3] delta velocity (fields 5-7), pimu.dt integration period (field 8)
 * @param a[] - incoming NMEA sentence buffer
 * @param aSize - size of a[] (unused)
 *
 * @return 0 always
 */
int nmea_parse_ppimu(pimu_t &pimu, const char a[], const int aSize)
{
    (void)aSize;
    char *ptr = (char *)&a[7];    // $PPIMU,

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

/**
 * @brief Parses a $PINS1 proprietary NMEA message into an INS navigation solution. See nmea_pins1() for the wire field order and insStatus/hdwStatus bitfield layout.
 *
 * @param ins - [out] parsed INS solution (ins_1_t): timeOfWeek/week (fields 1-2), insStatus (field 3, see eInsStatusFlags), hdwStatus (field 4, see eHdwStatusFlags), theta[3] roll/pitch/yaw (fields 5-7), uvw[3] body velocity (fields 8-10), lla[3] position (fields 11-13), ned[3] NED offset (fields 14-16)
 * @param a[] - incoming NMEA sentence buffer
 * @param aSize - size of a[] (unused)
 *
 * @return 0 always
 */
int nmea_parse_pins1(ins_1_t &ins, const char a[], const int aSize)
{
    (void)aSize;
    char *ptr = (char *)&a[7];    // $PINS1,

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

/**
 * @brief Parses a $PINS2 proprietary NMEA message into an INS navigation solution. See nmea_pins2() for the wire field order and insStatus/hdwStatus bitfield layout.
 *
 * @param ins - [out] parsed INS solution (ins_2_t): timeOfWeek/week (fields 1-2), insStatus (field 3, see eInsStatusFlags), hdwStatus (field 4, see eHdwStatusFlags), qn2b[4] attitude quaternion (fields 5-8), uvw[3] body velocity (fields 9-11), lla[3] position (fields 12-14)
 * @param a[] - incoming NMEA sentence buffer
 * @param aSize - size of a[] (unused)
 *
 * @return 0 always
 */
int nmea_parse_pins2(ins_2_t &ins, const char a[], const int aSize)
{
    (void)aSize;
    char *ptr = (char *)&a[7];    // $PINS2,

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

/**
 * @brief Parses a $PGPSP proprietary NMEA message into a GNSS position/velocity solution. See nmea_pgpsp() for the wire field order and status bitfield layout.
 *
 * @param gnssPos - [out] parsed GNSS position (gnss_pos_t): timeOfWeekMs/week (fields 1-2), status (field 3, see eGnssStatus - fix type in bits[12:8], flags in bits[28:13]; satsUsed is also derived from status's low byte), lla[3] (fields 4-6), hMSL (field 7), pDop/hAcc/vAcc (fields 8-10), cnoMean (field 15), towOffset (field 16), leapS (field 17)
 * @param gnssVel - [out] parsed GNSS velocity (gnss_vel_t): timeOfWeekMs (copied from gnssPos), vel[3] (fields 11-13), sAcc (field 14)
 * @param a[] - incoming NMEA sentence buffer
 * @param aSize - size of a[] (unused)
 *
 * @return 0 always
 */
int nmea_parse_pgpsp(gnss_pos_t &gnssPos, gnss_vel_t &gnssVel, const char a[], const int aSize)
{
    (void)aSize;
    char *ptr = (char *)&a[7];    // $PGPSP,

    // GPS timeOfWeekMs, week
    ptr = ASCII_to_u32(&(gnssPos.timeOfWeekMs), ptr);
    ptr = ASCII_to_u32(&(gnssPos.week), ptr);
    gnssVel.timeOfWeekMs = gnssPos.timeOfWeekMs;

    // status
    ptr = ASCII_to_u32(&(gnssPos.status), ptr);
    gnssPos.satsUsed = gnssPos.status & GNSS_STATUS_NUM_SATS_USED_MASK;

    // LLA, MSL altitude
    ptr = ASCII_to_vec3d(gnssPos.lla, ptr);
    ptr = ASCII_to_f32(&(gnssPos.hMSL), ptr);

    // pDop, hAcc, vAcc
    ptr = ASCII_to_f32(&(gnssPos.pDop), ptr);
    ptr = ASCII_to_f32(&(gnssPos.hAcc), ptr);
    ptr = ASCII_to_f32(&(gnssPos.vAcc), ptr);

    // Velocity, sAcc
    ptr = ASCII_to_vec3f(gnssVel.vel, ptr);
    ptr = ASCII_to_f32(&(gnssVel.sAcc), ptr);

    // cnoMean
    ptr = ASCII_to_f32(&(gnssPos.cnoMean), ptr);

    // Time of Week offset, leapS
    ptr = ASCII_to_f64(&(gnssPos.towOffset), ptr);
    ptr = ASCII_to_u8(&(gnssPos.leapS), ptr);

    return 0;
}

/**
 * @brief Parses a $ASCE special-case message ID that targets a specific GSV constellation + frequency-band mask, and updates the GSV filter mask accordingly.
 * @note Special-case IDs pack a target constellation into the upper nibble and a frequency-band bitmask into the lower nibble: bits [7:4] = constTarget (see eSatSvGnssId; SAT_SV_GNSS_ID_GNSS applies the mask to all constellations), bits [3:0] = freqMask (NMEA_GNGSV_FREQ_BAND1_BIT/BAND2_BIT/FREQ_5_BIT). Only applied if period != 0 (period==0 leaves the mask untouched, matching a "disable" request handled by the caller).
 *
 * @param inId - special-case NMEA message ID encoding constellation (high nibble) and frequency mask (low nibble); must be within [NMEA_MSG_ID_GNGSV_START, NMEA_MSG_ID_GNGSV_END]
 * @param period - requested broadcast period multiple; the GSV mask is only updated when non-zero
 *
 * @return NMEA_MSG_ID_GNGSV if successful, or 0 if inId is out of range or constTarget is unrecognized
 */
int parseASCE_GSV(int inId, int period)
{
    uint8_t constTarget = (inId & 0xf0) >> 4;
    uint8_t freqMask = (inId & 0x0f);

    if (inId < NMEA_MSG_ID_GNGSV_START || inId > NMEA_MSG_ID_GNGSV_END)
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

/**
 * @brief Parses a $ASCE (Ask/Set Communications Enable) message, enabling/disabling the requested NMEA messages at the requested periods on the requested port(s) of an rmci_t configuration vector.
 *
 * @param port - port the message was received on (used only when options select RMC_OPTIONS_PORT_CURRENT); if NULL, parsing is aborted
 * @param a[] - incoming NMEA sentence buffer
 * @param aSize - size of a[]; used to bound talker-string message-ID lookups
 * @param rmci - per-port NMEA broadcast configuration vector (indexed by port ID) to update
 *
 * @note parsed message fields: $ASCE,options,{msgID,msgPeriod}...
 *  1    options - bitmask (decimal or 0x-prefixed hex):
 *          bits[7:0]  = destination port selector (RMC_OPTIONS_PORT_MASK):
 *                         RMC_OPTIONS_PORT_CURRENT = port the message was received on
 *                         RMC_OPTIONS_PORT_ALL     = all ports
 *                         individual SER0/SER1/SER2/USB bits = specific port(s)
 *          bits[11:10] = NMEA speed-filter mode (RMC_OPTIONS_NMEA_SPEED_FILTER_BITMASK):
 *                         1 = enable filtering of small GNSS-noise-induced velocity in $GLL/$RMC/$VTG
 *                         2 = disable that filtering
 *  2n   msgID - NMEA message ID to enable/disable, either numeric or a talker string (looked up via getNmeaMsgId()); $GxGSV special-case IDs (see parseASCE_GSV()) select a constellation + frequency-band mask
 *  2n+1 msgPeriod - broadcast period multiple for that message (0 disables/single-shot, matching nmea_enable_stream())
 *      (the msgID,msgPeriod pair repeats for each requested message, up to 20 pairs, terminated by '*')
 *
 * @return the parsed options bitmask (0 if port is NULL)
 */
uint32_t nmea_parse_asce(port_handle_t port, const char a[], int aSize, std::vector<rmci_t*> rmci)
{
    (void)aSize;

    uint32_t options = 0;
    uint32_t id;
    uint32_t ports;
    uint8_t period;

    if (!port)
        return 0;
    
    char *ptr = (char*)&a[6];                // $ASCE
    char *end = (char*)&a[aSize];

    // extract options
    ptr = ASCII_options_to_u32(&options, ptr);

    // extract port from options
    ports = options&RMC_OPTIONS_PORT_MASK;

    // speed filter if requested
    switch ((options & RMC_OPTIONS_NMEA_SPEED_FILTER_BITMASK) >> RMC_OPTIONS_NMEA_SPEED_FILTER_OFFSET)
    {
        case RMC_OPTIONS_NMEA_SPEED_FILTER_ENABLE:
            s_dataSpeed.enableSpeedFilter = true;
            break;
        case RMC_OPTIONS_NMEA_SPEED_FILTER_DISABLE:
            s_dataSpeed.enableSpeedFilter = false;
            break;
    }
    
    for (int i=0; i<20; i++)
    {
        // end of nmea string
        if (*ptr == '*')
             break;
        
        // set id and increment ptr to next field
        if (isdigit(*ptr))
        {   // Is a number.  Read NMEA ID directly
            id = ((*ptr == ',') ? 0 : atoi(ptr));
        }
        else
        {   // Is a letter.  Convert talker string to NMEA ID
            char *ptr2 = ptr-1;
            id = getNmeaMsgId(ptr2, end-ptr2);
        }

        ptr = ASCII_find_next_field(ptr);

        // end of nmea string
        if (*ptr=='*')
            break;
        
        // set period multiple and increament ptr to next field
        period = ((*ptr==',') ? 0 : (uint8_t)atoi(ptr));    
        ptr = ASCII_find_next_field(ptr);

        // handle GSV cases
        if (id == NMEA_MSG_ID_GNGSV)
            parseASCE_GSV(NMEA_MSG_ID_GNGSV_5_3_2_1, period);
        else if (id >= NMEA_MSG_ID_GNGSV_START && id <= NMEA_MSG_ID_GNGSV_END)
            id = parseASCE_GSV(id, period);

        // Copy tmp to corresponding port(s)
        switch (ports)
        {    
        case RMC_OPTIONS_PORT_CURRENT:
            nmea_enable_stream(rmci[portId(port)]->rmcNmea.nmeaBits, rmci[portId(port)]->rmcNmea.nmeaPeriod, id, period);
            break;
        case RMC_OPTIONS_PORT_ALL:
            for (int i=0; i < NUM_COM_PORTS; i++) {
                nmea_enable_stream(rmci[i]->rmcNmea.nmeaBits, rmci[i]->rmcNmea.nmeaPeriod, id,  period);
            }
            if (id == NMEA_MSG_ID_GNGSV && period == 0)
            {
                for (int i = SAT_SV_GNSS_ID_GNSS; i < SAT_SV_GNSS_ID_COUNT; i++)
                    s_gsvMask.constMask[i] = 0;
            }
            break;
        default:    // Current port
            if (ports & RMC_OPTIONS_PORT_SER0)     { nmea_enable_stream(rmci[0]->rmcNmea.nmeaBits, rmci[0]->rmcNmea.nmeaPeriod, id, period); }
            if (ports & RMC_OPTIONS_PORT_SER1)     { nmea_enable_stream(rmci[1]->rmcNmea.nmeaBits, rmci[1]->rmcNmea.nmeaPeriod, id, period); }
            if (ports & RMC_OPTIONS_PORT_SER2)     { nmea_enable_stream(rmci[2]->rmcNmea.nmeaBits, rmci[2]->rmcNmea.nmeaPeriod, id, period); }
            if (ports & RMC_OPTIONS_PORT_USB)      { nmea_enable_stream(rmci[3]->rmcNmea.nmeaBits, rmci[3]->rmcNmea.nmeaPeriod, id, period); }
            break;
        }
    }
        
    return options;
}

/**
 * @brief Enables an NMEA message ID/period on a single indexed port of a GPX GRMC configuration vector, and OR's in any persistent options flags.
 *
 * @param grmci - per-port GPX GRMC broadcast configuration vector; no-op if i is out of range
 * @param i - index of the target port within grmci
 * @param id - NMEA message ID to enable (see nmea_enable_stream())
 * @param period - broadcast period multiple for the message
 * @param options - options bitmask from the originating $ASCE message; only the RMC_OPTIONS_PERSISTENT bit is retained and OR'd into grmci[i]->rmc.options
 */
inline void nmea_configure_grmci(const std::vector<grmci_t*>& grmci, int i, uint32_t id, uint8_t period, uint32_t options)
{
    if (i < ((int)grmci.size()))
    {
        nmea_enable_stream(grmci[i]->rmcNmea.nmeaBits, grmci[i]->rmcNmea.nmeaPeriod, id, period);
        grmci[i]->rmc.options |= (options & RMC_OPTIONS_PERSISTENT);
    }
}

/**
 * @brief Parses a $ASCE (Ask/Set Communications Enable) message and applies it to a GPX GRMC (device-to-device) broadcast configuration vector rather than the standard rmci_t vector (see nmea_parse_asce()).
 *
 * @param port port_handle_t the msg was Rxd on; if NULL, parsing is aborted
 * @param a const char[] incoming msg
 * @param aSize int size of msg a; used to bound talker-string message-ID lookups
 * @param grmci std::vector<grmci_t*> of GPX GRMC bits
 *
 * @note parsed message fields: $ASCE,options,{msgID,msgPeriod}...
 *  1    options - bitmask (decimal or 0x-prefixed hex):
 *          bits[7:0] = destination port selector (RMC_OPTIONS_PORT_MASK):
 *                        RMC_OPTIONS_PORT_CURRENT = port the message was received on
 *                        RMC_OPTIONS_PORT_ALL     = all ports
 *                        individual SER0/SER1/SER2/USB bits = specific port(s)
 *          bit9      = RMC_OPTIONS_PERSISTENT: save configuration to flash for use after reboot
 *  2n   msgID - NMEA message ID to enable/disable, either numeric or a talker string (looked up via getNmeaMsgId()); $GxGSV special-case IDs (see parseASCE_GSV()) select a constellation + frequency-band mask
 *  2n+1 msgPeriod - broadcast period multiple for that message
 *      (the msgID,msgPeriod pair repeats for each requested message, up to 20 pairs, terminated by '*')
 *
 * @return 0 on NULL port error
 * @return options if any (can be 0 if no options exist)
 */
uint32_t nmea_parse_asce_grmci(port_handle_t port, const char a[], int aSize, std::vector<grmci_t*> grmci)
{
    (void)aSize;

    uint32_t options = 0;
    uint32_t id;
    uint32_t ports;
    uint8_t period;

    if (!port)
        return 0;
    
    char *ptr = (char*)&a[6];                // $ASCE
    char *end = (char*)&a[aSize];
    
    // check if next index is ','
    if (*ptr != ',')
        options = (uint32_t)atoi(ptr);
    
    // get next uint32_t and assign it to options and move pointer
    ptr = ASCII_to_u32(&options, ptr);

    // extract port from options
    ports = options & RMC_OPTIONS_PORT_MASK;
    
    for (int i = 0; i < 20; i++)
    {
        // end of nmea string
        if (*ptr == '*')
             break;
        
        // set id and increament ptr to next field
        if (isdigit(*ptr))
        {   // Is a number.  Read NMEA ID directly
            id = ((*ptr == ',') ? 0 : atoi(ptr));
        }
        else
        {   // Is a letter.  Convert talker string to NMEA ID
            char *ptr2 = ptr-1;
            id = getNmeaMsgId(ptr2, end-ptr2);
        }

        ptr = ASCII_find_next_field(ptr);

        // end of nmea string
        if (*ptr=='*')
            break;
        
        // set period multiple and increament ptr to next field
        period = ((*ptr==',') ? 0 : (uint8_t)atoi(ptr));    
        ptr = ASCII_find_next_field(ptr);

        // handle GSV cases
        if (id == NMEA_MSG_ID_GNGSV)
            parseASCE_GSV(NMEA_MSG_ID_GNGSV, period);
        else if (id >= NMEA_MSG_ID_SPECIAL_CASE_START)
            id = parseASCE_GSV(id, period);

        // Copy tmp to corresponding port(s)
        switch (ports)
        {    
        case RMC_OPTIONS_PORT_CURRENT:
            nmea_configure_grmci(grmci, portId(port), id, period, options);
            break;
        
        case RMC_OPTIONS_PORT_ALL:        
            for (int i=0; i<((int)grmci.size()); i++) 
            {
                nmea_configure_grmci(grmci, i, id, period, options);
            }

            if (id == NMEA_MSG_ID_GNGSV && period == 0)
            {
                for (int i = SAT_SV_GNSS_ID_GNSS; i < SAT_SV_GNSS_ID_COUNT; i++)
                    s_gsvMask.constMask[i] = 0;
            }
            break;
            
        default:    // Current port
            if (ports & RMC_OPTIONS_PORT_SER0)  nmea_configure_grmci(grmci, 0, id, period, options);
            if (ports & RMC_OPTIONS_PORT_SER1)  nmea_configure_grmci(grmci, 1, id, period, options);
            if (ports & RMC_OPTIONS_PORT_SER2)  nmea_configure_grmci(grmci, 2, id, period, options);
            if (ports & RMC_OPTIONS_PORT_USB)   nmea_configure_grmci(grmci, 3, id, period, options);
            break;
        }
    }
        
    return options;
}

/**
 * @brief Parses a $GxGNS (GNSS Fix Data) message into a GNSS position solution, converting the multi-character positioning-mode field to internal fix-type/flags/accuracy values and computing ECEF position.
 * @note $GNS reports one mode character per constellation contributing to the fix (up to 4); the strongest fix type found across those characters (R=RTK fix > F=RTK float > D=DGPS > A=autonomous > E=dead reckoning) determines gnssPos.status's fix-type bits and a corresponding fixed hAcc/vAcc estimate (ZED-F9P-datasheet-derived), since $GNS itself does not report accuracy.
 *
 * @param a[] - incoming NMEA sentence buffer: $xxGNS,time,lat,NS,lon,EW,posMode,numSV,HDOP,alt,sep,diffAge,diffStation,navStatus*cs
 * @param aSize - size of a[] (unused)
 * @param gnssPos - [out] parsed GNSS position (gnss_pos_t): timeOfWeekMs (field 1), lla[0..1] (fields 2-5), status fix-type/flags/satsUsed (fields 6-7, see eGnssStatus), hMSL/lla[2] (fields 9-10), ecef[3] (derived)
 * @param utcTime - [out] parsed UTC time breakdown (field 1)
 * @param utcWeekday - UTC day of week (0=Sunday) used to convert UTC time-of-day to GPS time of week
 * @param statusFlags - additional eGnssStatus flag bits (see data_sets.h) to OR into gnssPos.status (e.g. caller-supplied context flags); GNSS_STATUS_FLAGS_GNSS_NMEA_DATA is always added
 *
 * @note parsed message fields:
 *  1   UTC time (HHMMSS.sss) -> utcTime / gnssPos.timeOfWeekMs
 *  2,3 Latitude (ddmm.mmmm, N/S) -> gnssPos.lla[0]
 *  4,5 Longitude (dddmm.mmmm, E/W) -> gnssPos.lla[1]
 *  6   Positioning mode, up to 4 characters, one per GNSS: N=no fix, A=autonomous, D=differential, R=RTK fixed, F=RTK float, E=dead reckoning -> gnssPos.status fix-type bits + hAcc/vAcc estimate
 *  7   Number of satellites used -> gnssPos.satsUsed / status low byte
 *  8   HDOP (not stored)
 *  9   MSL altitude -> gnssPos.hMSL
 *  10  Geoid separation -> combined with field 9 into gnssPos.lla[2] (ellipsoid altitude)
 *  11  Age of differential corrections (not stored)
 *  12  Differential station ID (not stored)
 *  13  Navigational status (not stored)
 *
 * @return 0 always
 */
int nmea_parse_gns(const char a[], const int aSize, gnss_pos_t &gnssPos, utc_time_t &utcTime, int utcWeekday, uint32_t statusFlags)
{
    (void)aSize;
    char *ptr = (char *)&a[7];    // $GxGNS,
    //$xxGNS,time,lat,NS,lon,EW,posMode,numSV,HDOP,alt,sep,diffAge,diffStation,navStatus*cs<CR><LF>

    // 1 - UTC time HHMMSS.sss
    ptr = ASCII_UtcTimeToGpsTowMs(&gnssPos.timeOfWeekMs, &utcTime, ptr, utcWeekday, gnssPos.leapS);
        
    //Latitude
    ixVector3d lla;
    lla[0] = ddmm2deg(atof(ptr));
    ptr = ASCII_find_next_field(ptr);
    if (*ptr == 'S')
        lla[0] = -lla[0];
    ptr = ASCII_find_next_field(ptr);

    //Longitude
    lla[1] = ddmm2deg(atof(ptr));
    ptr = ASCII_find_next_field(ptr);
    if (*ptr == 'W')
        lla[1] = -lla[1];
    ptr = ASCII_find_next_field(ptr);

    //Positioning Mode
    char pMode[4] = {0,0,0,0};
    if (*ptr != ',')
        pMode[0] = *ptr++;
    if (*ptr != ',')
        pMode[1] = *ptr++;
    if (*ptr != ',')
        pMode[2] = *ptr++;
    if (*ptr != ',')
        pMode[3] = *ptr++;
    ptr = ASCII_find_next_field(ptr);
        
    //Based off of ZED-F9P datasheet
    uint32_t fixType = GNSS_STATUS_FIX_NONE;
    statusFlags |= GNSS_STATUS_FLAGS_GNSS_NMEA_DATA;
    gnssPos.hAcc = 0.0f;
    if (pMode[0] == 'R' || pMode[1] == 'R' || pMode[2] == 'R' || pMode[3] == 'R')        // RTK fix
    {
        fixType = GNSS_STATUS_FIX_RTK_FIX;
        statusFlags |= 
            GNSS_STATUS_FLAGS_FIX_OK |
            GNSS_STATUS_FLAGS_GNSS1_RTK_POSITION_ENABLED |
            GNSS_STATUS_FLAGS_GNSS1_RTK_POSITION_VALID |
            GNSS_STATUS_FLAGS_RTK_FIX_AND_HOLD |
            GNSS_STATUS_FLAGS_DGPS_USED;
        gnssPos.hAcc = 0.05f;
    }
    else if (pMode[0] == 'F' || pMode[1] == 'F' || pMode[2] == 'F' || pMode[3] == 'F')    // RTK float
    {
        fixType = GNSS_STATUS_FIX_RTK_FLOAT;
        statusFlags |=
            GNSS_STATUS_FLAGS_FIX_OK |
            GNSS_STATUS_FLAGS_GNSS1_RTK_POSITION_ENABLED |
            GNSS_STATUS_FLAGS_DGPS_USED;
        gnssPos.hAcc = 0.4f;
    }
    else if (pMode[0] == 'D' || pMode[1] == 'D' || pMode[2] == 'D' || pMode[3] == 'D')    // Differential (DGPS)
    {
        fixType = GNSS_STATUS_FIX_DGPS;
        statusFlags |= 
            GNSS_STATUS_FLAGS_FIX_OK |
            GNSS_STATUS_FLAGS_DGPS_USED;
        gnssPos.hAcc = 0.8f;
    }
    else if (pMode[0] == 'A' || pMode[1] == 'A' || pMode[2] == 'A' || pMode[3] == 'A')    // Autonomous, 2D/3D
    {
        fixType = GNSS_STATUS_FIX_3D;
        statusFlags |= GNSS_STATUS_FLAGS_FIX_OK;
        gnssPos.hAcc = 1.5f;
    }
    else if (pMode[0] == 'E' || pMode[1] == 'E' || pMode[2] == 'E' || pMode[3] == 'E')    // Dead reckoning
    {
        fixType = GNSS_STATUS_FIX_DEAD_RECKONING_ONLY;
    }
    gnssPos.vAcc = 1.4f * gnssPos.hAcc;
            
    //Number of satellites used in solution
    gnssPos.satsUsed = atoi(ptr);
    ptr = ASCII_find_next_field(ptr);
        
    //HDOP
    ptr = ASCII_find_next_field(ptr);
        
    //MSL Altitude (altitude above mean sea level)
    lla[2] = atof(ptr);
    gnssPos.hMSL = (float)lla[2];
    ptr = ASCII_find_next_field(ptr);

    //Geoid separation (difference between ellipsoid and mean sea level)
    double sep = atof(ptr);
        
    //Store data        
    set_gnssPos_status_mask(&(gnssPos.status), gnssPos.satsUsed, (uint32_t)GNSS_STATUS_NUM_SATS_USED_MASK);
    set_gnssPos_status_mask(&(gnssPos.status), statusFlags, (uint32_t)GNSS_STATUS_FLAGS_MASK);
    set_gnssPos_status_mask(&(gnssPos.status), fixType, (uint32_t)GNSS_STATUS_FIX_MASK);
        
    gnssPos.lla[0] = lla[0];
    gnssPos.lla[1] = lla[1];
    gnssPos.lla[2] = lla[2] + sep;

    //Change LLA to radians
    lla[0] = C_DEG2RAD * lla[0];
    lla[1] = C_DEG2RAD * lla[1];
    lla[2] = gnssPos.lla[2];    // Use ellipsoid alt
        
    //Convert LLA to ECEF.  Ensure LLA uses ellipsoid alt 
    ixVector3d ecef;
    lla2ecef(lla, ecef);
                
    gnssPos.ecef[0] = ecef[0];
    gnssPos.ecef[1] = ecef[1];
    gnssPos.ecef[2] = ecef[2];    

    return 0;    
}

/**
 * @brief Parses a $GxGGA (Global Positioning System Fix Data) message into a GNSS position solution, converting the fix-quality code to internal fix-type/flags/accuracy values and computing ECEF position.
 * @note The standard NMEA fix-quality codes (field 6) are mapped to internal fix types (GNSS_STATUS_FIX_* / eGnssStatus) with fixed hAcc/vAcc accuracy estimates per fix type, since $GGA itself does not report accuracy directly.
 *
 * @param a[] - incoming NMEA sentence buffer: $xxGGA,time,lat,NS,lon,EW,quality,numSV,HDOP,alt,M,sep,M,diffAge,diffStation*cs
 * @param aSize - size of a[] (unused)
 * @param gnssPos - [out] parsed GNSS position (gnss_pos_t): timeOfWeekMs (field 1), lla[0..1] (fields 2-5), status fix-type/flags (field 6, see eGnssStatus), satsUsed (field 7), pDop=HDOP (field 8), hMSL/lla[2] (fields 9-12), ecef[3] (derived)
 * @param utcTime - [out] parsed UTC time breakdown (field 1)
 * @param utcWeekday - UTC day of week (0=Sunday) used to convert UTC time-of-day to GPS time of week
 * @param statusFlags - additional eGnssStatus flag bits (see data_sets.h) to OR into gnssPos.status; GNSS_STATUS_FLAGS_GNSS_NMEA_DATA is always added
 *
 * @note parsed message fields:
 *  1     UTC time (HHMMSS.sss) -> utcTime / gnssPos.timeOfWeekMs
 *  2,3   Latitude (ddmm.mmmm, N/S) -> gnssPos.lla[0]
 *  4,5   Longitude (dddmm.mmmm, E/W) -> gnssPos.lla[1]
 *  6     Fix quality: 0=invalid,1=autonomous,2=differential,3=PPS,4=RTK fixed,5=RTK float,6=dead reckoning -> gnssPos.status fix-type bits + hAcc/vAcc estimate
 *  7     Number of satellites used -> gnssPos.satsUsed / status low byte
 *  8     Horizontal dilution of precision (HDOP) -> gnssPos.pDop
 *  9,10  MSL altitude, "M" units -> gnssPos.hMSL
 *  11,12 Geoid separation, "M" units -> combined with field 9 into gnssPos.lla[2] (ellipsoid altitude)
 *  13    Age of differential corrections (not stored)
 *  14    Differential station ID (not stored)
 *
 * @return 0 always
 */
int nmea_parse_gga(const char a[], const int aSize, gnss_pos_t &gnssPos, utc_time_t &utcTime, int utcWeekday, uint32_t statusFlags)
{
    (void)aSize;
    char *ptr = (char *)&a[7];    // $GxGGA,
    
    // 1 - UTC time HHMMSS.sss
    ptr = ASCII_UtcTimeToGpsTowMs(&gnssPos.timeOfWeekMs, &utcTime, ptr, utcWeekday, gnssPos.leapS);
    // 2,3 - Latitude (deg)
    ptr = ASCII_DegMin_to_Lat(&(gnssPos.lla[0]), ptr);
    // 4,5 - Longitude (deg)
    ptr = ASCII_DegMin_to_Lon(&(gnssPos.lla[1]), ptr);

    // 6 - Fix quality
    uint32_t fixQuality;
    ptr = ASCII_to_u32(&fixQuality, ptr);
    gnssPos.hAcc = 0.0f;
    gnssPos.vAcc = 0.0f;

    uint32_t fixType = GNSS_STATUS_FIX_NONE;
    switch(fixQuality)
    {
        case 6:        // Dead reckoning
            fixType = GNSS_STATUS_FIX_DEAD_RECKONING_ONLY;
            break;

        case 5:        // RTK float
            fixType = GNSS_STATUS_FIX_RTK_FLOAT;
            statusFlags |=
                GNSS_STATUS_FLAGS_FIX_OK |
                GNSS_STATUS_FLAGS_GNSS1_RTK_POSITION_ENABLED |
                GNSS_STATUS_FLAGS_DGPS_USED;
            gnssPos.hAcc = 0.4f;
            break;
        
        case 4:        // RTK fix
            fixType = GNSS_STATUS_FIX_RTK_FIX;
            statusFlags |= 
                GNSS_STATUS_FLAGS_FIX_OK |
                GNSS_STATUS_FLAGS_GNSS1_RTK_POSITION_ENABLED |
                GNSS_STATUS_FLAGS_GNSS1_RTK_POSITION_VALID |
                GNSS_STATUS_FLAGS_RTK_FIX_AND_HOLD |
                GNSS_STATUS_FLAGS_DGPS_USED;
            gnssPos.hAcc = 0.05f;
            break;

        case 3:        // Time only
            fixType = GNSS_STATUS_FIX_TIME_ONLY;
            gnssPos.hAcc = 0.8f;
            break;

        case 2:        // Differential
            fixType = GNSS_STATUS_FIX_DGPS;
            statusFlags |= 
                GNSS_STATUS_FLAGS_FIX_OK |
                GNSS_STATUS_FLAGS_DGPS_USED;
            gnssPos.hAcc = 0.8f;
            break;

        case 1:        // Autonomous
            fixType = GNSS_STATUS_FIX_3D;
            statusFlags |= GNSS_STATUS_FLAGS_FIX_OK;
            gnssPos.hAcc = 1.5f;
            break;

        default:    break;
    }
    
    // 7 - Satellites used
    ptr = ASCII_to_u8(&(gnssPos.satsUsed), ptr);

    gnssPos.status = statusFlags | fixType | GNSS_STATUS_FLAGS_GNSS_NMEA_DATA;
    gnssPos.status |= gnssPos.satsUsed;

    // 8 - hDop
    ptr = ASCII_to_f32(&(gnssPos.pDop), ptr);

    // 9,10 - MSL altitude
    ptr = ASCII_to_f32(&(gnssPos.hMSL), ptr);
    ptr = ASCII_find_next_field(ptr);

    // 11,12 - Geoid separation = alt(HAE) - alt(MSL)
    double geoidSep;
    ptr = ASCII_to_f64(&(geoidSep), ptr);
    gnssPos.lla[2] = double(gnssPos.hMSL) + geoidSep;

    // Convert LLA to ECEF.  Ensure LLA uses ellipsoid altitude
    ixVector3d lla;
    lla[0] = C_DEG2RAD * gnssPos.lla[0];
    lla[1] = C_DEG2RAD * gnssPos.lla[1];
    lla[2] = gnssPos.lla[2];        // Use ellipsoid altitude
    lla2ecef(lla, gnssPos.ecef);

    // 13 - time since last DGPS update
    // 14 - DGPS station ID number

    return 0;
}

/**
 * @brief Parses a $GxGLL (Geographic Position, Latitude/Longitude) message into a GNSS position solution.
 * @note $GLL reports only a basic 2D fix with no fix-quality detail; a present position sets a generic GNSS_STATUS_FIX_2D bit, and the data-valid field can clear the fix status entirely.
 *
 * @param a[] - incoming NMEA sentence buffer: $xxGLL,lat,NS,lon,EW,time,status,posMode*cs
 * @param aSize - size of a[] (unused)
 * @param gnssPos - [out] updated GNSS position (gnss_pos_t): lla[0..1] (fields 1-4, zeroed if absent), timeOfWeekMs (field 5), status fix bit (field 6, see eGnssStatus GNSS_STATUS_FIX_2D / GNSS_STATUS_FIX_MASK)
 * @param utcTime - [out] parsed UTC time breakdown (field 5)
 * @param utcWeekday - UTC day of week (0=Sunday) used to convert UTC time-of-day to GPS time of week
 *
 * @note parsed message fields:
 *  1,2 Latitude (ddmm.mmmm, N/S) -> gnssPos.lla[0]; zeroed and fix cleared (GNSS_STATUS_FIX_MASK) if field is empty
 *  3,4 Longitude (dddmm.mmmm, E/W) -> gnssPos.lla[1]; sets GNSS_STATUS_FIX_2D when a position is present
 *  5   UTC time (HHMMSS.sss) -> utcTime / gnssPos.timeOfWeekMs
 *  6   Status: A=active/valid data, V=void -> if not 'A', clears gnssPos.status fix bits, timeOfWeekMs, and lla[0..1]
 *  7   Positioning mode (not stored)
 *
 * @return 0 always
 */
int nmea_parse_gll(const char a[], const int aSize, gnss_pos_t &gnssPos, utc_time_t &utcTime, int utcWeekday)
{
    (void)aSize;
    char *ptr = (char *)&a[7];    // $GxGLL,
    
    if (*ptr == ',')
    {   // pos has no value 
        // 1,2 - Latitude (deg)
        gnssPos.lla[0] = 0;
        // 3,4 - Longitude (deg)
        gnssPos.lla[1] = 0;

        // set status to no fix
        gnssPos.status &= ~GNSS_STATUS_FIX_MASK;

        ptr += 4;
    }
    else
    {   // pos has a value
        // 1,2 - Latitude (deg)
        ptr = ASCII_DegMin_to_Lat(&(gnssPos.lla[0]), ptr);
        // 3,4 - Longitude (deg)
        ptr = ASCII_DegMin_to_Lon(&(gnssPos.lla[1]), ptr);
        
        gnssPos.status |= GNSS_STATUS_FIX_2D;
    }

    // 5 - UTC time HHMMSS.sss
    ptr = ASCII_UtcTimeToGpsTowMs(&gnssPos.timeOfWeekMs, &utcTime, ptr, utcWeekday, gnssPos.leapS);
    
    // 6 - Valid (A=active, V=void)
    if (*ptr != 'A')             
    {
        gnssPos.status &= ~GNSS_STATUS_FIX_MASK;
        gnssPos.timeOfWeekMs = 0;
        gnssPos.lla[0] = 0.0;
        gnssPos.lla[1] = 0.0;
    }

    return 0;
}

/**
 * @brief Parses a $GxGSA (GNSS DOP and Active Satellites) message into fix-mode, satellite list, and DOP values.
 * @note Provides pDOP and navigation mode (saved to determine 2D/3D mode). Only field 2's 2D/3D fix mode is applied to gnssPos.status; a "no fix" code (1) intentionally leaves the existing fix status untouched (DO NOTHING) since a lack of GSA fix info shouldn't downgrade a fix reported by a more detailed message like $GGA/$GNS.
 *
 * @param a[] - incoming NMEA sentence buffer: $xxGSA,opMode,navMode{,svid}x12,PDOP,HDOP,VDOP,systemId*cs
 * @param aSize - size of a[] (unused)
 * @param gnssPos - [out] updated GNSS position (gnss_pos_t): status fix-mode bits (field 2, see eGnssStatus GNSS_STATUS_FIX_MASK), pDop (field 15), hAcc (field 16), vAcc (field 17)
 * @param sat - [out] optional satellite list (gnss_sat_t); if non-NULL, receives up to 12 satellite IDs used in the solution (fields 3-14); pass NULL to skip
 *
 * @note parsed message fields:
 *  1     Auto 2D/3D selection mode: M=manual, A=automatic (not stored)
 *  2     Fix mode: 1=fix not available, 2=2D, 3=3D -> gnssPos.status GNSS_STATUS_FIX_2D/GNSS_STATUS_FIX_3D (1 leaves status unchanged)
 *  3-14  IDs of up to 12 satellites used in the position fix -> sat->sat[i].svId (if sat != NULL)
 *  15    Position dilution of precision (PDOP) -> gnssPos.pDop
 *  16    Horizontal dilution of precision (HDOP) -> gnssPos.hAcc
 *  17    Vertical dilution of precision (VDOP) -> gnssPos.vAcc
 *
 * @return 0 always
 */
int nmea_parse_gsa(const char a[], const int aSize, gnss_pos_t &gnssPos, gnss_sat_t *sat)
{
    (void)aSize;
    char *ptr = (char *)&a[7];    // $GxGSA,
    //$xxGSA,opMode,navMode{,svid},PDOP,HDOP,VDOP,systemId*cs<CR><LF>
    
    // 1 - Auto selection of 2D or 3D
    ptr = ASCII_find_next_field(ptr);

    // 2 - Fix quality
    uint32_t fixQuality;
    ptr = ASCII_to_u32(&fixQuality, ptr);
    gnssPos.status &= ~GNSS_STATUS_FIX_MASK;
    switch(fixQuality)
    {
        default:    /* DO NOTHING */                      break;
        case 2:     gnssPos.status |= GNSS_STATUS_FIX_2D; break;
        case 3:     gnssPos.status |= GNSS_STATUS_FIX_3D; break;
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
    ptr = ASCII_to_f32(&(gnssPos.pDop), ptr);

    // 16 - hDop (hAcc)
    ptr = ASCII_to_f32(&(gnssPos.hAcc), ptr);

    // 17 - vDop (vAcc)
    ptr = ASCII_to_f32(&(gnssPos.vAcc), ptr);

    return 0;
}

/**
 * @brief Parses a single $GxGSV (Satellites in View) message, appending its satellites/signals to the running satellite and signal lists.
 * @note Provides satellite information. Multiple GSV messages will come in a block (see fields 1-2); the caller waits until the block is finished (msgNum == numMsgs) before flagging the accumulated data as ready. Also accumulates a running sum/count of non-zero C/N0 values for computing a mean C/N0 across the block.
 *
 * @param a[] - incoming NMEA sentence buffer: $GNGSV,numMsgs,msgNum,numSats,{svid,elv,az,cno}x(up to 4),signalId*cs
 * @param aSize - size of a[] (unused)
 * @param gnssSat - [out] running satellite list (gnss_sat_t); new satellites (by gnssId+svId) are appended up to MAX_NUM_SATELLITES; if NULL, parsing is skipped entirely and a[] is returned unchanged
 * @param gnssSig - [out] running satellite-signal list (gnss_sig_t); new signals are appended up to MAX_NUM_SAT_SIGNALS; if NULL, parsing is skipped entirely and a[] is returned unchanged
 * @param cnoSum - [in/out] running sum of non-zero C/N0 values across the block, used to compute cnoMean
 * @param cnoCount - [in/out] running count of non-zero C/N0 samples added to cnoSum
 *
 * @note parsed message fields:
 *  1        Total number of GSV messages in this group -> numMsgs (local)
 *  2        This message's sequence number within the group -> msgNum (local); combined with field 3 determines how many of the remaining satellites belong to this message (up to 4)
 *  3        Total number of satellite signals in view (across the whole group) -> numSigs (local)
 *  4+4n     Satellite ID (svId) -> decoded via talkerId_to_gnssId() into gnssId/svId/sigId, appended to gnssSat/gnssSig
 *  5+4n     Elevation (degrees) -> gnssSat[].elev
 *  6+4n     Azimuth (degrees) -> gnssSat[].azim
 *  7+4n     C/N0 (dB-Hz), empty if not tracking -> gnssSig[].cno, and accumulated into cnoSum/cnoCount if non-zero
 *           (fields 4-7 repeat for up to 4 satellites per message, n=0..3)
 *  last-1   Signal ID character (NMEA protocol >= 4.10 only, see nmea4p11_signalId_to_sigId()) -> retroactively sets sigId on all signals added by this message
 *  last     Checksum, begins with *
 *
 * @return pointer just past the checksum of the parsed message (start of the next sentence, if any)
 */
char* nmea_parse_gsv(const char a[], const int aSize, gnss_sat_t *gnssSat, gnss_sig_t *gnssSig, uint32_t *cnoSum, uint32_t *cnoCount)
{
    if (gnssSat == NULL || gnssSig == NULL)
    {   // Don't parse
        return (char*)a;
    }

    (void)aSize;
    char *ptr = (char *)&a[7];    // $GNGSV,
    // $GNGSV, numMsgs, msgNum, numSats, {,svid,elv,az,cno}, signalId *checksum <CR><LF>
        
    int32_t numMsgs, msgNum, numSigs;
    ptr = ASCII_to_i32(&numMsgs, ptr);            // 1 - number of messages
    ptr = ASCII_to_i32(&msgNum, ptr);            // 2 - message number
    ptr = ASCII_to_i32(&numSigs, ptr);            // 3 - number of satellite signals in view

    uint8_t gnssId = SAT_SV_GNSS_ID_UNKNOWN;
    uint8_t sigId = 0;
    uint8_t *sigIds[4] = { NULL };
    uint8_t **sigIdPtr = sigIds;
            
    // Process up to 4 satellites
    int satCnt = _MIN(numSigs - (msgNum - 1) * 4, 4);

    // Payload: {svid,elv,az,cno} up to 4x
    for (int i=0; i<satCnt; ++i)
    {
        uint8_t elev, cno;
        uint16_t svId, azim;

        ptr = ASCII_to_u16(&svId, ptr);        // 4 + 4x   svId
        ptr = ASCII_to_u8(&elev, ptr);        // 5 + 4x   elevation
        ptr = ASCII_to_u16(&azim, ptr);        // 6 + 4x   azimuth
        ptr = ASCII_to_u8(&cno, ptr);        // 7 + 4x   cno

        talkerId_to_gnssId(a, gnssId, svId, sigId);
                
        // Add to satellite info list
        for (uint32_t j=0;; j++)
        {
            if (j >= gnssSat->numSats)
            {   // Not in list
                if (gnssSat->numSats < MAX_NUM_SATELLITES)
                {   // Add to list
                    auto& dst = gnssSat->sat[gnssSat->numSats];
                    gnssSat->numSats++;
                    
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
            if ((gnssId == gnssSat->sat[j].gnssId) && (svId == gnssSat->sat[j].svId)) 
            { 
                break; 
            }    // already in list
        }

        // Add to satellite signal list
        for (uint32_t j=0;; j++)
        {
            if (j >= gnssSig->numSigs)
            {   // Not in list
                if (gnssSig->numSigs < MAX_NUM_SAT_SIGNALS)
                {   // Add to list
                    auto& dst = gnssSig->sig[gnssSig->numSigs];
                    gnssSig->numSigs++;
                    
                    dst.gnssId = gnssId;
                    dst.svId = (uint8_t)svId;
                    dst.sigId = sigId;    // Gets set at function end if using protocol > NMEA 4.1
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
        uint8_t sigId = nmea4p11_signalId_to_sigId(gnssId, *ptr);    // nmea signal ID
        ptr = ASCII_find_next_field(ptr);
        for (int i=0; i<4 && sigIds[i]!=NULL; i++)
        {   // Retroactivly set sigId satSig list
            *(sigIds[i]) = sigId;
        }
    }
    
    // Move past checksum
    return ptr + 5;
}

/**
 * @brief Parses a $INTEL proprietary NMEA message into KIM sub-module firmware version, GPS time, and ECEF velocity. See nmea_intel() for the wire field order.
 * @note Fields 12-14 (NED velocity) are present in the wire format but are commented out / not parsed here.
 *
 * @param a[] - incoming NMEA sentence buffer
 * @param aSize - size of a[] (unused)
 * @param info - [out] device info (dev_info_t); only info.firmwareVer[4] (field 2) is populated
 * @param pos - [out] GNSS position (gnss_pos_t): timeOfWeekMs (field 3, converted from seconds to ms), week (field 4), leapS (field 5)
 * @param vel - [out] GNSS velocity (gnss_vel_t): vel.vel[3] ECEF velocity (fields 9-11)
 * @param ppsPhase[2] - [out] 1PPS phase 1 and phase 2 (fields 6-7); currently always parsed as 0.0 (not implemented upstream)
 * @param ppsNoiseNs[1] - [out] quantization error of the time pulse, in ns (field 8); currently always parsed as 0
 *
 * @note parsed message fields:
 *  1     Message ID KIM (skipped, not parsed)
 *  2     KIM firmware version (major.minor.rev.build) -> info.firmwareVer
 *  3     GPS time of week (seconds) -> pos.timeOfWeekMs (converted to ms)
 *  4     GPS week number -> pos.week
 *  5     GPS leap seconds -> pos.leapS
 *  6     1PPS phase 1 (ns) -> ppsPhase[0]
 *  7     1PPS phase 2 (ns) -> ppsPhase[1]
 *  8     Quantization error of time pulse (ns) -> ppsNoiseNs[0]
 *  9-11  ECEF X/Y/Z velocity (m/s) -> vel.vel[0..2]
 *  12-14 North/East/Down velocity (m/s) (not currently parsed)
 *
 * @return 0 always
 */
int nmea_parse_intel(const char a[], const int aSize, dev_info_t &info, gnss_pos_t &pos, gnss_vel_t &vel, float ppsPhase[2], uint32_t ppsNoiseNs[1])
{
    (void)aSize;
    char *ptr = (char *)&a[7];    // $INTEL,
    
    // 1 - Message ID KIM
    ptr = ASCII_find_next_field(ptr);

    // 2 -    Fimrware version of KIM
    ptr = ASCII_to_ver4u8(info.firmwareVer, ptr);
    
    // 3 -    GPS Time of Week (seconds)
    uint32_t timeSec;
    ptr = ASCII_to_u32(&timeSec, ptr);
    pos.timeOfWeekMs = 1000*timeSec;

    // 4 -    GPS week number
    ptr = ASCII_to_u32(&(pos.week), ptr);
    
    // 5 -    GPS leap seconds
    ptr = ASCII_to_u8(&(pos.leapS), ptr);
    
    // 6 -    1PPS phase 1 (ns)
    ptr = ASCII_to_f32(&(ppsPhase[0]), ptr);
    
    // 7 -    1PPS phase 2 (ns)
    ptr = ASCII_to_f32(&(ppsPhase[1]), ptr);
    
    // 8 -    Quantization error of time pulse (ns)
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
int nmea_parse_powgps(const char a[], const int aSize, gnss_pos_t &pos)
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
    char *ptr = (char *)&a[8];  // $POWGPS,
    uint32_t timeValid;
    uint32_t lsValid;
    
    // 1 - GPS Time valid
    ptr = ASCII_to_u32(&timeValid, ptr);

    // 2 - GPS week number
    ptr = ASCII_to_u32(&(pos.week), ptr);

    // 3 - GPS Time of Week (us)
    ptr = ASCII_to_u64(&TOWus, ptr);
    pos.timeOfWeekMs = TOWus/1000;
    
    // 4 - GPS leap seconds valid
    ptr = ASCII_to_u32(&lsValid, ptr);
    
    // 5 - GPS leap seconds
    ptr = ASCII_to_u8(&(pos.leapS), ptr);

    // 6 - Holdover flag (0=no holdover, 1=EGR is in holdover)

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
int nmea_parse_powtlv(const char a[], const int aSize, gnss_pos_t &pos, gnss_vel_t &vel)
{
    (void)aSize;
    uint64_t TOWus;
    char *ptr = (char *)&a[8];    // $POWGPS,
    uint32_t temp;
    float horVel, courseMadeTrue;
    
    // 1 -  GPS Time valid
    ptr = ASCII_to_u32(&temp, ptr);

    // 2 -  GPS week number
    ptr = ASCII_to_u32(&(pos.week), ptr);

    // 3 -  GPS Time of Week (us)
    ptr = ASCII_to_u64(&TOWus, ptr);
    pos.timeOfWeekMs = TOWus/1000;  // convert to seconds

    // if time is not valid, set time to 0
    if (temp == 0) { pos.timeOfWeekMs = 0; pos.week = 0; }
    
    // 4 -  GPS leap seconds valid
    ptr = ASCII_to_u32(&temp, ptr);
    
    // 5 -  GPS leap seconds
    ptr = ASCII_to_u8(&(pos.leapS), ptr);

    // if LS not valid, set to 0
    if (temp == 0) { pos.leapS = 0; }

    // 6 -  Holdover flag (0=no holdover, 1=EGR is in holdover)
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

/**
 * @brief Parses a $GxRMC (Recommended Minimum Navigation Information) message into NED ground velocity.
 * @note Provides speed (speed and course over ground). Only speed/course-derived NED velocity is extracted; date/status/mode fields relevant to position are handled by other parsers ($GGA/$GNS/$GLL).
 *
 * @param a[] - incoming NMEA sentence buffer: $xxRMC,time,status,lat,NS,lon,EW,spd,cog,date,mv,mvEW,posMode,navStatus*cs
 * @param aSize - size of a[] (unused)
 * @param gnssVel - [out] parsed GNSS velocity (gnss_vel_t): timeOfWeekMs (field 1), vel[3] NED velocity (fields 7-8, vel[2]=0), status (GNSS_STATUS_FLAGS_GNSS_NMEA_DATA | statusFlags)
 * @param utcTime - [out] parsed UTC time breakdown (field 1)
 * @param utcWeekday - UTC day of week (0=Sunday) used to convert UTC time-of-day to GPS time of week
 * @param leapS - current GPS-UTC leap second offset, used in the UTC-to-GPS-time conversion
 * @param statusFlags - additional eGnssStatus flag bits (see data_sets.h) OR'd into gnssVel.status alongside GNSS_STATUS_FLAGS_GNSS_NMEA_DATA
 *
 * @note parsed message fields:
 *  1      UTC time (HHMMSS.sss) -> utcTime / gnssVel.timeOfWeekMs
 *  2      Status: A=active, V=void (not stored; skipped)
 *  3,4    Latitude, N/S (not stored; skipped)
 *  5,6    Longitude, E/W (not stored; skipped)
 *  7      Speed over ground (knots) -> combined with field 8 into gnssVel.vel[0..1] (NED)
 *  8      Course over ground (degrees true) -> combined with field 7 into gnssVel.vel[0..1] (NED)
 *  9      Date (ddmmyy) (not stored)
 *  10     Magnetic variation (not stored)
 *  11     Magnetic variation E/W (not stored)
 *  12     Positioning mode (not stored)
 *  13     Navigational status (not stored)
 *
 * @return 0 always
 */
int nmea_parse_rmc(const char a[], int aSize, gnss_vel_t &gnssVel, utc_time_t &utcTime, int utcWeekday, int leapS, uint32_t statusFlags)
{
    (void)aSize;
    char *ptr = (char *)&a[7];
    //$xxRMC,time,status,lat,NS,lon,EW,spd,cog,date,mv,mvEW,posMode,navStatus*cs<CR><LF>

    // 1 - UTC time HHMMSS.sss
    ptr = ASCII_UtcTimeToGpsTowMs(&gnssVel.timeOfWeekMs, &utcTime, ptr, utcWeekday, leapS);

    //Skip 5
    for (int i=0;i<5;++i)
    {
        ptr = ASCII_find_next_field(ptr);
    }

    //spd & cog
    float spdm_s = strtof(ptr, NULL) * C_KNOTS_METERS_F;
    ptr = ASCII_find_next_field(ptr);
    float cogRad = C_DEG2RAD_F * strtof(ptr, NULL);

    //Speed data in NED
    gnssVel.vel[0] = spdm_s * cosf(cogRad);
    gnssVel.vel[1] = spdm_s * sinf(cogRad);
    gnssVel.vel[2] = 0;
    //dependencies_.gnssVel.sAcc = 0;
            
    //Indicate it is coming from NMEA
    gnssVel.status = GNSS_STATUS_FLAGS_GNSS_NMEA_DATA | statusFlags;

    return 0;    
}

/**
 * @brief Parses a $GxVTG (Course Over Ground and Ground Speed) message into GNSS velocity, converting NED speed/course into either NED or ECEF velocity depending on vel.status.
 *
 * @param a[] - incoming NMEA sentence buffer
 * @param aSize - size of a[] (unused)
 * @param vel - [in/out] GNSS velocity (gnss_vel_t): vel[3] updated from fields 1 and 5 (course + speed); if vel.status's GNSS_STATUS_FLAGS_GNSS_NMEA_DATA bit is set the result is stored directly as NED, otherwise it is rotated into ECEF using refLla
 * @param refLla[3] - reference latitude/longitude (radians) used for the NED->ECEF rotation when vel is not already NED
 *
 * @note parsed message fields:
 *  1   Track made good (degrees true) -> combined with field 5 to compute NED velocity
 *  2   "T" - track is relative to True North (not stored; skipped)
 *  3   Track made good (degrees magnetic) (not stored)
 *  4   "M" - track is relative to magnetic north (not stored; skipped)
 *  5   Speed over ground (knots) -> combined with field 1 to compute NED velocity
 *  6   "N" - speed is measured in knots (not stored; skipped)
 *  7   Speed over ground (km/h) (not stored)
 *  8   "K" - speed is measured in km/h (not stored; skipped)
 *  9   Mode indicator: A=autonomous, D=differential, E=estimated/dead reckoning, M=manual, S=simulator, N=not valid (not stored; skipped)
 *
 * @return 0 always
 */
int nmea_parse_vtg(const char a[], int aSize, gnss_vel_t &vel, const double refLla[3])
{
    (void)aSize;
    char *ptr = (char *)&a[7];    // $GxVTG,

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
    if (vel.status & GNSS_STATUS_FLAGS_GNSS_NMEA_DATA)
    {   // NED velocity
        cpy_Vec3_Vec3(vel.vel, velNed);
    }
    else
    {   // ECEF velocity
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
    //         A: Autonomous mode
    //         D: Differential mode
    //         E: Estimated (dead reckoning) mode
    //         M: Manual Input mode
    //         S: Simulator mode
    //         N: Data not valid
    ptr = ASCII_find_next_field(ptr);

    return 0;
}

/**
 * @brief Parses a $GxZDA (Time and Date) message into GPS time-of-week/week and a UTC calendar date breakdown.
 *
 * @param a[] - incoming NMEA sentence buffer
 * @param aSize - size of a[] (unused)
 * @param gpsTowMs - [out] GPS time of week, in milliseconds, converted from the parsed UTC date/time
 * @param gpsWeek - [out] GPS week number, converted from the parsed UTC date/time
 * @param date - [out] parsed UTC calendar date (utc_date_t): day/month/year (fields 2-4) plus derived weekday
 * @param time - [out] parsed UTC time (utc_time_t): hour/minute/second/millisecond (field 1)
 * @param leapS - current GPS-UTC leap second offset, used in the UTC-to-GPS-time conversion
 *
 * @note parsed message fields:
 *  1   UTC time (HHMMSS) -> time.hour/minute/second/millisecond
 *  2   Day -> date.day
 *  3   Month -> date.month
 *  4   Year -> date.year
 *  5   Local zone hours offset from GMT (fixed at 00; not stored)
 *  6   Local zone minutes offset from GMT (fixed at 00; not stored)
 *
 * @return 0 always
 */
int nmea_parse_zda(const char a[], int aSize, uint32_t &gpsTowMs, uint32_t &gpsWeek, utc_date_t &date, utc_time_t &time, int leapS)
{
    (void)aSize;
    char *ptr = (char *)&a[7];    // $GxZDA,

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
    int datetime[7] = { date.year, date.month, date.day, time.hour, time.minute, time.second, time.millisecond };        // year,month,day,hour,min,sec,msec
    UtcDateTimeToGpsTime(datetime, leapS, gpsTowMs, gpsWeek);
    date.weekday = gpsTowMsToUtcWeekday(gpsTowMs, leapS);

    // 5,6 - Local time zone offset from GMT (00,00)
    return 0;
}

/**
 * @brief Clears the GSV per-constellation frequency-band filter mask (s_gsvMask), disabling all $GxGSV output until re-enabled via nmea_setGsvFilter() or a $ASCE GSV request.
 */
void gsv_clear_const_mask()
{
    memset(&s_gsvMask, 0, sizeof(gsvMask_t));
}

/**
 * @brief Converts an NMEA message ID (eNmeaMsgIdInx) to its talker string (the message name following '$', e.g. "GNGGA", "PIMU").
 *
 * @param msgId - NMEA message ID (see eNmeaMsgIdInx)
 *
 * Returns the talker string on success, or an an empty string on error
*/
std::string nmeaMsgIdToTalker(int msgId)
{
    switch(msgId)
    {
        case NMEA_MSG_ID_PIMU:  return "PIMU";
        case NMEA_MSG_ID_PPIMU: return "PPIMU";
        case NMEA_MSG_ID_PRIMU: return "PRIMU";
        case NMEA_MSG_ID_PINS1: return "PINS1";
        case NMEA_MSG_ID_PINS2: return "PINS2";
        case NMEA_MSG_ID_PGPSP: return "PGPSP";
        case NMEA_MSG_ID_GNGGA: return "GNGGA";
        case NMEA_MSG_ID_GNGLL: return "GNGLL";
        case NMEA_MSG_ID_GNGSA: return "GNGSA";
        case NMEA_MSG_ID_GNRMC: return "GNRMC";
        case NMEA_MSG_ID_GNZDA: return "GNZDA";
        case NMEA_MSG_ID_PASHR: return "PASHR";
        case NMEA_MSG_ID_PSTRB: return "PSTRB";
        case NMEA_MSG_ID_INFO:  return "INFO";
        case NMEA_MSG_ID_GNGSV: return "GNGSV";
        case NMEA_MSG_ID_GNVTG: return "GNVTG";
        case NMEA_MSG_ID_INTEL: return "INTEL";
        case NMEA_MSG_ID_ASCE:  return "ASCE";
        case NMEA_MSG_ID_BLEN:  return "BLEN";
        case NMEA_MSG_ID_EBLE:  return "EBLE";
        case NMEA_MSG_ID_NELB:  return "NELB";
        case NMEA_MSG_ID_PERS:  return "PERS";
        case NMEA_MSG_ID_SRST:  return "SRST";
        case NMEA_MSG_ID_STPB:  return "STPB";
        case NMEA_MSG_ID_STPC:  return "STPC";
    }

    return "";
}

