#ifndef NMEAUTILS_H_
#define NMEAUTILS_H_

#include "data_sets.h"

#ifdef __cplusplus
extern "C" {
#endif

#define NMEAUTILS_MAX_LEN 82
#define NMEAUTILS_MAX_HDRLEN 10

enum NmeaUtilsStatus
{
    NMEAUTILS_FOUND_CHKSUM = 1, // Reached checksum of message
    NMEAUTILS_SUCCESS = 0,
    NMEAUTILS_ERR_ANY = -1, // Uncategorized error
    NMEAUTILS_ERR_START = -2, // Bad start char
    NMEAUTILS_ERR_END = -3, // Premature end
    NMEAUTILS_ERR_CONVERSION = -4, // Bad conversion
    NMEAUTILS_ERR_NOMATCH = -5, // Something didn't match
};

/**
 * @brief Move pointer to next field to parse
 *
 * @param ptr Pointer to pointer of current field being parsed (updated before return)
 * @return int8_t from NmeaUtilsStatus
 */
int8_t nmea_scan(char **ptr);

/**
 * @brief Compute checksum for data
 * @note only '$' messsages are supported now, '!' (encapsulated) messages will throw error
 *
 * @param str Pointer to '$' in sentence
 * @param sum Output, computed checksum
 * @return int8_t from NmeaUtilsStatus
 *  NMEAUTILS_ERR_START: str[0] != '$'
 */
int8_t nmea_checksum(char *str, uint8_t *sum);

/**
 * @brief Check NMEA header
 * @note max message ID length is NMEAUTILS_MAX_HDRLEN
 * 
 * @param str input message, starting with '$' at str[0] and ending with last char before final header semicolon
 * @param match message name to match against, with $ or ! as match[0], don't care chars as tilde ('~'), 0 instead of final semicolon
 * @return int8_t from NmeaUtilsStatus
 *  NMEAUTILS_ERR_NOMATCH: no match
 *  NMEAUTILS_ERR_END: max header check length exceeded
 */
int8_t nmea_check_header(char *str, const char* match);

/**
 * @brief Get uint8_t
 *
 * @param ptr first char of value, updated after read to next comma-delimited value (if available)
 * @param val pointer to destination of data
 * @return int8_t from NmeaUtilsStatus
 *  NMEAUTILS_END: moved pointer to start of checksum
 */
int8_t nmea_get_u8(char **ptr, uint8_t *val);

/**
 * @brief Get uint16_t
 *
 * @param ptr first char of value, updated after read to next comma-delimited value (if available)
 * @param val pointer to destination of data
 * @return int8_t from NmeaUtilsStatus
 *  NMEAUTILS_END: moved pointer to start of checksum
 */
int8_t nmea_get_u16(char **ptr, uint16_t *val);

/**
 * @brief Get uint32_t
 *
 * @param ptr first char of value, updated after read to next comma-delimited value (if available)
 * @param val pointer to destination of data
 * @return int8_t from NmeaUtilsStatus
 *  NMEAUTILS_END: moved pointer to start of checksum
 */
int8_t nmea_get_u32(char **ptr, uint32_t *val);

/**
 * @brief Get int32_t
 *
 * @param ptr first char of value, updated after read to next comma-delimited value (if available)
 * @param val pointer to destination of data
 * @return int8_t from NmeaUtilsStatus
 *  NMEAUTILS_END: moved pointer to start of checksum
 */
int8_t nmea_get_s32(char **ptr, int32_t *val);

/**
 * @brief Get int32_t
 *
 * @param ptr first char of value, updated after read to next comma-delimited value (if available)
 * @param val pointer to destination of data
 * @return int8_t from NmeaUtilsStatus
 *  NMEAUTILS_END: moved pointer to start of checksum
 */
int8_t nmea_get_f32(char **ptr, float *val);

/**
 * @brief Get int32_t
 *
 * @param ptr first char of value, updated after read to next comma-delimited value (if available)
 * @param val pointer to destination of data
 * @return int8_t from NmeaUtilsStatus
 *  NMEAUTILS_END: moved pointer to start of checksum
 */
int8_t nmea_get_f64(char **ptr, double *val);

/**
 * @brief Get vector of comma-delimited floats
 *
 * @param ptr first char of value, updated after read to next comma-delimited value (if available)
 * @param val pointer to destination of data
 * @param n number of floats to parse
 * @return int8_t from NmeaUtilsStatus
 *  NMEAUTILS_END: moved pointer to start of checksum
 */
int8_t nmea_get_vf32(char **ptr, float *vec, uint8_t n);

/**
 * @brief Get vector of comma-delimited doubles
 *
 * @param ptr first char of value, updated after read to next comma-delimited value (if available)
 * @param val pointer to destination of data
 * @param n number of floats to parse
 * @return int8_t from NmeaUtilsStatus
 *  NMEAUTILS_END: moved pointer to start of checksum
 */
int8_t nmea_get_vf64(char **ptr, double *vec, uint8_t n);

/**
 * @brief Get latitude and longitude from ddmm.mm,a,dddmm.mm,a format (a is N/E/S/W)
 *
 * @param ptr first char of value, updated after read to next comma-delimited value (if available)
 * @param val pointer to destination of data
 * @return int8_t from NmeaUtilsStatus
 */
int8_t nmea_get_latlon(char **ptr, double *latlon);

/**
 * @brief Get string between delimiters, with a max length
 * @note Anything beyond max length is discarded and parsing jumps to next data
 * @note If ptr doesn't have enough chars, zeroes are filled in the rest of the data
 * 
 * @param ptr first char of value, updated after read to next comma-delimited value (if available)
 * @param dest destination for chars read
 * @param len maximum number of chars to copy before jump
 * @return int8_t from NmeaUtilsStatus
 */
int8_t nmea_get_strn(const char **ptr, char *dest, uint8_t len);

#ifdef __cplusplus
}
#endif

#endif // NMEAUTILS_H_
