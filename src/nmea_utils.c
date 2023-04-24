/**
 * @file protocol_nmea.c
 * @author Dave Cutting (davidcutting42@gmail.com)
 * @brief Inertial Sense
 * @date 2023-04-23
 *
 * @copyright Copyright (C) 2023 Inertial Sense, Inc.
 *
 */

#include <stdint.h>

#include "nmea_utils.h"

int8_t nmea_scan(char **ptr)
{
    if (ptr == NULL) // NULL pointer
        return NMEAUTILS_ERR_ANY;

    char *tmp = *ptr;

    if (tmp == NULL) // NULL pointer
        return NMEAUTILS_ERR_ANY;

    // Use a for loop to enforce max length of NMEA
    for (uint8_t idx = 0; idx < NMEAUTILS_MAX_LEN; idx++)
    {
        if (*tmp == ',')
        { // Reached next field, jump over comma and return
            *ptr = tmp + 1;
            return NMEAUTILS_SUCCESS;
        }
        else if (*tmp == '*')
        { // Found checksum delimiter, point to checksum data and return
            *ptr = tmp + 1;
            return NMEAUTILS_FOUND_CHKSUM;
        }
        else if ((*tmp & 0x80U) || (*tmp < 32))
        { // Invalid NMEA char (non-printable ASCII), error
            break;
        }

        tmp++;
    }

    return NMEAUTILS_ERR_ANY;
}

int8_t nmea_checksum(char *str, uint8_t *sum)
{
    char *tmp;

    if (str == NULL || sum == NULL) // NULL pointer check
        return NMEAUTILS_ERR_ANY;
    if (*str != '$') // Invalid message ('!' unsupported)
        return NMEAUTILS_ERR_START;

    *sum = 0U;
    tmp = str + 1U;

    // Use a for loop to enforce max length of NMEA
    for (uint8_t idx = 0; idx < NMEAUTILS_MAX_LEN - 5; idx++)
    {
        if (*tmp == '*')
        { // Reached checksum delimiter
            return NMEAUTILS_SUCCESS;
        }
        else if ((*tmp & 0x80U) || (*tmp < 32))
        { // Invalid NMEA char (non-printable ASCII) or \r\n, error
            break;
        }

        *sum ^= *tmp;
        tmp++;
    }

    return NMEAUTILS_ERR_ANY;
}

int8_t nmea_check_header(char *str, const char *match)
{
    if (str == NULL || str[0] != '$')
        return NMEAUTILS_ERR_START;

    for (size_t i = 0; i < NMEAUTILS_MAX_HDRLEN; i++)
    {
        if (match[i] == '\0' && str[i] == ',')
            return NMEAUTILS_SUCCESS;
        if (match[i] == '~')
            continue; // wildcard char
        if (str[i] != match[i])
            return NMEAUTILS_ERR_NOMATCH;
        if ((str[i] & 0x80U) || (str[i] < 32))
            break; // non-printable char
    }

    return NMEAUTILS_ERR_END;
}

int8_t nmea_get_u8(char **ptr, uint8_t *val)
{
    int8_t err = 0;
    uint32_t tmp = strtoul(*ptr, ptr, 10); // `ptr` is updated in strtol, usually ',' or '*'

    if (errno == ERANGE || tmp & 0xFFFFFF00)
        err = 1;
    else // Good conversion
        *val = (uint8_t)tmp;
    int8_t ret = nmea_scan(ptr); // Do this before returning so we have a valid ptr setting
    if (err)
        return NMEAUTILS_ERR_CONVERSION;
    return ret;
}

int8_t nmea_get_u16(char **ptr, uint16_t *val)
{
    int8_t err = 0;
    uint32_t tmp = strtoul(*ptr, ptr, 10); // `ptr` is updated in strtol, usually ',' or '*'

    if (errno == ERANGE || tmp & 0xFFFF0000)
        err = 1;
    else // Good conversion
        *val = (uint16_t)tmp;
    int8_t ret = nmea_scan(ptr); // Do this before returning so we have a valid ptr setting
    if (err)
        return NMEAUTILS_ERR_CONVERSION;
    return ret;
}

int8_t nmea_get_u32(char **ptr, uint32_t *val)
{
    int8_t err = 0;
    uint32_t tmp = strtoul(*ptr, ptr, 10); // `ptr` is updated in strtol, usually ',' or '*'

    if (errno == ERANGE)
        err = errno;
    else // Good conversion
        *val = (uint32_t)tmp;
    int8_t ret = nmea_scan(ptr); // Do this before returning so we have a valid ptr setting
    if (err)
        return NMEAUTILS_ERR_CONVERSION;
    return ret;
}

int8_t nmea_get_s32(char **ptr, int32_t *val)
{
    int8_t err = 0;
    int32_t tmp = strtol(*ptr, ptr, 10); // `ptr` is updated in strtol, usually ',' or '*'

    if (errno == ERANGE)
        err = errno;
    else // Good conversion
        *val = (int32_t)tmp;
    int8_t ret = nmea_scan(ptr); // Do this before returning so we have a valid ptr setting
    if (err)
        return NMEAUTILS_ERR_CONVERSION;
    return ret;
}

int8_t nmea_get_f32(char **ptr, float *val)
{
    int8_t err = 0;
    float tmp = strtof(*ptr, ptr); // `ptr` is updated in strtol, usually ',' or '*'

    if (errno == ERANGE)
        err = errno;
    else // Good conversion
        *val = tmp;
    int8_t ret = nmea_scan(ptr); // Do this before returning so we have a valid ptr setting
    if (err)
        return NMEAUTILS_ERR_CONVERSION;
    return ret;
}

int8_t nmea_get_f64(char **ptr, double *val)
{
    int8_t err = 0;
    double tmp = strtod(*ptr, ptr); // `ptr` is updated in strtol, usually ',' or '*'

    if (errno == ERANGE)
        err = errno;
    else // Good conversion
        *val = tmp;
    int8_t ret = nmea_scan(ptr); // Do this before returning so we have a valid ptr setting
    if (err)
        return NMEAUTILS_ERR_CONVERSION;
    return ret;
}

int8_t nmea_get_vf32(char **ptr, float *vec, uint8_t n)
{
    // ptr checked by nmea_get_f32
    if (vec == NULL)
        return NMEAUTILS_ERR_ANY;

    for (size_t i = 0; i < n; i++)
    {
        int8_t ret = nmea_get_f32(ptr, &vec[i]);

        if (ret < NMEAUTILS_SUCCESS)
            return ret;
    }
}

int8_t nmea_get_vf64(char **ptr, double *vec, uint8_t n)
{
    // ptr checked by nmea_get_f64
    if (vec == NULL)
        return NMEAUTILS_ERR_ANY;

    for (size_t i = 0; i < n; i++)
    {
        int8_t ret = nmea_get_f64(ptr, &vec[i]);

        if (ret < NMEAUTILS_SUCCESS)
            return ret;
    }
}

int8_t nmea_get_latlon(char **ptr, double *latlon)
{
    int8_t ret;
    double tmp;

    for (uint8_t i = 0; i < 2; i++)
    {
        // Get the raw doubles
        ret = nmea_get_f64(ptr, &tmp);
        if (ret < NMEAUTILS_SUCCESS)
            return ret;

#define RECIP_60_D (0.01666666666) // (1.0/60.0) to speed up double using multiplication
        double deg = (int)tmp / 100;
        tmp -= deg * 100;
        tmp = deg + (tmp * RECIP_60_D); // Divide by 60

        // Parse compass direction
        if (**ptr == 'N' || **ptr == 'E')
            latlon[i] = tmp;
        else if (**ptr == 'S' || **ptr == 'W')
            latlon[i] = -tmp;
        else
            return NMEAUTILS_ERR_ANY;

        ret = nmea_scan(ptr);
        if (ret < NMEAUTILS_SUCCESS)
            return ret;
        else if (i == 0 && ret == NMEAUTILS_FOUND_CHKSUM) // premature end
            return NMEAUTILS_ERR_END;
    }

    return NMEAUTILS_SUCCESS;
}

int8_t nmea_get_strn(const char **ptr, char *dest, uint8_t len)
{
    const char *tmp = *ptr;

    for (size_t i = 0; i < len; i++)
    {
        if (tmp[i] == ',' || tmp[i] == '*')
        { // Found comma or checksum, so fill zeroes in rest of data
            memset(tmp + i, '\0', len - i);
            break;
        }
        if ((tmp[i] & 0x80U) || (tmp[i] < 32))
            return NMEAUTILS_ERR_ANY;

        dest[i] = tmp[i];
    }

    // Go to next
    return nmea_scan(ptr);
}
