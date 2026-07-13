/**
 * @file protocol_nmea.h
 * @brief NMEA 0183 encode (binary->NMEA) and decode (NMEA->binary) functions for the IS SDK.
 *
 * Provides two families of functions:
 *  - Encode: serialize Inertial Sense data structures into NMEA sentences
 *    (standard GGA/RMC/VTG/GSV and proprietary PINS1/PINS2/PIMU/PGPSP/PASHR, etc.).
 *  - Decode/Parse: deserialize NMEA sentences back into Inertial Sense data structures.
 *
 * All encode functions share the signature convention
 * @code int nmea_xxx(char a[], const int aSize, <data args>...); @endcode
 * where @p a is the output buffer, @p aSize is its capacity, and the return value is the
 * number of bytes written (negative on failure).
 *
 * @author Inertial Sense, Inc.
 * @copyright Copyright (c) 2014-2025 Inertial Sense, Inc. - http://inertialsense.com
 */

#ifndef PROTOCOL_NMEA_H_
#define PROTOCOL_NMEA_H_

#include <vector>
#include <string>

#include "data_sets.h"
#include "time_conversion.h"

#if !defined(GPX_1) && !defined(IS_IMX) && !defined(NAV_POST_PROCESS)
extern uint32_t g_cpu_msec;
#endif

/** Pre-formatted NMEA command: query device info ($INFO). */
#define NMEA_CMD_QUERY_DEVICE_INFO                      "$INFO*0E\r\n"
/** Pre-formatted NMEA command: stop all broadcasts on all ports ($STPB). */
#define NMEA_CMD_STOP_ALL_BROADCASTS_ALL_PORTS          "$STPB*15\r\n"
/** Pre-formatted NMEA command: stop all broadcasts on current port ($STPC). */
#define NMEA_CMD_STOP_ALL_BROADCASTS_CUR_PORT           "$STPC*14\r\n"
/** Pre-formatted NMEA command: save persistent messages to flash ($PERS). */
#define NMEA_CMD_SAVE_PERSISTENT_MESSAGES_TO_FLASH      "$PERS*14\r\n"
/** Pre-formatted NMEA command: software reset ($SRST). */
#define NMEA_CMD_SOFTWARE_RESET                         "$SRST*06\r\n"
/** Byte length of the fixed-size NMEA command strings above, including CR+LF. */
#define NMEA_CMD_SIZE                                   10

/** NMEA protocol version selector used with nmea_set_protocol_version(). */
enum eNmeaProtocolVersion
{
    NMEA_PROTOCOL_2P3   = 230,  //!< NMEA 0183 v2.3 (SDK default)
    NMEA_PROTOCOL_4P10  = 410,  //!< NMEA 0183 v4.10
};

//////////////////////////////////////////////////////////////////////////
// Utility functions
//////////////////////////////////////////////////////////////////////////

/**
 * @brief Set the GSV satellite visibility filter for a single constellation.
 * @param constellation Constellation identifier (e.g., NMEA GSV talker index).
 * @param filter        Filter bitmask to apply for that constellation.
 */
void nmea_setGsvFilter(int constellation, uint8_t filter);

/**
 * @brief Set GSV satellite visibility filters for all constellations at once.
 * @param filters Pointer to an array of filter bytes, one per constellation.
 */
void nmea_setGsvFilter(uint8_t* filters);

/**
 * @brief Get the current GSV satellite visibility filter for a single constellation.
 * @param constellation Constellation identifier.
 * @return Current filter bitmask for that constellation.
 */
int nmea_getGsvFilter(int constellation);

/**
 * @brief Get the GSV satellite visibility filters for all constellations.
 * @param filters Output array; one byte per constellation is written.
 */
void nmea_getGsvFilter(uint8_t* filters);

/**
 * @brief Enable NMEA streaming for a specific message ID at a given period.
 * @param bits          RMC bits word to update (bit corresponding to @p nmeaId is set).
 * @param period        Period array; entry at @p nmeaId is set to @p periodMultiple.
 * @param nmeaId        NMEA message identifier (index into the period array).
 * @param periodMultiple Broadcast period multiplier (0 = disabled, 1 = full rate, etc.).
 */
void nmea_enable_stream(uint32_t& bits, uint8_t* period, uint32_t nmeaId, uint8_t periodMultiple);

/**
 * @brief Set the active NMEA protocol version for all subsequent encode operations.
 * @param protocol_version Version code (see @ref eNmeaProtocolVersion).
 */
void nmea_set_protocol_version(int protocol_version);

/**
 * @brief Set the GNSS constellation ID used when formatting NMEA talker IDs.
 * @param gnssId GNSS constellation identifier.
 */
void nmea_set_gnss_id(int gnssId);

/**
 * @brief Compute the NMEA 0183 XOR checksum over a sentence body.
 * @param str  Pointer to the sentence bytes (must not include leading '$').
 * @param size Number of bytes to checksum.
 * @return 8-bit XOR checksum value.
 */
uint32_t nmea_compute_checksum(uint8_t* str, int size);

/**
 * @brief Append a printf-style formatted field to a growing NMEA sentence buffer.
 * @param buf     Output buffer.
 * @param bufSize Capacity of @p buf in bytes.
 * @param offset  Current write offset; updated on return.
 * @param fmt     printf-style format string.
 * @param ...     Format arguments.
 */
void nmea_sprint(char buf[], int bufSize, int &offset, const char *fmt, ...);

/**
 * @brief Wrapper around nmea_sprint() that promotes a float argument to double.
 * @param buf     Output buffer.
 * @param bufSize Capacity of @p buf in bytes.
 * @param offset  Current write offset; updated on return.
 * @param fmt     printf-style format string (single floating-point field).
 * @param v       Float value to format (promoted to double internally).
 */
inline void nmea_sprint_f(char* buf, int bufSize, int& offset, const char* fmt, float v)
{
    nmea_sprint(buf, bufSize, offset, fmt, (double)v);
}

/**
 * @brief Append the NMEA sentence footer (checksum + CRLF) to the buffer.
 * @param a     Output buffer containing the sentence body written so far.
 * @param aSize Capacity of @p a in bytes.
 * @param n     Current write offset; updated to the new end position on return.
 * @return Number of bytes written on success, or a negative value on failure.
 */
int nmea_sprint_footer(char* a, int aSize, int &n);

/**
 * @brief Advance a pointer to the next comma-delimited field in an NMEA sentence.
 * @param str Pointer to the current position in the sentence.
 * @return Pointer to the first character of the next field, or NULL at end of sentence.
 */
char *ASCII_find_next_field(char *str);

/**
 * @brief Parse an ASCII decimal string into a uint8_t and advance the input pointer.
 * @param val Output: parsed value.
 * @param ptr Input: pointer to the ASCII string; advanced past the parsed field.
 * @return Updated pointer (same as @p ptr after advancing).
 */
char *ASCII_to_u8(uint8_t *val, char *ptr);

/**
 * @brief Parse an ASCII decimal string into a uint16_t and advance the input pointer.
 * @param val Output: parsed value.
 * @param ptr Input: pointer to the ASCII string; advanced past the parsed field.
 * @return Updated pointer (same as @p ptr after advancing).
 */
char *ASCII_to_u16(uint16_t *val, char *ptr);

/**
 * @brief Parse an ASCII decimal string into a uint32_t and advance the input pointer.
 * @param val Output: parsed value.
 * @param ptr Input: pointer to the ASCII string; advanced past the parsed field.
 * @return Updated pointer (same as @p ptr after advancing).
 */
char *ASCII_to_u32(uint32_t *val, char *ptr);

/**
 * @brief Parse an ASCII decimal string into a signed int32_t and advance the input pointer.
 * @param val Output: parsed value.
 * @param ptr Input: pointer to the ASCII string; advanced past the parsed field.
 * @return Updated pointer (same as @p ptr after advancing).
 */
char *ASCII_to_i32(int32_t *val, char *ptr);

/**
 * @brief Parse three consecutive ASCII float fields into a float[3] vector.
 * @param vec Output: array of 3 floats filled from successive comma-separated fields.
 * @param ptr Input: pointer to the first field; advanced past the three parsed fields.
 * @return Updated pointer.
 */
char *ASCII_to_vec3f(float vec[], char *ptr);

/**
 * @brief Parse four consecutive ASCII float fields into a float[4] vector.
 * @param vec Output: array of 4 floats filled from successive comma-separated fields.
 * @param ptr Input: pointer to the first field; advanced past the four parsed fields.
 * @return Updated pointer.
 */
char *ASCII_to_vec4f(float vec[], char *ptr);

/**
 * @brief Parse three consecutive ASCII double fields into a double[3] vector.
 * @param vec Output: array of 3 doubles filled from successive comma-separated fields.
 * @param ptr Input: pointer to the first field; advanced past the three parsed fields.
 * @return Updated pointer.
 */
char *ASCII_to_vec3d(double vec[], char *ptr);

/**
 * @brief Convert NMEA DDMM.mmm coordinate format to decimal degrees.
 * @param ddmm Value in DDMM.mmmm format (e.g., 4730.5000 = 47 deg 30.5 min).
 * @return Equivalent decimal degrees.
 */
double ddmm2deg(double ddmm);

/**
 * @brief Apply a state/mask update to a gnss_pos_t::status field.
 * @param status Pointer to the status word to modify.
 * @param state  New state bits to apply within the masked region.
 * @param mask   Bitmask selecting which bits of @p status are affected.
 */
void set_gnssPos_status_mask(uint32_t *status, uint32_t state, uint32_t mask);

/**
 * @brief Format GPS time from a gnss_pos_t as a UTC HHMMSS.sss string and append to buffer.
 * @param a      Output buffer.
 * @param aSize  Capacity of @p a.
 * @param offset Current write offset; updated on return.
 * @param pos    GNSS position data containing the GPS time of week.
 */
void nmea_GPSTimeToUTCTimeMsPrecision(char* a, int aSize, int &offset, gnss_pos_t &pos);

/**
 * @brief Safe snprintf wrapper that ensures null-termination and returns bytes written.
 * @param buf     Output buffer.
 * @param bufSize Capacity of @p buf.
 * @param fmt     printf-style format string.
 * @param ...     Format arguments.
 * @return Number of bytes written (not counting the null terminator).
 */
int ssnprintf(char buf[], int bufSize, const char *fmt, ...);

//////////////////////////////////////////////////////////////////////////
// Binary to NMEA (encode)
//////////////////////////////////////////////////////////////////////////

/**
 * @brief Encode a $INFO device information sentence.
 * @param a     Output buffer.
 * @param aSize Capacity of @p a.
 * @param info  Device information to encode.
 * @return Number of bytes written, or negative on failure.
 */
int nmea_dev_info(char a[], const int aSize, dev_info_t &info);

/**
 * @brief Encode a $ASCE (auto-start configuration enable) sentence.
 * @param a    Output buffer.
 * @param aSize Capacity of @p a.
 * @param nRMC RMC NMEA configuration to encode.
 * @return Number of bytes written, or negative on failure.
 */
int nmea_ASCE(char a[], const int aSize, rmcNmea_t* nRMC);

/**
 * @brief Encode a $PTOW time-of-week sentence from IMU and INS timestamps.
 * @param a       Output buffer.
 * @param aSize   Capacity of @p a.
 * @param imuTow  IMU time of week (seconds).
 * @param insTow  INS time of week (seconds).
 * @param gpsWeek Current GPS week number.
 * @return Number of bytes written, or negative on failure.
 */
int tow_to_nmea_ptow(char a[], const int aSize, double imuTow, double insTow, unsigned int gpsWeek);

/**
 * @brief Encode a $PIMU proprietary IMU sentence.
 * @param a    Output buffer.
 * @param aSize Capacity of @p a.
 * @param imu  IMU data to encode.
 * @param name Sensor name string appended to the sentence.
 * @return Number of bytes written, or negative on failure.
 */
int nmea_pimu(char a[], const int aSize, imu_t &imu, const char name[]);

/**
 * @brief Encode a $PPIMU preintegrated IMU sentence.
 * @param a     Output buffer.
 * @param aSize Capacity of @p a.
 * @param pimu  Preintegrated IMU data to encode.
 * @return Number of bytes written, or negative on failure.
 */
int nmea_ppimu(char a[], const int aSize, pimu_t &pimu);

/**
 * @brief Encode a $PINS1 proprietary INS sentence (Euler angles, LLA, NED velocity).
 * @param a    Output buffer.
 * @param aSize Capacity of @p a.
 * @param ins1 INS1 data to encode.
 * @return Number of bytes written, or negative on failure.
 */
int nmea_pins1(char a[], const int aSize, ins_1_t &ins1);

/**
 * @brief Encode a $PINS2 proprietary INS sentence (quaternion, LLA).
 * @param a    Output buffer.
 * @param aSize Capacity of @p a.
 * @param ins2 INS2 data to encode.
 * @return Number of bytes written, or negative on failure.
 */
int nmea_pins2(char a[], const int aSize, ins_2_t &ins2);

/**
 * @brief Encode a $PSTRB strobe-in timestamp sentence.
 * @param a      Output buffer.
 * @param aSize  Capacity of @p a.
 * @param strobe Strobe timestamp data to encode.
 * @return Number of bytes written, or negative on failure.
 */
int nmea_pstrb(char a[], const int aSize, strobe_in_time_t &strobe);

/**
 * @brief Encode a $PGPSP proprietary GNSS position and velocity sentence.
 * @param a    Output buffer.
 * @param aSize Capacity of @p a.
 * @param pos  GNSS position data to encode.
 * @param vel  GNSS velocity data to encode.
 * @return Number of bytes written, or negative on failure.
 */
int nmea_pgpsp(char a[], const int aSize, gnss_pos_t &pos, gnss_vel_t &vel);

/**
 * @brief Encode a standard $GGA GNSS fix sentence.
 * @param a    Output buffer.
 * @param aSize Capacity of @p a.
 * @param pos  GNSS position data to encode.
 * @return Number of bytes written, or negative on failure.
 */
int nmea_gga(char a[], const int aSize, gnss_pos_t &pos);

/**
 * @brief Encode a standard $GLL geographic latitude/longitude sentence.
 * @param a    Output buffer.
 * @param aSize Capacity of @p a.
 * @param pos  GNSS position data to encode.
 * @return Number of bytes written, or negative on failure.
 */
int nmea_gll(char a[], const int aSize, gnss_pos_t &pos);

/**
 * @brief Encode a standard $GSA GNSS DOP and active satellite sentence.
 * @param a       Output buffer.
 * @param aSize   Capacity of @p a.
 * @param pos     GNSS position data (provides fix type and DOP).
 * @param gnssSat GNSS satellite data (provides active PRNs).
 * @return Number of bytes written, or negative on failure.
 */
int nmea_gsa(char a[], const int aSize, gnss_pos_t &pos, gnss_sat_t &gnssSat);

/**
 * @brief Encode a standard $RMC recommended minimum specific GNSS sentence.
 * @param a              Output buffer.
 * @param aSize          Capacity of @p a.
 * @param pos            GNSS position data.
 * @param vel            GNSS velocity data.
 * @param magDeclination Magnetic declination in degrees (added to true heading).
 * @return Number of bytes written, or negative on failure.
 */
int nmea_rmc(char a[], const int aSize, gnss_pos_t &pos, gnss_vel_t &vel, float magDeclination);

/**
 * @brief Encode a standard $ZDA UTC date and time sentence.
 * @param a    Output buffer.
 * @param aSize Capacity of @p a.
 * @param pos  GNSS position data (provides GPS time of week and week number).
 * @return Number of bytes written, or negative on failure.
 */
int nmea_zda(char a[], const int aSize, gnss_pos_t &pos);

/**
 * @brief Encode a standard $VTG track made good and ground speed sentence.
 * @param a                   Output buffer.
 * @param aSize               Capacity of @p a.
 * @param pos                 GNSS position data.
 * @param vel                 GNSS velocity data.
 * @param magVarCorrectionRad Magnetic variation correction (radians); default 0.
 * @return Number of bytes written, or negative on failure.
 */
int nmea_vtg(char a[], const int aSize, gnss_pos_t &pos, gnss_vel_t &vel, float magVarCorrectionRad=0.0f);

/**
 * @brief Encode a $PASHR proprietary attitude sentence.
 * @param a     Output buffer.
 * @param aSize Capacity of @p a.
 * @param pos   GNSS position data.
 * @param ins1  INS1 attitude data.
 * @param heave Heave estimate in metres.
 * @param sigma INL2 NED sigma (uncertainty) estimates.
 * @return Number of bytes written, or negative on failure.
 */
int nmea_pashr(char a[], const int aSize, gnss_pos_t &pos, ins_1_t &ins1, float heave, inl2_ned_sigma_t &sigma);

/**
 * @brief Encode $GSV satellite-in-view sentences for a single constellation.
 * @param a      Output buffer.
 * @param aSize  Capacity of @p a.
 * @param gsat   GNSS satellite data (all constellations).
 * @param gsig   GNSS signal data (all constellations).
 * @param gnssId Constellation identifier to filter for.
 * @param noCno  When true, omit C/N0 (signal strength) fields.
 * @return Number of bytes written, or negative on failure.
 */
int nmea_gsv_gnss(char a[], const int aSize, gnss_sat_t &gsat, gnss_sig_t &gsig, uint8_t gnssId, bool noCno=false);

/**
 * @brief Encode $GSV satellite-in-view sentences for all active constellations.
 * @param a       Output buffer.
 * @param aSize   Capacity of @p a.
 * @param gnssSat GNSS satellite data.
 * @param gnssSig GNSS signal data.
 * @return Number of bytes written, or negative on failure.
 */
int nmea_gsv(char a[], const int aSize, gnss_sat_t &gnssSat, gnss_sig_t &gnssSig);

/**
 * @brief Encode a $INTEL proprietary GNSS sentence (IS-internal extended format).
 * @param a    Output buffer.
 * @param aSize Capacity of @p a.
 * @param info Device information.
 * @param pos  GNSS position data.
 * @param vel  GNSS velocity data.
 * @return Number of bytes written, or negative on failure.
 */
int nmea_intel(char a[], const int aSize, dev_info_t &info, gnss_pos_t &pos, gnss_vel_t &vel);

/**
 * @brief Encode a $POWGPS proprietary GNSS position sentence.
 * @param a    Output buffer.
 * @param aSize Capacity of @p a.
 * @param pos  GNSS position data.
 * @return Number of bytes written, or negative on failure.
 */
int nmea_powgps(char a[], const int aSize, gnss_pos_t &pos);

/**
 * @brief Encode a $POWTLV proprietary GNSS position and velocity sentence.
 * @param a    Output buffer.
 * @param aSize Capacity of @p a.
 * @param pos  GNSS position data.
 * @param vel  GNSS velocity data.
 * @return Number of bytes written, or negative on failure.
 */
int nmea_powtlv(char a[], const int aSize, gnss_pos_t &pos, gnss_vel_t &vel);

//////////////////////////////////////////////////////////////////////////
// NMEA to Binary (decode / parse)
//////////////////////////////////////////////////////////////////////////

/**
 * @brief Parse a $INFO sentence into a dev_info_t structure.
 * @param info  Output: device information.
 * @param a     Input NMEA sentence buffer.
 * @param aSize Length of @p a.
 * @return 0 on success, non-zero on parse error.
 */
int nmea_parse_info(dev_info_t &info, const char a[], const int aSize);

/**
 * @brief Parse a $PIMU sentence into an imu_t structure.
 * @param imu   Output: IMU data.
 * @param a     Input NMEA sentence buffer.
 * @param aSize Length of @p a.
 * @return 0 on success, non-zero on parse error.
 */
int nmea_parse_pimu(imu_t &imu, const char a[], const int aSize);

/**
 * @brief Parse a $PIMU sentence into an imu_t structure, converting to raw IMU units.
 * @param imu   Output: raw IMU data.
 * @param a     Input NMEA sentence buffer.
 * @param aSize Length of @p a.
 * @return 0 on success, non-zero on parse error.
 */
int nmea_parse_pimu_to_rimu(imu_t &imu, const char a[], const int aSize);

/**
 * @brief Parse a $PPIMU sentence into a pimu_t structure.
 * @param pimu  Output: preintegrated IMU data.
 * @param a     Input NMEA sentence buffer.
 * @param aSize Length of @p a.
 * @return 0 on success, non-zero on parse error.
 */
int nmea_parse_ppimu(pimu_t &pimu, const char a[], const int aSize);

/**
 * @brief Parse a $PINS1 sentence into an ins_1_t structure.
 * @param ins   Output: INS1 data.
 * @param a     Input NMEA sentence buffer.
 * @param aSize Length of @p a.
 * @return 0 on success, non-zero on parse error.
 */
int nmea_parse_pins1(ins_1_t &ins, const char a[], const int aSize);

/**
 * @brief Parse a $PINS2 sentence into an ins_2_t structure.
 * @param ins   Output: INS2 data.
 * @param a     Input NMEA sentence buffer.
 * @param aSize Length of @p a.
 * @return 0 on success, non-zero on parse error.
 */
int nmea_parse_pins2(ins_2_t &ins, const char a[], const int aSize);

/**
 * @brief Parse a $PGPSP sentence into gnss_pos_t and gnss_vel_t structures.
 * @param gnssPos Output: GNSS position data.
 * @param gnssVel Output: GNSS velocity data.
 * @param a       Input NMEA sentence buffer.
 * @param aSize   Length of @p a.
 * @return 0 on success, non-zero on parse error.
 */
int nmea_parse_pgpsp(gnss_pos_t &gnssPos, gnss_vel_t &gnssVel, const char a[], const int aSize);

/**
 * @brief Parse a $ASCE sentence and update an rmci_t configuration array.
 * @param port  Port handle that received the sentence (used to scope the configuration).
 * @param a     Input NMEA sentence buffer.
 * @param aSize Length of @p a.
 * @param rmci  Vector of pointers to per-port RMC configuration structures to update.
 * @return Bitmask of ports whose configuration was updated.
 */
uint32_t nmea_parse_asce(port_handle_t port, const char a[], int aSize, std::vector<rmci_t*> rmci);

/**
 * @brief Parse a $ASCE sentence and update a grmci_t configuration array.
 * @param port  Port handle that received the sentence.
 * @param a     Input NMEA sentence buffer.
 * @param aSize Length of @p a.
 * @param grmci Vector of pointers to per-port GNSS RMC configuration structures to update.
 * @return Bitmask of ports whose configuration was updated.
 */
uint32_t nmea_parse_asce_grmci(port_handle_t port, const char a[], int aSize, std::vector<grmci_t*> grmci);

/**
 * @brief Parse a $GNS GNSS fix sentence into a gnss_pos_t structure.
 * @param a           Input NMEA sentence buffer.
 * @param aSize       Length of @p a.
 * @param gnssPos     Output: GNSS position data.
 * @param utcTime     Output: parsed UTC time.
 * @param utcWeekday  Day of week (0=Sunday) used to determine the GPS week.
 * @param statusFlags Additional status flags OR'd into gnssPos.status.
 * @return 0 on success, non-zero on parse error.
 */
int nmea_parse_gns(const char a[], const int aSize, gnss_pos_t &gnssPos, utc_time_t &utcTime, int utcWeekday, uint32_t statusFlags=0);

/**
 * @brief Parse a $GGA GNSS fix sentence into a gnss_pos_t structure.
 * @param a           Input NMEA sentence buffer.
 * @param aSize       Length of @p a.
 * @param gnssPos     Output: GNSS position data.
 * @param utcTime     Output: parsed UTC time.
 * @param utcWeekday  Day of week (0=Sunday) used to determine the GPS week.
 * @param statusFlags Additional status flags OR'd into gnssPos.status.
 * @return 0 on success, non-zero on parse error.
 */
int nmea_parse_gga(const char a[], const int aSize, gnss_pos_t &gnssPos, utc_time_t &utcTime, int utcWeekday, uint32_t statusFlags=0);

/**
 * @brief Parse a $GLL geographic latitude/longitude sentence into a gnss_pos_t structure.
 * @param a          Input NMEA sentence buffer.
 * @param aSize      Length of @p a.
 * @param gnssPos    Output: GNSS position data.
 * @param utcTime    Output: parsed UTC time.
 * @param utcWeekday Day of week (0=Sunday) used to determine the GPS week.
 * @return 0 on success, non-zero on parse error.
 */
int nmea_parse_gll(const char a[], const int aSize, gnss_pos_t &gnssPos, utc_time_t &utcTime, int utcWeekday);

/**
 * @brief Parse a $GSA DOP and active satellites sentence into a gnss_pos_t structure.
 * @param a       Input NMEA sentence buffer.
 * @param aSize   Length of @p a.
 * @param gnssPos Output: DOP fields updated in-place.
 * @param gnssSat Optional output: active satellite PRNs updated; pass NULL to skip.
 * @return 0 on success, non-zero on parse error.
 */
int nmea_parse_gsa(const char a[], const int aSize, gnss_pos_t &gnssPos, gnss_sat_t *gnssSat=NULL);

/**
 * @brief Parse one or more $GSV satellite-in-view sentences.
 * @param a        Input NMEA sentence buffer.
 * @param aSize    Length of @p a.
 * @param gnssSat  Output: satellite data updated with parsed entries.
 * @param gnssSig  Output: signal data updated with parsed entries.
 * @param cnoSum   Optional output: running sum of C/N0 values; pass NULL to skip.
 * @param cnoCount Optional output: count of C/N0 values accumulated; pass NULL to skip.
 * @return Pointer to the next unprocessed byte in @p a, or NULL on parse error.
 */
char* nmea_parse_gsv(const char a[], const int aSize, gnss_sat_t *gnssSat, gnss_sig_t *gnssSig, uint32_t *cnoSum, uint32_t *cnoCount);

/**
 * @brief Parse a $INTEL proprietary sentence into device info, GNSS position and velocity.
 * @param a          Input NMEA sentence buffer.
 * @param aSize      Length of @p a.
 * @param info       Output: device information.
 * @param gnssPos    Output: GNSS position data.
 * @param vel        Output: GNSS velocity data.
 * @param ppsPhase   Output: 2-element PPS phase array.
 * @param ppsNoiseNs Output: 1-element PPS noise (nanoseconds) array.
 * @return 0 on success, non-zero on parse error.
 */
int nmea_parse_intel(const char a[], const int aSize, dev_info_t &info, gnss_pos_t &gnssPos, gnss_vel_t &vel, float ppsPhase[2], uint32_t ppsNoiseNs[1]);

/**
 * @brief Parse a $POWGPS sentence into a gnss_pos_t structure.
 * @param a     Input NMEA sentence buffer.
 * @param aSize Length of @p a.
 * @param pos   Output: GNSS position data.
 * @return 0 on success, non-zero on parse error.
 */
int nmea_parse_powgps(const char a[], const int aSize, gnss_pos_t &pos);

/**
 * @brief Parse a $POWTLV sentence into gnss_pos_t and gnss_vel_t structures.
 * @param a     Input NMEA sentence buffer.
 * @param aSize Length of @p a.
 * @param pos   Output: GNSS position data.
 * @param vel   Output: GNSS velocity data.
 * @return 0 on success, non-zero on parse error.
 */
int nmea_parse_powtlv(const char a[], const int aSize, gnss_pos_t &pos, gnss_vel_t &vel);

/**
 * @brief Parse a $RMC recommended minimum specific GNSS sentence.
 * @param a           Input NMEA sentence buffer.
 * @param aSize       Length of @p a.
 * @param gnssVel     Output: GNSS velocity data (speed and heading fields updated).
 * @param utcTime     Output: parsed UTC time.
 * @param utcWeekday  Day of week (0=Sunday) used to determine the GPS week.
 * @param leapS       Current UTC-GPS leap seconds.
 * @param statusFlags Additional status flags OR'd into gnssVel.status.
 * @return 0 on success, non-zero on parse error.
 */
int nmea_parse_rmc(const char a[], int aSize, gnss_vel_t &gnssVel, utc_time_t &utcTime, int utcWeekday, int leapS, uint32_t statusFlags=0);

/**
 * @brief Parse a $VTG track made good and ground speed sentence.
 * @param a      Input NMEA sentence buffer.
 * @param aSize  Length of @p a.
 * @param vel    Output: GNSS velocity data (speed and heading fields updated).
 * @param refLla Reference LLA position used to compute NED velocity direction.
 * @return 0 on success, non-zero on parse error.
 */
int nmea_parse_vtg(const char a[], const int aSize, gnss_vel_t &vel, const double refLla[3]);

/**
 * @brief Parse a $ZDA UTC date and time sentence.
 * @param a        Input NMEA sentence buffer.
 * @param aSize    Length of @p a.
 * @param gpsTowMs Output: GPS time of week in milliseconds.
 * @param gpsWeek  Output: GPS week number.
 * @param date     Output: parsed UTC calendar date.
 * @param time     Output: parsed UTC time of day.
 * @param leapS    Current UTC-GPS leap seconds used for time conversion.
 * @return 0 on success, non-zero on parse error.
 */
int nmea_parse_zda(const char a[], const int aSize, uint32_t &gpsTowMs, uint32_t &gpsWeek, utc_date_t &date, utc_time_t &time, int leapS);

/**
 * @brief Get the GSV constellation presence bitmask for a given constellation identifier.
 * @param constellation Constellation ID.
 * @return Bitmask for that constellation, or 0 if the identifier is invalid.
 */
uint8_t gsv_get_const_mask(uint8_t constellation);

#endif /* PROTOCOL_NMEA_H_ */
