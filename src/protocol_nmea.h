#ifndef PROTOCOL_NMEA_H_
#define PROTOCOL_NMEA_H_

#include "data_sets.h"
#include "time_conversion.h"

#if !defined(GPX_1) && !defined(IMX_5) && !defined(NAV_POST_PROCESS)
extern uint32_t g_cpu_msec;
#endif

#define NMEA_CMD_QUERY_DEVICE_INFO                      "$INFO*0E\r\n"
#define NMEA_CMD_STOP_ALL_BROADCASTS_ALL_PORTS          "$STPB*15\r\n"
#define NMEA_CMD_STOP_ALL_BROADCASTS_CUR_PORT           "$STPC*14\r\n"
#define NMEA_CMD_SAVE_PERSISTENT_MESSAGES_TO_FLASH      "$PERS*14\r\n"
#define NMEA_CMD_SOFTWARE_RESET                         "$SRST*06\r\n"
#define NMEA_CMD_SIZE                                   10

enum eNmeaProtocolVersion
{
    NMEA_PROTOCOL_2P3 		= 230,	// 2.3 (Default version)
    NMEA_PROTOCOL_4P10 		= 410,	// 4.10
};

//////////////////////////////////////////////////////////////////////////
// Utility functions
//////////////////////////////////////////////////////////////////////////
void nmea_setGsvFilter(int constellation, uint8_t filter);
void nmea_setGsvFilter(uint8_t* filters);
int nmea_getGsvFilter(int constellation);
void nmea_getGsvFilter(uint8_t* filters);
void nmea_enable_stream(uint32_t& bits, uint8_t* period, uint32_t nmeaId, uint8_t periodMultiple);
void nmea_set_protocol_version(int protocol_version);
void nmea_set_gnss_id(int gnssId);
uint32_t nmea_compute_checksum(uint8_t* str, int size);
void nmea_sprint(char buf[], int bufSize, int &offset, const char *fmt, ...);
int nmea_sprint_footer(char* a, int aSize, int &n);
char *ASCII_find_next_field(char *str);
char *ASCII_to_u8(uint8_t *val, char *ptr);
char *ASCII_to_u16(uint16_t *val, char *ptr);
char *ASCII_to_u32(uint32_t *val, char *ptr);
char *ASCII_to_i32(int32_t *val, char *ptr);
char *ASCII_to_vec3f(float vec[], char *ptr);
char *ASCII_to_vec4f(float vec[], char *ptr);
char *ASCII_to_vec3d(double vec[], char *ptr);
double ddmm2deg(double ddmm);
void set_gpsPos_status_mask(uint32_t *status, uint32_t state, uint32_t mask);
void nmea_GPSTimeToUTCTimeMsPrecision(char* a, int aSize, int &offset, gps_pos_t &pos);
void nmea_GPSTimeToUTCTimeMsPrecision_ZDA_debug(char* a, int aSize, int &offset, gps_pos_t &pos);
int ssnprintf(char buf[], int bufSize, const char *fmt, ...);

//////////////////////////////////////////////////////////////////////////
// Binary to NMEA
//////////////////////////////////////////////////////////////////////////
int nmea_dev_info(char a[], const int aSize, dev_info_t &info);
int nmea_ASCE(char a[], const int aSize, rmcNmea_t* nRMC);
int tow_to_nmea_ptow(char a[], const int aSize, double imuTow, double insTow, unsigned int gpsWeek);
int nmea_pimu(char a[], const int aSize, imu_t &imu, const char name[]);
int nmea_ppimu(char a[], const int aSize, pimu_t &pimu);
int nmea_pins1(char a[], const int aSize, ins_1_t &ins1);
int nmea_pins2(char a[], const int aSize, ins_2_t &ins2);
int nmea_pstrb(char a[], const int aSize, strobe_in_time_t &strobe);
int nmea_pgpsp(char a[], const int aSize, gps_pos_t &pos, gps_vel_t &vel);
int nmea_gga(char a[], const int aSize, gps_pos_t &pos);
int nmea_gll(char a[], const int aSize, gps_pos_t &pos);
int nmea_gsa(char a[], const int aSize, gps_pos_t &pos, gps_sat_t &gpsSat);
int nmea_rmc(char a[], const int aSize, gps_pos_t &pos, gps_vel_t &vel, float magDeclination);
int nmea_zda(char a[], const int aSize, gps_pos_t &pos);
int nmea_vtg(char a[], const int aSize, gps_pos_t &pos, gps_vel_t &vel, float magVarCorrectionRad=0.0f);
int nmea_pashr(char a[], const int aSize, gps_pos_t &pos, ins_1_t &ins1, float heave, inl2_ned_sigma_t &sigma);
int nmea_gsv_gnss(char a[], const int aSize, gps_sat_t &gsat, gps_sig_t &gsig, uint8_t gnssId, bool noCno=false);
int nmea_gsv(char a[], const int aSize, gps_sat_t &gpsSat, gps_sig_t &gpsSig);
int nmea_intel(char a[], const int aSize, dev_info_t &info, gps_pos_t &pos, gps_vel_t &vel);
int nmea_powgps(char a[], const int aSize, gps_pos_t &pos);
int nmea_powtlv(char a[], const int aSize, gps_pos_t &pos, gps_vel_t &vel);

//////////////////////////////////////////////////////////////////////////
// NMEA to Binary
//////////////////////////////////////////////////////////////////////////
int nmea_parse_info(dev_info_t &info, const char a[], const int aSize);
int nmea_parse_pimu(imu_t &imu, const char a[], const int aSize);
int nmea_parse_pimu_to_rimu(imu_t &imu, const char a[], const int aSize);
int nmea_parse_ppimu(pimu_t &pimu, const char a[], const int aSize);
int nmea_parse_pins1(ins_1_t &ins, const char a[], const int aSize);
int nmea_parse_pins2(ins_2_t &ins, const char a[], const int aSize);
int nmea_parse_pgpsp(gps_pos_t &gpsPos, gps_vel_t &gpsVel, const char a[], const int aSize);
uint32_t nmea_parse_asce(int pHandle, const char a[], int aSize, rmci_t rmci[NUM_COM_PORTS]);
uint32_t nmea_parse_asce_grmci(int pHandle, const char a[], int aSize, grmci_t rmci[NUM_COM_PORTS]);
// NMEA standard messages
int nmea_parse_gns(const char a[], const int aSize, gps_pos_t &gpsPos, utc_time_t &utcTime, int utcWeekday, uint32_t statusFlags=0);
int nmea_parse_gga(const char a[], const int aSize, gps_pos_t &gpsPos, utc_time_t &utcTime, int utcWeekday, uint32_t statusFlags=0);
int nmea_parse_gll(const char a[], const int aSize, gps_pos_t &gpsPos, utc_time_t &utcTime, int utcWeekday);
int nmea_parse_gsa(const char a[], const int aSize, gps_pos_t &gpsPos, gps_sat_t *gpsSat=NULL);
char* nmea_parse_gsv(const char a[], const int aSize, gps_sat_t *gpsSat, gps_sig_t *gpsSig, uint32_t *cnoSum, uint32_t *cnoCount);
int nmea_parse_intel(const char a[], const int aSize, dev_info_t &info, gps_pos_t &gpsPos, gps_vel_t &vel, float ppsPhase[2], uint32_t ppsNoiseNs[1]);
int nmea_parse_powgps(const char a[], const int aSize, gps_pos_t &pos);
int nmea_parse_powtlv(const char a[], const int aSize, gps_pos_t &pos, gps_vel_t &vel);
int nmea_parse_rmc(const char a[], int aSize, gps_vel_t &gpsVel, utc_time_t &utcTime, int utcWeekday, int leapS, uint32_t statusFlags=0);
int nmea_parse_vtg(const char a[], const int aSize, gps_vel_t &vel, const double refLla[3]);
int nmea_parse_zda(const char a[], const int aSize, uint32_t &gpsTowMs, uint32_t &gpsWeek, utc_date_t &date, utc_time_t &time, int leapS);

/**
 * Gets GSV constellation mask for a given constellation ID
 * returns mask if value constellation passed and 0 if
 * an invalid constellation is passed
*/
uint8_t gsv_get_const_mask(uint8_t constellation);

#endif /* PROTOCOL_NMEA_H_ */
