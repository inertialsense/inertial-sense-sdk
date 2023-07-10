#ifndef PROTOCOL_NMEA_H_
#define PROTOCOL_NMEA_H_

#include "data_sets.h"


enum eNmeaMsgIdUint
{
	NMEA_MSG_UINT_ASCB = 0x41534342,		// "ASCB" - NMEA messages broadcast periods
	NMEA_MSG_UINT_ASCE = 0x41534345,		// "ASCE" - NMEA messages broadcast enable
	NMEA_MSG_UINT_STPB = 0x53545042,		// "STPB" - Stop broadcasts on all ports
	NMEA_MSG_UINT_STPC = 0x53545043,		// "STPC" - Stop broadcasts on current port
	NMEA_MSG_UINT_BLEN = 0x424c454e,		// "EBLE" - Enable bootloader on IMX (app firmware update)
	NMEA_MSG_UINT_SRST = 0x53525354,		// "SRTS" - Software reset
	NMEA_MSG_UINT_INFO = 0x494e464f,		// "INFO" - Device info
	NMEA_MSG_UINT_PERS = 0x50455253,		// "PERS" - Save perstent messages

	NMEA_MSG_UINT_PIMU = 0x50494d55,		// "PIMU"
	NMEA_MSG_UINT_PPIM = 0x5050494d,		// "PPIM"
	NMEA_MSG_UINT_PRIM = 0x5052494d,		// "PRIM"
	NMEA_MSG_UINT_PINS = 0x50494e53,		// "PINS"
	NMEA_MSG_UINT_PGPS = 0x50475053,		// "PGPS"
	NMEA_MSG_UINT_PASH = 0x50415348,		// "PASH"
	
	NMEA_MSG_UINT_GGA = 0x4747412c,			// "GGA,"
	NMEA_MSG_UINT_GLL = 0x474c4c2c,			// "GLL,"
	NMEA_MSG_UINT_GSA = 0x4753412c,			// "GSA,"
	NMEA_MSG_UINT_RMC = 0x524d432c, 		// "RMC,"
	NMEA_MSG_UINT_ZDA = 0x5a44412c, 		// "ZDA,"
};

enum eNmeaProtocolVersion
{
	NMEA_PROTOCOL_2P3 		= 0,	// <4.10
	NMEA_PROTOCOL_4P10 		= 410,	// 4.10
};


//////////////////////////////////////////////////////////////////////////
// Utility functions
//////////////////////////////////////////////////////////////////////////
void nmea_enable_stream(rmci_t &rmci, uint32_t nmeaId, uint8_t periodMultiple);
void nmea_set_protocol_version(int protocol_version);
void nmea_set_gnss_id(int gnssId);
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
void nmea_set_rmc_period_multiple(rmci_t &rmci, nmea_msgs_t tmp);

//////////////////////////////////////////////////////////////////////////
// Binary to NMEA
//////////////////////////////////////////////////////////////////////////
int nmea_dev_info(char a[], const int aSize, dev_info_t &info);
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
int nmea_pashr(char a[], const int aSize, gps_pos_t &pos, ins_1_t &ins1, float heave, inl2_ned_sigma_t &sigma);
int nmea_gsv_gnss(char a[], int aSize, int &offset, gps_sat_t &gsat, gps_sig_t &gsig, uint8_t gnssId, bool noCno=false);
int nmea_gsv(char a[], const int aSize, gps_sat_t &gpsSat, gps_sig_t &gpsSig);


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
int nmea_gga_to_did_gps(gps_pos_t &gpsPos, const char a[], const int aSize, uint32_t weekday);
int nmea_gll_to_did_gps(gps_pos_t &gpsPos, const char a[], const int aSize, uint32_t weekday);
int nmea_gsa_to_did_gps(gps_pos_t &gpsPos, gps_sat_t &gpsSat, const char a[], const int aSize);
int nmea_gsv_to_did_gps_sat(gps_sat_t &gpsSat, const char a[], const int aSize);

uint32_t nmea_parse_ascb(int pHandle, const char msg[], int msgSize, rmci_t rmci[NUM_COM_PORTS]);
uint32_t nmea_parse_asce(int pHandle, const char msg[], int msgSize, rmci_t rmci[NUM_COM_PORTS]);
int nmea_parse_zda(const char msgBuf[], int msgSize, double &day, double &month, double &year);
int nmea_parse_gns(const char msgBuf[], int msgSize, gps_pos_t *gpsPos, double datetime[6], uint32_t *satsUsed, uint32_t statusFlags=0);
int nmea_parse_gga(const char a[], int aSize, gps_pos_t *gpsPos, double datetime[6], uint32_t *satsUsed, uint32_t statusFlags=0);
int nmea_parse_rmc(const char a[], int aSize, gps_vel_t *gpsVel, double datetime[6], uint32_t statusFlags=0);
int nmea_parse_gsa(const char a[], int aSize, gps_pos_t *gpsPos, int *navMode);
char* nmea_parse_gsv(const char a[], int aSize, gps_sat_t *gpsSat, gps_sig_t *gpsSig, uint32_t *cnoSum, uint32_t *cnoCount);



#endif /* PROTOCOL_NMEA_H_ */
