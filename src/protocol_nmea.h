#ifndef PROTOCOL_NMEA_H_
#define PROTOCOL_NMEA_H_

#include "data_sets.h"


enum eNmeaMsgIdUint
{
	ASCII_MSG_ID_ASCB = 0x41534342,		// ASCII messages broadcast periods
	ASCII_MSG_ID_STPB = 0x53545042,		// Stop broadcasts on all ports
	ASCII_MSG_ID_STPC = 0x53545043,		// Stop broadcasts on current port
	ASCII_MSG_ID_BLEN = 0x424c454e,		// Enable bootloader on uINS
	ASCII_MSG_ID_EBLE = 0x45424c45,		// Enable bootloader on EVB
	ASCII_MSG_ID_NELB = 0x4e454c42,		// Enable SAM-BA mode
	ASCII_MSG_ID_SRST = 0x53525354,		// Software reset
	ASCII_MSG_ID_INFO = 0x494e464f,		// Device info
	ASCII_MSG_ID_PERS = 0x50455253,		// Save perstent messages

	ASCII_MSG_ID_PIMU = 0x50494d55,
	ASCII_MSG_ID_PPIM = 0x5050494d,
	ASCII_MSG_ID_PRIM = 0x5052494d,
	ASCII_MSG_ID_PINS = 0x50494e53,
	ASCII_MSG_ID_PGPS = 0x50475053,
	ASCII_MSG_ID_GPGG = 0x47504747,
	ASCII_MSG_ID_GPGL = 0x4750474c,
	ASCII_MSG_ID_GPGS = 0x47504753,
	ASCII_MSG_ID_GPRM = 0x4750524d,
	ASCII_MSG_ID_GPZD = 0x47505a44,
	ASCII_MSG_ID_PASH = 0x50415348,
};


//////////////////////////////////////////////////////////////////////////
// Utility functions
//////////////////////////////////////////////////////////////////////////
uint32_t ASCII_compute_checksum(uint8_t* str, int size);
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
void nmea_set_rmc_period_multiple(rmci_t &rmci, ascii_msgs_t tmp);

//////////////////////////////////////////////////////////////////////////
// DID to NMEA
//////////////////////////////////////////////////////////////////////////
int did_dev_info_to_nmea_info(char a[], const int aSize, dev_info_t &info);
int tow_to_nmea_ptow(char a[], const int aSize, double imuTow, double insTow, unsigned int gpsWeek);
int did_imu_to_nmea_pimu(char a[], const int aSize, imu_t &imu, const char name[]);
int did_pimu_to_nmea_ppimu(char a[], const int aSize, pimu_t &pimu);
int did_ins1_to_nmea_pins1(char a[], const int aSize, ins_1_t &ins1);
int did_ins2_to_nmea_pins2(char a[], const int aSize, ins_2_t &ins2);
int did_strobe_to_nmea_pstrb(char a[], const int aSize, strobe_in_time_t &strobe);
int did_gps_to_nmea_pgpsp(char a[], const int aSize, gps_pos_t &pos, gps_vel_t &vel);
int did_gps_to_nmea_gga(char a[], const int aSize, gps_pos_t &pos);
int did_gps_to_nmea_gll(char a[], const int aSize, gps_pos_t &pos);
int did_gps_to_nmea_gsa(char a[], const int aSize, gps_pos_t &pos, gps_sat_t &sat);
int did_gps_to_nmea_rmc(char a[], const int aSize, gps_pos_t &pos, gps_vel_t &vel, float magDeclination);
int did_gps_to_nmea_zda(char a[], const int aSize, gps_pos_t &pos);
int did_gps_to_nmea_pashr(char a[], const int aSize, gps_pos_t &pos, ins_1_t &ins1, float heave, inl2_ned_sigma_t &sigma);

//////////////////////////////////////////////////////////////////////////
// NMEA to DID
//////////////////////////////////////////////////////////////////////////
int nmea_pimu_to_did_imu(imu_t &imu, const char a[], const int aSize);
int nmea_pimu_to_did_rimu(imu_t &imu, const char a[], const int aSize);
int nmea_ppimu_to_did_pimu(pimu_t &pimu, const char a[], const int aSize);
int nmea_pins1_to_did_ins1(ins_1_t &ins, const char a[], const int aSize);
int nmea_pins2_to_did_ins2(ins_2_t &ins, const char a[], const int aSize);
int nmea_pgpsp_to_did_gps(gps_pos_t &gpsPos, gps_vel_t &gpsVel, const char a[], const int aSize);

//////////////////////////////////////////////////////////////////////////
// NMEA parse
//////////////////////////////////////////////////////////////////////////
uint32_t parse_nmea_ascb(int pHandle, const char msg[], int msgSize, rmci_t rmci[NUM_COM_PORTS]);
int parse_nmea_zda(const char msgBuf[], int msgSize, double &day, double &month, double &year);
int parse_nmea_gns(const char msgBuf[], int msgSize, gps_pos_t *gpsPos, double datetime[6], uint32_t *satsUsed, uint32_t statusFlags=0);
int parse_nmea_gga(const char msg[], int msgSize, gps_pos_t *gpsPos, double datetime[6], uint32_t *satsUsed, uint32_t statusFlags=0);
int parse_nmea_rmc(const char msg[], int msgSize, gps_vel_t *gpsVel, double datetime[6], uint32_t statusFlags=0);
int parse_nmea_gsa(const char msg[], int msgSize, gps_pos_t *gpsPos, int *navMode);
int parse_nmea_gsv(const char msg[], int msgSize, gps_sat_t* gpsSat, int lastGSVmsg[2], int *satPointer, uint32_t *cnoSum, uint32_t *cnoCount);



#endif /* PROTOCOL_NMEA_H_ */
