#ifndef _D_CRC_H
#define _D_ADC_H

#include "ISBoards.h"
#include "stm32l4xx.h"

#ifdef __cplusplus
extern "C" {
#endif

#define CRC_DATASIZ_BYTE    0x00
#define CRC_DATASIZ_HWORD   0x01
#define CRC_DATASIZ_WORD    0x02

#define CRC_POLYSIZE_32     0x00
#define CRC_POLYSIZE_16     0x08
#define CRC_POLYSIZE_8      0x10
#define CRC_POLYSIZE_7      0x18

#define CRC_REVIN_NONE      0x00
#define CRC_REVIN_BYTE      0x20
#define CRC_REVIN_HWORD     0x40
#define CRC_REVIN_WORD      0x60

#define CRC_REVOUT_NONE     0x00
#define CRC_REVOUT_WORD     0x80

void crc_init(void);
int8_t crc_start(uint32_t settings, uint32_t polynomial, uint32_t initial);
uint32_t calc_crc_bytes(uint8_t* data, uint8_t len);

#ifdef __cplusplus
}
#endif

#endif	// _D_ADC_H
