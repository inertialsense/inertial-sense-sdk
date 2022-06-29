#include "bootloaderShared.h"
#include <string.h>

#define BOOTLOADER_HASH_MIXER 0x5bd1e995
#define BOOTLOADER_HASH_SHIFTER 24

uint8_t bootloaderRequiredSignature_EVB_2_16K[BOOTLOADER_SIGNATURE_SIZE] = 
	{ 0x45, 0x0F, 0xF9, 0xA7, 0x17, 0x7D, 0x4E, 0x99, 0xDB, 0x53, 0xA2, 0x72, 0xE7, 0xC3, 0xE1, 0x51 };
uint8_t bootloaderRequiredSignature_uINS_3_16K[BOOTLOADER_SIGNATURE_SIZE] = 
	{ 0x96, 0x0F, 0xF9, 0xA7, 0x17, 0x7D, 0x4E, 0x99, 0xDB, 0x53, 0xA2, 0x72, 0xE7, 0xC3, 0xE1, 0xE5 };
uint8_t bootloaderRequiredSignature_SAMx70_bootloader_16K[BOOTLOADER_SIGNATURE_SIZE] =
	{ 0x43, 0x0F, 0xF9, 0xA7, 0x17, 0x7D, 0x4E, 0x99, 0xDB, 0x53, 0xA2, 0x72, 0xE7, 0xC3, 0xE1, 0x35 };
uint8_t bootloaderRequiredSignature_EVB_2_24K[BOOTLOADER_SIGNATURE_SIZE] = 
	{ 0x46, 0x0F, 0xF9, 0xA7, 0x17, 0x7D, 0x4E, 0x99, 0xDB, 0x53, 0xA2, 0x72, 0xE7, 0xC3, 0xE1, 0x52 };
uint8_t bootloaderRequiredSignature_uINS_3_24K[BOOTLOADER_SIGNATURE_SIZE] =
	{ 0x97, 0x0F, 0xF9, 0xA7, 0x17, 0x7D, 0x4E, 0x99, 0xDB, 0x53, 0xA2, 0x72, 0xE7, 0xC3, 0xE1, 0xE6 };
uint8_t bootloaderRequiredSignature_SAMx70_bootloader_24K[BOOTLOADER_SIGNATURE_SIZE] =
	{ 0x44, 0x0F, 0xF9, 0xA7, 0x17, 0x7D, 0x4E, 0x99, 0xDB, 0x53, 0xA2, 0x72, 0xE7, 0xC3, 0xE1, 0x36 };
uint8_t bootloaderRequiredSignature_uINS_5[BOOTLOADER_SIGNATURE_SIZE] =
	{ 0xA5, 0x0F, 0xF9, 0xA7, 0x17, 0x7D, 0x4E, 0x99, 0xDB, 0x53, 0xA2, 0x72, 0xE7, 0xC3, 0xE1, 0x42 };
uint8_t bootloaderRequiredSignature_STM32L4_bootloader[BOOTLOADER_SIGNATURE_SIZE] =
	{ 0x20, 0x0F, 0xF9, 0xA7, 0x17, 0x7D, 0x4E, 0x99, 0xDB, 0x53, 0xA2, 0x72, 0xE7, 0xC3, 0xE1, 0xFA };

// calculate bootloader hash code over a set of data
uint32_t calculateBootloaderHashCode(uint32_t hashCode, const uint32_t* start, const uint32_t* end)
{
	uint32_t value;
	while (start < end)
	{
		value = *start++;
		value *= BOOTLOADER_HASH_MIXER;
		value ^= value >> BOOTLOADER_HASH_SHIFTER;
		value *= BOOTLOADER_HASH_MIXER;
		hashCode *= BOOTLOADER_HASH_MIXER;
		hashCode ^= value;
	}
	return hashCode;
}