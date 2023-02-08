#include "firmwareSignatures.h"
#include "ISConstants.h"
#include <string.h>

#define BOOTLOADER_HASH_MIXER 0x5bd1e995
#define BOOTLOADER_HASH_SHIFTER 24

/**
 * This is a signature that is embedded at the start of each firmware image that
 * we distribute. The signature is linked right after the vector table in 
 * firmware so it can be searched for
 */

// Current signatures
#define sign_uINS_3_24K 	{ 0x97, 0x0F, 0xF9, 0xA7, 0x17, 0x7D, 0x4E, 0x99, 0xDB, 0x53, 0xA2, 0x72, 0xE7, 0xC3, 0xE1, 0xE6 }
#define sign_EVB_2_24K 		{ 0x96, 0x0F, 0xF9, 0xA7, 0x17, 0x7D, 0x4E, 0x99, 0xDB, 0x53, 0xA2, 0x72, 0xE7, 0xC3, 0xE1, 0xE5 }
#define sign_IMX_5p0 		{ 0xA5, 0x0F, 0xF9, 0xA7, 0x17, 0x7D, 0x4E, 0x99, 0xDB, 0x53, 0xA2, 0x72, 0xE7, 0xC3, 0xE1, 0x42 }
#define sign_GPX_1 			{ 0xB0, 0x0F, 0xF9, 0xA7, 0x17, 0x7D, 0x4E, 0x99, 0xDB, 0x53, 0xA2, 0x72, 0xE7, 0xC3, 0xE1, 0xBB }

const uint8_t bootloaderRequiredSignature_EVB_2_24K[BOOTLOADER_SIGNATURE_SIZE] = sign_EVB_2_24K;
const uint8_t bootloaderRequiredSignature_uINS_3_24K[BOOTLOADER_SIGNATURE_SIZE] = sign_uINS_3_24K;
const uint8_t bootloaderRequiredSignature_IMX_5p0[BOOTLOADER_SIGNATURE_SIZE] = sign_IMX_5p0;
const uint8_t bootloaderRequiredSignature_GPX_1[BOOTLOADER_SIGNATURE_SIZE] = sign_GPX_1;

#if PLATFORM_IS_EMBEDDED
__attribute__ ((section(".fw_signature"))) const uint8_t _fw_signature_[BOOTLOADER_SIGNATURE_SIZE] = 
#if defined(IMX_5)
sign_IMX_5p0;
#elif defined(GPX_1)
sign_GPX_1;
#else
#error "Unknown device type"
#endif	// Device type

#endif	// PLATFORM_IS_EMBEDDED
