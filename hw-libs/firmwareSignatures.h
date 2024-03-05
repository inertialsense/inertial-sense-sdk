#ifndef __BOOTLOADER_SHARED_H__
#define __BOOTLOADER_SHARED_H__
#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define BOOTLOADER_SIGNATURE_SIZE 16U   // Must be multiple of 4

extern const uint8_t bootloaderRequiredSignature_IMX5p0[BOOTLOADER_SIGNATURE_SIZE];
extern const uint8_t bootloaderRequiredSignature_GPX1p0[BOOTLOADER_SIGNATURE_SIZE];

#ifdef __cplusplus
}
#endif
#endif  // __BOOTLOADER_SHARED_H__
