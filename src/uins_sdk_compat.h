#ifndef __IS_UINS_SDK_COMPAT_H
#define __IS_UINS_SDK_COMPAT_H

#ifdef __cplusplus
extern "C" {
#endif

#include "serialPort.h" // sam
#include "serialPort_dfu.h"
// TODO:  #include "serialPort_uart.h"

#include "serialPortPlatform.h" // sam
#include "serialPortPlatform_dfu.h"
// TODO:  #include "serialPortPlatform_uart.h"

#include "inertialSenseBootLoader.h" // sam
#include "inertialSenseBootLoader_dfu.h"
// TODO:  #include "inertialSenseBootLoader_uart.h"

typedef enum bootloader_flags {
    IS_BOOTLOADER_FLAG_SAM   = 0b00000000,  // default
    IS_BOOTLOADER_FLAG_DFU   = 0b00000001,
    IS_BOOTLOADER_FLAG_UART  = 0b00000010,

    IS_BOOTLOADER_FLAG_RSVD1 = 0b00000100,
    IS_BOOTLOADER_FLAG_RSVD2 = 0b00001000,
    IS_BOOTLOADER_FLAG_RSVD3 = 0b00010000,
    IS_BOOTLOADER_FLAG_RSVD4 = 0b00100000,
    IS_BOOTLOADER_FLAG_RSVD5 = 0b01000000,
    IS_BOOTLOADER_FLAG_RSVD6 = 0b10000000,
} bootloader_flags_t;

typedef unsigned char communications_flags_t;   // 1111 1111

typedef struct uins_device
{
    int version_major;
    int version_minor;
    communications_flags_t bootloader_flash_support;
} uins_device_t;

void create_device_uins31(uins_device_t* device);
void create_device_uins50(uins_device_t* device);

int serialPortPlatformInitCompat(uins_device_t* device, serial_port_t* serialPort);
void serialPortSetPortCompat(uins_device_t* device, serial_port_t* serialPort, const char* port);
int serialPortOpenCompat(uins_device_t* device, serial_port_t* serialPort, const char* port, int baudRate, int blocking);
int serialPortOpenRetryCompat(uins_device_t* device, serial_port_t* serialPort, const char* port, int baudRate, int blocking);
int serialPortIsOpenCompat(uins_device_t* device, serial_port_t* serialPort);
int serialPortCloseCompat(uins_device_t* device, serial_port_t* serialPort);
int serialPortFlushCompat(uins_device_t* device, serial_port_t* serialPort);
int serialPortReadCompat(uins_device_t* device, serial_port_t* serialPort, unsigned char* buffer, int readCount);
int serialPortReadTimeoutCompat(uins_device_t* device, serial_port_t* serialPort, unsigned char* buffer, int readCount, int timeoutMilliseconds);
int serialPortReadTimeoutAsyncCompat(uins_device_t* device, serial_port_t* serialPort, unsigned char* buffer, int readCount, pfnSerialPortAsyncReadCompletion callback);
int serialPortReadLineCompat(uins_device_t* device, serial_port_t* serialPort, unsigned char* buffer, int bufferLength);
int serialPortReadLineTimeoutCompat(uins_device_t* device, serial_port_t* serialPort, unsigned char* buffer, int bufferLength, int timeoutMilliseconds);
int serialPortReadAsciiCompat(uins_device_t* device, serial_port_t* serialPort, unsigned char* buffer, int bufferLength, unsigned char** asciiData);
int serialPortReadAsciiTimeoutCompat(uins_device_t* device, serial_port_t* serialPort, unsigned char* buffer, int bufferLength, int timeoutMilliseconds, unsigned char** asciiData);
int serialPortReadCharCompat(uins_device_t* device, serial_port_t* serialPort, unsigned char* c);
int serialPortReadCharTimeoutCompat(uins_device_t* device, serial_port_t* serialPort, unsigned char* c, int timeoutMilliseconds);
int serialPortWriteCompat(uins_device_t* device, serial_port_t* serialPort, const unsigned char* buffer, int writeCount);
int serialPortWriteLineCompat(uins_device_t* device, serial_port_t* serialPort, const unsigned char* buffer, int writeCount);
int serialPortWriteAsciiCompat(uins_device_t* device, serial_port_t* serialPort, const char* buffer, int bufferLength);
int serialPortWriteAndWaitForCompat(uins_device_t* device, serial_port_t* serialPort, const unsigned char* buffer, int writeCount, const unsigned char* waitFor, int waitForLength);
int serialPortWriteAndWaitForTimeoutCompat(uins_device_t* device, serial_port_t* serialPort, const unsigned char* buffer, int writeCount, const unsigned char* waitFor, int waitForLength, const int timeoutMilliseconds);
int serialPortWaitForCompat(uins_device_t* device, serial_port_t* serialPort, const unsigned char* waitFor, int waitForLength);
int serialPortWaitForTimeoutCompat(uins_device_t* device, serial_port_t* serialPort, const unsigned char* waitFor, int waitForLength, int timeoutMilliseconds);
int serialPortGetByteCountAvailableToReadCompat(uins_device_t* device, serial_port_t* serialPort);
int serialPortGetByteCountAvailableToWriteCompat(uins_device_t* device, serial_port_t* serialPort);
int serialPortSleepCompat(uins_device_t* device, serial_port_t* serialPort, int sleepMilliseconds);

int bootloadFileCompat(serial_port_t* port, const char* fileName, const char* bootName, const void* obj, pfnBootloadProgress uploadProgress, pfnBootloadProgress verifyProgress);
int bootloadFileExCompat(bootload_params_t* params);
int bootloadUpdateBootloaderCompat(serial_port_t* port, const char* fileName, const void* obj, pfnBootloadProgress uploadProgress, pfnBootloadProgress verifyProgress);
int bootloadUpdateBootloaderExCompat(bootload_params_t* p);
int bootloadGetBootloaderVersionFromFileCompat(const char* bootName, int* verMajor, char* verMinor);
int enableBootloaderCompat(serial_port_t* port, int baudRate, char* error, int errorLength, const char* bootloadEnableCmd);
int disableBootloaderCompat(serial_port_t* port, char* error, int errorLength);
int bootloaderCycleBaudRateCompat(int baudRate);
int bootloaderClosestBaudRateCompat(int baudRate);

#ifdef __cplusplus
}
#endif

#endif // __IS_UINS_SDK_COMPAT_H
