#include "uins_sdk_compat.h"

void create_device_uins31(uins_device_t* device)
{
    memset(device, 0, sizeof(device));
    device->version_major = 3;
    device->version_minor = 1;
    device->bootloader_flash_support = IS_BOOTLOADER_FLAG_SAM;
}

void create_device_uins50(uins_device_t* device)
{
    memset(device, 0, sizeof(device));
    device->version_major = 5;
    device->version_minor = 0;
    device->bootloader_flash_support = IS_BOOTLOADER_FLAG_DFU | IS_BOOTLOADER_FLAG_UART;
}

int serialPortPlatformInitCompat(uins_device_t* device, serial_port_t* serialPort)
{
    if (device->bootloader_flash_support & IS_BOOTLOADER_FLAG_DFU)
    {
        return serialPortPlatformInitDfu(serialPort);
    }

    return serialPortPlatformInit(serialPort);
}

void serialPortSetPortCompat(uins_device_t* device, serial_port_t* serialPort, const char* port)
{
    if (device->bootloader_flash_support & IS_BOOTLOADER_FLAG_DFU)
    {
        serialPortSetPortDfu(serialPort, port);
        return;
    }

    serialPortSetPort(serialPort, port);
}

int serialPortOpenCompat(uins_device_t* device, serial_port_t* serialPort, const char* port, int baudRate, int blocking)
{
    if (device->bootloader_flash_support & IS_BOOTLOADER_FLAG_DFU)
    {
        return serialPortOpenDfu(serialPort, port, baudRate, blocking);
    }

    return serialPortOpen(serialPort, port, baudRate, blocking);
}

int serialPortOpenRetryCompat(uins_device_t* device, serial_port_t* serialPort, const char* port, int baudRate, int blocking)
{
    return 0;
}

int serialPortIsOpenCompat(uins_device_t* device, serial_port_t* serialPort)
{
    return 0;
}

int serialPortCloseCompat(uins_device_t* device, serial_port_t* serialPort)
{
    return 0;
}

int serialPortFlushCompat(uins_device_t* device, serial_port_t* serialPort)
{
    return 0;
}

int serialPortReadCompat(uins_device_t* device, serial_port_t* serialPort, unsigned char* buffer, int readCount)
{
    return 0;
}

int serialPortReadTimeoutCompat(uins_device_t* device, serial_port_t* serialPort, unsigned char* buffer, int readCount, int timeoutMilliseconds)
{
    return 0;
}

int serialPortReadTimeoutAsyncCompat(uins_device_t* device, serial_port_t* serialPort, unsigned char* buffer, int readCount, pfnSerialPortAsyncReadCompletion callback)
{
    return 0;
}

int serialPortReadLineCompat(uins_device_t* device, serial_port_t* serialPort, unsigned char* buffer, int bufferLength)
{
    return 0;
}

int serialPortReadLineTimeoutCompat(uins_device_t* device, serial_port_t* serialPort, unsigned char* buffer, int bufferLength, int timeoutMilliseconds)
{
    return 0;
}

int serialPortReadAsciiCompat(uins_device_t* device, serial_port_t* serialPort, unsigned char* buffer, int bufferLength, unsigned char** asciiData)
{
    return 0;
}

int serialPortReadAsciiTimeoutCompat(uins_device_t* device, serial_port_t* serialPort, unsigned char* buffer, int bufferLength, int timeoutMilliseconds, unsigned char** asciiData)
{
    return 0;
}

int serialPortReadCharCompat(uins_device_t* device, serial_port_t* serialPort, unsigned char* c)
{
    return 0;
}

int serialPortReadCharTimeoutCompat(uins_device_t* device, serial_port_t* serialPort, unsigned char* c, int timeoutMilliseconds)
{
    return 0;
}

int serialPortWriteCompat(uins_device_t* device, serial_port_t* serialPort, const unsigned char* buffer, int writeCount)
{
    return 0;
}

int serialPortWriteLineCompat(uins_device_t* device, serial_port_t* serialPort, const unsigned char* buffer, int writeCount)
{
    return 0;
}

int serialPortWriteAsciiCompat(uins_device_t* device, serial_port_t* serialPort, const char* buffer, int bufferLength)
{
    return 0;
}

int serialPortWriteAndWaitForCompat(uins_device_t* device, serial_port_t* serialPort, const unsigned char* buffer, int writeCount, const unsigned char* waitFor, int waitForLength)
{
    return 0;
}

int serialPortWriteAndWaitForTimeoutCompat(uins_device_t* device, serial_port_t* serialPort, const unsigned char* buffer, int writeCount, const unsigned char* waitFor, int waitForLength, const int timeoutMilliseconds)
{
    return 0;
}

int serialPortWaitForCompat(uins_device_t* device, serial_port_t* serialPort, const unsigned char* waitFor, int waitForLength)
{
    return 0;
}

int serialPortWaitForTimeoutCompat(uins_device_t* device, serial_port_t* serialPort, const unsigned char* waitFor, int waitForLength, int timeoutMilliseconds)
{
    return 0;
}

int serialPortGetByteCountAvailableToReadCompat(uins_device_t* device, serial_port_t* serialPort)
{
    return 0;
}

int serialPortGetByteCountAvailableToWriteCompat(uins_device_t* device, serial_port_t* serialPort)
{
    return 0;
}

int serialPortSleepCompat(uins_device_t* device, serial_port_t* serialPort, int sleepMilliseconds)
{
    return 0;
}

///////////////////////////////////////////////////////////////////////////////

int bootloadFileCompat(serial_port_t* port, const char* fileName, const char* bootName, const void* obj, pfnBootloadProgress uploadProgress, pfnBootloadProgress verifyProgress)
{
    return 0;
}

int bootloadFileExCompat(bootload_params_t* params)
{
    return 0;
}

int bootloadUpdateBootloaderCompat(serial_port_t* port, const char* fileName, const void* obj, pfnBootloadProgress uploadProgress, pfnBootloadProgress verifyProgress)
{
    return 0;
}

int bootloadUpdateBootloaderExCompat(bootload_params_t* p)
{
    return 0;
}

int bootloadGetBootloaderVersionFromFileCompat(const char* bootName, int* verMajor, char* verMinor)
{
    return 0;
}

int enableBootloaderCompat(serial_port_t* port, int baudRate, char* error, int errorLength, const char* bootloadEnableCmd)
{
    return 0;
}

int disableBootloaderCompat(serial_port_t* port, char* error, int errorLength)
{
    return 0;
}

int bootloaderCycleBaudRateCompat(int baudRate)
{
    return 0;
}

int bootloaderClosestBaudRateCompat(int baudRate)
{
    return 0;
}

