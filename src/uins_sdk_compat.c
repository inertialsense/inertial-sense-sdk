#include "uins_sdk_compat.h"

#include "serialPort.h"
#include "serialPortPlatform.h"
#include "inertialSenseBootLoader.h"
#include "inertialSenseBootLoader_dfu.h"

uins_device uins_31()
{
    uins_device d;
    d.version_major = 3;
    d.version_minor = 1;
    d.bootloader_flash_support = IS_DEVICE_INTERFACE_FLAG_SAM;
    return d;
}

uins_device uins_40()
{
    uins_device d;
    d.version_major = 4;
    d.version_minor = 0;
    d.bootloader_flash_support = IS_DEVICE_INTERFACE_FLAG_SAM;
    return d;
}

uins_device uins_50()
{
    uins_device d;
    d.version_major = 3;
    d.version_minor = 1;
    d.bootloader_flash_support = IS_DEVICE_INTERFACE_FLAG_DFU | IS_DEVICE_INTERFACE_FLAG_UART;
    return d;
}

uins_device_interface* uins_create_device_interface(
    uins_device device,
    const uins_device_uri unique_identifier
)
{
    // TODO: uri parsing function
    // TODO: 
}

uins_operation_result uins_destroy_device_interface(uins_device_interface* interface)
{

}

uins_operation_result uins_open(uins_device_interface* interface)
{

}

uins_operation_result uins_close(uins_device_interface* interface)
{

}

uins_operation_result uins_read(uins_device_interface* interface, int read_count, uins_data_buffer buffer)
{

}

uins_operation_result uins_write(uins_device_interface* interface, int write_count, uins_data_buffer buffer)
{

}

uins_operation_result uins_update_flash(
    const uins_device_interface* interface,
    const char* firmware_file_path,
    uins_update_flash_style firmware_type,
    uins_verification_style verification_style,
    pfnUinsDeviceInterfaceError error_callback,
    pfnUinsDeviceInterfaceTaskProgress upload_progress_callback,
    pfnUinsDeviceInterfaceTaskProgress verify_progress_callback,
    const void* user_data
)
{
    if (interface->uri_properties.scheme & IS_DEVICE_INTERFACE_FLAG_DFU)
    {
        struct dfu_config config;
        create_dfu_config(&config);

        // TODO: setup up correct configuration using function parameters
        
        int ok = bootloadFileExDfu(config);
        if (ok) {
            return IS_OP_OK;
        } else {
            return IS_OP_ERROR;
        }
    }

    // TODO: legacy SAM with serialPort and bootloader calls
}
