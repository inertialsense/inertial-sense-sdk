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
    d.version_major = 5;
    d.version_minor = 0;
    d.bootloader_flash_support = IS_DEVICE_INTERFACE_FLAG_DFU | IS_DEVICE_INTERFACE_FLAG_UART;
    return d;
}

void uins_list_devices(uins_device_uri_list* list, uins_list_devices_callback_fn callback_fn)
{
    // TODO: backward compatibility with other urls
    // TODO: filter parameters

    // dfu://0483/df11/0/0x08000000
    uinsProbeDfuDevices(list, callback_fn);
}

uins_device_interface* uins_create_device_interface(
    uins_device device,
    const uins_device_uri uri
)
{
    uins_device_interface* interface = malloc(sizeof(uins_device_interface));

    interface->log_level = 1;

    // dfu://0483/df11/0/0x08000000

    char uri_scheme[5];
    char serial_number[IS_SN_MAX_SIZE_V5];

    int uri_scan_status = sscanf(uri,
        "%5[^:]%*[:/]%s",
        uri_scheme,
        serial_number);

    if(strncmp(uri_scheme, "sam", 3) == 0)
    {
        interface->uri_properties.scheme = IS_SCHEME_SAM;
    }
    else if(strncmp(uri_scheme, "dfu", 3) == 0)
    {
        interface->uri_properties.scheme = IS_SCHEME_DFU;
        strcpy(interface->uri_properties.serial_number, serial_number);
    }
    else if(strncmp(uri_scheme, "uart", 4) == 0)
    {
        interface->uri_properties.scheme = IS_SCHEME_UART;
    }
    else
    {
        interface->uri_properties.scheme = IS_SCHEME_UNKNOWN;
    }

    return interface;
}

uins_operation_result uins_change_log_level(uins_device_interface* interface, uins_device_interface_log_level log_level)
{
    if (log_level < 0 || log_level > 5)
    {
        return IS_OP_ERROR;
    }

    interface->log_level = log_level;
    return IS_OP_OK;
}

uins_operation_result uins_destroy_device_interface(uins_device_interface* interface)
{
    free(interface);

    return IS_OP_OK;
}

uins_operation_result uins_update_flash(
    const uins_device_interface* interface,
    const char* firmware_file_path,
    uins_update_flash_style firmware_type,
    uins_verification_style verification_style,
    pfnUinsDeviceInterfaceError error_callback,
    pfnUinsDeviceInterfaceTaskProgress update_progress_callback,
    pfnUinsDeviceInterfaceTaskProgress verify_progress_callback,
    const void* user_data
)
{
    if (!firmware_file_path)
    {
        return IS_OP_ERROR;
    }

    if (interface->uri_properties.scheme == IS_SCHEME_DFU)
    {
        // TODO: once we integrate the probe function dfu_config can go away 

        struct dfu_config config;
        create_dfu_config(&config);

        config.bin_file_path = firmware_file_path;
        config.match_vendor = "0483";
        config.match_product = "df11";
        config.match_iface_alt_index = 0;
        config.dfuse_options = "0x08000000";
        config.match_serial = interface->uri_properties.serial_number;
        config.match_serial_dfu = interface->uri_properties.serial_number;

        uins_device_context context;
        context.interface = interface;
        context.user_data = user_data;
        context.progress_callback = update_progress_callback;
        context.error_callback = error_callback;

        int ret = uinsBootloadFileExDfu(&context, config);
        if (ret == 0)
        {
            return IS_OP_OK;
        }
        else
        {
            return IS_OP_ERROR;
        }
    }
    else if (interface->uri_properties.scheme == IS_SCHEME_UART)
    {
        // TODO: UART support
    }

    // TODO: legacy SAM with serialPort and bootloader calls
    return IS_OP_ERROR;
}
