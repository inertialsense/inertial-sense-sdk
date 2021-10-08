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

uins_device_interface* uins_create_device_interface(
    uins_device device,
    const uins_device_uri uri
)
{
    uins_device_interface* interface = malloc(sizeof(uins_device_interface));

    interface->log_level = 1;

    // dfu://0483/df11/0/0x08000000

    char uri_scheme[5];
    unsigned int vendor_id;
    unsigned int product_id;
    unsigned int alt_id;
    char dfuse_address[11];

    int uri_scan_status = sscanf(uri,
        "%5[^:]%*[:/]%x/%x/%u/%s",
        uri_scheme,
        &vendor_id,
        &product_id,
        &alt_id,
        dfuse_address);

    if(strncmp(uri_scheme, "sam", 3) == 0)
    {
        interface->uri_properties.scheme = IS_SCHEME_SAM;
    }
    else if(strncmp(uri_scheme, "dfu", 3) == 0)
    {
        interface->uri_properties.scheme = IS_SCHEME_DFU;
        strcpy(interface->uri_properties.address, dfuse_address);
    }
    else if(strncmp(uri_scheme, "uart", 4) == 0)
    {
        interface->uri_properties.scheme = IS_SCHEME_UART;
    }
    else
    {
        interface->uri_properties.scheme = IS_SCHEME_UNKNOWN;
    }

    interface->uri_properties.vid = vendor_id;
    interface->uri_properties.pid = product_id;
    interface->uri_properties.alt = alt_id;
   
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
    pfnUinsDeviceInterfaceTaskProgress upload_progress_callback,
    pfnUinsDeviceInterfaceTaskProgress verify_progress_callback,
    const void* user_data
)
{
    if (interface->uri_properties.scheme == IS_SCHEME_DFU)
    {
        struct dfu_config config;
        create_dfu_config(&config);

        config.match_vendor = interface->uri_properties.vid;
        config.match_product = interface->uri_properties.pid;
        config.match_iface_alt_index = interface->uri_properties.alt;
        config.dfuse_options = interface->uri_properties.address;
        config.bin_file_path = firmware_file_path;

        uins_device_context context;
        context.interface = interface;
        context.user_data = user_data;
        context.progress_callback = upload_progress_callback;
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
    
    if (interface->uri_properties.scheme == IS_SCHEME_DFU)
    {
        // TODO: UART support
    }

    // TODO: legacy SAM with serialPort and bootloader calls
    return IS_OP_ERROR;
}
