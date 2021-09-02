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
    const uins_device_uri uri
)
{
    uins_device_interface* interface = malloc(sizeof(uins_device_interface));

    // dfu://0483/df11/0/0x08000000

    char uri_scheme[5];
    unsigned int vendor_id;
    unsigned int product_id;
    unsigned int alt_id;
    unsigned int device_address;

    int uri_scan_status = sscanf(uri,
        "%5[^:]%*[:/]%x/%x/%u/%x",
        uri_scheme,
        &vendor_id,
        &product_id,
        &alt_id,
        &device_address);

    if(strncmp(uri_scheme, "sam", 3) == 0)
    {
        interface->uri_properties.scheme = IS_SCHEME_SAM;
    }
    else if(strncmp(uri_scheme, "dfu", 3) == 0)
    {
        interface->uri_properties.scheme = IS_SCHEME_DFU;
    }
    else if(strncmp(uri_scheme, "uart", 4) == 0)
    {
        interface->uri_properties.scheme = IS_SCHEME_UART;
    }

    interface->uri_properties.vid = vendor_id;
    interface->uri_properties.pid = product_id;
    interface->uri_properties.alt = alt_id;
    interface->uri_properties.address = device_address;
    
    return interface;
}

uins_operation_result uins_destroy_device_interface(uins_device_interface* interface)
{
    free(interface);

    return IS_OP_OK;
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
        // dfu://0483/df11/0/0x08000000
        
        config.match_vendor = 0x0483;
        // config.match_vendor_dfu;

        config.match_product = 0xdf11;
        // config.match_product_dfu;
        
        // config.match_serial_dfu;
        
        config.match_iface_alt_index = 0;
        
        config.dfuse_options = "0x08000000";

        config.verbose = 3; // more logs

        config.bin_file_path = "/home/sfusco/code/inertialsense/uins5/imx/cpp/hdw-src/research/stm32/bootloader_entry_test/Debug Nucleo/bootloader_entry_test.bin";
        
        int ok = bootloadFileExDfu(config);
        if (ok) {
            return IS_OP_OK;
        } else {
            return IS_OP_ERROR;
        }
    }

    // TODO: legacy SAM with serialPort and bootloader calls
}
