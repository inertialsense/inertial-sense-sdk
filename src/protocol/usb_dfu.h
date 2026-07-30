/**
 * @file usb_dfu.h
 * @brief Protocol definitions for the USB Device Firmware Update (DFU) class,
 *        as adapted from the OpenPCD project's USB DFU implementation. Covers
 *        the DFU functional descriptor, class-specific request codes, status/
 *        state codes, and related wire-protocol constants per the USB DFU
 *        Specification, Revision 1.1.
 *
 * @author Harald Welte <hwelte@hmw-consulting.de> (original OpenPCD implementation, 2006)
 * @copyright Copyright (c) 2006 Harald Welte. Licensed under the GNU General
 *            Public License, version 2 or later — see full license text below.
 */

#ifndef USB_DFU_H
#define USB_DFU_H
/* USB Device Firmware Update Implementation for OpenPCD
 * (C) 2006 by Harald Welte <hwelte@hmw-consulting.de>
 *
 * Protocol definitions for USB DFU
 *
 * This ought to be compliant to the USB DFU Spec 1.0 as available from
 * http://www.usb.org/developers/devclass_docs/usbdfu10.pdf
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <stdint.h>

#define USB_DT_DFU            0x21    //!< USB descriptor type value identifying a DFU functional descriptor

#ifdef _MSC_VER
# pragma pack(push)
# pragma pack(1)
#endif /* _MSC_VER */
/**
 * DFU functional descriptor, as defined in Section 4.1.3 of the USB DFU
 * Specification, Revision 1.1. Describes the DFU-specific capabilities of
 * the interface it is attached to.
 */
struct usb_dfu_func_descriptor {
    uint8_t                     bLength;            //!< size of this descriptor, in bytes (USB_DT_DFU_SIZE)
    uint8_t                     bDescriptorType;     //!< descriptor type (USB_DT_DFU)
    uint8_t                     bmAttributes;        //!< bitmask of DFU capability/behavior flags, see USB_DFU_* bits below
#define USB_DFU_CAN_DOWNLOAD    (1 << 0)    //!< bit 0: device supports DFU_DNLOAD (host-to-device firmware download)
#define USB_DFU_CAN_UPLOAD      (1 << 1)    //!< bit 1: device supports DFU_UPLOAD (device-to-host firmware upload)
#define USB_DFU_MANIFEST_TOL    (1 << 2)    //!< bit 2: device is manifestation-tolerant (can communicate after manifestation without a reset)
#define USB_DFU_WILL_DETACH     (1 << 3)    //!< bit 3: device will perform a bus detach-attach sequence on receipt of DFU_DETACH, host need not issue a USB reset
    uint16_t                    wDetachTimeOut;      //!< maximum time, in milliseconds, the device waits after a DFU_DETACH before giving up on the reset
    uint16_t                    wTransferSize;       //!< maximum number of bytes the device can accept per control-write transaction (DFU_DNLOAD/DFU_UPLOAD block size)
    uint16_t                    bcdDFUVersion;       //!< BCD-encoded version of the DFU specification this descriptor conforms to (e.g. 0x0110 for v1.1)
#ifdef _MSC_VER
    };
# pragma pack(pop)
#elif defined __GNUC__
# if defined __MINGW32__
    } __attribute__ ((__packed__, __gcc_struct__));
# else
} __attribute__ ((__packed__));
# endif
#else
    #warning "No way to pack struct on this compiler? This will break!"
#endif /* _MSC_VER */

#define USB_DT_DFU_SIZE            9    //!< size, in bytes, of a usb_dfu_func_descriptor

#define USB_TYPE_DFU            (LIBUSB_REQUEST_TYPE_CLASS|LIBUSB_RECIPIENT_INTERFACE)    //!< libusb request-type/recipient bits for a DFU class request targeting an interface

/* DFU class-specific requests (Section 3, DFU Rev 1.1) */
#define USB_REQ_DFU_DETACH          0x00    //!< requests the device leave run-time mode and enter DFU mode (or vice versa in DFU mode, per state)
#define USB_REQ_DFU_DNLOAD          0x01    //!< requests the device receive a block of firmware data from the host (host-to-device transfer)
#define USB_REQ_DFU_UPLOAD          0x02    //!< requests the device send a block of firmware data to the host (device-to-host transfer)
#define USB_REQ_DFU_GETSTATUS       0x03    //!< requests the device's current status (bStatus, bwPollTimeout, bState, iString)
#define USB_REQ_DFU_CLRSTATUS       0x04    //!< clears an error condition and returns the device to the dfuIDLE state
#define USB_REQ_DFU_GETSTATE        0x05    //!< requests the device's current state (a single dfu_state byte)
#define USB_REQ_DFU_ABORT           0x06    //!< aborts the current download/upload operation and returns the device to the dfuIDLE state

/* DFU_GETSTATUS bStatus values (Section 6.1.2, DFU Rev 1.1) */
#define DFU_STATUS_OK               0x00    //!< no error condition is present
#define DFU_STATUS_errTARGET        0x01    //!< file is not targeted for use by this device
#define DFU_STATUS_errFILE          0x02    //!< file failed a vendor-specific verification test
#define DFU_STATUS_errWRITE         0x03    //!< device is unable to write memory
#define DFU_STATUS_errERASE         0x04    //!< memory erase function failed
#define DFU_STATUS_errCHECK_ERASED  0x05    //!< memory erase check failed
#define DFU_STATUS_errPROG          0x06    //!< program memory function failed
#define DFU_STATUS_errVERIFY        0x07    //!< programmed memory failed verification
#define DFU_STATUS_errADDRESS       0x08    //!< cannot program memory due to a received address that is out of range
#define DFU_STATUS_errNOTDONE       0x09    //!< received DFU_DNLOAD with wLength = 0, but device does not think it has all of the data yet
#define DFU_STATUS_errFIRMWARE      0x0a    //!< device's firmware is corrupt; it cannot return to run-time (non-DFU) operations
#define DFU_STATUS_errVENDOR        0x0b    //!< iString indicates a vendor-specific error
#define DFU_STATUS_errUSBR          0x0c    //!< device detected an unexpected USB reset
#define DFU_STATUS_errPOR           0x0d    //!< device detected an unexpected power-on reset
#define DFU_STATUS_errUNKNOWN       0x0e    //!< something went wrong, but the device does not know what it was
#define DFU_STATUS_errSTALLEDPKT    0x0f    //!< device stalled an unexpected request

/**
 * DFU device state machine states, as defined in Section 6.1.2 / Figure A.1
 * of the USB DFU Specification, Revision 1.1. Reported in the bState field
 * returned by DFU_GETSTATUS and DFU_GETSTATE.
 */
enum dfu_state {
    DFU_STATE_appIDLE               = 0,    //!< device is running its normal application; DFU capable but not yet requested
    DFU_STATE_appDETACH             = 1,    //!< device is running its normal application, has received DFU_DETACH, and is waiting for a USB reset
    DFU_STATE_dfuIDLE               = 2,    //!< device is in DFU mode, idle, and waiting for a download/upload request
    DFU_STATE_dfuDNLOAD_SYNC        = 3,    //!< device has received a download block and is synchronizing with the host before continuing
    DFU_STATE_dfuDNBUSY             = 4,    //!< device is busy programming a received download block into memory and cannot communicate
    DFU_STATE_dfuDNLOAD_IDLE        = 5,    //!< device has programmed a block and is idle, waiting for more DFU_DNLOAD data
    DFU_STATE_dfuMANIFEST_SYNC      = 6,    //!< device has received the final download block and is waiting for DFU_GETSTATUS before entering manifestation
    DFU_STATE_dfuMANIFEST           = 7,    //!< device is in the manifestation phase, programming the received image into its final location
    DFU_STATE_dfuMANIFEST_WAIT_RST  = 8,    //!< manifestation is complete but the device is not manifestation-tolerant and is waiting for a USB reset
    DFU_STATE_dfuUPLOAD_IDLE        = 9,    //!< device is in DFU mode, idle, with an upload in progress and waiting for further DFU_UPLOAD requests
    DFU_STATE_dfuERROR              = 10    //!< device has encountered an error and is waiting for a DFU_CLRSTATUS request
};

/* USB string descriptor should contain max 126 UTF-16 characters
 * but 253 would even accomodate any UTF-8 encoding */
#define MAX_DESC_STR_LEN 253    //!< maximum length, in bytes, of a USB string descriptor buffer used when reading DFU-related string descriptors

#endif /* USB_DFU_H */
