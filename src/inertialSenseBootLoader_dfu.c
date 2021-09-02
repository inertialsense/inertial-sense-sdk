/*
 * dfu-util
 *
 * Copyright 2007-2008 by OpenMoko, Inc.
 * Copyright 2010-2012 Stefan Schmidt
 * Copyright 2013-2014 Hans Petter Selasky <hps@bitfrost.no>
 * Copyright 2010-2021 Tormod Volden
 *
 * Originally written by Harald Welte <laforge@openmoko.org>
 *
 * Based on existing code of dfu-programmer-0.4
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

#include "inertialSenseBootLoader_dfu.h"

int bootloadFileExDfu(struct dfu_config config)
{

/*

vid 0483
pid df11

Found DFU: [0483:df11] ver=2200, devnum=42, cfg=1, intf=0, path="1-3", alt=3, name="@Device Feature/0xFFFF0000/01*004 e", serial="205834865852"
Found DFU: [0483:df11] ver=2200, devnum=42, cfg=1, intf=0, path="1-3", alt=2, name="@OTP Memory /0x1FFF7000/01*0001Ke", serial="205834865852"
Found DFU: [0483:df11] ver=2200, devnum=42, cfg=1, intf=0, path="1-3", alt=1, name="@Option Bytes  /0x1FFF7800/01*040 e", serial="205834865852"
Found DFU: [0483:df11] ver=2200, devnum=42, cfg=1, intf=0, path="1-3", alt=0, name="@Internal Flash  /0x08000000/0256*0002Kg", serial="205834865852"

dfu://0483/df11/3

*/

	int expected_size = 0;
	unsigned int transfer_size = 0;
	struct dfu_status status;
	libusb_context *ctx;
	struct dfu_file file;
	char *end;
	int final_reset = 0;
	int wait_device = 0;
	int ret;
	int dfuse_device = 0;
	int fd;
	const char *dfuse_options = NULL;
	int detach_delay = 5;
	uint16_t runtime_vendor;
	uint16_t runtime_product;

	// create_dfu_config(&config);

	memset(&file, 0, sizeof(file));

	/* make sure all prints are flushed */
	setvbuf(stdout, NULL, _IONBF, 0);


	if (config.match_config_index == 0) {
		/* Handle "-c 0" (unconfigured device) as don't care */
		config.match_config_index = -1;
	}

	if (wait_device) {
		printf("Waiting for device, exit with ctrl-C\n");
	}

	ret = libusb_init(&ctx);
	if (ret)
		errx(EX_IOERR, "unable to initialize libusb: %s", libusb_error_name(ret));

	if (config.verbose > 2) {
#if defined(LIBUSB_API_VERSION) && LIBUSB_API_VERSION >= 0x01000106
		libusb_set_option(ctx, LIBUSB_OPTION_LOG_LEVEL, LIBUSB_LOG_LEVEL_DEBUG);
#else
		libusb_set_debug(ctx, 255);
#endif
	}
probe:
	probe_devices(ctx, &config);

	if (config.dfu_root == NULL) {
		if (wait_device) {
			milli_sleep(20);
			goto probe;
		} else {
			warnx("No DFU capable USB device available");
			libusb_exit(ctx);
		}
	} else if (file.bcdDFU == 0x11a && dfuse_multiple_alt(config.dfu_root)) {
		printf("Multiple alternate interfaces for DfuSe file\n");
	} else if (config.dfu_root->next != NULL) {
		/* We cannot safely support more than one DFU capable device
		 * with same vendor/product ID, since during DFU we need to do
		 * a USB bus reset, after which the target device will get a
		 * new address */
		errx(EX_IOERR, "More than one DFU capable USB device found! "
		       "Try `--list' and specify the serial number "
		       "or disconnect all but one device\n");
	}

	/* We have exactly one device. Its libusb_device is now in config.dfu_root->dev */

	printf("Opening DFU capable USB device...\n");
	ret = libusb_open(config.dfu_root->dev, &config.dfu_root->dev_handle);
	if (ret || !config.dfu_root->dev_handle)
		errx(EX_IOERR, "Cannot open device: %s", libusb_error_name(ret));

	printf("Device ID %04x:%04x\n", config.dfu_root->vendor, config.dfu_root->product);

	/* If first interface is DFU it is likely not proper run-time */
	if (config.dfu_root->interface > 0)
		printf("Run-Time device");
	else
		printf("Device");
	printf(" DFU version %04x\n",
	       libusb_le16_to_cpu(config.dfu_root->func_dfu.bcdDFUVersion));

	if (config.verbose) {
		printf("DFU attributes: (0x%02x)", config.dfu_root->func_dfu.bmAttributes);
		if (config.dfu_root->func_dfu.bmAttributes & USB_DFU_CAN_DOWNLOAD)
			printf(" bitCanDnload");
		if (config.dfu_root->func_dfu.bmAttributes & USB_DFU_CAN_UPLOAD)
			printf(" bitCanUpload");
		if (config.dfu_root->func_dfu.bmAttributes & USB_DFU_MANIFEST_TOL)
			printf(" bitManifestationTolerant");
		if (config.dfu_root->func_dfu.bmAttributes & USB_DFU_WILL_DETACH)
			printf(" bitWillDetach");
		printf("\n");
		printf("Detach timeout %d ms\n", libusb_le16_to_cpu(config.dfu_root->func_dfu.wDetachTimeOut));
	}

	/* Transition from run-Time mode to DFU mode */
	if (!(config.dfu_root->flags & DFU_IFF_DFU)) {
		int err;
		/* In the 'first round' during runtime mode, there can only be one
		* DFU Interface descriptor according to the DFU Spec. */

		/* FIXME: check if the selected device really has only one */

		runtime_vendor = config.dfu_root->vendor;
		runtime_product = config.dfu_root->product;

		printf("Claiming USB DFU (Run-Time) Interface...\n");
		ret = libusb_claim_interface(config.dfu_root->dev_handle, config.dfu_root->interface);
		if (ret < 0) {
			errx(EX_IOERR, "Cannot claim interface %d: %s",
				config.dfu_root->interface, libusb_error_name(ret));
		}

		/* Needed for some devices where the DFU interface is not the first,
		 * and should also be safe if there are multiple alt settings.
		 * Otherwise skip the request since it might not be supported
		 * by the device and the USB stack may or may not recover */
		if (config.dfu_root->interface > 0 || config.dfu_root->flags & DFU_IFF_ALT) {
			printf("Setting Alternate Interface zero...\n");
			ret = libusb_set_interface_alt_setting(config.dfu_root->dev_handle, config.dfu_root->interface, 0);
			if (ret < 0) {
				errx(EX_IOERR, "Cannot set alternate interface zero: %s", libusb_error_name(ret));
			}
		}

		printf("Determining device status...\n");
		err = dfu_get_status(config.dfu_root, &status);
		if (err == LIBUSB_ERROR_PIPE) {
			printf("Device does not implement get_status, assuming appIDLE\n");
			status.bStatus = DFU_STATUS_OK;
			status.bwPollTimeout = 0;
			status.bState  = DFU_STATE_appIDLE;
			status.iString = 0;
		} else if (err < 0) {
			errx(EX_IOERR, "error get_status: %s", libusb_error_name(err));
		} else {
			printf("DFU state(%u) = %s, status(%u) = %s\n", status.bState,
			       dfu_state_to_string(status.bState), status.bStatus,
			       dfu_status_to_string(status.bStatus));
		}
		milli_sleep(status.bwPollTimeout);

		switch (status.bState) {
		case DFU_STATE_appIDLE:
		case DFU_STATE_appDETACH:
			printf("Device really in Run-Time Mode, send DFU "
			       "detach request...\n");
			if (dfu_detach(config.dfu_root->dev_handle,
				       config.dfu_root->interface, 1000) < 0) {
				warnx("error detaching");
			}
			if (config.dfu_root->func_dfu.bmAttributes & USB_DFU_WILL_DETACH) {
				printf("Device will detach and reattach...\n");
			} else {
				printf("Resetting USB...\n");
				ret = libusb_reset_device(config.dfu_root->dev_handle);
				if (ret < 0 && ret != LIBUSB_ERROR_NOT_FOUND)
					errx(EX_IOERR, "error resetting "
						"after detach: %s", libusb_error_name(ret));
			}
			break;
		case DFU_STATE_dfuERROR:
			printf("dfuERROR, clearing status\n");
			if (dfu_clear_status(config.dfu_root->dev_handle,
					     config.dfu_root->interface) < 0) {
				errx(EX_IOERR, "error clear_status");
			}
			/* fall through */
		default:
			warnx("WARNING: Device already in DFU mode? (bState=%d %s)",
			      status.bState, dfu_state_to_string(status.bState));
			libusb_release_interface(config.dfu_root->dev_handle,
			    config.dfu_root->interface);
			goto dfustate;
		}
		libusb_release_interface(config.dfu_root->dev_handle,
					 config.dfu_root->interface);
		libusb_close(config.dfu_root->dev_handle);
		config.dfu_root->dev_handle = NULL;

		/* keeping handles open might prevent re-enumeration */
		disconnect_devices(&config);

		milli_sleep(detach_delay * 1000);

		/* Change match vendor and product to impossible values to force
		 * only DFU mode matches in the following probe */
		config.match_vendor = config.match_product = 0x10000;

		probe_devices(ctx, &config);

		if (config.dfu_root == NULL) {
			errx(EX_IOERR, "Lost device after RESET?");
		} else if (config.dfu_root->next != NULL) {
			errx(EX_IOERR, "More than one DFU capable USB device found! "
				"Try `--list' and specify the serial number "
				"or disconnect all but one device");
		}

		/* Check for DFU mode device */
		if (!(config.dfu_root->flags | DFU_IFF_DFU))
			errx(EX_PROTOCOL, "Device is not in DFU mode");

		printf("Opening DFU USB Device...\n");
		ret = libusb_open(config.dfu_root->dev, &config.dfu_root->dev_handle);
		if (ret || !config.dfu_root->dev_handle) {
			errx(EX_IOERR, "Cannot open device");
		}
	} else {
		/* we're already in DFU mode, so we can skip the detach/reset
		 * procedure */
		/* If a match vendor/product was specified, use that as the runtime
		 * vendor/product, otherwise use the DFU mode vendor/product */
		runtime_vendor = config.match_vendor < 0 ? config.dfu_root->vendor : config.match_vendor;
		runtime_product = config.match_product < 0 ? config.dfu_root->product : config.match_product;
	}

dfustate:
	printf("Claiming USB DFU Interface...\n");
	ret = libusb_claim_interface(config.dfu_root->dev_handle, config.dfu_root->interface);
	if (ret < 0) {
		errx(EX_IOERR, "Cannot claim interface - %s", libusb_error_name(ret));
	}

	if (config.dfu_root->flags & DFU_IFF_ALT) {
		printf("Setting Alternate Interface #%d ...\n", config.dfu_root->altsetting);
		ret = libusb_set_interface_alt_setting(config.dfu_root->dev_handle, config.dfu_root->interface, config.dfu_root->altsetting);
		if (ret < 0) {
			errx(EX_IOERR, "Cannot set alternate interface: %s", libusb_error_name(ret));
		}
	}

status_again:
	printf("Determining device status...\n");
	ret = dfu_get_status(config.dfu_root, &status );
	if (ret < 0) {
		errx(EX_IOERR, "error get_status: %s", libusb_error_name(ret));
	}
	printf("DFU state(%u) = %s, status(%u) = %s\n", status.bState,
	       dfu_state_to_string(status.bState), status.bStatus,
	       dfu_status_to_string(status.bStatus));

	milli_sleep(status.bwPollTimeout);

	switch (status.bState) {
	case DFU_STATE_appIDLE:
	case DFU_STATE_appDETACH:
		errx(EX_PROTOCOL, "Device still in Run-Time Mode!");
		break;
	case DFU_STATE_dfuERROR:
		printf("Clearing status\n");
		if (dfu_clear_status(config.dfu_root->dev_handle, config.dfu_root->interface) < 0) {
			errx(EX_IOERR, "error clear_status");
		}
		goto status_again;
		break;
	case DFU_STATE_dfuDNLOAD_IDLE:
	case DFU_STATE_dfuUPLOAD_IDLE:
		printf("Aborting previous incomplete transfer\n");
		if (dfu_abort(config.dfu_root->dev_handle, config.dfu_root->interface) < 0) {
			errx(EX_IOERR, "can't send DFU_ABORT");
		}
		goto status_again;
		break;
	case DFU_STATE_dfuIDLE:
	default:
		break;
	}

	if (DFU_STATUS_OK != status.bStatus ) {
		printf("WARNING: DFU Status: '%s'\n",
			dfu_status_to_string(status.bStatus));
		/* Clear our status & try again. */
		if (dfu_clear_status(config.dfu_root->dev_handle, config.dfu_root->interface) < 0)
			errx(EX_IOERR, "USB communication error");
		if (dfu_get_status(config.dfu_root, &status) < 0)
			errx(EX_IOERR, "USB communication error");
		if (DFU_STATUS_OK != status.bStatus)
			errx(EX_PROTOCOL, "Status is not OK: %d", status.bStatus);

		milli_sleep(status.bwPollTimeout);
	}

	printf("DFU mode device DFU version %04x\n",
	       libusb_le16_to_cpu(config.dfu_root->func_dfu.bcdDFUVersion));

	if (config.dfu_root->func_dfu.bcdDFUVersion == libusb_cpu_to_le16(0x11a))
		dfuse_device = 1;
	else if (dfuse_options)
		printf("Warning: DfuSe option used on non-DfuSe device\n");

	/* Get from device or user, warn if overridden */
	int func_dfu_transfer_size = libusb_le16_to_cpu(config.dfu_root->func_dfu.wTransferSize);
	if (func_dfu_transfer_size) {
		printf("Device returned transfer size %i\n", func_dfu_transfer_size);
		if (!transfer_size)
			transfer_size = func_dfu_transfer_size;
		else
			printf("Warning: Overriding device-reported transfer size\n");
	} else {
		if (!transfer_size)
			errx(EX_USAGE, "Transfer size must be specified");
	}

#ifdef __linux__
	/* limited to 4k in libusb Linux backend */
	if ((int)transfer_size > 4096) {
		transfer_size = 4096;
		printf("Limited transfer size to %i\n", transfer_size);
	}
#endif /* __linux__ */

	if (transfer_size < config.dfu_root->bMaxPacketSize0) {
		transfer_size = config.dfu_root->bMaxPacketSize0;
		printf("Adjusted transfer size to %i\n", transfer_size);
	}

	// "download" means copy from host to device
	// case MODE_DOWNLOAD:

	if (((file.idVendor  != 0xffff && file.idVendor  != runtime_vendor) ||
			(file.idProduct != 0xffff && file.idProduct != runtime_product)) &&
		((file.idVendor  != 0xffff && file.idVendor  != config.dfu_root->vendor) ||
			(file.idProduct != 0xffff && file.idProduct != config.dfu_root->product))) {
		errx(EX_USAGE, "Error: File ID %04x:%04x does "
			"not match device (%04x:%04x or %04x:%04x)",
			file.idVendor, file.idProduct,
			runtime_vendor, runtime_product,
			config.dfu_root->vendor, config.dfu_root->product);
	}
	if (dfuse_device || dfuse_options || file.bcdDFU == 0x11a) {
		ret = dfuse_do_dnload(config.dfu_root, transfer_size, &file, dfuse_options, &config);
	} else {
		ret = dfuload_do_dnload(config.dfu_root, transfer_size, &file, &config);
	}
	if (ret < 0)
		ret = EX_IOERR;
	else
		ret = EX_OK;
	
	// break; // MODE_DOWNLOAD

	if (!ret && final_reset) {
		ret = dfu_detach(config.dfu_root->dev_handle, config.dfu_root->interface, 1000);
		if (ret < 0) {
			/* Even if detach failed, just carry on to leave the
                           device in a known state */
			warnx("can't detach");
		}
		printf("Resetting USB to switch back to Run-Time mode\n");
		ret = libusb_reset_device(config.dfu_root->dev_handle);
		if (ret < 0 && ret != LIBUSB_ERROR_NOT_FOUND) {
			warnx("error resetting after download: %s", libusb_error_name(ret));
			ret = EX_IOERR;
		}
	}

	libusb_close(config.dfu_root->dev_handle);
	config.dfu_root->dev_handle = NULL;

	disconnect_devices(&config);
	libusb_exit(ctx);
	return ret;
}
