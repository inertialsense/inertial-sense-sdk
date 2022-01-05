/*
 * DfuSe specific functions
 * 
 * This implements the ST Microsystems DFU extensions (DfuSe)
 * as per the DfuSe 1.1a specification (ST documents AN3156, AN2606)
 * The DfuSe file format is described in ST document UM0391.
 *
 * Copyright 2010-2018 Tormod Volden <debian.tormod@gmail.com>
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

#ifdef HAVE_CONFIG_H
# include "config.h"
#endif

#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>

#include "dfu_portable.h"
#include "dfu.h"
#include "usb_dfu.h"
#include "dfu_file.h"
#include "dfuse.h"
#include "dfuse_mem.h"
#include "quirks.h"

#include "../../../src/uins_log.h"

#define DFU_TIMEOUT 5000

static unsigned int quad2uint(unsigned char *p)
{
	return (*p + (*(p + 1) << 8) + (*(p + 2) << 16) + (*(p + 3) << 24));
}

static int dfuse_parse_options(const uins_device_context const * context, const char *options, struct dfu_config* config)
{
	char *end;
	const char *endword;
	unsigned int number;

	/* address, possibly empty, must be first */
	if (*options != ':') {
		endword = strchr(options, ':');
		if (!endword)
			endword = options + strlen(options); /* GNU strchrnul */

		number = strtoul(options, &end, 0);
		if (end == endword) {
			config->dfuse_address = number;
			config->dfuse_address_present = 1;
		} else {
			uinsLogError(context, EX_USAGE, "Invalid dfuse address");
			return EX_USAGE;
		}
		options = endword;
	}

	while (*options) {
		if (*options == ':') {
			options++;
			continue;
		}
		endword = strchr(options, ':');
		if (!endword)
			endword = options + strlen(options);

		if (!strncmp(options, "force", endword - options)) {
			config->dfuse_force++;
			options += 5;
			continue;
		}
		if (!strncmp(options, "leave", endword - options)) {
			config->dfuse_leave = 1;
			options += 5;
			continue;
		}
		// if (!strncmp(options, "unprotect", endword - options)) {
		// 	config->dfuse_unprotect = 1;
		// 	options += 9;
		// 	continue;
		// }
		// if (!strncmp(options, "mass-erase", endword - options)) {
		// 	config->dfuse_mass_erase = 1;
		// 	options += 10;
		// 	continue;
		// }
		if (!strncmp(options, "will-reset", endword - options)) {
			config->dfuse_will_reset = 1;
			options += 10;
			continue;
		}

		/* any valid number is interpreted as upload length */
		number = strtoul(options, &end, 0);
		if (end == endword) {
			config->dfuse_length = number;
		} else {
			uinsLogError(context, 0, "Invalid dfuse modifier");
			uinsLogDebug(context, "Invalid dfuse modifier: %s", options);
			return EX_USAGE;
		}
		options = endword;
	}
}

/* DFU_UPLOAD request for DfuSe 1.1a */
static int dfuse_upload(
	struct dfu_if *dif,
	const unsigned short length,
	unsigned char *data,
	unsigned short transaction
)
{
	int status;

	status = libusb_control_transfer(dif->dev_handle,
		 /* bmRequestType */	 LIBUSB_ENDPOINT_IN |
					 LIBUSB_REQUEST_TYPE_CLASS |
					 LIBUSB_RECIPIENT_INTERFACE,
		 /* bRequest      */	 DFU_UPLOAD,
		 /* wValue        */	 transaction,
		 /* wIndex        */	 dif->interface,
		 /* Data          */	 data,
		 /* wLength       */	 length,
					 DFU_TIMEOUT);
	if (status < 0) {
		// warnx("dfuse_upload: libusb_control_transfer returned %d (%s)", status, libusb_error_name(status));
	}
	return status;
}

/* DFU_DNLOAD request for DfuSe 1.1a */
static int dfuse_download(
	struct dfu_if *dif,
	const unsigned short length,
	unsigned char *data,
	unsigned short transaction
)
{
	int status;

	status = libusb_control_transfer(dif->dev_handle,
		 /* bmRequestType */	 LIBUSB_ENDPOINT_OUT |
					 LIBUSB_REQUEST_TYPE_CLASS |
					 LIBUSB_RECIPIENT_INTERFACE,
		 /* bRequest      */	 DFU_DNLOAD,
		 /* wValue        */	 transaction,
		 /* wIndex        */	 dif->interface,
		 /* Data          */	 data,
		 /* wLength       */	 length,
					 DFU_TIMEOUT);

	// if (status < 0) {
	// 	/* Silently fail on leave request on some unpredictable devices */
	// 	if ((dif->quirks & QUIRK_DFUSE_LEAVE) && !length && !data && transaction == 2)
	// 		return status;
	// 	// warnx("dfuse_download: libusb_control_transfer returned %d (%s)", status, libusb_error_name(status));
	// }

	return status;
}

/* DfuSe only commands */
/* Leaves the device in dfuDNLOAD-IDLE state */
static int dfuse_special_command(
	const uins_device_context const * context,
	struct dfu_config* config,
	struct dfu_if *dif,
	unsigned int address,
	enum dfuse_command command
)
{
	const char* dfuse_command_name[] = { "SET_ADDRESS" , "ERASE_PAGE",
					     "MASS_ERASE", "READ_UNPROTECT"};
	unsigned char buf[5];
	int length;
	int ret;
	struct dfu_status dst;
	int firstpoll = 1;
	int zerotimeouts = 0;
	int polltimeout = 0;
	int stalls = 0;

	if (command == ERASE_PAGE) {
		struct memsegment *segment;
		int page_size;

		segment = find_segment(dif->mem_layout, address);
		if (!segment || !(segment->memtype & DFUSE_ERASABLE)) {
			uinsLogError(context, EX_USAGE, "Page can not be erased");
			uinsLogDebug(context, "Page at 0x%08x can not be erased", address);
			return EX_USAGE;
		}
		page_size = segment->pagesize;
		
		uinsLogDebug(context, "Erasing page size %i at address 0x%08x, page "
				"starting at 0x%08x\n", page_size, address,
				address & ~(page_size - 1));
		
		buf[0] = 0x41;	/* Erase command */
		length = 5;
		config->last_erased_page = address & ~(page_size - 1);
	} else if (command == SET_ADDRESS) {
		uinsLogDebug(context, "  Setting address pointer to 0x%08x\n", address);
		buf[0] = 0x21;	/* Set Address Pointer command */
		length = 5;
	} else if (command == MASS_ERASE) {
		buf[0] = 0x41;	/* Mass erase command when length = 1 */
		length = 1;
	} else if (command == READ_UNPROTECT) {
		buf[0] = 0x92;
		length = 1;
	} else {
		uinsLogError(context, 0, "Non-supported special command");
		uinsLogDebug(context, "Non-supported special command %d", command);
		return EX_SOFTWARE;
	}

	buf[1] = address & 0xff;
	buf[2] = (address >> 8) & 0xff;
	buf[3] = (address >> 16) & 0xff;
	buf[4] = (address >> 24) & 0xff;

	ret = dfuse_download(dif, length, buf, 0);
	if (ret < 0) {
		uinsLogError(context, 0, "Error during special command download");
		uinsLogDebug(context, "Error during special command \"%s\" download", dfuse_command_name[command]);
		return EX_IOERR;
	}
	do {
		ret = dfu_get_status(dif, &dst);
		/* Workaround for some STM32L4 bootloaders that report a too
		 * short poll timeout and may stall the pipe when we poll */
		if (ret == LIBUSB_ERROR_PIPE && polltimeout != 0 && stalls < 3) {
			dst.bState = DFU_STATE_dfuDNBUSY;
			stalls++;
			uinsLogDebug(context, "* Device stalled USB pipe, reusing last poll timeout\n");
		} else if (ret < 0) {
			uinsLogError(context, 0, "Error during special command get_status");
			uinsLogDebug(context, "Error during special command \"%s\" get_status", dfuse_command_name[command]);
			return EX_IOERR;
		} else {
			polltimeout = dst.bwPollTimeout;
		}
		if (firstpoll) {
			firstpoll = 0;
			if (dst.bState != DFU_STATE_dfuDNBUSY) {
				uinsLogDebug(context, "DFU state(%u) = %s, status(%u) = %s\n", dst.bState,
				       dfu_state_to_string(dst.bState), dst.bStatus,
				       dfu_status_to_string(dst.bStatus));
				uinsLogError(context, 0, "Wrong state after command download");
				uinsLogDebug(context, "Wrong state after command \"%s\" download", dfuse_command_name[command]);
				return EX_PROTOCOL;
			}
			/* STM32F405 lies about mass erase timeout */
			if (command == MASS_ERASE && dst.bwPollTimeout == 100) {
				polltimeout = 35000; /* Datasheet says up to 32 seconds */
				uinsLogDebug(context, "Setting timeout to 35 seconds\n");
			}
		}
		/* wait while command is executed */
		// uinsLogDebug(context, "   Poll timeout %i ms\n", polltimeout);
		milli_sleep(polltimeout);
		if (command == READ_UNPROTECT)
			return ret;
		/* Workaround for e.g. Black Magic Probe getting stuck */
		if (dst.bwPollTimeout == 0) {
			if (++zerotimeouts == 100)
			{
				uinsLogError(context, 0, "Device stuck after special command request");
				return EX_IOERR;
			}
		} else {
			zerotimeouts = 0;
		}
	} while (dst.bState == DFU_STATE_dfuDNBUSY);

	if (dst.bStatus != DFU_STATUS_OK) {
		uinsLogError(context, 0, "dfuse command failed");
		uinsLogDebug(context, "dfuse command %s not correctly executed", dfuse_command_name[command]);
		uinsLogDebug(context, "dfuse status %s", dfu_status_to_string(dst.bStatus));
		return EX_IOERR;
	}
	return ret;
}

/* returns number of bytes sent */
static int dfuse_dnload_chunk(
	const uins_device_context const * context,
	struct dfu_config* config,
	struct dfu_if *dif,
	unsigned char *data,
	int size,
	int transaction
)
{
	int bytes_sent;
	struct dfu_status dst;
	int ret;

	ret = dfuse_download(dif, size, size ? data : NULL, transaction);
	if (ret < 0) {
		uinsLogError(context, EX_IOERR, "Error during download");
		return ret;
	}
	bytes_sent = ret;

	do {
		ret = dfu_get_status(dif, &dst);
		if (ret < 0) {
			if (-9 == ret && config->dfuse_skip_get_status_after_download)
			{
				// option bytes fails get status but resets device
				return bytes_sent;
			}
			uinsLogError(context, EX_IOERR, "Error during download get_status");
			return ret;
		}
		milli_sleep(dst.bwPollTimeout);
	} while (dst.bState != DFU_STATE_dfuDNLOAD_IDLE &&
		 dst.bState != DFU_STATE_dfuERROR &&
		 dst.bState != DFU_STATE_dfuMANIFEST &&
		 !(config->dfuse_will_reset && (dst.bState == DFU_STATE_dfuDNBUSY)));

	if (dst.bState == DFU_STATE_dfuMANIFEST)
			uinsLogDebug(context, "Transitioning to dfuMANIFEST state\n");

	if (dst.bStatus != DFU_STATUS_OK) {
		uinsLogDebug(context, " failed!\n");
		uinsLogDebug(context, "DFU state(%u) = %s, status(%u) = %s\n", dst.bState,
		       dfu_state_to_string(dst.bState), dst.bStatus,
		       dfu_status_to_string(dst.bStatus));
		return -1;
	}
	return bytes_sent;
}

static void dfuse_do_leave(
	const uins_device_context const * context,
	struct dfu_config* config,
	struct dfu_if *dif
)
{
	uinsLogDebug(context, "Submitting leave request...\n");
	dfuse_download(dif, 0, NULL, 2);

	struct dfu_status dst;
	dfu_get_status(dif, &dst);
	uinsLogDebug(context, "state: %s\n", dfu_state_to_string(dst.bState));
	
	uinsLogDebug(context, "Resetting device...\n");
	int reset_error = libusb_reset_device(dif->dev_handle);
	if (reset_error)
	{
		uinsLogDebug(context, "Reset Failed: %d\n", libusb_error_name(reset_error));
	}
}

int dfuse_do_upload(
	const uins_device_context const * context,
	struct dfu_config* config,
	struct dfu_if *dif,
	int xfer_size,
	int fd,
	const char *dfuse_options
)
{
	int total_bytes = 0;
	int upload_limit = 0;
	unsigned char *buf;
	int transaction;
	int ret;

	buf = dfu_malloc(xfer_size);

	if (dfuse_options)
	{
		ret = dfuse_parse_options(context, dfuse_options, config);
		if (ret) return ret;
	}

	if (config->dfuse_length)
		upload_limit = config->dfuse_length;
	if (config->dfuse_address_present) {
		struct memsegment *mem_layout, *segment;

		mem_layout = parse_memory_layout((char *)dif->alt_name, config);
		if (!mem_layout)
			uinsLogError(context, EX_IOERR, "Failed to parse memory layout");
		if (dif->quirks & QUIRK_DFUSE_LAYOUT)
			fixup_dfuse_layout(dif, &mem_layout);

		segment = find_segment(mem_layout, config->dfuse_address);
		if (!config->dfuse_force &&
		    (!segment || !(segment->memtype & DFUSE_READABLE)))
		{
			uinsLogError(context, EX_USAGE, "Page is not readable");
			uinsLogDebug(context, "Page at 0x%08x is not readable", config->dfuse_address);
		}

		if (!upload_limit) {
			if (segment) {
				upload_limit = segment->end - config->dfuse_address + 1;
				uinsLogDebug(context, "Limiting upload to end of memory segment, "
				       "%i bytes\n", upload_limit);
			} else {
				/* unknown segment - i.e. "force" has been used */
				upload_limit = 0x4000;
				uinsLogDebug(context, "Limiting upload to %i bytes\n", upload_limit);
			}
		}
		dfuse_special_command(context, config, dif, config->dfuse_address, SET_ADDRESS);
		dfu_abort_to_idle(dif);
	} else {
		/* Boot loader decides the start address, unknown to us */
		/* Use a short length to lower risk of running out of bounds */
		if (!upload_limit) {
			uinsLogWarn(context, 0, "Unbound upload not supported on DfuSe devices");
			upload_limit = 0x4000;
		}
		uinsLogDebug(context, "Limiting default upload to %i bytes\n", upload_limit);
	}

	dfu_progress_bar(context, "Upload", 0, 1);

	transaction = 2;
	while (1) {
		int rc;

		/* last chunk can be smaller than original xfer_size */
		if (upload_limit - total_bytes < xfer_size)
			xfer_size = upload_limit - total_bytes;
		rc = dfuse_upload(dif, xfer_size, buf, transaction++);
		if (rc < 0) {
			ret = rc;
			goto out_free;
		}

		dfu_file_write_crc(fd, 0, buf, rc);
		total_bytes += rc;

		if (total_bytes < 0)
		{
			uinsLogError(context, EX_SOFTWARE, "Received too many bytes");
			return EX_SOFTWARE;
		}

		if (rc < xfer_size || total_bytes >= upload_limit) {
			/* last block, return successfully */
			ret = 0;
			break;
		}
		dfu_progress_bar(context, "Upload", total_bytes, upload_limit);
	}

	dfu_progress_bar(context, "Upload", total_bytes, total_bytes);

	if (config->dfuse_leave)
	{
		dfu_abort_to_idle(dif);
		dfuse_do_leave(context, config, dif);
	}

 out_free:
	free(buf);

	return ret;
}

/* Writes an element of any size to the device, taking care of page erases */
/* returns 0 on success, otherwise -EINVAL */
int dfuse_dnload_element(
	const uins_device_context const * context,
	struct dfu_config* config,
	struct dfu_if *dif,
	unsigned int dwElementAddress,
	unsigned int dwElementSize,
	unsigned char *data,
	int xfer_size
)
{
	int p;
	int ret;
	struct memsegment *segment;

	/* Check at least that we can write to the last address */
	segment = find_segment(dif->mem_layout, dwElementAddress + dwElementSize - 1);
	if (!config->dfuse_force && (!segment || !(segment->memtype & DFUSE_WRITEABLE)))
	{
		uinsLogError(context, EX_USAGE, "Last page is not writeable");
		uinsLogDebug(context, "Last page at 0x%08x is not writeable", dwElementAddress + dwElementSize - 1);
	}

	// dfu_progress_bar(context, "Erase   ", 0, 1);

	/* First pass: Erase involved pages if needed */
	for (p = 0; p < (int)dwElementSize; p += xfer_size) {
		int page_size;
		unsigned int erase_address;
		unsigned int address = dwElementAddress + p;
		int chunk_size = xfer_size;

		segment = find_segment(dif->mem_layout, address);
		if (!config->dfuse_force && (!segment || !(segment->memtype & DFUSE_WRITEABLE)))
		{
			uinsLogError(context, 0, "Page not writeable");
			uinsLogDebug(context, "Page at 0x%08x is not writeable", address);
			return EX_USAGE;
		}
		
		/* If the location is not in the memory map we skip erasing */
		/* since we wouldn't know the correct page size for flash erase */
		if (!segment)
			continue;

		page_size = segment->pagesize;

		/* check if this is the last chunk */
		if (p + chunk_size > (int)dwElementSize)
			chunk_size = dwElementSize - p;

		/* Erase only for flash memory downloads */
		if ((segment->memtype & DFUSE_ERASABLE) /* && !config->dfuse_mass_erase */) {
			/* erase all involved pages */
			for (erase_address = address;
			     erase_address < address + chunk_size;
			     erase_address += page_size)
			{
				if ((erase_address & ~(page_size - 1)) != config->last_erased_page)
				{
					dfuse_special_command(context, config, dif, erase_address, ERASE_PAGE);
				}
			}

			if (((address + chunk_size - 1) & ~(page_size - 1)) != config->last_erased_page)
			{
				uinsLogDebug(context, " Chunk extends into next page, erase it as well\n");
				dfuse_special_command(context, config, dif, address + chunk_size - 1, ERASE_PAGE);
			}
			
			// dfu_progress_bar(context, "Erase   ", p, dwElementSize);
		}
	}

	// dfu_progress_bar(context, "Erase   ", dwElementSize, dwElementSize);
	
	// dfu_progress_bar(context, "Download", 0, 1);

	/* Second pass: Write data to (erased) pages */
	for (p = 0; p < (int)dwElementSize; p += xfer_size) {
		dfu_progress_bar(context, "Download", p, dwElementSize);
	
		unsigned int address = dwElementAddress + p;
		int chunk_size = xfer_size;

		/* check if this is the last chunk */
		if (p + chunk_size > (int)dwElementSize)
			chunk_size = dwElementSize - p;

		if (context->interface->log_level) {
			uinsLogDebug(context, " Download from image offset "
			       "%08x to memory %08x-%08x, size %i\n",
			       p, address, address + chunk_size - 1,
			       chunk_size);
		}
				
		dfuse_special_command(context, config, dif, address, SET_ADDRESS);

		/* transaction = 2 for no address offset */
		ret = dfuse_dnload_chunk(context, config, dif, data + p, chunk_size, 2);
		if (ret != chunk_size) {
			uinsLogError(context, ret, "Failed to write chunk");
			uinsLogDebug(context, "Failed to write whole chunk: %i of %i bytes", ret, chunk_size);
			return -EINVAL;
		}
	}

	dfu_progress_bar(context, "Download", dwElementSize, dwElementSize);

	return 0;
}

static int
dfuse_memcpy(const uins_device_context const * context, unsigned char *dst, unsigned char **src, int *rem, int size)
{
	if (size > *rem) {
		uinsLogError(context, 0, "Corrupt DfuSe file");
		uinsLogDebug(context, "Corrupt DfuSe file:  Cannot read %d bytes from %d bytes", size, *rem);
		return EX_NOINPUT;
	}

	if (dst != NULL)
	{
		memcpy(dst, *src, size);
	}

	(*src) += size;
	(*rem) -= size;

	return 0;
}

/* Download raw binary file to DfuSe device */
static int dfuse_do_bin_dnload(
	const uins_device_context const * context,
	struct dfu_config* config,
	struct dfu_if *dif,
	int xfer_size,
	struct dfu_file *file,
	unsigned int start_address
)
{
	unsigned int dwElementAddress;
	unsigned int dwElementSize;
	unsigned char *data;
	int ret;

	dwElementAddress = start_address;
	dwElementSize = file->size.total - file->size.suffix - file->size.prefix;

	uinsLogDebug(context, "Downloading element to address = 0x%08x, size = %i\n",
	       dwElementAddress, dwElementSize);

	data = file->firmware + file->size.prefix;

	ret = dfuse_dnload_element(context, config, dif, dwElementAddress, dwElementSize, data, xfer_size);
	if (ret == 0)
	{
		uinsLogDebug(context, "File downloaded successfully\n");
	}

	return ret;
}

/* Parse a DfuSe file and download contents to device */
static int dfuse_do_dfuse_dnload(
	const uins_device_context const * context,
	struct dfu_config* config,
	struct dfu_if *dif,
	int xfer_size,
	struct dfu_file *file
)
{
	uint8_t dfuprefix[11];
	uint8_t targetprefix[274];
	uint8_t elementheader[8];
	int image;
	int element;
	int bTargets;
	int bAlternateSetting;
	struct dfu_if *adif;
	int dwNbElements;
	unsigned int dwElementAddress;
	unsigned int dwElementSize;
	uint8_t *data;
	int ret;
	int rem;
	int mem_copy_failed;
	int bFirstAddressSaved = 0;

	rem = file->size.total - file->size.prefix - file->size.suffix;
	data = file->firmware + file->size.prefix;

	/* Must be larger than a minimal DfuSe header and suffix */
	if (rem < (int)(sizeof(dfuprefix) + sizeof(targetprefix) + sizeof(elementheader)))
	{
		uinsLogError(context, 0, "File too small for a DfuSe file");
		return EX_DATAERR;
    }

	mem_copy_failed = dfuse_memcpy(context, dfuprefix, &data, &rem, sizeof(dfuprefix));
	if (mem_copy_failed)
	{
		return -EINVAL;
	}

	if (strncmp((char *)dfuprefix, "DfuSe", 5)) {
		uinsLogError(context, 0, "No valid DfuSe signature");
		return -EINVAL;
	}
	if (dfuprefix[5] != 0x01) {
		uinsLogError(context, EX_DATAERR, "DFU format revision not supported");
		uinsLogDebug(context, "DFU format revision %i not supported", dfuprefix[5]);
		return -EINVAL;
	}
	bTargets = dfuprefix[10];
	uinsLogDebug(context, "File contains %i DFU images\n", bTargets);

	for (image = 1; image <= bTargets; image++) {
		uinsLogDebug(context, "Parsing DFU image %i\n", image);
		
		mem_copy_failed = dfuse_memcpy(context, targetprefix, &data, &rem, sizeof(targetprefix));
		if (mem_copy_failed)
		{
			return -EINVAL;
		}
		
		if (strncmp((char *)targetprefix, "Target", 6))
		{
			uinsLogError(context, EX_DATAERR, "No valid target signature");
			return -EINVAL;
		}
		
		bAlternateSetting = targetprefix[6];
		if (targetprefix[7])
			uinsLogDebug(context, "Target name: %s\n", &targetprefix[11]);
		else
			uinsLogDebug(context, "No target name\n");
		dwNbElements = quad2uint((unsigned char *)targetprefix + 270);
		uinsLogDebug(context, "Image for alternate setting %i, ", bAlternateSetting);
		uinsLogDebug(context, "(%i elements, ", dwNbElements);
		uinsLogDebug(context, "total size = %i)\n",
		       quad2uint((unsigned char *)targetprefix + 266));

		adif = dif;
		while (adif) {
			if (bAlternateSetting == adif->altsetting) {
				adif->dev_handle = dif->dev_handle;
				uinsLogDebug(context, "Setting Alternate Interface #%d ...\n", adif->altsetting);
				ret = libusb_set_interface_alt_setting(
					  adif->dev_handle,
					  adif->interface, adif->altsetting);
				if (ret < 0) {
					uinsLogError(context, ret, "Cannot set alternate interface");
					return EX_IOERR;
				}
				break;
			}
			adif = adif->next;
		}

		// if (!adif)
		// 	warnx("No alternate setting %d (skipping elements)", bAlternateSetting);

		for (element = 1; element <= dwNbElements; element++) {
			uinsLogDebug(context, "Parsing element %i, ", element);

			mem_copy_failed = dfuse_memcpy(context, elementheader, &data, &rem, sizeof(elementheader));
			if (mem_copy_failed)
			{
				return -EINVAL;
			}

			dwElementAddress =
			    quad2uint((unsigned char *)elementheader);
			dwElementSize =
			    quad2uint((unsigned char *)elementheader + 4);
			uinsLogDebug(context, "address = 0x%08x, ", dwElementAddress);
			uinsLogDebug(context, "size = %i\n", dwElementSize);

			if (!bFirstAddressSaved) {
				bFirstAddressSaved = 1;
				config->dfuse_address = dwElementAddress;
			}
			/* sanity check */
			if ((int)dwElementSize > rem)
			{
				uinsLogError(context, EX_DATAERR, "File too small for element size");
				return EX_DATAERR;
			}

			if (adif)
				ret = dfuse_dnload_element(context, config, adif, dwElementAddress, dwElementSize, data, xfer_size);
			else
				ret = 0;

			/* advance read pointer */
			mem_copy_failed = dfuse_memcpy(context, NULL, &data, &rem, dwElementSize);
			if (mem_copy_failed)
			{
				return -EINVAL;
			}

			if (ret != 0)
				return ret;
		}
	}

	if (rem != 0)
		uinsLogDebug(context, "%d bytes leftover", rem);

	uinsLogDebug(context, "Done parsing DfuSe file\n");

	return 0;
}

int dfuse_do_dnload(
	const uins_device_context const * context,
	struct dfu_config* config,
	struct dfu_if *dif,
	int xfer_size,
	struct dfu_file *file,
	const char *dfuse_options
)
{
	int ret;
	struct dfu_if *adif;

	if (dfuse_options)
	{
		dfuse_parse_options(context, dfuse_options, config);
	}

	adif = dif;
	while (adif) {
		adif->mem_layout = parse_memory_layout((char *)adif->alt_name, config);
		if (!adif->mem_layout)
		{
			uinsLogError(context, 0, "Failed to parse memory layout for alternate interface");
			uinsLogDebug(context, "Failed to parse memory layout for alternate interface %i", adif->altsetting);
			return EX_IOERR;
		}

		if (adif->quirks & QUIRK_DFUSE_LAYOUT)
		{
			fixup_dfuse_layout(adif, &(adif->mem_layout));
		}

		adif = adif->next;
	}

	// if (config->dfuse_unprotect) {
	// 	if (!config->dfuse_force) {
	// 		uinsLogError(context, EX_USAGE, "The read unprotect command will erase the flash memory and can only be used with force\n");
	// 	}
	// 	ret = dfuse_special_command(config, dif, 0, READ_UNPROTECT);
	// 	uinsLogDebug(context, "Device disconnects, erases flash and resets now\n");
	// 	return ret;
	// }

	// if (config->dfuse_mass_erase) {
	// 	if (!config->dfuse_force) {
	// 		uinsLogError(interface, user_data, EX_USAGE, "The mass erase command "
	// 			"can only be used with force");
	// 	}
	// 	uinsLogDebug(interface, "Performing mass erase, this can take a moment\n");
	// 	ret = dfuse_special_command(config, dif, 0, MASS_ERASE);
	// }

	if (!file->name) {
		uinsLogDebug(context, "DfuSe command mode\n");
		ret = 0;
	} else if (config->dfuse_address_present) {
		if (file->bcdDFU == 0x11a) {
			uinsLogError(context, EX_USAGE, "This is a DfuSe file, not meant for raw download");
		}
		ret = dfuse_do_bin_dnload(context, config, dif, xfer_size, file, config->dfuse_address);
	} else {
		if (file->bcdDFU != 0x11a) {
			uinsLogDebug(context, "file->bcdDFU: %d\n", file->bcdDFU);
			uinsLogError(context, EX_USAGE, "Only DfuSe file version 1.1a is supported for raw binary download, use the dfuse address");
			return EX_USAGE;
		}
		ret = dfuse_do_dfuse_dnload(context, config, dif, xfer_size, file);
	}

	adif = dif;
	while (adif)
	{
		free_segment_list(adif->mem_layout);
		adif = adif->next;
	}

	if (!config->dfuse_will_reset)
	{
		dfu_abort_to_idle(dif);
	}

	if (config->dfuse_leave)
	{
		dfuse_do_leave(context, config, dif);
	}

	return ret;
}

/* Check if we have one interface, possibly multiple alternate interfaces */
int dfuse_multiple_alt(struct dfu_if *dfu_root)
{
	libusb_device *dev = dfu_root->dev;
	uint8_t configuration = dfu_root->configuration;
	uint8_t interface = dfu_root->interface;
	struct dfu_if *dif = dfu_root->next;

	while (dif) {
		if (dev != dif->dev ||
		    configuration != dif->configuration ||
		    interface != dif->interface)
			return 0;
		dif = dif->next;
	}
	return 1;
}
