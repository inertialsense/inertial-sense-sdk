/*
 * DFU transfer routines
 *
 * This is supposed to be a general DFU implementation, as specified in the
 * USB DFU 1.0 and 1.1 specification.
 *
 * The code was originally intended to interface with a USB device running the
 * "sam7dfu" firmware (see http://www.openpcd.org/) on an AT91SAM7 processor.
 *
 * Copyright 2007-2008 Harald Welte <laforge@gnumonks.org>
 * Copyright 2013 Hans Petter Selasky <hps@bitfrost.no>
 * Copyright 2014-2016 Tormod Volden <debian.tormod@gmail.com>
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

#define __USE_MINGW_ANSI_STDIO 1
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>

#include <libusb.h>

#include "dfu_portable.h"
#include "dfu.h"
#include "usb_dfu.h"
#include "dfu_file.h"
#include "dfu_load.h"
#include "quirks.h"

int dfuload_do_upload(struct dfu_if *dif, int xfer_size,
    int expected_size, int fd)
{
	off_t total_bytes = 0;
	unsigned short transaction = 0;
	unsigned char *buf;
	int ret;

	buf = dfu_malloc(xfer_size);

	printf("Copying data from DFU device to PC\n");

	while (1) {
		int rc;
		dfu_progress_bar("Upload", total_bytes, expected_size);
		rc = dfu_upload(dif->dev_handle, dif->interface,
				xfer_size, transaction++, buf);
		if (rc < 0) {
			warnx("\nError during upload (%s)",
			      libusb_error_name(rc));
			ret = rc;
			break;
		}

		dfu_file_write_crc(fd, 0, buf, rc);
		total_bytes += rc;

		if (total_bytes < 0)
			errx(EX_SOFTWARE, "\nReceived too many bytes (wraparound)");

		if (rc < xfer_size) {
			/* last block, return */
			ret = 0;
			break;
		}
	}
	free(buf);
	if (ret == 0) {
		dfu_progress_bar("Upload", total_bytes, total_bytes);
	} else {
		dfu_progress_bar("Upload", total_bytes, expected_size);
		printf("\n");
	}
	if (total_bytes == 0)
		printf("\nFailed.\n");
	else
		printf("Received a total of %lli bytes\n", (long long) total_bytes);

	if (expected_size != 0 && total_bytes != expected_size)
		warnx("Unexpected number of bytes uploaded from device");
	return ret;
}

int dfuload_do_dnload(struct dfu_if *dif, int xfer_size, struct dfu_file *file)
{
	off_t bytes_sent;
	off_t expected_size;
	unsigned char *buf;
	unsigned short transaction = 0;
	struct dfu_status dst;
	int ret;

	printf("Copying data from PC to DFU device\n");

	buf = file->firmware;
	expected_size = file->size.total - file->size.suffix;
	bytes_sent = 0;

	dfu_progress_bar("Download", 0, 1);
	while (bytes_sent < expected_size) {
		off_t bytes_left;
		int chunk_size;

		bytes_left = expected_size - bytes_sent;
		if (bytes_left < xfer_size)
			chunk_size = (int) bytes_left;
		else
			chunk_size = xfer_size;

		ret = dfu_download(dif->dev_handle, dif->interface,
				   chunk_size, transaction++, chunk_size ? buf : NULL);
		if (ret < 0) {
			warnx("Error during download (%s)",
			      libusb_error_name(ret));
			goto out;
		}
		bytes_sent += chunk_size;
		buf += chunk_size;

		do {
			ret = dfu_get_status(dif, &dst);
			if (ret < 0) {
				errx(EX_IOERR, "Error during download get_status (%s)",
				     libusb_error_name(ret));
				goto out;
			}

			if (dst.bState == DFU_STATE_dfuDNLOAD_IDLE ||
					dst.bState == DFU_STATE_dfuERROR)
				break;

			/* Wait while device executes flashing */
			milli_sleep(dst.bwPollTimeout);
			if (verbose > 1)
				fprintf(stderr, "Poll timeout %i ms\n", dst.bwPollTimeout);

		} while (1);

		if (dst.bStatus != DFU_STATUS_OK) {
			printf(" failed!\n");
			printf("DFU state(%u) = %s, status(%u) = %s\n", dst.bState,
				dfu_state_to_string(dst.bState), dst.bStatus,
				dfu_status_to_string(dst.bStatus));
			ret = -1;
			goto out;
		}
		dfu_progress_bar("Download", bytes_sent, bytes_sent + bytes_left);
	}

	/* send one zero sized download request to signalize end */
	ret = dfu_download(dif->dev_handle, dif->interface, 0, transaction, NULL);
	if (ret < 0) {
		errx(EX_IOERR, "Error sending completion packet (%s)",
		     libusb_error_name(ret));
		goto out;
	}

	dfu_progress_bar("Download", bytes_sent, bytes_sent);

	if (verbose)
		printf("Sent a total of %lli bytes\n", (long long) bytes_sent);

get_status:
	/* Transition to MANIFEST_SYNC state */
	ret = dfu_get_status(dif, &dst);
	if (ret < 0) {
		warnx("unable to read DFU status after completion (%s)",
		      libusb_error_name(ret));
		goto out;
	}
	printf("DFU state(%u) = %s, status(%u) = %s\n", dst.bState,
		dfu_state_to_string(dst.bState), dst.bStatus,
		dfu_status_to_string(dst.bStatus));

	milli_sleep(dst.bwPollTimeout);

	/* FIXME: deal correctly with ManifestationTolerant=0 / WillDetach bits */
	switch (dst.bState) {
	case DFU_STATE_dfuMANIFEST_SYNC:
	case DFU_STATE_dfuMANIFEST:
		/* some devices (e.g. TAS1020b) need some time before we
		 * can obtain the status */
		milli_sleep(1000);
		goto get_status;
		break;
	case DFU_STATE_dfuMANIFEST_WAIT_RST:
		printf("Resetting USB to switch back to runtime mode\n");
		ret = libusb_reset_device(dif->dev_handle);
		if (ret < 0 && ret != LIBUSB_ERROR_NOT_FOUND) {
			fprintf(stderr, "error resetting after download (%s)\n",
				libusb_error_name(ret));
		}
		break;
	case DFU_STATE_dfuIDLE:
		break;
	}
	printf("Done!\n");

out:
	return ret;
}
