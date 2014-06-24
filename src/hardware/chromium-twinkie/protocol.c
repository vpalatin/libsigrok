/*
 * This file is part of the libsigrok project.
 *
 * Copyright 2014 Google, Inc
 *
 * This program is free software: you can redistribute it and/or modify
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
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <config.h>
#include <stdint.h>
#include <string.h>
#include <glib.h>
#include <glib/gstdio.h>
#include <libusb.h>
#include <stdio.h>
#include <errno.h>
#include <math.h>
#include <libsigrok/libsigrok.h>
#include "libsigrok-internal.h"
#include "protocol.h"

#define COMMAND_START_ACQUISITION	1
#define COMMAND_ABORT_ACQUISITION_ASYNC	2
#define COMMAND_ABORT_ACQUISITION_SYNC	0x7d

static int do_console_command(const struct sr_dev_inst *sdi,
			      const uint8_t *command, uint8_t cmd_len,
			      uint8_t *reply, uint8_t reply_len)
{
	uint8_t buf[64];
	struct sr_usb_dev_inst *usb;
	int ret, xfer;

	/* Temporary HACK */
	return SR_OK;

	usb = sdi->conn;

	if (cmd_len < 1 || cmd_len > 64 || reply_len > 64 ||
	    command == NULL || (reply_len > 0 && reply == NULL))
		return SR_ERR_ARG;

	ret = libusb_bulk_transfer(usb->devhdl, 2, buf, cmd_len, &xfer, 1000);
	if (ret != 0) {
		sr_dbg("Failed to send console command 0x%02x: %s.",
		       command[0], libusb_error_name(ret));
		return SR_ERR;
	}
	if (xfer != cmd_len) {
		sr_dbg("Failed to send console command: incorrect length "
		       "%d != %d.", xfer, cmd_len);
		return SR_ERR;
	}

	if (reply_len == 0)
		return SR_OK;

	ret = libusb_bulk_transfer(usb->devhdl, 0x80 | 1, buf, reply_len,
				   &xfer, 1000);
	if (ret != 0) {
		sr_dbg("Failed to receive reply to console command: %s.",
		       libusb_error_name(ret));
		return SR_ERR;
	}
	if (xfer != reply_len) {
		sr_dbg("Failed to receive reply to console command: "
		       "incorrect length %d != %d.", xfer, reply_len);
		return SR_ERR;
	}

	return SR_OK;
}

SR_PRIV int twinkie_start_acquisition(const struct sr_dev_inst *sdi)
{
	static const uint8_t command[1] = {
		COMMAND_START_ACQUISITION,
	};
	int ret;

	if ((ret = do_console_command(sdi, command, 1, NULL, 0)) != SR_OK)
		return ret;

	return SR_OK;
}

SR_PRIV int twinkie_init_device(const struct sr_dev_inst *sdi)
{
	(void)sdi;

	return SR_OK;
}

static void finish_acquisition(struct dev_context *devc)
{
	struct sr_datafeed_packet packet;
	struct sr_dev_inst *sdi = devc->cb_data;

	/* Terminate session. */
	packet.type = SR_DF_END;
	sr_session_send(devc->cb_data, &packet);

	/* Remove fds from polling. */
	usb_source_remove(sdi->session, devc->ctx);

	devc->num_transfers = 0;
	g_free(devc->transfers);
	g_free(devc->convbuffer);
}

static void free_transfer(struct libusb_transfer *transfer)
{
	struct dev_context *devc;
	unsigned int i;

	devc = transfer->user_data;

	g_free(transfer->buffer);
	transfer->buffer = NULL;
	libusb_free_transfer(transfer);

	for (i = 0; i < devc->num_transfers; i++) {
		if (devc->transfers[i] == transfer) {
			devc->transfers[i] = NULL;
			break;
		}
	}

	devc->submitted_transfers--;
	if (devc->submitted_transfers == 0)
		finish_acquisition(devc);
}

static void export_samples(struct dev_context *devc, size_t cnt)
{
	struct sr_datafeed_packet packet;
	struct sr_datafeed_logic logic;

	/* export the received data */
	packet.type = SR_DF_LOGIC;
	packet.payload = &logic;
	if (devc->limit_samples &&
	    cnt > devc->limit_samples - devc->sent_samples)
		cnt = devc->limit_samples - devc->sent_samples;
	logic.length = cnt;
	logic.unitsize = 1;
	logic.data = devc->convbuffer;
	sr_session_send(devc->cb_data, &packet);
	devc->sent_samples += cnt;
}

static void expand_sample_data(struct dev_context *devc,
			       const uint8_t *src, size_t srccnt)
{
	int i, f;
	size_t b;
	size_t rdy_samples, left_samples;
	int frames = srccnt / 64;

	for (f = 0; f < frames; f++) {
		int ch = (src[1] >> 4) & 3; /* samples channel number */
		int bit = 1 << ch; /* channel bit mask */
		struct cc_context *cc = devc->cc + ch;
		uint8_t *dest = devc->convbuffer + cc->idx;

		if (ch >= 2) /* only acquires CCx channels */
			continue;

		/* TODO: check timestamp, overflow, sequence number */

		/* skip header, go to edges data */
		src+=4;
		for (i = 0; i < 60; i++,src++)
			if (*src == cc->prev_src) {
				cc->rollbacks++;
			} else {
				uint8_t diff = *src - cc->prev_src;
				int fixup = cc->rollbacks && (((int)*src < (int)cc->prev_src) || (*src == 0xff));
				size_t total = (fixup ? cc->rollbacks - 1 : cc->rollbacks) * 256 + diff;

				if (total + cc->idx > devc->convbuffer_size) {
					sr_warn("overflow %d+%zd/%zd\n",
						cc->idx, total,
						devc->convbuffer_size);
					/* reset current decoding */
					cc->rollbacks = 0;
					break;
				}

				/* insert bits in the buffer */
				if (cc->level)
					for (b = 0 ; b < total ; b++, dest++)
						*dest |= bit;
				else
					dest += total;
				cc->idx += total;

				/* flip level on the next edge */
				cc->level = ~cc->level;

				cc->rollbacks = 0;
				cc->prev_src = *src;
			}
		/* expand repeated rollbacks */
		if (cc->rollbacks > 1) {
			size_t total = 256 * (cc->rollbacks - 1);
			if (total + cc->idx > devc->convbuffer_size) {
				sr_warn("overflow %d+%zd/%zd\n",
					cc->idx, total, devc->convbuffer_size);
				/* reset current decoding */
				total = 0;
			}
			/* insert bits in the buffer */
			if (cc->level)
				for (b = 0 ; b < total ; b++, dest++)
					*dest |= bit ;
			cc->idx += total;
			cc->rollbacks = 1;
		}
	}

	/* samples ready to be pushed (with both channels) */
	rdy_samples = MIN(devc->cc[0].idx, devc->cc[1].idx);
	left_samples = MAX(devc->cc[0].idx, devc->cc[1].idx) - rdy_samples;
	/* skip empty transfer */
	if (rdy_samples == 0)
		return;

	export_samples(devc, rdy_samples);

	/* clean up what we have sent */
	memmove(devc->convbuffer, devc->convbuffer + rdy_samples, left_samples);
	memset(devc->convbuffer + left_samples, 0, rdy_samples);
	devc->cc[0].idx -= rdy_samples;
	devc->cc[1].idx -= rdy_samples;
}

SR_PRIV void twinkie_receive_transfer(struct libusb_transfer *transfer)
{
	gboolean packet_has_error = FALSE;
	struct dev_context *devc;

	devc = transfer->user_data;

	/*
	 * If acquisition has already ended, just free any queued up
	 * transfer that come in.
	 */
	if (devc->sent_samples < 0) {
		free_transfer(transfer);
		return;
	}

	if (transfer->status || transfer->actual_length)
		sr_info("receive_transfer(): status %d received %d bytes.",
			transfer->status, transfer->actual_length);

	switch (transfer->status) {
	case LIBUSB_TRANSFER_NO_DEVICE:
		devc->sent_samples = -2;
		free_transfer(transfer);
		return;
	case LIBUSB_TRANSFER_COMPLETED:
	case LIBUSB_TRANSFER_TIMED_OUT: /* We may have received some data though. */
		break;
	default:
		packet_has_error = TRUE;
		break;
	}

	if (transfer->actual_length % 64) {
		sr_err("Bad USB packet size.");
		packet_has_error = TRUE;
	}

	if (transfer->actual_length == 0 || packet_has_error)
		goto resubmit;

	/* decode received edges */
	expand_sample_data(devc, transfer->buffer, transfer->actual_length);

	if (devc->limit_samples &&
			(uint64_t)devc->sent_samples >= devc->limit_samples) {
		devc->sent_samples = -2;
		free_transfer(transfer);
		return;
	}
resubmit:
	if (libusb_submit_transfer(transfer) != LIBUSB_SUCCESS)
		free_transfer(transfer);
}
