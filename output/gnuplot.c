/*
 * This file is part of the libsigrok project.
 *
 * Copyright (C) 2010 Uwe Hermann <uwe@hermann-uwe.de>
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
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301 USA
 */

#include <stdlib.h>
#include <string.h>
#include <glib.h>
#include "config.h" /* Needed for PACKAGE_STRING and others. */
#include "libsigrok.h"
#include "libsigrok-internal.h"

#define LOG_PREFIX "output/gnuplot"

struct context {
	unsigned int num_enabled_channels;
	uint64_t samplerate;
	uint64_t samplecount;
	gboolean header_done;
	uint8_t *prevsample;
	int *channel_index;
};

static const char *gnuplot_header = "\
# Sample data in space-separated columns format usable by gnuplot.\n";
static const char *gnuplot_header2 = "\
#\n# Column\tChannel\n\
# -----------------------------------------------------------------------------\n\
# 0\t\tSample counter (for internal gnuplot purposes)\n";


static int init(struct sr_output *o)
{
	struct context *ctx;
	struct sr_channel *ch;
	GSList *l;
	unsigned int i;

	if (!o || !o->sdi)
		return SR_ERR_ARG;

	ctx = g_malloc0(sizeof(struct context));
	o->internal = ctx;
	ctx->num_enabled_channels = 0;
	for (l = o->sdi->channels; l; l = l->next) {
		ch = l->data;
		if (ch->type != SR_CHANNEL_LOGIC)
			continue;
		if (!ch->enabled)
			continue;
		ctx->num_enabled_channels++;
	}
	if (ctx->num_enabled_channels <= 0) {
		sr_err("No logic channel enabled.");
		return SR_ERR;
	}
	ctx->channel_index = g_malloc(sizeof(int) * ctx->num_enabled_channels);

	/* Once more to map the enabled channels. */
	for (i = 0, l = o->sdi->channels; l; l = l->next) {
		ch = l->data;
		if (ch->type != SR_CHANNEL_LOGIC)
			continue;
		if (!ch->enabled)
			continue;
		ctx->channel_index[i++] = ch->index;
	}

	return SR_OK;
}

static GString *gen_header(struct sr_output *o)
{
	struct context *ctx;
	struct sr_channel *ch;
	GVariant *gvar;
	GString *header;
	time_t t;
	unsigned int num_channels, i;
	char *samplerate_s;

	ctx = o->internal;
	if (ctx->samplerate == 0) {
		if (sr_config_get(o->sdi->driver, o->sdi, NULL, SR_CONF_SAMPLERATE,
				&gvar) == SR_OK) {
			ctx->samplerate = g_variant_get_uint64(gvar);
			g_variant_unref(gvar);
		}
	}

	t = time(NULL);
	header = g_string_sized_new(512);
	g_string_printf(header, "%s", gnuplot_header);
	g_string_append_printf(header, "# Generated by %s on %s",
			PACKAGE_STRING, ctime(&t));

	num_channels = g_slist_length(o->sdi->channels);
	g_string_append_printf(header, "# Acquisition with %d/%d channels",
			ctx->num_enabled_channels, num_channels);
	if (ctx->samplerate != 0) {
		samplerate_s = sr_samplerate_string(ctx->samplerate);
		g_string_append_printf(header, " at %s", samplerate_s);
		g_free(samplerate_s);
	}
	g_string_append_printf(header, "\n");

	g_string_append_printf(header, "%s", gnuplot_header2);

	/* Columns / channels */
	for (i = 0; i < ctx->num_enabled_channels; i++) {
		ch = g_slist_nth_data(o->sdi->channels, ctx->channel_index[i]);
		g_string_append_printf(header, "# %d\t\t%s\n", i + 1, ch->name);
	}

	return header;
}

static int receive(struct sr_output *o, const struct sr_datafeed_packet *packet,
		GString **out)
{
	const struct sr_datafeed_meta *meta;
	const struct sr_datafeed_logic *logic;
	const struct sr_config *src;
	GSList *l;
	struct context *ctx;
	const uint8_t *sample;
	unsigned int curbit, p, idx, i;

	*out = NULL;
	if (!o || !o->internal)
		return SR_ERR_BUG;
	ctx = o->internal;

	if (packet->type == SR_DF_META) {
		meta = packet->payload;
		for (l = meta->config; l; l = l->next) {
			src = l->data;
			if (src->key != SR_CONF_SAMPLERATE)
				continue;
			ctx->samplerate = g_variant_get_uint64(src->data);
		}
	}

	if (packet->type != SR_DF_LOGIC)
		return SR_OK;
	logic = packet->payload;

	if (!ctx->prevsample) {
		/* Can't allocate this until we know the stream's unitsize. */
		ctx->prevsample = g_malloc0(logic->unitsize);
	}

	if (!ctx->header_done) {
		*out = gen_header(o);
		ctx->header_done = TRUE;
	} else {
		*out = g_string_sized_new(512);
	}

	for (i = 0; i <= logic->length - logic->unitsize; i += logic->unitsize) {
		sample = logic->data + i;
		ctx->samplecount++;

		/*
		 * Don't output the same sample multiple times, but make
		 * sure to output at least the first and last sample.
		 */
		if (i > 0 && i < logic->length - logic->unitsize) {
			if (!memcmp(sample, ctx->prevsample, logic->unitsize))
				continue;
		}
		memcpy(ctx->prevsample, sample, logic->unitsize);

		/* The first column is a counter (needed for gnuplot). */
		g_string_append_printf(*out, "%" PRIu64 "\t", ctx->samplecount);

		/* The next columns are the values of all channels. */
		for (p = 0; p < ctx->num_enabled_channels; p++) {
			idx = ctx->channel_index[p];
			curbit = (sample[idx / 8] & ((uint8_t) (1 << (idx % 8)))) >> (idx % 8);
			g_string_append_printf(*out, "%d ", curbit);
		}
		g_string_append_printf(*out, "\n");
	}

	return SR_OK;
}

static int cleanup(struct sr_output *o)
{
	struct context *ctx;

	if (!o || !o->internal)
		return SR_ERR_BUG;
	ctx = o->internal;
	g_free(ctx->channel_index);
	g_free(ctx->prevsample);
	g_free(ctx);

	return SR_OK;
}

SR_PRIV struct sr_output_format output_gnuplot = {
	.id = "gnuplot",
	.description = "Gnuplot",
	.init = init,
	.receive = receive,
	.cleanup = cleanup,
};
