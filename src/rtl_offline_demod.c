/*
 * rtl-sdr, turns your Realtek RTL2832 based DVB dongle into a SDR receiver
 * Copyright (C) 2012 by Steve Markgraf <steve@steve-m.de>
 * Copyright (C) 2012 by Hoernchen <la@tfc-server.de>
 * Copyright (C) 2012 by Kyle Keen <keenerd@gmail.com>
 * Copyright (C) 2013 by Elias Oenal <EliasOenal@gmail.com>
 * Copyright (C) 2015 by Hayati Ayguen <h_ayguen@web.de>
 * Copyright (C) 2021 by Ahmet Genç <ahmetgenc93@gmail.com>
 * Copyright (C) 2021 by Omer Faruk Kirli <omerfarukkirli@gmail.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
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

#include <errno.h>
#include <signal.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>

#ifndef _WIN32
#include <unistd.h>
#else
#include <windows.h>
#include <fcntl.h>
#include <io.h>
#include "getopt/getopt.h"
#define usleep(x) Sleep(x/1000)
#if defined(_MSC_VER) && (_MSC_VER < 1900)
#define snprintf _snprintf
#endif
#if defined(_MSC_VER) && (_MSC_VER < 1800)
#define round(x) (x > 0.0 ? floor(x + 0.5): ceil(x - 0.5))
#endif
#define _USE_MATH_DEFINES
#endif

#include <math.h>

#include <rtl-sdr.h>
#include <rtl_app_ver.h>
#include "convenience/convenience.h"
#include "convenience/rtl_convenience.h"
#include "convenience/wavewrite.h"
#include "demod.h"

#define ARRAY_LEN(x)	(sizeof(x)/sizeof(x[0]))

static volatile int do_exit = 0;
static int verbosity = 0;
static int writeMpx = 0;    /* 0 : writes audio */

static struct demod_state dm;
static uint32_t dongle_buf_len = 512;    /* dongle.buf_len */
static uint32_t dm_rate = 0;


void usage(int verbosity)
{
	fprintf(stderr,
		"rtl_offline_demod, an offline demodulator without the downsampling from capture-rate\n"
		"rtl_offline_demod  version %d.%d %s (%s))\n",
		APP_VER_MAJOR, APP_VER_MINOR, APP_VER_ID, __DATE__
		);
	fprintf(stderr,
		"Usage:\trtl_offline_demod [-options]\n"
		"\t[-v increase verbosity (default: 0)]\n"
		"\t[-M modulation (default: fm)]\n"
		"\t	fm or nbfm or nfm, wbfm or wfm, mwfm, am, ook, usb, lsb\n"
		"\t	wbfm == -M fm -s 170k -A fast -r 32k -E deemp\n"
		"\t	ook == -M am -E adc,  ook == -M am without adc option\n"
		"\t[-s sample_rate (default: 24k)]\n"
		"\t[-r resample_rate (default: none / same as -s)]\n"

		"\t[-W length of single buffer in units of 512 samples (default: 32 was 256)]\n"
		"\t[-c de-emphasis_time_constant in us for wbfm. 'us' or 'eu' for 75/50 us (default: us)]\n"
		"\t[-E enable_option (default: none)]\n"
		"\t	use multiple -E to enable multiple options\n"
		"\t	adc:    enable dc blocking filter on demodulated audio\n"
		"\t	deemp:  enable de-emphasis filter\n"
		"\t[-q dc_avg_factor for option rdc (default: 9)]\n"
		"Experimental options:\n"
		"\t[-A std/fast/lut/ale choose atan math (default: std)]\n"
		"\n"
		);
	exit(1);
}

#ifdef _WIN32
BOOL WINAPI
sighandler(int signum)
{
	if (CTRL_C_EVENT == signum) {
		fprintf(stderr, "Signal caught, exiting!\n");
		do_exit = 1;
		return TRUE;
	}
	return FALSE;
}
#else
static void sighandler(int signum)
{
	if (signum == SIGPIPE) {
		/* fprintf(stderr, "Signal %d caught, trying to continue.\n", signum); */
		return;
	}
	fprintf(stderr, "Signal %d caught, exiting!\n", signum);
	do_exit = 1;
}
#endif


#if defined(_MSC_VER) && (_MSC_VER < 1800)
static double log2(double n)
{
	return log(n) / log(2.0);
}
#endif


int full_demod(struct demod_state *d)
{
	int nwritten, errFlags = 0;	/* return is bitmask of error flags: value 1: mpx error; 2: audio error; 4: cbb error */

	/* downsample_input(d); */

	d->mode_demod(d);  /* lowpassed -> result */

	if (writeMpx) {
		nwritten = (int)fwrite(d->result, 2, d->result_len, stdout);
		if (nwritten != d->result_len) {
			fprintf(stderr, "error writing %d samples of mpx-stream .. result %d\n", d->result_len, nwritten);
			errFlags = errFlags | 1;
		}
		/* no need for audio? */
		return errFlags;
	}

	/* use nicer filter here too? */
	if (d->deemph) {
		deemph_filter(d);}
	if (d->dc_block_audio) {
		dc_block_audio_filter(d->result, d->result_len, &(d->adc_avg), d->adc_block_const);}
	if (d->rate_out2 > 0) {
		low_pass_real(d);
		/* arbitrary_resample(d->result, d->result, d->result_len, d->result_len * d->rate_out2 / d->rate_out); */
	}

	nwritten = (int)fwrite(d->result, 2, d->result_len, stdout);
	if (nwritten != d->result_len) {
		fprintf(stderr, "error writing %d samples of aud-stream .. result %d\n", d->result_len, nwritten);
		errFlags = errFlags | 2;
	}
	return errFlags;
}


int read_from_file(FILE * f) {
	size_t nmemb = dongle_buf_len / ( 2 * sizeof(uint8_t) );
	unsigned char *buf = (unsigned char *)malloc(nmemb * sizeof(uint16_t));
#ifdef _WIN32
	if (f == stdin)
		_setmode(_fileno(f), _O_BINARY);
#endif

	const unsigned cbb_rate = dm.rate_out;
	const unsigned aud_rate = (dm.rate_out2 >= 0) ? (unsigned)dm.rate_out2 : cbb_rate;

	while (!do_exit && !feof(f)) {
		/* @todo: check/wait for a free buffer? */
		/* assume input is rate limited with pv -L */
		size_t rd = fread(buf, sizeof(uint16_t), nmemb, f);
		if (rd != nmemb)
			return 1;

		full_demod(&dm);
	}
	free(buf);
	return 0;
}


static void controller_fn()
{
	/* optimal_settings() */
	if (dm.mode_demod == &fm_demod) {
		dm.output_scale = 1;}

	fprintf(stderr, "Buffer size: %u Bytes == %u quadrature samples == %0.2fms\n",
		(unsigned)dongle_buf_len,
		(unsigned)dongle_buf_len / 2,
		1000 * 0.5 * (float)dongle_buf_len / (float)dm_rate);

	fprintf(stderr, "Output at %u Hz.\n", dm.rate_in/dm.post_downsample);
}


int main(int argc, char **argv)
{
#ifndef _WIN32
	struct sigaction sigact;
#endif
	int r, opt;
	int timeConstant = 75; /* default: U.S. 75 uS */
	struct demod_state *demod = &dm;
	/* demod_thread_init_ring_buffers(&dm_thr, MAXIMUM_BUF_LENGTH); */
	demod_init(demod, 1, 1);

	while ((opt = getopt(argc, argv, "s:r:E:A:M:c:W:hv")) != -1) {
		switch (opt) {
		case 's':
			demod->rate_in = (uint32_t)atofs(optarg);
			demod->rate_out = (uint32_t)atofs(optarg);
			break;
		case 'r':
			demod->rate_out2 = (int)atofs(optarg);
			break;
		case 'E':
			if (strcmp("adc", optarg) == 0) {
				demod->dc_block_audio = 1;}
			if (strcmp("deemp",  optarg) == 0) {
				demod->deemph = 1;}
			break;
		case 'A':
			if (strcmp("std",  optarg) == 0) {
				demod->custom_atan = 0;}
			if (strcmp("fast", optarg) == 0) {
				demod->custom_atan = 1;}
			if (strcmp("lut",  optarg) == 0) {
				atan_lut_init();
				demod->custom_atan = 2;}
			if (strcmp("ale", optarg) == 0) {
				demod->custom_atan = 3;}
			break;
		case 'M':
			if (strcmp("nbfm",  optarg) == 0 || strcmp("nfm",  optarg) == 0 || strcmp("fm",  optarg) == 0) {
				demod->mode_demod = &fm_demod;}
			if (strcmp("am",  optarg) == 0) {
				demod->mode_demod = &am_demod;
				demod->dc_block_audio = 1;	/* remove the DC */
			}
			if (strcmp("ook",  optarg) == 0) {
				demod->mode_demod = &am_demod;
				demod->dc_block_audio = 0;
			}
			if (strcmp("usb", optarg) == 0) {
				demod->mode_demod = &usb_demod;}
			if (strcmp("lsb", optarg) == 0) {
				demod->mode_demod = &lsb_demod;}
			if (strcmp("wbfm",  optarg) == 0 || strcmp("wfm",  optarg) == 0) {
				demod->mode_demod = &fm_demod;
				demod->rate_in = 170000;
				demod->rate_out = 170000;
				demod->rate_out2 = 32000;
				demod->custom_atan = 1;
				demod->deemph = 1;
			}
			break;
		case 'c':
			if (strcmp("us",  optarg) == 0)
				timeConstant = 75;
			else if (strcmp("eu", optarg) == 0)
				timeConstant = 50;
			else
				timeConstant = (int)atof(optarg);
			break;
		case 'v':
			++verbosity;
			break;
		case 'W':
			dongle_buf_len = 512 * atoi(optarg);
			if (dongle_buf_len > MAXIMUM_BUF_LENGTH) {
				fprintf(stderr, "Warning: limiting buffers from option -W to %d\n", MAXIMUM_BUF_LENGTH / 512);
				dongle_buf_len = MAXIMUM_BUF_LENGTH;
			}
			break;
		case 'h':
		case '?':
		default:
			usage(verbosity);
			break;
		}
	}

	if (demod->deemph) {
		double tc = (double)timeConstant * 1e-6;
		demod->deemph_a = (int)round(1.0/((1.0-exp(-1.0/(demod->rate_out * tc)))));
		if (verbosity)
			fprintf(stderr, "using wbfm deemphasis filter with time constant %d us\n", timeConstant );
	}

	if (verbosity)
		fprintf(stderr, "verbosity set to %d\n", verbosity);

	/* quadruple sample_rate to limit to Δθ to ±π/2 */
	demod->rate_in *= demod->post_downsample;

#ifndef _WIN32
	sigact.sa_handler = sighandler;
	sigemptyset(&sigact.sa_mask);
	sigact.sa_flags = 0;
	sigaction(SIGINT, &sigact, NULL);
	sigaction(SIGTERM, &sigact, NULL);
	sigaction(SIGQUIT, &sigact, NULL);
	sigaction(SIGPIPE, &sigact, NULL);
#else
	SetConsoleCtrlHandler( (PHANDLER_ROUTINE) sighandler, TRUE );
#endif

	/* controller_fn() */
	{
		/* optimal_settings() */
		if (dm.mode_demod == &fm_demod)
			dm.output_scale = 1;

		fprintf(stderr, "Buffer size: %u Bytes == %u quadrature samples == %0.2fms\n",
			(unsigned)dongle_buf_len,
			(unsigned)dongle_buf_len / 2,
			1000 * 0.5 * (float)dongle_buf_len / (float)dm_rate);

		fprintf(stderr, "Output at %u Hz.\n", dm.rate_in/dm.post_downsample);
	}

	r = read_from_file(stdin);

	if (do_exit) {
		fprintf(stderr, "\nUser cancel, exiting...\n");
	}
	else {
		fprintf(stderr, "\nEnd of input file or stream...\n");
	}

	/* dongle_cleanup(&dongle); */
	return r >= 0 ? r : -r;
}

/* vim: tabstop=8:softtabstop=8:shiftwidth=8:noexpandtab */
