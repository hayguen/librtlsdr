#ifndef RTL_DEMOD_H
#define RTL_DEMOD_H

/*
 * rtl-sdr, turns your Realtek RTL2832 based DVB dongle into a SDR receiver
 * Copyright (C) 2012 by Steve Markgraf <steve@steve-m.de>
 * Copyright (C) 2012 by Hoernchen <la@tfc-server.de>
 * Copyright (C) 2012 by Kyle Keen <keenerd@gmail.com>
 * Copyright (C) 2013 by Elias Oenal <EliasOenal@gmail.com>
 * Copyright (C) 2015 by Hayati Ayguen <h_ayguen@web.de>
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


/*
 * written because people could not do real time
 * FM demod on Atom hardware with GNU radio
 * based on rtl_sdr.c and rtl_tcp.c and rtl_fm.c
 */

#include <stdio.h>
#include <stdint.h>
#include <pthread.h>

#define DEFAULT_BUF_LENGTH		(1 * 16384)
#define MAXIMUM_OVERSAMPLE		16
#define MAXIMUM_BUF_LENGTH		(MAXIMUM_OVERSAMPLE * DEFAULT_BUF_LENGTH)

/* forward declarations */
struct output_state;
struct cmd_state;


struct demod_state
{
	int	  exit_flag;
	pthread_t thread;
	int16_t  lowpassed[MAXIMUM_BUF_LENGTH];
	int	  lp_len;
	int16_t  lp_i_hist[10][6];
	int16_t  lp_q_hist[10][6];
	int16_t  result[MAXIMUM_BUF_LENGTH];
	int16_t  droop_i_hist[9];
	int16_t  droop_q_hist[9];
	int	  result_len;
	int	  rate_in;
	int	  rate_out;
	int	  rate_out2;
	int	  now_r, now_j;
	int	  pre_r, pre_j;
	int	  prev_index;
	int	  downsample;	/* min 1, max 256 */
	int	  post_downsample;
	int	  output_scale;
	int	  squelch_level, conseq_squelch, squelch_hits, terminate_on_squelch;
	int	  downsample_passes;
	int	  comp_fir_size;
	int	  custom_atan;
	int	  deemph, deemph_a;
	int	  now_lpr;
	int	  prev_lpr_index;
	int	  dc_block_audio, dc_avg, adc_block_const;
	int	  dc_block_raw, dc_avgI, dc_avgQ, rdc_block_const;
	void	 (*mode_demod)(struct demod_state*);
	pthread_rwlock_t rw;
	pthread_cond_t ready;
	pthread_mutex_t ready_m;
	struct output_state *output_target;
	struct cmd_state *cmd;
};

void demod_init(struct demod_state *s, struct output_state *output, struct cmd_state *cmd);
void demod_cleanup(struct demod_state *s);

void rotate16_neg90(int16_t *buf, uint32_t len);

void fifth_order(int16_t *data, int length, int16_t *hist);
void generic_fir(int16_t *data, int length, int *fir, int16_t *hist);
int rms(int16_t *samples, int len, int step, int omitDCfix);

int low_pass_simple(int16_t *signal2, int len, int step);

int atan_lut_init(void);

void fm_demod(struct demod_state *fm);
void am_demod(struct demod_state *fm);
void usb_demod(struct demod_state *fm);
void lsb_demod(struct demod_state *fm);
void raw_demod(struct demod_state *fm);

void low_pass(struct demod_state *d);
void low_pass_real(struct demod_state *s);
void deemph_filter(struct demod_state *fm);
void dc_block_audio_filter(struct demod_state *fm);
void dc_block_raw_filter(struct demod_state *fm, int16_t *buf, int len);


#define CIC_TABLE_MAX 10
extern int cic_9_tables[][10];


#endif

/* vim: tabstop=8:softtabstop=8:shiftwidth=8:noexpandtab */
