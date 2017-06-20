/*
 * CDDL HEADER START
 *
 * The contents of this file are subject to the terms of the
 * Common Development and Distribution License, Version 1.0 only
 * (the "License").  You may not use this file except in compliance
 * with the License.
 *
 * You can obtain a copy of the license in the file COPYING
 * or http://www.opensource.org/licenses/CDDL-1.0.
 * See the License for the specific language governing permissions
 * and limitations under the License.
 *
 * When distributing Covered Code, include this CDDL HEADER in each
 * file and include the License file COPYING.
 * If applicable, add the following below this CDDL HEADER, with the
 * fields enclosed by brackets "[]" replaced with your own identifying
 * information: Portions Copyright [yyyy] [name of copyright owner]
 *
 * CDDL HEADER END
 */
/*
 * Copyright 2017 Saso Kiselkov. All rights reserved.
 */

#ifndef	_BP_WAV_H_
#define	_BP_WAV_H_

#include <stdint.h>
#include <al.h>

#include "config.h"
#include "types.h"

#ifdef	__cplusplus
extern "C" {
#endif

typedef struct wav_fmt_hdr {
	uint16_t	datafmt;	/* PCM = 1 */
	uint16_t	n_channels;
	uint32_t	srate;		/* sample rate in Hz */
	uint32_t	byte_rate;	/* (srate * bps * #channels) / 8 */
	uint16_t	padding;	/* unused */
	uint16_t	bps;		/* bits per sample */
} wav_fmt_hdr_t;

typedef struct wav_s {
	char		*name;
	wav_fmt_hdr_t	fmt;
	double		duration;	/* in seconds */
	ALuint		albuf;
	ALuint		alsrc;
} wav_t;

#define	openal_set_shared_ctx	SYMBOL_PREFIX(openal_set_shared_ctx)
void openal_set_shared_ctx(bool_t flag);
#define	openal_init		SYMBOL_PREFIX(openal_init)
bool_t openal_init();
#define	openal_fini		SYMBOL_PREFIX(openal_fini)
void openal_fini();

#define	wav_load		SYMBOL_PREFIX(wav_load)
wav_t *wav_load(const char *filename, const char *descr_name);
#define	wav_free		SYMBOL_PREFIX(wav_free)
void wav_free(wav_t *wav);

#define	wav_set_gain		SYMBOL_PREFIX(wav_set_gain)
void wav_set_gain(wav_t *wav, float gain);
#define	wav_play		SYMBOL_PREFIX(wav_play)
bool_t wav_play(wav_t *wav);
#define	wav_stop		SYMBOL_PREFIX(wav_stop)
void wav_stop(wav_t *wav);

#ifdef	__cplusplus
}
#endif

#endif	/* _BP_WAV_H_ */
