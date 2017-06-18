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

#include <stdio.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#include <alc.h>

#include "assert.h"
#include "list.h"
#include "log.h"
#include "riff.h"
#include "types.h"

#include "wav.h"

#define	WAVE_ID	FOURCC("WAVE")
#define	FMT_ID	FOURCC("fmt ")
#define	DATA_ID	FOURCC("data")

static ALCdevice *old_dev = NULL, *my_dev = NULL;
static ALCcontext *old_ctx = NULL, *my_ctx = NULL;
#ifdef	TEST_STANDALONE_BUILD
static bool_t use_shared = B_TRUE;
#else	/* !TEST_STANDALONE_BUILD */
static bool_t use_shared = B_FALSE;
#endif	/* !TEST_STANDALONE_BUILD */
static bool_t ctx_saved = B_FALSE;
static bool_t openal_inited = B_FALSE;

/*
 * ctx_save/ctx_restore must be used to bracket all OpenAL calls. This makes
 * sure private contexts are handled properly (when in use). If shared
 * contexts are used, these functions are no-ops.
 */
static bool_t
ctx_save(void)
{
	ALuint err;

	if (use_shared)
		return (B_TRUE);

	ASSERT(!ctx_saved);
	dbg_log(wav, 1, "ctx_save()");

	old_ctx = alcGetCurrentContext();
	if (old_ctx != NULL) {
		old_dev = alcGetContextsDevice(old_ctx);
		VERIFY(old_dev != NULL);
	} else {
		old_dev = NULL;
	}

	alcMakeContextCurrent(my_ctx);
	if ((err = alcGetError(my_dev)) != ALC_NO_ERROR) {
		dbg_log(wav, 0, "Error switching to my audio context (0x%x)",
		    err);
		return (B_FALSE);
	}

	ctx_saved = B_TRUE;

	return (B_TRUE);
}

static bool_t
ctx_restore(void)
{
	ALuint err;

	if (use_shared)
		return (B_TRUE);

	ASSERT(ctx_saved);
	/* To prevent tripping ASSERT in ctx_save if ctx_restore fails */
	ctx_saved = B_FALSE;

	dbg_log(wav, 1, "ctx_restore()");
	if (old_ctx != NULL) {
		alcMakeContextCurrent(old_ctx);
		VERIFY(old_dev != NULL);
		if ((err = alcGetError(old_dev)) != ALC_NO_ERROR) {
			dbg_log(wav, 0, "Error restoring shared audio "
			    "context (0x%x)", err);
			return (B_FALSE);
		}
	}

	return (B_TRUE);
}

void
xtcas_openal_set_shared_ctx(bool_t flag)
{
	ASSERT(!openal_inited);
	dbg_log(wav, 1, "xtcas_openal_set_shared_ctx = %d", flag);
	use_shared = flag;
}

bool_t
xtcas_openal_init(void)
{

	dbg_log(wav, 1, "xtcas_openal_init");

	ASSERT(!openal_inited);

	if (!ctx_save())
		return (B_FALSE);

	if (!use_shared || alcGetCurrentContext() == NULL) {
		ALuint err;

		my_dev = alcOpenDevice(NULL);
		if (my_dev == NULL) {
			dbg_log(wav, 0, "Cannot init audio system: "
			    "device open failed.");
			(void) ctx_restore();
			return (B_FALSE);
		}
		my_ctx = alcCreateContext(my_dev, NULL);
		if ((err = alcGetError(my_dev)) != ALC_NO_ERROR) {
			dbg_log(wav, 0, "Cannot init audio system: "
			    "create context failed (0x%x)", err);
			alcCloseDevice(my_dev);
			(void) ctx_restore();
			return (B_FALSE);
		}
		VERIFY(my_ctx != NULL);
		/* No current context, install our own */
		if (alcGetCurrentContext() == NULL) {
			ASSERT(use_shared);
			alcMakeContextCurrent(my_ctx);
			if ((err = alcGetError(my_dev)) != ALC_NO_ERROR) {
				dbg_log(wav, 0, "Error installing my audio "
				    "context (0x%x)", err);
				alcDestroyContext(my_ctx);
				alcCloseDevice(my_dev);
				return (B_FALSE);
			}
		}
	}

	if (!ctx_restore())
		return (B_FALSE);

	openal_inited = B_TRUE;

	return (B_TRUE);
}

void
xtcas_openal_fini()
{
	dbg_log(wav, 1, "xtcas_openal_fini");

	if (!openal_inited)
		return;

	if (!use_shared) {
		alcDestroyContext(my_ctx);
		alcCloseDevice(my_dev);
		my_ctx = NULL;
		my_dev = NULL;
	}
	openal_inited = B_FALSE;
}

/*
 * Loads a WAV file from a file and returns a buffered representation
 * ready to be passed to OpenAL. Currently we only support mono or
 * stereo raw PCM (uncompressed) WAV files.
 */
wav_t *
xtcas_wav_load(const char *filename, const char *descr_name)
{
	wav_t *wav = NULL;
	FILE *fp;
	size_t filesz;
	riff_chunk_t *riff = NULL;
	uint8_t *filebuf = NULL;
	riff_chunk_t *chunk;
	int sample_sz;
	ALuint err;
	ALfloat zeroes[3] = { 0.0, 0.0, 0.0 };

	ASSERT(openal_inited);

	dbg_log(wav, 1, "Loading wav file %s", filename);

	if ((fp = fopen(filename, "rb")) == NULL) {
		dbg_log(wav, 0, "Error loading WAV file \"%s\": "
		    "can't open file.", filename);
		return (NULL);
	}

	fseek(fp, 0, SEEK_END);
	filesz = ftell(fp);
	fseek(fp, 0, SEEK_SET);

	if ((wav = calloc(1, sizeof (*wav))) == NULL)
		goto errout;
	if ((filebuf = malloc(filesz)) == NULL)
		goto errout;
	if (fread(filebuf, 1, filesz, fp) != filesz)
		goto errout;
	if ((riff = riff_parse(WAVE_ID, filebuf, filesz)) == NULL) {
		dbg_log(wav, 0, "Error loading WAV file \"%s\": "
		    "file doesn't appear to be valid RIFF.", filename);
		goto errout;
	}

	wav->name = strdup(descr_name);

	chunk = riff_find_chunk(riff, FMT_ID, 0);
	if (chunk == NULL || chunk->datasz < sizeof (wav->fmt)) {
		dbg_log(wav, 0, "Error loading WAV file \"%s\": "
		    "file missing or malformed `fmt ' chunk.", filename);
		goto errout;
	}
	memcpy(&wav->fmt, chunk->data, sizeof (wav->fmt));
	if (riff->bswap) {
		wav->fmt.datafmt = BSWAP16(wav->fmt.datafmt);
		wav->fmt.n_channels = BSWAP16(wav->fmt.n_channels);
		wav->fmt.srate = BSWAP32(wav->fmt.srate);
		wav->fmt.byte_rate = BSWAP32(wav->fmt.byte_rate);
		wav->fmt.bps = BSWAP16(wav->fmt.bps);
	}

	/* format support check */
	if (wav->fmt.datafmt != 1 ||
	    (wav->fmt.n_channels != 1 && wav->fmt.n_channels != 2) ||
	    (wav->fmt.bps != 8 && wav->fmt.bps != 16)) {
		dbg_log(wav, 0, "Error loading WAV file \"%s\": "
		    "unsupported audio format.", filename);
		goto errout;
	}

	/*
	 * Check the DATA chunk is present and contains the correct number
	 * of samples.
	 */
	sample_sz = (wav->fmt.n_channels * wav->fmt.bps) / 8;
	chunk = riff_find_chunk(riff, DATA_ID, 0);
	if (chunk == NULL || (chunk->datasz & (sample_sz - 1)) != 0) {
		dbg_log(wav, 0, "Error loading WAV file %s: `data' chunk "
		    "missing or contains bad number of samples.", filename);
		goto errout;
	}

	wav->duration = ((double)(chunk->datasz / sample_sz)) / wav->fmt.srate;

	/* BSWAP the samples if necessary */
	if (riff->bswap && wav->fmt.bps == 16) {
		for (uint16_t *s = (uint16_t *)chunk->data;
		    (uint8_t *)s < chunk->data + chunk->datasz;
		    s++)
			*s = BSWAP16(*s);
	}

	if (!ctx_save())
		goto errout;

	alGenBuffers(1, &wav->albuf);
	if ((err = alGetError()) != AL_NO_ERROR) {
		dbg_log(wav, 0, "Error loading WAV file %s: "
		    "alGenBuffers failed (0x%x).", filename, err);
		(void) ctx_restore();
		goto errout;
	}
	if (wav->fmt.bps == 16)
		alBufferData(wav->albuf, wav->fmt.n_channels == 2 ?
		    AL_FORMAT_STEREO16 : AL_FORMAT_MONO16,
		    chunk->data, chunk->datasz, wav->fmt.srate);
	else
		alBufferData(wav->albuf, wav->fmt.n_channels == 2 ?
		    AL_FORMAT_STEREO8 : AL_FORMAT_MONO8,
		    chunk->data, chunk->datasz, wav->fmt.srate);

	if ((err = alGetError()) != AL_NO_ERROR) {
		dbg_log(wav, 0, "Error loading WAV file %s: "
		    "alBufferData failed (0x%x).", filename, err);
		(void) ctx_restore();
		goto errout;
	}

	alGenSources(1, &wav->alsrc);
	if ((err = alGetError()) != AL_NO_ERROR) {
		dbg_log(wav, 0, "Error loading WAV file %s: "
		    "alGenSources failed (0x%x).", filename, err);
		(void) ctx_restore();
		goto errout;
	}
#define	CHECK_ERROR(stmt) \
	do { \
		stmt; \
		if ((err = alGetError()) != AL_NO_ERROR) { \
			dbg_log(wav, 0, "Error loading WAV file %s, \"%s\" " \
			    "failed with error 0x%x", filename, #stmt, err); \
			alDeleteSources(1, &wav->alsrc); \
			VERIFY3S(alGetError(), ==, AL_NO_ERROR); \
			wav->alsrc = 0; \
			(void) ctx_restore(); \
			goto errout; \
		} \
	} while (0)
	CHECK_ERROR(alSourcei(wav->alsrc, AL_BUFFER, wav->albuf));
	CHECK_ERROR(alSourcef(wav->alsrc, AL_PITCH, 1.0));
	CHECK_ERROR(alSourcef(wav->alsrc, AL_GAIN, 1.0));
	CHECK_ERROR(alSourcei(wav->alsrc, AL_LOOPING, 0));
	CHECK_ERROR(alSourcefv(wav->alsrc, AL_POSITION, zeroes));
	CHECK_ERROR(alSourcefv(wav->alsrc, AL_VELOCITY, zeroes));

	(void) ctx_restore();

	dbg_log(wav, 1, "wav load complete, duration %.2fs", wav->duration);

	riff_free_chunk(riff);
	free(filebuf);
	fclose(fp);

	return (wav);

errout:
	if (filebuf != NULL)
		free(filebuf);
	if (riff != NULL) {
		char *dump = riff_dump(riff);
		dbg_log(wav, 0, "File format dump (for debugging):\n%s", dump);
		free(dump);
		riff_free_chunk(riff);
	}
	xtcas_wav_free(wav);
	fclose(fp);

	return (NULL);
}

/*
 * Destroys a WAV file as returned by xtcas_wav_load().
 */
void
xtcas_wav_free(wav_t *wav)
{
	if (wav == NULL)
		return;

	dbg_log(wav, 1, "xtcas_wav_free %s", wav->name);

	ASSERT(openal_inited);

	VERIFY(ctx_save());
	free(wav->name);
	if (wav->alsrc != 0) {
		alSourceStop(wav->alsrc);
		alDeleteSources(1, &wav->alsrc);
	}
	if (wav->albuf != 0)
		alDeleteBuffers(1, &wav->albuf);
	VERIFY(ctx_restore());

	free(wav);
}

/*
 * Sets the audio gain (volume) of a WAV file from 0.0 (silent) to 1.0
 * (full volume).
 */
void
xtcas_wav_set_gain(wav_t *wav, float gain)
{
	ALuint err;

	if (wav == NULL || wav->alsrc == 0)
		return;

	dbg_log(wav, 1, "xtcas_wav_set_gain %s %f", wav->name, (double)gain);

	ASSERT(openal_inited);

	VERIFY(ctx_save());

	alSourcef(wav->alsrc, AL_GAIN, gain);
	if ((err = alGetError()) != AL_NO_ERROR)
		dbg_log(wav, 0, "Error changing gain of WAV %s, error 0x%x.",
		    wav->name, err);

	VERIFY(ctx_restore());
}

/*
 * Starts playback of a WAV file loaded through xtcas_wav_load.
 * Playback volume is full (1.0) or the last value set by xtcas_wav_set_gain.
 */
bool_t
xtcas_wav_play(wav_t *wav)
{
	ALuint err;

	if (wav == NULL)
		return (B_FALSE);

	dbg_log(wav, 1, "xtcas_wav_play %s", wav->name);

	ASSERT(openal_inited);

	VERIFY(ctx_save());

	alSourcePlay(wav->alsrc);
	if ((err = alGetError()) != AL_NO_ERROR) {
		dbg_log(wav, 0, "Can't play sound: alSourcePlay "
		    "failed (0x%x).", err);
		VERIFY(ctx_restore());
		return (B_FALSE);
	}

	VERIFY(ctx_restore());

	return (B_TRUE);
}

/*
 * Stops playback of a WAV file started via xtcas_wav_play and
 * resets the playback position back to the start of the file.
 */
void
xtcas_wav_stop(wav_t *wav)
{
	ALuint err;

	if (wav == NULL)
		return;

	dbg_log(wav, 1, "xtcas_wav_stop %s", wav->name);

	if (wav->alsrc == 0)
		return;

	ASSERT(openal_inited);
	VERIFY(ctx_save());
	alSourceStop(wav->alsrc);
	if ((err = alGetError()) != AL_NO_ERROR) {
		dbg_log(wav, 0, "Can't stop sound, alSourceStop "
		    "failed (0x%x).", err);
	}
	VERIFY(ctx_restore());
}
