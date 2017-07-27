/*
 * CDDL HEADER START
 *
 * This file and its contents are supplied under the terms of the
 * Common Development and Distribution License ("CDDL"), version 1.0.
 * You may only use this file in accordance with the terms of version
 * 1.0 of the CDDL.
 *
 * A full copy of the text of the CDDL should have accompanied this
 * source.  A copy of the CDDL is also available via the Internet at
 * http://www.illumos.org/license/CDDL.
 *
 * CDDL HEADER END
*/
/*
 * Copyright 2017 Saso Kiselkov. All rights reserved.
 */

#include <XPLMUtilities.h>

#include <acfutils/assert.h>
#include <acfutils/dr.h>
#include <acfutils/helpers.h>
#include <acfutils/intl.h>
#include <acfutils/log.h>
#include <acfutils/wav.h>

#include "msg.h"
#include "xplane.h"

typedef struct {
	const char	*const filename;
	wav_t		*wav;
} msg_info_t;

static msg_info_t msgs[MSG_NUM_MSGS] = {
	{ .filename = "plan_start.wav", .wav = NULL },
	{ .filename = "plan_end.wav", .wav = NULL },
	{ .filename = "driving_up.wav", .wav = NULL },
	{ .filename = "ready2conn.wav", .wav = NULL },
	{ .filename = "winch.wav", .wav = NULL },
	{ .filename = "connected.wav", .wav = NULL },
	{ .filename = "start_pb.wav", .wav = NULL },
	{ .filename = "start_tow.wav", .wav = NULL },
	{ .filename = "op_complete.wav", .wav = NULL },
	{ .filename = "disco.wav", .wav = NULL },
	{ .filename = "done_right.wav", .wav = NULL },
	{ .filename = "done_left.wav", .wav = NULL },
};

bool_t inited = B_FALSE;
static dr_t sound_on;
static dr_t radio_vol;

bool_t
msg_init(void)
{
	char *path;

	ASSERT(!inited);

	for (message_t msg = 0; msg < MSG_NUM_MSGS; msg++) {
		/* first try the localized version */
		path = mkpathname(bp_xpdir, bp_plugindir, "data", "msgs",
		    acfutils_xplang2code(XPLMGetLanguage()),
		    msgs[msg].filename, NULL);
		if (!file_exists(path, NULL)) {
			/* if that doesn't exist, try the English version */
			free(path);
			path = mkpathname(bp_xpdir, bp_plugindir, "data",
			    "msgs", "en", msgs[msg].filename, NULL);
		}
		msgs[msg].wav = wav_load(path, msgs[msg].filename);
		if (msgs[msg].wav == NULL) {
			logMsg("BetterPushback initialization error, unable "
			    "to load sound file %s", path);
			free(path);
			goto errout;
		}
		free(path);
	}

	fdr_find(&sound_on, "sim/operation/sound/sound_on");
	fdr_find(&radio_vol, "sim/operation/sound/radio_volume_ratio");

	inited = B_TRUE;

	return (B_TRUE);
errout:
	for (message_t msg = 0; msg < MSG_NUM_MSGS; msg++) {
		if (msgs[msg].wav != NULL) {
			wav_free(msgs[msg].wav);
			msgs[msg].wav = NULL;
		}
	}
	return (B_FALSE);
}

void
msg_fini(void)
{
	if (!inited)
		return;
	for (message_t msg = 0; msg < MSG_NUM_MSGS; msg++) {
		if (msgs[msg].wav != NULL) {
			wav_free(msgs[msg].wav);
			msgs[msg].wav = NULL;
		}
	}
	inited = B_FALSE;
}

void
msg_play(message_t msg)
{
	VERIFY3U(msg, <, MSG_NUM_MSGS);
	ASSERT(inited);
	if (dr_geti(&sound_on) == 0)
		return;
	wav_set_gain(msgs[msg].wav, dr_getf(&radio_vol));
	wav_play(msgs[msg].wav);
}

double
msg_dur(message_t msg)
{
	VERIFY3U(msg, <, MSG_NUM_MSGS);
	ASSERT(inited);
	return (msgs[msg].wav->duration);
}
