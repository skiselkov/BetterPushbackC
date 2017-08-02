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

#include <string.h>

#include <XPLMUtilities.h>

#include <acfutils/assert.h>
#include <acfutils/dr.h>
#include <acfutils/helpers.h>
#include <acfutils/icao2cc.h>
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
	{ .filename = "plan_start.opus", .wav = NULL },
	{ .filename = "plan_end.opus", .wav = NULL },
	{ .filename = "driving_up.opus", .wav = NULL },
	{ .filename = "ready2conn.opus", .wav = NULL },
	{ .filename = "winch.opus", .wav = NULL },
	{ .filename = "connected.opus", .wav = NULL },
	{ .filename = "start_pb.opus", .wav = NULL },
	{ .filename = "start_tow.opus", .wav = NULL },
	{ .filename = "op_complete.opus", .wav = NULL },
	{ .filename = "disco.opus", .wav = NULL },
	{ .filename = "done_right.opus", .wav = NULL },
	{ .filename = "done_left.opus", .wav = NULL }
};

bool_t inited = B_FALSE;
static dr_t sound_on;
static dr_t radio_vol;

bool_t
msg_init(const char *my_lang, const char *icao, lang_pref_t lang_pref)
{
	const char *arpt_cc;
	char msg_dir_name[8];
	char *path;
	bool_t isdir;

	ASSERT(!inited);

	switch (lang_pref) {
	case LANG_PREF_MATCH_REAL:
		arpt_cc = icao2cc(icao);
		break;
	case LANG_PREF_NATIVE:
		arpt_cc = NULL;
		break;
	default:
		VERIFY3U(lang_pref, ==, LANG_PREF_MATCH_ENGLISH);
		my_lang = "en";
		arpt_cc = icao2cc(icao);
		break;
	};

	if (arpt_cc != NULL) {
		snprintf(msg_dir_name, sizeof (msg_dir_name), "%s_%s",
		    my_lang, arpt_cc);
	} else {
		strlcpy(msg_dir_name, my_lang, sizeof (msg_dir_name));
	}

	/* First we try the exact lang_country combo. */
	path = mkpathname(bp_xpdir, bp_plugindir, "data", "msgs",
	    msg_dir_name, NULL);
	if (arpt_cc != NULL && (!file_exists(path, &isdir) || !isdir)) {
		/*
		 * If our language matches the local one, try a plain
		 * version of the language pack.
		 */
		const char *lang = cc2lang(arpt_cc);
		if (strcmp(my_lang, lang) == 0) {
			strlcpy(msg_dir_name, lang, sizeof (msg_dir_name));
			free(path);
			path = mkpathname(bp_xpdir, bp_plugindir, "data",
			    "msgs", lang, NULL);
		}
	}
	if (arpt_cc != NULL && (!file_exists(path, &isdir) || !isdir)) {
		/* Try a country-local variant of English */
		snprintf(msg_dir_name, sizeof (msg_dir_name),
		    "en_%s", arpt_cc);
		free(path);
		path = mkpathname(bp_xpdir, bp_plugindir, "data",
		    "msgs", msg_dir_name, NULL);
	}
	if (arpt_cc != NULL && (!file_exists(path, &isdir) || !isdir)) {
		/* Try a language-local variant of english */
		const char *lang = cc2lang(arpt_cc);

		snprintf(msg_dir_name, sizeof (msg_dir_name), "en-%s", lang);
		free(path);
		path = mkpathname(bp_xpdir, bp_plugindir, "data",
		    "msgs", msg_dir_name, NULL);
	}
	if (!file_exists(path, &isdir) || !isdir) {
		/* No matching lang_CC, fall back to default "en". */
		strlcpy(msg_dir_name, "en", sizeof (msg_dir_name));
	}
	free(path);

	for (message_t msg = 0; msg < MSG_NUM_MSGS; msg++) {
		char *path = mkpathname(bp_xpdir, bp_plugindir, "data",
		    "msgs", msg_dir_name, msgs[msg].filename, NULL);
		msgs[msg].wav = wav_load(path, msgs[msg].filename);
		if (msgs[msg].wav == NULL) {
			logMsg("BetterPushback initialization error, unable "
			    "to load sound file %s (prefdir: %s)", path,
			    msg_dir_name);
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
