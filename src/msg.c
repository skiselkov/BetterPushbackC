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

/*
 * This examines an optional "cc_aliases.cfg" file in our messages directory.
 * This can used to remap a country code to another code to for example
 * utilize the second country's audio set as a common set. One example of
 * this is en_GB, which also applies in British overseas territories.
 */
static void
alias_cc(const char *cc, char aliased_cc[3])
{
	char *path = mkpathname(bp_xpdir, bp_plugindir, "data", "msgs",
	    "cc_aliases.cfg", NULL);
	FILE *fp = fopen(path, "r");
	char cc_in[8], cc_out[8];

	free(path);
	if (fp == NULL)
		goto errout;

	while (!feof(fp)) {
		if (fscanf(fp, "%7s %7s", cc_in, cc_out) != 2)
			break;
		if (*cc_in == '#') {
			int c;
			do {
				c = fgetc(fp);
			} while (c != '\n' && c != '\r' && c != EOF);
			continue;
		}
		if (strcmp(cc, cc_in) == 0) {
			strlcpy(aliased_cc, cc_out, 3);
			fclose(fp);
			return;
		}
	}

	fclose(fp);
errout:
	strlcpy(aliased_cc, cc, 3);
}

bool_t
msg_init(const char *my_lang, const char *icao, lang_pref_t lang_pref)
{
	const char *arpt_cc = icao2cc(icao);
	const char *arpt_lang = icao2lang(icao);
	enum { MAX_MATCHES = 4 };
	char match_set[MAX_MATCHES][8] = { {0}, {0}, {0}, {0} };
	const char *msg_dir_name = NULL;
	char cc[3];

	ASSERT(!inited);

	if (arpt_cc != NULL)
		alias_cc(arpt_cc, cc);
	else
		strcpy(cc, "XX");

	switch (lang_pref) {
	case LANG_PREF_MATCH_REAL:
		/*
		 * For match real our preference order is:
		 * 1) try "my_lang_CC" for country-local accent of my language
		 * 2) if arpt_lang == my_lang, try "my_lang" for generic
		 *	language fallback
		 * 3) try "en_CC" for country-local English accent
		 * 4) try "en-arpt_lang" for generic English accent of local
		 *	language
		 */
		snprintf(match_set[0], sizeof (*match_set), "%s_%s",
		    my_lang, cc);
		if (strcmp(arpt_lang, my_lang) == 0)
			strlcpy(match_set[1], my_lang, sizeof (*match_set));
		snprintf(match_set[2], sizeof (*match_set), "en_%s", cc);
		snprintf(match_set[3], sizeof (*match_set), "en-%s", arpt_lang);
		CTASSERT(MAX_MATCHES > 3);
		break;
	case LANG_PREF_NATIVE:
		/*
		 * For native, our preference order is:
		 * 1) try "my_lang", ignoring any local accent
		 * 2) try "my_lang_CC" for country-local accent
		 */
		strlcpy(match_set[0], my_lang, sizeof (*match_set));
		snprintf(match_set[1], sizeof (*match_set), "%s_%s",
		    my_lang, cc);
		CTASSERT(MAX_MATCHES > 1);
		break;
	default:
		/*
		 * For match-English, preference order is:
		 * 1) try "en_CC" for country-local English accent
		 * 2) try "en-arpt_lang" for generic English accent of local
		 *	language
		 */
		VERIFY3U(lang_pref, ==, LANG_PREF_MATCH_ENGLISH);
		snprintf(match_set[0], sizeof (*match_set), "en_%s", cc);
		snprintf(match_set[1], sizeof (*match_set), "en-%s", arpt_lang);
		CTASSERT(MAX_MATCHES > 1);
		break;
	};

	for (int i = 0; i < MAX_MATCHES; i++) {
		char *path;
		bool_t isdir;

		if (*match_set[i] == 0)
			continue;
		path = mkpathname(bp_xpdir, bp_plugindir, "data", "msgs",
		    match_set[i], NULL);
		if (file_exists(path, &isdir) && isdir) {
			free(path);
			msg_dir_name = match_set[i];
			break;
		}
		free(path);
	}

	if (msg_dir_name == NULL) {
		/* final fallback for everything: generic English */
		msg_dir_name = "en";
	}

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
