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

#ifndef	_MSG_H_
#define	_MSG_H_

#include <acfutils/types.h>

#ifdef	__cplusplus
extern "C" {
#endif

typedef enum {
	LANG_PREF_MATCH_REAL,
	LANG_PREF_NATIVE,
	LANG_PREF_MATCH_ENGLISH
} lang_pref_t;

typedef enum {
	MSG_PLAN_START,
	MSG_PLAN_END,
	MSG_DRIVING_UP,
	MSG_RDY2CONN,
	MSG_RDY2CONN_NOPARK,
	MSG_WINCH,
	MSG_CONNECTED,
	MSG_START_PB,
	MSG_START_TOW,
	MSG_START_PB_NOSTART,
	MSG_START_TOW_NOSTART,
	MSG_OP_COMPLETE,
	MSG_DISCO,
	MSG_DONE_RIGHT,
	MSG_DONE_LEFT,
	MSG_NUM_MSGS
} message_t;

bool_t msg_init(const char *my_lang, const char *icao, lang_pref_t lang_pref);
void msg_fini();
void msg_play(message_t msg);
void msg_stop(void);
double msg_dur(message_t msg);

#ifdef	__cplusplus
}
#endif

#endif	/* _MSG_H_ */
