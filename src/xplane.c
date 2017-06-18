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

#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#include "XPLMUtilities.h"
#include "XPLMProcessing.h"

#include "helpers.h"

#include "bp.h"

#define BP_PLUGIN_NAME		"BetterPushback 1.0"
#define BP_PLUGIN_SIG		"skiselkov.BetterPushback1.0"
#define BP_PLUGIN_DESCRIPTION	"Generic automated pushback plugin"

static bool_t		started = B_FALSE;
static XPLMCommandRef	start_cmd, stop_cmd;

static int start_cmd_handler(XPLMCommandRef, XPLMCommandPhase, void *);
static int stop_cmd_handler(XPLMCommandRef, XPLMCommandPhase, void *);
static float floop_cb(float, float, int, void *);

static int
start_cmd_handler(XPLMCommandRef cmd, XPLMCommandPhase phase, void *refcon)
{
	UNUSED(cmd);
	UNUSED(phase);
	UNUSED(refcon);

	/* make sure we're stopped */
	stop_cmd_handler(NULL, 0, NULL);

	if (!bp_init())
		return (1);

	bp_test_setup();

	XPLMRegisterFlightLoopCallback(floop_cb, -1, NULL);
	started = B_TRUE;

	return (1);
}

static int
stop_cmd_handler(XPLMCommandRef cmd, XPLMCommandPhase phase, void *refcon)
{
	UNUSED(cmd);
	UNUSED(phase);
	UNUSED(refcon);

	if (!started)
		return (1);

	bp_fini();
	XPLMUnregisterFlightLoopCallback(floop_cb, NULL);

	started = B_FALSE;

	return (1);
}

static float
floop_cb(float elapsed_since_last_call, float elapsed_since_last_floop,
    int counter, void *refcon)
{
	UNUSED(elapsed_since_last_call);
	UNUSED(elapsed_since_last_floop);
	UNUSED(counter);
	UNUSED(refcon);

	if (!bp_run()) {
		/* can't unregister from callback, so just stop the callback */
		return (0);
	}

	return (-1.0);
}

PLUGIN_API int
XPluginStart(char *name, char *sig, char *desc)
{
	strcpy(name, BP_PLUGIN_NAME);
	strcpy(sig, BP_PLUGIN_SIG);
	strcpy(desc, BP_PLUGIN_DESCRIPTION);

	start_cmd = XPLMCreateCommand("BetterPushback/start", "Start BP");
	stop_cmd = XPLMCreateCommand("BetterPushback/stop", "Stop BP");

	return (1);
}

PLUGIN_API void
XPluginStop(void)
{
}

PLUGIN_API void
XPluginEnable(void)
{
	XPLMRegisterCommandHandler(start_cmd, start_cmd_handler, 1, NULL);
	XPLMRegisterCommandHandler(stop_cmd, stop_cmd_handler, 1, NULL);
}

PLUGIN_API void
XPluginDisable(void)
{
	XPLMUnregisterCommandHandler(start_cmd, start_cmd_handler, 1, NULL);
	XPLMUnregisterCommandHandler(stop_cmd, stop_cmd_handler, 1, NULL);
	stop_cmd_handler(NULL, 0, NULL);
}

PLUGIN_API void
XPluginReceiveMessage(XPLMPluginID from, int msg, void *param)
{
	UNUSED(from);
	UNUSED(msg);
	UNUSED(param);
}
