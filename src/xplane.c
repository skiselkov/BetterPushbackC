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

#include <XPLMUtilities.h>
#include <XPLMPlugin.h>

#include "helpers.h"
#include "log.h"

#include "bp.h"

#define BP_PLUGIN_NAME		"BetterPushback 1.0"
#define BP_PLUGIN_SIG		"skiselkov.BetterPushback1.0"
#define BP_PLUGIN_DESCRIPTION	"Generic automated pushback plugin"

static XPLMCommandRef	start_pb, stop_pb, start_cam, stop_cam;

static int start_pb_handler(XPLMCommandRef, XPLMCommandPhase, void *);
static int stop_pb_handler(XPLMCommandRef, XPLMCommandPhase, void *);
static int start_cam_handler(XPLMCommandRef, XPLMCommandPhase, void *);
static int stop_cam_handler(XPLMCommandRef, XPLMCommandPhase, void *);

static int
start_pb_handler(XPLMCommandRef cmd, XPLMCommandPhase phase, void *refcon)
{
	UNUSED(cmd);
	UNUSED(refcon);
	if (phase != xplm_CommandEnd)
		return (1);
	bp_cam_fini();
	if (!bp_init())
		return (1);
	bp_start();
	return (1);
}

static int
stop_pb_handler(XPLMCommandRef cmd, XPLMCommandPhase phase, void *refcon)
{
	UNUSED(cmd);
	UNUSED(refcon);
	if (phase != xplm_CommandEnd)
		return (1);
	bp_stop();
	return (1);
}

static int
start_cam_handler(XPLMCommandRef cmd, XPLMCommandPhase phase, void *refcon)
{
	UNUSED(cmd);
	UNUSED(refcon);
	if (phase != xplm_CommandEnd)
		return (1);
	bp_cam_init();
	return (1);
}

static int
stop_cam_handler(XPLMCommandRef cmd, XPLMCommandPhase phase, void *refcon)
{
	UNUSED(cmd);
	UNUSED(refcon);
	if (phase != xplm_CommandEnd)
		return (1);
	bp_cam_fini();
	return (1);
}

PLUGIN_API int
XPluginStart(char *name, char *sig, char *desc)
{
	strcpy(name, BP_PLUGIN_NAME);
	strcpy(sig, BP_PLUGIN_SIG);
	strcpy(desc, BP_PLUGIN_DESCRIPTION);

	start_pb = XPLMCreateCommand("BetterPushback/start", "Start BP");
	stop_pb = XPLMCreateCommand("BetterPushback/stop", "Stop BP");
	start_cam = XPLMCreateCommand("BetterPushback/start_cam",
	    "Start BP Camera");
	stop_cam = XPLMCreateCommand("BetterPushback/stop_cam",
	    "Stop BP Camera");

	return (1);
}

PLUGIN_API void
XPluginStop(void)
{
}

PLUGIN_API void
XPluginEnable(void)
{
	XPLMRegisterCommandHandler(start_pb, start_pb_handler, 1, NULL);
	XPLMRegisterCommandHandler(stop_pb, stop_pb_handler, 1, NULL);
	XPLMRegisterCommandHandler(start_cam, start_cam_handler, 1, NULL);
	XPLMRegisterCommandHandler(stop_cam, stop_cam_handler, 1, NULL);
}

PLUGIN_API void
XPluginDisable(void)
{
	XPLMUnregisterCommandHandler(start_pb, start_pb_handler, 1, NULL);
	XPLMUnregisterCommandHandler(stop_pb, stop_pb_handler, 1, NULL);
	XPLMUnregisterCommandHandler(start_cam, start_pb_handler, 1, NULL);
	XPLMUnregisterCommandHandler(stop_cam, stop_pb_handler, 1, NULL);
	stop_pb_handler(NULL, 0, NULL);
}

PLUGIN_API void
XPluginReceiveMessage(XPLMPluginID from, int msg, void *param)
{
	UNUSED(from);
	UNUSED(param);

	switch (msg) {
	case XPLM_MSG_LIVERY_LOADED:
	case XPLM_MSG_AIRPORT_LOADED:
	case XPLM_MSG_PLANE_UNLOADED:
		/* Force a reinit to re-read aircraft size params */
		bp_fini();
		break;
	}
}
