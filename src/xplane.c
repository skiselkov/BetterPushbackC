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

#include <XPLMMenus.h>
#include <XPLMUtilities.h>
#include <XPLMPlugin.h>

#include <acfutils/acfutils.h>
#include <acfutils/helpers.h>
#include <acfutils/log.h>

#include "bp.h"
#include "xplane.h"

#define BP_PLUGIN_NAME		"BetterPushback 1.0"
#define BP_PLUGIN_SIG		"skiselkov.BetterPushback1.0"
#define BP_PLUGIN_DESCRIPTION	"Generic automated pushback plugin"

static XPLMCommandRef	start_pb, stop_pb, start_cam, stop_cam;
static XPLMMenuID	root_menu;
static int		plugins_menu_item;
static int		start_pb_plan_menu_item, stop_pb_plan_menu_item;
static int		start_pb_menu_item, stop_pb_menu_item;

static int start_pb_handler(XPLMCommandRef, XPLMCommandPhase, void *);
static int stop_pb_handler(XPLMCommandRef, XPLMCommandPhase, void *);
static int start_cam_handler(XPLMCommandRef, XPLMCommandPhase, void *);
static int stop_cam_handler(XPLMCommandRef, XPLMCommandPhase, void *);

static bool_t		start_after_cam = B_FALSE;

static char		xpdir[512];
static char		plugindir[512];
const char *const	bp_xpdir = xpdir;
const char *const	bp_plugindir = plugindir;

static int
start_pb_handler(XPLMCommandRef cmd, XPLMCommandPhase phase, void *refcon)
{
	UNUSED(cmd);
	UNUSED(refcon);
	if (phase != xplm_CommandEnd)
		return (1);
	XPLMCommandOnce(stop_cam);
	if (!bp_init())
		return (1);
	if (bp_num_segs() == 0) {
		XPLMSpeakString("Ground to cockpit. Please show me where "
		    "you want to go.");
		start_after_cam = B_TRUE;
		XPLMCommandOnce(start_cam);
		return (1);
	}
	if (!bp_start())
		return (1);

	XPLMEnableMenuItem(root_menu, start_pb_plan_menu_item, B_FALSE);
	XPLMEnableMenuItem(root_menu, stop_pb_plan_menu_item, B_FALSE);
	XPLMEnableMenuItem(root_menu, start_pb_menu_item, B_FALSE);
	XPLMEnableMenuItem(root_menu, stop_pb_menu_item, B_TRUE);

	return (1);
}

static int
stop_pb_handler(XPLMCommandRef cmd, XPLMCommandPhase phase, void *refcon)
{
	UNUSED(cmd);
	UNUSED(refcon);
	if (phase != xplm_CommandEnd || !bp_init())
		return (1);
	(void) bp_stop();
	return (1);
}

static int
start_cam_handler(XPLMCommandRef cmd, XPLMCommandPhase phase, void *refcon)
{
	UNUSED(cmd);
	UNUSED(refcon);
	if (phase != xplm_CommandEnd || !bp_init() || !bp_cam_start())
		return (1);
	XPLMEnableMenuItem(root_menu, start_pb_plan_menu_item, B_FALSE);
	XPLMEnableMenuItem(root_menu, stop_pb_plan_menu_item, B_TRUE);
	XPLMEnableMenuItem(root_menu, start_pb_menu_item, B_FALSE);
	XPLMEnableMenuItem(root_menu, stop_pb_menu_item, B_FALSE);
	return (1);
}

static int
stop_cam_handler(XPLMCommandRef cmd, XPLMCommandPhase phase, void *refcon)
{
	UNUSED(cmd);
	UNUSED(refcon);
	if (phase != xplm_CommandEnd || !bp_init() || !bp_cam_stop())
		return (1);
	XPLMEnableMenuItem(root_menu, start_pb_plan_menu_item, B_TRUE);
	XPLMEnableMenuItem(root_menu, stop_pb_plan_menu_item, B_FALSE);
	XPLMEnableMenuItem(root_menu, start_pb_menu_item, B_TRUE);
	XPLMEnableMenuItem(root_menu, stop_pb_menu_item, B_FALSE);
	if (start_after_cam) {
		if (bp_num_segs() != 0)
			XPLMCommandOnce(start_pb);
		start_after_cam = B_FALSE;
	} else if (bp_can_start(NULL)) {
		XPLMSpeakString("Ground to cockpit. Plan acknowledged, "
		    "call me through the menu when you are ready.");
	}
	return (1);
}

static void
menu_cb(void *inMenuRef, void *inItemRef)
{
	UNUSED(inMenuRef);
	XPLMCommandOnce((XPLMCommandRef)inItemRef);
}

void
bp_done_notify(void)
{
	XPLMEnableMenuItem(root_menu, start_pb_menu_item, B_TRUE);
	XPLMEnableMenuItem(root_menu, stop_pb_menu_item, B_FALSE);
	XPLMEnableMenuItem(root_menu, start_pb_plan_menu_item, B_TRUE);
	XPLMEnableMenuItem(root_menu, stop_pb_plan_menu_item, B_FALSE);
}

PLUGIN_API int
XPluginStart(char *name, char *sig, char *desc)
{
	char *p;

	acfutils_logfunc = XPLMDebugString;

	/* Always use Unix-native paths on the Mac! */
	XPLMEnableFeature("XPLM_USE_NATIVE_PATHS", 1);

	XPLMGetSystemPath(xpdir);
	XPLMGetPluginInfo(XPLMGetMyID(), NULL, plugindir, NULL, NULL);

#if	IBM
	fix_pathsep(xpdir);
	fix_pathsep(plugindir);
#endif	/* IBM */

	/* cut off the trailing path component (our filename) */
	if ((p = strrchr(plugindir, DIRSEP)) != NULL)
		*p = '\0';
	/* cut off an optional '32' or '64' trailing component */
	if ((p = strrchr(plugindir, DIRSEP)) != NULL) {
		if (strcmp(p + 1, "64") == 0 || strcmp(p + 1, "32") == 0)
			*p = '\0';
	}

	if (strstr(plugindir, xpdir) == plugindir) {
		int xpdir_len = strlen(xpdir);
		int plugindir_len = strlen(plugindir);
		memmove(plugindir, &plugindir[xpdir_len],
		    plugindir_len - xpdir_len + 1);
	}

	strcpy(name, BP_PLUGIN_NAME);
	strcpy(sig, BP_PLUGIN_SIG);
	strcpy(desc, BP_PLUGIN_DESCRIPTION);

	/* We can't delete commands, so put their creation here */
	start_pb = XPLMCreateCommand("BetterPushback/start",
	    "Start BetterPushback");
	stop_pb = XPLMCreateCommand("BetterPushback/stop",
	    "Stop BetterPushback");
	start_cam = XPLMCreateCommand("BetterPushback/start_planner",
	    "Start BetterPushback planner");
	stop_cam = XPLMCreateCommand("BetterPushback/stop_planner",
	    "Stop BetterPushback planner");

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

	plugins_menu_item = XPLMAppendMenuItem(XPLMFindPluginsMenu(),
	    "Better Pushback", NULL, 1);
	root_menu = XPLMCreateMenu("Better Pushback", XPLMFindPluginsMenu(),
	    plugins_menu_item, menu_cb, NULL);

	start_pb_menu_item = XPLMAppendMenuItem(root_menu, "Start pushback",
	    start_pb, 1);
	stop_pb_menu_item = XPLMAppendMenuItem(root_menu, "Stop pushback",
	    stop_pb, 1);
	start_pb_plan_menu_item = XPLMAppendMenuItem(root_menu,
	    "Pre-plan pushback", start_cam, 1);
	stop_pb_plan_menu_item = XPLMAppendMenuItem(root_menu,
	    "Close pushback planner", stop_cam, 1);

	XPLMEnableMenuItem(root_menu, start_pb_menu_item, B_TRUE);
	XPLMEnableMenuItem(root_menu, stop_pb_menu_item, B_FALSE);
	XPLMEnableMenuItem(root_menu, start_pb_plan_menu_item, B_TRUE);
	XPLMEnableMenuItem(root_menu, stop_pb_plan_menu_item, B_FALSE);
}

PLUGIN_API void
XPluginDisable(void)
{
	XPLMUnregisterCommandHandler(start_pb, start_pb_handler, 1, NULL);
	XPLMUnregisterCommandHandler(stop_pb, stop_pb_handler, 1, NULL);
	XPLMUnregisterCommandHandler(start_cam, start_pb_handler, 1, NULL);
	XPLMUnregisterCommandHandler(stop_cam, stop_pb_handler, 1, NULL);
	bp_fini();

	XPLMDestroyMenu(root_menu);
	XPLMRemoveMenuItem(XPLMFindPluginsMenu(), plugins_menu_item);
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
