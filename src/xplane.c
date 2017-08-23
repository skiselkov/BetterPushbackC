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
#include <XPLMProcessing.h>

#include <acfutils/assert.h>
#include <acfutils/core.h>
#include <acfutils/crc64.h>
#include <acfutils/helpers.h>
#include <acfutils/intl.h>
#include <acfutils/log.h>
#include <acfutils/wav.h>
#include <acfutils/time.h>

#include "bp.h"
#include "cab_view.h"
#include "cfg.h"
#include "msg.h"
#include "tug.h"
#include "xplane.h"

/* Enables leaving bp_tug_name set to facilitate local master/slave debug */
/*#define	SLAVE_DEBUG*/

#define BP_PLUGIN_NAME		"BetterPushback-" BP_PLUGIN_VERSION
#define BP_PLUGIN_SIG		"skiselkov.BetterPushback"
#define BP_PLUGIN_DESCRIPTION	"Generic automated pushback plugin"

#define	STATUS_CHECK_INTVAL	1	/* second */
enum {
	SMARTCOPILOT_STATE_OFF = 0,	/* disconnected */
	SMARTCOPILOT_STATE_SLAVE = 1,	/* connected and we're slave */
	SMARTCOPILOT_STATE_MASTER = 2	/* connected and we're master */
};

static bool_t		inited = B_FALSE;

static XPLMCommandRef	start_pb, stop_pb, start_cam, stop_cam, conn_first;
static XPLMCommandRef	cab_cam;
static XPLMMenuID	root_menu;
static int		plugins_menu_item;
static int		start_pb_plan_menu_item, stop_pb_plan_menu_item;
static int		start_pb_menu_item, stop_pb_menu_item;
static int		cab_cam_menu_item;
static int		prefs_menu_item;

static int start_pb_handler(XPLMCommandRef, XPLMCommandPhase, void *);
static int stop_pb_handler(XPLMCommandRef, XPLMCommandPhase, void *);
static int start_cam_handler(XPLMCommandRef, XPLMCommandPhase, void *);
static int stop_cam_handler(XPLMCommandRef, XPLMCommandPhase, void *);
static int conn_first_handler(XPLMCommandRef, XPLMCommandPhase, void *);
static int cab_cam_handler(XPLMCommandRef, XPLMCommandPhase, void *);

static bool_t		start_after_cam = B_FALSE;

static char		xpdir[512];
static char		plugindir[512];
const char *const	bp_xpdir = xpdir;
const char *const	bp_plugindir = plugindir;

static bool_t		smartcopilot_present;
static dr_t		smartcopilot_state;

int			bp_xp_ver, bp_xplm_ver;
XPLMHostApplicationID	bp_host_id;
airportdb_t		*airportdb = NULL;

static bool_t bp_priv_enable(void);
static void bp_priv_disable(void);
static float bp_do_reload(float, float, int, void *);

static bool_t			reload_rqst = B_FALSE;
static XPLMCreateFlightLoop_t	reload_floop = {
    .structSize = sizeof (reload_floop),
    .phase = xplm_FlightLoop_Phase_AfterFlightModel,
    .callbackFunc = bp_do_reload,
    .refcon = NULL
};
static XPLMFlightLoopID		reload_floop_ID = NULL;

/*
 * These datarefs are for syncing two instances of BetterPushback over the
 * net via syncing addons such as smartcopilot. This works as follows:
 * 1) Master/slave must not be switched during pushback (undefined behavior
 *	may result). It's possible to observe if BetterPushback is running by
 *	monitoring the read-only boolean "bp/started" dataref. This dataref
 *	must NOT be synced, it's only a hint to smartcopilot whether
 *	switching is safe.
 * 2) The boolean dataref "bp/slave_mode" must be set to 0 on the master and
 *	1 on the slave.
 * 3) The boolean dataref "bp/op_complete" must be synced from master to
 *	slave. This signals to bp_run() that the pushback stage needs to
 *	progress to either PB_STEP_STOPPING if the tug has already attached,
 *	or immediately to bp_complete() if it has not. The master sets this
 *	dataref in response to either a "Stop" command request, or when it
 *	has processed all pushback segments. The slave cannot set this
 *	(master controls when pushback stops).
 * 4) The boolean dataref "bp/plan_complete" must be synced from master to
 *	slave. This is a signal from the master machine to the slave that
 *	if late_plan_requested was in effect, the slave can continue with
 *	the state transitions past the late_plan_requested limit. This is
 *	needed because we don't transfer the route to the slave, so the
 *	slave cannot use the presence of a route as a condition to continue.
 * 5) The string dataref "bp/tug_name" must be synced from master to slave.
 *	This string identifies which tug model the master selected (since tug
 *	selection is non-deterministic). The slave then instances its tug
 *	object using tug_alloc_man. Both master and slave must have identical
 *	tug libraries, otherwise sync fails.
 * 6) The command "BetterPushback/start" should be synced from mater to slave.
 *	There's no need to sync any other commands. The planning GUI is
 *	disabled on the slave machine and stopping of the pushback can only be
 *	performed by the master machine.
 */
static dr_t	bp_started_dr, slave_mode_dr, op_complete_dr, plan_complete_dr;
static dr_t	bp_tug_name_dr;
bool_t		bp_started = B_FALSE, slave_mode = B_FALSE;
bool_t		op_complete = B_FALSE, plan_complete = B_FALSE;
char		bp_tug_name[64] = { 0 };

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
	if (bp_num_segs() == 0 && !slave_mode) {
		if (!bp_cam_start())
			return (1);
		XPLMEnableMenuItem(root_menu, start_pb_plan_menu_item, B_FALSE);
		XPLMEnableMenuItem(root_menu, stop_pb_plan_menu_item, B_TRUE);
		XPLMEnableMenuItem(root_menu, start_pb_menu_item, B_FALSE);
		XPLMEnableMenuItem(root_menu, stop_pb_menu_item, B_FALSE);
		msg_play(MSG_PLAN_START);
		start_after_cam = B_TRUE;
		return (1);
	}
	op_complete = B_FALSE;
	late_plan_requested = B_FALSE;
	if (!bp_start())
		return (1);

	XPLMEnableMenuItem(root_menu, start_pb_plan_menu_item, B_FALSE);
	XPLMEnableMenuItem(root_menu, stop_pb_plan_menu_item, B_FALSE);
	XPLMEnableMenuItem(root_menu, start_pb_menu_item, B_FALSE);
	XPLMEnableMenuItem(root_menu, stop_pb_menu_item, !slave_mode);

	return (1);
}

static int
stop_pb_handler(XPLMCommandRef cmd, XPLMCommandPhase phase, void *refcon)
{
	UNUSED(cmd);
	UNUSED(refcon);
	if (slave_mode)
		return (1);
	if (phase != xplm_CommandEnd || !bp_init())
		return (1);
	(void) bp_stop();
	op_complete = B_TRUE;
	if (!slave_mode) {
		/* Reset the menu back */
		late_plan_requested = B_FALSE;
		XPLMEnableMenuItem(root_menu, start_pb_menu_item, B_FALSE);
	}
	return (1);
}

static int
start_cam_handler(XPLMCommandRef cmd, XPLMCommandPhase phase, void *refcon)
{
	UNUSED(cmd);
	UNUSED(refcon);
	if (slave_mode || late_plan_requested)
		return (1);
	if (phase != xplm_CommandEnd || !bp_init() || !bp_cam_start()) {
		start_after_cam = B_FALSE;
		return (1);
	}
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
	if (slave_mode)
		return (1);
	if (phase != xplm_CommandEnd || !bp_init() || !bp_cam_stop())
		return (1);
	XPLMEnableMenuItem(root_menu, start_pb_plan_menu_item, B_TRUE);
	XPLMEnableMenuItem(root_menu, stop_pb_plan_menu_item, B_FALSE);
	XPLMEnableMenuItem(root_menu, start_pb_menu_item, B_TRUE);
	XPLMEnableMenuItem(root_menu, stop_pb_menu_item, B_FALSE);
	if (late_plan_requested) {
		XPLMEnableMenuItem(root_menu, start_pb_plan_menu_item, B_FALSE);
		XPLMEnableMenuItem(root_menu, stop_pb_plan_menu_item, B_FALSE);
		XPLMEnableMenuItem(root_menu, start_pb_menu_item,
		    bp_num_segs() == 0);
		XPLMEnableMenuItem(root_menu, stop_pb_menu_item, B_TRUE);
	} else if (start_after_cam) {
		if (bp_num_segs() != 0)
			XPLMCommandOnce(start_pb);
	} else if (bp_can_start(NULL)) {
		msg_play(MSG_PLAN_END);
	}

	start_after_cam = B_FALSE;

	return (1);
}

static int
conn_first_handler(XPLMCommandRef cmd, XPLMCommandPhase phase, void *refcon)
{
	UNUSED(cmd);
	UNUSED(refcon);
	if (phase != xplm_CommandEnd || !bp_init() || bp_started || slave_mode)
		return (0);
	late_plan_requested = B_TRUE;
	(void) bp_cam_stop();
	if (!bp_start())
		return (1);
	if (!slave_mode) {
		XPLMEnableMenuItem(root_menu, start_pb_plan_menu_item, B_FALSE);
		XPLMEnableMenuItem(root_menu, stop_pb_plan_menu_item, B_FALSE);
		XPLMEnableMenuItem(root_menu, start_pb_menu_item,
		    bp_num_segs() == 0);
		XPLMEnableMenuItem(root_menu, stop_pb_menu_item, B_TRUE);
	}
	return (1);
}

static int
cab_cam_handler(XPLMCommandRef cmd, XPLMCommandPhase phase, void *refcon)
{
	UNUSED(cmd);
	UNUSED(refcon);
	if (phase != xplm_CommandEnd)
		return (0);
	if (!cab_view_start()) {
		XPLMSpeakString(_("Unable to select pushback tug view at "
		    "this time."));
		return (0);
	}
	return (1);
}

static void
menu_cb(void *inMenuRef, void *inItemRef)
{
	UNUSED(inMenuRef);

	if (inItemRef == &prefs_menu_item) {
		bp_conf_open();
	} else {
		XPLMCommandOnce((XPLMCommandRef)inItemRef);
	}
}

void
bp_done_notify(void)
{
	if (!slave_mode) {
		XPLMEnableMenuItem(root_menu, start_pb_menu_item, B_TRUE);
		XPLMEnableMenuItem(root_menu, stop_pb_menu_item, B_FALSE);
		XPLMEnableMenuItem(root_menu, start_pb_plan_menu_item, B_TRUE);
		XPLMEnableMenuItem(root_menu, stop_pb_plan_menu_item, B_FALSE);
	}

#ifndef	SLAVE_DEBUG
	bp_tug_name[0] = '\0';
#endif
}

/*
 * Notification from BP engine that a reconnect has been requested at the
 * appropriate time. Behave as if the user had hit "connect first" and on
 * the master's machine invoke the planner.
 */
void
bp_reconnect_notify(void)
{
	if (slave_mode)
		return;

	late_plan_requested = B_TRUE;
	VERIFY(bp_cam_start());
	msg_play(MSG_PLAN_START);
	XPLMEnableMenuItem(root_menu, start_pb_plan_menu_item, B_FALSE);
	XPLMEnableMenuItem(root_menu, stop_pb_plan_menu_item, B_TRUE);
	XPLMEnableMenuItem(root_menu, start_pb_menu_item, B_FALSE);
	XPLMEnableMenuItem(root_menu, stop_pb_menu_item, B_TRUE);
}

const char *
bp_get_lang(void)
{
	const char *c;
	if (conf_get_str(bp_conf, "lang", &c))
		return (c);
	return (acfutils_xplang2code(XPLMGetLanguage()));
}

void
slave_mode_cb(dr_t *dr)
{
	UNUSED(dr);
	VERIFY(!bp_started);

	if (slave_mode) {
		bp_fini();
		XPLMEnableMenuItem(root_menu, start_pb_menu_item, B_FALSE);
		XPLMEnableMenuItem(root_menu, stop_pb_menu_item, B_FALSE);
		XPLMEnableMenuItem(root_menu, start_pb_plan_menu_item, B_FALSE);
		XPLMEnableMenuItem(root_menu, stop_pb_plan_menu_item, B_FALSE);
	} else {
		XPLMEnableMenuItem(root_menu, start_pb_menu_item, B_TRUE);
		XPLMEnableMenuItem(root_menu, stop_pb_menu_item, B_FALSE);
		XPLMEnableMenuItem(root_menu, start_pb_plan_menu_item, B_TRUE);
		XPLMEnableMenuItem(root_menu, stop_pb_plan_menu_item, B_FALSE);
	}
}

static float
status_check(float elapsed, float elapsed2, int counter, void *refcon)
{
	UNUSED(elapsed);
	UNUSED(elapsed2);
	UNUSED(counter);
	UNUSED(refcon);

	XPLMEnableMenuItem(root_menu, cab_cam_menu_item, cab_view_can_start());

	if (!smartcopilot_present)
		return (1);

	if (dr_geti(&smartcopilot_state) == SMARTCOPILOT_STATE_SLAVE &&
	    !slave_mode) {
		if (bp_started) {
			XPLMSpeakString(_("Pushback failure: smartcopilot "
			    "attempted to switch master/slave or network "
			    "connection lost. Stopping operation."));
		}
		/*
		 * If we were in master mode, stop the camera, flush out all
		 * pushback segments and inhibit all menu items. The master
		 * will control us.
		 */
		bp_fini();
		XPLMEnableMenuItem(root_menu, start_pb_menu_item, B_FALSE);
		XPLMEnableMenuItem(root_menu, stop_pb_menu_item, B_FALSE);
		XPLMEnableMenuItem(root_menu, start_pb_plan_menu_item, B_FALSE);
		XPLMEnableMenuItem(root_menu, stop_pb_plan_menu_item, B_FALSE);
		slave_mode = B_TRUE;
	} else if (dr_geti(&smartcopilot_state) != SMARTCOPILOT_STATE_SLAVE &&
	    slave_mode) {
		if (bp_started) {
			XPLMSpeakString(_("Pushback failure: smartcopilot "
			    "attempted to switch master/slave or network "
			    "connection lost. Stopping operation."));
		}
		/* If we were in slave mode, reenable the menu items. */
		bp_fini();
		XPLMEnableMenuItem(root_menu, start_pb_menu_item, B_TRUE);
		XPLMEnableMenuItem(root_menu, stop_pb_menu_item, B_FALSE);
		XPLMEnableMenuItem(root_menu, start_pb_plan_menu_item, B_TRUE);
		XPLMEnableMenuItem(root_menu, stop_pb_plan_menu_item, B_FALSE);
		slave_mode = B_FALSE;
	}

	return (STATUS_CHECK_INTVAL);
}

static void
xlate_init(void)
{
	char *po_file = mkpathname(xpdir, plugindir, "data", "po",
	    bp_get_lang(), "strings.po", NULL);
	(void) acfutils_xlate_init(po_file);
	free(po_file);
}

PLUGIN_API int
XPluginStart(char *name, char *sig, char *desc)
{
	char *p;

	log_init(XPLMDebugString, "BetterPushback");
	crc64_init();
	crc64_srand(microclock());
	logMsg("This is BetterPushback-" BP_PLUGIN_VERSION
	    " libacfutils-%s", libacfutils_version);

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

	/*
	 * Now we strip a leading xpdir from plugindir, so that now plugindir
	 * will be relative to X-Plane's root directory.
	 */
	if (strstr(plugindir, xpdir) == plugindir) {
		int xpdir_len = strlen(xpdir);
		int plugindir_len = strlen(plugindir);
		memmove(plugindir, &plugindir[xpdir_len],
		    plugindir_len - xpdir_len + 1);
	}

	strcpy(name, BP_PLUGIN_NAME);
	strcpy(sig, BP_PLUGIN_SIG);
	strcpy(desc, BP_PLUGIN_DESCRIPTION);

	/* We need the configuration very early to be able to pick the lang */
	if (!bp_conf_init())
		return (0);

	/* We need the i18n support really early, so init early */
	xlate_init();

	/* We can't delete commands, so put their creation here */
	start_pb = XPLMCreateCommand("BetterPushback/start",
	    _("Start pushback"));
	stop_pb = XPLMCreateCommand("BetterPushback/stop",
	    _("Stop pushback"));
	start_cam = XPLMCreateCommand("BetterPushback/start_planner",
	    _("Start pushback planner"));
	stop_cam = XPLMCreateCommand("BetterPushback/stop_planner",
	    _("Stop pushback planner"));
	conn_first = XPLMCreateCommand("BetterPushback/connect_first",
	    _("Connect tug before entering pushback plan"));
	cab_cam = XPLMCreateCommand("BetterPushback/cab_camera",
	    _("View from tug's cab."));

	bp_boot_init();
	tug_glob_init();

	dr_create_i(&bp_started_dr, (int *)&bp_started, B_FALSE,
	    "bp/started");
	dr_create_i(&slave_mode_dr, (int *)&slave_mode, B_TRUE,
	    "bp/slave_mode");
	slave_mode_dr.write_cb = slave_mode_cb;
	dr_create_i(&op_complete_dr, (int *)&op_complete, B_TRUE,
	    "bp/op_complete");
	dr_create_i(&plan_complete_dr, (int *)&plan_complete, B_TRUE,
	    "bp/plan_complete");
	dr_create_b(&bp_tug_name_dr, bp_tug_name, sizeof (bp_tug_name),
	    B_TRUE, "bp/tug_name");

	XPLMGetVersions(&bp_xp_ver, &bp_xplm_ver, &bp_host_id);

	reload_floop_ID = XPLMCreateFlightLoop(&reload_floop);

	return (1);
}

PLUGIN_API void
XPluginStop(void)
{
	bp_conf_fini();
	acfutils_xlate_fini();
	tug_glob_fini();
	bp_shut_fini();
	dr_delete(&bp_started_dr);
	dr_delete(&slave_mode_dr);
	dr_delete(&op_complete_dr);
	dr_delete(&bp_tug_name_dr);

	if (reload_floop_ID != NULL) {
		XPLMDestroyFlightLoop(reload_floop_ID);
		reload_floop_ID = NULL;
	}
}

/*
 * The actual enable/disable bootstrapping code is in bp_priv_{enable,disable}.
 * This is to allow these routines to be called from our configuration code
 * as well, but using the XPlugin{Enable,Disable} interface could result in
 * linking problems, since every plugin in the system needs to have these
 * functions externally exported.
 */
PLUGIN_API int
XPluginEnable(void)
{
	return (bp_priv_enable());
}

PLUGIN_API void
XPluginDisable(void)
{
	bp_priv_disable();
}

PLUGIN_API void
XPluginReceiveMessage(XPLMPluginID from, int msg, void *param)
{
	UNUSED(from);
	UNUSED(param);

	switch (msg) {
	case XPLM_MSG_AIRPORT_LOADED:
	case XPLM_MSG_PLANE_LOADED:
	case XPLM_MSG_PLANE_UNLOADED:
		/* Force a reinit to re-read aircraft size params */
		smartcopilot_present = dr_find(&smartcopilot_state,
		    "scp/api/ismaster");
		stop_cam_handler(NULL, xplm_CommandEnd, NULL);
		bp_fini();
		cab_view_fini();
#ifndef	SLAVE_DEBUG
		bp_tug_name[0] = '\0';
#endif
		break;
	}
}

static bool_t
bp_priv_enable(void)
{
	char *cachedir = mkpathname(xpdir, "Output", "caches",
	    "BetterPushbackAirports.cache", NULL);

	ASSERT(!inited);

	/*
	 * Reinit translations & config to allow switching languages on
	 * the fly.
	 */
	acfutils_xlate_fini();
	xlate_init();
	bp_conf_fini();
	if (!bp_conf_init())
		return (0);

	airportdb = calloc(1, sizeof (*airportdb));
	airportdb_create(airportdb, bp_xpdir, cachedir);

	if (!recreate_cache(airportdb) || !openal_init())
		goto errout;

	XPLMRegisterCommandHandler(start_pb, start_pb_handler, 1, NULL);
	XPLMRegisterCommandHandler(stop_pb, stop_pb_handler, 1, NULL);
	XPLMRegisterCommandHandler(start_cam, start_cam_handler, 1, NULL);
	XPLMRegisterCommandHandler(stop_cam, stop_cam_handler, 1, NULL);
	XPLMRegisterCommandHandler(conn_first, conn_first_handler, 1, NULL);
	XPLMRegisterCommandHandler(cab_cam, cab_cam_handler, 1, NULL);

	plugins_menu_item = XPLMAppendMenuItem(XPLMFindPluginsMenu(),
	    "Better Pushback", NULL, 1);
	root_menu = XPLMCreateMenu("Better Pushback", XPLMFindPluginsMenu(),
	    plugins_menu_item, menu_cb, NULL);

	start_pb_plan_menu_item = XPLMAppendMenuItem(root_menu,
	    _("Pre-plan pushback"), start_cam, 1);
	stop_pb_plan_menu_item = XPLMAppendMenuItem(root_menu,
	    _("Close pushback planner"), stop_cam, 1);
	start_pb_menu_item = XPLMAppendMenuItem(root_menu,
	    _("Start pushback"), start_pb, 1);
	stop_pb_menu_item = XPLMAppendMenuItem(root_menu,
	    _("Stop pushback"), stop_pb, 1);
	cab_cam_menu_item = XPLMAppendMenuItem(root_menu,
	    _("Tug cab view"), cab_cam, 1);
	prefs_menu_item = XPLMAppendMenuItem(root_menu,
	    _("Preferences..."), &prefs_menu_item, 1);

	XPLMEnableMenuItem(root_menu, start_pb_menu_item, B_TRUE);
	XPLMEnableMenuItem(root_menu, stop_pb_menu_item, B_FALSE);
	XPLMEnableMenuItem(root_menu, start_pb_plan_menu_item, B_TRUE);
	XPLMEnableMenuItem(root_menu, stop_pb_plan_menu_item, B_FALSE);
	XPLMEnableMenuItem(root_menu, cab_cam_menu_item, B_FALSE);

	XPLMRegisterFlightLoopCallback(status_check, STATUS_CHECK_INTVAL, NULL);

	inited = B_TRUE;

	free(cachedir);

	return (1);

errout:
	openal_fini();
	if (cachedir != NULL)
		free(cachedir);
	if (airportdb != NULL) {
		airportdb_destroy(airportdb);
		free(airportdb);
		airportdb = NULL;
	}

	return (0);
}

static void
bp_priv_disable(void)
{
	if (!inited)
		return;

	XPLMUnregisterCommandHandler(start_pb, start_pb_handler, 1, NULL);
	XPLMUnregisterCommandHandler(stop_pb, stop_pb_handler, 1, NULL);
	XPLMUnregisterCommandHandler(start_cam, start_pb_handler, 1, NULL);
	XPLMUnregisterCommandHandler(stop_cam, stop_pb_handler, 1, NULL);
	XPLMUnregisterCommandHandler(conn_first, conn_first_handler, 1, NULL);
	XPLMUnregisterCommandHandler(cab_cam, cab_cam_handler, 1, NULL);

	bp_fini();
	openal_fini();
	cab_view_fini();

	airportdb_destroy(airportdb);
	free(airportdb);
	airportdb = NULL;

	XPLMDestroyMenu(root_menu);
	XPLMRemoveMenuItem(XPLMFindPluginsMenu(), plugins_menu_item);
	XPLMUnregisterFlightLoopCallback(status_check, NULL);

	inited = B_FALSE;
}

static float
bp_do_reload(float u1, float u2, int u3, void *u4)
{
	UNUSED(u1);
	UNUSED(u2);
	UNUSED(u3);
	UNUSED(u4);
	if (reload_rqst) {
		bp_priv_disable();
		VERIFY(bp_priv_enable());
		reload_rqst = B_FALSE;
	}
	return (0);
}

void
bp_sched_reload(void)
{
	reload_rqst = B_TRUE;
	ASSERT(reload_floop_ID != NULL);
	XPLMScheduleFlightLoop(reload_floop_ID, -1, 1);
}
