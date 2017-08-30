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

#include <XPLMCamera.h>
#include <XPLMGraphics.h>
#include <XPLMScenery.h>
#include <XPLMUtilities.h>

#include <XPStandardWidgets.h>

#include <acfutils/assert.h>
#include <acfutils/dr.h>
#include <acfutils/intl.h>
#include <acfutils/widget.h>
#include <acfutils/time.h>

#include "bp.h"
#include "bp_cam.h"
#include "cab_view.h"
#include "xplane.h"

#define	INCR_SMALL	0.005
#define	INCR_MED	0.0125
#define	INCR_BIG	0.025
#define	INCR_ROT_SLOW	0.05
#define	INCR_ROT	0.2
#define	INCR_ROT_FAST	0.8
#define	INCR_ZOOM_SLOW	0.02
#define	INCR_ZOOM	0.04
#define	INCR_ZOOM_FAST	0.08

#define	HINTBAR_HEIGHT	20
#define	HINTBAR_TIMEOUT	SEC2USEC(5)

static bool_t started = B_FALSE;

/*
 * The delta to the tug's nominal camera position.
 * d_orient is the delta in X (heading) and Y (pitch).
 */
static vect3_t	d_pos = ZERO_VECT3;
static vect2_t	d_orient = ZERO_VECT2;
static double	zoom = 1.0;
static XPLMWindowID win = NULL;
static uint64_t hintbar_start = 0;
static XPWidgetID hintbar = NULL;
static dr_t cab_pos_dr;

static view_cmd_info_t view_cmds[] = {
    VCI_POS("sim/general/left",				-INCR_MED, 0, 0),
    VCI_POS("sim/general/right",			INCR_MED, 0, 0),
    VCI_POS("sim/general/up",				0, INCR_MED, 0),
    VCI_POS("sim/general/down",				0, -INCR_MED, 0),
    VCI_POS("sim/general/forward",			0, 0, INCR_MED),
    VCI_POS("sim/general/backward",			0, 0, -INCR_MED),
    VCI_ZOOM("sim/general/zoom_in",			INCR_ZOOM),
    VCI_ZOOM("sim/general/zoom_out",			-INCR_ZOOM),
    VCI_ROT("sim/general/hat_switch_left",		-INCR_ROT, 0),
    VCI_ROT("sim/general/hat_switch_right",		INCR_ROT, 0),
    VCI_ROT("sim/general/hat_switch_up",		0, INCR_ROT),
    VCI_ROT("sim/general/hat_switch_down",		0, -INCR_ROT),
    VCI_ROT("sim/general/hat_switch_up_left",		-INCR_ROT, INCR_ROT),
    VCI_ROT("sim/general/hat_switch_up_right",		INCR_ROT, INCR_ROT),
    VCI_ROT("sim/general/hat_switch_down_left",		-INCR_ROT, -INCR_ROT),
    VCI_ROT("sim/general/hat_switch_down_right",	-INCR_ROT, INCR_ROT),
    VCI_POS("sim/general/left_fast",			-INCR_BIG, 0, 0),
    VCI_POS("sim/general/right_fast",			INCR_BIG, 0, 0),
    VCI_POS("sim/general/up_fast",			0, INCR_BIG, 0),
    VCI_POS("sim/general/down_fast",			0, -INCR_BIG, 0),
    VCI_POS("sim/general/forward_fast",			0, 0, INCR_BIG),
    VCI_POS("sim/general/backward_fast",		0, 0, -INCR_BIG),
    VCI_ZOOM("sim/general/zoom_in_fast",		INCR_ZOOM_FAST),
    VCI_ZOOM("sim/general/zoom_out_fast",		-INCR_ZOOM_FAST),
    VCI_POS("sim/general/left_slow",			-INCR_SMALL, 0, 0),
    VCI_POS("sim/general/right_slow",			INCR_SMALL, 0, 0),
    VCI_POS("sim/general/up_slow",			0, INCR_SMALL, 0),
    VCI_POS("sim/general/down_slow",			0, -INCR_SMALL, 0),
    VCI_POS("sim/general/forward_slow",			0, 0, INCR_SMALL),
    VCI_POS("sim/general/backward_slow",		0, 0, -INCR_SMALL),
    VCI_ZOOM("sim/general/zoom_in_slow",		INCR_ZOOM_SLOW),
    VCI_ZOOM("sim/general/zoom_out_slow",		-INCR_ZOOM_SLOW),
    VCI_ROT("sim/general/rot_up",			0, INCR_ROT),
    VCI_ROT("sim/general/rot_down",			0, -INCR_ROT),
    VCI_ROT("sim/general/rot_left",			-INCR_ROT, 0),
    VCI_ROT("sim/general/rot_right",			INCR_ROT, 0),
    VCI_ROT("sim/general/rot_up_fast",			0, INCR_ROT_FAST),
    VCI_ROT("sim/general/rot_down_fast",		0, -INCR_ROT_FAST),
    VCI_ROT("sim/general/rot_left_fast",		-INCR_ROT_FAST, 0),
    VCI_ROT("sim/general/rot_right_fast",		INCR_ROT_FAST, 0),
    VCI_ROT("sim/general/rot_up_slow",			0, INCR_ROT_SLOW),
    VCI_ROT("sim/general/rot_down_slow",		0, -INCR_ROT_SLOW),
    VCI_ROT("sim/general/rot_left_slow",		-INCR_ROT_SLOW, 0),
    VCI_ROT("sim/general/rot_right_slow",		INCR_ROT_SLOW, 0),
    { .name = NULL }
};

void
cab_view_init(void)
{
	d_pos = ZERO_VECT3;
	d_orient = ZERO_VECT2;
	zoom = 1.0;
}

void
cab_view_fini(void)
{
	if (win != NULL) {
		XPLMDestroyWindow(win);
		win = NULL;
	}
}

bool_t
cab_view_can_start(void)
{
	return (!started && bp_started && bp_ls.tug != NULL);
}

static int
cam_ctl(XPLMCameraPosition_t *pos, int losing_control, void *refcon)
{
	XPLMProbeRef probe;
	XPLMProbeInfo_t info = { .structSize = sizeof (XPLMProbeInfo_t) };
	vect3_t tug_pos, norm, cam_pos, norm_hdg;

	UNUSED(refcon);

	if (pos == NULL || losing_control || !bp_started || !started) {
		cab_view_stop();
		return (0);
	}

	probe = XPLMCreateProbe(xplm_ProbeY);
	VERIFY3U(XPLMProbeTerrainXYZ(probe, bp_ls.tug->pos.pos.x, 0,
	    -bp_ls.tug->pos.pos.y, &info), ==, xplm_ProbeHitTerrain);
	/* Must be upright, no driving on ceilings! */
	ASSERT3F(info.normalY, >, 0.0);

	norm = VECT3(info.normalX, info.normalY, info.normalZ);
	tug_pos = VECT3(bp_ls.tug->pos.pos.x, info.locationY,
	    bp_ls.tug->pos.pos.y);

	cam_pos = vect3_add(bp_ls.tug->info->cam_pos, d_pos);
	cam_pos.y += bp_ls.tug->info->cab_lift_h * dr_getf(&cab_pos_dr);
	cam_pos = vect3_rot(cam_pos, -bp_ls.tug->pos.hdg, 1);
	cam_pos = vect3_add(cam_pos, tug_pos);

	norm_hdg = vect3_rot(VECT3(info.normalX, info.normalY, -info.normalZ),
	    bp_ls.tug->pos.hdg, 1);
	UNUSED(norm_hdg);
	UNUSED(norm);

	pos->x = cam_pos.x;
	pos->y = cam_pos.y;
	pos->z = -cam_pos.z;
	pos->heading = bp_ls.tug->pos.hdg + d_orient.x;
	pos->pitch = -RAD2DEG(atan(norm_hdg.z / norm_hdg.y)) + d_orient.y;
	pos->roll = RAD2DEG(atan(norm_hdg.x / norm_hdg.y));
	pos->zoom = zoom;

	XPLMDestroyProbe(probe);

	return (1);
}

static void
win_draw(XPLMWindowID inWindowID, void *inRefcon)
{
	int w, h;

	UNUSED(inWindowID);
	UNUSED(inRefcon);
	XPLMGetScreenSize(&w, &h);
	XPLMSetWindowGeometry(win, 0, h, w, 0);

	if (hintbar != NULL && microclock() - hintbar_start > HINTBAR_TIMEOUT) {
		XPDestroyWidget(hintbar, 1);
		hintbar = NULL;
	}
}

static void
win_key(XPLMWindowID inWindowID, char inKey, XPLMKeyFlags inFlags,
    char inVirtualKey, void *inRefcon, int losingFocus)
{
	UNUSED(inWindowID);
	UNUSED(inKey);
	UNUSED(inFlags);
	UNUSED(inVirtualKey);
	UNUSED(inRefcon);
	UNUSED(losingFocus);
}

static XPLMCursorStatus
win_cursor(XPLMWindowID inWindowID, int x, int y, void *inRefcon)
{
	UNUSED(inWindowID);
	UNUSED(x);
	UNUSED(y);
	UNUSED(inRefcon);
	return (xplm_CursorDefault);
}

static int
win_click(XPLMWindowID inWindowID, int x, int y, XPLMMouseStatus inMouse,
    void *inRefcon)
{
	static int down_x, down_y;

	UNUSED(inWindowID);
	UNUSED(x);
	UNUSED(y);
	UNUSED(inMouse);
	UNUSED(inRefcon);

	if (inMouse == xplm_MouseDrag) {
		d_orient.x += (x - down_x) / 3.0;
		d_orient.y += (y - down_y) / 3.0;
		d_orient.x = normalize_hdg(d_orient.x);
		d_orient.y = MIN(MAX(d_orient.y, -90), 90);
	}
	down_x = x;
	down_y = y;

	return (1);
}

static int
win_wheel(XPLMWindowID inWindowID, int x, int y, int wheel, int clicks,
    void *inRefcon)
{
	UNUSED(inWindowID);
	UNUSED(x);
	UNUSED(y);
	UNUSED(wheel);
	UNUSED(clicks);
	UNUSED(inRefcon);

	if (clicks > 0) {
		for (int i = clicks; i > 0; i--)
			zoom *= 1 + INCR_ZOOM;
	} else {
		for (int i = clicks; i < 0; i++)
			zoom *= 1 - INCR_ZOOM;
	}

	return (1);
}

static int
move_view_handler(XPLMCommandRef cmd, XPLMCommandPhase phase, void *refcon)
{
	view_cmd_info_t *ci;
	vect3_t v;

	UNUSED(cmd);

	if (phase == xplm_CommandBegin || phase == xplm_CommandEnd)
		return (0);
	ci = &view_cmds[(uintptr_t)refcon];

	d_orient.x = normalize_hdg(d_orient.x + ci->rot.x);
	d_orient.y = MIN(MAX(d_orient.y + ci->rot.y, -90), 90);
	v = vect3_rot(vect3_rot(ci->pos, d_orient.y, 0), -d_orient.x, 1);
	d_pos = vect3_add(d_pos, v);
	zoom *= ci->zoom;

	return (0);
}

bool_t
cab_view_start(void)
{
	if (!cab_view_can_start())
		return (B_FALSE);

	fdr_find(&cab_pos_dr, "bp/anim/cab_position");
	/*
	 * To avoid aircraft parts clipping over scenery, we need to first
	 * shift the view to an external one, before controlling the camera.
	 */
	XPLMCommandOnce(XPLMFindCommand("sim/view/circle"));

	XPLMControlCamera(xplm_ControlCameraUntilViewChanges, cam_ctl, NULL);
	started = B_TRUE;

	if (win == NULL) {
		XPLMCreateWindow_t win_ops = {
		    .structSize = sizeof (XPLMCreateWindow_t),
		    .left = 0, .top = 0, .right = 0, .bottom = 0, .visible = 1,
		    .drawWindowFunc = win_draw,
		    .handleMouseClickFunc = win_click,
		    .handleKeyFunc = win_key,
		    .handleCursorFunc = win_cursor,
		    .handleMouseWheelFunc = win_wheel,
		    .refcon = NULL
		};
		win = XPLMCreateWindowEx(&win_ops);
	}
	ASSERT(win != NULL);
	XPLMBringWindowToFront(win);

	/*
	 * Because in the cab view we're using the left mouse button to
	 * reorient the view, display a short message for 5 seconds to let
	 * the user know of this difference.
	 */
	if (hintbar == NULL) {
		const char *str = _("Left mouse button to reorient view");
		int x, y;
		int w = XPLMMeasureString(xplmFont_Proportional,
		    str, strlen(str));
		XPWidgetID caption;

		XPLMGetScreenSize(&x, &y);
		hintbar = create_widget_rel((x - w) / 2 - 10,
		    HINTBAR_HEIGHT * 3, B_FALSE, w + 20,
		    HINTBAR_HEIGHT, 0, "", 1, NULL, xpWidgetClass_MainWindow);
		XPSetWidgetProperty(hintbar, xpProperty_MainWindowType,
		    xpMainWindowStyle_Translucent);

		caption = create_widget_rel(10, 0, B_FALSE, w, HINTBAR_HEIGHT,
		    1, str, 0, hintbar, xpWidgetClass_Caption);
		XPSetWidgetProperty(caption, xpProperty_CaptionLit, 1);

		XPShowWidget(hintbar);
		hintbar_start = microclock();
	}

	for (int i = 0; view_cmds[i].name != NULL; i++) {
		if (view_cmds[i].cmd == NULL) {
			view_cmds[i].cmd = XPLMFindCommand(view_cmds[i].name);
			VERIFY_MSG(view_cmds[i].cmd != NULL, "%s",
			    view_cmds[i].name);
			XPLMRegisterCommandHandler(view_cmds[i].cmd,
			    move_view_handler, 1, (void *)(uintptr_t)i);
		}
	}

	return (B_TRUE);
}

void
cab_view_stop(void)
{
	if (win != NULL) {
		XPLMDestroyWindow(win);
		win = NULL;
	}
	if (hintbar != NULL) {
		XPDestroyWidget(hintbar, 1);
		hintbar = NULL;
	}

	for (int i = 0; view_cmds[i].name != NULL; i++) {
		if (view_cmds[i].cmd != NULL) {
			XPLMUnregisterCommandHandler(view_cmds[i].cmd,
			    move_view_handler, 1, (void *)(uintptr_t)i);
			view_cmds[i].cmd = NULL;
		}
	}

	started = B_FALSE;
}
