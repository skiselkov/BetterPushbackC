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

#include <string.h>
#include <stddef.h>

#if	IBM
#include <gl.h>
#elif	APL
#include <OpenGL/gl.h>
#else	/* LIN */
#include <GL/gl.h>
#endif	/* LIN */

#include <XPLMCamera.h>
#include <XPLMDisplay.h>
#include <XPLMGraphics.h>
#include <XPLMScenery.h>
#include <XPLMUtilities.h>
#include <XPLMProcessing.h>

#include <acfutils/assert.h>
#include <acfutils/geom.h>
#include <acfutils/math.h>
#include <acfutils/list.h>
#include <acfutils/time.h>

#include "bp.h"
#include "dr.h"
#include "driving.h"
#include "xplane.h"

#define	PB_DEBUG_INTF

#define	STRAIGHT_STEER_RATE	40	/* degrees per second */
#define	TURN_STEER_RATE		10	/* degrees per second */
#define	NORMAL_ACCEL		0.25	/* m/s^2 */
#define	BRAKE_PEDAL_THRESH	0.1	/* brake pedal angle, 0..1 */
#define	FORCE_PER_TON		5000	/* max push force per ton, Newtons */
#define	BREAKAWAY_THRESH	0.1	/* m/s */
#define	SEG_TURN_MULT		0.9	/* leave 10% for oversteer */
#define	SPEED_COMPLETE_THRESH	0.05	/* m/s */
#define	MAX_STEER_ANGLE		60	/* beyond this our push algos go nuts */
#define	MAX_ANG_VEL		2.5	/* degrees per second */

#ifdef	DEBUG
static vect2_t	c_pt = NULL_VECT2, s_pt = NULL_VECT2;
static double	line_hdg = NAN;
#endif	/* DEBUG */

typedef struct {
	double		nw_z, main_z;
} acf_t;
typedef struct {
	vehicle_t	veh;
	acf_t		acf;		/* our aircraft */

	vehicle_pos_t	cur_pos;
	vehicle_pos_t	last_pos;

	double		cur_t;		/* current time in seconds */
	double		last_t;		/* cur_t from previous run */
	double		last_mis_hdg;	/* previous steering misalignment */

	/* deltas from last_* to cur_* */
	vehicle_pos_t	d_pos;		/* delta from last_pos to cur_pos */
	double		d_t;		/* delta time from last_t to cur_t */

	double		last_force;
	vect2_t		turn_c_pos;

	bool_t		starting;	/* waiting for brake release */
	bool_t		stopping;	/* stopping at end of operation */
	bool_t		stopped;	/* stopped moving, waiting for pbrk */

	list_t		segs;
} bp_state_t;

static struct {
	dr_t	lbrake, rbrake;
	dr_t	pbrake;
	dr_t	rot_force_N;
	dr_t	axial_force;
	dr_t	local_x, local_y, local_z;
	dr_t	hdg;
	dr_t	vx, vy, vz;
	dr_t	sim_time;
	dr_t	acf_mass;
	dr_t	tire_z;
	dr_t	nw_steerdeg1, nw_steerdeg2;
	dr_t	tire_steer_cmd;
	dr_t	override_steer;
	dr_t	gear_deploy;

	dr_t	camera_fov_h, camera_fov_v;
	dr_t	view_is_ext;
} drs;

static bool_t inited = B_FALSE, cam_inited = B_FALSE, started = B_FALSE;
static bp_state_t bp;

static float bp_run(void);

static void
turn_nosewheel(double req_angle, double rate)
{
	double rate_of_turn, steer_incr, cur_nw_angle;

	cur_nw_angle = dr_getf(&drs.tire_steer_cmd);
	if (cur_nw_angle == req_angle)
		return;

	/*
	 * Modulate the steering increment to always be
	 * correctly within our rate of turn limits.
	 */
	rate_of_turn = rate * bp.d_t;
	steer_incr = MIN(ABS(req_angle - cur_nw_angle), rate_of_turn);
	cur_nw_angle += (cur_nw_angle < req_angle ? steer_incr : -steer_incr);
	/* prevent excessive deflection */
	cur_nw_angle = MIN(cur_nw_angle, bp.veh.max_steer);
	cur_nw_angle = MAX(cur_nw_angle, -bp.veh.max_steer);
	dr_setf(&drs.tire_steer_cmd, cur_nw_angle);
}

static void
push_at_speed(double targ_speed, double max_accel)
{
	double force_lim, force_incr, force, angle_rad, accel_now, d_v, Fx, Fz;

	/*
	 * Multiply force limit by weight in tons - that's at most how
	 * hard we'll try to push the aircraft. This prevents us from
	 * flinging the aircraft across the tarmac in case some external
	 * factor is blocking us (like chocks).
	 */
	force_lim = FORCE_PER_TON * (dr_getf(&drs.acf_mass) / 1000);
	/*
	 * The maximum single-second force increment is 1/10 of the maximum
	 * pushback force limit. This means it'll take up to 10s for us to
	 * apply full pushback force.
	 */
	force_incr = (force_lim / 10) * bp.d_t;

	force = bp.last_force;
	accel_now = (bp.cur_pos.spd - bp.last_pos.spd) / bp.d_t;
	d_v = targ_speed - bp.cur_pos.spd;

	/*
	 * Calculate the vector components of our force on the aircraft
	 * to correctly apply angular momentum forces below.
	 * N.B. we only push in the horizontal plane, hence no Fy component.
	 */
	angle_rad = DEG2RAD(dr_getf(&drs.tire_steer_cmd));
	Fx = -force * sin(angle_rad);
	Fz = force * cos(angle_rad);

	dr_setf(&drs.axial_force, dr_getf(&drs.axial_force) + Fz);
	dr_setf(&drs.rot_force_N, dr_getf(&drs.rot_force_N) -
	    Fx * bp.acf.nw_z);

	/*
	 * This is some fudge needed to get some high-thrust aircraft
	 * going, otherwise we'll just jitter in-place due to thinking
	 * we're overdoing acceleration.
	 */
	if (ABS(bp.cur_pos.spd) < BREAKAWAY_THRESH)
		max_accel *= 100;

	if (d_v > 0) {
		if (d_v < max_accel && ABS(bp.cur_pos.spd) >= BREAKAWAY_THRESH)
			max_accel = d_v;
		if (accel_now > max_accel)
			force += force_incr;
		else if (accel_now < max_accel)
			force -= force_incr;
	} else if (d_v < 0) {
		max_accel *= -1;
		if (d_v > max_accel && ABS(bp.cur_pos.spd) >= BREAKAWAY_THRESH)
			max_accel = d_v;
		if (accel_now < max_accel)
			force -= force_incr;
		else if (accel_now > max_accel)
			force += force_incr;
	}

	/* Don't overstep the force limits for this aircraft */
	force = MIN(force_lim, force);
	force = MAX(-force_lim, force);

	bp.last_force = force;
}

static bool_t
bp_state_init(void)
{
	double tire_z_main[8];
	int n_main;

	memset(&bp, 0, sizeof (bp));
	list_create(&bp.segs, sizeof (seg_t), offsetof(seg_t, node));

	bp.turn_c_pos = NULL_VECT2;

	dr_getvf(&drs.tire_z, &bp.acf.nw_z, 0, 1);
	n_main = dr_getvf(&drs.tire_z, tire_z_main, 1, 8);
	if (n_main < 1) {
		XPLMSpeakString("Pushback failure: aircraft seems to only "
		    "have one gear leg.");
		return (B_FALSE);
	}
	for (int i = 0; i < n_main; i++)
		bp.acf.main_z += tire_z_main[i];
	bp.acf.main_z /= n_main;
	bp.veh.wheelbase = bp.acf.main_z - bp.acf.nw_z;
	if (bp.veh.wheelbase <= 0) {
		XPLMSpeakString("Pushback failure: aircraft has non-positive "
		    "wheelbase. Sorry, tail draggers aren't supported.");
		return (B_FALSE);
	}
	bp.veh.max_steer = MIN(MAX(dr_getf(&drs.nw_steerdeg1),
	    dr_getf(&drs.nw_steerdeg2)), MAX_STEER_ANGLE);

	return (B_TRUE);
}

bool_t
bp_init(void)
{
	if (inited)
		return (B_TRUE);

	memset(&drs, 0, sizeof (drs));

	dr_init(&drs.lbrake, "sim/cockpit2/controls/left_brake_ratio");
	dr_init(&drs.rbrake, "sim/cockpit2/controls/right_brake_ratio");
	if (XPLMFindDataRef("model/controls/park_break") != NULL)
		dr_init(&drs.pbrake, "model/controls/park_break");
	else
		dr_init(&drs.pbrake, "sim/flightmodel/controls/parkbrake");
	dr_init(&drs.rot_force_N, "sim/flightmodel/forces/N_plug_acf");
	dr_init(&drs.axial_force, "sim/flightmodel/forces/faxil_plug_acf");
	dr_init(&drs.local_x, "sim/flightmodel/position/local_x");
	dr_init(&drs.local_y, "sim/flightmodel/position/local_y");
	dr_init(&drs.local_z, "sim/flightmodel/position/local_z");
	dr_init(&drs.hdg, "sim/flightmodel/position/psi");
	dr_init(&drs.vx, "sim/flightmodel/position/local_vx");
	dr_init(&drs.vy, "sim/flightmodel/position/local_vy");
	dr_init(&drs.vz, "sim/flightmodel/position/local_vz");
	dr_init(&drs.sim_time, "sim/time/total_running_time_sec");
	dr_init(&drs.acf_mass, "sim/flightmodel/weight/m_total");
	dr_init(&drs.tire_z, "sim/flightmodel/parts/tire_z_no_deflection");
	dr_init(&drs.nw_steerdeg1, "sim/aircraft/gear/acf_nw_steerdeg1");
	dr_init(&drs.nw_steerdeg2, "sim/aircraft/gear/acf_nw_steerdeg2");
	dr_init(&drs.tire_steer_cmd,
	    "sim/flightmodel/parts/tire_steer_cmd");
	dr_init(&drs.override_steer,
	    "sim/operation/override/override_wheel_steer");
	dr_init(&drs.gear_deploy, "sim/aircraft/parts/acf_gear_deploy");

	dr_init(&drs.camera_fov_h,
	    "sim/graphics/view/field_of_view_deg");
	dr_init(&drs.camera_fov_v,
	    "sim/graphics/view/vertical_field_of_view_deg");
	dr_init(&drs.view_is_ext, "sim/graphics/view/view_is_external");

	if (!bp_state_init())
		return (B_FALSE);

	inited = B_TRUE;

	return (B_TRUE);
}

bool_t
bp_can_start(char **reason)
{
	seg_t *seg;
	vect2_t pos;

	if (dr_getf(&drs.gear_deploy) != 1) {
		if (reason != NULL)
			*reason = "Pushback failure: gear not extended.";
		return (B_FALSE);
	}
	if (vect3_abs(VECT3(dr_getf(&drs.vx), dr_getf(&drs.vy),
	    dr_getf(&drs.vz))) >= 1) {
		if (reason != NULL)
			*reason = "Pushback failure: aircraft not stationary.";
		return (B_FALSE);
	}

	seg = list_head(&bp.segs);
	if (seg == NULL) {
		if (reason != NULL) {
			*reason = "Pushback failure: please first plan your "
			    "pushback to tell me where you want to go.";
		}
		return (B_FALSE);
	}
	pos = VECT2(dr_getf(&drs.local_x), -dr_getf(&drs.local_z));
	if (vect2_dist(pos, seg->start_pos) >= 3) {
		if (reason != NULL) {
			*reason = "Pushback failure: aircraft has moved. "
			    "Please plan a new pushback path.";
		}
		do {
			list_remove(&bp.segs, seg);
			free(seg);
			seg = list_head(&bp.segs);
		} while (seg != NULL);
		return (B_FALSE);
	}

	return (B_TRUE);
}

bool_t
bp_start(void)
{
	char *reason;

	if (started)
		return (B_TRUE);
	if (!bp_can_start(&reason)) {
		XPLMSpeakString(reason);
		return (B_FALSE);
	}

	if (dr_getf(&drs.pbrake) == 0) {
		XPLMSpeakString("Ground to cockpit. Cannot connect tow. "
		    "Please set the parking brake first.");
		return (B_FALSE);
	}

	XPLMRegisterFlightLoopCallback((XPLMFlightLoop_f)bp_run, -1, NULL);
	started = B_TRUE;

	if (dr_getf(&drs.pbrake) == 1) {
		bp.starting = B_TRUE;
		XPLMSpeakString("Ground to cockpit. Tow is connected and "
		    "bypass pin has been inserted. Release parking brake.");
	}

	return (B_TRUE);
}

bool_t
bp_stop(void)
{
	if (!started)
		return (B_FALSE);
	/* Delete all pushback segments, that'll stop us */
	for (seg_t *seg = list_head(&bp.segs); seg != NULL;
	    seg = list_head(&bp.segs)) {
		list_remove_head(&bp.segs);
		free(seg);
	}

	return (B_TRUE);
}

void
bp_fini(void)
{
	if (!inited)
		return;

	bp_stop();

	dr_seti(&drs.override_steer, 0);

	if (started) {
		XPLMUnregisterFlightLoopCallback((XPLMFlightLoop_f)bp_run,
		    NULL);
		started = B_FALSE;
	}

	for (seg_t *seg = list_head(&bp.segs); seg != NULL;
	    seg = list_head(&bp.segs)) {
		list_remove_head(&bp.segs);
		free(seg);
	}
	list_destroy(&bp.segs);

	inited = B_FALSE;
}

static void
bp_gather(void)
{
	/*
	 * CAREFUL!
	 * X-Plane's north-south axis (Z) is flipped to our understanding, so
	 * whenever we access 'local_z' or 'vz', we need to flip it.
	 */
	bp.cur_pos.pos = VECT2(dr_getf(&drs.local_x),
	    -dr_getf(&drs.local_z));
	bp.cur_pos.hdg = dr_getf(&drs.hdg);
	bp.cur_pos.spd = vect2_dotprod(hdg2dir(bp.cur_pos.hdg),
	    VECT2(dr_getf(&drs.vx), -dr_getf(&drs.vz)));
	bp.cur_t = dr_getf(&drs.sim_time);
}

static float
bp_run(void)
{
	seg_t *seg;
	bool_t last = B_FALSE;

	bp_gather();

	if (bp.cur_t <= bp.last_t)
		return (B_TRUE);

	bp.d_pos.pos = vect2_sub(bp.cur_pos.pos, bp.last_pos.pos);
	bp.d_pos.hdg = bp.cur_pos.hdg - bp.last_pos.hdg;
	bp.d_pos.spd = bp.cur_pos.spd - bp.last_pos.spd;
	bp.d_t = bp.cur_t - bp.last_t;

	dr_seti(&drs.override_steer, 1);

	if (bp.starting && dr_getf(&drs.pbrake) != 1) {
		seg = list_head(&bp.segs);
		ASSERT(seg != NULL);
		if (seg->backward)
			XPLMSpeakString("Starting pushback!");
		else
			XPLMSpeakString("Starting tow!");
		bp.starting = B_FALSE;
	}

	while ((seg = list_head(&bp.segs)) != NULL) {
		double steer, speed;

		last = B_TRUE;
		/* Pilot pressed brake pedals or set parking brake, stop */
		if (dr_getf(&drs.lbrake) > BRAKE_PEDAL_THRESH ||
		    dr_getf(&drs.rbrake) > BRAKE_PEDAL_THRESH ||
		    dr_getf(&drs.pbrake) != 0)
			break;
		if (drive_segs(&bp.cur_pos, &bp.veh, &bp.segs, MAX_ANG_VEL,
		    &bp.last_mis_hdg, bp.d_t, &steer, &speed)) {
			double steer_rate = (seg->type == SEG_TYPE_STRAIGHT ?
			    STRAIGHT_STEER_RATE : TURN_STEER_RATE);
			turn_nosewheel(steer, steer_rate);
			push_at_speed(speed, NORMAL_ACCEL);
			break;
		}
	}

	bp.last_pos = bp.cur_pos;
	bp.last_t = bp.cur_t;

	if (seg != NULL) {
		return (-1);
	} else {
		if (last)
			bp.stopping = B_TRUE;
		turn_nosewheel(0, STRAIGHT_STEER_RATE);
		push_at_speed(0, NORMAL_ACCEL);
		if (ABS(bp.cur_pos.spd) < SPEED_COMPLETE_THRESH &&
		    !bp.stopped) {
			XPLMSpeakString("Operation complete, set parking "
			    "brake.");
			bp.stopped = B_TRUE;
		}
		if (bp.stopped) {
			/*
			 * Apply the brakes to prevent the aircraft from
			 * attempting to move while we're waiting for the
			 * parking brake to be set.
			 */
			dr_setf(&drs.lbrake, 1);
			dr_setf(&drs.rbrake, 1);
		}
		if (dr_getf(&drs.pbrake) == 0) {
			return (-1);
		}
		dr_setf(&drs.lbrake, 0);
		dr_setf(&drs.rbrake, 0);
		dr_seti(&drs.override_steer, 0);
		started = B_FALSE;
		XPLMSpeakString("Tow is disconnected and steering pin has "
		    "been removed. Have a nice day!");
		bp_done_notify();

		/*
		 * Reinitialize our state so we're starting with a clean
		 * slate next time.
		 */
		bp_state_init();

		return (0);
	}
}

static vect3_t cam_pos;
static double cam_height;
static double cam_hdg;
static double cursor_hdg;
static list_t pred_segs;
static XPLMCommandRef circle_view_cmd;
static XPLMWindowID fake_win;
static vect2_t cursor_world_pos;
static bool_t force_root_win_focus = B_TRUE;

#define	ABV_TERR_HEIGHT		1.5	/* meters */
#define	MAX_PRED_DISTANCE	10000	/* meters */
#define	ANGLE_DRAW_STEP		5
#define	ORIENTATION_LINE_LEN	200

#define	INCR_SMALL		5
#define	INCR_MED		25
#define	INCR_BIG		125

#define	AMBER_TUPLE		VECT3(0.9, 0.9, 0)	/* RGB color */
#define	RED_TUPLE		VECT3(1, 0, 0)		/* RGB color */
#define	GREEN_TUPLE		VECT3(0, 1, 0)		/* RGB color */

#define	CLICK_THRESHOLD_US	200000			/* microseconds */
#define	US_PER_CLICK_ACCEL	60000			/* microseconds */
#define	US_PER_CLICK_DEACCEL	120000			/* microseconds */
#define	MAX_ACCEL_MULT		10
#define	WHEEL_ANGLE_MULT	0.5

#define	PREDICTION_DRAWING_PHASE	xplm_Phase_Airplanes

typedef struct {
	const char	*name;
	XPLMCommandRef	cmd;
	vect3_t		incr;
} view_cmd_info_t;

static view_cmd_info_t view_cmds[] = {
    { .name = "sim/general/left", .incr = VECT3(-INCR_MED, 0, 0) },
    { .name = "sim/general/right", .incr = VECT3(INCR_MED, 0, 0) },
    { .name  = "sim/general/up", .incr = VECT3(0, 0, INCR_MED) },
    { .name = "sim/general/down", .incr = VECT3(0, 0, -INCR_MED) },
    { .name = "sim/general/forward", .incr = VECT3(0, -INCR_MED, 0) },
    { .name = "sim/general/backward", .incr = VECT3(0, INCR_MED, 0) },
    { .name = "sim/general/zoom_in", .incr = VECT3(0, -INCR_MED, 0) },
    { .name = "sim/general/zoom_out", .incr = VECT3(0, INCR_MED, 0) },
    { .name = "sim/general/hat_switch_left", .incr = VECT3(-INCR_MED, 0, 0) },
    { .name = "sim/general/hat_switch_right", .incr = VECT3(INCR_MED, 0, 0) },
    { .name = "sim/general/hat_switch_up", .incr = VECT3(0, 0, INCR_MED) },
    { .name = "sim/general/hat_switch_down", .incr = VECT3(0, 0, -INCR_MED) },
    { .name = "sim/general/hat_switch_up_left",
	.incr = VECT3(-INCR_MED, 0, INCR_MED) },
    { .name = "sim/general/hat_switch_up_right",
	.incr = VECT3(INCR_MED, 0, INCR_MED) },
    { .name = "sim/general/hat_switch_down_left",
	.incr = VECT3(-INCR_MED, 0, -INCR_MED) },
    { .name = "sim/general/hat_switch_down_right",
	.incr = VECT3(INCR_MED, 0, -INCR_MED) },
    { .name = "sim/general/left_fast", .incr = VECT3(-INCR_BIG, 0, 0) },
    { .name = "sim/general/right_fast", .incr = VECT3(INCR_BIG, 0, 0) },
    { .name = "sim/general/up_fast", .incr = VECT3(0, 0, INCR_BIG) },
    { .name = "sim/general/down_fast", .incr = VECT3(0, 0, -INCR_BIG) },
    { .name = "sim/general/forward_fast", .incr = VECT3(0, -INCR_BIG, 0) },
    { .name = "sim/general/backward_fast", .incr = VECT3(0, INCR_BIG, 0) },
    { .name = "sim/general/zoom_in_fast", .incr = VECT3(0, -INCR_BIG, 0) },
    { .name = "sim/general/zoom_out_fast", .incr = VECT3(0, INCR_BIG, 0) },
    { .name = "sim/general/left_slow", .incr = VECT3(-INCR_SMALL, 0, 0) },
    { .name = "sim/general/right_slow", .incr = VECT3(INCR_SMALL, 0, 0) },
    { .name = "sim/general/up_slow", .incr = VECT3(0, 0, INCR_SMALL) },
    { .name = "sim/general/down_slow", .incr = VECT3(0, 0, -INCR_SMALL) },
    { .name = "sim/general/forward_slow", .incr = VECT3(0, -INCR_SMALL, 0) },
    { .name = "sim/general/backward_slow", .incr = VECT3(0, INCR_SMALL, 0) },
    { .name = "sim/general/zoom_in_slow", .incr = VECT3(0, -INCR_SMALL, 0) },
    { .name = "sim/general/zoom_out_slow", .incr = VECT3(0, INCR_SMALL, 0) },
    { .name = NULL }
};

static int
move_camera(XPLMCommandRef cmd, XPLMCommandPhase phase, void *refcon)
{
	static uint64_t last_cmd_t = 0;
	UNUSED(cmd);

	if (phase == xplm_CommandBegin) {
		last_cmd_t = microclock();
	} else if (phase == xplm_CommandContinue) {
		uint64_t now = microclock();
		double d_t = (now - last_cmd_t) / 1000000.0;
		unsigned i = (uintptr_t)refcon;
		vect2_t v = vect2_rot(VECT2(view_cmds[i].incr.x,
		    view_cmds[i].incr.z), cam_hdg);
		cam_pos = vect3_add(cam_pos, VECT3(v.x * d_t, 0, v.y * d_t));
		cam_height += view_cmds[i].incr.y * d_t;
		last_cmd_t = now;
	}
	return (0);
}

static int
cam_ctl(XPLMCameraPosition_t *pos, int losing_control, void *refcon)
{
	int x, y, w, h;
	double rx, ry, dx, dy, rw, rh;
	double fov_h, fov_v;
	vect2_t start_pos, end_pos;
	double start_hdg;
	seg_t *seg;
	int n;

	UNUSED(refcon);

	if (pos == NULL || losing_control || !cam_inited)
		return (0);

	pos->x = cam_pos.x;
	pos->y = cam_pos.y + cam_height;
	pos->z = -cam_pos.z;
	pos->pitch = -90;
	pos->heading = cam_hdg;
	pos->roll = 0;
	pos->zoom = 1;

	XPLMGetMouseLocation(&x, &y);
	XPLMGetScreenSize(&w, &h);
	/* make the mouse coordinates relative to the screen center */
	rx = ((double)x - w / 2) / (w / 2);
	ry = ((double)y - h / 2) / (h / 2);
	fov_h = DEG2RAD(dr_getf(&drs.camera_fov_h));
	fov_v = DEG2RAD(dr_getf(&drs.camera_fov_v));
	rw = cam_height * tan(fov_h / 2);
	rh = cam_height * tan(fov_v / 2);
	dx = rw * rx;
	dy = rh * ry;

	/*
	 * Don't make predictions if due to the camera FOV angle (>= 180 deg)
	 * we could be placing the prediction object very far away.
	 */
	if (dx > MAX_PRED_DISTANCE || dy > MAX_PRED_DISTANCE)
		return (1);

	for (seg = list_head(&pred_segs); seg != NULL;
	    seg = list_head(&pred_segs)) {
		list_remove(&pred_segs, seg);
		free(seg);
	}

	seg = list_tail(&bp.segs);
	if (seg != NULL) {
		start_pos = seg->end_pos;
		start_hdg = seg->end_hdg;
	} else {
		start_pos = VECT2(dr_getf(&drs.local_x),
		    -dr_getf(&drs.local_z));	/* inverted X-Plane Z */
		start_hdg = dr_getf(&drs.hdg);
	}

	end_pos = vect2_add(VECT2(cam_pos.x, cam_pos.z),
	    vect2_rot(VECT2(dx, dy), pos->heading));
	cursor_world_pos = VECT2(end_pos.x, end_pos.y);

	n = compute_segs(&bp.veh, start_pos, start_hdg, end_pos,
	    cursor_hdg, &pred_segs);
	if (n > 0) {
		seg = list_tail(&pred_segs);
		seg->user_placed = B_TRUE;
	}

	return (1);
}

static void
draw_segment(const seg_t *seg)
{
	XPLMProbeRef probe = XPLMCreateProbe(xplm_ProbeY);
	XPLMProbeInfo_t info = { .structSize = sizeof (XPLMProbeInfo_t) };

	glColor3f(0, 0, 1);
	glLineWidth(3);

	switch (seg->type) {
	case SEG_TYPE_STRAIGHT:
		glBegin(GL_LINES);
		VERIFY3U(XPLMProbeTerrainXYZ(probe, seg->start_pos.x, 0,
		    -seg->start_pos.y, &info), ==, xplm_ProbeHitTerrain);
		glVertex3f(seg->start_pos.x, info.locationY + ABV_TERR_HEIGHT,
		    -seg->start_pos.y);
		VERIFY3U(XPLMProbeTerrainXYZ(probe, seg->end_pos.x, 0,
		    -seg->end_pos.y, &info), ==, xplm_ProbeHitTerrain);
		glVertex3f(seg->end_pos.x, info.locationY + ABV_TERR_HEIGHT,
		    -seg->end_pos.y);
		glEnd();
		break;
	case SEG_TYPE_TURN: {
		vect2_t c = vect2_add(seg->start_pos, vect2_scmul(
		    vect2_norm(hdg2dir(seg->start_hdg), seg->turn.right),
		    seg->turn.r));
		vect2_t c2s = vect2_sub(seg->start_pos, c);
		double s, e, rhdg;

		glBegin(GL_LINES);
		rhdg = rel_hdg(seg->start_hdg, seg->end_hdg);
		s = MIN(0, rhdg);
		e = MAX(0, rhdg);
		ASSERT3F(s, <=, e);
		for (double a = s; a < e; a += ANGLE_DRAW_STEP) {
			vect2_t p;
			double s = MIN(ANGLE_DRAW_STEP, e - a);
			p = vect2_add(c, vect2_rot(c2s, a));
			VERIFY3U(XPLMProbeTerrainXYZ(probe, p.x, 0, -p.y,
			    &info), ==, xplm_ProbeHitTerrain);
			glVertex3f(p.x, info.locationY + ABV_TERR_HEIGHT, -p.y);
			p = vect2_add(c, vect2_rot(c2s, a + s));
			glVertex3f(p.x, info.locationY + ABV_TERR_HEIGHT, -p.y);
		}
		glEnd();
		break;
	}
	}

	XPLMDestroyProbe(probe);
}

static void
draw_acf_symbol(vect3_t pos, double hdg, double wheelbase, vect3_t color)
{
	vect2_t v;
	vect3_t p;

	/*
	 * The wheelbase of most airlines is very roughly 1/3 of their
	 * total length, so multiplying by 1.5 makes this value approx
	 * half their size and gives a good rough "size ring" radius.
	 */
	wheelbase *= 1.5;

	glLineWidth(4);
	glColor3f(color.x, color.y, color.z);

	glBegin(GL_LINES);
	v = vect2_rot(VECT2(-wheelbase, 0), hdg);
	p = vect3_add(pos, VECT3(v.x, 0, v.y));
	glVertex3f(p.x, p.y, -p.z);
	v = vect2_rot(VECT2(wheelbase, 0), hdg);
	p = vect3_add(pos, VECT3(v.x, 0, v.y));
	glVertex3f(p.x, p.y, -p.z);
	v = vect2_rot(VECT2(0, wheelbase / 2), hdg);
	p = vect3_add(pos, VECT3(v.x, 0, v.y));
	glVertex3f(p.x, p.y, -p.z);
	v = vect2_rot(VECT2(0, -wheelbase), hdg);
	p = vect3_add(pos, VECT3(v.x, 0, v.y));
	glVertex3f(p.x, p.y, -p.z);
	v = vect2_rot(VECT2(-wheelbase / 2, -wheelbase), hdg);
	p = vect3_add(pos, VECT3(v.x, 0, v.y));
	glVertex3f(p.x, p.y, -p.z);
	v = vect2_rot(VECT2(wheelbase / 2, -wheelbase), hdg);
	p = vect3_add(pos, VECT3(v.x, 0, v.y));
	glVertex3f(p.x, p.y, -p.z);
	glEnd();
}

#ifdef	DEBUG
static void
draw_cross(vect2_t pos, double y)
{
	vect2_t v;

	glLineWidth(4);
	glColor3f(1, 0, 1);

	glBegin(GL_LINES);
	v = vect2_add(vect2_rot(VECT2(-1, 0), cam_hdg), pos);
	glVertex3f(v.x, y, -v.y);
	v = vect2_add(vect2_rot(VECT2(1, 0), cam_hdg), pos);
	glVertex3f(v.x, y, -v.y);
	v = vect2_add(vect2_rot(VECT2(0, 1), cam_hdg), pos);
	glVertex3f(v.x, y, -v.y);
	v = vect2_add(vect2_rot(VECT2(0, -1), cam_hdg), pos);
	glVertex3f(v.x, y, -v.y);
	glEnd();
}
#endif	/* DEBUG */

static int
draw_prediction(XPLMDrawingPhase phase, int before, void *refcon)
{
	seg_t *seg;
	XPLMProbeRef probe = XPLMCreateProbe(xplm_ProbeY);
	XPLMProbeInfo_t info = { .structSize = sizeof (XPLMProbeInfo_t) };

	UNUSED(phase);
	UNUSED(before);
	UNUSED(refcon);

	if (dr_geti(&drs.view_is_ext) != 1)
		XPLMCommandOnce(circle_view_cmd);

	XPLMSetGraphicsState(0, 0, 0, 0, 0, 0, 0);

	for (seg = list_head(&bp.segs); seg != NULL;
	    seg = list_next(&bp.segs, seg))
		draw_segment(seg);

	for (seg = list_head(&pred_segs); seg != NULL;
	    seg = list_next(&pred_segs, seg))
		draw_segment(seg);

	if ((seg = list_tail(&pred_segs)) != NULL) {
		vect2_t dir_v = hdg2dir(seg->end_hdg);
		vect2_t x;

		VERIFY3U(XPLMProbeTerrainXYZ(probe, seg->end_pos.x, 0,
		    -seg->end_pos.y, &info), ==, xplm_ProbeHitTerrain);
		if (seg->type == SEG_TYPE_TURN || !seg->backward) {
			glBegin(GL_LINES);
			glColor3f(0, 1, 0);
			glVertex3f(seg->end_pos.x, info.locationY +
			    ABV_TERR_HEIGHT, -seg->end_pos.y);
			x = vect2_add(seg->end_pos, vect2_scmul(dir_v,
			    ORIENTATION_LINE_LEN));
			glVertex3f(x.x, info.locationY + ABV_TERR_HEIGHT, -x.y);
			glEnd();
		}
		if (seg->type == SEG_TYPE_TURN || seg->backward) {
			glBegin(GL_LINES);
			glColor3f(1, 0, 0);
			glVertex3f(seg->end_pos.x, info.locationY +
			    ABV_TERR_HEIGHT, -seg->end_pos.y);
			x = vect2_add(seg->end_pos, vect2_neg(vect2_scmul(
			    dir_v, ORIENTATION_LINE_LEN)));
			glVertex3f(x.x, info.locationY + ABV_TERR_HEIGHT, -x.y);
			glEnd();
		}
		draw_acf_symbol(VECT3(seg->end_pos.x, info.locationY +
		    ABV_TERR_HEIGHT, seg->end_pos.y), seg->end_hdg,
		    bp.veh.wheelbase, AMBER_TUPLE);
	} else {
		VERIFY3U(XPLMProbeTerrainXYZ(probe, cursor_world_pos.x, 0,
		    -cursor_world_pos.y, &info), ==, xplm_ProbeHitTerrain);
		draw_acf_symbol(VECT3(cursor_world_pos.x, info.locationY +
		    ABV_TERR_HEIGHT, cursor_world_pos.y), cursor_hdg,
		    bp.veh.wheelbase, RED_TUPLE);
	}

	if ((seg = list_tail(&bp.segs)) != NULL) {
		VERIFY3U(XPLMProbeTerrainXYZ(probe, seg->end_pos.x, 0,
		    -seg->end_pos.y, &info), ==, xplm_ProbeHitTerrain);
		draw_acf_symbol(VECT3(seg->end_pos.x, info.locationY +
		    ABV_TERR_HEIGHT, seg->end_pos.y), seg->end_hdg,
		    bp.veh.wheelbase, GREEN_TUPLE);
	}

#ifdef	DEBUG
	if (!IS_NULL_VECT(c_pt)) {
		ASSERT(!IS_NULL_VECT(s_pt));
		ASSERT(!isnan(line_hdg));
		double c_pt_y, s_pt_y;
		double l = vect2_dist(s_pt, c_pt);
		vect2_t dir_p = vect2_add(vect2_scmul(hdg2dir(line_hdg), l),
		    s_pt);

		VERIFY3U(XPLMProbeTerrainXYZ(probe, c_pt.x, 0,
		    -c_pt.y, &info), ==, xplm_ProbeHitTerrain);
		c_pt_y = info.locationY;
		VERIFY3U(XPLMProbeTerrainXYZ(probe, s_pt.x, 0,
		    -s_pt.y, &info), ==, xplm_ProbeHitTerrain);
		s_pt_y = info.locationY;

		draw_cross(c_pt, c_pt_y);
		draw_cross(s_pt, s_pt_y);

		glLineWidth(2);
		glColor3f(0.5, 0.5, 1);
		glBegin(GL_LINES);
		glVertex3f(s_pt.x, s_pt_y, -s_pt.y);
		glVertex3f(c_pt.x, c_pt_y, -c_pt.y);
		glVertex3f(s_pt.x, s_pt_y, -s_pt.y);
		glVertex3f(dir_p.x, s_pt_y, -dir_p.y);
		glEnd();
	}
#endif	/* DEBUG */

	XPLMDestroyProbe(probe);

	return (1);
}

static void
fake_win_draw(XPLMWindowID inWindowID, void *inRefcon)
{
	int w, h;
	UNUSED(inWindowID);
	UNUSED(inRefcon);

	XPLMGetScreenSize(&w, &h);
	XPLMSetWindowGeometry(fake_win, 0, h, w, 0);

	if (!XPLMIsWindowInFront(fake_win))
		XPLMBringWindowToFront(fake_win);
	if (force_root_win_focus)
		XPLMTakeKeyboardFocus(0);
}

static void
fake_win_key(XPLMWindowID inWindowID, char inKey, XPLMKeyFlags inFlags,
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
fake_win_cursor(XPLMWindowID inWindowID, int x, int y, void *inRefcon)
{
	UNUSED(inWindowID);
	UNUSED(x);
	UNUSED(y);
	UNUSED(inRefcon);
	return (xplm_CursorDefault);
}

static int
fake_win_click(XPLMWindowID inWindowID, int x, int y, XPLMMouseStatus inMouse,
    void *inRefcon)
{
	static uint64_t down_t = 0;
	static int down_x = 0, down_y = 0;

	UNUSED(inWindowID);
	UNUSED(inRefcon);

	if (inMouse == xplm_MouseDown) {
		down_t = microclock();
		down_x = x;
		down_y = y;
		force_root_win_focus = B_FALSE;
	} else if (inMouse == xplm_MouseDrag) {
		if (x != down_x || y != down_y) {
			int w, h;
			double fov_h, fov_v, rx, ry, rw, rh, dx, dy;
			vect2_t v;

			XPLMGetScreenSize(&w, &h);
			rx = ((double)x - down_x) / (w / 2);
			ry = ((double)y - down_y) / (h / 2);
			fov_h = DEG2RAD(dr_getf(&drs.camera_fov_h));
			fov_v = DEG2RAD(dr_getf(&drs.camera_fov_v));
			rw = cam_height * tan(fov_h / 2);
			rh = cam_height * tan(fov_v / 2);
			dx = rw * rx;
			dy = rh * ry;
			v = vect2_rot(VECT2(dx, dy), cam_hdg);
			cam_pos.x -= v.x;
			cam_pos.z -= v.y;
			down_x = x;
			down_y = y;
		}
	} else {
		if (microclock() - down_t < CLICK_THRESHOLD_US) {
			/*
			 * Transfer whatever is in pred_segs to the normal
			 * segments and clear pred_segs.
			 */
			for (seg_t *seg = list_head(&pred_segs); seg != NULL;
			    seg = list_head(&pred_segs)) {
				list_remove(&pred_segs, seg);
				list_insert_tail(&bp.segs, seg);
			}
		}
		force_root_win_focus = B_TRUE;
	}

	return (1);
}

static int
fake_win_wheel(XPLMWindowID inWindowID, int x, int y, int wheel, int clicks,
    void *inRefcon)
{
	UNUSED(inWindowID);
	UNUSED(x);
	UNUSED(y);
	UNUSED(wheel);
	UNUSED(clicks);
	UNUSED(inRefcon);

	if (wheel == 0 && clicks != 0) {
		static int accel = 1;
		static uint64_t last_wheel_t = 0;
		uint64_t now = microclock();
		uint64_t us_per_click = (now - last_wheel_t) / ABS(clicks);

		if (us_per_click < US_PER_CLICK_ACCEL)
			accel = MIN(accel + 1, MAX_ACCEL_MULT);
		else if (us_per_click > US_PER_CLICK_DEACCEL)
			accel = 1;

		cursor_hdg = normalize_hdg(cursor_hdg +
		    clicks * accel * WHEEL_ANGLE_MULT);
		last_wheel_t = now;
	}

	return (0);
}

static int
key_sniffer(char inChar, XPLMKeyFlags inFlags, char inVirtualKey, void *refcon)
{
	UNUSED(inChar);
	UNUSED(refcon);

	/* Only allow the plain key to be pressed, no modifiers */
	if (inFlags != xplm_DownFlag)
		return (1);

	switch (inVirtualKey) {
	case XPLM_VK_RETURN:
	case XPLM_VK_ESCAPE:
		XPLMCommandOnce(XPLMFindCommand("BetterPushback/stop_planner"));
		return (0);
	case XPLM_VK_CLEAR:
	case XPLM_VK_BACK:
	case XPLM_VK_DELETE:
		/* Delete the segments up to the next user-placed segment */
		if (list_tail(&bp.segs) == NULL)
			return (0);
		list_remove_tail(&bp.segs);
		for (seg_t *seg = list_tail(&bp.segs); seg != NULL &&
		    !seg->user_placed; seg = list_tail(&bp.segs)) {
			list_remove_tail(&bp.segs);
			free(seg);
		}
		return (0);
	}

	return (1);
}

bool_t
bp_cam_start(void)
{
	XPLMCreateWindow_t fake_win_ops = {
	    .structSize = sizeof (XPLMCreateWindow_t),
	    .left = 0, .top = 0, .right = 0, .bottom = 0, .visible = 1,
	    .drawWindowFunc = fake_win_draw,
	    .handleMouseClickFunc = fake_win_click,
	    .handleKeyFunc = fake_win_key,
	    .handleCursorFunc = fake_win_cursor,
	    .handleMouseWheelFunc = fake_win_wheel,
	    .refcon = NULL
	};

	if (cam_inited || !bp_init())
		return (B_FALSE);

	c_pt = NULL_VECT2;
	s_pt = NULL_VECT2;
	line_hdg = NAN;

#ifndef	PB_DEBUG_INTF
	if (vect3_abs(VECT3(dr_getf(&drs.vx), dr_getf(&drs.vy),
	    dr_getf(&drs.vz))) >= 1) {
		XPLMSpeakString("Can't start planner: aircraft not "
		    "stationary.");
		return (B_FALSE);
	}
	if (dr_getf(&drs.pbrake) == 0) {
		XPLMSpeakString("Can't start pushback planner: please "
		    "set the parking brake first.");
		return (B_FALSE);
	}
	if (started) {
		XPLMSpeakString("Can't start planner: pushback already in "
		    "progress. Please stop the pushback operation first.");
		return (B_FALSE);
	}
#endif	/* !PB_DEBUG_INTF */

	XPLMGetScreenSize(&fake_win_ops.right, &fake_win_ops.top);

	circle_view_cmd = XPLMFindCommand("sim/view/circle");
	ASSERT(circle_view_cmd != NULL);
	XPLMCommandOnce(circle_view_cmd);

	fake_win = XPLMCreateWindowEx(&fake_win_ops);
	ASSERT(fake_win != NULL);
	XPLMBringWindowToFront(fake_win);
	XPLMTakeKeyboardFocus(fake_win);

	list_create(&pred_segs, sizeof (seg_t), offsetof(seg_t, node));
	force_root_win_focus = B_TRUE;
	cam_height = 15 * bp.veh.wheelbase;
	/* We keep the camera position in our coordinates for ease of manip */
	cam_pos = VECT3(dr_getf(&drs.local_x),
	    dr_getf(&drs.local_y), -dr_getf(&drs.local_z));
	cam_hdg = dr_getf(&drs.hdg);
	cursor_hdg = dr_getf(&drs.hdg);
	XPLMControlCamera(xplm_ControlCameraForever, cam_ctl, NULL);

	XPLMRegisterDrawCallback(draw_prediction, PREDICTION_DRAWING_PHASE, 0,
	    NULL);

	for (int i = 0; view_cmds[i].name != NULL; i++) {
		view_cmds[i].cmd = XPLMFindCommand(view_cmds[i].name);
		VERIFY(view_cmds[i].cmd != NULL);
		XPLMRegisterCommandHandler(view_cmds[i].cmd, move_camera,
		    1, (void *)(uintptr_t)i);
	}
	XPLMRegisterKeySniffer(key_sniffer, 1, NULL);

	cam_inited = B_TRUE;

	return (B_TRUE);
}

bool_t
bp_cam_stop(void)
{
	if (!cam_inited)
		return (B_FALSE);

	for (seg_t *seg = list_head(&pred_segs); seg != NULL;
	    seg = list_head(&pred_segs)) {
		list_remove(&pred_segs, seg);
		free(seg);
	}
	list_destroy(&pred_segs);
	XPLMUnregisterDrawCallback(draw_prediction, PREDICTION_DRAWING_PHASE,
	    0, NULL);
	XPLMDestroyWindow(fake_win);

	for (int i = 0; view_cmds[i].name != NULL; i++) {
		XPLMUnregisterCommandHandler(view_cmds[i].cmd, move_camera,
		    1, (void *)(uintptr_t)i);
	}
	XPLMUnregisterKeySniffer(key_sniffer, 1, NULL);

	cam_inited = B_FALSE;

	return (B_TRUE);
}

unsigned
bp_num_segs(void)
{
	if (!bp_init())
		return (0);
	return (list_count(&bp.segs));
}
