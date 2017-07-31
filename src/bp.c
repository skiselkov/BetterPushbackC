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
#include <errno.h>

#if	IBM
#include <gl.h>
#elif	APL
#include <OpenGL/gl.h>
#else	/* LIN */
#include <GL/gl.h>
#endif	/* LIN */

#include <png.h>

#include <XPLMCamera.h>
#include <XPLMDisplay.h>
#include <XPLMGraphics.h>
#include <XPLMNavigation.h>
#include <XPLMScenery.h>
#include <XPLMUtilities.h>
#include <XPLMPlanes.h>
#include <XPLMProcessing.h>

#include <acfutils/assert.h>
#include <acfutils/dr.h>
#include <acfutils/geom.h>
#include <acfutils/intl.h>
#include <acfutils/math.h>
#include <acfutils/list.h>
#include <acfutils/time.h>
#include <acfutils/wav.h>

#include "bp.h"
#include "driving.h"
#include "msg.h"
#include "tug.h"
#include "xplane.h"

#define	PB_DEBUG_INTF

#define	MIN_XPLANE_VERSION	10500	/* X-Plane 10.50 */
#define	MIN_XPLANE_VERSION_STR	"10.50"	/* X-Plane 10.50 */

#define	STRAIGHT_STEER_RATE	20	/* degrees per second */
#define	TURN_STEER_RATE		10	/* degrees per second */
#define	MAX_FWD_SPEED		4	/* m/s [~8 knots] */
#define	MAX_REV_SPEED		1.11	/* m/s [4 km/h, "walking speed"] */
#define	NORMAL_ACCEL		0.25	/* m/s^2 */
#define	NORMAL_DECEL		0.17	/* m/s^2 */
#define	BRAKE_PEDAL_THRESH	0.1	/* brake pedal angle, 0..1 */
#define	FORCE_PER_TON		5000	/* max push force per ton, Newtons */
/*
 * X-Plane 10's tire model is a bit less forgiving of slow creeping,
 * so bump the minimum breakaway speed on that version.
 */
#define	BREAKAWAY_THRESH	(bp.xplane_version >= 11000 ? 0.1 : 0.35)
#define	SEG_TURN_MULT		0.9	/* leave 10% for oversteer */
#define	SPEED_COMPLETE_THRESH	0.05	/* m/s */
/* beyond this our push algos go nuts */
#define	MAX_STEER_ANGLE		(bp.xplane_version < 11000 ? 50 : 65)
#define	MIN_STEER_ANGLE		40	/* minimum sensible tire steer angle */
#define	MAX_ANG_VEL		2.5	/* degrees per second */
#define	PB_CRADLE_DELAY		10	/* seconds */
#define	PB_WINCH_DELAY		10	/* seconds */
#define	PB_CONN_DELAY		25.0	/* seconds */
#define	PB_CONN_LIFT_DELAY	13.0	/* seconds */
#define	PB_CONN_LIFT_DURATION	9.0	/* seconds */
#define	PB_START_DELAY		5	/* seconds */
#define	PB_DRIVING_TURN_OFFSET	15	/* meters */
#define	PB_LIFT_TE		0.075	/* fraction */
#define	STATE_TRANS_DELAY	2	/* seconds, state transition delay */
#define	CLEAR_SIGNAL_DELAY	15	/* seconds */
#define	TUG_DRIVE_AWAY_DIST	80	/* meters */
#define	MAX_DRIVING_AWAY_DELAY	30	/* seconds */

#define	MAX_ARPT_DIST		10000	/* meters */

#define	TUG_DRAWING_PHASE		xplm_Phase_Objects
#define	TUG_DRAWING_PHASE_BEFORE	1

#define	TUG_APPCH_LONG_DIST	(6 * bp.tug->veh.wheelbase)

#define	MIN_RADIO_VOLUME_THRESH	0.1

typedef enum {
	PB_STEP_OFF,
	PB_STEP_TUG_LOAD,
	PB_STEP_START,
	PB_STEP_DRIVING_UP_CLOSE,
	PB_STEP_OPENING_CRADLE,
	PB_STEP_WAITING_FOR_PBRAKE,
	PB_STEP_DRIVING_UP_CONNECT,
	PB_STEP_GRABBING,
	PB_STEP_LIFTING,
	PB_STEP_CONNECTED,
	PB_STEP_STARTING,
	PB_STEP_PUSHING,
	PB_STEP_STOPPING,
	PB_STEP_STOPPED,
	PB_STEP_LOWERING,
	PB_STEP_UNGRABBING,
	PB_STEP_MOVING_AWAY,
	PB_STEP_CLOSING_CRADLE,
	PB_STEP_STARTING2CLEAR,
	PB_STEP_MOVING2CLEAR,
	PB_STEP_CLEAR_SIGNAL,
	PB_STEP_DRIVING_AWAY
} pushback_step_t;

typedef struct {
	int		nw_i;
	double		nw_z;
	double		main_z;
	double		nw_len;
	unsigned	nw_type;
	double		tirrad;
} acf_t;

typedef struct {
	int			xplane_version;
	int			xplm_version;
	XPLMHostApplicationID	host_id;

	vehicle_t	veh;
	acf_t		acf;		/* our aircraft */

	struct {
		vect2_t	start_acf_pos;
		bool_t	pbrk_rele_called;
		bool_t	pbrk_set_called;
		bool_t	complete;
	} winching;

	vehicle_pos_t	cur_pos;
	vehicle_pos_t	last_pos;

	double		cur_t;		/* current time in seconds */
	double		last_t;		/* cur_t from previous run */
	double		last_mis_hdg;	/* previous steering misalignment */

	/* deltas from last_* to cur_* */
	vehicle_pos_t	d_pos;		/* delta from last_pos to cur_pos */
	double		d_t;		/* delta time from last_t to cur_t */

	double		last_steer;
	double		last_force;

	pushback_step_t	step;		/* current PB step */
	double		step_start_t;	/* PB step start time */
	double		last_voice_t;	/* last voice message start time */

	tug_t		*tug;
	vect2_t		start_pos;	/* where the pushback originated */
	double		start_hdg;	/* which way we were facing at start */

	list_t		segs;
} bp_state_t;

typedef struct {
	const char	*acf;
	const char	*author;
} acf_info_t;

static struct {
	dr_t	lbrake, rbrake;
	dr_t	pbrake, pbrake_rat;
	dr_t	rot_force_N;
	dr_t	axial_force;
	dr_t	local_x, local_y, local_z;
	dr_t	local_vx, local_vy, local_vz;
	dr_t	lat, lon;
	dr_t	hdg;
	dr_t	vx, vy, vz;
	dr_t	sim_time;
	dr_t	acf_mass;
	dr_t	mtow;
	dr_t	tire_z, leg_len, tirrad;
	dr_t	nw_steerdeg1, nw_steerdeg2;
	dr_t	tire_steer_cmd;
	dr_t	override_steer;
	dr_t	gear_types;
	dr_t	gear_steers;
	dr_t	gear_deploy;

	dr_t	camera_fov_h, camera_fov_v;
	dr_t	view_is_ext;
	dr_t	visibility;
	dr_t	cloud_types[3];
	dr_t	use_real_wx;

	dr_t	author;
} drs;

static bool_t inited = B_FALSE, cam_inited = B_FALSE;
static bool_t floop_registered = B_FALSE;
static bp_state_t bp;

static float bp_run(float elapsed, float elapsed2, int counter, void *refcon);
static void bp_complete(void);
static bool_t load_buttons(void);
static void unload_buttons(void);
static int key_sniffer(char inChar, XPLMKeyFlags inFlags, char inVirtualKey,
    void *refcon);

static bool_t radio_volume_warn = B_FALSE;

static const acf_info_t incompatible_acf[] = {
    { .acf = NULL, .author = NULL }
};

/*
 * This flag is set by the planner if the user clicked on the "connect first"
 * button. This commands us to start pushback without a plan, but stop just
 * short of actually starting to move the aircraft. This is used when the
 * pushback direction isn't known ahead of time and the tower assigns the
 * direction at the last moment. The user can attach the tug and wait for
 * pushback clearance, then do a quick plan and immediately commence pushing.
 */
bool_t late_plan_requested = B_FALSE;

static bool_t
pbrake_is_set(void)
{
	return (dr_getf(&drs.pbrake) != 0 || dr_getf(&drs.pbrake_rat) != 0);
}

static bool_t
acf_is_compatible(void)
{
	char my_acf[512], my_path[512];
	char my_author[512];

	XPLMGetNthAircraftModel(0, my_acf, my_path);
	dr_gets(&drs.author, my_author, sizeof (my_author));

	for (int i = 0; incompatible_acf[i].acf != NULL; i++) {
		if (strcmp(incompatible_acf[i].acf, my_acf) == 0 &&
		    (incompatible_acf[i].author == NULL ||
		    strcmp(incompatible_acf[i].author, my_author) == 0))
			return (B_FALSE);
	}

	return (B_TRUE);
}

/*
 * Locates the airport nearest to our current location, but which is also
 * within 10km (MAX_ARPT_DIST). If a suitable airport is found, its ICAO
 * code is placed in the return argument `icao' and the function returns
 * B_TRUE. Otherwise the variable is left untouched and B_FALSE is returned.
 */
static bool_t
find_nearest_airport(char icao[8])
{
	geo_pos2_t my_pos = GEO_POS2(dr_getf(&drs.lat), dr_getf(&drs.lon));
	vect3_t my_pos_ecef = sph2ecef(GEO_POS3(my_pos.lat, my_pos.lon, 0));
	list_t *list;
	airport_t *arpt;
	double min_dist = 1e10;

	*icao = 0;

	load_nearest_airport_tiles(airportdb, my_pos);
	list = find_nearest_airports(airportdb, my_pos);

	for (arpt = list_head(list); arpt != NULL;
	    arpt = list_next(list, arpt)) {
		double dist = vect3_dist(arpt->ecef, my_pos_ecef);
		logMsg("candidate: %s dist: %.0f", arpt->icao, dist);
		if (dist < min_dist) {
			strlcpy(icao, arpt->icao, sizeof (arpt->icao));
			min_dist = dist;
			logMsg("new closest: %s dist: %.0f", arpt->icao, dist);
		}
	}
	free_nearest_airport_list(list);
	unload_distant_airport_tiles(airportdb, NULL_GEO_POS2);

	logMsg("nearest airport: \"%s\"", icao);

	return (*icao != 0);
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

static double
turn_nosewheel(double req_angle, double rate)
{
	double rate_of_turn, steer_incr, cur_nw_angle, rate_of_turn_fract;

	/* limit the steering request to what we can actually do */
	req_angle = MIN(req_angle, bp.veh.max_steer);
	req_angle = MAX(req_angle, -bp.veh.max_steer);

	dr_getvf(&drs.tire_steer_cmd, &cur_nw_angle, bp.acf.nw_i, 1);
	if (cur_nw_angle == req_angle)
		return (0);

	/*
	 * Modulate the steering increment to always be
	 * correctly within our rate of turn limits.
	 */
	rate_of_turn = rate * bp.d_t;
	steer_incr = MIN(ABS(req_angle - cur_nw_angle), rate_of_turn);
	rate_of_turn_fract = (cur_nw_angle < req_angle ?
	    steer_incr : -steer_incr) / rate_of_turn;
	cur_nw_angle += (cur_nw_angle < req_angle ? steer_incr : -steer_incr);
	dr_setvf(&drs.tire_steer_cmd, &cur_nw_angle, bp.acf.nw_i, 1);

	return (rate_of_turn_fract);
}

static void
push_at_speed(double targ_speed, double max_accel, bool_t allow_snd_ctl)
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
	dr_getvf(&drs.tire_steer_cmd, &angle_rad, bp.acf.nw_i, 1);
	angle_rad = DEG2RAD(angle_rad);
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

	if (allow_snd_ctl) {
		tug_set_TE_override(bp.tug, B_TRUE);
		if ((bp.cur_pos.spd > 0 && force < 0) ||
		    (bp.cur_pos.spd < 0 && force > 0)) {
			double spd_fract = (ABS(bp.cur_pos.spd) /
			    bp.tug->info->max_fwd_speed);
			double force_fract = fabs(force /
			    bp.tug->info->max_TE);
			tug_set_TE_snd(bp.tug, AVG(force_fract, spd_fract),
			    bp.d_t);
		} else {
			tug_set_TE_snd(bp.tug, 0, bp.d_t);
		}
	}
}

static bool_t
read_gear_info(void)
{
	double tire_z[10];
	int gear_steers[10], gear_types[10], gear_is[10];
	int n_gear = 0;

	/* First determine where the gears are */
	for (int i = 0, n = dr_getvi(&drs.gear_types, gear_types, 0, 10);
	    i < n; i++) {
		/*
		 * Gear types are:
		 * 0) Nothing.
		 * 1) Skid.
		 * 2+) Wheel based gear in various arrangements. A tug can
		 *	provide a filter for this.
		 */
		if (gear_types[i] >= 2)
			gear_is[n_gear++] = i;
	}

	/* Read nosegear long axis deflections */
	VERIFY3S(dr_getvf(&drs.tire_z, tire_z, 0, 10), >=, n_gear);
	bp.acf.nw_i = -1;
	bp.acf.nw_z = 1e10;

	/* Next determine which gear steers. Pick the one most forward. */
	VERIFY3S(dr_getvi(&drs.gear_steers, gear_steers, 0, 10), >=, n_gear);
	for (int i = 0; i < n_gear; i++) {
		if (gear_steers[gear_is[i]] == 1 &&
		    tire_z[gear_is[i]] < bp.acf.nw_z) {
			bp.acf.nw_i = gear_is[i];
			bp.acf.nw_z = tire_z[gear_is[i]];
		}
	}
	if (bp.acf.nw_i == -1) {
		XPLMSpeakString(_("Pushback failure: aircraft appears to not "
		    "have any steerable gears."));
		return (B_FALSE);
	}

	/* Nose gear strut length and tire radius */
	VERIFY3S(dr_getvf(&drs.leg_len, &bp.acf.nw_len, bp.acf.nw_i, 1), ==, 1);
	VERIFY3S(dr_getvf(&drs.tirrad, &bp.acf.tirrad, bp.acf.nw_i, 1), ==, 1);

	/* Read nosewheel type */
	bp.acf.nw_type = gear_types[bp.acf.nw_i];

	/* Compute main gear Z deflection as mean of all main gears */
	for (int i = 0; i < n_gear; i++) {
		if (gear_is[i] != bp.acf.nw_i)
			bp.acf.main_z += tire_z[gear_is[i]];
	}
	bp.acf.main_z /= n_gear - 1;

	return (B_TRUE);
}

static bool_t
bp_state_init(void)
{
	memset(&bp, 0, sizeof (bp));
	list_create(&bp.segs, sizeof (seg_t), offsetof(seg_t, node));

	XPLMGetVersions(&bp.xplane_version, &bp.xplm_version,
	    &bp.host_id);
	if (bp.xplane_version < MIN_XPLANE_VERSION) {
		char msg[256];
		snprintf(msg, sizeof (msg), _("Pushback failure: X-Plane "
		    "version too old. This plugin requires at least X-Plane "
		    "%s to operate."), MIN_XPLANE_VERSION_STR);
		XPLMSpeakString(msg);
		return (B_FALSE);
	}

	if (!read_gear_info())
		return (B_FALSE);

	bp.veh.wheelbase = bp.acf.main_z - bp.acf.nw_z;
	bp.veh.fixed_z_off = -bp.acf.main_z;	/* X-Plane's Z is negative */
	if (bp.veh.wheelbase <= 0) {
		XPLMSpeakString(_("Pushback failure: aircraft has non-positive "
		    "wheelbase. Sorry, tail draggers aren't supported."));
		return (B_FALSE);
	}
	bp.veh.max_steer = MIN(MAX(dr_getf(&drs.nw_steerdeg1),
	    dr_getf(&drs.nw_steerdeg2)), MAX_STEER_ANGLE);
	/*
	 * Some aircraft have a broken declaration here and only declare the
	 * high-speed rudder steering angle. For those, ignore what they say
	 * and use our MAX_STEER_ANGLE.
	 */
	if (bp.veh.max_steer < MIN_STEER_ANGLE)
		bp.veh.max_steer = MAX_STEER_ANGLE;
	bp.veh.max_fwd_spd = MAX_FWD_SPEED;
	bp.veh.max_rev_spd = MAX_REV_SPEED;
	bp.veh.max_ang_vel = MAX_ANG_VEL;
	bp.veh.max_accel = NORMAL_ACCEL;
	bp.veh.max_decel = NORMAL_DECEL;

	bp.step = PB_STEP_OFF;
	bp.step_start_t = 0;

	return (B_TRUE);
}

bool_t
bp_init(void)
{
	dr_t radio_vol, sound_on;

	/*
	 * Due to numerous spurious bug reports of missing ground crew audio,
	 * check that the user hasn't turned down the radio volume and just
	 * forgotten about it. Warn the user if the volume is very low.
	 */
	fdr_find(&sound_on, "sim/operation/sound/sound_on");
	fdr_find(&radio_vol, "sim/operation/sound/radio_volume_ratio");
	if (dr_getf(&radio_vol) < MIN_RADIO_VOLUME_THRESH &&
	    dr_geti(&sound_on) == 1 && !radio_volume_warn) {
		XPLMSpeakString(_("Pushback advisory: you have your radio "
		    "volume turned very low and may not be able to hear "
		    "ground crew. Please increase your radio volume in "
		    "the X-Plane sound preferences."));
		radio_volume_warn = B_TRUE;
	}

	if (inited)
		return (B_TRUE);

	memset(&drs, 0, sizeof (drs));

	fdr_find(&drs.lbrake, "sim/cockpit2/controls/left_brake_ratio");
	fdr_find(&drs.rbrake, "sim/cockpit2/controls/right_brake_ratio");
	if (/* FlightFactor A320 */
	    !dr_find(&drs.pbrake, "model/controls/park_break") &&
	    /* Felis Tu-154M */
	    !dr_find(&drs.pbrake, "sim/custom/controll/parking_brake")) {
		fdr_find(&drs.pbrake, "sim/flightmodel/controls/parkbrake");
	}
	fdr_find(&drs.pbrake_rat, "sim/cockpit2/controls/parking_brake_ratio");
	fdr_find(&drs.rot_force_N, "sim/flightmodel/forces/N_plug_acf");
	fdr_find(&drs.axial_force, "sim/flightmodel/forces/faxil_plug_acf");
	fdr_find(&drs.local_x, "sim/flightmodel/position/local_x");
	fdr_find(&drs.local_y, "sim/flightmodel/position/local_y");
	fdr_find(&drs.local_z, "sim/flightmodel/position/local_z");
	fdr_find(&drs.local_vx, "sim/flightmodel/position/local_vx");
	fdr_find(&drs.local_vy, "sim/flightmodel/position/local_vy");
	fdr_find(&drs.local_vz, "sim/flightmodel/position/local_vz");
	fdr_find(&drs.lat, "sim/flightmodel/position/latitude");
	fdr_find(&drs.lon, "sim/flightmodel/position/longitude");
	fdr_find(&drs.hdg, "sim/flightmodel/position/psi");
	fdr_find(&drs.vx, "sim/flightmodel/position/local_vx");
	fdr_find(&drs.vy, "sim/flightmodel/position/local_vy");
	fdr_find(&drs.vz, "sim/flightmodel/position/local_vz");
	fdr_find(&drs.sim_time, "sim/time/total_running_time_sec");
	fdr_find(&drs.acf_mass, "sim/flightmodel/weight/m_total");
	fdr_find(&drs.tire_z, "sim/flightmodel/parts/tire_z_no_deflection");
	fdr_find(&drs.mtow, "sim/aircraft/weight/acf_m_max");
	fdr_find(&drs.leg_len, "sim/aircraft/parts/acf_gear_leglen");
	fdr_find(&drs.tirrad, "sim/aircraft/parts/acf_gear_tirrad");
	fdr_find(&drs.nw_steerdeg1, "sim/aircraft/gear/acf_nw_steerdeg1");
	fdr_find(&drs.nw_steerdeg2, "sim/aircraft/gear/acf_nw_steerdeg2");
	fdr_find(&drs.tire_steer_cmd,
	    "sim/flightmodel/parts/tire_steer_cmd");
	fdr_find(&drs.override_steer,
	    "sim/operation/override/override_wheel_steer");
	fdr_find(&drs.gear_types, "sim/aircraft/parts/acf_gear_type");
	fdr_find(&drs.gear_steers, "sim/aircraft/overflow/acf_gear_steers");
	fdr_find(&drs.gear_deploy, "sim/aircraft/parts/acf_gear_deploy");

	fdr_find(&drs.camera_fov_h,
	    "sim/graphics/view/field_of_view_deg");
	fdr_find(&drs.camera_fov_v,
	    "sim/graphics/view/vertical_field_of_view_deg");
	fdr_find(&drs.view_is_ext, "sim/graphics/view/view_is_external");
	fdr_find(&drs.visibility, "sim/weather/visibility_reported_m");
	fdr_find(&drs.cloud_types[0], "sim/weather/cloud_type[0]");
	fdr_find(&drs.cloud_types[1], "sim/weather/cloud_type[1]");
	fdr_find(&drs.cloud_types[2], "sim/weather/cloud_type[2]");
	fdr_find(&drs.use_real_wx, "sim/weather/use_real_weather_bool");

	fdr_find(&drs.author, "sim/aircraft/view/acf_author");

	if (!bp_state_init())
		return (B_FALSE);

	if (!load_buttons())
		return (B_FALSE);

	inited = B_TRUE;

	return (B_TRUE);
}

static int
draw_tugs(XPLMDrawingPhase phase, int before, void *refcon)
{
	UNUSED(phase);
	UNUSED(before);
	UNUSED(refcon);

	if (bp.tug == NULL) {
		/*
		 * If we have no tug loaded, we must either be in the
		 * tug-selection phase, or be slaved to a master instance
		 * which has not yet notified us which tug to use.
		 */
		ASSERT(bp.step <= PB_STEP_TUG_LOAD || slave_mode);
		return (1);
	}

	if (list_head(&bp.tug->segs) == NULL &&
	    bp.step >= PB_STEP_GRABBING && bp.step <= PB_STEP_UNGRABBING) {
		double hdg, tug_hdg, tug_spd, steer;
		vect2_t pos, dir, off_v, tug_pos;

		pos = VECT2(dr_getf(&drs.local_x), -dr_getf(&drs.local_z));
		hdg = dr_getf(&drs.hdg);
		dr_getvf(&drs.tire_steer_cmd, &steer, bp.acf.nw_i, 1);
		tug_hdg = normalize_hdg(hdg + steer);
		dir = hdg2dir(hdg);
		if (bp.step == PB_STEP_GRABBING &&
		    bp.tug->info->lift_type == LIFT_WINCH) {
			/*
			 * When winching the aircraft forward, we keep the tug
			 * in a fixed position relative to where the aircraft
			 * was when the winching operation started.
			 */
			tug_set_pos(bp.tug, vect2_add(bp.winching.start_acf_pos,
			    vect2_scmul(hdg2dir(hdg), (-bp.acf.nw_z) +
			    (-bp.tug->info->plat_z))), hdg, 0);
		} else {
			off_v = vect2_scmul(hdg2dir(tug_hdg),
			    (-bp.tug->info->lift_wall_z) +
			    tug_lift_wall_off(bp.tug));
			tug_pos = vect2_add(vect2_add(pos, vect2_scmul(dir,
			    -bp.acf.nw_z)), off_v);
			tug_spd = bp.cur_pos.spd / cos(DEG2RAD(fabs(steer)));
			tug_set_pos(bp.tug, tug_pos, tug_hdg, tug_spd);
		}
	}

	tug_draw(bp.tug, bp.cur_t);

	return (1);
}

bool_t
bp_can_start(char **reason)
{
	seg_t *seg;

	if (!acf_is_compatible()) {
		if (reason != NULL)
			*reason = "Pushback failure: aircraft is not "
			    "compatible with BetterPushback.";
		return (B_FALSE);
	}

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
	if (seg == NULL && !late_plan_requested && !slave_mode) {
		if (reason != NULL) {
			*reason = "Pushback failure: please first plan your "
			    "pushback to tell me where you want to go.";
		}
		return (B_FALSE);
	}

	return (B_TRUE);
}

static void
bp_delete_all_segs(void)
{
	seg_t *seg;
	while ((seg = list_remove_head(&bp.segs)) != NULL)
		free(seg);
}

bool_t
bp_start(void)
{
	char *reason;

	if (bp_started)
		return (B_TRUE);
	if (!bp_can_start(&reason)) {
		XPLMSpeakString(reason);
		return (B_FALSE);
	}

	bp_gather();
	bp.last_pos = bp.cur_pos;
	bp.last_t = bp.cur_t;

	bp.step = 1;
	bp.step_start_t = bp.cur_t;

	/*
	 * Memorize where we were at the start. We will use this to determine
	 * which way to turn when disconnecting and where to attempt to go
	 * once we're done.
	 */
	bp.start_pos = bp.cur_pos.pos;
	bp.start_hdg = bp.cur_pos.hdg;

	XPLMRegisterFlightLoopCallback(bp_run, -1, NULL);
	floop_registered = B_TRUE;
	XPLMRegisterDrawCallback(draw_tugs, TUG_DRAWING_PHASE,
	    TUG_DRAWING_PHASE_BEFORE, NULL);

	if (!slave_mode && !late_plan_requested)
		route_save(&bp.segs);

	bp_started = B_TRUE;

	return (B_TRUE);
}

bool_t
bp_stop(void)
{
	if (!bp_started)
		return (B_FALSE);

	bp_delete_all_segs();
	late_plan_requested = B_FALSE;

	return (B_TRUE);
}

void
bp_fini(void)
{
	if (!inited)
		return;

	bp_complete();
	if (floop_registered) {
		XPLMUnregisterFlightLoopCallback(bp_run, NULL);
		floop_registered = B_FALSE;
	}

	bp_delete_all_segs();
	list_destroy(&bp.segs);

	unload_buttons();

	radio_volume_warn = B_FALSE;

	inited = B_FALSE;
}

static void
acf2tug_steer(void)
{
	double cur_steer, d_hdg, tug_spd, tug_steer_cmd;
	vect2_t v, c, s, isects[2];

	dr_getvf(&drs.tire_steer_cmd, &cur_steer, bp.acf.nw_i, 1);
	tug_spd = bp.cur_pos.spd / cos(DEG2RAD(fabs(cur_steer)));

	/*
	 * When we're nearly stopped, the steering algorithm can command
	 * weird deflections. Neutralize steering when we're nearly stopped.
	 */
	if (fabs(tug_spd) <= 0.01) {
		tug_set_steering(bp.tug, 0, bp.d_t);
		return;
	}
	/*
	 * d_hdg is the total change in tug heading, which is a component of
	 * the amount of heading change the aircraft is experiencing + the
	 * amount of nosewheel deflection change being applied (since the
	 * nosewheel *is* our tug).
	 */
	d_hdg = (bp.d_pos.hdg + (cur_steer - bp.last_steer)) / bp.d_t;
	if (tug_spd < 0)
		d_hdg = -d_hdg;	/* reverse steering when going backwards */

	/*
	 * We model tug steering as follows:
	 *
	 *        ....
	 *            --.
	 *              _-x (s)
	 *        (c)_-  /.
	 * (front) +    / .
	 *         |   / .
	 *  wheel  |  /.
	 *   base  | /(v)
	 *         |/
	 *  (rear) + (0,0)
	 *
	 * Set the rear axle as our coordinate origin (0,0). Paint an infinite
	 * vector (v) from the rear axle at an angle of d_hdg. Place a circle
	 * around the front axle at (c) with radius tug_spd. Where the circle
	 * intersects vector (v) is our steering target (s). If the circle
	 * intersects at two points, pick the one furthest from (0,0). The
	 * resulting steering command is described as an angle vector from
	 * (c) to (s). Convert that vector into a degree value and feed that
	 * to the tug.
	 */
	v = vect2_set_abs(hdg2dir(d_hdg), 1e10);
	c = VECT2(0, bp.tug->veh.wheelbase);

	switch (vect2circ_isect(v, ZERO_VECT2, c, tug_spd, B_TRUE, isects)) {
	case 0:
		tug_set_steering(bp.tug,
		    bp.tug->info->max_steer * (d_hdg > 0 ? 1 : -1), bp.d_t);
		return;
	case 1:
		s = isects[0];
		break;
	case 2:
		s = vect2_abs(isects[0]) > vect2_abs(isects[1]) ? isects[0] :
		    isects[1];
		break;
	}

	tug_steer_cmd = dir2hdg(vect2_sub(s, c));

	if (tug_steer_cmd >= 180)
		tug_steer_cmd -= 360;

	tug_set_steering(bp.tug, tug_steer_cmd, bp.d_t);
}

static bool_t
bp_run_push(void)
{
	seg_t *seg;

	while ((seg = list_head(&bp.segs)) != NULL) {
		double steer, speed;

		/* Pilot pressed brake pedals or set parking brake, stop */
		if (dr_getf(&drs.lbrake) > BRAKE_PEDAL_THRESH ||
		    dr_getf(&drs.rbrake) > BRAKE_PEDAL_THRESH ||
		    pbrake_is_set()) {
			tug_set_TE_snd(bp.tug, 0, bp.d_t);
			break;
		}
		if (drive_segs(&bp.cur_pos, &bp.veh, &bp.segs,
		    &bp.last_mis_hdg, bp.d_t, &steer, &speed)) {
			double steer_rate = (seg->type == SEG_TYPE_STRAIGHT ?
			    STRAIGHT_STEER_RATE : TURN_STEER_RATE);
			(void) turn_nosewheel(steer, steer_rate);
			push_at_speed(speed, bp.veh.max_accel, B_TRUE);
			acf2tug_steer();
			break;
		}
	}

	return (seg != NULL);
}

/*
 * Tears down a pushback session. This resets all state variables, unloads the
 * tug model and prepares us for another start.
 */
static void
bp_complete(void)
{
	if (!bp_started)
		return;

	bp_started = B_FALSE;
	late_plan_requested = B_FALSE;
	plan_complete = B_FALSE;

	if (bp.tug != NULL) {
		tug_free(bp.tug);
		bp.tug = NULL;
	}

	XPLMUnregisterDrawCallback(draw_tugs, TUG_DRAWING_PHASE,
	    TUG_DRAWING_PHASE_BEFORE, NULL);

	if (!slave_mode) {
		dr_seti(&drs.override_steer, 0);
		dr_setf(&drs.lbrake, 0);
		dr_setf(&drs.rbrake, 0);
		dr_setvf(&drs.leg_len, &bp.acf.nw_len, bp.acf.nw_i, 1);
	}

	bp_done_notify();
	/*
	 * Reinitialize our state so we're starting with a clean slate
	 * next time.
	 */
	bp_state_init();
}

/*
 * Returns B_TRUE when the late plan phase can be exited. This occurs when:
 * 1) if the machine is a master, the user must have completed the plan
 *	AND exited the pushback camera.
 * 2) if the machine is a slave, the plan_completed flag is synced from the
 *	master machine.
 */
static bool_t
late_plan_end_cond(void)
{
	return ((!slave_mode && list_head(&bp.segs) != NULL && !cam_inited) ||
	    (slave_mode && plan_complete));
}

static bool_t
pb_step_tug_load(void)
{
	if (!slave_mode) {
		char icao[8] = { 0 };

		(void) find_nearest_airport(icao);
		bp.tug = tug_alloc_auto(dr_getf(&drs.mtow),
		    dr_getf(&drs.leg_len), bp.acf.tirrad,
		    bp.acf.nw_type, strcmp(icao, "") != 0 ? icao : NULL);
		if (bp.tug == NULL) {
			XPLMSpeakString(_("Pushback failure: no suitable "
			    "tug for your aircraft."));
			bp_complete();
			return (B_FALSE);
		}
		strlcpy(bp_tug_name, bp.tug->info->tug_name,
		    sizeof (bp_tug_name));
	} else {
		char tug_name[sizeof (bp_tug_name)];
		char *ext;
		char icao[8] = { 0 };

		/* make sure the tug name is properly terminated */
		memcpy(tug_name, bp_tug_name, sizeof (tug_name));
		tug_name[sizeof (tug_name) - 1] = '\0';

		/* wait until the tug name has been synced */
		if (strcmp(tug_name, "") == 0)
			return (B_TRUE);

		/* security check - must not contain a dir separator */
		if (strchr(tug_name, '/') != NULL ||
		    strchr(tug_name, '\\') != NULL)
			return (B_TRUE);

		/* sanity check - must end in '.tug' */
		ext = strrchr(tug_name, '.');
		if (ext == NULL || strcmp(&ext[1], "tug") != 0)
			return (B_TRUE);

		(void) find_nearest_airport(icao);

		bp.tug = tug_alloc_man(tug_name, bp.acf.tirrad, icao);
		if (bp.tug == NULL) {
			char msg[256];
			snprintf(msg, sizeof (msg), _("Pushback failure: "
			    "master requested tug \"%s\", which we don't have "
			    "in our in our library. Please sync your tug "
			    "libraries before trying again."), tug_name);
			XPLMSpeakString(msg);
			bp_complete();
			return (B_FALSE);
		}
	}
	if (!bp.tug->info->drive_debug) {
		vect2_t p_start, dir;
		dir = hdg2dir(bp.cur_pos.hdg);
		p_start = vect2_add(bp.cur_pos.pos, vect2_scmul(dir,
		    -bp.acf.nw_z + TUG_APPCH_LONG_DIST));
		p_start = vect2_add(p_start, vect2_scmul(vect2_norm(dir,
		    B_TRUE), 10 * bp.tug->veh.wheelbase));
		tug_set_pos(bp.tug, p_start, normalize_hdg(bp.cur_pos.hdg - 90),
		    bp.tug->veh.max_fwd_spd);
	} else {
		tug_set_pos(bp.tug, bp.cur_pos.pos, bp.cur_pos.hdg, 0);
	}
	bp.step++;
	bp.step_start_t = bp.cur_t;

	return (B_TRUE);
}

static void
pb_step_start(void)
{
	if (!bp.tug->info->drive_debug) {
		vect2_t left_off, p_end, dir;

		dir = hdg2dir(bp.cur_pos.hdg);

		left_off = vect2_add(bp.cur_pos.pos, vect2_scmul(dir,
		    -bp.acf.nw_z + TUG_APPCH_LONG_DIST));
		left_off = vect2_add(left_off, vect2_scmul(
		    vect2_norm(dir, B_FALSE), 2 * bp.tug->veh.wheelbase));
		p_end = vect2_add(bp.cur_pos.pos, vect2_scmul(dir,
		    (-bp.acf.nw_z) + bp.tug->info->apch_dist));

		VERIFY(tug_drive2point(bp.tug, left_off,
		    normalize_hdg(bp.cur_pos.hdg - 90)));
		VERIFY(tug_drive2point(bp.tug, p_end, bp.cur_pos.hdg));
	} else {
		for (seg_t *seg = list_head(&bp.segs); seg != NULL;
		    seg = list_next(&bp.segs, seg)) {
			seg_t *seg2 = calloc(1, sizeof (*seg2));
			memcpy(seg2, seg, sizeof (*seg2));
			list_insert_tail(&bp.tug->segs, seg2);
		}
	}

	msg_play(MSG_DRIVING_UP);
	bp.step++;
	bp.step_start_t = bp.cur_t;
	bp.last_voice_t = bp.cur_t;
}

static void
pb_step_driving_up_close(void)
{
	if (!tug_is_stopped(bp.tug)) {
		/*
		 * Keep resetting the start time to enforce the state
		 * transition delay once the tug stops.
		 */
		bp.step_start_t = bp.cur_t;
	} else  if (bp.cur_t - bp.step_start_t >= STATE_TRANS_DELAY) {
		tug_set_cradle_beeper_on(bp.tug, B_TRUE);
		tug_set_cradle_lights_on(B_TRUE);
		tug_set_hazard_lights_on(B_TRUE);
		bp.step++;
		bp.step_start_t = bp.cur_t;
	}
}

static void
pb_step_waiting_for_pbrake(void)
{
	vect2_t p_end, dir;

	if (!pbrake_is_set() ||
	    /* wait until the rdy2conn message has stopped playing */
	    bp.cur_t - bp.last_voice_t < msg_dur(MSG_RDY2CONN)) {
		/* keep resetting the start time to enforce a delay */
		bp.step_start_t = bp.cur_t;
		return;
	}
	/*
	 * After the parking brake is set and the message has finished
	 * playing, wait a short moment until starting to move again.
	 */
	if (bp.cur_t - bp.step_start_t < STATE_TRANS_DELAY)
		return;

	dir = hdg2dir(bp.cur_pos.hdg);
	if (bp.tug->info->lift_type == LIFT_GRAB) {
		p_end = vect2_add(bp.cur_pos.pos, vect2_scmul(dir,
		    (-bp.acf.nw_z) + (-bp.tug->info->lift_wall_z) +
		    tug_lift_wall_off(bp.tug)));
	} else {
		vect2_t d = vect2_neg(hdg2dir(bp.tug->pos.hdg));
		p_end = vect2_add(bp.tug->pos.pos, vect2_scmul(d,
		    bp.tug->info->apch_dist + bp.tug->info->plat_z));
	}
	(void) tug_drive2point(bp.tug, p_end, bp.cur_pos.hdg);
	bp.step++;
	bp.step_start_t = bp.cur_t;
}

static void
pb_step_driving_up_connect(void)
{
	if (!slave_mode) {
		dr_setf(&drs.lbrake, 0.9);
		dr_setf(&drs.rbrake, 0.9);
	}
	if (!tug_is_stopped(bp.tug)) {
		/*
		 * Keep resetting the start time to enforce a state
		 * transition delay once the tug stops.
		 */
		bp.step_start_t = bp.cur_t;
	} else if (bp.cur_t - bp.step_start_t >= STATE_TRANS_DELAY) {
		tug_set_cradle_beeper_on(bp.tug, B_TRUE);
		bp.winching.start_acf_pos = bp.cur_pos.pos;
		bp.step++;
		bp.step_start_t = bp.cur_t;
	}
}

static void
pb_step_connect_grab(void)
{
	double d_t = bp.cur_t - bp.step_start_t;
	double cradle_closed_fract = d_t / PB_CONN_LIFT_DELAY;

	cradle_closed_fract = MAX(MIN(cradle_closed_fract, 1), 0);
	tug_set_lift_arm_pos(bp.tug, 1 - cradle_closed_fract, B_TRUE);

	if (!slave_mode) {
		/* When grabbing, keep the aircraft firmly in place */
		dr_setf(&drs.lbrake, 0.9);
		dr_setf(&drs.rbrake, 0.9);
	}

	if (cradle_closed_fract >= 1) {
		bp.step++;
		bp.step_start_t = bp.cur_t;
	}
}

static void
pb_step_connect_winch(void)
{
	double d_t = bp.cur_t - bp.step_start_t;
	const tug_info_t *ti = bp.tug->info;
	double winch_total, winched_dist;

	/* spend some time putting the winching strap in place */
	if (!bp.winching.complete && d_t < STATE_TRANS_DELAY)
		return;

	tug_set_lift_pos(0);
	tug_set_winch_on(bp.tug, B_TRUE);

	/* after installing the strap, wait some more to make the pbrake call */
	if (!bp.winching.complete && d_t < 2 * STATE_TRANS_DELAY) {
		tug_set_lift_arm_pos(bp.tug, 1.0, B_TRUE);
		return;
	}

	if (!bp.winching.complete && pbrake_is_set()) {
		if (!bp.winching.pbrk_rele_called) {
			msg_play(MSG_WINCH);
			bp.last_voice_t = bp.cur_t;
			bp.winching.pbrk_rele_called = B_TRUE;
		}
		return;
	}

	if (!slave_mode) {
		/* When grabbing, keep the aircraft firmly in place */
		dr_setf(&drs.lbrake, 0);
		dr_setf(&drs.rbrake, 0);
	}

	winch_total = ti->lift_wall_z - ti->plat_z - tug_lift_wall_off(bp.tug);
	winched_dist = vect2_dist(bp.winching.start_acf_pos, bp.cur_pos.pos);
	if (winched_dist < winch_total && !bp.winching.complete) {
		/*
		 * While 'winch_total' tells us how far we need to winch,
		 * the animation values are as a proportion of the maximum
		 * possible winching distance (i.e. at the smallest tirrad).
		 */
		double x = winched_dist / (ti->lift_wall_z - ti->plat_z);
		if (!slave_mode) {
			double lift = ti->plat_h * x + bp.acf.nw_len;
			push_at_speed(0.05, 0.05, B_FALSE);
			dr_setvf(&drs.leg_len, &lift, bp.acf.nw_i, 1);
		}
		tug_set_lift_arm_pos(bp.tug, 1 - x, B_TRUE);
		tug_set_TE_override(bp.tug, B_TRUE);
		tug_set_TE_snd(bp.tug, PB_LIFT_TE, bp.d_t);
	} else {
		bp.winching.complete = B_TRUE;
	}

	if (bp.winching.complete) {
		bp.step++;
		bp.step_start_t = bp.cur_t;
	}
}

static void
pb_step_grab(void)
{
	if (!slave_mode) {
		double steer = 0;
		dr_setvf(&drs.tire_steer_cmd, &steer, bp.acf.nw_i, 1);
	}
	tug_set_lift_in_transit(B_TRUE);
	if (bp.tug->info->lift_type == LIFT_GRAB)
		pb_step_connect_grab();
	else
		pb_step_connect_winch();
}

static void
pb_step_lift(void)
{
	double d_t = bp.cur_t - bp.step_start_t;
	double lift;
	double lift_fract = d_t / PB_CONN_LIFT_DURATION;

	lift_fract = MAX(MIN(lift_fract, 1), 0);
	tug_set_lift_pos(lift_fract);

	/* Iterate the lift */
	lift = (bp.tug->info->lift_height * lift_fract) + bp.acf.nw_len +
	    tug_plat_h(bp.tug);
	if (!slave_mode) {
		dr_setf(&drs.lbrake, 0.9);
		dr_setf(&drs.rbrake, 0.9);
		dr_setvf(&drs.leg_len, &lift, bp.acf.nw_i, 1);
	}

	/*
	 * While lifting, we simulate a ramp-up and ramp-down of the
	 * tug's Tractive Effort to simulate that the engine is
	 * being used to pressurize a hydraulic lift system.
	 */
	if (d_t < PB_CONN_LIFT_DURATION) {
		tug_set_TE_override(bp.tug, B_TRUE);
		tug_set_TE_snd(bp.tug, PB_LIFT_TE, bp.d_t);
	}
	if (d_t >= PB_CONN_LIFT_DURATION) {
		tug_set_TE_override(bp.tug, B_TRUE);
		tug_set_TE_snd(bp.tug, 0, bp.d_t);
		tug_set_cradle_beeper_on(bp.tug, B_FALSE);
		tug_set_lift_in_transit(B_FALSE);
		tug_set_TE_override(bp.tug, B_FALSE);
	}

	if (d_t >= PB_CONN_LIFT_DURATION + STATE_TRANS_DELAY) {
		if (late_plan_requested) {
			/*
			 * The user requsted a late plan, so this is as
			 * far as we can go without segments. Also wait
			 * for the camera to stop.
			 */
			if (!late_plan_end_cond())
				return;
			late_plan_requested = B_FALSE;
			/*
			 * We normally save the route during bp_start,
			 * but since the user requested late planning,
			 * we need to save it now.
			 */
			if (!slave_mode) {
				plan_complete = B_TRUE;
				route_save(&bp.segs);
			}
		}

		if (bp.tug->info->lift_type != LIFT_WINCH) {
			msg_play(MSG_CONNECTED);
			bp.last_voice_t = bp.cur_t;
		}
		bp.step++;
		bp.step_start_t = bp.cur_t;
	}
}

static void
pb_step_connected(void)
{
	if (pbrake_is_set()) {
		/*
		 * Keep resetting the start time to enforce the state delay
		 * after the parking brake is released.
		 */
		bp.step_start_t = bp.cur_t;
	} else if (bp.cur_t - bp.step_start_t >= STATE_TRANS_DELAY) {
		if (!slave_mode) {
			seg_t *seg = list_head(&bp.segs);

			ASSERT(seg != NULL);
			if (seg->backward)
				msg_play(MSG_START_PB);
			else
				msg_play(MSG_START_TOW);
		} else {
			/*
			 * Since we don't know the segs, we'll just
			 * assume it's going to be backward (as that's
			 * the most likely direction anyhow).
			 */
			msg_play(MSG_START_PB);
		}

		bp.step++;
		bp.step_start_t = bp.cur_t;
		bp.last_voice_t = bp.cur_t;
	}
}

static void
pb_step_pushing(void)
{
	if (!slave_mode) {
		dr_setf(&drs.lbrake, 0);
		dr_setf(&drs.rbrake, 0);
		dr_seti(&drs.override_steer, 1);
		if (!bp_run_push()) {
			bp.step++;
			bp.step_start_t = bp.cur_t;
			op_complete = B_TRUE;
		}
	} else {
		/*
		 * Since in slave mode we don't actually know our
		 * tractive effort, just simulate it by following
		 * the aircraft's speed of motion.
		 */
		tug_set_TE_override(bp.tug, B_FALSE);
		/*
		 * In slave mode we don't do anything, just following.
		 */
		acf2tug_steer();
	}
}

static void
pb_step_stopping(void)
{
	tug_set_TE_override(bp.tug, B_FALSE);
	if (!slave_mode) {
		(void) turn_nosewheel(0, STRAIGHT_STEER_RATE);
		push_at_speed(0, bp.veh.max_accel, B_FALSE);
	}
	acf2tug_steer();
	if (ABS(bp.cur_pos.spd) > SPEED_COMPLETE_THRESH) {
		/*
		 * Keep resetting the start time to enforce a delay
		 * once stopped.
		 */
		bp.step_start_t = bp.cur_t;
	} else {
		if (!slave_mode) {
			dr_setf(&drs.lbrake, 0.9);
			dr_setf(&drs.rbrake, 0.9);
		}
		if (bp.cur_t - bp.step_start_t >= STATE_TRANS_DELAY) {
			msg_play(MSG_OP_COMPLETE);
			bp.step++;
			bp.step_start_t = bp.cur_t;
			bp.last_voice_t = bp.cur_t;
		}
	}
}

static void
pb_step_stopped(void)
{
	if (!slave_mode) {
		turn_nosewheel(0, STRAIGHT_STEER_RATE);
		push_at_speed(0, bp.veh.max_accel, B_FALSE);
		dr_setf(&drs.lbrake, 0.9);
		dr_setf(&drs.rbrake, 0.9);
	}
	acf2tug_steer();
	if (!pbrake_is_set()) {
		/*
		 * Keep resetting the start time to enforce a delay
		 * when the parking brake is set.
		 */
		bp.step_start_t = bp.cur_t;
	} else if (bp.cur_t - bp.step_start_t >= msg_dur(MSG_OP_COMPLETE) +
	    STATE_TRANS_DELAY) {
		msg_play(MSG_DISCO);
		bp.step++;
		bp.step_start_t = bp.cur_t;
		bp.last_voice_t = bp.cur_t;
	}
}

static void
pb_step_lowering(void)
{
	double d_t = bp.cur_t - bp.step_start_t;
	double lift_fract = 1 - ((d_t - STATE_TRANS_DELAY) /
	    PB_CONN_LIFT_DURATION);
	double lift;

	if (!slave_mode) {
		turn_nosewheel(0, STRAIGHT_STEER_RATE);
		dr_setf(&drs.lbrake, 0.9);
		dr_setf(&drs.rbrake, 0.9);
	}
	acf2tug_steer();

	if (bp.cur_t - bp.last_voice_t < msg_dur(MSG_OP_COMPLETE)) {
		/*
		 * Keep resetting step_start_t to properly calculate
		 * lift_fract relative to our step_start_t.
		 */
		bp.step_start_t = bp.cur_t;
		return;
	}

	tug_set_lift_in_transit(B_TRUE);

	/* Slight delay after the parking brake ann was made */
	if (d_t <= STATE_TRANS_DELAY)
		return;

	lift_fract = MAX(MIN(lift_fract, 1), 0);

	/* Iterate the lift */
	lift = (bp.tug->info->lift_height * lift_fract) + bp.acf.nw_len +
	    tug_plat_h(bp.tug);
	if (!slave_mode)
		dr_setvf(&drs.leg_len, &lift, bp.acf.nw_i, 1);

	tug_set_lift_pos(lift_fract);
	tug_set_cradle_air_on(bp.tug, B_TRUE, bp.cur_t);
	tug_set_cradle_beeper_on(bp.tug, B_TRUE);

	if (lift_fract == 0) {
		tug_set_cradle_air_on(bp.tug, B_FALSE, bp.cur_t);
		bp.step++;
		bp.step_start_t = bp.cur_t;
	}
}

static bool_t
pb_step_ungrabbing_grab(void)
{
	double d_t = bp.cur_t - bp.step_start_t;
	double cradle_fract = d_t / PB_CRADLE_DELAY;

	cradle_fract = MAX(MIN(cradle_fract, 1), 0);
	tug_set_lift_arm_pos(bp.tug, cradle_fract, B_TRUE);

	if (cradle_fract >= 1.0)
		tug_set_cradle_beeper_on(bp.tug, B_FALSE);

	return (d_t >= PB_CRADLE_DELAY + STATE_TRANS_DELAY);
}

static bool_t
pb_step_ungrabbing_winch(void)
{
	double d_t = bp.cur_t - bp.step_start_t;

	/*
	 * enforce some delays between removing the winch strap and
	 * driving away
	 */
	if (d_t < STATE_TRANS_DELAY)
		return (B_FALSE);

	tug_set_winch_on(bp.tug, B_FALSE);

	if (d_t < 2 * STATE_TRANS_DELAY)
		return (B_FALSE);

	return (B_TRUE);
}

static void
pb_step_ungrabbing(void)
{
	bool_t complete;

	if (bp.tug->info->lift_type == LIFT_GRAB)
		complete = pb_step_ungrabbing_grab();
	else
		complete = pb_step_ungrabbing_winch();

	if (complete) {
		vect2_t dir, p;

		dir = hdg2dir(bp.cur_pos.hdg);
		if (!slave_mode) {
			dr_setf(&drs.lbrake, 0);
			dr_setf(&drs.rbrake, 0);
		}
		p = vect2_add(bp.cur_pos.pos, vect2_scmul(dir,
		    -bp.acf.nw_z + bp.tug->info->apch_dist));

		tug_set_lift_in_transit(B_FALSE);
		tug_set_TE_override(bp.tug, B_FALSE);
		(void) tug_drive2point(bp.tug, p, bp.cur_pos.hdg);

		bp.step++;
		bp.step_start_t = bp.cur_t;
	}
}

/*
 * This determines whether we perform a right or left turn. The direction of
 * the turn depends on whether our original starting position is to the left
 * or to the right of the aircraft.
 */
static bool_t
tug_clear_is_right(void)
{
	if (VECT2_EQ(bp.start_pos, bp.cur_pos.pos)) {
		return (B_TRUE);
	} else {
		return (rel_hdg(bp.cur_pos.hdg, dir2hdg(vect2_sub(bp.start_pos,
		    bp.cur_pos.pos))) >= 0);
	}
}

static void
pb_step_closing_cradle(void)
{
	double d_t = bp.cur_t - bp.step_start_t;

	tug_set_lift_in_transit(B_TRUE);
	tug_set_lift_arm_pos(bp.tug, 1 - d_t / PB_CRADLE_DELAY, B_FALSE);
	tug_set_tire_sense_pos(bp.tug, 1 - d_t / PB_CRADLE_DELAY);
	tug_set_lift_pos(d_t / PB_CRADLE_DELAY);

	if (d_t >= PB_CRADLE_DELAY) {
		tug_set_cradle_beeper_on(bp.tug, B_FALSE);
		tug_set_lift_in_transit(B_FALSE);
	}

	if (d_t >= PB_CRADLE_DELAY + STATE_TRANS_DELAY) {
		/* determine which direction we'll drive away */
		bool_t right = tug_clear_is_right();

		msg_play(right ? MSG_DONE_RIGHT : MSG_DONE_LEFT);
		tug_set_cradle_lights_on(B_FALSE);
		tug_set_hazard_lights_on(B_FALSE);

		bp.step++;
		bp.step_start_t = bp.cur_t;
		bp.last_voice_t = bp.cur_t;
	}
}

static void
pb_step_starting2clear(void)
{
	bool_t right;
	vect2_t turn_p, abeam_p, dir, norm_dir;
	double turn_hdg, back_hdg, square_side;

	/* Let the message play out before starting to move */
	if (bp.cur_t - bp.step_start_t < MAX(msg_dur(MSG_DONE_RIGHT),
	    msg_dur(MSG_DONE_LEFT)) + STATE_TRANS_DELAY)
		return;

	right = tug_clear_is_right();
	square_side = MAX(4 * bp.tug->veh.wheelbase, 1.5 * bp.veh.wheelbase);

	dir = hdg2dir(bp.cur_pos.hdg);
	norm_dir = vect2_norm(dir, right);

	/*
	 * turn_p is offset 3x tug wheelbase forward and
	 * half square_side to the direction of the turn.
	 */
	turn_p = vect2_add(bp.tug->pos.pos, vect2_scmul(dir,
	    3 * bp.tug->veh.wheelbase));
	turn_p = vect2_add(turn_p, vect2_scmul(norm_dir,
	    square_side / 2));
	turn_hdg = normalize_hdg(bp.cur_pos.hdg + (right ? 90 : -90));

	/*
	 * abeam point is displaced from turn_p back 2x tug wheelbase,
	 * 4x tug wheelbase in the direction of the turn and going the
	 * opposite way to the aircraft at a 45 degree angle.
	 */
	abeam_p = vect2_add(turn_p, vect2_scmul(vect2_neg(dir),
	    2 * bp.tug->veh.wheelbase));
	abeam_p = vect2_add(abeam_p, vect2_scmul(norm_dir,
	    4 * bp.tug->veh.wheelbase));
	back_hdg = normalize_hdg(turn_hdg + (right ? 45 : -45));

	VERIFY(tug_drive2point(bp.tug, turn_p, turn_hdg));
	VERIFY(tug_drive2point(bp.tug, abeam_p, back_hdg));

	bp.step++;
	bp.step_start_t = bp.cur_t;
}

static void
drive_away_fallback(void)
{
	/*
	 * If all else fails, reset the tug's position to get rid of an
	 * intermediate turn segment and just send the tug straight for
	 * a fixed distance.
	 */
	vect2_t end_p = vect2_add(bp.tug->pos.pos,
	    vect2_scmul(hdg2dir(bp.tug->pos.hdg), TUG_DRIVE_AWAY_DIST));

	tug_set_pos(bp.tug, bp.tug->pos.pos, bp.tug->pos.hdg, 0);
	VERIFY(tug_drive2point(bp.tug, end_p, bp.tug->pos.hdg));
}

static void
pb_step_clear_signal(void)
{
	double acf2start_lat_displ, acf2start_long_displ;
	vect2_t acf2start, acfdir;

	tug_set_clear_signal(B_TRUE, tug_clear_is_right());

	if (bp.cur_t - bp.step_start_t < CLEAR_SIGNAL_DELAY)
		return;

	/*
	 * In order to determine if we should be even attempting to reach
	 * our starting point, we make sure that start_pos isn't within a
	 * box as follows:
	 *                 -4 x wheelbase
	 *                   |<----->|
	 *                   |       |
	 *           ------- +-------+------------------>>> (to infinity)
	 *  1.5x     ^       |
	 * wheelbase |       |       |
	 *           v______ |   |___|__
	 *                   |   |   |
	 *                   |       |
	 *                   |
	 *                   +-------------------------->>>
	 */
	acf2start = vect2_sub(bp.start_pos, bp.cur_pos.pos);
	acfdir = hdg2dir(bp.cur_pos.hdg);
	acf2start_lat_displ = fabs(vect2_dotprod(vect2_norm(acfdir, B_TRUE),
	    acf2start));
	acf2start_long_displ = vect2_dotprod(acfdir, acf2start);

	if (acf2start_lat_displ < 1.5 * bp.veh.wheelbase &&
	    acf2start_long_displ > -4 * bp.veh.wheelbase) {
		drive_away_fallback();
	} else {
		double rhdg = fabs(rel_hdg(bp.tug->pos.hdg,
		    dir2hdg(vect2_sub(bp.start_pos, bp.tug->pos.pos))));
		/*
		 * start_pos seems far enough away from the aircraft that
		 * it won't be a problem if we drive to it. Just make sure
		 * we're not trying to back into it.
		 */
		if (rhdg >= 90 || !tug_drive2point(bp.tug, bp.start_pos,
		    bp.start_hdg)) {
			/*
			 * It's possible the start_pos is beyond a 90 degree
			 * turn, so we'd attempt to back into it. Try to stick
			 * in an intermediate 90-degree turn in its direction.
			 */
			bool_t right = (rel_hdg(bp.tug->pos.hdg, dir2hdg(
			    vect2_sub(bp.start_pos, bp.tug->pos.pos))) >= 0);
			vect2_t dir = hdg2dir(bp.tug->pos.hdg);
			vect2_t turn_p = vect2_add(bp.tug->pos.pos,
			    vect2_scmul(dir, 2 * bp.tug->veh.wheelbase));
			turn_p = vect2_add(turn_p, vect2_scmul(vect2_norm(dir,
			    right), 2 * bp.tug->veh.wheelbase));
			if (!tug_drive2point(bp.tug, turn_p, normalize_hdg(
			    bp.tug->pos.hdg + (right ? 90 : -90))) ||
			    !tug_drive2point(bp.tug, bp.start_pos,
			    bp.start_hdg)) {
				drive_away_fallback();
			}
		}
	}
	tug_set_clear_signal(B_FALSE, B_FALSE);
	bp.step++;
	bp.step_start_t = bp.cur_t;
}

static float
bp_run(float elapsed, float elapsed2, int counter, void *refcon)
{
	UNUSED(elapsed);
	UNUSED(elapsed2);
	UNUSED(counter);
	UNUSED(refcon);

	bp_gather();

	if (bp.cur_t <= bp.last_t)
		return (-1);

	bp.d_pos.pos = vect2_sub(bp.cur_pos.pos, bp.last_pos.pos);
	bp.d_pos.hdg = bp.cur_pos.hdg - bp.last_pos.hdg;
	bp.d_pos.spd = bp.cur_pos.spd - bp.last_pos.spd;
	bp.d_t = bp.cur_t - bp.last_t;

	ASSERT(bp.tug != NULL || bp.step <= PB_STEP_TUG_LOAD);
	if (bp.tug != NULL) {
		/* drive slowly while approaching & moving away from acf */
		tug_run(bp.tug, bp.d_t,
		    bp.step == PB_STEP_DRIVING_UP_CONNECT ||
		    bp.step == PB_STEP_MOVING_AWAY);
		tug_anim(bp.tug, bp.d_t, bp.cur_t);
	}

	if (!slave_mode) {
		if (bp.step >= PB_STEP_DRIVING_UP_CONNECT &&
		    bp.step <= PB_STEP_MOVING_AWAY)
			dr_seti(&drs.override_steer, 1);
		else
			dr_seti(&drs.override_steer, 0);
	}

	/*
	 * If we have no segs, means the user stopped the operation.
	 * Jump to the appropriate state. If we haven't connected yet,
	 * just disappear. If we have, jump to the stopping state.
	 */
	if (!late_plan_requested &&
	    ((!slave_mode && list_head(&bp.segs) == NULL) ||
	    (slave_mode && op_complete))) {
		if (bp.step < PB_STEP_GRABBING) {
			bp_complete();
			return (0);
		}
		if (bp.step < PB_STEP_STOPPING) {
			bp.step = PB_STEP_STOPPING;
		}
	}

	/*
	 * When performing quick debugging, skip the whole driving-up phase.
	 */
	if (!slave_mode && bp.tug != NULL && bp.tug->info->quick_debug) {
		if (bp.step < PB_STEP_CONNECTED) {
			double lift = bp.tug->info->lift_height + bp.acf.nw_len;
			bp.step = PB_STEP_CONNECTED;
			tug_set_lift_pos(1);
			tug_set_lift_arm_pos(bp.tug, 0, B_TRUE);
			dr_setvf(&drs.leg_len, &lift, bp.acf.nw_i, 1);
			/*
			 * Just a quick'n'dirty way of removing all tug driving
			 * segs. The actual tug position will be updated in
			 * draw_tugs.
			 */
			tug_set_pos(bp.tug, ZERO_VECT2, 0, 0);
		} else if (bp.step == PB_STEP_UNGRABBING) {
#if 0
			dr_setvf(&drs.leg_len, &bp.acf.nw_len, bp.acf.nw_i, 1);
			bp_complete();
			return (0);
#endif
		}
	}

	switch (bp.step) {
	case PB_STEP_OFF:
		VERIFY(bp.step != PB_STEP_OFF);
	case PB_STEP_TUG_LOAD:
		ASSERT3P(bp.tug, ==, NULL);
		if (!pb_step_tug_load())
			return (0);
		break;
	case PB_STEP_START:
		pb_step_start();
		break;
	case PB_STEP_DRIVING_UP_CLOSE:
		pb_step_driving_up_close();
		break;
	case PB_STEP_OPENING_CRADLE: {
		double d_t = bp.cur_t - bp.step_start_t;

		tug_set_lift_in_transit(B_TRUE);
		tug_set_lift_pos(1 - d_t / PB_CRADLE_DELAY);
		tug_set_lift_arm_pos(bp.tug, d_t / PB_CRADLE_DELAY, B_FALSE);
		tug_set_tire_sense_pos(bp.tug, d_t / PB_CRADLE_DELAY);
		if (d_t >= PB_CRADLE_DELAY) {
			tug_set_lift_in_transit(B_FALSE);
			tug_set_cradle_beeper_on(bp.tug, B_FALSE);
		}
		if (d_t >= PB_CRADLE_DELAY + STATE_TRANS_DELAY) {
			msg_play(MSG_RDY2CONN);
			bp.step++;
			bp.step_start_t = bp.cur_t;
			bp.last_voice_t = bp.cur_t;
		}
		break;
	}
	case PB_STEP_WAITING_FOR_PBRAKE:
		pb_step_waiting_for_pbrake();
		break;
	case PB_STEP_DRIVING_UP_CONNECT:
		pb_step_driving_up_connect();
		break;
	case PB_STEP_GRABBING:
		pb_step_grab();
		break;
	case PB_STEP_LIFTING:
		pb_step_lift();
		break;
	case PB_STEP_CONNECTED:
		pb_step_connected();
		break;
	case PB_STEP_STARTING:
		if (!slave_mode)
			dr_seti(&drs.override_steer, 1);
		if (bp.cur_t - bp.step_start_t >= PB_START_DELAY) {
			bp.step++;
			bp.step_start_t = bp.cur_t;
		} else {
			turn_nosewheel(0, STRAIGHT_STEER_RATE);
			push_at_speed(0, bp.veh.max_accel, B_FALSE);
		}
		break;
	case PB_STEP_PUSHING:
		pb_step_pushing();
		break;
	case PB_STEP_STOPPING:
		pb_step_stopping();
		break;
	case PB_STEP_STOPPED:
		pb_step_stopped();
		break;
	case PB_STEP_LOWERING:
		pb_step_lowering();
		break;
	case PB_STEP_UNGRABBING:
		pb_step_ungrabbing();
		break;
	case PB_STEP_MOVING_AWAY:
		if (bp.tug->info->lift_type == LIFT_WINCH && !slave_mode) {
			/*
			 * When moving the tug away from the aircraft, the
			 * aircraft will have been positioned on the platform.
			 * Slowly lower the nosewheel the rest of the way.
			 */
			double dist = vect2_dist(bp.cur_pos.pos,
			    bp.tug->pos.pos);
			const tug_info_t *ti = bp.tug->info;
			double plat_len = ti->lift_wall_z - ti->plat_z;
			double x, lift;

			dist -= (-bp.acf.nw_z);
			dist -= (-ti->lift_wall_z);
			x = 1 - (dist / plat_len);
			x = MIN(MAX(x, 0), 1);
			lift = ti->plat_h * x + bp.acf.nw_len;
			dr_setvf(&drs.leg_len, &lift, bp.acf.nw_i, 1);
		}
		if (tug_is_stopped(bp.tug)) {
			tug_set_cradle_beeper_on(bp.tug, B_TRUE);
			bp.step++;
			bp.step_start_t = bp.cur_t;
		}
		break;
	case PB_STEP_CLOSING_CRADLE:
		pb_step_closing_cradle();
		break;
	case PB_STEP_STARTING2CLEAR:
		pb_step_starting2clear();
		break;
	case PB_STEP_MOVING2CLEAR:
		if (tug_is_stopped(bp.tug)) {
			bp.step++;
			bp.step_start_t = bp.cur_t;
		}
		break;
	case PB_STEP_CLEAR_SIGNAL:
		pb_step_clear_signal();
		break;
	case PB_STEP_DRIVING_AWAY:
		if (tug_is_stopped(bp.tug) ||
		    bp.cur_t - bp.step_start_t > MAX_DRIVING_AWAY_DELAY) {
			bp_complete();
			/*
			 * Can't unregister floop from within, so just tell
			 * X-Plane to not call us anymore.
			 */
			return (0);
		}
		break;
	}

	bp.last_pos = bp.cur_pos;
	bp.last_t = bp.cur_t;
	dr_getvf(&drs.tire_steer_cmd, &bp.last_steer, bp.acf.nw_i, 1);

	return (-1);
}

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

#define	CLICK_DISPL_THRESH	5			/* pixels */
#define	US_PER_CLICK_ACCEL	60000			/* microseconds */
#define	US_PER_CLICK_DEACCEL	120000			/* microseconds */
#define	MAX_ACCEL_MULT		10
#define	WHEEL_ANGLE_MULT	0.5

#define	MIN_BUTTON_SCALE	0.5

#define	BP_PLANNER_VISIBILITY	40000	/* meters */

#define	PREDICTION_DRAWING_PHASE	xplm_Phase_Airplanes
#define	PREDICTION_DRAWING_PHASE_BEFORE	0

typedef struct {
	const char	*name;
	XPLMCommandRef	cmd;
	vect3_t		incr;
} view_cmd_info_t;

typedef struct {
	const char	*filename;	/* PNG filename in data/icons/<lang> */
	const int	vk;		/* function virtual key, -1 if none */
	GLuint		tex;		/* OpenGL texture object */
	GLbyte		*tex_data;	/* RGBA texture data */
	int		w, h;		/* button width & height in pixels */
} button_t;

static vect3_t		cam_pos;
static double		cam_height;
static double		cam_hdg;
static double		cursor_hdg;
static list_t		pred_segs;
static XPLMCommandRef	circle_view_cmd;
static XPLMWindowID	fake_win;
static vect2_t		cursor_world_pos;
static bool_t		force_root_win_focus = B_TRUE;
static float		saved_visibility;
static int		saved_cloud_types[3];
static bool_t		saved_real_wx;

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

static button_t buttons[] = {
    { .filename = "move_view.png", .vk = -1, .tex = 0, .tex_data = NULL },
    { .filename = "place_seg.png", .vk = -1, .tex = 0, .tex_data = NULL },
    { .filename = "rotate_seg.png", .vk = -1, .tex = 0, .tex_data = NULL },
    { .filename = "", .vk = -1, .tex = 0, .tex_data = NULL, .h = 64 },
    {
	.filename = "accept_plan.png", .vk = XPLM_VK_RETURN, .tex = 0,
	.tex_data = NULL
    },
    {
	.filename = "delete_seg.png", .vk = XPLM_VK_DELETE, .tex = 0,
	.tex_data = NULL
    },
    { .filename = "", .vk = -1, .tex = 0, .tex_data = NULL, .h = 64 },
    {
	.filename = "cancel_plan.png", .vk = XPLM_VK_ESCAPE, .tex = 0,
	.tex_data = NULL
    },
    {
	.filename = "conn_first.png", .vk = XPLM_VK_SPACE, .tex = 0,
	.tex_data = NULL
    },
    { .filename = NULL }
};
static int button_hit = -1, button_lit = -1;

static bool_t
load_icon(button_t *btn)
{
	char *filename;
	FILE *fp;
	size_t rowbytes;
	png_bytep *volatile rowp = NULL;
	png_structp pngp = NULL;
	png_infop infop = NULL;
	volatile bool_t res = B_TRUE;
	uint8_t header[8];

	/* try the localized version first */
	filename = mkpathname(bp_xpdir, bp_plugindir, "data", "icons",
	    acfutils_xplang2code(XPLMGetLanguage()), btn->filename, NULL);
	if (!file_exists(filename, NULL)) {
		/* if the localized version failed, try the English version */
		free(filename);
		filename = mkpathname(bp_xpdir, bp_plugindir, "data", "icons",
		    "en", btn->filename, NULL);
	}
	fp = fopen(filename, "rb");
	if (fp == NULL) {
		logMsg("Cannot open file %s: %s", filename, strerror(errno));
		res = B_FALSE;
		goto out;
	}
	if (fread(header, 1, sizeof (header), fp) != 8 ||
	    png_sig_cmp(header, 0, sizeof (header)) != 0) {
		logMsg("Cannot open file %s: invalid PNG header", filename);
		res = B_FALSE;
		goto out;
	}
	pngp = png_create_read_struct(PNG_LIBPNG_VER_STRING, NULL, NULL, NULL);
	VERIFY(pngp != NULL);
	infop = png_create_info_struct(pngp);
	VERIFY(infop != NULL);
	if (setjmp(png_jmpbuf(pngp))) {
		logMsg("Cannot open file %s: libpng error in init_io",
		    filename);
		res = B_FALSE;
		goto out;
	}
	png_init_io(pngp, fp);
	png_set_sig_bytes(pngp, 8);

	if (setjmp(png_jmpbuf(pngp))) {
		logMsg("Cannot open file %s: libpng read info failed",
		    filename);
		res = B_FALSE;
		goto out;
	}
	png_read_info(pngp, infop);
	btn->w = png_get_image_width(pngp, infop);
	btn->h = png_get_image_height(pngp, infop);

	if (png_get_color_type(pngp, infop) != PNG_COLOR_TYPE_RGBA) {
		logMsg("Bad icon file %s: need color type RGBA", filename);
		res = B_FALSE;
		goto out;
	}
	if (png_get_bit_depth(pngp, infop) != 8) {
		logMsg("Bad icon file %s: need 8-bit depth", filename);
		res = B_FALSE;
		goto out;
	}
	rowbytes = png_get_rowbytes(pngp, infop);

	rowp = malloc(sizeof (*rowp) * btn->h);
	VERIFY(rowp != NULL);
	for (int i = 0; i < btn->h; i++) {
		rowp[i] = malloc(rowbytes);
		VERIFY(rowp[i] != NULL);
	}

	if (setjmp(png_jmpbuf(pngp))) {
		logMsg("Bad icon file %s: error reading image file", filename);
		res = B_FALSE;
		goto out;
	}
	png_read_image(pngp, rowp);

	btn->tex_data = malloc(btn->h * rowbytes);
	for (int i = 0; i < btn->h; i++)
		memcpy(&btn->tex_data[i * rowbytes], rowp[i], rowbytes);

	glGenTextures(1, &btn->tex);
	glBindTexture(GL_TEXTURE_2D, btn->tex);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, btn->w, btn->h, 0, GL_RGBA,
	    GL_UNSIGNED_BYTE, btn->tex_data);

out:
	if (pngp != NULL)
		png_destroy_read_struct(&pngp, &infop, NULL);
	if (rowp != NULL) {
		for (int i = 0; i < btn->h; i++)
			free(rowp[i]);
		free(rowp);
	}
	if (fp != NULL)
		fclose(fp);
	free(filename);

	return (res);
}

static bool_t
load_buttons(void)
{

	for (int i = 0; buttons[i].filename != NULL; i++) {
		/* skip spacers */
		if (strcmp(buttons[i].filename, "") == 0)
			continue;
		if (!load_icon(&buttons[i])) {
			unload_buttons();
			return (B_FALSE);
		}
	}

	return (B_TRUE);
}

static void
unload_buttons(void)
{
	for (int i = 0; buttons[i].filename != NULL; i++) {
		if (buttons[i].tex != 0)
			glDeleteTextures(1, &buttons[i].tex);
		if (buttons[i].tex_data != NULL)
			free(buttons[i].tex_data);
	}
}

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

	while ((seg = list_remove_head(&pred_segs)) != NULL)
		free(seg);

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

	XPLMDestroyProbe(probe);

	return (1);
}

static void
fake_win_draw(XPLMWindowID inWindowID, void *inRefcon)
{
	double scale;
	int w, h, h_buttons, h_off;
	UNUSED(inWindowID);
	UNUSED(inRefcon);

	XPLMGetScreenSize(&w, &h);
	XPLMSetWindowGeometry(fake_win, 0, h, w, 0);

	if (!XPLMIsWindowInFront(fake_win))
		XPLMBringWindowToFront(fake_win);
	if (force_root_win_focus)
		XPLMTakeKeyboardFocus(0);

	XPLMSetGraphicsState(0, 1, 0, 0, 1, 0, 0);

	h_buttons = 0;
	for (int i = 0; buttons[i].filename != NULL; i++)
		h_buttons += buttons[i].h;

	scale = (double)h / h_buttons;
	scale = MIN(scale, 1);
	/* don't draw the buttons if we don't have enough space for them */
	if (scale < MIN_BUTTON_SCALE)
		return;

	h_off = (h + (h_buttons * scale)) / 2;
	for (int i = 0; buttons[i].filename != NULL;
	    i++, h_off -= buttons[i].h * scale) {
		button_t *btn = &buttons[i];

		if (btn->tex == 0)
			continue;
		glBindTexture(GL_TEXTURE_2D, btn->tex);
		glBegin(GL_QUADS);
		glTexCoord2f(0, 1);
		glVertex2f(w - btn->w * scale, h_off - btn->h * scale);
		glTexCoord2f(0, 0);
		glVertex2f(w - btn->w * scale, h_off);
		glTexCoord2f(1, 0);
		glVertex2f(w, h_off);
		glTexCoord2f(1, 1);
		glVertex2f(w, h_off - btn->h * scale);
		glEnd();

		if (i == button_hit) {
			/*
			 * If this button was hit by a mouse click, highlight
			 * it by drawing a translucent white quad over it.
			 */
			XPLMSetGraphicsState(0, 0, 0, 0, 1, 0, 0);
			glColor4f(1, 1, 1, 0.3);
			glBegin(GL_QUADS);
			glVertex2f(w - btn->w * scale, h_off - btn->h * scale);
			glVertex2f(w - btn->w * scale, h_off);
			glVertex2f(w, h_off);
			glVertex2f(w, h_off - btn->h * scale);
			glEnd();
			XPLMSetGraphicsState(0, 1, 0, 0, 1, 0, 0);
		} else if (i == button_lit) {
			XPLMSetGraphicsState(0, 0, 0, 0, 1, 0, 0);
			glColor4f(1, 1, 1, 1);
			glLineWidth(1);
			glBegin(GL_LINES);
			glVertex2f(w - btn->w * scale, h_off - btn->h * scale);
			glVertex2f(w - btn->w * scale, h_off);
			glVertex2f(w - btn->w * scale, h_off);
			glVertex2f(w, h_off);
			glVertex2f(w, h_off);
			glVertex2f(w, h_off - btn->h * scale);
			glVertex2f(w, h_off - btn->h * scale);
			glVertex2f(w - btn->w * scale, h_off - btn->h * scale);
			glEnd();
			XPLMSetGraphicsState(0, 1, 0, 0, 1, 0, 0);
		}
	}
}

static int
button_hit_check(int x, int y)
{
	double scale;
	int w, h, h_buttons, h_off;

	XPLMGetScreenSize(&w, &h);

	h_buttons = 0;
	for (int i = 0; buttons[i].filename != NULL; i++)
		h_buttons += buttons[i].h;

	scale = (double)h / h_buttons;
	scale = MIN(scale, 1);
	if (scale < MIN_BUTTON_SCALE)
		return (-1);

	h_off = (h + (h_buttons * scale)) / 2;
	for (int i = 0; buttons[i].filename != NULL;
	    i++, h_off -= buttons[i].h * scale) {
		if (x >= w - buttons[i].w * scale && x <= w &&
		    y >= h_off - buttons[i].h * scale && y <= h_off &&
		    buttons[i].vk != -1)
			return (i);
	}

	return (-1);
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
	int lit;

	UNUSED(inWindowID);
	UNUSED(x);
	UNUSED(y);
	UNUSED(inRefcon);

	if ((lit = button_hit_check(x, y)) != -1 &&buttons[lit].vk != -1)
		button_lit = lit;
	else
		button_lit = -1;

	return (xplm_CursorDefault);
}

static int
fake_win_click(XPLMWindowID inWindowID, int x, int y, XPLMMouseStatus inMouse,
    void *inRefcon)
{
	static int last_x = 0, last_y = 0;
	static int down_x = 0, down_y = 0;
	static bool_t dragging = B_FALSE;

	UNUSED(inWindowID);
	UNUSED(inRefcon);

	/*
	 * The mouse handling logic is as follows:
	 * 1) On mouse-down, we memorize where the click started, check if
	 *	we hit a button and reset the dragging variable.
	 * 2) While receiving dragging events, we check if the displacement
	 *	from the original start position is sufficient to consider
	 *	this click-and-drag. If it is, we set the `dragging' flag
	 *	to true. This inhibits clicking.
	 * 3) On mouse-up, if there was no click-and-drag, check if the
	 *	button we're on is still the same as what we hit on the
	 *	initial mouse-down (prevents from clicking one button and
	 *	sliding onto another one). If no button was hit and no
	 *	click-and-drag took place, count it as a click on the screen
	 *	attempting to place a segment.
	 */
	if (inMouse == xplm_MouseDown) {
		last_x = down_x = x;
		last_y = down_y = y;
		force_root_win_focus = B_FALSE;
		button_hit = button_hit_check(x, y);
		dragging = B_FALSE;
	} else if (inMouse == xplm_MouseDrag) {
		if (!dragging) {
			dragging = (ABS(x - down_x) >= CLICK_DISPL_THRESH ||
			    ABS(y - down_y) >= CLICK_DISPL_THRESH);
		}
		if (dragging && (x != last_x || y != last_y) &&
		    button_hit == -1) {
			int w, h;
			double fov_h, fov_v, rx, ry, rw, rh, dx, dy;
			vect2_t v;

			XPLMGetScreenSize(&w, &h);
			rx = ((double)x - last_x) / (w / 2);
			ry = ((double)y - last_y) / (h / 2);
			fov_h = DEG2RAD(dr_getf(&drs.camera_fov_h));
			fov_v = DEG2RAD(dr_getf(&drs.camera_fov_v));
			rw = cam_height * tan(fov_h / 2);
			rh = cam_height * tan(fov_v / 2);
			dx = rw * rx;
			dy = rh * ry;
			v = vect2_rot(VECT2(dx, dy), cam_hdg);
			cam_pos.x -= v.x;
			cam_pos.z -= v.y;
			last_x = x;
			last_y = y;
		}
	} else {
		if (!dragging) {
			if (button_hit != -1 &&
			    button_hit == button_hit_check(x, y)) {
				/* simulate a key press */
				ASSERT(buttons[button_hit].vk != -1);
				key_sniffer(0, xplm_DownFlag,
				    buttons[button_hit].vk, NULL);
			} else {
				/*
				 * Transfer whatever is in pred_segs to
				 * the normal segments and clear pred_segs.
				 */
				list_move_tail(&bp.segs, &pred_segs);
			}
		}
		button_hit = -1;
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
		XPLMCommandOnce(XPLMFindCommand("BetterPushback/stop_planner"));
		return (0);
	case XPLM_VK_ESCAPE:
		bp_delete_all_segs();
		XPLMCommandOnce(XPLMFindCommand("BetterPushback/stop_planner"));
		return (0);
	case XPLM_VK_CLEAR:
	case XPLM_VK_BACK:
	case XPLM_VK_DELETE:
		/* Delete the segments up to the next user-placed segment */
		free(list_remove_tail(&bp.segs));
		for (seg_t *seg = list_tail(&bp.segs); seg != NULL &&
		    !seg->user_placed; seg = list_tail(&bp.segs)) {
			list_remove_tail(&bp.segs);
			free(seg);
		}
		return (0);
	case XPLM_VK_SPACE:
		bp_delete_all_segs();
		XPLMCommandOnce(XPLMFindCommand(
		    "BetterPushback/connect_first"));
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
	char icao[8] = { 0 };

	if (cam_inited || !bp_init())
		return (B_FALSE);

	if (!acf_is_compatible()) {
		XPLMSpeakString(_("Pushback failure: aircraft is incompatible "
		    "with BetterPushback."));
		return (B_FALSE);
	}

	(void) find_nearest_airport(icao);
	if (!tug_available(dr_getf(&drs.mtow), bp.acf.nw_len, bp.acf.tirrad,
	    bp.acf.nw_type, strcmp(icao, "") != 0 ? icao : NULL)) {
		XPLMSpeakString(_("Pushback failure: no suitable tug for your "
		    "aircraft."));
		return (B_FALSE);
	}

#ifndef	PB_DEBUG_INTF
	if (vect3_abs(VECT3(dr_getf(&drs.vx), dr_getf(&drs.vy),
	    dr_getf(&drs.vz))) >= 1) {
		XPLMSpeakString(_("Can't start planner: aircraft not "
		    "stationary."));
		return (B_FALSE);
	}
	if (!pbrake_is_set()) {
		XPLMSpeakString(_("Can't start pushback planner: please "
		    "set the parking brake first."));
		return (B_FALSE);
	}
	if (bp_started) {
		XPLMSpeakString(_("Can't start planner: pushback already in "
		    "progress. Please stop the pushback operation first."));
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

	XPLMRegisterDrawCallback(draw_prediction, PREDICTION_DRAWING_PHASE,
	    PREDICTION_DRAWING_PHASE_BEFORE, NULL);

	for (int i = 0; view_cmds[i].name != NULL; i++) {
		view_cmds[i].cmd = XPLMFindCommand(view_cmds[i].name);
		VERIFY(view_cmds[i].cmd != NULL);
		XPLMRegisterCommandHandler(view_cmds[i].cmd, move_camera,
		    1, (void *)(uintptr_t)i);
	}
	XPLMRegisterKeySniffer(key_sniffer, 1, NULL);

	/* If the list of segs is empty, try to reload the saved state */
	if (list_head(&bp.segs) == NULL) {
		route_load(GEO_POS2(dr_getf(&drs.lat), dr_getf(&drs.lon)),
		    dr_getf(&drs.hdg), &bp.segs);
	}

	/*
	 * While the planner is active, we override the current visibility
	 * and real weather usage, so that the user can clearly see the path
	 * while planning. After we're done, we'll restore the settings.
	 */
	saved_visibility = dr_getf(&drs.visibility);
	saved_cloud_types[0] = dr_geti(&drs.cloud_types[0]);
	saved_cloud_types[1] = dr_geti(&drs.cloud_types[1]);
	saved_cloud_types[2] = dr_geti(&drs.cloud_types[2]);
	saved_real_wx = dr_geti(&drs.use_real_wx);

	dr_setf(&drs.visibility, BP_PLANNER_VISIBILITY);
	dr_seti(&drs.cloud_types[0], 0);
	dr_seti(&drs.cloud_types[1], 0);
	dr_seti(&drs.cloud_types[2], 0);
	dr_seti(&drs.use_real_wx, 0);

	cam_inited = B_TRUE;

	return (B_TRUE);
}

bool_t
bp_cam_stop(void)
{
	seg_t *seg;
	XPLMCommandRef cockpit_view_cmd;

	if (!cam_inited)
		return (B_FALSE);

	while ((seg = list_remove_head(&pred_segs)) != NULL)
		free(seg);
	list_destroy(&pred_segs);

	XPLMUnregisterDrawCallback(draw_prediction, PREDICTION_DRAWING_PHASE,
	    PREDICTION_DRAWING_PHASE_BEFORE, NULL);
	XPLMDestroyWindow(fake_win);

	for (int i = 0; view_cmds[i].name != NULL; i++) {
		XPLMUnregisterCommandHandler(view_cmds[i].cmd, move_camera,
		    1, (void *)(uintptr_t)i);
	}
	XPLMUnregisterKeySniffer(key_sniffer, 1, NULL);

	cockpit_view_cmd = XPLMFindCommand("sim/view/3d_cockpit_cmnd_look");
	ASSERT(cockpit_view_cmd != NULL);
	XPLMCommandOnce(cockpit_view_cmd);

	dr_setf(&drs.visibility, saved_visibility);
	dr_seti(&drs.cloud_types[0], saved_cloud_types[0]);
	dr_seti(&drs.cloud_types[1], saved_cloud_types[1]);
	dr_seti(&drs.cloud_types[2], saved_cloud_types[2]);
	dr_seti(&drs.use_real_wx, saved_real_wx);

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
