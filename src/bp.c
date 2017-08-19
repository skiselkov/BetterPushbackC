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
#include "cfg.h"
#include "msg.h"
#include "xplane.h"

/*#define	PB_DEBUG_INTF*/

#define	MIN_XPLANE_VERSION	10500	/* X-Plane 10.50 */
#define	MIN_XPLANE_VERSION_STR	"10.50"	/* X-Plane 10.50 */

#define	MAX_FWD_SPEED		4	/* m/s [~8 knots] */
#define	MAX_REV_SPEED		1.11	/* m/s [4 km/h, "walking speed"] */
#define	NORMAL_ACCEL		0.25	/* m/s^2 */
#define	NORMAL_DECEL		0.17	/* m/s^2 */
#define	BRAKE_PEDAL_THRESH	0.03	/* brake pedal angle, 0..1 */
#define	FORCE_PER_TON		5000	/* max push force per ton, Newtons */
/*
 * X-Plane 10's tire model is a bit less forgiving of slow creeping,
 * so bump the minimum breakaway speed on that version.
 */
#define	BREAKAWAY_THRESH	(bp_xp_ver >= 11000 ? 0.09 : 0.35)
#define	SEG_TURN_MULT		0.9	/* leave 10% for oversteer */
#define	SPEED_COMPLETE_THRESH	0.08	/* m/s */
/* beyond this our push algos go nuts */
#define	MAX_STEER_ANGLE		(bp_xp_ver < 11000 ? 50 : 75)
#define	MIN_STEER_ANGLE		40	/* minimum sensible tire steer angle */
#define	MAX_FWD_ANG_VEL		4.5	/* degrees per second */
#define	MAX_REV_ANG_VEL		2.5	/* degrees per second */
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

#define	TUG_APPCH_LONG_DIST	(6 * bp_ls.tug->veh.wheelbase)

#define	MIN_RADIO_VOLUME_THRESH	0.1

/*
 * When stopping the operation, tug and aircraft steering deflections must
 * be below these thresholds before we let the aircraft come to a complete
 * stop. Otherwise we continue pushing/towing at MIN_SPEED_XP10 to let the
 * steering straighten out.
 */
#define	TOW_COMPLETE_TUG_STEER_THRESH	5	/* degrees */
#define	TOW_COMPLETE_ACF_STEER_THRESH	2.5	/* degrees */

/*
 * When we get within this distance of the end of a straight segment that
 * terminates our pushback path, we neutralize steering to be able to stop
 * exactly on the dot.
 */
#define	NEARING_END_THRESHOLD		1	/* meters */

typedef struct {
	const char	*acf;
	const char	*author;
} acf_info_t;

static struct {
	dr_t	lbrake, rbrake;
	dr_t	pbrake, pbrake_rat;
	bool_t	pbrake_is_custom;
	dr_t	rot_force_N;
	dr_t	axial_force;
	dr_t	override_planepath;
	dr_t	local_x, local_y, local_z;
	dr_t	local_vx, local_vy, local_vz;
	dr_t	lat, lon;
	dr_t	pitch, roll, hdg, quaternion;
	dr_t	vx, vy, vz;
	dr_t	sim_time;
	dr_t	acf_mass;
	dr_t	mtow;
	dr_t	tire_z, tire_x, leg_len, tirrad;
	dr_t	nw_steerdeg1, nw_steerdeg2;
	dr_t	tire_steer_cmd;
	dr_t	override_steer;
	dr_t	nw_steer_on;
	dr_t	gear_types;
	dr_t	gear_steers;
	dr_t	gear_on_ground;
	dr_t	onground_any;
	dr_t	gear_deploy;
	dr_t	num_engns;
	dr_t	engn_running;
	dr_t	acf_livery_path;

	dr_t	camera_fov_h, camera_fov_v;
	dr_t	view_is_ext;
	dr_t	visibility;
	dr_t	cloud_types[3];
	dr_t	use_real_wx;

	dr_t	landing_lights_on;
	dr_t	taxi_light_on;

	dr_t	author;
	dr_t	sim_paused;
} drs;

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

bp_state_t bp;
bp_long_state_t bp_ls;

static bool_t inited = B_FALSE, cam_inited = B_FALSE;
static XPLMFlightLoopID	bp_floop = NULL;

static bool_t read_acf_file_info(void);
static float bp_run(float elapsed, float elapsed2, int counter, void *refcon);
static void bp_complete(void);
static bool_t load_buttons(void);
static void unload_buttons(void);
static int key_sniffer(char inChar, XPLMKeyFlags inFlags, char inVirtualKey,
    void *refcon);
static void tug_pos_update(vect2_t my_pos, double my_hdg, bool_t pos_only);

static bool_t load_icon(button_t *btn);
static void unload_icon(button_t *btn);
static void draw_icon(button_t *btn, int x, int y, double scale,
    bool_t is_clicked, bool_t is_lit);
static void nil_win_key(XPLMWindowID inWindowID, char inKey,
    XPLMKeyFlags inFlags, char inVirtualKey, void *inRefcon, int losingFocus);

static void disco_intf_hide(void);
static int disco_handler(XPLMCommandRef, XPLMCommandPhase, void *);
static int recon_handler(XPLMCommandRef, XPLMCommandPhase, void *);

static bool_t radio_volume_warn = B_FALSE;

static const acf_info_t incompatible_acf[] = {
    { .acf = NULL, .author = NULL }
};

static XPLMCommandRef disco_cmd = NULL, recon_cmd = NULL;
static button_t disco_buttons[2] = {
	{ .filename = "disconnect.png", .vk = -1, .tex = 0, .tex_data = NULL },
	{ .filename = "reconnect.png", .vk = -1, .tex = 0, .tex_data = NULL },
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
	if (drs.pbrake_is_custom)
		return (dr_getf(&drs.pbrake) != 0);
	return (dr_getf(&drs.pbrake) != 0 || dr_getf(&drs.pbrake_rat) != 0);
}

/*
 * Checks if ANY engine of the aircraft is running.
 */
static bool_t
eng_is_running(void)
{
	int num_engns = MIN(dr_geti(&drs.num_engns), 100);
	int engn_running[num_engns];

	dr_getvi(&drs.engn_running, engn_running, 0, num_engns);
	for (int i = 0; i < num_engns; i++) {
		if (engn_running[i] != 0)
			return (B_TRUE);
	}

	return (B_FALSE);
}

/*
 * Returns true if the engines may be started during pushback. Engines may
 * be started IF:
 *	1) there are two or more engines (i.e. they are on the wings and
 *	   won't risk hitting the tug.
 *	2) if there is one engine only, it may be started it if is a jet
 *	   engine. Civillian jet engines generally do not have their intake
 *	   on the nose of the aircraft.
 */
static bool_t
eng_ok2start(void)
{
	dr_t eng_type_dr;
	int eng_type;

	if (dr_geti(&drs.num_engns) > 1)
		return (B_TRUE);
	fdr_find(&eng_type_dr, "sim/aircraft/prop/acf_en_type");
	eng_type = dr_geti(&eng_type_dr);
	/*
	 * From X-Plane's DataRefs.txt, the engine types are:
	 *	0=recip carb		(prop, not OK to start)
	 *	1=recip injected	(prop, not OK to start)
	 *	2=free turbine		(prop, not OK to start)
	 *	3=electric		(prop, not OK to start)
	 *	4=lo bypass jet		(jet, OK to start)
	 *	5=hi bypass jet		(jet, OK to start)
	 *	6=rocket		(don't care, doesn't exist)
	 *	7=tip rockets		(don't care, doesn't exist)
	 *	8=fixed turbine		(prop, not OK to start)
	 */
	return (eng_type >= 4 && eng_type <= 5);
}

/*
 * Determines if an aircraft is likely to be an airliner.
 */
static bool_t
acf_is_airliner(void)
{
	/* For our purposes, airliners don't exist in the light category. */
	enum { AIRLINE_MIN_MTOW = 7000 };
	return (dr_getf(&drs.mtow) >= AIRLINE_MIN_MTOW &&
	    !bp.acf.model_flags.is_experimental &&
	    !bp.acf.model_flags.is_general_aviation &&
	    !bp.acf.model_flags.is_glider &&
	    !bp.acf.model_flags.is_helicopter &&
	    !bp.acf.model_flags.is_military &&
	    !bp.acf.model_flags.is_sci_fi &&
	    !bp.acf.model_flags.is_ultralight &&
	    !bp.acf.model_flags.is_vtol &&
	    !bp.acf.model_flags.fly_like_a_helo);
}

static void
read_acf_airline(char airline[1024])
{
	int n;
	char *p;

	(void) dr_gets(&drs.acf_livery_path, airline, 1024);
	n = strlen(airline);
	/* strip the final directory separator */
	if (n > 0) {
		airline[n - 1] = '\0';
		n--;
	}
	/* strip away any leading path components, leave only the last one */
	if ((p = strrchr(airline, '/')) != NULL) {
		int l;
		p++;
		l = n - (p - airline);
		memmove(airline, p, l + 1);
		n -= l;
	}
	if ((p = strrchr(airline, '\\')) != NULL) {
		int l;
		p++;
		l = n - (p - airline);
		memmove(airline, p, l + 1);
		n -= l;
	}
}

/*
 * On single-engine prop aircraft we must rotate the propeller prior to
 * attaching so that the blades are as far away from the ground as possible,
 * so they don't catch on our tug. Any other aircraft type, we leave alone.
 */
static void
prop_single_adjust(void)
{
	dr_t eng_type_dr, prop_angle_dr, num_blades_dr;
	int eng_type, num_blades;

	if (dr_geti(&drs.num_engns) > 1)
		return;
	fdr_find(&eng_type_dr, "sim/aircraft/prop/acf_en_type");
	eng_type = dr_geti(&eng_type_dr);
	/* See eng_ok2start for engine type designators */
	if (eng_type > 3 && eng_type < 8)
		return;
	fdr_find(&prop_angle_dr,
	    "sim/flightmodel2/engines/prop_rotation_angle_deg");
	fdr_find(&num_blades_dr, "sim/aircraft/prop/acf_num_blades");
	num_blades = dr_geti(&num_blades_dr);
	if (num_blades % 2 == 1) {
		/* odd numbers of blades mean we always go to 0 degrees */
		dr_setf(&prop_angle_dr, 0);
	} else {
		/* even numbers we rotate to put a gap at the bottom */
		dr_setf(&prop_angle_dr, 180 / num_blades);
	}
}

static void
brakes_set(bool_t flag)
{
	/*
	 * Maximum we can set is 0.9. Any more and we might kick the parking
	 * brake off.
	 */
	double val = (flag ? 0.9 : 0.0);
	ASSERT(!slave_mode);
	dr_setf(&drs.lbrake, val);
	dr_setf(&drs.rbrake, val);
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

static bool_t
acf_is_felis_tu154m(void)
{
	char my_acf[512], my_path[512];
	char my_author[512];

	XPLMGetNthAircraftModel(0, my_acf, my_path);
	dr_gets(&drs.author, my_author, sizeof (my_author));
	return (strcmp(my_acf, "tu154.acf") == 0 &&
	    strcmp(my_author, "Felis") == 0);
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
		if (dist < min_dist) {
			strlcpy(icao, arpt->icao, sizeof (arpt->icao));
			min_dist = dist;
		}
	}
	free_nearest_airport_list(list);
	unload_distant_airport_tiles(airportdb, NULL_GEO_POS2);

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

static void
reorient_aircraft(double d_roll, double d_pitch, double d_hdg)
{
	double phi = dr_getf(&drs.roll) + d_roll;
	double phi_mod = DEG2RAD(phi) / 2;
	double sin_phi_mod = sin(phi_mod), cos_phi_mod = cos(phi_mod);
	double theta = dr_getf(&drs.pitch) + d_pitch;
	double theta_mod = DEG2RAD(theta) / 2;
	double sin_theta_mod = sin(theta_mod), cos_theta_mod = cos(theta_mod);
	double psi = dr_getf(&drs.hdg) + d_hdg;
	double psi_mod = DEG2RAD(psi) / 2;
	double sin_psi_mod = sin(psi_mod), cos_psi_mod = cos(psi_mod);
	double q[4];

	q[0] = cos_psi_mod * cos_theta_mod * cos_phi_mod +
	    sin_psi_mod * sin_theta_mod * sin_phi_mod;
	q[1] = cos_psi_mod * cos_theta_mod * sin_phi_mod -
	    sin_psi_mod * sin_theta_mod * cos_phi_mod;
	q[2] = cos_psi_mod * sin_theta_mod * cos_phi_mod +
	    sin_psi_mod * cos_theta_mod * sin_phi_mod;
	q[3] = -cos_psi_mod * sin_theta_mod * sin_phi_mod +
	    sin_psi_mod * cos_theta_mod * cos_phi_mod;

	dr_setvf(&drs.quaternion, q, 0, 4);
}

/*
 * Computes the distance from the tug's fixed steering (rear) axle
 * to the aircraft's nosewheel.
 */
static double
tug_rear2acf_nw(void)
{
	double nlg_tug_z_off;
	switch (bp_ls.tug->info->lift_wall_loc) {
	case LIFT_WALL_FRONT:
		nlg_tug_z_off = bp_ls.tug->info->lift_wall_z - bp.acf.tirrad;
		break;
	case LIFT_WALL_CENTER:
		nlg_tug_z_off = bp_ls.tug->info->lift_wall_z;
		break;
	default:
		ASSERT3U(bp_ls.tug->info->lift_wall_loc, ==, LIFT_WALL_BACK);
		nlg_tug_z_off = bp_ls.tug->info->lift_wall_z + bp.acf.tirrad;
		break;
	}
	return (nlg_tug_z_off - bp_ls.tug->veh.fixed_z_off);
}

static void
turn_nosewheel(double req_steer)
{
	int dir_mult = (bp_ls.tug->pos.spd >= 0 ? 1 : -1);
	double cur_nw_steer, tug_turn_r, tug_turn_rate, rel_tug_turn_rate;
	double d_steer, nlg_tug_rear_off, d_hdg, turn_inc;
	vect2_t off_v;

	cur_nw_steer = rel_hdg(bp.cur_pos.hdg, bp_ls.tug->pos.hdg);

	/* limit the steering request to what we can actually do */
	req_steer = MIN(req_steer, bp.veh.max_steer);
	req_steer = MAX(req_steer, -bp.veh.max_steer);

	if (ABS(bp_ls.tug->cur_steer) > 0.01) {
		tug_turn_r = (1 / tan(DEG2RAD(bp_ls.tug->cur_steer))) *
		    bp_ls.tug->veh.wheelbase;
	} else {
		tug_turn_r = 1e10;
	}
	tug_turn_rate = (bp_ls.tug->pos.spd / (2 * M_PI * tug_turn_r)) * 360;
	rel_tug_turn_rate = tug_turn_rate - bp.d_pos.hdg / bp.d_t;

	cur_nw_steer += rel_tug_turn_rate * bp.d_t;
	cur_nw_steer = MIN(cur_nw_steer, 85);
	cur_nw_steer = MAX(cur_nw_steer, -85);
	d_steer = req_steer - cur_nw_steer;

	if (ABS(bp_ls.tug->pos.spd) > 0.01) {
		/*
		 * Limit steering of the tug at high speeds to prevent the
		 * tug swinging like crazy around.
		 */
		double tug_steer = dir_mult * 3 * d_steer;
		double speed;

		tug_steer = MIN(MAX(tug_steer, -bp_ls.tug->veh.max_steer),
		    bp_ls.tug->veh.max_steer);
		speed = ang_vel_speed_limit(&bp_ls.tug->veh, tug_steer,
		    bp_ls.tug->pos.spd);
		if (speed < bp_ls.tug->pos.spd)
			tug_steer *= speed / bp_ls.tug->pos.spd;
		tug_set_steering(bp_ls.tug, tug_steer, bp.d_t);
	}

	dr_setvf(&drs.tire_steer_cmd, &cur_nw_steer, bp.acf.nw_i, 1);

	/*
	 * Since the nosewheel always isn't exactly over the tug's fixed
	 * steering axle, we need to manually shift the aircraft's heading,
	 * so as appear as if it steering around the tug's fixed steering
	 * axle. We do so by calculating an incremental lateral displacement
	 * from the aircraft's point of view.
	 * nlg_tug_z_off: is the long offset along the tug's axis of the
	 *	centerpoint of the aircraft's nose landing gear.
	 * nlg_tug_rear_off: is the long offset along the tug's axis of
	 *	the center of the aircraft's nose landing gear relative to
	 *	where the fixed steering axle is located. This forms a
	 *	similar triangle to the triangle being formed when the tug's
	 *	steering turns.
	 * We compute the lateral steering displacement of the tug, apply
	 * a sin() function to reduce it based on how far the nosewheel is
	 * deflected (obviously we don't want any deflection at near 90
	 * degrees) and scale the similar triangles. The result is an
	 * absolute lateral displacement that the nosewheel should
	 * experience from the aircraft's point of view. We then translate
	 * that into a heading change and write that to the orientation
	 * quaternion, overriding the aircraft's heading.
	 */
	nlg_tug_rear_off = tug_rear2acf_nw();
	turn_inc = rel_tug_turn_rate * bp.d_t;

	/*
	 * We compute the lateral & longitudinal displacement in the
	 * tug's coordinates. We then rotate this vector to the aircraft's
	 * vector and apply the x component to the aircraft's heading.
	 */
	off_v.x = sin(DEG2RAD(turn_inc)) * (nlg_tug_rear_off /
	    bp_ls.tug->veh.wheelbase);
	off_v.y = (cos(DEG2RAD(turn_inc)) - 1) * (nlg_tug_rear_off /
	    bp_ls.tug->veh.wheelbase);
	off_v = vect2_rot(off_v, cur_nw_steer);
	d_hdg = RAD2DEG(asin(off_v.x / bp.veh.wheelbase));
	/*
	 * For some inexplicable reason, we have to amplify the heading change
	 * by around 10x to get it to show properly in the sim. Probably
	 * something to do with ground stickiness or heading change
	 * granularity/float rounding errors. Definitely file under "WTF".
	 */
	reorient_aircraft(0, 0, 10 * d_hdg);
}

static void
push_at_speed(double targ_speed, double max_accel, bool_t allow_snd_ctl,
    bool_t decelerating)
{
	double force_lim, force_incr, force, accel_now, d_v, Fx, Fz, steer;
	double cur_spd;

	VERIFY3S(dr_getvf(&drs.tire_steer_cmd, &steer, bp.acf.nw_i, 1), ==, 1);

	/*
	 * Primary speed control is in drive_on_line in driving.c, but that
	 * doesn't take into account the steering delay, only the idealized
	 * steering state it WANTS us to be at. So here we further limit
	 * speed to the actual steering angle that we are at right now.
	 */
	targ_speed = ang_vel_speed_limit(&bp.veh, steer, targ_speed);
	/* Also try to take the tug's angular velocity limits into account. */
	targ_speed = ang_vel_speed_limit(&bp_ls.tug->veh, bp_ls.tug->cur_steer,
	    targ_speed);

	/*
	 * Multiply force limit by weight in tons - that's at most how
	 * hard we'll try to push the aircraft. This prevents us from
	 * flinging the aircraft across the tarmac in case some external
	 * factor is blocking us (like chocks).
	 */
	force_lim = FORCE_PER_TON * (dr_getf(&drs.acf_mass) / 1000);

	/*
	 * Scale the maximum force increment by frame time. This means it'll
	 * take up to 1s for us to apply full pushback force.
	 */
	force_incr = force_lim * bp.d_t;

	/*
	 * We actually control ground speed to be the speed of the tug rather
	 * than the longitudinal speed of the aircraft. So scale the
	 * longitudinal speed based on nosewheel steering angle.
	 */
	if (bp_xp_ver >= 11000) {
		cur_spd = bp.cur_pos.spd / cos(DEG2RAD(fabs(steer)) / 2);
		accel_now = (bp.d_pos.spd / cos(DEG2RAD(fabs(steer)))) / bp.d_t;
	} else {
		/*
		 * XP10's buggy sticky tire model prevents us from reducing
		 * longitudinal speed below MIN_SPEED_XP10, so make sure we
		 * keep the speed up above that value.
		 */
		cur_spd = bp.cur_pos.spd;
		accel_now = bp.d_pos.spd / bp.d_t;
	}

	force = bp.last_force;
	d_v = targ_speed - cur_spd;

	/*
	 * This is some fudge needed to get some high-thrust aircraft
	 * going, otherwise we'll just jitter in-place due to thinking
	 * we're overdoing acceleration.
	 */
	if (ABS(cur_spd) < BREAKAWAY_THRESH)
		max_accel *= 100;

	if (d_v > 0) {		/* speed up */
		/*
		 * Modulate the acceleration to reach our target speed smoothly,
		 * unless we're trying to decelerate or we've not yet broken
		 * away (to prevent jumpiness on XP10's sticky tires).
		 */
		if (d_v < max_accel && !decelerating &&
		    ABS(bp.cur_pos.spd) >= BREAKAWAY_THRESH)
			max_accel = d_v;
		if (accel_now > max_accel)
			force -= force_incr;
		else if (accel_now < max_accel)
			force += force_incr;
	} else if (d_v < 0) {	/* slow down */
		max_accel *= -1;
		if (d_v > max_accel && !decelerating &&
		    ABS(bp.cur_pos.spd) >= BREAKAWAY_THRESH)
			max_accel = d_v;
		if (accel_now < max_accel)
			force += force_incr;
		else if (accel_now > max_accel)
			force -= force_incr;
	}

	/*
	 * Calculate the vector components of our force on the aircraft
	 * to correctly apply angular momentum forces below.
	 * N.B. we only push in the horizontal plane, hence no Fy component.
	 */
	Fx = force * sin(DEG2RAD(steer));
	Fz = force * cos(DEG2RAD(steer));

	dr_setf(&drs.axial_force, dr_getf(&drs.axial_force) - Fz);
	dr_setf(&drs.rot_force_N, dr_getf(&drs.rot_force_N) +
	    Fx * (-bp.acf.nw_z));

	/* Don't overstep the force limits for this aircraft */
	force = MIN(force_lim, force);
	force = MAX(-force_lim, force);

	bp.last_force = force;

	if (allow_snd_ctl) {
		tug_set_TE_override(bp_ls.tug, B_TRUE);
		if ((bp.cur_pos.spd > 0 && force > 0) ||
		    (bp.cur_pos.spd < 0 && force < 0)) {
			double spd_fract = (ABS(bp.cur_pos.spd) /
			    bp_ls.tug->info->max_fwd_speed);
			double force_fract = fabs(force /
			    bp_ls.tug->info->max_TE);
			tug_set_TE_snd(bp_ls.tug, AVG(force_fract, spd_fract),
			    bp.d_t);
		} else {
			tug_set_TE_snd(bp_ls.tug, 0, bp.d_t);
		}
	}
}

static bool_t
read_gear_info(void)
{
	double tire_z[10];
	int gear_steers[10], gear_types[10], gear_on_ground[10];
	int gear_deploy[10];

	dr_getvi(&drs.gear_deploy, gear_deploy, 0, 10);
	if (bp_xp_ver >= 11000)
		dr_getvi(&drs.gear_on_ground, gear_on_ground, 0, 10);
	else
		memset(gear_on_ground, 0xff, sizeof (gear_on_ground));

	/* First determine where the gears are */
	for (int i = 0, n = dr_getvi(&drs.gear_types, gear_types, 0, 10);
	    i < n; i++) {
		/*
		 * Gear types are:
		 * 0) Nothing.
		 * 1) Skid.
		 * 2+) Wheel based gear in various arrangements. A tug can
		 *	provide a filter for this.
		 *
		 * Also make sure to ONLY select gears which are deployed and
		 * are touching the ground. Some aircraft models have weird
		 * gears which are, for whatever reason, hovering in mid air
		 * (huh?).
		 */
		if (gear_types[i] >= 2 && gear_on_ground[i] != 0 &&
		    gear_deploy[i] != 0)
			bp.acf.gear_is[bp.acf.n_gear++] = i;
	}

	/* Read nosegear long axis deflections */
	VERIFY3S(dr_getvf(&drs.tire_z, tire_z, 0, 10), >=, bp.acf.n_gear);
	bp.acf.nw_i = -1;
	bp.acf.nw_z = 1e10;

	/* Next determine which gear steers. Pick the one most forward. */
	VERIFY3S(dr_getvi(&drs.gear_steers, gear_steers, 0, 10), >=,
	    bp.acf.n_gear);
	for (int i = 0; i < bp.acf.n_gear; i++) {
		if (gear_steers[bp.acf.gear_is[i]] == 1 &&
		    tire_z[bp.acf.gear_is[i]] < bp.acf.nw_z) {
			bp.acf.nw_i = bp.acf.gear_is[i];
			bp.acf.nw_z = tire_z[bp.acf.gear_is[i]];
		}
	}
	/*
	 * The Tu-154M needs some special care here, because it overrides
	 * gear_steers to 0 until the hydraulics are powered. We don't want
	 * to refuse to work in that case, so just hard-set the nw_i to 0.
	 */
	if (acf_is_felis_tu154m()) {
		bp.acf.nw_i = bp.acf.gear_is[0];
		bp.acf.nw_z = tire_z[bp.acf.gear_is[0]];
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
	for (int i = 0; i < bp.acf.n_gear; i++) {
		if (bp.acf.gear_is[i] != bp.acf.nw_i)
			bp.acf.main_z += tire_z[bp.acf.gear_is[i]];
	}
	bp.acf.main_z /= bp.acf.n_gear - 1;

	return (B_TRUE);
}

static bool_t
bp_state_init(void)
{
	memset(&bp, 0, sizeof (bp));
	list_create(&bp.segs, sizeof (seg_t), offsetof(seg_t, node));

	if (bp_xp_ver < MIN_XPLANE_VERSION) {
		char msg[256];
		snprintf(msg, sizeof (msg), _("Pushback failure: X-Plane "
		    "version too old. This plugin requires at least X-Plane "
		    "%s to operate."), MIN_XPLANE_VERSION_STR);
		XPLMSpeakString(msg);
		return (B_FALSE);
	}

	if (!read_acf_file_info()) {
		XPLMSpeakString(_("Pushback failure: error reading aircraft "
		    "files from disk."));
		return (B_FALSE);
	}
	if (bp.acf.model_flags.is_helicopter ||
	    bp.acf.model_flags.fly_like_a_helo) {
		XPLMSpeakString(_("Pushback failure: Are you seriously "
		    "trying to call pushback for a helicopter?"));
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
		bp.veh.max_steer = (MAX_STEER_ANGLE + MIN_STEER_ANGLE) / 2;
	bp.veh.max_fwd_spd = MAX_FWD_SPEED;
	bp.veh.max_rev_spd = MAX_REV_SPEED;
	bp.veh.max_fwd_ang_vel = MAX_FWD_ANG_VEL;
	bp.veh.max_rev_ang_vel = MAX_REV_ANG_VEL;
	bp.veh.max_accel = NORMAL_ACCEL;
	bp.veh.max_decel = NORMAL_DECEL;
	/*
	 * To achieve more accurate pushback results, we use our rear axle
	 * position to actually direct the pushback, not our aircraft's
	 * origin point.
	 */
	bp.veh.use_rear_pos = B_TRUE;

	bp.step = PB_STEP_OFF;
	bp.step_start_t = 0;

	return (B_TRUE);
}

static bool_t
audio_sys_init(void)
{
	lang_pref_t	lang_pref = LANG_PREF_MATCH_REAL;
	char		icao[8];

	find_nearest_airport(icao);
	(void) conf_get_i(bp_conf, "lang_pref", (int *)&lang_pref);
	if (!msg_init(bp_get_lang(), icao, lang_pref)) {
		XPLMSpeakString(_("Pushback failure: error initialising audio "
		    "messages. Please reinstall BetterPushback."));
		return (B_FALSE);
	}

	return (B_TRUE);
}

static bool_t
acf_on_gnd_stopped(const char **reason)
{
	if (dr_geti(&drs.onground_any) != 1) {
		if (reason != NULL)
			*reason = _("Pushback failure: aircraft not on ground.");
		return (B_FALSE);
	}
	if (vect3_abs(VECT3(dr_getf(&drs.vx), dr_getf(&drs.vy),
	    dr_getf(&drs.vz))) >= 1) {
		if (reason != NULL) {
			*reason = _("Pushback failure: aircraft not "
			    "stationary.");
		}
		return (B_FALSE);
	}
	if (dr_getf(&drs.gear_deploy) != 1) {
		if (reason != NULL)
			*reason = _("Pushback failure: gear not extended.");
		return (B_FALSE);
	}
	return (B_TRUE);
}

/*
 * Normally, we delay calling bp_init and bp_fini until the plugin is actually
 * needed. This can mess with 3rd party plugin integration which might need to
 * look for things such as commands we create much earlier. To solve this, we
 * have bp_boot_init and bp_shut_fini, which are called from XPluginStart and
 * XPluginStop.
 */
void
bp_boot_init(void)
{
	disco_cmd = XPLMCreateCommand("BetterPushback/disconnect",
	    _("Disconnect tow + headset and switch to hand signals."));
	recon_cmd = XPLMCreateCommand("BetterPushback/reconnect",
	    _("Reconnect tow and await further instructions."));
}

void
bp_shut_fini(void)
{
}

/*
 * Reads the aircraft's .acf file and grabs the info we want from it.
 */
static bool_t
read_acf_file_info(void)
{
	char my_acf[512], my_path[512];
	FILE *fp;
	char *line = NULL;
	size_t cap = 0;
	bool_t parsing_props = B_FALSE;

	XPLMGetNthAircraftModel(0, my_acf, my_path);
	fp = fopen(my_path, "rb");
	if (fp == NULL) {
		logMsg("Error reading %s: %s", my_acf, strerror(errno));
		return (B_FALSE);
	}

#define	PARSE_FLAG_PARAM(flag) \
	do { \
		size_t n; \
		char **comps = strsplit(line, " ", B_TRUE, &n); \
		if (n != 3) { \
			free_strlist(comps, n); \
			continue; \
		} \
		sscanf(comps[2], "%d", &bp.acf.model_flags.flag); \
		free_strlist(comps, n); \
	} while (0)

	while (getline(&line, &cap, fp) > 0) {
		strip_space(line);
		if (!parsing_props) {
			if (strcmp(line, "PROPERTIES_BEGIN") == 0)
				parsing_props = B_TRUE;
			continue;
		}
		if (strcmp(line, "PROPERTIES_END") == 0)
			break;
		if (strstr(line, "acf/_is_airliner") != NULL)
			PARSE_FLAG_PARAM(is_airliner);
		else if (strstr(line, "acf/_is_experimental") != NULL)
			PARSE_FLAG_PARAM(is_experimental);
		else if (strstr(line, "acf/_is_general_aviation") != NULL)
			PARSE_FLAG_PARAM(is_general_aviation);
		else if (strstr(line, "acf/_is_glider") != NULL)
			PARSE_FLAG_PARAM(is_glider);
		else if (strstr(line, "acf/_is_helicopter") != NULL)
			PARSE_FLAG_PARAM(is_helicopter);
		else if (strstr(line, "acf/_is_military") != NULL)
			PARSE_FLAG_PARAM(is_military);
		else if (strstr(line, "acf/_is_sci_fi") != NULL)
			PARSE_FLAG_PARAM(is_sci_fi);
		else if (strstr(line, "acf/_is_seaplane") != NULL)
			PARSE_FLAG_PARAM(is_seaplane);
		else if (strstr(line, "acf/_is_ultralight") != NULL)
			PARSE_FLAG_PARAM(is_ultralight);
		else if (strstr(line, "acf/_is_vtol") != NULL)
			PARSE_FLAG_PARAM(is_vtol);
		else if (strstr(line, "acf/_fly_like_a_helo") != NULL)
			PARSE_FLAG_PARAM(fly_like_a_helo);
	}

#undef	PARSE_FLAG_PARAM

	fclose(fp);

	return (B_TRUE);
}

bool_t
bp_init(void)
{
	const char *reason;
	dr_t radio_vol, sound_on;
	char my_acf[512], my_path[512];

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
		drs.pbrake_is_custom = B_FALSE;
	} else {
		drs.pbrake_is_custom = B_TRUE;
	}
	fdr_find(&drs.pbrake_rat, "sim/cockpit2/controls/parking_brake_ratio");
	fdr_find(&drs.rot_force_N, "sim/flightmodel/forces/N_plug_acf");
	fdr_find(&drs.axial_force, "sim/flightmodel/forces/faxil_plug_acf");
	fdr_find(&drs.override_planepath,
	    "sim/operation/override/override_planepath");
	fdr_find(&drs.local_x, "sim/flightmodel/position/local_x");
	fdr_find(&drs.local_y, "sim/flightmodel/position/local_y");
	fdr_find(&drs.local_z, "sim/flightmodel/position/local_z");
	fdr_find(&drs.local_vx, "sim/flightmodel/position/local_vx");
	fdr_find(&drs.local_vy, "sim/flightmodel/position/local_vy");
	fdr_find(&drs.local_vz, "sim/flightmodel/position/local_vz");
	fdr_find(&drs.lat, "sim/flightmodel/position/latitude");
	fdr_find(&drs.lon, "sim/flightmodel/position/longitude");
	fdr_find(&drs.roll, "sim/flightmodel/position/phi");
	fdr_find(&drs.pitch, "sim/flightmodel/position/theta");
	fdr_find(&drs.hdg, "sim/flightmodel/position/psi");
	fdr_find(&drs.quaternion, "sim/flightmodel/position/q");
	fdr_find(&drs.vx, "sim/flightmodel/position/local_vx");
	fdr_find(&drs.vy, "sim/flightmodel/position/local_vy");
	fdr_find(&drs.vz, "sim/flightmodel/position/local_vz");
	fdr_find(&drs.sim_time, "sim/time/total_running_time_sec");
	fdr_find(&drs.acf_mass, "sim/flightmodel/weight/m_total");
	fdr_find(&drs.tire_z, "sim/flightmodel/parts/tire_z_no_deflection");
	fdr_find(&drs.tire_x, "sim/flightmodel/parts/tire_x_no_deflection");
	fdr_find(&drs.mtow, "sim/aircraft/weight/acf_m_max");
	fdr_find(&drs.leg_len, "sim/aircraft/parts/acf_gear_leglen");
	fdr_find(&drs.tirrad, "sim/aircraft/parts/acf_gear_tirrad");
	fdr_find(&drs.nw_steerdeg1, "sim/aircraft/gear/acf_nw_steerdeg1");
	fdr_find(&drs.nw_steerdeg2, "sim/aircraft/gear/acf_nw_steerdeg2");
	fdr_find(&drs.tire_steer_cmd,
	    "sim/flightmodel/parts/tire_steer_cmd");
	fdr_find(&drs.override_steer,
	    "sim/operation/override/override_wheel_steer");
	fdr_find(&drs.nw_steer_on, "sim/cockpit2/controls/nosewheel_steer_on");
	fdr_find(&drs.gear_types, "sim/aircraft/parts/acf_gear_type");
	if (bp_xp_ver >= 11000) {
		fdr_find(&drs.gear_on_ground,
		    "sim/flightmodel2/gear/on_ground");
	}
	fdr_find(&drs.onground_any, "sim/flightmodel/failures/onground_any");
	fdr_find(&drs.gear_steers, "sim/aircraft/overflow/acf_gear_steers");
	fdr_find(&drs.gear_deploy, "sim/aircraft/parts/acf_gear_deploy");
	fdr_find(&drs.num_engns, "sim/aircraft/engine/acf_num_engines");
	fdr_find(&drs.engn_running, "sim/flightmodel/engine/ENGN_running");
	fdr_find(&drs.acf_livery_path, "sim/aircraft/view/acf_livery_path");

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

	fdr_find(&drs.landing_lights_on,
	    "sim/cockpit/electrical/landing_lights_on");
	fdr_find(&drs.taxi_light_on, "sim/cockpit/electrical/taxi_light_on");

	fdr_find(&drs.author, "sim/aircraft/view/acf_author");
	fdr_find(&drs.sim_paused, "sim/time/paused");

	XPLMRegisterCommandHandler(disco_cmd, disco_handler, 1, NULL);
	XPLMRegisterCommandHandler(recon_cmd, recon_handler, 1, NULL);

	/*
	 * We do this check before attempting to read gear info, because
	 * in-flight the gear info check will fail with "non-steerable"
	 * gears, which is a little cryptic to understand to the user.
	 */
	if (!acf_on_gnd_stopped(&reason))
		goto errout;

	if (!bp_state_init())
		goto errout;
	if (!audio_sys_init() || !load_buttons() ||
	    !load_icon(&disco_buttons[0]) || !load_icon(&disco_buttons[1]))
		goto errout;

	XPLMGetNthAircraftModel(0, my_acf, my_path);
	bp_ls.outline = acf_outline_read(my_path);
	if (bp_ls.outline == NULL)
		goto errout;

	inited = B_TRUE;

	return (B_TRUE);
errout:
	XPLMUnregisterCommandHandler(disco_cmd, disco_handler, 1, NULL);
	XPLMUnregisterCommandHandler(recon_cmd, recon_handler, 1, NULL);
	msg_fini();
	unload_buttons();
	unload_icon(&disco_buttons[0]);
	unload_icon(&disco_buttons[1]);
	if (bp_ls.outline != NULL) {
		acf_outline_free(bp_ls.outline);
		bp_ls.outline = NULL;
	}
	return (B_FALSE);
}

static int
draw_tugs(XPLMDrawingPhase phase, int before, void *refcon)
{
	UNUSED(phase);
	UNUSED(before);
	UNUSED(refcon);

	if (bp_ls.tug == NULL) {
		/*
		 * If we have no tug loaded, we must either be in the
		 * tug-selection phase, or be slaved to a master instance
		 * which has not yet notified us which tug to use.
		 */
		ASSERT(bp.step <= PB_STEP_TUG_LOAD || slave_mode);
		return (1);
	}

	if (list_head(&bp_ls.tug->segs) == NULL &&
	    bp.step >= PB_STEP_GRABBING &&
	    bp.step <= PB_STEP_UNGRABBING) {
		vect2_t my_pos = VECT2(dr_getf(&drs.local_x),
		    -dr_getf(&drs.local_z));
		double my_hdg = dr_getf(&drs.hdg);
		tug_pos_update(my_pos, my_hdg, B_TRUE);
	}

	tug_draw(bp_ls.tug, bp.cur_t);

	return (1);
}

bool_t
bp_can_start(const char **reason)
{
	seg_t *seg;

	if (!acf_is_compatible()) {
		if (reason != NULL)
			*reason = _("Pushback failure: aircraft is not "
			    "compatible with BetterPushback.");
		return (B_FALSE);
	}

	if (!acf_on_gnd_stopped(reason))
		return (B_FALSE);

	if (!eng_ok2start() && eng_is_running()) {
		if (reason != NULL) {
			*reason = _("Pushback failure: cannot push this "
			    "aircraft with engines running. Shutdown "
			    "engines first.");
		}
		return (B_FALSE);
	}

	seg = list_head(&bp.segs);
	if (seg == NULL && !late_plan_requested && !slave_mode) {
		if (reason != NULL) {
			*reason = _("Pushback failure: please first plan your "
			    "pushback to tell me where you want to go.");
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
	const char *reason;
	XPLMCreateFlightLoop_t floop = {
	    .structSize = sizeof (XPLMCreateFlightLoop_t),
	    .phase = xplm_FlightLoop_Phase_BeforeFlightModel,
	    .callbackFunc = bp_run,
	    .refcon = NULL
	};

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

	if (bp_floop == NULL)
		bp_floop = XPLMCreateFlightLoop(&floop);
	XPLMScheduleFlightLoop(bp_floop, -1, 1);

	XPLMRegisterDrawCallback(draw_tugs, TUG_DRAWING_PHASE,
	    TUG_DRAWING_PHASE_BEFORE, NULL);

	if (!slave_mode && !late_plan_requested)
		route_save(&bp.segs);

	bp_started = B_TRUE;
	bp_conf_set_save_enabled(!bp_started);

	/*
	 * Some aircraft (like the MD-80) do not have a taxi light switch,
	 * so if the previously loaded aircraft had taxi lights on, the
	 * dataref could left set to '1' with the pilot having no way of
	 * switching the lights off. So we manually make sure the lights
	 * are off here. This way we can be sure that if we see the light
	 * on during pushback, it was the pilot who turned it on.
	 */
	dr_seti(&drs.landing_lights_on, 0);
	dr_seti(&drs.taxi_light_on, 0);

	return (B_TRUE);
}

bool_t
bp_stop(void)
{
	seg_t *seg;

	if (!bp_started)
		return (B_FALSE);

	/* prevent trying to reach segment end hdg and apply correct back */
	bp.last_hdg = NAN;
	if ((seg = list_tail(&bp.segs)) != NULL)
		bp.last_seg_is_back = seg->backward;
	bp_delete_all_segs();
	late_plan_requested = B_FALSE;

	return (B_TRUE);
}

void
bp_fini(void)
{
	if (!inited)
		return;

	if (bp_ls.outline != NULL) {
		acf_outline_free(bp_ls.outline);
		bp_ls.outline = NULL;
	}

	if (bp_floop != NULL) {
		XPLMDestroyFlightLoop(bp_floop);
		bp_floop = NULL;
	}

	XPLMUnregisterCommandHandler(disco_cmd, disco_handler, 1, NULL);
	XPLMUnregisterCommandHandler(recon_cmd, recon_handler, 1, NULL);

	msg_fini();
	bp_complete();

	/* segs have been released in bp_complete */
	list_destroy(&bp.segs);

	unload_icon(&disco_buttons[0]);
	unload_icon(&disco_buttons[1]);
	unload_buttons();

	radio_volume_warn = B_FALSE;

	inited = B_FALSE;
}

static bool_t
nearing_end(void)
{
	double long_displ;
	seg_t *seg = list_head(&bp.segs);
	vect2_t end_dir, end2acf;

	if (seg->type != SEG_TYPE_STRAIGHT || seg != list_tail(&bp.segs))
		return (B_FALSE);

	end_dir = hdg2dir(seg->end_hdg);
	if (seg->backward)
		end_dir = vect2_neg(end_dir);
	end2acf = vect2_sub(bp.cur_pos.pos, seg->end_pos);
	long_displ = vect2_dotprod(end_dir, end2acf);
	return (long_displ > -NEARING_END_THRESHOLD);
}

/*
 * We need to compute a fake position for drive_segs. This is because when
 * steering, we don't actually perform simple steering around our nosewheel.
 * Instead, the nosewheel swings by being articulated with the tug service
 * as the platform. So instead of simply passing our true position to
 * drive_segs, we pretend that our centerline actually passes through the
 * tug's fixed steering axle.
 */
static vehicle_pos_t
corr_acf_pos(void)
{
	vect2_t dir = hdg2dir(bp.cur_pos.hdg);
	vect2_t main_pos = vect2_add(bp.cur_pos.pos,
	    vect2_scmul(dir, -bp.acf.main_z));
	vect2_t nw_pos = vect2_add(bp.cur_pos.pos,
	    vect2_scmul(dir, -bp.acf.nw_z));
	double tug_rear2acf_nw_l = tug_rear2acf_nw();
	double steer, corr_hdg;
	vect2_t tug_rear_pos, corr_dir, corr_pos;

	VERIFY3S(dr_getvf(&drs.tire_steer_cmd, &steer, bp.acf.nw_i, 1), ==, 1);
	tug_rear_pos = vect2_add(nw_pos, vect2_scmul(hdg2dir(normalize_hdg(
	    bp.cur_pos.hdg + steer + 180)), tug_rear2acf_nw_l));
	corr_dir = vect2_sub(tug_rear_pos, main_pos);
	corr_pos = vect2_add(main_pos, vect2_set_abs(corr_dir, bp.acf.main_z));
	corr_hdg = dir2hdg(corr_dir);

	return ((vehicle_pos_t){corr_pos, corr_hdg, bp.cur_pos.spd});
}

static bool_t
bp_run_push(void)
{
	seg_t *seg = list_head(&bp.segs);
	/*
	 * We memorize the direction of this segment in case we flip segments
	 * and the next one goes in the opposite direction.
	 */
	bool_t last_backward = (seg != NULL ? seg->backward : B_FALSE);

	while (seg != NULL) {
		double steer, speed;
		bool_t decel;
		vehicle_pos_t corr_pos;

		/* Pilot pressed brake pedals or set parking brake, stop */
		if (dr_getf(&drs.lbrake) >= BRAKE_PEDAL_THRESH ||
		    dr_getf(&drs.rbrake) >= BRAKE_PEDAL_THRESH ||
		    pbrake_is_set()) {
			tug_set_TE_snd(bp_ls.tug, 0, bp.d_t);
			dr_setf(&drs.axial_force, 0);
			dr_setf(&drs.rot_force_N, 0);
			bp.last_force = 0;
			break;
		}
		/*
		 * If we have reversed direction, wait a little to simulate
		 * the driver changing gear and flipping around.
		 */
		if (bp.reverse_t != 0.0) {
			if (bp.cur_t - bp.reverse_t < 2 * STATE_TRANS_DELAY) {
				push_at_speed(0, bp.veh.max_accel, B_TRUE,
				    B_FALSE);
				break;
			}
			bp.reverse_t = 0.0;
		}
		corr_pos = corr_acf_pos();
		if (drive_segs(&corr_pos, &bp.veh, &bp.segs,
		    &bp.last_mis_hdg, bp.d_t, &steer, &speed, &decel)) {
			if (!nearing_end()) {
				turn_nosewheel(steer);
			} else {
				/*
				 * When nearing the end of the route, we want
				 * to start neutralizing steering early to not
				 * overshoot too far.
				 */
				turn_nosewheel(0);
			}
			push_at_speed(speed, bp.veh.max_accel, B_TRUE, decel);
			break;
		}
		seg = list_head(&bp.segs);
		if (seg != NULL && seg->backward != last_backward) {
			bp.reverse_t = bp.cur_t;
			last_backward = seg->backward;
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
	/*
	 * Needs to go before the bp_started check in case the planner has
	 * placed segments, but user has not yet started pushback.
	 */
	bp_delete_all_segs();

	if (!bp_started)
		return;

	bp_started = B_FALSE;
	bp_conf_set_save_enabled(!bp_started);
	late_plan_requested = B_FALSE;
	plan_complete = B_FALSE;

	if (bp_ls.tug != NULL) {
		tug_free(bp_ls.tug);
		bp_ls.tug = NULL;
	}

	disco_intf_hide();

	XPLMUnregisterDrawCallback(draw_tugs, TUG_DRAWING_PHASE,
	    TUG_DRAWING_PHASE_BEFORE, NULL);

	if (!slave_mode) {
		dr_seti(&drs.override_steer, 0);
		brakes_set(B_FALSE);
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
		char airline[1024] = { 0 };

		(void) find_nearest_airport(icao);
		if (acf_is_airliner())
			read_acf_airline(airline);
		bp_ls.tug = tug_alloc_auto(dr_getf(&drs.mtow),
		    dr_getf(&drs.leg_len), bp.acf.tirrad,
		    bp.acf.nw_type, strcmp(icao, "") != 0 ? icao : NULL,
		    airline);
		if (bp_ls.tug == NULL) {
			XPLMSpeakString(_("Pushback failure: no suitable "
			    "tug for your aircraft."));
			bp_complete();
			return (B_FALSE);
		}
		strlcpy(bp_tug_name, bp_ls.tug->info->tug_name,
		    sizeof (bp_tug_name));
	} else {
		char tug_name[sizeof (bp_tug_name)];
		char *ext;
		char icao[8] = { 0 };
		char airline[1024] = { 0 };

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

		if (acf_is_airliner())
			read_acf_airline(airline);
		bp_ls.tug = tug_alloc_man(tug_name, bp.acf.tirrad, icao,
		    airline);
		if (bp_ls.tug == NULL) {
			char msg[256];
			snprintf(msg, sizeof (msg), _("Pushback failure: "
			    "master requested tug \"%s\", which we don't have "
			    "in our in our library. Please sync your tug "
			    "libraries before trying again."), tug_name);
			logMsg("%s", msg);
			XPLMSpeakString(msg);
			bp_complete();
			return (B_FALSE);
		}
	}
	if (!bp_ls.tug->info->drive_debug) {
		vect2_t p_start, dir;
		dir = hdg2dir(bp.cur_pos.hdg);
		p_start = vect2_add(bp.cur_pos.pos, vect2_scmul(dir,
		    -bp.acf.nw_z + TUG_APPCH_LONG_DIST));
		p_start = vect2_add(p_start, vect2_scmul(vect2_norm(dir,
		    B_TRUE), 10 * bp_ls.tug->veh.wheelbase));
		tug_set_pos(bp_ls.tug, p_start, normalize_hdg(bp.cur_pos.hdg -
		    90), bp_ls.tug->veh.max_fwd_spd);
	} else {
		tug_set_pos(bp_ls.tug, bp.cur_pos.pos, bp.cur_pos.hdg, 0);
	}
	bp.step++;
	bp.step_start_t = bp.cur_t;

	return (B_TRUE);
}

static void
pb_step_start(void)
{
	if (!bp_ls.tug->info->drive_debug) {
		vect2_t left_off, p_end, dir;

		dir = hdg2dir(bp.cur_pos.hdg);

		left_off = vect2_add(bp.cur_pos.pos, vect2_scmul(dir,
		    -bp.acf.nw_z + TUG_APPCH_LONG_DIST));
		left_off = vect2_add(left_off, vect2_scmul(
		    vect2_norm(dir, B_FALSE), 2 * bp_ls.tug->veh.wheelbase));
		p_end = vect2_add(bp.cur_pos.pos, vect2_scmul(dir,
		    (-bp.acf.nw_z) + bp_ls.tug->info->apch_dist));

		VERIFY(tug_drive2point(bp_ls.tug, left_off,
		    normalize_hdg(bp.cur_pos.hdg - 90)));
		VERIFY(tug_drive2point(bp_ls.tug, p_end, bp.cur_pos.hdg));
	} else {
		for (seg_t *seg = list_head(&bp.segs); seg != NULL;
		    seg = list_next(&bp.segs, seg)) {
			seg_t *seg2 = calloc(1, sizeof (*seg2));
			memcpy(seg2, seg, sizeof (*seg2));
			list_insert_tail(&bp_ls.tug->segs, seg2);
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
	if (!tug_is_stopped(bp_ls.tug)) {
		/*
		 * Keep resetting the start time to enforce the state
		 * transition delay once the tug stops.
		 */
		bp.step_start_t = bp.cur_t;
	} else  if (bp.cur_t - bp.step_start_t >= STATE_TRANS_DELAY) {
		tug_set_cradle_beeper_on(bp_ls.tug, B_TRUE);
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
	dr_t zibo_chocks;

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

	/* Workaround for Zibo 737 chocks being set - remove them. */
	if (dr_find(&zibo_chocks, "laminar/B738/fms/chock_status") &&
	    dr_geti(&zibo_chocks) != 0) {
		if (zibo_chocks.writable) {
			dr_seti(&zibo_chocks, 0);
		} else {
			XPLMSpeakString(_("Pushback warning: unable to remove "
			    "your chocks. Remove them yourself, or else I "
			    "won't be able to push your aircraft."));
		}
	}

	dir = hdg2dir(bp_ls.tug->pos.hdg);
	if (bp_ls.tug->info->lift_type == LIFT_GRAB) {
		p_end = vect2_add(bp_ls.tug->pos.pos, vect2_scmul(dir,
		    -(bp_ls.tug->info->apch_dist +
		    bp_ls.tug->info->lift_wall_z -
		    tug_lift_wall_off(bp_ls.tug))));
	} else {
		p_end = vect2_add(bp_ls.tug->pos.pos, vect2_scmul(dir,
		    -(bp_ls.tug->info->apch_dist + bp_ls.tug->info->plat_z)));
	}
	VERIFY(tug_drive2point(bp_ls.tug, p_end, bp.cur_pos.hdg));
	bp.step++;
	bp.step_start_t = bp.cur_t;
}

static void
pb_step_driving_up_connect(void)
{
	if (!slave_mode)
		brakes_set(B_TRUE);
	if (!tug_is_stopped(bp_ls.tug)) {
		/*
		 * Keep resetting the start time to enforce a state
		 * transition delay once the tug stops.
		 */
		bp.step_start_t = bp.cur_t;
	} else if (bp.cur_t - bp.step_start_t >= STATE_TRANS_DELAY) {
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
	tug_set_lift_arm_pos(bp_ls.tug, 1 - cradle_closed_fract, B_TRUE);

	if (!slave_mode) {
		/* When grabbing, keep the aircraft firmly in place */
		brakes_set(B_TRUE);
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
	const tug_info_t *ti = bp_ls.tug->info;
	double winch_total, winched_dist;

	/* spend some time putting the winching strap in place */
	if (!bp.winching.complete && d_t < STATE_TRANS_DELAY)
		return;

	tug_set_lift_pos(0);
	tug_set_winch_on(bp_ls.tug, B_TRUE);

	/* after installing the strap, wait some more to make the pbrake call */
	if (!bp.winching.complete && d_t < 2 * STATE_TRANS_DELAY) {
		tug_set_lift_arm_pos(bp_ls.tug, 1.0, B_TRUE);
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
		brakes_set(B_FALSE);
	}

	winch_total = ti->lift_wall_z - ti->plat_z -
	    tug_lift_wall_off(bp_ls.tug);
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
			push_at_speed(0.05, 0.05, B_FALSE, B_FALSE);
			dr_setvf(&drs.leg_len, &lift, bp.acf.nw_i, 1);
		}
		tug_set_lift_arm_pos(bp_ls.tug, 1 - x, B_TRUE);
		tug_set_TE_override(bp_ls.tug, B_TRUE);
		tug_set_TE_snd(bp_ls.tug, PB_LIFT_TE, bp.d_t);
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
	tug_set_cradle_beeper_on(bp_ls.tug, B_TRUE);
	tug_set_lift_in_transit(B_TRUE);
	if (bp_ls.tug->info->lift_type == LIFT_GRAB)
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
	lift = (bp_ls.tug->info->lift_height * lift_fract) + bp.acf.nw_len +
	    tug_plat_h(bp_ls.tug);
	if (!slave_mode) {
		brakes_set(B_TRUE);
		dr_setvf(&drs.leg_len, &lift, bp.acf.nw_i, 1);
	}

	/*
	 * While lifting, we simulate a ramp-up and ramp-down of the
	 * tug's Tractive Effort to simulate that the engine is
	 * being used to pressurize a hydraulic lift system.
	 */
	if (d_t < PB_CONN_LIFT_DURATION) {
		tug_set_TE_override(bp_ls.tug, B_TRUE);
		tug_set_TE_snd(bp_ls.tug, PB_LIFT_TE, bp.d_t);
	}
	if (d_t >= PB_CONN_LIFT_DURATION) {
		tug_set_TE_override(bp_ls.tug, B_TRUE);
		tug_set_TE_snd(bp_ls.tug, 0, bp.d_t);
		tug_set_cradle_beeper_on(bp_ls.tug, B_FALSE);
		tug_set_lift_in_transit(B_FALSE);
		tug_set_TE_override(bp_ls.tug, B_FALSE);
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

		if (bp_ls.tug->info->lift_type != LIFT_WINCH) {
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
	if (pbrake_is_set() ||
	    bp.cur_t - bp.last_voice_t < msg_dur(MSG_CONNECTED)) {
		/*
		 * Keep resetting the start time to enforce the state delay
		 * after the message is done and the parking brake is released.
		 */
		bp.step_start_t = bp.cur_t;
	} else if (bp.cur_t - bp.step_start_t >= STATE_TRANS_DELAY) {
		if (!slave_mode) {
			seg_t *seg = list_head(&bp.segs);

			ASSERT(seg != NULL);
			if (dr_geti(&drs.num_engns) == 0 ||
			    eng_is_running() || !eng_ok2start()) {
				msg_play(seg->backward ? MSG_START_PB_NOSTART :
				    MSG_START_TOW_NOSTART);
			} else {
				msg_play(seg->backward ? MSG_START_PB :
				    MSG_START_TOW);
			}
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
	if (dr_geti(&drs.landing_lights_on) != 0 ||
	    dr_geti(&drs.taxi_light_on) != 0) {
		if (!slave_mode)
			push_at_speed(0, bp.veh.max_accel, B_TRUE, B_TRUE);
		if (!bp.light_warn) {
			if (dr_geti(&drs.landing_lights_on) != 0) {
				XPLMSpeakString(_("Hey! Quit blinding me with "
				    "your landing lights! Turn them off!"));
			} else {
				XPLMSpeakString(_("Hey! Quit blinding me with "
				    "your taxi light! Turn it off!"));
			}
		}
		bp.light_warn = B_TRUE;
		return;
	} else if (bp.light_warn) {
		bp.light_warn = B_FALSE;
	}

	if (!slave_mode) {
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
		tug_set_TE_override(bp_ls.tug, B_FALSE);
	}
}

static void
pb_step_stopping(void)
{
	bool_t done = B_TRUE;

	tug_set_TE_override(bp_ls.tug, B_FALSE);
	if (!slave_mode) {
		vehicle_pos_t corr_pos;
		double steer, rhdg;

		VERIFY3S(dr_getvf(&drs.tire_steer_cmd, &steer, bp.acf.nw_i,
		    1), ==, 1);
		corr_pos = corr_acf_pos();
		;
		if (!isnan(bp.last_hdg) &&
		    fabs(rhdg = rel_hdg(corr_pos.hdg, bp.last_hdg)) > 1) {
			double nsteer = (bp.last_seg_is_back ? -10 : 10) * rhdg;
			turn_nosewheel(nsteer);
			push_at_speed(bp.last_seg_is_back ? -MIN_SPEED_XP10 :
			    MIN_SPEED_XP10, bp.veh.max_accel, B_FALSE, B_FALSE);
			done = B_FALSE;
		} else if (ABS(bp_ls.tug->cur_steer) >
		    TOW_COMPLETE_TUG_STEER_THRESH ||
		    ABS(steer) > TOW_COMPLETE_ACF_STEER_THRESH) {
			/* Keep pushing until steering is neutralized */
			turn_nosewheel(0);
			push_at_speed(bp.last_seg_is_back ? -MIN_SPEED_XP10 :
			    MIN_SPEED_XP10, bp.veh.max_accel, B_FALSE, B_FALSE);
			done = B_FALSE;
		} else {
			turn_nosewheel(0);
			push_at_speed(0, bp.veh.max_accel, B_FALSE, B_TRUE);
		}
	}
	if (ABS(bp.cur_pos.spd) >= SPEED_COMPLETE_THRESH || !done) {
		/*
		 * Keep resetting the start time to enforce a delay
		 * once stopped.
		 */
		bp.step_start_t = bp.cur_t;
	} else {
		if (!slave_mode)
			brakes_set(B_TRUE);
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
		turn_nosewheel(0);
		push_at_speed(0, bp.veh.max_accel, B_FALSE, B_FALSE);
		brakes_set(B_TRUE);
	}
	if (!pbrake_is_set()) {
		/*
		 * Keep resetting the start time to enforce a delay
		 * when the parking brake is set.
		 */
		bp.step_start_t = bp.cur_t;
	} else if (bp.cur_t - bp.step_start_t >= STATE_TRANS_DELAY &&
	    bp.cur_t - bp.last_voice_t >= msg_dur(MSG_OP_COMPLETE) +
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
		turn_nosewheel(0);
		brakes_set(B_TRUE);
	}

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
	lift = (bp_ls.tug->info->lift_height * lift_fract) + bp.acf.nw_len +
	    tug_plat_h(bp_ls.tug);
	if (!slave_mode)
		dr_setvf(&drs.leg_len, &lift, bp.acf.nw_i, 1);

	tug_set_lift_pos(lift_fract);
	tug_set_cradle_air_on(bp_ls.tug, B_TRUE, bp.cur_t);
	tug_set_cradle_beeper_on(bp_ls.tug, B_TRUE);

	if (lift_fract == 0) {
		tug_set_cradle_air_on(bp_ls.tug, B_FALSE, bp.cur_t);
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
	tug_set_lift_arm_pos(bp_ls.tug, cradle_fract, B_TRUE);

	if (cradle_fract >= 1.0)
		tug_set_cradle_beeper_on(bp_ls.tug, B_FALSE);

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

	tug_set_winch_on(bp_ls.tug, B_FALSE);

	if (d_t < 2 * STATE_TRANS_DELAY)
		return (B_FALSE);

	return (B_TRUE);
}

static void
pb_step_ungrabbing(void)
{
	bool_t complete;

	if (bp_ls.tug->info->lift_type == LIFT_GRAB)
		complete = pb_step_ungrabbing_grab();
	else
		complete = pb_step_ungrabbing_winch();

	if (complete) {
		if (!slave_mode)
			brakes_set(B_FALSE);

		tug_set_lift_in_transit(B_FALSE);
		tug_set_TE_override(bp_ls.tug, B_FALSE);

		/* reset the state for the disconnection phase */
		bp.reconnect = B_FALSE;
		bp.ok2disco = B_FALSE;

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
	tug_set_tire_sense_pos(bp_ls.tug, 1 - d_t / PB_CRADLE_DELAY);
	tug_set_lift_pos(d_t / PB_CRADLE_DELAY);

	if (d_t >= PB_CRADLE_DELAY) {
		tug_set_cradle_beeper_on(bp_ls.tug, B_FALSE);
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
disco_win_draw(XPLMWindowID inWindowID, void *inRefcon)
{
	int w, h, mx, my;

	UNUSED(inRefcon);
	XPLMGetScreenSize(&w, &h);
	XPLMGetMouseLocation(&mx, &my);

	XPLMSetGraphicsState(0, 1, 0, 0, 1, 0, 0);
	if (inWindowID == bp_ls.disco_win) {
		bool_t is_lit = (mx >= w / 2 - 1.5 * disco_buttons[0].w &&
		    mx <= w / 2 - 0.5 * disco_buttons[0].w &&
		    my >= h - 1.5 * disco_buttons[0].h &&
		    my <= h - 0.5 * disco_buttons[0].h);
		draw_icon(&disco_buttons[0], w / 2 - 1.5 * disco_buttons[0].w,
		    h - 1.5 * disco_buttons[0].h, 1.0,
		    B_FALSE, is_lit);
	} else {
		bool_t is_lit = (mx >= w / 2 + 0.5 * disco_buttons[1].w &&
		    mx <= w / 2 + 1.5 * disco_buttons[1].w &&
		    my >= h - 1.5 * disco_buttons[1].h &&
		    my <= h - 0.5 * disco_buttons[1].h);
		ASSERT(inWindowID == bp_ls.recon_win);
		draw_icon(&disco_buttons[1], w / 2 + 0.5 * disco_buttons[1].w,
		    h - 1.5 * disco_buttons[1].h, 1.0,
		    B_FALSE, is_lit);
	}
}

static int
disco_handler(XPLMCommandRef cmd, XPLMCommandPhase phase, void *refcon)
{
	UNUSED(cmd);
	UNUSED(phase);
	UNUSED(refcon);

	if (bp.step != PB_STEP_WAITING4OK2DISCO)
		return (0);
	bp.ok2disco = B_TRUE;

	return (1);
}

static int
recon_handler(XPLMCommandRef cmd, XPLMCommandPhase phase, void *refcon)
{
	UNUSED(cmd);
	UNUSED(phase);
	UNUSED(refcon);

	if (bp.step != PB_STEP_WAITING4OK2DISCO)
		return (0);

	/*
	 * Reconnection works as follows:
	 * 1) We shift state back to the grabbing step, so the tug starts
	 *    the reattachment and lift process.
	 * 2) We notify the GUI portion that a reconnection has taken place.
	 */
	op_complete = B_FALSE;
	bp.reconnect = B_TRUE;
	bp.step = PB_STEP_GRABBING;
	bp.step_start_t = bp.cur_t;
	bp_reconnect_notify();
	return (1);
}

static int
disco_win_click(XPLMWindowID inWindowID, int x, int y, XPLMMouseStatus inMouse,
    void *inRefcon)
{
	UNUSED(x);
	UNUSED(y);
	UNUSED(inRefcon);

	if (inMouse != xplm_MouseUp)
		return (1);
	if (inWindowID == bp_ls.disco_win) {
		XPLMCommandOnce(disco_cmd);
	} else if (inWindowID == bp_ls.recon_win)
		XPLMCommandOnce(recon_cmd);

	return (1);
}

static XPLMCursorStatus
nil_win_cursor(XPLMWindowID inWindowID, int x, int y, void *inRefcon)
{
	UNUSED(inWindowID);
	UNUSED(x);
	UNUSED(y);
	UNUSED(inRefcon);
	return (xplm_CursorDefault);
}

static int
nil_win_wheel(XPLMWindowID inWindowID, int x, int y, int wheel, int clicks,
    void *inRefcon)
{
	UNUSED(inWindowID);
	UNUSED(x);
	UNUSED(y);
	UNUSED(wheel);
	UNUSED(clicks);
	UNUSED(inRefcon);
	return (1);
}

static void
disco_intf_show(void)
{
	XPLMCreateWindow_t disco_ops = {
	    .structSize = sizeof (XPLMCreateWindow_t),
	    .left = 0, .top = 0, .right = 0, .bottom = 0, .visible = 1,
	    .drawWindowFunc = disco_win_draw,
	    .handleMouseClickFunc = disco_win_click,
	    .handleKeyFunc = nil_win_key,
	    .handleCursorFunc = nil_win_cursor,
	    .handleMouseWheelFunc = nil_win_wheel,
	    .refcon = NULL
	};
	int w, h;

	XPLMGetScreenSize(&w, &h);

	disco_ops.left = w / 2 - 1.5 * disco_buttons[0].w;
	disco_ops.right = w / 2 - 0.5 * disco_buttons[0].w;
	disco_ops.top = h - 0.5 * disco_buttons[0].h;
	disco_ops.bottom = h - 1.5 * disco_buttons[0].h;
	bp_ls.disco_win = XPLMCreateWindowEx(&disco_ops);
	ASSERT(bp_ls.disco_win != NULL);
	XPLMBringWindowToFront(bp_ls.disco_win);

	disco_ops.left = w / 2 + 0.5 * disco_buttons[1].w;
	disco_ops.right = w / 2 + 1.5 * disco_buttons[1].w;
	disco_ops.top = h - 0.5 * disco_buttons[1].h;
	disco_ops.bottom = h - 1.5 * disco_buttons[1].h;
	bp_ls.recon_win = XPLMCreateWindowEx(&disco_ops);
	ASSERT(bp_ls.recon_win != NULL);
	XPLMBringWindowToFront(bp_ls.recon_win);
}

static void
disco_intf_hide(void)
{
	if (bp_ls.disco_win != NULL) {
		XPLMDestroyWindow(bp_ls.disco_win);
		bp_ls.disco_win = NULL;
	}
	if (bp_ls.recon_win != NULL) {
		XPLMDestroyWindow(bp_ls.recon_win);
		bp_ls.recon_win = NULL;
	}
}

static void
pb_step_waiting4ok2disco(void)
{
	if (!bp.ok2disco) {
		if (bp_ls.disco_win == NULL && !slave_mode)
			disco_intf_show();

		/* Keep resetting the start time to enforce the delay */
		bp.step_start_t = bp.cur_t;
		return;
	}

	/* Once the user clicked disconnect, hide the buttons immediately */
	disco_intf_hide();

	if (bp.cur_t - bp.step_start_t >= STATE_TRANS_DELAY) {
		vect2_t dir, p;

		dir = hdg2dir(bp.cur_pos.hdg);
		p = vect2_add(bp.cur_pos.pos, vect2_scmul(dir,
		    -bp.acf.nw_z + bp_ls.tug->info->apch_dist));
		(void) tug_drive2point(bp_ls.tug, p, bp.cur_pos.hdg);

		bp.step++;
		bp.step_start_t = bp.cur_t;
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
	square_side = MAX(4 * bp_ls.tug->veh.wheelbase, 1.5 * bp.veh.wheelbase);

	dir = hdg2dir(bp.cur_pos.hdg);
	norm_dir = vect2_norm(dir, right);

	/*
	 * turn_p is offset 3x tug wheelbase forward and
	 * half square_side to the direction of the turn.
	 */
	turn_p = vect2_add(bp_ls.tug->pos.pos, vect2_scmul(dir,
	    3 * bp_ls.tug->veh.wheelbase));
	turn_p = vect2_add(turn_p, vect2_scmul(norm_dir,
	    square_side / 2));
	turn_hdg = normalize_hdg(bp.cur_pos.hdg + (right ? 90 : -90));

	/*
	 * abeam point is displaced from turn_p back 2x tug wheelbase,
	 * 4x tug wheelbase in the direction of the turn and going the
	 * opposite way to the aircraft at a 45 degree angle.
	 */
	abeam_p = vect2_add(turn_p, vect2_scmul(vect2_neg(dir),
	    2 * bp_ls.tug->veh.wheelbase));
	abeam_p = vect2_add(abeam_p, vect2_scmul(norm_dir,
	    4 * bp_ls.tug->veh.wheelbase));
	back_hdg = normalize_hdg(turn_hdg + (right ? 45 : -45));

	VERIFY(tug_drive2point(bp_ls.tug, turn_p, turn_hdg));
	VERIFY(tug_drive2point(bp_ls.tug, abeam_p, back_hdg));

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
	vect2_t end_p = vect2_add(bp_ls.tug->pos.pos,
	    vect2_scmul(hdg2dir(bp_ls.tug->pos.hdg), TUG_DRIVE_AWAY_DIST));

	tug_set_pos(bp_ls.tug, bp_ls.tug->pos.pos, bp_ls.tug->pos.hdg, 0);
	VERIFY(tug_drive2point(bp_ls.tug, end_p, bp_ls.tug->pos.hdg));
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
		double rhdg = fabs(rel_hdg(bp_ls.tug->pos.hdg,
		    dir2hdg(vect2_sub(bp.start_pos, bp_ls.tug->pos.pos))));
		/*
		 * start_pos seems far enough away from the aircraft that
		 * it won't be a problem if we drive to it. Just make sure
		 * we're not trying to back into it.
		 */
		if (rhdg >= 90 || !tug_drive2point(bp_ls.tug, bp.start_pos,
		    bp.start_hdg)) {
			/*
			 * It's possible the start_pos is beyond a 90 degree
			 * turn, so we'd attempt to back into it. Try to stick
			 * in an intermediate 90-degree turn in its direction.
			 */
			bool_t right = (rel_hdg(bp_ls.tug->pos.hdg, dir2hdg(
			    vect2_sub(bp.start_pos, bp_ls.tug->pos.pos))) >= 0);
			vect2_t dir = hdg2dir(bp_ls.tug->pos.hdg);
			vect2_t turn_p = vect2_add(bp_ls.tug->pos.pos,
			    vect2_scmul(dir, 2 * bp_ls.tug->veh.wheelbase));
			turn_p = vect2_add(turn_p, vect2_scmul(vect2_norm(dir,
			    right), 2 * bp_ls.tug->veh.wheelbase));
			if (!tug_drive2point(bp_ls.tug, turn_p, normalize_hdg(
			    bp_ls.tug->pos.hdg + (right ? 90 : -90))) ||
			    !tug_drive2point(bp_ls.tug, bp.start_pos,
			    bp.start_hdg)) {
				drive_away_fallback();
			}
		}
	}
	tug_set_clear_signal(B_FALSE, B_FALSE);
	bp.step++;
	bp.step_start_t = bp.cur_t;
}

/*
 * Updates the tug's position with respect to where we are and its orientation
 * based on the tug's current steering input. When `pos_only' is true, only
 * the tug's position is update to match our nose gear position, but we leave
 * its heading untouched. This is because this can be called from the draw
 * function as well, which might update more frequently than the flight loop,
 * so we want to keep the tug firmly attached to our nosewheel, but not
 * actually change any params that might affect our steering.
 */
static void
tug_pos_update(vect2_t my_pos, double my_hdg, bool_t pos_only)
{
	double tug_hdg, tug_spd, steer, radius;
	vect2_t dir, tug_pos;

	dr_getvf(&drs.tire_steer_cmd, &steer, bp.acf.nw_i, 1);

	tug_spd = bp.cur_pos.spd / cos(DEG2RAD(fabs(steer)));

	radius = tan(DEG2RAD(90 - bp_ls.tug->cur_steer)) *
	    bp_ls.tug->veh.wheelbase;
	if (pos_only) {
		tug_hdg = bp_ls.tug->pos.hdg;
	} else if (slave_mode) {
		/*
		 * In slave mode, the tug tracks our nosewheel and doesn't
		 * actually do any steering of its own.
		 */
		tug_hdg = normalize_hdg(my_hdg + steer);
	} else if (fabs(radius) < 1e3) {
		double d_hdg = RAD2DEG(tug_spd / radius) * bp.d_t;
		double r_hdg;

		tug_hdg = normalize_hdg(bp_ls.tug->pos.hdg + d_hdg);
		r_hdg = rel_hdg(my_hdg, tug_hdg);
		/* check if we hit the hard steering stop */
		if (r_hdg > bp.veh.max_steer)
			tug_hdg = normalize_hdg(my_hdg + bp.veh.max_steer);
		else if (r_hdg < -bp.veh.max_steer)
			tug_hdg = normalize_hdg(my_hdg - bp.veh.max_steer);
	} else {
		tug_hdg = bp_ls.tug->pos.hdg;
	}

	dir = hdg2dir(my_hdg);
	if (bp.step == PB_STEP_GRABBING &&
	    bp_ls.tug->info->lift_type == LIFT_WINCH) {
		/*
		 * When winching the aircraft forward, we keep the tug in a
		 * fixed position relative to where the aircraft was when the
		 * winching operation started.
		 */
		tug_set_pos(bp_ls.tug, vect2_add(bp.winching.start_acf_pos,
		    vect2_scmul(dir, (-bp.acf.nw_z) +
		    (-bp_ls.tug->info->plat_z))), my_hdg, 0);
	} else {
		vect2_t off_v = vect2_scmul(hdg2dir(tug_hdg),
		    (-bp_ls.tug->info->lift_wall_z) +
		    tug_lift_wall_off(bp_ls.tug));
		tug_pos = vect2_add(vect2_add(my_pos, vect2_scmul(dir,
		    -bp.acf.nw_z)), off_v);
		tug_set_pos(bp_ls.tug, tug_pos, tug_hdg, tug_spd);
	}
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

	ASSERT(bp_ls.tug != NULL || bp.step <= PB_STEP_TUG_LOAD);
	if (bp_ls.tug != NULL) {
		/* drive slowly while approaching & moving away from acf */
		tug_run(bp_ls.tug, bp.d_t,
		    bp.step == PB_STEP_DRIVING_UP_CONNECT ||
		    bp.step == PB_STEP_MOVING_AWAY);
		tug_anim(bp_ls.tug, bp.d_t, bp.cur_t);

		if (list_head(&bp_ls.tug->segs) == NULL &&
		    bp.step >= PB_STEP_GRABBING &&
		    bp.step <= PB_STEP_UNGRABBING)
			tug_pos_update(bp.cur_pos.pos, bp.cur_pos.hdg, B_FALSE);
	}

	if (!slave_mode) {
		/*
		 * We persistently try to enable nosewheel steering. If by
		 * reaching PB_STEP_START nosewheel steering is still disabled,
		 * that means something else is resetting the variable to '0'.
		 * Stop the operation, somebody is trying to mess with us.
		 */
		if (bp.step > PB_STEP_START && dr_geti(&drs.nw_steer_on) != 1) {
			XPLMSpeakString(_("Pushback failure: your flight "
			    "controls are preventing me from steering the "
			    "aircraft. Unbind any buttons you have set to "
			    "\"toggle nosewheel steering\"."));
			msg_stop();
			bp_complete();
			return (0);
		}
		dr_seti(&drs.nw_steer_on, 1);
		if (bp.step >= PB_STEP_DRIVING_UP_CONNECT &&
		    bp.step <= PB_STEP_MOVING_AWAY) {
			dr_seti(&drs.override_steer, 1);
		} else {
			dr_seti(&drs.override_steer, 0);
		}
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
			/*
			 * If we're effectively stopped, skip the stopping
			 * step to avoid playing MSG_OP_COMPLETE.
			 */
			if (ABS(bp.cur_pos.spd) < SPEED_COMPLETE_THRESH &&
			    pbrake_is_set()) {
				bp.step = PB_STEP_STOPPED;
			} else {
				bp.step = PB_STEP_STOPPING;
			}
		}
	}

	/*
	 * When performing quick debugging, skip the whole driving-up phase.
	 */
	if (!slave_mode && bp_ls.tug != NULL && bp_ls.tug->info->quick_debug) {
		if (bp.step < PB_STEP_CONNECTED) {
			double lift = bp_ls.tug->info->lift_height +
			    bp.acf.nw_len;
			bp.step = PB_STEP_CONNECTED;
			tug_set_lift_pos(1);
			tug_set_lift_arm_pos(bp_ls.tug, 0, B_TRUE);
			dr_setvf(&drs.leg_len, &lift, bp.acf.nw_i, 1);
			/*
			 * Just a quick'n'dirty way of removing all tug driving
			 * segs. The actual tug position will be updated in
			 * draw_tugs.
			 */
			tug_set_pos(bp_ls.tug, ZERO_VECT2, bp.cur_pos.hdg, 0);
		} else if (bp.step == PB_STEP_UNGRABBING) {
			dr_setvf(&drs.leg_len, &bp.acf.nw_len, bp.acf.nw_i, 1);
			bp_complete();
			return (0);
		}
	}

	if (bp.step != PB_STEP_WAITING4OK2DISCO) {
		/*
		 * If the user requests reconnection, we cannot destroy the
		 * window from within the mouse handler, so we destroy it
		 * here instead.
		 */
		disco_intf_hide();
	}

	switch (bp.step) {
	case PB_STEP_OFF:
		VERIFY(bp.step != PB_STEP_OFF);
	case PB_STEP_TUG_LOAD:
		ASSERT3P(bp_ls.tug, ==, NULL);
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
		tug_set_tire_sense_pos(bp_ls.tug, d_t / PB_CRADLE_DELAY);
		if (d_t >= PB_CRADLE_DELAY) {
			tug_set_lift_in_transit(B_FALSE);
			tug_set_cradle_beeper_on(bp_ls.tug, B_FALSE);
			prop_single_adjust();
		}
		if (d_t >= PB_CRADLE_DELAY + STATE_TRANS_DELAY) {
			if (!bp.reconnect) {
				if (pbrake_is_set())
					msg_play(MSG_RDY2CONN_NOPARK);
				else
					msg_play(MSG_RDY2CONN);
				bp.last_voice_t = bp.cur_t;
			}
			bp.step++;
			bp.step_start_t = bp.cur_t;
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
		if (!slave_mode) {
			dr_seti(&drs.override_steer, 1);
			brakes_set(B_FALSE);
		}
		if (bp.cur_t - bp.step_start_t >= PB_START_DELAY) {
			bp.step++;
			bp.step_start_t = bp.cur_t;
		} else if (!slave_mode) {
			seg_t *seg = list_tail(&bp.segs);
			ASSERT(seg != NULL);
			bp.last_seg_is_back = seg->backward;
			/*
			 * Try to straighten out if we don't end
			 * in a straight segment.
			 */
			if (seg->type == SEG_TYPE_TURN)
				bp.last_hdg = seg->end_hdg;
			else
				bp.last_hdg = NAN;
			turn_nosewheel(0);
			push_at_speed(0, bp.veh.max_accel, B_FALSE, B_FALSE);
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
	case PB_STEP_WAITING4OK2DISCO:
		pb_step_waiting4ok2disco();
		break;
	case PB_STEP_MOVING_AWAY:
		if (bp_ls.tug->info->lift_type == LIFT_WINCH && !slave_mode) {
			/*
			 * When moving the tug away from the aircraft, the
			 * aircraft will have been positioned on the platform.
			 * Slowly lower the nosewheel the rest of the way.
			 */
			double dist = vect2_dist(bp.cur_pos.pos,
			    bp_ls.tug->pos.pos);
			const tug_info_t *ti = bp_ls.tug->info;
			double plat_len = ti->lift_wall_z - ti->plat_z;
			double x, lift;

			dist -= (-bp.acf.nw_z);
			dist -= (-ti->lift_wall_z);
			x = 1 - (dist / plat_len);
			x = MIN(MAX(x, 0), 1);
			lift = ti->plat_h * x + bp.acf.nw_len;
			dr_setvf(&drs.leg_len, &lift, bp.acf.nw_i, 1);
		}
		if (tug_is_stopped(bp_ls.tug)) {
			tug_set_cradle_beeper_on(bp_ls.tug, B_TRUE);
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
		if (tug_is_stopped(bp_ls.tug)) {
			bp.step++;
			bp.step_start_t = bp.cur_t;
		}
		break;
	case PB_STEP_CLEAR_SIGNAL:
		pb_step_clear_signal();
		break;
	case PB_STEP_DRIVING_AWAY:
		if (tug_is_stopped(bp_ls.tug) ||
		    bp.cur_t - bp.step_start_t > MAX_DRIVING_AWAY_DELAY) {
			bp_complete();
			/*
			 * Can't unregister floop from within, so just tell
			 * X-Plane to not call us anymore. bp_fini will take
			 * care of the rest.
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
static XPLMObjectRef	cam_lamp_obj = NULL;

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
	    bp_get_lang(), btn->filename, NULL);
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

static void
unload_icon(button_t *btn)
{
	if (btn->tex != 0) {
		glDeleteTextures(1, &btn->tex);
		btn->tex = 0;
	}
	if (btn->tex_data != NULL) {
		free(btn->tex_data);
		btn->tex_data = NULL;
	}
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
	for (int i = 0; buttons[i].filename != NULL; i++)
		unload_icon(&buttons[i]);
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
	double wing_long_off = bp.acf.main_z - bp_ls.outline->wingtip.y;
	vect2_t wing_off_l = VECT2(-bp_ls.outline->semispan, wing_long_off);
	vect2_t wing_off_r = VECT2(bp_ls.outline->semispan, wing_long_off);

	switch (seg->type) {
	case SEG_TYPE_STRAIGHT: {
		float h1, h2;
		vect2_t wing_l, wing_r, p;

		VERIFY3U(XPLMProbeTerrainXYZ(probe, seg->start_pos.x, 0,
		    -seg->start_pos.y, &info), ==, xplm_ProbeHitTerrain);
		h1 = info.locationY;
		VERIFY3U(XPLMProbeTerrainXYZ(probe, seg->end_pos.x, 0,
		    -seg->end_pos.y, &info), ==, xplm_ProbeHitTerrain);
		h2 = info.locationY;

		glColor3f(0, 0, 1);
		glLineWidth(3);
		glBegin(GL_LINES);
		glVertex3f(seg->start_pos.x, h1, -seg->start_pos.y);
		glVertex3f(seg->end_pos.x, h2, -seg->end_pos.y);
		glEnd();

		wing_l = vect2_rot(wing_off_l, seg->start_hdg);
		wing_r = vect2_rot(wing_off_r, seg->start_hdg);

		glColor3f(1, 0.25, 1);
		glLineWidth(2);
		glBegin(GL_LINES);
		p = vect2_add(seg->start_pos, wing_r);
		glVertex3f(p.x, h1, -p.y);
		p = vect2_add(seg->end_pos, wing_r);
		glVertex3f(p.x, h1, -p.y);
		p = vect2_add(seg->end_pos, wing_l);
		glVertex3f(p.x, h1, -p.y);
		p = vect2_add(seg->start_pos, wing_l);
		glVertex3f(p.x, h1, -p.y);
		glEnd();
		break;
	}
	case SEG_TYPE_TURN: {
		vect2_t c = vect2_add(seg->start_pos, vect2_scmul(
		    vect2_norm(hdg2dir(seg->start_hdg), seg->turn.right),
		    seg->turn.r));
		vect2_t c2s = vect2_sub(seg->start_pos, c);
		double s, e, rhdg;

		rhdg = rel_hdg(seg->start_hdg, seg->end_hdg);
		s = MIN(0, rhdg);
		e = MAX(0, rhdg);
		ASSERT3F(s, <=, e);
		for (double a = s; a < e; a += ANGLE_DRAW_STEP) {
			vect2_t p1, p2, p;
			vect2_t wing1_l, wing1_r, wing2_l, wing2_r;
			double step = MIN(ANGLE_DRAW_STEP, e - a);

			wing1_l = vect2_rot(wing_off_l, seg->start_hdg + a);
			wing1_r = vect2_rot(wing_off_r, seg->start_hdg + a);
			wing2_l = vect2_rot(wing_off_l,
			    seg->start_hdg + a + step);
			wing2_r = vect2_rot(wing_off_r,
			    seg->start_hdg + a + step);

			p1 = vect2_add(c, vect2_rot(c2s, a));
			p2 = vect2_add(c, vect2_rot(c2s, a + step));

			VERIFY3U(XPLMProbeTerrainXYZ(probe, p1.x, 0, -p1.y,
			    &info), ==, xplm_ProbeHitTerrain);

			glColor3f(0, 0, 1);
			glLineWidth(3);
			glBegin(GL_LINES);
			glVertex3f(p1.x, info.locationY, -p1.y);
			glVertex3f(p2.x, info.locationY, -p2.y);
			glEnd();

			glColor3f(1, 0.25, 1);
			glLineWidth(2);
			glBegin(GL_LINES);
			p = vect2_add(p1, wing1_r);
			glVertex3f(p.x, info.locationY, -p.y);
			p = vect2_add(p2, wing2_r);
			glVertex3f(p.x, info.locationY, -p.y);
			p = vect2_add(p1, wing1_l);
			glVertex3f(p.x, info.locationY, -p.y);
			p = vect2_add(p2, wing2_l);
			glVertex3f(p.x, info.locationY, -p.y);
			glEnd();
		}
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
	double tire_x[10], tire_z[10], tirrad[10];

	/*
	 * The wheelbase of most airlines is very roughly 1/3 of their
	 * total length, so multiplying by 1.5 makes this value approx
	 * half their size and gives a good rough "size ring" radius.
	 */
	wheelbase *= 1.5;

	glLineWidth(2);
	glColor3f(color.x, color.y, color.z);

	glBegin(GL_LINES);
	for (size_t i = 0; i + 1 < bp_ls.outline->num_pts; i++) {
		/* skip gaps in outline */
		if (IS_NULL_VECT(bp_ls.outline->pts[i]) ||
		    IS_NULL_VECT(bp_ls.outline->pts[i + 1]))
			continue;

		v = bp_ls.outline->pts[i];
		v = vect2_rot(VECT2(v.x, bp.acf.main_z - v.y), hdg);
		p = vect3_add(pos, VECT3(v.x, 0, v.y));
		glVertex3f(p.x, p.y, -p.z);

		v = bp_ls.outline->pts[i + 1];
		v = vect2_rot(VECT2(v.x, bp.acf.main_z - v.y), hdg);
		p = vect3_add(pos, VECT3(v.x, 0, v.y));
		glVertex3f(p.x, p.y, -p.z);

		v = bp_ls.outline->pts[i];
		v = vect2_rot(VECT2(-v.x, bp.acf.main_z - v.y), hdg);
		p = vect3_add(pos, VECT3(v.x, 0, v.y));
		glVertex3f(p.x, p.y, -p.z);

		v = bp_ls.outline->pts[i + 1];
		v = vect2_rot(VECT2(-v.x, bp.acf.main_z - v.y), hdg);
		p = vect3_add(pos, VECT3(v.x, 0, v.y));
		glVertex3f(p.x, p.y, -p.z);
	}
	glEnd();

	dr_getvf(&drs.tire_x, tire_x, 0, 10);
	dr_getvf(&drs.tire_z, tire_z, 0, 10);
	dr_getvf(&drs.tirrad, tirrad, 0, 10);

	glBegin(GL_QUADS);
	for (int i = 0; i < bp.acf.n_gear; i++) {
		double tr = tirrad[bp.acf.gear_is[i]];
		vect2_t c;

		v = VECT2(tire_x[bp.acf.gear_is[i]],
		    bp.acf.main_z - tire_z[bp.acf.gear_is[i]]);

		c = vect2_rot(vect2_add(v, VECT2(-tr, -tr)), hdg);
		p = vect3_add(pos, VECT3(c.x, 0, c.y));
		glVertex3f(p.x, p.y, -p.z);
		c = vect2_rot(vect2_add(v, VECT2(-tr, tr)), hdg);
		p = vect3_add(pos, VECT3(c.x, 0, c.y));
		glVertex3f(p.x, p.y, -p.z);
		c = vect2_rot(vect2_add(v, VECT2(tr, tr)), hdg);
		p = vect3_add(pos, VECT3(c.x, 0, c.y));
		glVertex3f(p.x, p.y, -p.z);
		c = vect2_rot(vect2_add(v, VECT2(tr, -tr)), hdg);
		p = vect3_add(pos, VECT3(c.x, 0, c.y));
		glVertex3f(p.x, p.y, -p.z);
	}
	glEnd();
}

static int
draw_prediction(XPLMDrawingPhase phase, int before, void *refcon)
{
	seg_t *seg;
	XPLMProbeRef probe = XPLMCreateProbe(xplm_ProbeY);
	XPLMProbeInfo_t info = { .structSize = sizeof (XPLMProbeInfo_t) };
	XPLMDrawInfo_t di;

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
			glVertex3f(seg->end_pos.x, info.locationY,
			    -seg->end_pos.y);
			x = vect2_add(seg->end_pos, vect2_scmul(dir_v,
			    ORIENTATION_LINE_LEN));
			glVertex3f(x.x, info.locationY, -x.y);
			glEnd();
		}
		if (seg->type == SEG_TYPE_TURN || seg->backward) {
			glBegin(GL_LINES);
			glColor3f(1, 0, 0);
			glVertex3f(seg->end_pos.x, info.locationY,
			    -seg->end_pos.y);
			x = vect2_add(seg->end_pos, vect2_neg(vect2_scmul(
			    dir_v, ORIENTATION_LINE_LEN)));
			glVertex3f(x.x, info.locationY, -x.y);
			glEnd();
		}
		draw_acf_symbol(VECT3(seg->end_pos.x, info.locationY,
		    seg->end_pos.y), seg->end_hdg, bp.veh.wheelbase,
		    AMBER_TUPLE);
	} else {
		VERIFY3U(XPLMProbeTerrainXYZ(probe, cursor_world_pos.x, 0,
		    -cursor_world_pos.y, &info), ==, xplm_ProbeHitTerrain);
		draw_acf_symbol(VECT3(cursor_world_pos.x, info.locationY,
		    cursor_world_pos.y), cursor_hdg, bp.veh.wheelbase,
		    RED_TUPLE);
	}

	if ((seg = list_tail(&bp.segs)) != NULL) {
		VERIFY3U(XPLMProbeTerrainXYZ(probe, seg->end_pos.x, 0,
		    -seg->end_pos.y, &info), ==, xplm_ProbeHitTerrain);
		draw_acf_symbol(VECT3(seg->end_pos.x, info.locationY,
		    seg->end_pos.y), seg->end_hdg, bp.veh.wheelbase,
		    GREEN_TUPLE);
	}

	/* Draw the night-lighting lamp so the user can see under the cursor */
	VERIFY3U(XPLMProbeTerrainXYZ(probe, cursor_world_pos.x, 0,
	    -cursor_world_pos.y, &info), ==, xplm_ProbeHitTerrain);
	di.structSize = sizeof (di);
	di.x = cursor_world_pos.x;
	di.y = info.locationY;
	di.z = -cursor_world_pos.y;
	di.heading = 0;
	di.pitch = 0;
	di.roll = 0;
	XPLMDrawObjects(cam_lamp_obj, 1, &di, 1, 1);

	XPLMDestroyProbe(probe);

	return (1);
}

static void
draw_icon(button_t *btn, int x, int y, double scale, bool_t is_clicked,
    bool_t is_lit)
{
	glBindTexture(GL_TEXTURE_2D, btn->tex);
	glBegin(GL_QUADS);
	glTexCoord2f(0, 1);
	glVertex2f(x, y);
	glTexCoord2f(0, 0);
	glVertex2f(x, y + btn->h * scale);
	glTexCoord2f(1, 0);
	glVertex2f(x + btn->w * scale, y + btn->h * scale);
	glTexCoord2f(1, 1);
	glVertex2f(x + btn->w * scale, y);
	glEnd();

	if (is_clicked) {
		/*
		 * If this button was hit by a mouse click, highlight
		 * it by drawing a translucent white quad over it.
		 */
		XPLMSetGraphicsState(0, 0, 0, 0, 1, 0, 0);
		glColor4f(1, 1, 1, 0.3);
		glBegin(GL_QUADS);
		glVertex2f(x, y);
		glVertex2f(x, y + btn->h * scale);
		glVertex2f(x + btn->w * scale, y + btn->h * scale);
		glVertex2f(x + btn->w * scale, y);
		glEnd();
		XPLMSetGraphicsState(0, 1, 0, 0, 1, 0, 0);

	} else if (is_lit) {
		XPLMSetGraphicsState(0, 0, 0, 0, 1, 0, 0);
		glColor4f(1, 1, 1, 1);
		glLineWidth(1);
		glBegin(GL_LINES);
		glVertex2f(x, y);
		glVertex2f(x, y + btn->h * scale);
		glVertex2f(x, y + btn->h * scale);
		glVertex2f(x + btn->w * scale, y + btn->h * scale);
		glVertex2f(x + btn->w * scale, y + btn->h * scale);
		glVertex2f(x + btn->w * scale, y);
		glVertex2f(x, y);
		glEnd();
		XPLMSetGraphicsState(0, 1, 0, 0, 1, 0, 0);
	}
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

		draw_icon(btn, w - btn->w * scale, h_off - btn->h * scale,
		    scale, i == button_hit, i == button_lit);
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
nil_win_key(XPLMWindowID inWindowID, char inKey, XPLMKeyFlags inFlags,
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

	switch ((unsigned char)inVirtualKey) {
	case XPLM_VK_RETURN:
	case XPLM_VK_ENTER:
	case XPLM_VK_NUMPAD_ENT:
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
	    .handleKeyFunc = nil_win_key,
	    .handleCursorFunc = fake_win_cursor,
	    .handleMouseWheelFunc = fake_win_wheel,
	    .refcon = NULL
	};
	char icao[8] = { 0 };
	char *cam_obj_path;
	char airline[1024] = { 0 };

	if (cam_inited || !bp_init())
		return (B_FALSE);

	cam_obj_path = mkpathname(bp_xpdir, bp_plugindir, "objects",
	    "night_lamp.obj", NULL);
	cam_lamp_obj = XPLMLoadObject(cam_obj_path);
	if (cam_lamp_obj == NULL) {
		logMsg("Error loading pushback lamp %s. Please reinstall "
		    "BetterPushback.", cam_obj_path);
		free(cam_obj_path);
		return (B_FALSE);
	}
	free(cam_obj_path);

	if (!acf_is_compatible()) {
		XPLMSpeakString(_("Pushback failure: aircraft is incompatible "
		    "with BetterPushback."));
		return (B_FALSE);
	}

	(void) find_nearest_airport(icao);
	if (acf_is_airliner())
		read_acf_airline(airline);
	if (!tug_available(dr_getf(&drs.mtow), bp.acf.nw_len, bp.acf.tirrad,
	    bp.acf.nw_type, strcmp(icao, "") != 0 ? icao : NULL, airline)) {
		XPLMSpeakString(_("Pushback failure: no suitable tug for your "
		    "aircraft."));
		return (B_FALSE);
	}

#ifndef	PB_DEBUG_INTF
	if (vect3_abs(VECT3(dr_getf(&drs.vx), dr_getf(&drs.vy),
	    dr_getf(&drs.vz))) > 0.1) {
		XPLMSpeakString(_("Can't start planner: aircraft not "
		    "stationary."));
		return (B_FALSE);
	}
	if (bp_started && !late_plan_requested) {
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

	XPLMUnloadObject(cam_lamp_obj);
	cam_lamp_obj = NULL;

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
