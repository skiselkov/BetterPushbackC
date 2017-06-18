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

#include "assert.h"
#include "geom.h"
#include "list.h"
#include "dr.h"

#define	STRAIGHT_STEER_RATE	40	/* degrees per second */
#define	TURN_STEER_RATE		10	/* degrees per second */
#define	STRAIGHT_SPEED		1.11	/* m/s [4 km/h, "walking speed"] */
#define	FAST_STRAIGHT_SPEED	7	/* m/s [~14 knots] */
#define	TURN_SPEED		0.75	/* m/s [2.5 km/h] */
#define	STRAIGHT_ACCEL		0.25	/* m/s^2 */
#define	TURN_ACCEL		0.25	/* m/s^2 */
#define	BRAKE_PEDAL_THRESH	0.1	/* brake pedal angle, 0..1 */
#define	FORCE_PER_TON		5000	/* max push force per ton, Newtons */
#define	BREAKAWAY_THRESH	0.1	/* m/s */
#define	SEG_TURN_MULT		0.9	/* leave 10% for oversteer */
#define	TURN_COMPLETE_THRESH	2	/* degrees */

typedef struct {
	double		mass;
	double		wheelbase;
	double		nw_z, main_z;
	double		max_nw_angle;
} acf_t;

typedef enum {
	SEG_TYPE_STRAIGHT,
	SEG_TYPE_TURN
} seg_type_t;

typedef struct {
	seg_type_t	type;

	vect2_t		start_pos;
	double		start_hdg;
	vect2_t		end_pos;
	double		end_hdg;

	/*
	 * A backward pushback segment looks like this:
	 *         ^^  (start_hdg)
	 *      ---++  (start_pos)
	 *      ^  ||
	 *      |  ||
	 * (s1) |  ||
	 *      |  ||
	 *      v  || (r)
	 *      ---||-----+
	 *          \\    | (r)    (end_hdg)
	 *            \\  |            |
	 *              ``=============<+ (end_pos)
	 *                |             |
	 *                |<----------->|
	 *                      (s2)
	 *
	 * A towing segment is similar, but the positions of the respective
	 * segments is reversed.
	 */
	bool_t		backward;
	union {
		double		len;	/* straight segment length (meters) */
		struct {
			double	r;	/* turn radius (meters) */
			bool_t	right;	/* turn center is right or left */
		} turn;
	};

	list_node_t	node;
} seg_t;

typedef struct {
	acf_t		acf;		/* our aircraft */

	vect2_t		cur_pos;	/* current position in meters */
	double		cur_hdg;	/* current heading in degrees */
	double		cur_spd;	/* current speed in m/s */
	double		cur_t;		/* current time in seconds */

	vect2_t		last_pos;	/* cur_pos from previous run */
	double		last_hdg;	/* cur_hdg from previous run */
	double		last_spd;	/* cur_spd from previous run */
	double		last_t;		/* cur_t from previous run */

	/* deltas from last_* to cur_* */
	vect2_t		d_pos;		/* delta from last_pos to cur_pos */
	double		d_hdg;		/* delta from last_hdg to cur_hdg */
	double		d_t;		/* delta time from last_t to cur_t */

	double		last_force;
	vect2_t		turn_c_pos;

	list_t		segs;
} bp_state_t;

static struct {
	dr_t	lbrake, rbrake;
	dr_t	pbrake;
	dr_t	rot_force_N;
	dr_t	axial_force;
	dr_t	local_x, local_z;
	dr_t	hdg;
	dr_t	gs;
	dr_t	sim_time;
	dr_t	acf_mass;
	dr_t	tire_z;
	dr_t	nw_steerdeg1;
	dr_t	tire_steer_cmd;
	dr_t	override_steer;
} drs;

static bool_t inited = B_FALSE;
static bp_state_t bp;

static int
compute_segs(const acf_t *acf, vect2_t start_pos, double start_hdg,
    vect2_t end_pos, double end_hdg, list_t *segs)
{
	seg_t *s1, *s2;
	vect2_t turn_edge, s1_v, s2_v;
	vect2_t s2e_v = vect2_sub(end_pos, start_pos);
	double rhdg = rel_hdg(start_hdg, dir2hdg(s2e_v));
	double min_radius, l1, l2, r;
	bool_t backward = (fabs(rhdg) > 90);

	/* If the start & end positions overlap, no operation is required */
	if (VECT2_EQ(start_pos, end_pos))
		return (start_hdg == end_hdg ? 0 : -1);

	/*
	 * If the amount of heading change is zero, just project the desired
	 * end point onto a straight vector from our starting position and
	 * construct a single straight segment to reach that point.
	 */
	if (start_hdg == end_hdg) {
		vect2_t dir_v = hdg2dir(start_hdg + (backward ? 180 : 0));
		double len = vect2_dotprod(dir_v, s2e_v);

		end_pos = vect2_add(vect2_set_abs(dir_v, len), start_pos);

		s1 = calloc(1, sizeof (*s1));
		s1->type = SEG_TYPE_STRAIGHT;
		s1->start_pos = start_pos;
		s1->start_hdg = start_hdg;
		s1->end_pos = end_pos;
		s1->end_hdg = end_hdg;
		s1->backward = backward;
		s1->len = len;

		list_insert_tail(segs, s1);

		return (1);
	}

	s1_v = hdg2dir(start_hdg);
	if (backward)
		s1_v = vect2_neg(s1_v);
	s2_v = hdg2dir(end_hdg);
	if (!backward)
		s2_v = vect2_neg(s2_v);

	turn_edge = vect2vect_isect(s1_v, start_pos, s2_v, end_pos, B_TRUE);
	if (IS_NULL_VECT(turn_edge))
		return (-1);

	l1 = vect2_abs(vect2_sub(turn_edge, start_pos));
	l2 = vect2_abs(vect2_sub(turn_edge, end_pos));
	r = MIN(l1, l2);
	/*
	 * Compute minimum radius using less than max_nw_angl (hence
	 * SEG_TURN_MULT), to allow for some oversteering correction.
	 */
	min_radius = tan(DEG2RAD(acf->max_nw_angle * SEG_TURN_MULT)) *
	    acf->wheelbase;
	if (r < min_radius)
		return (-1);
	l1 -= r;
	l2 -= r;

	if (l1 == 0) {
		/* No initial straight segment */
		s2 = calloc(1, sizeof (*s2));
		s2->type = SEG_TYPE_STRAIGHT;
		s2->start_pos = vect2_add(end_pos, vect2_set_abs(s2_v, l2));
		s2->start_hdg = end_hdg;
		s2->end_pos = end_pos;
		s2->end_hdg = end_hdg;
		s2->backward = backward;
		s2->len = l2;

		s1 = calloc(1, sizeof (*s1));
		s1->type = SEG_TYPE_TURN;
		s1->start_pos = start_pos;
		s1->start_hdg = start_hdg;
		s1->end_pos = s2->start_pos;
		s1->end_hdg = s2->start_hdg;
		s1->backward = backward;
		s1->turn.r = r;
		s1->turn.right = (rhdg >= 0);

		list_insert_tail(segs, s1);
		list_insert_tail(segs, s2);
	} else {
		/* No final straight segment */
		s1 = calloc(1, sizeof (*s1));
		s1->type = SEG_TYPE_STRAIGHT;
		s1->start_pos = start_pos;
		s1->start_hdg = start_hdg;
		s1->end_pos = vect2_add(start_pos, vect2_set_abs(s1_v, l1));
		s1->end_hdg = start_hdg;
		s1->backward = backward;
		s1->len = l1;

		s2 = calloc(1, sizeof (*s2));
		s2->type = SEG_TYPE_TURN;
		s2->start_pos = s1->end_pos;
		s2->start_hdg = s1->end_hdg;
		s2->end_pos = end_pos;
		s2->end_hdg = end_hdg;
		s2->backward = backward;
		s2->turn.r = r;
		s2->turn.right = (rhdg >= 0);

		list_insert_tail(segs, s1);
		list_insert_tail(segs, s2);
	}

	return (2);
}

static void
turn_nosewheel(double req_angle, double rate)
{
	double rate_of_turn, steer_incr, cur_nw_angle;

	cur_nw_angle = bp_dr_getf(&drs.tire_steer_cmd);
	if (cur_nw_angle == req_angle)
		return;

	/*
	 * Modulate the steering increment to always be
	 * correctly within our rate of turn limits.
	 */
	rate_of_turn = rate * bp.d_t;
	steer_incr = MIN(ABS(req_angle - cur_nw_angle), rate_of_turn);
	cur_nw_angle += (cur_nw_angle < req_angle ? steer_incr : -steer_incr);
	bp_dr_setf(&drs.tire_steer_cmd, cur_nw_angle);
}

static void
push_at_speed(double targ_speed, double max_accel)
{
	double force_lim, force_incr, force, cur_nw_angle, accel_now, d_v;
	vect2_t force_v;

	/* Pilot pressed brake pedals or set parking brake, stop pushing */
	if (bp_dr_getf(&drs.lbrake) > BRAKE_PEDAL_THRESH ||
	    bp_dr_getf(&drs.rbrake) > BRAKE_PEDAL_THRESH ||
	    bp_dr_getf(&drs.pbrake) != 0)
		return;

	/*
	 * Multiply force limit by weight in tons - that's at most how
	 * hard we'll try to push the aircraft. This prevents us from
	 * flinging the aircraft across the tarmac in case some external
	 * factor is blocking us (like chocks).
	 */
	force_lim = FORCE_PER_TON * (bp.acf.mass / 1000);
	/*
	 * The maximum single-second force increment is 1/10 of the maximum
	 * pushback force limit. This means it'll take up to 10s for us to
	 * apply full pushback force.
	 */
	force_incr = (force_lim / 10) * bp.d_t;

	cur_nw_angle = bp_dr_getf(&drs.tire_steer_cmd);
	force = bp.last_force;
	force_v = VECT2(-bp.last_force * sin(DEG2RAD(cur_nw_angle)),
	    bp.last_force * cos(DEG2RAD(cur_nw_angle)));
	d_v = targ_speed - bp.cur_spd;
	accel_now = d_v / bp.d_t;

	bp_dr_setf(&drs.rot_force_N, bp_dr_getf(&drs.rot_force_N) -
	    force_v.x * bp.acf.nw_z);
	bp_dr_setf(&drs.axial_force, bp_dr_getf(&drs.axial_force) +
	    force_v.y);

	/* Once we have broken away, start limiting acceleration */
	if (ABS(bp.cur_spd) >= BREAKAWAY_THRESH) {
		max_accel = (d_v > 0 ? MIN(d_v, max_accel) :
		    MAX(d_v, max_accel));
	}

	/* Apply the force limit */
	if (accel_now > max_accel)
		force += force_incr;
	else if (accel_now < max_accel)
		force -= force_incr;
	force = MIN(force_lim, force);
	force = MAX(-force_lim, force);

	bp.last_force = force;
}

static void
straight_run(vect2_t s, double hdg, double speed)
{
	vect2_t c, s2c;
	double s2c_hdg, mis_hdg, increment;

	/*
	 * Here we implement trying to keep the aircraft stable in crosswinds.
	 * We deflect the nosewheel, trying to keep an imaginary point `c'
	 * along the aircraft's centerline axis and displaced from its origin
	 * point one wheelbase back (or forward, if towing) on a straight line
	 * from the start of the straight run along the run heading.
	 * Nosewheel deflection is calculated using two parameters:
	 * 1) displacement of the point from the line. The further this point
	 *    is displaced, the further we counter-steer to get it back.
	 * 2) rate of aircraft heading change (hdg_rate). We use this to dampen
	 *    the step above, so we don't yo-yo through the center point.
	 * Steering commands are given as increments to the currently commanded
	 * nosewheel deflection, rather than the absolute value. This allows us
	 * to settle into a deflected state once alignment is achieved to help
	 * continuously counter a constant crosswind.
	 */

	/* this is the point we're tring to align */
	c = vect2_add(bp.cur_pos, vect2_set_abs(hdg2dir(bp.cur_hdg),
	    (speed > 0 ? bp.acf.wheelbase : -bp.acf.wheelbase)));

	/*
	 * Calculate a direction vector pointing from s to c (or
	 * vice versa if pushing back) and transform into a heading.
	 */
	s2c = vect2_sub(c, s);
	s2c_hdg = dir2hdg(s2c);

	/*
	 * Calculate the required steering change. mis_hdg is the angle by
	 * which point `c' is deflected from the ideal straight line.
	 * (2 * d_hdg) here is applied as dampening, so if we're swinging
	 * back into alignment, we start reducing deflection, so we don't
	 * overshoot.
	 */
	mis_hdg = -rel_hdg(hdg, s2c_hdg);
	increment = (mis_hdg + (2 * bp.d_hdg) / bp.d_t) / bp.acf.max_nw_angle;

	/* Steering works in reverse when pushing back. */
	if (speed < 0)
		increment = -increment;

	/*
	 * Adjust total required deflection and limit it to our maximum
	 * allowable deflection.
	 */
	turn_nosewheel(bp_dr_getf(&drs.tire_steer_cmd) + increment,
	    STRAIGHT_STEER_RATE);
	push_at_speed(speed, STRAIGHT_ACCEL);
}

static void
turn_run(vect2_t c, double radius, bool_t right, double speed)
{
	vect2_t refpt = vect2_add(bp.cur_pos,
	    vect2_set_abs(vect2_neg(hdg2dir(bp.cur_hdg)), bp.acf.main_z));
	double act_radius = vect2_abs(vect2_sub(refpt, c));
	double d_radius = act_radius - radius;
	double turn_rate_mult = ABS(d_radius) / (bp.acf.wheelbase / 10);
	double a;

	/* Don't turn the nosewheel if we're traveling in the wrong direction */
	if ((speed > 0 && bp.cur_spd < 0) || (speed < 0 && bp.cur_spd > 0) ||
	    d_radius < 0)
		a = 0;
	else
		a = (right ? bp.acf.max_nw_angle : -bp.acf.max_nw_angle);
	turn_nosewheel(a, TURN_STEER_RATE * turn_rate_mult);
	push_at_speed(speed, TURN_ACCEL);
}

bool_t
bp_init(void)
{
	double tire_z_main[8];
	int n_main;

	bp_dr_find(&drs.lbrake, "sim/cockpit2/controls/left_brake_ratio");
	bp_dr_find(&drs.rbrake, "sim/cockpit2/controls/right_brake_ratio");
	bp_dr_find(&drs.pbrake, "sim/flightmodel/controls/parkbrake");
	bp_dr_find(&drs.rot_force_N, "sim/flightmodel/forces/N_plug_acf");
	bp_dr_find(&drs.axial_force, "sim/flightmodel/forces/faxil_plug_acf");
	bp_dr_find(&drs.local_x, "sim/flightmodel/position/local_x");
	bp_dr_find(&drs.local_z, "sim/flightmodel/position/local_z");
	bp_dr_find(&drs.hdg, "sim/flightmodel/position/psi");
	bp_dr_find(&drs.gs, "sim/flightmodel/position/groundspeed");
	bp_dr_find(&drs.sim_time, "sim/time/total_running_time_sec");
	bp_dr_find(&drs.acf_mass, "sim/flightmodel/weight/m_total");
	bp_dr_find(&drs.tire_z, "sim/flightmodel/parts/tire_z_no_deflection");
	bp_dr_find(&drs.nw_steerdeg1, "sim/aircraft/gear/acf_nw_steerdeg1");
	bp_dr_find(&drs.tire_steer_cmd,
	    "sim/flightmodel/parts/tire_steer_cmd");
	bp_dr_find(&drs.override_steer,
	    "sim/operation/override/override_wheel_steer");

	memset(&bp, 0, sizeof (bp));
	list_create(&bp.segs, sizeof (seg_t), offsetof(seg_t, node));

	bp.turn_c_pos = NULL_VECT2;

	bp.acf.mass = bp_dr_getf(&drs.acf_mass);
	bp_dr_getvf(&drs.tire_z, &bp.acf.nw_z, 0, 1);
	n_main = bp_dr_getvf(&drs.tire_z, tire_z_main, 1, 8);
	if (n_main < 1) {
		/* Huh? Aircraft only has one gear leg?! */
		logMsg("Aircraft only has one gear leg?!");
		return (B_FALSE);
	}
	for (int i = 0; i < n_main; i++)
		bp.acf.main_z += tire_z_main[i];
	bp.acf.main_z /= n_main;
	bp.acf.wheelbase = bp.acf.main_z - bp.acf.nw_z;
	if (bp.acf.wheelbase <= 0) {
		logMsg("Aircraft has non-positive wheelbase. "
		    "Sorry, tail draggers aren't supported.");
		return (B_FALSE);
	}
	bp.acf.max_nw_angle = bp_dr_getf(&drs.nw_steerdeg1);

	inited = B_TRUE;

	return (B_TRUE);
}

void
bp_fini(void)
{
	if (!inited)
		return;

	for (seg_t *s = list_head(&bp.segs); s != NULL; s = list_head(&bp.segs))
		free(s);
	list_destroy(&bp.segs);

	inited = B_FALSE;
}

bool_t
bp_run(void)
{
	seg_t *seg;

	bp.cur_pos = VECT2(bp_dr_getf(&drs.local_x),
	    bp_dr_getf(&drs.local_z));
	bp.cur_hdg = bp_dr_getf(&drs.hdg);
	bp.cur_t = bp_dr_getf(&drs.sim_time);
	bp.cur_spd = bp_dr_getf(&drs.gs);

	bp.d_pos = vect2_sub(bp.cur_pos, bp.last_pos);
	bp.d_hdg = bp.cur_hdg - bp.last_hdg;
	bp.d_t = bp.cur_t - bp.last_t;

	bp_dr_seti(&drs.override_steer, 1);

	while ((seg = list_head(&bp.segs)) != NULL) {
		if (seg->type == SEG_TYPE_STRAIGHT) {
			double len = vect2_abs(vect2_sub(bp.cur_pos,
			    seg->start_pos));
			if (len >= seg->len) {
				list_remove(&bp.segs, seg);
				free(seg);
				continue;
			}
			if (!seg->backward) {
				straight_run(seg->start_pos,
				    -seg->start_hdg, STRAIGHT_SPEED);
			} else {
				straight_run(seg->start_pos,
				    seg->start_hdg, -STRAIGHT_SPEED);
			}
		} else {
			vect2_t c;
			if (fabs(rel_hdg(bp.cur_hdg, seg->end_hdg)) <
			    TURN_COMPLETE_THRESH) {
				list_remove(&bp.segs, seg);
				free(seg);
				continue;
			}
			/*
			 * `c' is the center of the turn. Displace it
			 * at right angle to start_hdg at start_pos by
			 * the turn radius.
			 */
			c = vect2_add(vect2_set_abs(vect2_norm(hdg2dir(
			    seg->start_hdg), seg->turn.right), seg->turn.r),
			    seg->start_pos);
			turn_run(c, seg->turn.r, seg->turn.right, TURN_SPEED);
		}
		break;
	}

	bp.last_pos = bp.cur_pos;
	bp.last_hdg = bp.cur_hdg;
	bp.last_t = bp.cur_t;
	bp.last_spd = bp.cur_spd;

	return (seg != NULL);
}

void
bp_test_setup(void)
{
	bp.cur_pos = VECT2(bp_dr_getf(&drs.local_x),
	    bp_dr_getf(&drs.local_z));
	bp.cur_hdg = bp_dr_getf(&drs.hdg);
	bp.cur_t = bp_dr_getf(&drs.sim_time);
	bp.cur_spd = bp_dr_getf(&drs.gs);

	compute_segs(&bp.acf, bp.cur_pos, bp.cur_hdg,
	    vect2_add(bp.cur_pos, vect2_set_abs(hdg2dir(bp.cur_hdg), 50)),
	    bp.cur_hdg, &bp.segs);
}
