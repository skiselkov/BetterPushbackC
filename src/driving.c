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

#include <stddef.h>
#include <string.h>
#include <errno.h>

#include <acfutils/avl.h>
#include <acfutils/helpers.h>
#include <acfutils/math.h>

#include <XPLMGraphics.h>
#include <XPLMScenery.h>
#include <XPLMUtilities.h>

#include "driving.h"
#include "xplane.h"

#define	SEG_TURN_MULT		0.9	/* leave 10% for oversteer */
#define	SPEED_COMPLETE_THRESH	0.05	/* m/s */
#define	MIN_TURN_RADIUS		1.5	/* in case the aircraft is tiny */
#define	MIN_STEERING_ARM_LEN	4	/* meters */
#define	MAX_OFF_PATH_ANGLE	20	/* degrees */
#define	OFF_PATH_CORR_ANGLE	35	/* degrees */
#define	STEERING_SENSITIVE	90	/* degrees */
#define	MIN_SPEED_XP10		0.6	/* m/s */
#define	CRAWL_SPEED(xpversion, veh)	/* m/s */ \
	(((xpversion) >= 11000 || !veh->xp10_bug_ign) ? 0.05 : MIN_SPEED_XP10)

#define	MIN_SEG_LEN		0.1	/* meters */

#define	STEER_GATE(x, g)	MIN(MAX((x), -g), g)

#define	ROUTE_DIST_LIM		30	/* meters */
#define	ROUTE_HDG_LIM		10	/* degrees */
#define	ROUTE_TABLE_DIRS	bp_xpdir, "Output", "caches"
#define	ROUTE_TABLE_FILENAME	"BetterPushback_routes.dat"

/*
 * When constructing the oblique case, rounding errors can cause us to
 * compute a case as usable, but the construction will fail (because the
 * actual required turn radius will have dropped below the minimum). So
 * when invoking the oblique case, we inflate our minimum radius by this
 * factor to work around the rounding errors.
 */
#define	OBLIQUE_RADIUS_FACT	1.02

#define	STRAIGHT_SEG_ANGLE_LIM	1

/* Turns on aggressive debug logging. */
/*#define	DRIVING_DEBUG_LOGGING*/

/*
 * A route table is an AVL tree that holds sets of driving segments, each
 * associated with a particular starting position (first start_pos & start_hdg
 * or the first segment). This allows us to store and retrieve previously used
 * driving instructions so the user doesn't have to keep re-entering them if
 * they repeatedly push back from the same starting positions.
 * This table is stored in Output/caches/BetterPushback_routes.dat as
 * a text file. See routes_store for details on the format.
 */
typedef struct {
	geo_pos2_t	pos;		/* start geographical position */
	vect3_t		pos_ecef;	/* start position in ECEF */
	double		hdg;		/* start true heading in degrees */
	list_t		segs;
	avl_node_t	node;
} route_t;

static int compute_segs_impl(const vehicle_t *veh, vect2_t start_pos,
    double start_hdg, vect2_t end_pos, double end_hdg, list_t *segs,
    bool_t recurse);
static double turn_run_speed(const int xpversion, const vehicle_t *veh,
    list_t *segs, double rhdg, double radius, bool_t backward,
    const seg_t *next);
static double straight_run_speed(const int xpversion, const vehicle_t *veh,
    list_t *segs, double rmng_d, bool_t backward, const seg_t *next);

static int
construct_segs_oblique(const vehicle_t *veh, vect2_t start_pos,
    double start_hdg, vect2_t midpt, double mid_hdg, vect2_t end_pos,
    double end_hdg, list_t *segs)
{
	int n1, n2;

	n1 = compute_segs_impl(veh, start_pos, start_hdg, midpt, mid_hdg,
	    segs, B_FALSE);
	if (n1 == -1)
		return (-1);
	n2 = compute_segs_impl(veh, midpt, mid_hdg, end_pos, end_hdg,
	    segs, B_FALSE);
	if (n2 == -1) {
		for (int i = 0; i < n1; i++) {
			seg_t *seg = list_remove_tail(segs);
			free(seg);
		}
		return (-1);
	}

	return (n1 + n2);
}

static int
compute_segs_oblique(const vehicle_t *veh, vect2_t start_pos,
    double start_hdg, vect2_t end_pos, double end_hdg, list_t *segs,
    bool_t backward, double radius)
{
	const vect2_t d1 = hdg2dir(start_hdg), d2 = hdg2dir(end_hdg);
	const vect2_t sv = vect2_scmul(d1, backward ? -1e10 : 1e10);
	const vect2_t ev = vect2_scmul(d2, backward ? 1e10 : -1e10);

	for (int i = 0; i < 2; i++) {
		for (int j = 0; j < 2; j++) {
			vect2_t c1 = vect2_add(start_pos,
			    vect2_scmul(vect2_norm(d1, i), radius));
			vect2_t c2 = vect2_add(end_pos,
			    vect2_scmul(vect2_norm(d2, j), radius));
			vect2_t s = vect2_mean(c1, c2);
			vect2_t s2c1 = vect2_sub(c1, s);
			vect2_t s2c2 = vect2_sub(c2, s);
			double a;

			if (!IS_NULL_VECT(vect2vect_isect(s2c1, s, sv,
			    start_pos, B_TRUE)) ||
			    !IS_NULL_VECT(vect2vect_isect(s2c2, s, ev,
			    end_pos, B_TRUE)))
				continue;

			a = RAD2DEG(asin(radius / vect2_abs(s2c1)));
			if (isnan(a))
				continue;

			if (!IS_NULL_VECT(vect2vect_isect(vect2_set_abs(
			    vect2_rot(s2c1, a), 1e10), s, sv, start_pos,
			    B_TRUE)) &&
			    !IS_NULL_VECT(vect2vect_isect(vect2_set_abs(
			    vect2_rot(s2c2, a), 1e10), s, ev, end_pos,
			    B_TRUE))) {
				int n = construct_segs_oblique(veh, start_pos,
				    start_hdg, s, dir2hdg(vect2_rot(backward ?
				    s2c1 : s2c2, a)), end_pos, end_hdg, segs);
				if (n != -1)
					return (n);
			}

			if (!IS_NULL_VECT(vect2vect_isect(vect2_set_abs(
			    vect2_rot(s2c1, -a), 1e10), s, sv, start_pos,
			    B_TRUE)) &&
			    !IS_NULL_VECT(vect2vect_isect(vect2_set_abs(
			    vect2_rot(s2c2, -a), 1e10), s, ev, end_pos,
			    B_TRUE))) {
				int n = construct_segs_oblique(veh, start_pos,
				    start_hdg, s, dir2hdg(vect2_rot(backward ?
				    s2c1 : s2c2, -a)), end_pos, end_hdg, segs);
				if (n != -1)
					return (n);
			}
		}
	}

	return (-1);
}

static int
compute_segs_impl(const vehicle_t *veh, vect2_t start_pos, double start_hdg,
    vect2_t end_pos, double end_hdg, list_t *segs, bool_t recurse)
{
	seg_t *s1, *s2;
	vect2_t turn_edge, s1_v, s2_v, s2e_v;
	double rhdg, min_radius, l1, l2, x, a, r;
	bool_t backward;

	/* If the start & end positions overlap, no operation is required */
	if (vect2_dist(start_pos, end_pos) < MIN_SEG_LEN) {
		if (ABS(start_hdg - end_hdg) < STRAIGHT_SEG_ANGLE_LIM)
			return (0);
		else
			return (-1);
	}
	s2e_v = vect2_sub(end_pos, start_pos);
	rhdg = rel_hdg(start_hdg, dir2hdg(s2e_v));
	backward = (fabs(rhdg) > 90);

	/*
	 * Compute minimum radius using less than max_steer (hence
	 * SEG_TURN_MULT), to allow for some oversteering correction.
	 * Also limit the radius to something sensible (MIN_TURN_RADIUS).
	 */
	min_radius = MAX(tan(DEG2RAD(90 - (veh->max_steer * SEG_TURN_MULT))) *
	    veh->wheelbase, MIN_TURN_RADIUS);

	/*
	 * If the amount of heading change is tiny, just project the desired
	 * end point onto a straight vector from our starting position and
	 * construct a single straight segment to reach that point.
	 */
	if (fabs(start_hdg - end_hdg) < STRAIGHT_SEG_ANGLE_LIM &&
	    (ABS(rhdg) < STRAIGHT_SEG_ANGLE_LIM ||
	    ABS(rhdg) > 180 - STRAIGHT_SEG_ANGLE_LIM)) {
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

	s1_v = vect2_scmul(hdg2dir(start_hdg), 1e10);
	if (backward)
		s1_v = vect2_neg(s1_v);
	s2_v = vect2_scmul(hdg2dir(end_hdg), 1e10);
	if (!backward)
		s2_v = vect2_neg(s2_v);

	turn_edge = vect2vect_isect(s1_v, start_pos, s2_v, end_pos, B_TRUE);
	if (IS_NULL_VECT(turn_edge)) {
		if (recurse) {
			return (compute_segs_oblique(veh, start_pos, start_hdg,
			    end_pos, end_hdg, segs, backward, min_radius *
			    OBLIQUE_RADIUS_FACT));
		} else {
			return (-1);
		}
	}

	l1 = vect2_dist(turn_edge, start_pos);
	l2 = vect2_dist(turn_edge, end_pos);
	x = MIN(l1, l2);
	l1 -= x;
	l2 -= x;

	a = (180 - ABS(rel_hdg(start_hdg, end_hdg)));
	r = x * tan(DEG2RAD(a / 2));
	if (r < min_radius) {
		if (recurse)
			return (compute_segs_oblique(veh, start_pos, start_hdg,
			    end_pos, end_hdg, segs, backward, min_radius *
			    OBLIQUE_RADIUS_FACT));
		else
			return (-1);
	}
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
	}

	list_insert_tail(segs, s1);
	list_insert_tail(segs, s2);

	return (2);
}

int
compute_segs(const vehicle_t *veh, vect2_t start_pos, double start_hdg,
    vect2_t end_pos, double end_hdg, list_t *segs)
{
	return (compute_segs_impl(veh, start_pos, start_hdg, end_pos,
	    end_hdg, segs, B_TRUE));
}

/*
 * Computes the absolute position where the vehicle's fixed (rear) axle
 * crosses its longitudinal axis. This is the starting point for all steering.
 */
static vect2_t
veh_pos2fixed_axle(const vehicle_pos_t *pos, const vehicle_t *veh)
{
	return (vect2_add(pos->pos, vect2_scmul(hdg2dir(pos->hdg),
	    veh->fixed_z_off)));
}

static void
drive_on_line(const vehicle_pos_t *pos, const vehicle_t *veh,
    vect2_t line_start, double line_hdg, double speed, double arm_len,
    double steer_corr_amp, double *last_mis_hdg, double d_t,
    double *steer_out, double *speed_out)
{
	vect2_t c, s2c, align_s, dir_v, fixed_pos;
	double s2c_hdg, mis_hdg, steering_arm, turn_radius, ang_vel, rhdg;
	double cur_hdg, steer, d_mis_hdg;
	bool_t overcorrecting = B_FALSE;

	cur_hdg = (speed >= 0 ? pos->hdg : normalize_hdg(pos->hdg + 180));
	fixed_pos = veh_pos2fixed_axle(pos, veh);

	/* Neutralize steering until we're traveling in our direction */
	if ((speed < 0 && pos->spd > 0) || (speed > 0 && pos->spd < 0)) {
		*steer_out = 0;
		*speed_out = speed;
		return;
	}

	/* this is the point we're tring to align */
	steering_arm = MAX(arm_len, MIN_STEERING_ARM_LEN);
	c = vect2_add(fixed_pos, vect2_scmul(hdg2dir(cur_hdg), steering_arm));

	/*
	 * We project our position onto the ideal straight line. Limit the
	 * projection backwards to be at least 1m ahead, otherwise we might
	 * steer in the opposite sense than we want.
	 */
	dir_v = hdg2dir(line_hdg);
	align_s = vect2_add(line_start, vect2_scmul(dir_v,
	    vect2_dotprod(vect2_sub(fixed_pos, line_start), dir_v)));

	/*
	 * Calculate a direction vector pointing from s to c (or
	 * vice versa if pushing back) and transform into a heading.
	 */
	s2c = vect2_sub(c, align_s);
	s2c_hdg = dir2hdg(s2c);

	mis_hdg = rel_hdg(s2c_hdg, line_hdg);
	rhdg = rel_hdg(cur_hdg, line_hdg);
	d_mis_hdg = (mis_hdg - (*last_mis_hdg)) / d_t;
	UNUSED(d_mis_hdg);

	/*
	 * Calculate the required steering change. mis_hdg is the angle by
	 * which point `c' is deflected from the ideal straight line. So
	 * simply steer in the opposite direction to try and nullify it.
	 */
	steer = STEER_GATE(mis_hdg + rhdg, veh->max_steer);

	/*
	 * Watch out for overcorrecting. If our heading is too far in the
	 * opposite direction, limit our relative angle to the desired path
	 * angle to MAX_OFF_PATH_ANGLE and steer that way until we get back
	 * on track.
	 */
	if (mis_hdg < 0 && rhdg > MAX_OFF_PATH_ANGLE) {
		steer = STEER_GATE(rhdg - OFF_PATH_CORR_ANGLE, veh->max_steer);
		overcorrecting = B_TRUE;
	} else if (mis_hdg > 0 && rhdg < -MAX_OFF_PATH_ANGLE) {
		steer = STEER_GATE(rhdg + OFF_PATH_CORR_ANGLE, veh->max_steer);
		overcorrecting = B_TRUE;
	}

	/*
	 * If we've come off the path even with overcorrection, slow down
	 * until we're re-established again.
	 */
	if (overcorrecting) {
		speed = MAX(MIN(speed, veh->max_rev_spd), -veh->max_rev_spd);
	}

	/*
	 * Limit our speed to not overstep maximum angular velocity for
	 * a correction maneuver. This helps in case we get kicked off
	 * from a straight line very far and need to correct a lot.
	 */
	turn_radius = tan(DEG2RAD(90 - ABS(steer))) * veh->wheelbase;
	ang_vel = RAD2DEG(ABS(speed) / turn_radius);
	if (speed >= 0)
		speed *= MIN(veh->max_fwd_ang_vel / ang_vel, 1);
	else
		speed *= MIN(veh->max_rev_ang_vel / ang_vel, 1);

	/* Steering works in reverse when pushing back. */
	if (speed < 0)
		steer = -steer;

#ifdef	DRIVING_DEBUG_LOGGING
	printf("mis_hdg: %5.1f d_mis_hdg: %5.1f rhdg: %5.1f arm: %3.1f oc:%d "
	    "st:%5.1f\n", mis_hdg, d_mis_hdg, rhdg, steering_arm,
	    overcorrecting, steer);
#endif	/* DRIVING_DEBUG_LOGGING */

	/*
	 * At very short wheelbases, the steering correction amplification is
	 * causing more trouble than good. So gradually reduce it out as the
	 * wheelbase goes from MIN_STEERING_ARM_LEN to 0.1m.
	 */

	if (veh->wheelbase < MIN_STEERING_ARM_LEN) {
		steer_corr_amp = fx_lin(veh->wheelbase, 0.5, 1,
		    MIN_STEERING_ARM_LEN, steer_corr_amp);
	}

	*steer_out = steer * steer_corr_amp;
	*speed_out = speed;

	*last_mis_hdg = mis_hdg;
}

static double
next_seg_speed(const int xpversion, const vehicle_t *veh, list_t *segs,
    const seg_t *next, bool_t cur_backward)
{
	if (next != NULL && next->backward == cur_backward) {
		if (next->type == SEG_TYPE_STRAIGHT) {
			return (straight_run_speed(xpversion, veh, segs,
			    next->len, next->backward,list_next(segs, next)));
		} else {
			return (turn_run_speed(xpversion, veh, segs,
			    rel_hdg(next->start_hdg, next->end_hdg),
			    next->turn.r, next->backward,
			    list_next(segs, next)));
		}
	} else {
		/*
		 * At the end of the operation or when reversing direction,
		 * target a nearly stopped speed.
		 */
		return (CRAWL_SPEED(xpversion, veh));
	}
}

/*
 * Estimates the speed we want to achieve during a turn run. This basically
 * treats the circle we're supposed to travel as if it were a straight line
 * (thus employing the straight_run_speed algorithm), but limits the maximum
 * angular velocity around the circle to max_{fwd,rev}_ang_vel to limit
 * side-loading. This means the tighter the turn, the slower our speed.
 */
static double
turn_run_speed(const int xpversion, const vehicle_t *veh, list_t *segs,
    double rhdg, double radius, bool_t backward, const seg_t *next)
{
	double rmng_d = (2 * M_PI * radius) * (rhdg / 360.0);
	double spd = straight_run_speed(xpversion, veh, segs, rmng_d,
	    backward, next);
	double rmng_t = rmng_d / spd;
	double ang_vel = rhdg / rmng_t;

	if (!backward)
		spd *= MIN(veh->max_fwd_ang_vel / ang_vel, 1);
	else
		spd *= MIN(veh->max_rev_ang_vel / ang_vel, 1);
	if (xpversion < 11000 && !veh->xp10_bug_ign) {
		/*
		 * X-Plane 10's tire model is much sticker, so don't slow down
		 * too much or we are going to stick to the ground.
		 */
		if (spd > 0 && spd < MIN_SPEED_XP10)
			spd = MIN_SPEED_XP10;
		else if (spd < 0 && spd > -MIN_SPEED_XP10)
			spd = -MIN_SPEED_XP10;
	}

	return (spd);
}

static double
straight_run_speed(const int xpversion, const vehicle_t *veh, list_t *segs,
    double rmng_d, bool_t backward, const seg_t *next)
{
	double next_spd, cruise_spd, spd, crawl_spd;
	double ts[2];

	next_spd = next_seg_speed(xpversion, veh, segs, next, backward);
	cruise_spd = (backward ? veh->max_rev_spd : veh->max_fwd_spd);
	crawl_spd = CRAWL_SPEED(bp_xp_ver, veh);

	if (rmng_d < crawl_spd)
		return (MAX(next_spd, crawl_spd));

	/*
	 * Pretend we have less distance left so as to reach our target speed
	 * 1-2 seconds ahead of entering the next segment. The purpose of this
	 * is to help prevent an overshoot on the last segment (so we stop
	 * almost exactly where we wanted to). This is because the deceleration
	 * always trails our desired speed by a little, so when ending the tow,
	 * we always end up going a little bit beyond our target.
	 */
	if (rmng_d > crawl_spd)
		rmng_d -= crawl_spd;

	/*
	 * This algorithm works as follows:
	 * We know the remaining distance and the next segment's target
	 * speed. So we work backwards to determine what maximum speed
	 * we could be going in order to hit next_spd using NORMAL_DECEL.
	 *
	 *          (speed)
	 *          ^
	 * max ---> |
	 * spd      |\       (NORMAL_DECEL slope)
	 *          |  \    /
	 *          |    \ V
	 *          |      \
	 *          |        \
	 *          |          \
	 *          |           + <--- next_spd
	 *          |           |
	 *          +-----------+------------->
	 *          |   rmng_d  |    (distance)
	 *          |<--------->|
	 *
	 * Here's the general equation for acceleration:
	 *
	 * d = 1/2at^2 + vt
	 *
	 * Where:
	 *	'd' = rmng_d
	 *	'a' = NORMAL_DECEL
	 *	'v' = next_spd
	 *	't' = <unknown>
	 *
	 * This is a simple quadratic equation (1/2at^2 + vt - d = 0), so
	 * we can solve for the only unknown, time 't'. If we have two
	 * results, taking greater value i.e. the one lying in the future,
	 * we simply calculate the initial max_spd = next_spd + at. This
	 * is our theoretical maximum. Taking the lesser of that and the
	 * target cruise speed, we arrive at our final governed speed `spd'.
	 */
	switch (quadratic_solve(0.5 * veh->max_decel, next_spd, -rmng_d, ts)) {
	case 1:
		spd = MIN(veh->max_decel * ts[0] + next_spd, cruise_spd);
		break;
	case 2:
		spd = MIN(veh->max_decel * MAX(ts[0], ts[1]) + next_spd,
		    cruise_spd);
		break;
	default:
		spd = next_spd;
		break;
	}

	return (spd);
}

static void
turn_run(const vehicle_pos_t *pos, const vehicle_t *veh, const seg_t *seg,
    double *last_mis_hdg, double d_t, double speed, double *out_steer,
    double *out_speed)
{
	double start_hdg = seg->start_hdg;
	double end_hdg = seg->end_hdg;
	vect2_t c2r, r, dir_v;
	double hdg, cur_radial, start_radial, end_radial;
	bool_t cw = ((seg->turn.right && !seg->backward) ||
	    (!seg->turn.right && seg->backward));
	vect2_t fixed_pos = veh_pos2fixed_axle(pos, veh);

	/*
	 * `c' is the center of the turn. Displace it at right angle to
	 * start_hdg at start_pos by the turn radius.
	 */
	vect2_t c = vect2_add(vect2_set_abs(vect2_norm(hdg2dir(start_hdg),
	    seg->turn.right), seg->turn.r), seg->start_pos);

	c2r = vect2_set_abs(vect2_sub(fixed_pos, c), seg->turn.r);
	cur_radial = dir2hdg(c2r);
	r = vect2_add(c, c2r);
	dir_v = vect2_norm(c2r, seg->turn.right);
	start_radial = normalize_hdg(start_hdg + (seg->turn.right ? -90 : 90));
	end_radial = normalize_hdg(end_hdg + (seg->turn.right ? -90 : 90));
	if (is_on_arc(cur_radial, start_radial, end_radial, cw)) {
		hdg = dir2hdg(dir_v);
	} else if (fabs(rel_hdg(cur_radial, start_radial)) <
	    fabs(rel_hdg(cur_radial, end_radial))) {
		hdg = start_hdg;
	} else {
		hdg = end_hdg;
	}
	if (seg->backward)
		hdg = normalize_hdg(hdg + 180);

	speed = (!seg->backward ? speed : -speed);
	drive_on_line(pos, veh, r, hdg, speed, veh->wheelbase / 5,
	    3, last_mis_hdg, d_t, out_steer, out_speed);
}

bool_t
drive_segs(const vehicle_pos_t *pos, const vehicle_t *veh, list_t *segs,
    double *last_mis_hdg, double d_t, double *out_steer, double *out_speed)
{
	seg_t *seg = list_head(segs);
	int xpversion;

	XPLMGetVersions(&xpversion, NULL, NULL);

	ASSERT(seg != NULL);
	if (seg->type == SEG_TYPE_STRAIGHT) {
		vect2_t dir = !seg->backward ? hdg2dir(seg->start_hdg) :
		    vect2_neg(hdg2dir(seg->start_hdg));
		double len = vect2_dotprod(vect2_sub(pos->pos, seg->start_pos),
		    dir);
		double speed = straight_run_speed(xpversion, veh, segs,
		    seg->len - len, seg->backward, list_next(segs, seg));
		double hdg = (!seg->backward ? seg->start_hdg :
		    normalize_hdg(seg->start_hdg + 180));

		if (len >= seg->len) {
			list_remove(segs, seg);
			free(seg);
			return (B_FALSE);
		}

		speed = (!seg->backward ? speed : -speed);
		drive_on_line(pos, veh, seg->start_pos, hdg, speed,
		    veh->wheelbase / 2, 3, last_mis_hdg, d_t, out_steer,
		    out_speed);
	} else {
		double rhdg = fabs(rel_hdg(pos->hdg, seg->end_hdg));
		double end_hdg = (!seg->backward ? seg->end_hdg :
		    normalize_hdg(seg->end_hdg + 180));
		double end_brg = fabs(rel_hdg(end_hdg, dir2hdg(
		    vect2_sub(pos->pos, seg->end_pos))));
		double speed = turn_run_speed(xpversion, veh, segs, ABS(rhdg),
		    seg->turn.r, seg->backward, list_next(segs, seg));

		/*
		 * Segment complete when we are past the end_pos point
		 * (delta between end_hdg and a vector from end_pos to
		 * cur_pos is <= 90 degrees) and we are pointing at least
		 * roughly in the correct direction (otherwise we might
		 * complete >180 degree turns too early).
		 */
		if (end_brg < 90 && rhdg < 90) {
			list_remove(segs, seg);
			free(seg);
			return (B_FALSE);
		}
		turn_run(pos, veh, seg, last_mis_hdg, d_t, speed,
		    out_steer, out_speed);
	}

	/* limit desired steering */
	*out_steer = STEER_GATE(*out_steer, veh->max_steer);

	return (B_TRUE);
}

/* Converts a seg_t from using geographic to local coordinates */
void
seg_world2local(seg_t *seg)
{
	double unused;

	if (seg->have_local_coords)
		return;

	XPLMWorldToLocal(seg->start_pos_geo.lat, seg->start_pos_geo.lon, 0,
	    &seg->start_pos.x, &unused, &seg->start_pos.y);
	XPLMWorldToLocal(seg->end_pos_geo.lat, seg->end_pos_geo.lon, 0,
	    &seg->end_pos.x, &unused, &seg->end_pos.y);
	/* X-Plane's Z axis is flipped to ours */
	seg->start_pos.y = -seg->start_pos.y;
	seg->end_pos.y = -seg->end_pos.y;
	seg->have_local_coords = B_TRUE;
}

/* Converts a seg_t from using local to geographic coordinates */
void
seg_local2world(seg_t *seg)
{
	double unused;
	XPLMProbeRef probe;
	XPLMProbeInfo_t info = { .structSize = sizeof (XPLMProbeInfo_t) };

	if (seg->have_world_coords)
		return;

	probe = XPLMCreateProbe(xplm_ProbeY);

	/* X-Plane's Z axis is flipped to ours */
	VERIFY3U(XPLMProbeTerrainXYZ(probe, seg->start_pos.x, 0,
	    -seg->start_pos.y, &info), ==, xplm_ProbeHitTerrain);
	XPLMLocalToWorld(seg->start_pos.x, info.locationY, -seg->start_pos.y,
	    &seg->start_pos_geo.lat, &seg->start_pos_geo.lon, &unused);

	VERIFY3U(XPLMProbeTerrainXYZ(probe, seg->end_pos.x, 0,
	    -seg->end_pos.y, &info), ==, xplm_ProbeHitTerrain);
	XPLMLocalToWorld(seg->end_pos.x, info.locationY, -seg->end_pos.y,
	    &seg->end_pos_geo.lat, &seg->end_pos_geo.lon, &unused);

	seg->have_world_coords = B_TRUE;
	XPLMDestroyProbe(probe);
}

static void
route_free(route_t *r)
{
	seg_t *seg;
	while ((seg = list_remove_head(&r->segs)) != NULL)
		free(seg);
	list_destroy(&r->segs);
	free(r);
}

static void
route_seg_append(avl_tree_t *route_table, route_t *r, const seg_t *seg)
{
	seg_t *seg2 = calloc(1, sizeof (*seg2));

	memcpy(seg2, seg, sizeof (*seg2));
	seg_local2world(seg2);
	/* first segment appended completes the route start pos & hdg */
	if (list_head(&r->segs) == NULL) {
		route_t *r2;

		r->pos = seg2->start_pos_geo;
		r->pos_ecef = geo2ecef(GEO_POS3(r->pos.lat,
		    r->pos.lon, 0), &wgs84);
		r->hdg = seg2->start_hdg;

		while ((r2 = avl_find(route_table, r, NULL)) != NULL) {
			avl_remove(route_table, r2);
			route_free(r2);
		}
		avl_add(route_table, r);
	}
	list_insert_tail(&r->segs, seg2);
}

static route_t *
route_alloc(avl_tree_t *route_table, const list_t *segs)
{
	route_t *r = calloc(1, sizeof (*r));

	r->pos = NULL_GEO_POS2;
	r->pos_ecef = NULL_VECT3;
	r->hdg = NAN;
	list_create(&r->segs, sizeof (seg_t), offsetof(seg_t, node));
	if (segs != NULL) {
		ASSERT(list_head(segs) != NULL);
		for (const seg_t *seg = list_head(segs); seg != NULL;
		    seg = list_next(segs, seg)) {
			route_seg_append(route_table, r, seg);
		}
	}

	return (r);
}

static int
route_table_compar(const void *a, const void *b)
{
	const route_t *r1 = a, *r2 = b;
	double dist, rhdg;

	ASSERT(!IS_NULL_VECT(r1->pos_ecef));
	ASSERT(!IS_NULL_VECT(r2->pos_ecef));
	ASSERT(!isnan(r1->hdg));
	ASSERT(!isnan(r2->hdg));

	dist = vect3_dist(r1->pos_ecef, r2->pos_ecef);
	rhdg = fabs(rel_hdg(r1->hdg, r2->hdg));

	if (dist <= ROUTE_DIST_LIM && rhdg <= ROUTE_HDG_LIM) {
		return (0);
	} else if ((r1->pos.lat * 1000 + r1->pos.lon) * 1000 + r1->hdg <
	    (r2->pos.lat * 1000 + r2->pos.lon) * 1000 + r2->hdg) {
		return (-1);
	} else {
		return (1);
	}
}

static avl_tree_t *
routes_load(void)
{
	char *filename = mkpathname(ROUTE_TABLE_DIRS, ROUTE_TABLE_FILENAME,
	    NULL);
	FILE *fp = fopen(filename, "r");
	route_t *r = NULL;
	avl_tree_t *t = calloc(1, sizeof (*t));

	avl_create(t, route_table_compar, sizeof (route_t),
	    offsetof(route_t, node));

	if (fp == NULL)
		goto out;

	while (!feof(fp)) {
		char word[64];
		if (fscanf(fp, "%63s", word) != 1)
			continue;
		if (*word == '#') {
			while (fgetc(fp) != '\n' && !feof(fp))
				;
			continue;
		}
		if (strcmp(word, "route") == 0) {
			if (r != NULL && list_head(&r->segs) == NULL)
				goto out;
			r = route_alloc(NULL, NULL);
		} else if (strcmp(word, "seg") == 0) {
			seg_t seg;

			memset(&seg, 0, sizeof (seg));
			if (r == NULL) {
				logMsg("Error parsing %s: 'seg' keyword must "
				    "follow a 'route' keyword.", filename);
				goto out;
			}
			if (fscanf(fp, "%u", &seg.type) != 1 ||
			    seg.type > SEG_TYPE_TURN) {
				logMsg("Error parsing %s: missing or bad "
				    "segment type following 'seg' keyword",
				    filename);
				goto out;
			}
			seg.have_world_coords = B_TRUE;
			if (fscanf(fp, "%lf %lf %lf %lf %lf %lf %u",
			    &seg.start_pos_geo.lat, &seg.start_pos_geo.lon,
			    &seg.start_hdg, &seg.end_pos_geo.lat,
			    &seg.end_pos_geo.lon, &seg.end_hdg,
			    &seg.backward) != 7 ||
			    !is_valid_hdg(seg.start_hdg) ||
			    !is_valid_hdg(seg.end_hdg)) {
				logMsg("Error parsing %s: bad coordinates "
				    "following 'seg' keyword", filename);
				goto out;
			}
			switch (seg.type) {
			case SEG_TYPE_STRAIGHT: {
				vect3_t start_ecef = geo2ecef(GEO_POS3(
				    seg.start_pos_geo.lat,
				    seg.start_pos_geo.lon, 0), &wgs84);
				vect3_t end_ecef = geo2ecef(GEO_POS3(
				    seg.end_pos_geo.lat,
				    seg.end_pos_geo.lon, 0), &wgs84);
				seg.len = vect3_dist(start_ecef, end_ecef);
				if (fscanf(fp, "%u", &seg.user_placed) != 1) {
					logMsg("Error parsing %s: bad length "
					    "following 'seg 0' keyword",
					    filename);
					goto out;
				}
				break;
			}
			case SEG_TYPE_TURN:
				if (fscanf(fp, "%lf %u %u", &seg.turn.r,
				    &seg.turn.right, &seg.user_placed) != 3) {
					logMsg("Error parsing %s: bad turn "
					    "info following 'seg 1' keyword",
					    filename);
					goto out;
				}
				break;
			}
			route_seg_append(t, r, &seg);
		} else {
			logMsg("Error parsing %s: unrecognized keyword '%s'",
			    filename, word);
			goto out;
		}
	}

out:
	if (r != NULL && list_head(&r->segs) == NULL) {
		logMsg("Error parsing %s: found route with no segments",
		    filename);
		route_free(r);
	}
	free(filename);
	if (fp != NULL)
		fclose(fp);

	return (t);
}

/*
 * Saves a driving segment table to the standard database location at
 * Output/caches/BetterPushback_segs_table.dat, creating the intermediate
 * directories as necessary.
 */
static bool_t
routes_store(avl_tree_t *t)
{
	char *dirname = mkpathname(ROUTE_TABLE_DIRS, NULL);
	char *filename;
	bool_t isdir;
	FILE *fp;

	if (!file_exists(dirname, &isdir) || !isdir) {
		if (!create_directory(dirname))
			return (B_FALSE);
	}
	free(dirname);
	filename = mkpathname(ROUTE_TABLE_DIRS, ROUTE_TABLE_FILENAME, NULL);
	fp = fopen(filename, "w");
	if (fp == NULL) {
		logMsg("Error writing file %s: %s", filename, strerror(errno));
		free(filename);
		return (B_FALSE);
	}

	fprintf(fp, "### This is the BetterPushback segment table ###\n"
	    "### This file is automatically generated. DO NOT EDIT! ###\n");

	for (route_t *r = avl_first(t); r != NULL; r = AVL_NEXT(t, r)) {
		fprintf(fp, "\nroute\n");
		for (seg_t *seg = list_head(&r->segs); seg != NULL;
		    seg = list_next(&r->segs, seg)) {
			ASSERT(seg->have_world_coords);
			fprintf(fp, "  seg %u %.17f %.17f %.1f %.17f %.17f "
			    "%.1f %u ",
			    seg->type, seg->start_pos_geo.lat,
			    seg->start_pos_geo.lon, seg->start_hdg,
			    seg->end_pos_geo.lat,
			    seg->end_pos_geo.lon, seg->end_hdg, seg->backward);
			if (seg->type == SEG_TYPE_STRAIGHT) {
				fprintf(fp, "%u\n", seg->user_placed);
			} else {
				ASSERT3U(seg->type, ==, SEG_TYPE_TURN);
				fprintf(fp, "%.3f %u %u\n", seg->turn.r,
				    seg->turn.right, seg->user_placed);
			}
		}
	}

	free(filename);
	fclose(fp);

	return (B_TRUE);
}

static void
routes_free(avl_tree_t *t)
{
	route_t *r;
	void *cookie = NULL;

	while ((r = avl_destroy_nodes(t, &cookie)) != NULL)
		route_free(r);
	avl_destroy(t);
	free(t);
}

/*
 * Given a start position, heading and driving segments, associates the
 * segments with the start position and saves them persistently driving
 * for later reuse via segs_load.
 */
void
route_save(const list_t *segs)
{
	avl_tree_t *t;

	ASSERT(list_head(segs) != NULL);

	t = routes_load();
	(void) route_alloc(t, segs);
	(void) routes_store(t);
	routes_free(t);
}

/*
 * Given a start position and heading, attempts to reload the driving
 * segments used last from this position. The resulting segments are
 * placed in `segs'. If no suitable driving segments for the given
 * position were found, the list is left unmodified. The `segs' list
 * must be empty when calling this function.
 */
void
route_load(geo_pos2_t start_pos, double start_hdg, list_t *segs)
{
	avl_tree_t *t;
	route_t srch, *r;

	ASSERT3P(list_head(segs), ==, NULL);

	t = routes_load();

	srch.pos = start_pos;
	srch.pos_ecef = geo2ecef(GEO_POS3(start_pos.lat, start_pos.lon, 0),
	    &wgs84);
	srch.hdg = start_hdg;

	r = avl_find(t, &srch, NULL);
	if (r != NULL) {
		for (seg_t *seg = list_head(&r->segs); seg != NULL;
		    seg = list_next(&r->segs, seg)) {
			seg_t *seg2 = calloc(1, sizeof (*seg2));
			memcpy(seg2, seg, sizeof (*seg2));
			seg_world2local(seg2);
			list_insert_tail(segs, seg2);
		}
	}

	routes_free(t);
}
