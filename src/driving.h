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

#ifndef	_DRIVING_H_
#define	_DRIVING_H_

#include <acfutils/list.h>
#include <acfutils/geom.h>

#ifdef	__cplusplus
extern "C" {
#endif

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

	/*
	 * Flag indicating if the user placed this segment. Non-user-placed
	 * segments (up to the last user-placed segment) are deleted from
	 * the segment list.
	 */
	bool_t		user_placed;

	list_node_t	node;
} seg_t;

int compute_segs(double wheelbase, double max_nw_angle,
    vect2_t start_pos, double start_hdg,
    vect2_t end_pos, double end_hdg, list_t *segs);

void drive_on_line(vect2_t cur_pos, double cur_hdg, double cur_spd,
    double wheelbase, double max_steer, vect2_t line_start, double line_hdg,
    double speed, double arm_len, double steer_corr_amp, double *last_mis_hdg,
    double d_t, double *steer_out, double *speed_out);

double turn_run_speed(list_t *segs, double rhdg, double radius,
    bool_t backward, double max_ang_vel, const seg_t *next);
double straight_run_speed(list_t *segs, double rmng_d, bool_t backward,
    double max_ang_vel, const seg_t *next);

#ifdef	__cplusplus
}
#endif

#endif	/* _DRIVING_H_ */
