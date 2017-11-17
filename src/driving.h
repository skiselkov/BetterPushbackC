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

#include <acfutils/avl.h>
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

	/*
	 * The vector positions are in local OpenGL coordinates space and
	 * are used by the steering algorithm. The geographic coordinates
	 * are used by the persistence algorithms to store the points in
	 * an absolute sense. Before passing such a segment to the driving
	 * algorithm, they must be converted to local coordinates using
	 * seg_world2local.
	 */
	bool_t	have_local_coords;
	bool_t	have_world_coords;

	vect2_t		start_pos;
	geo_pos2_t	start_pos_geo;
	double		start_hdg;
	vect2_t		end_pos;
	geo_pos2_t	end_pos_geo;
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

/*
 * Current position, orientation & velocity of vehicle.
 */
typedef struct {
	vect2_t	pos;		/* centerpoint position, world coords, meters */
	double	hdg;		/* true heading, world coords, degrees */
	double	spd;		/* forward speed, m/s, neg when reversing */
} vehicle_pos_t;

/*
 * Vehicle capability description. Used when determining steering commands.
 */
typedef struct {
	double	wheelbase;	/* distance from front to rear axle, meters */
	double	fixed_z_off;	/* long offset of rear axle from pos, meters */
	double	max_steer;	/* max steer angle, degrees */
	double	max_fwd_spd;	/* max forward speed, m/s */
	double	max_rev_spd;	/* max rev speed, m/s */
	double	max_fwd_ang_vel;/* max forward turn angular velocity, deg/s */
	double	max_rev_ang_vel;/* max reverse turn angular velocity, deg/s */
	double	max_centr_accel;/* max centripetal accel in a turn, m/s^2 */
	double	max_accel;	/* max acceleration, m/s^2 */
	double	max_decel;	/* max deceleration, m/s^2 */
	bool_t	xp10_bug_ign;	/* ignore X-Plane 10 stickiness bug */
	bool_t	use_rear_pos;	/* drive as if our pos is on our rear axle */
} vehicle_t;

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

int compute_segs(const vehicle_t *veh, vect2_t start_pos, double start_hdg,
    vect2_t end_pos, double end_hdg, list_t *segs);
bool_t drive_segs(const vehicle_pos_t *pos, const vehicle_t *veh, list_t *segs,
    double *last_mis_hdg, double d_t, double *out_steer, double *out_speed,
    bool_t *out_decelerating);
double ang_vel_speed_limit(const vehicle_t *veh, double steer, double speed);

void seg_world2local(seg_t *seg);
void seg_local2world(seg_t *seg);

void route_save(const list_t *segs);
void route_load(geo_pos2_t start_pos, double start_hdg, list_t *segs);

#define	MIN_SPEED_XP10	0.6
#define	CRAWL_SPEED(xpversion, veh)	/* m/s */ \
	(((xpversion) >= 11000 || (veh)->xp10_bug_ign) ? 0.1 : MIN_SPEED_XP10)

route_t *route_alloc(avl_tree_t *route_table, const list_t *segs);
void route_free(route_t *r);
void route_seg_append(avl_tree_t *route_table, route_t *r, const seg_t *seg);

void route_table_create(avl_tree_t *route_table);
void route_table_destroy(avl_tree_t *route_table);
bool_t route_table_store(avl_tree_t *route_table, const char *filename);

#ifdef	__cplusplus
}
#endif

#endif	/* _DRIVING_H_ */
