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
#include <stddef.h>

#include <acfutils/assert.h>
#include <acfutils/helpers.h>

#include "driving.h"
#include "truck.h"

#define	TRUCK_HEIGHT		1
#define	TRUCK_WHEELBASE		5
#define	TRUCK_FIXED_OFFSET	2.5
#define	TRUCK_MAX_STEER		60
#define	TRUCK_OBJ	\
	"MisterX_Library/Airport/Vehicles/Pushback_Trucks/Supertruck/White.obj"
#define	TRUCK_ACCEL		0.5
#define	TRUCK_STEER_RATE	40
#define	TRUCK_MAX_ANG_VEL	20

void
truck_create(truck_t *truck, vect2_t pos, double hdg)
{
	memset(truck, 0, sizeof (*truck));
	truck->pos.pos = pos;
	truck->pos.hdg = hdg;
	truck->veh.wheelbase = TRUCK_WHEELBASE;
	truck->veh.max_steer = TRUCK_MAX_STEER;
	truck->obj = XPLMLoadObject(TRUCK_OBJ);
	list_create(&truck->segs, sizeof (seg_t), offsetof(seg_t, node));
}

void
truck_destroy(truck_t *truck)
{
	seg_t *seg;

	XPLMUnloadObject(truck->obj);
	while ((seg = list_head(&truck->segs)) != NULL) {
		list_remove_head(&truck->segs);
		free(seg);
	}
	list_destroy(&truck->segs);

	memset(truck, 0, sizeof (*truck));
}

bool_t
truck_drive2point(truck_t *truck, vect2_t dst, double hdg)
{
	vect2_t cur_pos;
	double cur_hdg;
	seg_t *seg;

	seg = list_tail(&truck->segs);
	if (seg != NULL) {
		cur_pos = seg->end_pos;
		cur_hdg = seg->end_hdg;
	} else {
		cur_pos = truck->pos.pos;
		cur_hdg = truck->pos.hdg;
	}

	return (compute_segs(&truck->veh, cur_pos, cur_hdg, dst, hdg,
	    &truck->segs) >= 0);
}

void
truck_run(truck_t *truck, double d_t)
{
	double steer = 0, speed = 0;
	double accel, turn, radius, d_hdg_rad;
	vect2_t pos_incr;

	(void) drive_segs(&truck->pos, &truck->veh, &truck->segs,
	    TRUCK_MAX_ANG_VEL, &truck->last_mis_hdg, d_t, &steer, &speed);

	if (speed >= truck->pos.spd)
		accel = MIN(speed - truck->pos.spd, TRUCK_ACCEL / d_t);
	else
		accel = MAX(speed - truck->pos.spd, -TRUCK_ACCEL / d_t);

	if (steer >= truck->cur_steer)
		turn = MIN(steer - truck->cur_steer, TRUCK_STEER_RATE / d_t);
	else
		turn = MAX(steer - truck->cur_steer, -TRUCK_STEER_RATE / d_t);

	truck->pos.spd += accel;
	truck->cur_steer += turn;

	radius = MIN(tan(DEG2RAD(90 - truck->cur_steer)) * truck->veh.wheelbase,
	    1e6);
	d_hdg_rad = (truck->pos.spd / radius) * d_t;
	pos_incr = VECT2(sin(d_hdg_rad) * radius, cos(d_hdg_rad) * radius);
	truck->pos.pos = vect2_add(truck->pos.pos,
	    vect2_rot(pos_incr, truck->pos.hdg));
	truck->pos.hdg += RAD2DEG(d_hdg_rad);

	logMsg("truck pos: %.1fx%.1f spd: %.1f hdg: %.1f",
	    truck->pos.pos.x, truck->pos.pos.y, truck->pos.spd,
	    truck->pos.hdg);
}

void
truck_draw(truck_t *truck)
{
	XPLMDrawInfo_t di;
	XPLMProbeRef probe = XPLMCreateProbe(xplm_ProbeY);
	XPLMProbeInfo_t info = { .structSize = sizeof (XPLMProbeInfo_t) };
	vect3_t pos, norm;
	vect2_t v;

	/* X-Plane's Z axis is inverted to ours */
	VERIFY3U(XPLMProbeTerrainXYZ(probe, truck->pos.pos.x, 0,
	    -truck->pos.pos.y, &info), ==, xplm_ProbeHitTerrain);

	pos = VECT3(truck->pos.pos.x, info.locationY, -truck->pos.pos.y);
	norm = VECT3(info.normalX, info.normalY, info.normalZ);
	pos = vect3_add(pos, vect3_set_abs(norm, TRUCK_HEIGHT));

	v = VECT2(norm.x, -norm.z);
	v = vect2_rot(v, truck->pos.hdg);

	di.structSize = sizeof (di);
	di.x = pos.x;
	di.y = pos.y;
	di.z = pos.z;
	di.heading = truck->pos.hdg;
	di.pitch = acos(v.x / norm.y);
	di.roll = asin(v.x / norm.y);

	XPLMDrawObjects(truck->obj, 1, &di, 1, 1);
}
