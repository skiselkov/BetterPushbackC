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

#ifndef	_TRUCK_H_
#define	_TRUCK_H_

#include <XPLMScenery.h>
#include <acfutils/geom.h>

#include "driving.h"

#ifdef	__cplusplus
extern "C" {
#endif

#define	PB_TRUCK_CONN_OFFSET	2

typedef struct {
	vehicle_pos_t	pos;
	vehicle_t	veh;
	double		cur_steer;
	double		last_mis_hdg;
	double		fixed_offset;	/* fixed wheel Z offset from CG */
	XPLMObjectRef	obj;
	list_t		segs;
} truck_t;

void truck_create(truck_t *truck, vect2_t pos, double hdg);
void truck_destroy(truck_t *truck);

bool_t truck_drive2point(truck_t *truck, vect2_t dst, double hdg);
void truck_run(truck_t *truck, double d_t);
void truck_draw(truck_t *truck);

bool_t truck_is_stopped(const truck_t *truck);

#ifdef	__cplusplus
}
#endif

#endif	/* _TRUCK_H_ */
