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

#ifndef	_BP_GEOM_H_
#define	_BP_GEOM_H_

#include <stdlib.h>
#include <math.h>

#include "types.h"

#ifdef	__cplusplus
extern "C" {
#endif

typedef struct {
	double	x;
	double	y;
	double	z;
} vect3_t;

typedef struct {
	double	x;
	double	y;
} vect2_t;

/*
 * Unit conversions
 */
#define	RAD2DEG_RATIO	(M_PI / 180)		/* 1 rad / 180 deg */
#define	DEG2RAD_RATIO	(180 / M_PI)		/* 180 deg / 1 rad */
#define	DEG2RAD(d)	((d) * RAD2DEG_RATIO)	/* degrees to radians */
#define	RAD2DEG(r)	((r) * DEG2RAD_RATIO)	/* radians to degrees */

/*
 * Coordinate constructors.
 */
#define	VECT2(x, y)			((vect2_t){(x), (y)})
#define	VECT3(x, y, z)			((vect3_t){(x), (y), (z)})
#define	VECT2_EQ(a, b)			((a).x == (b).x && (a).y == (b).y)
#define	VECT2_PARALLEL(a, b)	\
	(((a).y == 0 && (b).y == 0) || (((a).x / (a).y) == ((b).x / (b).y)))

/*
 * Special coordinate values and tests for these special values.
 */
#define	ZERO_VECT2		((vect2_t){0.0, 0.0})
#define	ZERO_VECT3		((vect3_t){0.0, 0.0, 0.0})
#define	NULL_VECT2		((vect2_t){NAN, NAN})
#define	NULL_VECT3		((vect3_t){NAN, NAN, NAN})
#define	NULL_GEO_POS3		((geo_pos3_t){NAN, NAN, NAN})
#define	NULL_GEO_POS2		((geo_pos2_t){NAN, NAN})
#define	IS_NULL_VECT(a)		(isnan((a).x))
#define	IS_NULL_GEO_POS(a)	(isnan((a).lat))
#define	IS_ZERO_VECT2(a)	((a).x == 0.0 && (a).y == 0.0)
#define	IS_ZERO_VECT3(a)	((a).x == 0.0 && (a).y == 0.0 && (a).z == 0.0)

#define	VECT2_TO_VECT3(v, z)	((vect3_t){(v).x, (v).y, (z)})
#define	VECT3_TO_VECT2(v)	((vect2_t){(v).x, (v).y})

#ifndef	ABS
#define	ABS(x)	((x) > 0 ? (x) : -(x))
#endif

/* Math debugging */
#if	1
#define	PRINT_VECT2(v)	printf(#v "(%f, %f)\n", v.x, v.y)
#define	PRINT_VECT3(v)	printf(#v "(%f, %f, %f)\n", v.x, v.y, v.z)
#define	PRINT_GEO2(p)	printf(#p "(%f, %f)\n", p.lat, p.lon)
#define	PRINT_GEO3(p)	printf(#p "(%f, %f, %f)\n", p.lat, p.lon, p.elev)
#define	DEBUG_PRINT(...)	printf(__VA_ARGS__)
#else
#define	PRINT_VECT2(v)
#define	PRINT_VECT3(v)
#define	PRINT_GEO2(p)
#define	PRINT_GEO3(p)
#define	DEBUG_PRINT(...)
#endif

/*
 * Small helpers.
 */
#define	is_on_arc	SYMBOL_PREFIX(is_on_arc)
bool_t is_on_arc(double angle_x, double angle1, double angle2, bool_t cw);

/*
 * Vector math.
 */
#define	vect3_abs	SYMBOL_PREFIX(vect3_abs)
double vect3_abs(vect3_t a);
#define	vect2_abs	SYMBOL_PREFIX(vect2_abs)
double vect2_abs(vect2_t a);
#define	vect2_dist	SYMBOL_PREFIX(vect2_dist)
double vect2_dist(vect2_t a, vect2_t b);
#define	vect3_set_abs	SYMBOL_PREFIX(vect3_set_abs)
vect3_t vect3_set_abs(vect3_t a, double abs);
#define	vect2_set_abs	SYMBOL_PREFIX(vect2_set_abs)
vect2_t vect2_set_abs(vect2_t a, double abs);
#define	vect3_unit	SYMBOL_PREFIX(vect3_unit)
vect3_t vect3_unit(vect3_t a, double *l);
#define	vect2_unit	SYMBOL_PREFIX(vect2_unit)
vect2_t vect2_unit(vect2_t a, double *l);

#define	vect3_add	SYMBOL_PREFIX(vect3_add)
vect3_t vect3_add(vect3_t a, vect3_t b);
#define	vect2_add	SYMBOL_PREFIX(vect2_add)
vect2_t vect2_add(vect2_t a, vect2_t b);
#define	vect3_sub	SYMBOL_PREFIX(vect3_sub)
vect3_t vect3_sub(vect3_t a, vect3_t b);
#define	vect2_sub	SYMBOL_PREFIX(vect2_sub)
vect2_t vect2_sub(vect2_t a, vect2_t b);
#define	vect3_scmul	SYMBOL_PREFIX(vect3_scmul)
vect3_t vect3_scmul(vect3_t a, double b);
#define	vect2_scmul	SYMBOL_PREFIX(vect2_scmul)
vect2_t vect2_scmul(vect2_t a, double b);
#define	vect3_dotprod	SYMBOL_PREFIX(vect3_dotprod)
double vect3_dotprod(vect3_t a, vect3_t b);
#define	vect2_dotprod	SYMBOL_PREFIX(vect2_dotprod)
double vect2_dotprod(vect2_t a, vect2_t b);
#define	vect3_xprod	SYMBOL_PREFIX(vect3_xprod)
vect3_t vect3_xprod(vect3_t a, vect3_t b);
#define	vect3_mean	SYMBOL_PREFIX(vect3_mean)
vect3_t vect3_mean(vect3_t a, vect3_t b);

#define	vect2_norm	SYMBOL_PREFIX(vect2_norm)
vect2_t vect2_norm(vect2_t v, bool_t right);
#define	vect2_rot	SYMBOL_PREFIX(vect2_rot)
vect2_t vect2_rot(vect2_t v, double angle);
#define	vect2_neg	SYMBOL_PREFIX(vect2_neg)
vect2_t vect2_neg(vect2_t v);

/*
 * Interesections.
 */
#define	vect2vect_isect	SYMBOL_PREFIX(vect2vect_isect)
vect2_t vect2vect_isect(vect2_t da, vect2_t oa, vect2_t db, vect2_t ob,
    bool_t confined);

/*
 * Converting between headings and direction vectors on a 2D plane.
 */
#define	hdg2dir		SYMBOL_PREFIX(hdg2dir)
vect2_t hdg2dir(double truehdg);
#define	dir2hdg		SYMBOL_PREFIX(dir2hdg)
double dir2hdg(vect2_t dir);

#ifdef	__cplusplus
}
#endif

#endif	/* _BP_GEOM_H_ */
