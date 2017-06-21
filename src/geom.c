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

#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#include "assert.h"
#include "math.h"
#include "helpers.h"
#include "geom.h"

/*
 * Determines whether an angle is part of an arc.
 *
 * @param angle_x Angle who's membership of the arc to examine (in degrees).
 * @param angle1 Start angle of the arc (in degrees).
 * @param angle2 End angle of the arc (in degrees).
 * @param cw Flag indicating whether the arc progresses clockwise or
 *      counter clockwise from angle1 to angle2.
 *
 * @return B_TRUE if angle_x is on the arc, B_FALSE if it is not.
 */
bool_t
is_on_arc(double angle_x, double angle1, double angle2, bool_t cw)
{
	if (cw) {
		if (angle1 < angle2)
			return (angle_x >= angle1 && angle_x <= angle2);
		else
			return (angle_x >= angle1 || angle_x <= angle2);
	} else {
		if (angle1 < angle2)
			return (angle_x <= angle1 || angle_x >= angle2);
		else
			return (angle_x <= angle1 && angle_x >= angle2);
	}
}


/*
 * Returns the absolute value (length) of a 3-space vector:
 * r = |a|
 */
double
vect3_abs(vect3_t a)
{
	return (sqrt(POW2(a.x) + POW2(a.y) + POW2(a.z)));
}

/*
 * Same as vect3_abs, but for 2-space vectors.
 */
double
vect2_abs(vect2_t a)
{
	return (sqrt(POW2(a.x) + POW2(a.y)));
}

/*
 * Returns the distance between two points defined by vectors `a' and `b'.
 */
double
vect2_dist(vect2_t a, vect2_t b)
{
	return (vect2_abs(vect2_sub(a, b)));
}

/*
 * Sets the absolute value (length) of a vector without changing
 * its orientation.
 */
vect3_t
vect3_set_abs(vect3_t a, double abs)
{
	double oldval = vect3_abs(a);
	if (oldval != 0.0)
		return (vect3_scmul(a, abs / oldval));
	else
		return (ZERO_VECT3);
}

/*
 * Same as vect3_set_abs, but for 2-space vectors.
 */
vect2_t
vect2_set_abs(vect2_t a, double abs)
{
	double oldval = vect2_abs(a);
	if (oldval != 0.0)
		return (vect2_scmul(a, abs / oldval));
	else
		return (ZERO_VECT2);
}

/*
 * Returns a unit vector (vector with identical orientation but a length of 1)
 * for a given input vector. The length of the input vector is stored in `l'.
 */
vect3_t
vect3_unit(vect3_t a, double *l)
{
	double len;
	len = vect3_abs(a);
	if (len == 0)
		return (NULL_VECT3);
	if (l)
		*l = len;
	return (VECT3(a.x / len, a.y / len, a.z / len));
}

/*
 * Same as vect3_unit, but for 2-space vectors.
 */
vect2_t
vect2_unit(vect2_t a, double *l)
{
	double len;
	len = vect2_abs(a);
	if (len == 0)
		return (NULL_VECT2);
	if (l)
		*l = len;
	return (VECT2(a.x / len, a.y / len));
}

/*
 * Adds 3-space vectors `a' and `b' and returns the result:
 * _   _   _
 * r = a + b
 */
vect3_t
vect3_add(vect3_t a, vect3_t b)
{
	return (VECT3(a.x + b.x, a.y + b.y, a.z + b.z));
}

/*
 * Same as vect3_add, but for 2-space vectors.
 */
vect2_t
vect2_add(vect2_t a, vect2_t b)
{
	return (VECT2(a.x + b.x, a.y + b.y));
}

/*
 * Subtracts 3-space vector `b' from vector `a' and returns the result:
 * _   _   _
 * r = a - b
 */
vect3_t
vect3_sub(vect3_t a, vect3_t b)
{
	return (VECT3(a.x - b.x, a.y - b.y, a.z - b.z));
}

/*
 * Same as vect3_sub, but for 2-space vectors.
 */
vect2_t
vect2_sub(vect2_t a, vect2_t b)
{
	return (VECT2(a.x - b.x, a.y - b.y));
}

/*
 * Performs a scalar multiply of 3-space vector `a' and scalar value `b' and
 * returns the result:
 * _   _
 * r = ab
 */
vect3_t
vect3_scmul(vect3_t a, double b)
{
	return (VECT3(a.x * b, a.y * b, a.z * b));
}

/*
 * Same as vect3_scmul, but for 2-space vectors.
 */
vect2_t
vect2_scmul(vect2_t a, double b)
{
	return (VECT2(a.x * b, a.y * b));
}

/*
 * Returns the dot product of 3-space vectors `a' and `b':
 *     _   _
 * r = a . b
 */
double
vect3_dotprod(vect3_t a, vect3_t b)
{
	return (a.x * b.x + a.y * b.y + a.z * b.z);
}

/*
 * Same as vect3_dotprod, but for 2-space vectors.
 */
double
vect2_dotprod(vect2_t a, vect2_t b)
{
	return (a.x * b.x + a.y * b.y);
}

/*
 * Returns the cross product of 3-space vectors `a' and `b':
 * _   _   _
 * r = a x b
 */
vect3_t
vect3_xprod(vect3_t a, vect3_t b)
{
	return (VECT3(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z,
	    a.x * b.y - a.y * b.x));
}

/*
 * Returns the mean vector of 3-space vectors `a' and `b'. That is, the
 * resulting vector will point exactly in between `a' and `b':
 *
 *   ^.
 *   |  .
 *   |    .
 *   |     x.
 * a |   / c  .
 *   | /        .
 *   +----------->
 *     b
 */
vect3_t
vect3_mean(vect3_t a, vect3_t b)
{
	return (VECT3((a.x + b.x) / 2, (a.y + b.y) / 2, (a.z + b.z) / 2));
}

/*
 * Rotates vector `v' by 90 degrees either to the right or left. This is
 * faster than doing full trigonometric calculations in vect2_rot.
 */
vect2_t
vect2_norm(vect2_t v, bool_t right)
{
	if (right)
		return (VECT2(v.y, -v.x));
	else
		return (VECT2(-v.y, v.x));
}

/*
 * Rotates vector `v' by `a' degrees to the right.
 */
vect2_t
vect2_rot(vect2_t v, double a)
{
	double sin_a = sin(DEG2RAD(-a)), cos_a = cos(DEG2RAD(-a));
	return (VECT2(v.x * cos_a - v.y * sin_a, v.x * sin_a + v.y * cos_a));
}

/*
 * Negates vector `v' to point in the opposite direction.
 */
vect2_t
vect2_neg(vect2_t v)
{
	return (VECT2(-v.x, -v.y));
}

/*
 * Calculates a 2D vector/vector intersection point and returns it.
 *
 * @param a First vector.
 * @param oa Vector to origin of first vector from the coordinate origin.
 * @param b Second vector.
 * @param oa Vector to origin of second vector from the coordinate origin.
 * @param confined If B_TRUE, only intersects which lie between the vectors'
 *	start & ending points (inclusive) are considered. Otherwise any
 *	intersect along an infinite linear extension of the vectors is returned.
 *
 * @return A vector from the coordinate origin to the intersection point
 *	or NULL_VECT2 if the vectors are parallel (no intersection or inf
 *	many intersections if they're directly on top of each other).
 */
vect2_t
vect2vect_isect(vect2_t a, vect2_t oa, vect2_t b, vect2_t ob, bool_t confined)
{
	vect2_t p1, p2, p3, p4, r;
	double ca, cb, det;

	if (VECT2_PARALLEL(a, b))
		return (NULL_VECT2);

	if (VECT2_EQ(oa, ob))
		return (oa);

	p1 = oa;
	p2 = vect2_add(oa, a);
	p3 = ob;
	p4 = vect2_add(ob, b);

	det = (p1.x - p2.x) * (p3.y - p4.y) - (p1.y - p2.y) * (p3.x - p4.x);
	ASSERT(det != 0.0);
	ca = p1.x * p2.y - p1.y * p2.x;
	cb = p3.x * p4.y - p3.y * p4.x;
	r.x = (ca * (p3.x - p4.x) - cb * (p1.x - p2.x)) / det;
	r.y = (ca * (p3.y - p4.y) - cb * (p1.y - p2.y)) / det;

	if (confined) {
		if (r.x < MIN(p1.x, p2.x) - ROUND_ERROR ||
		    r.x > MAX(p1.x, p2.x) + ROUND_ERROR ||
		    r.x < MIN(p3.x, p4.x) - ROUND_ERROR ||
		    r.x > MAX(p3.x, p4.x) + ROUND_ERROR ||
		    r.y < MIN(p1.y, p2.y) - ROUND_ERROR ||
		    r.y > MAX(p1.y, p2.y) + ROUND_ERROR ||
		    r.y < MIN(p3.y, p4.y) - ROUND_ERROR ||
		    r.y > MAX(p3.y, p4.y) + ROUND_ERROR)
			return (NULL_VECT2);
	}

	return (r);
}

/*
 * Given a true heading in degrees, constructs a unit vector pointing in that
 * direction. 0 degress is parallel with y axis and hdg increases clockwise.
 */
vect2_t
hdg2dir(double truehdg)
{
	truehdg = DEG2RAD(truehdg);
	return (VECT2(sin(truehdg), cos(truehdg)));
}

/*
 * Given a direction vector, returns the true heading that the vector
 * is pointing. See hdg2dir for a description of the returned heading value.
 */
double
dir2hdg(vect2_t dir)
{
	if (dir.x >= 0 && dir.y >= 0)
		return (RAD2DEG(asin(dir.x / vect2_abs(dir))));
	if (dir.x < 0 && dir.y >= 0)
		return (360 + RAD2DEG(asin(dir.x / vect2_abs(dir))));
	if (dir.x >= 0 && dir.y < 0)
		return (180 - RAD2DEG(asin(dir.x / vect2_abs(dir))));
	return (180 - RAD2DEG(asin(dir.x / vect2_abs(dir))));
}
