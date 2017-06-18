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
 * The WGS84 ellipsoid parameters.
 */
const ellip_t wgs84 = {
	.a = 6378137.0,
	.b = 6356752.314245,
	.f = .00335281066474748071,
	.ecc = 0.08181919084296430238,
	.ecc2 = 0.00669437999019741354,
	.r = 6371200.0
};

/*
 * Naive implementation of matrix multiplication. We don't use this very
 * heavily and can thus afford to rely on compiler auto-vectorization to
 * get this optimized.
 * Multiplies `x' and `y' and places result in 'z'. `xrows', `ycols' and `sz'
 * have meanings as explained below:
 *                sz                     ycols               ycols
 *            |<------->|             |<------->|         |<------->|
 *            |         |             |         |         |         |
 *
 *       --- ++=       =++         .-++=       =++       ++=       =++ --
 * xrows ^   || x00 x01 || \/     /  || y00 y01 ||  ---  || z00 z01 ||  ^ xrows
 *       v   || x10 x11 || /\    /   || y10 y11 ||  ---  || z10 z11 ||  v
 *       --- ++=       =++      /  .-++=       =++       ++=       =++ --
 *                           --'  /
 *                        sz ^   /
 *                           v  /
 *                           --'
 */
static void
matrix_mul(const double *x, const double *y, double *z,
    size_t xrows, size_t ycols, size_t sz)
{
	memset(z, 0, sz * ycols * sizeof (double));
	for (size_t row = 0; row < xrows; row++) {
		for (size_t col = 0; col < ycols; col++) {
			for (size_t i = 0; i < sz; i++) {
				z[row * ycols + col] += x[row * sz + i] *
				    y[i * ycols + col];
			}
		}
	}
}

/*
 * Determines whether an angle is part of an arc.
 *
 * @param angle_x Angle who's membership of the arc to examine (in degrees).
 * @param angle1 Start angle of the arc (in degrees).
 * @param angle2 End angle of the arc (in degrees).
 * @param cw Flag indicating whether the arc progresses clockwise or
 *	counter clockwise from angle1 to angle2.
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
 * Converts surface coordinates on an Earth-sized spheroid into 3-space
 * coordinate vector in ECEF space. Please note that this considers the
 * Earth to be a perfect sphere and hence cannot be used for very precise
 * calculations. For more accurate conversions, use geo2ecef.
 *
 * @param pos The input position to convert.
 *
 * In 3-space, axes have their origins at the globe center point, are
 * perpendicular to each other and are designated as follows:
 * - x: positive & passing through lat=0, lon=0
 * - y: positive & passing through lat=0, lon=+90
 * - z: positive & passing through lat=90
 */
vect3_t
sph2ecef(geo_pos3_t pos)
{
	vect3_t result;
	double lat_rad, lon_rad, R, R0;

	lat_rad = DEG2RAD(pos.lat);
	lon_rad = DEG2RAD(pos.lon);

	/* R is total distance from center at alt_msl */
	R = pos.elev + EARTH_MSL;
	/*
	 * R0 is the radius of a circular cut parallel to the equator at the
	 * given latitude of a sphere with radius R.
	 */
	R0 = R * cos(lat_rad);
	/* Given R and R0, we can transform the geo coords into 3-space: */
	result.x = R0 * cos(lon_rad);
	result.y = R0 * sin(lon_rad);
	result.z = R * sin(lat_rad);

	return (result);
}

ellip_t
ellip_init(double semi_major, double semi_minor, double flattening)
{
	ellip_t ellip;

	ellip.a = semi_major;
	ellip.b = semi_minor;
	ellip.ecc2 = flattening * (2 - flattening);

	return (ellip);
}

geo_pos3_t
geo2sph(geo_pos3_t pos, const ellip_t *ellip)
{
	double		lat_r = DEG2RAD(pos.lat);
	double		sin_lat = sin(lat_r);
	double		p, z;
	double		Rc;	/* curvature of the prime vertical */
	geo_pos3_t	res;

	Rc = ellip->a / sqrt(1 - ellip->ecc2 * POW2(sin_lat));
	p = (Rc + pos.elev) * cos(lat_r);
	z = ((Rc * (1 - ellip->ecc2)) + pos.elev) * sin_lat;

	res.elev = sqrt(POW2(p) + POW2(z));
	res.lat = RAD2DEG(asin(z / res.elev));
	res.lon = pos.lon;

	return (res);
}

vect3_t
geo2ecef(geo_pos3_t pos, const ellip_t *ellip)
{
	double	h = pos.elev / 3.281;	/* convert to meters */
	double	lat_r = DEG2RAD(pos.lat);
	double	lon_r = DEG2RAD(pos.lon);
	double	Rc;	/* curvature of the prime vertical */
	vect3_t	res;
	double	sin_lat = sin(lat_r), cos_lat = cos(lat_r);
	double	sin_lon = sin(lon_r), cos_lon = cos(lon_r);

	Rc = ellip->a / sqrt(1 - ellip->ecc2 * POW2(sin_lat));

	res.x = (Rc + h) * cos_lat * cos_lon;
	res.y = (Rc + h) * cos_lat * sin_lon;
	res.z = (Rc * (1 - ellip->ecc2) + h) * sin_lat;

	return (res);
}

geo_pos3_t
ecef2geo(vect3_t pos, const ellip_t *ellip)
{
	geo_pos3_t	res;
	double		B;
	double		d;
	double		e;
	double		f;
	double		g;
	double		p;
	double		q;
	double		r;
	double		t;
	double		v;
	double		zlong;

	/*
	 * 1.0 compute semi-minor axis and set sign to that of z in order
	 * to get sign of Phi correct.
	 */
	B = (pos.z >= 0 ? ellip->b : -ellip->b);

	/*
	 * 2.0 compute intermediate values for latitude
	 */
	r = sqrt(POW2(pos.x) + POW2(pos.y));
	e = (B * pos.z - (POW2(ellip->a) - POW2(B))) / (ellip->a * r);
	f = (B * pos.z + (POW2(ellip->a) - POW2(B))) / (ellip->a * r);

	/*
	 * 3.0 find solution to:
	 *       t^4 + 2*E*t^3 + 2*F*t - 1 = 0
	 */
	p = (4.0 / 3.0) * (e * f + 1.0);
	q = 2.0 * (POW2(e) - POW2(f));
	d = POW3(p) + POW2(q);

	if (d >= 0.0) {
		v = pow((sqrt(d) - q), (1.0 / 3.0)) - pow((sqrt(d) + q),
		    (1.0 / 3.0));
	} else {
		v = 2.0 * sqrt(-p) * cos(acos(q / (p * sqrt(-p))) / 3.0);
	}

	/*
	 * 4.0 improve v
	 *       NOTE: not really necessary unless point is near pole
	 */
	if (POW2(v) < fabs(p))
		v = -(POW3(v) + 2.0 * q) / (3.0 * p);
	g = (sqrt(POW2(e) + v) + e) / 2.0;
	t = sqrt(POW2(g)  + (f - v * g) / (2.0 * g - e)) - g;

	res.lat = atan((ellip->a * (1.0 - POW2(t))) / (2.0 * B * t));

	/*
	 * 5.0 compute height above ellipsoid
	 */
	res.elev= (r - ellip->a * t) * cos(res.lat) + (pos.z - B) * sin(res.lat);

	/*
	 *   6.0 compute longitude east of Greenwich
	 */
	zlong = atan2(pos.y, pos.x);
	if (zlong < 0.0)
		zlong = zlong + (2 * M_PI);

	res.lon = zlong;

	/*
	 * 7.0 convert latitude and longitude to degrees & elev to feet
	 */
	res.lat = RAD2DEG(res.lat);
	res.lon = RAD2DEG(res.lon);
	res.elev = MET2FEET(res.elev);
	if (res.lon >= 180.0)
		res.lon -= 360.0;

	return (res);
}

/*
 * Converts a 3-space coordinate vector from ECEF coordinate space into
 * geocentric coordinates on an EARTH_MSL-radius spheroid.
 */
geo_pos3_t
ecef2sph(vect3_t v)
{
	geo_pos3_t pos;
	double lat_rad, lon_rad, R, R0;

	R0 = sqrt(v.x * v.x + v.y * v.y);
	R = vect3_abs(v);
	if (R0 == 0) {
		/* to prevent a div-by-zero at the poles */
		R0 = 0.000000001;
	}
	lat_rad = atan(v.z / R0);
	lon_rad = asin(v.y / R0);
	if (v.x < 0.0) {
		if (v.y >= 0.0)
			lon_rad = M_PI - lon_rad;
		else
			lon_rad = -M_PI - lon_rad;
	}
	pos.elev = R - EARTH_MSL;
	pos.lat = RAD2DEG(lat_rad);
	pos.lon = RAD2DEG(lon_rad);

	return (pos);
}

/*
 * Determines whether and where a vector intersects the surface of a sphere.
 * Returns the number of intersection points (zero, one or two).
 *
 * @param v Vector for which to determine the intersection.
 * @param o Vector pointing from the coordinate origin to the origin
 *	of vector v (i.e. displacement of `v' from the origin).
 * @param c Displacement of sphere center point from the coordinate origin.
 * @param r Radius of sphere.
 * @param confined If B_TRUE, only intersects which lie between the vector's
 *	start & ending point (inclusive) are returned. Otherwise any
 *	intersect along an infinite linear extension of the vector is returned.
 * @param i If not NULL, this function stores vectors pointing to the
 *	intersection points from the coordinate origin in this array:
 *	- if 0 is returned, two null vectors are stored in the array.
 *	- if 1 is returned, one null vector and one non-null vector pointing
 *	to the intersection point are stored in the array (ordering in the
 *	array is not guarantted as described here).
 *	- if 2 is returned, two non-null vectors pointing to the
 *	intersection points are stored in the array.
 */
unsigned
vect2sph_isect(vect3_t v, vect3_t o, vect3_t c, double r, bool_t confined,
    vect3_t i[2])
{
	vect3_t l, o_min_c;
	double d, l_dot_o_min_c, sqrt_tmp, o_min_c_abs;

	/* convert v into a unit vector 'l' and scalar distance 'd' */
	l = vect3_unit(v, &d);

	/* compute (o - c) and the dot product of l.(o - c) */
	o_min_c = vect3_sub(o, c);
	l_dot_o_min_c = vect3_dotprod(l, o_min_c);

	/*
	 * The full formula for the distance along L for the intersects is:
	 * -(l.(o - c)) +- sqrt((l.(o - c))^2 - abs(o - c)^2 + r^2)
	 * The part in the sqrt may be negative, zero or positive, indicating
	 * respectively no intersection, one touching point or two points, so
	 * before we start sqrt()ing away, we check which it is. Also, this
	 * checks for a solution on an infinite line between extending along
	 * v. Before we declare victory, we check that the computed
	 * points lie on the vector.
	 */
	o_min_c_abs = vect3_abs(o_min_c);
	sqrt_tmp = POW2(l_dot_o_min_c) - POW2(o_min_c_abs) + POW2(r);
	if (sqrt_tmp > 0) {
		/* Two solutions */
		double i1_d, i2_d;
		unsigned intersects = 0;

		sqrt_tmp = sqrt(sqrt_tmp);

		i1_d = -l_dot_o_min_c - sqrt_tmp;
		if ((i1_d >= 0 && i1_d <= d) || !confined) {
			/*
			 * Solution lies on vector, store a vector to it
			 * if the caller requested it.
			 */
			if (i != NULL)
				i[intersects] = vect3_add(vect3_scmul(l, i1_d),
				    o);
			intersects++;
		} else {
			/* Solution lies outside of line between o1 & o2 */
			i1_d = NAN;
			if (i != NULL)
				i[intersects] = NULL_VECT3;
		}

		/* ditto for the second intersect */
		i2_d = -l_dot_o_min_c + sqrt_tmp;
		if ((i2_d >= 0 && i2_d <= d) || !confined) {
			if (i != NULL)
				i[intersects] = vect3_add(vect3_scmul(l, i2_d),
				    o);
			intersects++;
		} else {
			i2_d = NAN;
			if (i != NULL)
				i[intersects] = NULL_VECT3;
		}

		return (intersects);
	} else if (sqrt_tmp == 0) {
		/* One solution */
		double i1_d;

		if (i != NULL)
			i[1] = NULL_VECT3;

		i1_d = -l_dot_o_min_c;
		if ((i1_d >= 0 && i1_d <= d) || !confined) {
			if (i != NULL)
				i[0] = vect3_add(vect3_scmul(l, i1_d), o);
			return (1);
		} else {
			if (i != NULL)
				i[0] = NULL_VECT3;
			return (0);
		}
	} else {
		/* No solution, no intersections, NaN i1 & i2 */
		if (i != NULL) {
			i[0] = NULL_VECT3;
			i[1] = NULL_VECT3;
		}
		return (0);
	}
}

/*
 * Determines whether and where a 2D vector intersects a 2D circle. The
 * meanings of the arguments and return value are exactly the same as in
 * vect2sph_isect.
 */
unsigned
vect2circ_isect(vect2_t v, vect2_t o, vect2_t c, double r, bool_t confined,
    vect2_t i[2])
{
	/*
	 * This is basically a simplified case of a vect2sph intersection,
	 * where both the vector and sphere's center lie on the xy plane.
	 * So just convert to 3D coordinates with z=0 and run vect2sph_isect.
	 * This only adds one extra coordinate to the calculation, which is
	 * generally negligible on performance.
	 */
	vect3_t v3 = VECT3(v.x, v.y, 0), o3 = VECT3(o.x, o.y, 0);
	vect3_t c3 = VECT3(c.x, c.y, 0);
	vect3_t i3[2];
	int n;

	n = vect2sph_isect(v3, o3, c3, r, confined, i3);
	if (i != NULL) {
		i[0] = VECT2(i3[0].x, i3[0].y);
		i[1] = VECT2(i3[1].x, i3[1].y);
	}

	return (n);
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
 * Computes the intersection of two circles. The circles are centered at
 * `ca' and `cb' and have radii `ra' and `rb' respectively.
 * The `i' argument must point to an array capable of holding at least two
 * points, which will be filled with the intersection points.
 * The function returns the number of intersections (and appropriate slots
 * filled in `i'). Possible values are:
 *	0: the circles do not intersect.
 *	1: the circles touch in one point.
 *	2: the circles intersection at two points.
 *	UINT_MAX: the circles are identical (`i' is NOT populated).
 */
unsigned
circ2circ_isect(vect2_t ca, double ra, vect2_t cb, double rb, vect2_t i[2])
{
	double a, d, h;
	vect2_t ca_cb, p2, ca_p2;

	if (VECT2_EQ(ca, cb) && ra == rb)
		return (UINT_MAX);

	ca_cb = vect2_sub(cb, ca);
	d = vect2_abs(ca_cb);
	if ((d == 0 && ra == rb) || d > ra + rb ||
	    d + MIN(ra, rb) < MAX(ra, rb))
		return (0);
	a = (POW2(ra) - POW2(rb) + POW2(d)) / (2 * d);
	if (POW2(ra) - POW2(a) < 0)
		h = 0;
	else
		h = sqrt(POW2(ra) - POW2(a));
	ca_p2 = vect2_set_abs(ca_cb, a);
	p2 = vect2_add(ca, ca_p2);

	if (h == 0) {
		i[0] = p2;
		ASSERT(!IS_NULL_VECT(i[0]));
		return (1);
	} else {
		i[0] = vect2_add(p2, vect2_set_abs(vect2_norm(ca_p2, B_FALSE),
		    h));
		i[1] = vect2_add(p2, vect2_set_abs(vect2_norm(ca_p2, B_TRUE),
		    h));
		ASSERT(!IS_NULL_VECT(i[0]) && !IS_NULL_VECT(i[1]));
		return (2);
	}
}

/*
 * Computes the intersection between a vector with origin `oa' and
 * direction & magnited `a', and a closed polygon described by the
 * points `poly' (the number of points in `poly_n'). The polygon MUST
 * consist of at least two points.
 * Returns the number of intersections between the vector and polygon. This
 * function only tests if the vector intersects the polygon's sides. If the
 * vector lies completely inside the polygon, this function returns 0.
 */
unsigned
vect2poly_isect(vect2_t a, vect2_t oa, const vect2_t *poly, size_t poly_n)
{
	unsigned n_isects = 0;

	ASSERT3U(poly_n, >=, 2);
	for (size_t i = 0; i < poly_n; i++) {
		vect2_t pt1 = poly[i];
		vect2_t pt2 = poly[(i + 1) % poly_n];
		vect2_t v = vect2_sub(pt2, pt1);
		vect2_t isect = vect2vect_isect(a, oa, v, pt1, B_TRUE);
		if (!IS_NULL_VECT(isect))
			n_isects++;
	}

	return (n_isects);
}

/*
 * Checks if the point `pt' lies inside of a closed polygon. See
 * vect2poly_isect for more details on polygons.
 * Returns B_TRUE if the point lies inside of thep polygon (or on one
 * of its sides) or B_FALSE otherwise.
 */
bool_t
point_in_poly(vect2_t pt, vect2_t *poly, size_t poly_n)
{
	ASSERT3U(poly_n, >=, 2);
	/*
	 * The simplest approach is ray casting. Construct a vector from `pt'
	 * to a point very far away and count how many edges of the polygon
	 * we hit. If we hit an even number, we're outside, otherwise we're
	 * inside.
	 */
	vect2_t v = vect2_sub(VECT2(1e20, 1e20), pt);
	unsigned isects = vect2poly_isect(v, pt, poly, poly_n);
	return ((isects & 1) != 0);
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

geo_pos2_t
geo_displace(const ellip_t *ellip, geo_pos2_t pos, double truehdg, double dist)
{
	return (geo_displace_dir(ellip, pos, hdg2dir(truehdg), dist));
}

geo_pos2_t
geo_displace_dir(const ellip_t *ellip, geo_pos2_t pos, vect2_t dir, double dist)
{
	double	dist_r = dist / EARTH_MSL;
	fpp_t	fpp;

	if (dist >= M_PI * EARTH_MSL / 2)
		return (NULL_GEO_POS2);
	fpp = gnomo_fpp_init(pos, 0, ellip, B_TRUE);
	dir = vect2_set_abs(dir, tan(dist_r));

	return (fpp2geo(dir, &fpp));
}

/*
 * Computes the number of latitudinal subdivisions used for tiling a spherical
 * surface. See world.c for a description of this tiling.
 */
unsigned
sphere_lat_subdiv(double radius, double partition_sz)
{
	ASSERT3F(radius, >=, partition_sz);
	return (ceil((radius * M_PI) / partition_sz) + 1);
}

/*
 * Computes the number of longitudinal subdivisions for a given latitude (given
 * in degrees, with 0 being the equator) used for tiling a spherical
 * surface. See world.c for a description of this tiling.
 */
unsigned
sphere_lon_subdiv(double radius, double lat, double partition_sz)
{
	ASSERT3F(lat, >=, -90.0);
	ASSERT3F(lat, <=, 90.0);
	ASSERT3F(radius, >=, partition_sz);
	double r = cos(DEG2RAD(lat)) * radius;
	return (ceil((2 * M_PI * r) / partition_sz));
}

bool_t
geo_pos2_from_str(const char *lat, const char *lon, geo_pos2_t *pos)
{
	pos->lat = atof(lat);
	pos->lon = atof(lon);
	return (is_valid_lat(pos->lat) && is_valid_lon(pos->lon));
}

bool_t
geo_pos3_from_str(const char *lat, const char *lon, const char *elev,
    geo_pos3_t *pos)
{
	pos->lat = atof(lat);
	pos->lon = atof(lon);
	pos->elev = atof(elev);
	return (is_valid_lat(pos->lat) && is_valid_lon(pos->lon) &&
	    is_valid_elev(pos->elev));
}

/* Cotangent */
static inline double
cot(double x)
{
	return (1.0 / tan(x));
}

/* Secant */
static inline double
sec(double x)
{
	return (1.0 / cos(x));
}

/*
 * Prepares a set of geographical coordinate translation parameters.
 *
 * @param displac The relative latitude & longitude (in degrees)
 *	between the origins of the two respective coordinate systems.
 *	For example, a displacement of +10 degrees of latitude (north)
 *	and +20 degrees of longitude (east) will result in an input
 *	coordinate of +5,+5 translating into -5,-15 in the target system
 *	(assuming `rotation' below is zero).
 *	Please note that these coordinates as well as all transformations
 *	are assumed to be in geocentric coordinates on the an EARTH_MSL
 *	radius spheroid.
 * @param rot The relative rotation of the axes of the target
 *	coordinate system to the source coordinate system in degrees
 *	counter-clockwise. For example, a rotation of +90 degrees and
 *	no translation applied to an input coordinate of +5 degrees
 *	of latitude (north) and +5 degrees of longitude (east) will
 *	translate into -5,+5.
 */
sph_xlate_t
sph_xlate_init(geo_pos2_t displac, double rot, bool_t inv)
{
	/*
	 * (ECEF axes:)
	 * lat xlate		y axis	alpha
	 * lon xlate		z axis	bravo
	 * viewport rotation	x axis	theta
	 */
	sph_xlate_t	xlate;
	double		alpha = DEG2RAD(!inv ? displac.lat : -displac.lat);
	double		bravo = DEG2RAD(!inv ? -displac.lon : displac.lon);
	double		theta = DEG2RAD(!inv ? rot : -rot);

#define	M(m, r, c)	((m)[(r) * 3 + (c)])
	double		R_a[3 * 3], R_b[3 * 3];
	double		sin_alpha = sin(alpha), cos_alpha = cos(alpha);
	double		sin_bravo = sin(bravo), cos_bravo = cos(bravo);
	double		sin_theta = sin(theta), cos_theta = cos(theta);

	/*
	 * +-                  -+
	 * | cos(a)   0  sin(a) |
	 * |    0     1     0   |
	 * | -sin(a)  0  cos(a) |
	 * +-                  -+
	 */
	memset(R_a, 0, sizeof (R_a));
	M(R_a, 0, 0) = cos_alpha;
	M(R_a, 0, 2) = sin_alpha;
	M(R_a, 1, 1) = 1;
	M(R_a, 2, 0) = -sin_alpha;
	M(R_a, 2, 2) = cos_alpha;

	/*
	 * +-                  -+
	 * | cos(g)  -sin(g)  0 |
	 * | sin(g)   cos(g)  0 |
	 * |   0        0     1 |
	 * +-                  -+
	 */
	memset(R_b, 0, sizeof (R_b));
	M(R_b, 0, 0) = cos_bravo;
	M(R_b, 0, 1) = -sin_bravo;
	M(R_b, 1, 0) = sin_bravo;
	M(R_b, 1, 1) = cos_bravo;
	M(R_b, 2, 2) = 1;

	if (!inv)
		matrix_mul(R_a, R_b, xlate.sph_matrix, 3, 3, 3);
	else
		matrix_mul(R_b, R_a, xlate.sph_matrix, 3, 3, 3);

	xlate.rot_matrix[0] = cos_theta;
	xlate.rot_matrix[1] = -sin_theta;
	xlate.rot_matrix[2] = sin_theta;
	xlate.rot_matrix[3] = cos_theta;
	xlate.inv = inv;

	return (xlate);
}

/*
 * Translates a point at `pos' using the translation specified by `xlate'.
 */
vect3_t
sph_xlate_vect(vect3_t p, const sph_xlate_t *xlate)
{
	vect3_t	q;
	vect2_t	r, s;

	if (xlate->inv) {
		/* Undo projection plane rotation along y axis. */
		r = VECT2(p.y, p.z);
		matrix_mul(xlate->rot_matrix, (double *)&r, (double *)&s,
		    2, 1, 2);
		p.y = s.x;
		p.z = s.y;
	}

	matrix_mul(xlate->sph_matrix, (double *)&p, (double *)&q, 3, 1, 3);

	if (!xlate->inv) {
		/*
		 * In the final projection plane, grab y & z coords & rotate
		 * along x axis.
		 */
		r = VECT2(q.y, q.z);
		matrix_mul(xlate->rot_matrix, (double *)&r, (double *)&s,
		    2, 1, 2);
		q.y = s.x;
		q.z = s.y;
	}

	return (q);
}

/*
 * Translates a point at `pos' using the geo translation specified by `xlate'.
 */
geo_pos2_t
sph_xlate(geo_pos2_t pos, const sph_xlate_t *xlate)
{
	vect3_t		v = sph2ecef(GEO2_TO_GEO3(pos, 0));
	vect3_t		r = sph_xlate_vect(v, xlate);
	geo_pos3_t	res = ecef2sph(r);
	return (GEO_POS2(res.lat, res.lon));
}

/*
 * Returns the great circle distance between two geodesic coordinates on the
 * Earth in meters.
 */
double
gc_distance(geo_pos2_t start, geo_pos2_t end)
{
	/*
	 * Convert both coordinates into 3D vectors and calculate the angle
	 * between them. GC distance is proportional to that angle.
	 */
	vect3_t	start_v = geo2ecef(GEO2_TO_GEO3(start, 0), &wgs84);
	vect3_t	end_v = geo2ecef(GEO2_TO_GEO3(end, 0), &wgs84);
	vect3_t	s2e = vect3_sub(end_v, start_v);
	double	s2e_abs = vect3_abs(s2e);
	double	alpha = asin(s2e_abs / 2 / EARTH_MSL);
	return	(2 * alpha * EARTH_MSL);
}

double
gc_point_hdg(geo_pos2_t start, geo_pos2_t end, double arg)
{
	/* FIXME: THIS IS BROKEN !!! */
	vect3_t	start_v, end_v, norm_v, an_v, incl_v;

	start_v = geo2ecef(GEO2_TO_GEO3(start, 0), &wgs84);
	end_v = geo2ecef(GEO2_TO_GEO3(end, 0), &wgs84);
	norm_v = vect3_set_abs(vect3_xprod(end_v, start_v), EARTH_MSL);
	an_v = vect3_set_abs(vect3_xprod(norm_v, VECT3(0, 0, 1)), EARTH_MSL);
	incl_v = vect3_xprod(norm_v, an_v);
	geo_pos3_t incl = ecef2geo(incl_v, &wgs84);
	double inclination = incl.lat;

	vect3_t arg_v = {sin(DEG2RAD(arg)) * EARTH_MSL,
	    cos(DEG2RAD(arg)) * EARTH_MSL, 0};
	arg_v = VECT3(sin(DEG2RAD(inclination)) * arg_v.x, arg_v.y,
	    sin(DEG2RAD(inclination)) * arg_v.z);
	arg_v = vect3_unit(vect3_xprod(arg_v, norm_v), NULL);
	double xy = sqrt(POW2(arg_v.x) + POW2(arg_v.y));

	return (RAD2DEG(acos(xy)));
}

/*
 * Prepares a set of projection parameters for projections from a fixed
 * origin along the projection axis onto a flat projection plane. The
 * plane is centered at `center' and is rotated `rot' degrees relative to
 * the sphere's native coordinate system. The projection plane touches the
 * sphere at the center point and is parallel to its surface there. Points
 * on the sphere are projected from a fixed origin along a projection axis
 * that is perpendicular to the projection plane and passes through its
 * center. The distance of this point along the projection axis from the
 * projection plane is `dist' with positive offsets increasing away from
 * the sphere's center.
 *
 *   projection.
 *       center \ | <- projection axis (positive offsets)
 *               v|
 *  ==============+=============+======= <- projection plane
 *         - '    |    ' -     / ^-- projected point
 *       /        |        \ x <-- projecting point
 *     /          |         /\
 *    | sphere    |       /   |
 *    | center -> +     /     |
 *    |           |   /       |
 *     \          | /        /
 *       \        + <- projection origin
 *         -      |      -
 *           '----|----'
 *                |
 *                | <- projection axis (negative offsets)
 *
 * You can pass INFINITY for `dist', in which case the projection origin
 * will be centered at +INFINITY, constructing an orthographic projection.
 * N.B. ATM there is no way to position the projection point at -INFINITY.
 * If you wish to construct an inverted orthographic projection, simply
 * flip the `x' coordinate of the projected points. Also, it is illegal
 * to pass dist == 0.0.
 */
fpp_t
fpp_init(geo_pos2_t center, double rot, double dist, const ellip_t *ellip,
    bool_t allow_inv)
{
	fpp_t fpp;

	VERIFY(dist != 0);
	fpp.allow_inv = allow_inv;
	fpp.ellip = ellip;
	if (ellip) {
		geo_pos2_t sph_ctr = GEO3_TO_GEO2(geo2sph(GEO2_TO_GEO3(center,
		    0), ellip));
		fpp.xlate = sph_xlate_init(sph_ctr, rot, B_FALSE);
		if (allow_inv)
			fpp.inv_xlate = sph_xlate_init(sph_ctr, rot, B_TRUE);
	} else {
		fpp.xlate = sph_xlate_init(center, rot, B_FALSE);
		if (allow_inv)
			fpp.inv_xlate = sph_xlate_init(center, rot, B_TRUE);
	}
	fpp.dist = dist;

	return (fpp);
}

/*
 * Constructs an orthographic projection. This is a flat plane projection with
 * the projection origin at +INFINITY. See `fpp_init' for more information.
 */
fpp_t
ortho_fpp_init(geo_pos2_t center, double rot, const ellip_t *ellip,
    bool_t allow_inv)
{
	return (fpp_init(center, rot, INFINITY, ellip, allow_inv));
}

/*
 * Constructs a gnomonic projection. This is a flat plane projection with
 * the origin at the Earth's center. See `fpp_init' for more information.
 */
fpp_t
gnomo_fpp_init(geo_pos2_t center, double rot, const ellip_t *ellip,
    bool_t allow_inv)
{
	return (fpp_init(center, rot, -EARTH_MSL, ellip, allow_inv));
}

/*
 * Constructs a stereographic projection. This is a projection with
 * the origin at the intersection of the projection axis and the surface
 * of the Earth opposite the projection plane's center point.
 * See `fpp_init' for more information.
 */
fpp_t
stereo_fpp_init(geo_pos2_t center, double rot, const ellip_t *ellip,
    bool_t allow_inv)
{
	return (fpp_init(center, rot, -2 * EARTH_MSL, ellip, allow_inv));
}

/*
 * Projects a point at `pos' according to the projection `proj' and returns
 * a 2D vector to the projected point's location on the projection plane.
 * If the specified point cannot be projected (because its projection falls
 * outside of the projection plane), NULL_VECT2 is returned instead.
 */
vect2_t
geo2fpp(geo_pos2_t pos, const fpp_t *fpp)
{
	vect3_t pos_v;
	vect2_t res_v;

	if (fpp->ellip != NULL)
		pos_v = geo2ecef(GEO2_TO_GEO3(pos, 0), fpp->ellip);
	else
		pos_v = sph2ecef(GEO2_TO_GEO3(pos, 0));
	pos_v = sph_xlate_vect(pos_v, &fpp->xlate);
	if (isfinite(fpp->dist)) {
		if (fpp->dist < 0.0 && pos_v.x <= fpp->dist + EARTH_MSL)
			return (NULL_VECT2);
		res_v.x = fpp->dist * (pos_v.y / (fpp->dist +
		    EARTH_MSL - pos_v.x));
		res_v.y = fpp->dist * (pos_v.z / (fpp->dist +
		    EARTH_MSL - pos_v.x));
	} else {
		res_v = VECT2(pos_v.y, pos_v.z);
	}

	return (res_v);
}

/*
 * Back-projects a point from a projection surface into spherical coordinate
 * space. N.B. since projection loses some information about the original
 * input point, back-projection is incomplete for projections where either:
 *	a) the projection origin was non-negative, or
 *	b) the projection origin was less than -EARTH_MSL
 * This means back-projection is only uniquely possible possible for
 * gnomonic, stereographic and other projections where the projection origin
 * lies "inside" the projected sphere. In case back-projection is not unique,
 * the point closer to the projection origin is returned.
 */
geo_pos2_t
fpp2geo(vect2_t pos, const fpp_t *fpp)
{
	vect3_t	v;
	vect3_t	o;
	vect3_t i[2];
	vect3_t r;
	int n;

	ASSERT(fpp->allow_inv);

	if (isfinite(fpp->dist)) {
		v = VECT3(-fpp->dist, pos.x, pos.y);
		o = VECT3(EARTH_MSL + fpp->dist, 0, 0);
	} else {
		/*
		 * For orthographic projections, we'll just pretend the
		 * projection point is *really* far away so that the
		 * error in the result is negligible.
		 */
		v = VECT3(-1e14, pos.x, pos.y);
		o = VECT3(1e14, 0, 0);
	}
	n = vect2sph_isect(v, o, ZERO_VECT3, EARTH_MSL, B_FALSE, i);
	if (n == 0) {
		/* Invalid input point, not a member of projection */
		return (NULL_GEO_POS2);
	}
	if (n == 2 && isfinite(fpp->dist)) {
		/*
		 * Two solutions on non-orthographic projections. Find the
		 * one that's closer to the projection origin while located
		 * between it and the projection plane and place it in i[0].
		 */
		if (fpp->dist >= -EARTH_MSL) {
			if (i[1].x > i[0].x)
				i[0] = i[1];
		} else {
			if (i[1].x < i[0].x)
				i[0] = i[1];
		}
	}
	/* Result is now in i[0]. Inv-xlate to global space & ecef2sph. */
	r = sph_xlate_vect(i[0], &fpp->inv_xlate);
	if (fpp->ellip != NULL)
		return (GEO3_TO_GEO2(ecef2geo(r, fpp->ellip)));
	else
		return (GEO3_TO_GEO2(ecef2sph(r)));
}

/*
 * Prepares a set of Lambert conformal conic projection parameters.
 *
 * @param reflat Reference latitude in degrees.
 * @param reflon Reference longitude in degrees.
 * @param stdpar1 First standard parallel in degrees.
 * @param stdpar2 Second standard parallel in degrees.
 *
 * @return The set of lcc parameters to pass to geo2lcc.
 */
lcc_t
lcc_init(double reflat, double reflon, double stdpar1, double stdpar2)
{
	double	phi0 = DEG2RAD(reflat);
	double	phi1 = DEG2RAD(stdpar1);
	double	phi2 = DEG2RAD(stdpar2);
	lcc_t	lcc;

	lcc.reflat = DEG2RAD(reflat);
	lcc.reflon = DEG2RAD(reflon);

	if (stdpar1 == stdpar2)
		lcc.n = sin(phi1);
	else
		lcc.n = log(cos(phi1) * sec(phi2)) / log(tan(M_PI / 4.0 +
		    phi2 / 2.0) * cot(M_PI / 4.0 + phi1 / 2.0));
	lcc.F = (cos(phi1) * pow(tan(M_PI / 4.0 * phi1 / 2.0), lcc.n)) /
	    lcc.n;
	lcc.rho0 = lcc.F * pow(cot(M_PI / 4.0 + phi0 / 2.0), lcc.n);

	return (lcc);
}

/*
 * Projects a point at `pos' using the projection `lcc'.
 */
vect2_t
geo2lcc(geo_pos2_t pos, const lcc_t *lcc)
{
	vect2_t		result;
	double		rho;
	double		lat = DEG2RAD(pos.lat);
	double		lon = DEG2RAD(pos.lon);

	rho = lcc->F * pow(cot(M_PI / 4 + lat / 2), lcc->n);
	result.x = rho * sin(lon - lcc->reflon);
	result.y = lcc->rho0 - rho * cos(lcc->n * (lat - lcc->reflat));

	return (result);
}

/*
 * Allocates a new generic Bezier curve structure with n_pts points.
 */
bezier_t *
bezier_alloc(size_t n_pts)
{
	bezier_t *curve = malloc(sizeof (*curve));

	curve->n_pts = n_pts;
	curve->pts = calloc(sizeof (vect2_t), n_pts);

	return (curve);
}

/*
 * Frees a generic Bezier curve previous allocated with bezier_alloc.
 */
void
bezier_free(bezier_t *curve)
{
	free(curve->pts);
	free(curve);
}

/*
 * Calculates the value of a function defined by a set of quadratic bezier
 * curve segments.
 *
 * @param x Function input value.
 * @param func The set of quadratic bezier curves defining the function.
 *	Please note that since this is a function, curve segments may
 *	not overlap. This is to guarantee that at any point 'x' the
 *	function resolves to one value.
 *
 * @return The function value at point 'x'. If the point is beyond the
 *	edges of the bezier curve segments describing the function, the
 *	'y' value of the first or last curve point is returned, i.e.
 *	beyond the curve boundaries, the function is assumed to be flat.
 */
double
quad_bezier_func(double x, const bezier_t *func)
{
	ASSERT(func->n_pts >= 3 || func->n_pts % 3 == 2);
	/* Check boundary conditions */
	if (x < func->pts[0].x)
		return (func->pts[0].y);
	else if (x > func->pts[func->n_pts - 1].x)
		return (func->pts[func->n_pts - 1].y);

	/* Point lies on a curve segment */
	for (size_t i = 0; i + 2 < func->n_pts; i += 2) {
		if (func->pts[i].x <= x) {
			vect2_t p0 = func->pts[i], p1 = func->pts[i + 1];
			vect2_t p2 = func->pts[i + 2];
			double y, t, ts[2];
			unsigned n;

			/*
			 * Quadratic Bezier curves are defined as follows:
			 *
			 * B(t) = (1-t)^2.P0 + 2(1-t)t.P1 + t^2.P2
			 *
			 * Where 't' is a number 0.0 - 1.0, increasing along
			 * curve as it progresses from P0 to P2 (P{0,1,2} are
			 * 2D vectors). However, since we are using the curves
			 * as functions, we first need to find which 't' along
			 * the curve corresponds to our 'x' input.
			 * Geometrically, quadratic Bezier curves are defined
			 * as a set of points `P' which satisfy:
			 *
			 * 1) let `A' be a point that is on the line between
			 *    P0 and P1. Its distance from P0 is `t' times
			 *    the distance of P1 from P0.
			 * 2) let `B' be a point that is on the line between
			 *    P1 and P2. Its distance from P1 is `t' times
			 *    the distance of P2 from P1.
			 * 3) `P' is a point that is on a line between A and B.
			 *    Its distance from A is `t' times the distance
			 *    of B from A.
			 *
			 * The x coordinate of `P' corresponds to the input
			 * `x' argument. Transforming this into an analytical
			 * representation and simplifying to only consider
			 * the X axis, we get (here 'a', 'b' represent the
			 * respective 2D vector's X coordinate only):
			 *
			 * a = t(p1 - p0) + p0
			 * b = t(p2 - p1) + p1
			 * x = t(b - a) + a
			 * x = t(t(p2 - p1) + p1 - t(p1 - p0) + p0) +
			 *     + t(p1 - p0) + p0
			 *
			 * Rearranging, we obtain this quadratic equation:
			 *
			 * 0 = (p2 - 2.p1 + p0)t^2 + 2(p1 - p0)t + p0 - x
			 *
			 * The solution we're interested in is only the one
			 * between 0.0 - 1.0 (inclusive). We then take that
			 * value and use it in the bezier curve equation at
			 * the top to obtain the function value at point 'x'.
			 */
			ASSERT3F(p0.x, <, p2.x);
			ASSERT3F(p0.x, <=, p1.x);
			ASSERT3F(p1.x, <=, p2.x);
			n = quadratic_solve(p2.x - 2 * p1.x + p0.x,
			    2 * (p1.x - p0.x), p0.x - x, ts);
			ASSERT(n != 0);
			if (ts[0] >= 0 && ts[0] <= 1) {
				t = ts[0];
			} else if (n == 2 && ts[1] >= 0 && ts[1] <= 1) {
				t = ts[1];
			} else {
				continue;
			}
			ASSERT3F(t, >=, 0.0);
			ASSERT3F(t, <=, 1.0);
			y = POW2(1 - t) * p0.y + 2 * (1 - t) * t * p1.y +
			    POW2(t) * p2.y;

			return (y);
		}
	}
	VERIFY(0);
}

/*
 * Calculates the value of a function `g' defined as the inverse of a function
 * `f' which is defined by a set of quadratic bezier curve segments:
 * g(f(x)) = x
 * IOW, this function is the inverse of quad_bezier_func, i.e. it looks for
 * an unknown `x' input to quad_bezier_func that produces a known `y' output.
 * Please note that the `func' argument of quad_bezier_func need not be a
 * bijective function definition, so quad_bezier_func_inv can return multiple
 * `x' values that all map to the given `y' value. As a special case, if the
 * `func' definition includes a constant portion (i.e. a potentially infinite
 * number of `x' values corresponds to a given `y' value),
 * quad_bezier_func_inv returns NULL and sets n_xs to SIZE_MAX;
 *
 * @param y Output value of the function `func' for which to look for a
 *	suitable `x' input value.
 * @param func The set of quadratic bezier curve defining the function with
 *	the same constraints as in quad_bezier_func.
 * @param n_xs Output parameter which will be filled with the number of
 *	`x' input points returned.
 *
 * @return An array of `x' values corresponding to the `y' argument. The
 *	number of elements in this array is filled in `n_xs'. Caller is
 *	responsible for freeing this array.
 */
double *
quad_bezier_func_inv(double y, const bezier_t *func, size_t *n_xs)
{
	double *xs = NULL;
	size_t num_xs = 0;

	for (size_t i = 0; i + 2 < func->n_pts; i += 2) {
		vect2_t p0 = func->pts[i], p1 = func->pts[i + 1];
		vect2_t p2 = func->pts[i + 2];
		double t, ts[2];
		unsigned n;

		if (p0.y == p1.y && p1.y == p2.y) {
			/* infinite number of results */
			free(xs);
			*n_xs = SIZE_MAX;
			return (NULL);
		}

		/*
		 * This quadratic equation is essentially the same as in
		 * quad_bezier_func, except we used the `y' coordinates to
		 * derive `t' and then calculate a corresponding `x' value.
		 */
		n = quadratic_solve(p2.y - 2 * p1.y + p0.y, 2 * (p1.y - p0.y),
		    p0.y - y, ts);
		for (unsigned i = 0; i < n; i++) {
			t = ts[i];
			if (t < 0.0 || t > 1.0)
				continue;
			num_xs++;
			xs = realloc(xs, num_xs * sizeof (*xs));
			xs[num_xs - 1] = POW2(1 - t) * p0.x +
			    2 * (1 - t) * t * p1.x + POW2(t) * p2.x;
		}
	}

	*n_xs = num_xs;
	return (xs);
}
