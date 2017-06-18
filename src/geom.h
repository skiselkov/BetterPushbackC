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

#ifndef	_XTCAS_GEOM_H_
#define	_XTCAS_GEOM_H_

#include <stdlib.h>
#include <math.h>

#include "types.h"

#ifdef	__cplusplus
extern "C" {
#endif

typedef struct {
	double	lat;
	double	lon;
} geo_pos2_t;

typedef struct {
	double	lat;
	double	lon;
	double	elev;
} geo_pos3_t;

typedef struct {
	double	x;
	double	y;
	double	z;
} vect3_t;

typedef struct {
	double	x;
	double	y;
} vect2_t;

typedef struct {
	double	a;	/* semi-major axis of the ellipsoid in meters */
	double	b;	/* semi-minor axis of the ellipsoid in meters */
	double	f;	/* flattening */
	double	ecc;	/* first eccentricity */
	double	ecc2;	/* first eccentricity squared */
	double	r;	/* mean radius in meters */
} ellip_t;

typedef struct {
	size_t	n_pts;
	vect2_t	*pts;
} bezier_t;

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
#define	GEO_POS2(lat, lon)		((geo_pos2_t){(lat), (lon)})
#define	GEO_POS3(lat, lon, elev)	((geo_pos3_t){(lat), (lon), (elev)})
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

#define	GEO2_TO_GEO3(v, a)	((geo_pos3_t){(v).lat, (v).lon, (a)})
#define	GEO3_TO_GEO2(v)		((geo_pos2_t){(v).lat, (v).lon})

#define	EARTH_MSL		6371200		/* meters */
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
 * The standard WGS84 ellipsoid.
 */
const ellip_t wgs84;

/*
 * Small helpers.
 */
bool_t is_on_arc(double angle_x, double angle1, double angle2, bool_t cw);

/*
 * Vector math.
 */
double vect3_abs(vect3_t a);
double vect2_abs(vect2_t a);
double vect2_dist(vect2_t a, vect2_t b);
vect3_t vect3_set_abs(vect3_t a, double abs);
vect2_t vect2_set_abs(vect2_t a, double abs);
vect3_t vect3_unit(vect3_t a, double *l);
vect2_t vect2_unit(vect2_t a, double *l);

vect3_t vect3_add(vect3_t a, vect3_t b);
vect2_t vect2_add(vect2_t a, vect2_t b);
vect3_t vect3_sub(vect3_t a, vect3_t b);
vect2_t vect2_sub(vect2_t a, vect2_t b);
vect3_t vect3_scmul(vect3_t a, double b);
vect2_t vect2_scmul(vect2_t a, double b);
double vect3_dotprod(vect3_t a, vect3_t b);
double vect2_dotprod(vect2_t a, vect2_t b);
vect3_t vect3_xprod(vect3_t a, vect3_t b);
vect3_t vect3_mean(vect3_t a, vect3_t b);

vect2_t vect2_norm(vect2_t v, bool_t right);
vect2_t vect2_rot(vect2_t v, double angle);
vect2_t vect2_neg(vect2_t v);

/*
 * Spherical, geodesic and ECEF coordinate conversion.
 */
ellip_t ellip_init(double semi_major, double semi_minor, double flattening);
geo_pos3_t geo2sph(geo_pos3_t pos, const ellip_t *ellip);
vect3_t geo2ecef(geo_pos3_t pos, const ellip_t *ellip);
geo_pos3_t ecef2geo(vect3_t pos, const ellip_t *ellip);
geo_pos3_t ecef2sph(vect3_t v);
vect3_t sph2ecef(geo_pos3_t pos);

/*
 * Interesections.
 */
unsigned vect2sph_isect(vect3_t v, vect3_t o, vect3_t c, double r,
    bool_t confined, vect3_t i[2]);
unsigned vect2circ_isect(vect2_t v, vect2_t o, vect2_t c, double r,
    bool_t confined, vect2_t i[2]);
vect2_t vect2vect_isect(vect2_t da, vect2_t oa, vect2_t db, vect2_t ob,
    bool_t confined);
unsigned circ2circ_isect(vect2_t ca, double ra, vect2_t cb, double rb,
    vect2_t i[2]);
unsigned vect2poly_isect(vect2_t a, vect2_t oa, const vect2_t *poly,
    size_t poly_n);
bool_t point_in_poly(vect2_t pt, vect2_t *poly, size_t poly_n);

/*
 * Converting between headings and direction vectors on a 2D plane.
 */
vect2_t hdg2dir(double truehdg);
double dir2hdg(vect2_t dir);

/*
 * Calculating coordinate displacement & radial intersection.
 */
geo_pos2_t geo_displace(const ellip_t *ellip, geo_pos2_t pos, double truehdg,
    double dist);
geo_pos2_t geo_displace_dir(const ellip_t *ellip, geo_pos2_t pos, vect2_t dir,
    double dist);

/*
 * Geometry parser & validator helpers.
 */
bool_t geo_pos2_from_str(const char *lat, const char *lon, geo_pos2_t *pos);
bool_t geo_pos3_from_str(const char *lat, const char *lon, const char *elev,
    geo_pos3_t *pos);

/*
 * Spherical coordinate system translation.
 */
typedef struct {
	double	sph_matrix[3 * 3];
	double	rot_matrix[2 * 2];
	bool_t	inv;
} sph_xlate_t;

sph_xlate_t sph_xlate_init(geo_pos2_t displacement, double rotation,
    bool_t inv);
geo_pos2_t sph_xlate(geo_pos2_t pos, const sph_xlate_t *xlate);
vect3_t sph_xlate_vect(vect3_t pos, const sph_xlate_t *xlate);

/*
 * Great circle functions.
 */
double gc_distance(geo_pos2_t start, geo_pos2_t end);
double gc_point_hdg(geo_pos2_t start, geo_pos2_t end, double arg);

/*
 * Generic spherical - to - flat-plane projections.
 */
typedef struct {
	const ellip_t	*ellip;
	sph_xlate_t	xlate;
	sph_xlate_t	inv_xlate;
	bool_t		allow_inv;
	double		dist;
} fpp_t;

fpp_t fpp_init(geo_pos2_t center, double rot, double dist,
    const ellip_t *ellip, bool_t allow_inv);
fpp_t ortho_fpp_init(geo_pos2_t center, double rot, const ellip_t *ellip,
    bool_t allow_inv);
fpp_t gnomo_fpp_init(geo_pos2_t center, double rot, const ellip_t *ellip,
    bool_t allow_inv);
fpp_t stereo_fpp_init(geo_pos2_t center, double rot, const ellip_t *ellip,
    bool_t allow_inv);
vect2_t geo2fpp(geo_pos2_t pos, const fpp_t *fpp);
geo_pos2_t fpp2geo(vect2_t pos, const fpp_t *fpp);

/*
 * Lambert conformal conic projection
 */
typedef struct {
	double	reflat;
	double	reflon;
	double	n;
	double	F;
	double	rho0;
} lcc_t;

lcc_t lcc_init(double reflat, double reflon, double stdpar1, double stdpar2);
vect2_t geo2lcc(geo_pos2_t pos, const lcc_t *lcc);

/*
 *  Bezier curve functions.
 */
bezier_t *bezier_alloc(size_t num_pts);
void bezier_free(bezier_t *curve);
double quad_bezier_func(double x, const bezier_t *func);
double *quad_bezier_func_inv(double y, const bezier_t *func, size_t *n_xs);

#ifdef	__cplusplus
}
#endif

#endif	/* _XTCAS_GEOM_H_ */
