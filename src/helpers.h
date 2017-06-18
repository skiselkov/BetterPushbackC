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

#ifndef	_XTCAS_HELPERS_H_
#define	_XTCAS_HELPERS_H_

#include <stdarg.h>
#include <assert.h>
#include <stdio.h>
#include <math.h>

#include "types.h"

#ifdef	__cplusplus
extern "C" {
#endif


#if	defined(__GNUC__) || defined(__clang__)
#define	BUILD_DIRSEP	'/'
#define	PRINTF_ATTR(x)	__attribute__ ((format (printf, x, x + 1)))
#ifndef	BSWAP32
#define	BSWAP16(x)	__builtin_bswap16((x))
#define	BSWAP32(x)	__builtin_bswap32((x))
#define	BSWAP64(x)	__builtin_bswap64((x))
#endif	/* BSWAP32 */
#else	/* !defined(__GNUC__) && !defined(__clang__) */
#define	BUILD_DIRSEP	'\\'
#define	PRINTF_ATTR(x)
#ifndef	BSWAP32
#define	BSWAP16(x)	\
	((((x) & 0xff00) >> 8) | \
	(((x) & 0x00ff) << 8))
#define	BSWAP32(x)	\
	(((x) >> 24) | \
	(((x) & 0x0000ff00) << 8) | \
	(((x) & 0x00ff0000) >> 8) | \
	((x) << 24))
#define	BSWAP64(x)	\
	(((x) >> 56) | \
	(((x) & 0x000000000000ff00llu) << 40) | \
	(((x) & 0x0000000000ff0000llu) << 24) | \
	(((x) & 0x00000000ff000000llu) << 8) | \
	(((x) & 0x000000ff00000000llu) >> 8) | \
	(((x) & 0x0000ff0000000000llu) >> 24) | \
	(((x) & 0x00ff000000000000llu) >> 40) | \
	((x) << 56))
#endif	/* BSWAP32 */
#endif	/* !defined(__GNUC__) && !defined(__clang__) */

#if	IBM
#define	DIRSEP		'\\'
#define	DIRSEP_S	"\\"    /* DIRSEP as a string */
#else	/* !IBM */
#define	DIRSEP		'/'
#define	DIRSEP_S	"/"     /* DIRSEP as a string */
#define	MAX_PATH	512
#endif	/* !IBM */

/* Minimum/Maximum allowable elevation AMSL of anything */
#define	MIN_ELEV	-2000.0
#define	MAX_ELEV	30000.0

/* Minimum/Maximum allowable altitude AMSL of anything */
#define	MIN_ALT		-2000.0
#define	MAX_ALT		100000.0

/* Maximum valid speed of anything */
#define	MAX_SPD		1000.0

/* Minimum/Maximum allowable arc radius on any procedure */
#define	MIN_ARC_RADIUS	0.1
#define	MAX_ARC_RADIUS	100.0

#define	UNUSED_ATTR		__attribute__((unused))
#define	UNUSED(x)		do { (void) sizeof (x); } while (0)

/*
 * Length and velocity unit conversions.
 */
#define	FEET2MET(x)	((x) * 0.3048)		/* feet to meters */
#define	MET2FEET(x)	((x) * 3.2808398950131)	/* meters to feet */
#define	NM2MET(x)	((x) * 1852)		/* nautical miles to meters */
#define	MET2NM(x)	((x) / 1852.0)		/* meters to nautical miles */
#define	KT2MPS(k)	(NM2MET(k) / 3600)	/* knots to m/s */
#define	MPS2KT(k)	(MET2NM(k) * 3600)	/* m/s to knots */
#define	FPM2MPS(f)	FEET2MET((f) / 60.0)	/* ft.min^-1 to m.s^-1 */
#define	MPS2FPM(m)	MET2FEET((m) * 60.0)	/* m.s^-1 to ft.min^-1 */

/* generic parser validator helpers */

static inline bool_t
is_valid_lat(double lat)
{
	return (lat <= 90.0 && lat >= -90.0);
}

static inline bool_t
is_valid_lon(double lon)
{
	return (lon <= 180.0 && lon >= -180.0);
}

static inline bool_t
is_valid_elev(double elev)
{
	return (elev >= MIN_ELEV && elev <= MAX_ELEV);
}

static inline bool_t
is_valid_alt(double alt)
{
	return (alt >= MIN_ALT && alt <= MAX_ALT);
}

static inline bool_t
is_valid_spd(double spd)
{
	return (spd >= 0.0 && spd <= MAX_SPD);
}

static inline bool_t
is_valid_hdg(double hdg)
{
	return (hdg >= 0.0 && hdg <= 360.0);
}

static inline double
normalize_hdg(double hdg)
{
	while (hdg < 0)
		hdg += 360;
	while (hdg >= 360)
		hdg -= 360;
	return (hdg);
}

double rel_hdg(double hdg1, double hdg2);

static inline bool_t
is_valid_arc_radius(double radius)
{
	return (radius >= MIN_ARC_RADIUS && radius <= MAX_ARC_RADIUS);
}

static inline bool_t
is_valid_bool(bool_t b)
{
	return (b == B_FALSE || b == B_TRUE);
}

bool_t is_valid_vor_freq(double freq_mhz);
bool_t is_valid_loc_freq(double freq_mhz);
bool_t is_valid_ndb_freq(double freq_khz);
bool_t is_valid_tacan_freq(double freq_mhz);
bool_t is_valid_rwy_ID(const char *rwy_ID);

/* CSV file & string processing helpers */
ssize_t parser_get_next_line(FILE *fp, char **linep, size_t *linecap,
    size_t *linenum);
ssize_t explode_line(char *line, char delim, char **comps, size_t capacity);
void strip_space(char *line);
void append_format(char **str, size_t *sz, const char *format, ...)
    PRINTF_ATTR(3);

char *mkpathname(const char *comp, ...);
char *mkpathname_v(const char *comp, va_list ap);

void my_strlcpy(char *restrict dest, const char *restrict src, size_t cap);
#if	IBM
ssize_t getline(char **lineptr, size_t *n, FILE *stream);
#endif	/* IBM */

#if	defined(__GNUC__) || defined(__clang__)
#define	highbit64(x)	(64 - __builtin_clzll(x) - 1)
#define	highbit32(x)	(32 - __builtin_clzll(x) - 1)
#else
#error	"Compiler platform unsupported, please add highbit definition"
#endif

/*
 * return x rounded up to the nearest power-of-2.
 */
#define	P2ROUNDUP(x)	(-(-(x) & -(1 << highbit64(x))))
/* Round `x' to the nearest multiple of `y' */
static inline double
roundmul(double x, double y)
{
	return (round(x / y) * y);
}
/* Round `x' DOWN to the nearest multiple of `y' */
static inline double
floormul(double x, double y)
{
	return (floor(x / y) * y);
}
#if	!defined(MIN) && !defined(MAX) && !defined(AVG)
#define	MIN(x, y)	((x) < (y) ? (x) : (y))
#define	MAX(x, y)	((x) > (y) ? (x) : (y))
#define	AVG(x, y)	(((x) + (y)) / 2)
#endif	/* MIN or MAX */

#define	USEC2SEC(sec)	(sec / 1000000.0)
#define	SEC2USEC(sec)	(sec * 1000000ll)

#ifdef	__cplusplus
}
#endif

#endif	/* _XTCAS_HELPERS_H_ */
