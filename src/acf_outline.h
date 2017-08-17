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

#ifndef	_ACF_OUTLINE_H_
#define	_ACF_OUTLINE_H_

#include <acfutils/geom.h>

#ifdef	__cplusplus
extern "C" {
#endif

typedef struct {
	double	semispan;	/* half wingspan (meters) */
	double	length;		/* total aircraft length (meters) */
	vect2_t	*pts;		/* outline point coordinates (meters) */
	size_t	num_pts;	/* number of elements in `pts' */
} acf_outline_t;

acf_outline_t *acf_outline_read(const char *filename, int nw_i, double nw_z_dr);
void acf_outline_free(acf_outline_t *outline);

#ifdef	__cplusplus
}
#endif

#endif	/* _ACF_OUTLINE_H_ */
