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

#ifndef	_BP_MATH_H_
#define	_BP_MATH_H_

#include "config.h"

#ifdef	__cplusplus
extern "C" {
#endif

#define	POW3(x)	((x) * (x) * (x))
#define	POW2(x)	((x) * (x))
#define	ROUND_ERROR	1e-10

#define	quadratic_solve		SYMBOL_PREFIX(quadratic_solve)
unsigned quadratic_solve(double a, double b, double c, double x[2]);

#ifdef	__cplusplus
}
#endif

#endif	/* _BP_GEOM_H_ */
