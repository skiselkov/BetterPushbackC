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

#ifndef	_XTCAS_TYPES_H_
#define	_XTCAS_TYPES_H_

#include <stdint.h>
#include <limits.h>

#ifdef	__cplusplus
extern "C" {
#endif

typedef unsigned int bool_t;

#ifndef	B_TRUE
#define	B_TRUE	1
#define	B_FALSE	0
#endif

#ifdef	__cplusplus
}
#endif

#endif	/* _XTCAS_TYPES_H_ */
