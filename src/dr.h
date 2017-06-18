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

#ifndef	_DR_H_
#define	_DR_H_

#include <XPLMDataAccess.h>
#include "types.h"

#ifdef	__cplusplus
extern "C" {
#endif

typedef struct {
	XPLMDataRef	dr;
	XPLMDataTypeID	type;
	bool_t		writable;
} dr_t;

void bp_dr_find(dr_t *dr, const char *fmt, ...);

int bp_dr_geti(dr_t *dr);
void bp_dr_seti(dr_t *dr, int i);

double bp_dr_getf(dr_t *dr);
void bp_dr_setf(dr_t *dr, double f);

int bp_dr_getvi(dr_t *dr, int *i, unsigned off, unsigned num);
void bp_dr_setvi(dr_t *dr, int *i, unsigned off, unsigned num);

int bp_dr_getvf(dr_t *dr, double *df, unsigned off, unsigned num);
void bp_dr_setvf(dr_t *dr, double *df, unsigned off, unsigned num);

#ifdef	__cplusplus
}
#endif

#endif	/* _DR_H_ */
