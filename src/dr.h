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

#include "config.h"
#include <acfutils/types.h>

#ifdef	__cplusplus
extern "C" {
#endif

#define	DR_MAX_NAME_LEN	128

typedef struct {
	char		name[DR_MAX_NAME_LEN];
	XPLMDataRef	dr;
	XPLMDataTypeID	type;
	bool_t		writable;
} dr_t;

#define	dr_init		SYMBOL_PREFIX(dr_init)
void dr_init(dr_t *dr, const char *fmt, ...) PRINTF_ATTR(2);

#define	dr_geti		SYMBOL_PREFIX(dr_geti)
int dr_geti(dr_t *dr);

#define	dr_set		SYMBOL_PREFIX(dr_seti)
void dr_seti(dr_t *dr, int i);

#define	dr_getf		SYMBOL_PREFIX(dr_getf)
double dr_getf(dr_t *dr);
#define	dr_setf		SYMBOL_PREFIX(dr_setf)
void dr_setf(dr_t *dr, double f);

#define	dr_getvi	SYMBOL_PREFIX(dr_getvi)
int dr_getvi(dr_t *dr, int *i, unsigned off, unsigned num);
#define	dr_setvi	SYMBOL_PREFIX(dr_setvi)
void dr_setvi(dr_t *dr, int *i, unsigned off, unsigned num);

#define	dr_getvf	SYMBOL_PREFIX(dr_getvf)
int dr_getvf(dr_t *dr, double *df, unsigned off, unsigned num);
#define	dr_setvf	SYMBOL_PREFIX(dr_setvf)
void dr_setvf(dr_t *dr, double *df, unsigned off, unsigned num);

#ifdef	__cplusplus
}
#endif

#endif	/* _DR_H_ */
