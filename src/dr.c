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

#include <stdarg.h>
#include <stdio.h>

#include "assert.h"

#include "dr.h"

#define	MAX_DR_NAME_LEN	256

void
bp_dr_find(dr_t *dr, const char *fmt, ...)
{
	char name[MAX_DR_NAME_LEN];
	va_list ap;

	va_start(ap, fmt);
	vsnprintf(name, sizeof (name), fmt, ap);
	va_end(ap);

	dr->dr = XPLMFindDataRef(name);
	ASSERT(dr->dr != NULL);
	dr->type = XPLMGetDataRefTypes(dr->dr);
	dr->writable = XPLMCanWriteDataRef(dr->dr);
}

int
bp_dr_geti(dr_t *dr)
{
	switch (dr->type) {
	case xplmType_Int:
		return (XPLMGetDatai(dr->dr));
	case xplmType_Float:
		return (XPLMGetDataf(dr->dr));
	case xplmType_Double:
		return (XPLMGetDatad(dr->dr));
	case xplmType_FloatArray:
	case xplmType_IntArray:
	case xplmType_Data: {
		int i;
		VERIFY3U(bp_dr_getvi(dr, &i, 0, 1), ==, 1);
		return (i);
	}
	default:
		VERIFY(0);
	}
}

void
bp_dr_seti(dr_t *dr, int i)
{
	VERIFY(dr->writable);
	switch (dr->type) {
	case xplmType_Int:
		XPLMSetDatai(dr->dr, i);
		break;
	case xplmType_Float:
		XPLMSetDataf(dr->dr, i);
		break;
	case xplmType_Double:
		XPLMSetDatad(dr->dr, i);
		break;
	case xplmType_FloatArray:
	case xplmType_IntArray:
	case xplmType_Data:
		bp_dr_setvi(dr->dr, &i, 0, 1);
	default:
		VERIFY(0);
	}
}

double
bp_dr_getf(dr_t *dr)
{
	switch (dr->type) {
	case xplmType_Int:
		return (XPLMGetDatai(dr->dr));
	case xplmType_Float:
		return (XPLMGetDataf(dr->dr));
	case xplmType_Double:
		return (XPLMGetDatad(dr->dr));
	case xplmType_FloatArray:
	case xplmType_IntArray:
	case xplmType_Data: {
		double f;
		VERIFY3U(bp_dr_getvf(dr, &f, 0, 1), ==, 1);
		return (f);
	}
	default:
		VERIFY(0);
	}
}

void
bp_dr_setf(dr_t *dr, double f)
{
	VERIFY(dr->writable);
	switch (dr->type) {
	case xplmType_Int:
		XPLMSetDatai(dr->dr, f);
		break;
	case xplmType_Float:
		XPLMSetDataf(dr->dr, f);
		break;
	case xplmType_Double:
		XPLMSetDatad(dr->dr, f);
		break;
	case xplmType_FloatArray:
	case xplmType_IntArray:
	case xplmType_Data: {
		bp_dr_setvf(dr->dr, &f, 0, 1);
	}
	default:
		VERIFY(0);
	}
}

int
bp_dr_getvi(dr_t *dr, int *i, unsigned off, unsigned num)
{
	switch (dr->type) {
	case xplmType_IntArray:
		return (XPLMGetDatavi(dr->dr, i, off, num));
	case xplmType_FloatArray: {
		float f[num];
		int n = XPLMGetDatavf(dr->dr, f, off, num);
		for (int x = 0; x < n; x++)
			i[x] = f[x];
		return (n);
	}
	case xplmType_Data: {
		uint8_t u[num];
		int n = XPLMGetDatab(dr->dr, u, off, num);
		for (int x = 0; x < n; x++)
			i[x] = u[x];
		return (n);
	}
	default:
		VERIFY(0);
	}
}

void
bp_dr_setvi(dr_t *dr, int *i, unsigned off, unsigned num)
{
	VERIFY(dr->writable);
	switch (dr->type) {
	case xplmType_IntArray:
		XPLMSetDatavi(dr->dr, i, off, num);
		break;
	case xplmType_FloatArray: {
		float f[num];
		for (unsigned x = 0; x < num; x++)
			f[x] = i[x];
		XPLMSetDatavf(dr->dr, f, off, num);
		break;
	}
	case xplmType_Data: {
		uint8_t u[num];
		for (unsigned x = 0; x < num; x++)
			u[x] = i[x];
		XPLMSetDatab(dr->dr, u, off, num);
		break;
	}
	default:
		VERIFY(0);
	}
}

int
bp_dr_getvf(dr_t *dr, double *df, unsigned off, unsigned num)
{
	switch (dr->type) {
	case xplmType_IntArray: {
		int i[num];
		int n = XPLMGetDatavi(dr->dr, i, off, num);
		for (int x = 0; x < n; x++)
			df[x] = i[x];
		return (n);
	}
	case xplmType_FloatArray: {
		float f[num];
		int n = XPLMGetDatavf(dr->dr, f, off, num);
		for (int x = 0; x < n; x++)
			df[x] = f[x];
		return (n);
	}
	case xplmType_Data: {
		uint8_t u[num];
		int n = XPLMGetDatab(dr->dr, u, off, num);
		for (int x = 0; x < n; x++)
			df[x] = u[x];
		return (n);
	}
	default:
		VERIFY(0);
	}
}

void
bp_dr_setvf(dr_t *dr, double *df, unsigned off, unsigned num)
{
	VERIFY(dr->writable);
	switch (dr->type) {
	case xplmType_IntArray: {
		int i[num];
		for (unsigned x = 0; x < num; x++)
			i[x] = df[x];
		XPLMSetDatavi(dr->dr, i, off, num);
		break;
	}
	case xplmType_FloatArray: {
		float f[num];
		for (unsigned x = 0; x < num; x++)
			f[x] = df[x];
		XPLMSetDatavf(dr->dr, f, off, num);
		break;
	}
	case xplmType_Data: {
		uint8_t u[num];
		for (unsigned x = 0; x < num; x++)
			u[x] = df[x];
		XPLMSetDatab(dr->dr, u, off, num);
		break;
	}
	default:
		VERIFY(0);
	}
}
