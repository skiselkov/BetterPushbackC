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

void
dr_init(dr_t *dr, const char *fmt, ...)
{
	va_list ap;

	va_start(ap, fmt);
	vsnprintf(dr->name, sizeof (dr->name), fmt, ap);
	va_end(ap);

	dr->dr = XPLMFindDataRef(dr->name);
	ASSERT_MSG(dr->dr != NULL, "%s", dr->name);
	dr->type = XPLMGetDataRefTypes(dr->dr);
	dr->writable = XPLMCanWriteDataRef(dr->dr);
}

int
dr_geti(dr_t *dr)
{
	if (dr->type & xplmType_Int)
		return (XPLMGetDatai(dr->dr));
	if (dr->type & xplmType_Float)
		return (XPLMGetDataf(dr->dr));
	if (dr->type & xplmType_Double)
		return (XPLMGetDatad(dr->dr));
	if (dr->type & (xplmType_FloatArray | xplmType_IntArray |
	    xplmType_Data)) {
		int i;
		VERIFY3U(dr_getvi(dr, &i, 0, 1), ==, 1);
		return (i);
	}
	VERIFY_MSG(0, "dr %s has bad type %x", dr->name, dr->type);
}

void
dr_seti(dr_t *dr, int i)
{
	ASSERT_MSG(dr->writable, "%s", dr->name);
	if (dr->type & xplmType_Int)
		XPLMSetDatai(dr->dr, i);
	else if (dr->type & xplmType_Float)
		XPLMSetDataf(dr->dr, i);
	else if (dr->type & xplmType_Double)
		XPLMSetDatad(dr->dr, i);
	else if (dr->type & (xplmType_FloatArray | xplmType_IntArray |
	    xplmType_Data))
		dr_setvi(dr, &i, 0, 1);
	else
		VERIFY_MSG(0, "dr %s has bad type %x", dr->name, dr->type);
}

double
dr_getf(dr_t *dr)
{
	if (dr->type & xplmType_Int)
		return (XPLMGetDatai(dr->dr));
	if (dr->type & xplmType_Float)
		return (XPLMGetDataf(dr->dr));
	if (dr->type & xplmType_Double)
		return (XPLMGetDatad(dr->dr));
	if (dr->type & (xplmType_FloatArray | xplmType_IntArray |
	    xplmType_Data)) {
		double f;
		VERIFY3U(dr_getvf(dr, &f, 0, 1), ==, 1);
		return (f);
	}
	VERIFY_MSG(0, "dr %s has bad type %x", dr->name, dr->type);
}

void
dr_setf(dr_t *dr, double f)
{
	ASSERT_MSG(dr->writable, "%s", dr->name);
	if (dr->type & xplmType_Int)
		XPLMSetDatai(dr->dr, f);
	else if (dr->type & xplmType_Float)
		XPLMSetDataf(dr->dr, f);
	else if (dr->type & xplmType_Double)
		XPLMSetDatad(dr->dr, f);
	else if (dr->type & (xplmType_FloatArray | xplmType_IntArray |
	    xplmType_Data))
		dr_setvf(dr, &f, 0, 1);
	else
		VERIFY_MSG(0, "dr %s has bad type %x", dr->name, dr->type);
}

int
dr_getvi(dr_t *dr, int *i, unsigned off, unsigned num)
{
	if (dr->type & xplmType_IntArray)
		return (XPLMGetDatavi(dr->dr, i, off, num));
	if (dr->type & xplmType_FloatArray) {
		float f[num];
		int n = XPLMGetDatavf(dr->dr, f, off, num);
		for (int x = 0; x < n; x++)
			i[x] = f[x];
		return (n);
	}
	if (dr->type & xplmType_Data) {
		uint8_t u[num];
		int n = XPLMGetDatab(dr->dr, u, off, num);
		for (int x = 0; x < n; x++)
			i[x] = u[x];
		return (n);
	}
	VERIFY_MSG(0, "dr %s has bad type %x", dr->name, dr->type);
}

void
dr_setvi(dr_t *dr, int *i, unsigned off, unsigned num)
{
	ASSERT_MSG(dr->writable, "%s", dr->name);
	if (dr->type & xplmType_IntArray) {
		XPLMSetDatavi(dr->dr, i, off, num);
	} else if (dr->type & xplmType_FloatArray) {
		float f[num];
		for (unsigned x = 0; x < num; x++)
			f[x] = i[x];
		XPLMSetDatavf(dr->dr, f, off, num);
	} else if (dr->type & xplmType_Data) {
		uint8_t u[num];
		for (unsigned x = 0; x < num; x++)
			u[x] = i[x];
		XPLMSetDatab(dr->dr, u, off, num);
	} else {
		VERIFY_MSG(0, "dr %s has bad type %x", dr->name, dr->type);
	}
}

int
dr_getvf(dr_t *dr, double *df, unsigned off, unsigned num)
{
	if (dr->type & xplmType_IntArray) {
		int i[num];
		int n = XPLMGetDatavi(dr->dr, i, off, num);
		for (int x = 0; x < n; x++)
			df[x] = i[x];
		return (n);
	}
	if (dr->type & xplmType_FloatArray) {
		float f[num];
		int n = XPLMGetDatavf(dr->dr, f, off, num);
		for (int x = 0; x < n; x++)
			df[x] = f[x];
		return (n);
	}
	if (dr->type & xplmType_Data) {
		uint8_t u[num];
		int n = XPLMGetDatab(dr->dr, u, off, num);
		for (int x = 0; x < n; x++)
			df[x] = u[x];
		return (n);
	}
	VERIFY_MSG(0, "dr %s has bad type %x", dr->name, dr->type);
}

void
dr_setvf(dr_t *dr, double *df, unsigned off, unsigned num)
{
	ASSERT_MSG(dr->writable, "%s", dr->name);
	if (dr->type & xplmType_IntArray) {
		int i[num];
		for (unsigned x = 0; x < num; x++)
			i[x] = df[x];
		XPLMSetDatavi(dr->dr, i, off, num);
	} else if (dr->type & xplmType_FloatArray) {
		float f[num];
		for (unsigned x = 0; x < num; x++)
			f[x] = df[x];
		XPLMSetDatavf(dr->dr, f, off, num);
	} else if (dr->type & xplmType_Data) {
		uint8_t u[num];
		for (unsigned x = 0; x < num; x++)
			u[x] = df[x];
		XPLMSetDatab(dr->dr, u, off, num);
	} else {
		VERIFY_MSG(0, "dr %s has bad type %x", dr->name, dr->type);
	}
}
