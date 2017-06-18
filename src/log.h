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

#ifndef _XTCAS_LOG_H_
#define _XTCAS_LOG_H_

#include <stdarg.h>
#include "helpers.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
	int	all;
	int	snd;
	int	wav;
	int	tcas;
	int	ra;
	int	cpa;
	int	test;
	int	xplane;
	int	sl;
	int	contact;
	int	threat;
} dbg_info_t;

extern dbg_info_t xtcas_dbg;

void xtcas_log_impl(const char *filename, int line,
    const char *fmt, ...) PRINTF_ATTR(3);
void xtcas_log_impl_v(const char *filename, int line, const char *fmt,
    va_list ap);
void xtcas_log_backtrace(void);

#define	logMsg(...) \
	xtcas_log_impl(NULL, 0, log_basename(__FILE__), __LINE__, __VA_ARGS__)

#if	defined(__GNUC__) || defined(__clang__)
#define	xtcas_basename(f) (__builtin_strrchr(f, BUILD_DIRSEP) ? \
	__builtin_strrchr(f, BUILD_DIRSEP) + 1 : f)
#else	/* !__GNUC__ && !__clang__ */
const char *xtcas_basename(const char *filename);
#endif	/* !__GNUC__ && !__clang__ */

#ifndef	TEST_STANDALONE_BUILD
#define	dbg_log(class, level, ...) \
	do { \
		if (xtcas_dbg.class >= level || xtcas_dbg.all >= level) { \
			xtcas_log_impl(xtcas_basename(__FILE__), __LINE__,  \
			    "[" #class "/" #level "]: " __VA_ARGS__); \
		} \
	} while (0)
#else	/* TEST_STANDALONE_BUILD */
#define	dbg_log(class, level, ...) \
	do { \
		if (xtcas_dbg.class >= level || xtcas_dbg.all >= level) { \
			xtcas_log_impl(NULL, 0, __VA_ARGS__); \
		} \
	} while (0)
#endif	/* TEST_STANDALONE_BUILD */

#ifdef __cplusplus
}
#endif

#endif /* _XTCAS_LOG_H_ */
