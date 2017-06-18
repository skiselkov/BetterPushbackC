/*
 * CDDL HEADER START
 *
 * The contents of this file are subject to the terms of the
 * Common Development and Distribution License, Version 1.0 only
 * (the "License").  You may not use this file except in compliance
 * with the License.
 *
 * You can obtain a copy of the license in the file COPYING
 * or http://www.opensource.org/licenses/CDDL-1.0.
 * See the License for the specific language governing permissions
 * and limitations under the License.
 *
 * When distributing Covered Code, include this CDDL HEADER in each
 * file and include the License file COPYING.
 * If applicable, add the following below this CDDL HEADER, with the
 * fields enclosed by brackets "[]" replaced with your own identifying
 * information: Portions Copyright [yyyy] [name of copyright owner]
 *
 * CDDL HEADER END
 */
/*
 * Copyright 2017 Saso Kiselkov. All rights reserved.
 */

#ifndef	_XRAAS_ASSERT_H_
#define	_XRAAS_ASSERT_H_

#include <assert.h>
#include <stdlib.h>

#include "log.h"
#include "helpers.h"

#ifdef	__cplusplus
extern "C" {
#endif

/*
 * ASSERT() and VERIFY() are assertion test macros. If the condition
 * expression provided as the argument to the macro evaluates as non-true,
 * the program prints a debug message specifying exactly where and what
 * condition was violated, a stack backtrace and a dumps core by
 * calling abort().
 *
 * The difference between ASSERT and VERIFY is that ASSERT compiles to
 * a no-op unless -DDEBUG is provided to the compiler. VERIFY always
 * checks its condition and dumps if it is non-true.
 */

#define	VERIFY_MSG(x, fmt, ...) \
	do { \
		if (!(x)) { \
			xtcas_log_impl(xtcas_basename(__FILE__), __LINE__, \
			    "assertion \"%s\" failed: " fmt, #x, __VA_ARGS__); \
			xtcas_log_backtrace(); \
			abort(); \
		} \
	} while (0)

#define	VERIFY(x)	VERIFY_MSG(x, "%s", "")

#define	VERIFY3_impl(x, op, y, type, fmt) \
	do { \
		type tmp_x = (type)(x); \
		type tmp_y = (type)(y); \
		if (!(tmp_x op tmp_y)) { \
			xtcas_log_impl(xtcas_basename(__FILE__), __LINE__, \
			    "assertion " #x " " #op " " #y " failed (" \
			    fmt " " #op " " fmt ")", tmp_x, tmp_y); \
			xtcas_log_backtrace(); \
			abort(); \
		} \
	} while (0)
#define	VERIFY3S(x, op, y)	VERIFY3_impl(x, op, y, long, "%lu")
#define	VERIFY3U(x, op, y)	VERIFY3_impl(x, op, y, unsigned long, "0x%lx")
#define	VERIFY3F(x, op, y)	VERIFY3_impl(x, op, y, double, "%f")
#define	VERIFY3P(x, op, y)	VERIFY3_impl(x, op, y, void *, "%p")

#ifdef	DEBUG
#define	ASSERT(x)		VERIFY(x)
#define	ASSERT3S(x, op, y)	VERIFY3S(x, op, y)
#define	ASSERT3U(x, op, y)	VERIFY3U(x, op, y)
#define	ASSERT3F(x, op, y)	VERIFY3F(x, op, y)
#define	ASSERT3P(x, op, y)	VERIFY3P(x, op, y)
#define	ASSERT_MSG(x, fmt, ...)	VERIFY_MSG(x, fmt, __VA_ARGS__)
#else	/* !DEBUG */
#define	ASSERT(x)		UNUSED(x)
#define	ASSERT3S(x, op, y)	UNUSED((x) op (y))
#define	ASSERT3U(x, op, y)	UNUSED((x) op (y))
#define	ASSERT3F(x, op, y)	UNUSED((x) op (y))
#define	ASSERT3P(x, op, y)	UNUSED((x) op (y))
#define	ASSERT_MSG(x, fmt, ...)
#endif	/* !DEBUG */

/*
 * Compile-time assertion. The condition 'x' must be constant.
 */
#define	CTASSERT(x)		_CTASSERT(x, __LINE__)
#define	_CTASSERT(x, y)		__CTASSERT(x, y)
#define	__CTASSERT(x, y)	\
	typedef char __compile_time_assertion__ ## y [(x) ? 1 : -1]

#ifdef	__cplusplus
}
#endif

#endif	/* _XRAAS_ASSERT_H_ */
