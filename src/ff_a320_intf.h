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

#ifndef	_FF_A320_INTF_H_
#define	_FF_A320_INTF_H_

#include <acfutils/types.h>

#ifdef __cplusplus
extern "C" {
#endif

bool_t ff_a320_intf_init(void);
void ff_a320_intf_fini(void);

#ifdef __cplusplus
}
#endif

#endif	/* _FF_A320_INTF_H_ */
