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

#ifndef	_CAB_VIEW_H_
#define	_CAB_VIEW_H_

#include <acfutils/types.h>

#ifdef	__cplusplus
extern "C" {
#endif

void cab_view_init(void);
void cab_view_fini(void);

bool_t cab_view_can_start(void);
bool_t cab_view_start(void);
void cab_view_stop(void);

#ifdef	__cplusplus
}
#endif

#endif	/* _CAB_VIEW_H_ */
