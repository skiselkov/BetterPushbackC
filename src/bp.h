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

#ifndef	_BP_H_
#define	_BP_H_

#include "types.h"

#ifdef	__cplusplus
extern "C" {
#endif

bool_t bp_init(void);
void bp_fini(void);
void bp_start(void);
void bp_stop(void);

void bp_cam_init(void);
void bp_cam_fini(void);

#ifdef	__cplusplus
}
#endif

#endif	/* _BP_H_ */
