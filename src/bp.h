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

#include "config.h"
#include <acfutils/types.h>

#ifdef	__cplusplus
extern "C" {
#endif

#define bp_init		SYMBOL_PREFIX(bp_init)
bool_t bp_init(void);

#define	bp_fini		SYMBOL_PREFIX(bp_fini)
void bp_fini(void);

#define	bp_start	SYMBOL_PREFIX(bp_start)
bool_t bp_start(void);

#define	bp_can_start	SYMBOL_PREFIX(bp_can_start)
bool_t bp_can_start(char **reason);

#define	bp_stop		SYMBOL_PREFIX(bp_stop)
bool_t bp_stop(void);

#define	bp_cam_start	SYMBOL_PREFIX(bp_cam_start)
bool_t bp_cam_start(void);

#define	bp_cam_stop	SYMBOL_PREFIX(bp_cam_stop)
bool_t bp_cam_stop(void);

#define	bp_num_segs	SYMBOL_PREFIX(bp_num_segs)
unsigned bp_num_segs(void);

#ifdef	__cplusplus
}
#endif

#endif	/* _BP_H_ */
