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

#ifndef	_CFG_H_
#define	_CFG_H_

#include <acfutils/conf.h>

#ifdef	__cplusplus
extern "C" {
#endif

extern conf_t *bp_conf;

bool_t bp_conf_init();
bool_t bp_conf_save();
void bp_conf_fini();

void bp_conf_set_save_enabled(bool_t flag);

void bp_conf_open(void);

#ifdef	__cplusplus
}
#endif

#endif	/* _CFG_H_ */
