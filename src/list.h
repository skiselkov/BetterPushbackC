/*
 * CDDL HEADER START
 *
 * The contents of this file are subject to the terms of the
 * Common Development and Distribution License (the "License").
 * You may not use this file except in compliance with the License.
 *
 * You can obtain a copy of the license in the file COPYING
 * or http://www.opensolaris.org/os/licensing.
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
 * Copyright 2008 Sun Microsystems, Inc.  All rights reserved.
 * Use is subject to license terms.
 */

#ifndef	_BP_LIST_H
#define	_BP_LIST_H

#include <sys/types.h>

#include "config.h"

#ifdef	__cplusplus
extern "C" {
#endif

typedef struct list_node list_node_t;
typedef struct list list_t;

#define	list_create		SYMBOL_PREFIX(list_create)
void list_create(list_t *, size_t, size_t);
#define	list_destroy		SYMBOL_PREFIX(list_destroy)
void list_destroy(list_t *);

#define	list_insert_after	SYMBOL_PREFIX(list_insert_after)
void list_insert_after(list_t *, void *, void *);
#define	list_insert_before	SYMBOL_PREFIX(list_insert_before)
void list_insert_before(list_t *, void *, void *);
#define	list_insert_head	SYMBOL_PREFIX(list_insert_head)
void list_insert_head(list_t *, void *);
#define	list_insert_tail	SYMBOL_PREFIX(list_insert_tail)
void list_insert_tail(list_t *, void *);
#define	list_remove		SYMBOL_PREFIX(list_remove)
void list_remove(list_t *, void *);
#define	list_remove_head	SYMBOL_PREFIX(list_remove_head)
void *list_remove_head(list_t *);
#define	list_remove_tail	SYMBOL_PREFIX(list_remove_tail)
void *list_remove_tail(list_t *);
#define	list_move_tail		SYMBOL_PREFIX(list_move_tail)
void list_move_tail(list_t *, list_t *);

#define	list_head		SYMBOL_PREFIX(list_head)
void *list_head(const list_t *);
#define	list_tail		SYMBOL_PREFIX(list_tail)
void *list_tail(const list_t *);
#define	list_next		SYMBOL_PREFIX(list_next)
void *list_next(const list_t *, const void *);
#define	list_prev		SYMBOL_PREFIX(list_prev)
void *list_prev(const list_t *, const void *);
#define	list_is_empty		SYMBOL_PREFIX(list_is_empty)
int list_is_empty(const list_t *);

#define	list_link_init		SYMBOL_PREFIX(list_link_init)
void list_link_init(list_node_t *);
#define	list_link_replace	SYMBOL_PREFIX(list_link_replace)
void list_link_replace(list_node_t *, list_node_t *);

#define	list_link_active	SYMBOL_PREFIX(list_link_active)
int list_link_active(const list_node_t *);
#define	list_count		SYMBOL_PREFIX(list_count)
size_t list_count(const list_t *);

struct list_node {
	struct list_node *list_next;
	struct list_node *list_prev;
};

struct list {
	size_t	list_size;
	size_t	list_offset;
	size_t	list_count;
	struct list_node list_head;
};

#ifdef	__cplusplus
}
#endif

#endif	/* _BP_LIST_H */
