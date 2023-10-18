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
 * Copyright 2023 Saso Kiselkov. All rights reserved.
 */

#ifndef	_BP_H_
#define	_BP_H_

#include <XPLMDisplay.h>

#include <acfutils/geom.h>
#include <acfutils/types.h>

#include "acf_outline.h"
#include "driving.h"
#include "tug.h"

#ifdef	__cplusplus
extern "C" {
#endif

typedef enum {
	PB_STEP_OFF,
	PB_STEP_TUG_LOAD,
	PB_STEP_START,
	PB_STEP_DRIVING_UP_CLOSE,
	PB_STEP_OPENING_CRADLE,
	PB_STEP_WAITING_FOR_PBRAKE,
	PB_STEP_DRIVING_UP_CONNECT,
	PB_STEP_GRABBING,
	PB_STEP_LIFTING,
	PB_STEP_CONNECTED,
	PB_STEP_STARTING,
	PB_STEP_PUSHING,
	PB_STEP_STOPPING,
	PB_STEP_STOPPED,
	PB_STEP_LOWERING,
	PB_STEP_UNGRABBING,
	PB_STEP_WAITING4OK2DISCO,
	PB_STEP_MOVING_AWAY,
	PB_STEP_CLOSING_CRADLE,
	PB_STEP_STARTING2CLEAR,
	PB_STEP_MOVING2CLEAR,
	PB_STEP_CLEAR_SIGNAL,
	PB_STEP_DRIVING_AWAY
} pushback_step_t;

typedef struct {
	int		nw_i;		/* nose gear index the gear tables */
	double		nw_z;		/* nose gear long offset (meters) */
	double		main_z;		/* main gear long offset (meters) */
	double		nw_len;		/* nose gear leg length (meters) */
	unsigned	nw_type;	/* nosewheel gear arrangement type */
	double		tirrad;		/* nosewheel tire radius (meters) */
	int		n_gear;		/* number of gear */
	int		gear_is[10];	/* gear index in the gear tables */

	/*
	 * The following flags are extracted from the aircraft's .acf file
	 * at boot.
	 */
	struct {
		bool_t	is_airliner;		/* acf/_is_airliner */
		bool_t	is_experimental;	/* acf/_is_experimental */
		bool_t	is_general_aviation;	/* acf/_is_general_aviation */
		bool_t	is_glider;		/* acf/_is_glider */
		bool_t	is_helicopter;		/* acf/_is_helicopter */
		bool_t	is_military;		/* acf/_is_military */
		bool_t	is_sci_fi;		/* acf/_is_sci_fi */
		bool_t	is_seaplane;		/* acf/_is_seaplane */
		bool_t	is_ultralight;		/* acf/_is_ultralight */
		bool_t	is_vtol;		/* acf/_is_vtol*/
		bool_t	fly_like_a_helo;	/* acf/_fly_like_a_helo */
	} model_flags;
} acf_t;


/*
 * This is the pushback state structure.
 * CAUTION: do not store anything in here that requires teardown in bp_fini
 * because bp_state_init() wipes this struct to all 0x00 each time without
 * checking any pre-existing state. Use bp_long_state_t for that.
 */
typedef struct {
	vehicle_t       veh;            /* our driving params */
	acf_t           acf;            /* aux params of aircraft gear */

	struct {
		vect2_t start_acf_pos;
		bool_t  pbrk_rele_called;
		bool_t  pbrk_set_called;
		bool_t  complete;
	} winching;

	vehicle_pos_t	cur_pos;
	vehicle_pos_t	last_pos;

	double		cur_t;          /* current time in seconds */
	double		last_t;         /* cur_t from previous run */
	double		last_mis_hdg;   /* previous steering misalignment */

	/* deltas from last_* to cur_* */
	vehicle_pos_t	d_pos;          /* delta from last_pos to cur_pos */
	double		d_t;            /* delta time from last_t to cur_t */

	double		last_steer;
	double		last_force;
	double		tug_weight_force;

	pushback_step_t	step;		/* current PB step */
	double		step_start_t;	/* PB step start time */
	double		last_voice_t;	/* last voice message start time */

	double		reverse_t;	/* when reversing direction */

	bool_t		ok2disco;	/* user has ok'd disconnection */
	bool_t		reconnect;	/* user has requested reconnection */

	vect2_t		start_pos;	/* where the pushback originated */
	double		start_hdg;	/* which way we were facing at start */

	list_t		segs;
	bool_t		last_seg_is_back;
	double		last_hdg;

	bool_t		light_warn;

	struct {
		float	nosewheel_rot_spd;	/* rad/sec */
	} anim;
} bp_state_t;

/*
 * Stores state information which needs teardown in bp_fini and persists for
 * a longer time than bp_state_t (which is wiped every time in bp_state_init).
 */
typedef struct {
	XPLMWindowID	disco_win;
	XPLMWindowID	recon_win;
	tug_t		*tug;
	acf_outline_t	*outline;	/* size & outline of aircraft shape */
} bp_long_state_t;

extern bp_state_t bp;
extern bp_long_state_t bp_ls;

void bp_boot_init(void);
void bp_shut_fini(void);

bool_t bp_init(void);
void bp_fini(void);

bool_t bp_start(void);
bool_t bp_stop(void);

bool_t bp_can_start(const char **reason);
unsigned bp_num_segs(void);
void bp_delete_all_segs(void);

bool_t acf_is_compatible(void);
bool_t acf_is_airliner(void);
void read_acf_airline(char airline[1024]);
bool_t find_nearest_airport(char icao[8]);

extern bool_t late_plan_requested;

#ifdef	__cplusplus
}
#endif

#endif	/* _BP_H_ */
