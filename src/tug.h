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

#ifndef	_TUG_H_
#define	_TUG_H_

#include <XPLMScenery.h>

#include <acfutils/avl.h>
#include <acfutils/dr.h>
#include <acfutils/geom.h>
#include <acfutils/wav.h>

#include "driving.h"

#ifdef	__cplusplus
extern "C" {
#endif

#define	PB_TUG_CONN_OFFSET	2

typedef struct {
	char	*tug;		/* main OBJ */

	double	max_steer;	/* max steering deflection, degrees */
	double	max_fwd_speed;	/* m/s */
	double	max_rev_speed;	/* m/s */
	double	max_accel;	/* m/s^2 */
	double	max_decel;	/* m/s^2 */

	/*
	 * Axle & wheel parameters. 'z' is the offset from the origin point
	 * along the vehicle's long axis in meters, forward being positive.
	 * 'radius' is the wheel radius in meters.
	 */
	double	front_z;	/* meters */
	double	front_radius;	/* meters */
	double	rear_z;		/* meters */
	double	rear_radius;	/* meters */
	double	lift_z;

	double	lift_height;	/* how high we lift the acf nose gear, meters */

	double	height;		/* origin point height above ground, meters */
	double	min_mtow;	/* min allowable acf MTOW, kg */
	double	max_mtow;	/* max allowable acf MTOW, kg */
	double	min_nlg_len;	/* min acf nose landing gear length, meters */
	char	*arpt;		/* airport ICAO */

	char	*engine_snd;	/* engine noise WAV */
	char	*air_snd;	/* air release sound WAV */
	char	*beeper_snd;	/* bepper sound WAV */

	int	sort_rand;	/* random sorting number */

	avl_node_t	node;
} tug_info_t;

typedef struct {
	vehicle_pos_t	pos;
	vehicle_t	veh;
	double		cur_steer;
	double		last_mis_hdg;

	XPLMObjectRef	tug, front, rear;
	double		front_phi, rear_phi;	/* tire roll angle, degrees */

	tug_info_t	*info;

	bool_t		engine_snd_playing;
	float		pitch;
	wav_t		*engine_snd;
	wav_t		*air_snd;
	wav_t		*beeper_snd;

	bool_t		cradle_air_on;
	bool_t		cradle_air_snd_on;
	double		cradle_air_chg_t;

	bool_t		cradle_beeper_snd_on;

	dr_t		cam_x;
	dr_t		cam_y;
	dr_t		cam_z;
	dr_t		cam_hdg;
	dr_t		cam_is_ext;

	dr_t		sound_on;
	dr_t		ext_vol;

	unsigned	num_cockpit_window_drs;
	dr_t		cockpit_window_drs[2];

	list_t		segs;
} tug_t;

void tug_glob_init(void);
void tug_glob_fini(void);

bool_t tug_create(tug_t *tug, double mtow, double ng_len, const char *arpt,
    vect2_t pos, double hdg);
void tug_destroy(tug_t *tug);

bool_t tug_drive2point(tug_t *tug, vect2_t dst, double hdg);
void tug_run(tug_t *tug, double d_t);
void tug_draw(tug_t *tug, double cur_t, double d_t);
void tug_set_TE_snd(tug_t *tug, double TE_fract);
void tug_set_cradle_air_on(tug_t *tug, bool_t flag, double cur_t);
void tug_set_cradle_beeper_on(tug_t *tug, bool_t flag);

bool_t tug_is_stopped(const tug_t *tug);

#ifdef	__cplusplus
}
#endif

#endif	/* _TUG_H_ */
