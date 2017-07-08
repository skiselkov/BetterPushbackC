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
	double	lift_wall_z;	/* lift forward fixed long offset, meters */
	double	max_tirrad;	/* max acf tire radius, meters */
	double	max_tirrad_f;	/* lift_arm_anim value for max_tirrad */
	double	min_tirrad;	/* min acf tire rad, meters (lift_arm_anim=0) */

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

	bool_t	anim_debug;	/* animation debugging active */
	bool_t	drive_debug;	/* driving debugging active */

	avl_node_t	node;
} tug_info_t;

typedef struct {
	vehicle_pos_t	pos;
	vehicle_t	veh, veh_slow;
	double		cur_steer;
	double		last_mis_hdg;

	XPLMObjectRef	tug;
	double		front_phi, rear_phi;	/* tire roll angle, degrees */

	tug_info_t	*info;

	double		tirrad;			/* acf tire radius, meters */

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

tug_t *tug_alloc(double mtow, double ng_len, double tirrad, const char *arpt);
void tug_free(tug_t *tug);

void tug_set_pos(tug_t *tug, vect2_t pos, double hdg, double spd);
bool_t tug_drive2point(tug_t *tug, vect2_t dst, double hdg);
void tug_run(tug_t *tug, double d_t, bool_t drive_slow);
void tug_draw(tug_t *tug, double cur_t);
void tug_set_TE_snd(tug_t *tug, double TE_fract);
void tug_set_cradle_air_on(tug_t *tug, bool_t flag, double cur_t);
void tug_set_cradle_beeper_on(tug_t *tug, bool_t flag);
void tug_set_steering(tug_t *tug, double steer, double d_t);

bool_t tug_is_stopped(const tug_t *tug);

void tug_set_lift_pos(float x);
void tug_set_lift_arm_pos(const tug_t *tug, float x, bool_t grabbing_tire);
void tug_set_tire_sense_pos(const tug_t *tug, float x);
void tug_set_cradle_lights_on(bool_t flag);
void tug_set_hazard_lights_on(bool_t flag);

#ifdef	__cplusplus
}
#endif

#endif	/* _TUG_H_ */
