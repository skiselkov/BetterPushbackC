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

#include <XPLMInstance.h>
#include <XPLMScenery.h>

#include <acfutils/avl.h>
#include <acfutils/dr.h>
#include <acfutils/geom.h>
#include <acfutils/wav.h>

#include "driving.h"

#ifdef	__cplusplus
extern "C" {
#endif

#define	TUG_WHEELBASE(tug)	((tug)->info->rear_z - (tug)->info->front_z)

/* Defines how the tug lifts the aircraft up. */
typedef enum {
	/* grab the nose wheel and lift it up using arms - the default */
	LIFT_GRAB,
	/* winch nose gear onto a platform and then lift the platform */
	LIFT_WINCH
} lift_t;

typedef enum {
	LIFT_WALL_FRONT,
	LIFT_WALL_CENTER,
	LIFT_WALL_BACK
} lift_wall_loc_t;

typedef struct {
	char	*tug_name;	/* name of tug directory under objects/tugs */
	char	*tugdir;	/* pointer to tug directory */

	lift_t	lift_type;
	bool_t	electric_drive;

	char	*tug;		/* main OBJ */

	double	mass;		/* tug mass, kg */
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
	lift_wall_loc_t	lift_wall_loc;
	double	max_tirrad;	/* max acf tire radius, meters */
	double	max_tirrad_f;	/* lift_arm_anim value for max_tirrad */
	double	min_tirrad;	/* min acf tire rad, meters (lift_arm_anim=0) */

	double	lift_height;	/* how high we lift the acf nose gear, meters */

	double	height;		/* origin point height above ground, meters */
	double	min_mtow;	/* min allowable acf MTOW, kg */
	double	max_mtow;	/* max allowable acf MTOW, kg */
	double	min_nlg_len;	/* min acf nose landing gear length, meters */
	double	apch_dist;	/* how close we apch to open cradle, meters */

	double	max_TE;		/* maximum tractive effort, Newtons */

	unsigned	num_fwd_gears;
	unsigned	num_rev_gears;
	unsigned	gear_compat;

	char	*arpt;		/* airport ICAO */

	char	*engine_snd;	/* engine noise WAV */
	char	*air_snd;	/* air release sound WAV */
	char	*beeper_snd;	/* bepper sound WAV */

	char	*engine_snd_in;	/* engine noise WAV (inside) */
	char	*air_snd_in;	/* air release sound WAV (inside) */
	char	*beeper_snd_in;	/* bepper sound WAV (inside) */

	int	sort_rand;	/* random sorting number */

	/*
	 * Information related to winch-style tugs.
	 * 1) plat_z - Long axis of the end of the lift platform (meters).
	 * 2) plat_h - Height of the base of the platform above ground when
	 *	the platform is fully lowered (meters). This information is
	 *	used to animate the nose gear being lifted up as it is being
	 *	winched onto the platform.
	 */
	double	plat_z, plat_h;

	bool_t	anim_debug;	/* animation debugging active */
	bool_t	drive_debug;	/* driving debugging active */
	bool_t	quick_debug;	/* quick test debugging active */

	/*
	 * In order to support one tug having a swappable texture based on
	 * the airport, before we load the tug, we copy its obj into a
	 * temporary location and prepend the TEXTURE, TEXTURE_LIT and
	 * TEXTURE_NORMAL values with `liveries/<livname>'.
	 */
	char	*livname;

	vect3_t	cam_pos;
	double	cab_lift_h;

	avl_node_t	node;
} tug_info_t;

typedef struct {
	vehicle_pos_t	pos;
	vehicle_t	veh, veh_slow, veh_super_slow;
	bool_t		steer_override;
	double		cur_steer;
	double		last_mis_hdg;

	XPLMObjectRef	tug;
	XPLMInstanceRef	instance;
	double		front_phi, rear_phi;	/* tire roll angle, degrees */

	/* async loading info */
	char		*objpath;
	bool_t		load_in_prog;
	bool_t		destroyed;

	tug_info_t	*info;

	double		tirrad;			/* acf tire radius, meters */

	bool_t		engine_snd_playing;

	wav_t		*engine_snd;
	wav_t		*air_snd;
	wav_t		*beeper_snd;

	wav_t		*engine_snd_in;
	wav_t		*air_snd_in;
	wav_t		*beeper_snd_in;

	bool_t		cradle_air_on;
	bool_t		cradle_air_snd_on;
	double		cradle_air_chg_t;
	double		last_TE_fract;
	bool_t		TE_override;

	bool_t		cradle_beeper_snd_on;

	dr_t		cam_x;
	dr_t		cam_y;
	dr_t		cam_z;
	dr_t		cam_hdg;
	dr_t		cam_is_ext;

	dr_t		sound_on;
	dr_t		ext_vol, int_vol;

	unsigned	num_cockpit_window_drs;
	dr_t		cockpit_window_drs[2];

	list_t		segs;
} tug_t;

bool_t tug_glob_init(void);
void tug_glob_fini(void);

bool_t tug_available(double mtow, double ng_len, double tirrad,
    unsigned gear_type, const char *arpt, const char *airline);
tug_t *tug_alloc_man(const char *tug_name, double tirrad, const char *arpt,
    const char *airline);
tug_t *tug_alloc_auto(double mtow, double ng_len, double tirrad,
    unsigned gear_type, const char *arpt, const char *airline);
void tug_free(tug_t *tug);

void tug_set_pos(tug_t *tug, vect2_t pos, double hdg, double spd);
bool_t tug_drive2point(tug_t *tug, vect2_t dst, double hdg);
void tug_run(tug_t *tug, double d_t, bool_t drive_slow);
void tug_anim(tug_t *tug, double d_t, double cur_t);
void tug_draw(tug_t *tug, double cur_t);
void tug_set_TE_override(tug_t *tug, bool_t override);
void tug_set_TE_snd(tug_t *tug, double TE_fract, double d_t);
void tug_set_cradle_air_on(tug_t *tug, bool_t flag, double cur_t);
void tug_set_cradle_beeper_on(tug_t *tug, bool_t flag);
void tug_set_steering(tug_t *tug, double steer, double d_t);

bool_t tug_is_stopped(const tug_t *tug);
double tug_plat_h(const tug_t *tug);
double tug_lift_wall_off(const tug_t *tug);

void tug_set_lift_pos(float x);
void tug_set_lift_arm_pos(const tug_t *tug, float x, bool_t grabbing_tire);
void tug_set_tire_sense_pos(const tug_t *tug, float x);
void tug_set_cradle_lights_on(bool_t flag);
void tug_set_hazard_lights_on(bool_t flag);
void tug_set_winch_on(tug_t *tug, bool_t flag);
void tug_set_clear_signal(bool_t on, bool_t right);
void tug_set_lift_in_transit(bool_t flag);

#ifdef	__cplusplus
}
#endif

#endif	/* _TUG_H_ */
