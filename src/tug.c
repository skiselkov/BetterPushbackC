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

#include <errno.h>
#include <string.h>
#include <stddef.h>

#include <XPLMPlugin.h>

#include <acfutils/avl.h>
#include <acfutils/assert.h>
#include <acfutils/helpers.h>
#include <acfutils/math.h>

#include "driving.h"
#include "tug.h"
#include "xplane.h"

#define	TUG_MAX_STEER		60
#define	TUG_STEER_RATE		60	/* deg/s */

#define	TUG_MAX_SPD		1	/* m/s */
#define	TUG_MAX_ANG_VEL	20	/* deg/s */
#define	TUG_MAX_ACCEL		1	/* m/s^2 */
#define	TUG_MAX_DECEL		0.5	/* m/s^2 */

#define	TUG_SND_MAX_DIST	15	/* meters */
#define	VOLUME_INSIDE_MODIFIER	0.2

#define	TUG_CRADLE_CHG_D_T	0.5
#define	TUG_CRADLE_AIR_MOD	0.5

static float front_drive_anim = 0, front_steer_anim = 0.5;
static float rear_drive_anim = 0;
static dr_t front_drive_anim_dr, front_steer_anim_dr;
static dr_t rear_drive_anim_dr;

static bool_t inited = B_FALSE;

struct tug_info_s {
	char	*tug;		/* main OBJ */
	char	*front_wheel;	/* front wheels OBJ */
	vect3_t	front_pos;	/* meters */
	double	front_radius;	/* meters */
	char	*rear_wheel;	/* rear wheels OBJ */
	vect3_t	rear_pos;	/* meters */
	double	rear_radius;	/* meters */
	double	height;		/* meters */
	double	min_mtow;	/* kg */
	double	max_mtow;	/* kg */
	double	min_ng_len;	/* meters */
	char	*arpt;		/* airport ICAO */
	char	*engine_snd;	/* engine noise WAV */
	char	*air_snd;	/* air release sound WAV */
	char	*beeper_snd;	/* bepper sound WAV */
	int	sort_rand;	/* random sorting number */

	avl_node_t	node;
};

/*
 * Tug info ordering function for avl_tree_t.
 */
int
tug_info_compar(const void *a, const void *b)
{
	const tug_info_t *ta = a, *tb = b;

	if (ta->sort_rand < tb->sort_rand) {
		return (-1);
	} else if (ta->sort_rand > tb->sort_rand) {
		return (1);
	} else {
		int n = strcmp(ta->tug, tb->tug);
		if (n < 0)
			return (-1);
		else if (n > 0)
			return (1);
		ASSERT0(memcmp(ta, tb, sizeof (*ta)));
		return (0);
	}
}

static void
tug_info_free(tug_info_t *ti)
{
	free(ti->tug);
	free(ti->front_wheel);
	free(ti->rear_wheel);
	free(ti->arpt);
	free(ti->engine_snd);
	free(ti->air_snd);
	free(ti->beeper_snd);
	free(ti);
}

static tug_info_t *
tug_info_read(const char *tugdir)
{
	tug_info_t *ti = NULL;
	char *cfgfilename = mkpathname(tugdir, "info.cfg", NULL);
	FILE *fp = fopen(cfgfilename, "r");
	char option[256], arg[256];

	if (fp == NULL) {
		logMsg("Malformed tug %s, cannot open info.cfg: %s",
		    tugdir, strerror(errno));
		free(cfgfilename);
		return (NULL);
	}

	ti = calloc(1, sizeof (*ti));
	ti->front_pos = NULL_VECT3;
	ti->rear_pos = NULL_VECT3;

	while (!feof(fp)) {
		if (fscanf(fp, "%255s", option) != 1)
			continue;
		if (option[0] == '#') {
			while (fgetc(fp) != '\n' && !feof(fp))
				;
			continue;
		}

#define	READ_FILENAME(optname, result) \
	do { \
		bool_t isdir; \
		if (fscanf(fp, "%255s", arg) != 1) { \
			logMsg("Malformed tug config file %s: expected " \
			    "string following '" optname "'", cfgfilename); \
			goto errout; \
		} \
		(result) = mkpathname(tugdir, arg, NULL); \
		if (!file_exists((result), &isdir) || isdir) { \
			logMsg("Malformed tug config file %s: '%s' not " \
			    "found or is not a file.", cfgfilename, \
			    (result)); \
			goto errout; \
		} \
	} while (0)
#define	READ_REAL(optname, result) \
	do { \
		if (fscanf(fp, "%lf", (result)) != 1) { \
			logMsg("Malformed tug config file %s: expected " \
			    "number following '" optname "'", cfgfilename); \
			goto errout; \
		} \
	} while (0)

		if (strcmp(option, "tug_obj") == 0) {
			READ_FILENAME("tug_obj", ti->tug);
		} else if (strcmp(option, "front_wheel_obj") == 0) {
			READ_FILENAME("front_wheel_obj", ti->front_wheel);
		} else if (strcmp(option, "front_x") == 0) {
			READ_REAL("front_x", &ti->front_pos.x);
		} else if (strcmp(option, "front_y") == 0) {
			READ_REAL("front_y", &ti->front_pos.y);
		} else if (strcmp(option, "front_z") == 0) {
			READ_REAL("front_z", &ti->front_pos.z);
		} else if (strcmp(option, "front_r") == 0) {
			READ_REAL("front_r", &ti->front_radius);
		} else if (strcmp(option, "rear_wheel_obj") == 0) {
			READ_FILENAME("rear_wheel_obj", ti->rear_wheel);
		} else if (strcmp(option, "rear_x") == 0) {
			READ_REAL("rear_x", &ti->rear_pos.x);
		} else if (strcmp(option, "rear_y") == 0) {
			READ_REAL("rear_y", &ti->rear_pos.y);
		} else if (strcmp(option, "rear_z") == 0) {
			READ_REAL("rear_z", &ti->rear_pos.z);
		} else if (strcmp(option, "rear_r") == 0) {
			READ_REAL("rear_r", &ti->rear_radius);
		} else if (strcmp(option, "height") == 0) {
			READ_REAL("height", &ti->height);
		} else if (strcmp(option, "min_mtow") == 0) {
			READ_REAL("min_mtow", &ti->min_mtow);
		} else if (strcmp(option, "max_mtow") == 0) {
			READ_REAL("max_mtow", &ti->max_mtow);
		} else if (strcmp(option, "min_ng_len") == 0) {
			READ_REAL("min_ng_len", &ti->min_ng_len);
		} else if (strcmp(option, "arpt") == 0) {
			READ_FILENAME("arpt", ti->arpt);
		} else if (strcmp(option, "engine_snd") == 0) {
			READ_FILENAME("engine_snd", ti->engine_snd);
		} else if (strcmp(option, "air_snd") == 0) {
			READ_FILENAME("air_snd", ti->air_snd);
		} else if (strcmp(option, "beeper_snd") == 0) {
			READ_FILENAME("beeper_snd", ti->beeper_snd);
		} else {
			logMsg("Malformed tug config file %s: unknown "
			    "option '%s'", cfgfilename, option);
			goto errout;
		}
#undef	READ_REAL
#undef	READ_FILENAME
	}

#define	VALIDATE_TUG(cond, optname) \
	do { \
		if (cond) { \
			logMsg("Malformed tug config file %s: missing " \
			    "or bad value for keyword '%s'", cfgfilename, \
			    (optname)); \
			goto errout; \
		} \
	} while (0)
#define	VALIDATE_TUG_STR(field, optname) \
	VALIDATE_TUG((field) == NULL, (optname))
#define	VALIDATE_TUG_REAL(field, optname) \
	VALIDATE_TUG((field) <= 0.0, (optname))
#define	VALIDATE_TUG_REAL_NAN(field, optname) \
	VALIDATE_TUG(isnan(field), (optname))

	VALIDATE_TUG_STR(ti->tug, "tug_obj");
	VALIDATE_TUG_STR(ti->front_wheel, "front_wheel_obj");
	VALIDATE_TUG_REAL_NAN(ti->front_pos.x, "front_x");
	VALIDATE_TUG_REAL_NAN(ti->front_pos.y, "front_y");
	VALIDATE_TUG_REAL_NAN(ti->front_pos.z, "front_z");
	VALIDATE_TUG_REAL(ti->front_radius, "front_r");
	VALIDATE_TUG_STR(ti->rear_wheel, "rear_wheel_obj");
	VALIDATE_TUG_REAL_NAN(ti->rear_pos.x, "rear_x");
	VALIDATE_TUG_REAL_NAN(ti->rear_pos.y, "rear_y");
	VALIDATE_TUG_REAL_NAN(ti->rear_pos.z, "rear_z");
	VALIDATE_TUG_REAL(ti->rear_radius, "rear_r");
	VALIDATE_TUG_REAL(ti->max_mtow, "max_mtow");
	if (ti->min_mtow >= ti->max_mtow) {
		logMsg("Malformed tug config file %s: min_mtow >= max_mtow",
		    cfgfilename);
		goto errout;
	}
	VALIDATE_TUG_REAL(ti->min_ng_len, "min_ng_len");
	VALIDATE_TUG_STR(ti->engine_snd, "engine_snd");

#undef	VALIDATE_TUG_STR
#undef	VALIDATE_TUG_REAL
#undef	VALIDATE_TUG_REAL_NAN
#undef	VALIDATE_TUG

	fclose(fp);
	free(cfgfilename);
	return (ti);

errout:
	if (ti != NULL)
		tug_info_free(ti);
	fclose(fp);
	free(cfgfilename);
	return (NULL);
}

static tug_info_t *
tug_info_select(double mtow, double ng_len, const char *arpt)
{
	char *tugdir;
	DIR *dirp;
	struct dirent *de;
	avl_tree_t tis;
	tug_info_t *ti, *ti_oth;
	void *cookie = NULL;

	tugdir = mkpathname(bp_xpdir, bp_plugindir, "objects", "tugs", NULL);
	dirp = opendir(tugdir);
	if (dirp == NULL) {
		free(tugdir);
		return (NULL);
	}

	avl_create(&tis, tug_info_compar, sizeof (tug_info_t),
	    offsetof(tug_info_t, node));

	while ((de = readdir(dirp)) != NULL) {
		const char *dot;
		char *tugpath;
		tug_info_t *ti;

		if ((dot = strrchr(de->d_name, '.')) == NULL ||
		    strcmp(&dot[1], "tug") != 0)
			continue;

		tugpath = mkpathname(tugdir, de->d_name, NULL);
		ti = tug_info_read(tugpath);
		/*
		 * A tug matches our selection criteria iff:
		 * 1) it could be read AND
		 * 2) its min_mtow is <= our MTOW
		 * 3) its max_mtow is >= our MTOW
		 * 4) its min nosegear length is <= our nosegear length
		 * 5) if the caller provided an airport identifier and the
		 *	tug is airport-specific, then the airport ID matches
		 */
		if (ti != NULL) {
			logMsg("max_mtow: %.0f min_mtow: %.0f min_ng: %.2f "
			    "mtow: %.0f ng: %.2f", ti->max_mtow, ti->min_mtow,
			    ti->min_ng_len, mtow, ng_len);
		}
		if (ti != NULL && ti->min_mtow <= mtow &&
		    mtow <= ti->max_mtow && ti->min_ng_len <= ng_len &&
		    (arpt == NULL || ti->arpt == NULL ||
		    strcmp(arpt, ti->arpt) == 0)) {
			avl_add(&tis, ti);
		}
		free(tugpath);
	}
	free(tugdir);

	ti = avl_first(&tis);
	if (ti != NULL)
		avl_remove(&tis, ti);

	while ((ti_oth = avl_destroy_nodes(&tis, &cookie)) != NULL)
		tug_info_free(ti_oth);
	avl_destroy(&tis);

	return (ti);
}

void
tug_glob_init(void)
{
	VERIFY(!inited);
	dr_create_f(&front_drive_anim_dr, &front_drive_anim, B_FALSE,
	    "bp/anim/front_drive");
	dr_create_f(&front_steer_anim_dr, &front_steer_anim, B_FALSE,
	    "bp/anim/front_steer");
	dr_create_f(&rear_drive_anim_dr, &rear_drive_anim, B_FALSE,
	    "bp/anim/rear_drive");
	inited = B_TRUE;
}

void
tug_glob_fini(void)
{
	if (!inited)
		return;
	dr_delete(&front_drive_anim_dr);
	dr_delete(&front_steer_anim_dr);
	dr_delete(&rear_drive_anim_dr);
	inited = B_FALSE;
}

bool_t
tug_create(tug_t *tug, double mtow, double ng_len, const char *arpt,
    vect2_t pos, double hdg)
{
	VERIFY(inited);

	memset(tug, 0, sizeof (*tug));

	list_create(&tug->segs, sizeof (seg_t), offsetof(seg_t, node));

	/* Select a matching tug from our repertoire */
	tug->info = tug_info_select(mtow, ng_len, arpt);
	if (tug->info == NULL)
		goto errout;

	tug->info = tug->info;

	tug->pos.pos = pos;
	tug->pos.hdg = hdg;

	tug->veh.wheelbase = fabs(tug->info->front_pos.z -
	    tug->info->rear_pos.z);
	tug->veh.fixed_z_off = tug->info->rear_pos.z;
	tug->veh.max_steer = TUG_MAX_STEER;
	tug->veh.max_fwd_spd = TUG_MAX_SPD;
	tug->veh.max_rev_spd = TUG_MAX_SPD;
	tug->veh.max_ang_vel = TUG_MAX_ANG_VEL;
	tug->veh.max_accel = TUG_MAX_ACCEL;
	tug->veh.max_decel = TUG_MAX_DECEL;

	tug->tug = XPLMLoadObject(tug->info->tug);
	if (tug->tug == NULL) {
		logMsg("Error loading tug object %s", tug->info->tug);
		goto errout;
	}

/*	tug->front = XPLMLoadObject(tug->info->front_wheel);
	if (tug->front == NULL) {
		logMsg("Error loading tug object %s", tug->info->front_wheel);
		goto errout;
	}
	tug->rear = XPLMLoadObject(tug->info->rear_wheel);
	if (tug->rear == NULL) {
		logMsg("Error loading tug object %s", tug->info->rear_wheel);
		goto errout;
	}*/

	tug->engine_snd = wav_load(tug->info->engine_snd, "tug_engine");
	if (tug->engine_snd == NULL) {
		logMsg("Error loading tug sound %s", tug->info->engine_snd);
		goto errout;
	}
	wav_set_loop(tug->engine_snd, B_TRUE);

	if (tug->info->engine_snd != NULL) {
		tug->air_snd = wav_load(tug->info->engine_snd, "tug_air");
		if (tug->air_snd == NULL) {
			logMsg("Error loading tug sound %s",
			    tug->info->engine_snd);
			goto errout;
		}
		wav_set_loop(tug->air_snd, B_TRUE);
	}
	if (tug->info->beeper_snd != NULL) {
		tug->beeper_snd = wav_load(tug->info->beeper_snd, "tug_beeper");
		if (tug->beeper_snd == NULL) {
			logMsg("Error loading tug sound %s",
			    tug->info->beeper_snd);
			goto errout;
		}
		wav_set_loop(tug->beeper_snd, B_TRUE);
	}

	fdr_find(&tug->cam_x, "sim/graphics/view/view_x");
	fdr_find(&tug->cam_y, "sim/graphics/view/view_y");
	fdr_find(&tug->cam_z, "sim/graphics/view/view_z");
	fdr_find(&tug->cam_hdg, "sim/graphics/view/view_heading");
	fdr_find(&tug->cam_is_ext, "sim/graphics/view/view_is_external");
	fdr_find(&tug->sound_on, "sim/operation/sound/sound_on");
	fdr_find(&tug->ext_vol, "sim/operation/sound/exterior_volume_ratio");

	/* Flight Factor A320 integration */
	if (dr_find(&tug->cockpit_window_drs[0], "model/controls/window_l")) {
		fdr_find(&tug->cockpit_window_drs[1],
		    "model/controls/window_r");
		tug->num_cockpit_window_drs = 2;
	/* IXEG 737 integration */
	} else if (dr_find(&tug->cockpit_window_drs[0],
	    "ixeg/733/misc/pilot_window")) {
		fdr_find(&tug->cockpit_window_drs[1],
		    "ixeg/733/misc/copilot_window");
		tug->num_cockpit_window_drs = 2;
	}

	return (B_TRUE);

errout:
	tug_destroy(tug);
	return (B_FALSE);
}

void
tug_destroy(tug_t *tug)
{
	seg_t *seg;

	while ((seg = list_head(&tug->segs)) != NULL) {
		list_remove_head(&tug->segs);
		free(seg);
	}
	list_destroy(&tug->segs);

	if (tug->info != NULL)
		tug_info_free(tug->info);

	if (tug->tug != NULL)
		XPLMUnloadObject(tug->tug);
	if (tug->front != NULL)
		XPLMUnloadObject(tug->front);
	if (tug->rear != NULL)
		XPLMUnloadObject(tug->rear);

	if (tug->engine_snd != NULL) {
		wav_stop(tug->engine_snd);
		wav_free(tug->engine_snd);
	}
	if (tug->air_snd != NULL) {
		wav_stop(tug->air_snd);
		wav_free(tug->air_snd);
	}
	if (tug->beeper_snd != NULL) {
		wav_stop(tug->beeper_snd);
		wav_free(tug->beeper_snd);
	}

	memset(tug, 0, sizeof (*tug));
}

bool_t
tug_drive2point(tug_t *tug, vect2_t dst, double hdg)
{
	vect2_t cur_pos;
	double cur_hdg;
	seg_t *seg;

	seg = list_tail(&tug->segs);
	if (seg != NULL) {
		cur_pos = seg->end_pos;
		cur_hdg = seg->end_hdg;
	} else {
		cur_pos = tug->pos.pos;
		cur_hdg = tug->pos.hdg;
	}

	return (compute_segs(&tug->veh, cur_pos, cur_hdg, dst, hdg,
	    &tug->segs) >= 0);
}

void
tug_run(tug_t *tug, double d_t)
{
	double steer = 0, speed = 0;
	double accel, turn, radius;

	if (!tug->engine_snd_playing) {
		wav_play(tug->engine_snd);
		tug->engine_snd_playing = B_TRUE;
	}

	if (list_head(&tug->segs) != NULL) {
		(void) drive_segs(&tug->pos, &tug->veh, &tug->segs,
		    &tug->last_mis_hdg, d_t, &steer, &speed);
	} else if (tug->pos.spd == 0) {
		return;
	}

	if (speed >= tug->pos.spd)
		accel = MIN(speed - tug->pos.spd, TUG_MAX_ACCEL * d_t);
	else
		accel = MAX(speed - tug->pos.spd, -TUG_MAX_ACCEL * d_t);

	if (steer >= tug->cur_steer)
		turn = MIN(steer - tug->cur_steer, TUG_STEER_RATE * d_t);
	else
		turn = MAX(steer - tug->cur_steer, -TUG_STEER_RATE * d_t);

	tug->pos.spd += accel;
	tug->cur_steer += turn;

	radius = tan(DEG2RAD(90 - tug->cur_steer)) * tug->veh.wheelbase;
	if (radius > -1e3 && radius < 1e3) {
		double d_hdg = RAD2DEG((tug->pos.spd / radius) * d_t);
		vect2_t p2c = VECT2(radius, tug->info->rear_pos.z);
		vect2_t c2np = vect2_rot(vect2_neg(p2c), d_hdg);
		vect2_t d_pos = vect2_rot(vect2_add(p2c, c2np), tug->pos.hdg);
		tug->pos.pos = vect2_add(tug->pos.pos, d_pos);
		tug->pos.hdg = normalize_hdg(tug->pos.hdg + d_hdg);
	} else {
		vect2_t dir = hdg2dir(tug->pos.hdg);
		tug->pos.pos = vect2_add(tug->pos.pos, vect2_scmul(dir,
		    tug->pos.spd * d_t));
	}

	tug_set_TE_snd(tug, (ABS(tug->pos.spd) / TUG_MAX_SPD) / 2);

	front_steer_anim = (tug->cur_steer / (2 * tug->veh.max_steer)) + 0.5;
}

void
tug_draw(tug_t *tug, double cur_t, double d_t)
{
	XPLMDrawInfo_t di;
	XPLMProbeRef probe = XPLMCreateProbe(xplm_ProbeY);
	XPLMProbeInfo_t info = { .structSize = sizeof (XPLMProbeInfo_t) };
	vect3_t pos, norm;
	vect2_t v;
	double gain;

	/* X-Plane's Z axis is inverted to ours */
	VERIFY3U(XPLMProbeTerrainXYZ(probe, tug->pos.pos.x, 0,
	    -tug->pos.pos.y, &info), ==, xplm_ProbeHitTerrain);

	pos = VECT3(tug->pos.pos.x, info.locationY, -tug->pos.pos.y);
	norm = VECT3(info.normalX, info.normalY, info.normalZ);
	pos = vect3_add(pos, vect3_set_abs(norm, tug->info->height));

	v = VECT2(norm.x, -norm.z);
	v = vect2_rot(v, tug->pos.hdg);

	di.structSize = sizeof (di);
	di.x = pos.x;
	di.y = pos.y;
	di.z = pos.z;
	di.heading = tug->pos.hdg;
	di.roll = -RAD2DEG(asin(v.x / norm.y));
	di.pitch = -RAD2DEG(asin(v.y / norm.y));

	XPLMDrawObjects(tug->tug, 1, &di, 1, 1);

	front_drive_anim += (tug->pos.spd * d_t) /
	    (2 * M_PI * tug->info->front_radius);
	while (front_drive_anim >= 1.0)
		front_drive_anim -= 1.0;
	while (front_drive_anim < 0.0)
		front_drive_anim += 1.0;

	rear_drive_anim += (tug->pos.spd * d_t) /
	    (2 * M_PI * tug->info->rear_radius);
	while (rear_drive_anim >= 1.0)
		rear_drive_anim -= 1.0;
	while (rear_drive_anim < 0.0)
		rear_drive_anim += 1.0;

	if (dr_geti(&tug->sound_on) == 1) {
		vect3_t cam_pos_xp = VECT3(dr_getf(&tug->cam_x),
		    dr_getf(&tug->cam_y), dr_getf(&tug->cam_z));
		double cam_dist = vect3_abs(vect3_sub(cam_pos_xp, pos));
		double window = 0;

		/* Camera distance modifier */
		cam_dist = MAX(cam_dist, 1);
		gain = POW2(TUG_SND_MAX_DIST / cam_dist);
		gain = MIN(gain, 1.0);
		gain *= dr_getf(&tug->ext_vol);

		/* Cockpit-window-open modifier */
		for (unsigned i = 0; i < tug->num_cockpit_window_drs; i++) {
			window = MAX(dr_getf(&tug->cockpit_window_drs[i]),
			    window);
		}
		/* We're only interested in the first 1/4 of the window range */
		window = MIN(1, window * 4);
		if (dr_geti(&tug->cam_is_ext) == 0)
			gain *= wavg(VOLUME_INSIDE_MODIFIER, 1, window);
		if (gain < 0.001)
			gain = 0;
	} else {
		gain = 0;
	}

	wav_set_gain(tug->engine_snd, gain);

	wav_set_gain(tug->beeper_snd, gain);

	if (tug->cradle_air_on) {
		double ramp = MIN((cur_t - tug->cradle_air_chg_t) /
		    TUG_CRADLE_CHG_D_T, 1);
		wav_set_gain(tug->air_snd, gain * ramp *
		    TUG_CRADLE_AIR_MOD);
		if (!tug->cradle_air_snd_on) {
			wav_play(tug->air_snd);
			tug->cradle_air_snd_on = B_TRUE;
		}
	} else if (cur_t - tug->cradle_air_chg_t <= TUG_CRADLE_CHG_D_T) {
		double ramp = 1 - ((cur_t - tug->cradle_air_chg_t) /
		    TUG_CRADLE_CHG_D_T);
		wav_set_gain(tug->air_snd, gain * ramp *
		    TUG_CRADLE_AIR_MOD);
	} else if (tug->cradle_air_snd_on) {
		wav_stop(tug->air_snd);
		tug->cradle_air_snd_on = B_FALSE;
	}
}

void
tug_set_TE_snd(tug_t *tug, double TE_fract)
{
	/*
	 * We do a double log of the linear TE fraction to obtain a more
	 * obvious engine note of the engine being under load.
	 */
	for (int i = 0; i < 2; i++)
		TE_fract = log((TE_fract * (M_E - 1)) + 1);
	wav_set_pitch(tug->engine_snd, 0.5 + TE_fract);
}

void
tug_set_cradle_air_on(tug_t *tug, bool_t flag, double cur_t)
{
	if (tug->cradle_air_on == flag)
		return;
	tug->cradle_air_on = flag;
	tug->cradle_air_chg_t = cur_t;
}

void
tug_set_cradle_beeper_on(tug_t *tug, bool_t flag)
{
	if (flag && !tug->cradle_beeper_snd_on)
		wav_play(tug->beeper_snd);
	else if (!flag && tug->cradle_beeper_snd_on)
		wav_stop(tug->beeper_snd);
	tug->cradle_beeper_snd_on = flag;
}

bool_t
tug_is_stopped(const tug_t *tug)
{
	return (list_head(&tug->segs) == NULL && tug->pos.spd == 0);
}
