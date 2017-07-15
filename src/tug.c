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

#include <XPLMUtilities.h>
#include <XPLMPlugin.h>

#include <acfutils/assert.h>
#include <acfutils/helpers.h>
#include <acfutils/math.h>
#include <acfutils/time.h>

#include "driving.h"
#include "tug.h"
#include "xplane.h"

#define	TUG_STEER_RATE		20	/* deg/s */

#define	TUG_MAX_ANG_VEL		20	/* deg/s */

#define	TUG_MAX_FWD_SPD		6	/* m/s */
#define	TUG_MAX_REV_SPD		3	/* m/s */
#define	TUG_MAX_ACCEL		1	/* m/s^2 */
#define	TUG_MAX_DECEL		0.5	/* m/s^2 */

#define	TUG_MIN_WHEELBASE	5	/* meters */

#define	TUG_SND_MAX_DIST	15	/* meters */
#define	VOLUME_INSIDE_MODIFIER	0.2

#define	TUG_CRADLE_CHG_D_T	0.5
#define	TUG_CRADLE_AIR_MOD	0.5
#define	TUG_HAZARD_REV_PER_SEC	2	/* revolutions per second of beacon */
#define	LIGHTS_ON_SUN_ANGLE	5	/* degrees, sets vehicle lights on */

#define	DRIVER_TURN_TIME	0.75	/* seconds for driver to turn around */

typedef enum {
	ANIM_FRONT_DRIVE,
	ANIM_FRONT_STEER,
	ANIM_REAR_DRIVE,
	ANIM_LIFT,
	ANIM_LIFT_ARM,
	ANIM_TIRE_SENSE,
	ANIM_VEHICLE_LIGHTS,
	ANIM_CRADLE_LIGHTS,
	ANIM_REVERSE_LIGHTS,
	ANIM_HAZARD_LIGHTS,
	ANIM_DRIVER_ORIENTATION,
	TUG_NUM_ANIMS
} ANIM_t;

typedef struct {
	const char	*name;
	dr_t		dr;
	float		value;
} ANIM_info_t;

static ANIM_info_t anim[TUG_NUM_ANIMS] = {
    { .name = "bp/anim/front_drive" },
    { .name = "bp/anim/front_steer" },
    { .name = "bp/anim/rear_drive" },
    { .name = "bp/anim/lift" },
    { .name = "bp/anim/lift_arm" },
    { .name = "bp/anim/tire_sense" },
    { .name = "bp/anim/vehicle_lights" },
    { .name = "bp/anim/cradle_lights" },
    { .name = "bp/anim/reverse_lights" },
    { .name = "bp/anim/hazard_lights" },
    { .name = "bp/anim/driver_orientation" }
};

static bool_t cradle_lights_req = B_FALSE;
static dr_t sun_pitch_dr;
static bool_t inited = B_FALSE;
static tug_t *glob_tug = NULL;
static XPLMCommandRef tug_reload_cmd;

static inline float
anim_gate(float x)
{
	while (x > 1.0)
		x -= 1.0;
	while (x < 0.0)
		x += 1.0;
	return (x);
}

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
	ti->front_z = NAN;
	ti->rear_z = NAN;
	ti->lift_wall_z = NAN;

	/* set some defaults */
	ti->max_fwd_speed = TUG_MAX_FWD_SPD;
	ti->max_rev_speed = TUG_MAX_REV_SPD;
	ti->max_accel = TUG_MAX_ACCEL;
	ti->max_decel = TUG_MAX_DECEL;

	while (!feof(fp)) {
		if (fscanf(fp, "%255s", option) != 1)
			continue;
		if (option[0] == '#') {
			while (fgetc(fp) != '\n' && !feof(fp))
				;
			continue;
		}

#if	IBM
#define	FIX_PATHSEP(x)	fix_pathsep(x)
#else	/* !IBM */
#define	FIX_PATHSEP(x)
#endif	/* !IBM */

#define	READ_FILENAME(optname, result) \
	do { \
		bool_t isdir; \
		if (fscanf(fp, "%255s", arg) != 1) { \
			logMsg("Malformed tug config file %s: expected " \
			    "string following '" optname "'", cfgfilename); \
			goto errout; \
		} \
		FIX_PATHSEP(arg); \
		unescape_percent(arg); \
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
		} else if (strcmp(option, "max_steer") == 0) {
			READ_REAL("max_steer", &ti->max_steer);
		} else if (strcmp(option, "max_fwd_speed") == 0) {
			READ_REAL("max_fwd_speed", &ti->max_fwd_speed);
		} else if (strcmp(option, "max_rev_speed") == 0) {
			READ_REAL("max_rev_speed", &ti->max_rev_speed);
		} else if (strcmp(option, "max_accel") == 0) {
			READ_REAL("max_accel", &ti->max_accel);
		} else if (strcmp(option, "max_decel") == 0) {
			READ_REAL("max_decel", &ti->max_decel);
		} else if (strcmp(option, "front_z") == 0) {
			READ_REAL("front_z", &ti->front_z);
		} else if (strcmp(option, "front_r") == 0) {
			READ_REAL("front_r", &ti->front_radius);
		} else if (strcmp(option, "rear_z") == 0) {
			READ_REAL("rear_z", &ti->rear_z);
		} else if (strcmp(option, "rear_r") == 0) {
			READ_REAL("rear_r", &ti->rear_radius);
		} else if (strcmp(option, "lift_wall_z") == 0) {
			READ_REAL("lift_wall_z", &ti->lift_wall_z);
		} else if (strcmp(option, "max_tirrad") == 0) {
			READ_REAL("max_tirrad", &ti->max_tirrad);
		} else if (strcmp(option, "max_tirrad_f") == 0) {
			READ_REAL("max_tirrad_f", &ti->max_tirrad_f);
		} else if (strcmp(option, "min_tirrad") == 0) {
			READ_REAL("min_tirrad", &ti->min_tirrad);
		} else if (strcmp(option, "height") == 0) {
			READ_REAL("height", &ti->height);
		} else if (strcmp(option, "min_mtow") == 0) {
			READ_REAL("min_mtow", &ti->min_mtow);
		} else if (strcmp(option, "max_mtow") == 0) {
			READ_REAL("max_mtow", &ti->max_mtow);
		} else if (strcmp(option, "min_nlg_len") == 0) {
			READ_REAL("min_nlg_len", &ti->min_nlg_len);
		} else if (strcmp(option, "lift_height") == 0) {
			READ_REAL("lift_height", &ti->lift_height);
		} else if (strcmp(option, "arpt") == 0) {
			READ_FILENAME("arpt", ti->arpt);
		} else if (strcmp(option, "engine_snd") == 0) {
			READ_FILENAME("engine_snd", ti->engine_snd);
		} else if (strcmp(option, "air_snd") == 0) {
			READ_FILENAME("air_snd", ti->air_snd);
		} else if (strcmp(option, "beeper_snd") == 0) {
			READ_FILENAME("beeper_snd", ti->beeper_snd);
		} else if (strcmp(option, "anim_debug") == 0) {
			logMsg("Animation debugging active on %s", tugdir);
			ti->anim_debug = B_TRUE;
		} else if (strcmp(option, "drive_debug") == 0) {
			logMsg("Driving debugging active on %s", tugdir);
			ti->drive_debug = B_TRUE;
		} else if (strcmp(option, "quick_debug") == 0) {
			logMsg("Quick test debugging active on %s", tugdir);
			ti->quick_debug = B_TRUE;
		} else {
			logMsg("Malformed tug config file %s: unknown "
			    "option '%s'", cfgfilename, option);
			goto errout;
		}
#undef	READ_REAL
#undef	READ_FILENAME
#undef	FIX_PATHSEP
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
	VALIDATE_TUG_REAL(ti->max_fwd_speed, "max_fwd_speed");
	VALIDATE_TUG_REAL(ti->max_rev_speed, "max_rev_speed");
	VALIDATE_TUG_REAL(ti->max_accel, "max_accel");
	VALIDATE_TUG_REAL(ti->max_decel, "max_decel");
	VALIDATE_TUG_REAL(ti->max_steer, "max_steer");
	VALIDATE_TUG_REAL_NAN(ti->front_z, "front_z");
	VALIDATE_TUG_REAL(ti->front_radius, "front_r");
	VALIDATE_TUG_REAL_NAN(ti->rear_z, "rear_z");
	VALIDATE_TUG_REAL(ti->rear_radius, "rear_r");
	VALIDATE_TUG_REAL_NAN(ti->lift_wall_z, "lift_wall_z");
	VALIDATE_TUG_REAL(ti->max_tirrad, "max_tirrad");
	VALIDATE_TUG_REAL(ti->max_tirrad_f, "max_tirrad_f");
	VALIDATE_TUG_REAL(ti->min_tirrad, "min_tirrad");
	VALIDATE_TUG_REAL(ti->max_mtow, "max_mtow");
	if (ti->min_mtow >= ti->max_mtow) {
		logMsg("Malformed tug config file %s: min_mtow >= max_mtow",
		    cfgfilename);
		goto errout;
	}
	VALIDATE_TUG_REAL(ti->min_nlg_len, "min_nlg_len");
	VALIDATE_TUG_REAL(ti->lift_height, "lift_height");
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
tug_info_select(double mtow, double ng_len, double tirrad, const char *arpt)
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

		if ((dot = strrchr(de->d_name, '.')) == NULL ||
		    strcmp(&dot[1], "tug") != 0)
			continue;

		tugpath = mkpathname(tugdir, de->d_name, NULL);
		ti = tug_info_read(tugpath);
		/* A debug flag means we always pick this tug. */
		if (ti != NULL && (ti->anim_debug || ti->drive_debug ||
		    ti->quick_debug)) {
			free(tugpath);
			goto out;
		}
		/*
		 * A tug matches our selection criteria iff:
		 * 1) it could be read AND
		 * 2) its min_mtow is <= our MTOW
		 * 3) its max_mtow is >= our MTOW
		 * 4) its min nosegear length is <= our nosegear length
		 * 5) our nose gear radius matches its min and max tire radius
		 * 6) if the caller provided an airport identifier and the
		 *	tug is airport-specific, then the airport ID matches
		 */
		if (ti != NULL && ti->min_mtow <= mtow &&
		    mtow <= ti->max_mtow && ti->min_nlg_len <= ng_len &&
		    ti->min_tirrad <= tirrad && ti->max_tirrad >= tirrad &&
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
out:
	while ((ti_oth = avl_destroy_nodes(&tis, &cookie)) != NULL)
		tug_info_free(ti_oth);
	avl_destroy(&tis);

	return (ti);
}

static int
reload_tug_handler(XPLMCommandRef cmd, XPLMCommandPhase phase, void *refcon)
{
	UNUSED(cmd);
	UNUSED(refcon);
	if (phase != xplm_CommandEnd)
		return (1);

	if (glob_tug != NULL) {
		XPLMUnloadObject(glob_tug->tug);
		glob_tug->tug = XPLMLoadObject(glob_tug->info->tug);
		VERIFY(glob_tug->tug != NULL);
	}

	return (1);
}

void
tug_glob_init(void)
{
	VERIFY(!inited);

	for (ANIM_t a = 0; a < TUG_NUM_ANIMS; a++)
		dr_create_f(&anim[a].dr, &anim[a].value, B_FALSE, anim[a].name);

	fdr_find(&sun_pitch_dr, "sim/graphics/scenery/sun_pitch_degrees");

	tug_reload_cmd = XPLMCreateCommand("BetterPushback/reload_tug",
            "Reload tug");
	XPLMRegisterCommandHandler(tug_reload_cmd, reload_tug_handler, 1, NULL);

	inited = B_TRUE;
}

void
tug_glob_fini(void)
{
	if (!inited)
		return;

	for (ANIM_t a = 0; a < TUG_NUM_ANIMS; a++)
		dr_delete(&anim[a].dr);

	inited = B_FALSE;
}

/*
 * Returns B_TRUE if we have a tug matching the aircraft selection criteria,
 * otherwise returns B_FALSE. Can be used to check for aircraft suitability
 * without having to actually load the tug object.
 */
bool_t
tug_available(double mtow, double ng_len, double tirrad, const char *arpt)
{
	tug_info_t *ti = tug_info_select(mtow, ng_len, tirrad, arpt);

	if (ti != NULL) {
		tug_info_free(ti);
		return (B_TRUE);
	}

	return (B_FALSE);
}

tug_t *
tug_alloc(double mtow, double ng_len, double tirrad, const char *arpt)
{
	tug_t *tug;

	VERIFY(inited);

	tug = calloc(1, sizeof (*tug));
	list_create(&tug->segs, sizeof (seg_t), offsetof(seg_t, node));

	/* Select a matching tug from our repertoire */
	tug->info = tug_info_select(mtow, ng_len, tirrad, arpt);
	if (tug->info == NULL)
		goto errout;

	tug->info = tug->info;
	tug->tirrad = tirrad;

	tug->veh.wheelbase = fabs(tug->info->front_z - tug->info->rear_z);
	tug->veh.wheelbase = MIN(tug->veh.wheelbase, TUG_MIN_WHEELBASE);
	tug->veh.fixed_z_off = tug->info->rear_z;
	tug->veh.max_steer = tug->info->max_steer;
	tug->veh.max_fwd_spd = tug->info->max_fwd_speed;
	tug->veh.max_rev_spd = tug->info->max_rev_speed;
	tug->veh.max_ang_vel = TUG_MAX_ANG_VEL;
	tug->veh.max_accel = tug->info->max_accel;
	tug->veh.max_decel = tug->info->max_decel;

	/* veh_slow is identical to 'veh', but with a much slower speed */
	tug->veh_slow = tug->veh;
	tug->veh_slow.max_fwd_spd = tug->veh.max_fwd_spd / 10;
	tug->veh_slow.max_rev_spd = tug->veh.max_rev_spd / 10;
	tug->veh_slow.max_accel = tug->info->max_accel / 3;
	tug->veh_slow.max_decel = tug->info->max_decel / 3;

	tug->tug = XPLMLoadObject(tug->info->tug);
	if (tug->tug == NULL) {
		logMsg("Error loading tug object %s", tug->info->tug);
		goto errout;
	}

	tug->engine_snd = wav_load(tug->info->engine_snd, "tug_engine");
	if (tug->engine_snd == NULL) {
		logMsg("Error loading tug sound %s", tug->info->engine_snd);
		goto errout;
	}
	wav_set_loop(tug->engine_snd, B_TRUE);

	if (tug->info->engine_snd != NULL) {
		tug->air_snd = wav_load(tug->info->air_snd, "tug_air");
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

	/*
	 * Initial state is:
	 * 1) lift in upper position
	 * 2) arms fully closed
	 * 3) cradle lights off
	 * 4) hazard lights off
	 */
	anim[ANIM_LIFT].value = 1;
	anim[ANIM_LIFT_ARM].value = 0;
	anim[ANIM_CRADLE_LIGHTS].value = 0;
	anim[ANIM_HAZARD_LIGHTS].value = 0;

	glob_tug = tug;

	return (tug);
errout:
	tug_free(tug);
	return (NULL);
}

void
tug_free(tug_t *tug)
{
	seg_t *seg;

	while ((seg = list_remove_head(&tug->segs)) != NULL)
		free(seg);
	list_destroy(&tug->segs);

	if (tug->info != NULL)
		tug_info_free(tug->info);

	if (tug->tug != NULL)
		XPLMUnloadObject(tug->tug);

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

	free(tug);
	glob_tug = NULL;
}

void
tug_set_pos(tug_t *tug, vect2_t pos, double hdg, double spd)
{
	seg_t *seg;

	tug->pos.pos = pos;
	tug->pos.hdg = hdg;
	tug->pos.spd = spd;

	/* flush any driving segments as those will have been invalidated */
	while ((seg = list_remove_head(&tug->segs)) != NULL)
		free(seg);
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

	tug->steer_override = B_FALSE;

	return (compute_segs(&tug->veh, cur_pos, cur_hdg, dst, hdg,
	    &tug->segs) >= 0);
}

void
tug_run(tug_t *tug, double d_t, bool_t drive_slow)
{
	double steer = 0, speed = 0;
	double accel, turn, radius;

	if (list_head(&tug->segs) != NULL) {
		drive_segs(&tug->pos, drive_slow ? &tug->veh_slow : &tug->veh,
		    &tug->segs, &tug->last_mis_hdg, d_t, &steer, &speed);
	}

	/* modulate our speed based on required steering angle */
	if (speed > 0) {
		speed = MIN(speed, tug->veh.max_fwd_spd *
		    (1.1 - (steer / tug->veh.max_steer)));
	} else {
		speed = MAX(speed, -tug->veh.max_rev_spd *
		    (1.1 - (steer / tug->veh.max_steer)));
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
	if (!tug->steer_override)
		tug->cur_steer += turn;

	radius = tan(DEG2RAD(90 - tug->cur_steer)) * tug->veh.wheelbase;
	if (radius > -1e3 && radius < 1e3) {
		double d_hdg = RAD2DEG((tug->pos.spd / radius) * d_t);
		vect2_t p2c = VECT2(radius, tug->info->rear_z);
		vect2_t c2np = vect2_rot(vect2_neg(p2c), d_hdg);
		vect2_t d_pos = vect2_rot(vect2_add(p2c, c2np), tug->pos.hdg);
		UNUSED(d_pos);
		if (!tug->info->anim_debug) {
			tug->pos.pos = vect2_add(tug->pos.pos, d_pos);
			tug->pos.hdg = normalize_hdg(tug->pos.hdg + d_hdg);
		}
	} else {
		if (!tug->info->anim_debug) {
			vect2_t dir = hdg2dir(tug->pos.hdg);
			tug->pos.pos = vect2_add(tug->pos.pos, vect2_scmul(dir,
			    tug->pos.spd * d_t));
		}
	}

	tug_set_TE_snd(tug, (ABS(tug->pos.spd) / tug->info->max_fwd_speed) / 4);
}

void
tug_anim(tug_t *tug, double d_t)
{
	/* Set up our wheel animations */
	if (!tug->info->anim_debug) {
		anim[ANIM_FRONT_DRIVE].value = anim_gate(((tug->pos.spd * d_t) /
		    (2 * M_PI * tug->info->front_radius)) +
		    anim[ANIM_FRONT_DRIVE].value);

		anim[ANIM_REAR_DRIVE].value = anim_gate(((tug->pos.spd * d_t) /
		    (2 * M_PI * tug->info->rear_radius)) +
		    anim[ANIM_REAR_DRIVE].value);

		anim[ANIM_FRONT_STEER].value = (tug->cur_steer /
		    (2 * tug->veh.max_steer)) + 0.5;

		anim[ANIM_VEHICLE_LIGHTS].value =
		    (dr_getf(&sun_pitch_dr) < LIGHTS_ON_SUN_ANGLE);
		if (tug->pos.spd < -0.1) {
			anim[ANIM_REVERSE_LIGHTS].value = B_TRUE;
			anim[ANIM_DRIVER_ORIENTATION].value = MIN(1,
			    anim[ANIM_DRIVER_ORIENTATION].value +
			    d_t / DRIVER_TURN_TIME);
		} else if (tug->pos.spd > 0.1) {
			anim[ANIM_REVERSE_LIGHTS].value = B_FALSE;
			anim[ANIM_DRIVER_ORIENTATION].value = MAX(0,
			    anim[ANIM_DRIVER_ORIENTATION].value -
			    d_t * DRIVER_TURN_TIME);
		}
		anim[ANIM_CRADLE_LIGHTS].value = (cradle_lights_req &&
		    dr_getf(&sun_pitch_dr) < LIGHTS_ON_SUN_ANGLE);
	} else {
		uint64_t mt = microclock();
		float value;

		value = (mt % 3000000) / 3000000.0;
		anim[ANIM_FRONT_DRIVE].value = value;
		anim[ANIM_FRONT_STEER].value = value;
		anim[ANIM_REAR_DRIVE].value = value;
		anim[ANIM_LIFT].value = value;
		anim[ANIM_LIFT_ARM].value = value;
		anim[ANIM_TIRE_SENSE].value = value;
		anim[ANIM_DRIVER_ORIENTATION].value = value;

		value = (mt / 1000000) % 2;
		anim[ANIM_VEHICLE_LIGHTS].value = value;
		anim[ANIM_CRADLE_LIGHTS].value = value;
		anim[ANIM_REVERSE_LIGHTS].value = value;
		anim[ANIM_HAZARD_LIGHTS].value = value;
	}
}

void
tug_draw(tug_t *tug, double cur_t)
{
	XPLMDrawInfo_t di;
	XPLMProbeRef probe = XPLMCreateProbe(xplm_ProbeY);
	XPLMProbeInfo_t info = { .structSize = sizeof (XPLMProbeInfo_t) };
	vect3_t pos, norm, norm_hdg;
	double gain;

	/* X-Plane's Z axis is inverted to ours */
	VERIFY3U(XPLMProbeTerrainXYZ(probe, tug->pos.pos.x, 0,
	    -tug->pos.pos.y, &info), ==, xplm_ProbeHitTerrain);
	/* Must be upright, no driving on ceilings! */
	ASSERT3F(info.normalY, >, 0.0);

	pos = VECT3(tug->pos.pos.x, info.locationY, -tug->pos.pos.y);
	norm = VECT3(info.normalX, info.normalY, info.normalZ);
	pos = vect3_add(pos, vect3_set_abs(norm, tug->info->height));

	norm_hdg = vect3_rot(VECT3(info.normalX, info.normalY, -info.normalZ),
	    tug->pos.hdg, 1);

	di.structSize = sizeof (di);
	di.x = pos.x;
	di.y = pos.y;
	di.z = pos.z;
	di.heading = tug->pos.hdg;
	di.pitch = -RAD2DEG(atan(norm_hdg.z / norm_hdg.y));
	di.roll = RAD2DEG(atan(norm_hdg.x / norm_hdg.y));

	XPLMDrawObjects(tug->tug, 1, &di, 1, 1);

	/*
	 * Sound control. No more drawing after this point.
	 */
	if (!tug->engine_snd_playing) {
		wav_play(tug->engine_snd);
		tug->engine_snd_playing = B_TRUE;
	}

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

	XPLMDestroyProbe(probe);
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

void
tug_set_steering(tug_t *tug, double req_steer, double d_t)
{
	double d_steer = (req_steer - tug->cur_steer) * d_t;
	double max_d_steer = TUG_STEER_RATE * d_t;
	d_steer = MIN(d_steer, max_d_steer);
	d_steer = MAX(d_steer, -max_d_steer);
	tug->cur_steer += d_steer;
	tug->steer_override = (list_head(&tug->segs) == NULL);
}

bool_t
tug_is_stopped(const tug_t *tug)
{
	return (list_head(&tug->segs) == NULL && tug->pos.spd == 0);
}

void
tug_set_lift_pos(float x)
{
	anim[ANIM_LIFT].value = MAX(MIN(x, 1.0), 0.0);
}

void
tug_set_lift_arm_pos(const tug_t *tug, float x, bool_t grabbing_tire)
{
	double min_val = 0;
	if (grabbing_tire) {
		const tug_info_t *ti = tug->info;
		min_val = wavg(0, ti->max_tirrad_f,
		    (tug->tirrad - ti->min_tirrad) / (ti->max_tirrad -
		    ti->min_tirrad));
	}

	anim[ANIM_LIFT_ARM].value = MAX(MIN(x, 1.0), min_val);
}

void
tug_set_tire_sense_pos(const tug_t *tug, float x)
{
	const tug_info_t *ti = tug->info;
	double max_val = wavg(0.0, 1.0, (tug->tirrad - ti->min_tirrad) /
	    (ti->max_tirrad - ti->min_tirrad));
	anim[ANIM_TIRE_SENSE].value = MAX(MIN(x, max_val), 0.0);
}

void
tug_set_cradle_lights_on(bool_t flag)
{
	cradle_lights_req = flag;
}

void
tug_set_hazard_lights_on(bool_t flag)
{
	anim[ANIM_HAZARD_LIGHTS].value = flag;
}
