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

#define PCRE2_CODE_UNIT_WIDTH 8
#include <pcre2.h>

#include <XPLMUtilities.h>
#include <XPLMPlugin.h>

#include <acfutils/assert.h>
#include <acfutils/crc64.h>
#include <acfutils/helpers.h>
#include <acfutils/math.h>
#include <acfutils/time.h>

#include "driving.h"
#include "tug.h"
#include "xplane.h"

#define	TUG_STEER_RATE		40	/* deg/s */
#define	TUG_FAST_STEER_RATE	50	/* deg/s */

#define	TUG_MAX_ANG_VEL		20	/* deg/s */

#define	TUG_MAX_FWD_SPD		6	/* m/s */
#define	TUG_MAX_REV_SPD		3	/* m/s */
#define	TUG_MAX_ACCEL		1	/* m/s^2 */
#define	TUG_MAX_DECEL		0.5	/* m/s^2 */

#define	TUG_MIN_WHEELBASE	5	/* meters */

#define	VOLUME_INSIDE_MODIFIER	0.5

#define	TUG_TE_RAMP_UP_DELAY		3	/* seconds */
#define	TUG_TE_FAST_RAMP_UP_DELAY	1	/* seconds */

#define	TUG_CRADLE_CHG_D_T	0.5
#define	TUG_CRADLE_AIR_MOD	0.5
#define	TUG_HAZARD_REV_PER_SEC	2	/* revolutions per second of beacon */
#define	LIGHTS_ON_SUN_ANGLE	5	/* degrees, sets vehicle lights on */

#define	DRIVER_TURN_TIME	0.75	/* seconds for driver to turn around */
#define	CAB_LIFT_TIME		4	/* seconds for cab to lift/lower */

#define	BEACON_FLASH_INTVAL	1000	/* milliseconds */
#define	BEACON_FLASH_DUR	50	/* milliseconds */

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
	ANIM_CAB_POSITION,
	ANIM_WINCH_ON,
	ANIM_CLEAR_SIGNAL,
	ANIM_LIFT_IN_TRANSIT,
	ANIM_BEACON_FLASH,
	TUG_NUM_ANIMS
} anim_t;

typedef struct {
	const char	*name;
	dr_t		dr;
	float		value;
} anim_info_t;

/*
 * Livery name with a random sort value. We use this to pick a random
 * livery if multiple match at a given airport.
 */
typedef struct {
	char		*livname;
	bool_t		airline_matched;
	uint64_t	sort_rand;
	avl_node_t	node;
} liv_t;

/*
 * These are the datarefs that drive animation of the tug. Their meanings are:
 *
 * bp/anim/front_drive: 0.0 - 1.0: front wheels rolling forward, one full
 *	360-degree rotation.
 * bp/anim/front_steer: 0.0 - 1.0: front wheels steering, full left: 0.0,
 *	straight: 0.5, full right: 1.0. To simulate all-wheel steering, simply
 *	animate all wheels using this dataref. Currently BP doesn't support
 *	switching steering modes, so animate the tug with one steering mode
 *	only.
 * bp/anim/rear_drive: 0.0 - 1.0: same as front_drive, but for rear wheels
 * bp/anim/lift: 0.0 - 1.0: main lifting mechanism motion from full down
 *	(0.0) to full up (1.0). The height of the lift is defined in the
 *	tug's info.cfg file.
 * bp/anim/lift_arm: 0.0 - 1.0: the arms grasping the nosewheel from fully
 *	closed (0.0) to fully open (1.0). 0.0 closed should correspond to
 *	the smallest wheel size that the tug can handle, and 1.0 to the fully
 *	open position, so that the tug can attach to the aircraft. Somewhere
 *	in between is the maximum wheel size the tug can support. This
 *	animation value is defined by the "max_tirrad_f" parameter in the
 *	tug's info.cfg file.
 * bp/anim/tire_sense: 0.0 - 1.0: many tugs feature a set of tire-sensing
 *	arms that adjust according to the nosewheel size. 0.0 is for minimum
 *	tire size and 1.0 for maximum.
 * bp/anim/vehicle_lights: 0/1: a show/hide flag that is set to 0 to hide
 *	night lights and 1 to show them (BP sets this when the sun gets so
 *	low that X-Plane turns on global night lighting).
 * bp/anim/cradle_lights: 0/1: a show/hide flag that is set to 0 hide the
 *	lights illuminating the lift cradle and 1 to show them.
 * bp/anim/reverse_lights: 0/1: a show/hide flag to indicate whether to show
 *	the white reverse light.
 * bp/anim/hazard_lights: 0/1: a show/hide flag to indicate whether to show
 *	the rotating hazard beacon light.
 * bp/anim/driver_orientation: 0.0 - 1.0: the driver's orientation from
 *	0.0 (forward) to 1.0 (backward). BP uses this to animate the driver
 *	turning their seat around to face backward when reversing.
 * bp/anim/cab_position: 0.0 - 1.0: for tugs with elevating cabs, 0.0 is
 *	the cab fully down and 1.0 is the cab fully up. BP sets this at the
 *	same time as the driver, i.e. the cab either raises or lowers
 *	depending on the direction the tug is moving (down - forward, up -
 *	backward).
 */

static anim_info_t anim[TUG_NUM_ANIMS] = {
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
    { .name = "bp/anim/driver_orientation" },
    { .name = "bp/anim/cab_position" },
    { .name = "bp/anim/winch_on" },
    { .name = "bp/anim/clear_signal" },
    { .name = "bp/anim/lift_in_transit" },
    { .name = "bp/anim/beacon_flash" }
};

static bool_t cradle_lights_req = B_FALSE;
static dr_t sun_pitch_dr;
static bool_t inited = B_FALSE;
static tug_t *glob_tug = NULL;
static XPLMCommandRef tug_reload_cmd;

static char *tug_liv_apply(const tug_info_t *ti);
static void tug_liv_destroy(char *objpath);

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
 * Livery name sorting function for avl_tree_t.
 */
int
liv_compar(const void *a, const void *b)
{
	const liv_t *la = a, *lb = b;
	int n;

	/* Airline matching takes precedence */
	if (la->airline_matched && !lb->airline_matched)
		return (-1);
	if (!la->airline_matched && lb->airline_matched)
		return (1);

	/* Otherwise, we sort by our random value */
	if (la->sort_rand < lb->sort_rand)
		return (-1);
	if (la->sort_rand > lb->sort_rand)
		return (1);

	/*
	 * Could be random coincidence the sort values are equal, sort
	 * by livery name - MUST be unique.
	 */
	n = strcmp(la->livname, lb->livname);
	if (n < 0)
		return (-1);
	if (n > 0)
		return (1);

	ASSERT0(memcmp(la, lb, sizeof (*la)));
	return (0);
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
	free(ti->tug_name);
	free(ti->tugdir);
	free(ti->tug);
	free(ti->arpt);
	free(ti->engine_snd);
	free(ti->air_snd);
	free(ti->beeper_snd);
	free(ti->engine_snd_in);
	free(ti->air_snd_in);
	free(ti->beeper_snd_in);
	free(ti->livname);
	free(ti);
}

static bool_t
regex_setup(const char *regex_str, const char *filename, pcre2_code **re,
    pcre2_match_data **match_data, uint32_t compile_options)
{
	int err;
	PCRE2_SIZE erroff;

	*re = pcre2_compile((PCRE2_SPTR)regex_str, PCRE2_ZERO_TERMINATED,
	    compile_options, &err, &erroff, NULL);
	if (*re == NULL) {
		PCRE2_UCHAR buf[256];

		pcre2_get_error_message(err, buf, sizeof (buf));
		logMsg("Malformed tug livery config %s: regex compile "
		    "failed \"%s\": char %d: %s", filename, regex_str,
		    (int)erroff, buf);
		return (B_FALSE);
	}
	*match_data = pcre2_match_data_create_from_pattern(*re, NULL);

	return (B_TRUE);
}

static bool_t
tug_info_liv_cfg_match(const char *info_pathname, const char *icao,
    const char *airline, bool_t *force_pick_this, bool_t *is_airline,
    bool_t *airline_matched, bool_t *airline_optional)
{
	FILE *fp = fopen(info_pathname, "r");
	bool_t icao_matched = B_FALSE, match_debug = B_FALSE;

	*force_pick_this = B_FALSE;

	if (fp == NULL) {
		logMsg("Error reading tug livery %s: cannot open info.cfg: %s",
		    info_pathname, strerror(errno));
		    return (B_FALSE);
	}
	while (!feof(fp)) {
		char cmd[32];

		if (fscanf(fp, "%31s", cmd) != 1)
			break;
		if (*cmd == '#') {
			/* skip comments */
			char c;
			do {
				c = fgetc(fp);
			} while (c != '\n' && c != EOF);
			continue;
		}

		if (strcmp(cmd, "icao") == 0) {
			char *regex_str;
			int rc;
			pcre2_code *re;
			pcre2_match_data *match_data;

			regex_str = parser_get_next_quoted_str(fp);
			if (icao_matched) {
				/* one match is enough */
				free(regex_str);
				continue;
			}

			if (regex_str == NULL || *regex_str == 0 ||
			    !regex_setup(regex_str, info_pathname, &re,
			    &match_data, 0)) {
				logMsg("Malformed tug livery config %s: bad "
				    "regex following \"icao\".", info_pathname);
				free(regex_str);
				break;
			}

			rc = pcre2_match(re, (PCRE2_SPTR)icao, strlen(icao),
			    0, 0, match_data, NULL);
			pcre2_match_data_free(match_data);
			pcre2_code_free(re);

			if (rc >= 0) {
				if (match_debug) {
					logMsg("match_debug: regex \"%s\" "
					    "matches airport \"%s\"",
					    regex_str, icao);
				}
				icao_matched = B_TRUE;
			} else if (match_debug) {
				logMsg("match_debug: regex \"%s\" DOESN'T "
				    "match airport \"%s\"", regex_str, icao);
			}
			free(regex_str);
		} else if (strcmp(cmd, "airline") == 0 ||
		    strcmp(cmd, "airlinei") == 0) {
			bool_t caseign = (strcmp(cmd, "airlinei") == 0);
			char *regex_str;
			int rc;
			pcre2_code *re;
			pcre2_match_data *match_data;
			uint32_t flags = PCRE2_UTF;
			*is_airline = B_TRUE;

			if (caseign)
				flags |= PCRE2_CASELESS;
			regex_str = parser_get_next_quoted_str(fp);
			if (*airline_matched) {
				/* one match is enough */
				free(regex_str);
				continue;
			}

			if (regex_str == NULL || *regex_str == 0 ||
			    !regex_setup(regex_str, info_pathname, &re,
			    &match_data, flags)) {
				logMsg("Malformed tug livery config %s: bad "
				    "regex following \"airline%s\".",
				    info_pathname, caseign ? "i" : "");
				free(regex_str);
				break;
			}
			rc = pcre2_match(re, (PCRE2_SPTR)airline,
			    strlen(airline), 0, 0, match_data, NULL);
			pcre2_match_data_free(match_data);
			pcre2_code_free(re);
			if (rc >= 0) {
				if (match_debug) {
					logMsg("match_debug: regex \"%s\" "
					    "matches airline \"%s\"",
					    regex_str, airline);
				}
				*airline_matched = B_TRUE;
			} else if (match_debug) {
				logMsg("match_debug: regex \"%s\" DOESN'T "
				    "match airline \"%s\"", regex_str, airline);
				*airline_matched = B_FALSE;
			}
			free(regex_str);
		} else if (strcmp(cmd, "airline_opt") == 0) {
			*airline_optional = B_TRUE;
		} else if (strcmp(cmd, "match_debug") == 0) {
			match_debug = B_TRUE;
		} else if (strcmp(cmd, "paint_debug") == 0) {
			icao_matched = B_TRUE;
			*force_pick_this = B_TRUE;
			logMsg("%s: \"paint_debug\" present forcibly picking",
			    info_pathname);
		} else {
			logMsg("Malformed tug livery config %s: unknown "
			    "keyword \"%s\".", info_pathname, cmd);
			break;
		}
	}

	fclose(fp);

	return (icao_matched);
}

static bool_t
tug_info_liv_select(tug_info_t *ti, const char *icao, const char *airline)
{
	bool_t isdir;
	char *livdir_path;
	avl_tree_t livs;
	bool_t found;
	void *cookie;
	liv_t *liv;

	avl_create(&livs, liv_compar, sizeof (liv_t), offsetof(liv_t, node));

	/* read the livery directory */
	if (icao != NULL && *icao != 0) {
		DIR *livdir;
		struct dirent *de;

		livdir_path = mkpathname(ti->tugdir, "liveries", NULL);
		livdir = opendir(livdir_path);

		if (livdir == NULL) {
			free(livdir_path);
			return (B_FALSE);
		}

		while ((de = readdir(livdir)) != NULL) {
			const char *dot = strrchr(de->d_name, '.');
			char *info_pathname;
			bool_t matched, force_pick_this;
			bool_t is_airline = B_FALSE, airline_matched = B_FALSE;
			bool_t airline_optional = B_FALSE;

			/* only select subdirs ending in ".livery" */
			if (dot == NULL || strcmp(&dot[1], "livery") != 0 ||
			    strcmp(de->d_name, "generic.livery") == 0)
				continue;
			info_pathname = mkpathname(ti->tugdir, "liveries",
			    de->d_name, "info.cfg", NULL);
			matched = tug_info_liv_cfg_match(info_pathname, icao,
			    airline, &force_pick_this, &is_airline,
			    &airline_matched, &airline_optional);
			if (matched && (!is_airline || !airline_optional ||
			    !airline_matched || force_pick_this)) {
				liv_t *liv = calloc(1, sizeof (*liv));
				liv->livname = strdup(de->d_name);
				liv->sort_rand = crc64_rand();
				liv->airline_matched = airline_matched;

				if (force_pick_this) {
					/*
					 * Delete all other liveries and
					 * immediately complete the loop.
					 */
					liv_t *oth_liv;

					cookie = NULL;
					while ((oth_liv = avl_destroy_nodes(
					    &livs, &cookie)) != NULL) {
						free(oth_liv->livname);
						free(oth_liv);
					}
					avl_add(&livs, liv);
					break;
				}
				avl_add(&livs, liv);
			}
			free(info_pathname);
		}

		closedir(livdir);
		free(livdir_path);
	}

	if (avl_numnodes(&livs) != 0) {
		/* Pick the first that matched */
		liv_t *liv = avl_first(&livs);
		ti->livname = strdup(liv->livname);
		found = B_TRUE;
	} else {
		/* No livery matched, see if we have a generic one */
		livdir_path = mkpathname(ti->tugdir, "liveries",
		    "generic.livery",
		    NULL);
		if (!file_exists(livdir_path, &isdir) || !isdir) {
			free(livdir_path);
			found = B_FALSE;
		} else {
			ti->livname = strdup("generic.livery");
			free(livdir_path);
			found = B_TRUE;
		}
	}

	cookie = NULL;
	while ((liv = avl_destroy_nodes(&livs, &cookie)) != NULL) {
		free(liv->livname);
		free(liv);
	}
	avl_destroy(&livs);

	return (found);
}

static tug_info_t *
tug_info_read(const char *tugdir, const char *tug_name, const char *icao,
    const char *airline)
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
	ti->tug_name = strdup(tug_name);
	ti->tugdir = strdup(tugdir);
	ti->front_z = NAN;
	ti->rear_z = NAN;
	ti->lift_wall_z = NAN;
	ti->apch_dist = NAN;
	ti->max_TE = NAN;
	ti->plat_z = NAN;
	ti->plat_h = NAN;
	ti->sort_rand = crc64_rand();

	/* set some defaults */
	ti->max_fwd_speed = TUG_MAX_FWD_SPD;
	ti->max_rev_speed = TUG_MAX_REV_SPD;
	ti->max_accel = TUG_MAX_ACCEL;
	ti->max_decel = TUG_MAX_DECEL;
	ti->num_fwd_gears = 1;
	ti->num_rev_gears = 1;
	ti->gear_compat = UINT_MAX;

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
#define	READ_NUMBER(fmt, optname, result) \
	do { \
		if (fscanf(fp, fmt, (result)) != 1) { \
			logMsg("Malformed tug config file %s: expected " \
			    "number following '" optname "'", cfgfilename); \
			goto errout; \
		} \
	} while (0)

		if (strcmp(option, "tug_obj") == 0) {
			READ_FILENAME("tug_obj", ti->tug);
		} else if (strcmp(option, "max_steer") == 0) {
			READ_NUMBER("%lf", "max_steer", &ti->max_steer);
		} else if (strcmp(option, "max_fwd_speed") == 0) {
			READ_NUMBER("%lf", "max_fwd_speed", &ti->max_fwd_speed);
		} else if (strcmp(option, "max_rev_speed") == 0) {
			READ_NUMBER("%lf", "max_rev_speed", &ti->max_rev_speed);
		} else if (strcmp(option, "max_accel") == 0) {
			READ_NUMBER("%lf", "max_accel", &ti->max_accel);
		} else if (strcmp(option, "max_decel") == 0) {
			READ_NUMBER("%lf", "max_decel", &ti->max_decel);
		} else if (strcmp(option, "front_z") == 0) {
			READ_NUMBER("%lf", "front_z", &ti->front_z);
		} else if (strcmp(option, "front_r") == 0) {
			READ_NUMBER("%lf", "front_r", &ti->front_radius);
		} else if (strcmp(option, "rear_z") == 0) {
			READ_NUMBER("%lf", "rear_z", &ti->rear_z);
		} else if (strcmp(option, "rear_r") == 0) {
			READ_NUMBER("%lf", "rear_r", &ti->rear_radius);
		} else if (strcmp(option, "lift_wall_z") == 0) {
			READ_NUMBER("%lf", "lift_wall_z", &ti->lift_wall_z);
		} else if (strcmp(option, "max_tirrad") == 0) {
			READ_NUMBER("%lf", "max_tirrad", &ti->max_tirrad);
		} else if (strcmp(option, "max_tirrad_f") == 0) {
			READ_NUMBER("%lf", "max_tirrad_f", &ti->max_tirrad_f);
		} else if (strcmp(option, "min_tirrad") == 0) {
			READ_NUMBER("%lf", "min_tirrad", &ti->min_tirrad);
		} else if (strcmp(option, "height") == 0) {
			READ_NUMBER("%lf", "height", &ti->height);
		} else if (strcmp(option, "min_mtow") == 0) {
			READ_NUMBER("%lf", "min_mtow", &ti->min_mtow);
		} else if (strcmp(option, "max_mtow") == 0) {
			READ_NUMBER("%lf", "max_mtow", &ti->max_mtow);
		} else if (strcmp(option, "min_nlg_len") == 0) {
			READ_NUMBER("%lf", "min_nlg_len", &ti->min_nlg_len);
		} else if (strcmp(option, "lift_height") == 0) {
			READ_NUMBER("%lf", "lift_height", &ti->lift_height);
		} else if (strcmp(option, "apch_dist") == 0) {
			READ_NUMBER("%lf", "apch_dist", &ti->apch_dist);
		} else if (strcmp(option, "max_TE") == 0) {
			READ_NUMBER("%lf", "max_TE", &ti->max_TE);
		} else if (strcmp(option, "num_fwd_gears") == 0) {
			READ_NUMBER("%u", "num_fwd_gears", &ti->num_fwd_gears);
		} else if (strcmp(option, "num_rev_gears") == 0) {
			READ_NUMBER("%u", "num_rev_gears", &ti->num_rev_gears);
		} else if (strcmp(option, "gear_compat") == 0) {
			READ_NUMBER("%x", "gear_compat", &ti->gear_compat);
		} else if (strcmp(option, "plat_z") == 0) {
			READ_NUMBER("%lf", "plat_z", &ti->plat_z);
		} else if (strcmp(option, "plat_h") == 0) {
			READ_NUMBER("%lf", "plat_h", &ti->plat_h);
		} else if (strcmp(option, "arpt") == 0) {
			READ_FILENAME("arpt", ti->arpt);
		} else if (strcmp(option, "engine_snd") == 0) {
			READ_FILENAME("engine_snd", ti->engine_snd);
		} else if (strcmp(option, "air_snd") == 0) {
			READ_FILENAME("air_snd", ti->air_snd);
		} else if (strcmp(option, "beeper_snd") == 0) {
			READ_FILENAME("beeper_snd", ti->beeper_snd);
		} else if (strcmp(option, "engine_snd_inside") == 0) {
			READ_FILENAME("engine_snd_inside", ti->engine_snd_in);
		} else if (strcmp(option, "air_snd_inside") == 0) {
			READ_FILENAME("air_snd_inside", ti->air_snd_in);
		} else if (strcmp(option, "beeper_snd_inside") == 0) {
			READ_FILENAME("beeper_snd_inside", ti->beeper_snd_in);
		} else if (strcmp(option, "anim_debug") == 0) {
			logMsg("Animation debugging active on %s", tugdir);
			ti->anim_debug = B_TRUE;
		} else if (strcmp(option, "drive_debug") == 0) {
			logMsg("Driving debugging active on %s", tugdir);
			ti->drive_debug = B_TRUE;
		} else if (strcmp(option, "quick_debug") == 0) {
			logMsg("Quick test debugging active on %s", tugdir);
			ti->quick_debug = B_TRUE;
		} else if (strcmp(option, "electric_drive") == 0) {
			ti->electric_drive = B_TRUE;
		} else if (strcmp(option, "lift_type") == 0) {
			char type[16];
			if (fscanf(fp, "%15s", type) != 1) {
				logMsg("Malformed tug config file %s: expected "
				    "word following \"lift_type\" keyword.",
				    cfgfilename);
				goto errout;
			}
			if (strcmp(type, "grab") == 0) {
				ti->lift_type = LIFT_GRAB;
			} else if (strcmp(type, "winch") == 0) {
				ti->lift_type = LIFT_WINCH;
			} else {
				logMsg("Malformed tug config file %s: invalid "
				    "value \"%s\" for \"lift_type\" (expected "
				    "\"grab\" or \"winch\").", cfgfilename,
				    type);
				goto errout;
			}
		} else if (strcmp(option, "lift_wall_loc") == 0) {
			char type[16];
			if (fscanf(fp, "%15s", type) != 1) {
				logMsg("Malformed tug config file %s: expected "
				    "word following \"lift_wall_loc\" keyword.",
				    cfgfilename);
				goto errout;
			}
			if (strcmp(type, "front") == 0) {
				ti->lift_wall_loc = LIFT_WALL_FRONT;
			} else if (strcmp(type, "center") == 0) {
				ti->lift_wall_loc = LIFT_WALL_CENTER;
			} else if (strcmp(type, "back") == 0) {
				ti->lift_wall_loc = LIFT_WALL_BACK;
			} else {
				logMsg("Malformed tug config file %s: bad "
				    "keyword following \"lift_wall_loc\".",
				    cfgfilename);
				goto errout;
			}
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
#define	VALIDATE_TUG_INT(field, optname) \
	VALIDATE_TUG((field) <= 0, (optname))
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
	VALIDATE_TUG_REAL_NAN(ti->apch_dist, "apch_dist");
	VALIDATE_TUG_REAL_NAN(ti->max_TE, "max_TE");
	VALIDATE_TUG_INT(ti->num_fwd_gears, "num_fwd_gears");
	VALIDATE_TUG_INT(ti->num_rev_gears, "num_rev_gears");
	VALIDATE_TUG_STR(ti->engine_snd, "engine_snd");
	if (ti->lift_type == LIFT_WINCH) {
		VALIDATE_TUG_REAL_NAN(ti->plat_z, "plat_z");
		VALIDATE_TUG_REAL_NAN(ti->plat_h, "plat_h");
	}

	if (ti->engine_snd_in == NULL)
		ti->engine_snd_in = strdup(ti->engine_snd);
	if (ti->air_snd_in == NULL && ti->air_snd != NULL)
		ti->air_snd_in = strdup(ti->air_snd);
	if (ti->beeper_snd_in == NULL && ti->beeper_snd != NULL)
		ti->beeper_snd_in = strdup(ti->beeper_snd);

#undef	VALIDATE_TUG_STR
#undef	VALIDATE_TUG_REAL
#undef	VALIDATE_TUG_REAL_NAN
#undef	VALIDATE_TUG

	if (!tug_info_liv_select(ti, icao, airline))
		goto errout;

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
tug_info_select(double mtow, double ng_len, double tirrad, unsigned gear_type,
    const char *arpt, const char *airline, char **reason)
{
	char *tugdir;
	DIR *dirp;
	struct dirent *de;
	avl_tree_t tis;
	tug_info_t *ti, *ti_oth;
	void *cookie = NULL;
	size_t cap = 0;

	if (reason != NULL) {
		ASSERT3P(*reason, ==, NULL);
		append_format(reason, &cap, "ACF: mtow: %.0f nlg_len: %.2f "
		    "tirrad: %.3f gear_type: %u icao: %s\n", mtow, ng_len,
		    tirrad, gear_type, arpt != NULL ? arpt : "(nil)");
	}

	tugdir = mkpathname(bp_xpdir, bp_plugindir, "objects", "tugs", NULL);
	dirp = opendir(tugdir);
	if (dirp == NULL) {
#if	!IBM	/* On Windows libacfutils already prints the error */
		logMsg("Error reading directory %s: %s", tugdir,
		    strerror(errno));
#endif	/* !IBM */
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
		ti = tug_info_read(tugpath, de->d_name, arpt, airline);
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
		if (ti != NULL && reason != NULL) {
			append_format(reason, &cap, "%s: min_mtow: %.0f "
			    "max_mtow: %.0f min_tirrad: %.3f max_tirrad: %.3f "
			    "min_nlg_len: %.2f gear_compat: %x icao: %s; "
			    "result=", de->d_name, ti->min_mtow, ti->max_mtow,
			    ti->min_tirrad, ti->max_tirrad, ti->min_nlg_len,
			    ti->gear_compat,
			    ti->arpt != NULL ? ti->arpt : "(nil)");
		}
		if (ti != NULL && ti->min_mtow <= mtow &&
		    mtow <= ti->max_mtow && ti->min_nlg_len <= ng_len &&
		    ti->min_tirrad <= tirrad && ti->max_tirrad >= tirrad &&
		    (arpt == NULL || ti->arpt == NULL ||
		    strcmp(arpt, ti->arpt) == 0) &&
		    ((1 << gear_type) & ti->gear_compat) != 0) {
			if (reason != NULL)
			    append_format(reason, &cap, "ACCEPT\n");
			avl_add(&tis, ti);
		} else if (reason != NULL) {
		    append_format(reason, &cap, "REJECT\n");
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
		char *objpath;

		XPLMUnloadObject(glob_tug->tug);
		objpath = tug_liv_apply(glob_tug->info);
		if (objpath == NULL) {
			logMsg("Error preparing tug object %s",
			    glob_tug->info->tug);
			return (1);
		}
		glob_tug->tug = XPLMLoadObject(objpath);
		tug_liv_destroy(objpath);
		VERIFY(glob_tug->tug != NULL);
	}

	return (1);
}

void
tug_glob_init(void)
{
	VERIFY(!inited);

	for (anim_t a = 0; a < TUG_NUM_ANIMS; a++) {
		dr_create_f(&anim[a].dr, &anim[a].value, B_FALSE, "%s",
		    anim[a].name);
	}

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

	for (anim_t a = 0; a < TUG_NUM_ANIMS; a++)
		dr_delete(&anim[a].dr);

	inited = B_FALSE;
}

static void
tug_liv_subst(const tug_info_t *ti, FILE *fp, const char *tex_type,
    const char *tex_name)
{
	char *pathname;
	bool_t exists;
	char *tex_name_dds, *dot;

	pathname = mkpathname(ti->tugdir, "liveries", ti->livname, tex_name,
	    NULL);
	exists = file_exists(pathname, NULL);
	free(pathname);
	if (exists) {
		fprintf(fp, "%s liveries/%s/%s\n", tex_type, ti->livname,
		    tex_name);
		return;
	}
	/*
	 * If we didn't find the texture file under its original name,
	 * it can still exist as a DDS variant. Try with .dds instead.
	 */
	tex_name_dds = malloc(strlen(tex_name) + 3 + 1); /* +3 for 'dds' */
	strcpy(tex_name_dds, tex_name);
	dot = strrchr(tex_name_dds, '.');
	if (dot != NULL) {
		strcpy(&dot[1], "dds");
		pathname = mkpathname(ti->tugdir, "liveries",
		    ti->livname, tex_name_dds, NULL);
		exists = file_exists(pathname, NULL);
		free(pathname);
	}
	free(tex_name_dds);
	if (exists) {
		fprintf(fp, "%s liveries/%s/%s\n", tex_type,
		    ti->livname, tex_name);
	} else {
		fprintf(fp, "%s liveries/generic.livery/%s\n",
		    tex_type, tex_name);
	}
}

static char *
tug_liv_apply(const tug_info_t *ti)
{
	char *objpath = mkpathname(ti->tugdir, "inuse.obj", NULL);
	char *line = NULL;
	size_t cap = 0;
	ssize_t n;
	FILE *inobj = NULL, *outobj = NULL;

	inobj = fopen(ti->tug, "r");
	if (inobj == NULL) {
		logMsg("Cannot open tug object %s: %s", ti->tug,
		    strerror(errno));
		goto errout;
	}
	outobj = fopen(objpath, "w");
	if (outobj == NULL) {
		logMsg("Cannot open tug object for writing %s: %s", objpath,
		    strerror(errno));
		goto errout;
	}

	while ((n = getline(&line, &cap, inobj)) >= 0) {
		if (strncmp(line, "TEXTURE ", 8) == 0 ||
		    strncmp(line, "TEXTURE\t", 8) == 0) {
			memmove(line, &line[7], strlen(&line[7]) + 1);
			strip_space(line);
			tug_liv_subst(ti, outobj, "TEXTURE", line);
		} else if (strncmp(line, "TEXTURE_LIT ", 12) == 0 ||
		    strncmp(line, "TEXTURE_LIT\t", 12) == 0) {
			memmove(line, &line[11], strlen(&line[11]) + 1);
			strip_space(line);
			tug_liv_subst(ti, outobj, "TEXTURE_LIT", line);
		} else if (strncmp(line, "TEXTURE_NORMAL ", 15) == 0 ||
		    strncmp(line, "TEXTURE_NORMAL\t", 15) == 0) {
			memmove(line, &line[14], strlen(&line[14]) + 1);
			strip_space(line);
			tug_liv_subst(ti, outobj, "TEXTURE_NORMAL", line);
		} else {
			fwrite(line, 1, n, outobj);
		}
	}

	free(line);
	fclose(inobj);
	fclose(outobj);

	return (objpath);

errout:
	free(line);
	if (inobj != NULL)
		fclose(inobj);
	if (outobj != NULL) {
		(void) remove_file(objpath, B_FALSE);
		fclose(outobj);
	}
	free(objpath);

	return (NULL);
}

static void
tug_liv_destroy(char *objpath)
{
	(void) remove_file(objpath, B_FALSE);
	free(objpath);
}

/*
 * Returns B_TRUE if we have a tug matching the aircraft selection criteria,
 * otherwise returns B_FALSE. Can be used to check for aircraft suitability
 * without having to actually load the tug object.
 */
bool_t
tug_available(double mtow, double ng_len, double tirrad, unsigned gear_type,
    const char *arpt, const char *airline)
{
	char *reason = NULL;
	tug_info_t *ti = tug_info_select(mtow, ng_len, tirrad, gear_type,
	    arpt, airline, &reason);

	if (ti == NULL)
		logMsg("Failed to find a tug for you, reason:\n%s", reason);
	free(reason);

	if (ti != NULL) {
		tug_info_free(ti);
		return (B_TRUE);
	}

	return (B_FALSE);
}

static tug_t *
tug_alloc_common(tug_info_t *ti, double tirrad)
{
	tug_t *tug;
	char *objpath;

	VERIFY(inited);

	tug = calloc(1, sizeof (*tug));
	list_create(&tug->segs, sizeof (seg_t), offsetof(seg_t, node));

	tug->info = ti;
	tug->tirrad = tirrad;

	tug->veh.wheelbase = MAX(TUG_WHEELBASE(tug), TUG_MIN_WHEELBASE);
	tug->veh.fixed_z_off = tug->info->rear_z;
	tug->veh.max_steer = tug->info->max_steer;
	tug->veh.max_fwd_spd = tug->info->max_fwd_speed;
	tug->veh.max_rev_spd = tug->info->max_rev_speed;
	tug->veh.max_fwd_ang_vel = TUG_MAX_ANG_VEL;
	tug->veh.max_rev_ang_vel = TUG_MAX_ANG_VEL;
	tug->veh.max_accel = tug->info->max_accel;
	tug->veh.max_decel = tug->info->max_decel;
	tug->veh.xp10_bug_ign = B_TRUE;

	/* veh_slow is identical to 'veh', but with a much slower speed */
	tug->veh_slow = tug->veh;
	if (!tug->info->electric_drive)
		tug->veh_slow.max_fwd_spd = tug->veh.max_fwd_spd / 10;
	else
		tug->veh_slow.max_fwd_spd = tug->veh.max_fwd_spd / 30;
	tug->veh_slow.max_rev_spd = tug->veh.max_rev_spd / 10;
	tug->veh_slow.max_accel = tug->info->max_accel / 3;
	tug->veh_slow.max_decel = tug->info->max_decel / 3;

	objpath = tug_liv_apply(tug->info);
	if (objpath == NULL) {
		logMsg("Error preparing tug object %s", tug->info->tug);
		goto errout;
	}
	tug->tug = XPLMLoadObject(objpath);
	tug_liv_destroy(objpath);

	if (tug->tug == NULL) {
		logMsg("Error loading tug object %s", tug->info->tug);
		goto errout;
	}

#define	LOAD_TUG_SOUND(sound) \
	do { \
		tug->sound = wav_load(tug->info->sound, #sound); \
		if (tug->sound == NULL) { \
			logMsg("Error loading tug sound %s", \
			    tug->info->sound); \
			goto errout; \
		} \
		wav_set_loop(tug->sound, B_TRUE); \
	} while (0)

	LOAD_TUG_SOUND(engine_snd);
	LOAD_TUG_SOUND(engine_snd_in);

	if (tug->info->air_snd != NULL) {
		LOAD_TUG_SOUND(air_snd);
		LOAD_TUG_SOUND(air_snd_in);
	}
	if (tug->info->beeper_snd != NULL) {
		LOAD_TUG_SOUND(beeper_snd);
		LOAD_TUG_SOUND(beeper_snd_in);
	}

	fdr_find(&tug->cam_x, "sim/graphics/view/view_x");
	fdr_find(&tug->cam_y, "sim/graphics/view/view_y");
	fdr_find(&tug->cam_z, "sim/graphics/view/view_z");
	fdr_find(&tug->cam_hdg, "sim/graphics/view/view_heading");
	fdr_find(&tug->cam_is_ext, "sim/graphics/view/view_is_external");
	fdr_find(&tug->sound_on, "sim/operation/sound/sound_on");
	if (bp_xp_ver >= 11000) {
		fdr_find(&tug->ext_vol,
		    "sim/operation/sound/exterior_volume_ratio");
		fdr_find(&tug->int_vol,
		    "sim/operation/sound/interior_volume_ratio");
	} else {
		fdr_find(&tug->ext_vol,
		    "sim/operation/sound/engine_volume_ratio");
		fdr_find(&tug->int_vol,
		    "sim/operation/sound/engine_volume_ratio");
	}

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
	/* Felis Tu-154M integration */
	} else if (dr_find(&tug->cockpit_window_drs[0],
		"sim/custom/anim/cockpit_window_left")) {
		fdr_find(&tug->cockpit_window_drs[1],
		    "sim/custom/anim/cockpit_window_right");
		tug->num_cockpit_window_drs = 2;
	}

	/*
	 * Initial state is:
	 * 1) everything at 0, except:
	 * 2) lift fully raised
	 * 3) lift arms fully open on grab-type lift tugs
	 */
	for (anim_t a = 0; a < TUG_NUM_ANIMS; a++)
		anim[a].value = 0;
	anim[ANIM_LIFT].value = 1;
	anim[ANIM_LIFT_ARM].value = 1;
	cradle_lights_req = B_FALSE;

	glob_tug = tug;

	return (tug);
errout:
	tug_free(tug);
	return (NULL);
}

tug_t *
tug_alloc_man(const char *tug_name, double tirrad, const char *arpt,
    const char *airline)
{
	char *tug_path = mkpathname(bp_xpdir, bp_plugindir, "objects", "tugs",
	    tug_name, NULL);
	tug_info_t *ti = tug_info_read(tug_path, tug_name, arpt, airline);

	free(tug_path);
	if (ti == NULL)
		return (NULL);

	return (tug_alloc_common(ti, tirrad));
}

tug_t *
tug_alloc_auto(double mtow, double ng_len, double tirrad, unsigned gear_type,
    const char *arpt, const char *airline)
{
	/* Auto-select a matching tug from our repertoire */
	char *reason = NULL;
	tug_info_t *ti = tug_info_select(mtow, ng_len, tirrad, gear_type, arpt,
	    airline, &reason);
	if (ti == NULL) {
		logMsg("Failed to find a tug for you, reason:\n%s", reason);
		free(reason);
		return (NULL);
	}
	free(reason);
	return (tug_alloc_common(ti, tirrad));
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

#define	WAV_STOP_AND_DESTROY(sound) \
	do { \
		if (tug->sound != NULL) { \
			wav_stop(tug->sound); \
			wav_free(tug->sound); \
			tug->sound = NULL; \
		} \
	} while (0)
	WAV_STOP_AND_DESTROY(engine_snd);
	WAV_STOP_AND_DESTROY(engine_snd_in);
	WAV_STOP_AND_DESTROY(air_snd);
	WAV_STOP_AND_DESTROY(air_snd_in);
	WAV_STOP_AND_DESTROY(beeper_snd);
	WAV_STOP_AND_DESTROY(beeper_snd_in);

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
		    &tug->segs, &tug->last_mis_hdg, d_t, &steer, &speed, NULL);
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

	if (!tug->TE_override) {
		/*
		 * In real vehicles gears are not evenly spaced, but since
		 * we don't really drive like a real car and to
		 * exaggerate the shifting effect, we simply space the gears.
		 * evenly.
		 */
		int n_gears = (tug->pos.spd >= 0 ? tug->info->num_fwd_gears :
		    tug->info->num_rev_gears);
		int gear = n_gears;
		double max_spd = (tug->pos.spd >= 0 ? tug->info->max_fwd_speed :
		    tug->info->max_rev_speed);
		double spd = ABS(tug->pos.spd);
		double TE = 0;

		/*
		 * Select the lowest applicable gear by simply going over the
		 * gears from highest to lowest. The lowest gear is simply the
		 * last one where our actual speed is below maximum speed for
		 * that gear ratio.
		 */
		while (gear > 0) {
			double max_gear_spd = max_spd / ((n_gears + 1) - gear);
			double min_gear_spd = max_gear_spd / 5;

			if (spd > max_gear_spd)
				break;
			TE = MAX((spd - min_gear_spd) / (max_gear_spd -
			    min_gear_spd), 0);
			gear--;
		}
		/*
		 * Reduce the TE pitch by a factor of 3 so we don't quite
		 * whine at our maximum rpm at max speed.
		 */
		tug_set_TE_snd(tug, TE / 3, d_t);
	}
}

void
tug_anim(tug_t *tug, double d_t, double cur_t)
{
	uint64_t cur_t_ms = cur_t * 1000;

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
			anim[ANIM_CAB_POSITION].value = MIN(1,
			    anim[ANIM_CAB_POSITION].value +
			    d_t / CAB_LIFT_TIME);
		} else if (tug->pos.spd > 0.1) {
			anim[ANIM_REVERSE_LIGHTS].value = B_FALSE;
			anim[ANIM_DRIVER_ORIENTATION].value = MAX(0,
			    anim[ANIM_DRIVER_ORIENTATION].value -
			    d_t / DRIVER_TURN_TIME);
			anim[ANIM_CAB_POSITION].value = MAX(0,
			    anim[ANIM_CAB_POSITION].value -
			    d_t / CAB_LIFT_TIME);
		}
		anim[ANIM_CRADLE_LIGHTS].value = (cradle_lights_req &&
		    dr_getf(&sun_pitch_dr) < LIGHTS_ON_SUN_ANGLE);
	} else {
		float value;
		int64_t mt = microclock();

		value = (mt % 3000000) / 3000000.0;
		anim[ANIM_FRONT_DRIVE].value = value;
		anim[ANIM_FRONT_STEER].value = value;
		anim[ANIM_REAR_DRIVE].value = value;
		anim[ANIM_LIFT].value = value;
		anim[ANIM_LIFT_ARM].value = value;
		anim[ANIM_TIRE_SENSE].value = value;
		anim[ANIM_DRIVER_ORIENTATION].value = value;
		anim[ANIM_CAB_POSITION].value = value;

		value = (mt / 1000000) % 2;
		anim[ANIM_VEHICLE_LIGHTS].value = value;
		anim[ANIM_CRADLE_LIGHTS].value = value;
		anim[ANIM_REVERSE_LIGHTS].value = value;
		anim[ANIM_HAZARD_LIGHTS].value = value;
		anim[ANIM_WINCH_ON].value = value;
		anim[ANIM_CLEAR_SIGNAL].value = ((mt / 1000000) % 3) - 1;
		anim[ANIM_LIFT_IN_TRANSIT].value = value;
	}

	anim[ANIM_BEACON_FLASH].value = ((cur_t_ms % BEACON_FLASH_INTVAL) <=
	    BEACON_FLASH_DUR && anim[ANIM_HAZARD_LIGHTS].value);
}

void
tug_draw(tug_t *tug, double cur_t)
{
	XPLMDrawInfo_t di;
	XPLMProbeRef probe = XPLMCreateProbe(xplm_ProbeY);
	XPLMProbeInfo_t info = { .structSize = sizeof (XPLMProbeInfo_t) };
	vect3_t pos, norm, norm_hdg;
	double gain_in, gain_out;

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
		wav_play(tug->engine_snd_in);
		tug->engine_snd_playing = B_TRUE;
	}

	if (dr_geti(&tug->sound_on) == 1) {
		vect3_t cam_pos_xp = VECT3(dr_getf(&tug->cam_x),
		    dr_getf(&tug->cam_y), dr_getf(&tug->cam_z));
		double cam_dist = vect3_abs(vect3_sub(cam_pos_xp, pos));
		double window = 0;

		/* Camera distance modifier */
		cam_dist = MAX(cam_dist, 1);
		gain_in = POW2((2 * tug->veh.wheelbase) / cam_dist);
		gain_in = MIN(gain_in, 1.0);

		if (tug->info->electric_drive) {
			double x;
			if (tug->TE_override) {
				x = tug->last_TE_fract;
			} else {
				if (tug->pos.spd == 0) {
					x = 0;
				} else {
					x = (ABS(tug->pos.spd) /
					    tug->info->max_fwd_speed) + 0.2;
				}
			}
			gain_in *= x;
		}

		/*
		 * Up to this point both internal and external noises behave
		 * the same from a volume perspective.
		 */
		gain_out = gain_in;

		gain_out *= dr_getf(&tug->ext_vol);
		gain_in *= dr_getf(&tug->int_vol) * VOLUME_INSIDE_MODIFIER;

		/* Cockpit-window-open modifier */
		for (unsigned i = 0; i < tug->num_cockpit_window_drs; i++) {
			window = MAX(dr_getf(&tug->cockpit_window_drs[i]),
			    window);
		}

		/* We're only interested in the first 1/4 of the window range */
		window = MIN(1, window * 4);
		if (dr_geti(&tug->cam_is_ext) == 0) {
			gain_in *= wavg(1, 0, window);
			gain_out *= wavg(0, 1, window);
		} else {
			gain_in = 0;
		}

		/* Cutoff for minimum noise intensity */
		if (gain_in < 0.001)
			gain_in = 0;
		if (gain_out < 0.001)
			gain_out = 0;
	} else {
		gain_in = 0;
		gain_out = 0;
	}

	wav_set_gain(tug->engine_snd, gain_out);
	wav_set_gain(tug->engine_snd_in, gain_in);

	if (tug->beeper_snd != NULL) {
		wav_set_gain(tug->beeper_snd, gain_out);
		wav_set_gain(tug->beeper_snd_in, gain_in);
	}

	if (tug->air_snd != NULL) {
		if (tug->cradle_air_on) {
			double ramp = MIN((cur_t - tug->cradle_air_chg_t) /
			    TUG_CRADLE_CHG_D_T, 1);
			wav_set_gain(tug->air_snd, gain_out * ramp *
			    TUG_CRADLE_AIR_MOD);
			wav_set_gain(tug->air_snd_in, gain_in * ramp *
			    TUG_CRADLE_AIR_MOD);
			if (!tug->cradle_air_snd_on) {
				wav_play(tug->air_snd);
				wav_play(tug->air_snd_in);
				tug->cradle_air_snd_on = B_TRUE;
			}
		} else if (cur_t - tug->cradle_air_chg_t <=
		    TUG_CRADLE_CHG_D_T) {
			double ramp = 1 - ((cur_t - tug->cradle_air_chg_t) /
			    TUG_CRADLE_CHG_D_T);
			wav_set_gain(tug->air_snd, gain_out * ramp *
			    TUG_CRADLE_AIR_MOD);
			wav_set_gain(tug->air_snd_in, gain_in * ramp *
			    TUG_CRADLE_AIR_MOD);
		} else if (tug->cradle_air_snd_on) {
			wav_stop(tug->air_snd);
			wav_stop(tug->air_snd_in);
			tug->cradle_air_snd_on = B_FALSE;
		}
	}

	XPLMDestroyProbe(probe);
}

void
tug_set_TE_override(tug_t *tug, bool_t flag)
{
	tug->TE_override = flag;
}

void
tug_set_TE_snd(tug_t *tug, double TE_fract, double d_t)
{
	double d_TE_fract;

	/*
	 * Modulate the engine note change so that it takes 1 second to
	 * ramp up from idle to max power.
	 */
	TE_fract = MIN(MAX(TE_fract, 0), 1);
	d_TE_fract = TE_fract - tug->last_TE_fract;
	if (tug->TE_override) {
		d_TE_fract = MIN(d_TE_fract, d_t / TUG_TE_RAMP_UP_DELAY);
		d_TE_fract = MAX(d_TE_fract, -d_t / TUG_TE_RAMP_UP_DELAY);
	} else {
		d_TE_fract = MIN(d_TE_fract, d_t / TUG_TE_FAST_RAMP_UP_DELAY);
		d_TE_fract = MAX(d_TE_fract, -d_t / TUG_TE_FAST_RAMP_UP_DELAY);
	}
	tug->last_TE_fract += d_TE_fract;

	wav_set_pitch(tug->engine_snd, 0.5 + tug->last_TE_fract);
	wav_set_pitch(tug->engine_snd_in, 0.5 + tug->last_TE_fract);
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
	if (tug->beeper_snd != NULL) {
		if (flag && !tug->cradle_beeper_snd_on) {
			wav_play(tug->beeper_snd);
			wav_play(tug->beeper_snd_in);
		}
		else if (!flag && tug->cradle_beeper_snd_on) {
			wav_stop(tug->beeper_snd);
			wav_stop(tug->beeper_snd_in);
		}
	}
	tug->cradle_beeper_snd_on = flag;
}

void
tug_set_steering(tug_t *tug, double req_steer, double d_t)
{
	double d_steer = req_steer - tug->cur_steer;
	double max_d_steer = TUG_FAST_STEER_RATE * d_t;
	d_steer = MIN(d_steer, max_d_steer);
	d_steer = MAX(d_steer, -max_d_steer);
	if (tug->cur_steer + d_steer > tug->info->max_steer)
		tug->cur_steer = tug->info->max_steer;
	else if (tug->cur_steer + d_steer < -tug->info->max_steer)
		tug->cur_steer = -tug->info->max_steer;
	else
		tug->cur_steer += d_steer;
	tug->steer_override = (list_head(&tug->segs) == NULL);
}

bool_t
tug_is_stopped(const tug_t *tug)
{
	return (list_head(&tug->segs) == NULL && tug->pos.spd == 0);
}

double
tug_plat_h(const tug_t *tug)
{
	if (tug->info->lift_type != LIFT_WINCH)
		return (0);
	return ((1 - (tug->tirrad / (tug->info->lift_wall_z -
	    tug->info->plat_z))) * tug->info->plat_h);
}

double
tug_lift_wall_off(const tug_t *tug)
{
	switch (tug->info->lift_wall_loc) {
	case LIFT_WALL_FRONT:
		return (tug->tirrad);
	case LIFT_WALL_CENTER:
		return (0);
	default:
		ASSERT3U(tug->info->lift_wall_loc, ==, LIFT_WALL_BACK);
		return (-tug->tirrad);
	}
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
		double x = (tug->tirrad - ti->min_tirrad) / (ti->max_tirrad -
		    ti->min_tirrad);
		x = MIN(MAX(x, 0), 1);
		min_val = wavg(0, ti->max_tirrad_f, x);
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

void
tug_set_winch_on(tug_t *tug, bool_t flag)
{
	UNUSED(tug);
	anim[ANIM_WINCH_ON].value = flag;
}

void
tug_set_clear_signal(bool_t on, bool_t right)
{
	if (on)
		anim[ANIM_CLEAR_SIGNAL].value = (right ? 1 : -1);
	else
		anim[ANIM_CLEAR_SIGNAL].value = 0;
}

void
tug_set_lift_in_transit(bool_t flag)
{
	anim[ANIM_LIFT_IN_TRANSIT].value = flag;
}
