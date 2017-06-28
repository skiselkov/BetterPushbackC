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

#include <string.h>
#include <stddef.h>

#include <XPLMPlugin.h>

#include <acfutils/assert.h>
#include <acfutils/helpers.h>
#include <acfutils/math.h>

#include "driving.h"
#include "truck.h"
#include "xplane.h"

#define	TRUCK_HEIGHT		0
#define	TRUCK_WHEELBASE		5
#define	TRUCK_FIXED_OFFSET	2.5
#define	TRUCK_MAX_STEER		60
#define	TRUCK_OBJ		("objects" DIRSEP_S "White.obj")
#define	TRUCK_ENGINE_SND	("data" DIRSEP_S "truck_engine.wav")
#define	TRUCK_AIR_SND		("data" DIRSEP_S "truck_air.wav")
#define	TRUCK_BEEPER_SND	("data" DIRSEP_S "truck_beeper.wav")
#define	TRUCK_STEER_RATE	40	/* deg/s */

#define	TRUCK_MAX_SPD		10	/* m/s */
#define	TRUCK_MAX_ANG_VEL	20	/* deg/s */
#define	TRUCK_MAX_ACCEL		1	/* m/s^2 */
#define	TRUCK_MAX_DECEL		0.5	/* m/s^2 */

#define	TRUCK_SND_MAX_DIST	15	/* meters */
#define	VOLUME_INSIDE_MODIFIER	0.2

#define	TRUCK_CRADLE_CHG_D_T	0.5
#define	TRUCK_CRADLE_AIR_MOD	0.5

void
truck_create(truck_t *truck, vect2_t pos, double hdg)
{
	char *truckpath, *sndpath;

	memset(truck, 0, sizeof (*truck));

	truck->pos.pos = pos;
	truck->pos.hdg = hdg;

	truck->veh.wheelbase = TRUCK_WHEELBASE;
	truck->veh.max_steer = TRUCK_MAX_STEER;
	truck->veh.max_fwd_spd = TRUCK_MAX_SPD;
	truck->veh.max_rev_spd = TRUCK_MAX_SPD;
	truck->veh.max_ang_vel = TRUCK_MAX_ANG_VEL;
	truck->veh.max_accel = TRUCK_MAX_ACCEL;
	truck->veh.max_decel = TRUCK_MAX_DECEL;

	truckpath = mkpathname(bp_plugindir, TRUCK_OBJ, NULL);
	truck->obj = XPLMLoadObject(truckpath);
	VERIFY(truck->obj != NULL);
	free(truckpath);

	sndpath = mkpathname(bp_xpdir, bp_plugindir, TRUCK_ENGINE_SND, NULL);
	truck->engine_snd = wav_load(sndpath, "truck_engine");
	VERIFY(truck->engine_snd != NULL);
	free(sndpath);

	sndpath = mkpathname(bp_xpdir, bp_plugindir, TRUCK_AIR_SND, NULL);
	truck->air_snd = wav_load(sndpath, "truck_air");
	VERIFY(truck->air_snd != NULL);
	free(sndpath);

	sndpath = mkpathname(bp_xpdir, bp_plugindir, TRUCK_BEEPER_SND, NULL);
	truck->beeper_snd = wav_load(sndpath, "truck_beeper");
	VERIFY(truck->beeper_snd != NULL);
	free(sndpath);

	wav_set_loop(truck->engine_snd, B_TRUE);
	wav_set_loop(truck->air_snd, B_TRUE);
	wav_set_loop(truck->beeper_snd, B_TRUE);

	dr_init(&truck->cam_x, "sim/graphics/view/view_x");
	dr_init(&truck->cam_y, "sim/graphics/view/view_y");
	dr_init(&truck->cam_z, "sim/graphics/view/view_z");
	dr_init(&truck->cam_hdg, "sim/graphics/view/view_heading");
	dr_init(&truck->cam_is_ext, "sim/graphics/view/view_is_external");
	dr_init(&truck->sound_on, "sim/operation/sound/sound_on");
	dr_init(&truck->ext_vol, "sim/operation/sound/exterior_volume_ratio");

	/* Flight Factor A320 integration */
	if (XPLMFindDataRef("model/controls/window_l") != NULL) {
		truck->num_cockpit_window_drs = 2;
		truck->cockpit_window_drs = calloc(2, sizeof (dr_t));
		dr_init(&truck->cockpit_window_drs[0],
		    "model/controls/window_l");
		dr_init(&truck->cockpit_window_drs[1],
		    "model/controls/window_r");
	/* IXEG 737 integration */
	} else if (XPLMFindDataRef("ixeg/733/misc/pilot_window") != NULL) {
		truck->num_cockpit_window_drs = 2;
		truck->cockpit_window_drs = calloc(2, sizeof (dr_t));
		dr_init(&truck->cockpit_window_drs[0],
		    "ixeg/733/misc/pilot_window");
		dr_init(&truck->cockpit_window_drs[1],
		    "ixeg/733/misc/copilot_window");
	}

	list_create(&truck->segs, sizeof (seg_t), offsetof(seg_t, node));
}

void
truck_destroy(truck_t *truck)
{
	seg_t *seg;

	XPLMUnloadObject(truck->obj);
	while ((seg = list_head(&truck->segs)) != NULL) {
		list_remove_head(&truck->segs);
		free(seg);
	}
	list_destroy(&truck->segs);
	free(truck->cockpit_window_drs);

	wav_stop(truck->engine_snd);
	wav_stop(truck->air_snd);
	wav_stop(truck->beeper_snd);

	wav_free(truck->engine_snd);
	wav_free(truck->air_snd);
	wav_free(truck->beeper_snd);

	memset(truck, 0, sizeof (*truck));
}

bool_t
truck_drive2point(truck_t *truck, vect2_t dst, double hdg)
{
	vect2_t cur_pos;
	double cur_hdg;
	seg_t *seg;

	seg = list_tail(&truck->segs);
	if (seg != NULL) {
		cur_pos = seg->end_pos;
		cur_hdg = seg->end_hdg;
	} else {
		cur_pos = truck->pos.pos;
		cur_hdg = truck->pos.hdg;
	}

	return (compute_segs(&truck->veh, cur_pos, cur_hdg, dst, hdg,
	    &truck->segs) >= 0);
}

void
truck_run(truck_t *truck, double d_t)
{
	double steer = 0, speed = 0;
	double accel, turn, radius, d_hdg_rad;
	vect2_t pos_incr;

	if (!truck->engine_snd_playing) {
		wav_play(truck->engine_snd);
		truck->engine_snd_playing = B_TRUE;
	}

	if (list_head(&truck->segs) != NULL) {
		(void) drive_segs(&truck->pos, &truck->veh, &truck->segs,
		    &truck->last_mis_hdg, d_t, &steer, &speed);
	} else if (truck->pos.spd == 0) {
		return;
	}

	if (speed >= truck->pos.spd)
		accel = MIN(speed - truck->pos.spd, TRUCK_MAX_ACCEL * d_t);
	else
		accel = MAX(speed - truck->pos.spd, -TRUCK_MAX_ACCEL * d_t);

	if (steer >= truck->cur_steer)
		turn = MIN(steer - truck->cur_steer, TRUCK_STEER_RATE * d_t);
	else
		turn = MAX(steer - truck->cur_steer, -TRUCK_STEER_RATE * d_t);

	truck->pos.spd += accel;
	truck->cur_steer += turn;

	radius = tan(DEG2RAD(90 - truck->cur_steer)) * truck->veh.wheelbase;
	if (radius < 1e6)
		d_hdg_rad = (truck->pos.spd / radius) * d_t;
	else
		d_hdg_rad = 0;
	pos_incr = VECT2(sin(d_hdg_rad) * truck->pos.spd * d_t,
	    cos(d_hdg_rad) * truck->pos.spd * d_t);
	truck->pos.pos = vect2_add(truck->pos.pos,
	    vect2_rot(pos_incr, truck->pos.hdg));
	truck->pos.hdg += RAD2DEG(d_hdg_rad);
	truck->pos.hdg = normalize_hdg(truck->pos.hdg);

	truck_set_TE_snd(truck, (ABS(truck->pos.spd) / TRUCK_MAX_SPD) / 2);
}

void
truck_draw(truck_t *truck, double cur_t)
{
	XPLMDrawInfo_t di;
	XPLMProbeRef probe = XPLMCreateProbe(xplm_ProbeY);
	XPLMProbeInfo_t info = { .structSize = sizeof (XPLMProbeInfo_t) };
	vect3_t pos, norm;
	vect2_t v;
	double gain;

	/* X-Plane's Z axis is inverted to ours */
	VERIFY3U(XPLMProbeTerrainXYZ(probe, truck->pos.pos.x, 0,
	    -truck->pos.pos.y, &info), ==, xplm_ProbeHitTerrain);

	pos = VECT3(truck->pos.pos.x, info.locationY, -truck->pos.pos.y);
	norm = VECT3(info.normalX, info.normalY, info.normalZ);
	pos = vect3_add(pos, vect3_set_abs(norm, TRUCK_HEIGHT));

	v = VECT2(norm.x, -norm.z);
	v = vect2_rot(v, truck->pos.hdg);

	di.structSize = sizeof (di);
	di.x = pos.x;
	di.y = pos.y;
	di.z = pos.z;
	di.heading = truck->pos.hdg;
	di.roll = -RAD2DEG(asin(v.x / norm.y));
	di.pitch = -RAD2DEG(asin(v.y / norm.y));

	XPLMDrawObjects(truck->obj, 1, &di, 1, 1);

	if (dr_geti(&truck->sound_on) == 1) {
		vect3_t cam_pos_xp = VECT3(dr_getf(&truck->cam_x),
		    dr_getf(&truck->cam_y), dr_getf(&truck->cam_z));
		double cam_dist = vect3_abs(vect3_sub(cam_pos_xp, pos));
		double window = 0;

		/* Camera distance modifier */
		cam_dist = MAX(cam_dist, 1);
		gain = POW2(TRUCK_SND_MAX_DIST / cam_dist);
		gain = MIN(gain, 1.0);
		gain *= dr_getf(&truck->ext_vol);

		/* Cockpit-window-open modifier */
		for (unsigned i = 0; i < truck->num_cockpit_window_drs; i++) {
			window = MAX(dr_getf(&truck->cockpit_window_drs[i]),
			    window);
		}
		/* We're only interested in the first 1/4 of the window range */
		window = MIN(1, window * 4);
		if (dr_geti(&truck->cam_is_ext) == 0)
			gain *= wavg(VOLUME_INSIDE_MODIFIER, 1, window);
		if (gain < 0.001)
			gain = 0;
	} else {
		gain = 0;
	}

	wav_set_gain(truck->engine_snd, gain);

	wav_set_gain(truck->beeper_snd, gain);

	if (truck->cradle_air_on) {
		double ramp = MIN((cur_t - truck->cradle_air_chg_t) /
		    TRUCK_CRADLE_CHG_D_T, 1);
		wav_set_gain(truck->air_snd, gain * ramp *
		    TRUCK_CRADLE_AIR_MOD);
		if (!truck->cradle_air_snd_on) {
			wav_play(truck->air_snd);
			truck->cradle_air_snd_on = B_TRUE;
		}
	} else if (cur_t - truck->cradle_air_chg_t <= TRUCK_CRADLE_CHG_D_T) {
		double ramp = 1 - ((cur_t - truck->cradle_air_chg_t) /
		    TRUCK_CRADLE_CHG_D_T);
		wav_set_gain(truck->air_snd, gain * ramp *
		    TRUCK_CRADLE_AIR_MOD);
	} else if (truck->cradle_air_snd_on) {
		wav_stop(truck->air_snd);
		truck->cradle_air_snd_on = B_FALSE;
	}
}

void
truck_set_TE_snd(truck_t *truck, double TE_fract)
{
	/*
	 * We do a double log of the linear TE fraction to obtain a more
	 * obvious engine note of the engine being under load.
	 */
	for (int i = 0; i < 2; i++)
		TE_fract = log((TE_fract * (M_E - 1)) + 1);
	wav_set_pitch(truck->engine_snd, 0.5 + TE_fract);
}

void
truck_set_cradle_air_on(truck_t *truck, bool_t flag, double cur_t)
{
	if (truck->cradle_air_on == flag)
		return;
	truck->cradle_air_on = flag;
	truck->cradle_air_chg_t = cur_t;
}

void
truck_set_cradle_beeper_on(truck_t *truck, bool_t flag)
{
	if (flag && !truck->cradle_beeper_snd_on)
		wav_play(truck->beeper_snd);
	else if (!flag && truck->cradle_beeper_snd_on)
		wav_stop(truck->beeper_snd);
	truck->cradle_beeper_snd_on = flag;
}

bool_t
truck_is_stopped(const truck_t *truck)
{
	return (list_head(&truck->segs) == NULL && truck->pos.spd == 0);
}
