/*
 * CDDL HEADER START
 *
 * The contents of this file are subject to the terms of the
 * Common Development and Distribution License, Version 1.0 only
 * (the "License").  You may not use this file except in compliance
 * with the License.
 *
 * You can obtain a copy of the license in the file COPYING
 * or http://www.opensource.org/licenses/CDDL-1.0.
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
 * Copyright 2022 Saso Kiselkov. All rights reserved.
 */

#include <string.h>
#include <stddef.h>
#include <errno.h>

#include <png.h>

#include <XPLMCamera.h>
#include <XPLMGraphics.h>
#include <XPLMInstance.h>
#include <XPLMNavigation.h>
#include <XPLMScenery.h>
#include <XPLMUtilities.h>
#include <XPLMPlanes.h>
#include <XPLMProcessing.h>

#include <acfutils/assert.h>
#include <acfutils/dr.h>
#include <acfutils/geom.h>
#include <acfutils/glew.h>
#include <acfutils/intl.h>
#include <acfutils/math.h>
#include <acfutils/list.h>
#include <acfutils/time.h>
#include <acfutils/wav.h>

#include <cglm/cglm.h>

#include "bp.h"
#include "bp_cam.h"
#include "driving.h"
#include "xplane.h"

#define	MAX_PRED_DISTANCE	10000	/* meters */
#define	ANGLE_DRAW_STEP		5
#define	ORIENTATION_LINE_LEN	200

#define	INCR_SMALL		5
#define	INCR_MED		25
#define	INCR_BIG		125

#define	AMBER_TUPLE		VECT3(0.9, 0.9, 0)	/* RGB color */
#define	RED_TUPLE		VECT3(1, 0, 0)		/* RGB color */
#define	GREEN_TUPLE		VECT3(0, 1, 0)		/* RGB color */

#define	CLICK_DISPL_THRESH	5			/* pixels */
#define	US_PER_CLICK_ACCEL	60000			/* microseconds */
#define	US_PER_CLICK_DEACCEL	120000			/* microseconds */
#define	MAX_ACCEL_MULT		10
#define	WHEEL_ANGLE_MULT	0.5

#define	MIN_BUTTON_SCALE	0.5

#define	BP_PLANNER_VISIBILITY	40000	/* meters */

#define	PREDICTION_DRAWING_PHASE	xplm_Phase_Window
#define	PREDICTION_DRAWING_PHASE_BEFORE	1

static vect3_t		cam_pos;
static double		cam_height;
static double		cam_hdg;
static double		cursor_hdg;
static list_t		pred_segs;
static XPLMCommandRef	circle_view_cmd;
static XPLMWindowID	fake_win;
static vect2_t		cursor_world_pos;
static bool_t		force_root_win_focus = B_TRUE;
static float		saved_visibility;
static int		saved_cloud_types[3];
static bool_t		saved_real_wx;
static XPLMObjectRef	cam_lamp_obj = NULL;
static XPLMInstanceRef	cam_lamp_inst = NULL;
static const char	*cam_lamp_drefs[] = { NULL };

static int key_sniffer(char inChar, XPLMKeyFlags inFlags, char inVirtualKey,
    void *refcon);

static struct {
	dr_t	lat, lon;
	dr_t	local_x, local_y, local_z;
	dr_t	local_vx, local_vy, local_vz;
	dr_t	cam_x, cam_y, cam_z;
	dr_t	hdg;
	dr_t	mtow;
	dr_t	tire_x, tire_z;
	dr_t	tirrad;
	dr_t	view_is_ext;
	dr_t	visibility;
	dr_t	cloud_types[3];
	dr_t	use_real_wx;
	dr_t	proj_matrix_3d;
	dr_t	viewport;
} drs;

static view_cmd_info_t view_cmds[] = {
    VCI_POS("sim/general/left", -INCR_MED, 0, 0),
    VCI_POS("sim/general/right", INCR_MED, 0, 0),
    VCI_POS("sim/general/up", 0, 0, INCR_MED),
    VCI_POS("sim/general/down", 0, 0, -INCR_MED),
    VCI_POS("sim/general/forward", 0, -INCR_MED, 0),
    VCI_POS("sim/general/backward", 0, INCR_MED, 0),
    VCI_POS("sim/general/zoom_in", 0, -INCR_MED, 0),
    VCI_POS("sim/general/zoom_out", 0, INCR_MED, 0),
    VCI_POS("sim/general/hat_switch_left", -INCR_MED, 0, 0),
    VCI_POS("sim/general/hat_switch_right", INCR_MED, 0, 0),
    VCI_POS("sim/general/hat_switch_up", 0, 0, INCR_MED),
    VCI_POS("sim/general/hat_switch_down", 0, 0, -INCR_MED),
    VCI_POS("sim/general/hat_switch_up_left", -INCR_MED, 0, INCR_MED),
    VCI_POS("sim/general/hat_switch_up_right", INCR_MED, 0, INCR_MED),
    VCI_POS("sim/general/hat_switch_down_left", -INCR_MED, 0, -INCR_MED),
    VCI_POS("sim/general/hat_switch_down_right", INCR_MED, 0, -INCR_MED),
    VCI_POS("sim/general/left_fast", -INCR_BIG, 0, 0),
    VCI_POS("sim/general/right_fast", INCR_BIG, 0, 0),
    VCI_POS("sim/general/up_fast", 0, 0, INCR_BIG),
    VCI_POS("sim/general/down_fast", 0, 0, -INCR_BIG),
    VCI_POS("sim/general/forward_fast", 0, -INCR_BIG, 0),
    VCI_POS("sim/general/backward_fast", 0, INCR_BIG, 0),
    VCI_POS("sim/general/zoom_in_fast", 0, -INCR_BIG, 0),
    VCI_POS("sim/general/zoom_out_fast", 0, INCR_BIG, 0),
    VCI_POS("sim/general/left_slow", -INCR_SMALL, 0, 0),
    VCI_POS("sim/general/right_slow", INCR_SMALL, 0, 0),
    VCI_POS("sim/general/up_slow", 0, 0, INCR_SMALL),
    VCI_POS("sim/general/down_slow", 0, 0, -INCR_SMALL),
    VCI_POS("sim/general/forward_slow", 0, -INCR_SMALL, 0),
    VCI_POS("sim/general/backward_slow", 0, INCR_SMALL, 0),
    VCI_POS("sim/general/zoom_in_slow", 0, -INCR_SMALL, 0),
    VCI_POS("sim/general/zoom_out_slow", 0, INCR_SMALL, 0),
    { .name = NULL }
};

static button_t buttons[] = {
    { .filename = "move_view.png", .vk = -1, .tex = 0, .tex_data = NULL },
    { .filename = "place_seg.png", .vk = -1, .tex = 0, .tex_data = NULL },
    { .filename = "rotate_seg.png", .vk = -1, .tex = 0, .tex_data = NULL },
    { .filename = "", .vk = -1, .tex = 0, .tex_data = NULL, .h = 64 },
    {
	.filename = "accept_plan.png", .vk = XPLM_VK_RETURN, .tex = 0,
	.tex_data = NULL
    },
    {
	.filename = "delete_seg.png", .vk = XPLM_VK_DELETE, .tex = 0,
	.tex_data = NULL
    },
    { .filename = "", .vk = -1, .tex = 0, .tex_data = NULL, .h = 64 },
    {
	.filename = "cancel_plan.png", .vk = XPLM_VK_ESCAPE, .tex = 0,
	.tex_data = NULL
    },
    {
	.filename = "conn_first.png", .vk = XPLM_VK_SPACE, .tex = 0,
	.tex_data = NULL
    },
    { .filename = NULL }
};
static int button_hit = -1, button_lit = -1;
static bool_t cam_inited = B_FALSE;

bool_t
load_icon(button_t *btn)
{
	char *filename;
	FILE *fp;
	size_t rowbytes;
	png_bytep *volatile rowp = NULL;
	png_structp pngp = NULL;
	png_infop infop = NULL;
	volatile bool_t res = B_TRUE;
	uint8_t header[8];

	/* try the localized version first */
	filename = mkpathname(bp_xpdir, bp_plugindir, "data", "icons",
	    bp_get_lang(), btn->filename, NULL);
	if (!file_exists(filename, NULL)) {
		/* if the localized version failed, try the English version */
		free(filename);
		filename = mkpathname(bp_xpdir, bp_plugindir, "data", "icons",
		    "en", btn->filename, NULL);
	}
	fp = fopen(filename, "rb");
	if (fp == NULL) {
		logMsg("Cannot open file %s: %s", filename, strerror(errno));
		res = B_FALSE;
		goto out;
	}
	if (fread(header, 1, sizeof (header), fp) != 8 ||
	    png_sig_cmp(header, 0, sizeof (header)) != 0) {
		logMsg("Cannot open file %s: invalid PNG header", filename);
		res = B_FALSE;
		goto out;
	}
	pngp = png_create_read_struct(PNG_LIBPNG_VER_STRING, NULL, NULL, NULL);
	VERIFY(pngp != NULL);
	infop = png_create_info_struct(pngp);
	VERIFY(infop != NULL);
	if (setjmp(png_jmpbuf(pngp))) {
		logMsg("Cannot open file %s: libpng error in init_io",
		    filename);
		res = B_FALSE;
		goto out;
	}
	png_init_io(pngp, fp);
	png_set_sig_bytes(pngp, 8);

	if (setjmp(png_jmpbuf(pngp))) {
		logMsg("Cannot open file %s: libpng read info failed",
		    filename);
		res = B_FALSE;
		goto out;
	}
	png_read_info(pngp, infop);
	btn->w = png_get_image_width(pngp, infop);
	btn->h = png_get_image_height(pngp, infop);

	if (png_get_color_type(pngp, infop) != PNG_COLOR_TYPE_RGBA) {
		logMsg("Bad icon file %s: need color type RGBA", filename);
		res = B_FALSE;
		goto out;
	}
	if (png_get_bit_depth(pngp, infop) != 8) {
		logMsg("Bad icon file %s: need 8-bit depth", filename);
		res = B_FALSE;
		goto out;
	}
	rowbytes = png_get_rowbytes(pngp, infop);

	rowp = safe_malloc(sizeof (*rowp) * btn->h);
	VERIFY(rowp != NULL);
	for (int i = 0; i < btn->h; i++) {
		rowp[i] = safe_malloc(rowbytes);
		VERIFY(rowp[i] != NULL);
	}

	if (setjmp(png_jmpbuf(pngp))) {
		logMsg("Bad icon file %s: error reading image file", filename);
		res = B_FALSE;
		goto out;
	}
	png_read_image(pngp, rowp);

	btn->tex_data = safe_malloc(btn->h * rowbytes);
	for (int i = 0; i < btn->h; i++)
		memcpy(&btn->tex_data[i * rowbytes], rowp[i], rowbytes);

	glGenTextures(1, &btn->tex);
	glBindTexture(GL_TEXTURE_2D, btn->tex);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, btn->w, btn->h, 0, GL_RGBA,
	    GL_UNSIGNED_BYTE, btn->tex_data);

out:
	if (pngp != NULL)
		png_destroy_read_struct(&pngp, &infop, NULL);
	if (rowp != NULL) {
		for (int i = 0; i < btn->h; i++)
			free(rowp[i]);
		free(rowp);
	}
	if (fp != NULL)
		fclose(fp);
	free(filename);

	return (res);
}

void
unload_icon(button_t *btn)
{
	if (btn->tex != 0) {
		glDeleteTextures(1, &btn->tex);
		btn->tex = 0;
	}
	if (btn->tex_data != NULL) {
		free(btn->tex_data);
		btn->tex_data = NULL;
	}
}

bool_t
load_buttons(void)
{
	for (int i = 0; buttons[i].filename != NULL; i++) {
		/* skip spacers */
		if (strcmp(buttons[i].filename, "") == 0)
			continue;
		if (!load_icon(&buttons[i])) {
			unload_buttons();
			return (B_FALSE);
		}
	}

	return (B_TRUE);
}

void
unload_buttons(void)
{
	for (int i = 0; buttons[i].filename != NULL; i++)
		unload_icon(&buttons[i]);
}

static int
move_camera(XPLMCommandRef cmd, XPLMCommandPhase phase, void *refcon)
{
	static uint64_t last_cmd_t = 0;
	UNUSED(cmd);

	if (phase == xplm_CommandBegin) {
		last_cmd_t = microclock();
	} else if (phase == xplm_CommandContinue) {
		uint64_t now = microclock();
		double d_t = (now - last_cmd_t) / 1000000.0;
		unsigned i = (uintptr_t)refcon;
		vect2_t v = vect2_rot(VECT2(view_cmds[i].pos.x,
		    view_cmds[i].pos.z), cam_hdg);
		cam_pos = vect3_add(cam_pos, VECT3(v.x * d_t, 0, v.y * d_t));
		cam_height += view_cmds[i].pos.y * d_t;
		last_cmd_t = now;
	}
	return (0);
}

static void
get_vp(vec4 vp)
{
	int vp_xp[4];

	ASSERT(vp != NULL);

	if (bp_xp_ver >= 12000) {
		XPLMGetScreenBoundsGlobal(&vp_xp[0], &vp_xp[3], &vp_xp[2],
		    &vp_xp[1]);
	} else {
		VERIFY3S(dr_getvi(&drs.viewport, vp_xp, 0, 4), ==, 4);
	}
	vp[0] = vp_xp[0];
	vp[1] = vp_xp[1];
	vp[2] = vp_xp[2] - vp_xp[0];
	vp[3] = vp_xp[3] - vp_xp[1];
	if (vp[2] == 0 || vp[3] == 0) {
		int scr_w, scr_h, left, top, right, bottom;
		/*
		 * Due to an outstanding X-Plane 11.50 beta bug, the viewport
		 * dataref might not be properly updated in OpenGL mode.
		 */
		XPLMGetScreenBoundsGlobal(&left, &top, &right, &bottom);
		scr_w = right;
		scr_h = top;

		vp[2] = scr_w;
		vp[3] = scr_h;
	}
}

/*
 * Un-projects a viewport coordinate at X, Y using the current projection
 * matrix and figures out to which point on the reference plane it
 * corresponds. The reference plane is a plane that is parallel with the
 * Earth's surface at the local coordinate origin and is elevation-centered
 * on the aircraft's current local Y coordinate.
 */
static void
vp_unproject(double x, double y, double *x_phys, double *y_phys)
{
	mat4 proj;
	vec4 vp;
	vec3 out_pt;

	ASSERT(x_phys != NULL);
	ASSERT(y_phys != NULL);

	VERIFY3S(dr_getvf32(&drs.proj_matrix_3d, (float *)proj, 0, 16), ==, 16);
	get_vp(vp);
	glm_unproject((vec3){x, y, 0.5}, proj, vp, out_pt);
	/*
	 * To avoid having to figure out the viewport Z coordinate that
	 * matches the reference plane distance, we scale the returned
	 * 3D coordinate based on Z-distance of the camera from the
	 * reference plane.
	 */
	ASSERT(!isnan(out_pt[0]));
	ASSERT(!isnan(out_pt[1]));
	ASSERT(out_pt[2] != 0);
	glm_vec3_scale(out_pt, ABS(cam_height / out_pt[2]), out_pt);
	ASSERT(!isnan(out_pt[0]));
	ASSERT(!isnan(out_pt[1]));
	*x_phys = out_pt[0];
	*y_phys = out_pt[1];
}

static int
cam_ctl(XPLMCameraPosition_t *pos, int losing_control, void *refcon)
{
	int x, y;
	double dx, dy, start_hdg;
	vect2_t start_pos, end_pos;
	seg_t *seg;
	int n;

	UNUSED(refcon);

	if (pos == NULL || losing_control || !cam_inited)
		return (0);

	pos->x = cam_pos.x;
	pos->y = cam_pos.y + cam_height;
	pos->z = -cam_pos.z;
	pos->pitch = -90;
	pos->heading = cam_hdg;
	pos->roll = 0;
	pos->zoom = 1;

	XPLMGetMouseLocationGlobal(&x, &y);
	vp_unproject(x, y, &dx, &dy);

	/*
	 * Don't make predictions if due to the camera FOV angle (>= 180 deg)
	 * we could be placing the prediction object very far away.
	 */
	if (dx > MAX_PRED_DISTANCE || dy > MAX_PRED_DISTANCE)
		return (1);

	while ((seg = list_remove_head(&pred_segs)) != NULL)
		free(seg);

	seg = list_tail(&bp.segs);
	if (seg != NULL) {
		start_pos = seg->end_pos;
		start_hdg = seg->end_hdg;
	} else {
		start_pos = VECT2(dr_getf(&drs.local_x),
		    -dr_getf(&drs.local_z));	/* inverted X-Plane Z */
		start_hdg = dr_getf(&drs.hdg);
	}

	end_pos = vect2_add(VECT2(cam_pos.x, cam_pos.z),
	    vect2_rot(VECT2(dx, dy), pos->heading));
	cursor_world_pos = VECT2(end_pos.x, end_pos.y);

	n = compute_segs(&bp.veh, start_pos, start_hdg, end_pos,
	    cursor_hdg, &pred_segs);
	if (n > 0) {
		seg = list_tail(&pred_segs);
		seg->user_placed = B_TRUE;
	}

	return (1);
}

static void
draw_segment(const seg_t *seg)
{
	XPLMProbeRef probe = XPLMCreateProbe(xplm_ProbeY);
	XPLMProbeInfo_t info = { .structSize = sizeof (XPLMProbeInfo_t) };
	double wing_long_off = bp.acf.main_z - bp_ls.outline->wingtip.y;
	vect2_t wing_off_l = VECT2(-bp_ls.outline->semispan, wing_long_off);
	vect2_t wing_off_r = VECT2(bp_ls.outline->semispan, wing_long_off);

	switch (seg->type) {
	case SEG_TYPE_STRAIGHT: {
		float h1, h2;
		vect2_t wing_l, wing_r, p;

		VERIFY3U(XPLMProbeTerrainXYZ(probe, seg->start_pos.x, 0,
		    -seg->start_pos.y, &info), ==, xplm_ProbeHitTerrain);
		h1 = info.locationY;
		VERIFY3U(XPLMProbeTerrainXYZ(probe, seg->end_pos.x, 0,
		    -seg->end_pos.y, &info), ==, xplm_ProbeHitTerrain);
		h2 = info.locationY;

		glColor3f(0, 0, 1);
		glLineWidth(3);
		glBegin(GL_LINES);
		glVertex3f(seg->start_pos.x, h1, -seg->start_pos.y);
		glVertex3f(seg->end_pos.x, h2, -seg->end_pos.y);
		glEnd();

		wing_l = vect2_rot(wing_off_l, seg->start_hdg);
		wing_r = vect2_rot(wing_off_r, seg->start_hdg);

		glColor3f(1, 0.25, 1);
		glLineWidth(2);
		glBegin(GL_LINES);
		p = vect2_add(seg->start_pos, wing_r);
		glVertex3f(p.x, h1, -p.y);
		p = vect2_add(seg->end_pos, wing_r);
		glVertex3f(p.x, h1, -p.y);
		p = vect2_add(seg->end_pos, wing_l);
		glVertex3f(p.x, h1, -p.y);
		p = vect2_add(seg->start_pos, wing_l);
		glVertex3f(p.x, h1, -p.y);
		glEnd();
		break;
	}
	case SEG_TYPE_TURN: {
		vect2_t c = vect2_add(seg->start_pos, vect2_scmul(
		    vect2_norm(hdg2dir(seg->start_hdg), seg->turn.right),
		    seg->turn.r));
		vect2_t c2s = vect2_sub(seg->start_pos, c);
		double s, e, rhdg;

		rhdg = rel_hdg(seg->start_hdg, seg->end_hdg);
		s = MIN(0, rhdg);
		e = MAX(0, rhdg);
		ASSERT3F(s, <=, e);
		for (double a = s; a < e; a += ANGLE_DRAW_STEP) {
			vect2_t p1, p2, p;
			vect2_t wing1_l, wing1_r, wing2_l, wing2_r;
			double step = MIN(ANGLE_DRAW_STEP, e - a);

			wing1_l = vect2_rot(wing_off_l, seg->start_hdg + a);
			wing1_r = vect2_rot(wing_off_r, seg->start_hdg + a);
			wing2_l = vect2_rot(wing_off_l,
			    seg->start_hdg + a + step);
			wing2_r = vect2_rot(wing_off_r,
			    seg->start_hdg + a + step);

			p1 = vect2_add(c, vect2_rot(c2s, a));
			p2 = vect2_add(c, vect2_rot(c2s, a + step));

			VERIFY3U(XPLMProbeTerrainXYZ(probe, p1.x, 0, -p1.y,
			    &info), ==, xplm_ProbeHitTerrain);

			glColor3f(0, 0, 1);
			glLineWidth(3);
			glBegin(GL_LINES);
			glVertex3f(p1.x, info.locationY, -p1.y);
			glVertex3f(p2.x, info.locationY, -p2.y);
			glEnd();

			glColor3f(1, 0.25, 1);
			glLineWidth(2);
			glBegin(GL_LINES);
			p = vect2_add(p1, wing1_r);
			glVertex3f(p.x, info.locationY, -p.y);
			p = vect2_add(p2, wing2_r);
			glVertex3f(p.x, info.locationY, -p.y);
			p = vect2_add(p1, wing1_l);
			glVertex3f(p.x, info.locationY, -p.y);
			p = vect2_add(p2, wing2_l);
			glVertex3f(p.x, info.locationY, -p.y);
			glEnd();
		}
		break;
	}
	}

	XPLMDestroyProbe(probe);
}

static void
draw_acf_symbol(vect3_t pos, double hdg, vect3_t color)
{
	vect2_t v;
	vect3_t p;
	double tire_x[10], tire_z[10], tirrad[10];

	glLineWidth(2);
	glColor3f(color.x, color.y, color.z);

	glBegin(GL_LINES);
	for (size_t i = 0; i + 1 < bp_ls.outline->num_pts; i++) {
		/* skip gaps in outline */
		if (IS_NULL_VECT(bp_ls.outline->pts[i]) ||
		    IS_NULL_VECT(bp_ls.outline->pts[i + 1]))
			continue;

		v = bp_ls.outline->pts[i];
		v = vect2_rot(VECT2(v.x, bp.acf.main_z - v.y), hdg);
		p = vect3_add(pos, VECT3(v.x, 0, v.y));
		glVertex3f(p.x, p.y, -p.z);

		v = bp_ls.outline->pts[i + 1];
		v = vect2_rot(VECT2(v.x, bp.acf.main_z - v.y), hdg);
		p = vect3_add(pos, VECT3(v.x, 0, v.y));
		glVertex3f(p.x, p.y, -p.z);

		v = bp_ls.outline->pts[i];
		v = vect2_rot(VECT2(-v.x, bp.acf.main_z - v.y), hdg);
		p = vect3_add(pos, VECT3(v.x, 0, v.y));
		glVertex3f(p.x, p.y, -p.z);

		v = bp_ls.outline->pts[i + 1];
		v = vect2_rot(VECT2(-v.x, bp.acf.main_z - v.y), hdg);
		p = vect3_add(pos, VECT3(v.x, 0, v.y));
		glVertex3f(p.x, p.y, -p.z);
	}
	glEnd();

	dr_getvf(&drs.tire_x, tire_x, 0, 10);
	dr_getvf(&drs.tire_z, tire_z, 0, 10);
	dr_getvf(&drs.tirrad, tirrad, 0, 10);

	glBegin(GL_QUADS);
	for (int i = 0; i < bp.acf.n_gear; i++) {
		double tr = tirrad[bp.acf.gear_is[i]];
		vect2_t c;

		v = VECT2(tire_x[bp.acf.gear_is[i]],
		    bp.acf.main_z - tire_z[bp.acf.gear_is[i]]);

		c = vect2_rot(vect2_add(v, VECT2(-tr, -tr)), hdg);
		p = vect3_add(pos, VECT3(c.x, 0, c.y));
		glVertex3f(p.x, p.y, -p.z);
		c = vect2_rot(vect2_add(v, VECT2(-tr, tr)), hdg);
		p = vect3_add(pos, VECT3(c.x, 0, c.y));
		glVertex3f(p.x, p.y, -p.z);
		c = vect2_rot(vect2_add(v, VECT2(tr, tr)), hdg);
		p = vect3_add(pos, VECT3(c.x, 0, c.y));
		glVertex3f(p.x, p.y, -p.z);
		c = vect2_rot(vect2_add(v, VECT2(tr, -tr)), hdg);
		p = vect3_add(pos, VECT3(c.x, 0, c.y));
		glVertex3f(p.x, p.y, -p.z);
	}
	glEnd();
}

static int
draw_prediction(XPLMDrawingPhase phase, int before, void *refcon)
{
	seg_t *seg;
	XPLMProbeRef probe = XPLMCreateProbe(xplm_ProbeY);
	XPLMProbeInfo_t info = { .structSize = sizeof (XPLMProbeInfo_t) };
	mat4 view, proj;
	XPLMDrawInfo_t di;
	vec3 up = {sin(DEG2RAD(cam_hdg)), 0, -cos(DEG2RAD(cam_hdg))};
	vec3 fwd = {0, -1, 0};
	vec3 cam_pos = {dr_getf(&drs.cam_x), dr_getf(&drs.cam_y),
	    dr_getf(&drs.cam_z)};

	UNUSED(phase);
	UNUSED(before);
	UNUSED(refcon);

	VERIFY3S(dr_getvf32(&drs.proj_matrix_3d, (float *)proj, 0, 16), ==, 16);
	glm_look(cam_pos, fwd, up, view);

	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glLoadMatrixf((float *)proj);

	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glLoadMatrixf((float *)view);

	if (dr_geti(&drs.view_is_ext) != 1)
		XPLMCommandOnce(circle_view_cmd);

	XPLMSetGraphicsState(0, 0, 0, 0, 0, 0, 0);

	for (seg = list_head(&bp.segs); seg != NULL;
	    seg = list_next(&bp.segs, seg))
		draw_segment(seg);

	for (seg = list_head(&pred_segs); seg != NULL;
	    seg = list_next(&pred_segs, seg))
		draw_segment(seg);

	if ((seg = list_tail(&pred_segs)) != NULL) {
		vect2_t dir_v = hdg2dir(seg->end_hdg);
		vect2_t x;

		VERIFY3U(XPLMProbeTerrainXYZ(probe, seg->end_pos.x, 0,
		    -seg->end_pos.y, &info), ==, xplm_ProbeHitTerrain);
		if (seg->type == SEG_TYPE_TURN || !seg->backward) {
			glBegin(GL_LINES);
			glColor3f(0, 1, 0);
			glVertex3f(seg->end_pos.x, info.locationY,
			    -seg->end_pos.y);
			x = vect2_add(seg->end_pos, vect2_scmul(dir_v,
			    ORIENTATION_LINE_LEN));
			glVertex3f(x.x, info.locationY, -x.y);
			glEnd();
		}
		if (seg->type == SEG_TYPE_TURN || seg->backward) {
			glBegin(GL_LINES);
			glColor3f(1, 0, 0);
			glVertex3f(seg->end_pos.x, info.locationY,
			    -seg->end_pos.y);
			x = vect2_add(seg->end_pos, vect2_neg(vect2_scmul(
			    dir_v, ORIENTATION_LINE_LEN)));
			glVertex3f(x.x, info.locationY, -x.y);
			glEnd();
		}
		draw_acf_symbol(VECT3(seg->end_pos.x, info.locationY,
		    seg->end_pos.y), seg->end_hdg, AMBER_TUPLE);
	} else {
		VERIFY3U(XPLMProbeTerrainXYZ(probe, cursor_world_pos.x, 0,
		    -cursor_world_pos.y, &info), ==, xplm_ProbeHitTerrain);
		draw_acf_symbol(VECT3(cursor_world_pos.x, info.locationY,
		    cursor_world_pos.y), cursor_hdg, RED_TUPLE);
	}

	if ((seg = list_tail(&bp.segs)) != NULL) {
		VERIFY3U(XPLMProbeTerrainXYZ(probe, seg->end_pos.x, 0,
		    -seg->end_pos.y, &info), ==, xplm_ProbeHitTerrain);
		draw_acf_symbol(VECT3(seg->end_pos.x, info.locationY,
		    seg->end_pos.y), seg->end_hdg, GREEN_TUPLE);
	}

	/* Draw the night-lighting lamp so the user can see under the cursor */
	VERIFY3U(XPLMProbeTerrainXYZ(probe, cursor_world_pos.x, 0,
	    -cursor_world_pos.y, &info), ==, xplm_ProbeHitTerrain);
	di.structSize = sizeof (di);
	di.x = cursor_world_pos.x;
	di.y = info.locationY;
	di.z = -cursor_world_pos.y;
	di.heading = 0;
	di.pitch = 0;
	di.roll = 0;
	ASSERT(cam_lamp_inst != NULL);
	XPLMInstanceSetPosition(cam_lamp_inst, &di, NULL);

	XPLMDestroyProbe(probe);

	glMatrixMode(GL_MODELVIEW);
	glPopMatrix();
	glMatrixMode(GL_PROJECTION);
	glPopMatrix();

	return (1);
}

void
draw_icon(button_t *btn, int x, int y, double scale, bool_t is_clicked,
    bool_t is_lit)
{
	glBindTexture(GL_TEXTURE_2D, btn->tex);
	glBegin(GL_QUADS);
	glTexCoord2f(0, 1);
	glVertex2f(x, y);
	glTexCoord2f(0, 0);
	glVertex2f(x, y + btn->h * scale);
	glTexCoord2f(1, 0);
	glVertex2f(x + btn->w * scale, y + btn->h * scale);
	glTexCoord2f(1, 1);
	glVertex2f(x + btn->w * scale, y);
	glEnd();

	if (is_clicked) {
		/*
		 * If this button was hit by a mouse click, highlight
		 * it by drawing a translucent white quad over it.
		 */
		XPLMSetGraphicsState(0, 0, 0, 0, 1, 0, 0);
		glColor4f(1, 1, 1, 0.3);
		glBegin(GL_QUADS);
		glVertex2f(x, y);
		glVertex2f(x, y + btn->h * scale);
		glVertex2f(x + btn->w * scale, y + btn->h * scale);
		glVertex2f(x + btn->w * scale, y);
		glEnd();
		XPLMSetGraphicsState(0, 1, 0, 0, 1, 0, 0);

	} else if (is_lit) {
		XPLMSetGraphicsState(0, 0, 0, 0, 1, 0, 0);
		glColor4f(1, 1, 1, 1);
		glLineWidth(1);
		glBegin(GL_LINES);
		glVertex2f(x, y);
		glVertex2f(x, y + btn->h * scale);
		glVertex2f(x, y + btn->h * scale);
		glVertex2f(x + btn->w * scale, y + btn->h * scale);
		glVertex2f(x + btn->w * scale, y + btn->h * scale);
		glVertex2f(x + btn->w * scale, y);
		glVertex2f(x, y);
		glEnd();
		XPLMSetGraphicsState(0, 1, 0, 0, 1, 0, 0);
	}
}

static void
fake_win_draw(XPLMWindowID inWindowID, void *inRefcon)
{
	double scale;
	int w, h, h_buttons, h_off, left, top, right, bottom;
	UNUSED(inWindowID);
	UNUSED(inRefcon);

	XPLMGetScreenBoundsGlobal(&left, &top, &right, &bottom);
	w = right;
	h = top;

	XPLMSetWindowGeometry(fake_win, 0, h, w, 0);

	if (!XPLMIsWindowInFront(fake_win))
		XPLMBringWindowToFront(fake_win);
	if (force_root_win_focus)
		XPLMTakeKeyboardFocus(0);

	XPLMSetGraphicsState(0, 1, 0, 0, 1, 0, 0);

	h_buttons = 0;
	for (int i = 0; buttons[i].filename != NULL; i++)
		h_buttons += buttons[i].h;

	scale = (double)h / h_buttons;
	scale = MIN(scale, 1);
	/* don't draw the buttons if we don't have enough space for them */
	if (scale < MIN_BUTTON_SCALE)
		return;

	h_off = (h + (h_buttons * scale)) / 2;
	for (int i = 0; buttons[i].filename != NULL;
	    i++, h_off -= buttons[i].h * scale) {
		button_t *btn = &buttons[i];

		if (btn->tex == 0)
			continue;

		draw_icon(btn, w - btn->w * scale, h_off - btn->h * scale,
		    scale, i == button_hit, i == button_lit);
	}
}

static int
button_hit_check(int x, int y)
{
	double scale;
	int w, h, h_buttons, h_off, left, top, right, bottom;

	XPLMGetScreenBoundsGlobal(&left, &top, &right, &bottom);
	w = right;
	h = top;

	h_buttons = 0;
	for (int i = 0; buttons[i].filename != NULL; i++)
		h_buttons += buttons[i].h;

	scale = (double)h / h_buttons;
	scale = MIN(scale, 1);
	if (scale < MIN_BUTTON_SCALE)
		return (-1);

	h_off = (h + (h_buttons * scale)) / 2;
	for (int i = 0; buttons[i].filename != NULL;
	    i++, h_off -= buttons[i].h * scale) {
		if (x >= w - buttons[i].w * scale && x <= w &&
		    y >= h_off - buttons[i].h * scale && y <= h_off &&
		    buttons[i].vk != -1)
			return (i);
	}

	return (-1);
}

void
nil_win_key(XPLMWindowID inWindowID, char inKey, XPLMKeyFlags inFlags,
    char inVirtualKey, void *inRefcon, int losingFocus)
{
	UNUSED(inWindowID);
	UNUSED(inKey);
	UNUSED(inFlags);
	UNUSED(inVirtualKey);
	UNUSED(inRefcon);
	UNUSED(losingFocus);
}

static XPLMCursorStatus
fake_win_cursor(XPLMWindowID inWindowID, int x, int y, void *inRefcon)
{
	int lit;

	UNUSED(inWindowID);
	UNUSED(x);
	UNUSED(y);
	UNUSED(inRefcon);

	if ((lit = button_hit_check(x, y)) != -1 &&buttons[lit].vk != -1)
		button_lit = lit;
	else
		button_lit = -1;

	return (xplm_CursorDefault);
}

static int
fake_win_click(XPLMWindowID inWindowID, int x, int y, XPLMMouseStatus inMouse,
    void *inRefcon)
{
	static int last_x = 0, last_y = 0;
	static int down_x = 0, down_y = 0;
	static bool_t dragging = B_FALSE;

	UNUSED(inWindowID);
	UNUSED(inRefcon);

	/*
	 * The mouse handling logic is as follows:
	 * 1) On mouse-down, we memorize where the click started, check if
	 *	we hit a button and reset the dragging variable.
	 * 2) While receiving dragging events, we check if the displacement
	 *	from the original start position is sufficient to consider
	 *	this click-and-drag. If it is, we set the `dragging' flag
	 *	to true. This inhibits clicking.
	 * 3) On mouse-up, if there was no click-and-drag, check if the
	 *	button we're on is still the same as what we hit on the
	 *	initial mouse-down (prevents from clicking one button and
	 *	sliding onto another one). If no button was hit and no
	 *	click-and-drag took place, count it as a click on the screen
	 *	attempting to place a segment.
	 */
	if (inMouse == xplm_MouseDown) {
		last_x = down_x = x;
		last_y = down_y = y;
		force_root_win_focus = B_FALSE;
		button_hit = button_hit_check(x, y);
		dragging = B_FALSE;
	} else if (inMouse == xplm_MouseDrag) {
		if (!dragging) {
			dragging = (ABS(x - down_x) >= CLICK_DISPL_THRESH ||
			    ABS(y - down_y) >= CLICK_DISPL_THRESH);
		}
		if (dragging && (x != last_x || y != last_y) &&
		    button_hit == -1) {
			double x_phys, last_x_phys, y_phys, last_y_phys, dx, dy;
			vect2_t v;

			vp_unproject(x, y, &x_phys, &y_phys);
			vp_unproject(last_x, last_y,
			    &last_x_phys, &last_y_phys);
			dx = x_phys - last_x_phys;
			dy = y_phys - last_y_phys;

			v = vect2_rot(VECT2(dx, dy), cam_hdg);
			cam_pos.x -= v.x;
			cam_pos.z -= v.y;
			last_x = x;
			last_y = y;
		}
	} else {
		if (!dragging) {
			if (button_hit != -1 &&
			    button_hit == button_hit_check(x, y)) {
				/* simulate a key press */
				ASSERT(buttons[button_hit].vk != -1);
				key_sniffer(0, xplm_DownFlag,
				    buttons[button_hit].vk, NULL);
			} else {
				/*
				 * Transfer whatever is in pred_segs to
				 * the normal segments and clear pred_segs.
				 */
				list_move_tail(&bp.segs, &pred_segs);
			}
		}
		button_hit = -1;
		force_root_win_focus = B_TRUE;
	}

	return (1);
}

static int
fake_win_wheel(XPLMWindowID inWindowID, int x, int y, int wheel, int clicks,
    void *inRefcon)
{
	UNUSED(inWindowID);
	UNUSED(x);
	UNUSED(y);
	UNUSED(wheel);
	UNUSED(clicks);
	UNUSED(inRefcon);

	if (wheel == 0 && clicks != 0) {
		static int accel = 1;
		static uint64_t last_wheel_t = 0;
		uint64_t now = microclock();
		uint64_t us_per_click = (now - last_wheel_t) / ABS(clicks);

		if (us_per_click < US_PER_CLICK_ACCEL)
			accel = MIN(accel + 1, MAX_ACCEL_MULT);
		else if (us_per_click > US_PER_CLICK_DEACCEL)
			accel = 1;

		cursor_hdg = normalize_hdg(cursor_hdg +
		    clicks * accel * WHEEL_ANGLE_MULT);
		last_wheel_t = now;
	}

	return (0);
}

static int
key_sniffer(char inChar, XPLMKeyFlags inFlags, char inVirtualKey, void *refcon)
{
	UNUSED(inChar);
	UNUSED(refcon);

	/* Only allow the plain key to be pressed, no modifiers */
	if (inFlags != xplm_DownFlag)
		return (1);

	switch ((unsigned char)inVirtualKey) {
	case XPLM_VK_RETURN:
	case XPLM_VK_ENTER:
	case XPLM_VK_NUMPAD_ENT:
		XPLMCommandOnce(XPLMFindCommand("BetterPushback/stop_planner"));
		return (0);
	case XPLM_VK_ESCAPE:
		bp_delete_all_segs();
		XPLMCommandOnce(XPLMFindCommand("BetterPushback/stop_planner"));
		return (0);
	case XPLM_VK_CLEAR:
	case XPLM_VK_BACK:
	case XPLM_VK_DELETE:
		/* Delete the segments up to the next user-placed segment */
		free(list_remove_tail(&bp.segs));
		for (seg_t *seg = list_tail(&bp.segs); seg != NULL &&
		    !seg->user_placed; seg = list_tail(&bp.segs)) {
			list_remove_tail(&bp.segs);
			free(seg);
		}
		return (0);
	case XPLM_VK_SPACE:
		bp_delete_all_segs();
		XPLMCommandOnce(XPLMFindCommand(
		    "BetterPushback/connect_first"));
		return (0);
	}

	return (1);
}

static void
find_drs(void)
{
	fdr_find(&drs.lat, "sim/flightmodel/position/latitude");
	fdr_find(&drs.lon, "sim/flightmodel/position/longitude");
	fdr_find(&drs.local_vx, "sim/flightmodel/position/local_vx");
	fdr_find(&drs.local_vy, "sim/flightmodel/position/local_vy");
	fdr_find(&drs.local_vz, "sim/flightmodel/position/local_vz");
	fdr_find(&drs.local_x, "sim/flightmodel/position/local_x");
	fdr_find(&drs.local_y, "sim/flightmodel/position/local_y");
	fdr_find(&drs.local_z, "sim/flightmodel/position/local_z");
	fdr_find(&drs.hdg, "sim/flightmodel/position/psi");
	fdr_find(&drs.tire_z, "sim/flightmodel/parts/tire_z_no_deflection");
	fdr_find(&drs.tire_x, "sim/flightmodel/parts/tire_x_no_deflection");
	fdr_find(&drs.tirrad, "sim/aircraft/parts/acf_gear_tirrad");

	fdr_find(&drs.mtow, "sim/aircraft/weight/acf_m_max");

	fdr_find(&drs.view_is_ext, "sim/graphics/view/view_is_external");
	/*
	 * These datarefs have been removed in XP12 with no replacements in
	 * sight. Disabling for now to get rid of errors/crashes with them.
	 */
	if (bp_xp_ver < 12000) {
		fdr_find(&drs.visibility, "sim/weather/visibility_reported_m");
		fdr_find(&drs.cloud_types[0], "sim/weather/cloud_type[0]");
		fdr_find(&drs.cloud_types[1], "sim/weather/cloud_type[1]");
		fdr_find(&drs.cloud_types[2], "sim/weather/cloud_type[2]");
		fdr_find(&drs.use_real_wx, "sim/weather/use_real_weather_bool");
	}

	fdr_find(&drs.cam_x, "sim/graphics/view/view_x");
	fdr_find(&drs.cam_y, "sim/graphics/view/view_y");
	fdr_find(&drs.cam_z, "sim/graphics/view/view_z");
	fdr_find(&drs.proj_matrix_3d, "sim/graphics/view/projection_matrix_3d");
	fdr_find(&drs.viewport, "sim/graphics/view/viewport");
}

bool_t
bp_cam_start(void)
{
	XPLMCreateWindow_t fake_win_ops = {
	    .structSize = sizeof (XPLMCreateWindow_t),
	    .left = 0, .top = 0, .right = 0, .bottom = 0, .visible = 1,
	    .drawWindowFunc = fake_win_draw,
	    .handleMouseClickFunc = fake_win_click,
	    .handleKeyFunc = nil_win_key,
	    .handleCursorFunc = fake_win_cursor,
	    .handleMouseWheelFunc = fake_win_wheel,
	    .refcon = NULL
	};
	char icao[8] = { 0 };
	char *cam_obj_path;
	char airline[1024] = { 0 };

	if (cam_inited || !bp_init())
		return (B_FALSE);

	find_drs();

	cam_obj_path = mkpathname(bp_xpdir, bp_plugindir, "objects",
	    "night_lamp.obj", NULL);
	cam_lamp_obj = XPLMLoadObject(cam_obj_path);
	if (cam_lamp_obj == NULL) {
		logMsg("Error loading pushback lamp %s. Please reinstall "
		    "BetterPushback.", cam_obj_path);
		free(cam_obj_path);
		return (B_FALSE);
	}
	free(cam_obj_path);

	if (!acf_is_compatible()) {
		XPLMSpeakString(_("Pushback failure: aircraft is incompatible "
		    "with BetterPushback."));
		return (B_FALSE);
	}

	(void) find_nearest_airport(icao);
	if (acf_is_airliner())
		read_acf_airline(airline);
	if (!tug_available(dr_getf(&drs.mtow), bp.acf.nw_len, bp.acf.tirrad,
	    bp.acf.nw_type, strcmp(icao, "") != 0 ? icao : NULL, airline)) {
		XPLMSpeakString(_("Pushback failure: no suitable tug for your "
		    "aircraft."));
		return (B_FALSE);
	}

#ifndef	PB_DEBUG_INTF
	if (vect3_abs(VECT3(dr_getf(&drs.local_vx), dr_getf(&drs.local_vy),
	    dr_getf(&drs.local_vz))) > 0.1) {
		XPLMSpeakString(_("Can't start planner: aircraft not "
		    "stationary."));
		return (B_FALSE);
	}
	if (bp_started && !late_plan_requested) {
		XPLMSpeakString(_("Can't start planner: pushback already in "
		    "progress. Please stop the pushback operation first."));
		return (B_FALSE);
	}
#endif	/* !PB_DEBUG_INTF */

	int left, top, right, bottom;
	XPLMGetScreenBoundsGlobal(&left, &top, &right, &bottom);
	fake_win_ops.right = right;
	fake_win_ops.top = top;

	circle_view_cmd = XPLMFindCommand("sim/view/circle");
	ASSERT(circle_view_cmd != NULL);
	XPLMCommandOnce(circle_view_cmd);

	fake_win = XPLMCreateWindowEx(&fake_win_ops);
	ASSERT(fake_win != NULL);
	XPLMBringWindowToFront(fake_win);
	XPLMTakeKeyboardFocus(fake_win);

	list_create(&pred_segs, sizeof (seg_t), offsetof(seg_t, node));
	force_root_win_focus = B_TRUE;
	cam_height = 15 * bp.veh.wheelbase;
	/* We keep the camera position in our coordinates for ease of manip */
	cam_pos = VECT3(dr_getf(&drs.local_x),
	    dr_getf(&drs.local_y), -dr_getf(&drs.local_z));
	cam_hdg = dr_getf(&drs.hdg);
	cursor_hdg = dr_getf(&drs.hdg);
	XPLMControlCamera(xplm_ControlCameraForever, cam_ctl, NULL);

	XPLMRegisterDrawCallback(draw_prediction, PREDICTION_DRAWING_PHASE,
	    PREDICTION_DRAWING_PHASE_BEFORE, NULL);
	cam_lamp_inst = XPLMCreateInstance(cam_lamp_obj, cam_lamp_drefs);

	for (int i = 0; view_cmds[i].name != NULL; i++) {
		view_cmds[i].cmd = XPLMFindCommand(view_cmds[i].name);
		VERIFY(view_cmds[i].cmd != NULL);
		XPLMRegisterCommandHandler(view_cmds[i].cmd, move_camera,
		    1, (void *)(uintptr_t)i);
	}
	XPLMRegisterKeySniffer(key_sniffer, 1, NULL);

	/* If the list of segs is empty, try to reload the saved state */
	if (list_head(&bp.segs) == NULL) {
		route_load(GEO_POS2(dr_getf(&drs.lat), dr_getf(&drs.lon)),
		    dr_getf(&drs.hdg), &bp.segs);
	}
	if (bp_xp_ver < 12000) {
		/*
		 * While the planner is active, we override the current
		 * visibility and real weather usage, so that the user can
		 * clearly see the path while planning. After we're done,
		 * we'll restore the settings.
		 */
		saved_visibility = dr_getf(&drs.visibility);
		saved_cloud_types[0] = dr_geti(&drs.cloud_types[0]);
		saved_cloud_types[1] = dr_geti(&drs.cloud_types[1]);
		saved_cloud_types[2] = dr_geti(&drs.cloud_types[2]);
		saved_real_wx = dr_geti(&drs.use_real_wx);

		dr_setf(&drs.visibility, BP_PLANNER_VISIBILITY);
		dr_seti(&drs.cloud_types[0], 0);
		dr_seti(&drs.cloud_types[1], 0);
		dr_seti(&drs.cloud_types[2], 0);
		dr_seti(&drs.use_real_wx, 0);
	}
	cam_inited = B_TRUE;

	return (B_TRUE);
}

bool_t
bp_cam_stop(void)
{
	seg_t *seg;
	XPLMCommandRef cockpit_view_cmd;

	if (!cam_inited)
		return (B_FALSE);

	if (cam_lamp_inst != NULL)
		XPLMDestroyInstance(cam_lamp_inst);
	cam_lamp_inst = NULL;
	if (cam_lamp_obj != NULL)
		XPLMUnloadObject(cam_lamp_obj);
	cam_lamp_obj = NULL;

	while ((seg = list_remove_head(&pred_segs)) != NULL)
		free(seg);
	list_destroy(&pred_segs);

	XPLMUnregisterDrawCallback(draw_prediction, PREDICTION_DRAWING_PHASE,
	    PREDICTION_DRAWING_PHASE_BEFORE, NULL);
	XPLMDestroyWindow(fake_win);

	for (int i = 0; view_cmds[i].name != NULL; i++) {
		XPLMUnregisterCommandHandler(view_cmds[i].cmd, move_camera,
		    1, (void *)(uintptr_t)i);
	}
	XPLMUnregisterKeySniffer(key_sniffer, 1, NULL);

	cockpit_view_cmd = XPLMFindCommand("sim/view/3d_cockpit_cmnd_look");
	ASSERT(cockpit_view_cmd != NULL);
	XPLMCommandOnce(cockpit_view_cmd);
	if (bp_xp_ver < 12000) {
		dr_setf(&drs.visibility, saved_visibility);
		dr_seti(&drs.cloud_types[0], saved_cloud_types[0]);
		dr_seti(&drs.cloud_types[1], saved_cloud_types[1]);
		dr_seti(&drs.cloud_types[2], saved_cloud_types[2]);
		dr_seti(&drs.use_real_wx, saved_real_wx);
	}
	cam_inited = B_FALSE;

	return (B_TRUE);
}

bool_t
bp_cam_is_running(void)
{
	return (cam_inited);
}
