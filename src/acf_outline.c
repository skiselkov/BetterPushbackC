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

#include <acfutils/assert.h>
#include <acfutils/acf_file.h>
#include <acfutils/perf.h>

#include "acf_outline.h"

#define	READ_PROP(x, func, ...) \
	do { \
		char path[64]; \
		const char *str; \
		VERIFY3S(snprintf(path, sizeof (path), __VA_ARGS__), <=, \
		    sizeof (path)); \
		str = acf_prop_find(acf, path); \
		if (str == NULL) \
			goto errout; \
		x = func(str); \
	} while (0)

#define	READ_INT(x, ...)	READ_PROP(x, atoi, __VA_ARGS__)
#define	READ_FLOAT(x, ...)	READ_PROP(x, atof, __VA_ARGS__)
#define	READ_FEET(x, offset, ...) \
	do { \
		READ_FLOAT(x, __VA_ARGS__); \
		x = FEET2MET(x) - offset; \
	} while (0)

bool_t
part_outline_read(const acf_file_t *acf, int part_nbr, vect2_t *pts, int s_dim,
    float z_ref)
{
	int r_dim;

	READ_INT(r_dim, "_part/%d/_r_dim", part_nbr);
	for (int s = 0; s < s_dim; s++) {
		vect2_t p = VECT2(-1e10, 0);

		for (int r = 0; r < r_dim; r++) {
			vect2_t p2;
			READ_FEET(p2.x, 0, "_part/%d/_geo_xyz/%d,%d,0",
			    part_nbr, s, r);
			READ_FEET(p2.y, z_ref, "_part/%d/_geo_xyz/%d,%d,2",
			    part_nbr, s, r);
			if (p2.x > p.x && (s > 0 || p2.y < p.y) &&
			    (s + 1 < s_dim || p2.y > p.y))
				p = p2;
		}
		pts[s] = p;
	}
	return (B_TRUE);
errout:
	return (B_FALSE);
}

bool_t
wing_outline_read(const acf_file_t *acf, int wing_nbr, vect2_t pts[5],
    double z_ref)
{
	vect2_t root, tip;
	double sweep, semilen, root_chord, tip_chord;

	READ_FLOAT(sweep, "_wing/%d/_sweep_design", wing_nbr);
	READ_FEET(semilen, 0, "_wing/%d/_semilen_SEG", wing_nbr);
	READ_FEET(root_chord, 0, "_wing/%d/_Croot", wing_nbr);
	READ_FEET(tip_chord, 0, "_wing/%d/_Ctip", wing_nbr);
	READ_FEET(root.x, 0, "_wing/%d/_crib_x_arm/0", wing_nbr);
	READ_FEET(root.y, z_ref, "_wing/%d/_crib_z_arm/0", wing_nbr);

	pts[0] = vect2_add(root, VECT2(0, -root_chord * 0.25));
	pts[1] = vect2_add(root, VECT2(0, root_chord * 0.75));
	tip = vect2_add(root, vect2_rot(VECT2(semilen, 0), -sweep));
	pts[2] = vect2_add(tip, VECT2(0, tip_chord * 0.75));
	pts[3] = vect2_add(tip, VECT2(0, -tip_chord * 0.25));
	pts[4] = pts[0];	/* close the loop */

	return (B_TRUE);
errout:
	return (B_FALSE);
}

acf_outline_t *
acf_outline_read(const char *filename, int nw_i, double nw_z_dr)
{
	acf_outline_t *outline = NULL;
	acf_file_t *acf = acf_file_read(filename);
	vect2_t *pts = NULL;
	int p, s_dim_fus;
	enum { N_WINGS = 5 };
	int wings[N_WINGS] = { 9, 11, 13, 15, 17 }; /* even nbr = left side */
	double nw_z, z_ref;

	if (acf == NULL)
		goto errout;

	outline = calloc(1, sizeof (*outline));

	READ_FEET(outline->semispan, 0, "acf/_size_x");
	READ_FEET(outline->length, 0, "acf/_size_z");

	READ_INT(s_dim_fus, "_part/56/_s_dim");
	READ_FEET(nw_z, 0, "_gear/%d/_gear_z", nw_i);
	z_ref = nw_z + (-nw_z_dr);
	printf("nw_z: %f   nw_z_dr: %f   z_ref: %f\n", nw_z, nw_z_dr, z_ref);

	outline->num_pts = s_dim_fus + N_WINGS * 6;
	outline->pts = calloc(outline->num_pts, sizeof (*pts));

	part_outline_read(acf, 56, outline->pts, s_dim_fus, z_ref);
	p = s_dim_fus;

	for (int i = 0; i < N_WINGS; i++) {
		outline->pts[p++] = NULL_VECT2;
		wing_outline_read(acf, wings[i], &outline->pts[p], z_ref);
		p += 5;
	}

	acf_file_free(acf);

	return (outline);

errout:
	if (outline != NULL)
		acf_outline_free(outline);
	if (acf != NULL)
		acf_file_free(acf);
	return (NULL);
}

void
acf_outline_free(acf_outline_t *outline)
{
	free(outline->pts);
	free(outline);
}
